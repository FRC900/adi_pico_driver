// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "adi_pico_driver/adis16470.h"
#include "std_srvs/Trigger.h"

class ImuNode {
public:
  Adis16470 imu;
  ros::NodeHandle node_handle_;
  ros::Publisher imu_data_pub_;
  ros::Publisher temp_data_pub_;
  ros::ServiceServer bias_srv_;
  std::string device_;
  std::string frame_id_;
  bool burst_mode_;
  bool publish_temperature_;
  double rate_;
  int bias_conf_;
  int dec_rate_;
  int filt_;

  bool bias_estimate(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res) {
    ROS_INFO("bias_estimate");
    if (imu.bias_correction_update() < 0) {
      res.success = false;
      res.message = "Bias correction update failed";
      return false;
    }
    res.success = true;
    res.message = "Success";
    return true;
  }

  explicit ImuNode(ros::NodeHandle nh) : node_handle_(nh) {
    // Read parameters
    node_handle_.param("device", device_, std::string("/dev/ttyACM0"));
    node_handle_.param("frame_id", frame_id_, std::string("imu"));
    node_handle_.param("burst_mode", burst_mode_, false);
    node_handle_.param("publish_temperature", publish_temperature_, true);
    node_handle_.param("rate", rate_, 100.0);
    node_handle_.param("bias_conffig", bias_conf_, 0x0000);
    node_handle_.param("filter_cntrl", filt_, 7);
    node_handle_.param("decimation_rate", dec_rate_, static_cast<int>(2000 / rate_));

    ROS_INFO("device: %s", device_.c_str());
    ROS_INFO("frame_id: %s", frame_id_.c_str());
    ROS_INFO("rate: %f [Hz]", rate_);
    ROS_INFO("burst_mode: %s", (burst_mode_ ? "true" : "false"));
    ROS_INFO("publish_temperature: %s",
             (publish_temperature_ ? "true" : "false"));

    // Data publisher
    imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("data_raw", 100);
    if (publish_temperature_) {
      temp_data_pub_ =
          node_handle_.advertise<sensor_msgs::Temperature>("temperature", 100);
    }

    // Bias estimate service
    bias_srv_ = node_handle_.advertiseService("bias_estimate",
                                              &ImuNode::bias_estimate, this);
  }

  ~ImuNode() { }

  /**
   * @brief Open IMU device file
   */
  bool open(void) {
    // Open device file
    if (imu.open_port(device_) < 0) {
      ROS_ERROR("Failed to open device %s", device_.c_str());
      return false;
    }
    // Wait 10ms for SPI ready
    usleep(10000);
    int16_t pid = 0;
    imu.get_product_id(pid);
    ROS_INFO("Product ID: %x\n", pid);
    imu.set_bias_estimation_time(bias_conf_);
    imu.set_filt_ctrl(filt_);
    imu.set_dec_rate(dec_rate_);
    return true;
  }

  void publish_imu_data() {
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = imu.accl[0];
    data.linear_acceleration.y = imu.accl[1];
    data.linear_acceleration.z = imu.accl[2];

    // Angular velocity
    data.angular_velocity.x = imu.gyro[0];
    data.angular_velocity.y = imu.gyro[1];
    data.angular_velocity.z = imu.gyro[2];

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    imu_data_pub_.publish(data);
  }
  void publish_temp_data() {
    if (!publish_temperature_)
      return;
    sensor_msgs::Temperature data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // imu Temperature
    data.temperature = imu.temp;
    data.variance = 0;

    temp_data_pub_.publish(data);
  }
  bool spin() {
    ros::Rate loop_rate(rate_);

    while (ros::ok()) {
      if (burst_mode_) {
        if (imu.update_burst() == 0) {
          publish_imu_data();
          publish_temp_data();
        } else {
          ROS_ERROR("Cannot update burst");
        }
      } else if (imu.update() == 0) {
        publish_imu_data();
        publish_temp_data();
      } else {
        ROS_ERROR("Cannot update");
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh("~");
  ImuNode node(nh);

  if (!node.open()) {
    ROS_ERROR("Failed to Open an IMU");
    return 1;
  }
  node.spin();
  return (0);
}