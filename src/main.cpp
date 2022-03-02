#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"
#include "sensor_msgs/Imu.h"

boost::asio::io_service port_io;
boost::asio::serial_port port(port_io, "/dev/ttyACM0");

void USB_WriteLine(std::string tx) {
    boost::asio::write(port, boost::asio::buffer(tx, tx.size()));
}

std::string USB_ReadLine() {
    std::vector<char> rx;
    char c;
    while (true) {
        boost::asio::read(port, boost::asio::buffer(&c, 1));
        switch (c) {
            case '\r':
                break;
            case '\n':
                return std::string(rx.begin(), rx.end());
            default:
                rx.push_back(c);
        }
    }
}

std::string USB_ReadStream() {
    std::array<char, 81> rx;
    boost::asio::read(port, boost::asio::buffer(rx));
    return std::string(rx.begin(), rx.end());
}

inline int16_t uint16_to_int16(uint16_t num) {
    return *((int16_t*)&num);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);

    boost::asio::serial_port_base::baud_rate baud_rate(115200);
    port.set_option(baud_rate);

    // Reboot Pico
    USB_WriteLine("cmd 4\r");
    std::string cmd_res = USB_ReadLine();
    if (cmd_res != "cmd 4") {
        ROS_ERROR_STREAM("adi_pico_driver : Unexpected response to command: " << cmd_res);
    }

    // Wait for Pico to start again
    ros::Duration(5.0).sleep();

    USB_WriteLine("echo 0\r");
    std::string echo_res = USB_ReadLine();
    if (echo_res != "echo 0") {
        ROS_ERROR_STREAM("adi_pico_driver : Unexpected response to command: " << echo_res);
    }

    USB_WriteLine("write 00 00\r");
    USB_WriteLine("read 72\r");
    ROS_INFO_STREAM("IMU PROD_ID: " << USB_ReadLine());
    USB_WriteLine("read 74\r");
    ROS_INFO_STREAM("IMU SERIAL_NUM: " << USB_ReadLine());

    USB_WriteLine("write 00 FD\r");
    USB_WriteLine("write 02 02\r");
    USB_WriteLine("write 00 FE\r");
    USB_WriteLine("write 12 00\r");
    USB_WriteLine("write 13 68\r");
    USB_WriteLine("write 00 FF\r");

    USB_WriteLine("stream 1\r");

    while (true) {
        std::string s = USB_ReadStream();
        std::array<uint16_t, 16> vals = {0};
        int j = 0;
        for (int i = 0; i < 16; i++) {
            vals[i] = std::stoul(s.substr(j, 4), nullptr, 16);
            j += 5;
        }

        // TODO: Validate checksums

        sensor_msgs::Imu msg;

        // msg.header.stamp.sec = (vals[0] << 16) | vals[1];
        // Microseconds to nanoseconds
        // msg.header.stamp.nsec = ((vals[1] << 16) | vals[2]) * 1000;
        msg.header.stamp = ros::Time::now();

        // DATA_CNTR
        // msg.header.seq = vals[14];
        msg.header.frame_id = "imu";

        // Angular velocity: {X,Y,Z}_GYRO_OUT
        msg.angular_velocity.x = uint16_to_int16(vals[7]) * M_PI / 180 * 0.1;
        msg.angular_velocity.y = uint16_to_int16(vals[8]) * M_PI / 180 * 0.1;
        msg.angular_velocity.z = uint16_to_int16(vals[9]) * M_PI / 180 * 0.1;

        // Linear acceleration: {X,Y,Z}_ACCL_OUT
        msg.linear_acceleration.x = uint16_to_int16(vals[10]) * 1.25 * 9.80665 / 1000.;
        msg.linear_acceleration.y = uint16_to_int16(vals[11]) * 1.25 * 9.80665 / 1000.;
        msg.linear_acceleration.z = uint16_to_int16(vals[12]) * 1.25 * 9.80665 / 1000.;

        // Orientation (not provided)
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 1;

        pub.publish(msg);
    }

    port.close();

    return 0;
}
