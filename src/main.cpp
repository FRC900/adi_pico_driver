#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/read.hpp"
#include "sensor_msgs/Imu.h"

boost::asio::io_service port_io;
boost::asio::serial_port port(port_io, "/dev/ttyACM0");

void USB_WriteLine(std::string tx) {
    port.write_some(boost::asio::buffer(tx, tx.size()));
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);

    boost::asio::serial_port_base::baud_rate baud_rate(115200);
    port.set_option(baud_rate);

    USB_WriteLine("echo 0\r");
    std::string res = USB_ReadLine();
    if (res != "echo 0") {
        ROS_ERROR_STREAM("Unexpected response to command: " << res);
    }

    USB_WriteLine("write 00 00\r");
    USB_WriteLine("read 72\r");
    ROS_INFO_STREAM("IMU PROD_ID: " << USB_ReadLine());

    USB_WriteLine("write 00 FD\r");
    USB_WriteLine("write 02 02\r");
    USB_WriteLine("write 00 FE\r");
    USB_WriteLine("write 12 00\r");
    USB_WriteLine("write 13 68\r");
    USB_WriteLine("write 00 FF\r");
    USB_WriteLine("stream 1\r");

    while (true) {
        std::string s = USB_ReadStream();
        std::cout << s << "\n\n";
        if (s[s.size() - 1] != '\n') {
            ROS_ERROR_STREAM("Misaligned stream item: " << s);
        }

        std::array<uint16_t, 16> vals = {0};
        int j = 0;
        for (int i = 0; i < 16; i++) {
            vals[i] = std::stoul(s.substr(j, 4), nullptr, 16);
            j += 5;
        }

        sensor_msgs::Imu msg;

        // msg.header.stamp.sec = (vals[0] << 16) | vals[1];
        // Microseconds to nanoseconds
        // msg.header.stamp.nsec = ((vals[1] << 16) | vals[2]) * 1000;
        msg.header.stamp = ros::Time::now();

        // DATA_CNTR
        // msg.header.seq = vals[14];
        msg.header.frame_id = "imu";

        // Linear acceleration: {X,Y,Z}_GYRO_OUT
        msg.linear_acceleration.x = static_cast<int16_t>(vals[7]) * M_PI / 180 * 0.1;
        msg.linear_acceleration.y = static_cast<int16_t>(vals[8]) * M_PI / 180 * 0.1;
        msg.linear_acceleration.z = static_cast<int16_t>(vals[9]) * M_PI / 180 * 0.1;

        // Angular velocity: {X,Y,Z}_ACCL_OUT
        msg.angular_velocity.x = static_cast<int16_t>(vals[10]) * 1.25 * 9.80665 / 1000.;
        msg.angular_velocity.y = static_cast<int16_t>(vals[11]) * 1.25 * 9.80665 / 1000.;
        msg.angular_velocity.z = static_cast<int16_t>(vals[12]) * 1.25 * 9.80665 / 1000.;

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
