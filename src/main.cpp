#include <iostream>
#include "ros/ros.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/read.hpp"

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

int main() {
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
        std::string item  = USB_ReadStream();
        std::cout << item << std::endl;
    }

    port.close();

    return 0;
}
