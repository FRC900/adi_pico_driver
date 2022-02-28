#include <iostream>
#include "ros/ros.h"
#include "boost/asio/serial_port.hpp"

boost::asio::io_service port_io;
boost::asio::serial_port port(port_io, "/dev/ttyACM0");

void USB_WriteLine(std::string tx);
std::string USB_ReadLine();

int main() {
    boost::asio::serial_port_base::baud_rate baud_rate(115200);
    port.set_option(baud_rate);

    USB_WriteLine("echo 0\r");
    std::cout << USB_ReadLine() << std::endl;

    USB_WriteLine("uptime\r");
    std::cout << USB_ReadLine() << std::endl;

    port.close();

    return 0;
}

void USB_WriteLine(std::string tx) {
    port.write_some(boost::asio::buffer(tx, tx.size()));
}

std::string USB_ReadLine() {
    std::vector<char> rx;
    char c;
    while (true) {
        port.read_some(boost::asio::buffer(&c, 1));
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
