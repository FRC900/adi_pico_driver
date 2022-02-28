#pragma once

#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>

class SerialPort
{
	public:
		SerialPort();
		int openPort(const std::string &device);
		void closePort();
		bool isOpened() const;
		bool flushPort();
		bool write_bytes(const std::vector<uint8_t> &, const double timeout = 1.0);
		bool read_bytes(std::vector<uint8_t> &, const double timeout = 1.0);
	private:
		boost::asio::io_service port_io;
		boost::asio::serial_port port;
		const double wdg_timeout;
		boost::asio::deadline_timer wdg;
		typedef enum
		{
			IDLE,
			WRITE,
			READ,
			TRANSFER,
			TIME_OUT,
			SERIAL_ERROR
		} PORT_STATUS;
		PORT_STATUS status;
		void wdg_handler(const boost::system::error_code &);
		void serial_handler(const boost::system::error_code &);
};

