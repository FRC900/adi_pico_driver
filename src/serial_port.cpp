#include <boost/bind.hpp>
#include "adi_pico_driver/serial_port.h"

/**
 * @brief Constructor
 */
SerialPort::SerialPort()
  : port_io()
  , port(port_io)
  , wdg_timeout(10)
  , wdg(port_io)
  , status(PORT_STATUS::IDLE)
{
}

/**
 * @brief Open device
 * @param device Device file name (e.g. /dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int SerialPort::openPort(const std::string &device)
{
  if (isOpened())
  {
    std::fprintf(stderr, "[ADI SerialPort] Failed to open. Already opened.\r\n");
    return -1;
  }

  boost::system::error_code ec;
  ec.clear();
  port.open(device, ec);
  if (ec.value() != 0)
  {
    std::fprintf(stderr, "[ADI SerialPort] Failed to open. Error code : %d (%s)\r\n", ec.value(), ec.message().c_str());
    return -1;
  }
  ec.clear();
  port.set_option(boost::asio::serial_port_base::character_size(8), ec);
  if (ec.value() != 0)
  {
    std::fprintf(stderr, "[ADI SerialPort] Failed to set options. Error code : %d\r\n", ec.value());
    return -1;
  }

  return 0;
}

/**
 * @brief Close device
 */
void SerialPort::closePort()
{
  if (isOpened())
  {
    port.cancel();
    port.close();
  }
}

/**
 * @brief Query if port is open
 */
bool SerialPort::isOpened() const
{
  return port.is_open();
}

/**
 * @brief Flush serial port
 */
bool SerialPort::flushPort()
{
  port.cancel();
  status = PORT_STATUS::IDLE;
  return tcflush(port.lowest_layer().native_handle(), TCIOFLUSH) == 0 ? true : false;
}

/**
 *  @brief ASIO watchdog handler
 */
void SerialPort::wdg_handler(const boost::system::error_code& ec)
{
  if (!ec)
  {
    switch (status)
    {
    case PORT_STATUS::READ:
        std::fprintf(stderr, "[ADI Serial Port] SPI read timeout\r\n");
        break;

      case PORT_STATUS::WRITE:
        std::fprintf(stderr, "[ADI Serial Port] SPI write timeout\r\n");
        break;

    default:
        std::fprintf(stderr, "[ADI Serial Port] Unkown SPI state %d in timeout\r\n", status);
        break;
    }
  status = PORT_STATUS::TIME_OUT;
  }
  else if (ec == boost::asio::error::operation_aborted)
  {
    // Canceling the watchdog isn't a port error - it means
    // the read/write completed successfully
    return;
  }
  else
  {
    if (ec.value() != ECANCELED)
    {
      std::fprintf(stderr, "[ADI SerialPort] ??? wdg handle error. Code : %d\r\n", ec.value());
    }
  status = PORT_STATUS::SERIAL_ERROR;
  }
  port.cancel();
}

/**
 *  @brief ASIO serial port handler
 */
void SerialPort::serial_handler(const boost::system::error_code& ec)
{
  if (ec)
  {
    std::fprintf(stderr, "[ADI SerialPort] serial error : %s\r\n", ec.message().c_str());
  status = PORT_STATUS::SERIAL_ERROR;
  }
  wdg.cancel();
}

/**
 *  @brief write_bytes to the port
 */
bool SerialPort::write_bytes(const std::vector<uint8_t>& tx_bytes, const double timeout)
{
  if (status != SerialPort::PORT_STATUS::IDLE)
  {
    std::fprintf(stderr, "[ADI Serial Port] SPI write while status not IDLE\r\n");
    return false;
  }

  port_io.reset();
  status = SerialPort::PORT_STATUS::WRITE;
  wdg.expires_from_now(boost::posix_time::millisec(timeout * 1000));
  wdg.async_wait(boost::bind(&SerialPort::wdg_handler, this, boost::asio::placeholders::error));
  boost::asio::async_write(port, boost::asio::buffer(tx_bytes),
      boost::bind(&SerialPort::serial_handler, this, boost::asio::placeholders::error));

  port_io.run();

  if (status == SerialPort::PORT_STATUS::TIME_OUT)
  {
    std::fprintf(stderr, "[ADI Serial Port] SPI write timeout\r\n");
  status = SerialPort::PORT_STATUS::IDLE;
    return false;
  }
  if (status == SerialPort::PORT_STATUS::SERIAL_ERROR)
  {
    std::fprintf(stderr, "[ADI Serial Port] SPI write error\r\n");
  status = SerialPort::PORT_STATUS::IDLE;
    return false;
  }
  status = SerialPort::PORT_STATUS::IDLE;
  return true;
}

/**
 *  @brief read_bytes from the port
 */
bool SerialPort::read_bytes(std::vector<uint8_t>& rx_bytes, const double timeout)
{
  if (status != SerialPort::PORT_STATUS::IDLE)
  {
    std::fprintf(stderr, "[ADI SerialPort] SPI read while status not IDLE\r\n");
    return false;
  }

  port_io.reset();
  status = SerialPort::PORT_STATUS::READ;
  wdg.expires_from_now(boost::posix_time::millisec(timeout * 1000));
  wdg.async_wait(boost::bind(&SerialPort::wdg_handler, this, boost::asio::placeholders::error));
  boost::asio::async_read(port, boost::asio::buffer(rx_bytes),
      boost::bind(&SerialPort::serial_handler, this, boost::asio::placeholders::error));

  port_io.run();

  if (status == SerialPort::PORT_STATUS::TIME_OUT)
  {
    std::fprintf(stderr, "[ADI Serial Port] SPI read timeout\r\n");
  status = SerialPort::PORT_STATUS::IDLE;
    return false;
  }
  if (status == SerialPort::PORT_STATUS::SERIAL_ERROR)
  {
    std::fprintf(stderr, "[ADI Serial Port] SPI read error\r\n");
  status = SerialPort::PORT_STATUS::IDLE;
    return false;
  }
  status = SerialPort::PORT_STATUS::IDLE;
  return true;
}

