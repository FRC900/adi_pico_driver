#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"
#include "sensor_msgs/Imu.h"
#include <queue>
#include <mutex>
#include <thread>
#include <array>

#define MEMORY_WARNING_BYTES 100000

template <typename T> class ThreadSafeQueue {
public:
  void push(const T& item) {
    mtx_.lock();
    queue_.push(item);
    mtx_.unlock();
  }
  T pop() {
    T item;
    while (queue_.size() == 0) {
      // block, otherwise this will deadlock (I think)
      ROS_INFO_STREAM_THROTTLE(0.5, "size is 0");
    }
    item = queue_.front();
    mtx_.lock();
    queue_.pop();
    mtx_.unlock();
    return item;
  }
  size_t size() {
    return queue_.size();
  }
private:
  std::mutex mtx_;
  std::queue<T> queue_;
};

#define DATA_BLOCK_SIZE 50
// This is multiples of 81, 81*50 = 4050

using DataBlock = std::array<char, 81*DATA_BLOCK_SIZE>;

ThreadSafeQueue<DataBlock> data;

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

// from https://stackoverflow.com/questions/28253352/how-to-determine-if-there-are-bytes-available-to-be-read-from-boostasioserial
/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.
std::size_t get_bytes_available(
  boost::asio::serial_port& serial_port,
  boost::system::error_code& error)
{
  error = boost::system::error_code();
  int value = 0;
#if defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
  COMSTAT status;
  if (0 != ::ClearCommError(serial_port.lowest_layer().native_handle(),
                            NULL, &status))
  {
    value = status.cbInQue;
  }
  // On error, set the error code.
  else
  {
    error = boost::system::error_code(::GetLastError(),
       boost::asio::error::get_system_category());
  }
#else // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
  if (0 == ::ioctl(serial_port.lowest_layer().native_handle(),
                   FIONREAD, &value))
  {
    error = boost::system::error_code(errno,
       boost::asio::error::get_system_category());
  }
#endif // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)

  return error ? static_cast<std::size_t>(0)
               : static_cast<size_t>(value);

}

/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.  Throws on error.
std::size_t get_bytes_available(boost::asio::serial_port& serial_port)
{
  boost::system::error_code error;
  std::size_t bytes_available = get_bytes_available(serial_port, error);
  if (error)
  {
    boost::throw_exception((boost::system::system_error(error)));
  }
  return bytes_available;
}

DataBlock USB_ReadStream() {
    if (get_bytes_available(port) == 0) {
      ROS_INFO_STREAM("reading data faster than pico!");
    } else if (get_bytes_available(port) > 4094) {
      ROS_WARN_STREAM("adi_pico_driver : serial port buffer overflow! likely losing data!");
    } else if (get_bytes_available(port) > 2000) {
      ROS_INFO_STREAM("big nope: " << get_bytes_available(port));
    }
    DataBlock rx;
    boost::asio::read(port, boost::asio::buffer(rx));
    return rx;
}

inline int16_t uint16_to_int16(uint16_t num) {
    return *((int16_t*)&num);
}

void process_data_loop(ros::NodeHandle &nh) {
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);
  while (ros::ok()) {
    if (data.size() * 81 * DATA_BLOCK_SIZE > MEMORY_WARNING_BYTES) {
      ROS_WARN_STREAM_THROTTLE(0.5, "adi_pico_driver : large memory usage in queue!");
    }
    ROS_INFO_STREAM_THROTTLE(1, "adi_pico_driver : processing data");
    auto item = data.pop();
    for (uint32_t i = 0; i < 81*DATA_BLOCK_SIZE; i+=81) {
      std::string s(item.begin()+i, item.begin()+i+81);
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
  }
}

void read_data_loop() {
  while (ros::ok()) {
    data.push(USB_ReadStream());
  }
}

// This is from Stack Overflow: https://stackoverflow.com/questions/22581315/how-to-discard-data-as-it-is-sent-with-boostasio/22598329#22598329
/// @brief Different ways a serial port may be flushed.
enum flush_type
{
  flush_receive = TCIFLUSH,
  flush_send = TCOFLUSH,
  flush_both = TCIOFLUSH
};

/// @brief Flush a serial port's buffers.
///
/// @param serial_port Port to flush.
/// @param what Determines the buffers to flush.
/// @param error Set to indicate what error occurred, if any.
void flush_serial_port(
  boost::asio::serial_port& serial_port,
  flush_type what,
  boost::system::error_code& error)
{
  if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what))
  {
    error = boost::system::error_code();
  }
  else
  {
    error = boost::system::error_code(errno,
        boost::asio::error::get_system_category());
  }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    boost::asio::serial_port_base::baud_rate baud_rate(115200);
    port.set_option(baud_rate);

    // Reboot Pico
    USB_WriteLine("cmd 4\r");

    boost::system::error_code error;
    flush_serial_port(port, flush_receive, error);

    ros::Duration(2.5).sleep();

    std::string cmd_res = USB_ReadLine();
    if (cmd_res != "cmd 4") {
        ROS_ERROR_STREAM("adi_pico_driver : Unexpected response to command: " << cmd_res);
    }

    // Wait for Pico to start again
    ros::Duration(2.5).sleep();

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

    std::thread t(process_data_loop, std::ref(nh));
    std::thread t2(read_data_loop); // more important to run fast

    t2.join();

    port.close();

    return 0;
}
