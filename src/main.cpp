#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <queue>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define BAUDRATE B115200
#define DEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define LINE_SIZE 81
#define LINE_ITEMS 16
#define SIGNATURE_INDEX 4
#define CHECKSUM_INDEX 15

const char init[] =
    "\r"
    "cmd 4\r"
    "echo 0\r"
    "write 00 FD\r"
    "write 02 02\r"
    "write 00 FE\r"
    "write 12 00\r"
    "write 13 68\r"
    "write 00 FF\r"
    "inc\r"
    "stream 1\r";

ros::Publisher pub;
ros::Time reset;
int fd;

void process_data(char *buf) {
    std::array<uint16_t, LINE_ITEMS> vals = {0};
    char *next_num = buf;
    for (int i = 0; i < 16; i++) {
        vals[i] = strtol(next_num, &next_num, 16);
    }

    uint32_t line_signature = 0;
    for (int i = 0; i < LINE_ITEMS; i++) {
        line_signature += vals[i];
    }
    /* Don't add the checksum to itself */
    line_signature -= vals[SIGNATURE_INDEX];

    /* Check the Pico's checksum */
    if ((uint16_t) line_signature != vals[SIGNATURE_INDEX]) {
        printf("Pico signature mismatch: calculated: %d recieved: %d\n",
               line_signature, vals[SIGNATURE_INDEX]);
        puts(buf);
        return;
    }

    /* Check the IMU's checksum */
    int imu_checksum = 0;
    /* After line signature is unused field, then start of IMU data */
    for (int i = SIGNATURE_INDEX + 2; i < CHECKSUM_INDEX; i++) {
        imu_checksum += vals[i] & 0xFF;
        imu_checksum += vals[i] >> 8;
    }

    /* Check the Pico's checksum */
    if (imu_checksum != vals[CHECKSUM_INDEX]) {
        printf("IMU signature mismatch: calculated: %d recieved: %d\n",
               imu_checksum, vals[CHECKSUM_INDEX]);
        puts(buf);
        return;
    }

    uint32_t us = vals[2] | (vals[3] << 16);

    /* Divide into seconds to avoid overflow when turning us to ns */
    uint32_t s = us / 1000000;
    us = us % 1000000;

    sensor_msgs::Imu msg;

    // Microseconds to nanoseconds
    msg.header.stamp = reset + ros::Duration(s, us * 1000);

    msg.header.frame_id = "imu";

    // Angular velocity: {X,Y,Z}_GYRO_OUT
    msg.angular_velocity.x = static_cast<int16_t>(vals[7]) * M_PI / 180 * 0.1;
    msg.angular_velocity.y = static_cast<int16_t>(vals[8]) * M_PI / 180 * 0.1;
    msg.angular_velocity.z = static_cast<int16_t>(vals[9]) * M_PI / 180 * 0.1;

    // Linear acceleration: {X,Y,Z}_ACCL_OUT
    msg.linear_acceleration.x = static_cast<int16_t>(vals[10]) * 1.25 * 9.80665 / 1000.;
    msg.linear_acceleration.y = static_cast<int16_t>(vals[11]) * 1.25 * 9.80665 / 1000.;
    msg.linear_acceleration.z = static_cast<int16_t>(vals[12]) * 1.25 * 9.80665 / 1000.;

    // Orientation (not provided)
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 1;

    pub.publish(msg);
}

void clear_line() {
    char c;
    do {
        read(fd, &c, 1);
    } while (c != '\n');
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    struct termios tty;

    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR("Failed to open %s: %s", DEVICE, strerror(errno));
        return 1;
    }

    cfsetospeed (&tty, BAUDRATE);
    cfsetispeed (&tty, BAUDRATE);
    tty.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    tty.c_lflag = ICANON;

    /* inter-character timer unused */
    tty.c_cc[VTIME] = 0;

    /* read(2) blocks until MIN bytes are available,
     * and returns up to the number of bytes requested. */
    tty.c_cc[VMIN] = LINE_SIZE;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    /* Write all init commands */
    if(write(fd, init, sizeof(init) - 1) == -1) {
        ROS_ERROR("Failed to write initialization sequence: %s", strerror(errno));
        return 1;
    }

    tcdrain(fd);
    reset = ros::Time::now();
    tcflush(fd, TCIFLUSH);

    pub = nh.advertise<sensor_msgs::Imu>("data_raw", 10);

    /* Null terminator for printing */
    char buf[LINE_SIZE+1];
    buf[LINE_SIZE] = '\0';
    while (ros::ok()) {
        int bytes = 0;
        while (bytes < LINE_SIZE) {
            /* read() isn't guaranteed to read MIN bytes,
             * keep calling it until it reads all of them */
            bytes += read(fd, buf + bytes, LINE_SIZE - bytes);
        }

        if (buf[LINE_SIZE - 1] != '\n') {
            ROS_WARN("Line not ending with \\n, skipping");
            clear_line();
            continue;
        }

        int length;
        ioctl(fd, FIONREAD, &length);

        /* The recieve buffer holds at most 4095 bytes */
        if (length >= 4000) {
            ROS_WARN("Serial port receive buffer holding %d out of 4095 bytes", length);
        }
        if (length == 4095) {
            ROS_ERROR("Serial port recieve buffer overflow");
            clear_line();
            continue;
        }

        process_data(buf);
    }
}
