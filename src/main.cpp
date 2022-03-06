// NOTE this requires access to /dev/ttyACM0, which may require adding the user to a group with permissions (e.g. dialout)

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

const char inc[] = "inc\r";
const char *init[] = {
    "\r",
    "cmd 4\r",
    "echo 0\r",
    "write 00 FD\r",
    "write 02 02\r",
    "write 00 FE\r",
    "write 12 00\r",
    "write 13 68\r",
    "write 00 FF\r",
    "stream 1\r"
};

ros::Publisher pub;

/* Time of incrementing PPS to (i - seconds_index) */
std::queue<ros::Time> inc_timestamps;
/* The earliest second PPS was incremented to */
int seconds_index = 0;

void process_data(char *buf) {
    std::array<uint16_t, 16> vals = {0};
    char *next_num = buf;
    for (int i = 0; i < 16; i++) {
        vals[i] = strtol(next_num, &next_num, 16);
    }

    // TODO: Validate checksums

    int seconds = vals[0] | (vals[1] << 16);
    int us = vals[2] | (vals[3] << 16);

    /* Remove timestamp for previous second once we recieve lines from the next second */
    if (seconds > seconds_index) {
        inc_timestamps.pop();
        seconds_index++;
    }
    ros::Time timestamp = inc_timestamps.front() + ros::Duration(0, us * 1000);

    sensor_msgs::Imu msg;

    // Microseconds to nanoseconds
    msg.header.stamp = timestamp;

    // DATA_CNTR
    // msg.header.seq = vals[14];
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

int main(int argc, char **argv) {
    int fd;
    struct termios tty;

    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR("%s: %s", DEVICE, strerror(errno));
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
    for (const char *s : init) {
        if(write(fd, s, strlen(s)) == -1) {
            ROS_ERROR("failed to write \"%s\" during initialization: %s", s, strerror(errno));
        }
    }

    /* Write all output, clear input */
    tcdrain(fd);
    tcflush(fd, TCIFLUSH);

    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");
    pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);

    char buf[LINE_SIZE + 1];
    buf[LINE_SIZE] = '\0';
    time_t seconds = 0;
    inc_timestamps.push(ros::Time::now());
    while (ros::ok()) {
        int bytes = 0;
        while (bytes < LINE_SIZE) {
            /* read() isn't guaranteed to read MIN bytes,
             * keep calling it until it reads all of them */
            bytes += read(fd, buf + bytes, LINE_SIZE - bytes);
        }

        if (buf[LINE_SIZE - 1] != '\n') {
            ROS_WARN("Line not ending with \\n, skipping");

            /* Discard characters until \n */
            char c;
            do {
                read(fd, &c, 1);
            } while (c != '\n');

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
        }

        process_data(buf);

        time_t s = time(NULL);
        if (s > seconds) {
            seconds = s;

            if(write(fd, inc, sizeof(inc) - 1) == -1) {
                ROS_ERROR("failed to increment PPS: %s", strerror(errno));
            }

            tcdrain(fd);

            inc_timestamps.push(ros::Time::now());
        }
    }
}
