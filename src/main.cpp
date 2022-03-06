// NOTE this requires access to /dev/ttyACM0, which may require adding the user to a group with permissions (e.g. dialout)

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <sys/ioctl.h>
#include "sensor_msgs/Imu.h"

#define BAUDRATE B115200
#define DEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define LINE_SIZE 81

/* Commands to initialize pico:
write 00 FD
write 02 02
write 00 FE
write 12 00
write 13 68
write 00 FF
stream 1
*/

const char inc[] = "inc\r";
const char *init[] = {
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


unsigned long long nsecs_offset;

inline int16_t uint16_to_int16(uint16_t num) {
    return *((int16_t*)&num);
}

void process_data(ros::Publisher &pub, char *buf) {
    std::array<uint16_t, 16> vals = {0};
    char *next_num = buf;
    for (int i = 0; i < 16; i++) {
        vals[i] = strtol(next_num, &next_num, 16);
    }

    // TODO: Validate checksums

    sensor_msgs::Imu msg;

    msg.header.stamp.sec = (vals[0] << 16) | vals[1] + nsecs_offset / 1000000000ULL;
    // Microseconds to nanoseconds
    msg.header.stamp.nsec = ((vals[1] << 16) | vals[2]) * 1000 + nsecs_offset % 1000000000ULL;
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

int main(int argc, char **argv) {
    int fd;
    struct termios newtio;

    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror(DEVICE);
        exit(-1);
    }

    cfsetospeed (&newtio, BAUDRATE);
    cfsetispeed (&newtio, BAUDRATE);
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = ICANON;

    /* inter-character timer unused */
    newtio.c_cc[VTIME] = 0;

    /* read(2) blocks until MIN bytes are available,
     * and returns up to the number of bytes requested. */
    newtio.c_cc[VMIN] = LINE_SIZE;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    /* Write all init commands */
    for (const char *s : init) {
        write(fd, s, strlen(s));
    }

    /* Write all output, clear input */
    tcdrain(fd);
    tcflush(fd, TCIFLUSH);

    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);

    char buf[LINE_SIZE + 1];
    buf[LINE_SIZE] = '\0';
    time_t seconds = 0;
    nsecs_offset = ros::Time::now().toNSec(); // May need to reset offset each `inc`, but also need to store the previous offset until the data referring to that second has been processed
    while (ros::ok()) {
        int bytes = 0;
        while (bytes < LINE_SIZE) {
            /* read() isn't guaranteed to read MIN bytes,
             * keep calling it until it reads all of them */
            bytes += read(fd, buf + bytes, LINE_SIZE - bytes);
        }

        if (buf[LINE_SIZE - 1] != '\n') {
            printf("ERROR: line not ending with \\n, skipping\n");

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
            printf("ERROR: serial port receive buffer overflow\n");
        }

        process_data(pub, buf);

        time_t s = time(NULL);
        if (s > seconds) {
            seconds = s;

            write(fd, inc, sizeof(inc) - 1);
            tcdrain(fd);
        }
    }
}
