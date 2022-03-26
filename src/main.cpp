#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define BAUDRATE B115200
#define DEVICE "/dev/PICO"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define LINE_SIZE 81
#define LINE_ITEMS 16
#define PICO_CHECK 4
#define DIAG_STAT 6
#define IMU_CHECK 15
#define CONSECUTIVE_ERRS 5

const char init[] =
    "write 00 FD\r"
    "write 02 02\r"
    "write 00 FE\r"
    "write 12 00\r"
    "write 13 68\r"
    "write 00 FF\r"
    "inc\r"
    "stream 1\r"
    "echo 0\r";
const char init_res[] =
    "write 00 FD\r\n"
    "write 02 02\r\n"
    "write 00 FE\r\n"
    "write 12 00\r\n"
    "write 13 68\r\n"
    "write 00 FF\r\n"
    "inc\r\n"
    "stream 1\r\n"
    "echo 0\r\n";
const char reset_imu[] = "cmd 4000\r";
const char reset_pico[] = "\rcmd 4\r";

ros::Publisher pub;
ros::Time reset;
int fd;
int errors = 0;

std::array<uint16_t, LINE_ITEMS> cur_vals;
char cur_line[LINE_SIZE + 1] = {0};

void init_tty() {
    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR("Failed to open %s: %s", DEVICE, strerror(errno));
        exit(1);
    }

    struct termios tty;
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

    if(write(fd, reset_pico, sizeof(reset_pico) - 1) == -1) {
        ROS_ERROR("Failed to reset pico during initialization: %s", strerror(errno));
    }
    tcdrain(fd);
    sleep(3);
    tcflush(fd, TCIFLUSH);

    /* Write all init commands */
    if(write(fd, init, sizeof(init) - 1) == -1) {
        ROS_ERROR("Failed to write initialization sequence: %s", strerror(errno));
        exit(1);
    }
    tcdrain(fd);
    reset = ros::Time::now();
    sleep(1);

    /* Read response  */
    size_t res_size = sizeof(init_res) - 1;
    char recieved[sizeof(init_res)];

    size_t bytes = 0;
    while (bytes < res_size) {
        /* read() isn't guaranteed to read MIN bytes,
         * keep calling it until it reads all of them */
        bytes += read(fd, recieved + bytes, res_size - bytes);
    }
    recieved[res_size] = '\0';

    if (strcmp(recieved, init_res)) {
        ROS_ERROR("Pico response mismatch: expected: \n%srecieved: \n%s", init_res, recieved);
        exit(1);
    }
    ROS_INFO("\n%s", recieved);

    tcflush(fd, TCIFLUSH);
}

void error_handler() {
    errors++;
    if (errors >= CONSECUTIVE_ERRS) {
        close(fd);
        init_tty();
    }
}

bool read_line() {
    int length;
    ioctl(fd, FIONREAD, &length);

    /* The recieve buffer holds at most 4095 bytes */
    if (length == 4095) {
        ROS_ERROR("Serial port recieve buffer overflow");
        return false;
    }
    if (length >= 4000) {
        ROS_WARN("Serial port receive buffer holding %d out of 4095 bytes", length);
    }

    int bytes = 0;
    while (bytes < LINE_SIZE) {
        /* read() isn't guaranteed to read MIN bytes,
         * keep calling it until it reads all of them */
        bytes += read(fd, cur_line + bytes, LINE_SIZE - bytes);
    }

    if (cur_line[LINE_SIZE - 1] != '\n') {
        ROS_WARN("Line not ending with \\n");
        return false;
    }
    return true;
}

bool parse_line() {
    char *next_num = cur_line;
    for (int i = 0; i < LINE_ITEMS; i++) {
        char c;

        cur_vals[i] = 0;
        for(int j = 0; j < 4; j++)
        {
            c = next_num[j];
            if(c >= '0' && c <= '9') {
                c = c - '0';
            } else if (c >= 'A' && c <='F') {
                c = c - 'A' + 10;
            } else {
                ROS_ERROR("Failed to parse line\n%s", cur_line);
                return false;
            }
            cur_vals[i] = cur_vals[i] * 16 + c;
        }
        next_num += 5;
    }

    uint32_t pico_check = 0;
    for (int i = 0; i < LINE_ITEMS; i++) {
        pico_check += cur_vals[i];
    }
    /* Don't add the checksum to itself */
    pico_check -= cur_vals[PICO_CHECK];

    if ((uint16_t) pico_check != cur_vals[PICO_CHECK]) {
        ROS_WARN("Pico checksum mismatch: calculated: %d recieved: %d\n%s",
               pico_check, cur_vals[PICO_CHECK], cur_line);
        return false;
    }

    int imu_check = 0;
    /* After line signature is unused field, then start of IMU data */
    for (int i = PICO_CHECK + 2; i < IMU_CHECK; i++) {
        imu_check += cur_vals[i] & 0xFF;
        imu_check += cur_vals[i] >> 8;
    }

    if (imu_check != cur_vals[IMU_CHECK]) {
        ROS_WARN("IMU checksum mismatch: calculated: %d recieved: %d\n%s",
                 imu_check, cur_vals[IMU_CHECK], cur_line);
        return false;
    }

    if (cur_vals[DIAG_STAT] & (1u << 1)) {
        ROS_WARN("IMU DIAG_STAT: data path overrun, resetting");
        if(write(fd, reset_imu, sizeof(reset_imu) - 1) == -1) {
            ROS_ERROR("Failed to reset IMU after data path overrun");
        }
    } else if (cur_vals[DIAG_STAT]) {
        ROS_WARN("IMU DIAG_STAT: 0x%02x", cur_vals[DIAG_STAT]);
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    pub = nh.advertise<sensor_msgs::Imu>("data_raw", 10);

    init_tty();

    while (ros::ok()) {
        if (read_line()) {
            errors = 0;
        } else {
            error_handler();

            /* Skip to end of line */
            char c;
            do {
                read(fd, &c, 1);
            } while (c != '\n');

            continue;
        }

        if (parse_line()) {
            errors = 0;
        } else {
            error_handler();
            continue;
        }

        uint32_t us = cur_vals[2] | (cur_vals[3] << 16);

        /* Divide into seconds to avoid overflow when turning us to ns */
        uint32_t s = us / 1000000;
        us = us % 1000000;

        sensor_msgs::Imu msg;

        // Microseconds to nanoseconds
        msg.header.stamp = reset + ros::Duration(s, us * 1000);

        msg.header.frame_id = "imu";

        // Angular velocity: {X,Y,Z}_GYRO_OUT
        msg.angular_velocity.x = static_cast<int16_t>(cur_vals[7]) * M_PI / 180 * 0.1;
        msg.angular_velocity.y = static_cast<int16_t>(cur_vals[8]) * M_PI / 180 * 0.1;
        msg.angular_velocity.z = static_cast<int16_t>(cur_vals[9]) * M_PI / 180 * 0.1;

        // Linear acceleration: {X,Y,Z}_ACCL_OUT
        msg.linear_acceleration.x = static_cast<int16_t>(cur_vals[10]) * 1.25 * 9.80665 / 1000.;
        msg.linear_acceleration.y = static_cast<int16_t>(cur_vals[11]) * 1.25 * 9.80665 / 1000.;
        msg.linear_acceleration.z = static_cast<int16_t>(cur_vals[12]) * 1.25 * 9.80665 / 1000.;

        // Orientation (not provided)
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 1;

        pub.publish(msg);
    }
}
