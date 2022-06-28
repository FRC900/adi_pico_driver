#include "ros/ros.h"
#include <queue>
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
#include <errno.h>

#define BUF_SIZ 64
#define BAUDRATE B115200

const char init_cmd[] =
    /* Clear pico command buffer of any unfinished commands */
    "\r"
    /* Reset pico configuration, putting in into a clean state */
    "cmd 4\r"
    /* Enable burst data capture */
    "write 00 FD\r"
    "write 02 02\r"
    /* Configure burst read signal */
    "write 00 FE\r"
    "write 12 00\r"
    "write 13 68\r"
    /* Move to output page */
    "write 00 FF\r"
    /* Reset timer */
    "inc\r"
    /* Print buffer contents */
    "stream 1\r";
const char init_res[] = "write 00 FD\r\n"
                        "write 02 02\r\n"
                        "write 00 FE\r\n"
                        "write 12 00\r\n"
                        "write 13 68\r\n"
                        "write 00 FF\r\n"
                        "inc\r\n"
                        "stream 1\r\n"
                        "echo 0\r\n";

class pico {
    char buf[BUF_SIZ];
    size_t i = BUF_SIZ;
    int fd;
    char *dev;
public:
    pico(char *dev);
    char next_char();
    void write_tty(const char *s);
    void read_tty(char *buf, size_t count);
    void scan_to_string(const char *s, int timeout);
    /* ros::Time */
};

char pico::next_char() {
    if (i < BUF_SIZ) {
        return buf[i++];
    }

    /* Refill buffer and reset index to beginning */
    int length;
    ioctl(fd, FIONREAD, &length);

    /* The recieve buffer holds at most 4095 bytes */
    if (length >= 4095) {
        ROS_ERROR("Serial port recieve buffer overflow");
    } else if (length >= 4000) {
        ROS_WARN("Serial port receive buffer holding %d out of 4095 bytes",
                    length);
    }

    int count = BUF_SIZ;
    char *ptr = buf;
    while (count) {
        /* read() isn't guaranteed to read MIN bytes,
         * keep calling it until it reads all of them */
        ssize_t bytes = read(fd, ptr, count);
        if (!bytes) {
            ROS_ERROR("Timeout on reading TTY; opening again");
            // TODO: TTY error or would block
        }
        count -= bytes;
        ptr += bytes;
    }
    i = 0;

    return buf[i++];
}

/* Writes string to pico, dropping the null terminator */
void pico::write_tty(const char *s) {
    ssize_t count = strlen(s);
    if(write(fd, s, count) != count) {
        ROS_ERROR("Failed to write() to pico: %s. Attempted string: %s",
                  strerror(errno), s);
        // TODO: TTY error
    }
    tcdrain(fd);
}

void pico::read_tty(char *buf, size_t count) {
    /* memcpy from standard buffer */
    /* read in blocks of 64 directly into buffer */
    /* fill standard buffer and memcpy remaining bytes */
}

/* Find string in pico output and read to the end of it */
/* Used for finding when init sequence was written, skipping to the end of the
 * line, and possibly finding where `inc` occurred. */
void pico::scan_to_string(const char *s, int timeout) {
    /* Use a fixed size ring buffer / deque. Read `len` characters, cmp, shift
     * forward by one. */
    size_t len = strlen(s);
    char *ptr = s;
    std::deque<char> chars;
    while (chars.size() < len) {
        chars.push(next_char());
        if (chars.back() == *ptr) {
            ptr++;
        } else {
            chars.pop();
        }
    }

}

pico::pico(char *dev) {
    fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    while (fd < 0) {
        if (errno == ENOENT || errno == ENXIO) {
            ROS_ERROR("TTY not available; retrying in 5s");
            sleep(5);
            fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
        } else {
            ROS_ERROR("Failed to open %s: %s", dev, strerror(errno));
            exit(1);
        }
    }

    struct termios tty;
    cfsetospeed (&tty, BAUDRATE);
    cfsetispeed (&tty, BAUDRATE);
    tty.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    tty.c_lflag = ICANON;

    /* timeout after 5/10 second */
    tty.c_cc[VTIME] = 5;

    /* read(2) blocks until MIN bytes are available,
     * and returns up to the number of bytes requested. */
    tty.c_cc[VMIN] = BUF_SIZ;
    tcsetattr(fd, TCSANOW, &tty);

    /* Reset pico configuration in RAM */
    //write_tty(reset_pico);
    /* Wait to recieve and discard rogue output from the pico */
    //sleep(3);
    //tcflush(fd, TCIFLUSH);

    write_tty(init_cmd);
    while (true) {
        putc(next_char(), stdout);
    }
    //reset = ros::Time::now();
    //sleep(1);

    /* Read response  */
    //char recieved[sizeof(init_res)];
    //read_tty(recieved, sizeof(init_res) - 1);
    //recieved[sizeof(init_res) - 1] = '\0';


    ///* Scanning for commands... */
    //if (strcmp(recieved, init_res)) {
    //    ROS_ERROR("Pico response mismatch: expected: \n%srecieved: \n%s",
    //              init_res, recieved);
    //    // TODO: pico error
    //}
    /* Reading sample lines... */
}

int main() {
    char device[] = "/dev/PICO";
    pico pico(device);
}
