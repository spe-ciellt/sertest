/*
 * Program to test serial port routines on different platforms.
 *
 * This is free and unencumbered software released into the public domain.
 *
 * (C) 2013, Ciellt AB, Stefan Petersen, (spe@ciellt.se)
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <signal.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <sys/stat.h>

#include <fcntl.h>
#include <termios.h>

#include <sys/time.h>

#include <ctype.h>

const char default_serialdevice[] = "/dev/ttyUSB0";
const int default_serialspeed = B115200;

const char teststring[] = "only carrige \n return in the middle";

static void
logperror(char *s)
{
    perror(s);
} /* logperror */


static int
open_serial(const char *device, int speed)
{
    int fd;
    struct termios termios;

    if ((fd = open(device, O_RDWR | O_NOCTTY)) < 0) {
	logperror("open serial device");
	return -1;
    }

    if (tcgetattr(fd, &termios) < 0) {
	logperror("tcgetattr");
	return -1;
    }

    if (cfsetispeed(&termios, speed) < 0) {
	logperror("cfsetispeed ");
	return -1;
    }

    if (cfsetospeed(&termios, speed) < 0) {
	logperror("cfsetospeed ");
	return -1;
    }

    /* Set port to raw mode, like cfmakeraw() would do */
    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= ~(ONLCR | OPOST);
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag |= CS8;

    if (tcsetattr(fd, TCSANOW, &termios) < 0) {
	logperror("tcsetattr");
	return -1;
    }

    return fd;
} /* open_serial */


int
string2speed(char *speed)
{
    int serialspeed = 0;

    switch (atoi(optarg)) {           
    case 1200:
        serialspeed = B1200;
        break;
    case 1800:
        serialspeed = B1800;
        break;
    case 2400:
        serialspeed = B2400;
        break;
    case 4800:
        serialspeed = B4800;
        break;
    case 9600:
        serialspeed = B9600;
        break;
    case 19200:
        serialspeed = B19200;
        break;
    case 38400:
        serialspeed = B38400;
        break;
    case 57600:
        serialspeed = B57600;
        break;
    case 115200:
        serialspeed = B115200;
        break;
    default:
        serialspeed = -1;
    }

    return serialspeed;
} /* string2speed */


#define TIMEOUT_READ_SEC 1
#define TIMEOUT_READ_USEC 0


static int
receive_teststring(int fd)
{
    fd_set rfds;
    struct timeval timeout;
    int nuf_fd;
    int nuf_read = 0;
    char buffer = 0;

    /* Fill in the fd_set */
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    /* Fill in timeout values */
    timeout.tv_sec = TIMEOUT_READ_SEC;
    timeout.tv_usec = TIMEOUT_READ_USEC;

    nuf_fd = select(fd + 1, &rfds, NULL, NULL, &timeout);

    if (nuf_fd == 1) {
        /* read available */
        /* FALLTHRU */
    } else if (nuf_fd == 0) {
        /* Timeout */
        return -2;
    } else if (nuf_fd == -1) {
        perror("select");
        return -1;
    } else {
        fprintf(stderr, "Unknown return value from read select.\n");
        return -1;
    }

    /* If we end up here, we have data to read */
    nuf_read = read(fd, (void *)&buffer, sizeof(buffer));
    if (nuf_read != sizeof(buffer)) {
        fprintf(stderr, "read uneven number of read data");
    }

    if (isprint(buffer)) {
        printf("%c", buffer);
    } else {
        printf("[%d]", buffer);
    }

    return 0;
} /* receive_teststring */


static void
send_teststring(int fd)
{
    int i;
    char out;

    for (i = 0; teststring[i] != 0; i++) {
        out = teststring[i];
        write(fd, &out, sizeof(out));
    }

} /* send_teststring */


int
main(int argc, char *argv[])
{
    int fd = 0;
    char *device = (char *)default_serialdevice;
    int speed = default_serialspeed;

    if (argc >= 2) {
        device = argv[1];
    }

    if (argc >= 3) {
        speed = string2speed(argv[2]);
    }

    if (speed < 0) {
        fprintf(stderr, "No such serial speed %s\n", argv[2]);
        exit(0);
    }

    if ((fd = open_serial(device, speed)) < 0) {
        fprintf(stderr, "Failed to open serial port %s at speed %d\n",
                device, speed);
        exit(0);
    }

    send_teststring(fd);
    while (receive_teststring(fd) == 0);

    close(fd);

    printf("\n");

    return 0;
} /* main */
