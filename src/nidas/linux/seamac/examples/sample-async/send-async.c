/*
 *	sample program to send async data
 *
 * 	Sealevel and Seamac are registered trademarks of Sealevel Systems 
 * 	Incorporated.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the Lesser GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	3 of the License, or (at your option) any later version.
 * 	LGPL v3
 *
 *	(c) Copyright 2007-2013 Sealevel Systems Inc.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <termios.h>
#include <unistd.h>

#include <linux/types.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include "seamac.h"

int main(int argc, char* argv[])
{
	int fd, rc, i;
	int disc = N_TTY;
	struct seamac_params params;
	struct termios termios;
	unsigned char *buf;
	int size = 256, iterations = 5;
	char *devname = "/dev/ttySM0";

	if (argc > 1)
		devname = argv[1];
	if (argc > 2)
		size = atoi(argv[2]);
	if (argc > 3)
		iterations = atoi(argv[3]);

	printf("sending %u async bytes on %s, %i runs\n", 
						size, devname, iterations);

	buf = malloc(size);
	if (!buf) {
		printf("can't allocate buffer\n");
		return ENOMEM;
	}

	// --------------------------------------------------------------------
	// Open port
	// --------------------------------------------------------------------
	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		return fd;
	}

	// set N_TTY (standard async) line discipline
	rc = ioctl(fd, TIOCSETD, &disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	
	// --------------------------------------------------------------------
	// Configure device settings
	// --------------------------------------------------------------------

	// First ensure the device is set to async mode... then we can use
	// standard termios calls to configure the port.
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_GPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}

	params.mode = SEAMAC_MODE_ASYNC;

	// now set the parameters
	rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_SPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}


	// These are standard tty ldisc settings -- make sure you set these
	// if you have problems with certain characters disapearing or being
	// transformed (0xOD->0x0A, Ox11, 0x13, etc.)
	rc = tcgetattr(fd, &termios);
	if (rc < 0) {
		printf("tcgetattr() error=%d %s\n", errno, strerror(errno));
		return rc;
	}

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = CREAD | CS8 | HUPCL | CLOCAL;
	termios.c_lflag = 0;
	termios.c_cc[VTIME] = 0;
	termios.c_cc[VMIN]  = 1;
	cfsetospeed(&termios, B9600);
	cfsetispeed(&termios, B9600);

	rc = tcsetattr(fd, TCSANOW, &termios);
	if (rc < 0) {
		printf("tcsetattr() error=%d %s\n", errno, strerror(errno));
		return rc;
	}


	// get current device parameters -- some of the settings in termios are
	// duplicated, but termios settings will override those set by this
	// ioctl on port close and when tcsetattr() is called.  If you want to
	// use a non-standard async port config: set sane settings with termios,
	// then set your custom options using the ioctl
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_GPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}

	// modify device parameters - note baud,parity,bits per char can
	// also be set using the standard termios struct (see above)
	// NOTE:  termios port (cflag) settings are ignored when mode is set
	// to anything other than ASYNC.
	params.mode = SEAMAC_MODE_ASYNC;
	params.rate = 38400;
	params.stopbits = SEAMAC_STOP_1;
	params.parity = SEAMAC_PARITY_NONE;
	params.rxbits = SEAMAC_BITS_8;
	params.txbits = SEAMAC_BITS_8;

	// If the device has software selectable interface, this will set it
	params.interface = SEAMAC_IF_232;

	// now set the parameters
	rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_SPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}


	// --------------------------------------------------------------------
	// TX Data
	// --------------------------------------------------------------------

	for (i = 0; i < iterations; i++) {
		int j;
		// initialize send buffer
		for (j = 0; j < size; j++)
			buf[j] = (unsigned char)j;

		printf("writing NOW\n");
		rc = write(fd, buf, size);
		if (rc < 0) {
			printf("write error=%d %s\n",
			       errno, strerror(errno));
			return rc;
		}
	
		rc = tcdrain(fd);
		printf("iteration %i complete (rc=%d)\n", (i + 1), rc);
	}

	printf("all data sent\n");
	return 0;
}

