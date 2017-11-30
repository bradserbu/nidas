/*
 * 	sample program to receive async data
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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "seamac.h"

int main(int argc, char* argv[])
{
	int fd;
	int rc;
	int disc = N_TTY;
	struct seamac_params params;
	struct termios termios;
	FILE *fp = NULL;
	int size = 4096;
	int count;
	unsigned char *buf = malloc(size);
	char *devname;

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySM1";

	printf("Looking for async data on %s\n", devname);

	if (!buf ) {
		printf("can't allocate buffer\n");
		return ENOMEM;
	}

	// open file to capture received data
	fp = fopen("data", "wb");
	if (fp == NULL) {
		printf("fopen(data) error=%d %s\n",
		       errno, strerror(errno));
		return errno;
	}


	// --------------------------------------------------------------------
	// open device with O_NONBLOCK to ignore DCD 
	// --------------------------------------------------------------------
	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open of %s error=%d %s\n",
		       devname, errno, strerror(errno));
		return errno;
	}

	// set N_TTY (standard async) line discipline
	rc = ioctl(fd, TIOCSETD, &disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	
	// --------------------------------------------------------------------
	// Configure port
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
	// RX Data
	// --------------------------------------------------------------------
	for (;;) {
		memset(buf, 0, size);

		rc = read(fd, buf, size);
		if (rc < 0) {
			printf("read() error=%d %s\n",
			       errno, strerror(errno));
			break;
		}
		if (rc == 0) {
			printf("read() returned with no data\n");
			continue;
		}

		printf("received %d bytes of data\n", rc);

		// write received data to capture file
		count = fwrite(buf, sizeof(char), rc, fp);
		if (count != rc) {
			printf("data file write error=%d (%s)\n",
			       errno, strerror(errno));
			break;
		}
		fflush(fp);
	}

	return 0;
}


