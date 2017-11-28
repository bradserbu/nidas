/*
 * 	sample program to send and receive framed bisync data
 * 	NOTE: 9198 custom product only (requires external clocking)
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
 *	(c) Copyright 2013 Sealevel Systems Inc.
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


void inline printb(unsigned char *data, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		printf("%02X%s", data[i], ((i+1) % 16 == 0) ? "\n" : " ");
	}
	printf("\n");
}


int main(int argc, char* argv[])
{
	int fd, rc, i, j;
	int hdlc_disc = N_HDLC;
	struct seamac_params params;
	unsigned char *buf, *rxbuf;
	int size = 128, iterations = 1;
	char *devname = "/dev/ttySM0";

	if (argc > 1)
		devname = argv[1];
	if (argc > 2)
		size = atoi(argv[2]);
	if (argc > 3)
		iterations = atoi(argv[3]);

	buf = malloc(size + 4);
	if (!buf) {
		printf("can't allocate buffer\n");
		rc = -ENOMEM;
		goto malloc_fail;
	}
	rxbuf = malloc(size);
	if (!rxbuf) {
		printf("can't allocate buffer\n");
		rc = -ENOMEM;
		goto malloc_fail;
	}

	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		goto malloc_fail;
	}

	// set N_HDLC line discipline (used for frame-oriented data)
	rc = ioctl(fd, TIOCSETD, &hdlc_disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
		       errno, strerror(errno));
		goto malloc_fail;
	}

	// get current device parameters
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_GPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		goto malloc_fail;
	}

	if (params.mode != SEAMAC_MODE_BISYNC_FRAMED) {
		printf("error: device does not support bisync-framed... correct hardware?\n");
		rc = -1;
		goto malloc_fail;
	}
	if (params.interface != SEAMAC_IF_423) {
		printf("error: device does not support RS-423... correct hardware?\n");
		rc = -1;
		goto malloc_fail;
	}
	if (params.syncflag != 0x5C5C) {
		printf("error: device does not support 5C5C syncflag... correct hardware?\n");
		rc = -1;
		goto malloc_fail;
	}

	// modify device parameters if desired (idlepattern, handshaking, pre/post tx delay)
	params.idlepattern = 0xFFFF;
	params.handshaking = SEAMAC_HANDSHAKE_RTSCTS;
//	params.handshaking = SEAMAC_HANDSHAKE_NONE;
	params.pretxdelay = 0; 
	params.posttxdelay = 0; 

	// set current device parameters
	rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_SPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		goto malloc_fail;
	}

	// Loopback test
	for (i = 0; i < iterations; i++) {
		unsigned int txrate, rxrate;
	
		printf("sending %u byte bisync frames on %s\n", size, devname);

		ioctl(fd, SEAMAC_IOCTL_GTXCLKRATE, &txrate);
		ioctl(fd, SEAMAC_IOCTL_GRXCLKRATE, &rxrate);

		printf(" tx clk rate = %i Hz\n", txrate);
		printf(" rx clk rate = %i Hz\n", rxrate);

		// initialize send buffer
		for (j = 2; j < size + 2; j++)
			buf[j] = (unsigned char) j;

		// initialize start and end flags
		buf[0] = buf[1] = buf[size + 2] = buf[size + 3] = 0x5C;

		printb(buf, size + 4);
//		ioctl(fd, SEAMAC_IOCTL_HUNT);
		rc = write(fd, buf, size + 4);
		if (rc < 0) {
			printf("write error=%d %s\n", errno, strerror(errno));
			goto malloc_fail;
		}
		printf("all data sent rc=%d\n", rc);

		// initialize receive buffer
		memset(rxbuf, 0x00, size);

		rc = read(fd, rxbuf, size);
		if (rc < 0) {
			printf("read error=%d %s\n", errno, strerror(errno));
		}
		printb(rxbuf, rc);

		//usleep(1000 * 500);
	}

	rc = 0;

 malloc_fail:
	if (buf) free(buf);
	if (rxbuf) free(rxbuf);

	return rc;
}

