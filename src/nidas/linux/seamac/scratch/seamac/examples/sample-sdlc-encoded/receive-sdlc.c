/*
 * 	sample program to receive SDLC data (Clock encoded in data)
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
	int fd;
	int rc;
	int hdlc_disc = N_HDLC;
	struct seamac_params params;
	FILE *fp = NULL;
	int size = 4096;
	int count;
	unsigned char *buf = malloc(size);
	char *devname;

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySM1";

	printf("searching for SDLC frames on %s\n", devname);

	if (!buf ) {
		printf("can't allocate buffer\n");
		return ENOMEM;
	}

	// open file to capture received data
	fp = fopen("sdlc-data.bin", "wb");
	if (fp == NULL) {
		printf("fopen(data) error=%d %s\n",
		       errno, strerror(errno));
		return errno;
	}

	// Open device for communication
	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		return fd;
	}

	// set N_HDLC line discipline (used for SDLC mode)
	rc = ioctl(fd, TIOCSETD, &hdlc_disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	// get current device parameters
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_GPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}

	// modify device parameters 
	params.mode = SEAMAC_MODE_SDLC;
	params.rate = 1228800;  //16X expected FM (32X in NRZI)
	params.encoding = SEAMAC_ENCODE_FM0;
	params.txclk = SEAMAC_CLK_DPLL;
	params.rxclk = SEAMAC_CLK_DPLL;
	params.rxclktype = SEAMAC_RXCLK_TTL;
	params.telement = SEAMAC_TELEMENT_DPLL;
	params.parity = SEAMAC_PARITY_NONE;
	params.txbits = SEAMAC_BITS_8;
	params.rxbits = SEAMAC_BITS_8;
	params.addrfilter = 0xFF;
	params.addrrange = 0;
	params.crcpreset = SEAMAC_CRC_PRESET0;
	params.idlemode = SEAMAC_IDLE_FLAG;
	params.underrun = SEAMAC_UNDERRUN_FLAG;

	// If the device has software selectable interface, this will set it
	params.interface = SEAMAC_IF_232;


	// set current device parameters
	rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(SEAMAC_IOCTL_SPARAMS) on device %s"
		       " failed with err=%d %s\n",
		       devname, errno, strerror(errno));
		return rc;
	}

	for (;;) {
		memset(buf, 0, size);

		// Read will block until a frame is available
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

		printf("received %d byte SDLC frame\n", rc);

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


