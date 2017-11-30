/*
 * 	sample program to send data using External Sync
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
	int fd, rc, i, sigs;
	int disc = N_TTY;
	struct seamac_params params;
	unsigned char *buf;
	int size = 1;
	int iters = 10;
	char *devname = "/dev/ttySM0";

	if (argc > 1)
		devname = argv[1];
	if (argc > 2)
		size = atoi(argv[2]);
	if (argc > 3)
		iters = atoi(argv[3]);

	printf("sending %u byte frames on %s:external sync\n", size, devname);

	buf = malloc(size);
	if (!buf) {
		printf("can't allocate buffer\n");
		return ENOMEM;
	}

	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		return fd;
	}

	// set N_TTY ldisc if it has been changed
	rc = ioctl(fd, TIOCSETD, &disc);
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
	params.mode = SEAMAC_MODE_EXTERNAL;
	params.rate = 1000;    // 1kbps
	params.encoding = SEAMAC_ENCODE_NRZ;
	params.txclk = SEAMAC_CLK_BRG;
	params.rxclk = SEAMAC_CLK_BRG;
	params.rxclktype = SEAMAC_RXCLK_TTL;
	params.telement = SEAMAC_TELEMENT_BRG;
	params.parity = SEAMAC_PARITY_NONE;
	params.txbits = SEAMAC_BITS_8;
	params.rxbits = SEAMAC_BITS_8;
	params.addrfilter = 0x00;
	params.addrrange = 0;
	params.crcpreset = SEAMAC_CRC_PRESET0;
	params.idlemode = SEAMAC_IDLE_FLAG;
	params.underrun = SEAMAC_UNDERRUN_FLAG;
	params.syncflag = 0xFFFF;
	params.sixbitflag = 0;
	params.crctype = SEAMAC_CRC_NONE;

	// NOTE: allow extra time for final character
	params.posttxdelay = (1000 * 8 * 1) / params.rate; 

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

	// External sync - sync pin (CTS) driven low when sync is active
	sigs = TIOCM_RTS | TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIS, &sigs);
	if (rc < 0) {
		printf("failed to set modem control lines.\n");
		return rc;
	}

	for (i = 0; i < iters; i++) {
		int j;

		// Message payload (all printable ascii characters)
		for (j = 0; j < size; j++)
			buf[j] = (0x21 + (j % 0x5E));

		// External sync uses an external signal to trigger 
		// synchronization.  RTS and DSR are used here
		sigs = TIOCM_RTS | TIOCM_DTR;
		rc = ioctl(fd, TIOCMBIC, &sigs);
		if (rc < 0) {
			printf("failed to set modem control lines.\n");
			return rc;
		}

		rc = write(fd, buf, size);
		if (rc < 0) {
			printf("write error=%d %s\n",
			       errno, strerror(errno));
			return rc;
		}

		printf("data sent(%d)... ", rc);
		rc = tcdrain(fd);
		printf("tcdrain()=%d\n", rc);

		sigs = TIOCM_RTS | TIOCM_DTR;
		rc = ioctl(fd, TIOCMBIS, &sigs);
		if (rc < 0) {
			printf("failed to set modem control lines.\n");
			return rc;
		}
	}

	free(buf);
	return 0;
}

