/*
 * 	sample program to receive Monosync data
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

int configure(int device)
{
	int retval = 0;
	int ldisc = N_TTY;	//Monosync, Bisync, Raw use standard ldisc
	struct seamac_params params;
	struct termios termios;

	// ensure the proper ldisc is set
	retval = ioctl(device, TIOCSETD, &ldisc);
	if(retval < 0) {
		printf("ERROR, can't set line discipline (error=%d %s)\n",
		       errno, strerror(errno));
		return retval;
	}

	// These are standard tty ldisc settings -- make sure you set these
	// if you have problems with certain characters disapearing or being
	// transformed (0xOD->0x0A, Ox11, 0x13, etc.)
	retval = tcgetattr(device, &termios);
	if (retval < 0) {
		printf("tcgetattr() error=%d %s\n", errno, strerror(errno));
		return retval;
	}

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = CREAD | CS8 | HUPCL | CLOCAL;
	termios.c_lflag = 0;
	termios.c_cc[VTIME] = 0;
	termios.c_cc[VMIN]  = 1;
	cfsetospeed(&termios, B9600);
	cfsetispeed(&termios, B9600);

	retval = tcsetattr(device, TCSANOW, &termios);
	if (retval < 0) {
		printf("tcsetattr() error=%d %s\n", errno, strerror(errno));
		return retval;
	}

	// get current device parameters
	retval = ioctl(device, SEAMAC_IOCTL_GPARAMS, &params);
	if (retval < 0) {
		printf("ioctl(SEAMAC_IOCTL_GPARAMS) failed with err=%d %s\n",
		       errno, strerror(errno));
		return retval;
	}

	// modify device parameters 
	params.mode = SEAMAC_MODE_MONOSYNC;
	params.rate = 0;    // this is a slave, getting clk from master
	params.encoding = SEAMAC_ENCODE_NRZ;
	params.txclk = SEAMAC_CLK_RXCLK;
	params.rxclk = SEAMAC_CLK_RXCLK;
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
	params.syncflag = 0xFF16;
	params.sixbitflag = 0;
	params.crctype = SEAMAC_CRC_NONE;

	// If the device has a software selectable interface, this will set it
	params.interface = SEAMAC_IF_232;

	// set current device parameters
	retval = ioctl(device, SEAMAC_IOCTL_SPARAMS, &params);
	if (retval < 0) {
		printf("ioctl(SEAMAC_IOCTL_SPARAMS) failed with err=%d %s\n",
		       errno, strerror(errno));
	}

	return retval;
}

int main(int argc, char* argv[])
{
	char *devname;
	int device;
	FILE *captureFile = NULL;
	int retval = 0;

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySM1";

	// open file to capture received data
	captureFile = fopen("data", "wb");
	if (captureFile == NULL) {
		printf("fopen(data) error=%d %s\n",
		       errno, strerror(errno));
		return errno;
	}

	// Open device for communication
	device = open(devname, O_RDWR, 0);
	if (device < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		return device;
	}

	// configure device communications parameters
	retval = configure(device);
	if (retval < 0)
		return retval;

	printf("Searching for monosync frames on %s\n", devname);

	// Receive all possible messages
	for (;;) {
		int i, count = 0, fcount = 0;
		int STX = 0, EOT = 0;
		unsigned char message[4096];

		memset(message, 0, sizeof(message));

		// go until EOT or buffer full
		while (!EOT && count < sizeof(message)) {
			// read from device
			retval = read(device, 
				      &message[count], 
				      sizeof(message) - count);

			if (retval < 0) {
				printf("read error = %d (%s)\n", 
					errno, strerror(errno));
				return retval;
			}

			// search for STX
			if (!STX) {
				for (i = 0; i < retval; i++) {
					if (message[count + i] == 0x02) {
						STX = 1;
						retval -= (i + 1);

						// shift data after STX foward
						memmove(&message[count],
							&message[count + i + 1],
							retval);
						break;
					}
				}
			}

			// It may now be necessary to re-sync the receiver
			if (!STX && message[count] != 0x16)
				ioctl(device, SEAMAC_IOCTL_HUNT);

			// no data read or no STX in the data
			if (retval == 0)
				continue;

			// look for EOT
			for (i = 0; i < retval; i++) {
				if (message[count + i] == 0x03) {
					EOT = 1;
					retval = i;
					break;
				}
			}

			// adjust buffer alignments...
			if (STX)
				count += retval;
		}

		printf("received %d byte monosync frame\n", count);

		// write received data to capture file
		fcount = fwrite(message, sizeof(char), count, captureFile);
		if (fcount != count) {
			printf("data file write error=%d (%s)\n",
			       errno, strerror(errno));
			break;
		}
		fflush(captureFile);
	}

	return 0;
}


