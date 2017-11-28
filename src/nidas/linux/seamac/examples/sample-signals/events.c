/*
 * 	sample program to read and write port control signals (event driven)
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
#include <linux/serial.h> //icount struct

#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "seamac.h"

void print_icount(struct serial_icounter_struct *icount)
{
	if (icount == NULL) {
		printf("NULL\n");
		return;
	}

	printf("icount { rx: %i,  tx: %i,  cts: %i,  dsr: %i,  dcd: %i }\n",
		icount->rx, icount->tx, icount->cts, icount->dsr, icount->dcd);
}

void print_icount_delta(int fd, struct serial_icounter_struct *pre, struct serial_icounter_struct *post) {
	unsigned long signals = 0;
	ioctl(fd, TIOCMGET, &signals);

	if (pre->cts != post->cts)
		printf("  CTS change detected [%s] (%i -> %i)\n", signals&TIOCM_CTS?" ON":"OFF", pre->cts, post->cts);

	if (pre->dcd != post->dcd)
		printf("  DCD change detected [%s] (%i -> %i)\n", signals&TIOCM_CAR?" ON":"OFF", pre->dcd, post->dcd);

	if (pre->dsr != post->dsr)
		printf("  DSR change detected [%s] (%i -> %i)\n", signals&TIOCM_DSR?" ON":"OFF", pre->dsr, post->dsr);
}

int main(int argc, char* argv[])
{
	int fd;
	unsigned int signals = (TIOCM_CAR | TIOCM_DSR | TIOCM_CTS);
	struct serial_icounter_struct pre_icount, post_icount;
	char *devname = "/dev/ttySM0";

	// No args, use default port name, otherwise use argument
	if (argc > 1) {
		devname = argv[1];
	}

	fd = open(devname, O_RDWR, 0);
	if (fd < 0) {
		printf("open of %s error=%d %s\n", devname, errno, strerror(errno));
		return fd;
	}

	system("clear");

	while (1) {
		ioctl(fd, TIOCGICOUNT, &pre_icount);
//		print_icount(&pre_icount);
	
		printf("\nwaiting for modem control event...\n");
		ioctl(fd, TIOCMIWAIT, signals);

		ioctl(fd, TIOCGICOUNT, &post_icount);
//		print_icount(&post_icount);

		print_icount_delta(fd, &pre_icount, &post_icount);
	}

	close(fd);
	return 0;
}

