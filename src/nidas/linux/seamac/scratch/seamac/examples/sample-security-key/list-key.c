/*
 * 	sample program to list port security key
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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "seamac.h"

void print_key(char *devname)
{
	int fd;
	unsigned int security;

	// --------------------------------------------------------------------
	// open device
	// --------------------------------------------------------------------
	fd = open(devname, O_RDWR, 0);
	if (fd < 0)
		printf("open of %s error=%d %s\n", devname, errno, strerror(errno));
	else {
		// --------------------------------------------------------------------
		// Request security key with IOCTL
		// --------------------------------------------------------------------
		if (ioctl(fd, SEAMAC_IOCTL_GSECURITY, &security) < 0)
			printf("ioctl(SEAMAC_IOCTL_GSECURITY) on device %s"
				" failed with err=%d %s\n",
				devname, errno, strerror(errno));
		else
			printf("%s security key is %08X\n", devname, security);

		close(fd);
	}
}

int main(int argc, char* argv[])
{
	int i;

	// No args specified, just print what should be the first port
	if (argc == 1) {
		print_key("/dev/ttySM0");
		return 0;
	}

	// Assume each argument is a devname and try to print the key
	for (i = 1; i < argc; i++)
		print_key(argv[i]);

	return 0;
}


