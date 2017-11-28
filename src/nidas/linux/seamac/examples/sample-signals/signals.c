/*
 * 	sample program to read and write port control signals (polled)
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

// Attempt to read a character from CLI, return if timeout
char getch() {
	char buf = 0;
	struct termios old;

	fflush(stdout);
	if(tcgetattr(0, &old) < 0)
		perror("tcsetattr()");

	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 0;	// VTIME > VMIN = 0
	old.c_cc[VTIME] = 2;	// timeout or data available

	if(tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");

	if(read(0, &buf, 1) < 0)
		perror("read()");

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if(tcsetattr(0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");

	return buf;
}

// --------------------------------------------------------------------
// Request and display signal states with IOCTL (custom)
// --------------------------------------------------------------------
void print_signals(int fd)
{
	unsigned int signals;

	if (ioctl(fd, SEAMAC_IOCTL_GSIGNALS, &signals) < 0)
		printf("ioctl(SEAMAC_IOCTL_GSIGNALS) failed with err=%d %s\n", errno, strerror(errno));
	else {
		printf("------------------\n");
		printf("control signals\n");
		printf("------------------\n");
		printf("1 || %3s || %s\n", "RTS", signals&SEAMAC_RTS?"ON":"OFF");
		printf("2 || %3s || %s\n", "DTR", signals&SEAMAC_DTR?"ON":"OFF");
		printf("3 || %3s || %s\n", "RL", signals&SEAMAC_RL?"ON":"OFF");
		printf("4 || %3s || %s\n", "LL", signals&SEAMAC_LL?"ON":"OFF");
		printf("  || %3s || %s\n", "CTS", signals&SEAMAC_CTS?"ON":"OFF");
		printf("  || %3s || %s\n", "DCD", signals&SEAMAC_DCD?"ON":"OFF");
		printf("  || %3s || %s\n", "DSR", signals&SEAMAC_DSR?"ON":"OFF");

		printf("\n\n");
	}
}

// --------------------------------------------------------------------
// Get current signal states and invert specified signal with IOCTLs (custom)
// --------------------------------------------------------------------
void toggle_signal(int fd, unsigned int toggle)
{
	unsigned int signals;

	if (ioctl(fd, SEAMAC_IOCTL_GSIGNALS, &signals) < 0)
		printf("ioctl(SEAMAC_IOCTL_GSIGNALS) failed with err=%d %s\n", errno, strerror(errno));
	else {
		signals ^= toggle;
		ioctl(fd, SEAMAC_IOCTL_SSIGNALS, &signals);
	}
}

int main(int argc, char* argv[])
{
	int fd, quit = 0;
	unsigned int init = 0;
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

	// Cannot read all outputs, so set initial states
	ioctl(fd, SEAMAC_IOCTL_SSIGNALS, &init);

	while (!quit) {	
		char key;

		system("clear");
		print_signals(fd);
		printf("\nPress number to toggle; Q to quit.\n");

		// Input request will timeout after 200ms causing a refresh (polling)
		key = getch();
		switch (key) {
			case '1': 	toggle_signal(fd, SEAMAC_RTS); break;
			case '2': 	toggle_signal(fd, SEAMAC_DTR); break;
			case '3': 	toggle_signal(fd, SEAMAC_RL); break;
			case '4': 	toggle_signal(fd, SEAMAC_LL); break;
			case 'Q':
			case 'q': 	quit = 1; break;
		}
	}

	close(fd);
	return 0;
}

