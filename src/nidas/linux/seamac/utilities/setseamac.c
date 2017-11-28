/*
 * 	Sealevel Seamac configuration utility
 *
 * 	Sealevel and Seamac are registered trademarks of Sealevel Systems 
 * 	Incorporated.
 *
 * 	This program is free software; you can redistribute it and/or
 * 	modify it under the terms of the Lesser GNU General Public License
 * 	as published by the Free Software Foundation; either version
 * 	3 of the License, or (at your option) any later version.
 *	LGPL v3
 *
 * 	(c) Copyright 2007-2013 Sealevel Systems Inc.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <linux/serial.h>

#include <sys/ioctl.h>

#include "../include/seamac.h"

unsigned char silent_mode, command_mode, bring_the_noise, slave_mode;
char *progname;
char *version_str = "1.4";



#define FLAG_ARGUMENT	0x80000000
#define FLAG_INVERTS	0x40000000
#define FLAG_MODE	0x00000001
#define FLAG_INTERFACE	0x00000002
#define FLAG_LOOPBACK	0x00000004
#define FLAG_BASE	0x00000008
#define FLAG_RATE	0x00000010
#define FLAG_ENCODE	0x00000020
#define FLAG_TXCLK	0x00000040
#define FLAG_RXCLK	0x00000080
#define FLAG_RXCLKT	0x00000100
#define FLAG_TELEMENT	0x00000200
#define FLAG_STOPBITS	0x00000400
#define FLAG_PARITY	0x00000800
#define FLAG_TXBITS	0x00001000
#define FLAG_RXBITS	0x00002000
#define FLAG_SYNCFLAG	0x00004000
#define FLAG_6BITFLAG	0x00008000
#define FLAG_ADDR	0x00010000
#define FLAG_ADDRRANGE	0x00020000
#define FLAG_CRCTYPE	0x00040000
#define FLAG_CRCPRESET	0x00080000
#define FLAG_IDLEMODE	0x00100000
#define FLAG_UNDERRUN	0x00200000
#define FLAG_RTSCONTROL 0x00400000
#define FLAG_PRETXDLY 	0x00800000
#define FLAG_POSTTXDLY 	0x01000000


void set_null(struct seamac_params *p, unsigned int a){ return; }
void set_mode(struct seamac_params *p, unsigned int a){ p->mode = a; }
void set_interface(struct seamac_params *p, unsigned int a){ p->interface = a; }
void set_loopback(struct seamac_params *p, unsigned int a){ p->loopback = a; }
void set_baseclk(struct seamac_params *p, unsigned int a){ p->baseclk = a; }
void set_rate(struct seamac_params *p, unsigned int a){ p->rate = a; }
void set_encoding(struct seamac_params *p, unsigned int a){ p->encoding = a; }
void set_txclk(struct seamac_params *p, unsigned int a){ p->txclk = a; }
void set_rxclk(struct seamac_params *p, unsigned int a){ p->rxclk = a; }
void set_rxclktype(struct seamac_params *p, unsigned int a){ p->rxclktype = a; }
void set_telement(struct seamac_params *p, unsigned int a){ p->telement = a; }
void set_stopbits(struct seamac_params *p, unsigned int a){ p->stopbits = a; }
void set_parity(struct seamac_params *p, unsigned int a){ p->parity = a; }
void set_txbits(struct seamac_params *p, unsigned int a){ p->txbits = a; }
void set_rxbits(struct seamac_params *p, unsigned int a){ p->rxbits = a; }
void set_syncflag(struct seamac_params *p, unsigned int a){ p->syncflag = a; }
void set_sixbit(struct seamac_params *p, unsigned int a){ p->sixbitflag = a; }
void set_addr(struct seamac_params *p, unsigned int a){ p->addrfilter = a; }
void set_addrrange(struct seamac_params *p, unsigned int a){ p->addrrange = a; }
void set_crctype(struct seamac_params *p, unsigned int a){ p->crctype = a; }
void set_crcpreset(struct seamac_params *p, unsigned int a){ p->crcpreset = a; }
void set_idlemode(struct seamac_params *p, unsigned int a){ p->idlemode = a; }
void set_underrun(struct seamac_params *p, unsigned int a){ p->underrun = a; }
void set_rtscontrol(struct seamac_params *p, unsigned int a){ p->rtscontrol = a; }
void set_pretxdelay(struct seamac_params *p, unsigned int a){ p->pretxdelay = a; }
void set_posttxdelay(struct seamac_params *p, unsigned int a){ p->posttxdelay = a; }

struct command {
	char *name;
	void (*cmd)(struct seamac_params *, unsigned int);
	unsigned char arg;
	int flag;
	char *pretty;
	char *description;
};

struct command config_list[] = {
	{"async", 	set_mode,	SEAMAC_MODE_ASYNC,	FLAG_MODE,
	 "async",	"set device to asynchronous mode"},
	{"sdlc",	set_mode,	SEAMAC_MODE_SDLC,	FLAG_MODE,
	 "sdlc",	"set device to sdlc mode"},
	{"mono",	set_mode,	SEAMAC_MODE_MONOSYNC,	FLAG_MODE,
	 "monosync",	"set device to monosync mode"},
	{"bi",		set_mode,	SEAMAC_MODE_BISYNC,	FLAG_MODE,
	 "bisync",	"set device to bisync mode"},
	{"ext",		set_mode,	SEAMAC_MODE_EXTERNAL,	FLAG_MODE,
	 "external",	"set device to external sync mode (CTS)"},
	{"extrts",	set_mode,	SEAMAC_MODE_EXTERNAL_RTS,	FLAG_MODE,
	 "external",	"set device to external sync mode (RTS)"},

	{"if_off",	set_interface,	SEAMAC_IF_OFF,		FLAG_INTERFACE,
	 "off",		"disable electrical interface"},
	{"rs232",	set_interface,	SEAMAC_IF_232,		FLAG_INTERFACE,
	 "RS-232",	"select rs-232 electrical interface"},
	{"rs485",	set_interface,	SEAMAC_IF_485,		FLAG_INTERFACE,
	 "RS-485",	"select rs-485 electrical interface"},
	{"rs485t",	set_interface,	SEAMAC_IF_485T,		FLAG_INTERFACE,
	 "RS-485T",	"select rs-485 terminated electrical interface"},
	{"rs530",	set_interface,	SEAMAC_IF_530,		FLAG_INTERFACE,
	 "RS-530",	"select rs-530/rs422/rs449/X.21 electrical interface"},
	{"rs530a",	set_interface,	SEAMAC_IF_530A,		FLAG_INTERFACE,
	 "RS-530A",	"select rs-530A electrical interface"},
	{"v35",		set_interface,	SEAMAC_IF_V35,		FLAG_INTERFACE,
	 "V35",		"select V35 electrical interface"},
	{"if_hwselect",	set_null,	SEAMAC_IF_HWSELECT,	FLAG_INTERFACE,
	 "hw select",	"ERROR!  This option should not be listed!"},

	{"loopback",	set_loopback,	0xFF,	FLAG_INVERTS|FLAG_LOOPBACK,
	 NULL, 		"enable internal loopback mode"},

	{"baud_base",	set_baseclk,	0,	FLAG_ARGUMENT|FLAG_BASE,
	 NULL, 		"modify the base input oscillator frequency"},
	{"baud",	set_rate,	0,	FLAG_ARGUMENT|FLAG_RATE,
	 NULL, 		"modify the data rate generated by the BRG"},

	{"nrz",		set_encoding,	SEAMAC_ENCODE_NRZ,	FLAG_ENCODE,
	 "NRZ",		"set data encoding to NRZ mode"},
	{"nrzi",	set_encoding,	SEAMAC_ENCODE_NRZI,	FLAG_ENCODE,
	 "NRZI",	"set data encoding to NRZI"},
	{"fm0",		set_encoding,	SEAMAC_ENCODE_FM0,	FLAG_ENCODE,
	 "FM0",		"set data encoding to biphase space (fm0)"},
	{"fm1",		set_encoding,	SEAMAC_ENCODE_FM1,	FLAG_ENCODE,
	 "FM1",		"set data encoding to biphase mark (fm1)"},

	{"txc",		set_txclk,	SEAMAC_CLK_TXCLK,	FLAG_TXCLK,
	 "txclkpin",	"set tx clock source as txclk pin"},
	{"tx_rxc",	set_txclk,	SEAMAC_CLK_RXCLK,	FLAG_TXCLK,
	 "rxclkpin",	"set tx clock source as rxclk pin"},
	{"tx_dpll",	set_txclk,	SEAMAC_CLK_DPLL,	FLAG_TXCLK,
	 "DPLL",	"set tx clock source as the Digital Phase-Locked Loop"},
	{"tx_brg",	set_txclk,	SEAMAC_CLK_BRG,		FLAG_TXCLK,
	 "BRG",		"set tx clock source as the Baud Rate Generator"},

	{"rxc",		set_rxclk,	SEAMAC_CLK_RXCLK,	FLAG_RXCLK,
	 "rxclkpin",	"set rx clock source as rxclk pin"},
	{"rx_txc",	set_rxclk,	SEAMAC_CLK_TXCLK,	FLAG_RXCLK,
	 "txclkpin",	"set rx clock source as txclk pin"},
	{"rx_dpll",	set_rxclk,	SEAMAC_CLK_DPLL,	FLAG_RXCLK,
	 "DPLL",	"set rx clock source as the Digital Phase-Locked Loop"},
	{"rx_brg",	set_rxclk,	SEAMAC_CLK_BRG,		FLAG_RXCLK,
	 "BRG",		"set rx clock source as the Buad Rate Generator"},

	{"rx_ttl",	set_rxclktype,	SEAMAC_RXCLK_TTL,	FLAG_RXCLKT,
	 "TTL",		"set rx input clock type to TTL"},
	{"rx_xtal",	set_rxclktype,	SEAMAC_RXCLK_XTAL,	FLAG_RXCLKT,
	 "Xtal",	"set rx input clock type to Crystal Oscillator"},

	{"timing_xtal",	set_telement,	SEAMAC_TELEMENT_XTAL,	FLAG_TELEMENT,
	 "Xtal",	"set timing element output pin to Crystal Oscillator"},
	{"timing_txc",	set_telement,	SEAMAC_TELEMENT_TXCLK,	FLAG_TELEMENT,
	 "txclk",	"set timing element to tx clock"},
	{"timing_brg",	set_telement,	SEAMAC_TELEMENT_BRG,	FLAG_TELEMENT,
	 "BRG",		"set timing element to Baud Rate Generator"},
	{"timing_dpll",	set_telement,	SEAMAC_TELEMENT_DPLL,	FLAG_TELEMENT,
	 "DPLL",	"set timing element to Digital Phase-Locked Loop"},

	{"stop_1",	set_stopbits,	SEAMAC_STOP_1,		FLAG_STOPBITS,
	 "1",		"set async stop bits to 1"},
	{"stop_1.5",	set_stopbits,	SEAMAC_STOP_1_5,	FLAG_STOPBITS,
	 "1.5",		"set async stop bits to 1.5"},
	{"stop_2",	set_stopbits,	SEAMAC_STOP_2,		FLAG_STOPBITS,
	 "2",		"set async stop bits to 2"},

	{"parity_none",	set_parity,	SEAMAC_PARITY_NONE,	FLAG_PARITY,
	 "N",		"set parity to NONE"},
	{"parity_even",	set_parity,	SEAMAC_PARITY_EVEN,	FLAG_PARITY,
	 "E",		"set parity to EVEN"},
	{"parity_odd",	set_parity,	SEAMAC_PARITY_ODD,	FLAG_PARITY,
	 "O",		"set parity to ODD"},

	{"tx_5bits",	set_txbits,	SEAMAC_BITS_5,		FLAG_TXBITS,
	 "5",		"set tx size to 5 bits/char"},
	{"tx_6bits",	set_txbits,	SEAMAC_BITS_6,		FLAG_TXBITS,
	 "6",		"set tx size to 6 bits/char"},
	{"tx_7bits",	set_txbits,	SEAMAC_BITS_7,		FLAG_TXBITS,
	 "7",		"set tx size to 7 bits/char"},
	{"tx_8bits",	set_txbits,	SEAMAC_BITS_8,		FLAG_TXBITS,
	 "8",		"set tx size to 8 bits/char"},
	
	{"rx_5bits",	set_rxbits,	SEAMAC_BITS_5,		FLAG_RXBITS,
	 "5",		"set rx size to 5 bits/char"},
	{"rx_6bits",	set_rxbits,	SEAMAC_BITS_6,		FLAG_RXBITS,
	 "6",		"set rx size to 6 bits/char"},
	{"rx_7bits",	set_rxbits,	SEAMAC_BITS_7,		FLAG_RXBITS,
	 "7",		"set rx size to 7 bits/char"},
	{"rx_8bits",	set_rxbits,	SEAMAC_BITS_8,		FLAG_RXBITS,
	 "8",		"set rx size to 8 bits/char"},

	{"syncflag",	set_syncflag,	0,	FLAG_ARGUMENT|FLAG_SYNCFLAG,
	 NULL, 		"set mono or bi sync flag"},
	{"sixbit",	set_sixbit,	0xFF,	FLAG_INVERTS|FLAG_6BITFLAG,
	 NULL, 		"set sync flag size to 6 or 12 bits (mono or bi)"},

	{"addrfilter",	set_addr,	0,	FLAG_ARGUMENT|FLAG_ADDR,
	 NULL, 		"set address filter to reject all unmatching"},
	{"addrrange",	set_addrrange,	0xFF,	FLAG_INVERTS|FLAG_ADDRRANGE,
	 NULL, 		"modify acceptable address to ignore lower nibble"},

	{"crcoff",	set_crctype,	SEAMAC_CRC_NONE,	FLAG_CRCTYPE,
	 "off",		"set crc mode to off"},
	{"crc16",	set_crctype,	SEAMAC_CRC_16,		FLAG_CRCTYPE,
	 "16",		"set crc mode to 16"},
	{"ccitt",	set_crctype,	SEAMAC_CRC_CCITT,	FLAG_CRCTYPE,
	 "ccitt",	"set the sdlc crc mode"},

	{"crcpreset0",	set_crcpreset,	SEAMAC_CRC_PRESET0,	FLAG_CRCPRESET,
	 "0",		"set initial state of crc checker/generator to zero"},
	{"crcpreset1",	set_crcpreset,	SEAMAC_CRC_PRESET1,	FLAG_CRCPRESET,
	 "1",		"set intial state of crc checker/generator to one"},

	{"idleflag",	set_idlemode,	SEAMAC_IDLE_FLAG,	FLAG_IDLEMODE,
	 "flags",	"set device to idle sync flags"},
	{"idleones",	set_idlemode,	SEAMAC_IDLE_ONES,	FLAG_IDLEMODE,
	 "ones",	"set device to idle ones"},

	{"uflag",	set_underrun,	SEAMAC_UNDERRUN_FLAG,	FLAG_UNDERRUN,
	 "flag",	"set device to close frame with flag on tx underrun"},
	{"uabort",	set_underrun,	SEAMAC_UNDERRUN_ABORT,	FLAG_UNDERRUN,
	 "abort",	"set device to close frame with abort on tx underrun"},

	{"rtscontrol",	set_rtscontrol,	SEAMAC_RTSCONTROL_TOGGLE, 	FLAG_INVERTS|FLAG_RTSCONTROL,
	 NULL, 		"enable automatic RTS toggling during data tx"},
	{"pretxdelay",	set_pretxdelay,		0,			FLAG_ARGUMENT|FLAG_PRETXDLY,
	 NULL, 		"time delay in ms before tx after rts is enabled"},
	{"posttxdelay",	set_posttxdelay,	0,			FLAG_ARGUMENT|FLAG_POSTTXDLY,
	 NULL, 		"time delay in ms after tx before rts is disabled"},

	{NULL,		NULL,		0,			0,
	 NULL,		NULL}, 					//break cond
};

int atonum(char *s)
{
	int n;

	while (*s == ' ')
		s++;
	if (strncmp(s, "0x", 2) == 0 || strncmp(s, "0X", 2) == 0)
		sscanf(s + 2, "%x", &n);
	else if (s[0] == '0' && s[1])
		sscanf(s + 1, "%o", &n);
	else
		sscanf(s, "%d", &n);
	return n;
}

char *to_str(int param, unsigned char value)
{
	struct command *p;

	for (p = config_list; p->name; p++) {
		if (p->flag & param) 
			if (p->flag & (FLAG_ARGUMENT | FLAG_INVERTS) 
							|| (p->arg == value))
				return p->pretty;
	}

	return "unknown";
}

void print_serial(struct serial_struct *serinfo)
{
	if (command_mode) return;

	printf("\t");

	printf("(%sseamac) ", (serinfo->type != PORT_SEALEVEL_SYNC)?"non-":"");
	printf("iobase 0x%04X, ", serinfo->port);
	printf("irq %d", serinfo->irq);
	
	printf("\n");
}

unsigned int param_val(int flag, struct seamac_params *p)
{
	flag &= ~(FLAG_ARGUMENT | FLAG_INVERTS);
	switch (flag) {
		case FLAG_MODE:		return p->mode;
		case FLAG_INTERFACE:	return p->interface;
		case FLAG_LOOPBACK:	return p->loopback;
		case FLAG_BASE: 	return p->baseclk;
		case FLAG_RATE:		return p->rate;
		case FLAG_ENCODE:	return p->encoding;
		case FLAG_TXCLK:	return p->txclk;
		case FLAG_RXCLK:	return p->rxclk;
		case FLAG_RXCLKT:	return p->rxclktype;
		case FLAG_TELEMENT:	return p->telement;
		case FLAG_STOPBITS:	return p->stopbits;
		case FLAG_PARITY:	return p->parity;
		case FLAG_TXBITS:	return p->txbits;
		case FLAG_RXBITS:	return p->rxbits;
		case FLAG_SYNCFLAG:	return p->syncflag;
		case FLAG_6BITFLAG:	return p->sixbitflag;
		case FLAG_ADDR:		return p->addrfilter;
		case FLAG_ADDRRANGE:	return p->addrrange;
		case FLAG_CRCTYPE:	return p->crctype;
		case FLAG_CRCPRESET:	return p->crcpreset;
		case FLAG_IDLEMODE:	return p->idlemode;
		case FLAG_UNDERRUN:	return p->underrun;
		case FLAG_RTSCONTROL:	return p->rtscontrol;
		case FLAG_PRETXDLY:	return p->pretxdelay;
		case FLAG_POSTTXDLY:	return p->posttxdelay;
		default:		return 0;
	}
}

char *param_title(int flag)
{
	flag &= ~(FLAG_ARGUMENT | FLAG_INVERTS);
	switch (flag) {
		case FLAG_MODE:		return "Mode";
		case FLAG_INTERFACE:	return "IFace";
		case FLAG_LOOPBACK:	return "Loopback";
		case FLAG_BASE: 	return "Base";
		case FLAG_RATE:		return "Rate";
		case FLAG_ENCODE:	return "Encode";
		case FLAG_TXCLK:	return "Txclk";
		case FLAG_RXCLK:	return "Rxclk";
		case FLAG_RXCLKT:	return "Rxclktype";
		case FLAG_TELEMENT:	return "Telement";
		case FLAG_STOPBITS:	return "Stopbits";
		case FLAG_PARITY:	return "Parity";
		case FLAG_TXBITS:	return "Txbits";
		case FLAG_RXBITS:	return "Rxbits";
		case FLAG_SYNCFLAG:	return "Syncflag";
		case FLAG_6BITFLAG:	return "Sixbit";
		case FLAG_ADDR:		return "Address";
		case FLAG_ADDRRANGE:	return "Range";
		case FLAG_CRCTYPE:	return "CRC";
		case FLAG_CRCPRESET:	return "CRCpreset";
		case FLAG_IDLEMODE:	return "Idle";
		case FLAG_UNDERRUN:	return "Underrun";
		case FLAG_RTSCONTROL:	return "RtsControl";
		case FLAG_PRETXDLY:	return "PreTxDelay";
		case FLAG_POSTTXDLY:	return "PostTxDelay";
		default:		return "?!?!?";
	}
}


void print_params(struct seamac_params *p)
{
	struct command *c;

	if (command_mode) {
		for (c = config_list; c->name; c++) {
			if (c->flag & FLAG_INVERTS) {
				if (!param_val(c->flag, p)) printf("^");
				printf("%s ", c->name);
			}
			else if (c->flag & FLAG_ARGUMENT) {
				printf("%s ", c->name);
				printf("%d ", param_val(c->flag, p));
			}
			else if (c->arg == param_val(c->flag, p))
				printf("%s ", c->name);
		}
	}
	else if (bring_the_noise) {
		int i = 0;

		printf("\t\t");
		for (c = config_list; c->name; c++) {
			if (c->flag & FLAG_INVERTS) {
				printf("%s: ", param_title(c->flag));
				if (!param_val(c->flag, p)) 	printf("off  ");
				else 				printf("on  ");
			}
			else if (c->flag & FLAG_ARGUMENT) {
				printf("%s: ", param_title(c->flag));
				printf("%d  ", param_val(c->flag, p));
			}
			else if (c->arg == param_val(c->flag, p)) {
				printf("%s: ", param_title(c->flag));
				printf("%s  ", c->pretty);
			}
			else
				continue;

			if ((++i % 4) == 0) printf("\n\t\t");
		}
	}
	else {
		printf("\t\t");

		printf("%s: ", param_title(FLAG_MODE));
		printf("%s  ", to_str(FLAG_MODE, p->mode));

		printf("%s: ", param_title(FLAG_INTERFACE));
		printf("%s  ", to_str(FLAG_INTERFACE, p->interface));

		printf("%s: ", param_title(FLAG_RATE));
		printf("%d  ", p->rate);

		printf("%s", to_str(FLAG_TXBITS, p->txbits));
		if (p->rxbits != p->txbits)
			printf("T/%sR", to_str(FLAG_RXBITS, p->rxbits));
		printf("%s", to_str(FLAG_PARITY, p->parity));
		if (p->mode == SEAMAC_MODE_ASYNC)
			printf("%s", to_str(FLAG_STOPBITS, p->stopbits));
		printf("\n\t\t");

		printf("%s: ", param_title(FLAG_LOOPBACK));
		printf("%s  ", (p->loopback)?"on":"off");

		printf("%s: ", param_title(FLAG_ENCODE));
		printf("%s  ", to_str(FLAG_ENCODE, p->encoding));
	}
	printf("\n\n");
}

int update_settings(struct seamac_params *params, char **arg)
{
	struct command *p;
	char *word;
	int do_invert;

	while(*arg) {
		word = *arg++;
		do_invert = 0;
		if (word[0] == '^') {
			do_invert = 1;
			word++;
		}

		for (p = config_list; p->name; p++)
			if (!strcasecmp(p->name, word))
				break;

		if (!p->name) {
			fprintf(stderr, "Invalid option: %s\n", word);
			return -EINVAL;
		}
		if (do_invert && !(p->flag & FLAG_INVERTS)) {
			fprintf(stderr, "Cannot invert option (%s)\n", word);
			return -EINVAL;
		}
		if ((p->flag & FLAG_ARGUMENT) && !(*arg)) {
			fprintf(stderr, "Option (%s) requires a value\n", word);
			return -EINVAL;
		}

		if (do_invert)
			p->cmd(params, ~p->arg);
		else if (!(p->flag & FLAG_ARGUMENT))
			p->cmd(params, p->arg);
		else
			p->cmd(params, atonum(*arg++));
	}
	return 0;
}

void update_termios(int fd, struct seamac_params *params)
{
	struct termios options;
	speed_t speed;

	// In ASYNC mode, termios settings override those set by the ioctl
	// when a port is close/re-opened.
	if (params->mode != SEAMAC_MODE_ASYNC)
		return;

	tcgetattr(fd, &options);

	// We lose baud precision
	if 	(params->rate >= 230400) speed = B230400;
	else if (params->rate >= 115200) speed = B115200;
	else if (params->rate >= 57600)  speed = B57600;
	else if (params->rate >= 38400)  speed = B38400;
	else if (params->rate >= 19200)  speed = B19200;
	else if (params->rate >= 9600)   speed = B9600;
	else if (params->rate >= 4800)   speed = B4800;
	else if (params->rate >= 2400)   speed = B2400;
	else if (params->rate >= 1800)   speed = B1800;
	else if (params->rate >= 1200)   speed = B1200;
	else if (params->rate >= 600)    speed = B600;
	else if (params->rate >= 300)    speed = B300;
	else if (params->rate >= 200)    speed = B200;
	else if (params->rate >= 150)    speed = B150;
	else if (params->rate >= 134)    speed = B134;
	else if (params->rate >= 110)    speed = B110;
	else if (params->rate >= 75)     speed = B75;
	else                             speed = B50;

	cfsetspeed(&options, speed);

	options.c_cflag = CLOCAL | CREAD;

	// We take databits from txbits
	switch (params->txbits) {
	case SEAMAC_BITS_5:	options.c_cflag |= CS5; break;
	case SEAMAC_BITS_6:	options.c_cflag |= CS6; break;
	case SEAMAC_BITS_7:	options.c_cflag |= CS7; break;
	default:
	case SEAMAC_BITS_8:	options.c_cflag |= CS8; break;
	}

	// only two options for stopbits (1.5 is forced to 2)
	switch (params->stopbits) {
	case SEAMAC_STOP_1:	break;
	default:
	case SEAMAC_STOP_1_5:
	case SEAMAC_STOP_2:	options.c_cflag |= CSTOPB; break;
	}

	// Parity settings need to be fixed now
	switch (params->parity) {
	default:
	case SEAMAC_PARITY_NONE: options.c_iflag = IGNPAR; break;
	case SEAMAC_PARITY_ODD:	 options.c_cflag |= PARODD;
	case SEAMAC_PARITY_EVEN: options.c_cflag |= PARENB; 
				 options.c_iflag = INPCK; break;
	}

	tcsetattr(fd, TCSANOW, &options);
}

int write_timed(int fd, void *buf, int size, int timeout)
{
	struct timeval tv;
	fd_set fds;
	int rc, wsize;

	wsize = 0;
	tv.tv_sec = timeout;
	tv.tv_usec = 0;

	// use select call to wait for data available
	do {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		rc = select(fd + 1, NULL, &fds, NULL, &tv);
		if (rc < 0)
			return rc;
		else if (rc > 0) {
			rc = write(fd, buf, size);
			if (rc < 0)
				return rc;
			else
				wsize += rc;
		}
	}
	while (wsize < size);

	return wsize;
}

int read_timed(int fd, void *buf, int size, int timeout)
{
	struct timeval tv;
	fd_set fds;
	int rc;

	if (timeout != -1) {
		// use select() to wait for read data
		tv.tv_sec  = timeout;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);

		rc = select(fd+1, &fds, NULL, NULL, &tv);
		if (rc <= 0)
			return rc;
	}

	rc = read(fd, buf, size);
	return rc;
}

int test_device(int fd, int timeout, int size, int count, int sdlc)
{
	unsigned char *txbuf, *rxbuf;
	int rc, rsize, iter, dsize = size;

	printf("timeout: %d\t size: %d\t count: %d\n", timeout, size, count);
	printf("%s %s\n\n", (slave_mode)?"slave":"", (sdlc)?"sdlc":"");

	size += 3;  // For extra framing bytes

	txbuf = malloc(size * sizeof(char));
	rxbuf = malloc(size * sizeof(char));
	if (!txbuf || !rxbuf) {
		fprintf(stderr, "can't allocate buffers\n");
		return -ENOMEM;
	}
	
	for (iter = 0; iter < count; iter++) {
		if (!slave_mode) {
			int i;
			for (i = 0; i < size; i++)
				txbuf[i] = (unsigned char) i;

			txbuf[0] = 0xFF;
			txbuf[size - 1] = 0xad;
			txbuf[size - 2] = 0xde;

			if (!silent_mode)
				fprintf(stderr,"TX #%d (%d bytes) ",iter,dsize);

			// Last 2 bytes added automagically in sdlc mode (CRC)
			rc = write_timed(fd, txbuf, sdlc?size-2:size, timeout);
			if (rc < 0)
				printf("FAILED (%d)\n", rc);
			if (!silent_mode)
				fprintf(stderr, "OK... ");
		}

		memset(rxbuf, 0x00, size);
		rsize = rc = 0;
		
		// 1/2 a second pause to allow some of the data to be tx'd
		usleep(500000);

		if (!silent_mode)
			fprintf(stderr, "RX #%d ", iter);

		do {
			rc = read_timed(fd, &rxbuf[rsize], size-rsize, timeout);
			if (rc < 0) {
				fprintf(stderr, "FAILED (%d)\n\n", rc);
				return rc;
			}

			rsize += rc;
		} while (!sdlc && rc && rsize < size);

		if (!silent_mode)
			fprintf(stderr, "(%d bytes) ... ", rsize);
	
		if (slave_mode) {
			if (!rsize) {
				iter--;
				fprintf(stderr, "\n");
				continue;
			}
			memcpy(txbuf, rxbuf, size);
			write_timed(fd, txbuf, (sdlc)?rsize-2:rsize, timeout);
			fprintf(stderr, "TX echo\n");
		}
		else {
			if (memcmp(rxbuf, txbuf, (sdlc)?size-2:size) == 0)
				fprintf(stderr, "OK\n");
			else {
				fprintf(stderr, "FAILED\n");
				return -1;
			}
		}
	}
	
	if (!slave_mode)
		printf("\nLoopback Test: PASSED\n\n");

	return 0;
}


void get_info(char *dev)
{
	int fd, rc;
	struct serial_struct serinfo;
	struct seamac_params params;

	// Open dev
	fd = open(dev, O_RDWR|O_NONBLOCK, 0);
	if (fd < 0) {
		perror(dev);
		return;
	}

	// General serial port info (should work on ANY serial port)
	rc = ioctl(fd, TIOCGSERIAL, &serinfo);
	if (rc < 0) {
		perror("Cannot get serial info");
		return;
	}
	
	printf("%s ", dev);
	print_serial(&serinfo);
	
	// Seamac specific port settings (some of this is mirrored in termios)
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		return;
	}

	print_params(&params);

	close(fd);
}

int set_params(char *dev, char **args)
{
	int fd, rc;
	struct seamac_params params;

	// Open the port.
	fd = open(dev, O_RDWR|O_NONBLOCK, 0);
	if (fd < 0) {
		perror(dev);
		return -ENOENT;
	}

	// Get the current settings.
	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if (rc < 0) {
		perror("Couldn't get device settings\n");
		return -EPERM;
	}

	// Decode the arguments and fix up the params struct accordingly
	rc = update_settings(&params, args);
	if (rc < 0)
		return rc;
	update_termios(fd, &params);
	
	// Set the new settings.
	rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params);
	if (rc < 0) {
		perror("Couldn't set device settings\n");
		return -EPERM;
	}

	close(fd);
	if (!silent_mode) get_info(dev);

	return 0;
}

int loopback_test(char *dev, char **arg)
{
	int fd, rc, old_ldisc, new_ldisc = N_TTY;
	int timeout = 1, size = 1024, count = 5, sdlc = 0;
	struct seamac_params params;
	struct termios options, oldoptions;

	printf("loopback test(%s)\n", dev);

	while (*arg) {
		char *cmd = *arg++, *argument = *arg++;

		if (strcasecmp(cmd, "timeout") == 0) {
			if (argument == NULL || atonum(argument) < -1) {
				fprintf(stderr, "timeout requires arg >= -1");
				return -EINVAL;
			}
			timeout = atonum(argument);
			continue;
		}
		if (strcasecmp(cmd, "size") == 0) {
			if (argument == NULL || atonum(argument) <= 0) {
				fprintf(stderr, "size requires arg > 0");
				return -EINVAL;
			}
			size = atonum(argument);
			continue;
		}
		if (strcasecmp(cmd, "count") == 0) {
			if (argument == NULL || atonum(argument) <= 0) {
				fprintf(stderr, "count requires arg > 0");
				return -EINVAL;
			}
			count = atonum(argument);
			continue;
		}

		fprintf(stderr, "Unknown command %s\n", cmd);
		return -EINVAL;
	}

	fd = open(dev, O_RDWR|O_NONBLOCK, 0);
	if (fd < 0) {
		perror(dev);
		return -ENOENT;
	}

	rc = ioctl(fd, SEAMAC_IOCTL_GPARAMS, &params);
	if ((rc == 0) && (params.mode != SEAMAC_MODE_ASYNC)) {
		new_ldisc = N_HDLC;
		sdlc = 1;
	}

	ioctl(fd, TIOCGETD, &old_ldisc);
	rc = ioctl(fd, TIOCSETD, &new_ldisc);
	if (rc < 0) {
		fprintf(stderr, "Can't set line discipline\n");
		return -1;
	}

	tcgetattr(fd, &oldoptions);
	tcgetattr(fd, &options);

	options.c_oflag = 0; 			// Raw output
	options.c_lflag = 0; 			// Enable canonical input
	options.c_cflag |= (CLOCAL | CREAD); 	// Enable the rx and set local
	options.c_iflag = 0; 			// Input magic begone!
	tcsetattr(fd, TCSANOW, &options);

	rc = test_device(fd, timeout, size, count, sdlc);

	ioctl(fd, TIOCSETD, &old_ldisc);
	tcsetattr(fd, TCSANOW, &oldoptions);
	close(fd);

	return rc;
}

int usage()
{
	int prev = 0;
	struct command *p;

	fprintf(stderr, "%s\n", version_str);
	fprintf(stderr, "(Build: %s %s)\n\n", __DATE__, __TIME__);

	fprintf(stderr, "usage:");
	fprintf(stderr, "\t %s device [-qVth] [cmd1 [arg]] ...\n", progname);
	fprintf(stderr, "\t %s -g [-aG] device1 ...\n\n", progname);

	fprintf(stderr, "Available commands: (* = Takes an argument)\n");
	fprintf(stderr, "\t\t(^ = Can be inverted by preceding with '^')\n\n");

	// Thanks to the commands array, we have all the commands/types already
	for (p = config_list; p->name; p++) {
		if (p->cmd == set_null) continue;

		// put a blank line between command type groupings
		if (prev != (p->flag & ~(FLAG_ARGUMENT | FLAG_INVERTS))) {
			prev = (p->flag & ~(FLAG_ARGUMENT | FLAG_INVERTS));
			fprintf(stderr, "\n");
		}

		fprintf(stderr, "  ");

		if (p->flag & FLAG_ARGUMENT)
			fprintf(stderr, "* ");
		if (p->flag & FLAG_INVERTS)
			fprintf(stderr, "^ ");

		// command / description
		fprintf(stderr, "%-15s\t%s\n", p->name, p->description);
	}

	return 0;
}

int main(int argc, char **argv)
{
	int	get_flag = 0, test_flag = 0;
	int	c;
	extern int optind;
	
	progname = argv[0];
	
	// Display usage if no arguments are passed or --help or help is
	if (argc == 1 || !strcmp(argv[1], "--help") || !strcmp(argv[1], "help"))
		return usage();

	while ((c = getopt(argc, argv, "agGhqstV")) != EOF) {
		switch (c) {
		case 'a':
			bring_the_noise++;
			break;
		case 'g':
			get_flag++;
			break;
		case 'G':
			command_mode++;
			break;
		case 'q':
			silent_mode++;
			break;
		case 's':
			slave_mode++;
			break;
		case 't':
			test_flag++;
			break;
		case 'V':
			fprintf(stderr, "%s\n", version_str);
			return 0;
		case 'h':
		default:
			return usage();
		}
	}

	// Treat all arguments as device files and print info about each
	if (get_flag) {
		argv += optind;
		while (*argv)
			get_info(*argv++);
		return 0;
	}

	// We reached the end of the arguments, but found no device
	if (argc == optind) {
		usage();
		return -1;
	}

	// Run port test (can be setup for different testing methods/parameters)
	if (test_flag)
		return loopback_test(argv[optind], (argv + optind + 1));

	// No special options and no further arguments (they want port info)
	if (argc-optind == 1)
		get_info(argv[optind]);
	// No special options and settings to change
	else
		return set_params(argv[optind], (argv + optind + 1));

	return 0;
}

