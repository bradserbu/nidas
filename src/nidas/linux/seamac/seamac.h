/*
 *	Sealevel Systems Synchronous Serial driver API.
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
 *	(c) Copyright 2007-2014 Sealevel Systems Inc.
 *
 */

// This is the n_hdlc ldisc id.  It has been part of the kernel for a long time,
// and it is very helpful when working with frame-oriented data.  If this line
// discipline isn't used in SDLC (or other frame oriented modes) there is no
// guarantee that data being passed to and from userspace will not be broken up
// into smaller blocks by internal buffering.
#ifndef N_HDLC
#define N_HDLC 13
#endif

// For now, and because it's easily identifiable, PCI ID is port type
// this is the value of serial_struct.type returned by the TIOCGSERIAL ioctl
#define PORT_SEALEVEL_SYNC		0x135e

// The port configuration struct - shared between userspace and kernel space
// NOTE:  Not every one of these options is currently working in the driver,
//            works with other options, or works across all device models.
struct seamac_params {
	// Overall device configuration mode (further config is subset)
	unsigned char mode;
	#define SEAMAC_MODE_ASYNC		0
	#define SEAMAC_MODE_SDLC 		1
	#define SEAMAC_MODE_MONOSYNC		2
	#define SEAMAC_MODE_BISYNC		3
	#define SEAMAC_MODE_BISYNC_FRAMED	6
	#define SEAMAC_MODE_EXTERNAL		4
	#define SEAMAC_MODE_EXTERNAL_RTS	5

	// Electrical interface to use for this port - unused on ISA/PC-104
	unsigned char interface;
	#define SEAMAC_IF_OFF		0x00
	#define SEAMAC_IF_232		0x02
	#define SEAMAC_IF_422		0x0D
	#define SEAMAC_IF_423		0xFD
	#define SEAMAC_IF_485		0x05
	#define SEAMAC_IF_485E		0x45
	#define SEAMAC_IF_485T		0x04
	#define SEAMAC_IF_485TE		0x44
	#define SEAMAC_IF_530		0x0D
	#define SEAMAC_IF_530A		0x0F
	#define SEAMAC_IF_V35		0x0E
	#define SEAMAC_IF_HWSELECT	0xFF

	// Enable/disable internal loopback
	unsigned char loopback;

	// This is the value of the oscillator... if you don't use a custom 
	// oscillator, you don't need to change this value
	unsigned int baseclk;
	#define SEAMAC_ISA_CLK		7372800
	#define SEAMAC_PCI_CLK		7372800
	#define SEAMAC_PCMCIA_CLK 	14745600

	// The data rate in bits/second
	unsigned int rate;

	// Sync modes - clk can be encoded in the signal to cut down on wiring
	unsigned char encoding;
	#define SEAMAC_ENCODE_NRZ	0x00
	#define SEAMAC_ENCODE_NRZI	0x20
	#define SEAMAC_ENCODE_FM1	0x40
	#define SEAMAC_ENCODE_FM0	0x60

	// The clk for each signal can come from 4 different places
	unsigned char txclk;
	unsigned char rxclk;
	#define SEAMAC_CLK_RXCLK	0x00
	#define SEAMAC_CLK_TXCLK	0x01
	#define SEAMAC_CLK_BRG		0x02
	#define SEAMAC_CLK_DPLL		0x03

	// It could be helpful to wire an oscillator directly to the rxclk
	unsigned char rxclktype;
	#define SEAMAC_RXCLK_TTL	0x00
	#define SEAMAC_RXCLK_XTAL	0x80

	// What should the port echo back out on the telement line?
	unsigned char telement;
	#define SEAMAC_TELEMENT_XTAL	0x00
	#define SEAMAC_TELEMENT_TXCLK	0x01
	#define SEAMAC_TELEMENT_BRG	0x02
	#define SEAMAC_TELEMENT_DPLL	0x03

	// Async - Number of stop bits
	unsigned char stopbits;
	#define SEAMAC_STOP_1		0x04
	#define SEAMAC_STOP_1_5		0x08
	#define SEAMAC_STOP_2		0x0C

	// Primarily Async - Type of parity
	unsigned char parity;
	#define SEAMAC_PARITY_NONE	0x00
	#define SEAMAC_PARITY_EVEN	0x03
	#define SEAMAC_PARITY_ODD	0x01

	// How many bits per character for DATA?
	unsigned char txbits;
	unsigned char rxbits;
	#define SEAMAC_BITS_8		0x03
	#define SEAMAC_BITS_7		0x01
	#define SEAMAC_BITS_6		0x02
	#define SEAMAC_BITS_5		0x00

	// Monosync/bisync flag.  8 or 16 bits depending on mode (l.s.B is flag)
	unsigned short syncflag;
	
	// The size of the flag can be modified to 6 or 12 (based on mode)
	unsigned char sixbitflag;
	#define SEAMAC_6_BIT_FLAG 	0x01

	// The device can filter messages based on the addr field
	unsigned char addrfilter;
	
	// Or it can just filter a range of 16 addresses
	unsigned char addrrange;

	// Only selectable in non SDLC mode
	unsigned char crctype;
	#define SEAMAC_CRC_NONE	 	0x01
	#define SEAMAC_CRC_16	 	0x04
	#define SEAMAC_CRC_CCITT 	0x00

	// The CRC can be preset to a value of 0 or 1
	unsigned char crcpreset;
	#define SEAMAC_CRC_PRESET0 	0x00
	#define SEAMAC_CRC_PRESET1 	0x80

	// The device can either idle 1's or flags
	unsigned char idlemode;
	#define SEAMAC_IDLE_FLAG 	0x00
	#define SEAMAC_IDLE_ONES 	0x08
	#define SEAMAC_IDLE_CUSTOM	0x80

	// Custom idle pattern (for supported devices, idlemode = CUSTOM)
	unsigned short idlepattern;

	// Control what happens on a tx underrun condition
	unsigned char underrun;
	#define SEAMAC_UNDERRUN_ABORT	0x04
	#define SEAMAC_UNDERRUN_FLAG	0x00

	// Hardware flow control options
	unsigned char handshaking;
	#define SEAMAC_HANDSHAKE_NONE		0x00
	#define SEAMAC_HANDSHAKE_RTSCTS		0x01

	// Automatic RTS control during TX by driver
	unsigned char rtscontrol;
	#define SEAMAC_RTSCONTROL_DISABLE	0x00
	#define SEAMAC_RTSCONTROL_TOGGLE 	0x01

	// Delay (ms) before TX to allow bus to settle, useful for slower remote devices that require RTSCONTROL
	unsigned short pretxdelay;

	// Delay (ms) after TX to allow bus to settle, useful for slower remote devices that require RTSCONTROL
	unsigned short posttxdelay;
};

// Outputs (values are irrelevant, grow as necessary)
#define SEAMAC_RTS	1
#define SEAMAC_DTR	2
#define SEAMAC_LL	4
#define SEAMAC_RL	8

// Inputs
#define SEAMAC_CTS	16
#define SEAMAC_DCD	32
#define SEAMAC_DSR	64


// These are the port configuration ioctls.  The standard tcgetattr/tcsetattr
// termios callbacks work fine in async mode, but they don't make much sense in
// sync mode.  These extra ioctls let userspace applications configure the ports
// in more advanced ways on the fly.  Look at the beginning of this header file
// at the seamac_params struct for specific examples of how to configure a port.
// The SEAMAC_IOCTL_GTXACTIVE ioctl queries whether currently transmitting.
#define SEAMAC_MAGIC		0xED
#define SEAMAC_IOCTL_GPARAMS 	_IOR(SEAMAC_MAGIC, 0, struct seamac_params)
#define SEAMAC_IOCTL_SPARAMS 	_IOW(SEAMAC_MAGIC, 1, struct seamac_params)
#define SEAMAC_IOCTL_HUNT 	_IO(SEAMAC_MAGIC, 2)
#define SEAMAC_IOCTL_GTXACTIVE	_IOR(SEAMAC_MAGIC, 3, int)
#define SEAMAC_IOCTL_GSIGNALS	_IOR(SEAMAC_MAGIC, 4, unsigned int)
#define SEAMAC_IOCTL_SSIGNALS	_IOW(SEAMAC_MAGIC, 5, unsigned int)
#define SEAMAC_IOCTL_GSECURITY	_IOR(SEAMAC_MAGIC, 6, unsigned int)
#define SEAMAC_IOCTL_GTXCLKRATE	_IOR(SEAMAC_MAGIC, 7, unsigned int)
#define SEAMAC_IOCTL_GRXCLKRATE	_IOR(SEAMAC_MAGIC, 8, unsigned int)



// So userspace applications can include header without unused variable warnings
#ifdef SEAMAC_DRIVER_INCLUDE

#include <linux/serial_core.h>
extern void seamac_deregister_port(struct uart_port *port);
extern int seamac_register_port(struct uart_port *port);

#define SEALEVEL_MAX_PORTS_PER_ADAPTER  4
#define SEALEVEL_MAX_ISA		2

// Sealevel Vendor/Product ID's (PCI id is already in kernel)
#define PCI_DEVICE_ID_SEALEVEL_5102	0x5102
#define PCI_DEVICE_ID_SEALEVEL_5103	0x5103
#define PCI_DEVICE_ID_SEALEVEL_5402	0x5402
#define PCI_DEVICE_ID_SEALEVEL_5102E	0xe102
#define PCI_DEVICE_ID_SEALEVEL_5103E	0xe103
#define PCI_DEVICE_ID_SEALEVEL_5402E	0xe402
#define PCI_DEVICE_ID_SEALEVEL_9113	0x9113
#define PCI_DEVICE_ID_SEALEVEL_9155	0x9155
#define PCI_DEVICE_ID_SEALEVEL_9198	0x9198

#define ISA_DEVICE_ID_SEALEVEL_3512	0x3512

#define PCMCIA_VENDOR_ID_SEALEVEL	0x0167
#define PCMCIA_DEVICE_ID_SEALEVEL_3612	0x3612

#define SEALEVEL_SYNC_Z85230		0x0001
#define SEALEVEL_SYNC_Z85230_IOSHARED	0x0002
#define SEALEVEL_SYNC_Z85230_INTENABLE	0x0004
#define SEALEVEL_SYNC_9198		0x0010
#define SEALEVEL_SYNC_SINGLEPORT	0x1000
#define SEALEVEL_SYNC_DUALPORT		0x2000
#define SEALEVEL_SYNC_QUADPORT		0x4000

// This is a default port setting - this is async mode and is the standard port
// config for Linux serial ports

static const struct seamac_params seamac_defaults = {
	.mode = SEAMAC_MODE_ASYNC,
	.interface = SEAMAC_IF_232,
	.rate = 38400,
	.stopbits = SEAMAC_STOP_1,
	.parity = SEAMAC_PARITY_NONE,
	.txbits = SEAMAC_BITS_8,
	.rxbits = SEAMAC_BITS_8,
	.loopback = 0x00,
	.encoding = SEAMAC_ENCODE_NRZ,
	.txclk = SEAMAC_CLK_BRG,
	.rxclk = SEAMAC_CLK_BRG,
	.rxclktype = SEAMAC_RXCLK_TTL,
	.telement = SEAMAC_TELEMENT_BRG,
	.syncflag = 0x7FFE,
	.sixbitflag = 0x00,
	.addrfilter = 0xFF,
	.addrrange = 0x00,
	.crctype = SEAMAC_CRC_NONE,
	.crcpreset = SEAMAC_CRC_PRESET0,
	.idlemode = SEAMAC_IDLE_FLAG,
	.idlepattern = 0x0000,
	.underrun = SEAMAC_UNDERRUN_FLAG,
	.handshaking = SEAMAC_HANDSHAKE_NONE,
	.rtscontrol = SEAMAC_RTSCONTROL_DISABLE,
	.pretxdelay = 0,
	.posttxdelay = 0,
};
// these 'defaut' to make the sdlc/hdlc driver work
/*static const struct seamac_params seamac_defaults = {
        .mode = SEAMAC_MODE_SDLC,
        .interface = SEAMAC_IF_422, //SEAMAC_IF_HWSELECT?
        .rate = 4000000,
        .stopbits = SEAMAC_STOP_1,
        .parity = SEAMAC_PARITY_NONE,
        .txbits = SEAMAC_BITS_8,
        .rxbits = SEAMAC_BITS_8,
        .loopback = 0x00,
        .encoding = SEAMAC_ENCODE_NRZ,
        .txclk = SEAMAC_CLK_RXCLK,
        .rxclk = SEAMAC_CLK_RXCLK,
        .rxclktype = SEAMAC_RXCLK_TTL,
        .telement = SEAMAC_TELEMENT_DPLL,
        .parity = SEAMAC_PARITY_NONE,
        .txbits = SEAMAC_BITS_8,
        .rxbits = SEAMAC_BITS_8,
        .syncflag = 0x7FFE, //example has 0xffff
        .sixbitflag = 0x00,
        .addrfilter = 0xFF,
        .addrrange = 0x00,
        .crctype = SEAMAC_CRC_CCITT,
        .crcpreset = SEAMAC_CRC_PRESET1,
        .idlemode = SEAMAC_IDLE_FLAG,
        .idlepattern = 0x0000,
        .underrun = SEAMAC_UNDERRUN_ABORT, //example says FLAG not ABORT, but we've been trying flag so ...
        .handshaking = SEAMAC_HANDSHAKE_NONE,
        .rtscontrol = SEAMAC_RTSCONTROL_DISABLE,
        .pretxdelay = 0,
        .posttxdelay = 0,
	.baseclk = 20000000,
};
*/

#endif

