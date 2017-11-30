/*
 *	Sealevel Systems Z85X30 driver implementation
 *
 *	Sealevel and Seamac are registered trademarks of Sealevel Systems
 *	Incorporated.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *	GPL v2
 *
 *	(c) Copyright 2007-2014 Sealevel Systems, Inc.
 *
 */


#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/circ_buf.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/uaccess.h>

#define SEAMAC_DRIVER_INCLUDE
#include "seamac.h"
#include "debug.h"
#include "compat.h"
#include "z85230.h"


/*******************************************************************/
/*              8530 Write Register 0                              */
/*******************************************************************/
#define     RESET_EXT_STATUS_INT        0x10
#define     SEND_ABORT                  0x18
#define     ENABLE_INT_NEXT_RX_CHAR     0x20
#define     RESET_TX_INT_PEND           0x28
#define     ERROR_RESET                 0x30
#define     RESET_HIGHEST_IUS           0x38
#define     NULL_CODE                   0x00
#define     RESET_RX_CRC                0x40
#define     RESET_TX_CRC                0x80
#define     RESET_TX_UR_EOM             0xC0

/*******************************************************************/
/*              8530 Write Register 1                              */
/*******************************************************************/
#define     WAIT_DMA_DISABLE            0x00
#define     INT_DISABLE                 0x00
#define     EXT_INT_ENABLE              0x01
#define     TX_INT_ENABLE               0x02
#define     PARITY_IS_SP_COND           0x04
#define     RX_INT_DISABLE              0x00
#define     RX_INT_1ST_CHAR_SP_COND     0x08
#define     RX_INT_ALL_CHAR_SP_COND     0x10
#define     RX_INT_SP_COND              0x18
#define     WAIT_DMA_RX                 0x20
#define     WAIT_DMA_TX                 0x00
#define     WAIT_DMA_FUNCTION           0x40
#define     WAIT_REQ_ENABLE             0x80

/*******************************************************************/
/*              8530 Write Register 2                              */
/*******************************************************************/
/* This register is used as a general purpose scratch              */
/*  register on the ACB cards.                                     */
/*******************************************************************/

/*******************************************************************/
/*              8530 Write Register 3                              */
/*******************************************************************/
#define     RX_ENABLE                   0x01
#define     SYNC_CHAR_LOAD_INHIBIT      0x02
#define     ADDRESS_SEARCH_MODE         0x04
#define     RX_CRC_ENABLE               0x08
#define     ENTER_HUNT_MODE             0x10
#define     AUTO_ENABLES                0x20
#define     RX_5_BITS                   0x00
#define     RX_6_BITS                   0x80
#define     RX_7_BITS                   0x40
#define     RX_8_BITS                   0xC0

/*******************************************************************/
/*              8530 Write Register 4                              */
/*******************************************************************/
#define     PARITY_ENABLE            0x01
#define     PARITY_DISABLE           0x00
#define     PARITY_EVEN              0x02
#define     PARITY_ODD               0x00
#define     SYNC_MODE_ENABLE         0x00
#define     STOP_BIT_1               0x04
#define     STOP_BIT_1_5             0x08
#define     STOP_BIT_2               0x0C
#define     SYNC_8_BIT               0x00
#define     SYNC_16_BIT              0x10
#define     SDLC_MODE                0x20
#define     EXTERNAL_SYNC            0x30
#define     X1_CLOCK                 0x00
#define     X16_CLOCK                0x40
#define     X32_CLOCK                0x80
#define     X64_CLOCK                0xC0

/*******************************************************************/
/*              8530 Write Register 5                              */
/*******************************************************************/
#define     TX_CRC_ENABLE            0x01
#define     RTS_ON                   0x02
#define     CRC_16                   0x04
#define     CRC_SDLC                 0x00
#define     TX_ENABLE                0x08
#define     SEND_BREAK               0x10
#define     TX_5_BITS                0x00
#define     TX_6_BITS                0x40
#define     TX_7_BITS                0x20
#define     TX_8_BITS                0x60
#define     DTR_ON                   0x80

/*******************************************************************/
/*              8530 Write Register 6                              */
/*******************************************************************/
/* This register holds the sync character or SDLC address          */
/*    flag                                                         */
/*******************************************************************/

/*******************************************************************/
/*              8530 Write Register 7                              */
/*******************************************************************/
/* This register holds the sync character or SDLC flag             */
/*******************************************************************/

/*******************************************************************/
/*      8530 Write Register 7 Prime (ESCC and CMOS only)           */
/*******************************************************************/

#define     WR7_PRIME       16

#define     AUTO_TX_FLAG    0x01
#define     AUTO_EOM_RST    0x02
#define     AUTO_RTS_DEACT  0x04
#define     RX_FIFO_LEVEL   0x08
#define     DTR_REQ_TIMING  0x10
#define     TX_FIFO_LEVEL   0x20
#define     EX_READ_ENABLE  0x40

/*******************************************************************/
/*              8530 Write Register 8                              */
/*******************************************************************/
/* This register is the transmit buffer register                   */
/*******************************************************************/

/*******************************************************************/
/*              8530 Write Register 9                              */
/*******************************************************************/
#define     VIS             0x01
#define     NV              0x02
#define     DLC             0x04
#define     MI_ENABLE       0x08
#define     MI_DISABLE      0x00
#define     STATUS_HIGH     0x10
#define     SW_INT_ACK      0x20
#define     NO_RESET        0x00
#define     CH_B_RESET      0x40
#define     CH_A_RESET      0x80
#define     FORCE_HW_RESET  0xC0

/*******************************************************************/
/*              8530 Write Register 10                             */
/*******************************************************************/
#define     SYNC_6_BIT              0x01
#define     LOOP_MODE               0x02
#define     ABORT_ON_UNDERRUN       0x04
#define     MARK_IDLE               0x08
#define     GO_ACTIVE_ON_POLL       0x10
#define     NRZ                     0x00
#define     NRZI                    0x20
#define     FM1                     0x40
#define     FM0                     0x60
#define     CRC_PRESET_TO_ZERO      0x00
#define     CRC_PRESET_TO_ONE       0x80

/*******************************************************************/
/*              8530 Write Register 11                             */
/*******************************************************************/
#define     TRXC_OUT_XTAL           0x00
#define     TRXC_OUT_TX_CLOCK       0x01
#define     TRXC_OUT_BRG            0x02
#define     TRXC_OUT_DPLL_OUT       0x03
#define     TRXC_IS_OUTPUT          0x04
#define     TX_CLK_RTXC_PIN         0x00
#define     TX_CLK_TRXC_PIN         0x08
#define     TX_CLK_BRG              0x10
#define     TX_CLK_DPLL_OUT         0x18
#define     RX_CLK_RTXC_PIN         0x00
#define     RX_CLK_TRXC_PIN         0x20
#define     RX_CLK_BRG              0x40
#define     RX_CLK_DPLL_OUT         0x60

/*******************************************************************/
/*              8530 Write Register 12                             */
/*******************************************************************/
/* This Register sets the Lower Byte of Time Constant              */
/*******************************************************************/

/*******************************************************************/
/*              8530 Write Register 13                             */
/*******************************************************************/
/* This Register sets the Upper Byte of Time Constant              */
/*******************************************************************/

/*******************************************************************/
/*              8530 Write Register 14                             */
/*******************************************************************/
#define     BRG_ENABLE                      0x01
#define     BRG_DISABLE                     0x00
#define     BRG_SOURCE_PCLK                 0x02
#define     BRG_SOURCE_RTXC                 0x00
#define     REQUEST_FUNCTION                0x04
#define     AUTO_ECHO                       0x08
#define     LOCAL_LOOPBACK                  0x10
#define     DPLL_NULL                       0x00
#define     DPLL_ENTER_SEARCH_MODE          0x20
#define     DPLL_RESET_MISSING_CLOCK        0x40
#define     DPLL_DISABLE_DPLL               0x60
#define     DPLL_SOURCE_BRG                 0x80
#define     DPLL_SOURCE_RTXC                0xA0
#define     DPLL_FM_MODE                    0xC0
#define     DPLL_NRZI_MODE                  0xE0


/*******************************************************************/
/*              8530 Read Register 0                               */
/*******************************************************************/
#define     RX_CHAR_AVAIL       0x01
#define     ZERO_COUNT          0x02
#define     TX_BUFFER_EMPTY     0x04
#define     DCD_STATUS          0x08
#define     SYNC_HUNT           0x10
#define     CTS_STATUS          0x20
#define     TX_UNDERRUN_EOM     0x40
#define     BREAK_ABORT         0x80

/*******************************************************************/
/*              8530 Read Register 1                               */
/*******************************************************************/
#define     ALL_SENT            0x01
#define     RESIDUE_CODE_2      0x02
#define     RESIDUE_CODE_1      0x04
#define     RESIDUE_CODE_0      0x08
#define     PARITY_ERROR        0x10
#define     RX_OVERRUN_ERROR    0x20
#define     CRC_FRAMING_ERROR   0x40
#define     END_OF_FRAME        0x80
#define     ERROR_ANY           0xF0

/*******************************************************************/
/*              8530 Read Register 2                               */
/*******************************************************************/
/* This Register returns the Interrupt Status via CH B.            */
/*******************************************************************/

/*******************************************************************/
/*              8530 Read Register 3                               */
/*******************************************************************/
#define     CHB_EXT_STAT_IP     0x01
#define     CHB_TX_IP           0x02
#define     CHB_RX_IP           0x04
#define     CHA_EXT_STAT_IP     0x08
#define     CHA_TX_IP           0x10
#define     CHA_RX_IP           0x20

/*******************************************************************/
/*              8530 Read Register 10                              */
/*******************************************************************/
#define     ON_LOOP             0x02
#define     LOOP_SENDING        0x10
#define     TWO_CLOCKS_MISSING  0x40
#define     ONE_CLOCK_MISSING   0x80
/*******************************************************************/
/*              8530 Read Register 12                              */
/*******************************************************************/
/* This Register returns the Lower Byte of Time Constant           */
/*******************************************************************/

/*******************************************************************/
/*              8530 Read Register 13                              */
/*******************************************************************/
/* This Register returns the Upper Byte of Time Constant           */
/*******************************************************************/

/*******************************************************************/
/*              8530 Read and Write Register 15                    */
/*******************************************************************/
#define     WR7P_ACCESS         0x01
#define     NO_EXT_STAT_INT     0x00
#define     ZERO_COUNT_IE       0x02
#define     DCD_IE              0x08
#define     SYNC_HUNT_IE        0x10
#define     CTS_IE              0x20
#define     TX_UNDERRUN_IE      0x40
#define     BREAK_ABORT_IE      0x80

/*******************************************************************/
/*              Sealevel Custom Clock Control Register 6           */
/*******************************************************************/
#define     TXCIN               0x02
#define     TXCOUT              0x00

// ----------------------------------------------------------------------------
// Kernelspace structs
// ----------------------------------------------------------------------------
struct z85_port;

// Adapter (board) level information
struct z85_adapter {
	unsigned int driver_data;	// Note this must be at the top of PCI_DEV data
	unsigned int devid;
	struct device *dev;

	unsigned int numports;
	struct z85_port *ports;

	unsigned int irq;
	unsigned int isr_assigned;
};

// Port level information (wrapper around UART_PORT with HW specific info)
struct z85_port {
	struct uart_port port;
	unsigned int portnum;

	unsigned int iobase;

	// 16 base registers + 1 special 7' (prime) register R7' = 0x10?
	// remember that some of the values stored here are commands...
	unsigned char z85_reg[16 + 1];

	struct circ_buf rxbuf;
	unsigned char txactive;
	unsigned char rxactive;

	struct seamac_params params;

	struct z85_port *next;
	struct z85_adapter *sad;
};

// ----------------------------------------------------------------------------
// ESCC helper routines
// ----------------------------------------------------------------------------
static void 
zctrl_write(struct z85_port *port, unsigned char val, unsigned char reg)
{
	if (reg != WR7_PRIME)
		outb(reg, port->iobase + 1);
	else {
		zctrl_write(port, (port->z85_reg[15] | WR7P_ACCESS), 15);
		outb(7, port->iobase + 1);
	}

	outb(val, port->iobase + 1);
	port->z85_reg[reg] = val;

	if (reg == WR7_PRIME) {
		zctrl_write(port, (port->z85_reg[15] & ~WR7P_ACCESS), 15);
	}
}

static unsigned char 
zctrl_read(struct z85_port *port, unsigned char reg)
{
	if (reg > 15)
		return 0x00;

	outb(reg, port->iobase + 1);
	return inb(port->iobase + 1);
}

static inline void zdata_write(struct z85_port *port, unsigned char val)
{
	outb(val, port->iobase);
}

static inline unsigned char zdata_read(struct z85_port *port)
{
	return inb(port->iobase);
}

static inline unsigned int 
z85_brg_to_bps(struct z85_port *sp, unsigned short brg)
{
	unsigned int clk = sp->params.baseclk;

	if (sp->params.mode == SEAMAC_MODE_ASYNC)
		clk = clk / 16;

	return (clk / 2 / (brg + 2));
}
static inline unsigned short 
z85_bps_to_brg(struct z85_port *sp, unsigned int bps)
{
	unsigned int clk = sp->params.baseclk;

	if (sp->params.mode == SEAMAC_MODE_ASYNC)
		clk = clk / 16;

	if (bps == 0)
		return 0xFF;

	return ((clk / (2 * bps)) - 2);
}

// Actual Zilog HW config
static void z85_change_params(struct z85_port *sp)
{
	int i;
	unsigned char MIC = 0x00;
	struct seamac_params *params = &sp->params;

	DBGALL("change_params()\n");
	
	sp->port.uartclk = params->baseclk;

	// depending on the overall mode chosen, we need to force some params
	if (params->mode == SEAMAC_MODE_ASYNC) {
		DBGALL("ASYNC defaults\n");
		params->encoding = SEAMAC_ENCODE_NRZ;
		params->txclk = SEAMAC_CLK_BRG;
		params->rxclk = SEAMAC_CLK_BRG;
		params->rxclktype = SEAMAC_RXCLK_TTL;
		params->telement = SEAMAC_TELEMENT_BRG;
		params->addrrange = 0;
		params->crctype = SEAMAC_CRC_NONE;
	}
	else if (params->mode == SEAMAC_MODE_MONOSYNC || 
	    params->mode == SEAMAC_MODE_BISYNC || 
	    params->mode == SEAMAC_MODE_EXTERNAL ||
	    params->mode == SEAMAC_MODE_EXTERNAL_RTS) {
		DBGALL("MONO, BI, or EXT defaults\n");
		params->addrrange = 0;
		params->crctype = SEAMAC_CRC_NONE;
		params->underrun = SEAMAC_UNDERRUN_FLAG;
	}
	else if (params->mode == SEAMAC_MODE_SDLC) {
		DBGALL("SDLC defaults\n");
		params->syncflag = 0x007E;
		params->sixbitflag = 0;
		params->crctype = SEAMAC_CRC_CCITT;
	}

	// make the clocks work right for loopback mode
	if (params->loopback)
		params->rxclk = params->txclk;

	// init sequence: channel reset, select mode, all other options
	MIC = sp->z85_reg[9];  // save a copy of Master Interrupt Control reg
	zctrl_write(sp, CH_A_RESET, 9);
	mdelay(20);

	// reg 4: overall mode select
	if (params->mode == SEAMAC_MODE_ASYNC) {
		DBGALL("->ASYNC\n");
		sp->z85_reg[4] = X16_CLOCK | params->stopbits;
	}
	else if (params->mode == SEAMAC_MODE_MONOSYNC) {
		DBGALL("->MONO\n");
		sp->z85_reg[4] = X1_CLOCK | SYNC_8_BIT;
	}
	else if (params->mode == SEAMAC_MODE_BISYNC) {
		DBGALL("->BI\n");
		sp->z85_reg[4] = X1_CLOCK | SYNC_16_BIT;
	}
	else if (params->mode == SEAMAC_MODE_SDLC) {
		DBGALL("->SDLC\n");
		sp->z85_reg[4] = X1_CLOCK | SDLC_MODE;
	}
	else if (params->mode == SEAMAC_MODE_EXTERNAL) {
		DBGALL("->EXT\n");
		sp->z85_reg[4] = X1_CLOCK | EXTERNAL_SYNC;
	}
	else if (params->mode == SEAMAC_MODE_EXTERNAL_RTS) {
		DBGALL("->EXT(rts)\n");
		sp->z85_reg[4] = X1_CLOCK | EXTERNAL_SYNC;
	}
	sp->z85_reg[4] |= params->parity;

	zctrl_write(sp, sp->z85_reg[4], 4);

	// reg3: RX control/params
	sp->z85_reg[3] = (params->rxbits << 6);
	if (params->crctype != SEAMAC_CRC_NONE)
		sp->z85_reg[3] |= RX_CRC_ENABLE;
	if (params->addrfilter != 0xFF)
		sp->z85_reg[3] |= ADDRESS_SEARCH_MODE;
	if (params->addrrange)
		sp->z85_reg[3] |= SYNC_CHAR_LOAD_INHIBIT;

	zctrl_write(sp, sp->z85_reg[3], 3);

	// reg 5: TX control/params
	sp->z85_reg[5] = (params->txbits << 5);
	if (params->crctype != SEAMAC_CRC_NONE)
		sp->z85_reg[5] |= TX_CRC_ENABLE;

	zctrl_write(sp, sp->z85_reg[5], 5);

	// reg 6: sync characters or sdlc address field
	sp->z85_reg[6] = (params->syncflag & 0xFF);
	if (params->mode == SEAMAC_MODE_MONOSYNC && params->sixbitflag)
		sp->z85_reg[6] = (params->syncflag << 5) | 
						(params->syncflag & 0x3F);
	if (params->mode == SEAMAC_MODE_BISYNC && params->sixbitflag)
		sp->z85_reg[6] = ((params->syncflag << 4) | 0x0F);
	if (params->mode == SEAMAC_MODE_SDLC)
		sp->z85_reg[6] = params->addrfilter;
	
	zctrl_write(sp, sp->z85_reg[6], 6);

	// reg 7: sync characters or sdlc flag
	sp->z85_reg[7] = (params->syncflag & 0xFF);
	if (params->mode == SEAMAC_MODE_MONOSYNC && params->sixbitflag)
		sp->z85_reg[7] = (params->syncflag << 2);
	if (params->mode == SEAMAC_MODE_BISYNC)
		sp->z85_reg[7] = ((params->syncflag >> 8) & 0xFF);
	if (params->mode == SEAMAC_MODE_BISYNC && params->sixbitflag)
		sp->z85_reg[7] = ((params->syncflag >> 4) & 0xFF);

	zctrl_write(sp, sp->z85_reg[7], 7);

	// reg 10: Misc. TX/RX Control
	sp->z85_reg[10] = params->crcpreset | params->encoding;
	sp->z85_reg[10] |= params->underrun;
	if (params->sixbitflag)
		sp->z85_reg[10] |= 0x01;

	zctrl_write(sp, sp->z85_reg[10], 10);

	// Other options can now be set
	// preserve reg 1: TX/RX int, dma
	zctrl_write(sp, sp->z85_reg[1], 1);

	// reg 11: Clock mode control
	sp->z85_reg[11] = (params->txclk << 3) | (params->rxclk << 5);
	sp->z85_reg[11] |= params->rxclktype;
	if (params->txclk != SEAMAC_CLK_TXCLK)
		sp->z85_reg[11] |= params->telement | TRXC_IS_OUTPUT;

	zctrl_write(sp, sp->z85_reg[11], 11);
	// If TX clock is external, don't drive it
	outb((params->txclk == SEAMAC_CLK_TXCLK) ? TXCIN : TXCOUT, sp->iobase + 6);

	// reg 12: BRG lo, reg 13: BRG hi
	zctrl_write(sp, z85_bps_to_brg(sp, params->rate), 12);
	zctrl_write(sp, (z85_bps_to_brg(sp, params->rate) >> 8), 13);

	// reg 15: Ext status int control
	zctrl_write(sp, 0x00, 15);

	// reg 7PRIME CRC generator EOM reset
	if (params->mode == SEAMAC_MODE_SDLC)
		zctrl_write(sp, AUTO_EOM_RST, WR7_PRIME);

	// Enable TX FIFO Level interrupt (to allow 4B writes)
	zctrl_write(sp, sp->z85_reg[WR7_PRIME] | TX_FIFO_LEVEL, WR7_PRIME);

	// reg 14: BRG and DPLL control
	zctrl_write(sp, BRG_SOURCE_PCLK | DPLL_DISABLE_DPLL, 14);

	if (params->encoding == SEAMAC_ENCODE_NRZ || 
					params->encoding == SEAMAC_ENCODE_NRZI)
		zctrl_write(sp, BRG_SOURCE_PCLK | DPLL_NRZI_MODE, 14);
	else
		zctrl_write(sp, BRG_SOURCE_PCLK | DPLL_FM_MODE, 14);

	zctrl_write(sp, BRG_SOURCE_PCLK | DPLL_SOURCE_BRG, 14);
	zctrl_write(sp, BRG_SOURCE_PCLK | DPLL_ENTER_SEARCH_MODE, 14);
	zctrl_write(sp, BRG_ENABLE | BRG_SOURCE_PCLK, 14);

	// configure electrical interface
	if (sp->sad->devid == ISA_DEVICE_ID_SEALEVEL_3512 || sp->sad->devid == PCI_DEVICE_ID_SEALEVEL_5103)
		params->interface = SEAMAC_IF_HWSELECT;

	if (params->mode == SEAMAC_MODE_EXTERNAL)
		outb(((params->interface & 0x0F) | 0x10), sp->iobase + 5);
	else if (params->mode == SEAMAC_MODE_EXTERNAL_RTS)
		outb(((params->interface & 0x0F) | 0x20), sp->iobase + 5);
	else
		outb((params->interface & 0x0F), sp->iobase + 5);

	// 485 Echo disable
	if (params->interface & 0x40)
		outb(inb(sp->iobase + 5) | 0x40, sp->iobase + 5);

	// restore saved copy of Master Interrupt Control register
	zctrl_write(sp, MIC, 9);

	if (sp->rxactive)
		zctrl_write(sp, (sp->z85_reg[3] | RX_ENABLE), 3);
	if (sp->txactive)
		zctrl_write(sp, (sp->z85_reg[5] | TX_ENABLE), 5);

	DBGALL("Z85 REG DUMP: \n");
	for (i = 0; i < sizeof(sp->z85_reg); i++)
		DBGALL("  %2d:%02X\n", i, sp->z85_reg[i]);
}

// ----------------------------------------------------------------------------
// ISR routines
// ----------------------------------------------------------------------------

// Called by ISR whenever an interrupt is triggered indicating more chars avail
static unsigned int z85_rx_chars(struct z85_port *sp)
{
	struct tty_struct *tty = cpat_tty(sp->port);
	unsigned int flag = 0;
	unsigned char status, ch, rr1, eof = 0, crc = 0;
	struct circ_buf *rxbuf = &sp->rxbuf;

	DBGISR("start %s(%X:%i)\n", __FUNCTION__, sp->sad->devid, sp->portnum);
	status = zctrl_read(sp, 0);
	if (!status & RX_CHAR_AVAIL) {
		DBGISR("invalid rx_chars ISR called, quiting\n");
		return IRQ_HANDLED;
	}

	while (status & RX_CHAR_AVAIL) {
		ch = zdata_read(sp);
		DBGISR("received char %02X\n", ch);

		// The Zilog stuffs 1's into unused bits... mask them
		if (sp->params.rxbits == SEAMAC_BITS_5)
			ch &= 0x1F;
		if (sp->params.rxbits == SEAMAC_BITS_6)
			ch &= 0x3F;
		if (sp->params.rxbits == SEAMAC_BITS_7)
			ch &= 0x7F;

		flag = TTY_NORMAL;

		sp->port.icount.rx++;

		rr1 = zctrl_read(sp, 1);
		zctrl_write(sp, RESET_HIGHEST_IUS, 0);

		// Since SDLC mode is frame oriented, we need
		// to decide if we have reached the end of our frame yet
		if (sp->params.mode == SEAMAC_MODE_SDLC) {
			crc = (rr1 & CRC_FRAMING_ERROR);
			eof = (rr1 & END_OF_FRAME);
			DBGISR("crc error = %d\n", crc);

			rr1 &= ~(CRC_FRAMING_ERROR | END_OF_FRAME);
		}

		rr1 |= (status & BREAK_ABORT);   // Put BREAK_ABORT in rr1

		if (unlikely((rr1 & ERROR_ANY))) {
			DBGERR("Receive error occured\n");
			if (rr1 & BREAK_ABORT) {
				DBGERR("break abort\n");
				rr1 &= ~(PARITY_ERROR | CRC_FRAMING_ERROR);
				sp->port.icount.brk++;
				if (uart_handle_break(&sp->port))
					goto ignore_char;
			}
			else if (rr1 & PARITY_ERROR) {
				DBGERR("parity\n");
				sp->port.icount.parity++;
			}
			else if (rr1 & CRC_FRAMING_ERROR) {
				DBGERR("framing\n");
				sp->port.icount.frame++;
			}
			if (rr1 & RX_OVERRUN_ERROR) {
				DBGERR("overrun\n");
				sp->port.icount.overrun++;
			}

			rr1 &= sp->port.read_status_mask;

			if (rr1 & BREAK_ABORT)
				flag = TTY_BREAK;
			else if (rr1 & PARITY_ERROR)
				flag = TTY_PARITY;
			else if (rr1 & CRC_FRAMING_ERROR)
				flag = TTY_FRAME;
	
			zctrl_write(sp, ERROR_RESET, 0);
		}

		if (cpat_uart_handle_sysrq_char(&sp->port, ch))
			goto ignore_char;

		// Allow all other modes, except SDLC/EXT use standard tty ldisc
		if (sp->params.mode != SEAMAC_MODE_SDLC)
			uart_insert_char(&sp->port, rr1, RX_OVERRUN_ERROR, ch, 
									flag);
		else {
			if (!CIRC_SPACE(rxbuf->head, rxbuf->tail, PAGE_SIZE)) {
				eof = 1;
				sp->port.icount.buf_overrun++;
			}
			else {
				DBGISR("push %X->rxbuf[%i]\n", ch, rxbuf->head);
				rxbuf->buf[rxbuf->head] = ch;
				rxbuf->head += 1;
				rxbuf->head &= (PAGE_SIZE - 1);
			}
		}

		if (eof) 
			break;

	ignore_char:
		status = zctrl_read(sp, 0);
	}

	// Standard tty ldisc
	if (sp->params.mode != SEAMAC_MODE_SDLC) {
		spin_unlock(&sp->port.lock);
		cpat_tty_flip_buffer_push(sp, tty);
		spin_lock(&sp->port.lock);
	}
	else if (eof) {
		int count = CIRC_CNT(rxbuf->head, rxbuf->tail, PAGE_SIZE);
		if (crc) {
			DBGERR("pushing frame with crc error\n");
			sp->port.icount.frame++;
		}

		ldisc_receive_buf(tty, rxbuf->buf, NULL, count);
		rxbuf->head = rxbuf->tail = 0;

		zctrl_write(sp, RESET_RX_CRC, 0);
	}

	DBGISR("end %s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

// This is called whenever more room is available in the tx fifo... 
// WR7' has the tx buffer level set to interrupt on fifo empty, 
// so we can write 4B
static unsigned int z85_tx_chars(struct z85_port *sp)
{
	struct uart_port *port = &sp->port;
	struct circ_buf *xmit = cpat_xmit(sp->port);
	unsigned char ch;
	int txcount = 0;

	DBGISR("start %s(%X:%i)\n", __FUNCTION__, sp->sad->devid, sp->portnum);

	if (uart_tx_stopped(port)) {
		DBGISR("tx has been stopped\n");
		zctrl_write(sp, RESET_TX_INT_PEND, 0);
		return IRQ_HANDLED;
	}

	// If the FIFO is empty and the Queue is empty, we are done
	if (uart_circ_empty(xmit)) {
		zctrl_write(sp, RESET_TX_INT_PEND, 0);
		port->ops->stop_tx(port cpat_tty_start_stop_arg);
		return IRQ_HANDLED;
	}

	// Write up to 4 chars at a time
	while (!uart_circ_empty(xmit) && txcount < 4) {
		if (sp->port.x_char) {
			ch = sp->port.x_char;
			sp->port.x_char = 0;
		}
		else {
			ch = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		zdata_write(sp, ch);
		sp->port.icount.tx++;
		txcount++;
		DBGISR("writing char %02X to FIFO\n", ch);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sp->port);

	DBGISR("end %s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

static irqreturn_t z85_isr(int irq, void *dev_id)
{
	struct z85_adapter *sad = dev_id;
	struct z85_port *sp;
	unsigned int handled = 0;
	
	// Interrupts are shared across a single adapter (may be many channels)
	for (sp = sad->ports; sp != NULL; sp = sp->next) {
		unsigned char status;
		
		spin_lock(&sp->port.lock);

		status = zctrl_read(sp, 3);
		if (status & (CHA_TX_IP | CHA_RX_IP)) {
			DBGISR("IRQ %04X:%i\n", sad->devid, sp->portnum);
			zctrl_write(sp, RESET_HIGHEST_IUS, 0);

			if (status & CHA_TX_IP)
				handled = z85_tx_chars(sp);
			if (status & CHA_RX_IP)
				handled = z85_rx_chars(sp);
		}

		spin_unlock(&sp->port.lock);
	}

	if (handled)
		return handled;

	return IRQ_NONE;
}



// ----------------------------------------------------------------------------
// UART routines
// ----------------------------------------------------------------------------

// Is there room in the tx buffer?
static unsigned int z85_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned char status, active;
	struct z85_port *sp = (struct z85_port *) port;

	DBGUART("start %s\n", __FUNCTION__);

	spin_lock_irqsave(&port->lock, flags);
	status = zctrl_read(sp, 0);
	active = sp->txactive;
	spin_unlock_irqrestore(&port->lock, flags);

	if (!active && (status & TX_BUFFER_EMPTY)) {
		DBGUART("end %s (fifo empty)\n", __FUNCTION__);
		return TIOCSER_TEMT;
	}
	
	DBGUART("end %s (not empty)\n", __FUNCTION__);
	return 0;
}

// Read the current state of the modem control lines.  DSR is a little odd.
static unsigned int z85_get_mctrl(struct uart_port *port)
{
	unsigned int ret = 0;
	struct z85_port *sp = (struct z85_port *) port;
	unsigned char status = zctrl_read(sp, 0);

	DBGUART("start %s rr0=%X\n", __FUNCTION__, status);

	if (status & DCD_STATUS)
		ret |= TIOCM_CAR;
	if (status & CTS_STATUS)
		ret |= TIOCM_CTS;

	// DSR comes from a special port on the CPLD (BASE+4 bit D0)
	// except on the 5103, where it comes from Zilog chB DCD
	if (sp->sad->devid != PCI_DEVICE_ID_SEALEVEL_5103) {
		if (!(inb(sp->iobase + 4) & 0x01))
			ret |= TIOCM_DSR;
	}
	else {
		ret |= TIOCM_DSR;  // return as always active for now FIXME
		DBGUART("5103 detected, DSR unknown, returning active\n");
	}

	DBGUART("end %s (%08X)\n", __FUNCTION__, ret);
	return ret;
}

// Set modem control signals.  DTR is also a little oddball, so it will also
// be later added
static void z85_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct z85_port *sp = (struct z85_port *) port;
	unsigned char reg5 = (sp->z85_reg[5] & (~DTR_ON & ~RTS_ON));

	DBGUART("start %s (%8X)\n", __FUNCTION__, mctrl);
	if (mctrl & TIOCM_RTS)
		reg5 |= RTS_ON;
	if (mctrl & TIOCM_DTR)
		reg5 |= DTR_ON;

	// The 3512 handles DTR from within the CPLD (BASE + 4) bit D0
	if (sp->sad->devid == ISA_DEVICE_ID_SEALEVEL_3512)
		outb((mctrl & TIOCM_DTR)?0x01:0x00, sp->iobase + 4);

	zctrl_write(sp, reg5, 5);

	DBGUART("end %s reg5=%X\n", __FUNCTION__, reg5);
}

// Quit transmitting as soon as possible.
static void z85_stop_tx(struct uart_port *port cpat_tty_start_stop_param)
{
	struct z85_port *sp = (struct z85_port *) port;

	DBGUART("start %s\n", __FUNCTION__);

	// Disable Transmit Interrupts to stop the tx engine
	zctrl_write(sp, (sp->z85_reg[1] & ~TX_INT_ENABLE), 1);

	// TX Post Delay (ms) max delay == short.Max == 65535 ms
	if (sp->params.posttxdelay > 0)
		mdelay(sp->params.posttxdelay);

	// If RTS_TOGGLE is enabled, set the RTS output state
	if (sp->params.rtscontrol == SEAMAC_RTSCONTROL_TOGGLE) {
		unsigned int mctrl = z85_get_mctrl(port);

		DBGUART("rtscontrol toggling... enable\n");
		z85_set_mctrl(port, mctrl & ~TIOCM_RTS);
	}
	// Trick the transmitter into idling ones, instead of flags
	if (sp->params.mode != SEAMAC_MODE_SDLC 
			&& sp->params.idlemode == SEAMAC_IDLE_ONES)
	zctrl_write(sp, (sp->z85_reg[5] & ~TX_ENABLE), 5);

	sp->txactive = 0;
	DBGUART("end %s\n", __FUNCTION__);
}

// Start transmitting any available characters
static void z85_start_tx(struct uart_port *port cpat_tty_start_stop_param)
{
	struct z85_port *sp = (struct z85_port *) port;
	unsigned char status;

	DBGUART("start %s\n", __FUNCTION__);

	status = zctrl_read(sp, 0);

	// TX busy?  Just wait for the TX done interrupt.
	if (sp->txactive) {
		DBGUART("transmit is active, just wait for next ISR\n");
		return;
	}

	zctrl_write(sp, RESET_TX_CRC, 0);

	// Enable Transmit Interrupts to start the tx engine 
	zctrl_write(sp, (sp->z85_reg[5] | TX_ENABLE), 5);
	zctrl_write(sp, (sp->z85_reg[1] | TX_INT_ENABLE), 1);
	sp->txactive = 1;
	
	// If RTS_TOGGLE is enabled, set the RTS output state
	if (sp->params.rtscontrol == SEAMAC_RTSCONTROL_TOGGLE) {
		unsigned int mctrl = z85_get_mctrl(port);

		DBGUART("rtscontrol toggling... enable\n");
		z85_set_mctrl(port, mctrl | TIOCM_RTS);
	}

	// TX Pre Delay (ms) max delay == short.Max == 65535 ms
	if (sp->params.pretxdelay > 0)
		mdelay(sp->params.pretxdelay);

	// Go ahead and start the tx if there are any chars ready
	z85_tx_chars(sp);
	
	DBGUART("end %s\n", __FUNCTION__);
}

// Cease receiving
static void z85_stop_rx(struct uart_port *port)
{
	struct z85_port *sp = (struct z85_port *) port;

	DBGUART("start %s\n", __FUNCTION__);
	sp->rxactive = 0;
	zctrl_write(sp, (sp->z85_reg[1] & ~RX_INT_SP_COND), 1);
	DBGUART("end %s\n", __FUNCTION__);
}

// Enable external modem status interrupts - We use polling for the modem
// status, so there's nothing for this function to do
static void z85_enable_ms(struct uart_port *port)
{
}

// Start/stop sending line break
static void z85_break_ctl(struct uart_port *port, int break_state)
{
	struct z85_port *sp = (struct z85_port *) port;
	unsigned long flags;

	DBGUART("start %s(%i)\n", __FUNCTION__, break_state);
	spin_lock_irqsave(&port->lock, flags);

	// Told to send break && not currently doing so
	if ((break_state) && ~(sp->z85_reg[5] & SEND_BREAK))
		zctrl_write(sp, (sp->z85_reg[5] | SEND_BREAK), 5);

	// Told to stop sending break && currently are
	else if (~(break_state) && (sp->z85_reg[5] & SEND_BREAK))
		zctrl_write(sp, (sp->z85_reg[5] & ~SEND_BREAK), 5);

	spin_unlock_irqrestore(&port->lock, flags);
	DBGUART("end %s\n", __FUNCTION__);
}

// Startup the device (really just start interrupts)
static int z85_startup(struct uart_port *port)
{
	unsigned long flags;
	struct z85_port *sp = (struct z85_port *) port;
	struct z85_adapter *sad = sp->sad;

	DBGUART("start %s\n", __FUNCTION__);

	DBGUART("alloc rxbuf\n");
	if (!sp->rxbuf.buf) {
		unsigned long page = get_zeroed_page(GFP_KERNEL);
		if (!page) {
			DBGERR("not enough memory to allocate rx buffer\n");
			return -EBUSY;
		}

		sp->rxbuf.buf = (unsigned char *) page;
		sp->rxbuf.head = sp->rxbuf.tail = 0;
	}
	DBGUART("alloc rxbuf done\n");

	// Only init ISR once per adapter
	if (!(sad->isr_assigned++))
		request_irq(sad->irq, &z85_isr, IRQF_SHARED, "seamac", sad);

	spin_lock_irqsave(&port->lock, flags);

	// Update settings
	z85_change_params(sp);

	// Configure what generates an interrupt
	zctrl_write(sp, RX_INT_ALL_CHAR_SP_COND, 1);
	zctrl_write(sp, (sp->z85_reg[1] & ~EXT_INT_ENABLE), 1);
	zctrl_write(sp, (sp->z85_reg[15] & ~(DCD_IE|CTS_IE|SYNC_HUNT_IE)), 15);
	zctrl_write(sp, RESET_EXT_STATUS_INT, 0);

	zctrl_write(sp, ERROR_RESET, 0);
	zctrl_write(sp, RESET_RX_CRC, 0);
	zctrl_write(sp, RESET_HIGHEST_IUS, 0);

	zctrl_write(sp, MI_ENABLE, 9);	// Z8X530 MIE
	zctrl_write(sp, (sp->z85_reg[3] | RX_ENABLE), 3);

	sp->txactive = 0;
	sp->rxactive = 1;

	spin_unlock_irqrestore(&port->lock, flags);
	DBGUART("end %s\n", __FUNCTION__);

	return 0;
}

// Shutdown port ISR so that PCMCIA can be removed while device not active
static void z85_shutdown(struct uart_port *port)
{
	unsigned int irqcount;
	unsigned long flags;
	struct z85_port *sp = (struct z85_port *) port;
	struct z85_adapter *sad = sp->sad;

	DBGUART("start %s\n", __FUNCTION__);

	spin_lock_irqsave(&port->lock, flags);
	zctrl_write(sp, (sp->z85_reg[3] & ~RX_ENABLE), 3);
	zctrl_write(sp, (sp->z85_reg[5] & ~TX_ENABLE), 5);
	zctrl_write(sp, (sp->z85_reg[1] & ~EXT_INT_ENABLE), 1);
	zctrl_write(sp, (sp->z85_reg[15] & ~(DCD_IE|CTS_IE|SYNC_HUNT_IE)), 15);
	zctrl_write(sp, MI_DISABLE, 9);

	sad->isr_assigned--;
	irqcount = sad->isr_assigned;

	spin_unlock_irqrestore(&port->lock, flags);

	// Only close if there are no more ports using this interrupt
	if (irqcount == 0)
		free_irq(sad->irq, sad);

	DBGUART("dealloc rxbuf\n");
	if (sp->rxbuf.buf) {
		free_page((unsigned long)sp->rxbuf.buf);
		sp->rxbuf.buf = NULL;
		sp->rxbuf.head = sp->rxbuf.tail = 0;
	}
	DBGUART("rxbuf gone\n");

	DBGUART("end %s\n", __FUNCTION__);
}

// Configure port based on termios settings (If not in async will only change
// *_status_mask flags.  This includes baud rate.  Use ioctl instead).
static void
z85_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	struct z85_port *sp = (struct z85_port *) port;
	unsigned long flags;
	unsigned int baud;

	DBGUART("start %s\n", __FUNCTION__);

	// Status control things are allowed to happen for all modes...
	// except maybe framing errors need to be stopped for SYNC?
	port->read_status_mask = RX_OVERRUN_ERROR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= CRC_FRAMING_ERROR | PARITY_ERROR;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= BREAK_ABORT;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= CRC_FRAMING_ERROR | PARITY_ERROR;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BREAK_ABORT;
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= RX_OVERRUN_ERROR;
	}
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask = 0xff;

	if (sp->params.mode != SEAMAC_MODE_ASYNC) {
		z85_change_params(sp);
		DBGUART("end %s (SYNC)\n", __FUNCTION__);
		return;
	}

	// ASYNC only things....
	spin_lock_irqsave(&port->lock, flags);

	baud = uart_get_baud_rate(port, termios, old, 300, 115200);
	uart_update_timeout(port, termios->c_cflag, baud);

	sp->params.rate = baud;

	// Determine number of bits in a char for rx/tx... both same for async
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		sp->params.rxbits = sp->params.txbits = SEAMAC_BITS_5;
		break;
	case CS6:
		sp->params.rxbits = sp->params.txbits = SEAMAC_BITS_6;
		break;
	case CS7:
		sp->params.rxbits = sp->params.txbits = SEAMAC_BITS_7;
		break;
	case CS8:
	default:
		sp->params.rxbits = sp->params.txbits = SEAMAC_BITS_8;
		break;
	};
	
	if (termios->c_cflag & CSTOPB)
		sp->params.stopbits = SEAMAC_STOP_2;
	else
		sp->params.stopbits = SEAMAC_STOP_1;

	if ((termios->c_cflag & PARENB) && (termios->c_cflag & PARODD))
		sp->params.parity = SEAMAC_PARITY_ODD;
	else if ((termios->c_cflag & PARENB) && ~(termios->c_cflag & PARODD))
		sp->params.parity = SEAMAC_PARITY_EVEN;
	else
		sp->params.parity = SEAMAC_PARITY_NONE;

	z85_change_params(sp);
	spin_unlock_irqrestore(&port->lock, flags);
	DBGUART("end %s (ASYNC)\n", __FUNCTION__);
}

// Return an ID string of port type - modifying the string may be a bad idea,
// all the other serial drivers just use constant strings... (works well tho..)
static char z85_port_id_string[] = "FFFF port XXX (Z85230)                 ";
static const char *z85_type(struct uart_port *port)
{
	struct z85_port *sp = (struct z85_port *) port;
	unsigned short device = sp->sad->devid;
	int portnum = sp->portnum;

	sprintf(z85_port_id_string, "%X port %03i (Z85230)", device, portnum);
	return z85_port_id_string;
}

static void z85_release_port(struct uart_port *port)
{
}

static int z85_request_port(struct uart_port *port)
{
	return 0;
}

// These do not need to do anything interesting either.
static void z85_config_port(struct uart_port *port, int flags)
{

}

static int z85_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return -EINVAL;
}

// Port specific ioctls (configuration)
static int z85_get_params(struct z85_port *sp, void __user *to)
{
	void *from = (void *) &sp->params;

	if (copy_to_user(to, from, sizeof(struct seamac_params)))
		return -EFAULT;

	return 0;
}

// Allow user to set port configuration structure
static int z85_set_params(struct z85_port *sp, void __user *from)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	void *to = (void *) &sp->params;

	if (copy_from_user(to, from, sizeof(struct seamac_params)))
		return -EFAULT;

	spin_lock_irqsave(&port->lock, flags);
	z85_change_params(sp);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;	
}

// Allow user to enter SYNC HUNT in Monosync and Bisync modes
static int z85_enter_hunt(struct z85_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	if (sp->params.mode == SEAMAC_MODE_MONOSYNC || 
					sp->params.mode == SEAMAC_MODE_BISYNC ||
					sp->params.mode == SEAMAC_MODE_EXTERNAL ||
					sp->params.mode == SEAMAC_MODE_EXTERNAL_RTS)
	{
		if (sp->rxbuf.buf != NULL)
			zctrl_write(sp, (sp->z85_reg[3] | ENTER_HUNT_MODE), 3);
	}
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

// Check if transmission is active
static int z85_get_txactive(struct z85_port *sp, void __user *to)
{
	int active = z85_tx_empty(&sp->port) != TIOCSER_TEMT;
	return copy_to_user(to, &active, sizeof(active));
}

// A custom method to retrieve modem control signals (and others) all in one bitmask
static int z85_get_signals(struct z85_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int signals = 0;
	void *from = (void *) &signals;
	unsigned char status;

	spin_lock_irqsave(&port->lock, flags);

 	status = zctrl_read(sp, 0);

	if (status & DCD_STATUS)
		signals |= SEAMAC_DCD;
	if (status & CTS_STATUS)
		signals |= SEAMAC_CTS;

	// DSR comes from a special port on the CPLD (BASE+4 bit D0)
	// except on the 5103, where it comes from Zilog chB DCD
	if (sp->sad->devid != PCI_DEVICE_ID_SEALEVEL_5103) {
		if (!(inb(sp->iobase + 4) & 0x01))
			signals |= SEAMAC_DSR;
	}
	else {
		signals |= SEAMAC_DSR;  // return as always active for now FIXME
		DBGUART("5103 detected, DSR unknown, returning active\n");
	}

	// It is a pain to read the values of register 5, so just use the last write
	if (sp->z85_reg[5] & RTS_ON)
		signals |= SEAMAC_RTS;
	if (sp->z85_reg[5] & DTR_ON)
		signals |= SEAMAC_DTR;

	// LL & RL are controlled by the CPLD (BASE+6 D2 and D3) neg logic
	if ((inb(sp->iobase + 6) & 0x04) == 0)
		signals |= SEAMAC_LL;
	if ((inb(sp->iobase + 6) & 0x08) == 0)
		signals |= SEAMAC_RL;

	spin_unlock_irqrestore(&port->lock, flags);

	if (copy_to_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	return 0;
}

// A custom method to set all of the output modem control (and other) signals
static int z85_set_signals(struct z85_port *sp, void __user *from)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int signals = 0;
	void *to = (void *) &signals;

	if (copy_from_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	spin_lock_irqsave(&port->lock, flags);

	// RTS and DTR are set in register 5 of the Zilog
	if (signals & SEAMAC_RTS)
		zctrl_write(sp, sp->z85_reg[5] | RTS_ON, 5);
	else
		zctrl_write(sp, sp->z85_reg[5] & ~RTS_ON, 5);
	if (signals & SEAMAC_DTR)
		zctrl_write(sp, sp->z85_reg[5] | DTR_ON, 5);
	else
		zctrl_write(sp, sp->z85_reg[5] & ~DTR_ON, 5);

	// LL & RL are controlled by the CPLD (BASE+6 D2 and D3) neg logic
	if (signals & SEAMAC_LL)
		outb(inb(sp->iobase + 6) & ~0x04, sp->iobase + 6);
	else
		outb(inb(sp->iobase + 6) | 0x04, sp->iobase + 6);
	if (signals & SEAMAC_RL)
		outb(inb(sp->iobase + 6) & ~0x08, sp->iobase + 6);
	else
		outb(inb(sp->iobase + 6) | 0x08, sp->iobase + 6);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;	
}

// Retrieve the per-port factory programmed security code
static int z85_get_security(struct z85_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int security = 0;
	void *from = (void *) &security;

	spin_lock_irqsave(&port->lock, flags);

	if (sp->sad->devid == PCMCIA_DEVICE_ID_SEALEVEL_3612)
		security = inb(sp->iobase + 7);
	else 
		security = inb(sp->iobase + 14) + (inb(sp->iobase + 15) << 8);

	spin_unlock_irqrestore(&port->lock, flags);

	if (copy_to_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	return 0;
}

static int 
z85_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct z85_port *sp = (struct z85_port *) port;

	switch (cmd) {
	case SEAMAC_IOCTL_GPARAMS:
		return z85_get_params(sp, (void __user *) arg);
	case SEAMAC_IOCTL_SPARAMS:
		return z85_set_params(sp, (void __user *) arg);
	case SEAMAC_IOCTL_HUNT:
		return z85_enter_hunt(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GTXACTIVE:
		return z85_get_txactive(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GSIGNALS:
		return z85_get_signals(sp, (void __user *) arg);
	case SEAMAC_IOCTL_SSIGNALS:
		return z85_set_signals(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GSECURITY:
		return z85_get_security(sp, (void __user *) arg);
	default:
		return -ENOIOCTLCMD;
	}
}

static struct uart_ops z85_pops = {
	.tx_empty	=	z85_tx_empty,
	.set_mctrl	=	z85_set_mctrl,
	.get_mctrl	=	z85_get_mctrl,
	.stop_tx	=	z85_stop_tx,
	.start_tx	=	z85_start_tx,
	.stop_rx	=	z85_stop_rx,
	.enable_ms	=	z85_enable_ms,
	.break_ctl	=	z85_break_ctl,
	.startup	=	z85_startup,
	.shutdown	=	z85_shutdown,
	.set_termios	=	z85_set_termios,
	.type		=	z85_type,
	.release_port	=	z85_release_port,
	.request_port	=	z85_request_port,
	.config_port	=	z85_config_port,
	.verify_port	=	z85_verify_port,
	.ioctl 		=	z85_ioctl,
};

// -----------------------------------------------------------------------------
// Move to startup/shutdown maybe?
// -----------------------------------------------------------------------------

static void z85_hw_init(struct z85_adapter *sad)
{
	struct z85_port temp;
	struct z85_port *sp_cha, *sp_chb = &temp;

	for (sp_cha = sad->ports; sp_cha != NULL; sp_cha = sp_cha->next) {
		// Reset the port first of all
		zctrl_write(sp_cha, FORCE_HW_RESET, 9);
		zctrl_write(sp_cha, CH_B_RESET, 9);

		// Configure port per default parameters
		z85_change_params(sp_cha);

		// Clean SCC FIFO, just a precaution
		while (RX_CHAR_AVAIL & zctrl_read(sp_cha, 0))
			zdata_read(sp_cha);

		// No interrupts yet, please (wait until the port is in use)
		zctrl_write(sp_cha, MI_DISABLE, 9);
		
		// This is some channel B magic, just trying to keep it quiet
		sp_chb->iobase = sp_cha->iobase + 2;
		zctrl_write(sp_chb, INT_DISABLE, 1);
	}
}

static void z85_hw_deinit(struct z85_adapter *sad)
{
	struct z85_port *sp;

	for (sp = sad->ports; sp != NULL; sp = sp->next) {
		// No more interrupts, if we somehow still have them enabled...
		zctrl_write(sp, MI_DISABLE, 9);

		// The final state of the card should be disabled
		sp->params.interface = SEAMAC_IF_OFF;
		z85_change_params(sp);
	}
}


// ----------------------------------------------------------------------------
// Device info structure allocation & initialization / deallocation.
// ----------------------------------------------------------------------------

// Request memory for port information.  Also set some initial values.
static struct z85_port 
*z85_alloc_port(struct z85_adapter *sad, int portnum, unsigned int iobase, 
				unsigned int clk, struct z85_port *next)
{
	struct z85_port *sp;

	sp = kzalloc(sizeof(*sp), GFP_KERNEL);
	if (sp == NULL)
		return NULL;

	sp->port.iobase = iobase;
	sp->port.iotype = UPIO_PORT;
	sp->port.irq = sad->irq;
	sp->port.uartclk = clk;
	sp->port.fifosize = 4;
	sp->port.ops = &z85_pops;
	sp->port.type = PORT_SEALEVEL_SYNC;
	sp->port.flags = 0;
	sp->port.dev = sad->dev;
	
	// Where are the Zilog windows?
	sp->iobase = iobase;
	sp->portnum = portnum;
	sp->sad = sad;

	// Make sure rxbuf head/tail are clear (note *buf is alloc'd on request)
	sp->rxbuf.buf = NULL;
	sp->rxbuf.head = sp->rxbuf.tail = 0;

	memcpy(&sp->params, &seamac_defaults, sizeof(struct seamac_params));
	sp->params.baseclk = clk;

	sp->next = next;

	return sp;
}

// Free a port information struct. Also clearing any values previously set.
static void z85_dealloc_port(struct z85_port *sp)
{
	sp->port.iobase = 0;
	sp->port.iotype = 0;
	sp->port.irq = 0;
	sp->port.uartclk = 0;
	sp->port.fifosize = 0;
	sp->port.ops = NULL;
	sp->port.type = 0;
	sp->port.flags = 0;
	sp->port.line = 0;
	sp->port.dev = NULL;
	
	// Where are the Zilog windows?
	sp->iobase = 0;
	sp->portnum = 0;

	sp->next = NULL;
	sp->sad = NULL;
	
	sp->rxbuf.buf = NULL;
	sp->rxbuf.head = sp->rxbuf.tail = 0;

	kfree(sp);
}

// Free any space used by a z85_adapter (note the list of ports will be
// deallocated through the z85_port_dealloc function).  Cleanup members too.
static void z85_dealloc_adapter(struct z85_adapter *sad)
{
	sad->devid = 0;
	sad->numports = 0;
	sad->irq = 0;
	sad->isr_assigned = 0;

	// Walk down the list deallocating any ports attached to this adapter
	while (sad->ports != NULL) {
		struct z85_port *temp = sad->ports;
		sad->ports = sad->ports->next;
		z85_dealloc_port(temp);
	}

	kfree(sad);
}

// Allocate an adapter information struct.  This function, in turn, allocates
// information structs for each port for a given adapter.
static struct z85_adapter 
*z85_alloc_adapter(struct device *dev, unsigned int devid, int numports,
					unsigned int *iobase, unsigned int irq, unsigned int clk)
{
	int i;
	struct z85_adapter *sad = kzalloc(sizeof(*sad), GFP_KERNEL);

	if (sad == NULL)
		return NULL;

	sad->dev = dev;
	sad->devid = devid;
	sad->numports = numports;
	sad->irq = irq;
	sad->isr_assigned = 0;
	sad->ports = NULL;

	for (i = (numports - 1); i >= 0; i--) {
		struct z85_port *temp = sad->ports;
		sad->ports = z85_alloc_port(sad, i, iobase[i], clk, temp);
		
		// Was there enough mem?  If not, dealloc it all
		if (sad->ports == NULL) {
			DBGERR("Adapter allocation failed!\n");
			sad->ports = temp;
			z85_dealloc_adapter(sad);
			return NULL;
		}
	} 

	return sad;
}

static void z85_deregister_ports(struct z85_adapter *sad)
{
	struct z85_port *sp;

	for (sp = sad->ports; sp != NULL; sp = sp->next)
		seamac_deregister_port(&sp->port);
}

static int z85_register_ports(struct z85_adapter *sad)
{
	struct z85_port *sp;
	int rc;
	
	for (sp = sad->ports; sp != NULL; sp = sp->next) {
		rc = seamac_register_port(&sp->port);
				
		// If one of the ports fails, take them all back
		if (rc) {
			z85_deregister_ports(sad);
			return rc;
		}
	}

	return 0;
}


// -----------------------------------------------------------------------------
// PCI device detection functions.
// -----------------------------------------------------------------------------
int __devinit z85_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct z85_adapter *sad;
	int rc = 0, numports = 1, i;
	unsigned int iobase[SEALEVEL_MAX_PORTS_PER_ADAPTER];

	rc = pci_request_regions(dev, "seamac");
	if(rc) {
		DBGERR("I/O regions already in use!\n");
		goto fail_regions;
	}

	// Alloc and initialize adapter/port structs
	if (id->driver_data & SEALEVEL_SYNC_QUADPORT)
		numports = 4;
	else if (id->driver_data & SEALEVEL_SYNC_DUALPORT)
		numports = 2;
	else
		numports = 1;

	if (id->driver_data & SEALEVEL_SYNC_Z85230) {
		for (i = 0; i < numports; i++)
			iobase[i] = pci_resource_start(dev, (2 + i));
	}
	else if (id->driver_data & SEALEVEL_SYNC_Z85230_IOSHARED)
	{
		// BAR register mapping is different from standard PCI products...
		for (i = 0; i < numports; i++)
			iobase[i] = pci_resource_start(dev, 2) + (16 * i);
	}
	else {
		DBGERR("I/O Region mapping unknown!\n");
		goto fail_regions;
	}

	// The Bridge IC needs to have interrupts enabled
	if (id->driver_data & SEALEVEL_SYNC_Z85230_INTENABLE) {
		void __iomem *addr = pci_iomap(dev, 0, pci_resource_len(dev, 0));

		if (addr) {
			u32 val = ioread32(addr + 0x68);
			iowrite32(val | 0x00000800, addr + 0x68);
			pci_iounmap(dev, addr);
		}
	}

	sad = z85_alloc_adapter(&dev->dev, dev->device, numports, iobase, dev->irq, SEAMAC_PCI_CLK);
	if (sad == NULL) {
		DBGERR("no free memory!\n");
		rc = -ENOMEM;
		goto fail_alloc;
	}
	
	// Initialize the ports
	z85_hw_init(sad);

	// Register the ports
	rc = z85_register_ports(sad);
	if (rc) {
		DBGERR("Port registration failed!\n");
		goto fail_register;
	}

	pci_set_drvdata(dev, sad);

	return 0;

 fail_register:
	z85_dealloc_adapter(sad);
 fail_alloc:
	pci_release_regions(dev);
 fail_regions:
	return rc;
}

// A PCI Seamac device is being removed
void __devexit z85_pci_remove(struct pci_dev *dev)
{
	struct z85_adapter *sad = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);

	z85_deregister_ports(sad);

	if (sad->isr_assigned)
		free_irq(dev->irq, sad);

	z85_hw_deinit(sad);
	z85_dealloc_adapter(sad);

	pci_release_regions(dev);
}


// -----------------------------------------------------------------------------
// PCMCIA device detection.  (Optional)
// -----------------------------------------------------------------------------
#ifdef PCMCIA_SUPPORT

// Callback for card insertion.
static int __devinit z85_pcmcia_attach(struct pcmcia_device *dev)
{
	int rc = 0;
	struct z85_adapter *sad;

	// Allocate and initialize adapter info struct
	sad = z85_alloc_adapter(&dev->dev, SEALEVEL_SYNC_Z85230_PCMCIA, 1, 
				   &dev->resource[0]->start, dev->irq, SEAMAC_PCMCIA_CLK);
	dev->priv = (void*) sad;
	if (sad == NULL) {
		DBGERR("no free memory!\n");
		return -ENOMEM;
	}

	// Initialize the ports
	z85_hw_init(sad);

	// Register the ports
	rc = z85_register_ports(sad);
	if (rc) {
		DBGERR("Port registration failed!\n");
	}

	return rc;
}

// Callback for card ejection.
static void z85_pcmcia_detach(struct pcmcia_device *dev)
{
	struct z85_adapter *sad = dev->priv;
	dev->priv = NULL;

	z85_deregister_ports(sad);
	z85_dealloc_adapter(sad);
}
#endif	//PCMCIA_SUPPORT


// -----------------------------------------------------------------------------
// ISA device specification. (ISA isn't plug and play, we must be told about it)
// This can also be optionally disabled.
// -----------------------------------------------------------------------------
#ifdef ISA_SUPPORT

static struct z85_adapter *z85_isa_adapter[SEALEVEL_MAX_ISA];

int z85_isa_attach(int isa_io, int isa_irq)
{
	struct z85_adapter *sad;

	request_region(isa_io, 8, "seamac");
	sad = z85_alloc_adapter(NULL, ISA_DEVICE_ID_SEALEVEL_3512, 1, &isa_io, 
				isa_irq, SEAMAC_ISA_CLK);

	if (sad == NULL) {
		DBGERR("no free memory!\n");
		return -ENOMEM;
	}
	
	// Initialize the ports
	z85_hw_init(sad);

	// Register the ports
	if (z85_register_ports(sad)) {
		DBGERR("Port registration failed!\n");
		z85_dealloc_adapter(sad);
	}

	z85_isa_adapter[0] = sad;

	return 0;
}

void z85_isa_detach(int index)
{
	struct z85_adapter *sad = z85_isa_adapter[0];
	z85_isa_adapter[0] = NULL;

	if (sad) {
		z85_deregister_ports(sad);
		release_region(sad->ports[0].port.iobase, 8);

		// Unhook the ISR, if necessary
		if (sad->isr_assigned)
			free_irq(sad->irq, sad);

		z85_dealloc_adapter(sad);
	}
}
#endif	//ISA_SUPPORT

