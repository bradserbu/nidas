/*
 *	Sealevel Systems Custom Bi-Synchronous IP core driver implementation
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
 *	(c) Copyright 2013 Sealevel Systems, Inc.
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
#include "ssi9198.h"


// Memory map structure of 9198 IP
#define SSI_9198_REG_ID		0x00
#define SSI_9198_REG_REV	0x04
#define SSI_9198_REG_HWDBG	0x0C
#define SSI_9198_REG_CONTROL	0x14
#define SSI_9198_REG_CONTROL_RESET	0x00010000

#define SSI_9198_REG_TX8	0x20
#define SSI_9198_REG_TX32	0x24
#define SSI_9198_REG_TXFIFO	0x28
#define SSI_9198_REG_IDLE	0x2C
#define SSI_9198_REG_TXRATE	0x30
#define SSI_9198_REG_TXCTL	0x38
#define SSI_9198_REG_TXCTL_MCDISFILTER	0x10000000
#define SSI_9198_REG_TXCTL_MCSYNC	0x01000000
#define SSI_9198_REG_TXCTL_TXCLKEDGE	0x00100000
#define SSI_9198_REG_TXCTL_HSENABLE	0x00010000
#define SSI_9198_REG_TXCTL_TXEN		0x00000100
#define SSI_9198_REG_TXCTL_TXRST	0x00000001
#define SSI_9198_REG_TXCLKDLY	0x3C

#define SSI_9198_REG_RX8	0x50
#define SSI_9198_REG_RX32	0x54
#define SSI_9198_REG_RXFIFO	0x58
#define SSI_9198_REG_RXRATE	0x5C
#define SSI_9198_REG_RXCTL	0x64
#define SSI_9198_REG_RXCTL_RXCLKEDGE	0x00100000
#define SSI_9198_REG_RXCTL_RXEN		0x00000100
#define SSI_9198_REG_RXCTL_RXRST	0x00000001
#define SSI_9198_REG_RXCLKDLY	0x68

#define SSI_9198_REG_MCOUT	0x34
#define SSI_9198_REG_MCOUT_RTS		0x00000001
#define SSI_9198_REG_MCIN	0x60
#define SSI_9198_REG_MCIN_CTS		0x00010000
#define SSI_9198_REG_MCIN_DSR		0x00000100
#define SSI_9198_REG_MCIN_CD		0x00000001

#define SSI_9198_REG_ISR	0x80
#define SSI_9198_REG_IMR	0x84
#define SSI_9198_REG_MIR	0x88
#define SSI_9198_REG_ICR	0x8C
#define SSI_9198_REG_ITXFIFO	0x90
#define SSI_9198_REG_IRQ_TXFIFO		0x00020000
#define SSI_9198_REG_IRQ_TXEMT		0x00010000
#define SSI_9198_REG_IRQ_CTS		0x00000008
#define SSI_9198_REG_IRQ_DSR		0x00000004
#define SSI_9198_REG_IRQ_CD		0x00000002
#define SSI_9198_REG_IRQ_RXEOF		0x00000001


// ----------------------------------------------------------------------------
// Kernelspace structs
// ----------------------------------------------------------------------------

// For now there will only ever be 1 port on a single device.
// may want an adapter abstraction structure later...
struct ssi9198_port {
	struct uart_port port;

	unsigned int devid;
	struct device *dev;

	void __iomem *mem;
	unsigned int irq;

	struct circ_buf rxbuf;
	unsigned char txactive;
	unsigned char rxactive;

	struct seamac_params params;
};


// ----------------------------------------------------------------------------
// Register mapping helpers
// ----------------------------------------------------------------------------
static inline u32 ssi9198_read(struct ssi9198_port *sp, u8 reg)
{
	u32 temp = ioread32(sp->mem + reg);
	return (temp << 16) | (temp >> 16);
}

static inline void ssi9198_write(struct ssi9198_port *sp, u8 reg, u32 val)
{
	u32 temp = (val << 16) | (val >> 16);
	iowrite32(temp, sp->mem + reg);
}

static inline void ssi9198_or(struct ssi9198_port *sp, u8 reg, u32 val)
{
	u32 temp = ssi9198_read(sp, reg);
	ssi9198_write(sp, reg, temp | val);
}

static inline void ssi9198_and(struct ssi9198_port *sp, u8 reg, u32 val)
{
	u32 temp = ssi9198_read(sp, reg);
	ssi9198_write(sp, reg, temp & val);
}

// HW config
static void ssi9198_change_params(struct ssi9198_port *sp)
{
	struct seamac_params *params = &sp->params;

	DBGALL("seamac_change_params(%i) - %04X\n", sp->port.line, sp->devid);

	// Default values that cannot be changed for this architecture
	params->mode = SEAMAC_MODE_BISYNC_FRAMED;
	params->interface = SEAMAC_IF_423;
	params->loopback = 0x00;
	params->baseclk = 0;
	params->rate = 0;
	params->encoding = SEAMAC_ENCODE_NRZ;
	params->txclk = SEAMAC_CLK_TXCLK;
	params->rxclk = SEAMAC_CLK_RXCLK;
	params->rxclktype = SEAMAC_RXCLK_TTL;
	params->telement = 0x00;
	params->stopbits = 0x00;
	params->parity = SEAMAC_PARITY_NONE;
	params->rxbits = SEAMAC_BITS_8;
	params->txbits = SEAMAC_BITS_8;
	params->syncflag = 0x5C5C;
	params->sixbitflag = 0;
	params->addrfilter = 0xFF;
	params->crctype = SEAMAC_CRC_NONE;
	params->crcpreset = SEAMAC_CRC_PRESET0;
	params->idlemode = SEAMAC_IDLE_CUSTOM;
	params->underrun = SEAMAC_UNDERRUN_FLAG;
	params->rtscontrol = SEAMAC_RTSCONTROL_DISABLE;

	// Configure values that CAN be set
	ssi9198_write(sp, SSI_9198_REG_IDLE, params->idlepattern);
	DBGUART(" IDLE = %X\n", ssi9198_read(sp, SSI_9198_REG_IDLE));

	if (params->handshaking == SEAMAC_HANDSHAKE_NONE)
		ssi9198_and(sp, SSI_9198_REG_TXCTL, ~SSI_9198_REG_TXCTL_HSENABLE);
	else if (params->handshaking == SEAMAC_HANDSHAKE_RTSCTS)
		ssi9198_or(sp, SSI_9198_REG_TXCTL, SSI_9198_REG_TXCTL_HSENABLE);
	
	DBGUART(" TXCTL = %X\n", ssi9198_read(sp, SSI_9198_REG_TXCTL));


	// I see no reason not to allow these values to be set
	//params->pretxdelay;
	//params->posttxdelay;
}

// ----------------------------------------------------------------------------
// ISR routines
// ----------------------------------------------------------------------------

// Called by ISR whenever an interrupt is triggered whenever a FRAME is available
static unsigned int ssi9198_rx_chars(struct ssi9198_port *sp)
{
	struct tty_struct *tty = cpat_tty(sp->port);
	struct circ_buf *rxbuf = &sp->rxbuf;
	u32 ch, fifo;
	int count;

	DBGISR("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// Start by clearing the interrupt condition
	ssi9198_write(sp, SSI_9198_REG_ICR, SSI_9198_REG_IRQ_RXEOF);

	// Now read the contents of the RX FIFO (up to 128 B, 4 bytes at a time)
	while ((fifo = ssi9198_read(sp, SSI_9198_REG_RXFIFO)) >= 4) {
		ch = ssi9198_read(sp, SSI_9198_REG_RX32);

		// put the 4 bytes from the fifo into temp buffer
		rxbuf->buf[rxbuf->head] = (u8)((ch >> 0) & 0xFF);
		rxbuf->head += 1;
		rxbuf->head &= (PAGE_SIZE - 1);
		
		rxbuf->buf[rxbuf->head] = (u8)((ch >> 8) & 0xFF);
		rxbuf->head += 1;
		rxbuf->head &= (PAGE_SIZE - 1);

		rxbuf->buf[rxbuf->head] = (u8)((ch >> 16) & 0xFF);
		rxbuf->head += 1;
		rxbuf->head &= (PAGE_SIZE - 1);

		rxbuf->buf[rxbuf->head] = (u8)((ch >> 24) & 0xFF);
		rxbuf->head += 1;
		rxbuf->head &= (PAGE_SIZE - 1);

		sp->port.icount.rx += 4;
		DBGISR("reading 4 chars %08X from FIFO\n", ch);
	}

	// Read the rest of the FIFO 1 bytes at a time (drain)
	if (fifo != 0) {
		while ((fifo = ssi9198_read(sp, SSI_9198_REG_RXFIFO)) >= 1) {
			ch = ssi9198_read(sp, SSI_9198_REG_RX8);

			// put the 4 bytes from the fifo into temp buffer
			rxbuf->buf[rxbuf->head] = (u8)((ch >> 0) & 0xFF);
			rxbuf->head += 1;
			rxbuf->head &= (PAGE_SIZE - 1);

			sp->port.icount.rx++;
			DBGISR("reading 1 chars %02X from FIFO\n", ch);
		}
	}

	// Push the entire frame to the LDISC
	count = CIRC_CNT(rxbuf->head, rxbuf->tail, PAGE_SIZE);

	ldisc_receive_buf(tty, rxbuf->buf, NULL, count);
	rxbuf->head = rxbuf->tail = 0;

	DBGISR("end %s framesize = %i\n", __FUNCTION__, count);
	return IRQ_HANDLED;
}

// This is called whenever the TX FIFO has emptied, send next frame
static unsigned int ssi9198_tx_chars(struct ssi9198_port *sp)
{
	struct uart_port *port = &sp->port;
	struct circ_buf *xmit = cpat_xmit(sp->port);
	u32 ch;

	DBGISR("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// Start by clearing the interrupt condition
	ssi9198_write(sp, SSI_9198_REG_ICR, SSI_9198_REG_IRQ_TXFIFO | SSI_9198_REG_IRQ_TXEMT);

	if (uart_tx_stopped(port)) {
		DBGISR("tx has been stopped\n");
		return IRQ_HANDLED;
	}

	// If the FIFO is empty and the Queue is empty, we are done
	if (uart_circ_empty(xmit)) {
		port->ops->stop_tx(port cpat_tty_start_stop_arg);
		return IRQ_HANDLED;
	}

	// Write up to 256 chars, up to 4 at a time
	while (uart_circ_chars_pending(xmit) >= 4) {

		// Do not try to over fill the tx FIFO
		if (ssi9198_read(sp, SSI_9198_REG_TXFIFO) >= 250) {
			DBGISR("end %s (write32 TXFIFO full)\n", __FUNCTION__);
			break;
		}

		// write 4 characters to the FIFO at once (32-bit register)
		ch = 0;

		ch |= (xmit->buf[xmit->tail] << 0) & 0x000000FF;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		ch |= (xmit->buf[xmit->tail] << 8) & 0x0000FF00;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		ch |= (xmit->buf[xmit->tail] << 16) & 0x00FF0000;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		ch |= (xmit->buf[xmit->tail] << 24) & 0xFF000000;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		ssi9198_write(sp, SSI_9198_REG_TX32, ch);
		sp->port.icount.tx += 4;
		DBGISR("writing 4 chars %08X to FIFO\n", ch);
	}

	// Write up to 256 chars, 1 at a time (draining)
	while (uart_circ_chars_pending(xmit) >= 1) {

		// Do not try to over fill the tx FIFO
		if (ssi9198_read(sp, SSI_9198_REG_TXFIFO) >= 250) {
			DBGISR("end %s (write8 TXFIFO full)\n", __FUNCTION__);
			break;
		}

		// write 1 character to the FIFO
		ch = 0;

		ch |= (xmit->buf[xmit->tail] << 0) & 0x000000FF;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		ssi9198_write(sp, SSI_9198_REG_TX8, ch);
		sp->port.icount.tx++;
		DBGISR("writing 1 chars %02X to FIFO\n", ch);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sp->port);

	DBGISR("end %s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

// Modem control signal changed interrupt handler
static unsigned int ssi9198_modem_status(struct ssi9198_port *sp, u32 status)
{
	struct uart_port *port = &sp->port;

	DBGISR("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// Start by clearing the interrupt condition
	ssi9198_write(sp, SSI_9198_REG_ICR, SSI_9198_REG_IRQ_CTS | SSI_9198_REG_IRQ_DSR | SSI_9198_REG_IRQ_CD);

	// Which signal(s) changed?
	if (status & SSI_9198_REG_IRQ_CTS) {
		port->icount.cts++;
		DBGISR(" cts IRQ toggle count = %i\n", port->icount.cts);
	}
	if (status & SSI_9198_REG_IRQ_DSR) {
		port->icount.dsr++;
		DBGISR(" dsr IRQ toggle count = %i\n", port->icount.dsr);
	}
	if (status & SSI_9198_REG_IRQ_CD) {
		port->icount.dcd++;
		DBGISR(" dcd IRQ toggle count = %i\n", port->icount.dcd);
	}

	wake_up_interruptible(&port->state->port.delta_msr_wait);

	DBGISR("end %s\n", __FUNCTION__);
	return IRQ_HANDLED;
}


static irqreturn_t ssi9198_isr(int irq, void *dev_id)
{
	struct ssi9198_port *sp = dev_id;
	unsigned int handled = 0;
	u32 status;

	spin_lock(&sp->port.lock);

	status = ssi9198_read(sp, SSI_9198_REG_MIR);
	if (status) {
		DBGISR("IRQ%i seamac(%i) - %04X  MIR = %X\n", sp->irq, sp->port.line, sp->devid, status);

		// Transmit interrupt
		if (status & (SSI_9198_REG_IRQ_TXFIFO | SSI_9198_REG_IRQ_TXEMT))
			handled = ssi9198_tx_chars(sp);

		// Receive interrupt
		if (status & SSI_9198_REG_IRQ_RXEOF)
			handled = ssi9198_rx_chars(sp);

		// Modem Control signals interrupt
		if (status & (SSI_9198_REG_IRQ_CTS | SSI_9198_REG_IRQ_DSR | SSI_9198_REG_IRQ_CD))
			handled = ssi9198_modem_status(sp, status);
	}

	spin_unlock(&sp->port.lock);

	if (handled)
		return handled;

	return IRQ_NONE;
}


// ----------------------------------------------------------------------------
// UART routines
// ----------------------------------------------------------------------------

// Is there room in the tx buffer?
static unsigned int ssi9198_tx_empty(struct uart_port *port)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	unsigned long flags;
	unsigned char active;
	u32 txfifo;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	spin_lock_irqsave(&port->lock, flags);
	txfifo = ssi9198_read(sp, SSI_9198_REG_TXFIFO);
	active = sp->txactive;
	spin_unlock_irqrestore(&port->lock, flags);

	if (!active && txfifo == 0) {
		DBGUART("end %s (fifo empty)\n", __FUNCTION__);
		return TIOCSER_TEMT;
	}

	DBGUART("end %s (not empty -- %u)\n", __FUNCTION__, txfifo);
	return 0;
}

// Read the current state of the modem control lines.
static unsigned int ssi9198_get_mctrl(struct uart_port *port)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	unsigned int ret = 0;
	u32 mask;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

 	mask = ssi9198_read(sp, SSI_9198_REG_MCIN);
	DBGUART(" mcin = %X\n", mask);

	// ALL MCTRLS ARE ACTIVE LOW ON 9198
	if ((mask & SSI_9198_REG_MCIN_CTS) == 0)
		ret |= TIOCM_CTS;
	
	if ((mask & SSI_9198_REG_MCIN_DSR) == 0)
		ret |= TIOCM_DSR;

	if ((mask & SSI_9198_REG_MCIN_CD) == 0)
		ret |= TIOCM_CAR;

	// If RI is not present, it should always be marked inactive

	DBGUART("end %s (%X)\n", __FUNCTION__, ret);
	return ret;
}

// Set modem control signals.
static void ssi9198_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	u32 mask = 0;

	DBGUART("start %s(%i, %X) - %04X\n", __FUNCTION__, sp->port.line, mctrl, sp->devid);

	// ALL MCTRLS ARE ACTIVE LOW ON 9198
	if ((mctrl & TIOCM_RTS) == 0)
		mask |= SSI_9198_REG_MCOUT_RTS;

	ssi9198_write(sp, SSI_9198_REG_MCOUT, mask);

	DBGUART("end %s (mcout = %X)\n", __FUNCTION__, mask);
}

// Quit transmitting as soon as possible.
static void ssi9198_stop_tx(struct uart_port *port cpat_tty_start_stop_param)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// Disable Transmit Interrupts to stop the tx engine
	ssi9198_and(sp, SSI_9198_REG_IMR, ~SSI_9198_REG_IRQ_TXEMT);

	// TX Post Delay (ms) max delay == short.Max == 65535 ms
	if (sp->params.posttxdelay > 0)
		mdelay(sp->params.posttxdelay);

	sp->txactive = 0;
	DBGUART("end %s\n", __FUNCTION__);
}

// Start transmitting any available characters
static void ssi9198_start_tx(struct uart_port *port cpat_tty_start_stop_param)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// TX busy?  Just wait for the TX done interrupt.
	if (sp->txactive) {
		DBGUART("transmit is active, just wait for next ISR\n");
		return;
	}

	// Enable Transmit Interrupts to start the tx engine
	ssi9198_or(sp, SSI_9198_REG_IMR, SSI_9198_REG_IRQ_TXEMT);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));
	
	sp->txactive = 1;
	
	// TX Pre Delay (ms) max delay == short.Max == 65535 ms
	if (sp->params.pretxdelay > 0)
		mdelay(sp->params.pretxdelay);

	// Go ahead and start the tx if there are any chars ready
	ssi9198_tx_chars(sp);
	
	DBGUART("end %s\n", __FUNCTION__);
}

// Cease receiving
static void ssi9198_stop_rx(struct uart_port *port)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);
	ssi9198_and(sp, SSI_9198_REG_IMR, ~SSI_9198_REG_IRQ_RXEOF);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));

	sp->rxactive = 0;
	DBGUART("end %s\n", __FUNCTION__);
}

// Enable external modem status interrupts
static void ssi9198_enable_ms(struct uart_port *port)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);
	ssi9198_or(sp, SSI_9198_REG_IMR, SSI_9198_REG_IRQ_CTS | SSI_9198_REG_IRQ_DSR |SSI_9198_REG_IRQ_CD);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));

	DBGUART("end %s\n", __FUNCTION__);
}

// Start/stop sending line break
static void ssi9198_break_ctl(struct uart_port *port, int break_state)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	DBGUART("start %s(%i, %i) - %04X\n", __FUNCTION__, sp->port.line, break_state, sp->devid);
	DBGUART("end %s (VOID)\n", __FUNCTION__);
}

// Startup the device (really just start rx interrupts)
static int ssi9198_startup(struct uart_port *port)
{
	unsigned long flags;
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	u32 temptx, temprx;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

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

	request_irq(sp->irq, &ssi9198_isr, IRQF_SHARED, "seamac", sp);

	spin_lock_irqsave(&port->lock, flags);

	// Persist handshake bits
	temptx = ssi9198_read(sp, SSI_9198_REG_TXCTL);
	temprx = ssi9198_read(sp, SSI_9198_REG_RXCTL);

	// Reset TX and RX channels
	ssi9198_write(sp, SSI_9198_REG_TXCTL, SSI_9198_REG_TXCTL_TXRST);
	ssi9198_write(sp, SSI_9198_REG_RXCTL, SSI_9198_REG_RXCTL_RXRST);

	// Enable TX and RX and any other special features
	ssi9198_write(sp, SSI_9198_REG_TXCTL, SSI_9198_REG_TXCTL_TXEN | temptx);
	ssi9198_write(sp, SSI_9198_REG_RXCTL, SSI_9198_REG_RXCTL_RXEN | temprx);

	mdelay(30);
	
	// Update settings -- last valid settings
	ssi9198_change_params(sp);

	// Configure what generates an interrupt and clear out any old interrupt states
	// Only interrupt on RX for now, start_tx will enable the TX interrupts and enable_ms
	// will enable the mctrl interrupts
	ssi9198_write(sp, SSI_9198_REG_IMR, SSI_9198_REG_IRQ_RXEOF);
	ssi9198_write(sp, SSI_9198_REG_ICR, 0xFFFFFFFF);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));

	// Book-keeping
	sp->txactive = 0;
	sp->rxactive = 1;

	spin_unlock_irqrestore(&port->lock, flags);
	DBGUART("end %s\n", __FUNCTION__);

	return 0;
}

// Shutdown port ISR
static void ssi9198_shutdown(struct uart_port *port)
{
	unsigned long flags;
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	spin_lock_irqsave(&port->lock, flags);
	ssi9198_write(sp, SSI_9198_REG_IMR, 0x00000000);
	ssi9198_write(sp, SSI_9198_REG_ICR, 0xFFFFFFFF);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));
	spin_unlock_irqrestore(&port->lock, flags);

	free_irq(sp->irq, sp);

	DBGUART("dealloc rxbuf\n");
	if (sp->rxbuf.buf) {
		free_page((unsigned long)sp->rxbuf.buf);
		sp->rxbuf.buf = NULL;
		sp->rxbuf.head = sp->rxbuf.tail = 0;
	}
	DBGUART("rxbuf gone\n");

	DBGUART("end %s\n", __FUNCTION__);
}

// Configure port based on termios settings.  Does nothing for this
// architecture since only Bi-Sync mode is available.  Use ioctl instead.
static void
ssi9198_set_termios(struct uart_port *port, struct ktermios *termios,
					struct ktermios *old)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	termios->c_cflag = CLOCAL | CREAD;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);
	// Nothing to do, this is purely synchronous hardware...
	DBGUART("end %s (VOID)\n", __FUNCTION__);
}

// Return an ID string of port type - modifying the string may be a bad idea,
// all the other serial drivers just use constant strings... (works well tho..)
static char ssi9198_port_id_string[50];
static const char *ssi9198_type(struct uart_port *port)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;
	unsigned short device = sp->devid;

	u32 id = ssi9198_read(sp, SSI_9198_REG_ID);
	u32 rev = ssi9198_read(sp, SSI_9198_REG_REV);

	sprintf(ssi9198_port_id_string, "%X (SL-%X rev %X)", device, id, rev);
	return ssi9198_port_id_string;
}

static void ssi9198_release_port(struct uart_port *port)
{
}

static int ssi9198_request_port(struct uart_port *port)
{
	return 0;
}

// These do not need to do anything interesting either.
static void ssi9198_config_port(struct uart_port *port, int flags)
{

}

static int ssi9198_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return -EINVAL;
}

// Port specific ioctls (configuration)
static int ssi9198_get_params(struct ssi9198_port *sp, void __user *to)
{
	void *from = (void *) &sp->params;

	if (copy_to_user(to, from, sizeof(struct seamac_params)))
		return -EFAULT;

	return 0;
}

// Allow user to set port configuration structure
static int ssi9198_set_params(struct ssi9198_port *sp, void __user *from)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	void *to = (void *) &sp->params;

	if (copy_from_user(to, from, sizeof(struct seamac_params)))
		return -EFAULT;

	spin_lock_irqsave(&port->lock, flags);
	ssi9198_change_params(sp);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;	
}

// Check if transmission is active (FIFO empty?)
static int ssi9198_get_txactive(struct ssi9198_port *sp, void __user *to)
{
	int active = ssi9198_tx_empty(&sp->port) != TIOCSER_TEMT;
	return copy_to_user(to, &active, sizeof(active));
}

// A custom method to retrieve modem control signals (and others) all in one bitmask
static int ssi9198_get_signals(struct ssi9198_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int signals = 0;
	void *from = (void *) &signals;
	u32 maskin, maskout;

	spin_lock_irqsave(&port->lock, flags);
 	maskin = ssi9198_read(sp, SSI_9198_REG_MCIN);
 	maskout = ssi9198_read(sp, SSI_9198_REG_MCOUT);
	spin_unlock_irqrestore(&port->lock, flags);
	
	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	DBGUART(" mcin = %X\n", maskin);
	DBGUART(" mcout = %X\n", maskout);

	// ALL MCTRLS ARE ACTIVE LOW ON 9198

	if ((maskout & SSI_9198_REG_MCOUT_RTS) == 0)
		signals |= SEAMAC_RTS;

	if ((maskin & SSI_9198_REG_MCIN_CTS) == 0)
		signals |= SEAMAC_CTS;
	
	if ((maskin & SSI_9198_REG_MCIN_DSR) == 0)
		signals |= SEAMAC_DSR;

	if ((maskin & SSI_9198_REG_MCIN_CD) == 0)
		signals |= SEAMAC_DCD;
		
	if (copy_to_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	return 0;
}

// A custom method to set all of the output modem control (and other) signals
static int ssi9198_set_signals(struct ssi9198_port *sp, void __user *from)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int signals = 0;
	void *to = (void *) &signals;
	u32 mask = 0;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	if (copy_from_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	// ALL MCTRLS ARE ACTIVE LOW ON 9198
	if ((signals & SEAMAC_RTS) == 0)
		mask |= SSI_9198_REG_MCOUT_RTS;

	DBGUART(" mcout = %X\n", mask);

	spin_lock_irqsave(&port->lock, flags);
	ssi9198_write(sp, SSI_9198_REG_MCOUT, mask);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;	
}

static int ssi9198_get_txclkrate(struct ssi9198_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int rate = 0;
	void *from = (void *) &rate;
	u32 count;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	spin_lock_irqsave(&port->lock, flags);
 	count = ssi9198_read(sp, SSI_9198_REG_TXRATE);
	spin_unlock_irqrestore(&port->lock, flags);
	
	DBGUART(" tx count = %i\n", count);

	// Clk frequency = 1 / (count * 50ns)
	if (count != 0)
		rate = 1000000000 / (count * 50);

	DBGUART(" tx rate = %i\n", rate);

	if (copy_to_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	return 0;
}

static int ssi9198_get_rxclkrate(struct ssi9198_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	unsigned int rate = 0;
	void *from = (void *) &rate;
	u32 count;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	spin_lock_irqsave(&port->lock, flags);
 	count = ssi9198_read(sp, SSI_9198_REG_RXRATE);
	spin_unlock_irqrestore(&port->lock, flags);
	
	DBGUART(" rx count = %i\n", count);

	// Clk frequency = 1 / (count * 50ns)
	if (count != 0)
		rate = 1000000000 / (count * 50);

	DBGUART(" rx rate = %i\n", rate);

	if (copy_to_user(to, from, sizeof(unsigned int)))
		return -EFAULT;

	return 0;
}

static int ssi9198_enter_hunt(struct ssi9198_port *sp, void __user *to)
{
	struct uart_port *port = (struct uart_port *) sp;
	unsigned long flags;
	u32 temp;

	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	spin_lock_irqsave(&port->lock, flags);

	// Preserve any extra rx options
	temp = ssi9198_read(sp, SSI_9198_REG_RXCTL);

	// Reset RX channels
	ssi9198_write(sp, SSI_9198_REG_RXCTL, SSI_9198_REG_RXCTL_RXRST);

	// Re-enable rx 
	ssi9198_write(sp, SSI_9198_REG_RXCTL, temp | SSI_9198_REG_RXCTL_RXEN);

	spin_unlock_irqrestore(&port->lock, flags);
	
	return 0;
}

static int 
ssi9198_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct ssi9198_port *sp = (struct ssi9198_port *) port;

	switch (cmd) {
	case SEAMAC_IOCTL_GPARAMS:
		return ssi9198_get_params(sp, (void __user *) arg);
	case SEAMAC_IOCTL_SPARAMS:
		return ssi9198_set_params(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GTXACTIVE:
		return ssi9198_get_txactive(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GSIGNALS:
		return ssi9198_get_signals(sp, (void __user *) arg);
	case SEAMAC_IOCTL_SSIGNALS:
		return ssi9198_set_signals(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GTXCLKRATE:
		return ssi9198_get_txclkrate(sp, (void __user *) arg);
	case SEAMAC_IOCTL_GRXCLKRATE:
		return ssi9198_get_rxclkrate(sp, (void __user *) arg);
	case SEAMAC_IOCTL_HUNT:
		return ssi9198_enter_hunt(sp, (void __user *) arg);
	
	// NOT IMPLEMENTED OR SUPPORTED ON THIS ARCHITECTURE
	case SEAMAC_IOCTL_GSECURITY:
	default:
		return -ENOIOCTLCMD;
	}
}

static struct uart_ops ssi9198_pops = {
	.tx_empty	=	ssi9198_tx_empty,
	.set_mctrl	=	ssi9198_set_mctrl,
	.get_mctrl	=	ssi9198_get_mctrl,
	.stop_tx	=	ssi9198_stop_tx,
	.start_tx	=	ssi9198_start_tx,
	.stop_rx	=	ssi9198_stop_rx,
	.enable_ms	=	ssi9198_enable_ms,
	.break_ctl	=	ssi9198_break_ctl,
	.startup	=	ssi9198_startup,
	.shutdown	=	ssi9198_shutdown,
	.set_termios	=	ssi9198_set_termios,
	.type		=	ssi9198_type,
	.release_port	=	ssi9198_release_port,
	.request_port	=	ssi9198_request_port,
	.config_port	=	ssi9198_config_port,
	.verify_port	=	ssi9198_verify_port,
	.ioctl 		=	ssi9198_ioctl,
};

// -----------------------------------------------------------------------------
// Move to startup/shutdown maybe?
// -----------------------------------------------------------------------------

static void ssi9198_hw_init(struct ssi9198_port *sp)
{
	DBGUART("start %s(%i) - %04X\n", __FUNCTION__, sp->port.line, sp->devid);

	// Reset hardware and default all configuration
	ssi9198_write(sp, SSI_9198_REG_CONTROL, SSI_9198_REG_CONTROL_RESET);

	// Enable tx and rx (default handshaking to enabled)
	ssi9198_write(sp, SSI_9198_REG_TXCTL, SSI_9198_REG_TXCTL_TXEN | SSI_9198_REG_TXCTL_HSENABLE);
	ssi9198_write(sp, SSI_9198_REG_RXCTL, SSI_9198_REG_RXCTL_RXEN);

	DBGUART(" TXCTL = %X\n", ssi9198_read(sp, SSI_9198_REG_TXCTL));
	DBGUART(" RXCTL = %X\n", ssi9198_read(sp, SSI_9198_REG_RXCTL));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));

	// Configure port per default parameters (also sets any hardware defaults)
	ssi9198_change_params(sp);

	// No interrupts yet, please (wait until the port is in use)
	ssi9198_write(sp, SSI_9198_REG_IMR, 0x00000000);
	ssi9198_write(sp, SSI_9198_REG_ICR, 0xFFFFFFFF);

	DBGUART("   IMR = %X\n", ssi9198_read(sp, SSI_9198_REG_IMR));
	DBGUART("   MIR = %X\n", ssi9198_read(sp, SSI_9198_REG_MIR));
	DBGUART("   ISR = %X\n", ssi9198_read(sp, SSI_9198_REG_ISR));
	DBGUART(" HWDBG = %X\n", ssi9198_read(sp, SSI_9198_REG_HWDBG));
}

static void ssi9198_hw_deinit(struct ssi9198_port *sp)
{
	// No more interrupts, if we somehow still have them enabled...
	ssi9198_write(sp, SSI_9198_REG_IMR, 0x00000000);
	ssi9198_write(sp, SSI_9198_REG_ICR, 0xFFFFFFFF);

	// The final state of the card should be disabled
	ssi9198_write(sp, SSI_9198_REG_TXCTL, SSI_9198_REG_TXCTL_TXRST);
	ssi9198_write(sp, SSI_9198_REG_RXCTL, SSI_9198_REG_RXCTL_RXRST);
}


// ----------------------------------------------------------------------------
// Device info structure allocation & initialization / deallocation.
// ----------------------------------------------------------------------------

// Request memory for port information.  Also set some initial values.
static struct ssi9198_port *ssi9198_alloc_port(struct device *dev, unsigned int devid, 
				unsigned long base, void __iomem *mem, unsigned int irq)
{
	struct ssi9198_port *sp;

	sp = kzalloc(sizeof(*sp), GFP_KERNEL);
	if (sp == NULL)
		return NULL;

	sp->port.iobase = 0;
	sp->port.mapbase = base;
	sp->port.membase = (unsigned char __iomem *) mem;
	sp->port.iotype = UPIO_MEM32;
	sp->port.irq = irq;
	sp->port.uartclk = 0;
	sp->port.fifosize = 256;
	sp->port.ops = &ssi9198_pops;
	sp->port.type = PORT_SEALEVEL_SYNC;
	sp->port.flags = 0;
	sp->port.dev = dev;
	
	// Note most of these values are just copies of values from uart_port... redundant?
	sp->dev = dev;
	sp->devid = devid;
	
	sp->mem = mem;
	sp->irq = irq;
	
	// book-keeping
	sp->txactive = 0;
	sp->rxactive = 0;
	
	// Make sure rxbuf head/tail are clear (note *buf is alloc'd on request)
	sp->rxbuf.buf = NULL;
	sp->rxbuf.head = sp->rxbuf.tail = 0;

	// copy the port defaults over, we can fix them if they aren't right later
	memcpy(&sp->params, &seamac_defaults, sizeof(struct seamac_params));
	sp->params.baseclk = 0;

	return sp;
}

// Free a port information struct. Also clearing any values previously set.
static void ssi9198_dealloc_port(struct ssi9198_port *sp)
{
	sp->port.iobase = 0;
	sp->port.membase = NULL;
	sp->port.iotype = 0;
	sp->port.irq = 0;
	sp->port.uartclk = 0;
	sp->port.fifosize = 0;
	sp->port.ops = NULL;
	sp->port.type = 0;
	sp->port.flags = 0;
	sp->port.line = 0;
	sp->port.dev = NULL;

	sp->dev = NULL;
	sp->devid = 0;
	sp->mem = NULL;
	sp->irq = 0;
	sp->txactive = 0;
	sp->rxactive = 0;
	
	sp->rxbuf.buf = NULL;
	sp->rxbuf.head = sp->rxbuf.tail = 0;

	kfree(sp);
}

		
// -----------------------------------------------------------------------------
// PCI device detection functions.
// -----------------------------------------------------------------------------
int __devinit ssi9198_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct ssi9198_port *sad;
	unsigned long base;
	void __iomem *mem;
	int rc = 0;

	// Request memory map block for BAR 0 (BAR 1 is also a mirror)
	base = pci_resource_start(dev, 0);
	mem = pci_iomap(dev, 0, 0);
	if(!mem) {
		DBGERR("Failed to map memory region!\n");
		goto fail_regions;
	}
	else
		DBGINFO("memory region (%lX) mapped: %p\n", base, mem);

	// Allocate port information structure
	sad = ssi9198_alloc_port(&dev->dev, dev->device, base, mem, dev->irq);
	if (sad == NULL) {
		DBGERR("no free memory!\n");
		rc = -ENOMEM;
		goto fail_alloc;
	}
	
	// Check for driver/hardware compatability
	if (ssi9198_read(sad, SSI_9198_REG_ID) != 0x9198135e) {
		DBGERR("Hardware id does not match expected!\n");
		goto fail_alloc;
	}

	// Initialize the ports
	ssi9198_hw_init(sad);

	// Register the uart_port
	rc = seamac_register_port(&sad->port);
	if (rc) {
		DBGERR("Port registration failed!\n");
		goto fail_register;
	}

	pci_set_drvdata(dev, sad);

	return 0;

 fail_register:
	ssi9198_dealloc_port(sad);
 fail_alloc:
	pci_iounmap(dev, mem);
 fail_regions:
	return rc;
}

// A PCI Seamac device is being removed
void __devexit ssi9198_pci_remove(struct pci_dev *dev)
{
	struct ssi9198_port *sad = pci_get_drvdata(dev);
	void __iomem *mem = sad->mem;

	pci_set_drvdata(dev, NULL);

	seamac_deregister_port(&sad->port);

	ssi9198_hw_deinit(sad);
	ssi9198_dealloc_port(sad);

	pci_iounmap(dev, mem);
}

