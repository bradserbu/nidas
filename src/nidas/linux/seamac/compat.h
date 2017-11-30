/*
 *	Sealevel Systems Synchronous Serial kernel compatibility layer
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

#ifndef __COMPAT_H__
#define __COMPAT_H__
#include <linux/version.h>   //just in case

 // ---------------------------------------------------------------------------
 // 2.6.20 brings a new kernel termios
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
   #define ktermios termios
 #else
   //already up to date
 #endif
 
 // ---------------------------------------------------------------------------
 // The old write interface required 4 arguments and not just 3.
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
   #define seamac_write(a,b,c) seamac_write(a, int from_user, b, c)
 #else
   //already up to date
 #endif

 // ---------------------------------------------------------------------------
 // After 2.6.10 there is a new module parameter interface 
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
   #define module_param(a,b,c) MODULE_PARM(a,"i")
   //need to dereference fourth argument
   #define cpat_module_param_array_named(a,b,c,d,e) \
      module_param_array_named(a,b,c,*d,e)
 #else
   #define cpat_module_param_array_named(a,b,c,d,e) \
      module_param_array_named(a,b,c,d,e)
 #endif

 // ---------------------------------------------------------------------------
 // >2.6.10 brings a new ldisc interface.  Access is different now.
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
   static inline void ldisc_receive_buf(struct tty_struct *tty,
			      const char *data, char *flags, int count)
   {
	if (!tty || !data || !count)
		return;
	if (tty->ldisc.receive_buf)
		tty->ldisc.receive_buf(tty, data, flags, count);
   }
 #elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
   static inline void ldisc_receive_buf(struct tty_struct *tty,
			      const char *data, char *flags, int count)
   {
	struct tty_ldisc *ld;
	
	if (!tty || !data || !count)
		return;
	ld = tty_ldisc_ref(tty);
	if (ld && ld->receive_buf)
		ld->receive_buf(tty, data, flags, count);
	tty_ldisc_deref(ld);
   }
 #else // Later kernel versions added a ldisc operations struct
   static inline void ldisc_receive_buf(struct tty_struct *tty,
			      const char *data, char *flags, int count)
   {
	struct tty_ldisc *ld;
	
	if (!tty || !data || !count)
		return;
	ld = tty_ldisc_ref(tty);
	if (ld && ld->ops && ld->ops->receive_buf)
		ld->ops->receive_buf(tty, data, flags, count);
	tty_ldisc_deref(ld);
   }
 #endif
 
 // ---------------------------------------------------------------------------
 // 2.6.20 drops the third arg in interrupt routine 
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
   #define seamac_isr(a, b) seamac_isr(a, b, struct pt_regs *regs)
 #else
   //already up to date
 #endif

 // ---------------------------------------------------------------------------
 // 2.6.20 drops the third arg in the handle_sysrq method
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
   #define cpat_uart_handle_sysrq_char(a, b) uart_handle_sysrq_char(a, b, NULL)
 #else
   #define cpat_uart_handle_sysrq_char(a, b) uart_handle_sysrq_char(a, b)
 #endif

 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
   #define cpat_tty(a) a.info->tty
 #elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
   #define cpat_tty(a) a.info->port.tty
 #else
   #define cpat_tty(a) a.state->port.tty
 #endif

 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
   #define cpat_xmit(a) &a.info->xmit
 #else
   #define cpat_xmit(a) &a.state->xmit
 #endif

 // ---------------------------------------------------------------------------
 // 2.6.14 removes the {start,stop}_tx methods' tty_{start,stop} argument
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14)
   #define cpat_tty_start_stop_param , unsigned int tty_start_stop_dummy
   #define cpat_tty_start_stop_arg , 0
 #else
   #define cpat_tty_start_stop_param 
   #define cpat_tty_start_stop_arg 
 #endif

 // ---------------------------------------------------------------------------
 // 2.6.14 adds uart_insert_char()
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14)
   //code taken from commit 05ab3014636ff60a319d37cdf37dca594b015eec
   //Russell King <rmk@dyn-67.arm.linux.org.uk>, 9 May 2005
   #include <linux/tty_flip.h>
   static inline void
   uart_insert_char(struct uart_port *port, unsigned int status,
		    unsigned int overrun, unsigned int ch, unsigned int flag)
   {
	struct tty_struct *tty = port->info->tty;

	if ((status & port->ignore_status_mask & ~overrun) == 0)
		tty_insert_flip_char(tty, ch, flag);

	/*
	 * Overrun is special.  Since it's reported immediately,
	 * it doesn't affect the current character.
	 */
	if (status & ~port->ignore_status_mask & overrun)
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
   }
 #else
   //already up to date
 #endif

 // ---------------------------------------------------------------------------
 // 2.6.18 changes the SA_* flags to IRQF_* flags
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
   #define IRQF_SHARED SA_SHIRQ
 #else
   //already up to date
 #endif

 // ---------------------------------------------------------------------------
 // 3.8.0 removes unused init/exit macros
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
   #define __devinit 
   #define __devinitdata 
   #define __devinitconst 
   #define __devexit 
   #define __devexitdata 
   #define __devexitconst 
 #endif

 // ---------------------------------------------------------------------------
 // 3.9.0 changes tty_flip_buffer_push include location and signature
 // ---------------------------------------------------------------------------
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
    #include <linux/tty_flip.h>
    #define cpat_tty_flip_buffer_push(a, b) tty_flip_buffer_push(&a->port.state->port)
 #else
    #define cpat_tty_flip_buffer_push(a, b) tty_flip_buffer_push(b)
 #endif

#endif //__COMPAT_H__
