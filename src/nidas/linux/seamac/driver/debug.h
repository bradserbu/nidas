/*
 *	Sealevel Systems Synchronous Serial driver debug helper macros
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
 *	(c) Copyright 2007-2013 Sealevel Systems, Inc.
 *
 */

extern unsigned int debug_level;

#define DBGERR(fmt, args...) \
        if (debug_level >= 1) printk(KERN_ERR "seamac(error): " fmt, ## args)
#define DBGINFO(fmt, args...) \
        if (debug_level >= 2) printk(KERN_INFO "seamac(info): " fmt, ## args)
#define DBGDEVICE(fmt, args...) \
        if (debug_level & 4) printk(KERN_INFO "seamac(dev_driver): " fmt, ## args)
#define DBGUART(fmt, args...) \
        if (debug_level & 8) printk(KERN_INFO "seamac(uart_driver): " fmt, ## args)
#define DBGISR(fmt, args...) \
        if (debug_level & 16) printk(KERN_INFO "seamac(isr): " fmt, ## args)
#define DBGTEMP(fmt, args...) \
        if (debug_level & 32) printk(KERN_INFO "seamac(temp): " fmt, ## args)
#define DBGALL(fmt, args...) \
        if (debug_level & 128) printk(KERN_INFO "seamac(all): " fmt, ## args)
