/*
 *	Sealevel Systems Synchronous Serial driver core
 *
 * 	Sealevel and Seamac are registered trademarks of Sealevel Systems 
 * 	Incorporated.
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

#include <linux/module.h>
#include <linux/kernel.h>

#define SEAMAC_DRIVER_INCLUDE
#include "seamac.h"
#include "debug.h"
#include "compat.h"

// Hardware architectures supported
#include "z85230.h"	// Zilog Z8523X
#include "ssi9198.h"	// Sealevel Custom IP (BI-SYNC like)

static char *driver_name = "Sealevel Synchronous Serial Driver";
static char *driver_version = "1.6.6";

// See debug.h for verbosity level definitions
unsigned int debug_level = 0x02;
module_param(debug_level, int, 0);

#define SEALEVEL_DRVNAME		"seamac"
#define SEALEVEL_DEVNAME		"ttySM"
#define SEALEVEL_MAJOR			240
#define SEALEVEL_MINOR			0
#define SEALEVEL_MAX_PORTS		128

// -----------------------------------------------------------------------------
// Uart Driver registration data
// -----------------------------------------------------------------------------

static struct uart_driver seamac_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= SEALEVEL_DRVNAME,
	.dev_name	= SEALEVEL_DEVNAME,
	.major		= SEALEVEL_MAJOR,
	.minor		= SEALEVEL_MINOR,
	.nr		= SEALEVEL_MAX_PORTS,
};


static unsigned int seamac_port_count;  // Note: no hot-plug capability

int seamac_register_port(struct uart_port *port)
{
	int rc;
	
	DBGINFO("calling uart_add_one_port\n");
	port->line = seamac_port_count;
	rc = uart_add_one_port(&seamac_uart_driver, port);
	DBGINFO("uart_add_one_port returned val=%i\n", rc);
	seamac_port_count++;
		
	return rc;
}

void seamac_deregister_port(struct uart_port *port)
{
	uart_remove_one_port(&seamac_uart_driver, port);
	seamac_port_count--;
}


// -----------------------------------------------------------------------------
// PCI device detection functions.
// -----------------------------------------------------------------------------

// Identifies the devices (by vendor id/device id) this driver works with
static struct pci_device_id seamac_pci_tbl[] = {
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5102, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_SINGLEPORT },
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5103, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_SINGLEPORT },
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5402, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_QUADPORT },
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5102E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_Z85230_INTENABLE | SEALEVEL_SYNC_SINGLEPORT },
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5103E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_Z85230_INTENABLE | SEALEVEL_SYNC_SINGLEPORT },

	// IO Region is mapped a little differently
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_5402E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230_IOSHARED | SEALEVEL_SYNC_Z85230_INTENABLE | SEALEVEL_SYNC_QUADPORT },

	// Custom form-factor (PMC 5402)
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_9113, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_QUADPORT },
	// Custom form-factor (PMC 5402 -- Modified IO mapping to fix a problem)
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_9155, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_QUADPORT },

	// Custom Bisync Mini PCI-Express device (9198)
	{ PCI_VENDOR_ID_SEALEVEL, PCI_DEVICE_ID_SEALEVEL_9198, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		SEALEVEL_SYNC_9198 | SEALEVEL_SYNC_SINGLEPORT },

	{ 0, }
};

static int __devinit seamac_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int rc = pci_enable_device(dev);
	if (rc) {
		DBGERR("Cannot enable device -- %04X!\n", dev->device);
		goto fail_enable;
	}

	// Call into hardware type to continue enumeration
	if (id->driver_data & (SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_Z85230_IOSHARED))
		return z85_pci_probe(dev, id);
	else if (id->driver_data & SEALEVEL_SYNC_9198)
		return ssi9198_pci_probe(dev, id);
	else {
		DBGERR("%s: unsupported hardware type detected (check PCI_TBL) -- %04X\n", 
			__FUNCTION__, dev->device);
		goto fail_enum;
	}

 fail_enum:
	pci_disable_device(dev);
 fail_enable:
	return rc;
}

// A PCI Seamac device is being removed
static void __devexit seamac_pci_remove(struct pci_dev *dev)
{
	int i;

	DBGDEVICE("%s start\n", __FUNCTION__);

	// Search pci_tbl for matching device entry
	for (i = 0; i < sizeof(seamac_pci_tbl) / sizeof(struct pci_device_id); i++) {
		DBGDEVICE(" checking pci_tbl entry %i = %X:%X\n", i, seamac_pci_tbl[i].vendor, seamac_pci_tbl[i].device);
		if (seamac_pci_tbl[i].device == 0 || seamac_pci_tbl[i].device == dev->device) {
			DBGDEVICE(" found end of list or device match (%X)\n", seamac_pci_tbl[i].device);
			break;	// found
		}
	}

	// Call into hardware type to continue removal
	if (seamac_pci_tbl[i].driver_data & (SEALEVEL_SYNC_Z85230 | SEALEVEL_SYNC_Z85230_IOSHARED))
		z85_pci_remove(dev);
	else if (seamac_pci_tbl[i].driver_data & SEALEVEL_SYNC_9198)
		ssi9198_pci_remove(dev);
	else {
		// This is a critical error, somehow we got into the wrong device removal function...
		DBGERR("%s: unsupported hardware type detected (check PCI_TBL) -- %04X\n",
			__FUNCTION__, dev->device);
	}

	// Disable the device
	pci_disable_device(dev);
	
	DBGDEVICE("%s end\n", __FUNCTION__);
}

// Tells the PCI subsystem where the device id list is, and what to do on
// detection/removal of one of those devices
static struct pci_driver seamac_pci_driver = {
	.name		= SEALEVEL_DRVNAME,
	.id_table 	= seamac_pci_tbl,
	.probe		= seamac_pci_probe,
	.remove		= seamac_pci_remove,
};


// -----------------------------------------------------------------------------
// PCMCIA device detection.  (Optional)
// -----------------------------------------------------------------------------

#ifdef PCMCIA_SUPPORT
// List of PCMCIA devices this driver supports...
static struct pcmcia_device_id seamac_pcmcia_tbl[] __devinitdata = {
	PCMCIA_DEVICE_MANF_CARD_PROD_ID1(PCMCIA_VENDOR_ID_SEALEVEL, 
					 PCMCIA_DEVICE_ID_SEALEVEL_3612, 
					 "Sealevel Systems Inc.", 
					 0x9e6fc64b),
	PCMCIA_DEVICE_NULL
};

// pcmcia_loop_config callback
static int seamac_pcmcia_ioprobe(struct pcmcia_device *dev, void *priv)
{
	return pcmcia_request_io(dev);
}

// Callback for card insertion.
static int __devinit seamac_pcmcia_attach(struct pcmcia_device *dev)
{
	int rc = 0;

	// Config card and enable auto config for IO and IRQ
	dev->config_flags |= CONF_ENABLE_IRQ | CONF_AUTO_SET_IO;
	if (pcmcia_loop_config(dev, seamac_pcmcia_ioprobe, NULL)) {
		DBGERR("pcmcia_loop_config() failed!\n");
		return -ENODEV;
	}

	// Enable hardware
	if (pcmcia_enable_device(dev)) {
		DBGERR("pcmcia_enable_device() failed!\n");
		return -ENODEV;
	}

	// If there is ever a different hardware type PCMCIA, this will need to get smarter
	if ((rc = z85_pcmcia_attach(dev)) < 0)
		pcmcia_disable_device(dev);

	return rc;
}

// Callback for card ejection.
static void seamac_pcmcia_detach(struct pcmcia_device *dev)
{
	// If there is ever a different hardware type PCMCIA, this will need to get smarter
	z85_pcmcia_detach(dev);

	// Release all OS allocated resources and disable device
	pcmcia_disable_device(dev);
}

// PCMCIA driver description
static struct pcmcia_driver seamac_pcmcia_driver = {
	.owner		= THIS_MODULE,
	.drv		= {
		.name	= SEALEVEL_DRVNAME,
	},
	.probe		= seamac_pcmcia_attach,
	.remove		= seamac_pcmcia_detach,
	.id_table       = seamac_pcmcia_tbl,
};
#endif	//PCMCIA_SUPPORT


// -----------------------------------------------------------------------------
// ISA device specification. (ISA isn't plug and play, we must be told about it)
// This can also be optionally disabled.
// -----------------------------------------------------------------------------

#ifdef ISA_SUPPORT

static int isa_io[SEALEVEL_MAX_ISA];
static int isa_irq[SEALEVEL_MAX_ISA];
static int isa_io_count, isa_irq_count;
cpat_module_param_array_named(io, isa_io, int, &isa_io_count, 0);
cpat_module_param_array_named(irq, isa_irq, int, &isa_irq_count, 0);
// TODO: Add support for hardware type identification (z85, ssi9198, xxxx, etc.)

static int seamac_isa_register_driver(void)
{
	int i;
	for (i = 0; ((i < SEALEVEL_MAX_ISA) && isa_io[i] && isa_irq[i]); i++)
		z85_isa_attach(isa_io[i], isa_irq[i]);

	return 0;
}

static void isa_unregister_driver(void)
{
	int i;
	for (i = 0; i < SEALEVEL_MAX_ISA; i++)
		z85_isa_detach(i);
}
#endif	//ISA_SUPPORT


// -----------------------------------------------------------------------------
// Kernel module entry/exit
// -----------------------------------------------------------------------------

// Module initialization
static int __init seamac_init_module(void)
{
	int ret;
	DBGINFO("Loading %s(rev=%s build=%s %s)\n", driver_name, driver_version,
		 __DATE__, __TIME__);

	seamac_port_count = 0;

	if ((ret = uart_register_driver(&seamac_uart_driver)) > 0)
		goto uart_fail;

#ifdef ISA_SUPPORT
	if ((ret = seamac_isa_register_driver()) < 0)
		goto isa_fail;
#endif

	if ((ret = pci_register_driver(&seamac_pci_driver)) < 0)
		goto pci_fail;

#ifdef PCMCIA_SUPPORT
	if ((ret = pcmcia_register_driver(&seamac_pcmcia_driver)) < 0)
		goto pcmcia_fail;
#endif

	return ret;

#ifdef PCMCIA_SUPPORT
 pcmcia_fail:
	DBGINFO("%s PCMCIA load failure!\n", __FUNCTION__);
#endif
	pci_unregister_driver(&seamac_pci_driver);
 pci_fail:
	DBGINFO("%s PCI load failure!\n", __FUNCTION__);
#ifdef ISA_SUPPORT
	isa_unregister_driver();
 isa_fail:
	DBGINFO("%s ISA load failure!\n", __FUNCTION__);
#endif
	uart_unregister_driver(&seamac_uart_driver);
 uart_fail:	
	return ret;
}

// Module termination
static void __exit seamac_cleanup_module(void)
{
	DBGINFO("%s\n", __FUNCTION__);

#ifdef PCMCIA_SUPPORT
	pcmcia_unregister_driver(&seamac_pcmcia_driver);
#endif

	pci_unregister_driver(&seamac_pci_driver);

#ifdef ISA_SUPPORT
	isa_unregister_driver();
#endif

	uart_unregister_driver(&seamac_uart_driver);
}


MODULE_AUTHOR("Sealevel Systems, Inc. <support@sealevel.com>");
MODULE_DESCRIPTION("Sealevel Synchronous Serial Driver");
MODULE_LICENSE("GPL v2");

MODULE_DEVICE_TABLE(pci, seamac_pci_tbl);
#ifdef PCMCIA_SUPPORT
MODULE_DEVICE_TABLE(pcmcia, seamac_pcmcia_tbl);
#endif

module_init(seamac_init_module);
module_exit(seamac_cleanup_module);
