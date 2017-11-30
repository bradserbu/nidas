/*
 *	Sealevel Systems Z85X30 driver shared functions
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

#include <linux/pci.h>
extern int z85_pci_probe(struct pci_dev *dev, const struct pci_device_id *id);
extern void z85_pci_remove(struct pci_dev *dev);


#ifdef PCMCIA_SUPPORT
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>
extern int z85_pcmcia_attach(struct pcmcia_device *dev);
extern void z85_pcmcia_detach(struct pcmcia_device *dev);
#endif

#ifdef ISA_SUPPORT
extern int z85_isa_attach(int isa_io, int isa_irq);
extern void z85_isa_detach(int index);
#endif

