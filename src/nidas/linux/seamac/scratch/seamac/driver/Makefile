#
# $Id: Makefile,v 1.4 2011/12/28 15:33:03 thomasw Exp $
#
# General build makefile.  (Drivers)

KDIR ?= /lib/modules/$(shell uname -r)/build
MODDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/serial
INCLUDE ?= $(shell pwd)/../include

CP ?= /bin/cp
RM ?= /bin/rm
DEPMOD ?= /sbin/depmod


## ============================================================================
HEADER = $(INCLUDE)/seamac.h
DEPENS = seamac_core.c z85230.c z85230.h ssi9198.c ssi9198.h $(HEADER)
OBJS = seamac_core.o z85230.o ssi9198.o
MODULES = seamac.ko
TARGET = seamac

## ============================================================================




#ECFLGS += " -I$(INCLUDE) -DPCMCIA_SUPPORT -DISA_SUPPORT"
ECFLGS += " -I$(INCLUDE) -DISA_SUPPORT"
#ECFLGS += " -I$(INCLUDE) -DPCMCIA_SUPPORT"
#ECFLGS += " -I$(INCLUDE)"


# -----------------------------------------------------------------------------
#if KERENLRELEASE has been initialized, we are being run from the kbuild system
# -----------------------------------------------------------------------------
ifneq ($(KERNELRELEASE),)
  obj-m += $(TARGET).o
  seamac-objs := $(OBJS)

# -----------------------------------------------------------------------------
# Otherwise we are being run from somewhere else and need to start up kbuild
# -----------------------------------------------------------------------------
else

all: $(MODULES)
	@echo " *Kernel Driver Module compiled"
	@echo

objs: all

$(HEADER):
	@echo
	@echo "  ERROR!  Can't find include file!"
	@echo
	@echo "       I suggest you obtain another copy of this distribution."
	@echo
	@exit 1

$(KDIR):
	@echo
	@echo "  ERROR!  No kernel source found in $(KDIR)"
	@echo
	@echo "       If the above directory is not the correct location"
	@echo "       for your source, please update the KDIR environment"
	@echo "       variable or pass it to the Makefile directly --"
	@echo "       make KDIR=/usr/src/linux"
	@echo
	@exit 2

$(KDIR)/.config:
	@echo
	@echo "  ERROR!  No configured kernel source found in $(KDIR)"
	@echo
	@echo "       If your kernel source is located at that directory"
	@echo "       you probably need to configure it.  First change"
	@echo "       directory to your kernel source (cd /KDIR) then run"
	@echo "       make xconfig, make menuconfig, or make oldconfig"
	@echo "       to configure your kernel source.  This step is"
	@echo "       necessary because some header files are generated at"
	@echo "       this step."
	@echo
	@exit 3

$(MODULES): $(KDIR) $(KDIR)/.config $(DEPENS)
	@echo "   Building kernel module(s): $(MODULES)"
	@$(MAKE) -s -C $(KDIR) SUBDIRS=$(shell pwd) EXTRA_CFLAGS=$(ECFLGS) modules

install: $(MODULES)
	@echo "   Installing modules to $(MODDIR)"
	@mkdir --parents $(MODDIR)
	@for n in $(MODULES); do $(CP) $$n $(MODDIR); done
	@$(DEPMOD) -a
	@echo " *Done Installing $(MODULES)"
	@echo

uninstall:
	@echo "   Uninstalling modules from $(MODDIR)"
	@for n in $(MODULES); do $(RM) -f ${MODDIR}/$$n; done
	@$(DEPMOD) -a
	@echo " *$(MODULES) removed"
	@echo

clean:
	@echo "   Removing driver module build files"
	@$(RM) -rf *~ *.o *.ko *mod* .*.o* .*.ko* .*.mod* .tmp* *.symvers 
	@echo " *Module build files removed"
	@echo

endif
