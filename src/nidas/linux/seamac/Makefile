## 2007, Copyright University Corporation for Atmospheric Research
# if KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language
#
export CC       = armbe-linux-gcc
ifneq ($(KERNELRELEASE),)
        obj-m := seamac.o
        seamac-objs:=seamac_core.o z85230.o ssi9198.o    

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) CFLAGS_MODULE="-DMODULE -DISA_SUPPORT  $(KCFLAGS)" modules

endif
