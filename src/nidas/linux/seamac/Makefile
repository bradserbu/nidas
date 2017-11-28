#
# General build makefile.  (Main)

## These variable MUST be accurate for the installation to succeed.
export KDIR = /lib/modules/$(shell uname -r)/build
export MODDIR = /lib/modules/$(shell uname -r)/kernel/drivers/serial

## Change these if you wish to change default installation directories.
export INCLUDEDIR = /usr/local/include
export SUAPPDIR = /sbin
export APPDIR = /usr/bin

## Change these if your system is configured differently.
export CP 	= /bin/cp
export RM 	= /bin/rm
export MKDIR 	= /bin/mkdir
export CHMOD    = /bin/chmod
export CC 	= /usr/bin/gcc

export INCLUDE = $(shell pwd)/include
## ============================================================================
## The following values are for internal package management and should only
## be changed if you know what you are doing.
PRODUCT = SeaMAC
VERSION = 1.6.6
DISTRO = SM010606

#CVS project name, used for install scripts
PROJECT = seamac
SUBDIRS = include driver utilities scripts examples
CVS = `find -iname \CVS`
SVN = `find -iname \.svn`

TAR 	= /bin/tar
GZIP 	= /bin/gzip

## ============================================================================
all: subdirs

objs:
	@echo "----------------------------------------------------------------"
	@echo "Building all objects."
	@echo "----------------------------------------------------------------"
	@echo
	@for n in $(SUBDIRS); do $(MAKE) -s -C $$n objs || exit 1; done
	@echo "----------------------------------------------------------------"
	@echo "Done building objects."
	@echo "----------------------------------------------------------------"

subdirs:
	@echo "----------------------------------------------------------------"
	@echo "Building all."
	@echo "----------------------------------------------------------------"
	@echo
	@for n in $(SUBDIRS); do $(MAKE) -s -C $$n all || exit 1; done
	@echo "----------------------------------------------------------------"
	@echo "Done building all sources."
	@echo "----------------------------------------------------------------"

clean:
	@echo "----------------------------------------------------------------"
	@echo "Removing all build files."
	@echo "----------------------------------------------------------------"
	@echo
	@for n in $(SUBDIRS); do $(MAKE) -s -C $$n clean || exit 1; done
	@echo "----------------------------------------------------------------"
	@echo "Done removing all build files."
	@echo "----------------------------------------------------------------"

install:
	@echo "----------------------------------------------------------------"
	@echo "Installing $(PRODUCT) v$(VERSION) suite."
	@echo "----------------------------------------------------------------"
	@echo
	@for n in $(SUBDIRS); do $(MAKE) -s -C $$n install || exit 1; done
	@echo "----------------------------------------------------------------"
	@echo "Done installing $(PRODUCT) v$(VERSION) suite."
	@echo "----------------------------------------------------------------"

uninstall:
	@echo "----------------------------------------------------------------"
	@echo "Uninstalling $(PRODUCT) v$(VERSION) suite."
	@echo "----------------------------------------------------------------"
	@echo
	@for n in $(SUBDIRS); do $(MAKE) -s -C $$n uninstall || exit 1; done
	@echo "----------------------------------------------------------------"
	@echo "Done uninstalling $(PRODUCT) v$(VERSION) suite."
	@echo "----------------------------------------------------------------"

cleaninstall: clean install

purge: uninstall clean

distro: clean
	@echo "----------------------------------------------------------------"
	@echo "Building $(PRODUCT) v$(VERSION) distribution package!"
	@echo "----------------------------------------------------------------"
	@$(RM) -rf $(PRODUCT)_$(VERSION).tar.gz
	@$(RM) -rf $(CVS)
	@$(RM) -rf $(SVN)
	@$(RM) -rf ./TESTING
	@$(TAR) -cf $(DISTRO).tar ../${PROJECT}
	@$(GZIP) $(DISTRO).tar
	@echo "----------------------------------------------------------------"
	@echo "Done!"
	@echo "----------------------------------------------------------------"
