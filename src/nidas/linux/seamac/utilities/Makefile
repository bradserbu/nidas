#
# General build makefile, only for examples.

APPDIR ?= /usr/bin
MAN1DIR ?= /usr/share/man/man1
INCLUDE ?= $(shell pwd)/../include
HEADER = $(INCLUDE)/seamac.h

CC ?=      /usr/bin/gcc
MKDIR ?=   /bin/mkdir
CP ?=      /bin/cp
RM ?=      /bin/rm
CHMOD ?=   /bin/chmod


## ============================================================================
MODEL = Configuration/Test Utility
OBJS = setseamac.o
APPS = setseamac
MAN1 = setseamac.1.gz

CFLAGS = -Wall -O3 -I$(INCLUDE)


## ============================================================================
all: $(APPS)
	@echo " *$(MODEL) are prepared"
	@echo

$(HEADER):
	@echo
	@echo "  ERROR!  Can't find include file!"
	@echo
	@echo "       I suggest you obtain another copy of this distribution."
	@echo
	@exit 1

$(OBJS): %.o: %.c $(HEADER)
	@echo "   Compiling $(MODEL) source: $*.o"
	@$(CC) $(CFLAGS) -c $< -o $@

$(APPS): %: %.o
	@echo "   Linking $^"
	@$(CC) $(CFLAGS) $^ -o $@

objs: $(OBJS)
	@echo " *$(MODEL) objects prepared"
	@echo

install: $(APPS)
	@echo "   Installing ${MODEL} to ${APPDIR}"
	@for n in $(APPS); do $(CP) -r $$n ${APPDIR}; done
	@for n in $(APPS); do $(CHMOD) 755 ${APPDIR}/$$n; done
	@for n in $(MAN1); do $(CP) -r $$n ${MAN1DIR}; done
	@echo " *${MODEL} installed"
	@echo

uninstall:
	@echo "   Uninstalling ${MODEL} from ${APPDIR}"
	@for n in $(APPS); do $(RM) -f ${APPDIR}/$$n; done
	@for n in $(MAN1); do $(RM) -f ${MAN1DIR}/$$n; done
	@echo " *${MODEL} uninstalled"
	@echo 

clean: 
	@echo "   Removing ${MODEL} build files"
	@$(RM) -rf $(OBJS) $(APPS)
	@echo " *${MODEL} build files removed"
	@echo
