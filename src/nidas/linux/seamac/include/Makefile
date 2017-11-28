#
# General build makefile.  (Includes)

INCLUDEDIR ?= /usr/local/include

CP ?= /bin/cp
RM ?= /bin/rm


## ============================================================================
FILES = seamac.h


## ============================================================================
all: $(FILES)
	@echo " *Include dir is prepared"
	@echo

objs: $(FILES)
	@echo " *Include dir is prepared"
	@echo

install: $(FILES)
	@echo "   Installing header files to $(INCLUDEDIR)"
	@for n in $(FILES); do $(CP) -r $$n $(INCLUDEDIR) || exit 1; done
	@echo " *Done installing header files"
	@echo

uninstall:
	@echo "   Uninstalling header files from $(INCLUDEDIR)"
	@for n in $(FILES); do $(RM) -rf ${INCLUDEDIR}/$$n || exit 1; done
	@echo " *Header files removed"
	@echo

clean: $(FILES)
	@echo " *Nothing to clean in Include dir"
	@echo
