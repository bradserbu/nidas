#
# General build makefile.  (Scripts)

SUAPPDIR ?= /sbin

CP ?= /bin/cp
RM ?= /bin/rm


## ============================================================================
FILES = seamacload seamacunload


## ============================================================================
all: $(FILES)
	@echo " *Scripts dir is prepared"
	@echo

objs: $(FILES)
	@echo " *Scripts dir is prepared"
	@echo

install: $(FILES)
	@echo "   Installing scripts to $(SUAPPDIR)"
	@for n in $(FILES); do $(CP) -r $$n $(SUAPPDIR) || exit 1; done
	@echo " *Done installing scripts"
	@echo

uninstall:
	@echo "   Uninstalling scripts from $(SUAPPDIR)"
	@for n in $(FILES); do $(RM) -rf ${SUAPPDIR}/$$n || exit 1; done
	@echo " *Scripts removed"
	@echo

clean: $(FILES)
	@echo " *Nothing to clean in Scripts dir"
	@echo
