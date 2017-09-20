#
# File:
#    Makefile
#
# Description:
#    Makefile for the coda primary and secondary readout lists
#    running on an Intel-based controller running Linux
#
#
#
DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

# Plug in your default primary readout lists here..
VMEROL	= ti_master_list.so

# Override VMEROL with command line goal
ifneq (,$(findstring .so,$(MAKECMDGOALS)))
	VMEROL	= $(MAKECMDGOALS)
else
	ifneq (,$(findstring .o,$(MAKECMDGOALS)))
		VMEROL	= $(MAKECMDGOALS)
	endif
endif

# Add shared library dependencies here.  (jvme, ti, are already included)
ROLLIBS	+= -lremex

ifeq (${ARCH}, PPC)
	OS=VXWORKS
endif

ifndef ARCH
	ifdef LINUXVME_LIB
		ARCH=$(shell uname -m)
		OS=LINUX
	else
		ARCH=PPC
		OS=VXWORKS
	endif
endif

# For vxWorks, where to find jvme.h
JVME_INC		= ${LINUXVME_INC}

# Directory for more up to date includes
MYINC	= feature-vxCompat_include

ifdef LINUX
LINUXVME_LIB		?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC		?= ${CODA}/extensions/linuxvme/include
endif

DT:=\""$(shell date)"\"

WARN_SUPPRESS		=
WARN_SUPPRESS		+= -Wno-unused
WARN_SUPPRESS		+= -Wno-format

# DEFs for compiling primary readout lists
ifeq (${OS}, LINUX)
CC			= gcc
AR                      = ar
RANLIB                  = ranlib
ifdef DEBUG
	CFLAGS		= -g -Wall
else
	CFLAGS		= -O3
endif
CFLAGS			+= -m32 -L. -L${LINUXVME_LIB}
CFLAGS			+= -DLINUX -DDAYTIME=$(DT)
LIBS			= -lrt -lpthread -ljvme -lti $(ROLLIBS)
INCS			= -I. \
				-isystem $(MYINC) \
				-isystem ${LINUXVME_INC} \
				-isystem ${CODA}/common/include
DEPS			+= $(VMEROL:%.so=%.d)
else #VXWORKS
CC			= ccppc
ifdef DEBUG
	CFLAGS		= -Wall -g
else
	CFLAGS		= -O2
endif
CFLAGS			+= -fno-builtin -D_GNU_TOOL -DVXWORKS -DVXWORKSPPC
CFLAGS			+= -mcpu=604 -mlongcall
CFLAGS			+= -fno-for-scope
CFLAGS			+= -fno-writable-strings
CFLAGS			+= -fvolatile -DDAYTIME=$(DT)
CFLAGS			+= -c -DCPU=PPC604 -w
INCS			= -I. -isystem ${CODA}/common/include -isystem ${JVME_INC}
DEPS			+= $(VMEROL:%.o=%.d)
endif #LINUX

CFLAGS			+= ${WARN_SUPPRESS}

CCRL                    = ${CODA_BIN}/ccrl

ifeq (${OS}, LINUX)
all: $(VMEROL)

%.so: %.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) ${INCS} ${LIBS} \
		-DINIT_NAME=$(@:.so=__init) -o $@ $<
else
all: $(VMEROL:%.so=%.o)

%.o: %.c
	@echo " CC     $@"
	${Q}${CC} ${CFLAGS} ${INCS} -DINIT_NAME=$(@:.o=__init) -o $@ $<

endif

%.c: %.crl
	@echo " CCRL   $@"
	@${CCRL} $<

clean:
	@rm -f $(VMEROL) $(DEPS)

realclean:
	@rm -f *.so *.d *.o *~

ifeq (${OS}, LINUX)
%.d: %.c
	@echo " DEP    $@"
	@set -e; rm -f $@; \
	$(CC) -MM -shared $(CFLAGS) $(INCS)  $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.so $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
else
%.d: %.c
	@echo " DEP    $@"
	@set -e; rm -f $@; \
	$(CC) -MM $(CFLAGS) $(INCS)  $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
endif

ifeq (,$(findstring clean,$(MAKECMDGOALS)))
-include $(DEPS)
endif

.PHONY: all
