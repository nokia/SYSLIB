# #################################################################################
#   MEMLOG makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include ./memloglib.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lrt -lpthread  -lrm -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) -lhplib-no-osal_$(SYSLIB_DEVICE)  	\
	   -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE)  \
           -lresmgr_$(SYSLIB_DEVICE) -lmemlog_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib  -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib  -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib  \
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib 		\
                   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/memlog/lib  -L$(ARAGODIR)/lib
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the defaults
default: all

# This builds the core Library.
lib: memlogLib

# This cleans the core Library.
libClean: memlogLibClean

# Clean: This cleans the core library
clean: libClean testClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the MEMLOG'
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

