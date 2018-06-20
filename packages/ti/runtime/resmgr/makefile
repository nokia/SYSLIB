# #################################################################################
#   RESMGR makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include resmgrlib.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lrt -lpthread -lrm -lresmgr_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib    \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib     \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib        \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(SNOW3G_INSTALL_PATH)/ti/snow3g/lib             \
           -L$(ARAGODIR)/lib

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the core Library.
lib: resmgrLib

# This cleans the core Library.
libClean: resmgrLibClean

# Clean: This cleans the core library
clean: libClean

# Test: This builds all the unit tests
test:

# Test Clean: This cleans all the unit tests
testClean:

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the RESMGR '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

