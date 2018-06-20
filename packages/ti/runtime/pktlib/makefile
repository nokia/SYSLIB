# #################################################################################
#   PKTLIB makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include pktlib.mak
include armTest.mak
include dspTest.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lpthread -lname_$(SYSLIB_DEVICE) -lrt -lcppi_$(SYSLIB_DEVICE) -lrm -lqmss_$(SYSLIB_DEVICE) 	\
		   -lpktlib_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -lhplib-no-osal_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib \
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib   -L$(ARAGODIR)/lib
ifeq ($(ARM_DSP_DOWNLOAD), 1)
	# Loading DSP build from ARM using remoteproc
	DSP_LDFLAGS += --ram_model --define=ARM_DSP_DOWNLOAD=1 
	DSP_CFLAGS += -DARM_DSP_DOWNLOAD=1
	DSP_XSFLAGS += -DARM_DSP_DOWNLOAD=1
endif
###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the core Library.
lib: pktLib

# This cleans the core Library.
libClean: pktLibClean

# Test: This builds all the unit tests
test: dspCore0 dspCore1 armTest

# Test Clean: This cleans all the unit tests
testClean: dspCore0Clean dspCore1Clean armTestClean

# Clean: This cleans out all the objects
clean: libClean testClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the PKTLIB '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'dspCore0             -> Build the DSP Core0 Unit Test'
	@echo 'dspCore0Clean        -> Clean the DSP Core0 Unit Test'
	@echo 'dspCore1             -> Build the DSP Core1 Unit Test'
	@echo 'dspCore1Clean        -> Clean the DSP Core1 Unit Test'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

