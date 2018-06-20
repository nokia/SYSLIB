# #################################################################################
#   DAT makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include ./datlib.mak
include ./dspArmTest.mak
include ./dspArmMemlog.mak
include ./dspTest.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(CUIA_INSTALL_PATH)/include -I$(ARAGODIR)/include
STD_LIBS = -lrt -lpthread -ldat_$(SYSLIB_DEVICE) -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) 	\
		   -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) -lnetfp_$(SYSLIB_DEVICE) 	\
		   -luintc_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -lpthread -lsa -lrm  		\
		   -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) -lhplib-no-osal_$(SYSLIB_DEVICE)  	\
		   -lmemlog_$(SYSLIB_DEVICE) -lLoggerStreamer2 -lLogSync
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib  -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib		\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib  -L$(SYSLIB_INSTALL_PATH)/ti/runtime/dat/lib		\
                   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/memlog/lib -L$(CUIA_INSTALL_PATH)/lib  -L$(ARAGODIR)/lib
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
default: clean all

# This builds the core Library.
lib: datLib

# This cleans the core Library.
libClean: datLibClean

# Clean: This cleans the core library
clean: libClean testClean

# DSP only test targets
dspTest: dspCore0 dspCore1
dspTestClean: dspCore0Clean dspCore1Clean

# ARM-DSP consumer test targets
dspArmTest: dspArmCore0 dspArmCore1 dspArm
dspArmTestClean: dspArmCore0Clean dspArmCore1Clean dspArmClean

# ARM-DSP MEMLOG test targets
memlogTest: dspArmMemlogCore0 dspArmMemlogCore1
memlogTestClean: dspArmMemlogCore0Clean dspArmMemlogCore1Clean

# Test: This builds all the unit tests
test: dspArmTest memlogTest dspTest

# Test Clean: This cleans all the unit tests
testClean: dspArmTestClean memlogTestClean dspTestClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the DAT '
	@echo 'clean                   -> Clean out all the objects'
	@echo 'docs                    -> Build the library documentation'
	@echo 'lib                     -> Build the Core Library only'
	@echo 'libClean                -> Clean the Core Library only'
	@echo 'dspArmTest              -> Build the DSP ARM Consumer Test suite'
	@echo 'dspArmTestClean         -> Clean the DSP ARM Consumer Unit Test suite'
	@echo 'dspArm                  -> Build the ARM Unit Test'
	@echo 'dspArmClean             -> Clean the ARM Unit Test'
	@echo 'dspArmCore0             -> Build the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore0Clean        -> Clean the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore1             -> Build the DSP-ARM Core1 Unit Test'
	@echo 'dspArmCore1Clean        -> Clean the DSP-ARM Core1 Unit Test'
	@echo 'memlogTest              -> Build the memlog Unit Test Suite'
	@echo 'memlogTestClean         -> Clean the memlog Unit Test Stuie'
	@echo 'dspArmMemlogClean       -> Clean the ARM Unit Test'
	@echo 'dspArmMemlogCore0       -> Build the DSP-ARM Core0 Unit Test'
	@echo 'dspArmMemlogCore0Clean  -> Clean the DSP-ARM Core0 Unit Test'
	@echo 'dspArmMemlogCore1       -> Build the DSP-ARM Core1 Unit Test'
	@echo 'dspArmMemlogCore1Clean  -> Clean the DSP-ARM Core1 Unit Test'
	@echo 'test                    -> Build all the Unit Test applicable for the module'
	@echo 'testClean               -> Clean all the Unit Tests applicable for the module'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

