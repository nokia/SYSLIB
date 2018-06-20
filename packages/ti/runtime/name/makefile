# #################################################################################
#   NAME makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include ./namelib.mak
include ./dspTest.mak
include ./armTest.mak
include ./utils.mak
include ./dspArmTest.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lrt -lpthread -lname_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) -lrm -lqmss_$(SYSLIB_DEVICE) 		\
		   -lpktlib_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE) 					\
	   	   -ljosh_$(SYSLIB_DEVICE)	-lhplib-no-osal_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib  -L$(ARAGODIR)/lib

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the core Library.
lib: nameLib

# This cleans the core Library.
libClean: nameLibClean

# Clean: This cleans all the objects
clean: libClean testClean utilsClean

# Test: This builds all the unit tests
test: armTest dspCore0 dspCore1 dspArmCore0 dspArmCore1 dspArm

# Test Clean: This cleans all the unit tests
testClean: armTestClean dspCore0Clean dspCore1Clean dspArmCore0Clean dspArmCore1Clean dspArmClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the NAME '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'utils                -> Build the Utils only'
	@echo 'utilsClean           -> Clean the Utils only'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'dspArm               -> Build the DSP-ARM ARM Executable Unit Test'
	@echo 'dspArmClean          -> Clean the DSP-ARM ARM Executable Unit Test'
	@echo 'dspArmCore0          -> Build the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore0Clean     -> Clean the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore1          -> Build the DSP-ARM Core1 Unit Test'
	@echo 'dspArmCore1Clean     -> Clean the DSP-ARM Core1 Unit Test'
	@echo 'dspCore0             -> Build the DSP Core0 Unit Test'
	@echo 'dspCore0Clean        -> Clean the DSP Core0 Unit Test'
	@echo 'dspCore1             -> Build the DSP Core1 Unit Test'
	@echo 'dspCore1Clean        -> Clean the DSP Core1 Unit Test'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo 'utils                -> Build all the Unit Test applicable for the module'
	@echo 'utilsClean           -> Clean all the Unit Tests applicable for the module'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test utils

