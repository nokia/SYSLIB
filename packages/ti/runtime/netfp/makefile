###################################################################################
#   NETFP makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

include ../common/syslib.mak
include ./netfplib.mak
include ./dspTest.mak
include ./armTest.mak
include ./dspArmTest.mak
#-include ./armUtils.mak
#-include ./dspUtils.mak

###################################################################################
# Unit Test defintions
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(SNOW3G_INSTALL_PATH) -I$(ARAGODIR)/include
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib    \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib     \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib        \
           -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(SNOW3G_INSTALL_PATH)/ti/snow3g/lib             \
           -L$(ARAGODIR)/lib
STD_LIBS = -lrt -lpthread -lcppi_$(SYSLIB_DEVICE) -lrm -lqmss_$(SYSLIB_DEVICE) -lpktlib_$(SYSLIB_DEVICE)	\
           -lmsgcom_$(SYSLIB_DEVICE) -lnetfp_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE)                       \
           -luintc_$(SYSLIB_DEVICE) -lname_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE)                       \
           -lhplib-no-osal_$(SYSLIB_DEVICE) -lsa -lsnow3g
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif
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
lib: netfpLib

# This cleans the core Library.
libClean: netfpLibClean

# Clean: This cleans the core library and all the unit tests
clean: libClean armTestClean dspArmClean dspArmCore0Clean dspArmCore1Clean dspCore0Clean dspCore1Clean utilsClean queueDumpClean

# Test: This builds all the unit tests
test: armTest dspArm dspArmCore0 dspArmCore1 dspCore0 dspCore1

# Test Clean: This cleans all the unit tests
testClean: armTestClean dspArmClean dspArmCore0Clean dspArmCore1Clean dspCore0Clean dspCore1Clean queueDumpClean utilsClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the NETFP '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'dspArm               -> Build the ARM Unit Test'
	@echo 'dspArmClean          -> Clean the ARM Unit Test'
	@echo 'dspArmCore0          -> Build the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore0Clean     -> Clean the DSP-ARM Core0 Unit Test'
	@echo 'dspArmCore1          -> Build the DSP-ARM Core1 Unit Test'
	@echo 'dspArmCore1Clean     -> Clean the DSP-ARM Core1 Unit Test'
	@echo 'dspCore0             -> Build the DSP Core0 Unit Test'
	@echo 'dspCore0Clean        -> Clean the DSP Core0 Unit Test'
	@echo 'dspCore1             -> Build the DSP Core1 Unit Test'
	@echo 'dspCore1Clean        -> Clean the DSP Core1 Unit Test'
	@echo 'queueDump            -> Build the DSP Queue Dump Utility'
	@echo 'queueDumpClean       -> Clean the DSP Queue Dump Utility'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test docs


