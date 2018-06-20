# #################################################################################
#   MSGCOM makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include ./msgcomlib.mak
include ./dspTest.mak
include ./armTest.mak
include ./dspArmTest.mak
include ./virtualTest.mak
include ./stressTest.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lrt -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) -lpthread -lcppi_$(SYSLIB_DEVICE) -lrm 		\
		   -lqmss_$(SYSLIB_DEVICE) -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE)						\
		   -lresmgr_$(SYSLIB_DEVICE) -lhplib-no-osal_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib	 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib  	\
		   -L$(ARAGODIR)/lib
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
lib: msgcomLib

# This cleans the core Library.
libClean: msgcomLibClean

# Clean: This cleans the core library
clean: libClean testClean

# Test: This builds all the unit tests
test: armTest dspCore0 dspCore1 dspArm virtualCore0 virtualCore1 stressCore0 stressCore1 stress

# Test Clean: This cleans all the unit tests
testClean: armTestClean dspCore0Clean dspCore1Clean dspArmClean stressClean virtualCore0Clean virtualCore1Clean stressCore0Clean stressCore1Clean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the MSGCOM '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'utils                -> Build the Utils only'
	@echo 'utilsClean           -> Clean the Utils only'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'dspArm               -> Build the ARM Unit Test'
	@echo 'dspArmClean          -> Clean the ARM Unit Test'
	@echo 'dspCore0             -> Build the DSP Core0 Unit Test'
	@echo 'dspCore0Clean        -> Clean the DSP Core0 Unit Test'
	@echo 'dspCore1             -> Build the DSP Core1 Unit Test'
	@echo 'dspCore1Clean        -> Clean the DSP Core1 Unit Test'
	@echo 'virtualCore0         -> Build the Virtual MSGCOM Core0 Test'
	@echo 'virtualCore0Clean    -> Clean the Virtual MSGCOM Core0 Test'
	@echo 'virtualCore1         -> Build the Virtual MSGCOM Core1 Test'
	@echo 'virtualCore1Clean    -> Clean the Virtual MSGCOM Core1 Test'
	@echo 'stressCore0          -> Build the Stress MSGCOM Test for Core0'
	@echo 'stressCore0Clean     -> Clean the Stress MSGCOM Test for Core0'
	@echo 'stressCore1          -> Build the Stress MSGCOM Test for Core1'
	@echo 'stressCore1Clean     -> Clean the Stress MSGCOM Test for Core1'
	@echo 'stress               -> Build the Stress MSGCOM Test for ARM'
	@echo 'stressClean          -> Clean the Stress MSGCOM Test for ARM'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

