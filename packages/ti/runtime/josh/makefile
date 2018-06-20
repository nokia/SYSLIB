# #################################################################################
#   JOSH makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include ./joshlib.mak
include ./dspTest.mak

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

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the core Library.
lib: joshLib

# This cleans the core Library.
libClean: joshLibClean

# Clean: This cleans the core library
clean: libClean testClean

# Test: This builds all the unit tests
test: dspCore0 dspCore1

# Test Clean: This cleans all the unit tests
testClean: dspCore0Clean dspCore1Clean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the JOSH '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'dspCore0             -> Build the DSP Core0 Unit Test'
	@echo 'dspCore0Clean        -> Clean the DSP Core0 Unit Test'
	@echo 'dspCore1             -> Build the DSP Core1 Unit Test'
	@echo 'dspCore1Clean        -> Clean the DSP Core1 Unit Test'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

