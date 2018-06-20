# #################################################################################
#   UINTC makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../common/syslib.mak
include uintclib.mak
include armTest.mak

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -lrt -lpthread -lname_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) -lrm -lqmss_$(SYSLIB_DEVICE) 		\
		   -lpktlib_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE) 					\
		   -lhplib-no-osal_$(SYSLIB_DEVICE)
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib 	\
		   -L$(ARAGODIR)/lib

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

# This builds the core Library.
lib: uintcLib

# This cleans the core Library.
libClean: uintcLibClean

# Clean: This cleans lib and unit tests
clean: libClean testClean

# Test: This builds all the unit tests
test: armTest

# Test Clean: This cleans all the unit tests
testClean: armTestClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the UINTC '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'lib                  -> Build the Core Library only'
	@echo 'libClean             -> Clean the Core Library only'
	@echo 'armTest              -> Build ARM Unit Test'
	@echo 'armTestClean         -> Clean ARM Unit Test'
	@echo '****************************************************************************************'

docs:
	$(DOXYGEN) docs/Doxyfile

all: lib test

