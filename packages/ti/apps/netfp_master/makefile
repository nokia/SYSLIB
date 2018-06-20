# #################################################################################
#   NETFP Master makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak
include armTest.mak

.PHONY: clean all

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib	-L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib	 \
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib \
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib   \
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib -L$(ARAGODIR)/lib
STD_LIBS = \
    -lrt -lsa \
    -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) \
    -lnetfpsmlog -lDDAL -lpktlib_$(SYSLIB_DEVICE) \
    -lpthread -lm -lrm -lhplib-no-osal_$(SYSLIB_DEVICE)

LIB_DEPS = \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib/libname_$(SYSLIB_DEVICE).a \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib/libresmgr_$(SYSLIB_DEVICE).a \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib/libmsgcom_$(SYSLIB_DEVICE).a \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib/libuintc_$(SYSLIB_DEVICE).a \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib/libnetfp_$(SYSLIB_DEVICE).a \
  $(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib/libjosh_$(SYSLIB_DEVICE).a

# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif

# Sources
NETFP_MASTER_SOURCES = src/master.c src/osal.c src/qos.c src/listlib.c src/mgmt.c src/netfp_master_logging.c

# Objects
NETFP_MASTER_OBJECTS = $(NETFP_MASTER_SOURCES:.c=.o)

# Dependency
NETFP_MASTER_DEPENDS = $(NETFP_MASTER_SOURCES:.c=.d)

# Server name
NETFP_MASTER_OUT = netfp_master_$(SYSLIB_DEVICE).out

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

netfpMasterClean:
	@echo 'Cleaning the NETFP Mastetr'
	rm -f $(NETFP_MASTER_OBJECTS) $(NETFP_MASTER_OUT) $(NETFP_MASTER_DEPENDS)

# Clean: This cleans the core library
clean: netfpMasterClean armTestClean

$(NETFP_MASTER_OUT): $(NETFP_MASTER_OBJECTS) $(LIB_DEPS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(NETFP_MASTER_OBJECTS) $(LIB_DEPS)  -Wl,--end-group -o $(NETFP_MASTER_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP Master'
	@echo '******************************************************************************'

# Test: This builds the Unit Test
test: armTest

# Test Clean: This cleans all the unit tests
testClean: armTestClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the NETFP Master '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo 'all                  -> Built the executable & Unit Tests'
	@echo '****************************************************************************************'

# Build the executable & test
all: $(NETFP_MASTER_OUT) test

# Dependency handling
-include $(NETFP_MASTER_SOURCES:.c=.d)


