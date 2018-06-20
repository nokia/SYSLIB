# #################################################################################
#   NETFP Proxy makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak

.PHONY: clean all

###################################################################################
# Setup the build options to build V1 or V2
###################################################################################
NETFP_PROXY_V1_SUPPORT=yes

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include -I$(ARAGODIR)/include/libnl3
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib 		\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib  -L$(ARAGODIR)/lib
STD_LIBS = -lnetfp_$(SYSLIB_DEVICE) -lrt -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) -lsa 				\
		   -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -lrm 				\
		   -lhplib-no-osal_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE)  				\
		   -lnetfpsmlog -lDDAL \
		   -lpthread -lrt -lpthread -lnl-3 -lnl-route-3 -rdynamic -lresolv -ldl  					\
		   -luintc_$(SYSLIB_DEVICE) 
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif

# Sources
NETFP_PROXY_SOURCES = src/core.c src/netmgr.c src/route.c src/iface.c src/osal.c 				\
		      src/listlib.c src/test.c                             \
	              src/netfp_proxy_custom.c src/netfp_proxy_logging.c 

ifeq ($(NETFP_PROXY_V1_SUPPORT), yes)
NETFP_PROXY_SOURCES += src/neigh.c
else
NETFP_PROXY_SOURCES += src/neigh2.c src/arp.c src/ndisc.c src/pmtu4.c src/pmtu6.c
endif

# Objects
NETFP_PROXY_OBJECTS = $(NETFP_PROXY_SOURCES:.c=.o)

# Dependency
NETFP_PROXY_DEPENDS = $(NETFP_PROXY_SOURCES:.c=.d)

# Application name
NETFP_PROXY_OUT = netfpproxy_$(SYSLIB_DEVICE).out

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean lib libClean test testClean docs help

netfpProxyClean:
	@echo 'Cleaning the NETFP Proxy'
	rm -f $(NETFP_PROXY_OBJECTS) $(NETFP_PROXY_OUT) $(NETFP_PROXY_DEPENDS)

clean: netfpProxyClean armTestClean pluginClean

$(NETFP_PROXY_OUT): $(NETFP_PROXY_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(NETFP_PROXY_OBJECTS) -Wl,--end-group -o $(NETFP_PROXY_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP Proxy'
	@echo '******************************************************************************'

# Test: This builds the Unit Test
test: armTest

# Test Clean: This cleans all the unit tests
testClean: armTestClean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the NETFP Proxy '
	@echo 'clean                -> Clean out all the objects'
	@echo 'docs                 -> Build the library documentation'
	@echo 'armTest              -> Build the ARM Unit Test'
	@echo 'armTestClean         -> Clean the ARM Unit Test'
	@echo 'plugin               -> Build the NETFP Plugin'
	@echo 'pluginClean          -> Clean the NETFP Plugin'
	@echo 'test                 -> Build all the Unit Test applicable for the module'
	@echo 'testClean            -> Clean all the Unit Tests applicable for the module'
	@echo 'all                  -> Built the executable & Unit Tests'
	@echo '****************************************************************************************'

# Build the executable & test
all: $(NETFP_PROXY_OUT) test plugin

# Dependency handling
-include $(NETFP_PROXY_SOURCES:.c=.d)

include armTest.mak
include plugin.mak

