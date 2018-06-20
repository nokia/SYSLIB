# #################################################################################
#   NETFP Server makefile
#
#  NOTE:
#      (C) Copyright 2013-2014 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak

.PHONY: clean all

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
ifneq ($(SA3GPP_INSTALL_PATH),)
  STD_INCL += -I$(SA3GPP_INSTALL_PATH)
endif
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib 		\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(ARAGODIR)/lib
STD_LIBS = -lnetfp_$(SYSLIB_DEVICE) -lrt -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) -lsa 				\
		   -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) 	  				\
		   -luintc_$(SYSLIB_DEVICE) -lrm -lhplib-no-osal_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) 			\
		   -lqmss_$(SYSLIB_DEVICE) -lpthread -lsa3gpp -lnetfpsmlog -lDDAL
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif

# Sources
NETFP_SERVER_SOURCES = src/netfp_server.c src/netfp_masterServerIf.c src/osal.c src/netfpServer_logging.c

# Objects
NETFP_SERVER_OBJECTS = $(NETFP_SERVER_SOURCES:.c=.o)

# Dependency
NETFP_SERVER_DEPENDS = $(NETFP_SERVER_SOURCES:.c=.d)

# Server name
NETFP_SERVER_OUT = netfp_server_$(SYSLIB_DEVICE).out

clean:
	@echo 'Cleaning the NETFP Server'
	rm -f $(NETFP_SERVER_OBJECTS) $(NETFP_SERVER_OUT) $(NETFP_SERVER_DEPENDS)

$(NETFP_SERVER_OUT): $(NETFP_SERVER_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(NETFP_SERVER_OBJECTS) -Wl,--end-group -o $(NETFP_SERVER_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP Server'
	@echo '******************************************************************************'

all: $(NETFP_SERVER_OUT)

# Dependency handling
-include $(NETFP_SERVER_SOURCES:.c=.d)

