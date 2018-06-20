# #################################################################################
#   DAT Server makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak

.PHONY: clean all

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(CUIA_INSTALL_PATH)/include -I$(ARAGODIR)/include
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib  	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/dat/lib    -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib  -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib		\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/memlog/lib -L$(CUIA_INSTALL_PATH)/lib -L$(ARAGODIR)/lib
STD_LIBS = -lrt -ldat_$(SYSLIB_DEVICE) -lname_$(SYSLIB_DEVICE) -lnetfp_$(SYSLIB_DEVICE) -lsa 				\
		   -ljosh_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE) 			  			\
		   -lpktlib_$(SYSLIB_DEVICE) -lresmgr_$(SYSLIB_DEVICE) -lrm -lhplib-no-osal_$(SYSLIB_DEVICE) 		\
		   -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) -lmemlog_$(SYSLIB_DEVICE) -lpthread -lLoggerStreamer2 -lLogSync
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
STD_LIBS += -lpa2
else
STD_LIBS += -lpa
endif

# Sources
DAT_SERVER_SOURCES = src/dat_server.c src/osal.c

# Objects
DAT_SERVER_OBJECTS = $(DAT_SERVER_SOURCES:.c=.o)

# Dependency
DAT_SERVER_DEPENDS = $(DAT_SERVER_SOURCES:.c=.d)

# Additional compilation flags
ARM_CFLAGS += -Dxdc_target_name__=GCArmv5T -Dxdc_target_types__=gnu/targets/arm/std.h

# Server name
DAT_SERVER_OUT = dat_server_$(SYSLIB_DEVICE).out

default: all

clean:
	@echo 'Cleaning the DAT Server'
	rm -f $(DAT_SERVER_OBJECTS) $(DAT_SERVER_OUT) $(DAT_SERVER_DEPENDS)

$(DAT_SERVER_OUT): $(DAT_SERVER_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(DAT_SERVER_OBJECTS) -Wl,--end-group -o $(DAT_SERVER_OUT)
	@echo '******************************************************************************'
	@echo 'Built the DAT Server'
	@echo '******************************************************************************'

all: $(DAT_SERVER_OUT)

# Dependency handling
-include $(DAT_SERVER_SOURCES:.c=.d)

