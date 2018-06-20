# #################################################################################
#   Resource Manager makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak

.PHONY: clean all

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(ARAGODIR)/lib
STD_LIBS = -lrm -lhplib-no-osal_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE) \
		   -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) -lpthread -lnetfpsmlog -lDDAL

# Sources
RESMGR_SERVER_SOURCES = src/resmgr_server.c src/resmgr_system.c src/listlib.c src/osal.c src/resmgr_server_logging.c

# Objects
RESMGR_SERVER_OBJECTS = $(RESMGR_SERVER_SOURCES:.c=.o)

# Dependency
RESMGR_SERVER_DEPENDS = $(RESMGR_SERVER_SOURCES:.c=.d)

# Server name
RESMGR_SERVER_OUT = resmgr_server_$(SYSLIB_DEVICE).out

clean:
	@echo 'Cleaning the RESMGR Server'
	rm -f $(RESMGR_SERVER_OBJECTS) $(RESMGR_SERVER_OUT) $(RESMGR_SERVER_DEPENDS)

$(RESMGR_SERVER_OUT): $(RESMGR_SERVER_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(RESMGR_SERVER_OBJECTS) -lsystemd -Wl,--end-group -o $(RESMGR_SERVER_OUT)
	@echo '******************************************************************************'
	@echo 'Built the RESMGR Server'
	@echo '******************************************************************************'

all: $(RESMGR_SERVER_OUT)

# Dependency handling
-include $(RESMGR_SERVER_SOURCES:.c=.d)

