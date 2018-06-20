# #################################################################################
#   Name Proxy makefile
#
#  NOTE:
#      (C) Copyright 2013 Texas Instruments, Inc.
# #################################################################################

include ../../runtime/common/syslib.mak

.PHONY: clean all

###################################################################################
# Standard definitions to build the module
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(ARAGODIR)/include
STD_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib 	 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(ARAGODIR)/lib
LOC_LIBS = -lrt -lname_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) -lpktlib_$(SYSLIB_DEVICE) 					\
		   -lresmgr_$(SYSLIB_DEVICE) -lrm -lhplib-no-osal_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) 			\
		   -lqmss_$(SYSLIB_DEVICE) -lpthread

# Sources
NAME_PROXY_SOURCES = src/proxy.c src/osal.c

# Objects
NAME_PROXY_OBJECTS = $(NAME_PROXY_SOURCES:.c=.o)

# Dependency
NAME_PROXY_DEPENDS = $(NAME_PROXY_SOURCES:.c=.d)

# Application name
NAME_PROXY_OUT = name_proxy_$(SYSLIB_DEVICE).out

clean:
	@echo 'Cleaning the Name Proxy Application'
	rm -f $(NAME_PROXY_OBJECTS) $(NAME_PROXY_OUT) $(NAME_PROXY_DEPENDS)

$(NAME_PROXY_OUT): $(NAME_PROXY_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(NAME_PROXY_OBJECTS) -Wl,--end-group -o $(NAME_PROXY_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NAME Proxy'
	@echo '******************************************************************************'

all: $(NAME_PROXY_OUT)

# Dependency handling
-include $(NAME_PROXY_SOURCES:.c=.d)

