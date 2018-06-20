# #################################################################################
#   SOC Init makefile
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
ifneq ($(PDK_INSTALL_PATH),)
  STD_INCL += -I$(PDK_INSTALL_PATH)
endif
STD_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(ARAGODIR)/lib
LOC_LIBS = -lrt -lrm -lresmgr_$(SYSLIB_DEVICE) -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) \
		   -lhplib-no-osal_$(SYSLIB_DEVICE) -lpthread

# Sources
SOC_INIT_SOURCES = src/soc_init.c src/$(SYSLIB_DEVICE).c src/osal.c src/listlib.c

# Objects
SOC_INIT_OBJECTS = $(SOC_INIT_SOURCES:.c=.o)

# Dependency
SOC_INIT_DEPENDS = $(SOC_INIT_SOURCES:.c=.d)

# Application name
SOC_INIT_OUT = soc_init_$(SYSLIB_DEVICE).out

clean:
	@echo 'Cleaning the SOC Initialization Application'
	rm -f $(SOC_INIT_OBJECTS) $(SOC_INIT_OUT) $(SOC_INIT_DEPENDS)

$(SOC_INIT_OUT): $(SOC_INIT_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(SOC_INIT_OBJECTS) -Wl,--end-group -o $(SOC_INIT_OUT)
	@echo '******************************************************************************'
	@echo 'Built the SOC Initialization application'
	@echo '******************************************************************************'

all: $(SOC_INIT_OUT)

# Dependency handling
-include $(SOC_INIT_SOURCES:.c=.d)

