###################################################################################
#   MSGCOM DSP Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP Core0
###################################################################################
.PHONY: virtualCore0 virtualCore0Clean

# Core0 Source Files
VIRTUAL_CORE0_CFG		= test/virtual_queue/core0.cfg
VIRTUAL_CORE0_CMD       = test/virtual_queue/core0_linker.cmd
VIRTUAL_CORE0_CONFIGPKG = test/virtual_queue/core0_configPkg_$(SYSLIB_DEVICE)

ifeq ($(ARM_DSP_DOWNLOAD),1)
	VIRTUAL_CORE0_OUT       = $(VIRTUAL_CORE0_CONFIGPKG)/msgcom_virtual_core0_armdl.out
	VIRTUAL_CORE0_MAP       = $(VIRTUAL_CORE0_CONFIGPKG)/msgcom_virtual_core0_armdl.map
else
	VIRTUAL_CORE0_OUT       = $(VIRTUAL_CORE0_CONFIGPKG)/msgcom_virtual_core0.out
	VIRTUAL_CORE0_MAP       = $(VIRTUAL_CORE0_CONFIGPKG)/msgcom_virtual_core0.map
endif
VIRTUAL_CORE0_SOURCES   = $(MSGCOM_LIB_SOURCES)																\
						  test/virtual_queue/main_core0.c test/virtual_queue/stress_reader.c 				\
						  test/virtual_queue/core0_osal.c													\
						  $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   		\
						  $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c \
						  test/testautomation.c
VIRTUAL_CORE0_DEPENDS 	= $(VIRTUAL_CORE0_SOURCES:.c=.c.pp)
VIRTUAL_CORE0_OBJECTS 	= $(VIRTUAL_CORE0_SOURCES:.c=.obj)

# Increase the optimization level for the stress test:
DSP_CFLAGS += -O3

# Core0 RTSC Configuration
virtualCore0.cfg: $(VIRTUAL_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(VIRTUAL_CORE0_CONFIGPKG) $(VIRTUAL_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
virtualCore0: BUILD_CONFIGPKG=$(VIRTUAL_CORE0_CONFIGPKG)
virtualCore0: virtualCore0.cfg $(VIRTUAL_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(VIRTUAL_CORE0_MAP) -l$(VIRTUAL_CORE0_CONFIGPKG)/linker.cmd -o $(VIRTUAL_CORE0_OUT) $(VIRTUAL_CORE0_OBJECTS) -l"libc.a" $(VIRTUAL_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Virtual MSGCOM DSP Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
virtualCore0Clean:
	@echo 'Cleaning the DSP Core0 Unit Test objects'
	@rm -f $(VIRTUAL_CORE0_OBJECTS) $(VIRTUAL_CORE0_OUT) $(VIRTUAL_CORE0_DEPENDS)
	@echo 'Cleaning the DSP Core0 RTSC package'
	@$(DEL) $(VIRTUAL_CORE0_CONFIGPKG)

# Dependency handling
-include $(VIRTUAL_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# DSP Core1
###################################################################################
.PHONY: virtualCore1 virtualCore1Clean

# Core1 Files
VIRTUAL_CORE1_CFG		= test/virtual_queue/core1.cfg
VIRTUAL_CORE1_CMD       = test/virtual_queue/core1_linker.cmd
VIRTUAL_CORE1_CONFIGPKG = test/virtual_queue/core1_configPkg_$(SYSLIB_DEVICE)
ifeq ($(ARM_DSP_DOWNLOAD),1)
	VIRTUAL_CORE1_MAP       = $(VIRTUAL_CORE1_CONFIGPKG)/msgcom_virtual_core1_armdl.map
	VIRTUAL_CORE1_OUT       = $(VIRTUAL_CORE1_CONFIGPKG)/msgcom_virtual_core1_armdl.out
else
	VIRTUAL_CORE1_MAP       = $(VIRTUAL_CORE1_CONFIGPKG)/msgcom_virtual_core1.map
	VIRTUAL_CORE1_OUT       = $(VIRTUAL_CORE1_CONFIGPKG)/msgcom_virtual_core1.out
endif
VIRTUAL_CORE1_SOURCES   = $(MSGCOM_LIB_SOURCES) 															\
						  test/virtual_queue/main_core1.c test/virtual_queue/stress_writer.c 				\
						  test/virtual_queue/core1_osal.c													\
                      	  $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c 		\
                      	  $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
VIRTUAL_CORE1_DEPENDS 	= $(VIRTUAL_CORE1_SOURCES:.c=.c.pp)
VIRTUAL_CORE1_OBJECTS 	= $(VIRTUAL_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
virtualCore1.cfg: $(VIRTUAL_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(VIRTUAL_CORE1_CONFIGPKG) $(VIRTUAL_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Executable
virtualCore1: BUILD_CONFIGPKG=$(VIRTUAL_CORE1_CONFIGPKG)
virtualCore1: virtualCore1.cfg $(VIRTUAL_CORE1_OBJECTS)
	$(DSP_CC) $(DSP_LDFLAGS) -m $(VIRTUAL_CORE1_MAP) -l$(VIRTUAL_CORE1_CONFIGPKG)/linker.cmd -o $(VIRTUAL_CORE1_OUT) $(VIRTUAL_CORE1_OBJECTS) -l"libc.a" $(VIRTUAL_CORE1_CMD)
	@echo '******************************************************************************'
	@echo 'Built the Virtual MSGCOM DSP Core1 Executable '
	@echo '******************************************************************************'

# Core1 Cleanup
virtualCore1Clean:
	@echo 'Cleaning the DSP Core1 Unit Test objects'
	@rm -f $(VIRTUAL_CORE1_OBJECTS) $(VIRTUAL_CORE1_OUT) $(VIRTUAL_CORE1_DEPENDS)
	@echo 'Cleaning the DSP Core1 RTSC package'
	@$(DEL) $(VIRTUAL_CORE1_CONFIGPKG)

# Dependency handling
-include $(VIRTUAL_CORE1_SOURCES:.c=.c.pp)


