###################################################################################
#   DAT DSP-ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP-ARM: Core0
###################################################################################
.PHONY: dspArmMemlogCore0 dspArmMemlogCore0Clean

# Core0 Source Files
DSP_ARM_MEMLOG_CORE0_CFG       = 	test/arm_dsp_memLogging/core0.cfg
DSP_ARM_MEMLOG_CORE0_CMD       = 	test/core0_linker.cmd
DSP_ARM_MEMLOG_CORE0_CONFIGPKG = 	test/arm_dsp_memLogging/core0_configPkg_$(SYSLIB_DEVICE)
DSP_ARM_MEMLOG_CORE0_MAP       = 	$(DSP_ARM_MEMLOG_CORE0_CONFIGPKG)/dat_arm_dsp_memLogging_core0.map
DSP_ARM_MEMLOG_CORE0_OUT       = 	$(DSP_ARM_MEMLOG_CORE0_CONFIGPKG)/dat_arm_dsp_memLogging_core0.out
DSP_ARM_MEMLOG_CORE0_SOURCES   = 	$(DAT_LIB_SOURCES) $(DAT_DSP_SOURCES)										\
							test/arm_dsp_memLogging/test_core0.c test/main_core0.c test/core0_osal.c		\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_ARM_MEMLOG_CORE0_DEPENDS = $(DSP_ARM_MEMLOG_CORE0_SOURCES:.c=.c.pp)
DSP_ARM_MEMLOG_CORE0_OBJECTS = $(DSP_ARM_MEMLOG_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
dspArmMemlogCore0.cfg: $(DSP_ARM_MEMLOG_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_ARM_MEMLOG_CORE0_CONFIGPKG) $(DSP_ARM_MEMLOG_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
dspArmMemlogCore0: BUILD_CONFIGPKG=$(DSP_ARM_MEMLOG_CORE0_CONFIGPKG)
dspArmMemlogCore0: dspArmMemlogCore0.cfg $(DSP_ARM_MEMLOG_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_ARM_MEMLOG_CORE0_MAP) -l$(DSP_ARM_MEMLOG_CORE0_CONFIGPKG)/linker.cmd -o $(DSP_ARM_MEMLOG_CORE0_OUT) $(DSP_ARM_MEMLOG_CORE0_OBJECTS) -l"libc.a" $(DSP_ARM_MEMLOG_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the DAT MEMLOG DSP-ARM Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
dspArmMemlogCore0Clean:
	@echo 'Cleaning the memlog DSP-ARM Core0 Unit Test objects'
	@rm -f $(DSP_ARM_MEMLOG_CORE0_OBJECTS) $(DSP_ARM_MEMLOG_CORE0_OUT) $(DSP_ARM_MEMLOG_CORE0_DEPENDS)
	@echo 'Cleaning the memlog DSP-ARM Core0 RTSC package'
	@$(DEL) $(DSP_ARM_MEMLOG_CORE0_CONFIGPKG)

# Dependency handling
-include $(DSP_ARM_MEMLOG_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# DSP-ARM: Core1
###################################################################################
.PHONY: dspArmMemlogCore1 dspArmMemlogCore1Clean

# Core1 Source Files
DSP_ARM_MEMLOG_CORE1_CFG	= 	test/arm_dsp_memLogging/core1.cfg
DSP_ARM_MEMLOG_CORE1_CMD       = 	test/core1_linker.cmd
DSP_ARM_MEMLOG_CORE1_CONFIGPKG = 	test/arm_dsp_memLogging/core1_configPkg_$(SYSLIB_DEVICE)
DSP_ARM_MEMLOG_CORE1_MAP       = 	$(DSP_ARM_MEMLOG_CORE1_CONFIGPKG)/dat_arm_dsp_memLogging_core1.map
DSP_ARM_MEMLOG_CORE1_OUT       = 	$(DSP_ARM_MEMLOG_CORE1_CONFIGPKG)/dat_arm_dsp_memLogging_core1.out
DSP_ARM_MEMLOG_CORE1_SOURCES   = 	$(DAT_LIB_SOURCES) $(DAT_DSP_SOURCES)										\
							test/arm_dsp_memLogging/test_core1.c test/main_core1.c test/core0_osal.c		\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_ARM_MEMLOG_CORE1_DEPENDS = $(DSP_ARM_MEMLOG_CORE1_SOURCES:.c=.c.pp)
DSP_ARM_MEMLOG_CORE1_OBJECTS = $(DSP_ARM_MEMLOG_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
dspArmMemlogCore1.cfg: $(DSP_ARM_MEMLOG_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_ARM_MEMLOG_CORE1_CONFIGPKG) $(DSP_ARM_MEMLOG_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Build Target
dspArmMemlogCore1: BUILD_CONFIGPKG=$(DSP_ARM_MEMLOG_CORE1_CONFIGPKG)
dspArmMemlogCore1: dspArmMemlogCore1.cfg $(DSP_ARM_MEMLOG_CORE1_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_ARM_MEMLOG_CORE1_MAP) -l$(DSP_ARM_MEMLOG_CORE1_CONFIGPKG)/linker.cmd -o $(DSP_ARM_MEMLOG_CORE1_OUT) $(DSP_ARM_MEMLOG_CORE1_OBJECTS) -l"libc.a" $(DSP_ARM_MEMLOG_CORE1_CMD)
		@echo '******************************************************************************'
		@echo 'Built the DAT MEMLOG DSP-ARM Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
dspArmMemlogCore1Clean:
	@echo 'Cleaning the memlog DSP-ARM Core1 Unit Test objects'
	@rm -f $(DSP_ARM_MEMLOG_CORE1_OBJECTS) $(DSP_ARM_MEMLOG_CORE1_OUT) $(DSP_ARM_MEMLOG_CORE1_DEPENDS)
	@echo 'Cleaning the memlog DSP-ARM Core1 RTSC package'
	@$(DEL) $(DSP_ARM_MEMLOG_CORE1_CONFIGPKG)

# Dependency handling
-include $(DSP_ARM_MEMLOG_CORE1_SOURCES:.c=.c.pp)

