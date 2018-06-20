###################################################################################
#   NAME DSP-ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP-ARM: ARM Test Application
###################################################################################
.PHONY: dspArm dspArmClean

# ARM Source files
DSP_ARM_TEST_ARMSOURCES =	test/dsp_arm/armMain.c test/dsp_arm/armOsal.c

# Objects
DSP_ARM_TEST_ARMOBJECTS = $(DSP_ARM_TEST_ARMSOURCES:.c=.o)

# Dependency
DSP_ARM_TEST_DEPENDS 	= $(DSP_ARM_TEST_ARMSOURCES:.c=.d)

# Executable
DSP_ARM_TEST_ARMOUT     = 	test/dsp_arm/test_dsp_arm_name_$(SYSLIB_DEVICE).out

dspArm: $(DSP_ARM_TEST_ARMOBJECTS)
	$(ARM_CC) -Wl,--start-group $(STD_LIBS) $(LOC_LIBS) $(DSP_ARM_TEST_ARMOBJECTS) -Wl,--end-group -o $(DSP_ARM_TEST_ARMOUT)
	@echo '******************************************************************************'
	@echo 'Built the NAME DSP-ARM ARM Executable '
	@echo '******************************************************************************'

dspArmClean:
	@echo 'Cleaning the DSP ARM Unit Test objects'
	@rm -f $(DSP_ARM_TEST_ARMOUT) $(DSP_ARM_TEST_ARMOBJECTS) $(DSP_ARM_TEST_DEPENDS)

# ARM Dependency handling
-include $(DSP_ARM_TEST_ARMSOURCES:.c=.d)

###################################################################################
# DSP-ARM: Core0
###################################################################################
.PHONY: dspArmCore0 dspArmCore0Clean

# Core0 Source Files
DSP_ARM_CORE0_CFG		= 	test/dsp_arm/core0.cfg
DSP_ARM_CORE0_CMD       = 	test/dsp_arm/core0_linker.cmd
DSP_ARM_CORE0_CONFIGPKG = 	test/dsp_arm/core0_configPkg_$(SYSLIB_DEVICE)
DSP_ARM_CORE0_MAP       = 	$(DSP_ARM_CORE0_CONFIGPKG)/name_dsp_arm_core0.map
DSP_ARM_CORE0_OUT       = 	$(DSP_ARM_CORE0_CONFIGPKG)/name_dsp_arm_core0.out
DSP_ARM_CORE0_SOURCES   = 	$(NAME_LIB_SOURCES) $(NAME_DSP_SOURCES)										\
							test/dsp_arm/main_core0.c test/dsp_arm/core0_osal.c 						\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_ARM_CORE0_DEPENDS 	= $(DSP_ARM_CORE0_SOURCES:.c=.c.pp)
DSP_ARM_CORE0_OBJECTS   = $(DSP_ARM_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
dspArmCore0.cfg: $(DSP_ARM_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_ARM_CORE0_CONFIGPKG) $(DSP_ARM_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
dspArmCore0: BUILD_CONFIGPKG=$(DSP_ARM_CORE0_CONFIGPKG)
dspArmCore0: dspArmCore0.cfg $(DSP_ARM_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_ARM_CORE0_MAP) -l$(DSP_ARM_CORE0_CONFIGPKG)/linker.cmd -o $(DSP_ARM_CORE0_OUT) $(DSP_ARM_CORE0_OBJECTS) -l"libc.a" $(DSP_ARM_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the NAME DSP-ARM Core0 Executable '
		@echo '******************************************************************************'


# Core0 Cleanup
dspArmCore0Clean:
	@echo 'Cleaning the DSP-ARM Core0 Unit Test objects'
	@rm -f $(DSP_ARM_CORE0_OBJECTS) $(DSP_ARM_CORE0_OUT) $(DSP_ARM_CORE0_DEPENDS)
	@echo 'Cleaning the DSP-ARM Core0 RTSC package'
	@$(DEL) $(DSP_ARM_CORE0_CONFIGPKG)

# Dependency handling
-include $(DSP_ARM_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# DSP-ARM: Core1
###################################################################################
.PHONY: dspArmCore1 dspArmCore1Clean

# Core1 Source Files
DSP_ARM_CORE1_CFG		= 	test/dsp_arm/core1.cfg
DSP_ARM_CORE1_CMD       = 	test/dsp_arm/core1_linker.cmd
DSP_ARM_CORE1_CONFIGPKG = 	test/dsp_arm/core1_configPkg_$(SYSLIB_DEVICE)
DSP_ARM_CORE1_MAP       = 	$(DSP_ARM_CORE1_CONFIGPKG)/name_dsp_arm_core1.map
DSP_ARM_CORE1_OUT       = 	$(DSP_ARM_CORE1_CONFIGPKG)/name_dsp_arm_core1.out
DSP_ARM_CORE1_SOURCES   = 	$(NAME_LIB_SOURCES) $(NAME_DSP_SOURCES)										\
							test/dsp_arm/main_core1.c test/dsp_arm/core1_osal.c 						\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_ARM_CORE1_DEPENDS 	= $(DSP_ARM_CORE1_SOURCES:.c=.c.pp)
DSP_ARM_CORE1_OBJECTS   = $(DSP_ARM_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
dspArmCore1.cfg: $(DSP_ARM_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_ARM_CORE1_CONFIGPKG) $(DSP_ARM_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Build Target
dspArmCore1: BUILD_CONFIGPKG=$(DSP_ARM_CORE1_CONFIGPKG)
dspArmCore1: dspArmCore1.cfg $(DSP_ARM_CORE1_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_ARM_CORE1_MAP) -l$(DSP_ARM_CORE1_CONFIGPKG)/linker.cmd -o $(DSP_ARM_CORE1_OUT) $(DSP_ARM_CORE1_OBJECTS) -l"libc.a" $(DSP_ARM_CORE1_CMD)
		@echo '******************************************************************************'
		@echo 'Built the NAME DSP-ARM Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
dspArmCore1Clean:
	@echo 'Cleaning the DSP-ARM Core1 Unit Test objects'
	@rm -f $(DSP_ARM_CORE1_OBJECTS) $(DSP_ARM_CORE1_OUT) $(DSP_ARM_CORE1_DEPENDS)
	@echo 'Cleaning the DSP-ARM Core1 RTSC package'
	@$(DEL) $(DSP_ARM_CORE1_CONFIGPKG)

# Dependency handling
-include $(DSP_ARM_CORE1_SOURCES:.c=.c.pp)


