###################################################################################
#   NETFP DSP-ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP-ARM: ARM Test Application
###################################################################################
.PHONY: dspArm dspArmClean

# ARM Source files
DSP_ARM_TEST_ARMSOURCES =	test/dsp-arm/armMain.c test/dsp-arm/armOsal.c test/dsp-arm/armSocketv4.c 	\
							test/dsp-arm/armSocketv6.c test/dsp-arm/armWildCardingv4.c 					\
							test/dsp-arm/armWildCardingv6.c 

# Objects
DSP_ARM_TEST_ARMOBJECTS = $(DSP_ARM_TEST_ARMSOURCES:.c=.o)

# Dependency
DSP_ARM_TEST_DEPENDS 	= $(DSP_ARM_TEST_ARMSOURCES:.c=.d)

# Executable
DSP_ARM_TEST_ARMOUT     = test/dsp-arm/test_dsp_arm_netfp_$(SYSLIB_DEVICE).out

# Test Executable
dspArm: $(DSP_ARM_TEST_ARMOBJECTS)
	$(ARM_CC) -Wl,--start-group $(STD_LIBS) $(LOC_LIBS) $(DSP_ARM_TEST_ARMOBJECTS) -Wl,--end-group -o $(DSP_ARM_TEST_ARMOUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP DSP-ARM Executable '
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
DSP_ARM_CORE0_CFG		= 	test/dsp-arm/core0.cfg
DSP_ARM_CORE0_CMD       = 	test/dsp-arm/core0_linker.cmd
DSP_ARM_CORE0_CONFIGPKG = 	test/dsp-arm/core0_configPkg_$(SYSLIB_DEVICE)
ifeq ($(ARM_DSP_DOWNLOAD),1)
	DSP_ARM_CORE0_MAP       = 	$(DSP_ARM_CORE0_CONFIGPKG)/netfp_dsp_arm_core0_armdl.map
	DSP_ARM_CORE0_OUT       = 	$(DSP_ARM_CORE0_CONFIGPKG)/netfp_dsp_arm_core0_armdl.out
else
	DSP_ARM_CORE0_MAP       = 	$(DSP_ARM_CORE0_CONFIGPKG)/netfp_dsp_arm_core0.map
	DSP_ARM_CORE0_OUT       = 	$(DSP_ARM_CORE0_CONFIGPKG)/netfp_dsp_arm_core0.out
endif
DSP_ARM_CORE0_SOURCES   = 	$(NETFP_LIB_SOURCES) $(NETFP_DSP_SOURCES)									\
							test/dsp-arm/dspMainCore0.c test/dsp-arm/dspSetup.c                         \
							test/dsp-arm/dspSocketv4.c test/dsp-arm/dspSocketv6.c                       \
							test/dsp-arm/dspWildcardingv4.c test/dsp-arm/dspWildcardingv6.c             \
							test/dsp-arm/reestablishment.c test/dsp-arm/hook.c                          \
							test/dsp-arm/core0_osal.c test/dsp-arm/frameProtoCrc.c test/dsp-arm/net.c   \
							test/dsp-arm/fp_reestablishment.c test/dsp-arm/reestablishmentStress.c      \
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c   \
							test/test2automation.c

DSP_ARM_CORE0_DEPENDS = $(DSP_ARM_CORE0_SOURCES:.c=.c.pp)
DSP_ARM_CORE0_OBJECTS = $(DSP_ARM_CORE0_SOURCES:.c=.obj)

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
		@echo 'Built the NETFP DSP-ARM Core0 Executable '
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
DSP_ARM_CORE1_CFG		= 	test/dsp-arm/core1.cfg
DSP_ARM_CORE1_CMD       = 	test/dsp-arm/core1_linker.cmd
DSP_ARM_CORE1_CONFIGPKG = 	test/dsp-arm/core1_configPkg_$(SYSLIB_DEVICE)
ifeq ($(ARM_DSP_DOWNLOAD),1)
	DSP_ARM_CORE1_MAP       = 	$(DSP_ARM_CORE1_CONFIGPKG)/netfp_dsp_arm_core1_armdl.map
	DSP_ARM_CORE1_OUT       = 	$(DSP_ARM_CORE1_CONFIGPKG)/netfp_dsp_arm_core1_armdl.out
else
	DSP_ARM_CORE1_MAP       = 	$(DSP_ARM_CORE1_CONFIGPKG)/netfp_dsp_arm_core1.map
	DSP_ARM_CORE1_OUT       = 	$(DSP_ARM_CORE1_CONFIGPKG)/netfp_dsp_arm_core1.out
endif
DSP_ARM_CORE1_SOURCES   = 	$(NETFP_LIB_SOURCES) $(NETFP_DSP_SOURCES)									\
							test/dsp-arm/dspMainCore1.c test/dsp-arm/dspSetup.c                         \
							test/dsp-arm/dspSocketv4.c test/dsp-arm/dspSocketv6.c                       \
							test/dsp-arm/dspWildcardingv4.c test/dsp-arm/dspWildcardingv6.c             \
							test/dsp-arm/reestablishment.c test/dsp-arm/hook.c                          \
							test/dsp-arm/core1_osal.c test/dsp-arm/frameProtoCrc.c test/dsp-arm/net.c   \
							test/dsp-arm/fp_reestablishment.c test/dsp-arm/reestablishmentStress.c      \
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c   \
							test/test2automation.c
DSP_ARM_CORE1_DEPENDS = $(DSP_ARM_CORE1_SOURCES:.c=.c.pp)
DSP_ARM_CORE1_OBJECTS = $(DSP_ARM_CORE1_SOURCES:.c=.obj)

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
		@echo 'Built the NETFP DSP-ARM Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
dspArmCore1Clean:
	@echo 'Cleaning the DSP-ARM Core1 Unit Test objects'
	@rm -f $(DSP_ARM_CORE1_OBJECTS) $(DSP_ARM_CORE1_OUT) $(DSP_ARM_CORE1_DEPENDS)
	@echo 'Cleaning the DSP-ARM Core1 RTSC package'
	@$(DEL) $(DSP_ARM_CORE1_CONFIGPKG)

# Dependency handling
-include $(DSP_ARM_CORE1_SOURCES:.c=.c.pp)

