###################################################################################
#   MSGCOM Stress Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP Core0
###################################################################################
.PHONY: stressCore0 stressCore0Clean

# Core0 Source Files
STRESS_CORE0_CFG	   = test/stress/test_core0.cfg
STRESS_CORE0_CMD       = test/stress/linker_core0.cmd
STRESS_CORE0_CONFIGPKG = test/stress/core0_configPkg_$(SYSLIB_DEVICE)
ifeq ($(ARM_DSP_DOWNLOAD),1)
	STRESS_CORE0_MAP       = $(STRESS_CORE0_CONFIGPKG)/msgcom_stress_core0_armdl.map
	STRESS_CORE0_OUT       = $(STRESS_CORE0_CONFIGPKG)/msgcom_stress_core0_armdl.out
else
	STRESS_CORE0_MAP       = $(STRESS_CORE0_CONFIGPKG)/msgcom_stress_core0.map
	STRESS_CORE0_OUT       = $(STRESS_CORE0_CONFIGPKG)/msgcom_stress_core0.out
endif
STRESS_CORE0_SOURCES   = $(MSGCOM_LIB_SOURCES)															\
						 test/stress/main_core0.c test/stress/dspWriter.c test/stress/dspOsal.c			\
						 $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
						 $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c      \
						 test/testautomation.c
STRESS_CORE0_DEPENDS   = $(STRESS_CORE0_SOURCES:.c=.c.pp)
STRESS_CORE0_OBJECTS   = $(STRESS_CORE0_SOURCES:.c=.obj)

# Increase the optimization level for the stress test:
DSP_CFLAGS += -O3

# Core0 RTSC Configuration
stressCore0.cfg: $(STRESS_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(STRESS_CORE0_CONFIGPKG) $(STRESS_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
stressCore0: BUILD_CONFIGPKG=$(STRESS_CORE0_CONFIGPKG)
stressCore0: stressCore0.cfg $(STRESS_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(STRESS_CORE0_MAP) -l$(STRESS_CORE0_CONFIGPKG)/linker.cmd -o $(STRESS_CORE0_OUT) \
		$(STRESS_CORE0_OBJECTS) -l"libc.a" $(STRESS_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Stress MSGCOM DSP Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
stressCore0Clean:
	@echo 'Cleaning the DSP Core0 Unit Test objects'
	@rm -f $(STRESS_CORE0_OBJECTS) $(STRESS_CORE0_OUT) $(STRESS_CORE0_DEPENDS)
	@echo 'Cleaning the DSP Core0 RTSC package'
	@$(DEL) $(STRESS_CORE0_CONFIGPKG)

# Dependency handling
-include $(STRESS_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# DSP Core1
###################################################################################
.PHONY: stressCore1 stressCore1Clean

# Core1 Files
STRESS_CORE1_CFG	   = test/stress/test_core1.cfg
STRESS_CORE1_CMD       = test/stress/linker_core1.cmd
STRESS_CORE1_CONFIGPKG = test/stress/core1_configPkg_$(SYSLIB_DEVICE)
ifeq ($(ARM_DSP_DOWNLOAD),1)
	STRESS_CORE1_MAP       = $(STRESS_CORE1_CONFIGPKG)/msgcom_stress_core1_armdl.map
	STRESS_CORE1_OUT       = $(STRESS_CORE1_CONFIGPKG)/msgcom_stress_core1_armdl.out
else
	STRESS_CORE1_MAP       = $(STRESS_CORE1_CONFIGPKG)/msgcom_stress_core1.map
	STRESS_CORE1_OUT       = $(STRESS_CORE1_CONFIGPKG)/msgcom_stress_core1.out
endif
STRESS_CORE1_SOURCES   = $(MSGCOM_LIB_SOURCES) 														\
						 test/stress/main_core1.c test/stress/dspWriter.c test/stress/dspOsal.c		\
                      	 $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c 	\
                      	 $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
STRESS_CORE1_DEPENDS   = $(STRESS_CORE1_SOURCES:.c=.c.pp)
STRESS_CORE1_OBJECTS   = $(STRESS_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
stressCore1.cfg: $(STRESS_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(STRESS_CORE1_CONFIGPKG) $(STRESS_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Executable
stressCore1: BUILD_CONFIGPKG=$(STRESS_CORE1_CONFIGPKG)
stressCore1: stressCore1.cfg $(STRESS_CORE1_OBJECTS)
	$(DSP_CC) $(DSP_LDFLAGS) -m $(STRESS_CORE1_MAP) -l$(STRESS_CORE1_CONFIGPKG)/linker.cmd -o $(STRESS_CORE1_OUT) \
	$(STRESS_CORE1_OBJECTS) -l"libc.a" $(STRESS_CORE1_CMD)
	@echo '******************************************************************************'
	@echo 'Built the Stress MSGCOM DSP Core1 Executable '
	@echo '******************************************************************************'

# Core1 Cleanup
stressCore1Clean:
	@echo 'Cleaning the DSP Core1 Unit Test objects'
	@rm -f $(STRESS_CORE1_OBJECTS) $(STRESS_CORE1_OUT) $(STRESS_CORE1_DEPENDS)
	@echo 'Cleaning the DSP Core1 RTSC package'
	@$(DEL) $(STRESS_CORE1_CONFIGPKG)

# Dependency handling
-include $(STRESS_CORE1_SOURCES:.c=.c.pp)

###################################################################################
# Stress ARM Test
###################################################################################
.PHONY: stress stressClean

# Source Files
STRESS_ARM_TEST_SOURCES = test/stress/main.c test/stress/armOsal.c test/stress/armReader.c

# Objects
STRESS_ARM_TEST_OBJECTS = $(STRESS_ARM_TEST_SOURCES:.c=.o)

# Dependency
STRESS_ARM_TEST_DEPENDS = $(STRESS_ARM_TEST_SOURCES:.c=.d)

# Executable
STRESS_ARM_TEST_OUT     = test/stress/test_dsp_arm_stress_msgcom_$(SYSLIB_DEVICE).out

# Test Executable
stress: $(STRESS_ARM_TEST_OBJECTS)
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(STRESS_ARM_TEST_OBJECTS) -Wl,--end-group -o $(STRESS_ARM_TEST_OUT)
	@echo '******************************************************************************'
	@echo 'Built the MSGCOM ARM Executable '
	@echo '******************************************************************************'

# Test Cleanup
stressClean:
	@echo 'Cleaning the ARM Unit Test objects'
	@rm -f $(STRESS_ARM_TEST_OUT) $(STRESS_ARM_TEST_OBJECTS) $(STRESS_ARM_TEST_DEPENDS)

# Dependency handling
-include $(STRESS_ARM_TEST_SOURCES:.c=.d)

