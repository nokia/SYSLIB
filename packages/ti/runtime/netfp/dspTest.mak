###################################################################################
#   NETFP DSP Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP Core0
###################################################################################
.PHONY: dspCore0 dspCore0Clean

# Core0 Source Files
DSP_CORE0_CFG		= 	test/dsp/core0.cfg
DSP_CORE0_CMD       = 	test/dsp/core0_linker.cmd
DSP_CORE0_CONFIGPKG = 	test/dsp/core0_configPkg_$(SYSLIB_DEVICE)
DSP_CORE0_MAP       = 	$(DSP_CORE0_CONFIGPKG)/netfp_dsp_core0.map
DSP_CORE0_OUT       = 	$(DSP_CORE0_CONFIGPKG)/netfp_dsp_core0.out
DSP_CORE0_SOURCES   = 	$(NETFP_LIB_SOURCES) $(NETFP_DSP_SOURCES)									\
						test/dsp/main_core0.c test/dsp/core0_osal.c                                 \
						$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
						$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_CORE0_DEPENDS 	= $(DSP_CORE0_SOURCES:.c=.c.pp)
DSP_CORE0_OBJECTS 	= $(DSP_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
dspCore0.cfg: $(DSP_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_CORE0_CONFIGPKG) $(DSP_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
dspCore0: BUILD_CONFIGPKG=$(DSP_CORE0_CONFIGPKG)
dspCore0: dspCore0.cfg $(DSP_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_CORE0_MAP) -l$(DSP_CORE0_CONFIGPKG)/linker.cmd -o $(DSP_CORE0_OUT) $(DSP_CORE0_OBJECTS) -l"libc.a" $(DSP_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the NETFP DSP Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
dspCore0Clean:
	@echo 'Cleaning the DSP Core0 Unit Test objects'
	@rm -f $(DSP_CORE0_OBJECTS) $(DSP_CORE0_OUT) $(DSP_CORE0_DEPENDS)
	@echo 'Cleaning the DSP Core0 RTSC package'
	@$(DEL) $(DSP_CORE0_CONFIGPKG)

# Dependency handling
-include $(DSP_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# DSP Core1
###################################################################################
.PHONY: dspCore1 dspCore1Clean

# Core1 Files
DSP_CORE1_CFG		= 	test/dsp/core1.cfg
DSP_CORE1_CMD       = 	test/dsp/core1_linker.cmd
DSP_CORE1_CONFIGPKG = 	test/dsp/core1_configPkg_$(SYSLIB_DEVICE)
DSP_CORE1_MAP       = 	$(DSP_CORE1_CONFIGPKG)/netfp_dsp_core1.map
DSP_CORE1_OUT       = 	$(DSP_CORE1_CONFIGPKG)/netfp_dsp_core1.out
DSP_CORE1_SOURCES   = 	$(NETFP_LIB_SOURCES) $(NETFP_DSP_SOURCES)                                 	\
						test/dsp/main_core1.c test/dsp/core1_osal.c									\
                      	test/dsp/largepacket.c test/dsp/net.c test/dsp/basic.c                    	\
                      	test/dsp/encode.c test/dsp/decode.c test/dsp/pktTransform.c                 \
                      	test/dsp/reassemblyv4.c test/dsp/gtpuControlMsg.c test/dsp/test_srb.c       \
                      	test/dsp/setup.c test/dsp/stats.c test/dsp/socketv4.c                     	\
                      	test/dsp/wildcardingv4.c test/dsp/sp_reEstablishment.c                     	\
                      	test/dsp/fp_reEstablishment.c                                             	\
                      	test/dsp/sourceHandOver.c                                             		\
                      	test/dsp/sp_targetHandOver.c                                             	\
                        test/dsp/fp_targetHandOver.c                                                \
                        test/dsp/basic_ho.c                                                			\
                        test/dsp/basicReestablishment.c   											\
						test/dsp/stress_ho.c   											            \
                      	$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c 	\
                      	$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
DSP_CORE1_DEPENDS = $(DSP_CORE1_SOURCES:.c=.c.pp)
DSP_CORE1_OBJECTS = $(DSP_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
dspCore1.cfg: $(DSP_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(DSP_CORE1_CONFIGPKG) $(DSP_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Executable
dspCore1: BUILD_CONFIGPKG=$(DSP_CORE1_CONFIGPKG)
dspCore1: dspCore1.cfg $(DSP_CORE1_OBJECTS)
	$(DSP_CC) $(DSP_LDFLAGS) -m $(DSP_CORE1_MAP) -l$(DSP_CORE1_CONFIGPKG)/linker.cmd -o $(DSP_CORE1_OUT) $(DSP_CORE1_OBJECTS) -l"libc.a" $(DSP_CORE1_CMD)
	@echo '******************************************************************************'
	@echo '* Built the NETFP DSP Core1 Executable *'
	@echo '******************************************************************************'

# Core1 Cleanup
dspCore1Clean:
	@echo 'Cleaning the DSP Core1 Unit Test objects'
	@rm -f $(DSP_CORE1_OBJECTS) $(DSP_CORE1_OUT) $(DSP_CORE1_DEPENDS)
	@echo 'Cleaning the DSP Core1 RTSC package'
	@$(DEL) $(DSP_CORE1_CONFIGPKG)

# Dependency handling
-include $(DSP_CORE1_SOURCES:.c=.c.pp)

