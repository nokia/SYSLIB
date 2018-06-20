# #################################################################################
#   Release10-Deployment2 Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

.PHONY: rel10D2Clean rel10D2

###################################################################################
# Release10 Deployment2: DSP Core0
###################################################################################

.PHONY: rel10D2Core0 rel10D2Core0Clean

# Core0 Source Files
REL10_D2_DSP_CORE0_CFG       = rel10-d2/l2/core0.cfg
REL10_D2_DSP_CORE0_CMD       = rel10-d2/l2/linker_core0.cmd
REL10_D2_DSP_CORE0_CONFIGPKG = rel10-d2/l2/core0_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE0_MAP       = $(REL10_D2_DSP_CORE0_CONFIGPKG)/rel10D2_core0.map
REL10_D2_DSP_CORE0_OUT       = $(REL10_D2_DSP_CORE0_CONFIGPKG)/rel10D2_core0.out
REL10_D2_DSP_CORE0_SOURCES   = common/root_dsp_client.c rel10-d2/l2/l2_dp.c	rel10-d2/l2/osal.c						\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE0_DEPENDS = $(REL10_D2_DSP_CORE0_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE0_OBJECTS = $(REL10_D2_DSP_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
rel10D2Core0.cfg: $(REL10_D2_DSP_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE0_CONFIGPKG) $(REL10_D2_DSP_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
rel10D2Core0: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE0_CONFIGPKG)
rel10D2Core0: rel10D2Core0.cfg $(REL10_D2_DSP_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE0_MAP) -l$(REL10_D2_DSP_CORE0_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE0_OUT) $(REL10_D2_DSP_CORE0_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
rel10D2Core0Clean:
	@echo 'Cleaning the Release10 Deployment2 Core0 objects'
	@rm -f $(REL10_D2_DSP_CORE0_OBJECTS) $(REL10_D2_DSP_CORE0_OUT) $(REL10_D2_DSP_CORE0_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core0 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE0_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core1
###################################################################################

.PHONY: rel10-d2Core1 rel10-d2Core1Clean

# Core1 Source Files
REL10_D2_DSP_CORE1_CFG	     = rel10-d2/sim_phy/core1.cfg
REL10_D2_DSP_CORE1_CMD       = rel10-d2/sim_phy/linker_core1.cmd
REL10_D2_DSP_CORE1_CONFIGPKG = rel10-d2/sim_phy/core1_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE1_MAP       = $(REL10_D2_DSP_CORE1_CONFIGPKG)/rel10D2_core1.map
REL10_D2_DSP_CORE1_OUT       = $(REL10_D2_DSP_CORE1_CONFIGPKG)/rel10D2_core1.out
REL10_D2_DSP_CORE1_SOURCES   = common/root_dsp_client.c rel10-d2/sim_phy/sim_phy_slave.c						\
                               rel10-d2/sim_phy/osal.c rel10-d2/sim_phy/logging.c								\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   		\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE1_DEPENDS = $(REL10_D2_DSP_CORE1_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE1_OBJECTS = $(REL10_D2_DSP_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
rel10D2Core1.cfg: $(REL10_D2_DSP_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE1_CONFIGPKG) $(REL10_D2_DSP_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Build Target
rel10D2Core1: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE1_CONFIGPKG)
rel10D2Core1: rel10D2Core1.cfg $(REL10_D2_DSP_CORE1_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE1_MAP) -l$(REL10_D2_DSP_CORE1_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE1_OUT) $(REL10_D2_DSP_CORE1_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE1_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
rel10D2Core1Clean:
	@echo 'Cleaning the Release10 Deployment2 Core1 objects'
	@rm -f $(REL10_D2_DSP_CORE1_OBJECTS) $(REL10_D2_DSP_CORE1_OUT) $(REL10_D2_DSP_CORE1_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core1 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE1_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE1_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core2
###################################################################################

.PHONY: rel10-d2Core2 rel10-d2Core2Clean

# Core2 Source Files
REL10_D2_DSP_CORE2_CFG	     = rel10-d2/sim_phy/core2.cfg
REL10_D2_DSP_CORE2_CMD       = rel10-d2/sim_phy/linker_core2.cmd
REL10_D2_DSP_CORE2_CONFIGPKG = rel10-d2/sim_phy/core2_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE2_MAP       = $(REL10_D2_DSP_CORE2_CONFIGPKG)/rel10D2_core2.map
REL10_D2_DSP_CORE2_OUT       = $(REL10_D2_DSP_CORE2_CONFIGPKG)/rel10D2_core2.out
REL10_D2_DSP_CORE2_SOURCES   = common/root_dsp_client.c rel10-d2/sim_phy/sim_phy_master.c rel10-d2/sim_phy/fapi.c	\
                               rel10-d2/sim_phy/osal.c rel10-d2/sim_phy/logging.c									\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   			\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE2_DEPENDS = $(REL10_D2_DSP_CORE2_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE2_OBJECTS = $(REL10_D2_DSP_CORE2_SOURCES:.c=.obj)

# Core2 RTSC Configuration
rel10D2Core2.cfg: $(REL10_D2_DSP_CORE2_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE2_CONFIGPKG) $(REL10_D2_DSP_CORE2_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core2 Build Target
rel10D2Core2: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE2_CONFIGPKG)
rel10D2Core2: rel10D2Core2.cfg $(REL10_D2_DSP_CORE2_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE2_MAP) -l$(REL10_D2_DSP_CORE2_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE2_OUT) $(REL10_D2_DSP_CORE2_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE2_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core2 Executable '
		@echo '******************************************************************************'

# Core2 Cleanup
rel10D2Core2Clean:
	@echo 'Cleaning the Release10 Deployment2 Core2 objects'
	@rm -f $(REL10_D2_DSP_CORE2_OBJECTS) $(REL10_D2_DSP_CORE2_OUT) $(REL10_D2_DSP_CORE2_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core2 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE2_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE2_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core3
###################################################################################

.PHONY: rel10-d2Core3 rel10-d2Core3Clean

# Core3 Source Files
REL10_D2_DSP_CORE3_CFG	     = rel10-d2/l2/core3.cfg
REL10_D2_DSP_CORE3_CMD       = rel10-d2/l2/linker_core3.cmd
REL10_D2_DSP_CORE3_CONFIGPKG = rel10-d2/l2/core3_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE3_MAP       = $(REL10_D2_DSP_CORE3_CONFIGPKG)/rel10D2_core3.map
REL10_D2_DSP_CORE3_OUT       = $(REL10_D2_DSP_CORE3_CONFIGPKG)/rel10D2_core3.out
REL10_D2_DSP_CORE3_SOURCES   = common/root_dsp_client.c rel10-d2/l2/l2_sched.c	rel10-d2/l2/osal.c 		\
                               rel10-d2/l2/fapi.c rel10-d2/l2/logging.c						\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE3_DEPENDS = $(REL10_D2_DSP_CORE3_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE3_OBJECTS = $(REL10_D2_DSP_CORE3_SOURCES:.c=.obj)

# Core3 RTSC Configuration
rel10D2Core3.cfg: $(REL10_D2_DSP_CORE3_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE3_CONFIGPKG) $(REL10_D2_DSP_CORE3_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core3 Build Target
rel10D2Core3: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE3_CONFIGPKG)
rel10D2Core3: rel10D2Core3.cfg $(REL10_D2_DSP_CORE3_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE3_MAP) -l$(REL10_D2_DSP_CORE3_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE3_OUT) $(REL10_D2_DSP_CORE3_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE3_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core3 Executable '
		@echo '******************************************************************************'

# Core3 Cleanup
rel10D2Core3Clean:
	@echo 'Cleaning the Release10 Deployment2 Core3 objects'
	@rm -f $(REL10_D2_DSP_CORE3_OBJECTS) $(REL10_D2_DSP_CORE3_OUT) $(REL10_D2_DSP_CORE3_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core3 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE3_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE3_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core4
###################################################################################

.PHONY: rel10-d2Core4 rel10-d2Core4Clean

# Core4 Source Files
REL10_D2_DSP_CORE4_CFG       = rel10-d2/l2/core4.cfg
REL10_D2_DSP_CORE4_CMD       = rel10-d2/l2/linker_core4.cmd
REL10_D2_DSP_CORE4_CONFIGPKG = rel10-d2/l2/core4_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE4_MAP       = $(REL10_D2_DSP_CORE4_CONFIGPKG)/rel10D2_core4.map
REL10_D2_DSP_CORE4_OUT       = $(REL10_D2_DSP_CORE4_CONFIGPKG)/rel10D2_core4.out
REL10_D2_DSP_CORE4_SOURCES   = common/root_dsp_client.c rel10-d2/l2/l2_dp.c	rel10-d2/l2/osal.c						\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE4_DEPENDS = $(REL10_D2_DSP_CORE4_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE4_OBJECTS = $(REL10_D2_DSP_CORE4_SOURCES:.c=.obj)

# Core4 RTSC Configuration
rel10D2Core4.cfg: $(REL10_D2_DSP_CORE4_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE4_CONFIGPKG) $(REL10_D2_DSP_CORE4_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core4 Build Target
rel10D2Core4: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE4_CONFIGPKG)
rel10D2Core4: rel10D2Core4.cfg $(REL10_D2_DSP_CORE4_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE4_MAP) -l$(REL10_D2_DSP_CORE4_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE4_OUT) $(REL10_D2_DSP_CORE4_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE4_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core4 Executable '
		@echo '******************************************************************************'

# Core4 Cleanup
rel10D2Core4Clean:
	@echo 'Cleaning the Release10 Deployment2 Core4 objects'
	@rm -f $(REL10_D2_DSP_CORE4_OBJECTS) $(REL10_D2_DSP_CORE4_OUT) $(REL10_D2_DSP_CORE4_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core4 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE4_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE4_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core5
###################################################################################

.PHONY: rel10-d2Core5 rel10-d2Core5Clean

# Core5 Source Files
REL10_D2_DSP_CORE5_CFG	     = rel10-d2/sim_phy/core5.cfg
REL10_D2_DSP_CORE5_CMD       = rel10-d2/sim_phy/linker_core5.cmd
REL10_D2_DSP_CORE5_CONFIGPKG = rel10-d2/sim_phy/core5_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE5_MAP       = $(REL10_D2_DSP_CORE5_CONFIGPKG)/rel10D2_core5.map
REL10_D2_DSP_CORE5_OUT       = $(REL10_D2_DSP_CORE5_CONFIGPKG)/rel10D2_core5.out
REL10_D2_DSP_CORE5_SOURCES   = common/root_dsp_client.c rel10-d2/sim_phy/sim_phy_slave.c						\
                               rel10-d2/sim_phy/osal.c rel10-d2/sim_phy/logging.c									\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE5_DEPENDS = $(REL10_D2_DSP_CORE5_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE5_OBJECTS = $(REL10_D2_DSP_CORE5_SOURCES:.c=.obj)

# Core5 RTSC Configuration
rel10D2Core5.cfg: $(REL10_D2_DSP_CORE5_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE5_CONFIGPKG) $(REL10_D2_DSP_CORE5_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core5 Build Target
rel10D2Core5: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE5_CONFIGPKG)
rel10D2Core5: rel10D2Core5.cfg $(REL10_D2_DSP_CORE5_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE5_MAP) -l$(REL10_D2_DSP_CORE5_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE5_OUT) $(REL10_D2_DSP_CORE5_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE5_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core5 Executable '
		@echo '******************************************************************************'

# Core5 Cleanup
rel10D2Core5Clean:
	@echo 'Cleaning the Release10 Deployment2 Core5 objects'
	@rm -f $(REL10_D2_DSP_CORE5_OBJECTS) $(REL10_D2_DSP_CORE5_OUT) $(REL10_D2_DSP_CORE5_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core5 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE5_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE5_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core6
###################################################################################

.PHONY: rel10-d2Core6 rel10-d2Core6Clean

# Core6 Source Files
REL10_D2_DSP_CORE6_CFG	     = rel10-d2/sim_phy/core6.cfg
REL10_D2_DSP_CORE6_CMD       = rel10-d2/sim_phy/linker_core6.cmd
REL10_D2_DSP_CORE6_CONFIGPKG = rel10-d2/sim_phy/core6_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE6_MAP       = $(REL10_D2_DSP_CORE6_CONFIGPKG)/rel10D2_core6.map
REL10_D2_DSP_CORE6_OUT       = $(REL10_D2_DSP_CORE6_CONFIGPKG)/rel10D2_core6.out
REL10_D2_DSP_CORE6_SOURCES   = common/root_dsp_client.c rel10-d2/sim_phy/sim_phy_master.c rel10-d2/sim_phy/fapi.c	\
                               rel10-d2/sim_phy/osal.c rel10-d2/sim_phy/logging.c									\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE6_DEPENDS = $(REL10_D2_DSP_CORE6_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE6_OBJECTS = $(REL10_D2_DSP_CORE6_SOURCES:.c=.obj)

# Core6 RTSC Configuration
rel10D2Core6.cfg: $(REL10_D2_DSP_CORE6_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE6_CONFIGPKG) $(REL10_D2_DSP_CORE6_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core6 Build Target
rel10D2Core6: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE6_CONFIGPKG)
rel10D2Core6: rel10D2Core6.cfg $(REL10_D2_DSP_CORE6_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE6_MAP) -l$(REL10_D2_DSP_CORE6_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE6_OUT) $(REL10_D2_DSP_CORE6_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE6_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core6 Executable '
		@echo '******************************************************************************'

# Core6 Cleanup
rel10D2Core6Clean:
	@echo 'Cleaning the Release10 Deployment2 Core6 objects'
	@rm -f $(REL10_D2_DSP_CORE6_OBJECTS) $(REL10_D2_DSP_CORE6_OUT) $(REL10_D2_DSP_CORE6_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core6 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE6_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE6_SOURCES:.c=.c.pp)

###################################################################################
# Release10 Deployment2: DSP Core7
###################################################################################

.PHONY: rel10-d2Core7 rel10-d2Core7Clean

# Core7 Source Files
REL10_D2_DSP_CORE7_CFG       = rel10-d2/l2/core7.cfg
REL10_D2_DSP_CORE7_CMD       = rel10-d2/l2/linker_core7.cmd
REL10_D2_DSP_CORE7_CONFIGPKG = rel10-d2/l2/core7_configPkg_$(SYSLIB_DEVICE)
REL10_D2_DSP_CORE7_MAP       = $(REL10_D2_DSP_CORE7_CONFIGPKG)/rel10D2_core7.map
REL10_D2_DSP_CORE7_OUT       = $(REL10_D2_DSP_CORE7_CONFIGPKG)/rel10D2_core7.out
REL10_D2_DSP_CORE7_SOURCES   = common/root_dsp_client.c rel10-d2/l2/l2_sched.c	rel10-d2/l2/osal.c 					\
                               rel10-d2/l2/fapi.c rel10-d2/l2/logging.c												\
                               $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                               $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_D2_DSP_CORE7_DEPENDS = $(REL10_D2_DSP_CORE7_SOURCES:.c=.c.pp)
REL10_D2_DSP_CORE7_OBJECTS = $(REL10_D2_DSP_CORE7_SOURCES:.c=.obj)

# Core7 RTSC Configuration
rel10D2Core7.cfg: $(REL10_D2_DSP_CORE7_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_D2_DSP_CORE7_CONFIGPKG) $(REL10_D2_DSP_CORE7_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core7 Build Target
rel10D2Core7: BUILD_CONFIGPKG=$(REL10_D2_DSP_CORE7_CONFIGPKG)
rel10D2Core7: rel10D2Core7.cfg $(REL10_D2_DSP_CORE7_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_D2_DSP_CORE7_MAP) -l$(REL10_D2_DSP_CORE7_CONFIGPKG)/linker.cmd -o $(REL10_D2_DSP_CORE7_OUT) $(REL10_D2_DSP_CORE7_OBJECTS) -l"libc.a" $(REL10_D2_DSP_CORE7_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release10 Deployment2 Core7 Executable '
		@echo '******************************************************************************'

# Core7 Cleanup
rel10D2Core7Clean:
	@echo 'Cleaning the Release10 Deployment2 Core7 objects'
	@rm -f $(REL10_D2_DSP_CORE7_OBJECTS) $(REL10_D2_DSP_CORE7_OUT) $(REL10_D2_DSP_CORE7_DEPENDS)
	@echo 'Cleaning the Release10 Deployment2 Core7 RTSC package'
	@$(DEL) $(REL10_D2_DSP_CORE7_CONFIGPKG)

# Dependency handling
-include $(REL10_D2_DSP_CORE7_SOURCES:.c=.c.pp)

###################################################################################
# The REL10 Deployment2 Demo is supported only for K2H.
###################################################################################
ifeq ($(SYSLIB_DEVICE), k2h)
rel10D2: rel10D2Core0 rel10D2Core1 rel10D2Core2 rel10D2Core3 rel10D2Core4 rel10D2Core5 \
	     rel10D2Core6 rel10D2Core7
rel10D2Clean: rel10D2Core0Clean rel10D2Core1Clean rel10D2Core2Clean rel10D2Core3Clean \
		      rel10D2Core4Clean rel10D2Core5Clean rel10D2Core6Clean rel10D2Core7Clean
else
rel10D2:
	@echo 'Error: Release 10 Deployment 2 Demo is NOT supported for the K2L/K2K'
rel10D2Clean:
		@echo 'Error: Release 10 Deployment 2 Demo is NOT supported for the K2L/K2K'
endif

