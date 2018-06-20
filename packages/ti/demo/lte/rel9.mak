# #################################################################################
#   REL9 Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

.PHONY: rel9Clean rel9

###################################################################################
# Release9: DSP Core0
###################################################################################

.PHONY: rel9Core0 rel9Core0Clean

# Core0 Source Files
REL9_DSP_CORE0_CFG	 = rel9/l2/core0.cfg
REL9_DSP_CORE0_CMD       = rel9/l2/linker_core0.cmd
REL9_DSP_CORE0_CONFIGPKG = rel9/l2/core0_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE0_MAP       = $(REL9_DSP_CORE0_CONFIGPKG)/rel9_core0.map
REL9_DSP_CORE0_OUT       = $(REL9_DSP_CORE0_CONFIGPKG)/rel9_core0.out
REL9_DSP_CORE0_SOURCES   = common/root_dsp_client.c rel9/l2/l2_dp.c	rel9/l2/osal.c						\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE0_DEPENDS = $(REL9_DSP_CORE0_SOURCES:.c=.c.pp)
REL9_DSP_CORE0_OBJECTS = $(REL9_DSP_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
rel9Core0.cfg: $(REL9_DSP_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE0_CONFIGPKG) $(REL9_DSP_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
rel9Core0: BUILD_CONFIGPKG=$(REL9_DSP_CORE0_CONFIGPKG)
rel9Core0: rel9Core0.cfg $(REL9_DSP_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE0_MAP) -l$(REL9_DSP_CORE0_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE0_OUT) $(REL9_DSP_CORE0_OBJECTS) -l"libc.a" $(REL9_DSP_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
rel9Core0Clean:
	@echo 'Cleaning the Release9 Core0 objects'
	@rm -f $(REL9_DSP_CORE0_OBJECTS) $(REL9_DSP_CORE0_OUT) $(REL9_DSP_CORE0_DEPENDS)
	@echo 'Cleaning the Release9 Core0 RTSC package'
	@$(DEL) $(REL9_DSP_CORE0_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core1
###################################################################################

.PHONY: rel9Core1 rel9Core1Clean

# Core1 Source Files
REL9_DSP_CORE1_CFG	 = rel9/sim_phy/core1.cfg
REL9_DSP_CORE1_CMD       = rel9/sim_phy/linker_core1.cmd
REL9_DSP_CORE1_CONFIGPKG = rel9/sim_phy/core1_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE1_MAP       = $(REL9_DSP_CORE1_CONFIGPKG)/rel9_core1.map
REL9_DSP_CORE1_OUT       = $(REL9_DSP_CORE1_CONFIGPKG)/rel9_core1.out
REL9_DSP_CORE1_SOURCES   = common/root_dsp_client.c rel9/sim_phy/sim_phy_slave.c						\
                           rel9/sim_phy/osal.c rel9/sim_phy/logging.c									\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE1_DEPENDS = $(REL9_DSP_CORE1_SOURCES:.c=.c.pp)
REL9_DSP_CORE1_OBJECTS = $(REL9_DSP_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
rel9Core1.cfg: $(REL9_DSP_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE1_CONFIGPKG) $(REL9_DSP_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Build Target
rel9Core1: BUILD_CONFIGPKG=$(REL9_DSP_CORE1_CONFIGPKG)
rel9Core1: rel9Core1.cfg $(REL9_DSP_CORE1_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE1_MAP) -l$(REL9_DSP_CORE1_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE1_OUT) $(REL9_DSP_CORE1_OBJECTS) -l"libc.a" $(REL9_DSP_CORE1_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
rel9Core1Clean:
	@echo 'Cleaning the Release9 Core1 objects'
	@rm -f $(REL9_DSP_CORE1_OBJECTS) $(REL9_DSP_CORE1_OUT) $(REL9_DSP_CORE1_DEPENDS)
	@echo 'Cleaning the Release9 Core1 RTSC package'
	@$(DEL) $(REL9_DSP_CORE1_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE1_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core2
###################################################################################

.PHONY: rel9Core2 rel9Core2Clean

# Core2 Source Files
REL9_DSP_CORE2_CFG	 = rel9/sim_phy/core2.cfg
REL9_DSP_CORE2_CMD       = rel9/sim_phy/linker_core2.cmd
REL9_DSP_CORE2_CONFIGPKG = rel9/sim_phy/core2_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE2_MAP       = $(REL9_DSP_CORE2_CONFIGPKG)/rel9_core2.map
REL9_DSP_CORE2_OUT       = $(REL9_DSP_CORE2_CONFIGPKG)/rel9_core2.out
REL9_DSP_CORE2_SOURCES   = common/root_dsp_client.c rel9/sim_phy/sim_phy_master.c rel9/sim_phy/fapi.c	\
                           rel9/sim_phy/osal.c rel9/sim_phy/logging.c									\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE2_DEPENDS = $(REL9_DSP_CORE2_SOURCES:.c=.c.pp)
REL9_DSP_CORE2_OBJECTS = $(REL9_DSP_CORE2_SOURCES:.c=.obj)

# Core2 RTSC Configuration
rel9Core2.cfg: $(REL9_DSP_CORE2_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE2_CONFIGPKG) $(REL9_DSP_CORE2_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core2 Build Target
rel9Core2: BUILD_CONFIGPKG=$(REL9_DSP_CORE2_CONFIGPKG)
rel9Core2: rel9Core2.cfg $(REL9_DSP_CORE2_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE2_MAP) -l$(REL9_DSP_CORE2_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE2_OUT) $(REL9_DSP_CORE2_OBJECTS) -l"libc.a" $(REL9_DSP_CORE2_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core2 Executable '
		@echo '******************************************************************************'

# Core2 Cleanup
rel9Core2Clean:
	@echo 'Cleaning the Release9 Core2 objects'
	@rm -f $(REL9_DSP_CORE2_OBJECTS) $(REL9_DSP_CORE2_OUT) $(REL9_DSP_CORE2_DEPENDS)
	@echo 'Cleaning the Release9 Core2 RTSC package'
	@$(DEL) $(REL9_DSP_CORE2_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE2_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core3
###################################################################################

.PHONY: rel9Core3 rel9Core3Clean

# Core3 Source Files
REL9_DSP_CORE3_CFG	 = rel9/l2/core3.cfg
REL9_DSP_CORE3_CMD       = rel9/l2/linker_core3.cmd
REL9_DSP_CORE3_CONFIGPKG = rel9/l2/core3_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE3_MAP       = $(REL9_DSP_CORE3_CONFIGPKG)/rel9_core3.map
REL9_DSP_CORE3_OUT       = $(REL9_DSP_CORE3_CONFIGPKG)/rel9_core3.out
REL9_DSP_CORE3_SOURCES   = common/root_dsp_client.c rel9/l2/l2_sched.c	rel9/l2/osal.c 					\
                           rel9/l2/fapi.c rel9/l2/logging.c												\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE3_DEPENDS = $(REL9_DSP_CORE3_SOURCES:.c=.c.pp)
REL9_DSP_CORE3_OBJECTS = $(REL9_DSP_CORE3_SOURCES:.c=.obj)

# Core3 RTSC Configuration
rel9Core3.cfg: $(REL9_DSP_CORE3_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE3_CONFIGPKG) $(REL9_DSP_CORE3_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core3 Build Target
rel9Core3: BUILD_CONFIGPKG=$(REL9_DSP_CORE3_CONFIGPKG)
rel9Core3: rel9Core3.cfg $(REL9_DSP_CORE3_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE3_MAP) -l$(REL9_DSP_CORE3_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE3_OUT) $(REL9_DSP_CORE3_OBJECTS) -l"libc.a" $(REL9_DSP_CORE3_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core3 Executable '
		@echo '******************************************************************************'

# Core3 Cleanup
rel9Core3Clean:
	@echo 'Cleaning the Release9 Core3 objects'
	@rm -f $(REL9_DSP_CORE3_OBJECTS) $(REL9_DSP_CORE3_OUT) $(REL9_DSP_CORE3_DEPENDS)
	@echo 'Cleaning the Release9 Core3 RTSC package'
	@$(DEL) $(REL9_DSP_CORE3_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE3_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core4
###################################################################################

.PHONY: rel9Core4 rel9Core4Clean

# Core4 Source Files
REL9_DSP_CORE4_CFG	 = rel9/l2/core4.cfg
REL9_DSP_CORE4_CMD       = rel9/l2/linker_core4.cmd
REL9_DSP_CORE4_CONFIGPKG = rel9/l2/core4_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE4_MAP       = $(REL9_DSP_CORE4_CONFIGPKG)/rel9_core4.map
REL9_DSP_CORE4_OUT       = $(REL9_DSP_CORE4_CONFIGPKG)/rel9_core4.out
REL9_DSP_CORE4_SOURCES   = common/root_dsp_client.c rel9/l2/l2_dp.c	rel9/l2/osal.c						\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE4_DEPENDS = $(REL9_DSP_CORE4_SOURCES:.c=.c.pp)
REL9_DSP_CORE4_OBJECTS = $(REL9_DSP_CORE4_SOURCES:.c=.obj)

# Core4 RTSC Configuration
rel9Core4.cfg: $(REL9_DSP_CORE4_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE4_CONFIGPKG) $(REL9_DSP_CORE4_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core4 Build Target
rel9Core4: BUILD_CONFIGPKG=$(REL9_DSP_CORE4_CONFIGPKG)
rel9Core4: rel9Core4.cfg $(REL9_DSP_CORE4_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE4_MAP) -l$(REL9_DSP_CORE4_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE4_OUT) $(REL9_DSP_CORE4_OBJECTS) -l"libc.a" $(REL9_DSP_CORE4_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core4 Executable '
		@echo '******************************************************************************'

# Core4 Cleanup
rel9Core4Clean:
	@echo 'Cleaning the Release9 Core4 objects'
	@rm -f $(REL9_DSP_CORE4_OBJECTS) $(REL9_DSP_CORE4_OUT) $(REL9_DSP_CORE4_DEPENDS)
	@echo 'Cleaning the Release9 Core4 RTSC package'
	@$(DEL) $(REL9_DSP_CORE4_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE4_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core5
###################################################################################

.PHONY: rel9Core5 rel9Core5Clean

# Core5 Source Files
REL9_DSP_CORE5_CFG	 = rel9/sim_phy/core5.cfg
REL9_DSP_CORE5_CMD       = rel9/sim_phy/linker_core5.cmd
REL9_DSP_CORE5_CONFIGPKG = rel9/sim_phy/core5_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE5_MAP       = $(REL9_DSP_CORE5_CONFIGPKG)/rel9_core5.map
REL9_DSP_CORE5_OUT       = $(REL9_DSP_CORE5_CONFIGPKG)/rel9_core5.out
REL9_DSP_CORE5_SOURCES   = common/root_dsp_client.c rel9/sim_phy/sim_phy_slave.c						\
                           rel9/sim_phy/osal.c rel9/sim_phy/logging.c									\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE5_DEPENDS = $(REL9_DSP_CORE5_SOURCES:.c=.c.pp)
REL9_DSP_CORE5_OBJECTS = $(REL9_DSP_CORE5_SOURCES:.c=.obj)

# Core5 RTSC Configuration
rel9Core5.cfg: $(REL9_DSP_CORE5_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE5_CONFIGPKG) $(REL9_DSP_CORE5_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core5 Build Target
rel9Core5: BUILD_CONFIGPKG=$(REL9_DSP_CORE5_CONFIGPKG)
rel9Core5: rel9Core5.cfg $(REL9_DSP_CORE5_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE5_MAP) -l$(REL9_DSP_CORE5_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE5_OUT) $(REL9_DSP_CORE5_OBJECTS) -l"libc.a" $(REL9_DSP_CORE5_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core5 Executable '
		@echo '******************************************************************************'

# Core5 Cleanup
rel9Core5Clean:
	@echo 'Cleaning the Release9 Core5 objects'
	@rm -f $(REL9_DSP_CORE5_OBJECTS) $(REL9_DSP_CORE5_OUT) $(REL9_DSP_CORE5_DEPENDS)
	@echo 'Cleaning the Release9 Core5 RTSC package'
	@$(DEL) $(REL9_DSP_CORE5_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE5_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core6
###################################################################################

.PHONY: rel9Core6 rel9Core6Clean

# Core6 Source Files
REL9_DSP_CORE6_CFG	 = rel9/sim_phy/core6.cfg
REL9_DSP_CORE6_CMD       = rel9/sim_phy/linker_core6.cmd
REL9_DSP_CORE6_CONFIGPKG = rel9/sim_phy/core6_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE6_MAP       = $(REL9_DSP_CORE6_CONFIGPKG)/rel9_core6.map
REL9_DSP_CORE6_OUT       = $(REL9_DSP_CORE6_CONFIGPKG)/rel9_core6.out
REL9_DSP_CORE6_SOURCES   = common/root_dsp_client.c rel9/sim_phy/sim_phy_master.c rel9/sim_phy/fapi.c	\
                           rel9/sim_phy/osal.c rel9/sim_phy/logging.c									\
                           $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
                           $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE6_DEPENDS = $(REL9_DSP_CORE6_SOURCES:.c=.c.pp)
REL9_DSP_CORE6_OBJECTS = $(REL9_DSP_CORE6_SOURCES:.c=.obj)

# Core6 RTSC Configuration
rel9Core6.cfg: $(REL9_DSP_CORE6_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE6_CONFIGPKG) $(REL9_DSP_CORE6_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core6 Build Target
rel9Core6: BUILD_CONFIGPKG=$(REL9_DSP_CORE6_CONFIGPKG)
rel9Core6: rel9Core6.cfg $(REL9_DSP_CORE6_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE6_MAP) -l$(REL9_DSP_CORE6_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE6_OUT) $(REL9_DSP_CORE6_OBJECTS) -l"libc.a" $(REL9_DSP_CORE6_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core6 Executable '
		@echo '******************************************************************************'

# Core6 Cleanup
rel9Core6Clean:
	@echo 'Cleaning the Release9 Core6 objects'
	@rm -f $(REL9_DSP_CORE6_OBJECTS) $(REL9_DSP_CORE6_OUT) $(REL9_DSP_CORE6_DEPENDS)
	@echo 'Cleaning the Release9 Core6 RTSC package'
	@$(DEL) $(REL9_DSP_CORE6_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE6_SOURCES:.c=.c.pp)

###################################################################################
# Release9: DSP Core7
###################################################################################

.PHONY: rel9Core7 rel9Core7Clean

# Core7 Source Files
REL9_DSP_CORE7_CFG	 = rel9/l2/core7.cfg
REL9_DSP_CORE7_CMD       = rel9/l2/linker_core7.cmd
REL9_DSP_CORE7_CONFIGPKG = rel9/l2/core7_configPkg_$(SYSLIB_DEVICE)
REL9_DSP_CORE7_MAP       = $(REL9_DSP_CORE7_CONFIGPKG)/rel9_core7.map
REL9_DSP_CORE7_OUT       = $(REL9_DSP_CORE7_CONFIGPKG)/rel9_core7.out
REL9_DSP_CORE7_SOURCES   = common/root_dsp_client.c rel9/l2/l2_sched.c	rel9/l2/osal.c 					\
						   rel9/l2/fapi.c rel9/l2/logging.c												\
						   $(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   	\
						   $(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL9_DSP_CORE7_DEPENDS = $(REL9_DSP_CORE7_SOURCES:.c=.c.pp)
REL9_DSP_CORE7_OBJECTS = $(REL9_DSP_CORE7_SOURCES:.c=.obj)

# Core7 RTSC Configuration
rel9Core7.cfg: $(REL9_DSP_CORE7_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL9_DSP_CORE7_CONFIGPKG) $(REL9_DSP_CORE7_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core7 Build Target
rel9Core7: BUILD_CONFIGPKG=$(REL9_DSP_CORE7_CONFIGPKG)
rel9Core7: rel9Core7.cfg $(REL9_DSP_CORE7_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL9_DSP_CORE7_MAP) -l$(REL9_DSP_CORE7_CONFIGPKG)/linker.cmd -o $(REL9_DSP_CORE7_OUT) $(REL9_DSP_CORE7_OBJECTS) -l"libc.a" $(REL9_DSP_CORE7_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Release9 Core7 Executable '
		@echo '******************************************************************************'

# Core7 Cleanup
rel9Core7Clean:
	@echo 'Cleaning the Release9 Core7 objects'
	@rm -f $(REL9_DSP_CORE7_OBJECTS) $(REL9_DSP_CORE7_OUT) $(REL9_DSP_CORE7_DEPENDS)
	@echo 'Cleaning the Release9 Core7 RTSC package'
	@$(DEL) $(REL9_DSP_CORE7_CONFIGPKG)

# Dependency handling
-include $(REL9_DSP_CORE7_SOURCES:.c=.c.pp)

###################################################################################
#
# The REL9 Demo is supported only for K2H.
#
###################################################################################
ifeq ($(SYSLIB_DEVICE), k2h)
rel9Clean: rel9Core0Clean rel9Core1Clean rel9Core2Clean rel9Core3Clean \
		   rel9Core4Clean rel9Core5Clean rel9Core6Clean rel9Core7Clean
else
rel9Clean:
		@echo 'Error: REL9 Demo is NOT supported for the K2L/K2K'
endif

ifeq ($(SYSLIB_DEVICE), k2h)
rel9: rel9Core0 rel9Core1 rel9Core2 rel9Core3 rel9Core4 rel9Core5 rel9Core6 rel9Core7
else
rel9:
	@echo 'Error: REL9 Demo is NOT supported for the K2L/K2K'
endif

