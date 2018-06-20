# #################################################################################
#   REL10-Deployment1 Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

.PHONY: rel10D1Clean rel10D1

###################################################################################
# Rel10-Deployment1: L2 ARM Application
###################################################################################

# RAT Master Source files.
DEMO_RAT_REL10D1_SOURCES = common/root_arm_client.c rel10-d1/l2/l2.c rel10-d1/l2/fapi.c rel10-d1/l2/osal.c \
						   rel10-d1/l2/logging.c rel10-d1/l2/l2_dp.c

# Objects
DEMO_RAT_REL10D1_OBJECTS = $(DEMO_RAT_REL10D1_SOURCES:.c=.o)

# Dependency
DEMO_RAT_REL10D1_DEPENDS = $(DEMO_RAT_REL10D1_SOURCES:.c=.d)

# Demo Rel10-Deployment1 L2 Executable
DEMO_RAT_REL10D1_OUT = demo_rel10d1_l2_$(SYSLIB_DEVICE).out

$(DEMO_RAT_REL10D1_OUT): $(DEMO_RAT_REL10D1_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group -g $(L2_LIBS) $(LOC_LIBS) $(DEMO_RAT_REL10D1_OBJECTS) -Wl,--end-group -o $(DEMO_RAT_REL10D1_OUT)
	@echo '******************************************************************************'
	@echo 'Built the RAT Demo Deployment1 L2'
	@echo '******************************************************************************'

rel10D1L2Clean:
		@echo 'Cleaning the RAT Demo Deployment1 L2'
		rm -f $(DEMO_RAT_REL10D1_OBJECTS) $(DEMO_RAT_REL10D1_OUT) $(DEMO_RAT_REL10D1_DEPENDS)

rel10D1L2: $(DEMO_RAT_REL10D1_OUT)

# Dependency handling
-include $(DEMO_RAT_REL10D1_SOURCES:.c=.d)

###################################################################################
# Rel10-Deployment1: DSP Core0
###################################################################################

.PHONY: rel10D1Core0 rel10D1Core0Clean

# Core0 Source Files
REL10_DSP_CORE0_CFG	  = rel10-d1/sim_phy/core0.cfg
REL10_DSP_CORE0_CMD       = rel10-d1/sim_phy/linker_core0.cmd
REL10_DSP_CORE0_CONFIGPKG = rel10-d1/sim_phy/core0_configPkg_$(SYSLIB_DEVICE)
REL10_DSP_CORE0_MAP       = $(REL10_DSP_CORE0_CONFIGPKG)/rel10D1_core0.map
REL10_DSP_CORE0_OUT       = $(REL10_DSP_CORE0_CONFIGPKG)/rel10D1_core0.out
REL10_DSP_CORE0_SOURCES   = common/root_dsp_client.c rel10-d1/sim_phy/sim_phy_slave.c 					\
							rel10-d1/sim_phy/osal.c	rel10-d1/sim_phy/logging.c							\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_DSP_CORE0_DEPENDS = $(REL10_DSP_CORE0_SOURCES:.c=.c.pp)
REL10_DSP_CORE0_OBJECTS = $(REL10_DSP_CORE0_SOURCES:.c=.obj)

# Core0 RTSC Configuration
rel10D1Core0.cfg: $(REL10_DSP_CORE0_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_DSP_CORE0_CONFIGPKG) $(REL10_DSP_CORE0_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core0 Build Target
rel10D1Core0: BUILD_CONFIGPKG=$(REL10_DSP_CORE0_CONFIGPKG)
rel10D1Core0: rel10D1Core0.cfg $(REL10_DSP_CORE0_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_DSP_CORE0_MAP) -l$(REL10_DSP_CORE0_CONFIGPKG)/linker.cmd -o $(REL10_DSP_CORE0_OUT) $(REL10_DSP_CORE0_OBJECTS) -l"libc.a" $(REL10_DSP_CORE0_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Rel10 Deployment1 Core0 Executable '
		@echo '******************************************************************************'

# Core0 Cleanup
rel10D1Core0Clean:
	@echo 'Cleaning the Rel10 Deployment1 Core0 objects'
	@rm -f $(REL10_DSP_CORE0_OBJECTS) $(REL10_DSP_CORE0_OUT) $(REL10_DSP_CORE0_DEPENDS)
	@echo 'Cleaning the Rel10 Deployment1 Core0 RTSC package'
	@$(DEL) $(REL10_DSP_CORE0_CONFIGPKG)

# Dependency handling
-include $(REL10_DSP_CORE0_SOURCES:.c=.c.pp)

###################################################################################
# Rel10-Deployment1: DSP Core1
###################################################################################

.PHONY: rel10D1Core1 rel10D1Core1Clean

# Core1 Source Files
REL10_DSP_CORE1_CFG		  = rel10-d1/sim_phy/core1.cfg
REL10_DSP_CORE1_CMD       = rel10-d1/sim_phy/linker_core1.cmd
REL10_DSP_CORE1_CONFIGPKG = rel10-d1/sim_phy/core1_configPkg_$(SYSLIB_DEVICE)
REL10_DSP_CORE1_MAP       = $(REL10_DSP_CORE1_CONFIGPKG)/rel10D1_core1.map
REL10_DSP_CORE1_OUT       = $(REL10_DSP_CORE1_CONFIGPKG)/rel10D1_core1.out
REL10_DSP_CORE1_SOURCES   = common/root_dsp_client.c rel10-d1/sim_phy/sim_phy_master.c 					\
							rel10-d1/sim_phy/osal.c	rel10-d1/sim_phy/fapi.c rel10-d1/sim_phy/logging.c	\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_DSP_CORE1_DEPENDS = $(REL10_DSP_CORE1_SOURCES:.c=.c.pp)
REL10_DSP_CORE1_OBJECTS = $(REL10_DSP_CORE1_SOURCES:.c=.obj)

# Core1 RTSC Configuration
rel10D1Core1.cfg: $(REL10_DSP_CORE1_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_DSP_CORE1_CONFIGPKG) $(REL10_DSP_CORE1_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core1 Build Target
rel10D1Core1: BUILD_CONFIGPKG=$(REL10_DSP_CORE1_CONFIGPKG)
rel10D1Core1: rel10D1Core1.cfg $(REL10_DSP_CORE1_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_DSP_CORE1_MAP) -l$(REL10_DSP_CORE1_CONFIGPKG)/linker.cmd -o $(REL10_DSP_CORE1_OUT) $(REL10_DSP_CORE1_OBJECTS) -l"libc.a" $(REL10_DSP_CORE1_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Rel10 Deployment1 Core1 Executable '
		@echo '******************************************************************************'

# Core1 Cleanup
rel10D1Core1Clean:
	@echo 'Cleaning the Rel10 Deployment1 Core1 objects'
	@rm -f $(REL10_DSP_CORE1_OBJECTS) $(REL10_DSP_CORE1_OUT) $(REL10_DSP_CORE1_DEPENDS)
	@echo 'Cleaning the Rel10 Deployment1 Core1 RTSC package'
	@$(DEL) $(REL10_DSP_CORE1_CONFIGPKG)

# Dependency handling
-include $(REL10_DSP_CORE1_SOURCES:.c=.c.pp)

###################################################################################
# Rel10-Deployment1: DSP Core2
###################################################################################

.PHONY: rel10D1Core2 rel10D1Core2Clean

# Core2 Source Files
REL10_DSP_CORE2_CFG		  = rel10-d1/sim_phy/core2.cfg
REL10_DSP_CORE2_CMD       = rel10-d1/sim_phy/linker_core2.cmd
REL10_DSP_CORE2_CONFIGPKG = rel10-d1/sim_phy/core2_configPkg_$(SYSLIB_DEVICE)
REL10_DSP_CORE2_MAP       = $(REL10_DSP_CORE2_CONFIGPKG)/rel10D1_core2.map
REL10_DSP_CORE2_OUT       = $(REL10_DSP_CORE2_CONFIGPKG)/rel10D1_core2.out
REL10_DSP_CORE2_SOURCES   = common/root_dsp_client.c rel10-d1/sim_phy/sim_phy_slave.c 					\
							rel10-d1/sim_phy/osal.c	rel10-d1/sim_phy/logging.c							\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_DSP_CORE2_DEPENDS = $(REL10_DSP_CORE2_SOURCES:.c=.c.pp)
REL10_DSP_CORE2_OBJECTS = $(REL10_DSP_CORE2_SOURCES:.c=.obj)

# Core2 RTSC Configuration
rel10D1Core2.cfg: $(REL10_DSP_CORE2_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_DSP_CORE2_CONFIGPKG) $(REL10_DSP_CORE2_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core2 Build Target
rel10D1Core2: BUILD_CONFIGPKG=$(REL10_DSP_CORE2_CONFIGPKG)
rel10D1Core2: rel10D1Core2.cfg $(REL10_DSP_CORE2_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_DSP_CORE2_MAP) -l$(REL10_DSP_CORE2_CONFIGPKG)/linker.cmd -o $(REL10_DSP_CORE2_OUT) $(REL10_DSP_CORE2_OBJECTS) -l"libc.a" $(REL10_DSP_CORE2_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Rel10 Deployment1 Core2 Executable '
		@echo '******************************************************************************'

# Core2 Cleanup
rel10D1Core2Clean:
	@echo 'Cleaning the Rel10 Deployment1 Core2 objects'
	@rm -f $(REL10_DSP_CORE2_OBJECTS) $(REL10_DSP_CORE2_OUT) $(REL10_DSP_CORE2_DEPENDS)
	@echo 'Cleaning the Rel10 Deployment1 Core2 RTSC package'
	@$(DEL) $(REL10_DSP_CORE2_CONFIGPKG)

# Dependency handling
-include $(REL10_DSP_CORE2_SOURCES:.c=.c.pp)

###################################################################################
# Rel10-Deployment1: DSP Core3
###################################################################################

.PHONY: rel10D1Core3 rel10D1Core3Clean

# Core3 Source Files
REL10_DSP_CORE3_CFG		  = rel10-d1/sim_phy/core3.cfg
REL10_DSP_CORE3_CMD       = rel10-d1/sim_phy/linker_core3.cmd
REL10_DSP_CORE3_CONFIGPKG = rel10-d1/sim_phy/core3_configPkg_$(SYSLIB_DEVICE)
REL10_DSP_CORE3_MAP       = $(REL10_DSP_CORE3_CONFIGPKG)/rel10D1_core3.map
REL10_DSP_CORE3_OUT       = $(REL10_DSP_CORE3_CONFIGPKG)/rel10D1_core3.out
REL10_DSP_CORE3_SOURCES   = common/root_dsp_client.c rel10-d1/sim_phy/sim_phy_master.c 					\
							rel10-d1/sim_phy/osal.c	rel10-d1/sim_phy/fapi.c rel10-d1/sim_phy/logging.c	\
							$(PDK_INSTALL_PATH)/ti/drv/cppi/device/$(SYSLIB_DEVICE)/src/cppi_device.c   \
							$(PDK_INSTALL_PATH)/ti/drv/qmss/device/$(SYSLIB_DEVICE)/src/qmss_device.c
REL10_DSP_CORE3_DEPENDS = $(REL10_DSP_CORE3_SOURCES:.c=.c.pp)
REL10_DSP_CORE3_OBJECTS = $(REL10_DSP_CORE3_SOURCES:.c=.obj)

# Core3 RTSC Configuration
rel10D1Core3.cfg: $(REL10_DSP_CORE3_CFG)
	@echo 'Configuring RTSC packages...'
	$(DSP_XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(DSP_XSFLAGS) -o $(REL10_DSP_CORE3_CONFIGPKG) $(REL10_DSP_CORE3_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

# Core3 Build Target
rel10D1Core3: BUILD_CONFIGPKG=$(REL10_DSP_CORE3_CONFIGPKG)
rel10D1Core3: rel10D1Core3.cfg $(REL10_DSP_CORE3_OBJECTS)
		$(DSP_CC) $(DSP_LDFLAGS) -m $(REL10_DSP_CORE3_MAP) -l$(REL10_DSP_CORE3_CONFIGPKG)/linker.cmd -o $(REL10_DSP_CORE3_OUT) $(REL10_DSP_CORE3_OBJECTS) -l"libc.a" $(REL10_DSP_CORE3_CMD)
		@echo '******************************************************************************'
		@echo 'Built the Rel10 Deployment1 Core3 Executable '
		@echo '******************************************************************************'

# Core3 Cleanup
rel10D1Core3Clean:
	@echo 'Cleaning the Rel10 Deployment1 Core3 objects'
	@rm -f $(REL10_DSP_CORE3_OBJECTS) $(REL10_DSP_CORE3_OUT) $(REL10_DSP_CORE3_DEPENDS)
	@echo 'Cleaning the Rel10 Deployment1 Core3 RTSC package'
	@$(DEL) $(REL10_DSP_CORE3_CONFIGPKG)

# Dependency handling
-include $(REL10_DSP_CORE3_SOURCES:.c=.c.pp)

###################################################################################
# DEMO Supported are only K2H and K2L. K2K is not a supported DEMO.
###################################################################################
ifneq ($(SYSLIB_DEVICE), k2k)
rel10D1:      rel10D1L2      rel10D1Core0      rel10D1Core1      rel10D1Core2      rel10D1Core3
rel10D1Clean: rel10D1L2Clean rel10D1Core0Clean rel10D1Core1Clean rel10D1Core2Clean rel10D1Core3Clean
else
rel10D1:
	@echo 'Error: K2K Demo is NOT supported. Please refer to the K2H demo as an example'
rel10D1Clean:
		@echo 'Error: K2K Demo is NOT supported. Please refer to the K2H demo as an example'
endif

