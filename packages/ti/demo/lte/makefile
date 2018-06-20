# #################################################################################
#   Demo makefile
#
#  NOTE:
#      (C) Copyright 2013-2014 Texas Instruments, Inc.
# #################################################################################

include rel9.mak
include rel10D1.mak
include rel10D2.mak
include demoMaster.mak

###################################################################################
# Standard Definitions
###################################################################################
STD_INCL = -I$(SYSLIB_INSTALL_PATH) -I$(CUIA_INSTALL_PATH)/include -I$(ARAGODIR)/include
LOC_LIBS = -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/josh/lib 			\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/resmgr/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/pktlib/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/msgcom/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/netfp/lib 	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/uintc/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/domain/lib		\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/root/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/fapi_tracing/lib	\
		   -L$(SYSLIB_INSTALL_PATH)/ti/runtime/dat/lib -L$(SYSLIB_INSTALL_PATH)/ti/runtime/memlog/lib           \
                   -L$(CUIA_INSTALL_PATH)/lib -L$(ARAGODIR)/lib 
MASTER_LIBS = -lrt -lpthread -lroot_master_$(SYSLIB_DEVICE) -ljosh_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE)	        \
	      -lname_$(SYSLIB_DEVICE)
L2_LIBS = -lroot_slave_$(SYSLIB_DEVICE) -ldomain_$(SYSLIB_DEVICE) -lfapitracing_$(SYSLIB_DEVICE) 			\
		  -lnetfp_$(SYSLIB_DEVICE) -lrt -lname_$(SYSLIB_DEVICE) -ldat_$(SYSLIB_DEVICE) -lmemlog_$(SYSLIB_DEVICE)\
		  -ljosh_$(SYSLIB_DEVICE) -lsa -lpktlib_$(SYSLIB_DEVICE) -lmsgcom_$(SYSLIB_DEVICE) 			\
		  -lresmgr_$(SYSLIB_DEVICE) -luintc_$(SYSLIB_DEVICE) -lrm -lhplib-no-osal_$(SYSLIB_DEVICE) 		\
		  -lcppi_$(SYSLIB_DEVICE) -lqmss_$(SYSLIB_DEVICE) -lpthread -lLoggerStreamer2 -lLogSync
# K2L requires a different version of the PA Library
ifeq ($(SYSLIB_DEVICE), k2l)
L2_LIBS += -lpa2
else
L2_LIBS += -lpa
endif

###################################################################################
# Tool Definitions:
###################################################################################
ARM_CC  = $(CROSS_COMPILE)gcc
ARM_LD 	= $(CROSS_COMPILE)ld
ARM_AR 	= $(CROSS_COMPILE)ar
DSP_XS  = xs
DSP_CC  = cl6x
DSP_XDC = xdc
DEL     = rm -Rf

###################################################################################
# Platform specific definitions
###################################################################################
ifeq ($(SYSLIB_DEVICE), k2h)
PLATFORM = ti.runtime.platforms.tmdxevm6638lxe
PLATFORM_DEFINE   	= DEVICE_K2H
endif
ifeq ($(SYSLIB_DEVICE), k2k)
PLATFORM = ti.runtime.platforms.tmdxevm6638lxe
PLATFORM_DEFINE   	= DEVICE_K2K
endif
ifeq ($(SYSLIB_DEVICE), k2l)
PLATFORM = ti.runtime.platforms.k2l
PLATFORM_DEFINE		= DEVICE_K2L
endif

###################################################################################
# ARM Compiler & Linker Flags
###################################################################################
ARM_CFLAGS  = -g -Wall $(STD_INCL) -MD -D_LITTLE_ENDIAN -D__ARMv7 -DDEVICE_K2 						\
			  -D$(PLATFORM_DEFINE) -D_GNU_SOURCE -D_VIRTUAL_ADDR_SUPPORT							\
			  $(LOC_INCL) -Dxdc_target_name__=GCArmv5T -Dxdc_target_types__=gnu/targets/arm/std.h
ARM_AR_OPTS = rcs

###################################################################################
# DSP Compiler & Linker Flags
###################################################################################
DSP_XSFLAGS  = -t ti.targets.elf.C66 -DSYSLIB_DEVICE=$(SYSLIB_DEVICE) -p $(PLATFORM) -r debug -c $(CGT_INSTALL_PATH)
DSP_INCLUDE  = -i$(CGT_INSTALL_PATH) -i$(CGT_INSTALL_PATH)/include
DSP_CFLAGS   = -mv6600 --abi=eabi -g --define=$(PLATFORM_DEFINE) --define=DEVICE_K2 								\
			   --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile $(DSP_INCLUDE)
DSP_LDFLAGS  = -mv6600 --abi=eabi -g --define=DEVICE_K2 --display_error_number --diag_warning=225 					\
    		   --diag_wrap=off -z --reread_libs --warn_sections --display_error_number --diag_wrap=off --rom_model 	\
		       -i$(CGT_INSTALL_PATH)/lib

###################################################################################
# Build Suffix
###################################################################################
.c.o:
	@echo '[ARM Device: $(SYSLIB_DEVICE)] Building file: $<'
	@$(ARM_CC) $(ARM_CFLAGS) -c $< -o $@

%.obj: %.c
	@echo '[DSP Device: $(SYSLIB_DEVICE)] Building file: $<'
	@$(DSP_CC) -c $(DSP_CFLAGS) --preproc_dependency="$<.pp" -@ $(BUILD_CONFIGPKG)/compiler.opt "$<" --output_file $@

###################################################################################
# Standard Targets which need to be implemented by each SYSLIB module. This plugs
# into the release scripts.
###################################################################################

.PHONY: all clean

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the LTE DEMO '
	@echo 'clean                -> Clean out all the objects'
	@echo 'all                  -> Build all the applicable demos for the device'
	@echo 'demoMaster           -> Build the DEMO Master Executable'
	@echo 'demoMasterClean      -> Clean the DEMO Master only'
	@echo 'rel9                 -> Build the Release9 Executables [K2H Only]'
	@echo 'rel9Clean            -> Clean the Release9 Executables [K2H Only]'
	@echo 'rel10D1              -> Build the REL10-Deployment1 Executables [K2H,K2L Only]'
	@echo 'rel10D1Clean         -> Clean the REL10-Deployment1 Executables [K2H,K2L Only]'
	@echo 'rel10D2              -> Build the REL10-Deployment2 Executables [K2H Only]'
	@echo 'rel10D2Clean         -> Clean the REL10-Deployment2 Executables [K2H Only]'
	@echo '****************************************************************************************'

ifeq ($(SYSLIB_DEVICE), k2h)
###################################################################################
# On K2H; we support all the 3 DEMO versions:
# 	- Release 9
# 	- Release 10 Deployment 1
# 	- Release 10 Deployment 2
###################################################################################
all:   demoMaster rel10D1 rel10D2 rel9
clean: demoMasterClean rel10D1Clean rel10D2Clean rel9Clean
endif

ifeq ($(SYSLIB_DEVICE), k2k)
###################################################################################
# On K2K; we dont support any of the DEMO versions. K2H Demo can be used instead
# as a starting point.
###################################################################################
all:
	@echo 'Demo is not supported on K2K; please refer to the K2H Demo'
clean:
	@echo 'Demo is not supported on K2K; please refer to the K2H Demo'
endif

ifeq ($(SYSLIB_DEVICE), k2l)
###################################################################################
# On K2L; we support
# 	- Release 10 Deployment 1
###################################################################################
all:   demoMaster rel10D1
clean: demoMasterClean rel10D1Clean
endif

