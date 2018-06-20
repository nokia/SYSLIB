###################################################################################
#   SYSLIB common makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################
QUIET=@


###################################################################################
# Tool Definitions:
###################################################################################
DSP_XS  = xs
DSP_CC  = cl6x
DSP_XDC = xdc
ARM_CC  = $(CROSS_COMPILE)gcc
ARM_LD  = $(CROSS_COMPILE)ld
ARM_AR  = $(CROSS_COMPILE)ar

###################################################################################
# Common Definitions:
###################################################################################
DEL     = rm -Rf
DOXYGEN = doxygen

###################################################################################
# Platform specific definitions
###################################################################################
ifeq ($(SYSLIB_DEVICE), k2h)
  PLATFORM        = ti.runtime.platforms.tmdxevm6638lxe
  PLATFORM_DEFINE = DEVICE_K2H
endif
ifeq ($(SYSLIB_DEVICE), k2k)
  PLATFORM        = ti.runtime.platforms.tmdxevm6638lxe
  PLATFORM_DEFINE = DEVICE_K2K
endif
ifeq ($(SYSLIB_DEVICE), k2l)
  PLATFORM        = ti.runtime.platforms.k2l
  PLATFORM_DEFINE = DEVICE_K2L
endif

###################################################################################
# ARM Compiler & Linker Flags
###################################################################################
#TODO
#enable -Wmissing-prototypes
ARM_CFLAGS  = \
  -O3 \
  -Wall -Wextra -Wno-sign-compare -Wstrict-prototypes -Wshadow -Wformat=2 -Wwrite-strings \
  -g $(STD_INCL) -MD -MP -D_LITTLE_ENDIAN -D__ARMv7 -DDEVICE_K2 \
  -D$(PLATFORM_DEFINE) -D_GNU_SOURCE -D_VIRTUAL_ADDR_SUPPORT \
  $(NOK_INC) \
  $(LOC_INCL)
ARM_LDFLAGS = -g $(STD_LIBS) $(LOC_LIBS)
ARM_AR_OPTS = rcs

###################################################################################
# DSP Compiler & Linker Flags
###################################################################################
DSP_XSFLAGS  = -t ti.targets.elf.C66 -DSYSLIB_DEVICE=$(SYSLIB_DEVICE) -p $(PLATFORM) -r debug -c $(CGT_INSTALL_PATH)
DSP_INCLUDE  = -i$(CGT_INSTALL_PATH) -i$(CGT_INSTALL_PATH)/include
DSP_CFLAGS   = \
   -mv6600 --abi=eabi -g --define=$(PLATFORM_DEFINE) --define=DEVICE_K2 \
   --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile $(DSP_INCLUDE)
DSP_LDFLAGS  = \
   -mv6600 --abi=eabi -g --define=$(PLATFORM_DEFINE) --define=DEVICE_K2 --display_error_number --diag_warning=225 \
   --diag_wrap=off -z --reread_libs --warn_sections --display_error_number --diag_wrap=off --rom_model \
   -i$(CGT_INSTALL_PATH)/lib

###################################################################################
# Build Suffix
###################################################################################
.c.o:
	@echo '[ARM Device: $(SYSLIB_DEVICE)] Building file: $<'
	$(QUIET)$(ARM_CC) $(ARM_CFLAGS) -c $< -o $@

%.obj: %.c
	@echo '[DSP Device: $(SYSLIB_DEVICE)] Building file: $<'
	$(QUIET)$(DSP_CC) -c $(DSP_CFLAGS) --preproc_dependency="$<.pp" -@ $(BUILD_CONFIGPKG)/compiler.opt "$<" --output_file $@

