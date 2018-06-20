###################################################################################
#   MSGCOM DSP-ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# DSP-ARM: ARM Test Application
###################################################################################
.PHONY: dspArm dspArmClean

# ARM Source files
DSP_ARM_TEST_ARMSOURCES =	test/dsp_arm/main.c test/dsp_arm/osal.c 			\
							test/dsp_arm/reader.c test/dsp_arm/writer.c

# Objects
DSP_ARM_TEST_ARMOBJECTS = $(DSP_ARM_TEST_ARMSOURCES:.c=.o)

# Dependency
DSP_ARM_TEST_DEPENDS 	= $(DSP_ARM_TEST_ARMSOURCES:.c=.d)

# Executable
DSP_ARM_TEST_ARMOUT	 	= test/dsp_arm/test_dsp_arm_msgcom_$(SYSLIB_DEVICE).out

dspArm: $(DSP_ARM_TEST_ARMOBJECTS)
	$(ARM_CC) -Wl,--start-group $(STD_LIBS) $(LOC_LIBS) $(DSP_ARM_TEST_ARMOBJECTS) -Wl,--end-group -o $(DSP_ARM_TEST_ARMOUT)
	@echo '******************************************************************************'
	@echo 'Built the MSGCOM DSP-ARM ARM Executable '
	@echo '******************************************************************************'

dspArmClean:
	@echo 'Cleaning the DSP ARM Unit Test objects'
	@rm -f $(DSP_ARM_TEST_ARMOUT) $(DSP_ARM_TEST_ARMOBJECTS) $(DSP_ARM_TEST_DEPENDS)

# ARM Dependency handling
-include $(DSP_ARM_TEST_ARMSOURCES:.c=.d)


