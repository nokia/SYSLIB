###################################################################################
#   PKTLIB ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# ARM Only Unit Test
###################################################################################
.PHONY: armTest armTestClean

# Source Files
ARM_TEST_SOURCES = test/arm/main.c test/arm/osal.c

# Objects
ARM_TEST_OBJECTS = $(ARM_TEST_SOURCES:.c=.o)

# Dependency
ARM_TEST_DEPENDS = $(ARM_TEST_SOURCES:.c=.d)

# Executable
ARM_TEST_OUT     = test/arm/test_pktlib_$(SYSLIB_DEVICE).out

# Test Executable
armTest: $(ARM_TEST_OBJECTS)
	$(ARM_CC) -Wl,--start-group $(ARM_LDFLAGS) $(ARM_TEST_OBJECTS) -Wl,--end-group -o $(ARM_TEST_OUT)
	@echo '******************************************************************************'
	@echo 'Built the PKTLIB ARM Executable '
	@echo '******************************************************************************'

# Test Cleanup
armTestClean:
	@echo 'Cleaning the ARM Unit Test objects'
	@rm -f $(ARM_TEST_OUT) $(ARM_TEST_OBJECTS) $(ARM_TEST_DEPENDS)

# Dependency handling
-include $(ARM_TEST_SOURCES:.c=.d)


