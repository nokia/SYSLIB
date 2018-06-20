###################################################################################
#   NETFP ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# ARM Only Unit Test
###################################################################################
.PHONY: armTest armTestClean

# Source Files
ARM_TEST_SOURCES = 	test/arm/main.c test/arm/osal.c test/arm/setup.c test/arm/socketv4.c     		\
					test/arm/test_srb.c test/arm/encode.c test/arm/sp_reestablishment.c             \
					test/arm/fp_reestablishment.c test/arm/basic.c test/arm/routeRecalculation.c    \
					test/arm/routeRecalculationv6.c test/arm/srcrouting.c                           \
					test/arm/wildcardingv4.c test/arm/sourceHandOver.c test/arm/fp_targetHandOver.c \
					test/arm/sp_targetHandOver.c test/arm/multicast.c test/arm/reestablishment.c	\
					test/arm/basicReestablishment.c test/arm/basic_ho.c

# Objects
ARM_TEST_OBJECTS = $(ARM_TEST_SOURCES:.c=.o)

# Dependency
ARM_TEST_DEPENDS = $(ARM_TEST_SOURCES:.c=.d)

# Executable
ARM_TEST_OUT     = test/arm/test_netfp_$(SYSLIB_DEVICE).out

# Test Executable
armTest: $(ARM_TEST_OBJECTS)
	$(ARM_CC) -Wl,--start-group $(STD_LIBS) $(LOC_LIBS) $(ARM_TEST_OBJECTS) -Wl,--end-group -o $(ARM_TEST_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP ARM Executable '
	@echo '******************************************************************************'

# Test Cleanup
armTestClean:
	@echo 'Cleaning the ARM Unit Test objects'
	@rm -f $(ARM_TEST_OUT) $(ARM_TEST_OBJECTS) $(ARM_TEST_DEPENDS)

# Dependency handling
-include $(ARM_TEST_SOURCES:.c=.d)

