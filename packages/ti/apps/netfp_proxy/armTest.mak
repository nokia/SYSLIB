###################################################################################
#   NETFP Master ARM Unit Test makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# ARM Only Unit Test
###################################################################################
.PHONY: armTest armTestClean

# Source Files
ARM_TEST_SOURCES = test/cmd_shell_app/cmd_shell.c test/cmd_shell_app/cmd_shell_main.c \
				   test/cmd_shell_app/listlib.c

# Objects
ARM_TEST_OBJECTS = $(ARM_TEST_SOURCES:.c=.o)

# Dependency
ARM_TEST_DEPENDS = $(ARM_TEST_SOURCES:.c=.d)

# Executable
ARM_TEST_OUT     = test/cmd_shell_app/cmd_shell_$(SYSLIB_DEVICE).out

# Test Executable
armTest: $(ARM_TEST_OBJECTS)
	$(ARM_CC) -Wl,--start-group -L$(SYSLIB_INSTALL_PATH)/ti/runtime/name/lib -lrt -lpthread -lname_$(SYSLIB_DEVICE) $(ARM_TEST_OBJECTS) \
	-Wl,--end-group -o $(ARM_TEST_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP Proxy command shell application'
	@echo '******************************************************************************'

# Test Cleanup
armTestClean:
	@echo 'Cleaning the ARM Unit Test objects'
	@rm -f $(ARM_TEST_OUT) $(ARM_TEST_OBJECTS) $(ARM_TEST_DEPENDS)

# Dependency handling
-include $(ARM_TEST_SOURCES:.c=.d)

