###################################################################################
#   PKTLIB Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: pktLib pktLibClean

# Sources
PKTLIB_LIB_SOURCES = src/pktlib.c

# Objects
PKTLIB_ARM_LIB_OBJECTS += $(PKTLIB_LIB_SOURCES:.c=.o)

# Dependency
PKTLIB_LIB_DEPENDS = $(PKTLIB_LIB_SOURCES:.c=.d)

# ARM Library Name
PKTLIB_LIB = lib/libpktlib_$(SYSLIB_DEVICE).a

# Build the ARM Library
pktLib:$(PKTLIB_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(PKTLIB_LIB) $(PKTLIB_ARM_LIB_OBJECTS)

# Clean the ARM Library
pktLibClean:
	@echo 'Cleaning the ARM PKTLIB Library Objects'
	@rm -f $(PKTLIB_ARM_LIB_OBJECTS) $(PKTLIB_LIB) $(PKTLIB_LIB_DEPENDS)

# Dependency handling
-include $(PKTLIB_LIB_SOURCES:.c=.d)

