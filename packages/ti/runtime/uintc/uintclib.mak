###################################################################################
#   UINTC Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: uintcLib uintcLibClean

# Sources
UINTC_LIB_SOURCES = src/uintc.c src/uintc_listlib.c

# Objects
UINTC_LIB_OBJECTS = $(UINTC_LIB_SOURCES:.c=.o)

# Dependency
UINTC_LIB_DEPENDS = $(UINTC_LIB_SOURCES:.c=.d)

# ARM Library Name
UINTC_LIB = lib/libuintc_$(SYSLIB_DEVICE).a

# Build the ARM Library
uintcLib:$(UINTC_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(UINTC_LIB) $(UINTC_LIB_OBJECTS)

# Clean the ARM Library
uintcLibClean:
	@echo 'Cleaning the ARM UINTC Library Objects'
	@rm -f $(UINTC_LIB_OBJECTS) $(UINTC_LIB) $(UINTC_LIB_DEPENDS)

# Dependency handling
-include $(UINTC_LIB_SOURCES:.c=.d)

