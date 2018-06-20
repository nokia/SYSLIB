###################################################################################
#   JOSH Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: joshLib joshLibClean

# Sources
JOSH_LIB_SOURCES = src/josh.c src/listlib.c

# Objects
JOSH_ARM_LIB_OBJECTS = $(JOSH_LIB_SOURCES:.c=.o)

# Dependency
JOSH_LIB_DEPENDS 	 = $(JOSH_LIB_SOURCES:.c=.d)

# ARM Library Name
JOSH_LIB = lib/libjosh_$(SYSLIB_DEVICE).a

# Build the ARM Library
joshLib:$(JOSH_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(JOSH_LIB) $(JOSH_ARM_LIB_OBJECTS)

# Clean the ARM Library
joshLibClean:
	@echo 'Cleaning the ARM JOSH Library Objects'
	@rm -f $(JOSH_ARM_LIB_OBJECTS) $(JOSH_LIB) $(JOSH_LIB_DEPENDS)

# Dependency handling
-include $(JOSH_LIB_SOURCES:.c=.d)

