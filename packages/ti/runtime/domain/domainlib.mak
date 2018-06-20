###################################################################################
#   DOMAIN Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: domainLib domainLibClean

# Sources
DOMAIN_LIB_SOURCES = src/domain.c

# Objects
DOMAIN_ARM_LIB_OBJECTS = $(DOMAIN_LIB_SOURCES:.c=.o)

# Dependency
DOMAIN_LIB_DEPENDS 	 = $(DOMAIN_LIB_SOURCES:.c=.d)

# ARM Library Name
DOMAIN_LIB = lib/libdomain_$(SYSLIB_DEVICE).a

# Additional compilation flags
ARM_CFLAGS += -Dxdc_target_name__=GCArmv5T -Dxdc_target_types__=gnu/targets/arm/std.h

# Build the ARM Library
domainLib:$(DOMAIN_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(DOMAIN_LIB) $(DOMAIN_ARM_LIB_OBJECTS)

# Clean the ARM Library
domainLibClean:
	@echo 'Cleaning the ARM DOMAIN Library Objects'
	@rm -f $(DOMAIN_ARM_LIB_OBJECTS) $(DOMAIN_LIB) $(DOMAIN_LIB_DEPENDS)

# Dependency handling
-include $(DOMAIN_LIB_SOURCES:.c=.d)

