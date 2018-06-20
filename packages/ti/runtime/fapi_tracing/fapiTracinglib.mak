###################################################################################
#   FAPI Tracing Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: fapiTracingLib fapiTracingLibClean

# Sources
FAPI_TRACING_LIB_SOURCES = src/fapi_tracing.c

# Objects
FAPI_TRACING_ARM_LIB_OBJECTS += $(FAPI_TRACING_LIB_SOURCES:.c=.o)

# Dependency
FAPI_TRACING_LIB_DEPENDS = $(FAPI_TRACING_LIB_SOURCES:.c=.d)

# ARM Library Name
FAPI_TRACING_LIB = lib/libfapitracing_$(SYSLIB_DEVICE).a

# Build the ARM Library
fapiTracingLib:$(FAPI_TRACING_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(FAPI_TRACING_LIB) $(FAPI_TRACING_ARM_LIB_OBJECTS)

# Clean the ARM Library
fapiTracingLibClean:
	@echo 'Cleaning the ARM FAPI Tracing Library Objects'
	@rm -f $(FAPI_TRACING_ARM_LIB_OBJECTS) $(FAPI_TRACING_LIB) $(FAPI_TRACING_LIB_DEPENDS)

# Dependency handling
-include $(FAPI_TRACING_LIB_SOURCES:.c=.d)

