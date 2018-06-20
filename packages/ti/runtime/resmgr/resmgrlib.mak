###################################################################################
#   RESMGR Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: resmgrLib resmgrLibClean

# Sources
RESMGR_LIB_SOURCES = src/resmgr.c src/resmgr_sockClient.c src/resmgr_k2l.c \
					 src/resmgr_k2hk.c

# Objects
RESMGR_LIB_OBJECTS = $(RESMGR_LIB_SOURCES:.c=.o)

# Dependency
RESMGR_LIB_DEPENDS = $(RESMGR_LIB_SOURCES:.c=.d)

# ARM Library Name
RESMGR_LIB = lib/libresmgr_$(SYSLIB_DEVICE).a

# Build the ARM Library
resmgrLib:$(RESMGR_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(RESMGR_LIB) $(RESMGR_LIB_OBJECTS)

# Clean the ARM Library
resmgrLibClean:
	@echo 'Cleaning the ARM RESMGR Library Objects'
	rm -f $(RESMGR_LIB_OBJECTS) $(RESMGR_LIB) $(RESMGR_LIB_DEPENDS)

# Dependency handling
-include $(RESMGR_LIB_SOURCES:.c=.d)

