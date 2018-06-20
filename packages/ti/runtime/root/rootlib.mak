###################################################################################
#   ROOT Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: rootLib rootLibClean rootMasterLib rootMasterLibClean rootSlaveLib rootSlaveLibClean

# Sources
ROOT_MASTER_LIB_SOURCES = src/root_domain.c src/root_master.c
ROOT_SLAVE_LIB_SOURCES  = src/root_domain.c src/root_slave.c

# Realm specific sources
ROOT_ARM_SOURCE  = src/root_arm.c
ROOT_DSP_SOURCE  = src/root_dsp.c

# Objects
ROOT_MASTER_ARM_LIB_OBJECTS  = $(ROOT_MASTER_LIB_SOURCES:.c=.o)
ROOT_MASTER_ARM_LIB_OBJECTS += $(ROOT_ARM_SOURCE:.c=.o)
ROOT_SLAVE_ARM_LIB_OBJECTS   = $(ROOT_SLAVE_LIB_SOURCES:.c=.o)
ROOT_SLAVE_ARM_LIB_OBJECTS  += $(ROOT_ARM_SOURCE:.c=.o)

# Dependency
ROOT_MASTER_LIB_DEPENDS  = $(ROOT_MASTER_LIB_SOURCES:.c=.d)
ROOT_MASTER_LIB_DEPENDS += $(ROOT_ARM_SOURCE:.c=.d)
ROOT_SLAVE_LIB_DEPENDS   = $(ROOT_SLAVE_LIB_SOURCES:.c=.d)
ROOT_SLAVE_LIB_DEPENDS  += $(ROOT_ARM_SOURCE:.c=.d)

# ARM Library Name
ROOT_MASTER_LIB = lib/libroot_master_$(SYSLIB_DEVICE).a
ROOT_SLAVE_LIB  = lib/libroot_slave_$(SYSLIB_DEVICE).a

rootMasterLib: $(ROOT_MASTER_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(ROOT_MASTER_LIB) $(ROOT_MASTER_ARM_LIB_OBJECTS)

rootMasterLibClean:
	@echo 'Cleaning the ARM ROOT Master Library Objects'
	rm -f $(ROOT_MASTER_ARM_LIB_OBJECTS) $(ROOT_MASTER_LIB) $(ROOT_MASTER_LIB_DEPENDS)

rootSlaveLib: $(ROOT_SLAVE_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(ROOT_SLAVE_LIB) $(ROOT_SLAVE_ARM_LIB_OBJECTS)

rootSlaveLibClean:
	@echo 'Cleaning the ARM ROOT Slave Library Objects'
	rm -f $(ROOT_SLAVE_ARM_LIB_OBJECTS) $(ROOT_SLAVE_LIB) $(ROOT_SLAVE_LIB_DEPENDS)

# Build the ARM Library
rootLib: rootMasterLib rootSlaveLib

# Clean the ARM Library
rootLibClean: rootMasterLibClean rootSlaveLibClean

# Dependency handling
-include $(ROOT_MASTER_LIB_SOURCES:.c=.d)
-include $(ROOT_ARM_SOURCE:.c=.d)
-include $(ROOT_SLAVE_LIB_SOURCES:.c=.d)

