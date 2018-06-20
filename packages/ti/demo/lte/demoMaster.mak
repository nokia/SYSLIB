# #################################################################################
#   ARM Demo Master makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
# #################################################################################

.PHONY: demoMasterClean demoMaster

###################################################################################
# Standard definitions to build the module
###################################################################################

# RAT Master Source files.
DEMO_RAT_MASTER_SOURCES = rat/master.c rat/rel10-d2.c rat/rel10-d1.c rat/rel9.c

# Objects
DEMO_RAT_MASTER_OBJECTS = $(DEMO_RAT_MASTER_SOURCES:.c=.o)

# Dependency
DEMO_RAT_MASTER_DEPENDS = $(DEMO_RAT_MASTER_SOURCES:.c=.d)

# Demo Master Executable
DEMO_RAT_MASTER_OUT = demo_master_$(SYSLIB_DEVICE).out

demoMasterClean:
	@echo 'Cleaning the RAT Demo Master'
	@rm -f $(DEMO_RAT_MASTER_OBJECTS) $(DEMO_RAT_MASTER_OUT) $(DEMO_RAT_MASTER_DEPENDS)

$(DEMO_RAT_MASTER_OUT): $(DEMO_RAT_MASTER_OBJECTS)
	echo "Building $@"
	$(ARM_CC) -Wl,--start-group -g $(MASTER_LIBS) $(LOC_LIBS) $(DEMO_RAT_MASTER_OBJECTS) -Wl,--end-group -o $(DEMO_RAT_MASTER_OUT)
	@echo '******************************************************************************'
	@echo 'Built the RAT Demo Master'
	@echo '******************************************************************************'

demoMaster: $(DEMO_RAT_MASTER_OUT)

# Dependency handling
-include $(DEMO_RAT_MASTER_SOURCES:.c=.d)

