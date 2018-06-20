###################################################################################
#   NAME Utils makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# Utils
###################################################################################
.PHONY: utils utilsClean

# Source Files
UTILS_SOURCES = utils/nr_mgr.c utils/osal.c

# Objects
UTILS_OBJECTS = $(UTILS_SOURCES:.c=.o)

# Dependency
UTILS_DEPENDS = $(UTILS_SOURCES:.c=.d)

# Executable
UTILS_OUT     = utils/nr_mgr_$(SYSLIB_DEVICE).out

# Test Executable
utils: $(UTILS_OBJECTS)
	$(ARM_CC) -Wl,--start-group $(STD_LIBS) $(LOC_LIBS) $(UTILS_OBJECTS) -Wl,--end-group -o $(UTILS_OUT)

# Test Cleanup
utilsClean:
	@echo 'Cleaning the Name Utils objects'
	@rm -f $(UTILS_OUT) $(UTILS_OBJECTS) $(UTILS_DEPENDS)

# Dependency handling
-include $(UTILS_SOURCES:.c=.d)




