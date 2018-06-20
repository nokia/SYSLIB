###################################################################################
#   DAT Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: datLib datLibClean

# Sources
DAT_LIB_SOURCES = src/dat_listlib.c src/dat_srv.c src/dat_client.c src/dat_log.c src/dat_uio.c

# Objects
DAT_ARM_LIB_OBJECTS += $(DAT_LIB_SOURCES:.c=.o)

# Dependency
DAT_LIB_DEPENDS = $(DAT_LIB_SOURCES:.c=.d)

# ARM Library Name
DAT_LIB = lib/libdat_$(SYSLIB_DEVICE).a

# Build the ARM Library
datLib:$(DAT_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(DAT_LIB) $(DAT_ARM_LIB_OBJECTS)

# Clean the ARM Library
datLibClean:
	@echo 'Cleaning the ARM DAT Library Objects'
	@rm -f $(DAT_ARM_LIB_OBJECTS) $(DAT_LIB) $(DAT_LIB_DEPENDS)

# Dependency handling
-include $(DAT_LIB_SOURCES:.c=.d)

