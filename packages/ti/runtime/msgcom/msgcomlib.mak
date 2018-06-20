###################################################################################
#   MSGCOM Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: msgcomLib msgcomLibClean

# Sources
MSGCOM_LIB_SOURCES = src/msgcom.c src/msgcom_queue.c src/msgcom_queueDMA.c src/listlib.c

# Objects
MSGCOM_ARM_LIB_OBJECTS += $(MSGCOM_LIB_SOURCES:.c=.o)

# Dependency
MSGCOM_LIB_DEPENDS = $(MSGCOM_LIB_SOURCES:.c=.d)

# ARM Library Name
MSGCOM_LIB = lib/libmsgcom_$(SYSLIB_DEVICE).a

# Build the ARM Library
msgcomLib:$(MSGCOM_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(MSGCOM_LIB) $(MSGCOM_ARM_LIB_OBJECTS)

# Clean the ARM Library
msgcomLibClean:
	@echo 'Cleaning the ARM MSGCOM Library Objects'
	@rm -f $(MSGCOM_ARM_LIB_OBJECTS) $(MSGCOM_LIB) $(MSGCOM_LIB_DEPENDS)

# Dependency handling
-include $(MSGCOM_LIB_SOURCES:.c=.d)

