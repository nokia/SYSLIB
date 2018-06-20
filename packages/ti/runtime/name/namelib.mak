###################################################################################
#   NAME Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: nameLib nameLibClean

# Sources
NAME_LIB_SOURCES = src/hash.c src/name_proxy.c src/name_client.c 	\
				   src/name_listlib.c src/name_infraDMA.c 			\

# Realm specific sources
NAME_ARM_SOURCES = src/name_armDatabase.c src/name_armProxyClient.c
NAME_DSP_SOURCES = src/name_dspDatabase.c src/name_dspProxyClient.c

# Objects
NAME_ARM_LIB_OBJECTS += $(NAME_LIB_SOURCES:.c=.o)
NAME_ARM_LIB_OBJECTS += $(NAME_ARM_SOURCES:.c=.o)

# Dependency
NAME_ARM_DEPENDS	 =  $(NAME_LIB_SOURCES:.c=.d)
NAME_ARM_DEPENDS	 += $(NAME_ARM_SOURCES:.c=.d)

# ARM Library Name
NAME_LIB = lib/libname_$(SYSLIB_DEVICE).a

# Build the ARM Library
nameLib:$(NAME_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(NAME_LIB) $(NAME_ARM_LIB_OBJECTS)

# Clean the ARM Library
nameLibClean:
	@echo 'Cleaning the ARM NAME Library Objects'
	@rm -f $(NAME_ARM_LIB_OBJECTS) $(NAME_LIB) $(NAME_ARM_DEPENDS)

# Dependency handling
-include $(NAME_LIB_SOURCES:.c=.d)

