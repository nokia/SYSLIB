###################################################################################
#   NETFP Proxy Plugin Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

###################################################################################
# ARM Only Unit Test
###################################################################################
.PHONY: plugin pluginClean

# Additional flag for Plugin:
ARM_CFLAGS += -fpic

# Source Files
PLUGIN_SOURCES =  src/netfp_proxy_custom_plugin.c src/netfp_proxy_custom.c src/fzm_l3qos_custom.c

# Objects
PLUGIN_OBJECTS = $(PLUGIN_SOURCES:.c=.o)

# Dependency
PLUGIN_DEPENDS = $(PLUGIN_SOURCES:.c=.d)

# Executable
PLUGIN_OUT     = netfpproxy_plugin_$(SYSLIB_DEVICE).so

# Plugin Shared Library
plugin: $(PLUGIN_OBJECTS)
	$(ARM_CC) -Wl,-soname=$@.1 -shared $(LDFLAGS) $(PLUGIN_OBJECTS) -o $(PLUGIN_OUT)
	@echo '******************************************************************************'
	@echo 'Built the NETFP Proxy Plugin'
	@echo '******************************************************************************'

# Plugin Cleanup
pluginClean:
	@echo 'Cleaning the Plugin Test objects'
	@rm -f $(PLUGIN_OUT) $(PLUGIN_OBJECTS) $(PLUGIN_DEPENDS)

# Dependency handling
-include $(PLUGIN_SOURCES:.c=.d)

