###################################################################################
#   NETFP Core Library Makefile
#
#  NOTE:
#      (C) Copyright 2015 Texas Instruments, Inc.
###################################################################################

.PHONY: netfpLib netfpLibClean

# [NETFP Library]: Sources
NETFP_LIB_SOURCES = src/netfp.c src/netfp_client.c src/netfp_srv.c src/netfp_iface.c src/netfp_ipv6.c	\
					src/netfp_socket.c src/netfp_pa.c src/netfp_reassembly.c src/netfp_listlib.c        \
                    src/netfp_mgmt.c src/netfp_sa.c src/netfp_lte.c src/netfp_ipsec.c                   \
                    src/netfp_salldesp.c src/netfp_trafficMgmt.c src/netfp_fastPath.c                   \
                    src/netfp_ethRuleMgmt.c src/netfp_proxyServer.c src/netfp_multicast.c

# [NETFP Library]: Realm specific sources
NETFP_DSP_SOURCES = src/netfp_dspRealm.c
NETFP_ARM_SOURCES = src/netfp_armRealm.c

# [NETFP Library]: ARM Library Name
NETFP_LIB = lib/libnetfp_$(SYSLIB_DEVICE).a

# ARM NETFP Library will have the ARM Realm specific files.
NETFP_ARM_LIB_OBJECTS += $(NETFP_LIB_SOURCES:.c=.o)
NETFP_ARM_LIB_OBJECTS += $(NETFP_ARM_SOURCES:.c=.o)

# DSP NETFP Library will have the DSP Realm specific files.
NETFP_DSP_LIB_OBJECTS += $(NETFP_LIB_SOURCES:.c=.obj)
NETFP_DSP_LIB_OBJECTS += $(NETFP_DSP_SOURCES:.c=.obj)

# Dependency files
NETFP_DEPENDS  = $(NETFP_LIB_SOURCES:.c=.d)
NETFP_DEPENDS += $(NETFP_ARM_SOURCES:.c=.d)
NETFP_DEPENDS += $(NETFP_DSP_SOURCES:.c=.d)

# [NETFP Library]: Build the ARM Library
netfpLib:$(NETFP_ARM_LIB_OBJECTS)
	mkdir -p lib
	echo "Archiving $@"
	$(ARM_AR) $(ARM_AR_OPTS) $(NETFP_LIB) $(NETFP_ARM_LIB_OBJECTS)

# [NETFP Library]: Clean the ARM Library
netfpLibClean:
	@echo 'Cleaning the ARM NETFP Library Objects'
	@rm -f $(NETFP_ARM_LIB_OBJECTS) $(NETFP_LIB) $(NETFP_DEPENDS)

# [NETFP Library]: Dependency handling
-include $(NETFP_LIB_SOURCES:.c=.d)


