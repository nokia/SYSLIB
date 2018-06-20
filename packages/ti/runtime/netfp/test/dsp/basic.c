/*
 *   @file  basic.c
 *
 *   @brief
 *      Unit Test code for the basic API testing.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2014 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/HeapMemMP.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/**********************************************************************
 ************************ Local Defintions ****************************
 **********************************************************************/

/* Maximum number of rekey iterations being tested */
#define TEST_MAX_REKEY_ITERATIONS           30

/* For testing, local definition without using internal headers */
#define NETFP_VLAN_PRIORITYTAG_DEFAULT      1

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

Netfp_Reason    socketNotifyReason;

/* IPSec encryption key */
static uint8_t encKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
/* IPSec authentication key */
static uint8_t authKey[32] =
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
         0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;
extern Pktlib_HeapHandle    mtuReceiveHeap;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle     appPktlibInstanceHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Notify function which is registered with the socket to be invoked
 *      when there is a configuration change in the NETFP subsystem
 *
 *  @param[in]  reason
 *      Reason code because of which the socket was notified.
 *  @param[in]  sockHandle
 *      Socket Handle which was affected
 *
 *  @retval
 *      Not applicable
 */
static void Test_notifySocketFunction(Netfp_Reason reason, Netfp_SockHandle sockHandle)
{
    printf ("Debug: Notify function invoked for socket %p [Reason: '%s']\n", sockHandle, Netfp_getReasonString(reason));
    socketNotifyReason = reason;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the NETFP Interface module API.
 *      The function tests the following API:
 *          - Interface Creation/Deletion
 *          - Add/Delete IP Address
 *
 *      The contents of the routing table & interface are dumped
 *      on the console.
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_netfpInterface(void)
{
    Netfp_InterfaceCfg  ifConfig;
    Netfp_InterfaceCfg  ifConfig1;
    Netfp_IfHandle      ifHandle;
    Netfp_InterfaceIP   globalIP[NETFP_MAX_IP_ADDRESS];
    Netfp_InterfaceIP   globalIP1[NETFP_MAX_IP_ADDRESS];
    Netfp_OptionTLV     optCfg;
    uint32_t            mtu;
    int32_t             index;
    int32_t             errCode;

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("NETFP Interface Testing:\n");

    /* Initialize the interface configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = 192;
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = 168;
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = 100;
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = 1;
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth1.100");

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = 0x00;
    ifConfig.macAddress[1] = 0x01;
    ifConfig.macAddress[2] = 0x02;
    ifConfig.macAddress[3] = 0x03;
    ifConfig.macAddress[4] = 0x04;
    ifConfig.macAddress[5] = 0x05;

    /* Create & Register the Interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        printf ("Error: Unable to create the NETFP interface [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Interface '%s' is created with Handle %p\n", ifConfig.name, ifHandle);

    /* Find the interface and get the interface configuration. */
    if (Netfp_findInterface (netfpClientHandle, ifConfig.name, &ifConfig1, &errCode) != ifHandle)
    {
        printf ("Error: Unable to find the interface\n");
        return -1;
    }

    /* Validate and ensure that the interface configurations are the same */
    if (memcmp((void *)&ifConfig, (void *)&ifConfig1, sizeof(Netfp_InterfaceCfg)) != 0)
    {
        printf ("Error: Interface configurations do NOT match\n");
        return -1;
    }

    /* Add an IPv4 Address.  */
    globalIP[0].ipAddress.ver  = Netfp_IPVersion_IPV4;
    globalIP[0].subnetMask.ver = Netfp_IPVersion_IPV4;

    /* Use the Network IP address from the configuration. */
    globalIP[0].ipAddress.addr.ipv4.u.a8[0]  = 192;
    globalIP[0].ipAddress.addr.ipv6.u.a8[1]  = 168;
    globalIP[0].ipAddress.addr.ipv6.u.a8[2]  = 200;
    globalIP[0].ipAddress.addr.ipv6.u.a8[3]  = 1;
    globalIP[0].subnetMask.addr.ipv4.u.a8[0] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[1] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[2] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[3] = 0x0;

    /* Add multiple IPv4 address: */
    optCfg.type   = Netfp_Option_ADD_IP;
    optCfg.length = sizeof(Netfp_InterfaceIP);
    optCfg.value  = (void*)&globalIP[0];
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to add the second IPv4 address [Error code %d]\n", errCode);
        return -1;
    }

    /***************************************************************************
     * Add Multiple IP addresses to the interface
     *  - Global IPv6 Address
     ***************************************************************************/

    /* Cycle through and add all the IPv6 addresses to the specific interface */
    for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
    {
        /* YES. */
        globalIP[index].ipAddress.ver  = Netfp_IPVersion_IPV6;
        globalIP[index].subnetMask.ver = Netfp_IPVersion_IPV6;

        /* Use the Network IP address from the configuration. */
        globalIP[index].ipAddress.addr.ipv6.u.a8[0] = (index + 1);
        globalIP[index].ipAddress.addr.ipv6.u.a8[1] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[2] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[3] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[4] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[5] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[6] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[7] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[8] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[9] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[10] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[11] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[12] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[13] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[14] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[15] = (index + 1);

        /* Subnet Mask is assumed to be 64 bits */
        globalIP[index].subnetMask.addr.ipv6.u.a32[0] = 0xFFFFFFFF;
        globalIP[index].subnetMask.addr.ipv6.u.a32[1] = 0xFFFFFFFF;
        globalIP[index].subnetMask.addr.ipv6.u.a32[2] = 0x0;
        globalIP[index].subnetMask.addr.ipv6.u.a32[3] = 0x0;

        /* Add the IPv6 address to the interface. */
        optCfg.type     =   Netfp_Option_ADD_IP;
        optCfg.length   =   sizeof(Netfp_InterfaceIP);
        optCfg.value    =   (void*)&globalIP[index];

        if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
        {
            printf ("Error: Unable to add the global IPv6 address [Error code %d]\n", errCode);
            return -1;
        }
    }

    /* Get all the IPv6 address which have been configured on the system
     * NOTE: The 'length' field in the option configuration should be set to
     * handle the MAX IPv6 address which can be configured on the interface */
    optCfg.type     =   Netfp_Option_GET_IPv6;
    optCfg.length   =   sizeof(globalIP1);
    optCfg.value    =   (void*)&globalIP1[0];
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to get the IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* We now need to validate the IPv6 address configured vs. retreived configuration */
    if (memcmp ((void *)&globalIP, (void *)&globalIP1, optCfg.length) != 0)
    {
        printf ("Error: IPv6 address configuration retreival failed\n");
        return -1;
    }

    /***************************************************************************
     * Delete IP addresses from the interface
     *  - Delete the last global IP address which was added.
    ***************************************************************************/
    optCfg.type     =   Netfp_Option_DEL_IP;
    optCfg.length   =   sizeof(Netfp_InterfaceIP);
    optCfg.value    =   (void*)&globalIP[index - 1];
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Modify the MTU of the interface */
    mtu             = 1400;
    optCfg.type     = Netfp_Option_MTU;
    optCfg.length   = 4;
    optCfg.value    = &mtu;
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Modify the value of mtu. We want to ensure that we get the value from the NETFP Server */
    mtu = 0xFFFF;

    /* Get the MTU of the interface and verify if this is the same as what we modified it above */
    optCfg.type     = Netfp_Option_MTU;
    optCfg.length   = 4;
    optCfg.value    = &mtu;
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Validate the received mtu: */
    if (mtu != 1400)
    {
        printf ("Error: Unable to configure the MTU [%d] of the interface\n", *(int32_t *)optCfg.value);
        return -1;
    }
    printf ("Debug: Interface MTU Get/Set Test passed\n");

    /* Now we delete the interface. This should clean up the entire network routes also. */
    if (Netfp_deleteInterface(netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Interface Deletion Failed with error code %d\n", errCode);
        return -1;
    }

    /* Test passed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the NETFP VLAN Interface module API.
 *      The function tests the following API:
 *          - Interface Creation/Deletion
 *          - Add/Delete IP Address
 *
 *      The contents of the routing table & interface are dumped
 *      on the console.
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_netfpVLANInterface(void)
{
    Netfp_InterfaceCfg      ifConfig;
    Netfp_InterfaceCfg      ifConfig1;
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceIP       globalIP[NETFP_MAX_IP_ADDRESS];
    Netfp_InterfaceIP       globalIP1[NETFP_MAX_IP_ADDRESS];
    Netfp_OptionTLV         optCfg;
    uint32_t                mtu;
    int32_t                 index;
    Netfp_VLANPriorityMap   priorityMap;
    int32_t                 errCode;

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("NETFP VLAN Interface Testing:\n");

    /* Initialize the interface configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_VLAN;
    ifConfig.mtu                           = 1500;
    ifConfig.vlanId                        = 200;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = 192;
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = 168;
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = 100;
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = 1;
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth1.200");

    /* Setup the VLAN priority mapping: By default all NETFP Socket priority are marked as 0 */
    for (index = 0; index < NETFP_MAX_SOCK_PRIORITY; index++)
        ifConfig.vlanMap.socketToVLANPriority[index] = 0;

    /* Explicity remap socket priority 2 to VLAN priority 7 */
    ifConfig.vlanMap.socketToVLANPriority[2] = 7;

    /* Explicity remap socket priority 5 to VLAN priority 1 */
    ifConfig.vlanMap.socketToVLANPriority[5] = 1;

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = 0x00;
    ifConfig.macAddress[1] = 0x01;
    ifConfig.macAddress[2] = 0x02;
    ifConfig.macAddress[3] = 0x03;
    ifConfig.macAddress[4] = 0x04;
    ifConfig.macAddress[5] = 0x05;

    /* Create & Register the Interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        printf ("Error: Unable to create the NETFP interface [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Interface '%s' is created with Handle %p\n", ifConfig.name, ifHandle);

    /* Find the interface and get the interface configuration. */
    if (Netfp_findInterface (netfpClientHandle, ifConfig.name, &ifConfig1, &errCode) != ifHandle)
    {
        printf ("Error: Unable to find the interface\n");
        return -1;
    }

    /* Validate and ensure that the interface configurations are the same */
    if (memcmp((void *)&ifConfig, (void *)&ifConfig1, sizeof(Netfp_InterfaceCfg)) != 0)
    {
        printf ("Error: Interface configurations do NOT match\n");
        return -1;
    }

    /* Modify the VLAN Mapping: */
    priorityMap.socketToVLANPriority[0] = 7;

    /* Populate the TLV: */
    optCfg.type   = Netfp_Option_VLAN_EGRESS_PRIORITY;
    optCfg.length = sizeof(Netfp_VLANPriorityMap);
    optCfg.value  = &priorityMap;

    /* Modify the mapping: */
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Modifying the VLAN Mapping failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Modifying the VLAN Mapping was successful\n");

    /* Reset the VLAN Priority mapping: */
    memset ((void *)&priorityMap, 0, sizeof(Netfp_VLANPriorityMap));

    /* Get the VLAN Mapping: */
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Getting the VLAN Mapping failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Verify if the VLAN Mapping configuration was consistent: */
    if (priorityMap.socketToVLANPriority[0] != 7)
    {
        printf ("Error: Getting the VLAN Mapping was incorrect [Expected %d Got %d]\n", 7, priorityMap.socketToVLANPriority[0]);
        return -1;
    }
    printf ("Debug: Getting the VLAN Mapping was successful\n");

    /* Add an IPv4 Address.  */
    globalIP[0].ipAddress.ver  = Netfp_IPVersion_IPV4;
    globalIP[0].subnetMask.ver = Netfp_IPVersion_IPV4;

    /* Use the Network IP address from the configuration. */
    globalIP[0].ipAddress.addr.ipv4.u.a8[0]  = 192;
    globalIP[0].ipAddress.addr.ipv6.u.a8[1]  = 168;
    globalIP[0].ipAddress.addr.ipv6.u.a8[2]  = 200;
    globalIP[0].ipAddress.addr.ipv6.u.a8[3]  = 1;
    globalIP[0].subnetMask.addr.ipv4.u.a8[0] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[1] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[2] = 255;
    globalIP[0].subnetMask.addr.ipv4.u.a8[3] = 0x0;

    /* Add multiple IPv4 address:  */
    optCfg.type   = Netfp_Option_ADD_IP;
    optCfg.length = sizeof(Netfp_InterfaceIP);
    optCfg.value  = (void*)&globalIP[0];
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to add multiple IPv4 addresses [Error code %d]\n", errCode);
        return -1;
    }

    /***************************************************************************
     * Add Multiple IP addresses to the interface
     *  - Global IPv6 Address
     ***************************************************************************/

    /* Cycle through and add all the IPv6 addresses to the specific interface */
    for (index = 0; index < NETFP_MAX_IP_ADDRESS; index++)
    {
        /* YES. */
        globalIP[index].ipAddress.ver  = Netfp_IPVersion_IPV6;
        globalIP[index].subnetMask.ver = Netfp_IPVersion_IPV6;

        /* Use the Network IP address from the configuration. */
        globalIP[index].ipAddress.addr.ipv6.u.a8[0] = (index + 1);
        globalIP[index].ipAddress.addr.ipv6.u.a8[1] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[2] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[3] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[4] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[5] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[6] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[7] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[8] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[9] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[10] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[11] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[12] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[13] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[14] = 0;
        globalIP[index].ipAddress.addr.ipv6.u.a8[15] = (index + 1);

        /* Subnet Mask is assumed to be 64 bits */
        globalIP[index].subnetMask.addr.ipv6.u.a32[0] = 0xFFFFFFFF;
        globalIP[index].subnetMask.addr.ipv6.u.a32[1] = 0xFFFFFFFF;
        globalIP[index].subnetMask.addr.ipv6.u.a32[2] = 0x0;
        globalIP[index].subnetMask.addr.ipv6.u.a32[3] = 0x0;

        /* Add the IPv6 address to the interface. */
        optCfg.type     =   Netfp_Option_ADD_IP;
        optCfg.length   =   sizeof(Netfp_InterfaceIP);
        optCfg.value    =   (void*)&globalIP[index];

        if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
        {
            printf ("Error: Unable to add the global IPv6 address [Error code %d]\n", errCode);
            return -1;
        }
    }

    /* Get all the IPv6 address which have been configured on the system
     * NOTE: The 'length' field in the option configuration should be set to
     * handle the MAX IPv6 address which can be configured on the interface */
    optCfg.type     =   Netfp_Option_GET_IPv6;
    optCfg.length   =   sizeof(globalIP1);
    optCfg.value    =   (void*)&globalIP1[0];
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to get the IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* We now need to validate the IPv6 address configured vs. retreived configuration */
    if (memcmp ((void *)&globalIP, (void *)&globalIP1, optCfg.length) != 0)
    {
        printf ("Error: IPv6 address configuration retreival failed\n");
        return -1;
    }

    /***************************************************************************
     * Delete IP addresses from the interface
     *  - Delete the last global IP address which was added.
    ***************************************************************************/
    optCfg.type     =   Netfp_Option_DEL_IP;
    optCfg.length   =   sizeof(Netfp_InterfaceIP);
    optCfg.value    =   (void*)&globalIP[index - 1];
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Modify the MTU of the interface */
    mtu             = 1400;
    optCfg.type     = Netfp_Option_MTU;
    optCfg.length   = 4;
    optCfg.value    = &mtu;
    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Modify the value of mtu. We want to ensure that we get the value from the NETFP Server */
    mtu = 0xFFFF;

    /* Get the MTU of the interface and verify if this is the same as what we modified it above */
    optCfg.type     = Netfp_Option_MTU;
    optCfg.length   = 4;
    optCfg.value    = &mtu;
    if (Netfp_getIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        printf ("Error: Unable to delete the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Validate the received mtu: */
    if (mtu != 1400)
    {
        printf ("Error: Unable to configure the MTU [%d] of the interface\n", *(int32_t *)optCfg.value);
        return -1;
    }
    printf ("Debug: Interface MTU Get/Set Test passed\n");

    /* Test passed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an interface.
 *
 *  @param[in]  ifname
 *      Interface Name.
 *
 *  @retval
 *      Success -   Interface Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_IfHandle Test_createInterface(char* ifname)
{
    Netfp_InterfaceCfg  ifConfig;
    Netfp_IfHandle      ifHandle;
    int32_t             errCode;

    /* Initialize the interface configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = 192;
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = 168;
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = 100;
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = 1;
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, ifname);

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = 0x00;
    ifConfig.macAddress[1] = 0x01;
    ifConfig.macAddress[2] = 0x02;
    ifConfig.macAddress[3] = 0x03;
    ifConfig.macAddress[4] = 0x04;
    ifConfig.macAddress[5] = 0x05;

    /* Create & register the interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
        printf ("Error: Unable to create the NETFP interface [Error code %d]\n", errCode);

    /* Return the interface handle. */
    return ifHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a fast path
 *
 *  @param[in]  fpName
 *      Name of the fast path which is to be created
 *  @param[in]  spId
 *      Security Policy Identifier
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_InboundFPHandle Test_createInboundFastPath
(
    char*       fpName,
    uint32_t    spId,
    int32_t*    errCode
)
{
    Netfp_InboundFPCfg  inboundFPCfg;

    /* Initialize the fast path configuration */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Initialize spidMode */
    if(spId == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = (uint32_t)spId;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 200;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (inboundFPCfg.name, fpName);

    /* Create the inbound fast path */
    return Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, errCode);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a fast path
 *
 *  @param[in]  fpName
 *      Name of the fast path which is to be created
 *  @param[in]  spId
 *      Security Policy Identifier
 *  @param[in]  ifName
 *      Manual routing interface name to be used. Can be NULL for NETFP Proxy
 *  @param[in]  nextHopMACAddress
 *      Manual routing next hop MAC address to be used.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_OutboundFPHandle Test_createOutboundFastPath
(
    char*               fpName,
    uint32_t            spId,
    char*               ifName,
    uint8_t*            nextHopMACAddress,
    int32_t*            errCode
)
{
    Netfp_OutboundFPCfg outboundFPCfg;
    Netfp_IfHandle      ifHandle;

    /* Initialize the fast path configuration */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = (uint32_t)spId;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 200;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 100;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (outboundFPCfg.name, fpName);

    /* Was an interface name specified? */
    if (ifName != NULL)
    {
        ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, errCode);
        if (ifHandle == NULL)
        {
            printf ("Error: Unable to find the interface '%s' [Error code %d]\n", ifName, *errCode);
            return NULL;
        }

        /* Populate the manual routing section: */
        outboundFPCfg.ifHandle = ifHandle;
        memcpy ((void *)&outboundFPCfg.nextHopMACAddress[0], (void *)nextHopMACAddress, 6);
    }

    /* Create the outbound fast path */
    return Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, errCode);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a security association
 *
 *  @param[in]  spi
 *      SPI identifier
 *  @param[in]  direction
 *      Direction of the IPSEC Channel
 *  @param[in]  ifHandle
 *      Interface handle; valid only for outbound SA
 *  @param[in]  nextHopMACAddress
 *      Next HOP MAC Address; valid only for outbound SA
 *
 *  @retval
 *      Success -   SA Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SAHandle Test_createSA
(
    uint32_t            spi,
    Netfp_Direction     direction,
    Netfp_IfHandle      ifHandle,
    uint8_t*            nextHopMACAddress
)
{
    Netfp_SACfg         saConfig;
    Netfp_SAHandle      saHandle;
    int32_t             errCode;

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = direction;
    saConfig.spi                    = spi;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
    saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else
        saConfig.ipsecCfg.keyAuthSize   = 0;

    if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
        saConfig.ipsecCfg.keyEncSize = 0;
    else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
        saConfig.ipsecCfg.keyEncSize = 20;
    else
        saConfig.ipsecCfg.keyEncSize = 24;

    /* Copy the keys. */
    memcpy (&saConfig.ipsecCfg.keyAuth, &authKey[0], saConfig.ipsecCfg.keyAuthSize);
    memcpy (&saConfig.ipsecCfg.keyEnc,  &encKey[0],  saConfig.ipsecCfg.keyEncSize);

    /* Configure the lifetime */
    saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

    /* Setup the Source & Destination IP Address for the SA. */
    if (direction == Netfp_Direction_INBOUND)
    {
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 200;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 100;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }
    else
    {
        /* Setup the Source & Destination IP Address for the SA. */
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 100;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 200;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }

    /* Populate the next hop MAC address and interface handle. */
    if (ifHandle != NULL)
    {
        saConfig.ifHandle = ifHandle;
        memcpy ((void*)&saConfig.nextHopMACAddress[0], (void*)nextHopMACAddress, 6);
    }

    /* Create the SA. */
    saHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (saHandle == NULL)
        printf ("Error: Unable to create SA %d\n", errCode);
    return saHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a security policy
 *
 *  @param[in]  saHandle
 *      SA Handle associated with the security policy
 *  @param[in]  direction
 *      Direction of the security policy
 *  @param[in]  spId
 *      Security Policy identifier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_createSP
(
    Netfp_SAHandle      saHandle,
    Netfp_Direction     direction,
    uint32_t            spId
)
{
    Netfp_SPCfg     spConfig;
    int32_t         errCode;

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = direction;
    spConfig.saHandle                = saHandle;

    if (direction == Netfp_Direction_INBOUND)
    {
        spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        spConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        spConfig.srcIP.addr.ipv4.u.a8[2] = 200;
        spConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        spConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        spConfig.dstIP.addr.ipv4.u.a8[2] = 100;
        spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }
    else
    {
        spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        spConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        spConfig.srcIP.addr.ipv4.u.a8[2] = 100;
        spConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        spConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        spConfig.dstIP.addr.ipv4.u.a8[2] = 200;
        spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }

    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spId;

    /* Add the Inbound Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        printf ("Error: Unable to create Inbound SP Error %d\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a security policy with
 *  wild carding address.
 *
 *  @param[in]  saHandle
 *      SA Handle associated with the security policy
 *  @param[in]  direction
 *      Direction of the security policy
 *  @param[in]  spId
 *      Security Policy identifier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_createWcSP
(
    Netfp_SAHandle      saHandle,
    Netfp_Direction     direction,
    uint32_t            spId
)
{
    Netfp_SPCfg     spConfig;
    int32_t         errCode;

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = direction;
    spConfig.saHandle                = saHandle;

    if (direction == Netfp_Direction_INBOUND)
    {
        spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.srcIP.addr.ipv4.u.a8[0] = 0;
        spConfig.srcIP.addr.ipv4.u.a8[1] = 0;
        spConfig.srcIP.addr.ipv4.u.a8[2] = 0;
        spConfig.srcIP.addr.ipv4.u.a8[3] = 0;

        spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        spConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        spConfig.dstIP.addr.ipv4.u.a8[2] = 100;
        spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }
    else
    {
        spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        spConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        spConfig.srcIP.addr.ipv4.u.a8[2] = 100;
        spConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
        spConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        spConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        spConfig.dstIP.addr.ipv4.u.a8[2] = 200;
        spConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }

    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spId;

    /* Add the Inbound Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        printf ("Error: Unable to create Inbound SP Error %d\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the NETFP Fast Path API.
 *      The function tests the following API:
 *          - Duplicate Inbound Fast Path creation and deletion
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_fastPath(void)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_InboundFPHandle   fpInboundHandle1;
    Netfp_InboundFPHandle   fpInboundHandle2;
    Netfp_InboundFPHandle   fpInboundHandle3;
    Netfp_SAHandle          inboundSAHandle;
    Netfp_IfHandle          ifHandle;
    int32_t                 errCode;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing the multiple child Inbound Fast Path addition and deletion\n");

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /*************************************************************************************
     * TEST CASE:
     *  - Create inbound fast path with invalid security policy identifer
     *  - The fast path creation should fail.
     *************************************************************************************/
    inboundFPCfg.spId                       = (uint32_t)100;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 200;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (inboundFPCfg.name, "Inbound-invalidSP");

    /* Create the inbound fast path */
    fpInboundHandle1 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle1 != NULL)
    {
        printf ("Error: Inbound Fast Path created with an invalid security policy identifier\n");
        return -1;
    }

    /* Test Passed. */
    printf ("Test 1: Fast Path with invalid security policy identifer passed\n");

    /*************************************************************************************
     * TEST CASE:
     *  - Create 3 similar inbound fast paths
     *  - Delete them in the same order in which they have been created
     *  - Validate and ensure that they have been deleted and no longer exist.
     *************************************************************************************/

    /* Create the Inbound-1 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-1");
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    fpInboundHandle1 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle1 == NULL)
    {
        printf ("Error: Unable to create Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-2 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-2");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle2 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle2 == NULL)
    {
        printf ("Error: Unable to create Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-3 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-3");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle3 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle3 == NULL)
    {
        printf ("Error: Unable to create Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle1, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle2, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle3, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-1", &errCode) != NULL)
    {
        printf ("Error: Inbound-1 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-2", &errCode) != NULL)
    {
        printf ("Error: Inbound-2 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-3", &errCode) != NULL)
    {
        printf ("Error: Inbound-3 Fast Path has not been deleted\n");
        return -1;
    }

    /* Test Passed. */
    printf ("Test 2: Multiple Inbound Fast Path test passed [Deletion Order: FP1, FP2, FP3]\n");

    /*************************************************************************************
     * TEST CASE:
     *  - Create 3 similar inbound fast paths
     *  - Delete them in the reverse order in which they have been created
     *  - Validate and ensure that they have been deleted and no longer exist.
     *************************************************************************************/

    /* Create the Inbound-1 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-1");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle1 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle1 == NULL)
    {
        printf ("Error: Unable to create Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-2 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-2");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle2 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle2 == NULL)
    {
        printf ("Error: Unable to create Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-3 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-3");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle3 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle3 == NULL)
    {
        printf ("Error: Unable to create Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle3, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle2, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle1, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-1", &errCode) != NULL)
    {
        printf ("Error: Inbound-1 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-2", &errCode) != NULL)
    {
        printf ("Error: Inbound-2 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-3", &errCode) != NULL)
    {
        printf ("Error: Inbound-3 Fast Path has not been deleted\n");
        return -1;
    }

    /* Test Passed. */
    printf ("Test 3: Multiple Inbound Fast Path test passed [Deletion Order: FP3, FP2, FP1]\n");

    /*************************************************************************************
     * TEST CASE:
     *  - Create 3 similar inbound fast paths with different spids - Need to verify different
          Fast Path parents are created for each of the Fast Path
     *  - Delete them in the reverse order in which they have been created
     *  - Validate and ensure that they have been deleted and no longer exist.
     *************************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound security association */
    inboundSAHandle = Test_createSA (0xdeaddead, Netfp_Direction_INBOUND, ifHandle, &nextHopMACAddress[0]);

    Test_createSP (inboundSAHandle, Netfp_Direction_INBOUND,  100);
    Test_createSP (inboundSAHandle, Netfp_Direction_INBOUND,  200);

    /* Create the Inbound-1 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-1");
    inboundFPCfg.spId     = 100;
    inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    fpInboundHandle1 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle1 == NULL)
    {
        printf ("Error: Unable to create Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-2 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-2");
    inboundFPCfg.spId     = 200;
    inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    fpInboundHandle2 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle2 == NULL)
    {
        printf ("Error: Unable to create Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-3 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-3");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle3 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle3 == NULL)
    {
        printf ("Error: Unable to create Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle3, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle2, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle1, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-1", &errCode) != NULL)
    {
        printf ("Error: Inbound-1 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-2", &errCode) != NULL)
    {
        printf ("Error: Inbound-2 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-3", &errCode) != NULL)
    {
        printf ("Error: Inbound-3 Fast Path has not been deleted\n");
        return -1;
    }

    /* Cleanup the outbound security policy */
    if (Netfp_delSP(netfpClientHandle, 100, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the outbound security policy */
    if (Netfp_delSP(netfpClientHandle, 200, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the outbound security association */
    if (Netfp_delSA(netfpClientHandle, inboundSAHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }

    /* Test Passed. */
    printf ("Test 4: Multiple Inbound Fast Path with different SPs test passed [Deletion Order: FP3, FP2, FP1]\n");

    /*************************************************************************************
     * TEST CASE:
     *  - Create 3 similar inbound fast paths with different spids - Need to verify different
          Fast Path parents are created for each of the Fast Path
     *  - Delete them in the reverse order in which they have been created
     *  - Validate and ensure that they have been deleted and no longer exist.
     *************************************************************************************/

    inboundFPCfg.spId                       = (uint32_t)100;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 0;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 100;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (inboundFPCfg.name, "Inbound-invalidFP");

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound security association */
    inboundSAHandle = Test_createSA (0xdeaddead, Netfp_Direction_INBOUND, ifHandle, &nextHopMACAddress[0]);

    Test_createWcSP (inboundSAHandle, Netfp_Direction_INBOUND,  100);

    /* Create the Inbound-1 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-wc1");
    inboundFPCfg.spId     = 100;
    inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    fpInboundHandle1 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle1 == NULL)
    {
        printf ("Error: Unable to create Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-2 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-wc2");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    fpInboundHandle2 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle2 == NULL)
    {
        printf ("Error: Unable to create Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Create the Inbound-3 Fast Path */
    strcpy (inboundFPCfg.name, "Inbound-wc3");
    inboundFPCfg.spId     = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
    fpInboundHandle3 = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle3 == NULL)
    {
        printf ("Error: Unable to create Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle3, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-3 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle2, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-2 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Delete the fast paths */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpInboundHandle1, &errCode) < 0)
    {
        printf ("Error: Unable to delete Inbound-1 Fast Path Error %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-wc1", &errCode) != NULL)
    {
        printf ("Error: Inbound-1 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-wc2", &errCode) != NULL)
    {
        printf ("Error: Inbound-2 Fast Path has not been deleted\n");
        return -1;
    }

    /* Validate & ensure that the fast paths have all been deleted */
    if (Netfp_findInboundFastPath(netfpClientHandle, "Inbound-wc3", &errCode) != NULL)
    {
        printf ("Error: Inbound-3 Fast Path has not been deleted\n");
        return -1;
    }

    /* Cleanup the outbound security policy */
    if (Netfp_delSP(netfpClientHandle, 100, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the outbound security association */
    if (Netfp_delSA(netfpClientHandle, inboundSAHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }

    /* Test Passed. */
    printf ("Test 5: Inbound Wild Carding Fast Path test passed [Deletion Order: FP3, FP2, FP1]\n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a socket
 *
 *  @param[in]  inboundFPHandle
 *      Inbound Fast Path Handle
 *  @param[in]  outboundFPHandle
 *      Outbound Fast Path Handle
 *  @param[in]  flowId
 *      Flow Identifier
 *
 *  @retval
 *      Success -   Socket Handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SockHandle Test_createSocket
(
    Netfp_InboundFPHandle   inboundFPHandle,
    Netfp_OutboundFPHandle  outboundFPHandle,
    int32_t                 flowId
)
{
    Netfp_SockHandle    sockHandle;
    int32_t             errCode;
    Netfp_SockAddr      sockAddress;

    /* Open a socket. */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
    {
        printf ("Error: NETFP Socket Creation Failed with Error Code %d\n", errCode);
        return NULL;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 1024;
    sockAddress.op.bind.inboundFPHandle = inboundFPHandle;
    sockAddress.op.bind.flowId          = flowId;
    sockAddress.op.bind.appInfo         = 0x1230;
    sockAddress.op.bind.queueHandle     = 2000;
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed with Error Code %d\n", errCode);
        return NULL;
    }

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1024;
    sockAddress.op.connect.outboundFPHandle = outboundFPHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed with Error Code %d\n", errCode);
        return NULL;
    }

    /* Return the socket. */
    return sockHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE user
 *
 *  @param[in]  ueId
 *      UE identifier
 *  @param[in]  rxFlowId
 *      Receive Flow identifier to be configured.
 *
 *  @retval
 *      Success -   User handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_UserHandle Test_createUser
(
    uint8_t     ueId,
    int32_t     rxFlowId
)
{
    Netfp_UserCfg       userCfg;
    uint8_t             encryptionKey[16];
    uint8_t             integrityKey[16];
    Netfp_UserHandle    ueHandle;
    uint32_t            index;
    int32_t             errCode;

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Initialize the integrity and ciphering keys */
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (TSCL & 0xFF);
        integrityKey[index]  = (TSCL & 0xFF);
    }

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Populate the user security configuration. */
    userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
    userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.ueId             = ueId;
    userCfg.srbFlowId        = rxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb1Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    if (ueHandle == NULL)
        printf ("Error: LTE Creation User %d failed [Error code %d]\n", ueId, errCode);

    /* Return the user handle. */
    return ueHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE channel
 *
 *  @param[in]  ueHandle
 *      User handle on which the channel is created
 *  @param[in]  rbId
 *      Radio Bearer identifier
 *  @param[in]  fpInboundV4Handle
 *      Inbound handle
 *  @param[in]  fpOutboundV4Handle
 *      Outbound handle
 *  @param[in]  gtpuChannelHandle
 *      Handle to which non fast path packets will be placed after reception
 *  @param[in]  encodeChannel
 *      Packets where encoded packets are placed
 *  @param[in]  decodeChannel
 *      Packets where decoded packets are placed
 *  @param[in]  rxFlowId
 *      Flow identifier to be used
 *  @param[in]  gtpuIdentifier
 *      GTPU identifier
 *
 *  @retval
 *      Success -   Socket handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SockHandle Test_createLTEChannel
(
    Netfp_UserHandle        ueHandle,
    uint8_t                 rbId,
    Netfp_InboundFPHandle   fpInboundV4Handle,
    Netfp_OutboundFPHandle  fpOutboundV4Handle,
    Qmss_QueueHnd           gtpuChannelHandle,
    Qmss_QueueHnd           encodeChannel,
    Qmss_QueueHnd           decodeChannel,
    int32_t                 rxFlowId,
    uint32_t                gtpuIdentifier
)
{
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    Netfp_SockHandle            lteDRBChannel;
    int32_t                     errCode;

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = gtpuChannelHandle;
    lteChannelBindCfg.fpHandle        = fpInboundV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = 0;
    lteChannelBindCfg.isHOInProgress  = 0;
    lteChannelBindCfg.chDrbEnc        = encodeChannel;

    /* Populate the channel connect configuration: */
    lteChannelConnectCfg.fpHandle       = fpOutboundV4Handle;
    lteChannelConnectCfg.sin_gtpuId     = gtpuIdentifier;
    lteChannelConnectCfg.qci            = 3;
    lteChannelConnectCfg.dscp           = 0x22;
    lteChannelConnectCfg.flowId         = rxFlowId;
    lteChannelConnectCfg.chDrbDec       = decodeChannel;

    /* Create the DRB LTE channel: */
    lteDRBChannel = Netfp_createLTEChannel (ueHandle, rbId, Netfp_SockFamily_AF_INET,
                                            &lteChannelBindCfg, &lteChannelConnectCfg, &errCode);
    if (lteDRBChannel == NULL)
        printf ("Error: Failed to create the LTE channel for %d [Error code %d]\n", rbId, errCode);

    return lteDRBChannel;
}

/**
 *  @b Description
 *  @n
 *      The function creates the following NETFP Building Blocks
 *          - Interface
 *          - Inbound Fast Path
 *          - Outbound Fast Path
 *          - Socket
 *
 *       The function then deletes the "Inbound" fast path
 *
 *      Expected Result:
 *      The following modules are deleted:
 *              - Inbound Fast Path
 *              - Socket is notified to be not active
 *      The following modules are untouched:
 *              - Interface
 *              - Outbound Fast Path
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_inboundFPDeletion(int32_t flowId)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_SockHandle        socketHandle;
    int32_t                 errCode;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing the Netfp_deleteInboundFastPath API with event propogation \n");

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", NETFP_INVALID_SPID, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", NETFP_INVALID_SPID, "eth0.100", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that we can find the outbound fast path */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != fpOutboundHandle)
    {
        printf ("Error: Find outbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that we can find the inbound fast path */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != fpInboundHandle)
    {
        printf ("Error: Find inbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the Test Socket: This resides over the fast paths which have been created */
    socketHandle = Test_createSocket(fpInboundHandle, fpOutboundHandle, flowId);
    if (socketHandle == NULL)
        return -1;

    /* Sanity Check: Ensure that the socket is operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 1)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /* Reset the global socket reason: */
    socketNotifyReason = Netfp_Reason_UNKNOWN;

    /**********************************************************************************
     * Event Generation: Delete the inbound interface
     *********************************************************************************/
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpInboundHandle, &errCode) < 0)
    {
        printf ("Error: Inbound fast path deletion failed [Error code %d]\n", errCode);
        return -1;
    }

    /**********************************************************************************
     * Validations:
     *********************************************************************************/

    /* Sanity Check: The Outbound Fast path should be valid. */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != fpOutboundHandle)
    {
        printf ("Error: Outbound Fast Path was deleted by the inbound FP deletion event\n");
        return -1;
    }

    /* Sanity Check: The Inbound Fast path should no longer remain */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != NULL)
    {
        printf ("Error: Inbound Fast Path was not deleted\n");
        return -1;
    }

    /* Sanity Check: Ensure that the interface has NOT been deleted by the event. */
    if (Netfp_findInterface (netfpClientHandle, "eth0.100", NULL, &errCode) == NULL)
    {
        printf ("Error: Interface has been deleted by the inbound fast path deletion\n");
        return -1;
    }

    /* Sanity Check: The Test Socket should have been notified about the event. Sockets are not
     * notified immediately. In the test code the Test Task and the NETFP Client execution tasks
     * have the same priority so we yield the Test Task here to allow time for the NETFP client
     * tasks to execute which will allow the notification on the socket to come through. */
    Task_sleep(10);

    /* The socket should have been marked as NON active because of the delete FP event. */
    if (socketNotifyReason != Netfp_Reason_FAST_PATH_DELETE)
    {
        printf ("Error: Invalid socket reason detected [Reason: %d]\n", socketNotifyReason);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is no longer operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 0)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /**********************************************************************************
     * Cleanup: The cleanup is done in the correct order
     *********************************************************************************/

    /* Clean up the test socket. */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
    {
        printf ("Error: Closing Test socket failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the outbound fast path */
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpOutboundHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function creates the following NETFP Building Blocks
 *          - Interface
 *          - Inbound Fast Path
 *          - Outbound Fast Path
 *          - Socket
 *
 *       The function then deletes the "Outbound" fast path
 *
 *      Expected Result:
 *      The following modules are deleted:
 *              - Outbound Fast Path
 *              - Socket is notified to be not active
 *      The following modules are untouched:
 *              - Interface
 *              - Inbound Fast Path
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_outboundFPDeletion(int32_t flowId)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_SockHandle        socketHandle;
    int32_t                 errCode;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing the Netfp_deleteOutboundFastPath API with event propogation \n");

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", NETFP_INVALID_SPID, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", NETFP_INVALID_SPID, "eth0.100", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that we can find the outbound fast path */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != fpOutboundHandle)
    {
        printf ("Error: Find outbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Sanity Check: Ensure that we can find the inbound fast path */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != fpInboundHandle)
    {
        printf ("Error: Find inbound fast path failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the Test Socket: This resides over the fast paths which have been created */
    socketHandle = Test_createSocket(fpInboundHandle, fpOutboundHandle, flowId);
    if (socketHandle == NULL)
        return -1;

    /* Sanity Check: Ensure that the socket is operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 1)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /* Reset the global socket reason: */
    socketNotifyReason = Netfp_Reason_UNKNOWN;

    /**********************************************************************************
     * Event Generation: Delete the outbound interface
     *********************************************************************************/
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpOutboundHandle, &errCode) < 0)
    {
        printf ("Error: Outbound fast path deletion failed [Error code %d]\n", errCode);
        return -1;
    }

    /**********************************************************************************
     * Validations:
     *********************************************************************************/

    /* Sanity Check: The Inbound Fast path should be valid. */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != fpInboundHandle)
    {
        printf ("Error: Inbound Fast Path was deleted by the outbound FP deletion event\n");
        return -1;
    }

    /* Sanity Check: The Outbound Fast path should no longer remain */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != NULL)
    {
        printf ("Error: Outbound Fast Path was not deleted\n");
        return -1;
    }

    /* Sanity Check: Ensure that the interface has NOT been deleted by the event. */
    if (Netfp_findInterface (netfpClientHandle, "eth0.100", NULL, &errCode) == NULL)
    {
        printf ("Error: Interface has been deleted by the outbound fast path deletion\n");
        return -1;
    }

    /* Sanity Check: The Test Socket should have been notified about the event. Sockets are not
     * notified immediately. In the test code the Test Task and the NETFP Client execution tasks
     * have the same priority so we yield the Test Task here to allow time for the NETFP client
     * tasks to execute which will allow the notification on the socket to come through. */
    Task_sleep(10);

    /* The socket should have been marked as NON active because of the delete FP event. */
    if (socketNotifyReason != Netfp_Reason_FAST_PATH_DELETE)
    {
        printf ("Error: Invalid socket reason detected [Reason: %d]\n", socketNotifyReason);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is no longer operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 0)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /**********************************************************************************
     * Cleanup: The cleanup is done in the correct order
     *********************************************************************************/

    /* Clean up the test socket. */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
    {
        printf ("Error: Closing Test socket failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the inbound fast path */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpInboundHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function creates the following NETFP Building Blocks
 *          - Interface
 *          - Inbound Fast Path
 *          - Outbound Fast Path
 *          - Inbound SA
 *          - Outbound SA
 *          - Inbound SP
 *          - Outbound SP
 *          - Socket
 *
 *       The function then deletes the "Inbound SA"
 *
 *      Expected Result:
 *      The following modules are deleted:
 *              - Inbound SA
 *              - Inbound SP
 *              - Inbound FP
 *              - Socket
 *      The following modules are untouched:
 *              - Outbound Fast Path
 *              - Outbound SP
 *              - Outbound SA
 *              - Interface
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_saDeletion(int32_t flowId)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_SAHandle          inboundSAHandle;
    Netfp_SAHandle          outboundSAHandle;
    Netfp_SockHandle        socketHandle;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    uint32_t                inboundSPId = 40;
    uint32_t                outboundSPId  = 60;
    int32_t                 errCode;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing the Netfp_delSA API with event propogation \n");

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth1.100");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound & outbound security association */
    inboundSAHandle = Test_createSA (0xdeaddead, Netfp_Direction_INBOUND, NULL, NULL);
    outboundSAHandle  = Test_createSA (0xdeadbeef, Netfp_Direction_OUTBOUND, ifHandle, &nextHopMACAddress[0]);

    /* Create the inbound & outbound security policy */
    Test_createSP (inboundSAHandle, Netfp_Direction_INBOUND,  inboundSPId);
    Test_createSP (outboundSAHandle,  Netfp_Direction_OUTBOUND, outboundSPId);

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", inboundSPId, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path. */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", outboundSPId, "eth1.100", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the Test Socket: This resides over the fast paths which have been created */
    socketHandle = Test_createSocket(fpInboundHandle, fpOutboundHandle, flowId);
    if (socketHandle == NULL)
        return -1;

    /* Sanity Check: Ensure that the socket is operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 1)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /* Reset the global socket reason: */
    socketNotifyReason = Netfp_Reason_UNKNOWN;

    /**********************************************************************************
     * Event Generation: Delete the inbound security association.
     *********************************************************************************/
    if (Netfp_delSA (netfpClientHandle, inboundSAHandle, &errCode) < 0)
    {
        printf ("Error: Security association deletion failed [Error code %d]\n", errCode);
        return -1;
    }

    /**********************************************************************************
     * Validations:
     *********************************************************************************/

    /* Sanity Check: The Inbound Fast path should no longer be valid. */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != NULL)
    {
        printf ("Error: Inbound Fast Path was not deleted by the security association deletion event\n");
        return -1;
    }

    /* Sanity Check: The Outbound Fast path should not be affected by inbound security associaton */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != fpOutboundHandle)
    {
        printf ("Error: Outbound Fast Path was affected by the security association deletion event\n");
        return -1;
    }

    /* Sanity Check: Ensure that the interface has NOT been deleted by the event. */
    if (Netfp_findInterface (netfpClientHandle, "eth1.100", NULL, &errCode) == NULL)
    {
        printf ("Error: Interface has been deleted by the interface deletion event\n");
        return -1;
    }

    /* Sanity Check: The "inbound" security association and policy should have been deleted so if we try to
     * create a fast path again with the inbound security policy it should fail with the appropriate error
     * code. */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", inboundSPId, &errCode);
    if (fpInboundHandle != NULL)
    {
        printf ("Error: Successfully created inbound fast path after the security association delete event\n");
        return -1;
    }
    if (errCode != NETFP_EINVAL)
    {
        printf ("Error: Inbound FP creation failed but invalid error code detected %d\n", errCode);
        return -1;
    }

    /* Sanity Check: The Test Socket should have been notified about the event. Sockets are not
     * notified immediately. In the test code the Test Task and the NETFP Client execution tasks
     * have the same priority so we yield the Test Task here to allow time for the NETFP client
     * tasks to execute which will allow the notification on the socket to come through. */
    Task_sleep(10);

    /* The socket should have been marked as NON active because of the delete SA event. */
    if (socketNotifyReason != Netfp_Reason_SA_DELETE)
    {
        printf ("Error: Invalid socket reason detected [Reason: %d]\n", socketNotifyReason);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is no longer operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 0)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /**********************************************************************************
     * Cleanup: The cleanup is done in the correct order
     *********************************************************************************/

    /* Clean up the test socket. */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
    {
        printf ("Error: Closing Test socket failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the outbound fast path */
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpOutboundHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the outbound security policy */
    if (Netfp_delSP(netfpClientHandle, outboundSPId, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the outbound security association */
    if (Netfp_delSA(netfpClientHandle, outboundSAHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Delete the interface. */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface (Error Code %d)\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function creates the following NETFP Building Blocks
 *          - Interface
 *          - Inbound Fast Path
 *          - Outbound Fast Path
 *          - Inbound SA
 *          - Outbound SA
 *          - Inbound SP
 *          - Outbound SP
 *          - Socket
 *
 *       The function then deletes the Interface
 *
 *      Expected Result:
 *      The following modules are deleted:
 *              - Outbound SA
 *              - Outbound SP
 *              - Outbound FP
 *              - Socket
 *              - Interface
 *      The following modules are untouched:
 *              - Inbound Fast Path
 *              - Inbound SP
 *              - Inbound SA
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_interfaceDeletion(int32_t flowId)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_SAHandle          inboundSAHandle;
    Netfp_SAHandle          outboundSAHandle;
    Netfp_SockHandle        socketHandle;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    uint32_t                inboundSPId = 40;
    uint32_t                outboundSPId  = 60;
    int32_t                 errCode;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing the Netfp_deleteInterface API with event propogation\n");

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth1.6");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound & outbound security association */
    inboundSAHandle = Test_createSA (0xdeaddead, Netfp_Direction_INBOUND, NULL, NULL);
    outboundSAHandle  = Test_createSA (0xdeadbeef, Netfp_Direction_OUTBOUND, ifHandle, &nextHopMACAddress[0]);

    /* Create the inbound & outbound security policy */
    Test_createSP (inboundSAHandle, Netfp_Direction_INBOUND,  inboundSPId);
    Test_createSP (outboundSAHandle,  Netfp_Direction_OUTBOUND, outboundSPId);

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", inboundSPId, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path. */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", outboundSPId, "eth1.6", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the Test Socket: This resides over the fast paths which have been created */
    socketHandle = Test_createSocket(fpInboundHandle, fpOutboundHandle, flowId);
    if (socketHandle == NULL)
        return -1;

    /* Sanity Check: Ensure that the socket is operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 1)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /* Reset the global socket reason: */
    socketNotifyReason = Netfp_Reason_UNKNOWN;

    /**********************************************************************************
     * Event Generation: Delete the interface
     *********************************************************************************/
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Interface deletion failed [Error code %d]\n", errCode);
        return -1;
    }

    /**********************************************************************************
     * Validations:
     *********************************************************************************/

    /* Sanity Check: The Outbound Fast path should no longer be valid. */
    if (Netfp_findOutboundFastPath (netfpClientHandle, "Outbound", &errCode) != NULL)
    {
        printf ("Error: Outbound Fast Path was not deleted by the security association deletion event\n");
        return -1;
    }

    /* Sanity Check: The Inbound Fast path should not be affected by the interface */
    if (Netfp_findInboundFastPath (netfpClientHandle, "Inbound", &errCode) != fpInboundHandle)
    {
        printf ("Error: Inbound Fast Path was affected by the security association deletion event\n");
        return -1;
    }

    /* Sanity Check: Ensure that the interface has been deleted by the event. */
    if (Netfp_findInterface (netfpClientHandle, "eth1.6", NULL, &errCode) != NULL)
    {
        printf ("Error: Interface has not been deleted\n");
        return -1;
    }

    /* Sanity Check: The Test Socket should have been notified about the event. Sockets are not
     * notified immediately. In the test code the Test Task and the NETFP Client execution tasks
     * have the same priority so we yield the Test Task here to allow time for the NETFP client
     * tasks to execute which will allow the notification on the socket to come through. */
    Task_sleep(10);

    /* The socket should have been marked as NON active because of the delete SA event. */
    if (socketNotifyReason != Netfp_Reason_INTERFACE_DELETE)
    {
        printf ("Error: Invalid socket reason detected [Reason: %d]\n", socketNotifyReason);
        return -1;
    }

    /* Sanity Check: Ensure that the socket is no longer operational */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState != 0)
    {
        printf ("Error: Invalid socket state %d detected\n", socketState);
        return -1;
    }

    /**********************************************************************************
     * Cleanup: The cleanup is done in the correct order
     *********************************************************************************/

    /* Clean up the test socket. */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
    {
        printf ("Error: Closing Test socket failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Clean up the inbound fast path */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpInboundHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the inbound security policy */
    if (Netfp_delSP(netfpClientHandle, inboundSPId, &errCode) < 0)
    {
        printf ("Error: Unable to delete the route [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup the inbound security association */
    if (Netfp_delSA(netfpClientHandle, inboundSAHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the inbound SA [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the event management framework
 *      in NETFP
 *
 *  @retval
 *      Not Applicable.
 */
int32_t Test_netfpDeletion(void)
{
    Netfp_FlowCfg       flowCfg;
    int32_t             myFlowHandle;
    int32_t             errCode;

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Testing event propogation with the Netfp_deleteXXX API \n");

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;
    strcpy (flowCfg.name, "TestFlow");

    /* Create a test flow which will be used in the unit tests. */
    myFlowHandle = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (myFlowHandle < 0)
    {
        printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return -1;
    }

    if (Test_inboundFPDeletion(myFlowHandle) < 0)
        return -1;

    if (Test_outboundFPDeletion(myFlowHandle) < 0)
        return -1;

    if (Test_saDeletion(myFlowHandle) < 0)
        return -1;

    if (Test_interfaceDeletion(myFlowHandle) < 0)
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the creation and deletion of multiple
 *      IPSEC channels.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Test_IPSecChannel (void)
{
    uint32_t            index;
    int32_t             errCode;
    uint32_t            spi          = 1000;
    uint32_t            inboundSpId  = 2000;
    uint32_t            outboundSpId = 3000;
    Netfp_SAHandle      inboundSaHandle[64];
    Netfp_SAHandle      outboundSaHandle[64];
    Netfp_IfHandle      ifHandle;
    uint32_t            numIPSECChannels;
    uint8_t             nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

#if (defined (DEVICE_K2H) || defined (K2K))
    numIPSECChannels = 60;
#elif (defined (DEVICE_K2L))
    numIPSECChannels = 30;
#else
#error "Unsupported Device"
#endif

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Creating the INBOUND IPSec Channels & Security Policies for %d channels\n", numIPSECChannels);

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth1.100");
    if (ifHandle == NULL)
        return -1;

    /* Cycle through and create all the IPSEC channels & security policies for the INBOUND direction. */
    for (index = 0; index < numIPSECChannels; index++)
    {
        /* Create the IPSEC Channel */
        inboundSaHandle[index] = Test_createSA (spi++, Netfp_Direction_INBOUND, NULL, NULL);
        if (inboundSaHandle[index] == NULL)
        {
            printf ("Error: Inbound SA handle for index %d failed\n", index);
            return -1;
        }

        /* Create & register the security policy */
        if (Test_createSP (inboundSaHandle[index], Netfp_Direction_INBOUND, inboundSpId + index) < 0)
        {
            printf ("Error: Inbound SP handle for index %d failed\n", index);
            return -1;
        }
    }

    /* Cycle through and create all the IPSEC channels & security policies for the INBOUND direction. */
    printf ("Debug: Creating the OUTBOUND IPSec Channels & Security Policies for %d channels\n", numIPSECChannels);
    for (index = 0; index < numIPSECChannels; index++)
    {
        /* Create the IPSEC Channel */
        outboundSaHandle[index] = Test_createSA (spi++, Netfp_Direction_OUTBOUND, ifHandle, &nextHopMACAddress[0]);
        if (outboundSaHandle[index] == NULL)
        {
            printf ("Error: Outbound SA handle for index %d failed\n", index);
            return -1;
        }

        /* Create & register the security policy */
        if (Test_createSP (outboundSaHandle[index], Netfp_Direction_OUTBOUND, outboundSpId + index) < 0)
        {
            printf ("Error: Outbound SP handle for index %d failed\n", index);
            return -1;
        }
    }

    /* Cycle through and delete all the IPSEC security channels which have been created */
    printf ("Debug: Deleting the security policies & associations\n");
    for (index = 0; index < numIPSECChannels; index++)
    {
        /* Delete the inbound security policy */
        if (Netfp_delSP (netfpClientHandle, inboundSpId + index, &errCode) < 0)
        {
            printf ("Error: Unable to delete inbound security policy %d [Error code %d]\n", inboundSpId + index, errCode);
            return -1;
        }

        /* Delete the outbound security policy */
        if (Netfp_delSP (netfpClientHandle, outboundSpId + index, &errCode) < 0)
        {
            printf ("Error: Unable to delete inbound security policy %d [Error code %d]\n", inboundSpId + index, errCode);
            return -1;
        }

        /* Delete the inbound IPSEC Channel */
        if (Netfp_delSA (netfpClientHandle, inboundSaHandle[index], &errCode) < 0)
        {
            printf ("Error: Unable to delete INBOUND IPSEC channel %d [Error code %d]\n", index, errCode);
            return -1;
        }

        /* Delete the inbound IPSEC Channel */
        if (Netfp_delSA (netfpClientHandle, outboundSaHandle[index], &errCode) < 0)
        {
            printf ("Error: Unable to delete OUTBOUND IPSEC channel %d [Error code %d]\n", index, errCode);
            return -1;
        }
    }
    printf("Debug: Security policy & association test passed\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the creation and deletion of multiple users
 *      and LTE channels
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Test_lte (void)
{
    Netfp_IfHandle          ifHandle;
    Netfp_OutboundFPHandle  fpOutboundHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_FlowCfg           flowCfg;
    int32_t                 myFlowHandle;
    Netfp_SockHandle        socketHandle[64][11];
    Netfp_UserHandle        ueHandle[64];
    int32_t                 errCode;
    int32_t                 ueIndex;
    int32_t                 rbIndex;
    Qmss_QueueHnd           gtpuQueue;
    Qmss_QueueHnd           encodeQueue;
    Qmss_QueueHnd           decodeQueue;
    uint8_t                 isAllocated;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
    uint32_t                gtpuIdentifier = 0xabcd000;

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Executing the LTE User & Channel Tests\n");

    /**********************************************************************************
     * Test Initialization:
     *********************************************************************************/

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth1");
    if (ifHandle == NULL)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", NETFP_INVALID_SPID, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", NETFP_INVALID_SPID, "eth1", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;
    strcpy (flowCfg.name, "LTE-Test-Flow");

    /* Create a test flow which will be used in the unit tests. */
    myFlowHandle = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (myFlowHandle < 0)
    {
        printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Open the encode/decode queue */
    encodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    decodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

    /* Cycle through and create 64 users */
    for (ueIndex = 0; ueIndex < 64; ueIndex++)
    {
        /* Create a user */
        ueHandle[ueIndex] = Test_createUser (ueIndex, myFlowHandle);
        if (ueHandle[ueIndex] == NULL)
            return -1;

        /* Cycle through and create channels for each DRB */
        for (rbIndex = 3; rbIndex < 11; rbIndex++)
        {
            socketHandle[ueIndex][rbIndex] = Test_createLTEChannel (ueHandle[ueIndex], rbIndex,
                                                                 fpInboundHandle, fpOutboundHandle,
                                                                 gtpuQueue, encodeQueue, decodeQueue,
                                                                 myFlowHandle, gtpuIdentifier++);
            if (socketHandle[ueIndex][rbIndex] == NULL)
                return -1;
        }
    }

    /* Cycle through and create 64 users */
    for (ueIndex = 0; ueIndex < 64; ueIndex++)
    {
        /* Cycle through and create channels for each DRB */
        for (rbIndex = 3; rbIndex < 11; rbIndex++)
        {
            if (Netfp_deleteLTEChannel (socketHandle[ueIndex][rbIndex] , &errCode) < 0)
            {
                printf ("Error: Unable to delete the channel [Error code %d]\n", errCode);
                return -1;
            }
        }

        /* Now delete the user */
        if (Netfp_deleteUser (ueHandle[ueIndex], &errCode) < 0)
        {
            printf ("Error: Unable to delete the user [Error code %d]\n", errCode);
            return -1;
        }
    }

    /* Shutdown the inbound and outbound fast path */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpInboundHandle, &errCode) < 0)
    {
        printf ("Error: NETFP Inbound fast path deletion failed [Error Code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: NETFP Inbound fast path deleted successfully\n");

    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpOutboundHandle, &errCode) < 0)
    {
        printf ("Error: NETFP Outbound fast path deletion failed [Error Code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: NETFP Outbound fast path deleted successfully\n");

    /* Delete the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: NETFP Interface deletion failed [Error Code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: NETFP Interface deleted successfully\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to test rekey a security association
 *
 *  @param[in]  spi
 *      SPI identifier
 *  @param[in]  direction
 *      Direction of the IPSEC Channel
 *  @param[in]  delOrder
 *      The order of deletion of SAs.
 *      0 - from oldest to latest
 *      1 - from latest to oldest
 *
 *  @retval
 *      Success -   SA Handle
 *  @retval
 *      Error   -   NULL
 */
static int32_t Test_rekeySA
(
    uint32_t            spi,
    Netfp_Direction     direction,
    uint32_t            delOrder
)
{
    Netfp_SACfg             saConfig;
    Netfp_SAHandle          saInboundHandle[TEST_MAX_REKEY_ITERATIONS];
    Netfp_SAHandle          saOutboundHandle[TEST_MAX_REKEY_ITERATIONS];
    int32_t                 errCode;
    int32_t                 index;
    Netfp_IfHandle          ifHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_InboundFPHandle   fpOutboundHandle;
    Netfp_SockHandle        socketHandle;
    int32_t                 status;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Executing the REKEY %s SA Tests [Del Order is %s]\n",
            (direction == Netfp_Direction_INBOUND) ? "Inbound" : "Outbound",
            (delOrder == 0) ? "Oldest->Newest" : "Newest->Oldest");
    printf ("***********************************************************************************\n");

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Template configuration. */
    saConfig.direction              = direction;
    saConfig.spi                    = spi;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
    saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else
        saConfig.ipsecCfg.keyAuthSize   = 0;

    if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
        saConfig.ipsecCfg.keyEncSize = 0;
    else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
        saConfig.ipsecCfg.keyEncSize = 20;
    else
        saConfig.ipsecCfg.keyEncSize = 24;

    /* Copy the keys. */
    memcpy (&saConfig.ipsecCfg.keyAuth, &authKey[0], saConfig.ipsecCfg.keyAuthSize);
    memcpy (&saConfig.ipsecCfg.keyEnc,  &encKey[0],  saConfig.ipsecCfg.keyEncSize);

    /* Configure the lifetime */
    saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

    /* Setup the Source & Destination IP Address for the SA. */
    if (direction == Netfp_Direction_INBOUND)
    {
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 200;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 100;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }
    else
    {
        /* Setup the Source & Destination IP Address for the SA. */
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 100;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 200;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }

    /* Populate the next hop MAC address and interface handle. */
    if (ifHandle != NULL)
    {
        saConfig.ifHandle = ifHandle;
        memcpy ((void*)&saConfig.nextHopMACAddress[0], (void*)nextHopMACAddress, 6);
    }

    /* Reset the security association: */
    memset ((void *)&saInboundHandle[0], 0, sizeof(saInboundHandle));
    memset ((void *)&saOutboundHandle[0], 0, sizeof(saOutboundHandle));

    /* Create the INBOUND & OUTBOUND Security Association: */
    saInboundHandle[0]  = Test_createSA (0xdead0000, Netfp_Direction_INBOUND,  ifHandle, &nextHopMACAddress[0]);
    saOutboundHandle[0] = Test_createSA (0xdead0001, Netfp_Direction_OUTBOUND, ifHandle, &nextHopMACAddress[0]);

    /* Create a security policy: */
    if (Test_createSP(saInboundHandle[0], Netfp_Direction_INBOUND, 1000) < 0)
        return -1;

    /* Create a security policy: */
    if (Test_createSP(saOutboundHandle[0], Netfp_Direction_OUTBOUND, 1001) < 0)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", 1000, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

     /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", 1001, "eth0.100", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Is the fast path active? */
    if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
    {
        printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
        return -1;
    }
    if (status == 0)
    {
        printf ("Error: Outbound fast path status is NOT ACTIVE\n");
        return -1;
    }

    /* Create the socket: */
    socketHandle = Test_createSocket (fpInboundHandle, fpOutboundHandle, 1);
    if (socketHandle == NULL)
        return -1;

    /* Socket should be active because the fast path and SA are active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
        return -1;
    }

    /***************************************************************************************************
     * Test Case: Rekey the SA [N Times]
     ***************************************************************************************************/
    for (index = 1; index < TEST_MAX_REKEY_ITERATIONS; index++)
    {
        /* Change SPI for rekey */
        saConfig.spi++;

        /* Reset the global socket reason: */
        socketNotifyReason = Netfp_Reason_UNKNOWN;
        if (direction == Netfp_Direction_INBOUND)
        {
            /* REKEY the INBOUND security association: */
            saInboundHandle[index] = Netfp_rekeySA (netfpClientHandle, saInboundHandle[index - 1], &saConfig, &errCode);
            if (saInboundHandle[index] == NULL)
            {
                printf ("Error: Unable to rekey SA[%d] [Error code: %d]\n", index, errCode);
                return -1;
            }
        }
        else
        {
            /* REKEY the OUTBOUND security association: */
            saOutboundHandle[index] = Netfp_rekeySA (netfpClientHandle, saOutboundHandle[index - 1], &saConfig, &errCode);
            if (saOutboundHandle[index] == NULL)
            {
                printf ("Error: Unable to rekey SA[%d] [Error code: %d]\n", index, errCode);
                return -1;
            }
        }

        /* Allow time for the NETFP Client Task to execute which will invoke the socket NOTIFY function */
        Task_sleep(10);

        /* Once the rekeying is done; ensure that the notification function was invoked */
        if (socketNotifyReason != Netfp_Reason_SP_ACTIVE)
        {
            printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
            return -1;
        }
    }

    /* Cleanup: All the security associations; the delete order is parameterized. We know that the
     * Root SA is the last entry */
    if (delOrder == 0)
    {
        /* Delete starting from the oldest entry. */
        for (index = 0; index < (TEST_MAX_REKEY_ITERATIONS - 1); index++)
        {
            /* Reset the global socket reason: */
            socketNotifyReason = Netfp_Reason_UNKNOWN;
            if (direction == Netfp_Direction_INBOUND)
            {
                /* Delete the INBOUND Security association: */
                if (Netfp_delSA(netfpClientHandle, saInboundHandle[index], &errCode) < 0 )
                {
                    printf ("Error: Unable to delete SA %d\n", errCode);
                    return -1;
                }
                saInboundHandle[index] = NULL;
            }
            else
            {
                /* Delete the OUTBOUND Security association: */
                if (Netfp_delSA(netfpClientHandle, saOutboundHandle[index], &errCode) < 0 )
                {
                    printf ("Error: Unable to delete SA %d\n", errCode);
                    return -1;
                }
                saOutboundHandle[index] = NULL;
            }

            /* Allow time for the NETFP Client Task to execute */
            Task_sleep(10);

            /* Deletion of the non-root SA should NOT trigger any socket event. */
            if (socketNotifyReason != Netfp_Reason_UNKNOWN)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }

        /* We will now be deleting the Root SA */
        socketNotifyReason = Netfp_Reason_UNKNOWN;

        /* Delete Root SA */
        if (direction == Netfp_Direction_INBOUND)
        {
            if (Netfp_delSA(netfpClientHandle, saInboundHandle[TEST_MAX_REKEY_ITERATIONS - 1], &errCode) < 0 )
            {
                printf ("Error: Unable to delete SA %d\n", errCode);
                return -1;
            }
            saInboundHandle[TEST_MAX_REKEY_ITERATIONS - 1] = NULL;
        }
        else
        {
            if (Netfp_delSA(netfpClientHandle, saOutboundHandle[TEST_MAX_REKEY_ITERATIONS - 1], &errCode) < 0 )
            {
                printf ("Error: Unable to delete SA %d\n", errCode);
                return -1;
            }
            saOutboundHandle[TEST_MAX_REKEY_ITERATIONS - 1] = NULL;
        }

        /* Allow time for the NETFP Client Task to execute */
        Task_sleep(10);

        /* Deletion of the Root SA should trigger the socket event with the correct reason */
        if (socketNotifyReason != Netfp_Reason_SA_DELETE)
        {
            printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
            return -1;
        }
    }
    else
    {
        /* Delete starting from the newest entry which implies that We will now be deleting the Root SA */
        socketNotifyReason = Netfp_Reason_UNKNOWN;

        /* Delete Root SA */
        if (direction == Netfp_Direction_INBOUND)
        {
            if (Netfp_delSA(netfpClientHandle, saInboundHandle[TEST_MAX_REKEY_ITERATIONS - 1], &errCode) < 0 )
            {
                printf ("Error: Unable to delete SA %d\n", errCode);
                return -1;
            }
            saInboundHandle[TEST_MAX_REKEY_ITERATIONS - 1] = NULL;
        }
        else
        {
            if (Netfp_delSA(netfpClientHandle, saOutboundHandle[TEST_MAX_REKEY_ITERATIONS - 1], &errCode) < 0 )
            {
                printf ("Error: Unable to delete SA %d\n", errCode);
                return -1;
            }
            saOutboundHandle[TEST_MAX_REKEY_ITERATIONS - 1] = NULL;
        }

        /* Allow time for the NETFP Client Task to execute */
        Task_sleep(10);

        /* Deletion of the Root SA should trigger the socket event with the correct reason */
        if (socketNotifyReason != Netfp_Reason_SA_DELETE)
        {
            printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
            return -1;
        }

        /* Delete the non-root SA */
        for (index = (TEST_MAX_REKEY_ITERATIONS - 2); index >= 0; index--)
        {
            /* Reset the global socket reason: */
            socketNotifyReason = Netfp_Reason_UNKNOWN;

            /* Delete SA */
            if (direction == Netfp_Direction_INBOUND)
            {
                if (Netfp_delSA(netfpClientHandle, saInboundHandle[index], &errCode) < 0 )
                {
                    printf ("Error: Unable to delete SA %d\n", errCode);
                    return -1;
                }
                saInboundHandle[index] = NULL;
            }
            else
            {
                if (Netfp_delSA(netfpClientHandle, saOutboundHandle[index], &errCode) < 0 )
                {
                    printf ("Error: Unable to delete SA %d\n", errCode);
                    return -1;
                }
                saOutboundHandle[index] = NULL;
            }

            /* Allow time for the NETFP Client Task to execute */
            Task_sleep(10);

            /* Deletion of the non-root SA should NOT trigger any socket event. */
            if (socketNotifyReason != Netfp_Reason_UNKNOWN)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }
    }

    /* Clean up the test socket. */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
    {
        printf ("Error: Closing Test socket failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Cleanup all the security association which are still active: */
    for (index = 0; index < TEST_MAX_REKEY_ITERATIONS; index++)
    {
        if (saInboundHandle[index] != NULL)
        {
            if (Netfp_delSA(netfpClientHandle, saInboundHandle[index], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
        }
        if (saOutboundHandle[index] != NULL)
        {
            if (Netfp_delSA(netfpClientHandle, saOutboundHandle[index], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
        }
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the Netfp_stopSA API
 *
 *  @retval
 *      Success -  0
 *  @retval
 *      Error   -  <0
 */
static int32_t Test_stopSA(Netfp_Direction direction)
{
    Netfp_SAHandle          saInboundHandle[TEST_MAX_REKEY_ITERATIONS];
    Netfp_SAHandle          saOutboundHandle[TEST_MAX_REKEY_ITERATIONS];
    int32_t                 errCode;
    uint32_t                index;
    Netfp_IfHandle          ifHandle;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_InboundFPHandle   fpOutboundHandle;
    Netfp_SockHandle        socketHandle;
    Netfp_OptionTLV         optInfo;
    uint8_t                 socketState;
    Netfp_SACfg             saConfig;
    int32_t                 status;
    uint8_t                 nextHopMACAddress[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    /* Debug Message: */
    printf ("***********************************************************************************\n");
    printf ("Debug: Executing the STOP %s SA Tests\n", (direction == Netfp_Direction_INBOUND) ? "Inbound" : "Outbound");
    printf ("***********************************************************************************\n");

    /* Create a dummy interface */
    ifHandle = Test_createInterface("eth0.100");
    if (ifHandle == NULL)
        return -1;

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the template SA Configuration. */
    saConfig.direction              = direction;
    saConfig.spi                    = 5000;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = Netfp_IpsecAuthMode_HMAC_SHA1;
    saConfig.ipsecCfg.encMode       = Netfp_IpsecCipherMode_NULL;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else
        saConfig.ipsecCfg.keyAuthSize   = 0;

    if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_NULL)
        saConfig.ipsecCfg.keyEncSize = 0;
    else if (saConfig.ipsecCfg.encMode == Netfp_IpsecCipherMode_AES_CTR)
        saConfig.ipsecCfg.keyEncSize = 20;
    else
        saConfig.ipsecCfg.keyEncSize = 24;

    /* Copy the keys. */
    memcpy (&saConfig.ipsecCfg.keyAuth, &authKey[0], saConfig.ipsecCfg.keyAuthSize);
    memcpy (&saConfig.ipsecCfg.keyEnc,  &encKey[0],  saConfig.ipsecCfg.keyEncSize);

    /* Configure the lifetime */
    saConfig.ipsecCfg.lifetime.softByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardByteLimit   = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.softPacketLimit = 0xFFFFFFFFFFFFFFFF;
    saConfig.ipsecCfg.lifetime.hardPacketLimit = 0xFFFFFFFFFFFFFFFF;

    /* Setup the Source & Destination IP Address for the SA. */
    if (direction == Netfp_Direction_INBOUND)
    {
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 200;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 100;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }
    else
    {
        /* Setup the Source & Destination IP Address for the SA. */
        saConfig.srcIP.ver = Netfp_IPVersion_IPV4;
        saConfig.srcIP.addr.ipv4.u.a8[0] = 192;
        saConfig.srcIP.addr.ipv4.u.a8[1] = 168;
        saConfig.srcIP.addr.ipv4.u.a8[2] = 100;
        saConfig.srcIP.addr.ipv4.u.a8[3] = 1;

        saConfig.dstIP.ver = Netfp_IPVersion_IPV4;
        saConfig.dstIP.addr.ipv4.u.a8[0] = 192;
        saConfig.dstIP.addr.ipv4.u.a8[1] = 168;
        saConfig.dstIP.addr.ipv4.u.a8[2] = 200;
        saConfig.dstIP.addr.ipv4.u.a8[3] = 1;
    }

    /* Populate the next hop MAC address and interface handle. */
    if (ifHandle != NULL)
    {
        saConfig.ifHandle = ifHandle;
        memcpy ((void*)&saConfig.nextHopMACAddress[0], (void*)nextHopMACAddress, 6);
    }

    /* Reset the security association: */
    memset ((void *)&saInboundHandle[0], 0, sizeof(saInboundHandle));
    memset ((void *)&saOutboundHandle[0], 0, sizeof(saOutboundHandle));

    /* Create the INBOUND & OUTBOUND Security Association: */
    saInboundHandle[0]  = Test_createSA (0xdead0000, Netfp_Direction_INBOUND,  ifHandle, &nextHopMACAddress[0]);
    saOutboundHandle[0] = Test_createSA (0xdead0001, Netfp_Direction_OUTBOUND, ifHandle, &nextHopMACAddress[0]);

    /* Create the inbound security policy: */
    if (Test_createSP(saInboundHandle[0], Netfp_Direction_INBOUND, 2000) < 0)
        return -1;

    /* Create the outbound security policy: */
    if (Test_createSP(saOutboundHandle[0], Netfp_Direction_OUTBOUND, 2001) < 0)
        return -1;

    /* Create the inbound fast path */
    fpInboundHandle = Test_createInboundFastPath ("Inbound", 2000, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

     /* Create the outbound fast path */
    fpOutboundHandle = Test_createOutboundFastPath ("Outbound", 2001, "eth0.100", &nextHopMACAddress[0], &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create the outbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Is the fast path active? */
    if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
    {
        printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
        return -1;
    }
    if (status == 0)
    {
        printf ("Error: Outbound fast path status is NOT ACTIVE\n");
        return -1;
    }

    /* Create the socket: */
    socketHandle = Test_createSocket (fpInboundHandle, fpOutboundHandle, 1);
    if (socketHandle == NULL)
        return -1;

    /* Socket should be active because the fast path and SA are active: */
    optInfo.type   = Netfp_Option_STATE;
    optInfo.length = 1;
    optInfo.value  = (void*)&socketState;
    if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
    {
        printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
        return -1;
    }
    if (socketState == 0)
    {
        printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
        return -1;
    }

    /* Execute the test depending upon the direction */
    for (index = 1; index < TEST_MAX_REKEY_ITERATIONS; index++)
    {
        /* Reset the global socket reason: */
        socketNotifyReason = Netfp_Reason_UNKNOWN;

        /***************************************************************************************************
         * Test Case  : Stop the SA
         * Expectation:
         *  - Stopping the SA should mark the corresponding security policy, fast path and socket INACTIVE
         *  - Socket should be notified with the appropriate reason code i.e. SP_INACTIVE
         ***************************************************************************************************/
        if (direction == Netfp_Direction_INBOUND)
        {
            /* Stop the INBOUND Security association: */
            if (Netfp_stopSA (netfpClientHandle, saInboundHandle[index - 1], &errCode) < 0)
            {
                printf ("Error: Stopping the SA failed [Error code %d]\n", errCode);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Security Policy is marked as INACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2000, &status, &errCode) < 0)
            {
                printf ("Error: Getting inbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 1)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Security Policy remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2001, &status, &errCode) < 0)
            {
                printf ("Error: Getting outbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Fast Path is marked as INACTIVE
             ***************************************************************************************************/
            if (Netfp_isInboundFastPathActive(netfpClientHandle, fpInboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the inbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 1)
            {
                printf ("Error: Inbound fast path status is ACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Fast Path remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Outbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket is marked as INACTIVE
             ***************************************************************************************************/
            optInfo.type   = Netfp_Option_STATE;
            optInfo.length = 1;
            optInfo.value  = (void*)&socketState;
            if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
            {
                printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
                return -1;
            }
            if (socketState == 1)
            {
                printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket Notifications reasons are reported
             ***************************************************************************************************/
            if (socketNotifyReason != Netfp_Reason_SP_INACTIVE)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }
        else
        {
            /* Stop the OUTBOUND Security association: */
            if (Netfp_stopSA (netfpClientHandle, saOutboundHandle[index - 1], &errCode) < 0)
            {
                printf ("Error: Stopping the SA failed [Error code %d]\n", errCode);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Security Policy is marked as INACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2001, &status, &errCode) < 0)
            {
                printf ("Error: Getting outbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 1)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Security Policy remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2000, &status, &errCode) < 0)
            {
                printf ("Error: Getting inbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Fast Path is marked as INACTIVE
             ***************************************************************************************************/
            if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 1)
            {
                printf ("Error: Outbound fast path status is ACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Fast Path remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isInboundFastPathActive(netfpClientHandle, fpInboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the inbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Inbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket is marked as INACTIVE
             ***************************************************************************************************/
            optInfo.type   = Netfp_Option_STATE;
            optInfo.length = 1;
            optInfo.value  = (void*)&socketState;
            if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
            {
                printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
                return -1;
            }
            if (socketState == 1)
            {
                printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket Notifications reasons are reported
             ***************************************************************************************************/
            if (socketNotifyReason != Netfp_Reason_SP_INACTIVE)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }

        /* Reset the global socket reason: */
        socketNotifyReason = Netfp_Reason_UNKNOWN;

        /***************************************************************************************************
         * Test Case: Rekeying SA
         * Expectation:
         *  - After rekeying the new SA plugs into the security policy activating it. This will also activate
         *    the fast path and sockets.
         *  - Socket should be notified with the appropriate reason code i.e. SP_ACTIVE
         ***************************************************************************************************/
        if (direction == Netfp_Direction_INBOUND)
        {
            /* Rekey the SA */
            saInboundHandle[index] = Netfp_rekeySA (netfpClientHandle, saInboundHandle[index - 1], &saConfig, &errCode);
            if (saInboundHandle[index] == NULL)
            {
                printf ("Error: NETFP Rekey SA failed [Error code %d]\n", errCode);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Security Policy is marked as ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2000, &status, &errCode) < 0)
            {
                printf ("Error: Getting inbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Security Policy remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2001, &status, &errCode) < 0)
            {
                printf ("Error: Getting outbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Fast Path is marked as ACTIVE
             ***************************************************************************************************/
            if (Netfp_isInboundFastPathActive(netfpClientHandle, fpInboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the inbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Inbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Fast Path remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Outbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket is marked as ACTIVE
             ***************************************************************************************************/
            optInfo.type   = Netfp_Option_STATE;
            optInfo.length = 1;
            optInfo.value  = (void*)&socketState;
            if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
            {
                printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
                return -1;
            }
            if (socketState == 0)
            {
                printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket Notifications reasons are reported
             ***************************************************************************************************/
            if (socketNotifyReason != Netfp_Reason_SP_ACTIVE)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }
        else
        {
            /* Rekey the SA */
            saOutboundHandle[index] = Netfp_rekeySA (netfpClientHandle, saOutboundHandle[index - 1], &saConfig, &errCode);
            if (saOutboundHandle[index] == NULL)
            {
                printf ("Error: NETFP Rekey SA failed [Error code %d]\n", errCode);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Security Policy is marked as ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2001, &status, &errCode) < 0)
            {
                printf ("Error: Getting outbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Security Policy remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isSPActive (netfpClientHandle, 2000, &status, &errCode) < 0)
            {
                printf ("Error: Getting inbound security policy Status failed [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Invalid security policy state %d detected\n", status);
                return -1;
            }

            /***************************************************************************************************
             * Validations: OUTBOUND Fast Path is marked as ACTIVE
             ***************************************************************************************************/
            if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Outbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: INBOUND Fast Path remains ACTIVE
             ***************************************************************************************************/
            if (Netfp_isInboundFastPathActive(netfpClientHandle, fpInboundHandle, &status, &errCode) < 0)
            {
                printf ("Error: Unable to get the inbound fast path status [Error code %d]\n", errCode);
                return -1;
            }
            if (status == 0)
            {
                printf ("Error: Inbound fast path status is INACTIVE\n");
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket is marked as ACTIVE
             ***************************************************************************************************/
            optInfo.type   = Netfp_Option_STATE;
            optInfo.length = 1;
            optInfo.value  = (void*)&socketState;
            if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
            {
                printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
                return -1;
            }
            if (socketState == 0)
            {
                printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
                return -1;
            }

            /***************************************************************************************************
             * Validations: Socket Notifications reasons are reported
             ***************************************************************************************************/
            if (socketNotifyReason != Netfp_Reason_SP_ACTIVE)
            {
                printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
                return -1;
            }
        }

        /* Reset the global socket reason: */
        socketNotifyReason = Netfp_Reason_UNKNOWN;

        /***************************************************************************************************
         * Test Case: Deleting the security association
         * Expectation:
         *  - The old security association is not used and deleting it should have no impact on any other
         *    components.
         *  - Socket should NOT be notified
         ***************************************************************************************************/
        if (direction == Netfp_Direction_INBOUND)
        {
            /* Delete the old Inbound SA: The SA is not used anymore. There should be no impact on anything. */
            if (Netfp_delSA(netfpClientHandle, saInboundHandle[index - 1], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
            saInboundHandle[index - 1] = NULL;
        }
        else
        {
            /* Delete the old Outbound SA: The SA is not used anymore. There should be no impact on anything. */
            if (Netfp_delSA(netfpClientHandle, saOutboundHandle[index - 1], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
            saOutboundHandle[index - 1] = NULL;
        }

        /***************************************************************************************************
         * Validations: INBOUND Security Policy is marked as ACTIVE
         ***************************************************************************************************/
        if (Netfp_isSPActive (netfpClientHandle, 2000, &status, &errCode) < 0)
        {
            printf ("Error: Getting inbound security policy Status failed [Error code %d]\n", errCode);
            return -1;
        }
        if (status == 0)
        {
            printf ("Error: Invalid security policy state %d detected\n", status);
            return -1;
        }

        /***************************************************************************************************
         * Validations: OUTBOUND Security Policy remains ACTIVE
         ***************************************************************************************************/
        if (Netfp_isSPActive (netfpClientHandle, 2001, &status, &errCode) < 0)
        {
            printf ("Error: Getting outbound security policy Status failed [Error code %d]\n", errCode);
            return -1;
        }
        if (status == 0)
        {
            printf ("Error: Invalid security policy state %d detected\n", status);
            return -1;
        }

        /***************************************************************************************************
         * Validations: INBOUND Fast Path is marked as ACTIVE
         ***************************************************************************************************/
        if (Netfp_isInboundFastPathActive(netfpClientHandle, fpInboundHandle, &status, &errCode) < 0)
        {
            printf ("Error: Unable to get the inbound fast path status [Error code %d]\n", errCode);
            return -1;
        }
        if (status == 0)
        {
            printf ("Error: Inbound fast path status is INACTIVE\n");
            return -1;
        }

        /***************************************************************************************************
         * Validations: OUTBOUND Fast Path remains ACTIVE
         ***************************************************************************************************/
        if (Netfp_isOutboundFastPathActive(netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
        {
            printf ("Error: Unable to get the outbound fast path status [Error code %d]\n", errCode);
            return -1;
        }
        if (status == 0)
        {
            printf ("Error: Outbound fast path status is INACTIVE\n");
            return -1;
        }

        /***************************************************************************************************
         * Validations: Socket is marked as ACTIVE
         ***************************************************************************************************/
        optInfo.type   = Netfp_Option_STATE;
        optInfo.length = 1;
        optInfo.value  = (void*)&socketState;
        if (Netfp_getSockOpt (socketHandle, &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
            return -1;
        }
        if (socketState == 0)
        {
            printf ("Error: Socket handle 0x%x invalid socket state %d detected\n", (uint32_t)socketHandle, socketState);
            return -1;
        }

        /***************************************************************************************************
         * Validations: No socket notification should have been detected
         ***************************************************************************************************/
        if (socketNotifyReason != Netfp_Reason_UNKNOWN)
        {
            printf ("Error: Unexpected socket notification detected Got %d\n", socketNotifyReason);
            return -1;
        }
    }

    /***************************************************************************************************
     * Test Cleanup:
     ***************************************************************************************************/

    /* Close the socket: */
    if (Netfp_closeSocket (socketHandle, &errCode) < 0)
        printf ("Error: Unable to close the socket [Error code %d]\n", errCode);

    /* Cleanup the fast path */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpInboundHandle, &errCode) < 0)
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpOutboundHandle, &errCode) < 0)
        printf ("Error: Unable to delete the outbound fast path [Error code %d]\n", errCode);

    /* Cleanup the security policy: */
    if (Netfp_delSP (netfpClientHandle, 2000, &errCode) < 0)
        printf ("Error: Unable to delete the inbound security policy [Error code %d]\n", errCode);
    if (Netfp_delSP (netfpClientHandle, 2001, &errCode) < 0)
        printf ("Error: Unable to delete the outbound security policy [Error code %d]\n", errCode);

    /* Cleanup all the security association which are still active: */
    for (index = 0; index < TEST_MAX_REKEY_ITERATIONS; index++)
    {
        if (saInboundHandle[index] != NULL)
        {
            if (Netfp_delSA(netfpClientHandle, saInboundHandle[index], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
        }
        if (saOutboundHandle[index] != NULL)
        {
            if (Netfp_delSA(netfpClientHandle, saOutboundHandle[index], &errCode) < 0)
            {
                printf ("Error: Unable to delete the security association [Error code %d]\n", errCode);
                return -1;
            }
        }
    }

    /* Clean up the interface */
    if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the interface [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_basicTask(UArg arg0, UArg arg1)
{
    printf ("*******************************************************\n");
    printf ("***************** Basic API Testing *******************\n");
    printf ("*******************************************************\n");

    /* Test the Interface API: */
    if (Test_netfpInterface() < 0)
    {
        printf ("Error: NETFP Interface API tests failed\n");
        return;
    }

    /* Test the Fast Path API: */
    if (Test_fastPath() < 0)
    {
        printf ("Error: NETFP Interface API tests failed\n");
        return;
    }

    /* Test the SA Stop API: */
    if (Test_stopSA(Netfp_Direction_INBOUND) < 0)
    {
        printf ("Error: NETFP Inbound Stop SA API test failed\n");
        return;
    }
    printf ("Debug: NETFP Inbound Stop SA test passed\n");

    /* Test the SA Stop API: */
    if (Test_stopSA(Netfp_Direction_OUTBOUND) < 0)
    {
        printf ("Error: NETFP Outbound Stop SA API test failed\n");
        return;
    }
    printf ("Debug: NETFP Outbound Stop SA test passed\n");

    /* Test the SA rekey API: */
    if (Test_rekeySA(0xdeaddead, Netfp_Direction_INBOUND, 0) < 0)
    {
        printf ("Error: NETFP inbound SA rekey API tests failed\n");
        return;
    }
    printf ("Debug: NETFP inbound SA rekey API tests with normal delete order passed\n");

    /* Test the SA rekey API: */
    if (Test_rekeySA(0xdeaddead, Netfp_Direction_INBOUND, 1) < 0)
    {
        printf ("Error: NETFP inbound SA rekey API tests failed\n");
        return;
    }
    printf ("Debug: NETFP inbound SA rekey API tests with reverse delete order passed\n");

    /* Test the SA rekey API: */
    if (Test_rekeySA(0xdeaddead, Netfp_Direction_OUTBOUND, 0) < 0)
    {
        printf ("Error: NETFP SA outbound rekey API tests failed\n");
        return;
    }
    printf ("Debug: NETFP outbound SA rekey API tests with normal delete order passed\n");

    /* Test the SA rekey API: */
    if (Test_rekeySA(0xdeaddead, Netfp_Direction_OUTBOUND, 1) < 0)
    {
        printf ("Error: NETFP SA outbound rekey API tests failed\n");
        return;
    }
    printf ("Debug: NETFP outbound SA rekey API tests with reverse delete order passed\n");

    /* Test the IPSEC Channel creation/deletion */
    if (Test_IPSecChannel() < 0)
    {
        printf ("Error: NETFP IPSEC Channel creation/deletion tests failed\n");
        return;
    }

    /* Test the NETFP deletion API */
    if (Test_netfpDeletion() < 0)
    {
        printf ("Error: NETFP Reference Counter tests failed\n");
        return;
    }

    /* Test the LTE 3GPP User & radio bearers */
    if (Test_lte() < 0)
    {
        printf ("Error: NETFP LTE tests failed\n");
        return;
    }

    /* Test the Interface API: */
    if (Test_netfpVLANInterface() < 0)
    {
        printf ("Error: NETFP Interface API tests failed\n");
        return;
    }

    /* All the tests passed. */
    printf ("Debug: Basic Testing passed\n");
    printf ("***********************************************************************************\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      This is a process which sends out packets with different priority tags.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_prioMarkTask(UArg arg0, UArg arg1)
{
    int32_t                 errCode;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_InboundFPHandle   fpEgressHandle;
    Ti_Pkt*                 ptrPkt;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataBufferLen;
    Netfp_SockHandle        sockHandle;
    Netfp_SockAddr          sockAddress;
    uint32_t                i;
    Netfp_OptionTLV         sockOpt;
    Netfp_OptionTLV         getSockOpt;
    int8_t                  sockPriority;

    /* Get the ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressHandle == NULL)
    {
        printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Get the egress fast path handle */
    fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressHandle == NULL)
    {
        printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Create a socket */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
    {
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 5000;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = 2000;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.notifyFunction  = NULL;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    printf ("Debug: Socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 5000;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    printf ("Debug: Socket has been connected successfully\n");

    /* Initialize the socket option: */
    memset ((void *)&sockOpt, 0, sizeof(Netfp_OptionTLV));

    /* Initialize the socket priority: */
    sockPriority = 0;

    /* Populate the socket option: */
    sockOpt.type   = Netfp_Option_PRIORITYTAG;
    sockOpt.length = 1;
    sockOpt.value  = &sockPriority;

    getSockOpt.type   = Netfp_Option_PRIORITYTAG;
    getSockOpt.length = 1;
    getSockOpt.value  = &sockPriority;

    for(i=0; i<10; i++)
    {
        /* Send out data: */
        ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, mtuReceiveHeap, 128);
        if (ptrPkt == NULL)
        {
            printf ("Error: Unable to allocate a packet:\n");
            return;
        }

        /* Get the data buffer */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

        /* Populate a message: */
        snprintf((char*)ptrDataBuffer, 128, "Hello from the test application. Packet nbr %d.", i+1);

        if( i > 0 )
        {
            if( i == 8 )
              sockPriority = -1;
            else
              sockPriority = i;

            /* Should fail when i=9 */
            if( (Netfp_setSockOpt(sockHandle,&sockOpt, &errCode) < 0) && (i != 9))
            {
                printf ("Error: NETFP Set Socket Priority (%d) Failed [Error Code %d]\n", sockPriority, errCode);
            }

            if( Netfp_getSockOpt(sockHandle,&getSockOpt, &errCode) < 0)
            {
                if( !((i >= 8) && (errCode == NETFP_ENOTREADY)) )
                {
                    printf ("Error: NETFP Get Socket Priority Failed [Error Code %d]\n", errCode);
                }
                /* NOT READY is expected when i >= 8  */
            }
            else
            {
                /* sockPriority is now populated by getSockOpt()
                   when enabled prio == i, when disabled prio == default */
                if( ((i <  8) && (sockPriority != i)) ||
                    ((i >= 8) && (sockPriority != NETFP_VLAN_PRIORITYTAG_DEFAULT)) )
                {
                    printf ("Error: getSockOpt return unexpected value %d/%d.\n", i, sockPriority);
                }
            }
        }

        /* Send out the packet: */
        if (Netfp_send (sockHandle, ptrPkt, 0, &errCode) < 0)
            printf ("Error: Unable to send data [Error code %d]\n", errCode);
    }
    printf ("Debug: %d packets are sent through Socket (%p)\n", i, sockHandle);

    /* Close the socket: */
    if (Netfp_closeSocket (sockHandle, &errCode) < 0)
        printf ("Error: Unable to close the socket [Error code %d]\n", errCode);

    /* Cleanup the fast path */
    if (Netfp_deleteInboundFastPath (netfpClientHandle, fpIngressHandle, &errCode) < 0)
        printf ("Error: Unable to delete the inbound fast path [Error code %d]\n", errCode);
    if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpEgressHandle, &errCode) < 0)
        printf ("Error: Unable to delete the outbound fast path [Error code %d]\n", errCode);

    return;
}

