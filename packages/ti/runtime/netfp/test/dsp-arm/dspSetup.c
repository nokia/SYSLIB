/*
 *   @file  dspSetup.c
 *
 *   @brief
 *      The file sets up the test environment and kick starts
 *      the appropriate test.
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
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

/* Test Include Files */
#include "netCfg.h"

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

/* The application is configuring the reassembly timeout period in seconds */
#define TEST_REASSEMBLY_TIMER_PERIOD_SEC           10

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Buffer used to store NETFP configuration file */
char                    netfpConfigBuffer[2500];

/* Unit Test: eNodeB MAC Address */
uint8_t                 eNBMacAddress[6];

/* NETFP Configuration after parsing the DAT file. */
Test_NetfpConfigInfo    netfpConfig;

/* NETFP Unit Test Selection: */
uint32_t                testSelection;

/* NETFP Event Handle for reassembly */
Event_Handle            netfpEventHnd;

/* NETFP Large Channel: */
MsgCom_ChHandle         netfpLargeChannel;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

/* System Configuration Handle. */
extern Resmgr_SysCfgHandle  handleSysCfg;

/* Global MSGCOM Instance handle. */
extern Msgcom_InstHandle    appMsgcomInstanceHandle;

/* Global application resource configuration */
extern Resmgr_ResourceCfg   appResourceConfig;

/* Global Database handle: */
extern Name_DBHandle        globalNameDatabaseHandle;

/* MTU receive heap: */
extern Pktlib_HeapHandle    mtuReceiveHeap;

/* Tasks:  */
extern void Test_socketIPv6Task(UArg arg0, UArg arg1);
extern void Test_socketTask(UArg arg0, UArg arg1);
extern void Test_WildcardingV4Task(UArg arg0, UArg arg1);
extern void Test_WildcardingV6Task(UArg arg0, UArg arg1);
extern void Test_benchmarkingReestablishTask(UArg arg0, UArg arg1);
extern void Test_hookTask(UArg arg0, UArg arg1);
extern void Test_nonSecSocketTask(UArg arg0, UArg arg1);
extern void Test_frameProtocolCrcTask(UArg arg0, UArg arg1);
extern void Test_fpDrbReestablishmentTask(UArg arg0, UArg arg1);
extern void Test_reestablishmentThread(UArg arg0, UArg arg1);
uint32_t getUserInput(void);
char* testautomation2_netfpdatPtr(void);
int32_t testautomation2_netfpdatSz(void);
/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Registered function for the reassembly heap allocation API
 *
 *  @param[in]  size
 *      Size of the reassembly buffer to be allocated
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Pointer to the allocated memory
 */
static uint8_t* Test_reassemblyHeapAlloc (uint32_t size, uint32_t arg)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), size, 0, NULL);
}

/**
 *  @b Description
 *  @n
 *      Registered function for the reassembly heap cleanup API
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be cleaned up
 *  @param[in]  size
 *      Size of the reassembly buffer
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Pointer to the allocated memory
 */
static void Test_reassemblyHeapFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, size);
}

/**
 *  @b Description
 *  @n
 *      The task is used to handle the reassembly of fragmented packets and
 *      also the timing out of the reassembly contexts.
 *
 *  @retval
 *      Not applicable
 */
static void Test_reassemblyTask(UArg arg0, UArg arg1)
{
    uint32_t    events;
    int32_t     errCode;

    while (1)
    {
        /* Wait for an event to be triggered */
        events = Event_pend(netfpEventHnd, Event_Id_NONE, Event_Id_00 + Event_Id_01, BIOS_WAIT_FOREVER);

        /* Event received: Is this because the reassembly timer expired? */
        if (events & Event_Id_00)
        {
            /* Reassembly Timer: */
            if (Netfp_reassemblyTimerTick (netfpClientHandle, TEST_REASSEMBLY_TIMER_PERIOD_SEC, &errCode) < 0)
            {
                /* NETFP Client: Reassembly Timer Processing failed.  */
                System_printf ("Error: NETFP Reassembly Timer Tick failed [Error code %d]\n", errCode);
            }
        }

        /* Event received: Is this because a fragmented packet was received? */
        if (events & Event_Id_01)
        {
            /* Fragmented Packet: Reassemble the packet */
            if (Netfp_reassembly (netfpClientHandle, &errCode) < 0)
            {
                /* NETFP Client: Packet reassembly failed. Display the error message */
                System_printf ("Error: NETFP Client Reassembly failed [Error code %d]\n", errCode);
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to parse the NETFP configuration file which
 *      is indicated by the file pointer.
 *
 *  @param[in]  configBuffer
 *      Data Buffer which has the NETFP configuration to be parsed
 *  @param[in]  configBufferSize
 *      Size of NETFP configuration file.
 *  @param[out] ptrNetfpConfig
 *      NETFP Configuration populated by this API.
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_parseConfigFile(char* configBuffer, int32_t configBufferSize, Test_NetfpConfigInfo* ptrNetfpConfig)
{
    char    buffer[256];
    char*   tokenName, *ptrConfigBuffer, *tmp;
    char*   tokenValue;
    int32_t index, size = 0, totalSize = 0;
    const char*   delimitters = " =,;\n\r";
    const char*   newline = "\n";

    ptrConfigBuffer = configBuffer;
    tmp = ptrConfigBuffer;

    /* Parse the file contents till complete. */
    while (totalSize < configBufferSize)
    {
        /* Read one line at a time */
        tmp = strpbrk  (tmp, newline);

        if (tmp == NULL)
            return -1;

        /* Skip the newline character */
        tmp = tmp + 1;

        /* Get the size of the line read */
        size = (tmp - ptrConfigBuffer);

        totalSize += size;

        memcpy (buffer, ptrConfigBuffer, size);

        /* Store the last processed line */
        ptrConfigBuffer = tmp;

        /* Skip the comments and empty lines */
        if (buffer[0] == '#' || buffer[0] == '\n' || buffer[0] == '\r')
            continue;

        /* Get the Token Name */
        tokenName = strtok(buffer, delimitters);

        /* Get the Token Value */
        tokenValue = strtok(NULL, delimitters);

        /* Validate the token name with what is expected. */
        if (strcmp (tokenName, "ENODEB_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                netfpConfig.eNodeBMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "ENODEB_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                netfpConfig.eNodeBIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS0") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.eNodeBIPAddress6[0]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS1") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.eNodeBIPAddress6[1]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS2") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.eNodeBIPAddress6[2]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS3") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.eNodeBIPAddress6[3]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "SGW0_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                netfpConfig.secGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "SGW0_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.secGwIPAddress6) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "SGW0_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                netfpConfig.secGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                netfpConfig.pdnGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN2_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                ptrNetfpConfig->pdnGw2IPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                netfpConfig.pdnGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN2_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                ptrNetfpConfig->pdnGw2MACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &netfpConfig.pdnGwIPAddress6) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "PDN2_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGw2IPAddress6) < 0)
                return -1;
        }
        else
        {
            /* Invalid Token Name. */
            System_printf ("Error: Token Name '%s' is NOT recognized\n", tokenName);
            return -1;
        }
    }

    /* File has been successfully parsed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Call back function registered for reassembly channels which is invoked
 *      when a fragmented packet is received
 *
 *  @param[in]  chHandle
 *      Channel Handle on which a packet was received.
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_setupFragmentReceived(MsgCom_ChHandle chHandle, uint32_t arg)
{
    /* Wake up the NETFP Reassembly Task: The reassembly task should ideally be configured
     * as a higher priority task than the NETFP Client Task since this task will be handling
     * data in the fast path. */
    Event_post(netfpEventHnd, Event_Id_01);
}

/**
 *  @b Description
 *  @n
 *      Call back function registered with the clock module which provides a periodic
 *      timer for reassembly services
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_reassemblyTimerExpired (void)
{
    /* Wake up the NETFP Reassembly Task: This indicates that the reassembly timer has expired */
    Event_post(netfpEventHnd, Event_Id_00);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to display a CLI command and get the
 *      user entered value.
 *
 *  @param[in]  displayStr
 *      String to be displayed on the CLI
 *  @param[out]  ptrValue
 *      User defined output
 *
 *  @retval
 *      Not applicable
 */
static void Test_cliGetUint32(char* displayStr, uint32_t* ptrValue)
{
    /* Display the string: */
    System_printf ("%s\n", displayStr);
#ifdef ARM_DSP_DOWNLOAD
    * ptrValue = getUserInput();
#else
    /* Wait for the user input */
	scanf ("%d", ptrValue);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the IPv4 environment for the test
 *
 *  @param[in]  spidIngress
 *      Ingress security policy identifier
 *  @param[in]  spidEgress
 *      Egress security policy identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4FastPath
(
    uint32_t spidIngress,
    uint32_t spidEgress
)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/
    if (spidIngress != 0)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

        /* Populate the Fast Path configuration. */
        inboundFPCfg.spId                 = spidIngress;

        /* Initialize spidMode */
        if(spidIngress == NETFP_INVALID_SPID)
            inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
        else
            inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

        inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = netfpConfig.pdnGwIPAddress[0];
        inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = netfpConfig.pdnGwIPAddress[1];
        inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = netfpConfig.pdnGwIPAddress[2];
        inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = netfpConfig.pdnGwIPAddress[3];
        inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = netfpConfig.eNodeBIPAddress[0];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = netfpConfig.eNodeBIPAddress[1];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = netfpConfig.eNodeBIPAddress[2];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = netfpConfig.eNodeBIPAddress[3];
        strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

        /* Add the Ingress IPv4 Fast Path. */
        fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
        if (fpIngressHandle == NULL)
        {
            System_printf ("Error: Unable to create Ingress IPv4 fast path Error %d\n", errCode);
            return -1;
        }
        System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);
    }

    /************************************************************************************
     * Egress IPv4 Fast Path:
     ************************************************************************************/
    if (spidEgress != 0)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));
        
        /* Populate the Fast Path configuration. */
        outboundFPCfg.spId                    = spidEgress;
        outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = netfpConfig.pdnGwIPAddress[0];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = netfpConfig.pdnGwIPAddress[1];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = netfpConfig.pdnGwIPAddress[2];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = netfpConfig.pdnGwIPAddress[3];
        outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = netfpConfig.eNodeBIPAddress[0];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = netfpConfig.eNodeBIPAddress[1];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = netfpConfig.eNodeBIPAddress[2];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = netfpConfig.eNodeBIPAddress[3];
        
        /* Setup QCI-DSCP mapping for the egress fastpath */
        for (i = 0; i < 64; i ++)
            outboundFPCfg.dscpMapping[i] = i;
        strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");
        
        /* Add the IPv4 Egress Fast Path. */
        fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
        if (fpEgressHandle == NULL)
        {
            System_printf ("Error: Unable to create Egress IPv4 fast path Error %d\n", errCode);
            return -1;
        }
        System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);
        
        /* Loop around till the fast path is active: */
        while (1)
        {
            int32_t status;
            if(Netfp_isOutboundFastPathActive(netfpClientHandle, fpEgressHandle, &status, &errCode) < 0)
            {
                System_printf ("Error: Getting outbound IPv4 fast path status failed [Error code %d]\n", errCode);
                return -1;
            }
            if(status == 1)
                break;
            Task_sleep(10);
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the IPv4 environment for the wildcarding test
 *
 *  @param[in]  spidWcIngress
 *      Ingress security policy identifier
 *  @param[in]  spidWcEgress
 *      Egress security policy identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4WildcardingFastPath
(
    uint32_t spidWcIngress,
    uint32_t spidWcEgress
)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;

    /************************************************************************************
     * Ingress Wild Carded IPv4 Fast Path:
     ************************************************************************************/
    if (spidWcIngress != 0)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));
        
        /* Initialize spidMode */
        if(spidWcIngress == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
        else if (spidWcIngress == 0)
        inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
        else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
        
        inboundFPCfg.spId                    = spidWcIngress;
        inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 0;
        inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = netfpConfig.eNodeBIPAddress[0];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = netfpConfig.eNodeBIPAddress[1];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = netfpConfig.eNodeBIPAddress[2];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = netfpConfig.eNodeBIPAddress[3];
        strcpy (inboundFPCfg.name, "Ingress-IPv4-WCFastPath");
    
        /* Add the Ingress IPv4 Fast Path. */
        fpIngressHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPCfg, &errCode);
        if (fpIngressHandle == NULL)
        {
            System_printf ("Error: Unable to create wild carded Ingress fast path Error %d\n", errCode);
            return -1;
        }
        System_printf ("Debug: Ingress Wild carded IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);
        
        /* Delete Fast path test */
        if (Netfp_deleteInboundFastPath(netfpClientHandle, fpIngressHandle, &errCode) < 0 )
        {
            System_printf ("Error: Unable to delete wildcarding Ingress fast path Error %d\n", errCode);
            return -1;
        }
        
        /* Add the WildCarding Ingress Fast Path. */
        fpIngressHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPCfg, &errCode);
        if (fpIngressHandle == NULL)
        {
            System_printf ("Error: Unable to create wild carded Ingress fast path Error %d\n", errCode);
            return -1;
        }
        System_printf ("Debug: Wild carding Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);
    }

    /************************************************************************************
     * Egress IPv4 Wildcarding Fast Path:
     ************************************************************************************/
    if (spidWcEgress != 0)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));
        
        /* Populate the configuration: */
        outboundFPCfg.spId                       = spidWcEgress;
        outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
        outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = netfpConfig.pdnGw2IPAddress[0];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = netfpConfig.pdnGw2IPAddress[1];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = netfpConfig.pdnGw2IPAddress[2];
        outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = netfpConfig.pdnGw2IPAddress[3];
        outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
        outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = netfpConfig.eNodeBIPAddress[0];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = netfpConfig.eNodeBIPAddress[1];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = netfpConfig.eNodeBIPAddress[2];
        outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = netfpConfig.eNodeBIPAddress[3];
        
        /* Setup QCI-DSCP mapping for the egress fastpath */
        for (i = 0; i < 64; i ++)
            outboundFPCfg.dscpMapping[i] = i;
        strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath2");
        
        /* Add the IPv4 Egress Fast Path. */
        fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
        if (fpEgressHandle == NULL)
        {
            System_printf ("Error: Unable to create Egress fast path for wild carding testing, Error %d\n", errCode);
            return -1;
        }
        
        /* Loop around till the fast path is active: */
        while (1)
        {
            int32_t status;
            if(Netfp_isOutboundFastPathActive(netfpClientHandle, fpEgressHandle, &status, &errCode) < 0)
            {
                System_printf ("Error: Getting outbound fast path status failed [Error code %d]\n", errCode);
                return -1;
            }
            if(status == 1)
                break;
            Task_sleep(10);
        }
        System_printf ("Debug: Egress IPv4 Fast Path for Wild card testing is added, Handle=0x%p\n", fpEgressHandle);
    }
    return 0;
}
/**
 *  @b Description
 *  @n
 *      The function is used to setup the IPv6 environment for the test
 *
 *  @param[in]  spidIngress
 *      Ingress security policy identifier
 *  @param[in]  spidEgress
 *      Egress security policy identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv6FastPath
(
    uint32_t spidIngress,
    uint32_t spidEgress
)

{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;

    /************************************************************************************
     * Ingress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId            = spidIngress;
    inboundFPCfg.srcIP.ver       = Netfp_IPVersion_IPV6;
    inboundFPCfg.srcIP.addr.ipv6 = netfpConfig.pdnGwIPAddress6;
    inboundFPCfg.dstIP.ver       = Netfp_IPVersion_IPV6;
    inboundFPCfg.dstIP.addr.ipv6 = netfpConfig.eNodeBIPAddress6[0];
    strcpy (inboundFPCfg.name, "Ingress-IPv6-FastPath");
    if(spidIngress == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    /* Add the Ingress IPv6 Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress IPv6 fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv6 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
     * Egress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId            = spidEgress;
    outboundFPCfg.dstIP.ver       = Netfp_IPVersion_IPV6;
    outboundFPCfg.dstIP.addr.ipv6 = netfpConfig.pdnGwIPAddress6;
    outboundFPCfg.srcIP.ver       = Netfp_IPVersion_IPV6;
    outboundFPCfg.srcIP.addr.ipv6 = netfpConfig.eNodeBIPAddress6[0];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv6-FastPath");

    /* Add the IPv6 Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress IPv6 fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv6 Fast Path Handle 0x%p\n", fpEgressHandle);

    /* Loop around till the fast path is active: */
    while (1)
    {
        int32_t status;

        if(Netfp_isOutboundFastPathActive(netfpClientHandle, fpEgressHandle, &status, &errCode) < 0)
        {
            System_printf ("Error: Getting outbound IPv6 fast path status failed [Error code %d]\n", errCode);
            return -1;
        }
        if(status == 1)
            break;
        Task_sleep(10);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test adding non-secure wildcarding
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4NonSecureWildcardingFastPath(void)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    int32_t                 errCode;

    /************************************************************************************
     * Ingress Wild Carded IPv4 Fast Path:
     ************************************************************************************/
    if (1)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));
        
        /* Initialize spidMode */
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;

        inboundFPCfg.spId                    = 0;
        inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 0;
        inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 0;
        inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
        inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = 10; //netfpConfig.eNodeBIPAddress[0];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = 10; //netfpConfig.eNodeBIPAddress[1];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = 10; //netfpConfig.eNodeBIPAddress[2];
        inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = 2;  //netfpConfig.eNodeBIPAddress[3];
        strcpy (inboundFPCfg.name, "InIPv4-NonSecureWCFP");
    
        /* Add the Ingress IPv4 Fast Path. */
        fpIngressHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPCfg, &errCode);
        if (fpIngressHandle == NULL)
        {
            System_printf ("Error: Unable to create wild carded Ingress fast path Error %d\n", errCode);
            return -1;
        }
        System_printf ("Debug: Wild carding Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the IPv6 environment for the wildcarding test
 *
 *  @param[in]  spidWcIngress
 *      Ingress security policy identifier
 *  @param[in]  spidWcEgress
 *      Egress security policy identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv6WildcardingFastPath
(
    uint32_t spidWcIngress,
    uint32_t spidWcEgress
)

{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;
    Netfp_IP6N              pdnWcIPAddress6 = {0};

    /************************************************************************************
     * Ingress Wild Carding IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    if(spidWcIngress == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else if (spidWcIngress == 0)
        inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.spId            = spidWcIngress;
    inboundFPCfg.srcIP.ver       = Netfp_IPVersion_IPV6;
    inboundFPCfg.srcIP.addr.ipv6 = pdnWcIPAddress6;
    inboundFPCfg.dstIP.ver       = Netfp_IPVersion_IPV6;
    inboundFPCfg.dstIP.addr.ipv6 = netfpConfig.eNodeBIPAddress6[0];
    strcpy (inboundFPCfg.name, "Ingress-IPv6-WcFastPath");

    /* Add the Ingress IPv6 Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Wild carding Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress Wild Carding IPv6 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
     * Egress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the fast path configuration: */
    outboundFPCfg.spId            = spidWcEgress;
    outboundFPCfg.dstIP.ver       = Netfp_IPVersion_IPV6;
    outboundFPCfg.dstIP.addr.ipv6 = netfpConfig.pdnGw2IPAddress6;
    outboundFPCfg.srcIP.ver       = Netfp_IPVersion_IPV6;
    outboundFPCfg.srcIP.addr.ipv6 = netfpConfig.eNodeBIPAddress6[0];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv6-FastPath2");

    /* Add the IPv6 Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create second Egress fast path for wild carding testing, Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv6 Fast Path for Wild carding is created, Handle 0x%p\n", fpEgressHandle);

    /* Loop around till the fast path is active: */
    while (1)
    {
        int32_t status;
        if(Netfp_isOutboundFastPathActive(netfpClientHandle, fpEgressHandle, &status, &errCode) < 0)
        {
            System_printf ("Error: Getting outbound fast path status failed [Error code %d]\n", errCode);
            return -1;
        }
        if(status == 1)
            break;
        Task_sleep(10);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which sets up the environment for executing
 *      the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_envSetupTask(UArg arg0, UArg arg1)
{
    int32_t                 			size;
    Task_Params             			taskParams;
    int32_t                 			errCode;
    Msgcom_ChannelCfg       			chConfig;
    Netfp_ReassemblyConfig  			reassemblyConfig;
    Resmgr_ResourceCfg*     			ptrCfg;
    MsgCom_ChHandle         			outerIPChannel;
    MsgCom_ChHandle         			innerIPChannel;
    Clock_Handle            			reassemblyTimer;
    Clock_Params            			clockParams;
    Netfp_DefaultReassemblyMgmtCfg      defaultReassemblyCfg;
    uint32_t                            testSelection;
    Name_ResourceCfg                    namedResCfg;
    char                                ip6String[128];
    uint32_t                            spidIPv4Ingress;
    uint32_t                            spidIPv4Egress;
    uint32_t                            spidIPv6Ingress;
    uint32_t                            spidIPv6Egress;
    uint32_t                            spidIPv4WcIngress;
    uint32_t                            spidIPv4WcEgress;
    uint32_t                            spidIPv6WcIngress;
    uint32_t                            spidIPv6WcEgress;
    uint32_t                            enabledl3Shaper = 0;
    uint32_t                            wildcardingTest = 1;


#ifdef ARM_DSP_DOWNLOAD
    char* netfpcbuf;
    size = testautomation2_netfpdatSz();
    netfpcbuf=testautomation2_netfpdatPtr();
    /* Parse the NETFP Configuration. */
    if (Test_parseConfigFile(netfpcbuf, size, &netfpConfig) < 0)
    {
        /* Error: NETFP configuration file parsing failed. */
        System_printf ("Error: Unable to parse the NETFP configuration file.\n");
        return;
    }
    
#else
    FILE*                   			configFileHandle;
    /* Open the NETFP configuration file for the tests */
    configFileHandle = fopen ("c:\\netfp.dat", "r");
    if (configFileHandle == NULL)
    {
        System_printf ("Error: Unable to open NETFP configuration file (c:\\netfp.dat)\n");
        return;
    }

    /* Initialize the size. */
    size = 0;

    /* Read data from the file and place it into the buffer */
    while (!feof(configFileHandle))
    {
        netfpConfigBuffer[size++] = fgetc(configFileHandle);
        if (size == sizeof (netfpConfigBuffer))
        {
            System_printf ("Error: Configuration file is too big\n");
            return;
        }
    }

    /* Remove the extra byte which was incremented */
    size = size - 1;

    /* Close the file. */
    fclose (configFileHandle);

    /* Parse the NETFP Configuration. */
    if (Test_parseConfigFile(netfpConfigBuffer, size, &netfpConfig) < 0)
    {
        /* Error: NETFP configuration file parsing failed. */
        System_printf ("Error: Unable to parse the NETFP configuration file.\n");
        return;
    }
#endif
    /* Get the application resource configuration. */
    ptrCfg = &appResourceConfig;
    /* Reassembly operations in our test setup are handled on Core1. */
    if (DNUM == 1)
    {
        /* Create an event object for the reassembly. The object will be posted to whenever a
         * fragmented packet is received on either of the MSGCOM reassembly channels. */
        netfpEventHnd = Event_create(NULL, NULL);
        if (netfpEventHnd == NULL)
            return;

        /* Initialize the channel configuration. */
        memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Populate the channel configuration. */
        chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                        = Test_setupFragmentReceived;
        chConfig.msgcomInstHandle                                   = appMsgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 1;

        /* Outer-IP reassembly channel: */
        outerIPChannel = Msgcom_create ("OuterIP-Reassembly-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (outerIPChannel == NULL)
        {
            System_printf ("Error: Unable to open the Msgcom channel Error : %d\n", errCode);
            return;
        }

        /* Initialize the channel configuration. */
        memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Populate the channel configuration. */
        chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                                        = Test_setupFragmentReceived;
        chConfig.msgcomInstHandle                                   = appMsgcomInstanceHandle;
        chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[1].accChannel;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[1].queue;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[1].pdspId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[1].eventId;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 1;

        /* Inner IP reassembly channel: */
        innerIPChannel = Msgcom_create ("InnerIP-Reassembly-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (innerIPChannel == NULL)
        {
            System_printf ("Error: Unable to open the Msgcom channel Error : %d\n", errCode);
            return;
        }

        /* Initialize the channel configuration. */
        memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Populate the channel configuration: */
        chConfig.mode                           = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.msgcomInstHandle               = appMsgcomInstanceHandle;
        chConfig.appCallBack                    = NULL;
        chConfig.u.queueCfg.interruptMode       = Msgcom_QueueInterruptMode_NO_INTERRUPT;

        /* Large Packet Channel: This is the channel where the packets > NETFP_PASS_MAX_BUFFER_SIZE will be received
        * since the NETCP cannot handle these packets. */
        netfpLargeChannel = Msgcom_create ("Large-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (netfpLargeChannel == NULL)
        {
            System_printf ("Error: Unable to open the Msgcom channel Error : %d\n", errCode);
            return;
        }

        /* Initialize the reassembly configuration */
        memset ((void*)&reassemblyConfig, 0, sizeof(Netfp_ReassemblyConfig));

        /* Initialize the default reassembly configuration. */
        memset ((void*)&defaultReassemblyCfg, 0, sizeof(Netfp_DefaultReassemblyMgmtCfg));

        /* Populate the default reassembly configuration: */
        defaultReassemblyCfg.bufferThresholdUpper = 20000;
        defaultReassemblyCfg.bufferThresholdLower = 18000;

        /* Populate the reassembly configuration:
         *  - Memory region 1 allocated for the reassembly operations.
         *  - The reassembly inner, outer and large channels should be *WRITER* channels which
         *    are always found. */
        reassemblyConfig.reassemblyMemRegion    = ptrCfg->memRegionResponse[1].memRegionHandle;
        reassemblyConfig.numFragmentedPkt       = 64;
        reassemblyConfig.numReassemblyContexts  = 128;
        reassemblyConfig.reassemblyTimeout      = 30;               /* Unit - Seconds */
        reassemblyConfig.outerIpReassemChannel  = outerIPChannel;
        reassemblyConfig.innerIpReassemChannel  = innerIPChannel;
        reassemblyConfig.largePacketChannel     = Msgcom_find (appMsgcomInstanceHandle, "Large-Channel", &errCode);
        reassemblyConfig.reassemblyHeapAlloc    = Test_reassemblyHeapAlloc;
        reassemblyConfig.reassemblyHeapFree     = Test_reassemblyHeapFree;
        reassemblyConfig.reassemblyMgmt         = NULL;
        reassemblyConfig.ptrReassemblyMgmtCfg   = &defaultReassemblyCfg;
        reassemblyConfig.sizeReassemblyCfg      = sizeof(defaultReassemblyCfg);

        /* Register the client to handle the reassembly for all fragmented packets */
        if (Netfp_registerReassemblyService (netfpClientHandle, &reassemblyConfig, &errCode) < 0)
        {
            /* Error: Failed to initialize the NETFP reassembly */
            System_printf ("Error: NETFP Reassembly initialization failed\n");
            return;
        }

        /* Initialize the Clock Parameters */
        Clock_Params_init(&clockParams);
        clockParams.period      = 1000 * TEST_REASSEMBLY_TIMER_PERIOD_SEC;     /* Convert to millisec */
        clockParams.startFlag   = TRUE;

        /* Instantiate a clock to check for reassembly timeouts */
        reassemblyTimer = Clock_create ((Clock_FuncPtr)Test_reassemblyTimerExpired, TEST_REASSEMBLY_TIMER_PERIOD_SEC, &clockParams, NULL);
        if (reassemblyTimer == NULL)
        {
            System_printf ("Error: Reassembly Timer creation failed.\n");
            return;
        }

        /* Create the reassembly task which is responsible for handling the reassembly
         * of all received fragments. This task is created since the NETFP Client has
         * been registered for reassembly. */
        Task_Params_init(&taskParams);
        taskParams.stackSize = 2048;
        taskParams.priority  = 2;
        Task_create(Test_reassemblyTask, &taskParams, NULL);
    }

    /* Keep a copy of the eNB MAC Address (Its useful for other tests) */
    eNBMacAddress[0] = netfpConfig.eNodeBMACAddress[0];
    eNBMacAddress[1] = netfpConfig.eNodeBMACAddress[1];
    eNBMacAddress[2] = netfpConfig.eNodeBMACAddress[2];
    eNBMacAddress[3] = netfpConfig.eNodeBMACAddress[3];
    eNBMacAddress[4] = netfpConfig.eNodeBMACAddress[4];
    eNBMacAddress[5] = netfpConfig.eNodeBMACAddress[5];

    /****************************************************************************
     * CLI Based Menu: Executes only on DSP Core0
     ****************************************************************************/
    if (DNUM == 0)
    {
        /* Display the setup configuration: */
        System_printf ("-------------------------------------------------------------\n");
        System_printf ("Test Environment [Configured via C:/netfp.dat]:\n");
        System_printf ("My IPv4 Address             : %d.%d.%d.%d\n",
                        netfpConfig.eNodeBIPAddress[0], netfpConfig.eNodeBIPAddress[1],
                        netfpConfig.eNodeBIPAddress[2], netfpConfig.eNodeBIPAddress[3]);
        Netfp_convertIP6ToStr (netfpConfig.eNodeBIPAddress6[0], &ip6String[0]);
        System_printf ("My IPv6 Address             : %s\n", ip6String);
        System_printf ("PDN IPv4 Address            : %d.%d.%d.%d\n",
                        netfpConfig.pdnGwIPAddress[0], netfpConfig.pdnGwIPAddress[1],
                        netfpConfig.pdnGwIPAddress[2], netfpConfig.pdnGwIPAddress[3]);
        Netfp_convertIP6ToStr (netfpConfig.pdnGwIPAddress6, &ip6String[0]);
        System_printf ("PDN IPv6 Address            : %s\n", ip6String);
        System_printf ("SeGW IPv4 Address           : %d.%d.%d.%d\n",
                        netfpConfig.secGwIPAddress[0], netfpConfig.secGwIPAddress[1],
                        netfpConfig.secGwIPAddress[2], netfpConfig.secGwIPAddress[3]);
        Netfp_convertIP6ToStr (netfpConfig.secGwIPAddress6, &ip6String[0]);
        System_printf ("SeGW IPv6 Address           : %s\n", ip6String);
        System_printf ("-------------------------------------------------------------\n");

        /* Loop around for a selection: */
        while (1)
        {
            Task_sleep (20);
            System_printf ("*******************************************************\n");
            System_printf ("DSP-ARM NETFP Unit Test Menu                \n");
            System_printf ("Please select the type of test to execute:  \n");
            System_printf ("1. Data Test                                \n");
            System_printf ("2. Hook Test                                \n");
            System_printf ("3. Reestablishment API benchmarking         \n");
            System_printf ("4. Frame Protocol CRC Test                  \n");
            System_printf ("5. FP Re-establishment Test                 \n");
            System_printf ("6. Re-establishment Stress Test             \n");
            Test_cliGetUint32(">Enter your selection: ", &testSelection);

            /* Was there a valid test selection: */
            if ((testSelection >= 1) && (testSelection <= 6))
                break;
        }

        /* Create a named resource and indicate to the other core the test which is being executed. */
        memset ((void *)&namedResCfg, 0, sizeof(Name_ResourceCfg));
        namedResCfg.handle1  = (uint32_t)testSelection;
        strcpy(namedResCfg.name, "TestSelection");
        if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, &namedResCfg, &errCode) < 0)
            System_printf ("Error: Creating Named resource failed with error code %d\n", errCode);
    }
    else
    {
        /* Loop around till we get a selection: */
        while (1)
        {
            if (Name_findResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, "TestSelection", &namedResCfg, &errCode) == 0)
                break;
            Task_sleep(100);
        }

        /* Get the test selection from the named resource object. */
        testSelection = (uint32_t)namedResCfg.handle1;
    }

    /* Setup and launch the tests */
    switch (testSelection)
    {
        case 1:
        {
            /************************************************************************
             * Data Test:
             ************************************************************************/

            if (DNUM == 0)
            {
                /* Enable L3 Qos? */
                Test_cliGetUint32("Enable L3Qos(1 to enable, 0 to disable)? ", &enabledl3Shaper);
                Test_cliGetUint32("WildCarding Test(1 to enable, 0 to disable)? ", &wildcardingTest);

                /* Offload the Ingress and Egress SPID: */
                Test_cliGetUint32("Ingress IPv4 SPID: ", &spidIPv4Ingress);
                Test_cliGetUint32("Egress  IPv4 SPID: ", &spidIPv4Egress);
                Test_cliGetUint32("Ingress IPv6 SPID: ", &spidIPv6Ingress);
                Test_cliGetUint32("Egress  IPv6 SPID: ", &spidIPv6Egress);

                /* Setup the ingress and egress IPv4 Security policy identifier. */
                if (Test_setupIPv4FastPath(spidIPv4Ingress, spidIPv4Egress) < 0)
                    return;

                /* Setup the IPv6 Fast paths */
                if (Test_setupIPv6FastPath(spidIPv6Ingress, spidIPv6Egress) < 0)
                    return;

                if(wildcardingTest)
                {
                    uint32_t     nonSecWcTest=0;

                    Test_cliGetUint32("Ingress IPv4 wildcarding SPID: ", &spidIPv4WcIngress);
                    Test_cliGetUint32("Egress  IPv4 wildcarding SPID: ", &spidIPv4WcEgress);

                    Test_cliGetUint32("Ingress IPv6 wildcarding SPID: ", &spidIPv6WcIngress);
                    Test_cliGetUint32("Egress  IPv6 wildcarding SPID: ", &spidIPv6WcEgress);
                
                    /* Setup the ingress and egress IPv4 Security policy identifier. */
                    if (Test_setupIPv4WildcardingFastPath(spidIPv4WcIngress, spidIPv4WcEgress) < 0)
                        return;

                    /* Setup the IPv6 Fast paths */
                    if (Test_setupIPv6WildcardingFastPath(spidIPv6WcIngress, spidIPv6WcEgress) < 0)
                        return;

                    Test_cliGetUint32("Non secure wild carding entry test(1/0): ", &nonSecWcTest);
                    if(nonSecWcTest)
                    {
                        /* Setup the non-secure ingress - Test adding only */
                        Test_setupIPv4NonSecureWildcardingFastPath();

                        /* Setup the task parameters for the IPv4 Socket Task */
                        Task_Params_init(&taskParams);
                        taskParams.stackSize = 8*1024;
                        taskParams.priority  = 5;
                        Task_create(Test_nonSecSocketTask, &taskParams, NULL);
                    }    
                }
            }

            /* Setup the task parameters for the IPv4 Socket Task */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_socketTask, &taskParams, NULL);

            /* Setup the task parameters for the IPv6 Socket Task */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;

            /* Enable L3 Qos setup on Core 0? */
            if(DNUM == 0)
            {
                taskParams.arg0 = enabledl3Shaper;
            }
            Task_create(Test_socketIPv6Task, &taskParams, NULL);

            Task_sleep(100);
            if(wildcardingTest)
            {
                /* Setup the task parameters for the IPv4 Socket Task */
                Task_Params_init(&taskParams);
                taskParams.stackSize = 8*1024;
                taskParams.priority  = 5;
                Task_create(Test_WildcardingV4Task, &taskParams, NULL);
    
                /* Setup the task parameters for the IPv4 Socket Task */
                Task_Params_init(&taskParams);
                taskParams.stackSize = 8*1024;
                taskParams.priority  = 5;
                Task_create(Test_WildcardingV6Task, &taskParams, NULL);
            }
            break;
        }
        case 2:
        {
            /************************************************************************
             * Hook Test:
             ************************************************************************/
            if (DNUM == 0)
            {
                /* Offload the Ingress and Egress SPID: */
                Test_cliGetUint32("Ingress IPv4 SPID: ", &spidIPv4Ingress);
                Test_cliGetUint32("Egress  IPv4 SPID: ", &spidIPv4Egress);

                /* Setup the ingress and egress IPv4 security policy identifier. */
                if (Test_setupIPv4FastPath(spidIPv4Ingress, spidIPv4Egress) < 0)
                    return;
            }

            /* Setup the task parameters for the hook Task */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_hookTask, &taskParams, NULL);
            break;
        }
        case 3:
        {
            /************************************************************************
             * Reestablishment Procedure Benchmarking Test:
             ************************************************************************/
            if (DNUM == 0)
            {
                /* Offload the Ingress and Egress SPID: */
                Test_cliGetUint32("Ingress IPv4 SPID: ", &spidIPv4Ingress);
                Test_cliGetUint32("Egress  IPv4 SPID: ", &spidIPv4Egress);

                /* Setup the ingress and egress IPv4 Security policy identifier. */
                if (Test_setupIPv4FastPath(spidIPv4Ingress, spidIPv4Egress) < 0)
                    return;
            }

            /* Setup the task parameters for the Benchmarking Task */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_benchmarkingReestablishTask, &taskParams, NULL);
            break;
        }
        case 4:
        {
            /************************************************************************
             * Frame Protocol CRC Test:
             ************************************************************************/
            if (DNUM == 0)
            {
                return;
            }

            /* Execute the Frame Protocol CRC packet tests. We dont use the default test environment */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_frameProtocolCrcTask, &taskParams, NULL);
            break;
        }
        case 5:
        {
            /************************************************************************
             * FP Reestablishment Test:
             ************************************************************************/
            if (DNUM == 0)
            {
                /* Offload the Ingress and Egress SPID: */
                Test_cliGetUint32("Ingress IPv4 SPID: ", &spidIPv4Ingress);
                Test_cliGetUint32("Egress  IPv4 SPID: ", &spidIPv4Egress);

                /* Setup the ingress and egress IPv4 Security policy identifier. */
                if (Test_setupIPv4FastPath(spidIPv4Ingress, spidIPv4Egress) < 0)
                    return;
            }

            /* Execute the Frame Protocol CRC packet tests. We dont use the default test environment */
            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_fpDrbReestablishmentTask, &taskParams, NULL);
            break;
        }
        case 6:
        {
            /************************************************************************
             * FP Reestablishment Stress Test:
             ************************************************************************/
            if (DNUM == 0)
            {
                return;
            }

            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 5;
            Task_create(Test_reestablishmentThread, &taskParams, NULL);
            break;
        }

        default:
        {
            System_printf ("Error: Invalid TEST selection [%d]\n", testSelection);
            return;
        }
    }
    return;
}


