/*
 *   @file  setup.c
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
 ************************** Local Structures **************************
 **********************************************************************/


/* IP Tunnel Type: */
typedef enum Test_IPTunnel
{
    Test_IPTunnel_4only     = 0x1,
    Test_IPTunnel_6only,
    Test_IPTunnel_4_in_4,
    Test_IPTunnel_6_in_4,
    Test_IPTunnel_6_in_6
}Test_IPTunnel;

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
extern Netfp_ClientHandle       netfpClientHandle;

/* System Configuration Handle. */
extern Resmgr_SysCfgHandle      handleSysCfg;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Global system named resource instance handle. */
extern Name_DBHandle            globalNameDatabaseHandle;

/* Global application resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* MTU receive heap: */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Task Handle */
extern Task_Handle              netfpClientTaskHandle;

/* Tasks:  */
extern void Test_statsTask(UArg arg0, UArg arg1);
extern void Test_socketTask(UArg arg0, UArg arg1);
extern void Test_ipv4ReassemblyTask(UArg arg0, UArg arg1);
extern void Test_largePacketTask(UArg arg0, UArg arg1);
extern void Test_pktTransformTask(UArg arg0, UArg arg1);
extern void Test_basicTask(UArg arg0, UArg arg1);
extern void Test_srbTask(UArg arg0, UArg arg1);
extern void Test_drbEncodeTask(UArg arg0, UArg arg1);
extern void Test_drbDecodeTask(UArg arg0, UArg arg1);
extern void Test_gtpuControlMsgTask(UArg arg0, UArg arg1);
extern void Test_fpDrbReestablishmentTask(UArg arg0, UArg arg1);
extern void Test_spDrbReestablishmentTask(UArg arg0, UArg arg1);
extern void Test_wildCardingTask(UArg arg0, UArg arg1);
extern void Test_drbSourceHOTask(UArg arg0, UArg arg1);
extern void Test_drbFPTargetHOTask(UArg arg0, UArg arg1);
extern void Test_drbSPTargetHOTask(UArg arg0, UArg arg1);
extern void Test_basicReestablishmentTask(UArg arg0, UArg arg1);
extern void Test_basicHOTask(UArg arg0, UArg arg1);
extern void Test_stressHandOver(UArg arg0, UArg arg1);
extern void Test_prioMarkTask(UArg arg0, UArg arg1);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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
                ptrNetfpConfig->eNodeBMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
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
                ptrNetfpConfig->eNodeBIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS0") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[0]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS1") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[1]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS2") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[2]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS3") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[3]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "SGW0_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                ptrNetfpConfig->secGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "SGW0_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->secGwIPAddress6) < 0)
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
                ptrNetfpConfig->secGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
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
                ptrNetfpConfig->pdnGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
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
                ptrNetfpConfig->pdnGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
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
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGwIPAddress6) < 0)
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
 *      Registered function for the reassembly heap allocation API
 *
 *  @param[in]  size
 *      Size of the reassembly buffer to be allocated
 *  @param[in]  arg
 *      Optional application specific argument
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
 *      Optional application specific argument

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
 *      Utility function which is used to display the IPSEC Key information
 *
 *  @param[in]  authMode
 *      Authentication mode
 *  @param[in]  cipherMode
 *      Ciphering Mode
 *  @param[in]  authKey
 *      Authentication key
 *  @param[in]  sizeAuthKey
 *      Size of the authentication key
 *  @param[in]  encKey
 *      Encyrption Key
 *  @param[in]  mode
 *      Size of the encryption key
 *
 *  @retval
 *      Not applicable
 */
static void Test_displayIPSecInfo
(
    Netfp_IpsecAuthMode     authMode,
    Netfp_IpsecCipherMode   cipherMode,
    char*                   authKey,
    uint8_t                 sizeAuthKey,
    char*                   encKey,
    uint8_t                 sizeEncKey
)
{
    int32_t index;

    /* Display the authentication mode */
    switch (authMode)
    {
        case Netfp_IpsecAuthMode_HMAC_SHA1:
        {
            System_printf ("HMAC-SHA1: 0x");
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_MD5:
        {
            System_printf ("HMAC-MD5: 0x");
            break;
        }
        case Netfp_IpsecAuthMode_AES_XCBC:
        {
            System_printf ("AES-XCBC: 0x");
            break;
        }
        case Netfp_IpsecAuthMode_HMAC_SHA2_256:
        {
            System_printf ("HMAC-SHA2_256: 0x");
            break;
        }
        case Netfp_IpsecAuthMode_AES_GMAC:
        {
            System_printf ("AES-GMAC: 0x");
            break;
        }
        default:
        {
            System_printf ("Error: INVALID authentication mode\n");
            return;
        }
    }

    /* Display the authentication key */
    for (index = 0; index < sizeAuthKey; index++)
        System_printf ("%x", authKey[index]);
    System_printf ("\n");

    /* Display the ciphering mode */
    switch (cipherMode)
    {
        case Netfp_IpsecCipherMode_AES_CTR:
        {
            System_printf ("AES_CTR: 0x");
            break;
        }
        case Netfp_IpsecCipherMode_AES_CBC:
        {
            System_printf ("AES_CBS: 0x");
            break;
        }
        case Netfp_IpsecCipherMode_3DES_CBC:
        {
            System_printf ("3DES-CBC: 0x");
            break;
        }
        case Netfp_IpsecCipherMode_DES_CBC:
        {
            System_printf ("DES-CBC: 0x");
            break;
        }
        case Netfp_IpsecCipherMode_AES_GCM:
        {
            System_printf ("AES-GCM: 0x");
            break;
        }
        case Netfp_IpsecCipherMode_NULL:
        {
            System_printf ("NULL\n");
            return;
        }
        default:
        {
            System_printf ("Error: INVALID ciphering mode\n");
            return;
        }
    }

    /* Display the encryption key */
    for (index = 0; index < sizeEncKey; index++)
        System_printf ("%x", encKey[index]);
    System_printf ("\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPv4 environment
 *      for the test
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4Env(char* ifName)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_IfHandle          ifHandle;
    int32_t                 errCode, i;
    Test_NetfpConfigInfo*   ptrNetfpConfig;

    /* Get the interface handle: */
    ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to find the interface '%s': Error %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Debug Message: */
	System_printf ("Debug: PDN Gateway IP Address: %d.%d.%d.%d via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
					ptrNetfpConfig->pdnGwIPAddress[0],  ptrNetfpConfig->pdnGwIPAddress[1],  ptrNetfpConfig->pdnGwIPAddress[2],
                    ptrNetfpConfig->pdnGwIPAddress[3],  ptrNetfpConfig->pdnGwMACAddress[0], ptrNetfpConfig->pdnGwMACAddress[1],
                    ptrNetfpConfig->pdnGwMACAddress[2], ptrNetfpConfig->pdnGwMACAddress[3], ptrNetfpConfig->pdnGwMACAddress[4],
                    ptrNetfpConfig->pdnGwMACAddress[5]);

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId			            = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);


    /************************************************************************************
     * Wild carding Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spidMode                = Netfp_SPIDMode_INVALID;
    inboundFPCfg.spId                    = NETFP_INVALID_SPID;
    inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = 0;
    inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-WcFastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath(netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Wild carding Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);

    /* Delete Fast path test */
    if (Netfp_deleteInboundFastPath(netfpClientHandle, fpIngressHandle, &errCode) < 0 )
    {
        System_printf ("Error: Unable to delete wildcarding Ingress fast path Error %d\n", errCode);
        return -1;
    }

    /* Add the WildCarding Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Wild carding Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
     * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
	memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
	outboundFPCfg.spId					     = NETFP_INVALID_SPID;
	outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
	outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
	outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
	outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* We use the directed route to reach the PDN gateway. */
    outboundFPCfg.ifHandle             = ifHandle;
    outboundFPCfg.nextHopMACAddress[0] = ptrNetfpConfig->pdnGwMACAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = ptrNetfpConfig->pdnGwMACAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = ptrNetfpConfig->pdnGwMACAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = ptrNetfpConfig->pdnGwMACAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = ptrNetfpConfig->pdnGwMACAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = ptrNetfpConfig->pdnGwMACAddress[5];

    /* Add the Egress Fast Path. */
	fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);


    /************************************************************************************
     * Second Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                    = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->pdnGw2IPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->pdnGw2IPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->pdnGw2IPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->pdnGw2IPAddress[3];
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->eNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->eNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->eNodeBIPAddress[3];

    /* Setup the manual next HOP MAC Address: */
    outboundFPCfg.ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, &errCode);
    outboundFPCfg.nextHopMACAddress[0] = ptrNetfpConfig->pdnGw2MACAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = ptrNetfpConfig->pdnGw2MACAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = ptrNetfpConfig->pdnGw2MACAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = ptrNetfpConfig->pdnGw2MACAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = ptrNetfpConfig->pdnGw2MACAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = ptrNetfpConfig->pdnGw2MACAddress[5];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath2");

    /* Add the Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Second Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPv6 environment for the test
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv6Env(char* ifName)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_IfHandle          ifHandle;
    int32_t                 errCode, i;
    char                    ip6String[128];
    Test_NetfpConfigInfo*   ptrNetfpConfig;

    /* Get the interface handle: */
    ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to find the interface '%s': Error %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Debug Message: */
    Netfp_convertIP6ToStr (ptrNetfpConfig->pdnGwIPAddress6, &ip6String[0]);
	System_printf ("Debug: PDN Gateway IP Address: %s via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
					ip6String, ptrNetfpConfig->pdnGwMACAddress[0], ptrNetfpConfig->pdnGwMACAddress[1],
                    ptrNetfpConfig->pdnGwMACAddress[2], ptrNetfpConfig->pdnGwMACAddress[3], ptrNetfpConfig->pdnGwMACAddress[4],
                    ptrNetfpConfig->pdnGwMACAddress[5]);

    /************************************************************************************
     * Ingress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId               = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode           = Netfp_SPIDMode_INVALID;
    inboundFPCfg.srcIP.ver          = Netfp_IPVersion_IPV6;
    inboundFPCfg.srcIP.addr.ipv6    = ptrNetfpConfig->pdnGwIPAddress6;
    inboundFPCfg.dstIP.ver          = Netfp_IPVersion_IPV6;
    inboundFPCfg.dstIP.addr.ipv6    = ptrNetfpConfig->eNodeBIPAddress6[0];
    strcpy (inboundFPCfg.name, "Ingress-IPv6-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv6 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
	 * Egress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
	memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
	outboundFPCfg.spId			       = NETFP_INVALID_SPID;
	outboundFPCfg.dstIP.ver            = Netfp_IPVersion_IPV6;
	outboundFPCfg.dstIP.addr.ipv6      = ptrNetfpConfig->pdnGwIPAddress6;
	outboundFPCfg.srcIP.ver            = Netfp_IPVersion_IPV6;
    outboundFPCfg.srcIP.addr.ipv6      = ptrNetfpConfig->eNodeBIPAddress6[0];
    outboundFPCfg.ifHandle             = ifHandle;
    outboundFPCfg.nextHopMACAddress[0] = ptrNetfpConfig->pdnGwMACAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = ptrNetfpConfig->pdnGwMACAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = ptrNetfpConfig->pdnGwMACAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = ptrNetfpConfig->pdnGwMACAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = ptrNetfpConfig->pdnGwMACAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = ptrNetfpConfig->pdnGwMACAddress[5];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv6-FastPath");

    /* Add the Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv6 Fast Path Handle 0x%p\n", fpEgressHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup an IPv4 Fast path which resides on top
 *      of an IPv4 tunnel.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4inIPv4 (char* ifName)
{
    uint32_t                spidIngress = 100;
    uint32_t                spidEgress  = 200;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    Netfp_SACfg             saConfig;
    Netfp_SAHandle          saHandle;
    Netfp_SPCfg             spConfig;
    Netfp_IfHandle          ifHandle;
    Netfp_IpsecAuthMode     authMode   = Netfp_IpsecAuthMode_HMAC_SHA1;
    Netfp_IpsecCipherMode   cipherMode = Netfp_IpsecCipherMode_3DES_CBC;
    char    authKey[] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
                          0x14,0x15,0x16,0x17,0x18 };
    char    encKey[]  = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
                          0x14,0x15,0x16,0x17,0x18 };

    /* Get the interface handle: */
    ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to find the interface '%s': Error %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPSEC Information: */
    Test_displayIPSecInfo (authMode, cipherMode, &authKey[0], sizeof(authKey), &encKey[0], sizeof(encKey));
	System_printf ("Debug: Security Gateway IP Address: %03d.%03d.%03d.%03d via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
					ptrNetfpConfig->secGwIPAddress[0],  ptrNetfpConfig->secGwIPAddress[1],  ptrNetfpConfig->secGwIPAddress[2],
                    ptrNetfpConfig->secGwIPAddress[3],  ptrNetfpConfig->secGwMACAddress[0], ptrNetfpConfig->secGwMACAddress[1],
                    ptrNetfpConfig->secGwMACAddress[2], ptrNetfpConfig->secGwMACAddress[3], ptrNetfpConfig->secGwMACAddress[4],
                    ptrNetfpConfig->secGwMACAddress[5]);

    /************************************************************************************
     * Ingress security association:
     ************************************************************************************/

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_INBOUND;
    saConfig.spi                    = 200;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = authMode;
    saConfig.ipsecCfg.encMode       = cipherMode;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;    
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

    /* Setup the IP address associated with the security associations. */
    saConfig.srcIP.ver                         = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0]           = ptrNetfpConfig->secGwIPAddress[0];
    saConfig.srcIP.addr.ipv4.u.a8[1]           = ptrNetfpConfig->secGwIPAddress[1];
    saConfig.srcIP.addr.ipv4.u.a8[2]           = ptrNetfpConfig->secGwIPAddress[2];
    saConfig.srcIP.addr.ipv4.u.a8[3]           = ptrNetfpConfig->secGwIPAddress[3];
    saConfig.dstIP.ver                         = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0]           = ptrNetfpConfig->eNodeBIPAddress[0];
    saConfig.dstIP.addr.ipv4.u.a8[1]           = ptrNetfpConfig->eNodeBIPAddress[1];
    saConfig.dstIP.addr.ipv4.u.a8[2]           = ptrNetfpConfig->eNodeBIPAddress[2];
    saConfig.dstIP.addr.ipv4.u.a8[3]           = ptrNetfpConfig->eNodeBIPAddress[3];

    /* Create the SA. */
    saHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (saHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress SA %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress SA %x created successfully\n", saHandle);

    /************************************************************************************
     * Ingress security policy:
     ************************************************************************************/

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_INBOUND;
    spConfig.saHandle                = saHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->pdnGwIPAddress[0];
    spConfig.srcIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->pdnGwIPAddress[1];
    spConfig.srcIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->pdnGwIPAddress[2];
    spConfig.srcIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->pdnGwIPAddress[3];
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->eNodeBIPAddress[0];
    spConfig.dstIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->eNodeBIPAddress[1];
    spConfig.dstIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->eNodeBIPAddress[2];
    spConfig.dstIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->eNodeBIPAddress[3];
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spidIngress;

    /* Add the Ingress Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        System_printf ("Error: Unable to create Ingress SP Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress SP %d created successfully\n", spConfig.spId);

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId			            = spidIngress;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);


    /************************************************************************************
     * Egress security association:
     ************************************************************************************/

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_OUTBOUND;
    saConfig.spi                    = 100;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = authMode;
    saConfig.ipsecCfg.encMode       = cipherMode;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;
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

    /* Setup the IP address associated with the security associations. */
    saConfig.srcIP.ver                         = Netfp_IPVersion_IPV4;
    saConfig.srcIP.addr.ipv4.u.a8[0]           = ptrNetfpConfig->eNodeBIPAddress[0];
    saConfig.srcIP.addr.ipv4.u.a8[1]           = ptrNetfpConfig->eNodeBIPAddress[1];
    saConfig.srcIP.addr.ipv4.u.a8[2]           = ptrNetfpConfig->eNodeBIPAddress[2];
    saConfig.srcIP.addr.ipv4.u.a8[3]           = ptrNetfpConfig->eNodeBIPAddress[3];
    saConfig.dstIP.ver                         = Netfp_IPVersion_IPV4;
    saConfig.dstIP.addr.ipv4.u.a8[0]           = ptrNetfpConfig->secGwIPAddress[0];
    saConfig.dstIP.addr.ipv4.u.a8[1]           = ptrNetfpConfig->secGwIPAddress[1];
    saConfig.dstIP.addr.ipv4.u.a8[2]           = ptrNetfpConfig->secGwIPAddress[2];
    saConfig.dstIP.addr.ipv4.u.a8[3]           = ptrNetfpConfig->secGwIPAddress[3];

    /* Setup the next hop MAC address information: */
    saConfig.ifHandle               = ifHandle;
    saConfig.nextHopMACAddress[0]   = ptrNetfpConfig->secGwMACAddress[0];
    saConfig.nextHopMACAddress[1]   = ptrNetfpConfig->secGwMACAddress[1];
    saConfig.nextHopMACAddress[2]   = ptrNetfpConfig->secGwMACAddress[2];
    saConfig.nextHopMACAddress[3]   = ptrNetfpConfig->secGwMACAddress[3];
    saConfig.nextHopMACAddress[4]   = ptrNetfpConfig->secGwMACAddress[4];
    saConfig.nextHopMACAddress[5]   = ptrNetfpConfig->secGwMACAddress[5];

    /* Create the SA. */
    saHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (saHandle == NULL)
    {
        System_printf ("Error: Unable to create egress SA %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress SA %x created successfully\n", saHandle);

    /************************************************************************************
     * Egress security policy:
     ************************************************************************************/

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_OUTBOUND;
    spConfig.saHandle                = saHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.srcIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->eNodeBIPAddress[0];
    spConfig.srcIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->eNodeBIPAddress[1];
    spConfig.srcIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->eNodeBIPAddress[2];
    spConfig.srcIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->eNodeBIPAddress[3];
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
    spConfig.dstIP.addr.ipv4.u.a8[0] = ptrNetfpConfig->pdnGwIPAddress[0];
    spConfig.dstIP.addr.ipv4.u.a8[1] = ptrNetfpConfig->pdnGwIPAddress[1];
    spConfig.dstIP.addr.ipv4.u.a8[2] = ptrNetfpConfig->pdnGwIPAddress[2];
    spConfig.dstIP.addr.ipv4.u.a8[3] = ptrNetfpConfig->pdnGwIPAddress[3];
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spidEgress;

    /* Add the egress security policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        System_printf ("Error: Unable to create egress SP Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress SP %d created successfully\n", spConfig.spId);

    /************************************************************************************
     * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
	memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
	outboundFPCfg.spId					     = spidEgress;
	outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
	outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
	outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
	outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* Add the Egress Fast Path. */
	fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup an IPv6 Fast path which resides on top
 *      of an IPv6 tunnel.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv6inIPv6 (char* ifName)
{
    uint32_t                spidIngress = 100;
    uint32_t                spidEgress  = 200;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    int32_t                 errCode, i;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    Netfp_SACfg             saConfig;
    Netfp_SAHandle          saHandle;
    Netfp_SPCfg             spConfig;
    char    authKey[] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
                          0x14,0x15,0x16,0x17,0x18 };
    char    encKey[]  = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
                          0x14,0x15,0x16,0x17,0x18 };
    char                    ip6String[128];
    Netfp_IfHandle          ifHandle;
    Netfp_IpsecAuthMode     authMode   = Netfp_IpsecAuthMode_HMAC_SHA1;
    Netfp_IpsecCipherMode   cipherMode = Netfp_IpsecCipherMode_AES_CTR;

    /* Get the interface handle: */
    ifHandle = Netfp_findInterface (netfpClientHandle, ifName, NULL, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to find the interface '%s': Error %d\n", errCode);
        return -1;
    }

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPSEC Information: */
    Test_displayIPSecInfo (authMode, cipherMode, &authKey[0], sizeof(authKey), &encKey[0], sizeof(encKey));
    Netfp_convertIP6ToStr (ptrNetfpConfig->secGwIPAddress6, &ip6String[0]);
	System_printf ("Debug: Security Gateway IP Address: %s via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
					ip6String, ptrNetfpConfig->secGwMACAddress[0], ptrNetfpConfig->secGwMACAddress[1], ptrNetfpConfig->secGwMACAddress[2],
                    ptrNetfpConfig->secGwMACAddress[3], ptrNetfpConfig->secGwMACAddress[4], ptrNetfpConfig->secGwMACAddress[5]);

    /************************************************************************************
     * Ingress security association:
     ************************************************************************************/

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_INBOUND;
    saConfig.spi                    = 200;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = authMode;
    saConfig.ipsecCfg.encMode       = cipherMode;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;
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

    /* Setup the IP address associated with the security associations. */
    saConfig.srcIP.ver          = Netfp_IPVersion_IPV6;
    saConfig.srcIP.addr.ipv6    = ptrNetfpConfig->secGwIPAddress6;
    saConfig.dstIP.ver          = Netfp_IPVersion_IPV6;
    saConfig.dstIP.addr.ipv6    = ptrNetfpConfig->eNodeBIPAddress6[0];

    /* Create the SA. */
    saHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (saHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress SA %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress SA %x created successfully\n", saHandle);

    /************************************************************************************
     * Ingress security policy:
     ************************************************************************************/

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_INBOUND;
    spConfig.saHandle                = saHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.srcIP.addr.ipv6         = ptrNetfpConfig->pdnGwIPAddress6;
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.dstIP.addr.ipv6         = ptrNetfpConfig->eNodeBIPAddress6[0];
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spidIngress;

    /* Add the Ingress Security Policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        System_printf ("Error: Unable to create Ingress SP Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress SP %d created successfully\n", spConfig.spId);

    /************************************************************************************
     * Ingress IPv6 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId		                = spidIngress;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_SPECIFIC;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPCfg.srcIP.addr.ipv6            = ptrNetfpConfig->pdnGwIPAddress6;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV6;
    inboundFPCfg.dstIP.addr.ipv6            = ptrNetfpConfig->eNodeBIPAddress6[0];
    strcpy (inboundFPCfg.name, "Ingress-IPv6-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv6 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
     * Egress security association:
     ************************************************************************************/

    /* Initialize the SA Configuration. */
    memset ((void *)&saConfig, 0, sizeof (Netfp_SACfg));

    /* Populate the SA Configuration. */
    saConfig.direction              = Netfp_Direction_OUTBOUND;
    saConfig.spi                    = 100;
    saConfig.ipsecCfg.protocol      = Netfp_IPSecProto_IPSEC_ESP;
    saConfig.ipsecCfg.mode          = Netfp_IPSecMode_IPSEC_TUNNEL;
    saConfig.ipsecCfg.authMode      = authMode;
    saConfig.ipsecCfg.encMode       = cipherMode;
    saConfig.ipsecCfg.keyMacSize    = 12;

    if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_MD5)
        saConfig.ipsecCfg.keyAuthSize   = 16;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA1)
        saConfig.ipsecCfg.keyAuthSize   = 20;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_AES_XCBC)
        saConfig.ipsecCfg.keyAuthSize   = 12;
    else if (saConfig.ipsecCfg.authMode == Netfp_IpsecAuthMode_HMAC_SHA2_256)
        saConfig.ipsecCfg.keyAuthSize   = 30;
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

    /* Setup the IP address associated with the security associations. */
    saConfig.srcIP.ver          = Netfp_IPVersion_IPV6;
    saConfig.srcIP.addr.ipv6    = ptrNetfpConfig->eNodeBIPAddress6[0];
    saConfig.dstIP.ver          = Netfp_IPVersion_IPV6;
    saConfig.dstIP.addr.ipv6    = ptrNetfpConfig->secGwIPAddress6;

    /* Setup the next hop MAC address information: */
    saConfig.ifHandle               = ifHandle;
    saConfig.nextHopMACAddress[0]   = ptrNetfpConfig->secGwMACAddress[0];
    saConfig.nextHopMACAddress[1]   = ptrNetfpConfig->secGwMACAddress[1];
    saConfig.nextHopMACAddress[2]   = ptrNetfpConfig->secGwMACAddress[2];
    saConfig.nextHopMACAddress[3]   = ptrNetfpConfig->secGwMACAddress[3];
    saConfig.nextHopMACAddress[4]   = ptrNetfpConfig->secGwMACAddress[4];
    saConfig.nextHopMACAddress[5]   = ptrNetfpConfig->secGwMACAddress[5];

    /* Create the SA. */
    saHandle = Netfp_addSA (netfpClientHandle, &saConfig, &errCode);
    if (saHandle == NULL)
    {
        System_printf ("Error: Unable to create egress SA %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress SA %x created successfully\n", saHandle);

    /************************************************************************************
     * Egress security policy:
     ************************************************************************************/

    /* Initialize the SP Configuration. */
    memset ((void *)&spConfig, 0, sizeof (Netfp_SPCfg));

    /* Populate the SP Configuration */
    spConfig.direction               = Netfp_Direction_OUTBOUND;
    spConfig.saHandle                = saHandle;
    spConfig.srcIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.srcIP.addr.ipv6         = ptrNetfpConfig->eNodeBIPAddress6[0];
    spConfig.dstIP.ver               = Netfp_IPVersion_IPV6;
    spConfig.dstIP.addr.ipv6         = ptrNetfpConfig->pdnGwIPAddress6;
    spConfig.srcIPPrefixLen          = 24;
    spConfig.dstIPPrefixLen          = 24;
    spConfig.protocol                = 0x32;
    spConfig.srcPortStart            = 0;
    spConfig.srcPortEnd              = 0;
    spConfig.gtpuIdStart             = 0x0;
    spConfig.gtpuIdEnd               = 0x0;
    spConfig.dscpFilter              = 0;
    spConfig.spId                    = spidEgress;

    /* Add the egress security policy. */
    if (Netfp_addSP (netfpClientHandle, &spConfig, &errCode) < 0)
    {
        System_printf ("Error: Unable to create egress SP Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress SP %d created successfully\n", spConfig.spId);

    /************************************************************************************
	 * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
	outboundFPCfg.spId					     = spidEgress;
	outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV6;
	outboundFPCfg.dstIP.addr.ipv6            = ptrNetfpConfig->pdnGwIPAddress6;
	outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV6;
	outboundFPCfg.srcIP.addr.ipv6            = ptrNetfpConfig->eNodeBIPAddress6[0];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv6-FastPath");

    /* Add the Egress Fast Path. */
	fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv6 Fast Path Handle 0x%p\n", fpEgressHandle);
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
 *      Optional application defined argument
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
 *      The function is used to setup the default test environment. The parameters for the test
 *      environment are parsed from the netfp.dat file. In most test cases the default test
 *      environment should be suffient.
 *
 *  @param[in]  tunnelType
 *      Tunnel Type which needs to be created
 *
 *  @retval
 *      Not Applicable.
 */
int32_t Test_setupDefaultEnv (Test_IPTunnel tunnelType)
{
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceCfg      ifConfig;
    Netfp_InterfaceIP       globalIP;
    Netfp_OptionTLV         optCfg;
    int32_t                 errCode;

    /* Create the interface: The interface is common to IPv4 or IPv6; so we create this
     * upfront. Initialize the configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = netfpConfig.eNodeBIPAddress[0];
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = netfpConfig.eNodeBIPAddress[1];
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = netfpConfig.eNodeBIPAddress[2];
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = netfpConfig.eNodeBIPAddress[3];
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth0");

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = netfpConfig.eNodeBMACAddress[0];
    ifConfig.macAddress[1] = netfpConfig.eNodeBMACAddress[1];
    ifConfig.macAddress[2] = netfpConfig.eNodeBMACAddress[2];
    ifConfig.macAddress[3] = netfpConfig.eNodeBMACAddress[3];
    ifConfig.macAddress[4] = netfpConfig.eNodeBMACAddress[4];
    ifConfig.macAddress[5] = netfpConfig.eNodeBMACAddress[5];

    /* Create the interface */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to create the interface [Error Code %d]\n", errCode);
        return -1;
    }

    /* Setup the IPv6 address in the interface.  */
    globalIP.ipAddress.ver  = Netfp_IPVersion_IPV6;
    globalIP.subnetMask.ver = Netfp_IPVersion_IPV6;

    /* Assign the IPv6 address from the NETFP configuration */
    globalIP.ipAddress.addr.ipv6 = netfpConfig.eNodeBIPAddress6[0];

    /* Subnet Mask is assumed to be 64 bits */
    globalIP.subnetMask.addr.ipv6.u.a32[0] = 0xFFFFFFFF;
    globalIP.subnetMask.addr.ipv6.u.a32[1] = 0xFFFFFFFF;
    globalIP.subnetMask.addr.ipv6.u.a32[2] = 0x0;
    globalIP.subnetMask.addr.ipv6.u.a32[3] = 0x0;

    /* Add the IPv6 address to the interface. */
    optCfg.type     =   Netfp_Option_ADD_IP;
    optCfg.length   =   sizeof(Netfp_InterfaceIP);
    optCfg.value    =   (void*)&globalIP;

    if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to add the global IPv6 address [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the test setup as requested */
    switch (tunnelType)
    {
        case Test_IPTunnel_4only:
        {
            errCode = Test_setupIPv4Env(&ifConfig.name[0]);
            break;
        }
        case Test_IPTunnel_6only:
        {
            errCode = Test_setupIPv6Env(&ifConfig.name[0]);
            break;
        }
        case Test_IPTunnel_4_in_4:
        {
            errCode = Test_setupIPv4inIPv4(&ifConfig.name[0]);
            break;
        }
        case Test_IPTunnel_6_in_6:
        {
            errCode = Test_setupIPv6inIPv6(&ifConfig.name[0]);
            break;
        }
        default:
        {
            System_printf ("Error: Unsupported tunnel option %d\n", tunnelType);
            return -1;
        }
    }

    /* Were we able to setup the test environment? */
    if (errCode < 0)
        return -1;

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
    int32_t                             size;
    FILE*                               configFileHandle;
    Task_Params                         taskParams;
    int32_t                             errCode;
    Msgcom_ChannelCfg                   chConfig;
    Netfp_ReassemblyConfig              reassemblyConfig;
    Resmgr_ResourceCfg*                 ptrCfg;
    Name_ResourceCfg                    namedResCfg;
    MsgCom_ChHandle                     outerIPChannel;
    MsgCom_ChHandle                     innerIPChannel;
    Clock_Handle                        reassemblyTimer;
    Clock_Params                        clockParams;
    Netfp_DefaultReassemblyMgmtCfg      defaultReassemblyCfg;
    Test_NetfpConfigInfo*               ptrNetfpConfig;
    char                                ip6String[128];

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Get the application configuration. */
    ptrCfg = &appResourceConfig;

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
    chConfig.mode                      = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack               = NULL;
    chConfig.msgcomInstHandle          = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode  = Msgcom_QueueInterruptMode_NO_INTERRUPT;

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
     *    are always found.
     *  - Default NETFP supplied reassembly management. */
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

    /* Keep a copy of the eNB MAC Address (Its useful for other tests) */
    eNBMacAddress[0] = netfpConfig.eNodeBMACAddress[0];
    eNBMacAddress[1] = netfpConfig.eNodeBMACAddress[1];
    eNBMacAddress[2] = netfpConfig.eNodeBMACAddress[2];
    eNBMacAddress[3] = netfpConfig.eNodeBMACAddress[3];
    eNBMacAddress[4] = netfpConfig.eNodeBMACAddress[4];
    eNBMacAddress[5] = netfpConfig.eNodeBMACAddress[5];

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

    /* Create the statistics monitoring task: This will always run in the background.
     * To get statistics for a particular core; select the core in CCS and use the
     * set the global variable gDisplayStats to 1; this will dump the contents on the
     * CCS console. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 5;
    Task_create(Test_statsTask, &taskParams, NULL);

    /* Create the reassembly task which is responsible for handling the reassembly
     * of all received fragments. This task is created since the NETFP Client has
     * been registered for reassembly. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 2;
    Task_create(Test_reassemblyTask, &taskParams, NULL);

    /* Display the setup configuration: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Test Environment [Configured via C:/netfp.dat]:\n");
    System_printf ("My IPv4 Address             : %d.%d.%d.%d\n",
                    ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
                    ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    Netfp_convertIP6ToStr (ptrNetfpConfig->eNodeBIPAddress6[0], &ip6String[0]);
    System_printf ("My IPv6 Address             : %s\n", ip6String);
    System_printf ("PDN IPv4 Address            : %d.%d.%d.%d\n",
                    ptrNetfpConfig->pdnGwIPAddress[0], ptrNetfpConfig->pdnGwIPAddress[1],
                    ptrNetfpConfig->pdnGwIPAddress[2], ptrNetfpConfig->pdnGwIPAddress[3]);
    Netfp_convertIP6ToStr (ptrNetfpConfig->pdnGwIPAddress6, &ip6String[0]);
    System_printf ("PDN IPv6 Address            : %s\n", ip6String);
    System_printf ("SeGW IPv4 Address           : %d.%d.%d.%d\n",
                    ptrNetfpConfig->secGwIPAddress[0], ptrNetfpConfig->secGwIPAddress[1],
                    ptrNetfpConfig->secGwIPAddress[2], ptrNetfpConfig->secGwIPAddress[3]);
    Netfp_convertIP6ToStr (ptrNetfpConfig->secGwIPAddress6, &ip6String[0]);
    System_printf ("SeGW IPv6 Address           : %s\n", ip6String);
    System_printf ("-------------------------------------------------------------\n");

    /***********************************************************************
     * CLI Simulation:
     ***********************************************************************/
    while (1)
    {
        Task_sleep (20);
        System_printf ("*******************************************************\n");
        System_printf ("DSP NETFP Unit Test Menu                    \n");
        System_printf ("Please select the type of test to execute:  \n");
        System_printf ("1. Socket                                   \n");
        System_printf ("2. Reassembly IPv4                          \n");
        System_printf ("3. Large Packet                             \n");
        System_printf ("4. Non Secure Socket IPv4 Packet Transform  \n");
        System_printf ("5. Basic Testing                            \n");
        System_printf ("6. SRB Testing                              \n");
        System_printf ("7. DRB (Encode) Testing                     \n");
        System_printf ("8. GTPU Control Message                     \n");
        System_printf ("9. Secure 4-4 Socket (Hardcoded keys)       \n");
        System_printf ("10. Delete Client                           \n");
        System_printf ("11. DRB (Decode) Testing                    \n");
        System_printf ("12. Non Secure IPv6 Socket Test  Transform  \n");
        System_printf ("13. Secure 6-6 Socket Test                  \n");
        System_printf ("14. Fast Path Re-establishment Testing      \n");
        System_printf ("15. Slow Path Re-establishment Testing      \n");
        System_printf ("16. Wild carding Testing                    \n");
        System_printf ("17. Source Hand Over Testing                \n");
        System_printf ("18. Fast Path Target Hand Over Testing      \n");
        System_printf ("19. Slow Path Target Hand Over Testing      \n");
        System_printf ("20. Basic Reestablishment Testing           \n");
        System_printf ("21. Basic Handover Testing                  \n");
        System_printf ("22. Handover Stress Testing                 \n");
        System_printf ("23. Priority Marking                        \n");
        System_printf ("*******************************************************\n");
        System_printf ("> Enter your selection: ");

        /* Wait for the user input. */
        scanf ("%d", &testSelection);

        /* Validate the selection: */
        if ((testSelection >= 1) && (testSelection <= 23))
            break;
    }

    /* Create a named resource and indicate to the other core the test which is being executed. */
    memset ((void *)&namedResCfg, 0, sizeof(Name_ResourceCfg));
    namedResCfg.handle1  = (uint32_t)testSelection;
    strcpy(namedResCfg.name, "TestSelection");
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, &namedResCfg, &errCode) < 0)
        System_printf ("Error: Creating Named resource failed with error code %d\n", errCode);

    /* Setup the task parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 5;

    /* Launch the appropriate test task on the basis of the selection:
     * NOTE: The value of the test selection here and in the statistics task
     * need to be kept in SYNC. Else there will be incorrect displays */
    switch (testSelection)
    {
        case 1:
        {
            /* Execute the socket tests: We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_socketTask, &taskParams, NULL);
            break;
        }
        case 2:
        {
            /* Execute the IPv4 reassembly tests. We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_ipv4ReassemblyTask, &taskParams, NULL);
            break;
        }
        case 3:
        {
            /* Execute the Large packet tests. We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_largePacketTask, &taskParams, NULL);
            break;
        }
        case 4:
        {
            /* Execute the Packet Transform tests. We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);

            /* Set the task argument to indicate that the test is executing on IPv4. */
            taskParams.arg0 = 1;
            Task_create(Test_pktTransformTask, &taskParams, NULL);
            break;
        }
        case 5:
        {
            /* Execute the Basic API tests. */
            Task_create(Test_basicTask, &taskParams, NULL);
            break;
        }
        case 6:
        {
            /* Execute the SRB tests. */
            Task_create(Test_srbTask, &taskParams, NULL);
            break;
        }
        case 7:
        {
            /* Execute the DRB encoding tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_drbEncodeTask, &taskParams, NULL);
            break;
        }
        case 8:
        {
            /* Execute the GTPU control message tests. We dont use the default test environment */
            Task_create(Test_gtpuControlMsgTask, &taskParams, NULL);
            break;
        }
        case 9:
        {
            /* Execute the socket tests: We use the secure default environment */
            Test_setupDefaultEnv (Test_IPTunnel_4_in_4);
            Task_create(Test_socketTask, &taskParams, NULL);
            break;
        }
        case 10:
        {
            /* Stop the NETFP Client */
            if (Netfp_stopClient (netfpClientHandle, &errCode) < 0)
                System_printf ("Error: NETFP client stop failed [Error code %d]\n", errCode);
            else
                System_printf ("Debug: NETFP client stopped successfully\n");

            /* Delete the NETFP Client Execution Task */
            Task_delete (&netfpClientTaskHandle);

            /* Delete the NETFP client. */
            if (Netfp_deleteClient (netfpClientHandle, &errCode) < 0)
                System_printf ("Error: NETFP client deletion failed [Error code %d]\n", errCode);
            else
                System_printf ("Debug: NETFP client deleted successfully\n");
            break;
        }
        case 11:
        {
            /* Execute the DRB decoding tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_drbDecodeTask, &taskParams, NULL);
            break;
        }
        case 12:
        {
            /* Execute the Socket Packet Transform tests in normal IPv6 mode */
            Test_setupDefaultEnv (Test_IPTunnel_6only);

            /* Set the task argument to indicate that the test is executing on IPv6. */
            taskParams.arg0 = 0;
            Task_create(Test_pktTransformTask, &taskParams, NULL);
            break;
        }
        case 13:
        {
            /* Setup the test environment to create a 6 in 6 tunnel and execute the IPv6 tests */
            Test_setupDefaultEnv (Test_IPTunnel_6_in_6);

            /* Set the task argument to indicate that the test is executing on IPv6. */
            taskParams.arg0 = 0;
            Task_create(Test_pktTransformTask, &taskParams, NULL);
            break;
        }
        case 14:
        {
            /* Execute the fast path DRB Re-establishment tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_fpDrbReestablishmentTask, &taskParams, NULL);
            break;
        }
        case 15:
        {
            /* Execute the slow path DRB Re-establishment tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_spDrbReestablishmentTask, &taskParams, NULL);
            break;
        }

        case 16:
        {
            /* Execute the socket tests: We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_wildCardingTask, &taskParams, NULL);
            break;
        }
        case 17:
        {
            /* Execute the source HandOver tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_drbSourceHOTask, &taskParams, NULL);
            break;
        }
        case 18:
        {
            /* Execute the fast path target HandOver tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_drbFPTargetHOTask, &taskParams, NULL);
            break;
        }
        case 19:
        {
            /* Execute the slow path target HandOver tests. */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_drbSPTargetHOTask, &taskParams, NULL);
            break;
        }
        case 20:
        {
            /* Execute the Basic Reestablishment Test: */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_basicReestablishmentTask, &taskParams, NULL);
            break;
        }
        case 21:
        {
            /* Execute the Basic Handover Test: */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_basicHOTask, &taskParams, NULL);
            break;
        }
        case 22:
        {
            /* Execute the Handover Stress Test: */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_stressHandOver, &taskParams, NULL);
            break;
        }
        case 23:
        {
            /* Execute the socket tests: We use the default test environment */
            Test_setupDefaultEnv (Test_IPTunnel_4only);
            Task_create(Test_prioMarkTask, &taskParams, NULL);
            break;
        }
        default:
        {
            System_printf ("Error: Undefined selection %d; aborting test\n", testSelection);
            break;
        }
    }
    return;
}

