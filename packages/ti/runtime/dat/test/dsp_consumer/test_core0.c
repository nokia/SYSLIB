/**
 *   @file  test_core0.c
 *
 *   @brief
 *      Debug & Trace Unit Test Core0 Code.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/name/name_db.h>

/* Logger streamer Include Files */
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

#include "../netCfg.h"

/**********************************************************************
 ************************** Unit Test Externs *************************
 **********************************************************************/

/* Global SYSLIB Module Instance Handles: */
extern Pktlib_InstHandle    appPktlibInstanceHandle;
extern Msgcom_InstHandle    appMsgcomInstanceHandle;
extern Netfp_ClientHandle   netfpClientHandle;
extern Name_ClientHandle    nameClientHandle;
extern Name_DBHandle        globalNameDatabaseHandle;

/* Heaps */
extern Pktlib_HeapHandle    datClientHeap;
extern Pktlib_HeapHandle    datConsumerHeap;

/* Cache Invalidate API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/* DAT: */
extern void* Dat_osalMalloc (uint32_t , uint32_t );
extern void  Dat_osalFree (void* , uint32_t );
extern void* Dat_osalEnterSingleCoreCriticalSection (void);
extern void  Dat_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Dat_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Dat_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Dat_osalCreateSem(void);
extern void  Dat_osalDeleteSem(void*);
extern void  Dat_osalPostSem(void*);
extern void  Dat_osalPendSem(void*);

/**********************************************************************
 ************************** Unit Test Globals *************************
 **********************************************************************/

/* Producer name: This is a well known producer name across the test domain */
static const char*      gProducerName = "Test-UIA";

/* Statistics: Counter which keeps track of the number of messages received on the
 * consumer */
uint32_t                consumerMessage = 0;

/* Buffer used to store NETFP configuration file */
char                    netfpConfigBuffer[2500];

/* NETFP Configuration after parsing the DAT file. */
Test_NetfpConfigInfo    netfpConfig;

/* Display the DAT Server */
uint32_t                gDebugDatServer = 0;

/* DAT Server Handle: */
Dat_ServerHandle        datServerHandle;

/* DAT Client Handle: */
Dat_ClientHandle        datClientHandle;

/* DAT Consumer Socket Handle */
Netfp_SockHandle        datSockHandle;

/* DAT Server Name: */
const char*             gDatServerName   = "DatServer_LTE9A";

/* DAT Consumer */
Dat_ConsHandle          datConsumerHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      This task to consumer buffers for all created consumers.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_consumerTask(UArg arg0, UArg arg1)
{
    Ti_Pkt*     ptrMessage;

    /* This task is created after consumer is created. */
    while(datConsumerHandle)
    {
        /* Poll message for consumer */
		do{
            ptrMessage = Dat_processConsumer(datConsumerHandle);
    		if(ptrMessage)
    		{
                /* Increment the number of messages received by the consumer. */
                consumerMessage++;
                Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
    		}
		}while(ptrMessage);

		/* Poll consumer every 2ms */
		Task_sleep(2);
    }

    return;
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
        else if (strcmp (tokenName, "RNC_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGwIPAddress6) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "PDN_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGwIPAddress6) < 0)
                return -1;
        }
        else
        {
            /* Invalid Token Name. */
            //return -1;
        }
    }

    /* File has been successfully parsed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered callback function with the DAT server to log messages
 *
 *  @retval
 *      Not Applicable.
 */
static void Dat_ServerLogFunction(Dat_LogLevel logLevel, char* fmt, va_list arg)
{
    VaList ap;

    va_start(ap, fmt);
    System_vprintf(fmt, ap);
    va_end(ap);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the DAT Server
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_datServer(UArg arg0, UArg arg1)
{
    int32_t datClientIndex;
    int32_t datClientRegisterationStatus[2];
    int32_t errCode;
    char*   datClientNames[] =
    {
        "Dat_Client_LTE9A_L2DP",
        "Dat_Client_LTE9A_L1",
        NULL
    };

    /* Initialize the client registeration status */
    for (datClientIndex = 0; datClientIndex < 2; datClientIndex++)
        datClientRegisterationStatus[datClientIndex] = 0;

    /* Execute the DAT Server which executes in a polled mode */
    while (1)
    {
        /* Cycle through and register all the clients. */
        for (datClientIndex = 0; datClientIndex < 2; datClientIndex++)
        {
            /* Register the clients. */
            if (datClientRegisterationStatus[datClientIndex] == 0)
            {
                /* Try and register the DAT client */
                if (Dat_registerClient (datServerHandle, datClientNames[datClientIndex],  &errCode) == 0)
                {
                    /* DAT client registered successsfully */
                    System_printf ("Debug: DAT Client %s registered successfully\n", datClientNames[datClientIndex]);
                    datClientRegisterationStatus[datClientIndex] = 1;
                }
                else
                {
                    /* DAT client was not registered */
                    if (errCode == DAT_ENOTREADY)
                    {
                        /* DAT client was not ready so try again later */
                        break;
                    }
                    else
                    {
                        /* DAT client registeration failed */
                        System_printf ("Error: DAT Client registeration failed [Error code %d]\n", errCode);
                        break;
                    }
                }
            }
        }

        /* Execute the DAT server */
        Dat_executeServer (datServerHandle);

        /* Debug Service: Display the DAT Server details if requested. */
        if (gDebugDatServer == 1)
        {
            Dat_displayServer (datServerHandle);
            gDebugDatServer = 0;
        }

        /* Relinuqish time slice */
        Task_sleep(10);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the NETFP Client
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_datClient(UArg arg0, UArg arg1)
{
    /* Execute the DAT Client: This is executed in a polled mode since the
     * DAT clients execute in the control path. */
    while (1)
    {
        Dat_executeClient (datClientHandle);
        Dat_executeBackgroundActions (datClientHandle);
        Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the networking environment.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Test_netEnvironmentSetup(void)
{
    int32_t                 size;
    FILE*                   configFileHandle;
    int32_t                 errCode;
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceCfg      ifConfig;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    int32_t                 i;

    /* Open the NETFP configuration file for the tests */
    configFileHandle = fopen ("c:\\netfp.dat", "r");
    if (configFileHandle == NULL)
    {
        System_printf ("Error: Unable to open NETFP configuration file (c:\\netfp.dat)\n");
        return -1;
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
            return -1;
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
        return -1;
    }

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
    strcpy (ifConfig.name, "eth0.100");

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

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPv4 Setup Configuration: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Default IPv4 Test Environment Setup:\n");
    System_printf ("NETFP Client Handle     : %p\n", netfpClientHandle);
    System_printf ("My IP Address           : %d.%d.%d.%d\n",
                    ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
                    ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    System_printf ("Remote IP Address       : %d.%d.%d.%d via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                    ptrNetfpConfig->pdnGwIPAddress[0], ptrNetfpConfig->pdnGwIPAddress[1],
                    ptrNetfpConfig->pdnGwIPAddress[2], ptrNetfpConfig->pdnGwIPAddress[3],
                    ptrNetfpConfig->pdnGwMACAddress[0], ptrNetfpConfig->pdnGwMACAddress[1],
                    ptrNetfpConfig->pdnGwMACAddress[2], ptrNetfpConfig->pdnGwMACAddress[3],
                    ptrNetfpConfig->pdnGwMACAddress[4], ptrNetfpConfig->pdnGwMACAddress[5]);
    System_printf ("-------------------------------------------------------------\n");

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
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
     * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = NETFP_INVALID_SPID;
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

    /* Setup the manual route: */
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
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_core0Task(UArg arg0, UArg arg1)
{
    Dat_ConsHandle          consumerHandle;
    Dat_ConsumerCfg         consumerCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockAddr          sockAddress;
    int32_t                 errCode;
    Dat_ServerCfg           datServerCfg;
    Dat_ClientCfg           datClientCfg;
    Task_Params             taskParams;
    int32_t                 clientStatus;

    /* Initialize the DAT server configuration */
    memset ((void *)&datServerCfg, 0, sizeof(Dat_ServerCfg));

    /* Populate the DAT Server configuration
     * - Named resource instance identifier is set to 1; the same as named resource domain initialized above */
    strcpy (datServerCfg.serverName, gDatServerName);
    datServerCfg.nrInstanceId           = 0x1;
    datServerCfg.pktlibInstHandle       = appPktlibInstanceHandle;
    datServerCfg.msgcomInstHandle       = appMsgcomInstanceHandle;
    datServerCfg.realm                  = Dat_ExecutionRealm_DSP;
    datServerCfg.logFxn                 = Dat_ServerLogFunction;
    datServerCfg.serverHeapHandle       = datClientHeap;
    datServerCfg.malloc                 = Dat_osalMalloc;
    datServerCfg.free                   = Dat_osalFree;
    datServerCfg.beginMemAccess         = Dat_osalBeginMemoryAccess;
    datServerCfg.endMemAccess           = Dat_osalEndMemoryAccess;
    datServerCfg.createSem              = Dat_osalCreateSem;
    datServerCfg.deleteSem              = Dat_osalDeleteSem;
    datServerCfg.postSem                = Dat_osalPostSem;
    datServerCfg.pendSem                = Dat_osalPendSem;
    datServerCfg.enterCS                = Dat_osalEnterSingleCoreCriticalSection;
    datServerCfg.exitCS                 = Dat_osalExitSingleCoreCriticalSection;

    /* Initialize the DAT Server */
    datServerHandle = Dat_initServer (&datServerCfg, &errCode);
    if (datServerHandle == NULL)
    {
        System_printf ("Error: Unable to create the DAT server [Error code %d]\n", errCode);
        return;
    }

    /* Launch the DAT Server Execution Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 3;
    Task_create(Test_datServer, &taskParams, NULL);

    /* SYNC Point: Ensure that the DAT Server has been started. This is required before
     * we create and register the DAT client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready; since the DAT server and
         * client are executing in the same realm we dont need to specify the NAME client handle. */
        if (Dat_startServer (globalNameDatabaseHandle, NULL, (char*)gDatServerName, &errCode) != NULL)
            break;

        /* Check the error code. */
        if (errCode != DAT_ENOTREADY)
        {
            System_printf ("Error: DAT Starting server failed [Error code %d]\n", errCode);
            return;
        }
        Task_sleep(1);
    }

    /* Initialize the DAT client configuration */
    memset ((void *)&datClientCfg, 0, sizeof(Dat_ClientCfg));

    /* Populate the DAT client configuration
     * - The NAME client handle is set to NULL since the DAT Server & Client in the test
     *   are executing in the same realm (i.e. on DSP) */
    strcpy (datClientCfg.clientName, "Dat_Client_LTE9A_L2DP");
    strcpy (datClientCfg.serverName, gDatServerName);
    datClientCfg.pktlibInstHandle  = appPktlibInstanceHandle;
    datClientCfg.msgcomInstHandle  = appMsgcomInstanceHandle;
    datClientCfg.databaseHandle    = globalNameDatabaseHandle;
    datClientCfg.clientHeapHandle  = datClientHeap;
    datClientCfg.nameClientHandle  = NULL;
    datClientCfg.id                = DNUM;
    datClientCfg.realm             = Dat_ExecutionRealm_DSP;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.beginMemAccess    = Dat_osalBeginMemoryAccess;
    datClientCfg.endMemAccess      = Dat_osalEndMemoryAccess;
    datClientCfg.createSem         = Dat_osalCreateSem;
    datClientCfg.deleteSem         = Dat_osalDeleteSem;
    datClientCfg.postSem           = Dat_osalPostSem;
    datClientCfg.pendSem           = Dat_osalPendSem;
    datClientCfg.enterCS           = Dat_osalEnterSingleCoreCriticalSection;
    datClientCfg.exitCS            = Dat_osalExitSingleCoreCriticalSection;

    /* Initialize the DAT client. */
    datClientHandle = Dat_initClient (&datClientCfg, &errCode);
    if (datClientHandle == NULL)
    {
        System_printf ("Error: Unable to create the DAT client [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT client %p created successfully\n", datClientHandle);

    /* Start the DAT client: DAT clients can only be started after they have been registered
     * by the server */
    while (1)
    {
        /* Start the DAT client */
        clientStatus = Dat_startClient (datClientHandle, &errCode);
        if (clientStatus < 0)
        {
            System_printf ("Error: DAT Client registration status failed [Error code %d]\n", errCode);
            return;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        Task_sleep(1);
    }

    /* Launch the DAT Client Execution Task:
     * - This is set to be the lowest priority task in the system and it will execute when
     *   there is no other active task. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 1;
    Task_create(Test_datClient, &taskParams, NULL);

    /* Setup the NETFP environment. */
    if (Test_netEnvironmentSetup() < 0)
    {
        System_printf ("Error: Unable to setup the NETFP environment\n");
        return;
    }

    /* Get the ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath(netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Get the egress fast path handle */
    fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Create a socket */
    datSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (datSockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = 1235 + DNUM;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
	sockAddress.op.bind.queueHandle     = NULL;

    /* Bind the socket. */
    if (Netfp_bind (datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been bound successfully\n");

    /* Populate the connect information. */
	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1235;
	sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been connected successfully\n");

    /* Initialize the consumer configuration */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    strcpy(consumerCfg.producerName, gProducerName);
    consumerCfg.heapHandle          = datConsumerHeap;

    /* Create the consumer */
    consumerHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
    if (consumerHandle == NULL)
    {
        System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Consumer Handle %x created successfully\n", consumerHandle);

    /* Try and connect the consumer to the producer  */
    while (1)
    {
        /* Connect the consumer & producer */
        if (Dat_connectConsumer (consumerHandle, &errCode) == 0)
        {
            System_printf ("Debug: Producer & consumer successfully connected\n");
            break;
        }

        /* Error: Connect between the consumer & producer failed. Use the error code to determine
         * the reason for failure. */
        if (errCode == DAT_ENOTREADY)
        {
            /* Producer was not operational. */
            Task_sleep(10);
            continue;
        }
        System_printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
        return;
    }

    /* Save consumer handle to be used in process Consumer */
    datConsumerHandle =  consumerHandle;

    /* Create consumer task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 1;
    Task_create(Test_consumerTask, &taskParams, NULL);

    return;
}


