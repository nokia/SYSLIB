/*
 *   @file  setup.c
 *
 *   @brief
 *      The file implements the functionality which is required before
 *      the tests can be created
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
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

/* Test configurations Include file */
#include "netCfg.h"

/**********************************************************************
 ************************** Local Structures **************************
 **********************************************************************/

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Buffer used to store NETFP configuration file */
char                    netfpConfigBuffer[2500];

/* NETFP Configuration after parsing the DAT file. */
Test_NetfpConfigInfo    netfpConfig;

/* NETFP Unit Test Selection: */
uint32_t                testSelection;
/* NETFP Proxy pid value */
uint32_t                proxyPid;

/* Global NETFP handles. */
Netfp_InboundFPHandle   fpIngressHandle = NULL;
Netfp_OutboundFPHandle  fpEgressHandle  = NULL;
Netfp_IfHandle          ifHandle        = NULL;
Netfp_InboundFPHandle   fpWcIngressHandle = NULL;
Netfp_OutboundFPHandle  fpEgress2Handle  = NULL;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle. */
extern Netfp_ClientHandle      netfpClientHandle;

/* Name Database Handle: */
extern Name_DBHandle           globalNameDatabaseHandle;

/* PKTLIB heap handles: */
extern Pktlib_HeapHandle   netfpDataRxHeap;

/* Unit Tests: */
extern void* Test_socketThread(void* arg);
extern void* Test_wildcardingv4Thread(void* arg);
extern void* Test_basicThread(void* arg);
extern void* Test_srbTask(void* arg);
extern void* Test_encodeTask(void* arg);
extern void* Test_multiSocketThread(void* arg);
extern void* Test_fpDrbReestablishmentTask(void* arg);
extern void* Test_spDrbReestablishmentTask(void* arg);
extern void* Test_routeRecalculationThread(void* arg);
extern void* Test_drbSourceHOTask(void* arg);
extern void* Test_drbFPTargetHOTask(void* arg);
extern void* Test_drbSPTargetHOTask(void* arg);
extern void* Test_mbmsThread(void* arg);
extern void* Test_multicastSendThread(void* arg);
extern void* Test_reestablishmentThread(void* arg);
extern void* Test_basicReestablishmentThread(void* arg);
extern void* Test_basicHOTask(void* arg);
extern void* Test_prioMarkTask(void* arg);

/* Setup Source Routing Environment: */
extern int32_t Test_setupSourceRoutingEnv();
extern void Test_cleanupSourceRoutingEnv (void);

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
            printf ("Error: Token Name '%s' is NOT recognized\n", tokenName);
            return -1;
        }
    }

    /* File has been successfully parsed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the default test environment. The parameters for the test
 *      environment are parsed from the netfp.dat file. In most test cases the default test
 *      environment should be suffient.
 *
 *  @param[in]  enabledl3Shaper
 *      Flag which indicates if the L3 shaper is to be configured
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_setupDefaultEnv (int32_t enabledl3Shaper)
{
    uint32_t                spidIngress;
    uint32_t                spidEgress;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    int32_t                 errCode;
    int32_t                 i;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    int32_t                 status;
    Name_ResourceCfg        nrConfig;
    Netfp_OptionTLV         optCfg;

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPv4 Setup Configuration: */
    printf ("-------------------------------------------------------------\n");
    printf ("Default IPv4 Test Environment Setup:\n");
    printf ("NETFP Client Handle     : %p\n", netfpClientHandle);
    printf ("My IP Address           : %d.%d.%d.%d\n",
                    ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
                    ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    printf ("Remote IP Address       : %d.%d.%d.%d\n",
                    ptrNetfpConfig->pdnGwIPAddress[0], ptrNetfpConfig->pdnGwIPAddress[1],
                    ptrNetfpConfig->pdnGwIPAddress[2], ptrNetfpConfig->pdnGwIPAddress[3]);
    printf ("-------------------------------------------------------------\n");

    /* Find the ingress policy to use for test */
    while (1)
    {
        if (Name_findResource ( globalNameDatabaseHandle,
                                Name_ResourceBucket_USER_DEF1,
                                "Ingress_SPID_IPv4_UP",
                                &nrConfig,
                                &errCode) == 0)
        {
            /* Found the Ingress IPv4 Policy to use for test */
            spidIngress =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep(1000*20);
                continue;
            }
            else
            {
                /* Error getting the policy to use for test. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for Ingress\n", spidIngress);

    /* Find the egress policy to use for test */
    while (1)
    {
        if (Name_findResource ( globalNameDatabaseHandle,
                                Name_ResourceBucket_USER_DEF1,
                                "Egress_SPID_IPv4_UP",
                                &nrConfig,
                                &errCode) == 0)
        {
            /* Found the Egress IPv4 Policy to use for test */
            spidEgress =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep(1000*20);
                continue;
            }
            else
            {
                /* Error getting the policy to use for test. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for Egress\n", spidEgress);

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId		            = spidIngress;

    /* Initialize spiMode */
    if(spidIngress == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;
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
        printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    printf ("Debug: Ingress IPv4 Fast Path Handle 0x%x\n", (uint32_t)fpIngressHandle);

    /************************************************************************************
	 * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
	memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

	/* Populate the Fast Path configuration. */
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");
	outboundFPCfg.spId					    = spidEgress;
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

    /* Setup the fast path mapping: Socket Priority is the mapped to the same Inner DSCP */
	for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;

    /* Create the outbound fast path: */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
	if (fpEgressHandle == NULL)
	{
	    printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
		return -1;
    }
	printf ("Debug: Egress IPv4 Fast Path Handle 0x%x [Waiting for it to be active]\n", (uint32_t)fpEgressHandle);

    /* Wait for fast path to become active? */
    while (1)
    {
        /* Get the fast path status: */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, fpEgressHandle, &status, &errCode) < 0)
        {
            printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if (status == 1)
            break;

        /* Sleep and try again after some time. */
        usleep(1000*20);
    }
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x is active\n", (uint32_t)fpEgressHandle);

    /* Do we need to setup the L3 Shaper? */
    if (enabledl3Shaper == 1)
    {
        Netfp_FlowCfg       flowCfg;
        int32_t             rxFlowId;
        Netfp_IfHandle      ifHandle;
        Netfp_L3QoSCfg      l3QOSCfg;

        /* NOTIFY the user: The DTS files of the kernel should be properly configured
         * for the L3 Shaper to operate. */
        printf ("-------------------------------------------------------------\n");
        printf ("NOTE: The L3 shaper requires the correct kernel DTS files    \n");
        printf ("The test assumes that the L3 QOS queues are 8000 onwards     \n");
        printf ("Please verify the kernel DTS files if this is NOT the case   \n");
        printf ("Assuming that the interface is eth0.                         \n");
        printf ("------------------------------------------------------------ \n");

        /* We need to create a flow for the L3 Shaper to work. */
        strcpy (flowCfg.name, "L3_QOS");
        flowCfg.numHeaps      = 1;
        flowCfg.heapHandle[0] = netfpDataRxHeap;
        flowCfg.sopOffset     = 0;

        /* Create the flow. */
        rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
        if (rxFlowId < 0)
        {
            printf ("Error: Unable to create the QOS flow [Error code %d]\n", errCode);
            return -1;
        }
        printf ("Debug: QOS Flow %d has been created successfully\n", rxFlowId);

        /* Initialize the L3 QOS configuration */
        memset ((void *)&l3QOSCfg, 0, sizeof(Netfp_L3QoSCfg));

        /* Setup the L3 shaper for the interface */
        ifHandle = Netfp_findInterface (netfpClientHandle, "eth0", NULL, &errCode);
        if (ifHandle == NULL)
        {
            printf ("Error: Interface eth0 NOT found; modify the test code with the correct interface [Error %d]\n", errCode);
            return -1;
        }

        /* Setup the L3 shaper: */
        l3QOSCfg.isEnable = 1;
        l3QOSCfg.flowId   = rxFlowId;

        /* By default all DSCP are mapped to the best effort queue i.e. 8072. */
        for (i = 0; i < 64; i++)
            l3QOSCfg.qid[i] = 8072;

        /***********************************************************************************
         * DSCP2  is mapped to Queue 8079 which is hp-cos7 [See the DTS]
         * DSCP63 is mapped to Queue 8010 which is wrr-cos2 [See the DTS]
         ***********************************************************************************/
        l3QOSCfg.qid[2]  = 8079;
        l3QOSCfg.qid[63] = 8074;

        /* Setup the L3 QOS Shaper: */
        optCfg.type   = Netfp_Option_L3_QOS;
        optCfg.length = sizeof(Netfp_L3QoSCfg);
        optCfg.value  = (void*)&l3QOSCfg;

        /* We can now setup the L3 QOS shaper: */
        if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
        {
            printf ("Error: Unable to configure the L3 Shaper[Error code %d]\n", errCode);
            return -1;
        }

        /* Display the configuration on the console. */
        printf ("L3 Shaper configured as follows: \n");
        printf ("DSCP2  is mapped to QOS Queue %d i.e. hp-cos7\n", l3QOSCfg.qid[2]);
        printf ("DSCP63 is mapped to QOS Queue %d i.e. wrr-cos2\n", l3QOSCfg.qid[63]);
        printf ("Verify by displaying the QOS statistics\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the default test environment. The parameters for the test
 *      environment are parsed from the netfp.dat file. In most test cases the default test
 *      environment should be suffient.
 *
 *  @param[in]  enabledl3Shaper
 *      Flag which indicates if the L3 shaper is to be configured
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_setupMulticastEnv (void)
{
    Netfp_InboundFPCfg      inboundFPCfg;
    int32_t                 errCode;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    uint8_t                 multicastIP[4] = { 227, 0, 100, 2 };

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPv4 Setup Configuration: */
    printf ("-------------------------------------------------------------\n");
    printf ("Default IPv4 Test Environment Setup:\n");
    printf ("NETFP Client Handle     : %p\n", netfpClientHandle);
    printf ("My IP Address           : %d.%d.%d.%d\n",
            ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
            ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    printf ("Remote IP Address       : %d.%d.%d.%d\n",
             multicastIP[0], multicastIP[1], multicastIP[2], multicastIP[3]);
    printf ("-------------------------------------------------------------\n");

    /* Initialize the multicast services: */
    if (Netfp_initMulticastServices (netfpClientHandle, &errCode) < 0)
    {
        printf ("Error: Unable to initialize the multicast services [Error code %d]\n", errCode);
        return -1;
    }

    /************************************************************************************
     * Multicast Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.fastpathMode               = Netfp_FastpathMode_MULTICASTSHARED;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = multicastIP[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = multicastIP[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = multicastIP[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = multicastIP[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Multicast-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    printf ("Debug: Multicast IPv4 Fast Path Handle 0x%x\n", (uint32_t)fpIngressHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the wildcarding test environment. The parameters for the test
 *      environment are parsed from the netfp.dat file. In most test cases the default test
 *      environment should be suffient.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Test_setupWildCardingEnv (void)
{
    uint32_t                spidIngressWc;
    uint32_t                spidEgressWc;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_OutboundFPCfg     outboundFPCfg;
    int32_t                 errCode;
    int32_t                 i;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    Name_ResourceCfg        nrConfig;
    int32_t                 status;

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Find the ingress policy to use for wild carding test */
    while (1)
    {
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                "IngressWC_SPID_IPv4_UP", &nrConfig, &errCode) == 0)
        {
            /* Found the Ingress IPv4 Policy to use for test */
            spidIngressWc =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep(1000*20);
                continue;
            }
            else
            {
                /* Error getting the policy to use for test. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for wild carding Ingress\n", spidIngressWc);

    /* Find the egress policy to use for wild carding test */
    while (1)
    {
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                "EgressWC_SPID_IPv4_UP", &nrConfig, &errCode) == 0)
        {
            /* Found the Egress IPv4 Policy to use for test */
            spidEgressWc =    (uint32_t)nrConfig.handle1;
            break;
        }
        else
        {
            /* Error finding the policy. Check the error */
            if (errCode == NAME_ENOTFOUND)
            {
                /* Policy not found. Wait for sometime and retry */
                usleep(1000*20);
                continue;
            }
            else
            {
                /* Error getting the policy to use for test. Cant proceed */
                return -1;
            }
        }
    }
    printf ("Debug: Using Policy Id %d for wildcarding Egress\n", spidEgressWc);

    /************************************************************************************
     * Wildcarding Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof(Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    if(spidIngressWc == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    inboundFPCfg.spId                       = spidIngressWc;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 0;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 0;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-WcFastPath");

    /* Add the Ingress Fast Path. */
    fpWcIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpWcIngressHandle == NULL)
    {
        printf ("Error: Unable to create Wildcarding Ingress fast path Error %d\n", errCode);
        return -1;
    }
    printf ("Debug: Wildcarding Ingress IPv4 Fast Path Handle 0x%p\n", fpWcIngressHandle);

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the outbound fast path configuration: */
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath2");
    outboundFPCfg.spId                    = spidEgressWc;
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

    /* Setup the fast path mapping: */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = 0;

    /* Add the 2nd Egress Fast Path. */
    fpEgress2Handle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgress2Handle == NULL)
    {
        printf ("Error: Unable to create 2nd Egress fast path to %d.%d.%d.%d, Error %d\n",
                ptrNetfpConfig->pdnGw2IPAddress[0], ptrNetfpConfig->pdnGw2IPAddress[1],
                ptrNetfpConfig->pdnGw2IPAddress[2], ptrNetfpConfig->pdnGw2IPAddress[3],
                errCode);
        return -1;
    }

    /* Wait for fast path to become active? */
    while (1)
    {
        /* Get the fast path status: */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, fpEgress2Handle, &status, &errCode) < 0)
        {
            printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if (status == 1)
            break;

        /* Sleep and try again after some time. */
        usleep(1000*20);
    }
    printf ("Debug: Second Egress IPv4 Fast Path for wildcarding Handle 0x%p\n", fpEgress2Handle);
    return 0;
}

/**
*  @b Description
*  @n
*      The function is used to test the transmission of multicast packets
*
*  @retval
*      0   - Success
*  @retval
*      <0  - Error
*/
static int32_t Test_setupInfoonlyFPEnv(void)
{
    int32_t                 errCode;
    Netfp_InboundFPHandle   fpInboundHandle;
    Netfp_InboundFPHandle   fpOutboundHandle;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_InboundFPCfg      inboundFPCfg;
    int32_t                 i;
    int32_t                 status;
    uint8_t                 nextHopMACAddress[6] = { 0x00, 0x04, 0x23, 0xbd, 0x95, 0x0e };
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceCfg      ifConfig;

#if 0
    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPv4 Setup Configuration: */
    printf ("-------------------------------------------------------------\n");
    printf ("Default IPv4 Test Environment Setup:\n");
    printf ("NETFP Client Handle     : %p\n", netfpClientHandle);
    printf ("My IP Address           : %d.%d.%d.%d\n",
                    ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
                    ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    printf ("Remote IP Address       : %d.%d.%d.%d\n",
                    ptrNetfpConfig->pdnGwIPAddress[0], ptrNetfpConfig->pdnGwIPAddress[1],
                    ptrNetfpConfig->pdnGwIPAddress[2], ptrNetfpConfig->pdnGwIPAddress[3]);
    printf ("-------------------------------------------------------------\n");

    /* Initialize the fast path configuration */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));
#endif

    /* Initialize spidMode */
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.fastpathMode               = Netfp_FastpathMode_INFOONLY;

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 2;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 1;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Create the inbound fast path */
    fpInboundHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpInboundHandle == NULL)
    {
        printf ("Error: Unable to create the inbound fast path [Error code %d]\n", errCode);
        return -1;
    }

    /************************************************************************************
     * Egress IPv4 Fast Path:
     ************************************************************************************/
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
    strcpy (ifConfig.name, "eth0");

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
        return -1;

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = 227;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = 0;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = 100;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = 2;
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = 192;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = 168;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = 1;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = 1;
    outboundFPCfg.ifHandle = ifHandle;
    memcpy ((void *)&outboundFPCfg.nextHopMACAddress[0], (void *)nextHopMACAddress, 6);

    /* Setup the fast path mapping: Socket Priority is the mapped to the same Inner DSCP */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;

    /* Create the outbound fast path: */
    fpOutboundHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpOutboundHandle == NULL)
    {
        printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x [Waiting for it to be active]\n", (uint32_t)fpOutboundHandle);

    /* Wait for fast path to become active? */
    while (1)
    {
        /* Get the fast path status: */
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, fpOutboundHandle, &status, &errCode) < 0)
        {
            printf ("Error: Getting status of the outbound fast path failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Is the fast path active? */
        if (status == 1)
            break;

        /* Sleep and try again after some time. */
        usleep(1000*20);
    }
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%x is active\n", (uint32_t)fpOutboundHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clean up the default test environment.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_cleanupDefaultEnv (void)
{
    int32_t     errCode;

    if (fpIngressHandle != NULL)
    {
        if (Netfp_deleteInboundFastPath (netfpClientHandle, fpIngressHandle, &errCode) < 0)
        {
    	    printf ("Error: NETFP Ingress fast path deletion failed [Error Code %d]\n", errCode);
            return;
        }
        printf ("Debug: NETFP Ingress fast path deleted successfully\n");
    }
    if (fpEgressHandle != NULL)
    {
        if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpEgressHandle, &errCode) < 0)
        {
    	    printf ("Error: NETFP Egress fast path deletion failed [Error Code %d]\n", errCode);
	    	return;
        }
        printf ("Debug: NETFP Egress fast path deleted successfully\n");
    }

    /* Delete wild carding fast path handles */
    if (fpWcIngressHandle != NULL)
    {
        if (Netfp_deleteInboundFastPath (netfpClientHandle, fpWcIngressHandle, &errCode) < 0)
        {
            printf ("Error: NETFP Ingress fast path deletion failed [Error Code %d]\n", errCode);
            return;
        }
        printf ("Debug: NETFP Ingress fast path deleted successfully\n");
    }
    if (fpEgress2Handle != NULL)
    {
        if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpEgress2Handle, &errCode) < 0)
        {
            printf ("Error: NETFP Egress fast path deletion failed [Error Code %d]\n", errCode);
                return;
        }
        printf ("Debug: NETFP Egress fast path deleted successfully\n");
    }

    /* Delete the interface */
    if (ifHandle != NULL)
    {
        if (Netfp_deleteInterface (netfpClientHandle, ifHandle, &errCode) < 0)
        {
    	    printf ("Error: NETFP Interface deletion failed [Error Code %d]\n", errCode);
	    	return;
        }
        printf ("Debug: NETFP Interface deleted successfully\n");
    }

    /* Done cleaning up */
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void SetupTask(void* arg0)
{
    pthread_t       receiveThread;
    int32_t         size;
    FILE*           configFileHandle;
    int32_t         errCode = 0;
    uint32_t*       params = (uint32_t*)arg0;
    int             paramsIndex=0;

	/* Open the NETFP configuration file for the tests */
	configFileHandle = fopen ("./netfp.dat", "r");
	if (configFileHandle == NULL)
	{
		printf ("Error: Unable to open NETFP configuration file (c:\\netfp.dat)\n");
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
                    printf ("Error: Configuration file is too big\n");
                    fclose (configFileHandle);
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
        printf ("Error: Unable to parse the NETFP configuration file.\n");
        return;
    }

    if(params)
    {
        testSelection = params[paramsIndex++];
        if ((testSelection >= 1) && (testSelection <= 18))
        {
            if (( testSelection == 9 ) && ( params[paramsIndex] == 0 )){
                printf("Error: missing parameter for 9.  Route recalculation\n");
                return;
            }
            printf("Test automation: testSelection %d\n",testSelection);
        }
        else
        {
            printf("Error test automation: bad testSelection %d!!!\n",testSelection);
            return;
        }
        
    }
    else
    {
        /***********************************************************************
         * CLI Simulation:
         ***********************************************************************/
        while (1)
        {
            usleep(1000*20);
            printf ("*******************************************************\n");
            printf ("ARM NETFP Unit Test Menu \n");
            printf ("Please select the type of test to execute:\n");
            printf ("0.  Exit\n");
            printf ("1.  Socket        \n");
            printf ("2.  Basic Testing \n");
            printf ("3.  SRB\n");
            printf ("4.  Encode DRB\n");
            printf ("5.  Source routing\n");
            printf ("6.  Fast Path Re-establishment\n");
            printf ("7.  Slow Path Re-establishment\n");
            printf ("8.  Socket [L3 Shaper]\n");
            printf ("9.  Route recalculation\n");
            printf ("10. Wildcarding Test\n");
            printf ("11. Source Hand Over\n");
            printf ("12. Fast Path Target Hand Over\n");
            printf ("13. Slow Path Target Hand Over\n");
            printf ("14. MBMS Tests\n");
            printf ("15. Multicast Send\n");
            printf ("16. Simulated SRB/DRB Encode/Decode with Reestablishment\n");
            printf ("17. Basic Reestablishment\n");
            printf ("18. Basic Handover\n");
            printf ("19. Priority Marking\n");
            printf ("*******************************************************\n");
            printf ("> Enter your selection: ");

            /* Wait for the user input. */
            scanf ("%d", &testSelection);

            /* Exit! */
            if (testSelection == 0)
                goto cleanup_and_exit;

            /* Validate the selection: */
            if ((testSelection >= 1) && (testSelection <= 19))
                break;
        }
    }
    /* Launch the appropriate test task on the basis of the selection:
     * NOTE: The value of the test selection here and in the statistics task
     * need to be kept in SYNC. Else there will be incorrect displays */
    switch (testSelection)
    {
        case 1:
        {
            /* Execute the socket tests: Setup the default environment. */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_socketThread, NULL);
            break;
        }
        case 2:
        {
            /* Execute the basic tests: The test sets up its own environment */
            errCode = pthread_create (&receiveThread, NULL, Test_basicThread, NULL);
            break;
        }
        case 3:
        {
            /* Execute the basic tests: The test sets up its own environment */
            errCode = pthread_create (&receiveThread, NULL, Test_srbTask, NULL);
            break;
        }
        case 4:
        {
            /* Execute the DRB encode tests: The test sets up its own environment */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_encodeTask, NULL);
            break;
        }
        case 5:
        {
            /* Execute the Source IP based routing tests: The test sets up its own environment */
            Test_setupSourceRoutingEnv();
            errCode = pthread_create (&receiveThread, NULL, Test_multiSocketThread, NULL);
            break;
        }
        case 6:
        {
            /* Execute the re-establishment tests: The test sets up its own environment */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_fpDrbReestablishmentTask, NULL);
            break;
        }
        case 7:
        {
            /* Execute the re-establishment tests: The test sets up its own environment */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_spDrbReestablishmentTask, NULL);
            break;
        }
        case 8:
        {
            /* Execute the L3 shaper tests: */
            Test_setupDefaultEnv(1);
            errCode = pthread_create (&receiveThread, NULL, Test_socketThread, NULL);
            break;
        }
        case 9:
        {
            /* Route Recalculation tests: */
            printf ("> Enter the Netfp Proxy pid: ");

            /* Get the NetfpProxy PID.*/
           if(params)
            {
                proxyPid = params[1];
            }
            else
            {
                scanf ("%d", &proxyPid);
            }
            printf ("Netfp Proxy pid is:%d\n", proxyPid);

            errCode = pthread_create (&receiveThread, NULL, Test_routeRecalculationThread, &proxyPid);
            break;
        }
        case 10:
        {
            /* Execute the wildcarding tests: */
            Test_setupDefaultEnv(0);
            Test_setupWildCardingEnv();
            errCode = pthread_create (&receiveThread, NULL, Test_wildcardingv4Thread, NULL);
            errCode = pthread_create (&receiveThread, NULL, Test_socketThread, NULL);
            break;
        }
        case 11:
        {
            /* Execute the Source HO tests: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_drbSourceHOTask, NULL);
            break;
        }
        case 12:
        {
            /* Execute the Fast Path Target HO tests: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_drbFPTargetHOTask, NULL);
            break;
        }
        case 13:
        {
            /* Execute the Slow Path Target HO tests: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_drbSPTargetHOTask, NULL);
            break;
        }
        case 14:
        {
            /* Execute the MBMS Tests */
            Test_setupMulticastEnv();
            errCode = pthread_create (&receiveThread, NULL, Test_mbmsThread, NULL);
            break;
        }
        case 15:
        {
            /* Execute the Multicast packet send with info only inbound FP */
            Test_setupInfoonlyFPEnv();
            errCode = pthread_create (&receiveThread, NULL, Test_multicastSendThread, NULL);
            break;
        }
        case 16:
        {
            /* Reestablishment Stress Test: */
            errCode = pthread_create (&receiveThread, NULL, Test_reestablishmentThread, NULL);
            break;
        }
        case 17:
        {
            /* Basic Reestablishment Test: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_basicReestablishmentThread, NULL);
            break;
        }
        case 18:
        {
            /* Basic HO Task: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_basicHOTask, NULL);
            break;
        }
        case 19:
        {
            /* Priority marking/tagging: */
            Test_setupDefaultEnv(0);
            errCode = pthread_create (&receiveThread, NULL, Test_prioMarkTask, NULL);
            break;
        }
        default:
        {
            printf ("Error: Undefined selection %d; aborting test\n", testSelection);
            return;
        }
    }

    /* Once the IPv4 Test environment has been configured. Start the test tasks. */
    if (errCode < 0)
    {
        printf ("Error: Unable to create the launch the test thread for selection %d [Error code %d]\n", testSelection, errCode);
        return;
    }

    /* Wait for the threads to terminate */
    pthread_join (receiveThread, NULL);

cleanup_and_exit:

    /* Cleanup and exit */
    if (testSelection == 1 || testSelection == 4 || testSelection == 6 || testSelection == 9 )
    {
        /* Clean up the default environment */
        Test_cleanupDefaultEnv ();
    }
    else if (testSelection == 5)
    {
        /* Clean up the source routing environment */
        Test_cleanupSourceRoutingEnv ();
    }
    else
    {
        /* The tests clean up after themselves. Nothing to do */
    }

    return;
}
