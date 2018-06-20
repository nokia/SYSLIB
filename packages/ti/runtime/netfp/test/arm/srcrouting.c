/*
 *   @file  srcrouting.c
 *
 *   @brief   
 *      The file implements the functions required to execute
 *      source routing based data tests
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
#include <ti/runtime/name/name_db.h>

#include "benchmark.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV4_BASE_UDP_PORT          10000

/* Number of parallel sockets created for test */
#define TEST_MAX_NUM_SOCKETS                    4

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of UDP-IPv4 Packets received */
static uint32_t            gNumIPv4PktRxed     = 0;

/* Global counter for the number of UDP packet dropped. */
static uint32_t            gNumIPv4PktDropped  = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */ 
BENCHMARK_INIT(Netfp_getPayloadIPv4_srcRouting)
BENCHMARK_INIT(Netfp_sendIPv4_srcRouting)

/* Ingress fast path handles */ 
Netfp_InboundFPHandle fpIngressHandle[TEST_MAX_NUM_SOCKETS];

/* Egress fast path handles */ 
Netfp_OutboundFPHandle      fpEgressHandle[TEST_MAX_NUM_SOCKETS];

/* NETFP socket handles */ 
Netfp_SockHandle    sockHandle[TEST_MAX_NUM_SOCKETS];

/* Msgcom Rx channel handles for each of the NETFP sockets */ 
MsgCom_ChHandle     udpChannelHandle[TEST_MAX_NUM_SOCKETS];

/* Global variable to track total number of fastpaths created for test */ 
int32_t             gFastPathCounter = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle   netfpDataRxHeap;
extern volatile uint32_t   testComplete;
extern Netfp_ClientHandle  netfpClientHandle;

/* Global MSGCOM instance handle. */
extern Msgcom_InstHandle   appMsgcomInstanceHandle;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle   appPktlibInstanceHandle;

/* Name Database Handle: */
extern Name_DBHandle           globalNameDatabaseHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function is used to cleanup the MSGCOM data buffers if there are 
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle associated with the MSGCOM instance
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void myFreeMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n  
 *      This is an exported function which is used to display the statistics related
 *      to the IPv4 Socket Tests.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_socketIPv4DisplayStats(void)
{
    printf ("-------------------------------------------------------------\n");
    printf ("IPv4 Socket::::\n");

    /* Display the module statistics. */
    printf ("Number of IPv4 (UDP+GTPU) Pkt Received: %d\n", gNumIPv4PktRxed);
    printf ("Number of IPv4 (UDP+GTPU) Pkt Dropped : %d\n", gNumIPv4PktDropped);

    /* Display the benchmarking information for the NETFP Functions. */
    printf ("Benchmarking Information for IPv4 Stress Test:\n");

    BENCHMARK_DISPLAY(Netfp_getPayloadIPv4_srcRouting);
    BENCHMARK_DISPLAY(Netfp_sendIPv4_srcRouting);
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to process the received packet.
 *
 *  @param[in]  sockHandle
 *      Socket Handle on which the packet was received
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_socketProcessPacket (Netfp_SockHandle sockHandle, Ti_Pkt* ptrRxPkt)
{
    Ti_Pkt*             ptrTxPkt;
    uint8_t*            ptrRxDataBuffer;
    uint8_t*            ptrTxDataBuffer;
    uint32_t            rxDataBufferLen;
    uint32_t            txDataBufferLen;
    int32_t             retVal;
    int32_t             errCode;
    Netfp_PeerSockAddr  peerSockAddress;

    /* Increment the statistics. */
    gNumIPv4PktRxed++;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Use the receive payload to get the data buffer: Take timestamps also. */
    //BENCHMARK_START(Netfp_getPayloadIPv4);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    //BENCHMARK_END(Netfp_getPayloadIPv4);
    if (retVal < 0)
    {
        printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    //BENCHMARK_UPDATE(Netfp_getPayloadIPv4);

    /* Display the peer socket address information only for the first packet */
    if (gNumIPv4PktRxed == 1)
    {
        printf ("Debug: Received Packet from Peer Address %d.%d.%d.%d Port = %d\n", 
                       peerSockAddress.sin_addr.addr.ipv4.u.a8[0], peerSockAddress.sin_addr.addr.ipv4.u.a8[1], 
                       peerSockAddress.sin_addr.addr.ipv4.u.a8[2], peerSockAddress.sin_addr.addr.ipv4.u.a8[3],
                       peerSockAddress.sin_port);
    }

    /* Send out the packet. */ 
    txDataBufferLen = rxDataBufferLen;

    /* Allocate another packet from the NETFP Data receive Heap */
    ptrTxPkt = Pktlib_allocPacket(appPktlibInstanceHandle, netfpDataRxHeap, txDataBufferLen);
    if (ptrTxPkt == NULL)
    {
        //gNumIPv4PktDropped++;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
        return -1;
    }

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrTxPkt, &ptrTxDataBuffer, &txDataBufferLen);

    /* Copy the data buffer */
    memcpy (ptrTxDataBuffer, ptrRxDataBuffer, txDataBufferLen);

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrTxPkt, txDataBufferLen);
    Pktlib_setDataBufferLen(ptrTxPkt, txDataBufferLen);

    /* Send out the socket on the appropriate socket: Take timestamps also. */
    //BENCHMARK_START(Netfp_sendIPv4);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    //BENCHMARK_END(Netfp_sendIPv4);
    if (retVal < 0)
    {
        printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    //BENCHMARK_UPDATE(Netfp_sendIPv4);

    /* Clean up the received packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      This is the receive task which receives the packets and processes them.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_multiSocketThread(void* arg)
{
    int32_t                 i, errCode;
    Ti_Pkt*                 ptrRxPkt;

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the 
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        for (i = 0; i < TEST_MAX_NUM_SOCKETS; i ++)
        {
            /* Check if a packets has been received on the the LTE channel */
            Msgcom_getMessage (udpChannelHandle[i], (MsgCom_Buffer**)&ptrRxPkt);

            /* Process the packet: */
            if (ptrRxPkt != NULL)
                Test_socketProcessPacket(sockHandle[i], ptrRxPkt);                
        }
    }

    /* Test has completed execution. Display the statistics */
    Test_socketIPv4DisplayStats ();

    for (i = 0; i < TEST_MAX_NUM_SOCKETS; i ++)
    {
        /* Shutdown the socket */
        printf ("Debug: Closing the NETFP Socket '%d'\n", i);
        if (Netfp_closeSocket (sockHandle[i], &errCode) < 0)
        {
	        printf ("Error: NETFP socket '%d' close failed [Error Code %d]\n", i, errCode);
		    return NULL;
        }
        printf ("Debug: NETFP Socket '%d' closed successfully\n", i);

        /* Shutdown the MSGCOM channel */
        errCode = Msgcom_delete (udpChannelHandle[i], myFreeMsgBuffer);
        if (errCode < 0)
        {
	        printf ("Error: MSGCOM UDP channel '%d' close failed [Error Code %d]\n", i, errCode);
		    return NULL;
        }
        printf ("Debug: MSGCOM UDP channel '%d' deleted successfully\n", i);
    }

    /* TODO: Shutdown the NETFP Flow */

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the IPv4 environment 
 *      for the test based on the Inner IP address, PDN IP address
 *      specified
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupIPv4Env
(
    uint8_t*                ptrRNCIPAddr, 
    uint8_t*                ptrEnodeBInnerIPAddr, 
    uint8_t                 bUseSourceRouting,
    int32_t                 fpCtr
)
{
#if 0
    uint32_t                spidIngress;
    uint32_t                spidEgress;
    Netfp_FastPathCfg       fpConfig;
    int32_t                 errCode;
    int32_t                 i;
    uint32_t                resolveRouteId;
    Name_ResourceCfg        nrConfig;
    Netfp_RouteHandle       routeHandle;
    Netfp_IPAddr            dstIPAddr, srcIPAddr;
    Netfp_SockAddr          sockAddress;
    Msgcom_ChannelCfg       chConfig;
    Netfp_FlowCfg           flowCfg;
    int32_t                 rxFlowId;
    char                    name[NETFP_MAX_CHAR];

    /* Display the IPv4 Setup Configuration: */
    printf ("-------------------------------------------------------------\n");
    printf ("Default IPv4 Test Environment Setup:\n");
    printf ("NETFP Client Handle    : %p\n", netfpClientHandle);
    printf ("Inner IP Address       : %d.%d.%d.%d\n", 
                    ptrEnodeBInnerIPAddr[0], ptrEnodeBInnerIPAddr[1], 
                    ptrEnodeBInnerIPAddr[2], ptrEnodeBInnerIPAddr[3]);
    printf ("Remote IP Address      : %d.%d.%d.%d\n",
                    ptrRNCIPAddr[0], ptrRNCIPAddr[1], 
                    ptrRNCIPAddr[2], ptrRNCIPAddr[3]);
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
    memset ((void *)&fpConfig, 0, sizeof (Netfp_FastPathCfg));

    /* Populate the Fast Path configuration. */
    fpConfig.direction                  = Netfp_Direction_INBOUND;

    /* Initialize spiMode */
    if(spidIngress == NETFP_INVALID_SPID)
        fpConfig.spidMode = Netfp_SPIDMode_INVALID;
    else 
        fpConfig.spidMode = Netfp_SPIDMode_SPECIFIC;

    fpConfig.spId					    = spidIngress;
    fpConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    fpConfig.srcIP.addr.ipv4.u.a8[0]    = ptrRNCIPAddr[0];
    fpConfig.srcIP.addr.ipv4.u.a8[1]    = ptrRNCIPAddr[1];
    fpConfig.srcIP.addr.ipv4.u.a8[2]    = ptrRNCIPAddr[2];
    fpConfig.srcIP.addr.ipv4.u.a8[3]    = ptrRNCIPAddr[3];
    fpConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    fpConfig.dstIP.addr.ipv4.u.a8[0]    = ptrEnodeBInnerIPAddr[0];
    fpConfig.dstIP.addr.ipv4.u.a8[1]    = ptrEnodeBInnerIPAddr[1];
    fpConfig.dstIP.addr.ipv4.u.a8[2]    = ptrEnodeBInnerIPAddr[2];
    fpConfig.dstIP.addr.ipv4.u.a8[3]    = ptrEnodeBInnerIPAddr[3];
    sprintf (fpConfig.name, "Ingress-IPv4-FastPath-%d", fpCtr);

    /* Add the Ingress Fast Path. */
    fpIngressHandle[fpCtr] = Netfp_addFastPath (netfpClientHandle, &fpConfig, &errCode);
    if (fpIngressHandle[fpCtr] == NULL)
    {
        printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    printf ("Debug: Ingress IPv4 Fast Path Handle[%d]: 0x%p\n", fpCtr, fpIngressHandle[fpCtr]);

    /************************************************************************************
	 * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Trigger a route lookup request to Egress FastPath endpoint.
     * This is needed for Egress FastPath creation. */
    dstIPAddr.ver               =   Netfp_IPVersion_IPV4;
    dstIPAddr.addr.ipv4.u.a8[0] =   ptrRNCIPAddr[0];
    dstIPAddr.addr.ipv4.u.a8[1] =   ptrRNCIPAddr[1];
    dstIPAddr.addr.ipv4.u.a8[2] =   ptrRNCIPAddr[2];
    dstIPAddr.addr.ipv4.u.a8[3] =   ptrRNCIPAddr[3];

    srcIPAddr.ver               =   Netfp_IPVersion_IPV4;
    srcIPAddr.addr.ipv4.u.a8[0] =   ptrEnodeBInnerIPAddr[0];
    srcIPAddr.addr.ipv4.u.a8[1] =   ptrEnodeBInnerIPAddr[1];
    srcIPAddr.addr.ipv4.u.a8[2] =   ptrEnodeBInnerIPAddr[2];
    srcIPAddr.addr.ipv4.u.a8[3] =   ptrEnodeBInnerIPAddr[3];
    if (bUseSourceRouting)
    {
        /* Get the best route using the destination, and source IP addresses */
        if (Netfp_resolveRoute (netfpClientHandle, &dstIPAddr, &srcIPAddr, &resolveRouteId, &errCode) < 0)
        {
            /* Error resolving route */                        
            printf ("Error: Failed to resolve route for %d.%d.%d.%d [Error code %d]\n", 
                    ptrRNCIPAddr[0], ptrRNCIPAddr[1], 
                    ptrRNCIPAddr[2], ptrRNCIPAddr[3], errCode);
            return -1;
        }
    }
    else
    {
        /* Get the best route just based on the destination IP address */
        if (Netfp_resolveRoute (netfpClientHandle, &dstIPAddr, NULL, &resolveRouteId, &errCode) < 0)
        {
            /* Error resolving route */                        
            printf ("Error: Failed to resolve route for %d.%d.%d.%d [Error code %d]\n", 
                    ptrRNCIPAddr[0], ptrRNCIPAddr[1], 
                    ptrRNCIPAddr[2], ptrRNCIPAddr[3], errCode);
            return -1;
        }
    }

    /* Wait for route resolution to be completed */
    while (1)
    {
        routeHandle = Netfp_isRouteResolved (netfpClientHandle, resolveRouteId, &errCode);            
        if (routeHandle)
        {
            /* Route has been resolved and installed in NetFP succesfully.
             * Proceed with Egress FastPath creation. */
            break;
        }
        else
        {
            /* Wait for sometime and check on the status */                
            usleep(1000*20);
            continue;
        }
    }

    /* Initialize the fast path configuration. */
    memset ((void *)&fpConfig, 0, sizeof (Netfp_FastPathCfg));

    /* Populate the Fast Path configuration. */
    fpConfig.direction                  = Netfp_Direction_OUTBOUND;

    /* Initialize spiMode */
    if(spidEgress == NETFP_INVALID_SPID)
        fpConfig.spidMode = Netfp_SPIDMode_INVALID;
    else 
        fpConfig.spidMode = Netfp_SPIDMode_SPECIFIC;

    fpConfig.spId                       = spidEgress;
    fpConfig.dstIP.ver                  = Netfp_IPVersion_IPV4;
    fpConfig.dstIP.addr.ipv4.u.a8[0]    = ptrRNCIPAddr[0];
    fpConfig.dstIP.addr.ipv4.u.a8[1]    = ptrRNCIPAddr[1];
    fpConfig.dstIP.addr.ipv4.u.a8[2]    = ptrRNCIPAddr[2];
    fpConfig.dstIP.addr.ipv4.u.a8[3]    = ptrRNCIPAddr[3];
    fpConfig.srcIP.ver                  = Netfp_IPVersion_IPV4;
    fpConfig.srcIP.addr.ipv4.u.a8[0]    = ptrEnodeBInnerIPAddr[0];
    fpConfig.srcIP.addr.ipv4.u.a8[1]    = ptrEnodeBInnerIPAddr[1];
    fpConfig.srcIP.addr.ipv4.u.a8[2]    = ptrEnodeBInnerIPAddr[2];
    fpConfig.srcIP.addr.ipv4.u.a8[3]    = ptrEnodeBInnerIPAddr[3];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        fpConfig.dscpMapping[i] = i;    
    sprintf (fpConfig.name, "Egress-IPv4-FastPath-%d", fpCtr);

    /* Setup the route to use for this fast path */
    fpConfig.routeHandle                = routeHandle;

    /* Add the Egress Fast Path. */
    fpEgressHandle[fpCtr] = Netfp_addFastPath (netfpClientHandle, &fpConfig, &errCode);
    if (fpEgressHandle[fpCtr] == NULL)
    {
        printf ("Error: Unable to create Egress fast path[%d], Error %d\n", fpCtr, errCode);
        return -1;
    }
    printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle[fpCtr]);

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    sprintf (flowCfg.name, "SocketTestFlow-%d", fpCtr);
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        printf ("Error: Unable to create the flow '%s', [Error code %d]\n", flowCfg.name, errCode);
        return -1;
    }
    printf ("Debug: Flow '%d' has been created successfully\n", rxFlowId);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    sprintf (name, "Socket-IPv4-Channel-%d", fpCtr);
    udpChannelHandle[fpCtr] = Msgcom_create (name, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle[fpCtr] == 0)
    {
        printf ("Error: Unable to open the LTE IPv4 Channel '%s', Error : %d\n", name, errCode);
        return -1;
    }

    /* Create a socket */
	sockHandle[fpCtr] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle[fpCtr] == NULL)
	{
        printf ("Error: NETFP Socket '%d' Creation Failed [Error Code %d]\n", fpCtr, errCode);
	    return -1;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;    
	sockAddress.sin_port                = TEST_SOCKET_IPV4_BASE_UDP_PORT + fpCtr;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle[fpCtr];
    sockAddress.op.bind.flowId          = rxFlowId;
	sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle[fpCtr]);

    /* Bind the socket. */
	if (Netfp_bind (sockHandle[fpCtr], &sockAddress, &errCode) < 0)
    {
	    printf ("Error: NETFP Bind Failed for socket '%d' on port '%d' [Error Code %d]\n", 
                fpCtr, sockAddress.sin_port, errCode);
		return -1;
    }
	printf ("Debug: Socket '%d' has been bound successfully to port '%d'\n", fpCtr, sockAddress.sin_port);

    /* Populate the connect information. */
	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = TEST_SOCKET_IPV4_BASE_UDP_PORT + fpCtr;
	sockAddress.op.connect.outboundFPHandle = fpEgressHandle[fpCtr];

    /* Connect the socket */
	if (Netfp_connect(sockHandle[fpCtr], &sockAddress, &errCode) < 0)
    {
	    printf ("Error: NETFP Connect Failed for socket '%d' to port '%d' [Error Code %d]\n", 
                fpCtr, sockAddress.sin_port, errCode);
		return -1;
    }
    printf ("Debug: Socket '%d' has been connected successfully to '%d'\n", fpCtr, sockAddress.sin_port);

    /* Debug Message: */
    printf ("------------------------------------------------------------------------------\n");
    printf ("Please use a traffic generation application to send packets to the UDP Port %d\n", sockAddress.sin_port);    
    printf ("------------------------------------------------------------------------------\n");

    return 0;
#else
    return -1;
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to setup Source routing based test environment.
 *      The test currently uses hard coded IP addresses based on the
 *      Source routing configuration files released with syslib.
 *      Use conf/syslib_duc_lte_srcrouting_trust_9 for non-secure
 *      source based routing setup and use 
 *      conf/syslib_duc_lte_srcrouting_untrust_10 for secure setup
 *
 *  @retval
 *      Not Applicable
 */
void Test_setupSourceRoutingEnv (void)
{
    uint8_t     pdnGwAddress1[4] = {192, 168, 3, 1};
    uint8_t     pdnGwAddress2[4] = {192, 168, 3, 2};
    uint8_t     innerIPAddress1[4] = {192, 168, 1, 2};
    uint8_t     innerIPAddress2[4] = {192, 168, 1, 5};

    /* Setup the Source routing based IPv4 Test Environment for all the 
     * eNodeB Inner IP address and PDN G/w IP address combinations. */
    if (Test_setupIPv4Env(pdnGwAddress1, innerIPAddress1, 1, gFastPathCounter) < 0)
    {
        printf ("Error: Failed to setup the IPv4 Test Environment for Fast Path '%d' (using tbl_1)\n", gFastPathCounter);
        return;
    }
    gFastPathCounter++;

    if (Test_setupIPv4Env(pdnGwAddress2, innerIPAddress1, 1, gFastPathCounter) < 0)
    {
        printf ("Error: Failed to setup the IPv4 Test Environment for Fast Path '%d' (using tbl_1)\n", gFastPathCounter);
        return;
    }
    gFastPathCounter++;

    if (Test_setupIPv4Env(pdnGwAddress1, innerIPAddress2, 1, gFastPathCounter) < 0)
    {
        printf ("Error: Failed to setup the IPv4 Test Environment for Fast Path '%d' (using tbl_2)\n", gFastPathCounter);
        return;
    }
    gFastPathCounter++;

    if (Test_setupIPv4Env(pdnGwAddress1, innerIPAddress1, 0, gFastPathCounter) < 0)
    {
        printf ("Error: Failed to setup the IPv4 Test Environment for Fast Path '%d' (using main)\n", gFastPathCounter);
        return;
    }
    gFastPathCounter++;

    /* Done initializing for the test */
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to clean up Source routing based test environment.
 *
 *  @retval
 *      Not Applicable
 */
void Test_cleanupSourceRoutingEnv (void)
{
    int32_t     i;        
    int32_t     errCode;

    for (i = 0; i < gFastPathCounter; i++)
    {
        if (fpIngressHandle[i] != NULL)
        {
            if (Netfp_deleteInboundFastPath (netfpClientHandle, fpIngressHandle[i], &errCode) < 0)
            {
    	        printf ("Error: NETFP Ingress fast path '%d' deletion failed [Error Code %d]\n", i, errCode);
                return;
            }
            printf ("Debug: NETFP Ingress fast path '%d' deleted successfully\n", i);
        }
        if (fpEgressHandle[i] != NULL)
        {
            if (Netfp_deleteOutboundFastPath (netfpClientHandle, fpEgressHandle[i], &errCode) < 0)
            {
    	        printf ("Error: NETFP Egress fast path deletion '%d' failed [Error Code %d]\n", i, errCode);
	    	    return;
            }
            printf ("Debug: NETFP Egress fast path '%d' deleted successfully\n", i);
        }
    }

    /* Done cleaning up */
    return;
}

