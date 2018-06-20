/*
 *   @file  socketv4.c
 *
 *   @brief
 *      Stress Test for UDP sockets using IPv4
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
#include <unistd.h>

/* PDK Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

#include "benchmark.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV4_BASE_UDP_PORT          2152

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of UDP-IPv4 Packets received */
uint32_t        gNumIPv4PktRxed     = 0;

/* Global counter for the number of UDP packet dropped. */
uint32_t        gNumIPv4PktDropped  = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_getPayloadIPv4)
BENCHMARK_INIT(Netfp_sendIPv4)

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
 *      Registered notify function registered with the socket to be invoked when
 *      the NETFP Server generates an event which affects the socket.
 *
 *  @param[in]  reason
 *      Reason because of which the notification function is invoked
 *  @param[in]  sockHandle
 *      Socket Handle affected by the notification
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_socketNotifyFunction (Netfp_Reason reason, Netfp_SockHandle sockHandle)
{
    Netfp_OptionTLV     optInfo;
    uint32_t            socketMTU;
    int32_t             errCode;

    /* Has there been an MTU change? */
    if ((reason == Netfp_Reason_IF_MTU_CHANGE) || (reason == Netfp_Reason_PMTU_CHANGE))
    {
        /* Get the socket MTU associated with the socket. */
        optInfo.type   = Netfp_Option_SOCK_MTU;
        optInfo.length = 4;
        optInfo.value  = (void*)&socketMTU;

        /* Get the socket option: */
        if (Netfp_getSockOpt (sockHandle, &optInfo, &errCode) < 0)
        {
            printf ("Error: Getting socket option failed [Error code %d]\n", errCode);
            return;
        }
        printf ("Debug: Test Socket MTU Reason [%s] is %d bytes\n", Netfp_getReasonString(reason), socketMTU);
    }
    else
    {
        /* Debug Message: */
        printf ("Debug: Test Socket Notified Reason: [%s]\n", Netfp_getReasonString(reason));
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is an exported function which is used to display the statistics related
 *      to the IPv4 Socket Tests.
 *
 *  @param[in]  sockHandle
 *      Socket Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Test_socketIPv4DisplayStats(Netfp_SockHandle sockHandle)
{
    Netfp_SocketStats           totalStats;
    Netfp_SocketStats           priorityStats;
    uint8_t                     priority;
    Netfp_ExtendedSocketStats   extendedStats;
    Netfp_OptionTLV             option;
    int32_t                     errCode;

    printf ("-------------------------------------------------------------\n");
    printf ("IPv4 Socket::::\n");

    /* Display the module statistics. */
    printf ("Number of IPv4 (UDP+GTPU) Pkt Received: %d\n", gNumIPv4PktRxed);
    printf ("Number of IPv4 (UDP+GTPU) Pkt Dropped : %d\n", gNumIPv4PktDropped);

    /* Display the benchmarking information for the NETFP Functions. */
    printf ("Benchmarking Information for IPv4 Stress Test:\n");
    BENCHMARK_DISPLAY(Netfp_getPayloadIPv4);
    BENCHMARK_DISPLAY(Netfp_sendIPv4);

    /* Get the socket priority: */
    option.type   = Netfp_Option_PRIORITY;
    option.length = sizeof(uint8_t);
    option.value  = &priority;

    /* Get the socket priority: */
    if (Netfp_getSockOpt (sockHandle, &option, &errCode) < 0)
        printf ("ERROR: Getting socket priority failed [Error code %d]\n", errCode);

    /* Populate the option to get priority statistics */
    option.type   = Netfp_Option_PRIORITY_STATISTICS;
    option.length = sizeof(Netfp_SocketStats);
    option.value  = &priorityStats;

    /* Get the socket priority statistics: */
    if (Netfp_getSockOpt (sockHandle, &option, &errCode) < 0)
        printf ("ERROR: Getting socket priority statistics failed [Error code %d]\n", errCode);

    /* Populate the option to get total statistics */
    option.type   = Netfp_Option_TOTAL_STATISTICS;
    option.length = sizeof(Netfp_SocketStats);
    option.value  = &totalStats;

    /* Get the socket total statistics: */
    if (Netfp_getSockOpt (sockHandle, &option, &errCode) < 0)
        printf ("ERROR: Getting socket total statistics failed [Error code %d]\n", errCode);

    /* Populate the option to get the extended statistics */
    option.type   = Netfp_Option_EXTENDED_STATISTICS;
    option.length = sizeof(Netfp_ExtendedSocketStats);
    option.value  = &extendedStats;

    /* Get the socket extended statistics: */
    if (Netfp_getSockOpt (sockHandle, &option, &errCode) < 0)
        printf ("ERROR: Getting socket extended statistics failed [Error code %d]\n", errCode);

    /* Sanity Testing: Ensure that the total statistics are in SYNC */
    if (memcmp ((void*)&extendedStats.totalStats, (void*)&totalStats, sizeof(Netfp_SocketStats)) != 0)
        printf ("ERROR: Mismatch in TOTAL statistics\n");

    /* Sanity Testing: Ensure that the priority statistics are in SYNC */
    if (memcmp ((void*)&extendedStats.priorityStats[priority], (void*)&priorityStats, sizeof(Netfp_SocketStats)) != 0)
        printf ("ERROR: Mismatch in PRIORITY statistics\n");

    /* Display out all the socket statistics */
    printf ("****************************************************************************\n");
    printf ("Extended Statistics:\n");
    printf ("Number of UDP Pkts  Transmitted                : %llu\n", extendedStats.numUDPTxPkts);
    printf ("Number of GTPU Pkts Transmitted                : %llu\n", extendedStats.numGTPUTxPkts);
    printf ("Number of 6-6 Pkts  Transmitted                : %llu\n", extendedStats.numTxPkts6in6Tunnel);
    printf ("Number of 4-4 Pkts  Transmitted                : %llu\n", extendedStats.numTxPkts4in4Tunnel);
    printf ("Number of 6-4 Pkts  Transmitted                : %llu\n", extendedStats.numTxPkts6in4Tunnel);
    printf ("Number of 4-6 Pkts  Transmitted                : %llu\n", extendedStats.numTxPkts4in6Tunnel);
    printf ("Number of IPv4 Fragments                       : %llu\n", extendedStats.numIPv4Frags);
    printf ("Number of IPv6 Fragments                       : %llu\n", extendedStats.numIPv6Frags);
    printf ("Number of IPv4 Fragments Failed                : %d\n",   extendedStats.numIPv4FragsFail);
    printf ("Number of IPv6 Fragments Failed                : %d\n",   extendedStats.numIPv6FragsFail);
    printf ("Total Number of IP Packets Transmitted         : %llu\n", extendedStats.totalStats.outIPPackets);
    printf ("Total Number of IP Octets  Transmitted         : %llu\n", extendedStats.totalStats.outIPOctets);
    printf ("Total Number of Eth Octets Transmitted         : %llu\n", extendedStats.totalStats.outEthOctets);
    printf ("Priority %d Number of IP Packets Transmitted   : %llu\n", priority, extendedStats.priorityStats[priority].outIPPackets);
    printf ("Priority %d Number of IP Octets  Transmitted   : %llu\n", priority, extendedStats.priorityStats[priority].outIPOctets);
    printf ("Priority %d Number of Eth Octets Transmitted   : %llu\n", priority, extendedStats.priorityStats[priority].outEthOctets);
    printf ("****************************************************************************\n");
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
    BENCHMARK_START(Netfp_getPayloadIPv4);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    BENCHMARK_END(Netfp_getPayloadIPv4);
    if (retVal < 0)
    {
        printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_getPayloadIPv4);

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
        gNumIPv4PktDropped++;
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
    BENCHMARK_START(Netfp_sendIPv4);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    BENCHMARK_END(Netfp_sendIPv4);
    if (retVal < 0)
    {
        printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_sendIPv4);

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
void* Test_socketThread(void* arg)
{
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockHandle        sockHandle;
    Netfp_SockAddr          sockAddress;
    Msgcom_ChannelCfg       chConfig;
    MsgCom_ChHandle         udpChannelHandle;
    Netfp_OptionTLV         sockOpt;
    Netfp_FlowCfg           flowCfg;
    int32_t                 rxFlowId;
    int32_t                 errCode;
    Ti_Pkt*                 ptrRxPkt;
    uint8_t                 sockPriority;

    /* Get the ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressHandle == NULL)
    {
        printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return NULL;
    }

    /* Get the egress fast path handle */
    fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressHandle == NULL)
    {
        printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return NULL;
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "SocketTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create ("Socket-IPv4-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == 0)
    {
        printf ("Error: Unable to open the LTE IPv4 Channel Error : %d\n", errCode);
        return NULL;
    }

    /* Create a socket */
	sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
	{
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
	    return NULL;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = TEST_SOCKET_IPV4_BASE_UDP_PORT;
    sockAddress.op.bind.flowId          = rxFlowId;
	sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle);
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.notifyFunction  = Test_socketNotifyFunction;

    /* Bind the socket. */
	if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
	    printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
		return NULL;
    }
	printf ("Debug: Socket has been bound successfully\n");

    /* Populate the connect information. */
	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = TEST_SOCKET_IPV4_BASE_UDP_PORT;
	sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
	if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
	    printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: Socket has been connected successfully\n");

    /* Initialize the socket option: */
    memset ((void *)&sockOpt, 0, sizeof(Netfp_OptionTLV));

    /* Initialize the socket priority: */
    sockPriority = 2;

    /* Populate the socket option: */
    sockOpt.type   = Netfp_Option_PRIORITY;
    sockOpt.length = 1;
    sockOpt.value  = &sockPriority;

    /* Setup the appropriate socket priority: */
    if (Netfp_setSockOpt (sockHandle, &sockOpt, &errCode) < 0)
    {
	    printf ("Error: NETFP Set Socket Priority Failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: Socket priority is %d [Handle is %p]\n", sockPriority, sockHandle);

    /* PMTU Testing: */
    {
        char        testSelection;
        uint8_t     dontFrag = 1;

        /* Should we execute the PMTU Tests: */
        printf ("Enable PMTU Testing [Y/N]:");
        scanf (" %c", &testSelection);

        if ((testSelection == 'y') || (testSelection == 'Y'))
        {
            /* Populate the socket option: */
            sockOpt.type   = Netfp_Option_DONT_FRAG;
            sockOpt.length = 1;
            sockOpt.value  = &dontFrag;

            /* Setup the appropriate socket priority: */
            if (Netfp_setSockOpt (sockHandle, &sockOpt, &errCode) < 0)
            {
    	        printf ("Error: NETFP Set DONT Frag Option Failed [Error Code %d]\n", errCode);
	        	return NULL;
            }
            printf ("Debug: Socket configured with DONT FRAG\n");
        }
    }

    /* Debug Message: */
    printf ("------------------------------------------------------------------------------\n");
    printf ("Please use a traffic generation application to send packets to the UDP Port %d\n", sockAddress.sin_port);
    printf ("------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        /* Check if a packets has been received on the the LTE channel */
        Msgcom_getMessage (udpChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Process the packet: */
        if (ptrRxPkt != NULL)
            Test_socketProcessPacket(sockHandle, ptrRxPkt);
    }

    /* Test has completed execution. Display the statistics */
    Test_socketIPv4DisplayStats (sockHandle);

    /* Shutdown the socket */
    printf ("Debug: Closing the NETFP Socket\n");
    if (Netfp_closeSocket (sockHandle, &errCode) < 0)
    {
	    printf ("Error: NETFP socket close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: NETFP Socket closed successfully\n");

    /* Shutdown the MSGCOM channel */
    errCode = Msgcom_delete (udpChannelHandle, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM UDP channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM UDP channel deleted successfully\n");

    /* TODO: Shutdown the NETFP Flow */

    return NULL;
}

