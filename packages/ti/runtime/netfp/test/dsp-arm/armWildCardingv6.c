/*
 *   @file  armWildCardingv6.c
 *
 *   @brief
 *      Stress Test for UDP sockets using IPv6 on the ARM client
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013 Texas Instruments, Inc.
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

/* Benchmarking Include Files */
#include "armBenchmark.h"

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of UDP-IPv6 Packets received */
uint32_t        gNumWcIPv6PktRxed     = 0;

/* Global counter for the number of UDP packet dropped. */
uint32_t        gNumWcIPv6PktDropped  = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(WC_Netfp_getPayloadIPv6)
BENCHMARK_INIT(WC_Netfp_sendIPv6)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        netfpDataRxHeap;
extern uint32_t                 testComplete;
extern Netfp_ClientHandle       netfpClientHandle;
extern uint32_t                 gPortNumber;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

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
static void Test_socketNotify (Netfp_Reason reason, Netfp_SockHandle sockHandle)
{
    printf ("Debug: Notify function invoked for socket %p [Reason: '%s']\n", sockHandle, Netfp_getReasonString(reason));
    switch(reason)
    {
        case Netfp_Reason_L3_QOS:
        case Netfp_Reason_SP_ACTIVE:
            break;

        default:
            testComplete = 1;
            break;
    }
}

/**
 *  @b Description
 *  @n
 *      This is an exported function which is used to display the statistics related
 *      to the IPv6 Socket Tests.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_socketIPv6DisplayStats(void)
{
    printf ("-------------------------------------------------------------\n");
    printf ("IPv6 Wild Carding Socket::::\n");

    /* Display the module statistics. */
    printf ("Number of IPv6 (UDP+GTPU) Pkt Received: %d\n", gNumWcIPv6PktRxed);
    printf ("Number of IPv6 (UDP+GTPU) Pkt Dropped : %d\n", gNumWcIPv6PktDropped);

    /* Display the benchmarking information for the NETFP Functions. */
    printf ("Benchmarking Information for IPv6 Stress Test:\n");
    BENCHMARK_DISPLAY(WC_Netfp_getPayloadIPv6);
    BENCHMARK_DISPLAY(WC_Netfp_sendIPv6);
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
    gNumWcIPv6PktRxed++;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Use the receive payload to get the data buffer: Take timestamps also. */
    BENCHMARK_START(WC_Netfp_getPayloadIPv6);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    BENCHMARK_END(WC_Netfp_getPayloadIPv6);
    if (retVal < 0)
    {
        printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(WC_Netfp_getPayloadIPv6);

    /* Display the peer socket address information only for the first packet */
    if (gNumWcIPv6PktRxed == 1)
    {
        /* Peer might be an IPv4 or IPv6 address? */
        if (peerSockAddress.sin_addr.ver == Netfp_IPVersion_IPV4)
        {
            printf ("Debug: Received Packet from Peer Address %d.%d.%d.%d Port = %d\n",
                    peerSockAddress.sin_addr.addr.ipv4.u.a8[0], peerSockAddress.sin_addr.addr.ipv4.u.a8[1],
                    peerSockAddress.sin_addr.addr.ipv4.u.a8[2], peerSockAddress.sin_addr.addr.ipv4.u.a8[3],
                    peerSockAddress.sin_port);
        }
        else
        {
            char ip6String[128];

            /* Convert the IPv6 address to string format: */
            Netfp_convertIP6ToStr (peerSockAddress.sin_addr.addr.ipv6, &ip6String[0]);
            printf ("Debug: Received Packet from Peer Address %s Port = %d\n", ip6String, peerSockAddress.sin_port);
        }
    }

    /* Send out the packet. */
    txDataBufferLen = rxDataBufferLen;

    /* Allocate another packet from the NETFP Data Heap */
    ptrTxPkt = Pktlib_allocPacket(appPktlibInstanceHandle, netfpDataRxHeap, txDataBufferLen);
    if (ptrTxPkt == NULL)
    {
        gNumWcIPv6PktDropped++;
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
    BENCHMARK_START(WC_Netfp_sendIPv6);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    BENCHMARK_END(WC_Netfp_sendIPv6);
    if (retVal < 0)
    {
        printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(WC_Netfp_sendIPv6);

    /* Clean up the received packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the test thread which send and receives IPv6 packets.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_wildCarding6Thread(void* arg)
{
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockHandle        sockHandle;
    Netfp_SockAddr          sockAddress;
    Msgcom_ChannelCfg       chConfig;
    MsgCom_ChHandle         udpChannelHandle;
    Netfp_FlowCfg           flowCfg;
    int32_t                 rxFlowId;
    int32_t                 errCode;
    Ti_Pkt*                 ptrRxPkt;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Get the ingress fast path handle: This is shared by the NETFP clients. */
    while (1)
    {
        fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv6-WcFastPath", &errCode);
        if (fpIngressHandle != NULL)
            break;
        usleep(100);
    }

    /* Get the egress fast path handle: This is shared by the NETFP clients. */
    while (1)
    {
        fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv6-FastPath2", &errCode);
        if (fpEgressHandle != NULL)
            break;
        usleep(100);
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "SocketIPv6WcTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        printf ("Error: Unable to create the flow for IPv6 wildcardiing socket [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Flow %d  for IPv6 wildcarding has been created successfully\n", rxFlowId);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create a unique channel name */
    sprintf (channelName, "Socket-IPv6-WcChannel-%d", getpid());

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == 0)
    {
        printf ("Error: Unable to open the LTE IPv6 Channel Error : %d\n", errCode);
        return NULL;
    }

    /* Create a socket */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
    if (sockHandle == NULL)
    {
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                = gPortNumber + 100 + 1;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = rxFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle);
    sockAddress.op.bind.notifyFunction  = Test_socketNotify;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                    = gPortNumber + 100 + 1;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Socket has been connected successfully\n");

    /* Debug Message: */
    printf ("-------------------------------------------------------------\n");
    printf ("IPv6 Wild Carding UDP Port         = %d\n",   sockAddress.sin_port);
    printf ("-------------------------------------------------------------\n");

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
    Test_socketIPv6DisplayStats ();
    return NULL;
}

