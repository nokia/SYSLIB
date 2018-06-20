/*
 *   @file  wildcardingv4.c
 *
 *   @brief   
 *      Stress Test for NETFP IPv4 wildcarding
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
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/platform/platform.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/* Benchmarking Include Files */
#include "benchmark.h"

/* Test Include Files */
#include "netCfg.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV4_BASE_UDP_PORT          10050

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/
typedef struct testStats{
    /* Global Counter for number of UDP-IPv4 Packets received */
    uint32_t        numIPv4PktRxed;
    
    /* Global counter for the number of UDP packet dropped. */
    uint32_t        numIPv4PktDropped;
    
    /* Global Counter for number of UDP-IPv4 Packets received on wild 
       carding fast path */
    uint32_t        numWcIPv4PktRxed;
    
    /* Global counter for the number of UDP packet droppedon wild 
       carding fast path */
    uint32_t        numWcIPv4PktDropped;
}testStats;

/* Test statistics */
testStats gStats={0};

/* Global Event Object which keeps track of IPv4-UDP events.*/
Event_Handle    ipv4EventObject;

/* Global Variables which keep track of the NETFP functions being benchmarked */ 
BENCHMARK_INIT(Netfp_getPayloadIPv4)
BENCHMARK_INIT(Netfp_sendIPv4)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        netfp10KReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* NETFP Configuration after parsing the DAT file. */
extern Test_NetfpConfigInfo    netfpConfig;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Callback function registered with the UDP MSGCOM channel which 
 *      is invoked when a UDP packet is received.
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *
 *  @retval
 *      Not Applicable
 */
static void Test_udpChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post(ipv4EventObject, arg);
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to send the received packet back to the sender.
 *
 *  @param[in]  sockHandle
 *      Socket Handle on which the packet was received
 *  @param[in]  ptrPayload
 *      Pointer to the received packet payload.
 *  @param[in]  txDataBufferLen
 *      Data buffer length.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_socketSendPacket (Netfp_SockHandle sockHandle, uint8_t* ptrPayload, uint32_t txDataBufferLen)
{
    int32_t         retVal;
    int32_t         errCode;
    Ti_Pkt*         ptrTxPkt;
    uint8_t*        ptrTxDataBuffer;

    /* Allocate another packet from the NDK Heap */
    ptrTxPkt = Pktlib_allocPacket(appPktlibInstanceHandle, netfp10KReceiveHeap, txDataBufferLen);
    if (ptrTxPkt == NULL)
    {
        gStats.numIPv4PktDropped++;
        return -1;
    }

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrTxPkt, &ptrTxDataBuffer, &txDataBufferLen);

    /* Copy the data buffer */
    memcpy (ptrTxDataBuffer, ptrPayload, txDataBufferLen);

    /* Writeback the data buffer */
    appWritebackBuffer(ptrTxDataBuffer, txDataBufferLen);

    /* Set the packet & data buffer length. */
    Pktlib_setPacketLen(ptrTxPkt, txDataBufferLen);
    Pktlib_setDataBufferLen(ptrTxPkt, txDataBufferLen);

    /* Send out the socket on the appropriate socket: Take timestamps also. */
    BENCHMARK_START(Netfp_sendIPv4);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    BENCHMARK_END(Netfp_sendIPv4);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_sendIPv4);       

    return 0;
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
static int32_t Test_socketProcessPacket 
(
    Netfp_SockHandle    sockHandle, 
    Ti_Pkt*             ptrRxPkt,
    uint8_t**           ptrPayload, 
    uint32_t*           dataLen, 
    Netfp_PeerSockAddr* ptrPeerAddress
)
{
    uint8_t*            ptrRxDataBuffer;
    uint32_t            rxDataBufferLen;
    int32_t             retVal;
    Netfp_PeerSockAddr  peerSockAddress;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Use the receive payload to get the data buffer: Take timestamps also. */
    BENCHMARK_START(Netfp_getPayloadIPv4);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    BENCHMARK_END(Netfp_getPayloadIPv4);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_getPayloadIPv4);

    /* Display the peer socket address information only for the first packet */
    if (gStats.numIPv4PktRxed == 1)
    {
        System_printf ("Debug: Received Packet from Peer Address %d.%d.%d.%d Port = %d\n", 
                       peerSockAddress.sin_addr.addr.ipv4.u.a8[0], peerSockAddress.sin_addr.addr.ipv4.u.a8[1], 
                       peerSockAddress.sin_addr.addr.ipv4.u.a8[2], peerSockAddress.sin_addr.addr.ipv4.u.a8[3],
                       peerSockAddress.sin_port);
    }

    /* Update packet info to caller */ 
    *ptrPayload = ptrRxDataBuffer;
    *dataLen = rxDataBufferLen; 
    if(ptrPeerAddress != NULL)
    {
        memcpy((void *)ptrPeerAddress, &peerSockAddress.sin_addr, sizeof(Netfp_PeerSockAddr));
    }

    /* Clean up the received packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
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
void Test_wildCardingTask(UArg arg0, UArg arg1)
{
    Netfp_InboundFPHandle       fpIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle;
    Netfp_InboundFPHandle       fpWcIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle2;
    Netfp_SockHandle            sockHandle;
    Netfp_SockHandle            WcSockHandle;
    Netfp_SockAddr              sockAddress;
    Msgcom_ChannelCfg           chConfig;
    MsgCom_ChHandle             udpChannelHandle;
    MsgCom_ChHandle             udpWcChannelHandle;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     errCode;
    Ti_Pkt*                     ptrRxPkt;
    Error_Block                 eb;
    uint32_t                    events;
    uint32_t                    payloadLen;
    Netfp_PeerSockAddr          ptrSrcIP;
    uint8_t*                    ptrPayload;

    /* Get the ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
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

    /* Get the ingress Wild carding fast path handle */
    fpWcIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-WcFastPath", &errCode);
    if (fpWcIngressHandle == NULL)
    {
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Get the egress fast path handle */
    fpEgressHandle2 = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath2", &errCode);
    if (fpEgressHandle2 == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Create the Event Object  */
    Error_init(&eb);
    ipv4EventObject = Event_create(NULL, &eb);
    if (ipv4EventObject == NULL) 
    {
        System_printf ("Error: IPv4 Event Object creation failed\n");
        return;
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "SocketTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfp10KReceiveHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        System_printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_udpChannelCallback;
    chConfig.arg                                                    = Event_Id_00;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create ("Socket-IPv4-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == NULL)
    {
        System_printf ("Error: Unable to open the UDP Channel Error : %d\n", errCode);
        return;
    }

    /* Initialize the wild carding channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_udpChannelCallback;
    chConfig.arg                                                    = Event_Id_01;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the Message communicator channel. */
    udpWcChannelHandle = Msgcom_create ("Socket-IPv4-WcChannel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == NULL)
    {
        System_printf ("Error: Unable to open the UDP Channel Error : %d\n", errCode);
        return;
    }

    /* Create a socket */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (sockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }
    
    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM);
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = rxFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle);
    
    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been bound successfully\n");
    
    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM);
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been connected successfully\n");

    /* Debug Message: */
    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Please use a traffic generation application to send packets to the UDP Port %d\n", sockAddress.sin_port);    
    System_printf ("------------------------------------------------------------------------------\n");

    /* Create a wild carding socket */
    WcSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (WcSockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }
    
    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM * 2);
    sockAddress.op.bind.inboundFPHandle = fpWcIngressHandle;
    sockAddress.op.bind.flowId          = rxFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpWcChannelHandle);
    
    /* Bind the socket. */
    if (Netfp_bind (WcSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been bound successfully\n");
    
    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = (TEST_SOCKET_IPV4_BASE_UDP_PORT + DNUM * 2);
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle2;
    
    /* Connect the socket */
    if (Netfp_connect(WcSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Wild carding Socket has been connected successfully\n");

    /* Debug Message: */
    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Please use a traffic generation application to send packets to the wild carding UDP Port %d\n", sockAddress.sin_port);
    System_printf ("------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the 
     *  sockets.
     *********************************************************************************/
    while (1)
    {

        /* Wait for a UDP packet to be received. */
        events = Event_pend(ipv4EventObject, Event_Id_NONE, Event_Id_00 + Event_Id_01, BIOS_WAIT_FOREVER);

        /* Did we get a packet from normal UDP channel ? */
        if (events & Event_Id_00)
        {
            while(1)
            {
                /* Extract the packet from the MSGCOM channel. */ 
                Msgcom_getMessage (udpChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);
    
                /* Proceed only if there is a message pending. */
                if (ptrRxPkt == NULL)
                    break;
    
                /* Process the received packet. */
                if (Test_socketProcessPacket(sockHandle, ptrRxPkt, &ptrPayload, &payloadLen, NULL) < 0 )
                {
                   gStats.numIPv4PktDropped++;
                   Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                   continue;
               }
               else
               {
                   /* Increment the statistics. */
                   gStats.numIPv4PktRxed++;
               }
    
               /* Send the packet to originator */
               Test_socketSendPacket(sockHandle, ptrPayload, payloadLen);
    
               /* Cleanup the received packet */
               Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
            }
        }

        /* Did we get a wild carded UDP channel ? */
        if (events & Event_Id_01)
        {
            /* YES: wild carded UDP Packet was received; cycle through and process all the received packets. */
            while (1)
            {
                /* Extract the packet from the MSGCOM channel. */
                Msgcom_getMessage (udpWcChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

                /* Proceed only if there is a message pending. */
                if (ptrRxPkt == NULL)
                    break;

                /* Process the received packet. */
                if (Test_socketProcessPacket(WcSockHandle, ptrRxPkt, &ptrPayload, &payloadLen, &ptrSrcIP) < 0)
                {
                    gStats.numWcIPv4PktDropped++;
                    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                    continue;
                }
                else
                {
                    /* Increment the statistics. */
                    gStats.numWcIPv4PktRxed++;
                }
                
                /* for srcIP = pdnGwIPAddress, use sockHandle */ 
                if( memcmp((void *)&ptrSrcIP.sin_addr.addr.ipv4.u.a32, (void *)&netfpConfig.pdnGwIPAddress[0], 4) == 0 )
                {
                    /* Send the packet to originator */
                    Test_socketSendPacket(sockHandle, ptrPayload, payloadLen);
                }
                
                /* for srcIP = pdnGw2IPAddress, use WcSockHandle */ 
                if( memcmp((void *)&ptrSrcIP.sin_addr.addr.ipv4.u.a32, (void *)&netfpConfig.pdnGw2IPAddress[0], 4)  == 0 )
                {
                    /* Send the packet to originator */
                    Test_socketSendPacket(WcSockHandle, ptrPayload, payloadLen);
                }

                /* Cleanup the received packet */
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
            }
        }
    }
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
void Test_wildCardingIPv4DisplayStats(void)
{
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("IPv4 Wild Carding::::\n");

    /* Display the module statistics. */
    System_printf ("Number of IPv4 (UDP+GTPU) Pkt Received on normal channel: %d\n", gStats.numIPv4PktRxed);
    System_printf ("Number of IPv4 (UDP+GTPU) Pkt Dropped  on normal channel: %d\n", gStats.numIPv4PktDropped);

    System_printf ("Number of IPv4 (UDP+GTPU) Pkt Received on Wildcarding channel: %d\n", gStats.numWcIPv4PktRxed);
    System_printf ("Number of IPv4 (UDP+GTPU) Pkt Dropped  on Wildcarding channel: %d\n", gStats.numWcIPv4PktDropped);

    /* Display the benchmarking information for the NETFP Functions. */
    System_printf ("Benchmarking Information for IPv4 Stress Test:\n");
    BENCHMARK_DISPLAY(Netfp_getPayloadIPv4);
    BENCHMARK_DISPLAY(Netfp_sendIPv4);
    return;
}

