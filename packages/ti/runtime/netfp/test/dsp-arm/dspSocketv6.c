/*
 *   @file  dspSocketv6.c
 *
 *   @brief
 *      Stress Test for NETFP IPv6 Sockets which is executed on the DSP
 *      cores.
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

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_IPV6_BASE_UDP_PORT          20000

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of UDP-IPv6 Packets received */
uint32_t        gNumIPv6PktRxed     = 0;

/* Global counter for the number of UDP packet dropped. */
uint32_t        gNumIPv6PktDropped  = 0;

/* Flag to display stats at runtime */
extern uint32_t gDisplayStats;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_getPayloadIPv6)
BENCHMARK_INIT(Netfp_sendIPv6)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle    netfp10KReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

/* Global MSGCOM Instance handle. */
extern Msgcom_InstHandle    appMsgcomInstanceHandle;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg   appResourceConfig;

/* Global Test status */
extern uint32_t        gTestCompletion;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("IPv6 Socket::::\n");

    /* Display the module statistics. */
    System_printf ("Number of IPv6 (UDP+GTPU) Pkt Received: %d\n", gNumIPv6PktRxed);
    System_printf ("Number of IPv6 (UDP+GTPU) Pkt Dropped : %d\n", gNumIPv6PktDropped);

    /* Display the benchmarking information for the NETFP Functions. */
    System_printf ("Benchmarking Information for IPv6 Stress Test:\n");
    BENCHMARK_DISPLAY(Netfp_getPayloadIPv6);
    BENCHMARK_DISPLAY(Netfp_sendIPv6);
    return;
}

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
    System_printf ("Debug: Notify function invoked for socket %p [Reason: '%s']\n", sockHandle, Netfp_getReasonString(reason));
    switch(reason)
    {
        case Netfp_Reason_L3_QOS:
        case Netfp_Reason_SP_ACTIVE:
        case Netfp_Reason_SA_DELETE:
            break;

        default:
            gTestCompletion = 1;
            /* Display socket statistics */
            Test_socketIPv6DisplayStats();
            break;
    }
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
    gNumIPv6PktRxed++;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Use the receive payload to get the data buffer: Take timestamps also. */
    BENCHMARK_START(Netfp_getPayloadIPv6);
    retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &peerSockAddress);
    BENCHMARK_END(Netfp_getPayloadIPv6);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to get the payload for packet 0x%p\n", ptrRxPkt);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_getPayloadIPv6);

    /* Display the peer socket address information only for the first packet */
    if (gNumIPv6PktRxed == 1)
    {
        /* Peer might be an IPv4 or IPv6 address? */
        if (peerSockAddress.sin_addr.ver == Netfp_IPVersion_IPV4)
        {
            System_printf ("Debug: Received Packet from Peer Address %d.%d.%d.%d Port = %d\n",
                           peerSockAddress.sin_addr.addr.ipv4.u.a8[0], peerSockAddress.sin_addr.addr.ipv4.u.a8[1],
                           peerSockAddress.sin_addr.addr.ipv4.u.a8[2], peerSockAddress.sin_addr.addr.ipv4.u.a8[3],
                           peerSockAddress.sin_port);
        }
        else
        {
            char ip6String[128];

            /* Convert the IPv6 address to string format: */
            Netfp_convertIP6ToStr (peerSockAddress.sin_addr.addr.ipv6, &ip6String[0]);
            System_printf ("Debug: Received Packet from Peer Address %s Port = %d\n", ip6String, peerSockAddress.sin_port);
        }
    }

    /* Send out the packet. */
    txDataBufferLen = rxDataBufferLen;

    /* Allocate another packet from the NETFP 10K Heap */
    ptrTxPkt = Pktlib_allocPacket(appPktlibInstanceHandle, netfp10KReceiveHeap, txDataBufferLen);
    if (ptrTxPkt == NULL)
    {
        gNumIPv6PktDropped++;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
        return -1;
    }

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrTxPkt, &ptrTxDataBuffer, &txDataBufferLen);

    /* Copy the data buffer */
    memcpy (ptrTxDataBuffer, ptrRxDataBuffer, txDataBufferLen);

    /* Writeback the data buffer */
    appWritebackBuffer(ptrTxDataBuffer, txDataBufferLen);

    /* Send out the socket on the appropriate socket: Take timestamps also. */
    BENCHMARK_START(Netfp_sendIPv6);
    retVal = Netfp_send (sockHandle, ptrTxPkt, 0, &errCode);
    BENCHMARK_END(Netfp_sendIPv6);
    if (retVal < 0)
    {
        System_printf ("Error: Unable to send the packet Error Code: %d\n", errCode);
        return -1;
    }

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_sendIPv6);

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
void Test_socketIPv6Task(UArg arg0, UArg arg1)
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
    Netfp_OptionTLV         sockOpt;
    uint32_t                enabledl3Shaper = 1;
    uint8_t                 sockPriority;

    /* Get L3 shaper setting */
    enabledl3Shaper = arg0;

    /* Get the ingress fast path handle: The fast paths are shared across different clients. */
    while (1)
    {
        fpIngressHandle = Netfp_findInboundFastPath(netfpClientHandle, "Ingress-IPv6-FastPath", &errCode);
        if (fpIngressHandle != NULL)
            break;
        Task_sleep(10);
    }

    /* Get the egress fast path handle: The fast paths are shared across different clients. */
    while (1)
    {
        fpEgressHandle = Netfp_findOutboundFastPath(netfpClientHandle, "Egress-IPv6-FastPath", &errCode);
        if (fpEgressHandle != NULL)
            break;
        Task_sleep(10);
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose.
     * Flows are specific to a NETFP Client. */
    strcpy (flowCfg.name, "SocketIPv6TestFlow");
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

    /* Create a unique channel name */
    sprintf (channelName, "Socket-IPv6-Channel-%d", DNUM);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == NULL)
    {
        System_printf ("Error: Unable to open the UDP Channel Error : %d\n", errCode);
        return;
    }

    /* Create a socket */
    sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET6, &errCode);
    if (sockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket(%p) has been created successfully\n", sockHandle);

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                = (TEST_SOCKET_IPV6_BASE_UDP_PORT + DNUM);
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = rxFlowId;
    sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(udpChannelHandle);
    sockAddress.op.bind.notifyFunction  = Test_notifySocketFunction;

    /* Bind the socket. */
    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET6;
    sockAddress.sin_port                    = (TEST_SOCKET_IPV6_BASE_UDP_PORT + DNUM);
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been connected successfully\n");

    /* Do we need to setup the L3 Shaper? */
    if ( (enabledl3Shaper == 1) && (DNUM ==0))
    {
        Netfp_FlowCfg       flowCfg;
        int32_t             rxFlowId;
        Netfp_IfHandle      ifHandle;
        Netfp_L3QoSCfg      l3QOSCfg;
        Netfp_OptionTLV     optCfg;
        uint32_t            i;

        /* NOTIFY the user: The DTS files of the kernel should be properly configured
         * for the L3 Shaper to operate. */
        System_printf ("-------------------------------------------------------------\n");
        System_printf ("NOTE: The L3 shaper requires the correct kernel DTS files    \n");
        System_printf ("The test assumes that the L3 QOS queues are 8000 onwards     \n");
        System_printf ("Please verify the kernel DTS files if this is NOT the case   \n");
        System_printf ("Assuming that the interface is eth0.                         \n");
        System_printf ("------------------------------------------------------------ \n");

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
            System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: Socket priority is %d [Handle is %p]\n", sockPriority, sockHandle);

        /* We need to create a flow for the L3 Shaper to work. */
        strcpy (flowCfg.name, "L3_QOS");
        flowCfg.numHeaps      = 1;
        flowCfg.heapHandle[0] = netfp10KReceiveHeap;
        flowCfg.sopOffset     = 0;

        /* Create the flow. */
        rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
        if (rxFlowId < 0)
        {
            System_printf ("Error: Unable to create the QOS flow [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: QOS Flow %d has been created successfully\n", rxFlowId);

        /* Initialize the L3 QOS configuration */
        memset ((void *)&l3QOSCfg, 0, sizeof(Netfp_L3QoSCfg));

        /* Setup the L3 shaper for the interface */
        ifHandle = Netfp_findInterface (netfpClientHandle, "eth0", NULL, &errCode);
        if (ifHandle == NULL)
        {
            System_printf ("Error: Interface eth0 NOT found; modify the test code with the correct interface [Error %d]\n", errCode);
            return;
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

        /* Setup the L3 shaper: */
        if (Netfp_setIfOpt (netfpClientHandle, ifHandle, &optCfg, &errCode) < 0)
        {
            System_printf ("Error: Unable to setup the L3 shaper [Error %d]\n", errCode);
            return;
        }

        /* Display the configuration on the console. */
        System_printf ("L3 Shaper configured as follows: \n");
        System_printf ("DSCP2  is mapped to QOS Queue %d i.e. hp-cos7\n", l3QOSCfg.qid[2]);
        System_printf ("DSCP63 is mapped to QOS Queue %d i.e. wrr-cos2\n", l3QOSCfg.qid[63]);
        System_printf ("Verify by displaying the QOS statistics\n");
    }

    /* Debug Message: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("IPv6 UDP Port         = %d\n",   sockAddress.sin_port);
    System_printf ("-------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (gTestCompletion == 0)
    {
        /* Extract the packet from the MSGCOM channel. */
        Msgcom_getMessage (udpChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Proceed only if there is a message pending. */
        if (ptrRxPkt != NULL)
            Test_socketProcessPacket(sockHandle, ptrRxPkt);

        if(gDisplayStats)
        {
            /* Display the statistics since the test is complete. */
            Test_socketIPv6DisplayStats();

            /* Reset the flag */
            gDisplayStats = 0;

            BENCHMARK_CLEAR(Netfp_sendIPv6);
        }
    }

    /* Display the statistics since the test is complete. */
    Test_socketIPv6DisplayStats();
}

