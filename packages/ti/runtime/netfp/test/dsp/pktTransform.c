/*
 *   @file  pktTransform.c
 *
 *   @brief
 *      The file implements the unit test which tests the NETFP API
 *      when the received packet is transformed.
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

/* MCSDK Include Files */
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

/* Benchmarking Include Files */
#include "benchmark.h"

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Base UDP port number for the test */
#define TEST_SOCKET_BASE_UDP_PORT               10000

/* Split Size: This will split the received packets into the specific size */
#define TEST_SPLIT_SIZE                         1000

/* Maximum number of chained packets supported. */
#define TEST_MAX_CHAINED_PKTS                   24

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Packet Transform Test Unit Selection: */
uint32_t    gPktTransformSelection  = 0;

/* This keeps track of the number of packets received which are ordered
 * on the basis of the number of packets in the chain. */
uint32_t    gPktReceiveCounter[TEST_MAX_CHAINED_PKTS]  = { 0 };

/* This keeps track of the number of packets transmited which are ordered
 * on the basis of the number of packets in the chain. */
uint32_t    gPktSendCounter[TEST_MAX_CHAINED_PKTS]  = { 0 };

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* MTU Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;
extern Pktlib_HeapHandle        netfp10KReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Global Application Resource Configuration: */
extern Resmgr_ResourceCfg          appResourceConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is called to split the packet
 *
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success -   Packet to be sent out
 *  @retval
 *      Error   -   NULL
 */
static Ti_Pkt* Test_splitPacket (Ti_Pkt* ptrRxPkt)
{
    Ti_Pkt*     ptrSplitPkt;
    Ti_Pkt*     ptrResultPkt;
    uint32_t    packetLen;
    uint32_t    numSplits;
    int32_t     index;
    uint32_t    splitSize;
    Ti_Pkt*     ptrPkt1[32];
    Ti_Pkt*     ptrPkt2[32];
    int32_t     retVal;

    /* Get the packet length of the received packet. */
    packetLen = Pktlib_getPacketLen(ptrRxPkt);

    /* Split the packet into multiple chunks only if the packet length > TEST_SPLIT_SIZE bytes. */
    if (packetLen < TEST_SPLIT_SIZE)
        return ptrRxPkt;

    /* Split the packet into multiple chunks only if the packet length is not a perfect multiple of TEST_SPLIT_SIZE */
    if ((packetLen % TEST_SPLIT_SIZE) == 0)
        return ptrRxPkt;

    /* We split the packet into equal sized chunks. */
    numSplits = (packetLen/TEST_SPLIT_SIZE) + 1;

    /* Determine the split size. */
    splitSize = TEST_SPLIT_SIZE;

    /* Create the splits */
    for (index = 0; index < numSplits; index++)
    {
        /* Allocate a zero buffer packet. */
        ptrSplitPkt = Pktlib_allocPacket (appPktlibInstanceHandle, mtuReceiveHeap, 0);
        if (ptrSplitPkt == NULL)
            return NULL;

        /* Split the packet */
        retVal = Pktlib_splitPacket (appPktlibInstanceHandle, ptrRxPkt, ptrSplitPkt, splitSize, &ptrPkt1[index], &ptrPkt2[index]);
        if (retVal < 0)
        {
            System_printf ("Error: Split Packet returned error %d\n", retVal);
            return NULL;
        }
        if (retVal == 1)
        {
            /* We are done. */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrSplitPkt);
            break;
        }

        /* Determine the next split size */
        packetLen = packetLen - splitSize;
        if (packetLen > splitSize)
            splitSize = TEST_SPLIT_SIZE;
        else
            splitSize = packetLen;

        /* We need to split the remaining packet further. */
        ptrRxPkt = ptrPkt2[index];
    }

    /* There should be at least 2 packets here; so merge them together. */
    ptrResultPkt = ptrPkt1[0];

    /* Now we merge all the packets again. */
    for (index = 1; index < numSplits; index++)
        ptrResultPkt = Pktlib_packetMerge (appPktlibInstanceHandle, ptrResultPkt, ptrPkt1[index], NULL);

    return ptrResultPkt;
}

/**
 *  @b Description
 *  @n
 *      The function is called to split the packet
 *
 *  @param[in]  ptrOrigPkt
 *      Pointer to the original received packet.
 *
 *  @retval
 *      Success -   Cloned Packet
 *  @retval
 *      Error   -   NULL
 */
static Ti_Pkt* Test_clonePacket (Ti_Pkt* ptrRxPacket)
{
    uint32_t    packetCount;
    Ti_Pkt*     ptrResultPkt;
    Ti_Pkt*     ptrClonePacket;
    uint32_t    index;

    /* Determine the number of packets which are chained. */
    packetCount = Pktlib_packetBufferCount(ptrRxPacket);

    /* Cycle through and create the clones */
    for (index = 0; index < packetCount; index++)
    {
        /* Allocate a zero buffer packet for the cloning operation. */
        ptrClonePacket = Pktlib_allocPacket(appPktlibInstanceHandle, mtuReceiveHeap, 0);
        if (ptrClonePacket == NULL)
            return NULL;

        /* Create a chain of clone packets. */
        if (index > 0)
            Pktlib_packetMerge (appPktlibInstanceHandle, ptrResultPkt, ptrClonePacket, NULL);
        else
            ptrResultPkt = ptrClonePacket;
    }

    /* Create the actual clone. */
    if (Pktlib_clonePacket (appPktlibInstanceHandle, ptrRxPacket, ptrResultPkt) < 0)
    {
        System_printf ("Error: Packet Cloning failed\n");
        return NULL;
    }
    return ptrResultPkt;
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
    uint8_t*    ptrPayload;
    uint32_t    payloadLen;
    int32_t     retVal;
    int32_t     errCode;
    Ti_Pkt*     ptrTransformPacket;
    uint32_t    numPktsInChain;

    /* Record the number of packets in the received chain. */
    numPktsInChain = Pktlib_packetBufferCount(ptrRxPkt);
    if (numPktsInChain >= TEST_MAX_CHAINED_PKTS)
    {
        /* Warning: Number of packets in the chain is limited to the MAX packets allowed in the chain. */
        System_printf ("Warning: Detected %d packets in chain test configured to handle only %d\n",
                       numPktsInChain, TEST_MAX_CHAINED_PKTS);
        numPktsInChain = TEST_MAX_CHAINED_PKTS;
    }

    /* Increment the statistics. */
    gPktReceiveCounter[numPktsInChain]++;

    /* Invalidate the data buffer of the received packet. We only need to invalidate the
     * networking headers so 1 cache line (128 bytes) should be good enough. */
    Pktlib_getDataBuffer (ptrRxPkt, &ptrPayload, &payloadLen);
    appInvalidateBuffer (ptrPayload, CACHE_L2_LINESIZE);

    /* Get the application payload:  */
    if (Netfp_getPayload (ptrRxPkt, 0, &ptrPayload, &payloadLen, NULL) < 0)
    {
        System_printf ("Error: Retreiving the application payload data failed\n");
        return -1;
    }

    /* Manipulate the packet as per the test selection */
    switch (gPktTransformSelection)
    {
        case 1:
        {
            /* No transformation of the packet. */
            ptrTransformPacket = ptrRxPkt;
            break;
        }
        case 2:
        {
            /* Transform the packet by splitting it into chunks */
            ptrTransformPacket = Test_splitPacket (ptrRxPkt);
            if (ptrTransformPacket == NULL)
            {
                /* Error: Unable to split the packet. Cleanup the memory */
                System_printf ("Error: LTE Split packet failed\n");
                Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
                return -1;
            }
            break;
        }
        case 3:
        {
            /* Create a clone of the received packet. */
            ptrTransformPacket = Test_clonePacket (ptrRxPkt);
            if (ptrTransformPacket == NULL)
            {
                /* Error: Unable to split the packet. Cleanup the memory */
                System_printf ("Error: LTE Clone packet failed\n");
                Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
                return -1;
            }
            break;
        }
        case 4:
        {
            Ti_Pkt* ptrTempPkt;

            /* Create a clone of the received packet. */
            ptrTransformPacket = Test_clonePacket (ptrRxPkt);
            if (ptrTransformPacket == NULL)
            {
                /* Error: Unable to split the packet. Cleanup the memory */
                System_printf ("Error: LTE Clone packet failed\n");
                Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
                return -1;
            }

            /* The original packet is being sent out */
            ptrTempPkt         = ptrTransformPacket;
            ptrTransformPacket = ptrRxPkt;
            ptrRxPkt           = ptrTempPkt;
            break;
        }
    }

    /* Send counter increment depending upon the number of packets in the transformed chain. */
    numPktsInChain = Pktlib_packetBufferCount(ptrTransformPacket);
    if (numPktsInChain >= TEST_MAX_CHAINED_PKTS)
    {
        /* Warning: Number of packets in the chain is limited to the MAX packets allowed in the chain. */
        System_printf ("Warning: Detected %d packets in chain test configured to handle only %d\n",
                       numPktsInChain, TEST_MAX_CHAINED_PKTS);
        numPktsInChain = TEST_MAX_CHAINED_PKTS;
    }
    gPktSendCounter[numPktsInChain]++;

    /* Send out the data on the socket. */
    retVal = Netfp_send(sockHandle, ptrTransformPacket, 0x0, &errCode);
    if (retVal < 0)
    {
        System_printf ("Error: LTE Channel Send failed\n");
        return -1;
    }

    /* Perform any cleanups depending upon the test selection */
    switch (gPktTransformSelection)
    {
        case 1:
        {
            /* No transformation: There is no need for any cleanup. This is done automatically by the CPDMA */
            break;
        }
        case 2:
        {
            /* Split Packet: All we need to do is run the garbage collector on the heap. */
            Pktlib_garbageCollection (appPktlibInstanceHandle, mtuReceiveHeap);
            break;
        }
        case 3:
        {
            /* Clone Packet: The clone packet is sent out and so we need to clean up the original
             * and also execute the garabage collector. */
            Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
            Pktlib_garbageCollection (appPktlibInstanceHandle, mtuReceiveHeap);
            break;
        }
        case 4:
        {
            /* Clone Packet: The original packet is sent out and so we need to clean up the
             * cloned packet and also execute the garabage collector. The cloned packet is
             * stored in the ptrRxPkt. */
            Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
            Pktlib_garbageCollection (appPktlibInstanceHandle, mtuReceiveHeap);
            break;
        }
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
void Test_pktTransformTask(UArg arg0, UArg arg1)
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
    Netfp_SockFamily        sockFamily;
    Pktlib_HeapHandle       flowHeapHandle;
    uint32_t                ipv4Mode = arg0;

    /* Are we executing in the IPv4 or IPv6 mode? */
    if (ipv4Mode == 1)
    {
        /* Display the banner: */
        System_printf ("Debug: Executing the packet transformation test in IPv4 mode\n");

        /* YES. Get the ingress fast path handle */
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

        /* Setup the socket family. */
        sockFamily = Netfp_SockFamily_AF_INET;

        /* Use the MTU receive heap to receive data for IPv4; this will allow us to receive and validate chained packets. */
        flowHeapHandle = mtuReceiveHeap;
    }
    else
    {
        /* Display the banner: */
        System_printf ("Debug: Executing the packet transformation test in IPv6 mode\n");

        /* NO. Get the ingress fast path handle */
        fpIngressHandle = Netfp_findInboundFastPath(netfpClientHandle, "Ingress-IPv6-FastPath", &errCode);
        if (fpIngressHandle == NULL)
        {
            System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
            return;
        }

        /* Get the egress fast path handle */
        fpEgressHandle = Netfp_findOutboundFastPath(netfpClientHandle, "Egress-IPv6-FastPath", &errCode);
        if (fpEgressHandle == NULL)
        {
            System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
            return;
        }

        /* Setup the socket family. */
        sockFamily = Netfp_SockFamily_AF_INET6;

        /* Use the 10K packet heap to receive data for IPv6; this is because we are currently not supporting chained
         * packets in the IPv6 send packet. */
        flowHeapHandle = mtuReceiveHeap;
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "PktTransformTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = flowHeapHandle;
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
    chConfig.mode                                                    = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                             = NULL;
    chConfig.msgcomInstHandle                                        = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                                = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    udpChannelHandle = Msgcom_create ("Socket-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (udpChannelHandle == NULL)
    {
        System_printf ("Error: Unable to open the UDP Channel Error : %d\n", errCode);
        return;
    }

    /* Create the socket on the basis of the operational mode */
    sockHandle = Netfp_socket (netfpClientHandle, sockFamily, &errCode);
    if (sockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));

    /* Select the socket family on the basis of the operational mode. */
    sockAddress.sin_family              = sockFamily;
    sockAddress.sin_port                = (TEST_SOCKET_BASE_UDP_PORT + DNUM);
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

    /* Populate the remaining fields */
    sockAddress.sin_family                  = sockFamily;
    sockAddress.sin_port                    = (TEST_SOCKET_BASE_UDP_PORT + DNUM);
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Socket has been connected successfully\n");

    /************************************************************************************************
     * CLI Simulation:
     ************************************************************************************************/
    while (1)
    {
        /* Display the MENU. */
        {
            /* IPv4 & IPv6: */
            System_printf ("*******************************************************\n");
            System_printf ("Packet Transform CLI Menu.\n");
            System_printf ("*******************************************************\n");
            System_printf ("1. No operation on the received packet\n");
            System_printf ("2. Split the packet into %d size\n", TEST_SPLIT_SIZE);
            System_printf ("3. Clone the packet and send original\n");
            System_printf ("4. Clone the packet and send clone\n");

            /* Get the test selection input */
            System_printf ("Enter test selection: ");
            scanf ("%d", &gPktTransformSelection);

            /* Validate the selection */
            if ((gPktTransformSelection <= 0) || (gPktTransformSelection > 4))
                continue;

            /* Valid Selection: */
            System_printf ("\n");
            break;
        }
    }

    /* Debug Message: */
    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Please use a traffic generation application to send packets to the UDP Port %d\n", sockAddress.sin_port);
    System_printf ("------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (1)
    {
        /* Extract the packet from the MSGCOM channel. */
        Msgcom_getMessage (udpChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Proceed only if there is a message pending. */
        if (ptrRxPkt == NULL)
        {
            /* Error: This should NOT happen; since the channel is a BLOCKING channel. */
            System_printf ("Error: MSGCOM Blocking channel failure\n");
            return;
        }

        /* Process the received packet. */
        Test_socketProcessPacket(sockHandle, ptrRxPkt);
    }
}

/**
 *  @b Description
 *  @n
 *      This is an exported function which is used to display the statistics related
 *      to the Packet Transformation tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_pktTransformDisplayStats(void)
{
    uint32_t    index;

    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Pkt Transform Tests::::\n");

    /* Cycle through and display the statistics. */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Receive Statistics\n");
    for (index = 1; index < TEST_MAX_CHAINED_PKTS; index++)
    {
        if (gPktReceiveCounter[index] != 0)
            System_printf ("Chain %d: Rx Packets %d\n", index, gPktReceiveCounter[index]);
    }

    /* Cycle through and display the statistics. */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Transmit Statistics\n");
    for (index = 1; index < TEST_MAX_CHAINED_PKTS; index++)
    {
        if (gPktSendCounter[index] != 0)
            System_printf ("Chain %d: Tx Packets %d\n", index, gPktSendCounter[index]);
    }
    return;
}


