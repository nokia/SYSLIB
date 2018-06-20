/*
 *   @file  stress_reader.c
 *
 *   @brief   
 *      Reader Test: The file implements the functionality which stress
 *      tests the virtual channel over a PHYSICAL accumulated channel and
 *      collects the benchmarking information.
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
 *
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
#include <ti/sysbios/knl/Clock.h>
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
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>

/* Test specific include files */
#include "test_config.h"
#include "benchmark.h"

/**********************************************************************
 ************************** Reader Globals ****************************
 **********************************************************************/

/* Global Benchmark information which records the time taken to receive
 * a message. */
BENCHMARK_INIT(Msgcom_getMessage)

/* Global list of virtual channel handles which are being tested. */
MsgCom_ChHandle        gVirtualChannelHandle[TEST_MAX_VIRTUAL_CHANNELS];

/**********************************************************************
 ************************** Extern Definitions ************************
 **********************************************************************/

/* Buffer Invalidation Functions: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/* Resource Manager Configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* MSGCOM Instance handle. */
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

uint32_t getUserInput(void);
/**********************************************************************
 ************************** Reader Functions **************************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Initialization of the MSGCOM channels for the stress test
 *
 *  @param[in]  channelType
 *      Channel type for the MSGCOM channel.
 *  @param[in]  interruptMode
 *      Interrupt mode for the physical channel
 *  @param[in]  msgcomDataHeap
 *      Heap Handle from which packets received will be allocated from in the
 *      case of MSGCOM Queue DMA channels.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t readerStressInitChannels
(
    Msgcom_ChannelType          channelType,
    Msgcom_QueueInterruptMode   interruptMode,
    Pktlib_HeapHandle           msgcomDataHeap
)
{
    char                   virtualChannelName[MSG_COM_MAX_CHANNEL_LEN];
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    int32_t                errCode;
    int32_t                index;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* MSGCOM channel configuration. */
    if (channelType == Msgcom_ChannelType_QUEUE)
    {
        /* MSGCOM Queue Channel */
        if (interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT)
        {
            /* Accumulated channel configuration */
            chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.appCallBack                                        = NULL;
            chConfig.msgcomInstHandle                                   = appMsgcomInstanceHandle;
            chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 5;
	        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 0;
        }
        else
        {
            /* Direct Interrupt channel configuration */
            chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.appCallBack                                            = NULL;
            chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
            chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;
        }
    }
    else
    {
        /* MSGCOM Queue DMA Channel */
        channelType = Msgcom_ChannelType_QUEUE_DMA;

        /* Populate the channel configuration on the basis of the interrupt mode. */
        if (interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT)
        {
            /* Accumulated channel configuration */
            chConfig.mode                                                     = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.appCallBack                                              = NULL;
            chConfig.msgcomInstHandle                                         = appMsgcomInstanceHandle;
            chConfig.u.queueDMACfg.interruptMode                              = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
            chConfig.u.queueDMACfg.rxFreeQueueNum                             = (Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(msgcomDataHeap))).qNum;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries    = 5;
        	chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pacingTimerCount  = 0;
        }
        else
        {
            /* Direct Interrupt channel configuration */
            chConfig.mode                                                           = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.appCallBack                                                    = NULL;
            chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
            chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
            chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(msgcomDataHeap))).qNum;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[0].queue;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[0].cpIntcId;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[0].systemInterrupt;
            chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[0].hostInterrupt;
        }
    }

    /* Create the PHYSICAL Channel. */
    phyChHandle = Msgcom_create("StressTestChannel", channelType, &chConfig, &errCode);
    if (phyChHandle == NULL)
    {
        System_printf ("Error: Unable to create the PHYSICAL channel Error : %d\n", errCode);
        return -1;
    }

    /* We will now create the specific number of virtual Channels over this PHYSICAL Channel */
    for (index = 0; index < TEST_MAX_VIRTUAL_CHANNELS; index++)
    {
        /* Create a unique channel name. */
        sprintf (virtualChannelName, "StressTest-Virtual-%d", index);

        /* Create the 'virtual' channel configuration. */
        chConfig.mode                              = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.appCallBack                       = NULL;
        chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
        chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

        /* Create a virtual channel. */
        gVirtualChannelHandle[index] = Msgcom_create(&virtualChannelName[0], Msgcom_ChannelType_VIRTUAL, &chConfig, &errCode);
        if (gVirtualChannelHandle[index] == NULL)
        {
            System_printf ("Error: Unable to create virtual channel Error : %d\n", errCode);
            return -1;
        }
    }

    /* Channels have been created */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Reader Stress Test which receives messages over the channel and collects
 *      the benchmarking information.
 *
 *  @param[in]  msgcomDataHeap
 *      Heap Handle from which packets are received and which needs to be monitored
 *      for memory leaks. 
 *
 *  @retval
 *      Not Applicable.
 */
static void readerStressTest(Pktlib_HeapHandle msgcomDataHeap)
{
    int32_t                 index;
    Ti_Pkt*                 ptrMessage;
    uint32_t*               ptrDataBuffer;
    uint32_t                dataLen;
    uint32_t                messageId[TEST_MAX_VIRTUAL_CHANNELS];
    uint32_t                numMessagesRxed = 0;
    uint32_t                numUnexpectedMsgIdRxed = 0;
    int32_t                 errCode;

    /* Initialize the message identifier */
    memset ((void*)&messageId, 0, sizeof(messageId));

    /* Execute the test */
    while (1)
    {
        /* Loop around and check all the virtual channels for a message */
        for (index = 0; index < TEST_MAX_VIRTUAL_CHANNELS; index++)
        {
            /* Get the message and take a snapshot of the time taken */
            BENCHMARK_START(Msgcom_getMessage);
            Msgcom_getMessage (gVirtualChannelHandle[index], (MsgCom_Buffer**)&ptrMessage);
            BENCHMARK_END(Msgcom_getMessage);
            BENCHMARK_UPDATE(Msgcom_getMessage);

            /* Did we get a message? If so process the message. */
            if (ptrMessage != NULL)
                break;
        }

        /* Check if we got a message and if not continue looping */
        if (ptrMessage == NULL)
            continue;

        /* Increment the number of messages which have been received. */
        numMessagesRxed++;

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Check if we received the special end of test message */
        if (*ptrDataBuffer == 0xDEADDEAD)
        {
            /* Cleanup the message. */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);
            break;
        }

        /* Each message has a unique message identifier. Here we keep track and ensure that 
         * we are receiving the messages in order. */
        if (*ptrDataBuffer != messageId[index])
            numUnexpectedMsgIdRxed++;

        /* Initialize the message identifier tracker */
        messageId[index] = *ptrDataBuffer + 1;

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);
    }

    /* Display the reader benchmark information */
    System_printf ("*******************************************************\n");
    System_printf ("Reader Benchmark Information:\n");
    BENCHMARK_DISPLAY(Msgcom_getMessage);
    System_printf ("Debug: Number of unexpected messages received: %d\n", numUnexpectedMsgIdRxed);
    System_printf ("*******************************************************\n");

    /* Control comes here implies that we received the end of life packet. So we can discount this off */
    numMessagesRxed = numMessagesRxed - 1;

    /* Sanity Check: Did the reader get all the messages */
    if (numMessagesRxed != TEST_MAX_ITERATIONS)
        System_printf ("Error: Writer sent %d messages but only %d was received\n", TEST_MAX_ITERATIONS, numMessagesRxed);
    else
        System_printf ("Debug: Sucessfully received all messages %d\n", numMessagesRxed);

    /* Get the MSGCOM Data Heap: This is the heap which is used to allocate messages 
     * exchanged between the reader and writer. We display the statistics at the end
     * of the test and check for memory leaks. */
    msgcomDataHeap = Pktlib_findHeapByName(appPktlibInstanceHandle, "Message_DataHeap", &errCode);
    if (msgcomDataHeap == NULL)
    {
        System_printf ("FATAL Error: Unable to find the MSGCOM Data Heap [Error code %d]\n", errCode);
        return;
    }
}

/**
 *  @b Description
 *  @n  
 *      The function implements the simply CLI interface to get the test configuration.
 *
 *  @param[out]  channelType
 *      Channel type for the MSGCOM channel.
 *  @param[out]  interruptMode
 *      Interrupt mode for the physical channel
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t readerGetConfig
(
    Msgcom_ChannelType*         channelType, 
    Msgcom_QueueInterruptMode*  interruptMode
)
{
    uint32_t    input;

    /* CLI Simulation: */
    System_printf ("*******************************************************\n");
    System_printf ("MSGCOM Virtual Channel Test Menu \n");
    System_printf ("Please select the Channel configuration for the test\n");

    /* Select the Physical channel interrupt mode. */
    System_printf ("Physical Channel Type [1 for Queue-DMA; all other values are Queue]: ");
    input = getUserInput();
    if( input == UINT32_MAX )
    {
        System_printf ("Error: wrong input param 1\n");
        return -1;
    }
    if (input == 1)
        *channelType = Msgcom_ChannelType_QUEUE_DMA;
    else
        *channelType = Msgcom_ChannelType_QUEUE;

    /* Select the Physical channel interrupt mode. */
    System_printf ("Physical Channel Interrupt Mode [2 for Direct; all other values are Accumulated]: ");
    input = getUserInput();
    if( input == UINT32_MAX )
    {
        System_printf ("Error: wrong input param 2\n");
        return -1;
    }
    if (input == 2)
        *interruptMode = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    else
        *interruptMode = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;

    /* Configuration is complete. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Reader Stress Test for virtual accumulated channels.
 *
 *  @retval
 *      Not Applicable.
 */
void readerStressTask(UArg arg0, UArg arg1)
{
    Pktlib_HeapHandle           msgcomDataHeap;
    Msgcom_ChannelType          channelType;
    Msgcom_QueueInterruptMode   interruptMode;
    int32_t                     errCode;
    Pktlib_HeapStats            startHeapStats;
    Pktlib_HeapStats            endHeapStats;

    /* Get the MSGCOM Data Heap: This is the heap which is used to allocate messages 
     * exchanged between the reader and writer. */
    msgcomDataHeap = Pktlib_findHeapByName(appPktlibInstanceHandle, "Message_DataHeap", &errCode);
    if (msgcomDataHeap == NULL)
    {
        System_printf ("FATAL Error: Unable to find the MSGCOM Data Heap\n");
        return;
    }

    /* Get the heap statistics at the beginning of the test */
    Pktlib_getHeapStats(msgcomDataHeap, &startHeapStats);

    /* Initialize the CLI */
    if (readerGetConfig(&channelType, &interruptMode) < 0)
        return;

    /* Initialize the MSGCOM channels which are to be tested. */
    if (readerStressInitChannels(channelType, interruptMode, msgcomDataHeap) < 0)
        return;

    /* Debug Message: */
    System_printf ("*******************************************************\n");
    System_printf ("Stress Testing: \n");
    System_printf ("Physical channel type          : %s \n", (channelType == Msgcom_ChannelType_QUEUE) ? "Queue" : "Queue-DMA");
    System_printf ("Physical channel Interrupt Mode: %s \n", (interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT) ? "Accumulated" : "Direct");
    System_printf ("Number of virtual channels     : %d\n", TEST_MAX_VIRTUAL_CHANNELS);
    System_printf ("Burst Packets                  : %d\n", TEST_MAX_BURST);
    System_printf ("*******************************************************\n");

    /* Perform the test. */
    readerStressTest(msgcomDataHeap);

    /* Get the heap statistics at the end of the test  */
    Pktlib_getHeapStats(msgcomDataHeap, &endHeapStats);

    /* Ensure that there are no memory leaks */
    if ( (endHeapStats.numFreeDataPackets   != startHeapStats.numFreeDataPackets)   || 
         (endHeapStats.numZeroBufferPackets != startHeapStats.numZeroBufferPackets) ||
         (endHeapStats.numPacketsinGarbage  != startHeapStats.numPacketsinGarbage)
       )
    {
        System_printf ("Error: Memory Leak Detected.\n");

        /* Dump the Heap Statistics at the beginning and end of the test  */
        System_printf ("-----------------------------------------------------------------------------\n");
        System_printf ("Heap Statistics for Heap 0x%x @ the Start of the test\n", msgcomDataHeap);
        System_printf ("Free Data Buffer Packets              :%d\n", startHeapStats.numFreeDataPackets);
        System_printf ("Data Buffer Threshold Status          :%d\n", startHeapStats.dataBufferThresholdStatus);
        System_printf ("Data Buffer Starvation Counter        :%d\n", startHeapStats.dataBufferStarvationCounter);
        System_printf ("Zero Data Buffer Packets              :%d\n", startHeapStats.numZeroBufferPackets);
        System_printf ("Zero Data Buffer Threshold Status     :%d\n", startHeapStats.zeroDataBufferThresholdStatus);
        System_printf ("Zero Data Buffer Starvation Counter   :%d\n", startHeapStats.zeroDataBufferStarvationCounter);
        System_printf ("Number of Packets in Garbage          :%d\n", startHeapStats.numPacketsinGarbage);

        System_printf ("Heap Statistics for Heap 0x%x @ the End of the test\n", msgcomDataHeap);
        System_printf ("Free Data Buffer Packets              :%d\n", endHeapStats.numFreeDataPackets);
        System_printf ("Data Buffer Threshold Status          :%d\n", endHeapStats.dataBufferThresholdStatus);
        System_printf ("Data Buffer Starvation Counter        :%d\n", endHeapStats.dataBufferStarvationCounter);
        System_printf ("Zero Data Buffer Packets              :%d\n", endHeapStats.numZeroBufferPackets);
        System_printf ("Zero Data Buffer Threshold Status     :%d\n", endHeapStats.zeroDataBufferThresholdStatus);
        System_printf ("Zero Data Buffer Starvation Counter   :%d\n", endHeapStats.zeroDataBufferStarvationCounter);
        System_printf ("Number of Packets in Garbage          :%d\n", endHeapStats.numPacketsinGarbage);
    }
    else
    {
        System_printf ("Debug: Test passed. No memory leaks detected\n");
    }

    /* Display the data buffer threshold status at the end of the test */
    System_printf ("Debug: Data Buffer Starvation Counter: %d\n", endHeapStats.dataBufferStarvationCounter);
    return;
}

