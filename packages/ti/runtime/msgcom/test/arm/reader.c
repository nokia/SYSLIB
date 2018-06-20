/**
 *   @file  reader.c
 *
 *   @brief
 *      Reader Unit Test Code.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* This is the MAXIMUM number of test messages which are exchanged between
 * the reader and writer. */
extern uint32_t  MAX_TEST_MESSAGES;

/* Global PKTLIB Heap Handle: */
extern Pktlib_HeapHandle       testHeapHandle;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global application resource configuration */
Resmgr_ResourceCfg             appResourceConfig;

/**********************************************************************
 ************************* Unit Test Functions ************************
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
    Pktlib_freePacket(appPktlibInstanceHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Debug utility function which displays the statistics of the heap
 *
 *  @param[in]  testHeapHandle
 *      PKTLIB heap handle
 *
 *  @retval
 *      Not Applicable.
 */
static void displayHeapStats(Pktlib_HeapHandle testHeapHandle)
{
    Pktlib_HeapStats    heapStats;

    /* Get the heap statistics */
    Pktlib_getHeapStats (testHeapHandle, &heapStats);

    /* Display the heap stats */
    printf ("Debug: Free Data Packet:%d Zero Buffer Packets:%d Garbage Packets:%d\n",
             heapStats.numFreeDataPackets, heapStats.numZeroBufferPackets, heapStats.numPacketsinGarbage);
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_basic_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                     = mode;
    chConfig.appCallBack              = NULL;
    chConfig.msgcomInstHandle         = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Queue-NonBlocking";
    else
        channelName = "Queue-Blocking";

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 160)
        {
            printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 160)
        {
            printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < 160; index++)
        {
            if (*(ptrDataBuffer + index) != 0xAA)
            {
                printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* Get the heap statistics and display */
    displayHeapStats(testHeapHandle);

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the interrupt queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_implicitNotify_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = mode;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Implicit-Queue-NonBlocking";
    else
        channelName = "Implicit-Queue-Blocking";

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 90)
        {
            printf ("Error: Invalid packet length detected in the received packet [Got %d].\n", Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            printf ("Error: Invalid data length detected in the received packet [Got %d Expected %d].\n",
                     dataLen, Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* Get the heap statistics and display */
    displayHeapStats(testHeapHandle);

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the accumulated queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_accumulated_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = mode;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type                  = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel            = appResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue              = appResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId                = appResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId           = appResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries        = 5;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount      = 0;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Accumulated-Queue-NonBlocking";
    else
        channelName = "Accumulated-Queue-Blocking";

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 100)
        {
            printf ("Error: Invalid packet length detected in the received packet [Got %d].\n", Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            printf ("Error: Invalid data length detected in the received packet [Got %d Expected %d].\n",
                     dataLen, Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* Get the heap statistics and display */
    displayHeapStats(testHeapHandle);

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_basic_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Queue-DMA-NonBlocking";
    else
        channelName = "Queue-DMA-Blocking";

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = mode;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum= (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(testHeapHandle)));

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 10)
        {
            printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 10)
        {
            printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0xAA)
            {
                printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* Get the heap statistics and display */
    displayHeapStats(testHeapHandle);

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue DMA reader channel with accumulated
 *      interrupt support.
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_accumulated_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                     = mode;
    chConfig.appCallBack                                              = NULL;
    chConfig.msgcomInstHandle                                         = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                              = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                             = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(testHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[3].accChannel;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[3].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[3].pdspId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[3].eventId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries    = 5;
	chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pacingTimerCount  = 0;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        /* Use the correct channel name. */
        channelName = "Accumulated-QDMA-NonBlock";
    }
    else
    {
        /* Use the correct channel name. */
        channelName = "Accumulated-QDMA-Block";
    }

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 40)
        {
            printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 40)
        {
            printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x33)
            {
                printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue DMA reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_implicitNotify_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                           = mode;
    chConfig.appCallBack                                                    = NULL;
    chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(testHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Impl-QDMA-NonBlock";
    else
        channelName = "Impl-QDMA-Block";

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 20)
        {
            printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 20)
        {
            printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x33)
            {
                printf ("Error: Invalid packet data payload (0x%p) detected (Got 0x%x)\n",
                               ptrDataBuffer, *(ptrDataBuffer + index));
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* Delete the MSGCOM Channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* Get the heap statistics and display */
    displayHeapStats(testHeapHandle);

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Reader thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread
 *
 *  @retval
 *      Always NULL
 */
void* reader(void* arg)
{
    printf ("Debug: Reader Test Starting.\n");

    /*************************************************************************
     ************************* QUEUE-DMA Reader Test *************************
     *************************************************************************/
    if (test_queue_dma_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue DMA Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue DMA Reader Non-Blocking Test Passed\n");

    if (test_queue_dma_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue DMA Reader Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue DMA Reader Blocking Test Passed\n");

    if (test_queue_dma_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue DMA Reader Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue DMA Reader Blocking Test Passed\n");

    if (test_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue DMA Reader Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue DMA Reader Blocking Test Passed\n");

    if (test_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue DMA Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue DMA Reader Non-Blocking Test Passed\n");

    /*************************************************************************
     *************************** QUEUE Reader Test ***************************
     *************************************************************************/
    if (test_queue_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue Reader Non-Blocking Test Passed\n");

    if (test_queue_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue Reader Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue Reader Blocking Test Passed\n");

    if (test_queue_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue Reader Non-Blocking Test Passed\n");

    if (test_queue_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue Reader Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue Reader Blocking Test Passed\n");

    if (test_queue_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue Reader Non-Blocking Test Passed\n");

    /* Control comes here implies that all the reader tests passed */
    printf ("*******************************************************************\n");
    printf ("Debug: MSGCOM Reader Unit Test all passed\n");
    printf ("*******************************************************************\n");
    return NULL;
}

