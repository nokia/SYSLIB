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
 *
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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* PDK & CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 *************************** Local Definitions ************************
 **********************************************************************/

/* MAXIMUM number of receive buffers which are used to store data on the
 * Virtual Channels. */
#define     MAX_RECV_BUFFER             11

/**********************************************************************
 *************************** Local Declarations ***********************
 **********************************************************************/

/* Global Counter to keep track of the Virtual channel callback notifications. */
uint32_t            gVirtualChannelCallBackCounter = 0;

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
extern Pktlib_HeapHandle        myPktLibHeapHandle;
extern Pktlib_HeapHandle        mySharedHeapHandle;
extern Name_DBHandle            globalNameDatabaseHandle;
extern Name_ClientHandle        nameClientHandle;
extern Pktlib_InstHandle        appPktlibInstanceHandle;
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Number of test iterations: */
extern uint32_t             numTestIterations;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg      appResourceConfig;

/**********************************************************************
 *************************** Reader Functions *************************
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
    System_printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
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
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 160)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 160)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < 160; index++)
        {
            if (*(ptrDataBuffer + index) != 0xAA)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

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
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 90)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the accumulator queue reader test
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
    chConfig.mode                                               = mode;
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

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        /* Use the correct channel name. */
        channelName = "Accumulated-Queue-NonBlocking";
    }
    else
    {
        /* Use the correct channel name. */
        channelName = "Accumulated-Queue-Blocking";
    }

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 128)
        {
            System_printf ("Error: Invalid packet length detected Expected 128 Got %d\n",
                            Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xDD)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

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
 *  @param[in]  nameClientHandle
 *      Agent client handle which indicates if the reader channel has
 *      to be pushed from the DSP realm to the ARM realm
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_basic_reader(Msgcom_ChannelMode mode, Name_ClientHandle nameClientHandle)
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
    chConfig.u.queueDMACfg.rxFreeQueueNum= (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Do we need to push the channel from the DSP to the ARM realm? */
    if (nameClientHandle != NULL)
    {
        if (Name_push (nameClientHandle, channelName, Name_ResourceBucket_INTERNAL_SYSLIB,
                       Name_ResourceOperationType_CREATE, &errorCode) < 0)
        {
            System_printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", channelName, errorCode);
            return -1;
        }
        System_printf ("Debug: Channel Name '%s' pushed successfully to the ARM realm\n", channelName);
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the data buffer */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 10)
        {
            System_printf ("Error: Invalid packet length detected in the received packet [Received %d bytes]\n", Pktlib_getPacketLen(ptrMessage));
            return -1;
        }
        if (dataLen != 10)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0xAA)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
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
    chConfig.mode                                                         = mode;
    chConfig.appCallBack                                                  = NULL;
    chConfig.msgcomInstHandle                                             = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                  = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                 = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Impl-QDMA-NonBlock";
    else
        channelName = "Impl-QDMA-Block";

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the data buffer */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 20)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 20)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x33)
            {
                System_printf ("Error: Invalid packet data payload (0x%p) detected (Got 0x%x)\n",
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

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
    chConfig.u.queueDMACfg.rxFreeQueueNum                             = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries    = 5;
	chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pacingTimerCount  = 0;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        /* Use the correct channel name. */
        channelName = "Acc-Queue-DMA-NonBlocking";
    }
    else
    {
        /* Use the correct channel name. */
        channelName = "Acc-Queue-DMA-Blocking";
    }

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the data buffer */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 15)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 15)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x44)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
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
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_basic_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Channel Name for Physical and Virtual Channels are created here. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        channelName         = "Queue-NonBlocking";
        virtualChannelName  = "VirtualQueue-NonBlock";
    }
    else
    {
        channelName         = "Queue-Blocking";
        virtualChannelName  = "VirtualQueue-Block";
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the "physical" channel configuration. */
    chConfig.mode                     = mode;
    chConfig.appCallBack              = NULL;
    chConfig.msgcomInstHandle         = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the PHYSICAL MSGCOM channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Reset the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                              = mode;
    chConfig.appCallBack                       = NULL;
    chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
    chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 5)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 5)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x11)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    errorCode = Msgcom_delete (chHandle, myFreeMsgBuffer);
    if (errorCode < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed [Error code %d]\n", virtualChannelName, errorCode);
        return -1;
    }

    /* Delete the physical channel now. */
    errorCode = Msgcom_delete (phyChHandle, myFreeMsgBuffer);
    if (errorCode < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed [Error code %d]\n", channelName, errorCode);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered callback function for the
 *      Implicit Notify Blocking Reader Channel
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *  @param[in]  arg
 *      Optional application registered argument.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static void myVirtualChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    if (arg != 0xdeaddead)
        System_printf ("Error: Optional argument is invalid [%x]\n", arg);

    /* Increment the Virtual channel callback counter. */
    gVirtualChannelCallBackCounter++;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function tests the interrupt virtual queue reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Reset the global counter */
    gVirtualChannelCallBackCounter = 0;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        channelName        = "Implicit-Queue-NonBlocking";
        virtualChannelName = "Virtual-Implicit-Queue-NonBlock";
    }
    else
    {
        channelName        = "Implicit-Queue-Blocking";
        virtualChannelName = "Virtual-Implicit-Queue-Blocking";
    }

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

    /* Create the PHYSICAL MSGCOM channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Reset the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                              = mode;
    chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
    chConfig.arg                               = 0xdeaddead;
    chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

    /* Register the callback only in the case of non-blocking mode. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        chConfig.appCallBack = myVirtualChannelCallback;
    else
        chConfig.appCallBack = NULL;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: Virtual Channel 0x%p has been created\n", chHandle);

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;

            /* We have gotten a message on the virtual channel. But make sure
             * that the virtual channel callback was also activated */
            if (gVirtualChannelCallBackCounter == 0)
            {
                System_printf ("Error: Virtual Channel callback NOT activated\n");
                return -1;
            }
            System_printf ("Debug: Implicit Virtual Channel callback counter is %d\n",
                           gVirtualChannelCallBackCounter);
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 60)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 60)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < 60; index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed\n", virtualChannelName);
        return -1;
    }

    /* Delete the physical channel now. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the accumulator virtual queue reader test
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_accumulated_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Reset the global counter */
    gVirtualChannelCallBackCounter = 0;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        /* Use the correct channel name. */
        channelName         = "Acc-Queue-NonBlocking";
        virtualChannelName  = "Virtual-Acc-Queue-NonBlocking";
    }
    else
    {
        /* Use the correct channel name. */
        channelName         = "Acc-Queue-Blocking";
        virtualChannelName  = "Virtual-Acc-Queue-Blocking";
    }

    /* Populate the channel configuration. */
    chConfig.mode                                               = mode;
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

    /* Create the Message communicator channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open channel Error : %d\n", errorCode);
        return -1;
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                              = mode;
    chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
    chConfig.arg                               = 0xdeaddead;
    chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

    /* Register the callback only in the case of non-blocking mode. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        chConfig.appCallBack = myVirtualChannelCallback;
    else
        chConfig.appCallBack = NULL;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;

            /* We have gotten a message on the virtual channel. But make sure
             * that the virtual channel callback was also activated */
            if (gVirtualChannelCallBackCounter == 0)
            {
                System_printf ("Error: Virtual Channel callback NOT activated\n");
                return -1;
            }
            System_printf ("Debug: Accumulated Virtual Channel callback counter is %d\n",
                           gVirtualChannelCallBackCounter);
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 128)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 128)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < 128; index++)
        {
            if (*(ptrDataBuffer + index) != 0xDD)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed\n", virtualChannelName);
        return -1;
    }

    /* Delete the physical channel now. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue DMA reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_basic_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        channelName         = "Queue-DMA-NonBlocking";
        virtualChannelName  = "VIRQueue-DMA-NonBlock";
    }
    else
    {
        channelName         = "Queue-DMA-Blocking";
        virtualChannelName  = "VIRQueue-DMA-Blocking";
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = mode;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum= (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));

    /* Create the PHYSICAL MSGCOM channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Reset the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                              = mode;
    chConfig.appCallBack                       = NULL;
    chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
    chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the data buffer */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 10)
        {
            System_printf ("Error: Invalid packet length (%d) detected in the received packet (%d)\n",
                            Pktlib_getPacketLen(ptrMessage), messageCounter);
            return -1;
        }
        if (dataLen != 10)
        {
            System_printf ("Error: Invalid data length (%d) detected in the received packet.\n", dataLen);
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0xAA)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed\n", virtualChannelName);
        return -1;
    }

    /* Delete the physical channel now. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue DMA reader channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_implicitNotify_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    MsgCom_ChHandle        phyChHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        channelName         = "Impl-QDMA-NonBlock";
        virtualChannelName  = "VirtImpl-QDMA-NonBlock";
    }
    else
    {
        channelName         = "Impl-QDMA-Block";
        virtualChannelName  = "VirtImpl-QDMA-Block";
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                           = mode;
    chConfig.appCallBack                                                    = NULL;
    chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the PHYSICAL MSGCOM channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Reset the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                           = mode;
    chConfig.appCallBack                    = NULL;
    chConfig.msgcomInstHandle               = appMsgcomInstanceHandle;
    chConfig.u.virtualChannelCfg.phyChannel = phyChHandle;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
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
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 20)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x33)
            {
                System_printf ("Error: Invalid packet data payload (0x%p) detected (Got 0x%x)\n",
                               ptrDataBuffer, *(ptrDataBuffer + index));
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed\n", virtualChannelName);
        return -1;
    }

    /* Delete the physical channel now. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue DMA reader channel with
 *      accumulated interrupt support.
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_accumulated_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char*                  virtualChannelName;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    MsgCom_ChHandle        phyChHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                messageCounter = 0;
    int32_t                errorCode;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
    {
        /* Use the correct channel name. */
        channelName         = "Acc-Queue-DMA-NonBlocking";
        virtualChannelName  = "Virt-Acc-Queue-DMA-NonBlock";
    }
    else
    {
        /* Use the correct channel name. */
        channelName         = "Acc-Queue-DMA-Blocking";
        virtualChannelName  = "Virt-Acc-Queue-DMA-Blocking";
    }

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                     = mode;
    chConfig.appCallBack                                              = NULL;
    chConfig.msgcomInstHandle                                         = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                              = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                             = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries    = 5;
	chConfig.u.queueDMACfg.queueDMAIntrUnion.accCfg.pacingTimerCount  = 0;

    /* Create the PHYSICAL MSGCOM channel. */
    phyChHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errorCode);
    if (phyChHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errorCode);
        return -1;
    }

    /* Reset the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                              = mode;
    chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
    chConfig.appCallBack                       = NULL;
    chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

    /* Create a virtual channel. */
    chHandle = Msgcom_create(virtualChannelName, Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
        return -1;
    }

    /* Wait for the messages to be received. */
    while (messageCounter < numTestIterations)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
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
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the data buffer */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 15)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 15)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != 0x44)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Increment the message counter. */
        messageCounter++;
    }

    /* TEST: Physical Channel cannot be deleted before the Virtual channels ae deleted
     * Here we try and delete the PHYSCIAL channel but this should fail with an error code. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) != MSGCOM_CHANNEL_IN_USE)
    {
        System_printf ("Error: MSGCOM PHYSICAL Channel %s was successfully deleted while virtual channel was enabled.\n", channelName);
        return -1;
    }

    /* Delete the virtual channel before deleting the Physical channel. */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Virtual Channel %s deletion failed\n", virtualChannelName);
        return -1;
    }

    /* Delete the physical channel now. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Physical Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue DMA reader channel with
 *      accumulated interrupt support.
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_multiple_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    char                   virtualChannelName[MSG_COM_MAX_CHANNEL_LEN];
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        phyChHandle;
    MsgCom_ChHandle        chHandle[5];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                chIndex;
    int32_t                errorCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName        = "MyPhy-NonBlock-Channel";
    else
        channelName        = "MyPhy-Block-Channel";

    /* Populate the channel configuration. */
    chConfig.mode                                                   = mode;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create a PHYSICAL Channel. */
    phyChHandle = Msgcom_create(channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errorCode);
    if (phyChHandle == NULL)
    {
        System_printf ("Error: Unable to find the PHYSICAL channel %s Error : %d\n", channelName, errorCode);
        return -1;
    }

    /* We will now create 3 Virtual Channels over this PHYSICAL Channel */
    for (index = 0; index < 3; index++)
    {
        /* Create a unique channel name. */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
            sprintf (virtualChannelName, "MyVirt-NonBlock-Channel-%d", index);
        else
            sprintf (virtualChannelName, "MyVirt-Block-Channel--%d", index);

        /* Create the 'virtual' channel configuration. */
        chConfig.mode                              = mode;
        chConfig.appCallBack                       = NULL;
        chConfig.msgcomInstHandle                  = appMsgcomInstanceHandle;
        chConfig.u.virtualChannelCfg.phyChannel    = phyChHandle;

        /* Create a virtual channel. */
        chHandle[index] = Msgcom_create(&virtualChannelName[0], Msgcom_ChannelType_VIRTUAL, &chConfig, &errorCode);
        if (chHandle[index] == NULL)
        {
            System_printf ("Error: Unable to create virtual channel Error : %d\n", errorCode);
            return -1;
        }
    }

    /* All the Virtual channels are created. We will now wait for the data
     * to come on Channel 2; then Channel 1 and then Channel 0. Each data will be validated */
    chIndex = 2;

    /* Loop around till the test is complete or if an error was detected. */
    while (1)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle[chIndex], (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }
            if (ptrMessage == NULL)
                continue;
        }
        else
        {
            /* No. Get the message if there is none available this will be blocked. */
            if (Msgcom_getMessage (chHandle[chIndex], (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }

            /* Control comes here implies that a message was available. */
        }

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 60)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != 60)
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < dataLen; index++)
        {
            if (*(ptrDataBuffer + index) != chIndex)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Are we done with testing all the channels. */
        if (chIndex == 0)
            break;

        /* Go to the next channel. */
        chIndex = chIndex - 1;
    }

    /* Delete all the virtual channels. */
    for (chIndex = 0; chIndex < 3; chIndex++)
    {
        /* Delete the virtual channel. */
        if (Msgcom_delete (chHandle[chIndex], myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
            return -1;
        }
    }

    /* Delete the physical channel. */
    if (Msgcom_delete (phyChHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: MSGCOM Channel %s deletion failed\n", channelName);
        return -1;
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests multiple queue reader channels for direct &
 *      accumulated interrupts. The idea behind the test is to ensure that all
 *      the supported Direct Interrupt & Accumulated channels are verified for
 *      basic functionality.
 *
 *  @param[in]  mode
 *      Interrupt mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_multiple_queue_reader_channel (Msgcom_QueueInterruptMode mode)
{
    char                   channelName[MSG_COM_MAX_CHANNEL_LEN];
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle[64];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                errCode;
    uint32_t               maxChannels;
    Name_ResourceCfg       namedResource;
    uint8_t                messageTracker[64];

    /* Reset the message tracker. */
    memset ((void *)&messageTracker, 0, sizeof(messageTracker));

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Create the channel configuration using the interrupt mode. */
    if (mode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Get the maximum number of channels which are being tested here. */
        maxChannels = appResourceConfig.numQpendQueues;

        /* Initialize the named resource configuration. */
        memset ((void*)&namedResource, 0, sizeof(Name_ResourceCfg));

        /* Populate the named resource configuration. */
        namedResource.handle1  = maxChannels;
        strncpy(namedResource.name, "MAX_QPEND_CHANNELS", NAME_MAX_CHAR);

        /* Share the number of accumulator channels which are being tested. */
        if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                &namedResource, &errCode) < 0)
            return -1;

        /* Debug Message: */
        System_printf ("Debug: Multiple Reader Channel [Direct Interrupt Queues] Max Channels %d\n", maxChannels);
    }
    else
    {
        /* Get the maximum number of channels which are being tested here. */
        maxChannels = appResourceConfig.numAccumalatorChannels;

        /* Initialize the named resource configuration. */
        memset ((void*)&namedResource, 0, sizeof(Name_ResourceCfg));

        /* Populate the named resource configuration. */
        namedResource.handle1  = maxChannels;
        strncpy(namedResource.name, "MAX_ACCUMULATOR_CHANNELS", NAME_MAX_CHAR);

        /* Share the number of accumulator channels which are being tested. */
        if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                &namedResource, &errCode) < 0)
            return -1;

        /* Debug Message: */
        System_printf ("Debug: Multiple Reader Channel [Accumulated Queues] Max Channels %d\n", maxChannels);
    }

    /* We are supporting only a MAX of 64 channels in the test */
    if (maxChannels > 64)
    {
        System_printf ("Error: Test Multiple Reader supports only 64 channels [Requested for %d channels]\n", maxChannels);
        return -1;
    }

    /* Create all the MSGCOM Channels. */
    for (index = 0; index < maxChannels; index++)
    {
        /* Initialize the channel configuration. */
        memset ((void*)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Create the channel name. */
        if (mode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
        {
            sprintf (channelName, "TestReaderQueuePend-%d", index);

            /* Populate the channel configuration. */
            chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
            chConfig.appCallBack                                            = NULL;
            chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[index].queue;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[index].cpIntcId;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[index].systemInterrupt;
            chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[index].hostInterrupt;
        }
        else
        {
            sprintf (channelName, "TestReaderAccumulated-%d", index);

            /* Populate the channel configuration. */
            chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
            chConfig.appCallBack                                        = NULL;
            chConfig.msgcomInstHandle                                   = appMsgcomInstanceHandle;
            chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[index].accChannel;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[index].queue;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[index].pdspId;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[index].eventId;
            chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 5;
	        chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 0;
        }

        /* Create the Message communicator channel. */
        chHandle[index] = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
        if (chHandle[index] == 0)
        {
            System_printf ("Error: Unable to open the channel %d Error : %d\n", index, errCode);
            return -1;
        }
    }

    /* Loop around forever until all the messages have been received. */
    while (1)
    {
        /* Cycle through all the channels and check for a message. */
        for (index = 0; index < maxChannels; index++)
        {
            /* Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle[index], (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }
            /* Did we get a message? If not cycle to the next channel. */
            if (ptrMessage != NULL)
                break;
        }

        /* Did we get a message? If not cycle through and check all the channels again. */
        if (ptrMessage == NULL)
            continue;

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Set the message tracker to indicate that the message was received on the channel. */
        messageTracker[index] = 1;

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 90)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Cycle through all the channels and check for if messages have been received on all the channels or not? */
        for (index = 0; index < maxChannels; index++)
        {
            /* Has a message been received on the channel? */
            if (messageTracker[index] == 0)
                break;
        }

        /* Are we done with the test? */
        if (index == maxChannels)
            break;
    }

    /* At the end of the test cleanup all the MSGCOM channels which had been opened */
    for (index = 0; index < maxChannels; index++)
    {
        /* Delete the MSGCOM Channel. */
        if (Msgcom_delete (chHandle[index], myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: MSGCOM Channel %d deletion failed\n", index);
            return -1;
        }
    }

    /* All the messages were received and validated. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests multiple queue DMA reader channels to ensure that
 *      the it is possible to create and use all the MAX Queue DMA channels.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_multiple_queueDMA_reader_channel (void)
{
    char                   channelName[MSG_COM_MAX_CHANNEL_LEN];
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle[128];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                index;
    int32_t                errCode;
    uint32_t               maxChannels;
    Name_ResourceCfg       namedResource;
    uint8_t                messageTracker[128];

    /* Reset the message tracker. */
    memset ((void *)&messageTracker, 0, sizeof(messageTracker));

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

#if defined(DEVICE_K2H) || defined(DEVICE_K2K)
    /* Get the maximum number of channels which are being tested here: On Hawking there are 128
     * receive flows and so we should be able to create those many channels. We remove 8 because
     * these are hardcoded for use by the NAME Proxy. 2 are used by the NETFP Master [Internal NETFP Server
     * and Client]. This definition is device specific */
    maxChannels = 118;
#elif defined (DEVICE_K2L)
    /* Get the maximum number of channels which are being tested here: On Lamarr there are 64
     * receive flows and so we should be able to create those many channels. We remove 8 because
     * these are hardcoded for use by the NAME Proxy. 2 are used by the NETFP Master [Internal NETFP Server
     * and Client]. This definition is device specific */
    maxChannels = 54;
#else
    System_printf ("Error: Multiple Queue DMA Channel limits are not not configured for the device\n");
    return -1;
#endif

    /* Initialize the named resource configuration. */
    memset ((void*)&namedResource, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    namedResource.handle1  = maxChannels;
    strncpy(namedResource.name, "MAX_QUEUE_DMA_CHANNELS", NAME_MAX_CHAR);

    /* Share the number of accumulator channels which are being tested. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                            &namedResource, &errCode) < 0)
        return -1;

    /* Debug Message: */
    System_printf ("Debug: Multiple Queue DMA Reader Channel Max Channels %d\n", maxChannels);

    /* We are supporting only a MAX of 128 channels in the test */
    if (maxChannels > 128)
    {
        System_printf ("Error: Test Multiple Reader supports only 128 channels [Requested for %d channels]\n", maxChannels);
        return -1;
    }

    /* Create all the MSGCOM Channels. */
    for (index = 0; index < maxChannels; index++)
    {
        /* Initialize the channel configuration. */
        memset ((void*)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Create the channel name */
        sprintf (channelName, "TestReaderQueueDMAPend-%d", index);

        /* Populate the channel configuration. */
        chConfig.mode                           = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.msgcomInstHandle               = appMsgcomInstanceHandle;
        chConfig.appCallBack                    = NULL;
        chConfig.u.queueDMACfg.interruptMode    = Msgcom_QueueInterruptMode_NO_INTERRUPT;
        chConfig.u.queueDMACfg.rxFreeQueueNum   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));

        /* Create the MSGCOM channel. */
        chHandle[index] = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errCode);
        if (chHandle[index] == 0)
        {
            System_printf ("Error: Unable to open the channel %d [Error code %d]\n", index, errCode);
            return -1;
        }
    }

    /* Loop around forever until all the messages have been received. */
    while (1)
    {
        /* Cycle through all the channels and check for a message. */
        for (index = 0; index < maxChannels; index++)
        {
            /* Wait for the message to be received. */
            if (Msgcom_getMessage (chHandle[index], (MsgCom_Buffer**)&ptrMessage) < 0)
            {
                System_printf ("Error: Unable to get a message\n");
                return -1;
            }
            /* Did we get a message? If not cycle to the next channel. */
            if (ptrMessage != NULL)
                break;
        }

        /* Did we get a message? If not cycle through and check all the channels again. */
        if (ptrMessage == NULL)
            continue;

        /* Message Received: Get the data buffer from the received message. */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataLen);

        /* Set the message tracker to indicate that the message was received on the channel. */
        messageTracker[index] = 1;

        /* Validations:
         *  - Ensure the packet length matches what we sent.
         *  - Ensure the data length matches what we sent.
         *  - Ensure that the data pattern matches what was transmitted. */
        if (Pktlib_getPacketLen(ptrMessage) != 90)
        {
            System_printf ("Error: Invalid packet length detected in the received packet.\n");
            return -1;
        }
        if (dataLen != Pktlib_getPacketLen(ptrMessage))
        {
            System_printf ("Error: Invalid data length detected in the received packet.\n");
            return -1;
        }
        for (index = 0; index < Pktlib_getPacketLen(ptrMessage); index++)
        {
            if (*(ptrDataBuffer + index) != 0xCC)
            {
                System_printf ("Error: Invalid packet data payload detected\n");
                return -1;
            }
        }

        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);

        /* Cycle through all the channels and check for if messages have been received on all the channels or not? */
        for (index = 0; index < maxChannels; index++)
        {
            /* Has a message been received on the channel? */
            if (messageTracker[index] == 0)
                break;
        }

        /* Are we done with the test? */
        if (index == maxChannels)
            break;
    }

    /* At the end of the test cleanup all the MSGCOM channels which had been opened */
    for (index = 0; index < maxChannels; index++)
    {
        /* Delete the MSGCOM Channel. */
        if (Msgcom_delete (chHandle[index], myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: MSGCOM Channel %d deletion failed\n", index);
            return -1;
        }
    }

    /* All the messages were received and validated. */
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the reader task for DSP core to core tests.
 *
 *  @retval
 *      Not Applicable.
 */
Void readerControlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Debug Message: The Reader tests are about to start. */
    System_printf ("Debug: Starting the CORE to CORE reader tests.\n");

    /*************************************************************************
     *************************** QUEUE Reader Test ***************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue Reader Test */
    if (test_queue_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Basic Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue Reader Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Reader Test */
    if (test_queue_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue Reader Non-Blocking Test Passed\n");
    if (test_queue_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue Reader Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Reader Test */
    if (test_queue_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue Reader Non-Blocking Test Passed\n");
    if (test_queue_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue Reader Blocking Test Passed\n");

    /*****************************************************************************
     *********************** Multiple Queue Channel Tests ************************
     *****************************************************************************/

    /* Test: Multiple Queue Accumulator channel */
    if (test_multiple_queue_reader_channel(Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT) < 0)
    {
        System_printf ("Error: Multiple Reader Channel Accumulator Interrupt Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Reader Channel Accumulator Interrupt Test Passed\n");

    /* Test: Multiple Queue Interrupt channel */
    if (test_multiple_queue_reader_channel(Msgcom_QueueInterruptMode_DIRECT_INTERRUPT) < 0)
    {
        System_printf ("Error: Multiple Reader Channel Direct Interrupt Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Reader Channel Direct Interrupt Test Passed\n");

    /*************************************************************************
     ************************* QUEUE-DMA Reader Test *************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Reader Test */
    if (test_queue_dma_basic_reader(Msgcom_ChannelMode_NON_BLOCKING, NULL) < 0)
    {
        System_printf ("Error: Basic Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue DMA Reader Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Reader Test */
    if (test_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue DMA Reader Non-Blocking Test Passed\n");
    if (test_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue DMA Reader Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Reader Test */
    if (test_queue_dma_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue DMA Reader Non-Blocking Test Passed\n");
    if (test_queue_dma_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue DMA Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue DMA Reader Blocking Test Passed\n");

    /*****************************************************************************
     ********************* Multiple Queue DMA Channel Tests **********************
     *****************************************************************************/
    if (test_multiple_queueDMA_reader_channel() < 0)
    {
        System_printf ("Error: Multiple Queue DMA Reader Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Queue DMA Reader Test Passed\n");

    /*****************************************************************************
     *********************** Virtual Queue Channel Reader Test *******************
     *****************************************************************************/

    /* Test 1: Execute the VIRTUAL BASIC Queue Reader Test */
    if (test_virtual_queue_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Basic Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Basic Queue Reader Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Reader Test */
    if (test_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue Reader Non-Blocking Test Passed\n");
    if (test_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue Reader Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Reader Test */
    if (test_virtual_queue_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue Reader Non-Blocking Test Passed\n");
    if (test_virtual_queue_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue Reader Blocking Test Passed\n");

    /*****************************************************************************
     ********************** Virtual Queue DMA Channel Reader Test ****************
     *****************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Reader Test */
    if (test_virtual_queue_dma_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Basic Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Basic Queue DMA Reader Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Reader Test */
    if (test_virtual_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue DMA Reader Non-Blocking Test Passed\n");

    if (test_virtual_queue_dma_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue DMA Reader Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Reader Test */
    if (test_virtual_queue_dma_accumulated_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue DMA Reader Non-Blocking Test Passed\n");
    if (test_virtual_queue_dma_accumulated_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue DMA Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue DMA Reader Blocking Test Passed\n");

    /*****************************************************************************
     ******************** Multiple Virtual Queue Channel Reader Test *************
     *****************************************************************************/

    /* Test 1: Execute Multiple Virtual Channels Implicit Notify Queue Reader Test */
    if (test_multiple_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Multiple Virtual Implicit Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Virtual Implicit Reader Non-Blocking Test Passed\n");

    /* Test 2: Execute Multiple Virtual Channels Implicit Notify Queue Reader Test */
    if (test_multiple_virtual_queue_implicitNotify_reader(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Multiple Virtual Implicit Queue Reader Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Virtual Implicit Queue Reader Blocking Test Passed\n");

    System_printf ("------------------------------\n");
    System_printf ("Debug: All Reader Tests Passed\n");
    System_printf ("------------------------------\n");

    System_printf ("Debug: Detected %d entries in the named resource database\n",
                    Name_dumpDatabase(globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &errCode));
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the reader task for the ARM (Writer) sending messages to
 *      the DSP (Reader)
 *
 *  @retval
 *      Not Applicable.
 */
Void test_ARMWriterDSPReader(UArg arg0, UArg arg1)
{
    /* Debug Message: The Reader tests are about to start. */
    System_printf ("Debug: Starting the ARM Writer to the DSP Reader tests.\n");

    /*************************************************************************
     ************************* QUEUE-DMA Reader Test *************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Reader Test: Since these tests are running
     * between the ARM and DSP realms we need to push the channel name using the created
     * agent client handle. */
    if (test_queue_dma_basic_reader(Msgcom_ChannelMode_NON_BLOCKING, nameClientHandle) < 0)
    {
        System_printf ("Error: Basic Queue DMA Reader Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue DMA Reader Non-Blocking Test Passed\n");
    return;
}

