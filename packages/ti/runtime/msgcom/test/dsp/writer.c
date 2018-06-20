/**
 *   @file  writer.c
 *
 *   @brief
 *      Writer Unit Test Code.
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

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
extern Pktlib_HeapHandle        myPktLibHeapHandle;
extern Pktlib_HeapHandle        mySharedHeapHandle;
extern Name_DBHandle            globalNameDatabaseHandle;
extern Pktlib_InstHandle        appPktlibInstanceHandle;
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Number of test iterations: */
extern uint32_t            numTestIterations;

/* Application Cache Management API */
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg      appResourceConfig;

/**********************************************************************
 *************************** Writer Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
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
 *      The function tests the basic queue writer channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_basic_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Queue-NonBlocking";
    else
        channelName = "Queue-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 160);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 160; index++)
            *(ptrDataBuffer + index) = 0xAA;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 160);

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, 160);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message over the polling queue channel\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue implicit notification writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_implicitNotify_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Implicit-Queue-NonBlocking";
    else
        channelName = "Implicit-Queue-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 90);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 90; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 90);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 90);
        Pktlib_setDataBufferLen(ptrMessage, 90);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the accumulated queue writer channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_accumulated_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Accumulated-Queue-NonBlocking";
    else
        channelName = "Accumulated-Queue-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 128);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 128; index++)
            *(ptrDataBuffer + index) = 0xDD;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 128);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 128);
        Pktlib_setDataBufferLen(ptrMessage, 128);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message over the polling queue channel\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the polling queue writer channel
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_basic_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Queue-DMA-NonBlocking";
    else
        channelName = "Queue-DMA-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Local core specific Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 16);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 10;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0xAA;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully transmitted. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue DAM implicit notification writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_implicitNotify_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Impl-QDMA-NonBlock";
    else
        channelName = "Impl-QDMA-Block";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 20;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0x33;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length. */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue DMA accumulated writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_queue_dma_accumulated_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Acc-Queue-DMA-NonBlocking";
    else
        channelName = "Acc-Queue-DMA-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 15;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0x44;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length. */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the basic virtual queue writer channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_basic_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "VirtualQueue-NonBlock";
    else
        channelName = "VirtualQueue-Block";

    /* Wait for the message communicator 'virtual' channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 5);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 5; index++)
            *(ptrDataBuffer + index) = 0x11;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 5);

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, 5);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message over the polling queue channel\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue implicit notification writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Virtual-Implicit-Queue-NonBlock";
    else
        channelName = "Virtual-Implicit-Queue-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 60; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 60);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 60);
        Pktlib_setDataBufferLen(ptrMessage, 60);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the accumulated virtual queue writer channel
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_accumulated_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Create a unique channel name. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Virtual-Acc-Queue-NonBlocking";
    else
        channelName = "Virtual-Acc-Queue-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 128);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 128; index++)
            *(ptrDataBuffer + index) = 0xDD;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 128);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 128);
        Pktlib_setDataBufferLen(ptrMessage, 128);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message over the polling queue channel\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the polling queue writer channel
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_basic_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "VIRQueue-DMA-NonBlock";
    else
        channelName = "VIRQueue-DMA-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Local core specific Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 16);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 10;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0xAA;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully transmitted. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the queue DAM implicit notification writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_implicitNotify_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "VirtImpl-QDMA-NonBlock";
    else
        channelName = "VirtImpl-QDMA-Block";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 20;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0x33;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length. */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the virtual queue DMA accumulated writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_virtual_queue_dma_accumulated_writer(Msgcom_ChannelMode mode)
{
    char*                  channelName;
    MsgCom_ChHandle        chHandle;
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                messageCounter = 0;
    int32_t                index;
    int32_t                errCode;

    /* Get the channel name; which depends upon the mode being tested. */
    if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        channelName = "Virt-Acc-Queue-DMA-NonBlock";
    else
        channelName = "Virt-Acc-Queue-DMA-Blocking";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < numTestIterations)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 15;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0x44;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length. */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* Delete the msgcom channel */
    if (Msgcom_delete (chHandle, myFreeMsgBuffer) < 0)
    {
        printf ("Error: Unable to delete the writer channel '%s'\n", channelName);
        return -1;
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the multiple virtual queue implicit notification writer
 *
 *  @param[in]  mode
 *      Blocking or Non-Blocking channel mode.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_multiple_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode mode)
{
    char                   virtualChannelName[MSG_COM_MAX_CHANNEL_LEN];
    MsgCom_ChHandle        chHandle[5];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                errCode;
    int32_t                index = 0;
    int32_t                chIndex = 0;

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Create the virtual channel name for which we are waiting. */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
            sprintf (virtualChannelName, "MyVirt-NonBlock-Channel-%d", index);
        else
            sprintf (virtualChannelName, "MyVirt-Block-Channel--%d", index);

        /* Check if the communicator channel has been created or not? */
        chHandle[index] = Msgcom_find(appMsgcomInstanceHandle, virtualChannelName, &errCode);
        if (chHandle[index] != NULL)
        {
            /* Have all the channels been created. */
            if (index == 2)
                break;

            /* Wait for the next channel to be created. */
            index = index + 1;
        }
        else
        {
            Task_sleep (2);
        }
    }

    /* Send out the messages from Channel 2 to Channel 1 to Channel 0 */
    chIndex = 2;

    /* Send out the messages to the reader. */
    while (1)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 60; index++)
            *(ptrDataBuffer + index) = chIndex;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 60);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 60);
        Pktlib_setDataBufferLen(ptrMessage, 60);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle[chIndex], (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Are we done with sending messages to all the channels? */
        if (chIndex == 0)
            break;

        /* Get the next channel number to where the message is to be sent. */
        chIndex = chIndex - 1;
    }

    /* Delete all the msgcom channels */
    for (chIndex = 0; chIndex < 3; chIndex++)
    {
        if (Msgcom_delete (chHandle[chIndex], myFreeMsgBuffer) < 0)
        {
            printf ("Error: Unable to delete the writer channel %p\n", chHandle[chIndex]);
            return -1;
        }
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests multiple queue writer channels for direct &
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
static int32_t test_multiple_queue_writer_channel (Msgcom_QueueInterruptMode mode)
{
    char                   channelName[MSG_COM_MAX_CHANNEL_LEN];
    MsgCom_ChHandle        chHandle[128];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                chIndex = 0;
    int32_t                index = 0;
    uint32_t               maxChannels;
    int32_t                errCode;
    Name_ResourceCfg       namedResource;

    /* Get the maximum number of channels which are being tested here. */
    if (mode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Sanity Check: Loop around till we find the MAX channels being tested */
        while (1)
        {
            if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                   "MAX_QPEND_CHANNELS", &namedResource, &errCode) == 0)
                break;
            Task_sleep(1);
        }
        maxChannels = namedResource.handle1;
    }
    else
    {
        /* Sanity Check: Loop around till we find the MAX channels being tested */
        while (1)
        {
            if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                                   "MAX_ACCUMULATOR_CHANNELS", &namedResource, &errCode) == 0)
                break;
            Task_sleep(1);
        }
        maxChannels = namedResource.handle1;
    }

    if (maxChannels > 128)
    {
        System_printf ("Error: Test Multiple writer channels supports a max of 64 channels only [Requested %d]\n", maxChannels);
        return -1;
    }

    /* Cycle through and find all the channels which have been created */
    for (index = 0; index < maxChannels; index++)
    {
        /* Create the channel name. */
        if (mode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
            sprintf (channelName, "TestReaderQueuePend-%d", index);
        else
            sprintf (channelName, "TestReaderAccumulated-%d", index);

        /* Loop around and find the channel name. */
        while (1)
        {
            chHandle[index] = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
            if (chHandle[index] != NULL)
                break;
            Task_sleep(1);
        }
    }

    /* Control comes here implies that all the channels have been found. Send out the
     * messages to the reader. */
    for (chIndex = 0; chIndex < maxChannels; chIndex++)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 90);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 90; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 90);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 90);
        Pktlib_setDataBufferLen(ptrMessage, 90);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle[chIndex], (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }
    }

    /* Delete all the msgcom channels */
    for (chIndex = 0; chIndex < maxChannels; chIndex++)
    {
        /* Delete the writer channel: */
        errCode = Msgcom_delete (chHandle[chIndex], myFreeMsgBuffer);
        if (errCode < 0)
        {
            System_printf ("Error: Unable to delete the writer channel '%s' [Error code %d]\n", channelName, errCode);
            return -1;
        }
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests multiple queue DMA writer channels
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t test_multiple_queueDMA_writer_channel (void)
{
    char                   channelName[MSG_COM_MAX_CHANNEL_LEN];
    MsgCom_ChHandle        chHandle[128];
    Ti_Pkt*                ptrMessage;
    uint8_t*               ptrDataBuffer;
    uint32_t               dataLen;
    int32_t                chIndex = 0;
    int32_t                index = 0;
    uint32_t               maxChannels;
    int32_t                errCode;
    Name_ResourceCfg       namedResource;

    /* Sanity Check: Loop around till we find the MAX channels being tested */
    while (1)
    {
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                               "MAX_QUEUE_DMA_CHANNELS", &namedResource, &errCode) == 0)
        break;
        Task_sleep(1);
    }
    maxChannels = namedResource.handle1;

    if (maxChannels > 128)
    {
        System_printf ("Error: Test Multiple writer channels supports a max of 128 channels [Requested %d]\n", maxChannels);
        return -1;
    }

    /* Cycle through and find all the channels which have been created */
    for (index = 0; index < maxChannels; index++)
    {
        sprintf (channelName, "TestReaderQueueDMAPend-%d", index);

        /* Loop around and find the channel name. */
        while (1)
        {
            chHandle[index] = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
            if (chHandle[index] != NULL)
                break;
            Task_sleep(1);
        }
    }

    /* Control comes here implies that all the channels have been found. Send out the
     * messages to the reader. */
    for (chIndex = 0; chIndex < maxChannels; chIndex++)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, mySharedHeapHandle, 90);
        if (ptrMessage == NULL)
        {
            System_printf ("Error: Out of memory in sending packet for %d channel\n", chIndex);
            return -1;
        }

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 90; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Writeback the contents of the data buffer. */
        appWritebackBuffer(ptrDataBuffer, 90);

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 90);
        Pktlib_setDataBufferLen(ptrMessage, 90);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle[chIndex], (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
            return -1;
        }
    }

    /* Delete all the msgcom channels */
    for (chIndex = 0; chIndex < maxChannels; chIndex++)
    {
        /* Delete the writer channel: */
        errCode = Msgcom_delete (chHandle[chIndex], myFreeMsgBuffer);
        if (errCode < 0)
        {
            System_printf ("Error: Unable to delete the queue DMA writer channel '%s' [Error code %d]\n", channelName, errCode);
            return -1;
        }
    }

    /* All the messages have been successfully sent over. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the writer task
 *
 *  @retval
 *      Not Applicable.
 */
Void writerControlTask(UArg arg0, UArg arg1)
{
    /* Debug Message: The Writer tests are about to start. */
    System_printf ("Debug: Starting the CORE to CORE writer tests.\n");

    /*************************************************************************
     *************************** QUEUE Writer Test ***************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue Writer Test */
    if (test_queue_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Basic Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue Writer Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Writer Test */
    if (test_queue_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue Writer Non-Blocking Test Passed\n");
    if (test_queue_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue Writer Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Writer Test */
    if (test_queue_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue Writer Non-Blocking Test Passed\n");
    if (test_queue_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue Writer Blocking Test Passed\n");

    /*****************************************************************************
     *********************** Multiple Queue Channel Tests ************************
     *****************************************************************************/
    /* Test: Multiple Queue Accumulated channel */
    if (test_multiple_queue_writer_channel(Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT) < 0)
    {
        System_printf ("Error: Multiple Writer Channel Accumulated Interrupt Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Writer Channel Accumulated Interrupt Test Passed\n");

    /* Test: Multiple Queue Interrupt channel */
    if (test_multiple_queue_writer_channel(Msgcom_QueueInterruptMode_DIRECT_INTERRUPT) < 0)
    {
        System_printf ("Error: Multiple Writer Channel Direct Interrupt Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Writer Channel Direct Interrupt Test Passed\n");

    /*************************************************************************
     ************************* QUEUE-DMA Writer Test *************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Writer Test */
    if (test_queue_dma_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Basic Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue DMA Writer Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Writer Test */
    if (test_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue DMA Writer Non-Blocking Test Passed\n");
    if (test_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Implicit Notify Queue DMA Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Implicit Notify Queue DMA Writer Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Writer Test */
    if (test_queue_dma_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue DMA Writer Non-Blocking Test Passed\n");
    if (test_queue_dma_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Accumulated Queue DMA Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Accumulated Queue DMA Writer Blocking Test Passed\n");

    /*****************************************************************************
     ********************* Multiple Queue DMA Channel Tests **********************
     *****************************************************************************/
    if (test_multiple_queueDMA_writer_channel() < 0)
    {
        System_printf ("Error: Multiple Queue DMA Writer Channel Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Queue DMA Writer Channel Test Passed\n");

    /*****************************************************************************
     *********************** Virtual Queue Channel Writer Test *******************
     *****************************************************************************/

    /* Test 1: Execute the BASIC Virtual Queue Writer Test */
    if (test_virtual_queue_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Basic Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Basic Queue Writer Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Writer Test */
    if (test_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue Writer Non-Blocking Test Passed\n");
    if (test_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue Writer Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Writer Test */
    if (test_virtual_queue_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue Writer Non-Blocking Test Passed\n");
    if (test_virtual_queue_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue Writer Blocking Test Passed\n");

    /*****************************************************************************
     ********************* Virtual Queue DMA Channel Writer Test *****************
     *****************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Writer Test */
    if (test_virtual_queue_dma_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Basic Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Basic Queue DMA Writer Non-Blocking Test Passed\n");

    /* Test 2: Execute the Implicit Notification Queue Writer Test */
    if (test_virtual_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue DMA Writer Non-Blocking Test Passed\n");
    if (test_virtual_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Implicit Notify Queue DMA Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Implicit Notify Queue DMA Writer Blocking Test Passed\n");

    /* Test 3: Execute the Accumulated Queue Writer Test */
    if (test_virtual_queue_dma_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue DMA Writer Non-Blocking Test Passed\n");
    if (test_virtual_queue_dma_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Virtual Accumulated Queue DMA Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Virtual Accumulated Queue DMA Writer Blocking Test Passed\n");

    /*****************************************************************************
     ******************** Multiple Virtual Queue Channel Writer Test *************
     *****************************************************************************/

    /* Test 1: Execute the Multiple Virtual Queue Implicit Notification Queue Writer Test */
    if (test_multiple_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Multiple Virtual Implicit Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Virtual Implicit Writer Non-Blocking Test Passed\n");

    /* Test 2: Execute the Multiple Virtual Queue Implicit Notification Queue Writer Test */
    if (test_multiple_virtual_queue_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        System_printf ("Error: Multiple Virtual Implicit Writer Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Multiple Virtual Implicit Writer Blocking Test Passed\n");

    System_printf ("------------------------------\n");
    System_printf ("Debug: All Writer Tests Passed\n");
    System_printf ("------------------------------\n");
}

/**
 *  @b Description
 *  @n
 *      This is the test task for executing the DSP Writer and ARM reader
 *
 *  @retval
 *      Not Applicable.
 */
Void test_DSPWriterARMReader (UArg arg0, UArg arg1)
{
    /* Debug Message: The Reader tests are about to start. */
    System_printf ("Debug: Starting the DSP Writer to the ARM Reader tests.\n");

    /*************************************************************************
     ************************* QUEUE-DMA writer Test *************************
     *************************************************************************/

    /* Test 1: Execute the BASIC Queue DMA Writer Test */
    if (test_queue_dma_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        System_printf ("Error: Basic Queue DMA Writer Non-Blocking Test FAILED\n");
        return;
    }
    System_printf ("Debug: Basic Queue DMA Writer Non-Blocking Test Passed\n");
    return;
}

