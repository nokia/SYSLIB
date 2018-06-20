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
#include <unistd.h>

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
    printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
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
        /* Sleep and check again after some time. */
        sleep (1);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 160);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 160; index++)
            *(ptrDataBuffer + index) = 0xAA;

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, 160);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            printf ("Error: Unable to send the message over the polling queue channel\n");
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
        sleep (1);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 90);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 90; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 90);
        Pktlib_setDataBufferLen(ptrMessage, 90);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            printf ("Error: Unable to send the message\n");
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
 *      The function tests the queue accumulated writer
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

    /* Get the channel name; which depends upon the mode being tested. */
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
        sleep (1);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 90);
        if (ptrMessage == NULL)
            continue;

        /* Populate the data buffer */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);
        for (index = 0; index < 100; index++)
            *(ptrDataBuffer + index) = 0xCC;

        /* Set the packet & data buffer length of the message */
        Pktlib_setPacketLen (ptrMessage, 100);
        Pktlib_setDataBufferLen(ptrMessage, 100);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            printf ("Error: Unable to send the message\n");
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
        sleep(1);
    }
    
    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 16);
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
            printf ("Error: Unable to send the message\n");
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
 *      The function tests the queue DMA accumulated channel writer
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
        channelName = "Accumulated-QDMA-NonBlock";
    else
        channelName = "Accumulated-QDMA-Block";

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        sleep(1);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 60);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data buffer length and populate the payload. */
        dataLen = 40;
        for (index = 0; index < dataLen; index++)
            *(ptrDataBuffer + index) = 0x33;
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length. */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            printf ("Error: Unable to send the message\n");
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
 *      The function tests the queue DMA implicit notification writer
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
        sleep(1);
    }

    /* Send out the messages to the reader. */
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Allocate a message from the Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, testHeapHandle, 60);
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
            printf ("Error: Unable to send the message\n");
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
 *      Writer thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread
 *
 *  @retval
 *      Always NULL
 */
void* writer(void* arg)
{
    printf ("Debug: Writer Test Starting.\n");

    /*************************************************************************
     ************************* QUEUE DMA Writer Test *************************
     *************************************************************************/
    if (test_queue_dma_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue DMA Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue DMA Writer Non-Blocking Test Passed\n");

    if (test_queue_dma_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue DMA Writer Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue DMA Writer Blocking Test Passed\n");    
    
    if (test_queue_dma_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue DMA Writer Non Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue DMA Writer Non Blocking Test Passed\n");    

    if (test_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue DMA Writer Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue DMA Writer Blocking Test Passed\n");    

    if (test_queue_dma_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Implicit Notify Queue DMA Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Notify Queue DMA Writer Non-Blocking Test Passed\n");    

    /*************************************************************************
     *************************** QUEUE Writer Test ***************************
     *************************************************************************/
    if (test_queue_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue Writer Non-Blocking Test Passed\n");

    if (test_queue_implicitNotify_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Implicit Queue Writer Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Queue Writer Blocking Test Passed\n");

    if (test_queue_implicitNotify_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Implicit Queue Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Implicit Queue Writer Non-Blocking Test Passed\n");

    if (test_queue_accumulated_writer(Msgcom_ChannelMode_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue Writer Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue Writer Blocking Test Passed\n");

    if (test_queue_accumulated_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Accumulated Queue Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Accumulated Queue Writer Non-Blocking Test Passed\n");

    /* Control comes here implies that all the writer tests passed */
    printf ("*******************************************************************\n");
    printf ("Debug: MSGCOM Writer Unit Test all passed\n");
    printf ("*******************************************************************\n");
    return NULL;
}

