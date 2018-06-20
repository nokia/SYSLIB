/**
 *   @file  writer.c
 *
 *   @brief   
 *      Writer Unit Test Code.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

/* MCSDK Include Files. */
#include <ti/runtime/hplib/hplib_util.h>

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
Msgcom_InstHandle              appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global test execution status. */
extern uint32_t gTestExecutionCompleted;

/**********************************************************************
 ************************* Unit Test Functions ************************
 **********************************************************************/

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

    /* Debug Message: */
    printf ("Debug: Finding QueueDMA Channel '%s'\n", channelName);

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        chHandle = Msgcom_find(appMsgcomInstanceHandle, channelName, &errCode);
        if (chHandle != NULL)
            break;
        sleep(1);
    }

    /* Debug Message: */
    printf ("Debug: QueueDMA Channel '%s' has been found. Sending messages...\n", channelName);
    
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
        errCode = Msgcom_putMessage(chHandle, (MsgCom_Buffer*)ptrMessage);
        if (errCode < 0)
        {
            printf ("Error: Unable to send the message\n");
            return -1;
        }

        /* Increment the message counter */
        messageCounter++;
    }

    /* All the messages have been successfully transmitted. */
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

    /* Test: Execute the writer tests on the Queue DMA channel in non blocking mode. */
    if (test_queue_dma_basic_writer(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue DMA Writer Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue DMA Writer Non-Blocking Test Passed\n");

    /* Test have been completed successfully. */
    gTestExecutionCompleted = 1;
    return NULL;
}

