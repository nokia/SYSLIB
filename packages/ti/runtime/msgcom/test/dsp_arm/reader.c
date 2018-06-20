/**
 *   @file  reader.c
 *
 *   @brief   
 *      Reader Unit Test Code for DSP-ARM
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

/* MCSDK Include Files. */
#include <ti/runtime/hplib/hplib_util.h>

/* SYSLIB Include Files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_proxyClient.h>
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

/* Name Client Handle: */
extern Name_ClientHandle        nameClientHandle;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle       appMsgcomInstanceHandle;

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
 *      The function is used to cleanup the MSGCOM data buffers if there are 
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle registered with the MSGCOM instance
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
 *      The function tests the Queue DMA reader channel
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
    int32_t                errCode;

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
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errCode);
    if (chHandle == 0)
    {
        printf ("Error: Unable to open the channel. Error : %d\n", errCode);
        return -1;
    }

    /* Push the channel name from the ARM to the DSP realm. */
    if (Name_push (nameClientHandle, channelName, Name_ResourceBucket_INTERNAL_SYSLIB, 
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", channelName, errCode);
        return -1;
    }
    printf ("Debug: Channel Name '%s' pushed successfully to the DSP realm\n", channelName);

    /* Wait for the messages to be received. */ 
    while (messageCounter < MAX_TEST_MESSAGES)
    {
        /* Is the channel operating in non-blocking mode? */
        if (mode == Msgcom_ChannelMode_NON_BLOCKING)
        {
            /* Yes. Wait for the message to be received. */
            Msgcom_getMessage (chHandle, (MsgCom_Buffer**)&ptrMessage); 
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
            printf ("Error: Invalid packet length detected in the received packet [Received %d bytes]\n", Pktlib_getPacketLen(ptrMessage));
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
#if 0
    cpu_set_t              cpu_set;

    /* Initialize the CPU set: This will allow the benchmarking */
    CPU_ZERO(&cpu_set);
    CPU_SET(0, &cpu_set);
    hplib_utilSetupThread(0, &cpu_set);
#endif    
    printf ("Debug: Reader Test Starting.\n");

    if (test_queue_dma_basic_reader(Msgcom_ChannelMode_NON_BLOCKING) < 0)
    {
        printf ("Error: Basic Queue DMA Reader Non-Blocking Test FAILED\n");
        return NULL;
    }
    printf ("Debug: Basic Queue DMA Reader Non-Blocking Test Passed\n");

    /* Test have been completed successfully. */
    gTestExecutionCompleted = 1;
    return NULL;
}

