/**
 *   @file  msgcom_queueDMA.c
 *
 *   @brief
 *      The file implements the message communication transport interface
 *      over Queues using CPDMA
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

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/msgcom/include/msgcom_internal.h>
#include <ti/runtime/pktlib/pktlib.h>

/* CPPI/QMSS Include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ********************* Queue DMA Transport Functions ******************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which enables the system interrupt if one was associated
 *      with the channel.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static inline void Msgcom_queueDMAEnableSysInt (Msgcom_channel* ptrChannel)
{
    if (ptrChannel->interruptInfo.sysInterrupt != -1)
        ptrChannel->ptrInstanceMCB->cfg.enableSysInt(ptrChannel->interruptInfo.cpIntcId,
                                                     ptrChannel->interruptInfo.sysInterrupt);
}

/**
 *  @b Description
 *  @n
 *      Utility function which disables the system interrupt if one was associated
 *      with the channel.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static inline void Msgcom_queueDMADisableSysInt (Msgcom_channel* ptrChannel)
{
#ifndef __ARMv7
    if (ptrChannel->interruptInfo.sysInterrupt != -1)
        ptrChannel->ptrInstanceMCB->cfg.disableSysInt(ptrChannel->interruptInfo.cpIntcId,
                                                      ptrChannel->interruptInfo.sysInterrupt);
#else
    /* ARM: In the case of the ARM; the UIO driver disables the UINTC interrupt because the
     * DTS file configures the interrupt_mode as 1. MSGCOM does *not* explicitly need to
     * do this and this saves an extra system call. */
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a received message buffer to the internal
 *      software buffered list.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *  @param[in]  msgBuffer
 *      Pointer to the message buffer to be added
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Msgcom_queueDMAAddBuffer(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer)
{
    void*               csHandle;
    Msgcom_ListNode*    ptrListNode;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Critical Section Enter: */
    csHandle = ptrInstanceMCB->cfg.enterCS();

    /* Get the list node associated with the buffer */
    ptrListNode = Msgcom_getListNode(msgBuffer);

    /* Enqueue the packet into the virtual channel list */
    Msgcom_listEnqueue (&ptrChannel->listObj, ptrListNode);

    /* Release the critical section. */
    ptrInstanceMCB->cfg.exitCS(csHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to remove a received message buffer from the internal
 *      software buffered list.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      The message buffer in the list. This can be NULL if the internal buffer was empty.
 */
static MsgCom_Buffer Msgcom_queueDMARemoveBuffer(Msgcom_channel* ptrChannel)
{
    MsgCom_Buffer       msgBuffer;
    void*               csHandle;
    Msgcom_ListNode*    ptrListNode;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Critical Section Enter: */
    csHandle = ptrInstanceMCB->cfg.enterCS();

    /* Get the list node associated with the buffer */
    ptrListNode = Msgcom_listDequeue (&ptrChannel->listObj);
    if (ptrListNode != NULL)
        msgBuffer = Msgcom_getBuffer(ptrListNode);
    else
        msgBuffer = NULL;

    /* Release the critical section. */
    ptrInstanceMCB->cfg.exitCS(csHandle);
    return msgBuffer;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create a named resource for the specific
 *      channel name in the named resource database.
 *
 *  @param[in]  channelName
 *      Name of the channel which is being created and being registered with the
 *      name server.
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   Error code returned by the resource manager
 */
static int32_t Msgcom_queueDMACreateNamedResource (const char* channelName, Msgcom_channel* ptrChannel)
{
    Name_ResourceCfg    namedResourceCfg;
    int32_t             errCode;

    /* Initialize the named resource configuration block. */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration: In queue channels the following is the
     * convention used to store information:
     *  - handle1 -> MSGCOM Channel Family
     *  - handle2 -> This stores the transmit queue which is used by the writer to send
     *               messages to the reader
     *  - handle3 -> Tag information field to be populated while sending messages */
    namedResourceCfg.handle1  = (uint32_t)Msgcom_ChannelType_QUEUE_DMA;
    namedResourceCfg.handle2  = ptrChannel->uInfo.queueDMAInfo.messageTxQueue;
    namedResourceCfg.handle3  = (ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  << 24) |
                                (ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  << 16) |
                                (ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi << 8)  |
                                (ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo);
    strncpy(namedResourceCfg.name, channelName, NAME_MAX_CHAR);

    /* Create the MSGCOM queue channel into the named resource database. */
    if (Name_createResource(ptrChannel->ptrInstanceMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, &errCode) < 0)
        return errCode;

    /* Named resource has been created successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the internal queue associated with
 *      the MSGCOM Queue DMA channel.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - Queue Number.
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMAGetInternalMsgQueueInfo(Msgcom_channel* ptrChannel)
{
    uint32_t      queueID;

    /* We dont support virtual channels. */
    if (ptrChannel->isVirtualChannel == 1)
        return MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED;

    /* Is this an Accumulated interrupt queue? */
    if ((ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        /* For NO Interrupt and DIRECT Interrupt we return the message receive queue. */
        queueID = Qmss_getQIDFromHandle(ptrChannel->uInfo.queueDMAInfo.messageRxQueue);
    }
    else
    {
        /* For Accumulated interrupts we return the "Accumulated monitored queue" */
        queueID = Qmss_getQIDFromHandle(ptrChannel->uInfo.queueDMAInfo.messageTmpQueue);
    }
    return (int32_t) queueID;
}

/**
 *  @b Description
 *  @n
 *      The function is used to select the Queue DMA block which can be used. There can be multiple
 *      Queue-DMA blocks on the device. Also there are 32 Rx, Tx (CPPI) channels and 32 Tx queues while
 *      there are 64 Rx flows. This implies that for flow 0 and 32; there could be a reuse of the
 *      same CPPI Tx channel, Queue.
 *
 *      The function selects which Queue-DMA block can be used and also which Rx/Tx channel along with
 *      the flow number.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *  @param[in]  ptrRxFlowCfg
 *      Pointer to the receive flow configuration
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Msgcom_queueDMASelectFlowChannels(Msgcom_channel* ptrChannel, Cppi_RxFlowCfg* ptrRxFlowCfg)
{
    Msgcom_InstanceMCB* ptrInstanceMCB;
    int32_t             index;
    uint8_t             isAllocated;
    Cppi_TxChInitCfg    txChCfg;
    Cppi_RxChInitCfg    rxChCfg;
    uint32_t            flowId;
    uint32_t            channelId;
    uint32_t            counter;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Are we executing in the dummy allocation mode? */
    if (ptrInstanceMCB->dummyAllocationMode == 1)
    {
        /***********************************************************************************************************
         * DUMMY Allocation Mode:
         ***********************************************************************************************************/
        for (index = 0; index < ptrInstanceMCB->numCPDMAInstances; index++)
        {
            /*  Open the flow for the specific CPDMA Instance */
            ptrChannel->uInfo.queueDMAInfo.flowChannelHnd = Cppi_configureRxFlow (ptrInstanceMCB->cppiHnd[index], ptrRxFlowCfg, &isAllocated);
            if (ptrChannel->uInfo.queueDMAInfo.flowChannelHnd == NULL)
            {
                /* Error: Unable to open the flow: This could be because we have exhausted all the flows in the CPDMA instance
                 * Continue to the next CPDMA Instance. */
                continue;
            }

            /* Get the receive flow identifier */
            flowId = Cppi_getFlowId(ptrChannel->uInfo.queueDMAInfo.flowChannelHnd);

            /* Populate the flow identifier in the tag information for the packet. */
            ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  = flowId;

            /* Since there are 64 receive flows and only 32 CPPI Tx, Rx Channels and 32 Tx Queues; we reuse the channels after 32. */
            if (flowId >= 32)
                channelId = flowId - 32;
            else
                channelId = flowId;

            /* Enable the appropriate transmit channel */
            txChCfg.channelNum  = channelId;
            txChCfg.priority    = 0;
            txChCfg.filterEPIB  = 0;
            txChCfg.filterPS    = 0;
            txChCfg.aifMonoMode = 0;
            txChCfg.txEnable    = Cppi_ChState_CHANNEL_ENABLE;

            /* Open the Transmit Channel: */
            ptrChannel->uInfo.queueDMAInfo.txChannelHnd = Cppi_txChannelOpen (ptrInstanceMCB->cppiHnd[index], &txChCfg, &isAllocated);

            /* Enable the appropriate receive channel */
            rxChCfg.channelNum  = channelId;
            rxChCfg.rxEnable    = Cppi_ChState_CHANNEL_ENABLE;

            /* Open the receive channel. */
            ptrChannel->uInfo.queueDMAInfo.rxChannelHnd = Cppi_rxChannelOpen (ptrInstanceMCB->cppiHnd[index], &rxChCfg, &isAllocated);

            /* Populate the flow identifier in the tag information for the packet. */
            ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  = 0;
            ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  = flowId;

            /* Debug Message: */
            System_printf ("Debug: MSGCOM Queue-DMA allocated [CPPI Block %d, Flow Id: %d Channel Id: %d]\n",
                            index, flowId, channelId);

            /* Forget the receive and transmit channels. This is done because the channels cannot be closed ever after they have been
             * enabled. The reason why this is done because there are 32 Rx, Tx (CPPI) channels and 32 Tx queues while there are
             * 64 Rx flows. This implies that for flow 0 and 32; there could be a reuse of the same CPPI Tx channel, Queue. If the
             * MSGCOM channel on delete shuts down the shared CPPI Tx channel the other active MSGCOM channel will be affected.
             * Resetting the handles to NULL will ensure that on MSGCOM channel deletion the CPPI channels are not closed
             * and remain active.
             *
             * NOTE: Any entity system which uses the CPDMA Infrastructure besides MSGCOM needs to ensure that this rule is followed. */
            ptrChannel->uInfo.queueDMAInfo.txChannelHnd = NULL;
            ptrChannel->uInfo.queueDMAInfo.rxChannelHnd = NULL;

            /* Open an Infrastructure Transmit Queue for CPDMA. */
            ptrChannel->uInfo.queueDMAInfo.messageTxQueue = Qmss_queueOpen(ptrInstanceMCB->queueType[index],
                                                                           ptrInstanceMCB->queueBase[index] + channelId,
                                                                           &isAllocated);
            if (ptrChannel->uInfo.queueDMAInfo.messageTxQueue < 0)
                return MSGCOM_QUEUE_OPEN_FAILED;

            /* Successfully allocated the receive and transit channels */
            return 0;
        }

        /* Control comes here implies that we were unable to allocate any flow. */
        return -1;
    }

    /***********************************************************************************************************
     * Load Balanced Mode: Use the CPDMA Instance Index as the starting seed.
     ***********************************************************************************************************/
    index   = ptrInstanceMCB->cpdmaInstanceIndex;
    counter = 0;

    /* Cycle through all the CPDMA instances which are supported */
    for (counter = 0; counter < ptrInstanceMCB->numCPDMAInstances; counter++)
    {
        /*  Open the flow for the specific CPDMA Instance */
        ptrChannel->uInfo.queueDMAInfo.flowChannelHnd = Cppi_configureRxFlow (ptrInstanceMCB->cppiHnd[index], ptrRxFlowCfg, &isAllocated);
        if (ptrChannel->uInfo.queueDMAInfo.flowChannelHnd != NULL)
            break;

        /* Move to the next CPDMA Instance */
        index = (index + 1) % ptrInstanceMCB->numCPDMAInstances;
    }

    /* Were we able to open a flow? */
    if (counter == ptrInstanceMCB->numCPDMAInstances)
    {
        /* Error: Unable to open the flow: This could be because we have exhausted all the flows in all the
         * CPDMA instances */
        return -1;
    }

    /* Get the receive flow identifier */
    flowId = Cppi_getFlowId(ptrChannel->uInfo.queueDMAInfo.flowChannelHnd);

    /* Populate the flow identifier in the tag information for the packet. */
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  = flowId;

    /* Since there are 64 receive flows and only 32 CPPI Tx, Rx Channels and 32 Tx Queues; we reuse the channels after 32. */
    if (flowId >= 32)
        channelId = flowId - 32;
    else
        channelId = flowId;

    /* Enable the appropriate transmit channel */
    txChCfg.channelNum  = channelId;
    txChCfg.priority    = 0;
    txChCfg.filterEPIB  = 0;
    txChCfg.filterPS    = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable    = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the Transmit Channel: */
    ptrChannel->uInfo.queueDMAInfo.txChannelHnd = Cppi_txChannelOpen (ptrInstanceMCB->cppiHnd[index], &txChCfg, &isAllocated);

    /* Enable the appropriate receive channel */
    rxChCfg.channelNum  = channelId;
    rxChCfg.rxEnable    = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the receive channel. */
    ptrChannel->uInfo.queueDMAInfo.rxChannelHnd = Cppi_rxChannelOpen (ptrInstanceMCB->cppiHnd[index], &rxChCfg, &isAllocated);

    /* Populate the flow identifier in the tag information for the packet. */
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  = 0;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  = flowId;

    /* Debug Message: */
    System_printf ("Debug: MSGCOM Queue-DMA allocated [CPPI Block %d, Flow Id: %d Channel Id: %d]\n",
                    index, flowId, channelId);

    /* Forget the receive and transmit channels. This is done because the channels cannot be closed ever after they have been
     * enabled. The reason why this is done because there are 32 Rx, Tx (CPPI) channels and 32 Tx queues while there are
     * 64 Rx flows. This implies that for flow 0 and 32; there could be a reuse of the same CPPI Tx channel, Queue. If the
     * MSGCOM channel on delete shuts down the shared CPPI Tx channel the other active MSGCOM channel will be affected.
     * Resetting the handles to NULL will ensure that on MSGCOM channel deletion the CPPI channels are not closed
     * and remain active.
     *
     * NOTE: Any entity system which uses the CPDMA Infrastructre besides MSGCOM needs to ensure that this rule is followed. */
    ptrChannel->uInfo.queueDMAInfo.txChannelHnd = NULL;
    ptrChannel->uInfo.queueDMAInfo.rxChannelHnd = NULL;

    /* Open an Infrastructure Transmit Queue for CPDMA. */
    ptrChannel->uInfo.queueDMAInfo.messageTxQueue = Qmss_queueOpen(ptrInstanceMCB->queueType[index],
                                                                   ptrInstanceMCB->queueBase[index] + channelId,
                                                                   &isAllocated);
    if (ptrChannel->uInfo.queueDMAInfo.messageTxQueue < 0)
        return MSGCOM_QUEUE_OPEN_FAILED;

    /* Modify the seed to the next CPDMA Instance */
    ptrInstanceMCB->cpdmaInstanceIndex = (ptrInstanceMCB->cpdmaInstanceIndex + 1) % ptrInstanceMCB->numCPDMAInstances;

    /* Successfully allocated the receive and transit channels */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a QUEUE DMA channel with the specified
 *      configuration.
 *
 *  @param[in]  channelName
 *      Name of the channel
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *  @param[in]  ptrChCfg
 *      Pointer to the channel configuration information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMACreateReader(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_ChannelCfg* ptrChCfg)
{
    uint8_t             isAllocated;
    Cppi_RxFlowCfg      rxFlowCfg;
    Qmss_Queue          queueInfo;
    Qmss_QueueHnd       rxQueueHandle;
    Qmss_QueueHnd       rxFreeQueueHnd;
    Qmss_Result         result;
    Qmss_AccCmdCfg      accCfg;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Copy the channel configuration */
    memcpy ((void *)&ptrChannel->cfg, (void*)ptrChCfg, sizeof(Msgcom_ChannelCfg));

    /* This is a PHYSICAL channel which has not been virtualized as of now. */
    ptrChannel->virtualChannelIdentifier = -1;

    /* Basic Validation: Ensure that the interrupt mode is correctly configured. */
    if ((ptrChannel->cfg.u.queueDMACfg.interruptMode != Msgcom_QueueInterruptMode_NO_INTERRUPT)     &&
        (ptrChannel->cfg.u.queueDMACfg.interruptMode != Msgcom_QueueInterruptMode_DIRECT_INTERRUPT) &&
        (ptrChannel->cfg.u.queueDMACfg.interruptMode != Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT))
        return MSGCOM_INVALID_PARAM;

    /* Check if the channel is blocking or not? If so we need to create a semaphore for this. */
    if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrChannel->semHandle = ptrInstanceMCB->cfg.createSem();

    /* Do we allow interrupt support or not? */
    if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT)
    {
        /* No Interrupt mode. We can open a general purpose receive queue for this.*/
        ptrChannel->uInfo.queueDMAInfo.messageRxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                       QMSS_PARAM_NOT_SPECIFIED,
                                                                       &isAllocated);
        if (ptrChannel->uInfo.queueDMAInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Populate the interrupt information: This is a NON interruptible channel. */
        ptrChannel->interruptInfo.sysInterrupt   = -1;
        ptrChannel->interruptInfo.cpIntcId       = -1;
        ptrChannel->interruptInfo.hostInterrupt  = -1;

        /* Our actual receive queue is the same as the receive queue we have opened. */
        rxQueueHandle = ptrChannel->uInfo.queueDMAInfo.messageRxQueue;
    }
    else if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Get the queue information for the message receive queue. */
        queueInfo = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue;

        /* Open the message receive queue */
        ptrChannel->uInfo.queueDMAInfo.messageRxQueue = Qmss_queueOpen ((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                                                        Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                        &isAllocated);
        if (ptrChannel->uInfo.queueDMAInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Initialize the interrupt information: */
        memset ((void *)&ptrChannel->interruptInfo, 0, sizeof(MsgCom_Interrupt));

        /* Populate the interrupt information: */
        ptrChannel->interruptInfo.sysInterrupt   = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt;
        ptrChannel->interruptInfo.cpIntcId       = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId;
        ptrChannel->interruptInfo.hostInterrupt  = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt;

        /* Register the Interrupt using the OSAL */
        if (ptrInstanceMCB->cfg.registerIsr(channelName, queueInfo,Msgcom_channelRxHandler, (MsgCom_ChHandle)ptrChannel,
                                            &ptrChannel->interruptInfo) < 0)
            return MSGCOM_REGISTER_ISR_FAILED;

#ifdef __ARMv7
        /* TODO: UIO currently requires us to enable the interrupts after they have been processed
         * else there are no other interrupts which are generated. So we workaround this problem
         * by setting the system interrupt to be the same as the host interrupt. This will now allow
         * us to use the EnableSysInt and DisableSysInt functions. */
        ptrChannel->interruptInfo.sysInterrupt = ptrChannel->interruptInfo.hostInterrupt;
#endif

        /* Our actual receive queue is the same as the receive queue we have opened. */
        rxQueueHandle = ptrChannel->uInfo.queueDMAInfo.messageRxQueue;
    }
    else
    {
        /* Low priority accumulation interrupts not supported at the moment */
        if (ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.type == Msgcom_AccumulatedChannelType_LOW)
            return MSGCOM_LOW_PRIO_INTERRUPT_NOT_SUPPORTED;

        /* Store the accumulator queue */
        queueInfo          = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accQueue;
        ptrChannel->pdspId = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.pdspId;

        /* Open the receive queue as specified by the device configuration. */
        ptrChannel->uInfo.queueDMAInfo.messageTmpQueue = Qmss_queueOpen((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                                                        Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                        &isAllocated);
        if (ptrChannel->uInfo.queueDMAInfo.messageTmpQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Our actual receive queue is the queue which will be monitored by the accumulator. */
        rxQueueHandle = ptrChannel->uInfo.queueDMAInfo.messageTmpQueue;

        /* Open a generic message receive queue: For accumulated interrupt we will pop packets from the
         * accumulator list and place them in this receive queue. */
        ptrChannel->uInfo.queueDMAInfo.messageRxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                       QMSS_PARAM_NOT_SPECIFIED,
                                                                       &isAllocated);
        if (ptrChannel->uInfo.queueDMAInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* HIGH Priority Channel: Program the accumulator immediately. */
        accCfg.channel             = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel;
        accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
        accCfg.queueEnMask         = 0;
        accCfg.queMgrIndex         = Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo));
        accCfg.maxPageEntries      = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries;
        accCfg.timerLoadCount      = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.pacingTimerCount;
        accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_NEW_PACKET;
        accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
        accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
        accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

        /* We need to get a global list which has the following
         *  - PING & PONG which is 'accCfg.maxPageEntries' * 4' bytes each.
         *  - The 'accCfg.maxPageEntries' already includes the entry count. */
        accCfg.listAddress = (uint32_t)ptrInstanceMCB->cfg.malloc(Msgcom_MemAllocMode_CACHE_COHERENT, accCfg.maxPageEntries * 4 * 2);
        if (accCfg.listAddress == 0)
            return MSGCOM_MALLOC_FAILED;

        /* Initialize the accumulator memory */
        memset ((void *)accCfg.listAddress, 0, accCfg.maxPageEntries * 4 * 2);

        /* Program the accumulator. */
        result = Qmss_programAccumulator (ptrChannel->pdspId, &accCfg);
        if (result != QMSS_ACC_SOK)
            return MSGCOM_ACCUMULATOR_CONFIG_FAILED;

        /* Initialize the interrupt information: */
        memset ((void *)&ptrChannel->interruptInfo, 0, sizeof(MsgCom_Interrupt));

        /* Populate the interrupt information: */
        ptrChannel->interruptInfo.sysInterrupt   = -1;
        ptrChannel->interruptInfo.cpIntcId       = -1;
        ptrChannel->interruptInfo.hostInterrupt  = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.interruptId;

        /* Register the Interrupt using the OSAL */
        if (ptrInstanceMCB->cfg.registerIsr(channelName, queueInfo,Msgcom_channelRxHandler, (MsgCom_ChHandle)ptrChannel,
                                            &ptrChannel->interruptInfo) < 0)
            return MSGCOM_REGISTER_ISR_FAILED;

#ifdef __ARMv7
        /* TODO: UIO currently requires us to enable the interrupts after they have been processed
         * else there are no other interrupts which are generated. So we workaround this problem
         * by setting the system interrupt to be the same as the host interrupt. This will now allow
         * us to use the EnableSysInt and DisableSysInt functions. */
        ptrChannel->interruptInfo.sysInterrupt = ptrChannel->interruptInfo.hostInterrupt;
#endif

        /* Setup the channel information. */
        ptrChannel->usePing     = 1;
        ptrChannel->pingAddress = (uint32_t)accCfg.listAddress;
        ptrChannel->pongAddress = (uint32_t)(accCfg.listAddress + (accCfg.maxPageEntries * 4));
        ptrChannel->messageIdx  = 0;
    }

    /* Open the receive free queue as specified in the configuration. */
    rxFreeQueueHnd = Qmss_queueOpen((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                    ptrChannel->cfg.u.queueDMACfg.rxFreeQueueNum,
                                    &isAllocated);
    if (rxFreeQueueHnd < 0)
        return MSGCOM_QUEUE_OPEN_FAILED;

    /* Initialize the receive flow configuration.*/
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Place the received data into the receive queue which has been opened.
     *  - In the case of No Interrupt and Direct Interrupt this is the same as the Receive queue
     *  - In the case of Accumulated Interrupts this is the Accumulator queue being monitored. */
    queueInfo = Qmss_getQueueNumber (rxQueueHandle);


    /* Configure QMSS Barrier Queue, only in case of ARM */
#if defined(__ARM_ARCH_7A__)
    uint32_t const rxQueueHandleTmp = QMSS_QUEUE_QID(rxQueueHandle);
    if ((ptrChannel->cfg.queueBarrierType == MSMC) && (ptrInstanceMCB->qmssBarrierQMsmc != (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED)) {
        System_printf ("Debug: MSGCOM Queue-DMA Configuring MSMC Queue Barrier\n"); //fzm
        Qmss_Queue msmc_q_tmp = Qmss_getQueueNumber(ptrInstanceMCB->qmssBarrierQMsmc);
        rxFlowCfg.rx_dest_qnum   = msmc_q_tmp.qNum;
        rxFlowCfg.rx_dest_qmgr   = msmc_q_tmp.qMgr;

        // Configure destination Q where descriptor will be put:
        rxFlowCfg.rx_dest_tag_lo      = (rxQueueHandleTmp & 0xff);
        rxFlowCfg.rx_dest_tag_hi      = ((rxQueueHandleTmp & 0xff00) >> 8);
        rxFlowCfg.rx_dest_tag_lo_sel  = 1;
        rxFlowCfg.rx_dest_tag_hi_sel  = 1;

    } else if ((ptrChannel->cfg.queueBarrierType == DDR) && (ptrInstanceMCB->qmssBarrierQDdr != (Qmss_QueueHnd)QMSS_PARAM_NOT_SPECIFIED)) {
        System_printf ("Debug: MSGCOM Queue-DMA Configuring DDR Queue Barrier\n"); //fzm
        Qmss_Queue ddr_q_tmp = Qmss_getQueueNumber(ptrInstanceMCB->qmssBarrierQDdr);
        rxFlowCfg.rx_dest_qnum   = ddr_q_tmp.qNum;
        rxFlowCfg.rx_dest_qmgr   = ddr_q_tmp.qMgr;

        // Configure destination Q where descriptor will be put:
        rxFlowCfg.rx_dest_tag_lo      = (rxQueueHandleTmp & 0xff);
        rxFlowCfg.rx_dest_tag_hi      = ((rxQueueHandleTmp & 0xff00) >> 8);
        rxFlowCfg.rx_dest_tag_lo_sel  = 1;
        rxFlowCfg.rx_dest_tag_hi_sel  = 1;

    } else {
        System_printf ("Debug: MSGCOM Queue-DMA Not Configuring Queue Barrier\n"); //fzm
        rxFlowCfg.rx_dest_qnum   = queueInfo.qNum;
        rxFlowCfg.rx_dest_qmgr   = queueInfo.qMgr;
    }
#else /* #if defined(__ARM_ARCH_7A__) */
    rxFlowCfg.rx_dest_qnum       = queueInfo.qNum;
    rxFlowCfg.rx_dest_qmgr       = queueInfo.qMgr;
#endif  /* #if defined(__ARM_ARCH_7A__) */

    /* Set the Destination TAG SRC HI to be overwritten from received descriptor. */
    rxFlowCfg.rx_src_tag_hi_sel = 0x4; // Virtual Channel!

    rxFlowCfg.rx_sop_offset      = 0x0;
    rxFlowCfg.rx_desc_type       = Cppi_DescType_HOST;
    rxFlowCfg.rx_einfo_present   = 0x1;

    /* Configure the receive free queue from the configuration */
    queueInfo = Qmss_getQueueNumber (rxFreeQueueHnd);
    rxFlowCfg.rx_fdq0_sz0_qnum  = queueInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr  = queueInfo.qMgr;
    rxFlowCfg.rx_fdq1_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq1_qmgr      = queueInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq2_qmgr      = queueInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum      = queueInfo.qNum;
    rxFlowCfg.rx_fdq3_qmgr      = queueInfo.qMgr;

    /* In case of startvation, retry it. */
    rxFlowCfg.rx_error_handling = 1;

    /* Allow the resource manager to determine the flow identifier. */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;

    /* Select the best CPPI DMA block: Tx, Rx channels and the Flow identifier. */
    if (Msgcom_queueDMASelectFlowChannels(ptrChannel, &rxFlowCfg) < 0)
    {
        /* Close the receive free queue */
        Qmss_queueClose (rxFreeQueueHnd);
        return MSGCOM_FLOW_CONFIG_FAILED;
    }

    /* Close the receive free queue */
    Qmss_queueClose (rxFreeQueueHnd);

    /* Create and register the channel name into the named resource database. */
    return Msgcom_queueDMACreateNamedResource (channelName, ptrChannel);
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a VIRTUAL QUEUE DMA channel over a previously
 *      create PHYSICAL QUEUE channel.
 *
 *  @param[in]  channelName
 *      Virtual channel name being created
 *  @param[in]  ptrVirtualChannel
 *      Pointer to the virtual queue channel which is being created
 *  @param[in]  ptrPhyChannel
 *      Pointer to the physical queue channel which has been created
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMACreateVirtualReader(const char* channelName, Msgcom_channel* ptrVirtualChannel, Msgcom_channel* ptrPhyChannel)
{
    void*               contextInfo;
    uint32_t            virtualChannelIdentifier;
    MsgCom_AppCallBack  appCallback;
    uint32_t            arg;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Sanity Check: Both the channels should belong to the same instance since instances are self contained */
    if (ptrPhyChannel->ptrInstanceMCB != ptrVirtualChannel->ptrInstanceMCB)
        return MSGCOM_INVALID_PARAM;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrPhyChannel->ptrInstanceMCB;

    /* Inherit the runtime information from the PHYSICAL Channel. */
    memcpy ((void *)&ptrVirtualChannel->uInfo.queueDMAInfo, (void*)&ptrPhyChannel->uInfo.queueDMAInfo,
            sizeof(Msgcom_QueueDMAChannelInfo));

    /* Inherit all the configuration from the PHYSICAL channel except the callback and argument */
    appCallback = ptrVirtualChannel->cfg.appCallBack;
    arg         = ptrVirtualChannel->cfg.arg;

    /* Copy the PHYSICAL channel configuration. */
    memcpy ((void *)&ptrVirtualChannel->cfg, (void*)&ptrPhyChannel->cfg, sizeof(Msgcom_ChannelCfg));

    /* Restore the Application callback & argument registered for the virtual channel. */
    ptrVirtualChannel->cfg.appCallBack = appCallback;
    ptrVirtualChannel->cfg.arg         = arg;

    /* Critical Section: Channel Identifier is a shared resource which needs to be protected.
     * If the PHYSICAL channel was NOT virtualized; make it since now this physical channel is
     * tainted with virtual channels. */
    contextInfo = ptrInstanceMCB->cfg.enterCS();
    for (virtualChannelIdentifier = 0; virtualChannelIdentifier < MSG_COM_MAX_VIRTUAL_CHANNEL; virtualChannelIdentifier++)
    {
        /* Is the virtual channel slot free? */
        if (ptrPhyChannel->virtualChannel[virtualChannelIdentifier] == NULL)
        {
            /* YES. Allocate this for the virtual channel. */
            ptrPhyChannel->virtualChannel[virtualChannelIdentifier] = ptrVirtualChannel;
            break;
        }
    }

    /* The physical channel has now been tainted with virtual channels. */
    ptrPhyChannel->virtualChannelIdentifier = 0;

    /* Critical Section: The Virtual channel database access is complete. */
    ptrInstanceMCB->cfg.exitCS(contextInfo);

    /* Assign the virtual queue identifier to this channel. */
    ptrVirtualChannel->virtualChannelIdentifier = virtualChannelIdentifier;

    /* Remember the physical channel on top of which the virtual channel is created. */
    ptrVirtualChannel->phyChannelHandle = ptrPhyChannel;

    /* Initialize the software virtual channel list. */
    Msgcom_listInit (&ptrVirtualChannel->listObj);

    /* Initialize the virtual identifier in the Destination TAG LO Information */
    ptrVirtualChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi = virtualChannelIdentifier;

    /* Check if the channel is blocking or not? If so we need to create a semaphore for this. */
    if (ptrVirtualChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrVirtualChannel->semHandle = ptrInstanceMCB->cfg.createSem();

    /* Create and register the channel into the named resource database. */
    return Msgcom_queueDMACreateNamedResource (channelName, ptrVirtualChannel);
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a writer channel. Writer channels are created when
 *      the application finds a MSGCOM channel.
 *
 *  @param[in]  channelName
 *      Name of the writer channel which is being created
 *  @param[in]  ptrChannel
 *      Pointer to the MSGCOM channel block
 *  @param[in]  ptrNamedResCfg
 *      Pointer to the named resource configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMACreateWriter
(
    const char*         channelName,
    Msgcom_channel*     ptrChannel,
    Name_ResourceCfg*   ptrNamedResCfg,
    int32_t*            errCode
)
{
    /* Initialize the channel information from the named resource configuration block.
     * - The Queue DMA channels on reader creation populate the information which is required
     *   by the writer channel in a certain format. */
    ptrChannel->channelType                          = Msgcom_ChannelType_QUEUE_DMA;
    ptrChannel->uInfo.queueDMAInfo.messageTxQueue    = ptrNamedResCfg->handle2;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagHi  = (ptrNamedResCfg->handle3 & 0xFF000000) >> 24;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.srcTagLo  = (ptrNamedResCfg->handle3 & 0x00FF0000) >> 16;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagHi = (ptrNamedResCfg->handle3 & 0x0000FF00) >> 8;
    ptrChannel->uInfo.queueDMAInfo.tagInfo.destTagLo = (ptrNamedResCfg->handle3 & 0x000000FF);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which extracts the virtual channel information from the
 *      meta information in the received message
 *
 *  @param[in]  msgBuffer
 *      Pointer to the received message
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Virtual Channel Identifier.
 */
static int32_t Msgcom_queueDMAGetVirtualChannel(MsgCom_Buffer* msgBuffer)
{
    /* Get the tag information from the received message. */
    Cppi_DescTag const tagInfo = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc*)msgBuffer);

    /* Return the virtual channel identifier */
    return tagInfo.srcTagHi;
}

/**
 *  @b Description
 *  @n
 *      The function is used to put a message over the specified DMA queue
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *  @param[in]  msgBuffer
 *      Pointer to the message to be sent out.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMAPut(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer)
{
    /* Configure the TAG into the descriptor:
     *  - For VIRTUAL Channels we have already populated the TAG Destination LO with the virtual channel
     *    identifier.
     *  - For both PHYSICAL and VIRTUAL channels the TAG Source LO has the flow identifier. */
    Pktlib_setTags((Ti_Pkt *)msgBuffer, &ptrChannel->uInfo.queueDMAInfo.tagInfo);

    /* Since the NAME Infrastructure channels do not support transmission/reception of
     * PS Info. Reset these fields in descriptor to ensure that this is not happening.
     * This can cause the CPDMA to lock up if incorrectly configured. */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)Pktlib_getDescFromPacket(msgBuffer), 0);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)msgBuffer);

    /* Write the packet to the specified message queue. Currently descriptor size is hardcoded.  */
    Qmss_queuePushDescSize (ptrChannel->uInfo.queueDMAInfo.messageTxQueue, (void*)msgBuffer, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode the data received on a physical channel to
 *      multiple virtual channels. This will remove the messages and place them into the
 *      appropriate virtual channel receive buffers
 *
 *  @param[in]  ptrPhyChannel
 *      Pointer to the physical channel.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Msgcom_virtualqueueDMAChannelReceiveDecode (Msgcom_channel* ptrPhyChannel)
{
    int32_t             virtualChannelId;
    Msgcom_channel*     ptrVirtualChannel;
    MsgCom_Buffer       msgBuffer;
    uint32_t*           rxCompletionList;
    uint32_t            count;
    void*               csHandle;
    int32_t             retVal;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrPhyChannel->ptrInstanceMCB;

    /* Disable the System Interrupt if one was associated with the PHYSICAL channel. */
    Msgcom_queueDMADisableSysInt(ptrPhyChannel);

    /* Is the Physical Channel Direct Interrupt or in NO interrupt mode? */
    if ((ptrPhyChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrPhyChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        while (1)
        {
            /* Pop a packet from the message queue. */
            msgBuffer = Qmss_queuePop (ptrPhyChannel->uInfo.queueDMAInfo.messageRxQueue);
            if (msgBuffer == NULL)
            {
                /* Message was NOT available in the PHYSICAL channel. We can reenable
                 * system interrupts if one was associated with the channel; this way we can
                 * now get an interrupt if a newer packet is received. */
                Msgcom_queueDMAEnableSysInt(ptrPhyChannel);
                return;
            }

            /* Message was available. Get the message. */
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(msgBuffer));

            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
             * takes over ownership of the packet.
             ******************************************************************************/
            Pktlib_getOwnership(ptrPhyChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)msgBuffer);

            /* Get the virtual channel information from the message */
            virtualChannelId  = Msgcom_queueDMAGetVirtualChannel(msgBuffer);
            ptrVirtualChannel = ptrPhyChannel->virtualChannel[virtualChannelId];
            if (ptrVirtualChannel == NULL)
                while (1);

            /* Add the received message buffer to the virtual channel. If there is no space in the
             * virtual channel receive buffers we will drop the packet. */
            if (Msgcom_queueDMAAddBuffer (ptrVirtualChannel, msgBuffer) < 0)
                continue;

            /* If the virtual channel was blocking we post the semaphore. */
            if (ptrVirtualChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
                ptrInstanceMCB->cfg.postSem(ptrVirtualChannel->semHandle);

            /* Call the application call back routine if one was registered for the virtual channel
             * because data is NOW available. */
            if (ptrVirtualChannel->cfg.appCallBack != NULL)
                ptrVirtualChannel->cfg.appCallBack((MsgCom_ChHandle)ptrVirtualChannel, ptrVirtualChannel->cfg.arg);
        }
    }

    /* Accumulated Interrupt Channels are handled differently */
    while (1)
    {
        /* We can proceed only if an interrupt was detected. If there was no interrupt
         * detected we are done with the work here. */
        if (ptrPhyChannel->interruptDetected == 0)
            return;

        /* We received an interrupt; we need to now check if we need to process the PING
         * or PONG list. */
        if (ptrPhyChannel->usePing == 1)
            rxCompletionList = (uint32_t*)ptrPhyChannel->pingAddress;
        else
            rxCompletionList = (uint32_t*)ptrPhyChannel->pongAddress;

        /* Cycle through all the entries in the receive completion list. */
        count = *rxCompletionList;

        /* Message was available. Use the Message Index to pick it up. */
        msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR(rxCompletionList[ptrPhyChannel->messageIdx + 1]);

        /* Convert the descriptor which we read from the completion list to a virtual address*/
        msgBuffer = Osal_qmssConvertDescPhyToVirt(0, msgBuffer);

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
         * takes over ownership of the packet.
         ******************************************************************************/
        Pktlib_getOwnership(ptrPhyChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)msgBuffer);

        /* Get the virtual channel information from the message */
        virtualChannelId  = Msgcom_queueDMAGetVirtualChannel(msgBuffer);
        ptrVirtualChannel = ptrPhyChannel->virtualChannel[virtualChannelId];
        if (ptrVirtualChannel == NULL)
            while (1);

        /* Clear the accumulator entry. */
        rxCompletionList[ptrPhyChannel->messageIdx + 1] = 0;

        /* Add the received message buffer to the virtual channel. If there is no space in the
         * virtual channel receive buffers we will drop the packet. But continue to handle the
         * rest of the packets in the physical channel. This is required because we might have
         * multiple virtual channels on a single physical channel and we cannot allow one *bad*
         * virtual channel to stop operation on the rest of the channels. */
        retVal = Msgcom_queueDMAAddBuffer (ptrVirtualChannel, msgBuffer);

        /* Increment the message index */
        ptrPhyChannel->messageIdx++;

        /* Have we processed all the entries? */
        if (count == ptrPhyChannel->messageIdx)
        {
            /* We have processed all the packets; swap PING and PONG and acknowledge the interrupt. */
            if (ptrPhyChannel->usePing == 1)
                ptrPhyChannel->usePing = 0;
            else
                ptrPhyChannel->usePing = 1;

            /* Reset the message index */
            ptrPhyChannel->messageIdx = 0;

            /* CRITICAL Section: Decrement the number of interrupts which have been
             * detected. This is a shared counter with the receive handler and so needs to
             * be protected. */
            csHandle = ptrInstanceMCB->cfg.enterCS();
            ptrPhyChannel->interruptDetected--;
            ptrInstanceMCB->cfg.exitCS(csHandle);

            /* Signal end of interrupt processing */
            if (ptrPhyChannel->pdspId == Qmss_PdspId_PDSP7) //fzm
            {
                Qmss_ackInterruptByIntd(0, ptrPhyChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel, 1);
                Qmss_setEoiVectorByIntd(0, Qmss_IntdInterruptType_HIGH, ptrPhyChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel);
            }
            else
            {
                Qmss_ackInterruptByIntd(1, ptrPhyChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel, 1);
                Qmss_setEoiVectorByIntd(1, Qmss_IntdInterruptType_HIGH, ptrPhyChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel);
            }
            /* Reenable the interrupts if they had been disabled. */
            Msgcom_queueDMAEnableSysInt (ptrPhyChannel);
        }

        /* It is possible that by the time control comes here the packet has been dropped since the virtual
         * channel receive buffer is full; so if this is the case there is no need to post the semaphore
         * and invoke the callback function */
        if (retVal < 0)
            continue;

        /* If the virtual channel was blocking we post the semaphore. */
        if (ptrVirtualChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
            ptrInstanceMCB->cfg.postSem(ptrVirtualChannel->semHandle);

        /* Call the application call back routine if one was registered for the virtual channel
         * because data is NOW available. */
        if (ptrVirtualChannel->cfg.appCallBack != NULL)
            ptrVirtualChannel->cfg.appCallBack((MsgCom_ChHandle)ptrVirtualChannel, ptrVirtualChannel->cfg.arg);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a message over the specified DMA queue
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *  @param[out]  msgBuffer
 *      Pointer to the message which is received.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMAGet(Msgcom_channel* ptrChannel, MsgCom_Buffer** msgBuffer)
{
    uint32_t*               rxCompletionList;
    uint32_t                count;
    void*                   csHandle;
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Get the MSGCOM instance information. */
    ptrInstanceMCB = ptrChannel->ptrInstanceMCB;

    /* Check if the channel is a virtual channel or not? */
    if (ptrChannel->isVirtualChannel == 1)
    {
        while (1)
        {
            /* Virtual channel: Get a pending packet from the virtual channel if one was already present */
            *msgBuffer = Msgcom_queueDMARemoveBuffer(ptrChannel);
            if (*msgBuffer != NULL)
                return 0;

            /* Message was not available in the virtual channel. There could be messages which
             * are pending to be serviced in the physical channel. These messages need to be decoded
             * and placed into the appropriate virtual channel buffers. */
            Msgcom_virtualqueueDMAChannelReceiveDecode(ptrChannel->phyChannelHandle);

            /* Now lets try again and see if there is any pending messages on the specific virtual channel */
            *msgBuffer = Msgcom_queueDMARemoveBuffer(ptrChannel);
            if (*msgBuffer != NULL)
                return 0;

            /* Message was NOT available. Is this a blocking channel? */
            if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
            {
                /* YES. Pend on the semaphore till the data is available. */
                ptrInstanceMCB->cfg.pendSem(ptrChannel->semHandle);
                continue;
            }
            else
            {
                /* NO. Non-Blocking channel with no data available. */
                return 0;
            }
        }
    }

    /* PHYSICAL Channel: For DIRECT Interrupts and NO Interrupts we pop off a message from the receive queue. */
    if ((ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        while (1)
        {
            /* Pop a packet from the message queue. */
            *msgBuffer = Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageRxQueue);
            if (*msgBuffer != NULL)
            {
                /* Message was available. We return the received message. */
                *msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(*msgBuffer));

                /******************************************************************************
                 * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
                 * takes over ownership of the packet.
                 ******************************************************************************/
                Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)(*msgBuffer));
                break;
            }
            else
            {
                /* Message was NOT available. We should reenable the system interrupt so that we
                 * can get notified once we get newer packets. */
                Msgcom_queueDMAEnableSysInt(ptrChannel);

                // fzm--> Check if any descriptor arrived between last check and interrupt enable
                *msgBuffer = Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageRxQueue);
                if (*msgBuffer != NULL)
                {
                    *msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(*msgBuffer));
                    Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)(*msgBuffer));
                    Msgcom_queueDMADisableSysInt(ptrChannel);
                    break;
                }
                //fzm<--

                /* Is this a blocking channel? */
                if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
                {
                    /* YES. Pend on the semaphore till the data is available. */
                    ptrInstanceMCB->cfg.pendSem(ptrChannel->semHandle);
                    continue;
                }
                else
                {
                    /* NO. Non-Blocking channel with no data available. */
                    break;
                }
            }
        }
        /* Get operation was successful. */
        return 0;
    }

    /* Accumulated Interrupt Channels are handled differently. */
    while (1)
    {
        /* We can proceed only if an interrupt was detected? */
        if (ptrChannel->interruptDetected == 0)
        {
            /* No Interrupt Detected. If this is a blocking socket we wait for a message to arrive. */
            if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
            {
                ptrInstanceMCB->cfg.pendSem(ptrChannel->semHandle);
                continue;
            }

            /* Non-blocking socket; there is no message available. */
            *msgBuffer = NULL;
            return 0;
        }

        /* We received an interrupt; we need to now check if we need to process the PING
         * or PONG list. */
        if (ptrChannel->usePing == 1)
            rxCompletionList = (uint32_t*)ptrChannel->pingAddress;
        else
            rxCompletionList = (uint32_t*)ptrChannel->pongAddress;

        /* Cycle through all the entries in the receive completion list. */
        count = *rxCompletionList;

        /* Message was available. Use the Message Index to pick it up. */
        *msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR(rxCompletionList[ptrChannel->messageIdx + 1]);

        /* Convert the descriptor which we read from the completion list to a virtual address*/
        *msgBuffer = Osal_qmssConvertDescPhyToVirt(0, *msgBuffer);

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
         * takes over ownership of the packet.
         ******************************************************************************/
        Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)(*msgBuffer));

        /* Clear the accumulator entry. */
        rxCompletionList[ptrChannel->messageIdx + 1] = 0;

        /* Increment the message index */
        ptrChannel->messageIdx++;

        /* Have we processed all the entries? */
        if (count == ptrChannel->messageIdx)
        {
            /* We have processed all the packets; swap PING and PONG and acknowledge the interrupt. */
            if (ptrChannel->usePing == 1)
                ptrChannel->usePing = 0;
            else
                ptrChannel->usePing = 1;

            /* Reset the message index */
            ptrChannel->messageIdx = 0;

            /* CRITICAL Section: Decrement the number of interrupts which have been
             * detected. This is a shared counter with the receive handler and so needs to
             * be protected. */
            csHandle = ptrInstanceMCB->cfg.enterCS();
            ptrChannel->interruptDetected--;
            ptrInstanceMCB->cfg.exitCS(csHandle);

            /* Signal end of interrupt processing */
            if (ptrChannel->pdspId == Qmss_PdspId_PDSP7) //fzm
            {
                Qmss_ackInterruptByIntd(0, ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel, 1);
                Qmss_setEoiVectorByIntd(0, Qmss_IntdInterruptType_HIGH, ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel);
            }
            else
            {
                Qmss_ackInterruptByIntd(1, ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel, 1);
                Qmss_setEoiVectorByIntd(1, Qmss_IntdInterruptType_HIGH, ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel);
            }
            /* Reenable the interrupts if they had been disabled. */
            Msgcom_queueDMAEnableSysInt (ptrChannel);
        }
        /* Get operation was successful. */
        return 0;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to handle the Queue DMA Channel ISR.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Msgcom_queueDMARxHandler(Msgcom_channel* ptrChannel)
{
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Get the MSGCOM instance information. */
    ptrInstanceMCB = ptrChannel->ptrInstanceMCB;

    /* The Channel receive Handler is supposed to execute only for PHYSICAL channels. */
    if (ptrChannel->isVirtualChannel == 1)
        return;

    /* Disable the System Interrupt if one was associated with the PHYSICAL channel. We have already
     * received a notification that there was data present in the queue. We dont need any more
     * system interrupts to indicate this again. */
    Msgcom_queueDMADisableSysInt(ptrChannel);

    /* Check if this is for NON Interrupt support. */
    if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT)
    {
        /* YES. The application can periodically call the Message Communicator Receive Handler
         * to check if there is data on the message Rx Queue or not? This mode can be used to
         * support BLOCKING channels in the NON interrupt mode. Check if there are packets
         * available on the message receive queue or not? */
        if (Qmss_getQueueEntryCount(ptrChannel->uInfo.queueDMAInfo.messageRxQueue) != 0)
        {
            /* YES. Message was available. Determine if the channel was virtualized. */
            if (ptrChannel->virtualChannelIdentifier != -1)
            {
                /* Channel was virtualized. We need to pick up the data from the PHYSICAL channel
                 * and decode it to wake up the VIRTUAL channels. */
                Msgcom_virtualqueueDMAChannelReceiveDecode(ptrChannel);
            }
            else
            {
                /* Channel was NOT virtualized. */
                if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
                    ptrInstanceMCB->cfg.postSem(ptrChannel->semHandle);
            }
        }
        else
        {
            /* No Message was available. */
            return;
        }
    }

    /* Check the Interrupt Mode? */
    if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Interrupt Mode: Is the channel virtualized? */
        if (ptrChannel->virtualChannelIdentifier != -1)
        {
            /* Channel was virtualized. We need to pick up the data from the PHYSICAL channel
             * and decode it to wake up the VIRTUAL channels. */
            Msgcom_virtualqueueDMAChannelReceiveDecode(ptrChannel);
        }
        else
        {
            /* Channel was NOT virtualized; so post the semaphore if the channel was blocking since
             * data is now available. */
            if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
                ptrInstanceMCB->cfg.postSem(ptrChannel->semHandle);
        }
    }
    else
    {
        void*   csHandle;

        /* CRITICAL Section: Increment the number of interrupts which have been
         * detected. This is a shared counter with the receive handler and so needs to
         * be protected. */
        csHandle = ptrInstanceMCB->cfg.enterCS();
        ptrChannel->interruptDetected++;
        ptrInstanceMCB->cfg.exitCS(csHandle);

        /* Is the channel virtualized? */
        if (ptrChannel->virtualChannelIdentifier != -1)
        {
            /* Channel was virtualized. We need to pick up the data from the PHYSICAL channel
             * and decode it to wake up the VIRTUAL channels. */
            Msgcom_virtualqueueDMAChannelReceiveDecode(ptrChannel);
        }
        else
        {
            /* Channel was NOT virtualized; so post the semaphore if the channel was blocking since
             * data is now available. */
            if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
                ptrInstanceMCB->cfg.postSem(ptrChannel->semHandle);
        }
    }

    /* Call the application call back routine if one was registered. */
    if (ptrChannel->cfg.appCallBack != NULL)
        ptrChannel->cfg.appCallBack((MsgCom_ChHandle)ptrChannel, ptrChannel->cfg.arg);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a previously opened queue DMA channel.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information to be deleted.
 *  @param[in]  freePkt
 *      Application registered call back API to cleanup any pending messages
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Msgcom_queueDMADelete(Msgcom_channel* ptrChannel, Msgcom_freePkt freePkt)
{
    void*                       contextInfo;
    uint32_t                    index;
    uint32_t*                   accumlatorList;
    Qmss_Queue                  queueInfo;
    Qmss_Result                 result;
    Qmss_AccCmdCfg              accCfg;
    Msgcom_channel*             ptrPhyChannel;
    MsgCom_Buffer*              msgBuffer;
    Msgcom_InstanceMCB*         ptrInstanceMCB;

    /* Get the MSGCOM instance information. */
    ptrInstanceMCB = ptrChannel->ptrInstanceMCB;

    /* Determine the type of channel being deleted? */
    if (ptrChannel->isReader == 0)
    {
        /* Writer channel: These channels are responsible for ensuring that the transmit queue is
         * empty and cleaning up any packets */
        msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageTxQueue));
        while (msgBuffer != NULL)
        {
            freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageTxQueue));
        }
        return 0;
    }

    /* Critical Section: The Virtual channel database is a shared resource and needs to be
     * protected against concurrent access. */
    contextInfo = ptrInstanceMCB->cfg.enterCS();

    /* Is this a virtual channel which is being deleted? */
    if (ptrChannel->isVirtualChannel == 1)
    {
        /* YES. Get the PHYSICAL channel handle. */
        ptrPhyChannel = ptrChannel->phyChannelHandle;

        /* Cycle through all the virtual channels and update the physical channel database. */
        for (index = 0; index < MSG_COM_MAX_VIRTUAL_CHANNEL; index++)
        {
            /* Did we get a match on the channel being deleted? */
            if (ptrPhyChannel->virtualChannel[index] == ptrChannel)
            {
                /* YES. Update the physcial channel record to delete the virtualized channel */
                ptrPhyChannel->virtualChannel[index] = NULL;
                break;
            }
        }

        /* Are there any more virtual channels associated with the physical channel? */
        for (index = 0; index < MSG_COM_MAX_VIRTUAL_CHANNEL; index++)
        {
            if (ptrPhyChannel->virtualChannel[index] != NULL)
                break;
        }

        /* Physical channel will remain virtualized even if there is a single virtual
         * channel associated with it else move it back to the non virtualized state. */
        if (index == MSG_COM_MAX_VIRTUAL_CHANNEL)
            ptrPhyChannel->virtualChannelIdentifier = -1;
        else
            ptrPhyChannel->virtualChannelIdentifier = 0;

        /* Critical Section End: Virtual Channel database access is complete. */
        ptrInstanceMCB->cfg.exitCS(contextInfo);

        /* Cleanup any pending packets in the virtual channel buffers. */
        while (1)
        {
            /* Remove the buffer from the internal buffer. */
            msgBuffer = Msgcom_queueDMARemoveBuffer(ptrChannel);
            if (msgBuffer == NULL)
                break;

            /* Invoke the application call back API to clean the packet. */
            freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
        }

        /* Delete the semaphore we got for this channel's blocking implementation */
        if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
            ptrInstanceMCB->cfg.deleteSem (ptrChannel->semHandle);

        /* Virtual channel deletion is successful. */
        return 0;
    }

    /* Physical Channel is being deleted. Ensure that there are no virtual channels associated with it. */
    for (index = 0; index < MSG_COM_MAX_VIRTUAL_CHANNEL; index++)
    {
        /* Is there an association present. */
        if (ptrChannel->virtualChannel[index] != NULL)
        {
            /* YES. There exists a virtual channel on the physical channel. This channel cannot be deleted */
            ptrInstanceMCB->cfg.exitCS(contextInfo);
            return MSGCOM_CHANNEL_IN_USE;
        }
    }

    /* Critical Section End: Virtual Channel database access is complete. */
    ptrInstanceMCB->cfg.exitCS(contextInfo);

#ifdef __ARMv7
    /* TODO: UIO currently requires us to enable the interrupts after they have been processed
     * else there are no other interrupts which are generated. Here we restore back the system
     * interrupt to -1 to undo the workaround. System interrupt is not applicable in the ARM
     * domain */
    ptrChannel->interruptInfo.sysInterrupt = -1;
#endif

    /* Delete the channel as per its properties: */
    if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT)
    {
        /* Accumulator Channel: Initialize the accumulator configuration. */
        memset(&accCfg, 0, sizeof (Qmss_AccCmdCfg));

        /* Populate the accumulator configuration. */
        accCfg.channel = ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.accChannel;
        accCfg.command = Qmss_AccCmd_DISABLE_CHANNEL;

        /* Disable the accumulator. */
        result = Qmss_programAccumulator(ptrChannel->pdspId, &accCfg);
        if (result != QMSS_ACC_SOK)
            return MSGCOM_ACCUMULATOR_CONFIG_FAILED;

        /* Get the queue number */
        queueInfo = Qmss_getQueueNumber(ptrChannel->uInfo.queueDMAInfo.messageTmpQueue);

        /* Remove the registered ISR using the OSAL */
        if (ptrInstanceMCB->cfg.deregisterIsr(ptrChannel->channelName, queueInfo, &ptrChannel->interruptInfo) < 0)
            return MSGCOM_DEREGISTER_ISR_FAILED;

        /* Cycle through the accumulated PING list & clean up any pending packets. */
        accumlatorList = (uint32_t *)ptrChannel->pingAddress;
        for (index = 0; index < accumlatorList[0]; index++)
        {
            /* Get the Message buffer from the accumulated list. */
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)accumlatorList[index + 1]);

            /* Convert the descriptor which we read from the completion list to a virtual address*/
            msgBuffer = Osal_qmssConvertDescPhyToVirt(0, msgBuffer);

            /* Pass the control to the application cleanup API. */
            if (msgBuffer != NULL)
                freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
        }

        /* Cycle through the accumulated PONG list & clean up any pending packets. */
        accumlatorList = (uint32_t *)ptrChannel->pongAddress;
        for (index = 0; index < accumlatorList[0]; index++)
        {
            /* Get the Message buffer from the accumulated list. */
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)accumlatorList[index + 1]);

            /* Convert the descriptor which we read from the completion list to a virtual address*/
            msgBuffer = Osal_qmssConvertDescPhyToVirt(0, msgBuffer);

            /* Pass the control to the application cleanup API. */
            if (msgBuffer != NULL)
                freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
        }

        /* Cleanup the memory allocated for the Accumulated lists. */
        ptrInstanceMCB->cfg.free(Msgcom_MemAllocMode_CACHE_COHERENT, (void*)ptrChannel->pingAddress,
                        ptrChannel->cfg.u.queueDMACfg.queueDMAIntrUnion.accCfg.maxPageEntries * 4 * 2);
    }
    else if (ptrChannel->cfg.u.queueDMACfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Direct Interrupt Channel: Get the queue information for the message receive queue. */
        queueInfo = Qmss_getQueueNumber(ptrChannel->uInfo.queueDMAInfo.messageRxQueue);

        /* Deregister the ISR from the OSAL. */
        if (ptrInstanceMCB->cfg.deregisterIsr(ptrChannel->channelName, queueInfo, &ptrChannel->interruptInfo) < 0)
            return MSGCOM_DEREGISTER_ISR_FAILED;
    }

    /* Empty the receive queue. */
    msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageRxQueue));
    while (msgBuffer != NULL)
    {
        freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
        msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueDMAInfo.messageRxQueue));
    }

    /* Close & disable the transmit channel. */
    if (ptrChannel->uInfo.queueDMAInfo.txChannelHnd != NULL)
    {
        Cppi_channelDisable(ptrChannel->uInfo.queueDMAInfo.txChannelHnd);
        Cppi_channelClose(ptrChannel->uInfo.queueDMAInfo.txChannelHnd);
    }

    /* Close & disable the receive channel. */
    if (ptrChannel->uInfo.queueDMAInfo.rxChannelHnd != NULL)
    {
        Cppi_channelDisable(ptrChannel->uInfo.queueDMAInfo.rxChannelHnd);
        Cppi_channelClose(ptrChannel->uInfo.queueDMAInfo.rxChannelHnd);
    }

    /* Close the receive flow. */
    if (ptrChannel->uInfo.queueDMAInfo.flowChannelHnd != NULL)
        Cppi_closeRxFlow (ptrChannel->uInfo.queueDMAInfo.flowChannelHnd);

    /* Delete the semaphore we got for this channel's blocking implementation */
    if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrInstanceMCB->cfg.deleteSem (ptrChannel->semHandle);

    /* Close the receive & transmit queue. */
    Qmss_queueClose(ptrChannel->uInfo.queueDMAInfo.messageRxQueue);
    Qmss_queueClose(ptrChannel->uInfo.queueDMAInfo.messageTxQueue);

    /* Close the temporary queue */
    if (ptrChannel->uInfo.queueDMAInfo.messageTmpQueue != 0)
        Qmss_queueClose(ptrChannel->uInfo.queueDMAInfo.messageTmpQueue);

    return 0;
}
