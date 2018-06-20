/**
 *   @file  msgcom_queue.c
 *
 *   @brief
 *      The file implements the message communication transport interface
 *      over Queues
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

/* MCSDK Include Files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* SYSLIB Include Files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/msgcom/include/msgcom_internal.h>
#include <ti/runtime/pktlib/pktlib.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ********************** Queue Transport Functions *********************
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
static inline void Msgcom_queueEnableSysInt (Msgcom_channel* ptrChannel)
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
static inline void Msgcom_queueDisableSysInt (Msgcom_channel* ptrChannel)
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
static int32_t Msgcom_queueCreateNamedResource (const char* channelName, Msgcom_channel* ptrChannel)
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
     *  - handle3 -> Set to 1 indicates that the channel is a virtual channel or else it
     *               will be set to 0.
     *  - handle4 -> For virtual channels this is the virtual channel identifier. */
    namedResourceCfg.handle1  = (uint32_t)Msgcom_ChannelType_QUEUE;
    namedResourceCfg.handle2  = (uint32_t)ptrChannel->uInfo.queueInfo.messageTxQueue;
    namedResourceCfg.handle3  = ptrChannel->isVirtualChannel;
    namedResourceCfg.handle4  = ptrChannel->virtualChannelIdentifier;
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
static int32_t Msgcom_queueGetVirtualChannel(MsgCom_Buffer* msgBuffer)
{
    /* Get the tag information from the received message. */
    Cppi_DescTag const tagInfo = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc*)msgBuffer);

    /* Return the virtual channel identifier */
    return tagInfo.srcTagHi;
}

/**
 *  @b Description
 *  @n
 *      Utility function which sets the virtual channel information
 *      into the message.
 *
 *  @param[in]  msgBuffer
 *      Pointer to the received message
 *  @param[in]  virtualChannelId
 *      Virtual Channel Identifier to be configured.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Msgcom_queueSetVirtualChannel(MsgCom_Buffer* msgBuffer, int32_t const virtualChannelId)
{
    Cppi_DescTag    tagInfo;

    /* Configure the TAG Information. */
    tagInfo.destTagLo = 0;
    tagInfo.destTagHi = 0;
    tagInfo.srcTagHi  = virtualChannelId;
    tagInfo.srcTagLo  = 0;

    /* Set the tag information from the received message. */
    Pktlib_setTags((Ti_Pkt *)msgBuffer, &tagInfo);
    return;
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
static int32_t Msgcom_queueAddBuffer(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer)
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
static MsgCom_Buffer Msgcom_queueRemoveBuffer(Msgcom_channel* ptrChannel)
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
 *      The function is used to get the internal queue associated with
 *      the MSGCOM Queue channel.
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
int32_t Msgcom_queueGetInternalMsgQueueInfo(Msgcom_channel* ptrChannel)
{
    uint32_t      queueID;

    /* We dont support virtual channels. */
    if (ptrChannel->isVirtualChannel == 1)
        return MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED;

    /* Is this an Accumulated interrupt queue? */
    if ((ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        /* For NO Interrupt and DIRECT Interrupt we return the message receive queue.
         * When a packet is received it will be present in the specified queue. */
        queueID = Qmss_getQIDFromHandle(ptrChannel->uInfo.queueInfo.messageRxQueue);
    }
    else
    {
        /* For Accumulated interrupts we return the "Accumulated monitored queue". In this case
         * this is the same as the transmit queue because when we push a packet to this queue
         * we want the ACCUMULATOR to execute. */
        queueID = Qmss_getQIDFromHandle(ptrChannel->uInfo.queueInfo.messageTxQueue);
    }
    return (int32_t) queueID;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a QUEUE channel with the specified
 *      configuration.
 *
 *  @param[in]  channelName
 *      Channel Name
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
int32_t Msgcom_queueCreateReader(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_ChannelCfg* ptrChCfg)
{
    uint8_t             isAllocated;
    Qmss_Queue          queueInfo;
    Qmss_Result         result;
    Qmss_AccCmdCfg      accCfg;
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Copy the channel configuration passed */
    memcpy ((void *)&ptrChannel->cfg, (void*)ptrChCfg, sizeof(Msgcom_ChannelCfg));

    /* This is a PHYSICAL channel which has not been virtualized as of now. */
    ptrChannel->virtualChannelIdentifier = -1;

    /* Check if the channel is blocking or not? If so we need to create a semaphore for this. */
    if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrChannel->semHandle = ptrInstanceMCB->cfg.createSem();

    /* Do we allow interrupt support or not? */
    if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT)
    {
        /* No Interrupt mode: Open the message receive queue. */
        ptrChannel->uInfo.queueInfo.messageRxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                    QMSS_PARAM_NOT_SPECIFIED,
                                                                    &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Get the queue information for the message receive queue. */
        queueInfo = Qmss_getQueueNumber(ptrChannel->uInfo.queueInfo.messageRxQueue);

        /* Open the message transmit queue which is the same as the message receive queue. */
        ptrChannel->uInfo.queueInfo.messageTxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                    Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                    &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageTxQueue  < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Populate the interrupt information: This is a NON interruptible channel. */
        ptrChannel->interruptInfo.sysInterrupt   = -1;
        ptrChannel->interruptInfo.cpIntcId       = -1;
        ptrChannel->interruptInfo.hostInterrupt  = -1;

        /* Message queue was opened successfully. */
        return Msgcom_queueCreateNamedResource (channelName, ptrChannel);
    }
    else if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Get the queue information for the message receive queue. */
        queueInfo = ptrChannel->cfg.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue;

        /* Cycle through all the direct interrupt queue groups and open a direct interrupt queue. */
        ptrChannel->uInfo.queueInfo.messageRxQueue = Qmss_queueOpen ((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                                                     Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                     &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Open the message transmit queue which is the same as the message receive queue. */
        ptrChannel->uInfo.queueInfo.messageTxQueue = Qmss_queueOpen((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                                                    Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                    &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageTxQueue  < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Initialize the MSGCOM interrupt information: */
        memset ((void *)&ptrChannel->interruptInfo, 0, sizeof(MsgCom_Interrupt));

        /* Populate the interrupt information: */
        ptrChannel->interruptInfo.sysInterrupt   = ptrChannel->cfg.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt;
        ptrChannel->interruptInfo.cpIntcId       = ptrChannel->cfg.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId;
        ptrChannel->interruptInfo.hostInterrupt  = ptrChannel->cfg.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt;

        /* Register the Interrupt using the OSAL */
        if (ptrInstanceMCB->cfg.registerIsr(channelName, queueInfo, Msgcom_channelRxHandler, (MsgCom_ChHandle)ptrChannel,
                                            &ptrChannel->interruptInfo) < 0)
            return MSGCOM_REGISTER_ISR_FAILED;

#ifdef __ARMv7
        /* TODO: UIO currently requires us to enable the interrupts after they have been processed
         * else there are no other interrupts which are generated. So we workaround this problem
         * by setting the system interrupt to be the same as the host interrupt. This will now allow
         * us to use the EnableSysInt and DisableSysInt functions. */
        ptrChannel->interruptInfo.sysInterrupt = ptrChannel->interruptInfo.hostInterrupt;
#endif

        /* Message queue was opened successfully. */
        return Msgcom_queueCreateNamedResource (channelName, ptrChannel);
    }
    else if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT)
    {
        /* Low priority accumulation interrupts not supported at the moment */
        if (ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.type == Msgcom_AccumulatedChannelType_LOW)
            return MSGCOM_LOW_PRIO_INTERRUPT_NOT_SUPPORTED;

        /* Get the queue information & PDSP Identifier which are to be used. */
        queueInfo          = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accQueue;
        ptrChannel->pdspId = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.pdspId;

        /* Open the message transmit queue */
        ptrChannel->uInfo.queueInfo.messageTxQueue = Qmss_queueOpen((Qmss_QueueType)QMSS_PARAM_NOT_SPECIFIED,
                                                                    Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo)),
                                                                    &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageTxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* Open a generic message receive queue. In the case of accumulated interrupt we will simply
         * copy packets from the accumulator list to this queue. */
        ptrChannel->uInfo.queueInfo.messageRxQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                    QMSS_PARAM_NOT_SPECIFIED,
                                                                    &isAllocated);
        if (ptrChannel->uInfo.queueInfo.messageRxQueue < 0)
            return MSGCOM_QUEUE_OPEN_FAILED;

        /* HIGH Priority Channel: Program the accumulator immediately. */
    	accCfg.channel             = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel;
	    accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
    	accCfg.queueEnMask         = 0;
	    accCfg.queMgrIndex         = Qmss_getQIDFromHandle(Qmss_getQueueHandle(queueInfo));
    	accCfg.maxPageEntries      = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries;
	    accCfg.timerLoadCount      = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount;
	    accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_NEW_PACKET;
	    accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
    	accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
	    accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

        /* We need to get a global list which has the following
         *  - PING & PONG which is 'accCfg.maxPageEntries' * 4' bytes each.
         *  - The 'accCfg.maxPageEntries' already includes the entry count. */
        accCfg.listAddress = (uint32_t)ptrInstanceMCB->cfg.malloc(Msgcom_MemAllocMode_CACHE_COHERENT,
                                                                  accCfg.maxPageEntries * 4 * 2);
        if (accCfg.listAddress == 0)
            return MSGCOM_MALLOC_FAILED;

        /* Initialize the accumulator memory */
        memset ((void *)accCfg.listAddress, 0, accCfg.maxPageEntries * 4 * 2);

        /* Program the accumulator. */
        result = Qmss_programAccumulator (ptrChannel->pdspId, &accCfg);
        if (result != QMSS_ACC_SOK)
            return MSGCOM_ACCUMULATOR_CONFIG_FAILED;

        /* Initialize the MSGCOM interrupt information: */
        memset ((void *)&ptrChannel->interruptInfo, 0, sizeof(MsgCom_Interrupt));

        /* Populate the interrupt information:
         * - DSP realm: The accumulated interrupts are not routed via the CPINTC module
         * Set the fields appropriately. */
        ptrChannel->interruptInfo.sysInterrupt   = -1;
        ptrChannel->interruptInfo.cpIntcId       = -1;
        ptrChannel->interruptInfo.hostInterrupt  = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.interruptId;

        /* Register the Interrupt using the OSAL */
        if (ptrInstanceMCB->cfg.registerIsr(channelName, queueInfo, Msgcom_channelRxHandler, (MsgCom_ChHandle)ptrChannel,
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

        /* Create and register the channel into the named resource database. */
        return Msgcom_queueCreateNamedResource (channelName, ptrChannel);
    }

    /* Invalid interrupt mode configuration. */
    return MSGCOM_INVALID_PARAM;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a VIRTUAL QUEUE channel over a previously
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
int32_t Msgcom_queueCreateVirtualReader(const char* channelName, Msgcom_channel* ptrVirtualChannel, Msgcom_channel* ptrPhyChannel)
{
    void*                   contextInfo;
    uint32_t                virtualChannelIdentifier;
    MsgCom_AppCallBack      appCallback;
    uint32_t                arg;
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Sanity Check: Both the channels should belong to the same instance since instances are self contained */
    if (ptrPhyChannel->ptrInstanceMCB != ptrVirtualChannel->ptrInstanceMCB)
        return MSGCOM_INVALID_PARAM;

    /* Get the MSGCOM instance information. */
    ptrInstanceMCB = ptrPhyChannel->ptrInstanceMCB;

    /* Inherit the runtime information from the PHYSICAL Channel. */
    memcpy ((void *)&ptrVirtualChannel->uInfo.queueInfo, (void*)&ptrPhyChannel->uInfo.queueInfo,
            sizeof(Msgcom_QueueChannelInfo));

    /* Inherit all the configuration from the PHYSICAL channel except the callback and optional argument */
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

    /* Was a virtual channel allocated? */
    if (virtualChannelIdentifier == MSG_COM_MAX_VIRTUAL_CHANNEL)
        return -1;

    /* Assign the virtual queue identifier to this channel. */
    ptrVirtualChannel->virtualChannelIdentifier = virtualChannelIdentifier;

    /* Remember the physical channel on top of which the virtual channel is created. */
    ptrVirtualChannel->phyChannelHandle = ptrPhyChannel;

    /* Initialize the software virtual channel list. */
    Msgcom_listInit (&ptrVirtualChannel->listObj);

    /* Check if the channel is blocking or not? If so we need to create a semaphore for this. */
    if (ptrVirtualChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrVirtualChannel->semHandle = ptrInstanceMCB->cfg.createSem();

    /* Create and register the channel into the named resource database. */
    return Msgcom_queueCreateNamedResource (channelName, ptrVirtualChannel);
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
 *      Pointer to the allocated MSGCOM channel block
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
int32_t Msgcom_queueCreateWriter
(
    const char*         channelName,
    Msgcom_channel*     ptrChannel,
    Name_ResourceCfg*   ptrNamedResCfg,
    int32_t*            errCode
)
{
    /* Initialize the channel information from the named resource configuration block.
     * - The Queue channels on reader creation populate the information which is required
     *   by the writer channel in a certain format. */
    ptrChannel->channelType                             = Msgcom_ChannelType_QUEUE;
    ptrChannel->uInfo.queueInfo.messageTxQueue          = ptrNamedResCfg->handle2;
    ptrChannel->isVirtualChannel                        = ptrNamedResCfg->handle3;
    ptrChannel->virtualChannelIdentifier                = ptrNamedResCfg->handle4;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to put a message over the specified queue
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
int32_t Msgcom_queuePut(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer)
{
    /* For virtual channels add the 'virtual channel' META information to the message. */
    if (ptrChannel->isVirtualChannel == 1)
        Msgcom_queueSetVirtualChannel(msgBuffer, ptrChannel->virtualChannelIdentifier);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)msgBuffer);

    /* Write the packet to the specified message transmit queue */
    Qmss_queuePushDesc (ptrChannel->uInfo.queueInfo.messageTxQueue, (void*)msgBuffer);
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
static void Msgcom_virtualChannelReceiveDecode (Msgcom_channel* ptrPhyChannel)
{
    Msgcom_InstanceMCB* ptrInstanceMCB;
    int32_t             virtualChannelId;
    Msgcom_channel*     ptrVirtualChannel;
    MsgCom_Buffer       msgBuffer;
    uint32_t*           rxCompletionList;
    uint32_t            count;
    void*               csHandle;
    int32_t             retVal;

    /* Get the instance MCB from the physical channel */
    ptrInstanceMCB = ptrPhyChannel->ptrInstanceMCB;

    /* Disable the System Interrupt if one was associated with the PHYSICAL channel. */
    Msgcom_queueDisableSysInt(ptrPhyChannel);

    /* Is the Physical Channel Direct Interrupt or in NO interrupt mode? */
    if ((ptrPhyChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrPhyChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        while (1)
        {
            /* Pop a packet from the message queue. */
            msgBuffer = Qmss_queuePop (ptrPhyChannel->uInfo.queueInfo.messageRxQueue);
            if (msgBuffer == NULL)
            {
                /* Message was NOT available in the PHYSICAL channel. We can reenable
                 * system interrupts if one was associated with the channel; this way we can
                 * now get an interrupt if a newer packet is received. */
                Msgcom_queueEnableSysInt(ptrPhyChannel);
                return;
            }

            /* Message was available. Get the message. */
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(msgBuffer));

            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
             * takes over ownership of the packet.
             ******************************************************************************/
            Pktlib_getOwnership(ptrPhyChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt*)msgBuffer);

            /* Get the handle to the virtual channel. */
            virtualChannelId = Msgcom_queueGetVirtualChannel(msgBuffer);
            ptrVirtualChannel = ptrPhyChannel->virtualChannel[virtualChannelId];
            if (ptrVirtualChannel == NULL)
                while (1);

            /* Add the received message buffer to the virtual channel. If there is no space in the
             * virtual channel receive buffers we will drop the packet. */
            if (Msgcom_queueAddBuffer (ptrVirtualChannel, msgBuffer) < 0)
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
        if (msgBuffer == NULL)
            while (1);

        /* Convert the descriptor which we read from the completion list to a virtual address*/
        msgBuffer = Osal_qmssConvertDescPhyToVirt(0, msgBuffer);

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
         * takes over ownership of the packet.
         ******************************************************************************/
        Pktlib_getOwnership(ptrPhyChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt*)msgBuffer);

        /* Get the handle to the virtual channel. */
        virtualChannelId = Msgcom_queueGetVirtualChannel(msgBuffer);
        ptrVirtualChannel = ptrPhyChannel->virtualChannel[virtualChannelId];
        if (ptrVirtualChannel == NULL)
        {
            while (1)
            {
            }
        }

		/* Clear the accumulator entry. */
		rxCompletionList[ptrPhyChannel->messageIdx + 1] = 0;

        /* Add the received message buffer to the virtual channel. If there is no space in the
         * virtual channel receive buffers we will drop the packet. But continue to handle the
         * rest of the packets in the physical channel. This is required because we might have
         * multiple virtual channels on a single physical channel and we cannot allow one *bad*
         * virtual channel to stop operation on the rest of the channels. */
        retVal = Msgcom_queueAddBuffer (ptrVirtualChannel, msgBuffer);

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
    			Qmss_ackInterruptByIntd(0, ptrPhyChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel, 1);
	    		Qmss_setEoiVectorByIntd (0, Qmss_IntdInterruptType_HIGH, ptrPhyChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel);
            }
            else
            {
    			Qmss_ackInterruptByIntd(1, ptrPhyChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel, 1);
	    		Qmss_setEoiVectorByIntd(1, Qmss_IntdInterruptType_HIGH, ptrPhyChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel);
            }

            /* Reenable the interrupts if they had been disabled. */
            Msgcom_queueEnableSysInt(ptrPhyChannel);
		}

        /* It is possible that by the time control comes here the packet has been dropped since the virtual
         * channel receive buffer is full; so if this is the case there is no need to post the semaphore
         * and invoke the callback function */
        if (retVal < 0)
            continue;

        /* Message was successfully added to the virtual channel receive buffer. If the virtual channel
         * was blocking we post the semaphore. */
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
 *      The function is used to get a message over the specified queue
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
int32_t Msgcom_queueGet(Msgcom_channel* ptrChannel, MsgCom_Buffer** msgBuffer)
{
    uint32_t*               rxCompletionList;
    uint32_t                count;
    void*                   csHandle;
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Check if the channel is a virtual channel or not? */
    if (ptrChannel->isVirtualChannel == 1)
    {
        while (1)
        {
            /* Virtual channel: Get a pending packet from the virtual channel if one was already present */
            *msgBuffer = Msgcom_queueRemoveBuffer(ptrChannel);
            if (*msgBuffer != NULL)
                return 0;

            /* Message was not available in the virtual channel. There could be messages which
             * are pending to be serviced in the physical channel. These messages need to be decoded
             * and placed into the appropriate virtual channel buffers. */
            Msgcom_virtualChannelReceiveDecode(ptrChannel->phyChannelHandle);

            /* Now lets try again and see if there is any pending messages on the specific virtual channel */
            *msgBuffer = Msgcom_queueRemoveBuffer(ptrChannel);
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
    if ((ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT) ||
        (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT))
    {
        while (1)
        {
            /* Pop a packet from the message queue. */
            *msgBuffer = Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageRxQueue);
            if (*msgBuffer != NULL)
            {
                /* Message was available. We return the received message. */
                *msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(*msgBuffer));

                /******************************************************************************
                 * NOTE: There is an ownership change here from the CPDMA to the DSP. The DSP
                 * takes over ownership of the packet.
                 ******************************************************************************/
                Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt*)(*msgBuffer));
                break;
            }
            else
            {
                /* Message was NOT available: We should reenable the system interrupts so that we can get
                 * notified once we get 'newer' packets. */
                Msgcom_queueEnableSysInt(ptrChannel);


                //<fzm>
                //https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/4891.aspx
                // Check if any descriptor arrived between last check and interrupt enable
                *msgBuffer = Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageRxQueue);
                if (*msgBuffer != NULL)
                {
                    *msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)(*msgBuffer));
                    Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt *)(*msgBuffer));
                    Msgcom_queueDisableSysInt(ptrChannel);
                    break;
                }
                //</fzm>

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
        Pktlib_getOwnership(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (Ti_Pkt*)(*msgBuffer));

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
    			Qmss_ackInterruptByIntd(0, ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel, 1);
	    		Qmss_setEoiVectorByIntd(0, Qmss_IntdInterruptType_HIGH, ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel);
            }
            else
            {
    			Qmss_ackInterruptByIntd(1, ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel, 1);
	    		Qmss_setEoiVectorByIntd(1, Qmss_IntdInterruptType_HIGH, ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel);
            }

            /* Reenable the interrupts if they had been disabled. */
            Msgcom_queueEnableSysInt(ptrChannel);
        }
        /* Get operation was successful. */
        return 0;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to handle the Queue Channel ISR.
 *
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *  	Not Applicable
 */
void Msgcom_queueRxHandler(Msgcom_channel* ptrChannel)
{
    Msgcom_InstanceMCB*     ptrInstanceMCB;

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* The Channel receive Handler is supposed to execute only for PHYSICAL channels. */
    if (ptrChannel->isVirtualChannel == 1)
        return;

    /* Disable the System Interrupt if one was associated with the PHYSICAL channel. We have already
     * received a notification that there was data present in the queue. We dont need any more
     * system interrupts to indicate this again. */
    Msgcom_queueDisableSysInt(ptrChannel);

    /* Check if this is for NON Interrupt support. */
    if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_NO_INTERRUPT)
    {
        /* YES. The application can periodically call the Message Communicator Receive Handler
         * to check if there is data on the message Rx Queue or not? This mode can be used to
         * support BLOCKING channels in the NON interrupt mode. Check if there are packets
         * available on the message receive queue or not? */
        if (Qmss_getQueueEntryCount(ptrChannel->uInfo.queueInfo.messageRxQueue) != 0)
        {
            /* YES. Message was available. Determine if the channel was virtualized. */
            if (ptrChannel->virtualChannelIdentifier != -1)
            {
                /* Channel was virtualized. We need to pick up the data from the PHYSICAL channel
                 * and decode it to wake up the VIRTUAL channels. */
                Msgcom_virtualChannelReceiveDecode(ptrChannel);
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
    if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Interrupt Mode: Is the channel virtualized? */
        if (ptrChannel->virtualChannelIdentifier != -1)
        {
            /* Channel was virtualized. We need to pick up the data from the PHYSICAL channel
             * and decode it to wake up the VIRTUAL channels. */
            Msgcom_virtualChannelReceiveDecode(ptrChannel);
        }
        else
        {
            /* Channel was NOT virtualized */
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
            Msgcom_virtualChannelReceiveDecode(ptrChannel);
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
 *      The function is used to delete a previously opened queue channel.
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
int32_t Msgcom_queueDelete(Msgcom_channel* ptrChannel, Msgcom_freePkt freePkt)
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

    /* Get the MSGCOM Instance information */
    ptrInstanceMCB = (Msgcom_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Determine the type of channel being deleted? */
    if (ptrChannel->isReader == 0)
    {
        /* Writer channel: These channels are responsible for ensuring that the transmit queue is
         * empty and cleaning up any packets which have not been sent out. */
        msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageTxQueue));
        while (msgBuffer != NULL)
        {
            freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
            msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageTxQueue));
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
                /* YES. Update the physical channel record to delete the virtualized channel */
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
            msgBuffer = Msgcom_queueRemoveBuffer(ptrChannel);
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
    if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT)
    {
        /* Accumulator Channel: Initialize the accumulator configuration. */
        memset(&accCfg, 0, sizeof (Qmss_AccCmdCfg));

        /* Populate the accumulator configuration. */
        accCfg.channel = ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.accChannel;
        accCfg.command = Qmss_AccCmd_DISABLE_CHANNEL;

        /* Disable the accumulator. */
        result = Qmss_programAccumulator(ptrChannel->pdspId, &accCfg);
        if (result != QMSS_ACC_SOK)
            return MSGCOM_ACCUMULATOR_CONFIG_FAILED;

        /* Get the queue number */
        queueInfo = Qmss_getQueueNumber(ptrChannel->uInfo.queueInfo.messageTxQueue);

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
        ptrInstanceMCB->cfg.free(Msgcom_MemAllocMode_CACHE_COHERENT,
                                 (void*)ptrChannel->pingAddress,
                                 ptrChannel->cfg.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries * 4 * 2);
    }
    else if (ptrChannel->cfg.u.queueCfg.interruptMode == Msgcom_QueueInterruptMode_DIRECT_INTERRUPT)
    {
        /* Direct Interrupt Channel: Get the queue information for the message receive queue. */
        queueInfo = Qmss_getQueueNumber(ptrChannel->uInfo.queueInfo.messageRxQueue);

        /* Deregister the ISR from the OSAL. */
        if (ptrInstanceMCB->cfg.deregisterIsr(ptrChannel->channelName, queueInfo, &ptrChannel->interruptInfo) < 0)
            return MSGCOM_DEREGISTER_ISR_FAILED;
    }

    /* Empty the receive queue. */
    msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageRxQueue));
    while (msgBuffer != NULL)
    {
        freePkt(ptrChannel->ptrInstanceMCB->cfg.pktlibInstHandle, (MsgCom_ChHandle)ptrChannel, msgBuffer);
        msgBuffer = (MsgCom_Buffer*)QMSS_DESC_PTR((uint32_t)Qmss_queuePop (ptrChannel->uInfo.queueInfo.messageRxQueue));
    }

    /* Close the receive & transmit queue. */
    Qmss_queueClose(ptrChannel->uInfo.queueInfo.messageRxQueue);
    Qmss_queueClose(ptrChannel->uInfo.queueInfo.messageTxQueue);

    /* Delete the semaphore we got for this channel's blocking implementation */
    if (ptrChannel->cfg.mode == Msgcom_ChannelMode_BLOCKING)
        ptrInstanceMCB->cfg.deleteSem (ptrChannel->semHandle);

    return 0;
}
