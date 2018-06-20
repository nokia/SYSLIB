/**
 *   @file  msgcom_internal.h
 *
 *   @brief
 *      Header file for the Message Communicator. The file exposes the data structures
 *      and exported API which are available for use by applications.
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

#ifndef __MSG_COM_INTERNAL_H__
#define __MSG_COM_INTERNAL_H__

/* MCSDK Include files */
#include <ti/csl/csl_cache.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* SYSLIB Include files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/msgcom/include/listlib.h>

/** @addtogroup MSG_COM_INTERNAL_SYMBOL
 @{ */

/**
 * @brief   Maximum number of CPDMA blocks which can be supported by MSGCOM
 */
#define MSGCOM_MAX_CPDMA_BLOCKS             2

/**
@}
*/

/** @addtogroup MSG_COM_INTERNAL_ENUM
 @{ */

/**
 * @brief
 *  Enumeration Type which describes the channel property.
 *
 * @details
 *  Channels can be created to be reader or writer.
 */
typedef enum Msgcom_ChannelDesc
{
    /**
     * @brief   Channel is reader. This channel will be used to recieve data
     * from a writer channel.
     */
    Msgcom_ChannelDesc_READER       = 0x1,

    /**
     * @brief   Channel is writer. This channel will be used to send data to a
     * reader channel.
     */
    Msgcom_ChannelDesc_WRITER       = 0x2
}Msgcom_ChannelDesc;

/**
@}
*/

/** @addtogroup MSG_COM_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Queue Channel Information.
 *
 * @details
 *  The data structure is used to store the channel information pertinent
 *  to Queue Channels.
 */
typedef struct Msgcom_QueueChannelInfo
{
    /**
     * @brief   This is the message receive queue on which messages are received.
     */
    Qmss_QueueHnd       messageRxQueue;

    /**
     * @brief   This is the message transmit queue on which messages are transmitted.
     */
    Qmss_QueueHnd       messageTxQueue;
}Msgcom_QueueChannelInfo;

/**
 * @brief
 *  Queue DMA Channel Information.
 *
 * @details
 *  The data structure is used to store the channel information pertinent
 *  to Queue Channels.
 */
typedef struct Msgcom_QueueDMAChannelInfo
{
    /**
     * @brief   This is the transmit queue used by the writer to push messages.
     * This has to be an infrastructure transmit queue.
     */
    Qmss_QueueHnd       messageTxQueue;

    /**
     * @brief   This is the receive queue used by the reader to read messages.
     */
    Qmss_QueueHnd       messageRxQueue;

    /**
     * @brief   This is a temporary internal queue which holds the HIGH Priority
     * queue handle in the case of ACCUMULATED interrupts. This is NOT used in
     * the other modes.
     */
    Qmss_QueueHnd       messageTmpQueue;

    /**
     * @brief   This is the CPPI Transmit channel handle associated with the message
     * communication channel.
     */
    Cppi_ChHnd          txChannelHnd;

    /**
     * @brief   This is the CPPI Receive channel handle associated with the message
     * communication channel.
     */
    Cppi_ChHnd          rxChannelHnd;

    /**
     * @brief   This is the CPPI Flow handle associated with the message
     * communication channel.
     */
    Cppi_FlowHnd        flowChannelHnd;

    /**
     * @brief   Tag Information which needs to be configured in each descriptor
     * before it can be transmitted.
     */
    Cppi_DescTag        tagInfo;
}Msgcom_QueueDMAChannelInfo;

/**
 * @brief
 *  Instance MCB
 *
 * @details
 *  The structure is used to describe the instance MCB
 */
typedef struct Msgcom_InstanceMCB
{
    /**
     * @brief   Instance configuration which is used to create the MSGCOM
     * instance.
     */
    Msgcom_InstCfg       cfg;

    /**
     * @brief   Number of CPDMA instances which are present in the device
     */
    int32_t             numCPDMAInstances;

    /**
     * @brief   Dummy Allocation Mode: If the device supports multiple instances
     * the mode can be used to determine how the allocations should be done.
     * 1) Dummy Mode: Allocate all the channels from Instance0 and then move to
     *    Instance1.
     * 2) Load Balanced Mode: Use Instance0, then Instance1 then Instance0 etc.
     */
    int32_t             dummyAllocationMode;

    /**
     * @brief   Load balancing per MSGCOM instance. This is the CPDMA instance
     * index which is being used. If there are multiple CPDMA instances
     * MSGCOM will load balance between these instances.
     */
    uint32_t            cpdmaInstanceIndex;

    /**
     * @brief   Handle to the QMSS Queue Type blocks which can be used by
     * MSGCOM for the specific device
     */
    Qmss_QueueType      queueType[MSGCOM_MAX_CPDMA_BLOCKS];

    /**
     * @brief   Queue Base which can be used by MSGCOM for the specific device
     */
    uint32_t            queueBase[MSGCOM_MAX_CPDMA_BLOCKS];

    /**
     * @brief   Handle to the CPPI QMSS Infrastructure block which can be used by
     * MSGCOM for the specific device
     */
    Cppi_Handle         cppiHnd[MSGCOM_MAX_CPDMA_BLOCKS];

#if defined(__ARM_ARCH_7A__)
    Qmss_QueueHnd       qmssBarrierQMsmc;
    Qmss_QueueHnd       qmssBarrierQDdr;
#endif /* #if defined(__ARM_ARCH_7A__) */
}Msgcom_InstanceMCB;

/**
 * @brief
 *  Channel Structure.
 *
 * @details
 *  The data structure is used to describe a channel.
 */
typedef struct Msgcom_channel
{
    /**
     * @brief   Pointer to the MSGCOM instance to which the channel belongs.
     */
    Msgcom_InstanceMCB* ptrInstanceMCB;

    /**
     * @brief   Name of the channel
     */
    char                channelName[MSG_COM_MAX_CHANNEL_LEN + 1];

    /**
     * @brief   This is the channel type for which the configuration is valid.
     */
    uint32_t            channelType;

    /**
     * @brief   Reader or writer channel
     */
    uint32_t            isReader;

    /**
     * @brief   Physical or Virtual Channel.
     */
    uint32_t            isVirtualChannel;

    /**
     * @brief   Channel configuration supplied by the application.
     */
    Msgcom_ChannelCfg   cfg;

    /**
     * @brief   PING List Address
     */
    uint32_t            pingAddress;

    /**
     * @brief   PONG List Address
     */
    uint32_t            pongAddress;

    /**
     * @brief   OS semaphore handle used to support blocking/non-blocking
     * mode.
     */
    void*               semHandle;

    /**
     * @brief   Virtual Channel Identifier
     */
    int32_t             virtualChannelIdentifier;

    /**
     * @brief   Virtual Channel Database. Each physical channel can have a number of
     * virtual channels installed executing on top of it.
     */
    MsgCom_ChHandle     virtualChannel[MSG_COM_MAX_VIRTUAL_CHANNEL];

    /**
     * @brief   This is applicable only to the virtual channel and is the physical
     * channel on top of which the physical channel is created.
     */
    MsgCom_ChHandle     phyChannelHandle;

    /**
     * @brief   MSGCOM channel interrupt information which is populated only if the
     * channel is created in NON Interrupt mode.
     */
    MsgCom_Interrupt    interruptInfo;

    /**
     * @brief   Accumulator PDSP Identifier associated
     */
    Qmss_PdspId         pdspId;

    /**
     * @brief   Accumulator specific runtime information which indicates which
     * list is to be accessed.
     */
    uint32_t            usePing;

    /**
     * @brief   Index to the message to be processed from the accumulator list.
     */
    int32_t             messageIdx;

    /**
     * @brief   Virtual channel linked list
     */
    Msgcom_ListObj      listObj;

    /**
     * @brief   Counter which keeps track of the number of interrupts which are
     * detected.
     */
    volatile uint32_t   interruptDetected;

    /**
     * @brief   Channel information is dependent on the type of channel.
     */
    union               uInfo
    {
        /**
         * @brief   This is the Queue Based channel configuration information.
         */
        Msgcom_QueueChannelInfo         queueInfo;

        /**
         * @brief   This is the Queue-DMA Based channel configuration information.
         */
        Msgcom_QueueDMAChannelInfo      queueDMAInfo;
    }uInfo;
}Msgcom_channel;

/**
 * @brief
 *  Channel Interface data structure
 *
 * @details
 *  This data structure allows different channels to be plugged into the message
 *  communicator module.
 */
typedef struct Msgcom_ChannelInterface
{
    /**
     * @brief   Channel Create reader interface: The function is invoked to create a channel.
     */
    int32_t(*createReader)(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_ChannelCfg* ptrChannelCfg);

    /**
     * @brief   Channel Create virtual reader interface: The function is invoked to create a virtual channel.
     */
    int32_t(*createVirtualReader)(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_channel* ptrVirtualChannel);

    /**
     * @brief   Channel Create virtual writer interface: The function is invoked to create a virtual channel.
     */
    int32_t (*createWriter)(const char* channelName, Msgcom_channel* ptrChannel, Name_ResourceCfg* ptrNamedResCfg, int32_t* errCode);

    /**
     * @brief   Channel Put: This is used to put/send the specified message on the
     * channel.
     */
    int32_t(*put)(Msgcom_channel* ptrChannel, MsgCom_Buffer*  msgBuffer);

    /**
     * @brief   Channel Get: This is used to get/receive a specified message on the
     * channel.
     */
    int32_t(*get)(Msgcom_channel* ptrChannel, MsgCom_Buffer** msgBuffer);

    /**
     * @brief   Channel Rx Handler: This is used to handle the receive/get event.
     * Channels might be tied to interrupts and this is used to handle the specific
     * event.
     */
    void(*rxHandler)(Msgcom_channel* ptrChannel);

    /**
     * @brief   Channel Delete: This is used to delete the specific channel.
     */
    int32_t(*delete)(Msgcom_channel* ptrChannel, Msgcom_freePkt freePkt);

    /**
     * @brief   Channel Get Internal Message Queue Info: This is used to get the
     * internal message queue information.
     */
    int32_t(*getInternalMsgQueueInfo)(Msgcom_channel* ptrChannel);
}Msgcom_ChannelInterface;

/**
@}
*/

/**********************************************************************
 **************************** Internal API ****************************
 **********************************************************************/

/** @addtogroup MSG_COM_INTERNAL_FUNCTION
 @{ */

/**
 *  @b Description
 *  @n
 *      Utility function which is used to get the list node from the MSGCOM
 *      buffer.
 *
 *  @param[in]  ptrBuffer
 *      Pointer to the MSGCOM buffer
 *
 *  \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Maps the MSGCOM Buffer to the internal list node.
 */
static inline Msgcom_ListNode* Msgcom_getListNode(MsgCom_Buffer* ptrBuffer)
{
    return (Msgcom_ListNode*)((uint8_t*)ptrBuffer + offsetof (Cppi_HostDesc, softwareInfo0));
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to get the MSGCOM buffer from the list node
 *
 *  @param[in]  ptrListNode
 *      Pointer to the MSGCOM list node
 *
 *  \ingroup MSG_COM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Maps the internal list node to the MSGCOM Buffer
 */
static inline MsgCom_Buffer* Msgcom_getBuffer(Msgcom_ListNode* ptrListNode)
{
    return (MsgCom_Buffer*)((uint8_t*)ptrListNode - offsetof (Cppi_HostDesc, softwareInfo0));
}

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern int32_t Msgcom_queueCreateReader(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_ChannelCfg* ptrChannelCfg);
extern int32_t Msgcom_queueCreateVirtualReader(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_channel* ptrPhyChannel);
extern int32_t Msgcom_queueCreateWriter(const char* channelName, Msgcom_channel* ptrChannel, Name_ResourceCfg* ptrNamedResCfg, int32_t* errCode);
extern int32_t Msgcom_queuePut(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer);
extern int32_t Msgcom_queueGet(Msgcom_channel* ptrChannel, MsgCom_Buffer** msgBuffer);
extern void    Msgcom_queueRxHandler(Msgcom_channel* ptrChannel);
extern int32_t Msgcom_queueDelete(Msgcom_channel* ptrChannel, Msgcom_freePkt freePkt);
extern int32_t Msgcom_queueGetInternalMsgQueueInfo(Msgcom_channel* ptrChannel);

extern int32_t Msgcom_queueDMACreateReader(const char* channelName, Msgcom_channel* ptrChannel, Msgcom_ChannelCfg* ptrChannelCfg);
extern int32_t Msgcom_queueDMACreateVirtualReader(const char* channelName, Msgcom_channel* ptrVirtualChannel, Msgcom_channel* ptrPhyChannel);
extern int32_t Msgcom_queueDMACreateWriter(const char* channelName, Msgcom_channel* ptrChannel, Name_ResourceCfg* ptrNamedResCfg, int32_t* errCode);
extern int32_t Msgcom_queueDMAPut(Msgcom_channel* ptrChannel, MsgCom_Buffer* msgBuffer);
extern int32_t Msgcom_queueDMAGet(Msgcom_channel* ptrChannel, MsgCom_Buffer** msgBuffer);
extern void    Msgcom_queueDMARxHandler(Msgcom_channel* ptrChannel);
extern int32_t Msgcom_queueDMADelete(Msgcom_channel* ptrChannel, Msgcom_freePkt freePkt);
extern int32_t Msgcom_queueDMAGetInternalMsgQueueInfo(Msgcom_channel* ptrChannel);

#endif /* __MSG_COM_INTERNAL_H__ */
