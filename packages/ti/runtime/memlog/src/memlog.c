/**
 *   @file  memlog.c
 *
 *   @brief
 *      The file implements the resource management library
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

#ifdef __ARMv7
#include <ti/runtime/hplib/hplib.h>

#define System_printf   printf
#define CACHE_L2_LINESIZE 128
#else
#include <xdc/runtime/System.h>
/* CSL Include Files. */
#include <ti/csl/csl_cacheAux.h>
#endif

/* SYSLIB include files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/memlog/memlog.h>
#include <ti/runtime/memlog/include/listlib.h>

/**********************************************************************
 ************************ Internal Definitions ************************
 **********************************************************************/

/**********************************************************************
 ************************ Internal Structures *************************
 **********************************************************************/

/**
 * @brief
 *  Memory logging channel statistics
 *
 * @details
 *  The structure holds the statistics for a memory logging channel.
 */
typedef struct Memlog_channelStats
{
    /**
     * @brief   counter for total logging messages.
     */
    uint32_t            totalMsg;

    /**
     * @brief   counter for successfuuly sent logging messages.
     */
    uint32_t            sentMsg;

    /**
     * @brief   counter for dropped logging messages because logging is stopped.
     */
    uint32_t            droppedMsg;

    /**
     * @brief   counter for dropped logging messages because memory logging error.
     */
    uint32_t            sentFailedMsg;
}Memlog_channelStats;


/**
 * @brief
 *  Instance MCB
 *
 * @details
 *  The structure is used to describe the instance MCB
 */
typedef struct Memlog_InstanceMCB
{
    /**
     * @brief   Instance configuration which is used to create the MEMLOG
     * instance.
     */
    Memlog_InstCfg      cfg;
}Memlog_InstanceMCB;

/**
 * @brief
 *  Memory logging log buffer info.
 *
 * @details
 *  The structure describes the definition of Memory logging log buffer info used for relinking
 */
typedef struct Memlog_logBufferInfo
{
    /**
     * @brief   Links to other buffers.
     */
    Memlog_ListNode     links;

    /**
     * @brief   Log buffer address.
     */
    uint32_t            bufAddr;
}Memlog_logBufferInfo;

/**
 * @brief
 *  Memory logging Channel.
 *
 * @details
 *  The structure describes the memory logging channel
 */
typedef struct Memlog_channel
{
    /**
     * @brief   Memory logging channel configuration
     */
    Memlog_ChannelCfg           cfg;

    /**
     * @brief   Handle to the Memory Logging module instance
     */
    Memlog_InstanceMCB*         ptrInstanceMCB;

    /**
     * @brief   Memory logging Heap handle used for infra-DMA to save logs in Memory
     */
    Pktlib_HeapHandle           msgcomHeapHandle;

    /**
     * @brief   MsgCom reader channel handle used for relinking buffers
     */
    MsgCom_ChHandle             msgcomReaderChanHandle;

    /**
     * @brief   MsgCom writer channel handle used for saving Logs in memory
     */
    MsgCom_ChHandle             msgcomWriterChanHandle;

    /**
     * @brief   Number of buffers available in the provided memory block for memory logging.
     */
    uint32_t                    numLogBuffers;

    /**
     * @brief   Head of the List of memory logging buffers.
     */
    Memlog_logBufferInfo*       ptrLogBufListHead;

    /**
     * @brief   Tail of the List of memory logging buffers.
     */
    Memlog_logBufferInfo*       ptrLogBufListTail;

    /**
     * @brief   Number of buffers in the list.
     */
    uint32_t                    numBufferInList;

    /**
     * @brief   Number of pending packet before entering the STOPPING state
     */
    uint32_t                    numPendingPkt;

    /**
     * @brief   General purpose Queue used for Sync between producer and producer controller.
     */
    Qmss_QueueHnd               syncQueue;

    /**
     * @brief   General purpose Queue used for Sync between producer and producer controller.
     */
    Ti_Pkt*                     syncPkt;

    /**
     * @brief   Memory logging channel stats.
     */
    Memlog_channelStats          stats;
}Memlog_channel;

/**
 * @brief
 *  Memory logging  Controller created on ARM side.
 *
 * @details
 *  The structure describes the memory logging controller used to control
 *  a memory logging channel. It contains logging channel information used
 *  to retrive logs from memory.
 */
typedef struct MemLog_Controller
{
    /**
     * @brief   Name of the memory logging channel.
     */
    char                 name[MEMLOG_MAX_CHAR];

    /**
     * @brief   MEMLOG Instance handle associated with the controller. Controller exists
     * only in the context of the MEMLOG instance.
     */
    Memlog_InstHandle    memlogInstHandle;

    /**
     * @brief   Memory logging channel information saved in Named database.
     */
    MemLog_LoggerInfo    loggerInfo;
}MemLog_Controller;

/**
 * @brief
 *  Memory logging MCB
 *
 * @details
 *  Memory logging  master control block.
 */
typedef struct Memlog_MCB
{
    /**
     * @brief  System configuration.
     */
    Memlog_InstCfg              cfg;
}Memlog_MCB;

/**********************************************************************
 ************************ Extern Functions ****************************
 **********************************************************************/


/**********************************************************************
 ******************** Memory Logging Functions ************************
 **********************************************************************/
/**
 *  @b Description
 *  @n
 *      This init function is used to create a general purpose Queue used for
 *  syncing between two entities. There will be only sync packet in the queue.
 *
 *  @param[in]  syncPkt
 *      Original sync packet to put into the sync Queue
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Queue handle if the queue is setup successfully
 *  @retval
 *      0            if the error happened
 */
inline static Qmss_QueueHnd Memlog_semInit(Ti_Pkt* syncPkt, int32_t* errCode)
{
    Qmss_QueueHnd       syncQueue;
    uint8_t             isAllocated;

    /* Open the memlogging sync queue per producer to keep producer and producer controller synced. */
    syncQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                               QMSS_PARAM_NOT_SPECIFIED,
                               &isAllocated);
    if (syncQueue < 0)
    {
        *errCode = MEMLOG_ENOMEM;
        return (Qmss_QueueHnd)-1;
    }

    /* Push the sync pkt to sync Q */
    Qmss_queuePushDescSizeRaw (syncQueue, (void*)syncPkt, 128);

    return (Qmss_QueueHnd)syncQueue;
}

/**
 *  @b Description
 *  @n
 *      This deinit function is used to close a general purpose Queue used for
 *  syncing between two entities. There will be only sync packet in the queue.
 *
 *  @param[in]  qHnd
 *      QMSS Queue handle that used as semaphore
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Queue handle if the queue is setup successfully
 *  @retval
 *      0            if the error happened
 */
inline static Ti_Pkt* Memlog_semDeinit(Qmss_QueueHnd qHnd)
{
    Ti_Pkt*         syncPkt = NULL;

    /* Pop the packet from the Q and return it */
    syncPkt = Qmss_queuePopRaw (qHnd);

    /* Close the memlogging sync queue  */
    Qmss_queueClose(qHnd);

    return syncPkt;
}

#ifdef __ARMv7
/**
 *  @b Description
 *  @n
 *      This function acquires a semaphore for direct access.
 *
 *  @param[in]  qHnd
 *      Queue Handle to acquire the sync packet. Packet is always used as physical address
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Handle to the sync packet (Physical address)
 */
inline static Ti_Pkt* Memlog_semAcquire(Qmss_QueueHnd qHnd)
{
    Ti_Pkt*         syncPkt;

    /* Pop from the queue */
    syncPkt = Qmss_queuePopRaw (qHnd);
    return syncPkt;
}

/**
 *  @b Description
 *  @n
 *      This function releases a semaphore which had been acquired.
 *
 *  @param[in]  qHnd
 *      Queue Handle to release the sync packet.
 *  @param[in]  syncPkt
 *      Physical address of the sync packet. This packet should be the acuired
 *  when calling Memlog_semAcquire.
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval     None
 */
inline static void Memlog_semRelease (Qmss_QueueHnd qHnd, Ti_Pkt* syncPkt)
{
    /* Push the sync packet to the sysc Q */
    Qmss_queuePushDescSizeRaw (qHnd, (void*)syncPkt, 128);
}
#endif

/**
 *  @b Description
 *  @n
 *     This function checks if the specified semaphore is acquired or not?
 *
 *  @param[in]  qHnd
 *      Queue handle to be checked to see if it is empty.
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Semaphore is acuquired(Queue is empty)      -   0
 *  @retval
 *      Semaphre is free(Queue is not empty)        -   1
 */
inline static uint8_t  Memlog_semIsFree(Qmss_QueueHnd qHnd)
{
    return (Qmss_getQueueEntryCount(qHnd) > 0 ? 1 :0);
}

/**
 *  @b Description
 *  @n
 *      Registered callback function which is invoked to cleanup any pending
 *      packets in the MSGCOM channel when the channels are being deleted
 *
 *  @param[in]   pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]   chHandle
 *      Channel handle being deleted
 *  @param[in]   msgBuffer
 *      Message which is pending and needs to be cleaned up
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Memlog_freePkt(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    Pktlib_freePacket (pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate memory logging buffers from a pre-allocated memory block
 *
 *  @param[in]  size
 *      Size of the buffer to be allocated, size should be cache size aligned
 *  @param[in]  arg
 *      Argument to the malloc function. This argument points to the memory logging structure.
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Address of the allocated buffer
 */
static uint8_t* Memlog_bufferMalloc(uint32_t size,  uint32_t arg)
{
    Memlog_channel*         ptrMemLogChan;
    Memlog_logBufferInfo*   ptrLogBuffer;
    Memlog_InstanceMCB*     ptrInstanceMCB;
    uint32_t                bufferAddr;
    void*                   csHandle;

    /* Sanity Check */
    if( (size == 0) || (arg == 0) )
        return NULL;

    /* Get Memory logging info from the argument */
    ptrMemLogChan = (Memlog_channel*)arg;

    /* Get MEMLOG instance MCB */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemLogChan->cfg.memlogInstHandle;

    csHandle = ptrInstanceMCB->cfg.enterCS();
    /* Get a free buffer from list */
    ptrLogBuffer = (Memlog_logBufferInfo*)Memlog_listRemove ((Memlog_ListNode**)&ptrMemLogChan->ptrLogBufListHead);
    if(ptrMemLogChan->ptrLogBufListHead == NULL)
        ptrMemLogChan->ptrLogBufListTail = NULL;

    if(ptrLogBuffer != NULL )
        ptrMemLogChan->numBufferInList--;

    ptrInstanceMCB->cfg.exitCS(csHandle);

    if(ptrLogBuffer == NULL )
        return NULL;

    /* Get the address and free the logBuffer */
    bufferAddr = ptrLogBuffer->bufAddr;
    ptrInstanceMCB->cfg.free(Memlog_MemAllocMode_INTERNAL, ptrLogBuffer, sizeof(Memlog_logBufferInfo));

    /* Return buffer address */
    return (uint8_t*)(bufferAddr);
}

/**
 *  @b Description
 *  @n
 *      The function is used to free memory logging buffers
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be freed.
 *  @param[in]  size
 *      Size of the buffer to be freed, size should be cache size aligned
 *  @param[in]  arg
 *      Argument for the free function. This argument is the pointer to memory logging structure
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
static void Memlog_bufferFree(uint8_t* ptr, uint32_t size,  uint32_t arg)
{
    Memlog_channel*         ptrMemLogChan;
    Memlog_logBufferInfo*   ptrLogBuffer;
    Memlog_InstanceMCB*     ptrInstanceMCB;
    void*                   csHandle;

    /* Sanity Check */
    if(ptr == NULL || size == 0 || arg == 0)
        return;

    /* Get Memory logging info from the argument */
    ptrMemLogChan = (Memlog_channel*)arg;

    /* Get MEMLOG instance MCB */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemLogChan->cfg.memlogInstHandle;

    ptrLogBuffer = ptrInstanceMCB->cfg.malloc(Memlog_MemAllocMode_INTERNAL, sizeof(Memlog_logBufferInfo));
    if (ptrLogBuffer != NULL)
    {
        ptrLogBuffer->bufAddr = (uint32_t)ptr;

        csHandle = ptrInstanceMCB->cfg.enterCS();

        /* Add to log buffer list */
        Memlog_listAddTail ((Memlog_ListNode**)&ptrMemLogChan->ptrLogBufListTail, (Memlog_ListNode*)ptrLogBuffer);

        /* A node is added to an empty list, update head also */
        if((ptrMemLogChan->ptrLogBufListHead == NULL) && (ptrMemLogChan->ptrLogBufListTail != NULL))
            ptrMemLogChan->ptrLogBufListHead = ptrMemLogChan->ptrLogBufListTail;

        ptrMemLogChan->numBufferInList++;

        ptrInstanceMCB->cfg.exitCS(csHandle);
    }

    return ;

}

/**
 *  @b Description
 *  @n
 *      The function is used to collect descriptors from MsgCom channel.
 *
 *  @param[in]  ptrMemLogChan
 *      Pointer to memory logging channel.
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Memlog_garbageCollection(Memlog_channel* ptrMemLogChan)
{
    Ti_Pkt*                pMessage;
    uint8_t*               bufAddr;
    Memlog_InstanceMCB*    ptrInstanceMCB;
    uint32_t               bufferLen;

    /* Get the MEMLOG instance MCB: . */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemLogChan->cfg.memlogInstHandle;
    if(ptrInstanceMCB == NULL)
        return -1;

    do {
        /* get a message from ReceiveQ */
        if (Msgcom_getMessage(ptrMemLogChan->msgcomReaderChanHandle, (MsgCom_Buffer**)&pMessage) < 0)
        {
            return -1;
        }
        if( pMessage == NULL )
        {
            break;
        }
        /* Free the current log buffer */
        Pktlib_getDataBuffer((Ti_Pkt *)pMessage, (uint8_t **)&bufAddr, &bufferLen);

        Memlog_bufferFree(bufAddr, ptrMemLogChan->cfg.bufferSize,(uint32_t)ptrMemLogChan);

        /* Allocate a new log buffer */
        bufAddr = Memlog_bufferMalloc(ptrMemLogChan->cfg.bufferSize,(uint32_t)ptrMemLogChan);
        /* Link the current descriptor to the new log buffer */
        Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)pMessage, bufAddr, ptrMemLogChan->cfg.bufferSize);

        /* Write back the packet */
        Pktlib_releaseOwnership(ptrInstanceMCB->cfg.pktlibInstHandle, pMessage);

        /* Push the packet into the corresponding free queue */
        Qmss_queuePushDescSize (Pktlib_getInternalHeapQueue(Pktlib_getPktHeap(pMessage)), (void*)pMessage, 0x80);
    }while (pMessage != NULL);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to save log buffer in memory
 *
 *  @param[in]  pPkt
 *      Handle to the logging packet to be saved in memory.
 *  @param[in]  memlogHandle
 *      Handle to Memory logging channel .
 *  @param[in]  errCode
 *      Error code populated by the API
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Memlog_loggingFunc
(
    Ti_Pkt*             pPkt,
    Memlog_ChHandle     memlogHandle,
    int32_t*            errCode
)
{
    Memlog_channel*         ptrMemLogChan;
    Memlog_InstanceMCB*     ptrInstanceMCB;

    /* Get Memory loging control block */
    ptrMemLogChan = (Memlog_channel*)memlogHandle;

    /* Get the MEMLOG instance MCB: . */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemLogChan->cfg.memlogInstHandle;

    /* Sanity check of the paket and msgcom channle */
    if (pPkt == NULL || ptrMemLogChan->msgcomWriterChanHandle == NULL)
    {
        /* Sanity check : validation of packet and Msgcom channel */
        System_printf("invalid pkt or msgcom channel handle.\n");
        if(pPkt)
            Pktlib_freePacket(ptrInstanceMCB->cfg.pktlibInstHandle, pPkt);
        ptrMemLogChan->stats.droppedMsg++;
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Send the message to msgcom channel. */
    if (Msgcom_putMessage(ptrMemLogChan->msgcomWriterChanHandle, (MsgCom_Buffer*)pPkt) < 0)
    {
        /* MsgCom put message failed , free the packet */
        Pktlib_freePacket(ptrInstanceMCB->cfg.pktlibInstHandle, pPkt);
        ptrMemLogChan->stats.droppedMsg++;
        *errCode = MEMLOG_EMSGCOM;
        return -1;
    }

    /* Garbage collection of Msgcom channel descriptors */
    Memlog_garbageCollection(ptrMemLogChan);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send messages to memory in background task
 *
 *  @param[in]  memlogHandle
 *      Handle to the MEMLOG channel
 *  @param[in]  ptrMessage
 *      Pointer to the message
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_saveLog
(
    Memlog_ChHandle     memlogHandle,
    Ti_Pkt*             ptrMessage
)
{
    int32_t             errCode;
    Memlog_channel*     ptrMemLogChan;
    Memlog_InstanceMCB* ptrInstanceMCB;

    /* Get Memory loging control block */
    ptrMemLogChan = (Memlog_channel*)memlogHandle;

    /* Get the MEMLOG instance MCB: . */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemLogChan->cfg.memlogInstHandle;

    /* Update stats */
    ptrMemLogChan->stats.totalMsg++;

    /* Send packet to memory logging interface */
    if (Memlog_semIsFree(ptrMemLogChan->syncQueue) == TRUE)
    {
        if (Memlog_loggingFunc(ptrMessage, memlogHandle, &errCode)  < 0)
        {
            /* Error: Unable to send out the packet. */
            ptrMemLogChan->stats.sentFailedMsg++;
            Pktlib_freePacket (ptrInstanceMCB->cfg.pktlibInstHandle, ptrMessage);
        }
        else
        {
            /* Buffer is saved to memory successfully */
            ptrMemLogChan->stats.sentMsg++;
        }
    }
    else
    {
        /* Dropped the packet */
        Pktlib_freePacket (ptrInstanceMCB->cfg.pktlibInstHandle, ptrMessage);
        ptrMemLogChan->stats.droppedMsg++;
    }
    /* Execute garbage collection; if the packet is cloned and it will end up in the garbage
     * collection Queue after CPDMA transmission. */
    Pktlib_garbageCollection (ptrInstanceMCB->cfg.pktlibInstHandle, ptrMemLogChan->msgcomHeapHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to create a named resource for the specific
 *      channel name in the named resource database.
 *
 *  @param[in]  channelName
 *      Name of the channel which is being created and being registered with the
 *      name server.
 *  @param[in]  ptrChannel
 *      Pointer to the channel information.
 *
 * \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   Error code returned by the resource manager
 */
static int32_t Memlog_chanCreateNamedResource
(
    char*               channelName,
    Memlog_channel*     ptrChannel
)
{
    Name_ResourceCfg    namedResourceCfg;
    Memlog_InstanceMCB* ptrInstanceMCB;
    int32_t             errCode;

    /* Get the MEMLOG Instance information. */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrChannel->ptrInstanceMCB;

    /* Initialize the named resource configuration block. */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration with the follow information:
     *  - handle1 -> MEMLOG execution realm
     *  - handle2 -> MEMLOG memory base address
     *  - handle3 -> MEMLOG buffer size
     *  - handle4 -> MEMLOG memory size
     *  - handle5 -> MEMLOG Sync Packet Queue
     */
    namedResourceCfg.handle1  = (uint32_t)ptrInstanceMCB->cfg.realm;
    namedResourceCfg.handle2  = (uint32_t)ptrChannel->cfg.memBaseAddr;
    namedResourceCfg.handle3  = (uint32_t)ptrChannel->cfg.bufferSize;
    namedResourceCfg.handle4  = (uint32_t)ptrChannel->cfg.memSize;
    namedResourceCfg.handle5  = (uint32_t)ptrChannel->syncQueue;
    strncpy(namedResourceCfg.name, channelName, MEMLOG_MAX_CHAR - 1);

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
 *      The function is used to create the MEMLOG channel with specified
 *      configurations. After the channel is created, application needs to push
 *      the Named resource in proper database to be found from ARM side.
 *
 *  @param[in]  ptrChannelConfig
 *      Channel specific configuration.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success   - Valid Channel Handle.
 *  @retval
 *      Error     - NULL (refer to the error code for more details)
 */
Memlog_ChHandle Memlog_create
(
    Memlog_ChannelCfg*     ptrChannelConfig,
    int32_t*               errCode
)
{
    Memlog_InstanceMCB*    ptrInstanceMCB;
    Memlog_channel*        ptrMemlogChan;
    Memlog_logBufferInfo*  ptrLogBuffer;
    Pktlib_HeapCfg         heapCfg;
    Msgcom_ChannelCfg      chConfig;
    char                   chname[MEMLOG_MAX_CHAR];
    uint32_t               index;
    void*                  csHandle;

    /* Sanity Check: Make sure we have a valid configuration passed */
    if ((ptrChannelConfig == NULL))
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Sanity check of the configuration */
    if ( (ptrChannelConfig->memBaseAddr == 0) ||
         (ptrChannelConfig->memSize     == 0) ||
         (ptrChannelConfig->bufferSize  == 0) ||
         (ptrChannelConfig->memlogInstHandle == NULL) )
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Get the MEMLOG Instance information. */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrChannelConfig->memlogInstHandle;

    /* Allocate memory for memlog channel */
    ptrMemlogChan  = ptrInstanceMCB->cfg.malloc(Memlog_MemAllocMode_INTERNAL, sizeof(Memlog_channel));
    if (ptrMemlogChan == NULL)
    {
        *errCode = MEMLOG_ENOMEM;
        return NULL;
    }

    /* Init the Memlog channel */
    memset ((void*)ptrMemlogChan, 0, sizeof(Memlog_channel));

    /* Save Memlog instance MCB handle */
    ptrMemlogChan->ptrInstanceMCB = ptrInstanceMCB;

    /* Save memory logging chanel configuration */
    memcpy((void *)&ptrMemlogChan->cfg, (void *)ptrChannelConfig, sizeof(Memlog_ChannelCfg));

    /* Make buffer cache aligned */
    if (ptrMemlogChan->cfg.bufferSize % CACHE_L2_LINESIZE)
        ptrMemlogChan->cfg.bufferSize += (CACHE_L2_LINESIZE - ptrMemlogChan->cfg.bufferSize % CACHE_L2_LINESIZE);

    /* Save Log buffer memory information */
    ptrMemlogChan->cfg.memSize     = ptrMemlogChan->cfg.memSize & ~(MEMLOG_MEMORY_ALIGNMENT -1) ;

    /* Calculate the number of log buffers */
    ptrMemlogChan->numLogBuffers = ptrMemlogChan->cfg.memSize / ptrMemlogChan->cfg.bufferSize;

    if (ptrMemlogChan->numLogBuffers < ptrMemlogChan->cfg.numPktDescs)
    {
        *errCode = MEMLOG_ENOMEM;
        return NULL;
    }

    /* Creating log buffer list to be used as runtime allocation and re-linking */
    for (index = 0; index < ptrMemlogChan->numLogBuffers; index++)
    {
        /* Allocate log buffer */
        ptrLogBuffer = ptrInstanceMCB->cfg.malloc(Memlog_MemAllocMode_INTERNAL, sizeof(Memlog_logBufferInfo));
        if(ptrLogBuffer == NULL)
        {
            *errCode = MEMLOG_ENOMEM;
             return NULL;
        }

        memset(ptrLogBuffer, 0, sizeof(Memlog_logBufferInfo));
        ptrLogBuffer->bufAddr = ptrMemlogChan->cfg.memBaseAddr + index * ptrMemlogChan->cfg.bufferSize;

        /* Add to log buffer list */
        /* Critical Section: Protect the access to the producer list. */
        csHandle = ptrInstanceMCB->cfg.enterCS();
        Memlog_listAddTail ((Memlog_ListNode**)&ptrMemlogChan->ptrLogBufListTail, (Memlog_ListNode*)ptrLogBuffer);

        /* A node is added to an empty list, update head also */
        if((ptrMemlogChan->ptrLogBufListHead == NULL) && (ptrMemlogChan->ptrLogBufListTail != NULL))
        {
            ptrMemlogChan->ptrLogBufListHead = ptrMemlogChan->ptrLogBufListTail;
        }

        ptrMemlogChan->numBufferInList++;
        ptrInstanceMCB->cfg.exitCS(csHandle);
    }

    /* Create Memory logging heap used for infra-DMA channel */
    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR,"MLHP_%s", ptrMemlogChan->cfg.name );
    heapCfg.pktlibInstHandle                = ptrInstanceMCB->cfg.pktlibInstHandle;
    heapCfg.memRegion                       = ptrChannelConfig->memRegion; 
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = ptrMemlogChan->cfg.bufferSize;
    heapCfg.numPkts                         = ptrMemlogChan->cfg.numPktDescs - 1;
    heapCfg.numZeroBufferPackets            = 1;     /* Used for SyncPkt */
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc  = Memlog_bufferMalloc;
    heapCfg.heapInterfaceTable.dataFree    = Memlog_bufferFree;
    heapCfg.arg                            = (uint32_t)ptrMemlogChan;

    /* Create the Heap for Msgcom channel. */
    ptrMemlogChan->msgcomHeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrMemlogChan->msgcomHeapHandle == NULL)
    {
        return NULL;
    }

    /* Create MsgCom channel for saving memory logs.
       Both msgcom reader and writer channel will be created on the same core.
       The writer channel is used to trigger QUEUE-DMA copy of the log;
       The reader channel is used to read the message, detach old buffer and attach a new buffer.
    */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.msgcomInstHandle            = ptrInstanceMCB->cfg.msgcomInstHandle;
    chConfig.appCallBack                 = NULL;
    chConfig.u.queueDMACfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum=
        (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrMemlogChan->msgcomHeapHandle)));

    /* Create the Message communicator channel. */
    snprintf(chname, MSG_COM_MAX_CHANNEL_LEN, "MLogCh_%s", ptrMemlogChan->cfg.name );
    ptrMemlogChan->msgcomReaderChanHandle = Msgcom_create (chname,
                                                           Msgcom_ChannelType_QUEUE_DMA,
                                                           (Msgcom_ChannelCfg*)&chConfig,
                                                           errCode);
    if (ptrMemlogChan->msgcomReaderChanHandle == NULL)
    {
        System_printf ("Error: Unable to open the MsgCom reader channel. Error : %d\n", *errCode);
        return NULL;
    }

    ptrMemlogChan->msgcomWriterChanHandle = Msgcom_find(ptrInstanceMCB->cfg.msgcomInstHandle,
                                                        chname,
                                                        errCode);
    if (ptrMemlogChan->msgcomWriterChanHandle == NULL)
    {
        System_printf ("Error: Unable to find the MsgCom writer channel. Error : %d\n", *errCode);
        return NULL;
    }

    /* Allocate sync packet from the zero buffer Q */
    ptrMemlogChan->syncPkt  = Pktlib_allocPacket (ptrInstanceMCB->cfg.pktlibInstHandle,
                                                  ptrMemlogChan->msgcomHeapHandle, 0);

    if (ptrMemlogChan->syncPkt == NULL)
    {
        *errCode = MEMLOG_ENOMEM;
        return NULL;
    }

    /* Push the sync pkt to sync Q */
    ptrMemlogChan->syncQueue = Memlog_semInit(ptrMemlogChan->syncPkt, errCode);
    if (ptrMemlogChan->syncQueue == (Qmss_QueueHnd)(-1))
    {
        return NULL;
    }

    /* Create named resource, Application needs to push it to the proper database */
    if ((*errCode = Memlog_chanCreateNamedResource(ptrMemlogChan->cfg.name, ptrMemlogChan)) < 0)
    {
        /* Creating Named Resource failed */
        return NULL;
    }

    return ((Memlog_ChHandle)ptrMemlogChan);
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize  mem logging related resource
 *
 *  @param[in]  memlogHandle
 *      Handle to the memory logging channel
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
int32_t memlog_delete
(
    Memlog_ChHandle*       memlogHandle,
    int32_t*               errCode
)
{
    Memlog_InstanceMCB*    ptrInstanceMCB;
    Memlog_channel*        ptrMemlogChan;
    uint32_t               bufferCnt = 0;
    Memlog_logBufferInfo*  ptrLogBuffer;
    Ti_Pkt*                syncPkt;

    /* Sanity Check */
    if (memlogHandle == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get memory logging channel */
    ptrMemlogChan    =  (Memlog_channel*)memlogHandle;

    /* Get the MEMLOG Instance information. */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrMemlogChan->cfg.memlogInstHandle;

    /* Deinit the sync Queue */
    syncPkt = Memlog_semDeinit(ptrMemlogChan->syncQueue);

    if(syncPkt)
        Pktlib_freePacket (ptrInstanceMCB->cfg.pktlibInstHandle, syncPkt);

    /* Delete heap for msgcom channel */
    if (Pktlib_deleteHeap (ptrInstanceMCB->cfg.pktlibInstHandle, ptrMemlogChan->msgcomHeapHandle, errCode) < 0)
    {
        System_printf  ("Error: Unable to delete the Memory logging Heap [Error code %d]\n", *errCode);
        return -1;
    }

    /* Free all buffers from list */
    do
    {
        /* Remove a buffer from list */
        ptrLogBuffer = (Memlog_logBufferInfo*)Memlog_listRemove ((Memlog_ListNode**)&ptrMemlogChan->ptrLogBufListHead);
        if(ptrLogBuffer == NULL )
            break;

        /* Update buffer counters */
        ptrMemlogChan->numBufferInList--;
        bufferCnt++;

        /* Free log buffer */
        ptrInstanceMCB->cfg.free(Memlog_MemAllocMode_INTERNAL, ptrLogBuffer, sizeof(Memlog_logBufferInfo));

    }while (ptrMemlogChan->numBufferInList);

    /* Sanity check of number of log buffers freed */
    if( (bufferCnt != ptrMemlogChan->numLogBuffers) ||
        (ptrMemlogChan->numBufferInList !=0 ))
            System_printf("Debug: Not all buffers are Freed and put back in buffer list!");

    /* Delete msgcom channels used by memory logging */
    *errCode = Msgcom_delete(ptrMemlogChan->msgcomReaderChanHandle,Memlog_freePkt);
    if (*errCode < 0)
    {
        System_printf ("Error: Producer memory logging reader channle %p deletion failed [Error code %d] \n",
                       ptrMemlogChan->msgcomReaderChanHandle, *errCode);
        return -1;
    }

    *errCode = Msgcom_delete(ptrMemlogChan->msgcomWriterChanHandle,Memlog_freePkt);
    if (*errCode < 0)
    {
        System_printf ("Error: Producer memory logging writer channle %p deletion failed [Error code %d] \n",
                       ptrMemlogChan->msgcomReaderChanHandle, *errCode);
        return -1;
    }

    /* Cleanup memory logging memory */
    ptrInstanceMCB->cfg.free(Memlog_MemAllocMode_INTERNAL, ptrMemlogChan, sizeof(Memlog_channel));

    return 0;
}

#ifdef __ARMv7
/**
 *  @b Description
 *  @n
 *      The function is used to create MEMLOG controller from ARM side.
 *
 *  @param[in]  memlogInstHandle
 *      Handle of the MEMLOG instance
 *  @param[in]  channelName
 *      Name of the MEMLOG channel
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   >0    Handle of the MEMLOG controller
 *  @retval
 *      Error   -   NULL
 */
Memlog_CtrlHandle Memlog_createMemlogController
(
    Memlog_InstHandle       memlogInstHandle,
    char*                   channelName,
    int32_t*                errCode
)
{
    Memlog_InstanceMCB*     ptrInstanceMCB;
    MemLog_Controller*      ptrMemlogController;
    MemLog_LoggerInfo       loggerInfo;
    Name_ResourceCfg        namedResourceCfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((memlogInstHandle == NULL) || (channelName == NULL) )
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Get the MEMLOG Instance information. */
    ptrInstanceMCB = (Memlog_InstanceMCB *)memlogInstHandle;

    /* Initialize to 0 */
    memset((void*)&loggerInfo, 0, sizeof(MemLog_LoggerInfo));

    /* Search for MEMLOG channel name in the named resource instance handle. The name
     * space for MEMLOG channels needs to be unique in the each named resource instance.
       The name space should be pushed to ARM side named resource database if created
       on DSP */
    if (Name_findResource (ptrInstanceMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           channelName, &namedResourceCfg, errCode) < 0)
    {
        /*  Error: The named resource lookup failed. */
        return NULL;
    }

    /* Save Producer control block on this client */
    ptrMemlogController = ptrInstanceMCB->cfg.malloc (Memlog_MemAllocMode_INTERNAL, sizeof(MemLog_Controller));
    if(ptrMemlogController != NULL)
    {
        memset((void*)ptrMemlogController, 0, sizeof(MemLog_Controller));

        /* Populate logger info */
        ptrMemlogController->memlogInstHandle = memlogInstHandle;
        strncpy((void *)ptrMemlogController->name, channelName, MEMLOG_MAX_CHAR -1);

        /* Save logger info from found named resource */
        ptrMemlogController->loggerInfo.realm         = (Memlog_ExecutionRealm)namedResourceCfg.handle1;
        ptrMemlogController->loggerInfo.memBase       = namedResourceCfg.handle2;
        ptrMemlogController->loggerInfo.bufferSize    = namedResourceCfg.handle3;
        ptrMemlogController->loggerInfo.memSize       = namedResourceCfg.handle4;
        ptrMemlogController->loggerInfo.syncQueue     = (Qmss_QueueHnd)namedResourceCfg.handle5;
    }
    else
    {
        *errCode = MEMLOG_ENOMEM;
        return NULL;
    }

    /* Return the producer controller handle */
    return (Memlog_CtrlHandle)ptrMemlogController;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete MEMLOG controller from ARM side.
 *
 *  @param[in]  MemlogCtrlHandle
 *      Handle of the MEMLOG controller
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t  Memlog_deleteMemlogController
(
    Memlog_CtrlHandle       MemlogCtrlHandle,
    int32_t*                errCode
)
{
    Memlog_InstanceMCB*     ptrInstanceMCB;
    MemLog_Controller*      ptrMemlogController;

    /* Sanity Check: Ensure that the arguments are valid */
    if (MemlogCtrlHandle == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get MEMLOG controller */
    ptrMemlogController = (MemLog_Controller *)MemlogCtrlHandle;

    /* Get the MEMLOG Instance information. */
    ptrInstanceMCB = (Memlog_InstanceMCB *)ptrMemlogController->memlogInstHandle;

    /* Cleanup the producer controller memory. */
    ptrInstanceMCB->cfg.free (Memlog_MemAllocMode_INTERNAL, ptrMemlogController, sizeof(MemLog_Controller));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get memory logging information.
 *
 *  @param[in]  memlogCtrlHandle
 *      Handle to the MEMLOG controller
 *  @param[in]  memlogChanName,
 *      Pointer to the string to store memlog channel name.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_getMemLogChanName
(
    Memlog_CtrlHandle       memlogCtrlHandle,
    char*                   memlogChanName,
    int32_t*                errCode
)
{
    MemLog_Controller*      ptrMemlogController;

    /* Sanity check of handle */
    if (memlogCtrlHandle == NULL || memlogChanName == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get Producer controller */
    ptrMemlogController = (MemLog_Controller *)memlogCtrlHandle;

    /* Return the MEMLOG logging information */
    strncpy(memlogChanName, &ptrMemlogController->name[0], MEMLOG_MAX_CHAR - 1);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get memory logging information.
 *
 *  @param[in]  memlogCtrlHandle
 *      Handle to the MEMLOG controller
 *  @param[in]  memlogInfo
 *      Pointer to the Memory logging information include memory base address,
 *      Memory size, buffer size etc.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_getMemlogChanInfo
(
    Memlog_CtrlHandle       memlogCtrlHandle,
    MemLog_LoggerInfo*      memlogInfo,
    int32_t*                errCode
)
{
    MemLog_Controller*      ptrMemlogController;

    /* Sanity check of handle */
    if (memlogCtrlHandle == NULL || memlogInfo == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get Producer controller */
    ptrMemlogController = (MemLog_Controller *)memlogCtrlHandle;

    /* Return the MEMLOG logging information */
    memcpy((void *)memlogInfo, &ptrMemlogController->loggerInfo, sizeof(MemLog_LoggerInfo));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop MEMLOG channel writing to memory. This is needed to
 *   guarantee  the integrity of the data retrieved from memory.
 *
 *  @param[in]  memlogCtrlHandle
 *      Handle to the MEMLOG controller
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_stopLogging
(
    Memlog_CtrlHandle       memlogCtrlHandle,
    int32_t*                errCode
)
{
    MemLog_Controller*      ptrMemlogController;

    /* Sanity check of handle */
    if (memlogCtrlHandle == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get MEMLOG controller */
    ptrMemlogController = (MemLog_Controller *)memlogCtrlHandle;

    /* Acquire the semaphore to stop logging */
    if (ptrMemlogController->loggerInfo.syncQueue &&
       (ptrMemlogController->loggerInfo.syncPkt == NULL) )
    {
        ptrMemlogController->loggerInfo.syncPkt =
           (void *)Memlog_semAcquire(ptrMemlogController->loggerInfo.syncQueue);
    }
    else
    {
        *errCode = MEMLOG_ENOTREADY;
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start MEMLOG logging on a MEMLOG channel
 *
 *  @param[in]  memlogCtrlHandle
 *      Handle to the MEMLOG controller
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_startLogging
(
    Memlog_CtrlHandle       memlogCtrlHandle,
    int32_t*                errCode
)
{
    MemLog_Controller*      ptrMemlogController;

    /* Sanity check of handle */
    if (memlogCtrlHandle == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get MEMLOG controller */
    ptrMemlogController = (MemLog_Controller *)memlogCtrlHandle;

    /* Release the semaphore to start logging */
    if (ptrMemlogController->loggerInfo.syncQueue && ptrMemlogController->loggerInfo.syncPkt)
    {
        Memlog_semRelease(ptrMemlogController->loggerInfo.syncQueue, ptrMemlogController->loggerInfo.syncPkt);
        ptrMemlogController->loggerInfo.syncPkt = NULL;
    }
    else
    {
        *errCode = MEMLOG_ENOTREADY;
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to put memory logging meta information in memory dump
 *      file for post processing of the buffers.
 *      The meta information is always put at the beginning of a given file
 *      This is used on ARM only
 *
 *  @param[in]  memlogChanHandle
 *      Handle to the MEMLOG controller
 *  @param[in]  fp
 *      Handle to the file needs to be updated.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Memlog_saveMetaInfoInFile
(
    Memlog_ChHandle         memlogChanHandle,
    FILE*                   fp,
    int32_t*                errCode
)
{
    MemLog_Controller*      ptrMemlogController;

    /* Sanity check of the parameters */
    if( ( memlogChanHandle == NULL )||
        (fp == NULL) )
    {
        *errCode = MEMLOG_EINVAL;
        return -1;
    }

    /* Get Producer controller */
    ptrMemlogController = (MemLog_Controller *)memlogChanHandle;

    /* Start from the beginning of a file */
    if(fseek(fp, 0, SEEK_SET) < 0 )
    {
        /* Error: Unable to send out the packet. */
        System_printf ("Error: fseek() failed.\n");
        return -1;
    }

    /* Add meta info in the file */
    fprintf(fp, "MemoryBase=0x%x \n", ptrMemlogController->loggerInfo.memBase);
    fprintf(fp, "MemorySize=0x%x \n", ptrMemlogController->loggerInfo.memSize);
    fprintf(fp, "BufferSize=%d \n",   ptrMemlogController->loggerInfo.bufferSize);
    fprintf(fp,  "\n");
    return 0;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to create a MEMLOG instance. MEMLOG instances need to be
 *      created in the context of each processing entity. MEMLOG channels exist only in
 *      the context of the MEMLOG instance.
 *
 *  @param[in]  ptrInstCfg
 *      Pointer to the instance configuration of memory logging.
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success     - Pointer to the MEMLOG instance
 *  @retval
 *      Error       - NULL
 */
Memlog_InstHandle Memlog_createInstance(Memlog_InstCfg* ptrInstCfg, int32_t* errCode)
{
    Memlog_InstanceMCB*     ptrInstanceMCB;

    /* Sanity Check: Validate the arguments. */
    if (ptrInstCfg == NULL)
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the OSAL functions are valid */
    if ((ptrInstCfg->malloc               == NULL) || (ptrInstCfg->free               == NULL)  ||
        (ptrInstCfg->enterCS              == NULL) || (ptrInstCfg->exitCS             == NULL)
//        (ptrInstCfg->mallocMemoryRegion   == NULL) || (ptrInstCfg->freeMemoryRegion   == NULL)  ||
//        (ptrInstCfg->beginMemAccess       == NULL) || (ptrInstCfg->endMemAccess       == NULL)
        )
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that valid instance handles have been passed */
    if ((ptrInstCfg->databaseHandle == NULL) || (ptrInstCfg->pktlibInstHandle == NULL))
    {
        *errCode = MEMLOG_EINVAL;
        return NULL;
    }

    /* Allocate memory for the MEMLOG instance. */
    ptrInstanceMCB = (Memlog_InstanceMCB*)ptrInstCfg->malloc (Memlog_MemAllocMode_INTERNAL, sizeof(Memlog_InstanceMCB));
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MEMLOG_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrInstanceMCB, 0, sizeof(Memlog_InstanceMCB));

    /* Populate the instance configuration. */
    memcpy ((void *)&ptrInstanceMCB->cfg, (void *)ptrInstCfg, sizeof(Memlog_InstCfg));

    /* MEMLOG instance has been created successfully. */
    return (Memlog_InstHandle)ptrInstanceMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the MEMLOG instance.
 *
 *  @param[in]  instHandle
 *      Handle to the MEMLOG instance
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup MEMLOG_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Memlog_deleteInstance(Memlog_InstHandle instHandle, int32_t* errCode)
{
    Memlog_InstanceMCB* ptrInstanceMCB;

    /* Sanity Check: Validate the arguments. */
    ptrInstanceMCB = (Memlog_InstanceMCB*)instHandle;
    if (ptrInstanceMCB == NULL)
    {
        *errCode = MEMLOG_ENOMEM;
        return -1;
    }

    /* Cleanup the memory associated with the instance. */
    ptrInstanceMCB->cfg.free (Memlog_MemAllocMode_INTERNAL, ptrInstanceMCB , sizeof(Memlog_InstanceMCB));
    return 0;
}


