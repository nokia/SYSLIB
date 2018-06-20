/**
 *   @file  pktlib.c
 *
 *   @brief
 *      The file implements the Packet Buffer Library. The library provides
 *      a well defined API which abstracts the CPPI descriptors and provides
 *      services which can be used by the applications without getting to the
 *      lower level details of the descriptors.
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

/* PDK Include files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

extern void* Osal_qmssVirtToPhy (void *ptr); //fzm

/**************************************************************************
 ************************* Local Definitions ******************************
 **************************************************************************/

/** @addtogroup PKTLIB_INTERNAL_SYMBOL
 @{ */

/**
 * @brief   Block starvation count. Starvation queues are allocated and
 * used in blocks of this size.
 */
#define PKTLIB_BLOCK_STARVATION_COUNT       4

/**
 * @brief   Maximum number of instances which are supported.
 */
#define PKTLIB_MAX_INSTANCE                 256

/**
 * @brief   This is the MAX number of heaps which can be managed by a super heap.
 */
#define PKTLIB_MAX_SUPER_MANAGED_HEAPS      4

/**
 * @brief   Flag which indicates that the packet is a CLONE
 */
#define PKTLIB_CLONE_PACKET                 0x1

/**
 * @brief   Descriptor sizes are stored in the packet and this is the starting bit where
 * the descriptor size is stored.
 */
#define PKTLIB_DESC_SIZE_START_BIT          22

/**
 * @brief   Descriptor sizes are stored in the packet and this is the end bit where
 * the descriptor size is stored.
 */
#define PKTLIB_DESC_SIZE_END_BIT            31

/**
 * @brief   This is the starting bit position which is used to store the cloning status
 */
#define PKTLIB_CLONE_END_BIT                31

/**
 * @brief   This is the ending bit position which is used to store the cloning status
 */
#define PKTLIB_CLONE_START_BIT              31

/**
 * @brief   This is the starting bit position which is used to store the reference counter
 */
#define PKTLIB_REF_CNT_START_BIT            0

/**
 * @brief   This is the ending bit position which is used to store the reference counter
 */
#define PKTLIB_REF_CNT_END_BIT              15

/**
 * @brief   This is the MAX reference count for a packet which is derived from the ending & starting
 * bit positions.
 */
#define PKTLIB_MAX_REF_COUNT                ((1 << (PKTLIB_REF_CNT_END_BIT - PKTLIB_REF_CNT_START_BIT + 1)) - 1)

/**
@}
*/

/**************************************************************************
 ************************* Local Structures *******************************
 **************************************************************************/

/** @addtogroup PKTLIB_INTERNAL_DATA_STRUCTURE
 @{ */


typedef struct Pktlib_StarvationQueueBlock Pktlib_StarvationQueueBlock;
/**
 * @brief
 *  PKTLIB StarvationQueue
 *
 * @details
 *  The PKTLIB StarvationQueue Details
 */
typedef struct Pktlib_StarvationQueue
{
    /**
     * @brief   Flag to determine if Queue is assigned
     */
    uint32_t isAssigned;

    /**
     * @brief   Count matching the instantaneous measure similar to reading the register
     *          if the starvation queue counter register was not shared.  Caps at 256 and
     *          Clear on read.
     */
    uint32_t currentCount;

    /**
     * @brief   sum of all read values from the starvation counter
     */
    uint32_t totalCount;

    /**
     * @brief   link to the parent starvation queue block
     */
    Pktlib_StarvationQueueBlock * starvationQueueBlock;
}Pktlib_StarvationQueue;

/**
 * @brief
 *  PKTLIB StarvationQueueBlock
 *
 * @details
 *  The PKTLIB StarvationQueue Details
 */
struct Pktlib_StarvationQueueBlock
{
    /**
     * @brief   link to the next block of 4 queues
     */
    struct Pktlib_StarvationQueueBlock *next;

    /**
     * @brief   Base Queue of the block of 4 registers
     */
    Qmss_QueueHnd baseQueue;

    /**
     * @brief   individual queues
     */
    Pktlib_StarvationQueue queues[4];

};


/**
 * @brief
 *  PKTLIB MCB
 *
 * @details
 *  The PKTLIB Instance master control block which holds information
 *  for the specific PKTLIB instance.
 */
typedef struct Pktlib_MCB
{
    /**
     * @brief   Instance configuration which was used to create the PKTLIB
     * instance
     */
    Pktlib_InstCfg      cfg;

    /**
     * @brief   Head of the list of starvation queue instances
     */
    Pktlib_StarvationQueueBlock *starvationHead;
}Pktlib_MCB;

/**
 * @brief
 *  Packet Library Heap
 *
 * @details
 *  The packet library heap has a list of packets with buffers
 *  and without buffers.
 */
typedef struct Pktlib_Heap
{
    /**
     * @brief   Name of the heap; used to identify it.
     */
    char                name[PKTLIB_MAX_CHAR + 1];

    /**
     * @brief   Instance identifier to which the heap belongs.
     */
    uint8_t             instanceId;

    /**
     * @brief   Instance handle on which the heap was created
     */
    Pktlib_InstHandle   instanceHandle;

    /**
     * @brief   Flag which identifies if the heap is a SUPER heap or not?
     */
    uint16_t            isSuperHeap;

    /**
     * @brief   Flag which identifies if the heap is shared or not?
     */
    uint16_t            sharedHeap;

    /**
     * @brief   Flag which indicates if starvation queues should be used or not?
     */
    uint16_t            useStarvationQueue;

    /**
     * @brief   This the base starvation queue which has been allocated to the heap.
     */
    Qmss_QueueHnd       baseStarvationQueue;

    /**
     * @brief   This is valid only for super heaps and indicates all the member
     * heaps which it is monitoring. This field is NOT applicable for normal
     * heaps.
     */
    Pktlib_HeapHandle   memberHeapHandles[PKTLIB_MAX_SUPER_MANAGED_HEAPS];

    /**
     * @brief   The threshold are passed during heap creation and the module
     * uses this to setup the queue thresholds in the QMSS queues. If the value
     * in the heap data buffer free queues falls below the threshold this is
     * recorded by the Navigator infrastructure.
     */
    uint32_t            dataBufferPktThreshold;

    /**
     * @brief   The threshold are passed during heap creation and the module
     * uses this to setup the queue thresholds in the QMSS queues. If the value
     * in the heap zero buffer free queues falls below the threshold this is
     * recorded by the Navigator infrastructure.
     */
    uint32_t            zeroBufferPktThreshold;

    /**
     * @brief   Each heap only has packets of a specific data size.
     */
    uint32_t            dataBufferSize;

    /**
     * @brief   Garbage Queue Information
     */
    Qmss_Queue          garbageQueueInfo;

    /**
     * @brief   Garbage Queue Handle
     */
    Qmss_QueueHnd       garbageQueueHnd;

    /**
     * @brief   Free queue information which has the queue manager information
     * where all the available packets with buffers are stored.
     */
    Qmss_Queue          freeQueueInfo;

    /**
     * @brief   Free queue handle which stores all the packets with buffers
     */
    Qmss_QueueHnd       freeQueueHnd;

    /**
     * @brief   Free queue information which has the queue manager information
     * where all the available packets with no buffers are stored.
     */
    Qmss_Queue          freeZeroQueueInfo;

    /**
     * @brief   Free queue handle which stores all the packets with no buffers.
     */
    Qmss_QueueHnd       freeZeroQueueHnd;

    /**
     * @brief   Descriptor Size of each descriptor.
     */
    int32_t             descSize;

    /**
     * @brief   Optional application defined argument which is to be passed to the
     * heap interface table
     */
    uint32_t            arg;

    /**
     * @brief   Heap Function Table which was passed during configuration.
     */
    Pktlib_HeapIfTable  heapFxnTable;

    /**
     * @brief   Memory region associated with the heap.
     */
    Qmss_MemRegion      memRegion;

    /**
     * @brief   These are the number of packets which are associated with the
     * data buffer size
     */
    uint32_t            numDataBufferPackets;

    /**
     * @brief   These are the number of zero buffer packets.
     */
    uint32_t            numZeroBufferPackets;

//<fzm>
    Pktlib_Prefetch     prefetchZeroBuffer;
    Pktlib_Prefetch     prefetchDataBuffer;

    /**
     * @brief   Flag which indicates if shared starvation queues should be used
     */
    uint32_t            useSharedStarvationQueue;

    /**
     * @brief   Starvation queue for the buffer free queue.
     */
    Pktlib_StarvationQueue * bufferStarvationQueue;

    /**
     * @brief   Starvation queue for the zero buffer free queue.
     */
    Pktlib_StarvationQueue * zeroBufferStarvationQueue;
//</fzm>

    /**
     * @brief   Padding for cache alignment
     */
    uint8_t             padding[52];
}Pktlib_Heap;

/**
 * @brief
 *  Packet Library information
 *
 * @details
 *  The packet library keeps certain information per packet. This information
 *  is required by the library to provide the required services. This structure
 *  is stored in the descriptor. We currently use the top 10 bits of the original
 *  buffer length and the original buffer pointer to store this information.
 */
typedef struct Pktlib_Info
{
    /**
     * @brief   Packet specific information.
     */
    uint32_t        information;

    /**
     * @brief   This is the original packet from where a packet is cloned.
     * This value is applicable only if the packet type above is set to CLONE
     */
    Ti_Pkt*         donor;

    /**
     * @brief   Pointer to the heap to which the packet belongs.
     */
    Pktlib_Heap*    ptrPktHeap;
}Pktlib_Info;

/**
@}
*/

/**********************************************************************
 ********************* Packet Library Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to set the descriptor power
 *      in the packet.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *  @param[in]  descriptorPower
 *      Descriptor power
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static inline void Pktlib_setDescriptorPower(Ti_Pkt* pPkt, uint16_t descriptorPower)
{
    Cppi_HostDesc*  ptrHostDesc;

    /* Get the host descriptor from the packet. */
    ptrHostDesc = Pktlib_getDescFromPacket(pPkt);

    /* Use the reserved bits in the original buffer length to set the descriptor power */
    CSL_FINSR(ptrHostDesc->origBufferLen, PKTLIB_DESC_SIZE_END_BIT, PKTLIB_DESC_SIZE_START_BIT, descriptorPower);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get the descriptor power
 *      in the packet.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Descriptor size
 */
static inline uint16_t Pktlib_getDescriptorPower(Ti_Pkt* pPkt)
{
    Cppi_HostDesc*  ptrHostDesc;

    /* Get the host descriptor from the packet. */
    ptrHostDesc = Pktlib_getDescFromPacket(pPkt);

    /* Use the reserved bits in the original buffer length to set the descriptor power */
    return CSL_FEXTR(ptrHostDesc->origBufferLen, PKTLIB_DESC_SIZE_END_BIT, PKTLIB_DESC_SIZE_START_BIT);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function is used to set the reference counter
 *
 *  @param[in]  ptrPktLibInfo
 *      Pointer to the internal packet library information
 *  @param[in]  refCount
 *      Reference counter to be set
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static inline void __Pktlib_setRefCount(Pktlib_Info* ptrPktLibInfo, uint16_t refCount)
{
    CSL_FINSR(ptrPktLibInfo->information, PKTLIB_REF_CNT_END_BIT, PKTLIB_REF_CNT_START_BIT, refCount);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function is used to get the reference counter
 *
 *  @param[in]  ptrPktLibInfo
 *      Pointer to the internal packet library information.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Reference Counter.
 */
static inline uint16_t __Pktlib_getRefCount(Pktlib_Info* ptrPktLibInfo)
{
    return CSL_FEXTR(ptrPktLibInfo->information, PKTLIB_REF_CNT_END_BIT, PKTLIB_REF_CNT_START_BIT);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function is used to set the packet cloning status
 *
 *  @param[in]  ptrPktLibInfo
 *      Pointer to the internal packet library information
 *  @param[in]  cloneStatus
 *      Clone Status
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static inline void __Pktlib_setPacketCloneStatus(Pktlib_Info* ptrPktLibInfo, uint8_t cloneStatus)
{
    CSL_FINSR(ptrPktLibInfo->information, PKTLIB_CLONE_END_BIT, PKTLIB_CLONE_START_BIT, cloneStatus);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function is used to get the packet cloning status
 *
 *  @param[in]  ptrPktLibInfo
 *      Pointer to the internal packet library information
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Clone status
 */
static inline uint8_t __Pktlib_getPacketCloneStatus(Pktlib_Info* ptrPktLibInfo)
{
    return CSL_FEXTR(ptrPktLibInfo->information, PKTLIB_CLONE_END_BIT, PKTLIB_CLONE_START_BIT);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get the packet heap
 *
 *  @param[in]  ptrPktLibInfo
 *      Pointer to the internal packet library information
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the packet heap
 */
static inline Pktlib_HeapHandle __Pktlib_getPktHeap(Pktlib_Info* ptrPktLibInfo)
{
    return (Pktlib_HeapHandle)ptrPktLibInfo->ptrPktHeap;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function is used to get the pointer to the
 *      packet library information.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the library information.
 */
static inline Pktlib_Info* Pktlib_getPktLibInfo(Ti_Pkt* pPkt)
{
    /* Convert the descriptor power to the descriptor size. */
    uint32_t descriptorSize = 1 << Pktlib_getDescriptorPower(pPkt);

    /* Use the descriptor size to offset into the packet to get the packet library information which is stored
     * at the end of the packet. */
    return (Pktlib_Info*)((uint8_t*)pPkt + descriptorSize - sizeof(Pktlib_Info));
}

/**
 *  @b Description
 *  @n
 *      The function is used to return the heap handle to which the packet belongs.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Pointer to the heap
 */
Pktlib_HeapHandle Pktlib_getPktHeap(Ti_Pkt* pPkt)
{
    Pktlib_Info*    ptrPktLibInfo;

    /* Get the packet library information */
    ptrPktLibInfo = Pktlib_getPktLibInfo(pPkt);

    /* Return the heap handle from the packet library information. */
    return __Pktlib_getPktHeap(ptrPktLibInfo);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to increment the reference
 *      counter for the specified packet. The function ensures that the
 *      reference counter in the packet does *NOT* exceed the maximum reference
 *      counter.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *     Old reference counter
 */
static inline uint16_t Pktlib_incRefCount(Ti_Pkt* pPkt)
{
    Pktlib_Info* ptrPktLibInfo;
    uint16_t     refCount;

    /* Get the packet library information. */
    ptrPktLibInfo = Pktlib_getPktLibInfo(pPkt);

    /* Get the current reference counter */
    refCount = __Pktlib_getRefCount(ptrPktLibInfo);
    if (refCount < PKTLIB_MAX_REF_COUNT)
        __Pktlib_setRefCount(ptrPktLibInfo, refCount + 1);

    /* Return the old reference count */
    return refCount;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to decrement the reference
 *      counter for the specified packet.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Old reference counter
 */
static inline uint16_t Pktlib_decRefCount(Ti_Pkt* pPkt)
{
    Pktlib_Info* ptrPktLibInfo;
    uint16_t     refCount;

    /* Get the packet library information. */
    ptrPktLibInfo = Pktlib_getPktLibInfo(pPkt);

    /* Get the current reference counter */
    refCount = __Pktlib_getRefCount(ptrPktLibInfo);
    if (refCount > 0)
        __Pktlib_setRefCount(ptrPktLibInfo, refCount - 1);

    /* Return the old reference count */
    return refCount;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to check if the
 *      packet is a data buffer packet or a bufferless packet?
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      1   -   Data Buffer attached
 *  @retval
 *      0   -   Bufferless packet
 */
static inline uint8_t Pktlib_isDataBufferPkt(Ti_Pkt* pPkt)
{
    Cppi_HostDesc* ptrHostDesc = Pktlib_getDescFromPacket(pPkt);

    /* Check if the original buffer address is valid or not? */
    if (ptrHostDesc->origBuffPtr == 0)
        return 0;

    /* Data buffer was attached. */
    return 1;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which modifies the return queue information in the
 *      descriptor to the specified queue. This is just a replacement function and will
 *      be replaced once we get an optimal CPPI API
 *
 *  @param[in]  descType
 *      Type of descriptor - Cppi_DescType_HOST, Cppi_DescType_MONOLITHIC
 *  @param[in]  descAddr
 *      Memory address of descriptor.
 *  @param[in]  queue
 *      Queue Manager - 0 or 1.
 *      Queue Number  - 0 to 4094 with in queue manager 0 or 1.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static inline void Pktlib_cppiSetReturnQueue(Cppi_DescType descType, Cppi_Desc* descAddr, Qmss_Queue* queue)
{
    Cppi_HostDesc* ptrHostDesc = (Cppi_HostDesc *)descAddr;

    /* Clear out the lower order 14 bits of the packet information */
    ptrHostDesc->packetInfo = ptrHostDesc->packetInfo & 0xFFFFC000;

    /* Set the Garbage Queue Information in the descriptor. */
    ptrHostDesc->packetInfo = CSL_FMKR(13, 12, queue->qMgr) |
                              CSL_FMKR(11, 0,  queue->qNum) |
                              ptrHostDesc->packetInfo;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which modifies the return queue information in the
 *      descriptor to the garbage queue.
 *
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static inline void Pktlib_setReturnQueueToGarbage(Ti_Pkt* pPkt)
{
    Pktlib_Heap*    ptrPktHeap;
    Pktlib_Info*    ptrPktLibInfo;

    /* Get the packet library information */
    ptrPktLibInfo = Pktlib_getPktLibInfo(pPkt);

    /* Get the heap information. */
    ptrPktHeap = (Pktlib_Heap*)__Pktlib_getPktHeap(ptrPktLibInfo);

    /* Set the return queue information to be the Heap Garbage Queue in the descriptor. */
    Pktlib_cppiSetReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)pPkt, &ptrPktHeap->garbageQueueInfo);
}

/**
 *  @b Description
 *  @n
 *      The function invalidates the cache contents of the packet. This needs
 *      to be called by the application if the packets are located in shared
 *      or external memory. If there are mutiple chained packets then the
 *      application needs to cycle through the chain and invalidate all the
 *      individual packets.
 *
 *  @param[in]  pPkt
 *      Packet to be invalidated
 *  @param[in]  ptrPktHeap
 *      Heap to which the packet belongs
 *  @param[in]  beginPktAccess
 *      Cache invalidation packet access function which is plugged in
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
static inline void __Pktlib_invalidatePkt
(
    Ti_Pkt*                     pPkt,
    Pktlib_Heap*                ptrPktHeap,
    Osal_PktlibBeginPktAccess   beginPktAccess
)
{
    /* Invalidate the packet. */
    beginPktAccess (ptrPktHeap, pPkt, ptrPktHeap->descSize);
}

/**
 *  @b Description
 *  @n
 *      The function writeback the cache contents of the packet. This needs
 *      to be called by the application if the packets are located in shared or external
 *      memory. If there are mutiple chained packets then the application needs to cycle
 *      through the chain and invalidate all the individual packets.
 *
 *  @param[in]  pPkt
 *      Packet to be written back
 *  @param[in]  ptrPktHeap
 *      Heap to which the packet belongs
 *  @param[in]  endPktAccess
 *      OSAL End packet access API
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
static inline void __Pktlib_writebackPkt
(
    Ti_Pkt*                     pPkt,
    Pktlib_Heap*                ptrPktHeap,
    Osal_PktlibEndPktAccess     endPktAccess
)
{
    /* Writeback the packet. */
    endPktAccess (ptrPktHeap, pPkt, ptrPktHeap->descSize);
}

#ifndef __ARMv7
/**
 *  @b Description
 *  @n
 *      The function is called when the packet ownership is being accepted by the DSP core.
 *      The ownership would transfer from another master i.e. another DSP/ARM core or a
 *      CDPMA block. The function cycles through all the packets which could be linked
 *      together and all the packets are now owned by the callee of the API.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  pPkt
 *      Head of the packet to be owned.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
void Pktlib_getOwnership
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pPkt
)
{
    Pktlib_Heap*                ptrPktHeap;
    Osal_PktlibBeginPktAccess   beginPktAccess;
    Pktlib_MCB*                 ptrPktlibMCB;

    /* Get the PKTLIB MCB */
    ptrPktlibMCB = (Pktlib_MCB *)pktlibInstHandle;

    /* Store the OSAL call functions which are used */
    beginPktAccess = ptrPktlibMCB->cfg.beginPktAccess;

    /* Cycle through all the packets */
    while (pPkt != NULL)
    {
        /* Get the pointer to the heap to which the packet belongs. We are reading the descriptor
         * before invalidating it but since the heap index is stored and is never modified this
         * should not be a problem. */
        ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(pPkt);

        /* Invalidate the packet. */
        __Pktlib_invalidatePkt (pPkt, ptrPktHeap, beginPktAccess);
        pPkt = Pktlib_getNextPacket(pPkt);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called when the packet ownership is being relinquished by the
 *      DSP core. The ownership could be transferred to another master i.e. another
 *      DSP/ARM core or a CDPMA block. The function cycles through all the packets
 *      which could be linked together and all the packets should now be released from
 *      the callee.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  pPkt
 *      Head of the packet to be released
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
void Pktlib_releaseOwnership
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pPkt
)
{
    Pktlib_Heap*                ptrPktHeap;
    Pktlib_MCB*                 ptrPktlibMCB;
    Osal_PktlibEndPktAccess     endPktAccess;

    /* Get the PKTLIB MCB */
    ptrPktlibMCB = (Pktlib_MCB *)pktlibInstHandle;

    /* Store all the OSAL call functions which will be used in the function */
    endPktAccess = ptrPktlibMCB->cfg.endPktAccess;

    /* Cycle through all the packets */
    while (pPkt != NULL)
    {
        /* Get the pointer to the heap to which the packet belongs. */
        ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(pPkt);

        /* Writeback the packet. */
        __Pktlib_writebackPkt(pPkt, ptrPktHeap, endPktAccess);
        pPkt = Pktlib_getNextPacket(pPkt);
    }
    return;
}
#endif /* __ARMv7 */

/**
 *  @b Description
 *  @n
 *      This function is used to read the starvation count. We cannot use the QMSS LLD API
 *      for this purpose because the register is a Clear-on-read.
 *
 *  @param[in]  ptrStarveQueue
 *      Pointer to the heap for which the statistics are required.
 *  @param[out]  dataBufferCount
 *      This is the data buffer starvation counter
 *  @param[out]  zeroDataBufferCount
 *      This is the zero data buffer starvation counter
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static void Pktlib_getQueueStarvationCount
(
    Pktlib_Heap* ptrPktHeap,
    uint8_t*     dataBufferCount,
    uint8_t*     zeroDataBufferCount
)
{
    uint32_t    starvationCount[PKTLIB_BLOCK_STARVATION_COUNT];

    *dataBufferCount = 0;
    Qmss_getStarvationCounts(ptrPktHeap->baseStarvationQueue,
                             PKTLIB_BLOCK_STARVATION_COUNT,
                             starvationCount);
    *dataBufferCount = starvationCount[0];
    *zeroDataBufferCount = starvationCount[1];

    return;
}


/**
 *  @b Description
 *  @n
 *      This function is used to populate the stats for the starvation
 *      queues in a given starvationQueueBlock.  This queries the
 *      hardware for the starvation counts of 4 queues and updates the
 *      internal storage structures.  It keeps a currentCount which
 *      caps at 255 to mimic the behavior of the hardware reads in the
 *      unshared case, and also a cumulative count of all reads
 *
 *  @param[in]  ptrStarvationQueueBlock
 *      Pointer to starvation queue block structure for which statistics
 *      should be updated.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static void Pktlib_populateStarvationQueueBlock
(
    Pktlib_StarvationQueueBlock* ptrStarvationQueueBlock
)
{
    uint32_t    starvationCount[PKTLIB_BLOCK_STARVATION_COUNT];

    Qmss_getStarvationCounts(ptrStarvationQueueBlock->baseQueue,
                             PKTLIB_BLOCK_STARVATION_COUNT,
                             starvationCount);
    for (int i = 0; i < PKTLIB_BLOCK_STARVATION_COUNT; i++)
    {
        ptrStarvationQueueBlock->queues[i].currentCount += starvationCount[i];
        ptrStarvationQueueBlock->queues[i].totalCount += starvationCount[i];
        if (ptrStarvationQueueBlock->queues[i].currentCount > 255)
        {
            ptrStarvationQueueBlock->queues[i].currentCount = 255;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is used to read the starvation count for a shared
 *      starvation queue.  It emulates the clear-on-read behavior of the
 *      register reads in the non-shared case.
 *      Pktlib_populateStarvationQueueBlock should be run before this
 *      function to ensure the stats are up to date from the hardware
 *
 *  @param[in]  ptrStarveQueue
 *      Pointer to starvation queue structure from which statistics should
 *      be retreived.
 *  @param[out]  currentCount
 *      This is the starvation counter since the last read, capped at 255,
 *      similar to the non-shared behavior
 *  @param[out]  totalCount
 *      This is the sum of the all the reads of the register for this
 *      queue.  This may be higher than the the sum of all values returned
 *      in currentCount as this will include counts incremented beyond 255
 *      by reads of other queues in the same block.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static void Pktlib_getSharedQueueStarvationCount
(
    Pktlib_StarvationQueue* ptrStarveQueue,
    uint8_t*     currentCount,
    uint32_t*    totalCount
)
{
    *currentCount = ptrStarveQueue->currentCount;
    ptrStarveQueue->currentCount = 0; //emulate clear on read behavior
    *totalCount = ptrStarveQueue->totalCount;
    return;
}

/**
 *  @b Description
 *  @n
 *      This function is used to find a free shared starvation queue
 *      allocated to this Pktlib instance but not yet assigned to a heap
 *
 *  @param[in]  ptrPktlibMCB
 *      Pointer to the pktlib master control block with the critical
 *      section osals and the pointer to the shared queue block list
 *  @param[out]  ptrPktlibStarvationQueue
 *      Pointer to the free starvation queue found
 *  @param[out]  pStarvationQueue
 *      The queue number corresponding to ptrPktlibStarvationQueue
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Queue found -   1
 *  @retval
 *      Queue not found  - 0
 */
static int32_t Pktlib_getFreeStarvationQueue
(
    Pktlib_MCB* ptrPktlibMCB,
    Pktlib_StarvationQueue** ptrPktlibStarvationQueue,
    Qmss_QueueHnd* pStarvationQueue
)
{
    Pktlib_StarvationQueueBlock *currentBlock = ptrPktlibMCB->starvationHead;
    while (currentBlock != NULL)
    {
        for (int i = 0; i < PKTLIB_BLOCK_STARVATION_COUNT; i++)
        {
            if (currentBlock->queues[i].isAssigned == 0)
            {
                currentBlock->queues[i].isAssigned = 1;
                *ptrPktlibStarvationQueue = &currentBlock->queues[i];
                *pStarvationQueue = currentBlock->baseQueue + i;
                return 1;
            }
        }
        currentBlock = currentBlock->next;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function creates a new starvation queue block.  It requests
 *      4 starvation queues which share the same statistics register and
 *      adds the populated starvation queue block to the list
 *
 *  @param[in]  ptrPktlibMCB
 *      Pointer to the pktlib master control block with the critical
 *      section osals and the pointer to the shared queue block list
 *  @param[out]  ptrStarvationQueueBlock
 *      Pointer to the created starvation queue block
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Pktlib_allocateStarvationQueueBlock
(
    Pktlib_MCB* ptrPktlibMCB,
    Pktlib_StarvationQueueBlock ** ptrStarvationQueueBlock
)
{
    Qmss_QueueHnd   starvationQueueBlock[PKTLIB_BLOCK_STARVATION_COUNT];
    Qmss_QueueHnd   pBaseStarvationQueue = Qmss_queueBlockOpen(starvationQueueBlock,
                                            Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                                            PKTLIB_BLOCK_STARVATION_COUNT,
                                            PKTLIB_BLOCK_STARVATION_COUNT);
    if (pBaseStarvationQueue < 0)
         return -1;

    *ptrStarvationQueueBlock = ptrPktlibMCB->cfg.malloc (Pktlib_MallocMode_GLOBAL, sizeof(Pktlib_StarvationQueueBlock));
    if (*ptrStarvationQueueBlock == NULL)
    {
        return -1;
    }

    memset ((void *)*ptrStarvationQueueBlock, 0, sizeof(Pktlib_StarvationQueueBlock));
    (*ptrStarvationQueueBlock)->baseQueue = pBaseStarvationQueue;
    for (int i = 0; i < PKTLIB_BLOCK_STARVATION_COUNT; i++)
    {
        (*ptrStarvationQueueBlock)->queues[i].starvationQueueBlock = *ptrStarvationQueueBlock;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to allocate a new starvation queue.  It
 *      will first check to see if the Pktlib instance has a free queue
 *      and if not, it will allocate a new block and assign the first
 *      queue from that block.
 *
 *  @param[in]  ptrPktlibMCB
 *      Pointer to the pktlib master control block with the critical
 *      section osals and the pointer to the shared queue block list
 *  @param[out]  ptrPktlibStarvationQueue
 *      Pointer to the free starvation queue assigned
 *  @param[out]  pStarvationQueue
 *      The queue number corresponding to the ptrPktlibStarvationQueue
 *      assigned
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Pktlib_allocateSharedStarvationQueue
(
    Pktlib_MCB* ptrPktlibMCB,
    Pktlib_StarvationQueue** ptrPktlibStarvationQueue,
    Qmss_QueueHnd* pStarvationQueue
)
{
    ptrPktlibMCB->cfg.sharedEnterCS();
    if (Pktlib_getFreeStarvationQueue(ptrPktlibMCB, ptrPktlibStarvationQueue, pStarvationQueue))
    {
        ptrPktlibMCB->cfg.sharedExitCS();
        return 0;
    }

    Pktlib_StarvationQueueBlock * ptrStarvationQueueBlock;
    if (Pktlib_allocateStarvationQueueBlock(ptrPktlibMCB, &ptrStarvationQueueBlock) < 0)
    {
        ptrPktlibMCB->cfg.sharedExitCS();
        return -1;
    }

    if (ptrPktlibMCB->starvationHead == NULL)
    {
        ptrPktlibMCB->starvationHead = ptrStarvationQueueBlock;
    }
    else
    {
        ptrStarvationQueueBlock->next = ptrPktlibMCB->starvationHead;
        ptrPktlibMCB->starvationHead = ptrStarvationQueueBlock;
    }
    ptrStarvationQueueBlock->queues[0].isAssigned = 1;
    ptrPktlibMCB->cfg.sharedExitCS();
    *ptrPktlibStarvationQueue = &(ptrStarvationQueueBlock->queues[0]);
    *pStarvationQueue = ptrStarvationQueueBlock->baseQueue;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to allocate a starvation queue. The starvation counters
 *      in the Navigator infrastructure are a clear on read and also there is no byte
 *      access to this register so a read for a queue would imply that all 4 queues
 *      in that register bank will have there counters reset. This function thus ensures
 *      that all allocations fit in the same register and the rest of the queues
 *      are marked as reserved.
 *
 *  @param[out]  pBaseStarvationQueue
 *      Pointer to the block of starvation queues allocated.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Pktlib_allocateStarvationQueue (Qmss_QueueHnd* pBaseStarvationQueue)
{
    Qmss_QueueHnd   starvationQueueBlock[PKTLIB_BLOCK_STARVATION_COUNT];

    *pBaseStarvationQueue = Qmss_queueBlockOpen(starvationQueueBlock,
                                            Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                                            PKTLIB_BLOCK_STARVATION_COUNT,
                                            PKTLIB_BLOCK_STARVATION_COUNT);
    if (*pBaseStarvationQueue < 0)
         return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get the descriptor size for
 *      the specified memory region.
 *
 *  @param[in]  memRegion
 *      Memory region for which the descriptor size is required.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Error           - -1
 *  @retval
 *      Success         - Descriptor Size
 */
static int32_t Pktlib_getDescSize (Qmss_MemRegion memRegion)
{
    Qmss_MemRegCfg      memRegionInfo;
    uint16_t            index;

    /* Get all the memory region configuration */
    if (Qmss_getMemoryRegionCfg (&memRegionInfo) != QMSS_SOK)
        return -1;

    /* Cycle through allthe memory regions. */
    for (index = 0; index < QMSS_MAX_MEM_REGIONS; index++)
    {
        /* Did we get a match? If so return the descriptor size */
        if (memRegionInfo.memRegInfo[index].memRegion == memRegion)
            return memRegionInfo.memRegInfo[index].descSize;
    }

    /* No match was found we return error */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a heap. There can exist multiple heaps
 *      in the system. Each heap has a specific set of properties and can be
 *      used by applications to have buffers & descriptors residing in different
 *      memory regions with different properties etc.
 *
 *  @param[in]  ptrHeapCfg
 *      Heap Configuration using which the heap is to be created
 *  @param[out]  errCode
 *      Error code populated if there was an error.
 *
 *  @sa
 *      Pktlib Error Codes
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Success -   Heap Handle
 *  @retval
 *      Error   -   NULL (refer to the error code for more details)
 */
Pktlib_HeapHandle Pktlib_createHeap(Pktlib_HeapCfg* ptrHeapCfg, int32_t* errCode)
{
    Pktlib_MCB*             ptrPktlibMCB;
    Pktlib_Info*            ptrPktLibInfo;
    uint32_t                index;
    Pktlib_Heap*            ptrPktHeap;
    Cppi_DescCfg            descCfg;
    Cppi_HostDesc*          ptrHostDesc;
    Qmss_QueueHnd           queueHandle;
    uint8_t*                ptrDataBuffer;
    uint32_t                numAllocated;
    uint8_t                 isAllocated;
    Name_ResourceCfg        nameResourceCfg;
    int32_t                 power = 0;

    /* Initialize the error code */
    *errCode = PKTLIB_EINVAL;

    /* Basic Validations: Ensure that a valid heap configuration was passed */
    if (ptrHeapCfg == NULL)
        return NULL;

    /* Basic Validation: Ensure that a valid instance handle was passed. */
    if (ptrHeapCfg->pktlibInstHandle == NULL)
        return NULL;

    /* Basic Validation: Heaps should either have packets with buffers or packets without buffers. */
    if ((ptrHeapCfg->numPkts == 0) && (ptrHeapCfg->numZeroBufferPackets == 0))
        return NULL;

    /* Basic Validation: If there are buffers to be allocated ensure that a valid alloc and free
     * API Was passed in the interface table */
    if (ptrHeapCfg->numPkts != 0)
    {
        if ((ptrHeapCfg->heapInterfaceTable.dataMalloc == NULL) ||
            (ptrHeapCfg->heapInterfaceTable.dataFree == NULL))
            return NULL;
    }

    /* Get the PKTLIB MCB */
    ptrPktlibMCB = (Pktlib_MCB*)ptrHeapCfg->pktlibInstHandle;

    /* Sanity Check: Ensure that there are no duplicate names; the name space should be
     * unique in the system */
    Name_findResource (ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       (char*)ptrHeapCfg->name, &nameResourceCfg, errCode);

    /* The heap name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the heap name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the heap. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Allocate memory for the new PKTLIB Heap */
    ptrPktHeap = ptrPktlibMCB->cfg.malloc (Pktlib_MallocMode_GLOBAL, sizeof(Pktlib_Heap));
    if (ptrPktHeap == NULL)
    {
        *errCode = PKTLIB_ENOMEM;
        return NULL;
    }

    /* Initialize the heap. */
    memset ((void *)ptrPktHeap, 0, sizeof(Pktlib_Heap));

    /* Copy heap configuration parameters to the heap */
    memcpy ((void *)&ptrPktHeap->heapFxnTable, (void *)&ptrHeapCfg->heapInterfaceTable, sizeof(Pktlib_HeapIfTable));
    ptrPktHeap->instanceId           = ptrPktlibMCB->cfg.instanceId;
    ptrPktHeap->instanceHandle       = ptrHeapCfg->pktlibInstHandle;
    ptrPktHeap->arg                  = ptrHeapCfg->arg;
    ptrPktHeap->sharedHeap           = ptrHeapCfg->sharedHeap;
    ptrPktHeap->memRegion            = ptrHeapCfg->memRegion;
    ptrPktHeap->numDataBufferPackets = ptrHeapCfg->numPkts;
    ptrPktHeap->numZeroBufferPackets = ptrHeapCfg->numZeroBufferPackets;

    /* Get the descriptor size */
    ptrPktHeap->descSize = Pktlib_getDescSize(ptrHeapCfg->memRegion);
    if (ptrPktHeap->descSize < 0)
    {
        /* This can fail if we were passed a memory region which does not exist */
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Ensure that the descriptor size is a multiple of 2. These are the only supported
     * descriptor sizes. Packet library now stores packet specific information at the end
     * of the descriptor. */
    {
        int32_t x = ptrPktHeap->descSize;
        while (1)
        {
            if ((x % 2) != 0)
            {
                *errCode = PKTLIB_EINVAL;
                return NULL;
            }
            power++;
            x = (x / 2);
            if (x == 1)
                break;
        }
    }

    /* PKTLIB support a maximum descriptor size of 1024. */
    if (power > 10)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Store the heap configuration parameters */
    ptrPktHeap->useStarvationQueue     = ptrHeapCfg->useStarvationQueue;
    ptrPktHeap->dataBufferPktThreshold = ptrHeapCfg->dataBufferPktThreshold;
    ptrPktHeap->zeroBufferPktThreshold = ptrHeapCfg->zeroBufferPktThreshold;
    ptrPktHeap->dataBufferSize         = ptrHeapCfg->dataBufferSize;

    /* Is this a heap which uses starvation queues? */
    if (ptrPktHeap->useStarvationQueue)
    {
        /* YES. So we need to allocate a starvation queue number which can be used. */
        if (ptrPktlibMCB->cfg.useSharedStarvationQueue)
        {
            if (Pktlib_allocateSharedStarvationQueue(ptrPktlibMCB, &ptrPktHeap->bufferStarvationQueue, &ptrPktHeap->baseStarvationQueue) < 0)
            {
                *errCode = PKLIB_ERESOURCE;
                return NULL;
            }
            ptrPktHeap->useSharedStarvationQueue = 1;
        }
        else
        {
            if (Pktlib_allocateStarvationQueue(&ptrPktHeap->baseStarvationQueue) < 0)
            {
                *errCode = PKLIB_ERESOURCE;
                return NULL;
            }
        }
    }

    /* Are there packets with data buffers? */
    if (ptrHeapCfg->numPkts != 0)
    {
        /* YES. Check if we need to allocate a STARVATION Counter queue or not? */
        if (ptrPktHeap->useStarvationQueue)
        {
            /* Use the allocated starvation queue */
            ptrPktHeap->freeQueueHnd = ptrPktHeap->baseStarvationQueue;
        }
        else
        {
            /* Allocate a general purpose queue where these packets will be stored. */
            ptrPktHeap->freeQueueHnd = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                      QMSS_PARAM_NOT_SPECIFIED,
                                                      &isAllocated);
            if (ptrPktHeap->freeQueueHnd < 0)
            {
                *errCode = PKLIB_ERESOURCE;
                return NULL;
            }
        }
    }

    /* Are there packets with zero data buffers? */
    if (ptrHeapCfg->numZeroBufferPackets != 0)
    {
        /* YES. Check if we need to allocate a STARVATION Counter queue or not? */
        if (ptrPktHeap->useStarvationQueue)
        {
            Qmss_QueueHnd zeroBufferStarvationQueue = 0;
            if (ptrPktHeap->useSharedStarvationQueue)
            {
                if (Pktlib_allocateSharedStarvationQueue(ptrPktlibMCB, &ptrPktHeap->bufferStarvationQueue, &zeroBufferStarvationQueue) < 0)
                {
                    *errCode = PKLIB_ERESOURCE;
                    return NULL;
                }
            }
            else
            {
                zeroBufferStarvationQueue = ptrPktHeap->baseStarvationQueue + 1;
            }

            /* Allocate a starvation queue where these packets will be stored. */
            ptrPktHeap->freeZeroQueueHnd = Qmss_queueOpen(Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                                                          zeroBufferStarvationQueue,
                                                          &isAllocated);
            if (ptrPktHeap->freeZeroQueueHnd < 0)
            {
                *errCode = PKLIB_ERESOURCE;
                return NULL;
            }
        }
        else
        {
            /* Allocate a general purpose queue where these packets will be stored. */
            ptrPktHeap->freeZeroQueueHnd = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                          QMSS_PARAM_NOT_SPECIFIED,
                                                          &isAllocated);
            if (ptrPktHeap->freeZeroQueueHnd < 0)
            {
                *errCode = PKLIB_ERESOURCE;
                return NULL;
            }
        }
    }

    /* Set the queue threshold: Initially we configure the thresholds so that they are set only if the
     * number of packets in the queue exceed the MAX allowed. This is done because the queues are empty
     * in the beginning and so this will cause a false low detection.
     *  - To be able to work with the thresholds the MPU needs to be configured. Please refer to the
     *    Navigator UG for more information. If this is NOT done the configuration of thresholds will
     *    NOT work; so here we also check for this condition. */
    if (ptrHeapCfg->numZeroBufferPackets != 0)
    {
        /* Zero Buffer Packets are configured. Is the zero buffer data threshold setup? */
        if (ptrHeapCfg->zeroBufferPktThreshold > 0)
        {
            /* YES. Set the Queue threshold to the MAX value. */
            Qmss_setQueueThreshold(ptrPktHeap->freeZeroQueueHnd, 1, 10);

            /* Check if the MPU has been properly setup? */
            if (Qmss_getQueueThreshold(ptrPktHeap->freeZeroQueueHnd) != 10)
            {
                *errCode = PKTLIB_EPERM;
                return NULL;
            }
        }
    }

    /* Are data buffer packets present in the heap. */
    if (ptrHeapCfg->numPkts != 0)
    {
        /* Data Buffer packets are configured. Is the data buffer threshold setup? */
        if (ptrHeapCfg->dataBufferPktThreshold > 0)
        {
            /* YES. Set the Queue threshold to the MAX value. */
            Qmss_setQueueThreshold(ptrPktHeap->freeQueueHnd, 1, 10);

            /* Check if the MPU has been properly setup? */
            if (Qmss_getQueueThreshold(ptrPktHeap->freeQueueHnd) != 10)
            {
                *errCode = PKTLIB_EPERM;
                return NULL;
            }
        }
    }

    /* <fzm> Setup the queue thresholds
     * it had to be moved from it's original position, because the threshold
     * set right above doesn't work well with queues that have more that 1023
     * descriptors - it gives false positives until the queue is used for the
     * first time (pop/push).
     * https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/p/6067/33203.aspx
    */
    if (ptrHeapCfg->numZeroBufferPackets != 0)
    {
        if (ptrHeapCfg->zeroBufferPktThreshold > 0)
        {
            uint32_t    threshold = 1;
            uint32_t    x = (ptrHeapCfg->zeroBufferPktThreshold + 1);
            while (1)
            {
                x = x / 2;
                if (x <= 1)
                    break;
                threshold++;
            }
            /* Setup the Queue threshold */
            Qmss_setQueueThreshold(ptrPktHeap->freeZeroQueueHnd, 0, threshold);
        }
    }
    if (ptrHeapCfg->numPkts != 0)
    {
        if (ptrHeapCfg->dataBufferPktThreshold > 0)
        {
            uint32_t    threshold = 1;
            uint32_t    x = (ptrHeapCfg->dataBufferPktThreshold + 1);
            while (1)
            {
                x = x / 2;
                if (x <= 1)
                    break;
                threshold++;
            }
            /* Setup the Queue threshold */
            Qmss_setQueueThreshold(ptrPktHeap->freeQueueHnd, 0, threshold);
        }
    }
    // </fzm>

    /* Initialize the descriptor configuration */
    memset ((void *)&descCfg, 0, sizeof(Cppi_DescCfg));

    /* Populate the descriptor configuration:  */
    descCfg.memRegion                 = ptrHeapCfg->memRegion;
    descCfg.descNum                   = ptrHeapCfg->numPkts + ptrHeapCfg->numZeroBufferPackets;
    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType                  = Cppi_DescType_HOST;
    descCfg.returnQueue.qMgr          = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum          = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.epibPresent               = Cppi_EPIB_EPIB_PRESENT;
    // <fzm>
    if(ptrHeapCfg->pushToHead)
        descCfg.returnPushPolicy          = Qmss_Location_HEAD;
    else
        descCfg.returnPushPolicy          = Qmss_Location_TAIL;
    // </fzm>
    descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_BUFFER;
    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors and place them into a temporary queue. */
    queueHandle = Cppi_initDescriptor (&descCfg, &numAllocated);
    if (queueHandle < 0)
    {
        /* Setup the error code properly. */
        *errCode = (int32_t)queueHandle;
        return NULL;
    }

    /* Make sure we were allocated what we requested for*/
    if (numAllocated != descCfg.descNum)
    {
        /* Setup the error code properly. */
        *errCode = PKLIB_ERESOURCE;
        return NULL;
    }

    /* Create the garbage queue. */
    ptrPktHeap->garbageQueueHnd = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                 QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (ptrPktHeap->garbageQueueHnd == 0)
    {
        /* Setup the error code properly. */
        *errCode = PKLIB_ERESOURCE;
        return NULL;
    }

    /* Get the queue information for the Garbage queue. */
    ptrPktHeap->garbageQueueInfo = Qmss_getQueueNumber(ptrPktHeap->garbageQueueHnd);

    /* Get the queue information of the Zero Buffer Queue. */
    ptrPktHeap->freeZeroQueueInfo = Qmss_getQueueNumber(ptrPktHeap->freeZeroQueueHnd);

    /* Allocate the specified number of zero packets and place them into the corresponding queue. */
    for (index = 0; index < ptrHeapCfg->numZeroBufferPackets; index++)
    {
        /* Pop a descriptor of the free queue. */
        ptrHostDesc = (Cppi_HostDesc*)QMSS_DESC_PTR(Qmss_queuePop(queueHandle));
        if (ptrHostDesc == NULL)
        {
            /* Setup the error code properly. */
            *errCode = PKLIB_ERESOURCE;
            return NULL;
        }

        // fzm
        // Store information about heap for external usage by other applications for debug purpose
        if (!index)
        {
            System_printf("DBG: %s GQ[%d], ZBQ[%d]-SIZE[%u]-START[%p]",
                          ptrHeapCfg->name,
                          ptrPktHeap->garbageQueueHnd,
                          ptrPktHeap->freeZeroQueueHnd,
                          ptrHeapCfg->numZeroBufferPackets,
                          Osal_qmssVirtToPhy((void *)ptrHostDesc));
        }

        /* Reset the original buffer information in the descriptor. */
        Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, NULL, 0);

        /* Set the descriptor power in the descriptor. */
        Pktlib_setDescriptorPower((Ti_Pkt*)ptrHostDesc, power);

        /* Get the packet library information in the packet */
        ptrPktLibInfo = Pktlib_getPktLibInfo((Ti_Pkt*)ptrHostDesc);

        /* Initialize the packet library information
         * - No references are being held
         * - No donors of the packet
         * - Reset the clone status since the packet has not been cloned */
        ptrPktLibInfo->ptrPktHeap   = ptrPktHeap;
        ptrPktLibInfo->donor        = 0;
        __Pktlib_setRefCount (ptrPktLibInfo, 0);
        __Pktlib_setPacketCloneStatus (ptrPktLibInfo, 0);

        /* All zero buffer packets which will be used only for cloning are by default to be
         * cleaned up by default if passed to the IP blocks to the garbage queues. This is
         * because cloned packets will hold references */
        Pktlib_cppiSetReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, &ptrPktHeap->garbageQueueInfo);

        /* Writeback the contents of the packet back into the memory  */
        ptrPktlibMCB->cfg.endMemAccess (ptrHostDesc, ptrPktHeap->descSize);

        /* Push this descriptor into the corresponding queue */
        Qmss_queuePushDesc(ptrPktHeap->freeZeroQueueHnd, (Cppi_Desc*)ptrHostDesc);
    }

    /* Get the queue information of the Data Buffer Queue. */
    ptrPktHeap->freeQueueInfo = Qmss_getQueueNumber(ptrPktHeap->freeQueueHnd);

    /* Allocate the specified number of data packets and place them into the corresponding queue. */
    /* Round the allocation to 64 bytes to avoid two buffers sharing a cache line */
    uint32_t allocSize = (((ptrHeapCfg->dataBufferSize - 1) >> 6) + 1) << 6;
    uint8_t* allocStart;
    if (ptrHeapCfg->numPkts > 0 && ptrHeapCfg->linearAlloc)
    {
        /* Allocate memory for the data buffer. */
        allocStart = ptrHeapCfg->heapInterfaceTable.dataMalloc(allocSize * ptrHeapCfg->numPkts, ptrHeapCfg->arg);
        if (allocStart == NULL)
        {
            /* Setup the error code properly. */
            *errCode = PKTLIB_ENOMEM;
            return NULL;
        }
    }

    for (index = 0; index < ptrHeapCfg->numPkts; index++)
    {
        /* Pop a descriptor of the free queue. */
        ptrHostDesc = (Cppi_HostDesc*)QMSS_DESC_PTR(Qmss_queuePop(queueHandle));
        if (ptrHostDesc == NULL)
        {
            /* Setup the error code properly. */
            *errCode = PKLIB_ERESOURCE;
            return NULL;
        }

        // fzm
        // Store information about heap for external usage by other applications for debug purpose
        if (!index)
        {
            System_printf("DBG: %s GQ[%d], FDQ[%d]-SIZE[%u]-START[%p]-AllocSz[%u]",
                          ptrHeapCfg->name,
                          ptrPktHeap->garbageQueueHnd,
                          ptrPktHeap->freeQueueHnd,
                          ptrHeapCfg->numPkts,
                          Osal_qmssVirtToPhy((void *)ptrHostDesc),
                          allocSize);
        }
        if (ptrHeapCfg->linearAlloc)
        {
            ptrDataBuffer = allocStart + allocSize * index;
        }
        else
        {
            /* Allocate memory for the data buffer. */
            ptrDataBuffer = ptrHeapCfg->heapInterfaceTable.dataMalloc(ptrHeapCfg->dataBufferSize, ptrHeapCfg->arg);
            if (ptrDataBuffer == NULL)
            {
                /* Setup the error code properly. */
                *errCode = PKTLIB_ENOMEM;
                return NULL;
            }
        }

        /* Set the original buffer information in the descriptor. */
        Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,
                                ptrDataBuffer, ptrHeapCfg->dataBufferSize);

        /* Set the descriptor power in the descriptor. */
        Pktlib_setDescriptorPower( (Ti_Pkt*)ptrHostDesc, power);

        /* Get the packet library information in the packet */
        ptrPktLibInfo = Pktlib_getPktLibInfo((Ti_Pkt*)ptrHostDesc);

        /* Initialize the packet library information
         * - No references are being held
         * - No donors of the packet
         * - Reset the clone status since the packet has not been cloned */
        ptrPktLibInfo->ptrPktHeap   = ptrPktHeap;
        ptrPktLibInfo->donor        = 0;
        __Pktlib_setRefCount (ptrPktLibInfo, 0);
        __Pktlib_setPacketCloneStatus (ptrPktLibInfo, 0);

        /* Set the data and payload length into the packet. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t*)ptrDataBuffer, ptrHeapCfg->dataBufferSize);

        /* Set the return queue to be the free queue */
        Pktlib_cppiSetReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, &ptrPktHeap->freeQueueInfo);

        /* Writeback the contents of the data buffer. The BIOS HeapMem heaps store the heap size in the data buffers
         * and since the cache is dirty it can cause spurious corruption in the received data buffers. So here we
         * release ownership of the data buffer and make sure that the cache and memory are synchronized. The cache
         * writeback is preferred here because an application might have prepopulated the data buffer during malloc */
        ptrPktlibMCB->cfg.endMemAccess (ptrDataBuffer, ptrHeapCfg->dataBufferSize);

        /* Writeback the contents of the packet back into the memory  */
        ptrPktlibMCB->cfg.endMemAccess (ptrHostDesc, ptrPktHeap->descSize);

        /* Push this descriptor into the corresponding queue */
        Qmss_queuePushDesc(ptrPktHeap->freeQueueHnd, (Cppi_Desc*)ptrHostDesc);
    }

    /* Close the temporary queue */
    Qmss_queueClose(queueHandle);

    /* The heap has been successfully allocated so initialize all the fields */
    strncpy(ptrPktHeap->name, ptrHeapCfg->name, PKTLIB_MAX_CHAR);

    /* Writeback the contents of the heap */
    ptrPktlibMCB->cfg.endMemAccess (ptrPktHeap, sizeof(Pktlib_Heap));

    /* Initialize the named resource configuration. */
    memset ((void*)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    nameResourceCfg.handle1  = (uint32_t)ptrPktHeap;
    strncpy(nameResourceCfg.name, ptrPktHeap->name, RESMGR_MAX_CHAR);

    /* Create & register the PKTLIB heap */
    if (Name_createResource(ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &nameResourceCfg, errCode) < 0)
        return NULL;

    /* There was no error and the heap has been successfully created. */
    *errCode = 0;
    return (Pktlib_HeapHandle)ptrPktHeap;
}

/**
 *  @b Description
 *  @n
 *      Internal function which compares the data buffer sizes of the 2 heaps
 *
 *  @param[in]  a
 *      Handle to the Heap1 to be compared
 *  @param[in]  b
 *      Handle to the Heap1 to be compared
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      0   - Data Buffer Sizes are the same
 *  @retval
 *      1   - Data Buffer size of Heap 'a' is greater than Data Buffer size of Heap 'b'
 *  @retval
 *      <0  - Data Buffer size of Heap 'a' is less than Data Buffer size of Heap 'b'
 */
static int32_t Pktlib_cmpDataBufferSize(Pktlib_HeapHandle a, Pktlib_HeapHandle b)
{
    if (Pktlib_getMaxBufferSize(a) > Pktlib_getMaxBufferSize(b))
        return 1;
    else if (Pktlib_getMaxBufferSize(a) < Pktlib_getMaxBufferSize(b))
        return -1;
    else
        return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to sort the heaps on the basis of the data buffer size.
 *
 *  @param[in]  memberHeaps
 *      Array of the heaps which have to be sorted.
 *  @param[in]  len
 *      Number of heaps to be sorted.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Pktlib_sortOnDataBufferSize
(
    Pktlib_HeapHandle memberHeaps[],
    uint32_t          len
)
{
    uint32_t                indx;
    uint32_t                heapStartIndex = 0;
    Pktlib_HeapHandle       cur_val;
    Pktlib_HeapHandle       prev_val;

    prev_val = memberHeaps[heapStartIndex];

    for (indx = heapStartIndex+1; indx < len; ++indx)
    {
        cur_val = memberHeaps[indx];
        if (Pktlib_cmpDataBufferSize(prev_val, cur_val) > 0)
        {
            /* out of order: array[indx-1] > array[indx] */
            uint32_t    indx2;
            memberHeaps[indx] = prev_val; /* move up the larger item first */

            /* find the insertion point for the smaller item */
            for (indx2 = indx - 1; indx2 > 0;)
            {
                Pktlib_HeapHandle temp_val = memberHeaps[indx2 - 1];
                if (Pktlib_cmpDataBufferSize(temp_val, cur_val) > 0)
                {
                    memberHeaps[indx2--] = temp_val;
                    /* still out of order, move up 1 slot to make room */
                }
                else
                    break;
            }
            memberHeaps[indx2] = cur_val; /* insert the smaller item right here */
        }
        else
        {
            /* in order, advance to next element */
            prev_val = cur_val;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a super heap. The super heap is a collection
 *      of multiple heaps of different data buffer sizes. When an application tries
 *      to allocate memory using a super heap the 'best' size heap is used to fit
 *      the allocation request. This allows applications to optimize the memory usage.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle for which the super heap is being created
 *  @param[in]  name
 *      Name of the super heap
 *  @param[in]  memberHeaps
 *      Array of heaps which will be managed by the super heaps.
 *  @param[in]  numMemberHeaps
 *      Number of member heaps which need to be monitored by the Super Heap.
 *  @param[out]  errCode
 *      Error code populated if there was an error else 0
 *
 *  @sa
 *      Pktlib Error Codes
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Success -   Super Heap Handle
 *  @retval
 *      Error   -   NULL
 */
Pktlib_HeapHandle Pktlib_createSuperHeap
(
    Pktlib_InstHandle   pktlibInstHandle,
    const char*         name,
    Pktlib_HeapHandle   memberHeaps[],
    int32_t             numMemberHeaps,
    int32_t*            errCode
)
{
    int32_t                 index;
    Pktlib_Heap*            ptrSuperHeap;
    Pktlib_MCB*             ptrPktlibMCB;
    Name_ResourceCfg        nameResourceCfg;

    /* Basic Validations: */
    if ((name == NULL) || (pktlibInstHandle == NULL))
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Super heaps are useful only if we have @ least 2 heaps to manage.
     * There is no point in executing with just 1 heap. Similarly we cannot
     * exceed the MAX permissible */
    if ((numMemberHeaps < 2) || (numMemberHeaps > PKTLIB_MAX_SUPER_MANAGED_HEAPS))
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Get the packet library instance information. */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Sanity Check: Ensure that there are no duplicate names; the name space should be
     * unique in the system */
    Name_findResource (ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       (char*)name, &nameResourceCfg, errCode);

    /* The heap name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the heap name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the heap. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Allocate memory for the super heap */
    ptrSuperHeap = ptrPktlibMCB->cfg.malloc (Pktlib_MallocMode_GLOBAL, sizeof(Pktlib_Heap));
    if (ptrSuperHeap == NULL)
    {
        *errCode = PKTLIB_ENOMEM;
        return NULL;
    }

    /* Initialize the super heap */
    memset ((void *)ptrSuperHeap, 0, sizeof(Pktlib_Heap));

    /* Populate the super heap with the necessary information. */
    strncpy(ptrSuperHeap->name, name, PKTLIB_MAX_CHAR);

    /* Mark this as a super heap. Super heaps are never shared. */
    ptrSuperHeap->isSuperHeap       = 1;
    ptrSuperHeap->sharedHeap        = 0;
    ptrSuperHeap->instanceId        = ptrPktlibMCB->cfg.instanceId;
    ptrSuperHeap->instanceHandle    = (Pktlib_InstHandle)ptrPktlibMCB;

    /* Copy over the member heap handles */
    for (index = 0; index < numMemberHeaps; index++)
        ptrSuperHeap->memberHeapHandles[index] = memberHeaps[index];

    /* We need to now sort the heaps on the data buffer size */
    Pktlib_sortOnDataBufferSize(ptrSuperHeap->memberHeapHandles, numMemberHeaps);

    /* There was no error; super heap has been created successfully. */
    *errCode = 0;

    /* Writeback the contents of the heap */
    ptrPktlibMCB->cfg.endMemAccess (ptrSuperHeap, sizeof(Pktlib_Heap));

    /* Initialize the named resource configuration. */
    memset ((void*)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    nameResourceCfg.handle1  = (uint32_t)ptrSuperHeap;
    strncpy(nameResourceCfg.name, ptrSuperHeap->name, RESMGR_MAX_CHAR);

    /* Create & register the PKTLIB heap */
    if (Name_createResource(ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &nameResourceCfg, errCode) < 0)
        return NULL;

    /* Super heap created successfully. */
    return (Pktlib_HeapHandle)ptrSuperHeap;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a previously created heap using the name
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle in which the heap is being found.
 *  @param[in]  name
 *      Name of the heap
 *  @param[out] errCode
 *      Error code populated on error
 *
  *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Success -   Heap Handle
 *  @retval
 *      Error   -   NULL
 */
Pktlib_HeapHandle Pktlib_findHeapByName
(
    Pktlib_InstHandle   pktlibInstHandle,
    const char*         name,
    int32_t*            errCode
)
{
    Pktlib_Heap*            ptrPktHeap;
    Pktlib_MCB*             ptrPktlibMCB;
    Name_ResourceCfg        nameResourceCfg;

    /* Sanity Check: Validate the arguments. */
    if (pktlibInstHandle == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Get the packet library instance MCB */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Find the heap name in the specified named resource instance */
    if (Name_findResource (ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           (char*)name, &nameResourceCfg, errCode) < 0)
    {
        /* Resource manager lookup failed. This could be because the name is
         * not found or because the resource manager find failed internally
         * In either case the error code is properly set. */
        return NULL;
    }

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap*)nameResourceCfg.handle1;

    /* Invalidate the contents of the heap */
    ptrPktlibMCB->cfg.beginMemAccess (ptrPktHeap, sizeof(Pktlib_Heap));

    /* Control comes here implies that there was a perfect match. This will happen
     * for all heaps irrespective of whether these heaps are LOCAL or SHARED since
     * the named resource is fundamentally shared across all cores. So here we
     * implemented the shared vs. local heap functionality. */
    if (ptrPktHeap->sharedHeap == 1)
    {
        /* Shared Heap: Return the handle to the shared heap */
        return (Pktlib_HeapHandle)ptrPktHeap;
    }

    /* Local heap. Lookup is allowed only if the heap instance matches the requesting
     * instance. Instances are core specific so this should be ok */
    if (ptrPktHeap->instanceHandle == pktlibInstHandle)
        return (Pktlib_HeapHandle)ptrPktHeap;

    /* Instance does not match so we cannot return the heap handle. */
    *errCode = 0;
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the stats of the specified heap.
 *
 *  @param[in]  heapHandle
 *      Handle of the heap for which the statistics are requested.
 *  @param[out] ptrHeapStats
 *      Heap Statistics populated by this API.
 *
  *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Pktlib_getHeapStats
(
    Pktlib_HeapHandle   heapHandle,
    Pktlib_HeapStats*   ptrHeapStats
)
{
    Pktlib_Heap*   ptrPktHeap = (Pktlib_Heap*)heapHandle;

    /* Get the snapshot of the stats. */
    ptrHeapStats->numFreeDataPackets   = Qmss_getQueueEntryCount(ptrPktHeap->freeQueueHnd);
    ptrHeapStats->numPacketsinGarbage  = Qmss_getQueueEntryCount(ptrPktHeap->garbageQueueHnd);

    /* Zero Queue stats are retreived only if the heap supports the mode. */
    if (ptrPktHeap->freeZeroQueueHnd)
        ptrHeapStats->numZeroBufferPackets = Qmss_getQueueEntryCount(ptrPktHeap->freeZeroQueueHnd);
    else
        ptrHeapStats->numZeroBufferPackets = 0;

    /* Check if the heap support data thresholds? */
    if (ptrPktHeap->dataBufferPktThreshold)
        ptrHeapStats->dataBufferThresholdStatus   = Qmss_getQueueThresholdStatus(ptrPktHeap->freeQueueHnd);
    else
        ptrHeapStats->dataBufferThresholdStatus   = 0;

    /* Check if the heap support zero buffer thresholds? */
    if (ptrPktHeap->zeroBufferPktThreshold)
        ptrHeapStats->zeroDataBufferThresholdStatus   = Qmss_getQueueThresholdStatus(ptrPktHeap->freeZeroQueueHnd);
    else
        ptrHeapStats->zeroDataBufferThresholdStatus   = 0;

    /* Check if the heap supports starvation queues */
    if (ptrPktHeap->useStarvationQueue)
    {
        if (ptrPktHeap->useSharedStarvationQueue)
        {
            /* Yes: Extract the starvation counters. */
            if (ptrPktHeap->bufferStarvationQueue)
            {
                Pktlib_populateStarvationQueueBlock(ptrPktHeap->bufferStarvationQueue->starvationQueueBlock);
                Pktlib_getSharedQueueStarvationCount(ptrPktHeap->bufferStarvationQueue,
                                               &ptrHeapStats->dataBufferStarvationCounter,
                                               &ptrHeapStats->extendedDataBufferStarvationCounter);
            }
            if (ptrPktHeap->zeroBufferStarvationQueue)
            {
                //Populating the block is not necessary if the buffer and zero buffer share the same block
                if ((ptrPktHeap->bufferStarvationQueue && ptrPktHeap->bufferStarvationQueue->starvationQueueBlock != ptrPktHeap->zeroBufferStarvationQueue->starvationQueueBlock) ||
                    ptrPktHeap->bufferStarvationQueue == NULL)
                {
                    Pktlib_populateStarvationQueueBlock(ptrPktHeap->zeroBufferStarvationQueue->starvationQueueBlock);
                }
                Pktlib_getSharedQueueStarvationCount(ptrPktHeap->zeroBufferStarvationQueue,
                                               &ptrHeapStats->zeroDataBufferStarvationCounter,
                                               &ptrHeapStats->extendedZeroDataBufferStarvationCounter);
            }
        }
        else
        {
            Pktlib_getQueueStarvationCount(ptrPktHeap, &ptrHeapStats->dataBufferStarvationCounter,
                                           &ptrHeapStats->zeroDataBufferStarvationCounter);
        }
    }
    else
    {
        /* No simply reset the starvation counters. */
        ptrHeapStats->dataBufferStarvationCounter = 0;
        ptrHeapStats->zeroDataBufferStarvationCounter = 0;
        ptrHeapStats->extendedDataBufferStarvationCounter = 0;
        ptrHeapStats->extendedZeroDataBufferStarvationCounter = 0;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup a packet.
 *
 *  @param[in]  ptrPktlibMCB
 *      Pointer to the PKTLIB MCB
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *  @param[in]  followLinks
 *      Flag which indicates if the links in the packet need to be
 *      followed.
 *
  *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void __Pktlib_freePacket
(
    Pktlib_MCB*     ptrPktlibMCB,
    Ti_Pkt*         pPkt,
    uint8_t         followLinks
)
{
    Ti_Pkt*                         pDonorPacket;
    Ti_Pkt*                         pNextPacket;
    Pktlib_Info*                    ptrPktLibInfo;
    uint16_t                        refCount;
    void*                           csHandle;
    Pktlib_Heap*                    ptrPktHeap;
    Osal_PktlibEnterCriticalSection enterCS;
    Osal_PktlibExitCriticalSection  exitCS;
    Osal_PktlibEndPktAccess         endPktAccess;

    /* Store all the OSAL call functions which will be used in the function */
    enterCS      = ptrPktlibMCB->cfg.enterCS;
    exitCS       = ptrPktlibMCB->cfg.exitCS;
    endPktAccess = ptrPktlibMCB->cfg.endPktAccess;

    /* Cycle through all the packets which have been chained together. */
    while (pPkt != NULL)
    {
        /* Get the library information for the packet. */
        ptrPktLibInfo = Pktlib_getPktLibInfo(pPkt);

        /* Get the next packet only if we need to follow the links */
        if (followLinks == 1)
            pNextPacket = Pktlib_getNextPacket(pPkt);
        else
            pNextPacket = NULL;

        /* Is the packet cloned or not? */
        if (__Pktlib_getPacketCloneStatus(ptrPktLibInfo) == 1)
        {
            /* Cloned Packet: Can the packet get cleaned up or are there references held for the
             * cloned packet also? */
            ptrPktHeap = (Pktlib_Heap*)__Pktlib_getPktHeap(ptrPktLibInfo);
            csHandle   = enterCS(ptrPktHeap);
            refCount   = Pktlib_decRefCount(pPkt);
            exitCS(ptrPktHeap, csHandle);

            /* Can the cloned packet be deleted? */
            if (refCount == 0)
            {
                /* YES. We are trying to free a cloned packet. Get the information about the donor packet. */
                pDonorPacket = ptrPktLibInfo->donor;

                /* Ensure that we reset the data buffer and length to be 0 for cloned packets. */
                Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pPkt, NULL, 0);

                /* Cleanup the cloned packet: Ensure that we reset the cloning flag. */
                __Pktlib_setPacketCloneStatus(ptrPktLibInfo, 0);
                ptrPktLibInfo->donor = 0;

                /* Kill the links. */
                Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc *)pPkt, NULL);

                /* Packet is being cleaned up. Clean it up depending on the type of the packet. */
                if (Pktlib_isDataBufferPkt(pPkt) == 1)
                {
                    /* BUFFER Packet is being cleaned up. BUFFER packets can never be cloned only
                     * ZERO Buffer packets are cloned. So this implies that there is an error in the
                     * packet library. */
                    while (1);
                }
                else
                {
                    /* ZERO Buffer Packet is to be cleaned. Get the pointer to the heap */
                    ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(pPkt);

                    /* Set the return queue for ZERO Buffer packets to be the garbage queue. This
                     * could have been overriden by applications. */
                    Pktlib_cppiSetReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)pPkt, &ptrPktHeap->garbageQueueInfo);

                    /* Zero buffer packets can only be allocated by the DSP; there is no need for the DSP to
                     * writeback the packet since the ownership is still with the DSP. Push the packet to
                     * the ZERO Free queue. */
                    Qmss_queuePushDesc (ptrPktHeap->freeZeroQueueHnd, (void*)pPkt);
                }
            }
            else
            {
                /* NO. References to the cloned packet are still being held so this cannot be deleted. We need to
                 * skip this packet & proceed to the next linked packet. */
                pDonorPacket = NULL;

                /* The cloned packet reference counter has been decreased and this packet can still not be cleaned
                 * up. But the ownership of the packet is still held by the DSP. There is no need to writeback
                 * the packet here. */
            }
        }
        else
        {
            /* We are trying to free an original packet. In this case we are the donor itself */
            pDonorPacket = pPkt;
        }

        /* Do we need to clean this packet or not? */
        if (pDonorPacket != NULL)
        {
            /* Critical Section: Decrement the reference counter of the donor packet
             * if somebody was holding a reference using the heap interface table. */
            ptrPktLibInfo = Pktlib_getPktLibInfo(pDonorPacket);
            ptrPktHeap    = (Pktlib_Heap*)__Pktlib_getPktHeap(ptrPktLibInfo);
            csHandle      = enterCS(ptrPktHeap);
            refCount      = Pktlib_decRefCount(pDonorPacket);
            exitCS(ptrPktHeap, csHandle);

            /* If there are no references to the DONOR packet we can clean this immediately. */
            if (refCount == 0)
            {
                /* Kill the links. */
                Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc *)pDonorPacket, NULL);

                /* Reset the data buffer with the original data buffer information. */
                ((Cppi_HostDesc*)pDonorPacket)->buffPtr = ((Cppi_HostDesc*)pDonorPacket)->origBuffPtr;

                /* Determine the packet type */
                if (Pktlib_isDataBufferPkt(pDonorPacket) == 1)
                {
                    /* BUFFER Packet is being cleaned up. Ensure that the return queue for
                     * these packets is setup to point to the heap free queue. These could have
                     * been modified to point to the garbage queue. */
                    Pktlib_cppiSetReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)pDonorPacket, &ptrPktHeap->freeQueueInfo);

                    /* CACHE Fix: Each packet here is treated independently and we need to ensure that the packet
                     * is immediately written back immediately because it will get pushed into the queue so the
                     * ownership will immediately end */
                    __Pktlib_writebackPkt(pDonorPacket, ptrPktHeap, endPktAccess);

                    /* Push the packet into the corresponding free queue */
                    Qmss_queuePushDesc (ptrPktHeap->freeQueueHnd, (void*)pDonorPacket);
                }
                else
                {
                    /* ZERO BUFFER Packet is being cleaned up. Here we ensure that the packet
                     * is placed into the free zero buffer queue. We dont need to set the return
                     * queue since for ZERO Buffer packets it is always configured to be
                     * the garbage queue.
                     *
                     * Control comes here only when a zero buffer is allocated and is freed up
                     * without using it. For example: In the case of split with the split size
                     * being aligned on packet boudary.
                     *
                     * Zero buffer packets can only be allocated by the DSP; there is no need
                     * for the DSP to writeback the packet since the ownership is still with the DSP. */
                    Qmss_queuePushDesc (ptrPktHeap->freeZeroQueueHnd, (void*)pDonorPacket);
                }
            }
            else
            {
                /* The donor packet reference counter has been decreased and this packet can still
                 * not be cleaned up. But the ownership of the packet is still held by the DSP. There
                 * is no need to writeback the packet here. */
            }
        }

        /* Goto the next packet. */
        pPkt = pNextPacket;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the garbage collector for a specific heap. Packets
 *      which are cloned or split and then passed down to the IP blocks for
 *      transmission could end up in the heap free queue(s) and would be
 *      available for subsequent allocation; this might result in
 *      unpredictable behavior because the clones are still in use. Hence
 *      packets which have a non-zero reference count have their return
 *      queue modified to a garbage collection queue. The API needs to be
 *      called periodically to ensure that packets are moved back to the
 *      free queues from the garbage collection queue.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  heapHandle
 *      Handle of the heap on which the garbage collector is executed.
 *
  *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Pktlib_garbageCollection(Pktlib_InstHandle pktlibInstHandle, Pktlib_HeapHandle heapHandle)
{
    Pktlib_garbageCollectionWithCount(pktlibInstHandle, heapHandle, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is the garbage collector for a specific heap. Packets
 *      which are cloned or split and then passed down to the IP blocks for
 *      transmission could end up in the heap free queue(s) and would be
 *      available for subsequent allocation; this might result in
 *      unpredictable behavior because the clones are still in use. Hence
 *      packets which have a non-zero reference count have their return
 *      queue modified to a garbage collection queue. The API needs to be
 *      called periodically to ensure that packets are moved back to the
 *      free queues from the garbage collection queue.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  heapHandle
 *      Handle of the heap on which the garbage collector is executed.
 *  @param[in]  count
 *      Max amount of descriptors to be handled during one function call. 0 = all
 *
  *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Pktlib_garbageCollectionWithCount(Pktlib_InstHandle pktlibInstHandle,
                                       Pktlib_HeapHandle heapHandle,
                                       uint32_t const count)
{
    uint32_t                iter;
    Ti_Pkt*                 ptrPkt;
#ifdef __ARMv7
    Osal_PktlibPhyToVirt    phyToVirt;
#endif
    /* Validations: Ensure a valid heap handle was passed */
    Pktlib_Heap* ptrPktHeap   = (Pktlib_Heap*)heapHandle;
    /* Get the pointer to the PKTLIB instance */
    Pktlib_MCB*  ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    if (ptrPktlibMCB == NULL || ptrPktHeap == NULL)
        return;

#ifdef __ARMv7
    /* Get the physical to virtual OSAL callout function. */
    phyToVirt = ptrPktlibMCB->cfg.phyToVirt;
#endif

    /* Pop packets of the garbage queue. We use the RAW API to pop the packets. This is
     * because we dont want the Queue Pop to follow the links anymore. This is because
     * the packets have been disjointed by the CPDMA block after placing them into the
     * garbage queue but the links in the packet are still valid. If we invoke the original
     * pop API we run into the issue of following the links and converting all the packets
     * into virtual address. */
#ifdef __ARMv7
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(phyToVirt(Qmss_queuePopRaw(ptrPktHeap->garbageQueueHnd)));
#else
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrPktHeap->garbageQueueHnd));
#endif

    for (iter = 0; ((iter < count || count == 0) && ptrPkt != NULL); iter++) {
#ifdef __ARMv7
        Cppi_HostDesc*  ptrHostDesc = Pktlib_getDescFromPacket(ptrPkt);

        /* Convert the individual buffer and original buffer pointers to be virtual address. */
        ptrHostDesc->buffPtr     = (uint32_t)phyToVirt((void *)ptrHostDesc->buffPtr);
        ptrHostDesc->origBuffPtr = (uint32_t)phyToVirt((void *)ptrHostDesc->origBuffPtr);

        /* Kill the links in the packet. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, NULL);
#endif

        /* No need to invalidate the packet. The packet can enter the garbage collection
         * only if the packet was pushed by a CPDMA block. This implies that the ownership
         * of the packet was relinquished by the DSP. Since the CPDMA block will never
         * modify the descriptor there is no point in invalidating the packet here.
         *
         * Cleanup the packet: We dont follow the links here because these packets
         * have been placed here by the CPDMA which has placed all the chained packets
         * into their individual return queues but has NOT killed the links within
         * the descriptor. */
        __Pktlib_freePacket(ptrPktlibMCB, ptrPkt, 0);

#ifdef __ARMv7
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(phyToVirt(Qmss_queuePopRaw(ptrPktHeap->garbageQueueHnd)));
#else
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrPktHeap->garbageQueueHnd));
#endif
    }
}


/**
 *  @b Description
 *  @n
 *      The function is used to cleanup a packet.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  pPkt
 *      Pointer to the packet.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_freePacket
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pPkt
)
{
    /* Cleanup the packet: This is called by the software to clean the packet
     * so here we need to follow the links and place all the chained packets into
     * the correct heap free queues. */
    __Pktlib_freePacket (pktlibInstHandle, pPkt, 1);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the MAX data size associated with a heap.
 *      During heap creation the size of the data buffers associated with the
 *      heap are specified. The function returns the size allocated. If the heap
 *      was created with no data buffers (only with zero data buffers) the
 *      function will return 0 else it will return the size of the data buffer.
 *
 *  @param[in]  heapHandle
 *      Handle of the heap from where the packet is to be allocated.
 *
  *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Size of the data buffer.
 */
uint32_t Pktlib_getMaxBufferSize(Pktlib_HeapHandle heapHandle)
{
    Pktlib_Heap* ptrPktHeap;

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* Ensure that a valid heap handle was passed. */
    if (ptrPktHeap == NULL)
        return 0;

    /* Return the size of the data buffer. */
    return ptrPktHeap->dataBufferSize;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a packet of the specified size from the
 *      super heap. If the size passed is 0; the function allocates a packet with no
 *      buffer i.e. bufferless packet. If the size passed is greater than the
 *      size of the data packet in the heap then the allocation will fail.
 *
 *  @param[in]  ptrPktlibMCB
 *      Pointer to the PKTLIB instance
 *  @param[in]  ptrSuperHeap
 *      Pointer to the super heap from which the packet is to be allocated
 *  @param[in]  size
 *      Size of the packet which is to be allocated.
 *
 *  \ingroup PKTLIB_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the packet
 *  @retval
 *      Error   -   NULL
 */
static Ti_Pkt* Pktlib_superHeapAllocPacket
(
    Pktlib_MCB*     ptrPktlibMCB,
    Pktlib_Heap*    ptrSuperHeap,
    uint32_t        size
)
{
    uint32_t    index;
    Ti_Pkt*     ptrPkt = NULL;

    /* Cycle through all the member heaps */
    for (index = 0; index < PKTLIB_MAX_SUPER_MANAGED_HEAPS; index++)
    {
        /* Allocate the packet using the member heap */
        ptrPkt = Pktlib_allocPacket(ptrPktlibMCB, ptrSuperHeap->memberHeapHandles[index], size);
        if (ptrPkt != NULL)
            return ptrPkt;

        /* Allocation failed; this could happen for a number of reasons
         *  (1) The data buffer size requested was greater than the heap buffer size
         *  (2) There were no packets available in the heap
         * For either of these cases we should basically just move on and try and use
         * the next member heap */
        continue;
    }

    /* Control comes here implies that the SUPER Heap was NOT able to satisfy the request */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a packet of the specified size from the
 *      heap. If the size passed is 0; the function allocates a packet with no
 *      buffer i.e. bufferless packet. If the size passed is greater than the
 *      size of the data packet in the heap then the allocation will fail.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  heapHandle
 *      Handle of the heap from where the packet is to be allocated.
 *  @param[in]  size
 *      Size of the packet which is to be allocated.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the packet
 *  @retval
 *      Error   -   NULL
 */
Ti_Pkt* Pktlib_allocPacket
(
    Pktlib_InstHandle   pktlibInstHandle,
    Pktlib_HeapHandle   heapHandle,
    uint32_t            size
)
{
    Pktlib_Heap*            ptrPktHeap;
    Ti_Pkt*                 ptrPkt;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataLen;
    Qmss_QueueHnd           queueHandle;
    Pktlib_MCB*             ptrPktlibMCB;
#ifdef __ARMv7
    Osal_PktlibPhyToVirt    phyToVirt;
#endif

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* Get the pointer to the PKTLIB instance */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Is this a Super heap? */
    if (ptrPktHeap->isSuperHeap == 1)
        return Pktlib_superHeapAllocPacket(ptrPktlibMCB, ptrPktHeap, size);

#ifdef __ARMv7
    /* Get the physical to virtual OSAL callout function. */
    phyToVirt = ptrPktlibMCB->cfg.phyToVirt;
#endif

    /* Check the size to determine the type of allocation?
     *  - Use the Zero Queue if the size is 0.
     *  - Use the data queue if the size is within the heap size.
     *  - Else the heap cannot satisfy the request */
    if (size == 0)
    {
        /* Zero Buffer Allocation: */
        queueHandle = ptrPktHeap->freeZeroQueueHnd;
    }
    else if (size <= ptrPktHeap->dataBufferSize)
    {
        /* Normal Buffer Allocation: */
        queueHandle = ptrPktHeap->freeQueueHnd;
    }
    else
    {
        /* Error: Request cannot be satisfied. */
        queueHandle = 0;
    }

    /* Allocation can proceed only if a valid queue handle was present.
     *  - This is a catch all because we can have a zero buffer allocation
     *    being attempted on a heap which was not configured for zero buffers */
    if (queueHandle == 0)
        return NULL;

    /* Pop off a packet from the queue. */
#ifdef __ARMv7
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(phyToVirt(Qmss_queuePopRaw(queueHandle)));
#else
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(queueHandle));
#endif

    /* Did we get a packet. */
    if (ptrPkt != NULL)
    {
#ifdef __ARMv7
        Cppi_HostDesc*  ptrDesc = (Cppi_HostDesc *)ptrPkt;
        if (ptrDesc->buffPtr)
        {
            ptrDesc->buffPtr = (uint32_t)phyToVirt((void *)(ptrDesc->buffPtr));
            if (!(ptrDesc->buffPtr)) return (void *)0;
        }

        if (ptrDesc->origBuffPtr)
        {
            ptrDesc->origBuffPtr = (uint32_t)phyToVirt((void *)(ptrDesc->origBuffPtr));
            if (!(ptrDesc->origBuffPtr)) return (void *)0;
        }
#endif

        /* Get the original buffer information. */
        Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt,
                                 (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Ensure that the original buffer and buffer address is one and the same. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, (uint8_t*)ptrDataBuffer, size);

        /* Kill any other links to the packets. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, NULL);

        /* Reset the PS Flags in the packet: */
        Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, 0x0);

        /* Setup the packet length  */
        Pktlib_setPacketLen(ptrPkt, size);
    }

    /* Return the allocated packet. */
    return ptrPkt;
}

/**
 *  @b Description
 *  @n
 *      The function is used to merge packets. The function chains "Packet2"
 *      to the end of "Packet1".
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  pPkt1
 *      Packet1 which is to be merged at the beginning of packet2
 *  @param[in]  pPkt2
 *      Packet2 which is merged at the end of packet1
 *  @param[in]  pLastPkt
 *      Optional parameter to the last element of the Packet1 chain. If this
 *      is known then packet2 is chained to the end of this packet; else the
 *      API will try and determine the last element in the Packet1 chain
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Merged packet
 */
Ti_Pkt* Pktlib_packetMerge
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pPkt1,
    Ti_Pkt*             pPkt2,
    Ti_Pkt*             pLastPkt
)
{
    Cppi_HostDesc*  ptrPrevDesc = NULL;
    Cppi_HostDesc*  ptrDesc;
    uint32_t        packetLength;

    /* Validations: Ensure that the packets passed to be merged are correct */
    if ((pPkt1 == NULL) || (pPkt2 == NULL))
        return NULL;

    /* Is the last packet in the chain specified? */
    if (pLastPkt == NULL)
    {
        /* NO. Get the pointer to the descriptor. */
        ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)pPkt1);
        while (ptrDesc != NULL)
        {
            /* Store the previous descriptor. */
            ptrPrevDesc = ptrDesc;

            /* Get the pointer to the next descriptor. */
            ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)ptrDesc);
        }
    }
    else
    {
        /* YES. Get the last element in the chain. */
        ptrPrevDesc = (Cppi_HostDesc*)pLastPkt;
    }

    /* Link the last buffer descriptor of packet1 with the packet2. If packet1 had only 1
     * descriptor we link the head descriptor of packet1 with packet2 else we link the
     * last descriptor from packet1 to the head descriptor of packet2 */
    if (ptrPrevDesc == NULL)
    {
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pPkt1, (Cppi_Desc*)pPkt2);
    }
    else
    {
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)ptrPrevDesc, (Cppi_Desc*)pPkt2);
    }

    /* Compute the total packet length after merging the packets. */
    packetLength = Pktlib_getPacketLen(pPkt1) + Pktlib_getPacketLen (pPkt2);

    /* Ensure that the packet length is properly setup in the packet1 to account for the total length */
    Pktlib_setPacketLen(pPkt1, packetLength);

    /* Reset the packet length in the packet2 because this has now been merged and the
     * length has already been accounted for */
    Pktlib_setPacketLen (pPkt2, 0);

    /* Return the merged packet. */
    return (Ti_Pkt*)(pPkt1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to clone a packet.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  ptrPktOrig
 *      Original Packet which is to be cloned.
 *  @param[in]  ptrClonePacket
 *      The pointer to the cloned packet which is a list of packets with
 *      zero buffers. There should be the same number of buffers as the
 *      original packet.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   PKTLIB error code
 */
int32_t Pktlib_clonePacket
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             ptrPktOrig,
    Ti_Pkt*             ptrClonePacket
)
{
    Pktlib_Info*                        pOrigPktLibInfo;
    Pktlib_Info*                        pClonePktLibInfo;
    uint8_t*                            ptrDataBuffer;
    uint32_t                            dataLen;
    uint32_t                            packetLength;
    Pktlib_Heap*                        ptrPktHeap;
    void*                               csHandle;
    uint16_t                            refCount;
    Pktlib_MCB*                         ptrPktlibMCB;
    Osal_PktlibEnterCriticalSection     enterCS;
    Osal_PktlibExitCriticalSection      exitCS;

    /* Get the PKTLIB instance MCB */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Store all the OSAL call functions which will be used in the function */
    enterCS      = ptrPktlibMCB->cfg.enterCS;
    exitCS       = ptrPktlibMCB->cfg.exitCS;

    /* Get the packet length of the original packet. */
    packetLength = Pktlib_getPacketLen (ptrPktOrig);

    /* Ensure that the cloned packet inherits the same packet length. */
    Pktlib_setPacketLen (ptrClonePacket, packetLength);

    /* Cycle through the chain of packets */
    while (ptrPktOrig != NULL)
    {
        /* We should always have a packet in the cloned chain. If not we are in trouble because
         * we will not be able to clone the packet. */
        if (ptrClonePacket == NULL)
            return PKTLIB_EINVAL;

        /* Get the packet lib info of the original & cloned packet. */
        pOrigPktLibInfo  = Pktlib_getPktLibInfo(ptrPktOrig);
        pClonePktLibInfo = Pktlib_getPktLibInfo(ptrClonePacket);

        /* All cloned packets should be 0 buffer packets. If these are buffer packets then
         * the callee has made a mistake and we cannot proceed. */
        if (Pktlib_isDataBufferPkt(ptrClonePacket) == 1)
            return PKTLIB_EINVAL;

        /* Set the flags in the cloned packet appropriately. */
        __Pktlib_setPacketCloneStatus(pClonePktLibInfo, 1);

        /* Ensure that we configure the return queue for the original packet to be the
         * garbage queue because we dont want these packets to be placed immediately
         * into the free queue (by the IP blocks) once they are done because we might have
         * other references. Note: The return queue for cloned packets is the GARBAGE queue
         * by default. */
        Pktlib_setReturnQueueToGarbage(ptrPktOrig);

        /* Determine the donor of the packet: This depends upon the packet type of the original */
        if (__Pktlib_getPacketCloneStatus(pOrigPktLibInfo) == 1)
        {
            /* The cloned packet is being cloned. So we set the cloned packet donor to
             * the donor of the original packet. */
            pClonePktLibInfo->donor = pOrigPktLibInfo->donor;
        }
        else
        {
            /* The original packet is being cloned. So we set the cloned packet donor to
             * the original packet. */
            pClonePktLibInfo->donor = ptrPktOrig;
        }

        /* Get the current buffer & buffer length of the original packet. */
        Pktlib_getDataBuffer(ptrPktOrig, &ptrDataBuffer, &dataLen);

        /* Set this in the cloned packet. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrClonePacket, ptrDataBuffer, dataLen);

        /* Critical Section Start: Incrementing the reference counter. */
        ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(ptrClonePacket);
        csHandle   = enterCS (ptrPktHeap);

        /* Increment the reference counter if allowed */
        refCount = Pktlib_incRefCount(pClonePktLibInfo->donor);

        /* Critical Section End: */
        exitCS (ptrPktHeap, csHandle);

        /* If we exceed the MAX reference count allowed there is no point continuing further */
        if (refCount >= PKTLIB_MAX_REF_COUNT)
        {
            /* Error: We need to undo the modifications. The packet was not cloned successfully. */
            __Pktlib_setPacketCloneStatus(pClonePktLibInfo, 0);
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrClonePacket, NULL, 0);
            pClonePktLibInfo->donor = 0;
            return PKTLIB_ELIMIT;
        }

        /* Get the next packet (original & Clone) */
        ptrPktOrig      = Pktlib_getNextPacket(ptrPktOrig);
        ptrClonePacket  = Pktlib_getNextPacket(ptrClonePacket);
    }

    /* Packet has been successfully cloned. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to split the packet. Splitting a packet
 *      is done at the 'splitPacketSize' specified. The head of the
 *      split packet is returned in 'pPkt1' while the head of the
 *      second split is returned in 'pPkt2'.
 *
 *      The zero buffer packet is marked as a clone because it refers to
 *      the data buffer packet. Data buffer packets cannot be freed till
 *      all the clones using them have been freed. The goal should be to
 *      free the CLONED packets as soon as possible so that the data buffer
 *      packets are not being held.
 *
 *      This variant of the function splits the packet and sets the zero
 *      buffer packet (i.e. Cloned Packet) to point to the data *before* the
 *      split size and so it is returned in the pPkt1. So use this function
 *      if the pPkt1 is getting cleaned before pPkt2.
 *
 *  @sa
 *      Pktlib_splitPacket2
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  pOrgPkt
 *      Pointer to the original packet which is to be split.
 *  @param[in]  pNewPkt
 *      Pointer to the packet which has no buffer attached to it.
 *  @param[in]  splitPacketSize
 *      Size of the packet which is to be split
 *  @param[out] pPkt1
 *      The pointer to the first packet after the split.
 *  @param[out] pPkt2
 *      The pointer to the second packet after the split.
 *
 *  @retval
 *      0   -   Success (The packet passed was used)
 *  @retval
 *      1   -   Success (The packet passed was NOT used)
 *  @retval
 *      <0  -   PKTLIB error code
 */
int32_t Pktlib_splitPacket
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pOrgPkt,
    Ti_Pkt*             pNewPkt,
    uint32_t            splitPacketSize,
    Ti_Pkt**            pPkt1,
    Ti_Pkt**            pPkt2
)
{
    uint32_t                            packetLength;
    Ti_Pkt*                             pSplitPkt;
    Ti_Pkt*                             pPrevSplitPkt = NULL;
    uint8_t*                            ptrDataBuffer;
    Pktlib_MCB*                         ptrPktlibMCB;
    Pktlib_Heap*                        ptrPktHeap;
    Osal_PktlibEnterCriticalSection     enterCS;
    Osal_PktlibExitCriticalSection      exitCS;

    /* Validations: Ensure that the packets passed to be split are valid */
    if ((pOrgPkt == NULL) || (pNewPkt == NULL))
        return PKTLIB_EINVAL;

    /* Get the PKTLIB instance MCB */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Store all the OSAL call functions which will be used in the function */
    enterCS      = ptrPktlibMCB->cfg.enterCS;
    exitCS       = ptrPktlibMCB->cfg.exitCS;

    /* Get the packet length of the packet which is to be split. */
    packetLength = Pktlib_getPacketLen (pOrgPkt);

    /* Validations: We cannot split a packet if the packet length is less than the split size */
    if (packetLength < splitPacketSize)
        return PKTLIB_EINVAL;

    /* If we are trying to split the packet into the same size as the current length
     * than this function is a no operation. Our work is already done */
    if (packetLength == splitPacketSize)
    {
        /* The size is the same. So the split packet1 is the same as the original packet
         * while the split packet2 is the NULL. */
        *pPkt1 = pOrgPkt;
        *pPkt2 = NULL;

        /* We have not used the packet which was passed to us in this case. */
        return 1;
    }

    /* Cycle through the descriptors till we can determine where the split has to be done. */
    pSplitPkt    = pOrgPkt;
    packetLength = 0;
    while (pSplitPkt != NULL)
    {
        /* Get the data buffer length. */
        packetLength = packetLength + Pktlib_getDataBufferLen(pSplitPkt);

        /* Have we reached the split length specified? */
        if (packetLength >= splitPacketSize)
            break;

        /* Remember the previous split packet. */
        pPrevSplitPkt = pSplitPkt;

        /* Get the next packet. */
        pSplitPkt = Pktlib_getNextPacket(pSplitPkt);
    }

    /* Did we break out if we have reached the end of the chain of packets. This case should not occur
     * the only possibility for this is that the packet length in the packet was messed up. */
    if (pSplitPkt == NULL)
        return PKTLIB_EINTERNAL;

    /* Handle the following cases:
     *  a) The split is at the end of the data buffer in the split packet
     *  b) The split is in the middle of the data buffer in the split packet. */
    if (packetLength == splitPacketSize)
    {
        /* This is case (a) above. Update the links to the split packets. */
        *pPkt1 = pOrgPkt;
        *pPkt2 = Pktlib_getNextPacket(pSplitPkt);

        /* Kill the next link in the split packet. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pSplitPkt, (Cppi_Desc*)NULL);

        /* Update the packet lengths in both the split packets.
         *  The split packet1 has a packet length which is the size specified in the API.
         *  The second split packet has whatever is left. */
        packetLength = Pktlib_getPacketLen(pOrgPkt) - splitPacketSize;
        Pktlib_setPacketLen ((*pPkt2), packetLength);
        Pktlib_setPacketLen ((*pPkt1), splitPacketSize);

        /* The split operation was successful but we did not use the packet which was passed to us. */
        return 1;
    }

    /* This is case (b).
     *  Start populating the new packet which was passed to us. */
    {
        uint32_t        dataBufferLen1;
        uint32_t        dataBufferLen2;
        Ti_Pkt*         ptrDonorPkt;
        uint32_t        orgDataBufferLen;
        Pktlib_Info*    pNewPktLibInfo;
        Pktlib_Info*    pSplitPktLibInfo;
        void*           csHandle;
        uint16_t        refCount;

        /* Get the heap to which the new packet belongs. */
        ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(pNewPkt);

        /* Get the packet lib info of the original & cloned packet. */
        pSplitPktLibInfo  = Pktlib_getPktLibInfo(pSplitPkt);
        pNewPktLibInfo    = Pktlib_getPktLibInfo(pNewPkt);

        /* Determine the donor of the packet: This depends upon the packet type of the original */
        if (__Pktlib_getPacketCloneStatus(pSplitPktLibInfo) == 1)
        {
            /* The cloned packet is being cloned. So we set the cloned packet donor to
             * the donor of the original packet. */
            ptrDonorPkt = pSplitPktLibInfo->donor;
        }
        else
        {
            /* The original packet is being cloned. So we set the cloned packet donor to
             * the original packet. */
            ptrDonorPkt = pSplitPkt;
        }

        /* Critical Section Start: Incrementing the reference counter. */
        csHandle = enterCS(ptrPktHeap);

        /* Increment the reference counter if allowed */
        refCount = Pktlib_incRefCount(ptrDonorPkt);

        /* Critical Section End: */
        exitCS(ptrPktHeap, csHandle);

        /* Are we allowed to proceed with splitting the packet? */
        if (refCount == PKTLIB_MAX_REF_COUNT)
            return PKTLIB_ELIMIT;

        /* We have now determined the descriptor on which the split will be done. Now we need
         * to determine the location in the buffer where the split is done. */
        Pktlib_getDataBuffer(pSplitPkt, &ptrDataBuffer, &orgDataBufferLen);

        /* Determine the lengths of the 2 data buffers accounting for the split. */
        dataBufferLen1 = splitPacketSize  - (packetLength - orgDataBufferLen);
        dataBufferLen2 = orgDataBufferLen - dataBufferLen1;

        /* The Split packet is now broken into 2 parts; the first part of the buffer is referenced by the
         * Zero Buffer Packet; the second part is referenced as is by the Split Packet
         *
         * Initialize the Zero Buffer Packet appropriately. */
        Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)pNewPkt, (uint8_t*)ptrDataBuffer, dataBufferLen1);

        /* Update the split packet appropriately. */
        Cppi_setData(Cppi_DescType_HOST, (Cppi_Desc*)pSplitPkt, (uint8_t*)(ptrDataBuffer + dataBufferLen1),
                     dataBufferLen2);

        /* Set the flags in the cloned packet appropriately. */
        __Pktlib_setPacketCloneStatus (pNewPktLibInfo, 1);

        /* Remember the DONOR. */
        pNewPktLibInfo->donor = ptrDonorPkt;

        /* Ensure that we configure the return queue for the split packet to be the
         * garbage queue because we dont want these packets to be placed immediately
         * into the free queue (by the IP blocks) once they are done because we might have
         * other references. Note: The return queue for 'new' packets is the GARBAGE queue
         * by default. */
        Pktlib_setReturnQueueToGarbage(pSplitPkt);
    }

    /* Get the packet length of the original packet. */
    packetLength = Pktlib_getPacketLen (pOrgPkt);

    /* Link the Zero Buffer Descriptor. */
    if (pPrevSplitPkt != NULL)
    {
        /* Link the packet previous to the split packet with the new packet */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pPrevSplitPkt, (Cppi_Desc*)pNewPkt);

        /* Populate the split packet information to return back. */
        *pPkt1 = pOrgPkt;
        *pPkt2 = pSplitPkt;
    }
    else
    {
        /* Populate the split packet information to return back. */
        *pPkt1 = pNewPkt;
        *pPkt2 = pSplitPkt;
    }

    /* Set the packet length correctly in the returned packets. */
    Pktlib_setPacketLen ((*pPkt1), splitPacketSize);
    Pktlib_setPacketLen ((*pPkt2), (packetLength - splitPacketSize));

    /* The packet has been split successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to split the packet. Splitting a packet
 *      is done at the 'splitPacketSize' specified. The head of the
 *      split packet is returned in 'pPkt1' while the head of the
 *      second split is returned in 'pPkt2'.
 *
 *      The zero buffer packet is marked as a clone because it refers to
 *      the data buffer packet. Data buffer packets cannot be freed till
 *      all the clones using them have been freed. The goal should be to
 *      free the CLONED packets as soon as possible so that the data buffer
 *      packets are not being held.
 *
 *      This variant of the function splits the packet and sets the zero
 *      buffer packet (i.e. Cloned Packet) to point to the data *after* the
 *      split size and so it is returned in the pPkt2. So use this function
 *      if the pPkt2 is getting cleaned before pPkt1.
 *
 *  @sa
 *      Pktlib_splitPacket
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  pOrgPkt
 *      Pointer to the original packet which is to be split.
 *  @param[in]  pNewPkt
 *      Pointer to the packet which has no buffer attached to it.
 *  @param[in]  splitPacketSize
 *      Size of the packet which is to be split
 *  @param[out] pPkt1
 *      The pointer to the first packet after the split.
 *  @param[out] pPkt2
 *      The pointer to the second packet after the split.
 *
 *  @retval
 *      0   -   Success (The packet passed was used)
 *  @retval
 *      1   -   Success (The packet passed was NOT used)
 *  @retval
 *      -1  -   Error
 */
int32_t Pktlib_splitPacket2
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             pOrgPkt,
    Ti_Pkt*             pNewPkt,
    uint32_t            splitPacketSize,
    Ti_Pkt**            pPkt1,
    Ti_Pkt**            pPkt2
)
{
    uint32_t                            packetLength;
    Ti_Pkt*                             pSplitPkt;
    uint8_t*                            ptrDataBuffer;
    Pktlib_MCB*                         ptrPktlibMCB;
    Pktlib_Heap*                        ptrPktHeap;
    Osal_PktlibEnterCriticalSection     enterCS;
    Osal_PktlibExitCriticalSection      exitCS;

    /* Validations: Ensure that the packets passed to be split are valid */
    if ((pOrgPkt == NULL) || (pNewPkt == NULL))
        return PKTLIB_EINVAL;

    /* Get the PKTLIB instance MCB */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;

    /* Store all the OSAL call functions which will be used in the function */
    enterCS      = ptrPktlibMCB->cfg.enterCS;
    exitCS       = ptrPktlibMCB->cfg.exitCS;

    /* Get the packet length of the packet which is to be split. */
    packetLength = Pktlib_getPacketLen(pOrgPkt);

    /* Validations: We cannot split a packet if the packet length is less than the split size */
    if (packetLength < splitPacketSize)
        return PKTLIB_EINVAL;

    /* If we are trying to split the packet into the same size as the current length
     * than this function is a no operation. Our work is already done */
    if (packetLength == splitPacketSize)
    {
        /* The size is the same. So the split packet1 is the same as the original packet
         * while the split packet2 is the NULL. */
        *pPkt1 = pOrgPkt;
        *pPkt2 = NULL;

        /* We have not used the packet which was passed to us in this case. */
        return 1;
    }

    /* Cycle through the descriptors till we can determine where the split has to be done. */
    pSplitPkt    = pOrgPkt;
    packetLength = 0;
    while (pSplitPkt != NULL)
    {
        /* Get the data buffer length. */
        packetLength = packetLength + Pktlib_getDataBufferLen(pSplitPkt);

        /* Have we reached the split length specified? */
        if (packetLength >= splitPacketSize)
            break;

        /* Get the next packet. */
        pSplitPkt = Pktlib_getNextPacket(pSplitPkt);
    }

    /* Did we break out if we have reached the end of the chain of packets. This case should not occur
     * the only possibility for this is that the packet length in the packet was messed up. */
    if (pSplitPkt == NULL)
        return PKTLIB_EINTERNAL;

    /* Handle the following cases:
     *  a) The split is at the end of the data buffer in the split packet
     *  b) The split is in the middle of the data buffer in the split packet. */
    if (packetLength == splitPacketSize)
    {
        /* This is case (a) above. Update the links to the split packets. */
        *pPkt1 = pOrgPkt;
        *pPkt2 = Pktlib_getNextPacket(pSplitPkt);

        /* Kill the next link in the split packet. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pSplitPkt, (Cppi_Desc*)NULL);

        /* Update the packet lengths in both the split packets.
         *  The split packet1 has a packet length which is the size specified in the API.
         *  The second split packet has whatever is left. */
        packetLength = Pktlib_getPacketLen (pOrgPkt) - splitPacketSize;
        Pktlib_setPacketLen ((*pPkt2), packetLength);
        Pktlib_setPacketLen ((*pPkt1), splitPacketSize);

        /* The split operation was successful but we did not use the packet which was passed to us. */
        return 1;
    }

    /* This is case (b).
     *  Start populating the new packet which was passed to us. */
    {
        uint32_t        dataBufferLen1;
        uint32_t        dataBufferLen2;
        Ti_Pkt*         ptrDonorPkt;
        uint32_t        orgDataBufferLen;
        Pktlib_Info*    pNewPktLibInfo;
        Pktlib_Info*    pSplitPktLibInfo;
        void*           csHandle;
        uint16_t        refCount;

        /* Get the heap to which the new packet belongs. */
        ptrPktHeap = (Pktlib_Heap*)Pktlib_getPktHeap(pNewPkt);

        /* Get the packet lib info of the original & cloned packet. */
        pSplitPktLibInfo  = Pktlib_getPktLibInfo(pSplitPkt);
        pNewPktLibInfo    = Pktlib_getPktLibInfo(pNewPkt);

        /* Determine the donor of the packet: This depends upon the packet type of the original */
        if (__Pktlib_getPacketCloneStatus(pSplitPktLibInfo) == 1)
        {
            /* The cloned packet is being cloned. So we set the cloned packet donor to
             * the donor of the original packet. */
            ptrDonorPkt = pSplitPktLibInfo->donor;
        }
        else
        {
            /* The original packet is being cloned. So we set the cloned packet donor to
             * the original packet. */
            ptrDonorPkt = pSplitPkt;
        }

        /* Critical Section Start: Incrementing the reference counter. */
        csHandle = enterCS(ptrPktHeap);

        /* Increment the reference count if we are allowed to do so */
        refCount = Pktlib_incRefCount(ptrDonorPkt);

        /* Critical Section End: */
        exitCS(ptrPktHeap, csHandle);

        /* Are we allowed to proceed with the split */
        if (refCount == PKTLIB_MAX_REF_COUNT)
            return PKTLIB_ELIMIT;

        /* We have now determined the descriptor on which the split will be done. Now we need
         * to determine the location in the buffer where the split is done. */
        Pktlib_getDataBuffer(pSplitPkt, &ptrDataBuffer, &orgDataBufferLen);

        /* Determine the lengths of the 2 data buffers accounting for the split. */
        dataBufferLen1 = splitPacketSize  - (packetLength - orgDataBufferLen);
        dataBufferLen2 = orgDataBufferLen - dataBufferLen1;

        /* The Split packet is now broken into 2 parts;
         *  - The first part of the packet is referred to by split packet
         *  - The second part of the packet is referred by the Zero Buffer Packet
         *
         * Initialize the Zero Buffer Packet appropriately. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)pNewPkt, (uint8_t*)(ptrDataBuffer + dataBufferLen1), dataBufferLen2);

        /* The zero buffer packet inherits all the links from the original split packet. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pNewPkt, (Cppi_Desc*)Pktlib_getNextPacket(pSplitPkt));

        /* Set the data buffer length in the split packet descriptor correctly. */
        Cppi_setDataLen(Cppi_DescType_HOST, (Cppi_Desc*)pSplitPkt, dataBufferLen1);

        /* Kill the links in the split packet. */
        Cppi_linkNextBD (Cppi_DescType_HOST, (Cppi_Desc*)pSplitPkt, NULL);

        /* Set the flags in the cloned packet appropriately. */
        __Pktlib_setPacketCloneStatus(pNewPktLibInfo, 1);

        /* Remember the DONOR. */
        pNewPktLibInfo->donor = ptrDonorPkt;

        /* Ensure that we configure the return queue for the split packet to be the
         * garbage queue because we dont want these packets to be placed immediately
         * into the free queue (by the IP blocks) once they are done because we might have
         * other references. Note: The return queue for 'new' packets is the GARBAGE queue
         * by default. */
        Pktlib_setReturnQueueToGarbage(pSplitPkt);
    }

    /* Get the packet length of the original packet. */
    packetLength = Pktlib_getPacketLen (pOrgPkt);

    /* The first packet always starts from the original packet. */
    *pPkt1 = pOrgPkt;

    /* The second packet is always the cloned packet which is after the split. */
    *pPkt2 = pNewPkt;

    /* Set the packet length correctly in the returned packets. */
    Pktlib_setPacketLen ((*pPkt1), splitPacketSize);
    Pktlib_setPacketLen ((*pPkt2), (packetLength - splitPacketSize));

    /* The packet has been split successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to copy meta data from the source packet into the destination
 *      packet. Meta-Data in the packet implies the EPIB and Protocol specific information
 *      in the packet.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle
 *  @param[in]  ptrPktDst
 *      Destination packet to which the meta data is to be copied.
 *  @param[in]  ptrPktSrc
 *      Source packet from where the meta data is to be copied
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   PKTLIB error code
 */
int32_t Pktlib_copyMetaData
(
    Pktlib_InstHandle   pktlibInstHandle,
    Ti_Pkt*             ptrPktDst,
    Ti_Pkt*             ptrPktSrc
)
{
    Pktlib_MCB*     ptrPktlibMCB;
    uint8_t         isEPIBPresent;
    uint8_t         isPSDataPresent;
    uint8_t*        ptrPSInfo;
    uint32_t        psInfoLen;
    Cppi_HostDesc*  ptrSrcHostDesc;
    Cppi_HostDesc*  ptrDstHostDesc;
    Cppi_Result     result;
    uint32_t        timeStamp;
    uint8_t         packetType;
    uint32_t        psFlags;
    uint32_t*       ptrSwInfo = NULL;

    /* Get the PKTLIB instance MCB */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;
    if (ptrPktlibMCB == NULL)
        return PKTLIB_EINVAL;

    /* We need to have a source & destination packet: */
    if ((ptrPktDst == NULL) || (ptrPktSrc == NULL))
        return PKTLIB_EINVAL;

    /* Get the pointer to the source & destination host descriptors */
    ptrSrcHostDesc = (Cppi_HostDesc*)ptrPktSrc;
    ptrDstHostDesc = (Cppi_HostDesc*)ptrPktDst;

    /* Get the PS Information from the source: */
    result = Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)ptrSrcHostDesc, (uint8_t **)&ptrPSInfo, &psInfoLen);
    if (result == CPPI_PSDATA_NOT_PRESENT)
        isPSDataPresent = 0;
    else
        isPSDataPresent = 1;

    /* Get the EPIB Information from the source: */
    if (Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrSrcHostDesc, &timeStamp) == CPPI_EPIB_NOT_PRESENT)
        isEPIBPresent = 0;
    else
        isEPIBPresent = 1;

    /* Is the EPIB information present? */
    if (isEPIBPresent == 1)
    {
        /* YES: The EPIB Information constitutes the Timestamp & Software information fields. We have already retrieved the timestamp
         * while trying to determine if the PS information is present or not. Now we get the software information */
        Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrSrcHostDesc, (uint8_t**)&ptrSwInfo);

        /* Copy over the fields */
        Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, *ptrSwInfo);
        Cppi_setSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, *(ptrSwInfo + 1));
        Cppi_setSoftwareInfo2 (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, *(ptrSwInfo + 2));
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, timeStamp);
    }

    /* Is the PS information present? */
    if (isPSDataPresent == 1)
        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, (uint8_t*)ptrPSInfo, psInfoLen);

    /* Copy over packet type */
    packetType = Cppi_getPacketType (Cppi_DescType_HOST, (Cppi_Desc *)ptrSrcHostDesc);
    Cppi_setPacketType (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, packetType);

    /* Copy over the protocol specific flags */
    psFlags = Cppi_getPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)ptrSrcHostDesc);
    Cppi_setPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)ptrDstHostDesc, psFlags);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to return the internal heap queue where the
 *      data buffer packets are stored.
 *
 *  @param[in]  heapHandle
 *      Heap handle.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      QMSS Queue Handle where buffer packets are stored. This could be
 *      NULL if the heap was created with no buffer packets or if this was
 *      a SUPER Heap.
 */
Qmss_QueueHnd Pktlib_getInternalHeapQueue(Pktlib_HeapHandle heapHandle)
{
    Pktlib_Heap* ptrPktHeap;

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* This API is not supported on the Super Heap */
    if (ptrPktHeap->isSuperHeap == 1)
        return 0;

    /* Return the Data Buffer Queue. */
    return ptrPktHeap->freeQueueHnd;
}

/**
 *  @b Description
 *  @n
 *      The function is used to return the internal heap queue where the
 *      zero buffer packets are stored.
 *
 *  @param[in]  heapHandle
 *      Heap handle.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      QMSS Queue Handle where zero packets are stored. This could be
 *      NULL if the heap was created with no zero buffer packets or if
 *      this is a SUPER HEAP.
 */
Qmss_QueueHnd Pktlib_getZeroHeapQueue(Pktlib_HeapHandle heapHandle)
{
    Pktlib_Heap* ptrPktHeap;

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* This API is not supported on the Super Heap */
    if (ptrPktHeap->isSuperHeap == 1)
        return 0;

    /* Return the Zero Buffer Queue. */
    return ptrPktHeap->freeZeroQueueHnd;
}

Qmss_QueueHnd Pktlib_getGarbageHeapQueue(Pktlib_HeapHandle heapHandle)
{
    Pktlib_Heap* ptrPktHeap;

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* Return the Garbage Buffer Queue. */
    return ptrPktHeap->garbageQueueHnd;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the version information of the packet library
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      Version Information.
 */
uint32_t Pktlib_getVersion (void)
{
    return PKTLIB_VERSION_ID;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the heap. Please ensure that the
 *      following criteria are met:-
 *
 *      @verbatim
       a) Once the heap is deleted it is  the responsibility of the
          application NOT to use the heap for allocations; there is no
          run time checking added in the PKTLIB API to determine if
          the heap is operational or not since this will cause
          performance penalties.
       b) The heap can only be deleted by the core which created the heap
          This needs to be taken care for "shared heaps". This is NOT
          enforced in the API but is the responsibility of the application.
        @endverbatim
 *
 *  @param[in]  pktlibInstHandle
 *      Heap handle to be deleted
 *  @param[in]  heapHandle
 *      Heap handle to be deleted
 *  @param[out]  errCode
 *      Error Code populated if there is an error.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   Error
 */
int32_t Pktlib_deleteHeap (Pktlib_InstHandle pktlibInstHandle, Pktlib_HeapHandle heapHandle, int32_t* errCode)
{
    Pktlib_MCB*     ptrPktlibMCB;
    Pktlib_Heap*    ptrPktHeap;
    Ti_Pkt*         ptrPkt;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferLen;
    int32_t         memoryRegionQueueNum;
    Qmss_QueueHnd   memoryRegionQueueHnd;
    uint8_t         isAllocated;

    /* Get the pointer to the heap. */
    ptrPktHeap = (Pktlib_Heap *)heapHandle;
    if (ptrPktHeap == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return -1;
    }

    /* Get the pointer to the PKTLIB instance */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;
    if (ptrPktlibMCB == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return -1;
    }

    /* The Packet Library supports the following types of heaps:
     *  - Shared Heaps
     *  - Local Heaps
     *  - Super Heaps */
    if (ptrPktHeap->isSuperHeap == 1)
    {
        /* Super Heap is getting deleted. The member heaps residing below a super
         * heap are NOT deleted. So remove the named resource from the INTERNAL
         * database */
        if (Name_deleteResource (ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                 ptrPktHeap->name, errCode) < 0)
        {
            /* FATAL Error: Deletion of named resource failed */
            return -1;
        }

        /* Cleanup the memory for the heap */
        ptrPktlibMCB->cfg.free (Pktlib_MallocMode_GLOBAL, ptrPktHeap, sizeof(Pktlib_Heap));
        return 0;
    }

    /* Sanity Check: Ensure that all the data buffers with which the heap was configured
     * are present in the free queue. Else we cannot delete the heap */
    if (ptrPktHeap->freeQueueHnd != 0)
    {
        if (Qmss_getQueueEntryCount(ptrPktHeap->freeQueueHnd) != ptrPktHeap->numDataBufferPackets)
        {
            *errCode = PKTLIB_EDATABUFFER_MISMATCH;
            return -1;
        }
    }

    /* Sanity Check: Ensure that all the zero buffer packets with which the heap was configured
     * are present in the free queue. Else we cannot delete the heap */
    if (ptrPktHeap->freeZeroQueueHnd != 0)
    {
        if (Qmss_getQueueEntryCount(ptrPktHeap->freeZeroQueueHnd) != ptrPktHeap->numZeroBufferPackets)
        {
            *errCode = PKTLIB_EZEROBUFFER_MISMATCH;
            return -1;
        }
    }

    /* Get the queue number & handle associated with the memory region. */
    memoryRegionQueueNum = Qmss_getMemRegQueueHandle(ptrPktHeap->memRegion);
    memoryRegionQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, memoryRegionQueueNum, &isAllocated);

    /* The heap has been sanity checked. Now we can delete the descriptors in the data buffer queue if there was
     * one associated with the heap. */
    if (ptrPktHeap->freeQueueHnd != 0)
    {
        while (Qmss_getQueueEntryCount(ptrPktHeap->freeQueueHnd) != 0)
        {
#ifdef __ARMv7
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(ptrPktlibMCB->cfg.phyToVirt(Qmss_queuePopRaw(ptrPktHeap->freeQueueHnd)));
#else
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrPktHeap->freeQueueHnd));
#endif
            /* Get the original data buffer pointer & size. */
            Cppi_getOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, &ptrDataBuffer, &dataBufferLen);

            /* Cleanup the memory associated with the data buffer packet */
            ptrPktHeap->heapFxnTable.dataFree(ptrDataBuffer, ptrPktHeap->dataBufferSize, ptrPktHeap->arg);

            /* Reset all the fields in the descriptor. */
            memset ((void *)ptrPkt, 0, ptrPktHeap->descSize);

            /* Place back the descriptor into the memory region free queue. */
            Qmss_queuePushDesc(memoryRegionQueueHnd, (Cppi_Desc*)ptrPkt);
        }
    }

    /* Now we can delete the descriptors in the zero data buffer queue if there was one associated with the heap. */
    if (ptrPktHeap->freeZeroQueueHnd != 0)
    {
        while (Qmss_getQueueEntryCount(ptrPktHeap->freeZeroQueueHnd) != 0)
        {
#ifdef __ARMv7
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(ptrPktlibMCB->cfg.phyToVirt(Qmss_queuePopRaw(ptrPktHeap->freeZeroQueueHnd)));
#else
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrPktHeap->freeZeroQueueHnd));
#endif
            /* Reset all the fields in the descriptor. */
            memset ((void *)ptrPkt, 0, ptrPktHeap->descSize);

            /* Place back the descriptor into the memory region free queue. */
            Qmss_queuePushDesc(memoryRegionQueueHnd, (Cppi_Desc*)ptrPkt);
        }
    }

    /* Now we need to close all the queues opened by the heap.
     * - Does the heap use starvation queues? */
    if (ptrPktHeap->useStarvationQueue)
    {
        /* YES. With starvation queues; there are 4 contigious queues which
         * are opened.  Queue0 is allocated to the data buffer queues; which
         * Queue1 is allocated to the zero buffer queue. Queue2 and Queue3
         * are unused */
        //TODO: close the shared starvation queues and set them to be not assigned in the MCB
        Qmss_queueClose(ptrPktHeap->baseStarvationQueue + 2);
        Qmss_queueClose(ptrPktHeap->baseStarvationQueue + 3);

        /* Queue 0: This is also the data buffer free queue which is closed here */
        Qmss_queueClose(ptrPktHeap->baseStarvationQueue);

        /* Queue 1: This is also the data buffer free queue which is closed here */
        Qmss_queueClose(ptrPktHeap->baseStarvationQueue + 1);
    }
    else
    {
        /* NO. Starvation queues are NOT configured. Close the data buffer
         * and zero data buffer queues if configured. */
        if (ptrPktHeap->freeQueueHnd != 0)
            Qmss_queueClose(ptrPktHeap->freeQueueHnd);

        if (ptrPktHeap->freeZeroQueueHnd != 0)
            Qmss_queueClose(ptrPktHeap->freeZeroQueueHnd);
    }

    /* All heaps are associated with a garbage queue. */
    Qmss_queueClose(ptrPktHeap->garbageQueueHnd);

    /* Close the memory region queue */
    Qmss_queueClose(memoryRegionQueueHnd);

    /* Remove the heap from the named resource database. */
    if (Name_deleteResource (ptrPktlibMCB->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                             ptrPktHeap->name, errCode) < 0)
    {
        /* FATAL Error: Deletion of named resource failed */
        return -1;
    }

    /* Cleanup the memory for the heap */
    ptrPktlibMCB->cfg.free (Pktlib_MallocMode_GLOBAL, ptrPktHeap, sizeof(Pktlib_Heap));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the PKTLIB instance. PKTLIB heaps
 *      are created within the context of a single PKTLIB instance.
 *
 *  @param[in]  ptrInstCfg
 *      Pointer to the PKTLIB instance configuration
 *  @param[out]  errCode
 *      Error Code populated if there is an error.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   Error
 */
Pktlib_InstHandle Pktlib_createInstance (Pktlib_InstCfg* ptrInstCfg, int32_t* errCode)
{
    Pktlib_MCB* ptrPktlibMCB;

    /* Sanity Check: Validate the arguments. */
    if (ptrInstCfg == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the PKTLIB heap is aligned on the cache boundary since the
     * heaps are shared across cores */
    if ((sizeof(Pktlib_Heap) % 128) != 0)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the OSAL callout has been specified. */
    if ((ptrInstCfg->malloc         == NULL)    ||  (ptrInstCfg->free           == NULL)    ||
        (ptrInstCfg->beginMemAccess == NULL)    ||  (ptrInstCfg->endMemAccess   == NULL)    ||
        (ptrInstCfg->beginPktAccess == NULL)    ||  (ptrInstCfg->endPktAccess   == NULL)    ||
        (ptrInstCfg->enterCS        == NULL)    ||  (ptrInstCfg->exitCS         == NULL))
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    if (ptrInstCfg->useSharedStarvationQueue &&
        ((ptrInstCfg->sharedEnterCS == NULL) || (ptrInstCfg->sharedExitCS == NULL)))
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

#ifdef __ARMv7
    /* Sanity Check: Ensure that the physical to virtual address is specified. This OSAL callout
     * is required only in the ARM domain */
    if (ptrInstCfg->phyToVirt == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }
#endif

    /* Sanity Check: Ensure that the valid instance handles have been passed */
    if ((ptrInstCfg->databaseHandle == NULL) || (ptrInstCfg->sysCfgHandle == NULL))
    {
        *errCode = PKTLIB_EINVAL;
        return NULL;
    }

    /* Allocate memory for the PKTLIB instance. */
    ptrPktlibMCB = ptrInstCfg->malloc (Pktlib_MallocMode_LOCAL, sizeof(Pktlib_MCB));
    if (ptrPktlibMCB == NULL)
    {
        *errCode = PKTLIB_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrPktlibMCB, 0 , sizeof(Pktlib_MCB));

    /* Populate the configuration */
    memcpy ((void *)&ptrPktlibMCB->cfg, (void *)ptrInstCfg, sizeof(Pktlib_InstCfg));

    /* Return the heap instance handle. */
    return (Pktlib_InstHandle)ptrPktlibMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the PKTLIB instance. It is the responsibility
 *      of the application to ensure that all the heaps created on the instance have
 *      been deleted before the instance gets deleted.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle which is being deleted
 *  @param[out]  errCode
 *      Error Code populated if there is an error.
 *
 *  \ingroup PKTLIB_FUNCTION
 *
 *  @retval
 *      0   -   Success
 *  @retval
 *      <0  -   Error
 */
int32_t Pktlib_deleteInstance (Pktlib_InstHandle pktlibInstHandle, int32_t* errCode)
{
    Pktlib_MCB* ptrPktlibMCB;

    /* Sanity Check: Validate the arguments. */
    ptrPktlibMCB = (Pktlib_MCB*)pktlibInstHandle;
    if (ptrPktlibMCB == NULL)
    {
        *errCode = PKTLIB_EINVAL;
        return -1;
    }
    /* Clean up memory for the PKTLIB instance. */
    ptrPktlibMCB->cfg.free (Pktlib_MallocMode_LOCAL, ptrPktlibMCB, sizeof(Pktlib_MCB));
    return 0;
}

//<fzm>
static void prefetchDescriptor(PrefetchedDescriptor* prefDescr, Qmss_QueueHnd queueHandle)
{
    prefDescr->ptrDesc = (Cppi_HostDesc*)QMSS_DESC_PTR(Osal_qmssPhyToVirt(Qmss_queuePopRaw(queueHandle)));

    if (prefDescr->ptrDesc)
    {
        __builtin_prefetch((const void *)prefDescr->ptrDesc, 1);
        __builtin_prefetch((char*)prefDescr->ptrDesc + 64, 1);
    }

    prefDescr->bufferPrefetched = 0;
}

static void prefetchBuffer(PrefetchedDescriptor* prefDescr)
{
    prefDescr->ptrDesc->origBuffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(prefDescr->ptrDesc->origBuffPtr));

    __builtin_prefetch((const void *)prefDescr->ptrDesc->origBuffPtr, 1);
    __builtin_prefetch((char *)prefDescr->ptrDesc->origBuffPtr + 64, 1);

    prefDescr->ptrDesc->buffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(prefDescr->ptrDesc->buffPtr));

    prefDescr->bufferPrefetched = 1;
}

Ti_Pkt* Pktlib_popPrefetch(Qmss_QueueHnd queueHandle, Pktlib_Prefetch* prefetchData, const uint32_t limit)
{
    // To not have gaps in prefechted data we expect such chain of descriptors
    // X.. XX. XXX

    // Descriptor-0...
    if(!prefetchData->prefetchedDescriptors[0].ptrDesc)
    {
        prefetchDescriptor(&prefetchData->prefetchedDescriptors[0], queueHandle);
    }

    if (prefetchData->prefetchedDescriptors[0].ptrDesc)
    {
        // We should enter this section when run this function first time and there are not prefetched descriptors at all
        if(!prefetchData->prefetchedDescriptors[0].bufferPrefetched && 
            prefetchData->prefetchedDescriptors[0].ptrDesc->origBuffPtr)
        {
            prefetchBuffer(&prefetchData->prefetchedDescriptors[0]);
        }

        // Descriptor-1...
        if(prefetchData->prefetchedDescriptors[1].ptrDesc &&
           prefetchData->prefetchedDescriptors[1].ptrDesc->origBuffPtr)
        {
            prefetchBuffer(&prefetchData->prefetchedDescriptors[1]);
        }
        // We should enter this section when run this function first time and there are not prefetched descriptors at all
        else if(!prefetchData->prefetchedDescriptors[1].ptrDesc && limit > 1)
        {
            prefetchDescriptor(&prefetchData->prefetchedDescriptors[1], queueHandle);
        }

        // Descriptor-2...
        if(prefetchData->prefetchedDescriptors[1].ptrDesc && limit > 2)
            prefetchDescriptor(&prefetchData->prefetchedDescriptors[2], queueHandle);
    }

    Cppi_HostDesc* ptrDesc = prefetchData->prefetchedDescriptors[0].ptrDesc;
    prefetchData->prefetchedDescriptors[0] = prefetchData->prefetchedDescriptors[1];
    prefetchData->prefetchedDescriptors[1] = prefetchData->prefetchedDescriptors[2];
    prefetchData->prefetchedDescriptors[2].ptrDesc = NULL;
    prefetchData->prefetchedDescriptors[2].bufferPrefetched = 0;

    return (Ti_Pkt*)ptrDesc;
}

Ti_Pkt* Pktlib_allocPacketPrefetch
(
    Pktlib_HeapHandle   heapHandle,
    uint32_t            size
)
{
    Qmss_QueueHnd       queueHandle;
    Pktlib_Prefetch*    ptrPrefetch;

    /* Get the pointer to the heap. */
    Pktlib_Heap* ptrPktHeap = (Pktlib_Heap *)heapHandle;

    /* Check the size to determine the type of allocation?
     *  - Use the Zero Queue if the size is 0.
     *  - Use the data queue if the size is within the heap size.
     *  - Else the heap cannot satisfy the request */
    if (size == 0)
    {
        /* Zero Buffer Allocation: */
        queueHandle = ptrPktHeap->freeZeroQueueHnd;
        ptrPrefetch = &ptrPktHeap->prefetchZeroBuffer;
    }
    else if (size <= ptrPktHeap->dataBufferSize)
    {
        /* Normal Buffer Allocation: */
        queueHandle = ptrPktHeap->freeQueueHnd;
        ptrPrefetch = &ptrPktHeap->prefetchDataBuffer;
    }
    else
    {
        /* Error: Request cannot be satisfied. */
        queueHandle = 0;
    }

    /* Allocation can proceed only if a valid queue handle was present.
     *  - This is a catch all because we can have a zero buffer allocation
     *    being attempted on a heap which was not configured for zero buffers */
    if (queueHandle == 0)
        return NULL;

    Cppi_Desc* ptrDesc = (Cppi_Desc*)Pktlib_popPrefetch(queueHandle, ptrPrefetch, PKTLIB_PREFETCHED_DESCRIPTORS_COUNT);

    /* Did we get a packet. */
    if (ptrDesc != NULL)
    {
        uint8_t*    ptrDataBuffer;
        uint32_t    dataLen;

        /* Get the original buffer information. */
        Cppi_getOriginalBufInfo (Cppi_DescType_HOST, ptrDesc,
                                 (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Ensure that the original buffer and buffer address is one and the same. */
        Cppi_setData (Cppi_DescType_HOST, ptrDesc, (uint8_t*)ptrDataBuffer, size);

        /* Kill any other links to the packets. */
        Cppi_linkNextBD (Cppi_DescType_HOST, ptrDesc, NULL);

        /* Reset the PS Flags in the packet: */
        Cppi_setPSFlags(Cppi_DescType_HOST, ptrDesc, 0x0);

        /* Setup the packet length  */
        Pktlib_setPacketLen((Ti_Pkt*)ptrDesc, size);
    }

    /* Return the allocated packet. */
    return (Ti_Pkt*)ptrDesc;
}
//</fzm>
