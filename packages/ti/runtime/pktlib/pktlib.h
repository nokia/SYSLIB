/**
 *   @file  pktlib.h
 *
 *   @brief
 *      Header file for the Packet Library. The file exposes the data structures
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

/** @defgroup PACKET_LIB_API Packet Library
 */
#ifndef __PKTLIB_H__
#define __PKTLIB_H__

/* MCSDK Include files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/pktlib/pktlibver.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup PKTLIB_SYMBOL  Packet Library Symbols
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_INTERNAL_SYMBOL  Packet Library Internal Symbols
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_ERROR_CODE  Packet Library Error codes
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_FUNCTION  Packet Library Exported Functions
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_INTERNAL_FUNCTION  Packet Library Internal Functions
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_DATA_STRUCTURE  Packet Library Data Structures
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_INTERNAL_DATA_STRUCTURE  Packet Library Internal Data Structures
@ingroup PACKET_LIB_API
*/
/**
@defgroup PKTLIB_OSAL_API  Packet Library OS Abstraction Layer
@ingroup PACKET_LIB_API
*/

/** @addtogroup PKTLIB_SYMBOL
 @{ */

/**
 * @brief   This is the maximum length of the heap name.
 */
#define PKTLIB_MAX_CHAR                     32
/**
@}
*/

#define PKTLIB_PREFETCHED_DESCRIPTORS_COUNT 3 //fzm

/** @addtogroup PKTLIB_ERROR_CODE
 *
 * @brief
 *  Base error code for the PKTLIB module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   The error code indicates an invalid argument was passed to the API.
 */
#define PKTLIB_EINVAL                       SYSLIB_ERRNO_PKTLIB_BASE-1

/**
 * @brief   This error code implies that there was a resource error.
 */
#define PKLIB_ERESOURCE                     SYSLIB_ERRNO_PKTLIB_BASE-2

/**
 * @brief   This error code implies that the MPU was not configured to allow the
 * PKTLIB to write to use the QMSS threshold functionality.
 */
#define PKTLIB_EPERM                        SYSLIB_ERRNO_PKTLIB_BASE-3

/**
 * @brief   This error code implies that an OUT of memory error was detected.
 */
#define PKTLIB_ENOMEM                       SYSLIB_ERRNO_PKTLIB_BASE-4

/**
 * @brief   The error code implies that the number of data buffers packets in the heap
 * is NOT the same as the number of the data buffer packets with which the heap was
 * created. This error is returned during heap deletion.
 */
#define PKTLIB_EDATABUFFER_MISMATCH         SYSLIB_ERRNO_PKTLIB_BASE-5

/**
 * @brief   The error code implies that the number of zero buffers packets in the heap
 * is NOT the same as the number of the zero buffer packets with which the heap was
 * created. This error is returned during heap deletion.
 */
#define PKTLIB_EZEROBUFFER_MISMATCH         SYSLIB_ERRNO_PKTLIB_BASE-6

/**
 * @brief   The error code implies that the packet has reached the maximum reference
 * counter and that the packet can no lomger be cloned or split anymore.
 */
#define PKTLIB_ELIMIT                       SYSLIB_ERRNO_PKTLIB_BASE-7

/**
 * @brief   The error code implies that the packet was corrupted and there was an internal
 * fault
 */
#define PKTLIB_EINTERNAL                    SYSLIB_ERRNO_PKTLIB_BASE-8

/**
@}
*/

/** @addtogroup PKTLIB_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  PKTLIB Malloc mode
 *
 * @details
 *  The enumeration describes the PKTLIB memory allocation mode. The PKTLIB module
 *  uses the OSAL function to allocate memory for different purposes.
 */
typedef enum Pktlib_MallocMode
{
    /**
     * @brief   In the DSP realm; this implies that the memory allocated should be from
     * the global DDR3 or MSMC memory pools. The memory should be accessible across different
     * DSP cores which are using the same PKTLIB instance.
     *
     * In the ARM realm the memory allocated is from within the process memory heap. If multiple
     * processes are sharing heaps then memory allocated should be from Linux shared memory pools.
     */
    Pktlib_MallocMode_GLOBAL    =   0x1,

    /**
     * @brief   The memory is allocated and used only within the specific execution realm and does
     * NOT need to be shared across different realms.
     */
    Pktlib_MallocMode_LOCAL     =   0x2
}Pktlib_MallocMode;

/**
 * @brief
 *  The packet library exposes the Ti_Pkt as an opaque handle.
 */
typedef void*   Ti_Pkt;

/**
 * @brief
 *  Packet Library Heap Handle
 */
typedef void*   Pktlib_HeapHandle;

/**
 * @brief
 *  Packet Library instance handle
 */
typedef void*   Pktlib_InstHandle;

/**
@}
*/

/** @addtogroup PKTLIB_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory of the specific mode.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  size
 *      Number of bytes of memory to be allocated
 *
 *  @retval
 *      Pointer to the allocated memory
 */
typedef void* (*Osal_PktlibMalloc) (Pktlib_MallocMode mode, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to free memory of the specific mode.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the allocated memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes of memory to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibFree) (Pktlib_MallocMode mode, void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is to invalidate the cache contents.
 *
 *  @param[in]  ptr
 *      Pointer to the memory
 *  @param[in]  size
 *      Number of bytes of memory
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibBeginMemAccess) (void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is to writeback the cache contents.
 *
 *  @param[in]  ptr
 *      Pointer to the memory
 *  @param[in]  size
 *      Number of bytes of memory
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibEndMemAccess) (void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is invalidate the cache contents of the packet. This does
 *      not perform any cache operations of the data buffer associated with the packet
 *
 *  @param[in]  ptr
 *      Pointer to the packet
 *  @param[in]  size
 *      Size of the packet
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibBeginPktAccess) (Pktlib_HeapHandle heapHandle, Ti_Pkt* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is writeback the cache contents of the packet. This does
 *      not perform any cache operations of the data buffer associated with the packet
 *
 *  @param[in]  ptr
 *      Pointer to the packet
 *  @param[in]  size
 *      Size of the packet
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibEndPktAccess) (Pktlib_HeapHandle heapHandle, Ti_Pkt* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to enter a critical section. Packets when split/cloned or freed
 *      cause the reference counter in the packet to be incremented/decremented and this counter
 *      needs to be protected against concurrent access across different threads on the same core
 *
 *      PKTLIB does *not* support reference counter modifications across different cores.
 *
 *  @param[in]  heapHandle
 *      Heap handle for which protection is required
 *
 *  @retval
 *      Not applicable
 */
typedef void* (*Osal_PktlibEnterCriticalSection) (Pktlib_HeapHandle heapHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to exit a critical section. Packets when split/cloned or freed
 *      cause the reference counter in the packet to be incremented/decremented and this counter
 *      needs to be protected against concurrent access across different threads on the same core
 *
 *      PKTLIB does *not* support reference counter modifications across different cores.
 *
 *  @param[in]  heapHandle
 *      Heap handle for which protection is required
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibExitCriticalSection) (Pktlib_HeapHandle heapHandle, void* csHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to enter a critical section. This is used to protect the
 *      shared starvation queue list if heaps are created or destroyed in more than one thread
 *
 *  @param[in]
 *      Not applicable
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_PktlibEnterSharedCriticalSection) (void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to exit a critical section. This is used to protect the
 *      shared starvation queue list if heaps are created or destroyed in more than one thread
 *
 *  @param[in]
 *      Not applicable
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_PktlibExitSharedCriticalSection) (void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to convert the physical address to a virtual address
 *      This OSAL API is applicable only in the ARM realm and is not used in the DSP
 *      realm
 *
 *  @param[in]  ptrPhysicalAddress
 *      Pointer to the physical address
 *
 *  @retval
 *      Virtual address
 */
typedef void* (*Osal_PktlibPhyToVirt) (void* ptrPhysicalAddress);

/**
@}
*/

/** @addtogroup PKTLIB_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Application registered call back function which is invoked to allocate
 *  data buffers for the PKTLIB heap
 *
 *  @param[in]  size
 *      Size of the data buffer to be allocated
 *  @param[in]  arg
 *      Optional application defined argument registered during heap creation
 *
 *  @retval
 *      Success -   Pointer to the allocated data buffer
 *  @retval
 *      Error   -   NULL
 */
typedef uint8_t* (*Pktlib_MallocData)(uint32_t size, uint32_t arg);

/**
 * @brief
 *  Application registered call back function which is invoked to clean
 *  data buffers associated with the PKTLIB heap
 *
 *  @param[in]  ptrDataBuffer
 *      Pointer to the data buffer to be cleaned up
 *  @param[in]  size
 *      Size of the data buffer to be allocated
 *  @param[in]  arg
 *      Optional application defined argument registered during heap creation
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Pktlib_FreeData)(uint8_t* ptrDataBuffer, uint32_t size, uint32_t arg);

/**
 * @brief
 *  The structure describes the Heap Interface Table
 *
 * @details
 *  There could exist multiple heaps in the system. Heaps can have different
 *  properties depending upon the memory region (cached vs. non-cached) and
 *  how the data buffers need to be allocated. The table here provides a
 *  well defined interface which allows this information to be registered
 *  during heap creation.
 */
typedef struct Pktlib_HeapIfTable
{
    /**
     * @brief   This API will be called by the packet library during heap creation
     * to allocate data memory for the packets.
     */
    Pktlib_MallocData       dataMalloc;

    /**
     * @brief   This API will be called by the packet library during heap deletion
     * or resizing to clean up data memory for the packets.
     */
    Pktlib_FreeData         dataFree;
}Pktlib_HeapIfTable;

/**
 * @brief
 *  The structure describes the Heap Configuration.
 *
 * @details
 *  The configuration is populated and passed to the PKTLIB module when a heap
 *  is being created.
 */
typedef struct Pktlib_HeapCfg
{
    /**
     * @brief   Heap Name which should be unique and identifies the heap.
     */
    char                name[PKTLIB_MAX_CHAR];

    /**
     * @brief   PKTLIB instance handle associated with the heap.
     */
    Pktlib_InstHandle   pktlibInstHandle;

    /**
     * @brief   QMSS memory region from where the packets will be carved out
     */
    Qmss_MemRegion      memRegion;

    /**
     * @brief   This flag identifies if the heap is shared or private. Private
     * heaps are visible only on the core while shared heaps are visible across
     * cores.
     */
    uint32_t            sharedHeap;

    /**
     * @brief   This flag indicates that the heap should use STARVATION queues
     * which allow the Navigator infrastructure to detect queue empty and record
     * statistics.
     */
    uint32_t            useStarvationQueue;

    /**
     * @brief   The threshold can be used to determine heap buffer usage. If the
     * data buffer packets in the heap fall below the threshold value this will
     * be detected. Please ensure that the thresholds are always a power of 2
     */
    uint32_t            dataBufferPktThreshold;

    /**
     * @brief   The threshold can be used to determine heap zero-buffer usage. If
     * the zero buffer packets in the heap fall below the threshold value this will
     * be detected. Please ensure that the thresholds are always a power of 2
     */
    uint32_t            zeroBufferPktThreshold;

    /**
     * @brief   Each heap has data buffers of the size specified here.
     */
    uint32_t            dataBufferSize;

    /**
     * @brief   These are the number of packets which are associated with the
     * data buffer size specified above.
     */
    uint32_t            numPkts;

    /**
     * @brief   These are the number of zero buffer packets which should be
     * present in the heap
     */
    uint32_t            numZeroBufferPackets;

    /**
     * @brief   Optional application defined argument which is passed to the application
     * registered heap interface table API during heap creation and deletion.
     */
    uint32_t            arg;

    /**
     * @brief   Heap Interface Function table which identifies functions for data allocation
     * and cleanup.
     */
    Pktlib_HeapIfTable  heapInterfaceTable;

    // <fzm>
    /**
    * @brief   This option specifies whether the created heap shall be created
    * with return push policy set to HEAD (it is TAIL by default)
    */
    uint32_t                pushToHead;

    /**
    * @brief   This option specifies whether the buffers will be allocated in a
    * single contiguous block, or individualy
    */
    uint32_t                linearAlloc;
    // </fzm>
}Pktlib_HeapCfg;

//<fzm>
/**
 * @brief
 *  Prefetching data support
 *
 * @details
 *  The prefetching variables have all data to support quicker pulling
 *  descriptor and its buffer into cache
 */
typedef struct PrefetchedDescriptor
{
    Cppi_HostDesc* ptrDesc;
    int bufferPrefetched;
}PrefetchedDescriptor;

typedef struct Pktlib_Prefetch
{
    PrefetchedDescriptor prefetchedDescriptors[PKTLIB_PREFETCHED_DESCRIPTORS_COUNT];
}Pktlib_Prefetch;
//</fzm>

/**
 * @brief
 *  The structure describes the Heap Statistics
 *
 * @details
 *  Heap statistics reported by the packet library
 */
typedef struct Pktlib_HeapStats
{
    /**
     * @brief   This is the current number of free data packets
     * which are available
     */
    uint32_t    numFreeDataPackets;

    /**
     * @brief   This is the current number of free packets (with no buffers)
     * which are available
     */
    uint32_t    numZeroBufferPackets;

    /**
     * @brief   This is the current number of packets (with & without) data buffers
     * which are in the garbage queue.
     */
    uint32_t    numPacketsinGarbage;

    /**
     * @brief   This field is set to indicate that the heap has hit the specified
     * data buffer threshold. It is applicable only if the 'starvation data threshold'
     * was specified during heap creation.
     */
    uint8_t    dataBufferThresholdStatus;

    /**
     * @brief   This is set only if the heap was configured to use starvation queues
     * and indicates the number of times the data buffer queue was starved
     * of packets.
     */
    uint8_t     dataBufferStarvationCounter;

    /**
     * @brief   This field is set to indicate that the heap has hit the specified
     * zero data buffer threshold.It is applicable only if the 'starvation zero buffer
     * treshold' was specified during heap creation.
     */
    uint8_t     zeroDataBufferThresholdStatus;

    /**
     * @brief   This is set only if the heap was configured to use starvation queues
     * and indicates the number of times the zero buffer queue was starved of packets.
     */
    uint8_t     zeroDataBufferStarvationCounter;

    /**
     * @brief   This is set only if the heap was configured to use starvation queues
     * and indicates the number of times the buffer queue was starved of packets.
     * This value is cumulative over all reads of the starvation count register.
     * This value is only populated if shared starvation queues are enabled in the
     * Pktlib instance
     */
    uint32_t     extendedDataBufferStarvationCounter;

    /**
     * @brief   This is set only if the heap was configured to use starvation queues
     * and indicates the number of times the zero buffer queue was starved of packets.
     * This value is cumulative over all reads of the starvation count register.
     * This value is only populated if shared starvation queues are enabled in the
     * Pktlib instance
     */
    uint32_t     extendedZeroDataBufferStarvationCounter;
}Pktlib_HeapStats;

/**
 * @brief
 *  Instance configuration
 *
 * @details
 *  PKTLIB instance configuration.
 */
typedef struct Pktlib_InstCfg
{
    /**
     * @brief   PKTLIB Instance identifier.
     */
    uint8_t                             instanceId;

    /**
     * @brief   This is the database handle associated with the PKTLIB instance.
     * All PKTLIB heaps created and found using the instance will be searched
     * in this database only
     */
    Name_DBHandle                       databaseHandle;

    /**
     * @brief   This is the handle to the system configuration.
     */
    Resmgr_SysCfgHandle                 sysCfgHandle;

    /**
     * @brief   OSAL Malloc
     */
    Osal_PktlibMalloc                   malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_PktlibFree                     free;

    /**
     * @brief   OSAL Begin memory access
     */
    Osal_PktlibBeginMemAccess           beginMemAccess;

    /**
     * @brief   OSAL End memory access
     */
    Osal_PktlibEndMemAccess             endMemAccess;

    /**
     * @brief   OSAL Begin Packet access
     */
    Osal_PktlibBeginPktAccess           beginPktAccess;

    /**
     * @brief   OSAL End Packet access
     */
    Osal_PktlibEndPktAccess             endPktAccess;

    /**
     * @brief   OSAL Enter critical section
     */
    Osal_PktlibEnterCriticalSection     enterCS;

    /**
     * @brief   OSAL Exit critical section
     */
    Osal_PktlibExitCriticalSection      exitCS;

    /**
     * @brief   OSAL Physical to Virtual address conversion. This is NOT used in the
     * DSP realm.
     */
    Osal_PktlibPhyToVirt                phyToVirt;

    /**
     * @brief   flag to indicate if shared starvation Queues are used
     */
    uint32_t                            useSharedStarvationQueue;

    /**
     * @brief   OSAL Enter shared critical section
     */
    Osal_PktlibEnterSharedCriticalSection     sharedEnterCS;

    /**
     * @brief   OSAL Exit shared critical section
     */
    Osal_PktlibExitSharedCriticalSection      sharedExitCS;
}Pktlib_InstCfg;

/**
@}
*/

/** @addtogroup PKTLIB_FUNCTION
 @{ */

/**
 *  @b Description
 *  @n
 *      The function is used to get the descriptor from the TI packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the TI packet
 *
 *  @retval
 *      Pointer to the host descriptor.
 */
static inline Cppi_HostDesc* Pktlib_getDescFromPacket(Ti_Pkt* ptrPkt)
{
    return (Cppi_HostDesc*)ptrPkt;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the TI packet from the descriptor
 *
 *  @param[in]  ptrHostDesc
 *      Pointer to the host descriptor.
 *
 *  @retval
 *      Pointer to the TI packet.
 */
static inline Ti_Pkt* Pktlib_getPacketFromDesc(Cppi_HostDesc* ptrHostDesc)
{
    return (Ti_Pkt*)ptrHostDesc;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of descriptors which are a part of the
 *      packet.
 *
 *  @param[in]  ptrPkt
 *      Packet for which the number of descriptors are required.
 *
 *  @retval
 *      Number of buffers.
 */
static inline uint8_t Pktlib_packetBufferCount(Ti_Pkt* ptrPkt)
{
    Cppi_HostDesc*  ptrDesc = Pktlib_getDescFromPacket(ptrPkt);
    uint8_t         count = 0;

    if (ptrDesc == NULL)
        return count;

    /* Cycle through all the descriptors in the packet. */
    do
    {
        /* Increment the number of the descriptors which are available. */
        count = count + 1;

        /* Get the next descriptor. */
        ptrDesc = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)ptrDesc);
    }while (ptrDesc != NULL);

    /* Return the number of buffers. */
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the data buffer associated with the packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[out]  ptrDataBuffer
 *      Data Buffer associated with the packet populated by this API
 *  @param[out]  dataLen
 *      Data Buffer Length populated by this API
 *
 *  @retval
 *     Not Applicable.
 */
static inline void Pktlib_getDataBuffer(Ti_Pkt* ptrPkt, uint8_t** ptrDataBuffer, uint32_t* dataLen)
{
    Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, ptrDataBuffer, dataLen);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the length of the data buffer associated with the packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *
 *  @retval
 *      Data Buffer associated with the packet.
 */
static inline uint32_t Pktlib_getDataBufferLen(Ti_Pkt* ptrPkt)
{
    return Cppi_getDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the length of the packet. The function
 *      should *only* be called for packets which have a single node or
 *      for the head of the chained list of packets. Using this API for any
 *      other packet is not supported and would cause issues especially
 *      if these packets are being pushed across peripherals.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *
 *  @retval
 *      Length of the packet.
 */
static inline uint32_t Pktlib_getPacketLen(Ti_Pkt* ptrPkt)
{
    /* Packet Length is stored in bits 0 to 21. */
    return Cppi_getPacketLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the next packet
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *
 *  @retval
 *      Next Linked Packet.
 */
static inline Ti_Pkt* Pktlib_getNextPacket(Ti_Pkt* ptrPkt)
{
    return (Ti_Pkt*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the data buffer length associated with the packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  dataLen
 *      Data Buffer Length to be configured
 *
 *  @retval
 *     Not Applicable.
 */
static inline void Pktlib_setDataBufferLen(Ti_Pkt* ptrPkt, uint32_t dataLen)
{
    Cppi_setDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, dataLen);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the length of the packet. The function
 *      should *only* be called for packets which have a single node or
 *      for the head of the chained list of packets. Using this API for any
 *      other packet is not supported and would cause issues especially
 *      if these packets are being pushed across peripherals.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  packetLen
 *      Length of the packet to be configured.
 *
 *  @retval
 *      Not applicable
 */
static inline void Pktlib_setPacketLen(Ti_Pkt* ptrPkt, uint32_t packetLen)
{
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, packetLen);
}

/**
 *  @b Description
 *  @n
 *      The function is used to offset the data buffer in the packet by a specified
 *      number of bytes. The data buffer length is also modified to account for the
 *      offset.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  offset
 *      Offset value in bytes.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t Pktlib_setDataOffset(Ti_Pkt* ptrPkt, int32_t offset)
{
    uint8_t*    ptrDataBuffer;
    uint32_t    dataLen;
    uint8_t*    ptrOrigDataBuffer;
    uint32_t    origDataLen;

    /* Get the current data information. */
    Pktlib_getDataBuffer(ptrPkt, (uint8_t**)&ptrDataBuffer, &dataLen);

    /* Sanity Check: Is the offset +ve or -ve? */
    if (offset >= 0)
    {
        /* +ve: Make sure that the offset does not exceed the current data length. */
        if (offset > dataLen)
            return -1;
    }
    else
    {
        /* -ve: Get the original buffer information. */
        Cppi_getOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt,
                                 (uint8_t**)&ptrOrigDataBuffer, &origDataLen);

        /* Make sure we dont go back beyond the original data buffer. */
        if ((dataLen - offset) > origDataLen)
            return -1;
    }

    /* Offset the data and account for it in the data length also. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt, (uint8_t*)(ptrDataBuffer + offset),
                  (dataLen - offset));

    /* Data was offset successfully.*/
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The PKTLIB uses the SOURCE_TAG_HI field to store its private information.
 *      This field can get overwritten using the native Cppi_setTag API. This API
 *      is a replacement of the above API because it ensures that the PKTLIB
 *      Private information is always preserved.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  tag
 *      Tags to be configured in the packet.
 *
 *  @retval
 *      Not applicable
 */
static inline void Pktlib_setTags(Ti_Pkt* ptrPkt, Cppi_DescTag* tag)
{
    uint8_t         pktlibPrivateInfo;
    Cppi_HostDesc*  ptrHostDesc;

    /* Get the host descriptor. */
    ptrHostDesc = Pktlib_getDescFromPacket(ptrPkt);

    /* Get the PKTLIB Private information. */
    pktlibPrivateInfo = CSL_FEXTR(ptrHostDesc->tagInfo, 31, 24);

    /* Store the new tag information into the packet. */
    ptrHostDesc->tagInfo = CSL_FMKR(7,  0,  tag->destTagLo)     |
                           CSL_FMKR(15, 8,  tag->destTagHi)     |
                           CSL_FMKR(23, 16, tag->srcTagLo)      |
                           CSL_FMKR(31, 24, pktlibPrivateInfo);
}



//fzm ---->
extern void* Osal_qmssPhyToVirt (void *ptr);
extern void* Osal_qmssVirtToPhy (void *ptr);

/**
 *  @b Description
 *  @n
 *      The function API is used to traverse a CPPI host descriptor and perform
 *      a virtual address to physical address translation on all address references in the descriptor
 *  @param[in] descAddr
 *      Virtual address of descriptor
 *
 *  @retval
 *      none
 */
static inline void* Pktlib_mVMConvertDescVirtToPhy(void *descAddr)
{
    if (!descAddr)
        return NULL;

    Cppi_HostDesc *nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR(descAddr);
    Cppi_HostDesc *prevBDPtr = NULL;
    while (nextBDPtr)
    {
        void *buffPtr = NULL;
        if (nextBDPtr->buffPtr)
        {
            buffPtr = (void *)nextBDPtr->buffPtr;
            nextBDPtr->buffPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(buffPtr));

            if (!(nextBDPtr->buffPtr))
                return NULL;
        }

        if (nextBDPtr->origBuffPtr)
        {
            nextBDPtr->origBuffPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(nextBDPtr->origBuffPtr));

            if (!(nextBDPtr->origBuffPtr))
                return NULL;
        }

        prevBDPtr = nextBDPtr;
        nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR((nextBDPtr->nextBDPtr));
        if (prevBDPtr->nextBDPtr)
        {
            prevBDPtr->nextBDPtr = (uint32_t)Osal_qmssVirtToPhy((void *)(prevBDPtr->nextBDPtr));

            if (!(prevBDPtr->nextBDPtr))
                return NULL;
        }
    }
    descAddr = Osal_qmssVirtToPhy(descAddr);

    if (!descAddr)
        return NULL;

    /* Issue memory barrier */
    __asm__ __volatile__ ("dsb st" : : : "memory");

    return descAddr;

}

/**
 *  @b Description
 *  @n
 *      The function API is used to traverse a CPPI host descriptor and perform
 *      a physical address to virtual address translation on all address references in the descriptor
 *  @param[in] descAddr
 *      Physical  address of descriptor
 *   @retval
 *      none
 */
static inline void* Pktlib_mVMConvertDescPhyToVirt(void *descAddr)
{
    if (!descAddr)
        return NULL;

    descAddr = Osal_qmssPhyToVirt(descAddr);

    Cppi_HostDesc *nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR(descAddr);
    while (nextBDPtr)
    {
        if (nextBDPtr->buffPtr)
        {
            nextBDPtr->buffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->buffPtr));

            if (!(nextBDPtr->buffPtr))
                return NULL;
        }

        if (nextBDPtr->origBuffPtr)
        {
            nextBDPtr->origBuffPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->origBuffPtr));

            if (!(nextBDPtr->origBuffPtr))
                return NULL;
        }

        if (nextBDPtr->nextBDPtr)
        {
            nextBDPtr->nextBDPtr = (uint32_t)Osal_qmssPhyToVirt((void *)(nextBDPtr->nextBDPtr));

            if (!(nextBDPtr->nextBDPtr))
                return NULL;
        }

        nextBDPtr = (Cppi_HostDesc *)QMSS_DESC_PTR((nextBDPtr->nextBDPtr));
    }

    return descAddr;
}
//<------ fzm

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

/* Instance API: */
extern Pktlib_InstHandle Pktlib_createInstance (Pktlib_InstCfg* ptrInstCfg, int32_t* errCode);
extern int32_t Pktlib_deleteInstance (Pktlib_InstHandle pktlibInstHandle, int32_t* errCode);

/* Packet Operation API: */
extern Ti_Pkt* Pktlib_packetMerge(Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* pPkt1, Ti_Pkt* pPkt2, Ti_Pkt* pLastPkt);
extern int32_t Pktlib_clonePacket(Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* ptrPktOrig, Ti_Pkt* ptrClonePacket);
extern Ti_Pkt* Pktlib_allocPacket(Pktlib_InstHandle pktlibInstHandle, Pktlib_HeapHandle heapHandle, uint32_t size);
//<fzm>
extern Ti_Pkt* Pktlib_allocPacketPrefetch(Pktlib_HeapHandle heapHandle, uint32_t size);
extern Ti_Pkt* Pktlib_popPrefetch(Qmss_QueueHnd queueHandle, Pktlib_Prefetch* prefetchData, const uint32_t limit);
//</fzm>
extern void Pktlib_freePacket(Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* pPkt);
extern int32_t Pktlib_splitPacket(Pktlib_InstHandle pktlibInstHandle,
                                  Ti_Pkt* pOrgPkt, Ti_Pkt* pNewPkt,
                                  uint32_t splitPacketSize,
                                  Ti_Pkt** pPkt1,
                                  Ti_Pkt** pPkt2);
extern int32_t Pktlib_splitPacket2(Pktlib_InstHandle pktlibInstHandle,
                                  Ti_Pkt* pOrgPkt, Ti_Pkt* pNewPkt,
                                  uint32_t splitPacketSize,
                                  Ti_Pkt** pPkt1,
                                  Ti_Pkt** pPkt2);
extern int32_t Pktlib_copyMetaData (Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* ptrPktDst, Ti_Pkt* ptrPktSrc);

/* Heap API: */
extern Pktlib_HeapHandle Pktlib_createHeap(Pktlib_HeapCfg* ptrHeapCfg,int32_t* errCode);
extern int32_t Pktlib_deleteHeap (Pktlib_InstHandle pktlibInstHandle, Pktlib_HeapHandle heapHandle, int32_t* errCode);
extern Pktlib_HeapHandle Pktlib_createSuperHeap(Pktlib_InstHandle   pktlibInstHandle,
                                                const char*         heapName,
                                                Pktlib_HeapHandle   memberHeap[],
                                                int32_t             numMemberHeaps,
                                                int32_t*            errCode);
extern Pktlib_HeapHandle Pktlib_findHeapByName (Pktlib_InstHandle pktlibInstHandle, const char* heapName, int32_t* errCode);
extern Pktlib_HeapHandle Pktlib_getPktHeap(Ti_Pkt* pPkt);
extern void Pktlib_garbageCollection(Pktlib_InstHandle pktlibInstHandle,
                                     Pktlib_HeapHandle heapHandle);
extern void Pktlib_garbageCollectionWithCount(Pktlib_InstHandle pktlibInstHandle,
                                              Pktlib_HeapHandle heapHandle,
                                              uint32_t const count);

/* Ownership API: On the ARM we are currently operating in cache coherent memory. So the API here have been
 * defined to be empty. The support will need to be added later on. */
#ifdef __ARMv7
#define Pktlib_getOwnership(X, Y) {}
#define Pktlib_releaseOwnership(X, Y) {}
#else
extern void Pktlib_getOwnership(Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* pPkt);
extern void Pktlib_releaseOwnership(Pktlib_InstHandle pktlibInstHandle, Ti_Pkt* pPkt);
#endif

/* Utility API: */
extern Qmss_QueueHnd Pktlib_getZeroHeapQueue(Pktlib_HeapHandle heapHandle);
extern Qmss_QueueHnd Pktlib_getInternalHeapQueue(Pktlib_HeapHandle heapHandle);
extern Qmss_QueueHnd Pktlib_getGarbageHeapQueue(Pktlib_HeapHandle heapHandle);
extern uint32_t Pktlib_getMaxBufferSize(Pktlib_HeapHandle heapHandle);
extern void Pktlib_getHeapStats(Pktlib_HeapHandle heapHandle, Pktlib_HeapStats* ptrHeapStats);

#ifdef __cplusplus
}
#endif

#endif /* __PKTLIB_H__ */
