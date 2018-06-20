/**
 *   @file  memlog.h
 *
 *   @brief
 *      Header file which specifies the memory logging.
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

/** @defgroup MEMLOG_API   Resource Management
 */
#ifndef __MEMLOG_H__
#define __MEMLOG_H__

/* MCSDK Include Files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm_services.h>

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_db.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup MEMLOG_DATA_STRUCTURE             Memory Logging Data Structures
@ingroup  RES_MGR_API
*/
/**
@defgroup MEMLOG_SYMBOL                    Memory Logging Defined Symbols
@ingroup MEMLOG_API
*/
/**
@defgroup MEMLOG_FUN                       Memory Logging Exported Functions
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_ENUM                      Memory Logging Enumerations
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_INTERNAL_FUN              Memory Logging Internal Functions
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_ARM_INTERNAL_FUN          Memory Logging ARM Internal Functions
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_DSP_INTERNAL_FUN          Memory Logging DSP Internal Functions
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_OSAL_API                  Memory Logging OS Adaptation Layer
@ingroup  MEMLOG_API
*/
/**
@defgroup MEMLOG_ERR_CODE                  Memory Logging Error Codes
@ingroup  MEMLOG_API
*/

/** @addtogroup MEMLOG_SYMBOL
 @{ */

/**
 * @brief   Maximum number of characters
 */
#define MEMLOG_MAX_CHAR                             32

/**
 * @brief   Memory alignment for memory logging buffers.
 * It is typically the Linux memory page size.
 */
#define MEMLOG_MEMORY_ALIGNMENT                     4096

/**
@}
*/

/** @addtogroup MEMLOG_ERR_CODE
 *
 * @brief
 *  Base error code for the MEMLOG module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   Invalid Argument
 */
#define MEMLOG_EINVAL                               SYSLIB_ERRNO_MEMLOG_BASE-1

/**
 * @brief   Out of Memory.
 */
#define MEMLOG_ENOMEM                               SYSLIB_ERRNO_MEMLOG_BASE-2

/**
 * @brief   Internal error
 */
#define MEMLOG_EINTERNAL                            SYSLIB_ERRNO_MEMLOG_BASE-3

/**
 * @brief   MsgCome error
 */
#define MEMLOG_EMSGCOM                              SYSLIB_ERRNO_MEMLOG_BASE-4

/**
 * @brief   Not ready
 */
#define MEMLOG_ENOTREADY                            SYSLIB_ERRNO_MEMLOG_BASE-5

/**
@}
*/

/** @addtogroup MEMLOG_ENUM
 @{ */

/**
 * @brief
 *  Memory allocation mode
 *
 * @details
 *  The MEMLOG module allocates memory for multiple purposes. There are certain considerations
 *  which need to be accounted for while allocating memory.
 */
typedef enum Memlog_MemAllocMode
{
    /**
     * @brief  This mode is used when the requesting memory for the accumulator list.
     * This memory needs to be cache coherent as this is allocated and used to program the
     * accumulator. The MSGCOM module does NOT perform any cache operations on this list
     * for performance.
     *
     * In the DSP realm:
     * - Allocate memory from the local L2 memory and return the global address
     * In the ARM realm:
     * - Allocate memory from the HPLIB memory pools.
     */
    Memlog_MemAllocMode_CACHE_COHERENT  = 0x1,

    /**
     * @brief   This mode is used to request memory for internal runtime data structures.
     */
    Memlog_MemAllocMode_INTERNAL        = 0x2
}Memlog_MemAllocMode;

/**
 * @brief
 *  MEMLOG Execution Realm enumeration
 *
 * @details
 *  The enumeration describes the realm in which the MEMLOG instance is executing.
 */
typedef enum Memlog_ExecutionRealm
{
    /**
     * @brief   MEMLOG instance is executing in the DSP realm.
     */
    Memlog_ExecutionRealm_DSP  = 0x1,

    /**
     * @brief   MEMLOG instance is executing in the ARM realm.
     */
    Memlog_ExecutionRealm_ARM  = 0x2
}Memlog_ExecutionRealm;

/**
@}
*/

/** @addtogroup MEMLOG_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Opaque MEMLOG Instance handle
 */
typedef void*   Memlog_InstHandle;

/**
 * @brief
 *  Opaque MEMLOG Channel handle
 */
typedef void*   Memlog_ChHandle;

/**
 * @brief
 *  Opaque MEMLOG channel controller handle
 */
typedef void*   Memlog_CtrlHandle;

/**
@}
*/

/** @addtogroup MEMLOG_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory required by the memory logging module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  size
 *      Number of bytes of memory to be allocated
 *
 *  @retval
 *      Pointer to the allocated memory
 */
typedef void* (*Osal_MemlogMalloc)(Memlog_MemAllocMode mode, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to free memory which was allocated by the memory logging module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the allocated memory
 *  @param[in]  size
 *      Number of bytes of memory which were allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void  (*Osal_MemlogFree)(Memlog_MemAllocMode mode, void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory for the memory region.
 *
 *  @param[in]  name
 *      Name of the memory region being inserted
 *  @param[in]  memRegType
 *      Memory region type being specified.
 *  @param[in]  numBytes
 *      Number of bytes of memory being allocated
 *
 *  @retval
 *      Pointer to the allocated memory region
 */
typedef void* (*Osal_MemlogMallocMemoryRegion)(char* name, Resmgr_MemRegionType memRegType, uint32_t numBytes);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to free memory for the memory region.
 *
 *  @param[in]  name
 *      Name of the memory region being inserted
 *  @param[in]  memRegType
 *      Memory region type being specified.
 *  @param[in]  ptr
 *      Pointer to the memory region memory.
 *  @param[in]  numBytes
 *      Number of bytes of memory being cleaned up
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_MemlogFreeMemoryRegion)(char* name, Resmgr_MemRegionType memRegType, void* ptr, uint32_t numBytes);

/**
 *  @b Description
 *  @n
 *      The function is used by the memory logging module to indicate that memory access is about
 *      to begin and the contents of the cache should be invalidated
 *
 *  @param[in]  ptr
 *      Pointer to the memory
 *  @param[in]  size
 *      Size of memory
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_MemlogBeginMemAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      The function is used by the memory logging module to indicate that memory has been accessed
 *      and so the contents of the cache should be written back.
 *
 *  @param[in]  ptr
 *      Pointer to the memory
 *  @param[in]  size
 *      Size of memory
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_MemlogEndMemAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
typedef void* (*Osal_MemlogEnterSingleCoreCS)(void);

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque handle to the criticial section object
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_MemlogExitSingleCoreCS)(void* csHandle);

/**
@}
*/

/** @addtogroup MEMLOG_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  MEMLOG channel configuration
 *
 * @details
 * This structure holds configuration needed to create memory logging channel.
 */
typedef struct Memlog_ChannelCfg
{
    /**
     * @brief   Name of the memory logging channel.
     */
    char                    name[MEMLOG_MAX_CHAR];

    /**
     * @brief   MEMLOG Instance handle associated with the channel. Channels exist
     * only in the context of the MEMLOG instance.
     */
    Memlog_InstHandle       memlogInstHandle;

    /**
     * @brief   QMSS memory Region to be used to create FAPI tracing capture
     *          heap.
     */
    Qmss_MemRegion          memRegion;

    /**
     * @brief   Memory logging buffer memory base address.It should be virtual address
     *          on ARM.
     */
    uint32_t                memBaseAddr;

    /**
     * @brief   Total logging buffer memory size in bytes.
     */
    uint32_t                memSize;

    /**
     * @brief   Size of logging buffer. It is L2 cache aligned.
     *          It is normally the MTU size of the transmit interface.
     */
    uint32_t                bufferSize;

    /**
     * @brief   Number of descriptors from a specified memory region that can be used
                for this memory logging channel.
     */
    uint32_t                numPktDescs;
}Memlog_ChannelCfg;

/**
 * @brief
 *  Memory logger Information.
 *
 * @details
 *  The structure describes the memory logger information shared between
 *  entities on DSP and ARM.
 */
typedef struct MemLog_LoggerInfo
{
    /**
     * @brief   Execution Realm in which the MEMLOG is executing.
     */
    Memlog_ExecutionRealm realm;

    /**
     * @brief   Memory logging buffer memory base address.
     */
    uint32_t            memBase;

    /**
     * @brief   Total logging buffer memory size.
     */
    //uint32_t            numLogBuffers;
    uint32_t            memSize;

    /**
     * @brief   Size of logging buffer. It is L2 cache aligned.
     *          It is normally the MTU size of the transmit interface.
     */
    uint32_t            bufferSize;

    /**
     * @brief   Hardware Queue used to start/stop memory logging at
     *          run-time.
     */
    Qmss_QueueHnd       syncQueue;

    /**
     * @brief   Packet used for Memory logging Sync.
     */
    void*               syncPkt;
}MemLog_LoggerInfo;

/**
 * @brief
 *  MEMLOG instance configuration
 *
 * @details
 *  Each processing entity (DSP core or ARM process) needs to create a MEMLOG
 *  instance. MEMLOG channels are created and exist only within each instance.
 */
typedef struct Memlog_InstCfg
{
    /**
     * @brief   Database handle associated with the MEMLOG instance. Information
     * for the MEMLOG channels created & found using this instance will refer only
     * to this specific database
     */
    Name_DBHandle                   databaseHandle;

    /**
     * @brief   This is the handle to the PKTLIB instance. Packets which are
     * received and sent via the MEMLOG channels belong to this PKTLIB instance
     */
    Pktlib_InstHandle               pktlibInstHandle;

    /**
     * @brief   This is the MSGCOM instance handle which is used to create and manage all
     * the MSGCOM channels used by MEMLOG.
     */
    Msgcom_InstHandle               msgcomInstHandle;

    /**
     * @brief   Server execution realm
     */
    Memlog_ExecutionRealm          realm;

    /**
     * @brief  OSAL API to allocate memory
     */
    Osal_MemlogMalloc               malloc;

    /**
     * @brief  OSAL API to clean memory
     */
    Osal_MemlogFree                 free;

    /**
     * @brief   OSAL Enter Critical Section
     */
    Osal_MemlogEnterSingleCoreCS    enterCS;

    /**
     * @brief   OSAL Exit Critical Section
     */
    Osal_MemlogExitSingleCoreCS     exitCS;
}Memlog_InstCfg;


/**
@}
*/

/***************************************************************************
 **************************** EXTERN Definitions ***************************
 ***************************************************************************/
/* MEMLOG channle APIs */
extern Memlog_ChHandle Memlog_create(Memlog_ChannelCfg* ptrChannelConfig, int32_t* errCode);
extern int32_t memlog_delete(Memlog_ChHandle* memlogHandle, int32_t* errCode);

/* MEMLOG controller APIs */
extern Memlog_CtrlHandle Memlog_createMemlogController(Memlog_InstHandle memlogInstHandle,
                                                       char* channelName, int32_t* errCode);
extern int32_t  Memlog_deleteMemlogController(Memlog_CtrlHandle MemlogCtrlHandle, int32_t* errCode);
extern int32_t Memlog_stopLogging(Memlog_CtrlHandle memlogCtrlHandle, int32_t* errCode);
extern int32_t Memlog_startLogging(Memlog_CtrlHandle memlogCtrlHandle, int32_t* errCode);
extern int32_t Memlog_saveMetaInfoInFile(Memlog_ChHandle memlogChanHandle, FILE* fp, int32_t* errCode);
extern int32_t Memlog_getMemlogChanInfo(Memlog_CtrlHandle memlogCtrlHandle,
                                            MemLog_LoggerInfo* memLogInfo, int32_t* errCode);
extern int32_t Memlog_getMemLogChanName(Memlog_CtrlHandle memlogCtrlHandle, char* memlogChanName, int32_t*  errCode);
extern int32_t Memlog_saveLog(Memlog_ChHandle memlogHandle, Ti_Pkt* ptrMessage);

/* MEMLOG instance APIs */
extern Memlog_InstHandle Memlog_createInstance(Memlog_InstCfg* ptrInstCfg, int32_t* errCode);
extern int32_t Memlog_deleteInstance(Memlog_InstHandle instHandle, int32_t* errCode);

#ifdef __cplusplus
}
#endif

#endif /*__MEELOG_H__ */

