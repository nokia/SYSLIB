/**
 *   @file  domain.h
 *
 *   @brief
 *      Domain Header Files
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

#ifndef __DOMAIN_H__
#define __DOMAIN_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup DOMAIN_SYMBOL              Domain Defined Symbols
@ingroup DOMAIN_LIB_API
*/
/**
@defgroup DOMAIN_ERROR_CODE          Domain Error Codes
@ingroup DOMAIN_LIB_API
*/
/**
@defgroup DOMAIN_FUNCTION            Domain Exported Functions
@ingroup DOMAIN_LIB_API
*/
/**
@defgroup DOMAIN_DATA_STRUCTURE      Domain Data Structures
@ingroup DOMAIN_LIB_API
*/
/**
@defgroup DOMAIN_INTERNAL_FUN        Domain Internal Functions
@ingroup DOMAIN_LIB_API
*/
/**
@defgroup DOMAIN_OSAL_API            Domain OSAL Functions
@ingroup DOMAIN_LIB_API
*/

/** @addtogroup DOMAIN_ERROR_CODE
 *
 * @brief 
 *  Base error code for the DOMAIN module is defined in the 
 *  \include ti/runtime/common/syslib.h 
 * 
 @{ */

/** 
 * @brief 
 *  Error code: Invalid argument.
 */
#define DOMAIN_EINVAL                    SYSLIB_ERRNO_DOMAIN_BASE-1

/** 
 * @brief 
 *  Error code: No memory error.
 */
#define DOMAIN_ENOMEM                    SYSLIB_ERRNO_DOMAIN_BASE-2

/**
@}
*/

/** @addtogroup DOMAIN_DATA_STRUCTURE
 @{ */

/**
 * @brief   Opaque Domain SYSLIB Handle 
 */
typedef void*   Domain_SyslibHandle;

/**
 * @brief 
 *  Domain Memory allocation mode
 *
 * @details
 *  Enumerating which describes the domain memory allocation mode
 */
typedef enum Domain_MallocMode
{
    /**
     * @brief   Memory allocated should be global and accessible across
     * all the cores.
     */
    Domain_MallocMode_GLOBAL    =   0x1,

    /**
     * @brief   Memory allocated can be local to the core
     */
    Domain_MallocMode_LOCAL     =   0x2 
}Domain_MallocMode;

/**
 * @brief 
 *  Domain Log levels
 *
 * @details
 *  Enumeration which describes the log level of messages which are generated by 
 *  the domain module. 
 */
typedef enum Domain_LogLevel
{
    /**
     * @brief   Debug message
     */
    Domain_LogLevel_DEBUG   = 0x1,

    /**
     * @brief   Error message
     */
    Domain_LogLevel_ERROR   = 0x2
}Domain_LogLevel;

/**
 * @brief 
 *  Domain SYSLIB Task Module
 *
 * @details
 *  Enumeration which describes the SYSLIB components which require an execution context 
 */
typedef enum Domain_TaskContext
{
    /**
     * @brief   Name module: The name proxy/client requires a task context which allows the
     * names to be passed between different execution realms. This should be a low priority
     * task which can be executed in polled mode.  
     */
    Domain_TaskContext_NAME   = 0x1,

    /**
     * @brief   NETFP module: The NETFP client requires a task context which allows the NETFP
     * clients to communicate with the NETFP server. This should be a high priority task in 
     * the system since the NETFP calls to configured 3GPP channels can reside in the critical
     * fast path code.
     */
    Domain_TaskContext_NETFP  = 0x2,

    /**
     * @brief   DAT module: The DAT client requires a task context which allows the executes
     * the DAT client services and also executes the background actions. It is highly recommended
     * that the task be set to a priority just above the Idle task since the task will execute 
     * forever to handle any producers on consumers.
     */
    Domain_TaskContext_DAT    = 0x3    
}Domain_TaskContext;

/**
 *  @b Description
 *  @n  
 *      Domain SYSLIB Service task entry point. 
 *
 *  @param[in]  arg
 *      Argument to the SYSLIB service
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Domain_SyslibServiceTask)(uint32_t arg);

/**
@}
*/

/** @addtogroup DOMAIN_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to allocate memory for its
 *      internal use
 *
 *  @param[in]  size
 *      Number of bytes to allocate
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_DomainMalloc)(uint32_t size);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to free memory 
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes to freeup
 *
 *  @retval
 *      None
 */
typedef void (*Osal_DomainFree)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to allocate memory for data buffers which 
 *      are used to create internal heaps used by SYSLIB services. SYSLIB services require 
 *      data buffers to be CACHE aligned.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  size
 *      Number of bytes to allocate
 *  @param[in]  alignment
 *      Alignment 
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_DomainDataMalloc)(Domain_MallocMode mode, uint32_t size, uint32_t align);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to free memory which
 *      was allocated to the data buffers used by the internal heaps
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes to freeup
 *
 *  @retval
 *      None
 */
typedef void (*Osal_DomainDataFree)(Domain_MallocMode mode, void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to launch a SYSLIB service task
 *      with the specific argument.
 *
 *  @param[in]  level
 *      Log Level
 *  @param[in]  fmt
 *      Formatted String
 *  @param[in]  arg
 *      Argument list
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_DomainLog)(Domain_LogLevel level, char* fmt, va_list arg);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to launch a SYSLIB service task
 *      with the specific argument.
 *
 *  @param[in]  syslibServiceTask
 *      SYSLIB service task function entry point
 *  @param[in]  context
 *      SYSLIB Context for which the task is being created. This information can be 
 *      used to determine the priority of the task.
 *  @param[in]  arg
 *      Argument to be passed to the service task
 *
 *  @retval
 *      Opaque task handle
 */
typedef void* (*Osal_DomainTaskCreate)(Domain_SyslibServiceTask syslibServiceTask, Domain_TaskContext context, uint32_t arg);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to terminate a previously 
 *      created SYSLIB service task
 *
 *  @param[in]  syslibServiceTask
 *      SYSLIB service task function entry point
 *  @param[in]  arg
 *      Argument to be passed to the service task
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_DomainTaskDelete)(void* taskHandle);

/**
 *  @b Description
 *  @n  
 *      The function is used by the Domain module to relinquish a SYSLIB service task by
 *      "time" useconds.
 *
 *  @param[in]  time
 *      Time in usec
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_DomainTaskRelinquish)(uint32_t time);

/**
@}
*/
        

/** @addtogroup DOMAIN_DATA_STRUCTURE
 @{ */

/**
 * @brief 
 *  Domain OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the domain module 
 */
typedef struct Domain_OsalFxnTable
{
    /**
     * @brief   OSAL Malloc
     */
    Osal_DomainMalloc           malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_DomainFree             free;

    /**
     * @brief   OSAL Data Buffer Malloc
     */
    Osal_DomainDataMalloc       dataMalloc;

    /**
     * @brief   OSAL Data Buffer Free
     */
    Osal_DomainDataFree         dataFree;

    /**
     * @brief   OSAL Logging API
     */
    Osal_DomainLog              log;

    /**
     * @brief   OSAL Task create
     */
    Osal_DomainTaskCreate       taskCreate;

    /**
     * @brief   OSAL Task delete
     */
    Osal_DomainTaskDelete       taskDelete;

    /**
     * @brief   OSAL Task relinquish
     */
    Osal_DomainTaskRelinquish   taskRelinquish;
}Domain_OsalFxnTable;

/**
 * @brief 
 *  Domain Resource Manager OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the RESMGR module
 */
typedef struct Domain_ResmgrOsalFxnTable
{
    /**
     * @brief  OSAL API to allocate memory
     */
    Osal_ResmgrMalloc               malloc;

    /**
     * @brief  OSAL API to clean memory
     */
    Osal_ResmgrFree                 free;

    /**
     * @brief  OSAL API to allocate memory for the specific memory region
     */
    Osal_ResmgrMallocMemoryRegion   mallocMemoryRegion;

    /**
     * @brief  OSAL API to free memory for the specific memory region
     */
    Osal_ResmgrFreeMemoryRegion     freeMemoryRegion;

    /**
     * @brief  OSAL API to create a semaphore
     */
    Osal_ResmgrCreateSem            createSem;

    /**
     * @brief  OSAL API to delete the semaphore
     */
    Osal_ResmgrDeleteSem            deleteSem;

    /**
     * @brief  OSAL API to post the semaphore
     */
    Osal_ResmgrPostSem              postSem;

    /**
     * @brief  OSAL API to pend the semaphore
     */
    Osal_ResmgrPendSem              pendSem;

    /**
     * @brief  OSAL API to invalidate the cache
     */
    Osal_ResmgrEndMemAccess         beginMemAccess;
    
    /**
     * @brief  OSAL API to writeback the cache
     */
    Osal_ResmgrEndMemAccess         endMemAccess;
}Domain_ResmgrOsalFxnTable;

/**
 * @brief 
 *  Domain Named Resources OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the NAME module for the database 
 *  management. This table is only applicable for the DSP execution realm.
 */
typedef struct Domain_NameDBOsalFxnTable
{
    /**
     * @brief  OSAL API to allocate memory
     */
    Osal_NameDBMalloc                           malloc;

    /**
     * @brief  OSAL API to clean memory
     */
    Osal_NameDBFree                             free;

    /**
     * @brief  OSAL API to enter the critical section
     */
    Osal_NameEnterMultipleCoreCS                enterCS;

    /**
     * @brief  OSAL API to exit the critical section
     */
    Osal_NameExitMultipleCoreCS                 exitCS;

    /**
     * @brief  OSAL API to invalidate the cache
     */
    Osal_NameBeginMemoryAccess                  beginMemAccess;

    /**
     * @brief  OSAL API to writeback the cache
     */
    Osal_NameEndMemoryAccess                    endMemAccess;
}Domain_NameDBOsalFxnTable;

/**
 * @brief 
 *  Name Client/Proxy OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the NAME module to provide
 *  the name client and proxy services
 */
typedef struct Domain_NameOsalFxnTable
{
    /**
     * @brief  OSAL API to allocate memory
     */
    Osal_NameMalloc                             malloc;

    /**
     * @brief  OSAL API to clean memory
     */
    Osal_NameFree                               free;

    /**
     * @brief  OSAL API to enter the critical section
     */
    Osal_NameEnterCS                            enterCS;

    /**
     * @brief  OSAL API to exit the critical section
     */
    Osal_NameExitCS                             exitCS;

    /**
     * @brief  OSAL API to invalidate the cache
     */
    Osal_NameBeginMemoryAccess                  beginMemAccess;

    /**
     * @brief  OSAL API to writeback the cache
     */
    Osal_NameEndMemoryAccess                    endMemAccess;

    /**
     * @brief  OSAL API to create the semaphore
     */
    Osal_NameCreateSem                          createSem;

    /**
     * @brief  OSAL API to delete the semaphore
     */
    Osal_NameDeleteSem                          deleteSem;

    /**
     * @brief  OSAL API to post the semaphore
     */
    Osal_NamePostSem                            postSem;

    /**
     * @brief  OSAL API to pend on a semaphore
     */
    Osal_NamePendSem                            pendSem;
}Domain_NameOsalFxnTable;

/**
 * @brief 
 *  Domain PKTLIB OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the PKTLIB module
 */
typedef struct Domain_PktlibOsalFxnTable
{
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
}Domain_PktlibOsalFxnTable;

/**
 * @brief 
 *  Domain MSGCOM OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the MSGCOM module
 */
typedef struct Domain_MsgcomOsalFxnTable
{
    /**
     * @brief   OSAL Malloc
     */
    Osal_MsgcomMalloc               malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_MsgcomFree                 free;

    /**
     * @brief   OSAL Register ISR
     */
    Osal_MsgcomRegisterIsr          registerIsr;

    /**
     * @brief   OSAL Deregister ISR
     */
    Osal_MsgcomDeregisterIsr        deregisterIsr;

    /**
     * @brief   OSAL Disable System Interrupt
     */
    Osal_MsgcomDisableSysInt        disableSysInt;

    /**
     * @brief   OSAL Enable System Interrupt
     */
    Osal_MsgcomEnableSysInt         enableSysInt;

    /**
     * @brief   OSAL Enter Critical Section
     */
    Osal_MsgcomEnterSingleCoreCS    enterCS;

    /**
     * @brief   OSAL Exit Critical Section
     */
    Osal_MsgcomExitSingleCoreCS     exitCS;

    /**
     * @brief   OSAL Semaphore creation 
     */
    Osal_MsgcomCreateSem            createSem;

    /**
     * @brief   OSAL Semaphore deletion
     */
    Osal_MsgcomDeleteSem            deleteSem;

    /**
     * @brief   OSAL Post Semaphore
     */
    Osal_MsgcomPostSem              postSem;

    /**
     * @brief   OSAL Pend Semaphore
     */
    Osal_MsgcomPendSem              pendSem;
}Domain_MsgcomOsalFxnTable;

/**
 * @brief 
 *  Domain NETFP OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the NETFP module
 */
typedef struct Domain_NetfpOsalFxnTable
{
    /**
     * @brief  OSAL Malloc
     */
    Osal_NetfpMalloc                            malloc;

    /**
     * @brief  OSAL Free
     */
    Osal_NetfpFree                              free;

    /**
     * @brief  OSAL Begin memory access 
     */
    Osal_NetfpBeginMemoryAccess                 beginMemAccess;

    /**
     * @brief  OSAL End memory access 
     */
    Osal_NetfpEndMemoryAccess                   endMemAccess;

    /**
     * @brief  OSAL Enter single core critical section
     */
    Osal_NetfpEnterCS                           enterCS;

    /**
     * @brief  OSAL Exit single core critical section
     */
    Osal_NetfpExitCS                            exitCS;    

    /**
     * @brief  OSAL Create Semaphore
     */
    Osal_NetfpCreateSem                         createSem;

    /**
     * @brief  OSAL Delete Semaphore
     */
    Osal_NetfpDeleteSem                         deleteSem;

    /**
     * @brief  OSAL Post Semaphore
     */
    Osal_NetfpPostSem                           postSem;

    /**
     * @brief  OSAL Pend Semaphore
     */
    Osal_NetfpPendSem                           pendSem;
}Domain_NetfpOsalFxnTable;

/**
 * @brief 
 *  Domain DAT OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the DAT module
 */
typedef struct Domain_DatOsalFxnTable
{
    /**
     * @brief   OSAL Malloc
     */
    Osal_DatMalloc                  malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_DatFree                    free;

    /**
     * @brief   OSAL Malloc for local memory
     */
    Osal_DatMallocLocalMemory       mallocLocal;

    /**
     * @brief   OSAL Free for local memory
     */
    Osal_DatFreeLocalMemory         freeLocal;

    /**
     * @brief   OSAL Cache invalidate
     */
    Osal_DatBeginMemoryAccess       beginMemAccess;

    /**
     * @brief   OSAL Cache writeback
     */
    Osal_DatEndMemoryAccess         endMemAccess;

    /**
     * @brief   OSAL Enter Critical Section
     */
    Osal_DatEnterCS                 enterCS;

    /**
     * @brief   OSAL Exit Critical Section
     */
    Osal_DatExitCS                  exitCS;

    /**
     * @brief   OSAL Semaphore creation 
     */
    Osal_DatCreateSem               createSem;

    /**
     * @brief   OSAL Semaphore deletion
     */
    Osal_DatDeleteSem               deleteSem;

    /**
     * @brief   OSAL Post Semaphore
     */
    Osal_DatPostSem                 postSem;

    /**
     * @brief   OSAL Pend Semaphore
     */
    Osal_DatPendSem                 pendSem;
}Domain_DatOsalFxnTable;

/**
 * @brief 
 *  Domain MEMLOG OSAL call function
 *
 * @details
 *  OSAL call function table which is used by the DAT module
 */
typedef struct Domain_MemlogOsalFxnTable
{
    /**
     * @brief   OSAL Begin Memory access
     */
    Osal_MemlogMalloc               malloc;

    /**
     * @brief   OSAL End Memory access
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
}Domain_MemlogOsalFxnTable;

/**
 * @brief 
 *  Domain SYSLIB configuration
 *
 * @details
 *  The structure is used to describe the domain platform SYSLIB configuration.
 *  Each domain is initialized with the 
 */
typedef struct Domain_SyslibCfg
{
    /**
     * @brief  SYSLIB configuration passed by the root master
     */
    Root_SyslibConfig               rootSyslibCfg;

    /**
     * @brief  Domain Resource configuration which specifies the memory region to
     * be used for SYSLIB services
     */
    Resmgr_ResourceCfg              domainResourceCfg;

    /**
     * @brief  Domain OSAL call function table
     */
    Domain_OsalFxnTable             domainOsalFxnTable;

    /**
     * @brief  Domain RESMGR OSAL call function table
     */
    Domain_ResmgrOsalFxnTable       resmgrOsalFxnTable;

    /**
     * @brief  Domain Name database OSAL call function table
     */
    Domain_NameDBOsalFxnTable       nameDBOsalFxnTable;

    /**
     * @brief  Domain Name Proxy/Client OSAL call function table
     */
    Domain_NameOsalFxnTable         nameOsalFxnTable;

    /**
     * @brief  Domain PKTLIB OSAL call function table
     */
    Domain_PktlibOsalFxnTable       pktlibOsalFxnTable;

    /**
     * @brief  Domain MSGCOM OSAL call function table
     */
    Domain_MsgcomOsalFxnTable       msgcomOsalFxnTable;

    /**
     * @brief  Domain NETFP OSAL call function table
     */
    Domain_NetfpOsalFxnTable        netfpOsalFxnTable;

    /**
     * @brief  Domain DAT OSAL call function table
     */
    Domain_DatOsalFxnTable          datOsalFxnTable;

    /**
     * @brief  Domain MEMLOG OSAL call function table
     */
    Domain_MemlogOsalFxnTable       memlogOsalFxnTable;
}Domain_SyslibCfg;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

/* Initialization API: */
extern Domain_SyslibHandle Domain_initSyslibServices
(
    uint32_t            appId,
    Domain_SyslibCfg*   ptrDomainSyslibCfg,
    int32_t*            errCode
);

/* Deinitialization API: */
extern int32_t Domain_deinitSyslibServices (Domain_SyslibHandle domainHandle, int32_t* errCode);

/* API to get SYSLIB service handles: */
extern Resmgr_SysCfgHandle Domain_getSysCfgHandle (Domain_SyslibHandle domainHandle);
extern Name_DBHandle Domain_getDatabaseHandle (Domain_SyslibHandle domainHandle);
extern Pktlib_InstHandle Domain_getPktlibInstanceHandle (Domain_SyslibHandle domainHandle);
extern Msgcom_InstHandle Domain_getMsgcomInstanceHandle (Domain_SyslibHandle domainHandle);
extern Name_ClientHandle Domain_getNameClientInstanceHandle (Domain_SyslibHandle domainHandle);
extern Netfp_ClientHandle Domain_getNetfpClientInstanceHandle (Domain_SyslibHandle domainHandle);
extern Dat_ClientHandle Domain_getDatClientInstanceHandle (Domain_SyslibHandle domainHandle);
extern Memlog_InstHandle Domain_getMemlogInstanceHandle (Domain_SyslibHandle domainHandle);

#ifdef __cplusplus
}
#endif

#endif /* __DOMAIN_H__ */

