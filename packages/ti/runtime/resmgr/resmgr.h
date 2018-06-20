/**
 *   @file  resmgr.h
 *
 *   @brief
 *      Header file which specifies the resource manager.
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

/**  @mainpage Resource Management
 *
 *   @section intro  Introduction
 *
 *   Applications can execute in the context of different execution realms (DSP or ARM).
 *   Each application will require access to hardware resources (such as Queues, Memory Regions,
 *   Accumulated channels etc) The SYSLIB resource management layer has been introduced to
 *   avoid any resource conflicts.
 *
 *   Application specify their requests upfront in a configuration table which is then passed to the
 *   module. The module parses the table and determines if the resource allocations can succeed or not
 *   and provides an appropriate response. In order to do the resource management module will need to
 *   communicate with the SYSRM server executing on the ARM. The module thus also provides an RMv2
 *   Transport interface which allows this communication to work.
 */

/** @defgroup RES_MGR_API   Resource Management
 */
#ifndef __RES_MGR_H__
#define __RES_MGR_H__

/* MCSDK Include Files */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm_services.h>

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup RES_MGR_DATA_STRUCTURE            Resource Management Data Structures
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_SYMBOL                    Resource Management Defined Symbols
@ingroup RES_MGR_API
*/
/**
@defgroup RES_MGR_FUN                       Resource Management Exported Functions
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_ENUM                      Resource Management Enumerations
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_INTERNAL_FUN              Resource Management Internal Functions
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_ARM_INTERNAL_FUN          Resource Management ARM Internal Functions
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_DSP_INTERNAL_FUN          Resource Management DSP Internal Functions
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_OSAL_API                  Resource Management OS Adaptation Layer
@ingroup  RES_MGR_API
*/
/**
@defgroup RES_MGR_ERR_CODE                  Resource Management Error Codes
@ingroup  RES_MGR_API
*/

/** @addtogroup RES_MGR_SYMBOL
 @{ */

/**
 * @brief   Base Hardware semaphore: The resource manager will assume all semaphores
 * before this are not available for resource allocation. We need to have some hardware
 * semaphores which are shared across multiple cores for common components such as
 * CPPI, QMSS etc.
 */
#define RESMGR_BASE_HW_SEM                         12

/**
 * @brief   Maximum number of characters
 */
#define RESMGR_MAX_CHAR                            32

/**
@}
*/

/** @addtogroup RES_MGR_ERR_CODE
 *
 * @brief
 *  Base error code for the RESMGR module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   Invalid Argument
 */
#define RESMGR_EINVAL                               SYSLIB_ERRNO_RESMGR_BASE-1

/**
 * @brief   Out of Memory.
 */
#define RESMGR_ENOMEM                               SYSLIB_ERRNO_RESMGR_BASE-2

/**
 * @brief   Number of accumulated channels exceed the max allowed
 */
#define RESMGR_EACC_CHANNEL_LIMIT                   SYSLIB_ERRNO_RESMGR_BASE-3

/**
 * @brief   Number of system interrupts exceed the max allowed
 */
#define RESMGR_ESYS_INTERRUPT_LIMIT                 SYSLIB_ERRNO_RESMGR_BASE-4

/**
 * @brief   Number of queue pends exceed the max allowed
 */
#define RESMGR_EQUEUE_PEND_LIMIT                    SYSLIB_ERRNO_RESMGR_BASE-5

/**
 * @brief   Number of hardware semaphores exceed the max allowed
 */
#define RESMGR_EHW_SEM_LIMIT                        SYSLIB_ERRNO_RESMGR_BASE-6

/**
 * @brief   Internal error
 */
#define RESMGR_EINTERNAL                            SYSLIB_ERRNO_RESMGR_BASE-7

/**
@}
*/

/** @addtogroup RES_MGR_ENUM
 @{ */

/**
 * @brief
 *  Memory allocation mode
 *
 * @details
 *  The enumeration describes the memory allocation mode.
 */
typedef enum Resmgr_MallocMode
{
    /**
     * @brief   Allocated memory could be local and need not be visible
     * across different cores.
     */
    Resmgr_MallocMode_LOCAL     =   0x1,

    /**
     * @brief   Allocated memory must be visible across different cores
     */
    Resmgr_MallocMode_GLOBAL    =   0x2
}Resmgr_MallocMode;

/**
 * @brief
 *  Memory Region Type
 *
 * @details
 *  Memory Region can have descriptors which are located in either of the following
 *  memory types.
 */
typedef enum Resmgr_MemRegionType
{
    /**
     * @brief   Descriptors are located in shared memory.
     */
    Resmgr_MemRegionType_MSMC     = 0x1,

    /**
     * @brief   Descriptors are located in local memory.
     */
    Resmgr_MemRegionType_LOCAL    = 0x2,

    /**
     * @brief   Descriptors are located in external memory.
     */
    Resmgr_MemRegionType_DDR3     = 0x3
}Resmgr_MemRegionType;

/**
 * @brief
 *  Memory Region Linking RAM Request
 *
 * @details
 *  The enumeration describes the linking RAM to which the requested memory region
 *  needs to be placed.
 */
typedef enum Resmgr_MemRegionLinkingRAM
{
    /**
     * @brief   Memory region descriptors should be placed in internal linking RAM
     */
    Resmgr_MemRegionLinkingRAM_INTERNAL = 0x1,

    /**
     * @brief   No specific requirements and so the descriptors will be placed in external linking RAM
     */
    Resmgr_MemRegionLinkingRAM_DONT_CARE = 0x2
}Resmgr_MemRegionLinkingRAM;

/**
 * @brief
 *  Execution Realm
 *
 * @details
 *  The enumeration describes the execution realm in which the resource manager
 *  services are being used.
 */
typedef enum Resmgr_ExecutionRealm
{
    /**
     * @brief   Resource Manager services executing on the DSP.
     */
    Resmgr_ExecutionRealm_DSP       = 0x1,

    /**
     * @brief   Resource Manager services executing on the ARM.
     */
    Resmgr_ExecutionRealm_ARM       = 0x2
}Resmgr_ExecutionRealm;

/**
@}
*/

/** @addtogroup RES_MGR_DATA_STRUCTURE
 @{ */

/**
 * @brief   Opaque handle which returns the system configuration block.
 */
typedef void*   Resmgr_SysCfgHandle;

/**
@}
*/

/** @addtogroup RES_MGR_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory required by the resource manager module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  size
 *      Number of bytes of memory to be allocated
 *
 *  @retval
 *      Pointer to the allocated memory
 */
typedef void* (*Osal_ResmgrMalloc)(Resmgr_MallocMode mode, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to free memory which was allocated by the resource manager module.
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
typedef void  (*Osal_ResmgrFree)(Resmgr_MallocMode mode, void* ptr, uint32_t size);

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
typedef void* (*Osal_ResmgrMallocMemoryRegion)(char* name, Resmgr_MemRegionType memRegType, uint32_t numBytes);

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
typedef void  (*Osal_ResmgrFreeMemoryRegion)(char* name, Resmgr_MemRegionType memRegType, void* ptr, uint32_t numBytes);

/**
 *  @b Description
 *  @n
 *      The function is used by the resource manager to indicate that memory access is about
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
typedef void (*Osal_ResmgrBeginMemAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      The function is used by the resource manager to indicate that memory has been accessed
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
typedef void (*Osal_ResmgrEndMemAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore. The semaphore is used by the underlying
 *      RM layer to ensure protection against multiple threads.
 *
 *  @retval
 *      Opaque semaphore handle
 */
typedef void* (*Osal_ResmgrCreateSem)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_ResmgrPendSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_ResmgrPostSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_ResmgrDeleteSem)(void* semHandle);

/**
@}
*/

/** @addtogroup RES_MGR_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Memory Region Configuration
 *
 * @details
 *  The structure describes the memory region configuration request.
 */
typedef struct Resmgr_MemRegionCfg
{
    /**
     * @brief   Memory Region Name
     */
    char                            name[RESMGR_MAX_CHAR + 1];

    /**
     * @brief   Memory Region Type: Local, MSMC or DDR3
     */
    Resmgr_MemRegionType            type;

    /**
     * @brief   Linking RAM preferences on where the memory region should be located
     */
    Resmgr_MemRegionLinkingRAM      linkingRAM;

    /**
     * @brief   Number of descriptors required to be present in the specified memory
     * region.
     */
    uint32_t                        numDesc;

    /**
     * @brief   Size of the descriptors in the specified memory region.
     */
    uint32_t                        sizeDesc;
}Resmgr_MemRegionCfg;

/**
 * @brief
 *  Memory Region Configuration response.
 *
 * @details
 *  This structure is used to describe the memory configuration response which includes
 *  the allocated memory region identifier.
 */
typedef struct Resmgr_MemRegionResponse
{
    /**
     * @brief   Memory Region Handle which was allocated for the request.
     */
    Qmss_MemRegion              memRegionHandle;

    /**
     * @brief   Start Index within the QM Linking RAM for the request.
     */
    uint32_t                    startIndex;

    /**
     * @brief   Linking RAM where the memory region descriptors have been placed
     */
    Resmgr_MemRegionLinkingRAM  linkingRAM;

    /**
     * @brief   Copy of the memory region request.
     */
    Resmgr_MemRegionCfg         memRegionCfg;
}Resmgr_MemRegionResponse;

/**
 * @brief
 *  Accumulator Configuration Response
 *
 * @details
  *  This structure is used to describe the response for the accumulator channel
  *  allocations.
 */
typedef struct Resmgr_AccChannelResponse
{
    /**
     * @brief   Allocated Accumulator Channel.
     */
    uint8_t                 accChannel;

    /**
     * @brief   Queue Information to which the accumulator channel is mapped to.
     */
    Qmss_Queue              queue;

    /**
     * @brief   QMSS PDSP identifier associated with the accumulated channel.
     */
    Qmss_PdspId             pdspId;

    /**
     * @brief   Event number to which the accumulator channel is mapped to. In the
     * case of the ARM this is the GIC interrupt which is mapped. In the case of the
     * DSP this is the INTC event identifier.
     */
    uint32_t                eventId;
}Resmgr_AccChannelResponse;

 /**
  * @brief
  *  Resource Manager Queue Pend Response.
  *
  * @details
  *  This structure is used to describe the response for the Queue Pend (Direct
  *  Interrupt Queue) allocations.
  */
typedef struct Resmgr_QpendQueueResponse
{
    /**
     * @brief   Allocated queue pend queue which is to be used.
     */
    Qmss_Queue              queue;

    /**
     * @brief   In the DSP realm; queue pend interrupts are mapped via the CPINTC module
     * This is the CPINTC identifier through which the interrupt is being routed. On the
     * ARM this is NOT used since interrupts are routed via the GIC controller.
     */
    int32_t                 cpIntcId;

    /**
     * @brief   This is the system interrupt to which the queue pend is mapped to.
     * This field is *only* applicable in the DSP realm and is NOT used in the
     * ARM realm.
     */
    int32_t                 systemInterrupt;

    /**
     * @brief   Host Interrupt to which the direct interrupt queues are mapped to.
     * - DSP the queue pend queues are wired via the CPINTC block and so this is
     *   the host interrupt to which the system interrupt will be routed to.
     * - ARM the queue pend queues are wired via the GIC block and so this is the
     *   GIC event identifier.
     */
    uint32_t                hostInterrupt;
}Resmgr_QpendQueueResponse;

/**
 * @brief
 *  Resource Manager configuration
 *
 * @details
 *  Each core is responsible for requesting platform resources using this configuration
 *  data structure. The platform configuration code will use the requests to determine
 *  the resource allocation and will populate the allocated resources.
 */
typedef struct Resmgr_ResourceCfg
{
    /****************************************************************************
     ************************** REQUEST Configuration ***************************
     ****************************************************************************/

    /**
     * @brief  This is the number of interrupt controller outputs which are required
     * by the requesting core. Certain interrupts are routed via the CPINTC module which
     * translates system interrupts to host interrupts. The system interrupt number
     * is well known however the select of the Host Interrupt is core specific.
     */
    uint32_t                    numCPINTCInterrupts;

    /**
     * @brief  This is the number of high priority accumulator channels requested.
     * This count should include all the MSGCOM accumulated channels which will be
     * opened by the application.
     */
    uint32_t                    numAccumalatorChannels;

    /**
     * @brief  This is the number of hardware semaphores requested. This count does
     * not need to reflect the number of hardware semaphores requested by the system
     * components. This should *only* be used by the application to request for
     * hardware semaphores which are required for their own purposes.
     */
    uint32_t                    numHwSemaphores;

	/**
     * @brief  This is the number of queue pend queues which are requested. This
     * count should include all the MSGCOM Direct Interrupt channels which will
     * be opened by the application.
     */
    uint32_t                    numQpendQueues;

    /**
     * @brief  This is the memory region requirements which are specfified by the
     * requesting core and populated by the MASTER core.
     */
    Resmgr_MemRegionCfg         memRegionCfg[QMSS_MAX_MEM_REGIONS];

    /****************************************************************************
     ************************* Response Configuration ***************************
     ****************************************************************************/

    /**
     * @brief  CPINTC Response Section:
     * This is the response to the number of CPINTC interrupts which had been
     * requested. The response maps the system interrupt to the host interrupt.
     */
    uint32_t                    cpIntcHostIntrResponse[18];

    /**
     * @brief  Accumulator Channel Response Section:
     * This is the response to the number of accumulator channels requested. The
     * response has a list of all the allocated accumulator channels which can be
     * used by each core.
     */
    Resmgr_AccChannelResponse   accChannelResponse[8];

    /**
     * @brief  Hardware Semaphore Response Section:
     * This is the response to the hardware semaphores which have been requested.
     */
    uint32_t                    hwSemResponse[32];

     /**
      * @brief  Queue Pend Response Section:
      * This is the response to the total number of queue pend queues which were
      * requested. The response returns the host interrupt mapping
      */
    Resmgr_QpendQueueResponse   qPendResponse[64];

    /**
     * @brief   Memory Region Response Section:
     * This is the response section to the requested memory regions.
     */
    Resmgr_MemRegionResponse    memRegionResponse[QMSS_MAX_MEM_REGIONS];
}Resmgr_ResourceCfg;

/**
 * @brief
 *  DSP system configuration
 *
 * @details
 *  The structure describes the configuration for the DSP system
 */
typedef struct Resmgr_DSPSystemCfg
{
    /**
     * @brief  Shared Memory address which is used by the ARM RM Server and DSP RM clients
     * to communuicate with each other.
     */
    uint32_t        sharedMemAddress;

    /**
     * @brief  Size of the shared memory block.
     */
    uint32_t        sizeSharedMemory;

    /**
     * @brief  IPC Source identifier which is used to generate interrupts to the SYSRM server
     * executing on the ARM.
     */
    uint32_t        sourceId;

    /**
     * @brief  ARM core identifier on which the SYSRM server is executing.
     */
    uint32_t        armCoreId;
}Resmgr_DSPSystemCfg;

/**
 * @brief
 *  Resource Manager System configuration
 *
 * @details
 *  System configuration which defines the resource requirements for all the cores (DSP
 *  and ARM) involved in the system.
 */
typedef struct Resmgr_SystemCfg
{
    /**
     * @brief  Realm in which the module is executing.
     */
    Resmgr_ExecutionRealm           realm;

    /**
     * @brief  Core identifier on which the module is executing
     */
    uint8_t                         coreId;

    /**
     * @brief  Name of the RM Client.
     */
    char                            rmClient[RM_NAME_MAX_CHARS];

    /**
     * @brief  Name of the RM Server.
     */
    char                            rmServer[RM_NAME_MAX_CHARS];

    /**
     * @brief  DSP system configuration which is applicable only in the DSP realm.
     * This parameter is ignored in the ARM realm.
     */
    Resmgr_DSPSystemCfg             dspSystemCfg;

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
     * @brief  OSAL API to indicate that the memory access is complete and cache contents
     * needs to be written back.
     */
    Osal_ResmgrBeginMemAccess       beginMemAccess;

    /**
     * @brief  OSAL API to indicate that the memory access is complete and cache contents
     * needs to be written back.
     */
    Osal_ResmgrEndMemAccess         endMemAccess;

    /**
     * @brief  OSAL API to create a semaphore used for protection against multiple threads
     */
    Osal_ResmgrCreateSem            createSem;

    /**
     * @brief  OSAL API to pend on semaphore used for protection against multiple threads
     */
    Osal_ResmgrPendSem              pendSem;

    /**
     * @brief  OSAL API to post the semaphore used for protection against multiple threads
     */
    Osal_ResmgrPostSem              postSem;

    /**
     * @brief  OSAL API to delete the semaphore used for protection against multiple threads
     */
    Osal_ResmgrDeleteSem            deleteSem;
}Resmgr_SystemCfg;

/**
@}
*/

/***************************************************************************
 **************************** EXTERN Definitions ***************************
 ***************************************************************************/

extern Resmgr_SysCfgHandle Resmgr_init(Resmgr_SystemCfg* ptrCfg, int32_t* errCode);
extern int32_t Resmgr_processConfig (Resmgr_SysCfgHandle sysRMHandle, Resmgr_ResourceCfg* ptrResCfg, int32_t* errCode);
extern int32_t Resmgr_deinit(Resmgr_SysCfgHandle sysRMHandle, int32_t* errCode);
extern Rm_ServiceHandle* Resmgr_getRMServiceHandle(Resmgr_SysCfgHandle hndSysConfig);
extern int32_t Resmgr_allocCustomResource (Resmgr_SysCfgHandle sysRMHandle,
                                           const char*         customName,
                                           uint32_t            numResources,
                                           uint32_t*           value,
                                           int32_t*            errCode);
extern int32_t Resmgr_freeCustomResource (Resmgr_SysCfgHandle sysRMHandle,
                                          const char*         customName,
                                          uint32_t            numResources,
                                          uint32_t            value,
                                          int32_t*            errCode);

extern int32_t Resmgr_allocSpecificCustomResource (Resmgr_SysCfgHandle sysRMHandle, const char* customName, uint32_t* value, int32_t* errCode); //fzm

extern int32_t Resmgr_nameServiceSet (Resmgr_SysCfgHandle sysRMHandle,
                                      char const * const  customName,
                                      uint32_t const      value,
                                      int32_t* const      errCode);
extern int32_t Resmgr_nameServiceGet (Resmgr_SysCfgHandle sysRMHandle,
                                      char const * const  customName,
                                      uint32_t* const     value,
                                      int32_t* const      errCode);
extern int32_t Resmgr_nameServiceDel (Resmgr_SysCfgHandle sysRMHandle,
                                      char const * const  customName,
                                      int32_t* const      errCode);


#ifdef __ARMv7
extern uint32_t Resmgr_getPASSVirtualAddress(Resmgr_SysCfgHandle hndSysConfig);
#endif

extern int32_t Resmgr_mapResource(Resmgr_SysCfgHandle sysRMHandle, const char* resourceName, uint32_t handle, int32_t* errCode);
extern int32_t Resmgr_getMappedResource(Resmgr_SysCfgHandle sysRMHandle,const char* resourceName, uint32_t* handle, int32_t* errCode);
extern int32_t Resmgr_unmapResource(Resmgr_SysCfgHandle sysRMHandle, const char* resourceName, int32_t* errCode);

#ifdef __cplusplus
}
#endif

#endif /*__RES_MGR_H__ */
