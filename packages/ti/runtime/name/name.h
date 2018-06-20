/**
 *   @file  name_db.h
 *
 *   @brief
 *      Common Header file for the Name library 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
 *  \par
 */

/** @defgroup NAME_API Name Module API
 */

#ifndef __NAME_H__
#define __NAME_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup NAME_SYMBOL                       Name Symbols 
@ingroup NAME_API
*/
/**
@defgroup NAME_ERROR_CODE                   Name Error codes
@ingroup NAME_API
*/
/**
@defgroup NAME_FUNCTION                     Name Exported Functions
@ingroup NAME_API
*/
/**
@defgroup NAME_NAMEASTRUCT                  Name Data Structures
@ingroup NAME_API
*/
/**
@defgroup NAME_OSAL_API                     Name OSAL Functions
@ingroup NAME_API
*/
/**
@defgroup NAME_INTERNAL_ARM_FUNCTION        Name Internal ARM Functions
@ingroup NAME_API
*/
/**
@defgroup NAME_INTERNAL_DSP_FUNCTION        Name Internal DSP Functions
@ingroup NAME_API
*/
/**
@defgroup NAME_INTERNAL_FUNCTION            Name Internal Functions
@ingroup NAME_API
*/
/**
@defgroup NAME_INTERNAL_SYMBOL              Name Internal Symbols 
@ingroup NAME_API
*/
/**
@defgroup NAME_INTERNAL_STRUCT              Name Internal Data Structures
@ingroup NAME_API
*/

/** @addtogroup NAME_SYMBOL
 @{ */

/**
 * @brief   Maximum number of characters supported
 */
#define NAME_MAX_CHAR                    32

/**
@}
*/

/** @addtogroup NAME_NAMEASTRUCT
 @{ */

/** 
 * @brief 
 *  Memory allocation mode
 *
 * @details
 *  The enumeration describes the memory allocation mode.
 */
typedef enum Name_MallocMode
{
    /**
     * @brief   Allocated memory should be allocated from a private core specific heap 
     */
    Name_MallocMode_LOCAL     =   0x1,

    /**
     * @brief   Allocated memory should be allocated from a global shared heap by other
     * DSP cores.
     */
    Name_MallocMode_GLOBAL    =   0x2 
}Name_MallocMode;

/**
@}
*/

/** @addtogroup NAME_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n  
 *     The function is used by the name database module to allocate memory
 *      
 *  @param[in]  mode
 *      Name memory allocation mode 
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_NameDBMalloc) (Name_MallocMode mode, uint32_t size, uint32_t alignment);

/**
 *  @b Description
 *  @n  
 *     The function is used by the name database module to cleanup memory. 
 *      
 *  @param[in]  mode
 *      Name memory allocation mode 
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_NameDBFree)(Name_MallocMode mode, void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *     The function is used by the name proxy/client module to allocate memory
 *      
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_NameMalloc) (uint32_t size, uint32_t alignment);

/**
 *  @b Description
 *  @n  
 *     The function is used by the name proxy/client module to cleanup memory. 
 *      
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_NameFree)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *     The function is used by the Name module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NameBeginMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *     The function is used by the Name module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NameEndMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n  
 *     The function is used by the Name module to protect its internal resources 
 *     from concurrent access within a single core. 
 *
 *  @retval
 *      Opaque critical section handle
 */
typedef void* (*Osal_NameEnterCS)(void);

/**
 *  @b Description
 *  @n  
 *     The function is used by the Name module to protect its internal 
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable 
 */
typedef void (*Osal_NameExitCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to protect the named resource database against concurrent
 *      access. The named resource database is shared across multiple cores and so the
 *      implementation should use a well known hardware semaphore which should be the same
 *      across all the cores.
 *
 *  @retval
 *      Opaque critical section handle
 */
typedef void* (*Osal_NameEnterMultipleCoreCS)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to release the named resource database against concurrent
 *      access. The named resource database is shared across multiple cores and so the
 *      implementation should use a well known hardware semaphore which should be the same
 *      across all the cores.
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  @retval
 *      Not applicable
 */
typedef void  (*Osal_NameExitMultipleCoreCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a Name job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee 
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
typedef void* (*Osal_NameCreateSem)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the Name job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle 
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_NamePendSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle 
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
typedef void (*Osal_NamePostSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *      Each JOB is associated with a unique semaphore.
 *
 *  @param[in]  semHandle 
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_NameDeleteSem)(void* semHandle);

/**
@}
*/

/** @addtogroup NAME_ERROR_CODE
 *
 * @brief 
 *  Base error code for the NAME module is defined in the 
 *  \include ti/runtime/common/syslib.h 
 * 
 @{ */

/** 
 * @brief 
 *  Error code: Invalid argument.
 */
#define NAME_EINVAL                     SYSLIB_ERRNO_NAME_BASE-1

/** 
 * @brief 
 *  Error code: No memory error.
 */
#define NAME_ENOMEM                     SYSLIB_ERRNO_NAME_BASE-2

/** 
 * @brief 
 *  Error code: Name services are not ready and available
 */
#define NAME_ENOTREADY                  SYSLIB_ERRNO_NAME_BASE-3

/** 
 * @brief 
 *  Error code: Name internal error
 */
#define NAME_EINTERNAL                  SYSLIB_ERRNO_NAME_BASE-4

/** 
 * @brief 
 *  Error code: No space available
 */
#define NAME_ENOSPACE                   SYSLIB_ERRNO_NAME_BASE-5

/** 
 * @brief 
 *  Error code: Internal JOSH error
 */
#define NAME_EJOSH                      SYSLIB_ERRNO_NAME_BASE-6

/** 
 * @brief 
 *  Error code: Name block is already in use
 */
#define NAME_EINUSE                      SYSLIB_ERRNO_NAME_BASE-7

/** 
 * @brief 
 *  Error code: Duplicate name
 */
#define NAME_EDUP                        SYSLIB_ERRNO_NAME_BASE-8

/** 
 * @brief 
 *  Error code: Name not found
 */
#define NAME_ENOTFOUND                   SYSLIB_ERRNO_NAME_BASE-9

/** 
 * @brief 
 *  Error code: Name functionality is not implemented.
 */
#define NAME_ENOTIMPL                    SYSLIB_ERRNO_NAME_BASE-10

/**
@}
*/

/** @addtogroup NAME_NAMEASTRUCT
 @{ */

/** 
 * @brief 
 *  Name Execution Realm enumeration
 *
 * @details
 *  The enumeration describes the realm in which the Name instance is executing.
 */
typedef enum Name_ExecutionRealm
{
    /**
     * @brief   Name instance is executing in the DSP realm.
     */
    Name_ExecutionRealm_DSP  = 0x1,

    /**
     * @brief   Name instance is executing in the ARM realm.
     */
    Name_ExecutionRealm_ARM  = 0x2
}Name_ExecutionRealm;

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* __NAME_H__ */

