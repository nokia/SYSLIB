/**
 *   @file  josh.h
 *
 *   @brief
 *      Header file for the Programmable Job Scheduler. The file
 *      exposes the data structures and exported API which are available for
 *      use by the application developers using the JOSH framework.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
 *  \par
 */

/** @defgroup JOSH_API JOSH API
 */

#ifndef __JOSH_H__
#define __JOSH_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/msgcom/msgcom.h>

/**
@defgroup JOSH_SYMBOL                   JOSH Symbols
@ingroup JOSH_API
*/
/**
@defgroup JOSH_ERROR_CODE               JOSH Error codes
@ingroup JOSH_API
*/
/**
@defgroup JOSH_FUNCTION                 JOSH API Functions
@ingroup JOSH_API
*/
/**
@defgroup JOSH_INTERNAL_FUN             JOSH Internal Functions
@ingroup JOSH_API
*/
/**
@defgroup JOSH_DATASTRUCT               JOSH Data Structures
@ingroup JOSH_API
*/
/**
@defgroup JOSH_OSAL_API                 JOSH OSAL Functions
@ingroup JOSH_API
*/

/** @addtogroup JOSH_SYMBOL
 @{ */

#if defined(_LITTLE_ENDIAN)

/**
 * @brief   Macro which converts 16 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU16(a)      (a)

/**
 * @brief   Macro which converts 32 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU32(a)      (a)

/**
 * @brief   Macro which converts 16 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU16(a)      (a)

/**
 * @brief   Macro which converts 32 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU32(a)      (a)

/**
 * @brief   Macro which converts 64 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU64(a)      (a)

/**
 * @brief   Macro which converts 64 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU64(a)      (a)

#elif defined(_BIG_ENDIAN)

/**
 * @brief   Macro which converts 16 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU16(a)      ( (((uint16_t)(a)>>8)&0xff) + (((uint16_t)(a)<<8)&0xff00) )

/**
 * @brief   Macro which converts 32 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU32(a)      ( (((uint32_t)(a)>>24)&0xff) + (((uint32_t)(a)>>8)&0xff00) + \
                                    (((uint32_t)(a)<<8)&0xff0000) + (((uint32_t)(a)<<24)&0xff000000) )

/**
 * @brief   Macro which converts 16 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU16(a)      josh_toNativeU16(a)

/**
 * @brief   Macro which converts 32 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU32(a)      josh_toNativeU32(a)

/**
 * @brief   Macro which converts 64 bit data received via the JOSH framework
 * to the Native endian format. This should be used before accessing any
 * ARGUMENT data.
 */
#define  josh_toNativeU64(a)      (uint64_t)((((uint64_t)(a)>>56)&0xff) + (((uint64_t)(a)>>48)&0xff00) + \
                                    (((uint64_t)(a)>>40)&0xff0000) + (((uint64_t)(a)>>32)&0xff000000) + \
                                    (((uint64_t)(a)<<56)&0xff00000000000000) + (((uint64_t)(a)<<48)&0x00ff000000000000) + \
                                    (((uint64_t)(a)<<40)&0x0000ff0000000000) + (((uint64_t)(a)<<32)&0x000000ff00000000) )

/**
 * @brief   Macro which converts 64 bit data sent via the JOSH framework
 * to the Remote endian format. This should be used before sending any
 * ARGUMENT data.
 */
#define  josh_toRemoteU64(a)      josh_toNativeU64(a)

#else
#error "Please define the Endianess (_BIG_ENDIAN) or (_LITTLE_ENDIAN)"
#endif

/**
 * @brief
 *  Maximum number of arguments that can be passed to the Jobs.
 */
#define JOSH_MAX_ARGS                   4

/**
 * @brief   Submit Job Type implies that a job has been submitted from the
 * MASTER to the SLAVE for processing.
 */
#define JOSH_PACKET_TYPE_SUBMIT         0x1

/**
 * @brief   Result Job Type implies that the job has been executed by the
 * SLAVE and the results are being posted back to the MASTER
*/
#define JOSH_PACKET_TYPE_RESULT         0x2

/**
@}
*/

/** @addtogroup JOSH_ERROR_CODE
 *
 * @brief
 *  Base error code for the JOSH module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief
 *  Error code: Invalid argument.
 */
#define JOSH_EINVAL                     SYSLIB_ERRNO_JOSH_BASE-1

/**
 * @brief
 *  Error code: No memory error.
 */
#define JOSH_ENOMEM                     SYSLIB_ERRNO_JOSH_BASE-2

/**
 * @brief
 *  Error code: No Message Available.
 */
#define JOSH_ENOMSG                     SYSLIB_ERRNO_JOSH_BASE-3

/**
 * @brief
 *  Error code: Internal Fatal Error.
 */
#define JOSH_EINTERNAL                  SYSLIB_ERRNO_JOSH_BASE-4

/**
 * @brief
 *  Error code: JOB not found
 */
#define JOSH_ENOJOB                     SYSLIB_ERRNO_JOSH_BASE-5

/**
 * @brief
 *  Error code: Semaphore not created
 */
#define JOSH_ENOSEM                     SYSLIB_ERRNO_JOSH_BASE-6

/**
 * @brief
 *  Error code: This implies that the JOSH node is still active with pending jobs
 *  and cannot be deleted.
 */
#define JOSH_EBUSY                      SYSLIB_ERRNO_JOSH_BASE-7

/**
@}
*/

/** @addtogroup JOSH_DATASTRUCT
 @{ */

/**
 * @brief
 *  This is the opaque handle which returns the handle to the JOSH processing node
 */
typedef void*   Josh_NodeHandle;

/**
 * @brief
 *  This is the opaque handle which is used for describing the jobs.
 */
typedef void*   Josh_JobHandle;

/**
 * @brief
 *  This is the opaque handle which is used for describing queues.
 */
typedef void*   Josh_QueueHandle;

/**
 * @brief
 *  Opaque handle which points to all the allocated arguments.
 */
typedef void*   Josh_ArgHandle;

/**
 * @brief
 *  This is the standard JOB prototype which is described by the JOSH
 *  framework.
 */
typedef uint32_t(*Josh_JobProtype)(void* arg1, void* arg2, void* arg3, void* arg4);

/**
 * @brief
 *  Enumeration which describes the processing node type
 *
 * @details
 *  Each processing node in the system can be one of the following types.
 */
typedef enum Josh_ProcessingNodeType
{
    /**
     * @brief   MASTER Nodes: These processing nodes are responsible for
     * sending jobs for execution.
     */
    Josh_ProcessingNodeType_MASTER      = 0x1,

    /**
     * @brief   SLAVE Nodes: These processing nodes only receive jobs and they
     * they only execute the jobs.
     */
    Josh_ProcessingNodeType_SLAVE       = 0x2
}Josh_ProcessingNodeType;

/**
 * @brief
 *  Enumeration which describes the argument types
 *
 * @details
 *  The properties of the arguments are describes here.
 */
typedef enum Josh_ArgumentType
{
    /**
     * @brief   Invalid Type
     */
    Josh_ArgumentType_INVALID            = 0x0,

    /**
     * @brief   Pass by value
     */
    Josh_ArgumentType_PASS_BY_VALUE      = 0x1,

    /**
     * @brief   Pass by reference
     */
    Josh_ArgumentType_PASS_BY_REF        = 0x2
}Josh_ArgumentType;

/**
 * @brief
 *  Argument Properties
 *
 * @details
 *  The structure is used to describe the arguments which are passed to the
 *  JOSH.
 */
typedef struct Josh_Argument
{
    /**
     * @brief   Argument Type
     */
    Josh_ArgumentType           type;

    /**
     * @brief   Length of the argument.
     */
    uint32_t                    length;

    /**
     * @brief   Argument buffer allocated by JOSH.
     */
    uint8_t*                    argBuffer;
}Josh_Argument;

/**
 * @brief
 *  Transport Interface
 *
 * @details
 *  The structure defines the custom transport interface used by JOSH.
 */
typedef struct Josh_TransportInterface
{
    /**
     * @brief   The allocation API which is invoked by the JOSH framework
     * to allocate a msgBuffer of the specified size. The function return NULL
     * on no memory else it will return a pointer to the data buffer. The
     * msgBuffer can be populated with any meta-information.
     */
    uint8_t*(*alloc)(uint32_t nodeId, int32_t size, void** msgBuffer);

    /**
     * @brief   The free API which is invoked by the JOSH framework
     * to free up a msgBuffer of the specified size. The msgBuffer is the
     * same as what was passed during allocation.
     */
    void (*free)(uint32_t nodeId, int32_t size, void* msgBuffer);

    /**
     * @brief   The get API which is invoked by the JOSH framework
     * when a packet is to be received. The function will return in 'ptrDataBuffer'
     * the actual message received. The 'msgBuffer' is also populated by the API
     * on a successful receive to any additional meta data. If there is no data
     * available the 'msgBuffer' should be set to NULL
     */
    void (*get)(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer);

    /**
     * @brief   The put API which is invoked by the JOSH framework when a packet
     * to be transmitted. The function will return <0 if there is an error else
     * 0 on success.
     */
    int (*put)(uint32_t nodeId, void* writerChannel, void* msgBuffer);
}Josh_TransportInterface;

/**
 * @brief
 *  Transport Configuration
 *
 * @details
 *  The configuration specifies the transport interface which is required by the
 *  JOSH processing node to communicate with its peer node.
 */
typedef struct Josh_TransportCfg
{
    /**
     * @brief   Custom Reader Channel Handle. This is NOT interpreted by the
     * JOSH framework and is passed to the Transport Interfaces as is.
     */
    void*                       readerChannel;

    /**
     * @brief   Custom Writer Channel Handle. This is NOT interpreted by the
     * JOSH framework and is passed to the Transport Interfaces as is.
     */
    void*                       writerChannel;

    /**
     * @brief   Transport Interface Functions.
     */
    Josh_TransportInterface     transport;
}Josh_TransportCfg;

/**
@}
*/

/** @addtogroup JOSH_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      Allocation API which is used by the JOSH framework to allocate
 *      memory for its internal purposes.
 *
 *  @param[in]  numBytes
 *      Number of bytes to be allocated
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Success -   Pointer to the allocated memory block
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_JoshMalloc)(uint32_t numBytes, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *      Memory cleanup API which is used by the JOSH framework to free
 *      memory which was previously allocated for its internal purposes.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes to be freed up
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_JoshFree)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to protect the JOSH resources from concurrent
 *      access across multiple threads.
 *
 *  @retval
 *      Opaque handle.
 */
typedef void* (*Osal_JoshEnterSingleCoreCS)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to release the critical section against
 *      concurrent access from multiple threads
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Opaque handle.
 */
typedef void (*Osal_JoshExitSingleCoreCS)(void* semHandle);

/**
 *  @b Description
   @n
 *      OSAL API which is used to create a semaphore associated with a JOB.
 *      If the jobs are executing in SYNC mode then once a JOB is submitted
 *      the callee will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
typedef void* (*Osal_JoshCreateSem)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_JoshPendSem)(void* semHandle);

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
typedef void (*Osal_JoshPostSem)(void* semHandle);

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
typedef void (*Osal_JoshDeleteSem)(void* semHandle);

/**
@}
*/

/** @addtogroup JOSH_DATASTRUCT
 @{ */

/**
 * @brief
 *  The structure describes the processing node configuration.
 *
 * @details
 *  Processing Nodes describes the entity on which the jobs will execute.
 *  The structure describes each processing node
 */
typedef struct Josh_NodeCfg
{
    /**
     * @brief   Node Identifier: Each JOSH Node is identified by a unique node identifer
     */
    uint32_t                        nodeId;

    /**
     * @brief   Transport configuration which is specific to the type.
     */
    Josh_TransportCfg               transportCfg;

    /**
     * @brief   OSAL Malloc
     */
    Osal_JoshMalloc                 malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_JoshFree                   free;

    /**
     * @brief   OSAL single core critical section enter
     */
    Osal_JoshEnterSingleCoreCS      enterCS;

    /**
     * @brief   OSAL single core critical section exit
     */
    Osal_JoshExitSingleCoreCS       exitCS;

    /**
     * @brief   OSAL create semaphore
     */
    Osal_JoshCreateSem              createSem;

    /**
     * @brief   OSAL post semaphore
     */
    Osal_JoshPostSem                postSem;

    /**
     * @brief   OSAL pend semaphore
     */
    Osal_JoshPendSem                pendSem;

    /**
     * @brief   OSAL delete semaphore
     */
    Osal_JoshDeleteSem              deleteSem;
}Josh_NodeCfg;

/**
@}
*/

/**********************************************************************************************
 * JOSH Exported API:
 **********************************************************************************************/
extern Josh_NodeHandle Josh_initNode(Josh_NodeCfg* ptrNode, int32_t* errCode);
extern int32_t Josh_deinitNode(Josh_NodeHandle nodeHandle, int32_t* errCode);
extern Josh_JobHandle Josh_findJobByAddress(Josh_NodeHandle nodeHandle, Josh_JobProtype jobFn);
extern Josh_JobHandle Josh_registerJob(Josh_NodeHandle nodeHandle, Josh_JobProtype jobFn);
extern Josh_ArgHandle Josh_addArguments (Josh_NodeHandle nodeHandle, Josh_Argument* args);
extern int32_t Josh_submitJob(Josh_JobHandle jobHandle, Josh_ArgHandle argHandle, uint32_t* result, int32_t* errCode);
extern int32_t Josh_submitAsyncJob(Josh_JobHandle jobHandle, Josh_ArgHandle argHandle, int32_t* errCode);
extern int32_t Josh_isJobCompleted(Josh_NodeHandle nodeHandle, int32_t jobId, uint32_t* jobResult, int32_t* errCode);
extern int32_t Josh_receive(Josh_NodeHandle nodeHandle, int32_t* errCode);
extern int32_t Josh_getArguments (Josh_NodeHandle nodeHandle, int32_t jobId, Josh_Argument* args);
extern int32_t Josh_freeJobInstance(Josh_NodeHandle nodeHandle, int32_t jobId);
extern uint32_t Josh_getTransactionId (uint8_t* ptrPacket);
extern uint32_t Josh_getNodeId (uint8_t* ptrPacket);
extern uint32_t Josh_getJobInstanceId (Josh_ArgHandle argHandle);
extern uint32_t Josh_getPacketType (uint8_t* ptrPacket);

#endif /* __JOSH_H__ */

