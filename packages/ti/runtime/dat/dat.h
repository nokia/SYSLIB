/**
 *   @file  dat.h
 *
 *   @brief
 *      Header file for the Debug & Trace library. The file
 *      exposes the data structures and exported API which are available for
 *      use by the application developers using the DAT framework.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */

/** @defgroup DAT_API DAT API
 */

#ifndef __DAT_H__
#define __DAT_H__

/* DAT verbosity Include Files. */
#include <ti/runtime/dat/dat_verbosity_levels.h>

#ifdef __ARMv7
/* UIA Include Files */
#include <ti/uiaplus/loggers/multistream/LoggerStreamer2.h>
#else
/* UIA Include Files */
#include <ti/uia/sysbios/LoggerStreamer2.h>
#endif

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/memlog/memlog.h>

/* XDC Include Files */
#include <xdc/std.h>
#include <xdc/runtime/Types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup DAT_SYMBOL                DAT Symbols
@ingroup DAT_API
*/
/**
@defgroup DAT_ERROR_CODE            DAT Error codes
@ingroup DAT_API
*/
/**
@defgroup DAT_FUNCTION              DAT Exported Functions
@ingroup DAT_API
*/
/**
@defgroup DAT_DATASTRUCT            DAT Data Structures
@ingroup DAT_API
*/
/**
@defgroup DAT_OSAL_API              DAT OSAL Functions
@ingroup DAT_API
*/
/**
@defgroup DAT_INTERNAL_FUNCTION     DAT Internal Functions
@ingroup DAT_API
*/
/**
@defgroup DAT_INTERNAL_SYMBOL       DAT Internal Symbols
@ingroup DAT_API
*/
/**
@defgroup DAT_INTERNAL_DATASTRUCT   DAT Internal Data Structures
@ingroup DAT_API
*/

/** @addtogroup DAT_SYMBOL
 @{ */

/**
 * @brief   Maximum number of characters supported
 */
#define DAT_MAX_CHAR                    32

/**
 * @brief   Maximum number of characters supported for trace object components
 */
#define DAT_TRACE_COMPONENT_MAX_CHAR    8

/**
 * @brief   Maximum number of consumers supported
 */
#define DAT_MAX_CONSUMER                8

/**
 * @brief   Maximum number of clients which can be attached to a DAT server
 */
#define DAT_MAX_CLIENTS                 16

/**
 * @brief   Maximum number of clients which can be attached to a DAT server
 */
#define DAT_CLIENT_MAX_DYNAMIC_INSTANCE   8

/**
 * @brief   Base instance id for dynamically allocated instances
 */
#define DAT_CLIENT_DYNAMIC_INSTANCE_BASE   32

/**
 * @brief   DAT verbosity mask size.
 */
#define DAT_MASK_SIZE                   16

/**
@}
*/
#ifdef __ARMv7
typedef Ptr LoggerStreamer2_Handle;
#endif
/** @addtogroup DAT_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to allocate memory.
 *
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Memory alignment requirments
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_DatMalloc) (uint32_t size, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to allocate memory in Local memory.
 *   This memory will be used to store trace verbosity settings which will be used in Dat_filter.
 *   Hence prefer fast memory access.
 *
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Memory alignment requirments
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_DatMallocLocalMemory) (uint32_t size, uint32_t alignment);


/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to invalidate the contents
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
typedef void (*Osal_DatBeginMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to writeback the contents
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
typedef void (*Osal_DatEndMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to protect its internal resources
 *     from concurrent access within a single core. DAT clients maintain consumers
 *     and producers which can be created/deleted in one thread and which are
 *     also handled in the background processing thread. These resources need
 *     to be protected against concurrent access.
 *
 *     NOTE: Please ensure that the implementation is NOT an interrupt disable/enable
 *     implementation since the critical section is held in the background processing
 *     thread. The implementation should use a software semaphore
 *
 *  @retval
 *      Opaque critical section handle
 */
typedef void* (*Osal_DatEnterCS)(void);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_DatExitCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to cleanup memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_DatFree)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to cleanup Local memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
typedef void (*Osal_DatFreeLocalMemory)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a DAT job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
typedef void* (*Osal_DatCreateSem)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the DAT job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_DatPendSem)(void* semHandle);

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
typedef void (*Osal_DatPostSem)(void* semHandle);

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
typedef void (*Osal_DatDeleteSem)(void* semHandle);

/**
@}
*/

/** @addtogroup DAT_ERROR_CODE
 *
 * @brief
 *  Base error code for the DAT module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief
 *  Error code: Invalid argument.
 */
#define DAT_EINVAL                     SYSLIB_ERRNO_DAT_BASE-1

/**
 * @brief
 *  Error code: No memory error.
 */
#define DAT_ENOMEM                     SYSLIB_ERRNO_DAT_BASE-2

/**
 * @brief
 *  Error code: DAT services are not ready and available
 */
#define DAT_ENOTREADY                  SYSLIB_ERRNO_DAT_BASE-3

/**
 * @brief
 *  Error code: DAT internal error
 */
#define DAT_EINTERNAL                  SYSLIB_ERRNO_DAT_BASE-4

/**
 * @brief
 *  Error code: No space available
 */
#define DAT_ENOSPACE                   SYSLIB_ERRNO_DAT_BASE-5

/**
 * @brief
 *  Error code: Internal JOSH error
 */
#define DAT_EJOSH                      SYSLIB_ERRNO_DAT_BASE-6

/**
 * @brief
 *  Error code: DAT block is already in use
 */
#define DAT_EINUSE                      SYSLIB_ERRNO_DAT_BASE-7

/**
 * @brief
 *  Error code: Duplicate name
 */
#define DAT_EDUP                        SYSLIB_ERRNO_DAT_BASE-8

/**
 * @brief
 *  Error code: DAT functionality is not implemented.
 */
#define DAT_ENOTIMPL                    SYSLIB_ERRNO_DAT_BASE-9
/**
 * @brief
 *  Error code: Does not exist.
 */
#define DAT_ENOTFOUND                   SYSLIB_ERRNO_DAT_BASE-10

/**
 * @brief
 *  Error code: Not engough resource (Josh) to handle the request
 */
#define DAT_ENORESOURCE                 SYSLIB_ERRNO_DAT_BASE-11

/**
 * @brief
 *  Error code: MsgCome error
 */
#define DAT_EMSGCOM                     SYSLIB_ERRNO_DAT_BASE-12
/**
@}
*/

/** @addtogroup DAT_DATASTRUCT
 @{ */

/**
 * @brief   DAT Client Handle
 */
typedef void*   Dat_ClientHandle;

/**
 * @brief   DAT Server Handle
 */
typedef void*   Dat_ServerHandle;

/**
 * @brief   DAT Producer Handle
 */
typedef void*   Dat_ProdHandle;

/**
 * @brief   DAT Consumer Handle
 */
typedef void*   Dat_ConsHandle;

/**
 * @brief   DAT Trace Object Handle
 */
typedef void*   Dat_TraceObjHandle;

/**
 * @brief
 *  Type of the trace event level.
 */
typedef uint32_t Dat_TraceLevel;

/**
 * @brief   DAT memory logging  instance Handle
 */
typedef void*   Dat_prodCtrlHandle;

/**
 * @brief
 *  DAT Execution Realm enumeration
 *
 * @details
 *  The enumeration describes the realm in which the DAT instance is executing.
 */
typedef enum Dat_ExecutionRealm
{
    /**
     * @brief   DAT instance is executing in the DSP realm.
     */
    Dat_ExecutionRealm_DSP  = 0x1,

    /**
     * @brief   DAT instance is executing in the ARM realm.
     */
    Dat_ExecutionRealm_ARM  = 0x2
}Dat_ExecutionRealm;

/**
 * @brief
 *  DAT Logging Levels
 *
 * @details
 *  Log Levels of messages which are generated by the DAT server
 */
typedef enum Dat_LogLevel
{
    /**
     * @brief   Debug message
     */
    Dat_LogLevel_DEBUG = 0x1,

    /**
     * @brief   Informational message
     */
    Dat_LogLevel_INFO = 0x2,

    /**
     * @brief   Error message
     */
    Dat_LogLevel_ERROR = 0x3
}Dat_LogLevel;

/**
 * @brief
 *  DAT producer type
 *
 * @details
 *  Enumeration used to indicate whether producer is UIA type or
 *  general-purpose.  General-purpose is used for statistics, measurements and
 *  any buffers that are not being written to by UIA APIs like Log_iwriteUC.
 */
typedef enum Dat_ProducerType
{
    /* Producer writes to UIA logger. */
    DAT_PRODUCER_UIA = 0,

    /* Producer writes to a user-created buffer. */
    DAT_PRODUCER_GENERAL_PURPOSE
}Dat_ProducerType;

/**
 * @brief
 *  DAT Consumer Status
 *
 * @details
 *  Consumers can be in any of the following states
 */
typedef enum Dat_ConsumerStatus
{
    /**
     * @brief   Consumer is connected and attached to a producer and is receiving
     * messages from the producer.
     */
    Dat_ConsumerStatus_CONNECTED            = 0x1,

    /**
     * @brief   Consumer is disconnected and is not attached to any producer
     */
    Dat_ConsumerStatus_DISCONNECTED         = 0x2,

    /**
     * @brief   Consumer is being connected to a producer.
     */
    Dat_ConsumerStatus_PENDING_CONNECT      = 0x3,

    /**
     * @brief   Consumer is being disconnected from a producer.
     */
    Dat_ConsumerStatus_PENDING_DISCONNECT   = 0x4
}Dat_ConsumerStatus;

/**
 *  @b Description
 *  @n
 *      DAT Server Logging function which is passed to the server during
 *      initialization. The function is used to pass messages from the server to
 *      the application during execution.
 *
 *  @param[in]  logLevel
 *      Log level of the message
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  arg
 *      Variable length of arguments.
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Dat_ServerLogFn)(Dat_LogLevel logLevel, const char* fmt, va_list arg)  __attribute__ ((format (printf, 2, 0)));

/**
 * @brief
 *  DAT LogSync configuration
 *
 * @details
 *  The LogSync configuration describes the various configuration properties
 *  which are required to generate LogSync events in UIA logging.
 */
typedef struct Dat_LogSyncCfg
{
    /**
     * @brief   Time period in milliseconds between each LogSync message
     *          used for multicore event correlation in the System Analyzer
     *          If set to 0, sending LogSync messages is disabled.
     */
    uint32_t           syncPeriod;

    /**
     * @brief   Handle to the LoggerStreamer2 logger used as the LogSync
     *          syncLogger.
     */
    void*              syncLogger;

    /**
     * @brief   Last recorded timestamp for LogSync
     */
    uint64_t            logSyncTimestamp;

}Dat_LogSyncCfg;

/**
 * @brief
 *  DAT Global TimeStamp Freq structure
 *
 * @details
 *  The configuration describes DAT timestamp frequency.
 */
typedef struct Dat_timeStampClockFreq
{
    /**
     * @brief   Time Stamp clock frequncy high 32bits value.
     */
    uint32_t           hi;

    /**
     * @brief   Time Stamp clock frequncy low 32bits value.
     */
    uint32_t           lo;

}Dat_timeStampClockFreq;

/**
 * @brief
 *  DAT Server configuration
 *
 * @details
 *  The DAT Server configuration describes the various configuration properties
 *  which are required to initialize and start the DAT Server.
 */
typedef struct Dat_ServerCfg
{
    /**
     * @brief  Name of the server which should be unique in the domain.
     */
    char                            serverName[DAT_MAX_CHAR];

    /**
     * @brief   Named resource instance identifier. Server will use this named
     * resource to store named resource information. It is responsibility of the
     * application to ensure that this identifier is the same across both the
     * DAT servers and clients use the samed named resource instance identifier.
     * Failure to do so will result in the DAT functionality to not work correctly.
     */
    uint32_t                        nrInstanceId;

    /**
     * @brief   PKTLIB Instance handle. All packets sent and received on the DAT client
     * configuration are operating within this specific PKTLIB instance.
     */
    Pktlib_InstHandle               pktlibInstHandle;

    /**
     * @brief   This is the MSGCOM instance handle which is used to create and manage all
     * the MSGCOM channels used between the DAT Server & DAT client(s). Please ensure
     * that the DAT servers and clients are on the same MSGCOM instance handle.
     */
    Msgcom_InstHandle               msgcomInstHandle;

    /**
     * @brief   Server execution realm
     */
    Dat_ExecutionRealm              realm;

    /**
     * @brief   Logging function. This is an optional parameter but if specified allows
     * application to get access to the log messages which are generated by the DAT
     * server
     */
    Dat_ServerLogFn                 logFxn;

    /**
     * @brief   Server heap handle which is used to send and receive messages to the
     * DAT clients. It is recommended that the heap handle be local to the server
     */
    Pktlib_HeapHandle               serverHeapHandle;

    /**
     * @brief  OSAL Malloc
     */
    Osal_DatMalloc                  malloc;

    /**
     * @brief  OSAL Free
     */
    Osal_DatFree                    free;

    /**
     * @brief  OSAL cache invalidation
     */
    Osal_DatBeginMemoryAccess       beginMemAccess;

    /**
     * @brief  OSAL cache writeback
     */
    Osal_DatEndMemoryAccess         endMemAccess;

    /**
     * @brief  OSAL create semaphore
     */
    Osal_DatCreateSem               createSem;

    /**
     * @brief  OSAL delete semaphore
     */
    Osal_DatDeleteSem               deleteSem;

    /**
     * @brief  OSAL post semaphore
     */
    Osal_DatPostSem                 postSem;

    /**
     * @brief  OSAL pend semaphore
     */
    Osal_DatPendSem                 pendSem;

    /**
     * @brief  OSAL Enter critical section
     */
    Osal_DatEnterCS                 enterCS;

    /**
     * @brief  OSAL Exit critical section
     */
    Osal_DatExitCS                  exitCS;
}Dat_ServerCfg;

/**
 * @brief
 *  DAT Instance configuration
 *
 * @details
 *  The structure describes the configuration which is required to
 *  create a DAT instance.
 */
typedef struct Dat_ClientCfg
{
    /**
     * @brief   Unique name associated with the DAT client
     */
    char                        clientName[DAT_MAX_CHAR];

    /**
     * @brief   Name of the DAT server
     */
    char                        serverName[DAT_MAX_CHAR];

    /**
     * @brief   Identifier for the DAT client. Each DAT client is associated
     * with the unique identifier which should be the same as the endpoint address
     * configured in the System analyzer. For DSP clients this should be configured
     * as the DNUM however on ARM clients this should be a unique identifier provided
     * across the system.
     */
    int32_t                     id;

    /**
     * @brief   PKTLIB instance handle used by the DAT client
     */
    Pktlib_InstHandle           pktlibInstHandle;

    /**
     * @brief   MSGCOM instance handle used by the DAT client
     */
    Msgcom_InstHandle           msgcomInstHandle;

    /**
     * @brief   DAT clients communicate with the DAT Server via MSGCOM channels.
     * Each MSGCOM channel used for this communication is a DIRECT Interrupt channel
     * whose configuration is specified here
     */
    Msgcom_DirectInterruptCfg   directInterruptCfg;

    /**
     * @brief   Database handle associated with the DAT client. Information about any producers
     * or consumers created using the DAT client handle will only be stored in this database.
     */
    Name_DBHandle               databaseHandle;

    /**
     * @brief   Client heap handle which is used to send and receive messages to the
     * DAT server. It is recommended that the heap handle be local to the client.
     */
    Pktlib_HeapHandle           clientHeapHandle;

    /**
     * @brief  Handle to the name client. This is required to be configured especially
     * if the DAT servers and clients are present in different realms. This can be set
     * to NULL if the client and server are in the same realm.
     */
    Name_ClientHandle           nameClientHandle;

    /**
     * @brief  LogSync configuration.
     */
    Dat_LogSyncCfg              logSync;

    /**
     * @brief  Global timestamp clock Freq for CUIA
     */
    Dat_timeStampClockFreq      globalTsFreq;

    /**
     * @brief   Execution Realm in which the DAT client is executing.
     */
    Dat_ExecutionRealm          realm;

    /**
     * @brief  OSAL Malloc
     */
    Osal_DatMalloc               malloc;

    /**
     * @brief  OSAL Free
     */
    Osal_DatFree                 free;

    /**
     * @brief  OSAL Malloc
     */
    Osal_DatMallocLocalMemory    mallocLocal;

    /**
     * @brief  OSAL Free
     */
    Osal_DatFreeLocalMemory      freeLocal;
    /**
     * @brief  OSAL cache invalidation
     */
    Osal_DatBeginMemoryAccess    beginMemAccess;

    /**
     * @brief  OSAL cache writeback
     */
    Osal_DatEndMemoryAccess     endMemAccess;

    /**
     * @brief  OSAL create semaphore
     */
    Osal_DatCreateSem           createSem;

    /**
     * @brief  OSAL delete semaphore
     */
    Osal_DatDeleteSem           deleteSem;

    /**
     * @brief  OSAL post semaphore
     */
    Osal_DatPostSem             postSem;

    /**
     * @brief  OSAL pend semaphore
     */
    Osal_DatPendSem             pendSem;

    /**
     * @brief  OSAL Enter critical section: The critical section is used to protect the
     * consumers & producers since these are created/deleted in the context of an application
     * thread but are also accessed in a background thread. Please do not use interrupt
     * disable/enable to implement this critical section since the critical section is held
     * in the background thread while all the consumers & producers are handled.
     */
    Osal_DatEnterCS             enterCS;

    /**
     * @brief  OSAL Exit critical section
     */
    Osal_DatExitCS              exitCS;
}Dat_ClientCfg;

/**
 * @brief
 *  DAT configuration type
 *
 * @details
 *  The structure describes the possible configuration type for client configuration after
 * Client is started.
 */
typedef enum Dat_clientConfigType
{
    DAT_CONF_LOGSYNC,
    DAT_CONF_GLOBAL_TSFREQ,
    DAT_CONF_MAX

}Dat_clientConfigType;

/**
 * @brief
 *  DAT configuration structure used for client configuration after client is started
 *
 * @details
 *  The structure describes client configurations after Client is started.
 */
typedef struct Dat_clientRuntimeCfg
{
    /**
     * @brief  Configuration type.
     */

    Dat_clientConfigType        cfgType;

    /**
     * @brief  LogSync configuration.
     */
    Dat_LogSyncCfg              logSync;

    /**
     * @brief  Global TimeStamp clock frequency setting.
     */
    Dat_timeStampClockFreq      globalTsFreq;

}Dat_clientRuntimeCfg;

/**
 * @brief
 *  DAT Producer Statistics
 *
 * @details
 *  The structure describes the statistics which are maintained for each
 *  producer
 */
typedef struct Dat_ProducerStats
{
    /**
     * @brief   This keeps track of the number of times the data buffer was
     * exchanged
     */
    uint32_t        bufferExchange;

    /**
     * @brief   This keeps track of the number of times the producer buffer
     * was overrun because the buffers had not been consumed.
     */
    uint32_t        bufferOverrun;

    /**
     * @brief   This keeps track of the number of times the producer buffer
     * was dropped since there were no active consumers connected.
     */
    uint32_t        noConsumers;

    /**
     * @brief   This keeps track of the number of messages which could not be
     * streamed out because of NETFP failures.
     */
    uint32_t        debugStreamingError;

    /**
     * @brief   This keeps track of the number of messages which were streamed
     * by the producer over NETFP
     */
    uint32_t        debugStreaming;

    /**
     * @brief   Number of messages dropped because packets could not be allocated
     * for cloning
     */
    uint32_t        allocFailures;

    /**
     * @brief   This keeps track of the maximum depth of the pending queue which
     * are buffers which have been exchanged but have still not been consumed.
     */
    uint32_t        maxQueueDepth;
}Dat_ProducerStats;

/**
 * @brief
 *  DAT Producer configuration
 *
 * @details
 *  The structure describes the DAT producer configuration which needs to
 *  be provided to create a producer. DAT producers will produce data which
 *  can be consumed by consumers.
 */
typedef struct Dat_ProducerCfg
{
    /**
     * @brief   Unique name associated with the producer
     */
    char                        name[DAT_MAX_CHAR];

    /**
     * @brief   Type of the producer -- UIA or General-purpose.
     */
    Dat_ProducerType            producerType;

    /**
     * @brief   Size of the producer buffer.
     */
    uint32_t                    bufferSize;

    /**
     * @brief   CRC16 value needed by loggerStreamer2, it is generated from application
     *          file name without extension. This value is only apllicable to ARM producer.
     */
    uint32_t                     crcApp16;

    /**
     * @brief   Flag to indicate the producer associate with the main logger on ARM.
     *          There should be one mainLogger from ARM.
     */
    uint32_t                     isMainLogger;

    /**
     * @brief   Clear (fill with zeros) every new buffer given to the producer.
     *          Applicable only for general-purpose producers.
     *          0: buffer is not cleared
     *          1: buffer is cleared
     */
    uint32_t                     clearBuffer;

    /**
     * @brief   Heap handle which is used by the producer to generate messages
     * by the DAT module. The heap handle should have zero buffer packets if
     * multiple consumers can be attached to a single producer.
     */
    Pktlib_HeapHandle           heapHandle;

    /**
     * @brief   Handle to the logger streamer object which is used to generate
     * UIA messages. There can only be one producer associated with a logger
     * streamer object
     */
    void*                       loggerStreamerHandle;

    /**
     * @brief   Producers can be configured to stream data out over the networking
     * interfaces. This is an optional feature and can be enabled by passing a
     * valid NETFP socket handle which is used for this streaming
     */
    Netfp_SockHandle            debugSocketHandle;

    /**
     * @brief   MEMLOG channel handle if memory logging is needed for the producer.
     */
    Memlog_ChHandle             memlogChanHandle;
}Dat_ProducerCfg;

/**
 * @brief
 *  DAT Consumer configuration
 *
 * @details
 *  The structure describes the DAT consumer configuration which needs to be
 *  provided to create a consumer and attach it with a producer.
 */
typedef struct Dat_ConsumerCfg
{
    /**
     * @brief   Producer name to which the consumer is connected.
     */
    char                    producerName[DAT_MAX_CHAR];

    /**
     * @brief   Heap handle which is used by the consumer to receive data
     * buffers produced from the producer.
     */
    Pktlib_HeapHandle       heapHandle;

}Dat_ConsumerCfg;

/**
 * @brief
 *  DAT Verbosity Configuration Options
 *
 * @details
 *  Specify whether verbosity modification applies to:
 *  - specified component
 *  - common component mask
 *  - class only setting (e.g., BIOS logging levels)
 */
typedef enum Dat_VerbosityOption
{
    /**
     * @brief   Verbosity modification applies to specified component
     */
    Dat_Verbosity_Component     =   0,

    /**
     * @brief   Verbosity modification to the common component mask which is
     * ORed with each individual component's mask
     */
    Dat_Verbosity_CommonComponent,

    /**
     * @brief   Verbosity modification at the class level
     *          Does not apply to individual components (e.g., BIOS logging
     *          levels)
     */
    Dat_Verbosity_Class
}Dat_VerbosityOption;

/**
 * @brief
 *  Structure which describes Verbosity Configuration
 *
 * @details
 *  Settings specified during Verbosity modification.
 */
typedef struct Dat_VerbosityCfg
{
    /**
     * @brief   Verbosity modification for:
     *          - specified component
     *          - all components
     *          - class only
     */
    Dat_VerbosityOption     verbosityOpt;

    /**
     * @brief   Name of the global trace object.
     */
    char                    traceObjectName[DAT_MAX_CHAR];

    /**
     * @brief   Trace component Id.
     */
    uint32_t                traceComponentId;

    /**
     * @brief   Verbosity level to be modified.
     */
    Dat_TraceLevel          verbosityLevel;

    /**
     * @brief   Set or reset verbosity level.
     */
    uint8_t                 isEnabled;

    /**
     * @brief   Specifies whether verbosity has to be modified for
     *          focused or non-focused components. Not applicable
     *          for class-level settings.
     */
    uint8_t                 isComponentFocused;
}Dat_VerbosityCfg;

/**
 * @brief
 *  Structure which describes verbosity settings of a trace component
 *
 * @details
 *  This structure is used to set the trace component default properties.
 */
typedef struct Dat_TraceComponentCfg
{
    /**
     * @brief   Name of the trace component.
     */
    char                    componentName[DAT_TRACE_COMPONENT_MAX_CHAR];

    /**
     * @brief   Focused verbosity level for this component.
     */
    Dat_TraceLevel          focusedLevel;

    /**
     * @brief   Non-focused verbosity level for this component.
     */
    Dat_TraceLevel          nonFocusedLevel;
}Dat_TraceComponentCfg;

/**
 * @brief
 *  Structure which describes default settings for the trace object verbosity
 *
 * @details
 *  Settings specified during verbosity initialization.
 */
typedef struct Dat_TraceObjectCfg
{
    /**
     * @brief   Name of the global trace object.
     */
    char                    traceObjectName[DAT_MAX_CHAR];

    /**
     * @brief   Total number of trace components.
     */
    uint32_t                numTraceComponents;

    /**
     * @brief   Default verbosity level for class-level logging.
     * Example: SYS/BIOS logging
     */
    Dat_TraceLevel          classLevel;

    /**
     * @brief   Default verbosity level for the common component mask. This is
     * ORed with individual component mask during filtering.
     */
    Dat_TraceLevel          commonCompLevel;

    /**
     * @brief   Array of all components containing component name and default
     * verbosity levels.
     */
    Dat_TraceComponentCfg*  ptrComponentArray;

}Dat_TraceObjectCfg;

/**
 * @brief
 *  Structure which describes verbosity settings of a trace component
 *
 * @details
 *  This structure is used to return trace component properties when queried.
 */
typedef struct Dat_TraceCompInfo
{
    /**
     * @brief   Name of the trace component.
     */
    char                    componentName[DAT_TRACE_COMPONENT_MAX_CHAR];

    /**
     * @brief   Verbosity level for this component.
     */
    Dat_TraceLevel          logLevel;

}Dat_TraceCompInfo;

/**
 * @brief
 *  DAT Trace Object configuration
 *
 * @details
 *  The structure describes the DAT trace object configuration which needs to
 *  be provided to create a global trace object.
 */
typedef struct Dat_TraceObjectBody
{
    /**
     * @brief   Unique name associated with the global trace object
     */
    char                    name[DAT_MAX_CHAR];

    /**
     * @brief   Total number of trace components.
     */
    uint32_t                numTraceComponents;

    /**
     * @brief   Size of the global trace object.
     */
    uint32_t                traceObjectSize;

    /**
     * @brief   Array of all components containing component name and default
     * verbosity levels.
     */
    Dat_TraceCompInfo*      ptrComponentArray;

}Dat_TraceObjectBody;

/**
 * @brief
 *  Enumeration for internal data type to access trace object layout.
 */
typedef enum Dat_TraceId
{
    /**
     * @brief   First entity in the trace object is the mask for the class.
     */
    DAT_CLASS_ID = 0,

    /**
     * @brief   Next entity is reserved for future class mask definitions.
     */
    DAT_CLASS_ID_RSVD,

    /**
     * @brief   List of trace components start from here.
     */
    DAT_COMPONENT_START_ID

}Dat_TraceId;


/*****************************************************************************
 ***************************** Exported API **********************************
 *****************************************************************************/

/* DAT Client Functions: */
extern Dat_ClientHandle Dat_initClient (Dat_ClientCfg* ptrClientCfg, int32_t* errCode);
extern int32_t Dat_startClient (Dat_ClientHandle clientHandle, int32_t* errCode);
extern int32_t Dat_stopClient (Dat_ClientHandle clientHandle, int32_t* errCode);
extern Dat_ServerHandle Dat_startServer (Name_DBHandle databaseHandle, Name_ClientHandle clientHandle, const char* serverName, int32_t* errCode);
extern int32_t Dat_isServerStopped (Name_DBHandle databaseHandle, Name_ClientHandle clientHandle, const char* serverName, int32_t* errCode);
extern int32_t Dat_deleteClient(Dat_ClientHandle clientHandle, int32_t* errCode);
extern void Dat_executeClient(Dat_ClientHandle clientHandle);
extern void Dat_executeBackgroundActions (Dat_ClientHandle clientHandle);
extern int32_t Dat_configureClient (Dat_ClientHandle clientHandle, Dat_clientRuntimeCfg* ptrDatClientCfg, int32_t* errCode);

/* DAT Server Functions: */
extern Dat_ServerHandle Dat_initServer(Dat_ServerCfg* ptrServerCfg, int32_t* errCode);
extern int32_t Dat_registerClient (Dat_ServerHandle serverHandle, const char* clientName, int32_t* errCode);
extern int32_t Dat_isClientActive(Dat_ServerHandle serverHandle, const char* clientName, int32_t* errCode);
extern int32_t Dat_deregisterClient(Dat_ServerHandle serverHandle, const char* clientName, int32_t* errCode);
extern int32_t Dat_deleteServer (Dat_ServerHandle serverHandle, int32_t* errCode);
extern int32_t Dat_executeServer(Dat_ServerHandle serverHandle);
extern void Dat_displayServer(Dat_ServerHandle serverHandle);

/* Exported DAT Services: */
extern Dat_ConsHandle Dat_createConsumer(Dat_ClientHandle clientHandle, Dat_ConsumerCfg* ptrConsumerCfg, int32_t* errCode);
extern Dat_ProdHandle Dat_createProducer (Dat_ClientHandle clientHandle,Dat_ProducerCfg* ptrProducerCfg, void** ptrLoggerHandle, int32_t* errCode);
extern int32_t Dat_connectConsumer(Dat_ConsHandle consumerHandle, int32_t* errCode);
extern int32_t Dat_disconnectConsumer (Dat_ConsHandle consumerHandle, int32_t* errCode);
extern int32_t Dat_getConsumerStatus (Dat_ConsHandle consumerHandle, Dat_ConsumerStatus* consumerStatus, int32_t* errCode);
extern int32_t Dat_deleteProducer (Dat_ProdHandle producerHandle, int32_t* errCode);
extern int32_t Dat_deleteConsumer (Dat_ConsHandle consumerHandle, int32_t* errCode);
extern int32_t Dat_getProducerStats (Dat_ProdHandle producerHandle, Dat_ProducerStats* ptrProducerStats, int32_t* errCode);
extern void Dat_flushAllBuffers (Dat_ClientHandle clientHandle);
extern Ti_Pkt* Dat_processConsumer(Dat_ConsHandle consumerHandle);

extern uint8_t* Dat_getProducerBuffer (Dat_ProdHandle producerHandle, const char* ProducerName, int32_t* errCode);
extern uint8_t* Dat_bufferExchange(Dat_ClientHandle clientHandle, Dat_ProdHandle producerHandle, uint8_t* oldBuffer);
extern uint32_t Dat_getGPProducerDataOffset(Dat_ConsHandle consumerHandle, int32_t* errCode);
extern uint32_t Dat_getGPProducerDataOffsetByRealm(Dat_ExecutionRealm realm, int32_t* errCode);

/* DAT Verbosity APIs */
extern int32_t Dat_createTraceObject(Dat_ClientHandle clientHandle, Dat_TraceObjectCfg* ptrTraceCfg, int32_t* errCode);
extern int32_t Dat_deleteTraceObject(Dat_ClientHandle clientHandle, const char* traceObjName, int32_t* errCode);
extern Dat_TraceObjHandle Dat_createTraceObjectInstance( Dat_ClientHandle clientHandle, const char* name, int32_t* errCode);
extern Dat_TraceObjectBody* Dat_getTraceObjectBody(Dat_TraceObjHandle traceObjectHandle);
extern int32_t Dat_deleteTraceObjectInstance(Dat_TraceObjHandle localTracehandle,int32_t* errCode);
extern int32_t Dat_modifyVerbosity (Dat_TraceObjHandle localTracehandle, Dat_VerbosityCfg* ptrVerbosityCfg, int32_t* errCode);
extern int32_t Dat_getNumComponents(Dat_TraceObjHandle localTracehandle, const char* traceObjectName, uint32_t* numComponents, int32_t* errCode);
extern int32_t Dat_getClassVerbosity(Dat_TraceObjHandle localTracehandle, const char* traceObjectName, Dat_TraceLevel* ptrClassLevel,
                                     Dat_TraceLevel* ptrCommonCompLevel, int32_t* errCode);
extern int32_t Dat_getComponentVerbosity (Dat_TraceObjHandle localTracehandle, const char* traceObjectName, uint32_t componentId,
                                          Dat_TraceComponentCfg* ptrComponentInfo, int32_t* errCode);
extern int32_t Dat_getTraceObjectNames (Dat_ClientHandle clientHandle, uint32_t nameArraySize, const char* nameArray, int32_t* errCode);

/* DAT-Logger Stream2 Interface Functions: */
extern Ptr Dat_exchangeFunction(LoggerStreamer2_Handle handle, uint8_t* fillBuffer);

/**
@}
*/

/*****************************************************************************
 ***************************** Exported API **********************************
 *****************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function checks the local instance of the trace object and
 *      indicates whether a given event has to be logged for the specified
 *      component.
 *
 *  @param[in]  ptrTraceObjBody
 *      Handle of the local trace object body that contains the verbosity settings
 *      for this component
 *  @param[in]  traceComponentId
 *      Enum value corresponding to the component. This is used as an index
 *      into the trace object array of verbosity levels.
 *  @param[in]  eventLevel
 *      Verbosity level of the log.
 *  @param[in]  isFocused
 *      Is this a focused instance of the component (0/1)?
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      0   -   Do not log
 *  @retval
 *      1   -   Must be logged
 */
static inline uint8_t Dat_filter
(
    Dat_TraceObjectBody*    ptrTraceObjBody,
    uint32_t                traceComponentId,
    Dat_TraceLevel          eventLevel,
    uint8_t                 isFocused
)
{
     Dat_TraceLevel          classMask;

    /* If trace object handle is NULL, it is the platform trace object. */
    if (ptrTraceObjBody == NULL)
        return 0;

    /* Derive the class mask. */
    classMask = ptrTraceObjBody->ptrComponentArray[DAT_CLASS_ID].logLevel;

    /* Check if DISABLEALL mask bit */
    if(classMask & ((uint32_t)DAT_CLASS_LEVEL_DISABLEALL << DAT_MASK_SIZE))
        return 0;

    /* If focused, use the higher 16 bits of the mask. */
    if (isFocused)
    {
        classMask  <<= DAT_MASK_SIZE;
        eventLevel <<= DAT_MASK_SIZE;
    }

    /* Event Filtering: filter based on the component and class verbosity settings. */
    if ((ptrTraceObjBody->ptrComponentArray[traceComponentId + DAT_COMPONENT_START_ID].logLevel | classMask) & eventLevel)
        return 1;

    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* __DAT_H__ */

