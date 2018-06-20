/**
 *   @file  dat_internal.h
 *
 *   @brief
 *      Internal header file for the Debug & Trace library.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */

#ifndef __DAT_INTERNAL_H__
#define __DAT_INTERNAL_H__

/* SYSLIB Include Files. */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/dat/include/listlib.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf

#include <sys/time.h>
/* UIA Include Files */
#include <ti/uiaplus/loggers/multistream/LoggerStreamer2.h>

#else
#include <xdc/runtime/System.h>

/* UIA Include Files */
#include <ti/uia/sysbios/LoggerStreamer2.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DAT_INTERNAL_DATASTRUCT
 @{ */

/**
 * @brief   Handle to the DAT consumer which exists on the server.
 */
typedef void*   Dat_ServerConsHandle;

/**
 * @brief   Handle to the DAT producer which exists on the server.
 */
typedef void*   Dat_ServerProdHandle;

/**
 * @brief   Handle to the DAT trace object which exists on the server.
 */
typedef void*   Dat_ServerTraceObjHandle;


/**
 * @brief   Handle to the DAT Local trace object which exists on the server.
 */
typedef void*   Dat_ServerLocalTraceObjHandle;

#ifdef __ARMv7
/**
 * @brief   The MACRO is used to initialize variables used to record the
 *          timestamp
 */
#define DAT_TIMESTAMP_INIT(X)           uint64_t    X

/**
 * @brief   The MACRO takes a snapshot of the Linux timestamp and records it.
 *          The timestamp is taken in multiple of micro seconds.
 */
#define DAT_TIMESTAMP(X)   { \
    struct timeval now; \
    gettimeofday(&now, NULL); \
    X = 1000000 * now.tv_sec + now.tv_usec; \
}

/**
 * @brief   The MACRO calculates the time elapsed in milli seconds
 */
#define DAT_TIME_ELAPSED(Z, X, Y)       Z = (X - Y)/1000

#else
/**
 * @brief   The MACRO is used to initialize variables used to record the
 *          timestamp
 */
#define DAT_TIMESTAMP_INIT(X)           uint64_t    X

/**
 * @brief   The MACRO takes a snapshot of the DSP timestamp and records it
 */
#define DAT_TIMESTAMP(X)                X = TSCL;

/**
 * @brief   The MACRO calculates the time elapsed in milli seconds (1 GHz clock)
 */
#define DAT_TIME_ELAPSED(Z, X, Y)       Z = (X - Y)/(1000*1000)
#endif

/**
 * @brief   Length of the UIA packet header.
 */
#ifdef __ARMv7
#define DAT_CUIA_PKT_HEADER_SIZE        sizeof(UIAPacket_Hdr5)
#define DAT_UIA_PKT_HEADER_SIZE         DAT_CUIA_PKT_HEADER_SIZE
#else
#define DAT_UIA_PKT_HEADER_SIZE         sizeof(UIAPacket_Hdr)
#define DAT_CUIA_PKT_HEADER_SIZE        20
#endif

/**
 * @brief   Length of the UIA event header.
 */
#define DAT_UIA_EVT_HEADER_SIZE         (10 * sizeof(uint32_t))

/**
 * @brief   Length of the UIA header -- packet header + event header for
 *          snapshot and timestamp.
 */
#define DAT_UIA_HEADER_OFFSET           (DAT_UIA_PKT_HEADER_SIZE + DAT_UIA_EVT_HEADER_SIZE)

/**
 * @brief
 *  DAT Client Status
 *
 * @details
 *  DAT Clients can be in one of the following states
 */
typedef enum Dat_ClientStatus
{
    /**
     * @brief   The DAT client is free and is not being used
     */
    Dat_ClientStatus_FREE        = 0x0,

    /**
     * @brief   The DAT client has been initialized but has still not been
     * registered with the server.
     */
    Dat_ClientStatus_INITIALIZED = 0x1,

    /**
     * @brief   The DAT client has been initialized and registered with the
     * server. DAT services are not usable till the registeration is complete.
     */
    Dat_ClientStatus_ACTIVE       = 0x2,

    /**
     * @brief   The DAT client is INACTIVE but has not yet been deregistered
     * from the DAT server.
     */
    Dat_ClientStatus_INACTIVE     = 0x3
}Dat_ClientStatus;

/**
 * @brief
 *  DAT Consumer
 *
 * @details
 *  The structure describes the DAT consumer as it exists on the client
 */
typedef struct Dat_Consumer
{
    /**
     * @brief   Links to other consumers.
     */
    Dat_ListNode                links;

    /**
     * @brief   Status which indicates if the consumer is connected.
     */
    uint32_t                    isConnected;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   Consumer configuration
     */
    Dat_ConsumerCfg             cfg;

    /**
     * @brief   Handle to the consumer created on the server
     */
    Dat_ServerConsHandle        serverConsumerHandle;

    /**
     * @brief   MSGCOM channel handles which allow the consumer to
     * receive data from the producer.
     */
    MsgCom_ChHandle             channelHandle;
}Dat_Consumer;

/**
 * @brief
 *  DAT Consumer
 *
 * @details
 *  The structure describes the DAT consumer as it exists on the server
 */
typedef struct Dat_ServerConsumer
{
    /**
     * @brief   Links to other consumers
     */
    Dat_ListNode                links;

    /**
     * @brief   Server Producer handle to which the consumer is connected
     */
    struct Dat_ServerProducer*  ptrServerProducer;

    /**
     * @brief   Job identifier associated while the consumer is being connected
     * This is valid only when the consumer is in the pending list
     */
    int32_t                     jobId;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   JOSH Node handle to the DAT client which created the producer
     */
    Josh_NodeHandle             joshNodeHandle;
}Dat_ServerConsumer;

/**
 * @brief
 *  Dat_LoggerFuncTbl
 *
 * @details
 *  The structure defindes the function table for different type of producer
 */
typedef struct Dat_LoggerFuncTbl
{
    void (*initBuffer)(void* handle, void* buffer, uint16_t src, uint16_t instance);
    int32_t (*prime) (void* handle);
    Ptr (*bufExchange)(LoggerStreamer2_Handle handle, uint8_t* fillBuffer);
}Dat_LoggerFuncTbl;

/**
 * @brief
 *  Enumeration for internal data type to access trace object layout.
 */
typedef enum Dat_loggerInstStatus
{
    /**
     * @brief   Logger Id is free to use.
     */
    DAT_LOGGER_ID_FREE = 0,

    /**
     * @brief   Logger Id is busy.
     */
    DAT_LOGGER_ID_BUSY,

    /**
     * @brief   Logger id is invalid.
     */
    DAT_LOGGER_ID_INVALID

}Dat_loggerInstStatus;

/**
 * @brief
 *  Dynamic logger instance control block.
 */
typedef struct Dat_loggerInstId
{
    /**
     * @brief   Links to put logger instance into list
     */
    Dat_ListNode               links;

    /**
     * @brief   Status of the logger instance
     */
    Dat_loggerInstStatus       status;

    /**
     * @brief   Logger Instance id value
     */
    uint16_t                   loggerInstId;

}Dat_loggerInstId;

/**
 * @brief
 *  UIO mapping for emu counters.
 */
typedef struct Dat_uioMap
{
    /**
     * @brief   File descriptor for the uio device
     */
    int           fd;

    /**
     * @brief   Memory map base address
     */
    int32_t       memMapBase;

    /*
     * @brief   Clock frequency high 32 bits
     */
    uint32_t      freqHi;

    /*
     * @brief   Clock frequency high 32 bits
     */
    uint32_t      freqLo;
}Dat_uioMap;


/**
 * @brief
 *  Dat Memory logging log buffer.
 *
 * @details
 *  The structure describes the definition of Memory logging log buffer
 */
typedef struct Dat_logBuffer
{
    /**
     * @brief   Links to other buffers.
     */
    Dat_ListNode                links;

    /**
     * @brief   Log buffer address.
     */
    uint32_t                    bufAddr;

}Dat_logBuffer;

/**
 * @brief
 *  Dat Producer info.
 *
 * @details
 *  The structure describes a producer information
 */
typedef struct Dat_producerInfo
{
    /**
     * @brief   Unique name associated with the producer
     */
    char                 name[DAT_MAX_CHAR];

    /**
     * @brief   Execution Realm in which the DAT producer is executing.
     */
    Dat_ExecutionRealm   realm;
}Dat_producerInfo;

/**
 * @brief
 *  Dat Producer controller.
 *
 * @details
 *  The structure describes producer controller running on a DAT client
 * used to control a remote DAT producer
 */
typedef struct Dat_prodController
{
    /**
     * @brief   Links to other producers.
     */
    Dat_ListNode        links;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle    clientHandle;

    /**
     * @brief   Pointer to the controlled producer information.
     */
    Dat_producerInfo    producerInfo;

  }Dat_prodController;


/**
 * @brief
 *  DAT Producer
 *
 * @details
 *  The structure describes the DAT producer as it exists on the DAT client
 */
typedef struct Dat_Producer
{
    /**
     * @brief   Links to other producers.
     */
    Dat_ListNode                links;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   Producer configuration
     */
    Dat_ProducerCfg             cfg;

    /**
     * @brief   Producer loggerStreamer function table by producer type
     */
    Dat_LoggerFuncTbl           loggerFunc;

    /**
     * @brief   owner the loggerStreamer
     */
    uint32_t                    loggerStreamerOwner;

    /**
     * @brief   Pending queue handle which is used to hold all the descriptors which
     * have been exchanged but have still not been pushed to the consumers
     */
    Qmss_QueueHnd               pendingQueueHandle;

    /**
     * @brief   The producer always has 1 packet which is in production.
     */
    Ti_Pkt*                     ptrProductionPkt;

    /**
     * @brief   Producer statistics
     */
    Dat_ProducerStats           stats;

    /**
     * @brief   Number of consumers attached to the producer.
     */
    int32_t                     numConsumers;

    /**
     * @brief   Sequence count -- applicable only for general-purpose producers.
     */
    uint32_t                    sequenceCount;

    /**
     * @brief   MSGCOM channel names associated with each consumer channel handle
     */
    char                        consumerChannelNames[DAT_MAX_CONSUMER][MSG_COM_MAX_CHANNEL_LEN];

    /**
     * @brief   MSGCOM channel handles which allow the producer to send data
     * to each consumer.
     */
    MsgCom_ChHandle             consumerChHandles[DAT_MAX_CONSUMER];

    /**
     * @brief   Logger instance Id for the producer
     */
    Dat_loggerInstId*           ptrLoggerId;
}Dat_Producer;

/**
 * @brief
 *  DAT Producer
 *
 * @details
 *  The structure describes the DAT consumer as it exists on the server
 */
typedef struct Dat_ServerProducer
{
    /**
     * @brief   Links to other producers
     */
    Dat_ListNode                links;

    /**
     * @brief   Name of the producer
     */
    char                        name[DAT_MAX_CHAR];

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   Handle to the producer on the DAT client.
     */
    Dat_ProdHandle              clientProducerHandle;

    /**
     * @brief   JOSH Node handle to the DAT client which created the producer
     */
    Josh_NodeHandle             joshNodeHandle;

    /**
     * @brief   Job identifier associated while the producer is being freeze and unfreeze.
     * This is valid only when the producer is in the producer operation pending list
     */
    int32_t                     jobId;

    /**
     * @brief   Execution Realm in which the DAT producer is executing.
     */
    Dat_ExecutionRealm          realm;

    /**
     * @brief   Links to all the consumers which are attached to the producer
     */
    Dat_ServerConsumer*         ptrConsumerList;
}Dat_ServerProducer;


/**
 * @brief
 *  DAT Local Trace Object on DAT client
 *
 * @details
 *  The structure describes the DAT trace object as it exists on the server
 */
typedef struct Dat_LocalTraceObject
{
    /**
     * @brief   Links to other consumers
     */
    Dat_ListNode                links;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   Handle to the server Local trace object
     */
    Dat_ServerLocalTraceObjHandle serverLocalTraceObj;

    /**
     * @brief   Trace Object body
     */
    Dat_TraceObjectBody           traceObject;

}Dat_LocalTraceObject;

/**
 * @brief
 *  DAT Global Trace Object
 *
 * @details
 *  The structure describes the DAT trace object as it exists on the server
 */
typedef struct Dat_ServerLocalTraceObject
{
    /**
     * @brief   Links to other trace objects
     */
    Dat_ListNode                links;

    /**
     * @brief   Links used to put verbosity operation list
     */
    Dat_ListNode                opLinks;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   Trace Object configuration
     */
    struct Dat_ServerGlobalTraceObject *ptrGlobalTraceObject;

    /**
     * @brief   Job identifier associated while the consumer is being connected
     * This is valid only when the consumer is in the pending list
     */
    int32_t                     jobId;

    /**
     * @brief   JOSH Node handle to the DAT client which created the producer
     */
    Josh_NodeHandle             joshNodeHandle;

}Dat_ServerLocalTraceObject;

/**
 * @brief
 *  DAT Global Trace Object
 *
 * @details
 *  The structure describes the DAT trace object as it exists on the server
 */
typedef struct Dat_ServerGlobalTraceObject
{
    /**
     * @brief   Links to other trace objects
     */
    Dat_ListNode                links;

    /**
     * @brief   Trace Object body
     */
    Dat_TraceObjectBody         traceObject;

    /**
     * @brief   Trace Object initializatio status
     */
    uint32_t                    initialized;

    /**
     * @brief   Handle to the DAT client
     */
    Dat_ClientHandle            clientHandle;

    /**
     * @brief   JOSH Node handle to the DAT client which created the producer
     */
    Josh_NodeHandle             joshNodeHandle;

    /**
     * @brief   Links to all the local trace object which are attached to this global trace object
     */
    Dat_ServerLocalTraceObject* ptrLocalTraceObjectList;

}Dat_ServerGlobalTraceObject;

/**
 * @brief
 *  DAT trace component block info used during trace object initialization to pass trace levels
 *  between DAT server and DAT client.
 *
 * @details
 *  The structure describes the DAT componnet block info to transfer components between client and server
 */
typedef struct Dat_traceCompBlock
{
    /**
     * @brief   Name of the trace object
     */
    char            name[DAT_MAX_CHAR];

    /**
     * @brief   Offset of the trace component block
     */
    uint32_t        blockOffset;

    /**
     * @brief   Size of the trace component block
     */
    uint32_t        blockSize;

    /**
     * @brief   Flag indicates this is the last blcok
     */
    uint32_t        lastBlock;

}Dat_traceCompBlock;

/**
 * @brief
 *  Configuration structure used to get class verbosity level.
 *
 * @details
 *  This data structure is used to return Class and common component verbosity settings.
 */
typedef struct Dat_classQueryCfg
{
    Dat_TraceLevel     classLevel;
    Dat_TraceLevel     commonCompLevel;
}Dat_classQueryCfg;

/**
 * @brief
 *  Configuration structure used to get individual component verbosity level.
 *
 * @details
 *  This data structure is used to return settings for a component.
 */
typedef struct Dat_componentQueryCfg
{
    uint32_t                componentId;
    Dat_TraceComponentCfg   componentInfo;
}Dat_componentQueryCfg;

/**
 * @brief
 *  Data structure used to query trace objects.
 *
 * @details
 *  This is an internal data structure used for trace object querying.
 */
typedef struct Dat_verbosityQueryCfg
{
    /**
     * @brief   Name of the trace object
     */
    char            name[DAT_MAX_CHAR];

    /**
     * @brief   Configuration for querying class/common component/component
     */
    union
    {
        uint32_t               numComponents;
        Dat_classQueryCfg      classCfg;
        Dat_componentQueryCfg  compCfg;
    }cfg;

}Dat_verbosityQueryCfg;


/**
 * @brief   DAT verbosity mask for non-focused components.
 */
#define DAT_COMP_NONFOCUSED_MASK        0x0000FFFF

/**
 * @brief   Mask for user-defined bits of the class mask.
 */
#define DAT_CLASS_USERDEF_MASK          ((uint32_t)((DAT_CLASS_LEVEL_USERDEF0 |\
                                         DAT_CLASS_LEVEL_USERDEF1 |\
                                         DAT_CLASS_LEVEL_USERDEF2 |\
                                         DAT_CLASS_LEVEL_USERDEF3 |\
                                         DAT_CLASS_LEVEL_USERDEF4 |\
                                         DAT_CLASS_LEVEL_USERDEF5 |\
                                         DAT_CLASS_LEVEL_USERDEF6 |\
                                         DAT_CLASS_LEVEL_USERDEF7 |\
                                         DAT_CLASS_LEVEL_USERDEF8 |\
                                         DAT_CLASS_LEVEL_USERDEF9 )<<\
                                         DAT_MASK_SIZE))

/**
 * @brief   Mask for non user-defined bits of the class mask.
 */
#define DAT_CLASS_BIOS_MASK             ((uint32_t)((DAT_CLASS_LEVEL_BIOSLOAD |\
                                          DAT_CLASS_LEVEL_BIOSTASK |\
                                          DAT_CLASS_LEVEL_BIOSMAIN |\
                                          DAT_CLASS_LEVEL_HWI |\
                                          DAT_CLASS_LEVEL_SWI) <<\
                                          DAT_MASK_SIZE))

/**
 * @brief   DAT class mask.
 */
#define DAT_CLASS_MASK                  (DAT_CLASS_BIOS_MASK | DAT_CLASS_USERDEF_MASK)

/**
 * @brief   Block size in number of bytes, used for transfer trace object components
 *          between DAT server and client.
 */
#define DAT_COMPONENT_INIT_BLOCKSIZE     1024

/**
 * @brief
 *  DAT Client MCB
 *
 * @details
 *  The structure describes the DAT client
 */
typedef struct Dat_ClientMCB
{
    /**
     * @brief   Client configuration
     */
    Dat_ClientCfg       cfg;

    /**
     * @brief   Opaque handle to the DAT server which is being used by the server
     */
    Dat_ServerHandle    serverHandle;

    /**
     * @brief  Execution realm of the DAT server
     */
    Dat_ExecutionRealm  serverRealm;

    /**
     * @brief  Status of the DAT Client.
     */
    Dat_ClientStatus    status;

    /**
     * @brief  Reader MSGCOM channel associated with the DAT client which is used
     * to receive messages from the DAT server
     */
    MsgCom_ChHandle     clientChannel;

    /**
     * @brief  Writer MSGCOM channel associated with the DAT client which is used
     * to send messages to the DAT server
     */
    MsgCom_ChHandle     serverChannel;

    /**
     * @brief  JOSH node handle associated with the DAT client which is used to execute
     * jobs between the DAT client & server.
     */
    Josh_NodeHandle     joshHandle;

#ifdef __ARMv7
    /**
     * @brief  FD for Hplib module, used on ARM core only
     */
    int32_t             hplibModFd;
#endif

    /*
     * @brief   LoggerStreamer2 init indicator for CUIA
     */
    uint32_t            loggerStreamerInitialized;

    /**
     * @brief   UIO map information assoicated with the producers created on ARM
     */
    Dat_uioMap*         ptrUioMapInfo;

    /**
     * @brief   List of the producers created by the DAT client
     */
    Dat_Producer*       ptrProducerList;

    /**
     * @brief   List of the consumers created by the DAT client
     */
    Dat_Consumer*       ptrConsumerList;

    /**
     * @brief   List of the Local trace object created by the DAT client
     */
    Dat_LocalTraceObject* ptrLocalTraceObjList;

    /**
     * @brief   List of the free logger Instance ids on the DAT client
     */
    Dat_loggerInstId*   ptrLoggerIdFreeList;

    /**
     * @brief   Producer controller list on a Dat client
     */
    Dat_prodController* Dat_prodCtrlrList;

}Dat_ClientMCB;

/**
 * @brief
 *  DAT Connect Information
 *
 * @details
 *  The structure describes information which is required to connect the
 *  DAT consumer with the DAT producer
 */
typedef struct Dat_ConnectInfo
{
    /**
     * @brief  Handle to the consumer server
     */
    Dat_ServerConsHandle    serverConsumerHandle;

    /**
     * @brief  Name of the producer to which the consumer is connecting
     */
    char                    producerName[DAT_MAX_CHAR];

    /**
     * @brief  Reader MSGCOM channel name created on the consumer. The
     * producer needs to send data to this channel
     */
    char                    consumerChannelName[MSG_COM_MAX_CHANNEL_LEN];
}Dat_ConnectInfo;

/**
 * @brief
 *  DAT Client Blocks
 *
 * @details
 *  The structure describes the DAT clients blocks which describe each DAT client
 *  block which is attached to the DAT server
 */
typedef struct Dat_ClientBlock
{
    /**
     * @brief  Name of the DAT client
     */
    char                    name[NETFP_MAX_CHAR];

    /**
     * @brief  Handle of the DAT client. This is an opaque handle from the DAT Server's
     * perspective.
     */
    Dat_ClientHandle        clientHandle;

    /**
     * @brief  Status of the DAT client on the server.
     */
    Dat_ClientStatus        status;

    /**
     * @brief  Handle of the server MSGCOM channel. There is a unique MSGCOM channel created
     * on the server which maps to a single DAT client.
     */
    MsgCom_ChHandle         serverChannel;

    /**
     * @brief  Handle of the client MSGCOM channel. There is a unique MSGCOM writer channel
     * found on the server which maps to a single DAT client.
     */
    MsgCom_ChHandle         clientChannel;

    /**
     * @brief  JOSH Node handle which has the JOSH framework handle between the DAT
     * server and the client. Active clients need to have a valid JOSH handle else the
     * client block is considered to be inactive and the rest of the fields in this
     * structure are ignored.
     */
    Josh_NodeHandle         joshHandle;
}Dat_ClientBlock;

/**
 * @brief
 *  DAT Server MCB
 *
 * @details
 *  The structure describes the DAT server
 */
typedef struct Dat_ServerMCB
{
    /**
     * @brief   Server configuration
     */
    Dat_ServerCfg           cfg;

    /**
     * @brief   Database handle associated with the DAT server. This is the database handle
     * which is used by the DAT server to store all its information.
     */
    Name_DBHandle           databaseHandle;

    /**
     * @brief   This tracks all the DAT clients which are attached to the DAT server
     */
    Dat_ClientBlock         clientBlock[DAT_MAX_CLIENTS];

    /**
     * @brief   List of the producers created on the DAT server
     */
    Dat_ServerProducer*     ptrProducerList;

    /**
     * @brief   List of the unattached consumers created on the DAT server. Consumer is
     * unattached if it is created but has not been connected to a producer.
     */
    Dat_ServerConsumer*     ptrUnattachedConsumerList;

    /**
     * @brief   List of the consumers on the DAT server which have been created and are
     * pending "connect" verifications.
     */
    Dat_ServerConsumer*     ptrPendingConnectList;

    /**
     * @brief   List of the consumers on the DAT server which have been created and are
     * pending "disconnect" verifications.
     */
    Dat_ServerConsumer*     ptrPendingDisconnectList;

    /**
     * @brief   List of the global trace object on the DAT server which have been created.
     */
    Dat_ServerGlobalTraceObject*  ptrGlobalTraceObjectList;

    /**
     * @brief   List of the trace object instances on the DAT server that have pending
     * verbosity operations.
     */
    Dat_ServerLocalTraceObject*     ptrPendingVerbosityOpList;

    /**
     * @brief   List of the producers on the DAT server that have pending freeze/unfreeze
     * operations.
     */
    Dat_ServerLocalTraceObject*     ptrPendingProducerOpList;

}Dat_ServerMCB;


/**
 * @brief
 *  DAT verbosiy modification block
 *
 * @details
 *  The structure describes the DAT verbosity modification block
 */
typedef struct Dat_verbosityOpBlock
{
    /**
     * @brief   Links used to put verbosity operation list
     */
    Dat_ListNode                opLinks;

    /**
     * @brief   Job identifier associated while the consumer is being connected
     * This is valid only when the consumer is in the pending list
     */
    int32_t                     jobId;

    /**
     * @brief   JOSH Node handle to the DAT client which created the producer
     */
    Josh_NodeHandle             joshNodeHandle;

    /**
     * @brief   Handle to the trace object
     */
    Dat_ServerLocalTraceObject* ptrTraceObj;
}Dat_verbosityOpBlock;

/**
@}
*/

/**
 *  @b Description
 *  @n
 *      Utility wrapper function which provides logs DAT server messages
 *      via the application supplied logging function
 *
 *  @param[in]  ptrDatServer
 *      Pointer to the DAT server
 *  @param[in]  logLevel
 *      Logging Level
 *  @param[in]  fmt
 *      Format string
 *  @param[in]  ...
 *      Variable arguments
 *
 *  \ingroup DAT_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable.
 */
static inline void Dat_logMsg
(
    Dat_ServerMCB*      ptrDatServer,
    Dat_LogLevel        logLevel,
    char*               fmt,
    ...
)
{
    va_list arg;

    /* Pass control to the application supplied calling function. */
    if (ptrDatServer->cfg.logFxn)
    {
        va_start (arg, fmt);
        ptrDatServer->cfg.logFxn (logLevel, fmt, arg);
        va_end (arg);
    }
    return;
}

/* Internal Exported API */
extern void Dat_registerServices(Josh_NodeHandle nodeHandle);
extern Dat_ServerConsHandle _Dat_createConsumer(Dat_ServerHandle srvHandle, Dat_ClientHandle clientHandle, int32_t* errCode);
extern int32_t _Dat_deleteConsumer(Dat_ServerHandle srvHandle, Dat_ServerConsHandle , int32_t* errCode);
extern int32_t _Dat_createProducer(Dat_ServerHandle srvHandle, Dat_ClientHandle clientHandle, Dat_producerInfo* producerInfo, int32_t* errCode);
extern int32_t _Dat_deleteProducer(Dat_ServerHandle srvHandle, char* name, int32_t* errCode);
extern int32_t _Dat_connectConsumer(Dat_ServerHandle serverHandle, Dat_ConnectInfo* ptrConnectInfo, int32_t* errCode);
extern int32_t __Dat_connectConsumer (Dat_ClientHandle clientHandle, Dat_ConnectInfo* ptrConnectInfo, int32_t* errCode);
extern int32_t _Dat_disconnectConsumer(Dat_ServerHandle serverHandle, Dat_ConnectInfo* ptrConnectInfo, int32_t* errCode);
extern int32_t __Dat_disconnectConsumer (Dat_ClientHandle clientHandle, Dat_ConnectInfo* ptrConnectInfo, int32_t* errCode);
extern int32_t _Dat_stopClient (Dat_ServerMCB* ptrDatServer, char* clientName, int32_t* errCode);
extern int32_t _Dat_getConsumerStatus(Dat_ServerMCB* ptrDatServer,Dat_ServerConsHandle serverConsumerHandle,
                                      Dat_ConsumerStatus* consumerStatus, int32_t* errCode);

extern int32_t Dat_primeProducerUIA (void* handle);
extern Ptr Dat_bufferExchangeGP(LoggerStreamer2_Handle handle, uint8_t* fillBuffer);
extern int32_t _Dat_getGPProdDataOffset (Dat_ServerHandle serverHandle, Dat_ServerConsHandle serverConsumerHandle,
                                         uint32_t* dataOffset, int32_t* errCode);

extern Dat_LoggerFuncTbl Dat_LoggerFuncUIA;
extern Dat_LoggerFuncTbl Dat_LoggerFuncGP;

/* Trace verbosity Control APIs*/
extern Dat_ServerTraceObjHandle _Dat_createTraceObject
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_TraceObjectCfg* ptrTraceObjCfg,
    int32_t*            errCode
);

extern Dat_ServerLocalTraceObjHandle _Dat_createTraceObjectInstance
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_TraceObjectBody* ptrTraceObjectBody,
    int32_t*            errCode
);

extern int32_t _Dat_initTraceComponentBlock
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_traceCompBlock* ptrCompBlock,
    int32_t*            errCode
);

extern int32_t _Dat_getTraceComponentBlock
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_traceCompBlock* ptrCompBlock,
    int32_t*            errCode
);

extern int32_t _Dat_deleteTraceObjectInstance
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    char*               traceObjectName,
    int32_t*            errCode
);

extern int32_t _Dat_deleteTraceObject
(
    Dat_ServerHandle    serverHandle,
    char*               traceObjectName,
    int32_t*            errCode
);

extern int32_t _Dat_modifyVerbosity
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_VerbosityCfg*   ptrVerbosityCfg,
    int32_t*            errCode
);

extern int32_t _Dat_getNumComponents
(
    Dat_ServerHandle       serverHandle,
    Dat_verbosityQueryCfg* queryCfg,
    int32_t*               errCode
);

extern int32_t _Dat_getClassVerbosity
(
    Dat_ServerHandle        serverHandle,
    Dat_verbosityQueryCfg*  queryCfg,
    int32_t*                errCode
);

extern int32_t _Dat_getComponentVerbosity
(
    Dat_ServerHandle        serverHandle,
    Dat_verbosityQueryCfg*  queryCfg,
    int32_t*                errCode
);

extern int32_t _Dat_getTraceObjectNames
(
    Dat_ServerHandle    serverHandle,
    uint32_t            nameArraySize,
    char*               nameArray,
    int32_t*            errCode
);

extern int32_t __Dat_modifyVerbosity
(
    Dat_ClientHandle    clientHandle,
    Dat_VerbosityCfg*   ptrVerbosityCfg,
    int32_t*            errCode
);

extern Dat_uioMap* Dat_uioInitMemMap
(
    Dat_ClientHandle clientHandle,
    char*            name,
    int32_t *errCode
);

extern int32_t Dat_UioDeinitMemMap
(
    Dat_ClientHandle clientHandle,
    Dat_uioMap* ptrUioMapInst,
    int32_t *errCode
);

extern uint32_t Dat_uio_read32
(
    Dat_uioMap*    ptrDatUioMap,
    uint32_t       offset
);

extern void Dat_uio_write32
(
    Dat_uioMap*    ptrDatUioMap,
    uint32_t       offset,
    uint32_t*      data
);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* __DAT_INTERNAL_H__ */

