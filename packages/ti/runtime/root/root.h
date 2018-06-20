/**
 *   @file  root.h
 *
 *   @brief
 *      Header file for the root library.
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

/** @defgroup ROOT_LIB_API Root Library
 */
#ifndef __ROOT_H__
#define __ROOT_H__

/* Standard Include Files. */
#include <stdint.h>
#include <stdio.h>

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup ROOT_SYMBOL  Root Defined Symbols
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_DATA_STRUCTURE  Root Data Structures
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_OSAL_API  Root OS Abstraction Layer
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_ENUM  Root Enumerations
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_INTERNAL_ENUM  Root Internal Enumerations
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_INTERNAL_DATA_STRUCTURE  Root Internal Data Structure
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_INTERNAL_FUNCTION  Root Internal Functions
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_ERROR_CODE  Root Error code
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_FUNCTIONS  Root exported functions
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_ARM_FUNCTIONS  Root Internal ARM Realm functions
@ingroup ROOT_LIB_API
*/
/**
@defgroup ROOT_DSP_FUNCTIONS  Root Internal DSP Realm functions
@ingroup ROOT_LIB_API
*/

/** @addtogroup ROOT_ERROR_CODE
 *
 * @brief
 *  Base error code for the ROOT module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief   Error Code: Out of memory
 */
#define ROOT_ENOMEM                        SYSLIB_ERRNO_ROOT_BASE-1

/**
 * @brief   Error Code: Invalid Arguments.
 */
#define ROOT_EINVAL                        SYSLIB_ERRNO_ROOT_BASE-2

/**
 * @brief   Error Code: Root services are not available.
 */
#define ROOT_ENOTREADY                     SYSLIB_ERRNO_ROOT_BASE-3

/**
 * @brief   Error Code: Internal Error
 */
#define ROOT_EINTERNAL                     SYSLIB_ERRNO_ROOT_BASE-4

/**
 * @brief   Error Code: Internal JOSH error
 */
#define ROOT_EJOSH                         SYSLIB_ERRNO_ROOT_BASE-5

/**
@}
*/

/** @addtogroup ROOT_SYMBOL
 @{ */

/**
 * @brief   Root Library Version. Versions numbers are encoded in the following
 * format:
 *  0xAABBCCDD -> Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 */
#define ROOT_VERSION_ID                         (0x01000000)

/**
 * @brief   Maximum number of characters
 */
#define ROOT_MAX_CHAR                           32

/**
 * @brief
 *  Maximum number of DSP cores which can be managed by a single root master.
 */
#define ROOT_MASTER_MAX_DSP_CORES               32

/**
 *  @b Description
 *  @n
 *      The macro is used to convert the core number to a slave core identifier
 *
 *  @param[in] DSP
 *      Flag which is set to 1 to indicate that the slave executes on DSP else
 *      set to 0 to indicate that the slave executes on ARM
 *  @param[in] CORENUM
 *      Core number or unique identifier. For DSP this should be the core number
 *      but for ARM process this identifier should be set to be immediately after
 *      the DSP core numbers. This number is used to identify which shared memory
 *      block is to be used for communication between the DSP and ARM
 */
#define ROOT_SLAVE_COREID(DSP, CORENUM)        ((1 << 31) | (DSP << 30) | CORENUM)

/**
@}
*/

/** @addtogroup ROOT_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to allocate memory
 *
 *  @param[in]  numBytes
 *      Number of bytes of memory to be allocated
 *  @param[in]  alignment
 *      Alignment requirements
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
typedef void* (*Osal_RootAlloc)(uint32_t numBytes, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to free memory
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  numBytes
 *      Number of bytes of memory to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Osal_RootFree)(void* ptr, uint32_t numBytes);

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to create a semaphore
 *
 *  @retval
 *      Opaque critical section handle
 */
typedef void* (*Osal_RootCreateSem)(void);

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
typedef void (*Osal_RootPendSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_RootPostSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_RootDeleteSem)(void* semHandle);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to enter the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @retval
 *      Opaque critical section handle
 */
typedef void* (*Osal_RootEnterCS)(void);

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to exit the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @param[in]  csHandle
 *      Opaque critical section handle
 *
 *  @retval
 *      Not Applicable.
 */
typedef void (*Osal_RootExitCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to invalidate the contents
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
typedef void (*Osal_RootBeginMemoryAccess)(void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to writeback the contents
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
typedef void (*Osal_RootEndMemoryAccess)(void* ptr, uint32_t size);

/**
@}
*/


/**
 * @brief   Root Slave Handle
 */
typedef void*       Root_SlaveHandle;

/**
 * @brief   Root Master Handle
 */
typedef void*       Root_MasterHandle;

/**
 * @brief   Root Slave core identifier (Generated using the ROOT_SLAVE_COREID macro)
 *
 * @sa ROOT_SLAVE_COREID
 */
typedef uint32_t    Root_SlaveCoreId;

/**
 * @brief
 *  Root OSAL calltable
 *
 * @details
 *  Root OS call table which exposes all the OS functions which
 */
typedef struct Root_OsalFxn
{
    Osal_RootAlloc              malloc;
    Osal_RootFree               free;
    Osal_RootCreateSem          createSem;
    Osal_RootDeleteSem          deleteSem;
    Osal_RootPostSem            postSem;
    Osal_RootPendSem            pendSem;
    Osal_RootEnterCS            enterCS;
    Osal_RootExitCS             exitCS;
    Osal_RootBeginMemoryAccess  beginMemAccess;
    Osal_RootEndMemoryAccess    endMemAccess;
}Root_OsalFxn;

/**
 * @brief
 *  Root master configuration
 *
 * @details
 *  On ARM there could be a single root master which manages multiple root domains on
 *  the DSP. This structure describes the configuration of the root master.
 */
typedef struct Root_MasterConfig
{
    /**
     * @brief  Unique name which identifies the root master.
     */
    char                    rootMasterName[ROOT_MAX_CHAR];

    /**
     * @brief  This is the address of the shared memory which is used for all root communication.
     * The address of the shared memory should be the same across the ARM and DSP.
     */
    uint8_t*                ptrSharedMemory;

    /**
     * @brief  This is the size of the shared memory which has been allocated for all root
     * communication. The size should be the same across the ARM and DSP.
     */
    uint32_t                sizeSharedMemory;

    /**
     * @brief  This is the number of domains which are a part of the specific root master
     */
    uint8_t                 numSlaves;

    /**
     * @brief  This is the *slave* core identifiers which are managed by this master domain
     * Use the ROOT_SLAVE_COREID to configure this field.
     *
     * @sa ROOT_SLAVE_COREID
     */
    Root_SlaveCoreId        slaveCoreId[ROOT_MASTER_MAX_DSP_CORES];

    /**
     * @brief  This is the device specific boot configuration physical address
     */
    uint32_t                bootCfgAddress;

    /**
     * @brief  This is the IPC (Inter process core communication) source identifier which
     * is used for communication between the master and slaves. This should be same across
     * the master and all the slaves.
     */
    uint32_t                ipcSrcId;

    /**
     * @brief  OSAL Function calltable
     */
    Root_OsalFxn            osalFxn;
}Root_MasterConfig;

/**
 * @brief
 *  Root Slave configuration
 *
 * @details
 *  The root slave configuration describes the various configuration properties which
 *  are required  to initialize and start the root services on the slave. Slave configuration
 *  should match the master configuration and should be provisioned carefully.
 */
typedef struct Root_SlaveConfig
{
    /**
     * @brief  Unique name which identifies the root master to which the slave belongs.
     */
    char                    rootMasterName[ROOT_MAX_CHAR];

    /**
     * @brief  This is the address of the shared memory which is used for all root communication.
     * The address of the shared memory should be the same across the ARM and DSP.
     */
    uint8_t*                ptrSharedMemory;

    /**
     * @brief  This is the size of the shared memory which has been allocated for all root
     * communication. The size should be the same across the ARM and DSP.
     */
    uint32_t                sizeSharedMemory;

    /**
     * @brief  Master core identifier which indicate the core on which the master is executing.
     * Since there is only 1 master in the system this should be set to the ARM core identifier
     * on which the master will execute. Please refer to the Device specifications [IPC section]
     * to determine the correct usage here.
     */
    uint8_t                 masterCoreId;

    /**
     * @brief  Slave core identifier which indicate the core on which the slave is executing.
     * On the DSP there will be 1 root slave which will execute on each core; hence the slave
     * identifier should be the same as the DSP core number. However on ARM there could be
     * multiple processes which could execute as slaves. It is recommended to allocate ARM
     * processes a number range after the DSP core numbers. This should be set using the
     * ROOT_SLAVE_COREID macro.
     *
     * @sa ROOT_SLAVE_COREID
     */
    Root_SlaveCoreId        slaveCoreId;

    /**
     * @brief  This is the device specific boot configuration physical address
     */
    uint32_t                bootCfgAddress;

    /**
     * @brief  This is the IPC (Inter process core communication) source identifier which
     * is used for communication between the master and slaves. This should be the same
     * between the master and slave configuration.
     */
    uint32_t                ipcSrcId;

    /**
     * @brief  OSAL Function calltable
     */
    Root_OsalFxn            osalFxn;
}Root_SlaveConfig;

/**
 * @brief
 *  Root DSP SYSLIB configuration
 *
 * @details
 *  The structure defines the root DSP SYSLIB configuration which is populated
 *  and passed from the root master to the root slave (executing on DSP)
 */
typedef struct Root_DSPSyslibConfig
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
     * @brief  Shared Memory address which is used to hold the central named resource
     * database which is unique to all the root slaves in the DSP domain.
     */
    uint32_t        namedResourceSharedMemAddress;

    /**
     * @brief  Size of the shared memory address which is used to hold the central named
     * resource database.
     */
    uint32_t        sizeNamedResourceSharedMemAddress;

    /**
     * @brief  On the DSP the named resource is a shared memory database. If the shared
     * memory database is accessible across multiple cores then a hardware semaphore needs
     * to be specified here to protect the database. The hardware semaphore should be the
     * same between these DSP cores.
     */
    uint32_t        nameHwSemaphore;

    /**
     * @brief  Flag which indicates if the DSP is responsible for initializing the shared
     * memory named resource database. In the DSP group there should be only 1 master which
     * initializes the named resource database else there could be a race condition which
     * would overwrite the memory.
     */
    uint32_t        initNamedResourceDatabase;

    /**
     * @brief  ARM core identifier which is executing the SYSRM server.
     */
    uint32_t        armCoreId;

    /**
     * @brief  SYSRM IPC source identifier which allows the DSP RM clients to communicate
     * with the SYSRM server on ARM.
     */
    uint8_t         sysRMIPCSourceId;

    /**
     * @brief   Flag which indicates if the name proxy needs to be instantiated or not.
     * If the flag is set to 0; none of the other fields in the structure are used. On the
     * DSP root slaves there should be 1 agent server.
     */
    uint8_t         instantiateNameProxy;

    /**
     * @brief   Local flow identifier: This is the flow identifer which is used
     * to receive messages from the peer ARM agent server
     */
    uint32_t        localFlowId;

    /**
     * @brief   Remote flow identifier: This is the flow identifier which is used
     * to send messages to the peer ARM agent server
     */
    uint32_t        remoteFlowId;

    /**
     * @brief  Name Proxy Shared memory address for internal synchronization
     */
    uint32_t        nameProxySharedMemAddress;
}Root_DSPSyslibConfig;

/**
 * @brief
 *  Root Name client configuration
 *
 * @details
 *  The structure defines the root name client configuration which is
 *  populated and passed from the root master to the root slave.
 */
typedef struct Root_NameClientSyslibConfig
{
    /**
     * @brief   Flag which indicates if the name client needs to be instantiated on the
     * root slave or not.
     */
    uint8_t         instantiateNameClient;
}Root_NameClientSyslibConfig;

/**
 * @brief
 *  Root DAT client configuration
 *
 * @details
 *  The structure defines the root DAT client configuration which is
 *  populated and passed from the root master to the root slave.
 */
typedef struct Root_DatClientSyslibConfig
{
    /**
     * @brief   Flag which indicates if the DAT client needs to be instantiated on the
     * root slave or not.
     */
    uint8_t         instantiateDatClient;

    /**
     * @brief  Name of the DAT server to which the client is connecting.
     */
    char            serverName[ROOT_MAX_CHAR];

    /**
     * @brief  Name of the DAT client
     */
    char            clientName[ROOT_MAX_CHAR];

    /**
     * @brief  Identifier of the DAT client: This is the identifier which is configured in the
     * SYSTEM analyzer to visualize the UIA log messages which are generated by the DAT client.
     * In the case of DSP cores; this is typically configured as the CORE number.
     */
    uint32_t        datClientId;
}Root_DatClientSyslibConfig;

/**
 * @brief
 *  Root NETFP Client configuration
 *
 * @details
 *  The structure defines the root NETFP client configuration which is
 *  populated and passed from the root master to the root slave.
 */
typedef struct Root_NetfpClientSyslibConfig
{
    /**
     * @brief   Flag which indicates if the NETFP client needs to be instantiated on the
     * root slave or not.
     */
    uint8_t         instantiateNetfpClient;

    /**
     * @brief  Name of the NETFP server to which the client is connecting.
     */
    char            serverName[ROOT_MAX_CHAR];

    /**
     * @brief  Name of the NETFP client
     */
    char            clientName[ROOT_MAX_CHAR];
}Root_NetfpClientSyslibConfig;


/**
 * @brief
 *  Root NETFP Client configuration
 *
 * @details
 *  The structure defines the root NETFP client configuration which is
 *  populated and passed from the root master to the root slave.
 */
typedef struct Root_MemlogSyslibConfig
{
    /**
     * @brief   Flag which indicates if the NETFP client needs to be instantiated on the
     * root slave or not.
     */
    uint8_t         instantiateMemlog;
}Root_MemlogSyslibConfig;

/**
 * @brief
 *  Root SYSLIB configuration
 *
 * @details
 *  The structure defines the root SYSLIB configuration which is populated and passed from
 *  the root master to the root slave.
 */
typedef struct Root_SyslibConfig
{
    /**
     * @brief   Name of the root slave: This is a string which identifies the slave.
     */
    char                                name[ROOT_MAX_CHAR];

    /**
     * @brief  Name of the RM client which will execute on the root slave.
     */
    char                                rmClient[ROOT_MAX_CHAR];

    /**
     * @brief  Name of the RM server to which the RM client will communicate
     */
    char                                rmServer[ROOT_MAX_CHAR];

    /**
     * @brief   Name Proxy name to which the name clients will connect to *OR* this is
     * the name of the proxy which will advertise its services to the clients.
     */
    char                                proxyName[ROOT_MAX_CHAR];

    /**
     * @brief  This is the core identifier on which the root slave is executing.
     */
    uint32_t                            coreId;

    /**
     * @brief  This is the named resource instance identifier which is shared across the
     * entire realm.
     */
    uint32_t                            nrInstanceId;

    /**
     * @brief  This is the QMSS accumulator hardware semaphore which needs to be the same
     * across the entire system. Programming the QMSS accumulator is done via a common set
     * of registers. The register set needs to be protected against concurrent access.
     */
    uint32_t                            qmssAccHwSemaphore;

    /**
     * @brief   DSP Root slave configuration
     */
    Root_DSPSyslibConfig                dspSyslibConfig;

    /**
     * @brief   DAT Client SYSLIB configuration
     */
    Root_DatClientSyslibConfig          datClientConfig;

    /**
     * @brief   Name Client SYSLIB configuration
     */
    Root_NameClientSyslibConfig         nameClientConfig;

    /**
     * @brief   NETFP Client SYSLIB configuration
     */
    Root_NetfpClientSyslibConfig        netfpClientConfig;

   /**
     * @brief   NETFP Client SYSLIB configuration
     */
    Root_MemlogSyslibConfig             memlogConfig;
}Root_SyslibConfig;

/**
 *  @b Description
 *  @n
 *      Application initialization entry point which is invoked on each root slave
 *      when an application domain is created.
 *
 *      The function needs to be implemented on the root *slave*
 *
 *  @param[in]  appId
 *      Application Identifier
 *  @param[in]  ptrDomainCfg
 *      Pointer to the application domain configuration
 *  @param[in]  ptrSyslibCfg
 *      Pointer to the SYSLIB configuration.
 *
 *  @retval
 *     Not applicable
 */
extern void appInit (uint32_t appId, void* ptrDomainCfg, Root_SyslibConfig* ptrSyslibCfg);

/**
 *  @b Description
 *  @n
 *      Application deinitialization entry point which is invoked on each root slave
 *      when an application domain is destroyed.
 *
 *      The function needs to be implemented on the root *slave*
 *
 *  @param[in]  appDomainHandle
 *      Application Domain Handle
 *
 *  @retval
 *     Not application
 */
extern void appDeinit (void* appDomainHandle);

/**
 *  @b Description
 *  @n
 *      Once the application domain has been created on the root slave the application
 *      uses the following function to inform the root master that the domain is active
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appId
 *      Application Identifier
 *  @param[in]  appDomainHandle
 *      Opaque application domain handle passed back to the root master
 *
 *  @retval
 *     Not applicable
 */
extern void appUp (uint32_t appId, void* appDomainHandle);

/**
 *  @b Description
 *  @n
 *      The function is invoked from the application domain on the root slave to the
 *      root master to inform that the domain on the root slave is not operational
 *      any more. The arguments could be used to pass any application specific
 *      information back to the root master.
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appDomainHandle
 *      Opaque application domain handle which has gone down
 *  @param[in]  arg0
 *      Application specific argument0
 *  @param[in]  arg1
 *      Application specific argument1
 *  @param[in]  arg2
 *      Application specific argument2
 *
 *  @retval
 *     Not applicable
 */
extern void appDown (void* appDomainHandle, uint32_t arg0, uint32_t arg1, uint32_t arg2);

/**
 *  @b Description
 *  @n
 *      The root master will notify the slave to deinitialize themselves. Once the slaves
 *      have deinitialized themselves they will announce this back to the master. This
 *      function is invoked on the master on the slave notification.
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appId
 *      Application Identifier
 *
 *  @retval
 *     Not applicable
 */
extern void appDeinitialized (uint32_t appId);

/**
@}
*/

#ifdef __cplusplus
}
#endif

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

/* Root Slave Exported API: */
extern Root_SlaveHandle Root_slaveCreate (Root_SlaveConfig* ptrRootCfg, int32_t* errCode);
extern int32_t Root_executeSlave (Root_SlaveHandle rootSlaveHandle, int32_t* errCode);

/* Root Master Exported API: */
extern Root_MasterHandle Root_masterCreate (Root_MasterConfig* ptrRootCfg, int32_t* errCode);
extern int32_t Root_executeMaster (Root_MasterHandle rootMasterHandle, int32_t* errCode);

/* Root Master Application exported API: */
extern int32_t Root_appInit (Root_MasterHandle rootMasterHandle, Root_SlaveCoreId slaveCoreId, uint32_t appId,
                             void* ptrDomainCfg, uint32_t sizeDomainCfg, Root_SyslibConfig* ptrSyslibCfg,
                             int32_t* errCode);
extern int32_t Root_appDeinit(Root_MasterHandle rootMasterHandle, Root_SlaveCoreId slaveCoreId, void* appDomainHandle, int32_t* errCode);

/* Root Slave Application exported API: */
extern int32_t Root_appUp (Root_SlaveHandle rootSlaveHandle, uint32_t appId, void* appDomainHandle, int32_t* errCode);
extern int32_t Root_appDown (Root_SlaveHandle rootSlaveHandle, void* appDomainHandle,
                             uint32_t arg0, uint32_t arg1, uint32_t arg2, int32_t* errCode);
extern int32_t Root_appDeinitialized(Root_SlaveHandle rootSlaveHandle, uint32_t appId, int32_t* errCode);

#endif /* __ROOT_H__ */

