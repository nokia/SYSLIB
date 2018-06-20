/**
 *   @file  root_internal.h
 *
 *   @brief
 *      Internal header file for the root library.
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
#ifndef __ROOT_INTERNAL_H__
#define __ROOT_INTERNAL_H__

/* MCSDK Include files */
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_bootcfgAux.h>

/* SYSLIB Include files. */
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/root/root.h>

/* UINTC is applicable only in the ARM space */
#ifdef __ARMv7
#include <ti/runtime/uintc/uintc.h>
#endif

/**************************************************************************
 ************************ Local Definitions *******************************
 **************************************************************************/

/**
 * @brief
 *  Internal Definition: This is the number of JOSH buffers which are shared between
 *  the master and slave.
 */
#define ROOT_INTERNAL_NUM_JOSH_BUFFER       2

/**
 * @brief
 *  Internal Definition: This is the size of each JOSH buffer which is exchanged between
 *  the master and slave.
 */
#define ROOT_INTERNAL_SIZE_BUFFER           1536

/** @addtogroup ROOT_INTERNAL_ENUM
 @{ */

/**
 * @brief
 *  Root module
 *
 * @details
 *  Root services can be executed on the master or on the slave.
 */
typedef enum Root_Module
{
    /**
     * @brief   Executing on the master
     */
    Root_Module_MASTER = 0x1,

    /**
     * @brief   Executing on the slave
     */
    Root_Module_SLAVE = 0x2
}Root_Module;

/**
 * @brief
 *  Root Execution Realm
 *
 * @details
 *  Root services can be be executed on the DSP or on ARM
 */
typedef enum Root_ExecutionRealm
{
    /**
     * @brief   Executing on the ARM
     */
    Root_ExecutionRealm_ARM = 0x1,

    /**
     * @brief   Executing on the DSP
     */
    Root_ExecutionRealm_DSP = 0x2
}Root_ExecutionRealm;

/**
@}
*/

/** @addtogroup ROOT_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Root JOSH Buffer
 *
 * @details
 *  The structure describes the JOSH buffer wrapper data structure which is used to keep
 *  track of the buffers exchanged between the master & slave domains.
 *  */
typedef struct Root_JoshBuffer
{
    /**
     * @brief  JOSH Buffer status:
     */
    uint8_t         status[CACHE_L2_LINESIZE];

    /**
     * @brief  Actual JOSH buffer:
     */
    uint8_t         buffer[ROOT_INTERNAL_SIZE_BUFFER];
}Root_JoshBuffer;

/**
 * @brief
 *  Root shared memory structure
 *
 * @details
 *  The structure describes the shared memory structure which is used to exchange information
 *  between the root master and slaves. Since the master & slave can execute on the DSP or
 *  ARM and since the transport is shared memory the structure needs to be aligned to account
 *  for the DSP cache line size and ARM page sizes.
 */
typedef struct Root_SharedMemStruct
{
    /**
     * @brief  This is shared memory address which is used to hold the status of the slave end
     * of the root domain. Aligned on the DSP cache line boundary.
     */
    uint8_t             slaveRootStatus[CACHE_L2_LINESIZE];

    /**
     * @brief  This is shared memory address which is used to hold the status of the master end
     * of the root domain. Aligned on the DSP cache line boundary.
     */
    uint8_t             masterRootStatus[CACHE_L2_LINESIZE];

    /**
     * @brief  JOSH Buffer Management: This allows the exchange of JOSH buffers from the
     * master to the slave.
     */
    Root_JoshBuffer     master_slave;

    /**
     * @brief  JOSH Buffer Management: This allows the exchange of JOSH buffers from the
     * slave to the master.
     */
    Root_JoshBuffer     slave_master;

    /**
     * @brief  Padding to align to ARM 4K page size boundary.
     */
    uint8_t             pad[512];
}Root_SharedMemStruct;

/**
 * @brief
 *  Root Master-Slave MCB
 *
 * @details
 *  This structure is used to hold information which is required by each root
 *  master to communicate with the slave.
 */
typedef struct Root_MasterSlaveMCB
{
    /**
     * @brief  Name of the root master associated with the node.
     */
    char                    rootMasterName[ROOT_MAX_CHAR + 1];

    /**
     * @brief  Module on which the master/slave is executing.
     */
    Root_Module             module;

#ifdef __ARMv7
    /**
     * @brief  Socket associated with the root master slave node.
     */
    int32_t                 masterSlaveSocket;
#endif

    /**
     * @brief  Execution realm on which the master/slave is executing.
     */
    Root_ExecutionRealm     realm;

    /**
     * @brief  This is the pointer to the shared memory which is used for master/slave
     * communication.
     */
    Root_SharedMemStruct*   ptrSharedMem;

    /**
     * @brief  JOSH node handle associated with the root.
     */
    Josh_NodeHandle         joshNodeHandle;

    /**
     * @brief  Virtual address which points to the device boot configuration address.
     */
    CSL_BootcfgRegs*        ptrBootCfgRegs;

    /**
     * @brief  This is the slave core identifier which is used to configure the node.
     * This is valid and applicable only for root slaves.
     */
    uint32_t                slaveCoreId;

    /**
     * @brief  This is the IPC (Inter process core communication) source identifier which
     * is used for communication between the ARM and DSP roots. This should be same across the
     * DSP and ARM.
     */
    uint32_t                ipcSrcId;

    /**
     * @brief  This is the peer core identifier i.e.
     * - Slave Realm : This is initialized to the master core identifier
     * - Master Realm: This is initialized to the slave core identifier
     */
    uint32_t                peerCoreId;

    /**
     * @brief  OSAL callout function table
     */
    Root_OsalFxn            osalFxn;
}Root_MasterSlaveMCB;

/**
 * @brief
 *  Root MCB
 *
 * @details
 *  Root master control block
 */
typedef struct Root_MCB
{
    /**
     * @brief  This is the configuration passed during the root slave creation.
     */
    Root_SlaveConfig        cfg;

    /**
     * @brief  This is the pointer to the shared memory which is used for master/slave
     * communication.
     */
    Root_MasterSlaveMCB     masterSlaveMCB;
}Root_MCB;

/**
 * @brief
 *  Root Master MCB
 *
 * @details
 *  Each root master stores information in this MCB
 */
typedef struct Root_MasterMCB
{
    /**
     * @brief  This is the configuration passed during the root master creation.
     */
    Root_MasterConfig       cfg;

    /**
     * @brief  This is the socket associated with the root master and is used by the
     * ARM root slave clients to notify the master.
     */
    int32_t                 masterSocket;

    /**
     * @brief  This is the IPC interrupt file descriptor which is registered to be
     * to receive the IPC source interrupt.
     */
    int32_t                 ipcInterruptFd;

#ifdef __ARMv7
    /**
     * @brief  UINTC handle used to handle the interrupts in ARM user space.
     */
    UintcHandle             uintcHandle;
#endif

    /**
     * @brief  Master slave root MCB
     */
    Root_MasterSlaveMCB     masterSlaveMCB[ROOT_MASTER_MAX_DSP_CORES];
}Root_MasterMCB;

/**
@}
*/

/**
 *  @b Description
 *  @n
 *      The function is used to validate the slave core identifier.
 *
 *  @param[in]  slaveCoreId
 *      Slave core identifier.
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   1 (Slave core identifier is valid)
 *  @retval
 *      Error   -   0 (Invalid)
 */
static inline uint8_t Root_isSlaveCoreIdValid (uint32_t slaveCoreId)
{
    /* Slave core identifiers are specified using the ROOT_SLAVE_COREID macro. */
    if (slaveCoreId & 0x80000000)
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode and get the slave core identifier
 *
 *  @param[in]  slaveCoreId
 *      Slave core identifier.
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Slave core number
 */
static inline uint32_t Root_getSlaveCoreId (uint32_t slaveCoreId)
{
    return (slaveCoreId & 0x3FFFFFFF);
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine if the slave executes on DSP
 *      or on the ARM
 *
 *  @param[in]  slaveCoreId
 *      Slave core identifier.
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      0           -   Slave executes on the ARM
 *  @retval
 *      Non Zero    -   Slave executes on the DSP
 */
static inline uint32_t Root_isSlaveOnDSP (uint32_t slaveCoreId)
{
    return (slaveCoreId & 0x40000000);
}

/***********************************************************************************************
 ******************************* Internal Exported Functions ***********************************
 ***********************************************************************************************/

/* Root Domain Exported Functions: */
extern Josh_NodeHandle Root_initializeDomainServices (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode);

/* Root DSP/ARM Realm exported functions */
extern uint32_t Root_mapPhyAddrToVirtual (uint32_t phyAddr, uint32_t size);
extern int32_t  Root_slaveInitializeSocket (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode);
extern int32_t  Root_isSlaveBufferReady (Root_MasterSlaveMCB* ptrMasterSlaveMCB);
extern int32_t  Root_notifySlave  (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode);
extern int32_t  Root_notifyMaster (Root_MasterSlaveMCB* ptrMasterSlaveMCB, int32_t* errCode);

/* Root *JOSH* Exported Functions: */
extern void _Root_appInit(uint32_t appId, void* ptrDomainCfg, Root_SyslibConfig* ptrSyslibCfg);
extern void _Root_appDeinit(void* appDomainHandle);
extern void _Root_appUp(uint32_t appId, void* appDomainHandle);
extern void _Root_appDown (void* appDomainHandle, uint32_t arg0, uint32_t arg1, uint32_t arg2);
extern void _Root_appDeinitialized(uint32_t appId);

#endif /* __ROOT_INTERNAL_H__ */
