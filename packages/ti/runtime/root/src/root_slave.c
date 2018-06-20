/**
 *   @file  root_slave.c
 *
 *   @brief
 *      The file implements the root slave module services
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ROOT Include Files. */
#include <ti/runtime/root/root.h>
#include <ti/runtime/root/include/root_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ************************** Global Variables ******************************
 **************************************************************************/

/* Global Root Slave MCB: This is maintained in a global variable because
 * there is only a single instance of the root slave which can execute on
 * a DSP core or ARM process. */
Root_MCB    rootSlaveMCB;

/**************************************************************************
 ************************** Root Slave Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This function is used to execute and process the root slave. Applications
 *      should ensure that this function is called in a thread context and is executed
 *      once the IPC interrupt is raised.
 *
 *      NOTE: This function needs to be plugged correctly in each root slave. Failure
 *      to do will prevent the master & slave to communicate with each other
 *
 *  @param[in]  rootSlaveHandle
 *      Slave Root Handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Root_executeSlave (Root_SlaveHandle rootSlaveHandle, int32_t* errCode)
{
    Root_MCB*   ptrRoot;

    /* Get the Root MCB */
    ptrRoot = (Root_MCB*)rootSlaveHandle;
    if (ptrRoot == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Process the received JOSH request */
    if (Josh_receive(ptrRoot->masterSlaveMCB.joshNodeHandle, errCode) < 0)
        return -1;

    /* Slave execution was successful. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a root slave.
 *
 *  @param[in]  ptrRootSlaveCfg
 *      Pointer to the Root slave configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   Pointer to the created root handle
 *  @retval
 *      Error   -   NULL
 */
Root_SlaveHandle Root_slaveCreate (Root_SlaveConfig* ptrRootSlaveCfg, int32_t* errCode)
{
    Root_MCB*   ptrRoot;
    uint8_t*    ptrSharedMemory;
    uint32_t    slaveCoreId;

    /* Sanity Check: Validate the arguments */
    if (ptrRootSlaveCfg == NULL)
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the OSAL Function Table */
    if ((ptrRootSlaveCfg->osalFxn.malloc         == NULL)  || (ptrRootSlaveCfg->osalFxn.free == NULL)          ||
        (ptrRootSlaveCfg->osalFxn.enterCS        == NULL)  || (ptrRootSlaveCfg->osalFxn.exitCS == NULL)        ||
        (ptrRootSlaveCfg->osalFxn.beginMemAccess == NULL)  || (ptrRootSlaveCfg->osalFxn.endMemAccess == NULL)  ||
        (ptrRootSlaveCfg->osalFxn.createSem      == NULL)  || (ptrRootSlaveCfg->osalFxn.deleteSem == NULL)     ||
        (ptrRootSlaveCfg->osalFxn.postSem        == NULL)  || (ptrRootSlaveCfg->osalFxn.pendSem == NULL))
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the slave core identifier is setup correctly */
    if (Root_isSlaveCoreIdValid(ptrRootSlaveCfg->slaveCoreId) == 0)
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Get the slave core identifier */
    slaveCoreId = Root_getSlaveCoreId (ptrRootSlaveCfg->slaveCoreId);

    /* Sanity Check: Ensure that the shared memory structure is within limits of the domain identifier. */
    ptrSharedMemory = (uint8_t*)(ptrRootSlaveCfg->ptrSharedMemory + (slaveCoreId * sizeof(Root_SharedMemStruct)));
    if (ptrSharedMemory > (ptrRootSlaveCfg->ptrSharedMemory + ptrRootSlaveCfg->sizeSharedMemory))
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Get the pointer to the root slave MCB */
    ptrRoot = &rootSlaveMCB;

    /* Initialize the allocated memory block. */
    memset ((void *)ptrRoot, 0, sizeof(Root_MCB));

    /* Copy over the slave configuration. */
    memcpy ((void *)&ptrRoot->cfg, (void *)ptrRootSlaveCfg, sizeof(Root_SlaveConfig));

    /* Populate the root master/slave MCB */
    strncpy(ptrRoot->masterSlaveMCB.rootMasterName, ptrRootSlaveCfg->rootMasterName, ROOT_MAX_CHAR);

    /* Copy over the OSAL function table */
    memcpy ((void*)&ptrRoot->masterSlaveMCB.osalFxn, (void *)&ptrRootSlaveCfg->osalFxn, sizeof(Root_OsalFxn));

    /* Map the physical address to the virtual address. */
    ptrRoot->masterSlaveMCB.ptrSharedMem = (Root_SharedMemStruct*)Root_mapPhyAddrToVirtual ((uint32_t)ptrSharedMemory,
                                                                                             sizeof(Root_SharedMemStruct));
    if (ptrRoot->masterSlaveMCB.ptrSharedMem == NULL)
    {
        *errCode = ROOT_EINTERNAL;
        return NULL;
    }
    ptrRoot->masterSlaveMCB.ptrBootCfgRegs = (CSL_BootcfgRegs*)Root_mapPhyAddrToVirtual((uint32_t)ptrRootSlaveCfg->bootCfgAddress,
                                                                                         sizeof(CSL_BootcfgRegs));
    if (ptrRoot->masterSlaveMCB.ptrBootCfgRegs == NULL)
    {
        *errCode = ROOT_EINTERNAL;
        return NULL;
    }

    /* The root is executing in the slave execution context */
    ptrRoot->masterSlaveMCB.module      = Root_Module_SLAVE;
    ptrRoot->masterSlaveMCB.ipcSrcId    = ptrRootSlaveCfg->ipcSrcId;
    ptrRoot->masterSlaveMCB.peerCoreId  = ptrRootSlaveCfg->masterCoreId;
    ptrRoot->masterSlaveMCB.slaveCoreId = slaveCoreId;
    if (Root_isSlaveOnDSP (ptrRootSlaveCfg->slaveCoreId) == 0)
        ptrRoot->masterSlaveMCB.realm  = Root_ExecutionRealm_ARM;
    else
        ptrRoot->masterSlaveMCB.realm  = Root_ExecutionRealm_DSP;

    /* Initialize the slave socket. */
    if (Root_slaveInitializeSocket (&ptrRoot->masterSlaveMCB, errCode) < 0)
    {
        /* Error: Domain service initialization failed. The root domain cannot be created */
        return NULL;
    }

    /* Initialize the domain services. */
    ptrRoot->masterSlaveMCB.joshNodeHandle = Root_initializeDomainServices(&ptrRoot->masterSlaveMCB, errCode);
    if (ptrRoot->masterSlaveMCB.joshNodeHandle == NULL)
    {
        /* Error: Domain service initialization failed. The root domain cannot be created */
        return NULL;
    }
    return (Root_SlaveHandle)ptrRoot;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a domain with the specific configuration
 *
 *  @param[in]  appId
 *      Application identifier.
 *  @param[in]  ptrDomainCfg
 *      Pointer to the domain configuration
 *  @param[in]  ptrSyslibCfg
 *      Pointer to the SYSLIB configuration
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void _Root_appInit
(
    uint32_t            appId,
    void*               ptrDomainCfg,
    Root_SyslibConfig*  ptrSyslibCfg
)
{
    appInit (appId, ptrDomainCfg, ptrSyslibCfg);
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the specified domain
 *
 *  @param[in]  appDomainHandle
 *      Application domain handle
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void _Root_appDeinit(void* appDomainHandle)
{
    appDeinit (appDomainHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root master that the slave application
 *      is active
 *
 *  @param[in]  appId
 *      Application identifier
 *  @param[in] appDomainHandle
 *      Application domain handle
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void _Root_appUp
(
    uint32_t    appId,
    void*       appDomainHandle
)
{
    System_printf ("FATAL Error: Invoking appUp in the slave realm\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root master that the slave application
 *      is down
 *
 *  @param[in]  appDomainHandle
 *      Application domain handle
 *  @param[in]  arg0
 *      Application argument0
 *  @param[in]  arg1
 *      Application argument1
 *  @param[in]  arg2
 *      Application argument2
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void _Root_appDown
(
    void*       appDomainHandle,
    uint32_t    arg0,
    uint32_t    arg1,
    uint32_t    arg2
)
{
    System_printf ("FATAL Error: Invoking appDown in the slave realm\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the root master that the slave application
 *      is down
 *
 *  @param[in]  appId
 *      Application identifier
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void _Root_appDeinitialized
(
    uint32_t    appId
)
{
    System_printf ("FATAL Error: Invoking appDeinitialized in the slave realm\n");
}

