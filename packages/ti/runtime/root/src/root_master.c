/**
 *   @file  root_master.c
 *
 *   @brief
 *      The file implements the root master services
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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* ROOT Include Files. */
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/root/include/root_internal.h>

#ifndef __ARMv7
#error "Root Master is applicable supported only on ARM"
#endif

/**************************************************************************
 ************************* Root Master Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This function is used to execute and process the root master. Applications
 *      should ensure that this function is called in a thread context and is executed
 *      once the IPC interrupt is raised. The root master will in turn handle all the
 *      slaves which have been registered with it.
 *
 *      NOTE: This function needs to be plugged correctly in each root master. Failure
 *      to do will prevent the master & slave to communicate with each other
 *
 *  @param[in]  rootMasterHandle
 *      Root Master Handle
 *  @param[in]  errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Root_executeMaster (Root_MasterHandle rootMasterHandle, int32_t* errCode)
{
    Root_MasterMCB*         ptrRootMasterMCB;
    Root_MasterSlaveMCB*    ptrRootMasterSlaveMCB;
    int32_t                 index;
    int32_t                 retVal;
    fd_set                  fds;

    /* Get the root master MCB */
    ptrRootMasterMCB = (Root_MasterMCB*)rootMasterHandle;
    if (ptrRootMasterMCB == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Set the read sockets */
    FD_ZERO(&fds);
    FD_SET(ptrRootMasterMCB->ipcInterruptFd, &fds);

    /* Wait for the data to arrive; all slaves notify the master via IPC interrupts. */
    retVal = select (ptrRootMasterMCB->ipcInterruptFd + 1, &fds, NULL, NULL, NULL);
    if (retVal < 0)
    {
        *errCode = errno;
        return -1;
    }

    /* Read from the DSP interrupt file descriptor.
     *  - This is required else the select will not block. */
    if (read(ptrRootMasterMCB->ipcInterruptFd, &retVal, sizeof(uint32_t)) != sizeof(uint32_t))
    {
        printf ("Root Internal Error: UINTC UIO read failed [%s]\n", strerror(errno));
        return -1;
    }

    /* The UIO module disable the event which needs to be reenabled from the application else
     * there will be no more interrupts detected. */
    if (Uintc_enableEvent (ptrRootMasterMCB->uintcHandle, ptrRootMasterMCB->cfg.ipcSrcId, errCode) < 0)
    {
        printf("Root Internal Error: Enable events failed [%d]\n", *errCode);
        return -1;
    }

    /* Cycle through all the specified slaves */
    for (index = 0; index < ptrRootMasterMCB->cfg.numSlaves; index++)
    {
        /* Get the pointer to the slave MCB */
        ptrRootMasterSlaveMCB = &ptrRootMasterMCB->masterSlaveMCB[index];

        /* Process the received JOSH request */
        if (Josh_receive(ptrRootMasterSlaveMCB->joshNodeHandle, errCode) < 0)
        {
            /* Error: Determine the error code. */
            if (*errCode == JOSH_ENOMSG)
            {
                /* There was no message to process. This is not a FATAL error since we have multiplexed a single
                 * IPC interrupt to multiple cores. */
                continue;
            }

            /* JOSH Failed: Return failure with the JOSH error code. */
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the root master which is responsible
 *      for monitoring all the configured root slaves and can be used by
 *      the applications to initialize or deinitialized these root slaves.
 *
 *  @param[in]  ptrRootCfg
 *      Pointer to the root configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   Root master handle
 *  @retval
 *      Error   -   NULL
 */
Root_MasterHandle Root_masterCreate (Root_MasterConfig* ptrRootCfg, int32_t* errCode)
{
    Root_MasterMCB*         ptrRootMasterMCB;
    Root_MasterSlaveMCB*    ptrRootMasterSlaveMCB;
    int32_t                 index;
    uint8_t*                ptrSharedMemory;
    struct sockaddr_un 	    socketAddress;
    uint32_t                slaveCoreId;
    UintcConfig             uintcConfig;

    /* Sanity Check: Validate the arguments */
    if (ptrRootCfg == NULL)
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the OSAL Function Table */
    if ((ptrRootCfg->osalFxn.malloc         == NULL)  || (ptrRootCfg->osalFxn.free == NULL)          ||
        (ptrRootCfg->osalFxn.enterCS        == NULL)  || (ptrRootCfg->osalFxn.exitCS == NULL)        ||
        (ptrRootCfg->osalFxn.beginMemAccess == NULL)  || (ptrRootCfg->osalFxn.endMemAccess == NULL)  ||
        (ptrRootCfg->osalFxn.createSem      == NULL)  || (ptrRootCfg->osalFxn.deleteSem == NULL)     ||
        (ptrRootCfg->osalFxn.postSem        == NULL)  || (ptrRootCfg->osalFxn.pendSem == NULL))
    {
        *errCode = ROOT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Cycle through all the specified domains and validate to ensure that the configuration is correct. */
    for (index = 0; index < ptrRootCfg->numSlaves; index++)
    {
        /* Sanity Check: The slave core identifier should be specified using the ROOT_SLAVE_COREID macro.
         * Failure to do this results in an error */
        if (Root_isSlaveCoreIdValid(ptrRootCfg->slaveCoreId[index]) == 0)
        {
            *errCode = ROOT_EINVAL;
            return NULL;
        }

        /* Get the slave core identifier */
        slaveCoreId = Root_getSlaveCoreId (ptrRootCfg->slaveCoreId[index]);

        /* Derive the shared memory structure */
        ptrSharedMemory = (uint8_t*)(ptrRootCfg->ptrSharedMemory + (slaveCoreId * sizeof(Root_SharedMemStruct)));

        /* Before we use the memory ensure that we have not gone beyond the configuration. */
        if (ptrSharedMemory > (ptrRootCfg->ptrSharedMemory + ptrRootCfg->sizeSharedMemory))
        {
            *errCode = ROOT_EINVAL;
            return NULL;
        }
    }

    /* Allocate memory for the root master MCB */
    ptrRootMasterMCB = malloc (sizeof(Root_MasterMCB));
    if (ptrRootMasterMCB == NULL)
    {
        *errCode = ROOT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrRootMasterMCB, 0, sizeof(Root_MasterMCB));

    /* Populate the Root MCB */
    memcpy ((void *)&ptrRootMasterMCB->cfg, (void *)ptrRootCfg, sizeof(Root_MasterConfig));

    /* Populate the user space interrupt configuration: */
    snprintf(uintcConfig.name, sizeof(uintcConfig.name), "%s", ptrRootCfg->rootMasterName);
    uintcConfig.mode = Uintc_Mode_APPLICATION_MANAGED;

    /* Initialize the user space interrupt module */
    ptrRootMasterMCB->uintcHandle = Uintc_init (&uintcConfig, errCode);
    if (ptrRootMasterMCB->uintcHandle == NULL)
    {
        /* Error: UINTC initialization failed. */
        free (ptrRootMasterMCB);
        return NULL;
    }

    /* Register the ISR with the UINTC module. */
    ptrRootMasterMCB->ipcInterruptFd = Uintc_registerIsr (ptrRootMasterMCB->uintcHandle, ptrRootCfg->ipcSrcId, NULL,
                                                         (void*)ptrRootCfg->ipcSrcId, errCode);
    if (ptrRootMasterMCB->ipcInterruptFd < 0)
    {
        free (ptrRootMasterMCB);
        return NULL;
    }

    /* Create the root master socket. */
    ptrRootMasterMCB->masterSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (ptrRootMasterMCB->masterSocket < 0)
    {
        /* Error: Root Master socket creation failed. */
    	perror("Error: Root Master socket creation Failed");
        *errCode = errno;
        free (ptrRootMasterMCB);
        return NULL;
	}

    /* Initialize the socket information. */
    memset(&socketAddress, 0, sizeof(socketAddress));
    socketAddress.sun_family = AF_UNIX;
    snprintf (socketAddress.sun_path, sizeof(socketAddress.sun_path), "%s/%s", Syslib_getRunTimeDirectory(), ptrRootCfg->rootMasterName);

    /* Unlink any previous root master instances: */
    unlink (socketAddress.sun_path);

	/* Bind the root master socket. */
    *errCode = bind(ptrRootMasterMCB->masterSocket, (struct sockaddr *)&socketAddress, SUN_LEN(&socketAddress));
    if (*errCode < 0)
    {
		/* Error: Unable to bind the socket. */
    	perror("Error: Root master socket BIND failed\n");
        *errCode = errno;
        free (ptrRootMasterMCB);
        return NULL;
	}

    /* Cycle through and setup all the slaves which will be managed by the root master. */
    for (index = 0; index < ptrRootMasterMCB->cfg.numSlaves; index++)
    {
        /* Get the root master slave MCB */
        ptrRootMasterSlaveMCB = &ptrRootMasterMCB->masterSlaveMCB[index];

        /* Get the slave core identifier */
        slaveCoreId = Root_getSlaveCoreId (ptrRootCfg->slaveCoreId[index]);

        /* Get the shared memory for the specific domain identifier. */
        ptrSharedMemory = (uint8_t*)(ptrRootCfg->ptrSharedMemory + (slaveCoreId * sizeof(Root_SharedMemStruct)));

        /* Populate the fields in the root master slave MCB which keeps track of
         * properties of each root slave */
        strcpy(ptrRootMasterSlaveMCB->rootMasterName, ptrRootCfg->rootMasterName);
        ptrRootMasterSlaveMCB->ptrBootCfgRegs    = (CSL_BootcfgRegs*)Root_mapPhyAddrToVirtual((uint32_t)ptrRootMasterMCB->cfg.bootCfgAddress,
                                                                                              sizeof(CSL_BootcfgRegs));
        ptrRootMasterSlaveMCB->ptrSharedMem      = (Root_SharedMemStruct*)Root_mapPhyAddrToVirtual((uint32_t)ptrSharedMemory,
                                                                                                   sizeof(Root_SharedMemStruct));
        ptrRootMasterSlaveMCB->module            = Root_Module_MASTER;
        ptrRootMasterSlaveMCB->ipcSrcId          = ptrRootMasterMCB->cfg.ipcSrcId;
        ptrRootMasterSlaveMCB->peerCoreId        = Root_getSlaveCoreId(ptrRootCfg->slaveCoreId[index]);
        ptrRootMasterSlaveMCB->masterSlaveSocket = ptrRootMasterMCB->masterSocket;
        ptrRootMasterSlaveMCB->slaveCoreId       = 0xFFFFFFFF;                      /* Not applicable. */

        /* Determine if the root slaves are on the DSP or on the ARM */
        if (Root_isSlaveOnDSP (ptrRootCfg->slaveCoreId[index]) == 0)
            ptrRootMasterSlaveMCB->realm         = Root_ExecutionRealm_ARM;
        else
            ptrRootMasterSlaveMCB->realm         = Root_ExecutionRealm_DSP;

        /* Copy over the OSAL function table */
        memcpy ((void*)&ptrRootMasterSlaveMCB->osalFxn, (void *)&ptrRootCfg->osalFxn, sizeof(Root_OsalFxn));

        /* The master is responsible for resetting the shared memory address between the master & slave */
        memset ((void *)ptrRootMasterSlaveMCB->ptrSharedMem, 0, sizeof(Root_SharedMemStruct));

        /* Debug message: */
        printf ("Debug: Root Master -> Slave %d IPC Src Id: %d Physical %p Virtual %p BootCfg Physical %x Virtual %p\n",
                slaveCoreId, ptrRootCfg->ipcSrcId, ptrSharedMemory, ptrRootMasterSlaveMCB->ptrSharedMem,
                ptrRootMasterMCB->cfg.bootCfgAddress, ptrRootMasterSlaveMCB->ptrBootCfgRegs);

        /* Initialize the domain JOSH services*/
        ptrRootMasterSlaveMCB->joshNodeHandle = Root_initializeDomainServices(ptrRootMasterSlaveMCB, errCode);
        if (ptrRootMasterSlaveMCB->joshNodeHandle == NULL)
        {
            /* Error: Domain service initialization failed. The root domain cannot be created */
            free (ptrRootMasterMCB);
            return NULL;
        }
    }
    return (Root_MasterHandle)ptrRootMasterMCB;
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
    appUp (appId, appDomainHandle);
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
    appDown (appDomainHandle, arg0, arg1, arg2);
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
    appDeinitialized (appId);
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
void _Root_appInit(uint32_t appId, void* ptrDomainCfg, Root_SyslibConfig* ptrSyslibCfg)
{
    printf ("FATAL Error: Invoking appInit in the ROOT Master\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the specified domain.
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
    printf ("FATAL Error: Invoking appDeinit in the ROOT Master\n");
}

