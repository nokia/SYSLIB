/**
 *   @file  netfp_srv.c
 *
 *   @brief
 *      The file implements the NETFP Server module
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
#include <sys/unistd.h> // fzm
#include <sys/fcntl.h> // fzm

/* PDK & CSL Include Files */
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_tsc.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>
#include <ti/runtime/netfp/include/netfp_josh_af_unix_functions.h> //fzm

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ********************** NETFP Client Server Functions *********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETCP subsystem
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   NETFP Server Handle
 *  @retval
 *      Error   -   NULL
 */
static int32_t Netfp_init (Netfp_ServerMCB* ptrNetfpServer, int32_t* errCode)
{
    Cppi_CpDmaInitCfg   cpdmaCfg;

    /* Initialize PASS CPDMA */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

    /* Populate the DMA configuration: Dont override the hardware configuration. SOC Init is responsible for the
     * hardware configuration. */
    cpdmaCfg.dmaNum       = Cppi_CpDma_PASS_CPDMA;
    cpdmaCfg.regWriteFlag = Cppi_RegWriteFlag_OFF;

    /* Open the PASS CPDMA */
    ptrNetfpServer->passCPDMAHandle = Cppi_open (&cpdmaCfg);
    if (ptrNetfpServer->passCPDMAHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Open the PA driver and initialize the PA subsystem */
    ptrNetfpServer->paHandle = Netfp_paInit(ptrNetfpServer, NETFP_MAX_L2_HANDLES, NETFP_MAX_L3_HANDLES);
    if (ptrNetfpServer->paHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the PA command handler and flows */
    if (Netfp_paInitCmdHandler(ptrNetfpServer, ptrNetfpServer->cfg.cmdHeapHandle) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Configure the PA subsystem to program and setup the command sets. The command sets need
     * to be programmed upfront and are then used while adding other LUT entries. */
    if (Netfp_paSetupCommandSets(ptrNetfpServer) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the SA LLD: */
    if (Netfp_saInit (ptrNetfpServer, errCode) < 0)
        return -1;

    /* NETFP initialized successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETFP server.
 *
 *  @param[in]  ptrServerCfg
 *      Pointer to the NETFP server configuration.
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   NETFP Server Handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_ServerHandle Netfp_initServer
(
    Netfp_ServerConfig* ptrServerCfg,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*        ptrNetfpServer;
    Name_ResourceCfg        namedResourceCfg;
    Name_DBHandle           databaseHandle;

    /* Sanity Check: Validate the arguments. */
    if (ptrServerCfg == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments. */
    if ((ptrServerCfg->serverHeapHandle == NULL)    ||
        (ptrServerCfg->msgcomInstHandle == NULL)    ||
        (ptrServerCfg->pktlibInstHandle == NULL)    ||
        (ptrServerCfg->sysRMHandle      == NULL) )
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check:Validate the OSAL call table */
    if ((ptrServerCfg->malloc                == NULL) || (ptrServerCfg->free                == NULL) ||
        (ptrServerCfg->mallocSecurityContext == NULL) || (ptrServerCfg->freeSecurityContext == NULL) ||
        (ptrServerCfg->beginMemAccess        == NULL) || (ptrServerCfg->endMemAccess        == NULL) ||
        (ptrServerCfg->enterCS               == NULL) || (ptrServerCfg->exitCS              == NULL) ||
        (ptrServerCfg->createSem             == NULL) || (ptrServerCfg->deleteSem           == NULL) ||
        (ptrServerCfg->postSem               == NULL) || (ptrServerCfg->pendSem             == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the named resource instance is valid */
    databaseHandle = Name_getDatabaseHandle (ptrServerCfg->nrInstanceId);
    if (databaseHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Each server name should be unique in the system */
    Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       ptrServerCfg->serverName, &namedResourceCfg, errCode);

    /* The instance name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the instance name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the server. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Allocate memory for the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)ptrServerCfg->malloc(sizeof(Netfp_ServerMCB), 0);
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrNetfpServer, 0, sizeof(Netfp_ServerMCB));

    /* Populate the configuration block. */
    memcpy ((void *)&ptrNetfpServer->cfg, (void *)ptrServerCfg, sizeof(Netfp_ServerConfig));

    /* Keep track of the name database handle */
    ptrNetfpServer->databaseHandle = databaseHandle;

    /* Initialize the NETCP subsystem  */
    if (Netfp_init (ptrNetfpServer, errCode) < 0)
        return NULL;

    /* Populate the named resource configuration. */
    namedResourceCfg.handle1  = (uint32_t)ptrNetfpServer;
    namedResourceCfg.handle2  = (uint32_t)ptrNetfpServer->cfg.realm;
    strncpy(namedResourceCfg.name, ptrServerCfg->serverName, NAME_MAX_CHAR);

    /* Create & register the NETFP server instance announcing that the NETFP server is operational. */
    if (Name_createResource (ptrNetfpServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                             &namedResourceCfg, errCode) < 0)
    {
        System_printf ("Error: Registering NETFP server '%s' failed [Error code %d]\n", namedResourceCfg.name, *errCode);
        return NULL;
    }

    /* Return the NETFP Server handle. */
    return (Netfp_ServerHandle)ptrNetfpServer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP client with the server. The NETFP
 *      client is operational only once it has been initialized and then registered
 *      with the NETFP server.
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP server
 *  @param[in]  clientName
 *      Name of the client which is being registered
 *  @param[in]  ptrDirectInterruptCfg
 *      Pointer to the direct interrupt configuration. NETFP Servers and clients use
 *      MSGCOM direct interrupt channels to communicate with each other; so this is
 *      the configuration which is used to create the MSGCOM channel on the NETFP server
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerClient
(
    Netfp_ServerHandle          serverHandle,
    const char*                 clientName,
    Msgcom_DirectInterruptCfg*  ptrDirectInterruptCfg,
    int32_t*                    errCode
)
{
    Name_ResourceCfg        namedResourceCfg;
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 index;
    void*                   criticalSection;
    Josh_NodeCfg            nodeCfg;

    /* Sanity Check: Validate the arguments. */
    if ((serverHandle == NULL) || (clientName == NULL) || (ptrDirectInterruptCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Each Instance should have a unique name; we dont allow duplicate names to exist in the domain. */
    if (Name_findResource (ptrNetfpServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           clientName, &namedResourceCfg, errCode) < 0)
    {
        /* Error: Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
        {
            /* Internal Error: Named resource module has returned an error. We return the error code
             * of the resource manager back to the application. */
            return -1;
        }

        /* Resource does not exist. */
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Critical Section Enter: */
    criticalSection = ptrNetfpServer->cfg.enterCS();

    int sockFd = -1;

    for (index = 0; index < NETFP_MAX_CLIENTS; index++)
    {
        /* Is this client free? */
        if (ptrNetfpServer->clientBlock[index].status != Netfp_ClientStatus_FREE)
            continue;

        /* Client block was free: Create the unique channel names. */
        struct sockaddr_un localAddress;
        memset(&localAddress, 0, sizeof(localAddress));

        localAddress.sun_family = AF_UNIX;
        snprintf (localAddress.sun_path, sizeof(localAddress.sun_path), "/tmp/%s-SRV", clientName);

        sockFd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if(sockFd < 0)
        {
            System_printf("Error: opening unix socket failed [%d]: %s", errno, strerror(errno));
            *errCode = NETFP_EINTERNAL;

            return -1;
        }
        unlink(localAddress.sun_path);
        int bindStatus = bind(sockFd, (const struct sockaddr*)&localAddress, sizeof(localAddress));
        if(bindStatus < 0)
        {
            System_printf("Error: bind unix socket to '%s' failed [%d]: %s", localAddress.sun_path, errno, strerror(errno));
            *errCode = NETFP_EINTERNAL;

            return -1;
        }

        /* The socket needs to be non-blocking to handle ServerTimeout task periodically */
        int fcntlStatus = fcntl(sockFd, F_SETFL, O_NONBLOCK);
        if(fcntlStatus < 0)
        {
            System_printf("Error: fcntl failed [%d]: %s", errno, strerror(errno));
            *errCode = NETFP_EINTERNAL;

            return -1;
        }

        snprintf (ptrNetfpServer->clientBlock[index].clientChannelName, sizeof(ptrNetfpServer->clientBlock[index].clientChannelName), "/tmp/%s-CLNT", clientName);
        strncpy(ptrNetfpServer->clientBlock[index].serverChannelName,
                localAddress.sun_path, sizeof(ptrNetfpServer->clientBlock[index].serverChannelName));

        /* Initialize the node information. */
        memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

        /* Populate the node configuration. */
        nodeCfg.nodeId                     = (uint32_t)sockFd;
        nodeCfg.transportCfg.readerChannel = ptrNetfpServer->clientBlock[index].serverChannelName;
        nodeCfg.transportCfg.writerChannel = ptrNetfpServer->clientBlock[index].clientChannelName;

        /* Populate the transport interface */
        nodeCfg.transportCfg.transport.alloc = Netfp_JoshAlloc_af_unix;
        nodeCfg.transportCfg.transport.free  = Netfp_JoshFree_af_unix;
        nodeCfg.transportCfg.transport.get   = Netfp_JoshGet_af_unix;
        nodeCfg.transportCfg.transport.put   = Netfp_JoshPut_af_unix;

        /* Populate the OSAL table */
        nodeCfg.malloc      = ptrNetfpServer->cfg.malloc;
        nodeCfg.free        = ptrNetfpServer->cfg.free;
        nodeCfg.enterCS     = ptrNetfpServer->cfg.enterCS;
        nodeCfg.exitCS      = ptrNetfpServer->cfg.exitCS;
        nodeCfg.createSem   = ptrNetfpServer->cfg.createSem;
        nodeCfg.deleteSem   = ptrNetfpServer->cfg.deleteSem;
        nodeCfg.postSem     = ptrNetfpServer->cfg.postSem;
        nodeCfg.pendSem     = ptrNetfpServer->cfg.pendSem;

        /* Initialize the JOSH node */
        ptrNetfpServer->clientBlock[index].joshHandle = Josh_initNode (&nodeCfg, errCode);
        if (ptrNetfpServer->clientBlock[index].joshHandle == NULL)
        {
            /* Error: JOSH Initialization failed. */
            *errCode = NETFP_EINTERNAL;
            break;
        }

        /* Populate the NETFP client block. */
        ptrNetfpServer->clientBlock[index].clientHandle = (Netfp_ClientHandle)namedResourceCfg.handle1;
        strncpy (ptrNetfpServer->clientBlock[index].name, clientName, NETFP_MAX_CHAR);
        break;
    }

    /* Critical Section Exit: */
    ptrNetfpServer->cfg.exitCS(criticalSection);

    /* We now need to determine if the client registration was successful or not? */
    if (index == NETFP_MAX_CLIENTS)
    {
        /* Error: There was no space to register the client. */
        *errCode = NETFP_ENOSPACE;
        return -1;
    }

    /* If the register client failed we simply return; the error code is already populated */
    if (ptrNetfpServer->clientBlock[index].joshHandle == NULL)
        return -1;

    /* Register the NETFP services. */
    if (Netfp_registerServices(ptrNetfpServer->clientBlock[index].joshHandle) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the event management services for each NETFP client. */
    if (Netfp_initClientEventMgmt(ptrNetfpServer, index, errCode) < 0)
        return -1;

    /* Modify the named resource to indicate that the client is now active and can be used. */
    namedResourceCfg.handle2 = Netfp_ClientStatus_ACTIVE;
    if (Name_modifyResource(ptrNetfpServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, errCode) < 0)
        return -1;

    /* Mark the client to be active in the server also. */
    ptrNetfpServer->clientBlock[index].status = Netfp_ClientStatus_ACTIVE;

    /* Client has been successfully registered with the server */
    return sockFd;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check if the specific client is still active and registered
 *      with the NETFP Server
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP server
 *  @param[in]  clientName
 *      Name of the client which is being checked
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Active      -   1
 *  @retval
 *      Inactive    -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Netfp_isClientActive
(
    Netfp_ServerHandle  serverHandle,
    const char*         clientName,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 clientBlockIndex;

    /* Sanity Check: Validate the arguments. */
    if ((serverHandle == NULL) || (clientName == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the pointer to the NETFP Server: */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;

    /* Cycle through all the registered clients */
    for (clientBlockIndex = 0; clientBlockIndex < NETFP_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Skip the free client block? */
        if (ptrNetfpServer->clientBlock[clientBlockIndex].status == Netfp_ClientStatus_FREE)
            continue;

        /* Is this client we are looking for? */
        if (strncmp (ptrNetfpServer->clientBlock[clientBlockIndex].name, clientName, NETFP_MAX_CHAR) == 0)
        {
            /* Return the client activity status. */
            if (ptrNetfpServer->clientBlock[clientBlockIndex].status == Netfp_ClientStatus_ACTIVE)
                return 1;
            return 0;
        }
    }

    /* Control comes here implies that the client name passed does not exist with the server and
     * so the client is basically NOT active. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deregister the NETFP client from the server.
 *      The NETFP client needs to be deleted from the NETFP server before
 *      they can be deregistered.
 *
 * @ sa Netfp_deleteClient
 *
 *  @param[in]  netfpServerHandle
 *      Handle to the NETFP Server
 *  @param[in]  clientName
 *      Name of the client which is being deregistered
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deregisterClient
(
    Netfp_ServerHandle      netfpServerHandle,
    const char*             clientName,
    int32_t*                errCode
)
{
    int32_t                 clientBlockIndex;
    Netfp_ServerMCB*        ptrNetfpServer;

    /* Sanity Check: Validate the arguments. */
    if ((netfpServerHandle == NULL) || (clientName == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    /* Cycle through all the registered clients */
    for (clientBlockIndex = 0; clientBlockIndex < NETFP_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Skip the free client block? */
        if (ptrNetfpServer->clientBlock[clientBlockIndex].status == Netfp_ClientStatus_FREE)
            continue;

        /* Is this client we are looking for? */
        if (strncmp (ptrNetfpServer->clientBlock[clientBlockIndex].name, clientName, NETFP_MAX_CHAR) != 0)
            continue;

        /* Ok; we have a name match also. Ensure that the client has been marked inactive before it it being deleted */
        if (ptrNetfpServer->clientBlock[clientBlockIndex].status != Netfp_ClientStatus_INACTIVE)
        {
            /* Client has not been stopped and so it cannot be deregistered from the server. */
            *errCode = NETFP_ENOTREADY;
            return -1;
        }

        /* NETFP client is INACTIVE and it can now be deregistered. */
        break;
    }

    /* Ensure we found a matching client block */
    if (clientBlockIndex == NETFP_MAX_CLIENTS)
    {
        /* Control comes here implies that an invalid client name was specified */
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Delete the JOSH node: JOSH Deinitializations can fail if there are pending jobs
     * This function can only be called if the clients are not using the NETFP services
     * while the deregisteration is being done. This should *NOT* happen since the NETFP
     * clients have already called the *deleteClient* API. */
    if (Josh_deinitNode (ptrNetfpServer->clientBlock[clientBlockIndex].joshHandle, errCode) < 0)
    {
        System_printf ("Error: JOSH Deinitialization for the NETFP client %s failed\n", clientName);
        return -1;
    }

    /* Initialize the event management services for each NETFP client. */
    if (Netfp_deinitClientEventMgmt(ptrNetfpServer, clientBlockIndex, errCode) < 0)
    {
        System_printf ("Error: NETFP Event Management deinit for the NETFP client %s failed\n", clientName);
        return -1;
    }

    /* Reset the NETFP client block. */
    memset ((void*)&ptrNetfpServer->clientBlock[clientBlockIndex], 0, sizeof(Netfp_ClientBlock));

    /* Ensure that the client block is marked as free */
    ptrNetfpServer->clientBlock[clientBlockIndex].status = Netfp_ClientStatus_FREE;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked on the NETFP server by a NETFP client to
 *      indicate to the server that it is no longer operational and should
 *      not be used.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  clientName
 *      Name of the client which is being deleted
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 * @sa Netfp_initClient
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t _Netfp_stopClient
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         clientName,
    int32_t*            errCode
)
{
    int32_t     clientBlockIndex;

    /* Cycle through all the clients to determine which client is being stopped. */
    for (clientBlockIndex = 0; clientBlockIndex < NETFP_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Is this the client? */
        if (strncmp (ptrNetfpServer->clientBlock[clientBlockIndex].name, clientName, NETFP_MAX_CHAR) == 0)
        {
            /* YES. The client is marked appropriately */
            ptrNetfpServer->clientBlock[clientBlockIndex].status = Netfp_ClientStatus_INACTIVE;

            /* Is this the proxy client and if so we need to deregister it also? */
            if (ptrNetfpServer->proxyClientBlock == &ptrNetfpServer->clientBlock[clientBlockIndex])
                ptrNetfpServer->proxyClientBlock = NULL;
            return 0;
        }
    }

    /* Control comes here implies that the NETFP client handle was never registered with the
     * NETFP Server. */
    *errCode = NETFP_EINTERNAL;
    return -1;
}

// <fzm>
int32_t _Netfp_getServerStatus
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Pktlib_HeapStats heapStats;

    Pktlib_getHeapStats(ptrNetfpServer->cfg.ipSecHeapHandle, &heapStats);
    if(heapStats.dataBufferStarvationCounter != 0u)
    {
        System_printf ("%s: ERR IPSecHeap starvation detected: %u", __FUNCTION__, heapStats.dataBufferStarvationCounter);
    }

    if(heapStats.numFreeDataPackets == 0u)
    {
        System_printf ("%s: IPSecHeap numFreePackets below threshold: %u", __FUNCTION__, heapStats.numFreeDataPackets);

        *errCode = NETFP_ENOMEM;
        return -1;
    }

    Pktlib_getHeapStats(ptrNetfpServer->cfg.cmdHeapHandle, &heapStats);
    if(heapStats.numFreeDataPackets < 16)
    {
        System_printf ("%s: cmdHeap numFreePackets below threshold: %u", __FUNCTION__, heapStats.numFreeDataPackets);

        *errCode = NETFP_ENOMEM;
        return -1;
    }

    return 0;
}
// </fzm>

/**
 *  @b Description
 *  @n
 *      This function is used to delete and shutdown the NETFP server. The NETFP
 *      server can only be deleted once all the NETFP clients attached to the server
 *      have been deleted
 *
 *  @param[in]  netfpServerHandle
 *      Handle to the NETFP server to be deleted
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Netfp_deleteServer (Netfp_ServerHandle netfpServerHandle, int32_t* errCode)
{
    Netfp_ServerMCB*        ptrNetfpServer;
    Name_ResourceCfg        namedResourceCfg;
    int32_t                 index;

    /* Sanity Check: Validate the arguments. */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Server Name should exist in the system */
    if (Name_findResource (ptrNetfpServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           ptrNetfpServer->cfg.serverName, &namedResourceCfg, errCode) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Cycle through and delete all the registered NETFP clients. */
    for (index = 0; index < NETFP_MAX_CLIENTS; index++)
    {
        /* Is the client still active? */
        if (ptrNetfpServer->clientBlock[index].status == Netfp_ClientStatus_FREE)
            continue;

        /* Try and deregister the NETFP client */
        if (Netfp_deregisterClient (ptrNetfpServer, ptrNetfpServer->clientBlock[index].name, errCode) < 0)
            return -1;
    }

    /* Cycle through and shutdown all the subcomponents registered with the NETFP server */
    if (Netfp_killInboundFastPath(ptrNetfpServer, errCode) < 0)
        return -1;
    if (Netfp_killOutboundFastPath(ptrNetfpServer, errCode) < 0)
        return -1;
    if (Netfp_killSecurityPolicy(ptrNetfpServer, errCode) < 0)
        return -1;
    if (Netfp_killSecurityAssociation(ptrNetfpServer, errCode) < 0)
        return -1;
    if (Netfp_killSecurityChannel(ptrNetfpServer, errCode) < 0)
        return -1;
    if (Netfp_killInterface(ptrNetfpServer, errCode) < 0)
        return -1;

    /* Release any resources which are opened by the NETFP Servers. */
    for (index = 0; index < QMSS_MAX_PASS_QUEUE; index++)
         Qmss_queueClose (ptrNetfpServer->netcpTxQueue[index]);
    Qmss_queueClose(ptrNetfpServer->paCfgRespQueue);

    /* Close the ciphering channel handle */
    Cppi_channelClose(ptrNetfpServer->cppiCipherTxChHnd);

    /* Close the flows which were opened by the NETFP server.
     *  - PA Response Flow used to receive the PA command responses
     *  - IPSEC Flow Handle which is used to receive data exchanged between the PA & SA. */
    Cppi_closeRxFlow(ptrNetfpServer->paRxFlowHandle);
    Cppi_closeRxFlow(ptrNetfpServer->ipsecFlowHandle);

    /* Close the PA CPDMA */
    Cppi_close (ptrNetfpServer->passCPDMAHandle);

    /* Delete the server name from the named resource database. */
    if (Name_deleteResource (ptrNetfpServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                             ptrNetfpServer->cfg.serverName, errCode) < 0)
        return -1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the NETFP Server to indicate that a timeout has occured
 *
 *  @param[in]  netfpServerHandle
 *      Handle to the NETFP server which is to be executed.
 *  @param[in]  timeoutInSec
 *      Timeout in seconds
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_executeServerTimeout
(
    Netfp_ServerHandle  netfpServerHandle,
    uint32_t            timeoutInSec,
    int32_t*            errCode
)
{
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;
    if (ptrNetfpServer == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Age the PMTU (Secure/Non-Secure) Fast Paths: */
    Netfp_ageFastPathPMTU(ptrNetfpServer, timeoutInSec);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the NETFP Server.
 *
 *  @param[in]  netfpServerHandle
 *      Handle to the NETFP server which is to be executed.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of NETFP clients serviced
 */
int32_t Netfp_executeServer(Netfp_ServerHandle netfpServerHandle)
{
    Netfp_ServerMCB*        ptrNetfpServer;
    uint32_t                index;
    int32_t                 errCode;
    int32_t                 numClientsServiced = 0;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;

    /* Cycle through all the clients and execute JOSH on all the clients */
    for (index = 0; index < NETFP_MAX_CLIENTS; index++)
    {
        /* Do we have an active NETFP client? */
        if (ptrNetfpServer->clientBlock[index].status != Netfp_ClientStatus_ACTIVE)
            continue;

        /* NETFP Client is ACTIVE: Execute the JOSH services */
        if (Josh_receive(ptrNetfpServer->clientBlock[index].joshHandle, &errCode) < 0)
        {
            /* Error: JOSH failure; we could have failed because there was no message which is not
             * an error. So simply ignore that error code but for all other error code we display
             * the error. */
            if (errCode != JOSH_ENOMSG)
                System_printf ("Error: JOSH Receive for Client index %d failed [Error code %d]\n", index, errCode);
        }
        else
        {
            /* Successfully process the JOSH jobs. */
            numClientsServiced++;
        }
    }

    /* Process the events for all the registered clients. */
    numClientsServiced = numClientsServiced + Netfp_processEvents (ptrNetfpServer);

    /* Process any route recomputation */
    numClientsServiced = numClientsServiced + Netfp_processRouteRecomputation(ptrNetfpServer, &errCode);

    return numClientsServiced;
}

/**
 *  @b Description
 *  @n
 *      Utility internal function which returns a string given the client status
 *
 *  @param[in]  clientStatus
 *      NETFP Server Handle
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      String name associated with the client status
 */
static inline const char* Netfp_getClientStatusString (Netfp_ClientStatus clientStatus)
{
    switch (clientStatus)
    {
        case Netfp_ClientStatus_FREE:
        {
            return "Free";
        }
        case Netfp_ClientStatus_INITIALIZED:
        {
            return "Initialized";
        }
        case Netfp_ClientStatus_INACTIVE:
        {
            return "Inactive";
        }
        case Netfp_ClientStatus_ACTIVE:
        {
            return "Active";
        }
        default:
        {
            return "Undefined";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is a utility function which is used to display a
 *      list of all the NETFP clients which have been attached and registered
 *      with the NETFP server
 *
 *  @param[in]  netfpServerHandle
 *      NETFP Server Handle
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of NETFP clients
 */
int32_t Netfp_displayClient(Netfp_ServerHandle netfpServerHandle)
{
    Netfp_ServerMCB*    ptrNetfpServer;
    int32_t             numClients = 0;
    int32_t             clientBlockIndex;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;
    if (ptrNetfpServer == NULL)
        return 0;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "NETFP Client Status\n");

    /* Cycle through all the registered clients */
    for (clientBlockIndex = 0; clientBlockIndex < NETFP_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Is this the client? */
        if (ptrNetfpServer->clientBlock[clientBlockIndex].status != Netfp_ClientStatus_FREE)
        {
            /* Display the banner. */
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "Client Name: %s Status: %s\n", ptrNetfpServer->clientBlock[clientBlockIndex].name,
                          Netfp_getClientStatusString(ptrNetfpServer->clientBlock[clientBlockIndex].status));
            numClients++;
        }
    }
    return numClients;
}

/**
 *  @b Description
 *  @n
 *      The function is a utility function which is used to display the
 *   genernal information on NetFP server.
 *
 *  @param[in]  netfpServerHandle
 *      NETFP Server Handle
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      None
 */
void Netfp_displayServerGenInfo(Netfp_ServerHandle netfpServerHandle)
{
    Netfp_ServerMCB*    ptrNetfpServer;
    Pktlib_HeapStats    heapStats;

    /* Get the NETFP Server MCB */
    ptrNetfpServer = (Netfp_ServerMCB*)netfpServerHandle;
    if (ptrNetfpServer == NULL)
        return ;

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "NETFP Server Info\n");

    /* Display interface based flow and queue */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Server Name: %s\n", ptrNetfpServer->cfg.serverName);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Interface Routing base Queue: %d\n", ptrNetfpServer->interfaceBaseQueue);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Interface Routing base Flow Id: %d\n", ptrNetfpServer->interfaceBaseFlowId);

    /* Get the heap statistics */
    Pktlib_getHeapStats(ptrNetfpServer->cfg.ipSecHeapHandle, &heapStats);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "IPSEC Heap Stats\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Free Data Packets     : %d\n", heapStats.numFreeDataPackets);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Data Buffer Threshold : %d\n", heapStats.dataBufferThresholdStatus);
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Data Buffer Starvation: %d\n", heapStats.dataBufferStarvationCounter);

    return ;
}

