/**
 *   @file  netfp_client.c
 *
 *   @brief
 *      The file implements the NETFP client module
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
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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
 *      This function is used to start the NETFP server. This needs to be invoked
 *      before a NETFP client can be created. NETFP clients can be executing in
 *      either the DSP/ARM execution realms; while the NETFP server (typically)
 *      executes in the ARM realm. DSP NETFP clients would need to cross the realm
 *      boundary to determine if the server is executing. The NAME client handle
 *      is thus the handle which allows the cross to be done.
 *
 *  @param[in]  databaseHandle
 *      Database Handle in which the NETFP server name will be advertised.
 *  @param[in]  clientHandle
 *      Name Client Handle to cross the execution realm boundary if required.
 *  @param[in]  serverName
 *      Name of the server which is to started
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   NETFP Server Handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_ServerHandle Netfp_startServer
(
    Name_DBHandle       databaseHandle,
    Name_ClientHandle   clientHandle,
    const char*         serverName,
    int32_t*            errCode
)
{
    Name_ResourceCfg    namedResourceCfg;
    Netfp_ServerMCB*    ptrNetfpServer;

    /* Sanity Check: Validate the arguments */
    if ((serverName == NULL) || (databaseHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Is there a NAME client handle specified? */
    if (clientHandle == NULL)
    {
        /* NO. We check the local database only. Each Instance should have a unique name; we dont allow duplicate names to exist
         * in the domain. */
        if (Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               serverName, &namedResourceCfg, errCode) < 0)
        {
            /* Error: Server name was NOT found or there was an internal error. */
            if (*errCode != NAME_ENOTFOUND)
            {
                /* Internal Error: Named resource module has returned an error. We return the error code
                 * of the resource manager back to the application. */
                return NULL;
            }

            /* Resource does not exist. */
            *errCode = NETFP_ENOTREADY;
            return NULL;
        }
    }
    else
    {
        /* YES. Check the remote database to determine if the server name has been advertised or not */
        if (Name_get (clientHandle, Name_getDatabaseInstanceId(databaseHandle, errCode),
                      Name_ResourceBucket_INTERNAL_SYSLIB, serverName,
                      &namedResourceCfg, errCode) < 0)
        {
            /* Error: Server name was NOT found or there was an internal error. */
            if (*errCode != NAME_ENOTFOUND)
            {
                /* Internal Error: Named resource module has returned an error. We return the error code
                 * of the resource manager back to the application. */
                return NULL;
            }

            /* Resource does not exist. */
            *errCode = NETFP_ENOTREADY;
            return NULL;
        }
    }

    /* Server name was found. Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)namedResourceCfg.handle1;
    if (ptrNetfpServer == NULL)
    {
        /* Error: Name Server module registered with a NULL handle should never occur. */
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Return the instance handle */
    return (Netfp_ServerHandle)ptrNetfpServer;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the NETFP clients to check if the NETFP
 *      server has been stopped or not. NETFP Clients and servers can execute
 *      in different execution realms and so the NAME client handle needs to
 *      be specified for DSP NETFP Clients to determine the status of the NETFP
 *      server executing on ARM.
 *
 *  @param[in]  databaseHandle
 *      Database handle in which the NETFP server advertises the status
 *  @param[in]  clientHandle
 *      Name Client handle which needs to be specified in the NETFP Client and
 *      Server are executing in different realms.
 *  @param[in]  serverName
 *      Name of the NETFP server which is being checked
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      1       -   Server has been stopped
 *  @retval
 *      0       -   Server has NOT been stopped
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_isServerStopped
(
    Name_DBHandle       databaseHandle,
    Name_ClientHandle   clientHandle,
    const char*         serverName,
    int32_t*            errCode
)
{
    Name_ResourceCfg    namedResourceCfg;

    /* Sanity Check: Validate the arguments */
    if ((serverName == NULL) || (databaseHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Is the NETFP client and server executing on the same realm? */
    if (clientHandle == NULL)
    {
        /* YES. The NETFP server should have deleted the NETFP server name from
         * the database. If the name is not found this indicates that the server has
         * been stopped */
        if (Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               serverName, &namedResourceCfg, errCode) == 0)
        {
            /* NETFP Server name exists in the named resource database so the server
             * has not been stopped. */
            return 0;
        }

        /* Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
            return -1;

        /* NETFP server name is not found so it has been stopped */
        return 1;
    }
    else
    {
        /* NO. The NETFP client and server are executing on different realms and so we need
         * to check in the remote database if the server is operational. */
        if (Name_get (clientHandle, Name_getDatabaseInstanceId(databaseHandle, errCode),
                      Name_ResourceBucket_INTERNAL_SYSLIB, serverName, &namedResourceCfg, errCode) == 0)
        {
            /* NETFP Server name exists in the named remote database so the server has not been stopped. */
            return 0;
        }

        /* Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
            return -1;

        /* NETFP server name is not found so it has been stopped */
        return 1;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETFP client. Clients can only
 *      be initialized once the server is operational.
 *
 *  @param[in]  ptrClientCfg
 *      Pointer to the NETFP client configuration.
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   NETFP Client Handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_ClientHandle Netfp_initClient
(
    Netfp_ClientConfig*     ptrClientCfg,
    int32_t*                errCode
)
{
    Cppi_CpDmaInitCfg       cpdmaCfg;
    Netfp_ClientMCB*        ptrNetfpClient;
    Name_ResourceCfg        namedResourceCfg;
    Netfp_ExecutionRealm    serverRealm;
    uint32_t                index;
    uint8_t                 isAllocated;
    Name_DBHandle           databaseHandle;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];
    Cppi_TxChInitCfg        txChCfg;
    uint32_t                passBaseQueue = QMSS_PASS_QUEUE_BASE;

    /* Sanity Check: Validate the arguments */
    if (ptrClientCfg == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrClientCfg->serverHandle == NULL) || (ptrClientCfg->pktlibInstHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that a valid networking header & fragmentation heap handle are specified. */
    if ((ptrClientCfg->netHeaderHeapHandle == NULL) || (ptrClientCfg->fragmentHeap == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments. */
    if ((ptrClientCfg->clientHeapHandle == NULL) ||
        (ptrClientCfg->msgcomInstHandle == NULL) ||
        (ptrClientCfg->pktlibInstHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check:Validate the OSAL call table */
    if ((ptrClientCfg->malloc           == NULL) || (ptrClientCfg->free            == NULL) ||
        (ptrClientCfg->beginMemAccess   == NULL) || (ptrClientCfg->endMemAccess    == NULL) ||
        (ptrClientCfg->enterCS          == NULL) || (ptrClientCfg->exitCS          == NULL) ||
        (ptrClientCfg->createSem        == NULL) || (ptrClientCfg->deleteSem       == NULL) ||
        (ptrClientCfg->postSem          == NULL) || (ptrClientCfg->pendSem         == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the named resource instance has already been created and exists. */
    databaseHandle = Name_getDatabaseHandle (ptrClientCfg->nrInstanceId);
    if (databaseHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Each client name should be unique in the system */
    Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       ptrClientCfg->clientName, &namedResourceCfg, errCode);

    /* The instance name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the instance name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the server. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Search the local database for the server name */
    if (Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           ptrClientCfg->serverName, &namedResourceCfg, errCode) < 0)
    {
        /* Error: Server name was not found. This could be because of an invalid server name OR the
         * server could be present in the other realm. To cross the other realm we will use the
         * NAME client module if configured. */
        if (ptrClientCfg->nameClientHandle == NULL)
        {
            /* Error: Server name was not found in the local database and we cannot cross the
             * execution realm boundary. */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
        else
        {
            /* Cross the realm boundary and search for the NETFP Server name in the remote database */
            if (Name_get (ptrClientCfg->nameClientHandle, ptrClientCfg->nrInstanceId,
                          Name_ResourceBucket_INTERNAL_SYSLIB, ptrClientCfg->serverName,
                          &namedResourceCfg, errCode) < 0)
            {
                /* Error: Server name was not found in the remote database also. This definately implies
                 * that either the server has not been created (invalid usage) or the server name is invalid */
                *errCode = NETFP_EINVAL;
                return NULL;
            }
            else
            {
                /* Success: Server name was found in the remote database. The server is valid and operational
                 * and can be used. */
                System_printf ("Debug: Server name [%s] found in the remote database\n", ptrClientCfg->serverName);
            }
        }
    }

    /* Get the execution realm of the server. */
    serverRealm = (Netfp_ExecutionRealm)namedResourceCfg.handle2;

    /* Allocate memory for the client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)ptrClientCfg->malloc(sizeof(Netfp_ClientMCB), 0);
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrNetfpClient, 0, sizeof(Netfp_ClientMCB));

    /* Populate the NETFP client MCB */
    memcpy ((void *)&ptrNetfpClient->cfg, (void *)ptrClientCfg, sizeof(Netfp_ClientConfig));

    /* Keep track of the database handle */
    ptrNetfpClient->databaseHandle = databaseHandle;

    /* The client has been initialized but is still not active: */
    ptrNetfpClient->status = Netfp_ClientStatus_INITIALIZED;

    /* Initialize the CPDMA configuration: */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));

    /* Populate the DMA configuration: Dont override the hardware configuration. SOC Init is responsible for the
     * hardware configuration. */
    cpdmaCfg.dmaNum       = Cppi_CpDma_PASS_CPDMA;
    cpdmaCfg.regWriteFlag = Cppi_RegWriteFlag_OFF;

    /* Open the PASS CPDMA for the NETFP Client. This is needed to open the flows. We dont want NETFP
     * clients to communicate with the Server to open a flow since flows are client-specific. */
    ptrNetfpClient->passCPDMAHandle = Cppi_open (&cpdmaCfg);
    if (ptrNetfpClient->passCPDMAHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Open all the NETCP Transmit queues. QMSS RM does not allow opening multiple transmit queues */
    for (index = 0; index < QMSS_MAX_PASS_QUEUE; index++)
        ptrNetfpClient->netcpTxQueue[index] = (Qmss_QueueHnd)(passBaseQueue + index);

    /* Initialize the transmit channel configuration: */
    memset ((void *)&txChCfg, 0, sizeof (Cppi_TxChInitCfg));

    /* Populate the channel configuration: */
    txChCfg.channelNum = NETFP_CPPI_CIPHER_TX_CHANNEL;
    txChCfg.txEnable   = Cppi_ChState_CHANNEL_ENABLE;

    /* Open the ciphering channel. We need access to the channel during LTE reestablishment/Handover. */
    ptrNetfpClient->cppiCipherTxChHnd = Cppi_txChannelOpen (ptrNetfpClient->passCPDMAHandle, &txChCfg, &isAllocated);
    if (ptrNetfpClient->cppiCipherTxChHnd == NULL)
        return NULL;

#if 0
    /* If the server and client are in different realms; we need to push the channel using the agent services */
    if (serverRealm != ptrClientCfg->realm)
    {
        /* YES. Server & clients are on different realms use the agent to push the name across */
        if (Name_push (ptrNetfpClient->cfg.nameClientHandle, channelName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            /* Error code already populated with the agent error code. */
            return NULL;
        }
        System_printf ("Debug: Pushing NETFP Client MSGCOM Channel = '%s' across realms\n", channelName);
    }
    else
    {
        /* NO. Server & client are on the same realm. */
        System_printf ("Debug: NETFP Client and Server on the same realm\n");
    }
#endif

    /* TODO: These are temporary queues which have been opened to handle the results between the F8 and F9
     * operations. These need to be obsoleted. There is one queue for each SRB1 and SRB2 in both the uplink
     * and downlink directions. */
    {
        /* Index 0 is NOT used. */
        ptrNetfpClient->F8ToF9QueueHnd[0] = 0;
        ptrNetfpClient->F9ToF8QueueHnd[0] = 0;

        ptrNetfpClient->F8ToF9QueueHnd[1] = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                           QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        ptrNetfpClient->F8ToF9QueueHnd[2] = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                           QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        ptrNetfpClient->F9ToF8QueueHnd[1] = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                           QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        ptrNetfpClient->F9ToF8QueueHnd[2] = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                           QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

        /* Debug: Display the queues */
        System_printf ("Debug: F8ToF9Queue1 = %x\n", ptrNetfpClient->F8ToF9QueueHnd[1]);
        System_printf ("Debug: F8ToF9Queue2 = %x\n", ptrNetfpClient->F8ToF9QueueHnd[2]);
        System_printf ("Debug: F9ToF8Queue1 = %x\n", ptrNetfpClient->F9ToF8QueueHnd[1]);
        System_printf ("Debug: F9ToF8Queue2 = %x\n", ptrNetfpClient->F9ToF8QueueHnd[2]);
    }

    /* Initialize the named resource configuration. */
    memset ((void *)&namedResourceCfg, 0 , sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    namedResourceCfg.handle1  = (uint32_t)ptrNetfpClient;
    namedResourceCfg.handle2  = (uint32_t)ptrNetfpClient->status;
    namedResourceCfg.handle3  = (uint32_t)ptrClientCfg->realm;
    strncpy(namedResourceCfg.name, ptrNetfpClient->cfg.clientName, NAME_MAX_CHAR);

    /* Create & register the NETFP client instance announcing that the NETFP client has been initialized. */
    if (Name_createResource(databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &namedResourceCfg, errCode) < 0)
    {
        System_printf ("Error: Registering NETFP client '%s' failed [Error code %d]\n", namedResourceCfg.name, *errCode);
        return NULL;
    }

    /* Do we need to push the resource between the realms? */
    if (serverRealm != ptrClientCfg->realm)
    {
        /* YES. Push the NETFP Client MSGCOM channel to the server realm. This will allow the server to perform a
         * find on this channel. */
        if (Name_push (ptrNetfpClient->cfg.nameClientHandle, channelName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            /* Error code already populated with the agent error code. */
            return NULL;
        }
        System_printf ("Debug: Pushing NETFP Client MSGCOM Channel = '%s' across realms\n", channelName);

        /* Push the NETFP client name to the server realm: This will indicate to the server that the client is
         * trying to register itself. */
        if (Name_push (ptrNetfpClient->cfg.nameClientHandle, ptrNetfpClient->cfg.clientName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: NETFP Client '%s' PUSH between realms failed [Error code %d] \n",
                            ptrNetfpClient->cfg.clientName, *errCode);
            return NULL;
        }
        System_printf ("Debug: NETFP Client '%s' pushed successfully between realms\n", ptrNetfpClient->cfg.clientName);
    }

    /* Return the NETFP client handle. */
    return (Netfp_ClientHandle)ptrNetfpClient;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the NETFP client. The NETFP client can only
 *      be started once the NETFP server has acknowledged its initialization.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client which is to be registered
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_FUNCTION
 *
 * @sa Netfp_registerClient
 *
 *  @retval
 *      1   - Client is active
 *  @retval
 *      0   - Client is not active
 *  @retval
 *      <0  - Error
 */
int32_t Netfp_startClient
(
    Netfp_ClientHandle  clientHandle,
    int32_t*            errCode
)
{
    Name_ResourceCfg        namedResourceCfg;
    Netfp_ClientMCB*        ptrNetfpClient;
    Josh_NodeCfg            nodeCfg;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Was the client registered with a name client handle? Do we need to cross the execution realms? */
    if (ptrNetfpClient->cfg.nameClientHandle == NULL)
    {
        /* NO. Check in the local database */
        if (Name_findResource (ptrNetfpClient->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               ptrNetfpClient->cfg.clientName, &namedResourceCfg, errCode) < 0)
        {
            /* Error: Resource was NOT found or there was an internal error. */
            if (*errCode != NAME_ENOTFOUND)
            {
                /* Internal Error: Named resource module has returned an error. We return the error code
                 * of the resource manager back to the application. */
                return -1;
            }

            /* Resource does not exist: This is not possible since the client was already created */
            *errCode = NETFP_EINTERNAL;
            return -1;
        }
        else
        {
            /* Get the status of the client
             *  - After successful registeration the server will mark handle2 to indicate the new status */
            ptrNetfpClient->status = (Netfp_ClientStatus)namedResourceCfg.handle2;
        }
    }
    else
    {
        /* YES. Check in the remote database */
        if (Name_get (ptrNetfpClient->cfg.nameClientHandle, ptrNetfpClient->cfg.nrInstanceId,
                      Name_ResourceBucket_INTERNAL_SYSLIB, ptrNetfpClient->cfg.clientName,
                      &namedResourceCfg, errCode) < 0)
        {
            /* Error: Resource was NOT found in the remote database. This is not possible
             * because once the client was created we had pushed the client name to the
             * other realm */
            *errCode = NETFP_EINTERNAL;
            return -1;
        }
        else
        {
            /* Get the status of the client
             *  - After successful registeration the server will mark handle2 to indicate the new status */
            ptrNetfpClient->status = (Netfp_ClientStatus)namedResourceCfg.handle2;
        }
    }

    /* Is the client active? */
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
        return 0;

    /* Client was active: Determine if we need to activate & start the NETFP Client JOSH services
     * This needs to be done only once. */
	if (ptrNetfpClient->joshHandle != NULL)
		return 1;

    int sockFd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if(sockFd < 0)
    {
        System_printf("Error: opening unix socket failed [%d]: %s", errno, strerror(errno));
        *errCode = NETFP_EINTERNAL;

        return -1;
    }

    struct sockaddr_un localAddress;
    memset(&localAddress, 0, sizeof(localAddress));

    localAddress.sun_family = AF_UNIX;
    snprintf(localAddress.sun_path, sizeof(localAddress.sun_path), "/tmp/%s-CLNT", ptrNetfpClient->cfg.clientName);
    unlink(localAddress.sun_path);

    int bindStatus = bind(sockFd, (const struct sockaddr*)&localAddress, sizeof(localAddress));
    if(bindStatus < 0)
    {
        System_printf("Error: bind unix socket to '%s' failed [%d]: %s", localAddress.sun_path, errno, strerror(errno));
        *errCode = NETFP_EINTERNAL;

        return -1;
    }

    snprintf(ptrNetfpClient->joshConfig.serverChannelName, sizeof(ptrNetfpClient->joshConfig.serverChannelName),
             "/tmp/%s-SRV", ptrNetfpClient->cfg.clientName);
    strncpy(ptrNetfpClient->joshConfig.clientChannelName, localAddress.sun_path, sizeof(ptrNetfpClient->joshConfig.clientChannelName));

    ptrNetfpClient->joshConfig.sockFd = sockFd;

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId                     = (uint32_t)ptrNetfpClient;
    nodeCfg.transportCfg.readerChannel = ptrNetfpClient->joshConfig.clientChannelName;
    nodeCfg.transportCfg.writerChannel = ptrNetfpClient->joshConfig.serverChannelName;

    /* Populate the transport interface */
    nodeCfg.transportCfg.transport.alloc = Netfp_JoshAlloc_af_unix;
    nodeCfg.transportCfg.transport.free  = Netfp_JoshFree_af_unix;
    nodeCfg.transportCfg.transport.get   = Netfp_JoshGet_af_unix_client;
    nodeCfg.transportCfg.transport.put   = Netfp_JoshPut_af_unix_client;

    /* Populate the OSAL table */
    nodeCfg.malloc      = ptrNetfpClient->cfg.malloc;
    nodeCfg.free        = ptrNetfpClient->cfg.free;
    nodeCfg.enterCS     = ptrNetfpClient->cfg.enterCS;
    nodeCfg.exitCS      = ptrNetfpClient->cfg.exitCS;
    nodeCfg.createSem   = ptrNetfpClient->cfg.createSem;
    nodeCfg.deleteSem   = ptrNetfpClient->cfg.deleteSem;
    nodeCfg.postSem     = ptrNetfpClient->cfg.postSem;
    nodeCfg.pendSem     = ptrNetfpClient->cfg.pendSem;

	/* Initialize the JOSH node */
	ptrNetfpClient->joshHandle = Josh_initNode (&nodeCfg, errCode);
	if (ptrNetfpClient->joshHandle == NULL)
	{
		/* Error: JOSH Initialization failed. */
		*errCode = NETFP_EINTERNAL;
		return -1;
	}

	/* Register the NETFP services. */
	if (Netfp_registerServices(ptrNetfpClient->joshHandle) < 0)
	{
		*errCode = NETFP_EINTERNAL;
		return -1;
	}

	/* Client was active so return the correct status. */
	return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the NETFP client. Once the NETFP
 *      client has been stopped it can no longer be used for any other
 *      services. NETFP client need to be stopped before they are
 *      deleted
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client which is to be stopped
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
int32_t Netfp_stopClient
(
    Netfp_ClientHandle  clientHandle,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*        ptrNetfpClient;
    char*                   ptrRemoteClientName;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Mark the client status appropriately. */
    ptrNetfpClient->status = Netfp_ClientStatus_INACTIVE;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_stopClient);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_stopClient
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  char*                   clientName,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = strlen(ptrNetfpClient->cfg.clientName) + 1; // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the client name. */
    ptrRemoteClientName = (char*)args[1].argBuffer;
    strncpy (ptrRemoteClientName, ptrNetfpClient->cfg.clientName, strlen(ptrNetfpClient->cfg.clientName) + 1); // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return result;
}

int32_t Netfp_getServerStatus
(
    Netfp_ClientHandle  clientHandle,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*        ptrNetfpClient;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_getServerStatus);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_getServerStatus
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[1].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete the NETFP client. NETFP clients
 *      need to be stopped before a client can be deleted.
 *
 *  @param[in]  netfpClientHandle
 *      Handle to the NETFP client which is to be deleted.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @sa Netfp_stopClient
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Netfp_deleteClient
(
    Netfp_ClientHandle  netfpClientHandle,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*        ptrNetfpClient;
    Name_ResourceCfg        namedResourceCfg;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)netfpClientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Clients can only be deleted once they have been stopped */
    if (ptrNetfpClient->status != Netfp_ClientStatus_INACTIVE)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* TODO: These are temporary queues which have been opened to handle the results between the F8 and F9
     * operations. Shutdown all these queues. */
    {
        Qmss_queueClose(ptrNetfpClient->F8ToF9QueueHnd[1]);
        Qmss_queueClose(ptrNetfpClient->F8ToF9QueueHnd[2]);
        Qmss_queueClose(ptrNetfpClient->F9ToF8QueueHnd[1]);
        Qmss_queueClose(ptrNetfpClient->F9ToF8QueueHnd[2]);
    }

    /* Close the CPPI cipher channel */
    *errCode = Cppi_channelClose (ptrNetfpClient->cppiCipherTxChHnd);
    if (*errCode < 0)
    {
        System_printf ("Error: CPPI Cipher Channel close failed [Error code %d] \n", *errCode);
        return -1;
    }

    /* Shutdown the JOSH services between the NETFP client and server. */
    if (Josh_deinitNode (ptrNetfpClient->joshHandle, errCode) < 0)
    {
        System_printf ("Error: JOSH Deinitialization for the NETFP client failed [Error code %d] \n", *errCode);
        return -1;
    }

    /* Construct the channel name which is being deleted */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-CLNT", ptrNetfpClient->cfg.clientName);

    /* Delete the NETFP client from the local database. */
    if (Name_deleteResource(ptrNetfpClient->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            ptrNetfpClient->cfg.clientName, errCode) < 0)
    {
        System_printf ("Error: Registering NETFP client '%s' failed [Error code %d]\n", namedResourceCfg.name, *errCode);
        return -1;
    }

    /* Do we need to announce this between execution realms? */
    if (ptrNetfpClient->cfg.nameClientHandle)
    {
        /* YES. The MSGCOM client channel is no longer active and needs to be flushed off from the remote
         * realm database also. The NETFP client is removed from the server database when the NETFP Client
         * is deregistered. */
        if (Name_push (ptrNetfpClient->cfg.nameClientHandle, channelName,
                        Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, errCode) < 0)
        {
            System_printf ("Error: NETFP Client [MSGCOM channel] '%s' PUSH between realms failed [Error code %d] \n",
                            channelName, *errCode);
            return -1;
        }
        System_printf ("Debug: NETFP Client [MSGCOM channel] '%s' pushed successfully between realms\n", channelName);

        /* Push the NETFP client name deletion from the server realm: This will indicate to the server that the
         * client is no longer active. */
        if (Name_push (ptrNetfpClient->cfg.nameClientHandle, ptrNetfpClient->cfg.clientName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, errCode) < 0)
        {
            System_printf ("Error: NETFP Client '%s' deletion push between realms failed [Error code %d] \n",
                            ptrNetfpClient->cfg.clientName, *errCode);
            return -1;
        }
        System_printf ("Debug: NETFP Client '%s' deletion pushed successfully between realms\n", ptrNetfpClient->cfg.clientName);
    }

    /* Close the PASA CPDMA handle. */
    Cppi_close(ptrNetfpClient->passCPDMAHandle);

    /* Free the memory */
    ptrNetfpClient->cfg.free (ptrNetfpClient, sizeof(Netfp_ClientMCB));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the NETFP clients.
 *
 *  @param[in]  netfpClientHandle
 *      Handle to the NETFP client which is to be executed.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_executeClient(Netfp_ClientHandle netfpClientHandle)
{
    Netfp_ClientMCB*    ptrNetfpClient;
    int32_t             errCode;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)netfpClientHandle;

    /* Execute the JOSH services */
    if (Josh_receive(ptrNetfpClient->joshHandle, &errCode) < 0)
    {
        /* Error: JOSH failure; we could have failed because there was no message which is not
         * an error. So simply ignore that error code but for all other error code we display
         * the error. */
        if (errCode != JOSH_ENOMSG)
            System_printf ("Error: JOSH Receive for Client %p failed [Error code %d]\n", ptrNetfpClient, errCode);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This function is used to register the software SNOW3G provided services
 *      with the specific NETFP client. Each NETFP client which uses the SNOW3G
 *      for SRB is required to do so.
 *
 *  @param[in]  netfpClientHandle
 *      Handle to the NETFP client which is to be executed.
 *  @param[in]  f8
 *      F8 SNOW3G function required for SRB ciphering
 *  @param[in]  f9
 *      F9 SNOW3G function required for SRB authentication
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
int32_t Netfp_register3GPPSnow3GServices
(
    Netfp_ClientHandle  netfpClientHandle,
    Netfp_F8Function    f8,
    Netfp_F9Function    f9,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*    ptrNetfpClient;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)netfpClientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the SNOW3G f8 and f9 API */
    if ((f8 == NULL) || (f9 == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Remember the services: */
    ptrNetfpClient->f8 = f8;
    ptrNetfpClient->f9 = f9;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to register the NETFP hook
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client
 *  @param[in]  ptrHookCfg
 *      Pointer to the hook configuration
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
int32_t Netfp_registerHook
(
    Netfp_ClientHandle  clientHandle,
    Netfp_HookCfg*      ptrHookCfg,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*    ptrNetfpClient;
    Netfp_Socket*       ptrNetfpSocket;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if ((ptrNetfpClient == NULL) || (ptrHookCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that a hook function is specified. */
    if (ptrHookCfg->hookFxn == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the rest of the arguments. */
    switch (ptrHookCfg->hook)
    {
        case Netfp_Hook_PRE_OUTER_REASSEMBLY:
        case Netfp_Hook_PRE_INNER_REASSEMBLY:
        case Netfp_Hook_POST_OUTER_REASSEMBLY:
        case Netfp_Hook_POST_INNER_REASSEMBLY:
        {
            /* Reassembly Hook: Validate the remaining arguments */
            if (ptrNetfpClient->ptrReassemblyMCB == NULL)
            {
                /* Error: Trying to register a hook for a client which has not been registered
                 * for reassembly. */
                *errCode = NETFP_ENOTREADY;
                return -1;
            }

            /* Sanity Check: Reassembly Hooks are for the entire system and not on a socket basis. */
            if (ptrHookCfg->sockHandle != NULL)
            {
                /* Error: Invalid argument */
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Duplicate hooks are not supported. */
            if (ptrNetfpClient->reassemblyHook[ptrHookCfg->hook] != NULL)
            {
                /* Error: Duplicate hook detected. */
                *errCode = NETFP_EINUSE;
                return -1;
            }

            /* Populate the Post Reassembly Hook information: */
            ptrNetfpClient->reassemblyHook[ptrHookCfg->hook]    = ptrHookCfg->hookFxn;
            ptrNetfpClient->reassemblyHookArg[ptrHookCfg->hook] = ptrHookCfg->hookArg;
            break;
        }
        case Netfp_Hook_POST_ROUTING:
        {
            /* Post Routing Hook: Validate the remaining arguments */
            ptrNetfpSocket = (Netfp_Socket*)ptrHookCfg->sockHandle;
            if (ptrNetfpSocket == NULL)
            {
                /* Post Routing is implemented on a per socket basis */
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Sanity Check: Duplicate hooks are not supported. */
            if (ptrNetfpSocket->postRoutingHook != NULL)
            {
                /* Error: Duplicate hook detected. */
                *errCode = NETFP_EINUSE;
                return -1;
            }

            /* Populate the Post Routing Hook information: */
            ptrNetfpSocket->postRoutingHook    = ptrHookCfg->hookFxn;
            ptrNetfpSocket->postRoutingHookArg = ptrHookCfg->hookArg;
            break;
        }
        default:
        {
            /* Error: Unsupported hook */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to unregister a previously registered NETFP hook
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client
 *  @param[in]  hook
 *      Hook to be unregistered
 *  @param[in]  sockHandle
 *      Socket Handle. This is required only if unregistering the socket POST_ROUTING
 *      hook. POST/PRE REASSEMBLY hooks shiuld pass this field as NULL.
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
int32_t Netfp_unregisterHook
(
    Netfp_ClientHandle  clientHandle,
    Netfp_Hook          hook,
    Netfp_SockHandle    sockHandle,
    int32_t*            errCode
)
{
    Netfp_ClientMCB*    ptrNetfpClient;
    Netfp_Socket*       ptrNetfpSocket;

    /* Get the NETFP Client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the rest of the arguments. */
    switch (hook)
    {
        case Netfp_Hook_PRE_OUTER_REASSEMBLY:
        case Netfp_Hook_PRE_INNER_REASSEMBLY:
        case Netfp_Hook_POST_OUTER_REASSEMBLY:
        case Netfp_Hook_POST_INNER_REASSEMBLY:
        {
            /* Unregister the hook: */
            ptrNetfpClient->reassemblyHook[hook]    = NULL;
            ptrNetfpClient->reassemblyHookArg[hook] = 0;
            break;
        }
        case Netfp_Hook_POST_ROUTING:
        {
            /* Post Routing Hook: Validate the remaining arguments */
            ptrNetfpSocket = (Netfp_Socket*)sockHandle;
            if (ptrNetfpSocket == NULL)
            {
                /* POST_ROUTING is implemented on a per socket basis */
                *errCode = NETFP_EINVAL;
                return -1;
            }

            /* Unregister the hook: */
            ptrNetfpSocket->postRoutingHook    = NULL;
            ptrNetfpSocket->postRoutingHookArg = 0;
            break;
        }
        default:
        {
            /* Error: Unsupported hook */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

//<fzm>

int32_t Netfp_registerSaEventHooks
(
        Netfp_ClientHandle   clientHandle,
        Netfp_SaEventHandler addSaEventHandler,
        Netfp_SaEventHandler delSaEventHandler,
        int32_t*             errCode
)
{
    Netfp_ClientMCB* ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    ptrNetfpClient->addSaEventHandler = addSaEventHandler;
    ptrNetfpClient->delSaEventHandler = delSaEventHandler;

    return 0;
}

//</fzm>
