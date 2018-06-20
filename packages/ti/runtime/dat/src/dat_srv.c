/**
 *   @file  dat_srv.c
 *
 *   @brief
 *      The file implements the DAT server functionality
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/dat/include/dat_internal.h>

/* UIA include files */
#include <ti/uia/runtime/UIAPacket.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************ DAT Server Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Registered callback function which is invoked to cleanup any pending
 *      packets in the MSGCOM channel when the channels are being deleted
 *
 *  @param[in]   pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]   chHandle
 *      Channel handle being deleted
 *  @param[in]   msgBuffer
 *      Message which is pending and needs to be cleaned up
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 dis*      Not applicable
 */
static void Dat_freePkt(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    Pktlib_freePacket (pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to allocate a message which is to be sent across
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be allocated
 *  @param[out]  msgBuffer
 *      Opaque handle to the message to be sent out
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
       Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Dat_serverJoshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Ti_Pkt*         ptrPkt;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferLen;
    Dat_ServerMCB*  ptrDatServer;

    /* Get the DAT Server information block. */
    ptrDatServer = (Dat_ServerMCB*)nodeId;

    /* Allocate a packet from the agent server heap */
    ptrPkt = Pktlib_allocPacket(ptrDatServer->cfg.pktlibInstHandle, ptrDatServer->cfg.serverHeapHandle, size);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: JOSH Packet allocation from heap %p [%d bytes] failed\n",
                       ptrDatServer->cfg.serverHeapHandle, size);
        return NULL;
    }

    /* Get the data buffer. */
    Pktlib_getDataBuffer(ptrPkt, &ptrDataBuffer, &dataBufferLen);

    /* Populate the memory handle with the pointer to the PKTLIB Packet. */
    *msgBuffer = (MsgCom_Buffer*)ptrPkt;

    /* Return the data buffer. */
    return ptrDataBuffer;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to free a previously allocated message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be freed up
 *  @param[in]  msgBuffer
 *      Opaque handle to the message to be cleaned up
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static void Dat_serverJoshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Ti_Pkt*         ptrPkt;
    Dat_ServerMCB*  ptrDatServer;

    /* Get the DAT Server information block. */
    ptrDatServer = (Dat_ServerMCB*)nodeId;

    /* Get the packet buffer. */
    ptrPkt = (Ti_Pkt*)msgBuffer;
    if (ptrPkt == NULL)
        return;

    /* Cleanup the packet. */
    Pktlib_freePacket(ptrDatServer->cfg.pktlibInstHandle, ptrPkt);
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to receive a message.
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   readerChannel
 *      Opaque reader channel on which the message is to be received
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has been received
 *  @param[in]  ptrDataBuffer
 *      Data buffer which points to the received message
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Dat_serverJoshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    uint32_t        dataBufferLen;
    Dat_ServerMCB*  ptrDatServer;

    /* Get the DAT Server. */
    ptrDatServer = (Dat_ServerMCB*)nodeId;

    /* Read the message from the specified reader channel. */
    Msgcom_getMessage((MsgCom_ChHandle)readerChannel, (MsgCom_Buffer**)msgBuffer);
    if (*msgBuffer == NULL)
    {
        /* No message was available. */
        *ptrDataBuffer = NULL;
        return;
    }

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)(*msgBuffer), ptrDataBuffer, &dataBufferLen);

    /* Invalidate the data buffer. */
    ptrDatServer->cfg.beginMemAccess (*ptrDataBuffer, dataBufferLen);
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to send a message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   writerChannel
 *      Opaque writer channel on which the message is to be transmitted
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has to be sent
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Dat_serverJoshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    uint32_t        dataBufferLen;
    uint8_t*        ptrDataBuffer;
    Dat_ServerMCB*  ptrDatServer;

    /* Get the DAT Server. */
    ptrDatServer = (Dat_ServerMCB*)nodeId;

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)msgBuffer, &ptrDataBuffer, &dataBufferLen);

    /* Writeback the data buffer. */
    ptrDatServer->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /* Send the message */
    Msgcom_putMessage((MsgCom_ChHandle)writerChannel, (MsgCom_Buffer*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the DAT server.
 *
 *  @param[in]  ptrServerCfg
 *      Pointer to the DAT server configuration.
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   DAT Server Handle
 *  @retval
 *      Error   -   NULL
 */
Dat_ServerHandle Dat_initServer
(
    Dat_ServerCfg*  ptrServerCfg,
    int32_t*        errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Name_ResourceCfg        namedResourceCfg;
    Name_DBHandle           databaseHandle;

    /* Sanity Check: Validate the arguments. */
    if (ptrServerCfg == NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments. */
    if ((ptrServerCfg->serverHeapHandle == NULL)    ||
        (ptrServerCfg->msgcomInstHandle == NULL)    ||
        (ptrServerCfg->pktlibInstHandle == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check:Validate the OSAL call table */
    if ((ptrServerCfg->malloc         == NULL) || (ptrServerCfg->free         == NULL) ||
        (ptrServerCfg->enterCS        == NULL) || (ptrServerCfg->exitCS       == NULL) ||
        (ptrServerCfg->beginMemAccess == NULL) || (ptrServerCfg->endMemAccess == NULL) ||
        (ptrServerCfg->createSem      == NULL) || (ptrServerCfg->deleteSem    == NULL) ||
        (ptrServerCfg->postSem        == NULL) || (ptrServerCfg->pendSem      == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the named resource instance is valid */
    databaseHandle = Name_getDatabaseHandle (ptrServerCfg->nrInstanceId);
    if (databaseHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Each server name should be unique in the system */
    Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, ptrServerCfg->serverName,
                       &namedResourceCfg, errCode);

    /* The instance name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the instance name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the server. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Allocate memory for the DAT Server: */
    ptrDatServer = (Dat_ServerMCB*)ptrServerCfg->malloc(sizeof(Dat_ServerMCB), 0);
    if (ptrDatServer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrDatServer, 0, sizeof(Dat_ServerMCB));

    /* Populate the configuration block. */
    memcpy ((void *)&ptrDatServer->cfg, (void *)ptrServerCfg, sizeof(Dat_ServerCfg));

    /* Keep track of the named resource instance handle */
    ptrDatServer->databaseHandle = databaseHandle;

    /* Populate the named resource configuration. */
    namedResourceCfg.handle1  = (uint32_t)ptrDatServer;
    namedResourceCfg.handle2  = (uint32_t)ptrDatServer->cfg.realm;
    strncpy(namedResourceCfg.name, ptrServerCfg->serverName, DAT_MAX_CHAR - 1);

    /* Create & register the DAT server instance announcing that the DAT server is operational. */
    if (Name_createResource(ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, errCode) < 0)
    {
        System_printf ("Error: Registering DAT server '%s' failed [Error code %d]\n", namedResourceCfg.name, *errCode);
        return NULL;
    }

    return (Dat_ServerHandle)ptrDatServer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the DAT client with the server. The DAT
 *      client is operational only once it has been initialized and then registered
 *      with the DAT server.
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT server
 *  @param[in]  clientName
 *      Name of the client which is being registered
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_registerClient
(
    Dat_ServerHandle            serverHandle,
    char*                       clientName,
    int32_t*                    errCode
)
{
    Name_ResourceCfg        namedResourceCfg;
    Dat_ServerMCB*          ptrDatServer;
    int32_t                 index;
    void*                   criticalSection;
    char                    serverChannelName[MSG_COM_MAX_CHANNEL_LEN];
    char                    clientChannelName[MSG_COM_MAX_CHANNEL_LEN];
    Msgcom_ChannelCfg       chConfig;
    Josh_NodeCfg            nodeCfg;

    /* Sanity Check: Validate the arguments. */
    if ((serverHandle == NULL) || (clientName == NULL))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT Server: */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Each Instance should have a unique name; we dont allow duplicate names to exist in the domain. */
    if (Name_findResource (ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
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
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Critical Section Enter: */
    criticalSection = ptrDatServer->cfg.enterCS();

    /* Cycle through all the DAT clients to determine if the client can be registered or not? */
    for (index = 0; index < DAT_MAX_CLIENTS; index++)
    {
        /* Is this client free? */
        if (ptrDatServer->clientBlock[index].status != Dat_ClientStatus_FREE)
            continue;

        /* Client block was free: Create the unique channel names. */
        snprintf (serverChannelName, MSG_COM_MAX_CHANNEL_LEN, "%s-SRV",  clientName);
        snprintf (clientChannelName, MSG_COM_MAX_CHANNEL_LEN, "%s-CLNT", clientName);

        /* Initialize the channel configuration: */
        memset ((void*)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

        /* Populate the channel configuration:
         * - Server channels are created as NON BLOCKING channels; since the DAT server always
         *   executes in polling mode. There is a crunch on the number of resources and since
         *   DAT services are control path we dont need to take up interrupt resources. */
        chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
        chConfig.msgcomInstHandle            = ptrDatServer->cfg.msgcomInstHandle;
        chConfig.appCallBack                 = NULL;
        chConfig.u.queueDMACfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;
        chConfig.u.queueDMACfg.rxFreeQueueNum= (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrDatServer->cfg.serverHeapHandle)));

        /* Open the MSGCOM Queue-DMA server channel handle. */
        ptrDatServer->clientBlock[index].serverChannel = Msgcom_create (serverChannelName, Msgcom_ChannelType_QUEUE_DMA,
                                                                        &chConfig, errCode);
        if (ptrDatServer->clientBlock[index].serverChannel == NULL)
            break;

        /* Find the client channel which is used to JOSH messages to the client. */
        ptrDatServer->clientBlock[index].clientChannel = Msgcom_find(ptrDatServer->cfg.msgcomInstHandle, clientChannelName, errCode);
        if (ptrDatServer->clientBlock[index].clientChannel == NULL)
        {
            /* Error: The client was registered yet its channel name does not exist. */
            *errCode = DAT_EINTERNAL;
            break;
        }

        /* Initialize the node information. */
        memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

        /* Populate the node configuration. */
        nodeCfg.nodeId                     = (uint32_t)ptrDatServer;
        nodeCfg.transportCfg.readerChannel = ptrDatServer->clientBlock[index].serverChannel;
        nodeCfg.transportCfg.writerChannel = ptrDatServer->clientBlock[index].clientChannel;

        /* Populate the transport interface */
        nodeCfg.transportCfg.transport.alloc = Dat_serverJoshAlloc;
        nodeCfg.transportCfg.transport.free  = Dat_serverJoshFree;
        nodeCfg.transportCfg.transport.get   = Dat_serverJoshGet;
        nodeCfg.transportCfg.transport.put   = Dat_serverJoshPut;

        /* Populate the OSAL table */
        nodeCfg.malloc      = ptrDatServer->cfg.malloc;
        nodeCfg.free        = ptrDatServer->cfg.free;
        nodeCfg.enterCS     = ptrDatServer->cfg.enterCS;
        nodeCfg.exitCS      = ptrDatServer->cfg.exitCS;
        nodeCfg.createSem   = ptrDatServer->cfg.createSem;
        nodeCfg.deleteSem   = ptrDatServer->cfg.deleteSem;
        nodeCfg.postSem     = ptrDatServer->cfg.postSem;
        nodeCfg.pendSem     = ptrDatServer->cfg.pendSem;

        /* Initialize the JOSH node */
        ptrDatServer->clientBlock[index].joshHandle = Josh_initNode (&nodeCfg, errCode);
        if (ptrDatServer->clientBlock[index].joshHandle == NULL)
        {
            /* Error: JOSH Initialization failed. */
            *errCode = DAT_EINTERNAL;
            break;
        }

        /* Populate the DAT client block. */
        ptrDatServer->clientBlock[index].clientHandle = (Dat_ClientHandle)namedResourceCfg.handle1;
        strncpy (ptrDatServer->clientBlock[index].name, clientName, DAT_MAX_CHAR - 1);
        break;
    }

    /* Critical Section Exit: */
    ptrDatServer->cfg.exitCS(criticalSection);

    /* We now need to determine if the client registration was successful or not? */
    if (index == DAT_MAX_CLIENTS)
    {
        /* Error: There was no space to register the client. */
        *errCode = DAT_ENOSPACE;
        return -1;
    }

    /* If the register client failed we simply return; the error code is already populated */
    if (ptrDatServer->clientBlock[index].joshHandle == NULL)
        return -1;

    /* Register the DAT services. */
    Dat_registerServices(ptrDatServer->clientBlock[index].joshHandle);

    /* Modify the named resource to indicate that the client is now active and can be used. */
    namedResourceCfg.handle3 = Dat_ClientStatus_ACTIVE;
    if (Name_modifyResource(ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, errCode) < 0)
        return -1;

    /* Mark the client to be active in the server also. */
    ptrDatServer->clientBlock[index].status = Dat_ClientStatus_ACTIVE;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check if the specific client is still active and registered
 *      with the DAT Server
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT server
 *  @param[in]  clientName
 *      Name of the client which is being checked
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Active      -   1
 *  @retval
 *      Inactive    -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_isClientActive
(
    Dat_ServerHandle    serverHandle,
    char*               clientName,
    int32_t*            errCode
)
{
    Dat_ServerMCB*  ptrDatServer;
    int32_t         clientBlockIndex;

    /* Sanity Check: Validate the arguments. */
    if ((serverHandle == NULL) || (clientName == NULL))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT Server: */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Cycle through all the registered clients */
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Skip the free client block? */
        if (ptrDatServer->clientBlock[clientBlockIndex].status == Dat_ClientStatus_FREE)
            continue;

        /* Is this client we are looking for? */
        if (strncmp (ptrDatServer->clientBlock[clientBlockIndex].name, clientName, DAT_MAX_CHAR) == 0)
        {
            /* Return the client activity status. */
            if (ptrDatServer->clientBlock[clientBlockIndex].status == Dat_ClientStatus_ACTIVE)
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
 *      The function is invoked on the DAT server by a DAT client to
 *      indicate to the server that it is no longer operational and should
 *      not be used.
 *
 *  @param[in]  ptrDatServer
 *      Pointer to the DAT server
 *  @param[in]  clientName
 *      Name of the client which is being stopped
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_INTERNAL_FUNCTION
 *
 * @sa Netfp_initClient
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t _Dat_stopClient
(
    Dat_ServerMCB*      ptrDatServer,
    char*               clientName,
    int32_t*            errCode
)
{
    int32_t     clientBlockIndex;

    /* Cycle through all the clients to determine which client is being stopped. */
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Is this the client? */
        if (strncmp (ptrDatServer->clientBlock[clientBlockIndex].name, clientName, DAT_MAX_CHAR) == 0)
        {
            /* YES. The client is marked appropriately */
            ptrDatServer->clientBlock[clientBlockIndex].status = Dat_ClientStatus_INACTIVE;
            return 0;
        }
    }

    /* Control comes here implies that the DAT client handle was never registered with the DAT Server. */
    *errCode = DAT_EINTERNAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deregister the DAT client from the server.
 *      The DAT client needs to be deleted from the DAT server before
 *      they can be deregistered.
 *
 * @ sa Dat_deleteClient
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT Server
 *  @param[in]  clientName
 *      Name of the client which is being deregistered
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deregisterClient
(
    Dat_ServerHandle    serverHandle,
    char*               clientName,
    int32_t*            errCode
)
{
    int32_t                 clientBlockIndex;
    Dat_ServerMCB*          ptrDatServer;

    /* Sanity Check: Validate the arguments. */
    if ((serverHandle == NULL) || (clientName == NULL))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Cycle through all the registered clients */
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Skip the free client block? */
        if (ptrDatServer->clientBlock[clientBlockIndex].status == Dat_ClientStatus_FREE)
            continue;

        /* Is this client we are looking for? */
        if (strncmp (ptrDatServer->clientBlock[clientBlockIndex].name, clientName, DAT_MAX_CHAR) != 0)
            continue;

        /* Ok; we have a name match also. Ensure that the client has been marked inactive before it it being deleted */
        if (ptrDatServer->clientBlock[clientBlockIndex].status != Dat_ClientStatus_INACTIVE)
        {
            /* Client has not been stopped and so it cannot be deregistered from the server. */
            *errCode = DAT_ENOTREADY;
            return -1;
        }

        /* DAT client is INACTIVE and it can now be deregistered. */
        break;
    }

    /* Ensure we found a matching client block */
    if (clientBlockIndex == DAT_MAX_CLIENTS)
    {
        /* Control comes here implies that an invalid client name was specified */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Delete the JOSH node: JOSH Deinitializations can fail if there are pending jobs
     * This function can only be called if the clients are not using the DAT services
     * while the deregisteration is being done. This should *NOT* happen since the DAT
     * clients have already called the *deleteClient* API. */
    if (Josh_deinitNode (ptrDatServer->clientBlock[clientBlockIndex].joshHandle, errCode) < 0)
    {
        System_printf ("Error: JOSH Deinitialization for the DAT client %s failed\n", clientName);
        return -1;
    }

    /* Delete the MSGCOM reader & writer channels  */
    *errCode = Msgcom_delete (ptrDatServer->clientBlock[clientBlockIndex].serverChannel, Dat_freePkt);
    if (*errCode < 0)
        return -1;
    *errCode = Msgcom_delete (ptrDatServer->clientBlock[clientBlockIndex].clientChannel, Dat_freePkt);
    if (*errCode < 0)
        return -1;

    System_printf ("Debug: DAT Client MSGCOM channels have been shutdown\n");

    /* Delete the DAT Client name from the server database. */
    if (Name_deleteResource(ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            ptrDatServer->clientBlock[clientBlockIndex].name, errCode) < 0)
    {
        System_printf ("Error: Deleting DAT Client '%s' failed [Error code %d]\n", ptrDatServer->clientBlock[clientBlockIndex].name, *errCode);
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: DAT Client '%s' has been removed from the server database\n", ptrDatServer->clientBlock[clientBlockIndex].name);

    /* Reset the DAT client block. */
    memset ((void*)&ptrDatServer->clientBlock[clientBlockIndex], 0, sizeof(Dat_ClientBlock));

    /* Ensure that the client block is marked as free */
    ptrDatServer->clientBlock[clientBlockIndex].status = Dat_ClientStatus_FREE;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete and shutdown the DAT server. The DAT
 *      server can only be deleted once all the DAT clients attached to the server
 *      have been deleted
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT server to be deleted
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Dat_deleteServer
(
    Dat_ServerHandle    serverHandle,
    int32_t*            errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Name_ResourceCfg        namedResourceCfg;
    int32_t                 index;

    /* Sanity Check: Validate the arguments. */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;
    if (ptrDatServer == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: Server Name should exist in the system */
    if (Name_findResource (ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           ptrDatServer->cfg.serverName, &namedResourceCfg, errCode) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Cycle through and delete all the registered DAT clients. */
    for (index = 0; index < DAT_MAX_CLIENTS; index++)
    {
        /* Is the client still active? */
        if (ptrDatServer->clientBlock[index].status == Dat_ClientStatus_FREE)
            continue;

        /* Try and deregister the DAT client */
        if (Dat_deregisterClient (ptrDatServer, ptrDatServer->clientBlock[index].name, errCode) < 0)
            return -1;
    }

    /* Delete the server name from the named resource database. */
    if (Name_deleteResource (ptrDatServer->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                             ptrDatServer->cfg.serverName, errCode) < 0)
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the DAT Server.
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT server which is to be executed.
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Number of NETFP clients serviced
 */
int32_t Dat_executeServer(Dat_ServerHandle serverHandle)
{
    Dat_ServerMCB*          ptrDatServer;
    int32_t                 clientBlockIndex;
    int32_t                 errCode;
    int32_t                 retVal;
    uint32_t                jobResult;
    Dat_ServerConsumer*     ptrServerConsumer;
    Dat_ServerProducer*     ptrServerProducer;
    int32_t                 numClientsServiced = 0;
    Dat_verbosityOpBlock*   ptrVerbosityOpBlk;
    Dat_verbosityOpBlock*   ptrPrevVerbosityOpBlk;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;
    if (ptrDatServer == NULL)
        return 0;

    /*******************************************************************************************
     * Pending verbosity operation list
     *******************************************************************************************/
    ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)Dat_listGetHead((Dat_ListNode**)&ptrDatServer->ptrPendingVerbosityOpList);
    while (ptrVerbosityOpBlk != NULL)
    {
        Dat_ServerLocalTraceObject* ptrLocalTraceObj;

        /* get the local trace object */
        ptrLocalTraceObj = ptrVerbosityOpBlk->ptrTraceObj;
        if(ptrLocalTraceObj == NULL)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: invalid trace object handle for job:%p, josh handle:%p\n",
                        ptrVerbosityOpBlk->jobId, ptrVerbosityOpBlk->joshNodeHandle);
            return numClientsServiced;
        }
        /* Is the job complete? */
        retVal = Josh_isJobCompleted (ptrVerbosityOpBlk->joshNodeHandle,
                                      ptrVerbosityOpBlk->jobId, &jobResult, &errCode);
        if (retVal < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: DAT Josh async completion job id %d [Error code %d]\n",
                        ptrVerbosityOpBlk->jobId, errCode);
            return numClientsServiced;
        }
        if (retVal == 0)
        {
            /* Job was NOT completed. */
            ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)Dat_listGetNext ((Dat_ListNode*)&ptrVerbosityOpBlk->opLinks);
            continue;
        }

        /* JOB was complete. Cleanup the job. */
        if ( Josh_freeJobInstance(ptrVerbosityOpBlk->joshNodeHandle, ptrVerbosityOpBlk->jobId) < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: Free Josh job failed!\n");
            continue;
        }

        /* The JOB has been processed successfully */
        ptrVerbosityOpBlk->jobId = -1;

        /* Remove the job from the pending connect list  */
        retVal = Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrPendingVerbosityOpList, (Dat_ListNode*)&ptrVerbosityOpBlk->opLinks);

        ptrPrevVerbosityOpBlk = ptrVerbosityOpBlk;

        /* Get the next pending request if any. */
        ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)Dat_listGetNext((Dat_ListNode*)&ptrVerbosityOpBlk->opLinks);

        /* Cleanup allocated memory */
        ptrDatServer->cfg.free (ptrPrevVerbosityOpBlk, sizeof(Dat_verbosityOpBlock));
    }

    /*******************************************************************************************
     * Josh receive
     *******************************************************************************************/

    /* Cycle through all the DAT clients. */
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Do we have an active DAT client? */
        if (ptrDatServer->clientBlock[clientBlockIndex].status != Dat_ClientStatus_ACTIVE)
            continue ;

        /* Execute the JOSH services on the DAT client */
        if (Josh_receive (ptrDatServer->clientBlock[clientBlockIndex].joshHandle, &errCode) < 0)
        {
            /* JOSH Service execution failed: Use the error code to determine the reason for failure */
            if (errCode != JOSH_ENOMSG)
            {
                /* Notify the application about the FATAL error. */
                Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: DAT Josh receive failed for client %d [Error code %d]\n",
                            clientBlockIndex, errCode);
            }
        }
        else
        {
            /* Successfully process the JOSH jobs. */
            numClientsServiced++;
        }
    }

    /*******************************************************************************************
     * Pending producer operation list
     *******************************************************************************************/
    ptrServerProducer = (Dat_ServerProducer*)Dat_listGetHead((Dat_ListNode**)&ptrDatServer->ptrPendingProducerOpList);
    while (ptrServerProducer != NULL)
    {

        /* Is the job complete? */
        retVal = Josh_isJobCompleted (ptrServerProducer->joshNodeHandle,
                                      ptrServerProducer->jobId, &jobResult, &errCode);
        if (retVal < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: DAT Josh async completion job id %d [Error code %d]\n",
                        ptrServerProducer->jobId, errCode);
            return numClientsServiced;
        }
        if (retVal == 0)
        {
            /* Job was NOT completed. */
            ptrServerProducer = (Dat_ServerProducer*)Dat_listGetNext ((Dat_ListNode*)ptrServerProducer);
            continue;
        }

        /* JOB was complete. Cleanup the job. */
        if (Josh_freeJobInstance(ptrServerProducer->joshNodeHandle, ptrServerProducer->jobId) < 0 )
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: Free Josh job failed!\n");
            continue;
        }

        /* The JOB has been processed successfully */
        ptrServerProducer->jobId = -1;

        /* Remove the job from the pending connect list  */
        Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrPendingProducerOpList, (Dat_ListNode*)ptrServerProducer);

        /* Get the next pending request if any. */
        ptrServerProducer = (Dat_ServerProducer*)Dat_listGetNext((Dat_ListNode*)ptrServerProducer);
    }

    /*******************************************************************************************
     * Pending connect list
     *******************************************************************************************/
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Is the job complete? */
        retVal = Josh_isJobCompleted (ptrServerConsumer->ptrServerProducer->joshNodeHandle,
                                      ptrServerConsumer->jobId, &jobResult, &errCode);
        if (retVal < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: DAT Josh async completion job id %d [Error code %d]\n",
                        ptrServerConsumer->jobId, errCode);
            return numClientsServiced;
        }
        if (retVal == 0)
        {
            /* Job was NOT completed. */
            ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
            continue;
        }

        /* JOB was complete. Cleanup the job. */
        if (Josh_freeJobInstance(ptrServerConsumer->ptrServerProducer->joshNodeHandle, ptrServerConsumer->jobId) < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: Free Josh job failed!\n");
            continue;
        }

        /* The JOB has been processed successfully */
        ptrServerConsumer->jobId = -1;

        /* Remove the job from the pending connect list  */
        Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList, (Dat_ListNode*)ptrServerConsumer);

        /* Move the consumer to the list of consumers attached to the producer */
        Dat_listAdd ((Dat_ListNode**)&ptrServerConsumer->ptrServerProducer->ptrConsumerList, (Dat_ListNode*)ptrServerConsumer);

        /* Get the next pending request if any. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList);

        numClientsServiced++;
    }

    /*******************************************************************************************
     * Pending disconnect list
     *******************************************************************************************/
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingDisconnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Is the job complete? */
        retVal = Josh_isJobCompleted (ptrServerConsumer->ptrServerProducer->joshNodeHandle,
                                      ptrServerConsumer->jobId, &jobResult, &errCode);
        if (retVal < 0)
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: DAT Josh async completion job id %d [Error code %d]\n",
                        ptrServerConsumer->jobId, errCode);
            return numClientsServiced;
        }
        if (retVal == 0)
        {
            /* Job was NOT completed. */
            ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
            continue;
        }

        /* Job was complete: Cleanup the job. */
        if ( Josh_freeJobInstance(ptrServerConsumer->ptrServerProducer->joshNodeHandle, ptrServerConsumer->jobId) < 0 )
        {
            Dat_logMsg (ptrDatServer, Dat_LogLevel_ERROR, "Error: Free Josh job failed!\n");
            continue;
        }

        /* The JOB has been processed successfully */
        ptrServerConsumer->jobId                = -1;
        ptrServerConsumer->ptrServerProducer    = NULL;

        /* Remove the job from the pending disconnect list  */
        Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrPendingDisconnectList, (Dat_ListNode*)ptrServerConsumer);

        /* Move the consumer to the list of unattached consumers */
        Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList, (Dat_ListNode*)ptrServerConsumer);

        /* Get the next pending request if any. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList);
    }
    return numClientsServiced;
}

/**
 *  @b Description
 *  @n
 *      This function is used to get the JOSH node handle associated with
 *      DAT client handle.
 *
 *  @param[in]  serverHandle
 *      DAT Server Handle
 *  @param[in]  datClientHandle
 *      Handle to the DAT client
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      JOSH Node handle
 */
static Josh_NodeHandle Dat_getJoshNodeHandle
(
    Dat_ServerHandle serverHandle,
    Dat_ClientHandle datClientHandle
)
{
    Dat_ServerMCB*  ptrDatServer;
    int32_t         clientBlockIndex;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Cycle through all the DAT clients. */
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Do we have an active DAT client? */
        if (ptrDatServer->clientBlock[clientBlockIndex].status != Dat_ClientStatus_ACTIVE)
            continue;

        /* Is this a match? */
        if (ptrDatServer->clientBlock[clientBlockIndex].clientHandle == datClientHandle)
            return ptrDatServer->clientBlock[clientBlockIndex].joshHandle;
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the producer in the server matching the
 *      producer name
 *
 *  @param[in]  ptrDatServer
 *      Pointer to the DAT server
 *  @param[in]  producerName
 *      Producer name
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the DAT producer on match
 *  @retval
 *      Error   -   No match found
 */
static Dat_ServerProducer* Dat_findProducer
(
    Dat_ServerMCB*      ptrDatServer,
    char*               producerName
)
{
    Dat_ServerProducer* ptrServerProducer;

    /* Cycle through all the producer which exist on the DAT server and find the producer
     * which matches the name */
    ptrServerProducer = (Dat_ServerProducer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrProducerList);
    while (ptrServerProducer != NULL)
    {
        /* Do we have a match? */
        if (strncmp (ptrServerProducer->name, producerName, DAT_MAX_CHAR) == 0)
            return ptrServerProducer;

        /* Get the next producer. */
        ptrServerProducer = (Dat_ServerProducer*)Dat_listGetNext ((Dat_ListNode*)ptrServerProducer);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and creates a consumer
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the server consumer
 *  @retval
 *      Error   -   NULL
 */
Dat_ServerConsHandle _Dat_createConsumer
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    int32_t*            errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerConsumer* ptrServerConsumer;
    Josh_NodeHandle     consumerJoshNodeHandle;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the JOSH Node handle mapped to the DAT client block. */
    consumerJoshNodeHandle = Dat_getJoshNodeHandle(ptrDatServer, clientHandle);
    if (consumerJoshNodeHandle == NULL)
    {
        /* Internal Error: No matching JOSH handle for the client handle. This indicates
         * there is a corruption in the internal server database. */
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Allocate memory for the consumer */
    ptrServerConsumer = (Dat_ServerConsumer*)ptrDatServer->cfg.malloc (sizeof(Dat_ServerConsumer), 0);
    if (ptrServerConsumer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrServerConsumer, 0, sizeof(Dat_ServerConsumer));

    /* Populate the server consumer block:
     *  On creation the consumer block is NOT connected to a producer. */
    ptrServerConsumer->clientHandle         = clientHandle;
    ptrServerConsumer->joshNodeHandle       = consumerJoshNodeHandle;
    ptrServerConsumer->jobId                = -1;
    ptrServerConsumer->ptrServerProducer    = NULL;

    /* Add the consumer to the unattached consumer list: */
    Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList, (Dat_ListNode*)ptrServerConsumer);
    return (Dat_ServerConsHandle)ptrServerConsumer;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and is responsible for getting the status of the specified
 *      consumer
 *
 *  @param[in]  ptrDatServer
 *      Pointer to the DAT Server
 *  @param[in]  serverConsumerHandle
 *      DAT Server consumer handle
 *  @param[out] consumerStatus
 *      Consumer status populated by the API
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getConsumerStatus
(
    Dat_ServerMCB*          ptrDatServer,
    Dat_ServerConsHandle    serverConsumerHandle,
    Dat_ConsumerStatus*     consumerStatus,
    int32_t*                errCode
)
{
    Dat_ServerConsumer*     ptrServerConsumer;
    Dat_ServerProducer*     ptrServerProducer;

    /**********************************************************************************************
     * Unattached list:
     **********************************************************************************************/
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList);
    while (ptrServerConsumer != NULL)
    {
        /* Is this the block we are looking for? */
        if ((Dat_ServerConsHandle)ptrServerConsumer == serverConsumerHandle)
        {
            *consumerStatus = Dat_ConsumerStatus_DISCONNECTED;
            return 0;
        }

        /* Get the next consumer block in the list. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /**********************************************************************************************
     * Pending connect list:
     **********************************************************************************************/
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Is this the block we are looking for? */
        if ((Dat_ServerConsHandle)ptrServerConsumer == serverConsumerHandle)
        {
            *consumerStatus = Dat_ConsumerStatus_PENDING_CONNECT;
            return 0;
        }

        /* Get the next consumer block in the list. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /**********************************************************************************************
     * Pending disconnect list:
     **********************************************************************************************/
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingDisconnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Is this the block we are looking for? */
        if ((Dat_ServerConsHandle)ptrServerConsumer == serverConsumerHandle)
        {
            *consumerStatus = Dat_ConsumerStatus_PENDING_DISCONNECT;
            return 0;
        }

        /* Get the next consumer block in the list. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /**********************************************************************************************
     * Cycle through all the producers and its list of attached consumers
     **********************************************************************************************/
    ptrServerProducer = (Dat_ServerProducer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrProducerList);
    while (ptrServerProducer != NULL)
    {
        /* For each producer cycle through all the attached consumers. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrServerProducer->ptrConsumerList);
        while (ptrServerConsumer != NULL)
        {
            /* Is this the block we are looking for? */
            if ((Dat_ServerConsHandle)ptrServerConsumer == serverConsumerHandle)
            {
                *consumerStatus = Dat_ConsumerStatus_CONNECTED;
                return 0;
            }

            /* Get the next consumer block in the list. */
            ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
        }

        /* Cycle through to the next producer */
        ptrServerProducer = (Dat_ServerProducer*)Dat_listGetNext ((Dat_ListNode*)ptrServerProducer);
    }

    /* Control comes here implies that the consumer handle was invalid and does not exist in the system */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and deletes a consumer
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  serverConsumerHandle
 *      Handle of the consumer as it exists on the server
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_deleteConsumer
(
    Dat_ServerHandle        serverHandle,
    Dat_ServerConsHandle    serverConsumerHandle,
    int32_t*                errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerConsumer* ptrServerConsumer;

    /* Get the DAT server & server consumer blocks. */
    ptrDatServer        = (Dat_ServerMCB*)serverHandle;
    ptrServerConsumer   = (Dat_ServerConsumer*)serverConsumerHandle;

    /* Make sure that the consumer is not in the process of being disconnected */
    if (ptrServerConsumer->jobId != -1)
    {
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Ensure that there is no producer attached to the consumer */
    if (ptrServerConsumer->ptrServerProducer != NULL)
    {
        /* Error: This indicates that the database for the consumer & producers between the server
         * and clients are not synchronized. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* The consumer should be a part of the UNATTACHED list now since it is not connected anymore */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList, (Dat_ListNode*)ptrServerConsumer);

    /* Cleanup the memory */
    ptrDatServer->cfg.free (ptrServerConsumer, sizeof(Dat_ServerConsumer));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and creates a producer
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  producerInfo
 *      Producer information including name and memory logging info
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_createProducer
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_producerInfo*   producerInfo,
    int32_t*            errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Dat_ServerProducer*     ptrServerProducer;
    Josh_NodeHandle         producerJoshNodeHandle;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Sanity Check: Ensure that the producer name is unique in the DAT server */
    if (Dat_findProducer (ptrDatServer, producerInfo->name) != NULL)
    {
        /* Error: Duplicate producer name */
        *errCode = DAT_EDUP;
        return -1;
    }

    /* Get the JOSH Node handle mapped to the DAT client block. */
    producerJoshNodeHandle = Dat_getJoshNodeHandle(ptrDatServer, clientHandle);
    if (producerJoshNodeHandle == NULL)
    {
        /* Internal Error: No matching JOSH handle for the client handle. This indicates
         * there is a corruption in the internal server database. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Allocate memory for the producer: */
    ptrServerProducer = (Dat_ServerProducer*)ptrDatServer->cfg.malloc (sizeof(Dat_ServerProducer), 0);
    if (ptrServerProducer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrServerProducer, 0, sizeof(Dat_ServerProducer));

    /* Populate the producer block:
     * - The producer has been created and there are no consumers attached to it. */
    strncpy (ptrServerProducer->name, producerInfo->name, DAT_MAX_CHAR - 1 );
    ptrServerProducer->clientHandle         = clientHandle;
    ptrServerProducer->ptrConsumerList      = NULL;
    ptrServerProducer->joshNodeHandle       = producerJoshNodeHandle;
    ptrServerProducer->realm                = producerInfo->realm;

    /* Add this producer to the list of producers which are maintained on the server */
    Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrProducerList, (Dat_ListNode*)ptrServerProducer);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and deletes a producer
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  ptrProducerName
 *      Name of the producer
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_deleteProducer
(
    Dat_ServerHandle    serverHandle,
    char*               ptrProducerName,
    int32_t*            errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Dat_ServerProducer*     ptrServerProducer;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the DAT Server Producer */
    ptrServerProducer = Dat_findProducer (ptrDatServer, ptrProducerName);
    if (ptrServerProducer == NULL)
    {
        /* Error: Producer name does not exist in the DAT Server */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that there are no more consumers */
    if (Dat_listGetHead((Dat_ListNode**)&ptrServerProducer->ptrConsumerList) != NULL)
    {
        /* Error: There is a discrepancy between the client and server on the consumer counts. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Remove this producer from the list of producers which are maintained on the server */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrProducerList, (Dat_ListNode*)ptrServerProducer);

    /* Free the memory */
    ptrDatServer->cfg.free (ptrServerProducer, sizeof(Dat_ServerProducer));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and connects a consumer with a producer. The function relays
 *      the connect request from the consumer via the DAT Server to the
 *      producer.
 *
 *  @param[in]  serverHandle
 *      DAT Server Handle
 *  @param[in]  ptrConnectInfo
 *      Pointer to the connect information
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_connectConsumer
(
    Dat_ServerHandle        serverHandle,
    Dat_ConnectInfo*        ptrConnectInfo,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ServerMCB*          ptrDatServer;
    Dat_ConnectInfo*        ptrRemoteConnectInfo;
    Dat_ServerConsumer*     ptrServerConsumer;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the DAT Server Consumer block */
    ptrServerConsumer = (Dat_ServerConsumer*)ptrConnectInfo->serverConsumerHandle;

    /* Consumers can be attached to only 1 producer. */
    if (ptrServerConsumer->ptrServerProducer != NULL)
    {
        /* Error: Consumer is already connected */
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Find the producer and link this to the consumer. */
    ptrServerConsumer->ptrServerProducer = Dat_findProducer (ptrDatServer, &ptrConnectInfo->producerName[0]);
    if (ptrServerConsumer->ptrServerProducer == NULL)
    {
        /* Error: Producer name is invalid or has not been created till now. */
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Remove the consumer from the unattached list */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList, (Dat_ListNode*)ptrServerConsumer);

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrServerConsumer->ptrServerProducer->joshNodeHandle, (Josh_JobProtype)__Dat_connectConsumer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t __Dat_connectConsumer
     * (
     * Dat_ClientHandle    clientHandle,
     * Dat_ConnectInfo*    ptrConnectInfo,
     * int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ClientHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_ConnectInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrServerConsumer->ptrServerProducer->joshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrServerConsumer->ptrServerProducer->clientHandle);

    /* Populate the connect information. */
    ptrRemoteConnectInfo = (Dat_ConnectInfo*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteConnectInfo, (void*)ptrConnectInfo, sizeof(Dat_ConnectInfo));

    /* Relay the connect information to the producer DAT client */
    ptrServerConsumer->jobId = Josh_submitAsyncJob(jobHandle, argHandle, errCode);
    if (ptrServerConsumer->jobId < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Add the consumer to the pending list */
    Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList, (Dat_ListNode*)ptrServerConsumer);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and disconnects a consumer from the producer. The function
 *      relays the disconnect request from the consumer via the DAT Server
 *      to the producer.
 *
 *  @param[in]  serverHandle
 *      DAT Server Handle
 *  @param[in]  ptrConnectInfo
 *      Pointer to the connect information
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_disconnectConsumer
(
    Dat_ServerHandle        serverHandle,
    Dat_ConnectInfo*        ptrConnectInfo,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ServerMCB*          ptrDatServer;
    Dat_ConnectInfo*        ptrRemoteConnectInfo;
    Dat_ServerConsumer*     ptrServerConsumer;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the DAT Server Consumer block */
    ptrServerConsumer = (Dat_ServerConsumer*)ptrConnectInfo->serverConsumerHandle;

    /* Consumers should be attached to a producer */
    if (ptrServerConsumer->ptrServerProducer == NULL)
    {
        /* Error: Consumer is not connected; control implies that the DAT Server & client
         * are not synchronized. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrServerConsumer->ptrServerProducer->joshNodeHandle, (Josh_JobProtype)__Dat_disconnectConsumer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t __Dat_disconnectConsumer
     * (
     * Dat_ClientHandle    clientHandle,
     * Dat_ConnectInfo*    ptrConnectInfo,
     * int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ClientHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_ConnectInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrServerConsumer->ptrServerProducer->joshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrServerConsumer->ptrServerProducer->clientHandle);

    /* Populate the connect information. */
    ptrRemoteConnectInfo = (Dat_ConnectInfo*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteConnectInfo, (void*)ptrConnectInfo, sizeof(Dat_ConnectInfo));

    /* Relay the connect information to the producer DAT client */
    ptrServerConsumer->jobId = Josh_submitAsyncJob(jobHandle, argHandle, errCode);
    if (ptrServerConsumer->jobId < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Remove the consumer from the producer list */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrServerConsumer->ptrServerProducer->ptrConsumerList,
                        (Dat_ListNode*)ptrServerConsumer);

    /* Add the consumer to the pending disconnect list */
    Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrPendingDisconnectList, (Dat_ListNode*)ptrServerConsumer);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and calculates the general purpose producer data offset
 *
 *  @param[in]  serverHandle
 *      DAT Server Handle
 *  @param[in]  serverConsumerHandle
 *      Pointer to the serverconsumer Handle
 *  @param[out] dataOffset
 *      Custom data offset
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getGPProdDataOffset
(
    Dat_ServerHandle        serverHandle,
    Dat_ServerConsHandle    serverConsumerHandle,
    uint32_t*               dataOffset,
    int32_t*                errCode
)
{
    Dat_ServerConsumer*     ptrServerConsumer;
    Dat_ServerProducer*     ptrServerProducer;

    /* Get the DAT Server Consumer block */
    ptrServerConsumer = (Dat_ServerConsumer*)serverConsumerHandle;

    /* Consumers can be attached to only 1 producer. */
    if (ptrServerConsumer->ptrServerProducer == NULL)
    {
        /* Error: Consumer is already connected */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    ptrServerProducer = ptrServerConsumer->ptrServerProducer;
    if(ptrServerProducer->realm == Dat_ExecutionRealm_DSP)
    {
        /* On DSP, UIA packet use the following for UIA packet header and event header */
        *dataOffset = sizeof(UIAPacket_Hdr) + DAT_UIA_EVT_HEADER_SIZE;
    }
    else
    {
        /* On ARM, UIA packet use the following for UIA packet header and event header */
        *dataOffset = DAT_CUIA_PKT_HEADER_SIZE + DAT_UIA_EVT_HEADER_SIZE;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the global trace object in the server matching the
 *      trace object name
 *
 *  @param[in]  ptrDatServer
 *      Pointer to the DAT server
 *  @param[in]  traceObjectName
 *      Trace Object name
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the DAT trace object on match
 *  @retval
 *      Error   -   No match found
 */
static Dat_ServerGlobalTraceObject* Dat_findGlobalTraceObject
(
    Dat_ServerMCB*      ptrDatServer,
    char*               traceObjectName
)
{
    Dat_ServerGlobalTraceObject* ptrServerTraceObj;

    /* Cycle through all the trace object which exist on the DAT server and find the trace object
     * which matches the name */
    ptrServerTraceObj = (Dat_ServerGlobalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrGlobalTraceObjectList);
    while (ptrServerTraceObj != NULL)
    {
        /* Do we have a match? */
        if (strncmp (ptrServerTraceObj->traceObject.name, traceObjectName, DAT_MAX_CHAR) == 0)
            return ptrServerTraceObj;

        /* Get the next trace object. */
        ptrServerTraceObj = (Dat_ServerGlobalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrServerTraceObj);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the global trace object in the server matching the
 *      trace object name
 *
 *  @param[in]  ptrServerTraceObj
 *      Handle to the server trace object
 *  @param[in]  clientHandle
 *      DAT client handle
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the DAT trace object on match
 *  @retval
 *      Error   -   No match found
 */
static Dat_ServerLocalTraceObject* Dat_findTraceObjectInstance
(
    Dat_ServerGlobalTraceObject*  ptrServerTraceObj,
    Dat_ClientHandle              clientHandle
)
{
    Dat_ServerLocalTraceObject* ptrServerLocalTraceObj;

    /* Cycle through all the trace object Instance which exist on the DAT server and find the trace object
     * which matches the client handle */
    ptrServerLocalTraceObj = (Dat_ServerLocalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrServerTraceObj->ptrLocalTraceObjectList);
    while (ptrServerLocalTraceObj != NULL)
    {
        /* Do we have a match? */
        if (clientHandle == ptrServerLocalTraceObj->clientHandle)
            return ptrServerLocalTraceObj;

        /* Get the next trace object. */
        ptrServerLocalTraceObj = (Dat_ServerLocalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrServerLocalTraceObj);

    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and creates a global trace object
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrTraceObjCfg
 *      DAT trace object configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the server consumer
 *  @retval
 *      Error   -   NULL
 */
Dat_ServerTraceObjHandle _Dat_createTraceObject
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_TraceObjectCfg* ptrTraceObjCfg,
    int32_t*            errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerGlobalTraceObject* ptrServerTraceObj;
    Josh_NodeHandle     traceObjectJoshNodeHandle;
    Dat_TraceCompInfo*  ptrTraceObject;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Sanity Check: Ensure that the trace object name is unique in the DAT server */
    if (Dat_findGlobalTraceObject (ptrDatServer, ptrTraceObjCfg->traceObjectName) != NULL)
    {
        /* Error: Duplicate producer name */
        *errCode = DAT_EDUP;
        return NULL;
    }

    /* Get the JOSH Node handle mapped to the DAT client block. */
    traceObjectJoshNodeHandle = Dat_getJoshNodeHandle(ptrDatServer, clientHandle);
    if (traceObjectJoshNodeHandle == NULL)
    {
        /* Internal Error: No matching JOSH handle for the client handle. This indicates
         * there is a corruption in the internal server database. */
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Allocate memory for the GlobalTraceObject */
    ptrServerTraceObj = (Dat_ServerGlobalTraceObject*)ptrDatServer->cfg.malloc (sizeof(Dat_ServerGlobalTraceObject), 0);
    if (ptrServerTraceObj == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrServerTraceObj, 0, sizeof(Dat_ServerGlobalTraceObject));

    /* Save trace object info: number of components, size of the trace object */
    ptrServerTraceObj->traceObject.numTraceComponents = ptrTraceObjCfg->numTraceComponents;

    /* Trace object also contains class levels. Incorporate that it into number of components. */
    ptrServerTraceObj->traceObject.numTraceComponents += DAT_COMPONENT_START_ID;

    ptrServerTraceObj->traceObject.traceObjectSize = sizeof(Dat_TraceCompInfo) *  ptrServerTraceObj->traceObject.numTraceComponents;

    /* Allocate memory for the trace object array */
    ptrTraceObject = (Dat_TraceCompInfo*)ptrDatServer->cfg.malloc (ptrServerTraceObj->traceObject.traceObjectSize, 0);
    if (ptrTraceObject == NULL)
    {
        *errCode = DAT_ENOMEM;
        ptrDatServer->cfg.free(ptrServerTraceObj, sizeof(Dat_ServerGlobalTraceObject));
        return NULL;
    }

    /* Save the initial verbosity value from cfg */
    /* First element is the class and common component mask. */
    ptrTraceObject[DAT_CLASS_ID].logLevel = (ptrTraceObjCfg->classLevel << DAT_MASK_SIZE) | ptrTraceObjCfg->commonCompLevel;

    /* Populate the server trace object block. */
    strncpy(ptrServerTraceObj->traceObject.name, ptrTraceObjCfg->traceObjectName, DAT_MAX_CHAR - 1);
    ptrServerTraceObj->traceObject.ptrComponentArray = ptrTraceObject;
    ptrServerTraceObj->clientHandle                  = clientHandle;
    ptrServerTraceObj->joshNodeHandle                = traceObjectJoshNodeHandle;

    /* Add the consumer to the global trace object list: */
    Dat_listAdd ((Dat_ListNode**)&ptrDatServer->ptrGlobalTraceObjectList, (Dat_ListNode*)ptrServerTraceObj);

    return (Dat_ServerTraceObjHandle)ptrServerTraceObj;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and creates a trace object instance
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrTraceObjectBody
 *      DAT trace object configuration used to create trace object instance
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the server consumer
 *  @retval
 *      Error   -   NULL
 */
Dat_ServerLocalTraceObjHandle _Dat_createTraceObjectInstance
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_TraceObjectBody* ptrTraceObjectBody,
    int32_t*            errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerLocalTraceObject* ptrServerLocalTraceObj;
    Dat_ServerGlobalTraceObject* ptrServerTraceObj;
    Josh_NodeHandle     traceObjectJoshNodeHandle;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Find global trace object name in the DAT server */
    if ( (ptrServerTraceObj = Dat_findGlobalTraceObject (ptrDatServer, ptrTraceObjectBody->name)) == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_ENOTFOUND;
        return NULL;
    }

    if(ptrServerTraceObj->initialized == 0)
    {
        /* Error: Global trace object is not ready */
        *errCode = DAT_ENOTREADY;
        return NULL;
    }

    /* Get the JOSH Node handle mapped to the DAT client block. */
    traceObjectJoshNodeHandle = Dat_getJoshNodeHandle(ptrDatServer, clientHandle);
    if (traceObjectJoshNodeHandle == NULL)
    {
        /* Internal Error: No matching JOSH handle for the client handle. This indicates
         * there is a corruption in the internal server database. */
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Allocate memory for the TraceObject */
    ptrServerLocalTraceObj = (Dat_ServerLocalTraceObject*)ptrDatServer->cfg.malloc (sizeof(Dat_ServerLocalTraceObject), 0);
    if (ptrServerLocalTraceObj == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrServerLocalTraceObj, 0, sizeof(Dat_ServerLocalTraceObject));

    /* Populate the server local trace object block. */
    ptrServerLocalTraceObj->clientHandle         = clientHandle;
    ptrServerLocalTraceObj->joshNodeHandle       = traceObjectJoshNodeHandle;

    /* Attach the local trace object to the global trace object */
    ptrServerLocalTraceObj->ptrGlobalTraceObject = ptrServerTraceObj;

    /* Return the trace object body settings */
    memcpy((void *)ptrTraceObjectBody, (void *)&ptrServerTraceObj->traceObject, sizeof(Dat_TraceObjectBody));

    /* Add the consumer to the global trace object list: */
    Dat_listAdd ((Dat_ListNode**)&ptrServerTraceObj->ptrLocalTraceObjectList, (Dat_ListNode*)ptrServerLocalTraceObj);

    return (Dat_ServerTraceObjHandle)ptrServerLocalTraceObj;
}

/**
 *  @b Description
 *  @n
 *      The function is the the registered service that runs on DAT server to initialize DAT trace object
 *  components levels.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrCompBlock
 *      DAT internal structure to transfer component array blocks
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   =0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_initTraceComponentBlock
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_traceCompBlock* ptrCompBlock,
    int32_t*            errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerGlobalTraceObject* ptrServerTraceObj;
    uint8_t*  ptrServerCompArray;
    uint8_t*  ptrClientCompArray;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Find global trace object name in the DAT server */
    if ( (ptrServerTraceObj = Dat_findGlobalTraceObject (ptrDatServer, ptrCompBlock->name)) == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_ENOTFOUND;
        return -1;
    }

    /* Calculate the read/write pointer */
    ptrServerCompArray = (uint8_t*)&ptrServerTraceObj->traceObject.ptrComponentArray[DAT_COMPONENT_START_ID];
    ptrClientCompArray = (uint8_t*)((uint32_t)ptrCompBlock + sizeof(Dat_traceCompBlock));

    /* Sanity check of the array size */
    if( ((uint32_t)ptrServerCompArray + ptrCompBlock->blockOffset + ptrCompBlock->blockSize) >
        ((uint32_t)&ptrServerTraceObj->traceObject.ptrComponentArray[0] + ptrServerTraceObj->traceObject.traceObjectSize) )
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Copy the trace component block */
    if (ptrServerTraceObj->initialized == 0 && ptrServerTraceObj->traceObject.ptrComponentArray != NULL)
    {
        memcpy(ptrServerCompArray + ptrCompBlock->blockOffset, ptrClientCompArray, ptrCompBlock->blockSize);
    }

    if(ptrCompBlock->lastBlock == 1)
        ptrServerTraceObj->initialized = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server to get component trace levels for a trace object.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrCompBlock
 *      DAT internal structure to transfer component array blocks
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   =0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getTraceComponentBlock
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_traceCompBlock* ptrCompBlock,
    int32_t*            errCode
)
{
    Dat_ServerMCB*      ptrDatServer;
    Dat_ServerGlobalTraceObject* ptrServerTraceObj;
    uint8_t*  ptrServerCompArray;
    uint8_t*  ptrClientCompArray;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Find global trace object name in the DAT server */
    if ( (ptrServerTraceObj = Dat_findGlobalTraceObject (ptrDatServer, ptrCompBlock->name)) == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_ENOTFOUND;
        return -1;
    }

    /* Calculate the read/write pointer */
    ptrServerCompArray = (uint8_t*)&ptrServerTraceObj->traceObject.ptrComponentArray[0] + ptrCompBlock->blockOffset;
    ptrClientCompArray = (uint8_t*)((uint32_t)ptrCompBlock + sizeof(Dat_traceCompBlock));

    /* Sanick check of the array size */
    if( ((uint32_t)ptrServerCompArray + ptrCompBlock->blockSize) >
         ((uint32_t)&ptrServerTraceObj->traceObject.ptrComponentArray[0] + ptrServerTraceObj->traceObject.traceObjectSize) )
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the trace component block */
    if (ptrServerTraceObj->traceObject.ptrComponentArray == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the component array to client */
    memcpy(ptrClientCompArray, ptrServerCompArray,  ptrCompBlock->blockSize);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and deletes a local trace object
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  traceObjectName
 *      Name of the trace object
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_deleteTraceObjectInstance
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    char*               traceObjectName,
    int32_t*            errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Dat_ServerGlobalTraceObject*  ptrServerTraceObj;
    Dat_ServerLocalTraceObject* ptrServerLocalTraceObj;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the DAT Server Global Trace object */
    ptrServerTraceObj = Dat_findGlobalTraceObject (ptrDatServer, traceObjectName);
    if (ptrServerTraceObj == NULL)
    {
        /* Error: Trace Object name does not exist in the DAT Server */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Server Trace object Instance */
    ptrServerLocalTraceObj = Dat_findTraceObjectInstance (ptrServerTraceObj, clientHandle);
    if (ptrServerLocalTraceObj == NULL)
    {
        /* Error: Trace Object name does not exist in the DAT Server */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Removed the local trace object from global trace object */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrServerTraceObj->ptrLocalTraceObjectList, (Dat_ListNode*)ptrServerLocalTraceObj);

    /* Cleanup the memory */
    ptrDatServer->cfg.free (ptrServerLocalTraceObj, sizeof(Dat_ServerLocalTraceObject));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and deletes a global trace object
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  traceObjectName
 *      DAT Trace Object name
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_deleteTraceObject
(
    Dat_ServerHandle    serverHandle,
    char*               traceObjectName,
    int32_t*            errCode
)
{
    Dat_ServerMCB*          ptrDatServer;
    Dat_ServerGlobalTraceObject*  ptrServerTraceObj;

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Get the DAT Server Global Trace object */
    ptrServerTraceObj = Dat_findGlobalTraceObject (ptrDatServer, traceObjectName);
    if (ptrServerTraceObj == NULL)
    {
        /* Error: Trace Object name does not exist in the DAT Server */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that there are no more local trace object attached */
    if (Dat_listGetHead((Dat_ListNode**)&ptrServerTraceObj->ptrLocalTraceObjectList) != NULL)
    {
        /* Error: There is a discrepancy between the client and server on the trace object instance. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Remove this global trace object from the list which are maintained on the server */
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatServer->ptrGlobalTraceObjectList, (Dat_ListNode*)ptrServerTraceObj);

    /* Free the memory */
    ptrDatServer->cfg.free (ptrServerTraceObj, sizeof(Dat_ServerGlobalTraceObject));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function modifies the verbosity levels.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrVerbosityCfg
 *      Pointer to the verbosity configuration.
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_modifyVerbosity
(
    Dat_ServerHandle    serverHandle,
    Dat_ClientHandle    clientHandle,
    Dat_VerbosityCfg*   ptrVerbosityCfg,
    int32_t*            errCode
)
{
    Dat_TraceCompInfo*      ptrTraceObject;
    Dat_ServerGlobalTraceObject* ptrGlobalTraceObj;
    Dat_ServerLocalTraceObject* ptrLocalTraceObj;
    Dat_ServerMCB*          ptrDatServer;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_ArgHandle          argHandle;
    Josh_JobHandle          jobHandle;
    int32_t                 jobId = -1;
    Dat_verbosityOpBlock*   ptrVerbosityOpBlk;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (ptrVerbosityCfg == NULL)
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the parameters are valid. */
    if (ptrVerbosityCfg->traceObjectName[0] == NULL)
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the server handle */
    ptrDatServer = (Dat_ServerMCB *)serverHandle;

    /* Find the global trace object */
    ptrGlobalTraceObj = Dat_findGlobalTraceObject(ptrDatServer, ptrVerbosityCfg->traceObjectName);
    if(ptrGlobalTraceObj == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Create the bit mask for the new verbosity level. */
    if (((ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Component) &&
        (ptrVerbosityCfg->isComponentFocused)) ||
        (ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Class))

        ptrVerbosityCfg->verbosityLevel <<= DAT_MASK_SIZE;

    /* Offset the TC ID if it is component type. Else, use the class ID to
     * index into the trace object array. */
    if (ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Component)
        ptrVerbosityCfg->traceComponentId += DAT_COMPONENT_START_ID;
    else
        ptrVerbosityCfg->traceComponentId = DAT_CLASS_ID;

    /* Modify the verbosity level. */
    ptrTraceObject = ptrGlobalTraceObj->traceObject.ptrComponentArray;

    if (ptrVerbosityCfg->isEnabled)
        ptrTraceObject[ptrVerbosityCfg->traceComponentId].logLevel |=  ptrVerbosityCfg->verbosityLevel;
    else
        ptrTraceObject[ptrVerbosityCfg->traceComponentId].logLevel &=  ~ptrVerbosityCfg->verbosityLevel;

    /* Go through local trace object list to update trace object level accordingly */
    ptrLocalTraceObj = (Dat_ServerLocalTraceObject*)Dat_listGetHead((Dat_ListNode**)&ptrGlobalTraceObj->ptrLocalTraceObjectList);
    while(ptrLocalTraceObj)
    {
        /* Originator, do not send the command back */
        if(clientHandle == ptrLocalTraceObj->clientHandle)
        {
            /* Get the next local trace object instance */
            ptrLocalTraceObj  = (Dat_ServerLocalTraceObject*)Dat_listGetNext((Dat_ListNode*)ptrLocalTraceObj);
            continue;
        }

        /* Get the actual job which we will execute */
        jobHandle = Josh_findJobByAddress(ptrGlobalTraceObj->joshNodeHandle, (Josh_JobProtype)__Dat_modifyVerbosity);
        if (jobHandle == NULL)
        {
            *errCode = DAT_EINTERNAL;
            return -1;
        }

        /* Initialize the arguments; to avoid any junk */
        memset ((void *)&args, 0, sizeof(args));

        /***************************************************************************
         * This is the function which is to be invoked:
         *
         * int32_t __Dat_modifyVerbosity
         * (
         *     Dat_ClientHandle    clientHandle,
         *     Dat_VerbosityCfg*   ptrVerbosityCfg,
         *     int32_t*            errCode
         * )
         ****************************************************************************/

        /* Populate the arguments.
         * - Argument 1: */
        args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[0].length = sizeof(Dat_ClientHandle);

        /*  - Argument 2: */
        args[1].type   = Josh_ArgumentType_PASS_BY_REF;
        args[1].length = sizeof(Dat_VerbosityCfg);

        /*  - Argument 3: */
        args[2].type   = Josh_ArgumentType_PASS_BY_REF;
        args[2].length = sizeof(int32_t);

        /* Add the arguments. */
        argHandle = Josh_addArguments (ptrLocalTraceObj->joshNodeHandle, &args[0]);
        if (argHandle == NULL)
        {
            *errCode = DAT_ENORESOURCE;
            return -1;
        }

        /* Populate the arguments*/
        *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrLocalTraceObj->clientHandle);

        /* Populate the verbosity configuration information. */
        memcpy ((void*)args[1].argBuffer, (void*)ptrVerbosityCfg, sizeof(Dat_VerbosityCfg));

        /* Relay the connect information to the trace object DAT client */
        jobId = Josh_submitAsyncJob(jobHandle, argHandle, errCode);
        if (jobId < 0)
        {
            System_printf("Error: Faild to submit Async Josh job!\n");
            return -1;
        }

        /* Allocate memory for trace modification block */
        ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)ptrDatServer->cfg.malloc (sizeof(Dat_verbosityOpBlock), 0);
        if (ptrVerbosityOpBlk)
        {
            memset(ptrVerbosityOpBlk, 0, sizeof(Dat_verbosityOpBlock));
            ptrVerbosityOpBlk->jobId = jobId;
            ptrVerbosityOpBlk->joshNodeHandle = ptrLocalTraceObj->joshNodeHandle;
            ptrVerbosityOpBlk->ptrTraceObj = ptrLocalTraceObj;

            /* Add the trace object instance to the pending list */
            Dat_listAddTail ((Dat_ListNode**)&ptrDatServer->ptrPendingVerbosityOpList, (Dat_ListNode*)&ptrVerbosityOpBlk->opLinks);
        }
        else
        {
            *errCode = DAT_ENOMEM;
            return -1;
        }

        /* Get the next local trace object instance */
        ptrLocalTraceObj  = (Dat_ServerLocalTraceObject*)Dat_listGetNext((Dat_ListNode*)ptrLocalTraceObj);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is typically called through the Agent to determine the
 *      number of trace components belonging to a trace object.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  queryCfg
 *      Query configuration.
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getNumComponents
(
    Dat_ServerHandle       serverHandle,
    Dat_verbosityQueryCfg* queryCfg,
    int32_t*               errCode
)
{
    Dat_ServerGlobalTraceObject*  traceObjInfo;
    Dat_ServerMCB*      ptrDatServer;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (queryCfg == NULL || queryCfg->name[0] == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Check if the global trace object exists. */
    traceObjInfo = Dat_findGlobalTraceObject (ptrDatServer, queryCfg->name);
    if (traceObjInfo == NULL )
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Determine number of components from size of the trace object. Remove the
     * class related fields. */
    queryCfg->cfg.numComponents = traceObjInfo->traceObject.numTraceComponents - DAT_COMPONENT_START_ID;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function gets the verbosity level of the class and common level for
 *      all components, given the name of the trace object it belongs to.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  queryCfg
 *      Query configuration.
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getClassVerbosity
(
    Dat_ServerHandle        serverHandle,
    Dat_verbosityQueryCfg*  queryCfg,
    int32_t*                errCode
)
{
    Dat_ServerGlobalTraceObject*    traceObjInfo;
    Dat_ServerMCB*                  ptrDatServer;

    /* Check for valid inputs. */
    if (queryCfg == NULL || queryCfg->name[0] == NULL )
    {
        *errCode = DAT_EINVAL;
        return -1;
    }
    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Find the trace object. */
    traceObjInfo = Dat_findGlobalTraceObject (ptrDatServer, queryCfg->name);
    if (traceObjInfo == NULL )
    {
        *errCode = DAT_EINVAL;
        return -1;
    }
    /* Found the trace object. Get the class masks now.  Not using multicore
     * locks to access trace object. Unlikely that trace object is being
     * modified at the same time as it is being queried. */
    queryCfg->cfg.classCfg.classLevel    = traceObjInfo->traceObject.ptrComponentArray[DAT_CLASS_ID].logLevel >> DAT_MASK_SIZE;
    queryCfg->cfg.classCfg.commonCompLevel = traceObjInfo->traceObject.ptrComponentArray[DAT_CLASS_ID].logLevel & DAT_COMP_NONFOCUSED_MASK;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function gets the verbosity level of a given component.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  queryCfg
 *      Query configuration.
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getComponentVerbosity
(
    Dat_ServerHandle        serverHandle,
    Dat_verbosityQueryCfg*  queryCfg,
    int32_t*                errCode
)
{
    Dat_ServerGlobalTraceObject*    traceObjInfo;
    Dat_ServerMCB*                  ptrDatServer;
    uint32_t                        componentId;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (queryCfg == NULL || queryCfg->name[0] == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Server MCB */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Find the global trace object exists. */
    traceObjInfo = Dat_findGlobalTraceObject (ptrDatServer, queryCfg->name);
    if (traceObjInfo == NULL )
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Skip the class fields. */
    componentId = queryCfg->cfg.compCfg.componentId + DAT_COMPONENT_START_ID;

    /* Sanity check of the component id */
    if(queryCfg->cfg.compCfg.componentId >= traceObjInfo->traceObject.numTraceComponents)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Found the trace object. Find the component now.  Not using multicore
     * locks to access trace object. Unlikely that trace object is being
     * modified at the same time as it is being queried. */
    strncpy (queryCfg->cfg.compCfg.componentInfo.componentName, traceObjInfo->traceObject.ptrComponentArray[componentId].componentName, DAT_TRACE_COMPONENT_MAX_CHAR-1);
    queryCfg->cfg.compCfg.componentInfo.focusedLevel    = traceObjInfo->traceObject.ptrComponentArray[componentId].logLevel >> DAT_MASK_SIZE;
    queryCfg->cfg.compCfg.componentInfo.nonFocusedLevel = traceObjInfo->traceObject.ptrComponentArray[componentId].logLevel & DAT_COMP_NONFOCUSED_MASK;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function gets names for trace objects.
 *
 *  @param[in]  serverHandle
 *      DAT server handle
 *  @param[in]  nameArraySize
 *      Size of the the array that will hold the trace object names
 *  @param[in]  nameArray
 *      Array to hold trace object names
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t _Dat_getTraceObjectNames
(
    Dat_ServerHandle    serverHandle,
    uint32_t            nameArraySize,
    char*               nameArray,
    int32_t*            errCode
)
{
    Dat_ServerMCB*                ptrDatServer;
    Dat_ServerGlobalTraceObject*  ptrTraceObject;
    Dat_ServerLocalTraceObject*   ptrTraceObjectInst;
    uint32_t                      traceObjectIndex = 0;
    char*                         ptrNameArray;

    /* Sanity check */
    if ( (nameArray == NULL) || (nameArraySize == 0) || (serverHandle == NULL))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT Server block. */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;

    /* Initialize name array */
    ptrNameArray = nameArray;

    /* Display a list of all the DAT producers */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Trace Object List:\n");
    ptrTraceObject = (Dat_ServerGlobalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrGlobalTraceObjectList);
    while (ptrTraceObject != NULL)
    {
        /* Log the producer name */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Trace Object Name = '%s'\n", ptrTraceObject->traceObject.name);

        /* Display a list of all the trace object instance client handle */
        ptrTraceObjectInst = (Dat_ServerLocalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrTraceObject->ptrLocalTraceObjectList);
        while (ptrTraceObjectInst != NULL)
        {
            /* Log the server consumer block */
            Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "-> TraceObject instance on client = %x\n", ptrTraceObjectInst->clientHandle);

            /* Get the next attached consumer */
            ptrTraceObjectInst = (Dat_ServerLocalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrTraceObjectInst);
        }

        /* Copy trace object name to the list */
        if( (ptrNameArray + DAT_MAX_CHAR) <= (nameArray + nameArraySize ))
            strncpy((char *)ptrNameArray, ptrTraceObject->traceObject.name, DAT_MAX_CHAR);

        /* Increment trace object index and name array pointer */
        traceObjectIndex++;
        ptrNameArray += DAT_MAX_CHAR;

        /* Get the next producer */
        ptrTraceObject = (Dat_ServerGlobalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrTraceObject);
    }

    /* Set number of trace objects */
    return traceObjectIndex;
}

/**
 *  @b Description
 *  @n
 *      This function is used to display the DAT Server details
 *      for debugging
 *
 *  @param[in]  serverHandle
 *      Handle to the DAT server
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Dat_displayServer(Dat_ServerHandle serverHandle)
{
    Dat_ServerMCB*          ptrDatServer;
    int32_t                 clientBlockIndex;
    Dat_ServerProducer*     ptrServerProducer;
    Dat_ServerConsumer*     ptrServerConsumer;
    Dat_ServerGlobalTraceObject*  ptrTraceObject;
    Dat_ServerLocalTraceObject*   ptrTraceObjectInst;
    Dat_verbosityOpBlock*   ptrVerbosityOpBlk;

    /* Get the pointer to the DAT Server block. */
    ptrDatServer = (Dat_ServerMCB*)serverHandle;
    if (ptrDatServer == NULL)
        return;

    /*******************************************************************************
     ******************************** DAT Clients **********************************
     *******************************************************************************/

    /* Display a list of all the DAT clients which are connected and active */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Active DAT Clients:\n");
    for (clientBlockIndex = 0; clientBlockIndex < DAT_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Do we have an active DAT client? */
        if (ptrDatServer->clientBlock[clientBlockIndex].status != Dat_ClientStatus_ACTIVE)
            continue;

        /* Log the DAT client name which is connected */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "DAT Client %d = '%s'\n",
                    clientBlockIndex, ptrDatServer->clientBlock[clientBlockIndex].name);
    }

    /*******************************************************************************
     ******************************* DAT Producers *********************************
     *******************************************************************************/

    /* Display a list of all the DAT producers */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Producer List:\n");
    ptrServerProducer = (Dat_ServerProducer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrProducerList);
    while (ptrServerProducer != NULL)
    {
        /* Log the producer name */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Producer Name = '%s'\n", ptrServerProducer->name);

        /* Display a list of all the consumers which are connected to this producer. */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrServerProducer->ptrConsumerList);
        while (ptrServerConsumer != NULL)
        {
            /* Log the server consumer block */
            Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "-> Consumer = %x\n", ptrServerConsumer);

            /* Get the next attached consumer */
            ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
        }

        /* Get the next producer */
        ptrServerProducer = (Dat_ServerProducer*)Dat_listGetNext ((Dat_ListNode*)ptrServerProducer);
    }

    /*******************************************************************************
     **************************** Unconnected Consumers ****************************
     *******************************************************************************/

    /* Display a list of all the DAT consumers which are not connected */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Unconnected Consumers:\n");
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrUnattachedConsumerList);
    while (ptrServerConsumer != NULL)
    {
        /* Log the server consumer block: */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Consumer = %x\n", ptrServerConsumer);

        /* Get the next consumer */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /*******************************************************************************
     ************************ Pending Connect Consumers ****************************
     *******************************************************************************/

    /* Display a list of all the DAT consumers which are pending connection requests */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Pending Connect Consumers:\n");
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingConnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Log the producer name for which the transaction is pending */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Consumer Name = '%s'\n", ptrServerConsumer->ptrServerProducer->name);

        /* Get the next consumer */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /*******************************************************************************
     ************************ Pending Disconnect Consumers *************************
     *******************************************************************************/

    /* Display a list of all the DAT consumers which are pending connection requests */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Pending Disconnect Consumers:\n");
    ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrPendingDisconnectList);
    while (ptrServerConsumer != NULL)
    {
        /* Log the producer name for which the transaction is pending */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Consumer Name = '%s'\n", ptrServerConsumer->ptrServerProducer->name);

        /* Get the next consumer */
        ptrServerConsumer = (Dat_ServerConsumer*)Dat_listGetNext ((Dat_ListNode*)ptrServerConsumer);
    }

    /*******************************************************************************
     ************************ Trace Object list ************************************
     *******************************************************************************/
    /* Display a list of all the DAT producers */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Trace Object List:\n");
    ptrTraceObject = (Dat_ServerGlobalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrDatServer->ptrGlobalTraceObjectList);
    while (ptrTraceObject != NULL)
    {
        /* Log the producer name */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Trace Object Name = '%s'\n", ptrTraceObject->traceObject.name);

        /* Display a list of all the trace object instance client handle */
        ptrTraceObjectInst = (Dat_ServerLocalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrTraceObject->ptrLocalTraceObjectList);
        while (ptrTraceObjectInst != NULL)
        {
            /* Log the server consumer block */
            Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "-> TraceObject instance on client = %x\n", ptrTraceObjectInst->clientHandle);

            /* Get the next attached consumer */
            ptrTraceObjectInst = (Dat_ServerLocalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrTraceObjectInst);
        }

        /* Get the next producer */
        ptrTraceObject = (Dat_ServerGlobalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrTraceObject);
    }

    /*******************************************************************************
     ************************ Pending verbosity operation list *********************
     *******************************************************************************/
    /* Display a list of all the DAT producers */
    Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "Pending verbosity operation List:\n");
    ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)Dat_listGetHead((Dat_ListNode**)&ptrDatServer->ptrPendingVerbosityOpList);
    while (ptrVerbosityOpBlk != NULL)
    {
        /* Log the verbosity pending block */
        Dat_logMsg (ptrDatServer, Dat_LogLevel_INFO, "-> Pending verbosity opertion  on client =%x , joshid =%d\n",
             ptrVerbosityOpBlk->ptrTraceObj->clientHandle, ptrVerbosityOpBlk->jobId);

        /* Get the next pending request if any. */
        ptrVerbosityOpBlk = (Dat_verbosityOpBlock*)Dat_listGetNext((Dat_ListNode*)&ptrVerbosityOpBlk->opLinks);
    }
    return;
}

