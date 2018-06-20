/**
 *   @file  dat_client.c
 *
 *   @brief
 *      The file implements the DAT client functionality.
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
#include <errno.h>

/* SYSLIB Include files */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/dat/include/dat_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>

/* CSL include files */
#include <ti/csl/cslr_sem.h>
#include <ti/csl/csl_cache.h>

/* UIA Include Files */
#include <ti/uiaplus/loggers/multistream/LoggerStreamer2.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/runtime/UIAPacket.h>
#include <ti/uia/runtime/UIAPacket_support.h>
#include <ti/uia/runtime/EventHdr.h>
#include <ti/uia/events/UIAEvt.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/runtime/LogSync.h>

/* HPLIB Include Files */
#include <ti/runtime/hplib/hplib.h>
#include <ti/runtime/hplib/hplibmod.h>
#include <ti/runtime/hplib/src/hplib_loc.h>

#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_semAux.h>

/* UIA Include Files */
#include <ti/uia/sysbios/LoggerStreamer2.h>
#include <ti/uia/runtime/LogSync.h>
#include <ti/uia/events/UIASync.h>
#include <ti/uia/runtime/UIAPacket.h>
#include <ti/uia/runtime/EventHdr.h>
#include <ti/uia/events/UIASnapshot.h>
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#endif

#ifdef __ARMv7
hplib_spinLock_T     loggerLock;
#endif

/**********************************************************************
 *********************** DAT Client Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      This is a utility function which creates a unique MSGCOM channel name
 *      for the consumer using the producer name and the consumer block address
 *      This combination will be unique in the system.
 *
 *  @param[in]   producerName
 *      Producer Name
 *  @param[in]   ptrConsumer
 *      Pointer to the consumer block
 *  @param[out]   ptrChannelName
 *      Pointer to the unique MSGCOM channel name
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static inline void Dat_constructChannelName
(
    char*           producerName,
    Dat_Consumer*   ptrConsumer,
    char*           ptrChannelName
)
{
    /* Create the unique channel name */
    snprintf (ptrChannelName, MSG_COM_MAX_CHANNEL_LEN, "%s-%x", producerName, (uint32_t)ptrConsumer);
}

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
 *      Not applicable
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
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Dat_clientJoshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Ti_Pkt*             ptrPkt;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataBufferLen;
    Dat_ClientMCB*      ptrDatClient;

    /* Get the DAT client information block. */
    ptrDatClient = (Dat_ClientMCB*)nodeId;

    /* Allocate a packet from the agent server heap */
    ptrPkt = Pktlib_allocPacket(ptrDatClient->cfg.pktlibInstHandle, ptrDatClient->cfg.clientHeapHandle, size);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: JOSH Packet allocation from heap %p [%d bytes] failed\n",
                       ptrDatClient->cfg.clientHeapHandle, size);
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
static void Dat_clientJoshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Ti_Pkt*             ptrPkt;
    Dat_ClientMCB*      ptrDatClient;

    /* Get the DAT Client information block. */
    ptrDatClient = (Dat_ClientMCB*)nodeId;

    /* Get the packet buffer. */
    ptrPkt = (Ti_Pkt*)msgBuffer;
    if (ptrPkt == NULL)
        return;

    /* Cleanup the packet. */
    Pktlib_freePacket(ptrDatClient->cfg.pktlibInstHandle, ptrPkt);
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
static void Dat_clientJoshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    uint32_t            dataBufferLen;
    Dat_ClientMCB*      ptrDatClient;

    /* Get the DAT Client information block. */
    ptrDatClient = (Dat_ClientMCB*)nodeId;

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
    ptrDatClient->cfg.beginMemAccess(*ptrDataBuffer, dataBufferLen);
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
static void Dat_clientJoshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    Dat_ClientMCB*      ptrDatClient;

    /* Get the DAT Client information block. */
    ptrDatClient = (Dat_ClientMCB*)nodeId;

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)msgBuffer, &ptrDataBuffer, &dataBufferLen);

    /* Writeback the data buffer. */
    ptrDatClient->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /* Send the message */
    Msgcom_putMessage((MsgCom_ChHandle)writerChannel, (MsgCom_Buffer*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to find a producer on the DAT client
 *      given the name
 *
 *  @param[in]   clientHandle
 *      DAT client handle
 *  @param[in]   producerName
 *      Name of the produder
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Match       - Pointer to the producer block
 *  @retval
 *      Non-Match   - NULL
 */
static Dat_Producer* Dat_findProducer (Dat_ClientHandle clientHandle, char* producerName)
{
    Dat_ClientMCB*  ptrDatClient;
    Dat_Producer*   ptrProducer;
    void*           csHandle;

    /* Get the DAT client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    /* Critical Section: Protect the producer list */
    csHandle = ptrDatClient->cfg.enterCS();

    /* Get the producer */
    ptrProducer = (Dat_Producer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrProducerList);
    while (ptrProducer != NULL)
    {
        /* Do we have a match? */
        if (strncmp (ptrProducer->cfg.name, producerName, DAT_MAX_CHAR) == 0)
            break;

        /* Get the next producer */
        ptrProducer = (Dat_Producer*)Dat_listGetNext ((Dat_ListNode*)ptrProducer);
    }

    /* Exit the critical section: */
    ptrDatClient->cfg.exitCS(csHandle);
    return ptrProducer;
}


/**
 *  @b Description
 *  @n
 *      The function is used to initialize the DAT client. Clients can only
 *      be initialized once the server is operational.
 *
 *  @param[in]  ptrClientCfg
 *      Pointer to the DAT client configuration.
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   DAT Client Handle
 *  @retval
 *      Error   -   NULL
 */
Dat_ClientHandle Dat_initClient
(
    Dat_ClientCfg*  ptrClientCfg,
    int32_t*        errCode
)
{
    Dat_ClientMCB*          ptrDatClient;
    Name_ResourceCfg        namedResourceCfg;
    Msgcom_ChannelCfg       chConfig;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];
    uint32_t                idx;
    Dat_loggerInstId*       ptrLoggerId;
    uint16_t                loggerBaseId;

    /* Sanity Check: Validate the arguments */
    if (ptrClientCfg == NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrClientCfg->msgcomInstHandle == NULL) ||
        (ptrClientCfg->pktlibInstHandle == NULL) ||
        (ptrClientCfg->databaseHandle   == NULL) ||
        (ptrClientCfg->clientHeapHandle == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the OSAL call table */
    if ((ptrClientCfg->malloc         == NULL) || (ptrClientCfg->free         == NULL) ||
        (ptrClientCfg->enterCS        == NULL) || (ptrClientCfg->exitCS       == NULL) ||
        (ptrClientCfg->beginMemAccess == NULL) || (ptrClientCfg->endMemAccess == NULL) ||
        (ptrClientCfg->createSem      == NULL) || (ptrClientCfg->deleteSem    == NULL) ||
        (ptrClientCfg->postSem        == NULL) || (ptrClientCfg->pendSem      == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

#ifndef __ARMv7
    /* Check if a valid sync logger has been passed. If sync period is 0, then
     * no sync points are required, and hence no sync logger is required.
     * However, if a non-zero sync period has been passed, then a valid sync
     * logger should be present too. */
    if ( (ptrClientCfg->logSync.syncLogger == NULL) &&
            (ptrClientCfg->logSync.syncPeriod > 0) )
    {
        /* Invalid logger handle. */
        *errCode = DAT_EINVAL;
        return NULL;
    }
#endif

    /* Sanity Check: Ensure that the client name is unique in the name space. */
    Name_findResource (ptrClientCfg->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       ptrClientCfg->clientName, &namedResourceCfg, errCode);

    /* The instance name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the instance name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the server. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Find the server. Check the local database  */
    if (Name_findResource (ptrClientCfg->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                           ptrClientCfg->serverName, &namedResourceCfg, errCode) < 0)
    {
        /* Error: Server name is not in the local database. Do we need to check the remote database? */
        if (ptrClientCfg->nameClientHandle == NULL)
        {
            /* Error: Server name was not found. This could be because of an invalid server name *OR*
             * the client was being created without starting the server. Either case there is an invalid
             * usage */
            *errCode = DAT_EINVAL;
            return NULL;
        }
        else
        {
            /* Check the remote database to see if the server is found there. */
            if (Name_get (ptrClientCfg->nameClientHandle, Name_getDatabaseInstanceId (ptrClientCfg->databaseHandle, errCode),
                          Name_ResourceBucket_INTERNAL_SYSLIB, ptrClientCfg->serverName, &namedResourceCfg, errCode) < 0)
            {
                /* Error: Server name was not found in the remote database. Either an invalid name was specified
                 * or the client was started without waiting for the server. */
                *errCode = DAT_EINVAL;
                return NULL;
            }
        }
    }

    /* Allocate memory for the DAT client */
    ptrDatClient = (Dat_ClientMCB*)ptrClientCfg->malloc (sizeof(Dat_ClientMCB), 0);
    if (ptrDatClient == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory */
    memset ((void*)ptrDatClient, 0, sizeof(Dat_ClientMCB));

    /* Populate the DAT client block. */
    memcpy ((void *)&ptrDatClient->cfg, (void *)ptrClientCfg, sizeof(Dat_ClientCfg));

#ifdef __ARMv7
    /* Initialize the Global timestamp clock frequency value if not configured */
    if( (ptrDatClient->cfg.globalTsFreq.lo == 0) &&
        (ptrDatClient->cfg.globalTsFreq.hi == 0) )
    {
        /* Set it to 1GHz */
        ptrDatClient->cfg.globalTsFreq.hi = 0;
        ptrDatClient->cfg.globalTsFreq.lo = 0x3B9ACA00;
    }

#endif

    /* Remember the DAT Server handle */
    ptrDatClient->serverHandle = (Dat_ServerHandle)namedResourceCfg.handle1;
    ptrDatClient->serverRealm  = (Dat_ExecutionRealm)namedResourceCfg.handle2;

     /* Get logger base instance id configuration */
    loggerBaseId = DAT_CLIENT_DYNAMIC_INSTANCE_BASE;

    /* Initialize logger instance list */
    for (idx=0; idx < DAT_CLIENT_MAX_DYNAMIC_INSTANCE; idx++)
    {
        /* Allocate logger id structure and add it to the free list */
        ptrLoggerId = (Dat_loggerInstId *)ptrDatClient->cfg.malloc(sizeof(Dat_loggerInstId), 0);
        if (ptrLoggerId == NULL)
        {
            *errCode = DAT_ENOMEM;
            return NULL;
        }

        /* Initialize the structure */
        ptrLoggerId->status       = DAT_LOGGER_ID_FREE;
        ptrLoggerId->loggerInstId = loggerBaseId + idx;

        /* Add the logger instance id to free list */
        Dat_listAddTail ((Dat_ListNode**)&ptrDatClient->ptrLoggerIdFreeList, (Dat_ListNode*)ptrLoggerId);
    }

    /* Create the unique client channel name; */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-CLNT", ptrDatClient->cfg.clientName);

    /* Initialize the channel configuration: */
    memset ((void*)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration:
     * - DAT client channels are QUEUE-DMA but execute in NO interrupt and non blocking
     *   mode which requires that applications poll on DAT clients.  */
    chConfig.mode                         = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.msgcomInstHandle             = ptrDatClient->cfg.msgcomInstHandle;
    chConfig.appCallBack                  = NULL;
    chConfig.u.queueDMACfg.interruptMode  = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrDatClient->cfg.clientHeapHandle)));

    /* Open the MSGCOM Queue-DMA client channel handle. */
    ptrDatClient->clientChannel = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
    if (ptrDatClient->clientChannel == NULL)
        return NULL;

    /* DAT client has been initialized. */
    ptrDatClient->status = Dat_ClientStatus_INITIALIZED;

    /* Initialize the named resource configuration. */
    memset ((void *)&namedResourceCfg, 0 , sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    namedResourceCfg.handle1  = (uint32_t)ptrDatClient;
    namedResourceCfg.handle2  = (uint32_t)ptrDatClient->cfg.realm;
    namedResourceCfg.handle3  = (uint32_t)ptrDatClient->status;
    strcpy(namedResourceCfg.name, ptrDatClient->cfg.clientName);

    /* Create & register the DAT client instance announcing that the DAT client has been initialized. */
    if (Name_createResource(ptrClientCfg->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, errCode) < 0)
    {
        System_printf ("Error: Registering DAT client '%s' failed [Error code %d]\n", namedResourceCfg.name, *errCode);
        return NULL;
    }

    /* If the server and client are in different realms; we need to push the channel using the name services */
    if (ptrDatClient->serverRealm != ptrClientCfg->realm)
    {
        /* YES. Server & clients are on different realms use the agent to push the names across
         * This includes the DAT Client & DAT client MSGCOM channel names. */
        if (Name_push (ptrClientCfg->nameClientHandle, channelName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            /* Error code already populated with the agent error code. */
            return NULL;
        }
        System_printf ("Debug: Pushing DAT Client MSGCOM Channel '%s' across realms\n", channelName);

        if (Name_push (ptrClientCfg->nameClientHandle, ptrDatClient->cfg.clientName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            /* Error code already populated with the agent error code. */
            return NULL;
        }
        System_printf ("Debug: Pushing DAT Client Name '%s' across realms\n", ptrDatClient->cfg.clientName);
    }
    else
    {
        /* NO. Server & client are on the same realm. */
        System_printf ("Debug: DAT Client and Server on the same realm\n");
    }

    /* Setup Semaphore access on DAT client */
    {
#ifdef __ARMv7
        /* Initialize uio map for EMU count */
        ptrDatClient->ptrUioMapInfo = Dat_uioInitMemMap(ptrDatClient, "emucnt", errCode);
        if(ptrDatClient->ptrUioMapInfo == NULL)
            return NULL;

        /* Spin Lock initialization , it is used for CUIA */
        hplib_mSpinLockInit(&loggerLock);
#endif
    }

    /* Record the timestamp to send out sync messages. */
    LogSync_enable();
    DAT_TIMESTAMP(ptrDatClient->cfg.logSync.logSyncTimestamp);

    return (Dat_ClientHandle)ptrDatClient;
}

/**
 *  @b Description
 *  @n
 *      This function is used to start the DAT server. This needs to be invoked
 *      before a DAT client can be created.
 *
 *  @param[in]  databaseHandle
 *      Database Handle to be looked at for the DAT Server
 *  @param[in]  clientHandle
 *      Name Client handle which needs to be specified if the DAT client and server
 *      are executing in different realms
 *  @param[in]  serverName
 *      Name of the server which is to started
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   DAT Server Handle
 *  @retval
 *      Error   -   NULL
 */
Dat_ServerHandle Dat_startServer
(
    Name_DBHandle       databaseHandle,
    Name_ClientHandle   clientHandle,
    char*               serverName,
    int32_t*            errCode
)
{
    Name_ResourceCfg    namedResourceCfg;
    Dat_ServerMCB*      ptrDatServer;

    /* Sanity Check: Validate the arguments */
    if ((serverName == NULL) || (databaseHandle == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Is there a name client handle specified? */
    if (clientHandle == NULL)
    {
        /* NO. Check the local database only to determine if the server has been registered or not? */
        if (Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               serverName, &namedResourceCfg, errCode) < 0)
        {
            /* Error: Resource was NOT found or there was an internal error. */
            if (*errCode != NAME_ENOTFOUND)
            {
                /* Internal Error: Named resource module has returned an error. We return the error code
                 * of the resource manager back to the application. */
                return NULL;
            }

            /* Resource does not exist. */
            *errCode = DAT_ENOTREADY;
            return NULL;
        }
    }
    else
    {
        /* YES. Check the remote database to determine if the server has been registered */
        if (Name_get (clientHandle, Name_getDatabaseInstanceId (databaseHandle, errCode),
                      Name_ResourceBucket_INTERNAL_SYSLIB, serverName, &namedResourceCfg, errCode) < 0)
        {
            /* Error: Resource was NOT found or there was an internal error. */
            if (*errCode != NAME_ENOTFOUND)
            {
                /* Internal Error: Named resource module has returned an error. We return the error code
                 * of the resource manager back to the application. */
                return NULL;
            }

            /* Resource does not exist. */
            *errCode = DAT_ENOTREADY;
            return NULL;
        }
    }

    /* Control comes here implies that the server has been created and is operational. Get the instance information */
    ptrDatServer = (Dat_ServerMCB*)namedResourceCfg.handle1;
    if (ptrDatServer == NULL)
    {
        /* Error: DAT Server module registered with a NULL handle should never occur. Corruption detected. */
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Return the instance handle */
    return (Dat_ServerHandle)ptrDatServer;
}



/**
 *  @b Description
 *  @n
 *      The function is used to start the DAT client. The DAT client can only
 *      be started once the DAT server has acknowledged its initialization
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is to be registered
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_initClient
 *
 *  @retval
 *      1   - Client is active
 *  @retval
 *      0   - Client is not active
 *  @retval
 *      <0  - Error
 */
int32_t Dat_startClient
(
    Dat_ClientHandle    clientHandle,
    int32_t*            errCode
)
{
    Name_ResourceCfg    namedResourceCfg;
    Dat_ClientMCB*      ptrDatClient;
    Josh_NodeCfg        nodeCfg;
    char                channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Is the server and client executing in the same realm?*/
    if (ptrDatClient->serverRealm == ptrDatClient->cfg.realm)
    {
        /* YES. Checking the local database */
        if (Name_findResource (ptrDatClient->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               ptrDatClient->cfg.clientName, &namedResourceCfg, errCode) < 0)
        {
            /* Resource does not exist: This is not possible since the client was already created; an invalid
             * argument was passed or something went wrong. */
            *errCode = DAT_EINTERNAL;
            return -1;
        }
    }
    else
    {
        /* NO. Checking the remote database */
        if (Name_get (ptrDatClient->cfg.nameClientHandle, Name_getDatabaseInstanceId (ptrDatClient->cfg.databaseHandle, errCode),
                      Name_ResourceBucket_INTERNAL_SYSLIB, ptrDatClient->cfg.clientName, &namedResourceCfg, errCode) < 0)
        {
            /* Resource does not exist: This is not possible since the client was already created; an invalid
             * argument was passed or something went wrong. */
            *errCode = DAT_EINTERNAL;
            return -1;
        }
    }

    /* Get the status of the client
     *  - After successful registeration the server will mark handle3 to indicate the new status */
    ptrDatClient->status = (Dat_ClientStatus)namedResourceCfg.handle3;

    /* Is the client active? */
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
        return 0;

    /* Activate the DAT Client JOSH services. */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-SRV", ptrDatClient->cfg.clientName);
    ptrDatClient->serverChannel = Msgcom_find (ptrDatClient->cfg.msgcomInstHandle, channelName, errCode);
    if (ptrDatClient->serverChannel == NULL)
    {
        /* Error: MSGCOM Channel does not exist in the local database. Is the client and server executing on
         * different realms? */
        if (ptrDatClient->cfg.nameClientHandle == NULL)
        {
            /* FATAL Error: MSGCOM channel does not exist; yet the client is marked as ACTIVE. */
            *errCode = DAT_EINTERNAL;
            return -1;
        }
        else
        {
            /* YES. Check the remote database; the MSGCOM channel was created by the server in the remote
             * database */
            if (Name_get (ptrDatClient->cfg.nameClientHandle, Name_getDatabaseInstanceId(ptrDatClient->cfg.databaseHandle, errCode),
                          Name_ResourceBucket_INTERNAL_SYSLIB, channelName, &namedResourceCfg, errCode) < 0)
            {
                /* FATAL Error: The MSGCOM channel was not found there; return the error code back to the
                 * application. */
                return -1;
            }

            /* Server channel information was retreived from the remote database. Add it locally */
            if (Name_createResource (ptrDatClient->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                     &namedResourceCfg, errCode) < 0)
            {
                /* FATAL Error: Unable to add the server channel information to the local database */
                return -1;
            }

            /* Now use the MSGCOM find to find the channel in the local database*/
            ptrDatClient->serverChannel = Msgcom_find (ptrDatClient->cfg.msgcomInstHandle, channelName, errCode);
            if (ptrDatClient->serverChannel == NULL)
            {
                /* FATAL Error: Still could not find the MSGCOM channel. This should never occur. */
                *errCode = DAT_EINTERNAL;
                return -1;
            }
        }
    }

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId                     = (uint32_t)ptrDatClient;
    nodeCfg.transportCfg.readerChannel = ptrDatClient->clientChannel;
    nodeCfg.transportCfg.writerChannel = ptrDatClient->serverChannel;

    /* Populate the transport interface */
    nodeCfg.transportCfg.transport.alloc = Dat_clientJoshAlloc;
    nodeCfg.transportCfg.transport.free  = Dat_clientJoshFree;
    nodeCfg.transportCfg.transport.get   = Dat_clientJoshGet;
    nodeCfg.transportCfg.transport.put   = Dat_clientJoshPut;

    /* Populate the OSAL table */
    nodeCfg.malloc      = ptrDatClient->cfg.malloc;
    nodeCfg.free        = ptrDatClient->cfg.free;
    nodeCfg.enterCS     = ptrDatClient->cfg.enterCS;
    nodeCfg.exitCS      = ptrDatClient->cfg.exitCS;
    nodeCfg.createSem   = ptrDatClient->cfg.createSem;
    nodeCfg.deleteSem   = ptrDatClient->cfg.deleteSem;
    nodeCfg.postSem     = ptrDatClient->cfg.postSem;
    nodeCfg.pendSem     = ptrDatClient->cfg.pendSem;

    /* Initialize the JOSH node */
    ptrDatClient->joshHandle = Josh_initNode (&nodeCfg, errCode);
    if (ptrDatClient->joshHandle == NULL)
    {
        /* Error: JOSH Initialization failed. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

#ifdef __ARMv7
    /* Open Hplib , it will used to get hardware timestamp */
    ptrDatClient->hplibModFd = hplib_utilModOpen();
    if(ptrDatClient->hplibModFd == -1)
    {
        System_printf("Error: Open HPlib module failed with error: '%s'\n",  strerror(errno));

        *errCode = DAT_EINTERNAL;
        return -1;
    }
#endif

    /* Register the DAT services. */
    Dat_registerServices(ptrDatClient->joshHandle);

    /* Client was active so return the correct status. */
    return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to config dat client
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is to be registered
 *  @param[in]  ptrDatClientCfg
 *      Handle to DAT client configuration
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 * @sa Dat_configureClient
 *
 *  @retval
 *      0   - Client configuration is successful
 *  @retval
 *      <0  - C lient configuration Error
 */
int32_t Dat_configureClient
(
    Dat_ClientHandle         clientHandle,
    Dat_clientRuntimeCfg*    ptrDatClientCfg,
    int32_t*                 errCode
)
{
    Dat_ClientMCB*      ptrDatClient;

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* DAT client has been initialized. */
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
         *errCode = DAT_ENOTREADY;
        return -1;
    }

    switch(ptrDatClientCfg->cfgType)
    {

        case DAT_CONF_LOGSYNC:
#ifndef __ARMv7

            /* Check if a valid sync logger has been passed. If sync period is 0, then
             * no sync points are required, and hence no sync logger is required.
             * However, if a non-zero sync period has been passed, then a valid sync
             * logger should be present too. */
            if ( (ptrDatClientCfg->logSync.syncLogger == NULL) &&
                    (ptrDatClientCfg->logSync.syncPeriod > 0))
            {
                /* Invalid logger handle. */
                *errCode = DAT_EINVAL;
                return -1;
            }
#endif
            /* Disable logSync */
            LogSync_disable();

            /* Save the new log Sync configuration */
            ptrDatClient->cfg.logSync.syncLogger = ptrDatClientCfg->logSync.syncLogger;
            ptrDatClient->cfg.logSync.syncPeriod = ptrDatClientCfg->logSync.syncPeriod;

            /* Record the timestamp to send out sync messages. */
            LogSync_enable();
            DAT_TIMESTAMP(ptrDatClient->cfg.logSync.logSyncTimestamp);
            break;

#ifdef __ARMv7
        case DAT_CONF_GLOBAL_TSFREQ:
            /* 64bits freq in the format of two 32bits valuse */
            ptrDatClient->cfg.globalTsFreq.hi = ptrDatClientCfg->globalTsFreq.hi;
            ptrDatClient->cfg.globalTsFreq.lo = ptrDatClientCfg->globalTsFreq.lo;
            break;
#endif

        default:
            *errCode = DAT_EINVAL;
            return -1;

    }

    /* Dat client configuration is successful */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the DAT clients to check if the DAT
 *      server has been stopped or not.
 *
 *  @param[in]  databaseHandle
 *      Database handle in which the DAT server advertises the status
 *  @param[in]  clientHandle
 *      Name client handle which needs to specified if the DAT server and
 *      client are executing in different execution realms.
 *  @param[in]  serverName
 *      Name of the DAT server which is being checked
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      1       -   Server has been stopped
 *  @retval
 *      0       -   Server has NOT been stopped
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_isServerStopped
(
    Name_DBHandle       databaseHandle,
    Name_ClientHandle   clientHandle,
    char*               serverName,
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

    /* Do we have a valid name client specified? */
    if (clientHandle == NULL)
    {
        /* NO. Check the local database since the DAT server should have deleted the server name from the
         * local database. If the name is not found this indicates that the server has been stopped */
        if (Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               serverName, &namedResourceCfg, errCode) == 0)
        {
            /* DAT Server name exists in the local database so the server has not been stopped. */
            return 0;
        }

        /* Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
            return -1;

        /* DAT server name is not found so it has been stopped */
        return 1;
    }
    else
    {
        /* YES. Check the remote database since the DAT server should have deleted the server name in the
         * remote database */
        if (Name_get (clientHandle, Name_getDatabaseInstanceId (databaseHandle, errCode),
                      Name_ResourceBucket_INTERNAL_SYSLIB, serverName, &namedResourceCfg, errCode) == 0)
        {
            /* DAT Server exists in the remote database so the server has not been stopped */
            return 0;
        }

        /* Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
            return -1;

        /* DAT server name is not found so it has been stopped */
        return 1;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the DAT client. Once the DAT
 *      client has been stopped it can no longer be used for any other
 *      services. DAT client need to be stopped before they are deleted
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is to be stopped
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_stopClient
(
    Dat_ClientHandle    clientHandle,
    int32_t*            errCode
)
{
    Dat_ClientMCB*          ptrDatClient;
    char*                   ptrRemoteClientName;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: DAT clients can only be deleted if all the producers and consumers attached to it
     * have been deleted */
    if ((Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrProducerList) != NULL) ||
        (Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrConsumerList) != NULL))
    {
        *errCode = DAT_EINUSE;
        return -1;
    }
#ifdef __ARMv7
    /* Close HPLIB Module */
    if(ptrDatClient->hplibModFd)
    {
        hplib_utilModClose();
    }
#endif
    /* Mark the client status appropriately. */
    ptrDatClient->status = Dat_ClientStatus_INACTIVE;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_stopClient);
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
     *  int32_t _Dat_stopClient
     *  (
     *  Dat_ServerMCB*          ptrDatServer,
     *  char*                   clientName,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = strlen(ptrDatClient->cfg.clientName);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Populate the client name. */
    ptrRemoteClientName = (char*)args[1].argBuffer;
    strcpy (ptrRemoteClientName, ptrDatClient->cfg.clientName);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *(args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete the DAT client. DAT clients
 *      need to be stopped before a client can be deleted.
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is to be deleted.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @sa Dat_stopClient
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_deleteClient
(
    Dat_ClientHandle    clientHandle,
    int32_t*            errCode
)
{
    Dat_ClientMCB*          ptrDatClient;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];
    Dat_loggerInstId*       ptrLoggerId;
    uint32_t                loggerCount = 0;

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Clients can only be deleted once they have been stopped */
    if (ptrDatClient->status != Dat_ClientStatus_INACTIVE)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Shutdown the JOSH services between the DAT client and server. */
    if (Josh_deinitNode (ptrDatClient->joshHandle, errCode) < 0)
    {
        System_printf ("Error: JOSH Deinitialization for the DAT client failed [Error code %d] \n", *errCode);
        return -1;
    }

    /* Construct the channel name which is being deleted */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-CLNT", ptrDatClient->cfg.clientName);

    /* Close the MSGCOM channels which was used by the DAT client. */
    *errCode = Msgcom_delete (ptrDatClient->clientChannel, Dat_freePkt);
    if (*errCode < 0)
    {
        System_printf ("Error: DAT Client [MSGCOM channel] %p deletion failed [Error code %d] \n",
                       ptrDatClient->clientChannel, *errCode);
        return -1;
    }
    *errCode = Msgcom_delete (ptrDatClient->serverChannel, Dat_freePkt);
    if (*errCode < 0)
    {
        System_printf ("Error: DAT Client [Server MSGCOM channel] %p deletion failed [Error code %d] \n",
                       ptrDatClient->serverChannel, *errCode);
        return -1;
    }

    /* Do we need to announce this between the execution realms? */
    if (ptrDatClient->cfg.nameClientHandle)
    {
        /* YES. The MSGCOM client channel is no longer active and needs to be flushed off from the remote realm
         * database also. The DAT Client name is already removed from the server database when the DAT client is
         * deregistered at the server. */
        if (Name_push (ptrDatClient->cfg.nameClientHandle, channelName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, errCode) < 0)
        {
            System_printf ("Error: DAT Client [MSGCOM channel] '%s' PUSH between realms failed [Error code %d] \n",
                            channelName, *errCode);
            return -1;
        }
        System_printf ("Debug: DAT Client [MSGCOM channel] '%s' removed from the remote database\n", channelName);

        /* Delete the DAT client name from the remote database also. */
        if (Name_deleteResource(ptrDatClient->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                ptrDatClient->cfg.clientName, errCode) < 0)
        {
            System_printf ("Error: Deleting DAT client '%s' from remote database failed [Error code %d]\n", ptrDatClient->cfg.clientName, *errCode);
            return -1;
        }
    }

    /* Free the logger instance id */
    while (1)
    {
        /* Get logger Id from Free list and free the memory */
        ptrLoggerId = (Dat_loggerInstId *)Dat_listRemove((Dat_ListNode**)&ptrDatClient->ptrLoggerIdFreeList);
        if (ptrLoggerId == NULL)
            break;

        ptrDatClient->cfg.free(ptrLoggerId, sizeof(Dat_loggerInstId));
        loggerCount++;
    }

    /* Sanity check to see if all the instance are freed */
    if (loggerCount != DAT_CLIENT_MAX_DYNAMIC_INSTANCE)
    {
        System_printf("Error: Deleting DAT client logger instance id has incorrect number of logger instances: %d\n", loggerCount);
    }

#ifdef __ARMv7

    /* Cleanup the Global timeStamp - EMU counter resources */
    if (ptrDatClient->ptrUioMapInfo)
        Dat_UioDeinitMemMap(ptrDatClient, ptrDatClient->ptrUioMapInfo, errCode);

#endif
    /* Free the memory */
    ptrDatClient->cfg.free (ptrDatClient, sizeof(Dat_ClientMCB));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the DAT clients.
 *
 *  @param[in]  clientHandle
 *      Handle to the DAT client which is to be executed.
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Dat_executeClient(Dat_ClientHandle clientHandle)
{
    Dat_ClientMCB*    ptrDatClient;
    int32_t           errCode;

    Dat_LogSyncCfg*   ptrLogSync;

    /* Init timestamp. */
    DAT_TIMESTAMP_INIT(timeStamp);
    DAT_TIMESTAMP_INIT(timeElapsed);

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    /* Execute the JOSH services */
    if (Josh_receive(ptrDatClient->joshHandle, &errCode) < 0)
    {
        /* Error: JOSH failure; we could have failed because there was no message which is not
         * an error. So simply ignore that error code but for all other error code we display
         * the error. */
        if (errCode != JOSH_ENOMSG)
            System_printf ("Error: JOSH Receive for DAT client %p failed [Error code %d]\n", ptrDatClient, errCode);
    }

    /* Get the LogSync setting */
    ptrLogSync = &ptrDatClient->cfg.logSync;

    /* Record the current timestamp and Calculate the time elapsed for the multicore sync. */
    DAT_TIMESTAMP(timeStamp);
    DAT_TIME_ELAPSED(timeElapsed, timeStamp, ptrLogSync->logSyncTimestamp);

    /* Record a sync point if the sync time period has been exceeded. */
    if ((timeElapsed >= ptrLogSync->syncPeriod) &&  ptrLogSync->syncPeriod )
    {
#ifdef __ARMv7
        /* Write log sync event in ARM to main logger */
        LogSync_putSyncPoint();

        /* Flush the buffer */
        LoggerStreamer2_flush();
#else
        /* Write log sync event */
        LogSync_writeSyncPoint();

        /* Flush the buffer */
        LoggerStreamer2_flush ((LoggerStreamer2_Handle)ptrLogSync->syncLogger);
#endif
        /* Save the log sync time stamp */
        DAT_TIMESTAMP(ptrLogSync->logSyncTimestamp);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the DAT consumer.
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrConsumerCfg
 *      Pointer to the consumer configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   DAT consumer handle
 *  @retval
 *      Error   -   NULL
 */
Dat_ConsHandle Dat_createConsumer
(
    Dat_ClientHandle    clientHandle,
    Dat_ConsumerCfg*    ptrConsumerCfg,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Consumer*           ptrConsumer;
    Msgcom_ChannelCfg       chConfig;
    void*                   csHandle;
    char                    consumerChannelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrConsumerCfg == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments. */
    if(ptrConsumerCfg->heapHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return NULL;
    }

    /* Allocate memory for the consumer */
    ptrConsumer = (Dat_Consumer*)ptrDatClient->cfg.malloc (sizeof(Dat_Consumer), 0);
    if (ptrConsumer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_createConsumer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Dat_ServerConsHandle _Dat_createConsumer
     *  (
     *  Dat_ServerHandle    serverHandle,
     *  Dat_ClientHandle    clientHandle,
     *  int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Populate the DAT Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Was the consumer created succesfully on the server? */
    if ((Dat_ServerConsHandle)result == NULL)
    {
        /* Error: Consumer creation failed on the DAT Server */
        ptrDatClient->cfg.free (ptrConsumer, sizeof(Dat_Consumer));
        return NULL;
    }

    /* Populate the consumer block. */
    memcpy ((void*)&ptrConsumer->cfg, (void *)ptrConsumerCfg, sizeof(Dat_ConsumerCfg));
    ptrConsumer->isConnected            = 0;
    ptrConsumer->clientHandle           = clientHandle;
    ptrConsumer->serverConsumerHandle   = (Dat_ServerConsHandle)result;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Consumer MSGCOM channel:
     * - Queue DMA Non Blocking channel */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = ptrDatClient->cfg.msgcomInstHandle;
    chConfig.u.queueDMACfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum= (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrConsumerCfg->heapHandle)));

    /* Create the unique channel name */
    Dat_constructChannelName (ptrConsumerCfg->producerName, ptrConsumer, &consumerChannelName[0]);

    /* Create the consumer reader channel. */
    ptrConsumer->channelHandle = Msgcom_create (consumerChannelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, errCode);
    if (ptrConsumer->channelHandle == NULL)
        return NULL;

    /* Do we need to push the channel between the realms? */
    if (ptrDatClient->cfg.nameClientHandle != NULL)
    {
        if (Name_push (ptrDatClient->cfg.nameClientHandle, consumerChannelName,
                        Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_CREATE, errCode) < 0)
        {
            System_printf ("Error: Consumer Channel name '%s' PUSH to ARM realm failed [Error code %d] \n",
                           consumerChannelName, *errCode);
            return NULL;
        }
        System_printf ("Debug: Consumer channel name '%s' pushed successfully between realms\n", consumerChannelName);
    }

    /* Critical Section: Protect the consumer list  */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listAdd ((Dat_ListNode**)&ptrDatClient->ptrConsumerList, (Dat_ListNode*)ptrConsumer);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Return the consumer handle. */
    return (Dat_ConsHandle)ptrConsumer;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a logger id for loggerStreamer
 *
 *  @param[in]  ptrDatClient
 *      Pointer to DAT client Control block
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Logger Id handle
 *  @retval
 *      Error   -   NULL
 */
static Dat_loggerInstId*  Dat_allocLoggerId( Dat_ClientMCB* ptrDatClient)
{

    Dat_loggerInstId* ptrLoggerId;

    ptrLoggerId = (Dat_loggerInstId *)Dat_listRemove((Dat_ListNode**) &ptrDatClient->ptrLoggerIdFreeList);
    if(ptrLoggerId == NULL)
    {
        return NULL;
    }
    ptrLoggerId->status = DAT_LOGGER_ID_BUSY;

    return ptrLoggerId;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a logger id for loggerStreamer
 *
 *  @param[in]  ptrDatClient
 *      Pointer to DAT client Control block
 *  @param[in]  ptrLoggerId
 *      Pointer to loggerId to be freed
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t  Dat_freeLoggerId( Dat_ClientMCB* ptrDatClient, Dat_loggerInstId* ptrLoggerId)
{

    if(ptrLoggerId != NULL)
    {
        /* Free the logger Instance id */
        ptrLoggerId->status = DAT_LOGGER_ID_FREE;

        Dat_listAddTail((Dat_ListNode**)&ptrDatClient->ptrLoggerIdFreeList, (Dat_ListNode*)ptrLoggerId);

        return 0;
    }

    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a DAT producer
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrProducerCfg
 *      Pointer to the producer configuration
 *  @param[out] ptrLoggerHandle
 *      Pointer to save the dynamacially created Logger Handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   DAT producer handle
 *  @retval
 *      Error   -   NULL
 */
#ifdef __ARMv7
Dat_ProdHandle Dat_createProducer
(
    Dat_ClientHandle    clientHandle,
    Dat_ProducerCfg*    ptrProducerCfg,
    void**              ptrLoggerHandle,
    int32_t*            errCode
)
{
    Dat_ClientMCB*          ptrDatClient;
    Dat_Producer*           ptrProducer;
    uint8_t                 isAllocated;
    void*                   csHandle;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_producerInfo        producerInfo;
    Dat_producerInfo*       ptrRemoteProducerInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrProducerCfg == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if (ptrProducerCfg->heapHandle == NULL || ptrProducerCfg->loggerStreamerHandle != NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    if( (ptrProducerCfg->producerType != DAT_PRODUCER_UIA)  &&
        (ptrProducerCfg->producerType != DAT_PRODUCER_GENERAL_PURPOSE) )
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the Logger Streamer2 buffer size is compliant with the
     * heap data buffer size. */
    if ( (ptrProducerCfg->bufferSize > Pktlib_getMaxBufferSize(ptrProducerCfg->heapHandle)) ||
         (ptrProducerCfg->bufferSize == 0) )
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return NULL;
    }

    /* Allocate memory for the producer */
    ptrProducer = (Dat_Producer*)ptrDatClient->cfg.malloc (sizeof(Dat_Producer), 0);
    if (ptrProducer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrProducer, 0, sizeof(Dat_Producer));

    /* Open the pending queue per producer to keep track of all packets which have been exchanged
     * but not consumed. */
    ptrProducer->pendingQueueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                     QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (ptrProducer->pendingQueueHandle < 0)
    {
        /* Error: Producer pending channel creation failed. */
        ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));

        *errCode = DAT_ENOSPACE;
        return NULL;
    }

    /* Populate the producer block. */
    ptrProducer->clientHandle   = clientHandle;
    ptrProducer->numConsumers   = 0;

    /* Gerneral Purpose buffer needs extra bytes to hold the UIA packet/event header */
    if (ptrProducerCfg->producerType == DAT_PRODUCER_GENERAL_PURPOSE)
        ptrProducerCfg->bufferSize  += DAT_UIA_HEADER_OFFSET;

    /* Make buffer cache aligned */
    if (ptrProducerCfg->bufferSize % CACHE_L2_LINESIZE)
        ptrProducerCfg->bufferSize += (CACHE_L2_LINESIZE - ptrProducerCfg->bufferSize % CACHE_L2_LINESIZE);

    /* Create logger streamer instance and init buffers */
    {
        Ti_Pkt*                 ptrPkt;
        Qmss_QueueHnd           tmpQueueHandle;
        uint8_t*                ptrDataBuffer;
        uint32_t                dataBufferSize;

        if(ptrProducerCfg->producerType == DAT_PRODUCER_UIA)
        {
            ptrProducer->loggerFunc = Dat_LoggerFuncUIA;
        }
        else if (ptrProducerCfg->producerType == DAT_PRODUCER_GENERAL_PURPOSE)
        {
            ptrProducer->loggerFunc = Dat_LoggerFuncGP;
        }

        /* Cycle through all the packets in the producer heap and ensure that the buffers are
         * initialized properly. Open a temporary queue to store these packets temporarily. */
        tmpQueueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                        QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (tmpQueueHandle < 0)
        {
            *errCode = DAT_ENOSPACE;
            return NULL;
        }

        /* Allocate Instance id from free pool */
        ptrProducer->ptrLoggerId = Dat_allocLoggerId (ptrDatClient);
        if(ptrProducer->ptrLoggerId == NULL)
        {
            *errCode = DAT_ENOSPACE;
            return NULL;
        }

        /* Allocate all the packets from the producer heap. */
        while (1)
        {
            ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle,
                                         ptrProducerCfg->heapHandle,
                                         ptrProducerCfg->bufferSize);
            if (ptrPkt == NULL)
                break;

            /* Get the data buffer and initialize the buffers for the logger streamer  */
            Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

            /* Producer buffer initialization */
            ptrProducer->loggerFunc.initBuffer((void *)ptrProducerCfg,
                                               ptrDataBuffer,
                                               ptrProducerCfg->crcApp16,
                                               ptrProducer->ptrLoggerId->loggerInstId);

            /* Write back the buffer header */
            ptrDatClient->cfg.endMemAccess(ptrDataBuffer, ptrProducerCfg->bufferSize);

            /* Push the initialized packet into the temporary queue */
            Qmss_queuePushDesc (tmpQueueHandle, (void*)ptrPkt);
        }

        /* Now free all the packets */
        while (1)
        {
            /* Take the packet from the temporary queue */
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(tmpQueueHandle));
            if (ptrPkt == NULL)
                break;

            /* Free the packet */
            Pktlib_freePacket (ptrDatClient->cfg.pktlibInstHandle, ptrPkt);
        }

        /* Close the temporary queue */
        Qmss_queueClose (tmpQueueHandle);

        /* Allocate the first packet from the producer heap */
        ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducerCfg->heapHandle,
                                      ptrProducerCfg->bufferSize);
        if (ptrPkt == NULL)
        {
            *errCode = DAT_EINTERNAL;
            return NULL;
        }

        /* Get the data buffer associated with the */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

        /* Remember the packet in production. */
        ptrProducer->ptrProductionPkt = ptrPkt;

        if(ptrDatClient->loggerStreamerInitialized == 0)
        {
            /* Create logger Streamer instance */
            LoggerStreamer2_init(ptrProducerCfg->crcApp16, getpid());

            /* Set the flag to avoid mulitple init */
            ptrDatClient->loggerStreamerInitialized = 1;
        }

        /* Create LoggerStreamer Instance */
        ptrProducerCfg->loggerStreamerHandle = LoggerStreamer2_create((UInt32*)ptrDataBuffer,
                                                                      ptrProducerCfg->bufferSize,
                                                                      (xdc_SizeT)ptrProducerCfg->bufferSize,
                                                                      ptrProducerCfg->isMainLogger,
                                                                      (UArg)ptrProducer,
                                                                      ptrProducer->ptrLoggerId->loggerInstId,
                                                                      0, 0   );

        if(ptrProducerCfg->loggerStreamerHandle == NULL)
        {
            System_printf("Error: Creating loggerStreamer2 instance FAILED. \n");

            /* Error: Producer pending channel creation failed. */
            ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));

            *errCode = DAT_EINTERNAL;
            return NULL;
        }

        /* Get the logger instance id and verify it */
        if ( LoggerStreamer2_getLoggerId (ptrProducerCfg->loggerStreamerHandle) !=
             ptrProducer->ptrLoggerId->loggerInstId)
        {
            *errCode = DAT_EINTERNAL;
            return NULL;
        }
        /* Save logger handle and mark the logger Owner */
        if(ptrLoggerHandle != NULL)
        {
            *ptrLoggerHandle = ptrProducerCfg->loggerStreamerHandle;
        }

        ptrProducer->loggerStreamerOwner    = 1;

    }

    /* Populate the producer configuration. */
    memcpy ((void*)&ptrProducer->cfg, (void *)ptrProducerCfg, sizeof(Dat_ProducerCfg));

    /* Prepare for the remote call to create producer */
    memset((void *)&producerInfo, 0, sizeof(Dat_producerInfo));
    strncpy (producerInfo.name, ptrProducerCfg->name, DAT_MAX_CHAR);
    producerInfo.realm = ptrDatClient->cfg.realm;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_createProducer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Dat_createProducer
     *  (
     *  Dat_ServerHandle    serverHandle,
     *  Dat_ClientHandle    clientHandle,
     *  Dat_producerInfo*   producerInfo,
     *  int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_producerInfo);;

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

    /* Populate the producer information */
    ptrRemoteProducerInfo = ( Dat_producerInfo*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteProducerInfo, (void*)&producerInfo, sizeof(Dat_producerInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Was the producer created succesfully on the server? */
    if ((int32_t)result < 0)
    {
        /* Error: Producer creation failed on the DAT Server */
        ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));
        return NULL;
    }

    /* Populate the producer block. */
    //memcpy ((void*)&ptrProducer->cfg, (void *)ptrProducerCfg, sizeof(Dat_ProducerCfg));
    ptrProducer->clientHandle   = clientHandle;
    ptrProducer->numConsumers   = 0;

    /* Critical Section: Protect the access to the producer list. */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listAdd ((Dat_ListNode**)&ptrDatClient->ptrProducerList, (Dat_ListNode*)ptrProducer);
    ptrDatClient->cfg.exitCS(csHandle);


    /* Return the producer handle */
    return (Dat_ProdHandle)ptrProducer;
}
#else

Dat_ProdHandle Dat_createProducer
(
    Dat_ClientHandle    clientHandle,
    Dat_ProducerCfg*    ptrProducerCfg,
    void**              ptrLoggerHandle,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Producer*           ptrProducer;
    uint8_t                 isAllocated;
    void*                   csHandle;
    Dat_producerInfo        producerInfo;
    Dat_producerInfo*       ptrRemoteProducerInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrProducerCfg == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if (ptrProducerCfg->heapHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return NULL;
    }

    /* Allocate memory for the producer */
    ptrProducer = (Dat_Producer*)ptrDatClient->cfg.malloc (sizeof(Dat_Producer), 0);
    if (ptrProducer == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrProducer, 0, sizeof(Dat_Producer));

    /* Sanity Check: Logger Streamer object can only be associated with a single producer. */
    if (ptrProducerCfg->loggerStreamerHandle != NULL)
    {
        /* Is there already a producer associated? */
        if (LoggerStreamer2_getContext(ptrProducerCfg->loggerStreamerHandle) != NULL)
        {
            *errCode = DAT_EINUSE;
            return NULL;
        }

        /* Sanity Check: Ensure that the Logger Streamer2 buffer size is compliant with the
         * heap data buffer size. */
        if ((uint32_t)LoggerStreamer2_getBufSize(ptrProducerCfg->loggerStreamerHandle) >
            Pktlib_getMaxBufferSize(ptrProducerCfg->heapHandle))
        {
            *errCode = DAT_EINVAL;
            return NULL;
        }

        ptrProducer->loggerStreamerOwner  = 0;
    }
    else
    {
        /* Logger streamer instance is not provided, create one */
        LoggerStreamer2_Params params;
        Error_Block eb;

        /* Initialize loggerStreamer prarms structure */
        LoggerStreamer2_Params_init(&params);
        params.bufSize = ptrProducerCfg->bufferSize;

        /* Gerneral Purpose buffer needs extra bytes to hold the UIA packet/event header */
        if (ptrProducerCfg->producerType == DAT_PRODUCER_GENERAL_PURPOSE)
            params.bufSize  += DAT_UIA_HEADER_OFFSET;

        /* Make buffer cache aligned */
        if (params.bufSize % CACHE_L2_LINESIZE)
            params.bufSize += (CACHE_L2_LINESIZE - params.bufSize % CACHE_L2_LINESIZE);

        /* Sanity Check: Ensure that the Logger Streamer2 buffer size is compliant with the
         * heap data buffer size. */
        if ( (params.bufSize > Pktlib_getMaxBufferSize(ptrProducerCfg->heapHandle)) ||
             (ptrProducerCfg->bufferSize == 0) )
        {
            *errCode = DAT_EINVAL;
            return NULL;
        }

        /* Allocate Instance id from free pool */
        ptrProducer->ptrLoggerId = Dat_allocLoggerId (ptrDatClient);
        if(ptrProducer->ptrLoggerId == NULL)
        {
            *errCode = DAT_ENOSPACE;
            return NULL;
        }

        /* Allocation is successful, save the loggerInstandId */
        params.instanceId = ptrProducer->ptrLoggerId->loggerInstId;

        if(ptrProducerCfg->producerType == DAT_PRODUCER_UIA)
        {
            params.exchangeFxn = (ti_uia_sysbios_LoggerStreamer2_ExchangeFxnType ) &Dat_exchangeFunction;
        }
        else if (ptrProducerCfg->producerType == DAT_PRODUCER_GENERAL_PURPOSE)
        {
            params.exchangeFxn = (ti_uia_sysbios_LoggerStreamer2_ExchangeFxnType ) &Dat_bufferExchangeGP;
        }
        else
        {
            *errCode = DAT_EINVAL;
            return NULL;
        }

        /* Create loggerStreamer2 instance */
        ptrProducerCfg->loggerStreamerHandle = LoggerStreamer2_create(&params, &eb);
        if(ptrProducerCfg->loggerStreamerHandle == NULL)
        {
            *errCode = DAT_EINTERNAL;
            return NULL;
        }

        /* Enable loggerStreamer instance */
        LoggerStreamer2_enable(ptrProducerCfg->loggerStreamerHandle);

        /* Save logger handle and mark the logger Owner */
        if(ptrLoggerHandle != NULL)
            *ptrLoggerHandle = ptrProducerCfg->loggerStreamerHandle;

        ptrProducer->loggerStreamerOwner = 1 ;
    }

    /* Open the pending queue per producer to keep track of all packets which have been exchanged
     * but not consumed. */
    ptrProducer->pendingQueueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                     QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (ptrProducer->pendingQueueHandle < 0)
    {
        /* Error: Producer pending channel creation failed. */
        ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));

        /* Delete loggerStreamer Instance */
        LoggerStreamer2_disable( ptrProducerCfg->loggerStreamerHandle );
        LoggerStreamer2_delete( ptrProducerCfg->loggerStreamerHandle );
        return NULL;
    }

    /* Prepare for the remote call to create producer */
    memset((void *)&producerInfo, 0, sizeof(Dat_producerInfo));
    strncpy (producerInfo.name, ptrProducerCfg->name, DAT_MAX_CHAR);
    producerInfo.realm = ptrDatClient->cfg.realm;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_createProducer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Dat_createProducer
     *  (
     *  Dat_ServerHandle    serverHandle,
     *  Dat_ClientHandle    clientHandle,
     *  Dat_producerInfo*   producerInfo,
     *  int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_producerInfo);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

    /* Populate the producer information */
    ptrRemoteProducerInfo = (Dat_producerInfo*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteProducerInfo, (void*)&producerInfo, sizeof(Dat_producerInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Was the producer created succesfully on the server? */
    if ((int32_t)result < 0)
    {
        /* Error: Producer creation failed on the DAT Server */
        ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));
        return NULL;
    }

    /* Populate the producer block. */
    memcpy ((void*)&ptrProducer->cfg, (void *)ptrProducerCfg, sizeof(Dat_ProducerCfg));
    ptrProducer->clientHandle   = clientHandle;
    ptrProducer->numConsumers   = 0;

    /* Associate the logger streamer object with the producer.
     * - Setup the prime function to ensure that the logger streamer has at least
     *   an initial buffer to get started upon. */
    if (ptrProducerCfg->loggerStreamerHandle != NULL)
    {
        Ti_Pkt*                 ptrPkt;
        Qmss_QueueHnd           tmpQueueHandle;
        uint8_t*                ptrDataBuffer;
        uint32_t                dataBufferSize;
        uint16_t                instanceId;

        if(ptrProducerCfg->producerType == DAT_PRODUCER_UIA)
        {
            ptrProducer->loggerFunc = Dat_LoggerFuncUIA;
        }
        else if (ptrProducerCfg->producerType == DAT_PRODUCER_GENERAL_PURPOSE)
        {
            ptrProducer->loggerFunc = Dat_LoggerFuncGP;
        }
        else
        {
            *errCode = DAT_EINVAL;
            return NULL;
        }

        /* Associate the producer context with the logger streamer */
        LoggerStreamer2_setContext(ptrProducerCfg->loggerStreamerHandle, (xdc_UArg)ptrProducer);

        /* Cycle through all the packets in the producer heap and ensure that the buffers are
         * initialized properly. Open a temporary queue to store these packets temporarily. */
        tmpQueueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                        QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (tmpQueueHandle < 0)
        {
            *errCode = DAT_ENOSPACE;
            return NULL;
        }

        /* Get logger Streamer instance id */
        instanceId = LoggerStreamer2_getInstanceId(ptrProducerCfg->loggerStreamerHandle);

        /* Allocate all the packets from the producer heap. */
        while (1)
        {
            ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle,
                                         ptrProducer->cfg.heapHandle,
                                         LoggerStreamer2_getBufSize(ptrProducerCfg->loggerStreamerHandle));
            if (ptrPkt == NULL)
                break;

            /* Get the data buffer and initialize the buffers for the logger streamer  */
            Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

            /* Producer buffer initialization */
            ptrProducer->loggerFunc.initBuffer(ptrProducerCfg->loggerStreamerHandle,
                                               ptrDataBuffer,
                                               ptrDatClient->cfg.id,
                                               instanceId);

            /* Write back the buffer header */
            ptrDatClient->cfg.endMemAccess(ptrDataBuffer, LoggerStreamer2_getBufSize(ptrProducerCfg->loggerStreamerHandle) );

            /* Push the initialized packet into the temporary queue */
            Qmss_queuePushDesc (tmpQueueHandle, (void*)ptrPkt);
        }

        /* Now free all the packets */
        while (1)
        {
            /* Take the packet from the temporary queue */
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(tmpQueueHandle));
            if (ptrPkt == NULL)
                break;

            /* Free the packet */
            Pktlib_freePacket (ptrDatClient->cfg.pktlibInstHandle, ptrPkt);
        }

        /* Close the temporary queue */
        Qmss_queueClose (tmpQueueHandle);

        /* Calling Prime function for the logger */
        //LoggerStreamer2_setContext(ptrProducerCfg->loggerStreamerHandle, (xdc_UArg)ptrProducer);
        ptrProducer->loggerFunc.prime(ptrProducerCfg->loggerStreamerHandle);
    }

    /* Critical Section: Protect the access to the producer list. */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listAdd ((Dat_ListNode**)&ptrDatClient->ptrProducerList, (Dat_ListNode*)ptrProducer);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Return the producer handle */
    return (Dat_ProdHandle)ptrProducer;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to connect a consumer with the producer whose
 *      name was specified during consumer creation. Consumers can be connected
 *      only once the producer has been created.
 *
 *  @param[in]  consumerHandle
 *      DAT consumer handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_connectConsumer
(
    Dat_ConsHandle      consumerHandle,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Consumer*           ptrConsumer;
    Dat_ConnectInfo         connectInfo;
    Dat_ConnectInfo*        ptrRemoteConnectInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if (consumerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the consumer MCB */
    ptrConsumer = (Dat_Consumer*)consumerHandle;

    /* Sanity Check: Consumers can be connected only once and are already in use by the producer */
    if (ptrConsumer->isConnected == 1)
    {
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrConsumer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_connectConsumer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Initialize the connect information */
    memset ((void*)&connectInfo, 0, sizeof(Dat_ConnectInfo));

    /* Populate the connect information: */
    connectInfo.serverConsumerHandle = ptrConsumer->serverConsumerHandle;
    strcpy(connectInfo.producerName, ptrConsumer->cfg.producerName);

    /* Create the unique channel name */
    Dat_constructChannelName (ptrConsumer->cfg.producerName, ptrConsumer, &connectInfo.consumerChannelName[0]);

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Dat_connectConsumer
     *  (
     *  Dat_ServerHandle        serverHandle,
     *  Dat_ConnectInfo*        ptrConnectInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_ConnectInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Populate the connect information */
    ptrRemoteConnectInfo = (Dat_ConnectInfo*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteConnectInfo, (void*)&connectInfo, sizeof(Dat_ConnectInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Validate if the consumer connect was successful? */
    if ((int32_t)result == 0)
        ptrConsumer->isConnected = 1;
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is an internal function which is invoked by the DAT
 *      server on the processing entity which executes the DAT producer
 *      and is invoked to indicate that a consumer has connected to it
 *
 *  @param[in]   clientHandle
 *      DAT client handle on which the producer executes
 *  @param[in]   ptrConnectInfo
 *      Pointer to the connect information
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
int32_t __Dat_connectConsumer
(
    Dat_ClientHandle    clientHandle,
    Dat_ConnectInfo*    ptrConnectInfo,
    int32_t*            errCode
)
{
    Dat_ClientMCB*      ptrDatClient;
    Dat_Producer*       ptrProducer;
    MsgCom_ChHandle     chHandle;
    int32_t             index;

    /* Get the pointer to the DAT client */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    /* Get the pointer to the producer */
    ptrProducer = (Dat_Producer*)Dat_findProducer(clientHandle, ptrConnectInfo->producerName);
    if (ptrProducer == NULL)
    {
        /* Error: The producer does not exist in the DAT Client realm. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* The MSGCOM channel should have been created by now. */
    chHandle = Msgcom_find (ptrDatClient->cfg.msgcomInstHandle, ptrConnectInfo->consumerChannelName, errCode);
    if (chHandle == NULL)
    {
        uint32_t            nameDatabaseId;
        Name_ResourceCfg    nameResourceCfg;

        /* Get name database id */
        nameDatabaseId   = Name_getDatabaseInstanceId(ptrDatClient->cfg.databaseHandle, errCode);

        /* Get the name resource from other realm */
        if (Name_get (ptrDatClient->cfg.nameClientHandle, nameDatabaseId, Name_ResourceBucket_INTERNAL_SYSLIB,
                      ptrConnectInfo->consumerChannelName, &nameResourceCfg, errCode) < 0)
        {
            System_printf ("Error: Remote get for channel '%s' failed. [Error code %d]\n", ptrConnectInfo->consumerChannelName, *errCode);
            return -1;
        }

        /* Found the name in remote database, create it in local database. */
        if (Name_createResource (ptrDatClient->cfg.databaseHandle,
                                 Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, errCode) < 0)
        {
            System_printf ("Error: Unable to create the name in database '%s' failed. [Error code %d]\n", ptrConnectInfo->consumerChannelName, *errCode);
            return -1;
        }

        /* Try to find Msgcom channel again */
        chHandle = Msgcom_find (ptrDatClient->cfg.msgcomInstHandle, ptrConnectInfo->consumerChannelName, errCode);
        if (chHandle == NULL)
        {
            System_printf ("Error: Unable to find msgcom channnel '%s' [Error code %d]\n", ptrConnectInfo->consumerChannelName, *errCode);
            return -1;
        }
    }
    /* Find a free space in the producer to insert the MSGCOM channel handle */
    for (index = 0; index < DAT_MAX_CONSUMER; index++)
    {
        /* Is the slot free? */
        if (ptrProducer->consumerChHandles[index] == NULL)
        {
            /* Register the consumer with the DAT client producer */
            ptrProducer->numConsumers++;
            strcpy (ptrProducer->consumerChannelNames[index], ptrConnectInfo->consumerChannelName);
            ptrProducer->consumerChHandles[index] = chHandle;
            return 0;
        }
    }

    /* Shutdown the MSGCOM channel since the connect failed because there was no space */
    Msgcom_delete (chHandle, Dat_freePkt);
    *errCode = DAT_ENOSPACE;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is an internal function which is invoked by the DAT
 *      server on the processing entity which executes the DAT producer
 *      and is invoked to indicate that a consumer has disconnected
 *      from it
 *
 *  @param[in]   clientHandle
 *      DAT client handle on which the producer executes
 *  @param[in]   ptrConnectInfo
 *      Pointer to the connect information
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
int32_t __Dat_disconnectConsumer
(
    Dat_ClientHandle    clientHandle,
    Dat_ConnectInfo*    ptrConnectInfo,
    int32_t*            errCode
)
{
    Dat_Producer*       ptrProducer;
    int32_t             index;

    /* Get the pointer to the producer */
    ptrProducer = (Dat_Producer*)Dat_findProducer(clientHandle, ptrConnectInfo->producerName);
    if (ptrProducer == NULL)
    {
        /* Error: The producer does not exist in the DAT Client realm. */
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Decrement the number of consumers */
    ptrProducer->numConsumers--;

    /* Delete the entry */
    for (index = 0; index < DAT_MAX_CONSUMER; index++)
    {
        /* Is the slot free? */
        if (ptrProducer->consumerChHandles[index] == NULL)
            continue;

        /* Is this what we are looking for? */
        if (strcmp (ptrProducer->consumerChannelNames[index], ptrConnectInfo->consumerChannelName) != 0)
            continue;

        /* Delete the entry */
        Msgcom_delete (ptrProducer->consumerChHandles[index], Dat_freePkt);

        /* Mark the slot free */
        ptrProducer->consumerChHandles[index]       = NULL;
        ptrProducer->consumerChannelNames[index][0] = 0;
        return 0;
    }

    /* Control comes here implies that the producer database does not have any entry matching the
     * consumer channel name */
    *errCode = DAT_EINTERNAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to disconnect a consumer from a producer. Consumers
 *      can only be deleted once they have been disconnected. Once a consumer has
 *      been disconnected it will not receive any more messages from the producer
 *
 *  @param[in]  consumerHandle
 *      DAT consumer handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_disconnectConsumer (Dat_ConsHandle consumerHandle, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Consumer*           ptrConsumer;
    Dat_ConnectInfo         connectInfo;
    Dat_ConnectInfo*        ptrRemoteConnectInfo;

    /* Sanity Check: Ensure that the arguments are valid */
    if (consumerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the consumer MCB */
    ptrConsumer = (Dat_Consumer*)consumerHandle;

    /* Sanity Check: Consumers need to be connected for them to disconnect. */
    if (ptrConsumer->isConnected == 0)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrConsumer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_disconnectConsumer);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Initialize the connect information */
    memset ((void*)&connectInfo, 0, sizeof(Dat_ConnectInfo));

    /* Populate the connect information: */
    connectInfo.serverConsumerHandle = ptrConsumer->serverConsumerHandle;
    strcpy(connectInfo.producerName, ptrConsumer->cfg.producerName);

    /* Create the unique channel name */
    Dat_constructChannelName (ptrConsumer->cfg.producerName, ptrConsumer, &connectInfo.consumerChannelName[0]);

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Dat_disconnectConsumer
     *  (
     *  Dat_ServerHandle        serverHandle,
     *  Dat_ConnectInfo*        ptrConnectInfo,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_ConnectInfo);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Populate the connect information */
    ptrRemoteConnectInfo = (Dat_ConnectInfo*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteConnectInfo, (void*)&connectInfo, sizeof(Dat_ConnectInfo));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Validate if the consumer disconnect was successful? */
    if ((int32_t)result == 0)
        ptrConsumer->isConnected = 0;
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the consumer. Consumers can only be
 *      deleted once they have been disconnected from the producer
 *
 *  @sa Dat_disconnectConsumer
 *
 *  @param[in]  consumerHandle
 *      DAT consumer handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deleteConsumer (Dat_ConsHandle consumerHandle, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Consumer*           ptrConsumer;
    void*                   csHandle;
    char                    consumerChannelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Sanity Check: Ensure that the arguments are valid */
    if (consumerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the consumer & DAT client block. */
    ptrConsumer  = (Dat_Consumer*)consumerHandle;
    ptrDatClient = (Dat_ClientMCB*)ptrConsumer->clientHandle;

    /* Is the consumer connected? Consumers need to be disconnected before they are deleted. */
    if (ptrConsumer->isConnected == 1)
    {
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_deleteConsumer);
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
     *  int32_t _Dat_deleteConsumer
     *  (
     *  Dat_ServerHandle        serverHandle,
     *  Dat_ServerConsHandle    serverConsumerHandle,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ServerConsHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the DAT Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ptrConsumer->serverConsumerHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Was the consumer created succesfully on the server? */
    if ((int32_t)result < 0)
        return -1;

    /* Create the unique channel name */
    Dat_constructChannelName (ptrConsumer->cfg.producerName, ptrConsumer, &consumerChannelName[0]);

    /* Delete the consumer reader channel. */
    Msgcom_delete (ptrConsumer->channelHandle, Dat_freePkt);

    /* Do we need to push the channel between the realms? */
    if (ptrDatClient->cfg.nameClientHandle != NULL)
    {
        if (Name_push (ptrDatClient->cfg.nameClientHandle, consumerChannelName,
                       Name_ResourceBucket_INTERNAL_SYSLIB, Name_ResourceOperationType_DELETE, errCode) < 0)
        {
            System_printf ("Error: Consumer Channel name '%s' PUSH to ARM realm failed [Error code %d] \n",
                           consumerChannelName, *errCode);
            return -1;
        }
        System_printf ("Debug: Consumer channel name '%s' pushed successfully between realms\n", consumerChannelName);
    }

    /* Critical Section: Protect the consumer list */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatClient->ptrConsumerList, (Dat_ListNode*)ptrConsumer);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Cleanup memory for the consumer */
    ptrDatClient->cfg.free (ptrConsumer, sizeof(Dat_Consumer));
    return 0;
}

#ifdef __ARMv7
/**
 *  @b Description
 *  @n
 *      The function is used to delete the producer. Producers can only be deleted
 *      once all the consumers connected to the producer have been disconnected.
 *
 *  @param[in]  producerHandle
 *      DAT producer handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deleteProducer (Dat_ProdHandle producerHandle, int32_t* errCode)
{
    Dat_ClientMCB*          ptrDatClient;
    Dat_Producer*           ptrProducer;
    void*                   csHandle;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    char*                   ptrRemoteProducerName;

    /* Sanity Check: Validate the arguments */
    if (producerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT producer */
    ptrProducer = (Dat_Producer*)producerHandle;

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Do we have consumers connected to the producer? */
    if (ptrProducer->numConsumers > 0)
    {
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_deleteProducer);
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
     *  int32_t _Dat_deleteProducer
     *  (
     *  Dat_ServerHandle    serverHandle,
     *  char*               ptrProducerName,
     *  int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = DAT_MAX_CHAR;

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Copy the producer name */
    ptrRemoteProducerName = (char*)args[1].argBuffer;
    strncpy (ptrRemoteProducerName, ptrProducer->cfg.name, DAT_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Was the producer created succesfully on the server? */
    if ((int32_t)result < 0)
        return -1;

    /* Delete the logger streamer instance created during producer creation */
    if( (ptrProducer->loggerStreamerOwner ) && (ptrProducer->cfg.loggerStreamerHandle != NULL))
    {
        uint16_t     loggerInstId;

        /* Disable loggerStreamer instance */
        LoggerFlexSupport_disable(ptrProducer->cfg.loggerStreamerHandle );

        /* Get the logger instance id and free it */
        loggerInstId = LoggerStreamer2_getLoggerId ((LoggerStreamer2_Handle)ptrProducer->cfg.loggerStreamerHandle);

        if (loggerInstId == ptrProducer->ptrLoggerId->loggerInstId)
        {
            /* Free the logger Instance id */
            if(Dat_freeLoggerId(ptrDatClient, ptrProducer->ptrLoggerId) < 0)
            {
                *errCode = DAT_EINTERNAL;
                return -1;
            }
            ptrProducer->ptrLoggerId = NULL;
        }
        else
        {
            *errCode = DAT_EINTERNAL;
            return -1;
        }

        /* Delete loggerStreamer instance */
        LoggerStreamer2_delete((LoggerStreamer2_Handle *)&ptrProducer->cfg.loggerStreamerHandle);
    }

    /* Critical Section: Protect the producer list */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatClient->ptrProducerList, (Dat_ListNode*)ptrProducer);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Cleanup the producer memory. */
    ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));
    return 0;
}
#else
/**
 *  @b Description
 *  @n
 *      The function is used to delete the producer. Producers can only be deleted
 *      once all the consumers connected to the producer have been disconnected.
 *
 *  @param[in]  producerHandle
 *      DAT producer handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deleteProducer (Dat_ProdHandle producerHandle, int32_t* errCode)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Producer*           ptrProducer;
    void*                   csHandle;
    char*                   ptrRemoteProducerName;

    /* Sanity Check: Validate the arguments */
    if (producerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT producer */
    ptrProducer = (Dat_Producer*)producerHandle;

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Do we have consumers connected to the producer? */
    if (ptrProducer->numConsumers > 0)
    {
        *errCode = DAT_EINUSE;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_deleteProducer);
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
     *  int32_t _Dat_deleteProducer
     *  (
     *  Dat_ServerHandle    serverHandle,
     *  char*               ptrProducerName,
     *  int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = DAT_MAX_CHAR;

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Copy the producer name */
    ptrRemoteProducerName = (char*)args[1].argBuffer;
    strncpy (ptrRemoteProducerName, ptrProducer->cfg.name, DAT_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Was the producer created succesfully on the server? */
    if ((int32_t)result < 0)
        return -1;

    /* Unset the context in the logger streamer. */
    LoggerStreamer2_setContext(ptrProducer->cfg.loggerStreamerHandle, NULL);

    /* Delete the logger streamer instance created during producer creation */
    if( (ptrProducer->loggerStreamerOwner ) && (ptrProducer->cfg.loggerStreamerHandle != NULL))
    {
        uint16_t     loggerInstId;

        /* Disable loggerStreamer instance */
        LoggerStreamer2_disable(ptrProducer->cfg.loggerStreamerHandle );

        /* Get the logger instance id and free it */
        loggerInstId = LoggerStreamer2_getInstanceId ((LoggerStreamer2_Handle)ptrProducer->cfg.loggerStreamerHandle);

        if (loggerInstId == ptrProducer->ptrLoggerId->loggerInstId)
        {
            /* Free the logger Instance id */
            if(Dat_freeLoggerId(ptrDatClient, ptrProducer->ptrLoggerId) < 0)
            {
                *errCode = DAT_EINTERNAL;
                return -1;
            }
            ptrProducer->ptrLoggerId = NULL;
        }
        else
        {
            *errCode = DAT_EINTERNAL;
            return -1;
        }
        /* Delete loggerStreamer instance */
        LoggerStreamer2_delete((LoggerStreamer2_Handle *)&ptrProducer->cfg.loggerStreamerHandle);
    }

    /* Critical Section: Protect the producer list */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatClient->ptrProducerList, (Dat_ListNode*)ptrProducer);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Cleanup the producer memory. */
    ptrDatClient->cfg.free (ptrProducer, sizeof(Dat_Producer));
    return 0;
}
#endif



/**
 *  @b Description
 *  @n
 *      The function is used to get the statistics from the producer.
 *
 *  @param[in]  producerHandle
 *      DAT producer handle
 *  @param[out]  ptrProducerStats
 *      Pointer to the producer statistics
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getProducerStats
(
    Dat_ProdHandle      producerHandle,
    Dat_ProducerStats*  ptrProducerStats,
    int32_t*            errCode
)
{
    Dat_Producer*   ptrProducer;

    /* Sanity Check: Validate the arguments */
    if ((producerHandle == NULL) || (ptrProducerStats == NULL))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT producer */
    ptrProducer = (Dat_Producer*)producerHandle;

    /* Copy over the producer statistics */
    memcpy ((void*)ptrProducerStats, (void*)&ptrProducer->stats, sizeof(Dat_ProducerStats));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the status of the consumer. Consumers can be in
 *      either of the following states:
 *      - Connected
 *          Consumers is connected to a producer and is operational
 *      - Pending connection
 *          Consumer is in process of getting connected to a producer
 *      - Disconnected
 *          Consumer has been disconnected from the producer and is now unattached
 *      - Pending disconnection
 *          Consumer is in process of getting disconnected from a producer
 *
 *  @param[in]  consumerHandle
 *      Consumer handle
 *  @param[out] consumerStatus
 *      Consumer status
 *  @param[out] errCode
 *      Error code populated on the error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getConsumerStatus
(
    Dat_ConsHandle      consumerHandle,
    Dat_ConsumerStatus* consumerStatus,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    Dat_Consumer*           ptrConsumer;

    /* Sanity Check: Validate the arguments */
    if (consumerHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the pointer to the DAT consumer */
    ptrConsumer = (Dat_Consumer*)consumerHandle;

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrConsumer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getConsumerStatus);
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
     *  int32_t _Dat_getConsumerStatus
     *  (
     *  Dat_ServerHandle        serverHandle,
     *  Dat_ServerConsHandle    serverConsumerHandle,
     *  Dat_ConsumerStatus*     consumerStatus,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ServerConsHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_ConsumerStatus);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ptrConsumer->serverConsumerHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the output values */
    *consumerStatus = *((Dat_ConsumerStatus*)args[2].argBuffer);
    *errCode        = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to flush the current producer buffer
 *
 *  @param[in]  ptrProducer
 *      Pointer to the DAT producer
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */

static void Dat_flushProducerBuffer(Dat_Producer* ptrProducer)
{
#ifdef __ARMv7
    LoggerStreamer2_iflush((Ptr)ptrProducer->cfg.loggerStreamerHandle);
#else
    LoggerStreamer2_flush ((LoggerStreamer2_Handle)ptrProducer->cfg.loggerStreamerHandle);
#endif

}

/**
 *  @b Description
 *  @n
 *      The function is used to send messages by the producer to the
 *      consumer.
 *
 *  @param[in]  ptrProducer
 *      Pointer to the DAT producer
 *  @param[in]  ptrDatClient
 *      Pointer to the DAT client
 *  @param[in]  ptrMessage
 *      Pointer to the message
 *  @param[in]  numConsumers
 *      Number of consumers to which the message is to be sent.
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t Dat_sendToConsumers
(
    Dat_Producer*     ptrProducer,
    Dat_ClientMCB*    ptrDatClient,
    Ti_Pkt*           ptrMessage,
    int32_t           numConsumers
)
{
    int32_t             errCode;
    int32_t             index;
    Ti_Pkt*             ptrConsumerMessage[DAT_MAX_CONSUMER + 2];
    int32_t             consumerCount;
    Pktlib_InstHandle   pktlibInstHandle;

    /* Get the PKTLIB instance handle. */
    pktlibInstHandle = ptrDatClient->cfg.pktlibInstHandle;

    /* Allocate zero buffer packets for the cloning. This is done upfront because if the heaps dont have
     * sufficient packets in the heap we simply drop the entire message. */
    for (consumerCount = 0; consumerCount < (numConsumers - 1); consumerCount++)
    {
        /* Allocate a zero buffer packet. */
        ptrConsumerMessage[consumerCount] = Pktlib_allocPacket (pktlibInstHandle, ptrProducer->cfg.heapHandle, 0);
        if (ptrConsumerMessage[consumerCount] == NULL)
        {
            /* Error: Unable to allocate the packet. */
            ptrProducer->stats.allocFailures++;

            /* Cleanup all the zero buffer packets which had been allocated till now. */
            for (index = 0; index < consumerCount; index++)
                Pktlib_freePacket (pktlibInstHandle, ptrConsumerMessage[index]);
            return -1;
        }
    }

    /* Zero buffer packets have been allocated. We now clone the packets. The last consumer
     * will always get the original packet. */
    for (consumerCount = 0; consumerCount < (numConsumers - 1); consumerCount++)
        Pktlib_clonePacket (pktlibInstHandle, ptrMessage, ptrConsumerMessage[consumerCount]);

    /* The last consumer will always get the original message */
    ptrConsumerMessage[consumerCount] = ptrMessage;

    /* Send all the messages which need to be sent to the connected consumers.
     * - We know the number of connected consumers that are attached to the producer */
    consumerCount = 0;

    /* Cycle through all the connected consumers and send the messages to each of them. */
    for (index = 0; index < DAT_MAX_CONSUMER; index++)
    {
        /* Do we have a valid connected consumer? If so send the message to the consumer */
        if (ptrProducer->consumerChHandles[index] != NULL)
            Msgcom_putMessage (ptrProducer->consumerChHandles[index], ptrConsumerMessage[consumerCount++]);
    }

    /* Do we need to send a message to Memory ? */
    if(ptrProducer->cfg.memlogChanHandle)
    {
        Memlog_saveLog(ptrProducer->cfg.memlogChanHandle, ptrConsumerMessage[consumerCount++]);
    }

    /* Do we need to send a message to the debug stream also? */
    if (ptrProducer->cfg.debugSocketHandle != NULL)
    {
        if (Netfp_send (ptrProducer->cfg.debugSocketHandle, ptrConsumerMessage[consumerCount], 0, &errCode) < 0)
        {
            /* Error: Unable to send out the packet. */
            ptrProducer->stats.debugStreamingError++;
            return -1;
        }
        else
        {
            ptrProducer->stats.debugStreaming++;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the background actions on the DAT
 *      producer.
 *
 *  @param[in]  ptrDatClient
 *      Pointer to the DAT Client
 *  @param[in]  ptrProducer
 *      Pointer to the DAT producer
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Dat_executeProducerBackground
(
    Dat_ClientMCB*    ptrDatClient,
    Dat_Producer*     ptrProducer
)
{
    uint8_t*          ptrDataBuffer;
    uint32_t          dataBufferLen;
    Ti_Pkt*           ptrMessage;
    uint32_t          numConsumers;

    /* Get the number of consumers (including the debug streaming) */
    numConsumers = ptrProducer->numConsumers + ((ptrProducer->cfg.debugSocketHandle == NULL) ? 0 : 1);

    /* Check producer memory logging settings */
    if ( ptrProducer->cfg.memlogChanHandle != NULL )
        numConsumers++;

    /* Consume all the messages for each producer. */
    while (1)
    {
        /* Get the pending message */
        ptrMessage = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrProducer->pendingQueueHandle));
        if (ptrMessage == NULL)
            break;

        /* Pending message: Are there any consumers? */
        if (numConsumers == 0)
        {
            /* No consumers: Packet dropped. */
            ptrProducer->stats.noConsumers++;
            Pktlib_freePacket (ptrDatClient->cfg.pktlibInstHandle, ptrMessage);
            continue;
        }

        /* Consumers were present. We need to writeback the contents of the data buffer
         * since the buffer is going to be consumed. */
        Pktlib_getDataBuffer(ptrMessage, &ptrDataBuffer, &dataBufferLen);
        ptrDatClient->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

        /* Send the message to all the consumers (including debug streaming) */
        if (Dat_sendToConsumers (ptrProducer, ptrDatClient, ptrMessage, numConsumers) < 0)
        {
            /* Error: Unable to send the message to the consumers. Clean the message and suspend
             * operation on this producer till next time. */
            Pktlib_freePacket (ptrDatClient->cfg.pktlibInstHandle, ptrMessage);
            break;
        }
    }

    /* Execute garbage collection; since packets are cloned and will end up in the garbage
     * collection after CPDMA transmission. */
    Pktlib_garbageCollection (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to prcess messages for a consumer
 *      consumer.
 *
 *  @param[in]  consumerHandle
 *      Pointer to the DAT consumer
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
Ti_Pkt* Dat_processConsumer
(
    Dat_ConsHandle    consumerHandle
)
{
    Ti_Pkt*           ptrMessage;
    Dat_Consumer*     ptrConsumer;

    ptrConsumer = (Dat_Consumer*)consumerHandle;

    /* Sanity check */
    if(consumerHandle == NULL)
        return NULL;

    /* Only connected consumers are handled */
    if (ptrConsumer->isConnected == 0)
        return NULL;

    /* Get a message on the consumer channel */
    Msgcom_getMessage((MsgCom_ChHandle)ptrConsumer->channelHandle, (MsgCom_Buffer**)&ptrMessage);

    return ptrMessage;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the background actions on the DAT
 *      clients.
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Not applicable.
 */
void Dat_executeBackgroundActions (Dat_ClientHandle clientHandle)
{
    Dat_ClientMCB*    ptrDatClient;
    Dat_Producer*     ptrProducer;
    void*             csHandle;

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
        return;

    /* Critical Section: Protect the producer & consumer lists. */
    csHandle = ptrDatClient->cfg.enterCS();

    /* Cycle through all the producers on the specific DAT client and send out all the pending messages */
    ptrProducer = (Dat_Producer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrProducerList);
    while (ptrProducer != NULL)
    {
        /* Send all the pending messages for each producer. */
        Dat_executeProducerBackground (ptrDatClient, ptrProducer);

        /* Get the next producer */
        ptrProducer = (Dat_Producer*)Dat_listGetNext ((Dat_ListNode*)ptrProducer);
    }

    /* Release the critical section. */
    ptrDatClient->cfg.exitCS(csHandle);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is applicable only for general-purpose producers.  It has
 *      to be called only once and returns the first buffer of the ring for the
 *      application to use.  Subsequently, @ref Dat_bufferExchange is used to
 *      get next buffers.
 *
 *  @param[in]  producerHandle
 *      Handle to the producer
 *  @param[in]  ProducerName
 *      Name of the producer
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to buffer
 *  @retval
 *      Error   -   NULL
 */
uint8_t* Dat_getProducerBuffer
(
    Dat_ProdHandle      producerHandle,
    char*               ProducerName,
    int32_t*            errCode
)
{
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;
    Dat_Producer*   ptrProducer;

    /* Sanity Check: Ensure that the parameters are valid. */
    if ((producerHandle == NULL) || (ProducerName == NULL))
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Get the pointer to the DAT producer */
    ptrProducer = (Dat_Producer*)producerHandle;

    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Sanity check: name match */
    if(strcmp (ptrProducer->cfg.name, ProducerName) != 0)
    {
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle,
                                 ptrProducer->cfg.bufferSize);
    if (ptrPkt == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Get the data buffer associated with the */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Remember the packet in production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Return buffer address with Header offset */
    return (ptrDataBuffer + DAT_UIA_HEADER_OFFSET);
}

/**
 *  @b Description
 *  @n
 *      The function closes the active buffer and returns a new buffer.
 *      If the consumer is slow, the same buffer will be returned again.
 *      This is applicable only for general-purpose producers.
 *
 *  @param[in]  clientHandle
 *      DAT client handle.
 *  @param[in]  producerHandle
 *      DAT producer handle.
 *  @param[out] oldBuffer
 *      Active buffer that has to be closed
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to new buffer
 *  @retval
 *      Error   -   NULL. This occurs if an invalid producer handle
 *                        or NULL buffer is passed.
 */
uint8_t* Dat_bufferExchange
(
    Dat_ClientHandle   clientHandle,
    Dat_ProdHandle     producerHandle,
    uint8_t*           oldBuffer
    )
    {
    Dat_Producer*       ptrProducer;
    void*               newBuffer = NULL;
    Dat_ClientMCB*      ptrDatClient;

    /* Sanity Check: Validate the arguments */
    if ((clientHandle == NULL) || (producerHandle == NULL) || (oldBuffer == NULL))
    {
        return NULL;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    /* Get the pointer to the DAT producer */
    ptrProducer = (Dat_Producer*)producerHandle;

    ptrProducer = Dat_findProducer(ptrDatClient, ptrProducer->cfg.name);
    if( (ptrProducer !=NULL) && ((Dat_ProdHandle)ptrProducer == producerHandle))
        newBuffer = ptrProducer->loggerFunc.bufExchange(ptrProducer->cfg.loggerStreamerHandle, oldBuffer);

    return((uint8_t*)newBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function gets the custom data offset of a Gernernal Purpose producer by Producer realm
 *  This offset is after UIA packet header and event header. The offset is different when
 *  producer is created on DSP or ARM.
 *
 *  @param[in]  realm
 *      DAT producer realm
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Offset to the the custom data
 *  @retval
 *      Error   -   =0.
 */
uint32_t Dat_getGPProducerDataOffsetByRealm
(
    Dat_ExecutionRealm realm,
    int32_t*           errCode
)
{
    uint32_t    dataOffset=0;

    if(realm == Dat_ExecutionRealm_DSP)
        dataOffset = sizeof(UIAPacket_Hdr) + DAT_UIA_EVT_HEADER_SIZE;

    else if (realm == Dat_ExecutionRealm_ARM)        
        dataOffset = DAT_CUIA_PKT_HEADER_SIZE + DAT_UIA_EVT_HEADER_SIZE;

    else
    {
        /* Error: Invalid realm  */
        *errCode = DAT_EINTERNAL;
    }
        
    return dataOffset;
}

/**
 *  @b Description
 *  @n
 *      The function gets the custom data offset of a Gernernal Purpose producer
 *  This offset is after UIA packet header and event header. The offset is different when
 *  producer is created on DSP or ARM.
 *
 *  @param[in]  consumerHandle
 *      DAT consumer client handle.
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Offset to the the custom data
 *  @retval
 *      Error   -   =0.
 */
uint32_t Dat_getGPProducerDataOffset
(
    Dat_ConsHandle     consumerHandle,
    int32_t*           errCode
)
{
    Josh_Argument       args[JOSH_MAX_ARGS];
    Josh_JobHandle      jobHandle;
    Josh_ArgHandle      argHandle;
    uint32_t            result;
    int32_t             jobId;
    Dat_Consumer*       ptrConsumer;
    Dat_ClientMCB*      ptrDatClient;
    uint32_t            offset;

    /* Sanity Check: Validate the arguments */
    if ((consumerHandle == NULL))
    {
        *errCode = DAT_EINVAL;
        return 0;
    }

    /* Get the consumer MCB */
    ptrConsumer = (Dat_Consumer*)consumerHandle;

    /* Sanity Check: Consumers can be connected only once and are already in use by the producer */
    if (ptrConsumer->isConnected != 1)
    {
        *errCode = DAT_ENOTREADY;
        return 0;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)ptrConsumer->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return 0;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getGPProdDataOffset);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return 0;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Dat_getGPProdDataOffset
     *  (
     *  Dat_ServerHandle        serverHandle,
     *  Dat_ServerConsHandle    serverConsumerHandle,
     *  uint32_t*               dataOffset,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/
    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ServerConsHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(uint32_t);

    /*  - Argument 3: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return 0;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ptrConsumer->serverConsumerHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return 0;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return 0;
    }

    /* Copy the returned data offset value. */
    offset = *((uint32_t*)args[2].argBuffer);

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return 0;
    }

    /* Validate if the consumer connect was successful? */
    if ((int32_t)result == 0)
        return offset;

    /* Return error from server, return 0 as offset */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function can be called from an exception handler to flush
 *      buffers for all producers associated with this DAT client.
 *
 *  @param[in]  clientHandle
 *      DAT client handle to flush all producer buffer.
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Dat_flushAllBuffers (Dat_ClientHandle clientHandle)
{
    Dat_ClientMCB*    ptrDatClient;
    Dat_Producer*     ptrProducer;
    void*             csHandle;

    /* Get the DAT Client MCB */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient == NULL)
        return;

    /* Critical Section: Protect the producer & consumer lists. */
    csHandle = ptrDatClient->cfg.enterCS();

    /* Cycle through all the producers on the specific DAT client and send out all the pending messages */
    ptrProducer = (Dat_Producer*)Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrProducerList);
    while (ptrProducer != NULL)
    {
        /* Flush the current buffer used by Logger */
        Dat_flushProducerBuffer (ptrProducer);

        /* Send all the pending messages for each producer. */
        Dat_executeProducerBackground (ptrDatClient, ptrProducer);

        /* Get the next producer */
        ptrProducer = (Dat_Producer*)Dat_listGetNext ((Dat_ListNode*)ptrProducer);
    }

    /* Release the critical section. */
    ptrDatClient->cfg.exitCS(csHandle);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find the global trace object on the client matching the
 *      trace object name
 *
 *  @param[in]  ptrDatClient
 *      Pointer to the DAT Client
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
static Dat_LocalTraceObject* Dat_findClientTraceObjectInst
(
    Dat_ClientMCB*      ptrDatClient,
    char*               traceObjectName
)
{
    Dat_LocalTraceObject* ptrTraceObjInst;

    /* Cycle through all the trace object which exist on the DAT server and find the trace object
     * which matches the name */
    ptrTraceObjInst = (Dat_LocalTraceObject*)Dat_listGetHead ((Dat_ListNode**)&ptrDatClient->ptrLocalTraceObjList);
    while (ptrTraceObjInst != NULL)
    {
        /* Do we have a match? */
        if (strncmp (ptrTraceObjInst->traceObject.name, traceObjectName, DAT_MAX_CHAR) == 0)
            return ptrTraceObjInst;

        /* Get the next trace object. */
        ptrTraceObjInst = (Dat_LocalTraceObject*)Dat_listGetNext ((Dat_ListNode*)ptrTraceObjInst);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the verbosity levels and register the trace
 *      object on DAT server. Multiple trace objects can be created per DAT
 *      instance.
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  ptrTraceCfg
 *      Pointer to the configuration of the trace object that has to be created.
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_createTraceObject
(
    Dat_ClientHandle    clientHandle,
    Dat_TraceObjectCfg* ptrTraceCfg,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    uint32_t                componentArraySize;
    Dat_ClientMCB*          ptrDatClient;
    Dat_traceCompBlock      compBlock;
    Dat_TraceCompInfo*      ptrComponentArray;
    uint32_t                index;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (clientHandle == NULL)
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }

    if ( (ptrTraceCfg == NULL) || (ptrTraceCfg->traceObjectName == NULL) ||
         (ptrTraceCfg->numTraceComponents > 0 && ptrTraceCfg->ptrComponentArray == NULL) )
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }


    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Register the global trace object with DAT server */
    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_createTraceObject);
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
     *  Dat_ServerTraceObjHandle _Dat_createTraceObject
     *  (
     *      Dat_ServerHandle    serverHandle,
     *      Dat_ClientHandle    clientHandle,
     *      Dat_TraceObjectCfg* ptrTraceObjCfg,
     *      int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_TraceObjectCfg);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

    /* Copy the trace cfg */
    memcpy (args[2].argBuffer, (uint8_t *)ptrTraceCfg, sizeof(Dat_TraceObjectCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }
    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* TODO: Find out the max size can be used to pass component array */
    //blockSize = Pktlib_getMaxBufferSize(ptrDatClient->cfg.clientHeapHandle);

    memset((void *)&compBlock, 0, sizeof(Dat_traceCompBlock));
    strncpy(compBlock.name, ptrTraceCfg->traceObjectName, DAT_MAX_CHAR);
    compBlock.blockSize = DAT_COMPONENT_INIT_BLOCKSIZE;
    compBlock.blockOffset = 0;

    /* Calculate the size of the component array */
    componentArraySize = sizeof(Dat_TraceCompInfo) * ptrTraceCfg->numTraceComponents;

    /* Allocate temporary memory and save the component array */
    ptrComponentArray = (Dat_TraceCompInfo*)ptrDatClient->cfg.mallocLocal(
                                            componentArraySize, 0);
    if(ptrComponentArray == NULL)
    {
        *errCode = DAT_ENOMEM;
        return -1;
    }

    /* Convert component configurations to the format stored in system */
    for (index = 0; index < ptrTraceCfg->numTraceComponents; index++)
    {
        Dat_TraceComponentCfg* ptrComponentCfg = &ptrTraceCfg->ptrComponentArray[index];

        /* Copy the name and default levels. */
        strncpy (ptrComponentArray[index].componentName, ptrComponentCfg->componentName, DAT_TRACE_COMPONENT_MAX_CHAR);
        ptrComponentArray[index].logLevel = (ptrComponentCfg->focusedLevel << DAT_MASK_SIZE) |
                                          ptrComponentCfg->nonFocusedLevel;
    }

    /* Copy component initial configuration to DAT server */
    while(componentArraySize > 0)
    {
        if (componentArraySize <= DAT_COMPONENT_INIT_BLOCKSIZE)
        {
            compBlock.blockSize = componentArraySize;
            compBlock.lastBlock = 1;
        }
        /* Pass the initial verbosity levels to DAT server */
        jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_initTraceComponentBlock);
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
         *  int32_t _Dat_initTraceComponent
         *  (
         *      Dat_ServerHandle    serverHandle,
         *      Dat_ClientHandle    clientHandle,
         *      Dat_TraceComponentCfg*  ptrComponentArray,
         *      int32_t*            errCode
         *  )
         ****************************************************************************/

        /* Populate the arguments.
         * - Argument 1: */
        args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[0].length = sizeof(Dat_ServerHandle);

        /*  - Argument 2: */
        args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[1].length = sizeof(Dat_ClientHandle);

        /*  - Argument 3: */
        args[2].type   = Josh_ArgumentType_PASS_BY_REF;
        args[2].length = sizeof(Dat_traceCompBlock) + compBlock.blockSize;

        /*  - Argument 4: */
        args[3].type   = Josh_ArgumentType_PASS_BY_REF;
        args[3].length = sizeof(int32_t);

        /* Add the arguments. */
        argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
        if (argHandle == NULL)
        {
            *errCode = DAT_EJOSH;
            return -1;
        }

        /* Populate the arguments*/
        *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
        *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

        /* Copy the component info for this transfer */
        memcpy ((uint8_t *)(args[2].argBuffer), (uint8_t *)&compBlock, sizeof(Dat_traceCompBlock));

        /* Copy the component array  */
        memcpy ((uint8_t *)(args[2].argBuffer + sizeof(Dat_traceCompBlock)),
                (uint8_t *)ptrComponentArray + compBlock.blockOffset,
                compBlock.blockSize);

        /* Submit the JOB to JOSH for execution. */
        jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
        if (jobId < 0)
        {
            *errCode = DAT_EJOSH;
            return -1;
        }
        /* Get the result arguments. */
        if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
        {
            *errCode = DAT_EINTERNAL;
            return -1;
        }

        /* Copy the error code value. */
        *errCode = *((int32_t*)args[3].argBuffer);

        /* Free the JOB Instance. */
        if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
        {
            *errCode = DAT_EJOSH;
            return -1;
        }
        /* Was the trace object initialized succesfully on the server? */
        if ((int32_t)result < 0)
            return -1;

        /* Anymore data from component array? */
        componentArraySize -= compBlock.blockSize;
        compBlock.blockOffset += compBlock.blockSize;
    }

    /* Free the temporary memory */
    ptrDatClient->cfg.freeLocal(ptrComponentArray, sizeof(Dat_TraceCompInfo) * ptrTraceCfg->numTraceComponents);

    /* Debug Message: */
    System_printf ("Debug: Trace object %s has been created on DAT client %p\n", ptrTraceCfg->traceObjectName, clientHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and creates a local trace object
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  name
 *      DAT trace object name
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the trace object instance handle
 *  @retval
 *      Error   -   NULL
 */
Dat_TraceObjHandle Dat_createTraceObjectInstance
(
    Dat_ClientHandle    clientHandle,
    char*               name,
    int32_t*            errCode
)
{
    Dat_ClientMCB*          ptrDatClient;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_LocalTraceObject*   ptrLocalTraceObj;
    Dat_TraceObjectBody*    ptrServerTraceObj;
    void*                   csHandle;
    Dat_traceCompBlock      compBlock;
    uint32_t                componentArraySize;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (clientHandle == NULL || name == NULL )
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return NULL;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return NULL;
    }

    /* Register the local trace object with DAT server */
    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_createTraceObjectInstance);
    if (jobHandle == NULL)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * Dat_ServerLocalTraceObjHandle _Dat_createTraceObjectInstance
     * (
     *     Dat_ServerHandle    serverHandle,
     *     Dat_ClientHandle    clientHandle,
     *     Dat_TraceObjectBody* ptrTraceCfg,
     *     int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_TraceObjectBody);

    /*  - Argument 5: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

    /* Copy the trace object name */
    memcpy (args[2].argBuffer, (uint8_t *)name, DAT_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if ( jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }
    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return NULL;
    }

    /* Was the local trace object created succesfully on the server? */
    if ((Dat_ServerLocalTraceObjHandle)result == NULL)
    {
        /* Error: local trace object creation failed on the DAT Server */
        return NULL;
    }

    /* Allocate memory for the Local trace object */
    ptrLocalTraceObj = (Dat_LocalTraceObject*)ptrDatClient->cfg.mallocLocal (sizeof(Dat_LocalTraceObject), 0);
    if (ptrLocalTraceObj == NULL)
    {
        *errCode = DAT_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrLocalTraceObj, 0, sizeof(Dat_LocalTraceObject));

    /* Create the copy of trace object locally, it is preferable the copy is saved in local memory */
    ptrServerTraceObj = (Dat_TraceObjectBody *)args[2].argBuffer;
    memcpy((uint8_t *)&(ptrLocalTraceObj->traceObject), (uint8_t *)ptrServerTraceObj, sizeof(Dat_TraceObjectBody));
    ptrLocalTraceObj->serverLocalTraceObj = (Dat_ServerLocalTraceObjHandle)result;

    /* Allocate and save the component array */
    ptrLocalTraceObj->traceObject.ptrComponentArray =
               (Dat_TraceCompInfo*)ptrDatClient->cfg.mallocLocal (ptrServerTraceObj->traceObjectSize, 0);
    if(ptrLocalTraceObj->traceObject.ptrComponentArray == NULL)
    {
        *errCode = DAT_ENOMEM;
        ptrDatClient->cfg.freeLocal ((void *)ptrLocalTraceObj, sizeof(Dat_LocalTraceObject));
        return NULL;
    }

    /* Get trace components from Server */
    memset((void *)&compBlock, 0, sizeof(Dat_traceCompBlock));
    strncpy(compBlock.name, name, DAT_MAX_CHAR);
    compBlock.blockSize = DAT_COMPONENT_INIT_BLOCKSIZE;
    compBlock.blockOffset = 0;

    /* Calculate the size of the component array */
    componentArraySize = ptrLocalTraceObj->traceObject.traceObjectSize;

    while(componentArraySize>0)
    {
        Dat_TraceCompInfo*  ptrServerCompArray;
        Dat_TraceCompInfo*  ptrClientCompArray;
        Dat_traceCompBlock* ptrCompBlock;

        if (componentArraySize <= DAT_COMPONENT_INIT_BLOCKSIZE)
        {
            compBlock.blockSize = componentArraySize;
            compBlock.lastBlock = 1;
        }

        /* Pass the initial verbosity levels to DAT server */
        jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getTraceComponentBlock);
        if (jobHandle == NULL)
        {
            *errCode = DAT_EINTERNAL;
            return NULL;
        }

        /* Initialize the arguments; to avoid any junk */
        memset ((void *)&args, 0, sizeof(args));

        /***************************************************************************
         * This is the function which is to be invoked:
         *
         *  int32_t _Dat_initTraceComponent
         *  (
         *      Dat_ServerHandle    serverHandle,
         *      Dat_ClientHandle    clientHandle,
         *      Dat_traceCompBlock* ptrCompBlock,
         *      int32_t*            errCode
         *  )
         ****************************************************************************/

        /* Populate the arguments.
         * - Argument 1: */
        args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[0].length = sizeof(Dat_ServerHandle);

        /*  - Argument 2: */
        args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[1].length = sizeof(Dat_ClientHandle);

        /*  - Argument 3: */
        args[2].type   = Josh_ArgumentType_PASS_BY_REF;
        args[2].length = sizeof(Dat_traceCompBlock) + compBlock.blockSize;

        /*  - Argument 4: */
        args[3].type   = Josh_ArgumentType_PASS_BY_REF;
        args[3].length = sizeof(int32_t);

        /* Add the arguments. */
        argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
        if (argHandle == NULL)
        {
            *errCode = DAT_EJOSH;
            return NULL;
        }

        /* Populate the arguments*/
        *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
        *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(clientHandle);

        /* Copy the component info for this transfer */
        memcpy ((uint8_t *)(args[2].argBuffer), (uint8_t *)&compBlock, sizeof(Dat_traceCompBlock));

        /* Submit the JOB to JOSH for execution. */
        jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
        if (jobId < 0)
        {
            *errCode = DAT_EJOSH;
            return NULL;
        }
        /* Get the result arguments. */
        if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
        {
            *errCode = DAT_EINTERNAL;
            return NULL;
        }

        /* Copy the error code value. */
        *errCode = *((int32_t*)args[3].argBuffer);

        /* Free the JOB Instance. */
        if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
        {
            *errCode = DAT_EJOSH;
            return NULL;
        }
        /* Was the trace object initialized succesfully on the server? */
        if ((int32_t)result < 0)
            return NULL;

        /* Copy the verbosity levels to local component array */
        ptrCompBlock = (Dat_traceCompBlock*)args[2].argBuffer;
        ptrClientCompArray = (Dat_TraceCompInfo*)((uint32_t)&ptrLocalTraceObj->traceObject.ptrComponentArray[0] + compBlock.blockOffset);
        ptrServerCompArray = (Dat_TraceCompInfo*)((uint32_t)ptrCompBlock + sizeof(Dat_traceCompBlock));

        memcpy(ptrClientCompArray, ptrServerCompArray, compBlock.blockSize);

        /* Anymore data from component array? */
        componentArraySize -= compBlock.blockSize;
        compBlock.blockOffset += compBlock.blockSize;
    }

    /* Save client and server trace object handles */
    ptrLocalTraceObj->clientHandle = clientHandle;

    /* Add Global trace object in DAT client */
    /* Critical Section: Protect the access to the list. */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listAdd ((Dat_ListNode**)&ptrDatClient->ptrLocalTraceObjList, (Dat_ListNode*)ptrLocalTraceObj);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Debug Message: */
    System_printf ("Debug: Local trace object %s (%p) has been created on DAT client %p\n",
                    name, ptrLocalTraceObj, clientHandle);

    return (Dat_TraceObjHandle)ptrLocalTraceObj;
}

/**
 *  @b Description
 *  @n
 *      The function is the API to delete a trace object instance
 *
 *  @param[in]  localTracehandle
 *      DAT local trace object client handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deleteTraceObjectInstance
(
    Dat_TraceObjHandle    localTracehandle,
    int32_t*              errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Dat_ClientMCB*          ptrDatClient;
    void*                   csHandle;
    Dat_LocalTraceObject*   ptrLocalTraceObj;

    /* Sanity Check: Ensure that the arguments are valid */
    if (localTracehandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the local trace object & DAT client. */
    ptrLocalTraceObj  = (Dat_LocalTraceObject *)localTracehandle;
    ptrDatClient = (Dat_ClientMCB*)ptrLocalTraceObj->clientHandle;

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_deleteTraceObjectInstance);
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
     * int32_t _Dat_deleteTraceObjectInstance
     * (
     *     Dat_ServerHandle    serverHandle,
     *     Dat_ClientHandle    clientHandle,
     *     char*               traceObjectName,
     *     int32_t*            errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = DAT_MAX_CHAR;

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the DAT Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient);

    /* Copy the trace object name */
    memcpy (args[2].argBuffer, (uint8_t *)ptrLocalTraceObj->traceObject.name, DAT_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Was the trace object deleted succesfully on the server? */
    if ((int32_t)result < 0)
        return -1;

    /* Critical Section: Protect the local trace object list */
    csHandle = ptrDatClient->cfg.enterCS();
    Dat_listRemoveNode ((Dat_ListNode**)&ptrDatClient->ptrLocalTraceObjList, (Dat_ListNode*)ptrLocalTraceObj);
    ptrDatClient->cfg.exitCS(csHandle);

    /* Cleanup memory for the trace object */
    if(ptrLocalTraceObj->traceObject.ptrComponentArray)
       ptrDatClient->cfg.freeLocal (ptrLocalTraceObj->traceObject.ptrComponentArray, ptrLocalTraceObj->traceObject.traceObjectSize);

    ptrDatClient->cfg.freeLocal (ptrLocalTraceObj, sizeof(Dat_LocalTraceObject));

    /* Debug Message: */
    System_printf ("Debug: Local trace object %p has been deleted on DAT client %p\n", localTracehandle, ptrDatClient);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered service which executes on the DAT
 *      server and deletes a global trace object
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  traceObjName
 *      DAT Trace Object name
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_deleteTraceObject
(
    Dat_ClientHandle    clientHandle,
    char*               traceObjName,
    int32_t*            errCode
)
{
    Josh_Argument       args[JOSH_MAX_ARGS];
    Josh_JobHandle      jobHandle;
    Josh_ArgHandle      argHandle;
    uint32_t            result;
    int32_t             jobId;
    Dat_ClientMCB*      ptrDatClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the global trace object & DAT client. */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_deleteTraceObject);
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
     *  int32_t _Dat_deleteTraceObject
     *  (
     *      Dat_ServerHandle    serverHandle,
     *      char*               traceObjectName,
     *      int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = DAT_MAX_CHAR;

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the DAT Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    strncpy((char *)args[1].argBuffer, traceObjName, DAT_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Was the trace object deleted succesfully on the server? */
    if ((int32_t)result < 0)
    {
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: Global trace object %s has been deleted on DAT client %p\n", traceObjName, ptrDatClient);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the used to find a global trace object and use the returned
 *   handle to query/modify trace objects.
 *
 *  @param[in]  traceObjectHandle
 *      DAT Trace Object instance handle
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the Trace Object body
 *  @retval
 *      Error   -   NULL
 */
Dat_TraceObjectBody* Dat_getTraceObjectBody
(
    Dat_TraceObjHandle    traceObjectHandle
)
{
    Dat_LocalTraceObject*    ptrLocalTraceObj;

    /* Sanity Check: Ensure that the arguments are valid */
    if (traceObjectHandle == NULL)
    {
        return NULL;
    }

    /* Get the local trace object  */
    ptrLocalTraceObj  = (Dat_LocalTraceObject *)traceObjectHandle;

    return &ptrLocalTraceObj->traceObject;
}

/**
 *  @b Description
 *  @n
 *      The function modifies the verbosity levels.
 *
 *  @param[in]  localTracehandle
 *      DAT client handle
 *  @param[in]  ptrVerbosityCfg
 *      Pointer to the verbosity configuration.
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_modifyVerbosity
(
    Dat_TraceObjHandle      localTracehandle,
    Dat_VerbosityCfg*       ptrVerbosityCfg,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ClientMCB*          ptrDatClient;
    Dat_LocalTraceObject*   ptrLocalTraceObj;
    uint32_t                result;
    int32_t                 jobId;
    Dat_TraceCompInfo*      ptrTraceObject;
    uint32_t                componentId;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (ptrVerbosityCfg == NULL || localTracehandle == NULL)
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the parameters are valid. */
    if (ptrVerbosityCfg->traceObjectName == NULL)
    {
        /* Invalid Params */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrLocalTraceObj     = (Dat_LocalTraceObject*)localTracehandle;
    ptrDatClient         = (Dat_ClientMCB*)ptrLocalTraceObj->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_modifyVerbosity);
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
     * int32_t _Dat_modifyVerbosity
     * (
     *     Dat_ServerHandle    serverHandle,
     *     Dat_VerbosityCfg*   ptrVerbosityCfg,
     *     int32_t*            errCode
     * )
     ****************************************************************************/
    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Dat_ClientHandle);

    /*  - Argument 2: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Dat_VerbosityCfg);

    /*  - Argument 3: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(uint32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(ptrLocalTraceObj->clientHandle);

    /* Copy the producer name */
    memcpy ((uint8_t *)args[2].argBuffer, (uint8_t *)ptrVerbosityCfg, sizeof(Dat_VerbosityCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return -1;
    }
    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    if((int32_t)result < 0)
    {
        return -1;
    }

    /* Save the configuration locally */
    /* Create the bit mask for the new verbosity level. */
    if (((ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Component) &&
        (ptrVerbosityCfg->isComponentFocused)) ||
        (ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Class))

        ptrVerbosityCfg->verbosityLevel <<= DAT_MASK_SIZE;

    /* Offset the TC ID if it is component type. Else, use the class ID to
     * index into the trace object array. */
    if (ptrVerbosityCfg->verbosityOpt == Dat_Verbosity_Component)
        componentId = ptrVerbosityCfg->traceComponentId + DAT_COMPONENT_START_ID;
    else
        componentId = DAT_CLASS_ID;

    /* Modify the verbosity level. */
    ptrTraceObject = ptrLocalTraceObj->traceObject.ptrComponentArray;

    if (ptrVerbosityCfg->isEnabled)
        ptrTraceObject[componentId].logLevel |=  ptrVerbosityCfg->verbosityLevel;
    else
        ptrTraceObject[componentId].logLevel &=  ~ptrVerbosityCfg->verbosityLevel;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is typically called through the Agent to determine the
 *      number of trace components belonging to a trace object.
 *
 *  @param[in]  localTracehandle
 *      DAT trace object handle
 *  @param[in]  traceObjectName
 *      Name of the trace object.
 *  @param[out] numComponents
 *      Number of components, returned by this function
 *  @param[out] errCode
 *      Error code returned by the function
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getNumComponents
(
    Dat_TraceObjHandle  localTracehandle,
    char*               traceObjectName,
    uint32_t*           numComponents,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ClientMCB*          ptrDatClient;
    Dat_LocalTraceObject*   ptrLocalTraceObj;
    uint32_t                result;
    int32_t                 jobId;
    Dat_verbosityQueryCfg   queryCfg;

    /* Sanity Check: Ensure that the parameters are valid. */
    if (localTracehandle == NULL || traceObjectName == NULL || numComponents == NULL)
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrLocalTraceObj     = (Dat_LocalTraceObject*)localTracehandle;
    ptrDatClient         = (Dat_ClientMCB*)ptrLocalTraceObj->clientHandle;

    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Set up query configuration */
    strncpy(queryCfg.name, traceObjectName, DAT_MAX_CHAR);

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getNumComponents);
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
     *  int32_t _Dat_getNumComponents
     *  (
     *      Dat_ServerHandle       serverHandle,
     *      Dat_verbosityQueryCfg* queryCfg,
     *      int32_t*               errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_verbosityQueryCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Copy the producer name */
    memcpy((char*)args[1].argBuffer, &queryCfg, sizeof(Dat_verbosityQueryCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return -1;
    }
    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    if((int32_t)result < 0)
    {
        return -1;
    }
    /* Copy the result */
    memcpy(&queryCfg, args[1].argBuffer, sizeof(Dat_verbosityQueryCfg));
    *numComponents = queryCfg.cfg.numComponents;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function gets the verbosity level of the class and common level for
 *      all components, given the name of the trace object it belongs to.
 *
 *  @param[in]  localTracehandle
 *      DAT trace object handle
 *  @param[in]  traceObjectName
 *      Name of the trace object to which the component belongs.
 *  @param[out] ptrClassLevel
 *      Pointer to class verbosity level.
 *  @param[out] ptrCommonCompLevel
 *      Pointer to common verbosity level of all components.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getClassVerbosity
(
    Dat_TraceObjHandle  localTracehandle,
    char*               traceObjectName,
    Dat_TraceLevel*     ptrClassLevel,
    Dat_TraceLevel*     ptrCommonCompLevel,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ClientMCB*          ptrDatClient;
    Dat_LocalTraceObject*   ptrLocalTraceObj;
    uint32_t                result;
    int32_t                 jobId;
    Dat_verbosityQueryCfg   queryCfg;

    /* Check for valid inputs. */
    if (traceObjectName == NULL || ptrClassLevel == NULL || ptrCommonCompLevel == NULL)
    {
        *errCode =  DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrLocalTraceObj     = (Dat_LocalTraceObject*)localTracehandle;
    ptrDatClient         = (Dat_ClientMCB*)ptrLocalTraceObj->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Set up query configuration */
    strncpy(queryCfg.name, traceObjectName, DAT_MAX_CHAR);

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getClassVerbosity);
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
     *  int32_t _Dat_getClassVerbosity
     *  (
     *     Dat_ServerHandle        serverHandle,
     *     Dat_verbosityQueryCfg*  queryCfg,
     *     int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_verbosityQueryCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Copy the producer name */
    memcpy((char*)args[1].argBuffer, &queryCfg, sizeof(Dat_verbosityQueryCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Check result */
    if((int32_t)result < 0)
    {
        return -1;
    }

    /* Copy the result */
    memcpy(&queryCfg, args[1].argBuffer, sizeof(Dat_verbosityQueryCfg));
    *ptrClassLevel = josh_toNativeU32(queryCfg.cfg.classCfg.classLevel);
    *ptrCommonCompLevel = josh_toNativeU32(queryCfg.cfg.classCfg.commonCompLevel);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function gets the verbosity level of a given component.
 *
 *  @param[in]  localTracehandle
 *      DAT trace object handle
 *  @param[in]  traceObjectName
 *      Name of the trace object to which this component belongs to.
 *  @param[in]  componentId
 *      Trace component ID
 *  @param[out] ptrComponentInfo
 *      Pointer to the component configuration returned by the funciton
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getComponentVerbosity
(
    Dat_TraceObjHandle      localTracehandle,
    char*                   traceObjectName,
    uint32_t                componentId,
    Dat_TraceComponentCfg*  ptrComponentInfo,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Dat_ClientMCB*          ptrDatClient;
    Dat_LocalTraceObject*   ptrLocalTraceObj;
    uint32_t                result;
    int32_t                 jobId;
    Dat_verbosityQueryCfg       queryCfg;

    /* Check for valid inputs. */
    if (traceObjectName == NULL || ptrComponentInfo == NULL)
    {
        *errCode =  DAT_EINVAL;
        return -1;
    }

    /* Get the DAT Client MCB: Services are available only if the client is active. */
    ptrLocalTraceObj   = (Dat_LocalTraceObject*)localTracehandle;
    ptrDatClient       = (Dat_ClientMCB*)ptrLocalTraceObj->clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Set up query configuration */
    memset(&queryCfg, 0, sizeof(Dat_verbosityQueryCfg));
    strncpy(queryCfg.name, traceObjectName, DAT_MAX_CHAR);
    queryCfg.cfg.compCfg.componentId = componentId;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getComponentVerbosity);
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
     *  int32_t _Dat_getComponentVerbosity
     *  (
     *      Dat_ServerHandle        serverHandle,
     *      Dat_verbosityQueryCfg*  queryCfg,
     *      int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Dat_verbosityQueryCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);

    /* Copy the component info */
    memcpy((char*)args[1].argBuffer, &queryCfg, sizeof(Dat_verbosityQueryCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return -1;
    }
    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrDatClient->joshHandle, jobId) < 0)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    if((int32_t)result < 0)
    {
        return -1;
    }

    /* Copy the result */
    memcpy(&queryCfg, args[1].argBuffer, sizeof(Dat_verbosityQueryCfg));
    memcpy((uint8_t *)ptrComponentInfo, &queryCfg.cfg.compCfg.componentInfo, sizeof(Dat_TraceComponentCfg));

    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function modifies the verbosity levels. This function is a registered function call by
 *   DAT server.
 *
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
int32_t __Dat_modifyVerbosity
(
    Dat_ClientHandle    clientHandle,
    Dat_VerbosityCfg*   ptrVerbosityCfg,
    int32_t*            errCode
)
{
    Dat_TraceCompInfo*      ptrTraceObject;
    Dat_ClientMCB*          ptrDatClient;
    Dat_LocalTraceObject*   ptrLocalTraceObj;

    /* Find the trace object instance on DAT client */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;

    ptrLocalTraceObj = Dat_findClientTraceObjectInst(ptrDatClient, ptrVerbosityCfg->traceObjectName);
    if(ptrLocalTraceObj == NULL)
    {
        /* Error: Global trace object is not found */
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Modify the verbosity level. */
    ptrTraceObject = ptrLocalTraceObj->traceObject.ptrComponentArray;

    if (ptrVerbosityCfg->isEnabled)
        ptrTraceObject[ptrVerbosityCfg->traceComponentId].logLevel |=  ptrVerbosityCfg->verbosityLevel;
    else
        ptrTraceObject[ptrVerbosityCfg->traceComponentId].logLevel &=  ~ptrVerbosityCfg->verbosityLevel;

    return 0;

}

/**
 *  @b Description
 *  @n
 *      The function gets the name for all trace objects.
 *
 *  @param[in]  clientHandle
 *      DAT client handle
 *  @param[in]  nameArraySize
 *      Size of the name array.
 *  @param[in]  nameArray
 *      Pointer to the name array.
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Success -   Number of trace objects
 *  @retval
 *      Error   -   <0
 */
int32_t Dat_getTraceObjectNames
(
    Dat_ClientHandle    clientHandle,
    uint32_t            nameArraySize,
    char*               nameArray,
    int32_t*            errCode
)
{
    Josh_Argument       args[JOSH_MAX_ARGS];
    Josh_JobHandle      jobHandle;
    Josh_ArgHandle      argHandle;
    Dat_ClientMCB*      ptrDatClient;
    uint32_t            result;
    int32_t             jobId;

    /* Sanity check */
    if((clientHandle == NULL) || (nameArray == NULL) || (nameArraySize > DAT_COMPONENT_INIT_BLOCKSIZE))
    {
        *errCode = DAT_EINVAL;
        return -1;
    }

    /* Find the trace object instance on DAT client */
    ptrDatClient = (Dat_ClientMCB*)clientHandle;
    if (ptrDatClient->status != Dat_ClientStatus_ACTIVE)
    {
        *errCode = DAT_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrDatClient->joshHandle, (Josh_JobProtype)_Dat_getTraceObjectNames);
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
     *  int32_t _Dat_getTraceObjectNames
     *  (
     *      Dat_ServerHandle    serverHandle,
     *      uint32_t            nameArraySize,
     *      char*               nameArray,
     *      int32_t*            errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Dat_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(uint32_t);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = nameArraySize;

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrDatClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Populate the arguments*/
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrDatClient->serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(nameArraySize);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0 )
    {
        *errCode = DAT_EJOSH;
        return -1;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (ptrDatClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = DAT_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);
    if ((int32_t)result < 0)
    {
        return -1;
    }

    /* Copy the result */
    memcpy(nameArray, args[2].argBuffer, nameArraySize);

    /* Debug Message: */
    System_printf ("Debug: Found %d trace objects on DAT server. \n", result);

    return (int32_t)result;
}
/**
 *  @b Description
 *  @n
 *      The function is used to register the DAT services
 *
 *  @param[in]   nodeHandle
 *      JOSH node handle on which the services are being registered
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Dat_registerServices(Josh_NodeHandle nodeHandle)
{
    /* DAT trace framework services */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_createConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_createProducer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_connectConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)__Dat_connectConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_deleteProducer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_deleteConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_disconnectConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)__Dat_disconnectConsumer);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_stopClient);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getConsumerStatus);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getGPProdDataOffset);

    /* DAT trace verbosity services */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_createTraceObject);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_deleteTraceObject);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_createTraceObjectInstance);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_initTraceComponentBlock);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getTraceComponentBlock);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_deleteTraceObjectInstance);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_modifyVerbosity);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)__Dat_modifyVerbosity);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getNumComponents);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getClassVerbosity);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getComponentVerbosity);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Dat_getTraceObjectNames);

    return;
}
