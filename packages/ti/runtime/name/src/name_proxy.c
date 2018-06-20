/**
 *   @file  name_proxy.c
 *
 *   @brief
 *      The file implements the name proxy module. Name Proxy are
 *      responsible for the ensuring that the name databases are
 *      synchronized between the different execution realms.
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

/* MCSDK Include files */
#include <ti/csl/csl_cache.h>

/* SYSLIB Include files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 **************************** Proxy Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility wrapper function which provides logs agent server messages
 *      via the application supplied logging function
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the agent server
 *  @param[in]  logLevel
 *      Logging Level
 *  @param[in]  fmt
 *      Formatted string arguments
 *  @param[in]  ...
 *      Variable arguments
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable.
 */
static inline void __attribute__ ((format (printf, 3, 4))) Name_logMsg (Name_ProxyMCB* ptrNameProxy, Name_LogLevel logLevel, const char* fmt, ...)
{
    va_list arg;

    /* Pass control to the application supplied calling function. */
    if (ptrNameProxy->cfg.logFxn)
    {
        va_start (arg, fmt);
        ptrNameProxy->cfg.logFxn (logLevel, fmt, arg);
        va_end (arg);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the enter critical section API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Always returns NULL
 */
void* NameProxy_osalDummyEnterCS (void)
{
   return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the exit critical section API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
void NameProxy_osalDummyExitCS (void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the create semaphore API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Always returns 0x1234
 */
void* NameProxy_osalDummyCreateSem (void)
{
    return (void*)0x1234;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the delete semaphore API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
void NameProxy_osalDummyDeleteSem(void* semHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the pend semaphore API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
void NameProxy_osalDummyPendSem(void* semHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a dummy implementation of the post semaphore API which
 *      is passed to the JOSH for the default client creation on the Name proxy. The
 *      default client is a special client which only handles the JOSH Submit requests
 *      from the name clients but never sends a JOSH submit request by itself
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
void NameProxy_osalDummyPostSem(void* semHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      This function is used to determine if the agent servers between the ARM and DSP
 *      realm are synhronized. Failure to synchronize the agent servers will result in
 *      agent client requests being dropped since the remote peer was not operational.
 *
 *  @param[in]  proxyHandle
 *      Handle to the name proxy
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      1 -   Servers are synched
 *  @retval
 *      0 -   Servers are not synched
 *  @retval
 *      -1 -  Error
 */
int32_t Name_isProxySynched (Name_ProxyHandle proxyHandle, int32_t* errCode)
{
    Name_ProxyMCB*          ptrNameProxy;
    Name_ResourceCfg        nameResourceCfg;
    Name_ClientCfg          defaultClientCfg;

    /* Get the name proxy block */
    ptrNameProxy = (Name_ProxyMCB*)proxyHandle;
    if (ptrNameProxy == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Are the proxies already synchronized? */
    if (ptrNameProxy->synched == 1)
        return 1;

    /* NO. Check the shared memory address. */
    ptrNameProxy->cfg.beginMemAccess (ptrNameProxy->ptrPeerRealmSharedAddress, CACHE_L2_LINESIZE);
    if (*(ptrNameProxy->ptrPeerRealmSharedAddress) == 1)
    {
        /* Synchronization has been acheived with the remote proxy. */
        ptrNameProxy->synched = 1;

        /* Find the named resource */
        if (Name_findResource (ptrNameProxy->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               ptrNameProxy->cfg.proxyName, &nameResourceCfg, errCode) < 0)
        {
            /* FATAL Error: This cannot happen and this implies that there is a corruption in the database */
            System_printf ("Error: Name Proxy %s lookup failed [Error code %d]\n",
                            ptrNameProxy->cfg.proxyName, *errCode);
            return -1;
        }

        /* Update the synhronization status: */
        nameResourceCfg.handle3 = 1;

        /* Modify the named resource status. */
        if (Name_modifyResource(ptrNameProxy->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                &nameResourceCfg, errCode) < 0)
        {
            /* FATAL Error: This should not happen. */
            System_printf ("Error: Name Proxy %s synch update failed [Error code %d]\n",
                            ptrNameProxy->cfg.proxyName, *errCode);
            return -1;
        }

        /* Name Proxy is operational: Create the default PROXY client which handles the
         * name requests from the remote peer. */
        memset ((void *)&defaultClientCfg, 0, sizeof(Name_ClientCfg));

        /* Populate the default client configuration: */
        strncpy (defaultClientCfg.clientName, ptrNameProxy->cfg.proxyName, NAME_MAX_CHAR);
        strncpy (defaultClientCfg.proxyName,  ptrNameProxy->cfg.proxyName, NAME_MAX_CHAR);
        defaultClientCfg.databaseHandle            = ptrNameProxy->cfg.databaseHandle;
        defaultClientCfg.realm                     = ptrNameProxy->cfg.realm;
        if (ptrNameProxy->cfg.realm == Name_ExecutionRealm_DSP)
        {
            defaultClientCfg.u.dspCfg.pktlibInstHandle = ptrNameProxy->cfg.pktlibInstHandle;
            defaultClientCfg.u.dspCfg.msgcomInstHandle = ptrNameProxy->cfg.u.dspCfg.msgcomInstHandle;;
            defaultClientCfg.u.dspCfg.clientHeapHandle = ptrNameProxy->cfg.u.dspCfg.clientProxyHeapHandle;
        }
        defaultClientCfg.malloc                    = ptrNameProxy->cfg.malloc;
        defaultClientCfg.free                      = ptrNameProxy->cfg.free;
        defaultClientCfg.beginMemAccess            = ptrNameProxy->cfg.beginMemAccess;
        defaultClientCfg.endMemAccess              = ptrNameProxy->cfg.endMemAccess;
        defaultClientCfg.enterCS                   = NameProxy_osalDummyEnterCS;
        defaultClientCfg.exitCS                    = NameProxy_osalDummyExitCS;
        defaultClientCfg.createSem                 = NameProxy_osalDummyCreateSem;
        defaultClientCfg.deleteSem                 = NameProxy_osalDummyDeleteSem;
        defaultClientCfg.postSem                   = NameProxy_osalDummyPostSem;
        defaultClientCfg.pendSem                   = NameProxy_osalDummyPendSem;

        /* Create the default name client */
        ptrNameProxy->defaultClientHandle = Name_initClient (&defaultClientCfg, errCode);
        if (ptrNameProxy->defaultClientHandle == NULL)
        {
            int32_t tmpErrCode;

            /* FATAL Error: The default client was NOT instantiated which implies that the PROXY is not up. */
            System_printf ("Error: Unable to create the default name client [Error code %d]\n", *errCode);

            /* Reset the proxy synchronization status: */
            nameResourceCfg.handle3 = 0;
            Name_modifyResource(ptrNameProxy->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                                    &nameResourceCfg, &tmpErrCode);
            return -1;
        }
        System_printf ("Debug: Default name client %p has been created successfully\n", ptrNameProxy->defaultClientHandle);

        /* Synchronized. */
        return 1;
    }
    /* Still not synchronized */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used add a tracking entry. Tracking entries keep track of JOSH
 *      requests which are under transit.
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the agent server
 *  @param[in]  clientId
 *      Client Identifier
 *  @param[in]  transactionId
 *      Transaction identifier
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Name_addTrackRequest
(
    Name_ProxyMCB*    ptrNameProxy,
    uint32_t          clientId,
    uint32_t          transactionId
)
{
    Name_TrackingEntry*    ptrNameTrackingEntry;

    /* Allocate an agent tracking entry from the server */
    ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listRemove((Name_ListNode**)&ptrNameProxy->freeTrackingList);
    if (ptrNameTrackingEntry == NULL)
        return -1;

    /* Populate the transaction tracking entry */
    ptrNameTrackingEntry->clientId      = clientId;
    ptrNameTrackingEntry->transactionId = transactionId;

    /* Add this agent tracking entry to the pending list. */
    Name_listAdd ((Name_ListNode**)&ptrNameProxy->pendingTrackingList, (Name_ListNode*)ptrNameTrackingEntry);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to find a tracking entry given the transaction identifer
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the agent server
 *  @param[in]  transactionId
 *      Transaction identifier
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   Pointer to the agent tracking entry
 *  @retval
 *      Error   -   NULL
 */
static Name_TrackingEntry* Name_findTrackRequest
(
    Name_ProxyMCB*      ptrNameProxy,
    uint32_t            transactionId
)
{
    Name_TrackingEntry*    ptrNameTrackingEntry;

    /* Get the head of the agent tracking entry. */
    ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listGetHead((Name_ListNode**)&ptrNameProxy->pendingTrackingList);
    while (ptrNameTrackingEntry != NULL)
    {
        /* Match the entry */
        if (ptrNameTrackingEntry->transactionId == transactionId)
            return ptrNameTrackingEntry;

        /* Get the next tracking entry. */
        ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listGetNext((Name_ListNode*)ptrNameTrackingEntry);
    }
    /* No matching entry found. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This function is used delete a tracking entry which matches the
 *      specified parameters
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the agent server
 *  @param[in]  ptrNameTrackingEntry
 *      Tracking entry to be deleted
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static void Name_delTrackRequest
(
    Name_ProxyMCB*          ptrNameProxy,
    Name_TrackingEntry*     ptrNameTrackingEntry
)
{
    /* Remove this from the tracker list. */
    Name_listRemoveNode((Name_ListNode**)&ptrNameProxy->pendingTrackingList, (Name_ListNode*)ptrNameTrackingEntry);

    /* Add the entry back to the free tracking list. */
    Name_listAdd ((Name_ListNode**)&ptrNameProxy->freeTrackingList, (Name_ListNode*)ptrNameTrackingEntry);
    return;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the name proxy. The name proxy receives
 *      name services requests from the peer proxy. It also interfaces with the
 *      local name clients to provide the name services
 *
 *  @param[in]  proxyHandle
 *      Handle to the name proxy
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_executeProxy (Name_ProxyHandle proxyHandle, int32_t* errCode)
{
    Name_ProxyMCB*          ptrNameProxy;
    uint32_t                clientId;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataLen;
    uint8_t*                ptrTxDataBuffer;
    uint32_t                msgBuffer;
    uint32_t                packetType;
    uint32_t                transactionId;
    Ti_Pkt*                 ptrPkt;
    Name_TrackingEntry*     ptrNameTrackingEntry;
    int32_t                 syncStatus;

    /* Get the Name Proxy MCB */
    ptrNameProxy = (Name_ProxyMCB*)proxyHandle;
    if (ptrNameProxy == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /*****************************************************************************************
     * Process messages received from the name clients.
     *****************************************************************************************/
    while (1)
    {
        /* Get the message from the client */
        Name_receiveFromClient (ptrNameProxy, &ptrDataBuffer, &dataLen, &msgBuffer, &clientId);
        if (dataLen == 0)
            break;

        /* We have received a request from the agent client. We can process the message only if the
         * peer agent server is synchronized and ready else we simply drop the request */
        syncStatus = Name_isProxySynched(ptrNameProxy, errCode);
        if (syncStatus < 0)
            return -1;

        /* Are we synchronized? */
        if (syncStatus == 0)
        {
            /* Report the message to the client. This indicates that the application start up sequence is
             * not correct since the clients have started before the servers have synched up with
             * each other. The applications should have ensured that the PROXY are synchronized before
             * starting the NAME Clients. Unfortunately there is nothing much we can do at this point. */
            Name_logMsg (ptrNameProxy, Name_LogLevel_ERROR,
                          "Error: Dropping request since agent servers are not synched\n");

            /* Cleanup the received message from the agent clients. */
            Name_freePacket (ptrNameProxy, msgBuffer);
            break;
        }

        /* Get the necessary information from the packet */
        transactionId = Josh_getTransactionId(ptrDataBuffer);
        packetType    = Josh_getPacketType(ptrDataBuffer);

        /* Processing is dependent on the type of packet: */
        if (packetType == JOSH_PACKET_TYPE_SUBMIT)
        {
            /* SUBMIT Packet: Name client has sent a SUBMIT request to execute a JOB on the remote peer
             * Record the message into the request tracker. */
            if (Name_addTrackRequest (ptrNameProxy, clientId, transactionId) < 0)
            {
                *errCode = NAME_ENOMEM;
                return -1;
            }
        }
        else
        {
            /* RESULT Packet: Name client has processed a JOB from the remote peer and is sending back
             * the results. Get the agent tracking entry. */
            ptrNameTrackingEntry = Name_findTrackRequest(ptrNameProxy, transactionId);
            if (ptrNameTrackingEntry == NULL)
            {
                /* Error: This implies that the server received a JOSH result packet for a transaction which
                 * was not present in the tracking database. This is a fatal internal error. */
                *errCode = NAME_EINTERNAL;
                return -1;
            }

            /* We are done with the tracking entry. */
            Name_delTrackRequest (ptrNameProxy, ptrNameTrackingEntry);
        }

        /* Debug Message: */
        Name_logMsg (ptrNameProxy, Name_LogLevel_DEBUG,
                      "Debug: Sending %s packet from client to remote server [Transaction Id %x]\n",
                      (packetType == JOSH_PACKET_TYPE_SUBMIT) ? "Submit" : "Result", transactionId);

        /* Allocate the packet which can be transported to the remote agent server. */
        ptrPkt = Pktlib_allocPacket(ptrNameProxy->cfg.pktlibInstHandle, ptrNameProxy->cfg.proxyHeapHandle, dataLen);
        if (ptrPkt == NULL)
        {
            *errCode = NAME_ENOMEM;
            return -1;
        }

        /* Get the data buffer associated with the packet */
        Pktlib_getDataBuffer(ptrPkt, &ptrTxDataBuffer, &dataLen);

        /* Copy the data to be sent out */
        memcpy ((void *)ptrTxDataBuffer, (void *)ptrDataBuffer, dataLen);

        /* Writeback the data buffer */
        ptrNameProxy->cfg.endMemAccess(ptrTxDataBuffer, dataLen);

        /* Relay the received packet to the remote peer agent server */
        Name_infraDMAPutMsg (ptrNameProxy, ptrPkt);

        /* Cleanup the received message from the agent clients. */
        Name_freePacket (ptrNameProxy, msgBuffer);
    }

    /*****************************************************************************************
     * Remote Agent Server: Process messages received from the remote agent server
     *****************************************************************************************/
    while (1)
    {
        /* Receive message from the remote peer and process the message: */
        Name_infraDMAGetMsg(ptrNameProxy, &ptrPkt);
        if (ptrPkt == NULL)
            break;

        /* Get the data buffer and length associated with the packet. */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataLen);

        /* Invalidate the contents of the data buffer */
        ptrNameProxy->cfg.beginMemAccess(ptrDataBuffer, dataLen);

        /* Get the necessary information from the packet */
        transactionId = Josh_getTransactionId(ptrDataBuffer);
        packetType    = Josh_getPacketType(ptrDataBuffer);

        /* Process the JOSH packet. */
        if (packetType == JOSH_PACKET_TYPE_SUBMIT)
        {
            /* SUBMIT Packet: The remote peer had sent a JOB to be executed on the local agent client.
             * The message will be sent to the default client for processing. */
            if (Name_addTrackRequest (ptrNameProxy, (uint32_t)ptrNameProxy->defaultClientHandle, transactionId) < 0)
            {
                *errCode = NAME_ENOMEM;
                return -1;
            }

            /* Relay the received packet to the default agent client */
            Name_sendToClient (ptrNameProxy, ptrDataBuffer, dataLen, (uint32_t)ptrNameProxy->defaultClientHandle);
        }
        else
        {
            /* RESULT Packet: The remote peer has processed a JOB on behalf of the local agent and is sending
             * back the results. Get the agent tracking entry. */
            ptrNameTrackingEntry = Name_findTrackRequest(ptrNameProxy, transactionId);
            if (ptrNameTrackingEntry == NULL)
            {
                /* Error: This implies that the server received a JOSH result packet for a transaction which
                 * was not present in the tracking database. This is a fatal internal error. */
                *errCode = NAME_EINTERNAL;
                return -1;
            }

            /* Relay the received packet from remote peer agent server to the corresponding agent client. */
            Name_sendToClient (ptrNameProxy, ptrDataBuffer, dataLen, ptrNameTrackingEntry->clientId);

            /* We are done with the tracking entry. */
            Name_delTrackRequest (ptrNameProxy, ptrNameTrackingEntry);
        }

        /* Debug Message: */
        Name_logMsg (ptrNameProxy, Name_LogLevel_DEBUG,
                      "Debug: Sending %s packet from remote server to client [Transaction Id %x]\n",
                      (packetType == JOSH_PACKET_TYPE_SUBMIT) ? "Submit" : "Result", transactionId);

        /* Free up the packet. */
        Pktlib_freePacket(ptrNameProxy->cfg.pktlibInstHandle, ptrPkt);
    }

    /* Process the default client if it has been registered */
    if (ptrNameProxy->defaultClientHandle != NULL)
        Name_executeClient (ptrNameProxy->defaultClientHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to initialize and create the NAME proxy. Name
 *      proxy allows names to be exchanged between the ARM/DSP realms.
 *
 *  @param[in]  ptrNameProxyCfg
 *      Pointer to the name proxy configuration
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   Name Proxy Handle
 *  @retval
 *      Error   -   NULL
 */
Name_ProxyHandle Name_initProxy (Name_ProxyCfg* ptrNameProxyCfg, int32_t* errCode)
{
    Name_ProxyMCB*          ptrNameProxy;
    Name_TrackingEntry*     ptrNameTrackingEntry;
    Name_ResourceCfg        nameResourceCfg;
    uint32_t                index;

    /* Sanity Check: Validate the arguments */
    if ((ptrNameProxyCfg == NULL) || (ptrNameProxyCfg->proxyName[0] == 0))
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrNameProxyCfg->proxyHeapHandle == NULL) || (ptrNameProxyCfg->sharedMemoryAddress == 0))
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments */
    if (ptrNameProxyCfg->pktlibInstHandle == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the arguments. */
    if ((ptrNameProxyCfg->malloc         == NULL) || (ptrNameProxyCfg->free         == NULL) ||
        (ptrNameProxyCfg->beginMemAccess == NULL) || (ptrNameProxyCfg->endMemAccess == NULL))
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: In the DSP realm; there should be a client server heap configured which will allow
     * communication between the server and clients. We also need a valid MSGCOM instance handle */
    if (ptrNameProxyCfg->realm == Name_ExecutionRealm_DSP)
    {
        if ((ptrNameProxyCfg->u.dspCfg.clientProxyHeapHandle == NULL) ||
            (ptrNameProxyCfg->u.dspCfg.msgcomInstHandle      == NULL))
        {
            *errCode = NAME_EINVAL;
            return NULL;
        }
    }

    /* Sanity Check: Ensure that a valid database handle has been configured. */
    if (ptrNameProxyCfg->databaseHandle == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Each proxy name should be unique in the system */
    Name_findResource (ptrNameProxyCfg->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                       ptrNameProxyCfg->proxyName, &nameResourceCfg, errCode);

    /* The instance name should be *unique*; so we need to get an error from the above API with the
     * error code set to resource does not exist. If we get anything else either the instance name
     * is not unique or the named resources croaked. Either way we cannot proceed with the creation
     * of the server. */
    if (*errCode != NAME_ENOTFOUND)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Allocate memory for the proxy MCB */
    ptrNameProxy = (Name_ProxyMCB*)ptrNameProxyCfg->malloc(sizeof(Name_ProxyMCB), 0);
    if (ptrNameProxy == NULL)
    {
        *errCode = NAME_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrNameProxy, 0, sizeof(Name_ProxyMCB));

    /* Populate the agent server MCB */
    memcpy((void *)&ptrNameProxy->cfg, (void *)ptrNameProxyCfg, sizeof(Name_ProxyCfg));

    /* Create the local realm server channel handle which will be used by the name clients
     * in the local execution realm to communicate with the name proxy. */
    ptrNameProxy->localRealmServerChannel = Name_createProxyChannel(ptrNameProxy, errCode);
    if (ptrNameProxy->localRealmServerChannel == NULL)
    {
        /* Cleanup the memory allocated for the name proxy. */
        ptrNameProxyCfg->free (ptrNameProxy, sizeof(Name_ProxyMCB));
        return NULL;
    }

    /* Initialize the request tracking list. */
    for (index = 0; index < NAME_MAX_TRACKING_ENTRIES; index++)
    {
        /* Allocate memory for the agent tracking entry. */
        ptrNameTrackingEntry = (Name_TrackingEntry*)ptrNameProxy->cfg.malloc (sizeof(Name_TrackingEntry), 0);
        if (ptrNameTrackingEntry == NULL)
        {
            /* Error: Unable to allocate memory for the request tracking entry. */
            *errCode = NAME_ENOMEM;

            /* Cleanup the memory allocated for the name proxy. */
            ptrNameProxyCfg->free (ptrNameProxy, sizeof(Name_ProxyMCB));
            return NULL;
        }

        /* Initialize the allocated block of memory */
        memset ((void*)ptrNameTrackingEntry, 0, sizeof(Name_TrackingEntry));

        /* Add the tracking entry to the free list. */
        Name_listAdd ((Name_ListNode**)&ptrNameProxy->freeTrackingList, (Name_ListNode*)ptrNameTrackingEntry);
    }

    /* Initialize and setup the infrastructure DMA. */
    if (Name_infraDMACreate(ptrNameProxy, errCode) < 0)
    {
        System_printf ("Error: Infrastructure DMA channel creation failed\n");
        return NULL;
    }

    /* Determine the address which depends upon the realm in which the server is executing */
    if (ptrNameProxy->cfg.realm == Name_ExecutionRealm_DSP)
    {
        ptrNameProxy->ptrMyRealmSharedAddress   = (uint8_t*)(ptrNameProxy->cfg.sharedMemoryAddress + CACHE_L2_LINESIZE);
        ptrNameProxy->ptrPeerRealmSharedAddress = (uint8_t*)ptrNameProxy->cfg.sharedMemoryAddress;
    }
    else
    {
        ptrNameProxy->ptrMyRealmSharedAddress   = (uint8_t*)ptrNameProxy->cfg.sharedMemoryAddress;
        ptrNameProxy->ptrPeerRealmSharedAddress = (uint8_t*)(ptrNameProxy->cfg.sharedMemoryAddress + CACHE_L2_LINESIZE);
    }

    /* Debug: Display the name proxy configuration. This is useful during initial bring up stages to ensure
     * that the DSP and ARM are synchronized with the correct configuration. */
    Name_logMsg (ptrNameProxy, Name_LogLevel_DEBUG, "Debug: Name Proxy Local  Flow Id: %d\n", ptrNameProxyCfg->localFlowId);
    Name_logMsg (ptrNameProxy, Name_LogLevel_DEBUG, "Debug: Name Proxy Remote Flow Id: %d\n", ptrNameProxyCfg->remoteFlowId);

    /* Set the flag. */
    *(ptrNameProxy->ptrMyRealmSharedAddress) = 1;
    ptrNameProxy->cfg.endMemAccess (ptrNameProxy->ptrMyRealmSharedAddress, CACHE_L2_LINESIZE);

    /* Name proxy has not been synchronized. */
    ptrNameProxy->synched = 0;

    /* Initialize the name resource */
    memset ((void*)&nameResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the name resource */
    strncpy (nameResourceCfg.name, ptrNameProxyCfg->proxyName, NAME_MAX_CHAR);
    nameResourceCfg.handle1 = (uint32_t)ptrNameProxy;
    nameResourceCfg.handle2 = 0;
    nameResourceCfg.handle3 = 0;                            /* Proxy Synchronization Status: */

    /* Create the name resource */
    if (Name_createResource (ptrNameProxy->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &nameResourceCfg, errCode) < 0)
    {
        ptrNameProxyCfg->free (ptrNameProxy, sizeof(Name_ProxyMCB));
        return NULL;
    }
    return (Name_ProxyHandle)ptrNameProxy;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete the name proxy. Name Proxy will only be deleted
 *      if there are no pending tracking entries between the local and remote proxies.
 *
 *  @param[in]  proxyHandle
 *      Server handle which is to be deleted
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteProxy (Name_ProxyHandle proxyHandle, int32_t* errCode)
{
    Name_ProxyMCB*         ptrNameProxy;
    Name_TrackingEntry*    ptrNameTrackingEntry;

    /* Get the agent server block. */
    ptrNameProxy = (Name_ProxyMCB*)proxyHandle;
    if (ptrNameProxy == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* We cannot kill the server if there are pending entries still present. */
    ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listGetHead((Name_ListNode**)&ptrNameProxy->pendingTrackingList);
    if (ptrNameTrackingEntry != NULL)
    {
        *errCode = NAME_ENOTREADY;
        return -1;
    }

    /* Shutdown the default client if registered */
    if (ptrNameProxy->defaultClientHandle != NULL)
        Name_deleteClient  (ptrNameProxy->defaultClientHandle, errCode);

    /* Purge all the entries of agent requests which are in the free list */
    ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listRemove((Name_ListNode**)&ptrNameProxy->freeTrackingList);
    while (ptrNameTrackingEntry != NULL)
    {
        /* Cleanup the memory for the agent tracking entry */
        ptrNameProxy->cfg.free (ptrNameTrackingEntry, sizeof(Name_TrackingEntry));

        /* Get the next entry from the tracking list. */
        ptrNameTrackingEntry = (Name_TrackingEntry*)Name_listRemove((Name_ListNode**)&ptrNameProxy->freeTrackingList);
    }

    /* Remove the named resource from the table */
    Name_deleteResource (ptrNameProxy->cfg.databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                         ptrNameProxy->cfg.proxyName, errCode);

    /* Shutdown the Infrastructure DMA channels used for communication with the remote peer. */
    if (Name_infraDMADelete (ptrNameProxy) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Shutdown the channel which allows clients to communicate with the local agent server */
    if (Name_deleteProxyChannel (ptrNameProxy->localRealmServerChannel, ptrNameProxy->cfg.proxyName, errCode) < 0)
        return -1;

    /* Cleanup the agent server instance */
    ptrNameProxy->cfg.free (ptrNameProxy, sizeof(Name_ProxyMCB));
    return 0;
}

