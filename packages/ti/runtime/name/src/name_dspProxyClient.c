/**
 *   @file  name_dspProxyClient.c
 *
 *   @brief
 *      The file implements the DSP execution realm functionality which
 *      is required for the name Proxy and client to operate in the DSP
 *      execution realm.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* MCSDK Include files */
#include <ti/csl/csl_cache.h>

/* SYSLIB Include files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 *********************** Name DSP Client Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Callback function registered with the MSGCOM channel which is invoked
 *      when the channel is being deleted
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  chHandle
 *       MSGCOM channel being deleted
 *  @param[in]  msgBuffer
 *      Pointer to the buffer being deleted
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static void Name_deleteMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a mapping entry which maps the client
 *      identifier to a MSGCOM channel handle.
 *
 *  @param[in]   ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in]   clientId
 *      Client identifier
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   Mapped Entry
 *  @retval
 *      Error   -   NULL
 */
static Name_DSPClientIdToChannel* Name_findClientIdToChannelMap
(
    Name_ProxyMCB*  ptrNameProxy,
    uint32_t        clientId
)
{
    Name_DSPClientIdToChannel* ptrMapEntry;

    /* Cycle through the entire list. */
    ptrMapEntry = (Name_DSPClientIdToChannel*)Name_listGetHead ((Name_ListNode**)&ptrNameProxy->ptrDSPClientIdChannelMap);
    while (ptrMapEntry != NULL)
    {
        /* Do we have a match? */
        if (ptrMapEntry->clientId == clientId)
            return ptrMapEntry;

        /* Get the next entry. */
        ptrMapEntry = (Name_DSPClientIdToChannel*)Name_listGetNext ((Name_ListNode*)ptrMapEntry);
    }
    /* Control comes here implies that there is no match. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a mapping entry which maps the specified
 *      client identifier to the the MSGCOM channel handle
 *
 *  @param[in]   ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in]   clientId
 *      Client identifier
 *  @param[in]   chHandle
 *      Channel Handle
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Name_addClientIdToChannelMap
(
    Name_ProxyMCB*      ptrNameProxy,
    uint32_t            clientId,
    MsgCom_ChHandle     chHandle
)
{
    Name_DSPClientIdToChannel* ptrMapEntry;

    /* Allocate memory to the map entry. */
    ptrMapEntry = (Name_DSPClientIdToChannel*)ptrNameProxy->cfg.malloc (sizeof(Name_DSPClientIdToChannel), CACHE_L2_LINESIZE);
    if (ptrMapEntry == NULL)
        return -1;

    /* Initialize the allocated memory block. */
    memset ((void *)ptrMapEntry, 0, sizeof(Name_DSPClientIdToChannel));

    /* Populate the fields. */
    ptrMapEntry->clientId      = clientId;
    ptrMapEntry->channelHandle = chHandle;

    /* Add to the mapped list. */
    Name_listAdd ((Name_ListNode**)&ptrNameProxy->ptrDSPClientIdChannelMap, (Name_ListNode*)ptrMapEntry);
    return 0;
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
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Name_dspClientJoshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Ti_Pkt*             ptrPkt;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataBufferLen;
    Name_ClientMCB*     ptrNameClient;

    /* Get the name client information block. */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Allocate a packet from the agent server heap */
    ptrPkt = Pktlib_allocPacket(ptrNameClient->cfg.u.dspCfg.pktlibInstHandle, ptrNameClient->cfg.u.dspCfg.clientHeapHandle, size);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: JOSH Packet allocation from heap %p failed\n", ptrNameClient->cfg.u.dspCfg.clientHeapHandle);
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
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static void Name_dspClientJoshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Ti_Pkt*             ptrPkt;
    Name_ClientMCB*     ptrNameClient;

    /* Get the name client information block. */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Get the packet buffer. */
    ptrPkt = (Ti_Pkt*)msgBuffer;
    if (ptrPkt == NULL)
        return;

    /* Cleanup the packet. */
    Pktlib_freePacket(ptrNameClient->cfg.u.dspCfg.pktlibInstHandle, ptrPkt);
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
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static void Name_dspClientJoshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    uint32_t            dataBufferLen;
    Name_ClientMCB*     ptrNameClient;

    /* Get the name client information block. */
    ptrNameClient = (Name_ClientMCB*)nodeId;

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
    ptrNameClient->cfg.beginMemAccess (*ptrDataBuffer, dataBufferLen);
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
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static void Name_dspClientJoshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    uint32_t            dataBufferLen;
    uint8_t*            ptrDataBuffer;
    Name_ClientMCB*     ptrNameClient;

    /* Get the name client information block. */
    ptrNameClient = (Name_ClientMCB*)nodeId;

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)msgBuffer, &ptrDataBuffer, &dataBufferLen);

    /* Writeback the data buffer. */
    ptrNameClient->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /* Send the message */
    Msgcom_putMessage((MsgCom_ChHandle)writerChannel, (MsgCom_Buffer*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      This function is used to create the DSP realm specific transport which is
 *      a MSGCOM channel and connect this with the name proxy by setting up the
 *      JOSH nodes appropriately.
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_createClientTransport
(
    Name_ClientMCB* ptrNameClient,
    int32_t*        errCode
)
{
    Josh_NodeCfg            nodeCfg;
    Msgcom_ChannelCfg       chConfig;
    char                    channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Sanity Check: Validate the realm specific arguments. */
    if ((ptrNameClient->cfg.u.dspCfg.pktlibInstHandle == NULL) ||
        (ptrNameClient->cfg.u.dspCfg.clientHeapHandle == NULL) ||
        (ptrNameClient->cfg.u.dspCfg.msgcomInstHandle == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Initialize the msgcom channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                     = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack              = NULL;
    chConfig.msgcomInstHandle         = ptrNameClient->cfg.u.dspCfg.msgcomInstHandle;
    chConfig.u.queueCfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create a unique channel name for the name client: We combine the proxy name with the
     * address of the name client block to generate a unique address. */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-%x", ptrNameClient->cfg.proxyName, (uint32_t)ptrNameClient);

    /* Create the name client channel: */
    ptrNameClient->clientReaderChannel = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, errCode);
    if (ptrNameClient->clientReaderChannel == NULL)
        return -1;

    /* Construct the channel name for the server. */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-dsp", ptrNameClient->cfg.proxyName);

    /* Find the agent server reader channel. The server should have created its well known reader channel by now */
    ptrNameClient->serverReaderChannel = Msgcom_find(ptrNameClient->cfg.u.dspCfg.msgcomInstHandle, channelName, errCode);
    if (ptrNameClient->serverReaderChannel == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId                     = (uint32_t)ptrNameClient;
    nodeCfg.transportCfg.readerChannel = ptrNameClient->clientReaderChannel;
    nodeCfg.transportCfg.writerChannel = ptrNameClient->serverReaderChannel;

    /* Populate the transport interface */
    nodeCfg.transportCfg.transport.alloc = Name_dspClientJoshAlloc;
    nodeCfg.transportCfg.transport.free  = Name_dspClientJoshFree;
    nodeCfg.transportCfg.transport.get   = Name_dspClientJoshGet;
    nodeCfg.transportCfg.transport.put   = Name_dspClientJoshPut;

    /* Populate the OSAL table */
    nodeCfg.malloc      = ptrNameClient->cfg.malloc;
    nodeCfg.free        = ptrNameClient->cfg.free;
    nodeCfg.enterCS     = ptrNameClient->cfg.enterCS;
    nodeCfg.exitCS      = ptrNameClient->cfg.exitCS;
    nodeCfg.createSem   = ptrNameClient->cfg.createSem;
    nodeCfg.deleteSem   = ptrNameClient->cfg.deleteSem;
    nodeCfg.postSem     = ptrNameClient->cfg.postSem;
    nodeCfg.pendSem     = ptrNameClient->cfg.pendSem;

    /* Initialize the JOSH. */
    ptrNameClient->joshHandle = Josh_initNode (&nodeCfg, errCode);
    if (ptrNameClient->joshHandle == NULL)
        return -1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete an agent client and deregister this with
 *      the agent server. The function can only be invoked under the following
 *      conditions:
 *          - Application should kill client execution
 *          - Application should ensure that there the client services are no
 *            longer being used.
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteClientTransport
(
    Name_ClientMCB*     ptrNameClient,
    int32_t*            errCode
)
{
    /* Ensure that the JOSH services are shutdown. */
    if (Josh_deinitNode (ptrNameClient->joshHandle , errCode) < 0)
    {
        /* Error: JOSH Services failed to shutdown. Use the error code to determine
         * the reason for failure. */
        if (*errCode == JOSH_EBUSY)
        {
            /* This implies that there were still pending jobs in the JOSH node.
             * The prerequisite for the function was not met. */
            *errCode = NAME_ENOTREADY;
            return -1;
        }
        /* Any other error; simply propogate the error code. */
        return -1;
    }

    /* Close the MSGCOM channels */
    *errCode = Msgcom_delete (ptrNameClient->clientReaderChannel, Name_deleteMsgBuffer);
    if (*errCode < 0)
        return *errCode;
    *errCode = Msgcom_delete (ptrNameClient->serverReaderChannel, Name_deleteMsgBuffer);
    if (*errCode < 0)
        return *errCode;

#if 0
    /* Deregister as the default client identifier. */
    if (ptrNameClient->cfg.isDefaultClient == 1)
        Name_registerDefaultClientId (ptrNameClient->nrInstHandle, ptrNameClient->cfg.serverName, NAME_INVALID_DEFAULT_CLIENT_ID);
#endif
    return 0;
}

/***************************************************************************************
 * Name Proxy-Client Interface Functions:
 ***************************************************************************************/

/**
 *  @b Description
 *  @n
 *      This function is used to initialize the Name Client-Proxy communication
 *      channels. This communication path is MSGCOM on the DSP realm
 *
 *  @param[in]   ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   Opaque handle to the communication path
 *  @retval
 *      Error   -   NULL
 */
void* Name_createProxyChannel
(
    Name_ProxyMCB*      ptrNameProxy,
    int32_t*            errCode
)
{
    Msgcom_ChannelCfg   chConfig;
    MsgCom_ChHandle     chHandle;
    char                channelName[MSG_COM_MAX_CHANNEL_LEN];

    /* Construct the channel name */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-dsp", ptrNameProxy->cfg.proxyName);

    /* Initialize the msgcom channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                     = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.msgcomInstHandle         = ptrNameProxy->cfg.u.dspCfg.msgcomInstHandle;
    chConfig.appCallBack              = NULL;
    chConfig.u.queueCfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the MSGCOM Server channel handle: All the agent clients attached to the server will send
     * messages to this channel handle. */
    chHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, errCode);
    if (chHandle != NULL)
        System_printf ("Debug: Server Communication Channel '%s' created %p successfully\n", channelName, chHandle);
    else
        System_printf ("Error: Server Communication Channel '%s' was not created [Error code %d]\n", channelName, chHandle, errCode);
    return chHandle;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to receive messages from the clients
 *      in the DSP realm
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[out] ptrDataBuffer
 *      Pointer to the data buffer of the received message
 *  @param[out] dataLen
 *      Data length of the received message
 *  @param[out] msgBuffer
 *      Message buffer associated with the received message
 *  @param[out] fromClientId
 *      Client identifer from where the packet was received.
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_receiveFromClient
(
    Name_ProxyMCB*      ptrNameProxy,
    uint8_t**           ptrDataBuffer,
    uint32_t*           dataLen,
    uint32_t*           msgBuffer,
    uint32_t*           fromClientId
)
{
    Ti_Pkt* ptrRxPacket;

    /* Get a pending message: */
    Msgcom_getMessage ((MsgCom_ChHandle)ptrNameProxy->localRealmServerChannel, (MsgCom_Buffer**)&ptrRxPacket);
    if (ptrRxPacket != NULL)
    {
        /* Get the data buffer and length associated with the packet */
        Pktlib_getDataBuffer (ptrRxPacket, (uint8_t**)ptrDataBuffer, dataLen);

        /* Invalidate the data buffer */
        ptrNameProxy->cfg.beginMemAccess (*ptrDataBuffer, *dataLen);

        /* Return the associated message buffer */
        *msgBuffer = (uint32_t)ptrRxPacket;

        /* Return the client identifier from where the packet was received. */
        *fromClientId = Josh_getNodeId(*ptrDataBuffer);
    }
    else
    {
        /* No message received. */
        *dataLen          = 0;
        *msgBuffer        = 0;
        *ptrDataBuffer    = 0;
        *fromClientId     = 0xFFFFFFFF;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to send messages to the clients
 *      in the DSP realm
 *
 *  @param[in]  ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in]  ptrTxDataBuffer
 *      Pointer to the data buffer of the message to be sent out
 *  @param[in]  txDataLen
 *      Data length of the message to be sent
 *  @param[in]  toClientId
 *      Client Id to which the message is being sent.
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_sendToClient
(
    Name_ProxyMCB*      ptrNameProxy,
    uint8_t*            ptrTxDataBuffer,
    uint32_t            txDataLen,
    uint32_t            toClientId
)
{
    Ti_Pkt*                     ptrPacket;
    uint8_t*                    ptrDataBuffer;
    uint32_t                    dataBufferLen;
    MsgCom_ChHandle             chHandle;
    char                        channelName[MSG_COM_MAX_CHANNEL_LEN];
    Name_DSPClientIdToChannel*  ptrMapEntry;
    int32_t                     errCode;

    /* Check the mapping entry to determine if there a map from client id to MSGCOM channel handle exists. */
    ptrMapEntry = Name_findClientIdToChannelMap (ptrNameProxy, toClientId);
    if (ptrMapEntry != NULL)
    {
        /* Mapping Entry Exists: Use the channel handle from the mapped entry. */
        chHandle = ptrMapEntry->channelHandle;
    }
    else
    {
        /* Mapping Entry does NOT exist: Initialize the channel name */
        memset ((void *)&channelName[0], 0, sizeof(channelName));

        /* Map the client identifer to a channel name. */
        snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "%s-%x", ptrNameProxy->cfg.proxyName, toClientId);

        /* Find the MSGCOM channel associated with the client to which the packet is to be sent */
        chHandle = Msgcom_find (ptrNameProxy->cfg.u.dspCfg.msgcomInstHandle, channelName, &errCode);
        if (chHandle == NULL)
            return -1;

        /* Add the client identifier to the server channel map database for subsequent request processing. */
        if (Name_addClientIdToChannelMap (ptrNameProxy, toClientId, chHandle) < 0)
            return -1;
    }

    /* Allocate a packet from the client heap */
    ptrPacket = Pktlib_allocPacket(ptrNameProxy->cfg.pktlibInstHandle, ptrNameProxy->cfg.u.dspCfg.clientProxyHeapHandle, txDataLen);
    if (ptrPacket == NULL)
        return -1;

    /* Get the data buffer associated with the packet */
    Pktlib_getDataBuffer(ptrPacket, &ptrDataBuffer, &dataBufferLen);

    /* Copy the data buffer */
    memcpy ((void*)ptrDataBuffer, (void *)ptrTxDataBuffer, dataBufferLen);

    /* Writeback the data buffer */
    ptrNameProxy->cfg.endMemAccess (ptrDataBuffer, dataBufferLen);

    /* Send the message to the Name client */
    Msgcom_putMessage (chHandle, (MsgCom_Buffer*)ptrPacket);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used by the server to free messages received from clients
 *      in the DSP realm
 *
 *  @param[in] ptrNameProxy
 *      Pointer to the name proxy
 *  @param[in] msgBuffer
 *      Message buffer to be cleaned up
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_freePacket(Name_ProxyMCB* ptrNameProxy, uint32_t msgBuffer)
{
    Ti_Pkt* ptrRxPacket;

    /* Get the packet associated with the message buffer */
    ptrRxPacket = (Ti_Pkt*)msgBuffer;

    /* Cleanup the packet */
    Pktlib_freePacket(ptrNameProxy->cfg.pktlibInstHandle, ptrRxPacket);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to deinitialize the Name Proxy communication
 *      channels.
 *
 *  @param[in]  localCommChannel
 *      Pointer to the local communication channel
 *  @param[in]  serverName
 *      Name of the agent server
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  \ingroup NAME_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteProxyChannel (void* localCommChannel, const char* serverName, int32_t* errCode)
{
    MsgCom_ChHandle     chHandle;

    /* Get the channel handle. */
    chHandle = (MsgCom_ChHandle)localCommChannel;

    /* Delete the channel */
    *errCode = Msgcom_delete (chHandle, Name_deleteMsgBuffer);
    if (*errCode < 0)
        return -1;
    return 0;
}

