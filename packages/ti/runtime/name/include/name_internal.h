/**
 *   @file  name_internal.h
 *
 *   @brief
 *      Internal header file for the Name library. 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
 *  \par
 */

#ifndef __NAME_INTERNAL_H__
#define __NAME_INTERNAL_H__

/* SYSLIB Include files */
#include <ti/runtime/name/name.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/include/listlib.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/** @addtogroup NAME_INTERNAL_SYMBOL
 @{ */

/**
 * @brief   Hash Bucket Size for each database
 */
#define NAME_DATABASE_HASH_BUCKET       64

/**
 * @brief   Hash Bucket Size for each database
 */
#define NAME_MAX_SUPPORTED_CLIENTS      32

/**
 * @brief   Maximum number of tracking entries which indicate the number
 * of concurrent pending transactions which can exist for an agent server
 */
#define NAME_MAX_TRACKING_ENTRIES       16

/**
@}
*/

/** @addtogroup NAME_INTERNAL_STRUCT
 @{ */


/** 
 * @brief 
 *  Agent Tracking Entry
 *
 * @details
 *  Agent Tracking entry keep track of JOSH transactions which are still in flight
 */
typedef struct Name_TrackingEntry
{
    /**
     * @brief   Links to the other tracking entries
     */
    Name_ListNode          links;

    /**
     * @brief   Client Identifier
     */
    uint32_t                clientId;

    /**
     * @brief   Pending Transaction identifier
     */
    uint32_t                transactionId;
}Name_TrackingEntry;

/** 
 * @brief 
 *  Name DSP Client Id to MSGCOM Channel map
 *
 * @details
 *  The structure is used to keep a map which maps each DSP agent client to
 *  a MSGCOM channel handle. Server will need to send the JOSH message to the
 *  clients via the MSGCOM Channel handle.
 */
typedef struct Name_DSPClientIdToChannel
{
    /**
     * @brief   Links to the other mapped entries
     */
    Name_ListNode           links;
        
    /**
     * @brief   Client identifier.
     */
    uint32_t                clientId;

    /**
     * @brief   MSGCOM channel handle
     */
    MsgCom_ChHandle         channelHandle;
}Name_DSPClientIdToChannel;

/** 
 * @brief 
 *  Name Proxy MCB
 *
 * @details
 *  Name Proxy MCB which is used to store internal information relevant to the specific 
 *  proxy instance which is being executed.
 */
typedef struct Name_ProxyMCB
{
    /**
     * @brief   Configuration of the proxy
     */
    Name_ProxyCfg               cfg;

    /**
     * @brief   List of free tracking entries 
     */
    Name_TrackingEntry*         freeTrackingList;

    /**
     * @brief   List of tracking entries which are pending 
     */
    Name_TrackingEntry*         pendingTrackingList;

    /**
     * @brief   Shared memory address specific to the local realm. This is used
     * to synchronize the server with its remote peer.
     */
    uint8_t*                    ptrMyRealmSharedAddress;

    /**
     * @brief   Shared memory address specific to the remote realm. This is used
     * to synchronize the server with its remote peer.
     */
    uint8_t*                    ptrPeerRealmSharedAddress;

    /**
     * @brief   Flag which indicates if the servers are synchronized with each other.
     */
    uint8_t                     synched;

    /**
     * @brief   Local realm channel handle which is used to receive messages from 
     * agent clients in the same realm.
     */
    void*                       localRealmServerChannel;

    /**
     * @brief   Pointer to the DSP Client identifier to the MSGCOM channel map. 
     */
    Name_DSPClientIdToChannel*  ptrDSPClientIdChannelMap;

    /**
     * @brief   Proxy created default client which handles all the name client requests from 
     * the remote proxy
     */
    Name_ClientHandle           defaultClientHandle;

    /**
     * @brief   Handle to the CPPI Infrastructure DMA block.
     */
    Cppi_Handle                 cppiHnd;

    /**
     * @brief   Handle to the CPPI receive channel which allows the server to 
     * receive messages from its peer.
     */
    Cppi_ChHnd                  rxChannelHnd;

    /**
     * @brief   Handle to the CPPI transmit channel which allows the server to 
     * send messages to its peer.
     */
    Cppi_ChHnd                  txChannelHnd;

    /**
     * @brief   This is the CPPI Flow handle which is configured. Messages received
     * from the remote peer are picked using this handle.
     */
    Cppi_FlowHnd                flowChannelHnd;

    /**
     * @brief   This is the transmit queue used by the writer to push messages.
     */
    Qmss_QueueHnd               messageTxQueue;

    /**
     * @brief   This is the receive queue used by the reader to read messages.
     */
    Qmss_QueueHnd               messageRxQueue;
}Name_ProxyMCB;

/** 
 * @brief 
 *  Name Client MCB
 *
 * @details
 *  The structure is used to store information pertinent to the name client
 */
typedef struct Name_ClientMCB
{
    /**
     * @brief   Client configuration
     */
    Name_ClientCfg              cfg;

    /**
     * @brief   Name proxy handle to which the client is connected
     */
    Name_ProxyHandle            proxyHandle;

    /**
     * @brief  JOSH Node handle associated with the name client.
     */
    Josh_NodeHandle             joshHandle;

    /**
     * @brief   Opaque handle to the client reader channel which is used to 
     * receive messages from the server
     */
    void*                       clientReaderChannel;

    /**
     * @brief   Opaque handle to the server reader channel which is used by
     * the server to receive messages from the client.
     */
    void*                       serverReaderChannel;
}Name_ClientMCB;

/***********************************************************************************
 * Internal Exported API: 
 ***********************************************************************************/

/* Hash function: */
extern uint32_t hash( register uint8_t *k, register uint32_t length, register uint32_t initval);

/* Proxy Realm Exported API: */
extern void*   Name_createProxyChannel(Name_ProxyMCB* ptrNameProxy, int32_t* errCode);
extern int32_t Name_receiveFromClient(Name_ProxyMCB* ptrNameProxy, uint8_t** ptrDataBuffer, 
                                      uint32_t* dataLen, uint32_t* msgBuffer, uint32_t* fromClientId);
extern int32_t Name_sendToClient(Name_ProxyMCB* ptrNameProxy, uint8_t* ptrTxDataBuffer, uint32_t txDataLen, uint32_t toClientId);
extern int32_t Name_freePacket(Name_ProxyMCB* ptrNameProxy, uint32_t msgBuffer);
extern int32_t Name_deleteProxyChannel (void* localCommChannel, const char* serverName, int32_t* errCode);

/* Infrastructure DMA Exported API: */
extern int32_t Name_infraDMACreate(Name_ProxyMCB* ptrNameProxy, int32_t* errCode);
extern int32_t Name_infraDMAGetMsg (Name_ProxyMCB* ptrNameProxy, Ti_Pkt** ptrPkt);
extern int32_t Name_infraDMAPutMsg (Name_ProxyMCB* ptrNameProxy, Ti_Pkt* ptrPkt);
extern int32_t Name_infraDMADelete (Name_ProxyMCB* ptrNameProxy);

/* Client Realm Exported API: */
extern int32_t Name_createClientTransport (Name_ClientMCB* ptrNameClient, int32_t* errCode);
extern int32_t Name_deleteClientTransport (Name_ClientMCB* ptrNameClient, int32_t* errCode);

/* Database Internal Exported API: */
extern int32_t Name_createResourceOwner(Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                        Name_ResourceCfg* ptrResourceCfg, const char* ptrOwnerName, int32_t* errCode);
extern int32_t Name_findResourceOwner (Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                       const char* name, Name_ResourceCfg* ptrNameResourceCfg,
                                       char* ptrOwnerName, int32_t* errCode);
/**
@}
*/

#endif /* __NAME_INTERNAL_H__ */

