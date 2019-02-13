/**
 *   @file  neigh.c
 *
 *   @brief
 *      Proxy Neighbor Management Module
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <getopt.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <limits.h>

/* NETFP Proxy Files. */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**************************************************************************
 **************************** Local Definitions ***************************
 **************************************************************************/

/* This is used to send a broadcast notification to all the registered
 * listener identifiers */
#define NETFP_PROXY_BROADCAST_NOTIFICATION      0x0

/* Neighbor Manager Polling Delay: Specified in seconds */
#define NETFP_PROXY_NEIGH_POLL_DELAY               1

/* Neighbor Manager Failed State Delay: */
#define NETFP_PROXY_NEIGH_FAILED_STATE_DELAY       5

/**************************************************************************
 **************************** Local Structures ****************************
 **************************************************************************/

/**
 * @brief
 *  Neighbor Management Message Type
 *
 * @details
 *  The enumeration describes the various messages which can be passed to the Neighbor
 *  management module
 */
typedef enum NetfpProxy_NeighMsgType
{
    /**
     * @brief   Proxy Core has requested the lookup of a particular IP address
     */
    NetfpProxy_NeighMsgType_LOOKUP        = 0x1,

    /**
     * @brief   Proxy core has requested to stop monitoring a particular IP address
     */
    NetfpProxy_NeighMsgType_DELETE,

    /**
     * @brief   Proxy core has requested to display the neighbor cache for debugging
     * purposes
     */
    NetfpProxy_NeighMsgType_DUMP,

    /**
     * @brief   Proxy core is notified about an update to the ARP cache.
     */
    NetfpProxy_NeighMsgType_NOTIFY
}NetfpProxy_NeighMsgType;

/**
 * @brief
 *  ARP Message
 *
 * @details
 *  The structure describes the ARP message entry.
 */
typedef struct NetfpProxy_NeighMsg
{
    /**
     * @brief   Message Type:
     */
    NetfpProxy_NeighMsgType     msgType;

    /**
     * @brief   Target IP Address for which the operation is requested.
     */
    Netfp_IPAddr                targetIP;

    /**
     * @brief   Target MAC address: This is the MAC address of the target IP once the message
     * has been resolved.
     */
    uint8_t                     targetMACAddress[6];

    /**
     * @brief   Interface name on which the ARP cache resolution is to be done.
     */
    char                        ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Listener identifier associated with the operation.
     */
    uint32_t                    listenerId;
}NetfpProxy_NeighMsg;

/**
 * @brief
 *  Neighbor Management Listener Identifier
 *
 * @details
 *  Each neighbor management entry can be associated with multiple
 *  unique listener identifiers.
 */
typedef struct NetfpProxy_ListenerNode
{
    /**
     * @brief   Link to other listener entries
     */
    List_Node       links;

    /**
     * @brief   Listener Identifier
     */
    uint32_t        listenerId;
}NetfpProxy_ListenerNode;

/**
 * @brief
 *  Neighbor Management Entry
 *
 * @details
 *  The structures tracks each entry in the Neighbor management module
 */
typedef struct NetfpProxy_NeighEntry
{
    /**
     * @brief   Link to other Neighbor entries
     */
    List_Node                   links;

    /**
     * @brief   Target IP Address which is being monitored
     */
    Netfp_IPAddr                targetIP;

    /**
     * @brief   NETLINK Target IP Address
     */
    struct nl_addr*             ptrNLTargetIP;

    /**
     * @brief   Listener Identifier List associated with the neighbor entry
     */
    NetfpProxy_ListenerNode*    ptrListenerList;

    /**
     * @brief   Target MAC address: This is the MAC address of the target IP after resolution
     * else this is set to 0
     */
    uint8_t                     targetMACAddress[6];

    /**
     * @brief   Failed timeout value which is used to send out periodic requests to try and
     * keep the neighbor alive. This is maintained on a per neighbor basis.
     */
    int32_t                     failedTimeout;

    /**
     * @brief   Interface name on which the resolution was initiated.
     */
    char                        ifName[NETFP_MAX_CHAR];

    /**
     * @brief   NETMGR Neighbor event subscriber handle
     */
    netmgr_subsc_hdl            neighEvtSubscHdl;
}NetfpProxy_NeighEntry;

/**
 * @brief
 *  Neighbor Management MCB
 *
 * @details
 *  The structures is the Neighbor Management MCB.
 */
typedef struct NetfpProxy_NeighMgmtMCB
{
    /**
     * @brief   Neighbor Management Thread
     */
    pthread_t                   neighThread;

    /**
     * @brief   NETMGR Neighbor Management module
     */
    NetfpProxy_NetMgrHandle     netMgrNeighHandle;

    /**
     * @brief   Pipe used to get messages from the NETFP Proxy Core module.
     */
    int32_t                     fromCorePipe[2];

    /**
     * @brief   Pipe used to send messages to the NETFP Proxy Core module.
     */
    int32_t                     toCorePipe[2];

    /**
     * @brief   Pointer to the neighbor management list.
     */
    NetfpProxy_NeighEntry**     ptrNeighMgmtList;
}NetfpProxy_NeighMgmtMCB;

/**************************************************************************
 **************************** Global Variables ****************************
 **************************************************************************/

/* Neighbor MCB: */
NetfpProxy_NeighMgmtMCB     gNeighMgmtMCB;

//fzm
int (*NetfpProxy_neighPluginARPing)(const char *if_name, struct sockaddr *dst_addr, int family) = NULL;


/**************************************************************************
 **************************** NEIGH Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is a utility function which determines if the Neighbor
 *      entry is resolved or not.
 *
 *  @param[in]  ptrNeighEntry
 *      Pointer to the neighbor entry
 *
 *  @retval
 *      Resolved    -   1
 *  @retval
 *      Unresolved  -   0
 */
static inline uint32_t NetfpProxy_neighIsResolved(NetfpProxy_NeighEntry* ptrNeighEntry)
{
    uint8_t     zeroMacAddress[6] = { 0x0 };

    /* Match the MAC address to the all zero MAC address */
    if (memcmp ((void *)&ptrNeighEntry->targetMACAddress[0], (void*)&zeroMacAddress[0], 6) == 0)
        return 0;
    return 1;
}

/**
 *  @b Description
 *  @n
 *      This is the lookup function which is invoked by the NETFP Proxy core to
 *      request an address lookup which maps the Target IP to the Next HOP MAC
 *      address. Lookup operations are not immediate and can take some time.
 *
 *  @param[in]  ifName
 *      Interface name on which the Neighbor lookup is to be done
 *  @param[in]  ptrTargetIP
 *      Pointer to the Target IP address for which the neighbor lookup is done.
 *  @param[in] listenerId
 *      Listener identifier associated with the lookup request.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighLookup
(
    const char*     ifName,
    Netfp_IPAddr*   ptrTargetIP,
    uint32_t        listenerId
)
{
    NetfpProxy_NeighMsg message;
    int error;

    /* Initialize the message contents */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType    = NetfpProxy_NeighMsgType_LOOKUP;
    message.listenerId = listenerId;
    memcpy ((void*)&message.targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));
    strncpy (message.ifName, ifName, NETFP_MAX_CHAR);

    /* Send the message to the Neighbor Management Module */
    if(writeToPipe(gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg), &error) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "NeighLookup request failed: error %s\n", strerror(error));
        return -1;
    }
    else
        return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the function which is invoked to stop monitoring the Target IP address.
 *      The PROXY core will invoke this once the SA or Fast paths are deleted
 *
 *  @param[in]  ifName
 *      Interface name on which the Neighbor is to be deleted
 *  @param[in]  ptrTargetIP
 *      IP address which no longer needs to be maintained
 *  @param[in] listenerId
 *      Listener identifier associated with the lookup request.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighStopLookup(const char* ifName, Netfp_IPAddr* ptrTargetIP, uint32_t listenerId)
{
    NetfpProxy_NeighMsg   message;
    int error;

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType    = NetfpProxy_NeighMsgType_DELETE;
    message.listenerId = listenerId;
    memcpy ((void*)&message.targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));
    strncpy (message.ifName, ifName, NETFP_MAX_CHAR);

    /* Send the message: */
    if(writeToPipe(gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg), &error) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "NeighStopLookup request failed: error %s\n", strerror(error));
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the function which is invoked by the NETFP Proxy core module to get
 *      a display dump of the neighbor caches.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_neighDump(void)
{
    NetfpProxy_NeighMsg   message;
    int error;

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType = NetfpProxy_NeighMsgType_DUMP;

    /* Send the message: */
    writeToPipe(gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg), &error);
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the function which is invoked by the NETFP Proxy core module to get the
 *      status of a lookup operation which had been submitted before. The function is
 *      invoked by the PROXY core module only if the neighbor had sent a notification.
 *
 *  @param[out]  listenerId
 *      Listener identifier for which the resolution results are available
 *  @param[out] ptrTargetIP
 *      Target IP address for which the resolution was completed
 *  @param[out] macAddress
 *      MAC address populated if the lookup was successfully completed.
 *
 *  @retval
 *      Success -   1 [Results are available]
 *  @retval
 *      Success -   0 [No results are available]
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighGetLookupStatus
(
    uint32_t*       listenerId,
    Netfp_IPAddr*   ptrTargetIP,
    uint8_t*        macAddress
)
{
    NetfpProxy_NeighMsg   message;
    int                   error;

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Read the message from the */
    if (readFromPipe (gNeighMgmtMCB.toCorePipe[0], &message, sizeof(NetfpProxy_NeighMsg), &error) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read from the toCore Pipe [Error: %s]\n", strerror(error));
        return -1;
    }

    /* Sanity Check: We should have received only NOTIFY messages here */
    if (message.msgType != NetfpProxy_NeighMsgType_NOTIFY)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid message [%d] received on the toCore Pipe \n", message.msgType);
        return -1;
    }

    /* Populate the output arguments */
    *listenerId = message.listenerId;
    memcpy ((void *)ptrTargetIP, (void *)&message.targetIP, sizeof(Netfp_IPAddr));
    memcpy ((void *)macAddress, (void *)&message.targetMACAddress, 6);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to notify the NETFP Proxy core that there has
 *      been a status modification to a neighbor cache entry. Notifications
 *      can be broadcasted to all the listener identifiers or could be
 *      directed to a specific one.
 *
 *  @param[in]  ptrNeighEntry
 *      Pointer to the neighbor entry for which the notification are done
 *  @param[in]  listenerId
 *      Listener Identifier to which we need to send the notification.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_neighNotify(NetfpProxy_NeighEntry* ptrNeighEntry, uint32_t listenerId)
{
    NetfpProxy_NeighMsg         message;
    NetfpProxy_ListenerNode*    ptrListenerNode;
    int                         error;

    /* Has the neighbor entry been resolved? If not then we setup the failed timeout */
    if (NetfpProxy_neighIsResolved (ptrNeighEntry) == 0)
        ptrNeighEntry->failedTimeout = NETFP_PROXY_NEIGH_FAILED_STATE_DELAY;

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType = NetfpProxy_NeighMsgType_NOTIFY;
    memcpy ((void *)&message.targetIP, (void *)&ptrNeighEntry->targetIP, sizeof(Netfp_IPAddr));
    memcpy ((void *)&message.targetMACAddress, (void *)&ptrNeighEntry->targetMACAddress[0], 6);

    /* Do we need to broadcast the notification to all the registered listener nodes? */
    if (listenerId == NETFP_PROXY_BROADCAST_NOTIFICATION)
    {
        /* BROADCAST: Cycle through the listener list. */
        ptrListenerNode = (NetfpProxy_ListenerNode*)List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList);
        while (ptrListenerNode != NULL)
        {
            /* Populate the listener identifier in the message and send it to the core. */
            message.listenerId = ptrListenerNode->listenerId;

            if(writeToPipe(gNeighMgmtMCB.toCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg), &error) < 0)
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "NeighNotify-Broadcast write to pipe failed: error %s\n", strerror(error));
            else
                /* Debug Message: */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: Sending notification to Neighbor %p Listener Id 0x%x\n",
                               ptrNeighEntry, message.listenerId); //fzm

            /* Get the next listener node: */
            ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
        }
    }
    else
    {
        /* DIRECTED: Populate the listener identifier in the message and send it to the core. */
        message.listenerId = listenerId;

        if(writeToPipe(gNeighMgmtMCB.toCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg), &error) < 0)
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "NeighNotify-DIRECTED write to pipe failed: error %s\n", strerror(error));
        else
            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: Sending notification to Neighbor %p Listener Id 0x%x\n",
                           ptrNeighEntry, message.listenerId); //fzm
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to used to match the listener identifier
 *      with a list of listener identifier being tracked in the neighbor
 *      entry
 *
 *  @param[in]  ptrNeighEntry
 *      Pointer to the neighbor management entry
 *  @param[in]  listenerId
 *      Listener Identifier which is being searched
 *
 *  @retval
 *      1   -   Match found
 *  @retval
 *      0   -   No match found
 */
static int32_t NetfpProxy_findListenerId
(
    NetfpProxy_NeighEntry*  ptrNeighEntry,
    uint32_t                listenerId
)
{
    NetfpProxy_ListenerNode* ptrListenerNode;

    /* Cycle through all the listener identifiers */
    ptrListenerNode = (NetfpProxy_ListenerNode*)List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList);
    while (ptrListenerNode != NULL)
    {
        /* Do we have a match? */
        if (ptrListenerNode->listenerId == listenerId)
            return 1;

        /* Get the next element in the list. */
        ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to used to create a listener node and link this to the
 *      neighbor management entry
 *
 *  @param[in]  ptrNeighEntry
 *      Pointer to the neighbor management entry
 *  @param[in]  listenerId
 *      Listener Identifier which is to be added
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_createListenerId
(
    NetfpProxy_NeighEntry*  ptrNeighEntry,
    uint32_t                listenerId
)
{
    NetfpProxy_ListenerNode* ptrListenerNode;

    /* Allocate memory for the listener node */
    ptrListenerNode = (NetfpProxy_ListenerNode*)malloc (sizeof(NetfpProxy_ListenerNode));
    if (ptrListenerNode == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OOM for the listener node\n");
        return -1;
    }

    /* Initialize the allocated memory */
    memset ((void *)ptrListenerNode, 0, sizeof(NetfpProxy_ListenerNode));

    /* Populate the listener identifier and add to the listener list */
    ptrListenerNode->listenerId = listenerId;
    List_addNode ((List_Node**)&ptrNeighEntry->ptrListenerList, (List_Node*)ptrListenerNode);

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: Neighbor %p created listener id 0x%x\n", ptrNeighEntry, listenerId); //fzm
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to used to delete a listener node and unlink this from the
 *      neighbor management entry
 *
 *  @param[in]  ptrNeighEntry
 *      Pointer to the neighbor management entry
 *  @param[in]  listenerId
 *      Listener Identifier which is to be added
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_deleteListenerId
(
    NetfpProxy_NeighEntry*  ptrNeighEntry,
    uint32_t                listenerId
)
{
    NetfpProxy_ListenerNode* ptrListenerNode;

    /* Cycle through all the listener identifiers */
    ptrListenerNode = (NetfpProxy_ListenerNode*)List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList);
    while (ptrListenerNode != NULL)
    {
        /* Do we have a match? */
        if (ptrListenerNode->listenerId == listenerId)
        {
            /* YES: Remove the listener node */
            List_removeNode ((List_Node**)&ptrNeighEntry->ptrListenerList, (List_Node*)ptrListenerNode);

            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Neighbor %p deleted listener id 0x%x\n", ptrNeighEntry, listenerId);

            /* Cleanup the memory */
            free (ptrListenerNode);
            return 0;
        }

        /* Get the next element in the list. */
        ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
    }
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check for a matching neighbor management entry
 *
 *  @param[in]  ptrIPAddress
 *      Pointer to the IP address for which we are finding a match
 *  @param[in]  ifName
 *      Interface name
 *  @param[in]  listenerId
 *      Listener Identifier
 *
 *  @retval
 *      Non-NULL    -   Matching entry
 *  @retval
 *      NULL        -   No matching entry
 */
static NetfpProxy_NeighEntry* NetfpProxy_findNeighMgmtEntry
(
    Netfp_IPAddr*   ptrIPAddress,
    const char*     ifName
)
{
    NetfpProxy_NeighEntry*  ptrNeighEntry;

    /* Cycle through the neighbor managment list */
    ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getHead((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList);
    while (ptrNeighEntry != NULL)
    {
        /* Do the entries belong to the IPv4 Family? */
        if ((ptrNeighEntry->targetIP.ver == Netfp_IPVersion_IPV4) &&
            (ptrIPAddress->ver == Netfp_IPVersion_IPV4))
        {
            /* IPv4 socket family: Do the IP address match? */
            if (ptrNeighEntry->targetIP.addr.ipv4.u.a32 == ptrIPAddress->addr.ipv4.u.a32)
            {
                /* YES: Does the interface name match? */
                if (strcmp (ptrNeighEntry->ifName, ifName) == 0)
                    return ptrNeighEntry;
            }
        }

        /* Do the entries belong to the IPv6 Family? */
        if ((ptrNeighEntry->targetIP.ver == Netfp_IPVersion_IPV6) &&
            (ptrIPAddress->ver == Netfp_IPVersion_IPV6))
        {
            /* IPv6 socket family: Do the IP address match? */
            if ((ptrNeighEntry->targetIP.addr.ipv6.u.a32[0] == ptrIPAddress->addr.ipv6.u.a32[0]) &&
                (ptrNeighEntry->targetIP.addr.ipv6.u.a32[1] == ptrIPAddress->addr.ipv6.u.a32[1]) &&
                (ptrNeighEntry->targetIP.addr.ipv6.u.a32[2] == ptrIPAddress->addr.ipv6.u.a32[2]) &&
                (ptrNeighEntry->targetIP.addr.ipv6.u.a32[3] == ptrIPAddress->addr.ipv6.u.a32[3]))
            {
                /* YES: Does the interface name match? */
                if (strcmp (ptrNeighEntry->ifName, ifName) == 0)
                    return ptrNeighEntry;
            }
        }

        /* Get the next neighbor management entry */
        ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getNext ((List_Node*)ptrNeighEntry);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to generate data which in turn will cause the Linux
 *      kernel to send out an ARP resolution request.
 *
 *  @param[in]  ptrIPAddress
 *      Pointer to the target IP address which is to be resolved.
 *  @param[in]  ifName
 *      Interface name
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_neighARPPing
(
    Netfp_IPAddr*   ptrIPAddress,
    const char*     ifName,
    uint32_t        do_resolve
)
{
//fzm->>
    if(NetfpProxy_neighPluginARPing != NULL)
    {

        int     family;
        union Address
        {
            struct sockaddr_in ipv4;
            struct sockaddr_in6 ipv6;
        } addr;

        (void) do_resolve;

        if (ptrIPAddress->ver == Netfp_IPVersion_IPV4)
        {
            struct sockaddr_in *sai = (struct sockaddr_in *) &addr;
            family = AF_INET;
            memcpy(&sai->sin_addr, ptrIPAddress->addr.ipv4.u.a8, 4);

        }
        else if (ptrIPAddress->ver == Netfp_IPVersion_IPV6)
        {
            struct sockaddr_in6 *sai = (struct sockaddr_in6 *) &addr;
            family = AF_INET6;
            memcpy(&sai->sin6_addr, ptrIPAddress->addr.ipv6.u.a8, 16);
        }
        else
        {
            return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        }

        NetfpProxy_neighPluginARPing(ifName, (struct sockaddr *)&addr, family);

        return NETFP_PROXY_RETVAL_SUCCESS;
    }

    else
    {
//<-fzm
    char    commandName[NAME_MAX];
    char    strIPAddress[256];

    if (ptrIPAddress->ver == Netfp_IPVersion_IPV4)
    {
        /* IPv4 Address Resolution: */
        snprintf (commandName, sizeof(commandName), "ping -q -c 1 %d.%d.%d.%d -I %s", ptrIPAddress->addr.ipv4.u.a8[0],
                  ptrIPAddress->addr.ipv4.u.a8[1], ptrIPAddress->addr.ipv4.u.a8[2], ptrIPAddress->addr.ipv4.u.a8[3],
                  ifName);
        system (commandName);
        return 0;
    }

    /* IPv6 Address Resolution: Convert the IP Address into a string: */
    Netfp_convertIP6ToStr (ptrIPAddress->addr.ipv6, &strIPAddress[0]);

    if (do_resolve)
    {
        /* Use the NDISC6 & PING6 to perform route resolution */
        snprintf (commandName, sizeof(commandName), "ndisc6 -q -r 3 -1 %s %s", strIPAddress, ifName);
        system (commandName);
        snprintf (commandName, sizeof(commandName), "ping6 -q -c 1 %s -I %s", strIPAddress, ifName);
        system (commandName);
    }
    else
    {
        /* Use the NDISC6 to update the neighbor table entry */
        snprintf (commandName, sizeof(commandName), "ndisc6 -q -r 3 -1 %s %s", strIPAddress, ifName);
        system (commandName);
    }
    return 0;
}
}//fzm

/**
 *  @b Description
 *  @n
 *      Neighbor Event Handler: This is executed in the context of the Neighbor Thread
 *
 *  @param[in]  action
 *      Event being notified
 *  @param[in]  type
 *      Event family (NETMGR_NEIGH_EVT)
 *  @param[in]  obj
 *      Modified network object
 *  @param[in]  infoPtr
 *      Network Proxy state info corresponding to this event (cookie).
 *      Handle passed to network manager at time of event registration.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_monitorNeighUpdates (int32_t action, int32_t type, void* obj, void*  infoPtr)
{
    NetfpProxy_NeighEntry*      ptrNeighEntry;
    struct rtnl_neigh*          nlNeigh;
    int32_t                     neighStatus;
    uint8_t                     macAddress[6];
    uint8_t                     zeroMacAddress[6] = { 0x0 };
    char                        addressString[128];
    char                        stateStr[128];

    /* Report the event to the application plugin if it installed an event handler */
    if (gNetfpProxyMcb.pluginCfg.report_net_event != NULL)
        gNetfpProxyMcb.pluginCfg.report_net_event (action, type, obj);

    /* Get the neighbor information: */
    nlNeigh       = (struct rtnl_neigh *)obj;
    ptrNeighEntry = (NetfpProxy_NeighEntry *)infoPtr;
    neighStatus   = rtnl_neigh_get_state (nlNeigh);

    /* Derive the MAC address: */
    if (rtnl_neigh_get_lladdr(nlNeigh) != NULL)
        memcpy ((void *)&macAddress[0], (void *)nl_addr_get_binary_addr(rtnl_neigh_get_lladdr(nlNeigh)), 6);
    else
        memset ((void *)&macAddress[0], 0, 6);

    /* Create the display strings: */
    nl_addr2str (rtnl_neigh_get_dst (nlNeigh), addressString, sizeof (addressString));
    rtnl_neigh_state2str (neighStatus, stateStr, sizeof (stateStr));

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Received %s Neighbor %p Event for %s state: %s for interface: %s!!\n",
                       NetfpProxy_action2Str (action), ptrNeighEntry, addressString, stateStr, ptrNeighEntry->ifName);
    /*fzm-->
    if ((neighStatus == NUD_REACHABLE || neighStatus == NUD_STALE ) &&
            (nl_addr_cmp (rtnl_neigh_get_lladdr (nlNeigh), rtnl_neigh_get_lladdr (ptrNeighEntry->nhNeigh)) != 0) )
    {
        gRouteMgmtMCB.isFlushPending = 1;
        return;
    }
    fzm<-- */

    /* Process the update event */
    switch (action)
    {
        case NL_ACT_DEL:
        {
            /**************************************************************************************
             * Neighbor Deleted: Reset the neighbor MAC Address & notify the PROXY core
             **************************************************************************************/
            memset ((void *)&ptrNeighEntry->targetMACAddress[0], 0, 6);
            NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
            break;
        }
        case NL_ACT_NEW:
        {
            /**************************************************************************************
             * New Neighbor Entry created: Was the entry resolved?
             **************************************************************************************/
            if (memcmp ((void *)&macAddress[0], (void *)&zeroMacAddress, 6) == 0)
            {
                /* NO: Neighbor is being created; but in the failed state because the MAC address is set to 0 */
                memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)&macAddress[0], 6);
                NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
                break;
            }

            /* Entry was resolved; we need to notify the PROXY Core if there is a change in the MAC address */
            if (memcmp ((void *)&macAddress[0], (void *)&ptrNeighEntry->targetMACAddress[0], 6) != 0)
            {
                /* YES. The MAC address was different. Copy over the new MAC address */
                memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)&macAddress[0], 6);
                NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
            }
            break;
        }
        case NL_ACT_CHANGE:
        {
            /**************************************************************************************
             * Neighbor Status changed: We need to handle each of the NEIGHBOR states
             **************************************************************************************/
            switch (neighStatus)
            {
                case NUD_REACHABLE: //pass through
                case NUD_PERMANENT:
                {
                    /* The neighbor is now reachable; copy over the MAC address and notify the PROXY core */
                    memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)&macAddress[0], 6);
                    NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
                    break;
                }
                case NUD_STALE:
                {
                    /* The neighbor has become stale; we need to keep this entry alive since it is in use */
                    NetfpProxy_neighARPPing (&ptrNeighEntry->targetIP, &ptrNeighEntry->ifName[0], 0);

                    /* Clear the "Stale" flag from cache for this entry.
                     * This will ensure that on refill we will be able to
                     * reload the entry fully from the kernel and trigger any
                     * update based on state flags. Put this in
                     * as a fix for a race condition, where in by the time we
                     * went to actually refill the cache, the entry in the kernel
                     * again went from reachable --> stale, so on reload from kernel
                     * we would miss this update and eventually the neighbor entry
                     * just got deleted from the kernel since we never got this stale
                     * update and hence never kept it alive. */
                    rtnl_neigh_unset_state (nlNeigh, NUD_STALE);

                    /* Do we need to update the neighbor entry? This can happen because when the neighbor was created
                     * it was already in the STALE stage and so in that case we can inherit the MAC address and notify
                     * the PROXY core. */
                    if (memcmp ((void *)&macAddress[0], (void *)&ptrNeighEntry->targetMACAddress[0], 6) != 0)
                    {
                        /* YES. The MAC address was different. Copy over the new MAC address */
                        memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)&macAddress[0], 6);
                        NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
                    }
                    break;
                }
                case NUD_FAILED:
                {
                    /* The neighbor is down. Reset the Target MAC address and notify the PROXY core. */
                    memset ((void *)&ptrNeighEntry->targetMACAddress[0], 0, 6);
                    NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
                    break;
                }
                default:
                {
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Ignoring Neigh status: %s\n", stateStr);
                    break;
                }
            }
            break;
        }
        default:
        {
            /* Unhandled action: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Warning: Ignoring %s Neighbor %p Event for %s state: %s for interface: %s!!\n",
                               NetfpProxy_action2Str (action), ptrNeighEntry, addressString, stateStr, ptrNeighEntry->ifName);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the messages received from the Neighbor Management
 *      module.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_neighProcessMessage(void)
{
    NetfpProxy_NeighMsg     message;
    NetfpProxy_NeighEntry*  ptrNeighEntry;
    int32_t                 errCode;
    uint32_t*               neighList;
    uint32_t                numEntries;
    struct rtnl_neigh*      ptrNeighbor;
    char                    addressString[128];
    uint32_t                index;

    /* Read the message from the PROXY */
    if (readFromPipe (gNeighMgmtMCB.fromCorePipe[0], &message, sizeof(NetfpProxy_NeighMsg), &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read from the PROXY Pipe, Error: %s\n", strerror(errCode));
        return;
    }

    /* Process the message */
    switch (message.msgType)
    {
        case NetfpProxy_NeighMsgType_LOOKUP:
        {
            /* Do we have a matching neighbor entry? */
            ptrNeighEntry = NetfpProxy_findNeighMgmtEntry(&message.targetIP, &message.ifName[0]);
            if (ptrNeighEntry != NULL)
            {
                /* YES: Duplicate neighbor entry is found. Is there a match on the listener identifier also? */
                if (NetfpProxy_findListenerId (ptrNeighEntry, message.listenerId) == 0)
                {
                    /* NO: This implies that a new listener identifier is being registered with the neighbor
                     * entry. Create the new listener node */
                    NetfpProxy_createListenerId (ptrNeighEntry, message.listenerId);
                }

                /* Notify the listener about the status immediately. */
                NetfpProxy_neighNotify (ptrNeighEntry, message.listenerId);
                break;
            }

            /* No neighbor entry is found. So we need to create a new neighbor entry */
            ptrNeighEntry = (NetfpProxy_NeighEntry*)malloc (sizeof(NetfpProxy_NeighEntry));
            if (ptrNeighEntry == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to allocate memory for NEIGH Entry\n");
                return;
            }

            /* Initialize the allocated memory */
            memset ((void *)ptrNeighEntry, 0, sizeof(NetfpProxy_NeighEntry));

            /* Populate the tracking entry: */
            memcpy ((void *)&ptrNeighEntry->targetIP, (void*)&message.targetIP, sizeof(Netfp_IPAddr));
            strncpy (ptrNeighEntry->ifName, message.ifName, NETFP_MAX_CHAR);

            /* Build the Netlink Target IP address: */
            if (ptrNeighEntry->targetIP.ver == Netfp_IPVersion_IPV4)
                ptrNeighEntry->ptrNLTargetIP = nl_addr_build (AF_INET, ptrNeighEntry->targetIP.addr.ipv4.u.a8, 4);
            else
                ptrNeighEntry->ptrNLTargetIP = nl_addr_build (AF_INET6, ptrNeighEntry->targetIP.addr.ipv6.u.a8, 16);

            /* Debug Message: */
            nl_addr2str (ptrNeighEntry->ptrNLTargetIP, addressString, sizeof (addressString));
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created Neighbor entry %p for %s\n", ptrNeighEntry, addressString);

            /* Create a listener node for the listener identifier */
            NetfpProxy_createListenerId (ptrNeighEntry, message.listenerId);

            /* Monitor the route and next hop neighbor entry used in this route for any changes. Any changes to link used
             * in route would trigger a route update too. We will process the route update as opposed to link update. */
            ptrNeighEntry->neighEvtSubscHdl = netmgr_register_neigh_updates (gNeighMgmtMCB.netMgrNeighHandle,
                                                                             ptrNeighEntry->ptrNLTargetIP,
                                                                             &NetfpProxy_monitorNeighUpdates,
                                                                             ptrNeighEntry, &errCode);
            if (ptrNeighEntry->neighEvtSubscHdl == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to register for neighbor updates\n");
                break;
            }

            /* Add to the tracking list: */
            List_addNode ((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList, (List_Node*)ptrNeighEntry);

            /***************************************************************************************************************
             * We will now try best effort to see if the neighbor cache already has the next HOP MAC address. In that case
             * we can simply inherit it upfront. If we cannot get it we will kick start the PING and then wait for the
             * neighbor updates.
             ***************************************************************************************************************/
            /* Check the neighbor cache; we might already have this entry in our cache. */
            if (netmgr_neigh_get (gNeighMgmtMCB.netMgrNeighHandle, ptrNeighEntry->ptrNLTargetIP, ptrNeighEntry->ifName, &neighList, &numEntries) == 0)
            {
                /* YES: We have a match; inherit it */
                ptrNeighbor = (struct rtnl_neigh *)neighList[0];

                /* Copy the MAC address: */
                memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)nl_addr_get_binary_addr (rtnl_neigh_get_lladdr (ptrNeighbor)), 6);
                NetfpProxy_neighNotify (ptrNeighEntry, message.listenerId);

                /* Cleanup the links & neighbor list */
                for (index = 0; index < numEntries; index++)
                    nl_object_put ((struct nl_object *) neighList[index]);
                free (neighList);
            }
            else
            {
                /* There is no MAC address resolved. We notify the PROXY and start the neighbor monitoring */
                memset ((void *)&ptrNeighEntry->targetMACAddress[0], 0, 6);
                NetfpProxy_neighNotify (ptrNeighEntry, message.listenerId);
            }
            break;
        }
        case NetfpProxy_NeighMsgType_DELETE:
        {
            /* Do we have a matching neighbor entry? */
            ptrNeighEntry = NetfpProxy_findNeighMgmtEntry(&message.targetIP, &message.ifName[0]);
            if (ptrNeighEntry == NULL)
                break;

            /* Is the listener identifier being tracked? */
            if (NetfpProxy_findListenerId (ptrNeighEntry, message.listenerId) == 0)
                break;

            /* Delete the listener node */
            NetfpProxy_deleteListenerId (ptrNeighEntry, message.listenerId);

            /* If there are no more listener identifiers on the neighbor we can delete it also? */
            if (List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList) == NULL)
            {
                /* Unregister for updates */
                netmgr_unregister_neigh_updates (gNeighMgmtMCB.netMgrNeighHandle, ptrNeighEntry->neighEvtSubscHdl);

                /* We have a valid entry and we now need to remove this from the neighbor list */
                List_removeNode ((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList, (List_Node*)ptrNeighEntry);

                /* Debug Message: */
                nl_addr2str (ptrNeighEntry->ptrNLTargetIP, addressString, sizeof (addressString));
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted Neighbor entry %p for %s\n", ptrNeighEntry, addressString);

                /* Release the target IP address */
                if (ptrNeighEntry->ptrNLTargetIP)
                    nl_addr_put (ptrNeighEntry->ptrNLTargetIP);

                /* Free the allocated memory */
                free (ptrNeighEntry);
            }
            break;
        }
        case NetfpProxy_NeighMsgType_DUMP:
        {
            NetfpProxy_ListenerNode* ptrListenerNode;
//fzm - use NetfpProxy_dumpMsg to capture this with the USR1 signal
            /* Display the Neighbor cache: */
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Neighbor Cache\n");
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");

            /* Cycle through the neighbor managment list */
            ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getHead((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList);
            while (ptrNeighEntry != NULL)
            {
                /* Display String: */
                nl_addr2str (ptrNeighEntry->ptrNLTargetIP, addressString, sizeof (addressString));

                /* Display the neighbor cache: */
                NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "%p: %s MAC 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:%02x IfName:%s [Timer %d]\n",
                                   ptrNeighEntry, addressString, ptrNeighEntry->targetMACAddress[0],
                                   ptrNeighEntry->targetMACAddress[1], ptrNeighEntry->targetMACAddress[2],
                                   ptrNeighEntry->targetMACAddress[3], ptrNeighEntry->targetMACAddress[4],
                                   ptrNeighEntry->targetMACAddress[5], ptrNeighEntry->ifName,
                                   ptrNeighEntry->failedTimeout);

                /* Cycle through all the listener nodes: */
                ptrListenerNode = (NetfpProxy_ListenerNode*)List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList);
                while (ptrListenerNode != NULL)
                {
                    /* Display the listener identifiers. */
                    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    Listener Id: 0x%x\n", ptrListenerNode->listenerId);
                    ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
                }

                /* Get the next neighbor management entry */
                ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getNext ((List_Node*)ptrNeighEntry);
            }

            /* This string is appended at the end of the log to enable collection with FPControl app */
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "************************</netfp_proxy_dump>\n");
            NetfpProxy_dumpDeInit();
            break;
        }
        default:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid message [%d] received at the NEIGH module\n", message.msgType);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked periodically by the NEIGH Module. It cycles through all the neighbors which
 *      have failed and starts the resolution procedure. This is required because if the Neighbor moves
 *      into the failed state we need to restart the ping and try our best to keep the monitor the neighbors
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_neighPoll (void)
{
    NetfpProxy_NeighEntry*  ptrNeighEntry;

    /* Cycle through the neighbor managment list */
    ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getHead((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList);
    while (ptrNeighEntry != NULL)
    {
        /* Is the entry resolved? */
        if (NetfpProxy_neighIsResolved (ptrNeighEntry) == 0)
        {
            /* NO. It is not resolved. Can we send out a ping? */
            ptrNeighEntry->failedTimeout = ptrNeighEntry->failedTimeout - NETFP_PROXY_NEIGH_POLL_DELAY;
            if (ptrNeighEntry->failedTimeout <= 0)
            {
                /* YES: Initiate the ping and send out the data. If the neighbor ever becomes reachable
                 * we will get a neighbor notification and we will then pass that to the PROXY core. */
                NetfpProxy_neighARPPing (&ptrNeighEntry->targetIP, &ptrNeighEntry->ifName[0], 1);

                /* Reset the failed timeout */
                ptrNeighEntry->failedTimeout = NETFP_PROXY_NEIGH_FAILED_STATE_DELAY;
            }
            else
            {
                /* NO. We cannot send out the ping right now. Continue waiting. */
            }
        }

        /* Get the next neighbor management entry */
        ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getNext ((List_Node*)ptrNeighEntry);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP Proxy Neighbor Management thread. The Neighbor
 *      management thread is waiting for data to arrive from the following
 *      modules:-
 *      (a) Proxy Core
 *      (b) NETMGR Interface module
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* NetfpProxy_neighThread(void *arg)
{
    fd_set          fds;
    int32_t         maxFd;
    int32_t         netmgrSocket;
    int32_t         errCode;
    int32_t         numMessages;
    struct timeval  timeout;

    /* Get the NETMGR Socket associated with the neighbor */
    netmgrSocket = netmgr_getSocket(gNeighMgmtMCB.netMgrNeighHandle);

    /* We are waiting for messages to arrive from the PROXY CORE */
    maxFd = max (gNeighMgmtMCB.fromCorePipe[0], netmgrSocket);

    /* Initialize the timeout: */
    timeout.tv_sec  = NETFP_PROXY_NEIGH_POLL_DELAY;
    timeout.tv_usec = 0;

    while (1)
    {
        /* Setup the event FIFO to wait on using select () */
        FD_ZERO (&fds);
        FD_SET (gNeighMgmtMCB.fromCorePipe[0], &fds);
        FD_SET (netmgrSocket, &fds);

        /* Wait for the events to arrive */
        errCode = select ((maxFd + 1), &fds, NULL, NULL, &timeout);
        if (errCode < 0)
        {
            /* If the error code is because of a signal retry again. */
            if (errno == EINTR)
                continue;
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to select on the NEIGH [Error: %s]\n", strerror(errno));
            break;
        }

        /* Is there a timeout? */
        if (errCode == 0)
        {
            /* YES: Poll the neighbors */
            NetfpProxy_neighPoll();

            /* Reset the timeout: */
            timeout.tv_sec  = NETFP_PROXY_NEIGH_POLL_DELAY;
            timeout.tv_usec = 0;
        }

        /* Did we get a message from the PROXY Core? */
        if (FD_ISSET(gNeighMgmtMCB.fromCorePipe[0], &fds) != 0)
            NetfpProxy_neighProcessMessage ();

        /* Did we get a message from the Neighbor NETMGR? */
        if (FD_ISSET(netmgrSocket, &fds) != 0)
        {
            /* Process the NETMGR Neighbor messages: */
            numMessages = netmgr_process_message (gNeighMgmtMCB.netMgrNeighHandle);
            if (numMessages < 0)
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to process the neighbor received message [%s (%d)]\n", nl_geterror(numMessages), numMessages);
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the NEIGH Module. This is executed in the context
 *      of the NETFP Proxy core thread
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_neighExecute (void)
{
    uint32_t        listenerId;
    Netfp_IPAddr    ipAddress;
    uint8_t         macAddress[6];

    /* Neighbor Event has been triggered and so get the lookup status. */
    NetfpProxy_neighGetLookupStatus (&listenerId, &ipAddress, &macAddress[0]);

    /* Pass this information to the route module: */
    NetfpProxy_routeUpdate(listenerId, &ipAddress, &macAddress[0]);
}

/**
 *  @b Description
 *  @n
 *      This is the initialization function which is invoked by the NETFP
 *      Proxy core module to initialize the neighbor management module.
 *
 *      The function initializes the ARP Cache & NDISC module
 *
 *  @retval
 *      Success -   Descriptor used for communication with the core module
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighInit (void)
{
    int32_t         retVal;
    int32_t         errCode;

    /* Initialize the Neighbor management MCB */
    memset ((void *)&gNeighMgmtMCB, 0, sizeof(NetfpProxy_NeighMgmtMCB));

    /* Open the PIPE which is used to communicate with the NETFP Proxy core module */
    if (pipe (gNeighMgmtMCB.fromCorePipe) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to open Core->NEIGH pipe [Error: %s]\n", strerror(errno));
        return -1;
    }

    if(fcntl(gNeighMgmtMCB.fromCorePipe[0], F_SETPIPE_SZ, 1048576) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable set fromCorePipe[0] size: %s", strerror(errno));
        return -1;
    }
    if(fcntl(gNeighMgmtMCB.fromCorePipe[1], F_SETPIPE_SZ, 1048576) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable set fromCorePipe[1] size: %s", strerror(errno));
        return -1;
    }

    /* Open the PIPE which is used to communicate with the NETFP Proxy core module */
    if (pipe (gNeighMgmtMCB.toCorePipe) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to open NEIGH->Core pipe [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the NETMGR for the neighbor module. */
    gNeighMgmtMCB.netMgrNeighHandle = netmgr_init (NetMgr_CacheType_NEIGH, &errCode);
    if (gNeighMgmtMCB.netMgrNeighHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Initialization of the NEIGH NETMGR failed [Error code %d]\n", errCode);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NEIGH NETMGR Initialized\n");

    /* Launch the Neighbor Manager thread */
	retVal = pthread_create (&gNeighMgmtMCB.neighThread, NULL, NetfpProxy_neighThread, NULL);
	if (retVal < 0)
	{
    	NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NEIGH thread create failed error code %d\n", retVal);
        return -1;
	}

    return gNeighMgmtMCB.toCorePipe[0];
}

/**
 *  @b Description
 *  @n
 *      The function is used to return the number of entries in the neighbor cache
 *
 *  @retval
 *      Success -   Number of entries
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighGetNumEntries()
{
    int32_t                 numEntries = 0;
    NetfpProxy_NeighEntry*  ptrNeighEntry;

    /* Cycle through the neighbor managment list */
    ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getHead((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList);
    while (ptrNeighEntry != NULL)
    {
        /* Increment the number of entries*/
        numEntries++;

        /* Get the next neighbor management entry */
        ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getNext ((List_Node*)ptrNeighEntry);
    }
    return numEntries;
}

/**
 *  @b Description
 *  @n
 *      This is the deinitialization function which is invoked by the NETFP Proxy core module
 *      to deinitialize the neighbor managment module.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighDeinit(void)
{
    /* Close the pipes. */
    close (gNeighMgmtMCB.fromCorePipe[0]);
    close (gNeighMgmtMCB.fromCorePipe[1]);
    close (gNeighMgmtMCB.toCorePipe[0]);
    close (gNeighMgmtMCB.toCorePipe[1]);
    return 0;
}

