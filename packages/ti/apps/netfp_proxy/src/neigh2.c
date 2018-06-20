/**
 *   @file  neigh2.c
 *
 *   @brief
 *      Proxy Neighbor Management Module which implements ARP and NDSIC
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
#include <sys/ioctl.h>
#include <netinet/if_ether.h>
#include <net/if_arp.h>
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
     * @brief   Listener Identifier List associated with the neighbor entry
     */
    NetfpProxy_ListenerNode*    ptrListenerList;

    /**
     * @brief   Target MAC address: This is the MAC address of the target IP after resolution
     * else this is set to 0
     */
    uint8_t                     targetMACAddress[6];

    /**
     * @brief   Interface name on which the resolution was initiated.
     */
    char                        ifName[NETFP_MAX_CHAR];
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
     * @brief   RAW Socket used to send/receive ARP requests/replies
     */
    int32_t                     arpSocket;

    /**
     * @brief   RAW Socket used to send/receive NDISC requests/replies
     */
    int32_t                     ndiscSocket;

    /**
     * @brief   RAW Socket used to process the IPv4 Path MTU
     */
    int32_t                     path4MTUSocket;

    /**
     * @brief   RAW Socket used to process the IPv6 Path MTU
     */
    int32_t                     path6MTUSocket;

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

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

/* ARP */
extern int32_t NetfpProxy_arpInit (uint32_t pollDelay);
extern int32_t NetfpProxy_arpDeinit (void);
extern void NetfpProxy_arpProcessPacket (void);
extern void NetfpProxy_arpDump (void);
extern void NetfpProxy_arpTimeoutHandler(void);
extern int32_t NetfpProxy_arpStartPing (const char* ifName, Netfp_IPAddr* ptrTargetIP, uint8_t* ptrTargetMACAddress);
extern int32_t NetfpProxy_arpStopPing (const char* ifName, Netfp_IPAddr* ptrTargetIP);

/* NDISC */
extern int32_t NetfpProxy_ndiscInit (uint32_t pollDelay);
extern int32_t NetfpProxy_ndiscDeinit (void);
extern void NetfpProxy_ndiscProcessPacket (void);
extern void NetfpProxy_ndiscDump (void);
extern void NetfpProxy_ndiscTimeoutHandler(void);
extern int32_t NetfpProxy_ndiscStartPing (const char* ifName, Netfp_IPAddr* ptrTargetIP, uint8_t* ptrTargetMACAddress);
extern int32_t NetfpProxy_ndiscStopPing (const char* ifName, Netfp_IPAddr* ptrTargetIP);

/* IPv4 Path MTU */
extern int32_t NetfpProxy_pmtu4Init (void);
extern int32_t NetfpProxy_pmtu4Deinit (void);
extern void NetfpProxy_pmtu4ProcessPacket (void);

/* IPv6 Path MTU */
extern int32_t NetfpProxy_pmtu6Init (void);
extern int32_t NetfpProxy_pmtu6Deinit (void);
extern void NetfpProxy_pmtu6ProcessPacket (void);

/**************************************************************************
 **************************** NEIGH Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is a utility function which converts an IP address
 *      to string format.
 *
 *  @param[in]  ptrIPAddress
 *      Pointer to the IP address
 *  @param[in]  strLen
 *      Length of the string
 *  @param[out] string
 *      Output string
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_neighConvertIPToString
(
    Netfp_IPAddr*   ptrIPAddress,
    int32_t         strLen,
    char*           string
)
{
    /* Use the address family to convert the IP address to string: */
    if (ptrIPAddress->ver == Netfp_IPVersion_IPV4)
    {
        snprintf (string, strLen, "%03d.%03d.%03d.%03d", ptrIPAddress->addr.ipv4.u.a8[0],
                  ptrIPAddress->addr.ipv4.u.a8[1], ptrIPAddress->addr.ipv4.u.a8[2],
                  ptrIPAddress->addr.ipv4.u.a8[3]);
    }
    else
    {
        Netfp_convertIP6ToStr (ptrIPAddress->addr.ipv6, string);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the MAC address associated with the interface
 *
 *  @param[in]  ifName
 *      Interface name for which the MAC address is retreived
 *  @param[out]  ptrMACAddress
 *      Populated MAC address retreived for the interface name
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighGetMACAddress(const char* ifName, uint8_t* ptrMACAddress)
{
    struct ifreq    ifr;
    int32_t         sock;

    /* Open a socket: This is required to get the MAC address */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Get the MAC address associated with the interface: */
    strcpy(ifr.ifr_name, ifName);
    if (ioctl(sock, SIOCGIFHWADDR, &ifr) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the MAC Address for %s [Error: %s]\n", ifName, strerror(errno));
        close(sock);
        return -1;
    }

    /* Copy over the MAC Address: */
    memcpy ((void *)ptrMACAddress, (void *)ifr.ifr_hwaddr.sa_data, 6);

    /* Close the socket */
    close (sock);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the IP address associated with the interface
 *
 *  @param[in]  ifName
 *      Interface name for which the MAC address is retreived
 *  @param[out]  ptrIPAddress
 *      Populated IP address retreived for the interface name
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_neighGetIPAddress(const char* ifName, Netfp_IPAddr* ptrIPAddress)
{
    struct ifreq    ifr;
    int32_t         sock;

    /* Open a socket: This is required to get the MAC address */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Get the IPv4 Address: */
    ifr.ifr_addr.sa_family = AF_INET;
    strcpy(ifr.ifr_name, ifName);
    if (ioctl(sock, SIOCGIFADDR, &ifr) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to get the IPv4 Address for %s [Error: %s]\n", ifName, strerror(errno));
        close(sock);
        return -1;
    }

    /* Close the socket: */
    close (sock);

    /* Populate the IPv4 address: */
    ptrIPAddress->ver = Netfp_IPVersion_IPV4;
    ptrIPAddress->addr.ipv4.u.a32 = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
    return 0;
}

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

    /* Initialize the message contents */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType    = NetfpProxy_NeighMsgType_LOOKUP;
    message.listenerId = listenerId;
    memcpy ((void*)&message.targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));
    strncpy (message.ifName, ifName, NETFP_MAX_CHAR);

    /* Send the message to the Neighbor Management Module */
    write (gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg));
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

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType    = NetfpProxy_NeighMsgType_DELETE;
    message.listenerId = listenerId;
    memcpy ((void*)&message.targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));
    strncpy (message.ifName, ifName, NETFP_MAX_CHAR);

    /* Send the message: */
    write (gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg));
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

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Populate the message: */
    message.msgType = NetfpProxy_NeighMsgType_DUMP;

    /* Send the message: */
    write (gNeighMgmtMCB.fromCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg));
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

    /* Initialize the message: */
    memset ((void *)&message, 0, sizeof(NetfpProxy_NeighMsg));

    /* Read the message from the */
    if (read (gNeighMgmtMCB.toCorePipe[0], &message, sizeof(NetfpProxy_NeighMsg)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read from the toCore Pipe [Error: %s]\n", strerror(errno));
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
static void NetfpProxy_neighNotify
(
    NetfpProxy_NeighEntry*  ptrNeighEntry,
    uint32_t                listenerId
)
{
    NetfpProxy_NeighMsg         message;
    NetfpProxy_ListenerNode*    ptrListenerNode;

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
            write (gNeighMgmtMCB.toCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg));

            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Sending notification to Neighbor 0x%x Listener Id 0x%x\n",
                               ptrNeighEntry, message.listenerId);

            /* Get the next listener node: */
            ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
        }
    }
    else
    {
        /* DIRECTED: Populate the listener identifier in the message and send it to the core. */
        message.listenerId = listenerId;
        write (gNeighMgmtMCB.toCorePipe[1], &message, sizeof(NetfpProxy_NeighMsg));

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Sending notification to Neighbor 0x%x Listener Id 0x%x\n",
                           ptrNeighEntry, message.listenerId);
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
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Neighbor 0x%x created listener id 0x%x\n", ptrNeighEntry, listenerId);
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
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Neighbor 0x%x deleted listener id 0x%x\n", ptrNeighEntry, listenerId);

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
    char*           ifName
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
    char                    addressString[256];

    /* Read the message from the PROXY */
    if (read (gNeighMgmtMCB.fromCorePipe[0], &message, sizeof(NetfpProxy_NeighMsg)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read from the PROXY Pipe [Error: %s]\n", strerror(errno));
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

            /* Debug Message: */
            NetfpProxy_neighConvertIPToString (&ptrNeighEntry->targetIP, sizeof(addressString), &addressString[0]);
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created Neighbor entry 0x%x for %s\n", ptrNeighEntry, addressString);

            /* Create a listener node for the listener identifier */
            NetfpProxy_createListenerId (ptrNeighEntry, message.listenerId);

            /* Notify the ARP/NDISC that we are interested in the the target IP: */
            if (ptrNeighEntry->targetIP.ver == Netfp_IPVersion_IPV4)
                NetfpProxy_arpStartPing (&ptrNeighEntry->ifName[0], &ptrNeighEntry->targetIP, &ptrNeighEntry->targetMACAddress[0]);
            else
                NetfpProxy_ndiscStartPing (&ptrNeighEntry->ifName[0], &ptrNeighEntry->targetIP, &ptrNeighEntry->targetMACAddress[0]);

            /* Notify the listener immediately about the status. */
            NetfpProxy_neighNotify (ptrNeighEntry, message.listenerId);

            /* Add to the tracking list: */
            List_addNode ((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList, (List_Node*)ptrNeighEntry);
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
                /* Inform the ARP/NDISC layer; to stop monitoring this IP address */
                if (ptrNeighEntry->targetIP.ver == Netfp_IPVersion_IPV4)
                    NetfpProxy_arpStopPing (&message.ifName[0], &message.targetIP);
                else
                    NetfpProxy_ndiscStopPing (&message.ifName[0], &message.targetIP);

                /* We have a valid entry and we now need to remove this from the neighbor list */
                List_removeNode ((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList, (List_Node*)ptrNeighEntry);

                /* Debug Message: */
                NetfpProxy_neighConvertIPToString (&ptrNeighEntry->targetIP, sizeof(addressString), &addressString[0]);
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleted Neighbor entry 0x%x for %s\n", ptrNeighEntry, addressString);

                /* Free the allocated memory */
                free (ptrNeighEntry);
            }
            break;
        }
        case NetfpProxy_NeighMsgType_DUMP:
        {
            NetfpProxy_ListenerNode* ptrListenerNode;

            /* Display the Neighbor cache: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Neighbor Cache\n");
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*******************************************\n");

            /* Cycle through the neighbor managment list */
            ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getHead((List_Node**)&gNeighMgmtMCB.ptrNeighMgmtList);
            while (ptrNeighEntry != NULL)
            {
                /* Convert the IPv4/IPv6 address to string format: */
                NetfpProxy_neighConvertIPToString (&ptrNeighEntry->targetIP, sizeof(addressString), &addressString[0]);

                /* Display the neighbor cache: */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "0x%x: %s MAC 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:%02x IfName:%s\n",
                                   ptrNeighEntry, addressString, ptrNeighEntry->targetMACAddress[0],
                                   ptrNeighEntry->targetMACAddress[1], ptrNeighEntry->targetMACAddress[2],
                                   ptrNeighEntry->targetMACAddress[3], ptrNeighEntry->targetMACAddress[4],
                                   ptrNeighEntry->targetMACAddress[5], ptrNeighEntry->ifName);

                /* Cycle through all the listener nodes: */
                ptrListenerNode = (NetfpProxy_ListenerNode*)List_getHead ((List_Node**)&ptrNeighEntry->ptrListenerList);
                while (ptrListenerNode != NULL)
                {
                    /* Display the listener identifiers. */
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "    Listener Id: 0x%x\n", ptrListenerNode->listenerId);
                    ptrListenerNode = (NetfpProxy_ListenerNode*)List_getNext ((List_Node*)ptrListenerNode);
                }

                /* Get the next neighbor management entry */
                ptrNeighEntry = (NetfpProxy_NeighEntry*)List_getNext ((List_Node*)ptrNeighEntry);
            }

            /* Display the ARP & NDISC caches */
            NetfpProxy_arpDump();
            NetfpProxy_ndiscDump();
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
 *      The function is used to update the NETFP Proxy neighbor about the status of an IP address
 *      which was being monitored. The function is exported and is available to the ARP and NDISC
 *      module to asynchronously update the NEIGH module about an update in the cache status.
 *
 *  @param[in]  ptrIPAddress
 *      Pointer to the IP address being monitored
 *  @param[in]  ifName
 *      Interface name
 *  @param[in]  ptrMACAddress
 *      Pointer to the new MAC address which is to be updated
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_neighUpdate
(
    Netfp_IPAddr* ptrIPAddress,
    const char*   ifName,
    uint8_t*      ptrMACAddress
)
{
    NetfpProxy_NeighEntry*  ptrNeighEntry;
    uint32_t                isAddressResolved;
    uint8_t                 zeroMACAddress[6] = { 0 };
    char                    addressString[256];

    /* Convert the IP address to string: */
    NetfpProxy_neighConvertIPToString (ptrIPAddress, sizeof(addressString), &addressString[0]);

    /* Find the neighbor entry for which we have received an update */
    ptrNeighEntry = NetfpProxy_findNeighMgmtEntry(ptrIPAddress, ifName);
    if (ptrNeighEntry == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: No Neighbor for %s Interface: %s\n", addressString, ifName);
        return;
    }

    /* Was the neighbor entry resolved or not? */
    if (memcmp ((void *)ptrMACAddress, (void*)&zeroMACAddress, 6) == 0)
        isAddressResolved = 0;
    else
        isAddressResolved = 1;

    /* Do we need to notify all the listeners? */
    if (memcmp ((void *)ptrMACAddress, (void *)&ptrNeighEntry->targetMACAddress[0], 6) != 0)
    {
        /* Different MAC Address: Notify the update to all registered listeners */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: [%s] MAC address Neighbor %s %02x:%02x:%02x:%02x:%02x:%02x %s\n",
                           (isAddressResolved == 1) ? "Resolved" : "Unresolved", addressString,
                           *ptrMACAddress, *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3),
                           *(ptrMACAddress + 4), *(ptrMACAddress + 5), ifName);

        /* Copy over the MAC address */
        memcpy ((void *)&ptrNeighEntry->targetMACAddress[0], (void *)ptrMACAddress, 6);
        NetfpProxy_neighNotify (ptrNeighEntry, NETFP_PROXY_BROADCAST_NOTIFICATION);
    }
    else
    {
        /* Same old MAC address: No need to update */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: [%s] Skipping MAC address notification Neighbor %s %02x:%02x:%02x:%02x:%02x:%02x %s\n",
                           (isAddressResolved == 1) ? "Resolved" : "Unresolved", addressString,
                           *ptrMACAddress, *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3),
                           *(ptrMACAddress + 4), *(ptrMACAddress + 5), ifName);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the NETFP Proxy Neighbor Management thread. The Neighbor
 *      management thread is waiting for data to arrive from the following
 *      modules:-
 *      (a) Proxy Core
 *      (b) ARP
 *      (c) NDISC6
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
    int32_t         errCode;
    struct timeval  timeout;

    /* We are waiting for messages to arrive from the PROXY CORE */
    maxFd = max (gNeighMgmtMCB.fromCorePipe[0], gNeighMgmtMCB.arpSocket);
    maxFd = max (maxFd, gNeighMgmtMCB.ndiscSocket);
    maxFd = max (maxFd, gNeighMgmtMCB.path4MTUSocket);
    maxFd = max (maxFd, gNeighMgmtMCB.path6MTUSocket);

    /* Initialize the timeout: */
    timeout.tv_sec  = NETFP_PROXY_NEIGH_POLL_DELAY;
    timeout.tv_usec = 0;

    while (1)
    {
        /* Setup the event FIFO to wait on using select () */
        FD_ZERO (&fds);
        FD_SET (gNeighMgmtMCB.fromCorePipe[0], &fds);
        FD_SET (gNeighMgmtMCB.arpSocket, &fds);
        FD_SET (gNeighMgmtMCB.ndiscSocket, &fds);
        FD_SET (gNeighMgmtMCB.path4MTUSocket, &fds);
        FD_SET (gNeighMgmtMCB.path6MTUSocket, &fds);

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
            NetfpProxy_arpTimeoutHandler();
            NetfpProxy_ndiscTimeoutHandler();

            /* Reset the timeout: */
            timeout.tv_sec  = NETFP_PROXY_NEIGH_POLL_DELAY;
            timeout.tv_usec = 0;
        }

        /* Did we get a message from the PROXY Core? */
        if (FD_ISSET(gNeighMgmtMCB.fromCorePipe[0], &fds) != 0)
            NetfpProxy_neighProcessMessage ();

        /* Did we get a message from the ARP socket? */
        if (FD_ISSET(gNeighMgmtMCB.arpSocket, &fds) != 0)
            NetfpProxy_arpProcessPacket();

        /* Did we get a message from the NDISC socket? */
        if (FD_ISSET(gNeighMgmtMCB.ndiscSocket, &fds) != 0)
            NetfpProxy_ndiscProcessPacket();

        /* Did we get a message from the IPv4 Path MTU socket? */
        if (FD_ISSET(gNeighMgmtMCB.path4MTUSocket, &fds) != 0)
            NetfpProxy_pmtu4ProcessPacket();

        /* Did we get a message from the IPv6 Path MTU socket? */
        if (FD_ISSET(gNeighMgmtMCB.path6MTUSocket, &fds) != 0)
            NetfpProxy_pmtu6ProcessPacket();
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
    int32_t retVal;

    /* Initialize the Neighbor management MCB */
    memset ((void *)&gNeighMgmtMCB, 0, sizeof(NetfpProxy_NeighMgmtMCB));

    /* Open the PIPE which is used to communicate with the NETFP Proxy core module */
    if (pipe (gNeighMgmtMCB.fromCorePipe) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to open Core->NEIGH pipe [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Open the PIPE which is used to communicate with the NETFP Proxy core module */
    if (pipe (gNeighMgmtMCB.toCorePipe) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to open NEIGH->Core pipe [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Initialize the ARP Cache: */
    gNeighMgmtMCB.arpSocket = NetfpProxy_arpInit (NETFP_PROXY_NEIGH_POLL_DELAY);
    if (gNeighMgmtMCB.arpSocket < 0)
        return -1;

    /* Initialize the NDISC Cache: */
    gNeighMgmtMCB.ndiscSocket = NetfpProxy_ndiscInit (NETFP_PROXY_NEIGH_POLL_DELAY);
    if (gNeighMgmtMCB.ndiscSocket < 0)
        return -1;

    /* Initialize the IPv4 Path MTU Module: */
    gNeighMgmtMCB.path4MTUSocket = NetfpProxy_pmtu4Init ();
    if (gNeighMgmtMCB.path4MTUSocket < 0)
        return -1;

    /* Initialize the IPv6 Path MTU Module: */
    gNeighMgmtMCB.path6MTUSocket = NetfpProxy_pmtu6Init ();
    if (gNeighMgmtMCB.path6MTUSocket < 0)
        return -1;

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
    /* Close the ARP module */
    NetfpProxy_arpDeinit();

    /* Close the NDISC module */
    NetfpProxy_ndiscDeinit();

    /* Close the IPv4 Path MTU Module */
    NetfpProxy_pmtu4Deinit();

    /* Close the IPv6 Path MTU Module */
    NetfpProxy_pmtu6Deinit();

    /* Close the pipes. */
    close (gNeighMgmtMCB.fromCorePipe[0]);
    close (gNeighMgmtMCB.fromCorePipe[1]);
    close (gNeighMgmtMCB.toCorePipe[0]);
    close (gNeighMgmtMCB.toCorePipe[1]);
    return 0;
}

