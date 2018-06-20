/**
 *   @file  ndisc.c
 *
 *   @brief
 *      The file implements the ICMPv6 Neighbor Discovery protocol used by
 *      the NEIGH module to track the MAC address of the Neighbors.
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
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <netinet/if_ether.h>
#include <net/if_arp.h>
#include <netinet/icmp6.h>

/* NETFP Proxy Files. */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**************************************************************************
 ************************* Local Structures *******************************
 **************************************************************************/

/**
 * @brief
 *  Update Reason
 *
 * @details
 *  The enumeration describes the reason for updating the NDISC Cache.
 */
typedef enum NetfpProxy_UpdateReason
{
    /**
     * @brief   Timeout detected
     */
    NetfpProxy_UpdateReason_TIMEOUT     = 0x1,

    /**
     * @brief   ICMPv6 Request/Reply packet received
     */
    NetfpProxy_UpdateReason_PACKET,
}NetfpProxy_UpdateReason;

/**
 * @brief
 *  NDISC Cache State
 *
 * @details
 *  The enumeration describes the state of the NDISC Cache entry
 */
typedef enum NetfpProxy_CacheState
{
    /**
     * @brief   NDISC Cache is INITIAL implying that the MAC address is unresolved and the entry is
     * just created
     */
    NetfpProxy_CacheState_INITIAL        = 0x1,

    /**
     * @brief   NDISC Cache is INACTIVE implying that the MAC address is unresolved.
     */
    NetfpProxy_CacheState_INACTIVE,

    /**
     * @brief   NDISC Cache is ACTIVE implying that the MAC address is resolved
     */
    NetfpProxy_CacheState_ACTIVE,

    /**
     * @brief   NDISC Cache is STALE implying that the MAC address is resolved but the
     * module is in process of revalidating the MAC address
     */
    NetfpProxy_CacheState_STALE
}NetfpProxy_CacheState;

/**
* @brief
*  NETFP Proxy NDISC cache configuration
*
* @details
*  The structure contains the NETFP Proxy NDISC cache configuration information
*  which is passed to the NDISC cache module during initialization.
*/
typedef struct NetfpProxy_NDISCCfg
{
    /**
     * @brief   Basic Polling Timer Tick provided by the NEIGH module
     */
    uint32_t    pollDelay;

    /**
     * @brief   This is the maximum number of retries attempted after which the NDISC cache
     * entry is moved from the STALE to the INACTIVE state.
     */
    uint32_t    maxInactiveRetries;

    /**
     * @brief   Once the NDISC cache entry is moved to the ACTIVE state; the field describes
     * the amount of time after which the entry will be moved to the STALE state.
     */
    uint32_t    activeTimeout;

    /**
     * @brief   This is the maximum number of retries attempted after which the NDISC cache
     * entry is moved from the INITIAL to the INACTIVE state.
     */
    uint32_t    initialRetries;

    /**
     * @brief   Once the NDISC entry is in the INITIAL stage this is the amount of time
     * between BROADCAST packets.
     */
    uint32_t    initialRetransmissionTimeout;

    /**
     * @brief   Once the NDISC cache entry is moved to the STALE state; the field describes
     * the time between directed NDISC requests.
     */
    uint32_t    staleRetransmissionTimeout;

    /**
     * @brief   Once the NDISC cache entry is moved to the INACTIVE state; the field describes
     * the time between broadcast NDISC requests.
     */
    uint32_t    inactiveRetransmissionTimeout;
}NetfpProxy_NDISCCfg;

/**
 * @brief
 *  ICMPv6 Link Layer Option
 *
 * @details
 *  The structure defines the standard ICMPv6 Source Link Layer option
 */
typedef struct NetfpProxy_ICMPv6LLOpt
{
    /**
     * @brief   Source Link Layer Option Type
     */
    uint8_t                 type;

    /**
     * @brief   Length of the option
     */
    uint8_t                 length;

    /**
     * @brief   The link-layer address for the sender.
     */
    uint8_t                 macAddress[6];
}NetfpProxy_ICMPv6LLOpt;

/**
 * @brief
 *  ICMPv6 Neighbor Solicitation Header
 *
 * @details
 *  The structure defines the standard ICMPv6 Neighbor solicitation header
 */
typedef struct NetfpProxy_ICMPv6NSHeader
{
    /**
     * @brief   ICMP message type
     */
    uint8_t                         type;

    /**
     * @brief   Code
     */
    uint8_t                         code;

    /**
     * @brief   ICMP checksum
     */
    uint16_t                        checksum;

    /**
     * @brief   Reserved field; should be set to 0
     */
    uint32_t                        reserved;

    /**
     * @brief   The IP address of the target of the solicitation.
     */
    Netfp_IP6N                      targetAddress;

    /**
     * @brief   Source Link layer option field.
     */
    NetfpProxy_ICMPv6LLOpt          srcLLOpt;
}NetfpProxy_ICMPv6NSHeader;

/**
 * @brief
 *  ICMPv6 Neighbor Advertisment Header
 *
 * @details
 *  The structure defines the standard ICMPv6 Neighbor advertisment header
 */
typedef struct NetfpProxy_ICMPv6NAHeader
{
    /**
     * @brief   ICMP message type
     */
    uint8_t                         type;

    /**
     * @brief   Code
     */
    uint8_t                         code;

    /**
     * @brief   ICMP checksum
     */
    uint16_t                        checksum;

    /**
     * @brief   Flags
     */
    uint8_t                         flags;

    /**
     * @brief   Reserved field
     */
    uint8_t                         reserved[3];

    /**
     * @brief   The IP address which was the target of the solicitation
     */
    Netfp_IP6N                      targetAddress;

    /**
     * @brief   Target Link layer option field.
     */
    NetfpProxy_ICMPv6LLOpt          targetLLOpt;
}NetfpProxy_ICMPv6NAHeader;

/**
 * @brief
 *  NDISC Cache
 *
 * @details
 *  The structure describes the NDISC Cache entry.
 */
typedef struct NetfpProxy_NDISCCache
{
    /**
     * @brief   Links to other NDISC cache entries
     */
    List_Node                       links;

    /**
     * @brief   State of the NDISC cache.
     */
    NetfpProxy_CacheState           state;

    /**
     * @brief   Target IPv4 Address associated with the NDISC cache entry.
     */
    Netfp_IPAddr                    targetIP;

    /**
     * @brief   Sender IPv4 Address associated with the NDISC cache entry.
     */
    Netfp_IPAddr                    senderIP;

    /**
     * @brief   Sender MAC address: This is the MAC address of the interface which is to
     * used to send out the NDISC requests.
     */
    uint8_t                         senderMACAddress[6];

    /**
     * @brief   Interface name associated with the NDISC cache entry.
     */
    char                            ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Timeout maintained in the NDISC cache.
     */
    uint32_t                        timeout;

    /**
     * @brief   Number of retries attempted for the NDISC cache.
     */
    int32_t                         numRetries;

    /**
     * @brief   Next HOP MAC Addresss associated with the IP address
     */
    uint8_t                         macAddress[6];
}NetfpProxy_NDISCCache;

/**
 * @brief
 *  NDISC Cache MCB
 *
 * @details
 *  The structures is the NDISC cache MCB which tracks all the information related
 *  to the module.
 */
typedef struct NetfpProxy_NDISCCacheMCB
{
    /**
     * @brief   NDISC RAW Socket: This is used to send/receive ICMPv6 packets to/from the network
     */
    int32_t                     sock;

    /**
     * @brief   NDISC configuration
     */
    NetfpProxy_NDISCCfg         cfg;

    /**
     * @brief   Pointer to the NDISC cache list
     */
    NetfpProxy_NDISCCache*      ptrNDISCCacheList;

    /**
     * @brief   Statistics which indicates the total number of packets received by the NDISC cache module
     */
    uint64_t                    numPacketsReceived;

    /**
     * @brief   Statistics which indicates the total number of packets which are processed by the
     * NDISC cache module
     */
    uint64_t                    numPacketsProcessed;

    /**
     * @brief   Statistics which indicates the total timeouts invoked by the NETFP Proxy core module.
     */
    uint64_t                    numTimeouts;
}NetfpProxy_NDISCCacheMCB;

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/* Global variable for mapping the NDISC cache */
NetfpProxy_NDISCCacheMCB  gNDISCCacheMCB;

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/

extern void NetfpProxy_neighUpdate(Netfp_IPAddr* ptrIPAddress, char* ifName, uint8_t* ptrMACAddress);
extern int32_t NetfpProxy_neighGetMACAddress(char* ifName, uint8_t* ptrMACAddress);
extern int32_t NetfpProxy_neighGetIPAddress(char* ifName, Netfp_IPAddr* ptrIPAddress);

extern unsigned if_nametoindex(const char *ifname);

/**************************************************************************
 ************************ NDISC Cache Functions ***************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which randomizes a timeout
 *
 *  @retval
 *      Random Timeout
 */
static inline uint8_t NetfpProxy_ndiscRandomizeTimeout (void)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which returns a string matching the NDISC cache state
 *
 *  @param[in]  ptrCache
 *      Pointer to the NDISC cache entry
 *
 *  @retval
 *      Associated String
 */
static inline char* NetfpProxy_arpGetCacheStateString (NetfpProxy_NDISCCache* ptrCache)
{
    switch (ptrCache->state)
    {
        case NetfpProxy_CacheState_INITIAL:
        {
            return "Initial";
        }
        case NetfpProxy_CacheState_INACTIVE:
        {
            return "Inactive";
        }
        case NetfpProxy_CacheState_ACTIVE:
        {
            return "Active";
        }
        case NetfpProxy_CacheState_STALE:
        {
            return "Stale";
        }
        default:
        {
            return "Unknown";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to find display the NDISC cache entry
 *
 *  @param[in]  ptrCache
 *      NDISC cache entry to be displayed
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_ndiscDisplayCacheEntry (NetfpProxy_NDISCCache* ptrCache)
{
    char addressString[256];

    /* Convert the IP6 address to string format */
    Netfp_convertIP6ToStr (ptrCache->targetIP.addr.ipv6, &addressString[0]);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: [%x] %s 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x %s %s\n",
                       ptrCache, addressString, ptrCache->macAddress[0], ptrCache->macAddress[1], ptrCache->macAddress[2],
                       ptrCache->macAddress[3], ptrCache->macAddress[4], ptrCache->macAddress[5],
                       NetfpProxy_arpGetCacheStateString(ptrCache), ptrCache->ifName);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find an NDISC cache entry matching the Target IPv4 address
 *
 *  @param[in]  ptrTargetIP
 *      Target IPv4 address for which the NDISC cache lookup is being done
 *
 *  @retval
 *      Not NULL    - Existing NDISC cache entry is found
 *  @retval
 *      NULL        - No NDISC cache entry
 */
static NetfpProxy_NDISCCache* NetfpProxy_ndiscFindCacheEntry (Netfp_IPAddr* ptrTargetIP)
{
    NetfpProxy_NDISCCache*    ptrCache;

    /* Cycle through the entire cache */
    ptrCache = (NetfpProxy_NDISCCache*)List_getHead ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList);
    while (ptrCache != NULL)
    {
        /* Do we have a match? */
        if ((ptrCache->targetIP.addr.ipv6.u.a32[0] == ptrTargetIP->addr.ipv6.u.a32[0]) &&
            (ptrCache->targetIP.addr.ipv6.u.a32[1] == ptrTargetIP->addr.ipv6.u.a32[1]) &&
            (ptrCache->targetIP.addr.ipv6.u.a32[2] == ptrTargetIP->addr.ipv6.u.a32[2]) &&
            (ptrCache->targetIP.addr.ipv6.u.a32[3] == ptrTargetIP->addr.ipv6.u.a32[3]))
            return ptrCache;

        /* Get the next entry */
        ptrCache = (NetfpProxy_NDISCCache*)List_getNext ((List_Node*)ptrCache);
    }
    return NULL;
}

/**
 *  @b Description
 *  @
 *      The function is used to send an ICMPv6 NDISC packet.
 *
 *  @param[in]  ptrCache
 *      Pointer to the NDISC cache entry for which the request is to be sent.
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_ndiscSendRequest (NetfpProxy_NDISCCache* ptrCache)
{
    NetfpProxy_ICMPv6NSHeader   neighSolicitMessage;
    struct sockaddr_in6         to;

    /* Initialize the neighbor solicitation message */
    memset ((void *)&neighSolicitMessage, 0, sizeof(NetfpProxy_ICMPv6NSHeader));

    /* Populate the message: */
    neighSolicitMessage.type                = ND_NEIGHBOR_SOLICIT;
    neighSolicitMessage.code                = 0;
    neighSolicitMessage.checksum            = 0;
    neighSolicitMessage.reserved            = 0;
    memcpy ((void *)&neighSolicitMessage.targetAddress, (void *)&ptrCache->targetIP.addr.ipv6, sizeof(Netfp_IP6N));
    neighSolicitMessage.srcLLOpt.type       = ND_OPT_SOURCE_LINKADDR;
    neighSolicitMessage.srcLLOpt.length     = 1;
    memcpy ((void *)&neighSolicitMessage.srcLLOpt.macAddress[0], (void*)ptrCache->senderMACAddress, 6);

    /* Initialize the destination */
    memset ((void *)&to, 0, sizeof(to));

#if 1
    /* Send the ICMPv6 NS message to the solicited node multicast group: */
    to.sin6_addr.s6_addr32[0] = htonl(0xFF020000);
    to.sin6_addr.s6_addr32[1] = htonl(0x00000000);
    to.sin6_addr.s6_addr32[2] = htonl(0x00000001);
    to.sin6_addr.s6_addr32[3] = (ptrCache->targetIP.addr.ipv6.u.a32[3] | htonl(0xFF000000));
    to.sin6_addr.s6_addr[13]  = ptrCache->targetIP.addr.ipv6.u.a8[13];
    to.sin6_addr.s6_addr[14]  = ptrCache->targetIP.addr.ipv6.u.a8[14];
    to.sin6_addr.s6_addr[15]  = ptrCache->targetIP.addr.ipv6.u.a8[15];
#else
    /* Send the ICMPv6 NS message to the Target IP address: */
    memcpy ((void *)&to.sin6_addr.s6_addr32[0], (void *)&ptrCache->targetIP.addr.ipv6, sizeof(Netfp_IP6N));
#endif

    /* We are sending the packet over the specified interface name. */
    to.sin6_scope_id = if_nametoindex(ptrCache->ifName);

    /* Send out the NDISC packet: */
    if (sendto (gNDISCCacheMCB.sock, &neighSolicitMessage, sizeof(NetfpProxy_ICMPv6NSHeader), 0,
                (const struct sockaddr *)&to, sizeof (to)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send NDISC request [Error: %s]\n", strerror(errno));
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to update the NDISC Cache entry. NDISC cache entries can be updated
 *      either on the reception of a packet or on timeout.
 *
 *  @param[in]  ptrCache
 *      Pointer to the cache entry being updated
 *  @param[in]  reason
 *      Reason for the cache entry update
 *  @param[in]  ptrMacAddress
 *      Pointer to the MAC address. Set to all 0 if the MAC Address was not resolved else
 *      set to the resolved MAC address
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_ndiscUpdateCacheEntry
(
    NetfpProxy_NDISCCache*  ptrCache,
    NetfpProxy_UpdateReason reason,
    uint8_t*                ptrMacAddress
)
{
    uint8_t     bIsValidMACAddress;
    uint8_t     bIsMACAddressChanged;

    /* Have the function been called from Timeout handling ? */
    if (reason == NetfpProxy_UpdateReason_PACKET)
    {
        /* NO. Are we updating to a valid MAC address? */
        if ((ptrMacAddress[0] == 0x00) && (ptrMacAddress[1] == 0x00) && (ptrMacAddress[2] == 0x00) &&
            (ptrMacAddress[3] == 0x00) && (ptrMacAddress[4] == 0x00) && (ptrMacAddress[5] == 0x00))
        {
            /* NO: The MAC address is not resolved. */
            bIsValidMACAddress = 0;
        }
        else
        {
            /* YES. The MAC address is resolved. */
            bIsValidMACAddress = 1;
        }
    }
    else
    {
        /* YES. MAC Address is not resolved since we dont have the information. */
        bIsValidMACAddress = 0;
    }

    /********************************************************************************************
     * NDISC Cache State Machine:
     ********************************************************************************************/
    switch (ptrCache->state)
    {
        case NetfpProxy_CacheState_INITIAL:
        {
            /* Has the NDISC Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * INITIAL -> ACTIVE State
                 *********************************************************************************/
                ptrCache->state      = NetfpProxy_CacheState_ACTIVE;
                ptrCache->timeout    = gNDISCCacheMCB.cfg.activeTimeout + NetfpProxy_ndiscRandomizeTimeout();
                ptrCache->numRetries = 0;
                memcpy ((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module that the NDISC cache state has transitioned */
                NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* YES. Update the timeout? */
                ptrCache->timeout = ptrCache->timeout - gNDISCCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrCache->timeout == 0)
                {
                    /* YES. Did we exceed all our retries also? */
                    if (ptrCache->numRetries == 0)
                    {
                        /*********************************************************************************
                         * INITIAL -> INACTIVE State
                         *********************************************************************************/
                        ptrCache->state      = NetfpProxy_CacheState_INACTIVE;
                        ptrCache->timeout    = gNDISCCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();
                        ptrCache->numRetries = 0;

                        /* INACTIVE State: This implies that the NDISC entry is unresolved. */
                        memset ((void*)&ptrCache->macAddress[0], 0, 6);

                        /* We need to notify the NETFP Proxy Core module that the NDISC cache state has transitioned */
                        NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
                    }
                    else
                    {
                        /*********************************************************************************
                         * INITIAL -> INITIAL State
                         *********************************************************************************/
                        ptrCache->state   = NetfpProxy_CacheState_INITIAL;
                        ptrCache->timeout = gNDISCCacheMCB.cfg.initialRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();

                        /* Decrement the number of retransmissions */
                        ptrCache->numRetries = ptrCache->numRetries - 1;

                        /* Send out an NDISC request */
                        NetfpProxy_ndiscSendRequest (ptrCache);
                    }
                }
            }
            break;
        }
        case NetfpProxy_CacheState_INACTIVE:
        {
            /* Has the NDISC Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * INACTIVE -> ACTIVE State
                 *********************************************************************************/
                ptrCache->state      = NetfpProxy_CacheState_ACTIVE;
                ptrCache->timeout    = gNDISCCacheMCB.cfg.activeTimeout + NetfpProxy_ndiscRandomizeTimeout();
                ptrCache->numRetries = 0;
                memcpy ((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module that the NDISC cache state has transitioned */
                NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* YES. Update the timeout? */
                ptrCache->timeout = ptrCache->timeout - gNDISCCacheMCB.cfg.pollDelay;

                /* Do we need to send out an NDISC request? */
                if (ptrCache->timeout == 0)
                {
                    /* YES. Time to send out an NDISC request */
                    NetfpProxy_ndiscSendRequest (ptrCache);

                    /* Reset the timeout */
                    ptrCache->timeout = gNDISCCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();
                }
            }
            break;
        }
        case NetfpProxy_CacheState_ACTIVE:
        {
            /* Has the NDISC Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * ACTIVE -> ACTIVE State
                 *********************************************************************************/
                ptrCache->state      = NetfpProxy_CacheState_ACTIVE;
                ptrCache->timeout    = gNDISCCacheMCB.cfg.activeTimeout + NetfpProxy_ndiscRandomizeTimeout();
                ptrCache->numRetries = 0;

                /* Is there a difference in the MAC address? */
                if (memcmp((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6) == 0)
                    bIsMACAddressChanged = 0;
                else
                    bIsMACAddressChanged = 1;

                /* Copy over the MAC Address */
                memcpy ((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module only if the MAC address has changed */
                if (bIsMACAddressChanged == 1)
                    NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* Update the timeout. */
                ptrCache->timeout = ptrCache->timeout - gNDISCCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrCache->timeout == 0)
                {
                    /*********************************************************************************
                     * ACTIVE -> STALE State
                     *********************************************************************************/
                    ptrCache->state      = NetfpProxy_CacheState_STALE;
                    ptrCache->timeout    = gNDISCCacheMCB.cfg.staleRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();
                    ptrCache->numRetries = gNDISCCacheMCB.cfg.maxInactiveRetries;

                    /* Send out the NDISC request: */
                    NetfpProxy_ndiscSendRequest (ptrCache);
                }
            }
            break;
        }
        case NetfpProxy_CacheState_STALE:
        {
            /* Has the NDISC Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * STALE -> ACTIVE State
                 *********************************************************************************/
                ptrCache->state      = NetfpProxy_CacheState_ACTIVE;
                ptrCache->timeout    = gNDISCCacheMCB.cfg.activeTimeout + NetfpProxy_ndiscRandomizeTimeout();
                ptrCache->numRetries = 0;

                /* Is there a difference in the MAC address? */
                if (memcmp((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6) == 0)
                    bIsMACAddressChanged = 0;
                else
                    bIsMACAddressChanged = 1;

                /* Copy over the MAC Address */
                memcpy ((void *)&ptrCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* Notify the NETFP Proxy if there is a MAC address change */
                if (bIsMACAddressChanged == 1)
                    NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* Update the timeout. */
                ptrCache->timeout = ptrCache->timeout - gNDISCCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrCache->timeout == 0)
                {
                    /* YES. Did we exceed all our retries also? */
                    if (ptrCache->numRetries == 0)
                    {
                        /*********************************************************************************
                         * STALE -> INACTIVE State
                         *********************************************************************************/
                        ptrCache->state      = NetfpProxy_CacheState_INACTIVE;
                        ptrCache->timeout    = gNDISCCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();
                        ptrCache->numRetries = 0;

                        /* Reset the MAC Address to all 0 because we are inactive */
                        memset ((void *)&ptrCache->macAddress[0], 0, 6);

                        /* We need to notify the NETFP Proxy Core module only if the MAC address has changed */
                        NetfpProxy_neighUpdate(&ptrCache->targetIP, ptrCache->ifName, &ptrCache->macAddress[0]);
                    }
                    else
                    {
                        /*********************************************************************************
                         * STALE -> STALE State
                         *********************************************************************************/
                        ptrCache->state   = NetfpProxy_CacheState_STALE;
                        ptrCache->timeout = gNDISCCacheMCB.cfg.staleRetransmissionTimeout + NetfpProxy_ndiscRandomizeTimeout();

                        /* Decrement the number of retransmissions */
                        ptrCache->numRetries = ptrCache->numRetries - 1;

                        /* Send out an NDISC request */
                        NetfpProxy_ndiscSendRequest (ptrCache);
                    }
                }
            }
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to display the contents of the NDISC cache. This is invoked
 *      by the NM module for debugging.
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_ndiscDump (void)
{
    NetfpProxy_NDISCCache*      ptrCache;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*********************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "NDISC Table\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*********************************************\n");

    /* Cycle through the NDISC Cache List: */
    ptrCache = (NetfpProxy_NDISCCache*)List_getHead ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList);
    while (ptrCache != NULL)
    {
        /* Display the NDISC Cache: */
        NetfpProxy_ndiscDisplayCacheEntry (ptrCache);

        /* Get the next entry */
        ptrCache = (NetfpProxy_NDISCCache*)List_getNext ((List_Node*)ptrCache);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the monitoring for the specific IP address
 *
 *  @param[in]  ifName
 *      Pointer to the interface name
 *  @param[in]  ptrTargetIP
 *      Pointer to the Target IP address
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ndiscStopPing(char* ifName, Netfp_IPAddr* ptrTargetIP)
{
    NetfpProxy_NDISCCache*      ptrCache;

    /* Find the matching ARP entry */
    ptrCache = NetfpProxy_ndiscFindCacheEntry (ptrTargetIP);
    if (ptrCache == NULL)
        return 0;

    /* Remove the entry from the ARP cache entry: */
    List_removeNode((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList, (List_Node*)ptrCache);

    /* Cleanup the cache entry: */
    free (ptrCache);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to lookup an NDISC cache entry matching the parameters. If there is no
 *      NDISC cache entry found; the function will initiate a lookup for the target IP address.
 *
 *  @param[in]  ifName
 *      Interface Name on which the NDISC resolution is to be done
 *  @param[in]  ptrTargetIP
 *      Pointer to the target IP which is being resolved
 *  @param[out] ptrTargetMACAddress
 *      Pointer to the target MAC Address associated with the target IP
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ndiscStartPing
(
    char*           ifName,
    Netfp_IPAddr*   ptrTargetIP,
    char*           ptrTargetMACAddress
)
{
    NetfpProxy_NDISCCache*      ptrCache;

    /* Are we already monitoring the IPv6 address? */
    ptrCache = NetfpProxy_ndiscFindCacheEntry (ptrTargetIP);
    if (ptrCache != NULL)
    {
        /* YES. NDISC Cache entry already exists which implies that the Target IP address is
         * already in the cache and so we dont need to initiate another lookup. Copy the
         * target MAC Address from the NDISC Cache entry */
        memcpy ((void *)ptrTargetMACAddress, (void *)&ptrCache->macAddress[0], 6);
        return 0;
    }

    /* Allocate a new cache entry: */
    ptrCache = (NetfpProxy_NDISCCache*)malloc (sizeof(NetfpProxy_NDISCCache));
    if (ptrCache == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to allocate NDISC cache memory\n");
        return -1;
    }

    /* Initialize the allocated memory */
    memset ((void*)ptrCache, 0, sizeof(NetfpProxy_NDISCCache));

    /* Get the MAC Address associated with the interface name */
    if (NetfpProxy_neighGetMACAddress(ifName, &ptrCache->senderMACAddress[0]) < 0)
        return -1;

    /* Populate the remaining fields in the NDISC Cache: */
    ptrCache->state         = NetfpProxy_CacheState_INITIAL;
    ptrCache->numRetries    = gNDISCCacheMCB.cfg.initialRetries;
    ptrCache->timeout       = gNDISCCacheMCB.cfg.initialRetransmissionTimeout;
    strncpy (ptrCache->ifName, ifName, NETFP_MAX_CHAR);
    memcpy ((void *)&ptrCache->targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));

    /* Add the NDISC cache entry to the list */
    List_addNode ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList, (List_Node*)ptrCache);

    /* The MAC address is not known at this point in time. We will send out the ICMPv6 request
     * and will notify the neighbor module on an update. But for now this IP address is not
     * reachable. */
    memset ((void *)ptrTargetMACAddress, 0, 6);

    /* Send out the NDISC Request to start the resolution: */
    NetfpProxy_ndiscSendRequest (ptrCache);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process NDISC packets received from the network.
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_ndiscProcessPacket (void)
{
    uint8_t                     packet[1500];
    NetfpProxy_ICMPv6NAHeader*  ptrNAHeader;
    NetfpProxy_ICMPv6NSHeader*  ptrNSHeader;
    Netfp_IPAddr                senderIPAddress;
    struct sockaddr_in6         from;
    NetfpProxy_NDISCCache*      ptrCache;
    char                        addressString[256];
    int32_t                     numBytes;
    size_t                      fromLen = sizeof(from);

    /* Initialize the from address */
    memset ((void *)&from , 0, sizeof(from));

    /* Read the message from the NDISC socket: */
    numBytes = recvfrom (gNDISCCacheMCB.sock, &packet, sizeof(packet), 0, &from, &fromLen);
    if (numBytes < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read packet from the NDISC socket [Error: %s]\n", strerror(errno));
        return;
    }

    /* Sanity Check: Validate the packet length */
    if ((numBytes != sizeof(NetfpProxy_ICMPv6NAHeader)) && (numBytes != sizeof(NetfpProxy_ICMPv6NSHeader)))
        return;

    /* Sanity Check: We only handle the neighbor solititation and advertisment packets. */
    if ((packet[0] != ND_NEIGHBOR_ADVERT) && (packet[0] != ND_NEIGHBOR_SOLICIT))
        return;

    /* Sanity Check: Ensure that the packet code is set to 0 */
    if (packet[1] != 0)
        return;

    /* Get the IP address of the sender of the solicitation message */
    senderIPAddress.ver                = Netfp_IPVersion_IPV6;
    senderIPAddress.addr.ipv6.u.a32[0] = from.sin6_addr.s6_addr32[0];
    senderIPAddress.addr.ipv6.u.a32[1] = from.sin6_addr.s6_addr32[1];
    senderIPAddress.addr.ipv6.u.a32[2] = from.sin6_addr.s6_addr32[2];
    senderIPAddress.addr.ipv6.u.a32[3] = from.sin6_addr.s6_addr32[3];

    /* Was the packet a Neighbor solicitation? */
    if (packet[0] == ND_NEIGHBOR_SOLICIT)
    {
        /* Get the pointer to the neighbor solicitation header */
        ptrNSHeader = (NetfpProxy_ICMPv6NSHeader*)&packet[0];

        /* Sanity Check: Ensure that the source link layer option is specified */
        if (ptrNSHeader->srcLLOpt.type != ND_OPT_SOURCE_LINKADDR)
            return;

        /* Sanity Check: Ensure that the source link layer address is specified */
        if (ptrNSHeader->srcLLOpt.length != 1)
            return;

        /* Debug Message: */
        Netfp_convertIP6ToStr (senderIPAddress.addr.ipv6, &addressString[0]);
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NDISC Neighbor Solicitiation for %s -> 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                           addressString, ptrNSHeader->srcLLOpt.macAddress[0], ptrNSHeader->srcLLOpt.macAddress[1],
                           ptrNSHeader->srcLLOpt.macAddress[2], ptrNSHeader->srcLLOpt.macAddress[3],
                           ptrNSHeader->srcLLOpt.macAddress[4], ptrNSHeader->srcLLOpt.macAddress[5]);

        /* Do we have the sender IP address in our NDISC cache? */
        ptrCache = NetfpProxy_ndiscFindCacheEntry ((Netfp_IPAddr*)&senderIPAddress);
        if (ptrCache != NULL)
        {
            /* YES. We need to update the NDISC cache entry.*/
            NetfpProxy_ndiscUpdateCacheEntry (ptrCache, NetfpProxy_UpdateReason_PACKET, &ptrNSHeader->srcLLOpt.macAddress[0]);
            return;
        }
        return;
    }

    /* Get the pointer to the neighbor advertisment header */
    ptrNAHeader = (NetfpProxy_ICMPv6NAHeader*)&packet[0];

    /* Sanity Check: Ensure that the source link layer option is specified */
    if (ptrNAHeader->targetLLOpt.type != ND_OPT_TARGET_LINKADDR)
        return;

    /* Sanity Check: Ensure that the source link layer address is specified */
    if (ptrNAHeader->targetLLOpt.length != 1)
        return;

    /* Debug Message: */
    Netfp_convertIP6ToStr (senderIPAddress.addr.ipv6, &addressString[0]);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NDISC Neighbor Advetisment for %s -> 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                       addressString, ptrNAHeader->targetLLOpt.macAddress[0], ptrNAHeader->targetLLOpt.macAddress[1],
                       ptrNAHeader->targetLLOpt.macAddress[2], ptrNAHeader->targetLLOpt.macAddress[3],
                       ptrNAHeader->targetLLOpt.macAddress[4], ptrNAHeader->targetLLOpt.macAddress[5]);

    /* Are we monitoring the IP address? */
    ptrCache = NetfpProxy_ndiscFindCacheEntry ((Netfp_IPAddr*)&senderIPAddress);
    if (ptrCache != NULL)
        NetfpProxy_ndiscUpdateCacheEntry (ptrCache, NetfpProxy_UpdateReason_PACKET, &ptrNAHeader->targetLLOpt.macAddress[0]);

    return;
}

/**
 *  @b Description
 *  @n
 *      NDISC Timeout Handler which is invoked periodically to age out the NDISC
 *      cached entries and to implement the NDISC Entry state machine
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_ndiscTimeoutHandler(void)
{
    NetfpProxy_NDISCCache*  ptrCache;
    uint8_t                 dummyMACAddress[6] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

    /* Cycle through the NDISC Cache List: */
    ptrCache = (NetfpProxy_NDISCCache*)List_getHead ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList);
    while (ptrCache != NULL)
    {
        /* Update the NDISC cache entry as per the state machine. */
        NetfpProxy_ndiscUpdateCacheEntry (ptrCache, NetfpProxy_UpdateReason_TIMEOUT, &dummyMACAddress[0]);

        /* Get the next entry */
        ptrCache = (NetfpProxy_NDISCCache*)List_getNext ((List_Node*)ptrCache);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NDISC module.
 *
 *  @param[in]  pollDelay
 *      Polling Granularity Delay in seconds
 *
 *  @retval
 *      Success     -   Pointer to the NDISC raw socket
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_ndiscInit (uint32_t pollDelay)
{
    struct icmp6_filter     icmp6Filter;
    int32_t                 dontRoute = 1;
    int32_t                 hopLimit = 255;
    NetfpProxy_NDISCCfg     ndiscCfg;

    /* Initialize the NDISC Cache configuration: */
    memset ((void *)&ndiscCfg, 0, sizeof(NetfpProxy_NDISCCfg));

    /* Populate the NDISC Cache configuration:
     * - Active Timeout is 60 seconds: Once an entry is marked as ACTIVE it will remain
     *   active till we move it into the STALE stage.
     * - Initial Retransmission timeout is 3 seconds: We want to perform the initial
     *   NDISC resolution as fast as possible.
     * - Stale Retransmission timeout is 60 seconds: The NDISC entry can still be used and
     *   we dont need to agressively send out requests to move it back to ACTIVE.
     *
     * NOTE: The TEST values are for internal testing and can be removed later on. */
#ifdef NDISC_TEST
    ndiscCfg.pollDelay                     = pollDelay;
    ndiscCfg.maxInactiveRetries            = 5;
    ndiscCfg.activeTimeout                 = 5;
    ndiscCfg.initialRetries                = 3;
    ndiscCfg.initialRetransmissionTimeout  = 5;
    ndiscCfg.staleRetransmissionTimeout    = 3;
    ndiscCfg.inactiveRetransmissionTimeout = 5;
#else
    ndiscCfg.pollDelay                     = pollDelay;
    ndiscCfg.maxInactiveRetries            = 5;
    ndiscCfg.activeTimeout                 = 60;
    ndiscCfg.initialRetries                = 3;
    ndiscCfg.initialRetransmissionTimeout  = 3;
    ndiscCfg.staleRetransmissionTimeout    = 60;
    ndiscCfg.inactiveRetransmissionTimeout = 5;
#endif

    /* Initialize the NDISC Cache module */
    memset ((void *)&gNDISCCacheMCB, 0, sizeof(NetfpProxy_NDISCCacheMCB));

    /* Copy the NDISC configuration: */
    memcpy ((void *)&gNDISCCacheMCB.cfg, (void*)&ndiscCfg, sizeof(NetfpProxy_NDISCCfg));

    /* Open the ICMPv6 Raw socket: */
    gNDISCCacheMCB.sock = socket(PF_INET6, SOCK_RAW, IPPROTO_ICMPV6);
	if (gNDISCCacheMCB.sock < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to open the NDISC RAW Socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Set ICMPv6 filter */
    ICMP6_FILTER_SETBLOCKALL (&icmp6Filter);
	ICMP6_FILTER_SETPASS (ND_NEIGHBOR_ADVERT, &icmp6Filter);
	if (setsockopt (gNDISCCacheMCB.sock, IPPROTO_ICMPV6, ICMP6_FILTER, &icmp6Filter, sizeof (struct icmp6_filter)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to setup the ICMPv6 Filter [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Set the DONT Route */
	if (setsockopt (gNDISCCacheMCB.sock, SOL_SOCKET, SO_DONTROUTE, &dontRoute, sizeof (int)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to configure the NDISC socket [Error: %s]\n", strerror(errno));
        return -1;
    }

	/* Set the Hop Limit to 255 as per the RFC */
    if (setsockopt (gNDISCCacheMCB.sock, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &hopLimit, sizeof (hopLimit)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to set the multicast hop limit [Error: %s]\n", strerror(errno));
        return -1;
    }
    if (setsockopt (gNDISCCacheMCB.sock, IPPROTO_IPV6, IPV6_UNICAST_HOPS, &hopLimit, sizeof (hopLimit)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to set the unicast hop limit [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NDISC socket [%d] has been created successfully\n", gNDISCCacheMCB.sock);

    /* Return the NDISC RAW Socket: */
    return gNDISCCacheMCB.sock;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize & shutdown the NDISC module.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_ndiscDeinit (void)
{
    NetfpProxy_NDISCCache*    ptrCache;

    /* Cleanup the NDISC Cache entries */
    ptrCache = (NetfpProxy_NDISCCache*)List_getHead ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList);
    while (ptrCache != NULL)
    {
        /* Cleanup the memory for the NDISC Cache entry */
        free (ptrCache);

        /* Get the new head. */
        ptrCache = (NetfpProxy_NDISCCache*)List_getHead ((List_Node**)&gNDISCCacheMCB.ptrNDISCCacheList);
    }

    /* Close the NDISC RAW Socket */
    close (gNDISCCacheMCB.sock);
    return 0;
}


