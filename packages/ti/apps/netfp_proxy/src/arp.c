/**
 *   @file  arp.c
 *
 *   @brief
 *      The file implements the ARP protocol used by the NEIGH module
 *      to track the MAC address of the Neighbors.
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
 *  The enumeration describes the reason for updating the ARP Cache.
 */
typedef enum NetfpProxy_UpdateReason
{
    /**
     * @brief   Timeout detected
     */
    NetfpProxy_UpdateReason_TIMEOUT     = 0x1,

    /**
     * @brief   ARP Request/Reply packet received
     */
    NetfpProxy_UpdateReason_PACKET,
}NetfpProxy_UpdateReason;

/**
 * @brief
 *  ARP Cache State
 *
 * @details
 *  The enumeration describes the state of the ARP Cache
 */
typedef enum NetfpProxy_ARPCacheState
{
    /**
     * @brief   ARP Cache is INITIAL implying that the MAC address is unresolved and the entry is
     * just created
     */
    NetfpProxy_ARPCacheState_INITIAL        = 0x1,

    /**
     * @brief   ARP Cache is INACTIVE implying that the MAC address is unresolved.
     */
    NetfpProxy_ARPCacheState_INACTIVE,

    /**
     * @brief   ARP Cache is ACTIVE implying that the MAC address is resolved
     */
    NetfpProxy_ARPCacheState_ACTIVE,

    /**
     * @brief   ARP Cache is STALE implying that the MAC address is resolved but the
     * module is in process of revalidating the MAC address
     */
    NetfpProxy_ARPCacheState_STALE
}NetfpProxy_ARPCacheState;

/**
* @brief
*  NETFP Proxy ARP cache configuration
*
* @details
*  The structure contains the NETFP Proxy ARP cache configuration information
*  which is passed to the ARP cache module during initialization.
*/
typedef struct NetfpProxy_ARPCfg
{
    /**
     * @brief   Basic Polling Timer Tick provided by the NEIGH module
     */
    uint32_t    pollDelay;

    /**
     * @brief   This is the maximum number of retries attempted after which the ARP cache
     * entry is moved from the STALE to the INACTIVE state.
     */
    uint32_t    maxInactiveRetries;

    /**
     * @brief   Once the ARP cache entry is moved to the ACTIVE state; the field describes
     * the amount of time after which the entry will be moved to the STALE state.
     */
    uint32_t    activeTimeout;

    /**
     * @brief   This is the maximum number of retries attempted after which the ARP cache
     * entry is moved from the INITIAL to the INACTIVE state.
     */
    uint32_t    initialRetries;

    /**
     * @brief   Once the ARP entry is in the INITIAL stage this is the amount of time
     * between BROADCAST packets.
     */
    uint32_t    initialRetransmissionTimeout;

    /**
     * @brief   Once the ARP cache entry is moved to the STALE state; the field describes
     * the time between directed ARP requests.
     */
    uint32_t    staleRetransmissionTimeout;

    /**
     * @brief   Once the ARP cache entry is moved to the INACTIVE state; the field describes
     * the time between broadcast ARP requests.
     */
    uint32_t    inactiveRetransmissionTimeout;
}NetfpProxy_ARPCfg;

/**
 * @brief
 *  NETFP Proxy ARP Header
 *
 * @details
 *  The structure describes the standard ARP Header
 */
typedef struct NetfpProxy_ARPHeader
{
    /**
     * @brief   The field specifies the type of the hardware used for the local network.
     */
	uint16_t    hardwareType;

    /**
     * @brief   This field is the complement of the Hardware Type field, specifying the type
     * of layer three addresses used in the message
     */
	uint16_t    protocolType;

    /**
     * @brief   Specifies how long hardware addresses are in this message.
     */
	uint8_t     hardwareAddrLen;

    /**
     * @brief   Specifies how long protocol addresses are in this message
     */
	uint8_t     protocolAddrLen;

    /**
     * @brief   This field specifies the nature of the ARP message being sent
     */
	uint16_t    arpOpType;

    /**
     * @brief   The hardware address of the device sending this message
     */
	uint8_t     senderHwAddress[6];

    /**
     * @brief   The IP address of the device sending this message.
     */
	uint8_t     senderIPAddress[4];

    /**
     * @brief   The hardware address of the device this message is being sent to.
     */
	uint8_t     targetHwAddress[6];

    /**
     * @brief   The IP address of the device this message is being sent to.
     */
	uint8_t     targetIPAddress[4];
}NetfpProxy_ARPHeader;

/**
 * @brief
 *  ARP Cache
 *
 * @details
 *  The structure describes the ARP Cache entry.
 */
typedef struct NetfpProxy_ARPCache
{
    /**
     * @brief   Links to other ARP cache entries
     */
    List_Node                       links;

    /**
     * @brief   State of the ARP cache.
     */
    NetfpProxy_ARPCacheState        state;

    /**
     * @brief   Target IPv4 Address associated with the ARP cache entry.
     */
    Netfp_IPAddr                    targetIP;

    /**
     * @brief   Target IPv4 Address associated with the ARP cache entry.
     */
    Netfp_IPAddr                    senderIP;

    /**
     * @brief   Sender MAC address: This is the MAC address of the interface which is to
     * used to send out the ARP requests.
     */
    uint8_t                         senderMACAddress[6];

    /**
     * @brief   Interface name associated with the ARP cache entry.
     */
    char                            ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Timeout maintained in the ARP cache.
     */
    uint32_t                        timeout;

    /**
     * @brief   Number of retries attempted for the ARP cache.
     */
    int32_t                         numRetries;

    /**
     * @brief   Next HOP MAC Addresss associated with the IP address
     */
    uint8_t                         macAddress[6];
}NetfpProxy_ARPCache;

/**
 * @brief
 *  ARP Cache MCB
 *
 * @details
 *  The structures is the ARP cache MCB which tracks all the information related to the
 *  module.
 */
typedef struct NetfpProxy_ARPCacheMCB
{
    /**
     * @brief   ARP configuration
     */
    NetfpProxy_ARPCfg           cfg;

    /**
     * @brief   ARP RAW Socket: This is used to send/receive ARP packets to/from the network
     */
    int32_t                     sock;

    /**
     * @brief   Pointer to the ARP cache list
     */
    NetfpProxy_ARPCache*        ptrARPCacheList;

    /**
     * @brief   Statistics which indicates the total number of packets received by the ARP cache module
     */
    uint64_t                    numPacketsReceived;

    /**
     * @brief   Statistics which indicates the total number of packets which are processed by the
     * ARP cache module
     */
    uint64_t                    numPacketsProcessed;

    /**
     * @brief   Statistics which indicates the total timeouts invoked by the NETFP Proxy core module.
     */
    uint64_t                    numTimeouts;
}NetfpProxy_ARPCacheMCB;

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/* Global variable for mapping the ARP cache */
NetfpProxy_ARPCacheMCB  gARPCacheMCB;

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/

extern void NetfpProxy_neighUpdate(Netfp_IPAddr* ptrIPAddress, char* ifName, uint8_t* ptrMACAddress);
extern int32_t NetfpProxy_neighGetMACAddress(char* ifName, uint8_t* ptrMACAddress);
extern int32_t NetfpProxy_neighGetIPAddress(char* ifName, Netfp_IPAddr* ptrIPAddress);

/**************************************************************************
 ************************* ARP Cache Functions ****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which randomizes a timeout
 *
 *  @retval
 *      Random Timeout
 */
static inline uint8_t NetfpProxy_arpRandomizeTimeout (void)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which returns a string matching the ARP cache state
 *
 *  @param[in]  ptrARPCache
 *      Pointer to the ARP Cache
 *
 *  @retval
 *      Associated String
 */
static inline char* NetfpProxy_arpGetCacheStateString (NetfpProxy_ARPCache* ptrARPCache)
{
    switch (ptrARPCache->state)
    {
        case NetfpProxy_ARPCacheState_INITIAL:
        {
            return "Initial";
        }
        case NetfpProxy_ARPCacheState_INACTIVE:
        {
            return "Inactive";
        }
        case NetfpProxy_ARPCacheState_ACTIVE:
        {
            return "Active";
        }
        case NetfpProxy_ARPCacheState_STALE:
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
 *      The function is used to find display the ARP cache entry
 *
 *  @param[in]  ptrARPCache
 *      ARP cache entry to be displayed
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_arpDisplayCacheEntry (NetfpProxy_ARPCache* ptrARPCache)
{
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: [%x] %03d.%03d.%03d.%03d 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x %s %s\n",
                       ptrARPCache,
                       ptrARPCache->targetIP.addr.ipv4.u.a8[0], ptrARPCache->targetIP.addr.ipv4.u.a8[1],
                       ptrARPCache->targetIP.addr.ipv4.u.a8[2], ptrARPCache->targetIP.addr.ipv4.u.a8[3],
                       ptrARPCache->macAddress[0], ptrARPCache->macAddress[1], ptrARPCache->macAddress[2],
                       ptrARPCache->macAddress[3], ptrARPCache->macAddress[4], ptrARPCache->macAddress[5],
                       NetfpProxy_arpGetCacheStateString(ptrARPCache), ptrARPCache->ifName);
}

/**
 *  @b Description
 *  @n
 *      The function is used to find an ARP cache entry matching the Target IPv4 address
 *
 *  @param[in]  ptrTargetIP
 *      Target IPv4 address for which the ARP cache lookup is being done
 *
 *  @retval
 *      Not NULL    - Existing ARP cache entry is found
 *  @retval
 *      NULL        - No ARP cache entry
 */
static NetfpProxy_ARPCache* NetfpProxy_arpFindCacheEntry (Netfp_IPAddr* ptrTargetIP)
{
    NetfpProxy_ARPCache*    ptrARPCache;

    /* Cycle through the entire cache */
    ptrARPCache = (NetfpProxy_ARPCache*)List_getHead ((List_Node**)&gARPCacheMCB.ptrARPCacheList);
    while (ptrARPCache != NULL)
    {
        /* Do we have a match? */
        if (ptrARPCache->targetIP.addr.ipv4.u.a32 == ptrTargetIP->addr.ipv4.u.a32)
            return ptrARPCache;

        /* Get the next entry */
        ptrARPCache = (NetfpProxy_ARPCache*)List_getNext ((List_Node*)ptrARPCache);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send an ARP request packet. The parameters in the ARP
 *      request packet are derived from the status of the ARP cache entry.
 *
 *  @param[in]  ptrARPCache
 *      Pointer to the ARP cache entry for which the ARP request is being sent
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_arpSendRequest (NetfpProxy_ARPCache* ptrARPCache)
{
    char                    arpPacket[128];
    NetfpProxy_ARPHeader*   ptrARPHeader;
    struct sockaddr         addr;

    /* Initialize the packet */
    memset ((void *)&arpPacket[0], 0, sizeof(arpPacket));

    /* Populate the Destination MAC address: */
    if ((ptrARPCache->state == NetfpProxy_ARPCacheState_ACTIVE) || (ptrARPCache->state == NetfpProxy_ARPCacheState_STALE))
    {
        /* If the ARP Cache entry is in the ACTIVE or STALE state; we will send directed ARP requests */
        arpPacket[0] = ptrARPCache->macAddress[0];
        arpPacket[1] = ptrARPCache->macAddress[1];
        arpPacket[2] = ptrARPCache->macAddress[2];
        arpPacket[3] = ptrARPCache->macAddress[3];
        arpPacket[4] = ptrARPCache->macAddress[4];
        arpPacket[5] = ptrARPCache->macAddress[5];
    }
    else
    {
        /* In the INITIAL or INACTIVE states; we will be sending BROADCAST packets */
        arpPacket[0] = 0xFF;
        arpPacket[1] = 0xFF;
        arpPacket[2] = 0xFF;
        arpPacket[3] = 0xFF;
        arpPacket[4] = 0xFF;
        arpPacket[5] = 0xFF;
    }

    /* Populate the source: */
    arpPacket[6]  = ptrARPCache->senderMACAddress[0];
    arpPacket[7]  = ptrARPCache->senderMACAddress[1];
    arpPacket[8]  = ptrARPCache->senderMACAddress[2];
    arpPacket[9]  = ptrARPCache->senderMACAddress[3];
    arpPacket[10] = ptrARPCache->senderMACAddress[4];
    arpPacket[11] = ptrARPCache->senderMACAddress[5];

    /* Populate the protocol type: */
    arpPacket[12] = 0x08;
    arpPacket[13] = 0x06;

    /* Get the pointer to the ARP header: */
    ptrARPHeader = (NetfpProxy_ARPHeader*)&arpPacket[14];

    /* Populate the ARP Header: */
    ptrARPHeader->hardwareType    = htons(ARPHRD_ETHER);
    ptrARPHeader->protocolType    = htons(ETH_P_IP);
	ptrARPHeader->hardwareAddrLen = 6;
    ptrARPHeader->protocolAddrLen = 4;
    ptrARPHeader->arpOpType       = htons(ARPOP_REQUEST);
    memcpy ((void *)&ptrARPHeader->senderHwAddress[0], (void *)&ptrARPCache->senderMACAddress, 6);
    memcpy ((void *)&ptrARPHeader->senderIPAddress[0], (void *)&ptrARPCache->senderIP.addr.ipv4.u.a8[0], 4);
    memset ((void *)&ptrARPHeader->targetHwAddress[0], 0, 6);
    memcpy ((void *)&ptrARPHeader->targetIPAddress[0], (void *)&ptrARPCache->targetIP.addr.ipv4.u.a8[0], 4);

    /* We are sending the packet over the specified interface name. */
	memset(&addr, 0, sizeof(addr));
	strcpy(addr.sa_data, ptrARPCache->ifName);

    /* Send out the packet. */
	if (sendto(gARPCacheMCB.sock, &arpPacket, (sizeof(NetfpProxy_ARPHeader) + ETH_HLEN), 0, &addr, sizeof(addr)) < 0)
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to send ARP request [Error: %s]\n", strerror(errno));
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to update the ARP Cache entry. ARP cache entries can be updated
 *      either on the reception of a packet or on timeout.
 *
 *  @param[in]  ptrARPCache
 *      Pointer to the ARP cache entry being updated
 *  @param[in]  reason
 *      Reason for the ARP cache entry update
 *  @param[in]  ptrMacAddress
 *      Pointer to the MAC address. Set to all 0 if the MAC Address was not resolved else
 *      set to the resolved MAC address
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_arpUpdateCacheEntry
(
    NetfpProxy_ARPCache*    ptrARPCache,
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
     * ARP Cache State Machine:
     ********************************************************************************************/
    switch (ptrARPCache->state)
    {
        case NetfpProxy_ARPCacheState_INITIAL:
        {
            /* Has the ARP Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * INITIAL -> ACTIVE State
                 *********************************************************************************/
                ptrARPCache->state      = NetfpProxy_ARPCacheState_ACTIVE;
                ptrARPCache->timeout    = gARPCacheMCB.cfg.activeTimeout + NetfpProxy_arpRandomizeTimeout();
                ptrARPCache->numRetries = 0;
                memcpy ((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module that the ARP cache state has transitioned */
                NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* YES. Update the timeout? */
                ptrARPCache->timeout = ptrARPCache->timeout - gARPCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrARPCache->timeout == 0)
                {
                    /* YES. Did we exceed all our retries also? */
                    if (ptrARPCache->numRetries == 0)
                    {
                        /*********************************************************************************
                         * INITIAL -> INACTIVE State
                         *********************************************************************************/
                        ptrARPCache->state      = NetfpProxy_ARPCacheState_INACTIVE;
                        ptrARPCache->timeout    = gARPCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();
                        ptrARPCache->numRetries = 0;

                        /* INACTIVE State: This implies that the ARP entry is unresolved. */
                        memset ((void*)&ptrARPCache->macAddress[0], 0, 6);

                        /* We need to notify the NETFP Proxy Core module that the ARP cache state has transitioned */
                        NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
                    }
                    else
                    {
                        /*********************************************************************************
                         * INITIAL -> INITIAL State
                         *********************************************************************************/
                        ptrARPCache->state   = NetfpProxy_ARPCacheState_INITIAL;
                        ptrARPCache->timeout = gARPCacheMCB.cfg.initialRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();

                        /* Decrement the number of retransmissions */
                        ptrARPCache->numRetries = ptrARPCache->numRetries - 1;

                        /* Send out an ARP request */
                        NetfpProxy_arpSendRequest (ptrARPCache);
                    }
                }
            }
            break;
        }
        case NetfpProxy_ARPCacheState_INACTIVE:
        {
            /* Has the ARP Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * INACTIVE -> ACTIVE State
                 *********************************************************************************/
                ptrARPCache->state      = NetfpProxy_ARPCacheState_ACTIVE;
                ptrARPCache->timeout    = gARPCacheMCB.cfg.activeTimeout + NetfpProxy_arpRandomizeTimeout();
                ptrARPCache->numRetries = 0;
                memcpy ((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module that the ARP cache state has transitioned */
                NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* YES. Update the timeout? */
                ptrARPCache->timeout = ptrARPCache->timeout - gARPCacheMCB.cfg.pollDelay;

                /* Do we need to send out an ARP request? */
                if (ptrARPCache->timeout == 0)
                {
                    /* YES. Time to send out an ARP request */
                    NetfpProxy_arpSendRequest (ptrARPCache);

                    /* Reset the timeout */
                    ptrARPCache->timeout = gARPCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();
                }
            }
            break;
        }
        case NetfpProxy_ARPCacheState_ACTIVE:
        {
            /* Has the ARP Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * ACTIVE -> ACTIVE State
                 *********************************************************************************/
                ptrARPCache->state      = NetfpProxy_ARPCacheState_ACTIVE;
                ptrARPCache->timeout    = gARPCacheMCB.cfg.activeTimeout + NetfpProxy_arpRandomizeTimeout();
                ptrARPCache->numRetries = 0;

                /* Is there a difference in the MAC address? */
                if (memcmp((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6) == 0)
                    bIsMACAddressChanged = 0;
                else
                    bIsMACAddressChanged = 1;

                /* Copy over the MAC Address */
                memcpy ((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* We need to notify the NETFP Proxy Core module only if the MAC address has changed */
                if (bIsMACAddressChanged == 1)
                    NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* Update the timeout. */
                ptrARPCache->timeout = ptrARPCache->timeout - gARPCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrARPCache->timeout == 0)
                {
                    /*********************************************************************************
                     * ACTIVE -> STALE State
                     *********************************************************************************/
                    ptrARPCache->state      = NetfpProxy_ARPCacheState_STALE;
                    ptrARPCache->timeout    = gARPCacheMCB.cfg.staleRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();
                    ptrARPCache->numRetries = gARPCacheMCB.cfg.maxInactiveRetries;

                    /* Send out the ARP request: */
                    NetfpProxy_arpSendRequest (ptrARPCache);
                }
            }
            break;
        }
        case NetfpProxy_ARPCacheState_STALE:
        {
            /* Has the ARP Cache entry been resolved? */
            if (bIsValidMACAddress == 1)
            {
                /*********************************************************************************
                 * STALE -> ACTIVE State
                 *********************************************************************************/
                ptrARPCache->state      = NetfpProxy_ARPCacheState_ACTIVE;
                ptrARPCache->timeout    = gARPCacheMCB.cfg.activeTimeout + NetfpProxy_arpRandomizeTimeout();
                ptrARPCache->numRetries = 0;

                /* Is there a difference in the MAC address? */
                if (memcmp((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6) == 0)
                    bIsMACAddressChanged = 0;
                else
                    bIsMACAddressChanged = 1;

                /* Copy over the MAC Address */
                memcpy ((void *)&ptrARPCache->macAddress[0], (void *)ptrMacAddress, 6);

                /* Notify the NETFP Proxy if there is a MAC address change */
                if (bIsMACAddressChanged == 1)
                    NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
            }

            /* Did we time out in the state? */
            if (reason == NetfpProxy_UpdateReason_TIMEOUT)
            {
                /* Update the timeout. */
                ptrARPCache->timeout = ptrARPCache->timeout - gARPCacheMCB.cfg.pollDelay;

                /* Did we timeout? */
                if (ptrARPCache->timeout == 0)
                {
                    /* YES. Did we exceed all our retries also? */
                    if (ptrARPCache->numRetries == 0)
                    {
                        /*********************************************************************************
                         * STALE -> INACTIVE State
                         *********************************************************************************/
                        ptrARPCache->state      = NetfpProxy_ARPCacheState_INACTIVE;
                        ptrARPCache->timeout    = gARPCacheMCB.cfg.inactiveRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();
                        ptrARPCache->numRetries = 0;

                        /* Reset the MAC Address to all 0 because we are inactive */
                        memset ((void *)&ptrARPCache->macAddress[0], 0, 6);

                        /* We need to notify the NETFP Proxy Core module only if the MAC address has changed */
                        NetfpProxy_neighUpdate(&ptrARPCache->targetIP, ptrARPCache->ifName, &ptrARPCache->macAddress[0]);
                    }
                    else
                    {
                        /*********************************************************************************
                         * STALE -> STALE State
                         *********************************************************************************/
                        ptrARPCache->state   = NetfpProxy_ARPCacheState_STALE;
                        ptrARPCache->timeout = gARPCacheMCB.cfg.staleRetransmissionTimeout + NetfpProxy_arpRandomizeTimeout();

                        /* Decrement the number of retransmissions */
                        ptrARPCache->numRetries = ptrARPCache->numRetries - 1;

                        /* Send out an ARP request */
                        NetfpProxy_arpSendRequest (ptrARPCache);
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
 *      The function is used to display the contents of the ARP cache. This is invoked
 *      by the NM module for debugging.
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_arpDump (void)
{
    NetfpProxy_ARPCache*    ptrARPCache;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*********************************************\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ARP Table\n");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "*********************************************\n");

    /* Cycle through the ARP Cache List: */
    ptrARPCache = (NetfpProxy_ARPCache*)List_getHead ((List_Node**)&gARPCacheMCB.ptrARPCacheList);
    while (ptrARPCache != NULL)
    {
        /* Display the ARP Cache: */
        NetfpProxy_arpDisplayCacheEntry (ptrARPCache);

        /* Get the next entry */
        ptrARPCache = (NetfpProxy_ARPCache*)List_getNext ((List_Node*)ptrARPCache);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      ARP Timeout Handler which is invoked periodically to age out the ARP
 *      cached entries and to implement the ARP Entry state machine
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_arpTimeoutHandler(void)
{
    NetfpProxy_ARPCache*    ptrARPCache;
    uint8_t                 dummyMACAddress[6] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

    /* Cycle through the ARP Cache List: */
    ptrARPCache = (NetfpProxy_ARPCache*)List_getHead ((List_Node**)&gARPCacheMCB.ptrARPCacheList);
    while (ptrARPCache != NULL)
    {
        /* Update the ARP cache entry as per the state machine. */
        NetfpProxy_arpUpdateCacheEntry (ptrARPCache, NetfpProxy_UpdateReason_TIMEOUT, &dummyMACAddress[0]);

        /* Get the next entry */
        ptrARPCache = (NetfpProxy_ARPCache*)List_getNext ((List_Node*)ptrARPCache);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the ARP monitoring for the specific IP address
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
int32_t NetfpProxy_arpStopPing(char* ifName, Netfp_IPAddr* ptrTargetIP)
{
    NetfpProxy_ARPCache*    ptrARPCache;

    /* Find the matching ARP entry */
    ptrARPCache = NetfpProxy_arpFindCacheEntry (ptrTargetIP);
    if (ptrARPCache == NULL)
        return 0;

    /* Remove the entry from the ARP cache entry: */
    List_removeNode((List_Node**)&gARPCacheMCB.ptrARPCacheList, (List_Node*)ptrARPCache);

    /* Cleanup the ARP Cache entry: */
    free (ptrARPCache);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to lookup an ARP cache entry matching the parameters. If there is no
 *      ARP cache entry found; the function will initiate a lookup for the target IP address.
 *
 *  @param[in]  ifName
 *      Interface Name on which the ARP resolution is to be done
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
int32_t NetfpProxy_arpStartPing
(
    char*           ifName,
    Netfp_IPAddr*   ptrTargetIP,
    char*           ptrTargetMACAddress
)
{
    NetfpProxy_ARPCache*    ptrARPCache;

    /* Is there already an ARP cache entry matching the request? */
    ptrARPCache = NetfpProxy_arpFindCacheEntry ((Netfp_IPAddr*)ptrTargetIP);
    if (ptrARPCache != NULL)
    {
        /* YES. ARP Cache entry already exists which implies that the Target IP address is
         * already in the cache and so we dont need to initiate another lookup. Copy the
         * target MAC Address from the ARP Cache entry */
        memcpy ((void *)ptrTargetMACAddress, (void *)&ptrARPCache->macAddress[0], 6);
        return 0;
    }

    /* Allocate a new cache entry: */
    ptrARPCache = (NetfpProxy_ARPCache*)malloc(sizeof(NetfpProxy_ARPCache));
    if (ptrARPCache == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to allocate ARP cache memory\n");
        return -1;
    }

    /* Initialize the allocated memory */
    memset ((void*)ptrARPCache, 0, sizeof(NetfpProxy_ARPCache));

    /* Get the MAC Address associated with the interface name */
    if (NetfpProxy_neighGetMACAddress(ifName, &ptrARPCache->senderMACAddress[0]) < 0)
        return -1;

    /* Get the IP Address associated with the interface name */
    if (NetfpProxy_neighGetIPAddress(ifName, &ptrARPCache->senderIP) < 0)
        return -1;

    /* Populate the remaining fields in the ARP Cache: */
    ptrARPCache->state         = NetfpProxy_ARPCacheState_INITIAL;
    ptrARPCache->numRetries    = gARPCacheMCB.cfg.initialRetries;
    ptrARPCache->timeout       = gARPCacheMCB.cfg.initialRetransmissionTimeout;
    strncpy (ptrARPCache->ifName, ifName, NETFP_MAX_CHAR);
    memcpy ((void *)&ptrARPCache->targetIP, (void *)ptrTargetIP, sizeof(Netfp_IPAddr));

    /* Add the ARP cache entry to the list */
    List_addNode ((List_Node**)&gARPCacheMCB.ptrARPCacheList, (List_Node*)ptrARPCache);

    /* The MAC address is not known at this point in time. We will send out the ARP request
     * and will notify the neighbor module on an update. But for now this IP address is not
     * reachable. */
    memset ((void *)ptrTargetMACAddress, 0, 6);

    /* Send out the ARP Request to start the resolution: */
    NetfpProxy_arpSendRequest (ptrARPCache);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process packets received from the network.
 *
 *  @retval
 *      Not Applicable.
 */
void NetfpProxy_arpProcessPacket (void)
{
    uint8_t                 packet[1500];
    uint16_t                protocol;
    NetfpProxy_ARPHeader*   ptrARPHeader;
    NetfpProxy_ARPCache*    ptrARPCache;
    uint8_t                 isBroadcast;
    Netfp_IPAddr            senderIPAddress;

    /* Increment the statistics: */
    gARPCacheMCB.numPacketsReceived++;

    /* Read the message from the ARP socket: */
    if (read (gARPCacheMCB.sock, &packet, sizeof(packet)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read packet from the ARP socket [Error: %s]\n", strerror(errno));
        return;
    }

    /* Is this a broadcast packet? */
    if ((packet[0] == 0xFF) && (packet[1] == 0xFF) && (packet[2] == 0xFF) &&
        (packet[3] == 0xFF) && (packet[4] == 0xFF) && (packet[5] == 0xFF))
        isBroadcast = 1;
    else
        isBroadcast = 0;

    /* Read the protocol type in the Ethernet header: */
    protocol = packet[12] << 8 | packet[13];

    /* Is this a VLAN Enabled packet? */
    if (protocol == ETH_P_8021Q)
    {
        /* Get the protocol type from the VLAN header: */
        protocol = packet[16] << 8 | packet[17];

        /* The ARP header starts after the VLAN header */
        ptrARPHeader = (NetfpProxy_ARPHeader*)&packet[18];
    }
    else
    {
        /* The ARP header starts immediately after the Ethernet header: */
        ptrARPHeader = (NetfpProxy_ARPHeader*)&packet[14];
    }

    /* Validations: We are interested in only ARP packets */
    if (protocol != ETH_P_ARP)
        return;

    /* Validations: We support only the Ethernet hardware type */
    if (ptrARPHeader->hardwareType != htons(ARPHRD_ETHER))
        return;

    /* Validations: We support only the IPv4 Protocol */
    if (ptrARPHeader->protocolType != htons(ETH_P_IP))
        return;

    /* Validations: The hardware & protocol size should be for Ethernet and IPv4 */
    if ((ptrARPHeader->hardwareAddrLen != 6) || (ptrARPHeader->protocolAddrLen != 4))
        return;

    /* Validations: Only ARP Request and Reply are handled here. */
    if ((ptrARPHeader->arpOpType != htons(ARPOP_REQUEST)) && (ptrARPHeader->arpOpType != htons(ARPOP_REPLY)))
        return;

    /* Validations: We are not accepting Broadcast/Multicast MAC address in the source or target hardware address */
    if ((ptrARPHeader->senderHwAddress[0] & 0x1) || (ptrARPHeader->targetHwAddress[0] & 0x1))
        return;

    /* Control comes here implies that the packet can be processed: */
    if (ptrARPHeader->arpOpType == htons(ARPOP_REQUEST))
    {
        /* ARP Request Packet: Is this a Gratuitous ARP packet? */
        if ((ptrARPHeader->senderIPAddress[0] == ptrARPHeader->targetIPAddress[0]) &&
            (ptrARPHeader->senderIPAddress[1] == ptrARPHeader->targetIPAddress[1]) &&
            (ptrARPHeader->senderIPAddress[2] == ptrARPHeader->targetIPAddress[2]) &&
            (ptrARPHeader->senderIPAddress[3] == ptrARPHeader->targetIPAddress[3]) &&
            (isBroadcast == 1))
        {
            /* YES. Gratuitous ARP Request packets are always processed */
            gARPCacheMCB.numPacketsProcessed++;

            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Gratuitous ARP Request for %03d.%03d.%03d.%03d\n",
                               ptrARPHeader->senderIPAddress[0], ptrARPHeader->senderIPAddress[1],
                               ptrARPHeader->senderIPAddress[2], ptrARPHeader->senderIPAddress[3]);

            /* Convert the sender IP address to correct format. */
            senderIPAddress.ver               = Netfp_IPVersion_IPV4;
            senderIPAddress.addr.ipv4.u.a8[0] = ptrARPHeader->senderIPAddress[0];
            senderIPAddress.addr.ipv4.u.a8[1] = ptrARPHeader->senderIPAddress[1];
            senderIPAddress.addr.ipv4.u.a8[2] = ptrARPHeader->senderIPAddress[2];
            senderIPAddress.addr.ipv4.u.a8[3] = ptrARPHeader->senderIPAddress[3];

            /* Do we have the sender IP address in our ARP cache? */
            ptrARPCache = NetfpProxy_arpFindCacheEntry ((Netfp_IPAddr*)&senderIPAddress);
            if (ptrARPCache != NULL)
            {
                /* YES. We need to update the ARP Cache entry; since we have received a Gratuitous ARP for
                 * an IP address we are monitoring. */
                NetfpProxy_arpUpdateCacheEntry (ptrARPCache, NetfpProxy_UpdateReason_PACKET, &ptrARPHeader->senderHwAddress[0]);
                return;
            }

            /* Control comes here implies that we received a Gratuitous ARP packet but we were not monitoring
             * the IP address. So we simply drop the packet here; but this packet was good enough to be processed */
            return;
        }

        /* Control comes here implies that we received a non Gratuitous ARP request packet. We are not interested
         * in this. */
        return;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: ARP Reply for %03d.%03d.%03d.%03d 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                       ptrARPHeader->senderIPAddress[0], ptrARPHeader->senderIPAddress[1],
                       ptrARPHeader->senderIPAddress[2], ptrARPHeader->senderIPAddress[3],
                       ptrARPHeader->senderHwAddress[0], ptrARPHeader->senderHwAddress[1],
                       ptrARPHeader->senderHwAddress[2], ptrARPHeader->senderHwAddress[3],
                       ptrARPHeader->senderHwAddress[4], ptrARPHeader->senderHwAddress[5]);

    /* Convert the sender IP address to correct format. */
    senderIPAddress.ver               = Netfp_IPVersion_IPV4;
    senderIPAddress.addr.ipv4.u.a8[0] = ptrARPHeader->senderIPAddress[0];
    senderIPAddress.addr.ipv4.u.a8[1] = ptrARPHeader->senderIPAddress[1];
    senderIPAddress.addr.ipv4.u.a8[2] = ptrARPHeader->senderIPAddress[2];
    senderIPAddress.addr.ipv4.u.a8[3] = ptrARPHeader->senderIPAddress[3];

    /* ARP Reply Packet: Is the IP address something we are interested in? It is possible to receive an ARP reply
     * for an IP address we are not interested in. In that case we simply drop the packet. */
    ptrARPCache = NetfpProxy_arpFindCacheEntry ((Netfp_IPAddr*)&senderIPAddress);
    if (ptrARPCache == NULL)
        return;

    /* Update the cache entry: */
    NetfpProxy_arpUpdateCacheEntry (ptrARPCache, NetfpProxy_UpdateReason_PACKET, &ptrARPHeader->senderHwAddress[0]);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the ARP module.
 *
 *  @param[in]  pollDelay
 *      Polling Granularity Delay in seconds
 *
 *  @retval
 *      Success     -   Pointer to the ARP raw socket
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_arpInit (uint32_t pollDelay)
{
    int32_t             enableBroadcast = 1;
    NetfpProxy_ARPCfg   arpCfg;

    /* Initialize the ARP Cache configuration: */
    memset ((void *)&arpCfg, 0, sizeof(NetfpProxy_ARPCfg));

    /* Populate the ARP Cache configuration:
     * - Active Timeout is 60 seconds: Once an entry is marked as ACTIVE it will remain
     *   active till we move it into the STALE stage.
     * - Initial Retransmission timeout is 3 seconds: We want to perform the initial
     *   ARP resolution as fast as possible.
     * - Stale Retransmission timeout is 60 seconds: The ARP entry can still be used and
     *   we dont need to agressively send out ARP requests to move it back to ACTIVE.
     *
     * NOTE: The TEST values are for internal testing and can be removed later on. */
#ifdef ARP_TEST
    arpCfg.pollDelay                     = pollDelay;
    arpCfg.maxInactiveRetries            = 5;
    arpCfg.activeTimeout                 = 3;
    arpCfg.initialRetries                = 3;
    arpCfg.initialRetransmissionTimeout  = 3;
    arpCfg.staleRetransmissionTimeout    = 3;
    arpCfg.inactiveRetransmissionTimeout = 5;
#else
    arpCfg.pollDelay                     = pollDelay;
    arpCfg.maxInactiveRetries            = 5;
    arpCfg.activeTimeout                 = 60;
    arpCfg.initialRetries                = 3;
    arpCfg.initialRetransmissionTimeout  = 3;
    arpCfg.staleRetransmissionTimeout    = 60;
    arpCfg.inactiveRetransmissionTimeout = 5;
#endif

    /* Initialize the ARP Cache module */
    memset ((void *)&gARPCacheMCB, 0, sizeof(NetfpProxy_ARPCacheMCB));

    /* Copy over the ARP confiuguration: */
    memcpy ((void *)&gARPCacheMCB.cfg, (void*)&arpCfg, sizeof(NetfpProxy_ARPCfg));

    /* Open the ARP socket: */
    gARPCacheMCB.sock = socket(PF_PACKET, SOCK_PACKET, htons(ETH_P_ARP));
	if (gARPCacheMCB.sock < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to open the ARP RAW Socket [Error: %s]\n", strerror(errno));
        return -1;
    }

    /* Enable the BROADCAST Option for the socket. */
    if (setsockopt(gARPCacheMCB.sock, SOL_SOCKET, SO_BROADCAST, &enableBroadcast, sizeof(int32_t)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "Error: Unable to enable the broadcast for the ARP Socket [Error: %s]\n", strerror(errno));
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: ARP socket [%d] has been created successfully\n", gARPCacheMCB.sock);

    /* Return the ARP RAW Socket: */
    return gARPCacheMCB.sock;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize & shutdown the ARP module.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t NetfpProxy_arpDeinit (void)
{
    NetfpProxy_ARPCache*    ptrARPCache;

    /* Cleanup the ARP Cache entries */
    ptrARPCache = (NetfpProxy_ARPCache*)List_getHead ((List_Node**)&gARPCacheMCB.ptrARPCacheList);
    while (ptrARPCache != NULL)
    {
        /* Cleanup the memory for the ARP Cache entry */
        free (ptrARPCache);

        /* Get the new head. */
        ptrARPCache = (NetfpProxy_ARPCache*)List_getHead ((List_Node**)&gARPCacheMCB.ptrARPCacheList);
    }

    /* Close the ARP RAW Socket */
    close (gARPCacheMCB.sock);
    return 0;
}

