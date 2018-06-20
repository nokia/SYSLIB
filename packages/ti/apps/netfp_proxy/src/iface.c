/**
 *   @file  iface.c
 *
 *   @brief
 *      NETFP Proxy Interface Implementation
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
#include <dlfcn.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/if_bridge.h>

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**********************************************************************
 ************************ Local Structures ****************************
 **********************************************************************/

/**
 * @brief
 *  NETFP Bridge Interface
 *
 * @details
 *  The structure holds information about interfaces below bridges
 */
typedef struct NetfpProxy_BridgeIface
{
    /**
     * @brief   Name of the interface
     */
    char    ifName[NETFP_MAX_CHAR];
}NetfpProxy_BridgeIface;

/**
 * @brief
 *  NETFP Proxy Interface configuration
 *
 * @details
 *  The NETFP Proxy interface configuration is the configuration which is required
 *  to create an interface within the NETFP Proxy.
 */
typedef struct NetfpProxy_InterfaceCfg
{
    /**
     * @brief   NETFP Interface configuration:
     */
    Netfp_InterfaceCfg      ifConfig;

    /**
     * @brief   List of IP address which are configured on the interface
     */
    Netfp_IPAddr            ipAddress[NETFP_PROXY_MAX_IP];

    /**
     * @brief   List of subnet masks which are configured on the interface
     */
    Netfp_IPAddr            subnetMask[NETFP_PROXY_MAX_IP];

    /**
     * @brief   Flag which indicates if the interface needs to be monitored by the NETMGR
     */
    uint32_t                monitorStatus;

    /**
     * @brief   Number of IP addresses which are configured
     */
    uint32_t                numIP;

    /**
     * @brief   For bridged interfaces: This is a list of all the interface names which reside
     * below the bridge. This allows us to link down to the child interfaces
     */
    NetfpProxy_BridgeIface  bridgePortList[NETFP_PROXY_MAX_BRIDGE_IF];

    /**
     * @brief   Number of bridged ports
     */
    int32_t                 numBridgedPorts;

    /**
     * @brief   Operational status of the link
     */
    uint32_t                linkStatus;
}NetfpProxy_InterfaceCfg;

/**
 * @brief
 *  NETFP Proxy Interface
 *
 * @details
 *  The NETFP Proxy interface block holds information about interface which has been
 *  offloaded and is being monitored by the NETFP Proxy.
 */
typedef struct NetfpProxy_Interface
{
    /**
     * @brief   Link to other NETFP Proxy interfaces
     */
    List_Node                       links;

    /**
     * @brief   Interface name
     */
    char                            ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Interface configuration
     */
    NetfpProxy_InterfaceCfg         proxyIfCfg;

    /**
     * @brief   NETFP interface handle
     */
    Netfp_IfHandle                  ifHandle;

    /**
     * @brief   Link status
     */
    uint32_t                        bIsLinkUp;

    /**
     * @brief   L3 QoS Shaping configuration
     */
    Netfp_L3QoSCfg                  l3QoSCfg;

    /**
     * @brief   Link info for this interface
     */
    struct rtnl_link*               link;

    /**
     * @brief   NETMGR Link event subscriber handle
     */
    netmgr_subsc_hdl                linkEvtSubscHdl;

    /**
     * @brief   NETMGR Address event subscriber handle
     */
    netmgr_subsc_hdl                addrEvtSubscHdl;

    /**
     * @brief   Pointer to the bridged interface. This is valid only for bridged port interfaces
     */
    struct NetfpProxy_Interface*    ptrBridgedInterface;
}NetfpProxy_Interface;

/**
 * @brief
 *  Route Management MCB
 *
 * @details
 *  The structures is the Route Management MCB.
 */
typedef struct NetfpProxy_IfMgmtMCB
{
    /**
     * @brief   NETMGR Route module
     */
    NetfpProxy_NetMgrHandle     netMgrIfHandle;

    /**
     * @brief   NETFP Proxy interface configuration information
     */
    NetfpProxy_Interface*       ifaceList;
}NetfpProxy_IfMgmtMCB;

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* Interface MCB: */
NetfpProxy_IfMgmtMCB        gIfMgmtMCB;

/**********************************************************************
 *********************** Interface Functions **************************
 **********************************************************************/

static int32_t NetfpProxy_ifaceInstantiate (const char*, NetfpProxy_InterfaceCfg*, int32_t*);
static int32_t NetfpProxy_ifaceDestroy (NetfpProxy_Interface*);

/**
 *  @b Description
 *  @n
 *      Utility API which constructs the *real* bridged interface name given
 *      the bridge name and interface name. These names are used to create
 *      interfaces in the NETFP Server.
 *
 *      Name will have the following format:
 *          <bridgename>-><ifName>
 *      i.e.
 *          br0->eth1
 *
 *  @param[in]  bridgeName
 *      Name of the bridge
 *  @param[in]  ifName
 *      Name of the interface under the bridge as present in Linux
 *  @param[out] bridgedPortName
 *      Bridged Port Name constructed
 *
 *  @retval
 *      Not Applicable.
 */
static inline void NetfpProxy_constructBridgedPortName (const char* bridgeName, const char* ifName, char* bridgedPortName)
{
    snprintf (bridgedPortName, NETFP_MAX_CHAR, "%s->%s", bridgeName, ifName);
}

/**
 *  @b Description
 *  @n
 *      Utility API to convert a given IPv4 prefix to subnet mask
 *
 *  @param[in]  prefixLen
 *      IPv4 prefix length
 *  @param[out]  mask
 *      Resulting Subnet mask
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpProxy_convertV4Prefix2Mask (uint32_t prefixLen, uint32_t* mask)
{
    /* Reset the mask to all zeros. As per prefix
     * length, we will set the appropriate bits
     * to one. Rest will be left to all zeros. */
    memset ((void *)mask, 0, sizeof (uint32_t));

    /* Calculate number of bits in a 32 bit word with ones
     * as per prefix length */
    *mask   =   ~((1 << (32 - prefixLen)) - 1);

    /* Return mask in network order */
    *mask   =   htonl (*mask);

    /* Conversion done */
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility API to convert a given IPv6 prefix to subnet mask
 *
 *  @param[in]  prefixLen
 *      IPv6 prefix length
 *  @param[out]  mask
 *      Resulting Subnet mask
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpProxy_convertV6Prefix2Mask (uint32_t prefixLen, uint32_t mask[4])
{
    uint32_t        n, calcMask, i = 0;

    /* Reset the mask to all zeros. As per prefix
     * length, we will set the appropriate bits
     * to one. Rest will be left to all zeros. */
    memset ((void *)mask, 0, sizeof (uint32_t) * 4);

    /* Calculate number of words with ones as per prefix length */
    n   =   prefixLen / 32;
    for (i = 0; i < n; i++)
    {
        calcMask    =   ~((1 << (32 - 32)) - 1);
        mask[i]     =   htonl (calcMask);
    }
    /* Calculate number of bits left with ones as per prefix length */
    n   =   prefixLen % 32;
    if (n)
    {
        calcMask    =   ~((1 << (32 - n)) - 1);
        mask[i]     =   htonl(calcMask);
    }

    /* Conversion done */
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility API to determine if the interface is a bridged interface or not?
 *
 *  @param[in]  ifName
 *      Name of the interface
 *
 *  @retval
 *      1   -   Bridge Interface
 *  @retval
 *      0   -   Not a bridge Interface
 */
static uint32_t NetfpProxy_ifaceIsBridge (const char* ifName)
{
    if (strncmp (ifName, "br", 2) == 0)
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility API to determine if the interface is a VLAN interface or not.
 *
 *  @param[in]  ifName
 *      Pointer to the name of the interface
 *  @param[out] vlanId
 *      VLAN Identifier populated if the interface was VLAN capable. This is
 *      an optional parameter and can be set to NULL.
 *  @param[out] ptrVLANPriorityMap
 *      VLAN Priority map populated if the interface was VLAN capable. This is
 *      an optional parameter and can be set to NULL
 *
 *  @retval
 *      1   -   VLAN Interface
 *  @retval
 *      0   -   Not a VLAN Interface
 */
static uint32_t NetfpProxy_ifaceIsVLAN
(
    const char*             ifName,
    uint32_t*               vlanId,
    Netfp_VLANPriorityMap*  ptrVLANPriorityMap
)
{
    char        fileName[PATH_MAX];
    FILE*       fp;
    char*       line = NULL;
    size_t      len;
    const char* vlanIDDelimitter    = ". ";
    const char* egressMapDelimitter = ": ";
    char*       vidToken;
    char*       sockPriorityToken;
    char*       vlanPrioToken;
    uint8_t     sockPriority;
    uint8_t     vlanPriority;

    /* Open the file: */
    snprintf (fileName, PATH_MAX, "/proc/net/vlan/%s", ifName);
    fp = fopen (fileName, "r");
    if (fp == NULL)
        return 0;

    /* Do we need to populate the output arguments? */
    if ((vlanId == NULL) || (ptrVLANPriorityMap == NULL))
    {
        /* NO: We know that this was a VLAN Interface since the file existed. */
        fclose (fp);
        return 1;
    }

    /* YES: Initialize the VLAN Identifier and Egress Priority map: */
    *vlanId = 0;
    memset ((void *)ptrVLANPriorityMap, 0, sizeof(Netfp_VLANPriorityMap));

    /* This is a VLAN interface: We need to parse the file to get the VLAN Identifier & Egress Priority Map */
    while (1)
    {
        /* Read the line from the file: */
        if (getline (&line, &len, fp) < 0)
            break;

        /* Get the VLAN identifier */
        if (strncmp (line, ifName, strlen(ifName)) == 0)
        {
            /* Found it: Tokenize to get the VLAN identifier */
            vidToken = strtok (line, vlanIDDelimitter);
            vidToken = strtok (NULL, vlanIDDelimitter);
            *vlanId = atoi (vidToken);
        }

        /* Does the line have the egress map? */
        if (strstr (line, "EGRESS priority mappings:" ) != NULL)
        {
            /* Found it: Skip the header */
            vidToken = strtok (line, egressMapDelimitter);
            vidToken = strtok (NULL, egressMapDelimitter);
            vidToken = strtok (NULL, egressMapDelimitter);

            while (1)
            {
                sockPriorityToken = strtok (NULL, egressMapDelimitter);
                vlanPrioToken     = strtok (NULL, egressMapDelimitter);

                /* Did we reach the end of the line? */
                if ((sockPriorityToken == NULL) || (vlanPrioToken == NULL))
                    break;

                /* Get the DSCP & VLAN Priorities from the egress map */
                sockPriority = atoi (sockPriorityToken);
                vlanPriority = atoi (vlanPrioToken);

                /* Sanity Check: */
                if ((sockPriority >= NETFP_MAX_SOCK_PRIORITY) || (vlanPriority > 8))
                {
                    /* Error: Out of range; report the error and continue */
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' skipping VLAN Egress Mapping for %d -> %d\n",
                                       ifName, sockPriority, vlanPriority);
                }
                else
                {
                    /* Store the mappings: */
                    ptrVLANPriorityMap->socketToVLANPriority[sockPriority] = vlanPriority;
                }
            }
        }

        /* Cleanup the memory allocated by reading from the file. */
        free (line);
        line = NULL;
    }
    /* Close the file */
    fclose (fp);
    return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the list of interfaces below a specific bridge
 *
 *  @param[in]  bridgeIfName
 *      Pointer to the Bridge
 *  @param[in]  ifList
 *      Pointer to list of interface names below the bridge
 *
 *  @retval
 *      Error   - <0
 *  @retval
 *      Success - Number of interfaces
 */
static int32_t NetfpProxy_ifaceGetBridgedInterfaceList(const char* bridgeIfName, NetfpProxy_BridgeIface* ifList)
{
    char            directoryName[PATH_MAX];
    DIR*            bridgeDirectory;
    struct dirent*  ptrIfaceDirEntry;
    uint32_t        count = 0;

    /* Find the interface name corresponding the port number */
    snprintf (directoryName, PATH_MAX, "/sys/class/net/%s/brif/", bridgeIfName);

    /* Open the directory: */
    bridgeDirectory = opendir (directoryName);
    if (bridgeDirectory == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Cant open bridge directory for %s\n", bridgeIfName);
        return -1;
    }

    /* Cycle through all the directories below the bridge */
    while (1)
    {
        /* Read the interface directory entry. */
        ptrIfaceDirEntry = readdir (bridgeDirectory);
        if (ptrIfaceDirEntry == NULL)
            break;

        /* Skip the current directory */
        if (ptrIfaceDirEntry->d_name[0] == '.')
            continue;

        /* Copy the interface name */
        strncpy (ifList[count].ifName, ptrIfaceDirEntry->d_name, NETFP_MAX_CHAR);

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridge '%s' detected interface '%s' @index %d\n",
                           bridgeIfName, ifList[count].ifName, count);

        /* Increment the count. */
        count++;
    }

    /* Close the directory */
    closedir (bridgeDirectory);
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a list of all the IP addresses which are
 *      added to the specific interface on Linux
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[out] ipAddressList
 *      IP Address List populated by the function
 *  @param[out] subnetMaskList
 *      Subnet Address List populated by the function
 *
 *  @retval
 *      Success - Number of IP addresses
 *  @retval
 *      Error   - <0
 */
static int32_t NetfpProxy_ifaceGetIP
(
    const char*     ifName,
    Netfp_IPAddr*   ipAddressList,
    Netfp_IPAddr*   subnetMaskList
)
{
    int32_t             errCode;
    uint32_t            index;
    uint32_t*           ipAddrList;
    uint32_t            numEntries = 0;
    struct rtnl_addr*   nlIPAddrInfo;
    struct nl_addr*     nlIPAddr;

    /* Get all the IPv4, IPv6 addresses populate them to the NETFP library. It is possible that an
     * interface does not have an IP address. */
    errCode = netmgr_addr_get (gIfMgmtMCB.netMgrIfHandle, ifName, &ipAddrList, &numEntries);
    if (errCode < 0 && errCode != NETFP_PROXY_RETVAL_E_OP_FAILED)
    {
        /* Error: error to get IP addresses. Returns NETFP_PROXY_RETVAL_E_OP_FAILED if no assigned IP(s) */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' addr_get failure [errCode: %d]\n",
                           ifName, errCode);
        return -1;
    }

    /* Sanity Check: Do not exceed the max permissible entries? */
    if (numEntries >= NETFP_PROXY_MAX_IP)
    {
        /* Error: We are exceeding the MAX supported entries */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' has %d IP addresses only %d are supported\n",
                           ifName, numEntries, NETFP_PROXY_MAX_IP);
        return -1;
    }

    /* Cycle through and store all the IPv4/IPv6 addresses/subnet mask */
    for (index = 0; index < numEntries; index++)
    {
        /* Convert the IP address into the NETFP format */
        nlIPAddrInfo =  (struct rtnl_addr*)ipAddrList[index];
        nlIPAddr     =  rtnl_addr_get_local (nlIPAddrInfo);

        if (nl_addr_iszero(nlIPAddr) == 1) {
            // Not a valid IP address
            ipAddressList[index].ver  = Netfp_IPVersion_INVALID;
            subnetMaskList[index].ver = Netfp_IPVersion_INVALID;
        } else {

            /* Is this an IPv4 address? */
            if (nl_addr_get_family (nlIPAddr) == AF_INET)
            {
                /* IPv4 Address: */
                ipAddressList[index].ver = Netfp_IPVersion_IPV4;
                memcpy ((void *)&ipAddressList[index].addr.ipv4.u.a8[0], (void *)nl_addr_get_binary_addr (nlIPAddr), 4);

                /* Populate the subnet mask: */
                subnetMaskList[index].ver = Netfp_IPVersion_IPV4;
                NetfpProxy_convertV4Prefix2Mask (nl_addr_get_prefixlen (nlIPAddr), &subnetMaskList[index].addr.ipv4.u.a32);

                /* Debug Message: Display the IP Address */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: '%s' [%d] %03d.%03d.%03d.%03d %03d.%03d.%03d.%03d\n",
                                   ifName, index,
                                   ipAddressList[index].addr.ipv4.u.a8[0],  ipAddressList[index].addr.ipv4.u.a8[1],
                                   ipAddressList[index].addr.ipv4.u.a8[2],  ipAddressList[index].addr.ipv4.u.a8[3],
                                   subnetMaskList[index].addr.ipv4.u.a8[0], subnetMaskList[index].addr.ipv4.u.a8[1],
                                   subnetMaskList[index].addr.ipv4.u.a8[2], subnetMaskList[index].addr.ipv4.u.a8[3]);
            }
            else
            {
                char strIPAddress[256];

                /* IPv6 Address: */
                ipAddressList[index].ver = Netfp_IPVersion_IPV6;
                memcpy ((void *)&ipAddressList[index].addr.ipv6.u.a8[0], (void *)nl_addr_get_binary_addr (nlIPAddr), 16);

                /* Populate the subnet mask: */
                subnetMaskList[index].ver = Netfp_IPVersion_IPV6;
                NetfpProxy_convertV6Prefix2Mask (nl_addr_get_prefixlen (nlIPAddr), &subnetMaskList[index].addr.ipv6.u.a32[0]);

                /* Convert the IPv6 address to string: */
                Netfp_convertIP6ToStr (ipAddressList[index].addr.ipv6, &strIPAddress[0]);

                /* Debug Message: */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: '%s' [%d] %s\n", ifName, index, strIPAddress);
            }
        }
    }

    /* Cleanup only if there are IP Addresses: */
    if (numEntries != 0)
    {
        /* Done processing IP addresses. Free its memory */
        for (index = 0; index < numEntries; index++)
            rtnl_addr_put ((struct rtnl_addr*)ipAddrList[index]);
        free(ipAddrList);
    }
    return numEntries;
}

/**
 *  @b Description
 *  @n
 *      The function cycles through all the IP address assigned to an interface
 *      and searches for a match.
 *
 *  @param[in]  intfInfo
 *      Interface info
 *  @param[in]  nlIPAddr
 *      IP address to look for
 *
 *  @retval
 *      >=0 -   Matching index
 *  @retval
 *      <0  -   No Match
 */
static int32_t NetfpProxy_ifaceFindIP (NetfpProxy_Interface* ptrNetfpProxyInterface, struct nl_addr* nlIPAddr)
{
    uint32_t    index;

    /* Cycle through all the IP addresses which are assigned to the interface */
    for (index = 0; index < NETFP_PROXY_MAX_IP; index++)
    {
        /* Search the same socket family: */
        if ((nl_addr_get_family (nlIPAddr) == AF_INET) &&
            (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver == Netfp_IPVersion_IPV4))
        {
            /* IPv4 Address: Do we have a match? */
            if (memcmp ((void *)&ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[0],
                        (void *)nl_addr_get_binary_addr (nlIPAddr), 4) == 0)
            {
                /* YES: Perfect return the matched index */
                return index;
            }
        }
        if ((nl_addr_get_family (nlIPAddr) == AF_INET6) &&
            (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver == Netfp_IPVersion_IPV6))
        {
            /* IPv6 Address: Do we have a match? */
            if (memcmp ((void *)&ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv6.u.a8[0],
                        (void *)nl_addr_get_binary_addr (nlIPAddr), 16) == 0)
            {
                /* YES: Perfect return the matched index */
                return index;
            }
        }
    }

    /* Control comes here; implies that there was no match */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find if an interface has already been created
 *      and offloaded by the NETFP Proxy.
 *
 *  @param[in]  ifName
 *      Interface name which is being searched for
 *
 *  @retval
 *      Not NULL - Matching interface block
 *  @retval
 *      NULL     - No matching interface block
 */
static NetfpProxy_Interface* NetfpProxy_ifaceFindInterface (const char* ifName)
{
    NetfpProxy_Interface*     ptrNetfpProxyInterface;

    /* Cycle through all the registered interfaces */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getHead ((List_Node**)&gIfMgmtMCB.ifaceList);
    while (ptrNetfpProxyInterface != NULL)
    {
        /* Do we have a match? */
        if (strncmp (ptrNetfpProxyInterface->ifName, ifName, NETFP_MAX_CHAR) == 0)
            return ptrNetfpProxyInterface;

        /* Get the next interface */
        ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getNext ((List_Node*)ptrNetfpProxyInterface);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine if the interface name specified lies under a bridge
 *      or not.
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[out] bridgeName
 *      Name of the bridge under which the interface is present.
 *
 *  @retval
 *      1   -   Interface resides under a bridge
 *  @retval
 *      0   -   Interface does not reside under a bridge
 */
static int32_t NetfpProxy_ifaceIsBridged (const char* ifName, char* bridgeName)
{
    NetfpProxy_Interface*     ptrNetfpProxyInterface;
    char                      bridgedPortName[NETFP_MAX_CHAR];

    /* Cycle through all the registered interfaces */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getHead ((List_Node**)&gIfMgmtMCB.ifaceList);
    while (ptrNetfpProxyInterface != NULL)
    {
        /* Is the interface bridged? */
        if (NetfpProxy_ifaceIsBridge (&ptrNetfpProxyInterface->ifName[0]) == 1)
        {
            /* YES: Construct the bridged port name */
            NetfpProxy_constructBridgedPortName (&ptrNetfpProxyInterface->ifName[0], ifName, &bridgedPortName[0]);

            /* Is there a match? */
            if (NetfpProxy_ifaceFindInterface (bridgedPortName) != NULL)
            {
                /* YES: We have found a match. Populate the arguments */
                strncpy (bridgeName, ptrNetfpProxyInterface->ifName, NETFP_MAX_CHAR);
                return 1;
            }
            else
            {
                /* NO: The interface does not reside under this bridge. Fall through and continue to the next interface */
            }
        }

        /* Get the next interface */
        ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getNext ((List_Node*)ptrNetfpProxyInterface);
    }

    /* Control comes here implies that the interface is not under any bridge. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to determine the bridged port interface on which the MAC address
 *      resides. This is done by matching the MAC address to the Bridge forwarding database
 *
 *  @param[in]  ptrBridgeInterface
 *      Pointer to the bridge interface
 *  @param[in]  ptrMACAddress
 *      Pointer MAC address
 *
 *  @retval
 *      Success -   Pointer to the bridged port interface
 *  @retval
 *      Error   -   NULL
 */
static NetfpProxy_Interface* NetfpProxy_ifaceGetBridgedPort
(
    NetfpProxy_Interface*   ptrBridgeInterface,
    uint8_t*                ptrMACAddress
)
{
    struct ifreq            ifr;
    struct __fdb_entry      fe;
    uint32_t                offset = 0;
    int                     bridgeSocket;
    unsigned long           args[4];
    int32_t                 portNumber = -1;
    DIR*                    ptrBridgeDirectory;
    struct dirent*          ptrIfaceDirEntry;
    char                    bridgeDirectory[PATH_MAX];
    FILE*                   ptrFile;
    uint32_t                intfPortNum;
    int32_t                 numEntries;
    NetfpProxy_Interface*   ptrBridgedPortInterface;
    char                    bridgeName[IFNAMSIZ];
    char                    bridgedPortName[NETFP_MAX_CHAR];
    int                     loopCounter = 5; //fzm

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: Bridged port search for MAC %02x:%02x:%02x:%02x:%02x:%02x\n", *ptrMACAddress,
                       *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3), *(ptrMACAddress + 4), *(ptrMACAddress + 5)); //fzm

    /****************************************************************************************
     * There are 2 cases in which a bridge can be configured:
     *  (a) br0 is added on top of eth0.x and eth1.x
     *  (b) br0.x is added on top of eth0 and eth1
     ****************************************************************************************/
    if (ptrBridgeInterface->proxyIfCfg.ifConfig.type == Netfp_InterfaceType_ETH)
    {
        /* Case (a): No translations are required */
        strncpy(bridgeName, ptrBridgeInterface->ifName, IFNAMSIZ);
    }
    else
    {
        /* Case (b): We need to derive the actual bridge name i.e. translate br0.x to br0 and then find everything below it. */
        strncpy(bridgeName, ptrBridgeInterface->ifName, IFNAMSIZ);
        strtok (bridgeName, ".");
    }

    /* Open a bridge socket: */
    bridgeSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (bridgeSocket < 0)
        return NULL;
//fzm - Fix for a infinite loop in case MAC is not in bridge list
    while( (portNumber == -1) && (--loopCounter >= 0) )
    {
        offset = 0;
        while (1)
        {

            /* Populate the arguments: */
            args[0] = BRCTL_GET_FDB_ENTRIES;
            args[1] = (unsigned long)&fe;
            args[2] = 1;
            args[3] = offset;

            /* Populate the arguments: */
            ifr.ifr_data = (char *)args;
            strncpy(ifr.ifr_name, bridgeName, IFNAMSIZ);

            /* Get the FDB entry from the bridge */
        numEntries = ioctl(bridgeSocket, SIOCDEVPRIVATE, &ifr);
        if (numEntries < 0)
            {
                /* Error: Unable to get the IOCTL; check the error code */
                if (errno == EAGAIN)
                    continue;

                /* IOCTL Failure: We could have reached the end of the FDB and still not found anything. */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to read FDB from the bridge [Error:%m]\n");
                break;
            }
            if (numEntries == 0)
            {
                /* Error: Bridge IOCTL did not return an FDB entry. */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Exhausted the FDB entries. No match found\n");
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Could not find the MAC %02x:%02x:%02x:%02x:%02x:%02x, ioctl returned 0, offset: %d\n", *ptrMACAddress,
                                   *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3), *(ptrMACAddress + 4), *(ptrMACAddress + 5), offset);
                break;
            }


            /* FDB Entry is available: Do we have a match */
            if (memcmp ((void *)&fe.mac_addr[0], (void*)ptrMACAddress, 6) == 0)
            {
                /* YES: FDB Entry matches the MAC address. We now have the port number */
                portNumber = fe.port_no;
                break;
            }
            else
            {
                /* NO: This is not what we are looking for. Increment the offset and try again */
                offset = offset + 1;
            }
        }

        if(portNumber == -1)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: MAC %02x:%02x:%02x:%02x:%02x:%02x not found, sleeping for %u us\n", *ptrMACAddress,
                               *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3), *(ptrMACAddress + 4), *(ptrMACAddress + 5), gNetfpProxyMcb.pollDelay * 20u);
            usleep (gNetfpProxyMcb.pollDelay * 20u);
        }
    }
    /* Close the bridge socket: */
    close (bridgeSocket);

    /* Did we get a port number? */
    if (portNumber == -1)
        return NULL;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: Bridged port is %d for MAC %02x:%02x:%02x:%02x:%02x:%02x\n", portNumber, *ptrMACAddress,
                       *(ptrMACAddress + 1), *(ptrMACAddress + 2), *(ptrMACAddress + 3), *(ptrMACAddress + 4), *(ptrMACAddress + 5)); //fzm

    /***********************************************************************************************
     * We now need to map the port number to an interface name:
     ***********************************************************************************************/
    snprintf (bridgeDirectory, PATH_MAX, "/sys/class/net/%s/brif/", bridgeName);
    ptrBridgeDirectory = opendir (bridgeDirectory);
    if (ptrBridgeDirectory == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Cant open bridge directory %s\n", bridgeDirectory);
        return NULL;
    }

    /* Cycle through all the directories below the bridge */
    while (1)
    {
        /* Read the interface directory entry. */
        ptrIfaceDirEntry = readdir (ptrBridgeDirectory);
        if (ptrIfaceDirEntry == NULL)
            break;

        /* Skip the current directory */
        if (ptrIfaceDirEntry->d_name[0] == '.')
            continue;

        /* Construct the name for the file where the port number is stored */
        snprintf (bridgeDirectory, PATH_MAX, "/sys/class/net/%s/brif/%s/port_no", bridgeName, ptrIfaceDirEntry->d_name);
        ptrFile = fopen (bridgeDirectory, "r");
        if (ptrFile == NULL)
            continue;

        /* Read the port number from the file: */
        fscanf (ptrFile, "%x", &intfPortNum);

        /* Close the file */
        fclose (ptrFile);

        /* Is this what we are looking for? */
        if (portNumber == intfPortNum)
        {
            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bridged port is %d %s for MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                               portNumber, ptrIfaceDirEntry->d_name, *ptrMACAddress, *(ptrMACAddress + 1), *(ptrMACAddress + 2),
                               *(ptrMACAddress + 3), *(ptrMACAddress + 4), *(ptrMACAddress + 5));
            break;
        }
    }

    /* Did we get an interface name? */
    if (ptrIfaceDirEntry != NULL)
    {
        /* YES: Construct the bridged port name */
        NetfpProxy_constructBridgedPortName (ptrBridgeInterface->ifName, ptrIfaceDirEntry->d_name, &bridgedPortName[0]);

        /* Find the bridged port interface */
        ptrBridgedPortInterface = NetfpProxy_ifaceFindInterface(bridgedPortName);
    }
    else
    {
        /* NO: Not matching entry found. */
        ptrBridgedPortInterface = NULL;
    }

    /* Close the directory */
    closedir (ptrBridgeDirectory);
    return ptrBridgedPortInterface;
}

/**
 *  @b Description
 *  @n
 *      The function is used to offload an interface to the NETFP Server once the route
 *      has been resolved to the next hop MAC address.
 *
 *  @param[in]  ifName
 *      Interface name
 *  @param[in]  ptrNextHopMACAddress
 *      Pointer Next Hop MAC address
 *  @param[out]  ifHandle
 *      Pointer to the offloaded interface handle
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceOffloadInterface (const char* ifName, uint8_t* ptrMACAddress, Netfp_IfHandle* ifHandle)
{
    NetfpProxy_Interface*     ptrNetfpProxyInterface;
    int32_t                   errCode;

    /* Has the interface already been offloaded? */
    ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(ifName);
    if (ptrNetfpProxyInterface == NULL)
    {
        /* Interface does not exist in the NETFP Server. So create the interface in the NETFP Server */
        if (NetfpProxy_ifaceCreateInterface (ifName, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' creation failed\n", ifName);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
            return -1;
        }

        /* Get the Proxy Interface block using the name: */
        ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface (ifName);
        if (ptrNetfpProxyInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Interface '%s' created but unable to find\n", ifName);
            return -1;
        }
    }

    /* Interface has been successfully created in the NETFP Server: We now need to return the interface
     * handle which is to be used to reach the specific MAC address. In most cases the MAC address is
     * directly accessible by the interface name specified; however in the case of a bridged interface
     * the MAC address would be accessible only via the real bridged port. */
    if (NetfpProxy_ifaceIsBridge (ifName) == 0)
    {
        /* NOT a bridged interface: In this case simply return the interface handle we have just created */
        *ifHandle = ptrNetfpProxyInterface->ifHandle;
        return 0;
    }

    /* Bridged Interface: Check out the Bridge FDB database to determine the actual port being used to
     * reach the neighbor MAC address. */
    ptrNetfpProxyInterface = NetfpProxy_ifaceGetBridgedPort (ptrNetfpProxyInterface, ptrMACAddress);
    if (ptrNetfpProxyInterface == NULL)
        return -1;

    /* Populate the interface handle */
    *ifHandle = ptrNetfpProxyInterface->ifHandle;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Offload Interface is '%s' NETFP Handle %p\n", ptrNetfpProxyInterface->ifName,
                       ptrNetfpProxyInterface->ifHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to propogate the interface changes to all the bridged port interfaces.
 *
 *  @param[in]  ptrNetfpProxyInterface
 *      Pointer to the proxy interface on which the interface change was detected
 *  @param[in]  ptrOptCfg
 *      Pointer to the option configuration which has to be performed on all the bridged port interfaces
 *
 *  @retval
 *      Not applicable
 */
static void NetfpProxy_ifacePropogateChanges (NetfpProxy_Interface* ptrNetfpProxyInterface, Netfp_OptionTLV* ptrOptCfg)
{
    int32_t                 errCode;
    int32_t                 index;
    NetfpProxy_Interface*   ptrNetfpProxyBridgedInterface;
    char                    bridgedPortName[NETFP_MAX_CHAR];

    /* Is the interface bridged? */
    if (NetfpProxy_ifaceIsBridge (&ptrNetfpProxyInterface->ifName[0]) == 0)
        return;

    /* Cycle through all the bridged interfaces */
    for (index = 0; index < NETFP_PROXY_MAX_BRIDGE_IF; index++)
    {
        /* Is there a valid bridged interface name? */
        if (ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName[0] == 0)
            continue;

        /* Get the bridged port name: */
        NetfpProxy_constructBridgedPortName (&ptrNetfpProxyInterface->ifName[0], &ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName[0],
                                             &bridgedPortName[0]);

        /* Find the interface: */
        ptrNetfpProxyBridgedInterface = NetfpProxy_ifaceFindInterface (bridgedPortName);
        if (ptrNetfpProxyBridgedInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Bridged Interface '%s' not found in database\n", bridgedPortName);
            return;
        }

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Propogating event to interface '%s'\n", bridgedPortName);

        /* Configure the link status on the bridged interface: */
        if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyBridgedInterface->ifHandle, ptrOptCfg, &errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' Type %d set if option failed [Error code %d]\n",
                               ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName, ptrOptCfg->type, errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
            }
        /* Inherit the MTU & Link status from the bridged interface */
        ptrNetfpProxyBridgedInterface->bIsLinkUp               = ptrNetfpProxyInterface->bIsLinkUp;
        ptrNetfpProxyBridgedInterface->proxyIfCfg.ifConfig.mtu = ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu;
    }
    return;
}

static int32_t NetfpProxy_ifaceAddIfaceToBridgeInterface
(
    NetfpProxy_Interface* ptrNetfpProxyInterface,
    NetfpProxy_BridgeIface bridgePortList[],
    int32_t numBridgedPorts
)
{
    NetfpProxy_InterfaceCfg* proxyIfCfg = &ptrNetfpProxyInterface->proxyIfCfg;
    NetfpProxy_InterfaceCfg proxyIfCfgBridged;
    struct rtnl_link* ptrLink;
    int32_t errCode;
    int32_t retVal = 0;

    for (int index = 0; index < numBridgedPorts; index++)
    {
        int found = 0;
        for(int i = 0; i < NETFP_PROXY_MAX_BRIDGE_IF; ++i)
        {
            if(strncmp(proxyIfCfg->bridgePortList[i].ifName, bridgePortList[index].ifName, NETFP_MAX_CHAR) == 0)
            {
                found = 1;
                break;
            }
        }
        if(found)
            continue;

        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Adding interface %s to %s", bridgePortList[index].ifName, ptrNetfpProxyInterface->ifName);

        /* Initialize the proxy interface configuration: */
        memset ((void *)&proxyIfCfgBridged, 0, sizeof(NetfpProxy_InterfaceCfg));

        /* Get the IP addresses associated with the bridged port interfaces */
        int32_t ret = NetfpProxy_ifaceGetIP(bridgePortList[index].ifName,
                                    &proxyIfCfgBridged.ipAddress[0],
                                    &proxyIfCfgBridged.subnetMask[0]);
        if (ret < 0)
        {
            retVal = -1;
            continue;
        }

         proxyIfCfgBridged.numIP = ret;

        /* Get the link information: */
        if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, bridgePortList[index].ifName, &ptrLink) < 0)
        {
            retVal = -1;
            continue;
        }

        /* Populate the proxy configuration: */
        if (proxyIfCfg->ipAddress[0].ver != Netfp_IPVersion_INVALID)
        {
            // Use the bridge IP
            proxyIfCfgBridged.ifConfig.ipAddress = proxyIfCfg->ipAddress[0];
        }
        else if (proxyIfCfgBridged.ipAddress[0].ver != Netfp_IPVersion_INVALID)
        {
            // Use the bridged IF IP address
            proxyIfCfgBridged.ifConfig.ipAddress = proxyIfCfgBridged.ipAddress[0];
        }

        if (proxyIfCfg->ipAddress[0].ver != Netfp_IPVersion_INVALID)
            proxyIfCfgBridged.ifConfig.subnetMask = proxyIfCfg->subnetMask[0];
        else if (proxyIfCfgBridged.ipAddress[0].ver != Netfp_IPVersion_INVALID)
            proxyIfCfgBridged.ifConfig.subnetMask = proxyIfCfgBridged.subnetMask[0];

        proxyIfCfgBridged.ifConfig.isLogicalInterface = 0;
        proxyIfCfgBridged.ifConfig.mtu                = proxyIfCfg->ifConfig.mtu;
        proxyIfCfgBridged.ifConfig.type               = proxyIfCfg->ifConfig.type;
        proxyIfCfgBridged.ifConfig.vlanId             = proxyIfCfg->ifConfig.vlanId;
        proxyIfCfgBridged.ifConfig.vlanMap            = proxyIfCfg->ifConfig.vlanMap;

        uint32_t sum = 0;
        for(int i = 0; i < sizeof(proxyIfCfg->ifConfig.macAddress); ++i)
            sum += proxyIfCfg->ifConfig.macAddress[i];

        if (sum)
        {
            // Bridge has valid MAC address
            memcpy ((void *)&proxyIfCfgBridged.ifConfig.macAddress[0],
                    (void *)&proxyIfCfg->ifConfig.macAddress[0],
                    6);
        }
        else
        {
            // Otherwise use physical link MAC address
            memcpy ((void *)&proxyIfCfgBridged.ifConfig.macAddress[0],
                    (void *)nl_addr_get_binary_addr (rtnl_link_get_addr (ptrLink)),
                    6);
        }

        /* Construct the bridged port name */
        NetfpProxy_constructBridgedPortName (ptrNetfpProxyInterface->ifName, bridgePortList[index].ifName, &proxyIfCfgBridged.ifConfig.name[0]);

        /* Is the interface operational? */
        if ((rtnl_link_get_flags (ptrLink) & IFF_UP) && (rtnl_link_get_flags (ptrLink) & IFF_RUNNING))
            proxyIfCfgBridged.linkStatus = 1;
        else
            proxyIfCfgBridged.linkStatus = 0;

        /* Bridged port interfaces are *not* monitored by the NETMGR */
        proxyIfCfgBridged.monitorStatus = 0;

        /* Instantiate the interface: */
        if (NetfpProxy_ifaceInstantiate (ptrNetfpProxyInterface->ifName, &proxyIfCfgBridged, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Setting up interface '%s' failed [Error code %d]", ptrNetfpProxyInterface->ifName, errCode);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
            rtnl_link_put (ptrLink);

            retVal = -1;
            continue;
        }

        NetfpProxy_Interface* ptrNetfpProxyBridgedPortInterface = NetfpProxy_ifaceFindInterface (&proxyIfCfgBridged.ifConfig.name[0]);
        if (ptrNetfpProxyBridgedPortInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Cannot find interface '%s' in the PROXY database", proxyIfCfgBridged.ifConfig.name);
            rtnl_link_put (ptrLink);

            retVal = -1;
            continue;
        }

        ptrNetfpProxyBridgedPortInterface->ptrBridgedInterface = ptrNetfpProxyInterface;

        Netfp_L3QoSCfg* ptrL3QOSCfg = &ptrNetfpProxyInterface->l3QoSCfg;
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: %s L3 Shaper on interface '%s'\n",
                           (ptrL3QOSCfg->isEnable == 1) ? "Enabling" : "Disabling", proxyIfCfgBridged.ifConfig.name);

        Netfp_OptionTLV optCfg;
        optCfg.type   = Netfp_Option_L3_QOS;
        optCfg.length = sizeof(Netfp_L3QoSCfg);
        optCfg.value  = (void*)ptrL3QOSCfg;

        /* Setup the L3 QOS shaper: */
        if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyBridgedPortInterface->ifHandle, &optCfg, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure the L3 Shaper on '%s'\n", proxyIfCfgBridged.ifConfig.name);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
            rtnl_link_put (ptrLink);

            retVal = -1;
            continue;
        }

        memcpy(&ptrNetfpProxyBridgedPortInterface->l3QoSCfg, ptrL3QOSCfg, sizeof(ptrNetfpProxyInterface->l3QoSCfg));

        rtnl_link_put (ptrLink);
    }

    memcpy(&proxyIfCfg->bridgePortList, bridgePortList, sizeof(proxyIfCfg->bridgePortList));
    proxyIfCfg->numBridgedPorts = numBridgedPorts;

    return retVal;
}

static int32_t NetfpProxy_ifaceRemIfaceFromBridgeInterface
(
    NetfpProxy_Interface* ptrNetfpProxyInterface,
    NetfpProxy_BridgeIface bridgePortList[],
    int32_t numBridgedPorts
)
{
    NetfpProxy_InterfaceCfg* proxyIfCfg = &ptrNetfpProxyInterface->proxyIfCfg;
    int32_t retVal = 0;

    for(int i = 0; i < NETFP_PROXY_MAX_BRIDGE_IF; ++i)
    {
        if(proxyIfCfg->bridgePortList[i].ifName[0] == '\0')
            continue;

        int found = 0;
        for (int index = 0; index < numBridgedPorts; index++)
        {
            if(strncmp(proxyIfCfg->bridgePortList[i].ifName, bridgePortList[index].ifName, NETFP_MAX_CHAR) == 0)
            {
                found = 1;
                break;
            }
        }

        if(found)
            continue;

        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Deleting interface %s from %s", proxyIfCfg->bridgePortList[i].ifName, ptrNetfpProxyInterface->ifName);

        char bridgedPortName[NETFP_MAX_CHAR];
        NetfpProxy_constructBridgedPortName (ptrNetfpProxyInterface->ifName, proxyIfCfg->bridgePortList[i].ifName, &bridgedPortName[0]);

        NetfpProxy_Interface* ptrNetfpProxyBridgedPortInterface = NetfpProxy_ifaceFindInterface (bridgedPortName);
        if (ptrNetfpProxyBridgedPortInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Bridged Port Interface '%s' not found in database\n",
                               proxyIfCfg->bridgePortList[i].ifName);
            retVal = -1;
            continue;
        }

        /* Destory the bridged port interface: */
        NetfpProxy_ifaceDestroy (ptrNetfpProxyBridgedPortInterface);
        memset(proxyIfCfg->bridgePortList[i].ifName, 0, sizeof(proxyIfCfg->bridgePortList[i].ifName));
    }

    proxyIfCfg->numBridgedPorts = numBridgedPorts;

    return retVal;
}

static int32_t NetfpProxy_ifaceReinitializeBridgeInterface
(
    NetfpProxy_Interface* ptrNetfpProxyInterface,
    const char* ifName
)
{
    Netfp_VLANPriorityMap bridgedVLANMap;
    uint32_t bridgedVLANId;
    const char* bridgeName;
    char tmpBuffer[NETFP_MAX_CHAR];

    if (NetfpProxy_ifaceIsVLAN (ifName, &bridgedVLANId, &bridgedVLANMap) == 0)
    {
        bridgeName = ifName;
    }
    else
    {
        strncpy (tmpBuffer, ifName, NETFP_MAX_CHAR);
        bridgeName = strtok (tmpBuffer, ".");
    }

    NetfpProxy_BridgeIface bridgePortList[NETFP_PROXY_MAX_BRIDGE_IF];
    memset ((void *)&bridgePortList[0], 0, sizeof(bridgePortList));

    int32_t numBridgedPorts = NetfpProxy_ifaceGetBridgedInterfaceList(bridgeName, &bridgePortList[0]);
    if(numBridgedPorts != ptrNetfpProxyInterface->proxyIfCfg.numBridgedPorts)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: changed numPorts on %s, was %d is %d",
                           ptrNetfpProxyInterface->ifName, ptrNetfpProxyInterface->proxyIfCfg.numBridgedPorts, numBridgedPorts);
    }

    if(ptrNetfpProxyInterface->proxyIfCfg.numBridgedPorts < numBridgedPorts)
        return NetfpProxy_ifaceAddIfaceToBridgeInterface(ptrNetfpProxyInterface, bridgePortList, numBridgedPorts);
    else if(ptrNetfpProxyInterface->proxyIfCfg.numBridgedPorts > numBridgedPorts)
        return NetfpProxy_ifaceRemIfaceFromBridgeInterface(ptrNetfpProxyInterface, bridgePortList, numBridgedPorts);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Link event handler. Handles all Link/Interface related events raised by
 *      the Network manager library.
 *
 *  @param[in]  action
 *      Event being notified
 *  @param[in]  type
 *      Event family (NETMGR_LINK_EVT)
 *  @param[in]  obj
 *      Modified network object
 *  @param[in]  infoPtr
 *      Network Proxy state info corresponding to this event (cookie).
 *      Handle passed to network manager at time of event registration.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ifaceMonitorLinkUpdates (int32_t action, int32_t type, void* obj, void*  infoPtr)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    struct rtnl_link*       nlLink;
    char                    tmpBuf[128];
    Netfp_OptionTLV         optCfg;
    int32_t                 errCode;
    uint32_t                linkStatus;

    /* Report the event to the application plugin if it installed an event handler */
    if (gNetfpProxyMcb.pluginCfg.report_net_event != NULL)
        gNetfpProxyMcb.pluginCfg.report_net_event (action, type, obj);

    /* Process event received based on its type */
    nlLink = (struct rtnl_link*)obj;

    /* Get the NETFP Proxy interface: */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)infoPtr;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Received %s Link event ...%s/%d/%s!!\n",
                       NetfpProxy_action2Str (action), rtnl_link_get_name (nlLink), rtnl_link_get_mtu (nlLink),
                       rtnl_link_flags2str(rtnl_link_get_flags (nlLink), tmpBuf, sizeof (tmpBuf)));

    /* Process the event accordingly: */
    switch (action)
    {
        case NL_ACT_DEL:
        {
            /* Received Link Delete Event: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Delete Interface '%s' event\n", ptrNetfpProxyInterface->ifName);
            if (NetfpProxy_ifaceDeleteInterface (ptrNetfpProxyInterface->ifName, &errCode) < 0)
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
            return;
        }
        case NL_ACT_NEW:
        {
            /* Received NEW Link event. New interfaces are added at runtime only via offload commands */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Ignoring New Link detected. \n");
            return;
        }
        case NL_ACT_CHANGE:
        {
            /* Did the link go up/down? */
            if ((rtnl_link_get_flags (nlLink) & IFF_UP) && (rtnl_link_get_flags (nlLink) & IFF_RUNNING))
                linkStatus = 1;
            else
                linkStatus = 0;

            /* Report the change to the NETFP only if required to do so: */
            if (ptrNetfpProxyInterface->bIsLinkUp != linkStatus)
            {
                /* There was a change in the LINK status; inform the NETFP */
                ptrNetfpProxyInterface->bIsLinkUp = linkStatus;

                /* Debug Message: */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Link status for '%s' has changed to %d\n",
                                   ptrNetfpProxyInterface->ifName, ptrNetfpProxyInterface->bIsLinkUp);

                /* Update the NETFP Interface configuration */
                optCfg.type   = Netfp_Option_IFACE_STATUS;
                optCfg.length = sizeof (uint32_t);
                optCfg.value  = (void *)&ptrNetfpProxyInterface->bIsLinkUp;
                Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, &errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm

                /* Propogate the changes: */
                NetfpProxy_ifacePropogateChanges (ptrNetfpProxyInterface, &optCfg);
            }

            /* Did link MTU change? */
            if (rtnl_link_get_mtu (nlLink) != ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: MTU for '%s' has changed from %d to %d\n",
                                   ptrNetfpProxyInterface->ifName, ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu, rtnl_link_get_mtu (nlLink));

                /* Keep track of the new MTU: */
                ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu = rtnl_link_get_mtu (nlLink);

                /* Let NETFP know of the MTU change */
                optCfg.type   = Netfp_Option_MTU;
                optCfg.length = sizeof (uint32_t);
                optCfg.value  = (void *)&ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu;
                Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, &errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm

                /* Propogate the changes: */
                NetfpProxy_ifacePropogateChanges (ptrNetfpProxyInterface, &optCfg);
            }

            char* ifName = rtnl_link_get_name (nlLink);
            if (NetfpProxy_ifaceIsBridge (ifName) == 0)
                return;

            NetfpProxy_ifaceReinitializeBridgeInterface (ptrNetfpProxyInterface, ifName);

            return;
        }
        default:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Dropping Unknown link update event [%d] detected\n", action);
            return;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Address event handler. Handles all IP Address related events raised by
 *      the Network manager library.
 *
 *  @param[in]  action
 *      Event being notified
 *  @param[in]  type
 *      Event family (NETMGR_ADDR_EVT)
 *  @param[in]  obj
 *      Modified network object
 *  @param[in]  infoPtr
 *      Network Proxy state info corresponding to this event (cookie).
 *      Handle passed to network manager at time of event registration.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ifaceMonitorAddrUpdates (int32_t action, int32_t type, void* obj, void*  infoPtr)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    struct nl_addr*         nlIPAddr;
    struct rtnl_addr*       rtnlIPAddr;
    Netfp_InterfaceIP       interfaceIP;
    int32_t                 index;
    Netfp_OptionTLV         optCfg;
    int32_t                 errCode;
    char                    tmpBuf[128];

    /* Report the event to the application plugin if an event handler is installed */
    if (gNetfpProxyMcb.pluginCfg.report_net_event != NULL)
        gNetfpProxyMcb.pluginCfg.report_net_event (action, type, obj);

    /* Get the NETFP Proxy interface */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)infoPtr;
    rtnlIPAddr = (struct rtnl_addr*)obj;
    nlIPAddr   = rtnl_addr_get_local (rtnlIPAddr);

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Received %s Addr event for %s...!!\n",
                       NetfpProxy_action2Str (action), nl_addr2str ((nlIPAddr), tmpBuf, sizeof (tmpBuf)));

    /* Process the event accordingly: */
    switch (action)
    {
        case NL_ACT_DEL:
        {
            /* Find the matching IP address: */
            index = NetfpProxy_ifaceFindIP (ptrNetfpProxyInterface, nlIPAddr);
            if (index < 0)
            {
                /* Error: IP address is not assigned to the interface */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Warning: Deleting address; but no local address found\n");
                return;
            }

            /* Initialize the Interface IP address */
            interfaceIP.ipAddress  = ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index];
            interfaceIP.subnetMask = ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index];

            /* Delete the IP address from the interface */
            optCfg.type   = Netfp_Option_DEL_IP;
            optCfg.length = sizeof (Netfp_InterfaceIP);
            optCfg.value  = (void*)&interfaceIP;
            if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, &errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Delete IP failed with error: %d.\n", errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
                return;
            }

            /* Mark the entry as free: */
            ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver  = Netfp_IPVersion_INVALID;
            ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].ver = Netfp_IPVersion_INVALID;
            break;
        }
        case NL_ACT_NEW:
        {
            /* Find the matching IP address: */
            index = NetfpProxy_ifaceFindIP (ptrNetfpProxyInterface, nlIPAddr);
            if (index >= 0)
            {
                /* Error: Duplicate IP address assignment */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Warning: Duplicate IP address assignment\n");
                return;
            }

            /* Cycle through all the IP addresses which are assigned to the interface */
            for (index = 0; index < NETFP_PROXY_MAX_IP; index++)
            {
                /* Do we have a free entry? */
                if (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver == Netfp_IPVersion_INVALID)
                    break;
            }
            if (index == NETFP_PROXY_MAX_IP)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: MAX limit of IP Addresses reached\n");
                return;
            }

            /* Is this an IPv4 address? */
            if (nl_addr_get_family (nlIPAddr) == AF_INET)
            {
                /* IPv4 Address: */
                ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver = Netfp_IPVersion_IPV4;
                memcpy ((void *)&ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[0], (void *)nl_addr_get_binary_addr (nlIPAddr), 4);

                /* Populate the subnet mask: */
                ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].ver = Netfp_IPVersion_IPV4;
                NetfpProxy_convertV4Prefix2Mask (nl_addr_get_prefixlen (nlIPAddr), &ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv4.u.a32);
            }
            else
            {
                /* IPv6 Address: */
                ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver = Netfp_IPVersion_IPV6;
                memcpy ((void *)&ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv6.u.a8[0], (void *)nl_addr_get_binary_addr (nlIPAddr), 16);

                /* Populate the subnet mask: */
                ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].ver = Netfp_IPVersion_IPV6;
                NetfpProxy_convertV6Prefix2Mask (nl_addr_get_prefixlen (nlIPAddr), &ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv6.u.a32[0]);
            }

            /* Setup the interface IP */
            interfaceIP.ipAddress  = ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index];
            interfaceIP.subnetMask = ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index];

            /* Add the IP address to NETFP */
            optCfg.type   = Netfp_Option_ADD_IP;
            optCfg.length = sizeof (Netfp_InterfaceIP);
            optCfg.value  = (void*)&interfaceIP;
            if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, &errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to add the address to NETFP Interface '%s' [Error code %d]\n",
                                   ptrNetfpProxyInterface->ifName, errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
                return;
            }
            break;
        }
        case NL_ACT_CHANGE:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Ignoring the address change event\n");
            return;
        }
        default:
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Dropping Unknown address update event [%d] detected\n", action);
            return;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to instantiate the interface in the NETFP Proxy and Server.
 *
 *  @param[in]  ifName
 *      Interface name.
 *  @param[in]  ptrProxyIfConfig
 *      Pointer to the proxy interface configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_ifaceInstantiate
(
    const char*                 ifName,
    NetfpProxy_InterfaceCfg*    ptrProxyIfConfig,
    int32_t*                    errCode
)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    uint32_t                index;
    Netfp_OptionTLV         optCfg;
    Netfp_InterfaceIP       interfaceIP;

    /* Allocate memory for the NETFP Proxy interface: */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)malloc (sizeof(NetfpProxy_Interface));
    if (ptrNetfpProxyInterface == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: OOM while allocating the proxy interface\n");
        return -1;
    }

    /* Initialize the allocated memory */
    memset ((void*)ptrNetfpProxyInterface, 0, sizeof(NetfpProxy_Interface));

    /* Populate the NETFP Proxy interface: */
    strncpy (ptrNetfpProxyInterface->ifName, ptrProxyIfConfig->ifConfig.name, NETFP_MAX_CHAR);
    memcpy ((void*)&ptrNetfpProxyInterface->proxyIfCfg, (void*)ptrProxyIfConfig, sizeof(NetfpProxy_InterfaceCfg));
    ptrNetfpProxyInterface->bIsLinkUp = ptrProxyIfConfig->linkStatus;

    /* Create the interface in the NETFP Server: */
    ptrNetfpProxyInterface->ifHandle = Netfp_createInterface (gNetfpProxyMcb.netfpClientHandle, &ptrProxyIfConfig->ifConfig, errCode);
    if (ptrNetfpProxyInterface->ifHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to create the NETFP Interface '%s' [Error code %d]\n",
                           ptrNetfpProxyInterface->ifName, *errCode);
        NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
         /* Cleanup the interface */
        free (ptrNetfpProxyInterface);

        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface '%s' NETFP Handle %p\n", ptrNetfpProxyInterface->ifName,
                       ptrNetfpProxyInterface->ifHandle);

    /* We have already added the first IP Address during interface creation. So here we add additional IP addreses if any */
    for (index = 1; index < ptrProxyIfConfig->numIP; index++)
    {
        /* Initialize the Interface IP address */
        interfaceIP.ipAddress  = ptrProxyIfConfig->ipAddress[index];
        interfaceIP.subnetMask = ptrProxyIfConfig->subnetMask[index];

        /* Add the IP address to NETFP */
        optCfg.type   = Netfp_Option_ADD_IP;
        optCfg.length = sizeof (Netfp_InterfaceIP);
        optCfg.value  = (void*)&interfaceIP;
        if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, errCode) < 0)
        {

            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to add the address to NETFP Interface '%s' [Error code %d]\n",
                               ptrNetfpProxyInterface->ifName, *errCode);
            NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm

            /* Cleanup the interface */
            free (ptrNetfpProxyInterface);

            return -1;
        }
    }

    /* Do we need to monitor the interface for link & address events? */
    if (ptrProxyIfConfig->monitorStatus == 1)
    {
        /* YES: Start monitoring this interface for link & address events */
        ptrNetfpProxyInterface->linkEvtSubscHdl = netmgr_register_link_updates (gIfMgmtMCB.netMgrIfHandle, ptrNetfpProxyInterface->ifName,
                                                                                &NetfpProxy_ifaceMonitorLinkUpdates, ptrNetfpProxyInterface, errCode);
        ptrNetfpProxyInterface->addrEvtSubscHdl = netmgr_register_addr_updates (gIfMgmtMCB.netMgrIfHandle, ptrNetfpProxyInterface->ifName,
                                                                                &NetfpProxy_ifaceMonitorAddrUpdates, ptrNetfpProxyInterface, errCode);
    }
    else
    {
        /* No: Reset the handles */
        ptrNetfpProxyInterface->linkEvtSubscHdl = NULL;
        ptrNetfpProxyInterface->addrEvtSubscHdl = NULL;

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Bypassing the Link/Address updates for '%s'\n", ptrNetfpProxyInterface->ifName);
    }

    /* Add the interface to the list of interfaces being handled by NETFP Proxy: */
    List_addNode ((List_Node**)&gIfMgmtMCB.ifaceList, (List_Node*)ptrNetfpProxyInterface);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create an interface and register with the
 *      NETFP Server.
 *
 *  @param[in]  ifName
 *      Interface name.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceCreateInterface (const char* ifName, int32_t* errCode)
{
    NetfpProxy_InterfaceCfg proxyIfCfg;
    NetfpProxy_InterfaceCfg proxyIfCfgBridged;
    const char*             bridgeName;
    int32_t                 index;
    NetfpProxy_BridgeIface  bridgePortList[NETFP_PROXY_MAX_BRIDGE_IF];
    int32_t                 numBridgedPorts;
    Netfp_InterfaceType     bridgedPortIfType;
    Netfp_InterfaceType     bridgedIfType;
    uint32_t                bridgedVLANId;
    uint32_t                bridgedPortVLANId;
    char                    tmpBuffer[NETFP_MAX_CHAR];
    Netfp_VLANPriorityMap   bridgedVLANMap;
    Netfp_VLANPriorityMap   bridgedPortVLANMap;
    struct rtnl_link*       ptrLink;
    uint32_t                bridgeMTU;
    int32_t                 bridgeMacAddrLen;
    int                     ret;
    NetfpProxy_Interface*   ptrNetfpProxyInterface;

    /* Sanity Check: We dont allow the creation of duplicate entries, but
       if it is a bridge, then reconfigure it, maybe number of interfaces changed */
    if ((ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(ifName)) != NULL)
    {
        if (NetfpProxy_ifaceIsBridge (ifName) == 1)
            return NetfpProxy_ifaceReinitializeBridgeInterface (ptrNetfpProxyInterface, ifName);
        else
            return 0;
    }

    /* Sanity Check: Ensure that the interface name exists in the system */
    if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, ifName, &ptrLink) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' not found\n", ifName);
        return -1;
    }
    rtnl_link_put (ptrLink);

    /* Is this a Bridge Interface? */
    if (NetfpProxy_ifaceIsBridge (ifName) == 0)
    {
        /***********************************************************************************
         * NON-Bridged Interface: This handles the following interfaces
         *  (a) ethX
         *  (b) ethX.y
         * Initialize the proxy interface configuration:
         ***********************************************************************************/
        memset ((void *)&proxyIfCfg, 0, sizeof(NetfpProxy_InterfaceCfg));

        /* We get all the IP addresses associated with the the interface */
        ret = NetfpProxy_ifaceGetIP(ifName, &proxyIfCfg.ipAddress[0], &proxyIfCfg.subnetMask[0]);
        if (ret < 0)
            return -1;
        proxyIfCfg.numIP = ret;

        /* Get the link information: */
        if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, ifName, &ptrLink) < 0)
            return -1;

        /* Populate the proxy configuration: */
        proxyIfCfg.ifConfig.ipAddress          = proxyIfCfg.ipAddress[0];
        proxyIfCfg.ifConfig.subnetMask         = proxyIfCfg.subnetMask[0];
        proxyIfCfg.ifConfig.isLogicalInterface = 0;
        proxyIfCfg.ifConfig.mtu                = rtnl_link_get_mtu (ptrLink);
        strncpy (proxyIfCfg.ifConfig.name, ifName, NETFP_MAX_CHAR);
        memcpy ((void *)&proxyIfCfg.ifConfig.macAddress[0], (void *)nl_addr_get_binary_addr (rtnl_link_get_addr (ptrLink)), 6);

        /* Is the interface a VLAN or not? */
        if (NetfpProxy_ifaceIsVLAN (proxyIfCfg.ifConfig.name, &proxyIfCfg.ifConfig.vlanId, &proxyIfCfg.ifConfig.vlanMap) == 1)
            proxyIfCfg.ifConfig.type = Netfp_InterfaceType_VLAN;
        else
            proxyIfCfg.ifConfig.type = Netfp_InterfaceType_ETH;

        /* Is the interface operational? */
        if ((rtnl_link_get_flags (ptrLink) & IFF_UP) && (rtnl_link_get_flags (ptrLink) & IFF_RUNNING))
            proxyIfCfg.linkStatus = 1;
        else
            proxyIfCfg.linkStatus = 0;

        /* The interface needs to be monitored by the NETMGR */
        proxyIfCfg.monitorStatus = 1;

        /* Instantiate the interface: */
        if (NetfpProxy_ifaceInstantiate (ifName, &proxyIfCfg, errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Setting up interface '%s' failed [Error code %d]\n", ifName, *errCode);
            rtnl_link_put (ptrLink); //fzm
            NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
            return -1;
        }

        /* Release the link */
        rtnl_link_put (ptrLink);
        return 0;
    }

    /****************************************************************************************
     * Bridged Interface: The bridge can be configured as following:
     *  (a) br0 is added on top of eth0.x and eth1.x
     *  (b) br0 is added on top of eth0 and eth1
     *  (c) br0.x is added on top of eth0 and eth1
     ****************************************************************************************/
    if (NetfpProxy_ifaceIsVLAN (ifName, &bridgedVLANId, &bridgedVLANMap) == 0)
    {
        /* Case (a) and Case (b): The bridge is an Ethernet interface type */
        bridgeName    = ifName;
        bridgedIfType = Netfp_InterfaceType_ETH;
    }
    else
    {
        /* Case (c): We need to derive the actual bridge name i.e. translate br0.x to br0 and then find everything below it.
         * Copy the interface name into the temporary buffer */
        strncpy (tmpBuffer, ifName, NETFP_MAX_CHAR);
        bridgeName    = strtok (tmpBuffer, ".");
        bridgedIfType = Netfp_InterfaceType_VLAN;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Creating Bridge Interface '%s' Interface Name '%s' %d\n",
                       bridgeName, ifName, bridgedIfType);

    /* Initialize the bridged port list */
    memset ((void *)&bridgePortList[0], 0, sizeof(bridgePortList));

    /* Get the number of interfaces below the bridge: */
    numBridgedPorts = NetfpProxy_ifaceGetBridgedInterfaceList(bridgeName, &bridgePortList[0]);
    if (numBridgedPorts < 0)
        return -1;

    /* If there are no bridged ports there is no point in continuing. */
    if (numBridgedPorts == 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Cannot create a bridged interface '%s' with no bridged ports\n", ifName);
        return -1;
    }

    /* All the bridged interfaces should have the same ethernet type. This is as configured by Linux. */
    if (NetfpProxy_ifaceIsVLAN (bridgePortList[0].ifName, &bridgedPortVLANId, &bridgedPortVLANMap) == 1)
        bridgedPortIfType = Netfp_InterfaceType_VLAN;
    else
        bridgedPortIfType = Netfp_InterfaceType_ETH;

    /* We now need to decide the interface type for the creation of this entire set:
     * This is the set of rules used:
     * (a) Bridge Interface (br0) is VLAN and then if the bridged interfaces (eth0, eth1) are ETH;
     *     the interface type is VLAN for all the interfaces
     * (b) Bridge Interface (br0) is ETH and then if the bridged interfaces (eth0, eth1) are VLAN;
     *     the interface type is VLAN for all the interfaces
     * (c) Bridge Interface (br0) is ETH and then if the bridged interfaces (eth0, eth1) are ETH;
     *     the interface type is ETH for all the interfaces
     * (d) Bridged Interface (br0) is VLAN and bridged interfaces are also VLAN is NOT supported */
    if ((bridgedIfType == Netfp_InterfaceType_VLAN) && (bridgedPortIfType == Netfp_InterfaceType_ETH))
    {
        /* Case (a): Bridged & Bridged Port interfaces are now VLAN */
        bridgedIfType      = Netfp_InterfaceType_VLAN;
        bridgedPortIfType  = Netfp_InterfaceType_VLAN;
        bridgedPortVLANId  = bridgedVLANId;
        bridgedPortVLANMap = bridgedVLANMap;
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Case(a) detected switching to VLAN interface with VLAN Id %d\n", bridgedVLANId);
    }
    else if ((bridgedIfType == Netfp_InterfaceType_ETH) && (bridgedPortIfType == Netfp_InterfaceType_VLAN))
    {
        /* Case (b): Bridged & Bridged Port interfaces are now VLAN */
        bridgedIfType     = Netfp_InterfaceType_VLAN;
        bridgedPortIfType = Netfp_InterfaceType_VLAN;
        bridgedVLANId     = bridgedPortVLANId;
        bridgedVLANMap    = bridgedPortVLANMap;
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Case(b) detected switching to VLAN interface with VLAN Id %d\n", bridgedVLANId);
    }
    else if ((bridgedIfType == Netfp_InterfaceType_ETH) && (bridgedPortIfType == Netfp_InterfaceType_ETH))
    {
        /* Case (c): Interface type is ETH */
        bridgedIfType     = Netfp_InterfaceType_ETH;
        bridgedPortIfType = Netfp_InterfaceType_ETH;
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Case(c) detected remaining as ETH interface\n");
    }
    else
    {
        /* Case (d): Unsupported */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Bridge %s is VLAN & Bridged Port %s is VLAN. Unsupported use case\n",
                           ifName, bridgePortList[0].ifName);
        return -1;
    }
//fzm-->
    if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, ifName, &ptrLink) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Getting link by name failed for interface: %s\n", ifName);
        return -1;
    }

    bridgeMTU = rtnl_link_get_mtu(ptrLink);
    rtnl_link_put (ptrLink);
//fzm<--

    /********************************************************************************
     * Setup up the bridged interfaces:
     *  Initialize the proxy interface configuration
     ********************************************************************************/
    memset ((void *)&proxyIfCfg, 0, sizeof(NetfpProxy_InterfaceCfg));

    /* Get the IP addresses associated with the bridged port interfaces */
    ret = NetfpProxy_ifaceGetIP(ifName, &proxyIfCfg.ipAddress[0], &proxyIfCfg.subnetMask[0]);
    if (ret < 0)
        return -1;
    proxyIfCfg.numIP = ret;

    /* Get the link information: */
    if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, ifName, &ptrLink) < 0)
        return -1;

    /* Get the bridge MTU: The bridge MTU is inherited and populated to all the bridged port interfaces */
    bridgeMTU = rtnl_link_get_mtu (ptrLink);

    /* Populate the proxy configuration:
     *  Bridged Interfaces are LOGICAL interfaces */
    proxyIfCfg.ifConfig.ipAddress          = proxyIfCfg.ipAddress[0];
    proxyIfCfg.ifConfig.subnetMask         = proxyIfCfg.subnetMask[0];
    proxyIfCfg.ifConfig.isLogicalInterface = 1;
    proxyIfCfg.ifConfig.mtu                = bridgeMTU;
    proxyIfCfg.ifConfig.type               = bridgedIfType;
    proxyIfCfg.ifConfig.vlanId             = bridgedVLANId;
    proxyIfCfg.ifConfig.vlanMap            = bridgedVLANMap;
    proxyIfCfg.numBridgedPorts             = numBridgedPorts;
    memcpy ((void *)&proxyIfCfg.bridgePortList, (void *)&bridgePortList, sizeof(bridgePortList));
    strncpy (proxyIfCfg.ifConfig.name, ifName, NETFP_MAX_CHAR);
    memcpy ((void *)&proxyIfCfg.ifConfig.macAddress[0], (void *)nl_addr_get_binary_addr (rtnl_link_get_addr (ptrLink)), 6);

    bridgeMacAddrLen = nl_addr_get_len(rtnl_link_get_addr (ptrLink));

    /* Is the interface operational? */
    if ((rtnl_link_get_flags (ptrLink) & IFF_UP) && (rtnl_link_get_flags (ptrLink) & IFF_RUNNING))
        proxyIfCfg.linkStatus = 1;
    else
        proxyIfCfg.linkStatus = 0;

    /* Bridged interfaces are monitored by the NETMGR */
    proxyIfCfg.monitorStatus = 1;

    /* Instantiate the interface: */
    if (NetfpProxy_ifaceInstantiate (ifName, &proxyIfCfg, errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Setting up interface '%s' failed [Error code %d]\n", ifName, *errCode);
        NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
        return -1;
    }

    /* Release the link: */
    rtnl_link_put (ptrLink);

    /********************************************************************************
     * Setup up the bridged port interfaces
     ********************************************************************************/
    for (index = 0; index < numBridgedPorts; index++)
    {
        /* Initialize the proxy interface configuration: */
        memset ((void *)&proxyIfCfgBridged, 0, sizeof(NetfpProxy_InterfaceCfg));

        /* Get the IP addresses associated with the bridged port interfaces */
        ret = NetfpProxy_ifaceGetIP(bridgePortList[index].ifName,
                                    &proxyIfCfgBridged.ipAddress[0],
                                    &proxyIfCfgBridged.subnetMask[0]);
        if (ret < 0)
            return -1;
         proxyIfCfgBridged.numIP = ret;

        /* Get the link information: */
        if (netmgr_link_get_by_name (gIfMgmtMCB.netMgrIfHandle, bridgePortList[index].ifName, &ptrLink) < 0)
            return -1;

        /* Populate the proxy configuration: */
        if (proxyIfCfg.ipAddress[0].ver != Netfp_IPVersion_INVALID){
            // Use the bridge IP
            proxyIfCfgBridged.ifConfig.ipAddress      = proxyIfCfg.ipAddress[0];
        } else if (proxyIfCfgBridged.ipAddress[0].ver != Netfp_IPVersion_INVALID) {
            // Use the bridged IF IP address
            proxyIfCfgBridged.ifConfig.ipAddress      = proxyIfCfgBridged.ipAddress[0];
        }
        if (proxyIfCfg.ipAddress[0].ver != Netfp_IPVersion_INVALID) {
            proxyIfCfgBridged.ifConfig.subnetMask     = proxyIfCfg.subnetMask[0];
        } else if (proxyIfCfgBridged.ipAddress[0].ver != Netfp_IPVersion_INVALID) {
            proxyIfCfgBridged.ifConfig.subnetMask     = proxyIfCfgBridged.subnetMask[0];
        }
        proxyIfCfgBridged.ifConfig.isLogicalInterface = 0;
        proxyIfCfgBridged.ifConfig.mtu                = bridgeMTU;
        proxyIfCfgBridged.ifConfig.type               = bridgedPortIfType;
        proxyIfCfgBridged.ifConfig.vlanId             = bridgedPortVLANId;
        proxyIfCfgBridged.ifConfig.vlanMap            = bridgedPortVLANMap;
        if (bridgeMacAddrLen) {
            // Bridge IF has valid MAC address
            memcpy ((void *)&proxyIfCfgBridged.ifConfig.macAddress[0],
                    (void *)&proxyIfCfg.ifConfig.macAddress[0],
                    6);
        } else {
            // Otherwise use physical link MAC address
            memcpy ((void *)&proxyIfCfgBridged.ifConfig.macAddress[0],
                    (void *)nl_addr_get_binary_addr (rtnl_link_get_addr (ptrLink)),
                    6);
        }

        /* Construct the bridged port name */
        NetfpProxy_constructBridgedPortName (ifName, bridgePortList[index].ifName, &proxyIfCfgBridged.ifConfig.name[0]);

        /* Is the interface operational? */
        if ((rtnl_link_get_flags (ptrLink) & IFF_UP) && (rtnl_link_get_flags (ptrLink) & IFF_RUNNING))
            proxyIfCfgBridged.linkStatus = 1;
        else
            proxyIfCfgBridged.linkStatus = 0;

        /* Bridged port interfaces are *not* monitored by the NETMGR */
        proxyIfCfgBridged.monitorStatus = 0;

        /* Instantiate the interface: */
        if (NetfpProxy_ifaceInstantiate (ifName, &proxyIfCfgBridged, errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Setting up interface '%s' failed [Error code %d]\n", ifName, *errCode);
            NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
            rtnl_link_put (ptrLink);
            return -1;
        }
        rtnl_link_put (ptrLink);
    }



    /**************************************************************************************
     * All the interfaces have been setup in the NETFP Proxy & Server. Link the
     * bridged interface & bridged port interfaces now.
     *************************************************************************************/
    {
        NetfpProxy_Interface*   ptrNetfpProxyBridgedPortInterface;
        NetfpProxy_Interface*   ptrNetfpProxyBridgedInterface;
        char                    bridgedPortName[NETFP_MAX_CHAR];

        /* Get the bridged interface: */
        ptrNetfpProxyBridgedInterface = NetfpProxy_ifaceFindInterface (ifName);
        if (ptrNetfpProxyBridgedInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Cannot find interface '%s' in the PROXY database\n", ifName);
            return -1;
        }

        /* Cycle through all the bridged ports and link them together */
        for (index = 0; index < numBridgedPorts; index++)
        {
            /* Construct the bridged port name */
            NetfpProxy_constructBridgedPortName (ifName, bridgePortList[index].ifName, &bridgedPortName[0]);

            /* Get the bridged port interface: */
            ptrNetfpProxyBridgedPortInterface = NetfpProxy_ifaceFindInterface (&bridgedPortName[0]);
            if (ptrNetfpProxyBridgedPortInterface == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Cannot find interface '%s' in the PROXY database\n",
                                                   bridgedPortName); //fzm
                return -1;
            }

            /* Bridged Port interface is now linked to the bridged interface */
            ptrNetfpProxyBridgedPortInterface->ptrBridgedInterface = ptrNetfpProxyBridgedInterface;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to destroy the interface from the NETFP Proxy and Server
 *
 *  @param[in]  ptrNetfpProxyInterface
 *      Pointer to the interface to be destroyed
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_ifaceDestroy (NetfpProxy_Interface* ptrNetfpProxyInterface)
{
    int32_t errCode;

    /* Delete the interface from the NETFP Server */
    if (Netfp_deleteInterface (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the interface '%s' [Error code %d]\n",
                           ptrNetfpProxyInterface->ifName, errCode);
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
        return -1;
    }

    /* Unregister the interface for link & address updates */
    if (ptrNetfpProxyInterface->linkEvtSubscHdl != NULL)
        netmgr_unregister_link_updates (gIfMgmtMCB.netMgrIfHandle, ptrNetfpProxyInterface->linkEvtSubscHdl);
    if (ptrNetfpProxyInterface->addrEvtSubscHdl != NULL)
        netmgr_unregister_addr_updates (gIfMgmtMCB.netMgrIfHandle, ptrNetfpProxyInterface->addrEvtSubscHdl);

    /* Remove the interface from the list */
    List_removeNode ((List_Node**)&gIfMgmtMCB.ifaceList, (List_Node*)ptrNetfpProxyInterface);

    /* Cleanup the interface */
    free (ptrNetfpProxyInterface);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete an interface
 *
 *  @param[in]  ifName
 *      Interface name.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceDeleteInterface (const char* ifName, int32_t* errCode)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    NetfpProxy_Interface*   ptrNetfpProxyBridgedPortInterface;
    uint32_t                index;
    char                    bridgedPortName[NETFP_MAX_CHAR];

    /* Get the pointer to the interface being deleted: */
    ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(ifName);
    if (ptrNetfpProxyInterface == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to find the interface '%s' for deletion\n", ifName);
        return -1;
    }

    /* Inform the routing module that the interface has been deleted. We simply go ahead and kill all the
     * routes which are residing on top of the interface. Killing the routes will also stop the neighbor
     * monitoring. */
    NetfpProxy_routeKill(ifName);

    /* Is this a bridged interface which is being deleted? */
    if (NetfpProxy_ifaceIsBridge (ifName) == 0)
    {
        /* NO: Destroy the interface from the NETFP Proxy/Server */
        NetfpProxy_ifaceDestroy (ptrNetfpProxyInterface);
        return 0;
    }

    /* Delete a bridged interface: Cycle through all the bridged port interfaces and delete them. */
    for (index = 0; index < NETFP_PROXY_MAX_BRIDGE_IF; index++)
    {
        /* Is there a valid bridged interface name? */
        if (ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName[0] == 0)
            continue;

        /* Construct the bridged port name */
        NetfpProxy_constructBridgedPortName (ifName, ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName, &bridgedPortName[0]);

        /* Find the interface: */
        ptrNetfpProxyBridgedPortInterface = NetfpProxy_ifaceFindInterface (bridgedPortName);
        if (ptrNetfpProxyBridgedPortInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Bridged Port Interface '%s' not found in database\n",
                               ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName);
            return -1;
        }

        /* Destory the bridged port interface: */
        NetfpProxy_ifaceDestroy (ptrNetfpProxyBridgedPortInterface);
    }

    /* Destroy the Bridge Interface */
    NetfpProxy_ifaceDestroy (ptrNetfpProxyInterface);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked from the IPC command handler and is invoked after the OAM application
 *      has modified the VLAN egress map configuration for a specific VLAN device. The function will
 *      read the new EGRESS MAP configuration and pass this to the NETFP Server
 *
 *  @param[in]  ifName
 *      Interface name.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceFlushVLANEgressMap(const char* ifName, int32_t* errCode)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    NetfpProxy_Interface*   ptrNetfpProxyBridgedInterface;
    NetfpProxy_Interface*   ptrNetfpProxyBridgedPortInterface;
    uint32_t                index;
    uint32_t                vlanId;
    Netfp_OptionTLV         optCfg;
    char                    bridgedPortName[NETFP_MAX_CHAR];
    char                    bridgeName[NETFP_MAX_CHAR];

    /* Is the interface under a bridge? */
    if (NetfpProxy_ifaceIsBridged (ifName, &bridgeName[0]) == 1)
    {
        /* YES: Interface is under a bridge. Construct the bridged port name */
        NetfpProxy_constructBridgedPortName (bridgeName, ifName, &bridgedPortName[0]);

        /* Get the NETFP Proxy interface associated with the interface */
        ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(bridgedPortName);
        if (ptrNetfpProxyInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' has not been added to NETFP Proxy\n", bridgedPortName);
            return -1;
        }
    }
    else
    {
        /* NO: Interface is not under a bridge. */
        ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(ifName);
        if (ptrNetfpProxyInterface == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' has not been added to NETFP Proxy\n", ifName);
            return -1;
        }
    }

    /* Get the VLAN identifer and Egress MAP configuration from the kernel */
    if (NetfpProxy_ifaceIsVLAN (ifName, &vlanId, &ptrNetfpProxyInterface->proxyIfCfg.ifConfig.vlanMap) == 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' is not a VLAN interface\n", ifName);
        return -1;
    }

    /* Setup the new VLAN configuration: */
    optCfg.type   = Netfp_Option_VLAN_EGRESS_PRIORITY;
    optCfg.length = sizeof(Netfp_VLANPriorityMap);
    optCfg.value  = &ptrNetfpProxyInterface->proxyIfCfg.ifConfig.vlanMap;

    /* Modify the mapping: */
    if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure VLAN Egress map for %s [Error code %d]\n", ifName, *errCode);
        NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface '%s' VLAN egress configured\n", ifName);

    /***********************************************************************************************
     * The egress map configuration might need to be propogated further. The following scenarios
     * need to be handled:
     *  Case (a): eth0.2 egress map for has changed. We dont need to do anything further.
     *  Case (b): br0 has eth0.2 and eth1.2. Egress Map for eth0.2 and eth1.2 is changed. We need
     *            to setup the egress map for br0 also.
     *  Case (c): br0.2 has eth0 and eth1. Egress Map for br0.2 changed. We need to setup the
     *            egress map for eth0 and eth1 also.
     ***********************************************************************************************/
    if (NetfpProxy_ifaceIsBridge (ifName) == 0)
    {
        /* Case (a) or Case (b): */
        ptrNetfpProxyBridgedInterface = ptrNetfpProxyInterface->ptrBridgedInterface;
        if (ptrNetfpProxyBridgedInterface == NULL)
        {
            /* Case (a): The interface does not reside below a bridge. */
            return 0;
        }
        else
        {
            /* Case (b): The interface resides belows a bridge. Modify the VLAN Egress map for the bridged interface.
             * Copy over the VLAN Egress map configuration: */
            memcpy ((void *)&ptrNetfpProxyBridgedInterface->proxyIfCfg.ifConfig.vlanMap,
                    (void *)&ptrNetfpProxyInterface->proxyIfCfg.ifConfig.vlanMap,
                    sizeof(Netfp_VLANPriorityMap));

            /* Setup the new VLAN configuration: */
            optCfg.type   = Netfp_Option_VLAN_EGRESS_PRIORITY;
            optCfg.length = sizeof(Netfp_VLANPriorityMap);
            optCfg.value  = &ptrNetfpProxyBridgedInterface->proxyIfCfg.ifConfig.vlanMap;

            /* Setup the configuration in the NETFP Server: */
            if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyBridgedInterface->ifHandle, &optCfg, errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure VLAN Egress map for %s [Error code %d]\n",
                                   ptrNetfpProxyBridgedInterface->ifName, *errCode);
                NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
                return -1;
            }
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface '%s' VLAN egress configured\n", ptrNetfpProxyBridgedInterface->ifName);
        }
        return 0;
    }

    /* Is the bridge VLAN or not? */
    if (NetfpProxy_ifaceIsVLAN (ifName, NULL, NULL) == 1)
    {
        /* Case (c): Cycle through all the bridged port interfaces */
        for (index = 0; index < NETFP_PROXY_MAX_BRIDGE_IF; index++)
        {
            /* Is there a valid bridged interface name? */
            if (ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName[0] == 0)
                continue;

            /* Construct the bridged port name */
            NetfpProxy_constructBridgedPortName (ifName, ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName, &bridgedPortName[0]);

            /* Find the interface: */
            ptrNetfpProxyBridgedPortInterface = NetfpProxy_ifaceFindInterface (bridgedPortName);
            if (ptrNetfpProxyBridgedPortInterface == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Bridged Port Interface '%s' not found in database\n",
                                   ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName);
                return -1;
            }

            /* Copy over the VLAN Egress map configuration: */
            memcpy ((void *)&ptrNetfpProxyBridgedPortInterface->proxyIfCfg.ifConfig.vlanMap,
                    (void *)&ptrNetfpProxyInterface->proxyIfCfg.ifConfig.vlanMap,
                    sizeof(Netfp_VLANPriorityMap));

            /* Setup the new VLAN configuration: */
            optCfg.type   = Netfp_Option_VLAN_EGRESS_PRIORITY;
            optCfg.length = sizeof(Netfp_VLANPriorityMap);
            optCfg.value  = &ptrNetfpProxyBridgedPortInterface->proxyIfCfg.ifConfig.vlanMap;

            /* Modify the mapping: */
            if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyBridgedPortInterface->ifHandle, &optCfg, errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure VLAN Egress map for %s [Error code %d]\n",
                                   ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName, *errCode);
                NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
                return -1;
            }
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface '%s' VLAN egress configured\n", bridgedPortName);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the L3 Shaper on the specific interface
 *
 *  @param[in]  ifName
 *      Interface name.
 *  @param[in]  ptrL3QOSCfg
 *      Pointer to the L3 QOS Configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceConfigureL3Shaper(const char* ifName, Netfp_L3QoSCfg* ptrL3QOSCfg, int32_t* errCode)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    NetfpProxy_Interface*   ptrNetfpProxyBridgedInterface;
    uint32_t                index;
    Netfp_OptionTLV         optCfg;
    char                    bridgedPortName[NETFP_MAX_CHAR];

    /* Get the NETFP Proxy interface associated with the interface */
    ptrNetfpProxyInterface = NetfpProxy_ifaceFindInterface(ifName);
    if (ptrNetfpProxyInterface == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Interface '%s' has not been added to NETFP Proxy\n", ifName);
        return -1;
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: %s L3 Shaper on interface '%s'\n",
                       (ptrL3QOSCfg->isEnable == 1) ? "Enabling" : "Disabling", ifName);

    /* Setup the L3 QOS Shaper: */
    optCfg.type   = Netfp_Option_L3_QOS;
    optCfg.length = sizeof(Netfp_L3QoSCfg);
    optCfg.value  = (void*)ptrL3QOSCfg;

    /* We can now setup the L3 QOS shaper: */
    if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyInterface->ifHandle, &optCfg, errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure the L3 Shaper on '%s'\n", ifName);
        NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
        return -1;
    }

    memcpy(&ptrNetfpProxyInterface->l3QoSCfg, ptrL3QOSCfg, sizeof(ptrNetfpProxyInterface->l3QoSCfg));

    /* Is this a bridged interface? If so we need to configure the L3 shaper on all the bridged port interfaces */
    if (NetfpProxy_ifaceIsBridge (ifName) == 1)
    {
        /* YES. Cycle through all the bridged port interfaces */
        for (index = 0; index < NETFP_PROXY_MAX_BRIDGE_IF; index++)
        {
            /* Is there a valid bridged interface name? */
            if (ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName[0] == 0)
                continue;

            /* Construct the bridged port name */
            NetfpProxy_constructBridgedPortName (ifName, ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName, &bridgedPortName[0]);

            /* Find the interface: */
            ptrNetfpProxyBridgedInterface = NetfpProxy_ifaceFindInterface (bridgedPortName);
            if (ptrNetfpProxyBridgedInterface == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "FATAL Error: Bridged Interface '%s' not found in database\n",
                                   ptrNetfpProxyInterface->proxyIfCfg.bridgePortList[index].ifName);
                return -1;
            }

            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: %s L3 Shaper on interface '%s'\n",
                               (ptrL3QOSCfg->isEnable == 1) ? "Enabling" : "Disabling", bridgedPortName);

            /* Setup the L3 QOS Shaper: */
            optCfg.type   = Netfp_Option_L3_QOS;
            optCfg.length = sizeof(Netfp_L3QoSCfg);
            optCfg.value  = (void*)ptrL3QOSCfg;

            /* We can now setup the L3 QOS shaper: */
            if (Netfp_setIfOpt (gNetfpProxyMcb.netfpClientHandle, ptrNetfpProxyBridgedInterface->ifHandle, &optCfg, errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to configure the L3 Shaper on '%s'\n", ifName);
                NetfpProxy_assertCriticalError(*errCode, __func__, __LINE__); //fzm
                return -1;
            }

            memcpy(&ptrNetfpProxyBridgedInterface->l3QoSCfg, ptrL3QOSCfg, sizeof(ptrNetfpProxyBridgedInterface->l3QoSCfg));
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to poll the NETFP Proxy Interface module for
 *      neighbor events
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ifaceExecute (void)
{
    int32_t numMessages;

    /* Process the interface messages: */
    numMessages = netmgr_process_message (gIfMgmtMCB.netMgrIfHandle);
    if (numMessages < 0)
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to process the interface received message [Error code %d]\n", numMessages);
}

/**
 *  @b Description
 *  @n
 *      The function is used to dump the interface internals and is used for debugging.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_ifaceDump(void)
{
    NetfpProxy_Interface*   ptrNetfpProxyInterface;
    uint32_t                index;
    char                    strIPv6Address[256];
    char                    strIPv6SubnetMask[256];

    /* Display the Interfaces: */
//fzm - use NetfpProxy_dumpMsg to capture this with the USR1 signal
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Offloaded Interfaces\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");

    /* Cycle through all the registered interfaces */
    ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getHead ((List_Node**)&gIfMgmtMCB.ifaceList);
    while (ptrNetfpProxyInterface != NULL)
    {
        /* Display the interface: */
        NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "%p: %s MAC 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:%02x\n",
                           ptrNetfpProxyInterface, ptrNetfpProxyInterface->ifName, ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[0],
                           ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[1], ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[2],
                           ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[3], ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[4],
                           ptrNetfpProxyInterface->proxyIfCfg.ifConfig.macAddress[5]);

        /* Cycle through and display all the IP addresses assigned to the interface */
        for (index = 0; index < NETFP_PROXY_MAX_IP; index++)
        {
            if (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver == Netfp_IPVersion_IPV4)
            {
                /* Display the IPv4 Address: */
                NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    inet addr: %03d.%03d.%03d.%03d Mask:%03d.%03d.%03d.%03d\n",
                                   ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[0],
                                   ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[1],
                                   ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[2],
                                   ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv4.u.a8[3],
                                   ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv4.u.a8[0],
                                   ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv4.u.a8[1],
                                   ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv4.u.a8[2],
                                   ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv4.u.a8[3]);
            }
            else if (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].ver == Netfp_IPVersion_IPV6)
            {
                /* Convert the IPv6 addresses to string: */
                Netfp_convertIP6ToStr (ptrNetfpProxyInterface->proxyIfCfg.ipAddress[index].addr.ipv6,  &strIPv6Address[0]);
                Netfp_convertIP6ToStr (ptrNetfpProxyInterface->proxyIfCfg.subnetMask[index].addr.ipv6, &strIPv6SubnetMask[0]);

                /* Display the IPv6 Address: */
                NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    inet6 addr: %s Mask:%s\n", strIPv6Address, strIPv6SubnetMask);
            }
            else
            {
                /* No IP address detected: Skip this entry. */
            }
        }

        /* Display the Link status: */
        NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    %s       MTU:%d         Bridge: %s\n",
                           (ptrNetfpProxyInterface->bIsLinkUp == 1) ? "UP" : "DOWN",
                           ptrNetfpProxyInterface->proxyIfCfg.ifConfig.mtu,
                           NetfpProxy_ifaceIsBridge(ptrNetfpProxyInterface->ifName) ? "YES" : "NO");

        /* Display the VLAN idenfifier*/
        if (ptrNetfpProxyInterface->proxyIfCfg.ifConfig.type == Netfp_InterfaceType_VLAN)
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    VLAN Id: %d\n", ptrNetfpProxyInterface->proxyIfCfg.ifConfig.vlanId);

        /* Get the next interface */
        ptrNetfpProxyInterface = (NetfpProxy_Interface*)List_getNext ((List_Node*)ptrNetfpProxyInterface);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETFP Proxy interface module
 *
 *  @retval
 *      Success -   Socket descriptor associated with the interface module
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceInit(void)
{
    int32_t         errCode;

    /* Initialize the interface management MCB */
    memset ((void*)&gIfMgmtMCB, 0, sizeof(NetfpProxy_IfMgmtMCB));

    /* Initialize the NETMGR for the interface module. */
    gIfMgmtMCB.netMgrIfHandle = netmgr_init (NetMgr_CacheType_LINK, &errCode);
    if (gIfMgmtMCB.netMgrIfHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Initialization of the Interface NETMGR failed [Error code %d]\n", errCode);
        return -1;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Interface NETMGR Initialized\n");

    /* Return the socket file descriptor associated with the Interface NETMGR */
    return netmgr_getSocket(gIfMgmtMCB.netMgrIfHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the NETFP Proxy interface module
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_ifaceDeinit(void)
{
    return 0;
}

