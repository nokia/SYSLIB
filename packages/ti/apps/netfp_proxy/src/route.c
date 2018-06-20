/**
 *   @file  route.c
 *
 *   @brief
 *      NETFP Proxy Route Implementation
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
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <dirent.h>
#include <errno.h> //fzm

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

/**********************************************************************
 ************************ Local Structures ****************************
 **********************************************************************/

/**
 * @brief
 *  NETFP Proxy Pending Request
 *
 * @details
 *  The NETFP Server can send multiple route resolution requests to
 *  the PROXY for route resolution. Each of the resolution requests
 *  map to the same next HOP MAC address but each have a different
 *  route resolution identifier.
 */
typedef struct NetfpProxy_PendingRequest
{
    /**
     * @brief   Link to other pending requests
     */
    List_Node               links;

    /**
     * @brief   Route Resolution Identifier
     */
    uint32_t                routeResolutionId;
}NetfpProxy_PendingRequest;

/**
 * @brief
 *  NETFP Proxy Route Info
 *
 * @details
 *  The structure holds information about all the routes which have been
 *  instantiated in the NETFP Proxy.
 */
typedef struct NetfpProxy_Route
{
    /**
     * @brief   Link to other routes
     */
    List_Node                       links;

    /**
     * @brief   Reference count
     */
    uint32_t                        refCount;

    /**
     * @brief   Destination IP for the route
     */
    struct nl_addr*                 dstIP;

    /**
     * @brief   Source IP used for the route resolution
     */
    struct nl_addr*                 srcIP;

    /**
     * @brief   Next Hop IP address for the route
     */
    struct nl_addr*                 nhIP;

    /**
     * @brief   Next Hop neighbor entry for the route
     */
    struct rtnl_neigh*              nhNeigh;

    /**
     * @brief   Link info for this route
     */
    char                            ifName[NETFP_MAX_CHAR];

    /**
     * @brief   Pointer to the pending route list of route requests
     * which have been received by the NETFP Server but have still not
     * been replied for.
     */
    NetfpProxy_PendingRequest*      ptrPendingRequestList;

    /**
     * @brief   Next HOP mac address
     */
    uint8_t                         nextHopMACAddress[6];
}NetfpProxy_Route;

/**
 * @brief
 *  Route Management MCB
 *
 * @details
 *  The structures is the Route Management MCB.
 */
typedef struct NetfpProxy_RouteMgmtMCB
{
    /**
     * @brief   NETMGR Route module
     */
    NetfpProxy_NetMgrHandle     netMgrRouteHandle;

    /**
     * @brief   Route Flush command in progress.
     */
    uint32_t                    isFlushPending;

    /**
     * @brief   Pointer to the route cache
     */
    NetfpProxy_Route**          ptrRouteCacheList;
}NetfpProxy_RouteMgmtMCB;

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

/* Route MCB: */
NetfpProxy_RouteMgmtMCB     gRouteMgmtMCB;

//fzm->>
static int nl_socket = -1;
//<<-fzm

/**********************************************************************
 ************************* Route Functions ****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to determine if the route is currently being tracked or not.
 *      The function is invoked after receiving an update from the neighbor thread. This
 *      is done to ensure the sanity between the neighbor & core threads
 *
 *  @param[in]  ptrProxyRoute
 *      Pointer to the proxy route
 *
 *  @retval
 *      1   -   Entry is valid & is being tracked by PROXY
 *  @retval
 *      0   -   Entry is not being tracked by PROXY
 */
static int32_t NetfpProxy_isRouteTracked(NetfpProxy_Route* ptrProxyRoute)
{
    NetfpProxy_Route*   ptrRoute;

    /* Cycle through the route cache */
    ptrRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrRoute != NULL)
    {
        /* Do we have a match? */
        if (ptrRoute == ptrProxyRoute)
            return 1;

        /* Get the next route cache element */
        ptrRoute = (NetfpProxy_Route*)List_getNext((List_Node*)ptrRoute);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a route which has already been cached
 *      in the NETFP Proxy.
 *
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the PROXY Server information block
 *
 *  @retval
 *      NULL        - No cached entry found
 *  @retval
 *      Not NULL    - Cached entry found
 */
static NetfpProxy_Route* NetfpProxy_findRoute(Netfp_ProxyServerInfo* ptrProxyServerInfo)
{
    NetfpProxy_Route*   ptrProxyRoute;

    /* Cycle through all the routes */
    ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        /* Does the entry belong to the IPv4 family? */
        if ((ptrProxyServerInfo->dstIP.ver == Netfp_IPVersion_IPV4) && (nl_addr_get_family(ptrProxyRoute->dstIP) == AF_INET) &&
            (ptrProxyServerInfo->srcIP.ver == Netfp_IPVersion_IPV4) && (nl_addr_get_family(ptrProxyRoute->srcIP) == AF_INET))
        {
            /* YES: Compare the IPv4 address */
            if ((memcmp ((void *)nl_addr_get_binary_addr(ptrProxyRoute->dstIP), (void*)&ptrProxyServerInfo->dstIP.addr.ipv4.u.a8[0], 4) == 0) &&
                (memcmp ((void *)nl_addr_get_binary_addr(ptrProxyRoute->srcIP), (void*)&ptrProxyServerInfo->srcIP.addr.ipv4.u.a8[0], 4) == 0))
                return ptrProxyRoute;
        }

        /* Does the entry belong to the IPv6 family? */
        if ((ptrProxyServerInfo->dstIP.ver == Netfp_IPVersion_IPV6) && (nl_addr_get_family(ptrProxyRoute->dstIP) == AF_INET6) &&
            (ptrProxyServerInfo->srcIP.ver == Netfp_IPVersion_IPV6) && (nl_addr_get_family(ptrProxyRoute->srcIP) == AF_INET6))
        {
            /* YES: Compare the IPv6 address */
            if((memcmp ((void *)nl_addr_get_binary_addr(ptrProxyRoute->dstIP), (void*)&ptrProxyServerInfo->dstIP.addr.ipv6.u.a8[0], 16) == 0) &&
               (memcmp ((void *)nl_addr_get_binary_addr(ptrProxyRoute->srcIP), (void*)&ptrProxyServerInfo->srcIP.addr.ipv6.u.a8[0], 16) == 0))
                return ptrProxyRoute;
        }

        /* Get the next route */
        ptrProxyRoute = (NetfpProxy_Route*)List_getNext ((List_Node*)ptrProxyRoute);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a cached route.
 *
 *  @param[in]  ptrProxyRoute
 *      Pointer to the NETFP Proxy cached route to be deleted
 *
 *  @retval
 *      Not applicabe
 */
static void NetfpProxy_deleteRoute (NetfpProxy_Route* ptrProxyRoute)
{
    char                        strDstIP[256];
    char                        strSrcIP[256];
    NetfpProxy_PendingRequest*  ptrPendingRequest;
    int32_t                     errCode;
    Netfp_ProxyServerInfo*      ptrProxyServerInfo;
    static Netfp_ProxyServerBulkMsg proxyServerBulkMsg; //this local variable is so big so keep it as static

    /* Get the display strings: */
    nl_addr2str (ptrProxyRoute->dstIP, strDstIP, sizeof (strDstIP));
    nl_addr2str (ptrProxyRoute->srcIP, strSrcIP, sizeof (strSrcIP));

    /* Are there any references held? */
    if (ptrProxyRoute->refCount > 1)
    {
        /* YES. Drop a reference. */
        ptrProxyRoute->refCount--;

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Skipping delete route cache [%p] %s -> %s [Count %d]\n",
                           ptrProxyRoute, strSrcIP, strDstIP, ptrProxyRoute->refCount);
        return;
    }

    /* We cannot delete a route which has a pending requests because we have not yet acknowledged the server. We
     * need to respond back to the NETFP Server else we will leak JOSH Jobs since the server will forever wait
     * for the response to be received. */

    proxyServerBulkMsg.numberOfEntries = 0;

    ptrPendingRequest = (NetfpProxy_PendingRequest*)List_removeHead ((List_Node**)&ptrProxyRoute->ptrPendingRequestList);
    while (ptrPendingRequest != NULL)
    {
        ptrProxyServerInfo = &proxyServerBulkMsg.proxyServerInfo[proxyServerBulkMsg.numberOfEntries];

        ptrProxyServerInfo->requestId = ptrPendingRequest->routeResolutionId;

        ptrProxyServerInfo->opType = Netfp_ProxyServerOp_RESPONSE;
        memset ((void*)&ptrProxyServerInfo->u.resp.nextHopMacAddress[0], 0, 6);
        ptrProxyServerInfo->u.resp.ifHandle = NULL;

        proxyServerBulkMsg.numberOfEntries++;

        /* Cleanup the memory for the pending request */
        free (ptrPendingRequest);

        /* Get the next pending request: */
        ptrPendingRequest = (NetfpProxy_PendingRequest*)List_removeHead ((List_Node**)&ptrProxyRoute->ptrPendingRequestList);

        /* Send response when we do not have more pending requests or reach out the limit for the bulk message */
        if ((ptrPendingRequest == NULL) || (proxyServerBulkMsg.numberOfEntries == BULK_INFO_MAX_ENTRIES))
        {
            /* Send the response back to the server. We simply indicate that the neighbor is not reachable; we are anyway
             * not monitoring the route anymore. */
            if (Netfp_sendProxyResponse (gNetfpProxyMcb.netfpClientHandle, &proxyServerBulkMsg, &errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Sending Proxy response failed, entries %u [Error: %d]\n", proxyServerBulkMsg.numberOfEntries, errCode);

                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
            }

            proxyServerBulkMsg.numberOfEntries = 0;
        }
    }

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Deleting route cache [%p] %s -> %s\n", ptrProxyRoute, strSrcIP, strDstIP);

    /* Free any allocated handles */
    if (ptrProxyRoute->dstIP)
        nl_addr_put (ptrProxyRoute->dstIP);
    if (ptrProxyRoute->srcIP)
        nl_addr_put (ptrProxyRoute->srcIP);
    if (ptrProxyRoute->nhIP)
        nl_addr_put (ptrProxyRoute->nhIP);
    if (ptrProxyRoute->nhNeigh)
        rtnl_neigh_put (ptrProxyRoute->nhNeigh);

    /* Remove from the list */
    List_removeNode ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList, (List_Node*)ptrProxyRoute);

    /* Clean up the memory */
    free(ptrProxyRoute);
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the route lookup procedure after it has been instantiated
 *      by a request from the NETFP Server.
 *
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the PROXY Server information block
 *
 *  @retval
 *      Success -   Pointer to the cached route
 *  @retval
 *      Error   -   NULL
 */
static NetfpProxy_Route* NetfpProxy_startRouteLookup(Netfp_ProxyServerInfo* ptrProxyServerInfo)
{
    int32_t             tableNo;
    char                strDstIP[256];
    char                strSrcIP[256];
    Netfp_IPAddr        nextHopIPAddress;
    int32_t             status;
    NetfpProxy_Route*   ptrProxyRoute;

    /* Are we already monitoring the route? */
    ptrProxyRoute = NetfpProxy_findRoute (ptrProxyServerInfo);
    if (ptrProxyRoute != NULL)
    {
        /* Debug message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Reusing route cache [%p]\n", ptrProxyRoute);

        /* Increment the reference counter */
        ptrProxyRoute->refCount++;

        /* Initialize the next hop IP Address for which we resolve the route: */
        if (nl_addr_get_family (ptrProxyRoute->nhIP) == AF_INET)
        {
            /* IPv4 Address: */
            nextHopIPAddress.ver = Netfp_IPVersion_IPV4;
            memcpy ((void*)&nextHopIPAddress.addr.ipv4.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 4);
        }
        else
        {
            /* IPv6 Address: */
            nextHopIPAddress.ver = Netfp_IPVersion_IPV6;
            memcpy ((void*)&nextHopIPAddress.addr.ipv6.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 16);
        }

        /* Initiate the next hop MAC address lookup: */
        if (NetfpProxy_neighLookup (&ptrProxyRoute->ifName[0], &nextHopIPAddress, (uint32_t)ptrProxyRoute) < 0)
        {
            /* Error: Unable to get the next hop MAC address */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Unable to perform the lookup for the next hop MAC address\n");
            return NULL;
        }
        return ptrProxyRoute;
    }

    /* Allocate memory for the new route: */
    ptrProxyRoute = (NetfpProxy_Route*)malloc(sizeof(NetfpProxy_Route));
    if (ptrProxyRoute == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Unable to allocate memory for the proxy route\n");
        return NULL;
    }

    /* Initialize the allocated memory: */
    memset ((void*)ptrProxyRoute, 0, sizeof(NetfpProxy_Route));

    /* Initialize the reference counter */
    ptrProxyRoute->refCount = 1;

    /* Are we performing an IPv4 or IPv6 route lookup? */
    if (ptrProxyServerInfo->dstIP.ver == Netfp_IPVersion_IPV4)
    {
        /* IPv4 Route Lookup: */
        ptrProxyRoute->dstIP = nl_addr_build (AF_INET, ptrProxyServerInfo->dstIP.addr.ipv4.u.a8, 4);
        ptrProxyRoute->srcIP = nl_addr_build (AF_INET, ptrProxyServerInfo->srcIP.addr.ipv4.u.a8, 4);
    }
    else
    {
        /* IPv6 Route Lookup: */
        ptrProxyRoute->dstIP = nl_addr_build (AF_INET6, ptrProxyServerInfo->dstIP.addr.ipv6.u.a8, 16);
        ptrProxyRoute->srcIP = nl_addr_build (AF_INET6, ptrProxyServerInfo->srcIP.addr.ipv6.u.a8, 16);
    }

    /* Debug Message:  */
    nl_addr2str (ptrProxyRoute->dstIP, strDstIP, sizeof (strDstIP));
    nl_addr2str (ptrProxyRoute->srcIP, strSrcIP, sizeof (strSrcIP));
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Created route cache [%p] %s -> %s\n", ptrProxyRoute, strSrcIP, strDstIP);

    /* Add the route to the list: */
    List_addNodeTail ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList, (List_Node*)ptrProxyRoute);

    /* Find the route to reach destination: */
    status = netmgr_find_route (gRouteMgmtMCB.netMgrRouteHandle, ptrProxyRoute->dstIP, ptrProxyRoute->srcIP,
                                &ptrProxyRoute->nhIP, &ptrProxyRoute->ifName[0], &tableNo);
    if (status < 0)
    {
        /* Error: Unable to resolve the route */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Unable to find the route for %s -> %s\n", strSrcIP, strDstIP);
        NetfpProxy_deleteRoute (ptrProxyRoute);
        return NULL;
    }

    /* Route has been found: We need to find the NEXT Hop MAC address */
    if (nl_addr_get_family (ptrProxyRoute->nhIP) == AF_INET)
    {
        /* IPv4 Address: */
        nextHopIPAddress.ver = Netfp_IPVersion_IPV4;
        memcpy ((void*)&nextHopIPAddress.addr.ipv4.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 4);
    }
    else
    {
        /* IPv6 Address: */
        nextHopIPAddress.ver = Netfp_IPVersion_IPV6;
        memcpy ((void*)&nextHopIPAddress.addr.ipv6.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 16);
    }

    /* Initiate the next hop MAC address lookup: */
    if (NetfpProxy_neighLookup (&ptrProxyRoute->ifName[0], &nextHopIPAddress, (uint32_t)ptrProxyRoute) < 0)
    {
        /* Error: Unable to get the next hop MAC address */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Error: Unable to perform the lookup for the next hop MAC address\n");
        NetfpProxy_deleteRoute (ptrProxyRoute);
        return NULL;
    }

    /* The route request has been submitted to get the next hop MAC address: */
    return ptrProxyRoute;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the route lookup procedure
 *
 *  @param[in]  ptrProxyServerInfo
 *      Pointer to the PROXY Server information block
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t NetfpProxy_stopRouteLookup(Netfp_ProxyServerInfo* ptrProxyServerInfo)
{
    NetfpProxy_Route*   ptrProxyRoute;
    Netfp_IPAddr        nextHopIPAddress;

    /* Are we already monitoring the route? */
    ptrProxyRoute = NetfpProxy_findRoute (ptrProxyServerInfo);
    if (ptrProxyRoute == NULL)
        return 0;

    /* Get the next hop IP Address: */
    if (nl_addr_get_family (ptrProxyRoute->nhIP) == AF_INET)
    {
        /* IPv4 Address: */
        nextHopIPAddress.ver = Netfp_IPVersion_IPV4;
        memcpy ((void*)&nextHopIPAddress.addr.ipv4.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 4);
    }
    else
    {
        /* IPv6 Address: */
        nextHopIPAddress.ver = Netfp_IPVersion_IPV6;
        memcpy ((void*)&nextHopIPAddress.addr.ipv6.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 16);
    }

    /* Stop the Neighbor monitoring: This is required only if there are no more references held. */
    if (ptrProxyRoute->refCount == 1)
        NetfpProxy_neighStopLookup(&ptrProxyRoute->ifName[0], &nextHopIPAddress, (uint32_t)ptrProxyRoute);

    /* Delete the route cache entry: The reference counters will keep it alive if required */
    NetfpProxy_deleteRoute (ptrProxyRoute);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to poll the route module.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_routePoll (void)
{
    /***********************************************************************************************
     * Do we have a pending route flush which we need to address? This can happen under the
     * following conditions:
     * 1) Pending resolve route requests
     *    We have responsed to the pending route requests. The NETFP Server will handle the route
     *    response and will then be notified about the route flush.
     * 2) NETFP Server Route recomputation in progress
     *    This is too handle the case when multiple flush commands from the OAM IPC interface are
     *    sent immediately one after another. The NETFP Server is still recomputing routes in response
     *    to the first flush command. So we wait until the NETFP Server is completely done. We then
     *    restart the flush again.
     ***********************************************************************************************/
    if (gRouteMgmtMCB.isFlushPending == 1)
        NetfpProxy_routeFlushCache ();
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the route lookup.
 *
 *  @param[in]  ptrProxyServerBulkInfo
 *      Pointer to the bulk message
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_routeLookup(Netfp_ProxyServerBulkMsg* ptrProxyServerBulkInfo)
{
    NetfpProxy_PendingRequest*  ptrPendingRequest;
    NetfpProxy_Route*           ptrProxyRoute;
    int32_t                     errCode;
    static Netfp_ProxyServerBulkMsg response;

    //only initialize number of entries to not allocate all data in big buffer
    response.numberOfEntries = 0;

    for (int i = 0 ; i < ptrProxyServerBulkInfo->numberOfEntries ; i++)
    {
        /* Do we need to start/stop the route lookup? */
        if (ptrProxyServerBulkInfo->proxyServerInfo[i].u.req.startMonitor == 0)
        {
            /* Stop the route lookup: Delete the route & notify the neighbor module. */
            if (NetfpProxy_stopRouteLookup (&ptrProxyServerBulkInfo->proxyServerInfo[i]) < 0)
                return -1;

            response.proxyServerInfo[response.numberOfEntries].opType = Netfp_ProxyServerOp_RESPONSE;
            memset ((void*)&response.proxyServerInfo[response.numberOfEntries].u.resp.nextHopMacAddress[0], 0, 6);
            response.proxyServerInfo[response.numberOfEntries].u.resp.ifHandle = NULL;
            response.proxyServerInfo[response.numberOfEntries].requestId = ptrProxyServerBulkInfo->proxyServerInfo[i].requestId;

            response.numberOfEntries++;

            continue;
        }

        /* Start the route lookup: */
        ptrProxyRoute = NetfpProxy_startRouteLookup (&ptrProxyServerBulkInfo->proxyServerInfo[i]);
        if (ptrProxyRoute == NULL)
        {
            response.proxyServerInfo[response.numberOfEntries].opType = Netfp_ProxyServerOp_RESPONSE;
            memset ((void*)&response.proxyServerInfo[response.numberOfEntries].u.resp.nextHopMacAddress[0], 0, 6);
            response.proxyServerInfo[response.numberOfEntries].u.resp.ifHandle = NULL;
            response.proxyServerInfo[response.numberOfEntries].requestId = ptrProxyServerBulkInfo->proxyServerInfo[i].requestId;

            response.numberOfEntries++;

            continue;
        }
        else
        {
            /***************************************************************************************
             * Start the route lookup:
             * - Setup a pending request node which will track the route resolution identifier and
             *   this is used to send back the response to the NETFP Server.
             ***************************************************************************************/
            ptrPendingRequest = (NetfpProxy_PendingRequest*)malloc (sizeof(NetfpProxy_PendingRequest));
            if (ptrPendingRequest == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to allocate memory for the pending request\n");
                return -1;
            }

            /* Initialize the allocated memory */
            memset ((void*)ptrPendingRequest, 0, sizeof(NetfpProxy_PendingRequest));

            /* Track the route resolution identifer: */
            ptrPendingRequest->routeResolutionId = ptrProxyServerBulkInfo->proxyServerInfo[i].requestId;

            /* Add the pending request to the proxy route: */
            List_addNodeTail ((List_Node**)&ptrProxyRoute->ptrPendingRequestList, (List_Node*)ptrPendingRequest);

            /* Debug Message: */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB,"VRB: Created pending request %p for Id: 0x%x \n",
                               ptrPendingRequest, ptrPendingRequest->routeResolutionId); //fzm
        }
    }

    if (response.numberOfEntries)
    {
        if (Netfp_sendProxyResponse (gNetfpProxyMcb.netfpClientHandle, &response, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Sending Proxy response failed, entries %u [Error: %d]\n", response.numberOfEntries, errCode);

            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to flush and kill routes which are associated with a specific
 *      interface.
 *
 *  @retval
 *      Not applicable
 */
int32_t NetfpProxy_routeKill(const char* ifName)
{
    NetfpProxy_Route*   ptrProxyRoute;
    Netfp_IPAddr        nextHopIPAddress;

    /* Cycle through all the cache routes */
    ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        /* Do we need to kill the route? */
        if (strncmp (&ptrProxyRoute->ifName[0], ifName, NETFP_MAX_CHAR) == 0)
        {
            /* YES: The route is residing on the interface and it needs to be killed. Get the next hop IP Address: */
            if (nl_addr_get_family (ptrProxyRoute->nhIP) == AF_INET)
            {
                /* IPv4 Address: */
                nextHopIPAddress.ver = Netfp_IPVersion_IPV4;
                memcpy ((void*)&nextHopIPAddress.addr.ipv4.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 4);
            }
            else
            {
                /* IPv6 Address: */
                nextHopIPAddress.ver = Netfp_IPVersion_IPV6;
                memcpy ((void*)&nextHopIPAddress.addr.ipv6.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 16);
            }

            /* Inform the Neighbor module to stop monitoring the next hop IP address: */
            NetfpProxy_neighStopLookup(&ptrProxyRoute->ifName[0], &nextHopIPAddress, (uint32_t)ptrProxyRoute);

            /* Delete the route: */
            NetfpProxy_deleteRoute (ptrProxyRoute);

            /* Restart from the head of the list: */
            ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
        }
        else
        {
            /* Get the next route: This route should not be killed. */
            ptrProxyRoute = (NetfpProxy_Route*)List_getNext ((List_Node*)ptrProxyRoute);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to flush the route and neighbor cache. This is invoked on the reception
 *      of an IPC command from the OAM application to indicate that the routing tables have changed.
 *      There are two parts involved in flushing the routes.
 *          (1) Flush and cleanup the local routes & neighbors
 *          (2) Inform the NETFP Server that all the fast paths & SA need to be recomputed.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_routeFlushCache (void)
{
    NetfpProxy_Route*   ptrProxyRoute;
    int32_t             errCode;
    Netfp_IPAddr        nextHopIPAddress;

    /* Set the flag to indicate that a route flush has been initiated */
    gRouteMgmtMCB.isFlushPending = 1;

    /* Inform the NETFP Server to perform the route recomputation? */
    if (Netfp_recomputeRoutes (gNetfpProxyMcb.netfpClientHandle, &errCode) < 0)
    {
        /* Error: Route recomputation failed: This could imply that there are already pending jobs which need
         * to be handled. In this case we postpone the route flush operation. We will defer it for the next poll  */
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
        if (errCode == NETFP_EINUSE)
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Server previous route recomputation in progress.\n");
        else
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NETFP Server route recomputation failed [Error code: %d]\n", errCode);
        return;
    }

    /* Route flush has been successfully initiated on the NETFP Server. The server will start sending requests to the
     * PROXY and these messages are still queued up in the core thread. We can now flush out our routes. */
    gRouteMgmtMCB.isFlushPending = 0;

    /* Debug Message: */
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: NETFP Proxy flushing the route cache\n");

    /* Flush the cache */
    if (netmgr_route_cache_flush(gRouteMgmtMCB.netMgrRouteHandle, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NetMgr route refill failed [Error code: %d]\n", errCode);
        return;
    }

    /* Cycle through all the routes and delete them. */
    ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        /* We need to drop all references which are held on the route since everything is going down */
        ptrProxyRoute->refCount = 1;

        /* Get the next hop IP Address: */
        if (nl_addr_get_family (ptrProxyRoute->nhIP) == AF_INET)
        {
            /* IPv4 Address: */
            nextHopIPAddress.ver = Netfp_IPVersion_IPV4;
            memcpy ((void*)&nextHopIPAddress.addr.ipv4.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 4);
        }
        else
        {
            /* IPv6 Address: */
            nextHopIPAddress.ver = Netfp_IPVersion_IPV6;
            memcpy ((void*)&nextHopIPAddress.addr.ipv6.u.a8[0], (void*)nl_addr_get_binary_addr(ptrProxyRoute->nhIP), 16);
        }

        /* Stop the Neighbor monitoring for the next hop IP address */
        NetfpProxy_neighStopLookup(&ptrProxyRoute->ifName[0], &nextHopIPAddress, (uint32_t)ptrProxyRoute);

        /* Delete the route */
        NetfpProxy_deleteRoute(ptrProxyRoute);

        /* Restart from the head of the list: */
        ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    }
    return;
}

//fzm->>
int NetfpProxy_delNgh(char *if_name, Netfp_IPAddr *ptrIPAddress)
{
    struct {
        struct nlmsghdr nlh;
        struct ndmsg ndh;
        uint8_t attrbuf[32];
    } req;
    struct rtattr *rtap;
    int msg_len = 0;

    memset(&req, 0, sizeof(req));

    req.nlh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ndmsg));
    req.nlh.nlmsg_type = RTM_DELNEIGH;
    req.nlh.nlmsg_flags = NLM_F_REQUEST;

    req.ndh.ndm_ifindex = if_nametoindex(if_name);
    req.ndh.ndm_state = NUD_NONE;
    req.ndh.ndm_flags = NTF_USE;

    rtap = (struct rtattr *)(((char *)NLMSG_DATA(&req)) + NLMSG_ALIGN(sizeof(struct ndmsg)));

    rtap->rta_type = NDA_DST;

    if (Netfp_IPVersion_IPV4 == ptrIPAddress->ver)
    {
        req.ndh.ndm_family = AF_INET;
        rtap->rta_len = RTA_LENGTH(sizeof(ptrIPAddress->addr.ipv4.u.a32));
        memcpy((void *)RTA_DATA(rtap), (void *)&ptrIPAddress->addr.ipv4.u.a32, RTA_PAYLOAD(rtap));
    }
    else
    {
        req.ndh.ndm_family = AF_INET6;
        rtap->rta_len = RTA_LENGTH(sizeof(ptrIPAddress->addr.ipv6.u.a32));
        memcpy((void *)RTA_DATA(rtap), (void *)&ptrIPAddress->addr.ipv6.u.a32, RTA_PAYLOAD(rtap));
    }
    req.nlh.nlmsg_len = RTA_ALIGN(req.nlh.nlmsg_len) + rtap->rta_len;

    msg_len = send(nl_socket, &req, req.nlh.nlmsg_len, 0);
    if (msg_len < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,
                           "%s: Failed to send the message error %s (%d)\n",
                           __func__,
                           strerror(errno),
                           errno);
    }

    return msg_len;
}
//<<-fzm

/**
 *  @b Description
 *  @n
 *      The function is used to update the route for a specific IP address with the
 *      MAC address.
 *
 *  @param[in]  listenerId
 *      Listener Identifier which the route module had initiated
 *  @param[in]  ptrIPAddress
 *      Pointer to the IP Address
 *  @param[in]  ptrMACAddress
 *      Pointer to the next hop MAC address
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_routeUpdate
(
    uint32_t        listenerId,
    Netfp_IPAddr*   ptrIPAddress,
    uint8_t*        ptrMACAddress
)
{
    NetfpProxy_PendingRequest*  ptrPendingRequest;
    Netfp_IfHandle              ifHandle;
    NetfpProxy_Route*           ptrProxyRoute;
    Netfp_ProxyServerInfo       proxyServerInfo;
    uint32_t                    numUpdates;
    int32_t                     errCode;
    uint8_t                     zeroMACAddress[6] = { 0x0 };
    uint8_t                     oldNextHopMacAddress[6];
    static Netfp_ProxyServerBulkMsg proxyServerBulkMsg;

    memset(oldNextHopMacAddress, 0, sizeof(oldNextHopMacAddress)); //fzm

    /* Get the cached route information: */
    ptrProxyRoute = (NetfpProxy_Route*)listenerId;
    if (ptrProxyRoute == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Neigh module is passing the listener id as NULL\n");
        return;
    }

    /* Cycle through all the routes which exist in the NETFP Proxy. We could have received an update
     * from the neighbor for a listener which had already been deleted. Even though we had told the
     * neighbor to stop monitioring there is still a small race condition where the neighbor update
     * is detected before the neighbor is stopped. */
    if (NetfpProxy_isRouteTracked(ptrProxyRoute) == 0)
        return;

    /* Keep a copy of the older resolved MAC address: */
    memcpy ((void *)&oldNextHopMacAddress[0], (void *)&ptrProxyRoute->nextHopMACAddress[0], 6);

    /* Update the cached route information: */
    memcpy ((void*)&ptrProxyRoute->nextHopMACAddress[0], (void *)ptrMACAddress, 6);

    /* Did we get a resolution? */
    if (memcmp ((void *)&ptrProxyRoute->nextHopMACAddress[0],(void*)&zeroMACAddress[0], 6) != 0)
    {
        /* YES. Entry was resolved; offload the interface if not already done so. */
        if (NetfpProxy_ifaceOffloadInterface (&ptrProxyRoute->ifName[0], &ptrProxyRoute->nextHopMACAddress[0], &ifHandle) < 0)
        {
            /* Error: Unable to offload the interface. Log the message */
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Unable to offload the interface '%s'\n", ptrProxyRoute->ifName);

//fzm-->
            if (NetfpProxy_delNgh(&ptrProxyRoute->ifName[0], ptrIPAddress) < 0)
            {
                /* Error: Unable to delete neighbor */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to delete the neighbor '%s'\n", ptrProxyRoute->ifName);
            }
//fzm<--

            ifHandle = NULL;
        }
    }
    else
    {
        /* NO. Entry was not resolved. */
        ifHandle = NULL;
    }

    /* Initialize the number of update messages sent back to the NETFP Server: */
    numUpdates = 0;

    /* Initialize the proxy server response: */
    memset ((void*)&proxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

    /* Populate the Proxy Server response: */
    proxyServerInfo.opType          = Netfp_ProxyServerOp_RESPONSE;
    proxyServerInfo.u.resp.ifHandle = ifHandle;
    memcpy ((void*)&proxyServerInfo.u.resp.nextHopMacAddress[0], (void *)ptrMACAddress, 6);
    if (ptrIPAddress->ver == Netfp_IPVersion_IPV4)
    {
        proxyServerInfo.dstIP.ver = Netfp_IPVersion_IPV4;
        proxyServerInfo.srcIP.ver = Netfp_IPVersion_IPV4;
        memcpy ((void *)&proxyServerInfo.dstIP.addr.ipv4.u.a8[0], (void *)nl_addr_get_binary_addr(ptrProxyRoute->dstIP), 4);
        memcpy ((void *)&proxyServerInfo.srcIP.addr.ipv4.u.a8[0], (void *)nl_addr_get_binary_addr(ptrProxyRoute->srcIP), 4);
    }
    if (ptrIPAddress->ver == Netfp_IPVersion_IPV6)
    {
        proxyServerInfo.dstIP.ver = Netfp_IPVersion_IPV6;
        proxyServerInfo.srcIP.ver = Netfp_IPVersion_IPV6;
        memcpy ((void *)&proxyServerInfo.dstIP.addr.ipv6.u.a8[0], (void *)nl_addr_get_binary_addr(ptrProxyRoute->dstIP), 16);
        memcpy ((void *)&proxyServerInfo.srcIP.addr.ipv6.u.a8[0], (void *)nl_addr_get_binary_addr(ptrProxyRoute->srcIP), 16);
    }

    /*********************************************************************************************
     * Route Entry has been updated: This could be because of the following reasons:
     *  (a) Neighbor Lookup request had been submitted
     *  (b) Asynchronous neighbor update notification has been received.
     *
     * Case (a): If there are any pending requests then we need to send back the response to the
     * NETFP Server.
     *********************************************************************************************/
    proxyServerBulkMsg.numberOfEntries = 0;

    ptrPendingRequest = (NetfpProxy_PendingRequest*)List_removeHead ((List_Node**)&ptrProxyRoute->ptrPendingRequestList);
    while (ptrPendingRequest != NULL)
    {
        proxyServerInfo.requestId = ptrPendingRequest->routeResolutionId;

        proxyServerBulkMsg.proxyServerInfo[proxyServerBulkMsg.numberOfEntries++] = proxyServerInfo;

        /* Increment the number of update messages sent out: */
        numUpdates++;

        /* Debug Message: */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,"Debug: Processed pending request %p for Id: 0x%x \n",
                           ptrPendingRequest, ptrPendingRequest->routeResolutionId);

        /* Cleanup the pending request entry: We are done with it. */
        free (ptrPendingRequest);

        /* Restart the search: */
        ptrPendingRequest = (NetfpProxy_PendingRequest*)List_removeHead ((List_Node**)&ptrProxyRoute->ptrPendingRequestList);

        if ((ptrPendingRequest == NULL) || (proxyServerBulkMsg.numberOfEntries == BULK_INFO_MAX_ENTRIES))
        {
            /* Pending request is available: */
            if (Netfp_sendProxyResponse (gNetfpProxyMcb.netfpClientHandle, &proxyServerBulkMsg, &errCode) < 0)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Sending Proxy response failed, entries %u [Error: %d]\n",
                                                           proxyServerBulkMsg.numberOfEntries, errCode);

                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
            }

            proxyServerBulkMsg.numberOfEntries = 0;
        }
    }

    /****************************************************************************************************************
     * Do we need to send an async update also to handle case(b) above?
     * There are two cases here which need to be addressed:
     *  1) There are 2 users for the route (ref count). We sent the response for the User(1). User(2) needs to be
     *     notified asynchronously.
     *  2) There is only 1 user (ref count) for this route. It had sent a pending request; which has been responsed
     *     There is no other user for this route and so there is no need to send the update.
     *****************************************************************************************************************/
    if (numUpdates != ptrProxyRoute->refCount)
    {
        /* Populate the proxy neighbor configuration: */
        proxyServerInfo.opType                 = Netfp_ProxyServerOp_UPDATE_NEIGH;
        proxyServerInfo.u.updateNeigh.ifHandle = ifHandle;
        memcpy ((void*)&proxyServerInfo.u.updateNeigh.oldMacAddress[0], (void *)&oldNextHopMacAddress[0], 6);
        memcpy ((void*)&proxyServerInfo.u.updateNeigh.nextHopMacAddress[0], (void *)ptrMACAddress, 6);

        /* Case (1): Send the asynchronous update to the NETFP Server */
        if (Netfp_asyncUpdate (gNetfpProxyMcb.netfpClientHandle, &proxyServerInfo, &errCode) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Update Neighbor status failed [Error code: %d]\n", errCode);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__); //fzm
        }
        else
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route %p has been updated in the NETFP Server\n", ptrProxyRoute);
        }
    }
    else
    {
        /* Case (2): Skip sending the update since we had already sent the proxy response */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Skipping async route [%p:%d] update\n", ptrProxyRoute, ptrProxyRoute->refCount);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the messages received on the ROUTE
 *      NETMGR module
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_routeExecute (void)
{
    int32_t numMessages;

    /* Process the route messages: */
    numMessages = netmgr_process_message (gRouteMgmtMCB.netMgrRouteHandle);
    if (numMessages < 0)
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unable to process the route received message [Error code %d]\n", numMessages);
}

/**
 *  @b Description
 *  @n
 *      The function is used to dump the route internals. This is used
 *      for debugging and is available to the PROXY core.
 *
 *  @retval
 *      Not applicable
 */
void NetfpProxy_routeDump (void)
{
    NetfpProxy_PendingRequest*  ptrPendingRequest;
    NetfpProxy_Route*           ptrProxyRoute;
    char                        srcIPString[256];
    char                        dstIPString[256];

//fzm - use NetfpProxy_dumpMsg to capture this with the USR1 signal
    /* Display the route cache: */
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Route Cache\n");
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "*******************************************\n");

    /* Display the route flush status: */
    NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "Debug: Is Flush Pending: %s\n", gRouteMgmtMCB.isFlushPending ? "Yes" : "No");

    /* Cycle through all the cache routes */
    ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        /* Get the display strings */
        nl_addr2str (ptrProxyRoute->dstIP, dstIPString, sizeof (dstIPString));
        nl_addr2str (ptrProxyRoute->srcIP, srcIPString, sizeof (srcIPString));

        /* Display the route cache: */
        NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "%p: %s->%s MAC 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:%02x [Count:%d]\n",
                           ptrProxyRoute, srcIPString, dstIPString, ptrProxyRoute->nextHopMACAddress[0],
                           ptrProxyRoute->nextHopMACAddress[1], ptrProxyRoute->nextHopMACAddress[2],
                           ptrProxyRoute->nextHopMACAddress[3], ptrProxyRoute->nextHopMACAddress[4],
                           ptrProxyRoute->nextHopMACAddress[5], ptrProxyRoute->refCount);

        /* Cycle through all the pending requests */
        ptrPendingRequest = (NetfpProxy_PendingRequest*)List_getHead ((List_Node**)&ptrProxyRoute->ptrPendingRequestList);
        while (ptrPendingRequest != NULL)
        {
            /* Display the pending requests: */
            NetfpProxy_dumpMsg (NETFP_PROXY_LOG_INFO, "    %p: Route Resolve Id: 0x%x\n",
                               ptrPendingRequest, ptrPendingRequest->routeResolutionId);

            /* Get the next pending request */
            ptrPendingRequest = (NetfpProxy_PendingRequest*)List_getNext ((List_Node*)ptrPendingRequest);
        }

        /* Get the next element */
        ptrProxyRoute = (NetfpProxy_Route*)List_getNext ((List_Node*)ptrProxyRoute);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the NETFP Proxy route module
 *
 *  @retval
 *      Success -   Socket descriptor associated with the route module
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_routeInit (void)
{
    int32_t         errCode;

    /* Initialize the route management module. */
    memset ((void *)&gRouteMgmtMCB, 0, sizeof(NetfpProxy_RouteMgmtMCB));

    /* Initialize the NETMGR for the interface module. */
    gRouteMgmtMCB.netMgrRouteHandle = netmgr_init (NetMgr_CacheType_ROUTE, &errCode);
    if (gRouteMgmtMCB.netMgrRouteHandle == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Initialization of the Route NETMGR failed [Error code %d]\n", errCode);
        return -1;
    }

//fzm-->
    nl_socket = socket(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE);

    if(nl_socket == -1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to create socket: %s (%d)\n", strerror(errno), errno);
        return -1;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Route NETMGR Initialized\n");
//fzm<--

    /* Return the socket file descriptor associated with the ROUTE NETMGR */
    return netmgr_getSocket(gRouteMgmtMCB.netMgrRouteHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of entries in the route cache.
 *
 *  @retval
 *      Success -   Number of entries
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_routeGetNumEntries(void)
{
    NetfpProxy_Route*   ptrProxyRoute;
    int32_t             numEntries = 0;

    /* Cycle through all the cache routes */
    ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        /* Increment the number of entries. */
        numEntries = numEntries + 1;

        /* Get the next element */
        ptrProxyRoute = (NetfpProxy_Route*)List_getNext ((List_Node*)ptrProxyRoute);
    }
    return numEntries;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the NETFP Proxy route module
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_routeDeinit(void)
{
//fzm-->
    if(nl_socket >= 0)
    {
        close(nl_socket);
    }
//fzm<--
    return 0;
}

