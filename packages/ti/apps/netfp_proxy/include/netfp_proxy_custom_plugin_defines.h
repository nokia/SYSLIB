#ifndef __NETFP_PROXY_CUSTOM_PLUGIN_DEFINES_H__
#define __NETFP_PROXY_CUSTOM_PLUGIN_DEFINES_H__

typedef void*   NetfpProxy_NetMgrHandle;

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

typedef struct NetfpProxy_SAInfo
{
    /**
     * @brief   Link to other SA Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Security Parameters Index (SPI)
     */
    uint32_t                spi;

    /**
     * @brief   SA Handle
     */
    Netfp_SAHandle          saHandle;

    /**
     * @brief   Reference count
     * SAs can be shared by multiple policies. This tracks
     * the number of policies using the SA. The SA is deleted
     * when this count reaches zero.
     */
    uint32_t                refCount;
}NetfpProxy_SAInfo;

typedef struct NetfpProxy_SPInfo
{
    /**
     * @brief   Link to other SP Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Policy id
     */
    uint32_t                policyId;
}NetfpProxy_SPInfo;

typedef struct NetfpProxy_reqIdInfo
{
    /**
     * @brief   Link to other reqid Info objects
     */
    List_Node               list_n;

    /**
     * @brief   Requiest Id
     */
    uint32_t                reqId;

    /**
     * @brief   Direction
     */
    Netfp_Direction         dir;

    /**
     * @brief   List of Security Associations 
     */
    NetfpProxy_SAInfo*      saList;

    /**
     * @brief   List of Security Policies
     */
    NetfpProxy_SPInfo*      spList;
}NetfpProxy_reqIdInfo;

typedef struct NetfpProxy_IPSECMgmtMCB
{
    /**
     * @brief   Time when the SA statistics were retreived from the NETFP Server
     */
    time_t                      statTime;

    /**
     * @brief   List of policies succesfully offloaded using Proxy.
     */
    NetfpProxy_reqIdInfo*   reqIdList;
}NetfpProxy_IPSECMgmtMCB;

#endif
