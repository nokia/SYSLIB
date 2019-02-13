/**
 *   @file  netfp_proxy_pvt.h
 *
 *   @brief
 *      Internal header file used by the NETFP Proxy.
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
#ifndef __NETFP_PROXY_PVT_H__
#define __NETFP_PROXY_PVT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Libnl include files */
#include <netlink/netlink.h>
#include <netlink/cache.h>
#include <netlink/object.h>
#include <netlink/route/addr.h>
#include <netlink/route/link.h>
#include <netlink/route/neighbour.h>
#include <netlink/route/route.h>
#include <netlink/route/nexthop.h>
#include <netlink/route/rule.h>
#include <linux/if.h>

/* SYSLIB Include Files. */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/listlib.h>

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_custom_plugin.h>

/**********************************************************************
 ************************** Definitions *******************************
 **********************************************************************/

/**
 * @brief
 *  Maximum supported IP address on an interface
 */
#define NETFP_PROXY_MAX_IP              32

/**
 * @brief
 *  Maximum Number of bridged interfaces supported
 */
#define NETFP_PROXY_MAX_BRIDGE_IF       32

/**
 * @brief
 *  Maximum Number of NETMGR caches which are being tracked & monitored
 */
#define NETMGR_CACHE_IDX_MAX            4

/**
 * @brief
 *  Path from where NETMGR loads the table names. Needed for
 *  LIBNL to translate table numbers to their names
 */
#define NETMGR_ROUTE_TABLE_PATH     "/etc/iproute2/rt_tables"

/**
 * @brief   Lock file for the Proxy daemon
 */
#define PROXY_LOCK_FILE                 "/var/lock/netfp_proxy"

/**
 * @brief
 *  Network Manager events supported
 */
#define NETMGR_LINK_EVT     0x1
#define NETMGR_ADDR_EVT     0x2
#define NETMGR_NEIGH_EVT    0x3
#define NETMGR_ROUTE_EVT    0x4

/** Special neighbor cache entry state to indicate that network manager
 *  has initiated an arping to maintain this entry in kernel cache. Indicates
 *  that last known state of this entry was NUD_STALE and cache state has been
 *  reset to enable an update from kernel on its new state */
#define NETMGR_NUD_RESET        0

/** Neighbor cache states recognized by Network manager */
#define NETMGR_NUD_VALID        (NUD_REACHABLE|NUD_PROBE|NUD_STALE|NUD_DELAY|NUD_PERMANENT)

/**********************************************************************
 ************************** Enums *************************************
 **********************************************************************/

/**
 * @brief
 *  Network Manager cache types
 */
typedef enum NetMgr_CacheType
{
    /**
     * @brief   Link Cache
     */
    NetMgr_CacheType_LINK  = 0,

    /**
     * @brief   Address Cache
     */
    NetMgr_CacheType_ADDR  = 1,

    /**
     * @brief   Neighbor Cache
     */
    NetMgr_CacheType_NEIGH  = 2,

    /**
     * @brief   Route Cache
     */
    NetMgr_CacheType_ROUTE  = 3,

    /**
     * @brief   Maximum Cache types supported
     */
    NetMgr_CacheType_MAX    = 4
}NetMgr_CacheType;

/**********************************************************************
 ************************** Structures ********************************
 **********************************************************************/

/* Opaque NETMGR Handle: */
typedef void*   NetfpProxy_NetMgrHandle;

/* Network manager subscriber info handle */
typedef void*   netmgr_subsc_hdl;

/* Event notification callback function */
typedef void (*netmgr_cb_fxn_t)(int32_t action, int32_t type, void* obj, void *arg);

/**
 * @brief
 *  NETFP Proxy Master Control Block
 *
 * @details
 *  Holds state information relevant for this instance of
 *  NetFP Proxy.
 */
typedef struct NetfpProxy_MCB
{
    /**
     * @brief   Plugin name
     */
    char                        pluginName[128];

    /**
     * @brief   Polling Delay in milliseconds.
     */
    uint32_t                    pollDelay;

    /**
     * @brief   SA stats update interval in seconds
     */
    uint32_t                    statsUpdateInterval;

    /**
     * @brief   NetFP server that the Proxy manages
     */
    char                        netfpServerName[NETFP_MAX_CHAR];

    /**
     * @brief   NetFP client name to use for Proxy
     */
    char                        netfpClientName[NETFP_MAX_CHAR];

    /**
     * @brief   RMv2 client name to use for Proxy
     */
    char                        rmClientName[RM_NAME_MAX_CHARS];

    /**
     * @brief Named resource instance identifier.
     */
    uint32_t                    nrInstanceId;

    /**
     * @brief Database handle associated with the named resource instance identifier.
     */
    Name_DBHandle               databaseHandle;

    /**
     * @brief   Proxy state.
     */
    uint32_t                    bIsInReset;

    /**
     * @brief   Test mode: For *internal* debugging only.
     */
    uint32_t                    testMode;

    /**
     * @brief   NETFP Proxys plugin configuration
     */
    NetfpProxy_PluginCfg        pluginCfg;

    /**
     * @brief Global System configuration handle.
     */
    Resmgr_SysCfgHandle         sysCfgHandle;

    /**
     * @brief Application specified Resource configuration.
     */
    Resmgr_ResourceCfg          appResourceConfig;

    /**
     * @brief MSGCOM Instance handle.
     */
    Msgcom_InstHandle           msgcomInstHandle;

    /**
     * @brief PKTLIB Instance handle.
     */
    Pktlib_InstHandle           pktlibInstHandle;

    /**
     * @brief NETFP Client Thread
     */
    pthread_t                   netfpClientThread;

    /**
     * @brief NETFP Proxy Thread
     */
    pthread_t                   netfpProxyThread;

    /**
     * @brief PKTLIB Heap used for NetFP client-server communication
     */
    Pktlib_HeapHandle           netfpProxyHeap;

    /**
     * @brief NetFP server handle.
     */
    Netfp_ServerHandle          netfpServerHandle;

    /**
     * @brief NetFP client handle.
     */
    Netfp_ClientHandle          netfpClientHandle;

    /**
     * @brief   Number of interfaces setup in NETFP
     */
    uint32_t                    numInterfaces;

    /**
     * @brief   Core module pipe
     */
    int32_t                     corePipe[2];

    /**
     * @brief   Interface module socket descriptor
     */
    int32_t                     ifSocket;

    /**
     * @brief   Route module socket descriptor
     */
    int32_t                     routeSocket;

    /**
     * @brief   Neighbor socket descriptor
     */
    int32_t                     neighSocket;

    /**
     * @brief   Plugin socket descriptor
     */
    int32_t                     pluginSocket;

}NetfpProxy_MCB;

/****************************************************************************
 ******************************** INLINE Functions **************************
 ****************************************************************************/

/**
 *  @b Description
 *  @n
 *      Given a Boolean value, returns corresponding string representation.
 *
 *  @param[in]  boolVal
 *      Boolean value
 *
 *  @retval
 *      String representation
 */
static inline const char* NetfpProxy_getBoolean2Str (uint32_t boolVal)
{
    switch (boolVal)
    {
        case 0:
        {
            return "No";
        }
        case 1:
        {
            return "Yes";
        }
        default:
        {
            return "INVALID";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function which computes the maximum of numbers.
 *
 *  @param[in]  a
 *      First Number
 *  @param[in]  b
 *      Second Number
 *
 *  @retval
 *      Largest number
 */
static inline uint32_t max (uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

/****************************************************************************
 ******************************** Global Variables **************************
 ****************************************************************************/
extern NetfpProxy_MCB gNetfpProxyMcb;
extern pthread_mutex_t libnlGuard;

/****************************************************************************
 ******************************** Exported API ******************************
 ****************************************************************************/
#if 0 //not used in fzm
/* IPSEC exported API: */
extern int32_t NetfpProxy_ipsecInit (void);
extern int32_t NetfpProxy_ipsecDeinit (void);
extern void    NetfpProxy_ipsecExecute (void);
extern void    NetfpProxy_ipsecDump (void);
extern int32_t NetfpProxy_ipsecStopOffload(NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen);
extern int32_t NetfpProxy_ipsecStartOffload(NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen);
#endif //fzm

/* Interface exported API: */
extern int32_t NetfpProxy_ifaceInit(void);
extern int32_t NetfpProxy_ifaceDeinit(void);
extern void    NetfpProxy_ifaceExecute (void);
extern void    NetfpProxy_ifaceDump(void);
extern int32_t NetfpProxy_ifaceCreateInterface (const char* ifName, int32_t* errCode);
extern int32_t NetfpProxy_ifaceDeleteInterface (const char* ifName, int32_t* errCode);
extern int32_t NetfpProxy_ifaceOffloadInterface (const char* ifName, uint8_t* ptrMACAddress, Netfp_IfHandle* ptrIfHandle);
extern int32_t NetfpProxy_ifaceConfigureL3Shaper(const char* ifName, Netfp_L3QoSCfg* ptrL3QOSCfg, int32_t* errCode);
extern int32_t NetfpProxy_ifaceFlushVLANEgressMap(const char* ifName, int32_t* errCode);

/* Route exported API: */
extern int32_t NetfpProxy_routeInit (void);
extern int32_t NetfpProxy_routeDeinit (void);
extern void    NetfpProxy_routeExecute (void);
extern void    NetfpProxy_routePoll (void);
extern void    NetfpProxy_routeDump(void);
extern int32_t NetfpProxy_routeLookup(Netfp_ProxyServerBulkMsg* ptrProxyServerBulkInfo);
extern void    NetfpProxy_routeFlushCache (void);
extern void    NetfpProxy_routeUpdate(uint32_t listenerId, Netfp_IPAddr* ptrIPAddress, uint8_t* ptrMACAddress);
extern int32_t NetfpProxy_routeKill(const char* ifName);
extern int32_t NetfpProxy_routeGetNumEntries(void);

/* Neighbor exported API: */
extern int32_t NetfpProxy_neighInit (void);
extern int32_t NetfpProxy_neighDeinit(void);
extern void    NetfpProxy_neighExecute (void);
extern void    NetfpProxy_neighDump(void);
extern int32_t NetfpProxy_neighLookup(const char* ifName, Netfp_IPAddr* ptrTargetIP, uint32_t listenerId);
extern int32_t NetfpProxy_neighStopLookup(const char* ifName, Netfp_IPAddr* ptrTargetIP, uint32_t listenerId);
extern int32_t NetfpProxy_neighGetLookupStatus(uint32_t* listenerId, Netfp_IPAddr* ptrTargetIP, uint8_t* macAddress);
extern int32_t NetfpProxy_neighGetNumEntries(void);

/* Core exported API: */
extern void Netfp_addToPendingProxyToServerList(int32_t jobId);
extern int32_t NetfpProxy_executeCommand(NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen);//fzm
extern void NetfpProxy_logMsg (NetfpProxy_LogLevel level, const char* fmt,...)  __attribute__ ((format (printf, 2, 3)));
extern int readFromPipe(int fd, void *buf, size_t count, int* error);
extern int writeToPipe(int fd, const void *buf, size_t count, int* error);

/********************************************************************************************
 * NETMGR Exported API:
 ********************************************************************************************/

/* Initialization/De-initialization APIs */
extern NetfpProxy_NetMgrHandle netmgr_init (NetMgr_CacheType cacheType, int32_t* errCode);
extern int32_t netmgr_deinit (NetfpProxy_NetMgrHandle netMgrHandle);

/* Link related APIs */
extern netmgr_subsc_hdl netmgr_register_link_updates (NetfpProxy_NetMgrHandle netMgrHandle, const char* if_name,
                                                      netmgr_cb_fxn_t cb_fxn, void* cb_arg, int32_t* error);
extern int32_t netmgr_unregister_link_updates (NetfpProxy_NetMgrHandle netMgrHandle, netmgr_subsc_hdl);
extern int32_t netmgr_link_get_by_name (NetfpProxy_NetMgrHandle netMgrHandle, const char* if_name, struct rtnl_link** nl_link);

/* Address related APIs */
extern netmgr_subsc_hdl netmgr_register_addr_updates (NetfpProxy_NetMgrHandle netMgrHandle, const char* if_name,
                                                      netmgr_cb_fxn_t cb_fxn, void* cb_arg, int32_t* error);
extern int32_t netmgr_unregister_addr_updates (NetfpProxy_NetMgrHandle netMgrHandle, netmgr_subsc_hdl);
extern int32_t netmgr_addr_get (NetfpProxy_NetMgrHandle netMgrHandle, const char* if_name, uint32_t** ipaddr_info_list, uint32_t* num_addrs);

/* Neighbor table (ARP table) related APIs */
extern netmgr_subsc_hdl netmgr_register_neigh_updates (NetfpProxy_NetMgrHandle netMgrHandle, struct nl_addr* ip_addr,
                                                       netmgr_cb_fxn_t cb_fxn, void* cb_arg, int32_t* error);
extern int32_t netmgr_unregister_neigh_updates (NetfpProxy_NetMgrHandle netMgrHandle, netmgr_subsc_hdl);
extern int32_t netmgr_neigh_get (NetfpProxy_NetMgrHandle netMgrHandle, struct nl_addr* ip_addr,
                                 const char* if_name, uint32_t** neigh_subsc_list, uint32_t* num_neighs);

/* Routing related APIs */
extern netmgr_subsc_hdl netmgr_register_route_updates (NetfpProxy_NetMgrHandle netMgrHandle, const char* if_name,
                                                       netmgr_cb_fxn_t cb_fxn, void* cb_arg, int32_t* error);
extern int32_t netmgr_unregister_route_updates (NetfpProxy_NetMgrHandle netMgrHandle, netmgr_subsc_hdl);
extern int32_t netmgr_find_route(NetfpProxy_NetMgrHandle netMgrHandle, struct nl_addr* dest_addr, struct nl_addr* src_addr,
                                struct nl_addr** nh_addr, char* if_name, int32_t* table_no);
extern int32_t netmgr_route_cache_flush(NetfpProxy_NetMgrHandle netMgrHandle, int32_t* errCode);

/* Generic Utility API */
extern int32_t netmgr_getSocket (NetfpProxy_NetMgrHandle netMgrHandle);
extern int32_t netmgr_process_message (NetfpProxy_NetMgrHandle netMgrHandle);

#ifdef __cplusplus
}
#endif

#endif

