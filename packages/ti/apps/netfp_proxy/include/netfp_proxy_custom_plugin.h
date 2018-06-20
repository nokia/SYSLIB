/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/
#ifndef __NETFP_PROXY_CUSTOM_PLUGIN_H__
#define __NETFP_PROXY_CUSTOM_PLUGIN_H__

#include <stdint.h>
#include <ti/apps/netfp_proxy/include/netfp_proxy_xfrm_bridge_interface.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>
#include <sys/socket.h>
#include "listlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NETFP_PROXY_MAXNUM_INTERFACES       8

typedef uint32_t        NetfpProxy_CmdId;

typedef struct NetfpProxy_msg
{
    struct nlmsghdr netlinkHdr;
    union
    {
        NetfpProxy_MsgHdr                       hdr_only;
        NetfpProxy_AddSAReq                     add_sa_req;
        NetfpProxy_AddSAResp                    add_sa_resp;
        NetfpProxy_AddSPReq                     add_sp_req;
        NetfpProxy_AddSPResp                    add_sp_resp;
        NetfpProxy_DeleteSAReq                  del_sa_req;
        NetfpProxy_DeleteSAResp                 del_sa_resp;
        NetfpProxy_DeleteSPReq                  del_sp_req;
        NetfpProxy_DeleteSPResp                 del_sp_resp;
        NetfpProxy_SAStatsReq                   sa_stats_req;
        NetfpProxy_SAStatsResp                  sa_stats_resp;
        NetfpProxy_ConfigL3QosReq               conf_l3qos_req;
        NetfpProxy_ConfigL3QosResp              conf_l3qos_resp;
        NetfpProxy_DeleteL3QosReq               del_l3qos_req;
        NetfpProxy_DeleteL3QosResp              del_l3qos_resp;
        NetfpProxy_ConfigureDscpPcpMappingReq   conf_dscp_pcp_mapping_req;
        NetfpProxy_ConfigureDscpPcpMappingResp  conf_dscp_pcp_mapping_resp;
        NetfpProxy_HeartbeatReq                 heartbeat_req;
        NetfpProxy_HeartbeatResp                heartbeat_resp;
        NetfpProxy_StopSaReq                    stop_sa_req;
        NetfpProxy_StopSaResp                   stop_sa_resp;
        NetfpProxy_AddInterfaceReq              add_iface_req;
        NetfpProxy_AddInterfaceResp             add_iface_resp;
    } msg;
} __attribute__((packed)) NetfpProxy_msg;

typedef enum NetfpProxy_CmdType
{
	//The flowing are used for the plugin responses in my_report_cmd_response()
    NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ = 1001,
    NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ,
    NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ,
	NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ,
    NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ,
    NETFP_PROXY_CMDTYPE_SETUP_PMTU_REQ,
    NETFP_PROXY_CMDTYPE_MAX
}NetfpProxy_CmdType;

/* Dummy Declarations: Not Used*/
/*
char NetfpProxy_ActionStr [6][128] =
{
    "Unspecified",
    "New",
    "Delete",
    "Get",
    "Set",
    "Change"
};
*/
typedef enum NetfpProxy_NetEventType
{
    NETFP_PROXY_EVENTTYPE_LINK                  =   1,
    NETFP_PROXY_EVENTTYPE_ADDRESS               =   2,
    NETFP_PROXY_EVENTTYPE_NEIGH                 =   3,
    NETFP_PROXY_EVENTTYPE_ROUTE                 =   4,
    NETFP_PROXY_EVENTTYPE_MAX                   =   5
} NetfpProxy_NetEventType;

typedef enum NetfpProxy_DscpPolicy
{
    NETFP_PROXY_DSCP_COPY_FROM_INNER        =       1,
    NETFP_PROXY_DSCP_USE_FIXED_VAL          =       2,
    NETFP_PROXY_DSCP_USE_MAP_TABLE          =       3,
} NetfpProxy_DscpPolicy;

typedef struct NetfpProxy_DscpCfg
{
    NetfpProxy_DscpPolicy   policy;
    union {
    uint8_t                 fixedVal;
    uint8_t                 mapTable[64];
    } value;
} NetfpProxy_DscpCfg;

typedef struct NetfpProxy_GtpuIdCfg
{
    uint32_t                min;
    uint32_t                max;
} NetfpProxy_GtpuIdCfg;

typedef enum NetfpProxy_IPSecFragLevel
{
    NETFP_PROXY_IPSEC_FRAG_INNER_IP =   0,
    NETFP_PROXY_IPSEC_FRAG_OUTER_IP =   1,
} NetfpProxy_IPSecFragLevel;

typedef struct NetfpProxy_L3QoSCfg
{
    int32_t     isQosEnable;
    int32_t     flowId;
    uint32_t    qid[64];
}NetfpProxy_L3QoSCfg;

typedef struct NetfpProxy_InterfaceAddReq
{
    char                    interfaceName[32];
}NetfpProxy_InterfaceAddReq;

typedef struct NetfpProxy_InterfaceDelReq
{
    char                    interfaceName[32];
}NetfpProxy_InterfaceDelReq;

typedef struct NetfpProxy_ConfigureL3QosReq
{
    char                    interfaceName[32];
    NetfpProxy_L3QoSCfg     l3QoSCfg;
}NetfpProxy_ConfigureL3QosReq;

typedef struct NetfpProxy_FlushVlanPriorityReq
{
    char                    interfaceName[32];
}NetfpProxy_FlushVlanPriorityReq;

/* Dummy Definations Ends Here */

typedef struct NetfpProxy_PluginCfg
{
    int32_t     numFpInterfaces; //deprecated
    char        fpInterfaceName[NETFP_PROXY_MAXNUM_INTERFACES][32]; //deprecated
    void (*report_cmd_response) (NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdData, void* appData, uint32_t appDataLen);
    void (*report_net_event) (int32_t action, NetfpProxy_NetEventType eventType, void* eventData);
    void (*run) (void);
    void (*logMsg) (NetfpProxy_LogLevel level, const char* fmt, va_list ap)  __attribute__ ((format (printf, 2, 0)));
    void (*exit) (void);
}NetfpProxy_PluginCfg;

void hexDump(void *addr, int len);
extern int32_t NetfpProxy_registerPlugin (NetfpProxy_PluginCfg* pluginCfg);
extern const char* NetfpProxy_cmdType2Str (NetfpProxy_CmdType cmd);
extern const char* NetfpProxy_action2Str (int32_t action);
extern const char* NetfpProxy_eventType2Str (NetfpProxy_NetEventType eventType);
extern int32_t NetfpProxy_executeCommand (NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen);
extern uint32_t if_nametoindex (const char *__ifname);
extern void NetfpProxy_setLogLevel(NetfpProxy_LogLevel level);

int32_t NetfpProxy_addSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_DeleteSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_addSP_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_DeleteSP_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_SAStats_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_ConfigL3Qos_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_DelL3Qos_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_ConfigDscpPcpMapping_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_Hearbeat_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_StopSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
int32_t NetfpProxy_NeighborPing_CmdHandler(void);
int32_t NetfpProxy_RouteFlush_CmdHandler(void);
int32_t NetfpProxy_AddInterface_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen);
#endif
