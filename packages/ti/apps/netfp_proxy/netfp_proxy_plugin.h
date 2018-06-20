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
#ifndef __NETFP_PROXY_PLUGIN_H__
#define __NETFP_PROXY_PLUGIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <stdint.h>

/* NETFP Proxy includes */
#include <ti/apps/netfp_proxy/netfp_proxy.h>
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/**
 * @brief
 *  NETFP Proxy Commands
 *
 * @details
 *  Used to indicate the type of command that needs to be
 *  executed by the NetFP Proxy.
 */
typedef enum NetfpProxy_CmdType
{
    /**
     * @brief
     *  Start offload of a security policy
     */
    NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ = 1,

    /**
     * @brief
     *  Stop offload of a security policy
     */
    NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ,

    /**
     * @brief
     *  Notify of IP Route Flush to Fast Path
     */
    NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ,

    /**
     * @brief
     *  Add Interface request
     */
    NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ,

    /**
     * @brief
     *  Delete Interface request
     */
    NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ,

    /**
     * @brief
     *  QoS Configuration request
     */
    NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ,

    /**
     * @brief
     *  Flush VLAN Egress Priority Mapping
     */
    NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ,

    /**
     * @brief
     *  Setup the PMTU Request
     */
    NETFP_PROXY_CMDTYPE_SETUP_PMTU_REQ,

    /**
     * @brief
     *  Max number of commands supported.
     */
    NETFP_PROXY_CMDTYPE_MAX
}NetfpProxy_CmdType;

/**
 * @brief
 *  Network event types supported
 */
typedef enum NetfpProxy_NetEventType
{
    /**
     * @brief
     *  Link related event
     */
    NETFP_PROXY_EVENTTYPE_LINK = 1,

    /**
     * @brief
     *  Address related event
     */
    NETFP_PROXY_EVENTTYPE_ADDRESS,

    /**
     * @brief
     *  Neighbor related event
     */
    NETFP_PROXY_EVENTTYPE_NEIGH,

    /**
     * @brief
     *  Routing related event
     */
    NETFP_PROXY_EVENTTYPE_ROUTE,

    /**
     * @brief
     *  Maximum event type
     */
    NETFP_PROXY_EVENTTYPE_MAX
}NetfpProxy_NetEventType;

/**
 * @brief   NETFP Proxy Command Id
 */
typedef uint32_t NetfpProxy_CmdId;

/**
 *  @b Description
 *  @n
 *      This is the callback function which is invoked by the NETFP Proxy to
 *      report responses to commands issued by the application.
 *
 *  @param[in]  cmd
 *      Command Type for which the response was generated
 *  @param[in]  cmdId
 *      Command Id
 *  @param[in]  cmdData
 *      Command data associated with the command
 *  @param[in]  appData
 *      Application Data associated with the command
 *  @param[in]  appDataLen
 *      Length of the application data
 *
 *  @retval
 *      Not applicable
 */
typedef void (*NetfpProxy_reportCmdResponse)(NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdData, void* appData, uint32_t appDataLen);

/**
 *  @b Description
 *  @n
 *      This is the callback function which is invoked by the NETFP Proxy to
 *      report networking events which have been captured by the NETFP Proxy
 *
 *  @param[in]  action
 *      Action associated with the event
 *  @param[in]  eventType
 *      Type of event
 *  @param[in]  eventData
 *      Event data
 *
 *  @retval
 *      Not applicable
 */
typedef void (*NetfpProxy_reportNetEvent)(int32_t action, NetfpProxy_NetEventType eventType, void* eventData);

/**
 *  @b Description
 *  @n
 *      This is the Plugin execution function registered with the NETFP Proxy and provides an
 *      execution context for the Plugin. This is called from within the PROXY core module.
 *
 *  @retval
 *      Not applicable
 */
typedef void (*NetfpProxy_executePlugin)(void);

/**
 *  @b Description
 *  @n
 *      This is the callback function invoked by the NETFP Proxy on the reception of an
 *      ICMP Fragmentation needed or an ICMPv6 Packet too big packet. Plugin can use this
 *      function to determine if they wish the PMTU information to be passed to the NETFP
 *      Server/Clients or whether PMTU updates need to be dropped.
 *
 *  @param[in]  srcIP
 *      Source IP address associated with the PMTU update
 *  @param[in]  dstIP
 *      Destination IP address associated with the PMTU update
 *  @param[in]  mtu
 *      Newly received MTU update
 *
 *  @retval
 *      0   -   Accept and pass along the PMTU update
 *  @retval
 *      <0  -   Drop the PMTU update
 */
typedef int32_t (*NetfpProxy_reportPMTU) (Netfp_IPAddr* srcIP, Netfp_IPAddr* dstIP, uint32_t* mtu);

/**
 *  @b Description
 *  @n
 *      This is the callback function invoked by the NETFP Proxy to log messages generated
 *      by the NETFP Proxy.
 *
 *  @param[in]  level
 *      Logging Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  ap
 *      Variable argument list
 *
 *  @retval
 *      Not applicable
 */
typedef void (*NetfpProxy_logMessages) (NetfpProxy_LogLevel level, char* fmt, va_list ap);

/**
 *  @b Description
 *  @n
 *      This is the Plugin exit function which is invoked when the NETFP Proxy is shutdown
 *
 *  @retval
 *      Not applicable
 */
typedef void (*NetfpProxy_exit)(void);

/**
 * @brief
 *  NETFP Proxy Plugin configuration
 *
 * @details
 *  The structure has the configuration which is required for a plugin to register
 *  itself with the NETFP Proxy.
 */
typedef struct NetfpProxy_PluginCfg
{
    /**
     * @brief  Plugin registered callback function to handle command responses.
     */
    NetfpProxy_reportCmdResponse   report_cmd_response;

    /**
     * @brief  Plugin registered callback function to get networking events.
     */
    NetfpProxy_reportNetEvent       report_net_event;

    /**
     * @brief   Plugin registered callback function which is the main execution function
     * for the plugin.
     */
    NetfpProxy_executePlugin        run;

    /**
     * @brief  Plugin registered callback function to get the PMTU Updates
     */
    NetfpProxy_reportPMTU           reportPMTU;

    /**
     * @brief   Plugin registered callback function for logging
     */
    NetfpProxy_logMessages          logMsg;

    /**
     * @brief   Plugin registered callback function invoked when the proxy is shutdown
     */
    NetfpProxy_exit                 exit;
} NetfpProxy_PluginCfg;


/* Exported APIs */
extern int32_t NetfpProxy_registerPlugin (NetfpProxy_PluginCfg* pluginCfg);
extern int32_t NetfpProxy_executeCommand (NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen);
extern char* NetfpProxy_cmdType2Str (NetfpProxy_CmdType cmd);
extern char* NetfpProxy_action2Str (int32_t action);
extern char* NetfpProxy_eventType2Str (NetfpProxy_NetEventType eventType);

#ifdef __cplusplus
}
#endif

#endif
