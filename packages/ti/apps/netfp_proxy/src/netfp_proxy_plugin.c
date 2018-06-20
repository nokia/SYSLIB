/**
 *   @file  netfp_proxy_plugin.c
 *
 *   @brief
 *      NETFP Proxy Plugin Interface implementation
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

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy_plugin.h>

/**
 * @brief   String equivalents of Command Types
 */
char NetfpProxy_CmdTypeStr [NETFP_PROXY_CMDTYPE_MAX - 1][128] =
{
    "Start Offload Security Policy",
    "Stop Offload Security Policy",
    "IP Route Flush",
    "Add Interface",
    "Delete Interface",
    "Configure L3 QoS",
    "Flush VLAN Priority"
};

#define NETFP_PROXY_ACTIONS_MAX         6
/**
 * @brief   String equivalents of Actions
 */
char NetfpProxy_ActionStr [NETFP_PROXY_ACTIONS_MAX][128] =
{
    "Unspecified",
    "New",
    "Delete",
    "Get",
    "Set",
    "Change"
};

/**
 * @brief   String equivalents of Event Types
 */
char NetfpProxy_EventTypeStr [NETFP_PROXY_EVENTTYPE_MAX - 1][128] =
{
    "Link",
    "IPv4/IPv6 Address",
    "Neighbor table",
    "Routing table"
};

/**
 *  @b Description
 *  @n
 *      Returns the string equivalents of any given
 *      NetFP Proxy command type.
 *
 *  @param[in]  cmd
 *      Command type
 *
 *  @retval
 *      Command type description in string format
 */
char* NetfpProxy_cmdType2Str (NetfpProxy_CmdType cmd)
{
    /* Validate the command passed */
    if (cmd < NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ || cmd >= NETFP_PROXY_CMDTYPE_MAX)
        return "Invalid command";
    else
        return NetfpProxy_CmdTypeStr[cmd-1];
}

/**
 *  @b Description
 *  @n
 *      Returns the string equivalents of any given
 *      NetFP Proxy command type.
 *
 *  @param[in]  cmd
 *      Command type
 *
 *  @retval
 *      Action description in string format
 */
char* NetfpProxy_action2Str (int32_t action)
{
    /* Validate the action passed */
    if (action >= NETFP_PROXY_ACTIONS_MAX)
        return "Invalid Action";
    else
        return NetfpProxy_ActionStr[action];
}

/**
 *  @b Description
 *  @n
 *      Returns the string equivalents of any given
 *      NetFP Proxy event type.
 *
 *  @param[in]  eventType
 *      Event type
 *
 *  @retval
 *      Event type description in string format
 */
char* NetfpProxy_eventType2Str (NetfpProxy_NetEventType eventType)
{
    /* Validate the event passed */
    if (eventType < NETFP_PROXY_EVENTTYPE_LINK || eventType >= NETFP_PROXY_EVENTTYPE_ROUTE)
        return "Invalid Event";
    else
        return NetfpProxy_EventTypeStr[eventType-1];
}

/**
 *  @b Description
 *  @n
 *      Plugin registration function: Must be called by the plugin
 *      before calling any other NetFP Proxy APIs. This registers
 *      the plugin's callback functions with the NetFP proxy and enables
 *      the communication between the Proxy and the plugin.
 *
 *  @param[in]  pluginCfg
 *      Application's plugin configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_registerPlugin (NetfpProxy_PluginCfg* pluginCfg)
{
    /* Check if all callback functions are specified. */
    if ((pluginCfg->report_cmd_response == NULL) || (pluginCfg->run == NULL) ||
        (pluginCfg->logMsg == NULL) || (pluginCfg->exit == NULL))
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Save the plugin's callback functions */
    gNetfpProxyMcb.pluginCfg.report_cmd_response  = pluginCfg->report_cmd_response;
    gNetfpProxyMcb.pluginCfg.report_net_event     = pluginCfg->report_net_event;
    gNetfpProxyMcb.pluginCfg.run                  = pluginCfg->run;
    gNetfpProxyMcb.pluginCfg.logMsg               = pluginCfg->logMsg;
    gNetfpProxyMcb.pluginCfg.exit                 = pluginCfg->exit;

    /* Registration done. Return success */
    return NETFP_PROXY_RETVAL_SUCCESS;
}

/**
 *  @b Description
 *  @n
 *      This API interprets and processes any commands from the plugin. It returns an error
 *      if parsing the command or its arguments fails, otherwise it dispatches the command
 *      for processing by the NetFP Proxy.
 *
 *  @param[in]  cmd
 *      Command to be issued to the Proxy
 *  @param[in]  cmdId
 *      Unique Id to identify this command request/response pair between the plugin and the Proxy.
 *  @param[in]  cmdCfg
 *      Command parameters
 *  @param[in]  appData
 *      Application specific data corresponding to this request. Will be passed back as-is in
 *      response from NetFP Proxy. Optional parameter.
 *  @param[in]  appDataLen
 *      Length of application specific data length passed to this API. Can be zero.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpProxy_executeCommand (NetfpProxy_CmdType cmd, NetfpProxy_CmdId cmdId, void* cmdCfg, void* appData, uint32_t appDataLen)
{
    switch (cmd)
    {
        case NETFP_PROXY_CMDTYPE_STOP_OFFLOAD_SP_REQ:
        {
            /* Stop the offload: */
            return NetfpProxy_ipsecStopOffload (cmdId, cmdCfg, appData, appDataLen);
        }
        case NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ:
        {
            /* Start the offload: */
            return NetfpProxy_ipsecStartOffload (cmdId, cmdCfg, appData, appDataLen);
        }
        case NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ:
        {
            NetfpProxy_IpRouteFlushResp ipRouteFlushResp;

            /* Initialize the Route flush command response */
            memset ((void *)&ipRouteFlushResp, 0, sizeof (NetfpProxy_IpRouteFlushResp));

            /* Flush the route cache: */
            NetfpProxy_routeFlushCache ();

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &ipRouteFlushResp, appData, appDataLen);
            break;
        }
        case NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ:
        {
            NetfpProxy_InterfaceAddReq*         ptrIfaceAddReq;
            NetfpProxy_InterfaceAddResp         ifaceAddResp;

            /* Initialize the add interface command response */
            memset ((void *)&ifaceAddResp, 0, sizeof (NetfpProxy_InterfaceAddResp));

            /* Get the interface add request: */
            ptrIfaceAddReq = (NetfpProxy_InterfaceAddReq*)cmdCfg;

            /* Create the interface */
            ifaceAddResp.retVal = NetfpProxy_ifaceCreateInterface (ptrIfaceAddReq->interfaceName, &ifaceAddResp.netfpErrCode);
            strcpy (ifaceAddResp.interfaceName, ptrIfaceAddReq->interfaceName);

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &ifaceAddResp, appData, appDataLen);
            break;
        }
        case NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ:
        {
            NetfpProxy_InterfaceDelReq*         ptrIfaceDelReq;
            NetfpProxy_InterfaceDelResp         ifaceDelResp;

            /* Initialize the del interface command response */
            memset ((void *)&ifaceDelResp, 0, sizeof (NetfpProxy_InterfaceDelResp));

            /* Get the interface delete request: */
            ptrIfaceDelReq = (NetfpProxy_InterfaceDelReq *)cmdCfg;

            /* Process the delete interface request */
            NetfpProxy_ifaceDeleteInterface (ptrIfaceDelReq->interfaceName, &ifaceDelResp.netfpErrCode);
            strcpy (ifaceDelResp.interfaceName, ptrIfaceDelReq->interfaceName);

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &ifaceDelResp, appData, appDataLen);
            break;
        }
        case NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ:
        {
            NetfpProxy_ConfigureL3QosReq*         ptrQosConfigReq;
            NetfpProxy_ConfigureL3QosResp         configQosResp;

            /* Initialize the L3 QOS command response */
            memset ((void *)&configQosResp, 0, sizeof (NetfpProxy_ConfigureL3QosResp));

            /* Get the pointer to the L3 QOS Request: */
            ptrQosConfigReq = (NetfpProxy_ConfigureL3QosReq *)cmdCfg;

            /* Populate the response: */
            strcpy (configQosResp.interfaceName, ptrQosConfigReq->interfaceName);

            /* Setup the L3 Shaper: */
            configQosResp.retVal = NetfpProxy_ifaceConfigureL3Shaper (ptrQosConfigReq->interfaceName, (Netfp_L3QoSCfg *)&ptrQosConfigReq->l3QoSCfg,
                                                                      &configQosResp.netfpErrCode);

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &configQosResp, appData, appDataLen);
            break;
        }
        case NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ:
        {
            NetfpProxy_FlushVlanPriorityReq*        ptrFlushVlanPrioReq;
            NetfpProxy_FlushVlanPriorityResp        flushVlanPrioResp;

            /* Initialize the flush VLAN Priority command response */
            memset ((void *)&flushVlanPrioResp, 0, sizeof (NetfpProxy_FlushVlanPriorityResp));

            /* Get the pointer to the flush VLAN Priority request */
            ptrFlushVlanPrioReq = (NetfpProxy_FlushVlanPriorityReq *)cmdCfg;

            /* Populate the response: */
            strcpy (flushVlanPrioResp.interfaceName, ptrFlushVlanPrioReq->interfaceName);

            /* Process the request */
            flushVlanPrioResp.retVal = NetfpProxy_ifaceFlushVLANEgressMap (ptrFlushVlanPrioReq->interfaceName, &flushVlanPrioResp.netfpErrCode);

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &flushVlanPrioResp, appData, appDataLen);
            break;
        }
        case NETFP_PROXY_CMDTYPE_SETUP_PMTU_REQ:
        {
            NetfpProxy_SetupPMTUReq*        ptrSetupPMTUReq;
            NetfpProxy_SetupPMTUResp        setupMTUResp;
            Netfp_ProxyServerInfo           proxyServerInfo;

            /* Initialize the setup PMTU response */
            memset ((void *)&setupMTUResp, 0, sizeof (NetfpProxy_SetupPMTUResp));

            /* Get the pointer to SETUP MTU Request */
            ptrSetupPMTUReq = (NetfpProxy_SetupPMTUReq *)cmdCfg;

            /* Initialize the Proxy Server Informational Block: */
            memset ((void *)&proxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

            /* Populate the Proxy Server Informational Block: */
            proxyServerInfo.opType                = Netfp_ProxyServerOp_UPDATE_MTU;
            memcpy ((void *)&proxyServerInfo.dstIP, (void *)&ptrSetupPMTUReq->dstIP, sizeof(Netfp_IPAddr));
            memcpy ((void *)&proxyServerInfo.srcIP, (void *)&ptrSetupPMTUReq->srcIP, sizeof(Netfp_IPAddr));
            proxyServerInfo.u.updateMTU.newMTU    = ptrSetupPMTUReq->pmtu;

            /* Send the asynchronous MTU update to the NETFP Server */
            if (Netfp_asyncUpdate (gNetfpProxyMcb.netfpClientHandle, &proxyServerInfo, &setupMTUResp.netfpErrCode) < 0)
                setupMTUResp.retVal = NETFP_PROXY_IPC_RETVAL_ERROR;

            /* Send the response to the application */
            gNetfpProxyMcb.pluginCfg.report_cmd_response (cmd, cmdId, &setupMTUResp, appData, appDataLen);
            break;
        }
        default:
        {
            /* Unspported message type from plugin. */
            return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        }
    }

    /* Done processing the message */
    return NETFP_PROXY_RETVAL_SUCCESS;
}

