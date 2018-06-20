/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/include/netfp_proxy_xfrm_bridge_interface.h>

#define NETFP_PROXY_ACTIONS_MAX         6
/**
 * @brief   String equivalents of Actions
 */
const char NetfpProxy_ActionStr [NETFP_PROXY_ACTIONS_MAX][128] =
{
    "Unspecified",
    "New",
    "Delete",
    "Get",
    "Set",
    "Change"
};

/**********************************************************************************/
/* NOTE: The NetfpProxy_MsgType array in netfp_proxy_xfrm_bridge_interface.h must be 
 * updated to reflect changes in NetfpProxy_CmdTypeStr */
const char NetfpProxy_CmdTypeStr [NETFP_PROXY_IPC_MSGTYPE_MAX][128] =
{
    "Unknown Message", /* Note, message type 0 is not used */
    "Adding Security Association Request",
    "Adding Security Association Response",
    "Adding Security Policy Request",
    "Adding Security Policy Response",
    "Deleting Security Association Request",
    "Deleting Security Association Response",
    "Deleting Security Policy Request",
    "Deleting Security Policy Response",
    "SA Stats Request",
    "SA Stats Response",
    "Configure L3 QoS Request",
    "Configure L3 QoS Response",
    "Delete L3 QoS Request",
    "Delete L3 QoS Response",
    "Configure DSCP PCP Mapping Request",
    "Configure DSCP PCP Mapping Response",
    "Heartbeat Request",
    "Heartbeat Response",
    "Stop Security Association Request",
    "Stop Security Association Response",
    "Neighbor Ping Request",
    "Route Flush Request",
    "Add Interface Request",
    "Add Interface Response"
};
/* NOTE: The NetfpProxy_MsgType array in netfp_proxy_xfrm_bridge_interface.h must be 
 * updated to reflect changes in NetfpProxy_CmdTypeStr */
/**********************************************************************************/

const char* NetfpProxy_action2Str (int32_t action)
{
    /* Validate the action passed */
    if (action >= NETFP_PROXY_ACTIONS_MAX)
        return "Invalid Action";
    else
        return NetfpProxy_ActionStr[action];
}

const char* NetfpProxy_cmdType2Str (NetfpProxy_CmdType cmd)
{
    /* Validate the command passed.  If it's greater than what is defined, use
       type 0 (unknown) */
    if(cmd >= ((NetfpProxy_CmdType) NETFP_PROXY_IPC_MSGTYPE_MAX))
        cmd = 0;
    return NetfpProxy_CmdTypeStr[cmd];
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
    int32_t result = NETFP_PROXY_RETVAL_E_OP_FAILED;
    switch (cmd)
    {
        case NETFP_PROXY_CMDTYPE_IP_ROUTE_FLUSH_REQ:
        {
            /* Flush the route cache: */
            NetfpProxy_routeFlushCache ();
            result = NETFP_PROXY_RETVAL_SUCCESS;
            break;
        }
        case NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ:
        {
            NetfpProxy_InterfaceAddReq* ptrIfaceAddReq = (NetfpProxy_InterfaceAddReq*)cmdCfg;

            /* Create the interface */
            int32_t errCode, retVal;
            retVal = NetfpProxy_ifaceCreateInterface (ptrIfaceAddReq->interfaceName, &errCode);
            if (retVal == 0)
                result = NETFP_PROXY_RETVAL_SUCCESS;

            break;
        }
        case NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ:
        {
            NetfpProxy_InterfaceDelReq* ptrIfaceDelReq = (NetfpProxy_InterfaceDelReq *)cmdCfg;

            /* Process the delete interface request */
            int32_t errCode, retVal;
            retVal = NetfpProxy_ifaceDeleteInterface (ptrIfaceDelReq->interfaceName, &errCode);
            if (retVal == 0)
                result = NETFP_PROXY_RETVAL_SUCCESS;

            break;
        }
        case NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ:
        {
            NetfpProxy_ConfigureL3QosReq* ptrQosConfigReq = (NetfpProxy_ConfigureL3QosReq *)cmdCfg;

            /* Setup the L3 Shaper: */
            int32_t errCode, retVal;
            retVal = NetfpProxy_ifaceConfigureL3Shaper (ptrQosConfigReq->interfaceName,
                (Netfp_L3QoSCfg *)&ptrQosConfigReq->l3QoSCfg, &errCode);
            if (retVal == 0)
                result = NETFP_PROXY_RETVAL_SUCCESS;

            break;
        }
        case NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ:
        {
            NetfpProxy_FlushVlanPriorityReq* ptrFlushVlanPrioReq = (NetfpProxy_FlushVlanPriorityReq *)cmdCfg;

            /* Process the request */
            int32_t errCode, retVal;
            retVal = NetfpProxy_ifaceFlushVLANEgressMap (ptrFlushVlanPrioReq->interfaceName, &errCode);
            if (retVal == 0)
                result = NETFP_PROXY_RETVAL_SUCCESS;

            break;
        }
        default:
        {
            /* Unspported message type from plugin. */
            result = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        }
    }

    /* Done processing the message */
    return result;
}

