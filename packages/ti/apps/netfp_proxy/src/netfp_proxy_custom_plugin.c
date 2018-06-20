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
/* Standard library includes */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <syslog.h>

/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/netfp_proxy.h>
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop.h>
#include <ti/apps/netfp_proxy/include/netfp_proxy_custom_plugin_defines.h>
#include <ti/apps/netfp_proxy/include/netfp_proxy_fzm_l3qos_custom.h>

#include <FPDisp/Utils/SharedMemoryLog/SmLog.h>
#include <ti/apps/netfp_config/include/NetFPConfig_SharedMemoryLog.h>
#include <ti/runtime/netfp/netfp.h>

pthread_t  NetlinkMsgThread;
pthread_t  NeighborPingThread;

/* Global variable definitions */
struct sockaddr_nl          NetlinkProxyAddr;
int32_t                     NetlinkProxySockFd = -1;
struct sockaddr_nl          NetlinkNeighPingAddr;
int32_t                     NetlinkNeighPingSockFd = -1;
struct sockaddr_nl          NetlinkRouteFlushAddr;
int32_t                     NetlinkRouteFlushSockFd = -1;
struct sockaddr_nl          NetlinkRouteSunAddr;
int32_t                     NetlinkRouteSockFd = -1;
int32_t                     NetlinkARPSockFd = -1;

extern NetfpProxy_RouteMgmtMCB  gRouteMgmtMCB;

/* This is a node to the list that maintains the added security associations */
typedef struct  {
    List_Node       listNode;
    Netfp_SAHandle  saHandle;
    Netfp_Direction direction;
} NetfpPlugin_SAListNode;

/* This is a node to the list that maintains the added security policies */
typedef struct {
    List_Node       listNode;
    ipsecmgr_policy_id_t policy_id;
} NetfpPlugin_SPListNode;

NetfpPlugin_SAListNode *gSAList = NULL;
NetfpPlugin_SPListNode *gSPList = NULL;

extern uint32_t gL3QosPassFlowId;

extern int (*netmgr_nl_update_neigh)(const char *if_name, struct sockaddr *dst_addr, int family);
extern int (*NetfpProxy_neighPluginARPing)(const char *if_name, struct sockaddr *dst_addr, int family);

extern void NetfpProxy_logMsgPlug(NetfpProxy_LogLevel level, const char* fmt, va_list ap) __attribute__ ((format (printf, 2, 0)));
extern NetfpProxy_LogLevel NetfpProxy_getLogLevel(void);

void NetfpPlugin_convertIPToString(Netfp_IPAddr* ptrIPAddress, int32_t strLen, char* string)
{
    /* Use the address family to convert the IP address to string: */
    if (ptrIPAddress->ver == Netfp_IPVersion_IPV4)
    {
        snprintf (string, strLen, "%d.%d.%d.%d", ptrIPAddress->addr.ipv4.u.a8[0],
                  ptrIPAddress->addr.ipv4.u.a8[1], ptrIPAddress->addr.ipv4.u.a8[2],
                  ptrIPAddress->addr.ipv4.u.a8[3]);
    }
    else
    {
        Netfp_convertIP6ToStr (ptrIPAddress->addr.ipv6, string);
    }
    return;
}

void NetfpPlugin_convertKeyToStr(uint8_t *key, uint32_t keyLen, char* str)
{
    int offset = 0;
    int i;
    for(i = 0; i < keyLen; ++i)
        offset += snprintf(str + offset, 3, "%02X", key[i]);
}

/**
 *  @b Description
 *  @n
 *      Find a SA List node on the list based on SA Handle.
 *
 *  @param[in]
 *      reqID       The request id to add the handle
 *
 *  @retval
 *      non-NULL  - pointer to the node that contains the request ID
 *      NULL - Error finding SA based on request ID
 */
NetfpPlugin_SAListNode *NetfpPlugin_SAListFindBySAHandle(Netfp_SAHandle saHandle)
{
    NetfpPlugin_SAListNode *saListNode = NULL;

    saListNode = (NetfpPlugin_SAListNode *) List_getHead((List_Node **) &gSAList);

    while(saListNode != NULL)
    {
        if(saListNode->saHandle == saHandle)
            break;

        saListNode = (NetfpPlugin_SAListNode *) List_getNext((List_Node *) saListNode);
    }

    return(saListNode);
}

/**
 *  @b Description
 *  @n
 *      Add a security assocation to the list.
 *
 *  @param[in]
 *      saHandle    The handle to add to the list.
 *
 *  @retval
 *      TRUE  - Success
 *      FALSE - Request ID is already on the list or malloc failed
 */
int NetfpPlugin_SAListAdd(Netfp_SAHandle saHandle, Netfp_Direction direction)
{
    NetfpPlugin_SAListNode *newSAListNode = NULL;

    if(NetfpPlugin_SAListFindBySAHandle(saHandle) != NULL)
    {
        return(FALSE);
    }

    if(NULL == (newSAListNode = (NetfpPlugin_SAListNode *) malloc(sizeof(NetfpPlugin_SAListNode))))
    {
        return(FALSE);
    }

    newSAListNode->saHandle = saHandle;
    newSAListNode->direction = direction;

    List_addNode((List_Node **) &gSAList, (List_Node *) newSAListNode);

    return(TRUE);
}

/**
 *  @b Description
 *  @n
 *      Remove a security assocation based on the list node pointer.
 *
 *  @param[in]
 *      saNodeToRemove  pointer to the node to remove
 *
 *  @retval
 *      TRUE  - Success
 *      FALSE - Invalid pointer or Node could not be removed.
 */
int NetfpPlugin_SAListRemove(NetfpPlugin_SAListNode *saNodeToRemove)
{
    if(NULL == saNodeToRemove)
    {
        return(FALSE);
    }

    if(List_removeNode((List_Node **) &gSAList, (List_Node *) saNodeToRemove))
    {
        return(FALSE);
    }

    free(saNodeToRemove);

    return(TRUE);
}

/**
 *  @b Description
 *  @n
 *      Find a SP List node on the list based on policy ID.
 *
 *  @param[in]
 *      policy_id       The policy ID to find
 *
 *  @retval
 *      non-NULL  - pointer to the node that contains the request ID
 *      NULL - Error finding SA based on request ID
 */
NetfpPlugin_SPListNode *NetfpPlugin_SPListFindByPolicyID(ipsecmgr_policy_id_t policy_id)
{
    NetfpPlugin_SPListNode *spListNode = NULL;

    spListNode = (NetfpPlugin_SPListNode *) List_getHead((List_Node **) &gSPList);

    while(spListNode != NULL)
    {
        if(spListNode->policy_id == policy_id)
            break;

        spListNode = (NetfpPlugin_SPListNode *) List_getNext((List_Node *) spListNode);
    }

    return(spListNode);
}

/**
 *  @b Description
 *  @n
 *      Add a security Policy to the list.
 *
 *  @param[in]
 *      policy_id   The policy ID to add to the list.
 *
 *  @retval
 *      TRUE  - Success
 *      FALSE - Request ID is already on the list or malloc failed
 */
int NetfpPlugin_SPListAdd(ipsecmgr_policy_id_t policy_id)
{
    NetfpPlugin_SPListNode *newSPListNode = NULL;

    if(NetfpPlugin_SPListFindByPolicyID(policy_id) != NULL)
    {
        return(FALSE);
    }

    if(NULL == (newSPListNode = (NetfpPlugin_SPListNode *) malloc(sizeof(NetfpPlugin_SPListNode))))
    {
        return(FALSE);
    }

    newSPListNode->policy_id = policy_id;

    List_addNode((List_Node **) &gSPList, (List_Node *) newSPListNode);

    return(TRUE);
}

/**
 *  @b Description
 *  @n
 *      Remove a security assocation based on the list node pointer.
 *
 *  @param[in]
 *      saNodeToRemove  pointer to the node to remove
 *
 *  @retval
 *      TRUE  - Success
 *      FALSE - Invalid pointer or Node could not be removed.
 */
int NetfpPlugin_SPListRemove(NetfpPlugin_SPListNode *spNodeToRemove)
{
    if(NULL == spNodeToRemove)
    {
        return(FALSE);
    }

    if(List_removeNode((List_Node **) &gSPList, (List_Node *) spNodeToRemove))
    {
        return(FALSE);
    }

    free(spNodeToRemove);

    return(TRUE);
}

/**
 *  @b Description
 *  @n
 *      Plugin's main loop. Polls for commands from user
 *      application over Netlink socket and executes them
 *      in Proxy's context.
 *
 *  @retval
 *      Not applicable
 */
void my_netlink_poll (void)
{
    NetfpProxy_msg      rxMsg;
    uint32_t            destAddrLen = sizeof (struct sockaddr_nl);
    struct sockaddr_nl  fromAddr;
    int32_t             msglen;
    int32_t             retVal;

    if ((msglen = recvfrom (NetlinkProxySockFd,
                    (void *)&rxMsg,
                    sizeof (NetfpProxy_msg),
                    0,
                    (struct sockaddr *)&fromAddr,
                    &destAddrLen)) > 0)
    {
        /* Got a command from user application. Parse it. */
        switch (rxMsg.msg.hdr_only.msgType)
        {
            case NETFP_PROXY_IPC_MSGTYPE_ADD_SA_REQ:
                {
                    retVal = NetfpProxy_addSA_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_ADD_SP_REQ:
                {
                    retVal = NetfpProxy_addSP_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_DEL_SA_REQ:
                {
                    retVal = NetfpProxy_DeleteSA_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_DEL_SP_REQ:
                {
                    retVal = NetfpProxy_DeleteSP_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_SA_STATS_REQ:
                {
                    retVal = NetfpProxy_SAStats_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_CONFIG_L3QOS_REQ:
                {
                    retVal = NetfpProxy_ConfigL3Qos_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_DEL_L3QOS_REQ:
                {
                    retVal = NetfpProxy_DelL3Qos_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_DSCP_PCP_MAPPING_REQ:
                {
                    retVal = NetfpProxy_ConfigDscpPcpMapping_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_HEARTBEAT_REQ:
                {
                    retVal = NetfpProxy_Hearbeat_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_STOP_SA_REQ:
                {
                    retVal = NetfpProxy_StopSA_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            case NETFP_PROXY_IPC_MSGTYPE_NEIGHBOR_PING_REQ:
                {
                    retVal = NetfpProxy_NeighborPing_CmdHandler();
                    break;
                }
             case NETFP_PROXY_IPC_MSGTYPE_ROUTE_FLUSH_REQ:
                {
                    retVal = NetfpProxy_RouteFlush_CmdHandler();
                    break;
                }
             case NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ:
                {
                    retVal = NetfpProxy_AddInterface_CmdHandler(&rxMsg, &fromAddr, destAddrLen, msglen);
                    break;
                }
            default:
                {
                    NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "ERROR: Unrecognized Request %d\n", rxMsg.msg.hdr_only.msgType);
                }
        }
        if(retVal)
        {
            NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "ERROR: Processing of message failed with return value %d\n", retVal);
        }
    }
    else if(msglen < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Receiving Netlink message: %m\n");
    }
    return;
}

void NetfpProxy_SendResponse(void *msgContentsPtr, uint32_t msgLen, NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen)
{
    NetfpProxy_msg responseMsg;

    /* Copy the message contents, then increase the message length
       to account for the Netlink header */
    memcpy(&(responseMsg.msg), msgContentsPtr, msgLen);
    msgLen += sizeof(struct nlmsghdr);

    /* Since this is an internal operation, assume msgContentsPtr,
       reqMsg, and addrData are not NULL.  Fill in the netlink message
       header.  Some things are copied from the request message. */
    responseMsg.netlinkHdr.nlmsg_len = msgLen;
    responseMsg.netlinkHdr.nlmsg_type = NLMSG_NOOP;
    responseMsg.netlinkHdr.nlmsg_flags = 0;
    responseMsg.netlinkHdr.nlmsg_seq = reqMsg->netlinkHdr.nlmsg_seq;
    responseMsg.netlinkHdr.nlmsg_pid = NETFP_PROXY_NETLINK_PID;

    /* Reuse the address from the request message, swapping the pid. */
    addrData->nl_pid = reqMsg->netlinkHdr.nlmsg_pid;

    /* Send the command response to User application */
    if (sendto (NetlinkProxySockFd, &responseMsg, msgLen, 0, (const struct sockaddr *) addrData, addrLen) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Failed sending response for %s\n", NetfpProxy_cmdType2Str(reqMsg->msg.hdr_only.msgType));
    }
    else if(reqMsg->msg.hdr_only.msgType != NETFP_PROXY_IPC_MSGTYPE_HEARTBEAT_REQ)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Sent response for %s, Sequence: 0x%08X to Netlink PID: %d\n",
                           NetfpProxy_cmdType2Str(reqMsg->msg.hdr_only.msgType), reqMsg->netlinkHdr.nlmsg_seq, reqMsg->netlinkHdr.nlmsg_pid);
        NetfpProxy_syslogMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Sent response for %s, Sequence: 0x%08X to Netlink PID: %d\n",
                              NetfpProxy_cmdType2Str(reqMsg->msg.hdr_only.msgType), reqMsg->netlinkHdr.nlmsg_seq, reqMsg->netlinkHdr.nlmsg_pid);
    }
}

int32_t NetfpProxy_addSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t                     index, errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    Netfp_SACfg                 saCfg = {0};
    Netfp_SAHandle              saHandle = NULL;
    NetfpPlugin_SAListNode*     saListNode;
    Netfp_SwContext             secContext;
    NetfpProxy_AddSAReq*        sa_req = &(reqMsg->msg.add_sa_req);
    NetfpProxy_AddSAResp        sa_resp;
    char                        strSrcIp[40], strDstIp[40];

    //Input Validation
    if(sa_req == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Input is NULL");
        goto RESP_ADD_SA;
    }

    if(msglen < (sizeof(NetfpProxy_AddSAReq) + sizeof(struct nlmsghdr)))
    {
        NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete Add SA Request Message. Size is %d, expected at least %d.",
            msglen, sizeof(NetfpProxy_AddSAReq) + sizeof(struct nlmsghdr));
        goto RESP_ADD_SA;
    }

    NetfpPlugin_convertIPToString((Netfp_IPAddr*)&sa_req->src_addr, sizeof(strSrcIp), strSrcIp);
    NetfpPlugin_convertIPToString((Netfp_IPAddr*)&sa_req->dst_addr, sizeof(strDstIp), strDstIp);

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSA Req: spi:0x%08X sa_mode:%u dir:%s root_sa:0x%08X proto:%u mode:%u arw:%u offload:%s",
                       sa_req->spi, sa_req->sa_mode, (sa_req->direction == NetfpProxy_Direction_INBOUND ? "in" : "out"),
                       sa_req->root_sa_handle, sa_req->ipsecCfg.proto,
                       sa_req->ipsecCfg.mode, sa_req->ipsecCfg.arw_size, (sa_req->offload_mode == NETFP_PROXY_OFFLOAD_MODE_HARDWARE ? "hw" : "sw"));
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSA Req: authMode:%u encMode:%u sa_flags:0x%X authKeyLen:%u encKeyLen:%u icvlen:%u",
                       sa_req->ipsecCfg.auth_mode, sa_req->ipsecCfg.enc_mode, sa_req->ipsecCfg.sa_flags,
                       sa_req->ipsecCfg.auth_key_len, sa_req->ipsecCfg.enc_key_len, sa_req->ipsecCfg.icvlen);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSA Req: esn_lo:%u esn_hi:%u nattSrcPort:%u nattDstPort:%u srcIp:%s dstIp:%s",
                       sa_req->ipsecCfg.esn_lo, sa_req->ipsecCfg.esn_hi, sa_req->ipsecCfg.natt_src_port,
                       sa_req->ipsecCfg.natt_dst_port, strSrcIp, strDstIp);

    if (NetfpProxy_getLogLevel() == NETFP_PROXY_LOG_VRB)
    {
        char strAuthKey[NETFP_PROXY_MAX_IPSEC_KEY_LEN*2 + 1], strEncKey[NETFP_PROXY_MAX_IPSEC_KEY_LEN*2 + 1];

        NetfpPlugin_convertKeyToStr(sa_req->ipsecCfg.auth_key, sa_req->ipsecCfg.auth_key_len, strAuthKey);
        NetfpPlugin_convertKeyToStr(sa_req->ipsecCfg.enc_key, sa_req->ipsecCfg.enc_key_len, strEncKey);

        NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "addSA Req: authKey:%s encKey:%s", strAuthKey, strEncKey);
    }

    /************* Verifiction from ipsec.c *************************/
    /* Sanity Check: Validate the direction */
    if ((sa_req->direction != NetfpProxy_Direction_INBOUND) && (sa_req->direction != NetfpProxy_Direction_OUTBOUND))
    {
        /* Error: Invalid direction */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid direction during adding SA [Direction %d]", sa_req->direction);
        goto RESP_ADD_SA;
    }

    /* Sanity Check: Validate the Address format */
    if ((sa_req->dst_addr.version != NetfpProxy_IPVersion_IPV4) && (sa_req->dst_addr.version != NetfpProxy_IPVersion_IPV6))
    {
        /* Error: Invalid address family */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid address family during adding SA [AF %d]", sa_req->dst_addr.version);
        goto RESP_ADD_SA;
    }

    /* Sanity Check: IPSec protocol in the only supported protocol */
    if (sa_req->ipsecCfg.proto != NetfpProxy_IPSecProto_IPSEC_ESP)
    {
        /* Error: Invalid protocol */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid protocol [Protocol %d]", sa_req->ipsecCfg.proto);
        goto RESP_ADD_SA;
    }

    /* Sanity Check: Only Tunnel mode is supported */
    if (sa_req->ipsecCfg.mode != NetfpProxy_IPSecMode_IPSEC_TUNNEL)
    {
        /* Error: Invalid protocol */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid mode [Mode %d]", sa_req->ipsecCfg.mode);
        goto RESP_ADD_SA;
    }

    /* Validate the key lengths. */
    if (sa_req->ipsecCfg.auth_key_len > NETFP_MAX_IPSEC_KEY_LEN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Authentication key size (%d) is invalid.", sa_req->ipsecCfg.auth_key_len);
        goto RESP_ADD_SA;
    }
    if (sa_req->ipsecCfg.enc_key_len > NETFP_MAX_IPSEC_KEY_LEN)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Encryption key size (%d) is invalid.", sa_req->ipsecCfg.enc_key_len);
        goto RESP_ADD_SA;
    }

    if ((sa_req->offload_mode != NETFP_PROXY_OFFLOAD_MODE_HARDWARE) && (sa_req->offload_mode != NETFP_PROXY_OFFLOAD_MODE_SOFTWARE))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid offload_mode for SA [%d]", sa_req->offload_mode);
        goto RESP_ADD_SA;
    }
    /************ END Verifiction from ipsec.c *************************/

    /************ Populate the SA config structure *********************/
    /* Get the SA direction. */
    if (sa_req->direction == NetfpProxy_Direction_INBOUND)
        saCfg.direction = Netfp_Direction_INBOUND;
    else
        saCfg.direction = Netfp_Direction_OUTBOUND;

    if (sa_req->offload_mode == NETFP_PROXY_OFFLOAD_MODE_HARDWARE)
        saCfg.offloadMode = Netfp_OffloadMode_HARDWARE;
    else
        saCfg.offloadMode = Netfp_OffloadMode_SOFTWARE;

    /* Get the SPI. */
    saCfg.spi = sa_req->spi;

    /* Get the IP protocol version. */
    if (sa_req->dst_addr.version == NetfpProxy_IPVersion_IPV4)
    {
        /* IPv4 Family: */
        saCfg.dstIP.ver = Netfp_IPVersion_IPV4;
        saCfg.srcIP.ver = Netfp_IPVersion_IPV4;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 4; index++)
        {
            saCfg.dstIP.addr.ipv4.u.a8[index] = sa_req->dst_addr.addr.ipv4.u.a8[index];
            saCfg.srcIP.addr.ipv4.u.a8[index] = sa_req->src_addr.addr.ipv4.u.a8[index];
        }
    }
    else
    {
        /* IPv6 Family: */
        saCfg.dstIP.ver = Netfp_IPVersion_IPV6;
        saCfg.srcIP.ver = Netfp_IPVersion_IPV6;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 16; index++)
        {
            saCfg.dstIP.addr.ipv6.u.a8[index] = sa_req->dst_addr.addr.ipv6.u.a8[index];
            saCfg.srcIP.addr.ipv6.u.a8[index] = sa_req->src_addr.addr.ipv6.u.a8[index];
        }
    }

    /* Only IPSEC protocols are supported */
    saCfg.ipsecCfg.protocol = Netfp_IPSecProto_IPSEC_ESP;
    saCfg.ipsecCfg.mode = Netfp_IPSecMode_IPSEC_TUNNEL;

    /* Get the authentication mode algorithm. */
    if (NetfpProxy_IpsecAuthMode_NULL == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_NULL;
    else if (NetfpProxy_IpsecAuthMode_HMAC_SHA1 == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_SHA1;
    else if (NetfpProxy_IpsecAuthMode_HMAC_MD5 == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_MD5;
    else if (NetfpProxy_IpsecAuthMode_AES_XCBC == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_AES_XCBC;
    else if (NetfpProxy_IpsecAuthMode_SHA2 == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_HMAC_SHA2_256;
    else if (NetfpProxy_IpsecAuthMode_GMAC == sa_req->ipsecCfg.auth_mode)
        saCfg.ipsecCfg.authMode = Netfp_IpsecAuthMode_AES_GMAC;
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Unsupported authentication algorithm (%d)", sa_req->ipsecCfg.auth_mode);
        goto RESP_ADD_SA;
    }

    /* Get the encryption mode algorithm. */
    if(NetfpProxy_IpsecCipherMode_NULL == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_NULL;
    else if(NetfpProxy_IpsecCipherMode_AES_CTR == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_CTR;
    else if(NetfpProxy_IpsecCipherMode_AES_CBC == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_CBC;
    else if(NetfpProxy_IpsecCipherMode_3DES_CBC == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_3DES_CBC;
    else if(NetfpProxy_IpsecCipherMode_DES_CBC == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_DES_CBC;
    else if(NetfpProxy_IpsecCipherMode_GCM == sa_req->ipsecCfg.enc_mode)
        saCfg.ipsecCfg.encMode = Netfp_IpsecCipherMode_AES_GCM;
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Unsupported encryption algorithm (%d)", sa_req->ipsecCfg.enc_mode);
        goto RESP_ADD_SA;
    }

    /* Populate ESN parameters.  If the ESN enable flag is set, copy over the high and low values */
    if (sa_req->ipsecCfg.sa_flags & IPSECMGR_SA_FLAGS_ESN)
    {
        saCfg.ipsecCfg.esnEnabled = 1;
        saCfg.ipsecCfg.esnHi = sa_req->ipsecCfg.esn_hi;
        saCfg.ipsecCfg.esnLo = sa_req->ipsecCfg.esn_lo;
    }
    else
    {
        /* If ESN is not enabled, the high is set to 0.  The low is the starting sequence number. */
        saCfg.ipsecCfg.esnEnabled = 0;
        saCfg.ipsecCfg.esnHi = 0;
        saCfg.ipsecCfg.esnLo = sa_req->ipsecCfg.esn_lo;
    }

    /* Store the key sizes */
    saCfg.ipsecCfg.keyAuthSize = sa_req->ipsecCfg.auth_key_len;
    saCfg.ipsecCfg.keyEncSize  = sa_req->ipsecCfg.enc_key_len;
    saCfg.ipsecCfg.keyMacSize  = sa_req->ipsecCfg.icvlen;

    /* Copy over the authentication/encryption keys. */
    memcpy ((void *)saCfg.ipsecCfg.keyAuth, sa_req->ipsecCfg.auth_key, sizeof (uint8_t) * sa_req->ipsecCfg.auth_key_len);
    memcpy ((void *)saCfg.ipsecCfg.keyEnc, sa_req->ipsecCfg.enc_key, sizeof (uint8_t) * sa_req->ipsecCfg.enc_key_len);

    /* Get the lifetime configuration */
    saCfg.ipsecCfg.lifetime.softByteLimit   = sa_req->lifetime.soft_byte_limit;
    saCfg.ipsecCfg.lifetime.hardByteLimit   = sa_req->lifetime.hard_byte_limit;
    saCfg.ipsecCfg.lifetime.softPacketLimit = sa_req->lifetime.soft_packet_limit;
    saCfg.ipsecCfg.lifetime.hardPacketLimit = sa_req->lifetime.hard_packet_limit;
    saCfg.ipsecCfg.lifetime.softAddExpires  = sa_req->lifetime.soft_add_expires;
    saCfg.ipsecCfg.lifetime.hardAddExpires  = sa_req->lifetime.hard_add_expires;
    saCfg.ipsecCfg.lifetime.softUseExpires  = sa_req->lifetime.soft_use_expires;
    saCfg.ipsecCfg.lifetime.hardUseExpires  = sa_req->lifetime.hard_use_expires;

    /* Set the replay window size. */
    saCfg.replayWindowSize = sa_req->ipsecCfg.arw_size;

    /* Add NAT-T UDP encapsulation info (Note: will be uncommented before final submission) */
    saCfg.nattEncapCfg.srcPort = sa_req->ipsecCfg.natt_src_port;
    saCfg.nattEncapCfg.dstPort = sa_req->ipsecCfg.natt_dst_port;

#if 0 //Devkit needs update
    /* Setup the fragmentation level for this SA. */
    if (sa_req->ipsecCfg.sa_flags & IPSECMGR_SA_FLAGS_OUTER_FRAG)
    {
        /* Perform outer IP fragmentation on this tunnel */
        saCfg.fragLevel  = Netfp_IPSecFragLevel_OUTER_IP;
    }
    else
    {
        /* Perform inner IP fragmentation on this tunnel */
        saCfg.fragLevel  = Netfp_IPSecFragLevel_INNER_IP;
    }
#else
    /* Enable Inner IP fragmentation by default */
    saCfg.fragLevel  = Netfp_IPSecFragLevel_INNER_IP;
#endif

    /************ END Populate the SA config structure *******************/

    /* Check if this is a rekey scenario */
    if(sa_req->ipsecCfg.sa_flags & NETFP_PROXY_SA_FLAGS_REKEY)
    {
        /* Make sure the root SA Handler is on the list.  This will also make sure it's not NULL
           since a NULL SA handler should never be added to the list */
        if((saListNode = NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) sa_req->root_sa_handle)) != NULL)
        {
            /* This a SA rekey request. Call the NETFP to add the rekeyed security association. */
            saHandle = Netfp_rekeySA (gNetfpProxyMcb.netfpClientHandle, saListNode->saHandle, &saCfg, &errCode);
            if (saHandle == NULL)
            {
                /* Note, in this case, we don't want to remove the SA or change the original SA association
                   with the SP.  Leave it as is and report the error. */
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Rekey SA failed [Error code %d]", errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
                errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
                goto RESP_ADD_SA;
            }
        } else {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Rekey requested, but root SA Handle not on list: 0x%08x", sa_req->root_sa_handle);
            goto RESP_ADD_SA;
        }
    }
    else
    {
        /* This is a new SA entry. Call Netfp_addSA to setup the SA in NetFP */
        saHandle = Netfp_addSA (gNetfpProxyMcb.netfpClientHandle, &saCfg, &errCode);
        if (saHandle == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding SA failed [Error code %d]", errCode);
            NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
            errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
            goto RESP_ADD_SA;
        }
    }

    if(TRUE != NetfpPlugin_SAListAdd(saHandle, saCfg.direction))
    {
        /* This sa handle is NULL (which is bad because the rekey/add was successful)
           or the sa handle is already on the list (which is bad because the saHandle is
           a reference to a data struture).  Either way, this should never happen, but there is
           not much we can do unless we 'undo' the rekey or add.  Flag the error, but continue*/
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Could not add Security Association to list.  Continuing anyway");
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Added SA Handle: 0x%08x\n", (uint32_t) saHandle);

    if(Netfp_getIPSecSwInfo(gNetfpProxyMcb.netfpClientHandle, saHandle, &secContext, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Netfp_getIPSecSwInfo failed with errCode: %d",errCode);
        memset((void *) &(sa_resp.sa_hw_ctx), 0, sizeof(NetfpProxy_SaHwCtx));
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
        errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    }
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,
            "DEBUG: Netfp_getIPSecSwInfo Successful: Retrieved SaHwCtx for flow_ID: 0x%08x (0x%08x)", secContext.flowId, gL3QosPassFlowId);
        if(secContext.swInfo.size > NETFP_PROXY_SA_SWINFO_MAX_SZ)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Netfp_getIPSecSwInfo returned large size: %d", secContext.swInfo.size);
            sa_resp.sa_hw_ctx.swinfo_sz = NETFP_PROXY_SA_SWINFO_MAX_SZ;
        }
        else
        {
            sa_resp.sa_hw_ctx.swinfo_sz = secContext.swInfo.size;
        }
        for(index = 0; index < NETFP_PROXY_SA_SWINFO_MAX_SZ; index ++)
        {
            if(index < sa_resp.sa_hw_ctx.swinfo_sz)
                sa_resp.sa_hw_ctx.swinfo[index] = secContext.swInfo.swInfo[index];
            else
                sa_resp.sa_hw_ctx.swinfo[index] = 0;
        }
        /* Use the flow id that was originally requested incase L3 QoS is not yet
           enabled or the route for the SA is not yet resolved */
        sa_resp.sa_hw_ctx.flow_id = gL3QosPassFlowId;
        errCode = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

RESP_ADD_SA:
    sa_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_ADD_SA_RESP;
    sa_resp.status      = errCode;
    sa_resp.sa_handle   = (uint32_t) saHandle;

    NetfpProxy_SendResponse(&sa_resp, sizeof(sa_resp), reqMsg, addrData, addrLen);

    return errCode;
}

int32_t NetfpProxy_DeleteSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t                 errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    NetfpPlugin_SAListNode *saListNode;
    NetfpProxy_DeleteSAReq* del_sa_req = &(reqMsg->msg.del_sa_req);
    NetfpProxy_DeleteSAResp del_sa_resp;

    //Input Validation
    if(del_sa_req == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Input is NULL\n");
        goto RESP_DEL_SA;
    }
    if(msglen < (sizeof(NetfpProxy_DeleteSAReq) + sizeof(struct nlmsghdr)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete Del SA Request Message. Size is %d, expected at least %d.\n",
            msglen, sizeof(NetfpProxy_DeleteSAReq) + sizeof(struct nlmsghdr));
        goto RESP_DEL_SA;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "delSA Req: saHandle:0x%08x newRootSaHandle:0x%08x",
                       del_sa_req->sa_handle, del_sa_req->new_root_sa_handle);

    /* Find the SA Node based on the SAHandle */
    if(NULL == (saListNode = NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) del_sa_req->sa_handle)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteSA Failed to find SA Handle\n");
        goto RESP_DEL_SA;
    }

    /* Check if the SA for the SP is being replaced with a new root SA.  If so,
       the new root SA handle will be non-NULL. */
    if(NULL != (Netfp_SAHandle) del_sa_req->new_root_sa_handle)
    {
        /* Make sure it's valid (i.e. it was previously added and is on the list) */
        if(NULL != NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) del_sa_req->new_root_sa_handle))
        {
            /* Replace the current SA with the new root SA.  If there was an error, report the
               error code, but change it to NETFP_PROXY_IPC_RETVAL_ERROR for the response message. */
            if(Netfp_changeRootSA(gNetfpProxyMcb.netfpClientHandle, saListNode->saHandle,
                (Netfp_SAHandle) del_sa_req->new_root_sa_handle, &errCode) == NULL)
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteSA failed to change root SA to: 0x%08x, errcode %d\n",
                    del_sa_req->new_root_sa_handle, errCode);
                NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
                errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
                goto RESP_DEL_SA;
            }
        } else {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteSA Failed to find new root SA on list: 0x%08x\n", del_sa_req->new_root_sa_handle);
            goto RESP_DEL_SA;
        }
    }

    /* Remove the SA using the NetFP call. */
    if(Netfp_delSA (gNetfpProxyMcb.netfpClientHandle, saListNode->saHandle, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteSA Netfp_delSA failed with errCode: %d\n", errCode);
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
        errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    }
    else
    {
       errCode = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

    /* Remove the SA handle from the list.  If it fails, leave the error code as is since it will indicate
       the the SA was removed with NetFP.  Log the error so we know something is wrong with the list though */
    if(TRUE != NetfpPlugin_SAListRemove(saListNode))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: NetfpPlugin_SAListRemove failed for node: 0x%08x\n", (uint32_t) saListNode);
    }

RESP_DEL_SA:
    del_sa_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_DEL_SA_RESP;
    del_sa_resp.status = errCode;

    NetfpProxy_SendResponse(&del_sa_resp, sizeof(del_sa_resp), reqMsg, addrData, addrLen);

    return errCode;
}

int32_t NetfpProxy_addSP_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    //Define the ipsecmgr parameters
    int32_t                 index, errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    NetfpPlugin_SAListNode* saListNode;
    Netfp_SPCfg             spCfg = {0};
    NetfpProxy_AddSPReq*    sp_req = &(reqMsg->msg.add_sp_req);
    NetfpProxy_AddSPResp    sp_resp;
    char                    strSrcIp[40], strDstIp[40];

    //Input Validation
    if(sp_req == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Input is NULL\n");
        goto RESP_ADD_SP;
    }
    if(msglen < (sizeof(NetfpProxy_AddSPReq) - sizeof(NetfpProxy_L5SelInfo) + sizeof(struct nlmsghdr)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete Add SP Request Message. Size is %d, expected at least %d.\n",
            msglen, sizeof(NetfpProxy_AddSPReq) - sizeof(NetfpProxy_L5SelInfo) + sizeof(struct nlmsghdr));
        goto RESP_ADD_SP;
    }

    NetfpPlugin_convertIPToString((Netfp_IPAddr*)&sp_req->src_addr_info.ip_addr, sizeof(strSrcIp), strSrcIp);
    NetfpPlugin_convertIPToString((Netfp_IPAddr*)&sp_req->dst_addr_info.ip_addr, sizeof(strDstIp), strDstIp);

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSP Req: policyId:0x%08X saHandle:0x%08X dir:%u srcPrefLen:%u srcIP:%s",
                       sp_req->policy_id, sp_req->sa_handle, sp_req->direction, sp_req->src_addr_info.prefix_len, strSrcIp);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSP Req: dstPrefLen:%u dstIP:%s srcPortStart:%u srcPortEnd:%u dstPortStart:%u dstPortEnd:%u",
                       sp_req->dst_addr_info.prefix_len, strDstIp, sp_req->src_port_info.port_start, sp_req->src_port_info.port_end,
                       sp_req->dst_port_info.port_start, sp_req->dst_port_info.port_end);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "addSP Req: l5Sel:%u l5SelProto:%u l5TeidStart:%u, l5TeidEnd:%u",
                       sp_req->l5_sel, sp_req->l5_sel_info.l5_sel_proto, sp_req->l5_sel_info.l5_sel_gtpu_teid_start, sp_req->l5_sel_info.l5_sel_gtpu_teid_end);

    /************* Verifiction from ipsec.c *************************/

    /* Sanity Check: protocol version */
    if((sp_req->src_addr_info.ip_addr.version != NetfpProxy_IPVersion_IPV4) && (sp_req->src_addr_info.ip_addr.version != NetfpProxy_IPVersion_IPV6))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid address family (%d)\n", sp_req->src_addr_info.ip_addr.version);
        goto RESP_ADD_SP;
    }

    /* Sanity Check: direction */
    if((sp_req->direction != NetfpProxy_Direction_OUTBOUND) && (sp_req->direction != NetfpProxy_Direction_INBOUND))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Invalid direction (%d)\n", sp_req->direction);
        goto RESP_ADD_SP;
    }

    /* Sanity Check: l5 Selector */
    if(sp_req->l5_sel == 1)
    {
        if(msglen < (sizeof(NetfpProxy_AddSPReq) + sizeof(struct nlmsghdr)))
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete Add SP Request Message l5_sel. Size is %d, expected at least %d.\n",
                msglen, sizeof(NetfpProxy_AddSPReq) + sizeof(struct nlmsghdr));
                goto RESP_ADD_SP;
        }
        if(sp_req->l5_sel_info.l5_sel_proto != NETFP_PROXY_L5_PROTO_GTPU)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: L5 protocol (%d) is invalid.\n", sp_req->l5_sel_info.l5_sel_proto);
            goto RESP_ADD_SP;
        }
    }

    /************ END Verifiction from ipsec.c *************************/

    /* Validate the SA handle if provided and make sure the direction is the same as the Request */
    if((Netfp_SAHandle) sp_req->sa_handle != NULL)
    {
        if(NULL == (saListNode = NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) sp_req->sa_handle)))
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Could not find Security Association Handle: 0x%08x\n", sp_req->sa_handle);
            goto RESP_ADD_SP;
        }
        if(((saListNode->direction == Netfp_Direction_OUTBOUND) && (sp_req->direction != NetfpProxy_Direction_OUTBOUND)) ||
           ((saListNode->direction == Netfp_Direction_INBOUND) && (sp_req->direction != NetfpProxy_Direction_INBOUND)))
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: SA direction (%d) does not match request direction (%d)", saListNode->direction, sp_req->direction);
            goto RESP_ADD_SP;
        }
    }

    /* No need to check if the policy is has already been added, this will be done by NetfpPlugin_SPListAdd */

    /************ Populate the SP config structure *********************/
    /* Populate the policy configuration */
    spCfg.spId = sp_req->policy_id;

    /* Get the IP protocol version. */
    if (sp_req->src_addr_info.ip_addr.version == NetfpProxy_IPVersion_IPV4)
    {
        /* IPv4 Family: */
        spCfg.dstIP.ver = Netfp_IPVersion_IPV4;
        spCfg.srcIP.ver = Netfp_IPVersion_IPV4;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 4; index++)
        {
            spCfg.dstIP.addr.ipv4.u.a8[index] = sp_req->dst_addr_info.ip_addr.addr.ipv4.u.a8[index];
            spCfg.srcIP.addr.ipv4.u.a8[index] = sp_req->src_addr_info.ip_addr.addr.ipv4.u.a8[index];
        }
    }
    else
    {
        /* IPv6 Family: */
        spCfg.dstIP.ver = Netfp_IPVersion_IPV6;
        spCfg.srcIP.ver = Netfp_IPVersion_IPV6;

        /* Populate the source and destination IP addresses. */
        for (index = 0; index < 16; index++)
        {
            spCfg.dstIP.addr.ipv6.u.a8[index] = sp_req->dst_addr_info.ip_addr.addr.ipv6.u.a8[index];
            spCfg.srcIP.addr.ipv6.u.a8[index] = sp_req->src_addr_info.ip_addr.addr.ipv6.u.a8[index];
        }
    }

    /* Get the security policy direction. */
    spCfg.direction = (sp_req->direction == NetfpProxy_Direction_OUTBOUND)?(Netfp_Direction_OUTBOUND):(Netfp_Direction_INBOUND);

    /* Populate remaining SP parameters. */
    spCfg.srcIPPrefixLen = sp_req->src_addr_info.prefix_len;
    spCfg.dstIPPrefixLen = sp_req->dst_addr_info.prefix_len;
    spCfg.protocol       = 0x32;
    spCfg.dstPortStart   = sp_req->dst_port_info.port_start;
    spCfg.dstPortEnd     = sp_req->dst_port_info.port_end;
    spCfg.srcPortStart   = sp_req->src_port_info.port_start;
    spCfg.srcPortEnd     = sp_req->src_port_info.port_end;

    /* If layer 5 properties are available then populate them. */
    if(sp_req->l5_sel == 1)
    {
        spCfg.gtpuIdStart = sp_req->l5_sel_info.l5_sel_gtpu_teid_start;
        spCfg.gtpuIdEnd   = sp_req->l5_sel_info.l5_sel_gtpu_teid_end;
    }
    else
    {
        /* Non-GTPU. Set the tunnel Ids to 0. */
        spCfg.gtpuIdStart = 0;
        spCfg.gtpuIdEnd   = 0;
    }

    /* Setup the SA handle passed in the policy configuration. */
    spCfg.saHandle = (Netfp_SAHandle) sp_req->sa_handle;

    /************ END Populate the SA config structure *******************/

    /************ Add the security policy ******************************/

    if(!NetfpPlugin_SPListAdd(sp_req->policy_id))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Could not add Policy ID to list\n");
        goto RESP_ADD_SP;
    }

    if(Netfp_addSP(gNetfpProxyMcb.netfpClientHandle, &spCfg, &errCode) < 0)
    {
        /* Error: NETFP Server was unable to add the security policy */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Adding security policy %d in NETFP Server failed [Error code: %d]\n",
                           sp_req->policy_id, errCode);
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
        if(TRUE != NetfpPlugin_SPListRemove(NetfpPlugin_SPListFindByPolicyID(sp_req->policy_id)))
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to remove Policy ID from list\n");
        }
        errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    }
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Adding NetfpProxy_addSP Successful\n");
        errCode = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

RESP_ADD_SP:
    sp_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_ADD_SP_RESP;
    sp_resp.sp_handle = sp_req->policy_id; // sp_handle is really not needed anymore.
    sp_resp.status    = errCode;

    NetfpProxy_SendResponse(&sp_resp, sizeof(sp_resp), reqMsg, addrData, addrLen);

    return errCode;
}

int32_t NetfpProxy_DeleteSP_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t                 errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    NetfpPlugin_SPListNode* spListNode;
    NetfpProxy_DeleteSPReq* del_sp_req = &(reqMsg->msg.del_sp_req);
    NetfpProxy_DeleteSPResp del_sp_resp;

    //Input Validation
    if(del_sp_req == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Input is NULL\n");
        goto RESP_DEL_SP;
    }
    if(msglen < (sizeof(NetfpProxy_DeleteSAReq) + sizeof(struct nlmsghdr)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete Del SA Request Message. Size is %d, expected at least %d.\n",
            msglen, sizeof(NetfpProxy_DeleteSAReq) + sizeof(struct nlmsghdr));
        goto RESP_DEL_SP;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "delSP Req: policyId:0x%08x spHandle:0x%08x", del_sp_req->policy_id, del_sp_req->sp_handle);

    /* Find the Policy ID from the list */
    if(NULL == (spListNode = NetfpPlugin_SPListFindByPolicyID(del_sp_req->policy_id)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Could not find the Policy ID on the list\n");
        goto RESP_DEL_SP;
    }

    if( Netfp_delSP(gNetfpProxyMcb.netfpClientHandle, del_sp_req->policy_id, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,"ERROR: Deleting SP failed with Errcode = %d\n", errCode);
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);

        if (errCode == NETFP_EINVAL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_INFO,"DelSP returned invalid val error, SP doesn't exist in server?");
            if(TRUE != NetfpPlugin_SPListRemove(spListNode))
            {
                NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to remove Policy ID from list\n");
            }
        }

        errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    }
    else
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Deleting NetfpProxy_deleteSP Successful\n");
        if(TRUE != NetfpPlugin_SPListRemove(spListNode))
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: Failed to remove Policy ID from list\n");
        }
        errCode = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

RESP_DEL_SP:
    del_sp_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_DEL_SP_RESP;
    del_sp_resp.status = errCode;

    NetfpProxy_SendResponse(&del_sp_resp, sizeof(del_sp_resp), reqMsg, addrData, addrLen);

    return errCode;
}

int32_t NetfpProxy_SAStats_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t                 errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    Netfp_IpSecStats        netfpIpsecStats;
    NetfpPlugin_SAListNode* saListNode;
    NetfpProxy_SAStatsReq*  sa_stat_req = &(reqMsg->msg.sa_stats_req);
    NetfpProxy_SAStatsResp  sa_stats_resp;

    memset(&sa_stats_resp, 0, sizeof(sa_stats_resp));

    if(sa_stat_req == NULL)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Input is NULL\n");
        goto RESP_SA_STATS;
    }
    if(msglen < (sizeof(NetfpProxy_SAStatsReq) + sizeof(struct nlmsghdr)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete SA Stats Request Message. Size is %u, expected at least %u.\n",
            msglen, sizeof(NetfpProxy_SAStatsReq) + sizeof(struct nlmsghdr));
        goto RESP_SA_STATS;
    }

    /* Find the SA Node based on the SAHandle */
    if(NULL == (saListNode = NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) sa_stat_req->sa_handle)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: NetfpProxy_SAStats_CmdHandler Failed to find SAHandle: 0x%08x", sa_stat_req->sa_handle);
        goto RESP_SA_STATS;
    }

    if (Netfp_getIPsecStats(gNetfpProxyMcb.netfpClientHandle, saListNode->saHandle, &netfpIpsecStats, &errCode) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: NetFP getIPsecStats failed with error code %d for SAHandle: 0x%08x", errCode, sa_stat_req->sa_handle);
        NetfpProxy_assertCriticalError(errCode, __func__, __LINE__);
        errCode = NETFP_PROXY_IPC_RETVAL_ERROR;
    }
    else
    {
        char buffer[100] = {0};

        if (saListNode->direction == Netfp_Direction_INBOUND)
        {
            sa_stats_resp.bytes   = netfpIpsecStats.u.in.inIpsecBytes;
            sa_stats_resp.packets = netfpIpsecStats.u.in.inIpsecPkts;
            sa_stats_resp.replay_fail = netfpIpsecStats.u.in.inIpsecDiscReplayFail;
            sa_stats_resp.espcrypto_fail = netfpIpsecStats.u.in.inIpsecDiscIntegrityFail;

            snprintf(buffer, sizeof(buffer), "inPkt:%llu inB:%llu inRepFail:%u inIntFail:%u",
                     netfpIpsecStats.u.in.inIpsecPkts,
                     netfpIpsecStats.u.in.inIpsecBytes,
                     netfpIpsecStats.u.in.inIpsecDiscReplayFail,
                     netfpIpsecStats.u.in.inIpsecDiscIntegrityFail);
        }
        else
        {
            sa_stats_resp.bytes   = netfpIpsecStats.u.out.outIpsecBytes;
            sa_stats_resp.packets = netfpIpsecStats.u.out.outIpsecPkts;

            snprintf(buffer, sizeof(buffer), "outPkt:%llu outB:%llu outSeqOvFail:%u",
                     netfpIpsecStats.u.out.outIpsecPkts,
                     netfpIpsecStats.u.out.outIpsecBytes,
                     netfpIpsecStats.u.out.outIpsecDiscSeqOv);
        }

        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: SAStats(0x%08x) spi=%X: %s", sa_stat_req->sa_handle, netfpIpsecStats.spi, buffer);
        errCode = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

RESP_SA_STATS:
    sa_stats_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_SA_STATS_RESP;
    sa_stats_resp.status = errCode;

    NetfpProxy_SendResponse(&sa_stats_resp, sizeof(sa_stats_resp), reqMsg, addrData, addrLen);

    return errCode;
}

int32_t NetfpProxy_ConfigL3Qos_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t status = NETFP_PROXY_IPC_RETVAL_ERROR;
    int rc;
    int i = 0;
    Netfp_L3QoSCfg l3QosCfg;
    NetfpProxy_ConfigL3QosReq *req = &(reqMsg->msg.conf_l3qos_req);
    NetfpProxy_ConfigL3QosResp  cfgl3qos_resp;

    if(msglen < (sizeof(NetfpProxy_ConfigL3QosReq) + sizeof(struct nlmsghdr)) || strcmp((char *) req->interface_name, "") == 0) {
	NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigL3QosReq: invalid received msg length %u or null intf\n", msglen);
        goto send_response;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ConfigL3QosReq: interfaceName: %s", req->interface_name);
    for(i = 0; i < 16; ++i)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ConfigL3QosReq: %s %s %s %s",
                           req->l3_qos_config[i*4], req->l3_qos_config[i*4 + 1],
                           req->l3_qos_config[i*4 + 2], req->l3_qos_config[i*4 + 3]);
    }

    // Check if interface is exist. If not, then add it.
    NetfpProxy_InterfaceAddReq addIntfReq;
    strncpy(addIntfReq.interfaceName, (char *) req->interface_name, sizeof(addIntfReq.interfaceName) - 1);
    addIntfReq.interfaceName[sizeof(addIntfReq.interfaceName)-1] = '\0';
    rc = NetfpProxy_executeCommand(NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ, reqMsg->netlinkHdr.nlmsg_seq, (void *)&addIntfReq, 0, 0);
    if(rc != NETFP_PROXY_RETVAL_SUCCESS) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigL3QosReq: add intf error return\n");
        goto send_response;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ConfigL3QosReq: shaper flowId %d\n", gL3QosPassFlowId);

    // set L3 config
    l3QosCfg.isEnable = 1;
    l3QosCfg.flowId = gL3QosPassFlowId;
    for(i = 0; i < 64; i++) {
        if(lookupL3QosQueue((const char *) req->l3_qos_config[i], &l3QosCfg.qid[i]) < 0) {
	    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigL3QosReq: invalid DSCP code[%d] name[%s]\n", i, req->l3_qos_config[i]);
            goto send_response;
        };
    }

    // Call proxy to set L3 QOS
    NetfpProxy_ConfigureL3QosReq l3Req;
    strncpy(l3Req.interfaceName, (char *) req->interface_name, sizeof(l3Req.interfaceName) - 1);
    l3Req.interfaceName[sizeof(l3Req.interfaceName)-1] = '\0';
    memcpy(&l3Req.l3QoSCfg, &l3QosCfg, sizeof(l3QosCfg));
    rc = NetfpProxy_executeCommand(NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ, reqMsg->netlinkHdr.nlmsg_seq, (void *) &l3Req, 0, 0);
    if(rc != NETFP_PROXY_RETVAL_SUCCESS) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigL3QosReq: config L3 QoS return error %d\n", rc);
        goto send_response;
    }
    status = NETFP_PROXY_IPC_RETVAL_SUCCESS;

send_response:
    cfgl3qos_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_CONFIG_L3QOS_RESP;
    cfgl3qos_resp.status = status;

    NetfpProxy_SendResponse(&cfgl3qos_resp, sizeof(cfgl3qos_resp), reqMsg, addrData, addrLen);

    return status;
}

int32_t NetfpProxy_DelL3Qos_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t status = NETFP_PROXY_IPC_RETVAL_ERROR;
    int rc = 0;
    Netfp_L3QoSCfg l3QosCfg;
    NetfpProxy_DeleteL3QosReq *req = &(reqMsg->msg.del_l3qos_req);
    NetfpProxy_DeleteL3QosResp  del3qos_resp;

    if(msglen < (sizeof(NetfpProxy_DeleteL3QosReq) + sizeof(struct nlmsghdr)) || strcmp((char *) req->interface_name, "") == 0) {
	NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteL3QosReq: invalid received msg length %u or null intf\n", msglen);
        goto send_response;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DeleteL3QosReq: interfaceName: %s", req->interface_name);

    // disable L3 config
    l3QosCfg.isEnable = 0;
    l3QosCfg.flowId = gL3QosPassFlowId;

    // Call proxy to set L3 QOS
    NetfpProxy_ConfigureL3QosReq l3Req;
    strncpy(l3Req.interfaceName, (char *) req->interface_name, sizeof(l3Req.interfaceName) - 1);
    memcpy(&l3Req.l3QoSCfg, &l3QosCfg, sizeof(l3QosCfg));

    rc = NetfpProxy_executeCommand(NETFP_PROXY_CMDTYPE_CONFIGURE_L3_QOS_REQ, reqMsg->netlinkHdr.nlmsg_seq, (void *) &l3Req, 0, 0);
    // Interface may have been deleted, so, ignore errors, proceed to delete the interface in proxy.
    if(rc != NETFP_PROXY_RETVAL_SUCCESS) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_INFO,
            "INFO: DelL3QosReq: config L3 QoS didn't work, interface already deleted? %d\n", rc);
    }

    // Delete the interface
    NetfpProxy_InterfaceDelReq delIntfReq;
    strncpy(delIntfReq.interfaceName, (char *) req->interface_name, sizeof(delIntfReq.interfaceName) - 1);
    rc = NetfpProxy_executeCommand(NETFP_PROXY_CMDTYPE_DEL_INTERFACE_REQ, reqMsg->netlinkHdr.nlmsg_seq, (void *)&delIntfReq, 0, 0);
    if(rc != NETFP_PROXY_RETVAL_SUCCESS) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: DeleteL3QosReq: delete intf error return\n");
        goto send_response;
    }

    status = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    // TBD : should we delete the interface?

send_response:
    del3qos_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_CONFIG_L3QOS_RESP;
    del3qos_resp.status = status;

    NetfpProxy_SendResponse(&del3qos_resp, sizeof(NetfpProxy_ConfigL3QosResp), reqMsg, addrData, addrLen);

    return status;
}

int32_t NetfpProxy_ConfigDscpPcpMapping_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t status = NETFP_PROXY_IPC_RETVAL_ERROR;
    int rc = 0;
    NetfpProxy_ConfigureDscpPcpMappingReq *req = &(reqMsg->msg.conf_dscp_pcp_mapping_req);
    NetfpProxy_ConfigureDscpPcpMappingResp conf_dscp_pcp_mapping_resp;
    NetfpProxy_FlushVlanPriorityReq FlushVlanPriortyReq;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "ConfigDscpPcpMapping: interfaceName %s", (char *) req->interface_name);

    if(msglen < (sizeof(NetfpProxy_ConfigureDscpPcpMappingReq) + sizeof(struct nlmsghdr)) || strcmp((char *) req->interface_name, "") == 0) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigDscpPcpMapping: invalid received msg length %d or null intf\n", msglen);
        goto send_response;
    }

    /* Copy the interface name to the message and then call the command handler */
    strncpy(FlushVlanPriortyReq.interfaceName, (char *) req->interface_name, sizeof(FlushVlanPriortyReq.interfaceName) - 1);

    rc = NetfpProxy_executeCommand(NETFP_PROXY_CMDTYPE_FLUSH_VLAN_PRIORITY_REQ, reqMsg->netlinkHdr.nlmsg_seq,
        (void *) &FlushVlanPriortyReq, 0, 0);

    /* Report the errors if there were any. */
    if(rc != NETFP_PROXY_RETVAL_SUCCESS) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: ConfigDscpPcpMapping: Flush VLan Priority return error %d\n", rc);
    }
    else {
        status = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

send_response:
    conf_dscp_pcp_mapping_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_DSCP_PCP_MAPPING_RESP;
    conf_dscp_pcp_mapping_resp.status = status;

    NetfpProxy_SendResponse(&conf_dscp_pcp_mapping_resp, sizeof(NetfpProxy_ConfigureDscpPcpMappingResp), reqMsg, addrData, addrLen);

    return status;
}

int32_t NetfpProxy_Hearbeat_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t status = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    NetfpProxy_HeartbeatResp heartbeat_resp;

    if(msglen < sizeof(NetfpProxy_HeartbeatReq) + sizeof(struct nlmsghdr)) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Hearbeat: invalid received msg length %d\n", msglen);
        status = NETFP_PROXY_IPC_RETVAL_ERROR;
    }

    if(criticalErrorOccurred)
        status = NETFP_PROXY_IPC_RETVAL_ERROR;

    heartbeat_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_HEARTBEAT_RESP;
    heartbeat_resp.status = status;

    NetfpProxy_SendResponse(&heartbeat_resp, sizeof(NetfpProxy_HeartbeatResp), reqMsg, addrData, addrLen);

    return status;
}

int32_t NetfpProxy_StopSA_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    int32_t status = NETFP_PROXY_IPC_RETVAL_ERROR;
    NetfpPlugin_SAListNode* saListNode;
    NetfpProxy_StopSaReq *stop_sa_req = &(reqMsg->msg.stop_sa_req);
    NetfpProxy_StopSaResp stop_sa_resp;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "stopSa Req: saHandle:0x%08X", stop_sa_req->sa_handle);

    if(msglen < sizeof(NetfpProxy_HeartbeatReq) + sizeof(struct nlmsghdr)) {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: StopSa: invalid received msg length %d\n", msglen);
        goto send_response;
    }

    /* Check to make sure the SA handler is present in our lists.  This will also check the case
       that the SA is NULL since that should never be in the list. */
    if(NULL == (saListNode = NetfpPlugin_SAListFindBySAHandle((Netfp_SAHandle) stop_sa_req->sa_handle)))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: StopSa: SA Handle was not found on the list 0x%08X\n", stop_sa_req->sa_handle);
        goto send_response;
    }

    if(Netfp_stopSA(gNetfpProxyMcb.netfpClientHandle, saListNode->saHandle, &status) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: StopSa: Netfp_stopSA failed with error code %d\n", status);
        NetfpProxy_assertCriticalError(status, __func__, __LINE__);
        status = NETFP_PROXY_IPC_RETVAL_ERROR;
    } else {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: StopSa: Netfp_stopSA returned success\n");
        status = NETFP_PROXY_IPC_RETVAL_SUCCESS;
    }

send_response:
    stop_sa_resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_STOP_SA_RESP;
    stop_sa_resp.status = status;

    NetfpProxy_SendResponse(&stop_sa_resp, sizeof(NetfpProxy_StopSaResp), reqMsg, addrData, addrLen);

    return status;
}

/**
 *  @b Description
 *  @n
 *      Plugin exit function.
 *
 *  @retval
 *      Not applicable
 */
void my_exit (void)
{
    pthread_cancel(NetlinkMsgThread);
    pthread_join(NetlinkMsgThread, NULL);

    pthread_cancel(NeighborPingThread);
    pthread_join(NeighborPingThread, NULL);

    /* Close the IPC socket */
    if(NetlinkProxySockFd  >= 0)
        close (NetlinkProxySockFd);

    /* Close the NETLINK sockets */
    if(NetlinkRouteSockFd  >= 0)
        close (NetlinkRouteSockFd);

    if(NetlinkNeighPingSockFd  >= 0)
        close (NetlinkNeighPingSockFd);

    if(NetlinkRouteFlushSockFd  >= 0)
        close (NetlinkRouteFlushSockFd);

    if(NetlinkARPSockFd  >= 0)
        close (NetlinkARPSockFd);
}

void my_report_cmd_response(NetfpProxy_CmdType cmd, uint32_t transId, void* cmdData, void* toAddr, uint32_t toAddrLen)
{
}

void my_report_net_event (int32_t action, NetfpProxy_NetEventType eventType, void* eventData)
{
}

#if 0 //deprecated
static int32_t NetfpProxy_isValidFPInterface (char* ifName)
{
    int32_t                 i, isValid = 0;
    uint32_t                *ipAddrList, numEntries = 0;
    struct rtnl_addr*       nlIPAddrInfo;
    struct nl_addr*         nlIPAddr;

    /* Get all the IPv4/IPv6 addresses configured on the interface */
    if (netmgr_addr_get (gRouteMgmtMCB.netMgrRouteHandle, ifName, &ipAddrList, &numEntries) < 0)
        return 0;

    /* Parse through IP address list to see if the interface has an
     * IPv4 address configured on it. */
    for (i = 0; i < numEntries; i ++)
    {
        nlIPAddrInfo    =   (struct rtnl_addr*)ipAddrList[i];
        nlIPAddr        =   rtnl_addr_get_local (nlIPAddrInfo);

        if (nl_addr_get_family (nlIPAddr) == AF_INET)
        {
            /* IPv4 address found. Valid interface. */
            isValid = 1;
            break;
        }
        else
        {
            /* IPv6 address found. Continue searching for IPv4 address */
            continue;
        }
    }

    /* Free LIBNL IP address list allocated */
    for (i = 0; i < numEntries; i ++)
        nl_object_free ((struct nl_object *)ipAddrList[i]);
    free (ipAddrList);

    return isValid;
}
#endif

int32_t NetfpProxy_registerPlugin (NetfpProxy_PluginCfg* pluginCfg)
{
    NetfpProxy_PluginCfg*       pluginMcb   =   &gNetfpProxyMcb.pluginCfg;

    /* Check if all callback functions are specified. */
    if ((pluginCfg->report_cmd_response == NULL) ||
        (pluginCfg->run == NULL) ||
        (pluginCfg->logMsg == NULL) ||
        (pluginCfg->exit == NULL))
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Save the plugin's callback functions */
    pluginMcb->report_cmd_response  =   pluginCfg->report_cmd_response;
    pluginMcb->report_net_event     =   pluginCfg->report_net_event;
    pluginMcb->run                  =   pluginCfg->run;
    pluginMcb->logMsg               =   pluginCfg->logMsg;
    pluginMcb->exit                 =   pluginCfg->exit;

    /* Registration done. Return success */
    return NETFP_PROXY_RETVAL_SUCCESS;
}

void hexDump (void *addr, int len)
{
    int i;
    char buf[128];
    char textbuf[17];
    unsigned char *pc = addr;

    if((addr == NULL) || (len == 0))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Nothing to Dump");
        return;
    }

    // Process every byte in the data.
    for (i = 0; i < len; i++)
    {
        // Multiple of 16 means new line (with line offset).
        if ((i % 16) == 0)
        {
            if (i != 0)
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, " %04X  %s '%s'\n", i-16, buf, textbuf);
            buf[0] = '\0';
            textbuf[0] = '\0';
        }
        // Now the hex code for the specific character and the character
        sprintf(buf,"%s%02X ",buf,pc[i]);
        if((pc[i] < ' ') || (pc[i] > '~'))
            textbuf[i%16] = '.';
        else
            textbuf[i%16] = pc[i];
        textbuf[(i%16) + 1] = '\0';
    }
    // the last bit of data will never be printed.  update the starting offset to be the
    // begining of the 16-byte stretch.
    if(i%16)
        i -= (i%16);
    else
        i -= 16;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, " %04X  %s '%s'\n", i, buf, textbuf);
}

int Plugin_arping(const char *if_name, struct sockaddr *dst_addr, int family)
{
    struct {
        struct nlmsghdr nlh;
        struct ndmsg ndh;
        uint8_t attrbuf[64];
    } req;
    struct rtattr *rtap;
    int rtl = 0;
    int msg_len = 0;
    uint16_t ndm_state = NUD_NONE;
    uint8_t ndm_flags = NTF_USE;
    uint32_t if_index = if_nametoindex(if_name);

    if(NetlinkARPSockFd == -1)
        NetlinkARPSockFd = socket(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE);

    if(NetlinkARPSockFd == -1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "%s: Failed to create socked: %s (%d)\n",
                           __func__,
                           strerror(errno),
                           errno);
        return -1;
    }

    memset(&req, 0, sizeof(req));

    req.nlh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ndmsg));
    req.nlh.nlmsg_type = RTM_NEWNEIGH;
    req.nlh.nlmsg_flags = NLM_F_REQUEST | NLM_F_CREATE;

    req.ndh.ndm_family = family;
    req.ndh.ndm_ifindex = if_index;
    req.ndh.ndm_state = ndm_state;
    req.ndh.ndm_flags = ndm_flags;

    rtap = (struct rtattr *)(((char *)NLMSG_DATA(&req)) + NLMSG_ALIGN(sizeof(struct ndmsg)));
    rtap->rta_type = NDA_DST;
    if (AF_INET == family)
    {
        struct sockaddr_in *daddr = (struct sockaddr_in*)dst_addr;
        rtap->rta_len = RTA_LENGTH(sizeof(uint32_t));
        *(uint32_t *)RTA_DATA(rtap) = daddr->sin_addr.s_addr;
    }
    else
    {
        struct sockaddr_in6 *daddr = (struct sockaddr_in6*)dst_addr;
        rtap->rta_len = RTA_LENGTH(sizeof(struct in6_addr));
        *(struct in6_addr *)RTA_DATA(rtap) = daddr->sin6_addr;
    }
    req.nlh.nlmsg_len = RTA_ALIGN(req.nlh.nlmsg_len) + rtap->rta_len;

    rtap = RTA_NEXT(rtap, rtl);
    rtap->rta_type = NDA_LLADDR;
    rtap->rta_len = RTA_LENGTH(ETH_ALEN);
    memset(RTA_DATA(rtap), 0, ETH_ALEN);
    req.nlh.nlmsg_len = RTA_ALIGN(req.nlh.nlmsg_len) + rtap->rta_len;

    msg_len = send(NetlinkARPSockFd, &req, req.nlh.nlmsg_len, 0);
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

void* Plugin_NeighborPingThread(void* arg)
{
    (void)arg;
    NetfpProxy_msg pingMsg;
    uint32_t msgLen = 0;

    /* Open a NETLINK socket */
    if ((NetlinkNeighPingSockFd = socket (AF_NETLINK, SOCK_RAW, NETLINK_USERSOCK)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "ERROR: Socket open failed, error: %d \n", errno);
        return NULL;
    }

    memset ((void *)&NetlinkNeighPingAddr, 0, sizeof (struct sockaddr_nl));

    /* Bind the socket to our unix path name */
    NetlinkNeighPingAddr.nl_family    =   AF_NETLINK;
    NetlinkNeighPingAddr.nl_groups    =   0;

    if (bind (NetlinkNeighPingSockFd, (const struct sockaddr *)&NetlinkNeighPingAddr, sizeof (struct sockaddr_nl)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Socket bind failed, error: %d \n", errno);
        return NULL;
    }
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Plugin Neighbor ping registered\n");

    msgLen = sizeof(struct nlmsghdr) + sizeof(struct NetfpProxy_MsgHdr);

    pingMsg.netlinkHdr.nlmsg_len = msgLen;
    pingMsg.netlinkHdr.nlmsg_type = NETFP_PROXY_IPC_MSGTYPE_NEIGHBOR_PING_REQ;
    pingMsg.netlinkHdr.nlmsg_flags = 0;
    pingMsg.netlinkHdr.nlmsg_seq = 0;
    pingMsg.netlinkHdr.nlmsg_pid = 0;
    pingMsg.msg.hdr_only.msgType = NETFP_PROXY_IPC_MSGTYPE_NEIGHBOR_PING_REQ;

#define NEIGHBOR_CHECK_INTERVAL_SEC 10

    while(1)
    {
        sleep(NEIGHBOR_CHECK_INTERVAL_SEC);
        if (sendto (NetlinkNeighPingSockFd, &pingMsg, msgLen, 0, (const struct sockaddr *)&NetlinkProxyAddr, sizeof(struct sockaddr_nl)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Failed sending neighbor ping request\n");
        }
    }

    return NULL;
}

int32_t NetfpProxy_NeighborPing_CmdHandler(void)
{
    union Plugin_Address
    {
        struct sockaddr_in ipv4;
        struct sockaddr_in6 ipv6;
    };

    NetfpProxy_Route* ptrProxyRoute = (NetfpProxy_Route*)List_getHead ((List_Node**)&gRouteMgmtMCB.ptrRouteCacheList);
    while (ptrProxyRoute != NULL)
    {
        union Plugin_Address addr;
        memset(&addr, 0, sizeof(union Plugin_Address));
        socklen_t addrLen = sizeof(addr);
        nl_addr_fill_sockaddr(ptrProxyRoute->nhIP, (struct sockaddr *)&addr, &addrLen);
        Plugin_arping(ptrProxyRoute->ifName, (struct sockaddr *)&addr, nl_addr_get_family(ptrProxyRoute->nhIP));
        ptrProxyRoute = (NetfpProxy_Route*)List_getNext ((List_Node*)ptrProxyRoute);
    }
    return 0;
}

int32_t NetfpProxy_RouteFlush_CmdHandler(void)
{
    /* Set the flag to indicate flush request */
    gRouteMgmtMCB.isFlushPending = 1;
    return 0;
}

int32_t NetfpProxy_AddInterface_CmdHandler(NetfpProxy_msg *reqMsg, struct sockaddr_nl *addrData, uint32_t addrLen, uint32_t msglen)
{
    NetfpProxy_AddInterfaceResp resp;
    resp.msg_hdr.msgType = NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP;
    resp.status = NETFP_PROXY_IPC_RETVAL_SUCCESS;

    do { /* handle errors as break */
        if (msglen < sizeof(NetfpProxy_AddInterfaceReq)+sizeof(struct nlmsghdr)) {
            NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "ERROR: Incomplete NetfpProxy_AddInterfaceReq message");
            resp.status = NETFP_PROXY_IPC_RETVAL_INV_PARAMS;
            break;
        }

        NetfpProxy_AddInterfaceReq* req = &(reqMsg->msg.add_iface_req);

        NetfpProxy_InterfaceAddReq addIntfReq;
        strncpy(addIntfReq.interfaceName, (char *)req->interface_name, sizeof(addIntfReq.interfaceName) - 1);
        addIntfReq.interfaceName[sizeof(addIntfReq.interfaceName)-1] = '\0';

        int rc = NetfpProxy_executeCommand(
            NETFP_PROXY_CMDTYPE_ADD_INTERFACE_REQ, reqMsg->netlinkHdr.nlmsg_seq, &addIntfReq, 0, 0);
        if (rc != NETFP_PROXY_RETVAL_SUCCESS) {
            NetfpProxy_logMsg(NETFP_PROXY_LOG_ERROR, "ERROR: NetfpProxy_AddInterface: error: rc=%d\n", rc);
            resp.status = NETFP_PROXY_IPC_RETVAL_ERROR;
            break;
        }

    } while (0);

    NetfpProxy_SendResponse(&resp, sizeof(resp), reqMsg, addrData, addrLen);

    return resp.status;
}

/**
 *  @b Description
 *  @n
 *      Polls to monitor Route change notification
 *
 *  @retval
 *      Not applicable
 */

void* Plugin_NetlinkMsgThread(void* arg)
{
    (void)arg;
    int32_t         numReadFds = 0;
    fd_set          readFdList;
    /* Jira 1401: Plugin_NetlinkMsgThread uses 100%
    struct timeval  timeout;
    */
    int32_t received_bytes = 0;
    uint32_t msgLen;
    struct  nlmsghdr *nlh;
    char    rxMsg[1024];
    NetfpProxy_msg flushMsg;

    /* Open a NETLINK sockets */
    if ((NetlinkRouteSockFd = socket (AF_NETLINK, SOCK_RAW, NETLINK_ROUTE)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "ERROR: Socket open failed, error: %d \n", errno);
        return NULL;
    }
    if ((NetlinkRouteFlushSockFd = socket (AF_NETLINK, SOCK_RAW, NETLINK_USERSOCK)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "ERROR: Socket open failed, error: %d \n", errno);
        return NULL;
    }


    memset ((void *)&NetlinkRouteSunAddr, 0, sizeof (struct sockaddr_nl));
    memset ((void *)&NetlinkRouteFlushAddr, 0, sizeof (struct sockaddr_nl));

    /* Bind the socket to our unix path name */
    NetlinkRouteSunAddr.nl_family    =   AF_NETLINK;
    NetlinkRouteSunAddr.nl_groups    =   RTMGRP_IPV4_ROUTE| RTMGRP_IPV4_RULE| RTMGRP_IPV6_ROUTE;

    NetlinkRouteFlushAddr.nl_family    =   AF_NETLINK;
    NetlinkRouteFlushAddr.nl_groups    =   0;


    if (bind (NetlinkRouteSockFd, (const struct sockaddr *)&NetlinkRouteSunAddr, sizeof (struct sockaddr_nl)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Socket 1 bind failed, error: %d \n", errno);
        return NULL;
    }

    if (bind (NetlinkRouteFlushSockFd, (const struct sockaddr *)&NetlinkRouteFlushAddr, sizeof (struct sockaddr_nl)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Socket 2 bind failed, error: %d \n", errno);
        return NULL;
    }

    numReadFds = NetlinkRouteSockFd + 1;

    /* Set the timeout to zero: Return immediately if there is no command message
     * from user application */
    /* Jira 1401: Plugin_NetlinkMsgThread uses 100%
    timeout.tv_sec  =   0;
    timeout.tv_usec =   0;
    */

    msgLen = sizeof(struct nlmsghdr) + sizeof(struct NetfpProxy_MsgHdr);

    flushMsg.netlinkHdr.nlmsg_len = msgLen;
    flushMsg.netlinkHdr.nlmsg_type = NETFP_PROXY_IPC_MSGTYPE_ROUTE_FLUSH_REQ;
    flushMsg.netlinkHdr.nlmsg_flags = 0;
    flushMsg.netlinkHdr.nlmsg_seq = 0;
    flushMsg.netlinkHdr.nlmsg_pid = 0;
    flushMsg.msg.hdr_only.msgType = NETFP_PROXY_IPC_MSGTYPE_ROUTE_FLUSH_REQ;

    while(1)
    {
	/* Setup the receive socket that needs to be monitored */
	FD_ZERO (&readFdList);
	FD_SET (NetlinkRouteSockFd, &readFdList);
	/* Check if we have recieved any messages on the NETLINK socket */
        /* Jira 1401: Plugin_NetlinkMsgThread uses 100%.  Use NULL for the time-out
           so it will block until something is received rather than return right away.
        if ((select (numReadFds, &readFdList, NULL, NULL, &timeout)) > 0)
        */
        if ((select (numReadFds, &readFdList, NULL, NULL, NULL)) > 0)
        {
            if (FD_ISSET (NetlinkRouteSockFd, &readFdList))
            {
		    if((received_bytes = recv(NetlinkRouteSockFd, rxMsg, sizeof(rxMsg), 0)) > 0)
		    {
			    /* cast the received buffer to netlink message type*/
			    nlh = (struct nlmsghdr *) rxMsg;

			    for ( ; NLMSG_OK(nlh, received_bytes); \
					    nlh = NLMSG_NEXT(nlh, received_bytes))
			    {
				    /* Check for netlink route update notification */
				    if (nlh->nlmsg_type == RTM_NEWROUTE || nlh->nlmsg_type == RTM_DELROUTE || \
						nlh->nlmsg_type == RTM_NEWRULE || nlh->nlmsg_type == RTM_DELRULE)
                    {
                        if (sendto (NetlinkRouteFlushSockFd, &flushMsg, msgLen, 0, (const struct sockaddr *)&NetlinkProxyAddr, sizeof(struct sockaddr_nl)) < 0)
                        {
                            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Failed sending neighbor flush request\n");
                        }

                    }

			    }

		    }
	    }
	}
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Plugin registration function.
 *
 *  @param[in]  instanceId
 *      Netfp Proxy instance Id that this plugin is
 *      loaded to run with
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   >0
 */
int32_t NetfpProxy_pluginInit (int32_t instanceId)
{
    NetfpProxy_PluginCfg        pluginCfg;
    int                         retVal;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,  "DEBUG: Registered plugin for Proxy PID %d \n", instanceId);

    /* Open a Unix socket to talk to User application */
    if((NetlinkProxySockFd = socket (AF_NETLINK, SOCK_RAW, NETLINK_USERSOCK)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR,  "ERROR: Socket open failed, error: %d \n", errno);
        my_exit();
        return -1;
    }

    /* Bind to the socket to listen for messages on the specific Netlink PID. */
    memset(&NetlinkProxyAddr, 0, sizeof(struct sockaddr_nl));
    NetlinkProxyAddr.nl_family = AF_NETLINK;
    NetlinkProxyAddr.nl_pid = NETFP_PROXY_NETLINK_PID;
    NetlinkProxyAddr.nl_groups = 0;

    if(bind(NetlinkProxySockFd, (const struct sockaddr *)&NetlinkProxyAddr, sizeof (struct sockaddr_nl)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Socket bind failed, error: %d \n", errno);
        my_exit ();
        return -1;
    }

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG,  "DEBUG: Plugin for Proxy listening on Netlink PID %d\n", NetlinkProxyAddr.nl_pid);

    /* Register our callbacks with NetFP Proxy daemon */
    memset ((void*)&pluginCfg, 0, sizeof (NetfpProxy_PluginCfg));

    /* Initialize interfaces ONLY at offload time. Setup NO interfaces at init time. */
    pluginCfg.numFpInterfaces       =   0;

    /* Setup plugin callback functions */
    pluginCfg.report_cmd_response   =   my_report_cmd_response;
    pluginCfg.report_net_event      =   my_report_net_event;
    pluginCfg.run                   =   my_netlink_poll;
    pluginCfg.logMsg                =   NetfpProxy_logMsgPlug;
    pluginCfg.exit                  =   my_exit;
    if (NetfpProxy_registerPlugin (&pluginCfg) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Plugin registration failed\n");
        my_exit ();
        return -1;
    }

    // fzm: Get L3 QOS mapping queues
    populateL3QosChannelTable();

    netmgr_nl_update_neigh = &Plugin_arping;
    NetfpProxy_neighPluginARPing = &Plugin_arping;

    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Plugin registration succeeded\n");

    retVal = pthread_create (&NetlinkMsgThread, NULL, Plugin_NetlinkMsgThread, NULL);
    if (retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NetlinkMsgThread thread create failed error code %d\n", retVal);
        my_exit ();
        return -1;
    }

    retVal = pthread_create (&NeighborPingThread, NULL, Plugin_NeighborPingThread, NULL);
    if (retVal < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error: NeighborPingThread thread create failed error code %d\n", retVal);
        my_exit ();
        return -1;
    }

    /* Return success */
    return NetlinkProxySockFd;
}
