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
#ifndef __NETFP_PROXY_IPC_H__
#define __NETFP_PROXY_IPC_H__

#include <stdint.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/netfp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   NetFP Proxy IPC return codes.
 */
#define NETFP_PROXY_IPC_RETVAL_SUCCESS                  0
#define NETFP_PROXY_IPC_RETVAL_ERROR                    -1
#define NETFP_PROXY_IPC_RETVAL_INV_PARAMS               -2

/**
 * @brief   NetFP Proxy default daemon socket path.
 */
#define NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH             "netfp_proxy_ipc_daemon.sock"

/**
 * @brief   NetFP Proxy default user application socket path.
 */
#define NETFP_PROXY_IPC_DEFAULT_USER_PATH               "netfp_proxy_ipc_user.sock"

/**
 * @brief   NetFP Proxy Transaction Id type
 */
typedef uint32_t        NetfpProxy_TransId;

/**
 * @brief   NetFP Proxy Policy Id
 */
typedef uint32_t        NetfpProxy_PolicyId;

/**
 * @brief
 *  Message Type
 *
 * @details
 *  Used to indicate the type of message being exchanged between
 *  the application and NetFP Proxy daemon.
 */
typedef enum NetfpProxy_MsgType
{
    NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ     = 1,
    NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP,
    NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_REQ,
    NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP,
    NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ,
    NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_RESP,
    NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ,
    NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP,
    NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_REQ,
    NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_RESP,
    NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_REQ,
    NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_RESP,
    NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_REQ,
    NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_RESP,
    NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_REQ,
    NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_RESP,
    NETFP_PROXY_IPC_MSGTYPE_MAX
} NetfpProxy_MsgType;

/**
 * @brief
 *  DSCP Policy type
 *
 * @details
 *  Indicates the possible actions that can be
 *  taken for egress policies in terms of DSCP
 *  processing.
 */
typedef enum NetfpProxy_DscpPolicy
{
    /**
     * @brief
     *  Copy DSCP from Inner IP header to Outer
     *  IP header as is.
     */
    NETFP_PROXY_DSCP_COPY_FROM_INNER        =       1,

    /**
     * @brief
     *  Use a fixed value for Outer IP header
     *  DSCP
     */
    NETFP_PROXY_DSCP_USE_FIXED_VAL          =       2,

    /**
     * @brief
     *  Use a mapping table to determine the Outer
     *  IP DSCP value
     */
    NETFP_PROXY_DSCP_USE_MAP_TABLE          =       3,
} NetfpProxy_DscpPolicy;

/**
 * @brief
 *  DSCP configuration. Indicates the mapping between
 *  Inner IP DSCP value to Outer IP DSCP value.
 */
typedef struct NetfpProxy_DscpCfg
{
    /**
     * @brief
     *  Type of DSCP processing to be done.
     *  If set to zero, by default policy is
     *  assumed to be NETFP_PROXY_DSCP_COPY_FROM_INNER.
     */
    NetfpProxy_DscpPolicy   policy;

    /**
     * @brief
     *  DSCP value to use
     */
    union {
    /**
     * @brief
     *  Fixed DSCP value to use when policy set to
     *  NETFP_PROXY_DSCP_USE_FIXED_VAL.
     */
    uint8_t                 fixedVal;

    /**
     * @brief
     *  DSCP mapping table. Used if policy set to
     *  NETFP_PROXY_DSCP_USE_MAP_TABLE.
     */
    uint8_t                 mapTable[64];
    } value;
} NetfpProxy_DscpCfg;

/**
 * @brief
 *  GTPU Id configuration for the policy.
 */
typedef struct NetfpProxy_GtpuIdCfg
{
    /**
     * @brief
     *  GTPU ID Range start. If set to zero,
     *  the range is set to ANY.
     */
    uint32_t                min;

    /**
     * @brief
     *  GTPU ID Range end
     */
    uint32_t                max;
} NetfpProxy_GtpuIdCfg;

/**
 * @brief
 *  IPSec Fragmentation levels
 *
 * @details
 *  Applicable only for egress policies.
 *
 *  Indicates on IPSec Netfp Tx path if fragmentation
 *  must be done on inner IP header or outer IP
 *  header.
 */
typedef enum NetfpProxy_IPSecFragLevel
{
    /**
     * @brief
     *  Fragment on inner IP. Packets are fragmented
     *  before encryption in Netfp S/w.
     */
    NETFP_PROXY_IPSEC_FRAG_INNER_IP =   0,

    /**
     * @brief
     *  Fragment on outer IP. Packets are fragmented
     *  after encryption in NetCP H/w.
     */
    NETFP_PROXY_IPSEC_FRAG_OUTER_IP =   1,
} NetfpProxy_IPSecFragLevel;

/**
 * @brief
 *  Offload Security Policy (SP) Request configuration.
 *
 *  Configuration to be passed with
 *  NETFP_PROXY_CMDTYPE_START_OFFLOAD_SP_REQ command
 *  from the application plugin.
 */
typedef struct NetfpProxy_OffloadSpReq
{
    /**
     * @brief
     *  Security policy ID to offload to Fast Path.
     */
    NetfpProxy_PolicyId     policyId;

    /**
     * @brief
     *  DSCP configuration (valid for Egress policies only)
     */
    NetfpProxy_DscpCfg      dscpCfg;

    /**
     * @brief
     *  Range of allowed GTPU Ids for this policy
     */
    NetfpProxy_GtpuIdCfg    gtpuIdRange;

    /**
     * @brief
     *  Boolean flag to indicate if this policy
     *  is to be shared between ARM and DSP.
     *
     *  If this is set to 1, the Proxy ensures that
     *  a corresponding context is created both in
     *  NETFP and Linux for this so that both the
     *  stacks can accept and process matching this
     *  policy.
     */
    uint32_t                bIsSharedPolicy;

    /**
     * @brief
     *  Boolean flag to indicate if this policy
     *  is a secure or a non-secure policy.
     */
    uint32_t                bIsSecure;

    /**
     * @brief
     *  Fragmentation configuration for IPSec.
     *
     *  Valid only for Egress Secure policies.
     */
    NetfpProxy_IPSecFragLevel   fragLevel;
} NetfpProxy_OffloadSpReq;

/**
 * @brief
 *  Offload Security Policy (SP) Response format
 */
typedef struct NetfpProxy_OffloadSpResp
{
    /**
     * @brief
     *  Security policy ID corresponding to the
     *  response.
     */
    NetfpProxy_PolicyId     policyId;

    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    uint32_t                netfpErrCode;

    /**
     * @brief
     *  Security association handle returned
     *  by NetFP library
     */
    uint32_t                netfpSaHandle;
} NetfpProxy_OffloadSpResp;

/**
 * @brief
 *  IP Route flush response format
 */
typedef struct NetfpProxy_IpRouteFlushResp
{
    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
} NetfpProxy_IpRouteFlushResp;

/**
 * @brief
 *  L3 QoS Shaping configuration
 */
typedef struct NetfpProxy_L3QoSCfg
{
    /**
     * @brief   L3 QoS configuration status. The configuration fields are NOT
     * valid if the flag is set to 0.
     */
    int32_t     isQosEnable;

    /**
     * @brief   Flow identifier to be used for the L3 shaping. The flow is used by
     * the L3 QOS block to receive packets into the QOS engine. These packets are
     * then shaped via the QOS firmware
     */
    int32_t     flowId;

    /**
     * @brief   Inner DSCP is mapped to a specific QOS queue for shaping.
     */
    uint32_t    qid[64];
}NetfpProxy_L3QoSCfg;

/**
 * @brief
 *  Interface add request format
 */
typedef struct NetfpProxy_InterfaceAddReq
{
    /**
     * @brief   Name of the interface to add
     */
    char                    interfaceName[32];
}NetfpProxy_InterfaceAddReq;

/**
 * @brief
 *  Interface add response format
 */
typedef struct NetfpProxy_InterfaceAddResp
{
    /**
     * @brief   Name of the interface added
     */
    char                    interfaceName[32];

    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
}NetfpProxy_InterfaceAddResp;

/**
 * @brief
 *  Interface delete request format
 */
typedef struct NetfpProxy_InterfaceDelReq
{
    /* @brief
     *  Interface name to delete
     */
    char                    interfaceName[32];
}NetfpProxy_InterfaceDelReq;

/**
 * @brief
 *  Interface delete response format
 */
typedef struct NetfpProxy_InterfaceDelResp
{
    /**
     * @brief   Name of the interface deleted
     */
    char                    interfaceName[32];

    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
}NetfpProxy_InterfaceDelResp;

/**
 * @brief
 *  Configure QoS request format
 */
typedef struct NetfpProxy_ConfigureL3QosReq
{
    /**
     * @brief   Name of the interface to configure
     */
    char                    interfaceName[32];

    /**
     * @brief   L3 QoS configuration for the interface
     */
    NetfpProxy_L3QoSCfg     l3QoSCfg;
}NetfpProxy_ConfigureL3QosReq;

/**
 * @brief
 *  Configure QoS response format
 */
typedef struct NetfpProxy_ConfigureL3QosResp
{
    /**
     * @brief   Name of the interface configured
     */
    char                    interfaceName[32];

    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
}NetfpProxy_ConfigureL3QosResp;

/**
 * @brief
 *  Flush VLAN Egress Priority Map request format
 *  The structure is used to specify the VLAN interface whose Egress Priority Mappings
 *  have to be read
 */
typedef struct NetfpProxy_FlushVlanPriorityReq
{
    /**
     * @brief   Name of the VLAN interface whose Egress Priority Mapping has to be read
     */
    char                    interfaceName[32];
}NetfpProxy_FlushVlanPriorityReq;

/**
 * @brief
 *  Flush VLAN Egress Priority Map response format
 */
typedef struct NetfpProxy_FlushVlanPriorityResp
{
    /**
     * @brief   Name of the VLAN interface whose Egress Priority Mapping was read
     */
    char                    interfaceName[32];

    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
}NetfpProxy_FlushVlanPriorityResp;

/**
 * @brief
 *  This is the structure which is populated to setup the PMTU
 */
typedef struct NetfpProxy_SetupPMTUReq
{
    /**
     * @brief
     *  Source IP address
     */
    Netfp_IPAddr        srcIP;

    /**
     * @brief
     *  Destination IP address
     */
    Netfp_IPAddr        dstIP;

    /**
     * @brief
     *  PMTU to be used
     */
    uint32_t            pmtu;
}NetfpProxy_SetupPMTUReq;

/**
 * @brief
 *  This is the structure which is populated in response to the
 *  setup PMTU request
 */
typedef struct NetfpProxy_SetupPMTUResp
{
    /**
     * @brief
     *  Error code of type NETFP_PROXY_IPC_RETVAL_
     */
    int32_t                 retVal;

    /**
     * @brief
     *  Error code returned by NetFP library
     */
    int32_t                 netfpErrCode;
}NetfpProxy_SetupPMTUResp;

/**
 * @brief
 *  Message Header format
 */
typedef struct NetfpProxy_MsgHdr
{
    /**
     * @brief
     *  Message type.
     */
    NetfpProxy_MsgType      msgType;

    /**
     * @brief
     *  Transaction Id for this request.
     */
    NetfpProxy_TransId      transId;
} NetfpProxy_MsgHdr;

/**
 * @brief
 *  Message format
 */
typedef struct NetfpProxy_msg
{
    /**
     * @brief
     *  Message header.
     */
    NetfpProxy_MsgHdr      hdr;

    /**
     * @brief
     *  Message body.
     */
    union
    {
        NetfpProxy_OffloadSpReq             spReq;
        NetfpProxy_OffloadSpResp            spResp;
        NetfpProxy_IpRouteFlushResp         ipRouteFlushResp;
        NetfpProxy_InterfaceAddReq          ifaceAddReq;
        NetfpProxy_InterfaceAddResp         ifaceAddResp;
        NetfpProxy_InterfaceDelReq          ifaceDelReq;
        NetfpProxy_InterfaceDelResp         ifaceDelResp;
        NetfpProxy_ConfigureL3QosReq        configL3QosReq;
        NetfpProxy_ConfigureL3QosResp       configL3QosResp;
        NetfpProxy_FlushVlanPriorityReq     flushVlanPriorityReq;
        NetfpProxy_FlushVlanPriorityResp    flushVlanPriorityResp;
        NetfpProxy_SetupPMTUReq             setupPMTUReq;
        NetfpProxy_SetupPMTUResp            setupPMTUResp;
    } body;
} NetfpProxy_msg;

#ifdef __cplusplus
}
#endif

#endif
