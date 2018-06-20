/**
 *******************************************************************
 *	@file		netfp_proxy_xfrm_bridge_interface.h
 *	@date		18, May 2015
 *	@version	0.6
 *
 *	@change history:
 *      16/02/15       Updated the message parameter name to
 *                     sync up with parameter names in ICD.
 *
 *      19/02/15       Updated following:
 *                     - Removed iface_name element from ADD_SP_REQ Message.
 *                     - Added sa_mode (for inflow/sideband flow support) in ADD_SA_REQ Message.
 *                     - Added Anti-replay window size (arw_size) in NetfpProxy_IPSecCfg  structure which is part of ADD_SA_REQ Message.
 *                     - Regrouped prefix_len and IP Addr into a separate structure NetfpProxy_AddrInfo
 *
 *      20/02/15       Fixed compilation errors.
 *
 *      24/02/15       Renamed NetfpProxy_AddrInfo to NetfpProxy_AddressInfo to avoid internal conflict with Proxy code.
 *                     Corrected NetfpProxy_IpsecAuthMode structure tag name.
 *                     Corrected typo lcvlen to icvlen
 *
 *      18/05/15       Added NetfpProxy_SaHwCtx which was added it to NetfpProxy_AddSAResp
 *
 *      20/05/15       Made updates to for Netlink support.
 *                     Replaced the IPC socket file names with NETFP_PROXY_NETLINK_PID
 *                     Removed the TransID.  Added NETFP_PROXY_IPC_MSGTYPE_MAX.
 *
 *      22/07/15       Made updates to support 2IKE and aligned some other messages with ICD
 *                     - Added CONFIGURE_DSCP_PCP_MAPPING_REQ Message
 *                     - Added CONFIGURE_DSCP_PCP_MAPPING_RESP Message
 *                     - Added HEARTBEAT_REQ Message
 *                     - Added HEARTBEAT_RESP Messages Message
 *                     - Added STOP_SA_REQ Mesage
 *                     - Added STOP_SA_RESP Messsage
 *                     - Added root_sa to ADD_SA_REQ Message
 *                     - Added new_root_sa_handle to DEL_SA_REQ Message
 *                     - Added NETFP_PROXY_SA_FLAGS_XXXX defines.
 *                     - Removed req_id from NetfpProxy_AddSAReq and NetfpProxy_AddSPReq
 *
 *      17/02/17       Added support for LTE1048 (X2 mesh)
 *                     - Added NetfpProxy_OffloadMode enum
 *                     - Added offload_mode field to NetfpProxy_AddSAReq structure
 *
 *  Copyright 2015 Nokia Solutions and Networks.  All rights reserved.
*********************************************************************/

#ifndef __NETFP_PROXY_XFRM_BRIDGE_IFACE_HEADER_H__
#define __NETFP_PROXY_XFRM_BRIDGE_IFACE_HEADER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NetFP Proxy Application Netlink socket PID. */
#define NETFP_PROXY_NETLINK_PID                         110

 /* NetFP Proxy ICD Message return codes.  */
#define NETFP_PROXY_IPC_RETVAL_SUCCESS                   0
#define NETFP_PROXY_IPC_RETVAL_ERROR                    -1
#define NETFP_PROXY_IPC_RETVAL_INV_PARAMS               -2

/* Maximum supported IPSEC key length */
#define NETFP_PROXY_MAX_IPSEC_KEY_LEN                   36

/* The Length (characters + \0) of the interface name */
#define NETFP_PROXY_INTERFACE_NAME_LEN                  32

/* Maximum SA SW Info Size*/
#define NETFP_PROXY_SA_SWINFO_MAX_SZ    3

/* Security association Flags */
#define NETFP_PROXY_SA_FLAGS_ESN                        0x01
#define NETFP_PROXY_SA_FLAGS_SHARED                     0x02
#define NETFP_PROXY_SA_FLAGS_REKEY                      0x04

/********** enum decalaration - start ************/

/* *********************************************************************/
/* NOTE: The NetfpProxy_CmdTypeStr array in netfp_proxy_custom.c must be
 * updated to reflect changes in NetfpProxy_MsgType */
/* *********************************************************************/
/* Message Type */
typedef enum NetfpProxy_MsgType
{
    NETFP_PROXY_IPC_MSGTYPE_ADD_SA_REQ                      =  1,
    NETFP_PROXY_IPC_MSGTYPE_ADD_SA_RESP                     =  2,
    NETFP_PROXY_IPC_MSGTYPE_ADD_SP_REQ                      =  3,
    NETFP_PROXY_IPC_MSGTYPE_ADD_SP_RESP                     =  4,
    NETFP_PROXY_IPC_MSGTYPE_DEL_SA_REQ                      =  5,
    NETFP_PROXY_IPC_MSGTYPE_DEL_SA_RESP                     =  6,
    NETFP_PROXY_IPC_MSGTYPE_DEL_SP_REQ                      =  7,
    NETFP_PROXY_IPC_MSGTYPE_DEL_SP_RESP                     =  8,
    NETFP_PROXY_IPC_MSGTYPE_SA_STATS_REQ                    =  9,
    NETFP_PROXY_IPC_MSGTYPE_SA_STATS_RESP                   = 10,
    NETFP_PROXY_IPC_MSGTYPE_CONFIG_L3QOS_REQ                = 11,
    NETFP_PROXY_IPC_MSGTYPE_CONFIG_L3QOS_RESP               = 12,
    NETFP_PROXY_IPC_MSGTYPE_DEL_L3QOS_REQ                   = 13,
    NETFP_PROXY_IPC_MSGTYPE_DEL_L3QOS_RESP                  = 14,
    NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_DSCP_PCP_MAPPING_REQ  = 15,
    NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_DSCP_PCP_MAPPING_RESP = 16,
    NETFP_PROXY_IPC_MSGTYPE_HEARTBEAT_REQ                   = 17,
    NETFP_PROXY_IPC_MSGTYPE_HEARTBEAT_RESP                  = 18,
    NETFP_PROXY_IPC_MSGTYPE_STOP_SA_REQ                     = 19,
    NETFP_PROXY_IPC_MSGTYPE_STOP_SA_RESP                    = 20,
    NETFP_PROXY_IPC_MSGTYPE_NEIGHBOR_PING_REQ               = 21,
    NETFP_PROXY_IPC_MSGTYPE_ROUTE_FLUSH_REQ                 = 22,
    NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ               = 23,
    NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP              = 24,
    NETFP_PROXY_IPC_MSGTYPE_MAX
} NetfpProxy_MsgType;
/* *********************************************************************/
/* NOTE: The NetfpProxy_CmdTypeStr array in netfp_proxy_custom.c must be
 * updated to reflect changes in NetfpProxy_MsgType */
/* *********************************************************************/


/* SA/SP Direction */
typedef enum NetfpProxy_Direction
{
    NetfpProxy_Direction_INBOUND     = 1,
    NetfpProxy_Direction_OUTBOUND    = 2

}NetfpProxy_Direction;

/* IPSEC protocols which are currently supported by NETFP */
typedef enum NetfpProxy_IPSecProto
{
    NetfpProxy_IPSecProto_IPSEC_ESP = 1

}NetfpProxy_IPSecProto;

/* IPSEC transport modes which are currently supported by NETFP */
typedef enum NetfpProxy_IPSecMode
{
    NetfpProxy_IPSecMode_IPSEC_TUNNEL = 1

}NetfpProxy_IPSecMode;

/* IPSEC Authentication modes currently supported */
typedef enum NetfpProxy_IpsecAuthMode
{
    NetfpProxy_IpsecAuthMode_NULL        = 1,
    NetfpProxy_IpsecAuthMode_HMAC_SHA1   = 2,
    NetfpProxy_IpsecAuthMode_HMAC_MD5    = 3,
    NetfpProxy_IpsecAuthMode_SHA2        = 4,
    NetfpProxy_IpsecAuthMode_GMAC        = 5,
    NetfpProxy_IpsecAuthMode_AES_XCBC    = 6
}NetfpProxy_IpsecAuthMode;

/* IPSEC ciphering modes currently supported */
typedef enum NetfpProxy_IpsecCipherMode
{
    NetfpProxy_IpsecCipherMode_NULL     = 0,
    NetfpProxy_IpsecCipherMode_AES_CTR  = 1,
    NetfpProxy_IpsecCipherMode_AES_CBC  = 2,
    NetfpProxy_IpsecCipherMode_3DES_CBC = 3,
    NetfpProxy_IpsecCipherMode_DES_CBC  = 4,
    NetfpProxy_IpsecCipherMode_GCM      = 5
}NetfpProxy_IpsecCipherMode;

typedef enum NetfpProxy_l5_sel_proto
{
    NETFP_PROXY_L5_PROTO_GTPU  = 1

}NetfpProxy_l5_sel_proto;

typedef enum NetfpProxy_SAMode
{
    NETFP_PROXY_SA_MODE_INFLOW    = 1,
    NETFP_PROXY_SA_MODE_SIDEBAND  = 2

}NetfpProxy_SAMode;

typedef enum NetfpProxy_OffloadMode
{
    NETFP_PROXY_OFFLOAD_MODE_HARDWARE = 0,
    NETFP_PROXY_OFFLOAD_MODE_SOFTWARE = 1
} NetfpProxy_OffloadMode;

/********** enum decalaration - end ************/


/********** Message Elements - Start ************/

/* Message Header format */
typedef struct NetfpProxy_MsgHdr
{
    NetfpProxy_MsgType    msgType;
} __attribute__((packed)) NetfpProxy_MsgHdr;


/* Enumeration which describes the IP version being used. */
typedef enum Netfpproxy_IPVersion
{
    NetfpProxy_IPVersion_IPV4    = 0x1,
    NetfpProxy_IPVersion_IPV6    = 0x2

}NetfpProxy_IPVersion;

/* IPv4 Address represented in network order. */
typedef struct NetfpProxy_IPv4_Addr
{
    union
    {
        uint8_t    a8[4];
        uint32_t   a32;
    }u;

} __attribute__((packed)) NetfpProxy_IPv4_Addr;

/* IPv6 Address represented in network order. */
typedef struct NetfpProxy_IPv6_Addr
{
    union
    {
        uint8_t    a8[16];
        uint16_t   a16[8];
        uint32_t   a32[4];
    }u;

} __attribute__((packed)) NetfpProxy_IPv6_Addr;

/* IP Address Format used by the NETFP Library */
typedef struct NetfpProxy_IPAddr
{
    NetfpProxy_IPVersion     version;
    union
    {
        NetfpProxy_IPv4_Addr ipv4;
        NetfpProxy_IPv6_Addr ipv6;
    }addr;

} __attribute__((packed)) NetfpProxy_IPAddr;



/* The structure describes the configuration for IPSEC. */
typedef struct NetfpProxy_IPSecCfg
{
    NetfpProxy_IPSecProto        proto;
    NetfpProxy_IPSecMode         mode;
    uint16_t                     arw_size;
    NetfpProxy_IpsecAuthMode     auth_mode;
    NetfpProxy_IpsecCipherMode   enc_mode;
    uint32_t                     sa_flags;
    uint16_t                     auth_key_len;
    uint16_t                     enc_key_len;
    uint16_t                     icvlen;
    uint8_t                      auth_key[NETFP_PROXY_MAX_IPSEC_KEY_LEN];
    uint8_t                      enc_key[NETFP_PROXY_MAX_IPSEC_KEY_LEN];
    uint32_t                     esn_lo;
    uint32_t                     esn_hi;
    uint32_t                     natt_src_port;
    uint32_t                     natt_dst_port;
} __attribute__((packed)) NetfpProxy_IPSecCfg;

/* Ipsec tunnel lifetime configuration */
typedef struct NetfpProxy_IPSecLifetime
{
    uint64_t    soft_byte_limit;
    uint64_t    hard_byte_limit;
    uint64_t    soft_packet_limit;
    uint64_t    hard_packet_limit;
    uint64_t    soft_add_expires;
    uint64_t    hard_add_expires;
    uint64_t    soft_use_expires;
    uint64_t    hard_use_expires;

} __attribute__((packed)) NetfpProxy_IPSecLifetime;

typedef struct NetfpProxy_AddressInfo
{
    uint8_t                prefix_len;
    NetfpProxy_IPAddr      ip_addr;
} __attribute__((packed)) NetfpProxy_AddressInfo;

typedef struct NetfpProxy_PortInfo
{
    uint16_t               port_start;
    uint16_t               port_end;
} __attribute__((packed)) NetfpProxy_PortInfo;

typedef struct NetfpProxy_L5SelInfo
{
    NetfpProxy_l5_sel_proto  l5_sel_proto;
    uint32_t                 l5_sel_gtpu_teid_start;
    uint32_t                 l5_sel_gtpu_teid_end;

} __attribute__((packed)) NetfpProxy_L5SelInfo;

typedef struct NetfpProxy_SaHwCtx
{
    uint16_t    swinfo_sz;
    uint32_t    swinfo[NETFP_PROXY_SA_SWINFO_MAX_SZ];
    uint16_t    flow_id; /* CPPI flow-id for Rx DMA from SA */
} __attribute__((packed)) NetfpProxy_SaHwCtx;

/********** Message Elements - End ************/


/********** Message Structure - Start *********/

/* Add SA Request Message */
typedef struct NetfpProxy_AddSAReq
{
    NetfpProxy_MsgHdr          msg_hdr;
    uint32_t                   spi;
    NetfpProxy_SAMode          sa_mode;
    NetfpProxy_Direction       direction;
    NetfpProxy_IPAddr          src_addr;
    NetfpProxy_IPAddr          dst_addr;
    NetfpProxy_IPSecCfg        ipsecCfg;
    NetfpProxy_IPSecLifetime   lifetime;
    uint32_t                   root_sa_handle;
    NetfpProxy_OffloadMode     offload_mode;
} __attribute__((packed)) NetfpProxy_AddSAReq;

/* Add SA Response Message */
typedef struct NetfpProxy_AddSAResp
{
    NetfpProxy_MsgHdr  msg_hdr;
    int32_t            status;
    int32_t            sa_handle;
    NetfpProxy_SaHwCtx sa_hw_ctx;
} __attribute__((packed)) NetfpProxy_AddSAResp;

/* Add SP Request Message */
typedef struct NetfpProxy_AddSPReq
{
    NetfpProxy_MsgHdr      msg_hdr;
    uint32_t               policy_id;
    uint32_t               sa_handle;
    NetfpProxy_Direction   direction;
    NetfpProxy_AddressInfo src_addr_info;
    NetfpProxy_AddressInfo dst_addr_info;
    NetfpProxy_PortInfo    src_port_info;
    NetfpProxy_PortInfo    dst_port_info;
    uint16_t               l5_sel;
    NetfpProxy_L5SelInfo   l5_sel_info;
} __attribute__((packed)) NetfpProxy_AddSPReq;

/* Add SP Response Message */
typedef struct NetfpProxy_AddSPResp
{
    NetfpProxy_MsgHdr msg_hdr;
    int32_t           status;
    uint32_t          sp_handle;

} __attribute__((packed)) NetfpProxy_AddSPResp;

/* Delete SA Request Message */
typedef struct NetfpProxy_DeleteSAReq
{
    NetfpProxy_MsgHdr  msg_hdr;
    uint32_t           sa_handle;
    uint32_t           new_root_sa_handle;
} __attribute__((packed)) NetfpProxy_DeleteSAReq;

/* Delete SA Response Message */
typedef struct NetfpProxy_DeleteSAResp
{
    NetfpProxy_MsgHdr  msg_hdr;
    int32_t            status;

} __attribute__((packed)) NetfpProxy_DeleteSAResp;

/* Delete SP Request Message */
typedef struct NetfpProxy_DeleteSPReq
{
    NetfpProxy_MsgHdr  msg_hdr;
    uint32_t           policy_id;
    uint32_t           sp_handle;

} __attribute__((packed)) NetfpProxy_DeleteSPReq;

/* Delete SP Response Message */
typedef struct NetfpProxy_DeleteSPResp
{
    NetfpProxy_MsgHdr  msg_hdr;
    int32_t            status;
} __attribute__((packed)) NetfpProxy_DeleteSPResp;

/* SA Stats Request*/
typedef struct NetfpProxy_SAStatsReq
{
    NetfpProxy_MsgHdr msg_hdr;
    uint32_t          sa_handle;
} __attribute__((packed)) NetfpProxy_SAStatsReq;

/* SA Stats Response*/
typedef struct NetfpProxy_SAStatsResp
{
    NetfpProxy_MsgHdr msg_hdr;
    int32_t           status;
    uint64_t          bytes;
    uint64_t          packets;
    uint64_t          replay_fail;
    uint64_t          espcrypto_fail;
} __attribute__((packed)) NetfpProxy_SAStatsResp;

typedef struct NetfpProxy_ConfigL3QosReq
{
    NetfpProxy_MsgHdr   msg_hdr;
    uint8_t             interface_name[NETFP_PROXY_INTERFACE_NAME_LEN];
    uint8_t             l3_qos_config[64][20];
} __attribute__((packed)) NetfpProxy_ConfigL3QosReq;

typedef struct NetfpProxy_ConfigL3QosResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_ConfigL3QosResp;

typedef struct NetfpProxy_DeleteL3QosReq
{
    NetfpProxy_MsgHdr   msg_hdr;
    uint8_t             interface_name[NETFP_PROXY_INTERFACE_NAME_LEN];
} __attribute__((packed)) NetfpProxy_DeleteL3QosReq;

typedef struct NetfpProxy_DeleteL3QosResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_DeleteL3QosResp;

typedef struct NetfpProxy_ConfigureDscpPcpMappingReq
{
    NetfpProxy_MsgHdr   msg_hdr;
    uint8_t             interface_name[NETFP_PROXY_INTERFACE_NAME_LEN];
} __attribute__((packed)) NetfpProxy_ConfigureDscpPcpMappingReq;

typedef struct NetfpProxy_ConfigureDscpPcpMappingResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_ConfigureDscpPcpMappingResp;

typedef struct NetfpProxy_HeartbeatReq
{
    NetfpProxy_MsgHdr   msg_hdr;
} __attribute__((packed)) NetfpProxy_HeartbeatReq;

typedef struct NetfpProxy_HeartbeatResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_HeartbeatResp;

typedef struct NetfpProxy_StopSaReq
{
    NetfpProxy_MsgHdr   msg_hdr;
    uint32_t            sa_handle;
} __attribute__((packed)) NetfpProxy_StopSaReq;

typedef struct NetfpProxy_StopSaResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_StopSaResp;

typedef struct NetfpProxy_AddInterfaceReq
{
    NetfpProxy_MsgHdr   msg_hdr;
    uint8_t             interface_name[NETFP_PROXY_INTERFACE_NAME_LEN];
} __attribute__((packed)) NetfpProxy_AddInterfaceReq;

typedef struct NetfpProxy_AddInterfaceResp
{
    NetfpProxy_MsgHdr   msg_hdr;
    int32_t             status;
} __attribute__((packed)) NetfpProxy_AddInterfaceResp;

/********** Message Structure - End *********/

#ifdef __cplusplus
}
#endif

#endif /* __NETFP_PROXY_XFRM_BRIDGE_IFACE_HEADER_H__ */
