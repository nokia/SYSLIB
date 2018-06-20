#ifndef NETFP_CONFIG_TRS_INTERFACE_H
#define NETFP_CONFIG_TRS_INTERFACE_H
/**
*******************************************************************
*	@file		NetFPConfig_TRS_interface.h
*	@date		29, September 2015
*	@version	1.1
*
*	@change history:
*
* Copyright 2015 Nokia Solutions and Networks.  All rights reserved.
*********************************************************************/

#include <netinet/in.h>
#include <linux/netlink.h>

static const int DSCP_NUMBER = 64;

static const int NETFP_CONFIG_STATUS_SUCCESS = 0;
static const int NETFP_CONFIG_STATUS_FAILURE = -1;

static const unsigned int NETFP_CONFIG_ACTION_ACCEPT = 0x0;
static const unsigned int NETFP_CONFIG_ACTION_DROP = 0x1;
static const unsigned int NETFP_CONFIG_ACTION_INC_MISMATCH = 0x2;
static const unsigned int NETFP_CONFIG_ACTION_INC_UNKNOWN = 0x4;
static const unsigned int NETFP_CONFIG_ACTION_REMOVE = 0x8;

/* Message Type */
typedef enum NetfpConfig_MsgType
{
    PM_STATS_REQ = 1,
    PM_STATS_RESP,
    MBTAC_DSCP_MAPPING_UPDATE_REQ,
    MBTAC_DSCP_MAPPING_UPDATE_RESP,
    MBTAC_STATS_REQ,
    MBTAC_STATS_RESP,
    IPSEC_STATS_REQ,
    IPSEC_STATS_RESP,
    INTERFACE_CONFIG_REQ,
    INTERFACE_CONFIG_RESP,
    VLAN_MISMATCH_STATS_REQ,
    VLAN_MISMATCH_STATS_RESP,
    HEARTBEAT_REQ,
    HEARTBEAT_RESP,
    GLOBAL_ERROR_STATS_REQ,
    GLOBAL_ERROR_STATS_RESP,
    ETH_OAM_CONFIG_REQ,
    ETH_OAM_CONFIG_RESP,
    IPSEC_CONFIG_REQ,
    IPSEC_CONFIG_RESP,
    MESSAGE_UNKNOWN
} NetfpConfig_MsgType;

typedef enum NetfpConfig_Ipsec_Enable
{
    Ipsec_Enable_ENABLE = 1,
    Ipsec_Enable_DISABLE
} NetfpConfig_Ipsec_Enable;

typedef enum NetfpConfig_Addr_Version
{
    IPVersion_IPV4 = AF_INET,
    IPVersion_IPV6 = AF_INET6
} NetfpConfig_Addr_Version;

typedef struct NetfpConfig_IPv4_Addr
{
    union
    {
        uint8_t    a8[4];
        uint32_t   a32;
    }u;
} __attribute__((packed)) NetfpConfig_IPv4_Addr;

typedef struct NetfpConfig_IPv6_Addr
{
    union
    {
        uint8_t    a8[16];
        uint16_t   a16[8];
        uint32_t   a32[4];
    }u;
} __attribute__((packed)) NetfpConfig_IPv6_Addr;

typedef struct NetFPConfig_MsgHdr
{
    NetfpConfig_MsgType msg_type;
} __attribute__((packed)) NetFPConfig_MsgHdr;

typedef struct NetfpConfig_IPAddr
{
    NetfpConfig_Addr_Version version;
    union {
        NetfpConfig_IPv4_Addr ipv4;
        NetfpConfig_IPv6_Addr ipv6;
    } addr;
} __attribute__((packed)) NetfpConfig_IPAddr;

typedef enum NetfpConfig_Eth_OAM_Enable
{
    Eth_OAM_Enable_ENABLE = 1,
    Eth_OAM_Enable_DISABLE
} NetfpConfig_Eth_OAM_Enable;

/********** PM Stats Messages - start ************/

/* PM_STATS_REQ Message */
typedef struct NetFPConfig_PmStatsReq
{
    NetFPConfig_MsgHdr header;
    uint32_t physical_port;
    uint32_t vlan_id;
    NetfpConfig_Addr_Version addr_ver;
} __attribute__((packed)) NetFPConfig_PmStatsReq;

/* PM_STATS_RESP Message */
typedef struct NetFPConfig_PmStatsResp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
    uint64_t ifInPackets;
    uint64_t ifInIPOctets;
    uint64_t ifInEthOctets;
    uint64_t ifOutPackets;
    uint64_t ifOutIPOctets;
    uint64_t ifOutEthOctets;
    uint64_t ifInDiscardPackets;
} __attribute__((packed)) NetFPConfig_PmStatsResp;

/* INTERFACE_CONFIG_REQ Message */
typedef struct NetFPConfig_Interface_Config_Req
{
    NetFPConfig_MsgHdr header;
    uint32_t physical_port;
    uint32_t vlan_id;
    uint32_t action;
} __attribute__((packed)) NetFPConfig_Interface_Config_Req;

/* INTERFACE_CONFIG_RESP Message */
typedef struct NetFPConfig_Interface_Config_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
} __attribute__((packed)) NetFPConfig_Interface_Config_Resp;

/* VLAN_ERROR_STATS_REQ Message */
typedef struct NetFPConfig_VLAN_Mismatch_Stats_Req
{
    NetFPConfig_MsgHdr header;
    uint32_t physical_port;
} __attribute__((packed)) NetFPConfig_VLAN_Mismatch_Stats_Req;

/* VLAN_ERROR_STATS_RESP Message */
typedef struct NetFPConfig_VLAN_Mismatch_Stats_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
    uint64_t VLAN_Mismatch;
} __attribute__((packed)) NetFPConfig_VLAN_Mismatch_Stats_Resp;

/* GLOBAL_ERROR_STATS_REQ Message */
typedef struct NetFPConfig_Global_Error_Stats_Req
{
    NetFPConfig_MsgHdr header;
} __attribute__((packed)) NetFPConfig_Global_Error_Stats_Req;

/* GLOBAL_ERROR_STATS_RESP Message */
typedef struct NetFPConfig_Global_Error_Stats_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
    uint64_t VLAN_Unknown;
} __attribute__((packed)) NetFPConfig_Global_Error_Stats_Resp;


/********** PM Stats Messages - end **************/


/********** MBTAC Messages - start ************/

/* MBTAC_DSCP_MAPPING_UPDATE_REQ Message */
typedef struct NetFPConfig_MBTAC_DSCP_Mapping_Update_Req
{
    NetFPConfig_MsgHdr header;
    NetfpConfig_IPAddr local_addr_info;
    uint8_t dscp_monitor[DSCP_NUMBER];
} __attribute__((packed)) NetFPConfig_MBTAC_DSCP_Mapping_Update_Req;

/* MBTAC_DSCP_MAPPING_UPDATE_RESP Message */
typedef struct NetFPConfig_MBTAC_DSCP_Mapping_Update_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
} __attribute__((packed)) NetFPConfig_MBTAC_DSCP_Mapping_Update_Resp;

/* MBTAC_STATS_REQ Message */
typedef struct NetFPConfig_MBTAC_Stats_Req
{
    NetFPConfig_MsgHdr header;
    NetfpConfig_IPAddr local_addr_info;
} __attribute__((packed)) NetFPConfig_MBTAC_Stats_Req;

/* MBTAC_STATS_RESP Message */
typedef struct NetFPConfig_MBTAC_Stats_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
    uint64_t inIPOctets;
    uint64_t inEthOctets;
    uint64_t outIPOctets;
    uint64_t outEthOctets;
} __attribute__((packed)) NetFPConfig_MBTAC_Stats_Resp;

/********** MBTAC Messages - end ************/


/********** Ipsec Messages - start ************/

/* IPSEC_CONFIG_REQ Message */
typedef struct NetFPConfig_IPSEC_Config_Req
{
    NetFPConfig_MsgHdr header;
    NetfpConfig_Ipsec_Enable ipsec_enable;
} __attribute__((packed)) NetFPConfig_IPSEC_Config_Req;

/* IPSEC_CONFIG_RESP Message */
typedef struct NetFPConfig_IPSEC_Config_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
} __attribute__((packed)) NetFPConfig_IPSEC_Config_Resp;

/* IPSEC_STATS_REQ Message */
typedef struct NetFPConfig_IPSEC_Stats_Req
{
    NetFPConfig_MsgHdr header;
} __attribute__((packed)) NetFPConfig_IPSEC_Stats_Req;

/* IPSEC_STATS_RESP Message */
typedef struct NetFPConfig_IPSEC_Stats_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
    uint64_t Protected_ESPFramesRx;
    uint64_t Bypassed_ESPFramesRx;
    uint64_t Discarded_ESPFramesRx;
    uint64_t DiscardedPolicyMismatch_FramesRx;
    uint64_t DiscardedDefault_FramesRx;
    uint64_t Protected_ESPFramesTx;
    uint64_t Bypassed_ESPFramesTx;
    uint64_t Discarded_ESPFramesTx;
    uint64_t DiscardedDefault_FramesTx;
    uint64_t Discarded_FailedEstablishFramesTx;
    uint64_t Discarded_TrafficSelectorMismatch_FramesRx;
} __attribute__((packed)) NetFPConfig_IPSEC_Stats_Resp;

/********** Ipsec Messages - end ************/


/********** Heartbeat Messages - start ************/

/* HEARTBEAT_REQ Message */
typedef struct NetFPConfig_Heartbeat_Req
{
    NetFPConfig_MsgHdr header;
} __attribute__((packed)) NetFPConfig_Heartbeat_Req;

/* HEARTBEAT_RESP Message */
typedef struct NetFPConfig_Heartbeat_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
} __attribute__((packed)) NetFPConfig_Heartbeat_Resp;

/********** Heartbeat Messages - end ************/

/********** Ethernet OAM Messages - start ************/

/* ETH_OAM_CONFIG_REQ Message */
typedef struct NetFPConfig_Eth_OAM_Config_Req
{
    NetFPConfig_MsgHdr header;
    uint32_t physical_port;
    NetfpConfig_Eth_OAM_Enable eth_oam_enable;
} __attribute__((packed)) NetFPConfig_Eth_OAM_Config_Req;

/* ETH_OAM_CONFIG_RESP Message */
typedef struct NetFPConfig_Eth_OAM_Config_Resp
{
    NetFPConfig_MsgHdr header;
    int32_t status;
} __attribute__((packed)) NetFPConfig_Eth_OAM_Config_Resp;

/********** Ethernet OAM Messages - end ************/

/********** NetFPConfig_Msg structure ***********/
typedef union NetFPConfig_MsgBody
{
    NetFPConfig_MsgHdr header;
    NetFPConfig_PmStatsReq pm_stats_req;
    NetFPConfig_PmStatsResp pm_stats_resp;
    NetFPConfig_Interface_Config_Req interface_config_req;
    NetFPConfig_Interface_Config_Resp interface_config_resp;
    NetFPConfig_VLAN_Mismatch_Stats_Req vlan_mismatch_stats_req;
    NetFPConfig_VLAN_Mismatch_Stats_Resp vlan_mismatch_stats_resp;
    NetFPConfig_Global_Error_Stats_Req global_error_stats_req;
    NetFPConfig_Global_Error_Stats_Resp global_error_stats_resp;
    NetFPConfig_MBTAC_DSCP_Mapping_Update_Req mbtac_dscp_mapping_update_req;
    NetFPConfig_MBTAC_DSCP_Mapping_Update_Resp mbtac_dscp_mapping_update_resp;
    NetFPConfig_MBTAC_Stats_Req mbtac_stats_req;
    NetFPConfig_MBTAC_Stats_Resp mbtac_stats_resp;
    NetFPConfig_IPSEC_Config_Req ipsec_config_req;
    NetFPConfig_IPSEC_Config_Resp ipsec_config_resp;
    NetFPConfig_IPSEC_Stats_Req ipsec_stats_req;
    NetFPConfig_IPSEC_Stats_Resp ipsec_stats_resp;
    NetFPConfig_Heartbeat_Req heartbeat_req;
    NetFPConfig_Heartbeat_Resp heartbeat_resp;
    NetFPConfig_Eth_OAM_Config_Req eth_oam_config_req;
    NetFPConfig_Eth_OAM_Config_Resp eth_oam_config_resp;
} __attribute__((packed)) NetFPConfig_MsgBody;

typedef struct NetFPConfig_Msg
{
    struct nlmsghdr netlink_header;
    NetFPConfig_MsgBody msg;
} __attribute__((packed)) NetFPConfig_Msg;
/********** NetFPCongih_msg structure ***********/

#endif // NETFP_CONFIG_TRS_INTERFACE_H
