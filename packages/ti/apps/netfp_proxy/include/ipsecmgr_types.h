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

#ifndef __IPSECMGR__TYPES_H__
#define __IPSECMGR__TYPES_H__

#ifndef __KERNEL__
#include <stdint.h>
#endif

/**
 * MAC address options.
 */
/* valid param flags */
#define MAC_ADDR_OPT_VALID_PARAM_VLAN_ID    0x1
#define MAC_ADDR_OPT_VALID_PARAM_VLAN_PRIO    0x2

/* MAC frame formats */
#define MAC_ADDR_OPT_FRAME_FORMAT_802_3    0x1
#define MAC_ADDR_OPT_FRAME_FORMAT_DIX    0x2

typedef struct {
    uint16_t    valid_params;
    uint16_t    vlan_id;
    uint8_t     frame_format;
    uint8_t     vlan_prio;
} ipsecmgr_mac_addr_options_t;

/**
 * IP address family.
 */
typedef enum {
    IPSECMGR_AF_IPV4 = 1,
    IPSECMGR_AF_IPV6 = 2
} ipsecmgr_af_t;

/**
 * IP address.
 */
typedef union {
    uint8_t ipv4[4];
    uint8_t ipv6[16];
} ipsecmgr_ipaddr_t;

/**
 * IPSec protocol types.
 */
typedef enum {
    SA_PROTO_AH =  1,
    SA_PROTO_ESP = 2
} ipsecmgr_ipsec_proto_t;

/**
 * IPSec SA identifier.
 */
typedef struct {
    uint32_t                spi;  /* IPSec Security Parameter index */
    ipsecmgr_ipaddr_t       daddr;/* Dest IP address */
    ipsecmgr_ipsec_proto_t  proto;
} ipsecmgr_sa_id_t;

/**
 * Authentication algorithms.
 */
typedef enum {
    SA_AALG_NONE = 0,
    SA_AALG_NULL = 1,
    SA_AALG_HMAC_MD5 = 2,
    SA_AALG_HMAC_SHA1 = 3,
    SA_AALG_HMAC_SHA2_256 = 4,
    SA_AALG_HMAC_SHA2_224 = 5,
    SA_AALG_AES_XCBC = 6,
    SA_AALG_AES_GMAC = 7
} ipsecmgr_aalg_t;

typedef struct {
    ipsecmgr_aalg_t algo;
    uint16_t        icvlen;
} ipsecmgr_auth_params_t;

/**
 * Encryption algorithms.
 */
typedef enum {
    SA_EALG_NONE        = 0,
    SA_EALG_NULL        = 1,
    SA_EALG_AES_CTR     = 2,
    SA_EALG_3DES_CBC    = 3,
    SA_EALG_AES_CBC     = 4,
    SA_EALG_AES_CCM     = 5,
    SA_EALG_AES_GCM     = 6,
    SA_EALG_DES_CBC     = 7
} ipsecmgr_ealg_t;

typedef struct {
    ipsecmgr_ealg_t algo;
} ipsecmgr_enc_params_t;

/**
 * IPSec SA & SP direction.
 */
typedef enum {
    DIR_INBOUND  = 1,
    DIR_OUTBOUND = 2
} ipsecmgr_dir_t;

/**
 * IPSec operation modes.
 */
typedef enum {
    SA_MODE_TRANSPORT = 1,
    SA_MODE_TUNNEL    = 2
} ipsecmgr_ipsec_mode_t;

typedef enum {
    /* Copy the DSCP values from Inner IP to Outer IP */
    SA_DSCP_MAP_USE_INNER_IP    =1,
    /* Use a fixed DSCP value in the outer IP */
    SA_DSCP_MAP_USE_FIXED_VAL   =2,
    /* Use a mapping table to determine DSCP value for the outer IP */
    SA_DSCP_MAP_USE_MAP_TABLE   =3
} ipsecmgr_sa_dscp_map_type_t;

/**
 * DSCP Mapping configuration from inner IP to outer IP:
 * Valid for direction == OUTBOUND
 */
typedef struct {
    ipsecmgr_sa_dscp_map_type_t type;
    union {
        uint8_t fixed_val;
        uint8_t map_table[64]; /* value of 0xff denotes no mapping specified */
    } value;
} ipsecmgr_sa_dscp_map_cfg_t;


/**
 * Lifetime configuration
 */
typedef struct {
    /* # of bytes that may be processed by the SA
     * before SOFT expiry */
    uint64_t    soft_byte_limit;
    /* # of bytes that may be processed by the SA
     * before HARD expiry */
    uint64_t    hard_byte_limit;
    /* # of packets that may be processed by the SA
     * before SOFT expiry */
    uint64_t    soft_packet_limit;
    /* # of packets that may be processed by the SA
     * before HARD expiry */
    uint64_t    hard_packet_limit;
    /* # of seconds after the creation of the SA until
     * its SOFT expiry */
    uint64_t    soft_add_expires;
    /* # of seconds after the creation of the SA until
     * its HARD expiry */
    uint64_t    hard_add_expires;
    /* # of seconds after the first use of the SA until
     * its SOFT expiry */
    uint64_t    soft_use_expires;
    /* # of seconds after the first use of the SA until
     * its HARD expiry */
    uint64_t    hard_use_expires;
} ipsecmgr_lft_cfg_t;

/**
 * current lifetime stats
 */
typedef struct {
    uint64_t    bytes;
    uint64_t    packets;
    uint64_t    add_time;
    uint64_t    use_time;
} ipsecmgr_lft_cur_t;


/* Value for sa_flags used for specifying additional SA options */
/* Indicates ESN is enabled */
#define IPSECMGR_SA_FLAGS_ESN       0x01
/* Indicates if the SA is shared between FP & Linux kernel */
#define IPSECMGR_SA_FLAGS_SHARED    0x02
/* Indicates if the SA is a rekey SA */
#define IPSECMGR_SA_FLAGS_REKEY     0x04
/* Indicates if outer IP fragmentation is required for
 * packets on this tunnel. Default is inner IP fragmentation.
 * For shared SA only inner IP fragmentation is supported.
 */
#define IPSECMGR_SA_FLAGS_OUTER_FRAG 0x08

typedef uint32_t    ipsecmgr_sa_flags_t;


/* Maximum key length supported */
#define IPSECMGR_SA_INFO_MAX_KEY_LEN 64

/**
 * IPSec SA params.
 */
typedef struct {
    ipsecmgr_sa_flags_t     flags;
    ipsecmgr_ipsec_mode_t   mode;
    ipsecmgr_ipaddr_t       saddr;
    ipsecmgr_lft_cfg_t      lft;
    /* Replay window size */
    uint32_t                replay_window;
    uint32_t                reqid;
    ipsecmgr_dir_t          dir;
    ipsecmgr_auth_params_t  auth;
    ipsecmgr_enc_params_t   enc;
    uint16_t                enc_key_len; /* in bytes */
    uint16_t                auth_key_len; /* in bytes */
    uint8_t                 enc_key[IPSECMGR_SA_INFO_MAX_KEY_LEN];
    uint8_t                 auth_key[IPSECMGR_SA_INFO_MAX_KEY_LEN];
    unsigned int            esnlo;
    unsigned int            esnhi;
} ipsecmgr_sa_info_t;


typedef enum {
    IPSECMGR_L5_PROTO_GTPU  = 1
} ipsecmgr_l5_proto_t;

typedef struct {
    ipsecmgr_l5_proto_t proto;
    union {
        struct {
            /* GTP-U TEID range. 0 represents "ANY" TEID */
            uint32_t    teid_start;
            uint32_t    teid_end;
        } gtpu;
    } value;
} ipsecmgr_l5_selector_t;

/**
 * IPSec SA selector used in IPSec Security policy.
 */
typedef struct {
    /* Destination IP address */
    ipsecmgr_ipaddr_t   daddr;

    /* Destination IP address prefix length
     * value of 0 means "ANY" destination IP */
    uint8_t             prefixlen_d;

    /* Source IP address */
    ipsecmgr_ipaddr_t   saddr;

    /* Source IP address prefix length
     * value of 0 means "ANY" source IP */
    uint8_t             prefixlen_s;

    /* IANA assigned layer 4 protocol number
     * protocol = "ANY" is represented by 255 */
    uint8_t             proto;

    /* Destination port range start
     * value of 0 means "ANY" destination port */
    uint16_t            dport_start;

    /* Destination port range end */
    uint16_t            dport_end;

    /* Source port range start
     * value of 0 means "ANY" source port */
    uint16_t            sport_start;

    /* Source port range end */
    uint16_t            sport_end;

    /* Layer 5 selector.
     * This is an optional parameter */
    ipsecmgr_l5_selector_t  *l5_selector;

} ipsecmgr_selector_t;

/**
 * Transaction identifier
 */
typedef uint32_t ipsecmgr_trans_id_t;

/**
 * Fast Path handle
 */
typedef uint32_t ipsecmgr_fp_handle_t;

/**
 * process identifier
 */
typedef uint32_t ipsecmgr_pid_t;

/**
 * Security policy identifier
 */
typedef uint32_t ipsecmgr_policy_id_t;

/**
 * Flag to indicate not to expire SA on stop_offload
 */
typedef uint32_t ipsecmgr_no_expire_sa_t;


/**
 * MAC interface name
 */
#define IPSECMGR_IFNAME_MAX_SZ  32
typedef struct {
    uint8_t name[IPSECMGR_IFNAME_MAX_SZ];
} ipsecmgr_ifname_t;

/**
 * ipsecmgr result codes
 */
typedef enum {
    RESULT_SUCCESS = 0,
    RESULT_FAILURE = -1, /* Generic Failure */
    RESULT_GET_SP_FAIL = -2, /* failed to retrieve Security Policy from kernel */
    RESULT_GET_SA_FAIL = -3, /* failed to retrieve Security Association from kernel */
    RESULT_ADD_SA_FAIL = -4, /* failed to add Security Association */
    RESULT_ADD_SP_FAIL = -5, /* failed to add Security Policy */
    RESULT_INVLD_PARAM = -6, /* Invalid parameter */
    RESULT_KNL_OFFLOAD_FAIL = -7, /* SA context offload to Linux failed */
    RESULT_NO_OFFLD_SP = -8, /* specified SP index is not offloaded */
    RESULT_DUPL_SP = -9, /* specified SP index is already offloaded */
} ipsecmgr_result_t;

/**
 * To indicate the type of response.
 */
typedef enum {
    RSP_TYPE_ACK  = 0x1,  /* API Request recieved */
    RSP_TYPE_DONE = 0x2   /* API Action is complete */
} ipsecmgr_rsp_type_t;

/**
 * Security association HW context.
 */
#define IPSECMGR_SA_SWINFO_MAX_SZ    3
typedef struct {
    uint16_t    swinfo_sz;
    uint32_t    swinfo[IPSECMGR_SA_SWINFO_MAX_SZ];
    uint16_t    flow_id; /* CPPI flow-id for Rx DMA from SA */
} ipsecmgr_sa_hw_ctx_t;

/**
 * UDP ecapsulation info for NAT-T.
 */
typedef struct {
    uint16_t    sport;
    uint16_t    dport;
} ipsecmgr_sa_encap_tmpl_t;

#endif /* __IPSECMGR__TYPES_H__ */

