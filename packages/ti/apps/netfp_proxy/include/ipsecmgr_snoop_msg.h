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

#ifndef __IPSECMGR_SNOOP_MSG_H__
#define __IPSECMGR_SNOOP_MSG_H__

/* Libnl includes */
#include <netlink/addr.h>
#include <netlink/xfrm/ae.h>
#include <netlink/xfrm/sa.h>
#include <netlink/xfrm/sp.h>
#include <netlink/xfrm/selector.h>
#include <netlink/xfrm/lifetime.h>

/* IPSecMgr includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_types.h>
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop.h>

/* Message identifier to be passed to the State machine */
enum snoop_msg_id {
    MSG_OFFLOAD_SP_REQ = 1,
    MSG_STOP_OFFLOAD,
    MSG_NEW_SA,
    MSG_SA_EXPIRY_KNL,
    MSG_SA_EXPIRY_FP,
    MSG_DEL_SA,
    MSG_DEL_SP,
    MSG_CREATE_SA_RSP,
    MSG_ADD_SP_RSP,
    MSG_TIMEOUT,
    MSG_DUMP_SM
};

#define MAX_ADDR_SIZE   128
/* Offload SP req msg */
struct snoop_offload_sp_req_s {
    uint32_t                policy_id;
    ipsecmgr_sa_dscp_map_cfg_t dscp_map_cfg;
    ipsecmgr_l5_selector_t  l5_sel;
    ipsecmgr_sa_flags_t     sa_flags;
    ipsecmgr_ifname_t       ifname;
    ipsecmgr_trans_id_t     trans_id;
    uint8_t                 addr[MAX_ADDR_SIZE];
    uint32_t                addr_size;
};

/* Stop Offload SP msg */
struct snoop_offload_sp_stop_s {
    uint32_t                policy_id;
    ipsecmgr_trans_id_t     trans_id;
    uint8_t                 addr[MAX_ADDR_SIZE];
    uint32_t                addr_size;
    uint32_t                no_expire_sa;
};

/* Fast Path response msg */
struct snoop_fp_resp_s {
    ipsecmgr_trans_id_t     trans_id;
    ipsecmgr_rsp_type_t     type;
    ipsecmgr_result_t       result;
    uint8_t                 valid_fp_handle;
    ipsecmgr_fp_handle_t    fp_handle;
};

/* Structure for specifying SA info retrieved from Kernel */
struct snoop_sa_info_s {
    /* XFRM SA state */
    struct xfrmnl_sa    *sa;
};

/* Structure for specifying SP info from Kernel */
struct snoop_sp_info_s {
    /* XFRM SA state */
    struct xfrmnl_sp    *sp;
};

/* Structure for specifying SA expiry from FP */
struct snoop_sa_exp_info_s {
    ipsecmgr_fp_handle_t    sa_hndl;
    ipsecmgr_lft_cur_t      lft;
    uint8_t                 hard;
};

struct snoop_timeout_info_s {
    uint32_t  cookie;
};

union MSG_BODY {
    struct snoop_offload_sp_req_s   offload_sp;
    struct snoop_offload_sp_stop_s  stop_offload;
    struct snoop_fp_resp_s          fp_resp;
    struct snoop_sa_info_s          sa_info;
    struct snoop_sp_info_s          sp_info;
    struct snoop_sa_exp_info_s      sa_exp_info;
    struct snoop_timeout_info_s     timeout;
};

typedef struct {
    enum snoop_msg_id   msg_id;
    union MSG_BODY      body;
} snoop_msg_t;


#endif /* __IPSECMGR_SNOOP_MSG_H__ */

