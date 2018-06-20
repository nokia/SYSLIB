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

#ifndef __IPSECMGR_SNOOP_LOC_H__
#define __IPSECMGR_SNOOP_LOC_H__

#include <sys/socket.h>
#include <netinet/in.h>
#include <mqueue.h>

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
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_msg.h>

/* NETCP CFG API's are synchronous (blocking) */
#define SYNCHRONOUS_NETCP_CFG_API

/* Interval(in ms) between event polls to be used by the forever loop */
#define EVENT_POLL_INTVL    100

/* Message Queue Name */
#define MSQ_NAME "/ipsecmgr_snoop_mq"

#define MSQ_NAME_MAX_LEN 64


/* Message Queue depth */
#define MSQ_DEPTH 10

enum policy_type {
    POLICY_IPSEC = 1,
    POLICY_NONE
};

struct sa_info_s {
    /* XFRM SA state */
    struct xfrmnl_sa       *sa;
    /* SA hw context */
    ipsecmgr_sa_hw_ctx_t    hw_ctx;
};
/* Data structure to hold all the Libnl XFRM info */
typedef struct ipsecmgr_xfrm_ctx_t
{
    struct nl_cache_mngr*   cache_mngr;
    struct nl_cache*        sa_cache;
    struct nl_cache*        sp_cache;
    struct nl_sock*         nl_sock;
} ipsecmgr_xfrm_ctx_t;

typedef struct {
    struct ipsecmgr_snoop_fp_cfg_cb     fp_cfg_cb;
    struct ipsecmgr_snoop_mgnt_cb       mgnt_cb;
    struct ipsecmgr_snoop_platform_cb   plat_cb;
    ipsecmgr_xfrm_ctx_t                 xfrm_ctx;
    mqd_t                               mq;
    char                                name[MSQ_NAME_MAX_LEN];
} ipsecmgr_snoop_ctx_t;

extern ipsecmgr_snoop_ctx_t snoop_ctx;

/* Intra module function prototypes */
int ipsecmgr_snoop_sm_init(void);
void ipsecmgr_snoop_sm_shutdown(void);
int ipsecmgr_snoop_sm_send_msg(snoop_msg_t *msg);
void ipsecmgr_snoop_sm_poll_msg();

int ipsecmgr_snoop_xfrm_init(void);
int ipsecmgr_snoop_xfrm_sp_get
(
    uint32_t            policy_id,
    struct xfrmnl_sp    **sp,
    ipsecmgr_dir_t      *dir,
    enum policy_type    *type
);
void ipsecmgr_snoop_xfrm_sp_put(struct xfrmnl_sp *);
int ipsecmgr_snoop_xfrm_sa_get
(
    struct xfrmnl_sp    *sp,
    struct xfrmnl_sa    **sa,
    int                 *reqid
);
uint32_t  ipsecmgr_snoop_xfrm_sa_getRefCount(struct xfrmnl_sa *sa);
void ipsecmgr_snoop_xfrm_sa_inherit(struct xfrmnl_sa    *sa);
void ipsecmgr_snoop_xfrm_sa_put(struct xfrmnl_sa *);
int ipsecmgr_snoop_xfrm_sp_compatible_sa
(
    struct xfrmnl_sp    *sp,
    struct xfrmnl_sa    *sa
);
int ipsecmgr_snoop_xfrm_sa_cmp
(
    struct xfrmnl_sa    *sa1,
    struct xfrmnl_sa    *sa2
);
int ipsecmgr_snoop_xfrm_sp_cmp
(
    struct xfrmnl_sp    *sp1,
    struct xfrmnl_sp    *sp2
);

int ipsecmgr_snoop_xfrm_convert_dir
(
    int             xfrm_dir,
    ipsecmgr_dir_t  *ipsecmgr_dir
);

int ipsecmgr_snoop_xfrm_get_oseq
(
    struct xfrmnl_sa    *sa,
    unsigned int        *oseq,
    unsigned int        *oseq_hi
);
int ipsecmgr_snoop_xfrm_get_lft_cur
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cur_t  *clft
);
int ipsecmgr_snoop_xfrm_get_lft_cfg
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cfg_t  *lft_cfg
);
int ipsecmgr_snoop_xfrm_update_lft
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cur_t  *clft
);
void ipsecmgr_snoop_poll_xfrm(uint32_t timeout_val_ms);
void ipsecmgr_snoop_xfrm_shutdown(void);

int ipsecmgr_snoop_fp_init(void);
void ipsecmgr_snoop_fp_shutdown(void);
int ipsecmgr_snoop_fp_create_sa
(
    struct sa_info_s            *sa_params,
    ipsecmgr_sa_flags_t         sa_flags,
    ipsecmgr_dir_t              dir,
    ipsecmgr_sa_dscp_map_cfg_t *dscp_map_cfg,
    ipsecmgr_ifname_t           *if_name,
    ipsecmgr_trans_id_t         *trans_id,
    ipsecmgr_fp_handle_t        *sa_handle,
    uint32_t                    *fp_result
);
int ipsecmgr_snoop_fp_add_sp
(
    struct xfrmnl_sp       *sp,
    ipsecmgr_l5_selector_t  *sp_l5_sel,
    uint32_t                reqid,
    ipsecmgr_fp_handle_t    sa_handle,
    ipsecmgr_trans_id_t     *trans_id,
    ipsecmgr_fp_handle_t    *sp_handle,
    uint32_t                *fp_result
);

int ipsecmgr_snoop_fp_del_sa
(
    uint32_t reqid,
    ipsecmgr_dir_t dir,
    ipsecmgr_fp_handle_t sa_handle
);

int ipsecmgr_snoop_fp_del_sp
(
    ipsecmgr_fp_handle_t    sp_handle,
    uint32_t                policy_id,
    uint32_t                reqid,
    ipsecmgr_dir_t          dir
);

int ipsecmgr_snoop_fp_get_sa_hw_ctx
(
    ipsecmgr_fp_handle_t    sa_handle,
    struct sa_info_s        *sa_info,
    uint32_t                *fp_result
);

int ipsecmgr_snoop_user_ipc_init();
void ipsecmgr_snoop_user_ipc_shutdown();
int ipsecmgr_snoop_user_ipc_send_offload_sp_done
(
    struct snoop_offload_sp_req_s *req,
    ipsecmgr_fp_handle_t    *sp_handle,
    ipsecmgr_fp_handle_t    *sa_handle,
    ipsecmgr_result_t       result,
    uint32_t                fp_result
);
int ipsecmgr_snoop_user_ipc_send_stop_offload_done
(
    struct snoop_offload_sp_stop_s *req,
    ipsecmgr_result_t       result
);
int ipsecmgr_snoop_user_ipc_send_rekey_event
(
    struct snoop_offload_sp_req_s *req,
    ipsecmgr_result_t       result,
    ipsecmgr_fp_handle_t    sa_handle
);

int ipsecmgr_snoop_init_timer_list(void);
void ipsecmgr_snoop_shutdown_timer_list(void);
void ipsecmgr_snoop_increment_tick(void);
int ipsecmgr_snoop_start_timer
(
    uint32_t ms,
    void *cb_handle,
    int32_t *timer_handle
);
int ipsecmgr_snoop_stop_timer(int32_t timer_handle);

int ipsecmgr_snoop_knl_mod_open();
int ipsecmgr_snoop_knl_set_ctx
(
    struct sa_info_s    *sa_info,
    ipsecmgr_dir_t      dir
);
int ipsecmgr_snoop_knl_del_ctx
(
    struct sa_info_s    *sa_info,
    ipsecmgr_dir_t      dir
);
int ipsecmgr_snoop_knl_check_sa_expiry
(
    struct sa_info_s    *sa_info
);
void ipsecmgr_snoop_knl_mod_close();

int ipsecmgr_snoop_mq_init();
int ipsecmgr_snoop_mq_send(snoop_msg_t *msg, uint32_t len);
int ipsecmgr_snoop_mq_recv(snoop_msg_t *msg, uint32_t *len);
void ipsecmgr_snoop_mq_shutdown(void);

#endif /* __IPSECMGR_SNOOP_LOC_H__ */

