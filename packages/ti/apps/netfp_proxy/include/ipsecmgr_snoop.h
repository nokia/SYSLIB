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

#ifndef __IPSECMGR__SNOOP_H__
#define __IPSECMGR__SNOOP_H__

#include <ti/apps/netfp_proxy/include/ipsecmgr_types.h>

/* Callbacks from the Snoop library */

/* Callback API to add IPSec Security Association
 * Returns 0 on success
 */
typedef int (*ipsecmgr_snoop_add_sa_t) (
    ipsecmgr_af_t           af,
    ipsecmgr_sa_id_t        *sa_id,
    ipsecmgr_sa_info_t      *sa_info,

    /* DSCP mapping config valid only in tunnel mode */
    ipsecmgr_sa_dscp_map_cfg_t *dscp_map_cfg,

    /* For DIR==OUTBOUND,this should be the MAC interface through which the
     * packet should be sent out.For DIR==INBOUND, this would be the MAC interface
     * to which SA could be linked.
     * This is an optional parameter.
     */
    ipsecmgr_ifname_t       *if_name,

    /* UDP ecapsulation info for NAT-T */
    ipsecmgr_sa_encap_tmpl_t   *encap,

    /* output parameter: FP lib SA handle */
    ipsecmgr_fp_handle_t    *sa_handle
);

/* Callback API to add IPSec Security Policy
 * Returns 0 on success
 */
typedef int (*ipsecmgr_snoop_add_sp_t) (
    ipsecmgr_af_t           af,
    ipsecmgr_selector_t     *sel,
    ipsecmgr_dir_t          dir,
    uint32_t                reqid,
    ipsecmgr_fp_handle_t    sa_handle, /* SA bundle not supported */
   /* Security Policy ID in the kernel */
    ipsecmgr_policy_id_t    policy_id,

    /* output parameter: FP lib SP handle */
    ipsecmgr_fp_handle_t    *sp_handle
);

/* Callback API to delete IPSec Security Policy
 * Returns 0 on success
 */
typedef int (*ipsecmgr_snoop_del_sp_t) (
    ipsecmgr_fp_handle_t   sp_handle,
    ipsecmgr_policy_id_t   policy_id,
    uint32_t reqid,
    ipsecmgr_dir_t         dir
);

/* Callback API to delete IPSec Security Association
 * Returns 0 on success
 */
 #if 1
typedef int (*ipsecmgr_snoop_del_sa_t) (
    uint32_t reqid,
    ipsecmgr_dir_t dir,
    ipsecmgr_fp_handle_t   sa_handle
);
#else
typedef int (*ipsecmgr_snoop_del_sa_t) (
    ipsecmgr_fp_handle_t   sa_handle
);

#endif
/* Callback API to get the IPSec Security Association
 * hardware context information from NETCP
 * Returns 0 on success
 */
typedef int (*ipsecmgr_snoop_get_sa_hw_ctx_t) (
    ipsecmgr_fp_handle_t    sa_handle,
    ipsecmgr_sa_hw_ctx_t    *hw_ctx
);

/* Callback API to delay */
typedef void (*ipsecmgr_snoop_sleep_t) (
    uint32_t    delay   /* in milliseconds */
);

/* Offload SP request parameters */
typedef struct {
    /* Transaction id of the request */
    ipsecmgr_trans_id_t     trans_id;

   /* Security Policy ID in the kernel */
    ipsecmgr_policy_id_t    policy_id;

    /* MAC interface name
     * This is an optional parameter. For egress, snoop library will find the
     * outgoing MAC interface from the Linux route table.
     * For ingress, application can provide this if it requires the ingress SA
     * to be linked to a particular MAC interface.
     */
    ipsecmgr_ifname_t       *if_name;

    /* SA tunnel header DSCP mapping cfg
     * This is an optional parameter. If not present the default value of
     * SA_DSCP_MAP_USE_INNER_IP will be assumed */
    ipsecmgr_sa_dscp_map_cfg_t *dscp_cfg;

    /* Layer 5 selector.
     * This is an optional parameter */
    ipsecmgr_l5_selector_t  *l5_selector;

    /* Additional SA flags */
    ipsecmgr_sa_flags_t     sa_flags;

} ipsecmgr_snoop_offload_sp_req_param_t;

/* Stop offload SP request parameters */
typedef struct {
    /* Transaction id of the request */
    ipsecmgr_trans_id_t     trans_id;

   /* Security Policy ID in the kernel */
    ipsecmgr_policy_id_t    policy_id;

   /* Flag to indicate not to expire SA on stop_offload */
   ipsecmgr_no_expire_sa_t no_expire_sa;

} ipsecmgr_snoop_stop_offload_req_param_t;

/* Rekey event parameters */
typedef struct {
    ipsecmgr_policy_id_t    policy_id;
    ipsecmgr_result_t       result;
    ipsecmgr_fp_handle_t    rekey_sa_handle;
} ipsecmgr_snoop_rekey_event_param_t;

/* Offload SP response parameters */
typedef struct {
    /* Transaction id to correlate request */
    ipsecmgr_trans_id_t     trans_id;
    ipsecmgr_rsp_type_t     type;
    ipsecmgr_result_t       result;

    /* FP lib err code */
    uint32_t                err_code;
    ipsecmgr_fp_handle_t    sp_handle;
    ipsecmgr_fp_handle_t    sa_handle;
} ipsecmgr_snoop_offload_sp_rsp_param_t;

/* Stop offload response parameters */
typedef struct {
    /* Transaction id to correlate request */
    ipsecmgr_trans_id_t     trans_id;
    ipsecmgr_rsp_type_t     type;
    ipsecmgr_result_t       result;
} ipsecmgr_snoop_stop_offload_rsp_param_t;

typedef int (*ipsecmgr_snoop_rekey_event_t)
(
    ipsecmgr_snoop_rekey_event_param_t  *params,

    /* IPC address of the user */
    void                                *addr,

    /* Address size */
    uint32_t                            size
);

typedef int (*ipsecmgr_snoop_offload_sp_rsp_t)
(
    ipsecmgr_snoop_offload_sp_rsp_param_t *rsp,

    /* IPC address of the user from which the request was recieved */
    void                                  *addr,

    /* Address size */
    uint32_t                              size
);

typedef int (*ipsecmgr_snoop_stop_offload_rsp_t)
(
    ipsecmgr_snoop_stop_offload_rsp_param_t *rsp,

    /* IPC address of the user from which the request was recieved */
    void                                  *addr,

    /* Address size */
    uint32_t                              size
);

/**
 * Logging levels
 */
enum ipsecmgr_snoop_log_level_e {
    LOG_LEVEL_INFO = 0,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
};

/**
 * callback to log message
 */
typedef void (*ipsecmgr_snoop_log_msg_t)
(
    enum ipsecmgr_snoop_log_level_e level, char *msg, ...
);

/* Fast Path configuration callbacks */
struct ipsecmgr_snoop_fp_cfg_cb {
    ipsecmgr_snoop_add_sa_t   add_sa;
    ipsecmgr_snoop_add_sp_t   add_sp;
    ipsecmgr_snoop_del_sa_t   del_sa;
    ipsecmgr_snoop_del_sp_t   del_sp;
    ipsecmgr_snoop_get_sa_hw_ctx_t get_sa_ctx;
};

/* IPC callbacks */
struct ipsecmgr_snoop_mgnt_cb {
    ipsecmgr_snoop_offload_sp_rsp_t     offload_sp_rsp;
    ipsecmgr_snoop_stop_offload_rsp_t   stop_offload_rsp;
    ipsecmgr_snoop_rekey_event_t        rekey_event;
};

/* Platform specific callbacks */
struct ipsecmgr_snoop_platform_cb {
    ipsecmgr_snoop_log_msg_t    log_msg;
    ipsecmgr_snoop_sleep_t      sleep;
};

/* API's */

/* API to initialize the library */
int ipsecmgr_snoop_init
(
    struct ipsecmgr_snoop_fp_cfg_cb     *fp_cfg_cb,
    struct ipsecmgr_snoop_mgnt_cb       *mgnt_cb,
    struct ipsecmgr_snoop_platform_cb   *plat_cb
);

/* API to initiate offload SP request */
int ipsecmgr_snoop_offload_sp_req
(
    ipsecmgr_snoop_offload_sp_req_param_t *req,

    /* IPC address of the user from which the request was recieved */
    void                                  *addr,

    /* Address size */
    uint32_t                              size
);

/* API to stop SP offload */
int ipsecmgr_snoop_stop_offload
(
    ipsecmgr_snoop_stop_offload_req_param_t *req,

    /* IPC address of the user from which the request was recieved */
    void                                  *addr,

    /* Address size */
    uint32_t                              size
);

/* API to notify SA expiration */
int ipsecmgr_snoop_sa_expiry
(
    ipsecmgr_fp_handle_t    sa_handle,

    /* set to non-zero value if the notification is
     * for HARD lifetime expiry */
    uint8_t                 hard,

    /* Current lifetime stats */
    ipsecmgr_lft_cur_t      *lft
);

/* API to dump IpsecMgr states */
int ipsecmgr_snoop_dump_state (void);

/* API to run the library
 */
void ipsecmgr_snoop_run(void);

/* API to shutdown */
void ipsecmgr_snoop_shutdown(void);

#endif /* __IPSECMGR__SNOOP_H__ */

