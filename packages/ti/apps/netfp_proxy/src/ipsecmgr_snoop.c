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

#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

ipsecmgr_snoop_ctx_t snoop_ctx;

int ipsecmgr_snoop_init
(
    struct ipsecmgr_snoop_fp_cfg_cb     *fp_cfg_cb,
    struct ipsecmgr_snoop_mgnt_cb       *mgnt_cb,
    struct ipsecmgr_snoop_platform_cb   *plat_cb
)
{
    if ((!fp_cfg_cb) || (!mgnt_cb) || (!plat_cb))
        return -1;

    if (!(fp_cfg_cb->add_sa) || !(fp_cfg_cb->add_sp) ||
        !(fp_cfg_cb->del_sa) || !(fp_cfg_cb->del_sp) ||
        !(fp_cfg_cb->get_sa_ctx) ||
        !(mgnt_cb->offload_sp_rsp) || !(mgnt_cb->stop_offload_rsp) ||
        !(plat_cb->sleep) || !(plat_cb->log_msg)) {
        return -1;
    }

    memset (&snoop_ctx, 0, sizeof(snoop_ctx));

    snoop_ctx.fp_cfg_cb = *fp_cfg_cb;
    snoop_ctx.mgnt_cb = *mgnt_cb;
    snoop_ctx.plat_cb = *plat_cb;

    /* Initializations */
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: init: starting initialization\n");

    if (ipsecmgr_snoop_mq_init()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing message Queue\n");
        return -1;
    }

    if (ipsecmgr_snoop_init_timer_list()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing timer list\n");
        return -1;
    }

    if (ipsecmgr_snoop_xfrm_init()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing xfrm interface\n");
        return -1;
    }

    if (ipsecmgr_snoop_fp_init()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing FP lib\n");
        return -1;
    }

    if (ipsecmgr_snoop_user_ipc_init()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing user_ipc\n");
        return -1;
    }

    if (ipsecmgr_snoop_sm_init()) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR, "snoop: init: Error initializing state machine\n");
        return -1;
    }

    return 0;
}

void ipsecmgr_snoop_run(void)
{
    /* Poll state machine messages */
    ipsecmgr_snoop_sm_poll_msg();

    /* Poll for messages from Kernel */
    ipsecmgr_snoop_poll_xfrm(0);

    snoop_ctx.plat_cb.sleep(EVENT_POLL_INTVL);

    /* Increment timer tick */
    ipsecmgr_snoop_increment_tick();

    return;
}

int ipsecmgr_snoop_offload_sp_req
(
    ipsecmgr_snoop_offload_sp_req_param_t   *ipc_req,
    void                                    *addr,
    uint32_t                                addr_size
)
{
    snoop_msg_t msg;
    struct snoop_offload_sp_req_s *req = &msg.body.offload_sp;

    memset (req, 0, sizeof(*req));

    req->policy_id = ipc_req->policy_id;
    req->trans_id = ipc_req->trans_id;
    req->sa_flags = ipc_req->sa_flags;

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
    "snoop: recieved offload_sp_req for sp-id(%d) trans_id(%d)\
    sa_flags(0x%x)\n", req->policy_id, req->trans_id, req->sa_flags);

    if (ipc_req->if_name)
        memcpy(&req->ifname, ipc_req->if_name, sizeof(ipsecmgr_ifname_t));

    if (ipc_req->dscp_cfg) {
        memcpy(&req->dscp_map_cfg, ipc_req->dscp_cfg,
                sizeof(ipsecmgr_sa_dscp_map_cfg_t));
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: offload_sp: dscp_cfg present\n");
    } else {
        req->dscp_map_cfg.type = SA_DSCP_MAP_USE_INNER_IP;
    }

    if (ipc_req->l5_selector) {
        memcpy(&req->l5_sel, ipc_req->l5_selector,
               sizeof(ipsecmgr_l5_selector_t));
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: offload_sp: l5_selector present\n");
    }

    if (addr_size > MAX_ADDR_SIZE) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: offload_sp: addr_size too big (%d)\n", addr_size);
        return -1;
    }

    memcpy(req->addr, (uint8_t *)addr, addr_size);
    req->addr_size = addr_size;

    /* Trigger the state machine */
    msg.msg_id = MSG_OFFLOAD_SP_REQ;
    if (ipsecmgr_snoop_sm_send_msg(&msg))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: offload_sp: sm_send_fail\n");
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_stop_offload
(
    ipsecmgr_snoop_stop_offload_req_param_t *param,
    void                                    *addr,
    uint32_t                                addr_size
)
{
    snoop_msg_t msg;
    struct snoop_offload_sp_stop_s *req = &msg.body.stop_offload;

    memset(req, 0, sizeof(*req));

    req->policy_id = param->policy_id;
    req->trans_id = param->trans_id;
    req->no_expire_sa = param->no_expire_sa;

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
        "snoop: recvd stop_offload for sp-id(%d) trans_id(%d), no_expire_sa(%d)\n",
        req->policy_id, req->trans_id,req->no_expire_sa);

    memcpy(req->addr, (uint8_t *)addr, addr_size);
    req->addr_size = addr_size;

    /* Trigger the state machine */
    msg.msg_id = MSG_STOP_OFFLOAD;
    if (ipsecmgr_snoop_sm_send_msg(&msg))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: stop_offload: sm_send_fail\n");
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_dump_state(void)
{
    snoop_msg_t msg;

    /* Trigger the state machine */
    msg.msg_id = MSG_DUMP_SM;
    if (ipsecmgr_snoop_sm_send_msg(&msg))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: dump_sm: sm_send_fail\n");
        return -1;
    }
    return 0;
}


int ipsecmgr_snoop_sa_expiry
(
    ipsecmgr_fp_handle_t    sa_handle,
    uint8_t                 hard,
    ipsecmgr_lft_cur_t      *lft
)
{
    snoop_msg_t msg;
    struct snoop_sa_exp_info_s *sa_info = &msg.body.sa_exp_info;

    memset(sa_info, 0, sizeof(*sa_info));

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
        "snoop: rcvd sa_expiry from FP for sa(0x%x) hard(%d)\n",
        sa_handle, hard);

    sa_info->sa_hndl = sa_handle;
    sa_info->hard = hard;
    sa_info->lft = *lft;

    /* Trigger the state machine */
    msg.msg_id = MSG_SA_EXPIRY_FP;
    if (ipsecmgr_snoop_sm_send_msg(&msg))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: sa_expiry: sm_send_fail\n");
        return -1;
    }

    return 0;
}

void ipsecmgr_snoop_shutdown(void)
{
    ipsecmgr_snoop_shutdown_timer_list();
    ipsecmgr_snoop_xfrm_shutdown();
    ipsecmgr_snoop_fp_shutdown();
    ipsecmgr_snoop_user_ipc_shutdown();
    ipsecmgr_snoop_sm_shutdown();
    ipsecmgr_snoop_mq_shutdown();
    return;
}

