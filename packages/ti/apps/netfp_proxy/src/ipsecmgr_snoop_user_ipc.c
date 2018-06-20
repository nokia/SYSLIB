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

/*
 * File implementing functions to handle offload SP request/response from
 * user application using ipsecmgr ipc library
*/

/* standard includes */

/* ipsecmgr includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop.h>

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

int ipsecmgr_snoop_user_ipc_send_offload_sp_done
(
    struct snoop_offload_sp_req_s *req,
    ipsecmgr_fp_handle_t    *sp_handle,
    ipsecmgr_fp_handle_t    *sa_handle,
    ipsecmgr_result_t       result,
    uint32_t                fp_result
)
{
    ipsecmgr_snoop_offload_sp_rsp_param_t resp_params;
    int ret;

    memset(&resp_params, 0, sizeof(resp_params));

    resp_params.result = result;
    resp_params.type = RSP_TYPE_DONE;
    resp_params.trans_id = req->trans_id;
    resp_params.err_code = fp_result;

    if (sp_handle) {
        resp_params.sp_handle = *sp_handle;
        resp_params.sa_handle = *sa_handle;
    }

    ret = snoop_ctx.mgnt_cb.offload_sp_rsp (&resp_params, (void *)req->addr,
                                           req->addr_size);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
        "snoop: user_ipc: failed to send ack done for offload_sp spi_id(%d) \
         error=%d\n", req->policy_id, ret);
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_user_ipc_send_stop_offload_done
(
    struct snoop_offload_sp_stop_s *req,
    ipsecmgr_result_t       result
)
{
    ipsecmgr_snoop_stop_offload_rsp_param_t resp_params;
    int ret;

    memset(&resp_params, 0, sizeof(resp_params));

    resp_params.type = RSP_TYPE_DONE;
    resp_params.result = result;
    resp_params.trans_id = req->trans_id;

    ret = snoop_ctx.mgnt_cb.stop_offload_rsp (&resp_params, (void *)req->addr,
                                           req->addr_size);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
        "snoop: user_ipc: failed to send ack for stop_offload sp_id(%d) error=%d\n",
        req->policy_id, ret);
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_user_ipc_send_rekey_event
(
    struct snoop_offload_sp_req_s *req,
    ipsecmgr_result_t       result,
    ipsecmgr_fp_handle_t    sa_handle
)
{
    ipsecmgr_snoop_rekey_event_param_t resp_params;
    int ret;

    if (!snoop_ctx.mgnt_cb.rekey_event)
    {
        /* callback not registered, hence ignore */
        return 0;
    }

    memset(&resp_params, 0, sizeof(resp_params));

    resp_params.result = result;
    resp_params.policy_id = req->policy_id;
    resp_params.rekey_sa_handle = sa_handle;

    ret = snoop_ctx.mgnt_cb.rekey_event (&resp_params, (void *)req->addr,
                                         req->addr_size);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
        "snoop: user_ipc: failed to send rekey event for sp_id(%d) error=%d\n",
        req->policy_id, ret);
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_user_ipc_init()
{
    return 0;
}

void ipsecmgr_snoop_user_ipc_shutdown()
{
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: user_ipc: shutdown\n");
    return;
}

