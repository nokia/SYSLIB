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
 * File implementing state machine to handle offload SP request from
 * user application
*/

/* standard includes */

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

enum  offload_sp_state {
    STATE_INIT = 1,
    STATE_QUERY_SP = STATE_INIT,
    STATE_POLL_SA,
    STATE_QUERY_SA,
    STATE_CREATE_SA,
    STATE_ADD_SP,
    STATE_KNL_OFFLOAD,
    STATE_POLL_REKEY,
    STATE_END
};

enum offload_sp_event {
    EVENT_ENTER = 1,
    EVENT_EXIT,
    EVENT_TIMEOUT,
    EVENT_FP_DONE,
    EVENT_FP_RCV,
    EVENT_SA_NEW,
    EVENT_SA_EXPIRE,
    EVENT_SA_DEL,
    EVENT_STOP_OFFLD
};

/* Events that get reported during debug */
#define OFFLOAD_SP_VERBOSE_EVENTS   \
    ((1 << EVENT_ENTER)     | \
     (1 << EVENT_EXIT)      | \
     (1 << EVENT_TIMEOUT)   | \
     (1 << EVENT_FP_DONE)   | \
     (1 << EVENT_FP_RCV)    | \
     (1 << EVENT_SA_NEW)    | \
     (1 << EVENT_SA_EXPIRE) | \
     (1 << EVENT_SA_DEL)    | \
     (1 << EVENT_STOP_OFFLD))

static const char* offload_sp_state_names[] = {
    [STATE_QUERY_SP]    = "Query_SP",
    [STATE_POLL_SA]     = "Poll_SA",
    [STATE_QUERY_SA]    = "Query_SA",
    [STATE_CREATE_SA]   = "Create_SA",
    [STATE_ADD_SP]      = "Add_SP",
    [STATE_KNL_OFFLOAD] = "Knl_Offload",
    [STATE_POLL_REKEY]  = "Poll_Rekey",
    [STATE_END]         = "End"
};

static const char* offload_sp_event_names[] = {
    [EVENT_ENTER]       = "enter",
    [EVENT_EXIT]        = "exit",
    [EVENT_TIMEOUT]     = "timeout",
    [EVENT_FP_DONE]     = "FP_done_ack",
    [EVENT_FP_RCV]      = "FP_rcv_ack",
    [EVENT_SA_NEW]      = "SA_new",
    [EVENT_SA_EXPIRE]   = "SA_expire",
    [EVENT_SA_DEL]      = "SA_delete",
    [EVENT_STOP_OFFLD]  = "stop_offload"
};

struct offload_sp_ctx_s {
    enum offload_sp_state   state;
    uint8_t                 in_use;
    struct snoop_offload_sp_req_s *offload_sp_req;
    struct snoop_offload_sp_stop_s *stop_offload;

    int                     reqid;
    ipsecmgr_dir_t          dir;
    enum policy_type        type;

    struct xfrmnl_sp        *sp;
    struct sa_info_s        sa_info;
    struct sa_info_s        rekey_sa_info;
    struct snoop_sa_exp_info_s    exp_info;
    uint8_t                 in_rekey;

    ipsecmgr_fp_handle_t    sp_handle;
    ipsecmgr_fp_handle_t    sa_handle;
    ipsecmgr_fp_handle_t    rekey_sa_handle;
    ipsecmgr_trans_id_t     last_fp_trans_id; /* Last transaction id used with FP */
    ipsecmgr_result_t       fp_result; /* result of last transaction with FP */
    ipsecmgr_result_t       user_result; /* Offload SP req result to user */
    int32_t                 timer_handle; /* Handle for the current running timer */
};

/* Pool of state machine instances */
#define MAX_OFFLOAD_SP_SM   64
struct offload_sp_ctx_s offload_sp_sm[MAX_OFFLOAD_SP_SM] = {{0}};

static int allocate_state_instance(struct offload_sp_ctx_s **state);
static void free_state_instance(struct offload_sp_ctx_s *state);

/* State machine event handlers */
typedef int (*offload_sp_state_handler)(
        struct offload_sp_ctx_s* ctx, enum offload_sp_event event);

static int offload_sp_state_query_sp(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_query_sa(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_poll_sa(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_create_sa(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_add_sp(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_end(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_knl_offload(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static int offload_sp_state_poll_rekey(struct offload_sp_ctx_s *ctx,
        enum offload_sp_event event);

static offload_sp_state_handler offload_sp_state_handlers[] = {
    [STATE_QUERY_SP]    = offload_sp_state_query_sp,
    [STATE_POLL_SA]     = offload_sp_state_poll_sa,
    [STATE_QUERY_SA]    = offload_sp_state_query_sa,
    [STATE_CREATE_SA]   = offload_sp_state_create_sa,
    [STATE_ADD_SP]      = offload_sp_state_add_sp,
    [STATE_KNL_OFFLOAD] = offload_sp_state_knl_offload,
    [STATE_POLL_REKEY]  = offload_sp_state_poll_rekey,
    [STATE_END]         = offload_sp_state_end
};

/* Feed an event into the state machine */
static int offload_sp_send_event
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret;
    enum offload_sp_state   state = ctx->state;

    if ((1 << event) & OFFLOAD_SP_VERBOSE_EVENTS)
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: sm: send_event: handling \"%s\" event in \"%s\" state\n",
            offload_sp_event_names[event],
            offload_sp_state_names[state]);

    ret = (*offload_sp_state_handlers[state])(ctx, event);

    if (ret < 0)
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: sm: send_event: error handling \"%s\" event in \"%s\" state\n",
            offload_sp_event_names[event],
            offload_sp_state_names[state]);
    return ret;
}

/* Shift to a different state, emit enter/exit events along the way */
static int offload_sp_set_state
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_state newstate
)
{
    int ret;
    enum offload_sp_state   oldstate = ctx->state;

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: sm: set_state: switching from \"%s\" state to \"%s\" state\n",
            offload_sp_state_names[oldstate],
            offload_sp_state_names[newstate]);

    ret = offload_sp_send_event(ctx, EVENT_EXIT);
    if (ctx->state != oldstate) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: sm: set_state: state change during exit not allowed\n");
        return -1;
    }

    if (ret < 0) return ret;

    ctx->state = newstate;

    ret = offload_sp_send_event(ctx, EVENT_ENTER);
    if (ret < 0) return ret;

    return ret;
}

int offload_sp_dump_state(void)
{
    int                     index;
    struct offload_sp_ctx_s *ctx;

    /* Get the context from global state database */
    ctx = &offload_sp_sm[0];

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "IPSECMGR STATE: \n");

    /* Cycle through all in_use offloaded policy */
    for (index=0; index< MAX_OFFLOAD_SP_SM; index++)
    {
        if (ctx->in_use)
        {
            /* Print out policy info */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: State Info: [%2d]: state=%s, req_id=%d, dir=%d, spi=%x\n",
                    index, offload_sp_state_names[ctx->state],ctx->reqid, ctx->dir,
                    (ctx->sa_info.sa==NULL)? 0: xfrmnl_sa_get_spi(ctx->sa_info.sa));

            /* Print out sa/sp info */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm Info          policy=%d sa=%p, refCount=%d, spi=%x\n",
                    (ctx->offload_sp_req==NULL)? 0: ctx->offload_sp_req->policy_id,
                    ctx->sa_info.sa,
                    (ctx->sa_info.sa==NULL)? 0: ipsecmgr_snoop_xfrm_sa_getRefCount(ctx->sa_info.sa),
                    (ctx->sa_info.sa==NULL)? 0: xfrmnl_sa_get_spi(ctx->sa_info.sa));

            /* Print out rekey info */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: rekey Info:        rekey=%d, rekeySaHandle=%p, rekey_sa_info=%p, spi=%x\n",
                    ctx->in_rekey, ctx->rekey_sa_handle, ctx->rekey_sa_info,
                    (ctx->rekey_sa_info.sa==NULL)? 0:xfrmnl_sa_get_spi(ctx->rekey_sa_info.sa));

            /* Print out fp info */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: fp Info:           saHandle=%p, spHandle=%p\n",
                    ctx->sa_handle, ctx->sp_handle);
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "*******************************************************************************\n");
        }

        /* Netx Context */
        ctx++;
    }
    return 0;
}

static int find_sm_by_policy_id
(
    uint32_t                policy_id,
    struct offload_sp_ctx_s **ctx
)
{
    int i;
    struct offload_sp_ctx_s *state = &offload_sp_sm[0];

    for (i=0; i< MAX_OFFLOAD_SP_SM; i++) {
        if ((state->in_use) &&
            (state->offload_sp_req->policy_id == policy_id)) {
            *ctx = state;
            return 0;
        }
        state++;
    }
    return -1;
}

static int find_sm_by_sa_hndl
(
    ipsecmgr_fp_handle_t    sa_hndl,
    struct offload_sp_ctx_s **ctx
)
{
    int i;
    struct offload_sp_ctx_s *state = &offload_sp_sm[0];

    for (i=0; i< MAX_OFFLOAD_SP_SM; i++)
    {
        if (state->in_use)
        {
            if(state->sa_handle == sa_hndl)
            {
                *ctx = state;
                return 0;
            }
        }
        state++;
    }
    return -1;
}

static int find_sm_by_sp
(
    struct xfrmnl_sp        *sp,
    struct offload_sp_ctx_s **ctx
)
{
    int i;
    struct offload_sp_ctx_s *state = &offload_sp_sm[0];

    for (i=0; i< MAX_OFFLOAD_SP_SM; i++)
    {
        if (state->in_use)
        {
            if(!ipsecmgr_snoop_xfrm_sp_cmp(state->sp, sp))
            {
                *ctx = state;
                return 0;
            }
        }
        state++;
    }
    return -1;
}

static int find_sa_by_sp
(
    struct xfrmnl_sp        *sp,
    struct xfrmnl_sa        **sa
)
{
    int i;
    struct offload_sp_ctx_s *state = &offload_sp_sm[0];

    for (i=0; i< MAX_OFFLOAD_SP_SM; i++)
    {
        if ((state->in_use) && (state->sa_info.sa != NULL))
        {
            if(!ipsecmgr_snoop_xfrm_sp_compatible_sa(sp, state->sa_info.sa))
            {
                *sa = state->sa_info.sa;
                return 0;
            }
        }
        state++;
    }
    return -1;
}

static int find_sm_by_fp_resp
(
    struct snoop_fp_resp_s *resp,
    struct offload_sp_ctx_s **ctx
)
{
    int i;
    struct offload_sp_ctx_s *state = &offload_sp_sm[0];

    for (i=0; i< MAX_OFFLOAD_SP_SM; i++) {
        if ((state->in_use) &&
            (state->last_fp_trans_id == resp->trans_id)) {
            *ctx = state;
            return 0;
        }
        state++;
    }
    return -1;
}

int ipsecmgr_snoop_sm_send_msg(snoop_msg_t *msg)
{
    if (ipsecmgr_snoop_mq_send(msg, sizeof(*msg)))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: sm: send_msg: failed msg_id(%d)\n", msg->msg_id);
        return -1;
    }
    return 0;
}

static int ipsecmgr_snoop_sm_handle_msg
(
    snoop_msg_t *msg
)
{
    switch (msg->msg_id)
    {
        case MSG_NEW_SA:
        /* Identify the state and pass the parameter */
        {
            struct xfrmnl_sa *sa = msg->body.sa_info.sa;
            struct offload_sp_ctx_s *state = &offload_sp_sm[0];
            uint32_t    found = 0;
            int         index;

            /* Cycle all states looking for matching sa to rekey */
            for (index = 0; index < MAX_OFFLOAD_SP_SM; index++)
            {
                if ((state->in_use) && (!state->in_rekey))
                {
                    if(!ipsecmgr_snoop_xfrm_sp_compatible_sa(state->sp, sa))
                    {

                        /* Mark the state in "rekey" state and save sa */
                        state->in_rekey = 1;
                        state->rekey_sa_info.sa = sa;

                        /* The first state will use the SA from message,
                           Subsequent state will need to inherit the SA */
                        if(!found)
                        {
                            /* Use the sa as SA object and mark found to be 1 */
                            found = 1;
                        }
                        else
                        {
                            /* Inherit SA , increment the refCount */
                            ipsecmgr_snoop_xfrm_sa_inherit(sa);
                        }
                        /* Send the event to state */
                        offload_sp_send_event(state, EVENT_SA_NEW);
                    }
                }
                state++;
            }

            /* At least one policy should be found */
            if(!found)
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: handle_msg: ignoring NEW SA event, refCount=%d\n", ipsecmgr_snoop_xfrm_sa_getRefCount(sa));

                /* Compatible SP is not found, free the object */
                ipsecmgr_snoop_xfrm_sa_put(sa);

                return -1;
            }
            break;
        }

        case MSG_SA_EXPIRY_FP:
        /* Identify the state and pass the parameter */
        {
            struct snoop_sa_exp_info_s *sa_info = &msg->body.sa_exp_info;
            struct offload_sp_ctx_s *state;

            if (find_sm_by_sa_hndl(sa_info->sa_hndl, &state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: couldn't find associated \
                     state for sa_hndl(0x%x)\n", sa_info->sa_hndl);
                return -1;
            }

            state->exp_info = *sa_info;
            offload_sp_send_event(state, EVENT_SA_EXPIRE);
            break;
        }

        case MSG_DEL_SA:
        /* Identify the state and pass the parameter */
        {
            struct xfrmnl_sa *sa = msg->body.sa_info.sa;
            struct offload_sp_ctx_s *state = &offload_sp_sm[0];
            uint32_t found = 0;
            int  index;

            /* Cycle through all state looking for matching sa to be deleted */
            for (index = 0; index < MAX_OFFLOAD_SP_SM; index++)
            {
                if ((state->in_use) && (state->sa_info.sa))
                {
                    if(!ipsecmgr_snoop_xfrm_sa_cmp(state->sa_info.sa, sa))
                    {
                        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                            "snoop: sm: handle_msg: [%2d]send event DEL_SA event to policy %p for fp sa=%p\n", index, state->sp, state->sa_handle);

                        /* Set the found flag */
                        found = 1;

                        /* Send event to state */
                        offload_sp_send_event(state, EVENT_SA_DEL);
                    }
                }
                state++;
            }

            /* At least one should be found */
            if (!found)
            {
               snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: handle_msg: ignoring DEL_SA event\n");

                ipsecmgr_snoop_xfrm_sa_put(sa);

                return -1;
            }

            /* sa is deleted, call xfrm function to put sa */
            ipsecmgr_snoop_xfrm_sa_put(sa);
            break;
        }

        case MSG_DEL_SP:
        /* Identify the state and pass the parameter */
        {
            struct xfrmnl_sp *sp = msg->body.sp_info.sp;
            struct offload_sp_ctx_s *state;

            if (find_sm_by_sp(sp, &state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: handle_msg: ignoring DEL_SP event\n");

                ipsecmgr_snoop_xfrm_sp_put(sp);

                return -1;
            }

            ipsecmgr_snoop_xfrm_sp_put(sp);
            state->stop_offload = NULL;
            offload_sp_send_event(state, EVENT_STOP_OFFLD);
            break;
        }

        case MSG_OFFLOAD_SP_REQ:
        /* Start a new SM instance */
        {
            struct offload_sp_ctx_s *state;
            ipsecmgr_result_t result;
            struct snoop_offload_sp_req_s *req_info_m, *req_info = &msg->body.offload_sp;

            /* Check for duplicate policy identifier */
            if (!find_sm_by_policy_id(req_info->policy_id, &state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: policy-id (%d) already exists\n",
                       req_info->policy_id);
                result = RESULT_DUPL_SP;
                goto sp_req_err;
            }

            if (allocate_state_instance(&state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: Couldn't get any free state instance\n");
                result = RESULT_FAILURE;
                goto sp_req_err;
            }

            if ((req_info_m = malloc (sizeof(struct snoop_offload_sp_req_s))) == NULL) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: Failed to allocate space for OFFLOAD SP REQ\n");
                result = RESULT_FAILURE;
                goto sp_req_err;
            }

            memcpy(req_info_m, req_info, sizeof(*req_info));
            state->offload_sp_req = req_info_m;
            offload_sp_send_event(state, EVENT_ENTER);
            break;

sp_req_err:
            ipsecmgr_snoop_user_ipc_send_offload_sp_done (req_info, NULL,
                                                           NULL, result, 0);
                return -1;
        }

        case MSG_CREATE_SA_RSP:
        case MSG_ADD_SP_RSP:
        /* Identify the state and pass the parameter */
        {
            struct snoop_fp_resp_s *fp_resp = &msg->body.fp_resp;
            struct offload_sp_ctx_s *state;

            if (find_sm_by_fp_resp(fp_resp, &state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: No state machine found for trans_id (0x%x)\n",
                    fp_resp->trans_id);
                return -1;
            }

            if (fp_resp->valid_fp_handle) {
                if (msg->msg_id == MSG_CREATE_SA_RSP) {
                    state->sa_handle = fp_resp->fp_handle;
                } else {
                    state->sp_handle = fp_resp->fp_handle;
                }
            }

            state->fp_result = fp_resp->result;

            if (fp_resp->type & RSP_TYPE_DONE) {
                offload_sp_send_event(state, EVENT_FP_DONE);
            } else {
                offload_sp_send_event(state, EVENT_FP_RCV);
            }
            break;
        }

        case MSG_TIMEOUT:
        {
            struct offload_sp_ctx_s *state = (struct offload_sp_ctx_s *)msg->body.timeout.cookie;
            if (state->in_use)
                offload_sp_send_event(state, EVENT_TIMEOUT);
            break;
        }

        case MSG_STOP_OFFLOAD:
        {
            struct offload_sp_ctx_s *state;
            ipsecmgr_result_t result;
            struct snoop_offload_sp_stop_s *req_info_m, *req_info = &msg->body.stop_offload;

            /* Check for duplicate policy identifier */
            if (find_sm_by_policy_id(req_info->policy_id, &state)) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: policy-id (%d) doesn't exist\n",
                       req_info->policy_id);
                result = RESULT_NO_OFFLD_SP;
                goto stop_off_err;
            }
            if ((req_info_m = malloc (sizeof(struct snoop_offload_sp_stop_s))) == NULL) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: handle_msg: Failed to allocate space for STOP_OFF REQ\n");
                result = RESULT_FAILURE;
                goto stop_off_err;
            }
            memcpy(req_info_m, req_info, sizeof(*req_info));
            state->stop_offload = req_info_m;
            offload_sp_send_event(state, EVENT_STOP_OFFLD);
            break;
stop_off_err:
             ipsecmgr_snoop_user_ipc_send_stop_offload_done(req_info,result);
             return -1;
        }
        case MSG_DUMP_SM:
        {
            offload_sp_dump_state();
            break;
        }
        default:
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                      "snoop: sm: handle_msg: unknown msg %d\n", msg->msg_id);
        }
    }
    return 0;
}

#define SNOOP_SM_NUM_MSG_TO_POLL 4
void ipsecmgr_snoop_sm_poll_msg()
{
    snoop_msg_t msg;
    int num_msgs = 0;
    uint32_t len = sizeof(msg);

    while (num_msgs < SNOOP_SM_NUM_MSG_TO_POLL)
    {
        if (ipsecmgr_snoop_mq_recv(&msg, &len))
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: sm: poll_msg: recv failed\n");
            return;
        }

        if (len)
        {
            ipsecmgr_snoop_sm_handle_msg(&msg);
        }
        else
        {
            break;
        }
        num_msgs++;
    }
    return;
}

/*****************************************************************************/
/*  State machine handler routines                                           */
/*****************************************************************************/

#define KERNEL_RSP_TIMEOUT  500
#define FP_RSP_TIMEOUT      1500
#define QUERY_SP_TIMEOUT    KERNEL_RSP_TIMEOUT
#define QUERY_SA_TIMEOUT    KERNEL_RSP_TIMEOUT
#define CREATE_SA_TIMEOUT   FP_RSP_TIMEOUT
#define ADD_SP_TIMEOUT      FP_RSP_TIMEOUT

static int offload_sp_state_query_sp
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            if (ipsecmgr_snoop_xfrm_sp_get(ctx->offload_sp_req->policy_id,
                                           &ctx->sp, &ctx->dir, &ctx->type))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: query_sp: sp_get failed\n");
                /* Set result and go to end state */
                ctx->user_result = RESULT_GET_SP_FAIL;
                offload_sp_set_state(ctx, STATE_END);
                break;
            }
            if (ctx->type == POLICY_NONE) {
                offload_sp_set_state(ctx, STATE_ADD_SP);
            } else {
                offload_sp_set_state(ctx, STATE_QUERY_SA);
            }
            break;
        }

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
        case EVENT_STOP_OFFLD:
        case EVENT_TIMEOUT:
        case EVENT_FP_DONE:
        case EVENT_FP_RCV:
            ret = -1;
            break;

        case EVENT_EXIT:
            break;


        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: sm: query_sp: unknown event\n");
            ret = -1;
        break;
    }
    return ret;
}

static int offload_sp_state_query_sa
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            /* Find sa in state machine database */
            if(find_sa_by_sp(ctx->sp, &ctx->sa_info.sa) == 0)
            {
                /* Found compatible SA, Save reqid */
                ctx->reqid = xfrmnl_sa_get_reqid(ctx->sa_info.sa);

                /* Inherit the SA */
                ipsecmgr_snoop_xfrm_sa_inherit(ctx->sa_info.sa);

                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: query_sa: Found matching sa=%p, reqid=%d\n", ctx->sa_info.sa, ctx->reqid);
            }
            /* Create sa from sp templates */
            else if (ipsecmgr_snoop_xfrm_sa_get(ctx->sp, &ctx->sa_info.sa, &ctx->reqid))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: query_sa: sa_get failed\n");
                /* Set result and go to end state */
                ctx->user_result = RESULT_GET_SA_FAIL;
                offload_sp_set_state(ctx, STATE_END);
                break;
            }
            else
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: query_sa: sa_get succeeded with sa=%p, refCount=%d\n",
                     ctx->sa_info.sa, ipsecmgr_snoop_xfrm_sa_getRefCount(ctx->sa_info.sa));
            }

            /* Moving to next State */
            offload_sp_set_state(ctx, STATE_CREATE_SA);
            break;
        }

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
        case EVENT_TIMEOUT:
        case EVENT_FP_DONE:
        case EVENT_FP_RCV:
            ret = -1;
            break;

        case EVENT_STOP_OFFLD:
            offload_sp_set_state(ctx, STATE_END);
            break;

        case EVENT_EXIT:
            break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: sm: query_sa: unknown event\n");
            ret = -1;
        break;
    }
    return ret;
}

static int offload_sp_state_create_sa
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            ipsecmgr_fp_handle_t *fp_sa_hndl;
            struct sa_info_s *sa_params;
            ipsecmgr_sa_flags_t sa_flags = ctx->offload_sp_req->sa_flags;

            if (ctx->in_rekey)
            {
                sa_params = &ctx->rekey_sa_info;
                fp_sa_hndl = &ctx->rekey_sa_handle;
                sa_flags |= IPSECMGR_SA_FLAGS_REKEY;
            }
            else
            {
                sa_params = &ctx->sa_info;
                fp_sa_hndl = &ctx->sa_handle;
            }

            if (ipsecmgr_snoop_fp_create_sa(sa_params,
                                            sa_flags,
                                            ctx->dir,
                                            &ctx->offload_sp_req->dscp_map_cfg,
                                            &ctx->offload_sp_req->ifname,
                                            &ctx->last_fp_trans_id,
                                            fp_sa_hndl,
                                            (unsigned int *)&ctx->fp_result))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: create_sa: fp_create_sa failed\n");
                /* Set result and go to end state */
                ctx->user_result = RESULT_ADD_SA_FAIL;
                /* For failure case, handle should already be NULL */
                if(ctx->in_rekey)
                    ctx->rekey_sa_handle = (ipsecmgr_fp_handle_t)NULL;
                else
                    ctx->sa_handle = (ipsecmgr_fp_handle_t)NULL;

                offload_sp_set_state(ctx, STATE_END);
                break;
            }
            else
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: create_sa: fp_create_sa succeeded with fp_sa_handle=%p, spi=%x\n",
                    *fp_sa_hndl, xfrmnl_sa_get_spi(sa_params->sa));
            }
#ifndef SYNCHRONOUS_NETCP_CFG_API
            /* Start timer for response from FP */
            if (ipsecmgr_snoop_start_timer(CREATE_SA_TIMEOUT,
                                           ctx, &ctx->timer_handle))
            {
                ctx->user_result = RESULT_ADD_SA_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            }
#else
            offload_sp_set_state(ctx, STATE_ADD_SP);
#endif
            break;
        }

        case EVENT_EXIT:
        break;

        case EVENT_TIMEOUT:
            ctx->user_result = RESULT_ADD_SA_FAIL;
            offload_sp_set_state(ctx, STATE_END);
        break;

        case EVENT_FP_RCV:
            /* Cancel FP_RCV timeout */
            ipsecmgr_snoop_stop_timer(ctx->timer_handle);
            if (ctx->fp_result != RESULT_SUCCESS) {
                /* Set result and go to end state */
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: create_sa: create_sa failed on FP (%d)\n",
                    ctx->fp_result);
                ctx->user_result = RESULT_ADD_SA_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            } else {
                /* Start timeout for FP_DONE */
                if (ipsecmgr_snoop_start_timer(CREATE_SA_TIMEOUT,
                                               ctx, &ctx->timer_handle)) {
                    ctx->user_result = RESULT_ADD_SA_FAIL;
                    offload_sp_set_state(ctx, STATE_END);
                }
            }
        break;

        case EVENT_FP_DONE:
        {
            /* Cancel FP_DONE timeout */
            ipsecmgr_snoop_stop_timer(ctx->timer_handle);
            if (ctx->fp_result != RESULT_SUCCESS) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: create_sa failed on FP (%d)\n",
                    ctx->fp_result);
                /* Set result and go to end state */
                ctx->user_result = RESULT_ADD_SA_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            } else {
                offload_sp_set_state(ctx, STATE_ADD_SP);
            }
        }
        break;

        case EVENT_STOP_OFFLD:
            offload_sp_set_state(ctx, STATE_END);
            break;

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
            ret= -1;
            break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: sm: create_sa: unknown event\n");
            ret = -1;
        break;
    }
    return ret;
}

static int offload_sp_state_add_sp
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            ipsecmgr_l5_selector_t  *l5_sel = NULL;

            if (ctx->in_rekey) {
                goto add_sp_skip;
            }


            if (ctx->offload_sp_req->l5_sel.proto) {
                l5_sel = &ctx->offload_sp_req->l5_sel;
            }

            if (ipsecmgr_snoop_fp_add_sp(ctx->sp,
                                         l5_sel,
                                         ctx->reqid,
                                         ctx->sa_handle,
                                         &ctx->last_fp_trans_id,
                                         &ctx->sp_handle,
                                         (unsigned int *)&ctx->fp_result))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: add_sp: ipsecmgr_fp_add_sp failed\n");
                /* Set result and go to end state */
                ctx->user_result = RESULT_ADD_SP_FAIL;
                offload_sp_set_state(ctx, STATE_END);
                break;
            }
add_sp_skip:
#ifndef SYNCHRONOUS_NETCP_CFG_API
            /* Start timer for response from FP */
            if (ipsecmgr_snoop_start_timer(ADD_SP_TIMEOUT,ctx,
                                           &ctx->timer_handle)) {
                ctx->user_result = RESULT_ADD_SP_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            }
#else
            ctx->user_result = RESULT_SUCCESS;
            if ((ctx->type != POLICY_NONE) &&
                (ctx->offload_sp_req->sa_flags & IPSECMGR_SA_FLAGS_SHARED)) {
                offload_sp_set_state(ctx, STATE_KNL_OFFLOAD);
            } else {
                offload_sp_set_state(ctx, STATE_POLL_REKEY);
            }
#endif
        }
        break;

        case EVENT_EXIT:
        break;

        case EVENT_TIMEOUT:
            ctx->user_result = RESULT_ADD_SP_FAIL;
            offload_sp_set_state(ctx, STATE_END);
        break;

        case EVENT_FP_RCV:
        {
            /* Cancel FP_RCV timeout */
            ipsecmgr_snoop_stop_timer(ctx->timer_handle);

            if (ctx->fp_result != RESULT_SUCCESS) {
                /* Set result and go to end state */
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: add_sp failed on FP (%d)\n", ctx->fp_result);
                ctx->user_result = RESULT_ADD_SP_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            } else {
                /* Start timeout for FP_DONE */
                if (ipsecmgr_snoop_start_timer(CREATE_SA_TIMEOUT,
                                               ctx, &ctx->timer_handle)) {
                    ctx->user_result = RESULT_ADD_SP_FAIL;
                    offload_sp_set_state(ctx, STATE_END);
                }
            }
        }
        break;

        case EVENT_FP_DONE:
        {
            /* Cancel FP_DONE timeout */
            ipsecmgr_snoop_stop_timer(ctx->timer_handle);

            if (ctx->fp_result != RESULT_SUCCESS) {
                /* Set result and go to end state */
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: add_sp failed on FP (%d)\n", ctx->fp_result);
                ctx->user_result = RESULT_ADD_SP_FAIL;
                offload_sp_set_state(ctx, STATE_END);
            } else {
                /* Set result and go to end state */
                ctx->user_result = RESULT_SUCCESS;
                if (ctx->offload_sp_req->sa_flags & IPSECMGR_SA_FLAGS_SHARED) {
                    offload_sp_set_state(ctx, STATE_KNL_OFFLOAD);
                } else {
                    offload_sp_set_state(ctx, STATE_POLL_REKEY);
                }
            }
        }
        break;

        case EVENT_STOP_OFFLD:
            offload_sp_set_state(ctx, STATE_END);
            break;

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
            ret =-1;
            break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                      "snoop: sm: unknown event\n");
            ret = -1;
        break;
    }
    return ret;
}

static int offload_sp_state_knl_offload
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            ipsecmgr_fp_handle_t fp_sa_hndl;
            struct sa_info_s *sa_params;

            if (ctx->in_rekey)
            {
                sa_params = &ctx->rekey_sa_info;
                fp_sa_hndl = ctx->rekey_sa_handle;
            }
            else
            {
                sa_params = &ctx->sa_info;
                fp_sa_hndl = ctx->sa_handle;
            }

            if (ipsecmgr_snoop_knl_mod_open()) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: knl_offload: failed to open mod interface\n");
                goto knl_offload_enter_fail;
            }

            if (ctx->dir == DIR_OUTBOUND) {
                if (ipsecmgr_snoop_fp_get_sa_hw_ctx(fp_sa_hndl, sa_params,
                                                    (unsigned int *)&ctx->fp_result))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                        "snoop: sm: knl_offload: fp_get_sa_hw_ctx failed\n");
                    goto knl_offload_enter_fail;
                }
            }

            if (ipsecmgr_snoop_knl_set_ctx(sa_params, ctx->dir))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: knl_offload: knl_set_ctx failed\n");
                goto knl_offload_enter_fail;
            }

            ctx->user_result = RESULT_SUCCESS;
            offload_sp_set_state(ctx, STATE_POLL_REKEY);
            break;

knl_offload_enter_fail:
            /* Set result and go to end state */
            ctx->user_result = RESULT_KNL_OFFLOAD_FAIL;
            if (ctx->in_rekey) {
                offload_sp_set_state(ctx, STATE_POLL_REKEY);
            } else {
                offload_sp_set_state(ctx, STATE_END);
            }
            break;
        }

        case EVENT_EXIT:
            ipsecmgr_snoop_knl_mod_close();
        break;

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
        case EVENT_TIMEOUT:
        case EVENT_FP_RCV:
        case EVENT_FP_DONE:
            ret = -1;
        break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                      "snoop: sm: unknown event\n");
            ret = -1;
        break;
    }
    return ret;
}

static int offload_sp_state_poll_rekey
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
            if (ctx->in_rekey) {
                ipsecmgr_snoop_user_ipc_send_rekey_event(
                            ctx->offload_sp_req,
                            ctx->user_result,
                            ctx->rekey_sa_handle);
#if 0
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: poll_rekey for rekeying: fp_create_sa succeeded with fp_sa_handle=%p, spi=%x\n",
                    ctx->rekey_sa_handle, xfrmnl_sa_get_spi(ctx->rekey_sa_info.sa));

                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: sm: poll_rekey for old sa:  fp_sa_handle=%p, spi=%x\n",
                    ctx->sa_handle, xfrmnl_sa_get_spi(ctx->sa_info.sa));
#endif
            } else {
                ipsecmgr_snoop_user_ipc_send_offload_sp_done(
                            ctx->offload_sp_req,
                            &ctx->sp_handle,
                            &ctx->sa_handle,
                            RESULT_SUCCESS,
                            0);
            }
            break;
        }

        case EVENT_SA_NEW:
            offload_sp_set_state(ctx, STATE_CREATE_SA);
            break;

        case EVENT_SA_EXPIRE:
        {
            ret = ipsecmgr_snoop_xfrm_update_lft(ctx->sa_info.sa,
                                           &ctx->exp_info.lft);
            if (ret) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: rekey: update_lft failed\n");
                ret = -1;
                break;
            }

            if (ipsecmgr_snoop_knl_mod_open()) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: rekey: failed to open mod interface\n");

            } else {
                if (ipsecmgr_snoop_knl_check_sa_expiry(&ctx->sa_info))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                        "snoop: sm: rekey: chk_expiry failed\n");
                    ret = -1;
                }
                ipsecmgr_snoop_knl_mod_close();
                break;
            }

            break;
        }

        case EVENT_SA_DEL:
        {
            if (ctx->sa_handle)
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: poll_rekey calling fp_del_sa: fp_sa_handle=%p\n",
                    ctx->sa_handle);

                ipsecmgr_snoop_fp_del_sa(ctx->reqid, ctx->dir, ctx->sa_handle);
                ctx->sa_handle = 0;
            }

            if (!(ctx->offload_sp_req->sa_flags & IPSECMGR_SA_FLAGS_SHARED)) {
                goto rekey_skip_knl_del_ctx;
            }

            if (ipsecmgr_snoop_knl_mod_open()) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: rekey: failed to open mod interface\n");

            } else {
                if (ctx->sa_info.sa) {
                    if (ipsecmgr_snoop_knl_del_ctx(&ctx->sa_info, ctx->dir))
                    {
                        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: sm: rekey: knl_del_ctx failed\n");
                    }
                }
                ipsecmgr_snoop_knl_mod_close();
            }

rekey_skip_knl_del_ctx:
            /* Free the old SA context */
            ipsecmgr_snoop_xfrm_sa_put(ctx->sa_info.sa);
            memset(&ctx->sa_info, 0, sizeof(ctx->sa_info));

            ctx->sa_handle = ctx->rekey_sa_handle;
            ctx->rekey_sa_handle = 0;
            ctx->sa_info = ctx->rekey_sa_info;
            memset(&ctx->rekey_sa_info, 0, sizeof(ctx->rekey_sa_info));
            ctx->in_rekey = 0;

            break;
        }

        case EVENT_STOP_OFFLD:
            offload_sp_set_state(ctx, STATE_END);
            break;

        case EVENT_TIMEOUT:
        case EVENT_FP_DONE:
        case EVENT_FP_RCV:
        default:
            ret = -1;
            break;

        case EVENT_EXIT:
            break;
    }
    return ret;
}

/* "poll_sa" state is not being used for now as the current requirement is that
   Security policy offload will be called only after SA is established
   by the IKE daemon.
*/
static int offload_sp_state_poll_sa
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    return 0;
}

static int offload_sp_state_end
(
    struct offload_sp_ctx_s *ctx,
    enum offload_sp_event event
)
{
    int ret = 0;

    switch (event) {

        case EVENT_ENTER:
        {
           if (ctx->sa_handle) {
                ipsecmgr_snoop_fp_del_sa(ctx->reqid, ctx->dir, ctx->sa_handle);
            }

            if (ctx->rekey_sa_handle) {
                ipsecmgr_snoop_fp_del_sa(ctx->reqid, ctx->dir, ctx->rekey_sa_handle);
            }

            if (ctx->sp_handle) {
                ipsecmgr_snoop_fp_del_sp(ctx->sp_handle,
                                         ctx->offload_sp_req->policy_id,
                                         ctx->reqid,
                                         ctx->dir);
            }

            if (!(ctx->offload_sp_req->sa_flags & IPSECMGR_SA_FLAGS_SHARED)) {
                goto end_bypass_knl_del_ctx;
            }

            if ((!ctx->sa_info.sa) && (!ctx->rekey_sa_info.sa)) {
                goto end_bypass_knl_del_ctx;
            }

            if (ipsecmgr_snoop_knl_mod_open()) {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: sm: end: failed to open mod interface\n");

            } else {
                if (ctx->sa_info.sa) {
                    if (ipsecmgr_snoop_knl_del_ctx(&ctx->sa_info, ctx->dir))
                    {
                        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: sm: end: knl_del_ctx failed-1\n");
                    }
                }
                if (ctx->rekey_sa_info.sa) {
                    if (ipsecmgr_snoop_knl_del_ctx(&ctx->rekey_sa_info, ctx->dir))
                    {
                        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: sm: end: knl_del_ctx failed-2\n");
                    }
                }
                ipsecmgr_snoop_knl_mod_close();
            }

end_bypass_knl_del_ctx:
            if (ctx->user_result != RESULT_SUCCESS)
            {

                ipsecmgr_snoop_user_ipc_send_offload_sp_done (
                                    ctx->offload_sp_req,
                                    NULL, NULL, ctx->user_result,
                                    ctx->fp_result);
            }
            else /* Stop offload command */
            {
                if (ctx->stop_offload == NULL)
                {
                    /* DEL_SP event */
                    free_state_instance(ctx);
                    break;
                }
                if (!ctx->stop_offload->no_expire_sa)
                {
                    /* Trigger expiry for the SA */
                    do
                    {
                        struct sa_info_s *sa_info;
                        ipsecmgr_lft_cur_t clft;
                        ipsecmgr_lft_cfg_t lft_cfg;

                        if (ctx->rekey_sa_info.sa)
                        {
                            sa_info = &ctx->rekey_sa_info;
                        }
                        else if (ctx->sa_info.sa)
                        {
                            sa_info = &ctx->sa_info;
                        }
                        else
                        {
                            break;
                        }

                        if (ipsecmgr_snoop_xfrm_get_lft_cfg(sa_info->sa, &lft_cfg))
                        {
                            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                "snoop: sm: end: no lifetime config\n");
                            break;
                        }

                        /* Soft expiry */
                        clft.bytes = lft_cfg.soft_byte_limit;
                        clft.packets = lft_cfg.soft_packet_limit;

                        if (ipsecmgr_snoop_xfrm_update_lft(sa_info->sa,
                                                           &clft))
                        {
                            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: sm: end: update_lft failed\n");
                            break;
                        }

                        if (ipsecmgr_snoop_knl_mod_open())
                        {
                            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: sm: end: failed to open mod interface\n");
                        }
                        else
                        {
                            if (ipsecmgr_snoop_knl_check_sa_expiry(sa_info))
                            {
                                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                "snoop: sm: end: chk_expiry failed\n");
                            }
                            ipsecmgr_snoop_knl_mod_close();
                        }

                    } while(0);
                }

                /* Stop offload response */
                ipsecmgr_snoop_user_ipc_send_stop_offload_done(ctx->stop_offload,
                                                               RESULT_SUCCESS);
            }

           free_state_instance(ctx);
        }
        break;

        case EVENT_SA_NEW:
        case EVENT_SA_EXPIRE:
        case EVENT_SA_DEL:
        case EVENT_EXIT:
        case EVENT_TIMEOUT:
        case EVENT_FP_DONE:
        case EVENT_FP_RCV:
            ret = -1;
            break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                                      "snoop: sm: unknown event\n");
            ret = -1;
            break;
    }
    return ret;
}

/*****************************************************************************/
/*  State allocator                                                          */
/*****************************************************************************/
static int allocate_state_instance
(
    struct offload_sp_ctx_s **state
)
{
    int i;

    for (i=0; i<MAX_OFFLOAD_SP_SM; i++) {
        if (!offload_sp_sm[i].in_use) {
            *state = &offload_sp_sm[i];
            (*state)->in_use = 1;
            (*state)->state = STATE_INIT;
            return 0;
        }
    }
    return -1;
}

static void free_state_instance
(
    struct offload_sp_ctx_s *state
)
{

    if (state->offload_sp_req)
    {
        free(state->offload_sp_req);
    }

    if (state->stop_offload)
    {
        free(state->stop_offload);
    }

    if (state->sp)
    {
        ipsecmgr_snoop_xfrm_sp_put(state->sp);
    }

    if (state->sa_info.sa)
    {
        ipsecmgr_snoop_xfrm_sa_put(state->sa_info.sa);
    }

    if (state->rekey_sa_info.sa)
    {
        ipsecmgr_snoop_xfrm_sa_put(state->rekey_sa_info.sa);
    }
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: sm: Free_state_instance is done for state=%p\n", state);

    memset(state, 0, sizeof(struct offload_sp_ctx_s));
    return;
}

int ipsecmgr_snoop_sm_init()
{
    memset(&offload_sp_sm, 0, sizeof(offload_sp_sm));
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: sm: initialized\n");
    return 0;
}

void ipsecmgr_snoop_sm_shutdown()
{
    memset(&offload_sp_sm, 0, sizeof(offload_sp_sm));
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: sm: shutdown\n");
    return;
}

