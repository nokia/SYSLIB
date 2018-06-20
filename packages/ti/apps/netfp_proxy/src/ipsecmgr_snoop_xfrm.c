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

#include <errno.h>
#include <inttypes.h>

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

static int cmp_subnet
(
    struct sockaddr *a1,
    struct sockaddr *a2,
    uint8_t         prefix
)
{
    struct sockaddr_in  *s4_1, *s4_2;
    struct sockaddr_in6 *s6_1, *s6_2;
    uint32_t mask4;
    uint8_t  mask6, i;

    switch (a1->sa_family) {
        case AF_INET:
            s4_1 = (struct sockaddr_in *)a1;
            s4_2 = (struct sockaddr_in *)a2;
            mask4 = (1<<prefix) -1;

            if ((s4_1->sin_addr.s_addr & mask4) ==
                (s4_2->sin_addr.s_addr & mask4)) {
                return 0;
            } else {
                return -1;
            }
        break;

        case AF_INET6:
            s6_1 = (struct sockaddr_in6 *)a1;
            s6_2 = (struct sockaddr_in6 *)a2;
            for (i=0; i<prefix/8; i++) {
                if (s6_1->sin6_addr.s6_addr[i] == s6_2->sin6_addr.s6_addr[i]) {
                    continue;
                }
                return -1;
            }
            if ((i < 15) && (prefix%8)) {
                mask6 = (1<< (prefix%8)) -1;
                if ((s6_1->sin6_addr.s6_addr[i] & mask6) ==
                    (s6_2->sin6_addr.s6_addr[i] & mask6)) {
                    return 0;
                }
            } else {
                return 0;
            }
            return -1;
        break;

        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: xfrm: cmp_subnet: unknown-af\n");
            return -1;
        break;
    }
}
int ipsecmgr_snoop_xfrm_get_oseq
(
    struct xfrmnl_sa    *sa,
    unsigned int        *oseq,
    unsigned int        *oseq_hi
)
{
    struct xfrmnl_ae *ae;
    struct nl_addr *addr;
    int spi, proto, retval, flags;
    unsigned int seq, replay_window, bmp, bmp_len;
    char tmpbuf [256];

    spi = xfrmnl_sa_get_spi(sa);
    proto = xfrmnl_sa_get_proto(sa);
    addr = xfrmnl_sa_get_daddr(sa);

    flags = xfrmnl_sa_get_flags(sa);
    if (flags == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_seq: no flags\n");
        return -1;
    }

    /* Get the current SA stats and info from the kernel */
    if ((retval = xfrmnl_ae_get_kernel(snoop_ctx.xfrm_ctx.nl_sock, addr, spi, proto, 0, 0, &ae)) < 0)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_seq: Error getting AE for SPI: 0x%x addr: %s protocol: %d \n",
            spi, nl_addr2str(addr, tmpbuf, 256), proto);
        return -1;
    }

    if (flags & XFRM_STATE_ESN) {
        retval = xfrmnl_ae_get_replay_state_esn(ae, oseq,&seq, &seq,
                                                oseq_hi,&replay_window,
                                                &bmp_len, &bmp);
    }
    else {
        retval = xfrmnl_ae_get_replay_state(ae,oseq,&seq,&bmp);
    }
    if (retval) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_seq: Error getting replay_state\n");
        return -1;
    }

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
        "snoop: xfrm: get_seq: oseq: 0x%x, seq: 0x%x, bmp: 0x%x\n",
         oseq, seq, bmp);

    return 0;
}

int ipsecmgr_snoop_xfrm_get_lft_cur
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cur_t  *clft
)
{
    struct xfrmnl_ae *ae;
    struct nl_addr *addr;
    int spi, proto, retval;
    char tmpbuf [256];

    spi = xfrmnl_sa_get_spi(sa);
    proto = xfrmnl_sa_get_proto(sa);
    addr = xfrmnl_sa_get_daddr(sa);

    /* Get the current SA stats and info from the kernel */
    if ((retval = xfrmnl_ae_get_kernel(snoop_ctx.xfrm_ctx.nl_sock, addr, spi, proto, 0, 0, &ae)) < 0)
	{
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_lft_cur: Error getting AE for SPI: 0x%x addr: %s protocol: %d \n", spi, nl_addr2str(addr, tmpbuf, 256), proto);
        return -1;
    }

    retval = xfrmnl_ae_get_curlifetime(ae, &clft->bytes, &clft->packets,
                                       &clft->add_time, &clft->use_time);
    if (retval) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_lft_cur: Error getting clft\n");
        return -1;
    }

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
        "snoop: xfrm: get_lft_cur: bytes(%"PRIu64") pkts(%"PRIu64") addtime(%"PRIu64") usetime(%"PRIu64")\n",
        clft->bytes, clft->packets, clft->add_time, clft->use_time);

    return 0;
}

int ipsecmgr_snoop_xfrm_get_lft_cfg
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cfg_t  *lft_cfg
)
{
    struct xfrmnl_ltime_cfg *lft;

    /* Lifetime configuration */
    lft = xfrmnl_sa_get_lifetime_cfg(sa);

    if (!lft)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: get_lft_cfg: no lifetime config\n");
        return -1;
    }

    lft_cfg->soft_byte_limit = xfrmnl_ltime_cfg_get_soft_bytelimit(lft);
    lft_cfg->hard_byte_limit = xfrmnl_ltime_cfg_get_hard_bytelimit(lft);
    lft_cfg->soft_packet_limit = xfrmnl_ltime_cfg_get_soft_packetlimit(lft);
    lft_cfg->hard_packet_limit = xfrmnl_ltime_cfg_get_hard_packetlimit(lft);
    lft_cfg->soft_add_expires = xfrmnl_ltime_cfg_get_soft_addexpires(lft);
    lft_cfg->hard_add_expires = xfrmnl_ltime_cfg_get_hard_addexpires(lft);
    lft_cfg->soft_use_expires = xfrmnl_ltime_cfg_get_soft_useexpires(lft);
    lft_cfg->hard_use_expires = xfrmnl_ltime_cfg_get_hard_useexpires(lft);

    return 0;
}

int ipsecmgr_snoop_xfrm_update_lft
(
    struct xfrmnl_sa    *sa,
    ipsecmgr_lft_cur_t  *clft
)
{
    struct xfrmnl_ae *ae, *ae_new;
    struct nl_addr *addr;
    int spi, proto, retval;
    uint64_t bytes, packets, addtime, usetime;
    char tmpbuf [256];

    spi = xfrmnl_sa_get_spi(sa);
    proto = xfrmnl_sa_get_proto(sa);
    addr = xfrmnl_sa_get_daddr(sa);

    /* Get the current SA stats and info from the kernel */
    if ((retval = xfrmnl_ae_get_kernel(snoop_ctx.xfrm_ctx.nl_sock, addr, spi, proto, 0, 0, &ae)) < 0)
	{
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: upd_lft: Error getting AE for SPI: 0x%x addr: %s protocol: %d \n", spi, nl_addr2str(addr, tmpbuf, 256), proto);
        return -1;
    }

    retval = xfrmnl_ae_get_curlifetime(ae, &bytes, &packets, &addtime, &usetime);
    if (retval) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: upd_lft: Error getting clft\n");
        return -1;
    }

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
        "snoop: xfrm: upd_lft: bytes(%"PRIu64") pkts(%"PRIu64") addtime(%"PRIu64") usetime(%"PRIu64")\n",
        clft->bytes, clft->packets, addtime, usetime);

    /* Create a new XFRM AE object and set it up with the new SA lifetime info */
    ae_new = (struct xfrmnl_ae*)nl_object_clone((struct nl_object*)ae);
    xfrmnl_ae_set_curlifetime(ae_new, clft->bytes, clft->packets, addtime, usetime);

    /* Proagate the new lifetime to the kernel */
    xfrmnl_ae_set(snoop_ctx.xfrm_ctx.nl_sock, ae_new, 0);

    /* Free the old, new XFRM AE objects */
    xfrmnl_ae_put (ae_new);
    xfrmnl_ae_put (ae);

    return 0;
}

int ipsecmgr_snoop_xfrm_convert_dir
(
    int             xfrm_dir,
    ipsecmgr_dir_t  *ipsecmgr_dir
)
{
    if (xfrm_dir == XFRM_POLICY_IN) {
        *ipsecmgr_dir = DIR_INBOUND;
    } else if (xfrm_dir == XFRM_POLICY_OUT) {
        *ipsecmgr_dir = DIR_OUTBOUND;
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: unsupported dir(%d)\n", xfrm_dir);
        return -1;
    }
    return 0;
}

/* Compare 2 SP's
 * return 0 on match
 */
int ipsecmgr_snoop_xfrm_sp_cmp
(
    struct xfrmnl_sp    *sp1,
    struct xfrmnl_sp    *sp2
)
{
    if ((xfrmnl_sp_get_index(sp1) == xfrmnl_sp_get_index(sp2)) &&
        (xfrmnl_sp_get_dir(sp1) == xfrmnl_sp_get_dir(sp2)))
    {
        return 0;
    }

    return -1;
}

/* Compare 2 SA's
 * return 0 on match
 */
int ipsecmgr_snoop_xfrm_sa_cmp
(
    struct xfrmnl_sa    *sa1,
    struct xfrmnl_sa    *sa2
)
{
    if ((xfrmnl_sa_get_spi(sa1) == xfrmnl_sa_get_spi(sa2)) &&
        (xfrmnl_sa_get_proto(sa1) == xfrmnl_sa_get_proto(sa2)) &&
        (nl_addr_cmp(xfrmnl_sa_get_daddr(sa1), xfrmnl_sa_get_daddr(sa2)) == 0))
    {
        return 0;
    }

    return -1;
}

/* Match a given SA with the SP template
 * returns 0 on match
 */
int ipsecmgr_snoop_xfrm_sp_compatible_sa
(
    struct xfrmnl_sp    *sp,
    struct xfrmnl_sa    *sa
)
{
    int num_tmpls, i, mode = xfrmnl_sa_get_mode(sa);
    struct xfrmnl_user_tmpl *tmpl;

    if ((mode != XFRM_MODE_TUNNEL) && (mode != XFRM_MODE_TRANSPORT))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
            "snoop: xfrm: sp_cmplt_sa: unsupported IPSec mode(%d)\n", mode);
        return -1;
    }

    num_tmpls = xfrmnl_sp_get_nusertemplates(sp);

    for (i=0; i<num_tmpls; i++)
    {
        if ((tmpl = xfrmnl_sp_usertemplate_n(sp, i)) == NULL)
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: xfrm: sp_cmptl_sa: Error getting tmpl(%d)from SP(%d)\n",
                i, xfrmnl_sp_get_index(sp));
            return -1;
        }

        /* Match the SA based on the policy template */
        if ((xfrmnl_sa_get_reqid(sa) == xfrmnl_user_tmpl_get_reqid(tmpl)) &&
            (mode ==  xfrmnl_user_tmpl_get_mode(tmpl)) &&
            (xfrmnl_sa_get_proto(sa) == xfrmnl_user_tmpl_get_proto(tmpl)))
        {
            if (mode == XFRM_MODE_TUNNEL)
            {
                if (!nl_addr_cmp(xfrmnl_sa_get_daddr(sa), xfrmnl_user_tmpl_get_daddr(tmpl)) &&
                    !nl_addr_cmp(xfrmnl_sa_get_saddr(sa), xfrmnl_user_tmpl_get_saddr(tmpl)))
                {
                    /* Found a match */
                    return 0;
                }
            }
            else if (mode == XFRM_MODE_TRANSPORT)
            {
                struct xfrmnl_sel *sel;
                struct nl_addr *sel_daddr, *sa_daddr;
                struct sockaddr_in6 sel_dsock, sa_dsock;
                socklen_t salen = sizeof(sel_dsock);

                sel = xfrmnl_sp_get_sel(sp);

                sel_daddr = xfrmnl_sel_get_daddr(sel);
                if (nl_addr_fill_sockaddr(sel_daddr, (struct sockaddr *)&sel_dsock, &salen))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                        "snoop: xfrm: sp_cmptl_sa: couldn't retrieve sel dst addr\n");
                    return -1;
                }

                sa_daddr = xfrmnl_sa_get_daddr(sa);
                salen = sizeof(sa_dsock);
                if (nl_addr_fill_sockaddr(sa_daddr, (struct sockaddr *)&sa_dsock, &salen))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                        "snoop: xfrm: sp_cmptl_sa: couldn't retrieve sa dst addr\n");
                    return -1;
                }

                if (!cmp_subnet((struct sockaddr *)&sel_dsock,
                                (struct sockaddr *)&sa_dsock,
                                xfrmnl_sel_get_prefixlen_d(sel)))
                {
                    /* Found a match */
                    return 0;
                }
            }
        }
    }

    return -1;
}

static void xfrm_receive_sp_updates
(
    struct nl_cache*    cache,
    struct nl_object*   obj,
    int                 action,
    void*               data
)
{
    struct xfrmnl_sp *sp;
    snoop_msg_t msg;

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: xfrm: sp_upd: rcvd SP update... action(%d)\n",action);

    switch (action)
    {
        case NL_ACT_NEW:
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sp_upd: Ignore new event!\n");
            break;
        }

        case NL_ACT_CHANGE:
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sp_upd: Ignore change event!\n");
            break;
        }

        case NL_ACT_DEL:
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm: sp_upd: Got SP Delete event!\n");
            sp = (struct xfrmnl_sp *)nl_object_clone(obj);
            msg.msg_id = MSG_DEL_SP;
            msg.body.sp_info.sp = sp;
            if (ipsecmgr_snoop_sm_send_msg(&msg))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: xfrm: sp_upd: sm_send failed\n");
                nl_object_free((struct nl_object *)sp);
            }
            break;
        }

        default:
        {
            /* Ignore the event */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sp_upd: Ignore unrecognized event(%d)!\n", action);
            break;
        }
    }
    return;
}


/**
 *  @b Description
 *  @n
 *      SA event handler
 *
 *  @param[in]  cache
 *      Corresponding SA cache handle
 *  @param[in]  obj
 *      SA Object that got updated/created/removed as a result
 *      of the event
 *  @param[in]  action
 *      Event that occured
 *  @param[in]  data
 *      Any user specific data that was passed during event
 *      registration is passed back here.
 *
 *  @retval
 *      Not Applicable.
 */
static void xfrm_receive_sa_updates
(
    struct nl_cache*    cache,
    struct nl_object*   obj,
    int                 action,
    void*               data
)
{
    struct xfrmnl_sa *sa, *sa_evt = (struct xfrmnl_sa*)obj;
    snoop_msg_t msg;

    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: xfrm: sa_upd: rcvd SA update... action(%d)\n",action);

    switch (action)
    {
        case NL_ACT_NEW:
        {
            sa = (struct xfrmnl_sa *)nl_object_clone(obj);
            msg.msg_id = MSG_NEW_SA;
            msg.body.sa_info.sa = sa;
            if (ipsecmgr_snoop_sm_send_msg(&msg))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: xfrm: sa_upd: sm_send failed\n");
                nl_object_free((struct nl_object *)sa);
            }
            break;
        }

        case NL_ACT_CHANGE:
        {
            /* Is this a Lifetime Expiry event? */
            if (xfrmnl_sa_is_expiry_reached (sa_evt))
            {
                if (xfrmnl_sa_is_hardexpiry_reached (sa_evt))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                        "snoop: xfrm: sa_upd: Got SA Hard Lifetime Expiry event!\n");
                    sa = (struct xfrmnl_sa *)nl_object_clone(obj);
                    msg.msg_id = MSG_DEL_SA;
                    msg.body.sa_info.sa = sa;
                    if (ipsecmgr_snoop_sm_send_msg(&msg))
                    {
                        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                            "snoop: xfrm: sa_upd: sm_send failed\n");
                        nl_object_free((struct nl_object *)sa);
                    }
                }
                else
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                        "snoop: xfrm: sa_upd: Got SA Soft Lifetime Expiry event...ignoring!\n");
                }
            }
            else
            {
                /* All other SA changes, Ignore. */
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm: sa_upd: ignoring Other SA update\n");
            }
            break;
        }

        case NL_ACT_DEL:
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm: sa_upd: Got SA Delete event!\n");
            sa = (struct xfrmnl_sa *)nl_object_clone(obj);
            msg.msg_id = MSG_DEL_SA;
            msg.body.sa_info.sa = sa;
            if (ipsecmgr_snoop_sm_send_msg(&msg))
            {
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: xfrm: sa_upd: sm_send failed\n");
                nl_object_free((struct nl_object *)sa);
            }
            break;
        }

        default:
        {
            /* Ignore the event */
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sa_upd: Ignore unrecognized event(%d)!\n", action);
            break;
        }
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Polls for any XFRM updates from kernel using Libnl/Libnl-XFRM
 *      APIs
 *
 *  @param[in]  timeout_val_ms
 *      Maximum time (milliseconds) to wait for any event from kernel
 *
 *  @retval
 *      Not Applicable.
 */
void ipsecmgr_snoop_poll_xfrm (uint32_t timeout_val_ms)
{
    int32_t         num_events;

    /* Wait for any event notifications from the cache manager for the timeout
     * value (in milliseconds) provided. */
    if ((num_events = nl_cache_mngr_poll(snoop_ctx.xfrm_ctx.cache_mngr,
                                        timeout_val_ms)) < 0)
    {
        /* Timeout reached. No updates to report */
    }
    else
    {
        /* Timeout reached. Found event notifications. */
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Cleans up Libnl XFRM cache and all relevant info
 *
 *  @retval
 *      Not Applicable.
 */
void ipsecmgr_snoop_xfrm_shutdown(void)
{
    /* Disconnect and free the XFRM netlink socket */
    if (snoop_ctx.xfrm_ctx.nl_sock) {
        nl_close(snoop_ctx.xfrm_ctx.nl_sock);
        nl_socket_free(snoop_ctx.xfrm_ctx.nl_sock);
    }

    if (snoop_ctx.xfrm_ctx.cache_mngr)
        nl_cache_mngr_free(snoop_ctx.xfrm_ctx.cache_mngr);

    return;
}


int ipsecmgr_snoop_xfrm_init()
{
    int32_t     retval;

    /* Initialize XFRM netlink context */
    memset ((void *)&snoop_ctx.xfrm_ctx, 0, sizeof (ipsecmgr_xfrm_ctx_t));

    /* Setup a netlink socket that we can use for sending AE events to kernel */
	if ((snoop_ctx.xfrm_ctx.nl_sock = nl_socket_alloc()) == NULL)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "Unable to allocate netlink socket, error: %d", errno);
        return -1;
    }

    /* Connect this socket to kernel's XFRM subsystem */
	if ((retval = nl_connect(snoop_ctx.xfrm_ctx.nl_sock, NETLINK_XFRM)) < 0)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "Unable to connect netlink socket: %s", nl_geterror (retval));
        ipsecmgr_snoop_xfrm_shutdown();
        return -1;
    }

    /* Next, create a Cache manager for NETLINK_XFRM so that we receive XFRM events from kernel */
    if ((retval = nl_cache_mngr_alloc (NULL, NETLINK_XFRM, NL_AUTO_PROVIDE, &snoop_ctx.xfrm_ctx.cache_mngr)) < 0)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "Error allocating NETLINK_XFRM Cache manager \n");
        ipsecmgr_snoop_xfrm_shutdown();
        return -1;
    }

    /* Add caches for all the XFRM components we want to monitor */
    if ((retval = nl_cache_mngr_add(snoop_ctx.xfrm_ctx.cache_mngr, "xfrm/sa",
                                     &xfrm_receive_sa_updates, NULL, &snoop_ctx.xfrm_ctx.sa_cache)) < 0)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "Error setting up XFRM SA cache, error: %d \n", retval);
        ipsecmgr_snoop_xfrm_shutdown();
        return -1;
    }
    if ((retval = nl_cache_mngr_add(snoop_ctx.xfrm_ctx.cache_mngr, "xfrm/sp",
                                     &xfrm_receive_sp_updates, NULL, &snoop_ctx.xfrm_ctx.sp_cache)) < 0)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "Error setting up XFRM SP cache, error: %d \n", retval);
        ipsecmgr_snoop_xfrm_shutdown();
        return -1;
    }

    /* XFRM netlink init done. Return success. */
    return 0;
}

int ipsecmgr_snoop_xfrm_sp_get
(
    uint32_t            policy_id,
    struct xfrmnl_sp    **sp,
    ipsecmgr_dir_t      *dir,
    enum policy_type    *type
)
{
    uint32_t xfrm_dir;
    int action, ntmpls;

    /* Get the direction based on the policy ID specified */
    xfrm_dir = xfrmnl_sp_index2dir(policy_id);

    if (ipsecmgr_snoop_xfrm_convert_dir(xfrm_dir, dir)) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: sp_get: Error invalid direction\n");
        return -1;
    }

    /* Get the SP from the cache */
    *sp = xfrmnl_sp_get(snoop_ctx.xfrm_ctx.sp_cache, policy_id, xfrm_dir);
    if (*sp == NULL)
	{
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "Error getting SP(%d) dir(%d)\n", policy_id, xfrm_dir);
        return -1;;
    }

    action = xfrmnl_sp_get_action(*sp);

    if (action == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: sp_get: Error SP has no action\n");
        return -1;
    }

    ntmpls = xfrmnl_sp_get_nusertemplates(*sp);

    if (action == XFRM_POLICY_ALLOW) {
        if (ntmpls) {
            *type = POLICY_IPSEC;
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sp_get: IPSec policy\n");
        } else {
            *type = POLICY_NONE;
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                "snoop: xfrm: sp_get: non IPSec policy\n");
        }
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: xfrm: sp_get: no support for offloading blocking policy\n");
        return -1;
    }

    return 0;
}

void ipsecmgr_snoop_xfrm_sa_inherit
(
    struct xfrmnl_sa    *sa
)
{
    nl_object_get((struct nl_object *) sa);
}

int ipsecmgr_snoop_xfrm_sa_get
(
    struct xfrmnl_sp    *sp,
    struct xfrmnl_sa    **sa,
    int                 *reqid
)
{
    struct xfrmnl_sa        *sa_filter;
    struct nl_cache         *sa_cache_filtered;
    struct nl_cache         *sa_nl_cache_first;
    struct xfrmnl_user_tmpl *tmpl;
    uint32_t                num_tmpls, i;
    int                     retval = -1;

    num_tmpls = xfrmnl_sp_get_nusertemplates(sp);

    if ((sa_filter = xfrmnl_sa_alloc()) == NULL)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: sa_get: Error allocating memory for SA object\n");
        return retval;
    }

    /* Get the template from the policy object. We'll use it for
     * looking up corresponding SA for it. */
    for (i=0; i<num_tmpls; i++)
    {
        if ((tmpl = xfrmnl_sp_usertemplate_n(sp, i)) == NULL)
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: xfrm: sa-get: Error getting tmpl(%d)from SP(%d)\n",
                i, xfrmnl_sp_get_index(sp));
            return retval;
        }

        /* Get the corresponding SA based on the policy template */
        xfrmnl_sa_set_reqid(sa_filter, xfrmnl_user_tmpl_get_reqid (tmpl));
        xfrmnl_sa_set_mode(sa_filter, xfrmnl_user_tmpl_get_mode (tmpl));
        xfrmnl_sa_set_proto(sa_filter, xfrmnl_user_tmpl_get_proto (tmpl));
        if (xfrmnl_user_tmpl_get_mode(tmpl) == XFRM_MODE_TUNNEL)
        {
            xfrmnl_sa_set_daddr(sa_filter, xfrmnl_user_tmpl_get_daddr(tmpl));
            xfrmnl_sa_set_saddr(sa_filter, xfrmnl_user_tmpl_get_saddr(tmpl));
        } else {
            /* TODO: transport mode */
        }

        /* Find all SA objects in cache that match the SA filter we just
         * constructed */
        if ((sa_cache_filtered = nl_cache_subset (snoop_ctx.xfrm_ctx.sa_cache,
                                    (struct nl_object *)sa_filter)) != NULL)
        {
            if (nl_cache_nitems(sa_cache_filtered) == 0)
            {
                /* Free the empty cache */
                nl_cache_free (sa_cache_filtered);
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm: sa_get: No matching SA for SP(%d) tmpl(%d)\n",
                    xfrmnl_sp_get_index(sp), i);
                continue;
            }
            /* Get the first matching SA */
            sa_nl_cache_first = (struct nl_cache *)nl_cache_get_first(sa_cache_filtered);
            if (sa_nl_cache_first)
            {
                *sa = (struct xfrmnl_sa*) nl_object_clone((struct nl_object *)sa_nl_cache_first);

                /* Free the cache filter object */
                nl_cache_free(sa_cache_filtered);
                if ((*reqid = xfrmnl_sa_get_reqid(*sa)) == -1) {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
                        "snoop: xfrm: sa_get: SA has no reqid for SP(%d) tmpl(%d)\n",
                        xfrmnl_sp_get_index(sp), i);
                    continue;
                }
                snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                    "snoop: xfrm: sa_get: Found SA for SP(%d), SPI(0x%x)\n",
                    xfrmnl_sp_get_index(sp), xfrmnl_sa_get_spi(*sa));
                retval = 0;
            }
            break;
        }
    }

    /* Free the SA filter object */
    xfrmnl_sa_put(sa_filter);
    return retval;
}

void ipsecmgr_snoop_xfrm_sp_put(struct xfrmnl_sp *sp)
{
    xfrmnl_sp_put(sp);
}

void ipsecmgr_snoop_xfrm_sa_put(struct xfrmnl_sa *sa)
{
    xfrmnl_sa_put(sa);
}

uint32_t  ipsecmgr_snoop_xfrm_sa_getRefCount(struct xfrmnl_sa *sa)
{
    return (nl_object_get_refcnt((struct nl_object *) sa));
}
