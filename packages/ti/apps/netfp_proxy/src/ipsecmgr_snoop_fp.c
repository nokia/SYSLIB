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
 * File implementing functions to create SP & SA in NETCP
*/

/* standard includes */
#include <stdlib.h>
#include <linux/xfrm.h>
#include <linux/udp.h>

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

/* Transaction id for FP transactions */
static ipsecmgr_trans_id_t   fp_trans_id;

int ipsecmgr_snoop_fp_create_sa
(
    struct sa_info_s            *sa_params,
    ipsecmgr_sa_flags_t         sa_flags,
    ipsecmgr_dir_t              dir,
    ipsecmgr_sa_dscp_map_cfg_t  *dscp_map_cfg,
    ipsecmgr_ifname_t           *if_name,
    ipsecmgr_trans_id_t         *trans_id,
    ipsecmgr_fp_handle_t        *sa_handle,
    uint32_t                    *fp_result
)
{
    struct xfrmnl_sa *sa = sa_params->sa;
    ipsecmgr_af_t af;
    ipsecmgr_sa_id_t sa_id;
    ipsecmgr_sa_info_t sa_info;
    struct nl_addr *saddr, *daddr;
    int ret, proto, mode, flags;
    char alg_name[64];
    unsigned int key_len, icv_len, a_family;
    char key[64];
    struct sockaddr_in6 s_sock, d_sock;
    socklen_t salen = sizeof(s_sock);
    ipsecmgr_sa_encap_tmpl_t encap, *pencap = NULL;
    unsigned int e_type, e_sport, e_dport;
    struct nl_addr *e_oa = NULL;

    memset(&sa_info, 0, sizeof(sa_info));
    memset(&sa_id, 0, sizeof(sa_id));

    saddr = xfrmnl_sa_get_saddr(sa);

    if (saddr == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no saddr\n");
        return -1;
    }

    daddr = xfrmnl_sa_get_daddr(sa);

    if (daddr == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no daddr\n");
        return -1;
    }

    a_family = nl_addr_guess_family(saddr);

    if (nl_addr_fill_sockaddr(saddr, (struct sockaddr *)&s_sock, &salen))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: couldn't retrieve src addr\n");
        return -1;
    }

    if (nl_addr_fill_sockaddr(daddr, (struct sockaddr *)&d_sock, &salen))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: couldn't retrieve dst addr\n");
        return -1;
    }

    if (a_family == AF_INET) {
        struct sockaddr_in *paddr;
        af = IPSECMGR_AF_IPV4;
        /* destination address */
        paddr = (struct sockaddr_in *)&d_sock;
        memcpy(sa_id.daddr.ipv4, &paddr->sin_addr, 4);
        /* Source address */
        paddr = (struct sockaddr_in *)&s_sock;
        memcpy(sa_info.saddr.ipv4, &paddr->sin_addr, 4);
    } else if (a_family == AF_INET6) {
        af = IPSECMGR_AF_IPV6;
        /* destination address */
        memcpy(sa_id.daddr.ipv6, &d_sock.sin6_addr, 16);
        /* Source address */
        memcpy(sa_info.saddr.ipv6, &s_sock.sin6_addr, 16);
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: unsupported af(%d)\n", a_family);
        return -1;
    }

    sa_id.spi = xfrmnl_sa_get_spi(sa);
    if (sa_id.spi == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no spi\n");
        return -1;
    }

    proto = xfrmnl_sa_get_proto(sa);
    switch(proto) {
        case IPPROTO_ESP:
            sa_id.proto = SA_PROTO_ESP;
            break;
        case IPPROTO_AH:
            sa_id.proto = SA_PROTO_AH;
            break;
        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: unsupported IPSec proto(%d)\n", proto);
            return -1;
    }

    mode = xfrmnl_sa_get_mode(sa);
    switch(mode) {
        case XFRM_MODE_TRANSPORT:
            sa_info.mode = SA_MODE_TRANSPORT;
            break;
        case XFRM_MODE_TUNNEL:
            sa_info.mode = SA_MODE_TUNNEL;
            break;
        default:
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: unsupported mode (%d)\n", mode);
            return -1;
    }

    sa_info.reqid = xfrmnl_sa_get_reqid(sa);
    if (sa_info.reqid == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no reqid\n");
        return -1;
    }

    sa_info.replay_window = xfrmnl_sa_get_replay_window(sa);
    if (sa_info.replay_window == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no replay_window\n");
        return -1;
    }

    sa_info.flags = sa_flags;
    sa_info.dir = dir;

    flags = xfrmnl_sa_get_flags(sa);
    if (flags == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no flags\n");
        return -1;
    }

    if (flags & XFRM_STATE_ESN) {
        sa_info.flags |= IPSECMGR_SA_FLAGS_ESN;
    }

    if (xfrmnl_sa_get_crypto_params(sa, alg_name, &key_len, key)) {
        sa_info.enc.algo = SA_EALG_NONE;
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
            "snoop: fp: create_sa: no enc algo\n");
    } else {
        if (!strcmp(alg_name, "ecb(cipher_null)")) {
            sa_info.enc.algo = SA_EALG_NULL;
        } else if (!strcmp(alg_name, "cbc(des3_ede)")) {
            sa_info.enc.algo = SA_EALG_3DES_CBC;
        } else if (!strcmp(alg_name, "cbc(aes)")) {
            sa_info.enc.algo = SA_EALG_AES_CBC;
        } else if (!strcmp(alg_name, "rfc3686(ctr(aes))")) {
            sa_info.enc.algo = SA_EALG_AES_CTR;
        } else if (!strcmp(alg_name, "cbc(des)")) {
            sa_info.enc.algo = SA_EALG_DES_CBC;
        } else {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: Unsupported encrypt algo (%s)\n",
            alg_name);
            return -1;
        }

        sa_info.enc_key_len = key_len/8;
        if (sa_info.enc_key_len)
            memcpy(sa_info.enc_key, key, sa_info.enc_key_len);
    }

    if (xfrmnl_sa_get_auth_params(sa, alg_name, &key_len, &icv_len, key)) {
        sa_info.auth.algo = SA_EALG_NONE;
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
            "snoop: fp: create_sa: no auth algo\n");
    } else {
        if (!strcmp(alg_name, "digest_null")) {
            sa_info.auth.algo = SA_AALG_NULL;
        } else if (!strcmp(alg_name, "hmac(md5)")) {
            sa_info.auth.algo = SA_AALG_HMAC_MD5;
        } else if (!strcmp(alg_name, "hmac(sha1)")) {
            sa_info.auth.algo = SA_AALG_HMAC_SHA1;
        } else if (!strcmp(alg_name, "hmac(sha256)")) {
            sa_info.auth.algo = SA_AALG_HMAC_SHA2_256;
        } else if (!strcmp(alg_name, "xcbc(aes)")) {
            sa_info.auth.algo = SA_AALG_AES_XCBC;
        } else {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: Unsupported auth algo (%s)\n",
            alg_name);
            return -1;
        }

        sa_info.auth_key_len = key_len/8;
        sa_info.auth.icvlen = icv_len/8;
        if (sa_info.auth_key_len)
            memcpy(sa_info.auth_key, key, sa_info.auth_key_len);
    }

    if (!((sa_info.enc.algo == SA_EALG_NONE) &&
          (sa_info.auth.algo == SA_AALG_NONE))) {
        goto skip_aead;
    }

    if (xfrmnl_sa_get_aead_params(sa, alg_name, &key_len, &icv_len, key)) {
        sa_info.enc.algo = SA_EALG_NONE;
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
            "snoop: fp: create_sa: no aead algo\n");
    } else {
        if (!strcmp(alg_name, "rfc4106(gcm(aes))")) {
            sa_info.enc.algo = SA_EALG_AES_GCM;
        } else if (!strcmp(alg_name, "rfc4309(ccm(aes))")) {
            sa_info.enc.algo = SA_EALG_AES_CCM;
        } else if (!strcmp(alg_name, "rfc4543(gcm(aes))")) {
            sa_info.enc.algo = SA_EALG_NULL;
            sa_info.auth.algo = SA_AALG_AES_GMAC;
        } else {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: Unsupported enc algo (%s)\n",
            alg_name);
            return -1;
        }

        sa_info.enc_key_len = key_len/8;
        sa_info.auth.icvlen = icv_len/8;
        if (sa_info.enc_key_len)
            memcpy(sa_info.enc_key, key, sa_info.enc_key_len);
    }

skip_aead:

    if (!strlen((char *)if_name->name)) {
        if_name = NULL;
    }

    /* Lifetime configuration */
    if (ipsecmgr_snoop_xfrm_get_lft_cfg(sa, &sa_info.lft))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa: no lifetime config\n");
        return -1;
    }

    /* Get NAT-T encapsulation info */
    if (!xfrmnl_sa_get_encap_tmpl(sa, &e_type, &e_sport, &e_dport, &e_oa)) {
        if (e_type == UDP_ENCAP_ESPINUDP) {
            encap.sport = (uint16_t)e_sport;
            encap.dport = (uint16_t)e_dport;
            pencap = &encap;
        }
        if (e_oa != NULL) nl_addr_put(e_oa);
    }

    if (ipsecmgr_snoop_xfrm_get_oseq(sa, &sa_info.esnlo, &sa_info.esnhi)) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                    "snoop: fp: create_sa: get_oseq error\n");
        return -1;
    }
    *trans_id = fp_trans_id++;
    ret = snoop_ctx.fp_cfg_cb.add_sa (af, &sa_id, &sa_info,
                                      dscp_map_cfg, if_name, pencap, sa_handle);

    *fp_result = ret;

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: create_sa failed, error code: %d\n", ret);
        return -1;
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: fp: create_sa trans_id: 0x%x\n", *trans_id);
    }
    return 0;
}

int ipsecmgr_snoop_fp_add_sp
(
    struct xfrmnl_sp       *sp,
    ipsecmgr_l5_selector_t  *sp_l5_sel,
    uint32_t                reqid,
    ipsecmgr_fp_handle_t    sa_handle,
    ipsecmgr_trans_id_t     *trans_id,
    ipsecmgr_fp_handle_t    *sp_handle,
    uint32_t                *fp_result
)
{
    struct xfrmnl_sel *sp_sel;
    struct nl_addr *saddr, *daddr;
    ipsecmgr_af_t af;
    ipsecmgr_selector_t sel;
    ipsecmgr_dir_t dir;
    int ret, sp_id, sp_dir, a_family;
    struct sockaddr_in6 s_sock, d_sock;
    socklen_t salen = sizeof(s_sock);

    memset(&sel, 0, sizeof(sel));

    sp_id = xfrmnl_sp_get_index(sp);

    if (sp_id == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: Error no policy_id\n");
        return -1;
    }

    sp_sel = xfrmnl_sp_get_sel(sp);

    if (sp_sel == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: Error no selector\n");
        return -1;
    }

    saddr = xfrmnl_sel_get_saddr(sp_sel);

    if (saddr == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: Error no saddr\n");
        return -1;
    }

    daddr = xfrmnl_sel_get_daddr(sp_sel);

    if (daddr == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: Error no daddr\n");
        return -1;
    }

    a_family = nl_addr_guess_family(saddr);

    if (nl_addr_fill_sockaddr(saddr, (struct sockaddr *)&s_sock, &salen))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: couldn't retrieve sel src addr\n");
        return -1;
    }

    if (nl_addr_fill_sockaddr(daddr, (struct sockaddr *)&d_sock, &salen))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: couldn't retrieve sel dst addr\n");
        return -1;
    }

    if (a_family == AF_INET) {
        struct sockaddr_in *paddr;
        af = IPSECMGR_AF_IPV4;
        /* destination address */
        paddr = (struct sockaddr_in *)&d_sock;
        memcpy(sel.daddr.ipv4, &paddr->sin_addr, 4);
        /* Source address */
        paddr = (struct sockaddr_in *)&s_sock;
        memcpy(sel.saddr.ipv4, &paddr->sin_addr, 4);
    } else if (a_family == AF_INET6) {
        af = IPSECMGR_AF_IPV6;
        /* destination address */
        memcpy(sel.daddr.ipv6, &d_sock.sin6_addr, 16);
        /* Source address */
        memcpy(sel.saddr.ipv6, &s_sock.sin6_addr, 16);
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: unsupported af(%d)\n", a_family);
        return -1;
    }

    sel.dport_start = sel.dport_end = xfrmnl_sel_get_dport(sp_sel);
    sel.sport_start = sel.sport_end = xfrmnl_sel_get_sport(sp_sel);
    sel.prefixlen_s = xfrmnl_sel_get_prefixlen_s(sp_sel);
    sel.prefixlen_d = xfrmnl_sel_get_prefixlen_d(sp_sel);
    sel.proto = xfrmnl_sel_get_proto(sp_sel);

    if ((sp_dir = xfrmnl_sp_get_dir(sp)) == -1) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: add_sp: Error no dir(%d)\n");
        return -1;
    }

    if (ipsecmgr_snoop_xfrm_convert_dir(sp_dir, &dir)) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: Error add_sp: unsupported dir(%d)\n", sp_dir);
        return -1;
    }

    sel.l5_selector = sp_l5_sel;

    *trans_id = fp_trans_id++;
    ret = snoop_ctx.fp_cfg_cb.add_sp(af, &sel, dir, reqid,
                            sa_handle, sp_id, sp_handle);
    *fp_result = ret;

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: Error add_sp failed, error code: %d\n", ret);
        return -1;
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: fp: add_sp trans_id: 0x%x\n", *trans_id);
    }
    return 0;
}

int ipsecmgr_snoop_fp_del_sa(uint32_t reqid, ipsecmgr_dir_t dir, ipsecmgr_fp_handle_t sa_handle)
{
    int ret;

    ret = snoop_ctx.fp_cfg_cb.del_sa (reqid, dir, sa_handle);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: Error del_sa failed, error code: %d\n", ret);
        return -1;
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
            "snoop: fp: del_sa trans_id: 0x%x\n", fp_trans_id);
    }
    return 0;
}

int ipsecmgr_snoop_fp_del_sp
(
    ipsecmgr_fp_handle_t    sp_handle,
    uint32_t                policy_id,
    uint32_t                reqid,
    ipsecmgr_dir_t          dir
)
{
    int ret;

    ret = snoop_ctx.fp_cfg_cb.del_sp (sp_handle, policy_id, reqid, dir);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: del_sp failed, error code: %d\n", ret);
        return -1;
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: del_sp trans_id: 0x%x\n", fp_trans_id);
    }
    return 0;
}

int ipsecmgr_snoop_fp_get_sa_hw_ctx
(
    ipsecmgr_fp_handle_t    sa_handle,
    struct sa_info_s        *sa_info,
    uint32_t                *fp_result
)
{
    int ret;

    *fp_result = 0;
    ret = snoop_ctx.fp_cfg_cb.get_sa_ctx (sa_handle, &sa_info->hw_ctx);

    if (ret) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: fp: get_sa_ctx failed, error code: %d\n", ret);
        *fp_result = ret;
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_fp_init()
{
    fp_trans_id = 0;
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: fp: initialized\n");
    return 0;
}

void ipsecmgr_snoop_fp_shutdown()
{
    fp_trans_id = 0;
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: fp: shutdown\n");
    return;
}

