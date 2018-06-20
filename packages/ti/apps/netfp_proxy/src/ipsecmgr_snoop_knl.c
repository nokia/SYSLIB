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
 * File implementing functions to interface to IPSecMgr Kernel module
*/

/* standard includes */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>
#include <ti/apps/netfp_proxy/include/ipsecmgr_mod.h>

#define MOD_NAME    "/dev/"IPSECMGR_MOD_DEVNAME
#define LOG_MSG snoop_ctx.plat_cb.log_msg

static int knl_fd;

static int snoop_knl_get_daddr
(
    struct xfrmnl_sa    *sa,
    xfrm_address_t      *daddr,
    uint16_t            *af
)
{
    struct nl_addr *addr;
    struct sockaddr_in6 dsock;
    socklen_t salen = sizeof(dsock);

    addr = xfrmnl_sa_get_daddr(sa);

    if (addr == NULL) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: get_daddr: no daddr\n");
        return -1;
    }

    *af = nl_addr_guess_family(addr);

    if (nl_addr_fill_sockaddr(addr, (struct sockaddr *)&dsock, &salen))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: get_daddr: couldn't retrieve dst addr\n");
        return -1;
    }

    if (*af == AF_INET) {
        struct sockaddr_in *paddr;
        /* destination address */
        paddr = (struct sockaddr_in *)&dsock;
        daddr->a4 = paddr->sin_addr.s_addr;
    } else if (*af == AF_INET6) {
        /* destination address */
        memcpy(daddr->a6, &dsock.sin6_addr, 16);
    } else {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: get_daddr: unsupported af(%d)\n", *af);
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_knl_set_ctx
(
    struct sa_info_s    *sa_info,
    ipsecmgr_dir_t      dir
)
{
    struct ipsecmgr_mod_user_sa_params sa_params;
    int ret;

    memset(&sa_params, 0, sizeof(sa_params));

    sa_params.dir = (dir == DIR_INBOUND) ?
                        IPSECMGR_MOD_DIR_IN : IPSECMGR_MOD_DIR_OUT;
    sa_params.proto = xfrmnl_sa_get_proto(sa_info->sa);
    sa_params.spi = xfrmnl_sa_get_spi(sa_info->sa);

    if (snoop_knl_get_daddr(sa_info->sa, &sa_params.daddr, &sa_params.af))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: set_ctx: Error couldn't get daddr\n");
        return -1;
    }

    if (dir == DIR_OUTBOUND) {
        sa_params.sa_ctx.swinfo_0 = sa_info->hw_ctx.swinfo[0];
        sa_params.sa_ctx.swinfo_1 = sa_info->hw_ctx.swinfo[1];
        sa_params.sa_ctx.flow_id  = sa_info->hw_ctx.flow_id;
    }

    ret = ioctl(knl_fd, IPSECMGR_MOD_IOC_OFFLOAD_SA | IPSECMGR_MOD_IOCMAGIC,
                &sa_params);

    if (ret) {
        LOG_MSG(LOG_LEVEL_ERROR,
        "snoop: knl: set_ctx ioctl returned error(%d)\n", ret);
        return -1;
    }
    return 0;
}

int ipsecmgr_snoop_knl_del_ctx
(
    struct sa_info_s    *sa_info,
    ipsecmgr_dir_t      dir
)
{
    struct ipsecmgr_mod_user_sa_params sa_params;
    int ret;

    memset(&sa_params, 0, sizeof(sa_params));

    sa_params.dir = (dir == DIR_INBOUND) ?
                        IPSECMGR_MOD_DIR_IN : IPSECMGR_MOD_DIR_OUT;

    sa_params.proto = xfrmnl_sa_get_proto(sa_info->sa);
    sa_params.spi = xfrmnl_sa_get_spi(sa_info->sa);

    if (snoop_knl_get_daddr(sa_info->sa, &sa_params.daddr, &sa_params.af))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: del_ctx: Error couldn't get daddr\n");
        return -1;
    }

    ret = ioctl(knl_fd, IPSECMGR_MOD_IOC_STOP_OFFLOAD | IPSECMGR_MOD_IOCMAGIC,
                &sa_params);

    if (ret) {
        LOG_MSG(LOG_LEVEL_ERROR,
            "snoop: knl: del_ctx ioctl returned error(%d)\n", ret);
        return -1;
    }
    return 0;
}

int ipsecmgr_snoop_knl_check_sa_expiry
(
    struct sa_info_s    *sa_info
)
{
    int err;
    struct ipsecmgr_mod_user_sa_params sa_params;

    memset(&sa_params, 0, sizeof(sa_params));

    sa_params.proto = xfrmnl_sa_get_proto(sa_info->sa);
    sa_params.spi = xfrmnl_sa_get_spi(sa_info->sa);

    if (snoop_knl_get_daddr(sa_info->sa, &sa_params.daddr, &sa_params.af))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: check_sa_expiry: Error couldn't get daddr\n");
        return -1;
    }

    err = ioctl(knl_fd,IPSECMGR_MOD_IOC_CHECK_SA_EXPIRE | IPSECMGR_MOD_IOCMAGIC,
                &sa_params);

    if (err) {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: knl: check_sa_expiry ioctl returned error(%d)\n", err);
        return -1;
    }

    return 0;
}

int ipsecmgr_snoop_knl_mod_open()
{
    knl_fd = open(MOD_NAME, O_RDWR);

    if (knl_fd == -1) {
        LOG_MSG(LOG_LEVEL_ERROR,
        "snoop: knl: open, failed to open %s: '%s'\n",
        MOD_NAME, strerror(errno));
        return -1;
    }

    LOG_MSG(LOG_LEVEL_INFO, "snoop: knl: module opened\n");
    return 0;
}

void ipsecmgr_snoop_knl_mod_close()
{
    if (knl_fd) close(knl_fd);
    knl_fd = 0;
    LOG_MSG(LOG_LEVEL_INFO, "snoop: knl: module closed\n");
}

