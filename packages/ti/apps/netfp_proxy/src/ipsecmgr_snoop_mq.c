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

#include <sys/stat.h>
#include <sys/types.h>
#include <mqueue.h>
#include <errno.h>
#include <unistd.h>

#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

int ipsecmgr_snoop_mq_init()
{
    struct mq_attr ma;
    pid_t pid;
    ma.mq_flags = O_NONBLOCK;
    ma.mq_maxmsg = MSQ_DEPTH;
    ma.mq_msgsize = sizeof(snoop_msg_t);
    ma.mq_curmsgs = 0;

    pid = getpid();
    snprintf(&snoop_ctx.name[0], MSQ_NAME_MAX_LEN-1, "%s%d", MSQ_NAME,pid);
    snoop_ctx.mq = mq_open(&snoop_ctx.name[0], O_RDWR | O_CREAT | O_NONBLOCK, 0700, &ma);

    if (snoop_ctx.mq == (mqd_t)-1)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: init: failed to create queue. errno(%d)\n", errno);
        return -1;
    }
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_WARN,
        "snoop: mq: init: mqs_name: %s, pid: %d\n",
            &snoop_ctx.name[0],pid);
    return 0;
}

int ipsecmgr_snoop_mq_send(snoop_msg_t *msg, uint32_t len)
{
    if (snoop_ctx.mq == (mqd_t)-1)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: send: mq not initialized\n");
        return -1;
    }

    if (mq_send(snoop_ctx.mq, (char *)msg, len, 1))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: send: failed. errno(%d)\n", errno);
        return -1;
    }
    return 0;
}

int ipsecmgr_snoop_mq_recv(snoop_msg_t *msg, uint32_t *len)
{
    int ret;

    if (snoop_ctx.mq == (mqd_t)-1)
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: recv: mq not initialized\n");
        return -1;
    }

    ret = mq_receive(snoop_ctx.mq, (char *)msg, *len, NULL);
    if (ret == -1)
    {
        if(errno == EAGAIN)
        {
            *len = 0;
            return 0;
        }
        else
        {
            snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
                "snoop: mq: recv: failed. errno(%d)\n", errno);
            return -1;
        }
    }

    *len = ret;
    return 0;
}

void ipsecmgr_snoop_mq_shutdown(void)
{
    if (mq_close(snoop_ctx.mq))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: shutdown: close failed errno(%d)\n", errno);
    }
    if (mq_unlink(&snoop_ctx.name[0]))
    {
        snoop_ctx.plat_cb.log_msg(LOG_LEVEL_ERROR,
            "snoop: mq: shutdown: unlink failed errno(%d)\n", errno);
    }
    return;
}

