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

/* standard includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* module specific includes */
#include <ti/apps/netfp_proxy/include/ipsecmgr_snoop_loc.h>

/* Provides a very simplistic timer list implementation for use by the state machine
   The timers should be multiple of 100 ms.
   Limitations:
   Timeouts are not accurate.
   Only 1 timer addition within timeout callback is handled.
*/

struct timer_s {
    uint32_t    num_ticks;
    void *      cb_handle;
    uint8_t     in_use;
};

#define MIN_TIMER_RES   EVENT_POLL_INTVL /* in milli seconds */
#define MAX_TIMERS  32
static struct timer_s tlist[MAX_TIMERS];

int ipsecmgr_snoop_start_timer(uint32_t ms, void *cb_handle, int32_t *timer_handle)
{
    int i;
    for (i=0; i<MAX_TIMERS; i++) {
        if (!tlist[i].in_use) {
            tlist[i].cb_handle = cb_handle;
            tlist[i].num_ticks = ms/MIN_TIMER_RES;
            *timer_handle = i;
            tlist[i].in_use = 1;
            return 0;
        }
    }
    return -1;
}

int ipsecmgr_snoop_stop_timer(int32_t timer_handle)
{
    if (timer_handle >= MAX_TIMERS) return -1;
    tlist[timer_handle].in_use = 0;
    return 0;
}

int ipsecmgr_snoop_init_timer_list()
{
    memset(tlist, 0, sizeof(tlist));
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: timer: initialized\n");
    return 0;
}

void ipsecmgr_snoop_shutdown_timer_list()
{
    memset(tlist, 0, sizeof(tlist));
    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO, "snoop: timer: shutdown\n");
    return;
}

void ipsecmgr_snoop_increment_tick()
{
    int i;
    snoop_msg_t msg;

    for (i=0; i<MAX_TIMERS; i++) {
        if (tlist[i].in_use) {
            if (!(--tlist[i].num_ticks)) {
                tlist[i].in_use = 0;
                /* Trigger the state machine */
                msg.msg_id = MSG_TIMEOUT;
                msg.body.timeout.cookie = (uint32_t)tlist[i].cb_handle;
                if (ipsecmgr_snoop_sm_send_msg(&msg))
                {
                    snoop_ctx.plat_cb.log_msg(LOG_LEVEL_INFO,
                        "snoop: timer: sm_send failed\n");
                }
            }
        }
    }
}

