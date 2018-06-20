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

#ifndef __TI_IPSECMGR_MOD_H__
#define __TI_IPSECMGR_MOD_H__

#include <linux/xfrm.h>

#define IPSECMGR_MOD_DEVNAME  "ipsecmgr"

#define IPSECMGR_MOD_IOCMAGIC 0x0000fe00

/* Supported "base" ioctl cmds for the driver. */
#define IPSECMGR_MOD_IOC_OFFLOAD_SA         1
#define IPSECMGR_MOD_IOC_CHECK_SA_EXPIRE    2
#define IPSECMGR_MOD_IOC_STOP_OFFLOAD       3

#define IPSECMGR_MOD_IOCCMDMASK   0x000000ff

#define IPSECMGR_MOD_DIR_OUT 1
#define IPSECMGR_MOD_DIR_IN  2

typedef struct {
    uint32_t    swinfo_0;   /* SWINFO word 0 */
    uint32_t    swinfo_1;   /* SWINFO word 0 */
    uint16_t    flow_id;    /* SA Rx DMA flow-id */
} ipsecmgr_mod_sa_ctx_info_t;

struct ipsecmgr_mod_user_sa_params {
    uint8_t                     dir;    /* IPSECMGR_MOD_DIR_ */
    uint16_t                    af;     /* address family as defined in socket.h */
    uint16_t                    proto;  /* IPPROTO as defined in in.h */
    uint32_t                    spi;    /* in host endian */
    xfrm_address_t              daddr;  /* dst address in Big endian */
    ipsecmgr_mod_sa_ctx_info_t  sa_ctx; /* Security Accelerator context parameters */
};

#endif /*__TI_IPSECMGR_MOD_H__ */

