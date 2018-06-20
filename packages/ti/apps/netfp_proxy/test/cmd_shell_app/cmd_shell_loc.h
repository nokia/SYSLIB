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

#ifndef __CMD_SHELL_LOC_H__
#define __CMD_SHELL_LOC_H__

/* Standard socket includes */
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <unistd.h>

/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/* SYSLIB includes */
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/* Module context for command shell */
typedef struct {
    uint32_t  trans_id;
} cmd_shell_ctx_t;

/* Command line interpreter task */
void* cmd_shell (void *args);

/* OFFLOAD_SP response handler */
void cmd_shell_offload_sp_rsp (NetfpProxy_msg* rxMsgPtr);

/* OFFLOAD_SP response - Publish to Name database */
int32_t cmd_shell_publish_sp_rsp (NetfpProxy_MsgType msgType, NetfpProxy_PolicyId policyId, const char* policyName);
#endif

