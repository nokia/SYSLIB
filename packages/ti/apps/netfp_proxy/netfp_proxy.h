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
#ifndef __NETFP_PROXY_H__
#define __NETFP_PROXY_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   NetFP Proxy return codes.
 */
#define     NETFP_PROXY_RETVAL_SUCCESS              0
#define     NETFP_PROXY_RETVAL_E_OP_FAILED          -1
#define     NETFP_PROXY_RETVAL_E_INVALID_PARAMS     -2
#define     NETFP_PROXY_RETVAL_E_NO_MEM             -3
#define     NETFP_PROXY_RETVAL_E_DUPLICATE          -4

/**
 * @brief   NetFP Proxy log levels
 */
typedef enum NetfpProxy_LogLevel
{
    NETFP_PROXY_LOG_ERROR,
    NETFP_PROXY_LOG_INFO,
    NETFP_PROXY_LOG_DEBUG,
    NETFP_PROXY_LOG_VRB //fzm
} NetfpProxy_LogLevel;

extern void NetfpProxy_logMsg (NetfpProxy_LogLevel level, const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
// <fzm>
extern uint8_t criticalErrorOccurred;
extern void NetfpProxy_assertCriticalError(const int32_t errorCode, const char* functionName, uint32_t lineNumber);
extern int32_t NetfpProxy_logInit(void);
extern int32_t NetfpProxy_logDeInit(void);
extern int32_t NetfpProxy_dumpInit(void);
extern void NetfpProxy_dumpDeInit(void);
extern void NetfpProxy_syslogMsg (NetfpProxy_LogLevel level, const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
extern void NetfpProxy_dumpMsg (NetfpProxy_LogLevel level, const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
// </fzm>

#ifdef __cplusplus
}
#endif

#endif
