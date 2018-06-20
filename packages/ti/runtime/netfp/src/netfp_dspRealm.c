/**
 *   @file  netfp_dspRealm.c
 *
 *   @brief
 *      The file implements the NETFP functionality which is DSP realm
 *      specific.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2014 Texas Instruments, Inc.
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
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* PDK & CSL Include Files */
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_cache.h>

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ************************ NETFP-DSP Realm Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function returns a unique identifier which can be used to send commands
 *      to the PA subsystem and also used for uniquely identifying security
 *      channels.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Allocated identifier.
 */
uint32_t Netfp_getUniqueId(void) //fzm
{
    return TSCL;
}

/**
 *  @b Description
 *  @n
 *      This API implements a clock delay logic using the Time Stamp
 *      Counter (TSC) of the DSP.
 *
 *  @param[in]  count
 *      Number of delay cycles to wait.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_cycleDelay(int32_t const count)
{
    int32_t const startTime = TSCL;

    /* Loop around waiting until delay cycles elapse */
    while (((int32_t)TSCL - startTime) < count);

    /* Return success */
    return;
}
