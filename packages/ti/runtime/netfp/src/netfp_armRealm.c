/**
 *   @file  netfp_armRealm.c
 *
 *   @brief
 *      The file implements the NETFP functionality which is ARM realm
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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ddal/ddal_cpu.h>

/**************************************************************************
 ************************ NETFP-ARM Realm Functions ***********************
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
uint32_t Netfp_getUniqueId(void)
{
    //fzm replaced random() for speed improvement
    //though no guarantee that this will be unique.
    return ddal_cpu_counter_read();
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
    (void)count;
    usleep (10);
}
