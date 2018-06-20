/**
 *   @file  resmgr_system.c
 *
 *   @brief
 *      Resource Manager System Initialization code.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013 Texas Instruments, Inc.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <unistd.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/runtime/hplib/hplib.h>
#include <ti/runtime/hplib/hplib_vm.h>

/* SYSLIB Include Files */
#include <ti/apps/resmgr_server/include/listlib.h>
#include <ti/apps/resmgr_server/include/resmgr_server.h>

#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#endif

/**********************************************************************
 ****************** Resource Manager System Functions *****************
 **********************************************************************/

/** @addtogroup RESMGR_FUNCTION
 @{ */

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the system components
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t ResmgrServer_systemInit (Rm_Handle rmServerHandle)
{
    int32_t                 result;
    hplib_memPoolAttr_T     memPoolAttributes[2];
    hplib_virtualAddrInfo_T hplibVirtualAddressInfo;

    /* Init attributes for DDR */
    memPoolAttributes[0].attr       = HPLIB_ATTR_KM_CACHED0;
    memPoolAttributes[0].phys_addr  = 0;
    memPoolAttributes[0].size       = 0;

    /* Create the shared memory segment. */
    hplib_shmCreate(HPLIB_SHM_SIZE);

    /* Initialize the virtual memory */
    result = hplib_vmInit(&hplibVirtualAddressInfo, 1, memPoolAttributes);
    if (result != hplib_OK)
    {
        ResmgrServer_log(ResmgrServer_LogLevel_ERROR, "Error: HPLIB VM Initialization failed.\n");
        return -1;
    }
    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: HPLIB VM Initialization done.\n");

    /* Initialize the memory for the specific pool. */
    hplib_initMallocArea(0);
    return 0;
}

/**
@}
*/

