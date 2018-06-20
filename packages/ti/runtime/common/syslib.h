/**
 *   @file  syslib.h
 *
 *   @brief
 *      Main header file for the SYSLIB package
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
 *  \par
 */

#ifndef __SYSLIB_H__
#define __SYSLIB_H__

#include <stdlib.h>

#if defined(DEVICE_K2H)
/*************************************************************
 * K2H Specific Definitions
 *************************************************************/
#include <ti/csl/device/k2h/src/cslr_device.h>
#include <ti/csl/device/k2h/src/csl_qm_queue.h>

/* ARM Master Core Identifier */
#define SYSLIB_ARM_CORE_ID          8

/* NETCP is NSS GEN1 */
#undef NSS_GEN2

#elif defined(DEVICE_K2K)
/*************************************************************
 * K2K Specific Definitions
 *************************************************************/
#include <ti/csl/device/k2k/src/cslr_device.h>
#include <ti/csl/device/k2k/src/csl_qm_queue.h>

/* ARM Master Core Identifier */
#define SYSLIB_ARM_CORE_ID          8

/* NETCP is NSS GEN1 */
#undef NSS_GEN2

#elif defined(DEVICE_K2L)
/*************************************************************
 * K2L Specific Definitions
 *************************************************************/
#include <ti/csl/device/k2l/src/cslr_device.h>
#include <ti/csl/device/k2l/src/csl_qm_queue.h>

/* ARM Master Core Identifier: Even though there are 4 DSP Cores
 * the specification states that the IPCGR interrupts for ARM are at
 * 8 and 9*/
#define SYSLIB_ARM_CORE_ID          8

/* NETCP is NSS GEN1 */
#define NSS_GEN2
#else
#error "Error: Unsupported device"
#endif

/* Base Error code for all the SYSLIB modules. */
#define SYSLIB_ERRNO_DAT_BASE               -2000
#define SYSLIB_ERRNO_DOMAIN_BASE            -3000
#define SYSLIB_ERRNO_DPM_BASE               -4000
#define SYSLIB_ERRNO_FAPI_TRACING_BASE      -5000
#define SYSLIB_ERRNO_JOSH_BASE              -6000
#define SYSLIB_ERRNO_MSGCOM_BASE            -7000
#define SYSLIB_ERRNO_NAME_BASE              -8000
#define SYSLIB_ERRNO_NETFP_BASE             -9000
#define SYSLIB_ERRNO_PKTLIB_BASE            -10000
#define SYSLIB_ERRNO_RESMGR_BASE            -11000
#define SYSLIB_ERRNO_ROOT_BASE              -12000
#define SYSLIB_ERRNO_UINTC_BASE             -13000
#define SYSLIB_ERRNO_NETFP_MASTER_BASE      -14000
#define SYSLIB_ERRNO_MEMLOG_BASE            -15000

#ifdef __ARMv7
/**
 *  @b Description
 *  @n
 *      This is a common API which is used across all the SYSLIB modules
 *      to determine the location for the run time directory. The run time
 *      directory is used to store UNIX Socket file endpoints etc.
 *
 *  @retval
 *      Pointer to the SYSLIB Runtime directory.
 */
static inline const char* Syslib_getRunTimeDirectory(void)
{
    const char* ptrRuntimeDirectory;

    /* Get the runtime directory from the environment variable */
    ptrRuntimeDirectory = getenv ("SYSLIB_RUNTIME_DIR");
    if (ptrRuntimeDirectory == NULL)
    {
        /* The environment variable is not specified default to the FHS standard */
        /*fzm*/
        ptrRuntimeDirectory = "/tmp";
    }

    /* Return the runtime directory */
    return ptrRuntimeDirectory;
}
#endif  /* __ARMv7 */

#endif /* __SYSLIB_H__ */

