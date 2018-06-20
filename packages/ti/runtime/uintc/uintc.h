/**
 *   @file  uintc.h
 *
 *   @brief
 *      Header file for the user space interrupt management layer.
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

/** @defgroup UINTC_API   User space interrupt management layer
 */
#ifndef __UINTC_H__
#define __UINTC_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>

/* Linux Include Files */
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup UINTC_FUNCTION                 User space interrupt Functions
@ingroup UINTC_API
*/
/**
@defgroup UINTC_SYMBOL                   User space interrupt defined Symbols
@ingroup UINTC_API
*/
/**
@defgroup UINTC_DATA_STRUCTURE           User space interrupt Data Structures
@ingroup UINTC_API
*/
/**
@defgroup UINTC_INTERNAL_SYMBOL          User space interrupt Internal Symbols
@ingroup UINTC_API
*/
/**
@defgroup UINTC_INTERNAL_FUNCTION        User space interrupt Internal Functions
@ingroup UINTC_API
*/
/**
@defgroup UINTC_INTERNAL_DATA_STRUCTURE  User space interrupt Internal Data Structures
@ingroup UINTC_API
*/
/**
@defgroup UINTC_ERROR_CODE               User space interrupt error Codes
@ingroup UINTC_API
*/

/** @addtogroup UINTC_SYMBOL
 @{ */

/**
 * @brief
 *  UINTC Maximum number of characters
 */
#define UINTC_MAX_CHAR              32

/**
@}
*/


/** @addtogroup UINTC_ERROR_CODE
 *
 * @brief
 *  Base error code for the UINTC module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/**
 * @brief
 *  Error: Invalid argument
 */
#define UINTC_EINVAL                                SYSLIB_ERRNO_UINTC_BASE-1

/**
 * @brief
 *  Error: Internal error.
 */
#define UINTC_EINTERR                               SYSLIB_ERRNO_UINTC_BASE-2

/**
 * @brief
 *  Error: Kernel DTS configuration mismatch for the UIO interrupt mapping
 */
#define UINTC_EDTSCFG                               SYSLIB_ERRNO_UINTC_BASE-3

/**
 * @brief
 *  Error: Out of memory error
 */
#define UINTC_ENOMEM                                SYSLIB_ERRNO_UINTC_BASE-4

/**
 * @brief
 *  Error: Duplicate interrupt registeration
 */
#define UINTC_EDUP                                  SYSLIB_ERRNO_UINTC_BASE-5

/**
 * @brief
 *  Error: UINTC Module has been deinitialized
 */
#define UINTC_EDEINIT                               SYSLIB_ERRNO_UINTC_BASE-6

/**
@}
*/

/** @addtogroup UINTC_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  This is the handle to the UINTC module.
 */
typedef void*  UintcHandle;

/**
 * @brief
 *  This is the application register ISR function which is invoked by the UINTC
 *  module when the system event is detected
 *
 *  @param[in]  arg
 *      Application supplied argument.
 */
typedef void (*UintcIsrHandler)(void* arg);

/**
 * @brief
 *  UINTC operating mode
 *
 * @details
 *  UINTC operating modes is used to define which entity is responsible for performing
 *  the select operation on all the registered interrupts.
 */
typedef enum Uintc_Mode
{
    /**
     * @brief   Application managed UINTC module implies that the application
     * will perform the "select" on all the registered interrupts. Dynamic
     * registeration and deregisteration of interrupts needs to be handled by the
     * application
     */
    Uintc_Mode_APPLICATION_MANAGED      = 0x1,

    /**
     * @brief  UINTC managed mode implies that the interrupts will be processed in the
     * "uintc_select" API and applications are notified through the registered callback
     * function. Dynamic registeration and deregisteration of interrupts are handled
     * by the UINTC module.
     *
     * @sa uintc_select
     */
    Uintc_Mode_UINTC_MANAGED            = 0x2
}Uintc_Mode;

/**
 * @brief
 *  UINTC Module configuration
 *
 * @details
 *  The UINTC module configuration is used to specify the properties of the
 *  UINTC module.
 */
typedef struct UintcConfig
{
    /**
     * @brief  Unique name for the UINTC module in the system. There could be
     * multiple UINTC modules existing in the system concurrently. It is required
     * that each of these modules be provided unique names.
     */
    char            name[UINTC_MAX_CHAR];

    /**
     * @brief  UINTC operating mode.
     */
    Uintc_Mode      mode;
}UintcConfig;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern UintcHandle Uintc_init (UintcConfig* ptrUintcConfig, int32_t* errCode);
extern int32_t Uintc_select
(
    UintcHandle     uintcHandle,
    struct timeval* ptrTimeout,
    int32_t*        errCode
);
extern int32_t Uintc_registerIsr
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    UintcIsrHandler     isrHandler,
    void*               arg,
    int32_t*            errCode
);
extern int32_t Uintc_deregisterIsr
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
);

// fzm ->
extern int32_t Uintc_registerAdditionalFd
(
    UintcHandle         uintcHandle,
    int32_t             fd,
    int32_t*            errCode
);
extern int32_t Uintc_deregisterAdditionalFd
(
    UintcHandle         uintcHandle,
    int32_t             fd,
    int32_t*            errCode
);
// <- fzm

extern int32_t Uintc_enableEvent
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
);
extern int32_t Uintc_disableEvent
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
);
extern int32_t Uintc_deinit
(
    UintcHandle         uintcHandle,
    int32_t*            errCode
);
extern int32_t Uintc_getStats
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    uint32_t*           numInterrupts,
    int32_t*            errCode
);
extern int32_t Uintc_registerMessageId
(
    UintcHandle         uintcHandle,
    uint32_t            messageId,
    UintcIsrHandler     isrHandler,
    void*               arg,
    int32_t*            errCode
);
extern int32_t Uintc_deregisterMessageId
(
    UintcHandle         uintcHandle,
    uint32_t            messageId,
    int32_t*            errCode
);
extern int32_t Uintc_sendMessageId
(
    UintcHandle     uintcHandle,
    uint32_t        messageId,
    int32_t*        errCode
);

#ifdef __cplusplus
}
#endif

#endif /* __UINTC_H__ */

