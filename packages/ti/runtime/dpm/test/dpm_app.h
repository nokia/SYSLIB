/**
 *   @file  dpm_app.h
 *
 *   @brief
 *      Application provided header file which has inline functions
 *      which allow the application to customize DPM behavior.
 *
 *      The DPM module needs to be compiled with this header file 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */

#ifndef __DPM_APP_H__
#define __DPM_APP_H__

#include <ti/runtime/dpm/dpm.h>

/* Application maintained RB Status Monitor */
extern Dpm_UserProfileStatus gQueueMonitorStatus[DPM_MAX_UE][DPM_MAX_RB];

/**
 *  @b Description
 *  @n
 *      The application call back function which is called by the DPM when
 *      there is a change in the status of the specific user profile. 
 *
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer
 *  @param[in]  status
 *      Status of the profile.
 *
 *  @retval
 *      Not Applicable
 */
static inline void Dpm_appStatusNotify
(
    uint8_t                 ueId, 
    uint8_t                 rbId, 
    Dpm_UserProfileStatus   status
)
{
    /* The call back functions are invoked by the DPM module whenever there is a change
     * in status of the Radio Bearer i.e. moving from the ACTIVE to the INACTIVE state
     * or vice versa. If the Radio bearer remains in the same state the call back 
     * function should *NOT* be triggered. */
    if (status == Dpm_UserProfileStatus_ACTIVE)
    {
        /* RADIO Bearer is moving to ACTIVE state. Are we already active? */
        if (gQueueMonitorStatus[ueId][rbId] == Dpm_UserProfileStatus_ACTIVE)
            System_printf ("Error: UE Id %d Radio Bearer %d is already ACTIVE\n", ueId, rbId);
    }
    else
    {
        /* RADIO Bearer is moving to INACTIVE state. Are we already inactive? */
        if (gQueueMonitorStatus[ueId][rbId] == Dpm_UserProfileStatus_INACTIVE)
            System_printf ("Error: UE Id %d Radio Bearer %d is already INACTIVE\n", ueId, rbId);
    }
    /* Set the new state of the Radio bearer */
    gQueueMonitorStatus[ueId][rbId] = status;
}

/**
 *  @b Description
 *  @n
 *      The application provided Bidding function which takes as 
 *      input the various information provided by the DPM module and
 *      return the associated bidding value.
 *
 *      It is recommended that this function be provided as an inline
 *      function else the overhead in calling a function will have a
 *      serious performance impact on the DPM API 
 *
 *  @sa
 *      Dpm_getSortedUserProfiles  
 *
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer
 *  @param[in]  qci
 *      QoS Class Indicator
 *  @param[in]  rxByteCount
 *      Total number of received bytes pending in the RB which have
 *      not yet been scheduled
 *  @param[in]  rlcPendingBytes
 *      Total number of received bytes pending at the RLC layer for
 *      the specific RB which are yet to be processed.
 *  @param[in]  rlcControlBytes
 *      Total number of control bytes received from the RLC
 *  @param[in]  rlcRetransmissionBytes
 *      Total number retransmission bytes received from the RLC
 *  @param[in]  holDelay
 *      Head of the line delay.
 *
 *  @retval
 *      Bid Value
 */
static inline uint32_t Dpm_appBidFunction 
(
    uint8_t     ueId, 
    uint8_t     rbId,
    uint8_t     qci,
    uint32_t    rxByteCount,
    uint32_t    rlcPendingBytes,
    uint32_t    rlcControlBytes,
    uint32_t    rlcRetransmissionBytes,
    uint32_t    holDelay
)
{
    return (10*ueId + rbId);
}

#endif /* __DPM_APP_H__ */

