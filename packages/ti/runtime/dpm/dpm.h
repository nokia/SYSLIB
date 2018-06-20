/**
 *   @file  dpm.h
 *
 *   @brief
 *      Header file for the Downlink Packet Management.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */

/** @defgroup DPM_API DPM API
 */

#ifndef __DPM_H__
#define __DPM_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>

/**
@defgroup DPM_SYMBOL  DPM Symbols 
@ingroup DPM_API
*/
/**
@defgroup DPM_ERROR_CODE  DPM Error codes
@ingroup DPM_API
*/
/**
@defgroup DPM_FUNCTION  DPM API Functions
@ingroup DPM_API
*/
/**
@defgroup DPM_INTERNAL_FUNCTION  DPM Internal Functions
@ingroup DPM_API
*/
/**
@defgroup DPM_DATASTRUCT  DPM Data Structures
@ingroup DPM_API
*/
/**
@defgroup DPM_OSAL_API  DPM OSAL Functions
@ingroup DPM_API
*/

/** @addtogroup DPM_SYMBOL
 @{ */

/** 
 * @brief 
 *  DPM Maximum number of UE supported by the DPM Driver
 */
#define DPM_MAX_UE              256

/** 
 * @brief 
 *  DPM Maximum number of RB supported with each User Id.
 */
#define DPM_MAX_RB              11

/** 
 * @brief 
 *  DPM Maximum number of QCI supported 
 */
#define DPM_MAX_QCI             10

/**
@}
*/

/** @addtogroup DPM_ERROR_CODE
 *
 * @brief 
 *  Base error code for the DPM module is defined in the 
 *  \include ti/runtime/common/syslib.h 
 * 
 @{ */

/**
 * @brief 
 *  Error code: Invalid argument.
 */
#define DPM_EINVAL                     SYSLIB_ERRNO_DPM_BASE-1

/** 
 * @brief 
 *  Error code: No memory error.
 */
#define DPM_ENOMEM                     SYSLIB_ERRNO_DPM_BASE-2

/** 
 * @brief 
 *  Error code: Internal FATAL Error.
 */
#define DPM_EINTERNAL                  SYSLIB_ERRNO_DPM_BASE-3

/** 
 * @brief 
 *  Error code: No space
 */
#define DPM_ENOSPACE                   SYSLIB_ERRNO_DPM_BASE-4

/**
@}
*/

/** @addtogroup DPM_DATASTRUCT
 @{ */

/**
 * @brief   Packet Drop policy call back function which is registered by the 
 * application. The function is invoked by the DPM driver to determine if the packet 
 * should be allowed to be processed in the DPM driver or not. The DPM driver passes 
 * information about the number of pending packets & total pending bytes associated 
 * with the user profile to this call back function. Application developers could
 * impement a customized drop mechanism using the provided parameters
 */
typedef uint32_t (*Dpm_PacketDropPolicy)(uint8_t ueId, uint8_t rbId, uint8_t qci,
                                         uint32_t pendingPkts, uint32_t pendingBytes);

/**
 * @brief 
 *  DPM Report Type
 *
 * @details
 *  Enumeration which describes the different types of reports which is supported by
 *  the DPM module.
 */
typedef enum Dpm_ReportType
{
    /**
     * @brief   Cumulative reports are reports which are persistent and maintained
     * over the life of the specific user & radio bearer
     */
    Dpm_ReportType_CUMULATIVE   = 0x1,

    /**
     * @brief   Fresh reports maintain information only till it has not been passed
     * to the scheduler. These reports can be used to determine the delta modifications
     * in the status since the last report was retreived.
     */
    Dpm_ReportType_FRESH        = 0x2
}Dpm_ReportType;

/**
 * @brief 
 *  User Profile Status
 *
 * @details
 *  Enumeration which describes the status of the user profile. 
 */
typedef enum Dpm_UserProfileStatus
{
    /**
     * @brief   User Profile is INACTIVE
     */
    Dpm_UserProfileStatus_INACTIVE  =   0x0,

    /**
     * @brief   User Profile is ACTIVE
     */
    Dpm_UserProfileStatus_ACTIVE    =   0x1
}Dpm_UserProfileStatus;

/**
 * @brief 
 *  DPM Radio Bearer Raw Report
 *
 * @details
 *  This is the raw report generated for each UE/radio bearer by 
 *  the DPM driver and is provided to the scheduler every TTI.
 */
typedef struct Dpm_Report
{
    /**
     * @brief   UE identifier.
     */
    uint8_t     ueId;

    /**
     * @brief   RB identifier.
     */
    uint8_t     rbId;

    /**
     * @brief   QoS Class Identifier.
     */
    uint8_t     qci;

    /**
     * @brief   Number of bytes received on the radio bearer.
     */
    uint32_t    rxByteCount;

    /**
     * @brief   This is the number of pending data which has still not been
     * processed by the RLC. These are cumulative statistics.
     */
    uint32_t    rlcPendingBytes;

    /**
     * @brief   This is the number of control bytes reported by the RLC
     */
    uint32_t    rlcControlBytes;

    /**
     * @brief   This is the number of retransmission bytes reported by RLC
     */
    uint32_t    rlcRetransmissionBytes;

    /**
     * @brief   Packet Delay budget. If the value is negative it implies that 
     * the packet has exceeded the QCI configured packet delay budget.
     */
    int32_t     pdb;

    /**
     * @brief   Fresh receive byte count which is the number of bytes of
     * data received since the last time the report was retreived. 
     */
    uint32_t    freshRxByteCount;

    /**
     * @brief   Number of pending packets which have not been picked up by the RLC
     */
    uint32_t	pendingPktCount;
}Dpm_Report;

/**
 * @brief 
 *  DPM Statistics
 *
 * @details
 *  The structure contains the statistics block which is populated by the
 *  DPM.
 */
typedef struct Dpm_Stats
{
    /**
     * @brief   Total number of packets received.
     */
    uint32_t    pktReceived;

    /**
     * @brief   Total number of packets dropped.
     */
    uint32_t    pktDropped;

    /**
     * @brief   Total number of bytes received
     */
    uint64_t    bytesReceived;

    /**
     * @brief   Total number of bytes dropped.
     */
    uint64_t    bytesDropped;

    /**
     * @brief   Timestamp of the packet at the head of the QCI list
     * This field is valid only if the QCI is specified while retreiving
     * statistics.
     */
    uint64_t    packetTimestamp;

    /**
     * @brief   This is the number of active users for the specific
     * QCI. This field is valid only if the QCI is specified while 
     * retreiving statistics.
     */
    uint32_t    numActiveUE;
}Dpm_Stats;

/**
 * @brief 
 *  DPM User Profile
 *
 * @details
 *  This is the user profile which returns the user & radio bearer identifier.
 */
typedef struct Dpm_UserProfile
{
    /**
     * @brief   UE identifier.
     */
    uint8_t         ueId;

    /**
     * @brief   RB identifier.
     */
    uint8_t         rbId;
}Dpm_UserProfile;

/**
 * @brief 
 *  DPM Configuration
 *
 * @details
 *  The structure describes the DPM configuration which is passed to 
 *  initialize the DPM driver
 */
typedef struct Dpm_Config
{
    /**
     * @brief   Optional drop policy function which if provided is invoked
     * during packet reception to determine if the packet should be dropped
     * or processed by the DPM.
     */
    Dpm_PacketDropPolicy        dropPolicyFxn;

    /**
     * @brief   PKTLIB Instance handle
     */
    Pktlib_InstHandle           pktlibInstHandle;

    /**
     * @brief   QCI Packet Delay budget specified in ticks.
     */
    uint32_t                    qciPacketDelayBudget[DPM_MAX_QCI];
}Dpm_Config;

/**
@}
*/

/** @addtogroup DPM_OSAL_API
 @{ */

/**
 *  @b Description
 *  @n
 *      Allocation API which is used by the DPM framework to allocate
 *      memory for its internal purposes.
 *
 *  @param[in]  numBytes 
 *      Number of bytes to be allocated
 *  @param[in]  alignment
 *      Alignment requirements 
 *
 *  @retval
 *      Success -   Pointer to the allocated & aligned memory block
 *  @retval
 *      Error   -   NULL
 */
extern void* Dpm_osalMalloc(uint32_t numBytes, uint32_t alignment);

/**
 *  @b Description
 *  @n
 *      Memory cleanup API which is used by the DPM framework to free
 *      memory which was previously allocated for its internal purposes.
 *
 *  @param[in]  ptr 
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size 
 *      Number of bytes to be freed up
 *
 *  @retval
 *      Not Applicable.
 */
extern void Dpm_osalFree (void* ptr, uint32_t size);

/**
 *  @b Description
 *  @n
 *      The function is used by the DPM to enter a critical section
 *      since a shared resource access is starting. 
 *
 *  @retval
 *      Opaque critical section object 
 */
extern void* Dpm_osalEnterCriticalSection(void);

/**
 *  @b Description
 *  @n
 *      The function is used by the DPM to exit a critical section
 *      since a shared resource access is complete
 *      
 *  @param[in]  csHandle 
 *      Opaque critical section object
 *
 *  @retval
 *      Not Applicable.
 */
extern void Dpm_osalExitCriticalSection(void* csHandle);

/**
 *  @b Description
 *  @n
 *      Cache API which is used to invalidate the contents of the
 *      cache
 *
 *  @param[in]  ptr
 *      Address of the data which is to be invalidated
 *  @param[in]  numBytes 
 *      Number of bytes to be invalidated
 *
 *  @retval
 *      Not Applicable
 */
extern void Dpm_osalInvalidate(void* ptr, uint32_t numBytes);

/**
 *  @b Description
 *  @n
 *      Cache API which is used to writeback the contents of the
 *      cache
 *
 *  @param[in]  ptr
 *      Address of the data which is to be written back.
 *  @param[in]  numBytes 
 *      Number of bytes to be written back.
 *
 *  @retval
 *      Not Applicable
 */
extern void Dpm_osalWriteback(void* ptr, uint32_t numBytes);

/**
@}
*/

extern int32_t Dpm_init  (Dpm_Config* ptrDPMConfig, int32_t* errCode);
extern int32_t Dpm_start (void);

extern int32_t Dpm_enqueuePkt (uint8_t ueId, uint8_t rbId, uint8_t qci, Ti_Pkt* ptrPkt, int32_t* errCode);
extern int32_t Dpm_timerExpiry (uint8_t qci, uint32_t timeout, int32_t* errCode);
extern int32_t Dpm_configureUserProfile (Dpm_UserProfile* ptrUserProfile, uint8_t flag);

/* DPM RLC Interface */
extern int32_t Dpm_dequeuePkt (uint8_t ueId, uint8_t rbId, Ti_Pkt** ptrPkt, int32_t* errCode);
extern int32_t Dpm_rlcReport (uint8_t ueId, uint8_t rbId, uint32_t pendingBytes,
						      uint32_t controlBytes, uint32_t retransmissionBytes);
extern int32_t Dpm_freeRLCPkt (uint8_t ueId, uint8_t rbId, Ti_Pkt* ptrPkt);

/* DPM-Scheduler Interface */
extern int32_t Dpm_getHeadUserProfile (Dpm_ReportType type,  Dpm_UserProfile* ptrUserProfile);
extern int32_t Dpm_getNextUserProfile (Dpm_ReportType  type, Dpm_UserProfile* ptrUserProfile, 
                                       Dpm_UserProfile* ptrNextUserProfile);
extern int32_t Dpm_getSortedUserProfiles(uint32_t maxUserProfiles, Dpm_UserProfile* sortedUserProfile, 
                                         uint32_t*numUserProfiles, int32_t* errCode);
extern int32_t Dpm_getNextSortedUserProfiles(uint32_t maxUserProfiles, Dpm_UserProfile* sortedUserProfile,
                                         uint32_t*numUserProfiles, int32_t* errCode);
extern int32_t Dpm_getReport (Dpm_UserProfile* ptrUserProfile, Dpm_Report* ptrReport);

/* Debug Functions: */
extern uint32_t Dpm_dumpRB (uint8_t ueId, uint8_t rbId, uint8_t dumpPacket);
extern uint32_t Dpm_dumpQCI (uint8_t qci, uint8_t dumpPacket);

/* Statistics API: */
extern int32_t Dpm_getStats (uint16_t ueId, uint8_t rbId, uint8_t qci, Dpm_Stats* ptrStats, int32_t* errCode);
extern int32_t Dpm_clearStats (void);

#endif /* __DPM_H__ */

