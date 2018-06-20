/**
 *   @file  dpm_report.c
 *
 *   @brief
 *      The file implements the Downlink Packet Management Report 
 *      functionality.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/dpm/dpm.h>
#include <ti/runtime/dpm/include/dpm_internal.h>
#include <ti/runtime/dpm/include/listlib.h>

/**********************************************************************
 ************************* Local Structures ***************************
 **********************************************************************/

/** @addtogroup DPM_DATASTRUCT
 @{ */

/**
 * @brief 
 *  DPM Report Information
 *
 * @details
 *  The structure contains the precomputed address & bit positions
 *  which can be used to set/get the report status. This information
 *  is precalculated during initialization time 
 */
typedef struct Dpm_ReportInfo
{
    /**
     * @brief   UE Report Status address which is the address which
     * has to be updated when a packet is received on the specific UE
     */
    uint32_t*       ueRawStatusAddress;

    /**
     * @brief   RB Report Status address which is the address which
     * has to be updated when a packet is received on the specific RB
     */
    uint32_t*       rbRawStatusAddress;
}Dpm_ReportInfo;

/**
 * @brief 
 *  DPM Report Format
 *
 * @details
 *  The report module is a generic module which operates on the following
 *  basic structure which allows fast access to determine a list of all
 *  UE and RB which have pending data. The format of the pending data is
 *  *opaque* and is not used by this module.
 */
typedef struct Dpm_ReportFormat
{
    /**
     * @brief   Report Status: This bitmask is used to indicate if the specific
     * UE is active or not? 
     */
    uint32_t        ueReportStatus[DPM_MAX_UE/32 + 1];

    /**
     * @brief   Report Status: This bitmask is used to indicate if the specific
     * RB associated with a UE is active or not?
     */
    uint32_t        rbReportStatus[DPM_MAX_UE];

    /**
     * @brief   The structure contains precomputed address & bit masks which can
     * be used to get/set the UE & RB report status efficiently.
     */
    Dpm_ReportInfo  rbReportInfo[DPM_MAX_UE][DPM_MAX_RB];

    /**
     * @brief   Each RB of a specific user is associated with an opaque data.
     */
    void*           rbReportData[DPM_MAX_UE][DPM_MAX_RB];
}Dpm_ReportFormat;

/**
@}
*/

/**********************************************************************
 ************************ DPM Report Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Internal helper function which is used to normalize the bit 
 *      position while storing/clearing the bit fields.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  bitPosition
 *      Bit Position to be normalized.
 *
 *  @retval
 *      Normalized bit Position
 */
static inline uint8_t Dpm_reportNormalizeBitPosition (uint8_t bitPosition)
{
#ifdef _LITTLE_ENDIAN
    return 31 - bitPosition;
#else
    return bitPosition;
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the report module.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[out]  errCode
 *      Error Code populated only if there was an error
 *
 *  @retval
 *      Success     -   Report Handle
 *  @retval
 *      Error       -   NULL
 */
Dpm_ReportHandle Dpm_reportInit (int32_t* errCode)
{
    Dpm_ReportFormat*   ptrReportFormat;
    Dpm_ReportInfo*     ptrReportInfo;
    uint32_t            ueIndex;
    uint32_t            rbIndex;
    uint32_t            ueBitPosition;
    uint32_t*           ptrRBStatusAddress;
    uint32_t*           ptrUEStatusAddress;

    /* Allocate memory for the report instance which is being created. */
    ptrReportFormat = Dpm_osalMalloc(sizeof(Dpm_ReportFormat), 0);
    if (ptrReportFormat == NULL)
    {
        *errCode = DPM_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory. */
    memset ((void*)ptrReportFormat, 0, sizeof(Dpm_ReportFormat));

    /* Get the pointer to the UE Status & the RB status */
    ptrUEStatusAddress = &ptrReportFormat->ueReportStatus[0];
    ptrRBStatusAddress = &ptrReportFormat->rbReportStatus[0];

    /* Populate the report information field. */
    for (ueIndex = 0; ueIndex < DPM_MAX_UE; ueIndex++)
    {
        /* Determine the bit position for the UE. */
        if ((ueIndex % 32) == 0)
            ueBitPosition = 0;

        /* Set the report information in the RB appropriately. */ 
        for (rbIndex = 0; rbIndex < DPM_MAX_RB; rbIndex++)
        {
            /* Get the report information field. */
            ptrReportInfo = &ptrReportFormat->rbReportInfo[ueIndex][rbIndex];

            /* Populate the report information field. */
            ptrReportInfo->rbRawStatusAddress  = ptrRBStatusAddress;
            ptrReportInfo->ueRawStatusAddress  = ptrUEStatusAddress;
        }
        
        /* We are done with the specific UE; so we move to the next RB address. */
        ptrRBStatusAddress = ptrRBStatusAddress + 1;

        /* Increment the UE Bit position. */
        ueBitPosition = ueBitPosition + 1;

        /* We are tracking 32 UE per address. So increment the address appropriately. */
        if (((ueIndex + 1) % 32) == 0)
            ptrUEStatusAddress = ptrUEStatusAddress + 1;
    }

    /* The report has been successfully initialized. */
    return (Dpm_ReportHandle)ptrReportFormat;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the report data associated with a User
 *      & specific radio bearer. The data is NOT interpreted by the report module
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  reportData
 *      Reporting Data to be associated with the user & radio bearer
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_reportConfigureData
(
    Dpm_ReportHandle    reportHandle, 
    uint8_t             ueId, 
    uint8_t             rbId, 
    void*               reportData
)
{
    Dpm_ReportFormat*   ptrReportFormat;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Associate the reporting data for the specific user & radio bearer. */
    ptrReportFormat->rbReportData[ueId][rbId] = reportData;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked to indicate to the reporting module that the
 *      specific user & radio bearer is now active. 
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_reportSet (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId)
{
    Dpm_ReportFormat*   ptrReportFormat;
    Dpm_ReportInfo*     ptrReportInfo;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Get the report information field. */
    ptrReportInfo = &ptrReportFormat->rbReportInfo[ueId][rbId];

    /* Set the bit in the UE status indicating that the UE is ACTIVE. */
    *ptrReportInfo->ueRawStatusAddress = _set (*ptrReportInfo->ueRawStatusAddress, 
                                                Dpm_reportNormalizeBitPosition (ueId), 
                                                Dpm_reportNormalizeBitPosition (ueId));

    /* Set the bit in the RB associated with the UE that it is ACTIVE. */
    *ptrReportInfo->rbRawStatusAddress = _set (*ptrReportInfo->rbRawStatusAddress, 
                                                Dpm_reportNormalizeBitPosition (rbId), 
                                                Dpm_reportNormalizeBitPosition (rbId));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked to indicate to the reporting module that the
 *      specific user & radio bearer is now inactive. 
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_reportClear (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId)
{
    Dpm_ReportFormat*   ptrReportFormat;
    Dpm_ReportInfo*     ptrReportInfo;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Get the report information field. */
    ptrReportInfo = &ptrReportFormat->rbReportInfo[ueId][rbId];

    /* Clear the bit in the RB associated with the UE that it is now inactive */
    *ptrReportInfo->rbRawStatusAddress = _clr (*ptrReportInfo->rbRawStatusAddress, 
                                               Dpm_reportNormalizeBitPosition (rbId), 
                                               Dpm_reportNormalizeBitPosition (rbId));

    /* Clear the bit in the UE status indicating that the UE is inactive only if
     * there is no active radio bearer */
    if (*ptrReportInfo->rbRawStatusAddress == 0)
    {  
        *ptrReportInfo->ueRawStatusAddress = _clr (*ptrReportInfo->ueRawStatusAddress, 
                                                   Dpm_reportNormalizeBitPosition (ueId), 
                                                   Dpm_reportNormalizeBitPosition (ueId));
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the reporting data associated with a specific user
 *      and radio bearer.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[out]  reportData
 *      Report Data associated with the user & radio bearer
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_reportGetData (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId, void** reportData)
{
    Dpm_ReportFormat*   ptrReportFormat;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Get the reporting data. */
    *reportData = ptrReportFormat->rbReportData[ueId][rbId];
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to iterate the report to determine an active user & radio
 *      bearer
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[out]  ueId
 *      User Identifier which is active
 *  @param[out]  rbId
 *      Radio Bearer Identifier which is active 
 *
 *  @retval
 *      0   -   Active UE was detected
 *  @retval
 *      -1  -   No active UE
 */
int32_t Dpm_reportFindActiveUeRb (Dpm_ReportHandle reportHandle, uint8_t* ueId, uint8_t* rbId)
{
    uint32_t            ueBlockIndex;
    uint8_t             blockId = 0;
    uint32_t            ueReportStatus;
    uint32_t            rbReportStatus;
    Dpm_ReportFormat*   ptrReportFormat;
    Dpm_ReportInfo*     ptrReportInfo;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Cycle through all the UE contexts. */
    for (ueBlockIndex = 0; ueBlockIndex < DPM_MAX_UE; ueBlockIndex = ueBlockIndex + 32)
    {
        /* Get the UE status for the the 32 users */
        ueReportStatus = *(ptrReportFormat->ueReportStatus + blockId);

        /* Is there any data pending on this block of 32 users? */
        if (ueReportStatus == 0)
        {
            /* Skip this & jump to the next set block of 32 users. */
            blockId = blockId + 1;
            continue;
        }

        /* Compute the corresponding user identifer. */
        *ueId = (blockId * 32) + _lmbd (0x1, ueReportStatus);;

        /* Get the report information field for the specific UE */
        ptrReportInfo = &ptrReportFormat->rbReportInfo[*ueId][0];

        /* Get the status of all the RB associated with the UE. */
        rbReportStatus = *(ptrReportInfo->rbRawStatusAddress);

        /* Is there any data received on any of the radio bearers? */
        if (rbReportStatus == 0)
            return -1;

        /* Get the radio bearer identifier. */
        *rbId = _lmbd (0x1, rbReportStatus);

        /* We have found an active user and an associated radio bearer which is active. */
        return 0;
    }

    /* Control comes here implies that there was no user which was active. */
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to iterate through the report starting from the 
 *      specific user identifier and radio bearer and searching for the next
 *      active user identifer & radio bearer.
 *
 *  @param[in]  reportHandle
 *      Report Handle
 *  @param[in]  ueId
 *      User Identifier to start the search
 *  @param[in]  rbId
 *      Radio Bearer Identifier to start the search
 *  @param[out] nextUeId
 *      Next User Identifier which is active
 *  @param[out] nextRbId
 *      Next Radio Bearer which is active
 *
 *  @retval
 *      0   -   Active UE was detected
 *  @retval
 *      -1  -   No active UE
 */
int32_t Dpm_reportFindNextActiveUeRb 
(
    Dpm_ReportHandle reportHandle, 
    uint8_t          ueId,
    uint8_t          rbId, 
    uint8_t*         nextUeId,
    uint8_t*         nextRbId
)
{
    uint32_t            ueBlockIndex;
    uint8_t             blockId = 0;
    uint8_t             nextBlockId;
    uint32_t            ueReportStatus;
    uint32_t            rbReportStatus;
    Dpm_ReportFormat*   ptrReportFormat;
    Dpm_ReportInfo*     ptrReportInfo;

    /* Get the report format. */
    ptrReportFormat = (Dpm_ReportFormat*)reportHandle;

    /* Ok. We need to skip to the next radio bearer for the specific UE.
     * But if this was the last radio bearer we then to move to the 
     * next UE */
    if (rbId != (DPM_MAX_RB - 1))
    {
        /* Same user but now we start the search for the next radio bearer. */
        ptrReportInfo = &ptrReportFormat->rbReportInfo[ueId][0];

        /* Get the status of all the RB associated with the UE. */
        rbReportStatus = (*ptrReportInfo->rbRawStatusAddress);

        /* Clear all radio bearers which are less than one the specified. */
        rbReportStatus = _clr (rbReportStatus, Dpm_reportNormalizeBitPosition (rbId), 
                               Dpm_reportNormalizeBitPosition(0));

        /* Is there any data received on any of the radio bearers for this UE?  */
        if (rbReportStatus != 0)
        {
            /* YES. Get the radio bearer identifier. */
            *nextRbId = _lmbd (0x1, rbReportStatus);
            *nextUeId = ueId;

            /* Next active user id & radio bearer found */
            return 0;
        }
        else
        {
            /* NO. None of the radio bearers on the specific UE were active;
             * we need to start the search now from the next UE. So fall through! */
        }
    }

    /* If this is already the last user we cannot proceed. The search is complete. */
    if (ueId == DPM_MAX_UE)
        return -1;    

    /* Get the block identifier for the current user identifier. */
    blockId = ueId/32;

    /* Get the block identifier for the next user identifier. */
    nextBlockId = (ueId + 1)/32;
    
    /* Start the search from the new user identifier. */
    for (ueBlockIndex = ueId; ueBlockIndex < DPM_MAX_UE; ueBlockIndex = ueBlockIndex + 32)
    {
        /* Get the UE status for the 32 users */
        ueReportStatus = *(ptrReportFormat->ueReportStatus + nextBlockId);

        /* If the current user & next user start in the same block; then in the first iteration of
         * the loop we need to reset the status bit of all the users before the current user since
         * the search is incremental. */
        if (blockId == nextBlockId)
        {
            /* Is this the first iteration of the loop? */
            if (ueBlockIndex == ueId)
                ueReportStatus = _clr (ueReportStatus, Dpm_reportNormalizeBitPosition(ueId), 
                                       Dpm_reportNormalizeBitPosition(0));
        }

        /* Is there any data pending on this block of 32 users? */
        if (ueReportStatus == 0)
        {
            /* Skip this and jump to the next block of users */
            nextBlockId = nextBlockId + 1;
            continue;
        }

        /* YES. Get the user identifier. */
        *nextUeId = (nextBlockId * 32) + _lmbd (0x1, ueReportStatus);

        /* Get the report information field for the specific UE */
        ptrReportInfo = &ptrReportFormat->rbReportInfo[*nextUeId][0];

        /* Get the status of all the RB associated with the UE. */
        rbReportStatus = (*ptrReportInfo->rbRawStatusAddress);

        /* Is there any data received on any of the radio bearers? */
        if (rbReportStatus == 0)
            return -1;

        /* Get the radio bearer identifier. */
        *nextRbId = _lmbd (0x1, rbReportStatus);

        /* We have found an active user and an associated radio bearer which is active. */
        return 0;
    }    

    /* Control comes here implies that there was no user which was active. */
    return -1;
}

