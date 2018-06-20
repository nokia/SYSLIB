/**
 *   @file  dpm_internal.h
 *
 *   @brief
 *      Internal Header file used within the DPM Module. 
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
#ifndef __DPM_INTERNAL_H__
#define __DPM_INTERNAL_H__

#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/dpm/include/listlib.h>
#include <ti/runtime/pktlib/pktlib.h>

/** @addtogroup DPM_SYMBOL
 @{ */

/**
 * @brief   This is the offset within the packet used to store the DPM
 * Packet node information.
 */
#define DPM_PACKET_NODE_OFFSET      64

/**
 * @brief   This is the MAX number of packets which can be processed 
 * by the RLC layer in a single TTI
 */
#define DPM_RLC_MAX_PACKET          16

/**
@}
*/


/** @addtogroup DPM_DATASTRUCT
 @{ */

/**
 * @brief   Opaque Handle returned by the report module.
 */
typedef void*       Dpm_ReportHandle;

/**
 * @brief 
 *  DPM QCI
 *
 * @details
 *  The structure describes the DPM Quality of Service class 
 *  indicator. 
 */
typedef struct Dpm_QCI
{
    /**
     * @brief   QCI identifier.
     */
    uint8_t                 qci;

    /**
     * @brief  Packet List which keeps track of the list of received packets
     * for the QCI. The list of packets here are all packets with match a specific
     * QCI but might belong to multiple RB from different UE contexts.
     */
    Dpm_ListObj             packetList;

    /**
     * @brief  Total number of packets received on the specific user profile
     */
    uint32_t                pktReceived;

    /**
     * @brief  Total number of packet dropped on the specific user profile
     * as indicated by the policy drop function.
     */
    uint32_t                pktDropped;

    /**
     * @brief  Total number of bytes received on the specific user profile
     */
    uint64_t                bytesReceived;

    /**
     * @brief  Total number of bytes dropped on the specific user profile
     * as indicated by the policy drop function.
     */
    uint64_t                bytesDropped;
}Dpm_QCI;

/**
 * @brief 
 *  DPM Radio Bearer
 *
 * @details
 *  The structure describes a radio bearer.
 */
typedef struct Dpm_RB
{
    /**
     * @brief   User Identifier associated with the RB
     */
    uint8_t                 ueId;

    /**
     * @brief   Radio Bearer Identifier.
     */
    uint8_t                 rbId;

    /**
     * @brief  Queue Handle assoicated with the RB. This is where all the
     * packets received on the RB will be stored.
     */
    Qmss_QueueHnd           rbQueueHnd;

    /**
     * @brief  Queue Handle to hold RLC processed packets.
     */
    Qmss_QueueHnd           rlcProcessedQueue;

    /**
     * @brief  Total number of packets received on the specific user profile
     */
    uint32_t                pktReceived;

    /**
     * @brief  Total number of bytes received on the specific user profile
     */
    uint64_t                bytesReceived;

    /**
     * @brief  Packet List which also keeps track of the received packets
     * on the RB.
     */
    Dpm_ListObj             packetList;

    /**
     * @brief  Total number of packet dropped on the specific user profile
     * as indicated by the policy drop function.
     */
    uint32_t                pktDropped;

    /**
     * @brief  Total number of bytes dropped on the specific user profile
     * as indicated by the policy drop function.
     */
    uint64_t                bytesDropped;
}Dpm_RB;

/**
 * @brief 
 *  DPM User Entity
 *
 * @details
 *  The structure describes a UE.
 */
typedef struct Dpm_UE
{
    /**
     * @brief   User Identifier which uniquely identifies a UE in the system
     * These should be unique in the system.
     */
    uint8_t         ueId;

    /**
     * @brief   Radio Bearers associated with each UE.
     */
    Dpm_RB          rb[DPM_MAX_RB];
}Dpm_UE;

/**
 * @brief 
 *  DPM Packet Node
 *
 * @details
 *  The structure describes a packet node.
 */
typedef struct Dpm_PacketNode
{
    /**
     * @brief   Links to other packets in the RB chain
     */
    Dpm_ListNode    rbPktLinks;

    /**
     * @brief   Links to other packets in the QCI chain
     */
    Dpm_ListNode    qciPktLinks;

    /**
     * @brief   Radio Bearer on which the packet was received.
     */
    Dpm_RB*         ptrRB;

    /**
     * @brief   QCI on which the packet was received.
     */
    Dpm_QCI*        ptrQCI;

    /**
     * @brief   Pointer to the actual descriptor which contains the 
     * received packet.
     */
    Ti_Pkt*         ptrPkt;

    /**
     * @brief   Timestamp of the received packet.
     */
    uint64_t        timeStamp;
}Dpm_PacketNode;

/**
 * @brief 
 *  DPM Master Control Block
 *
 * @details
 *  The structure keeps track of all the information pertinent to the
 *  DPM Module.
 */
typedef struct Dpm_MCB
{
    /**
     * @brief   DPM configuration passed by the application.
     */
    Dpm_Config          config;

    /**
     * @brief   The UE database which maintains all the active UE present
     * in the system on which packets are arriving.
     */
    Dpm_UE              ueDatabase[DPM_MAX_UE];

    /**
     * @brief   The QCI database keeps track of all packets which have arrived
     * on a per QCI basis.
     */
    Dpm_QCI             qciDatabase[DPM_MAX_QCI];

    /**
     * @brief   Cumulative Raw Report Handle.
     */
    Dpm_ReportHandle    cumulativeRawReportHandle;

    /**
     * @brief   Fresh Raw Report Handle.
     */
    Dpm_ReportHandle    freshRawReportHandle;

    /**
     * @brief   RLC Report Handle.
     */
    Dpm_ReportHandle    rlcReportHandle;

    /**
     * @brief   QCI Configured Packet Delay Budget.
     */
    int32_t*            qciConfiguredDelayBudget;

    /**
     * @brief   Packet Delay Budget: For each user & radio bearer.
     */
    int32_t*            packetDelayBudget;

    /**
     * @brief   Packet Timestamp for each user & radio bearer of the packet
     * which has arrived and which is at the head of the radio bearer queue.
     */
    uint64_t*           packetTimestamp;

    /**
     * @brief   RLC Pending Bytes for each user & radio bearer
     */
    uint32_t*           rlcPendingBytes;

    /**
     * @brief   RLC Control bytes for each user & radio bearer
     */
    uint32_t*           rlcControlBytes;

    /**
     * @brief   RLC Retransmission Bytes for each user & radio bearer
     */
    uint32_t*           rlcRetransmissionBytes;

    /**
     * @brief   Cumulative Receive Byte count for each user & radio bearer
     */
    uint32_t*           rxByteCount;

    /**
     * @brief   Fresh Receive Byte count for each user & radio bearer
     */
    uint32_t*           freshRxByteCount;

    /**
     * @brief   QCI associated with user & radio bearer.
     */
    uint32_t*           qci;

    /**
     * @brief   RB Active/Inactive Status List
     */
    uint32_t*           listRBStatus;

    /**
     * @brief   Memory allocated for the bidding.
     */
    uint32_t*           bidValues;

    /**
     * @brief   Memory allocated for the bidding index.
     */
    uint32_t*           biddingIndex;

    /**
     * @brief   Memory allocated for the user profile list.
     */
    uint32_t*           userProfileList;

    /**
     * @brief   Memory allocated for the sorting of the bids. This
     * is the input buffer which holds all the bids to be sorted.
     */
    uint32_t*           inputBidsToSort;

    /**
     * @brief   Memory allocated to hold the index of the input bids
     * which are to be sorted.
     */
    uint32_t*           inputBidIndexToSort;

    /**
     * @brief   Memory allocated to hold the sorting output.
     */
    uint32_t*           sortedOutputBids;

    /**
     * @brief   Memory allocated to hold the index of the sorted output
     * bids.
     */
    uint32_t*           sortedOutputBidsIndex;
    
    /**
     * @brief   Scratch pad memory used for sorting.
     */
    uint32_t*           sortingScratchMemory;

    /**
     * @brief   Number of pending active user profiles
     */
    uint32_t            numPendingActiveUserProfiles;
}Dpm_MCB;

/**
@}
*/

/**********************************************************************
 ************************** Extern Definitions ************************
 **********************************************************************/

/* DPM Driver Master Control Block. */
extern Dpm_MCB   gDPMMCB;

/* DPM Report Module: Internal Exported Functions */
extern Dpm_ReportHandle Dpm_reportInit (int32_t* errCode);
extern int32_t Dpm_reportConfigureData(Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId, void* reportData);
extern int32_t Dpm_reportSet (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId);
extern int32_t Dpm_reportClear (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId);
extern int32_t Dpm_reportGetData (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId, void** reportData);
extern int32_t Dpm_reportFindActiveUeRb (Dpm_ReportHandle reportHandle, uint8_t* ueId, uint8_t* rbId);
extern int32_t Dpm_reportFindNextActiveUeRb (Dpm_ReportHandle reportHandle, uint8_t ueId, uint8_t rbId,
                                             uint8_t* nextUeId, uint8_t* nextRbId);

#ifdef __cplusplus
}
#endif

#endif /* __DPM_INTERNAL_H__ */

