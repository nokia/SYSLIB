/**
 *   @file  dpm.c
 *
 *   @brief
 *      The file implements the Downlink Packet Management Functionality.
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
#include <c6x.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cache.h>

/* SYSLIB Include Files. */
#include <ti/runtime/dpm/dpm.h>
#include <ti/runtime/dpm/include/dpm_internal.h>
#include <ti/runtime/dpm/include/sch_bytesort.h>
#include <ti/runtime/dpm/include/sch_mergesort.h>
#include <ti/runtime/dpm/include/listlib.h>

/* Debug: */
#include <xdc/runtime/System.h>

/* Application provided functionality. */
#include "dpm_app.h"

/**********************************************************************
 ************************* Local Definitions **************************
 **********************************************************************/

/**
 * @brief   Internal definition which is used to debug the DPM selection
 * and sorting outputs. This is for internal debug only
 */
#undef  DPM_DEBUG

/**
 * @brief   DPM Application specified bidding function support is enabled
 * or not? If enabled it is required that the implementation of the 
 * function "Dpm_appBidFunction" be specified in the dpm_app.h
 */
#define DPM_APP_BID_FXN_SUPPORT

/**
 * @brief   Internal flag to determine if the user profile is active 
 * or not i.e. data is pending or not?
 */
#define DPM_USER_PROFILE_ACTIVE_BIT_FLAG    1

/**
 * @brief   Internal flag to determine if the user profile has been 
 * enabled for DPM monitoring or not?
 */
#define DPM_USER_PROFILE_ENABLE_BIT_FLAG    2

/**
 * @brief   Internal definition which is used to benchmark internal DPM
 * API. This is only for internal purposes.
 */
#undef DPM_BENCHMARK_ENABLE

#ifdef DPM_BENCHMARK_ENABLE

/**
 * @brief   The MACRO is used to initialize variables used for benchmarking
 */
#define DPM_BENCHMARK_INIT(X)           uint64_t    X

/**
 * @brief   The MACRO takes a snapshot of the DSP timestamp and records it
 */
#define DPM_BENCHMARK(X)                X = _itoll (TSCH, TSCL)

/**
 * @brief   The MACRO logs the benchmarking information on the console. X is the
 * starting timestamp and Y is the ending timestamp
 */
#define DPM_BENCHMARK_LOG(STR, X, Y)    System_printf (#STR " took %d ticks\n", (Y - X))

#else

#define DPM_BENCHMARK_INIT(X)
#define DPM_BENCHMARK(X)          
#define DPM_BENCHMARK_LOG(STR, X, Y)    

#endif /* DPM_BENCHMARK_ENABLE */

/**********************************************************************
 ************************* Global Variables ***************************
 **********************************************************************/

/**
 * @brief   Global DPM Master Control block.
 */
#pragma DATA_ALIGN   (gDPMMCB, CACHE_L2_LINESIZE)
#pragma DATA_SECTION (gDPMMCB, ".dpm");
Dpm_MCB   gDPMMCB;

/**********************************************************************
 *************************** DPM Functions ****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get a RB Block for
 *      a specific UE given the RB index.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  ueIndex
 *      User Identifier
 *  @param[in]  rbIndex
 *      RB Index for which the block is required.
 *
 *  @retval
 *      Pointer to the RB Block
 */
static inline Dpm_RB* Dpm_getRB(uint8_t ueIndex, uint8_t rbIndex)
{
    /* Return the pointer to the RB block. */
    return &gDPMMCB.ueDatabase[ueIndex].rb[rbIndex];
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get a QCI Block 
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  qciIndex
 *      QoS Class Indicator 
 *
 *  @retval
 *      Success     -   Pointer to the QCI Block
 *  @retval
 *      Error       -   NULL 
 */
static inline Dpm_QCI* Dpm_getQCI(uint8_t qciIndex)
{
    /* Return the pointer to the QCI block. */
    return &gDPMMCB.qciDatabase[qciIndex];
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to get the packet node
 *      information from the received packet. The packet node is stored
 *      in each packet.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *
 *  @retval
 *      Pointer to the packet node information.
 */
static inline Dpm_PacketNode* Dpm_getPacketNode(Ti_Pkt* ptrPkt)
{
    return (Dpm_PacketNode*)((uint8_t*)ptrPkt + DPM_PACKET_NODE_OFFSET);
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which gives the index to be used to 
 *      access the various data structures given a specific user &
 *      radio bearer
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer
 *
 *  @retval
 *      Unique index which is only applicable for the specific
 *      user & radio bearer.
 */
static inline uint16_t Dpm_getUniqueIndex(uint8_t ueId, uint8_t rbId)
{
    return ((ueId * DPM_MAX_RB) + rbId);
}

/**
 *  @b Description
 *  @n
 *      The function is used to process a received packet by the DPM 
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  qci
 *      QCI associated with the radio bearer.
 *  @param[in]  ptrPkt
 *      Pointer to the received packet.
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
int32_t Dpm_enqueuePkt (uint8_t ueId, uint8_t rbId, uint8_t qci, Ti_Pkt* ptrPkt, int32_t* errCode)
{
    Dpm_RB*         ptrRB;
    Dpm_PacketNode* ptrPacketNode;
    Dpm_QCI*        ptrQCI;
    uint32_t        rbPacketCount;
    void*           csContext;
    uint16_t        uniqueIndex;
    uint32_t        numPendingBytes;

    /* Get the packet node from the received packet. */
    ptrPacketNode = Dpm_getPacketNode(ptrPkt);

    /* Get the pointer to the RB. */
    ptrRB = Dpm_getRB(ueId, rbId);

    /* Get the pointer to the QCI */
    ptrQCI = Dpm_getQCI(qci);

    /* Get the unique index associated with the user & radio bearer. */
    uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

    /* Increment the statistics for the RB */
    ptrRB->pktReceived++;
    ptrRB->bytesReceived = ptrRB->bytesReceived + Pktlib_getPacketLen(ptrPkt);

    /* Increment the statistics for the QCI */
    ptrQCI->pktReceived++;
    ptrQCI->bytesReceived = ptrQCI->bytesReceived + Pktlib_getPacketLen(ptrPkt);

    /* Get the number of pending bytes if the hardware queue */
    numPendingBytes = Qmss_getQueueByteCount(ptrRB->rbQueueHnd);

    /* Is there a drop policy function specified? */
    if (gDPMMCB.config.dropPolicyFxn != NULL)
    {
        /* YES. Invoke the drop policy function? */
        if (gDPMMCB.config.dropPolicyFxn (ueId, rbId, qci, Qmss_getQueueEntryCount (ptrRB->rbQueueHnd), 
                                          (numPendingBytes + gDPMMCB.rlcPendingBytes[uniqueIndex])) == 0)
        {
            /* Set the error code. */
            *errCode = DPM_ENOSPACE;

            /* Increment the drop statistics for the RB */
            ptrRB->pktDropped++;
            ptrRB->bytesDropped = ptrRB->bytesDropped + Pktlib_getPacketLen(ptrPkt);

            /* Increment the drop statistics for the QCI */
            ptrQCI->pktDropped++;
            ptrQCI->bytesDropped = ptrQCI->bytesDropped + Pktlib_getPacketLen(ptrPkt);

            /* Packet is being dropped. */
            return -1;
        }
    }

    /* Control comes here implies that either the drop policy function was NOT specified or if
     * specified it accepted the packet and passed it to the DPM module to be processed. 
     *
     * Critical Section Enter: */
    csContext = Dpm_osalEnterCriticalSection();

    /* Get the current packet count of the RB */
    rbPacketCount = Dpm_listGetCount(&ptrRB->packetList);

    /* Populate the time stamp for the received packet. */
    ptrPacketNode->timeStamp = _itoll(TSCH, TSCL);
    ptrPacketNode->ptrRB     = ptrRB;
    ptrPacketNode->ptrQCI    = ptrQCI;
    ptrPacketNode->ptrPkt    = ptrPkt;

    /* Is the RB currently empty and is now moving to an active stage? */
    if (rbPacketCount == 0)
    {
        /* The user profile is now ACTIVE. */
        Dpm_appStatusNotify (ueId, rbId, Dpm_UserProfileStatus_ACTIVE);

        /* Make sure we mark it active in our list also. */
        gDPMMCB.listRBStatus[uniqueIndex] |= DPM_USER_PROFILE_ACTIVE_BIT_FLAG;

        /* First packet in the radio bearer: Update the packet time stamps */
        gDPMMCB.packetTimestamp[uniqueIndex] = ptrPacketNode->timeStamp;
    }

    /* Insert the packet into the RB Packet List */
    Dpm_listEnqueue (&ptrRB->packetList, (Dpm_ListNode *)&ptrPacketNode->rbPktLinks);

    /* Insert the packet into the QCI Packet List */
    Dpm_listEnqueue (&ptrQCI->packetList, (Dpm_ListNode *)&ptrPacketNode->qciPktLinks);

    /* Push the packet into the Radio bearer hardware queue: We assume that all descriptors are 128 byte aligned. */
    Qmss_queuePush(ptrRB->rbQueueHnd, (Cppi_Desc*)ptrPkt, Pktlib_getPacketLen(ptrPkt), 128, Qmss_Location_TAIL);

    /* Mark it in the report only if the user profile is enabled. */
    if (gDPMMCB.listRBStatus[uniqueIndex] & DPM_USER_PROFILE_ENABLE_BIT_FLAG)
    {
        /* Data has arrived for the specific user & radio bearer and so mark the cumulative & fresh reports  */
        Dpm_reportSet (gDPMMCB.cumulativeRawReportHandle, ptrRB->ueId, ptrRB->rbId);
        Dpm_reportSet (gDPMMCB.freshRawReportHandle, ptrRB->ueId, ptrRB->rbId);
    }

    /* Update the fields in the DPM Lists: 
     *  - QCI List -> Updated to the new received QCI
     *  - Cumulative Receive Byte count -> Total Pending received data bytes 
     *  - Fresh Receive Byte Count -> Total received bytes. */
    gDPMMCB.qci[uniqueIndex]              = qci;
    gDPMMCB.rxByteCount[uniqueIndex]      = numPendingBytes + Pktlib_getPacketLen(ptrPkt);
    gDPMMCB.freshRxByteCount[uniqueIndex] = gDPMMCB.freshRxByteCount[uniqueIndex] + Pktlib_getPacketLen(ptrPkt);

    /* Critical Section Exit: */
    Dpm_osalExitCriticalSection (csContext);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process a received packet by the DPM 
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[out]  ptrPkt
 *      Pointer to the received packet.
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_dequeuePkt (uint8_t ueId, uint8_t rbId, Ti_Pkt** ptrPkt, int32_t* errCode)
{
    Dpm_RB* ptrRB;

    /* Get the pointer to the RB. */
    ptrRB = Dpm_getRB(ueId, rbId);

    /* Remove the packet from the radio bearer. */
    *ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrRB->rbQueueHnd));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free the RLC Packet.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  ptrPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_freeRLCPkt (uint8_t ueId, uint8_t rbId, Ti_Pkt* ptrPkt)
{
    Dpm_RB* ptrRB;

    /* Get the pointer to the RB. */
    ptrRB = Dpm_getRB(ueId, rbId);

    /* Push the packet into the RB RLC processed packet queue */
    Qmss_queuePushDesc(ptrRB->rlcProcessedQueue, (Cppi_Desc*)ptrPkt);
	return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the RLC to indicate to the DPM the amount
 *      of data which was not processed in the TTI. This function needs to be
 *      invoked for each RLC instance even if all the data was consumed and
 *      there were no pendingBytes (i.e. 0).
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  pendingBytes
 *      Pending data bytes which have not been consumed by the RLC
 *  @param[in]  controlBytes
 *      Control data bytes
 *  @param[in]  retransmissionBytes
 *      Retransmission data bytes
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_rlcReport
(
    uint8_t  ueId,
    uint8_t  rbId,
    uint32_t pendingBytes,
    uint32_t controlBytes,
    uint32_t retransmissionBytes
)
{
    uint16_t    uniqueIndex;

    /* Get the unique index for the specific user & radio bearer */
    uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

    /* Update the pending data, control and retransmission in the RLC report. */
    gDPMMCB.rlcPendingBytes[uniqueIndex]        = pendingBytes;
    gDPMMCB.rlcControlBytes[uniqueIndex]        = controlBytes;
    gDPMMCB.rlcRetransmissionBytes[uniqueIndex] = retransmissionBytes;

    /* If the RLC reports valid data mark the RB as ACTIVE. */
    if ((pendingBytes + controlBytes + retransmissionBytes) > 0)
        gDPMMCB.listRBStatus[uniqueIndex] |= DPM_USER_PROFILE_ACTIVE_BIT_FLAG;

    /* Mark the user & radio bearer active in the RLC report. */
    Dpm_reportSet (gDPMMCB.rlcReportHandle, ueId, rbId);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to discard packets belonging to a specific QCI
 *      which are older than the timeout specified.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  qci
 *      QCI for which the expiry is to be executed.
 *  @param[in]  timeout
 *      Timeout (in ticks) if packets are older than this they are discarded
 *      and removed from further processing. 
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -  <0
 */
int32_t Dpm_timerExpiry (uint8_t qci, uint32_t timeout, int32_t* errCode)
{
    Dpm_QCI*            ptrQCI;
    Dpm_ListNode*       ptrListNode;
    Dpm_PacketNode*     ptrPacketNode;
    Ti_Pkt*             ptrPkt;
    void*               csContext;
    uint64_t            currentTicks; 
    uint16_t            uniqueIndex;
    uint8_t             ueId;
    uint8_t             rbId;

    /* Get the pointer to the QCI block. */
    ptrQCI = Dpm_getQCI(qci);

    /* Take a snapshot of the current DSP time */
    currentTicks = _itoll(TSCH, TSCL);

    /* Critical Section Enter: */
    csContext = Dpm_osalEnterCriticalSection();

    /* Cycle through all the packet in the QCI List. */
    ptrListNode = Dpm_listGetHead (&ptrQCI->packetList);
    while (ptrListNode != NULL)
    {
        /* Get the packet node from the list node */
        ptrPacketNode = (Dpm_PacketNode*)((uint8_t*)ptrListNode - offsetof (Dpm_PacketNode, qciPktLinks));

        /* Do we need to discard the packet? */
        if ((currentTicks - ptrPacketNode->timeStamp) > timeout)
        {
            /* YES. Remove the packet from the QCI list. */
            Dpm_listRemoveElement(&ptrQCI->packetList, (Dpm_ListNode *)&ptrPacketNode->qciPktLinks);

            /* Remove the packet from the RB list also. */
            Dpm_listRemoveElement(&ptrPacketNode->ptrRB->packetList, (Dpm_ListNode *)&ptrPacketNode->rbPktLinks);

            /* The packet has to be popped off the RB hardware queue too. */
            ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptrPacketNode->ptrRB->rbQueueHnd));

            /* Get the user profile information */
            ueId = ptrPacketNode->ptrRB->ueId;
            rbId = ptrPacketNode->ptrRB->rbId;

            /* Get the user profile index. */
            uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

            /* Is the user profile active i.e. are there are pending packets in the RB list or in 
             * the RLC? */
            if  ((Dpm_isListEmpty(&ptrPacketNode->ptrRB->packetList) == 1) &&
                 (gDPMMCB.rlcPendingBytes[uniqueIndex] == 0) &&
                 (gDPMMCB.rlcControlBytes[uniqueIndex] == 0) &&
                 (gDPMMCB.rlcRetransmissionBytes[uniqueIndex] == 0))
            {
                /* NO. The user profile is NOT active. */
                gDPMMCB.listRBStatus[uniqueIndex] &= ~DPM_USER_PROFILE_ACTIVE_BIT_FLAG;

                /* Clear the cumulative and fresh reports. */
                Dpm_reportClear(gDPMMCB.cumulativeRawReportHandle, ueId, rbId);
                Dpm_reportClear(gDPMMCB.freshRawReportHandle, ueId, rbId);

                /* The user & radio bearer is now moving to the INACTIVE stage */
                Dpm_appStatusNotify (ueId, rbId, Dpm_UserProfileStatus_INACTIVE);
            }

            /* Cleanup the packet. */
            Pktlib_freePacket(gDPMMCB.config.pktlibInstHandle, ptrPkt);

            /* Get the new head of the QCI list. */
            ptrListNode = Dpm_listGetHead (&ptrQCI->packetList);
        }
        else
        {
            /* If the heap of the packet in the QCI is NOT old enough; we dont need to cycle 
             * through the rest of the packets since the list is always maintained with the oldest
             * packet at the head of the list. */
            break;
        }
    }

    /* Critical Section Exit: */
    Dpm_osalExitCriticalSection (csContext);

    /* We are done with the */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked to merge the RLC report information 
 *      to get an updated report.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Dpm_updateReportWithRLC(void)
{
    uint8_t         ueId;
    uint8_t         rbId;
    uint16_t        uniqueIndex;
    uint8_t         nextUeId;
    uint8_t         nextRbId;    
    Dpm_RB*         ptrRB;
    Dpm_QCI*        ptrQCI;
    Ti_Pkt*			ptrPkt;
    void*           csContext;
    uint32_t        rlcProcessedPktCount;
    uint32_t        count;
    Dpm_PacketNode* ptrPacketNode;

    /* Find the active user identifier & radio bearer from the RLC report. */
    if (Dpm_reportFindActiveUeRb (gDPMMCB.rlcReportHandle, &ueId, &rbId) < 0)
    {
        /* RLC Reports were empty: There is no need to update the RAW reports. */ 
        return;
    }

    /* Cycle through all the active users & radio bearers in the RLC report. */
    while (1)
    {
        /* Get the unique index. */
        uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

        /* Get the pointer to the RB */
        ptrRB  = Dpm_getRB(ueId, rbId);

        /* Get the QCI associated with the RB */
        ptrQCI = Dpm_getQCI(gDPMMCB.qci[uniqueIndex]);

        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();

#if 1
        /* Get the number of packets in the RLC Process Queue */
        rlcProcessedPktCount = Qmss_getQueueEntryCount (ptrRB->rlcProcessedQueue);

        /* Cycle through all the RLC processed packets: Packets are processed in the FIFO order so
         * for the specified radio bearer we can remove the packets from the head of the list. */
        for (count = 0; count < rlcProcessedPktCount; count++)
        {
        	Dpm_PacketNode* ptrRLCPacketListNode;

            /* Remove the packet from the RLC Processed Queue also. */
            ptrPkt = Qmss_queuePop (ptrRB->rlcProcessedQueue);
            if (ptrPkt == NULL)
            {
            	System_printf ("Error: RLC Processed Queue is EMPTY (RLC Count %d %d:%d)\n",
                                rlcProcessedPktCount, ueId, rbId);
            	break;
            }

            /* Get the packet node */
            ptrPacketNode = Dpm_getPacketNode(ptrPkt);

            /* Dequeue the packet from the RB list also. */
            ptrRLCPacketListNode = (Dpm_PacketNode*)Dpm_listDequeue(&ptrRB->packetList);

            /* Sanity Check: The packet removed cannot be NULL. */
            if (ptrPacketNode == NULL)
            {
                System_printf ("Error: FATAL Empty Packet Node (RLC Count %d %d:%d)\n", 
                                rlcProcessedPktCount, ueId, rbId);
                break;
            }

            /* Sanity Check: Make sure that the software & hardware queues are synchronized. */
            if (ptrRLCPacketListNode != ptrPacketNode)
            {
            	System_printf ("Error: DPM Corruption S/W Node 0x%x H/W Node 0x%x\n", ptrRLCPacketListNode, ptrPacketNode);
            	break;
            }

            /* Dequeue the packet from the QCI packet list also. */
            Dpm_listRemoveElement(&ptrQCI->packetList, (Dpm_ListNode *)&ptrPacketNode->qciPktLinks);

            /* Cleanup the RLC Processed packet. */
            Pktlib_freePacket (gDPMMCB.config.pktlibInstHandle, ptrPkt);
        }
#else
        /* Compute the number of packets which have been processed by the RLC
         *  - All packets when received are placed into the RB packet list & the hardware queue
         *  - All packets when processed by the RLC are removed from the hardware queue */
        rlcProcessedPktCount = Dpm_listGetCount(&ptrRB->packetList) - Qmss_getQueueEntryCount (ptrRB->rbQueueHnd);

        /* Cycle through all the RLC processed packets: Packets are processed in the FIFO order so
         * for the specified radio bearer we can remove the packets from the head of the list. */
        for (count = 0; count < rlcProcessedPktCount; count++)
        {
            /* Dequeue the packet from the RB packet list. */
            ptrPacketNode = (Dpm_PacketNode*)Dpm_listDequeue(&ptrRB->packetList);

            /* Sanity Check: The packet removed cannot be NULL. */
            if (ptrPacketNode == NULL)
            {
                System_printf ("Error: FATAL Empty Packet Node (RLC Count %d %d:%d)\n",
                                rlcProcessedPktCount, ueId, rbId);
                return;
            }

            /* Dequeue the packet from the QCI packet list also. */
            Dpm_listRemoveElement(&ptrQCI->packetList, (Dpm_ListNode *)&ptrPacketNode->qciPktLinks);

            /* Remove the packet from the RLC Processed Queue also. */
            ptrPkt = Qmss_queuePop (ptrRB->rlcProcessedQueue);
            if (ptrPkt == NULL)
            {
            	System_printf ("Error: RLC Processed Queue is EMPTY (RLC Count %d %d:%d)\n",
                                rlcProcessedPktCount, ueId, rbId);
            	return;
            }

            /* Cleanup the RLC Processed packet. */
            Pktlib_freePacket (gDPMMCB.config.pktlibInstHandle, ptrPkt);
        }
#endif

        /****************************************************************************
         * Update the fields in the DPM Lists: 
         *  - Packet Time Stamp List
         *  - Cumulative Rx Byte Count
         ****************************************************************************/
        ptrPacketNode = (Dpm_PacketNode*)Dpm_listGetHead(&ptrRB->packetList);
        if (ptrPacketNode != NULL)
            gDPMMCB.packetTimestamp[uniqueIndex] = ptrPacketNode->timeStamp;
        else
            gDPMMCB.packetTimestamp[uniqueIndex] = 0;

        /* Cumulative Receive Byte is updated with the number of bytes in the RB Queue. */
        gDPMMCB.rxByteCount[uniqueIndex] = Qmss_getQueueByteCount(ptrRB->rbQueueHnd);

        /* Is this user & radio bearer active? */ 
        if ((gDPMMCB.rxByteCount[uniqueIndex] == 0)     && (gDPMMCB.rlcPendingBytes[uniqueIndex] == 0) &&
            (gDPMMCB.rlcControlBytes[uniqueIndex] == 0) && (gDPMMCB.rlcRetransmissionBytes[uniqueIndex] == 0))
        {
            /* RB is NOT active: Since there is no fresh data which has been received on the RB & 
             * there are no pending RLC bytes to be scheduled. So we mark the RB as INACTIVE in the
             * RLC reports. */
            Dpm_reportClear(gDPMMCB.cumulativeRawReportHandle, ueId, rbId);
            Dpm_reportClear(gDPMMCB.freshRawReportHandle, ueId, rbId);

            /* The user & radio bearer is now moving to the INACTIVE stage */
            Dpm_appStatusNotify (ueId, rbId, Dpm_UserProfileStatus_INACTIVE);

            /* Make sure we mark it inactive in our list also. */
            gDPMMCB.listRBStatus[uniqueIndex] &= ~DPM_USER_PROFILE_ACTIVE_BIT_FLAG;

            /* Reset the packet delay budget also since the RB is now inactive. */
            gDPMMCB.packetDelayBudget[uniqueIndex] = 0;
        }
        else
        {
            /* User profile has pending data and is ACTIVE but set the flag in the cumulative 
             * report only if the user profile was enabled. */
            if (gDPMMCB.listRBStatus[uniqueIndex] & DPM_USER_PROFILE_ENABLE_BIT_FLAG)
                Dpm_reportSet(gDPMMCB.cumulativeRawReportHandle, ueId, rbId);
        }

        /* Did we detect any fresh "pending data" in the RLC report? If so update the fresh 
         * RAW report for the specific user & radio bearer but do this only if the user profile
         * was enabled? */
        if ((gDPMMCB.listRBStatus[uniqueIndex] & DPM_USER_PROFILE_ENABLE_BIT_FLAG) && 
            (gDPMMCB.rlcPendingBytes[uniqueIndex]))
            Dpm_reportSet(gDPMMCB.freshRawReportHandle, ueId, rbId);

        /* Clear the RLC report for the specific user & radio bearer since this has been processed. */
        Dpm_reportClear(gDPMMCB.rlcReportHandle, ueId, rbId);

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);

        /* We are done with this RLC report; so now jump to the next active RLC report */
        if (Dpm_reportFindNextActiveUeRb (gDPMMCB.rlcReportHandle, ueId, rbId, &nextUeId, &nextRbId) < 0)
        {
            /* No more active UE in the RLC report. */
            break;
        }
        else
        {
            /* Active user & radio bearer in the RLC report. */
            ueId = nextUeId;
            rbId = nextRbId;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to update the packet delay budget for all users
 *      and their radio bearers. 
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Dpm_updatePacketDelayBudget (void)
{
    uint16_t            index;
    uint64_t            currentTicks;
    uint32_t            deltaTicks;
    uint32_t            qci;
    int32_t*  restrict  ptrPacketDelayBudgetList;
    uint32_t* restrict  ptrQCIList;
    uint64_t* restrict  ptrPacketTimestampList;
    uint32_t* restrict  ptrRBStatusList;

    /* Take a snapshot of the current DSP time */
    currentTicks = _itoll(TSCH, TSCL);

    /* Get the pointers */
    ptrPacketDelayBudgetList = &gDPMMCB.packetDelayBudget[0];
    ptrQCIList               = &gDPMMCB.qci[0];
    ptrPacketTimestampList   = &gDPMMCB.packetTimestamp[0];
    ptrRBStatusList          = &gDPMMCB.listRBStatus[0];

    /* DSP Optimization: */
    _nassert((int)ptrPacketDelayBudgetList % 8 == 0);
    _nassert((int)ptrQCIList % 8 == 0);
    _nassert((int)ptrPacketTimestampList % 8 == 0);
    _nassert((int)ptrRBStatusList % 8 == 0);

    /* Cycle through all the users & radio bearers. */
    #pragma MUST_ITERATE(DPM_MAX_UE*DPM_MAX_RB,,DPM_MAX_UE*DPM_MAX_RB)
    for (index = 0; index < DPM_MAX_UE*DPM_MAX_RB; index++)
    {
        /* Get the QCI associated with the user & radio bearer. */
        qci = ptrQCIList[index];

        /* Compute the delta time since the packet has been sitting in the RB */
        deltaTicks = currentTicks - *ptrPacketTimestampList;

        /* Is the user profile active? If so then update the PDB. */
        if (*ptrRBStatusList & DPM_USER_PROFILE_ACTIVE_BIT_FLAG)
            *ptrPacketDelayBudgetList = _ssub(gDPMMCB.qciConfiguredDelayBudget[qci], deltaTicks);

        /* Get the next elements. */
        ptrPacketTimestampList++;
        ptrPacketDelayBudgetList++;
        ptrRBStatusList++;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the report using the profile information.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ptrUserProfile
 *      Pointer to the user profile.
 *  @param[in]  ptrReport 
 *      Pointer to the report which is populated by the API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Dpm_getReport (Dpm_UserProfile* ptrUserProfile, Dpm_Report* ptrReport)
{
    uint16_t    uniqueIndex;
    Dpm_RB*		ptrRB;

    /* Get the unique index for the specific user & radio bearer */
    uniqueIndex = Dpm_getUniqueIndex(ptrUserProfile->ueId, ptrUserProfile->rbId);

    /* Get the RB Information */
    ptrRB = Dpm_getRB(ptrUserProfile->ueId, ptrUserProfile->rbId);

    /* Populate the report information block. */
    ptrReport->ueId                   = ptrUserProfile->ueId;
    ptrReport->rbId                   = ptrUserProfile->rbId;
    ptrReport->qci                    = gDPMMCB.qci[uniqueIndex];
    ptrReport->rxByteCount            = gDPMMCB.rxByteCount[uniqueIndex];
    ptrReport->rlcPendingBytes        = gDPMMCB.rlcPendingBytes[uniqueIndex];
    ptrReport->rlcControlBytes        = gDPMMCB.rlcControlBytes[uniqueIndex];
    ptrReport->rlcRetransmissionBytes = gDPMMCB.rlcRetransmissionBytes[uniqueIndex];
    ptrReport->pdb                    = gDPMMCB.packetDelayBudget[uniqueIndex];
    ptrReport->freshRxByteCount       = gDPMMCB.freshRxByteCount[uniqueIndex];
    ptrReport->pendingPktCount        = Qmss_getQueueEntryCount (ptrRB->rbQueueHnd);

    /* Once the report has been received reset the fresh counters */
    gDPMMCB.freshRxByteCount[uniqueIndex]         = 0;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the scheduler to get the head user profile
 *      of the specific report type.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @sa
 *      Dpm_getNextUserProfile
 *
 *  @param[in]  type
 *      Type of report for which the user profile is required.
 *  @param[out]  ptrUserProfile
 *      Pointer to the user profile populated
 *
 *  @retval
 *      Empty Report    - 0
 *  @retval
 *      Valid Report    - 1
 */
int32_t Dpm_getHeadUserProfile (Dpm_ReportType type, Dpm_UserProfile* ptrUserProfile)
{
    uint16_t            uniqueIndex;
    Dpm_ReportHandle    reportHandle;
    Dpm_UserProfile     nextUserProfile;

    /* Update Raw report using the RLC report. */
    Dpm_updateReportWithRLC();

    /* Update the packet delay budget */
    Dpm_updatePacketDelayBudget();

    /* Determine the report handle to be used. */
    if (type == Dpm_ReportType_CUMULATIVE)
        reportHandle = gDPMMCB.cumulativeRawReportHandle;
    else
        reportHandle = gDPMMCB.freshRawReportHandle;

    /* Find the initial user identifier & radio bearer which is active. */
    if (Dpm_reportFindActiveUeRb (reportHandle, &ptrUserProfile->ueId, &ptrUserProfile->rbId) < 0)
    {
        /* No Active UE was detected. This is an empty report */
        return 0;
    }

    /* Determine if the user profile is enabled or not. If the head user profile is NOT enabled
     * we will need to continue the search for the first enabled user profile. */
    while (1)
    {
        /* Get the unique index associated with the user profile. */
        uniqueIndex = Dpm_getUniqueIndex(ptrUserProfile->ueId, ptrUserProfile->rbId);

        /* Is the user profile enabled? */
        if (gDPMMCB.listRBStatus[uniqueIndex] & DPM_USER_PROFILE_ENABLE_BIT_FLAG)
        {
            /* YES. Fresh reports live a short life. Once they have been passed to the scheduler 
             * their pending status is cleared up. */
            if (reportHandle == gDPMMCB.freshRawReportHandle)
                Dpm_reportClear (reportHandle, ptrUserProfile->ueId, ptrUserProfile->rbId);
           
            /* Valid report has been generated. */
            return 1; 
        }
        else
        {
            /* NO. User Profile was not enabled; we need to get the next user profile. */
            if (Dpm_getNextUserProfile(type, ptrUserProfile, &nextUserProfile) == 0)
            {
                /* There was no enabled user profile left. So this will be an EMPTY report. */
                return 0;
            }

            /* Initialize the user profile. */
            ptrUserProfile->ueId = nextUserProfile.ueId;
            ptrUserProfile->rbId = nextUserProfile.rbId;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is invoked by the scheduler to get the next element in 
 *      the RAW report list
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  type
 *      Type of report for which the user profile is required. 
 *  @param[in]  ptrUserProfile
 *      Current user profile
 *  @param[out]  ptrNextUserProfile
 *      Pointer to the next user profile
 *
 *  @retval
 *      Empty Report    - 0
 *  @retval
 *      Valid Report    - 1
 */
int32_t Dpm_getNextUserProfile
(
    Dpm_ReportType      type, 
    Dpm_UserProfile*    ptrUserProfile, 
    Dpm_UserProfile*    ptrNextUserProfile
)
{
    int32_t             retVal;
    Dpm_ReportHandle    reportHandle;
    uint16_t            uniqueIndex;

    /* Determine the report handle to be used. */
    if (type == Dpm_ReportType_CUMULATIVE)
        reportHandle = gDPMMCB.cumulativeRawReportHandle;
    else
        reportHandle = gDPMMCB.freshRawReportHandle;

    while (1)
    {
        /* Given the current report element; find the next raw report */
        retVal = Dpm_reportFindNextActiveUeRb (reportHandle, ptrUserProfile->ueId, 
                                               ptrUserProfile->rbId, &ptrNextUserProfile->ueId, 
                                               &ptrNextUserProfile->rbId);
        if (retVal < 0)
        {
            /* There were no more active users or radio bearers. This is an empty report */
            return 0;
        }

        /* Get the unique index associated with the user profile. */
        uniqueIndex = Dpm_getUniqueIndex(ptrNextUserProfile->ueId, ptrNextUserProfile->rbId);

        /* Is the user profile enabled? */
        if (gDPMMCB.listRBStatus[uniqueIndex] & DPM_USER_PROFILE_ENABLE_BIT_FLAG)
        {
            /* YES. Fresh reports live a short life. Once they have been passed to the scheduler 
             * their pending status is cleared up. */
            if (reportHandle == gDPMMCB.freshRawReportHandle)
                Dpm_reportClear (reportHandle, ptrNextUserProfile->ueId, ptrNextUserProfile->rbId);

            /* Valid report is being returned. */
            return 1;
        }
        else
        {
            /* NO. The user profile was disabled. Try and get the next user profile. */
            ptrUserProfile->ueId = ptrNextUserProfile->ueId;
            ptrUserProfile->rbId = ptrNextUserProfile->rbId;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is called by the scheduler to get the sorted user
 *      profiles on the basis of the bid calculated by the application
 *      specified bidding function. 
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  maxUserProfiles
 *      Maximum number of sorted user profiles which is needed by the
 *      scheduler
 *  @param[out]  sortedUserProfile
 *      Sorted user profiles. This should have at least 'maxUserProfiles' 
 *      elements.
 *  @param[out]  numUserProfiles
 *      Number of user profiles populated by the API
 *  @param[out]  errCode
 *      Error code only populated if the function returned error.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_getSortedUserProfiles
(
    uint32_t            maxUserProfiles,
    Dpm_UserProfile*    sortedUserProfile,
    uint32_t*           numUserProfiles,
    int32_t*            errCode
)
{
    uint32_t            index;
    uint32_t            kPrime;
    int32_t*  restrict  ptrPacketDelayBudgetList;
    uint32_t* restrict  ptrRLCPendingByteList;
    uint32_t* restrict  ptrRLCControlByteList;
    uint32_t* restrict  ptrRLCRetransmissionByteList;
    uint32_t* restrict  ptrRxByteCountList;
    uint32_t* restrict  ptrBidValueList;
    uint32_t* restrict  ptrBiddingIndexList;
    uint32_t* restrict  ptrRBStatusList;
    int32_t*  restrict  ptrQCIConfigDelayBudget;
    uint32_t* restrict  ptrQCIList;
    uint64_t* restrict  ptrPacketTimestampList;
    uint32_t* restrict  ptrUserProfileList;
    int32_t             numActiveUserProfiles = 0;
    uint32_t            paddingElements;
    uint32_t*           ptrSelectedIndex;
    uint32_t            formattedIndex;
    uint64_t            currentTicks;
    uint32_t            deltaTicks;
    uint32_t            qci;

    /* Initialize the benchmarking variables. */
    DPM_BENCHMARK_INIT (benchmarkStart);
    DPM_BENCHMARK_INIT (benchmarkEnd);

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Update Raw report using the RLC report. */
    Dpm_updateReportWithRLC();

    /* Benchmarking End: */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(RLC Updates, benchmarkStart, benchmarkEnd);

    /* Initialize the pointers to the various lists. */
    ptrPacketDelayBudgetList     = &gDPMMCB.packetDelayBudget[0];
    ptrRLCPendingByteList        = &gDPMMCB.rlcPendingBytes[0];
    ptrRLCControlByteList        = &gDPMMCB.rlcControlBytes[0];
    ptrRLCRetransmissionByteList = &gDPMMCB.rlcRetransmissionBytes[0];
    ptrRxByteCountList           = &gDPMMCB.rxByteCount[0];
    ptrQCIList                   = &gDPMMCB.qci[0];
    ptrBidValueList              = &gDPMMCB.bidValues[0];
    ptrBiddingIndexList          = &gDPMMCB.biddingIndex[0];
    ptrRBStatusList              = &gDPMMCB.listRBStatus[0];
    ptrPacketTimestampList       = &gDPMMCB.packetTimestamp[0];
    ptrQCIConfigDelayBudget      = &gDPMMCB.qciConfiguredDelayBudget[0];
    ptrUserProfileList           = &gDPMMCB.userProfileList[0];

    /* DSP Optimization: */
    _nassert((int)ptrQCIConfigDelayBudget % 8 == 0);
    _nassert((int)ptrPacketTimestampList % 8 == 0);
    _nassert((int)ptrPacketDelayBudgetList % 8 == 0);
    _nassert((int)ptrRLCPendingByteList % 8 == 0);
    _nassert((int)ptrRLCControlByteList % 8 == 0);
    _nassert((int)ptrRLCRetransmissionByteList % 8 == 0);
    _nassert((int)ptrRxByteCountList % 8 == 0);
    _nassert((int)ptrQCIList % 8 == 0);
    _nassert((int)ptrBidValueList % 8 == 0);
    _nassert((int)ptrBiddingIndexList % 8 == 0);
    _nassert((int)ptrRBStatusList % 8 == 0);

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Take a snapshot of the current DSP time */
    currentTicks = _itoll(TSCH, TSCL);

    /* Cycle through all the users & radio bearers. */
    #pragma MUST_ITERATE(DPM_MAX_UE*DPM_MAX_RB,,DPM_MAX_UE*DPM_MAX_RB)
    for (index = 0; index < DPM_MAX_UE*DPM_MAX_RB; index++)
    {
        /* Get the QCI value associated with the radio bearer. */
        qci = *ptrQCIList++;

        /* Compute the delta time since the packet has been sitting in the RB */
        deltaTicks = currentTicks - *ptrPacketTimestampList++;

        /* Is the user profile active & enabled? */
        if (*ptrRBStatusList & (DPM_USER_PROFILE_ACTIVE_BIT_FLAG | DPM_USER_PROFILE_ENABLE_BIT_FLAG) == 
            (DPM_USER_PROFILE_ACTIVE_BIT_FLAG | DPM_USER_PROFILE_ENABLE_BIT_FLAG))
        {
            /* YES. Update the packet delay budget. */
            *ptrPacketDelayBudgetList = _ssub(*(ptrQCIConfigDelayBudget + qci), deltaTicks);

            /* Initialize the bidding index. */
            *ptrBiddingIndexList++ = numActiveUserProfiles;

            /* Store the user profile list which will be used to get the actual user profile
             * once the sorting is done. */
            *ptrUserProfileList++ = index;

            /* Compute the bid value */
            *ptrBidValueList++ = Dpm_appBidFunction (index/DPM_MAX_RB, index%DPM_MAX_RB, qci, 
                                                     *ptrRxByteCountList, *ptrRLCPendingByteList,
                                                     *ptrRLCControlByteList, *ptrRLCRetransmissionByteList,
                                                     /**ptrPacketDelayBudgetList*/deltaTicks);

            /* Increment the number of active radio bearers detected */
            numActiveUserProfiles++;
        }

        /* Increment the pointers to the various lists */
        ptrPacketDelayBudgetList++;
        ptrRxByteCountList++;
        ptrRLCPendingByteList++;
        ptrRLCControlByteList++;
        ptrRLCRetransmissionByteList++;
        ptrRBStatusList++;
    }

    /* Keep track of the number of active user profiles which have been detected till now. */
    gDPMMCB.numPendingActiveUserProfiles = numActiveUserProfiles;

    /* Benchmarking End: */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Bid Value Took, benchmarkStart, benchmarkEnd);

    /* Proceed only if there are active user profiles. */
    if (numActiveUserProfiles == 0)
    {
        /* No active user profiles detected; there is no need to select or sort 
         * the results. */
        *numUserProfiles = 0;
        return 0;
    }

    /* Detect the number of active user profiles which have been detected. */
    if (numActiveUserProfiles > maxUserProfiles)
    {
        /* We have more RAW reports than what the scheduler expects; so we need to select
         * the best of the lot here. */
        *numUserProfiles = maxUserProfiles;

        /* Compute the number of padding elements which are required (if any) 
         * The selection algorithm requires the number of raw reports to be a 
         * multiple of 16. If this is NOT the case we need to pad up */        
        if ((numActiveUserProfiles % 16) != 0)
        {
            /* Determine the number of padding elements which are required. */
            paddingElements = (((numActiveUserProfiles / 16) + 1) * 16) - numActiveUserProfiles; 
            while (paddingElements != 0)
            {
                /* These are the DUMMY Padding elements which are being created
                 * for the selection to work. These bids are set to 0 to ensure
                 * that they never get selected. The sorted reports for these
                 * padded elements are set to NULL */
                *ptrBidValueList++          = 0;
                *ptrBiddingIndexList++      = numActiveUserProfiles;

                /* Increment the number of raw reports detected */
                numActiveUserProfiles = numActiveUserProfiles + 1;

                /* Decrement the padding elements */
                paddingElements--;
            }
        }

        /* All the bids have been computed; we now get the best out of the lot. */
        kPrime = selectindex_bit32(numActiveUserProfiles, maxUserProfiles, &gDPMMCB.bidValues[0], &gDPMMCB.biddingIndex[0]);
        if (kPrime == 0)
        {
            /* Error: This is a FATAL Error and indicates that the selection failed. */
            *errCode = DPM_EINTERNAL;
            return -1;
        }

        /* The selected index points to the index of the "best bid" selection  */
        ptrSelectedIndex = &out_index[0];
    }
    else
    {
        /* We have less RAW reports than what the scheduler expects; there is no need to
         * perform a selection here. */
        *numUserProfiles = numActiveUserProfiles;

        /* All the reports need to be sorted. */
        kPrime = numActiveUserProfiles;

        /* The selected index points to the bidding index. */
        ptrSelectedIndex = &gDPMMCB.biddingIndex[0];
    }

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Sort the top bid values. */
    for (numActiveUserProfiles = 0; numActiveUserProfiles < kPrime; numActiveUserProfiles++)
    {
        /* Get the formatted index. */
        formattedIndex = *ptrSelectedIndex++;

        /* Populate the sorting "bid" values & indexes which need to be sorted. */
        gDPMMCB.inputBidsToSort[numActiveUserProfiles]     = gDPMMCB.bidValues[formattedIndex];
        gDPMMCB.inputBidIndexToSort[numActiveUserProfiles] = formattedIndex;
    }

    /* Benchmarking Stop: */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Sorting Input creation, benchmarkStart, benchmarkEnd);

    /* Benchmarking Start: Sorting */
    DPM_BENCHMARK(benchmarkStart);

    /* Sort the bid values. */
    SortIndex_merge32(&gDPMMCB.inputBidsToSort[0], kPrime, &gDPMMCB.inputBidIndexToSort[0], 
                      &gDPMMCB.sortedOutputBids[0], &gDPMMCB.sortedOutputBidsIndex[0], 
                      &gDPMMCB.sortingScratchMemory[0]);

    /* Benchmarking Stop: Sorting */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Sorting, benchmarkStart, benchmarkEnd);

    /* It is possible that kPrime will return more than required. The list is sorted in ascending
     * order of bids; so here we need to discount the bottom "x" results. */
    if (kPrime > *numUserProfiles)
        kPrime = (kPrime - *numUserProfiles);
    else
        kPrime = 0;

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Prepare the sorted user profile. The sorting is done in ascending order of bid values 
     * so here we reverse the order while preparing the final output. Also account for the kPrime
     * in the final sorted results because it would offset the results. */
    index = 0;
    numActiveUserProfiles = (*numUserProfiles + kPrime) - 1;
    while (index != *numUserProfiles)
    {
        /* Get the formatted index. */
        formattedIndex = gDPMMCB.sortedOutputBidsIndex[numActiveUserProfiles];

        /* Reset the BID Value to be 0 as this user profile has already been accounted for. */
        gDPMMCB.bidValues[formattedIndex] = 0;

        /* Get the user profile at this formatted index */
        formattedIndex = gDPMMCB.userProfileList[formattedIndex];

        /* Populate the formatted user profile. */
        sortedUserProfile[index].ueId = formattedIndex / DPM_MAX_RB;
        sortedUserProfile[index].rbId = formattedIndex % DPM_MAX_RB;

        /* Populate the next entry. */
        index++;
        numActiveUserProfiles--;
    }

    /* Keep track of the number of the active pending user profiles left behind after 
     * we have returned the first set. */
    gDPMMCB.numPendingActiveUserProfiles = gDPMMCB.numPendingActiveUserProfiles - *numUserProfiles;

    /* Benchmarking Stop: Output Preperation */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Output Preperation, benchmarkStart, benchmarkEnd);

    /* Sorted report is complete. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called by the scheduler to get the next sorted
 *      set of user profiles. The function should be called *only* after 
 *      the 'Dpm_getSortedUserProfiles' has been invoked. The function
 *      will not recompute the bid values but it will simply sort and
 *      return the next best set of valid bids.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  maxUserProfiles
 *      Maximum number of sorted user profiles which is needed by the
 *      scheduler
 *  @param[out]  sortedUserProfile
 *      Sorted user profiles. This should have at least 'maxUserProfiles' 
 *      elements.
 *  @param[out]  numUserProfiles
 *      Number of user profiles populated by the API
 *  @param[out]  errCode
 *      Error code only populated if the function returned error.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_getNextSortedUserProfiles
(
    uint32_t            maxUserProfiles,
    Dpm_UserProfile*    sortedUserProfile,
    uint32_t*           numUserProfiles,
    int32_t*            errCode
)
{
    uint32_t            index;
    uint32_t            kPrime;
    int32_t             numActiveUserProfiles = 0;
    uint32_t            paddingElements;
    uint32_t*           ptrSelectedIndex;
    uint32_t            formattedIndex;

    /* Initialize the benchmarking variables. */
    DPM_BENCHMARK_INIT (benchmarkStart);
    DPM_BENCHMARK_INIT (benchmarkEnd);

    /* Get the number of pending active user profiles. */
    numActiveUserProfiles = gDPMMCB.numPendingActiveUserProfiles;

    /* Proceed only if there are active user profiles. */
    if (numActiveUserProfiles == 0)
    {
        /* No active user profiles detected; there is no need to select or sort 
         * the results. */
        *numUserProfiles = 0;
        return 0;
    }

    /* Detect the number of active user profiles which have been detected. */
    if (numActiveUserProfiles > maxUserProfiles)
    {
        /* We have more user profiles than what the scehduler has requested. */
        *numUserProfiles = maxUserProfiles;

        /* Compute the number of padding elements which are required (if any) 
         * The selection algorithm requires the number of raw reports to be a 
         * multiple of 16. If this is NOT the case we need to pad up */        
        if ((numActiveUserProfiles % 16) != 0)
        {
            /* Determine the number of padding elements which are required. */
            paddingElements = (((numActiveUserProfiles / 16) + 1) * 16) - numActiveUserProfiles; 
            while (paddingElements != 0)
            {
                /* These are the DUMMY Padding elements which are being created
                 * for the selection to work. These bids are set to 0 to ensure
                 * that they never get selected. The sorted reports for these
                 * padded elements are set to NULL */
                gDPMMCB.bidValues[numActiveUserProfiles]    = 0;
                gDPMMCB.biddingIndex[numActiveUserProfiles] = numActiveUserProfiles;

                /* Increment the number of raw reports detected */
                numActiveUserProfiles = numActiveUserProfiles + 1;

                /* Decrement the padding elements */
                paddingElements--;
            }
        }

        /* All the bids have been computed; we now get the best out of the lot. */
        kPrime = selectindex_bit32(numActiveUserProfiles, maxUserProfiles, &gDPMMCB.bidValues[0], &gDPMMCB.biddingIndex[0]);
        if (kPrime == 0)
        {
            /* Error: This is a FATAL Error and indicates that the selection failed. */
            *errCode = DPM_EINTERNAL;
            return -1;
        }

        /* The selected index points to the index of the "best bid" selection  */
        ptrSelectedIndex = &out_index[0];
    }
    else
    {
        /* We have less RAW reports than what the scheduler expects; there is no need to
         * perform a selection here. */
        *numUserProfiles = numActiveUserProfiles;

        /* All the reports need to be sorted. */
        kPrime = numActiveUserProfiles;

        /* The selected index points to the bidding index. */
        ptrSelectedIndex = &gDPMMCB.biddingIndex[0];
    }

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Sort the top bid values. */
    for (numActiveUserProfiles = 0; numActiveUserProfiles < kPrime; numActiveUserProfiles++)
    {
        /* Get the formatted index. */
        formattedIndex = *ptrSelectedIndex++;

        /* Populate the sorting "bid" values & indexes which need to be sorted. */
        gDPMMCB.inputBidsToSort[numActiveUserProfiles]     = gDPMMCB.bidValues[formattedIndex];
        gDPMMCB.inputBidIndexToSort[numActiveUserProfiles] = formattedIndex;
    }

    /* Benchmarking Stop: */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Sorting Input creation, benchmarkStart, benchmarkEnd);

    /* Benchmarking Start: Sorting */
    DPM_BENCHMARK(benchmarkStart);

    /* Sort the bid values. */
    SortIndex_merge32(&gDPMMCB.inputBidsToSort[0], kPrime, &gDPMMCB.inputBidIndexToSort[0], 
                      &gDPMMCB.sortedOutputBids[0], &gDPMMCB.sortedOutputBidsIndex[0], 
                      &gDPMMCB.sortingScratchMemory[0]);

    /* Benchmarking Stop: Sorting */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Sorting, benchmarkStart, benchmarkEnd);

    /* It is possible that kPrime will return more than required. The list is sorted in ascending
     * order of bids; so here we need to discount the bottom "x" results. */
    if (kPrime > *numUserProfiles)
        kPrime = (kPrime - *numUserProfiles);
    else
        kPrime = 0;

    /* Benchmarking Start: */
    DPM_BENCHMARK(benchmarkStart);

    /* Prepare the sorted user profile. The sorting is done in ascending order of bid values 
     * so here we reverse the order while preparing the final output. Also account for the kPrime
     * in the final sorted results because it would offset the results. */
    index = 0;
    numActiveUserProfiles = (*numUserProfiles + kPrime) - 1;
    while (index != *numUserProfiles)
    {
        /* Get the formatted index. */
        formattedIndex = gDPMMCB.sortedOutputBidsIndex[numActiveUserProfiles];

        /* Reset the BID Value to be 0 as this user profile has already been accounted for. */
        gDPMMCB.bidValues[formattedIndex] = 0;

        /* Get the user profile at this formatted index */
        formattedIndex = gDPMMCB.userProfileList[formattedIndex];

        /* Populate the formatted user profile. */
        sortedUserProfile[index].ueId = formattedIndex / DPM_MAX_RB;
        sortedUserProfile[index].rbId = formattedIndex % DPM_MAX_RB;

        /* Populate the next entry. */
        index++;
        numActiveUserProfiles--;
    }

    /* Keep track of the number of the active pending user profiles left behind after we 
     * have returned the current set. */
    gDPMMCB.numPendingActiveUserProfiles = gDPMMCB.numPendingActiveUserProfiles - *numUserProfiles;

    /* Benchmarking Stop: Output Preperation */
    DPM_BENCHMARK(benchmarkEnd);
    DPM_BENCHMARK_LOG(Output Preperation, benchmarkStart, benchmarkEnd);

    /* Sorted report is complete. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the specific user profile
 *
 *  \ingroup DPM_FUNCTION      
 *
 *  @param[in]  ptrUserProfile
 *      User Profile which is to be configured
 *  @param[in]  flag
 *      Set to 1 to enable the user profile to be reported by 
 *      the DPM else set to 0 and the user profile will NOT be reported.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_configureUserProfile (Dpm_UserProfile* ptrUserProfile, uint8_t flag)
{
    uint16_t    uniqueIndex;

    /* Get the unique index for the specific user profile. */
    uniqueIndex = Dpm_getUniqueIndex(ptrUserProfile->ueId, ptrUserProfile->rbId);

    /* Set the internal flag appropriately. */
    if (flag == 1)
        gDPMMCB.listRBStatus[uniqueIndex] |= DPM_USER_PROFILE_ENABLE_BIT_FLAG;
    else
        gDPMMCB.listRBStatus[uniqueIndex] &= ~DPM_USER_PROFILE_ENABLE_BIT_FLAG;

    /* User profile has been configured. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Debug Function which is used to dump the contents of the radio bearer 
 *      associated with the user on the console. Please be aware that If the 
 *      packets are being displayed these are done with the critical section 
 *      locks held.
 *
 *  \ingroup DPM_FUNCTION      
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  dumpPacket
 *      Flag which indicates if the packets in the list also need 
 *      to be displayed or not.
 *
 *  @retval
 *      Number of packets in the specific user profile
 */
uint32_t Dpm_dumpRB (uint8_t ueId, uint8_t rbId, uint8_t dumpPacket)
{
    Dpm_RB*             ptrRB;
    Dpm_PacketNode*     ptrPacketNode;
    uint16_t            uniqueIndex;
    void*               csContext;
    uint32_t            packetCount;

    /* Get the radio bearer */
    ptrRB = Dpm_getRB (ueId, rbId);

    /* Get the unique index for user & radio bearer */
    uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

    /* Display the Radio Bearer Information */
    System_printf ("--------------------------------------------------------------------\n");
    System_printf ("%d:%d QCI:%d HwQueue:0x%x    \n", ueId, rbId, 
                    gDPMMCB.qci[uniqueIndex], ptrRB->rbQueueHnd);
    System_printf ("HwQueue-Packet Count:%d HwQueue Packet Byte: %d \n", 
                   Qmss_getQueueEntryCount(ptrRB->rbQueueHnd), Qmss_getQueueByteCount(ptrRB->rbQueueHnd));

    /* Do we need to dump the packet. */
    if (dumpPacket)
    {
        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();

        /* Cycle through all the packet in the radio bearer list. */
        ptrPacketNode = (Dpm_PacketNode*)Dpm_listGetHead (&ptrRB->packetList);
        while (ptrPacketNode != NULL)
        {
            /* Display the enqueued packet. */
            System_printf ("Count:%d Packet 0x%p Timestamp %ld\n", 
                           ++packetCount, ptrPacketNode->ptrPkt, ptrPacketNode->timeStamp);

            /* Get the next element in the radio bearer list. */
            ptrPacketNode = (Dpm_PacketNode*)Dpm_listGetNext (&ptrRB->packetList, (Dpm_ListNode*)ptrPacketNode);
        }

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);
    }
    else
    {
        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();
            
        /* Get the number of packets in the RB list. */
        packetCount = Dpm_listGetCount(&ptrRB->packetList);

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);        
    }
    System_printf ("--------------------------------------------------------------------\n");
    return packetCount;
}

/**
 *  @b Description
 *  @n
 *      Debug Function which is used to dump the contents of the QCI. Please
 *      be aware that the critical section locks are held while the contents 
 *      are dumped on the console
 *
 *  \ingroup DPM_FUNCTION      
 *
 *  @param[in]  qci
 *      QCI identifier
 *  @param[in]  dumpPacket
 *      Flag which indicates if the packets in the list also need to be 
 *      displayed or not.
 *
 *  @retval
 *      Number of packets in the QCI
 */
uint32_t Dpm_dumpQCI (uint8_t qci, uint8_t dumpPacket)
{
    Dpm_QCI*            ptrQCI;
    Dpm_ListNode*       ptrListNode;
    Dpm_PacketNode*     ptrPacketNode;
    void*               csContext;
    uint32_t            packetCount = 0;

    /* Get the pointer to the QCI block. */
    ptrQCI = Dpm_getQCI(qci);

    /* Display the QCI Information */
    System_printf ("--------------------------------------------------------------------\n");
    System_printf ("QCI: %d \n", qci);

    /* Do we need to dump the packet. */
    if (dumpPacket)
    {
        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();

        /* Cycle through all the packet in the QCI list. */
        ptrListNode = Dpm_listGetHead (&ptrQCI->packetList);
        while (ptrListNode != NULL)
        {
            /* Get the packet node from the list node */
            ptrPacketNode = (Dpm_PacketNode*)((uint8_t*)ptrListNode - offsetof (Dpm_PacketNode, qciPktLinks));

            /* Display the enqueued packet. */
            System_printf ("Count:%d %d:%d Packet 0x%p  Timestamp %ld\n", ++packetCount,
                            ptrPacketNode->ptrRB->ueId, ptrPacketNode->ptrRB->rbId,
                            ptrPacketNode->ptrPkt, ptrPacketNode->timeStamp);

            /* Get the next element from the QCI list. */
            ptrListNode = Dpm_listGetNext (&ptrQCI->packetList, ptrListNode);
        }

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);
    }
    else
    {
        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();

        /* Get the number of packets in the QCI list. */
        packetCount = Dpm_listGetCount(&ptrQCI->packetList);

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);
    }
    System_printf ("--------------------------------------------------------------------\n");
    return packetCount;
}

/**
 *  @b Description
 *  @n
 *      Internal Utility function which is used to allocate & initialize
 *      a block of memory.
 *
 *  \ingroup DPM_INTERNAL_FUNCTION
 *
 *  @param[in]  memorySize
 *      Size of the memory to be allocated
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
static uint8_t* Dpm_allocMemory (uint32_t memorySize, int32_t* errCode)
{
    uint8_t* ptrMemory;

    /* Allocate the memory block */
    ptrMemory = (void*)Dpm_osalMalloc (memorySize, 128);
    if (ptrMemory == NULL)
    {
        *errCode = DPM_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrMemory, 0, memorySize);
    return ptrMemory;

}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the DPM module.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ptrDPMConfig 
 *      Pointer to the DPM configuration.
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
int32_t Dpm_init (Dpm_Config* ptrDPMConfig, int32_t* errCode)
{
    uint16_t    ueId;
    Dpm_UE*     ptrUE;
    Dpm_RB*     ptrRB;
    uint8_t     rbId;
    uint8_t     qci;
    uint8_t     isAllocated;
    Dpm_QCI*    ptrQCI;
    uint32_t    numElements;

    /* Sanity Check: Validate the configuration */
    if (ptrDPMConfig == NULL)
    {
        *errCode = DPM_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate and ensure a valid PKTLIB instance handle has been passed */
    if (ptrDPMConfig->pktlibInstHandle == NULL)
    {
        *errCode = DPM_EINVAL;
        return -1;
    }

    /* Initialize the DPM MCB */
    memset ((void *)&gDPMMCB, 0, sizeof(Dpm_MCB));

    /* Copy the configuration. */
    memcpy ((void *)&gDPMMCB.config, (void*)ptrDPMConfig, sizeof(Dpm_Config));

    /* Setup the number of elements: We need to account for all users & radio bearers. */
    numElements = (DPM_MAX_UE * DPM_MAX_RB);

    /* Initialize the report module to handle the RAW reports. */
    gDPMMCB.cumulativeRawReportHandle = Dpm_reportInit (errCode);
    if (gDPMMCB.cumulativeRawReportHandle == NULL)
        return -1;

    /* Initialize the fresh raw report module to keep track of "fresh" packets 
     * which have been received but not yet been acknowledged by the scheduler. */
    gDPMMCB.freshRawReportHandle = Dpm_reportInit (errCode);
    if (gDPMMCB.freshRawReportHandle == NULL)
        return -1;

    /* Initialize the report module to handle RLC reports. */
    gDPMMCB.rlcReportHandle = Dpm_reportInit(errCode);
    if (gDPMMCB.rlcReportHandle == NULL)
        return -1;

    /************************************************************************************************
     * DPM Tracked entities which need to be stored for each user & radio bearer.
     ************************************************************************************************/

    /* QCI Configured Delay Budget: */
    gDPMMCB.qciConfiguredDelayBudget = (int32_t*)Dpm_allocMemory(sizeof(int32_t) * DPM_MAX_QCI, errCode);
    if (gDPMMCB.qciConfiguredDelayBudget == NULL)
        return -1;

    /* Populate the QCI configured delay budgets */
    for (qci = 0; qci < DPM_MAX_QCI; qci++)
        gDPMMCB.qciConfiguredDelayBudget[qci] = gDPMMCB.config.qciPacketDelayBudget[qci];

    /* Packet Delay Budget: */
    gDPMMCB.packetDelayBudget = (int32_t*)Dpm_allocMemory(sizeof(int32_t) * numElements, errCode);
    if (gDPMMCB.packetDelayBudget == NULL)
        return -1;

    /* Packet Delay Timestamp: */
    gDPMMCB.packetTimestamp = (uint64_t*)Dpm_allocMemory(sizeof(uint64_t) * numElements, errCode);
    if (gDPMMCB.packetTimestamp == NULL)
        return -1;

    /* RLC Pending Bytes: */
    gDPMMCB.rlcPendingBytes = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.rlcPendingBytes == NULL)
        return -1;

    /* RLC Control Bytes: */
    gDPMMCB.rlcControlBytes = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.rlcControlBytes == NULL)
        return -1;

    /* RLC Retransmission Bytes: */
    gDPMMCB.rlcRetransmissionBytes = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.rlcRetransmissionBytes == NULL)
        return -1;

    /* Cumulative Receive Byte Count: */
    gDPMMCB.rxByteCount = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.rxByteCount == NULL)
        return -1;

    /* Fresh Receive Byte Count: */
    gDPMMCB.freshRxByteCount = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.freshRxByteCount == NULL)
        return -1;

    /* QCI: */
    gDPMMCB.qci = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.qci == NULL)
        return -1;

    /* RB Status List: */
    gDPMMCB.listRBStatus = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
    if (gDPMMCB.listRBStatus == NULL)
        return -1;

    /************************************************************************************************
     * DPM Sorted Reports: Memory allocation & initialization of data structures which are 
     * required for the selection and sorting algorithms.
     ************************************************************************************************/

	/* Bid Values: This holds the bidding value for each active user & radio bearer */
	gDPMMCB.bidValues = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* Bid Index: This holds the bidding index for each active user & radio bearer. */
	gDPMMCB.biddingIndex = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* User Profile List: This holds information for all the user profiles. */
	gDPMMCB.userProfileList = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.userProfileList == NULL)
		return -1;

	/* Bid Value: This holds the bidding value which are to be sorted. */
	gDPMMCB.inputBidsToSort = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* Bid Index: This holds the bidding index which is to be sorted. */
	gDPMMCB.inputBidIndexToSort = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* Bid Values: This holds the sorted bidding values */
	gDPMMCB.sortedOutputBids = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* Bid Index: This holds the sorted bidding index */
	gDPMMCB.sortedOutputBidsIndex = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

	/* Scratch pad memory which is required by the sorting algorithm. */
	gDPMMCB.sortingScratchMemory = (uint32_t*)Dpm_allocMemory(sizeof(uint32_t) * numElements, errCode);
	if (gDPMMCB.bidValues == NULL)
		return -1;

    /**********************************************************************************
     ****************************** UE & RB Initialization ****************************
     **********************************************************************************/
    for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
    {
        /* Get the pointer to the user. */
        ptrUE = &gDPMMCB.ueDatabase[ueId];

        /* Populate the fields in the UE. */
        ptrUE->ueId = ueId;

        /* Cycle through and initialize all the radio bearers associated with the UE */
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Get the pointer to the radio bearer. */
            ptrRB = &ptrUE->rb[rbId];

            /* Populate the RB appropriately. */
            ptrRB->rbId   = rbId;
            ptrRB->ueId   = ueId;

            /* Initialize the RB packet list. */
            Dpm_listInit(&ptrRB->packetList);

            /* Open a queue associated with the RB */
            ptrRB->rbQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
            if (ptrRB->rbQueueHnd < 0)
            {
                *errCode = DPM_EINTERNAL;
                return -1;
            }

            /* Open the RLC processed queue */
            ptrRB->rlcProcessedQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                       QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
            if (ptrRB->rlcProcessedQueue < 0)
            {
                *errCode = DPM_EINTERNAL;
                return -1;
            }

            /* Register the radio bearer RAW report data with the report module. */
            if (Dpm_reportConfigureData(gDPMMCB.cumulativeRawReportHandle, ptrRB->ueId, ptrRB->rbId, NULL) < 0)
            {
                *errCode = DPM_EINTERNAL;
                return -1;
            }

            /* Register the radio bearer RLC report data with the report module. */
            if (Dpm_reportConfigureData(gDPMMCB.rlcReportHandle, ptrRB->ueId, ptrRB->rbId, NULL) < 0)
            {
                *errCode = DPM_EINTERNAL;
                return -1;
            }

            /* Register the radio bearer fresh RAW report data with the report module. */
            if (Dpm_reportConfigureData(gDPMMCB.freshRawReportHandle, ptrRB->ueId, ptrRB->rbId, NULL) < 0)
            {
                *errCode = DPM_EINTERNAL;
                return -1;
            }
        }
    }

    /**********************************************************************************
     ******************************* QCI Initialization *******************************
     **********************************************************************************/
    for (qci = 0; qci < DPM_MAX_QCI; qci++)
    {
        /* Allocate memory for the QCI Block. */
        ptrQCI = &gDPMMCB.qciDatabase[qci];

        /* Populate the QCI Block. */
        ptrQCI->qci = qci;

        /* Initialize the QCI packet list. */
        Dpm_listInit(&ptrQCI->packetList);
    }

    /* DPM Initialization was successful. */
    return 0;        
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the statistics. This is an overloaded
 *      function and the arguments can be wildcarded to get different types
 *      of statistics. 
 *  
 *   @verbatim
        Wildcarding Rules:
        ------------------
        ueId = 0xFFFF -> All users
        rbId = 0xFF   -> All radio bearers
        qci  = 0xFF   -> All QCI
	 @endverbatim
 *
 *   The function implements the following:-
 *
 *      - System Statistics (ueId, rbId and qci are all wildcarded)
 *      - Per QCI (qci is specified, ueId but rbId are wildcarded)
 *      - Per RB  (ueId and rbId are specified but qci is wildcarded)
 *      - Per User (ueId is specified but rbId and qci is wildcarded)
 *      - Per User/QCI (ueId and qci are specified but rbId is wildcarded)
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @param[in]  ueId
 *      User Identifer (See wildcarding rules)
 *  @param[in]  rbId
 *      Radio Bearer (See wildcarding rules)
 *  @param[in]  qci
 *      QCI (See wildcarding rules)
 *  @param[out] ptrStats
 *      Statistics populated by the API
 *  @param[out] errCode
 *      Error Code populated if the function failed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
int32_t Dpm_getStats (uint16_t ueId, uint8_t rbId, uint8_t qci, Dpm_Stats* ptrStats, int32_t* errCode)
{
    uint8_t             index;
    uint16_t            uniqueIndex;
    Dpm_QCI*            ptrQCI;
    Dpm_RB*             ptrRB;
    Dpm_ListNode*       ptrListNode;
    void*               csContext;
    Dpm_PacketNode*     ptrPacketNode;

    /* Sanity Check: Validate the arguments. */
    if (ptrStats == NULL)
    {
        *errCode = DPM_EINVAL;
        return -1;
    }

    /* Initialize the statistics block. */
    memset ((void*)ptrStats, 0, sizeof(Dpm_Stats));

    /* Is the statistics request across all users/radio bearers and all QCI? */
    if ((ueId == 0xFFFF) && (rbId == 0xFF) && (qci == 0xFF))
    {
        /* YES. We can simply cycle through all the QCI to get the complete system statistics */
        for (index = 1; index < DPM_MAX_QCI; index++)
        {
            /* Get the QCI block. */
            ptrQCI = Dpm_getQCI(index);

            /* Increment the statistics. */
            ptrStats->pktReceived   = ptrStats->pktReceived   + ptrQCI->pktReceived;
            ptrStats->pktDropped    = ptrStats->pktDropped    + ptrQCI->pktDropped;
            ptrStats->bytesReceived = ptrStats->bytesReceived + ptrQCI->bytesReceived;
            ptrStats->bytesDropped  = ptrStats->bytesDropped  + ptrQCI->bytesDropped;
        }

        /* Get the system statistics for all users across all QCI */
        return 0;
    }

    /* Is the statistics request for a specific user profile? */
    if ((ueId != 0xFFFF) && (rbId != 0xFF) && (qci == 0xFF))
    {
        /* YES. Get the RB block. */
        ptrRB = Dpm_getRB(ueId, rbId);

        /* Populate the statistics from the RB block. */
        ptrStats->pktReceived   = ptrRB->pktReceived;
        ptrStats->pktDropped    = ptrRB->pktDropped;
        ptrStats->bytesReceived = ptrRB->bytesReceived;
        ptrStats->bytesDropped  = ptrRB->bytesDropped;

        return 0;
    }

    /* Is the statistics request for a specific QCI but for all users/radio bearers? */
    if ((ueId == 0xFFFF) && (rbId == 0xFF) && (qci != 0xFF))
    {
        /* YES. Get the QCI block associated with the specified QCI. */
        ptrQCI = Dpm_getQCI(qci);

        /* Populate the statistics from the QCI block. */
        ptrStats->pktReceived   = ptrQCI->pktReceived;
        ptrStats->pktDropped    = ptrQCI->pktDropped;
        ptrStats->bytesReceived = ptrQCI->bytesReceived;
        ptrStats->bytesDropped  = ptrQCI->bytesDropped;

        /* Critical Section Enter: */
        csContext = Dpm_osalEnterCriticalSection ();

        /* Get the head packet enqueued in the QCI. */
        ptrListNode = Dpm_listGetHead (&ptrQCI->packetList);

        /* Critical Section Exit: */
        Dpm_osalExitCriticalSection (csContext);

        /* Was there a packet enqueued in the QCI? */
        if (ptrListNode != NULL)
        {
            /* YES. Get the packet node from the list node */
            ptrPacketNode = (Dpm_PacketNode*)((uint8_t*)ptrListNode - offsetof (Dpm_PacketNode, qciPktLinks));

            /* Record the packet arrival timestamp */
            ptrStats->packetTimestamp = ptrPacketNode->timeStamp;
        }

        /* Determine the number of active users in the system associated with the specific QCI. */
        for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
        {
            for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
            {
                /* Get the RB block. */
                ptrRB = Dpm_getRB(ueId, rbId);

                /* Get the unique index. */
                uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);

                /* Is this RB associated with the specific QCI. */
                if (gDPMMCB.qci[uniqueIndex] != qci)
                    continue;

                /* Is this radio bearer ACTIVE (Has data) ? */
                if ((gDPMMCB.rlcPendingBytes[uniqueIndex] != 0) || (gDPMMCB.rxByteCount[uniqueIndex] != 0))
                {
                    /* YES. This is an ACTIVE user; account for it. */
                    ptrStats->numActiveUE = ptrStats->numActiveUE + 1;

                    /* We are done with this user; we dont need to go through all the other
                     * radio bearers */
                    break;
                }
            }
        }
        return 0;
    }

    /* Is the statistics request for a specific user? */
    if ((ueId != 0xFFFF) && (rbId == 0xFF) && (qci == 0xFF))
    {
        /* Cycle through all the radio bearers associated with the specified user. */
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Get the RB Block. */
            ptrRB = Dpm_getRB(ueId, rbId);

            /* Increment the statistics. */
            ptrStats->pktReceived   = ptrStats->pktReceived + ptrRB->pktReceived;
            ptrStats->pktDropped    = ptrStats->pktDropped + ptrRB->pktDropped;
            ptrStats->bytesReceived = ptrStats->bytesReceived + ptrRB->bytesReceived;
            ptrStats->bytesDropped  = ptrStats->bytesDropped + ptrRB->bytesDropped;
        }
        return 0;
    }

    /* Is the statistics request for a specific user and QCI? */
    if ((ueId != 0xFFFF) && (rbId == 0xFF) && (qci != 0xFF))
    {
        /* Cycle through all the radio bearers associated with the specified user. */
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Get the RB Block. */
            ptrRB = Dpm_getRB(ueId, rbId);

            /* Get the unique index. */
            uniqueIndex = Dpm_getUniqueIndex(ueId, rbId);            

            /* Is this RB associated with the specific QCI? */
            if (gDPMMCB.qci[uniqueIndex] != qci)
                continue;

            /* Ok we have a match; increment the statistics. */
            ptrStats->pktReceived   = ptrStats->pktReceived + ptrRB->pktReceived;
            ptrStats->pktDropped    = ptrStats->pktDropped + ptrRB->pktDropped;
            ptrStats->bytesReceived = ptrStats->bytesReceived + ptrRB->bytesReceived;
            ptrStats->bytesDropped  = ptrStats->bytesDropped + ptrRB->bytesDropped;
        }
        return 0;
    }

    /* Any other combination is NOT supported. */
    *errCode = DPM_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clear all the statistics in the DPM i.e. the QCI 
 *      and user profile statistics.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
int32_t Dpm_clearStats (void)
{
    uint16_t    ueId;
    uint8_t     rbId;
    uint8_t     qci;
    Dpm_QCI*    ptrQCI;
    Dpm_RB*     ptrRB;

    /* Clear all the QCI statistics. */
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {
        /* Get the QCI block. */
        ptrQCI = Dpm_getQCI(qci);

        /* Clear the QCI statistics. */
        ptrQCI->pktReceived   = 0;
        ptrQCI->pktDropped    = 0;
        ptrQCI->bytesReceived = 0;
        ptrQCI->bytesDropped  = 0;
    }

    /* Cycle through all the users & radio bearers and clear them up also. */
    for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
    {
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Get the RB block */
            ptrRB = Dpm_getRB(ueId, rbId);

            /* Clear the RB statistics. */
            ptrRB->pktReceived   = 0;
            ptrRB->pktDropped    = 0;
            ptrRB->bytesReceived = 0;
            ptrRB->bytesDropped  = 0;
        }
    }

    /* All statistics have been cleared. */
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to start the DPM for each core.
 *
 *  \ingroup DPM_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   0
 */
int32_t Dpm_start (void)
{
    return 0;
}

