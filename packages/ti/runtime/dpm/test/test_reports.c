/*
 *   @file  test_reports.c
 *
 *   @brief   
 *      Test code which tests the various fresh, cumulative & sorted 
 *      reports and ensures that the reports are valid. These tests are
 *      done for multiple user & radio bearer combination.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* Message Communicator Include Files. */
#include <ti/runtime/msgcom/msgcom.h>

/* Packet Library Include Files. */
#include <ti/runtime/pktlib/pktlib.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* DPM Include Files. */
#include <ti/runtime/dpm/dpm.h>

/**********************************************************************
 ************************* Unit Test Definitions **********************
 **********************************************************************/

/* Maximum number of sorted user profiles */
#define TEST_MAX_SORTED_USER_PROFILE        8

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Heap for receiving downlink packets. */
extern Pktlib_HeapHandle   pktMgmtHeap;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle   appPktlibInstanceHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Utility Function which validates the report with the specified
 *      parameters
 *
 *  @param[in]  ptrReport
 *      Report contents which are to be validated
 *  @param[in]  ueId
 *      User Identifier
 *  @param[in]  rbId
 *      Radio Bearer
 *  @param[in]  qci
 *      QoS Class Indicator
 *  @param[in]  rxByteCount
 *      Receive Byte count 
 *  @param[in]  rlcPendingBytes
 *      Number of bytes pending in the RLC
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t testValidateReport 
(
    Dpm_Report* ptrReport,
    uint8_t     ueId,
    uint8_t     rbId,
    uint8_t     qci,
    uint32_t    rxByteCount,
    uint32_t    rlcPendingBytes
)
{
    /* Validate the radio bearer */ 
    if (ptrReport->rbId != rbId)
    {
        System_printf ("Error: Invalid radio bearer %d (Expected %d)\n", ptrReport->rbId, rbId);
        return -1;
    }

    /* Validate the user identifier */ 
    if (ptrReport->ueId != ueId)
    {
        System_printf ("Error: Invalid user id %d (Expected %d)\n", ptrReport->ueId, ueId);
        return -1;
    }

    /* Validate the QCI */
    if (ptrReport->qci != qci)
    {
        System_printf ("Error: Invalid QCI %d (Expected %d)\n", ptrReport->qci, qci);
        return -1;
    }

    /* Validate the receive byte count */
    if (ptrReport->rxByteCount != rxByteCount)
    {
        System_printf ("Error: Invalid rx bytes detected %d (Expected %d)\n", ptrReport->rxByteCount, rxByteCount);
        return -1;
    }

    /* Validate the RLC Pending Bytes */
    if (ptrReport->rlcPendingBytes != rlcPendingBytes)
    {
        System_printf ("Error: Invalid pending data bytes detected %d (Expected %d)\n", ptrReport->rlcPendingBytes, rlcPendingBytes);
        return -1;
    }

    /* Report is good. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test fresh reports for the specified 
 *      number of user & radio bearers
 *
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_freshDPMReports(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Ti_Pkt*             ptrNewPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    Dpm_Report          report;
    int32_t             errCode;
    uint16_t            ueId = 0;
    uint8_t             rbId;
    uint8_t             qci   = 7;
    uint16_t            index = 0;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Fresh Reports for %d users & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the specified users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* For each user; cycle through all the specified radio bearers. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 128);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the user profiles from the DPM
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Raw Report detected \n");
        return -1;
    }

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;

    /* Cycle through all the fresh reports. */
    while (1)
    {
        /* Get the report given the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 128, 0) < 0)
            return -1;

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }

        /* Increment the index */
        index = index + 1;

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_FRESH, &userProfile, &nextUserProfile) == 0)
            break;

        /* Store back the next RAW report. */
        userProfile = nextUserProfile;
    }

    /* Make sure that all the reports were detected. */
    if (index != (numUE * numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, (numUE * numRB));
        return -1;
    }

    /* Fresh reports are valid only till they have been passed to the scheduler. So 
     * now if we try and get the head of the fresh reports this should be empty */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 1)
    {
        System_printf ("Error: DPM Fresh Reports detected (This should NOT happen)\n");
        return -1;
    }

    /* Allocating another packet to simulate packet reception after the scheduler has gotten 
     * the fresh report. */
    ptrNewPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 256);
    if (ptrNewPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Push this packet to User Id: 5 and RB 0 */
    if (Dpm_enqueuePkt (5, 0, qci, ptrNewPkt, &errCode) < 0)
    {
        System_printf ("Error: Unable to receive the packet on the DPM.\n");
        return -1;
    }

    /*************************************************************************
     * Scheduler Simulation: Get the fresh reports 
     *************************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Raw Report detected \n");
        return -1;
    }

    /* There should be no more fresh reports. */
    if (Dpm_getNextUserProfile (Dpm_ReportType_FRESH, &userProfile, &nextUserProfile) == 1)
    {
        System_printf ("Error: Detected fresh report for %d:%d but the report should be empty\n", 
                        nextUserProfile.ueId, nextUserProfile.rbId);
        return -1;
    }

    /*************************************************************************
     * RLC Simulation: Dequeue & process all the packets from the DPM Module.
     *************************************************************************/
    /* Cycle through all the users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
            if (pktTracker[ueId][rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[ueId][rbId], ptrPkt);
                return -1;
            }

            /* Indicate to the DPM that there are no more pending bytes */
            Dpm_rlcReport (ueId, rbId, 0, 0, 0);

            /* Cleanup the packet */
            Dpm_freeRLCPkt(ueId, rbId, ptrPkt);
        }
    }

    /* Handle the "extra" packet we had pushed for testing the fresh reports. */
    if (Dpm_dequeuePkt (5, 0, &ptrPkt, &errCode) < 0)
    {
        System_printf ("Error: Dequeue Packet failed\n");
        return -1;
    }

    /* Validate & make sure this has the correct packet length. */
    if (ptrPkt != ptrNewPkt)
    {
        System_printf ("Error: Expected packet 0x%p but got 0x%p\n", ptrNewPkt, ptrPkt);
        return -1;
    }

    /* Indicate to the DPM that the RLC processing is complete. */
    Dpm_rlcReport (5, 0, 0, 0, 0);

    /* Cleanup the packet. */
    Dpm_freeRLCPkt (5, 0, ptrNewPkt);

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty fresh report in the next TTI. 
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 1)
    {
        System_printf ("Error: Non Empty fresh report detected\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test fresh reports & RLC reports
 *      together.
 *       
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_freshDPMRLCReports(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    Dpm_Report          report;
    int32_t             errCode;
    uint16_t            ueId;
    uint8_t             rbId;
    uint16_t            index = 0;
    uint8_t             qci = 3;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Fresh Reports (RLC Simulation) for %d users & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packets from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 64);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty User Profile detected \n");
        return -1;
    }

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;
 
    while (1)
    {
        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 64, 0) < 0)
            return -1;

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }

        /* Increment the index */
        index = index + 1;

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_FRESH, &userProfile, &nextUserProfile) == 0)
            break;

        /* Traverse the user profiles. */
        userProfile = nextUserProfile;
    }

    /* Make sure that all the reports were detected. */
    if (index != (numUE*numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, (numUE*numRB));
        return -1;
    }

    /*************************************************************************
     * RLC Simulation: Dequeue & process all the packets from the DPM Module.
     *************************************************************************/
    /* Cycle through all the users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* For each user cycle through all the specified radio bearers */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
            if (pktTracker[ueId][rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[ueId][rbId], ptrPkt);
                return -1;
            }

            /* All odd radio bearers have completely processed the entire RLC packet;
             * while even radio bearers have processed all but 50 bytes out of the 
             * initial payload. i.e. 
             *  RB 0, 2, 4, 6 should have 50 pending bytes while
             *  RB 1, 3, 5, 7 should have no pending bytes */
            if ((rbId % 2) == 0)
                Dpm_rlcReport (ueId, rbId, 50, 0, 0);
            else
                Dpm_rlcReport (ueId, rbId, 0, 0, 0);

            /* The RLC consumes the packet */
            Dpm_freeRLCPkt (ueId, rbId, ptrPkt);
        }
    }

    /*************************************************************************
     * Scheduler Simulation: Get the fresh reports. 
     *************************************************************************/

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty User Profile detected \n");
        return -1;
    }

    /* Cycle through all the user profiles. */
    while (1)
    {
        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 0, 50) < 0)
            return -1;

        /* We should only get RLC pending data on the even radio bearers. */
        rbId = (rbId + 2);
        if (rbId >= numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_FRESH, &userProfile, &nextUserProfile) == 0)
            break;

        /* Traverse the user profiles. */
        userProfile = nextUserProfile;
    }

    /*************************************************************************
     * RLC Simulation: RLC will now process all the even radio bearers too.
     *************************************************************************/
    for (ueId = 0; ueId < numUE; ueId++)
    {
        for (rbId = 0; rbId < numRB; rbId = rbId + 2)
            Dpm_rlcReport (ueId, rbId, 0, 0, 0);
    }

    /*************************************************************************
     * Scheduler Simulation: Get the Fresh Report head. This should be EMPTY 
     * since there no new packets were received & the RLC reports also 
     * indicated that all data was processed.
     *************************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) == 1)
    {
        System_printf ("Error: Head user profile was valid (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }
    
    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test cumulative reports for the specific
 *      number of users & radio bearers.
 *
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_cumulativeDPMReports(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    int32_t             errCode;
    uint16_t            ueId;
    uint8_t             qci = 5;
    uint8_t             rbId;
    uint16_t            index = 0;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Cumulative reports for %d & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);
    
    /* Cycle through all the specified users. */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers associated with the users. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packet of different size from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 384);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Raw Report detected \n");
        return -1;
    }

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;
 
    while (1)
    {
        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 384, 0) < 0)
            return -1;

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }

        /* Increment the index */
        index = index + 1;

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Traverse the user profiles. */
        userProfile = nextUserProfile;
    }

    /* Make sure that all the reports were detected. */
    if (index != (numUE*numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, (numUE*numRB));
        return -1;
    }

    /*************************************************************************
     * RLC Simulation: Dequeue & process all the packets from the DPM Module.
     *************************************************************************/
    for (ueId = 0; ueId < numUE; ueId++)
    {
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
            if (pktTracker[ueId][rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[rbId], ptrPkt);
                return -1;
            }

            /* Indicate to the DPM that there are no more pending bytes */
            Dpm_rlcReport (ueId, rbId, 0, 0, 0);

            /* Cleanup the packet */
            Dpm_freeRLCPkt(ueId, rbId, ptrPkt);
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test RLC reports in conjunction with 
 *      cummulative reports.
 *
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_cumulativeRLCReports(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    int32_t             errCode;
    uint16_t            ueId;
    uint8_t             rbId;
    uint8_t             qci = 1;
    uint16_t            index = 0;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Cumulative Reports (RLC Simulation) for %d users & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the users. */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers for the specific user */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packets from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 312);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver: */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Head User Profile is not present\n");
        return -1;
    }

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;
 
    while (1)
    {
        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 312, 0) < 0)
            return -1;

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }

        /* Increment the index */
        index = index + 1;

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Traverse the user profiles. */
        userProfile = nextUserProfile;
    }

    /* Make sure that all the reports were detected. */
    if (index != (numUE*numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, (numUE*numRB));
        return -1;
    }

    /*************************************************************************
     * RLC Simulation: 
     *************************************************************************/
    /* Cycle through all the users. */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* For each user cycle through all the radio bearers. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
            if (pktTracker[ueId][rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[ueId][rbId], ptrPkt);
                return -1;
            }

            /********************************************************************
             * RLC Simulation: All radio bearers except 0 have processed all the 
             * data. Radio Bearer 0 has 256 pending bytes
             ********************************************************************/
            if (rbId != 0)
                Dpm_rlcReport (ueId, rbId, 0, 0, 0);
            else
                Dpm_rlcReport (ueId, rbId, 256, 0, 0);

            /* The packet is with the RLC module. So lets clean it out here itself. */
            Dpm_freeRLCPkt(ueId, rbId, ptrPkt);
        }
    }

    /*************************************************************************
     * Scheduler Simulation: Get the cummulative reports. There should exist
     * a report only for radio bearer 0.
     *************************************************************************/

    /* Initialize the User & Radio Bearer */
    ueId = 0;
    rbId = 0;

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Head User Profile is not present\n");
        return -1;
    }
 
    while (1)
    {
        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report: The report should always be in ascending order.  */
        if (testValidateReport(&report, ueId, rbId, qci, 0, 256) < 0)
            return -1;

        /* Jump to the next user */
        ueId = ueId + 1;

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Traverse the user profiles. */
        userProfile = nextUserProfile;
    }

    /*************************************************************************
     * RLC Simulation: Dequeue & process all the packets on radio bearer 0
     *************************************************************************/
    for (ueId = 0; ueId < numUE; ueId++)
        Dpm_rlcReport (ueId, 0, 0, 0, 0);

    /* Get the head user profile: This should be empty  */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM Head User Profile is valid (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM sorted reports.
 *
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_sortedDPMReport(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile[TEST_MAX_SORTED_USER_PROFILE];
    int32_t             errCode;
    uint16_t            ueId;
    uint8_t             rbId;
    uint8_t             qci = 2;
    uint16_t            index = 0;
    uint16_t            count = 0;
    Pktlib_HeapStats    startHeapStats;
    uint32_t            numSortedUserProfiles;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Sorted Reports for %d users & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the users. */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers for the specific user */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate packets from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 64);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver: */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    while (1)
    {
        /********************************************************************
         * Scheduler Simulation: Get the sorted user profile
         ********************************************************************/
        if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                          &numSortedUserProfiles, &errCode) < 0)
        {
            System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
            return -1;
        }

        /* Are there any sorted reports available? */
        if (numSortedUserProfiles == 0)
            break;

        /**********************************************************************
         * Validation: 
         *
         * This validation is very specific to the implementation of the 
         * Dpm_appBidFunction function in the dpm_app.h. 
         *
         * The current *dummy* implementation gives highest bid value to larger 
         * userID and radio bearer.
         **********************************************************************/
#if 1
        {
            uint16_t    highestUE;
            uint8_t     highestRB;

            /* Initialize the highest UE & RB */
            highestUE = (numUE - 1);
            highestRB = (numRB - 1);

            /* Now discount all that have been processed. */
            for (index = 0; index < count; index++)
            {
                highestRB = highestRB - 1;
                if ((int8_t)highestRB < 0)
                {
                    highestUE = highestUE - 1;
                    highestRB = (numRB - 1);
                }
            }

            /* Cycle through all the sorted user profiles. */
            for (index = 0; index < numSortedUserProfiles; index++)
            {
                /* Get the report given the user profile. */
                Dpm_getReport (&userProfile[index], &report);

                /* Validate the report: The report should always be in ascending order.  */
                if (testValidateReport(&report, highestUE, highestRB, qci, 64, 0) < 0)
                    return -1;

                /* Move back the UE & RB appropriately. */
                highestRB = highestRB - 1;
                if ((int8_t)highestRB < 0)
                {
                    highestUE = highestUE - 1;
                    highestRB = (numRB - 1);
                }
            }
        }
#endif
        /* Keep track of the total number of sorted user profiles which have been reported. */
        count = count + numSortedUserProfiles;

        /*************************************************************************
         * RLC Simulation: Dequeue & process all the packets
         *************************************************************************/
        for (index = 0; index < numSortedUserProfiles; index++)
        {
            /* Get the report given the user profile. */
            Dpm_getReport (&userProfile[index], &report);

            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (userProfile[index].ueId, userProfile[index].rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Ensure that the removed packet is valid */
            if (pktTracker[userProfile[index].ueId][userProfile[index].rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", 
                                pktTracker[userProfile[index].ueId][userProfile[index].rbId], ptrPkt);
                return -1;
            }

            /* Indicate to the DPM that there are no more pending bytes */
            Dpm_rlcReport (userProfile[index].ueId, userProfile[index].rbId, 0, 0, 0);

            /* Cleanup the packet */
            Dpm_freeRLCPkt(userProfile[index].ueId, userProfile[index].rbId, ptrPkt);
        }
    }

    /* Make sure that all the user profiles were detected */
    if (count != (numUE*numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", count, (numUE*numRB));
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to traverse the DPM sorted reports 
 *      using the sorted report traversal API.
 *
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_sortedTraversalReports(uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile[TEST_MAX_SORTED_USER_PROFILE];
    int32_t             errCode;
    uint16_t            ueId;
    uint8_t             rbId;
    uint8_t             qci = 2;
    uint16_t            index = 0;
    uint16_t            count = 0;
    Pktlib_HeapStats    startHeapStats;
    uint32_t            numSortedUserProfiles;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Sorted Traversal Reports for %d users & %d RB\n", numUE, numRB);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the users. */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the radio bearers for the specific user */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate packets from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 64);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver: */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profile
     ********************************************************************/
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                   &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
        return -1;
    }

    while (1)
    {
        /* Are there any sorted reports available? */
        if (numSortedUserProfiles == 0)
            break;

        /**********************************************************************
         * Validation: 
         *
         * This validation is very specific to the implementation of the 
         * Dpm_appBidFunction function in the dpm_app.h. 
         *
         * The current *dummy* implementation gives highest bid value to larger 
         * userID and radio bearer.
         **********************************************************************/
#if 1
        {
            uint16_t    highestUE;
            uint8_t     highestRB;

            /* Initialize the highest UE & RB */
            highestUE = (numUE - 1);
            highestRB = (numRB - 1);

            /* Now discount all that have been processed. */
            for (index = 0; index < count; index++)
            {
                highestRB = highestRB - 1;
                if ((int8_t)highestRB < 0)
                {
                    highestUE = highestUE - 1;
                    highestRB = (numRB - 1);
                }
            }

            /* Cycle through all the sorted user profiles. */
            for (index = 0; index < numSortedUserProfiles; index++)
            {
                /* Get the report given the user profile. */
                Dpm_getReport (&userProfile[index], &report);

                /* Validate the report: The report should always be in ascending order.  */
                if (testValidateReport(&report, highestUE, highestRB, qci, 64, 0) < 0)
                    return -1;

                /* Move back the UE & RB appropriately. */
                highestRB = highestRB - 1;
                if ((int8_t)highestRB < 0)
                {
                    highestUE = highestUE - 1;
                    highestRB = (numRB - 1);
                }
            }
        }
#endif
        /* Keep track of the total number of sorted user profiles which have been reported. */
        count = count + numSortedUserProfiles;

        /*************************************************************************
         * RLC Simulation: Dequeue & process all the packets
         *************************************************************************/
        for (index = 0; index < numSortedUserProfiles; index++)
        {
            /* Get the report given the user profile. */
            Dpm_getReport (&userProfile[index], &report);

            /* Remove the packet from the specific user & radio bearer */
            if (Dpm_dequeuePkt (userProfile[index].ueId, userProfile[index].rbId, &ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: Dequeue Packet failed\n");
                return -1;
            }

            /* Ensure that the removed packet is valid */
            if (pktTracker[userProfile[index].ueId][userProfile[index].rbId] != ptrPkt)
            {
                System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", 
                                pktTracker[userProfile[index].ueId][userProfile[index].rbId], ptrPkt);
                return -1;
            }

            /* Indicate to the DPM that there are no more pending bytes */
            Dpm_rlcReport (userProfile[index].ueId, userProfile[index].rbId, 0, 0, 0);

            /* Cleanup the packet */
            Dpm_freeRLCPkt(userProfile[index].ueId, userProfile[index].rbId, ptrPkt);
        }

        /* Get the next set of sorted user profiles. */
        if (Dpm_getNextSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                           &numSortedUserProfiles, &errCode) < 0)
        {
            System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
            return -1;
        }
    }

    /* Make sure that all the user profiles were detected */
    if (count != (numUE*numRB))
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", count, (numUE*numRB));
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profile
     ********************************************************************/
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                   &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
        return -1;
    }
    if (numSortedUserProfiles != 0)
    {
        System_printf ("Error: There should be 0 pending user profiles but detected %d\n", numSortedUserProfiles);
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the packet delay budget for the specific 
 *      QCI using sorted reports.
 *
 *  @param[in]  qci
 *      QoS Class Indicator for which the test is to be executed
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_PDBUsingSortedReports(uint8_t qci)
{
    Ti_Pkt*             ptrPkt;
    uint32_t            index = 0;
    int32_t             errCode;
    uint8_t             ueId = 1;
    uint8_t             rbId = 1;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile[TEST_MAX_SORTED_USER_PROFILE];
    Pktlib_HeapStats    startHeapStats;
    uint32_t            numSortedUserProfiles;
    Pktlib_HeapStats    endHeapStats;
    int32_t             packetDelayBudgetSnapshot[10];

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing PDB (Using Sorted Profiles) for %d:%d qci %d\n", ueId, rbId, qci);

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 216);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Pass the packet to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /* Cycle through & record the statistics. */
    for (index = 0; index < 10; index++)
    {
        /********************************************************************
         * Scheduler Simulation: Get the sorted user profiles from the DPM
         ********************************************************************/
        if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                          &numSortedUserProfiles, &errCode) < 0)
        {
            System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
            return -1;
        }

        /* Ensure that we received only a single user profile */
        if (numSortedUserProfiles != 1)
        {
            System_printf ("Error: Received %d user profiles\n", numSortedUserProfiles);
            return -1;
        }

        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile[0], &report);

        /* Record the packet delay budget reported by the DPM */
        packetDelayBudgetSnapshot[index] = report.pdb;

        /* Between each run we let the task sleep for 50msec */
        Task_sleep(50);
    }

    /* NOTE: Currently the MAX QCI delay budget configured is 300msec. The loop above executes 
     * for 500msec; which implies that the last set of entries in the packet delay snapshots
     * should be set to -1. */
    for (index = 0; index < 10; index++)
        System_printf ("Debug: PDB [%d] = %d ticks\n", index, packetDelayBudgetSnapshot[index]);

    /* Sanity Check: Make sure that at least the last entry is -1 */
    if (packetDelayBudgetSnapshot[9] > 0)
    {
        System_printf ("Error: Packet delay budget is invalid %d ticks\n", packetDelayBudgetSnapshot[9]);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: RLC has finally decided to process the packet
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Cleanup the processed packet */
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt);

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                   &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
        return -1;
    }

    /* Ensure that we received no user profile since everything has been processed. */
    if (numSortedUserProfiles > 0)
    {
        System_printf ("Error: Received %d user profiles (Expected 0)\n", numSortedUserProfiles);
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the packet delay budget for the specific 
 *      QCI using user profiles.
 *
 *  @param[in]  qci
 *      QoS Class Indicator for which the test is to be executed
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_PDBUsingUserProfiles(uint8_t qci)
{
    Ti_Pkt*             ptrPkt;
    uint32_t            index = 0;
    int32_t             errCode;
    uint8_t             ueId = 1;
    uint8_t             rbId = 1;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    int32_t             packetDelayBudgetSnapshot[10];

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing PDB (Using Raw Profiles) for %d:%d qci %d\n", ueId, rbId, qci);

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 216);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Pass the packet to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /* Cycle through & record the statistics. */
    for (index = 0; index < 10; index++)
    {
        /********************************************************************
         * Scheduler Simulation: Get the sorted user profiles from the DPM
         ********************************************************************/
        if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
        {
            System_printf ("Error: Empty Head User Profile detected\n");
            return -1;
        }

        /* Get the report associated with the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report:  */
        if (testValidateReport(&report, ueId, rbId, qci, 216, 0) < 0)
            return -1;

        /* Record the packet delay budget reported by the DPM */
        packetDelayBudgetSnapshot[index] = report.pdb;

        /* Between each run we let the task sleep for 50msec */
        Task_sleep(50);
    }

    /* NOTE: Currently the MAX QCI delay budget configured is 300msec. The loop above executes 
     * for 500msec; which implies that the last set of entries in the packet delay snapshots
     * should be set to -1. */
    for (index = 0; index < 10; index++)
        System_printf ("Debug: PDB [%d] = %d ticks\n", index, packetDelayBudgetSnapshot[index]);

    /* Sanity Check: Make sure that at least the last entry is -1 */
    if (packetDelayBudgetSnapshot[9] > 0)
    {
        System_printf ("Error: Packet delay budget is invalid %d ticks\n", packetDelayBudgetSnapshot[9]);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: RLC has finally decided to process the packet
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Cleanup the processed packet */
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt);

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: Expected empty head profile but valid profile returned (%d:%d)\n", 
                        userProfile.ueId, userProfile.rbId);
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test reports for the specified 
 *      number of user & radio bearers 
 *
 *  @param[in]  reportType
 *      Report Type for which the test is to be executed.
 *  @param[in]  numUE
 *      Number of users 
 *  @param[in]  numRB 
 *      Number of radio bearers
 *  @param[in]  numDisabledUE 
 *      Number of disabled users
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t testDisabledReports
(
    Dpm_ReportType      reportType, 
    uint16_t            numUE, 
    uint8_t             numRB, 
    uint8_t             numDisabledUE
)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    Dpm_Report          report;
    int32_t             errCode;
    uint16_t            ueId = 0;
    uint8_t             rbId;
    uint8_t             qci   = 7;
    uint16_t            index = 0;
    uint32_t            numReportsExpected;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    Ti_Pkt*             pktTracker[DPM_MAX_UE][DPM_MAX_RB];

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM Disabled Reports (%s) for %d users & %d RB and %d disabled users\n", 
                    (reportType == Dpm_ReportType_FRESH) ? "Fresh" : "Cumulative", 
                    numUE, numRB, numDisabledUE);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through all the specified users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* For each user; cycle through all the specified radio bearers. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 128);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the allocated packet */
            pktTracker[ueId][rbId] = ptrPkt;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /* Cycle through and disable all the user profiles which have been specified. */
    for (ueId = 0; ueId < numDisabledUE; ueId++)
    {
        /* For each user; cycle through all the specified radio bearers. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Disable the user profile. */
            userProfile.ueId = ueId;
            userProfile.rbId = rbId;

            /* Disable the user profile. */
            if (Dpm_configureUserProfile(&userProfile, 0) < 0)
            {
                System_printf ("Error: Unable to disable user profile\n");
                return -1;
            }
        }
    }

    /* Determine how many reports are expected. */
    numReportsExpected = (numUE*numRB) - (numDisabledUE*numRB);

    /* Initialize to the first enabled user profile which we expect to retreive. */
    ueId = numDisabledUE;
    rbId = 0;

    /* Run through the loop and get all the ENABLED user profiles. */
    while (1)
    {
        /* For the first iteration we need to get the head user profile */
        if (index == 0)
        {
            if (Dpm_getHeadUserProfile (reportType, &userProfile) == 0)
                break;
        }
        else
        {
            /* Get the next user profile. */
            if (Dpm_getNextUserProfile (reportType, &userProfile, &nextUserProfile) == 0)
                break;

            /* Get the user profile to be processed. */
            userProfile = nextUserProfile;
        }

        /* Increment the number of reports retreived. */
        index++;

        /* Get the report given the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report. */
        if (testValidateReport(&report, ueId, rbId, qci, 128, 0) < 0)
            return -1;

        /***************************************************************
         * RLC Simulation: Dequeue and consume the packet. 
         ***************************************************************/
        if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (ueId, rbId, 0, 0, 0);

        /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
        if (pktTracker[ueId][rbId] != ptrPkt)
        {
            System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[ueId][rbId], ptrPkt);
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(ueId, rbId, ptrPkt);

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }
    }

    /* Make sure that all the expected reports were received. */
    if (index != numReportsExpected)
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, numReportsExpected);
        return -1;
    }

    /* Cycle through and enable all the user profiles which have been specified. */
    for (ueId = 0; ueId < numDisabledUE; ueId++)
    {
        /* For each user; cycle through all the specified radio bearers. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Disable the user profile. */
            userProfile.ueId = ueId;
            userProfile.rbId = rbId;

            /* Enable the user profile. */
            if (Dpm_configureUserProfile(&userProfile, 1) < 0)
            {
                System_printf ("Error: Unable to enable user profile\n");
                return -1;
            }
        }
    }

    /* Reset the counters */
    index = 0;
    ueId  = 0;
    rbId  = 0;

    /* We have now enabled all the users and should see their reports. */
    numReportsExpected = (numDisabledUE*numRB);

    /* Run through the loop and get all the newly enabled user profiles. */    
    while (1)
    {
        /* For the first iteration we need to get the head user profile */
        if (index == 0)
        {
            /* Get the head user profile. */
            if (Dpm_getHeadUserProfile (reportType, &userProfile) == 0)
                break;
        }
        else
        {
            /* Get the next user profile. */
            if (Dpm_getNextUserProfile (reportType, &userProfile, &nextUserProfile) == 0)
                break;

            /* Get the user profile to be processed. */
            userProfile = nextUserProfile;
        }

        /* Increment the number of reports retreived. */
        index++;

        /* Get the report given the user profile. */
        Dpm_getReport (&userProfile, &report);

        /* Validate the report. */
        if (testValidateReport(&report, ueId, rbId, qci, 128, 0) < 0)
            return -1;

        /***************************************************************
         * RLC Simulation: Dequeue and consume the packet. 
         ***************************************************************/
        if (Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (ueId, rbId, 0, 0, 0);

        /* Validate & ensure that the packet dequeued is the same as the one being tracked. */
        if (pktTracker[ueId][rbId] != ptrPkt)
        {
            System_printf ("Error: Invalid Packet dequeued Expected 0x%p Got 0x%p\n", pktTracker[ueId][rbId], ptrPkt);
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(ueId, rbId, ptrPkt);

        /* Increment the user & radio bearer appropriately. */
        rbId = (rbId + 1);
        if (rbId == numRB)
        {
            rbId = 0;
            ueId = ueId + 1;
        }
    }

    /* Make sure that all the expected reports were received. */
    if (index != numReportsExpected)
    {
        System_printf ("Error: Detected %d reports (Expected %d)\n", index, numReportsExpected);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty fresh report in the next TTI. 
     ********************************************************************/
    if (Dpm_getHeadUserProfile (reportType, &userProfile) == 1)
    {
        System_printf ("Error: Non Empty fresh report detected\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the packet delay budget for the specific 
 *      QCI using user profiles.
 *
 *  @param[in]  qci
 *      QoS Class Indicator for which the test is to be executed
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_multiplePacketPDBRawProfiles(uint8_t qci)
{
    Ti_Pkt*             ptrPkt;
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    int32_t             errCode;
    uint8_t             ueId = 1;
    uint8_t             rbId = 1;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            deltaTime = 100;
    int32_t             packetDelayBudgetSnapshot[2];

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing Multiple Packet PDB (Using Raw Profiles) for %d:%d qci %d\n", ueId, rbId, qci);

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 200);
    if (ptrPkt1 == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 300);
    if (ptrPkt2 == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Pass Packet1 to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt1, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /* Enqueue the second packet after the specified delay. */
    Task_sleep(deltaTime);

    /* Pass Packet2 to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt2, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: Empty Head User Profile detected\n");
        return -1;
    }

    /* Get the report associated with the user profile. */
    Dpm_getReport (&userProfile, &report);

    /* Validate the report: Since this is a cumulative report the total packet size is the 
     * sum total of Packet1 and Packet2. */
    if (testValidateReport(&report, ueId, rbId, qci, (Pktlib_getPacketLen(ptrPkt1) + Pktlib_getPacketLen(ptrPkt2)), 0) < 0)
        return -1;

    /* Record the packet delay budget reported by the DPM */
    packetDelayBudgetSnapshot[0] = report.pdb;

    /********************************************************************
     * RLC Simulation: Dequeue and process the first packet. 
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Ensure that this is the first packet. */
    if (ptrPkt != ptrPkt1)
    {
        System_printf ("Error: Invalid packet detected 0x%p (Expected 0x%p)\n", ptrPkt, ptrPkt1);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/    
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: Empty Head User Profile detected\n");
        return -1;
    }

    /* Get the report associated with the user profile. */
    Dpm_getReport (&userProfile, &report);

    /* Validate the report:  */
    if (testValidateReport(&report, ueId, rbId, qci, Pktlib_getPacketLen(ptrPkt2), 0) < 0)
        return -1;

    /* Record the packet delay budget reported by the DPM */
    packetDelayBudgetSnapshot[1] = report.pdb;

    /********************************************************************
     * RLC Simulation: Dequeue and process the second packet. 
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Ensure that this is the first packet. */
    if (ptrPkt != ptrPkt2)
    {
        System_printf ("Error: Invalid packet detected 0x%p (Expected 0x%p)\n", ptrPkt, ptrPkt2);
        return -1;
    }

    /* Cleanup the packets. */
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt1);
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt2);

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/    
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: Expected empty user profile but got for %d:%d\n", 
                        userProfile.ueId, userProfile.rbId);
        return -1;
    }

    /* Display the Packet Delay Budgets from the reports on the console. */
    System_printf ("Debug: Time between the packets          is %d ticks\n", (deltaTime*1000000));
    System_printf ("Debug: Packet Delay Budget for Profile 0 is %d ticks\n", packetDelayBudgetSnapshot[0]);
    System_printf ("Debug: Packet Delay Budget for Profile 1 is %d ticks\n", packetDelayBudgetSnapshot[1]);

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the packet delay budget for the specific 
 *      QCI using user profiles.
 *
 *  @param[in]  qci
 *      QoS Class Indicator for which the test is to be executed
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_multiplePacketPDBSortedReports(uint8_t qci)
{
    Ti_Pkt*             ptrPkt;
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    int32_t             errCode;
    uint8_t             ueId = 1;
    uint8_t             rbId = 1;
    Dpm_Report          report;
    Dpm_UserProfile     userProfile[TEST_MAX_SORTED_USER_PROFILE];
    uint32_t            numSortedUserProfiles;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            deltaTime = 200;
    int32_t             packetDelayBudgetSnapshot[2];

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing Multiple Packet PDB (Using Sorted Profiles) for %d:%d qci %d\n", ueId, rbId, qci);

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 100);
    if (ptrPkt1 == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Allocate a packet from the NETFP Data Receive Heap. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 150);
    if (ptrPkt2 == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
        return -1;
    }

    /* Pass Packet1 to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt1, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /* Enqueue the second packet after the specified delay. */
    Task_sleep(deltaTime);

    /* Pass Packet2 to the DPM driver */
    if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt2, &errCode) < 0)
    {
        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                      &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted user profiles\n");
        return -1;
    }

    /* There is only user profile which is ACTIVE */
    if (numSortedUserProfiles != 1)
    {
        System_printf ("Error: Detected %d user profiles\n", numSortedUserProfiles);
        return -1;
    }

    /* Get the report associated with the user profile. */
    Dpm_getReport (&userProfile[0], &report);

    /* Validate the report: Since this is a cumulative report the total packet size is the 
     * sum total of Packet1 and Packet2. */
    if (testValidateReport(&report, ueId, rbId, qci, (Pktlib_getPacketLen(ptrPkt1) + Pktlib_getPacketLen(ptrPkt2)), 0) < 0)
        return -1;

    /* Record the packet delay budget reported by the DPM */
    packetDelayBudgetSnapshot[0] = report.pdb;

    /********************************************************************
     * RLC Simulation: Dequeue and process the first packet. 
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Ensure that this is the first packet. */
    if (ptrPkt != ptrPkt1)
    {
        System_printf ("Error: Invalid packet detected 0x%p (Expected 0x%p)\n", ptrPkt, ptrPkt1);
        return -1;
    }

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/    
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                      &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted user profiles\n");
        return -1;
    }

    /* There is only user profile which is ACTIVE */
    if (numSortedUserProfiles != 1)
    {
        System_printf ("Error: Detected %d user profiles\n", numSortedUserProfiles);
        return -1;
    }

    /* Get the report associated with the user profile. */
    Dpm_getReport (&userProfile[0], &report);

    /* Validate the report:  */
    if (testValidateReport(&report, ueId, rbId, qci, Pktlib_getPacketLen(ptrPkt2), 0) < 0)
        return -1;

    /* Record the packet delay budget reported by the DPM */
    packetDelayBudgetSnapshot[1] = report.pdb;

    /********************************************************************
     * RLC Simulation: Dequeue and process the second packet. 
     ********************************************************************/
    Dpm_dequeuePkt (ueId, rbId, &ptrPkt, &errCode);
    Dpm_rlcReport (ueId, rbId, 0, 0, 0);

    /* Ensure that this is the first packet. */
    if (ptrPkt != ptrPkt2)
    {
        System_printf ("Error: Invalid packet detected 0x%p (Expected 0x%p)\n", ptrPkt, ptrPkt2);
        return -1;
    }

    /* Cleanup the packets. */
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt1);
    Dpm_freeRLCPkt(ueId, rbId, ptrPkt2);

    /********************************************************************
     * Scheduler Simulation: Get the sorted user profiles from the DPM
     ********************************************************************/
    if (Dpm_getSortedUserProfiles (TEST_MAX_SORTED_USER_PROFILE, &userProfile[0], 
                                      &numSortedUserProfiles, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the sorted user profiles\n");
        return -1;
    }

    /* There is only user profile which is ACTIVE */
    if (numSortedUserProfiles > 01)
    {
        System_printf ("Error: Detected %d user profiles Expected 0.\n", numSortedUserProfiles);
        return -1;
    }    

    /* Display the Packet Delay Budgets from the reports on the console. */
    System_printf ("Debug: Time between the packets          is %d ticks\n", (deltaTime*1000000));
    System_printf ("Debug: Packet Delay Budget for Profile 0 is %d ticks\n", packetDelayBudgetSnapshot[0]);
    System_printf ("Debug: Packet Delay Budget for Profile 1 is %d ticks\n", packetDelayBudgetSnapshot[1]);

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the discard timer functionality
 *
 *  @param[in]  qci
 *      QoS Class Indicator for which the test is to be executed
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_discardTimer (uint8_t qci)
{
    Ti_Pkt*             ptrPkt;
    int32_t             errCode;
    uint16_t            ueId = 0;
    uint8_t             rbId = 0;
    uint16_t            index = 0;
    uint32_t            interPktDelay = 30;
    uint32_t            numPackets = 4;
    uint32_t            numQCIPendingPackets;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            timeout;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing discard timer for QCI %d InterPacketDelay %d msec\n", qci, interPktDelay);

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /* Cycle through and inject the specified number of packets into the DPM. */
    for (index = 0; index < numPackets; index++)
    {
        /* Allocate a packet from the NETFP Data Receive Heap. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 128);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
            return -1;
        }

        /* Pass the packet to the DPM driver */
        if (Dpm_enqueuePkt (ueId++, rbId++, qci, ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
            return -1;
        }

        /* Introduce a delay between each packet */
        Task_sleep (interPktDelay);
    }

    /* Age the packet: Set a very large timeout. 
     *
     *  - If there were 4 packets which were injected with a delay of interPktDelay = 50
     *  - Packet 0 was inserted @ Time T0
     *  - Packet 1 was inserted @ Time (T0 + 1*interPktDelay) = T0+50
     *  - Packet 2 was inserted @ Time (T0 + 2*interPktDelay) = T0+100
     *  - Packet 3 was inserted @ Time (T0 + 3*interPktDelay) = T0+150
     * 
     * No packet should get removed because of this. */
    timeout = (interPktDelay * 1000000) * (numPackets + 2);
    System_printf ("Debug: Setting Timeout of %d ticks (No packet should be deleted)\n", timeout);

    if (Dpm_timerExpiry (qci, timeout, &errCode) < 0)
    {
        System_printf ("Error: DPM Timer Expiry failed with Error Code %d\n", errCode);
        return -1;
    }

    /* Use the dump QCI function to determine the number of packets which are still pending
     * inside the DPM. */
    numQCIPendingPackets = Dpm_dumpQCI(qci, 0);

    /* Sanity Check: Validate and ensure that no packet was timed out. */
    if (numQCIPendingPackets != numPackets)
    {
        System_printf ("Error: QCI packet was aged out %d (Expected %d)\n", numQCIPendingPackets, numPackets);
        return -1;
    }

    /* Ok. Now lets age out all the packets by specifying a very small timeout. */
    timeout = (interPktDelay * 1000000);
    System_printf ("Debug: Setting Timeout of %d ticks (All packets should be deleted)\n", timeout);

    /* Invoke the DPM Timer Expiry. */
    if (Dpm_timerExpiry (qci, timeout, &errCode) < 0)
    {
        System_printf ("Error: DPM Timer Expiry failed with Error Code %d\n", errCode);
        return -1;
    }

    /* Use the dump QCI function to determine the number of packets which are still pending
     * inside the DPM. */
    numQCIPendingPackets = Dpm_dumpQCI(qci, 0);

    /* Sanity Check: Validate and ensure that no packet was timed out. */
    if (numQCIPendingPackets != 0)
    {
        System_printf ("Error: QCI packets were not aged out %d \n", numQCIPendingPackets);
        return -1;
    }

    /* Dump the RB and make sure that the packet has been removed */
    ueId = 0;
    rbId = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Get the number of pending packets. */
        numQCIPendingPackets = Dpm_dumpRB (ueId, rbId, 0);
        if (numQCIPendingPackets != 0)
        {
            System_printf ("Error: User Profile %d:%d has %d packets\n", ueId, rbId, numQCIPendingPackets);
            return -1;
        }

        /* Goto the next user profile. */
        ueId++;
        rbId++;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM system statistics API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_systemStatistics(void)
{
    Ti_Pkt*             ptrPkt;
    int32_t             errCode;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    uint16_t            ueId  = 0;
    uint8_t             rbId  = 0;
    uint8_t             qci   = 1;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            numBytes = 0;
    uint32_t            numPackets = 0;
    Dpm_Stats           stats;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM System Statistics \n");

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /****************************************************************************
     * Clear statistics
     *  - Ensure that all statistics are valid.
     ****************************************************************************/

    /* Clear all the statistics which have been registered till now. */
    if (Dpm_clearStats() < 0)
    {
        System_printf ("Error: DPM Clear Statistics failed\n");
        return -1;
    }

    /* Get the System Statistics */
    if (Dpm_getStats(0xFFFF, 0xFF, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Since there is nothing which has been pushed; system statistics should return all 0 */
    if ((stats.pktReceived != 0)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != 0) || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Unexpected system statistics detected (Should be 0)\n");
        return -1;
    }
    System_printf ("Debug: Reset System Statistics test passed.\n");

    /****************************************************************************
     * Enqueue multiple packets into the DPM for different user profiles and QCI
     *  - Ensure that the system statistics are valid.
     ****************************************************************************/
    for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
    {
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the number of bytes pushed */
            numBytes   = numBytes + Pktlib_getPacketLen(ptrPkt);

            /* Keep track of the number of data pushed. */
            numPackets = numPackets + 1;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }

            /* Increment the QCI. */
            qci = (qci + 1) % DPM_MAX_QCI;
            if (qci == 0)
                qci = 1;
        }
    }

    /* Get the System Statistics */
    if (Dpm_getStats(0xFFFF, 0xFF, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Validate the System statistics. */ 
    if ((stats.pktReceived != numPackets)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != numBytes)   || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Invalid System Statistics detected.\n");
        return -1;
    }

    /********************************************************************
     * Scheduler & RLC Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Head User Profile detected\n");
        return -1;
    }
   
    /* Cycle through all the user profiles; dequeue and process the packets. */
    while (1)
    {
        /* Dequeue the packet */
        if (Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);
        
        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Get the user profile to be processed. */
        userProfile = nextUserProfile;
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM QCI statistics API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_qciStatistics(void)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    int32_t             errCode;
    uint16_t            ueId  = 0;
    uint8_t             rbId  = 0;
    uint8_t             qci   = 1;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            numBytes = 0;
    uint32_t            numPackets = 0;
    Dpm_Stats           stats;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM QCI Statistics \n");

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /****************************************************************************
     * Clear statistics
     *  - Ensure that all statistics are valid.
     ****************************************************************************/

    /* Clear all the statistics which have been registered till now. */
    if (Dpm_clearStats() < 0)
    {
        System_printf ("Error: DPM Clear Statistics failed\n");
        return -1;
    }

    /* Get the QCI Statistics */
    if (Dpm_getStats(0xFFFF, 0xFF, qci, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Since there is nothing which has been pushed; QCI statistics should return all 0 */
    if ((stats.pktReceived != 0)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != 0) || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Unexpected QCI statistics detected (Should be 0)\n");
        return -1;
    }
    System_printf ("Debug: Reset QCI Statistics test passed.\n");

    /****************************************************************************
     * Enqueue multiple packets into the DPM for different user profiles 
     *  - Ensure that the system statistics are valid.
     ****************************************************************************/
    for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
    {
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
                return -1;
            }

            /* Keep track of the number of bytes pushed */
            numBytes   = numBytes + Pktlib_getPacketLen(ptrPkt);

            /* Keep track of the number of data pushed. */
            numPackets = numPackets + 1;

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return -1;
            }
        }
    }

    /* Get the QCI Statistics */
    if (Dpm_getStats(0xFFFF, 0xFF, qci, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Validate the QCI statistics. */ 
    if ((stats.pktReceived != numPackets)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != numBytes)   || (stats.bytesDropped != 0) || 
        (stats.numActiveUE != DPM_MAX_UE))
    {
        System_printf ("Error: Invalid QCI Statistics detected.\n");
        return -1;
    }

    /********************************************************************
     * Scheduler & RLC Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Head User Profile detected\n");
        return -1;
    }
   
    /* Cycle through all the user profiles; dequeue and process the packets. */
    while (1)
    {
        /* Dequeue the packet */
        if (Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);
        
        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Get the user profile to be processed. */
        userProfile = nextUserProfile;
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM User Profile statistics API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_userProfileStatistics(void)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    int32_t             errCode;
    uint16_t            ueId  = 0;
    uint8_t             rbId  = 0;
    uint8_t             qci   = 1;
    uint16_t            index = 0;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            numBytes = 0;
    Dpm_Stats           stats;
    uint32_t            numPackets = 100;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM User Profile Statistics \n");

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /****************************************************************************
     * Clear statistics
     *  - Ensure that all statistics are valid.
     ****************************************************************************/

    /* Clear all the statistics which have been registered till now. */
    if (Dpm_clearStats() < 0)
    {
        System_printf ("Error: DPM Clear Statistics failed\n");
        return -1;
    }

    /* Get the User Profile Statistics */
    if (Dpm_getStats(ueId, rbId, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Since there is nothing which has been pushed; user profile statistics should return all 0 */
    if ((stats.pktReceived != 0)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != 0) || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Unexpected User Profile statistics detected (Should be 0)\n");
        return -1;
    }
    System_printf ("Debug: Reset User Profile Statistics test passed.\n");

    /****************************************************************************
     * Enqueue multiple packets into the DPM for the same user profiles 
     *  - Ensure that the system statistics are valid.
     ****************************************************************************/
    for (index = 0; index < numPackets; index++)
    {
        /* Allocate a packet from the NETFP Data Receive Heap. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
            return -1;
        }

        /* Keep track of the number of bytes pushed */
        numBytes   = numBytes + Pktlib_getPacketLen(ptrPkt);

        /* Pass the packet to the DPM driver */
        if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
            return -1;
        }
    }

    /* Get the User Profile Statistics */
    if (Dpm_getStats(ueId, rbId, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Validate the System statistics. */ 
    if ((stats.pktReceived != numPackets)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != numBytes)   || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Invalid User Profile Statistics detected.\n");
        return -1;
    }

    /********************************************************************
     * Scheduler & RLC Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Head User Profile detected\n");
        return -1;
    }
  
    /* Cycle through and dequeue all the packets. */ 
    for (index = 0; index < numPackets; index++)
    {
        /* Dequeue the packet */
        if (Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);
    }

    /* Indicate to the DPM that there are no more pending bytes */
    Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM User statistics API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_userStatistics(void)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    int32_t             errCode;
    uint16_t            ueId  = 0;
    uint8_t             rbId  = 0;
    uint8_t             qci   = 1;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            numBytes = 0;
    Dpm_Stats           stats;
    uint32_t            numPackets = 0;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM User Statistics \n");

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /****************************************************************************
     * Clear statistics
     *  - Ensure that all statistics are valid.
     ****************************************************************************/

    /* Clear all the statistics which have been registered till now. */
    if (Dpm_clearStats() < 0)
    {
        System_printf ("Error: DPM Clear Statistics failed\n");
        return -1;
    }

    /* Get the User Statistics */
    if (Dpm_getStats(ueId, 0xFF, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Since there is nothing which has been pushed; user statistics should return all 0 */
    if ((stats.pktReceived != 0)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != 0) || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Unexpected QCI statistics detected (Should be 0)\n");
        return -1;
    }
    System_printf ("Debug: Reset User Statistics test passed.\n");

    /****************************************************************************
     * Enqueue multiple packets into the DPM for the same user but different
     * radio bearers.
     *  - Ensure that the system statistics are valid.
     ****************************************************************************/
    for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
    {
        /* Allocate a packet from the NETFP Data Receive Heap. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
            return -1;
        }

        /* Keep track of the number of bytes pushed */
        numBytes   = numBytes + Pktlib_getPacketLen(ptrPkt);

        /* Keep track of the number of data pushed. */
        numPackets = numPackets + 1;

        /* Pass the packet to the DPM driver */
        if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
            return -1;
        }
    }

    /* Get the User Statistics */
    if (Dpm_getStats(ueId, 0xFF, 0xFF, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Validate the user statistics. */ 
    if ((stats.pktReceived != numPackets)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != numBytes)   || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Invalid user statistics detected.\n");
        return -1;
    }

    /********************************************************************
     * Scheduler & RLC Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Head User Profile detected\n");
        return -1;
    }
   
    /* Cycle through all the user profiles; dequeue and process the packets. */
    while (1)
    {
        /* Dequeue the packet */
        if (Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);

        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Get the user profile to be processed. */
        userProfile = nextUserProfile;
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the DPM User per QCI statistics API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_userQCIStatistics(void)
{
    Ti_Pkt*             ptrPkt;
    Dpm_UserProfile     userProfile;
    Dpm_UserProfile     nextUserProfile;
    int32_t             errCode;
    uint16_t            ueId  = 0;
    uint8_t             rbId  = 0;
    uint8_t             qci   = 1;
    Pktlib_HeapStats    startHeapStats;
    Pktlib_HeapStats    endHeapStats;
    uint32_t            numBytes = 0;
    Dpm_Stats           stats;
    uint32_t            numPackets = 0;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: DPM User-QCI Statistics \n");

    /* Record the PKTLIB Heap Statistics before the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &startHeapStats);

    /****************************************************************************
     * Clear statistics
     *  - Ensure that all statistics are valid.
     ****************************************************************************/

    /* Clear all the statistics which have been registered till now. */
    if (Dpm_clearStats() < 0)
    {
        System_printf ("Error: DPM Clear Statistics failed\n");
        return -1;
    }

    /* Get the User-QCI Statistics */
    if (Dpm_getStats(ueId, 0xFF, qci, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Since there is nothing which has been pushed; user-qci statistics should return all 0 */
    if ((stats.pktReceived != 0)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != 0) || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Unexpected QCI statistics detected (Should be 0)\n");
        return -1;
    }
    System_printf ("Debug: Reset User QCI Statistics test passed.\n");

    /****************************************************************************
     * Enqueue multiple packets into the DPM for the same user but different
     * radio bearers.
     *  - Ensure that the system statistics are valid.
     ****************************************************************************/
    for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
    {
        /* Allocate a packet from the NETFP Data Receive Heap. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the PKTLIB Packet Management Heap\n");
            return -1;
        }

        /* Keep track of the number of bytes pushed */
        numBytes   = numBytes + Pktlib_getPacketLen(ptrPkt);

        /* Keep track of the number of data pushed. */
        numPackets = numPackets + 1;

        /* Pass the packet to the DPM driver */
        if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
            return -1;
        }
    }

    /* Get the User Statistics */
    if (Dpm_getStats(ueId, 0xFF, qci, &stats, &errCode) < 0)
    {
        System_printf ("Error: DPM Get Statistics returned Error:%d\n", errCode);
        return -1;
    }

    /* Validate the user statistics. */ 
    if ((stats.pktReceived != numPackets)   || (stats.pktDropped != 0) || 
        (stats.bytesReceived != numBytes)   || (stats.bytesDropped != 0))
    {
        System_printf ("Error: Invalid user statistics detected.\n");
        return -1;
    }

    /********************************************************************
     * Scheduler & RLC Simulation: Get the Raw reports from the DPM driver. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 0)
    {
        System_printf ("Error: DPM Empty Head User Profile detected\n");
        return -1;
    }
   
    /* Cycle through all the user profiles; dequeue and process the packets. */
    while (1)
    {
        /* Dequeue the packet */
        if (Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode) < 0)
        {
            System_printf ("Error: Dequeue Packet failed\n");
            return -1;
        }

        /* Cleanup the packet */
        Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);

        /* Indicate to the DPM that there are no more pending bytes */
        Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

        /* Get the next user profile. */
        if (Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile) == 0)
            break;

        /* Get the user profile to be processed. */
        userProfile = nextUserProfile;
    }

    /********************************************************************
     * Scheduler Simulation: Get the Raw reports from the DPM driver.
     *  - The RLC has processed all the packets so now we should get an 
     *    Empty report in the next TTI. 
     ********************************************************************/

    /* Get the head user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
    {
        System_printf ("Error: DPM User Profile is not empty (This should NOT happen)\n");
        return -1;
    }

    /* Record the PKTLIB Heap Statistics after the test executes */
    Pktlib_getHeapStats(pktMgmtHeap, &endHeapStats);

    /* Memory Leak Check: */
    if ((startHeapStats.numFreeDataPackets   != endHeapStats.numFreeDataPackets)   ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets) ||
        (startHeapStats.numPacketsinGarbage  != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: Memory Leak Detected\n");
        return -1;
    }

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to test the various DPM provided reports for
 *      various combinations
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t test_reports (void)
{
    Dpm_UserProfile     userProfile;
    uint8_t             qci;

    /**************************************************************************************
     * TEST: Testing Empty User Profile 
     **************************************************************************************/
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Debug: Testing EMPTY User Profile\n");

    /* Get the cumulative user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) != 0)
    {
        System_printf ("Error: DPM Raw Report failed (Should be an EMPTY Report)\n");
        return -1;
    }

    /* Get the fresh user profile. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_FRESH, &userProfile) != 0)
    {
        System_printf ("Error: DPM Raw Report failed (Should be an EMPTY Report)\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM reports for single user and single radio bearer
     **************************************************************************************/
    if (test_freshDPMReports(1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM reports for 8 users & 8 radio bearers
     **************************************************************************************/
    if (test_freshDPMReports(8, 4) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM reports for 64 users & 8 radio bearers
     **************************************************************************************/
    if (test_freshDPMReports(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM reports for 256 users & 8 radio bearers
     **************************************************************************************/
    if (test_freshDPMReports(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM & RLC Pending simulation for 1 users & 1 radio bearer
     **************************************************************************************/
    if (test_freshDPMRLCReports(1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM & RLC Pending simulation for 8 users & 6 radio bearer
     **************************************************************************************/
    if (test_freshDPMRLCReports(8, 6) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }
    
    /**************************************************************************************
     * TEST: Testing Fresh DPM & RLC Pending simulation for 64 users & 8 radio bearer
     **************************************************************************************/
    if (test_freshDPMRLCReports(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Fresh DPM & RLC Pending simulation for 256 users & 8 radio bearer
     **************************************************************************************/
    if (test_freshDPMRLCReports(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM reports for 1 user & 1 radio bearer
     **************************************************************************************/
    if (test_cumulativeDPMReports(1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM reports for 8 user & 3 radio bearer
     **************************************************************************************/
    if (test_cumulativeDPMReports(8, 3) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM reports for 64 user & 8 radio bearer
     **************************************************************************************/
    if (test_cumulativeDPMReports(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM reports for 256 user & 8 radio bearer
     **************************************************************************************/
    if (test_cumulativeDPMReports(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM & RLC reports for 1 user & 1 radio bearer
     **************************************************************************************/
    if (test_cumulativeRLCReports(1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM & RLC reports for 8 user & 5 radio bearer
     **************************************************************************************/
    if (test_cumulativeRLCReports(8, 5) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Cumulative DPM & RLC reports for 64 user & 8 radio bearer
     **************************************************************************************/
    if (test_cumulativeRLCReports(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }
    
    /**************************************************************************************
     * TEST: Testing Cumulative DPM & RLC reports for 256 user & 8 radio bearer
     **************************************************************************************/
    if (test_cumulativeRLCReports(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 1 user & 1 radio bearer
     **************************************************************************************/
    if (test_sortedDPMReport(1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 2 user & 3 radio bearer
     **************************************************************************************/
    if (test_sortedDPMReport(2, 3) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 8 user & 4 radio bearer
     **************************************************************************************/
    if (test_sortedDPMReport(8, 4) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 64 user & 8 radio bearer
     **************************************************************************************/
    if (test_sortedDPMReport(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 256 user & 8 radio bearer
     **************************************************************************************/
    if (test_sortedDPMReport(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 1 user & 1 radio bearer
     **************************************************************************************/
    if (test_sortedTraversalReports (1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 2 user & 3 radio bearer
     **************************************************************************************/
    if (test_sortedTraversalReports(2, 3) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 8 user & 4 radio bearer
     **************************************************************************************/
    if (test_sortedTraversalReports(8, 4) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 64 user & 8 radio bearer
     **************************************************************************************/
    if (test_sortedTraversalReports(64, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Testing Sorted DPM reports for 256 user & 8 radio bearer
     **************************************************************************************/
    if (test_sortedTraversalReports(256, 8) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }    

    /**************************************************************************************
     * TEST: Fresh Reports with 1 user & 1 radio bearer and 1 user disabled
     **************************************************************************************/
    if (testDisabledReports (Dpm_ReportType_FRESH, 1, 1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Fresh Reports with 64 user & 8 radio bearer and 16 users disabled
     **************************************************************************************/
    if (testDisabledReports (Dpm_ReportType_FRESH, 64, 8, 16) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Cumulative Reports with 1 user & 1 radio bearer and 1 user disabled
     **************************************************************************************/
    if (testDisabledReports (Dpm_ReportType_CUMULATIVE, 1, 1, 1) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Cumulative Reports with 64 user & 8 radio bearer and 50 users disabled
     **************************************************************************************/
    if (testDisabledReports (Dpm_ReportType_CUMULATIVE, 64, 8, 50) < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: Discard Timer Functionality.
     **************************************************************************************/
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {    
        if (test_discardTimer (qci) < 0)
        {
            System_printf ("Error: Test Failed\n");
            return -1;
        }
    }

    /**************************************************************************************
     * TEST: Packet Delay Budget for QCI (Using sorted User profiles)
     **************************************************************************************/
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {
        if (test_PDBUsingSortedReports(qci) < 0)
        {
            System_printf ("Error: Test Failed\n");
            return -1;
        }
    }

    /**************************************************************************************
     * TEST: Packet Delay Budget for QCI (Using raw user profiles)
     **************************************************************************************/
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {
        if (test_PDBUsingUserProfiles(qci) < 0)
        {
            System_printf ("Error: Test Failed\n");
            return -1;
        }
    }

    /**************************************************************************************
     * TEST: Packet Delay Budget with multiple packets using raw user profiles.
     **************************************************************************************/
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {
        if (test_multiplePacketPDBRawProfiles(qci) < 0)
        {
            System_printf ("Error: Test Failed\n");
            return -1;
        }
    }

    /**************************************************************************************
     * TEST: Packet Delay Budget with multiple packets using sorted reports
     **************************************************************************************/
    for (qci = 1; qci < DPM_MAX_QCI; qci++)
    {
        if (test_multiplePacketPDBSortedReports(qci) < 0)
        {
            System_printf ("Error: Test Failed\n");
            return -1;
        }
    }

    /**************************************************************************************
     * TEST: System Statistics 
     **************************************************************************************/
    if (test_systemStatistics() < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: QCI Statistics 
     **************************************************************************************/
    if (test_qciStatistics() < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }
    
    /**************************************************************************************
     * TEST: User Profile Statistics 
     **************************************************************************************/
    if (test_userProfileStatistics() < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: User Statistics 
     **************************************************************************************/
    if (test_userStatistics() < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }

    /**************************************************************************************
     * TEST: User QCI Statistics 
     **************************************************************************************/
    if (test_userQCIStatistics() < 0)
    {
        System_printf ("Error: Test Failed\n");
        return -1;
    }
    
    /* Report tests passed. */
    return 0;
}

