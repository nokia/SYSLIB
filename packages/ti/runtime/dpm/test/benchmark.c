/*
 *   @file  benchmark.c
 *
 *   @brief   
 *      Benchmarking code which is used to benchmark the DPM API
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
 *
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

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>

/* Packet Library Include Files. */
#include <ti/runtime/pktlib/pktlib.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* DPM Include Files. */
#include <ti/runtime/dpm/dpm.h>

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

/* Maximum number of format reports. */
#define TEST_MAX_FORMAT_REPORT      8

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
 *      The function is used to benchmark empty raw reports.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t benchmark_EmptyReport (void)
{
    Dpm_UserProfile userProfile;
    uint32_t        timeStart;
    uint32_t        timeEnd;
    int32_t         retVal;

    /**************************************************************************************
     * TEST: Benchmark Empty Reports. 
     **************************************************************************************/
    System_printf ("--------------------------------------------------------------\n");
    System_printf ("Debug: Benchmarking EMPTY Report Generation\n");

    /* Get the RAW Report head. */
    timeStart = TSCL;
    retVal = Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile);
    timeEnd   = TSCL;
    if (retVal != 0)
    {
        System_printf ("Error: DPM Raw Report failed (Should be an EMPTY Report)\n");
        return -1;
    }
    System_printf ("Debug: Empty Report generation took %d ticks\n", (timeEnd - timeStart));
    return 0;        
}

/**
 *  @b Description
 *  @n  
 *      The function is used to benchmark raw reports for the specified number
 *      of users.
 *
 *  @param[in]  numUE
 *      Number of UE's 
 *  @param[in]  numRB
 *      Number of radio bearers 
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t benchmark_RawReport (uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*         ptrPkt;
    Dpm_UserProfile userProfile;
    Dpm_UserProfile nextUserProfile;
    Dpm_Report      report;
    uint32_t        timeStart;
    uint32_t        timeEnd;
    int32_t         retVal;
    int32_t         errCode;
    uint16_t        ueId;
    uint8_t         rbId;
    uint8_t         qci = 3;
    uint32_t        totalTimeTaken = 0;
    uint32_t        numIterations = 0;
    uint32_t        min = 0xFFFFFFFF;
    uint32_t        max = 0;

    /* Debug Message: */
	System_printf ("--------------------------------------------------------------\n");
	System_printf ("Benchmarking: Raw Reports for %d users & %d RB.\n", numUE, numRB);

    /* Cycle through and allocate packets for all the specified users */
    for (ueId = 0; ueId < numUE; ueId++)
    {
        /* Cycle through all the RB for the specific user. */
        for (rbId = 0; rbId < numRB; rbId++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
        	ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 128);
        	if (ptrPkt == NULL)
        	{
		        System_printf ("Error: Unable to allocate a packet from the NETFP Data Receive Heap\n");
		        return -1;
            }

        	/* Pass the packet to the DPM driver. */
        	retVal = Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode);
        	if (retVal < 0)
        	{
		        System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
        		return -1;
        	}
        }
    }

	/* Get the head user profile. */
	timeStart = TSCL;
	retVal = Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile);
	timeEnd   = TSCL;
	if (retVal != 1)
	{
		System_printf ("Error: DPM Raw Report failed\n");
		return -1;
	}
	System_printf ("Debug: Dpm_getHeadUserProfile Head took %d ticks\n", (timeEnd - timeStart));

    /* Get the report for the user profile. */
    timeStart = TSCL;
    Dpm_getReport (&userProfile, &report);
    timeEnd   = TSCL;
    System_printf ("Debug: Dpm_getReport took %d ticks\n", (timeEnd - timeStart));

    while (1)
    {
    	/* RLC Simulation: Dequeue the packet. */
	    retVal = Dpm_dequeuePkt (userProfile.ueId, userProfile.rbId, &ptrPkt, &errCode);
	    if (retVal < 0)
    	{
	    	System_printf ("Error: Dequeue Packet failed\n");
		    return -1;
    	}

    	/* RLC Simulation: Indicate to the DPM that there are no more pending bytes */
    	Dpm_rlcReport (userProfile.ueId, userProfile.rbId, 0, 0, 0);

    	/* Cleanup the dequeued packet. */
	    Dpm_freeRLCPkt(userProfile.ueId, userProfile.rbId, ptrPkt);

    	/* Get the next element in the cumulative report. */
    	timeStart = TSCL;
	    retVal = Dpm_getNextUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile, &nextUserProfile);
    	timeEnd = TSCL;

        /* Keep track of the total execution time. */
        totalTimeTaken = totalTimeTaken + (timeEnd - timeStart);
        numIterations++;

        /* Keep track of the minimum and maximum */
        min = _minu4(min, (timeEnd - timeStart));
        max = _maxu4(max, (timeEnd - timeStart));

        /* Are all the reports done with? */
	    if (retVal == 0)
            break;

        /* Store the next report & continue the loop. */
        userProfile = nextUserProfile;
    }

    /* Display the Raw report benchmarking information. */
	System_printf ("Debug: Dpm_getNextUserProfile -> Min %d Max %d and Avg %d ticks over %d iterations\n", 
                    min, max, totalTimeTaken/numIterations, numIterations);

    /* At the end of the function there should be no more reports. */
    if (Dpm_getHeadUserProfile (Dpm_ReportType_CUMULATIVE, &userProfile) == 1)
        return -1;

    /* Test Passed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to benchmark the sorted report
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
static int32_t benchmark_SortedReports (uint16_t numUE, uint8_t numRB)
{
    Ti_Pkt*             ptrPkt;
    uint32_t            timeStart;
    uint32_t            timeEnd;
    int32_t             retVal;
    int32_t             errCode;
	Dpm_UserProfile     userProfiles[TEST_MAX_FORMAT_REPORT];
	uint32_t            numSortedProfiles;
	uint16_t            ueId = 0;
	uint8_t             rbId = 0;
	uint8_t             qci = 5;
	uint8_t             index;
    uint32_t            totalTimeTaken = 0;
    uint32_t            numIterations = 0;
    uint32_t            min = 0xFFFFFFFF;
    uint32_t            max = 0;

	System_printf ("--------------------------------------------------------------\n");
	System_printf ("Benchmarking: Sorted reports for %d users & %d RB\n", numUE, numRB);

	/* Cycle through for both users. */
	for (ueId = 0; ueId < numUE; ueId++)
	{
		/* Cycle through for all radio bearers */
		for (rbId = 0; rbId < numRB; rbId++)
		{
			/* Allocate a packet from the NETFP Data Receive Heap. */
			ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 128);
			if (ptrPkt == NULL)
			{
				System_printf ("Error: Unable to allocate a packet from the NETFP Data Receive Heap\n");
				return -1;
			}

			/* Pass the packet to the DPM driver. */
			retVal = Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode);
			if (retVal < 0)
			{
				System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
				return -1;
			}
		}
	}

	/********************************************************************
	 * Scheduler Simulation: Cycle through and process all the reports.
	 ********************************************************************/
	while (1)
	{
		/* Get the sorted report. */
		timeStart = TSCL;
		retVal = Dpm_getSortedUserProfiles (TEST_MAX_FORMAT_REPORT, &userProfiles[0], &numSortedProfiles, &errCode);
		timeEnd   = TSCL;
		if (retVal < 0)
		{
			System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
			return -1;
		}

        /* Keep track of the total time taken. */
        totalTimeTaken = totalTimeTaken + (timeEnd - timeStart);
        numIterations++;

        /* Keep track of the minimum and maximum */
        min = _minu4(min, (timeEnd - timeStart));
        max = _maxu4(max, (timeEnd - timeStart));

		/* Are we done with all the profiles. */
		if (numSortedProfiles == 0)
			break;

		/*************************************************************************
		 * RLC Simulation: Dequeue & process all the reported packets
		 * from the DPM Module.
		 *************************************************************************/
		for (index = 0; index < numSortedProfiles; index++)
		{
			/* Remove the packet from the specific user & radio bearer */
			retVal = Dpm_dequeuePkt (userProfiles[index].ueId, userProfiles[index].rbId, &ptrPkt, &errCode);
			if (retVal < 0)
			{
				System_printf ("Error: Dequeue Packet failed\n");
				return -1;
			}

			/* Indicate to the DPM that there are no more pending bytes */
			Dpm_rlcReport (userProfiles[index].ueId, userProfiles[index].rbId, 0, 0, 0);

			/* Cleanup the packet */
			Dpm_freeRLCPkt(userProfiles[index].ueId, userProfiles[index].rbId, ptrPkt);
		}
	}

    /* Debug Message: */
    System_printf ("Debug: Dpm_getSortedUserProfiles -> Min %d Max %d and Avg %d ticks over %d iterations\n", 
                    min, max, totalTimeTaken/numIterations, numIterations);
    
    /* Benchmarking was complete. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to benchmark the DPM module.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t benchmarkDpm (void)
{
    /**************************************************************************************
     * TEST: Benchmarking an empty report
     **************************************************************************************/
    if (benchmark_EmptyReport() < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the raw report for 1 user & 1 RB
     **************************************************************************************/
    if (benchmark_RawReport(1, 1) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the raw report for 64 users & 8 RB
     **************************************************************************************/
    if (benchmark_RawReport(64, 8) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the raw report for 256 users & 8 RB
     **************************************************************************************/
    if (benchmark_RawReport(256, 8) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the sorted report for 1 user & 1 RB
     **************************************************************************************/
    if (benchmark_SortedReports(1, 1) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the sorted report for 32 users & 2 RB
     **************************************************************************************/
    if (benchmark_SortedReports(32, 2) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the sorted report for 64 users & 2 RB
     **************************************************************************************/
    if (benchmark_SortedReports(64, 2) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the sorted report for 64 users & 8 RB
     **************************************************************************************/
    if (benchmark_SortedReports(64, 8) < 0)
        return -1;

    /**************************************************************************************
     * TEST: Benchmarking the sorted report for 256 users & 8 RB
     **************************************************************************************/
    if (benchmark_SortedReports(256, 8) < 0)
        return -1;

    /* Benchmarking complete. */
    return 0;
}

