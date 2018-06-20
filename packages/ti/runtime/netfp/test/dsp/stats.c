/*
 *   @file  stats.c
 *
 *   @brief
 *      The file implements the statistics & benchmark collection for
 *      the unit tests
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

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/platform/platform.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/* Benchmarking Include Files */
#include <benchmark.h>

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* Global Counter to display the statistics for all tests. Set this
 * to a non-zero value to display the statistics. */
uint32_t    gDisplayStats = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* Exported Variable: This is the test selection variable which indicates
 * the the test which is executing. */
extern uint32_t testSelection;

/* Exported API: Exported statistics function from the sub-modules */
extern void Test_socketIPv4DisplayStats(void);
extern void Test_largeIPv4DisplayStats(void);
extern void Test_pktTransformDisplayStats(void);
extern void Test_displayDRBEncodeStats(void);
extern void Test_displayDRBDecodeStats(void);
extern void Test_displayDrbFpReestablishmentStats(void);
extern void Test_displayDrbSpReestablishmentStats(void);
extern void Test_displayDrbSourceHOStats(void);
extern void Test_displayDrbFPTargetHOStats(void);
extern void Test_displayDrbSPTargetHOStats(void);

/* Heap used for sending & receiving data */
extern Pktlib_HeapHandle   netfp10KReceiveHeap;
extern Pktlib_HeapHandle   mtuReceiveHeap;

/**********************************************************************
 ************************* Stats Functions ****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which display the heap statistics.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_statsDisplayHeapStats(void)
{
    Pktlib_HeapStats    heapStats;

    /* Display the heap statistics being used. The tests have been configured such that
     * the receive flow uses only one heap which has been configured for monitoring
     * the threshold */
    Pktlib_getHeapStats(netfp10KReceiveHeap, &heapStats);

    /* Display the heap statistics. */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Heap Statistics for 10K Receive Heap 0x%x\n", netfp10KReceiveHeap);
    System_printf ("Free Data Buffer Packets              :%d\n", heapStats.numFreeDataPackets);
    System_printf ("Data Buffer Threshold Status          :%d\n", heapStats.dataBufferThresholdStatus);
    System_printf ("Data Buffer Starvation Counter        :%d\n", heapStats.dataBufferStarvationCounter);
    System_printf ("Zero Data Buffer Packets              :%d\n", heapStats.numZeroBufferPackets);
    System_printf ("Zero Data Buffer Threshold Status     :%d\n", heapStats.zeroDataBufferThresholdStatus);
    System_printf ("Zero Data Buffer Starvation Counter   :%d\n", heapStats.zeroDataBufferStarvationCounter);
    System_printf ("Number of Packets in Garbage          :%d\n", heapStats.numPacketsinGarbage);

    /* Display the heap statistics being used. The tests have been configured such that
     * the receive flow uses only one heap which has been configured for monitoring
     * the threshold */
    Pktlib_getHeapStats(mtuReceiveHeap, &heapStats);

    /* Display the heap statistics. */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Heap Statistics for the MTU Heap 0x%x\n", mtuReceiveHeap);
    System_printf ("Free Data Buffer Packets              :%d\n", heapStats.numFreeDataPackets);
    System_printf ("Data Buffer Threshold Status          :%d\n", heapStats.dataBufferThresholdStatus);
    System_printf ("Data Buffer Starvation Counter        :%d\n", heapStats.dataBufferStarvationCounter);
    System_printf ("Zero Data Buffer Packets              :%d\n", heapStats.numZeroBufferPackets);
    System_printf ("Zero Data Buffer Threshold Status     :%d\n", heapStats.zeroDataBufferThresholdStatus);
    System_printf ("Zero Data Buffer Starvation Counter   :%d\n", heapStats.zeroDataBufferStarvationCounter);
    System_printf ("Number of Packets in Garbage          :%d\n", heapStats.numPacketsinGarbage);
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the statistics task
 *
 *  @retval
 *      Not Applicable.
 */
void Test_statsTask(UArg arg0, UArg arg1)
{
    System_printf ("Debug: Statistics module has been launched.\n");

    /* Loop around forever. */
    while (1)
    {
        /* Is there a request to display the statistics? */
        if (gDisplayStats == 0)
        {
            /* NO. Might as well sleep for some time; let the others execute
             * The task is low priority anyway; but still we dont need to hog all
             * the time. */
            Task_sleep(5);
            continue;
        }

        /* Statistics display request has been received: */
        System_printf ("-------------------------------------------------------------\n");

        /* Display the heap statistics. */
        Test_statsDisplayHeapStats();

        /* Display the statistics for each sub module test which is being executed.
         * This is dependent on the type of test being executed. */
        switch (testSelection)
        {
            case 1:
            {
                /* Display the statistics for the socket test. */
                Test_socketIPv4DisplayStats();
                break;
            }
            case 3:
            {
                /* Display the statistics for the large packet test. */
                Test_largeIPv4DisplayStats();
                break;
            }
            case 4:
            {
                /* Display the statistics for the packet transform test. */
                Test_pktTransformDisplayStats();
                break;
            }
            case 7:
            {
                /* Display the statistics for the packet transform test. */
                Test_displayDRBEncodeStats();
                break;
            }
            case 11:
            {
                Test_displayDRBDecodeStats();
                break;
            }
            case 12:
            {
                /* Display the statistics for the packet transform test. */
                Test_pktTransformDisplayStats();
                break;
            }
            case 14:
            {
            	Test_displayDrbFpReestablishmentStats();
            	break;
            }
            case 15:
            {
            	Test_displayDrbSpReestablishmentStats();
            	break;
            }
            case 17:
            {
            	Test_displayDrbSourceHOStats();
            	break;
            }
            case 18:
            {
            	Test_displayDrbFPTargetHOStats();
            	break;
            }
            case 19:
            {
            	Test_displayDrbSPTargetHOStats();
            	break;
            }
            case 20:
            {
                /* Basic Reestablishment Test: Nothing to do here */
                break;
            }
            default:
            {
                System_printf ("Warning: Test does not expose additional statistics.\n");
                break;
            }
        }
        /* All the statistics have been displayed. We display once and keep quiet. */
        gDisplayStats = 0;
        System_printf ("-------------------------------------------------------------\n");
    }
}

