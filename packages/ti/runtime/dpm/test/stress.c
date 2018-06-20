/*
 *   @file  stress.c
 *
 *   @brief   
 *      Stress Test code for the DPM Module.
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
#include <ti/drv/rm/rm.h>
#include <ti/runtime/dpm/dpm.h>

/**********************************************************************
 ************************* Unit Test Definitions **********************
 **********************************************************************/

/* Maximum number of sorted profiles. */
#define DPM_STRESS_MAX_SORTED_PROFILE           8

/* The stress test will display information on the console after the 
 * specified number of iterations. */
#define DPM_STRESS_TEST_DISPLAY                 10000

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Heap for receiving downlink packets. */
extern Pktlib_HeapHandle   pktMgmtHeap;

/* PKTLIB Instance handle */
extern Pktlib_InstHandle   appPktlibInstanceHandle;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Statistics */
uint32_t gNumPktGenerated = 0;
uint32_t gNumUserProfilesScheduled = 0;

/* Number of Packets enqueued tracker */
uint16_t    numPktEnqueued[32];
uint16_t    numPktEnqueuedIndex = 0;    

/**********************************************************************
 ************************* Stress Test Functions **********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Packet Generator Task which will generate and push packets into
 *      the DPM
 *
 *  @retval
 *      Not Applicable
 */
static void PacketGeneratorTask(UArg arg0, UArg arg1)
{
    uint16_t    ueId;
    uint8_t     rbId;
    uint8_t     qci;
    uint32_t    numPacket;
    Ti_Pkt*     ptrPkt;
    int32_t     errCode;
    uint32_t    index;
    uint32_t    lastDisplayPktCount = 0;

    /* Execute the Packet Generator Task. */
    System_printf ("Debug: Packet Generator Started\n");
    Task_sleep (5);

    /* Initialize the user & radio bearer. */
    ueId = 0;
    rbId = 0;
    qci  = 1;    

    /* Execute forever. */
    while (1)
    {
        /* Generate a unique number of packets which are to be pushed. */
        numPacket = (TSCL & 0x3F) % 32;

        /* Track this for debugging purposes. */
        numPktEnqueued[numPktEnqueuedIndex] = numPacket;
        numPktEnqueuedIndex = (numPktEnqueuedIndex + 1) % 32;

        /* Generate packets and push them into the DPM */
        for (index = 0; index < numPacket; index++)
        {
            /* Allocate a packet from the NETFP Data Receive Heap. */
            ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, pktMgmtHeap, 10);
            if (ptrPkt == NULL)
            {
                System_printf ("Error: Unable to allocate a packet from the NETFP Data Receive Heap\n");
                return;
            }

            /* Pass the packet to the DPM driver */
            if (Dpm_enqueuePkt (ueId, rbId, qci, ptrPkt, &errCode) < 0)
            {
                System_printf ("Error: DPM Receive Packet failed with Error %d\n", errCode);
                return;
            }

            /* Increment the number of packets generated. */
            gNumPktGenerated++;

            /* Increment the user & radio bearer. */
            rbId = (rbId + 1) % DPM_MAX_RB;
            if (rbId == 0)
                ueId = (ueId + 1) % DPM_MAX_UE;

            /* Increment the QCI */
            qci  = (qci + 1)  % DPM_MAX_QCI;
            if (qci == 0)
                qci = 1;
        }

        /* Sleep for some time */
        Task_sleep (1);

        /* Dump out periodic information on the console */
        if ((gNumPktGenerated / DPM_STRESS_TEST_DISPLAY) > lastDisplayPktCount)
        {
            System_printf ("Debug: Packet Generator transmitted %d packets\n", gNumPktGenerated);
            lastDisplayPktCount = gNumPktGenerated / DPM_STRESS_TEST_DISPLAY;
        }
    }
}

/**
 *  @b Description
 *  @n  
 *      Scheduler Task which will request the sorted profiles from the 
 *      DPM and then pass them to the RLC where the packet will be
 *      dequeued and processed.
 *
 *  @retval
 *      Not Applicable
 */
static void SchedulerTask(UArg arg0, UArg arg1)
{
    Dpm_UserProfile     userProfile[DPM_STRESS_MAX_SORTED_PROFILE];
    int32_t             errCode;
    Ti_Pkt*             ptrPkt;
    uint32_t            numSortedUserProfiles;
    uint32_t            index;

    /* Execute forever. */
    while (1)
    {
        /* Get the sorted user profiles from the DPM. */
        if (Dpm_getSortedUserProfiles (DPM_STRESS_MAX_SORTED_PROFILE, &userProfile[0], 
                                       &numSortedUserProfiles, &errCode) < 0)
        {
            System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
            return;
        }

        while (1)
        {
            /* Are there any sorted reports available? */
            if (numSortedUserProfiles == 0)
                break;

            /* Increment the number of user profiles scheduled. */
            gNumUserProfilesScheduled = gNumUserProfilesScheduled + numSortedUserProfiles;

            /* Process the sorted user profiles. */
            for (index = 0; index < numSortedUserProfiles; index++)
            {
                /* Remove the packet from the specific user & radio bearer */
                if (Dpm_dequeuePkt (userProfile[index].ueId, userProfile[index].rbId, &ptrPkt, &errCode) < 0)
                {
                    System_printf ("Error: Dequeue Packet failed\n");
                    return;
                }
    
                /* Indicate to the DPM that there are no more pending bytes */
                Dpm_rlcReport (userProfile[index].ueId, userProfile[index].rbId, 0, 0, 0);

                /* Cleanup the packet */
                Dpm_freeRLCPkt(userProfile[index].ueId, userProfile[index].rbId, ptrPkt);
            }


            /* Get the next set of sorted user profiles. */
            if (Dpm_getNextSortedUserProfiles (DPM_STRESS_MAX_SORTED_PROFILE, &userProfile[0], 
                                               &numSortedUserProfiles, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the sorted report from the DPM %d\n", errCode);
                return;
            }
        }
    }
}

/**
 *  @b Description
 *  @n  
 *      Stress Test code for the DPM.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t stress_test (void)
{
    Task_Params         taskParams;

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------------------------\n");
    System_printf ("Stress Test\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);

    /* Start the packet generator which has the HIGHEST priority since packets can arrive 
     * in interrupt context. */
    taskParams.priority  = 7;
    taskParams.stackSize = 16384;
    Task_create(PacketGeneratorTask, &taskParams, NULL);

    /* Start the Scheduler which */
    taskParams.priority  = 6;
    taskParams.stackSize = 16384;
    Task_create(SchedulerTask, &taskParams, NULL);

    return 0;
}

