/*
 *   @file  largepacket.c
 *
 *   @brief   
 *      The file implements the test code for large packets. Large
 *      packets are defined as packets > NETFP_PASS_MAX_BUFFER_SIZE. 
 *      Fragmented packets are received and reassembled by the NETFP 
 *      module. After reassembly if the packet size exceeds the limit
 *      these packets are passed to the Large packet channel. Since
 *      the NETCP cannot handle these packets; there is no classification
 *      done on these packets. This implies that the large channel will
 *      receive all *large* packets irrespective of protocol, port etc. 
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

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Counter for number of large packets received */
uint32_t        gNumLargePktRxed    = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

/* NETFP Large Channel: */
extern MsgCom_ChHandle      netfpLargeChannel;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_largePacketTask(UArg arg0, UArg arg1)
{
    Ti_Pkt*     ptrRxPkt;

    /* Debug Message: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Large Packets can be received on any port\n");
    System_printf ("-------------------------------------------------------------\n");

    /**********************************************************************************
     *  This is the main loop where the task waits for data to arrive on the large 
     *  channel.
     *********************************************************************************/
    while (1)
    {
        /* Get the message from the large packet channel: */
        Msgcom_getMessage (netfpLargeChannel, &ptrRxPkt);

        /* Did we get a large packet? */
        if (ptrRxPkt != NULL)
        {
            /* Debug Message: */
            System_printf ("Debug: Received Large Packet of %d bytes [%d chained packets]\n", 
                            Pktlib_getPacketLen(ptrRxPkt), Pktlib_packetBufferCount(ptrRxPkt));

            /* Sanity Check: Ensure that only packets > NETFP_PASS_MAX_BUFFER_SIZE are passed to the
             * large channel */
            if (Pktlib_getPacketLen(ptrRxPkt) < NETFP_PASS_MAX_BUFFER_SIZE)
                System_printf ("Error: Large Packet Violation detected\n");

            /* Increment the statistics. */
            gNumLargePktRxed++;
            Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
        }
        /* Relinquish time slice and allow other tasks to execute */
        Task_sleep(10);
    }
}

/**
 *  @b Description
 *  @n  
 *      This is an exported function which is used to display the statistics related
 *      to the IPv4 Socket Tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_largeIPv4DisplayStats(void)
{
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Large Packet::::\n");

    /* Display the module statistics. */
    System_printf ("Number of Large Pkt Received: %d\n", gNumLargePktRxed);
    return;
}

