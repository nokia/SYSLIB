/*
 *   @file  stress_writer.c
 *
 *   @brief   
 *      Writer Test: The file implements the functionality which stress
 *      tests the virtual channel over a PHYSICAL accumulated channel and
 *      collects the benchmarking information.
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
#include <ti/sysbios/knl/Clock.h>
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
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>

/* Test specific include files */
#include "test_config.h"
#include "benchmark.h"

/**********************************************************************
 ************************** Writer Globals ****************************
 **********************************************************************/

/* Global Benchmark information which records the time taken to receive
 * a message. */
BENCHMARK_INIT(Msgcom_putMessage)

/* Global list of virtual channel handles which are being tested. */
MsgCom_ChHandle        gVirtualChannelHandle[TEST_MAX_VIRTUAL_CHANNELS];

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* MSGCOM Instance handle. */
extern Msgcom_InstHandle    appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/* Buffer Writeback Functions: */
extern void appWritebackBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Writer Functions **************************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      Initialization of the MSGCOM channels for the stress test
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t writerStressInitChannels (void)
{
    char        virtualChannelName[MSG_COM_MAX_CHANNEL_LEN];
    int32_t     index;
    int32_t     errCode;

    /* Find the virtual channels which have been created. */
    for (index = 0; index < TEST_MAX_VIRTUAL_CHANNELS; index++)
    {
        /* Create a unique channel name. */
        sprintf (virtualChannelName, "StressTest-Virtual-%d", index);

        /* Create a virtual channel. */
        while (1)
        {
            gVirtualChannelHandle[index] = Msgcom_find(appMsgcomInstanceHandle, &virtualChannelName[0], &errCode);
            if (gVirtualChannelHandle[index] != NULL)
                break;
        }
    }
    /* Channels have been found */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Writer Stress Test which sends messages over the channels and collects
 *      the benchmarking information.
 *
 *  @retval
 *      Not Applicable.
 */
static void writerStressTest(void)
{
    Ti_Pkt*               ptrMessage;
    Ti_Pkt*               ptrEndTestMessage;
    uint32_t*             ptrDataBuffer;
    uint32_t              dataLen = 128;
    uint32_t              messageId[TEST_MAX_VIRTUAL_CHANNELS];
    uint32_t              virtualChannelIndex = 0;
    uint32_t              numOOMError = 0;
    uint32_t              numMessagesTxed = 0;
    Pktlib_HeapHandle     msgcomDataHeap;
    int32_t               errCode;

    /* Get the MSGCOM Data Heap: This is the heap which is used to allocate messages 
     * exchanged between the reader and writer. */
    msgcomDataHeap = Pktlib_findHeapByName(appPktlibInstanceHandle, "Message_DataHeap", &errCode);
    if (msgcomDataHeap == NULL)
    {
        System_printf ("FATAL Error: Unable to find the MSGCOM Data Heap [Error code %d]\n", errCode);
        return;
    }

    /* Allocate an end of test message and always keep this with us; so that we can always
     * terminate the test when we are done with the MAX Iterations. */
    ptrEndTestMessage = Pktlib_allocPacket (appPktlibInstanceHandle, msgcomDataHeap, dataLen);
    if (ptrEndTestMessage == NULL)
        return;

    /* Initialize the message identifier */
    memset ((void*)&messageId, 0, sizeof(messageId));

    /* Sleep for some time to make sure that the reader core is operational before we
     * start pushing messages */
    Task_sleep(1000);
    System_printf ("Debug: Writer core starting STRESS Test for %u packets\n", TEST_MAX_ITERATIONS);

    /* Execute the test */
    while (1)
    {
        /* Have we sent all the messages we are allowed to send */
        if ((numMessagesTxed + numOOMError) >= TEST_MAX_ITERATIONS)
            break;

        /* Allocate a message */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, msgcomDataHeap, dataLen);
        if (ptrMessage == NULL)
        {
            numOOMError++;
            continue;
        }

        /* Get the data buffer */
        Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

#if 0
        /* Populate the data buffer */
        *ptrDataBuffer = 0xAA;
#else
        /* Message identifiers are recorded for each virtual channel */
        *ptrDataBuffer = messageId[virtualChannelIndex]++;
#endif

        /* Writeback the cache contents */
        appWritebackBuffer(ptrDataBuffer, dataLen);

        /* Send out the message */
        BENCHMARK_START(Msgcom_putMessage);
        Msgcom_putMessage (gVirtualChannelHandle[virtualChannelIndex], ptrMessage);
        BENCHMARK_END(Msgcom_putMessage);
        BENCHMARK_UPDATE(Msgcom_putMessage);

        /* Round robin and send messages to all virtual channels. */
        virtualChannelIndex = (virtualChannelIndex + 1) % TEST_MAX_VIRTUAL_CHANNELS;

        /* Increment the number of messages transmitted. */
        numMessagesTxed++;

        /* Increase the size of the data buffer */
        dataLen = (dataLen + 1);
        if (dataLen == 512)
            dataLen = 128;

        /* After a sending a burst of TEST_MAX_BURST packets; we will sleep for some time. */
        if ((numMessagesTxed % TEST_MAX_BURST) == 0)
            Task_sleep(2);
    }

    /* Display the writer benchmark information */
    System_printf ("*******************************************************\n");
    System_printf ("Writer Benchmark Information: \n");
    BENCHMARK_DISPLAY(Msgcom_putMessage);
    System_printf ("Out of memory errors:  %d\n", numOOMError);
    System_printf ("*******************************************************\n");

    /* Sleep for some time; before we send the end of life messsage to the reader 
     * This will allow the reader time to process all the other data messages and 
     * ensure that we dont drop the end of life message. */
    Task_sleep(100);

    /* Send the end of test message to all virtual channel 0. This will indicate to the reader
     * that all the messages have been sent. */
    Pktlib_getDataBuffer(ptrEndTestMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

#if 0
    /* Populate the data buffer */
    *ptrDataBuffer       = 0xDE;
    *(ptrDataBuffer + 1) = 0xAD;
#else
    /* Populate the data buffer with the test ending message */
    *ptrDataBuffer = 0xDEADDEAD;
#endif

    /* Writeback the cache contents */
    appWritebackBuffer(ptrDataBuffer, dataLen);
    Msgcom_putMessage (gVirtualChannelHandle[0], ptrEndTestMessage);
    return;
}

/**
 *  @b Description
 *  @n  
 *      Writer Stress Test for virtual accumulated channels.
 *
 *  @retval
 *      Not Applicable.
 */
void writerStressTask(UArg arg0, UArg arg1)
{
    /* Initialize the MSGCOM channels which are to be tested. */
    if (writerStressInitChannels() < 0)
        return;

    /* Perform the test. */
    writerStressTest();
}


