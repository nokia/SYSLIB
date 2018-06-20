/**
 *   @file  reader.c
 *
 *   @brief
 *      Reader Unit Test Code.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <signal.h>

/* MCSDK Include files */
#include <ti/runtime/hplib/hplib.h>
#include <ti/runtime/hplib/hplib_util.h>
#include <ti/csl/csl_pllc.h>
#include <ti/csl/csl_tmr.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/uintc/uintc.h>

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* Global PKTLIB Heap Handle: */
extern Pktlib_HeapHandle       testHeapHandle;

/* Global Name database Handle: */
extern Name_DBHandle           globalNameDatabaseHandle;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
extern Pktlib_InstHandle       appPktlibInstanceHandle;

/* Name Client Handle: */
extern Name_ClientHandle        nameClientHandle;

/* UINTC Handle for user space interrupt handling: */
extern UintcHandle             uintcHandle;

/* Global application resource configuration */
extern Resmgr_ResourceCfg      appResourceConfig;

/**********************************************************************
 ************************* Local Structures ***************************
 **********************************************************************/

typedef struct Test_LatencyInfo
{
    uint32_t    dspCore;
    uint64_t    minCnt;
    uint64_t    maxCnt;
	uint64_t	aveCnt;
	float		minTime;
	float		maxTime;
	float		aveTime;
    uint64_t    numMeasurements;
    uint64_t    totalLatency;
}Test_LatencyInfo;

/**********************************************************************
 ************************ Global Definitions **************************
 **********************************************************************/

/* Global Structure which tracks the latency information: */
Test_LatencyInfo    latencyInfo[2];

/* Test Selection: */
uint32_t            testSelection;

/**********************************************************************
 ************************* Unit Test Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function gets the timer base address given the timer numbers
 *
 *  @param[in]  tmrNum
 *      Timer Number
 *  @param[out]  pBaseAddress
 *      Timer base address populated
 *
 *  @retval
 *      Return status
 */
static CSL_Status tmrGetBaseAddress
(
    CSL_InstNum 	       tmrNum,
    CSL_TmrBaseAddress*    pBaseAddress
)
{
    CSL_Status st = CSL_SOK;

	if (pBaseAddress == NULL)
        return CSL_ESYS_INVPARAMS;

    switch (tmrNum) {
#ifdef CSL_TIMER_0
    case CSL_TIMER_0:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_0_REGS;
        break;
#endif /* CSL_TIMER_0 */
#ifdef CSL_TIMER_1
    case CSL_TIMER_1:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_1_REGS;
        break;
#endif /* CSL_TIMER_1 */
#ifdef CSL_TIMER_2
    case CSL_TIMER_2:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_2_REGS;
        break;
#endif /* CSL_TIMER_2 */
#ifdef CSL_TIMER_3
    case CSL_TIMER_3:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_3_REGS;
        break;
#endif /* CSL_TIMER_3 */
#ifdef CSL_TIMER_4
    case CSL_TIMER_4:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_4_REGS;
        break;
#endif /* CSL_TIMER_4 */
#ifdef CSL_TIMER_5
    case CSL_TIMER_5:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_5_REGS;
        break;
#endif /* CSL_TIMER_5 */
#ifdef CSL_TIMER_6
    case CSL_TIMER_6:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_6_REGS;
        break;
#endif /* CSL_TIMER_6 */
#ifdef CSL_TIMER_7
    case CSL_TIMER_7:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_7_REGS;
        break;
#endif /* CSL_TIMER_7 */
#ifdef CSL_TIMER_8
    case CSL_TIMER_8:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_8_REGS;
        break;
#endif /* CSL_TIMER_8 */
#ifdef CSL_TIMER_9
    case CSL_TIMER_9:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_9_REGS;
        break;
#endif /* CSL_TIMER_9 */
#ifdef CSL_TIMER_10
    case CSL_TIMER_10:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_10_REGS;
        break;
#endif /* CSL_TIMER_10 */
#ifdef CSL_TIMER_11
    case CSL_TIMER_11:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_11_REGS;
        break;
#endif /* CSL_TIMER_11 */
#ifdef CSL_TIMER_12
    case CSL_TIMER_12:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_12_REGS;
        break;
#endif /* CSL_TIMER_12 */
#ifdef CSL_TIMER_13
    case CSL_TIMER_13:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_13_REGS;
        break;
#endif /* CSL_TIMER_13 */
#ifdef CSL_TIMER_14
    case CSL_TIMER_14:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_14_REGS;
        break;
#endif /* CSL_TIMER_14 */
#ifdef CSL_TIMER_15
    case CSL_TIMER_15:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_15_REGS;
        break;
#endif /* CSL_TIMER_15 */
#ifdef CSL_TIMER_16
    case CSL_TIMER_16:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_16_REGS;
        break;
#endif /* CSL_TIMER_16 */
#ifdef CSL_TIMER_17
    case CSL_TIMER_17:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_17_REGS;
        break;
#endif /* CSL_TIMER_17 */
#ifdef CSL_TIMER_18
    case CSL_TIMER_18:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_18_REGS;
        break;
#endif /* CSL_TIMER_18 */
#ifdef CSL_TIMER_19
    case CSL_TIMER_19:
        pBaseAddress->regs = (CSL_TmrRegsOvly)CSL_TIMER_19_REGS;
        break;
#endif /* CSL_TIMER_19 */
    default:
        pBaseAddress->regs = (CSL_TmrRegsOvly)NULL;
        st = CSL_ESYS_FAIL;
        break;
    }
    return st;
}

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle associated with the MSGCOM instance
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void myFreeMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(appPktlibInstanceHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Debug utility function which displays the statistics of the heap
 *
 *  @param[in]  testHeapHandle
 *      PKTLIB heap handle
 *
 *  @retval
 *      Not Applicable.
 */
static void displayHeapStats(Pktlib_HeapHandle testHeapHandle)
{
    Pktlib_HeapStats    heapStats;

    /* Get the heap statistics */
    Pktlib_getHeapStats (testHeapHandle, &heapStats);

    /* Display the heap stats */
    printf ("Debug: Free Data Packet:%d Zero Buffer Packets:%d Garbage Packets:%d\n",
             heapStats.numFreeDataPackets, heapStats.numZeroBufferPackets, heapStats.numPacketsinGarbage);
}

/**
 *  @b Description
 *  @n
 *      The function is used to record the latency after the message
 *      has been sent from the DSP Core and received by the ARM Process.
 *
 *  @param[in]  hTimer64
 *      Timer Handle
 *  @param[in]  dspCore
 *      DSP Core number
 *  @param[in]  cntLo
 *      Timer Count Lo
 *  @param[in]  cntHi
 *      Timer Count Hi
 *
 *  @retval
 *      Not Applicable.
 */
static void test_recordLatency
(
    CSL_TmrHandle   hTimer64,
    uint32_t        dspCore,
    uint32_t        cntLo,
    uint32_t        cntHi
)
{
    uint32_t	    t64CounterLo;
    uint32_t	    t64CounterHi;
	uint64_t        dspCount;
    uint64_t        currentCount;
    uint64_t        deltaCount;
	float		    deltaTime;
	float		    alpha = 0.999;

    /* Validate the arguments: */
    if (dspCore >= 2)
    {
        printf ("Error: Invalid DSP core number %d in the message\n", dspCore);
        return;
    }

    /* Convert the received DSP count into 64 bit */
    dspCount = ((uint64_t)cntHi << 32) | cntLo;

    /* Get the HPLIB Time stamp */
    // currentCount = hplib_mUtilGetTimestamp();

	/* Get timer64 count */
	t64CounterLo = hTimer64->regs->CNTLO;   /* Read Low first */
	t64CounterHi = hTimer64->regs->CNTHI;
	currentCount = ((uint64_t)t64CounterHi << 32) | t64CounterLo;

    /* Compute the latency [DSP Transmits and ARM Receives] */
    deltaCount = currentCount - dspCount;
	deltaTime = (float)deltaCount*5/1000;

    /* Store the latency information: */
    if (deltaCount < latencyInfo[dspCore].minCnt)
	{
        latencyInfo[dspCore].minCnt = deltaCount;
		latencyInfo[dspCore].minTime = deltaTime;
	}
    if (deltaCount > latencyInfo[dspCore].maxCnt)
	{
        latencyInfo[dspCore].maxCnt = deltaCount;
		latencyInfo[dspCore].maxTime = deltaTime;
	}
    if (latencyInfo[dspCore].aveTime == 0)
		latencyInfo[dspCore].aveTime = deltaTime;
	else
		latencyInfo[dspCore].aveTime = alpha*latencyInfo[dspCore].aveTime + (1-alpha)*deltaTime;

	latencyInfo[dspCore].numMeasurements++;
    latencyInfo[dspCore].totalLatency += deltaCount;
    return;
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGUSR1. This is overriden
 *      to display the test latency.
 *
 *  @param[in]  signo
 *      Signal Number
 *  @param[in]  siginfo
 *      Signal Information
 *  @param[in]  context
 *      Context information.
 *
 *  @retval
 *      Not Applicable.
 */
static void test_displayLatencyInfo (int sig, siginfo_t *siginfo, void *context)
{
    int32_t index;

    /* Display the test latency: */
    for (index = 0; index < 2; index++)
    {
        printf ("DSP Core %d --> Min:%llu Max:%llu Number:%llu Total Latency:%llu \n",
                 index, latencyInfo[index].minCnt, latencyInfo[index].maxCnt,
                 latencyInfo[index].numMeasurements, latencyInfo[index].totalLatency);
        printf ("DSP Core %d --> Min:%5.2fus Max:%5.2fus Ave %5.2fus \n",
                 index, latencyInfo[index].minTime, latencyInfo[index].maxTime, latencyInfo[index].aveTime);
    }

    /* Display Test Latency at the end of the test also. */
    displayHeapStats(testHeapHandle);

    /* Initialize the latency information: */
    memset ((void *)&latencyInfo, 0, sizeof(Test_LatencyInfo));
    latencyInfo[0].minCnt = 0xFFFFFFFF;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the received message
 *
 *  @param[in]  hTimer64
 *      Timer Handle
 *  @param[in]  armChannelHandle
 *      ARM Channel Handle on which the message was received
 *  @param[in]  dspChannelHandle
 *      DSP Channel Handle on which the message response is to be sent back.
 *
 *  @retval
 *      Number of packets processed.
 */
static int32_t test_processMessage
(
    CSL_TmrHandle   hTimer64,
    MsgCom_ChHandle armChannelHandle,
    MsgCom_ChHandle dspChannelHandle
)
{
    Ti_Pkt*     ptrMessage;
    uint32_t*   ptrDataBuffer;
    uint32_t    dataLen;

    /* Process all the messages: */
    if (Msgcom_getMessage (armChannelHandle, (MsgCom_Buffer**)&ptrMessage) < 0)
    {
        printf ("Error: Unable to get a message\n");
        return 0;
    }

    /* Did we pop off an empty message? */
    if (ptrMessage == NULL)
        return 0;

    /* Get the data buffer from the received message. */
    Pktlib_getDataBuffer(ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

    /* Validations:
     *  - Ensure the packet length matches what we sent.
     *  - Ensure the data length matches what we sent. */
    if (Pktlib_getPacketLen(ptrMessage) != 12)
    {
        printf ("Error: Invalid packet length detected in the received packet.\n");
        return 0;
    }
    if (dataLen != 12)
    {
        printf ("Error: Invalid data length detected in the received packet.\n");
        return 0;
    }

    /* Record the test latency: */
    test_recordLatency (hTimer64, *(ptrDataBuffer), *(ptrDataBuffer + 1), *(ptrDataBuffer + 2));

    /* Do we need to send back a response? */
    if ((testSelection == 1) || (testSelection == 3))
    {
        /* YES. Send the message. */
        if (Msgcom_putMessage(dspChannelHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            printf ("Error: Unable to send the message\n");
            return 0;
        }
    }
    else
    {
        /* Cleanup the received message */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);
    }

    /* Message has been processed. */
    return 1;
}

/**
 *  @b Description
 *  @n
 *      Reader thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread
 *
 *  @retval
 *      Always NULL
 */
void* reader(void* arg)
{
	struct sched_param     param;
	uint32_t               core;
    cpu_set_t              set;
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        armChannelHandle[2];
    MsgCom_ChHandle        dspChannelHandle[2];
    int32_t                errCode;
    int32_t                retVal;
    uint32_t               index;
    Name_ResourceCfg       namedResCfg;
    uint32_t               numMessages;
    struct sigaction       act;
    uint32_t               fd;
	CSL_Status             st;
	CSL_InstNum            tmrNum;
    CSL_TmrBaseAddress     t64_phy_addr;
    CSL_TmrBaseAddress     t64_virt_addr;
	CSL_TmrObj             t64_obj;
    void                   *map_base;
	uint32_t               map_size;
	uint32_t               map_mask;
	CSL_TmrHandle          hTimer64;

#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
    /* Select the ARM Core to which the stress test will be affiniated */
	core = 2;
#elif defined (DEVICE_K2L)
    /* Select the ARM Core to which the stress test will be affiniated */
	core = 1;
#else
#error "Error: Unsupported Device"
#endif

    /* Set the core affinity for the reader thread: */
	CPU_ZERO(&set);
	CPU_SET(core, &set);
	if (sched_setaffinity((pid_t) syscall (SYS_gettid), sizeof(cpu_set_t), &set))
	{
		printf("Error; sched_setaffinity error, core %d\n", core);
		return NULL;
	}
    printf("Debug: Reader thread is affiliated to the core %d\n", core);

    /* Setup the user defined signal to display the resource usage. */
	act.sa_sigaction = &test_displayLatencyInfo;
	act.sa_flags     = SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);

    /* Loop around till the test selection has been entered on the DSP Core0 */
    while (1)
    {
        if (Name_findResource(globalNameDatabaseHandle,Name_ResourceBucket_USER_DEF1,
                              "TestSelection", &namedResCfg, &errCode) < 0)
        {
            if (errCode == NAME_ENOTFOUND)
                continue;

            /* FATAL Error: Find API returned an error. */
            printf ("Error: Find Named resource failed with error code %d\n", errCode);
            return NULL;
        }
        /* Get the test selection. */
        testSelection = namedResCfg.handle1;
        break;
    }

    /* Initialize the latency information: */
    memset ((void *)&latencyInfo, 0, sizeof(Test_LatencyInfo));
    latencyInfo[0].minCnt = 0xFFFFFFFF;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                           = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                                    = NULL;
    chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(testHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    armChannelHandle[0] = Msgcom_create ("Stress-Test:ARM Reader0", Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errCode);
    if (armChannelHandle[0] == 0)
    {
        printf ("Error: Unable to open the channel [Error: %d]\n", errCode);
        return NULL;
    }

    /* Push the channel name from the ARM to the DSP realm. */
    if (Name_push (nameClientHandle, "Stress-Test:ARM Reader0", Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Channel name PUSH to ARM realm failed [Error code %d] \n", errCode);
        return NULL;
    }
    printf ("Debug: Channel Name0 pushed successfully to the DSP realm\n");

    /* Display the information: */
    printf ("Debug: Queue-Pend      : %x:%x\n", appResourceConfig.qPendResponse[0].queue.qMgr, appResourceConfig.qPendResponse[0].queue.qNum);
    printf ("Debug: System Interrupt: %d\n",    appResourceConfig.qPendResponse[0].systemInterrupt);
    printf ("Debug: Host Interrupt  : %d\n",    appResourceConfig.qPendResponse[0].hostInterrupt);
    printf ("Debug: Heap Free Queue : %d\n",    chConfig.u.queueDMACfg.rxFreeQueueNum);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                           = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                                    = NULL;
    chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(testHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the Message communicator channel. */
    armChannelHandle[1] = Msgcom_create ("Stress-Test:ARM Reader1", Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errCode);
    if (armChannelHandle[1] == 0)
    {
        printf ("Error: Unable to open the channel [Error: %d]\n", errCode);
        return NULL;
    }

    /* Push the channel name from the ARM to the DSP realm. */
    if (Name_push (nameClientHandle, "Stress-Test:ARM Reader1", Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Channel name PUSH to ARM realm failed [Error code %d] \n", errCode);
        return NULL;
    }
    printf ("Debug: Channel Name1 pushed successfully to the DSP realm\n");

    /* Create timer64 */
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf ("Error: Can't open fd \n");
        return NULL;
    }
#if (defined(DEVICE_K2H) || defined (DEVICE_K2K))
	tmrNum = CSL_TIMER_6;
	if ((st = tmrGetBaseAddress(tmrNum, &t64_phy_addr)) != CSL_SOK)
	{
        printf ("Error: Can't get Timer%d physical address\n", tmrNum);
        return NULL;
	}
#elif defined (DEVICE_K2L)
	tmrNum = CSL_TIMER_8;
	if ((st = tmrGetBaseAddress(tmrNum, &t64_phy_addr)) != CSL_SOK)
	{
        printf ("Error: Can't get Timer%d physical address\n", tmrNum);
        return NULL;
	}
#else
#error "Unsupported Device"
#endif
    /* Minimum page size for the mmapped region is 4K */
	map_size = 4*1024;
    map_mask = map_size - 1;
	map_base = mmap(0, map_size, (PROT_READ|PROT_WRITE), MAP_SHARED, fd, (off_t)t64_phy_addr.regs & ~map_mask);
    if(map_base == MAP_FAILED)
    {
        printf ("Error: Can't map physical address %x\n", (uint32_t)t64_phy_addr.regs);
        return NULL;
    }
    t64_virt_addr.regs = (CSL_TmrRegsOvly) map_base + ((off_t)t64_phy_addr.regs & map_mask);

    /* Create the Timer64 handle which will be used in the test */
    t64_obj.regs   = t64_virt_addr.regs;
    t64_obj.tmrNum = (CSL_InstNum)tmrNum;
	hTimer64 = &t64_obj;

    /* Display the information: */
    printf ("Debug: Queue-Pend      : %x:%x\n", appResourceConfig.qPendResponse[1].queue.qMgr, appResourceConfig.qPendResponse[1].queue.qNum);
    printf ("Debug: System Interrupt: %d\n",    appResourceConfig.qPendResponse[1].systemInterrupt);
    printf ("Debug: Host Interrupt  : %d\n",    appResourceConfig.qPendResponse[1].hostInterrupt);
    printf ("Debug: Heap Free Queue : %d\n",    chConfig.u.queueDMACfg.rxFreeQueueNum);
    printf ("Debug: Timer Used      : %d: %lx(virt) mapped to %x(phy)\n", tmrNum, (long)hTimer64->regs, (uint32_t)t64_phy_addr.regs);

    /* Wait for the DSP MSGCOM Channels to be created*/
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        dspChannelHandle[0] = Msgcom_find(appMsgcomInstanceHandle, "Stress-Test:DSP Reader0", &errCode);
        if (dspChannelHandle[0] != NULL)
            break;
        usleep(100);
    }

    /* Wait for the DSP MSGCOM Channels to be created*/
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
        dspChannelHandle[1] = Msgcom_find(appMsgcomInstanceHandle, "Stress-Test:DSP Reader1", &errCode);
        if (dspChannelHandle[1] != NULL)
            break;
        usleep(100);
    }

    /* Initiating the test: */
    printf ("Debug: Waiting for messages to arrive from the DSP Core(s)\n");

    /* Set the configured policy and priority */
    param.sched_priority = 60;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Setting Reader thread to SCHED FIFO, Priority %d\n", param.sched_priority);

    /* Initiating the test: */
    printf ("Debug: Waiting for messages to arrive from the DSP Core(s)\n");
    while (1)
    {
        retVal = Uintc_select (uintcHandle, NULL, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;
			
            /* Error: UINTC select failed */
            printf ("Error: UINTC select failed [Error code %d]\n", errCode);
            return NULL;
        }

        /* Loop around: Process all the messages */
        while (1)
        {
            /* Initialize the number of messages which have been processed */
            numMessages = 0;

            /* Cycle through and process all the messages */
            for (index = 0; index < 2; index++)
                numMessages = numMessages + test_processMessage (hTimer64, armChannelHandle[index], dspChannelHandle[index]);

            /* Did we process all the messages? */
            if (numMessages == 0)
                break;
        }
    }

    /* Delete the MSGCOM Channel. */
    for (index = 0; index < 2; index++)
    {
        if (Msgcom_delete (armChannelHandle[index], myFreeMsgBuffer) < 0)
            printf ("Error: MSGCOM ARM Channel deletion %d failed\n", index);
        if (Msgcom_delete (dspChannelHandle[index], myFreeMsgBuffer) < 0)
            printf ("Error: MSGCOM DSP Channel deletion %d failed\n", index);
    }

    /* Control comes here implies that all the reader tests passed */
    printf ("*******************************************************************\n");
    printf ("Debug: MSGCOM Reader Unit Test all passed\n");
    printf ("*******************************************************************\n");

    /* We are done with the test: */
    return NULL;
}

