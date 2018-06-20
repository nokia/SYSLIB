/*
 *   @file  reestablishment.c
 *
 *   @brief
 *      The file benchmarks the reestablishment procedure for fast path radio
 *      bearers.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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
 ************************ Unit Test Definitions ***********************
 **********************************************************************/

/* Test User Radio Bearer Identifier.  */
#define TEST_USER_RADIO_BEARER_ID           7

/* Maximum number of reestablishment procedure attempted in the test */
#define MAX_NUMBER_REESTABLISHMENT          20000

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Event Object which keeps track of Encoded and Decoded events. */
static Event_Handle     encodeDecodeEventObject;

/* Global variable which keeps track of the number of reestablishments initiated */
uint32_t                gNumReestablishmentInitiated = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_suspendLTEChannel)
BENCHMARK_INIT(Netfp_getLTEChannelOpt)
BENCHMARK_INIT(Netfp_reconfigureLTEChannel)
BENCHMARK_INIT(Netfp_resumeLTEChannel)
BENCHMARK_INIT(Netfp_deleteUser)
BENCHMARK_INIT(Netfp_createUser)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/* Statistics Display Functionality. */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Display the statistics for the Re-establishment DRB tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_displayDrbFpReestablishmentStats(void)
{
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("LTE Fast Path Re-establishment Tests::::\n");
    System_printf ("Number of Reestablishments Initiated: %d\n", gNumReestablishmentInitiated);
    BENCHMARK_DISPLAY(Netfp_getLTEChannelOpt);
    BENCHMARK_DISPLAY(Netfp_suspendLTEChannel);
    BENCHMARK_DISPLAY(Netfp_reconfigureLTEChannel);
    BENCHMARK_DISPLAY(Netfp_resumeLTEChannel);
    BENCHMARK_DISPLAY(Netfp_deleteUser);
    BENCHMARK_DISPLAY(Netfp_createUser);
    return;
}

/**
 *  @b Description
 *  @n
 *      Callback function registered when a GTPU packet is received
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *  @param[in]  arg
 *      Optional application specific argument
 *
 *  @retval
 *      Not Applicable
 */
static void Test_gtpuChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post(encodeDecodeEventObject, Event_Id_00);
    return;
}

/**
 *  @b Description
 *  @n
 *      Callback function registered when data is received on the encoded
 *      channel
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *  @param[in]  arg
 *      Optional application specific argument
 *
 *  @retval
 *      Not Applicable
 */
static void Test_encodeChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post(encodeDecodeEventObject, Event_Id_01);
    return;
}

/**
 *  @b Description
 *  @n
 *      Callback function registered when data is received on the decoded
 *      channel
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *  @param[in]  arg
 *      Optional application specific argument
 *
 *  @retval
 *      Not Applicable
 */
static void Test_decodeChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post(encodeDecodeEventObject, Event_Id_02);
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_benchmarkingReestablishTask(UArg arg0, UArg arg1)
{
    Netfp_InboundFPHandle       fpIngressV4Handle;
    Netfp_OutboundFPHandle      fpEgressV4Handle;
    Msgcom_ChannelCfg           chConfig;
    Netfp_UserCfg               userCfg;
    Netfp_UserHandle            ueHandle;
    uint8_t                     encryptionKey[16];
    uint8_t                     integrityKey[16];
    MsgCom_ChHandle             gtpuChannelHandle;
    MsgCom_ChHandle             encodeChannel;
    MsgCom_ChHandle             decodeChannel;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     index;
    int32_t                     errCode;
    int32_t                     retVal;
    Netfp_SockHandle            lteDRBChannel;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    uint32_t                    gtpuIdentifier = 0xdead12;
    Netfp_UserHandle            reestablishedUeHandle;
    Netfp_UserHandle            oldUeHandle = NULL;
    Netfp_SockHandle            reestablishedLteDRBChannel;
    char                        channelName[MSG_COM_MAX_CHANNEL_LEN];
    Netfp_OptionTLV             optCfg;
    uint32_t                    currentCountC;

    /* Get the ingress and egress IPv4 fast paths. */
    while (1)
    {
        fpIngressV4Handle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
        if (fpIngressV4Handle != NULL)
            break;
        Task_sleep(10);
    }
    while (1)
    {
        fpEgressV4Handle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
        if (fpEgressV4Handle != NULL)
            break;
        Task_sleep(10);
    }

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "SocketTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        System_printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Create the Event object used */
    encodeDecodeEventObject = Event_create(NULL, NULL);
    if (encodeDecodeEventObject == NULL)
    {
        System_printf ("Error: Encode-Decode Event Object creation failed\n");
        return;
    }

    /************************************************************************************
     * GTPU Channel: This is the channel where the GTPU packets will be received.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_gtpuChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the channel name: */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "GTPU-Channel-%d", DNUM);

    /* Create the Message communicator channel. */
    gtpuChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (gtpuChannelHandle == 0)
    {
        System_printf ("Error: Unable to open the channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Encode Channel: This is the channel where the packets will be encoded invoking the
     * NETFP encoding API.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_encodeChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the channel name: */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "Encode-%d", DNUM);

    /* Create the Message communicator channel. */
    encodeChannel = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (encodeChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Decode Channel: This is the channel where the packets will be present once they
     * have been decoded.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_decodeChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[3].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[3].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[3].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[3].hostInterrupt;

    /* Create the channel name: */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "Decode-%d", DNUM);

    /* Create the Message communicator channel. */
    decodeChannel = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (decodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Create the LTE User:
     ************************************************************************************/

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Initialize the integrity and ciphering keys */
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (TSCL & 0xFF);
        integrityKey[index]  = (TSCL & 0xFF);
    }

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Populate the user security configuration. */
    userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
    userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.ueId             = (DNUM + 4);
    userCfg.srbFlowId        = rxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = NULL;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb1Dec        = NULL;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Enc        = NULL;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Dec        = NULL;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    BENCHMARK_START(Netfp_createUser);
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    BENCHMARK_END(Netfp_createUser);
    if (ueHandle == NULL)
    {
        /* User Security Context creation failed. */
        System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: LTE User %d has been created\n", userCfg.ueId);

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_createUser);

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = Msgcom_getInternalMsgQueueInfo(gtpuChannelHandle);
    lteChannelBindCfg.fpHandle        = fpIngressV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = 1;
    lteChannelBindCfg.isHOInProgress  = 0;
    lteChannelBindCfg.chDrbEnc        = Msgcom_getInternalMsgQueueInfo(encodeChannel);

    /* Populate the channel connect configuration: */
    lteChannelConnectCfg.fpHandle       = fpEgressV4Handle;
    lteChannelConnectCfg.sin_gtpuId     = gtpuIdentifier;
    lteChannelConnectCfg.qci            = 3;
    lteChannelConnectCfg.dscp           = 0x22;
    lteChannelConnectCfg.flowId         = rxFlowId;
    lteChannelConnectCfg.chDrbDec       = Msgcom_getInternalMsgQueueInfo(decodeChannel);

    /* Create the DRB LTE channel: */
    lteDRBChannel = Netfp_createLTEChannel (ueHandle, TEST_USER_RADIO_BEARER_ID, Netfp_SockFamily_AF_INET,
                                            &lteChannelBindCfg, &lteChannelConnectCfg, &errCode);
    if (lteDRBChannel == NULL)
    {
        System_printf ("Error: Failed to create the LTE channel [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: LTE channel %x has been created\n", lteDRBChannel);

    /* Sleep for some time before starting the tests: */
    Task_sleep(1000*10);
    System_printf ("Debug: Starting the benchmarking reestablishment test on DSP Core%d\n", DNUM);

    /**********************************************************************************
     * Stress Test:
     *  Loop around and invoke the API which simulate the reestablishment procedure
     *********************************************************************************/
    while (gNumReestablishmentInitiated <= MAX_NUMBER_REESTABLISHMENT)
    {
        /* Initialize the integrity and ciphering keys */
        for (index = 0; index < 16; index++)
        {
            encryptionKey[index] = (TSCL & 0xFF);
            integrityKey[index]  = (TSCL & 0xFF);
        }

        /* Suspending the channel. */
        BENCHMARK_START(Netfp_suspendLTEChannel);
        retVal = Netfp_suspendLTEChannel (ueHandle, TEST_USER_RADIO_BEARER_ID, rxFlowId,  &errCode);
        BENCHMARK_END(Netfp_suspendLTEChannel);
        if (retVal < 0)
        {
            System_printf ("Error: Suspend Channel Failed (Error %d)\n", errCode);
            while(1);
        }

        /* Update & log the information. */
        BENCHMARK_UPDATE(Netfp_suspendLTEChannel);

        /* Initialize the user security configuration. */
        memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

		/* Populate the user security configuration. */
		userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
		userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
		userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
		userCfg.ueId             = (DNUM + 4);
		userCfg.srbFlowId        = rxFlowId;
		userCfg.initialCountC    = 0;
		userCfg.chSrb1Enc        = NULL;    /* Set to NULL only because we are testing DRB's here */
		userCfg.chSrb1Dec        = NULL;    /* Set to NULL only because we are testing DRB's here */
		userCfg.chSrb2Enc        = NULL;    /* Set to NULL only because we are testing DRB's here */
		userCfg.chSrb2Dec        = NULL;    /* Set to NULL only because we are testing DRB's here */
		memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
		memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
		memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

		/* Create a new user */
		BENCHMARK_START(Netfp_createUser);
		reestablishedUeHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
		BENCHMARK_END(Netfp_createUser);
		if (reestablishedUeHandle == NULL)
		{
			/* Adding a new user failed. */
			System_printf ("Error: LTE User Creation failed [Error code %d]\n", errCode);
			while(1);
		}

		/* Update & log the information. */
		BENCHMARK_UPDATE(Netfp_createUser);

		/* For fast path bearers SA maintains the CountC value. We are resuming countC in this test.
		 * Get the countc value from SA */
		currentCountC = 0xFFFF;

		/* Count C is being resumed; so we need the last countC which we had used. */
		optCfg.type     =   Netfp_Option_COUNTC;
		optCfg.length   =   4;
		optCfg.value    =   (void*)&currentCountC;

		BENCHMARK_START(Netfp_getLTEChannelOpt);
		retVal = Netfp_getLTEChannelOpt (ueHandle, TEST_USER_RADIO_BEARER_ID, &optCfg, &errCode);
		BENCHMARK_END(Netfp_getLTEChannelOpt);
		if (retVal < 0)
		{
			/* Failed to get CountC value. */
			System_printf ("Error: Failed to get countC [Error code %d]\n", errCode);
			return;
		}

		/* Update & log the information. */
		BENCHMARK_UPDATE(Netfp_getLTEChannelOpt);

		/* Increment countC value when using for the next packet */
		currentCountC = currentCountC + 1;

		/* The LTE channel was suspended; we need to reconfigure the channel */
		BENCHMARK_START(Netfp_reconfigureLTEChannel);
		reestablishedLteDRBChannel = Netfp_reconfigureLTEChannel(reestablishedUeHandle, ueHandle,
																 TEST_USER_RADIO_BEARER_ID, currentCountC, &errCode);
		BENCHMARK_END(Netfp_reconfigureLTEChannel);
		if (reestablishedLteDRBChannel == NULL)
		{
			System_printf ("Error: Reconfigure LTE channel Failed (Error %d)\n", errCode);
			while(1);
		}
		/* Update & log the information. */
		BENCHMARK_UPDATE(Netfp_reconfigureLTEChannel);

		/* Resume the user */
		BENCHMARK_START(Netfp_resumeLTEChannel);
		retVal = Netfp_resumeLTEChannel(reestablishedUeHandle, ueHandle, TEST_USER_RADIO_BEARER_ID, &errCode);
		BENCHMARK_END(Netfp_resumeLTEChannel);
		if (retVal < 0)
		{
			System_printf ("Error: Resume LTE channel Failed (Error %d)\n", errCode);
			while(1);
		}

		/* Update & log the information. */
		BENCHMARK_UPDATE(Netfp_resumeLTEChannel);

		/* Book keeping UE handles */
		if (oldUeHandle != NULL)
		{
			/* Delete the old User. This context will not be used anymore */
			BENCHMARK_START(Netfp_deleteUser);
			retVal = Netfp_deleteUser (oldUeHandle, &errCode);
			BENCHMARK_END(Netfp_deleteUser);
			if (retVal < 0)
			{
				System_printf ("Error: Delete old User Failed (Error %d)\n", errCode);
				while(1);
			}
		   /* Update & log the information. */
			BENCHMARK_UPDATE(Netfp_deleteUser);
		}

		/* Store the previous UE handle. This might be needed for re-ciphering */
		oldUeHandle = ueHandle;

		/* From this point on use; the new user handle */
		ueHandle = reestablishedUeHandle;

		/* From this point on use; the new channel handle */
		lteDRBChannel = reestablishedLteDRBChannel;

		/* Increment the counter */
		gNumReestablishmentInitiated++;
    }

    /* Display the results at the end of the test */
    Test_displayDrbFpReestablishmentStats();
    return;
}


