/*
 *   @file  basicReestablishment.c
 *
 *   @brief
 *      Basic Reestablishment Procedure
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
#include <unistd.h>
#include <pthread.h>
#include <sys/syscall.h>

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/**********************************************************************
 ************************ Local Definitions ***************************
 **********************************************************************/

/* Initial value of the countC */
#define  TEST_INITIAL_COUNTC            100

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;
extern Pktlib_HeapHandle    netfpDataRxHeap;
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/**********************************************************************
 ************************ Global Variables ****************************
 **********************************************************************/

Qmss_QueueHnd       gtpuQueue;
Qmss_QueueHnd       encodeQueue;
Qmss_QueueHnd       decodeQueue;

/**********************************************************************
 ********************* Reestablishment Procedure **********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE user
 *
 *  @param[in]  ueId
 *      UE identifier
 *  @param[in]  rxFlowId
 *      Receive Flow identifier to be configured.
 *  @param[in]  encodeQueue
 *      Encode Queue
 *  @param[in]  decodeQueue
 *      Decode Queue
 *
 *  @retval
 *      Success -   User handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_UserHandle Test_createUser
(
    uint8_t         ueId,
    int32_t         rxFlowId,
    Qmss_QueueHnd   encodeQueue,
    Qmss_QueueHnd   decodeQueue
)
{
    Netfp_UserCfg       userCfg;
    uint8_t             encryptionKey[16];
    uint8_t             integrityKey[16];
    Netfp_UserHandle    ueHandle;
    uint32_t            index;
    int32_t             errCode;

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Initialize the integrity and ciphering keys */
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (random() & 0xFF);
        integrityKey[index]  = (random() & 0xFF);
    }

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Populate the user security configuration. */
    userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
    userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.ueId             = ueId;
    userCfg.srbFlowId        = rxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = encodeQueue;
    userCfg.chSrb1Dec        = decodeQueue;
    userCfg.chSrb2Enc        = encodeQueue;
    userCfg.chSrb2Dec        = decodeQueue;
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    if (ueHandle == NULL)
        printf ("Error: LTE Creation User %d failed [Error code %d]\n", ueId, errCode);

    /* Return the user handle. */
    return ueHandle;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to create an LTE channel
 *
 *  @param[in]  ueHandle
 *      User handle on which the channel is created
 *  @param[in]  rbId
 *      Radio Bearer identifier
 *  @param[in]  fpInboundV4Handle
 *      Inbound handle
 *  @param[in]  fpOutboundV4Handle
 *      Outbound handle
 *  @param[in]  gtpuChannelHandle
 *      Handle to which non fast path packets will be placed after reception
 *  @param[in]  encodeChannel
 *      Packets where encoded packets are placed
 *  @param[in]  decodeChannel
 *      Packets where decoded packets are placed
 *  @param[in]  rxFlowId
 *      Flow identifier to be used
 *  @param[in]  gtpuIdentifier
 *      GTPU identifier
 *
 *  @retval
 *      Success -   Socket handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_SockHandle Test_createLTEChannel
(
    Netfp_UserHandle        ueHandle,
    uint8_t                 rbId,
    Netfp_InboundFPHandle   fpInboundV4Handle,
    Netfp_OutboundFPHandle  fpOutboundV4Handle,
    Qmss_QueueHnd           gtpuChannelHandle,
    Qmss_QueueHnd           encodeChannel,
    Qmss_QueueHnd           decodeChannel,
    int32_t                 rxFlowId,
    uint32_t                gtpuIdentifier
)
{
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    Netfp_SockHandle            lteDRBChannel;
    int32_t                     errCode;

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = gtpuChannelHandle;
    lteChannelBindCfg.fpHandle        = fpInboundV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = TEST_INITIAL_COUNTC;
    lteChannelBindCfg.enableFastPath  = 1;
    lteChannelBindCfg.isHOInProgress  = 0;
    lteChannelBindCfg.chDrbEnc        = encodeChannel;

    /* Populate the channel connect configuration: */
    lteChannelConnectCfg.fpHandle       = fpOutboundV4Handle;
    lteChannelConnectCfg.sin_gtpuId     = gtpuIdentifier;
    lteChannelConnectCfg.qci            = 3;
    lteChannelConnectCfg.dscp           = 0x22;
    lteChannelConnectCfg.flowId         = rxFlowId;
    lteChannelConnectCfg.chDrbDec       = decodeChannel;

    /* Create the DRB LTE channel: */
    lteDRBChannel = Netfp_createLTEChannel (ueHandle, rbId, Netfp_SockFamily_AF_INET,
                                            &lteChannelBindCfg, &lteChannelConnectCfg, &errCode);
    if (lteDRBChannel == NULL)
        printf ("Error: Failed to create the LTE channel for %d [Error code %d]\n", rbId, errCode);

    return lteDRBChannel;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_basicReestablishmentThread(void* arg)
{
    Netfp_OutboundFPHandle      fpEgressV4Handle;
    Netfp_InboundFPHandle       fpIngressV4Handle;
    Netfp_FlowCfg               flowCfg;
    int32_t                     myFlowHandle;
    uint32_t                    currentCountC;
    uint32_t                    appCountC;
    int32_t                     errCode;
    int32_t                     ueIndex = 127;
    int32_t                     rbIndex = 4;
    Netfp_OptionTLV             optCfg;
    uint8_t                     isAllocated;
    uint32_t                    gtpuIdentifier = 0xdead12;
    struct sched_param          param;
    uint32_t                    core;
    cpu_set_t                   set;
    Netfp_SockHandle            lteDRBHandle;
    Netfp_SockHandle            reestablishedLteDRBChannel;
    Netfp_UserHandle            ueHandle;
    Netfp_UserHandle            reestablishedUeHandle;
    uint32_t                    numS1Packets;
    uint32_t                    numS1SuspendedPackets;
    Pktlib_HeapStats            heapStats;
    Netfp_ExtendedSocketStats   extendedSocketStats;
    uint32_t                    index = 0;
    Ti_Pkt*                     ptrPkt;

    /* Set the core affinity: */
    core = 1;
    CPU_ZERO(&set);
    CPU_SET(core, &set);
    if (sched_setaffinity((pid_t) syscall (SYS_gettid), sizeof(cpu_set_t), &set))
    {
        printf("Error; sched_setaffinity error, core %d\n", core);
        return NULL;
    }
    printf("Debug: LTE Reestablishment Thread is affiliated to the core %d\n", core);

    /* Set the configured policy and priority */
    param.sched_priority = 2;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the Test thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Get the ingress and egress IPv4 fast paths. */
    fpIngressV4Handle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressV4Handle == NULL)
    {
        printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return NULL;
    }
    fpEgressV4Handle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressV4Handle == NULL)
    {
        printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return NULL;
    }

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;
    strcpy (flowCfg.name, "Reestablishment-Flow");

    /* Create a test flow which will be used in the unit tests. */
    myFlowHandle = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (myFlowHandle < 0)
    {
        printf ("Error: Fast Path Flow Creation Failed [Error code %d]\n", errCode);
        return NULL;
    }

    /* Get the heap statistics: */
    Pktlib_getHeapStats (netfpDataRxHeap, &heapStats);

    /* Open the encode/decode queue */
    encodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    decodeQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    gtpuQueue   = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

    /* Create a user: */
    ueHandle = Test_createUser (ueIndex, myFlowHandle, encodeQueue, decodeQueue);
    if (ueHandle == 0)
        return NULL;

    /* Create a DRB for the user: */
    lteDRBHandle = Test_createLTEChannel (ueHandle, rbIndex, fpIngressV4Handle, fpEgressV4Handle,
                                          gtpuQueue, encodeQueue, decodeQueue,
                                          myFlowHandle, gtpuIdentifier);
    if (lteDRBHandle == NULL)
        return NULL;

    /********************************************************************************************
     * Suspend the LTE Channel:
     ********************************************************************************************/
    if (Netfp_suspendLTEChannel (ueHandle, rbIndex, myFlowHandle, &errCode) < 0)
    {
        printf ("Error: Suspend Channel Failed (Error %d)\n", errCode);
        return NULL;
    }

    /* Get the current countC on the configured channel */
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (ueHandle, rbIndex, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        printf ("Error: Failed to get countC [Error code %d]\n", errCode);
        return NULL;
    }

    /* Sanity Check: Ensure that the countC is valid */
    if (currentCountC != (TEST_INITIAL_COUNTC - 1))
    {
        /* Error: Mismatch in the countC detected */
        printf ("Error: Expected countC %d Got %d\n", TEST_INITIAL_COUNTC, currentCountC);
        return NULL;
    }

    /* Get the extended socket statistics */
    optCfg.type   = Netfp_Option_EXTENDED_STATISTICS;
    optCfg.length = sizeof(Netfp_ExtendedSocketStats);
    optCfg.value  = (void*)&extendedSocketStats;
    if (Netfp_getSockOpt (lteDRBHandle, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Failed to get extended socket statistics [Error code %d]\n", errCode);
        return NULL;
    }

    /* Sanity Check: Ensure that the counters are valid */
    if (extendedSocketStats.reEstDroppedPackets != 0)
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Unexpected value in the extended socket statistics %d detected\n", (uint32_t)extendedSocketStats.reEstDroppedPackets);
        return NULL;
    }

    /************************************************************************************
     * Generating Packets on the S1 Link:
     ************************************************************************************/
    printf ("*******************************************************************************************\n");
    printf ("Please execute the BURST packet generator to sent GTPU Packets with Id: 0x%x\n", gtpuIdentifier);
    printf ("All packets should be of the same size (128 bytes) since the test will validate\n");
    printf ("and ensure that the COUNTC is correct. The packets received on the S1 Link\n");
    printf ("will be buffered until the HO procedure is complete. Heap %p is used to\n", netfpDataRxHeap);
    printf ("receive the data and is configured to buffer %d packets\n", heapStats.numFreeDataPackets);
    printf ("*******************************************************************************************\n");
    printf ("Please enter the number of packets AFTER the BURST Packet Generator has been executed\n");
    printf ("Enter the number of packets which had been generated on the S1 Link: ");
    scanf ("%d", &numS1Packets);
    printf ("Enter the number of suspended packets which will be HANDLED: ");
    scanf ("%d", &numS1SuspendedPackets);

    /* Sanity Check: Are we are processing all the suspended packets. */
    if (numS1SuspendedPackets > numS1Packets)
        numS1SuspendedPackets = numS1Packets;

    /***********************************************************************************
     * COUNTC Validations: Get the countC from the suspended LTE channel. Since no
     * packets were passed through the LTE channel the countC remains the initial value.
     * NOTE: CountC are always 1 less than the expected value.
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (ueHandle, rbIndex, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        printf ("Error: Failed to get countC on old UE [Error code %d]\n", errCode);
        return NULL;
    }
    currentCountC = currentCountC + 1;
    if (currentCountC != TEST_INITIAL_COUNTC)
    {
        /* Error: Unexpected countC detected */
        printf ("Error: Unexpected countC %d detected on the old channel [Expected %d]\n",
                 currentCountC, TEST_INITIAL_COUNTC);
        return NULL;
    }

    /**********************************************************************************
     * Recreate the user with a new configuration:
     *********************************************************************************/
    reestablishedUeHandle = Test_createUser (ueIndex, myFlowHandle, encodeQueue, decodeQueue);
    if (reestablishedUeHandle == NULL)
        return NULL;

    /* Reconfigure the LTE channel: We continue from the countC from the old suspended LTE channel */
    appCountC = currentCountC;
    reestablishedLteDRBChannel = Netfp_reconfigureLTEChannel(reestablishedUeHandle, ueHandle,
                                                             rbIndex, appCountC, &errCode);
    if (reestablishedLteDRBChannel == NULL)
    {
        printf ("Error: Reconfigure LTE channel Failed (Error %d)\n", errCode);
        return NULL;
    }
    printf ("Debug: Reconfigured the LTE Channel with countC: %d\n", appCountC);

    /* Process all the suspended packets: */
    for (index = 0; index < numS1SuspendedPackets; index++)
    {
        /* Get the suspended packet: */
        if (Netfp_getSuspendedPacket (ueHandle, rbIndex, &ptrPkt, &errCode) < 0)
        {
            /* Error: Get the suspended packet failed */
            printf ("Error: Unable to get the suspended packet [Error code %d]\n", errCode);
            return NULL;
        }

        /* Sanity Check: We should have received a packet */
        if (ptrPkt == NULL)
        {
            printf ("Error: No suspended packet present\n");
            return NULL;
        }

        /* Encode the DRB: */
        errCode = Netfp_encodeDrb (reestablishedUeHandle, ptrPkt, appCountC, rbIndex);
        if (errCode < 0)
        {
            /* Error: Encode the DRB failed */
            printf ("Error: Unable to encode the DRB [Error code %d]\n", errCode);
            return NULL;
        }

        /* Increment the countC */
        appCountC++;
    }
    printf ("Debug: Encoded %d suspended packets [countC is now %d]\n", numS1SuspendedPackets, appCountC);

    /**********************************************************************************
     * Resume the LTE Channel:
     *********************************************************************************/
    if (Netfp_resumeLTEChannel(reestablishedUeHandle, ueHandle, rbIndex, &errCode) < 0)
    {
        printf ("Error: Resume LTE channel Failed (Error %d)\n", errCode);
        return NULL;
    }
    printf ("Debug: Resumed the LTE Channel\n");

    /***********************************************************************************
     * COUNTC Validations: Old suspended LTE Channel. The channel was already suspended
     * so the countC should have remained the same.
     * NOTE: CountC are always 1 less than the expected value.
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (ueHandle, rbIndex, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        printf ("Error: Failed to get countC on old UE [Error code %d]\n", errCode);
        return NULL;
    }
    currentCountC = currentCountC + 1;
    if (currentCountC != TEST_INITIAL_COUNTC)
    {
        /* Error: Unexpected countC detected */
        printf ("Error: Unexpected countC %d detected on the old channel [Expected %d]\n",
                 currentCountC, TEST_INITIAL_COUNTC);
        return NULL;
    }

    /***********************************************************************************
     * COUNTC Validations: New LTE Channel. All the packets have been dropped so the
     * countC should still remain the same
     * NOTE: CountC are always 1 less than the expected value.
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (reestablishedUeHandle, rbIndex, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        printf ("Error: Failed to get countC on reestablished user [Error code %d]\n", errCode);
        return NULL;
    }
    currentCountC = currentCountC + 1;
    if (currentCountC != appCountC)
    {
        /* Error: Unexpected countC detected */
        printf ("Error: Unexpected countC %d detected on the new channel [Expected %d]\n",
                 currentCountC, appCountC);
        return NULL;
    }

    /***********************************************************************************
     * Extended statistics validations: Old LTE Channel Handle should have no drops
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_EXTENDED_STATISTICS;
    optCfg.length = sizeof(Netfp_ExtendedSocketStats);
    optCfg.value  = (void*)&extendedSocketStats;
    if (Netfp_getSockOpt (lteDRBHandle, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Failed to get extended socket statistics [Error code %d]\n", errCode);
        return NULL;
    }

    /* Sanity Check: Ensure that the counters are valid */
    if (extendedSocketStats.reEstDroppedPackets != 0)
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Unexpected value in the extended socket statistics for the old LTE Channel %d detected Expected %d\n",
                 (uint32_t)extendedSocketStats.reEstDroppedPackets, 0);
        return NULL;
    }

    /***********************************************************************************
     * Extended statistics validations: New LTE Channel Handle should have drops
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_EXTENDED_STATISTICS;
    optCfg.length = sizeof(Netfp_ExtendedSocketStats);
    optCfg.value  = (void*)&extendedSocketStats;
    if (Netfp_getSockOpt (reestablishedLteDRBChannel, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Failed to get extended socket statistics [Error code %d]\n", errCode);
        return NULL;
    }

    /* Sanity Check: Ensure that the counters are valid */
    if (extendedSocketStats.reEstDroppedPackets != (numS1Packets - numS1SuspendedPackets))
    {
        /* Error: Failed to get Extended Socket Statistics */
        printf ("Error: Unexpected value in the extended socket statistics for the new LTE Channel %d detected Expected %d\n",
                 (uint32_t)extendedSocketStats.reEstDroppedPackets, (numS1Packets - numS1SuspendedPackets));
        return NULL;
    }
    printf ("Debug: Basic Reestablishment Test Passed\n");

    /***********************************************************************************
     * Test Cleanup:
     ***********************************************************************************/
    printf ("Debug: Encode Queue %d packets\n", Qmss_getQueueEntryCount(encodeQueue));
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(encodeQueue));
    while (ptrPkt != NULL)
    {
        /* Cleanup the packet: */
        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(encodeQueue));
    }
    printf ("Debug: Decode Queue %d packets\n", Qmss_getQueueEntryCount(decodeQueue));
    ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(decodeQueue));
    while (ptrPkt != NULL)
    {
        /* Cleanup the packet: */
        Pktlib_freePacket (appPktlibInstanceHandle, ptrPkt);
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(encodeQueue));
    }

    /* Delete the users: */
    if (Netfp_deleteUser (ueHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the user [Error code %d]\n", errCode);
        return NULL;
    }
    if (Netfp_deleteUser (reestablishedUeHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the user [Error code %d]\n", errCode);
        return NULL;
    }
    return NULL;
}

