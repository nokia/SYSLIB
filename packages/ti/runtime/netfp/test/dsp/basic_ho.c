/*
 *   @file  basic_ho.c
 *
 *   @brief
 *      The test file implements the basic handover functionality. The
 *      test expects packets to be sent from the S1 Link. It simulates
 *      packets received from the X2 Link and ensures that the packets
 *      are ordered and encoded correctly without any drops.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014-2015 Texas Instruments, Inc.
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
#include "netCfg.h"
#include "net.h"

/**********************************************************************
 ************************ Unit Test Definitions ***********************
 **********************************************************************/

/* Test User Radio Bearer Identifier.  */
#define TEST_USER_RADIO_BEARER_ID           7

/* Test User User Equipment Identifier.  */
#define TEST_USER_UE_ID                     10

/* Test S1 countC */
#define TEST_S1_COUNTC                      1629

/* Test X2 countC */
#define TEST_X2_COUNTC                      9999

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_completeTargetHandOver)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* Statistics Display Functionality. */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

/* NETFP Configuration after parsing the DAT file. */
extern Test_NetfpConfigInfo    netfpConfig;

/* Packet Generation API: */
extern int32_t Test_netSendIPv4Payload
(
    uint8_t*    ptrPayload,
    uint16_t    payloadLen,
    uint32_t    gtpuId,
    uint16_t    gtpuMsgType,
    uint16_t    dstPort,
    uint16_t    srcPort,
    uint8_t*    dstIP,
    uint8_t*    srcIP,
    uint8_t     tos,
    uint8_t*    dstMAC,
    uint8_t*    srcMAC,
    uint32_t    loopback
);

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
void Test_basicHOTask(UArg arg0, UArg arg1)
{
    Netfp_InboundFPHandle       fpIngressV4Handle;
    Netfp_OutboundFPHandle      fpEgressV4Handle;
    Msgcom_ChannelCfg           chConfig;
    Netfp_UserCfg               userCfg;
    Netfp_UserHandle            ueHandle;
    uint8_t                     encryptionKey[16];
    uint8_t                     integrityKey[16];
    MsgCom_ChHandle             encodeChannel;
    MsgCom_ChHandle             decodeChannel;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     index;
    int32_t                     errCode;
    Ti_Pkt*                     ptrRxPkt;
    Netfp_SockHandle            lteDRBChannel;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    uint32_t                    numS1Packets;
    uint32_t                    numX2Packets = 2;
    uint32_t                    gtpuIdentifier = 0xdead12;
    uint8_t*                    ptrDataBuffer;
    uint32_t                    dataBufferLen;
    uint32_t                    targetHandOverId;
    int32_t                     retVal;
    uint32_t                    numDetectedS1Packets;
    uint32_t                    numDetectedX2Packets;
    uint32_t                    countC;
    Pktlib_HeapStats            heapStats;
    Netfp_OptionTLV             optCfg;
    uint32_t                    currentCountC;
    Qmss_QueueHnd               X2Queue;
    uint8_t                     isAllocated;

    /* Get the ingress and egress IPv4 fast paths. */
    fpIngressV4Handle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressV4Handle == NULL)
    {
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }
    fpEgressV4Handle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressV4Handle == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Open the X2 Buffering queue: */
    X2Queue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (X2Queue < 0)
        return;

    /* Populate the flow configuration: */
    strcpy (flowCfg.name, "HOTestFlow");
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

    /* Get the heap statistics associated with the MTU Heap. */
    Pktlib_getHeapStats (mtuReceiveHeap, &heapStats);

    /************************************************************************************
     * Encode Channel: This is the channel where the packets will be encoded invoking the
     * NETFP encoding API.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the Message communicator channel. */
    encodeChannel = Msgcom_create ("Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (encodeChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }
    System_printf ("Debug: Encode Queue 0x%x\n", Msgcom_getInternalMsgQueueInfo(encodeChannel));

    /************************************************************************************
     * Decode Channel: This is the channel where the packets will be present once they
     * have been decoded.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[1].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[1].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[1].hostInterrupt;

    /* Create the Message communicator channel. */
    decodeChannel = Msgcom_create ("Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (decodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }
    System_printf ("Debug: Decode Queue 0x%x\n", Msgcom_getInternalMsgQueueInfo(decodeChannel));

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
    userCfg.ueId             = TEST_USER_UE_ID;
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
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    if (ueHandle == NULL)
    {
        /* User Security Context creation failed. */
        System_printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: LTE User %d has been created\n", userCfg.ueId);

    /* Initialize the bind configuration. */
    memset ((void *)&lteChannelBindCfg, 0, sizeof(Netfp_LTEChannelBindCfg));

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = NULL;
    lteChannelBindCfg.fpHandle        = fpIngressV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 10;
    lteChannelBindCfg.enableFastPath  = 1;
    lteChannelBindCfg.isHOInProgress  = 1;
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

    /************************************************************************************
     * Generating Packets on the S1 Link:
     ************************************************************************************/
    System_printf ("*******************************************************************************************\n");
    System_printf ("Please execute the BURST packet generator to sent GTPU Packets with Id: 0x%x\n", gtpuIdentifier);
    System_printf ("All packets should be of the same size (128 bytes) since the test will validate\n");
    System_printf ("and ensure that the COUNTC is correct. The packets received on the S1 Link\n");
    System_printf ("will be buffered until the HO procedure is complete. Heap 0x%x is used to\n", mtuReceiveHeap);
    System_printf ("receive the data and is configured to buffer %d packets\n", heapStats.numFreeDataPackets);
    System_printf ("*******************************************************************************************\n");
    System_printf ("Please enter the number of packets AFTER the BURST Packet Generator has been executed\n");
    System_printf ("Enter the number of packets which had been generated on the S1 Link: ");
    scanf ("%d", &numS1Packets);
    Task_sleep(100);

    /***********************************************************************************
     * COUNTC Validations: Get the initial countC after the LTE channel has been created
     * Since no packets have passed through the LTE channel it is should be the same as
     * initially configured.
     * NOTE: CountC are always 1 less than the expected value.
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (ueHandle, TEST_USER_RADIO_BEARER_ID, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        System_printf ("Error: Failed to get countC on old UE [Error code %d]\n", errCode);
        return;
    }
    currentCountC = currentCountC + 1;
    if (currentCountC !=  lteChannelBindCfg.countC)
    {
        System_printf ("Error: Unexpected Initial countC %d detected [Expected %d]\n", currentCountC, lteChannelBindCfg.countC);
        return;
    }

    /***********************************************************************************************************
     * Generating packets on the X2 Link:
     ***********************************************************************************************************/
    countC = TEST_X2_COUNTC;
    System_printf ("Simulating %d packets received from the Source eNB\n", numX2Packets);
    for (index = 0; index < numX2Packets; index++)
    {
        /* Allocate a packet: */
        ptrRxPkt = Pktlib_allocPacket (appPktlibInstanceHandle, mtuReceiveHeap, 128);
        if (ptrRxPkt == NULL)
        {
            System_printf ("Error: Unable to allocate X2 Packets\n");
            return;
        }

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Populate the X2 Data Buffer: */
        *(uint32_t*)ptrDataBuffer       = 0xdeaddead;
        *((uint32_t*)ptrDataBuffer + 1) = index;

        /* Writeback the data buffer and release ownership of the packet: */
        appWritebackBuffer (ptrDataBuffer, dataBufferLen);
        Pktlib_releaseOwnership (appPktlibInstanceHandle, ptrRxPkt);

        /* Push the packet into the X2 queue */
        Qmss_queuePushDescSize (X2Queue, ptrRxPkt, 128);
    }

    /********************************************************************************************
     * Target Handover Procedure:
     *      Step 1: Handle all the packets from the X2 Link
     *      Step 2: Handle all the packets from the S1 Link
     *
     * We start by handling all the packets on the X2 Link.
     ********************************************************************************************/
    numDetectedX2Packets = 0;
    while (1)
    {
        /* Pop off the packet */
        ptrRxPkt = (Ti_Pkt *)QMSS_DESC_PTR(Qmss_queuePop(X2Queue));
        if (ptrRxPkt == NULL)
            break;

        /* X2 Packets are plain text: So the application needs to manually encode these packets */
        errCode = Netfp_encodeDrb (ueHandle, ptrRxPkt, countC++, TEST_USER_RADIO_BEARER_ID);
        if (errCode < 0)
        {
            /* Error: Encode the DRB failed */
            System_printf ("Error: Unable to encode the DRB [Error code %d]\n", errCode);
            return;
        }

        /* Increment the number of packets processed. */
        numDetectedX2Packets++;
    }
    if (numDetectedX2Packets != numX2Packets)
    {
        System_printf ("Error: Generated %d X2 Packets but only %d packets detected\n", numX2Packets, numDetectedX2Packets);
        return;
    }

    /***********************************************************************************
     * COUNTC Validations: Get the countC from the UE this should now account for the
     * additional X2 packets which have been handled.
     * NOTE: CountC are always 1 less than the expected value.
     ***********************************************************************************/
    optCfg.type   = Netfp_Option_COUNTC;
    optCfg.length = 4;
    optCfg.value  = (void*)&currentCountC;
    if (Netfp_getLTEChannelOpt (ueHandle, TEST_USER_RADIO_BEARER_ID, &optCfg, &errCode) < 0)
    {
        /* Error: Failed to get countC value. */
        System_printf ("Error: Failed to get countC on old UE [Error code %d]\n", errCode);
        return;
    }
    currentCountC = currentCountC + 1;
    if (currentCountC != (TEST_X2_COUNTC + numX2Packets))
    {
        System_printf ("Error: Unexpected countC %d detected after X2 [Expected %d]\n", currentCountC, TEST_X2_COUNTC + numX2Packets);
        return;
    }

    countC = TEST_S1_COUNTC;
    System_printf ("Debug: Initiating the Target HO Procedure [CountC is %d]\n", countC);

    /********************************************************************************************
     * Target Handover Procedure:
     *  Step (2): We handle all the buffered packets on the S1 Link
     ********************************************************************************************/
    BENCHMARK_START(Netfp_completeTargetHandOver);
    retVal = Netfp_completeTargetHandOver (ueHandle, TEST_USER_RADIO_BEARER_ID, countC, &targetHandOverId, &errCode);
    BENCHMARK_END(Netfp_completeTargetHandOver);
    if (retVal < 0)
    {
        /* Error: Initiating the Target HO Failed */
        System_printf ("Error: Initiating the Target HO failed [Error code %d]\n", errCode);
        return;
    }
    BENCHMARK_UPDATE(Netfp_completeTargetHandOver);
    Task_sleep(100);
    System_printf ("Debug: Target HO Procedure has been completed\n");

    while (1)
    {
        /* Check if there are packets buffered that have to be processed */
        retVal = Netfp_getTargetHandOverPackets (targetHandOverId, &ptrRxPkt, &errCode);
        if (retVal < 0)
        {
            /* Error: Getting the buffered packets. */
            System_printf ("Error: Getting target HandOver buffered packets failed [Error code %d]\n", errCode);
            return;
        }
        if (ptrRxPkt != NULL)
        {
            System_printf ("Error: Packets were buffered. This should not have happened in Fast Path\n");
            return;
        }
        break;
    }
    System_printf ("Debug: Target Handover packets have been handled\n");

    /********************************************************************************************
     * Validations: All the packets on the X2 Link should have ended up on the Encoded Channel
     * after these have been ciphered. Here we ensure that we received all the packets from X2.
     * We will also send these packets to be deciphered. Remember that X2 Packets need are handled
     * before S1 packets.
     ********************************************************************************************/
    numDetectedX2Packets = 0;
    countC               = TEST_X2_COUNTC;
    while (numDetectedX2Packets < numX2Packets)
    {
        uint32_t    rxCountC;

        /* Process all packets on the encode channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Increment the number of received packets: */
        numDetectedX2Packets++;

        /* Sanity Check: Encoded packet will have the COUNTC added to it. */
        if (Pktlib_getPacketLen(ptrRxPkt) != (128 + sizeof(countC)))
        {
            System_printf ("Error: Invalid data buffer length detected in encoded packet\n");
            return;
        }

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Invalidate the data buffer: */
        appInvalidateBuffer (ptrDataBuffer, dataBufferLen);

        /* Get the countC from the received data buffer and convert it to host order */
        rxCountC = *(uint32_t*)ptrDataBuffer;
        rxCountC = Netfp_ntohl (rxCountC);

        /* Sanity Check: Validate the countC */
        if (rxCountC != countC)
        {
            System_printf ("Error: Incorrect countC received %d expected %d\n", rxCountC, (countC + numDetectedX2Packets));
            return;
        }

        /* The packet has been ciphered by the NETCP. We will now decipher it to ensure that the contents of
         * the packet are correct. Deciphered packets are placed into the Decode Channel. Skip the COUNTC
         * before performing the Deciphering operation: */
        Pktlib_setPacketLen(ptrRxPkt, Pktlib_getPacketLen(ptrRxPkt) - sizeof(countC));
        Pktlib_setDataOffset (ptrRxPkt, sizeof(countC));
        retVal = Netfp_cipher (ueHandle, TEST_USER_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, countC,
                               Msgcom_getInternalMsgQueueInfo(decodeChannel));
        if (retVal < 0)
        {
            System_printf ("Error: Deciphering the packet failed [Error code %d]\n", retVal);
            return;
        }

        /* Increment the countC */
        countC++;
    }
    System_printf ("Debug: Validated X2 packets on the encoded channel\n");

    /********************************************************************************************
     * Validations: All the packets on the S1 Link should have ended up on the Encoded Channel
     * after these have been ciphered. Here we ensure that we received all the packets from S1.
     * We will also send these packets to be deciphered. Remember that X2 Packets need are handled
     * before S1 packets.
     ********************************************************************************************/
    numDetectedS1Packets = 0;
    countC               = TEST_S1_COUNTC;
    while (1)
    {
        uint32_t    rxCountC;

        /* Process all packets on the encode channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Increment the number of received packets: */
        numDetectedS1Packets++;

        /* Sanity Check: Encoded packet will have the COUNTC added to it. */
        if (Pktlib_getPacketLen(ptrRxPkt) != (128 + sizeof(countC)))
        {
            System_printf ("Error: Invalid data buffer length detected in encoded packet\n");
            return;
        }

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Invalidate the data buffer: */
        appInvalidateBuffer (ptrDataBuffer, dataBufferLen);

        /* Get the countC from the received data buffer and convert it to host order */
        rxCountC = *(uint32_t*)ptrDataBuffer;
        rxCountC = Netfp_ntohl (rxCountC);

        /* Sanity Check: Validate the countC */
        if (rxCountC != countC)
        {
            System_printf ("Error: Incorrect countC received %d expected %d\n", rxCountC, (countC + numDetectedS1Packets));
            return;
        }

        /* The packet has been ciphered by the NETCP. We will now decipher it to ensure that the contents of
         * the packet are correct. Deciphered packets are placed into the Decode Channel. Skip the COUNTC
         * before performing the Deciphering operation: */
        Pktlib_setPacketLen(ptrRxPkt, Pktlib_getPacketLen(ptrRxPkt) - sizeof(countC));
        Pktlib_setDataOffset (ptrRxPkt, sizeof(countC));
        retVal = Netfp_cipher (ueHandle, TEST_USER_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, countC,
                               Msgcom_getInternalMsgQueueInfo(decodeChannel));
        if (retVal < 0)
        {
            System_printf ("Error: Deciphering the packet failed [Error code %d]\n", retVal);
            return;
        }

        /* Increment the countC */
        countC++;
    }

    /* We should have got the same number of packets */
    if (numDetectedS1Packets != numS1Packets)
    {
        System_printf ("Error: Sent %d packets on the S1 Link but received only %d packets\n", numS1Packets, numDetectedS1Packets);
        return;
    }
    System_printf ("Debug: Validated S1 packets on the encoded channel\n");

    /********************************************************************************************
     * Validations: After deciphering we will validate to ensure that the X2 Packets are correct
     * and received in order.
     ********************************************************************************************/
    numDetectedX2Packets = 0;
    while (numDetectedX2Packets < numX2Packets)
    {
        uint32_t* ptrPayload;

        /* Process all packets on the encode channel. */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Sanity Check: Validate the packet */
        if (dataBufferLen != 128)
        {
            System_printf ("Error: Invalid data buffer length detected in X2 decoded packet\n");
            return;
        }

        /* Invalidate the data buffer: */
        appInvalidateBuffer (ptrDataBuffer, dataBufferLen);

        /* Validate the data payload: */
        ptrPayload  = (uint32_t*)ptrDataBuffer;
        if (*ptrPayload != 0xdeaddead)
        {
            System_printf ("Error: Invalid signature 0x%x detected in the X2 decoded packet\n", *ptrPayload);
            return;
        }

        /* Validate: Packet Ordering */
        if (*(ptrPayload + 1) != numDetectedX2Packets)
        {
            System_printf ("Error: Packet Order mismatch detected in the X2 decoded packet [Expected %d Got %d]\n",
                            numDetectedX2Packets, *(ptrPayload + 1));
            return;
        }

        /* Increment the number of received packets: */
        numDetectedX2Packets++;

        /* Cleanup the packet: */
        Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
    }
    System_printf ("Debug: Validated X2 packets on the decoded channel\n");

    /********************************************************************************************
     * Validations: After deciphering we will validate to ensure that the S1 Packets are correct
     * and received in order.
     ********************************************************************************************/
    numDetectedS1Packets = 0;
    while (1)
    {
        uint32_t* ptrPayload;

        /* Process all packets on the encode channel. */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Sanity Check: Validate the packet */
        if (dataBufferLen != 128)
        {
            System_printf ("Error: Invalid data buffer length detected in decoded packet\n");
            return;
        }

        /* Invalidate the data buffer: */
        appInvalidateBuffer (ptrDataBuffer, dataBufferLen);

        /* Validate the data payload: */
        ptrPayload  = (uint32_t*)ptrDataBuffer;
        if (*ptrPayload != 0xdeadbeef)
        {
            System_printf ("Error: Invalid signature 0x%x detected in the decoded packet\n", *ptrPayload);
            return;
        }

        /* Validate: Packet Ordering */
        if (*(ptrPayload + 1) != numDetectedS1Packets)
        {
            System_printf ("Error: Packet Order mismatch detected in the decoded packet [Expected %d Got %d]\n",
                            numDetectedS1Packets, *(ptrPayload + 1));
            return;
        }

        /* Increment the number of received packets: */
        numDetectedS1Packets++;

        /* Cleanup the packet: */
        Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
    }

    /* We should have got the same number of packets */
    if (numDetectedS1Packets != numS1Packets)
    {
        System_printf ("Error: Sent %d packets on the S1 Link but deciphered only %d packets\n", numS1Packets, numDetectedS1Packets);
        return;
    }
    System_printf ("Debug: Validated S1 packets on the decoded channel\n");

    /* Display the results: */
    BENCHMARK_DISPLAY(Netfp_completeTargetHandOver);
    System_printf ("Debug: Handover Tests are complete\n");
    return;
}

