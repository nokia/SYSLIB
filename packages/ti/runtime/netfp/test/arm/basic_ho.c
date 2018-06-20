/*
 *   @file  basic_ho.c
 *
 *   @brief
 *      The file implements the testing of the basic handover functionality
 *      for Fast Path channels.
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

/* MCSDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/drv/rm/rm.h>

/* Benchmarking Include Files */
#include "benchmark.h"

/**********************************************************************
 ************************ Unit Test Definitions ***********************
 **********************************************************************/

/* Test User Radio Bearer Identifier.  */
#define TEST_USER_RADIO_BEARER_ID           7

/* Test User User Equipment Identifier.  */
#define TEST_USER_UE_ID                     10

/* Test CountC */
#define TEST_START_COUNTC                   4931

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_completeTargetHandOver)

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle   netfpDataRxHeap;

/* Test completion status */
extern uint32_t            testComplete;

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle    appPktlibInstanceHandle;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle    appMsgcomInstanceHandle;

/* Statistics Display Functionality. */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_basicHOTask(void* arg)
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
    uint32_t                    numS1Packets;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    uint32_t                    gtpuIdentifier = 0xdead12;
    uint8_t*                    ptrDataBuffer;
    uint32_t                    dataBufferLen;
    uint32_t                    targetHandOverId;
    int32_t                     retVal;
    uint32_t                    numPackets;
    uint32_t                    countC;
    Pktlib_HeapStats            heapStats;

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

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "SocketTestFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = netfpDataRxHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Get the heap statistics: */
    Pktlib_getHeapStats (netfpDataRxHeap, &heapStats);

    /************************************************************************************
     * Encode Channel: This is the channel where the packets will be encoded invoking the
     * NETFP encoding API.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    encodeChannel = Msgcom_create ("Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (encodeChannel == 0)
    {
        printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return NULL;
    }

    /************************************************************************************
     * Decode Channel: This is the channel where the packets will be present once they
     * have been decoded.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    decodeChannel = Msgcom_create ("Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (decodeChannel == 0)
    {
        printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return NULL;
    }

    /************************************************************************************
     * Create the LTE User:
     ************************************************************************************/

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
    userCfg.ueId             = TEST_USER_UE_ID;
    userCfg.srbFlowId        = rxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb1Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    ueHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
    if (ueHandle == NULL)
    {
        /* User Security Context creation failed. */
        printf ("Error: LTE Creation User Security Context failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: LTE User %d has been created\n", userCfg.ueId);

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = rxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = (Qmss_QueueHnd)NULL;
    lteChannelBindCfg.fpHandle        = fpIngressV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 20;
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
        printf ("Error: Failed to create the LTE channel [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: LTE channel %p has been created\n", lteDRBChannel);

    /************************************************************************************
     * Generating Packets on the S1 Link:
     ************************************************************************************/
    printf ("*******************************************************************************************\n");
    printf ("Please execute the BURST packet generator to sent GTPU Packets with Id: 0x%x\n", gtpuIdentifier);
    printf ("All packets should be of the same size (128 bytes) since the test will validate\n");
    printf ("and ensure that the COUNTC is correct. The packets received on the S1 Link\n");
    printf ("will be buffered until the HO procedure is complete. Heap 0x%p is used to\n", netfpDataRxHeap);
    printf ("receive the data and is configured to buffer %d packets\n", heapStats.numFreeDataPackets);
    printf ("*******************************************************************************************\n");
    printf ("Please enter the number of packets AFTER the BURST Packet Generator has been executed\n");
    printf ("Enter the number of packets which had been generated on the S1 Link: ");
    scanf ("%d", &numS1Packets);

    /* Initialize the countC for the tests: */
    countC = TEST_START_COUNTC;
    printf ("Debug: Initiating the Target HO Procedure [CountC is %d]\n", countC);

    /********************************************************************************************
     * Target Handover Procedure:
     ********************************************************************************************/
    BENCHMARK_START(Netfp_completeTargetHandOver);
    retVal = Netfp_completeTargetHandOver (ueHandle, TEST_USER_RADIO_BEARER_ID, countC, &targetHandOverId, &errCode);
    BENCHMARK_END(Netfp_completeTargetHandOver);
    if (retVal < 0)
    {
        /* Error: Initiating the Target HO Failed */
        printf ("Error: Initiating the Target HO failed [Error code %d]\n", errCode);
        return NULL;
    }
    BENCHMARK_UPDATE(Netfp_completeTargetHandOver);
    usleep(100);
    printf ("Debug: Target HO Procedure has been completed\n");

    while (1)
    {
        /* Check if there are packets buffered that have to be processed */
        retVal = Netfp_getTargetHandOverPackets (targetHandOverId, &ptrRxPkt, &errCode);
        if (retVal < 0)
        {
            /* Error: Getting the buffered packets. */
            printf ("Error: Getting target HandOver buffered packets failed [Error code %d]\n", errCode);
            return NULL;
        }
        if (ptrRxPkt != NULL)
        {
            printf ("Error: Packets were buffered. This should not have happened in Fast Path\n");
            return NULL;
        }
        break;
    }
    printf ("Debug: Target Handover packets have been handled\n");

    /********************************************************************************************
     * Validations: All the packets on the S1 Link should have ended up on the Encoded Channel
     * after these have been ciphered. Here we ensure that we received all the packets from S1.
     * We will also send these packets to be deciphered
     ********************************************************************************************/
    numPackets = 0;
    countC     = TEST_START_COUNTC;
    while (1)
    {
        uint32_t    rxCountC;

        /* Process all packets on the encode channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Increment the number of received packets: */
        numPackets++;

        /* Sanity Check: Encoded packet will have the COUNTC added to it. */
        if (Pktlib_getPacketLen(ptrRxPkt) != (128 + sizeof(countC)))
        {
            printf ("Error: Invalid data buffer length detected in encoded packet\n");
            return NULL;
        }

        /* Get the data buffer & length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Get the countC from the received data buffer and convert it to host order */
        rxCountC = *(uint32_t*)ptrDataBuffer;
        rxCountC = Netfp_ntohl (rxCountC);

        /* Sanity Check: Validate the countC */
        if (rxCountC != countC)
        {
            printf ("Error: Incorrect countC received %d expected %d\n", rxCountC, (countC + numPackets));
            return NULL;
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
            printf ("Error: Deciphering the packet failed [Error code %d]\n", retVal);
            return NULL;
        }

        /* Increment the countC */
        countC++;
    }

    /* We should have got the same number of packets */
    if (numPackets != numS1Packets)
    {
        printf ("Error: Sent %d packets on the S1 Link but received only %d packets\n", numS1Packets, numPackets);
        return NULL;
    }
    printf ("Debug: Validated packets on the encoded channel\n");

    /********************************************************************************************
     * Validations: After deciphering we will validate to ensure that the S1 Packets are correct
     * and received in order.
     ********************************************************************************************/
    numPackets = 0;
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
            printf ("Error: Invalid data buffer length detected in decoded packet\n");
            return NULL;
        }

        /* Validate the data payload: */
        ptrPayload  = (uint32_t*)ptrDataBuffer;
        if (*ptrPayload != 0xdeadbeef)
        {
            printf ("Error: Invalid signature 0x%x detected in the decoded packet\n", *ptrPayload);
            return NULL;
        }

        /* Validate: Packet Ordering */
        if (*(ptrPayload + 1) != numPackets)
        {
            printf ("Error: Packet Order mismatch detected in the decoded packet [Expected %d Got %d]\n", numPackets, *(ptrPayload + 1));
            return NULL;
        }

        /* Increment the number of received packets: */
        numPackets++;

        /* Cleanup the packet: */
        Pktlib_freePacket (appPktlibInstanceHandle, ptrRxPkt);
    }

    /* We should have got the same number of packets */
    if (numPackets != numS1Packets)
    {
        printf ("Error: Sent %d packets on the S1 Link but deciphered only %d packets\n", numS1Packets, numPackets);
        return NULL;
    }
    printf ("Debug: Validated packets on the decoded channel\n");

    /* Display the results: */
    BENCHMARK_DISPLAY(Netfp_completeTargetHandOver);
    printf ("Debug: Handover Tests are complete\n");

    /* Shutdown the LTE channel */
    if (Netfp_deleteLTEChannel (lteDRBChannel, &errCode) < 0)
    {
	    printf ("Error: NETFP LTE DRB channel deletion failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: NETFP LTE DRB channel deletion was successful\n");

    /* Delete the user. */
    if (Netfp_deleteUser (ueHandle, &errCode) < 0)
    {
	    printf ("Error: NETFP LTE User deletion failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: NETFP LTE User deletion was successful\n");

    errCode = Msgcom_delete (encodeChannel, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM Encode channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM Encode channel deleted successfully\n");
    errCode = Msgcom_delete (decodeChannel, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM Decode channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM Decode channel deleted successfully\n");

    /* TODO: Shutdown the NETFP Flow */

    return NULL;
}

