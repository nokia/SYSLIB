/*
 *   @file  sourceHandOver.c
 *
 *   @brief
 *      The file implements the following functionality for the LTE channels:
 *          - Source HandOver for fast path radio bearers
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2015 Texas Instruments, Inc.
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

/* Unit Test Definitions which describe the state of the handover in the test */
#define TEST_HANDOVER_NOT_INITIATED         0
#define TEST_HANDOVER_INITIATED             1
#define TEST_HANDOVER_COMPLETE              2

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global Variable which is set to initiate the handover */
uint32_t        gInitiateSourceHO = TEST_HANDOVER_NOT_INITIATED;

/* Global Counter for number of HandOver received after the handover is initiated */
static uint32_t        gNumHOPktRxed = 0;

/* Global Counter for number of LTE IPv4 GTPU Packets transmitted */
static uint32_t        gNumPktTxed = 0;

/* Global Counter which keeps track of the number of packets which have been decoded */
static uint32_t        gNumPktDecodedPktRxed = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_initiateSourceHandOver)

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
extern Pktlib_InstHandle        appPktlibInstanceHandle;

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
 *      Display the statistics for the DRB Source HO tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_displayDrbSourceHOStats(void)
{
    printf ("-------------------------------------------------------------\n");
    printf ("LTE Source HandOver Tests::::\n");
    printf ("Source HandOver Status               : %s\n",
                    (gInitiateSourceHO == 0 ? "HO not initiated" : (gInitiateSourceHO == 2 ? "HO Completed" : "HO failed")));
    printf ("Number of IPv4 GTPU Pkt Decoded      : %d\n", gNumPktDecodedPktRxed);
    printf ("Number of IPv4 GTPU Pkt Transmitted  : %d\n", gNumPktTxed);
    printf ("Number of IPv4 HandOver Pkt Received : %d\n", gNumHOPktRxed);
    BENCHMARK_DISPLAY(Netfp_initiateSourceHandOver);
    return;
}


/**
 *  @b Description
 *  @n
 *      The function is used to decode the received packet.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *  @param[in]  decodeChannel
 *      Channel handle where the decoded packet is to be placed
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_decodePacket
(
    Netfp_UserHandle    ueHandle,
    Ti_Pkt*             ptrRxPkt,
    MsgCom_ChHandle     decodeChannel
)
{
    uint8_t*    ptrRxDataBuffer;
    uint32_t    rxDataBufferLen;
    int32_t     retVal;
    uint32_t    countC;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Get the countC associated with the packet. */
    countC = Netfp_ntohl(*(uint32_t *)ptrRxDataBuffer);

    /* Now we need to skip the countC */
    Pktlib_setPacketLen(ptrRxPkt, Pktlib_getPacketLen(ptrRxPkt) - 4);
    Pktlib_setDataOffset (ptrRxPkt, 4);

    /* We use the cipher function to decode the packet. */
    retVal = Netfp_cipher (ueHandle, TEST_USER_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, countC,
                           Msgcom_getInternalMsgQueueInfo(decodeChannel));

    /* Is there an error while encoding the data? */
    if (retVal < 0)
    {
        printf ("Error: Decoding data failed [Error code %d]\n", retVal);
        return;
    }
    else
        gNumPktDecodedPktRxed++;

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send out the packet.
 *
 *  @param[in]  lteChannel
 *      LTE 3GPP Channel Handle on which the packet is to be sent
 *  @param[in]  ptrRxPkt
 *      Pointer to the packet which has been received
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_sendPacket (Netfp_SockHandle lteChannel, Ti_Pkt* ptrRxPkt)
{
    int32_t     retVal;
    int32_t     errCode;

    /* Send out the data on the appropriate channel: Take timestamps also. */
    retVal = Netfp_send (lteChannel, ptrRxPkt, 0x0, &errCode);
    if (retVal < 0)
    {
        printf ("Error: Sending data failed [Error code %d]\n", errCode);
        return;
    }

    /* Increment the statistics. */
    gNumPktTxed++;

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called to process a packet which was received on the HandOver
 *      channel once the handover is initiated. The packet is not an encoded
 *      packet and so here we simply send it back to the test equipment.
 *
 *  @param[in]  lteHoSockHandle
 *      3GPP channel Handle on which the packet was received
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static void Test_processHandOverPacket (Netfp_SockHandle lteHoSockHandle, Ti_Pkt* ptrRxPkt)
{
    /* Increment the statistics. */
    gNumHOPktRxed++;

    /* Send the packet out. */
    Test_sendPacket(lteHoSockHandle, ptrRxPkt);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the source handover request. The function
 *      will move the channel to the HandOver mode. Once handover is initiated;
 *      there should be NO packet coming to the fast path encoded channel.
 *
 *  @param[in]  ueHandle
 *      User Security Context Handle which is being handed over.
 *  @param[in]  encodeChannel
 *      Encoded Channel where all encoded packets from the fast path will arrive
 *  @param[in]  decodeChannel
 *      This is the channel where the packets will end up once they have been
 *      decoded back
 *  @param[in]  sourceHandOverChannel
 *      This is the source HandOver channel where all newly received packets will endup
 *      These packets will not be "encrypted" but will arrive in plain text.
 *  @param[in]  rxFlowId
 *      Receive flow rxFlowId to be used to receive packets and place them into the
 *      sourceHandOverChannel queue.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_processSourceHandOver
(
    Netfp_UserHandle        ueHandle,
    MsgCom_ChHandle         encodeChannel,
    MsgCom_ChHandle         decodeChannel,
    MsgCom_ChHandle         sourceHandOverChannel,
    int32_t                 rxFlowId
)
{
    Ti_Pkt*     ptrRxPkt;
    int32_t     retVal, errCode;

    /* initiate source HandOver */
    BENCHMARK_START(Netfp_initiateSourceHandOver);
    retVal = Netfp_initiateSourceHandOver (ueHandle, TEST_USER_RADIO_BEARER_ID, rxFlowId,
                                         Msgcom_getInternalMsgQueueInfo(sourceHandOverChannel),
                                         &errCode);
    BENCHMARK_END(Netfp_initiateSourceHandOver);

    /* Were we able to initiate source HandOver? */
    if (retVal < 0)
    {
        /* Error: initiate source HandOver failed. */
        printf ("Error: Initiate source HandOver failed\n");
        return -1;
    }

    /* Update benchmarking. */
    BENCHMARK_UPDATE(Netfp_initiateSourceHandOver);

    /**************************************************************************
     * All new packets arriving will now end up in the HandOver channel.
     * However there might still be some packets which are already in the
     * ENCODED Channel. These packets have already been encoded and need to
     * be processed. In reality these packets would get sent to the Target
     * enodeB but in our test we will send "decode" the packets and send them
     * back to the Test Equipment.
     *************************************************************************/
    while (1)
    {
        /* Process all packets on the encode channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Decode the packet; this will now end up in the DECODED Channel and
         * will then be sent back to the Test Equipment.  */
        Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
    }

    /* Source Handover processed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void* Test_drbSourceHOTask(void* arg)
{
    Netfp_InboundFPHandle       fpIngressV4Handle;
    Netfp_OutboundFPHandle      fpEgressV4Handle;
    Msgcom_ChannelCfg           chConfig;
    Netfp_UserCfg               userCfg;
    Netfp_UserHandle            ueHandle;
    uint8_t                     encryptionKey[16];
    uint8_t                     integrityKey[16];
    MsgCom_ChHandle             HOChannel;
    MsgCom_ChHandle             encodeChannel;
    MsgCom_ChHandle             decodeChannel;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     index;
    int32_t                     errCode;
    Ti_Pkt*                     ptrRxPkt;
    Netfp_SockHandle            lteDRBChannel;
    Netfp_SockHandle            lteHoSockHandle;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    uint32_t                    gtpuIdentifier = 0xdead12;
    Netfp_SockAddr              sockAddress;

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

    /************************************************************************************
     * GTPU Channel: This is the channel where the GTPU packets will be received.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    HOChannel = Msgcom_create ("HandOver", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (HOChannel == 0)
    {
        printf ("Error: Unable to open the channel Error : %d\n", errCode);
        return NULL;
    }

    /* Open a UDP socket to send GTPU unciphered data from Source ENB to Target eNB
     * Here we are simulating by sending the packets to serving gateway
     */
	lteHoSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (lteHoSockHandle == NULL)
	{
        printf ("Error: Failed to create the LTE HO socket to receive data from Source eNB [Error code %d]\n", errCode);
	    return NULL;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = 2152;
	sockAddress.sin_gtpuId              = 0;
    sockAddress.op.bind.inboundFPHandle = fpIngressV4Handle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;

    /* Bind the socket. */
	if (Netfp_bind (lteHoSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: Failed to bind the LTE HO channel [Error code %d]\n", errCode);
        return NULL;
    }

    /* Populate the connect information. */
    sockAddress.op.connect.outboundFPHandle = fpEgressV4Handle;

    /* Connect the socket to the remote peer. */
    if (Netfp_connect(lteHoSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: Failed to connect the LTE HO channel [Error code %d]\n", errCode);
        return NULL;
    }

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
        printf ("Error: Failed to create the LTE channel [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: LTE channel %p has been created\n", lteDRBChannel);

    /* Debug Message: */
    printf ("---------------------------------------------------------------------------------------------\n");
    printf ("Please use a traffic generation application to send packets to the Fast Path GTPU Id 0x%x\n", gtpuIdentifier);
    printf ("---------------------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        /* Initiate HO after 100 packets */
        if ((gNumPktDecodedPktRxed == 100) && (gInitiateSourceHO == TEST_HANDOVER_NOT_INITIATED))
        {
            gInitiateSourceHO = TEST_HANDOVER_INITIATED;
            printf ("Initiating Source HANDOVER....\n");
        }

        /* Is HO activated? */
        if (gInitiateSourceHO == TEST_HANDOVER_INITIATED)
        {
            /* Process the Handover request. */
            if (Test_processSourceHandOver(ueHandle, encodeChannel, decodeChannel, HOChannel, rxFlowId) < 0)
            {
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                continue;
            }

            /* Once handover has been initiated; there is NO going back. This is a one time test
             * and here we set the flag to indicate that the handover is complete. */
            gInitiateSourceHO = TEST_HANDOVER_COMPLETE;
            printf ("Source HAND OVER Complete....\n");
        }

        /* Check if there is a message on the ENCODE channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Once handover is complete. We should have never received a packet on
             * the "ENCODED Channel" since all these packets should end up in the SLOW
             * Path Channel */
            if (gInitiateSourceHO == TEST_HANDOVER_COMPLETE)
            {
                printf ("Error: Received Packet 0x%p in FP Encode Channel (After HO)\n", ptrRxPkt);
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                continue;
            }

            /* Packet was ciphered using the new security context. Proceed normally */
            Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
        }

        /* NO message on the encode channel: Check if there is a message on the DECODE channel */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Send the packet out */
            Test_sendPacket(lteDRBChannel, ptrRxPkt);
        }

        /* Check if there is a message on the HO channel */
        Msgcom_getMessage (HOChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* We should receive packets on the HO channel only after the handover is COMPLETE
             * In all other scenarios we should NOT receive a packet. */
            if (gInitiateSourceHO != TEST_HANDOVER_COMPLETE)
            {
                printf ("Error: Received Packet 0x%p in HO Channel (Before HO)\n", ptrRxPkt);
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                continue;
            }

            /* The application should send these packets to the Target eNB by adding the GTPU extension header themselves.
             * Hence we use the GTPU socket opened above.
             * These packets are NOT encoded and here we are simulating the transmission of these packets
             * to the Target eNodeB.*/
            Test_processHandOverPacket (lteDRBChannel, ptrRxPkt);
        }
        else
        {
            /* Control comes here implies that there was no message on neither channels. Relinquish time */
            usleep(10);
        }
    }

    /* Test has completed execution. Display the statistics */
    Test_displayDrbSourceHOStats ();

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

    /* Shutdown the MSGCOM channel(s) */
    errCode = Msgcom_delete (HOChannel, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM HandOver channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM HandOver channel deleted successfully\n");
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

    /* Shutdown the socket */
    printf ("Debug: Closing the Socket\n");
    if (Netfp_closeSocket (lteHoSockHandle, &errCode) < 0)
    {
	    printf ("Error: NETFP socket close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: NETFP Socket closed successfully\n");

    /* TODO: Shutdown the NETFP Flow */

    return NULL;
}

