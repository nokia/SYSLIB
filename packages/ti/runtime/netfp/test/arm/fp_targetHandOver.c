/*
 *   @file  fp_targetHandOver.c
 *
 *   @brief
 *      The file implements the following functionality for the LTE channels:
 *          - Target HandOver for fast path radio bearers
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

/* Unit Test Definitions which indicate the status of the target handover. */
#define TEST_TARGET_HO_NOT_COMPLETE         0
#define TEST_TARGET_HO_COMPLETE_INITIATION  1
#define TEST_TARGET_HO_COMPLETE             2

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global counter which keeps track of the number of countC for the encoding
 * of packets. */
static uint32_t gCountC = 0;

/* Global Variable which is set to initiate the handover */
uint32_t               gInitiateFPTargetHO = TEST_TARGET_HO_NOT_COMPLETE;

/* Global Counter for number of LTE IPv4 GTPU Packets transmitted */
static uint32_t        gNumPktTxed = 0;

/* Global Counter which keeps track of the number of packets which have been decoded */
static uint32_t        gNumPktDecodedPktRxed = 0;

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
 *      Display the statistics for the DRB fast Path Target HandOver tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_displayDrbFPTargetHOStats(void)
{
    printf ("-------------------------------------------------------------\n");
    printf ("LTE FP Target HandOver Tests::::\n");
    printf ("Target HandOver Status               : %s\n",
                    (gInitiateFPTargetHO == 0 ? "HO not complete" : (gInitiateFPTargetHO == 2 ? "HO Completed" : "HO started")));
    printf ("Number of IPv4 GTPU Pkt Encoded      : %d\n", gCountC);
    printf ("Number of IPv4 GTPU Pkt Decoded      : %d\n", gNumPktDecodedPktRxed);
    printf ("Number of IPv4 GTPU Pkt Transmitted  : %d\n", gNumPktTxed);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to encode the received packet.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_encodePacket (Netfp_UserHandle ueHandle, Ti_Pkt* ptrRxPkt)
{
    uint8_t*    ptrRxDataBuffer;
    uint32_t    rxDataBufferLen;
    int32_t     retVal;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    retVal = Netfp_encodeDrb (ueHandle, ptrRxPkt, gCountC, TEST_USER_RADIO_BEARER_ID);

    /* Is there an error while encoding the data? */
    if (retVal < 0)
    {
        printf ("Error: Encoding data failed [Error code %d]\n", retVal);
        return;
    }

    /* Increment the countC */
    gCountC++;
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
 *      The function is used to process the target handover request. The function
 *      will initiate the target HandOver. Once the HO is complete no packets
 *      should come to the HO queue.
 *
 *  @param[in]  ueHandle
 *      User Security Context Handle which is being handed over.
 *  @param[in]  lteHoSockHandle
 *      Socket Handle used to receive data from source ENB.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_processTargetHandover
(
    Netfp_UserHandle        ueHandle,
    Netfp_SockHandle        lteHoSockHandle
)
{
    int32_t                 retVal, errCode;
    uint32_t                targetHandOverId;
    Ti_Pkt*                 ptrRxPkt;

    /* Initiate the target HandOver */
    retVal = Netfp_completeTargetHandOver (ueHandle, TEST_USER_RADIO_BEARER_ID, gCountC, &targetHandOverId, &errCode);

    /* Were we able to process target HandOver? */
    if (retVal < 0)
    {
        /* Error: processing target HandOver. */
        printf ("Error: processing target HandOver failed\n");
        return -1;
    }

    while (1)
    {
        /* Check if there are packets buffered that have to be processed */
        retVal = Netfp_getTargetHandOverPackets (targetHandOverId, &ptrRxPkt, &errCode);
        if (retVal < 0)
        {
            /* Error: Getting the buffered packets. */
            printf ("Error: Getting target HandOver buffered packets failed [Error code %d]\n", errCode);
            return -1;
        }
        if (ptrRxPkt == NULL)
        {
            printf ("Debug: Target HandOver is complete\n");
            break;
        }
        else
        {
            printf ("Error: Packets were buffered. This should not have happened in Fast Path\n");
        }
    }

    /* Delete the HO channel */
    retVal = Netfp_closeSocket(lteHoSockHandle, &errCode);
    if (retVal < 0)
    {
        /* Error: Deleting LTE HO channel  failed. */
        printf ("Error: Deleting LTE HO channel failed\n");
        return -1;
    }

    /* Target Handover processed. */
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
void* Test_drbFPTargetHOTask(void* arg)
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
    MsgCom_ChHandle             hoChannel;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     index;
    int32_t                     errCode;
    Ti_Pkt*                     ptrRxPkt;
    Netfp_SockHandle            lteDRBChannel;
    Netfp_SockHandle            lteHoSockHandle;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    Netfp_SockAddr              sockAddress;
    uint32_t                    gtpuIdentifier = 0xdead12;
    uint32_t                    hoGtpuIdentifier = 0xdead90;

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
    * HO Channel: This is the channel where the HandOver packets will be received.
    ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                       = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                = NULL;
    chConfig.msgcomInstHandle           = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode   = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    hoChannel = Msgcom_create ("HO_Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (hoChannel == 0)
    {
        printf ("Error: Unable to open the handOver channel Error : %d\n", errCode);
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
     * NETFP Downlink Handover Socket
     *  - To differentiate packets arriving from the SeGW and seNodeB we use different
     *    GTPU identifiers in the test case.
     ************************************************************************************/

    /* Create a socket to receive data from Source eNB during HandOver */
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
	sockAddress.sin_gtpuId              = hoGtpuIdentifier;
    sockAddress.op.bind.inboundFPHandle = fpIngressV4Handle;
    sockAddress.op.bind.flowId          = rxFlowId;
	sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(hoChannel);

    /* Bind the socket. */
	if (Netfp_bind (lteHoSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: Failed to bind the LTE HO channel [Error code %d]\n", errCode);
		return NULL;
    }

    /* Debug Message: */
    printf ("---------------------------------------------------------------------------------------------\n");
    printf ("Please use a traffic generation application to send packets to the Fast Path\n");
    printf ("\n");
    printf ("HO GTPU Identifier is 0x%x\n", hoGtpuIdentifier);
    printf ("GTPU    Identifier is 0x%x\n", gtpuIdentifier);
    printf ("---------------------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        /* Initiate HO after 100 packets */
        if ((gNumPktDecodedPktRxed == 100) && (gInitiateFPTargetHO == TEST_TARGET_HO_NOT_COMPLETE))
        {
            gInitiateFPTargetHO = TEST_TARGET_HO_COMPLETE_INITIATION;
            printf ("Initiating Target HAND OVER....\n");
        }

        /* Is HO activated? */
        if (gInitiateFPTargetHO == TEST_TARGET_HO_COMPLETE_INITIATION)
        {
            /* Process the Handover request. */
            if (Test_processTargetHandover(ueHandle, lteHoSockHandle) < 0)
            {
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                return NULL;
            }

            /* Once this is complete all data traffic arriving from the source eNodeB
             * should be completely stopped. */
            gInitiateFPTargetHO = TEST_TARGET_HO_COMPLETE;
            printf ("Target HAND OVER Complete....\n");
        }

        /* Check if there is a message on the ENCODE channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Packet was ciphered, decipher the packet */
            Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
        }

        /* NO message on the encode channel: Check if there is a message on the DECODE channel */
        Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Send the packet out */
            Test_sendPacket(lteDRBChannel, ptrRxPkt);
        }

        /* Process all packets on the handover channel. */
        Msgcom_getMessage (hoChannel, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Packets should be received on the handover channel only if the handover
             * is not complete and the source eNodeB is sending packets via the handover
             * GTPU Identifier. Once the handover complete is signalled; these packets
             * are dropped and are NOT processed. The packet generator should NOT send
             * packets with the HO GTPU Identifier anymore. */
            if (gInitiateFPTargetHO != TEST_TARGET_HO_NOT_COMPLETE)
            {
                Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                continue;
            }

            /* These are packets which have been received from the source eNodeB
             * and which now need to be encoded */
            Test_encodePacket(ueHandle, ptrRxPkt);
        }
        else
        {
            /* Control comes here implies that there was no message on neither channels. Relinquish time */
            usleep(10);
        }
    }

    /* Test has completed execution. Display the statistics */
    Test_displayDrbFPTargetHOStats ();

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

