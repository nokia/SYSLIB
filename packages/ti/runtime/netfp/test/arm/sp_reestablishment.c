/*
 *   @file  sp_reestablishment.c
 *
 *   @brief
 *      The file implements the following functionality for the LTE slow
 *      path channels:
 *          - Reestablishment for slow path radio bearers
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
#define TEST_USER_RADIO_BEARER_ID           1
/* Test User User Equipment Identifier.  */
#define TEST_USER_UE_ID                     10

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* Global counter which keeps track of the number of countC for the encoding
 * of packets. */
static uint32_t        gCountC = 0;
static uint32_t        gOldCountC = 0;

/* Global Variable which is set to initiate the reestablishment  */
static uint32_t        gInitiateReestablishment   = 0;

/* Global variable which keeps track of the number of reestablishments initiated */
static uint32_t        gNumReestablishmentInitiated = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle   netfpDataRxHeap;

/* Test completion status */
extern uint32_t            testComplete;

/* NETFP Client Handle: */
extern Netfp_ClientHandle   netfpClientHandle;

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
 *      Display the statistics for the Re-establishment tests
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_displayDrbSpReestablishmentStats(void)
{
    printf ("-------------------------------------------------------------\n");
    printf ("LTE Slow Path Re-establishment Tests::::\n");
    printf ("Number of packets decoded: %d\n", gCountC);
    printf ("Number of Reestablishments Initiated: %d\n", gNumReestablishmentInitiated);

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

    gCountC = countC;
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
void* Test_spDrbReestablishmentTask(void* arg)
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
    Ti_Pkt*                     ptrRxPkt;
    Netfp_SockHandle            lteDRBChannel;
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;
    uint32_t                    gtpuIdentifier = 0xdead12;
    Netfp_UserHandle            reestablishedUeHandle;
    Netfp_SockHandle            reestablishedLteDRBChannel;
    uint16_t                    rxUeId;
    uint8_t                     rxQci;
    uint8_t                     rxRbId;
    uint8_t                     rxContextId;

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
    gtpuChannelHandle = Msgcom_create ("GTPU_Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (gtpuChannelHandle == 0)
    {
        printf ("Error: Unable to open the channel Error : %d\n", errCode);
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
    lteChannelBindCfg.chDrbRohc       = Msgcom_getInternalMsgQueueInfo(gtpuChannelHandle);
    lteChannelBindCfg.fpHandle        = fpIngressV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = 0;
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
    printf ("Please use a traffic generation application to send packets to the Slow Path GTPU Id 0x%x\n", gtpuIdentifier);
    printf ("---------------------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (testComplete == 0)
    {
        if ((gCountC != 0) && (gCountC % 10 == 0) && (gInitiateReestablishment == 0) && (gOldCountC != gCountC))
        {
            gOldCountC = gCountC;
            gInitiateReestablishment = 1;
        }
        /* Is reestablishment activated? */
        if (gInitiateReestablishment == 1)
        {
            uint32_t                currentCountC;

            printf ("Debug: Starting re-establishment.....\n");

            /* YES. Initiate reestablishment by suspending the channel. */
            retVal = Netfp_suspendLTEChannel (ueHandle, TEST_USER_RADIO_BEARER_ID, rxFlowId,  &errCode);
            if (retVal < 0)
            {
                printf ("Error: Suspend Channel Failed (Error %d)\n", errCode);
                continue;
            }

            /* Update & log the information. */
#if 0
            while (1)
            {
                /* Process all packets on the LTE 3GPP channel. */
                Msgcom_getMessage (gtpuChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);
                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Encode the packet */
                Test_encodePacket(ueHandle, ptrRxPkt);
            }

            /* Process all the received encrypted packets and push them for decoding. */
            while (1)
            {
                /* Process all packets on the encode channel. */
                Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Decode the packet */
                Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
            }

            /* Process all the decoded packets and push them out */
            while (1)
            {
                /* Process all packets on the decode channel. */
                Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Send the packet out */
                Test_sendPacket(lteDRBChannel, ptrRxPkt);
            }
#endif
            /* Create the New LTE User */

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
            userCfg.chSrb1Enc        = (Qmss_QueueHnd)0;    /* Set to NULL only because we are testing DRB's here */
            userCfg.chSrb1Dec        = (Qmss_QueueHnd)0;    /* Set to NULL only because we are testing DRB's here */
            userCfg.chSrb2Enc        = (Qmss_QueueHnd)0;    /* Set to NULL only because we are testing DRB's here */
            userCfg.chSrb2Dec        = (Qmss_QueueHnd)0;    /* Set to NULL only because we are testing DRB's here */
            memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
            memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
            memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

            /* Create a new user */
            reestablishedUeHandle = Netfp_createUser (netfpClientHandle, &userCfg, &errCode);
            if (reestablishedUeHandle == NULL)
            {
                /* Adding a new user failed. */
                printf ("Error: LTE User Creation failed [Error code %d]\n", errCode);
                return NULL;
            }
            printf ("Debug: New LTE User %d has been created\n", userCfg.ueId);

            /* CountC is maintained by application */
            currentCountC = gCountC;

            /* The LTE channel was suspended; we need to reconfigure the channel */
            reestablishedLteDRBChannel = Netfp_reconfigureLTEChannel(reestablishedUeHandle, ueHandle,
                                            TEST_USER_RADIO_BEARER_ID, currentCountC, &errCode);
            if (reestablishedLteDRBChannel == NULL)
            {
                printf ("Error: Reconfigure LTE channel Failed (Error %d)\n", errCode);
                return NULL;
            }
            printf ("Debug: New LTE channel %x has been created\n", (uint32_t)reestablishedLteDRBChannel);

            retVal = Netfp_resumeLTEChannel(reestablishedUeHandle, ueHandle, TEST_USER_RADIO_BEARER_ID, &errCode);
            if (retVal < 0)
            {
                printf ("Error: Resume LTE channel Failed (Error %d)\n", errCode);
                return NULL;
            }
            printf ("Debug: New LTE channel %x has been resumed\n", (uint32_t)reestablishedLteDRBChannel);
            retVal = Netfp_deleteUser (ueHandle, &errCode);
            if (retVal < 0)
            {
                printf ("Error: Delete old User Failed (Error %d)\n", errCode);
                return NULL;
            }

            printf ("Debug: Old LTE user has been deleted\n");

            /* From this point on use; the new user handle */
            ueHandle = reestablishedUeHandle;

            /* From this point on use; the new channel handle */
            lteDRBChannel = reestablishedLteDRBChannel;

            /* Keep track of the number of reestablishment attempts */
            gNumReestablishmentInitiated++;

            /* Reestablishment is complete */
            gInitiateReestablishment = 0;
        }

        /* Process all packets on the LTE 3GPP channel. */
        Msgcom_getMessage (gtpuChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);
        if (ptrRxPkt != NULL)
        {
            /* Get the UE Id, RB Id and context Id */
            Netfp_getPacketId (ptrRxPkt, &rxUeId, &rxQci, &rxRbId, &rxContextId);
            if ((rxUeId != TEST_USER_UE_ID) || (rxRbId != TEST_USER_RADIO_BEARER_ID))
            {
                printf ("Error: UEId RbId mismatch Expected %d : %d Received %d : %d\n",
                                        TEST_USER_UE_ID, TEST_USER_RADIO_BEARER_ID, rxUeId, rxRbId);
            }

            /* Encode the packet */
            Test_encodePacket(ueHandle, ptrRxPkt);

            /* Check if there is a message on the ENCODE channel. */
            while (1)
            {
                Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
                if (ptrRxPkt != NULL)
                    break;
            }
            /* Process the packet */
            {
                /* Get the UE Id, RB Id and context Id */
                Netfp_getPacketId (ptrRxPkt, &rxUeId, &rxQci, &rxRbId, &rxContextId);
                if ((rxUeId != TEST_USER_UE_ID) || (rxRbId != TEST_USER_RADIO_BEARER_ID))
                {
                    printf ("Error: UEId RbId mismatch Expected %d : %d Received %d : %d\n",
                                        TEST_USER_UE_ID, TEST_USER_RADIO_BEARER_ID, rxUeId, rxRbId);
                }

                /* Decode the packet */
                Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
            }
            /* Check if there is a message on the DECODE channel */
            while (1)
            {
                Msgcom_getMessage (decodeChannel, (MsgCom_Buffer**)&ptrRxPkt);
                if (ptrRxPkt != NULL)
                    break;
            }
            /* Send the packet out */
            Test_sendPacket(lteDRBChannel, ptrRxPkt);
        }
    }

    /* Test has completed execution. Display the statistics */
    Test_displayDrbSpReestablishmentStats ();

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
    errCode = Msgcom_delete (gtpuChannelHandle, myFreeMsgBuffer);
    if (errCode < 0)
    {
	    printf ("Error: MSGCOM GTPU channel close failed [Error Code %d]\n", errCode);
		return NULL;
    }
    printf ("Debug: MSGCOM GTPU channel deleted successfully\n");
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

