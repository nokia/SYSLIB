/*
 *   @file  stress_ho.c
 *
 *   @brief
 *      The file implements the following functionality for the LTE fast
 *      path channels:
 *          - Stress tests to Hand Over Users
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

/**********************************************************************
 ************************ Unit Test Definitions ***********************
 **********************************************************************/
#define TEST_SOURCE_UEID                    10
#define TEST_SOURCE_GTPUID                  0xdead0000
#define TEST_TARGET_UEID                    20
#define TEST_TARGET_GTPUID                  0xdead1000
#define TEST_RADIO_BEARER_ID                3
#define TEST_HANDOVER_GTPUID                0xdeadbeef
#define TEST_UDP_CONTROL_PORT               1024

#define TEST_HANDOVER_NOT_INITIATED         0
#define TEST_HANDOVER_INITIATED             1
#define TEST_HANDOVER_COMPLETE              2


/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/
/* Global Variable which is set to initiate the handover */
uint32_t        gInitiateSourceHandover = TEST_HANDOVER_NOT_INITIATED;
uint32_t        gInitiateTargetHandover = TEST_HANDOVER_NOT_INITIATED;

Netfp_InboundFPHandle   gFpIngressHandle;
Netfp_OutboundFPHandle  gFpEgressHandle;
int32_t                 gRxFlowId;


/* Control socket used to communicate with the serving gateway */
Netfp_SockHandle        controlSockHandle;

/* Global Event Object which keeps track of Encoded and Decoded events. */
static Event_Handle            encodeDecodeEventObject;

/* Msgcom channels used by Source eNB */
MsgCom_ChHandle         sourceEncodeChannel;
MsgCom_ChHandle         sourceDecodeChannel;
MsgCom_ChHandle         sourceSlowPathChannel;

/* 3GPP LTE User and channel at Source eNB */
Netfp_UserHandle        sourceUeHandle;
Netfp_SockHandle        sourceChHandle;
Netfp_SockHandle        sourcelteHoChHandle;

/* Msgcom channels used by Target eNB */
MsgCom_ChHandle         targetEncodeChannel;
MsgCom_ChHandle         targetDecodeChannel;
MsgCom_ChHandle         targetRohcChannel;
MsgCom_ChHandle         targetHoDLChannel;

/* 3GPP LTE User and channel at Target eNB */
Netfp_UserHandle        targetUeHandle;
Netfp_SockHandle        targetChHandle;
Netfp_SockHandle        targetHoDLChHandle;

/* Queue to receieve control packets */
Qmss_QueueHnd           controlQueueHandle;

/* Global counters */
uint32_t                gNumHOInitiated         = 0;
uint32_t                gSourceEncodedPktRxed   = 0;
uint32_t                gSourceDecodedPktRxed   = 0;
uint32_t                gSourcePktTxed          = 0;
uint32_t                gSourceSlowPathPktRxed  = 0;
uint32_t                gTargetEncodedPktRxed   = 0;
uint32_t                gTargetDecodedPktRxed   = 0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/
/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Global variable for the application MSGCOM instance handle. */
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* NETFP Configuration after parsing the DAT file. */
extern Test_NetfpConfigInfo     netfpConfig;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* Statistics API */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

/* Function to send out a packet. */
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
 *      Callback function registered when data is received on the slow path
 *      channel
 *
 *  @param[in]  chHandle
 *      Channel Handle.
 *  @param[in]  arg
 *      Optional application specific argument.
 *  @retval
 *      Not Applicable
 */
static void Test_slowPathChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
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
 *      Optional application specific argument.
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
 *      Optional application specific argument.
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
 *      This is an exported function which is used to display the statistics related
 *      to the HandOver Tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_displayHandOverStats(void)
{
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("LTE HandOver Test ::::\n");

    /* Display the module statistics. */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Number of HandOvers initiated               : %d\n", gNumHOInitiated);
    System_printf ("Number of Encoded Packets at Source eNB     : %d\n", gSourceEncodedPktRxed);
    System_printf ("Number of Decoded Packets at Source eNB     : %d\n", gSourceDecodedPktRxed);
    System_printf ("Number of SlowPath Packets at Source eNB    : %d\n", gSourceSlowPathPktRxed);
    System_printf ("Number of Encoded Packets at Target eNB     : %d\n", gTargetEncodedPktRxed);
    System_printf ("Number of Decoded Packets at Target eNB     : %d\n", gTargetDecodedPktRxed);
    System_printf ("Number of Packets sent to serving gateway   : %d\n", gSourcePktTxed);
    System_printf ("-------------------------------------------------------------\n");
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

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Get the countC associated with the packet. */
    countC = Netfp_ntohl(*(uint32_t *)ptrRxDataBuffer);

    /* Now we need to skip the countC */
    Pktlib_setPacketLen(ptrRxPkt, rxDataBufferLen - 4);
    Pktlib_setDataOffset (ptrRxPkt, 4);

    /* We use the cipher function to decode the packet.
     *  - Special Deciphering done on the packet so the direction is 1 */
    retVal = Netfp_cipher (ueHandle, TEST_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, countC,
                           Msgcom_getInternalMsgQueueInfo(decodeChannel));
    /* Is there an error while encoding the data? */
    if (retVal < 0)
        System_printf ("Error: Decoding data failed\n");

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send out the packet.
 *
 *  @param[in]  lteChannel
 *      LTE 3GPP Channel Handle on which the packet is to be sent
 *  @param[in]  ptrPkt
 *      Pointer to the packet to be txed
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_sendPacket (Netfp_SockHandle lteChannel, Ti_Pkt* ptrPkt)
{
    uint8_t*    ptrRxDataBuffer;
    uint32_t    rxDataBufferLen;
    int32_t     retVal, errCode;

    /* Increment the statistics. */
    gSourcePktTxed++;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* In the fast path test: We have received the actual payload itself so
     * all we need to do here is simply ship it back */
    retVal = Netfp_send (lteChannel, ptrPkt, 0, &errCode);

    if (retVal < 0)
    {
        System_printf ("Error: Sending data failed [Error code %d]\n", errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called to process a packet which was received on the slow
 *      path channel at Source eNB once the handover is initiated. The packet will be
 *      sent to the Target eNB using the Target HandOver GTPU Id.
 *
 *  @param[in]  channelHandle
 *      3GPP channel Handle on which the packet was received
 *  @param[in]  ptrRxPkt
 *      Pointer to the received packet.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_sendPacketToTargeteNB (Netfp_SockHandle channelHandle, Ti_Pkt* ptrRxPkt)
{
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    uint8_t*                ptrRxDataBuffer;
    uint32_t                rxDataBufferLen;
    int32_t                 retVal;

    /* Increment the statistics. */
    gSourceSlowPathPktRxed++;

    /* Get the data buffer & length */
    Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Send the packet out. */
    retVal = Test_netSendIPv4Payload(ptrRxDataBuffer,   /* Payload Data      */
                rxDataBufferLen,                        /* Payload Length    */
                TEST_HANDOVER_GTPUID,                   /* GTPU Identifier   */
                0xFF,                                   /* GTPU Message Type */
                2152,                                   /* Destination Port  */
                2152,                                   /* Source Port       */
                &ptrNetfpConfig->eNodeBIPAddress[0],    /* Destination IP    */
                &ptrNetfpConfig->eNodeBIPAddress[0],    /* Source IP         */
                0,                                      /* TOS               */
                &ptrNetfpConfig->eNodeBMACAddress[0],   /* Destination MAC   */
                &ptrNetfpConfig->eNodeBMACAddress[0],   /* Source MAC        */
                1);                                     /* Loopback          */
    if (retVal < 0)
    {
        System_printf ("Error: Failed to send packet to Gateway\n");
        return -1;
    }

    /* TODO Free the packet? */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the source handover request. The function
 *      will disable the user and move all the channels to the slow path. Once
 *      handover is initiated; there should be NO packet coming to the fast path
 *      encoded channel.
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_processSourceHandover()
{
    int32_t     retVal, errCode;
    Ti_Pkt*     ptrPkt;
    uint8_t*    ptrDataBuffer;
    uint32_t    dataBufferLen;

    /* Initiate source HandOver */
    retVal = Netfp_initiateSourceHandOver (sourceUeHandle, TEST_RADIO_BEARER_ID, gRxFlowId,
                                         Msgcom_getInternalMsgQueueInfo(sourceSlowPathChannel),
                                         &errCode);

    /* Were we able to initiate source HandOver? */
    if (retVal < 0)
    {
        /* Error: initiate source HandOver failed. */
        System_printf ("Error: Initiate source HandOver failed\n");
        return -1;
    }

    /* Send a control message to the Serving Gateway to switch the eNB to target eNB */

    /* Allocate a dummy packet for the decode operation: */
    ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, mtuReceiveHeap, 128);
    if (ptrPkt != NULL)
    {
        /* Get the data buffer: */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferLen);

        /* Initialize the dummy data: */
        memset ((void *)ptrDataBuffer, 0x11, dataBufferLen);
        appWritebackBuffer (ptrDataBuffer, dataBufferLen);

        retVal = Netfp_send (controlSockHandle, ptrPkt, 0, &errCode);
        if (retVal < 0)
        {
            System_printf ("Error: Failed to send packet to serving gateway [Error code %d]\n", errCode);
            return -1;
        }
    }

    /* TODO Process the packets that have already been encoded. */
#if 0
    while (1)
    {
        /* Process all packets on the encode channel. */
        Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Get the data buffer & length */
        Pktlib_getDataBuffer (ptrRxPkt, &ptrDataBuffer, &dataBufferLen);

        /* Invalidate the data buffer. */
        appInvalidateBuffer(ptrDataBuffer, dataBufferLen);

        /* Get the countC associated with the packet. */
        countC = ntohl(*(uint32_t *)ptrDataBuffer);

        /* Now we need to skip the countC */
        Pktlib_setPacketLen(ptrRxPkt, dataBufferLen - 4);
        Pktlib_setDataOffset (ptrRxPkt, 4);

        /* Decode the packet and send iut to the Target UE */
        retVal = Netfp_cipher (sourceUeHandle, TEST_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, gNumPktEncoded,
                            Msgcom_getInternalMsgQueueInfo(decodeChannel));
    }
#endif
    /* Source Handover processed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the target handover request.
 *      It will first process the packets received over the X2 link.
 *      It will then call the Netfp library witht he correct countC to
 *      process the packets received over the S1 link and switch the
 *      channel to fast path.
 *
 *  @param[in]  ueHandle
 *      User Context Handle for which the handover is being initiated
 *      from the SeGW are placed.
 *  @param[in]  hoChannel
 *      This is the handover channel which was being used.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error    -  <0
 */
static int32_t Test_processTargetHO
(
    Netfp_UserHandle        ueHandle,
    MsgCom_ChHandle         hoChannel
)
{
    int32_t                 errCode, retVal;
    uint32_t                targetHandOverId;
    Ti_Pkt*                 ptrRxPkt;

    /* Process the packets that came over the X2 link. */
    while (1)
    {
        /* Process all packets on the slow path channel. */
        Msgcom_getMessage (targetHoDLChannel, (MsgCom_Buffer**)&ptrRxPkt);

        /* Did we process all the messages */
        if (ptrRxPkt == NULL)
            break;

        /* Reset the PS Flags in the packet: */
        Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)ptrRxPkt, 0x0);

        /* These are packets which have been received from the source eNodeB
         * and which now need to be encoded */
        retVal = Netfp_encodeDrb (ueHandle, ptrRxPkt, gSourcePktTxed++, TEST_RADIO_BEARER_ID);
    }

    /* Processing of X2 packets is done. Switch the channel to fast path. */
    retVal = Netfp_completeTargetHandOver (ueHandle, TEST_RADIO_BEARER_ID, gSourcePktTxed, &targetHandOverId, &errCode);

    if (retVal < 0)
    {
        printf ("Error: Initiating the Target HO failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Process all the packets buffered during HO */
    while (1)
    {
        retVal = Netfp_getTargetHandOverPackets (targetHandOverId, &ptrRxPkt, &errCode);
        if (retVal < 0)
        {
            /* Error: Getting the buffered packets. */
            printf ("Error: Getting target HandOver buffered packets failed [Error code %d]\n", errCode);
            return -1;
        }
        if (ptrRxPkt != NULL)
        {
            printf ("Error: Packets were buffered. This should not have happened in Fast Path\n");
            return -1;
        }
        break;
    }

    printf ("Debug: Target Handover Completed\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the user.
 *
 *  @param[in]  ueId
 *      UE Identifier.
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  @retval
 *      Success -   Handle to the user
 *  @retval
 *      Error   -   NULL
 */
static Netfp_UserHandle Test_createUser
(
    uint32_t                ueId,
    int32_t*                errCode
)
{
    Netfp_UserCfg           userCfg;
    uint8_t                 encryptionKey[16];
    uint8_t                 integrityKey[16];
    uint32_t                index;

    /* Initialize the user security configuration. */
    memset ((void *)&userCfg, 0, sizeof(Netfp_UserCfg));

    /* Initialize the integrity and ciphering keys */
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (TSCH & 0xFF);
        integrityKey[index]  = (TSCL & 0xFF);
    }

    /* Populate the user security configuration. */
    userCfg.authMode         = Netfp_3gppAuthMode_EIA2;
    userCfg.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userCfg.ueId             = ueId;
    userCfg.srbFlowId        = gRxFlowId;
    userCfg.initialCountC    = 0;
    userCfg.chSrb1Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb1Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userCfg.chSrb2Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userCfg.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userCfg.hKeyRrcInt));
    memcpy ((void *)&userCfg.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userCfg.hKeyRrcEnc));
    memcpy ((void *)&userCfg.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userCfg.hKeyUpEnc));

    /* Create the user */
    return (Netfp_createUser (netfpClientHandle, &userCfg, errCode));
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the 3GPP LTE channel.
 *  @param[in]  ueHandle
 *      NETFP user on which the channel is being added
 *  @param[in]  slowPathQueueHandle
 *      Queue where all the slow path packets will be present
 *  @param[in]  encodeQueueHandle
 *      Queue where all the encoded packets will be present
 *  @param[in]  decodeQueueHandle
 *      Queue where all the decoded packets will be present
 *  @param[in]  gtpuId
 *      UE Identifier
 *  @param[in]  rbId
 *      GTPU Identifier
 *  @param[in]  enableFastPath
 *      Indicates if fast path is enabled.
 *  @param[in]  isHoInProgress
 *      On target eNB, this field indicates HandOver is in progress for this radio bearer.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  @retval
 *      Success     -   3GPP Channel Handle
 *  @retval
 *      Error       -   NULL
 */
static Netfp_SockHandle Test_createLTEChannel
(
    Netfp_UserHandle        ueHandle,
    Qmss_QueueHnd           slowPathQueueHandle,
    Qmss_QueueHnd           encodeQueueHandle,
    Qmss_QueueHnd           decodeQueueHandle,
    uint32_t                gtpuId,
    uint32_t                rbId,
    uint32_t                enableFastPath,
    uint32_t                isHoInProgress,
    int32_t*                errCode
)
{
    Netfp_LTEChannelBindCfg     lteChannelBindCfg;
    Netfp_LTEChannelConnectCfg  lteChannelConnectCfg;

    /* Populate the channel bind configuration: */
    lteChannelBindCfg.flowId          = gRxFlowId;
    lteChannelBindCfg.notifyFunction  = NULL;
    lteChannelBindCfg.chDrbRohc       = slowPathQueueHandle;
    lteChannelBindCfg.fpHandle        = gFpIngressHandle;
    lteChannelBindCfg.sin_gtpuId      = gtpuId;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = enableFastPath;
    lteChannelBindCfg.isHOInProgress  = isHoInProgress;
    lteChannelBindCfg.chDrbEnc        = encodeQueueHandle;

    /* Populate the channel connect configuration: */
    lteChannelConnectCfg.fpHandle       = gFpEgressHandle;
    lteChannelConnectCfg.sin_gtpuId     = gtpuId;
    lteChannelConnectCfg.qci            = 3;
    lteChannelConnectCfg.dscp           = 0x22;
    lteChannelConnectCfg.flowId         = gRxFlowId;
    lteChannelConnectCfg.chDrbDec       = decodeQueueHandle;

    /* Create the DRB LTE channel: */
    return (Netfp_createLTEChannel (ueHandle, rbId, Netfp_SockFamily_AF_INET,
                                            &lteChannelBindCfg, &lteChannelConnectCfg, errCode));
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the data received at the source eNB
 *
 *  @retval
 *      None
 */
static void sourceEncodeDecodeData_task(UArg arg0, UArg arg1)
{
    Ti_Pkt*                 ptrRxPkt;
    uint32_t                events;

    /* Loop around performing the operations */
    while (1)
    {
        /* Wait for an event to occur in the system. */
        events = Event_pend(encodeDecodeEventObject, Event_Id_NONE,
                            Event_Id_00 + Event_Id_01 + Event_Id_02,
                            BIOS_WAIT_FOREVER);

        /* Check if HandOver procedure has been initiated? */
        if (gInitiateSourceHandover == TEST_HANDOVER_INITIATED)
        {
            /* Process the Handover request. */
            if (Test_processSourceHandover() < 0)
            {
                System_printf ("Error: Source Hand Over failed for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_SOURCE_UEID, TEST_RADIO_BEARER_ID, TEST_SOURCE_GTPUID);
                return;
            }

            /* Once handover has been initiated; there is NO going back. This is a one time test
             * and here we set the flag to indicate that the handover is complete. */
            gInitiateSourceHandover = TEST_HANDOVER_COMPLETE;
            gNumHOInitiated++;
        }

        /* Check if we received a packet over the handover channel. */
        if (events & Event_Id_00)
        {
            /* YES. Cycle through and process all the received packets and push them for decoding. */
            while (1)
            {
                /* Process all packets on the slow path channel. */
                Msgcom_getMessage (sourceSlowPathChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* These packets should be sent to the Target eNodeB */
                if (Test_sendPacketToTargeteNB (sourceChHandle, ptrRxPkt) < 0)
                {
                    System_printf ("Error: Unable to forward packet from source eNB to target eNB\n");
                    return;
                }
            }
        }

        /* Check if we received an encrypted packet? */
        if (events & Event_Id_01)
        {
            /* YES. Cycle through and process all the received packets and push them out */
            while (1)
            {
                /* Process all packets on the encode channel. */
                Msgcom_getMessage (sourceEncodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                gSourceEncodedPktRxed++;

                /* Decode the packet & push the packet to the decoded channel. */
                Test_decodePacket(sourceUeHandle, ptrRxPkt, sourceDecodeChannel);
            }
        }

        /* Check if we received a decrypted packet? */
        if (events & Event_Id_02)
        {
            /* YES. Cycle through and process all the received packets and push them out */
            while (1)
            {
                /* Process all packets on the decode channel. */
                Msgcom_getMessage (sourceDecodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                gSourceDecodedPktRxed++;

                /* Check if the packets have to be sent to Target eNB or back to Serving gateway */
                if (gInitiateSourceHandover == TEST_HANDOVER_COMPLETE)
                {
                    /* These packets should be sent to the Target eNodeB */
                    if (Test_sendPacketToTargeteNB (sourceChHandle, ptrRxPkt) < 0)
                    {
                        System_printf ("Error: Unable to forward slow path packet from source eNB to target eNB\n");
                        return;
                    }
                }
                else
                {
                    /* Send the packet to the Serving gateway */
                    Test_sendPacket(sourceChHandle, ptrRxPkt);
                }
            }
        }

        /* Relinquish time slice */
        Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *  This tasks sets up the source eNB
 *
 *  @retval
 *      Not Applicable.
 */
static void sourceHandOver_task(UArg arg0, UArg arg1)
{
    Task_Params             taskParams;
    int32_t                 errCode;
    Msgcom_ChannelCfg       chConfig;

    /************************************************************************************
     * Slow Path Channel: Packets arriving from the SeGW are stored here until the
     * handover is complete.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_slowPathChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;


    /* Create the Message communicator channel. */
    sourceSlowPathChannel = Msgcom_create ("Source Slow Path Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (sourceSlowPathChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Encode Channel: Encoded Packets will be received on this channel.
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


    /* Create the Message communicator channel. */
    sourceEncodeChannel = Msgcom_create ("Source Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (sourceEncodeChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Decode Channel: Packets once decoded are placed into this channel.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_decodeChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[2].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[2].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[2].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[2].hostInterrupt;


    /* Create the Message communicator channel. */
    sourceDecodeChannel = Msgcom_create ("Source Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (sourceDecodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }

    /* Create a user */
    sourceUeHandle = Test_createUser (TEST_SOURCE_UEID, &errCode);

    if (sourceUeHandle == NULL)
    {
        System_printf ("Error: LTE Creation User Security Context failed for ueId: %d, [Error code %d]\n", TEST_SOURCE_UEID, errCode);
        return;
    }

    /* Create the 3GPP LTE channel. */
    sourceChHandle = Test_createLTEChannel(sourceUeHandle,
                    NULL,
                    Msgcom_getInternalMsgQueueInfo(sourceEncodeChannel),
                    Msgcom_getInternalMsgQueueInfo(sourceDecodeChannel),
                    TEST_SOURCE_GTPUID, TEST_RADIO_BEARER_ID, 1, 0, &errCode);

    if (sourceChHandle == NULL)
    {
        System_printf ("Error: LTE Create Channel failed for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_SOURCE_UEID, TEST_RADIO_BEARER_ID, TEST_SOURCE_GTPUID);
        return;
    }

    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Source eNB - LTE Create Channel created for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_SOURCE_UEID, TEST_RADIO_BEARER_ID, TEST_SOURCE_GTPUID);
    System_printf ("------------------------------------------------------------------------------\n");

    /* Create a control socket to send HO command to the test equipment */
    /* Create a task to send and receive DRB data */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 6;
    Task_create(sourceEncodeDecodeData_task, &taskParams, NULL);

    return;
}


/**
 *  @b Description
 *  @n
 *      The function is used to process the data received at the target eNB
 *
 *  @retval
 *      None
 */
static void targetEncodeDecodeData_task(UArg arg0, UArg arg1)
{
    Ti_Pkt*                 ptrRxPkt;
    Ti_Pkt*                 ptrPkt;

    /* Loop around performing the operations */
    while (1)
    {
        /* Check if HandOver is initiated? */
        ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(controlQueueHandle));
        if (ptrPkt != NULL)
        {
            /* Target HandOver has been initiated. */
            gInitiateTargetHandover = TEST_HANDOVER_INITIATED;

            /* Clean up the received packet. */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
        }

        if (gInitiateTargetHandover == TEST_HANDOVER_INITIATED)
        {
            /* Process the Target HO. If there is an error we kill the test case. */
            if (Test_processTargetHO(targetUeHandle, targetHoDLChHandle) < 0)
                return;

            /* Once this is complete all data traffic arriving from the source eNodeB
             * should be completely stopped. */
            gInitiateTargetHandover = TEST_HANDOVER_COMPLETE;
        }

        /* Check if we received an encrypted packet? */
        while (1)
        {
            /* Process all packets on the encode channel. */
            Msgcom_getMessage (targetEncodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

            /* Did we process all the messages */
            if (ptrRxPkt == NULL)
                break;

            gTargetEncodedPktRxed++;

            /* Decode the packet & push the packet to the decoded channel. */
            Test_decodePacket(targetUeHandle, ptrRxPkt, targetDecodeChannel);
        }

        /* Check if we received a decrypted packet? */
        while (1)
        {
            /* Process all packets on the decode channel. */
            Msgcom_getMessage (targetDecodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

            /* Did we process all the messages */
            if (ptrRxPkt == NULL)
                break;

            gTargetDecodedPktRxed++;

            /* Send the packet out */
            Test_sendPacket(targetChHandle, ptrRxPkt);
        }

        /* Relinquish time slice */
        Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *  This tasks sets up the target eNB
 *
 *  @retval
 *      Not Applicable.
 */
static void targetHandOver_task(UArg arg0, UArg arg1)
{
    Task_Params             taskParams;
    int32_t                 errCode;
    Msgcom_ChannelCfg       chConfig;
    Netfp_SockAddr          sockAddress;

    /************************************************************************************
     * Slow Path Channel: Packets arriving from the SeGW are stored here until the
     * handover is complete.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode    = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    targetRohcChannel = Msgcom_create ("Target RoHC Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (targetRohcChannel == 0)
    {
        System_printf ("Error: Unable to open the Target RoHC channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Encode Channel: Encoded Packets will be received on this channel.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode    = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    targetEncodeChannel = Msgcom_create ("Target Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (targetEncodeChannel == 0)
    {
        System_printf ("Error: Unable to open the encode channel Error : %d\n", errCode);
        return;
    }

    /************************************************************************************
     * Decode Channel: Packets once decoded are placed into this channel.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode    = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    targetDecodeChannel = Msgcom_create ("Target Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (targetDecodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }

    /* Create a user */
    targetUeHandle = Test_createUser (TEST_TARGET_UEID, &errCode);

    if (targetUeHandle == NULL)
    {
        System_printf ("Error: LTE Creation User Security Context failed for ueId: %d, [Error code %d]\n", TEST_TARGET_UEID, errCode);
        return;
    }

    /* Create the 3GPP LTE channel. */
    targetChHandle = Test_createLTEChannel(targetUeHandle,
                    Msgcom_getInternalMsgQueueInfo(targetRohcChannel),
                    Msgcom_getInternalMsgQueueInfo(targetEncodeChannel),
                    Msgcom_getInternalMsgQueueInfo(targetDecodeChannel),
                    TEST_TARGET_GTPUID, TEST_RADIO_BEARER_ID, 1, 1, &errCode);

    if (targetChHandle == NULL)
    {
        System_printf ("Error: LTE Create Channel failed for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_TARGET_UEID, TEST_RADIO_BEARER_ID, TEST_TARGET_GTPUID);
        return;
    }

    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Target eNB - LTE Create Channel created for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_TARGET_UEID, TEST_RADIO_BEARER_ID, TEST_TARGET_GTPUID);
    System_printf ("------------------------------------------------------------------------------\n");

     /************************************************************************************
     * NETFP Downlink Handover Channel
     *  - To differentiate packets arriving from the SeGW and seNodeB we use different
     *    GTPU identifiers in the test case.
     ************************************************************************************/
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                        = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                 = NULL;
    chConfig.msgcomInstHandle            = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode    = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the Message communicator channel. */
    targetHoDLChannel = Msgcom_create ("Target HO DL Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (targetHoDLChannel == 0)
    {
        System_printf ("Error: Unable to open the Target HO DL Channel channel Error : %d\n", errCode);
        return;
    }

    /* Create a socket to receive data from Source eNB during HandOver */
	targetHoDLChHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (targetHoDLChHandle == NULL)
	{
        System_printf ("Error: Failed to create the LTE HO socket to receive data from Source eNB [Error code %d]\n", errCode);
	    return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = 2152;
	sockAddress.sin_gtpuId              = TEST_HANDOVER_GTPUID;
    sockAddress.op.bind.inboundFPHandle = gFpIngressHandle;
    sockAddress.op.bind.flowId          = gRxFlowId;
	sockAddress.op.bind.queueHandle     = Msgcom_getInternalMsgQueueInfo(targetHoDLChannel);

    /* Bind the socket. */
	if (Netfp_bind (targetHoDLChHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: Failed to bind the LTE HO channel [Error code %d]\n", errCode);
		return;
    }

    System_printf ("------------------------------------------------------------------------------\n");
    System_printf ("Target eNB - LTE DL HO Channel created for UEId: %d RBId: %d GTPU Id: 0x%x\n", TEST_TARGET_UEID, TEST_RADIO_BEARER_ID, TEST_HANDOVER_GTPUID);
    System_printf ("------------------------------------------------------------------------------\n");

    /* Create a control socket to send HO command to the test equipment */
    /* Create a task to send and receive SRB, DRB data */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 6;
    Task_create(targetEncodeDecodeData_task, &taskParams, NULL);
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the handOver tests.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t Test_stressHandOver(void)
{
    Task_Params             taskParams;
    uint8_t                 isAllocated;
    Netfp_SockAddr          sockAddress;
    int32_t                 errCode;
    Netfp_FlowCfg           flowCfg;

    /* Get the ingress and egress IPv4 fast paths. */
    gFpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (gFpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return -1;
    }
    gFpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (gFpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the flow configuration. */
    memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

    /* Populate the flow configuration: Use the NETFP Data Receive Heap for this purpose. */
    strcpy (flowCfg.name, "NetfpDataRx");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    gRxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (gRxFlowId < 0)
    {
        System_printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Flow %d has been created successfully\n", gRxFlowId);

    /* Create the Event object used */
    encodeDecodeEventObject = Event_create(NULL, NULL);
    if (encodeDecodeEventObject == NULL)
    {
        System_printf ("Error: Encode-Decode Event Object creation failed\n");
        return -1;
    }

    /* Create a UDP socket to communicate with the Serving Gateway */
	controlSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
	if (controlSockHandle == NULL)
	{
		printf ("Error: Unable to create socket [Error Code %d]\n", errCode);
		return -1;
	}

    controlQueueHandle = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);

	/* Populate the binding information */
	memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = TEST_UDP_CONTROL_PORT;
    sockAddress.op.bind.inboundFPHandle = gFpIngressHandle;
    sockAddress.op.bind.flowId          = gRxFlowId;
	sockAddress.op.bind.queueHandle     = controlQueueHandle;

	/* Bind the socket. */
	if (Netfp_bind (controlSockHandle, &sockAddress, &errCode) < 0)
	{
	    System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
		return -1;
	}

	/* Populate the connect information. */
	sockAddress.sin_family  = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port    = TEST_UDP_CONTROL_PORT;
	sockAddress.op.connect.outboundFPHandle = gFpEgressHandle;

	/* Connect the socket */
	if (Netfp_connect(controlSockHandle, &sockAddress, &errCode) < 0)
	{
	    System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
		return -1;
	}

    /* Create a task to add and delete UEs */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    taskParams.priority  = 5;
    Task_create(sourceHandOver_task, &taskParams, NULL);

    /* Create a task to add and delete UEs */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    taskParams.priority  = 5;
    Task_create(targetHandOver_task, &taskParams, NULL);

    /* Tests have been launched: */
    return 0;
}

