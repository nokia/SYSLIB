/*
 *   @file  sp_targetHandOver.c
 *
 *   @brief
 *      The file implements the following functionality for the LTE channels:
 *          - Target HandOver for slow path radio bearers
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

/* Global Event Object which keeps track of Encoded and Decoded events. */
static Event_Handle     encodeDecodeEventObject;

/* Global Variable which is set to initiate the handover */
uint32_t                gInitiateSPTargetHO = TEST_TARGET_HO_NOT_COMPLETE;

/* Global Counter for number of LTE IPv4 GTPU Packets transmitted */
static uint32_t         gNumPktTxed = 0;

/* Global Counter which keeps track of the number of packets which have been decoded */
static uint32_t         gNumPktDecodedPktRxed = 0;

/* Global Variables which keep track of the NETFP functions being benchmarked */
BENCHMARK_INIT(Netfp_encodeDrb)
BENCHMARK_INIT(Netfp_cipher)
BENCHMARK_INIT(Netfp_send)
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

/* Statistics Display Functionality. */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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
 *      Callback function registered when data is received on the handover
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
static void Test_handOverChannelCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post(encodeDecodeEventObject, Event_Id_01);
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
    Event_post(encodeDecodeEventObject, Event_Id_02);
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
    Event_post(encodeDecodeEventObject, Event_Id_03);
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

    /* Invalidate the data buffer. */
    appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

    /* Encode the data received: */
    BENCHMARK_START(Netfp_encodeDrb);
    retVal = Netfp_encodeDrb (ueHandle, ptrRxPkt, gCountC, TEST_USER_RADIO_BEARER_ID);
    BENCHMARK_END(Netfp_encodeDrb);

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_encodeDrb);

    /* Is there an error while encoding the data? */
    if (retVal < 0)
    {
        System_printf ("Error: Encoding data failed [Error code %d]\n", retVal);
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

    /* Invalidate the data buffer: We only need to invalidate 1 cache line because we are only interested
     * in reading the countC which is the first 4 bytes. */
    appInvalidateBuffer(ptrRxDataBuffer, CACHE_L2_LINESIZE);

    /* Get the countC associated with the packet. */
    countC = Netfp_ntohl(*(uint32_t *)ptrRxDataBuffer);

    /* Now we need to skip the countC */
    Pktlib_setPacketLen(ptrRxPkt, Pktlib_getPacketLen(ptrRxPkt) - 4);
    Pktlib_setDataOffset (ptrRxPkt, 4);

    /* We use the cipher function to decode the packet. */
    BENCHMARK_START(Netfp_cipher);
    retVal = Netfp_cipher (ueHandle, TEST_USER_RADIO_BEARER_ID, Netfp_3gppOperation_Decipher, 1, ptrRxPkt, countC,
                           Msgcom_getInternalMsgQueueInfo(decodeChannel));
    BENCHMARK_END(Netfp_cipher);

    /* Is there an error while encoding the data? */
    if (retVal < 0)
    {
        System_printf ("Error: Decoding data failed [Error code %d]\n", retVal);
        return;
    }
    else
        gNumPktDecodedPktRxed++;

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_cipher);
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
    BENCHMARK_START(Netfp_send);
    retVal = Netfp_send (lteChannel, ptrRxPkt, 0x0, &errCode);
    BENCHMARK_END(Netfp_send);
    if (retVal < 0)
    {
        System_printf ("Error: Sending data failed [Error code %d]\n", errCode);
        return;
    }

    /* Increment the statistics. */
    gNumPktTxed++;

    /* Update & log the information. */
    BENCHMARK_UPDATE(Netfp_send);
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
static int32_t Test_processTargetHandOver
(
    Netfp_UserHandle        ueHandle,
    Netfp_SockHandle        lteHoSockHandle
)
{
    int32_t                 retVal, errCode;
    uint32_t                targetHandOverId, bufferedPkts=0;
    Ti_Pkt*                 ptrRxPkt;

    /* Initiate the target HandOver */
    BENCHMARK_START(Netfp_completeTargetHandOver);
    retVal = Netfp_completeTargetHandOver (ueHandle, TEST_USER_RADIO_BEARER_ID, gCountC, &targetHandOverId, &errCode);
    BENCHMARK_END(Netfp_completeTargetHandOver);

    /* Were we able to process target HandOver? */
    if (retVal < 0)
    {
        /* Error: processing target HandOver. */
        System_printf ("Error: processing target HandOver failed\n");
        return -1;
    }

    /* Update benchmarking. */
    BENCHMARK_UPDATE(Netfp_completeTargetHandOver);

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
            break;
        else
        {
            /* Call the encode funtion to encode these packets */
            Test_encodePacket (ueHandle, ptrRxPkt);
            bufferedPkts++;
        }
    }
    printf ("Debug: %d buffered packets were processed\n", bufferedPkts);
    printf ("Debug: Target HandOver is complete\n");

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
void Test_drbSPTargetHOTask(UArg arg0, UArg arg1)
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
    MsgCom_ChHandle             hoChannel;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    int32_t                     index;
    int32_t                     errCode;
    uint32_t                    events;
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
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }
    fpEgressV4Handle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressV4Handle == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
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

    /* Create the Message communicator channel. */
    gtpuChannelHandle = Msgcom_create ("GTPU_Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
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

    /* Create the Message communicator channel. */
    encodeChannel = Msgcom_create ("Encode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
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
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[2].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[2].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[2].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[2].hostInterrupt;

    /* Create the Message communicator channel. */
    decodeChannel = Msgcom_create ("Decode", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (decodeChannel == 0)
    {
        System_printf ("Error: Unable to open the decode channel Error : %d\n", errCode);
        return;
    }

     /************************************************************************************
     * Handover Channel: Channel created during Target Handover where initially all the
     * packets from the source eNodeB will be arriving.
     ************************************************************************************/

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = Test_handOverChannelCallback;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[4].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[4].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[4].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[4].hostInterrupt;


    /* Create the Message communicator channel. */
    hoChannel = Msgcom_create ("HO_Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (hoChannel == 0)
    {
        System_printf ("Error: Unable to open the handOver channel Error : %d\n", errCode);
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
    lteChannelBindCfg.chDrbRohc       = Msgcom_getInternalMsgQueueInfo(gtpuChannelHandle);
    lteChannelBindCfg.fpHandle        = fpIngressV4Handle;
    lteChannelBindCfg.sin_gtpuId      = gtpuIdentifier;
    lteChannelBindCfg.countC          = 0;
    lteChannelBindCfg.enableFastPath  = 0;
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
     * NETFP Downlink Handover Socket
     *  - To differentiate packets arriving from the SeGW and seNodeB we use different
     *    GTPU identifiers in the test case.
     ************************************************************************************/

    /* Create a socket to receive data from Source eNB during HandOver */
	lteHoSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (lteHoSockHandle == NULL)
	{
        System_printf ("Error: Failed to create the LTE HO socket to receive data from Source eNB [Error code %d]\n", errCode);
	    return;
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
        System_printf ("Error: Failed to bind the LTE HO channel [Error code %d]\n", errCode);
		return;
    }

    /* Debug Message: */
    System_printf ("---------------------------------------------------------------------------------------------\n");
    System_printf ("Please use a traffic generation application to send packets to the Slow Path\n");
    System_printf ("\n");
    System_printf ("HO GTPU Identifier is 0x%x\n", hoGtpuIdentifier);
    System_printf ("GTPU    Identifier is 0x%x\n", gtpuIdentifier);
    System_printf ("---------------------------------------------------------------------------------------------\n");

    /**********************************************************************************
     * Stress Test:
     *  This is the main loop where the task waits for data to arrive from the
     *  sockets.
     *********************************************************************************/
    while (1)
    {
        /* Wait for an event to occur in the system. */
        events = Event_pend(encodeDecodeEventObject, Event_Id_NONE, Event_Id_00 + Event_Id_01 + Event_Id_02 + Event_Id_03,
                        BIOS_WAIT_FOREVER);

        /* Check if we received a packet over the handover channel. */
        if (events & Event_Id_00)
        {
            /* YES. Cycle through and process all the received packets and push them for decoding. */
            while (1)
            {
                /* Process all packets on the GTPU channel. */
                Msgcom_getMessage (gtpuChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Packets should be received on the GTPU channel only if the handover
                 * is complete. */
                if (gInitiateSPTargetHO != TEST_TARGET_HO_COMPLETE)
                {
                    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                    continue;
                }

                /* These are packets which have been received from the serving gateway
                 * and which now need to be encoded */
                Test_encodePacket(ueHandle, ptrRxPkt);
            }
        }

        /* Check if we received a packet over the handover channel. */
        if (events & Event_Id_01)
        {
            /* YES. Cycle through and process all the received packets and push them for decoding. */
            while (1)
            {
                /* Process all packets on the handover channel. */
                Msgcom_getMessage (hoChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Packets should be received on the handover channel only if the handover
                 * is not complete and the source eNodeB is sending packets via the handover
                 * GTPU Identifier. Once the handover complete is signalled; these packets
                 * are dropped and are NOT processed. The packet generator should NOT send
                 * packets with the HO GTPU Identifier anymore. */
                if (gInitiateSPTargetHO != TEST_TARGET_HO_NOT_COMPLETE)
                {
                    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
                    continue;
                }

                /* These are packets which have been received from the source eNodeB
                 * and which now need to be encoded */
                Test_encodePacket(ueHandle, ptrRxPkt);
            }
        }

        /* Check if we received an encyrpted packet? */
        if (events & Event_Id_02)
        {
            /* YES. Cycle through and process all the received packets and push them for decoding. */
            while (1)
            {
                /* Process all packets on the encode channel. */
                Msgcom_getMessage (encodeChannel, (MsgCom_Buffer**)&ptrRxPkt);

                /* Did we process all the messages */
                if (ptrRxPkt == NULL)
                    break;

                /* Packet was ciphered, decipher the packet */
                Test_decodePacket(ueHandle, ptrRxPkt, decodeChannel);
            }
        }

        /* Check if we received a decyrpted packet? */
        if (events & Event_Id_03)
        {
            /* YES. Cycle through and process all the received packets and push them out */
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
        }
        /* Is the handover complete? Simulating the end marker.  */
        if (gInitiateSPTargetHO == TEST_TARGET_HO_COMPLETE_INITIATION)
        {
            /* Process the Target HO. If there is an error we kill the test case. */
            if (Test_processTargetHandOver(ueHandle, lteHoSockHandle) < 0)
                return;

            /* Once this is complete all data traffic arriving from the source eNodeB
             * should be completely stopped. */
            gInitiateSPTargetHO = TEST_TARGET_HO_COMPLETE;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Display the statistics for the DRB slow Path Target HandOver tests
 *
 *  @retval
 *      Not Applicable.
 */
void Test_displayDrbSPTargetHOStats(void)
{
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("LTE SP Target HandOver Tests::::\n");
    System_printf ("Target HandOver Status               : %s\n",
                    (gInitiateSPTargetHO == 0 ? "HO not complete" : (gInitiateSPTargetHO == 2 ? "HO Completed" : "HO started")));
    System_printf ("Number of IPv4 GTPU Pkt Encoded      : %d\n", gCountC);
    System_printf ("Number of IPv4 GTPU Pkt Decoded      : %d\n", gNumPktDecodedPktRxed);
    System_printf ("Number of IPv4 GTPU Pkt Transmitted  : %d\n", gNumPktTxed);
    BENCHMARK_DISPLAY(Netfp_encodeDrb);
    BENCHMARK_DISPLAY(Netfp_cipher);
    BENCHMARK_DISPLAY(Netfp_send);
    BENCHMARK_DISPLAY(Netfp_completeTargetHandOver);
    return;
}


