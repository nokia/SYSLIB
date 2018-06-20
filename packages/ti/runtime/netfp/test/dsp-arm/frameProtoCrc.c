/*
 *   @file  frameProtoCrc.c
 *
 *   @brief
 *      The file implements the following functionality for the WCDMA Frame Protocol channels:
 *          - creation Frame protocl channel
 *          - Verification of CRC for Ingress packets
 *          - Computation of CRC for Outgoing packets
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

/* Test UDP Ingress Base port number. */
#define TEST_SOCKET_IPV4_INGRESS_UDP_PORT           0xDE00

/* Test UDP Egress Base port number. */
#define TEST_SOCKET_IPV4_EGRESS_UDP_PORT            0xAB00

/* Maximum loops waiting for the packet to come. */
#define TEST_MAX_WAIT_COUNTER   1000

/* Maximum number of frame protocol channels. */
#define TEST_NUM_FP_CHANNEL     17 

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* eNodeB IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testNodeBIPAddress[] = { 192, 168, 1, 2 };

/* Serving Gateway IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testRncIPAddress[]   = { 192, 168, 1, 1 };

/* Serving Gateway MAC Address: */
static uint8_t  testRncMacAddress[] = { 0x90, 0xe2, 0xba, 0x36, 0x8b, 0x1c };

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* eNodeB MAC Address: */
extern uint8_t                  eNBMacAddress[6];

/* NETFP Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* Global MSGCOM Instance handle.*/
extern Msgcom_InstHandle        appMsgcomInstanceHandle;

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg       appResourceConfig;

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* Statistics Display Functionality. */
extern void Test_statsDisplaySAStats (Netfp_SaProtocol type, Sa_Stats_t* ptrStats);

extern int32_t Test_netSendIPv4Payload(uint8_t* ptrPayload, uint16_t payloadLen,
                uint32_t gtpuId, uint16_t gtpuMsgType, uint16_t dstPort,
                uint16_t srcPort, uint8_t* dstIP, uint8_t* srcIP, uint8_t tos,
                uint8_t* dstMAC, uint8_t* srcMAC, uint32_t loopback);

/**********************************************************************
 ********************* Unit Test Data Structures **********************
 **********************************************************************/

/**
 * @brief
 *  Frame Protocol Test Configuration.
 *
 * @details
 *  This is a data structure for only test purposes which is used to store the
 *  possible frame protocol test configurations.
 */
typedef struct Test_frameProtoCfg
{
    /**
     * @brief   Frame Protocol type
     */
    Netfp_FpType    fpType;

    /**
     * @brief   Indicate if CRC is enabled
     */
    uint8_t         enableCrc;

    /**
     * @brief   Number of multiplexed DCH bearers
     */
    uint8_t         numDchBearers;

    /**
     * @brief   Payload offset
     */
    uint8_t         payloadOffset;
}Test_frameProtoCfg;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to create the IPv4 test environment. In the loopback
 *      testing we are only interested in creating an ingress fast path since the
 *      test simulate different types of packets and their behavior on reception
 *      in the NETCP subsystem.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Test_setupFrameProtoCrcEnv(void)
{
    Netfp_InboundFPHandle       fpIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle;
    int32_t                     errCode;
    Netfp_InboundFPCfg          inboundFPCfg;
    Netfp_OutboundFPCfg         outboundFPCfg;
    uint32_t 			        index;
    Netfp_IfHandle              ifHandle;
    Netfp_InterfaceCfg          ifConfig;

    /* Display the configuration: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("NETFP Configuration: \n");
    System_printf ("eNodeB MAC  Address: 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                    eNBMacAddress[0], eNBMacAddress[1],eNBMacAddress[2],
                    eNBMacAddress[3], eNBMacAddress[4], eNBMacAddress[5]);
    System_printf ("RNC    MAC  Address: 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
                    testRncMacAddress[0], testRncMacAddress[1], testRncMacAddress[2],
                    testRncMacAddress[3], testRncMacAddress[4], testRncMacAddress[5]);
    System_printf ("eNodeB IPv4 Address: %02d.%02d.%02d.%02d\n",
                    testNodeBIPAddress[0], testNodeBIPAddress[1],
                    testNodeBIPAddress[2], testNodeBIPAddress[3]);
    System_printf ("RNC    IPv4 Address: %02d.%02d.%02d.%02d\n",
                    testRncIPAddress[0], testRncIPAddress[1],
                    testRncIPAddress[2], testRncIPAddress[3]);
    System_printf ("-------------------------------------------------------------\n");

    /***************************************************************************
     * Setup the Interface in the NETFP Library.
     *  - The NETFP library needs to know about the networking interfaces which
     *    are participating in the Fast Path.
     *
     * For the test code there is one known interface "eth0". On TN & SCBP this
     * is the NDK created interface but on Appleton this is the ARM created
     * network interface.
     ***************************************************************************/
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = testNodeBIPAddress[0];
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = testNodeBIPAddress[1];
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = testNodeBIPAddress[2];
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = testNodeBIPAddress[3];
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth0");
    memcpy ((void *)&ifConfig.macAddress, (void *)&eNBMacAddress[0], 6);

    /* Create & Register the Interface with the NETFP Library. */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to create the NETFP interface\n");
        return -1;
    }
    System_printf ("Debug: Interface '%s' is created with Handle 0x%x\n", ifConfig.name, ifHandle);

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = testRncIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = testRncIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = testRncIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = testRncIPAddress[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = testNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = testNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = testNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = testNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);

    /* Initialize the fast path configuration */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = testRncIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = testRncIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = testRncIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = testRncIPAddress[3];
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = testNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = testNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = testNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = testNodeBIPAddress[3];
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* Fill in the mapping table */
    for (index = 0; index < 64; index++)
        outboundFPCfg.dscpMapping[index] = index;

    /* Populate the manual routing section: */
    outboundFPCfg.ifHandle = ifHandle;
    outboundFPCfg.nextHopMACAddress[0] = testRncMacAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = testRncMacAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = testRncMacAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = testRncMacAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = testRncMacAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = testRncMacAddress[5];

    /* Add the Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath(netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);

    return 0;
}

static int32_t Test_getFrameProtoConfig (char *testConfigName, Test_frameProtoCfg* ptrFpTestConfig)
{
    if (ptrFpTestConfig == NULL || testConfigName == NULL)
        return -1;

    if (strcmp(testConfigName, "HS_DSCH_TYPE1") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_HS_DSCH_TYPE1;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 7;
    }
    else if (strcmp(testConfigName, "HS_DSCH_TYPE2") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_HS_DSCH_TYPE2;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 0;
    }
    else if (strcmp(testConfigName, "HS_DSCH_TYPE3") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_HS_DSCH_TYPE3;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 0;
    }
    else if (strcmp(testConfigName, "DL_DCH_NO_CRC") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_DCH;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 0;
    }
    else if (strcmp(testConfigName, "DL_DCH_CRC_1") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_DCH;
        ptrFpTestConfig->enableCrc      = 1;
        ptrFpTestConfig->numDchBearers  = 1;
        ptrFpTestConfig->payloadOffset  = 3;
    }
    else if (strcmp(testConfigName, "DL_DCH_CRC_2") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_DCH;
        ptrFpTestConfig->enableCrc      = 1;
        ptrFpTestConfig->numDchBearers  = 2;
        ptrFpTestConfig->payloadOffset  = 4;
    }
    else if (strcmp(testConfigName, "DL_DCH_CRC_3") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_DCH;
        ptrFpTestConfig->enableCrc      = 1;
        ptrFpTestConfig->numDchBearers  = 3;
        ptrFpTestConfig->payloadOffset  = 5;
    }
    else if (strcmp(testConfigName, "DL_DCH_CRC_4") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_DCH;
        ptrFpTestConfig->enableCrc      = 1;
        ptrFpTestConfig->numDchBearers  = 4;
        ptrFpTestConfig->payloadOffset  = 6;
    }
    else if (strcmp(testConfigName, "DL_PCH") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_PCH;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 4;
    }
    else if (strcmp(testConfigName, "DL_FACH") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_DL_FACH;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 4;
    }
    else if (strcmp(testConfigName, "DL_EDCH") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_EDCH;
        ptrFpTestConfig->enableCrc      = 0;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 0;
    }
    else if (strcmp(testConfigName, "DL_EDCH_HFI") == 0)
    {
        ptrFpTestConfig->fpType         = Netfp_FpType_EDCH;
        ptrFpTestConfig->enableCrc      = 1;
        ptrFpTestConfig->numDchBearers  = 0;
        ptrFpTestConfig->payloadOffset  = 6;
    }
    else
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to get and validate the frame protocol packet.
 *
 *  @param[in]  dstQueue
 *      Destination Queue where the received packet should be present
 *  @param[in]  bindingId
 *      Binding ID of the Frame protocol channel used for validation
 *  @param[in]  payloadData
 *      Expected payload data
 *  @param[in]  payloadLen
 *      Expected payload length
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_processFrameProtoPacket
(
    Qmss_QueueHnd   dstQueue,
    uint32_t        bindingId,
    uint8_t*        payloadData,
    uint32_t        payloadLen
)
{
    Ti_Pkt*             ptrRxPkt;
    int32_t             index;
    int32_t             retVal;
    uint8_t             errorFlag;
    Netfp_PeerSockAddr  sockAddr;

    /* Set the index and loop around waiting for the packet to come. */
    index = 0;
    while (1)
    {
        uint8_t*    ptrRxDataBuffer;
        uint32_t    rxDataBufferLen;
        uint32_t    swInfo0;

        /* Did we get the packet? */
        ptrRxPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop (dstQueue));
        if (ptrRxPkt == NULL)
        {
            /* Have we exceeded the MAX wait counter? */
            if (index++ == TEST_MAX_WAIT_COUNTER)
                return -1;

            /* No. Keep trying. */
            continue;
        }

        /**************************************************************************
         * NOTE: The packet is directly getting popped off the queue and so in this
         * case the application is responsible for getting ownership of the packet.
         **************************************************************************/
        Pktlib_getOwnership (appPktlibInstanceHandle, ptrRxPkt);

        /* Get the software information of the packet. */
        swInfo0 = Cppi_getSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc*)ptrRxPkt);

        /* Is this our binding ID? */
        if (swInfo0 != bindingId)
        {
            System_printf ("Error: Binding ID mismatch. Expected 0x%x, Got 0x%x\n", bindingId, swInfo0);
            break;
        }

        /* Get the data buffer length */
        Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

        /* Invalidate the cache contents. */
        appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

        /* Skip the networking headers and get the Frame Protocol packet. */
        retVal = Netfp_getPayload (ptrRxPkt, 0, &ptrRxDataBuffer, &rxDataBufferLen, &sockAddr);
        if (retVal < 0)
        {
            System_printf ("Error: FP CRC Channel Get Payload failed for packet 0x%p\n", ptrRxPkt);
            break;
        }

        /* Sanity Check: Ensure that there is a perfect match on the data buffer length */
        if (rxDataBufferLen != payloadLen)
        {
            System_printf ("Error: Sent %d bytes, Got %d bytes.\n", payloadLen, rxDataBufferLen);
            break;
        }

        /* Sanity Check: Compare the received packet with the transmitted packet. */
        for (index = 0; index < rxDataBufferLen; index++)
        {
            if (*(payloadData + index) != *(ptrRxDataBuffer + index))
            {
                System_printf ("Error: Data Mismatch @ Index %d Expected 0x%x Got 0x%x\n", index,
                        *(payloadData + index), *(ptrRxDataBuffer + index));
                break;
            }
        }

        /* Sanity Check: Ensure that the error flags are clear. */
        errorFlag = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc*)ptrRxPkt) & 0xf;

        if (errorFlag != 0)  {
            System_printf ("Error: Packet returned with error flags = 0x%02x\n", errorFlag);
            break;
        }

        /* Packet was received; test passed. Cleanup the packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);

        return 0;
    }

    /* Control comes here implies that the test failed. Cleanup the packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
    return -1;
}

/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_frameProtocolCrcTask(UArg arg0, UArg arg1)
{
    Netfp_InboundFPHandle       fpIngressV4Handle;
    Netfp_OutboundFPHandle      fpEgressV4Handle;
    Netfp_FlowCfg               flowCfg;
    int32_t                     rxFlowId;
    uint32_t                    index, i, numPduBlocks;
    int32_t                     errCode, retVal;
    uint32_t                    bindingId = 0xB19D1D00;
    FILE*                       ptrTestFile;
    Test_frameProtoCfg          frameProtoTestCfg;
    uint8_t*                    payloadData;
    uint16_t                    tempData;
    char                        configName[30];
    char                        fileName[60];
    uint32_t                    payloadLen = 0;
    Netfp_SockHandle            sockHandle;
    Netfp_SockAddr              sockAddress;
    Qmss_QueueHnd               udpDestQueue;
    uint8_t                     isAllocated;
    Netfp_OptionTLV             optInfo;
    Netfp_Sock3GPPCfg           sock3GPPcfg;

    /* Setup the default test environment */
    if (Test_setupFrameProtoCrcEnv() < 0)
        return;

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

    /* Open a General Purpose Queue for the UDP/FP Data */
    udpDestQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                      QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (udpDestQueue == NULL)
    {
        System_printf ("Error: Unable to open the Fast Path Rx Queue\n");
        return;
    }

    /* Run the test for multiple iterations: Adding & Deleting sockets. */
    for (index = 0; index < TEST_NUM_FP_CHANNEL; index++)
    {
        /* Get the Frame Protocol test parameters from external file */
        sprintf (fileName, "C:/test_vectors/test_crc/test%d.txt", index+1);

        /* Open the test vector file. */
        ptrTestFile = fopen (fileName, "r");
        if (ptrTestFile != NULL)
        {
            /* Get configuration name */
            fscanf (ptrTestFile, "%s", configName);
            System_printf ("Debug: Testing configuration: %s\n", configName);

            /* Read in the test vector. */
            fscanf (ptrTestFile, "%d", &payloadLen);
            tempData = 0;
            payloadData = (uint8_t*) Memory_alloc (NULL, payloadLen, 0, NULL);
            if (payloadData == NULL)
            {
                System_printf ("Error: Failed to allocate payload of %d bytes.\n", payloadLen);
                return;
            }

            /* Populate the payload data. */
            for (i = 0; i < payloadLen; i++)
            {
                if (feof(ptrTestFile)) break;
                fscanf (ptrTestFile, "%hx", &tempData);
                *(payloadData+i) = tempData;
            }

            /* Get test parameters corresponding to config name */
            if (Test_getFrameProtoConfig (configName, &frameProtoTestCfg) < 0)
            {
                System_printf ("Error: Invalid Frame Protocol test configuration.\n");
                break;
            }

            /* Calculate the payload offset for HS-DSCH Type 2 and Type 3 based on the payload. */
            if (frameProtoTestCfg.fpType == Netfp_FpType_HS_DSCH_TYPE2)
            {
                numPduBlocks = *(payloadData+2) >> 3;
                frameProtoTestCfg.payloadOffset = (numPduBlocks % 2) ? (6.5 + 2.5*numPduBlocks) : (6 + 2.5*numPduBlocks);
            }
            else if (frameProtoTestCfg.fpType == Netfp_FpType_HS_DSCH_TYPE3)
            {
                numPduBlocks = *(payloadData+3) >> 3;
                frameProtoTestCfg.payloadOffset = (numPduBlocks % 2) ? (4.5 + 2.5*numPduBlocks) : (4 + 2.5*numPduBlocks);
            }
        }
        else
        {
            /* If test file does not exist, then exit */
            System_printf ("Error: Failed to open test vector file %s.\n", fileName);
            return;
        }

        /* Create a socket */
    	sockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
        if (sockHandle == NULL)
    	{
            System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
	        return;
        }

        sock3GPPcfg.protocol = Netfp_3GPPProto_FRAME_PROTOCOL;
        sock3GPPcfg.cfg.frameProtoCfg.enableRxFpCrc = frameProtoTestCfg.enableCrc;
        sock3GPPcfg.cfg.frameProtoCfg.fpType = frameProtoTestCfg.fpType;
        sock3GPPcfg.cfg.frameProtoCfg.numDchBearers = frameProtoTestCfg.numDchBearers;

        optInfo.type   = Netfp_Option_SOCK_3GPP_CFG;
        optInfo.length = sizeof(Netfp_Sock3GPPCfg);
        optInfo.value  = (void*)&sock3GPPcfg;

        if (Netfp_setSockOpt (sockHandle, &optInfo, &errCode) < 0)
        {
	        System_printf ("Error: NETFP Set socket option Failed [Error Code %d]\n", errCode);
    		return;
        }

        /* Populate the binding information */
        memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    	sockAddress.sin_port                = TEST_SOCKET_IPV4_INGRESS_UDP_PORT + index;
        sockAddress.op.bind.appInfo         = bindingId + index;
        sockAddress.op.bind.inboundFPHandle = fpIngressV4Handle;
        sockAddress.op.bind.flowId          = rxFlowId;
    	sockAddress.op.bind.queueHandle     = udpDestQueue;

        /* Bind the socket. */
	    if (Netfp_bind (sockHandle, &sockAddress, &errCode) < 0)
        {
	        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
		    return;
        }
    	System_printf ("Debug: Socket has been bound successfully\n");

        /* Is Frame Protocol payload CRC enabled for Tx? */
        if (!(strcmp (configName, "DL_EDCH") == 0 || strcmp (configName, "DL_DCH_NO_CRC") == 0))
        {
            uint8_t enableTxFpCrc = 1;

            optInfo.type   = Netfp_Option_FRAME_PROTOCOL_CRC_OFFLOAD;
            optInfo.length = sizeof(uint8_t);
            optInfo.value  = (void*)&enableTxFpCrc;

            if (Netfp_setSockOpt (sockHandle, &optInfo, &errCode) < 0)
            {
	            System_printf ("Error: NETFP Set socket option Failed [Error Code %d]\n", errCode);
    		    return;
            }
        }

        /* Populate the connect information. */
    	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
        sockAddress.sin_port                    = TEST_SOCKET_IPV4_EGRESS_UDP_PORT + index;
    	sockAddress.op.connect.outboundFPHandle = fpEgressV4Handle;

        /* Connect the socket */
    	if (Netfp_connect(sockHandle, &sockAddress, &errCode) < 0)
        {
	        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
    		return;
        }
        System_printf ("Debug: Socket has been connected successfully\n");

        /* Test the Ingress Packet Flow:- */
        retVal = Test_netSendIPv4Payload(payloadData,                               /* Payload Data      */
                                         payloadLen,                                /* Payload Length    */
                                         0,                                         /* GTPU Identifier   */
                                         0,                                         /* GTPU Message Type */
                                         TEST_SOCKET_IPV4_INGRESS_UDP_PORT + index, /* Destination Port  */
                                         TEST_SOCKET_IPV4_INGRESS_UDP_PORT + index, /* Source Port       */
                                         &testNodeBIPAddress[0],                    /* Destination IP    */
                                         &testRncIPAddress[0],                      /* Source IP         */
                                         0,                                         /* TOS               */
                                         &eNBMacAddress[0],                         /* Destination MAC   */
                                         &testRncMacAddress[0],                     /* Source MAC        */
                                         1);                                        /* Loopback          */

        if (retVal < 0)
        {
            System_printf ("Error: Sending Frame Protocol CRC Message failed\n");
            return;
        }

        /* Process the received packets in the destination queue. */
        if (Test_processFrameProtoPacket(udpDestQueue,
                                         bindingId+index,
                                         payloadData,
                                         payloadLen) < 0)
        {
            /* Error: Test Failed */
            System_printf ("Error: UDP port number 0x%x Data Test timed out.\n", TEST_SOCKET_IPV4_INGRESS_UDP_PORT + index);
            return;
        }

        /* Test the Egress Packet Flow:- */
        {
            Ti_Pkt*     ptrUDPPayload;
            uint8_t*    ptrDataBuffer;
            uint32_t    dataBufferLen;

            /* Allocate a packet from the heap */
            ptrUDPPayload = Pktlib_allocPacket(appPktlibInstanceHandle, mtuReceiveHeap, payloadLen);
            if (ptrUDPPayload == NULL)
                return;

            /* Get the data buffer. */
            Pktlib_getDataBuffer(ptrUDPPayload, &ptrDataBuffer, &dataBufferLen);

            /* Invalidate the cache contents. */
            appInvalidateBuffer(ptrDataBuffer, dataBufferLen);

            /* Initialize the packet payload. */
            memcpy ((void *)ptrDataBuffer, payloadData, dataBufferLen);

            /* Set CRC field in the packet to 0. */
//            if (!(strcmp (configName, "DL_EDCH") == 0 || strcmp (configName, "DL_DCH_NO_CRC") == 0 || strcmp (configName, "DL_EDCH_HFI")))
            memset (ptrDataBuffer+dataBufferLen-2, 0x00, 2);

            /* Writeback the data payload */
            appWritebackBuffer(ptrDataBuffer, dataBufferLen);

            /* Send the UDP Packet. */
            if (Netfp_send (sockHandle, ptrUDPPayload, frameProtoTestCfg.payloadOffset, &errCode) < 0)
            {
                System_printf ("Error: NetFP send failed [Error Code %d]\n", errCode);
                return;
            }
        }

        /* Delete the Frame Protocol Channel which has been created above. */
        if (Netfp_closeSocket (sockHandle, &errCode) < 0)
        {
            /* Error: Test failed. */
            System_printf ("Error: Failed to close socket [Error Code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: %s test (UDP port 0x%x) passed.\n", configName, TEST_SOCKET_IPV4_INGRESS_UDP_PORT + index);

        /* Free the buffer and close the file. */
        Memory_free (NULL, payloadData, payloadLen);
        fclose (ptrTestFile);
    }
    System_printf ("Debug: All Frame Protocol CRC Tests Passed\n");
}


