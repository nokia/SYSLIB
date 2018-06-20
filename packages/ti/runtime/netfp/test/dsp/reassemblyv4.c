/*
 *   @file  reassemblyv4.c
 *
 *   @brief   
 *      The file implements loopback testing for IPv4 reassembly. 
 *      The packets are read from a test vector file and then pushed into 
 *      the NETCP subsystem. This will cause the NETFP module to reassemble
 *      the packets and pass it back to the NETFP Client where the received
 *      packets are validated.
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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>

/* Test Include Files*/
#include <net.h>

/**********************************************************************
 *********************** Unit Test Definitions ************************
 **********************************************************************/

/* Maximum loops waiting for the packet to come. */
#define TEST_MAX_WAIT_COUNTER   1000

/* Maximum number of fast paths and frame protocol channels. */
#define TEST_NUM_TRAFFIC_FLOW    3
#define TEST_NUM_CHANNEL_PER_TF  4
#define TEST_NUM_FRAGS           36
#define TEST_NUM_NONFRAGS        2

/* Networking header sizes for IPv4, IPv6 */
#define HEADER_SIZE              42

#undef TEST_DEBUG 

/**********************************************************************
 ************************** Extern Functions **************************
 **********************************************************************/

/* Heap for data receive and transmit. */
extern Pktlib_HeapHandle        mtuReceiveHeap;

/* NETFP Client Handle: */
extern Netfp_ClientHandle       netfpClientHandle;

/* eNodeB MAC Address: */
extern uint8_t                  eNBMacAddress[6];

/* PKTLIB Instance Handle. */
extern Pktlib_InstHandle        appPktlibInstanceHandle;

/* Extern functions: */
extern int32_t Test_netSendPacket(uint8_t* ptrBuffer, uint16_t packetLen, uint32_t loopback);
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** Unit Test Globals *************************
 **********************************************************************/

/* List of fast path handles. */
Netfp_InboundFPHandle      fpHandle[TEST_NUM_TRAFFIC_FLOW];

/* List of all the socket handles */
Netfp_SockHandle    sockHandle[TEST_NUM_TRAFFIC_FLOW][TEST_NUM_CHANNEL_PER_TF];

/* eNodeB IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testNodeBIPAddress[] = { 0x64, 0x64, 0x01, 0x32 };

/* RNC IP Address: This is the address which is specified in the test vectors. */
static uint8_t  testRNCIPAddress[]   = { 0x64, 0x64, 0x01, 0x00 };

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function is used to add the specific number of fast paths. 
 *      The configuration of the fast paths is derived from the test vectors
 *      file.
 *
 *  @param[in]  numFastPath
 *      Number of fast paths to be created
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_setupFastPaths (uint32_t numFastPath)
{
    uint32_t            index;
    int32_t             errCode;
    Netfp_InboundFPCfg  fpConfig;

    /* Create the ingress fast paths. */
    for (index = 0; index < numFastPath; index++)
    {
        /* Initialize the fast path configuration. */
        memset ((void *)&fpConfig, 0, sizeof (Netfp_InboundFPCfg));

        /* Populate the Fast Path configuration. */
        fpConfig.spidMode                 = Netfp_SPIDMode_INVALID;
        fpConfig.spId					 = NETFP_INVALID_SPID;
        fpConfig.srcIP.ver               = Netfp_IPVersion_IPV4;
        fpConfig.dstIP.ver               = Netfp_IPVersion_IPV4;
        fpConfig.srcIP.addr.ipv4.u.a8[0] = testRNCIPAddress[0];
        fpConfig.srcIP.addr.ipv4.u.a8[1] = testRNCIPAddress[1];
        fpConfig.srcIP.addr.ipv4.u.a8[2] = testRNCIPAddress[2];
        fpConfig.srcIP.addr.ipv4.u.a8[3] = testRNCIPAddress[3] + index; 
        fpConfig.dstIP.addr.ipv4.u.a8[0] = testNodeBIPAddress[0];
        fpConfig.dstIP.addr.ipv4.u.a8[1] = testNodeBIPAddress[1];
        fpConfig.dstIP.addr.ipv4.u.a8[2] = testNodeBIPAddress[2];
        fpConfig.dstIP.addr.ipv4.u.a8[3] = testNodeBIPAddress[3]; 
        sprintf (fpConfig.name, "RNC-Test%d", index);

        /* Create the fast path */
        fpHandle[index] = Netfp_createInboundFastPath (netfpClientHandle, &fpConfig, &errCode);
        if (fpHandle[index] == NULL)
        {
            System_printf ("Error: Unable to create the fast path %s [Error code %d]\n", fpConfig.name, errCode);
            return -1;
        }
        System_printf ("Debug: Create the fast path %s Handle %p\n", fpConfig.name, fpHandle[index]);
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create the sockets which are used to receive the 
 *      reassembled packets
 *
 *  @param[in]  destQueue
 *      This is the queue where the packet need to land up after reassembly.
 *  @param[in]  numFastPath
 *      This is the number of fast path 
 *  @param[in]  numChannels
 *      Number of channels per fast path 
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_setupSockets
(
    Qmss_QueueHnd       destQueue,
    uint32_t            numFastPath,
    uint32_t            numChannels
)
{
    Netfp_SockAddr      sockAddress;
    int32_t             rxFlowId;
    Netfp_FlowCfg       flowCfg;
    int32_t             errCode;
    uint32_t            index;
    uint32_t            tf;
    uint16_t            udpPortNum = 0xDE00;

    /* Populate the flow configuration: */
    strcpy (flowCfg.name, "ReassemblyFlow");
    flowCfg.numHeaps      = 1;
    flowCfg.heapHandle[0] = mtuReceiveHeap;
    flowCfg.sopOffset     = 0;

    /* Create the flow. */
    rxFlowId = Netfp_createFlow (netfpClientHandle, &flowCfg, &errCode);
    if (rxFlowId < 0)
    {
        printf ("Error: Unable to create the flow [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Flow %d has been created successfully\n", rxFlowId);

    /* Add multiple channels per fast path. */
    for (tf = 0; tf < numFastPath; tf++)
    {
        for (index = 0; index < numChannels; index++)
        {
            /* Create a socket */
        	sockHandle[tf][index] = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
            if (sockHandle[tf][index] == NULL)
        	{
                printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
	            return -1;
            }

            /* Populate the binding information */
            memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
        	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;            
        	sockAddress.sin_port                = udpPortNum++;
            sockAddress.op.bind.inboundFPHandle = fpHandle[tf];
            sockAddress.op.bind.flowId          = rxFlowId;
        	sockAddress.op.bind.queueHandle     = destQueue;

            /* Bind the socket. */
        	if (Netfp_bind (sockHandle[tf][index], &sockAddress, &errCode) < 0)
            {
	            printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        		return -1;
            }
	        printf ("Debug: Socket[%d][%d] has been bound successfully to port %d\n", tf, index, sockAddress.sin_port);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Tests reassembly of out of order fragments.
 *          - Out of sequence packets.
 *          - Multiple traffic flows.
 *          - Includes full packets that match and don't match 
 *            active traffic flows.
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
static int32_t Test_reassemblyIPv4 (Qmss_QueueHnd destQueue)
{
    int32_t     		index;
    uint32_t     		headerSize;
    uint32_t            payloadLen = 0;
    FILE*               ptrTestFile;
    Ti_Pkt*             ptrRxPkt;
    uint32_t            counter = 0; 
    uint32_t            i;
    int32_t             testSuccess;
    char                testName[30];
    uint8_t*            payloadData;
    uint16_t            tempData;

    /* Set the flag to indicate that the test was successful. */
    testSuccess = 1;

    /* Populate test name. */
    strcpy (testName, "IPv4 Reassembly test #1");

    /* Determine sum of all header sizes. */
    headerSize = HEADER_SIZE;

    /* Test vector file. */
    ptrTestFile = fopen ("C:/test_vectors/test_reassembly/fragments.txt", "r");
    if (ptrTestFile == NULL)
    {
        printf ("Error: %s -- Unable to open test vector file\n", testName);
        return -1;
    }

    /* Test the reassembly flow by sending fragment packets to PA */
    for (index = 0; index < (TEST_NUM_FRAGS + TEST_NUM_NONFRAGS); index++)
    {
        /* Get fragment packet from test file:- */
        if (ptrTestFile != NULL)
        {
            /* Get payload info */
            fscanf (ptrTestFile, "%d", &payloadLen);
            tempData = 0;

            /* Allocate memory for the payload data. */
            payloadData = (uint8_t*) Memory_alloc (NULL, payloadLen, 0, NULL);
            if (payloadData == NULL)
            {
                testSuccess = 0;
                printf ("Error: %s -- Malloc of %d bytes for payload data unsuccessful.\n", testName, payloadLen);
                return -1;
            }
            
            /* Populate the payload data from the input file. */
            for (i = 0; i < payloadLen; i++)
            {
                if (feof(ptrTestFile)) break;
                fscanf (ptrTestFile, "%hx", &tempData);
                *(payloadData+i) = tempData;
            }
        }

        /* Populate the MAC address. */
        for (i = 0; i < 6; i++)
            *(payloadData+i) = eNBMacAddress[i];

        /* Send the packet to the NETCP subsystem and allow the packet to get classified */
        if (Test_netSendPacket (payloadData, payloadLen, 1) < 0)
        {
            printf ("Error: %s -- Ingress packet flow failed.\n", testName);
            testSuccess = 0;
        }
        
        /* Ingress path has been tested. Free the data buffer. */
        Memory_free (NULL, payloadData, payloadLen);
    }

    /* Close the test vectors file. */
    if (ptrTestFile != NULL)
        fclose (ptrTestFile);

    printf ("Debug: %s -- Sent fragments to NetCP. Awaiting reassembled packets.\n", testName);
    Task_sleep (10);

    /* Process Received packets */
    for (index = 0; index < (TEST_NUM_FRAGS/3) + TEST_NUM_NONFRAGS; index++)
    {
        uint8_t     errorFlag;
        uint8_t*    ptrRxDataBuffer;
        uint32_t    rxDataBufferLen;
        uint32_t    index1;
        uint32_t    packetLen;

        /* Set the index and loop around waiting for the packet to come. */
        counter = 0;
        while (1)
        {
            /* Did we get the packet? */
            ptrRxPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop (destQueue));
            if (ptrRxPkt == NULL)
            {
                /* Have we exceed the MAX Wait counter */
                if (counter++ == TEST_MAX_WAIT_COUNTER)
                {
                    printf ("Error: %s -- Packet Id: %d: Exceeded max wait counter. Dest Queue is %d\n", testName, index, destQueue);
                    testSuccess = 0;
                    break;
                }

                /* Keep trying. */
                continue;
            }

            /**************************************************************************
             * NOTE: The packet is directly getting popped off the queue and so in this
             * case the application is responsible for getting ownership of the packet.
             **************************************************************************/
            Pktlib_getOwnership (appPktlibInstanceHandle, ptrRxPkt);

            /* Get the data buffer & buffer length */
            Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

            /* Discount the 4 bytes of Ethernet FCS */
            packetLen = Pktlib_getPacketLen(ptrRxPkt);
            Pktlib_setPacketLen (ptrRxPkt, packetLen - 4);
            rxDataBufferLen = rxDataBufferLen - 4;
            Pktlib_setDataBufferLen (ptrRxPkt, rxDataBufferLen);

            /* Invalidate the cache contents. */
            appInvalidateBuffer(ptrRxDataBuffer, rxDataBufferLen);

            printf ("Debug: %s -- Received a packet of length %d bytes [%d Chain %d].\n", testName, rxDataBufferLen, 
                    Pktlib_getPacketLen(ptrRxPkt), Pktlib_packetBufferCount(ptrRxPkt));

            /* Sanity Check: Compare the received packet with the transmitted packet: Discounting the 4 extra bytes of 
             * Ethernet FCS which had been added by the reassembly module. */
            for (index1 = headerSize; index1 < Pktlib_getPacketLen(ptrRxPkt); index1++)
            {
                if (index1 == (rxDataBufferLen - 1))
                {
                    if (*(ptrRxDataBuffer + index1) != 0xab)
                    {
                        printf ("Error: %s -- Data Mismatch @ Index %d Expected 0x%x Got 0x%x\n", testName, index1,
                                0xab, *(ptrRxDataBuffer + index1));
                        testSuccess = 0;
                    }
                }
                else if (*(ptrRxDataBuffer + index1) != (index + 1))
                {
                    printf ("Error: %s -- Data Mismatch @ Index %d Expected 0x%x Got 0x%x\n", testName, index1,
                            (index + 1), *(ptrRxDataBuffer + index1));
                    testSuccess = 0;
                    break;
                }
            }
#ifdef TEST_DEBUG 
            /* Debug: Dump the received packet. */
            for (index1 = 0; index1 < rxDataBufferLen; index1++)
            {
                if (index1 % 4 == 0)
                {
                    printf(" ");
                    if (index1 %16 == 0)
                        printf("\n");
                }
                printf ("%02x ", *(ptrRxDataBuffer + index1));
            }
            printf ("\n");
#endif
            /* Sanity Check: Ensure that the error flags are clear. */
            errorFlag = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc*)ptrRxPkt) & 0xf;
            if (errorFlag != 0)  {
                printf ("Error: %s -- Packet returned with error flags = 0x%02x\n", testName, errorFlag);
                testSuccess = 0;
            }

            /* Cleanup the packet. */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
            break;
        }
    }
    return testSuccess;
}

/**
 *  @b Description
 *  @n  
 *      Tests reassembly behavior in case of duplicate and missing fragments.
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
int32_t Test_reassemblyIPv4MissingDuplicate (Qmss_QueueHnd destQueue)
{
    int32_t     		index;
    uint32_t            payloadLen = 0;
    uint32_t 			headerSize;
    FILE*               ptrTestFile;
    Ti_Pkt*             ptrRxPkt;
    uint32_t            counter = 0, i;
    int32_t             testSuccess;
    uint32_t            numTestFragments;
    char                testName[30];
    uint8_t*            payloadData;
    uint16_t            tempData;

    /* Set the flag to indicate that the test was successful. */
    testSuccess = 1;

    /* Number of input fragments. */
    numTestFragments = 7;

    /* Populate test name. */
    strcpy (testName, "IPv4 Reassembly test #2");
        
    /* Determine sum of all header sizes. */
    headerSize = HEADER_SIZE;

    /* Test vector file. */
    ptrTestFile = fopen ("C:/test_vectors/test_reassembly/missingduplicatefrags.txt", "r");
    if (ptrTestFile == NULL)
    {
        printf ("Error: %s -- Unable to open test vector file\n", testName);
        testSuccess = 0;
        return -1;
    }

    /* Test the reassembly flow by sending fragment packets to PA */
    for (index = 0; index < numTestFragments; index++)
    {
        /* Get fragment packet from test file:- */
        if (ptrTestFile != NULL)
        {
            /* Get payload info */
            fscanf (ptrTestFile, "%d", &payloadLen);
            tempData = 0;

            /* Allocate memory for the payload data. */
            payloadData = (uint8_t*) Memory_alloc (NULL, payloadLen, 0, NULL);
            if (payloadData == NULL)
            {
                testSuccess = 0;
                printf ("Error: %s -- Malloc of %d bytes for payload data unsuccessful.\n", testName, payloadLen);
                return -1;
            }
            
            /* Populate the payload data from the input file. */
            for (i = 0; i < payloadLen; i++)
            {
                if (feof(ptrTestFile)) break;
                fscanf (ptrTestFile, "%hx", &tempData);
                *(payloadData+i) = tempData;
            }
        }

        /* Populate the MAC address. */
        for (i = 0; i < 6; i++)
            *(payloadData+i) = eNBMacAddress[i];
        
        /* Send the packet to the NETCP subsystem and allow the packet to get classified */
        if (Test_netSendPacket (payloadData, payloadLen, 1) < 0)
        {
            printf ("Error: %s -- Ingress packet flow failed.\n", testName);
            testSuccess = 0;
        }
        
        /* Ingress path has been tested. Free the data buffer. */
        Memory_free (NULL, payloadData, payloadLen);
    }

    if (ptrTestFile != NULL)
        fclose (ptrTestFile);

    printf ("Debug: %s -- Sent fragments to NetCP. Awaiting reassembled packets.\n", testName);
    Task_sleep(10);

    /* Process Received packets */
    for (index = 0; index < 2; index++ )
    {
        uint8_t     errorFlag;
        uint8_t*    ptrRxDataBuffer;
        uint32_t    rxDataBufferLen;
        uint32_t    index1;
        uint32_t    packetLen;

        /* Set the index and loop around waiting for the packet to come. */
        counter = 0;
        while (1)
        {
            /* Did we get the packet? */
            ptrRxPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop (destQueue));
            if (ptrRxPkt == NULL)
            {
                /* Have we exceed the MAX Wait counter */
                if (counter++ == TEST_MAX_WAIT_COUNTER)
                {
                    if (index == 0)
                    {
                        printf ("Error: %s -- Exceeded max wait counter. Did not receive reassembled packet.\n", testName); 
                        testSuccess = 0;
                        break;
                    }
                    else
                        break;
                }

                /* Keep trying. */
                continue;
            }

            /**************************************************************************
             * NOTE: The packet is directly getting popped off the queue and so in this
             * case the application is responsible for getting ownership of the packet.
             **************************************************************************/
            Pktlib_getOwnership (appPktlibInstanceHandle, ptrRxPkt);

            /* Get the data buffer length */
            Pktlib_getDataBuffer(ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

            /* Discount the 4 bytes of Ethernet FCS */
            packetLen = Pktlib_getPacketLen(ptrRxPkt);
            Pktlib_setPacketLen (ptrRxPkt, packetLen - 4);
            rxDataBufferLen = rxDataBufferLen - 4;
            Pktlib_setDataBufferLen (ptrRxPkt, rxDataBufferLen);            

            /* Invalidate the cache contents. */
            appInvalidateBuffer (ptrRxDataBuffer, rxDataBufferLen);

            printf ("Debug: %s -- Received a packet of length %d bytes [%d Chain %d].\n", testName, rxDataBufferLen, 
                    Pktlib_getPacketLen(ptrRxPkt), Pktlib_packetBufferCount(ptrRxPkt));

            /* Sanity Check: Compare the received packet with the transmitted packet. */
            for (index1 = headerSize; index1 < rxDataBufferLen; index1++)
            {
                if (index1 == (rxDataBufferLen - 1))
                {
                    if (*(ptrRxDataBuffer + index1) != 0xab)
                    {
                        printf ("Error: %s -- Data Mismatch @ Index %d Expected 0x%x Got 0x%x\n", testName, index1,
                                0xab, *(ptrRxDataBuffer + index1));
                    }
                }
                else if (*(ptrRxDataBuffer + index1) != (index + 1))
                {
                    printf ("Error: %s -- Data Mismatch @ Index %d Expected 0x%x Got 0x%x\n", testName, index1,
                            (index + 1), *(ptrRxDataBuffer + index1));
                    break;
                }
            }
#ifdef TEST_DEBUG 
            /* Sanity Check: Compare the received packet with the transmitted packet. */
            printf ("Debug: %s -- Received Packet Data of Length %d: ", testName, rxDataBufferLen);
            for (index1 = 0; index1 < rxDataBufferLen; index1++)
            {
                if (index1 % 4 == 0)
                {
                    printf(" ");
                    if (index1 %16 == 0)
                        printf("\n");
                }
                printf ("%02x ", *(ptrRxDataBuffer + index1));
            }
            printf ("\n");
#endif
            /* Sanity Check: Ensure that the error flags are clear. */
            errorFlag = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc*)ptrRxPkt) & 0xf;

            if (errorFlag != 0)  {
                printf ("Error: %s -- Packet returned with error flags = 0x%02x\n", testName, errorFlag);
            }

            /* Cleanup the packet. */
            Pktlib_freePacket(appPktlibInstanceHandle, ptrRxPkt);
            break;
        } 
    }
    return testSuccess;
}

/**
 *  @b Description
 *  @n  
 *      Task which executes the IPv4 Reassembly tests. Packets are generated by
 *      the test application (using the test vectors) and pushed to the NETCP
 *
 *  @retval
 *      0   - Success
 *  @retval
 *      <0  - Error
 */
void Test_ipv4ReassemblyTask(UArg arg0, UArg arg1)
{
    Qmss_QueueHnd           destQueue;
    uint8_t                 isAllocated;
    int32_t                 retVal;

    /* Open a General Purpose Queue for the UDP/FP Data */
    destQueue = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (destQueue == NULL)
    {
        printf ("Error: Unable to open the Fast Path Rx Queue\n");
        return;
    }

    /* Create the fast paths for the fragmentation tests */
    retVal = Test_setupFastPaths (TEST_NUM_TRAFFIC_FLOW);
    if (retVal < 0)
        return;

    /* Create the sockets for the fragmentation tests */
    retVal = Test_setupSockets (destQueue, TEST_NUM_TRAFFIC_FLOW, TEST_NUM_CHANNEL_PER_TF);
    if (retVal < 0)
        return;

    /* Launch the IPv4 Reassembly Test: */
    if (Test_reassemblyIPv4 (destQueue) < 0)
    {
        System_printf ("Error: IPv4 Reassembly Test failed\n");
        return;
    }
    System_printf ("Debug: IPv4 Reassembly Test passed\n");

    /* Launch the IPv4 Missing Duplicate Test: */
    if (Test_reassemblyIPv4MissingDuplicate (destQueue) < 0)
    {
        System_printf ("Error: IPv4 Reassembly Test for missing/duplicate fragments failed\n");
        return;
    }
    System_printf ("Debug: IPv4 Reassembly Test for missing/duplicate fragments passed\n");
    return;
}
