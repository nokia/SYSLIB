/**
 *   @file  main.c
 *
 *   @brief
 *      PKTLIB ARM Unit Test code.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013 Texas Instruments, Inc.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

/* Device specific dependencies. */
#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined(DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#elif defined(DEVICE_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#else
#error "Error: Unsupported Device"
#endif

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 *********************** Unit Test Global Definitions *****************
 **********************************************************************/

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global variable for the database */
Name_DBHandle           globalNameDatabaseHandle;

/* Global PKTLIB instance handle */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global heap: */
Pktlib_HeapHandle       myHeap;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
	0,    /* Number of CPINTC Output  requested                               */
	0,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
	0,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                    Linking RAM,                           Num,     Size */
		{ "ARM-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  4096,    128},
		{ "ARM-DDR3-1", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  512,     128},
    }
};

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

/* RESMGR: */
extern void* Resmgr_osalMalloc(Resmgr_MallocMode , uint32_t );
extern void  Resmgr_osalFree(Resmgr_MallocMode , void* , uint32_t );
extern void* Resmgr_osalMallocMemoryRegion(char*, Resmgr_MemRegionType , uint32_t );
extern void  Resmgr_osalFreeMemoryRegion(char*, Resmgr_MemRegionType , void* , uint32_t );
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);

/* PKTLIB: */
extern void* Pktlib_osalMalloc(Pktlib_MallocMode, uint32_t);
extern void  Pktlib_osalFree(Pktlib_MallocMode, void*, uint32_t);
extern void  Pktlib_osalBeginMemAccess(void*, uint32_t);
extern void  Pktlib_osalEndMemAccess(void*, uint32_t);
extern void  Pktlib_osalBeginPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void  Pktlib_osalEndPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void* Pktlib_osalEnterCS(Pktlib_HeapHandle);
extern void  Pktlib_osalExitCS(Pktlib_HeapHandle, void*);
extern void* Pktlib_osalPhyToVirt(void* ptrPhysicalAddress);

/**********************************************************************
 ************************* PKTLIB Unit Test ***************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory to be allocated
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* myMalloc(uint32_t size, uint32_t arg)
{
    uint8_t*    ptr;

    /* Allocate memory from the HPLIB pools. */
    ptr = (uint8_t *)hplib_vmMemAlloc (size, 0, 0);
    if (ptr == NULL)
        return NULL;
    return ptr;
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void myFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Dummy IP send which is used to simulate the behavior when a descriptor is actually
 *      send to an IP block. The descriptor is recycled as per the CPPI specification using
 *      the specified return queue. This API simply simulates this.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *
 *  @retval
 *      Not Applicable
 */
static void dummy_ip_send(Ti_Pkt* ptrPkt)
{
    Qmss_Queue      returnQueueInfo;
    Qmss_QueueHnd   returnQueueHnd;
    Ti_Pkt*         ptrNextPacket;

    while (ptrPkt != NULL)
    {
        /* Get the link to the next packet. */
        ptrNextPacket = Pktlib_getNextPacket(ptrPkt);

        /* Get the return queue information to where the packet has to be pushed back into. */
        returnQueueInfo = Cppi_getReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt);

        /* Get the return queue */
        returnQueueHnd = Qmss_getQueueHandle(returnQueueInfo);

        /* Now push the descriptor back into the return queue. */
        Qmss_queuePushDesc (returnQueueHnd, (void*)ptrPkt);

        /* Once the packet has been pushed all the fields in the packet have been converted to physical
         * address. Since this is a simulation of the hardware behavior; we hack around the descriptor
         * and convert it back to a virtual address. At this point in time any address in the descriptor
         * has been converted to a physical address. */
        if (ptrNextPacket != NULL)
            ptrNextPacket = Osal_qmssConvertDescPhyToVirt (0, Pktlib_getNextPacket(ptrPkt));

        /* Get the next packet in the chain. */
        ptrPkt = ptrNextPacket;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is a utility API which is used to create packets
 *      using the specified parameters.
 *
 *  @param[in]  numPackets
 *      The number of packets which are to be created and chained together
 *  @param[in]  dataBufferSize
 *      Array of data buffer sizes which are to be attached to the packets
 *
 *  @retval
 *      Success     -   Pointer to the head of the packet.
 *  @retval
 *      Error       -   NULL
 */
static Ti_Pkt* create_packets
(
    uint32_t        numPackets,
    uint32_t        dataBufferSize[]
)
{
    Ti_Pkt*     ptrHeadPkt;
    Ti_Pkt*     ptrPkt;
    uint16_t    index;

    /* If there are no packets to create we are done. */
    if (numPackets == 0)
        return NULL;

    /* Allocate the head packet. */
    ptrHeadPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, dataBufferSize[0]);
    if (ptrHeadPkt == NULL)
        return NULL;

    /* Cycle through the rest of the packets. */
    for (index = 1; index < numPackets; index++)
    {
        /* Get the next packet. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, dataBufferSize[index]);
        if (ptrPkt == NULL)
            return NULL;

        /* Now we merge the packets together. */
        if (Pktlib_packetMerge(appPktlibInstanceHandle, ptrHeadPkt, ptrPkt, NULL) == NULL)
            return NULL;
    }

    /* Return the head of the packet. */
    return ptrHeadPkt;
}

/**
 *  @b Description
 *  @n
 *      The function test the packet library merge API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibraryMerge(void)
{
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    Ti_Pkt*             ptrMergedPkt;
    uint16_t            index;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataLen;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Merge API Test\n");

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Allocate a packet of 100 bytes. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Get the associated data buffer. */
    Pktlib_getDataBuffer(ptrPkt1, &ptrDataBuffer, &dataLen);

    /* Populate the data buffer with test data. */
    for (index = 0; index < 32; index++)
        *(ptrDataBuffer + index) = 0x11;

    /* Setup the packet & data buffer length appropriately. */
    Pktlib_setDataBufferLen(ptrPkt1, 32);
    Pktlib_setPacketLen(ptrPkt1, 32);

    /* Allocate another packet of 50 bytes. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 50);
    if (ptrPkt2 == NULL)
        return -1;

    /* Get the associated data buffer. */
    Pktlib_getDataBuffer(ptrPkt2, &ptrDataBuffer, &dataLen);

    /* Populate the data buffer with test data. */
    for (index = 0; index < 16; index++)
        *(ptrDataBuffer + index) = 0x22;

    /* Setup the packet & data buffer length appropriately. */
    Pktlib_setDataBufferLen(ptrPkt2, 16);
    Pktlib_setPacketLen(ptrPkt2, 16);

    /*********************************************************************************
     * TEST1: Merge the packet
     *  - Validate and ensure that the merged packet is correctly setup.
     *********************************************************************************/

    /* Now we merge the packets together */
    ptrMergedPkt = Pktlib_packetMerge(appPktlibInstanceHandle, ptrPkt1, ptrPkt2, NULL);
    if (ptrMergedPkt == NULL)
        return -1;

    /* Validate: There should be 2 buffers after merging the packets. */
    if (Pktlib_packetBufferCount(ptrMergedPkt) != 2)
        return -1;

    /* The total packet length of the merged packet should be 32 + 16 i.e. 48 */
    if (Pktlib_getPacketLen(ptrMergedPkt) != 48)
        return -1;

    /* The data buffer of the first merged packet should be 32 bytes and should have the 0x11 test
     * pattern. */
    Pktlib_getDataBuffer(ptrMergedPkt, &ptrDataBuffer, &dataLen);
    if (dataLen != 32)
        return -1;
    for (index = 0; index < dataLen; index++)
        if (*(ptrDataBuffer + index) != 0x11)
            return -1;

    /* The data buffer of the second merged packet should be 16 bytes and should have the 0x22 test
     * pattern. */
    Pktlib_getDataBuffer(Pktlib_getNextPacket(ptrMergedPkt), &ptrDataBuffer, &dataLen);
    if (dataLen != 16)
        return -1;
    for (index = 0; index < dataLen; index++)
        if (*(ptrDataBuffer + index) != 0x22)
            return -1;

    /* Cleanup the merged packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrMergedPkt);

    /*********************************************************************************
     * TEST2: Heap Statistics.
     *  - Ensure that the heap stats are correct
     *********************************************************************************/

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Validation: We should not have any packets in the garbage queue; merging packets should
     * not have any side-effects. */
    if ((startStats.numPacketsinGarbage != 0) || (endStats.numPacketsinGarbage != 0))
        return -1;

    /* Validation: Ensure that the number of data packet and zero data packets are the same at
     * the beginning & end of the test */
    if ((endStats.numFreeDataPackets   != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Test Passed. */
    printf ("Debug: Merge API Test API Passed\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function test the packet library clone API
 *
 *  @param[in]  freeQueueHnd
 *      Free Queue which has descriptors attached with buffers.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibraryClone()
{
    Ti_Pkt*     ptrOrigPkt;
    Ti_Pkt*     ptrClonePkt1;
    Ti_Pkt*     ptrClonePkt2;
    Ti_Pkt*     ptrTempPkt;
    uint16_t    index;
    uint32_t    sizeofOrgPacket = 0;
    uint32_t    numPackets;
    uint32_t    dataBufferSize[10];
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;

    /* Configurable Parameters: These are the parameters which can be modified by
     * to cause different behavior of the test code.
     *  Parameter 1: numPackets
     *      - This is the number of packets which are created and chained together
     *        to create the orignal packet which is to be cloned. */
    numPackets = 3;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Cloning Test starting with %d packet in a chain\n", numPackets);

    /* Cycle through and create all the packets. */
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 10 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Quick validation to ensure that the packets are correctly created. */
    if (Pktlib_getPacketLen(ptrOrigPkt) != sizeofOrgPacket)
        return -1;
    if (Pktlib_packetBufferCount(ptrOrigPkt) != numPackets)
        return -1;

    /* Cycle through and make sure that the orignal is correctly setup. */
    ptrTempPkt = ptrOrigPkt;
    index      = 0;
    while (ptrTempPkt != NULL)
    {
        uint8_t*  ptrDataBuffer;
        uint32_t  dataLen;

        /* Get the data buffer and length. */
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataLen);

        /* Check the data length also to ensure that this is what we had configured. */
        if (dataLen != (10 + index))
            return -1;

        /* Goto the next packet. */
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
        index++;
    }

    /******************************************************************************
     * TEST: Create a CLONE from the ORIGNAL
     *  - Ensure and validate that the ORIGNAL & CLONE are correct
     ******************************************************************************/
    printf ("Debug: Testing cloning from the original.\n");

    /* Create bufferless packets for cloning. */
    memset ((void *)&dataBufferSize, 0, sizeof(dataBufferSize));
    ptrClonePkt1 = create_packets(numPackets, dataBufferSize);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Clone the packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Validate the clone and make sure that the packet is correct. */
    if (Pktlib_getPacketLen(ptrOrigPkt) != Pktlib_getPacketLen(ptrClonePkt1))
        return -1;
    if (Pktlib_packetBufferCount(ptrOrigPkt) != Pktlib_packetBufferCount(ptrClonePkt1))
        return -1;

    /* We go further one step and iterate through all the packets in the clone and make sure they match
     * perfectly with what we expect */
    ptrTempPkt = ptrClonePkt1;
    index      = 0;
    while (ptrTempPkt != NULL)
    {
        uint8_t*  ptrDataBuffer;
        uint32_t  dataLen;

        /* Get the data buffer and length. */
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataLen);

        /* Check the data length also to ensure that this is what we had configured. */
        if (dataLen != (10 + index))
            return -1;

        /* Goto the next packet. */
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
        index++;
    }

    /******************************************************************************
     * TEST: Create a CLONE from the CLONE
     *  - Ensure and validate that the ORIGNAL & CLONE are correct
     ******************************************************************************/
    printf ("Debug: Testing cloning from the clone.\n");

    /* Create another set of bufferless packets for cloning but now we use the previous clone
     * to create another clone.  */
    ptrClonePkt2 = create_packets(numPackets, dataBufferSize);
    if (ptrClonePkt2 == NULL)
        return -1;

    /* Create another clone using the cloned buffer. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrClonePkt1, ptrClonePkt2) < 0)
        return -1;

    /* Validate the clone and make sure that the packet is correct. */
    if (Pktlib_getPacketLen(ptrOrigPkt) != Pktlib_getPacketLen(ptrClonePkt2))
        return -1;
    if (Pktlib_packetBufferCount(ptrOrigPkt) != Pktlib_packetBufferCount(ptrClonePkt2))
        return -1;

    /* We go further one step and iterate through all the packets in the clone and make sure they match
     * perfectly with what we expect */
    ptrTempPkt = ptrClonePkt2;
    index      = 0;
    while (ptrTempPkt != NULL)
    {
        uint8_t*  ptrDataBuffer;
        uint32_t  dataLen;

        /* Get the data buffer and length. */
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataLen);

        /* Check the data length also to ensure that this is what we had configured. */
        if (dataLen != (10 + index))
            return -1;

        /* Goto the next packet. */
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
        index++;
    }

    /******************************************************************************
     * TEST: Heap Statistics and checking for memory leaks.
     ******************************************************************************/
    printf ("Debug: Cleaning clones memory leak test\n");

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Validation: Before proceeding with the cleanup we make sure there are no packets in the
     * Garbage queue. */
    if (endStats.numPacketsinGarbage != 0)
        return -1;

    /* Cleanup the original packet */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Cleanup the second cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrClonePkt2);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrClonePkt1);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge an original packet after the clone (With CPDMA Simulation)
     ******************************************************************************/
    printf ("Debug: Merge an original packet after a cloned Packet (with CPDMA simulation).\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Now clean up the original packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Merge the Temporary Original packet to the end of the Clone packet. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrClonePkt1, ptrTempPkt, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrClonePkt1);

    /* Dummy IP Send. */
    dummy_ip_send(ptrClonePkt1);

    /* Execute the Garbage Collection. */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge an original packet after the clone with a software free
     ******************************************************************************/
    printf ("Debug: Merge an original packet after a cloned Packet (with software free)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Merge the Temporary Original packet to the end of the Clone packet. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrClonePkt1, ptrTempPkt, NULL);

    /* Free the chained packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrClonePkt1);

    /* Now clean up the original packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge a cloned packet after a cloned packet (with CPDMA Simulation)
     ******************************************************************************/
    printf ("Debug: Merge a cloned packet after a cloned packet (with CPDMA simulation)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt2 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt2) < 0)
        return -1;

    /* Merge the cloned packets */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrClonePkt1, ptrClonePkt2, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrClonePkt1);

    /* Dummy IP Send. */
    dummy_ip_send(ptrClonePkt1);

    /* Now clean up the original packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);
    Pktlib_freePacket(appPktlibInstanceHandle, ptrTempPkt);

    /* Execute the Garbage Collection. */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge a cloned packet after a cloned packet (with software free)
     ******************************************************************************/
    printf ("Debug: Merge a cloned packet after a cloned packet (with software free)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt2 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt2) < 0)
        return -1;

    /* Merge the Temporary Original packet to the end of the Original (referenced) packet. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrClonePkt1, ptrClonePkt2, NULL);

    /* Clean up the chained packet of clones. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrClonePkt1);

    /* Clean the original packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);
    Pktlib_freePacket(appPktlibInstanceHandle, ptrTempPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge a cloned packet after an original packet (with CPDMA simulation)
     ******************************************************************************/
    printf ("Debug: Merge a cloned packet after an original packet (with CPDMA simulation)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Merge the cloned packet after the original temporary packet. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt1, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrClonePkt1);

    /* Dummy IP Send. */
    dummy_ip_send(ptrTempPkt);

    /* Now clean up the original packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Execute the Garbage Collection. */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge a cloned packet after an original packet (with software free)
     ******************************************************************************/
    printf ("Debug: Merge a cloned packet after an original packet (with software free)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Allocate a bufferless packet for cloning. */
    ptrClonePkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrClonePkt1 == NULL)
        return -1;

    /* Now clone the original packet. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt1) < 0)
        return -1;

    /* Merge the cloned packet after the original temporary packet. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt1, NULL);

    /* Now clean up the original packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Now clean up the chained packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrTempPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /*************************************************************************************
     * TEST: Merge an original packet after an original packet (with CPDMA simulation)
     *************************************************************************************/
    printf ("Debug: Merge an original packet after an original packet (with CPDMA simulation)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Merge the original packets together. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrOrigPkt, ptrTempPkt, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrClonePkt1);

    /* Dummy IP Send. */
    dummy_ip_send(ptrOrigPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Merge an original packet after an original packet (with software free)
     ******************************************************************************/
    printf ("Debug: Merge an original packet after an original packet (with software free)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Merge the original packets together. */
    Pktlib_packetMerge (appPktlibInstanceHandle, ptrOrigPkt, ptrTempPkt, NULL);

    /* Now clean up the chained packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Debug Message: */
    printf ("Debug: All cloning tests passed.\n");

    /* Packet Clone Test was successful. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function test the packet library split API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibrarySplit(void)
{
    Ti_Pkt*             ptrOrigPkt;
    Ti_Pkt*             ptrSplitPkt;
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    Ti_Pkt*             ptrTempPkt;
    Ti_Pkt*             splitPacketArray[200];
    uint16_t            index;
    int32_t             retVal;
    uint8_t*            ptrDataBuffer;
    uint32_t            splitSize = 0;
    uint32_t	        middleSplit;
    uint32_t            dataBufferLen;
    uint32_t            packetLength;
    uint32_t            sizeofOrgPacket = 0;
    uint32_t            dataBufferSize[10];
    uint32_t            numPackets;
    uint32_t            perfectSplitPacket;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Configurable Parameters: These are the parameters which can be modified by
     * to cause different behavior of the test code.
     *  Parameter 1: numPackets
     *      - This is the number of packets which are created and chained together
     *        to create the orignal packet which is to be split.
     *  Parameter 2: middleSplit
     *      - This is to test the split in the middle of the chained packets.
     *  Parameter 3: perfectSplitPacket
     *      - This is to test the case where the split occurs at a perfect chained
     *        packet boundary.
     *        */
    numPackets         = 5;
    perfectSplitPacket = 3;
    middleSplit		   = 3;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 1: Split API Testing
     * - Split occurs in the middle of first packet.
     ******************************************************************************/
    printf ("Debug: Testing split in the middle of the first packet\n");

    /* Cycle through and create all the packets. */
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* To create the split in the middle we give the split size within the first packet itself */
    splitSize = 5;

    /* Allocate a new zero buffer packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split. */
    if (Pktlib_splitPacket(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) < 0)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/

    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - Only 1 buffer in the packet
     *  - The buffer length should be 'splitSize'.
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != 1)
        return -1;

    /* Get the data buffer and length and ensure this is correct. */
    Pktlib_getDataBuffer(ptrPkt1, &ptrDataBuffer, &dataBufferLen);
    if (dataBufferLen != splitSize)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - We would have the same number of buffers as the orignal.
     *  - The buffer length of the first buffer in the packet should be 'splitSize'.
     *  - The buffer address should be offset by the split size. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != numPackets)
        return -1;

    /* Get the data buffer and length and ensure this is correct. */
    Pktlib_getDataBuffer(ptrPkt2, &ptrDataBuffer, &dataBufferLen);
    if (dataBufferLen != dataBufferSize[0] - splitSize)
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Validation: We have cleaned both the packets and so there should be no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Ensure there are no memory leaks. */
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 2: Split API Testing
     * - Split occurs in the middle of the 3rd Packet.
     ******************************************************************************/
    printf ("Debug: Testing split in the middle of the %d packet\n", middleSplit);

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* To create the split in the middle of the nth packet. */
    splitSize = 0;
    for (index = 0; index < middleSplit; index++)
    	splitSize = splitSize + dataBufferSize[index];
    splitSize = splitSize + 3;

    /* Allocate a new zero buffer packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split. */
    if (Pktlib_splitPacket(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) < 0)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/

    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - There should be 'middleSplit' buffers in the packet
     *  - The buffer length should be 'splitSize'.
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != (middleSplit + 1))
        return -1;

    /* Cycle through all the linked packets and ensure that the packet lengths and
     * data buffer lengths are synched up. */
    packetLength = 0;
    ptrTempPkt   = ptrPkt1;
    while (ptrTempPkt != NULL)
    {
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataBufferLen);
        packetLength = packetLength + dataBufferLen;
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
    }
    if (packetLength != splitSize)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - We would have all the remaining packets after the middleSplit.
     *  - The buffer length of the first buffer in the packet should be 'splitSize'.
     *  - The buffer address should be offset by the split size. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != (numPackets - middleSplit))
        return -1;

    /* Cycle through all the linked packets and ensure that the packet lengths and
     * data buffer lengths are synched up. */
    packetLength = 0;
    ptrTempPkt   = ptrPkt2;
    while (ptrTempPkt != NULL)
    {
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataBufferLen);
        packetLength = packetLength + dataBufferLen;
        ptrTempPkt   = Pktlib_getNextPacket(ptrTempPkt);
    }
    if (packetLength != (sizeofOrgPacket - splitSize))
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /******************************************************************************
     * TEST 3: Split API Testing
     * - Split occurs right at the end of the packet boundary.
     ******************************************************************************/
    printf ("Debug: Testing split at the end of the %d packet\n", perfectSplitPacket);

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Setup the split size correctly. */
    splitSize = 0;
    for (index = 0; index < perfectSplitPacket; index++)
        splitSize = splitSize + dataBufferSize[index];

    /* Get a new packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split: Since this split is exactly at the end of the second packet the
     * spliPkt will not be used and so the API should return 1 to indicate so. */
    if (Pktlib_splitPacket(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) != 1)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/
    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - Only 2 buffer in the packet
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != perfectSplitPacket)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - Remaining buffers in the packet
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != (numPackets - perfectSplitPacket))
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Cleanup the memory for the split packet (since this was not used up) */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrSplitPkt);

    /******************************************************************************
     * TEST 4: Split API Testing
     * - Multiple splits done on the packet.
     ******************************************************************************/
    printf ("Debug: Testing 10 1 byte splits in the packet.\n");

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be split. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Initialize the split packet tracker */
    memset ((void *)&splitPacketArray[0], 0, sizeof(splitPacketArray));

    /* We now create multiple splits in the packet. */
    for (index = 0; index < 10; index++)
    {
        /* Get a new packet for the split. */
        ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
        if (ptrSplitPkt == NULL)
            return -1;

        /* Do the packet split: We split the packet into 1 byte each. */
        retVal = Pktlib_splitPacket(appPktlibInstanceHandle,  ptrOrigPkt, ptrSplitPkt, 1, &ptrPkt1, &ptrPkt2);

        /* Check if there is an error. */
        if (retVal < 0)
            return -1;

        /* If the packet was not used we can clean it up immediately. */
        if (retVal == 1)
            Pktlib_freePacket(appPktlibInstanceHandle, ptrSplitPkt);

        /* After the split is complete.
         *  ptrPkt1 should have 1 byte
         *  ptrPkt2 should have the rest of the data. */
        if (Pktlib_getPacketLen(ptrPkt1) != 1)
            return -1;

        /* Keep track of the split packet. */
        splitPacketArray[index] = ptrPkt1;

        /* Make sure the orignal packet is now the new modified packet. */
        ptrOrigPkt = ptrPkt2;
    }

    /* At the end of it we need to clean all the packets. */
    for (index = 0; index < 10; index++)
    {
        if (splitPacketArray[index] == 0)
            return -1;

        /* Clean the packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, splitPacketArray[index]);
    }

    /* Clean the orignal packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Get the heap stats */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Ensure there are no memory leaks. */
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Debug Message: Split API testing was successful. */
    printf ("Debug: Split API test was successful.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function test the packet library split API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibrarySplit2(void)
{
    Ti_Pkt*             ptrOrigPkt;
    Ti_Pkt*             ptrSplitPkt;
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    Ti_Pkt*             ptrTempPkt;
    Ti_Pkt*             splitPacketArray[200];
    uint16_t            index;
    int32_t             retVal;
    uint8_t*            ptrDataBuffer;
    uint32_t            splitSize = 0;
    uint32_t	        middleSplit;
    uint32_t            dataBufferLen;
    uint32_t            packetLength;
    uint32_t            sizeofOrgPacket = 0;
    uint32_t            dataBufferSize[10];
    uint32_t            numPackets;
    uint32_t            perfectSplitPacket;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Configurable Parameters: These are the parameters which can be modified by
     * to cause different behavior of the test code.
     *  Parameter 1: numPackets
     *      - This is the number of packets which are created and chained together
     *        to create the orignal packet which is to be split.
     *  Parameter 2: middleSplit
     *      - This is to test the split in the middle of the chained packets.
     *  Parameter 3: perfectSplitPacket
     *      - This is to test the case where the split occurs at a perfect chained
     *        packet boundary.
     *        */
    numPackets         = 5;
    perfectSplitPacket = 2;
    middleSplit		   = 4;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 1: Split API Testing
     * - Split occurs in the middle of first packet.
     ******************************************************************************/
    printf ("Debug: Testing split in the middle of the first packet\n");

    /* Cycle through and create all the packets. */
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* To create the split in the middle we give the split size within the first packet itself */
    splitSize = 5;

    /* Allocate a new zero buffer packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split. */
    if (Pktlib_splitPacket2(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) < 0)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/

    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - Only 1 buffer in the packet
     *  - The buffer length should be 'splitSize'.
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != 1)
        return -1;

    /* Get the data buffer and length and ensure this is correct. */
    Pktlib_getDataBuffer(ptrPkt1, &ptrDataBuffer, &dataBufferLen);
    if (dataBufferLen != splitSize)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - We would have the same number of buffers as the orignal.
     *  - The buffer length of the first buffer in the packet should be 'splitSize'.
     *  - The buffer address should be offset by the split size. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != numPackets)
        return -1;

    /* Get the data buffer and length and ensure this is correct. */
    Pktlib_getDataBuffer(ptrPkt2, &ptrDataBuffer, &dataBufferLen);
    if (dataBufferLen != dataBufferSize[0] - splitSize)
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Validation: We have cleaned both the packets and so there should be no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Ensure there are no memory leaks. */
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 2: Split API Testing
     * - Split occurs in the middle of the nth Packet.
     ******************************************************************************/
    printf ("Debug: Testing split in the middle of the %d packet\n", middleSplit);

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* To create the split in the middle of the nth packet. */
    splitSize = 0;
    for (index = 0; index < middleSplit; index++)
    	splitSize = splitSize + dataBufferSize[index];
    splitSize = splitSize + 3;

    /* Allocate a new zero buffer packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split. */
    if (Pktlib_splitPacket2(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) < 0)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/

    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - There should be 'middleSplit' buffers in the packet
     *  - The buffer length should be 'splitSize'.
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != (middleSplit + 1))
        return -1;

    /* Cycle through all the linked packets and ensure that the packet lengths and
     * data buffer lengths are synched up. */
    packetLength = 0;
    ptrTempPkt   = ptrPkt1;
    while (ptrTempPkt != NULL)
    {
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataBufferLen);
        packetLength = packetLength + dataBufferLen;
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
    }
    if (packetLength != splitSize)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - We would have all the remaining packets after the middleSplit.
     *  - The buffer length of the first buffer in the packet should be 'splitSize'.
     *  - The buffer address should be offset by the split size. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != (numPackets - middleSplit))
        return -1;

    /* Cycle through all the linked packets and ensure that the packet lengths and
     * data buffer lengths are synched up. */
    packetLength = 0;
    ptrTempPkt   = ptrPkt2;
    while (ptrTempPkt != NULL)
    {
        Pktlib_getDataBuffer(ptrTempPkt, &ptrDataBuffer, &dataBufferLen);
        packetLength = packetLength + dataBufferLen;
        ptrTempPkt   = Pktlib_getNextPacket(ptrTempPkt);
    }
    if (packetLength != (sizeofOrgPacket - splitSize))
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /******************************************************************************
     * TEST 3: Split API Testing
     * - Split occurs right at the end of the packet boundary.
     ******************************************************************************/
    printf ("Debug: Testing split at the end of the %d packet\n", perfectSplitPacket);

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Setup the split size correctly. */
    splitSize = 0;
    for (index = 0; index < perfectSplitPacket; index++)
        splitSize = splitSize + dataBufferSize[index];

    /* Get a new packet for the split. */
    ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrSplitPkt == NULL)
        return -1;

    /* Do the packet split: Since this split is exactly at the end of the second packet the
     * spliPkt will not be used and so the API should return 1 to indicate so. */
    if (Pktlib_splitPacket2(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, splitSize, &ptrPkt1, &ptrPkt2) != 1)
        return -1;

    /*******************************************************************************
     **************************** VALIDATE API RESULTS *****************************
     *******************************************************************************/

    /* Validate the API results for the first returned packet:
     *  - Packet Length should be 'splitSize' bytes
     *  - Only 2 buffer in the packet
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt1) != splitSize)
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt1) != perfectSplitPacket)
        return -1;

    /* Validate the API results for the second returned packet:
     *  - Packet Length should be 'splitSize' bytes less than the 'sizeofOrgPacket'
     *  - Remaining buffers in the packet
     *  - The data buffer should be what was passed during initialization. */
    if (Pktlib_getPacketLen(ptrPkt2) != (sizeofOrgPacket - splitSize))
        return -1;
    if (Pktlib_packetBufferCount(ptrPkt2) != (numPackets - perfectSplitPacket))
        return -1;

    /* Cleanup the split packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Cleanup the first cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Cleanup the memory for the split packet (since this was not used up) */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrSplitPkt);

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 4: Split API Testing
     * - Multiple splits done on the packet.
     ******************************************************************************/
    printf ("Debug: Testing 10 1 byte splits in the packet.\n");

    /* Cycle through and create all the packets. */
    sizeofOrgPacket = 0;
    for (index = 0; index < numPackets; index++)
    {
        /* Configure the data size */
        dataBufferSize[index]  = 20 + index;

        /* Keep track of the size of the orignal packet. */
        sizeofOrgPacket = sizeofOrgPacket + dataBufferSize[index];
    }

    /* Create the orignal packet which is to be split. */
    ptrOrigPkt = create_packets(numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Initialize the split packet tracker */
    memset ((void *)&splitPacketArray[0], 0, sizeof(splitPacketArray));

    /* We now create multiple splits in the packet. */
    for (index = 0; index < 10; index++)
    {
        /* Get a new packet for the split. */
        ptrSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
        if (ptrSplitPkt == NULL)
            return -1;

        /* Do the packet split: We split the packet into 1 byte each. */
        retVal = Pktlib_splitPacket2(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, 1, &ptrPkt1, &ptrPkt2);

        /* Check if there is an error. */
        if (retVal < 0)
            return -1;

        /* If the packet was not used we can clean it up immediately. */
        if (retVal == 1)
            Pktlib_freePacket(appPktlibInstanceHandle, ptrSplitPkt);

        /* After the split is complete.
         *  ptrPkt1 should have 1 byte
         *  ptrPkt2 should have the rest of the data. */
        if (Pktlib_getPacketLen(ptrPkt1) != 1)
            return -1;

        /* Keep track of the split packet. */
        splitPacketArray[index] = ptrPkt1;

        /* Make sure the orignal packet is now the new modified packet. */
        ptrOrigPkt = ptrPkt2;
    }

    /* At the end of it we need to clean all the packets. */
    for (index = 0; index < 10; index++)
    {
        if (splitPacketArray[index] == 0)
            return -1;

        /* Clean the packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, splitPacketArray[index]);
    }

    /* Clean the orignal packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Get the heap stats */
    Pktlib_getHeapStats(myHeap, &endStats);

    /* Ensure there are no memory leaks. */
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Debug Message: Split API testing was successful. */
    printf ("Debug: Split2 API test was successful.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the super heaps.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t test_pktLibSuperHeaps(void)
{
    Pktlib_HeapHandle   superHeapHandle;
    int32_t             errCode;
    Ti_Pkt*             ptrPkt;
    uint32_t            index;
    uint32_t            memberHeapInTest;
    uint32_t            packetSize;
    Pktlib_HeapHandle   memberHeaps[3];
    Pktlib_HeapStats    startStats[3];
    Pktlib_HeapStats    endStats[3];
    uint32_t            numMemberHeaps = 3;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Testing Super Heap \n");

    /* Find all the heaps which have been created till now */
    memberHeaps[0] = Pktlib_findHeapByName(appPktlibInstanceHandle, "My Test Heap", &errCode);
    if (memberHeaps[0] == NULL)
        return -1;
    memberHeaps[1] = Pktlib_findHeapByName(appPktlibInstanceHandle, "MyHeap1", &errCode);
    if (memberHeaps[1] == NULL)
        return -1;
    memberHeaps[2] = Pktlib_findHeapByName(appPktlibInstanceHandle, "MyHeap2", &errCode);
    if (memberHeaps[2] == NULL)
        return -1;

    /* Debug Message: */
    printf ("Debug: 'My Test Heap' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[0]));
    printf ("Debug: 'MyHeap1' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[1]));
    printf ("Debug: 'MyHeap2' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[2]));

    /* Create the super heap */
    superHeapHandle = Pktlib_createSuperHeap(appPktlibInstanceHandle, "SuperHeap", memberHeaps, 3, &errCode);
    if (superHeapHandle == NULL)
    {
        printf ("Error: Unable to create super heap error code: %d\n", errCode);
        return -1;
    }
    printf ("Debug: Super Heap has been created with handle 0x%p\n", superHeapHandle);

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(memberHeaps[0], &startStats[0]);
    Pktlib_getHeapStats(memberHeaps[1], &startStats[1]);
    Pktlib_getHeapStats(memberHeaps[2], &startStats[2]);

    /* Check: If we can find the Super Heap. */
    if (Pktlib_findHeapByName(appPktlibInstanceHandle, "SuperHeap", &errCode) != superHeapHandle)
    {
        printf ("Error: Super Heap could NOT be found\n");
        return -1;
    }

    /******************************************************************************
     * TEST 1:
     *  This test does an allocation with different packet sizes & verifies that
     *  the correct heap is used.
     *
     *  Starting from the heap which has the smallest size and moving to the heap
     *  with the largest size all possible combinations are tested to ensure that
     *  only the 'best sized' heap is touched.
     *
     *  The code is organized as such so that we can verify & test out which heap
     *  we are hitting.
     ******************************************************************************/
    printf ("Debug: Use Case 1 --> Different Packet Size allocation check\n");

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(memberHeaps[0], &startStats[0]);
    Pktlib_getHeapStats(memberHeaps[1], &startStats[1]);
    Pktlib_getHeapStats(memberHeaps[2], &startStats[2]);

    /* Set the index to the heap being tested: We know that the 'MyHeap1' is the heap
     * with the smallest data buffer size. This is added to the Super Heap as Member
     * Heap 1. */
    memberHeapInTest = 1;

    /* Debug Message: */
    printf ("Debug: Testing Packet Size allocations from 1 to %d bytes\n",
                   Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /* Cycle through all possible heap sizes and make sure the allocations are done properly. */
    for (packetSize = 1; packetSize <= Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]); packetSize++)
    {
        /* Allocate a packet from the Super Heap */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, packetSize);
        if (ptrPkt == NULL)
        {
            printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
            return -1;
        }

        /* The packet should only have been allocated from the member heap under test.
         * No other heaps should get touched. So here we take a snapshot of all the member
         * heaps. */
        Pktlib_getHeapStats(memberHeaps[0], &endStats[0]);
        Pktlib_getHeapStats(memberHeaps[1], &endStats[1]);
        Pktlib_getHeapStats(memberHeaps[2], &endStats[2]);

        /* One data buffer packet should have been removed from the heap */
        if (((endStats[memberHeapInTest].numFreeDataPackets + 1)  != startStats[memberHeapInTest].numFreeDataPackets)   ||
             (endStats[memberHeapInTest].numPacketsinGarbage      != startStats[memberHeapInTest].numPacketsinGarbage)  ||
             (endStats[memberHeapInTest].numZeroBufferPackets     != startStats[memberHeapInTest].numZeroBufferPackets))
        {
            printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
            return -1;
        }

        /* Other heaps should be untouched. */
        for (index = 0; index < numMemberHeaps; index++)
        {
            if (index != memberHeapInTest)
            {
                /* Check with the initial value and ensure sanity */
                if ((endStats[index].numPacketsinGarbage  != 0)                                     ||
                    (endStats[index].numFreeDataPackets   != startStats[index].numFreeDataPackets)  ||
                    (endStats[index].numZeroBufferPackets != startStats[index].numZeroBufferPackets))
                {
                    printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
                    return -1;
                }
            }
        }
        /* Clean up the allocated packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }

    /* Set the index to the heap being tested: We know that the 'MyHeap2' is the heap
     * with the next smallest data buffer size. This is added to the Super Heap as Member
     * Heap 2. */
    memberHeapInTest = 2;

    /* Debug Message: */
    printf ("Debug: Testing Packet Size allocations from %d to %d bytes\n",
                   Pktlib_getMaxBufferSize(memberHeaps[1]) + 1,
                   Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /* Cycle through all possible heap sizes and make sure the allocations are done properly.
     * NOTE: The smallest heap 'MyHeap1' has already been verified so we skip its packet size */
    for (packetSize = Pktlib_getMaxBufferSize(memberHeaps[1]) + 1;
         packetSize <= Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]);
         packetSize++)
    {
        /* Allocate a packet from the Super Heap */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, packetSize);
        if (ptrPkt == NULL)
        {
            printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
            return -1;
        }

        /* The packet should only have been allocated from the member heap under test.
         * No other heaps should get touched. So here we take a snapshot of all the member
         * heaps. */
        Pktlib_getHeapStats(memberHeaps[0], &endStats[0]);
        Pktlib_getHeapStats(memberHeaps[1], &endStats[1]);
        Pktlib_getHeapStats(memberHeaps[2], &endStats[2]);

        /* One data buffer packet should have been removed from the heap */
        if (((endStats[memberHeapInTest].numFreeDataPackets + 1)  != startStats[memberHeapInTest].numFreeDataPackets)   ||
             (endStats[memberHeapInTest].numPacketsinGarbage      != startStats[memberHeapInTest].numPacketsinGarbage)  ||
             (endStats[memberHeapInTest].numZeroBufferPackets     != startStats[memberHeapInTest].numZeroBufferPackets))
        {
            printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
            return -1;
        }

        /* Other heaps should be untouched. */
        for (index = 0; index < numMemberHeaps; index++)
        {
            if (index != memberHeapInTest)
            {
                /* Check with the initial value and ensure sanity */
                if ((endStats[index].numPacketsinGarbage  != 0)                                     ||
                    (endStats[index].numFreeDataPackets   != startStats[index].numFreeDataPackets)  ||
                    (endStats[index].numZeroBufferPackets != startStats[index].numZeroBufferPackets))
                {
                    printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
                    return -1;
                }
            }
        }
        /* Clean up the allocated packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }

    /* Set the index to the heap being tested: We know that the 'My Test Heap' is the heap
     * with the largest data buffer size. This is added to the Super Heap as Member
     * Heap 0. */
    memberHeapInTest = 0;

    /* Debug Message: */
    printf ("Debug: Testing Packet Size allocations from %d to %d bytes\n",
                   Pktlib_getMaxBufferSize(memberHeaps[2]) + 1,
                   Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /* Cycle through all possible heap sizes and make sure the allocations are done properly.
     * NOTE: The heap 'MyHeap2' has already been verified so we skip its packet size */
    for (packetSize = Pktlib_getMaxBufferSize(memberHeaps[2]) + 1;
         packetSize <= Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]);
         packetSize++)
    {
        /* Allocate a packet from the Super Heap */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, packetSize);
        if (ptrPkt == NULL)
        {
            printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
            return -1;
        }

        /* The packet should only have been allocated from the member heap under test.
         * No other heaps should get touched. So here we take a snapshot of all the member
         * heaps. */
        Pktlib_getHeapStats(memberHeaps[0], &endStats[0]);
        Pktlib_getHeapStats(memberHeaps[1], &endStats[1]);
        Pktlib_getHeapStats(memberHeaps[2], &endStats[2]);

        /* One data buffer packet should have been removed from the heap */
        if (((endStats[memberHeapInTest].numFreeDataPackets + 1)  != startStats[memberHeapInTest].numFreeDataPackets)   ||
             (endStats[memberHeapInTest].numPacketsinGarbage      != startStats[memberHeapInTest].numPacketsinGarbage)  ||
             (endStats[memberHeapInTest].numZeroBufferPackets     != startStats[memberHeapInTest].numZeroBufferPackets))
        {
            printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
            return -1;
        }

        /* Other heaps should be untouched. */
        for (index = 0; index < numMemberHeaps; index++)
        {
            if (index != memberHeapInTest)
            {
                /* Check with the initial value and ensure sanity */
                if ((endStats[index].numPacketsinGarbage  != 0)                                     ||
                    (endStats[index].numFreeDataPackets   != startStats[index].numFreeDataPackets)  ||
                    (endStats[index].numZeroBufferPackets != startStats[index].numZeroBufferPackets))
                {
                    printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
                    return -1;
                }
            }
        }
        /* Clean up the allocated packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }
    printf ("Debug: Super Heap Packet size allocation (1 to %d) bytes checks passed\n",
                    Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /******************************************************************************
     * TEST 2:
     *  The test does the ZERO Byte packet allocation on super heaps.
     ******************************************************************************/

    /* Allocate a packet from the Super Heap */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, 0);
    if (ptrPkt == NULL)
    {
        printf ("Error: Unable to allocate bufferless packet from super heap\n");
        return -1;
    }

    /* In the super heap the member heaps are always sorted in ascending order of the
     * packet size. In our test setup we know "MyHeap1" is the heap with the smallest
     * data buffer size and so the zero buffer packet will be allocated from it */
    memberHeapInTest = 1;

    /* Get the heap stats for the member heap */
    Pktlib_getHeapStats(memberHeaps[memberHeapInTest], &endStats[memberHeapInTest]);

    /* Verify: */
    if (endStats[memberHeapInTest].numZeroBufferPackets + 1 != startStats[memberHeapInTest].numZeroBufferPackets)
    {
        printf ("Error: Zero buffer packet was NOT allocated from the %d heap\n", memberHeapInTest);
        return -1;
    }

    /* Debug Message: */
    printf ("Debug: Use Case 2 --> Super Heap bufferless packet allocation passed\n");

    /* Cleanup the packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /******************************************************************************
     * TEST 3:
     *  The test does the ZERO Byte packet allocation on super heaps but it makes
     *  sure that once all the zero buffer descriptors in one heap are finished
     *  the next heap is iterated.
     ******************************************************************************/

    /* We start the test on the smallest data buffer heap (MyHeap1) */
    memberHeapInTest = 1;

    /* Get the heap stats for the member heap in test */
    Pktlib_getHeapStats(memberHeaps[memberHeapInTest], &endStats[memberHeapInTest]);

    /* Cycle through all the zero buffer packets */
    for (index = 0; index < endStats[memberHeapInTest].numZeroBufferPackets; index++)
    {
        /* Allocate a zero buffer packet from the Super Heap. */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, 0);
        if (ptrPkt == NULL)
        {
            printf ("Error: Unable to allocate bufferless packet from super heap\n");
            return -1;
        }
    }

    /* Get the heap stats for the member heap */
    Pktlib_getHeapStats(memberHeaps[memberHeapInTest], &endStats[memberHeapInTest]);

    /* We should now have no zero buffer packets in the heap. */
    if (endStats[memberHeapInTest].numZeroBufferPackets != 0)
    {
        printf ("Error: There are %d zero buffer packets still in the Heap\n",
                        endStats[memberHeapInTest].numZeroBufferPackets);
        return -1;
    }

    /* Next time we allocate from the Super Heap; it will just iterate and give us a packet from the
     * next member heap which is MyHeap2 */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, 0);
    if (ptrPkt == NULL)
    {
        printf ("Error: Unable to allocate bufferless packet from super heap\n");
        return -1;
    }

    /* Get the heap stats for the member heap --> MyHeap2 */
    Pktlib_getHeapStats(memberHeaps[2], &endStats[2]);

    /* Verify: */
    if (endStats[2].numZeroBufferPackets + 1 != startStats[2].numZeroBufferPackets)
    {
        printf ("Error: Zero buffer packet was NOT allocated from the Heap2\n");
        return -1;
    }

    /* Free the zero buffer packet */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Debug Message: */
    printf ("Debug: Use Case 3 --> Super Heap bufferless packet allocation (Member->Member) passed\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test multiple heaps and the ability of
 *      the packet library API to work with packets belonging to multiple
 *      heaps.
 *
 *  @param[in]  ptrResCfg
 *      Resource configuration
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_multipleHeaps(Resmgr_ResourceCfg* ptrResCfg)
{
    Pktlib_HeapHandle   myHeap1;
    Pktlib_HeapHandle   myHeap2;
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataLen;
    uint32_t            index;
    Pktlib_HeapCfg      heapCfg;
    int32_t             errCode;
    Pktlib_HeapStats    heapStats;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Testing Multiple Heaps\n");

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "MyHeap1");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[1].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 128;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;

    /* Create Heap1 with specified configuration. */
    myHeap1 = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap1 == NULL)
    {
        printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "MyHeap2");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[1].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;

    /* Create Heap2 with specified configuration. */
    myHeap2 = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap2 == NULL)
    {
        printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Allocate a packet of 100 bytes from Heap1 */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap1, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Get the data buffer for the Packet1 */
    Pktlib_getDataBuffer(ptrPkt1, &ptrDataBuffer, &dataLen);

    /* Create a dummy payload which we can use for verification. */
    for (index = 0; index < dataLen; index++)
        *(ptrDataBuffer + index) = 0xAA;

    /* Allocate a Zero buffer packet for the clone from Heap2 */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap2, 0);
    if (ptrPkt2 == NULL)
        return -1;

    /* Clone the packets. */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrPkt1, ptrPkt2) < 0)
        return -1;

    /* Validation: Ensure that the data buffer of the cloned packet has the length and data buffer
     * as we expected. */
    Pktlib_getDataBuffer(ptrPkt2, &ptrDataBuffer, &dataLen);
    if (dataLen != 100)
        return -1;

    /* Validation: Ensure that the data payload is correct. */
    for (index = 0; index < dataLen; index++)
        if (*(ptrDataBuffer + index) != 0xAA)
            return -1;

    /* Ok time to cleanup the packets */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Validation: Check for memory leaks on both the heaps all the packets should be present in the free queues */
    Pktlib_getHeapStats(myHeap1, &heapStats);
    if ((heapStats.numPacketsinGarbage  != 0)  ||
        (heapStats.numFreeDataPackets   != 16) ||
        (heapStats.numZeroBufferPackets != 16))
        return -1;
    Pktlib_getHeapStats(myHeap2, &heapStats);
    if ((heapStats.numPacketsinGarbage  != 0)  ||
        (heapStats.numFreeDataPackets   != 16) ||
        (heapStats.numZeroBufferPackets != 16))
        return -1;

    /* Debug Message: Multiple Heap API testing was successful. */
    printf ("Debug: Multiple heap test was successful.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the operation of free with different use cases
 *      and to ensure that packets are correctly placed into the free and garbage queues
 *      under different uses cases and there are no memory leaks. The uses cases
 *      described here are scenarios where the allocated packets are passed to an
 *      IP block.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibraryFree(void)
{
    Ti_Pkt*             ptrPkt1;
    Ti_Pkt*             ptrPkt2;
    Ti_Pkt*             ptrPkt3;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Testing Packet Library Free use cases\n");

    /******************************************************************************
     * TEST 1:
     *  This is a most common use case where the application allocates a packet,
     *  populates it with data and then passes it to an IP block. The packet should
     *  get returned automatically to the heap free buffer queue without any host
     *  intervention.
     *      - Packet P1 is allocated
     *      - Packet P1 is sent through the DUMMY IP Send
     *      - Packet P1 gets recycled as per the return queue in the descriptor.
     *      - Application DOES NOT CLEANUP P1.
     ******************************************************************************/
    printf ("Debug: Use Case1 -> Sending Packet to IP\n");

    /* Allocate a packet. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt1);

    /* Pass it to the DUMMY IP Block; which will end up passing it back to the corresponding return queue */
    dummy_ip_send(ptrPkt1);

    /* Validation: The packet automatically gets recycled by the IP block so the application does not need to
     * do anything. We should be the same as before and there should be no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 2:
     *  This is a slightly complicated use case which builds up on the simpler use
     *  case described above. The application allocates a packet, clones it and then
     *  passes the orignal packet to an IP block. In this scenario the Heap Garbage
     *  collector needs to execute because cloned packets have a non-zero reference
     *  count and these can only be cleaned up at a later point in time.
     *      - Packet P1 is allocated
     *      - Packet P2 is cloned from P1
     *      - Orignal Packet P1 is sent through the DUMMY IP Send
     *      - Cloned Packet P2 is cleaned up by the application.
     *      - Application DOES NOT CLEANUP P1.
     *      - Packet P1 should be in the garbage queue & is cleaned up by the
     *        garbage collector.
     ******************************************************************************/
    printf ("Debug: Use Case2 -> Cloning and Sending Orignal Packet to IP (With Garbage Collection)\n");

    /* Allocate a packet. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Allocate a zero buffer packet. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt2 == NULL)
        return -1;

    /* Clone the packets */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrPkt1, ptrPkt2) < 0)
        return -1;

    /* Cleanup the cloned packet. The CLONED packet is immediately placed back into
     * the heap zero free queue. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /******************************************************************************
     * NOTE: There is no reason to writeback the packet here before we pass the
     * packet to the CPDMA block; because the cloned packet API will ensure that the
     * packet contents are correctly written back.
     ******************************************************************************/

    /* Pass orignal packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue (This should be the Garbage Queue of the Heap) */
    dummy_ip_send(ptrPkt1);

    /* Validation:  The ORIGNAL Packet resides in the Garbage queue. The CLONED packet is immediately
     * removed and placed into the corresponding free zero queue. Get the heap stats and
     * ensure that this is met. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 1) || ((endStats.numFreeDataPackets + 1) != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Run the Garbage collector */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* This should now have removed the packet from the garbage queue and placed it into the free queue.
     * So now we should have no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 3:
     *  The application allocates a packet, clones it and then passes the
     *  cloned packet to an IP block. In this scenario the Heap Garbage
     *  collector needs to execute because cloned packets have a non-zero reference
     *  count and these can only be cleaned up at a later point in time.
     *      - Packet P1 is allocated
     *      - Packet P2 is cloned from P1
     *      - Cloned Packet P2 is sent through the DUMMY IP Send
     *      - Orignal Packet P1 is cleaned by the application
     *      - Application DOES NOT CLEANUP packet P2
     *      - Run the Garbage collector to ensure that the packet P2 is moved back
     *        to the Heap Free Queue
     ******************************************************************************/
    printf ("Debug: Use Case3 -> Cloning and Sending Cloned Packet to IP(With Garbage Collection)\n");

    /* Allocate a packet. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Allocate a zero buffer packet. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt2 == NULL)
        return -1;

    /* Clone the packets */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrPkt1, ptrPkt2) < 0)
        return -1;

    /******************************************************************************
     * NOTE: There is no reason to writeback the packet here before we pass the
     * packet to the CPDMA block; because the clone packet API will ensure that the
     * packet contents are correctly written back.
     ******************************************************************************/

    /* Pass cloned packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue (This should be the Garbage Queue) */
    dummy_ip_send(ptrPkt2);

    /* Cleanup the orignal packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* The Cloned packet will go to the GARBAGE Queue. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if (endStats.numPacketsinGarbage != 1)
        return -1;

    /* Run the Garbage collector */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* This should now have removed the packet from the garbage queue and placed it into
     * the free queue. So now we should have no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 4:
     *  This test case is used to showcase a doubly cloned packet is handled.
     *      - Packet P1 is allocated
     *      - Packet P2 is cloned from the orignal packet P1
     *      - Packet P3 is cloned from the clone P2
     *      - Packet P3 is sent to the IP block.
     ******************************************************************************/
    printf ("Debug: Use Case4 -> Double cloned IP send.\n");

    /* Allocate a packet. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Allocate a zero buffer packet. */
    ptrPkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt2 == NULL)
        return -1;

    /* Allocate a zero buffer packet. */
    ptrPkt3 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt3 == NULL)
        return -1;

    /* Clone the packets */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrPkt1, ptrPkt2) < 0)
        return -1;

    /* Clone the packets */
    if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrPkt2, ptrPkt3) < 0)
        return -1;

    /******************************************************************************
     * NOTE: There is no reason to writeback the packet here before we pass the
     * packet to the CPDMA block; because the clone packet API will ensure that the
     * packet contents are correctly written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt3);

    /* Pass cloned packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue */
    dummy_ip_send(ptrPkt3);

    /* Cleanup the cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Cleanup the orignal packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Run the Garbage collector */
    Pktlib_garbageCollection(appPktlibInstanceHandle, myHeap);

    /* Validation: Get the heap stats and make sure there are no memory leaks. We should NOT have
     * any packets in the GARBAGE Queue (No Garbage Collection required) */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Test Passed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the copying of the meta data from one packet
 *      to another.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t test_pktLibCopyMetaData(void)
{
    Ti_Pkt*     ptrOrigPkt;
    Ti_Pkt*     ptrTestPkt;
    uint8_t     psData[32];
    uint8_t*    ptrPSInfo = NULL;
    uint32_t    psInfoLen = 0;
    uint32_t    timeStamp = 0;
    uint32_t    swInfo0 = 0;
    uint32_t    swInfo1 = 0;
    uint32_t    swInfo2 = 0;

    /* Debug Message: */
    printf ("------------------------------------------------------\n");
    printf ("Debug: Testing Packet Library Copy Meta Data\n");

    /* Allocate the original packet from the heap: */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 50);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate the test packet from the heap: */
    ptrTestPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 50);
    if (ptrTestPkt == NULL)
        return -1;

    /* Setup the meta data [EPIB] in the original packet: */
    Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc*)ptrOrigPkt, 0xCCCCCCCC);
    Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrOrigPkt, 0x12345678);
    Cppi_setSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrOrigPkt, 0xABCDABCD);
    Cppi_setSoftwareInfo2 (Cppi_DescType_HOST, (Cppi_Desc*)ptrOrigPkt, 0xABABABAB);

    /* Setup the meta data [Protocol Specific] */
    memset ((void *)&psData[0], 0xEE, sizeof(psData));
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc*)ptrOrigPkt, &psData[0], sizeof(psData));

    /* Ensure that the test packet does not have the EPIB Information: */
    Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt, &timeStamp);
    swInfo0 = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    swInfo1 = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    swInfo2 = Cppi_getSoftwareInfo2 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    if ((timeStamp == 0xCCCCCCCC) || (swInfo0 == 0x12345678) || (swInfo1 == 0xABCDABCD) || (swInfo2 == 0xABABABAB))
    {
        printf ("Error: EPIB Information is the same in the test packet\n");
        return -1;
    }

    /* Ensure that the test packet does not have the PS Information: */
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrTestPkt, &ptrPSInfo, &psInfoLen);
    if (ptrPSInfo != NULL)
    {
        if (psInfoLen == sizeof(psData))
        {
            printf ("Error: PS Information length is the same in the test packet\n");
            return -1;
        }

        if (memcmp (ptrPSInfo, &psData, sizeof (psData)) == 0)
        {
            printf ("Error: PS Information data is the same in the test packet\n");
            return -1;
        }
    }

    /* Copy over the meta data from the original packet to the test packet */
    Pktlib_copyMetaData (appPktlibInstanceHandle, ptrTestPkt, ptrOrigPkt);

    /* Validate: Once the meta data has been copied; validate all the meta data */
    Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt, &timeStamp);
    swInfo0 = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    swInfo1 = Cppi_getSoftwareInfo1 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    swInfo2 = Cppi_getSoftwareInfo2 (Cppi_DescType_HOST, (Cppi_Desc*)ptrTestPkt);
    if ((timeStamp != 0xCCCCCCCC) || (swInfo0 != 0x12345678) || (swInfo1 != 0xABCDABCD) || (swInfo2 != 0xABABABAB))
    {
        printf ("Error: EPIB Information is NOT the same in the test packet\n");
        return -1;
    }

    /* Validate: The protocol specific information should be the same */
    Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrTestPkt, &ptrPSInfo, &psInfoLen);
    if (ptrPSInfo == NULL)
    {
        printf ("Error: PS Information is NOT present in the test packet\n");
        return -1;
    }

    /* Validate: Ensure that the length is what we expect it to be */
    if (psInfoLen != sizeof(psData))
    {
        printf ("Error: PS Information length is not the same in the test packet [Expected %d Got %d]\n", sizeof(psData), psInfoLen);
        return -1;
    }
    if (memcmp (ptrPSInfo, &psData, sizeof (psData)) != 0)
    {
        printf ("Error: PS Information data is not the same in the test packet\n");
        return -1;
    }

    /* Free the packets */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrTestPkt);
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the PKTLIB
 *
 *  @param[in]  ptrResCfg
 *      Resource configuration
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
static int32_t testPktlib (Resmgr_ResourceCfg* ptrResCfg)
{
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;

    /* Execute the tests: */
    printf ("************************************************************************\n");
    printf ("Debug: Executing the PKTLIB Unit Tests.\n");
    printf ("************************************************************************\n");

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "My Test Heap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 512;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 64;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }
    printf ("Debug: Heap %p has been created successfully\n", myHeap);

    /* Sanity Check: Find the PKTLIB heap */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, "My Test Heap", &errCode) != myHeap)
    {
        printf ("Error: Find Heap by name failed\n");
        return -1;
    }
    printf ("Debug: Find heap test passed\n");

    if (test_pktLibraryMerge() < 0)
    {
        printf ("Error: Merge Packet Test failed\n");
        return -1;
    }
    if (test_pktLibraryClone() < 0)
    {
        printf ("Error: Clone Packet Test failed\n");
        return -1;
    }
    if (test_pktLibrarySplit() < 0)
    {
        printf ("Error: Split Packet Test failed\n");
        return -1;
    }
    if (test_pktLibrarySplit2() < 0)
    {
        printf ("Error: Split Packet2 Test failed\n");
        return -1;
    }
    if (test_multipleHeaps(ptrResCfg) < 0)
    {
        printf ("Error: Multiple Heap Test failed\n");
        return -1;
    }
    if (test_pktLibraryFree() < 0)
    {
        printf ("Error: Free Test failed\n");
        return -1;
    }
    if (test_pktLibSuperHeaps() < 0)
    {
        printf ("Error: Super Heap Test failed\n");
        return -1;
    }
    if (test_pktLibCopyMetaData() < 0)
    {
        printf ("Error: Copy Meta Data Test failed\n");
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point for the unit test application
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t main(int argc, char* argv[])
{
    Resmgr_SystemCfg            sysConfig;
    int32_t                     errCode;
    Pktlib_InstCfg              pktlibInstCfg;
    Name_DatabaseCfg            databaseCfg;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = 1;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "UnitTest");

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                 = Resmgr_ExecutionRealm_ARM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.coreId                = 8;
    sysConfig.malloc                = Resmgr_osalMalloc;
    sysConfig.free                  = Resmgr_osalFree;
    sysConfig.mallocMemoryRegion    = Resmgr_osalMallocMemoryRegion;
    sysConfig.freeMemoryRegion      = Resmgr_osalFreeMemoryRegion;
    sysConfig.createSem             = Resmgr_osalCreateSem;
    sysConfig.pendSem               = Resmgr_osalPendSem;
    sysConfig.postSem               = Resmgr_osalPostSem;
    sysConfig.deleteSem             = Resmgr_osalDeleteSem;
    sysConfig.beginMemAccess        = Resmgr_osalBeginMemAccess;
    sysConfig.endMemAccess          = Resmgr_osalEndMemAccess;

    /* Initialize the system configuration. */
    handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (handleSysCfg == NULL)
	{
	    printf ("Error: SYSRM initialization failed with error code %d\n", errCode);
	    return -1;
    }
    printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = globalNameDatabaseHandle;
    pktlibInstCfg.sysCfgHandle      = handleSysCfg;
    pktlibInstCfg.malloc            = Pktlib_osalMalloc;
    pktlibInstCfg.free              = Pktlib_osalFree;
    pktlibInstCfg.beginMemAccess    = Pktlib_osalBeginMemAccess;
    pktlibInstCfg.endMemAccess      = Pktlib_osalEndMemAccess;
    pktlibInstCfg.beginPktAccess    = Pktlib_osalBeginPktAccess;
    pktlibInstCfg.endPktAccess      = Pktlib_osalEndPktAccess;
    pktlibInstCfg.enterCS           = Pktlib_osalEnterCS;
    pktlibInstCfg.exitCS            = Pktlib_osalExitCS;
    pktlibInstCfg.phyToVirt         = Pktlib_osalPhyToVirt;

    /* Create the PKTLIB instance */
    appPktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (appPktlibInstanceHandle == NULL)
    {
        printf ("Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Execute the unit tests. */
    if (testPktlib(&appResourceConfig) < 0)
        printf ("Error: PKTLIB unit tests failed\n");
    else
        printf ("Debug: PKTLIB unit tests passed\n");

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (appPktlibInstanceHandle, &errCode) < 0)
    {
        printf ("Error: PKLIB Instance deletion failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: PKTLIB Instance has been deleted\n");

    /* Delete the database */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
        printf ("Error: Database deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: Database deletion successful\n");

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");
    return 0;
}

