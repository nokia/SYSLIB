/**
 *   @file  main_core0.c
 *
 *   @brief
 *      PKTLIB Unit Test Code.
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
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/SysMin.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/Cache.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/pktlib/pktlib.h>

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Maximum Data Size of each buffer */
#define MAX_DATA_SIZE               1024

/**********************************************************************
 *********************** Unit Test Extern Definitions *****************
 **********************************************************************/

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* OSAL callout */
extern uint32_t l2_global_address (uint32_t addr);

/**********************************************************************
 *********************** Unit Test Global Definitions *****************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
Resmgr_SysCfgHandle         handleSysCfg;
Name_DBHandle				globalNameDatabaseHandle;
Pktlib_InstHandle           appPktlibInstanceHandle;
Pktlib_HeapHandle           myHeap;

/* Application requested resources */
Resmgr_ResourceCfg    appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
	0,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
    0,    /* Number of QPEND Queues requested                                 */
	/* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                      Linking RAM,                            Num,    Size */
		{ "Core0-DDR3",  Resmgr_MemRegionType_DDR3,   Resmgr_MemRegionLinkingRAM_DONT_CARE,   4096,    128},
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
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);

/* Name Database: */
extern void* Name_osalDBMalloc (Name_MallocMode, uint32_t, uint32_t);
extern void  Name_osalDBFree (Name_MallocMode, void* , uint32_t);
extern void* Name_osalEnterMultipleCoreCS (void);
extern void  Name_osalExitMultipleCoreCS (void*);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);

/* PKTLIB: */
extern void* Pktlib_osalMalloc(Pktlib_MallocMode, uint32_t);
extern void  Pktlib_osalFree(Pktlib_MallocMode, void*, uint32_t);
extern void  Pktlib_osalBeginMemAccess(void*, uint32_t);
extern void  Pktlib_osalEndMemAccess(void*, uint32_t);
extern void  Pktlib_osalBeginPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void  Pktlib_osalEndPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void* Pktlib_osalEnterCS(Pktlib_HeapHandle);
extern void  Pktlib_osalExitCS(Pktlib_HeapHandle, void*);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/


static void flushSysMinToArm()
{
#if (ARM_DSP_DOWNLOAD == 1)
    Cache_wbL1dAll();
    Cache_wbAll();
#endif
}

/**
 *  @b Description
 *  @n
 *      Utility function which validates the packet for the following:
 *      - Navigator Guidelines
 *      - Cache coherency
 *
 *  @param[in]  ptrPkt
 *      Packet which is to be validated
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t validatePkt(Ti_Pkt* ptrPkt)
{
    Ti_Pkt*     ptrTempPkt;
    uint32_t    packetLen;
    uint8_t*    ptrDataBuffer;
    uint32_t    dataBufferLen;

    /* Simulating the CPDMA getting ownership. */
    Pktlib_getOwnership(appPktlibInstanceHandle, ptrPkt);

    /* Now make sure that the packet length is correct for the chain of packets. */
    packetLen  = 0;
    ptrTempPkt = ptrPkt;
    while (ptrTempPkt != NULL)
    {
        /* As per the Navigator UG: The packet length of only the first packet in the chain
         * should be non-zero. All subsequent packets in the chain should have the packet
         * length configured as 0. */
        if (packetLen != 0)
        {
            if (Pktlib_getPacketLen(ptrTempPkt) != 0)
            {
                System_printf ("Error: Invalid packet length detected for packet 0x%x Got %d\n",
                                ptrTempPkt, Pktlib_getPacketLen(ptrTempPkt) );
                return -1;
            }
        }

        /* Get the data buffer */
        Pktlib_getDataBuffer (ptrTempPkt, &ptrDataBuffer, &dataBufferLen);

        /* Keep track of the total packet length */
        packetLen = packetLen + dataBufferLen;

        /* Get the next packet */
        ptrTempPkt = Pktlib_getNextPacket(ptrTempPkt);
    }

    /* Make sure that the sum total of buffer lengths matches the packet length */
    if (packetLen != Pktlib_getPacketLen(ptrPkt))
    {
        System_printf ("Error: Expected %d length but got %d\n", Pktlib_getPacketLen(ptrPkt), packetLen);
        return -1;
    }

    /* Packet looks good. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory which is to be allocated
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* myMalloc(uint32_t size, uint32_t arg)
{
    if (arg != 0xbeefbeef)
    {
        System_printf ("Error: Invalid application defined argument [%x] on application\n", arg);
        return NULL;
    }

    return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), size, 0, NULL);
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
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void myFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    if (arg != 0xbeefbeef)
    {
        System_printf ("Error: Invalid application defined argument [%x] on cleanup\n", arg);
        return;
    }
    Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, size);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Code.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t system_init (void)
{
	Name_DatabaseCfg        databaseCfg;
    Resmgr_SystemCfg        sysConfig;
    int32_t                 errCode;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 32;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE1_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE1_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 1;
    databaseCfg.dspCfg.malloc                     = Name_osalDBMalloc;
    databaseCfg.dspCfg.free                       = Name_osalDBFree;
    databaseCfg.dspCfg.enterCS                    = Name_osalEnterMultipleCoreCS;
    databaseCfg.dspCfg.exitCS                     = Name_osalExitMultipleCoreCS;
    databaseCfg.dspCfg.beginMemAccess             = Name_osalBeginMemAccess;
    databaseCfg.dspCfg.endMemAccess               = Name_osalEndMemAccess;

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    System_printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    sysConfig.coreId                            = DNUM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.malloc                            = Resmgr_osalMalloc;
    sysConfig.free                              = Resmgr_osalFree;
    sysConfig.mallocMemoryRegion                = Resmgr_osalMallocMemoryRegion;
    sysConfig.freeMemoryRegion                  = Resmgr_osalFreeMemoryRegion;
    sysConfig.createSem                         = Resmgr_osalCreateSem;
    sysConfig.pendSem                           = Resmgr_osalPendSem;
    sysConfig.postSem                           = Resmgr_osalPostSem;
    sysConfig.deleteSem                         = Resmgr_osalDeleteSem;
    sysConfig.beginMemAccess                    = Resmgr_osalBeginMemAccess;
    sysConfig.endMemAccess                      = Resmgr_osalEndMemAccess;
    sysConfig.dspSystemCfg.armCoreId        	= SYSLIB_ARM_CORE_ID;
    sysConfig.dspSystemCfg.sourceId         	= 17;
    sysConfig.dspSystemCfg.sharedMemAddress 	= DDR3_SYSLIB_RESMGR_RSVD;
    sysConfig.dspSystemCfg.sizeSharedMemory 	= DDR3_SYSLIB_RESMGR_RSVD_LEN;

    /* Initialize the system configuration. */
    handleSysCfg = Resmgr_init(&sysConfig, &errCode);
    if (handleSysCfg == NULL)
	{
	    System_printf ("Error: SYSRM initialization failed with error code %d\n", errCode);
	    return -1;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }
    return 0;
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

    /* Validate: Validate the packet for the Navigator guidelines */
    if (validatePkt(ptrPkt) < 0)
    {
        System_printf ("Error: CPDMA Validation failed for Packet 0x%p\n", ptrPkt);
        return;
    }

    while (ptrPkt != NULL)
    {
        /* Get the return queue information to where the packet has to be pushed back into. */
        returnQueueInfo = Cppi_getReturnQueue (Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt);

        /* Get the return queue */
        returnQueueHnd = Qmss_getQueueHandle(returnQueueInfo);

        /* Now push the descriptor back into the return queue. */
        Qmss_queuePushDesc (returnQueueHnd, (void*)ptrPkt);

        /* Get the next packet in the chain. */
        ptrPkt = Pktlib_getNextPacket(ptrPkt);
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
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Merge API Test\n");

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
    System_printf ("Debug: Merge API Test API Passed\n");
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
    Ti_Pkt*             ptrOrigPkt;
    Ti_Pkt*             ptrClonePkt1;
    Ti_Pkt*             ptrClonePkt2;
    Ti_Pkt*             ptrTempPkt;
    Ti_Pkt*             ptrClonePkt[256];
    uint16_t            index;
    uint32_t            sizeofOrgPacket = 0;
    uint32_t            numPackets;
    uint32_t            dataBufferSize[10];
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;
    uint32_t            maxNumberClones = 255;

    /* Configurable Parameters: These are the parameters which can be modified by
     * to cause different behavior of the test code.
     *  Parameter 1: numPackets
     *      - This is the number of packets which are created and chained together
     *        to create the orignal packet which is to be cloned. */
    numPackets = 3;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Cloning Test starting with %d packet in a chain\n", numPackets);

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
    System_printf ("Debug: Testing cloning from the original.\n");

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
    System_printf ("Debug: Testing cloning from the clone.\n");

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
    System_printf ("Debug: Cleaning clones memory leak test\n");

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
    System_printf ("Debug: Merge an original packet after a cloned Packet (with CPDMA simulation).\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrClonePkt1, ptrTempPkt, NULL);

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
    System_printf ("Debug: Merge an original packet after a cloned Packet (with software free)\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrClonePkt1, ptrTempPkt, NULL);

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
    System_printf ("Debug: Merge a cloned packet after a cloned packet (with CPDMA simulation)\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrClonePkt1, ptrClonePkt2, NULL);

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
    System_printf ("Debug: Merge a cloned packet after a cloned packet (with software free)\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrClonePkt1, ptrClonePkt2, NULL);

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
    System_printf ("Debug: Merge a cloned packet after an original packet (with CPDMA simulation)\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt1, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrTempPkt);

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
    System_printf ("Debug: Merge a cloned packet after an original packet (with software free)\n");

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
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrTempPkt, ptrClonePkt1, NULL);

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
    System_printf ("Debug: Merge an original packet after an original packet (with CPDMA simulation)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Merge the original packets together. */
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrOrigPkt, ptrTempPkt, NULL);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrOrigPkt);

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
    System_printf ("Debug: Merge an original packet after an original packet (with software free)\n");

    /* Create the orignal packet which is to be cloned. */
    ptrOrigPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Allocate another temporary original packet. */
    ptrTempPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    if (ptrTempPkt == NULL)
        return -1;

    /* Merge the original packets together. */
    Pktlib_packetMerge(appPktlibInstanceHandle, ptrOrigPkt, ptrTempPkt, NULL);

    /* Now clean up the chained packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrOrigPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST: Cloning the packet N times to ensure stability and no memory leaks
     ******************************************************************************/
    System_printf ("Debug: Cloning packet %d times\n", maxNumberClones);

    /* Setup the chain size */
    numPackets = 3;

    /* All the packets in the chain are 128 bytes each. */
    for (index = 0; index < numPackets; index++)
        dataBufferSize[index] = 128;

    /* Create the packet chain of the data packets */
    ptrOrigPkt = create_packets (numPackets, dataBufferSize);
    if (ptrOrigPkt == NULL)
        return -1;

    /* Reset the data buffer size to be 0 for cloning */
    for (index = 0; index < numPackets; index++)
        dataBufferSize[index] = 0;

    /* Reset all the cloned packets */
    for (index = 0; index < maxNumberClones; index++)
        ptrClonePkt[index] = NULL;

    /* Keep cloning the packet till it fails. */
    for (index = 0; index < maxNumberClones; index++)
    {
        /* Create the clone packet chain. */
        ptrClonePkt[index] = create_packets (numPackets, dataBufferSize);
        if (ptrClonePkt[index] == NULL)
            return -1;

        /* Clone the packet */
        if (Pktlib_clonePacket(appPktlibInstanceHandle, ptrOrigPkt, ptrClonePkt[index]) < 0)
        {
            System_printf ("Debug: Cloning failed for iteration %d\n", index);
            break;
        }
    }

    /* We now free all the packets which have been cloned and */
    for (index = 0; index < maxNumberClones; index++)
    {
        if (ptrClonePkt[index] != NULL)
            Pktlib_freePacket (appPktlibInstanceHandle, ptrClonePkt[index]);
    }

    /* Cleanup the original packet */
    Pktlib_freePacket (appPktlibInstanceHandle, ptrOrigPkt);

    /* Validation: Check for memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Debug Message: */
    System_printf ("Debug: All cloning tests passed.\n");
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
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 1: Split API Testing
     * - Split occurs in the middle of first packet.
     ******************************************************************************/
    System_printf ("Debug: Testing split in the middle of the first packet\n");

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
    System_printf ("Debug: Testing split in the middle of the %d packet\n", middleSplit);

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
    System_printf ("Debug: Testing split at the end of the %d packet\n", perfectSplitPacket);

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
    System_printf ("Debug: Testing 10 1 byte splits in the packet.\n");

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
        retVal = Pktlib_splitPacket(appPktlibInstanceHandle, ptrOrigPkt, ptrSplitPkt, 1, &ptrPkt1, &ptrPkt2);

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
    System_printf ("Debug: Split API test was successful.\n");
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
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 1: Split API Testing
     * - Split occurs in the middle of first packet.
     ******************************************************************************/
    System_printf ("Debug: Testing split in the middle of the first packet\n");

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
    System_printf ("Debug: Testing split in the middle of the %d packet\n", middleSplit);

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
    System_printf ("Debug: Testing split at the end of the %d packet\n", perfectSplitPacket);

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
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Split Test starting with %d packets in a chain\n", numPackets);

    /******************************************************************************
     * TEST 4: Split API Testing
     * - Multiple splits done on the packet.
     ******************************************************************************/
    System_printf ("Debug: Testing 10 1 byte splits in the packet.\n");

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
    System_printf ("Debug: Split API test was successful.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test multiple heaps and the ability of
 *      the packet library API to work with packets belonging to multiple
 *      heaps.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_multipleHeaps(void)
{
    Pktlib_HeapHandle       myHeap1;
    Pktlib_HeapHandle       myHeap2;
    Ti_Pkt*                 ptrPkt1;
    Ti_Pkt*                 ptrPkt2;
    uint8_t*                ptrDataBuffer;
    uint32_t                dataLen;
    uint32_t                index;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Pktlib_HeapStats        heapStats;
    Resmgr_ResourceCfg*     ptrResCfg;

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Multiple Heaps\n");

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "MyHeap1");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 128;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;

    /* Create Heap1 with specified configuration. */
    myHeap1 = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap1 == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "MyHeap2");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;

    /* Create Heap2 with specified configuration. */
    myHeap2 = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap2 == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
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
    System_printf ("Debug: Multiple heap test was successful.\n");
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
    Qmss_Queue          appQueueInfo;
    Qmss_QueueHnd       appQueueHnd;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;
    uint8_t             isAllocated;

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(myHeap, &startStats);

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Packet Library Free use cases\n");

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
    System_printf ("Debug: Use Case1 -> Sending Packet to IP\n");

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
    System_printf ("Debug: Use Case2 -> Cloning and Sending Orignal Packet to IP (With Garbage Collection)\n");

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
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt1);

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
    System_printf ("Debug: Use Case3 -> Cloning and Sending Cloned Packet to IP(With Garbage Collection)\n");

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
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt2);

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
     *  This test case is used to showcase how the Heap Garbage Collector is NOT
     *  required. The application allocates a packet but before passing it to the
     *  IP block overrides the return queue in the descriptor to an application
     *  owned queue. The packet is then passed to the IP block. Applications are
     *  now responsible for cleaning up the packet P1. This is called "Application
     *  Deferred Cleanup" where application can determine when to recycle packets
     *  back into the free queue.
     *      - Packet P1 is allocated
     *      - Application takes over ownership of the Packet P1
     *      - Packet P1 is sent through the DUMMY IP Send
     *      - P1 will reside in the application modified return queue.
     *      - Application DOES CLEANUP P1
     *
     *  This is similar to TEST 1. But the difference is that there is no automatic
     *  recycling of the packets to the heap free queue because the application
     *  holds on to the packet till it is determines it is done with the packet.
     *  For example: Hold on to the packet till we get an ACK from the other end.
     ******************************************************************************/
    System_printf ("Debug: Use Case4 -> Sending Packet to IP with App deferred cleanup\n");

    /* Allocate a packet. */
    ptrPkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 100);
    if (ptrPkt1 == NULL)
        return -1;

    /* Open the application queue */
    appQueueHnd =  Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                  QMSS_PARAM_NOT_SPECIFIED,
                                  &isAllocated);
    if (appQueueHnd < 0)
    {
        System_printf ("Error: Unable to open the temporary application queue\n");
        return -1;
    }

    /* Get the queue information */
    appQueueInfo = Qmss_getQueueNumber (appQueueHnd);

    /* Override the return queue information in the packet with the application value. */
    Cppi_setReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt1, appQueueInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt1);

    /* Pass cloned packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue (This is the application queue) */
    dummy_ip_send(ptrPkt1);

    /* This will cause the application queue to be completely flushed out. */
    Qmss_queuePushDesc(appQueueHnd, NULL);

    /* Cleanup the orignal packet. The ORIGNAL packet is now freed up. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Validation: Get the heap stats and make sure there are no memory leaks. We should NOT have
     * any packets in the GARBAGE Queue (No Garbage Collection required) */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 5:
     *  This test case is used to showcase how the Heap Garbage Collector is NOT
     *  required. This is the more complicated use case where the packet is allocated
     *  and cloned also. The cloned packet is passed to the IP block. Application
     *  takes over ownership and defers the cleanup of these packets.
     *      - Packet P1 is allocated
     *      - Packet P2 is cloned from the orignal packet P1
     *      - Application takes over ownership of the Packet P2
     *      - Packet P2 is sent through the DUMMY IP Send
     *      - Application DOES CLEANUP P1 and P2.
     ******************************************************************************/
    System_printf ("Debug: Use Case5 -> Cloning Packets and sending cloned packets to IP with App deferred cleanup\n");

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

    /* Open the application queue */
    appQueueHnd =  Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                  QMSS_PARAM_NOT_SPECIFIED,
                                  &isAllocated);
    if (appQueueHnd < 0)
    {
        System_printf ("Error: Unable to open the temporary application queue\n");
        return -1;
    }

    /* Get the queue information */
    appQueueInfo = Qmss_getQueueNumber (appQueueHnd);

    /* Override the return queue information in the cloned packet with the application return queue */
    Cppi_setReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)ptrPkt2, appQueueInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt2);

    /* Pass cloned packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue (This is the application queue) */
    dummy_ip_send(ptrPkt2);

    /* This will cause the application queue to be completely flushed out. */
    Qmss_queuePushDesc(appQueueHnd, NULL);

    /* Cleanup the cloned packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt2);

    /* Cleanup the orignal packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt1);

    /* Validation: Get the heap stats and make sure there are no memory leaks. We should NOT have
     * any packets in the GARBAGE Queue (No Garbage Collection required) */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /******************************************************************************
     * TEST 6:
     *  This test case is used to showcase a doubly cloned packet is handled.
     *      - Packet P1 is allocated
     *      - Packet P2 is cloned from the orignal packet P1
     *      - Packet P3 is cloned from the clone P2
     *      - Packet P3 is sent to the IP block.
     ******************************************************************************/
    System_printf ("Debug: Use Case6 -> Double cloned IP send.\n");

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
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrPkt3);

    /* Pass cloned packet to the DUMMY IP Block; which will end up passing it back to the
     * corresponding return queue (This is the application queue) */
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
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Super Heap \n");

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
    System_printf ("Debug: 'My Test Heap' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[0]));
    System_printf ("Debug: 'MyHeap1' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[1]));
    System_printf ("Debug: 'MyHeap2' has a MAX data buffer size of %d bytes\n", Pktlib_getMaxBufferSize(memberHeaps[2]));

    /* Create the super heap */
    superHeapHandle = Pktlib_createSuperHeap(appPktlibInstanceHandle, "SuperHeap", memberHeaps, 3, &errCode);
    if (superHeapHandle == NULL)
    {
        System_printf ("Error: Unable to create super heap error code: %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Super Heap has been created with handle 0x%p\n", superHeapHandle);

    /* Check: If we can find the Super Heap. */
    if (Pktlib_findHeapByName(appPktlibInstanceHandle, "SuperHeap", &errCode) != superHeapHandle)
    {
        System_printf ("Error: Super Heap could NOT be found\n");
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
    System_printf ("Debug: Use Case 1 --> Different Packet Size allocation check\n");

    /* Get the current heap statistics. */
    Pktlib_getHeapStats(memberHeaps[0], &startStats[0]);
    Pktlib_getHeapStats(memberHeaps[1], &startStats[1]);
    Pktlib_getHeapStats(memberHeaps[2], &startStats[2]);

    /* Set the index to the heap being tested: We know that the 'MyHeap1' is the heap
     * with the smallest data buffer size. This is added to the Super Heap as Member
     * Heap 1. */
    memberHeapInTest = 1;

    /* Debug Message: */
    System_printf ("Debug: Testing Packet Size allocations from 1 to %d bytes\n",
                   Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /* Cycle through all possible heap sizes and make sure the allocations are done properly. */
    for (packetSize = 1; packetSize <= Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]); packetSize++)
    {
        /* Allocate a packet from the Super Heap */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, packetSize);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
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
            System_printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
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
                    System_printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
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
    System_printf ("Debug: Testing Packet Size allocations from %d to %d bytes\n",
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
            System_printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
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
            System_printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
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
                    System_printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
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
    System_printf ("Debug: Testing Packet Size allocations from %d to %d bytes\n",
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
            System_printf ("Error: Unable to allocate %d bytes from super heap\n", packetSize);
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
            System_printf ("Error: Packet allocated was not from Heap %d for %d bytes\n", memberHeapInTest, packetSize);
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
                    System_printf ("Error: Heap %d was accessed for %d bytes alloc\n", index, packetSize);
                    return -1;
                }
            }
        }
        /* Clean up the allocated packet. */
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }
    System_printf ("Debug: Super Heap Packet size allocation (1 to %d) bytes checks passed\n",
                    Pktlib_getMaxBufferSize(memberHeaps[memberHeapInTest]));

    /******************************************************************************
     * TEST 2:
     *  The test does the ZERO Byte packet allocation on super heaps.
     ******************************************************************************/

    /* Allocate a packet from the Super Heap */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, 0);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate bufferless packet from super heap\n");
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
        System_printf ("Error: Zero buffer packet was NOT allocated from the %d heap\n", memberHeapInTest);
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: Use Case 2 --> Super Heap bufferless packet allocation passed\n");

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
            System_printf ("Error: Unable to allocate bufferless packet from super heap\n");
            return -1;
        }
    }

    /* Get the heap stats for the member heap */
    Pktlib_getHeapStats(memberHeaps[memberHeapInTest], &endStats[memberHeapInTest]);

    /* We should now have no zero buffer packets in the heap. */
    if (endStats[memberHeapInTest].numZeroBufferPackets != 0)
    {
        System_printf ("Error: There are %d zero buffer packets still in the Heap\n",
                        endStats[memberHeapInTest].numZeroBufferPackets);
        return -1;
    }

    /* Next time we allocate from the Super Heap; it will just iterate and give us a packet from the
     * next member heap which is MyHeap2 */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, superHeapHandle, 0);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate bufferless packet from super heap\n");
        return -1;
    }

    /* Get the heap stats for the member heap --> MyHeap2 */
    Pktlib_getHeapStats(memberHeaps[2], &endStats[2]);

    /* Verify: */
    if (endStats[2].numZeroBufferPackets + 1 != startStats[2].numZeroBufferPackets)
    {
        System_printf ("Error: Zero buffer packet was NOT allocated from the Heap2\n");
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: Use Case 3 --> Super Heap bufferless packet allocation (Member->Member) passed\n");

    /* Super Heap test was successful. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the thresholds in the heap.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t test_pktLibHeapThresholds(void)
{
    Pktlib_HeapCfg      heapCfg;
    Pktlib_HeapHandle   heapHandle;
    int32_t             errCode;
    Ti_Pkt*             ptrPkt;
    Pktlib_HeapStats    heapStats;
    uint32_t            index;
    uint8_t             isAllocated;
    Qmss_QueueHnd       queueHandle;
    Resmgr_ResourceCfg*     ptrResCfg;

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Heap Thresholds \n");

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Open a general purpose temporary queue */
    queueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                 QMSS_PARAM_NOT_SPECIFIED,
                                 &isAllocated);
    if (queueHandle < 0)
    {
        System_printf ("Error: Unable to open the temporary queue\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The thresholds should be a power of 2 */
    strcpy(heapCfg.name, "MyThresholdHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 64;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 4;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the heap with specified configuration. */
    heapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /******************************************************************************
     * TEST 1:
     *  Test the Data Buffer Starvation Threshold.
     *  - Keep allocating packets till the threshold value & make sure that the
     *    threshold status is NOT set.
     *  - Allocate another packet and make sure threshold status is SET.
     ******************************************************************************/

    /* We will allocate packets from the heap till we reach the threshold values and keep
     * verifying that the data buffer thresholds in the statistics are NOT updated. */
    for (index = 0; index < (heapCfg.numPkts - heapCfg.dataBufferPktThreshold) + 1; index++)
    {
        /* Get the heap stats for the heap. */
        Pktlib_getHeapStats(heapHandle, &heapStats);

        /* Validate: The threshold status for the Free Buffer queues should not be set */
        if (heapStats.dataBufferThresholdStatus != 0)
        {
            System_printf ("Error: Heap Data Threshold is already set\n");
            return -1;
        }

        /* Allocate a packet */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 10);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the heap\n");
            return -1;
        }

        /* Dump the packet into a temporary queue. */
        Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);
    }

    /* The next allocation of the data buffer packet should set the STARVATION threshold. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 10);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the heap\n");
        return -1;
    }

    /* Get the heap stats for the heap. */
    Pktlib_getHeapStats(heapHandle, &heapStats);

    /* Validate: Make sure that the data buffer threshold is SET */
    if (heapStats.dataBufferThresholdStatus == 0)
    {
        System_printf ("Error: Heap Data Threshold is NOT set\n");
        return -1;
    }

    /* Cleanup the packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Free all the packets from the temporary queue */
    while (1)
    {
        ptrPkt = Qmss_queuePop(queueHandle);
        if (ptrPkt == NULL)
            break;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }

    /* Once all the packets are back into the heap. The heap should NOT report data buffer threshold. */
    Pktlib_getHeapStats(heapHandle, &heapStats);
    if (heapStats.dataBufferThresholdStatus == 1)
    {
        System_printf ("Error: Heap Data Threshold is set\n");
        return -1;
    }
    System_printf ("Debug: Heap Data Buffer Threshold Test Passed\n");

    /******************************************************************************
     * TEST 2:
     *  Test the Zero Buffer Starvation Threshold.
     *  - Keep allocating packets till the threshold value & make sure that the
     *    threshold status is NOT set.
     *  - Allocate another packet and make sure threshold status is SET.
     ******************************************************************************/

    /* We will allocate packets from the heap till we reach the threshold values and keep
     * verifying that the zero data buffer thresholds in the statistics are NOT updated. */
    for (index = 0; index < (heapCfg.numPkts - heapCfg.zeroBufferPktThreshold) + 1; index++)
    {
        /* Get the heap stats for the heap. */
        Pktlib_getHeapStats(heapHandle, &heapStats);

        /* Validate: The threshold status for the Zero Buffer queues should not be set */
        if (heapStats.zeroDataBufferThresholdStatus != 0)
        {
            System_printf ("Error: Heap Zero Buffer Threshold is already set\n");
            return -1;
        }

        /* Allocate a packet */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 0);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the heap\n");
            return -1;
        }

        /* Dump the packet into a temporary queue. */
        Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);
    }

    /* The next allocation of the zero buffer packet should set the STARVATION threshold. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 0);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the heap\n");
        return -1;
    }

    /* Get the heap stats for the heap. */
    Pktlib_getHeapStats(heapHandle, &heapStats);

    /* Validate: Make sure that the zero buffer threshold is SET */
    if (heapStats.zeroDataBufferThresholdStatus == 0)
    {
        System_printf ("Error: Zero Buffer Threshold is NOT set\n");
        return -1;
    }

    /* Cleanup the packets. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Free all the packets from the temporary queue */
    while (1)
    {
        ptrPkt = Qmss_queuePop(queueHandle);
        if (ptrPkt == NULL)
            break;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }

    /* Once all the packets are back into the heap. The heap should NOT report zero data buffer threshold. */
    Pktlib_getHeapStats(heapHandle, &heapStats);
    if (heapStats.zeroDataBufferThresholdStatus == 1)
    {
        System_printf ("Error: Heap Zero Data Threshold is set\n");
        return -1;
    }
    System_printf ("Debug: Heap Zero Buffer Threshold Test Passed\n");

    /* Test passed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the starvation heaps
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t test_pktLibStarvationHeaps(void)
{
    Pktlib_HeapCfg          heapCfg;
    Pktlib_HeapHandle       heapHandle;
    int32_t                 errCode;
    Ti_Pkt*                 ptrPkt;
    Pktlib_HeapStats        heapStats;
    uint32_t                index;
    uint8_t                 isAllocated;
    Qmss_QueueHnd           queueHandle;
    Resmgr_ResourceCfg*     ptrResCfg;

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Starvation Heaps \n");

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Simulating the application opening a starvation queue and ensuring that the
     * PKTLIB starvation queue allocation works properly */
    queueHandle = Qmss_queueOpen(Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                                 QMSS_PARAM_NOT_SPECIFIED,
                                 &isAllocated);
    if (queueHandle < 0)
    {
        System_printf ("Error: Unable to open the starvation queue\n");
        return -1;
    }

    /* Open a general purpose temporary queue */
    queueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                 QMSS_PARAM_NOT_SPECIFIED,
                                 &isAllocated);
    if (queueHandle < 0)
    {
        System_printf ("Error: Unable to open the temporary queue\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "MyStarvationHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 1;
    heapCfg.dataBufferSize                  = 64;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the starvation heap with specified configuration. */
    heapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (heapHandle == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Starvation Heap created successfully\n");

    /******************************************************************************
     * TEST 1:
     *  Test the Data Buffer Starvation Counter.
     *  - Allocate and empty the heap
     ******************************************************************************/

    /* Get the heap stats for the starvation heap. */
    Pktlib_getHeapStats(heapHandle, &heapStats);

    /* We will allocate packets from the heap till we reach the threshold values and keep
     * verifying that the starvation thresholds in the statistics are NOT updated. */
    for (index = 0; index < heapStats.numFreeDataPackets; index++)
    {
        /* Allocate a packet */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 10);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the starvation heap\n");
            return -1;
        }

        /* Push the packet into the temporary queue */
        Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);
    }

    /* The heap free data buffer queue is EMPTY now. We do one more allocation and this should
     * increment the starvation counter. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 10);
    if (ptrPkt != NULL)
    {
        System_printf ("Error: The heap queue should have been empty\n");
        return -1;
    }

    /* Control comes here implies that there were no more data packets available. Get the heap
     * statistics */
    Pktlib_getHeapStats(heapHandle, &heapStats);
    if ((heapStats.zeroDataBufferStarvationCounter != 0) || (heapStats.dataBufferStarvationCounter != 1))
    {
        System_printf ("Error: Starvation counters are invalid\n");
        return -1;
    }

    /* Free all the packets from the temporary queue */
    while (1)
    {
        ptrPkt = Qmss_queuePop(queueHandle);
        if (ptrPkt == NULL)
            break;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }
    System_printf ("Debug: Starvation Heap Data Buffer starvation counter test passed\n");

    /******************************************************************************
     * TEST 2:
     *  Test the Zero Data Buffer Starvation Counter.
     *  - Allocate and empty the heap
     ******************************************************************************/

    /* Get the heap stats for the starvation heap. */
    Pktlib_getHeapStats(heapHandle, &heapStats);

    /* We will allocate packets from the heap till we reach the threshold values and keep
     * verifying that the starvation thresholds in the statistics are NOT updated. */
    for (index = 0; index < heapStats.numZeroBufferPackets; index++)
    {
        /* Allocate a packet */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 0);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the starvation heap\n");
            return -1;
        }

        /* Push the packet into the temporary queue */
        Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);
    }

    /* The heap free data buffer queue is EMPTY now. We do one more allocation and this should
     * increment the starvation counter. */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, heapHandle, 0);
    if (ptrPkt != NULL)
    {
        System_printf ("Error: The heap queue should have been empty\n");
        return -1;
    }

    /* Control comes here implies that there were no more data packets available. Get the heap
     * statistics */
    Pktlib_getHeapStats(heapHandle, &heapStats);
    if ((heapStats.zeroDataBufferStarvationCounter != 1) || (heapStats.dataBufferStarvationCounter != 0))
    {
        System_printf ("Error: Starvation counters are invalid\n");
        return -1;
    }

    /* Free all the packets from the temporary queue */
    while (1)
    {
        ptrPkt = Qmss_queuePop(queueHandle);
        if (ptrPkt == NULL)
            break;
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    }
    System_printf ("Debug: Starvation Heap Zero Data Buffer starvation counter test passed\n");
    System_printf ("------------------------------------------------------\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the delete heap API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibDeleteHeaps (void)
{
    int32_t                 memoryRegionQueueNum;
    Qmss_QueueHnd           memoryRegionQueueHnd;
    int32_t                 count;
    Pktlib_HeapCfg          heapCfg;
    Pktlib_HeapHandle       myHeap;
    Ti_Pkt*                 ptrPkt;
    uint8_t                 isAllocated;
    int32_t                 errCode;
    Pktlib_HeapHandle       memberHeaps[3];
    Memory_Stats            startStats;
    Memory_Stats            endStats;
    Resmgr_ResourceCfg*     ptrResCfg;

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Delete Heap \n");

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Get the queue number & handle associated with the MEMORY Region1. In the test
     * case we are using only a single memory region for all heaps. */
    memoryRegionQueueNum = Qmss_getMemRegQueueHandle(ptrResCfg->memRegionResponse[0].memRegionHandle);
    memoryRegionQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, memoryRegionQueueNum, &isAllocated);

    /* Get the number of descriptors which are available in the memory region.
     * We have created multiple heaps by this time; so we create a new heap with the remaining descriptors. */
    count = Qmss_getQueueEntryCount(memoryRegionQueueHnd);

    /* Get the default heap stats before we start the test */
    HeapMem_getStats(heap0, &startStats);

    /**************************************************************************************************
     * Delete Heap Test: With only data buffers
     *************************************************************************************************/

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "DeleteHeapTest");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 8;
    heapCfg.numPkts                         = count;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Once the heap has been created; we allocate a packet */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, heapCfg.dataBufferSize);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Failed to allocated a packet from the heap %s\n", heapCfg.name);
        return -1;
    }

    /* Delete the heap now. This should fail because there is a packet owned by the application. */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) == 0)
    {
        System_printf ("Error: Heap Deletion was successful though packet was owned by the application\n");
        return -1;
    }
    if (errCode != PKTLIB_EDATABUFFER_MISMATCH)
    {
        System_printf ("Error: Heap deletion failed with invalid error code %d\n", errCode);
        return -1;
    }

    /* Now we delete the allocated packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Now delete the heap */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) < 0)
    {
        System_printf ("Error: Heap Deletion failed with error code %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that all the descriptors have landed back into the memory region queue */
    if (count != Qmss_getQueueEntryCount(memoryRegionQueueHnd))
    {
        System_printf ("Error: Expected %d descriptors in memory region but got only %d\n",
                        count, Qmss_getQueueEntryCount(memoryRegionQueueHnd));
        return -1;
    }

    /* Sanity Check: Make sure that once the heap is deleted. This does not exist in the system */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, heapCfg.name, &errCode) != NULL)
    {
        System_printf ("Error: The heap %s is still present in the system\n", heapCfg.name);
        return -1;
    }
    System_printf ("Debug: Delete Heap with only data buffer test passed.\n");

    /* Get the heap stats once the test is complete */
    HeapMem_getStats(heap0, &endStats);

    /* Validate and ensure that there are no memory leaks. */
    if (startStats.totalFreeSize != endStats.totalFreeSize)
    {
        System_printf ("Error: Memory leak detected after heap deletion (Expected %d bytes but Got %d bytes)\n",
                        startStats.totalFreeSize, endStats.totalFreeSize);
        return -1;
    }

    /**************************************************************************************************
     * Delete Heap Test: With only zero data buffers
     *************************************************************************************************/

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "DeleteHeapTest");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = MAX_DATA_SIZE;
    heapCfg.numPkts                         = 0;
    heapCfg.numZeroBufferPackets            = count;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Once the heap has been created; we allocate a packet */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Failed to allocated a packet from the heap %s\n", heapCfg.name);
        return -1;
    }

    /* Delete the heap now. This should fail because there is a packet owned by the application. */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) == 0)
    {
        System_printf ("Error: Heap Deletion was successful though packet was owned by the application\n");
        return -1;
    }
    if (errCode != PKTLIB_EZEROBUFFER_MISMATCH)
    {
        System_printf ("Error: Heap deletion failed with invalid error code %d\n", errCode);
        return -1;
    }

    /* Now we delete the allocated packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Now delete the heap */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) < 0)
    {
        System_printf ("Error: Heap Deletion failed with error code %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that all the descriptors have landed back into the memory region queue */
    if (count != Qmss_getQueueEntryCount(memoryRegionQueueHnd))
    {
        System_printf ("Error: Expected %d descriptors in memory region but got only %d\n",
                        count, Qmss_getQueueEntryCount(memoryRegionQueueHnd));
        return -1;
    }

    /* Sanity Check: Make sure that once the heap is deleted. This does not exist in the system */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, heapCfg.name, &errCode) != NULL)
    {
        System_printf ("Error: The heap %s is still present in the system\n", heapCfg.name);
        return -1;
    }

    /* Get the heap stats once the test is complete */
    HeapMem_getStats(heap0, &endStats);

    /* Validate and ensure that there are no memory leaks. */
    if (startStats.totalFreeSize != endStats.totalFreeSize)
    {
        System_printf ("Error: Memory leak detected after heap deletion (Expected %d bytes but Got %d bytes)\n",
                        startStats.totalFreeSize, endStats.totalFreeSize);
        return -1;
    }
    System_printf ("Debug: Delete Heap with only zero buffer test passed.\n");

    /**************************************************************************************************
     * Delete Heap Test: With data buffers & zero data buffers.
     *************************************************************************************************/

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "DeleteHeapTest");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 8;
    heapCfg.numPkts                         = count/2;
    heapCfg.numZeroBufferPackets            = count/2;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Once the heap has been created; we allocate a packet */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, heapCfg.dataBufferSize);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Failed to allocated a packet from the heap %s\n", heapCfg.name);
        return -1;
    }

    /* Delete the heap now. This should fail because there is a packet owned by the application. */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) == 0)
    {
        System_printf ("Error: Heap Deletion was successful though packet was owned by the application\n");
        return -1;
    }
    if (errCode != PKTLIB_EDATABUFFER_MISMATCH)
    {
        System_printf ("Error: Heap deletion failed with invalid error code %d\n", errCode);
        return -1;
    }

    /* Now we delete the allocated packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Now delete the heap */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) < 0)
    {
        System_printf ("Error: Heap Deletion failed with error code %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that all the descriptors have landed back into the memory region queue */
    if (count != Qmss_getQueueEntryCount(memoryRegionQueueHnd))
    {
        System_printf ("Error: Expected %d descriptors in memory region but got only %d\n",
                        count, Qmss_getQueueEntryCount(memoryRegionQueueHnd));
        return -1;
    }

    /* Sanity Check: Make sure that once the heap is deleted. This does not exist in the system */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, heapCfg.name, &errCode) != NULL)
    {
        System_printf ("Error: The heap %s is still present in the system\n", heapCfg.name);
        return -1;
    }

    /* Get the heap stats once the test is complete */
    HeapMem_getStats(heap0, &endStats);

    /* Validate and ensure that there are no memory leaks. */
    if (startStats.totalFreeSize != endStats.totalFreeSize)
    {
        System_printf ("Error: Memory leak detected after heap deletion (Expected %d bytes but Got %d bytes)\n",
                        startStats.totalFreeSize, endStats.totalFreeSize);
        return -1;
    }
    System_printf ("Debug: Delete Heap with data buffer & zero buffer test passed.\n");

    /**************************************************************************************************
     * Delete Heap Test: With data buffers & zero data buffers & starvation queues.
     **************************************************************************************************/

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "DeleteHeapTest");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 1;
    heapCfg.dataBufferSize                  = 8;
    heapCfg.numPkts                         = count/2;
    heapCfg.numZeroBufferPackets            = count/2;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }

    /* Once the heap has been created; we allocate a packet */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Failed to allocated a packet from the heap %s\n", heapCfg.name);
        return -1;
    }

    /* Delete the heap now. This should fail because there is a packet owned by the application. */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) == 0)
    {
        System_printf ("Error: Heap Deletion was successful though packet was owned by the application\n");
        return -1;
    }
    if (errCode != PKTLIB_EZEROBUFFER_MISMATCH)
    {
        System_printf ("Error: Heap deletion failed with invalid error code %d\n", errCode);
        return -1;
    }

    /* Now we delete the allocated packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Now delete the heap */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, myHeap, &errCode) < 0)
    {
        System_printf ("Error: Heap Deletion failed with error code %d\n", errCode);
        return -1;
    }

    /* Validate & ensure that all the descriptors have landed back into the memory region queue */
    if (count != Qmss_getQueueEntryCount(memoryRegionQueueHnd))
    {
        System_printf ("Error: Expected %d descriptors in memory region but got only %d\n",
                        count, Qmss_getQueueEntryCount(memoryRegionQueueHnd));
        return -1;
    }

    /* Sanity Check: Make sure that once the heap is deleted. This does not exist in the system */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, heapCfg.name, &errCode) != NULL)
    {
        System_printf ("Error: The heap %s is still present in the system\n", heapCfg.name);
        return -1;
    }

    /* Get the heap stats once the test is complete */
    HeapMem_getStats(heap0, &endStats);

    /* Validate and ensure that there are no memory leaks. */
    if (startStats.totalFreeSize != endStats.totalFreeSize)
    {
        System_printf ("Error: Memory leak detected after heap deletion (Expected %d bytes but Got %d bytes)\n",
                        startStats.totalFreeSize, endStats.totalFreeSize);
        return -1;
    }
    System_printf ("Debug: Delete Heap with data buffer & zero buffer & starvation queue test passed.\n");

    /**************************************************************************************************
     * Delete Heap Test: Super Heap Deletion Test
     **************************************************************************************************/

    /* There is a super heap created in the above unit test we will reuse the same super heap to test
     * the deletion. The Super Heap was configured to internally use the following member heaps
     *  - "My Test Heap"
     *  - "MyHeap1"
     *  - "MyHeap2" */
    myHeap = Pktlib_findHeapByName(appPktlibInstanceHandle, "SuperHeap", &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to find the super heap handle\n");
        return -1;
    }

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

    /* Allocate a packet such that it is allocated from the heap "MyHeap2" */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, Pktlib_getMaxBufferSize(memberHeaps[2]));
    if (ptrPkt == NULL)
    {
        System_printf ("Error: Unable to allocate a packet from the super heap\n");
        return -1;
    }

    /* Delete the super heap: This should always succeed (since super heaps are just a wrapper
     * over the actual heaps) */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, myHeap, &errCode) < 0)
    {
        System_printf ("Error: Super Heap deletion failed with error code %d\n", errCode);
        return -1;
    }

    /* Ensure that the Super heap does not exist in the system */
    if (Pktlib_findHeapByName (appPktlibInstanceHandle, "SuperHeap", &errCode) != NULL)
    {
        System_printf ("Error: Super Heap is still present in the system\n");
        return -1;
    }

    /* We know that the packet was allocated from "MyHeap2" so if we try and delete it then we should
     * get a failure. */
    if (Pktlib_deleteHeap(appPktlibInstanceHandle, memberHeaps[2], &errCode) == 0)
    {
        System_printf ("Error: MyHeap2 was successfully deleted even though there was a missing packet\n");
        return -1;
    }
    if (errCode != PKTLIB_EDATABUFFER_MISMATCH)
    {
        System_printf ("Error: Heap deletion failed with invalid error code %d\n", errCode);
        return -1;
    }

    /* Now clean the packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /* Get the heap stats once the test is complete */
    HeapMem_getStats(heap0, &endStats);

    /* Validate and ensure that there are no memory leaks. */
    if (startStats.totalFreeSize != endStats.totalFreeSize)
    {
        System_printf ("Error: Memory leak detected after heap deletion (Expected %d bytes but Got %d bytes)\n",
                        startStats.totalFreeSize, endStats.totalFreeSize);
        return -1;
    }

    /* Super Heap deletion test passed. */
    System_printf ("Debug: Super Heap Deletion test passed.\n");

    /* Debug Message: */
    System_printf ("Debug: Delete Heap tests passed\n");
    System_printf ("------------------------------------------------------\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function tests the various packet library API
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_pktLibrary(void)
{
    int32_t errCode;

    if (Pktlib_findHeapByName(appPktlibInstanceHandle, "My Test Heap", &errCode) != myHeap)
        return -1;

    if (test_pktLibraryMerge() < 0)
        return -1;

    if (test_pktLibraryClone() < 0)
        return -1;

    if (test_pktLibrarySplit() < 0)
        return -1;

    if (test_pktLibrarySplit2() < 0)
        return -1;

    if (test_multipleHeaps() < 0)
        return -1;

    if (test_pktLibraryFree() < 0)
        return -1;

    if (test_pktLibSuperHeaps() < 0)
        return -1;

    if (test_pktLibHeapThresholds() < 0)
        return -1;

    if (test_pktLibStarvationHeaps() < 0)
        return -1;

    if (test_pktLibDeleteHeaps() < 0)
        return -1;

    /* All tests were completed successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to benchmark the split & free operations
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t benchmark_split (uint32_t numSplits)
{
    Ti_Pkt*     pkt0;
    Ti_Pkt*     pNewPkt;
    Ti_Pkt*     pSplitPkt;
    Ti_Pkt*     pPkt[32];
    uint32_t    index;
    uint32_t    splitSize;
    int32_t     retVal;
    uint32_t    startTime;
    uint32_t    endTime;
    uint32_t    dataBufferSize[32];
    uint32_t    numChainedPackets = 10;
    uint32_t    totalPacketSize = 0;

    /* Cycle through and populate the buffer sizes for all the packets in the chain. */
    for (index = 0 ; index < numChainedPackets; index++)
    {
        dataBufferSize[index] = 100 + index;
        totalPacketSize = totalPacketSize + dataBufferSize[index];
    }

    /* Create the packets. */
    pkt0 = create_packets (numChainedPackets, dataBufferSize);
    if (pkt0 == NULL)
    {
        System_printf ("Error: Unable to create the chained packets\n");
        return -1;
    }

    /* Compute the split size */
    splitSize = totalPacketSize/numSplits;

    /* Debug Message: */
    System_printf ("------------------------------------------------------------------\n");
    System_printf ("Debug: Benchmarking Splits and Free (%d splits of %d size) %d packets in the chain \n",
                    numSplits, splitSize, Pktlib_packetBufferCount(pkt0));

    /* Split the packet into equal sized chunks. */
    for (index = 0; index < numSplits; index++)
    {
        /* Allocating a Zero Buffer packet for splitting. */
        pSplitPkt = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
        if (pSplitPkt == NULL)
            return -1;

        /* Split the packet. */
        startTime = TSCL;
        retVal    = Pktlib_splitPacket(appPktlibInstanceHandle, pkt0, pSplitPkt, splitSize, &pPkt[index], &pNewPkt);
        endTime   = TSCL;
        System_printf ("Debug: Splitting packet took %d ticks\n", (endTime - startTime));

        /* Cleanup the packet if not used. */
        if (retVal == 1)
            Pktlib_freePacket(appPktlibInstanceHandle, pSplitPkt);
        if (retVal == -1)
        {
            System_printf ("Error: Split packet failed\n");
            return -1;
        }

        /* We need to continue splitting the packet. */
        pkt0 = pNewPkt;
    }

    /* Cleanup all the packets now. */
    for (index = 0; index < numSplits; index++)
    {
        /* Get the number of chained packets */
        numChainedPackets = Pktlib_packetBufferCount(pPkt[index]);

        startTime = TSCL;
        Pktlib_freePacket(appPktlibInstanceHandle, pPkt[index]);
        endTime   = TSCL;

        System_printf ("Debug: Free Split Packet (%d chained packets) took %d ticks\n", numChainedPackets, (endTime - startTime));
    }

    /* Cleanup the original split packet too. */
    numChainedPackets = Pktlib_packetBufferCount(pkt0);
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt0);
    endTime   = TSCL;
    System_printf ("Debug: Free Split Packet (%d chained packets) took %d ticks\n", numChainedPackets, (endTime - startTime));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function benchmarks the pkt library API using a predefined
 *      use case.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t benchmark_pktLibrary(void)
{
    Ti_Pkt*             pkt0;
    Ti_Pkt*             pkt1;
    Ti_Pkt*             pkt2;
    Ti_Pkt*             pkt3;
    Ti_Pkt*             pktHeadRoom;
    Ti_Pkt*             pktTailRoom;
    Ti_Pkt*             pktMergedPacket;
    uint32_t            startTime;
    uint32_t            endTime;
    uint32_t            adjustment;
    Pktlib_HeapStats    startStats;
    Pktlib_HeapStats    endStats;
    uint32_t            index;
    uint8_t*            ptrDataBuffer;
    uint32_t            dataLen;

    startTime  = TSCL;
    endTime    = TSCL;
    adjustment = endTime - startTime;
    System_printf ("Debug: Adjustment: %d\n", adjustment);

    /* Get the current heap statistics. */
    startTime  = TSCL;
    Pktlib_getHeapStats(myHeap, &startStats);
    endTime    = TSCL;
    System_printf ("Debug: Get Heap Stats took %d ticks\n", (endTime - startTime));

    /*****************************************************************
     ********************** PACKET LIBRARY Basics ********************
     *****************************************************************/

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Benchmarking the Packet Library Basic Operation\n");

    /* Allocate a packet. */
    startTime  = TSCL;
    pkt0      = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 600);
    endTime    = TSCL;
    System_printf ("Debug: Packet Allocation took %d ticks\n", (endTime - startTime));

    /* Set the packet length associated with the packet. */
    startTime  = TSCL;
    Pktlib_setPacketLen (pkt0, 100);
    endTime    = TSCL;
    System_printf ("Debug: Setting Packet Length took %d ticks\n", (endTime - startTime));

    /* Get the packet length associated with the packet. */
    startTime  = TSCL;
    dataLen    = Pktlib_getPacketLen (pkt0);
    endTime    = TSCL;
    System_printf ("Debug: Getting Packet Length %d bytes took %d ticks\n", dataLen, (endTime - startTime));

    /* Get the data buffer and length associated with the packet. */
    startTime  = TSCL;
    Pktlib_getDataBuffer (pkt0, &ptrDataBuffer, &dataLen);
    endTime    = TSCL;
    System_printf ("Debug: Packet Data Buffer 0x%p & Length %d bytes took %d ticks\n",
                    ptrDataBuffer, dataLen, (endTime - startTime));

    /* Cleanup the packet. */
    startTime  = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt0);
    endTime    = TSCL;
    System_printf ("Debug: Packet Cleanup took %d ticks\n", (endTime - startTime));

    /*****************************************************************
     ********************** PACKET LIBRARY MERGE *********************
     *****************************************************************/

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Benchmarking the Packet Library Merge Operation\n");

    /* Allocate a 200 byte packet from the heap. */
    startTime = TSCL;
    pkt0      = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 200);
    endTime   = TSCL;
    if (pkt0 == NULL)
        return -1;
    System_printf ("Debug: Allocating Pkt0 of 200 byte took %d ticks\n", (endTime - startTime));

    /* Allocate a 500 byte packet from the heap. */
    startTime = TSCL;
    pkt1      = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 500);
    endTime   = TSCL;
    if (pkt1 == NULL)
        return -1;
    System_printf ("Debug: Allocating Pkt1 of 500 byte Packet took %d ticks\n", (endTime - startTime));

    /* Merge the packets together. */
    startTime = TSCL;
    pktMergedPacket = Pktlib_packetMerge(appPktlibInstanceHandle, pkt0, pkt1, NULL);
    endTime   = TSCL;
    System_printf ("Debug: Merging Pkt0 and Pkt1 took %d ticks\n", (endTime - startTime));

    /* Allocate a 16 byte packet from the heap. */
    startTime    = TSCL;
    pktHeadRoom  = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 16);
    endTime      = TSCL;
    if (pktHeadRoom == NULL)
        return -1;
    System_printf ("Debug: Allocating Headroom 16 byte Packet took %d ticks\n", (endTime - startTime));

    /* Add the Headroom Packet */
    startTime = TSCL;
    pktMergedPacket = Pktlib_packetMerge(appPktlibInstanceHandle, pktHeadRoom, pktMergedPacket, NULL);
    endTime   = TSCL;
    System_printf ("Debug: Adding Headroom to Merged Packet took %d ticks\n", (endTime - startTime));

    /* Allocate a 400 byte packet from the heap and add this to the tail of the packet.*/
    startTime    = TSCL;
    pktTailRoom  = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 400);
    endTime      = TSCL;
    if (pktTailRoom == NULL)
        return -1;
    System_printf ("Debug: Allocating Tailroom 400 byte Packet took %d ticks\n", (endTime - startTime));

    /* Debug Message: */
    System_printf ("Debug: Adding Tailroom packet to existing packet with %d buffers\n",
            Pktlib_packetBufferCount(pktMergedPacket));

    /* Add the Tailroom Packet */
    startTime = TSCL;
    pktMergedPacket = Pktlib_packetMerge(appPktlibInstanceHandle, pktMergedPacket, pktTailRoom, NULL);
    endTime   = TSCL;
    System_printf ("Debug: Adding Tailroom to Merged Packet took %d ticks\n", (endTime - startTime));

    /* Debug Message: Print out the number of buffers which make the Merged packet. */
    System_printf ("Debug: Merged Packet has %d buffers\n", Pktlib_packetBufferCount(pktMergedPacket));

    /* Free the packets up. */
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pktMergedPacket);
    endTime   = TSCL;
    System_printf ("Debug: Free Merged Packet took %d ticks\n", (endTime - startTime));

    /* Get the heap stats and make sure there are no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /*****************************************************************
     ********************** PACKET LIBRARY CLONE *********************
     *****************************************************************/

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Benchmarking the Packet Library Clone Operation\n");

    /* Allocate a 800 byte packet from the heap. */
    startTime = TSCL;
    pkt0      = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 800);
    endTime   = TSCL;
    if (pkt0 == NULL)
        return -1;
    System_printf ("Debug: Allocating Pkt0 of 800 byte took %d ticks\n", (endTime - startTime));

    /* Allocating a Zero Buffer packet for cloning. */
    startTime = TSCL;
    pkt1      = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    endTime   = TSCL;
    if (pkt1 == NULL)
        return -1;
    System_printf ("Debug: Allocating 0 byte Packet took %d ticks\n", (endTime - startTime));

    /* Clone the packet. */
    System_printf ("Debug: Orignal Packet has %d buffers\n", Pktlib_packetBufferCount(pkt0));
    startTime = TSCL;
    Pktlib_clonePacket(appPktlibInstanceHandle, pkt0, pkt1);
    endTime   = TSCL;
    System_printf ("Debug: Cloning Packet took %d ticks\n", (endTime - startTime));

    /* Free the packets. */
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt0);
    endTime   = TSCL;
    System_printf ("Debug: Cleaning Orignal Packet took %d ticks\n", (endTime - startTime));

    /* Free the packets. */
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt1);
    endTime   = TSCL;
    System_printf ("Debug: Cleaning Cloned Packet took %d ticks\n", (endTime - startTime));

    /* Allocate multiple 800 byte orignal packet from the heap */
    pkt0 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 800);
    if (pkt0 == NULL)
        return -1;
    pkt1 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 800);
    if (pkt1 == NULL)
        return -1;

    /* Allocate multiple 0 byte packets for the clone. */
    pkt2 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);
    pkt3 = Pktlib_allocPacket(appPktlibInstanceHandle, myHeap, 0);

    /* Link the orignal and cloned packets. */
    Pktlib_packetMerge(appPktlibInstanceHandle, pkt0, pkt1, NULL);
    Pktlib_packetMerge(appPktlibInstanceHandle, pkt2, pkt3, NULL);

    /* Now start the cloning. */
    System_printf ("Debug: Orignal Packet has %d buffers\n", Pktlib_packetBufferCount(pkt0));
    startTime = TSCL;
    Pktlib_clonePacket(appPktlibInstanceHandle, pkt0, pkt2);
    endTime   = TSCL;
    System_printf ("Debug: Cloning Packet took %d ticks\n", (endTime - startTime));

    /* Free the orignal packets.*/
    System_printf ("Debug: Orignal Packet has %d buffers\n", Pktlib_packetBufferCount(pkt0));
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt0);
    endTime   = TSCL;
    System_printf ("Debug: Cleaning Orignal Packet took %d ticks\n", (endTime - startTime));

    /* Free the cloned packets */
    System_printf ("Debug: Cloned Packet has %d buffers\n", Pktlib_packetBufferCount(pkt2));
    startTime = TSCL;
    Pktlib_freePacket(appPktlibInstanceHandle, pkt2);
    endTime   = TSCL;
    System_printf ("Debug: Cleaning Cloned Packet took %d ticks\n", (endTime - startTime));

    /* Get the heap stats and make sure there are no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /*****************************************************************
     ********************** PACKET LIBRARY SPLIT *********************
     *****************************************************************/
    for (index = 1; index < 16; index++)
        benchmark_split(index);

    /* Get the heap stats and make sure there are no memory leaks. */
    Pktlib_getHeapStats(myHeap, &endStats);
    if ((endStats.numPacketsinGarbage != 0) || (endStats.numFreeDataPackets != startStats.numFreeDataPackets) ||
        (endStats.numZeroBufferPackets != startStats.numZeroBufferPackets))
        return -1;

    /* Benchmarking has been completed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test the multicore PKTLIB operations
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t test_multicore(void)
{
    Pktlib_HeapHandle       mySharedHeap;
    Ti_Pkt*                 ptrPkt;
    Ti_Pkt*                 ptrHeadPkt;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    int32_t                 index;
    Pktlib_HeapStats        startHeapStats;
    Pktlib_HeapStats        endHeapStats;
    Resmgr_ResourceCfg*     ptrResCfg;
    Qmss_QueueHnd           queueHandle;
    Name_ResourceCfg    	nameResourceCfg;
    uint8_t                 isAllocated;

    /* Debug Message: */
    System_printf ("------------------------------------------------------\n");
    System_printf ("Debug: Testing Multicore Heaps\n");

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "Multicore Shared Heap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 16;
    heapCfg.numZeroBufferPackets            = 16;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;

    /* Create the multicore shared heap with the specified configuration. */
    mySharedHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (mySharedHeap == NULL)
    {
        System_printf ("Error: Unable to create the shared multicore heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Get the heap statistics. */
    Pktlib_getHeapStats(mySharedHeap, &startHeapStats);

    /* Allocate the head packet. */
    ptrHeadPkt = Pktlib_allocPacket(appPktlibInstanceHandle, mySharedHeap, 128);
    if (ptrHeadPkt == NULL)
    {
        System_printf ("Error: Unable to allocate the head packet from the shared heap\n");
        return -1;
    }

    /* Allocate packets from the heaps and chain them together. */
    for (index = 1; index < heapCfg.numPkts; index++)
    {
        /* Allocate packet */
        ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, mySharedHeap, 128);
        if (ptrPkt == NULL)
        {
            System_printf ("Error: Unable to allocate a packet from the shared heap [Index %d]\n", index);
            return -1;
        }

        /* Chain the packets together. */
        Pktlib_packetMerge(appPktlibInstanceHandle, ptrHeadPkt, ptrPkt, NULL);
    }

    /* Ensure that we have cleaned up all the data buffer packets from the heap  */
    Pktlib_getHeapStats(mySharedHeap, &endHeapStats);
    if (endHeapStats.numFreeDataPackets != 0)
    {
        System_printf ("Error: Multicore Shared Heap has %d packets pending in the heap\n", endHeapStats.numFreeDataPackets);
        return -1;
    }

    /* Open a queue to exchange packets between the cores. */
    queueHandle = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                 QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (queueHandle < 0)
    {
        System_printf ("Error: Unable to open the queue for the multicore tests\n");
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug: Sending a chained packet with %d packets \n", Pktlib_packetBufferCount(ptrHeadPkt));

    /* Release the ownership */
    Pktlib_releaseOwnership(appPktlibInstanceHandle, ptrHeadPkt);

    /* Push the packet into the queue */
    Qmss_queuePushDesc (queueHandle, (void*)ptrHeadPkt);

    /* Use named resources to push the queue handle between the cores. */
    nameResourceCfg.handle1  = (uint32_t)queueHandle;
    strncpy(nameResourceCfg.name, "PKTLIB Unit Test", NAME_MAX_CHAR);

    /* Create the queue information. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF4, &nameResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to create the named resource [Error code %d]\n", errCode);
        return -1;
    }

    /* Loop around waiting for the cores to finish the test */
    while (1)
    {
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF4,
        					   "PKTLIB Unit Test", &nameResourceCfg, &errCode) < 0)
        {
            System_printf ("Error: PKTLIB Unit Test find named resource failed [Error code %d]\n", errCode);
            return -1;
        }

        if (nameResourceCfg.handle2 != 1)
        {
            Task_sleep(10);
            continue;
        }
        break;
    }

    /* Get the heap statistics. */
    Pktlib_getHeapStats(mySharedHeap, &endHeapStats);

    /* Ensure that there are no more memory leaks. */
    if ((endHeapStats.numPacketsinGarbage != 0) || (endHeapStats.numFreeDataPackets != startHeapStats.numFreeDataPackets) ||
        (endHeapStats.numZeroBufferPackets != startHeapStats.numZeroBufferPackets))
    {
        System_printf ("Error: Memory leak detected\n");
        return -1;
    }

    /* Shutdown the shared heap */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, mySharedHeap, &errCode) < 0)
    {
        System_printf ("Error: Deleting the shared multicore heap failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Multicore test on Core %d passed\n", DNUM);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      PKTLIB Test Task
 *
 *  @retval
 *      Not Applicable.
 */
void PktlibTestTask(UArg arg0, UArg arg1)
{
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Resmgr_ResourceCfg*     ptrResCfg;
    Pktlib_InstCfg          pktlibInstCfg;

    /* Get the resource manager configuration. */
    ptrResCfg = &appResourceConfig;

    /* Execute the tests: */
    System_printf ("************************************************************************\n");
    System_printf ("Debug: Executing the PKTLIB Unit Tests.\n");
    System_printf ("************************************************************************\n");
            
    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId     = 1;
    pktlibInstCfg.databaseHandle = globalNameDatabaseHandle;
    pktlibInstCfg.sysCfgHandle   = handleSysCfg;
    pktlibInstCfg.malloc         = Pktlib_osalMalloc;
    pktlibInstCfg.free           = Pktlib_osalFree;
    pktlibInstCfg.beginMemAccess = Pktlib_osalBeginMemAccess;
    pktlibInstCfg.endMemAccess   = Pktlib_osalEndMemAccess;
    pktlibInstCfg.beginPktAccess = Pktlib_osalBeginPktAccess;
    pktlibInstCfg.endPktAccess   = Pktlib_osalEndPktAccess;
    pktlibInstCfg.enterCS        = Pktlib_osalEnterCS;
    pktlibInstCfg.exitCS         = Pktlib_osalExitCS;

    /* Create the PKTLIB instance */
    appPktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, &errCode);
    if (appPktlibInstanceHandle == NULL)
    {
        System_printf ("Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
        return;
    }

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "My Test Heap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1024;
    heapCfg.numPkts                         = 1024;
    heapCfg.numZeroBufferPackets            = 1024;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0xbeefbeef;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;

    /* Create the Local Heap with specified configuration. */
    myHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (myHeap == NULL)
    {
        System_printf ("Error: Unable to create the heap error code %d\n", errCode);
        flushSysMinToArm();
        return;
    }
    System_printf ("Debug: Heap %p has been created successfully\n", myHeap);

    /* Test the packet Library API */
    if (test_pktLibrary() < 0)
    {
        System_printf ("Error: PACKET Library Unit Testing FAILED\n");
        flushSysMinToArm();
        return;
    }

    /* Benchmark the Packet Library with a use-case. */
    if (benchmark_pktLibrary() < 0)
    {
        System_printf ("Error: BENCHMARKING Packet Library FAILED\n");
        flushSysMinToArm();
        return;
    }

    /* Test the Multicore PKTLIB operations: */
    if (test_multicore() < 0)
    {
        System_printf ("Error: Multicore Packet Library test FAILED\n");
        flushSysMinToArm();
        return;
    }

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (appPktlibInstanceHandle, &errCode) < 0)
    {
        System_printf ("Error: PKLIB Instance deletion failed [Error code %d]\n", errCode);
        flushSysMinToArm();       
        return;
    }

    /* Execute the tests: */
    System_printf ("************************************************************************\n");
    System_printf ("Debug: Unit Test complete\n");
    System_printf ("************************************************************************\n");
    flushSysMinToArm();
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_sysInitTask(UArg arg0, UArg arg1)
{
    Task_Params         taskParams;

    /* Initialize the system modules */
	if (system_init() < 0)
        flushSysMinToArm();
	    return;

    /* Allow the system initialization to be done before we start the tests */
    Task_sleep(10);

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    Task_create(PktlibTestTask, &taskParams, NULL);
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
void main (void)
{
    Task_Params         taskParams;

    /* Enable the timestamp counter */
    TSCL = 0;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);
	CACHE_setL2Size (CACHE_512KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 64*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

