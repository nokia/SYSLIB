/**
 *   @file  main.c
 *
 *   @brief
 *      Unit Test Code for the UINTC
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
#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

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

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 *********************** Local Definitions ****************************
 **********************************************************************/

/* This is the ARM System Event & Queue which is being tested in the
 * UINTC unit test. These values are device specific */
#ifdef DEVICE_K2H
#define TEST_ARM_SYSTEM_EVENT       79
#define TEST_ARM_QUEUE_PEND         (QMSS_GIC400_QUEUE_BASE + QMSS_MAX_GIC400_QUEUE - 1)
#elif defined(DEVICE_K2K)
#define TEST_ARM_SYSTEM_EVENT       79
#define TEST_ARM_QUEUE_PEND         (QMSS_GIC400_QUEUE_BASE + QMSS_MAX_GIC400_QUEUE - 1)
#elif defined(DEVICE_K2L)
#define TEST_ARM_SYSTEM_EVENT       79
#define TEST_ARM_QUEUE_PEND         (QMSS_ARM_GIC_QUEUE_BASE + QMSS_MAX_ARM_GIC_QUEUE - 1)
#else
#error "Error: Unsupported Device"
#endif

/* Test Message Id: */
#define TEST_MESSAGE_ID              0x1234

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
extern void* Pktlib_osalPhyToVirt(void* );

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Global PKTLIB Heap Handle: */
Pktlib_HeapHandle       testPktHeap;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Name Database Handle: */
Name_DBHandle           globalNameDatabaseHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* User space interrupt handle block */
UintcHandle             uintcHandle;

/* Test Execution status */
uint32_t                testCompletionStatus = -1;

/* ISR Counter */
volatile uint32_t       isrCounter = 0;
volatile uint32_t       messageIdCounter = 0;

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
		{ "ARM-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_INTERNAL,   256,    128 },
    }
};

/**********************************************************************
 ************************* Unit Test Functions ************************
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
 *      The function is used to create and initialize the PKTLIB heaps.
 *
 *  @param[in]  ptrCfg
 *      Pointer to the resource manager configuration block.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_createPktlibHeaps(Resmgr_ResourceCfg* ptrCfg)
{
    Pktlib_HeapCfg        heapCfg;
    int32_t               errCode;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets */
    strcpy(heapCfg.name, "Test_Heap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    testPktHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (testPktHeap == NULL)
    {
	    printf ("Error: Unable to create test heap, error code %d\n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered ISR Handler:
 *
 *  @retval
 *      Not Applicable.
 */
static void myIsrHandler(void* arg)
{
    uint32_t    event = (uint32_t)arg;

    /* Sanity Check: Validate the argument. */
    if (event != TEST_ARM_SYSTEM_EVENT)
    {
        printf ("Error: UINTC argument validation failed [Got %d]\n", event);
        return;
    }

    /* Increment the ISR counter. */
    isrCounter++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered Message Id Handler:
 *
 *  @retval
 *      Not Applicable.
 */
static void myMessageIdHandler(void* arg)
{
    uint32_t event = (uint32_t)arg;

    /* Sanity Check: Validate the argument. */
    if (event != TEST_MESSAGE_ID)
    {
        printf ("Error: UINTC argument validation failed [Got %d]\n", event);
        return;
    }

    /* Increment the message id counter. */
    messageIdCounter++;
    return;
}


/**
 *  @b Description
 *  @n
 *      Test Thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* TestThread(void* arg)
{
    int32_t             errCode;
    Qmss_QueueHnd       queueHandle;
    Ti_Pkt*				ptrPkt;
    struct sched_param  param;

    /* Set the configured policy and priority to be lower than what has been
     * configured in the UINTC thread. */
    param.sched_priority = 3;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /******************************************************************************
     * Test: Register interrupt
     ******************************************************************************/

    /* Register interrupts using the UINTC module. */
    if (Uintc_registerIsr (uintcHandle, TEST_ARM_SYSTEM_EVENT, myIsrHandler, (void*)TEST_ARM_SYSTEM_EVENT, &errCode) < 0)
    {
        printf ("Error: Unable to register the interrupt [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Interrupt registered successfully\n");

    /******************************************************************************
     * Test: Deregister interrupt
     ******************************************************************************/

    /* Register interrupts using the UINTC module. */
    if (Uintc_deregisterIsr (uintcHandle, TEST_ARM_SYSTEM_EVENT, &errCode) < 0)
    {
        printf ("Error: Unable to deregister the interrupt [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Interrupt deregistered successfully\n");

    /******************************************************************************
     * Test: Invalid interrupt registeration
     ******************************************************************************/

    /* Register invalid system event using the UINTC module. */
    if (Uintc_registerIsr (uintcHandle, 435, myIsrHandler, (void*)435, &errCode) == 0)
    {
        printf ("Error: Invalid interrupt registeration was succesful\n");
        return NULL;
    }
    /* Validate the error code */
    if (errCode != UINTC_EDTSCFG)
    {
        printf ("Error: Invalid interrupt registeration failed but error code was incorrect [%d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Invalid interrupt registeration test passed\n");

    /******************************************************************************
     * Test: Duplicate interrupt
     ******************************************************************************/

    /* Register interrupts using the UINTC module. */
    if (Uintc_registerIsr (uintcHandle, TEST_ARM_SYSTEM_EVENT, myIsrHandler, (void*)TEST_ARM_SYSTEM_EVENT, &errCode) < 0)
    {
        printf ("Error: Unable to register the interrupt [Error code %d]\n", errCode);
        return NULL;
    }

    /* Register the same interrupts again with the UINTC module.*/
    if (Uintc_registerIsr (uintcHandle, TEST_ARM_SYSTEM_EVENT, myIsrHandler, (void*)TEST_ARM_SYSTEM_EVENT, &errCode) == 0)
    {
        printf ("Error: Duplicate interrupt registeration was completed successfully\n");
        return NULL;
    }
    /* Validate the error code */
    if (errCode != UINTC_EDUP)
    {
        printf ("Error: Duplicate interrupt registeration failed but error code was incorrect [%d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Duplicate interrupt registeration test passed\n");

    /******************************************************************************
     * Test: Interrupt generation test
     ******************************************************************************/

    /* Open the Test Queue */
    queueHandle = (Qmss_QueueHnd)TEST_ARM_QUEUE_PEND;

    /* Ensure that the queue is empty */
    Qmss_queueEmpty (queueHandle);

    /* Allocate a packet from the heap */
    ptrPkt = Pktlib_allocPacket (appPktlibInstanceHandle, testPktHeap, 128);
    if (ptrPkt == NULL)
	{
	    printf ("Error: Unable to allocate the packet from the test heap\n");
		return NULL;
    }

    /* Sanity Check: Ensure that the ISR counter is 0 */
    if (isrCounter != 0)
    {
	    printf ("Error: Spurious interrupt detected [%d]\n", isrCounter);
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
		return NULL;
    }

    /* Push the packet to the QMSS Queue */
    Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);

    /* We need to allow some time for the interrupt to be fired and pass through the UIO drivers and then
     * eventually up the UINTC module. */
    usleep(100);

    /* Once the packet has been pushed. An interrupt should have been generated. The ISR
     * counter should have gone up. */
    if (isrCounter != 1)
    {
	    printf ("Error: Interrupt not detected [%d]\n", isrCounter);
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
		return NULL;
    }
    printf ("Debug: UINTC Application ISR test passed\n");

    /* Remove the packet from the queue. No more interrupts will be generated. */
    ptrPkt = Qmss_queuePop (queueHandle);

    /* Enable system event in the UINTC module. This is because the first interrupt
     * would have disabled the interrupts. */
    if (Uintc_enableEvent (uintcHandle, TEST_ARM_SYSTEM_EVENT, &errCode) < 0)
    {
	    printf ("Error: Enable events failed [%d]\n", errCode);
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
		return NULL;
    }

    /* Reset the ISR counter */
    isrCounter = 0;

    /* Push the packet to the QMSS Queue */
    Qmss_queuePushDesc (queueHandle, (void*)ptrPkt);

    /* We need to allow some time for the interrupt to be fired and pass through the UIO drivers and then
     * eventually up the UINTC module. */
    usleep(100);

    /* Once the packet has been pushed. An interrupt should have been generated. The ISR
     * counter should have gone up. */
    if (isrCounter != 1)
    {
	    printf ("Error: Interrupt not detected [%d]\n", isrCounter);
        Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
		return NULL;
    }
    printf ("Debug: Multiple UINTC ISR test passed\n");

    /* Pop off the packet from the queue */
    ptrPkt = Qmss_queuePop(queueHandle);

    /* Cleanup the allocated packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);

    /******************************************************************************
     * Test: Message Id Test
     ******************************************************************************/

    /* Register message identifier using the UINTC module. */
    if (Uintc_registerMessageId (uintcHandle, TEST_MESSAGE_ID, myMessageIdHandler, (void*)TEST_MESSAGE_ID, &errCode) < 0)
    {
        printf ("Error: Unable to register the message id [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Message Id registered successfully\n");

    /******************************************************************************
     * Test: Deregister message id
     ******************************************************************************/

    /* Register message identifier using the UINTC module. */
    if (Uintc_deregisterMessageId (uintcHandle, TEST_MESSAGE_ID, &errCode) < 0)
    {
        printf ("Error: Unable to deregister the message id [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Message Id deregistered successfully\n");

    /******************************************************************************
     * Test: Duplicate message id registeration
     ******************************************************************************/

    /* Register message id using the UINTC module. */
    if (Uintc_registerMessageId (uintcHandle, TEST_MESSAGE_ID, myMessageIdHandler, (void*)TEST_MESSAGE_ID, &errCode) < 0)
    {
        printf ("Error: Unable to register the message id [Error code %d]\n", errCode);
        return NULL;
    }

    /* Register the same message id again with the UINTC module.*/
    if (Uintc_registerMessageId (uintcHandle, TEST_MESSAGE_ID, myMessageIdHandler, (void*)TEST_MESSAGE_ID, &errCode) == 0)
    {
        printf ("Error: Duplicate message id registeration was completed successfully\n");
        return NULL;
    }

    /* Validate the error code */
    if (errCode != UINTC_EDUP)
    {
        printf ("Error: Duplicate message id registeration failed but error code was incorrect [%d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Duplicate message id registeration test passed\n");

    /******************************************************************************
     * Test: Message Id generation test
     ******************************************************************************/

    /* Reset the ISR counter */
    messageIdCounter = 0;

    /* Send a message to the UINTC module */
    if (Uintc_sendMessageId (uintcHandle, TEST_MESSAGE_ID, &errCode) < 0)
    {
        printf ("Error: Unable to send the message id [Error code %d]\n", errCode);
        return NULL;
    }

    /* We need to allow some time for the message id to be received and processed by the UINTC Thread */
    usleep(100);

    /* Once the packet has been pushed. An interrupt should have been generated. The ISR
     * counter should have gone up. */
    if (messageIdCounter != 1)
    {
	    printf ("Error: Message Id not detected [%d]\n", messageIdCounter);
		return NULL;
    }
    printf ("Debug: Message id generation test passed\n");

    /******************************************************************************
     * Test: Invalid Message Id generation test
     ******************************************************************************/

    /* Reset the ISR counter */
    messageIdCounter = 0;

    /* Send a message to the UINTC module */
    if (Uintc_sendMessageId (uintcHandle, TEST_MESSAGE_ID + 1, &errCode) < 0)
    {
        printf ("Error: Unable to send the message id [Error code %d]\n", errCode);
        return NULL;
    }

    /* We need to allow some time for the message id to be received and processed by the UINTC Thread */
    usleep(100);

    /* Once the packet has been pushed. An interrupt should have been generated. The ISR
     * counter should have gone up. */
    if (messageIdCounter != 0)
    {
	    printf ("Error: Invalid Message Id detected [%d]\n", messageIdCounter);
		return NULL;
    }
    printf ("Debug: Invalid Message id generation test passed\n");

    /* Test was completed successfully. */
    testCompletionStatus = 0;
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      UINTC thread which waits and processes all the interrupts which have
 *      been registered by the UINTC instance.
 *
 *  @param[in]  arg
 *      Not used
 *
 *  @retval
 *      Always returns NULL.
 */
static void* UintcThread(void* arg)
{
    int32_t             errCode;
    int32_t             retVal;
    struct sched_param  param;

    /* Set the configured policy and priority */
    param.sched_priority = 5;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Loop around: The UINTC thread is waiting for an interrupt to arrive
     * on events which have been registered with the UINTC instance. */
    while (1)
    {
        /* Dispatch received events to the appropriate handler. */
        retVal = Uintc_select (uintcHandle, NULL, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;

            /* Report the UINTC Module error code: */
            printf ("Error: UINTC select failed [Error code %d]\n", errCode);
            return NULL;
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main(void)
{
    Resmgr_ResourceCfg*         ptrResCfg;
    pthread_t                   testThread;
    pthread_t                   uintcThread;
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    UintcConfig                 uintcConfig;
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

    /* Get the application specific resource configuration. */
    ptrResCfg = &appResourceConfig;

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

    /* Create the heaps which are required in the tests. */
    if (Test_createPktlibHeaps (ptrResCfg) < 0)
        return -1;

    /* Initialize and populate the user space interrupt configuration */
    memset ((void *)&uintcConfig, 0, sizeof(UintcConfig));

    /* Populate the user space interrupt configuration: */
    strcpy(uintcConfig.name, "TestUINTC");
    uintcConfig.mode = Uintc_Mode_UINTC_MANAGED;

    /* Initialize the user space interrupt module */
    uintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (uintcHandle == NULL)
    {
        printf ("Error: Unable to open the UINTC module\n");
        return -1;
    }
    printf ("Debug: UINTC module has been opened successfully.\n");

    /* Create the test thread. */
    errCode = pthread_create (&testThread, NULL, TestThread, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the UINTC thread. */
    errCode = pthread_create (&uintcThread, NULL, UintcThread, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Blocked till the test thread is done. */
    pthread_join (testThread,  NULL);

    /* Did the unit test work? */
    printf ("*****************************************\n");
    if (testCompletionStatus == -1)
        printf ("Error: UINTC module unit tests failed\n");
    else
        printf ("Debug: UINTC module unit tests passed\n");
    printf ("*****************************************\n");

    /* Shutdown and close the UINTC module */
    if (Uintc_deinit (uintcHandle, &errCode) < 0)
    {
        printf ("Error: Shutting down the UINTC module failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: Shutting down the UINTC module successful\n");

    /* Ensure that the UINTC thread is also dead. */
    pthread_join (uintcThread, NULL);

    /* Shutdown the test heap */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, testPktHeap, &errCode) < 0)
        printf ("Error: PKTLIB Test heap deletion failed [Error code %d]\n", errCode);

    /* Close the database */
    if (Name_deleteDatabase (globalNameDatabaseHandle, &errCode) < 0)
        printf ("Error: Deleting the database failed [Error code %d]\n", errCode);
    else
        printf ("Debug: Database deleted successfully\n");

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");

    /* Test execution complete. */
    return 0;
}

