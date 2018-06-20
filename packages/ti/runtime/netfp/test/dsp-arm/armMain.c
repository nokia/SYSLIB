/**
 *   @file  armMain.c
 *
 *   @brief
 *      Unit Test Code for the NETFP. This is the ARM client which attaches
 *      itself to the NETFP server.
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

/************************************************************************n
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* Device specific dependencies: */
#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#elif defined (DEVICE_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#endif

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 *********************** Extern Declarations **************************
 **********************************************************************/

/* Unit Tests: */
extern void* Test_socketThread(void* arg);
extern void* Test_socket6Thread(void* arg);
extern void* Test_wildCardingThread(void* arg);
extern void* Test_wildCarding6Thread(void* arg);
extern void Test_clearBenchmarkIPv4(void);
extern void Test_clearBenchmarkIPv6(void);

/**********************************************************************
 ************************ Extern APIs *********************************
 **********************************************************************/
extern hplib_RetValue hplib_utilInitProc();
extern pid_t gettid( void );

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

/* MSGCOM */
extern void*   Msgcom_osalMalloc(Msgcom_MemAllocMode , uint32_t );
extern void    Msgcom_osalFree(Msgcom_MemAllocMode , void* , uint32_t );
extern int32_t Msgcom_osalRegisterIsr(const char* , Qmss_Queue , MsgCom_Isr , MsgCom_ChHandle, MsgCom_Interrupt*);
extern int32_t Msgcom_osalDeregisterIsr(const char* , Qmss_Queue ,MsgCom_Interrupt*);
extern void    Msgcom_osalDisableSysInt(int32_t , int32_t );
extern void    Msgcom_osalEnableSysInt(int32_t , int32_t );
extern void*   Msgcom_osalEnterSingleCoreCS(void);
extern void    Msgcom_osalExitSingleCoreCS(void* );
extern void*   Msgcom_osalCreateSem(void);
extern void    Msgcom_osalDeleteSem(void* );
extern void    Msgcom_osalPendSem(void* );
extern void    Msgcom_osalPostSem(void* );

/* NETFP: */
extern void* Netfp_osalMalloc (uint32_t , uint32_t );
extern void  Netfp_osalFree (void* , uint32_t );
extern void* Netfp_osalEnterSingleCoreCriticalSection (void);
extern void  Netfp_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Netfp_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Netfp_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Netfp_osalCreateSem(void);
extern void  Netfp_osalDeleteSem(void*);
extern void  Netfp_osalPostSem(void*);
extern void  Netfp_osalPendSem(void*);

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Global PKTLIB Heap Handle: */
Pktlib_HeapHandle   netfpDataRxHeap;
Pktlib_HeapHandle   netfpHeaderHeap;
Pktlib_HeapHandle   netfpFragHeap;
Pktlib_HeapHandle 	agentReceiveHeap;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global variable for the database */
Name_DBHandle           globalNameDatabaseHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global variable for the application MSGCOM instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* User space interrupt handle block */
UintcHandle             uintcHandle;

/* NETFP Client Handle. */
Netfp_ClientHandle      netfpClientHandle;

/* Test Execution status */
uint32_t                testComplete = 0;

/* Port Number to be configured */
uint32_t                gPortNumber;

/* NETFP Server Name used by the test */
const char*             gNetfpServerName = "NetfpServer_LTE9A";

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
	0,    /* Number of CPINTC Output  requested                               */
	0,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
	1,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                    Linking RAM,                           Num,     Size */
		{ "ARM-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  4096,    128},
		{ "ARM-DDR3-1", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  512,     128},
    }
};

/**********************************************************************
 ************************* Unit Test Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Displays the usage of the test application
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_displayUsage (char* argv[])
{
    printf ("Error: Invalid usage\n");
    printf ("%s <instnum> <port number> <advanced benchmark>\n", argv[0]);
    printf ("<instnum> Instance number in sync with the values in the clients.txt\n");
    printf ("Port Number configured to 0 will delete the interface\n");
    printf ("Range is from 1 to 65535\n");
    printf ("<Advanced benchmark> will run the test thread on isolated core and with higher priority.\n");
    printf ("If advanced benmark is enabled, please put core 3 as an isolated core in Uboot - isolcpus=3 \n");
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGINT
 *
 *  @param[in]  signo
 *      Signal Number
 *  @param[in]  siginfo
 *      Signal Information
 *  @param[in]  context
 *      Context information.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_terminated (int sig, siginfo_t *siginfo, void *context)
{
    /* Test is being terminated */
    testComplete = 1;
    return;
}

/**
 *  @b Description
 *  @n
 *      Signal Handler installed to catch the SIGUSER0
 *
 *  @param[in]  signo
 *      Signal Number
 *  @param[in]  siginfo
 *      Signal Information
 *  @param[in]  context
 *      Context information.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_benchmarkClear (int sig, siginfo_t *siginfo, void *context)
{
    /* Clear benchmark data for IPv4 */
    Test_clearBenchmarkIPv4();

    /* Clear benchmark data for IPv6 */
    Test_clearBenchmarkIPv6();

    printf("Benchmark data is cleared!\n");
    return;
}

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
 *  @param[in]  instNum
 *      Instance Number
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_createPktlibHeaps(Resmgr_ResourceCfg* ptrCfg, uint32_t instNum)
{
    Pktlib_HeapCfg        heapCfg;
    int32_t               errCode;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets and is configured
     *    to have data buffers of 10K */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "NetFP_PktRxHeap_%d", instNum);
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 10*1024;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    netfpDataRxHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpDataRxHeap == NULL)
    {
	    printf ("Error: Unable to create shared heap, error code %d\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "NETFP_HeaderHeap_%d", instNum);
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 50;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    netfpHeaderHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpHeaderHeap == NULL)
    {
	    printf ("Error: Unable to create NETFP Header Heap\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - This heap is used to send fragmented packets provided the fragmentation is done
     *    in NETFP software. */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "NETFP_FragmentHeap_%d", instNum);
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 0;
    heapCfg.numPkts                         = 0;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    netfpFragHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpFragHeap == NULL)
    {
	    printf ("Error: Unable to create fragmentation Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets exchanged between
     *    the agent server and client. */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "Agent_Heap_%d", instNum);
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    agentReceiveHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (agentReceiveHeap == NULL)
    {
	    printf ("Error: Unable to create the agent heap, error code %d\n", errCode);
        return -1;
    }

    /* Debug: Dump the Queue Information for all the heaps. */
    printf ("Debug: NETFP Header Heap -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(netfpHeaderHeap));
    printf ("Debug: NETFP Data Heap   -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(netfpDataRxHeap));
    printf ("Debug: Agent Data Heap   -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(agentReceiveHeap));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      NETFP
 *
 *  @retval
 *      Not Applicable.
 */
static void* ClientExecutionTask(void* arg)
{
    /* Execute the NETFP clients till the tests complete. */
    while (1)
        Netfp_executeClient (netfpClientHandle);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the UINTC thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* UintcThread(void *arg)
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

    /* Debug Message: */
    printf("Debug: UINTC execution thread started\n");

    /* Loop around: The UINTC thread is waiting for an interrupt to arrive on events which have been registered
     * with the UINTC instance. */
    while (1)
    {
        /* Dispatch received events to the appropriate handler. */
        retVal = Uintc_select (uintcHandle, NULL, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;

            /* Error: UINTC select failed */
            printf ("Error: UINTC select failed [Error code %d]\n", errCode);
            return NULL;
        }
    }
    return NULL;
}

#if 0
/**
 *  @b Description
 *  @n
 *      This is a thread to enable access to PMU counters
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* Test_enablePMUCounter(void* arg)
{
    cpu_set_t               cpu_set;
    uint32_t                core;
    struct sched_param      param;
    pthread_attr_t          attr;

    /* Get thread sched param */
    pthread_getattr_np(pthread_self(), &attr);

    /* Change thread priority*/
    param.sched_priority = 60;
    pthread_attr_setschedparam(&attr, &param);

    /* Enable access to PMU counter on all cores */
    for(core = 0; core < 4; core++)
    {
        CPU_ZERO(&cpu_set);
        CPU_SET(core, &cpu_set);

        /* Set thread affinity */
        if (sched_setaffinity( gettid(), sizeof( cpu_set_t ), &cpu_set ))
        {
            printf("Error: sched_setaffinity() failed\n");
            return NULL;
        }

        /* Call hplib util function */
        if (hplib_utilInitProc() != hplib_OK)
        {
            printf("Error: hplib_utilInitProc() failed\n");
        }
        printf("Debug: hplib_utilInitProc() succeeded on core %d !\n", core);
    }

    return NULL;
}
#endif

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (int32_t argc, char* argv[])
{
    Resmgr_ResourceCfg*         ptrResCfg;
    pthread_t                   testThread;
    pthread_t                   netfpClientThread;
    pthread_t                   uintcThread;
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    struct sigaction            act;
    Netfp_ClientConfig          clientConfig;
    int32_t                     clientStatus;
    Netfp_ServerHandle          netfpServerHandle;
    uint32_t                    instNum;
    UintcConfig                 uintcConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    uint32_t                    advBenchmark=0;

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

    /* Validate the number of arguments. */
    if (argc != 4)
    {
        Test_displayUsage(argv);
        return -1;
    }

    /* Get the instance number */
    instNum = atoi (argv[1]);

    /* Get the port number */
    gPortNumber = atoi (argv[2]);
    printf ("Debug: Using port number %d\n", gPortNumber);
    if (gPortNumber > 65535)
    {
        Test_displayUsage(argv);
        return -1;
    }

    /* Get settings of benchmark */
    advBenchmark = atoi(argv[3]);

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

    /* Get the application resource configuration. */
    ptrResCfg = &appResourceConfig;

    /* Use the sa_sigaction field because the handles has two additional parameters */
    act.sa_sigaction = &Test_terminated;
    act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

    /* Setup the signal to clear benchmark data. */
    act.sa_sigaction = &Test_benchmarkClear;
    act.sa_flags     = SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);

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
    if (Test_createPktlibHeaps (ptrResCfg, instNum) < 0)
        return -1;

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = globalNameDatabaseHandle;
    msgcomInstCfg.sysCfgHandle      = handleSysCfg;
    msgcomInstCfg.pktlibInstHandle  = appPktlibInstanceHandle;
    msgcomInstCfg.malloc            = Msgcom_osalMalloc;
    msgcomInstCfg.free              = Msgcom_osalFree;
    msgcomInstCfg.registerIsr       = Msgcom_osalRegisterIsr;
    msgcomInstCfg.deregisterIsr     = Msgcom_osalDeregisterIsr;
    msgcomInstCfg.disableSysInt     = Msgcom_osalDisableSysInt;
    msgcomInstCfg.enableSysInt      = Msgcom_osalEnableSysInt;
    msgcomInstCfg.enterCS           = Msgcom_osalEnterSingleCoreCS;
    msgcomInstCfg.exitCS            = Msgcom_osalExitSingleCoreCS;
    msgcomInstCfg.createSem         = Msgcom_osalCreateSem;
    msgcomInstCfg.deleteSem         = Msgcom_osalDeleteSem;
    msgcomInstCfg.postSem           = Msgcom_osalPostSem;
    msgcomInstCfg.pendSem           = Msgcom_osalPendSem;

    /* Create the MSGCOM instance */
    appMsgcomInstanceHandle = Msgcom_createInstance (&msgcomInstCfg, &errCode);
    if (appMsgcomInstanceHandle == NULL)
    {
        printf ("Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize and populate the user space interrupt configuration */
    memset ((void *)&uintcConfig, 0, sizeof(UintcConfig));

    /* Populate the user space interrupt configuration: */
    snprintf(uintcConfig.name, 32, "TestUINTC_%d", instNum);
    uintcConfig.mode = Uintc_Mode_UINTC_MANAGED;
    uintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (uintcHandle == NULL)
    {
        printf ("Error: Unable to open the UINTC module\n");
        return -1;
    }
    printf ("Debug: UINTC module has been opened successfully.\n");

    /* SYNC Point: Ensure that the NETFP Server has been started. This is required before
     * we create and register the NETFP client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready. Since the NETFP Client and Server
         * are executing in the same realm. The NAME client handle is passed as NULL. */
        netfpServerHandle = Netfp_startServer (globalNameDatabaseHandle, NULL, (char*)gNetfpServerName, &errCode);
        if (netfpServerHandle != NULL)
            break;

        /* Check the error code. */
        if (errCode != NETFP_ENOTREADY)
        {
            printf ("Error: NETFP Starting server failed [Error code %d]\n", errCode);
            return -1;
        }
        usleep(1000*20);
    }

    /* Initialize the client configuration */
    memset ((void *)&clientConfig, 0, sizeof(Netfp_ClientConfig));

    /* Populate the client configuration: The NETFP server executes on the ARM
     * so there is no need for an agent client */
    strcpy (clientConfig.serverName, gNetfpServerName);
    sprintf (clientConfig.clientName, "Netfp_Client_LTE9A_TEST%d", instNum);
    clientConfig.serverHandle                       = netfpServerHandle;
    clientConfig.nrInstanceId                       = databaseCfg.instanceId;
    clientConfig.pktlibInstHandle                   = appPktlibInstanceHandle;
    clientConfig.msgcomInstHandle                   = appMsgcomInstanceHandle;
    clientConfig.directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    clientConfig.directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    clientConfig.directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    clientConfig.directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;
    clientConfig.nameClientHandle                   = NULL;
    clientConfig.netHeaderHeapHandle                = netfpHeaderHeap;
    clientConfig.fragmentHeap				        = netfpFragHeap;
    clientConfig.realm                              = Netfp_ExecutionRealm_ARM;
    clientConfig.clientHeapHandle                   = agentReceiveHeap;
    clientConfig.malloc                             = Netfp_osalMalloc;
    clientConfig.free                               = Netfp_osalFree;
    clientConfig.beginMemAccess                     = Netfp_osalBeginMemoryAccess;
    clientConfig.endMemAccess                       = Netfp_osalEndMemoryAccess;
    clientConfig.enterCS                            = Netfp_osalEnterSingleCoreCriticalSection;
    clientConfig.exitCS                             = Netfp_osalExitSingleCoreCriticalSection;
    clientConfig.createSem                          = Netfp_osalCreateSem;
    clientConfig.deleteSem                          = Netfp_osalDeleteSem;
    clientConfig.postSem                            = Netfp_osalPostSem;
    clientConfig.pendSem                            = Netfp_osalPendSem;

    /* Create the NETFP Client. */
    netfpClientHandle = Netfp_initClient (&clientConfig, &errCode);
    if (netfpClientHandle == NULL)
    {
        printf ("Error: NETFP Client Initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: NETFP Client '%s' Initialized %p\n", clientConfig.clientName, netfpClientHandle);

    /* Loop around and synchronize the client registeration with the server. */
    while (1)
    {
        /* Get the client status. */
        clientStatus = Netfp_startClient (netfpClientHandle, &errCode);
        if (clientStatus < 0)
        {
            printf ("Error: Client registeration status failed [Error code %d]\n", errCode);
            return -1;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        usleep(1000*20);
    }
    printf ("Debug: NETFP Client has been registered successfully with the server\n");

    /* Create the test thread. */
    errCode = pthread_create (&netfpClientThread, NULL, ClientExecutionTask, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the UINTC thread. */
    errCode = pthread_create (&uintcThread, NULL, UintcThread, NULL);
    if (errCode < 0)
    {
        printf ("Error: UINTC thread create failed error code %d\n", errCode);
        return -1;
    }

#if 0
    /* Enable access to PMU counters */
    pthread_create (&testThread, NULL, Test_enablePMUCounter, NULL);
    pthread_join(testThread, NULL);
#endif

    /* Create the IPv4 Socket thread. */
    errCode = pthread_create (&testThread, NULL, Test_socketThread, (void *)&advBenchmark);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the IPv6 Socket thread. */
    errCode = pthread_create (&testThread, NULL, Test_socket6Thread, (void *)&advBenchmark);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the IPv4 wildcarding thread. */
    errCode = pthread_create (&testThread, NULL, Test_wildCardingThread, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the IPv6 wildcarding thread. */
    errCode = pthread_create (&testThread, NULL, Test_wildCarding6Thread, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Blocked till the threads are done. */
    pthread_join (testThread, NULL);

    /* Stop the NETFP client */
    if (Netfp_stopClient (netfpClientHandle, &errCode) < 0)
        printf ("Error: NETFP Client stop failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Client stop successfully\n");

    /* Debug Message: */
    printf ("Debug: Canceling the UINTC and NETFP client threads\n");

    /* Tests have been completed: Cancel the NETFP Client & UINTC threads */
    pthread_cancel (uintcThread);
    pthread_cancel (netfpClientThread);

    /* Delete the NETFP client */
    if (Netfp_deleteClient (netfpClientHandle, &errCode) < 0)
        printf ("Error: NETFP Client deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Client deleted successfully\n");

    /* Shutdown all the PKTLIB Heaps which had been created */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpDataRxHeap, &errCode) < 0)
        printf ("Error: NETFP Data Receive Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Data Receive Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, agentReceiveHeap, &errCode) < 0)
        printf ("Error: Agent Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: Agent Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpHeaderHeap, &errCode) < 0)
        printf ("Error: NETFP Header Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Header Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpFragHeap, &errCode) < 0)
        printf ("Error: NETFP Fragmentation Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Fragmentation Heap deletion successful\n");

    /* Shutdown the UINTC module: This will initiate the shutdown of the UINTC thread. */
    if (Uintc_deinit (uintcHandle, &errCode) < 0)
    	printf("Error: UINTC module deinitialization failed [Error code %d]\n", errCode);
    else
        printf("Debug: UINTC module deinitializated successfully [Error code %d]\n", errCode);

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");

    return 0;
}

