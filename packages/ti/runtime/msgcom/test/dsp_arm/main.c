/**
 *   @file  main.c
 *
 *   @brief
 *      Unit Test Code for the message communicator module
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
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 *********************** Extern Declarations **************************
 **********************************************************************/

/* Reader & Writer Threads: */
extern void* reader(void* arg);
extern void* writer(void* arg);

/*****************************************************************************
 * OSAL Callout Functions:
 *****************************************************************************/

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

/* Name Proxy/Client: */
extern void* Name_osalMalloc (uint32_t, uint32_t);
extern void  Name_osalFree (void* , uint32_t);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);
extern void* Name_osalEnterCS (void);
extern void  Name_osalExitCS (void*);
extern void* Name_osalCreateSem (void);
extern void  Name_osalDeleteSem(void*);
extern void  Name_osalPendSem(void*);
extern void  Name_osalPostSem(void*);

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Maximum number of messages exchanged between the reader and writer */
const uint32_t  MAX_TEST_MESSAGES = 10;

/* Global PKTLIB Heap Handle: */
Pktlib_HeapHandle       testHeapHandle;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global variable for the database */
Name_DBHandle           globalNameDatabaseHandle;

/* Global variable for the application MSGCOM instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global variable which keeps track of the test execution status */
uint32_t                gTestExecutionCompleted = 0;

/* Global variable which keeps track of which mode is getting tested: */
uint32_t                gTestWriter;

/* Name Client Handle: */
Name_ClientHandle       nameClientHandle;

/* User space interrupt handle block */
UintcHandle             uintcHandle;

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
 *      Name Client Execution Task: The task provides an execution context for
 *      the NAME client
 *
 *  @retval
 *      Not Applicable.
 */
static void* NameClientTask(void* arg)
{
    Name_ClientHandle       nameClientHandle;

    /* Get the name client handle: */
    nameClientHandle = (Name_ClientHandle)arg;

    /* Loop around till the test completes */
    while (gTestExecutionCompleted == 0)
    {
        /* Execute the name client */
        Name_executeClient (nameClientHandle);
        usleep(10);
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
int32_t main (int argc, char* argv[])
{
    Resmgr_ResourceCfg*         ptrResCfg;
    pthread_t                   testThread;
    int32_t                     errCode;
    Pktlib_HeapCfg              heapCfg;
    Resmgr_SystemCfg            sysConfig;
    pthread_t                   nameClientThread;
    Name_ClientCfg              clientCfg;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    UintcConfig                 uintcConfig;

    if (argc != 2)
    {
        printf ("Error: Invalid Usage; please execute the application with the -r or -w option\n");
        return -1;
    }

    /* Process the arguments: */
    if (strcmp (argv[1] , "-w") == 0)
    {
        /* Writer mode: */
        gTestWriter = 1;
    }
    else if (strcmp (argv[1] , "-r") == 0)
    {
        /* Reader mode: */
        gTestWriter = 0;
    }
    else
    {
        printf ("Error: Invalid Usage; please execute the application with the -r or -w option\n");
        return -1;
    }

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
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
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

    /* Get the application specific configuration */
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
    strcpy(uintcConfig.name, "TestUINTC");
    uintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (uintcHandle == NULL)
    {
        printf ("Error: Unable to open the UINTC module\n");
        return -1;
    }
    printf ("Debug: UINTC module has been opened successfully.\n");

    /* Initialize the name client configuration */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "UnitTestARM");
    strcpy (clientCfg.proxyName, "NameServer_LTE9A");
    clientCfg.databaseHandle    = globalNameDatabaseHandle;
    clientCfg.realm             = Name_ExecutionRealm_ARM;
    clientCfg.malloc            = Name_osalMalloc;
    clientCfg.free              = Name_osalFree;
    clientCfg.beginMemAccess    = Name_osalBeginMemAccess;
    clientCfg.endMemAccess      = Name_osalEndMemAccess;
    clientCfg.enterCS           = Name_osalEnterCS;
    clientCfg.exitCS            = Name_osalExitCS;
    clientCfg.createSem         = Name_osalCreateSem;
    clientCfg.deleteSem         = Name_osalDeleteSem;
    clientCfg.postSem           = Name_osalPostSem;
    clientCfg.pendSem           = Name_osalPendSem;

    /* Create the name clients: Name clients can only be created once the PROXY is operational */
    while (1)
    {
        /* Create the name client */
        nameClientHandle = Name_initClient (&clientCfg, &errCode);
        if (nameClientHandle != NULL)
            break;

        if (errCode != NAME_ENOTREADY)
            printf ("Error: Name client creation failed [Error code %d]\n", errCode);
    }
    printf ("Debug: Name client %p has been created successfully\n", nameClientHandle);

    /* Create the name client task which provides an application context for the NAME Client */
    errCode = pthread_create (&nameClientThread, NULL, NameClientTask, nameClientHandle);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread %d\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration */
    memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "My Test Heap");
    heapCfg.memRegion                       = ptrResCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
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
    testHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (testHeapHandle == NULL)
    {
        printf ("Error: Unable to create the heap error code %d\n", errCode);
        return -1;
    }
    printf ("Debug: Heap %s has been created successfully\n", heapCfg.name);

    /* Display the debug banner: */
    printf ("*********************************************************\n");
    printf ("Debug: DSP ARM Unit Tests [Process Id: %d]\n", getpid());
    printf ("Debug: ARM is the %s\n", (gTestWriter == 1) ? "Writer" : "Reader");
    printf ("Debug: DSP is the %s\n", (gTestWriter == 0) ? "Writer" : "Reader");
    printf ("Debug: Ensure that the WRITERS are always executed first\n");
    printf ("*********************************************************\n");

    /* Launch the threads: */
    if (gTestWriter == 0)
    {
        /* Create the reader thread. */
        errCode = pthread_create (&testThread, NULL, reader, NULL);
        if (errCode < 0)
        {
            printf ("Error: Unable to create the reader thread %d\n", errCode);
            return -1;
        }
    }
    else
    {
        /* Create the writer thread. */
        errCode = pthread_create (&testThread, NULL, writer, NULL);
        if (errCode < 0)
        {
            printf ("Error: Unable to create the writer thread %d\n", errCode);
            return -1;
        }
    }

    /* Blocked till the threads are done. */
    pthread_join (testThread, NULL);
    pthread_join (nameClientThread, NULL);

    /* Close the PKTLIB Heap */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, testHeapHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the test heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Shutdown the PKTLIB instance */
    if (Pktlib_deleteInstance (appPktlibInstanceHandle, &errCode) < 0)
    {
        printf ("Error: Unable to delete the PKTLIB instance [Error code %d]\n", errCode);
        return -1;
    }

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
    return 0;
}

