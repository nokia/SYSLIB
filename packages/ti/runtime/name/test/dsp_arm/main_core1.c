/*
 *   @file  main_core0.c
 *
 *   @brief
 *      Unit Test code which executes on Core0
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
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* IPC Include Files  */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_device_interrupt.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>

/**********************************************************************
 *********************** Unit Test Local Definitions ******************
 **********************************************************************/

/* Maximum number of test iterations */
#define TEST_MAX_ITERATIONS      100

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
Resmgr_SysCfgHandle     handleSysCfg;
Name_DBHandle           globalNameDatabaseHandle;
Msgcom_InstHandle       appMsgcomInstanceHandle;
Pktlib_InstHandle       appPktlibInstanceHandle;
Pktlib_HeapHandle       nameClientProxyHeapHandle;
Name_ClientHandle       nameClientHandle;

/* Test completion Status: */
uint32_t    testComplete = 0;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
    1,    /* Number of Accumulator Channels requested                         */
    0,    /* Number of Hardware Semaphores requested                          */
    1,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
    {
        /* Name,           Type,                    Linking RAM,                            Num,    Size */
        { "Core1-DDR", Resmgr_MemRegionType_DDR3,   Resmgr_MemRegionLinkingRAM_DONT_CARE,    512,   128},
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

/* PKTLIB: */
extern void* Pktlib_osalMalloc(Pktlib_MallocMode, uint32_t);
extern void  Pktlib_osalFree(Pktlib_MallocMode, void*, uint32_t);
extern void  Pktlib_osalBeginMemAccess(void*, uint32_t);
extern void  Pktlib_osalEndMemAccess(void*, uint32_t);
extern void  Pktlib_osalBeginPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void  Pktlib_osalEndPktAccess(Pktlib_HeapHandle, Ti_Pkt*, uint32_t);
extern void* Pktlib_osalEnterCS(Pktlib_HeapHandle);
extern void  Pktlib_osalExitCS(Pktlib_HeapHandle, void*);

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

/**********************************************************************
 ************************** Unit Test Functions ***********************
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
 *      Application specified argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* mySharedMemoryMalloc(uint32_t size, uint32_t arg)
{
    /* Allocate memory from the shared memory heap. */
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
 *      Application specified argument
 *
 *  @retval
 *      Not Applicable.
 */
static void mySharedMemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    Memory_free((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, size);
}

/**
 *  @b Description
 *  @n
 *      The function is used to synhronize for the specific resource name.
 *
 *  @param[in]  syncName
 *      Resource name to be synchronized
 *  @param[in]  handle1
 *      Value of handle1 to be validated.
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_sync (char* syncName, uint32_t handle1)
{
    int32_t             errCode;
    Name_ResourceCfg    namedResourceCfg;

    /* Loop around till the synchronize name is found. */
    while (1)
    {
        /* Find the named resource. */
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                               syncName, &namedResourceCfg, &errCode) < 0)
        {
            if (errCode != NAME_ENOTFOUND)
            {
                printf ("Error: Find named resource %s failed [Error code %d]\n", syncName, errCode);
                return;
            }

            /* Synhronize name is not found. Try again after some time */
            Task_sleep (1);
            continue;
        }
        /* Perform the handle validation. */
        if (namedResourceCfg.handle1 != handle1)
            printf ("Error: Handle verification %s failed [Received %x Expected %x]\n", syncName, namedResourceCfg.handle1, handle1);
        break;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to synhronize for the specific resource name.
 *
 *  @param[in]  nameClientHandle
 *      Handle to the name client
 *  @param[in]  resName
 *      Name of the resource to be created and pushed
 *  @param[in]  handle1
 *      Handle1 value to be assigned
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_createPushNamedResource
(
    Name_ClientHandle   nameClientHandle,
    char*               resName,
    uint32_t            handle1
)
{
    Name_ResourceCfg    namedResourceCfg;
    int32_t             errCode;

    /* Create a named resource and indicate to the DSP cores that the ARM is now operational */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    strcpy(namedResourceCfg.name, resName);
    namedResourceCfg.handle1  = handle1;

    /* Create the named resource in the ARM realm. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            &namedResourceCfg, &errCode) < 0)
    {
        printf ("Error: Named resource creation '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    printf ("Debug: Created named resource '%s'.\n", namedResourceCfg.name);

    /* Push the named resource to the DSP realm */
    if (Name_push (nameClientHandle, &namedResourceCfg.name[0],
                   Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Named resource push '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the specific named resource and push it across
 *      to the DSP realm.
 *
 *  @param[in]  nameClientHandle
 *      Handle to the name client
 *  @param[in]  resName
 *      Name of the resource to be created and pushed
 *
 *  @retval
 *      Not Applicable.
 */
static void NameTest_deletePushNamedResource
(
    Name_ClientHandle   nameClientHandle,
    char*               resName
)
{
    int32_t errCode;

    /* Debug Message: */
    printf ("Debug: Deleting named resource '%s'\n", resName);

    /* Delete the named resource in the ARM realm. */
    if (Name_deleteResource(globalNameDatabaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                            resName, &errCode) < 0)
    {
        printf ("Error: Named resource deletion '%s' failed (Error %d) \n", resName, errCode);
        return;
    }

    /* Push the named resource to the DSP realm */
    if (Name_push (nameClientHandle, resName, Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_DELETE, &errCode) < 0)
    {
        printf ("Error: Delete named resource push '%s' failed (Error %d) \n", resName, errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Name Client Test Task
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_nameClientTask(UArg arg0, UArg arg1)
{
    char                resourceName[NAME_MAX_CHAR];
    uint8_t             index;
    Name_ResourceCfg    namedResourceCfg;
    int32_t             errCode;

    /* Populate the named resource configuration. */
    memset ((void *)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));
    strcpy(namedResourceCfg.name, "SecretDSPResource");
    namedResourceCfg.handle1  = 0x1111;
    namedResourceCfg.handle2  = 0x2222;

    /* Create the PRIVATE named resource in the DSP realm. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                            &namedResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Named resource creation '%s' failed [Error code %d]\n", namedResourceCfg.name, errCode);
        return;
    }
    System_printf ("Debug: Created private named resource '%s'.\n", namedResourceCfg.name);

    /* Inform the ARM application: The DSP cores are ready */
    NameTest_createPushNamedResource (nameClientHandle, "DSP-Core-Status", 0xDEAD);

    /* Wait for the ARM to be operational. */
    System_printf ("Debug: [Core %d] Waiting for the ARM to be operational\n", DNUM);
    NameTest_sync ("ARM-Status", 0xBEEF);
    System_printf ("Debug:[Core %d] ARM is operational\n", DNUM);

    /* Get the private ARM resource */
    if (Name_get (nameClientHandle, 1, Name_ResourceBucket_USER_DEF1,
                  "SecretARMResource", &namedResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Named resource get of ARM Private name failed [Error code %d]\n", errCode);
        return;
    }
    if ((namedResourceCfg.handle1 != 0xbabe) || (namedResourceCfg.handle2 != 0xface))
    {
        System_printf ("Debug: Invalid handles detected 0x%x 0x%x\n", namedResourceCfg.handle1, namedResourceCfg.handle2);
        return;
    }
    System_printf ("Debug: Get ARM Private named resource '%s' passed.\n", namedResourceCfg.name);

    /* Debug Message: */
    System_printf ("Debug:[Core %d] tests the creation/detection of named resources from ARM\n", DNUM);
    for (index = 0; index < TEST_MAX_ITERATIONS; index++)
    {
        /* Construct the name */
        sprintf(resourceName, "ARM-Test-%d", index);

        /* Loop around till we find the named resource and perform the validation also. */
        NameTest_sync (resourceName, index);

        /* Debug Message: */
        System_printf ("Debug:[Core %d] detecting ARM named resources '%s'\n", DNUM, resourceName);
    }
    System_printf ("Debug:[Core %d] Creating ARM named resources test passed\n");

    /* Once all the resources have been detected; set the flag to indicate that the creation test passed */
    NameTest_createPushNamedResource (nameClientHandle, "CreationTest", 0x12345678);

    /* Tests are complete */
    testComplete = 1;

    /* Test passed */
    System_printf ("Debug:[Core %d] All tests passed\n", DNUM);
    return;
}

/**
 *  @b Description
 *  @n
 *      Name Proxy/Client Task which provides an execution context for the
 *      Name Client to execute.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_nameClientProxyTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Execute the NAME Client Task: */
    while (testComplete == 0)
    {
        /* Execute the name client. */
        Name_executeClient (nameClientHandle);

        /* Relinquish time slice */
        Task_sleep(1);
    }

    /* Shutdown the name client */
    if (Name_deleteClient (nameClientHandle, &errCode) < 0)
        System_printf ("Error: Unable to delete the name client [Error code %d]\n", errCode);
    else
        System_printf ("Debug: Name client deleted successfully\n");
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and initialize the PKTLIB heaps.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_createPktlibHeaps(void)
{
    Pktlib_HeapCfg        heapCfg;
    int32_t               errCode;
    Resmgr_ResourceCfg*   ptrCfg;

    /* Get the application configuration. */
    ptrCfg = &appResourceConfig;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - This heap is used to send and receive messages between the name clients and proxy on the
     *    DSP cores. This heap needs to be shared between the proxy and client. */
    strcpy(heapCfg.name, "Name_Proxy_Client_Core1_Heap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    nameClientProxyHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (nameClientProxyHeapHandle == NULL)
    {
        System_printf ("Error: Unable to create Name Proxy Client Heap [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
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
static int32_t Test_systemInit (void)
{
    int32_t                 errCode;
    Resmgr_SystemCfg        sysConfig;
    Task_Params             taskParams;
    Name_DatabaseCfg        databaseCfg;
    Msgcom_InstCfg          msgcomInstCfg;
    Pktlib_InstCfg          pktlibInstCfg;
    Name_ClientCfg          clientCfg;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L2");
    strcpy (sysConfig.rmServer, "Rm_Server");
    sysConfig.coreId                            = DNUM;
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

    /* Process the application configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 1;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE1_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE1_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 0;
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
        System_printf ("Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the PKTLIB Heaps: */
    if (Test_createPktlibHeaps() < 0)
        return -1;

    /* Name Proxy is operational: Initialize the name client */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "NameClient_LTE9A_L1");
    strcpy (clientCfg.proxyName, "NameServer_LTE9A");
    clientCfg.databaseHandle            = globalNameDatabaseHandle;
    clientCfg.realm                     = Name_ExecutionRealm_DSP;
    clientCfg.u.dspCfg.pktlibInstHandle = appPktlibInstanceHandle;
    clientCfg.u.dspCfg.msgcomInstHandle = appMsgcomInstanceHandle;
    clientCfg.u.dspCfg.clientHeapHandle = nameClientProxyHeapHandle;
    clientCfg.malloc                    = Name_osalMalloc;
    clientCfg.free                      = Name_osalFree;
    clientCfg.beginMemAccess            = Name_osalBeginMemAccess;
    clientCfg.endMemAccess              = Name_osalEndMemAccess;
    clientCfg.enterCS                   = Name_osalEnterCS;
    clientCfg.exitCS                    = Name_osalExitCS;
    clientCfg.createSem                 = Name_osalCreateSem;
    clientCfg.deleteSem                 = Name_osalDeleteSem;
    clientCfg.postSem                   = Name_osalPostSem;
    clientCfg.pendSem                   = Name_osalPendSem;

    /* Create the name clients: Name clients can only be created once the PROXY is operational */
    while (1)
    {
        /* Create the name client */
        nameClientHandle = Name_initClient (&clientCfg, &errCode);
        if (nameClientHandle != NULL)
            break;

        if (errCode != NAME_ENOTREADY)
            System_printf ("Error: Name client creation failed [Error code %d]\n", errCode);
    }

    /* Launch the name client/proxy task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 2;
    Task_create(Test_nameClientProxyTask, &taskParams, NULL);

    /* Launch the name client task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 2;
    Task_create(Test_nameClientTask, &taskParams, NULL);

    return 0;
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
static void Test_sysInitTask(UArg arg0, UArg arg1)
{
    /* Initialize the system modules */
    if (Test_systemInit() < 0)
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
    Task_Params taskParams;

    /* Initialize TSCL register */
    TSCL = 0;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* System Initialization Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}


