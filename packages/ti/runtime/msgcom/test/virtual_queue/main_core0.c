/*
 *   @file  main_core0.c
 *
 *   @brief   
 *      Unit Test Framework for MSGCOM Virtual Channel Testing.
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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c66/Cache.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 *********************** Framework Extern Definitions *****************
 **********************************************************************/

/* Reader Stress Task */
extern void readerStressTask(UArg arg0, UArg arg1);

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
 ************************** Global Variables **************************
 **********************************************************************/

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global system named resource instance handle. */
Name_DBHandle           globalNameDatabaseHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Application requested resources */
Resmgr_ResourceCfg    appResourceConfig =
{
	3,    /* Number of CPINTC Output  requested                               */
	2,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
	1,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                      Linking RAM,                            Num,      Size */
		{ "Core0-DDR3-0", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,   16384,    128},
    }
};

/**********************************************************************
 ************************** Framework Functions ***********************
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
    Error_Block	errorBlock;

    /* Allocate memory from the shared memory heap. */ 
    return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), size, 0, &errorBlock);
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
    /* Cleanup the memory block. */
    Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, size);
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create and initialize the PKTLIB heaps
 *      which are required to initialize the SYSLIB components. Applications 
 *      can create additional heaps for their own use and this can be done 
 *      in the context of the application domain. 
 *
 *      The number of packets in the heap and the size & placement of 
 *      the buffers can be modified by the application to match their 
 *      requirements.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t App_createPktlibHeaps(void)
{
    Pktlib_HeapCfg        heapCfg;
    int32_t               errCode;
    Resmgr_ResourceCfg*   ptrCfg;
    Pktlib_HeapHandle     msgcomDataHeap;

    /* Get the application resource configuration. */
    ptrCfg = &appResourceConfig;

    /*************************************************************************
     * MSGCOM Data Receive Heap:
     *  - This is a shared heap which is used in the test to send/receive
     *    all the messages.
     *************************************************************************/

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "Message_DataHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 1;
    heapCfg.dataBufferSize                  = 512;
    heapCfg.numPkts                         = 16384;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
 
    /* Create a shared heap. */ 
    msgcomDataHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (msgcomDataHeap == NULL)
    {
	    System_printf ("Error: Unable to create the Message Data receive heap. Error Code %d\n", errCode);
        return -1;
    }

    /* Debug: Display the Heaps which have been created. */
    System_printf ("Debug: Message Data Heap  Buffer: %d Queue -> 0x%x\n", 
                   Pktlib_getMaxBufferSize(msgcomDataHeap), Pktlib_getInternalHeapQueue(msgcomDataHeap));
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function initializes the system and brings up the various SYSLIB 
 *      modules
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t App_systemInit(void)
{
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Name_ResourceCfg            systemInitResource;  
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

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

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 1;
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
        System_printf ("Error: Creating the PKTLIB instance failed [Error code %d]\n", errCode);
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

    /* Initialize and create the PKTLIB heaps required. */
    if (App_createPktlibHeaps() < 0)
        return -1;

    /* System is operational: Set the Named resource as an indication to the rest of the system
     * to perform their startup. */
    memset ((void*)&systemInitResource, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource */
    systemInitResource.handle1  = 0x12345678;
    systemInitResource.handle2  = 0;
    strcpy(systemInitResource.name, "TICore0InitDone");

    /* Create the named resource. Once this is done all the other cores in the system will get unblocked and
     * can start their initialization. */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1, 
                            &systemInitResource, &errCode) < 0)
    {
        System_printf ("Error: Named resource creation failed (Error %d) \n", errCode);
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Entry point for the system initialization task. The task is
 *      responsible for initializing the system and bringing up the 
 *      SYSLIB components. 
 *
 *      The task will then create the other tasks which are required
 *      for the SYSLIB components to work
 *
 *  @retval
 *      Not Applicable.
 */
static void App_sysInitTask(UArg arg0, UArg arg1)
{
    Task_Params taskParams;

    /* Debug Message: */
    System_printf ("*******************************************************\n");
    System_printf ("********* MSGCOM Virtual Channel Testing **************\n");
    System_printf ("*******************************************************\n");

    /* Initialize the System. */
	if (App_systemInit() < 0)
	    return;

    /* Reader Stress Test */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(readerStressTask, &taskParams, NULL);
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

    /* Enable the caches. */
    CACHE_setL1DSize (CACHE_L1_32KCACHE);
    CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Enable the timestamp */
    TSCL = 0;

    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    Ipc_start();

    /* This is where we configure the properties for each MAR region. 
     *  - MSMC Prefecth needs to be disabled. */
    Cache_setMar((Ptr)0x0C000000, 0x01000000, Cache_PC);    // Entire MSMC (Configured without prefetching support)

    /* Initialize & create the system initialization task */ 
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(App_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

