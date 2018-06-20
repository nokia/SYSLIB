/*
 *   @file  main_core0.c
 *
 *   @brief   
 *      Unit Test code for the DPM Application which executes on Core0
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
 *
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
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK & CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/dpm/dpm.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Packet Management Heap */
Pktlib_HeapHandle       pktMgmtHeap;

/* RB Status Monitoring Buffer. */
Dpm_UserProfileStatus   gQueueMonitorStatus[DPM_MAX_UE][DPM_MAX_RB];

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global system named resource instance handle. */
Name_DBHandle           globalNameDatabaseHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
	0,    /* Number of CPINTC Output  requested                               */
	2,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
	0,    /* Number of QPEND Queues requested                                 */
	/* Requested Memory Region Configuration. */
	{
		/* Name,           Type,                    Linking RAM,                            Num,    Size */
		{ "Core0-MSMC", Resmgr_MemRegionType_MSMC,   Resmgr_MemRegionLinkingRAM_DONT_CARE,  4096,   128},
		{ "Core0-Local", Resmgr_MemRegionType_LOCAL, Resmgr_MemRegionLinkingRAM_DONT_CARE,  64,     128},
	}
};

/**********************************************************************
 *********************** Unit Test Extern Definitions *****************
 **********************************************************************/

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* OSAL callout */
extern uint32_t l2_global_address (uint32_t addr);

/* Report Test Functions. */
extern int32_t test_reports (void);

/* Benchmark Functions. */
extern int32_t benchmarkDpm (void);

/* Stress Test Functions. */
extern int32_t stress_test (void);

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

/* MSGCOM: */
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
 *      The application specified drop policy function. The function 
 *      is called by the DPM during packet reception. 
 *
 *  @param[in]  ueId
 *      UE Identifier
 *  @param[in]  rbId
 *      Radio Bearer Identifier
 *  @param[in]  qci
 *      QCI associated with the radio bearer.
 *  @param[in]  pendingPkts
 *      Number of pending packets for the specific user profile.
 *  @param[in]  pendingBytes
 *      Number of pending bytes for the specific user profile.
 *
 *  @retval
 *      0     -   Drop the packet
 *  @retval
 *      1     -   Accept the packet.
 */
static uint32_t myAppDropPolicyFunction
(
    uint8_t     ueId, 
    uint8_t     rbId, 
    uint8_t     qci,
    uint32_t    pendingPkts, 
    uint32_t    pendingBytes
)
{
    /* Ok to accept all packets */
    return 1;
}

/**
 *  @b Description
 *  @n  
 *      The function will enable the user profiles for all users and
 *      radio bearers which can exist.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t testEnableAllUserProfiles (void)
{
    Dpm_UserProfile userProfile;
    uint16_t        ueId;
    uint8_t         rbId;

    /* Cycle through all the users */
    for (ueId = 0; ueId < DPM_MAX_UE; ueId++)
    {
        /* For each user cycle through all the radio bearers */
        for (rbId = 0; rbId < DPM_MAX_RB; rbId++)
        {
            /* Populate the user profile. */
            userProfile.ueId = ueId;
            userProfile.rbId = rbId;

            /* Enable the user profile. */
            if (Dpm_configureUserProfile (&userProfile, 1) < 0)
            {
                System_printf ("Error: Configuration of user profile failed\n");
                return -1;
            }
        }
    }
    /* All user profiles were enabled. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
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
    Pktlib_HeapCfg              heapCfg;
    int32_t                     errCode;
    Resmgr_ResourceCfg*         ptrCfg;
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

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

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    sysConfig.coreId                            = DNUM;
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

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Get the application configuration. */
    ptrCfg = &appResourceConfig;

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
    
    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy (heapCfg.name, "DPM_PacketRxHeap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 512;
    heapCfg.numPkts                         = 3000;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    
    /* Create a shared heap. */ 
    pktMgmtHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (pktMgmtHeap == NULL)
    {
	    System_printf ("Error: Unable to create shared heap, error code %d\n", errCode);
        return -1;
    }

    /* System has been initialized successfully */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      DPM Unit Test Task
 *
 *  @retval
 *      Not Applicable.
 */
void DPMTask(UArg arg0, UArg arg1)
{
    Msgcom_ChannelCfg      chConfig;
    MsgCom_ChHandle        chHandle;
    Dpm_Config             dpmConfig;
    int32_t                errCode;

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                        = NULL;
    chConfig.msgcomInstHandle                                   = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = appResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = appResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = appResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = appResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 5;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 0;

    /* Create the Message communicator channel. */
    chHandle = Msgcom_create ("DPMChannel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (chHandle == 0)
    {
        System_printf ("Error: Unable to open the channel. Error : %d\n", errCode);
        return;
    }
    System_printf ("Debug: DPM Accumulator Channel 0x%p has been created\n", chHandle);

    /* Initialize the DPM Configuration. */
    memset ((void *)&dpmConfig, 0, sizeof(Dpm_Config));

    /* Populate the DPM Configuration. */
    dpmConfig.dropPolicyFxn           = myAppDropPolicyFunction;
    dpmConfig.pktlibInstHandle        = appPktlibInstanceHandle;

    /* Specify the packet delay budget for each QCI. */
    dpmConfig.qciPacketDelayBudget[0] = 0;              /* Not used. */
    dpmConfig.qciPacketDelayBudget[1] = 100*1000000;    /* 100ms */
    dpmConfig.qciPacketDelayBudget[2] = 150*1000000;    /* 150ms */
    dpmConfig.qciPacketDelayBudget[3] = 300*1000000;    /* 300ms */
    dpmConfig.qciPacketDelayBudget[4] = 50*1000000;     /* 50ms  */
    dpmConfig.qciPacketDelayBudget[5] = 100*1000000;    /* 100ms */
    dpmConfig.qciPacketDelayBudget[6] = 100*1000000;    /* 100ms */
    dpmConfig.qciPacketDelayBudget[7] = 300*1000000;    /* 300ms */
    dpmConfig.qciPacketDelayBudget[8] = 300*1000000;    /* 300ms */
    dpmConfig.qciPacketDelayBudget[9] = 300*1000000;    /* 300ms */

    /* Initialize the DPM driver. */
    if (Dpm_init (&dpmConfig, &errCode) < 0)
    {
        System_printf ("Error: DPM Initialization failed Error Code: %d\n", errCode);
        return;
    }

    /* Debug Message: */
    System_printf ("Debug: DPM has been initialized successfully\n");

    /* Enable all the user profiles initially for the tests to execute */
    if (testEnableAllUserProfiles() < 0)
        return;

    /* Test the reports and DPM API */
    if (test_reports () < 0)
    {
        System_printf ("Error: Test Failed\n");
        return;
    }
    System_printf ("Debug: Test Passed\n");

    /* Enable all the user profiles for the benchmarking tests to complete; since
     * some user profiles would have been disabled as a part of the DPM Functionality
     * testing. */
    if (testEnableAllUserProfiles() < 0)
        return;

    /* Test: Benchmark the DPM Driver API */
    if (benchmarkDpm () < 0)
        return;

    /* Stress Test: */
    if (stress_test() < 0)
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

    /* Initialize the CPPI and QMSS Modules. */
	if (system_init() < 0)
	    return;

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(DPMTask, &taskParams, NULL);
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

    System_printf ("***********************************\n");
    System_printf ("********** DPM Unit Test **********\n");
    System_printf ("***********************************\n");

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    Ipc_start();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);
    
    /* Start BIOS */
    BIOS_start();
}
