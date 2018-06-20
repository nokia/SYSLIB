/**
 *   @file  main_core1.c
 *
 *   @brief
 *      Unit Test code which executes on Core1
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
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/sa/sa3gppEnabler/sa3gpp.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Core1 local PKTLIB Heaps */
Pktlib_HeapHandle       netfpHeaderHeap;
Pktlib_HeapHandle       netfpFragHeap;
Pktlib_HeapHandle       netfp10KReceiveHeap;
Pktlib_HeapHandle       mtuReceiveHeap;
Pktlib_HeapHandle       nameClientProxyHeapHandle;

/* System Configuration Handle. */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global system named resource instance handle. */
Name_DBHandle           globalNameDatabaseHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* NETFP Client Handle: */
Netfp_ClientHandle      netfpClientHandle;

/* Agent Client Handle: */
Name_ClientHandle       nameClientHandle;

/* NETFP & Agent Server Name used by the test */
const char*             gNetfpServerName = "NetfpServer_LTE9A";
const char*             gNameProxyName   = "NameServer_LTE9A";

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
    6,    /* Number of Accumulator Channels requested                         */
    0,    /* Number of Hardware Semaphores requested                          */
    6,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
    {
        /* Name,           Type,                    Linking RAM,                            Num,    Size */
        { "Core1-DDR3", Resmgr_MemRegionType_DDR3,   Resmgr_MemRegionLinkingRAM_DONT_CARE,  512,   128},
        { "Reassembly", Resmgr_MemRegionType_MSMC,   Resmgr_MemRegionLinkingRAM_DONT_CARE,  64,    128}
    }
};

/**********************************************************************
 *********************** Extern Declarations **************************
 **********************************************************************/

/* Test Environment setup: */
extern void Test_envSetupTask(UArg arg0, UArg arg1);

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
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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

    /* Get the resource configuration for the specific core. */
    ptrCfg = &appResourceConfig;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used to create networking headers which are populated by the NETFP
     *    library for every packet which is to be transmitted. */
    strcpy(heapCfg.name, "Core1_NETFP_HeaderHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    netfpHeaderHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpHeaderHeap == NULL)
    {
	    System_printf ("Error: Unable to create Networking Header Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - This heap is used to send fragmented packets provided the fragmentation is done
     *    in NETFP software. */
    strcpy(heapCfg.name, "Core1_NETFP_FragmentHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 0;
    heapCfg.numPkts                         = 0;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    netfpFragHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpFragHeap == NULL)
    {
	    System_printf ("Error: Unable to create fragmentation Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets on this core */
    strcpy(heapCfg.name, "Core1_10K_ReceiveHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 10*1024;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    netfp10KReceiveHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfp10KReceiveHeap == NULL)
    {
	    System_printf ("Error: Unable to create the 10K heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets on this core */
    strcpy(heapCfg.name, "Core1_MTU_ReceiveHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 64;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    mtuReceiveHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (mtuReceiveHeap == NULL)
    {
	    System_printf ("Error: Unable to create the MTU heap [Error code %d]\n", errCode);
        return -1;
    }

    /***************************************************************************
     * Name Proxy Client Heap: The heap is shared between the NAME proxy and the
     * local name clients. This is created by the core executing the NAME proxy
     * and is found on the client cores.
     ***************************************************************************/
    while (1)
    {
        nameClientProxyHeapHandle = Pktlib_findHeapByName (appPktlibInstanceHandle,
                                                           "NameProxyClientHeap", &errCode);
        if (nameClientProxyHeapHandle != NULL)
            break;

        /* Relinuqish time slice */
        Task_sleep(10);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the NETFP Client
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_netfpClient(UArg arg0, UArg arg1)
{
	/* Execute the NETFP Client. */
    while (1)
        Netfp_executeClient (netfpClientHandle);
}

/**
 *  @b Description
 *  @n
 *      Name Client Execution Task:
 *
 *  @retval
 *      Not Applicable.
 */
static void NameClientExecutionTask(UArg arg0, UArg arg1)
{
    while (1)
    {
        /* Execute the name client */
        Name_executeClient (nameClientHandle);

        /* Relinquish time slice allowing other tasks to execute */
        Task_sleep(2);
    }
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
    Task_Params                 taskParams;
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Netfp_ClientConfig          netfpClientCfg;
    int32_t                     clientStatus;
    Netfp_ServerHandle          netfpServerHandle;
    Name_ClientCfg              clientCfg;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L1");
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
	    return;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return;
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
	    return;
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
        return;
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
        return;
    }

    /* Create local PKTLIB Heaps for use on this core */
    Test_createPktlibHeaps ();

    /* Initialize the name client configuration block. */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "NameClient_LTE9A_L2SCHED");
    strcpy (clientCfg.proxyName, gNameProxyName);
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

    /* Name client are only created if the proxy is operational */
    while (1)
    {
        /* Initialize the name client */
        nameClientHandle = Name_initClient (&clientCfg, &errCode);
        if (nameClientHandle != NULL)
            break;

        /* Error: Unable to create the client. */
        if (errCode != NAME_ENOTREADY)
        {
            System_printf ("Error: Unable to create the name client [Error code %d]\n", errCode);
            return;
        }
        /* Proxy was not operational. Wait for some time and try again. */
        Task_sleep(10);
    }
    System_printf ("Debug: Name client %p has been created successfully\n", nameClientHandle);

    /* Launch the Name client execution task: */
    Task_Params_init(&taskParams);
    taskParams.priority         = 5;
    taskParams.stackSize        = 16*1024;
    Task_create(NameClientExecutionTask, &taskParams, NULL);

    /* Initialize the SA 3GPP Enabler */
    Sa_3gppEnabler();

    /* SYNC Point: Ensure that the NETFP Server has been started. This is required before
     * we create and register the NETFP client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready. The NETFP server is executing on ARM
         * while the NETFP client is executing on the DSP so we need to cross the realm boundary */
        netfpServerHandle = Netfp_startServer (globalNameDatabaseHandle, nameClientHandle, (char*)gNetfpServerName, &errCode);
        if (netfpServerHandle != NULL)
            break;

        /* Check the error code. */
        if (errCode != NETFP_ENOTREADY)
        {
            System_printf ("Error: NETFP Starting server failed [Error code %d]\n", errCode);
            return;
        }
        Task_sleep(10);
    }

    /* Initialize the client configuration */
    memset ((void *)&netfpClientCfg, 0, sizeof(Netfp_ClientConfig));

    /* Populate the client configuration.
     * - Agent Client Handle is configured since the NETFP Server and clients
     *   are located in different realms.  */
    strcpy (netfpClientCfg.serverName, gNetfpServerName);
    if (DNUM == 0)
        sprintf (netfpClientCfg.clientName, "Netfp_Client_LTE9A_L2DP");
    else
        sprintf (netfpClientCfg.clientName, "Netfp_Client_LTE9A_L1");
    netfpClientCfg.nrInstanceId                       = databaseCfg.instanceId;
    netfpClientCfg.serverHandle                       = netfpServerHandle;
    netfpClientCfg.pktlibInstHandle                   = appPktlibInstanceHandle;
    netfpClientCfg.directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[2].queue;
    netfpClientCfg.directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[2].cpIntcId;
    netfpClientCfg.directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[2].systemInterrupt;
    netfpClientCfg.directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[2].hostInterrupt;
    netfpClientCfg.nameClientHandle                   = nameClientHandle;
    netfpClientCfg.netHeaderHeapHandle                = netfpHeaderHeap;
    netfpClientCfg.fragmentHeap				          = netfpFragHeap;
    netfpClientCfg.realm                              = Netfp_ExecutionRealm_DSP;
    netfpClientCfg.clientHeapHandle                   = mtuReceiveHeap;
    netfpClientCfg.msgcomInstHandle                   = appMsgcomInstanceHandle;
    netfpClientCfg.malloc                             = Netfp_osalMalloc;
    netfpClientCfg.free                               = Netfp_osalFree;
    netfpClientCfg.beginMemAccess                     = Netfp_osalBeginMemoryAccess;
    netfpClientCfg.endMemAccess                       = Netfp_osalEndMemoryAccess;
    netfpClientCfg.enterCS                            = Netfp_osalEnterSingleCoreCriticalSection;
    netfpClientCfg.exitCS                             = Netfp_osalExitSingleCoreCriticalSection;
    netfpClientCfg.createSem                          = Netfp_osalCreateSem;
    netfpClientCfg.deleteSem                          = Netfp_osalDeleteSem;
    netfpClientCfg.postSem                            = Netfp_osalPostSem;
    netfpClientCfg.pendSem                            = Netfp_osalPendSem;

    /* Create the NETFP Client. */
    netfpClientHandle = Netfp_initClient (&netfpClientCfg, &errCode);
    if (netfpClientHandle == NULL)
    {
        System_printf ("Error: NETFP Client Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: NETFP Client [%s] Initialized %p\n", netfpClientCfg.clientName, netfpClientHandle);

    /* Loop around and synchronize the client registration with the server. */
    while (1)
    {
        /* Start the NETFP client */
        clientStatus = Netfp_startClient (netfpClientHandle, &errCode);
        if (clientStatus < 0)
        {
            System_printf ("Error: Client registration status failed [Error code %d]\n", errCode);
            return;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        Task_sleep(10);
    }
    System_printf ("Debug: NETFP Client [%s] registered successfully with the server\n", netfpClientCfg.clientName);

    /* Launch the NETFP Client Execution Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 5;
    Task_create(Test_netfpClient, &taskParams, NULL);

    /* Launch the Test environment configuration task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_envSetupTask, &taskParams, NULL);
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

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Enable the timestamp */
    TSCL = 0;

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* System Initialization Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}
