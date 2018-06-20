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
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files  */
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
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Shared PKTLIB Heaps */
Pktlib_HeapHandle       netfpTxHeap;
Pktlib_HeapHandle       paCommandHeap;

/* Core0 local PKTLIB Heaps */
Pktlib_HeapHandle       netfpHeaderHeap;
Pktlib_HeapHandle       netfpFragHeap;
Pktlib_HeapHandle       netfpDataRxHeap;

/* System Configuration Handle. */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global system named resource instance handle. */
Name_DBHandle           globalNameDatabaseHandle;

/* Global NETFP Server Semaphore handle. */
Semaphore_Handle        netfpServerSemHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Global NETFP Server Handle: */
Netfp_ServerHandle      netfpServerHandle;

/* This is an array of NETFP Client names which could connect with the NETFP
 * Server. */
const char* gNetfpClientNames[] =
{
    "SYSLIB-NETFP-DSP-1",
    NULL
};

/* Global variable which if set will delete the NETFP server. */
uint32_t                gDeleteNetfpServer = 0;

/* Global variable which displays the NETFP Server details: */
uint32_t                gDisplayServer = 0;

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
    5,    /* Number of Accumulator Channels requested                         */
    0,    /* Number of Hardware Semaphores requested                          */
    1,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
    {
        /* Name,           Type,                    Linking RAM,                            Num,    Size */
        { "Core0-DDR3",  Resmgr_MemRegionType_DDR3,   Resmgr_MemRegionLinkingRAM_DONT_CARE,   512,    128},
        { "Core0-Local", Resmgr_MemRegionType_LOCAL,  Resmgr_MemRegionLinkingRAM_DONT_CARE,   64,     128}
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

/* NETFP: */
extern void* Netfp_osalMalloc (uint32_t , uint32_t );
extern void  Netfp_osalFree (void* , uint32_t );
extern void* Netfp_osalMallocSecurityContext (Netfp_SaProtocol , uint32_t , uint32_t );
extern void  Netfp_osalFreeSecurityContext (Netfp_SaProtocol , void* , uint32_t );
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
    Error_Block    errorBlock;

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
 *      Logging function registered with the NETFP Server to debug and display
 *      log messages. The function displays all logging levels on the console
 *
 *  @param[in]  logLevel
 *      Logging Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  ...
 *      Variable argument list
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_netfpServerLog(Netfp_LogLevel logLevel, char* fmt, va_list arg)
{
#if 0
    VaList ap;

    va_start(ap, fmt);
    System_vprintf(fmt, ap);
    va_end(ap);
#else
    vprintf (fmt, arg);
#endif
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

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "NetFP_PktTxHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1600;
    heapCfg.numPkts                         = 20;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create a shared heap. */
    netfpTxHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpTxHeap == NULL)
    {
        System_printf ("Error: Unable to create shared heap, error code %d\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "PA_CommandHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 576;
    heapCfg.numPkts                         = 5;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create a Shared Heap for the PA Commands */
    paCommandHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (paCommandHeap == NULL)
    {
        System_printf ("Error: Unable to create PA command Heap\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "Core0_NETFP_HeaderHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 256;
    heapCfg.numPkts                         = 50;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create a local Heap for use on this core for the NETFP Headers */
    netfpHeaderHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpHeaderHeap == NULL)
    {
        System_printf ("Error: Unable to create NETFP Header Heap\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "Core0_NETFP_FragmentHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 0;
    heapCfg.numPkts                         = 0;
    heapCfg.numZeroBufferPackets            = 100;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create a zero buffer local Heap for NETFP S/w fragmentation on this core */
    netfpFragHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpFragHeap == NULL)
    {
        System_printf ("Error: Unable to create NETFP Header Heap\n");
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets and is configured
     *    to have data buffers of 10K */
    strcpy(heapCfg.name, "Core0_NetFP_PktRxHeap");
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

    /* Create a local heap for Rx/Tx of data packets on this core. */
    netfpDataRxHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpDataRxHeap == NULL)
    {
        System_printf ("Error: Unable to create shared heap, error code %d\n", errCode);
        return -1;
    }

    /* Debug: Dump the MAX Buffer Sizes of the heaps */
    System_printf ("Debug: PA Command Heap          Buffer: %d bytes\n", Pktlib_getMaxBufferSize(paCommandHeap));
    System_printf ("Debug: PASA/FP CRC Heap         Buffer: %d bytes\n", Pktlib_getMaxBufferSize(netfpTxHeap));
    System_printf ("Debug: Core0 NETFP Data Heap    Buffer: %d bytes\n", Pktlib_getMaxBufferSize(netfpDataRxHeap));
    System_printf ("Debug: Core0 NETFP Header Heap  Buffer: %d bytes\n", Pktlib_getMaxBufferSize(netfpHeaderHeap));

    /* Debug: Dump the Queue Information for all the heaps. */
    System_printf ("Debug: PA Command Heap          -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(paCommandHeap));
    System_printf ("Debug: PASA/FP CRC Heap         -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(netfpTxHeap));
    System_printf ("Debug: Core0 NETFP Data Heap    -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(netfpDataRxHeap));
    System_printf ("Debug: Core0 NETFP Header Heap  -> Queue 0x%x\n", Pktlib_getInternalHeapQueue(netfpHeaderHeap));
    System_printf ("Debug: Core0 NETFP Fragmentation Heap -> Queue 0x%x\n", Pktlib_getZeroHeapQueue(netfpFragHeap));

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
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

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
    sysConfig.dspSystemCfg.armCoreId            = SYSLIB_ARM_CORE_ID;
    sysConfig.dspSystemCfg.sourceId             = 17;
    sysConfig.dspSystemCfg.sharedMemAddress     = DDR3_SYSLIB_RESMGR_RSVD;
    sysConfig.dspSystemCfg.sizeSharedMemory     = DDR3_SYSLIB_RESMGR_RSVD_LEN;

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

    /* Initialize and create the PKTLIB heaps required. */
    if (Test_createPktlibHeaps() < 0)
        return -1;

    /* System has been initialized successfully */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the NETFP Server MSGCOM channels
 *
 *  @retval
 *      Not Applicable.
 */
void Test_NetfpServerCallBack(MsgCom_ChHandle chHandle, uint32_t arg)
{
    /* Post the NETFP Server semaphore */
    Semaphore_post (netfpServerSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is the task which displays the debug information of the NETFP Server
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_netfpServerDebugTask(UArg arg0, UArg arg1)
{
    while (1)
    {
        /* Do we need to display the NETFP Server status? */
        if (gDisplayServer == 1)
        {
            /* Display server general information */
            Netfp_displayServerGenInfo(netfpServerHandle);

            /* Display all the security contexts configured on the server: */
            Netfp_displaySecurityContext(netfpServerHandle);

            /* Display all the security associations configured on the server: */
            Netfp_displaySA(netfpServerHandle,       Netfp_Direction_INBOUND);
            Netfp_displaySA(netfpServerHandle,       Netfp_Direction_OUTBOUND);

            /* Display all the security policies offloaded on the server: */
            Netfp_displaySP(netfpServerHandle,       Netfp_Direction_INBOUND);
            Netfp_displaySP(netfpServerHandle,       Netfp_Direction_OUTBOUND);

            /* Display all the fast paths created on the server: */
            Netfp_displayInboundFastPath(netfpServerHandle);
            Netfp_displayOutboundFastPath(netfpServerHandle);

            /* Display the interface list on the server: */
            Netfp_displayInterface(netfpServerHandle);

            /* Display the reassembly information */
            Netfp_displayReassemblyContext(netfpServerHandle);

            /* Reset the flag after the display */
            gDisplayServer = 0;
        }
        Task_sleep(10);
    }

}

/**
 *  @b Description
 *  @n
 *      This is the task which initializes and executes the NETFP Server.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_netfpServer(UArg arg0, UArg arg1)
{
    Netfp_ServerConfig          serverConfig;
    Msgcom_DirectInterruptCfg   directInterruptCfg;
    int32_t                     numClientsServiced;
    int32_t                     errCode;
    uint8_t                     innerToOuterDSCPMap[64];
    uint32_t                    index = 0;

    /* Initialize the server configuration. */
    memset ((void *)&serverConfig, 0, sizeof(Netfp_ServerConfig));

    /* Populate the server configuration.
     *  - Bypass the NETCP initialization; since this is done by the Linux kernel
     *  - PASS Configuration Virtual Address is set to 0 since the server executes
     *    on the DSP */
    strcpy (serverConfig.serverName, "SYSLIB-NETFPServer");
    serverConfig.pktlibInstHandle       = appPktlibInstanceHandle;
    serverConfig.msgcomInstHandle       = appMsgcomInstanceHandle;
    serverConfig.nrInstanceId           = Name_getDatabaseInstanceId(globalNameDatabaseHandle, &errCode);
    serverConfig.realm                  = Netfp_ExecutionRealm_DSP;
    serverConfig.serverHeapHandle       = netfpTxHeap;
    serverConfig.logFxn                 = (Netfp_ServerLogFn)Test_netfpServerLog;
    serverConfig.maxSecurityChannels    = 1024*16;
    serverConfig.baseSecurityContextId  = 2000;
    serverConfig.passCfgVirtualAddress  = 0;
    serverConfig.initSecureSubsystem    = 0;
    serverConfig.cmdHeapHandle          = paCommandHeap;
    serverConfig.ipSecHeapHandle        = netfpTxHeap;
    serverConfig.rmServiceHandle        = Resmgr_getRMServiceHandle (handleSysCfg);
    serverConfig.sysRMHandle            = handleSysCfg;
    serverConfig.malloc                 = Netfp_osalMalloc;
    serverConfig.free                   = Netfp_osalFree;
    serverConfig.mallocSecurityContext  = Netfp_osalMallocSecurityContext;
    serverConfig.freeSecurityContext    = Netfp_osalFreeSecurityContext;
    serverConfig.beginMemAccess         = Netfp_osalBeginMemoryAccess;
    serverConfig.endMemAccess           = Netfp_osalEndMemoryAccess;
    serverConfig.enterCS                = Netfp_osalEnterSingleCoreCriticalSection;
    serverConfig.exitCS                 = Netfp_osalExitSingleCoreCriticalSection;
    serverConfig.createSem              = Netfp_osalCreateSem;
    serverConfig.deleteSem              = Netfp_osalDeleteSem;
    serverConfig.postSem                = Netfp_osalPostSem;
    serverConfig.pendSem                = Netfp_osalPendSem;

    /* Initialize and create the NETFP Server: */
    netfpServerHandle = Netfp_initServer (&serverConfig, &errCode);
    if (netfpServerHandle == NULL)
    {
        System_printf ("Error: Failed to initialize the NETFP Server [Error code %d]\n", errCode);
        return;
    }

    /* Debug Message: */
    System_printf ("Debug: NETFP Server '%s' Handle %p in the DSP realm is operational\n",
                    serverConfig.serverName, netfpServerHandle);

    /* Initialize the Inner to Outer DSCP map: Inner & Outer DSCP are the same. */
    for (index = 0; index < 64; index++)
        innerToOuterDSCPMap[index] = index;

    /* Set the physical interface markings in the NETFP server */
    if (Netfp_setupPhyInterface (netfpServerHandle, "eth0", 1, &innerToOuterDSCPMap[0], &errCode) < 0)
    {
        /* Error: Unable to set the interface markings: */
        System_printf("Error: Unable to set the interface marking for eth0 [Error %d]\n", errCode);
        return;
    }

    /* Set the physical interface markings in the NETFP server */
    if (Netfp_setupPhyInterface (netfpServerHandle, "eth1", 2, &innerToOuterDSCPMap[0], &errCode) < 0)
    {
        /* Error: Unable to set the interface markings: */
        System_printf("Error: Unable to set the interface marking for eth0 [Error %d]\n", errCode);
        return;
    }

    /**************************************************************************************
     * Interface Based Routing Setup: Device Specific
     * - We need to setup the fail route information here. This is used to ensure that if a
     *   packet does NOT match the fast path rules it gets sent to Linux for processing.
     *   This is device specific since these are the Ethernet drivers queues and flows.
     *   These values should also be specified in the Master configuration files. Since we
     *   are testing the NETFP Server on DSP; we have this hardcoded here.
     **************************************************************************************/
#if (defined (DEVICE_K2H) || defined (DEVICE_K2K))
    Netfp_initInterfaceRoutingInfo(netfpServerHandle, 22, 8704, &errCode);
#elif defined (DEVICE_K2L)
    Netfp_initInterfaceRoutingInfo(netfpServerHandle, 22, 528, &errCode);
#else
#error "Error: Unsupported Device"
#endif

    /* Create a semaphore for the NETFP Server */
    netfpServerSemHandle = Semaphore_create(0, NULL, NULL);

    /***************************************************************************
     * Synch Point:
     * Ensure that all the NETFP clients have been started and registered with
     * the NETFP Server before we proceed.
     ***************************************************************************/
    index = 0;
    while (gNetfpClientNames[index] != NULL)
    {
        /* Debug Message: */
        System_printf ("Debug: Server synching for NETFP Client [%s]\n", gNetfpClientNames[index]);
        while (1)
        {
            /* Each NETFP Client requires a queue pend queue. */
            if (index >= appResourceConfig.numQpendQueues)
            {
                System_printf ("Error: Requested for %d queue pend queues; but registering %d NETFP clients\n",
                                appResourceConfig.numQpendQueues, index);
                return;
            }

            /* Populate the direct interrupt configuration. */
            directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[index].queue;
            directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[index].cpIntcId;
            directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[index].systemInterrupt;
            directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[index].hostInterrupt;

            /* Register the NETFP Client */
            if (Netfp_registerClient (netfpServerHandle, (char*)gNetfpClientNames[index], &directInterruptCfg, &errCode) == 0)
                break;

            /* Registration failed; use the error code to determine the reason. */
            if (errCode != NETFP_ENOTREADY)
            {
                System_printf ("Error: NETFP Register client failed [Error code %d]\n", errCode);
                return;
            }
            /* Client is still not active; relinquish time slice and try again */
            Task_sleep(10);
        }
        System_printf ("Debug: NETFP Client [%s] synchronized\n", gNetfpClientNames[index]);
        index = index + 1;
    }
    System_printf ("Debug: Executing NETFP Server\n");

    /* Execute the NETFP Server and provide an execution context. */
    while (1)
    {
        /* Wait for the NETFP Server semaphore */
        Semaphore_pend (netfpServerSemHandle, BIOS_WAIT_FOREVER);

        /* Execute the NETFP Server and process all the NETFP clients. */
        while (1)
        {
            numClientsServiced = Netfp_executeServer (netfpServerHandle);
            if (numClientsServiced == 0)
                break;
        }
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
    Task_Params taskParams;

    /* Initialize the system modules */
    if (Test_systemInit() < 0)
        return;

    /* Once the system modules are operational. Launch the NETFP Server */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_netfpServer, &taskParams, NULL);

    /* Launch the NETFP Server Display Task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(Test_netfpServerDebugTask, &taskParams, NULL);
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

