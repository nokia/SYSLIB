/*
 *   @file  main_core0.c
 *
 *   @brief
 *      DAT Test Framework for Core0
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
#include <ti/drv/rm/rm.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Maximum number of IPSEC Tunnels which are opened by the System. */
#define NETFP_NUM_MAX_TUNNELS      10

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Local PKTLIB Heaps */
Pktlib_HeapHandle       netfpHeaderHeap;
Pktlib_HeapHandle       netfpFragHeap;
Pktlib_HeapHandle       datConsumerHeap;
Pktlib_HeapHandle       datClientHeap;
Pktlib_HeapHandle       datUIAProducerHeap;
Pktlib_HeapHandle       datProducerHeap1;
Pktlib_HeapHandle       datProducerHeap2;
Pktlib_HeapHandle       nameProxyHeapHandle;
Pktlib_HeapHandle       nameClientProxyHeapHandle;

/* System Configuration Handle. */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global database handle: */
Name_DBHandle           globalNameDatabaseHandle;

/* NETFP Client Handle: */
Netfp_ClientHandle      netfpClientHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* NETFP Client Handle: */
Netfp_ClientHandle      netfpClientHandle;

/* Name Handle: */
Name_ProxyHandle        nameProxyHandle;
Name_ClientHandle       nameClientHandle;

/* NETFP & Name Proxy Name used by the test */
const char*             gNetfpServerName = "NetfpServer_LTE9A";
const char*             gNameProxyName   = "NameServer_LTE9A";

/* Application requested resources */
Resmgr_ResourceCfg      appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
    1,    /* Number of Accumulator Channels requested                         */
    0,    /* Number of Hardware Semaphores requested                          */
    3,    /* Number of QPEND Queues requested                                 */
    /* Requested Memory Region Configuration. */
    {
        /* Name,           Type,                    Linking RAM,                              Num,    Size */
        { "Core0", Resmgr_MemRegionType_DDR3,   Resmgr_MemRegionLinkingRAM_DONT_CARE,    4096,   128},
    }
};

/**********************************************************************
 *********************** Unit Test Extern Definitions *****************
 **********************************************************************/

/* Application call back functions. */
extern void Test_core0Task(UArg arg0, UArg arg1);

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
     *  - The heap is used to create networking headers which are populated by the NETFP
     *    library for every packet which is to be transmitted. */
    strcpy(heapCfg.name, "Core0_NETFP_HeaderHeap");
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
    strcpy(heapCfg.name, "Core0_NETFP_FragmentHeap");
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
    strcpy(heapCfg.name, "Core0_Dat_MsgHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 256;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    datConsumerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datConsumerHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT consumer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used for the reception and transmission of all packets on this core */
    strcpy(heapCfg.name, "Dat_Client_ReceiveHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    datClientHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datClientHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT Client heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used by the DAT General producer 1 to generate messages */
    strcpy(heapCfg.name, "Core0_DAT_UIAProducerHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    datUIAProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datUIAProducerHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT Producer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used by the DAT General producer 1 to generate messages */
    strcpy(heapCfg.name, "Core0_DAT_GPProducerHeap1");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    datProducerHeap1 = Pktlib_createHeap(&heapCfg, &errCode);
    if (datProducerHeap1 == NULL)
    {
        System_printf ("Error: Unable to create the DAT Producer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used by the DAT General producer 2 to generate messages */
    strcpy(heapCfg.name, "Core0_DAT_GPProducerHeap2");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    datProducerHeap2 = Pktlib_createHeap(&heapCfg, &errCode);
    if (datProducerHeap2 == NULL)
    {
        System_printf ("Error: Unable to create the DAT Producer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used to exchange messages with the peer name proxy. */
    strcpy(heapCfg.name, "Core0_NameProxyHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    nameProxyHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (nameProxyHeapHandle == NULL)
    {
        System_printf ("Error: Unable to create the Agent heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used to exchange messages between the local name proxy and the name clients
     *    and also to send receive messages between the NETFP client and server. */
    strcpy(heapCfg.name, "NameProxyClientHeap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;
    nameClientProxyHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (nameClientProxyHeapHandle == NULL)
    {
        System_printf ("Error: Unable to create the Agent heap [Error code %d]\n", errCode);
        return -1;
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
 *      Agent Execution Task: On the core we have created an agent client
 *      and so we are responsible for executing the client.
 *
 *  @retval
 *      Not Applicable.
 */
static void NameClientProxyExecutionTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the name proxy */
        if (Name_executeProxy (nameProxyHandle, &errCode) < 0)
        {
            System_printf ("FATAL Error: Name Proxy execution failed [Error code %d]\n", errCode);
            return;
        }

        /* Execute the name client */
        Name_executeClient (nameClientHandle);

        /* Relinquish time slice allowing other tasks to execute */
        Task_sleep(2);
    }
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
static int32_t Test_systemInit(void)
{
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    Task_Params                 taskParams;
    Netfp_ClientConfig          clientConfig;
    int32_t                     clientStatus;
    Netfp_ServerHandle          netfpServerHandle;
    Name_ClientCfg              clientCfg;
    Name_ProxyCfg               proxyCfg;

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

    /* Initialize the named resource domain. */
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

    /* Initialize and create the PKTLIB heaps required. */
    if (Test_createPktlibHeaps() < 0)
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
        System_printf ("Error: MSGCOM Instance creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the name proxy configuration */
    memset ((void *)&proxyCfg, 0, sizeof(Name_ProxyCfg));

    /* Populate the configuration: */
    strcpy (proxyCfg.proxyName, gNameProxyName);
    proxyCfg.realm                          = Name_ExecutionRealm_DSP;
    proxyCfg.sharedMemoryAddress            = LTE1_DDR3_NAME_PROXY_RSVD;
    proxyCfg.databaseHandle                 = globalNameDatabaseHandle;
    proxyCfg.localFlowId                    = 1;
    proxyCfg.remoteFlowId                   = 0;
    proxyCfg.pktlibInstHandle               = appPktlibInstanceHandle;
    proxyCfg.proxyHeapHandle                = nameProxyHeapHandle;
    proxyCfg.logFxn                         = NULL;
    proxyCfg.malloc                         = Name_osalMalloc;
    proxyCfg.free                           = Name_osalFree;
    proxyCfg.beginMemAccess                 = Name_osalBeginMemAccess;
    proxyCfg.endMemAccess                   = Name_osalEndMemAccess;
    proxyCfg.u.dspCfg.clientProxyHeapHandle = nameClientProxyHeapHandle;
    proxyCfg.u.dspCfg.msgcomInstHandle      = appMsgcomInstanceHandle;

    /* Create the Name Proxy */
    nameProxyHandle = Name_initProxy (&proxyCfg, &errCode);
    if (nameProxyHandle == NULL)
    {
        System_printf ("Error: Unable to create the name proxy [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name proxy %p has been created successfully\n", nameProxyHandle);

    /* Ensure that the name proxy in the realms have synchronized. This is done to ensure that
     * the name clients do not start using these services before the synchronization is done
     * else the requests are lost */
    while (1)
    {
        int32_t synchStatus;

        /* Get the proxy synchronization status. */
        synchStatus = Name_isProxySynched (nameProxyHandle, &errCode);
        if (synchStatus < 0)
        {
            System_printf ("Error: PROXY Synchronization failed [Error code %d]\n", errCode);
            return -1;
        }
        if (synchStatus == 1)
            break;
        Task_sleep(10);
    }
    System_printf ("Debug: Name proxy synchronized\n");

    /* Name Proxy is operational: Initialize the name client */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "NameClient_LTE9A_L2SCHED");
    strcpy (clientCfg.proxyName, proxyCfg.proxyName);
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

    /* Create the name client */
    nameClientHandle = Name_initClient (&clientCfg, &errCode);
    if (nameClientHandle == NULL)
    {
        System_printf ("Error: Unable to create the name client [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Name client %p has been created successfully\n", nameClientHandle);

    /* Launch the Name client/proxy execution task: */
    Task_Params_init(&taskParams);
    taskParams.priority         = 5;
    taskParams.stackSize        = 16*1024;
    Task_create(NameClientProxyExecutionTask, &taskParams, NULL);

    /* SYNC Point: Ensure that the NETFP Server has been started. This is required before
     * we create and register the NETFP client. */
    while (1)
    {
        /* Try to start the server: Since the NETFP Server & Client are running on different executing realms. */
        netfpServerHandle = Netfp_startServer (globalNameDatabaseHandle, nameClientHandle, (char*)gNetfpServerName, &errCode);
        if (netfpServerHandle != NULL)
            break;

        /* Check the error code. */
        if (errCode != NETFP_ENOTREADY)
        {
            System_printf ("Error: NETFP Starting server failed [Error code %d]\n", errCode);
            return -1;
        }
        Task_sleep(1);
    }

    /* Initialize the client configuration */
    memset ((void *)&clientConfig, 0, sizeof(Netfp_ClientConfig));

    /* Populate the client configuration.
     * - Agent Client Handle is required since the NETFP Server & clients are
     *   executing in different realms. */
    strcpy (clientConfig.serverName, gNetfpServerName);
    if (DNUM == 0)
        sprintf (clientConfig.clientName, "Netfp_Client_LTE9A_L2DP");
    else
        sprintf (clientConfig.clientName, "Netfp_Client_LTE9A_L1");
    clientConfig.nrInstanceId                       = databaseCfg.instanceId;
    clientConfig.serverHandle                       = netfpServerHandle;
    clientConfig.pktlibInstHandle                   = appPktlibInstanceHandle;
    clientConfig.directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    clientConfig.directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    clientConfig.directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    clientConfig.directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;
    clientConfig.nameClientHandle                   = nameClientHandle;
    clientConfig.netHeaderHeapHandle                = netfpHeaderHeap;
    clientConfig.fragmentHeap                       = netfpFragHeap;
    clientConfig.realm                              = Netfp_ExecutionRealm_DSP;
    clientConfig.clientHeapHandle                   = nameClientProxyHeapHandle;
    clientConfig.msgcomInstHandle                   = appMsgcomInstanceHandle;
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
        System_printf ("Error: NETFP Client Initialization failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: NETFP Client [%s] Initialized %p\n", clientConfig.clientName, netfpClientHandle);

    /* Loop around and synchronize the client registration with the server. */
    while (1)
    {
        /* Start the NETFP client */
        clientStatus = Netfp_startClient (netfpClientHandle, &errCode);
        if (clientStatus < 0)
        {
            System_printf ("Error: Client registration status failed [Error code %d]\n", errCode);
            return -1;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        Task_sleep(1);
    }
    System_printf ("Debug: NETFP Client [%s] registered successfully with the server\n", clientConfig.clientName);

    /* Launch the NETFP Client Execution Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 5;
    Task_create(Test_netfpClient, &taskParams, NULL);

    /* System has been initialized successfully */
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
static void Test_sysInitTask(UArg arg0, UArg arg1)
{
    Task_Params taskParams;

    /* Initialize the System. */
    if (Test_systemInit() < 0)
        return;

    /* Core0 Test Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 5;
    Task_create(Test_core0Task, &taskParams, NULL);
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

    System_printf ("*********************************************\n");
    System_printf ("*********** Debug & Trace Unit Test *********\n");
    System_printf ("*********************************************\n");

    /* Enable the caches. */
    CACHE_setL1DSize (CACHE_L1_32KCACHE);
    CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Enable the timestamp */
    TSCL = 0;

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* Initialize & create the system initialization task */
    Task_Params_init(&taskParams);
    taskParams.priority  = 5;
    taskParams.stackSize = 32*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

