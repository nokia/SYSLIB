/**
 *   @file  main.c
 *
 *   @brief
 *      Unit Test Code for the DAT
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
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

/* Linux system Include Files */
#include <sys/time.h>

/* For Debugging only. */
#define System_printf   printf

/**********************************************************************
 *********************** Extern Declarations **************************
 **********************************************************************/

/* Test Setup Task: */
extern void* SetupTask(void* arg);

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

/* DAT: */
extern void* Dat_osalMalloc (uint32_t , uint32_t );
extern void  Dat_osalFree (void* , uint32_t );
extern void* Dat_osalEnterSingleCoreCriticalSection (void);
extern void  Dat_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Dat_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Dat_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Dat_osalCreateSem(void);
extern void  Dat_osalDeleteSem(void*);
extern void  Dat_osalPostSem(void*);
extern void  Dat_osalPendSem(void*);

/* MEMLOG: */
extern void* Memlog_osalMalloc (Memlog_MemAllocMode , uint32_t );
extern void  Memlog_osalFree (Memlog_MemAllocMode, void*, uint32_t );
extern void* Memlog_osalEnterCS (void);
extern void  Memlog_osalExitCS (void* csHandle);

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Global PKTLIB Heap Handle: */
Pktlib_HeapHandle       netfpClientServerHeap;
Pktlib_HeapHandle       netfpHeaderHeap;
Pktlib_HeapHandle       netfpFragHeap;
Pktlib_HeapHandle       datClientHeap;
Pktlib_HeapHandle       datConsumerHeap;
Pktlib_HeapHandle       datPMConsumerHeap;
Pktlib_HeapHandle       datProducerHeap;
Pktlib_HeapHandle       datGPProducerHeap;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* NETFP Client Handle. */
Netfp_ClientHandle      netfpClientHandle;

/* User space interrupt handle block */
UintcHandle             uintcHandle;

/* DAT Client Handle: */
Dat_ClientHandle        datClientHandle;

/* Global Name Database Handle: */
Name_DBHandle           globalNameDatabaseHandle;

/* Global variable for the application MSGCOM instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* Name Client Instance handle */
Name_ClientHandle       nameClientHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* MEMLOG Instance handle. */
Memlog_InstHandle       memlogInstHandle;

/* Test Execution status */
volatile uint32_t       testComplete = 0;
volatile uint32_t       killThreads  = 0;

/* NETFP & DAT Server Name used by the test */
const char*             gNetfpServerName = "NetfpServer_LTE9A";
const char*             gDatServerName   = "DatServer_LTE9A";

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
        }
};
/**********************************************************************
 ************************* Unit Test Functions ************************
 **********************************************************************/

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
     *  - The heap is used to exchange messages between the DAT server & client */
    strcpy(heapCfg.name, "DAT_Client_Heap");
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
    datClientHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datClientHeap == NULL)
    {
	    printf ("Error: Unable to create the DAT Client Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the heap configuration
     *  - The heap is used to receive messages from the producer. */
    strcpy(heapCfg.name, "DAT_Consumer_Heap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    datConsumerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datConsumerHeap == NULL)
    {
	    printf ("Error: Unable to create the DAT Consumer Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the heap configuration
     *  - The heap is used to receive messages from the producer. */
    strcpy(heapCfg.name, "DAT_PMConsumer_Heap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 1;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    datPMConsumerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datPMConsumerHeap == NULL)
    {
            printf ("Error: Unable to create the DAT Consumer Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the heap configuration
     *  - The heap is used to send producer debug messages from ARM producer. */
    strcpy(heapCfg.name, "DAT_Producer_Heap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 256;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    datProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datProducerHeap == NULL)
    {
        printf ("Error: Unable to create the DAT producer Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the heap configuration
     *  - The heap is used to send producer debug messages from ARM producer. */
    strcpy(heapCfg.name, "DAT_GPProducer_Heap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 128;
    heapCfg.numZeroBufferPackets            = 128;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    datGPProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (datGPProducerHeap == NULL)
    {
        printf ("Error: Unable to create the DAT General Purpose producer Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - The heap is used to exchange messages between the NETFP Client & Server. */
    strcpy(heapCfg.name, "NETFP_ClientServerPktHeap");
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 32;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 8;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = myMalloc;
    heapCfg.heapInterfaceTable.dataFree     = myFree;
    netfpClientServerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (netfpClientServerHeap == NULL)
    {
	    printf ("Error: Unable to create the NETFP Client Server Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "NETFP_HeaderHeap");
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
	    printf ("Error: Unable to create the NETFP Header Heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration
     *  - This heap is used to send fragmented packets provided the fragmentation is done
     *    in NETFP software. */
    strcpy(heapCfg.name, "NETFP_FragmentHeap");
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
	    printf ("Error: Unable to create the NETFP Fragmentation Heap [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
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
    printf ("Debug: Shutting down the UINTC Thread\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      NETFP Client execution task
 *
 *  @retval
 *      Not Applicable.
 */
static void* Netfp_ClientExecutionTask(void* arg)
{
    /* Execute the NETFP Client. The thread will be cancelled on process termination. */
    while (killThreads == 0)
        Netfp_executeClient (netfpClientHandle);
    printf ("Debug: Shutting down the NETFP Client Thread\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      DAT Client execution task
 *
 *  @retval
 *      Not Applicable.
 */
static void* Dat_ClientExecutionTask(void* arg)
{
    /* Execute the DAT Client: This is executed in a polled mode since the
     * DAT clients execute in the control path. */
    while (killThreads == 0)
    {
        Dat_executeClient (datClientHandle);
        Dat_executeBackgroundActions (datClientHandle);
    }
    printf ("Debug: Shutting down the DAT Client Thread\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the execution thread which provides an execution context for
 *      the Name client
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* Name_ClientExecutionTask(void *arg)
{
    Name_ClientHandle   nameClientHandle;

    /* Get the Name client handle */
    nameClientHandle = (Name_ClientHandle)arg;

    /* Execute the name client thread. */
    while (killThreads == 0)
    {
        /* Execute the name client */
        Name_executeClient (nameClientHandle);

        /* Relinquish time slice allowing other tasks to execute */
        usleep(100);
    }
    printf ("Debug: Shutting down the Name Client Thread\n");
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
int32_t main (void)
{
    Resmgr_ResourceCfg*         ptrResCfg;
    pthread_t                   testThread;
    pthread_t                   uintcThread;
    pthread_t                   netfpClientThread;
    pthread_t                   datClientThread;
    pthread_t                   nameThread;
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    struct sigaction            act;
    Netfp_ClientConfig          clientConfig;
    UintcConfig                 uintcConfig;
    Dat_ClientCfg               datClientCfg;
    int32_t                     clientStatus;
    Netfp_ServerHandle          netfpServerHandle;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    Name_DatabaseCfg            databaseCfg;
    Name_ClientCfg              clientCfg;
    Memlog_InstCfg              memlogInstConfig;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId  = 1;
    databaseCfg.realm       = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "LTE9A_L3");

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

    /* Get the application specific configuration */
    ptrResCfg = &appResourceConfig;

    /* Use the sa_sigaction field because the handles has two additional parameters */
    act.sa_sigaction = &Test_terminated;
    act.sa_flags     = SA_SIGINFO;

    /* Setup the signals. */
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGKILL, &act, NULL);
    sigaction(SIGINT,  &act, NULL);

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

    /* Populate the user space interrupt configuration:
     * - We are operating the UINTC module in the UINTC managed mode. */
    strcpy(uintcConfig.name, "TestUINTC");
    uintcConfig.mode = Uintc_Mode_UINTC_MANAGED;
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
    strcpy (clientCfg.clientName, "DAT_UnitTestARM");
    strcpy (clientCfg.proxyName, "NameServer_LTE9A");
    clientCfg.databaseHandle            = globalNameDatabaseHandle;
    clientCfg.realm                     = Name_ExecutionRealm_ARM;
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
            printf ("Error: Name client creation failed [Error code %d]\n", errCode);
    }
    printf ("Debug: Name client %p has been created successfully\n", nameClientHandle);

    /* Name Client Execution Thread: The thread is responsible for executing the name client */
    errCode = pthread_create (&nameThread, NULL, Name_ClientExecutionTask, nameClientHandle);
    if (errCode < 0)
    {
    	printf ("Error: Agent Thread failed to start error code %d \n", errCode);
        return -1;
    }

    /* SYNC Point: Ensure that the NETFP Server has been started. This is required before
     * we create and register the NETFP client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready. The NETFP Server & Client are
         * executing in the same realm. */
        netfpServerHandle = Netfp_startServer (globalNameDatabaseHandle, NULL, (char*)gNetfpServerName, &errCode);
        if (netfpServerHandle != NULL)
            break;

        /* Check the error code. */
        if (errCode != NETFP_ENOTREADY)
        {
            printf ("Error: NETFP Starting server failed [Error code %d]\n", errCode);
            return -1;
        }
        usleep(10);
    }

    /* Initialize the client configuration */
    memset ((void *)&clientConfig, 0, sizeof(Netfp_ClientConfig));

    /* Populate the client configuration.
     * - Name Client Handle is set to NULL since the NETFP Server and clients are
     *   both executing in the same realm. */
    strcpy (clientConfig.serverName, gNetfpServerName);
    strcpy (clientConfig.clientName, "Netfp_Client_LTE9A_TEST1");
    clientConfig.serverHandle                       = netfpServerHandle;
    clientConfig.nrInstanceId                       = databaseCfg.instanceId;
    clientConfig.msgcomInstHandle                   = appMsgcomInstanceHandle;
    clientConfig.directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    clientConfig.directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    clientConfig.directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    clientConfig.directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;
    clientConfig.pktlibInstHandle                   = appPktlibInstanceHandle;
    clientConfig.nameClientHandle                   = NULL;
    clientConfig.netHeaderHeapHandle                = netfpHeaderHeap;
    clientConfig.fragmentHeap		            = netfpFragHeap;
    clientConfig.realm                              = Netfp_ExecutionRealm_ARM;
    clientConfig.clientHeapHandle                   = netfpClientServerHeap;
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

    /* Loop around and synchronize the NETFP client registeration with the server. */
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
        usleep(1);
    }
    printf ("Debug: NETFP Client has been registered successfully with the server\n");

    /* SYNC Point: Ensure that the DAT Server has been started. This is required before
     * we create and register the DAT client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready; since the DAT server and
         * client are executing in the same realm there is no need to specify the NAME client
         * handle. */
        if (Dat_startServer (globalNameDatabaseHandle, NULL, (char*)gDatServerName, &errCode) != NULL)
            break;

        /* Check the error code. */
        if (errCode != DAT_ENOTREADY)
        {
            printf ("Error: DAT Starting server failed [Error code %d]\n", errCode);
            return -1;
        }
        usleep(1);
    }

    /* Initialize the DAT client configuration */
    memset ((void *)&datClientCfg, 0, sizeof(Dat_ClientCfg));

    /* Populate the DAT client configuration
     * - There is no need to specify a name client handle since the DAT client
     *   will act as a consumer for a producer which executes in another realm. */
    strcpy (datClientCfg.clientName, "Dat_Client_LTE9A_TEST1");
    strcpy (datClientCfg.serverName, gDatServerName);
    datClientCfg.pktlibInstHandle  = appPktlibInstanceHandle;
    datClientCfg.msgcomInstHandle  = appMsgcomInstanceHandle;
    datClientCfg.databaseHandle    = globalNameDatabaseHandle;
    datClientCfg.clientHeapHandle  = datClientHeap;
    datClientCfg.nameClientHandle  = NULL; 
    datClientCfg.id                = 0xFF;
    datClientCfg.realm             = Dat_ExecutionRealm_ARM;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.mallocLocal       = Dat_osalMalloc;
    datClientCfg.freeLocal         = Dat_osalFree;
    datClientCfg.beginMemAccess    = Dat_osalBeginMemoryAccess;
    datClientCfg.endMemAccess      = Dat_osalEndMemoryAccess;
    datClientCfg.createSem         = Dat_osalCreateSem;
    datClientCfg.deleteSem         = Dat_osalDeleteSem;
    datClientCfg.postSem           = Dat_osalPostSem;
    datClientCfg.pendSem           = Dat_osalPendSem;
    datClientCfg.enterCS           = Dat_osalEnterSingleCoreCriticalSection;
    datClientCfg.exitCS            = Dat_osalExitSingleCoreCriticalSection;
    datClientCfg.logSync.syncPeriod = 100; /* in ms */

    /* Initialize the DAT client. */
    datClientHandle = Dat_initClient (&datClientCfg, &errCode);
    if (datClientHandle == NULL)
    {
        printf ("Error: Unable to create the DAT client [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT client %p created successfully\n", datClientHandle);

    /* Start the DAT client: DAT clients can only be started after they have been registered
     * by the server */
    while (1)
    {
        /* Start the DAT client */
        clientStatus = Dat_startClient (datClientHandle, &errCode);
        if (clientStatus < 0)
        {
            printf ("Error: DAT Client registration status failed [Error code %d]\n", errCode);
            return -1;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        usleep(1);
    }

    /*******************************************************************************
     * TEST: Creating MEMLOG instance
     *******************************************************************************/

    /* Initialize and create the MEMLOG instance */
    memset ((void *)&memlogInstConfig, 0, sizeof(Memlog_InstCfg));

    memlogInstConfig.databaseHandle    = globalNameDatabaseHandle;
    memlogInstConfig.pktlibInstHandle  = appPktlibInstanceHandle;
    memlogInstConfig.msgcomInstHandle  = appMsgcomInstanceHandle;
    memlogInstConfig.realm             = Memlog_ExecutionRealm_ARM;
    memlogInstConfig.malloc            = Memlog_osalMalloc;
    memlogInstConfig.free              = Memlog_osalFree;
    memlogInstConfig.enterCS           = Memlog_osalEnterCS;
    memlogInstConfig.exitCS            = Memlog_osalExitCS;

    if ((memlogInstHandle =  Memlog_createInstance(&memlogInstConfig, &errCode)) == NULL)
    {
        System_printf ("Error: MEMLOG create instance Failed [Error Code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: MEMLOG instance has been created successfully\n");

    /* Create the NETFP client execution task. */
    errCode = pthread_create (&netfpClientThread, NULL, Netfp_ClientExecutionTask, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the DAT client execution task. */
    errCode = pthread_create (&datClientThread, NULL, Dat_ClientExecutionTask, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread%d\n", errCode);
        return -1;
    }

    /* Create the UINTC task. */
    errCode = pthread_create (&uintcThread, NULL, UintcThread, NULL);
    if (errCode < 0)
    {
        printf ("Error: UINTC thread create failed error code %d\n", errCode);
        return -1;
    }

    /* Create the DAT Test setup task */
    errCode = pthread_create (&testThread, NULL, SetupTask, NULL);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the test thread %d\n", errCode);
        return -1;
    }

    /* Blocked till the tests are done. */
    pthread_join (testThread, NULL);

    /* Stop the NETFP client */
    if (Netfp_stopClient (netfpClientHandle, &errCode) < 0)
        printf ("Error: NETFP Client stop failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Client stop successfully\n");

    /* Stop the DAT client */
    if (Dat_stopClient (datClientHandle, &errCode) < 0)
        printf ("Error: DAT Client stop failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT Client stop successfully\n");

    /* Debug Message: */
    printf ("Debug: Shutdown the UINTC, NETFP, Agent  & DAT threads\n");

    /* Indicate that the threads are to be shutdown. */
    killThreads = 1;

    /* Tests have been completed: Kill the NETFP Client, DAT client, Agent Client & UINTC threads
     * - Agent & DAT threads are polled so they will terminate by themselves. UINTC and NETFP client
     *   threads are terminated explicitly. */
    pthread_cancel (uintcThread);
    pthread_cancel (netfpClientThread);
    pthread_join (datClientThread, NULL);
    pthread_join (nameThread, NULL);

    /* Delete the NETFP client */
    if (Netfp_deleteClient (netfpClientHandle, &errCode) < 0)
        printf ("Error: NETFP Client deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Client deleted successfully\n");

    /* Delete the DAT client */
    if (Dat_deleteClient (datClientHandle, &errCode) < 0)
        printf ("Error: DAT Client deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT Client deleted successfully\n");

    /* Test execution is complete. Shutdown and delete the name client */
    if (Name_deleteClient (nameClientHandle, &errCode) < 0)
        printf ("Error: Agent client deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: Agent client deleted successfully\n");

    /* Shutdown all the PKTLIB Heaps which had been created */
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpClientServerHeap, &errCode) < 0)
        printf ("Error: NETFP Client Server Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Client Server Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpHeaderHeap, &errCode) < 0)
        printf ("Error: NETFP Header Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Header Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, netfpFragHeap, &errCode) < 0)
        printf ("Error: NETFP Fragmentation Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: NETFP Fragmentation Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, datClientHeap, &errCode) < 0)
        printf ("Error: DAT Client Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT Client Heap deletion successful\n");
    if (Pktlib_deleteHeap (appPktlibInstanceHandle, datConsumerHeap, &errCode) < 0)
        printf ("Error: DAT Consumer Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT Consumer Heap deletion successful\n");

    if (Pktlib_deleteHeap (appPktlibInstanceHandle, datPMConsumerHeap, &errCode) < 0)
        printf ("Error: DAT PM Consumer Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT PM Consumer Heap deletion successful\n");

    if (Pktlib_deleteHeap (appPktlibInstanceHandle, datProducerHeap, &errCode) < 0)
        printf ("Error: DAT Producer Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT Producer Heap deletion successful\n");

    if (Pktlib_deleteHeap (appPktlibInstanceHandle, datGPProducerHeap, &errCode) < 0)
        printf ("Error: DAT GPProducer Heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT GPProducer Heap deletion successful\n");

    /* Shutdown the UINTC module: This will initiate the shutdown of the UINTC thread. */
    if (Uintc_deinit (uintcHandle, &errCode) < 0)
    	printf("Error: UINTC module deinitialization failed [Error code %d]\n", errCode);
    else
        printf("Debug: UINTC module deinitializated successfully \n");

    /* Cleanup the resource manager configuration */
    if (Resmgr_deinit (handleSysCfg, &errCode) < 0)
        printf ("Error: Shutting down the system configuration failed\n");
    else
        printf ("Debug: Shutting down the system configuration passed\n");
    return 0;
}

