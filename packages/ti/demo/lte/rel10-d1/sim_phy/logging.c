/**
 *   @file  logging.c
 *
 *   @brief
 *      Sample simulated phy code which provides the logging infrastructure
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
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_device_interrupt.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include "sim_phy.h"

/* Logger streamer Include Files */
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* Simulated PHY Domain MCB */
extern AppSimPHYDomainMCB    myAppDomain;

/* Cache Invalidate API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 *************************** Logging Functions ************************
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
static uint8_t* Log_dataHeapMalloc(uint32_t size, uint32_t arg)
{
    Error_Block	errorBlock;

    /* Allocate memory from the shared memory heap. */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)arg, size, 0, &errorBlock);
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
static void Log_dataHeapFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    Memory_free ((xdc_runtime_IHeap_Handle)arg, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      This function is used to generating UIA messages for testing purpose.
 *
 *  @param[in]  arg0
 *      Not used.
 *  @param[in]  arg1
 *      Not used.
 *  @retval
 *      Not Applicable.
 */
static void Log_loggingMsgGenerator(UArg arg0, UArg arg1)
{
    uint32_t    logMsgCount = 0;
    uint32_t    index;

    while(1)
    {
        /* Generating log messages with a predefined load. */
        for(index = 0; index < 10; index ++)
            Log_write3(UIAEvt_detailWithStr, 0x10, (IArg)"Rel10 simPHY logging seq=%d", logMsgCount++);
		Task_sleep(1000);
    }
}

/**
 *  @b Description
 *  @n
 *      The function creates a socket used for DAT producer debug stream
 *
 *  @param[in]  netfpClientHandle
 *      Netfp client handle.
 *
 *  @retval
 *      socket Handle.
 */
static Netfp_SockHandle Log_datCreateDebugSocket(Netfp_ClientHandle  netfpClientHandle, int32_t* errCode)
{
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockAddr          sockAddress;
    Netfp_SockHandle        datDebugSockHandle;

	/* Find the ingress fast path handle */
	while (1)
	{
		fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress_FastPath_Debug", errCode);
		if (fpIngressHandle != NULL)
			break;
		Task_sleep(1);
	}

	/* Find the egress fast path handle */
	while (1)
	{
		fpEgressHandle = Netfp_findOutboundFastPath(netfpClientHandle, "Egress_FastPath_Debug", errCode);
		if (fpEgressHandle != NULL)
			break;
		Task_sleep(1);
	}

    /* Create Netfp socket */
	datDebugSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, errCode);
	if (datDebugSockHandle == NULL)
	{
		System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", *errCode);
		return NULL;
	}

	/* Populate the binding information */
	memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                = 3000 + DNUM ;
	sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
	sockAddress.op.bind.flowId          = 0xFFFFFFFF;
	sockAddress.op.bind.queueHandle     = NULL;

	/* Bind the socket. */
	if (Netfp_bind (datDebugSockHandle, &sockAddress, errCode) < 0)
	{
		System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", *errCode);
		return NULL;
	}

	/* Populate the connect information. */
	sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
	sockAddress.sin_port                    = 1235;
	sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

	/* Connect the socket */
	if (Netfp_connect(datDebugSockHandle, &sockAddress, errCode) < 0)
	{
		System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", *errCode);
		return NULL;
	}
	return datDebugSockHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the logging for the L2 Data plane
 *      Logging will allow the L2 DP to use the UIA logging streamers to
 *      generate and stream out data to the System analyzer.
 *
 *  @param[in]  ptrSimPhyDomain
 *      Pointer to the Simulated PHY Domain application MCB
 *  @param[in]  producerName
 *      Producer name
 *  @param[in]  ptrResourceCfg
 *      Pointer to the resource configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Log_initLogging
(
    AppSimPHYDomainMCB* ptrSimPhyDomain,
    char*               producerName,
    Resmgr_ResourceCfg* ptrResourceCfg
)
{
    Dat_ProducerCfg         producerCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Task_Params             taskParams;
    uint32_t                memLoggingSize = 1024*1024;
    uint32_t                memLoggingBase = 0;
    Memlog_ChannelCfg       memlogChanConfig;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration:
     *  - The heap is used for the exchange of messages between the L1 DAT Producer and the L2 Consumer */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "DatProducerHeap_%d", ptrSimPhyDomain->appId);
    heapCfg.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrSimPhyDomain->syslibHandle);
    heapCfg.memRegion                       = ptrResourceCfg->memRegionResponse[1].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 256;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
    heapCfg.heapInterfaceTable.dataMalloc   = Log_dataHeapMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Log_dataHeapFree;
    ptrSimPhyDomain->datProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (ptrSimPhyDomain->datProducerHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT producer heap [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: DAT Producer Heap [Free Queue %x]\n", Pktlib_getInternalHeapQueue(ptrSimPhyDomain->datProducerHeap));

    /*******************************************************************************
     * TEST: Creating MEMLOG channel for core0_GPProducer
     *******************************************************************************/

    /* Initialize and create MEMLOG channel */
    memset((void *)&memlogChanConfig, 0, sizeof(Memlog_ChannelCfg) );

    /* The memory should be reserved for Memory logging in DDR.
       In this demo, it uses memory from LTEn_DDR3_L1SLAVE or LTEn_DDR3_L1MASTER
       ddr3MemLoggingHeap is created in *.cfg file */
    memLoggingBase = (uint32_t)Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3MemLoggingHeap,
                                                 memLoggingSize,
                                                 MEMLOG_MEMORY_ALIGNMENT,
                                                 NULL);

    sprintf(memlogChanConfig.name, "%s%d", producerName, DNUM);
    memlogChanConfig.memlogInstHandle = Domain_getMemlogInstanceHandle(ptrSimPhyDomain->syslibHandle);
    memlogChanConfig.memRegion        = ptrResourceCfg->memRegionResponse[1].memRegionHandle;
    memlogChanConfig.memBaseAddr      = memLoggingBase;
    memlogChanConfig.memSize          = memLoggingSize;
    memlogChanConfig.bufferSize       = 1408;
    memlogChanConfig.numPktDescs      = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(ptrSimPhyDomain->datProducerHeap));

    if ((ptrSimPhyDomain->memlogChanHandle = Memlog_create(&memlogChanConfig, &errCode)) == NULL)
    {
        System_printf ("Error: MEMLOG create channel Failed [Error Code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: MEMLOG channel(%p) has been created successfully\n", ptrSimPhyDomain->memlogChanHandle );

    /* Push the channel name from the DSP to the ARM realm. */
    if (Name_push (Domain_getNameClientInstanceHandle(ptrSimPhyDomain->syslibHandle), memlogChanConfig.name, Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        System_printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", memlogChanConfig.name, errCode);
        return -1;
    }

    /* Initialize the producer configuration. */
    memset ((void*)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    sprintf(producerCfg.name, "%s%d",producerName, DNUM);
    producerCfg.heapHandle           = ptrSimPhyDomain->datProducerHeap;
    producerCfg.debugSocketHandle    = NULL;
    producerCfg.loggerStreamerHandle = logger0;
    producerCfg.producerType         = DAT_PRODUCER_UIA;
    producerCfg.memlogChanHandle     = ptrSimPhyDomain->memlogChanHandle;

    /* Instantiate the streaming socket only if the NETFP Client has been created? */
    if (Domain_getNetfpClientInstanceHandle(ptrSimPhyDomain->syslibHandle) != NULL)
    {
        /* Create the streaming socket. */
        producerCfg.debugSocketHandle = Log_datCreateDebugSocket(Domain_getNetfpClientInstanceHandle(ptrSimPhyDomain->syslibHandle), &errCode);
        if(producerCfg.debugSocketHandle == NULL)
        {
            System_printf ("Error: Unable to create the Dat debug socket \n", errCode);
            return -1;
        }
    }
    else
    {
        /* No NETFP client; no debug streaming. */
        producerCfg.debugSocketHandle = ptrSimPhyDomain->memlogChanHandle;
    }

    /* Create the producer */
    ptrSimPhyDomain->producerHandle = Dat_createProducer (Domain_getDatClientInstanceHandle(ptrSimPhyDomain->syslibHandle),
                                                          &producerCfg, NULL, &errCode);
    if (ptrSimPhyDomain->producerHandle == NULL)
    {
        System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                   producerCfg.name, ptrSimPhyDomain->producerHandle);

    /* Launch logging test task: */
    Task_Params_init(&taskParams);
    taskParams.priority         = 4;
    taskParams.stackSize        = 16*1024;
    Task_create(Log_loggingMsgGenerator, &taskParams, NULL);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the logging setup and is invoked
 *      when the application is going down.
 *
 *  @param[in]  ptrSimPhyDomain
 *      Pointer to the Simulated PHY Domain application MCB
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Log_deinitLogging (AppSimPHYDomainMCB* ptrSimPhyDomain)
{
    int32_t errCode;

    /* Debug Message: */
    System_printf ("Debug: Shutting down the logging infrastructure\n");

    /* Delete the producer: Producers can only be deleted once all the consumers have been
     * disconnected and deleted. */
    while (1)
    {
        if (Dat_deleteProducer (ptrSimPhyDomain->producerHandle, &errCode) == 0)
            break;

        if (errCode != DAT_EINUSE)
        {
            System_printf ("Error: DAT Delete Producer failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Relinquish time slice. */
        Task_sleep(1);
    }

    /* Delete MEMLOG channel */
    if(ptrSimPhyDomain->memlogChanHandle)
    {
        /* Delete Memory logging channel */
        if ( memlog_delete(ptrSimPhyDomain->memlogChanHandle, &errCode) < 0)
        {
            System_printf("Delete memlog channel failed with error Code=%d\n", errCode);
        }
        System_printf("Memlog channel has been deleted!\n");
    }

    /* Shutdown the PKTLIB heaps */
    if (Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrSimPhyDomain->syslibHandle),
                           ptrSimPhyDomain->datProducerHeap, &errCode) < 0)
        System_printf ("Error: PKTLIB DAT producer heap deletion failed [Error code %d]\n", errCode);
    else
        System_printf ("Debug: PKTLIB DAT producer heap deleted successfully\n");
    return 0;
}

