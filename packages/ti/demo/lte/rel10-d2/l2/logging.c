/**
 *   @file  logging.c
 *
 *   @brief
 *      Sample L2 code which initializes the logging infrastructure
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
#include "l2_lte.h"

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* LTE Stack Domain MCB */
extern AppLTEStackDomainMCB    myAppDomain;

/* Cache Invalidate API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/**********************************************************************
 *************************** Log Functions ****************************
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
 *      This task to consumer buffers for all created consumers.
 *
 *  @retval
 *      Not Applicable.
 */
static void Log_consumerTask(UArg arg0, UArg arg1)
{
    Ti_Pkt*         ptrMessage;
    uint8_t*        ptrBuffer;
    uint32_t        bufferSize;
    int32_t         errCode;
    AppLTEStackDomainMCB* ptrLTEStackDomain;

    /* Get the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg0;

    /* This task is created after consumer is created. */
    while(1)
    {
        /* Poll PHY master consumer */
        if ( ptrLTEStackDomain->consumerPhyMasterHandle)
        {
            /* Poll all the messages for the consumer (non-pacing consumer) */
            do
            {
                /* Poll message for consumer */
                ptrMessage = Dat_processConsumer(ptrLTEStackDomain->consumerPhyMasterHandle);
                if(ptrMessage)
                {
                    /* Get the data buffer and size */
                    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);
    
                    /* Invalidate the data buffer contents */
                    appInvalidateBuffer (ptrBuffer, bufferSize);
    
                    /* Send the message out via the DAT Socket Handle */
                    if (Netfp_send (myAppDomain.datSockHandle, ptrMessage, 0, &errCode) < 0)
                    {
                        System_printf ("Debug: Unable to consume messages via NETFP [Error code %d]\n", errCode);
                        Pktlib_freePacket (Domain_getPktlibInstanceHandle(myAppDomain.syslibHandle), ptrMessage);
                    }
                }
            }while(ptrMessage);
        }

        /* Poll PHY slave consumer */
        if ( ptrLTEStackDomain->consumerPhySlaveHandle)
        {
            /* Poll all the messages for the consumer (non-pacing consumer) */
            do
            {
                /* Poll message for consumer */
                ptrMessage = Dat_processConsumer(ptrLTEStackDomain->consumerPhySlaveHandle);
                if(ptrMessage)
                {
                    /* Get the data buffer and size */
                    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);

                    /* Invalidate the data buffer contents */
                    appInvalidateBuffer (ptrBuffer, bufferSize);

                    /* Send the message out via the DAT Socket Handle */
                    if (Netfp_send (myAppDomain.datSockHandle, ptrMessage, 0, &errCode) < 0)
                    {
                        System_printf ("Debug: Unable to consume messages via NETFP [Error code %d]\n", errCode);
                        Pktlib_freePacket (Domain_getPktlibInstanceHandle(myAppDomain.syslibHandle), ptrMessage);
                    }
                }
            }while(ptrMessage);
        }

        /* Poll consumer every 1ms */
        Task_sleep(1);
    }

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
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Log_createLoggingProducer
(
    AppLTEStackDomainMCB* ptrLTEStackDomain,
    Netfp_SockHandle      datSockHandle,
    char*                 producerName
)
{
    Dat_ProducerCfg         producerCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Resmgr_ResourceCfg      loggingResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        0,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                      Linking RAM,                           Num,     Size */
            { "DAT-ProducerLogging",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  256,     128},
        }
    };

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (Domain_getSysCfgHandle(ptrLTEStackDomain->syslibHandle), &loggingResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Logging configuration failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration:
     *  - The heap is used for the exchange of messages between the L1 DAT Producer and the L2 Consumer */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "DatProducerHeap_%d", ptrLTEStackDomain->appId);
    heapCfg.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    heapCfg.memRegion                       = loggingResourceCfg.memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
    heapCfg.heapInterfaceTable.dataMalloc   = Log_dataHeapMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Log_dataHeapFree;
    ptrLTEStackDomain->datProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (ptrLTEStackDomain->datProducerHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT producer heap [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: DAT Producer Heap [Free Queue %x]\n", Pktlib_getInternalHeapQueue(ptrLTEStackDomain->datProducerHeap));

    /* Initialize the producer configuration. */
    memset ((void*)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, producerName);
    producerCfg.heapHandle           = ptrLTEStackDomain->datProducerHeap;
    producerCfg.debugSocketHandle    = datSockHandle;
    producerCfg.loggerStreamerHandle = logger0;
    producerCfg.producerType         = DAT_PRODUCER_UIA;

    /* Create the producer */
    ptrLTEStackDomain->producerHandle = Dat_createProducer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                          &producerCfg, NULL, &errCode);
    if (ptrLTEStackDomain->producerHandle == NULL)
    {
        System_printf ("Error: Unable to create the producer %s [Error code %d]\n", producerName, errCode);
        return -1;
    }
    System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                   producerCfg.name, ptrLTEStackDomain->producerHandle);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to initialize the logging infrastructure
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack Domain application MCB
 *  @param[in]  phyId
 *      Phy identifier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Log_initLogging (AppLTEStackDomainMCB* ptrLTEStackDomain, char phyId)
{
    char                    fastPathName[NETFP_MAX_CHAR];
    char                    producerName[DAT_MAX_CHAR];
    Dat_ConsumerCfg         consumerCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Netfp_InboundFPHandle   ingressFPHandle;
    Netfp_OutboundFPHandle  egressFPHandle;
    Netfp_SockAddr          sockAddress;
    Task_Params             taskParams;
    Resmgr_ResourceCfg      loggingResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        0,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                      Linking RAM,                           Num,     Size */
            { "DAT-Logging",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  256,     128},
        }
    };

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (Domain_getSysCfgHandle(ptrLTEStackDomain->syslibHandle), &loggingResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Logging configuration failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration:
     *  - The heap is used for the exchange of messages between the L1 DAT Producer and the L2 Consumer */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "DatConsumerHeap_%d", ptrLTEStackDomain->appId);
    heapCfg.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    heapCfg.memRegion                       = loggingResourceCfg.memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
    heapCfg.heapInterfaceTable.dataMalloc   = Log_dataHeapMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Log_dataHeapFree;
    ptrLTEStackDomain->datConsumerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (ptrLTEStackDomain->datConsumerHeap == NULL)
    {
        System_printf ("Error: Unable to create the DAT consumer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* SYNC Point: Wait for the debug fast paths to be created. */
    System_printf ("Debug: Waiting for the debug fast paths to be created\n");

    /* Construct the fast path name: */
    snprintf (fastPathName, NETFP_MAX_CHAR, "Debug-Tracing-Ingress_%c", phyId);
    while (1)
    {
        /* Is the ingress fast path created? */
        ingressFPHandle = Netfp_findInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                     fastPathName, &errCode);
        if (ingressFPHandle != NULL)
            break;

        /* Relinquish time slice */
        Task_sleep(1);
    }
    System_printf ("Debug: Ingress Debug Fast Path found\n");

    /* Construct the fast path name */
    snprintf (fastPathName, NETFP_MAX_CHAR, "Debug-Tracing-Egress_%c", phyId);
    while (1)
    {
        /* Is the ingress fast path created? */
        egressFPHandle = Netfp_findOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                     fastPathName, &errCode);
        if (egressFPHandle != NULL)
            break;

        /* Relinquish time slice */
        Task_sleep(1);
    }
    System_printf ("Debug: Egress Debug Fast Path found\n");

    /* Create the DAT Socket handle */
    ptrLTEStackDomain->datSockHandle = Netfp_socket (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                     Netfp_SockFamily_AF_INET, &errCode);
    if (ptrLTEStackDomain->datSockHandle == NULL)
    {
        System_printf ("Error: DAT Socket Creation Failed [Error Code %d]\n", errCode);
        return -1;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 1235 + DNUM;
    sockAddress.op.bind.inboundFPHandle = ingressFPHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = NULL;

    /* Bind the socket. */
    if (Netfp_bind (ptrLTEStackDomain->datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: DAT socket Bind Failed [Error Code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: DAT consumer socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1235;
    sockAddress.op.connect.outboundFPHandle = egressFPHandle;

    /* Connect the socket */
    if (Netfp_connect(ptrLTEStackDomain->datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: DAT socket connect Failed [Error Code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: DAT consumer socket has been connected successfully\n");

    /* Initialize the consumer configuration. */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    snprintf(consumerCfg.producerName, DAT_MAX_CHAR, "UIA_CONTRACT_PHY_SLAVE_%c", phyId);
    consumerCfg.heapHandle          = ptrLTEStackDomain->datConsumerHeap;

    /* Create the consumer */
    ptrLTEStackDomain->consumerPhySlaveHandle = Dat_createConsumer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                    &consumerCfg, &errCode);
    if (ptrLTEStackDomain->consumerPhySlaveHandle == NULL)
    {
        System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the consumer configuration. */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    snprintf(consumerCfg.producerName, DAT_MAX_CHAR, "UIA_CONTRACT_PHY_MASTER_%c", phyId);
    consumerCfg.heapHandle          = ptrLTEStackDomain->datConsumerHeap;

    /* Create the consumer */
    ptrLTEStackDomain->consumerPhyMasterHandle = Dat_createConsumer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                    &consumerCfg, &errCode);
    if (ptrLTEStackDomain->consumerPhyMasterHandle == NULL)
    {
        System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }

    /* Connect the PHY SLAVE Consumer/Producer: This can only be done once the producers have been created */
    while (1)
    {
        if (Dat_connectConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) == 0)
            break;

        if (errCode != DAT_ENOTREADY)
        {
            System_printf ("Error: Connect Consumer/Producer failed [Error code %d]\n", errCode);
            return -1;
        }
        Task_sleep(1);
    }

    /* Connect the PHY MASTER Consumer/Producer: This can only be done once the producers have been created */
    while (1)
    {
        if (Dat_connectConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) == 0)
            break;

        if (errCode != DAT_ENOTREADY)
        {
            System_printf ("Error: Connect Consumer/Producer failed [Error code %d]\n", errCode);
            return -1;
        }
        Task_sleep(1);
    }

    /* Launch the consumer task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 2;
    taskParams.arg0      = (UArg)ptrLTEStackDomain;
    ptrLTEStackDomain->l2ConsumerTaskHandle = Task_create((ti_sysbios_knl_Task_FuncPtr)Log_consumerTask, &taskParams, NULL);

    /* Construct the producer name: */
    snprintf (producerName, DAT_MAX_CHAR, "UIA_CONTRACT_L2_%c", phyId);
    Log_createLoggingProducer(ptrLTEStackDomain, ptrLTEStackDomain->datSockHandle, producerName);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the logging setup and is invoked
 *      when the application is going down.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack Domain application MCB
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Log_deinitLogging (AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    int32_t             errCode;
    Dat_ConsumerStatus  consumerStatus;

    System_printf ("Debug: Shutting down the logging consumers\n");

    /* Disconnect the consumers from the producers: */
    if (ptrLTEStackDomain->consumerPhyMasterHandle != NULL)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) < 0)
        {
            System_printf ("Error: Disconnecting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (ptrLTEStackDomain->consumerPhyMasterHandle, &consumerStatus, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return -1;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) < 0)
        {
            System_printf ("Error: Deleting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    System_printf ("Debug: PHY Master consumer has been deleted\n");

    /* Disconnect the consumers from the producers: */
    if (ptrLTEStackDomain->consumerPhySlaveHandle != NULL)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) < 0)
        {
            System_printf ("Error: Disconnecting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (ptrLTEStackDomain->consumerPhySlaveHandle, &consumerStatus, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return -1;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) < 0)
        {
            System_printf ("Error: Deleting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    System_printf ("Debug: PHY Slave consumer has been deleted\n");

    /* Flush all producer buffers */
    Dat_flushAllBuffers(Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle));

    /* Delete L2 producers, the producer is created without any consumer, hence delete directly */
    if (Dat_deleteProducer (ptrLTEStackDomain->producerHandle, &errCode) < 0)
    {
        System_printf ("Error: DAT Delete L2 Producer failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: L2 producer has been deleted\n");

    /* Shutdown the NETFP socket which was created */
    if (Netfp_closeSocket (ptrLTEStackDomain->datSockHandle, &errCode) < 0)
        System_printf ("Error: DAT socket deletion failed [Error code %d]\n", errCode);
    else
        System_printf ("Debug: DAT socket deleted successfully\n");

    /* Shutdown the PKTLIB heaps for consumer */
    if (Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->datConsumerHeap, &errCode) < 0)
        System_printf ("Error: PKTLIB DAT consumer heap deletion failed [Error code %d]\n", errCode);
    else
        System_printf ("Debug: PKTLIB DAT consumer heap deleted successfully\n");

    /* Shutdown the PKTLIB heaps for producer */
    if (Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->datProducerHeap, &errCode) < 0)
        System_printf ("Error: PKTLIB DAT producer heap deletion failed [Error code %d]\n", errCode);
    else
        System_printf ("Debug: PKTLIB DAT producer heap deleted successfully\n");

    return 0;
}

