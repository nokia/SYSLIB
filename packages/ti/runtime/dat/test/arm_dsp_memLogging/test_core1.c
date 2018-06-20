/**
 *   @file  test_core1.c
 *
 *   @brief
 *      Core1 Test Code for Debug & Trace
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
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Memory.h>
#include <xdc/cfg/global.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/* Logger streamer Include Files */
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

/* Test trace log Include Files */
#include "../trace_L2.h"
#include "../trace_log.h"
#include "../benchmark.h"

/**********************************************************************
 ************************** Unit Test Externs *************************
 **********************************************************************/

/* Global SYSLIB Module Instance Handles: */
extern Resmgr_ResourceCfg   appResourceConfig;

extern Pktlib_InstHandle    appPktlibInstanceHandle;
extern Msgcom_InstHandle    appMsgcomInstanceHandle;
extern Netfp_ClientHandle   netfpClientHandle;
extern Name_ClientHandle    nameClientHandle;
extern Name_DBHandle        globalNameDatabaseHandle;

/* System Configuration Handle. */
extern Resmgr_SysCfgHandle  handleSysCfg;

/* DAT Client Heap */
extern Pktlib_HeapHandle    datClientHeap;
extern Pktlib_HeapHandle    datProducerHeap;
extern Pktlib_HeapHandle    datGPProducerHeap;
extern Pktlib_HeapHandle    datPMProducerHeap;
extern Pktlib_HeapHandle    datConsumerHeap;

/* NETFP Client Handle */
extern Netfp_ClientHandle   netfpClientHandle;

/* Cache Invalidate API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

/* DAT: */
extern void* Dat_osalMalloc (uint32_t , uint32_t );
extern void  Dat_osalFree (void* , uint32_t );
extern void* Dat_osalMallocLocal (uint32_t , uint32_t );
extern void  Dat_osalFreeLocal (void* , uint32_t );
extern void* Dat_osalEnterSingleCoreCriticalSection (void);
extern void  Dat_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Dat_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Dat_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Dat_osalCreateSem(void);
extern void  Dat_osalDeleteSem(void*);
extern void  Dat_osalPostSem(void*);
extern void  Dat_osalPendSem(void*);

/* MEMLOG: */
extern void* Memlog_osalMalloc(Msgcom_MemAllocMode mode, uint32_t numBytes);
extern void Memlog_osalFree(Msgcom_MemAllocMode mode, void* ptr, uint32_t numBytes);
extern void* Memlog_osalEnterSingleCoreCS(void);
extern void  Memlog_osalExitSingleCoreCS(void* csHandle);

/**********************************************************************
 ************************** Unit Test Globals *************************
 **********************************************************************/

/* Producer name: This is a well known producer name across the test domain */
static const char*  gDatServerName = "DatServer_LTE9A";

/* DAT Client Handle: */
Dat_ClientHandle    datClientHandle;

/* DAT Consumer Array */
Dat_ConsHandle      datConsumerArray[4];
uint32_t            datConsumerCount = 0;

/* Statistics: Counter which keeps track of the number of messages received on the
 * consumer */
uint32_t            consumerMessage = 0;

/* Global varible to indicate test completion */
uint32_t gTestComplete = 0;

/* Socket Handle: */
Netfp_SockHandle    datSockHandle;
Netfp_SockHandle    debugSASockHandle;

/* DAT General Purpose producer handles */
#define             MAX_GP_PRODUCER    1
typedef struct GenPurposeProducer
{
    char	        name[DAT_MAX_CHAR];
    uint32_t        bufSize;
    uint8_t*        buffer;
    Dat_ProdHandle  prodHandle;
    void*           logger;
}GenPurposeProducer;

GenPurposeProducer GPProducer[MAX_GP_PRODUCER];
GenPurposeProducer PMProducer;

/* Trace object body handle */
Dat_TraceObjectBody*    ptrTraceObjBody = NULL;
/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to execute the DAT Client
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_datClient(UArg arg0, UArg arg1)
{
    /* Execute the DAT Client: This is executed in a polled mode since the
     * DAT clients execute in the control path. */
    while (1)
    {
        Dat_executeClient (datClientHandle);
        Dat_executeBackgroundActions (datClientHandle);
        Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to general General purpose producer log messages.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_GPProducerLoggingTask(UArg arg0, UArg arg1)
{

    int32_t     errCode;
    uint32_t    prodIdx;
    uint32_t    idx;

    /* Get the handle for the first buffer */
    for (prodIdx=0; prodIdx < MAX_GP_PRODUCER; prodIdx++)
    {
        if(GPProducer[prodIdx].prodHandle == NULL)
        continue;

        GPProducer[prodIdx].buffer = Dat_getProducerBuffer(GPProducer[prodIdx].prodHandle, GPProducer[prodIdx].name, &errCode);
        if (GPProducer[prodIdx].buffer == NULL)
        {
            System_printf ("Error: Unable to get the first general purpose producer(%p) buffer [Error code %d]\n",
            GPProducer[prodIdx].prodHandle, errCode);
            while(1);
        }

        /* Fill data pattern in the buffer */
        for(idx=0;idx < GPProducer[prodIdx].bufSize;idx++)
            GPProducer[prodIdx].buffer[idx] = idx + 1 + prodIdx;
    }

    /* This task will do buffer exchange every 1ms on both Producers
     */
    while(!gTestComplete)
    {
        Task_sleep(1);

        if (Dat_filter (ptrTraceObjBody, TRACE_COMPONENT_L2_MAC1, DAT_COMP_LEVEL_PE0, TRACE_NON_FOCUSED))
        {
            /* Exchange the PM buffers. */
            GPProducer[0].buffer = Dat_bufferExchange (datClientHandle, GPProducer[0].prodHandle, GPProducer[0].buffer);

            /* Re-fill data for producer , since the buffer is cleared during buffer exchange */
            for(idx=0; idx < GPProducer[0].bufSize; idx++)
                GPProducer[0].buffer[idx] = idx + 1;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      DAT test code for Core1
 *
 *  @retval
 *      Not Applicable.
 */
void Test_core1Task(UArg arg0, UArg arg1)
{
    Dat_ProdHandle              prodHandle;
    Dat_ProducerCfg             producerCfg;
    int32_t                     errCode;
    uint32_t                    rateLogMessage = 10;
    uint32_t                    testDuration;
    uint32_t                    logIndex = 0;
    int32_t                     msgIndex;
    Dat_ClientCfg               datClientCfg;
    int32_t                     clientStatus;
    Task_Params                 taskParams;
    Task_Handle                 datTaskHandle;
    Dat_ProducerStats           producerStats;
    Name_ResourceCfg            namedResourceCfg;
    uint32_t                    memLoggingSize = 1024*1024;
    uint32_t                    memLoggingBase[4] = {0};
    Memlog_InstCfg              memlogInstConfig;
    Memlog_InstHandle           memlogInstHandle;
    Memlog_ChannelCfg           memlogChanConfig;
    Memlog_ChHandle             memlogChanHandle;

    Resmgr_ResourceCfg      memLoggingResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        0,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                      Linking RAM,                                 Num,     Size */
            { "MemLogging1",   Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,       1024,    128},
        }
    };


    /* SYNC Point: Ensure that the DAT Server has been started. This is required before
     * we create and register the DAT client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready; since the DAT server and
         * client are executing in different realm we need to specify the NAME client handle. */
        if (Dat_startServer (globalNameDatabaseHandle, nameClientHandle, (char*)gDatServerName, &errCode) != NULL)
            break;

        /* Check the error code. */
        if (errCode != DAT_ENOTREADY)
        {
            System_printf ("Error: DAT Starting server failed [Error code %d]\n", errCode);
            return;
        }
        Task_sleep(1);
    }

    /* Initialize the DAT client configuration */
    memset ((void *)&datClientCfg, 0, sizeof(Dat_ClientCfg));

    /* Populate the DAT client configuration */
    strcpy (datClientCfg.clientName, "Dat_Client_LTE9A_L1_SLAVE");
    strcpy (datClientCfg.serverName, gDatServerName);
    datClientCfg.pktlibInstHandle  = appPktlibInstanceHandle;
    datClientCfg.msgcomInstHandle  = appMsgcomInstanceHandle;
    datClientCfg.directInterruptCfg.queuePendQueue  = appResourceConfig.qPendResponse[2].queue;
    datClientCfg.directInterruptCfg.cpIntcId        = appResourceConfig.qPendResponse[2].cpIntcId;
    datClientCfg.directInterruptCfg.systemInterrupt = appResourceConfig.qPendResponse[2].systemInterrupt;
    datClientCfg.directInterruptCfg.hostInterrupt   = appResourceConfig.qPendResponse[2].hostInterrupt;
    datClientCfg.databaseHandle    = globalNameDatabaseHandle;
    datClientCfg.clientHeapHandle  = datClientHeap;
    datClientCfg.nameClientHandle  = nameClientHandle;
    datClientCfg.realm             = Dat_ExecutionRealm_DSP;
    datClientCfg.id                = DNUM;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.mallocLocal       = Dat_osalMallocLocal;
    datClientCfg.freeLocal         = Dat_osalFreeLocal;
    datClientCfg.beginMemAccess    = Dat_osalBeginMemoryAccess;
    datClientCfg.endMemAccess      = Dat_osalEndMemoryAccess;
    datClientCfg.createSem         = Dat_osalCreateSem;
    datClientCfg.deleteSem         = Dat_osalDeleteSem;
    datClientCfg.postSem           = Dat_osalPostSem;
    datClientCfg.pendSem           = Dat_osalPendSem;
    datClientCfg.enterCS           = Dat_osalEnterSingleCoreCriticalSection;
    datClientCfg.exitCS            = Dat_osalExitSingleCoreCriticalSection;
    datClientCfg.logSync.syncPeriod = 50;
    datClientCfg.logSync.syncLogger = logger0;

    /* Initialize the DAT client. */
    datClientHandle = Dat_initClient (&datClientCfg, &errCode);
    if (datClientHandle == NULL)
    {
        System_printf ("Error: Unable to create the DAT client [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT client %p created successfully\n", datClientHandle);

    /* Start the DAT client: DAT clients can only be started after they have been registered
     * by the server */
    while (1)
    {
        /* Start the DAT client */
        clientStatus = Dat_startClient (datClientHandle, &errCode);
        if (clientStatus < 0)
        {
            System_printf ("Error: DAT Client registration status failed [Error code %d]\n", errCode);
            return;
        }

        /* If the client has been registered; we can proceed */
        if (clientStatus == 1)
            break;

        /* Client has not been registered; wait for some time and try again */
        Task_sleep(1);
    }

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (handleSysCfg, &memLoggingResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Memory Logging resource configuration failed [Error code %d]\n", errCode);
        return ;
    }

    /* Launch the DAT Client Execution Task:
     * - This is setup to be a lower priority task which executes in the background when there is no
     *   other task to execute */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 1;
    datTaskHandle = Task_create(Test_datClient, &taskParams, NULL);

    System_printf ("******************************************\n");
    System_printf ("Rate of log messages per milliseconds: ");
    scanf ("%d", &rateLogMessage);
    System_printf ("Duration of the test in msec[0 for infinite]: ");
    scanf ("%d", &testDuration);
    System_printf ("******************************************\n");

    /* Validate the input */
    if (rateLogMessage == 0)
    {
        System_printf ("Error: Please enter a valid rate \n");
        return;
    }

    /*******************************************************************************
     * TEST: Creating MEMLOG instance
     *******************************************************************************/

    /* Initialize and create the MEMLOG instance */
    memset ((void *)&memlogInstConfig, 0, sizeof(Memlog_InstCfg));

    memlogInstConfig.databaseHandle    = globalNameDatabaseHandle;
    memlogInstConfig.pktlibInstHandle  = appPktlibInstanceHandle;
    memlogInstConfig.msgcomInstHandle  = appMsgcomInstanceHandle;
    memlogInstConfig.realm             = Memlog_ExecutionRealm_DSP;
    memlogInstConfig.malloc            = Memlog_osalMalloc;
    memlogInstConfig.free              = Memlog_osalFree;
    memlogInstConfig.enterCS           = Memlog_osalEnterSingleCoreCS;
    memlogInstConfig.exitCS            = Memlog_osalExitSingleCoreCS;

    if ((memlogInstHandle =  Memlog_createInstance(&memlogInstConfig, &errCode)) == NULL)
    {
        System_printf ("Error: MEMLOG create instance Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: MEMLOG instance has been created successfully\n");

    /*******************************************************************************
     * TEST: Creating MEMLOG channel for core1_UIAProducer
     *******************************************************************************/

    /* Initialize and create MEMLOG channel */
    memset((void *)&memlogChanConfig, 0, sizeof(Memlog_ChannelCfg) );

    memLoggingBase[0] = (uint32_t)Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3MemLoggingHeap,
                                                 memLoggingSize,
                                                 MEMLOG_MEMORY_ALIGNMENT,
                                                 NULL);

    strncpy ( &memlogChanConfig.name[0], "MemlogCore1_UIAProducer", MEMLOG_MAX_CHAR-1);
    memlogChanConfig.memlogInstHandle = memlogInstHandle;
    memlogChanConfig.memRegion        = memLoggingResourceCfg.memRegionResponse[0].memRegionHandle;
    memlogChanConfig.memBaseAddr      = memLoggingBase[0];
    memlogChanConfig.memSize          = memLoggingSize;
    memlogChanConfig.bufferSize       = 1408;
    memlogChanConfig.numPktDescs      = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(datProducerHeap));

    if ((memlogChanHandle = Memlog_create(&memlogChanConfig, &errCode)) == NULL)
    {
        System_printf ("Error: MEMLOG create channel Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: MEMLOG channel(%p) has been created successfully\n", memlogChanHandle );

    /* Push the channel name from the DSP to the ARM realm. */
    if (Name_push (nameClientHandle, "MemlogCore1_UIAProducer", Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", "MemlogCore0_GPProducer", errCode);
        return ;
    }
    /***********************************************************************************
     * Create the producer with memory logging enabled
     ***********************************************************************************/
    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Core1_UIAProducer");
    producerCfg.heapHandle           = datProducerHeap;
    producerCfg.debugSocketHandle    = NULL;
    producerCfg.loggerStreamerHandle = logger0;
    producerCfg.producerType         = DAT_PRODUCER_UIA;
    producerCfg.memlogChanHandle     = memlogChanHandle;

    /* Create the producer */
    prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
    if (prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Producer '%s' Handle %x [Logging to memory] created successfully\n",
                    producerCfg.name, prodHandle);

    /*******************************************************************************
     * TEST: Creating MEMLOG channel for core1_GPProducer
     *******************************************************************************/
    /* Initialize and create MEMLOG channel */
    memset((void *)&memlogChanConfig, 0, sizeof(Memlog_ChannelCfg) );

    memLoggingBase[1] = (uint32_t)Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3MemLoggingHeap,
                                                 memLoggingSize,
                                                 MEMLOG_MEMORY_ALIGNMENT,
                                                 NULL);

    strncpy ( &memlogChanConfig.name[0], "MemlogCore1_GPProducer", MEMLOG_MAX_CHAR-1);
    memlogChanConfig.memlogInstHandle = memlogInstHandle;
    memlogChanConfig.memRegion        = memLoggingResourceCfg.memRegionResponse[0].memRegionHandle;
    memlogChanConfig.memBaseAddr      = memLoggingBase[1];
    memlogChanConfig.memSize          = memLoggingSize;
    memlogChanConfig.bufferSize       = 1367 + Dat_getGPProducerDataOffsetByRealm(Dat_ExecutionRealm_DSP, &errCode);   /* 56bytes is the extra UIA header*/
    memlogChanConfig.numPktDescs      = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(datProducerHeap));

    if ((memlogChanHandle = Memlog_create(&memlogChanConfig, &errCode)) == NULL)
    {
        System_printf ("Error: MEMLOG create channel Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: MEMLOG channel(%p) has been created successfully\n", memlogChanHandle );

    /* Push the channel name from the DSP to the ARM realm. */
    if (Name_push (nameClientHandle, "MemlogCore1_GPProducer", Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", "MemlogCore0_GPProducer", errCode);
        return ;
    }

    /*******************************************************************************
     * TEST: Creating general purpose producer with memory logging
     *******************************************************************************/
    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Core1_GPProducer");
    producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
    producerCfg.bufferSize           = 1367;
    producerCfg.clearBuffer          = 1;
    producerCfg.heapHandle           = datGPProducerHeap;
    producerCfg.debugSocketHandle    = NULL;
    producerCfg.loggerStreamerHandle = NULL;
    producerCfg.memlogChanHandle     = memlogChanHandle;

    /* Create the producer */
    GPProducer[0].prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &GPProducer[0].logger, &errCode);
    if (GPProducer[0].prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the general purpose producer [Error code %d]\n", errCode);
        return;
    }
    GPProducer[0].bufSize = producerCfg.bufferSize;
    strcpy(GPProducer[0].name, producerCfg.name);

    System_printf ("Debug: Producer '%s' Handle %x [Logging to memory] created successfully\n",
                    producerCfg.name, GPProducer[0].prodHandle);

    /* Create a task to generate General Purpose producer logs */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 3;
    Task_create(Test_GPProducerLoggingTask, &taskParams, NULL);

    /* Run the log generation loop accounting for the polling delay and the number of messages */
    if (testDuration == 0)
    {
        /* Execute the test forever. */
        while (1)
        {
            for (msgIndex = 0; msgIndex < rateLogMessage; msgIndex++)
            {
                Log_iwriteUC3 (logger0, UIAEvt_detailWithStr,
                              (IArg)0x10, (IArg)"Test Log Generation Index: %x", logIndex++);
            }
            Task_sleep(1);
            testDuration = testDuration + 1;
        }
    }
    else
    {
        /* Execute the test for the specified time */
        while (testDuration != 0)
        {
            for (msgIndex = 0; msgIndex < rateLogMessage; msgIndex++)
                Log_write3(UIAEvt_detailWithStr, 0x10, (IArg)"Test Log Generation Index: %x", logIndex++);
            Task_sleep(1);
            testDuration = testDuration - 1;
        }
    }

    /* Get the producer statistics. */
    if (Dat_getProducerStats (prodHandle, &producerStats, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the producer statistics [Error code %d]\n", errCode);
        return;
    }

    /* Display the statistics on the console. */
    System_printf ("Debug: UIA Producer Statistics for Producer %p\n", prodHandle);
    System_printf ("Buffer Exchanged   : %d\n", producerStats.bufferExchange);
    System_printf ("Buffer Overrun     : %d\n", producerStats.bufferOverrun);
    System_printf ("No Consumers       : %d\n", producerStats.noConsumers);
    System_printf ("Streaming Error    : %d\n", producerStats.debugStreamingError);
    System_printf ("Streamed Messages  : %d\n", producerStats.debugStreaming);
    System_printf ("Allocated Failures : %d\n", producerStats.allocFailures);
    System_printf ("Max Queue Depth    : %d\n", producerStats.maxQueueDepth);

    /* Get the general producer statistics. */
    if (Dat_getProducerStats (GPProducer[0].prodHandle, &producerStats, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the producer statistics [Error code %d]\n", errCode);
        return;
    }

    /* Display the statistics on the console. */
    System_printf ("Debug: General Purpose Producer Statistics for Producer %p\n", prodHandle);
    System_printf ("Buffer Exchanged   : %d\n", producerStats.bufferExchange);
    System_printf ("Buffer Overrun     : %d\n", producerStats.bufferOverrun);
    System_printf ("No Consumers       : %d\n", producerStats.noConsumers);
    System_printf ("Streaming Error    : %d\n", producerStats.debugStreamingError);
    System_printf ("Streamed Messages  : %d\n", producerStats.debugStreaming);
    System_printf ("Allocated Failures : %d\n", producerStats.allocFailures);
    System_printf ("Max Queue Depth    : %d\n", producerStats.maxQueueDepth);

    /* Initialize the named resource configuration */
    memset ((void*)&namedResourceCfg, 0, sizeof(Name_ResourceCfg));

    /* Populate the named resource configuration. */
    namedResourceCfg.handle1  = (uint32_t)1;
    strcpy(namedResourceCfg.name, "DAT_UNIT_TEST_STATUS");

    /* Indicate that the test is going down.  */
    if (Name_createResource(globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                            &namedResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Creating test named resource failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT Log generation complete on Core %d\n", DNUM);

    /* Mark Test is completed, so that logs are not generated anymore. */
    gTestComplete = 1;

    /* Delete the producer: Producers can only be deleted once all the consumers have been
     * disconnected and deleted. */
    while (1)
    {
        if (Dat_deleteProducer (prodHandle, &errCode) == 0)
            break;

        if (errCode != DAT_EINUSE)
        {
            System_printf ("Error: DAT Delete Producer failed [Error code %d]\n", errCode);
            return;
        }
    }

    /* Delete the general purpose producer: Producers can only be deleted once all the consumers have been
     * disconnected and deleted. */
    while (1)
    {
        if (Dat_deleteProducer (GPProducer[0].prodHandle, &errCode) == 0)
            break;

        if (errCode != DAT_EINUSE)
        {
            System_printf ("Error: DAT Delete General Purpose Producer failed [Error code %d]\n", errCode);
            return;
        }
    }
    /* Stop the DAT client */
    if (Dat_stopClient (datClientHandle, &errCode) < 0)
    {
        System_printf ("Error: DAT Stop client failed [Error code %d]\n", errCode);
        return;
    }

    /* Stop the DAT Task */
    Task_delete(&datTaskHandle);

    /* Delete the DAT client */
    if (Dat_deleteClient (datClientHandle, &errCode) < 0)
    {
        System_printf ("Error: DAT Delete client failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT Test passed\n");
    return;
}

