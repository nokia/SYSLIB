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

/**********************************************************************
 ************************** Unit Test Globals *************************
 **********************************************************************/

/* Producer name: This is a well known producer name across the test domain */
static const char*  gDatServerName = "DatServer_LTE9A";
static const char*  gProducerName  = "Test-UIA";

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
    char            name[DAT_MAX_CHAR];
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

#if 0
/**
 *  @b Description
 *  @n
 *      Registered consumer call back function which is invoked on
 *      the reception of log messages from the producer.
 *
 *  @param[in]  consumerHandle
 *      DAT Consumer Handle
 *  @param[in]  ptrMessage
 *      Pointer to the received message
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_consumerFunction(Dat_ConsHandle consumerHandle, Ti_Pkt* ptrMessage)
{
    uint8_t*    ptrBuffer;
    uint32_t    bufferSize;
    int32_t     errCode;

    /* Increment the counter */
    consumerMessage++;

    /* Get the data buffer and size */
    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);

    /* Invalidate the data buffer contents */
    appInvalidateBuffer (ptrBuffer, bufferSize);

    /* Send the message out via the DAT Socket Handle */
    if (Netfp_send (datSockHandle, ptrMessage, 0, &errCode) < 0)
    {
        System_printf ("Debug: Unable to send the message via NETFP [Error code %d]\n", errCode);
        Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
    }
    return;
}
#endif

/**
 *  @b Description
 *  @n
 *      This task to consumer buffers for all created consumers.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_consumerTask(UArg arg0, UArg arg1)
{
    Ti_Pkt*         ptrMessage;
    uint8_t*        ptrBuffer;
    uint32_t        bufferSize;
    int32_t         errCode;
    uint32_t        idx;
    Dat_ConsHandle  consHandle;

    /* This task is created after consumer is created. */
    while(1)
    {
        /* Poll all valid consumers */
        for(idx=0; idx < 4; idx++)
        {
            consHandle = datConsumerArray[idx];

            if (consHandle == NULL)
                continue;

            /* Poll all the messages for the consumer (non-pacing consumer) */
            do{

                /* Poll message for consumer */
                ptrMessage = Dat_processConsumer(consHandle);
                if(ptrMessage)
                {
                    /* Increment the number of messages received by the consumer. */
                    consumerMessage++;

                    /* Get the data buffer and size */
                    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);

                    /* Invalidate the data buffer contents */
                    appInvalidateBuffer (ptrBuffer, bufferSize);

                    /* Send the message out via the DAT Socket Handle */
                    if (Netfp_send (datSockHandle, ptrMessage, 0, &errCode) < 0)
                    {
                        System_printf ("Debug: Unable to send the message via NETFP [Error code %d]\n", errCode);
                        Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
                    }
                }
            }while(ptrMessage);

        }

        /* Poll consumer every 2ms */
        Task_sleep(2);
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

    int32_t		errCode;
    uint32_t	prodIdx;
    uint32_t 	idx;

    /* Get the handle for the first buffer */
    for (prodIdx=0; prodIdx < MAX_GP_PRODUCER; prodIdx++)
    {
        GPProducer[prodIdx].buffer = Dat_getProducerBuffer(GPProducer[prodIdx].prodHandle,
                                                           GPProducer[prodIdx].name,
                                                           &errCode);
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
 *      The function is used to test PM measurements Producer .
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_PMProducerLoggingTask(UArg arg0, UArg arg1)
{
    int32_t     errCode;
    uint32_t    idx;
    uint32_t    numElment=50;
    uint32_t*   buffer;

    PMProducer.buffer = Dat_getProducerBuffer(PMProducer.prodHandle, PMProducer.name, &errCode);
    if (PMProducer.buffer == NULL)
    {
        System_printf ("Error: Unable to get the first general purpose producer(%p) buffer [Error code %d]\n",
        PMProducer.prodHandle, errCode);
        while(1);
    }

    buffer = (uint32_t *)PMProducer.buffer;
    /* Initialize data in the buffer */
    for(idx= 0; idx < numElment; idx++)
        buffer[idx] = 0x55;

    /* This task will do buffer exchange every 1ms on both Producers
    */
    while(!gTestComplete)
    {
        Task_sleep(1);

        if (1) //(Dat_filter (ptrTraceObjBody, TRACE_COMPONENT_L2_MAC1, DAT_COMP_LEVEL_PE0, TRACE_NON_FOCUSED))
        {
            /* Exchange the PM buffers. */
            buffer = (uint32_t *)Dat_bufferExchange (datClientHandle, PMProducer.prodHandle, (uint8_t *)buffer);

            /* Increment data in producer buffer */
            for(idx=0; idx < numElment; idx++)
                buffer[idx] = buffer[idx] + 1;
        }
    }
}
/**
 *  @b Description
 *  @n
 *      The function is used to test the creation/deletion of the producer
 *      and consumer
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_producerConsumer (void)
{
    Dat_ProdHandle              prodHandle;
    Dat_ConsHandle              consHandle;
    Dat_ProducerCfg             producerCfg;
    Dat_ConsumerCfg             consumerCfg;
    int32_t                     errCode;

    /*******************************************************************************
     * TEST: Basic create/delete producer
     *******************************************************************************/

    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Test_Producer");
    producerCfg.heapHandle          = datClientHeap;
    producerCfg.debugSocketHandle   = NULL;
    producerCfg.producerType        = DAT_PRODUCER_UIA;
    producerCfg.loggerStreamerHandle = NULL;
    producerCfg.bufferSize          = 1408;

    /* Create the producer */
    prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
    if (prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
        return -1;
    }

    /* Delete the producer */
    if (Dat_deleteProducer (prodHandle, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the producer [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 1 -> Basic create/delete producer test passed\n");

    /*******************************************************************************
     * TEST: Basic create/delete consumer
     *******************************************************************************/

    /* Initialize the consumer configuration */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    strcpy(consumerCfg.producerName, "Test_Producer");
    consumerCfg.heapHandle          = datClientHeap;
    producerCfg.loggerStreamerHandle = logger0;

    /* Create the consumer */
    consHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
    if (consHandle == NULL)
    {
        System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }

    /* Delete the consumer */
    if (Dat_deleteConsumer (consHandle, &errCode) < 0)
    {
        System_printf ("Error: Unable to delete the consumer [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 2 -> Basic create/delete consumer test passed\n");

    /*******************************************************************************
     * TEST: Connecting consumer & producer
     *******************************************************************************/

    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Test_Producer");
    producerCfg.heapHandle          = datClientHeap;
    producerCfg.debugSocketHandle   = NULL;
    producerCfg.producerType        = DAT_PRODUCER_UIA;
    producerCfg.loggerStreamerHandle = logger0;

    /* Create the producer */
    prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
    if (prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the consumer configuration */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    strcpy(consumerCfg.producerName, "Test_Producer");
    consumerCfg.heapHandle          = datClientHeap;

    /* Create the consumer */
    consHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
    if (consHandle == NULL)
    {
        System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }

    /* Connect the producer and consumer with each other */
    if (Dat_connectConsumer (consHandle, &errCode) < 0)
    {
        System_printf ("Error: Unable to connect the consumer [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 3 -> Basic connect producer/consumer test passed\n");

    /*******************************************************************************
     * TEST: Deleting connected consumer
     *******************************************************************************/

    /* Try and delete the consumer. This should NOT work because only *disconnected* consumers can be deleted
     * Ensure we get the proper error code */
    if (Dat_deleteConsumer (consHandle, &errCode) == 0)
    {
        System_printf ("Error: Connected consumer was deleted successfully\n");
        return -1;
    }

    /* Ensure that the proper error code was set */
    if (errCode != DAT_EINUSE)
    {
        System_printf ("Error: Connected consumer deletion failed with invalid error code %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 4 -> Deleting connected consumer test passed\n");

    /*******************************************************************************
     * TEST: Deleting producer with connected consumers
     *******************************************************************************/

    /* Try and delete the producer. This should NOT work because producers can only be deleted once all
     * the consumers have been disconnected from them. */
    if (Dat_deleteProducer (prodHandle, &errCode) == 0)
    {
        System_printf ("Error: Producer with connected consumer was deleted successfully\n");
        return -1;
    }

    /* Ensure that the proper error code was set */
    if (errCode != DAT_EINUSE)
    {
        System_printf ("Error: Producer with connected consuer deletion failed with invalid error code %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 5 -> Deleting producer with connected consumer test passed\n");

    /*******************************************************************************
     * TEST: Disconnect consumer
     *******************************************************************************/

    /* Disconnect the consumer. */
    if (Dat_disconnectConsumer (consHandle, &errCode) < 0)
    {
        System_printf ("Error: Disconnect consumer failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 6 -> Disconnect consumer test passed\n");

    /*******************************************************************************
     * TEST: Delete producer
     *******************************************************************************/

    /* Delete the producer */
    if (Dat_deleteProducer (prodHandle, &errCode) < 0)
    {
        System_printf ("Error: Producer deletion test failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 7 -> Producer deletion test passed\n");

    /*******************************************************************************
     * TEST: Delete consumer
     *******************************************************************************/

    /* Delete the consumer*/
    if (Dat_deleteConsumer (consHandle, &errCode) < 0)
    {
        System_printf ("Error: Consumer deletion test failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: Test 8 -> Consumer deletion test passed\n");
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to get trace verbosity.
 *
 *  @param[in]  traceObjectHandle
 *      Handle to trace object
 *  @param[in]  traceObjName
 *      Name of the trace Object
 *  @retval
 *      None
 */
static void Test_showTraceVerbosity
(
    Dat_TraceObjHandle  traceObjectHandle,
    char*               traceObjName
)
{
    Dat_TraceComponentCfg    componentList[TRACE_NUM_COMPONENT_L2];
    Dat_TraceObjectCfg       traceObjectCfg;
    int32_t                  errCode;
    uint32_t                 index;

    /* Initialize the structure */
    memset (&componentList[0], 0, sizeof(componentList));
    memset (&traceObjectCfg, 0, sizeof (Dat_TraceObjectCfg));

    /* Get Trace Object info */
    if (Dat_getNumComponents(traceObjectHandle, traceObjName, &traceObjectCfg.numTraceComponents, &errCode) < 0)
    {
        System_printf("Error: Unable to get number of trace components for %s, errCode = %d\n", traceObjName, errCode);
    }

    if (Dat_getClassVerbosity(traceObjectHandle, traceObjName, &traceObjectCfg.classLevel, &traceObjectCfg.commonCompLevel, &errCode) < 0)
    {
        System_printf("Error: Unable to get class/common components levels for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
    }

    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        if (Dat_getComponentVerbosity(traceObjectHandle, traceObjName, index,  &componentList[index], &errCode) < 0)
        {
            System_printf("Error: Unable to get class/common components levels for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
        }
    }

    /* Print out the verbosity settings */
    System_printf("Debug: There are %d trace components in trace object %s\n", traceObjectCfg.numTraceComponents, "L2-TRACE-OBJECT");
    System_printf("Debug: Class level:%x\n", traceObjectCfg.classLevel);
    System_printf("Debug: Common component level:%x\n", traceObjectCfg.commonCompLevel);
    System_printf("Debug: Component verbosity level:\n");
    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        System_printf("Debug: Index:%d\t foucus verbosity levle:%x\t non-foucus verbosity levle:%x\t\n",
                       index, componentList[index].focusedLevel, componentList[index].nonFocusedLevel);
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
    Netfp_InboundFPHandle       fpIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle;
    Netfp_SockAddr              sockAddress;
    int32_t                     errCode;
    uint32_t                    rateLogMessage;
    uint32_t                    testDuration;
    uint32_t                    logIndex = 0;
    int32_t                     msgIndex;
    Dat_ClientCfg               datClientCfg;
    Dat_ConsumerCfg             consumerCfg;
    int32_t                     clientStatus;
    Task_Params                 taskParams;
    Dat_ConsHandle              consumerHandle;
    Task_Handle                 datTaskHandle;
    Dat_ProducerStats           producerStats;
    Dat_ConsumerStatus          consumerStatus;
    Name_ResourceCfg            namedResourceCfg;
    int32_t                     testSelection;
    Dat_TraceObjHandle          traceObjectHandle = NULL;

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

    /* Get the ingress fast path handle */
    while (1)
    {
        fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
        if (fpIngressHandle != NULL)
            break;
        Task_sleep(1);
    }

    /* Get the egress fast path handle */
    while (1)
    {
        fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
        if (fpEgressHandle != NULL)
            break;
        Task_sleep(1);
    }

    /***********************************************************************************
    * Socket which streams data via the CONSUMER to a UDP Port 3000
    ***********************************************************************************/

    datSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (datSockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 3000;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = NULL;

    /* Bind the socket. */
    if (Netfp_bind (datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 3000;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been connected successfully\n");

    /***********************************************************************************
    * Socket which streams data via the CONSUMER to System Analyzer UDP Port 1235
    ***********************************************************************************/

    /* Create a socket */
    debugSASockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (debugSASockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 5000;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = NULL;

    /* Bind the socket. */
    if (Netfp_bind (debugSASockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1235;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(debugSASockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been connected successfully\n");

    /*******************************************************************************
     * TEST: Creating Trace Object Instance
     *******************************************************************************/
    /* Create a local instance of the L2 trace object. This is mandatory. */
    while (1)
    {

        /* Find the named resource. */
        traceObjectHandle = Dat_createTraceObjectInstance(datClientHandle, "L2-TRACE-OBJECT", &errCode);
        if (traceObjectHandle == NULL)
        {
            /* Check the error code. */
            if ( (errCode == DAT_ENOTREADY) || (errCode == DAT_ENOTFOUND) )
            {
                /* Resource has not yet been created; we will retry. */
                Task_sleep (10);
                continue;
            }
            System_printf ("Error: Creating local instance of L2 trace object failed with error %x\n", errCode);
            return ;
        }

        /* Created the local instance of the L2 trace object successfully. */
        System_printf ("Debug: Local instance of L2 trace object created.\n");
        break;
    }

    /* Get the trace object body to be used for filter */
    ptrTraceObjBody = Dat_getTraceObjectBody (traceObjectHandle);

    while (1)
    {
        System_printf ("************************************\n");
        System_printf ("** DAT ARM-DSP Unit Test CLI Menu **\n");
        System_printf ("************************************\n");
        System_printf ("1. API Testing\n");
        System_printf ("2. No consumers with debug streaming\n");
        System_printf ("3. Multiple consumers with debug streaming\n");
        System_printf ("4. Show trace object settings\n");
        System_printf ("Enter the selection: ");
        scanf("%d", &testSelection);
        if ((testSelection >= 1) && (testSelection <= 4))
            break;
    }

    switch (testSelection)
    {
        case 1:
        {
            /* Test the producer & consumer creation/deletion API */
            if (Test_producerConsumer() < 0)
                System_printf ("Error: Producer/Consumer creation/deletion tests failed\n");
            else
                System_printf ("Debug: Producer/Consumer creation/deletion tests passed\n");

            /* API testing is complete. There is no data generation here. */
            return;
        }
        case 2:
        {
        /***********************************************************************************
        * Create the producer with debug streaming enabled
        ***********************************************************************************/

        /* Initialize and create the DAT producer */
        memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

        /* Populate the producer configuration. */
        strcpy(producerCfg.name, "SingleProducer");
        producerCfg.heapHandle           = datProducerHeap;
        producerCfg.debugSocketHandle    = debugSASockHandle;
        producerCfg.loggerStreamerHandle = logger0;
        producerCfg.producerType         = DAT_PRODUCER_UIA;

        /* Create the producer */
        prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
        if (prodHandle == NULL)
        {
            System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                        producerCfg.name, prodHandle);

        /* There are no consumers in this test */
        consumerHandle = NULL;

        /*******************************************************************************
         * TEST: Creating general purpose producer
         *******************************************************************************/

        /* Initialize and create the DAT producer */
        memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

        /* Populate the producer configuration. */
        strcpy(producerCfg.name, "Test_Core1_GPProducer");
        producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
        producerCfg.bufferSize           = 1367;
        producerCfg.clearBuffer          = 1;
        producerCfg.heapHandle           = datGPProducerHeap;
        producerCfg.debugSocketHandle    = debugSASockHandle;
        producerCfg.loggerStreamerHandle = NULL;

        /* Create the producer */
        GPProducer[0].prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &GPProducer[0].logger,&errCode);
        if (GPProducer[0].prodHandle == NULL)
        {
            System_printf ("Error: Unable to create the general purpose producer [Error code %d]\n", errCode);
            return;
        }
        GPProducer[0].bufSize = producerCfg.bufferSize;
        strcpy(GPProducer[0].name, producerCfg.name);

        System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                        producerCfg.name, GPProducer[0].prodHandle);

        /* Create a task to generate General Purpose producer logs */
        {
            Task_Params taskParams;

            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 3;
            Task_create(Test_GPProducerLoggingTask, &taskParams, NULL);
        }

            break;
        }
        case 3:
        {
        /***********************************************************************************
         * Create the producer with debug streaming enabled
         ***********************************************************************************/
        /* Initialize and create the DAT producer */
        memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

        /* Populate the producer configuration. */
        strcpy(producerCfg.name, gProducerName);
        producerCfg.heapHandle           = datProducerHeap;
        producerCfg.debugSocketHandle    = debugSASockHandle;
        producerCfg.loggerStreamerHandle = logger0;
        producerCfg.producerType         = DAT_PRODUCER_UIA;

        /* Create the producer */
        prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
        if (prodHandle == NULL)
        {
            System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                    producerCfg.name, prodHandle);

        /* Initialize the consumer configuration. */
        memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

        /* Populate the consumer configuration. */
        strcpy(consumerCfg.producerName, gProducerName);
        consumerCfg.heapHandle          = datConsumerHeap;

        /* Create the consumer */
        consumerHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
        if (consumerHandle == NULL)
        {
            System_printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
            return;
        }
        System_printf ("Debug: Consumer Handle %x [Streaming Data to Port %d] created successfully\n",
                   consumerHandle, sockAddress.sin_port);

        /* Connect the consumer & producer */
        if (Dat_connectConsumer (consumerHandle, &errCode) < 0)
        {
            System_printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
            return;
        }

        /* Save consumer handle to be used in process Consumer */
        datConsumerArray[datConsumerCount++] =  consumerHandle;

        /* Create consumer task */
        Task_Params_init(&taskParams);
        taskParams.stackSize = 16*1024;
        taskParams.priority  = 1;
        Task_create(Test_consumerTask, &taskParams, NULL);

        /*******************************************************************************
         * TEST: Creating general purpose producer
         *******************************************************************************/

        /* Initialize and create the DAT producer */
        memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

        /* Populate the producer configuration. */
        strcpy(producerCfg.name, "Test_Core1_GPProducer");
        producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
        producerCfg.bufferSize	         = 1367;
        producerCfg.clearBuffer          = 1;
        producerCfg.heapHandle           = datGPProducerHeap;
        producerCfg.debugSocketHandle    = debugSASockHandle;
        producerCfg.loggerStreamerHandle = NULL;

        /* Create the producer */
        GPProducer[0].prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &GPProducer[0].logger, &errCode);
        if (GPProducer[0].prodHandle == NULL)
        {
            System_printf ("Error: Unable to create the general purpose producer [Error code %d]\n", errCode);
            return;
        }
        GPProducer[0].bufSize = producerCfg.bufferSize;
        strcpy(GPProducer[0].name, producerCfg.name);

        System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                    producerCfg.name, GPProducer[0].prodHandle);

        /* Create a task to generate General Purpose producer logs */
        Task_Params_init(&taskParams);
        taskParams.stackSize = 8*1024;
        taskParams.priority  = 3;
        Task_create(Test_GPProducerLoggingTask, &taskParams, NULL);

        /*******************************************************************************
         * TEST: Creating general purpose producer for Measurements
         *******************************************************************************/

        /* Initialize and create the DAT producer */
        memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

        /* Populate the producer configuration. */
        strcpy(producerCfg.name, "PM_Producer");
        producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
        producerCfg.bufferSize	         = 1024;
        producerCfg.clearBuffer          = 1;
        producerCfg.heapHandle           = datPMProducerHeap;
        producerCfg.debugSocketHandle    = NULL;
        producerCfg.loggerStreamerHandle = NULL;

        /* Create the producer */
        PMProducer.prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &PMProducer.logger, &errCode);
        if (PMProducer.prodHandle == NULL)
        {
            System_printf ("Error: Unable to create the general purpose producer [Error code %d]\n", errCode);
            return;
        }
        PMProducer.bufSize = producerCfg.bufferSize;
        strcpy(PMProducer.name, producerCfg.name);

        System_printf ("Debug: PM Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                        producerCfg.name, PMProducer.prodHandle);

        /* Create a task to generate General Purpose producer logs */
        {
            Task_Params taskParams;

            Task_Params_init(&taskParams);
            taskParams.stackSize = 8*1024;
            taskParams.priority  = 3;
            Task_create(Test_PMProducerLoggingTask, &taskParams, NULL);
        }

            break;
        }
        case 4:
        {
            Test_showTraceVerbosity(traceObjectHandle, "L2-TRACE-OBJECT");
            break;
        }
        default:
        {
            System_printf ("Error: Invalid selection\n");
            return;
        }
    }

    /* Run the log generation loop accounting for the polling delay and the number of messages */
    if (testDuration == 0)
    {
        /* Execute the test forever. */
        while (1)
        {
            for (msgIndex = 0; msgIndex < rateLogMessage; msgIndex++)
            {
                Trace_log3 (TRACE_COMPONENT_L2_MAC_UL, DAT_COMP_LEVEL_WARNING, TRACE_NON_FOCUSED, UIAEvt_detailWithStr,
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

    /* Disconnect and delete the consumer if it had been created */
    if (consumerHandle != NULL)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (consumerHandle, &errCode) < 0)
        {
            System_printf ("Error: Disconnecting consumer failed [Error code %d]\n", errCode);
            return;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (consumerHandle, &errCode) < 0)
        {
            System_printf ("Error: Deleting consumer failed [Error code %d]\n", errCode);
            return;
        }
    }

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


