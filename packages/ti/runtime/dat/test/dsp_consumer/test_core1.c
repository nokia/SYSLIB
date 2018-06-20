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

/**********************************************************************
 ************************** Unit Test Externs *************************
 **********************************************************************/

/* Global SYSLIB Module Instance Handles: */
extern Pktlib_InstHandle    appPktlibInstanceHandle;
extern Msgcom_InstHandle    appMsgcomInstanceHandle;
extern Netfp_ClientHandle   netfpClientHandle;
extern Name_ClientHandle    nameClientHandle;
extern Name_DBHandle        globalNameDatabaseHandle;

/* DAT Heaps */
extern Pktlib_HeapHandle    datClientHeap;
extern Pktlib_HeapHandle    datProducerHeap;

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

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

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
void Test_dummyConsumerFunction(Dat_ConsHandle consumerHandle, Ti_Pkt* ptrMessage)
{
    Pktlib_freePacket(appPktlibInstanceHandle, ptrMessage);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the NETFP Client
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
    producerCfg.loggerStreamerHandle = logger0;
	producerCfg.producerType 		 = DAT_PRODUCER_UIA;

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
    producerCfg.loggerStreamerHandle = logger0;
	producerCfg.producerType 		 = DAT_PRODUCER_UIA;

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
 *      DAT test code for Core1
 *
 *  @retval
 *      Not Applicable.
 */
void Test_core1Task(UArg arg0, UArg arg1)
{
    int32_t                     testSelection;
    Dat_ProdHandle              prodHandle;
    Dat_ProducerCfg             producerCfg;
    Netfp_InboundFPHandle       fpIngressHandle;
    Netfp_OutboundFPHandle      fpEgressHandle;
    Netfp_SockAddr              sockAddress;
    Netfp_SockHandle            datSockHandle;
    int32_t                     errCode;
    uint32_t                    rateLogMessage;
    uint32_t                    testDuration;
    uint32_t					msgIndex;
    uint32_t                    logIndex = 0;
    Dat_ClientCfg               datClientCfg;
    int32_t                     clientStatus;
    Dat_ProducerStats           producerStats;
    Task_Params                 taskParams;

    /* SYNC Point: Ensure that the DAT Server has been started. This is required before
     * we create and register the DAT client. */
    while (1)
    {
        /* Try to start the server: The server might not be ready; since the DAT server and
         * client are executing in the same realm we dont need to specify the NAME client handle. */
        if (Dat_startServer (globalNameDatabaseHandle, NULL, (char*)gDatServerName, &errCode) != NULL)
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

    /* Populate the DAT client configuration
     * - The NAME client handle is set to NULL since the DAT Server & Client in the test
     *   are executing in the same realm (i.e. on DSP) */
    strcpy (datClientCfg.clientName, "Dat_Client_LTE9A_L1");
    strcpy (datClientCfg.serverName, gDatServerName);
    datClientCfg.pktlibInstHandle  = appPktlibInstanceHandle;
    datClientCfg.msgcomInstHandle  = appMsgcomInstanceHandle;
    datClientCfg.databaseHandle    = globalNameDatabaseHandle;
    datClientCfg.clientHeapHandle  = datClientHeap;
    datClientCfg.nameClientHandle  = NULL;
    datClientCfg.realm             = Dat_ExecutionRealm_DSP;
    datClientCfg.id                = DNUM;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.malloc            = Dat_osalMalloc;
    datClientCfg.free              = Dat_osalFree;
    datClientCfg.beginMemAccess    = Dat_osalBeginMemoryAccess;
    datClientCfg.endMemAccess      = Dat_osalEndMemoryAccess;
    datClientCfg.createSem         = Dat_osalCreateSem;
    datClientCfg.deleteSem         = Dat_osalDeleteSem;
    datClientCfg.postSem           = Dat_osalPostSem;
    datClientCfg.pendSem           = Dat_osalPendSem;
    datClientCfg.enterCS           = Dat_osalEnterSingleCoreCriticalSection;
    datClientCfg.exitCS            = Dat_osalExitSingleCoreCriticalSection;

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

    /* Launch the DAT Client Execution Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 1;
    Task_create(Test_datClient, &taskParams, NULL);

    /* Display the CLI menu */
    while (1)
    {
        System_printf ("**************************************\n");
        System_printf ("********* DAT CLI Test Menu **********\n");
        System_printf ("**************************************\n");
        System_printf ("1. Producer/Consumer API Testing\n");
        System_printf ("2. Producer (Core1)-Consumer(Core0)        [No Debug Streaming]\n");
        System_printf ("3. Producer (Core1)-Consumer(Core0)        [Debug Streaming]\n");

        /* Wait for the user input. */
        System_printf ("Please enter your selection:");
        scanf ("%d", &testSelection);
        if ((testSelection >= 1) && (testSelection <= 3))
            break;
    }

    /* Execute the unit tests */
    switch (testSelection)
    {
        case 1:
        {
            /* Test the producer & consumer creation/deletion API */
            if (Test_producerConsumer() < 0)
                System_printf ("Error: Producer/Consumer creation/deletion tests failed\n");
            else
                System_printf ("Debug: Producer/Consumer creation/deletion tests passed\n");
            break;
        }
        case 2:
        {
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

		    /* Initialize and create the DAT producer */
		    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

            /* Populate the producer configuration.
             * - No Debug streaming */
            strcpy(producerCfg.name, gProducerName);
            producerCfg.heapHandle           = datProducerHeap;
            producerCfg.debugSocketHandle    = NULL;
            producerCfg.loggerStreamerHandle = logger0;
			producerCfg.producerType 		 = DAT_PRODUCER_UIA;

            /* Create the producer */
            prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
            if (prodHandle == NULL)
            {
                System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
                return;
            }
            System_printf ("Debug: Producer '%s' Handle %x created successfully\n", producerCfg.name, prodHandle);
            break;
        }
        case 3:
        {
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

            /* Create a socket */
        	datSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
            if (datSockHandle == NULL)
        	{
                System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        	    return;
            }

            /* Populate the binding information */
            memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
        	sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
        	sockAddress.sin_port                = 1235 + DNUM;
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
            sockAddress.sin_port                    = 1235;
        	sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

            /* Connect the socket */
        	if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
            {
	            System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        		return;
            }
            System_printf ("Debug: DAT consumer socket has been connected successfully\n");

		    /* Initialize and create the DAT producer */
		    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

            /* Populate the producer configuration.
             * - No Debug streaming */
            strcpy(producerCfg.name, gProducerName);
            producerCfg.heapHandle           = datProducerHeap;
            producerCfg.debugSocketHandle    = datSockHandle;
            producerCfg.loggerStreamerHandle = logger0;
			producerCfg.producerType 		 = DAT_PRODUCER_UIA;

            /* Create the producer */
            prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
            if (prodHandle == NULL)
            {
                System_printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
                return;
            }
            System_printf ("Debug: Producer '%s' Handle %x created successfully\n", producerCfg.name, prodHandle);
            break;
        }
        default:
        {
            System_printf ("Error: Unsupported option\n");
            break;
        }
    }

    /* Run the log generation loop accounting for the polling delay and the number of messages */
	if (testDuration == 0)
	{
        /* Execute the test forever. */
		while (1)
		{
            for (msgIndex = 0; msgIndex < rateLogMessage; msgIndex++)
                Log_write3(UIAEvt_detailWithStr, 0x10, (IArg)"Test Log Generation Index: %x", logIndex++);
            Task_sleep(1);
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
    System_printf ("Debug: Producer Statistics for Producer %p\n", prodHandle);
    System_printf ("Buffer Exchanged   : %d\n", producerStats.bufferExchange);
    System_printf ("Buffer Overrun     : %d\n", producerStats.bufferOverrun);
    System_printf ("No Consumers       : %d\n", producerStats.noConsumers);
    System_printf ("Streaming Error    : %d\n", producerStats.debugStreamingError);
    System_printf ("Streamed Messages  : %d\n", producerStats.debugStreaming);
    System_printf ("Allocated Failures : %d\n", producerStats.allocFailures);
    System_printf ("Max Queue Depth    : %d\n", producerStats.maxQueueDepth);
    return;
}

