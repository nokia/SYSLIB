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

/* MCSDK Include files */
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/memlog/memlog.h>
#include <ti/runtime/name/name_proxyClient.h>
#include "l2_lte.h"

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* LTE Stack Domain MCB */
extern AppLTEStackDomainMCB    myAppDomain;

/**********************************************************************
 ************************* Local definitions **************************
 **********************************************************************/
typedef struct memLogParam
{
    /* Producer name */
    char*                    producerName;

    /* Producer controller handler */
    Dat_prodCtrlHandle      ptrProdCtrlr;

}memLogParam;


/* Threads to run memory logging */
pthread_t               memLoggingMasterThread;
pthread_t               memLoggingSlaveThread;

uint32_t                gThreadTerminate = 0;

/**********************************************************************
 **************************  Test Functions **************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      This task to consumer buffers for all created consumers.
 *
 *  @retval
 *      Not Applicable.
 */
void* Log_consumerTask(void* arg)
{
    Ti_Pkt*         ptrMessage;
    uint8_t*        ptrBuffer;
    uint32_t        bufferSize;
    int32_t         errCode;
    AppLTEStackDomainMCB* ptrLTEStackDomain;

    /* Get the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg;

    /* This task is created after consumer is created. */
    while(!gThreadTerminate)
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

                    /* Send the message out via the DAT Socket Handle */
                    if (Netfp_send (myAppDomain.datSockHandle, ptrMessage, 0, &errCode) < 0)
                    {
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

                    /* Send the message out via the DAT Socket Handle */
                    if (Netfp_send (myAppDomain.datSockHandle, ptrMessage, 0, &errCode) < 0)
                    {
                        Pktlib_freePacket (Domain_getPktlibInstanceHandle(myAppDomain.syslibHandle), ptrMessage);
                    }
                }
            }while(ptrMessage);
        }

        /* Poll consumer every 1ms */
        usleep(1000);
    }
    return NULL;
}

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
    return hplib_vmMemAlloc (size, 0, 0);
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
    /* TODO: Currently HPLIB does not support free */
}
#if 0
/**
 *  @b Description
 *  @n
 *      The function is used to setup the logging for the L2 Data plane
 *      Logging will allow the L2 DP to use the UIA logging streamers to
 *      generate and stream out data to the System analyzer.
 *
 *  @param[in]  ptrSimPhyDomain
 *      Pointer to the Simulated PHY Domain application MCB
 *  @param[in]  datSockHandle
 *      Socket handle to be used for streaming
 *  @param[in]  producerName
 *      Producer name
 *  @param[in]  ptrL2ResourceCfg
 *      Pointer to the L2 resource configuration
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
    char*                 producerName,
    Resmgr_ResourceCfg*   ptrL2ResourceCfg
)
{
    Dat_ProducerCfg         producerCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    uint16_t                crcApp16;
    char appName[] =        "datArm";

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration:
     *  - The heap is used for the exchange of messages between the L1 DAT Producer and the L2 Consumer */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "DatProducerHeap_%d", ptrLTEStackDomain->appId);
    heapCfg.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    heapCfg.memRegion                       = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = Log_dataHeapMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Log_dataHeapFree;
    ptrLTEStackDomain->datProducerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (ptrLTEStackDomain->datProducerHeap == NULL)
    {
        printf ("Error: Unable to create the DAT producer heap [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT Producer Heap [Free Queue %x]\n", Pktlib_getInternalHeapQueue(ptrLTEStackDomain->datProducerHeap));

    /* Initialize the producer configuration. */
    memset ((void*)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, producerName);
    producerCfg.heapHandle           = ptrLTEStackDomain->datProducerHeap;
    producerCfg.debugSocketHandle    = datSockHandle;
    producerCfg.loggerStreamerHandle = NULL;
    producerCfg.producerType         = DAT_PRODUCER_UIA;
    producerCfg.bufferSize           = 1408;
    producerCfg.isMainLogger         = 1;

    /* Generate crc16 for loggerStreamer */
    crcApp16 = LoggerStreamer2_generateCRC((unsigned char const *)appName, (int)strlen(appName));
    producerCfg.crcApp16             = crcApp16;

    /* Create the producer */
    ptrLTEStackDomain->producerHandle = Dat_createProducer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                          &producerCfg, NULL, &errCode);
    if (ptrLTEStackDomain->producerHandle == NULL)
    {
        printf ("Error: Unable to create the producer %s [Error code %d]\n", producerName, errCode);
        return -1;
    }
    printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                   producerName, (uint32_t)ptrLTEStackDomain->producerHandle);

    return 0;
}
#endif

/**
 *  @b Description
 *  @n
 *     Task to handle memory logging.
 *
 *  @param[in]  arg
 *      Memlog channel name
 *
 *  @retval
 *      Not Applicable.
 */
static void* Log_memLoggingThread(void* arg)
{
    int32_t                 errCode;
    Memlog_CtrlHandle       ptrMemlogCtrl;
    MemLog_LoggerInfo       memlogInfo;
    char                    memlogChanName[MEMLOG_MAX_CHAR];
    uint32_t                mmap_fd;
    uint32_t                pageSize;
    void*                   mmapBaseAddress = NULL;
    char                    logFileName[64];
    FILE*                   fp = NULL;

    /* Get MEMLOG controller */
    ptrMemlogCtrl = (Memlog_CtrlHandle)arg;

    /* Get MEMLOG channel name */
    if (Memlog_getMemLogChanName(ptrMemlogCtrl, &memlogChanName[0], &errCode) < 0)
    {
        /* Get producer info failed */
        printf("Get MEMLOG channel name failed, error Code = %d \n", errCode);
        goto errExit;
    }

    /* Get MEMLOG channel info */
    if (Memlog_getMemlogChanInfo(ptrMemlogCtrl, &memlogInfo, &errCode) < 0)
    {
        /* Get producer info failed */
        printf("Get producer info failed, error Code = %d \n", errCode);
        goto errExit;
    }

    printf("Debug: Got producer information for %s\n", memlogChanName);
    printf("    memory address=0x%x\n", memlogInfo.memBase);
    printf("    memory size=0x%x\n", memlogInfo.memSize);
    printf("    buffer size=%d\n",   memlogInfo.bufferSize);
    printf("    producer realm=%d\n", memlogInfo.realm);

    /* Memory map the logging buffer */
    if (memlogInfo.realm == Memlog_ExecutionRealm_DSP)
    {
        /* Open the /dev/mem file - for non-cachable memroy */
        mmap_fd = open("/dev/mem", (O_RDWR | O_SYNC));
        if(mmap_fd == -1)
        {
            printf("Error: Unable to open device memory file\n");
            goto errExit;
        }

        /* Get the page size. */
        pageSize = sysconf(_SC_PAGE_SIZE);
        if (pageSize <= 0)
            goto errExit;

        /* Ensure block size and physical addresses are aligned to page size. */
        if ((memlogInfo.memSize % pageSize) || ((uint32_t)memlogInfo.memBase % pageSize))
        {
             printf("Error: Logging memory (base=0x%x, size=0x%x) is not page size (%d) aligned\n",
                     memlogInfo.memBase, memlogInfo.memSize, pageSize);
             goto errExit;
        }

        /* Create a mapping of the physical address. */
        mmapBaseAddress = mmap(0, memlogInfo.memSize, (PROT_READ|PROT_WRITE), MAP_SHARED, mmap_fd, (off_t)memlogInfo.memBase);
        if(mmapBaseAddress == MAP_FAILED)
        {
            printf("Error: Unable to map log memory!\n");
            goto errExit;
        }
        /* Close mmap file */
        close(mmap_fd);

        printf("Debug: Log memory(0x%x) is successfully mapped to 0x%x\n",  memlogInfo.memBase, (uint32_t)mmapBaseAddress);
    }
    else
    {
        mmapBaseAddress = (void *)memlogInfo.memBase;
    }
    printf("    memory virtual address=0x%x\n", (uint32_t)mmapBaseAddress);

    /* Open the log file */
    sprintf(logFileName, "%s%s.%s", "/tmp/", memlogChanName, "log");

    printf("log file: %s\n", logFileName);
    /* open a file, save the log in the file */
    fp = fopen(logFileName, "w");
    if(fp == NULL)
    {
        printf("ERROR: Open file %s failed.\n", logFileName);
        goto errExit;
    }

    /* Save log file periodically */
    while(!gThreadTerminate)
    {
        //printf("MemLogging test loop:%d\n", loop++);
        /* Sleep for 1s */
        usleep(1000000);

        /* Stop logging */
        Memlog_stopLogging (ptrMemlogCtrl, &errCode);

        /* Mandatory delay for 100ms to wait until producer side has done with writing memory */
        usleep(100000);

        if (Memlog_saveMetaInfoInFile(ptrMemlogCtrl, fp, &errCode) < 0)
        {
            /* Error: Unable to send out the packet. */
            printf ("Error: Save meta info  failed with error: %x.\n", errCode);
            goto errExit;
        }

        /* Save logs in a file in memory dump format */
        if(fseek(fp, 0, SEEK_CUR) < 0 )
        {
            /* Error: Unable to send out the packet. */
            printf ("Error: fseek() failed.\n");
            goto errExit;
        }

        if(fwrite((void *)mmapBaseAddress, 128,  memlogInfo.memSize/128, fp) < 0 )
        {
            /* Error: Unable to send out the packet. */
            printf ("Error: fwrite() failed.\n");
            goto errExit;
        }

        /* Start logging */
        Memlog_startLogging (ptrMemlogCtrl, &errCode);
    }

    Memlog_startLogging (ptrMemlogCtrl, &errCode);
errExit:

    /* Delete producer controller */
    if( ptrMemlogCtrl )
    {
        /* Delete producer controller */
        if ( Memlog_deleteMemlogController(ptrMemlogCtrl, &errCode) < 0)
        {
            printf("Delete memlog controller for %s failed with error Code=%d\n", memlogChanName, errCode);
        }
        printf("Memlog controller has been deleted!\n");
    }

    /* Close log file */
    if(fp)
        fclose(fp);

    /* Unmap LOg memory */
    if(mmapBaseAddress && (memlogInfo.realm == Memlog_ExecutionRealm_DSP) )
        munmap(mmapBaseAddress,  memlogInfo.memSize);

    printf("Debug: Thread for memory logging for producer %s exits!\n", memlogChanName);

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the logging infrastructure
 *
 *  @param[in]  ptrLTEStackDomain
 *      Pointer to the LTE Stack Domain application MCB
 *  @param[in]  ptrL2ResourceCfg
 *      Pointer to the L2 resource configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Log_initLogging (AppLTEStackDomainMCB* ptrLTEStackDomain, Resmgr_ResourceCfg* ptrL2ResourceCfg)
{
    Dat_ConsumerCfg         consumerCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;
    Netfp_SockAddr          sockAddress;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration:
     *  - The heap is used for the exchange of messages between the L1 DAT Producer and the L2 Consumer */
    snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "DatConsumerHeap_%d", ptrLTEStackDomain->appId);
    heapCfg.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    heapCfg.memRegion                       = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = 1536;
    heapCfg.numPkts                         = 256;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = Log_dataHeapMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Log_dataHeapFree;
    ptrLTEStackDomain->datConsumerHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (ptrLTEStackDomain->datConsumerHeap == NULL)
    {
        printf ("Error: Unable to create the DAT consumer heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the DAT Socket handle */
    ptrLTEStackDomain->datSockHandle = Netfp_socket (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                 Netfp_SockFamily_AF_INET, &errCode);
    if (ptrLTEStackDomain->datSockHandle == NULL)
    {
        printf ("Error: DAT Socket Creation Failed [Error Code %d]\n", errCode);
        return -1;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 1235;
    sockAddress.op.bind.inboundFPHandle = ptrLTEStackDomain->fapiTracingIngressFastPath;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = 0;

    /* Bind the socket. */
    if (Netfp_bind (ptrLTEStackDomain->datSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: DAT socket Bind Failed [Error Code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT consumer socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1235;
    sockAddress.op.connect.outboundFPHandle = ptrLTEStackDomain->fapiTracingEgressFastPath;

    /* Connect the socket */
    if (Netfp_connect(ptrLTEStackDomain->datSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: DAT socket connect Failed [Error Code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT consumer socket has been connected successfully\n");

    /* Initialize the consumer configuration. */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    strcpy(consumerCfg.producerName, "UIA_CONTRACT_PHY_SLAVE0");
    consumerCfg.heapHandle          = ptrLTEStackDomain->datConsumerHeap;

    /* Create the consumer */
    ptrLTEStackDomain->consumerPhySlaveHandle = Dat_createConsumer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                    &consumerCfg, &errCode);
    if (ptrLTEStackDomain->consumerPhySlaveHandle == NULL)
    {
        printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT consumer %p has been created successfully\n", ptrLTEStackDomain->consumerPhySlaveHandle);

    /* Initialize the consumer configuration. */
    memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

    /* Populate the consumer configuration. */
    strcpy(consumerCfg.producerName, "UIA_CONTRACT_PHY_MASTER1");
    consumerCfg.heapHandle          = ptrLTEStackDomain->datConsumerHeap;

    /* Create the consumer */
    ptrLTEStackDomain->consumerPhyMasterHandle = Dat_createConsumer (Domain_getDatClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                     &consumerCfg, &errCode);
    if (ptrLTEStackDomain->consumerPhyMasterHandle == NULL)
    {
        printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: DAT consumer %p has been created successfully\n", ptrLTEStackDomain->consumerPhyMasterHandle);

    /* Connect the PHY SLAVE Consumer/Producer: This can only be done once the producers have been created */
    while (1)
    {
        if (Dat_connectConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) == 0)
            break;

        if (errCode != DAT_ENOTREADY)
        {
            printf ("Error: Connect Consumer/Producer failed [Error code %d]\n", errCode);
            return -1;
        }
        usleep(100);
    }
    printf ("Debug: DAT consumer %p has been connected successfully\n", ptrLTEStackDomain->consumerPhySlaveHandle);

    /* Connect the PHY MASTER Consumer/Producer: This can only be done once the producers have been created */
    while (1)
    {
        if (Dat_connectConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) == 0)
            break;

        if (errCode != DAT_ENOTREADY)
        {
            printf ("Error: Connect Consumer/Producer failed [Error code %d]\n", errCode);
            return -1;
        }
        usleep(100);
    }
    printf ("Debug: DAT consumer %p has been connected successfully\n", ptrLTEStackDomain->consumerPhyMasterHandle);

    /* Launch the initialization thread */
    errCode = pthread_create (&ptrLTEStackDomain->consumerThread, NULL, (void *)Log_consumerTask, ptrLTEStackDomain);
    if (errCode < 0)
    {
    	printf ("Error: Unable to create the consumer thread [Error code %d]\n", errCode);
        return -1;
    }

    /*******************************************************************************
     * TEST: Creating MEMLOG controller
     *******************************************************************************/

    /* Create Memlog controller for PHY master UIA producer */
    while( (ptrLTEStackDomain->phyMasterMemlogChan = Memlog_createMemlogController(Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                     "UIA_CONTRACT_PHY_MASTER1", &errCode)) == NULL)
    {
        if(errCode == NAME_ENOTFOUND)
        {
            /* Can not find the memlog producer wait and try again */
            usleep(10000);

            continue;
        }
        /* Creation failed, print out the error */
        printf("Create memlogging controller for %s failed, error Code = %d \n", "UIA_CONTRACT_PHY_MASTER1", errCode);
        return -1;
    }

    /* Create thread to handle memory logging */
    errCode = pthread_create (&memLoggingMasterThread, NULL, Log_memLoggingThread, ptrLTEStackDomain->phyMasterMemlogChan);
    if (errCode < 0)
    {
        printf ("Error: Memory logging thread for UIA_CONTRACT_PHY_MASTER1 failed to start error code %d \n", errCode);
        return -1;
    }

    /* Create Memlog controller for PHY master UIA producer */
    while ( (ptrLTEStackDomain->phySlaveMemlogChan = Memlog_createMemlogController( Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle),
             "UIA_CONTRACT_PHY_SLAVE0", &errCode)) == NULL)
    {
        if(errCode == NAME_ENOTFOUND)
        {
            /* Can not find the memlog producer wait and try again */
            usleep(10000);

            continue;
        }
        /* Creation failed, print out the error */
        printf("Create memlogging controller for %s failed, error Code = %d \n", "UIA_CONTRACT_PHY_SLAVE0", errCode);
        return -1;
    }

    /* Create thread to handle memory logging */
    errCode = pthread_create (&memLoggingSlaveThread, NULL, Log_memLoggingThread, ptrLTEStackDomain->phySlaveMemlogChan);
    if (errCode < 0)
    {
        printf ("Error: Memory logging thread for UIA_CONTRACT_PHY_SLAVE0 failed to start error code %d \n", errCode);
        return -1;
    }

    /* Create Memlog controller for PHY master UIA producer */
    while( (ptrLTEStackDomain->phyMasterMemlogChan = Memlog_createMemlogController(Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                     "UIA_CONTRACT_PHY_MASTER3", &errCode)) == NULL)
    {
        if(errCode == NAME_ENOTFOUND)
        {
            /* Can not find the memlog producer wait and try again */
            usleep(10000);

            continue;
        }
        /* Creation failed, print out the error */
        printf("Create memlogging controller for %s failed, error Code = %d \n", "UIA_CONTRACT_PHY_MASTER3", errCode);
        return -1;
    }

    /* Create thread to handle memory logging */
    errCode = pthread_create (&memLoggingMasterThread, NULL, Log_memLoggingThread, ptrLTEStackDomain->phyMasterMemlogChan);
    if (errCode < 0)
    {
        printf ("Error: Memory logging thread for UIA_CONTRACT_PHY_MASTER3 failed to start error code %d \n", errCode);
        return -1;
    }

    /* Create Memlog controller for PHY master UIA producer */
    while ( (ptrLTEStackDomain->phySlaveMemlogChan = Memlog_createMemlogController( Domain_getMemlogInstanceHandle(ptrLTEStackDomain->syslibHandle),
             "UIA_CONTRACT_PHY_SLAVE2", &errCode)) == NULL)
    {
        if(errCode == NAME_ENOTFOUND)
        {
            /* Can not find the memlog producer wait and try again */
            usleep(10000);

            continue;
        }
        /* Creation failed, print out the error */
        printf("Create memlogging controller for %s failed, error Code = %d \n", "UIA_CONTRACT_PHY_SLAVE2", errCode);
        return -1;
    }

    /* Create thread to handle memory logging */
    errCode = pthread_create (&memLoggingSlaveThread, NULL, Log_memLoggingThread, ptrLTEStackDomain->phySlaveMemlogChan);
    if (errCode < 0)
    {
        printf ("Error: Memory logging thread for UIA_CONTRACT_PHY_SLAVE2 failed to start error code %d \n", errCode);
        return -1;
    }

    sleep(1);
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

    printf ("Debug: Delete producer controllers for memory logging\n");
    /* Remove producer controllers for master */
    if (ptrLTEStackDomain->phyMasterMemlogChan)
    {
        gThreadTerminate = 1;
        pthread_cancel(memLoggingMasterThread);
        pthread_join(memLoggingMasterThread, NULL);
        /* Delete producer controller */
        if (Memlog_deleteMemlogController(ptrLTEStackDomain->phyMasterMemlogChan, &errCode) < 0)
        {
            System_printf("Delete producer controller for master core failed with error Code=%d\n", errCode);
        }
    }

    /* Remove producer controllers for slave */
    if (ptrLTEStackDomain->phySlaveMemlogChan)
    {
        pthread_cancel(memLoggingSlaveThread);
        pthread_join(memLoggingSlaveThread, NULL);
        /* Delete producer controller */
        if ( Memlog_deleteMemlogController(ptrLTEStackDomain->phySlaveMemlogChan, &errCode) < 0)
        {
            System_printf("Delete producer controller for slave core failed with error Code=%d\n", errCode);
        }
    }

    printf ("Debug: Shutting down the logging consumers\n");

    /* Disconnect the consumers from the producers: */
    if (ptrLTEStackDomain->consumerPhyMasterHandle != NULL)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) < 0)
        {
            printf ("Error: Disconnecting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (ptrLTEStackDomain->consumerPhyMasterHandle, &consumerStatus, &errCode) < 0)
            {
                printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return -1;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (ptrLTEStackDomain->consumerPhyMasterHandle, &errCode) < 0)
        {
            printf ("Error: Deleting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    printf ("Debug: PHY Master consumer has been deleted\n");

    /* Disconnect the consumers from the producers: */
    if (ptrLTEStackDomain->consumerPhySlaveHandle != NULL)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) < 0)
        {
            printf ("Error: Disconnecting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (ptrLTEStackDomain->consumerPhySlaveHandle, &consumerStatus, &errCode) < 0)
            {
                printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return -1;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (ptrLTEStackDomain->consumerPhySlaveHandle, &errCode) < 0)
        {
            printf ("Error: Deleting PHY master consumer failed [Error code %d]\n", errCode);
            return -1;
        }
    }
    printf ("Debug: PHY Slave consumer has been deleted\n");

    /* Delete L2 producers, the producer is created without any consumer, hence delete directly */
    if (Dat_deleteProducer (ptrLTEStackDomain->producerHandle, &errCode) < 0)
    {
        printf ("Error: DAT Delete L2 Producer failed [Error code %d]\n", errCode);
        return -1;
    }
    printf ("Debug: L2 producer has been deleted\n");

    /* Shutdown the NETFP socket which was created */
    if (Netfp_closeSocket (ptrLTEStackDomain->datSockHandle, &errCode) < 0)
        printf ("Error: DAT socket deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: DAT socket deleted successfully\n");

    /* Shutdown the PKTLIB heaps for consumers */
    if (Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->datConsumerHeap, &errCode) < 0)
        printf ("Error: PKTLIB DAT consumer heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: PKTLIB DAT consumer heap deleted successfully\n");

    /* Shutdown the PKTLIB heaps for producers */
    if (Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->datProducerHeap, &errCode) < 0)
        printf ("Error: PKTLIB DAT producer heap deletion failed [Error code %d]\n", errCode);
    else
        printf ("Debug: PKTLIB DAT producer heap deleted successfully\n");

    return 0;


}

