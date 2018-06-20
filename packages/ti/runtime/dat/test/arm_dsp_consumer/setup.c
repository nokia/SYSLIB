/*
 *   @file  setup.c
 *
 *   @brief
 *      The file implements the functionality which is required before
 *      the tests can be created
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
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>

/* MCSDK Include files */
#include <ti/runtime/hplib/hplib.h>
#include <ti/runtime/hplib/hplib_vm.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

/* cUIA Include Files. */
#include <ti/uiaplus/loggers/multistream/LoggerStreamer2.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/events/UIABenchmark.h>
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/events/UIAErr.h>
#include <ti/uia/events/UIAStatistic.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <ti/uia/runtime/LogUC.h>

/* Trace verbosity test Include Files */
#include "../trace_L2.h"
#include "../trace_log.h"
#include "../benchmark.h"

/* For Debugging only. */
#define System_printf   printf

/**********************************************************************
 ************************ Unit Test Global variables ******************
 **********************************************************************/

/* NETFP Socket Handle: */
Netfp_SockHandle        datSockHandle;
Netfp_SockHandle        datProducerSockHandle;

/* Producer name: This is a well known producer name across the test domain */
static const char*      gProducerName = "Test-UIA";

/* Trace object verbosity array used for Dat_filter() */
Dat_TraceObjectBody*    ptrTraceObjBody = NULL;

/* Logger handle - created inside DAT library */
void*                   logger0;

/**********************************************************************
 ************************ Extern Declarations *************************
 **********************************************************************/

/* SYSLIB Module Handle. */
extern Netfp_ClientHandle      netfpClientHandle;
extern Dat_ClientHandle        datClientHandle;
extern Pktlib_InstHandle       appPktlibInstanceHandle;
extern Name_ClientHandle       nameClientHandle;
extern Memlog_InstHandle       memlogInstHandle;

/* DAT Consumer Heap: */
extern Pktlib_HeapHandle       datConsumerHeap;

/* DAT PM Consumer Heap: */
extern Pktlib_HeapHandle       datPMConsumerHeap;

/* DAT ARM Producer Heap */
extern Pktlib_HeapHandle       datProducerHeap;

/* DAT ARM GP Producer Heap */
extern Pktlib_HeapHandle       datGPProducerHeap;

/* Test Execution status */
extern volatile uint32_t       testComplete;

/* Global System configuration handle */
Resmgr_SysCfgHandle            handleSysCfg;

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Consumer  function which is invoked on
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
void Test_consumerFunction(Dat_ConsHandle consumerHandle, Ti_Pkt* ptrMessage)
{
    uint8_t*    ptrBuffer;
    uint32_t    bufferSize;
    int32_t     errCode;

    /* Get the data buffer and size */
    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);

    /* Send the message out via the DAT Socket Handle */
    if (Netfp_send (datSockHandle, ptrMessage, 0, &errCode) < 0)
    {
        printf ("Debug: Unable to send the message via NETFP [Error code %d]\n", errCode);
        Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *     Task to handle memory logging.
 *
 *  @param[in]  arg
 *      Producer name
 *
 *  @retval
 *      Not Applicable.
 */
static void* Test_memLoggingThread(void* arg)
{
    int32_t                 errCode;
    Memlog_CtrlHandle       ptrMemlogCtrl;
    MemLog_LoggerInfo       memLogInfo;
    char*                   producerName;
    uint32_t                mmap_fd;
    uint32_t                pageSize;
    void*                   mmapBaseAddress = NULL;
    char                    logFileName[64];
    FILE*                   fp = NULL;
    uint32_t                loop = 300000;

    /* Get Producer Name */
    producerName = (char *)arg;

    /* Create Producer Controller for core0 UIA producer */
    while ( (ptrMemlogCtrl = Memlog_createMemlogController(memlogInstHandle, producerName, &errCode)) == NULL)
    {

        if(errCode == NAME_ENOTFOUND)
        {
            /* Can not find the memlog producer wait and try again */
            usleep(10000);

            continue;
        }
        /* Creation failed, print out the error */
        printf("Create memlogging controller for %s failed, error Code = %d \n", producerName, errCode);
        goto errExit;
    }

    /* Check the logging type to see if it is Memory logging */
    if (Memlog_getMemlogChanInfo(ptrMemlogCtrl, &memLogInfo, &errCode) < 0)
    {
        /* Get producer info failed */
        printf("Get producer info failed, error Code = %d \n", errCode);
        goto errExit;
    }

    printf("Debug: Got producer information for %s\n", producerName);
    printf("    memory address=0x%x\n", memLogInfo.memBase);
    printf("    memory size=0x%x\n", memLogInfo.memSize);
    printf("    buffer size=%d\n",   memLogInfo.bufferSize);
    printf("    producer realm=%d\n", memLogInfo.realm);

    if (memLogInfo.realm == Memlog_ExecutionRealm_DSP)
    {
        /* Map the memory */
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
        if ((memLogInfo.memSize % pageSize) || ((uint32_t)memLogInfo.memBase % pageSize))
        {
             printf("Error: Logging memory (base=0x%x, size=0x%x) is not page size (%d) aligned\n",
                     memLogInfo.memBase, memLogInfo.memSize, pageSize);
             goto errExit;
        }

        /* Create a mapping of the physical address. */
        mmapBaseAddress = mmap(0, memLogInfo.memSize, (PROT_READ|PROT_WRITE), MAP_SHARED, mmap_fd, (off_t)memLogInfo.memBase);
        if(mmapBaseAddress == MAP_FAILED)
        {
            printf("Error: Unable to map log memory!\n");
            goto errExit;
        }
        /* Close mmap file */
        close(mmap_fd);

        printf("Debug: Log memory(0x%x) is successfully mapped to 0x%x\n",  memLogInfo.memBase, (uint32_t)mmapBaseAddress);
    }
    else
    {
        mmapBaseAddress = (void *)memLogInfo.memBase;
    }
    printf("    memory virtual address=0x%x\n", (uint32_t)mmapBaseAddress);

    /* Open the log file */
    sprintf(logFileName, "%s%s.%s", "/tmp/", producerName, "log");

    printf("log file: %s\n", logFileName);
    /* open a file, save the log in the file */
    fp = fopen(logFileName, "w");
    if(fp == NULL)
    {
        printf("ERROR: Open file %s failed.\n", logFileName);
        goto errExit;
    }

    /* Save log file periodically */
    while( (loop--) && (!testComplete))
    {
        printf("MemLogging test loop:%d\n", loop);
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

        if(fwrite((void *)mmapBaseAddress, 128,  memLogInfo.memSize/128, fp) < 0 )
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
            printf("Delete memlog controller for %s failed with error Code=%d\n", producerName, errCode);
        }
        printf("Memlog controller has been deleted!\n");
    }

    /* Close log file */
    if(fp)
        fclose(fp);

    /* Unmap LOg memory */
    if(mmapBaseAddress && (memLogInfo.realm == Memlog_ExecutionRealm_DSP) )
        munmap(mmapBaseAddress,  memLogInfo.memSize);

    printf("Debug: Thread for memory logging for producer %s exits!\n", producerName);

    return NULL;
}

/**
 *  @b Description
 *  @n
 *     Consumer task example that is used to get messages from producer.
 *  In this example, only one consumer is handled.
 *
 *  @param[in]  arg
 *      Pointer to the consumerHandle
 *
 *  @retval
 *      Not Applicable.
 */
static void* Test_consumerTask(void* arg)
{
    Dat_ConsHandle    consumerHandle;
    Ti_Pkt*           ptrMessage;

    printf("Debug: Consumer thread for consumer:%p\n", arg);
    /* Get the consumer handle */
    consumerHandle = (Dat_ConsHandle)arg;

    while(1)
    {
        do{
            /* Get Consumer packet */
            ptrMessage = Dat_processConsumer(consumerHandle);
            if(ptrMessage)
            {
                Test_consumerFunction(consumerHandle, ptrMessage);
            }
        }while(ptrMessage);

        /* Sleep for 1ms */
        usleep(1000);
        Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
    }
    printf("Test_consumerTask exits!\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      PM measurements consumer function.
 *
 *  @param[in]  consumerHandle
 *      DAT Consumer Handle
 *  @param[in]  ptrMessage
 *      Pointer to the received message
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_PMConsumerFunction(Ti_Pkt* ptrMessage, uint32_t dataOffset)
{
    uint8_t*    ptrBuffer;
    uint32_t    bufferSize;
    uint32_t    idx;
    uint32_t    numElem=50;
    uint32_t*   buffer;

    /* Get the data buffer and size */
    Pktlib_getDataBuffer (ptrMessage, &ptrBuffer, &bufferSize);
    buffer = (uint32_t *)(ptrBuffer + dataOffset);

    /* Show stats */
    printf("PM Stats: \n");
    for (idx=0; idx < numElem; idx=idx+4)
    {
        printf("%d    %d    %d    %d    \n", buffer[idx], buffer[idx+1], buffer[idx+2], buffer[idx+3]);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      PM measurement consumer example.
 *
 *  @param[in]  arg
 *      DAT Consumer Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void* Test_PMConsumerTask(void* arg)
{
    Dat_ConsHandle    consumerHandle;
    Ti_Pkt*           ptrMessage;
    uint32_t          dataOffset;
    int32_t           errCode;

    /* Get the consumer handle */
    consumerHandle = (Dat_ConsHandle)arg;

    /* Get data buffer offset */
    dataOffset = Dat_getGPProducerDataOffset(consumerHandle, &errCode);
    if(dataOffset == 0)
    {
        printf("Getting PM producer data offset failed with error [%x]\n", errCode);
    }
    else
        printf("PM producer data offset = %d\n", dataOffset);

    while(1)
    {
        /* Get Consumer packet */
        ptrMessage = Dat_processConsumer(consumerHandle);
        printf("ptrMessage=%p\n", ptrMessage);
        if(ptrMessage)
        {
            /* Check producer buffer */
            Test_PMConsumerFunction(ptrMessage, dataOffset);
            Pktlib_freePacket (appPktlibInstanceHandle, ptrMessage);
        }

        /* Sleep for 1s */
        usleep(1000000);
    }
    printf("Test_PMConsumerTask exits!\n");
    return NULL;
}
/*
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
    Dat_TraceComponentCfg    componentList[TRACE_COMPONENT_L2_MAX];
    Dat_TraceObjectCfg       traceObjectCfg;
    int32_t                  errCode;
    uint32_t                 index;

    /* Initialize the structure */
    memset (&componentList[0], 0, sizeof(componentList));
    memset (&traceObjectCfg, 0, sizeof (Dat_TraceObjectCfg));

    /* Get Trace Object info */
    if (Dat_getNumComponents(traceObjectHandle, traceObjName, &traceObjectCfg.numTraceComponents, &errCode) < 0)
    {
        printf("Error: Unable to get number of trace components for %s, errCode = %d\n", traceObjName, errCode);
    }

    if (Dat_getClassVerbosity(traceObjectHandle, traceObjName, &traceObjectCfg.classLevel, &traceObjectCfg.commonCompLevel, &errCode) < 0)
    {
        printf("Error: Unable to get class/common components levels for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
    }

    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        if (Dat_getComponentVerbosity(traceObjectHandle, traceObjName, index,  &componentList[index], &errCode) < 0)
        {
            printf("Error: Unable to get class/common components levels for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
        }
    }

    /* Print out the verbosity settings */
    printf("Debug: There are %d trace components in trace object %s\n", traceObjectCfg.numTraceComponents, "L2-TRACE-OBJECT");
    printf("Debug: Class level:%x\n", traceObjectCfg.classLevel);
    printf("Debug: Common component level:%x\n", traceObjectCfg.commonCompLevel);
    printf("Debug: Component verbosity level:\n");
    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        printf("Debug: Index:%d\t foucus verbosity levle:%x\t non-foucus verbosity levle:%x\t\n",
                       index, componentList[index].focusedLevel, componentList[index].nonFocusedLevel);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the trace verbosity.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t Test_deleteTraceObjects(Dat_TraceObjHandle  traceObjectHandle)
{
    int32_t errCode;

    /* Remove local trace object */
    if (Dat_deleteTraceObjectInstance(traceObjectHandle, &errCode) < 0 )
    {
        printf("Error: Removing local trace object(%p) failed with errCode = %d\n", traceObjectHandle, errCode);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This fucntion will create Local trace object instance and try to modify verbosity levels.
 *  The Global trace object is created on a DSP core.
 *
 *  @retval
 *      Not Applicable.
 */
static Dat_TraceObjHandle Test_setupTraceVerbosity(void)
{
    int32_t                  errCode;
    uint32_t                 index;
    Dat_TraceObjectCfg       traceObjectCfg;
    Dat_TraceObjHandle       traceObjectHandle = NULL;
    Dat_VerbosityCfg         verbosityCfg;
    char                     traceObjectNameArray[16][DAT_MAX_CHAR];
    uint32_t                 numTraceObject;

    /* Create a local instance of the L2 trace object. This is mandatory. */
    while (1)
    {

        /* Find the named resource. */
        traceObjectHandle = Dat_createTraceObjectInstance(datClientHandle, "L2-TRACE-OBJECT", &errCode);
        if (traceObjectHandle == NULL)
        {
            /* Check the error code. */
            if (errCode == DAT_EINVAL)
            {
                /* Resource has not yet been created; we will retry. */
            	usleep (1000);
                continue;
            }
            printf ("Error: Creating local instance of L2 trace object failed with error %x\n", errCode);
            return NULL;
        }

        /* Created the local instance of the L2 trace object successfully. */
        printf ("Debug: Local instance of L2 trace object(%p) is created.\n", traceObjectHandle);
        break;
    }

    /* Get the trace object body to be used for filter */
    ptrTraceObjBody = Dat_getTraceObjectBody (traceObjectHandle);

    /* Show the initial trace level settings */
    //Test_showTraceVerbosity(traceObjectHandle, "L2-TRACE-OBJECT");

    /****************************************************************
     *           Test modification of trace object on ARM
     ***************************************************************/
    memset((void *)&verbosityCfg, 0, sizeof(verbosityCfg));
    strncpy(verbosityCfg.traceObjectName, "L2-TRACE-OBJECT", DAT_MAX_CHAR);

    /* Modify common component verbosity level */
    verbosityCfg.verbosityOpt     = Dat_Verbosity_CommonComponent;
    verbosityCfg.verbosityLevel   = DAT_COMP_LEVEL_EXCEPTION | DAT_COMP_LEVEL_ERROR | DAT_COMP_LEVEL_WARNING |
	                            DAT_COMP_LEVEL_BENCHMARK0 | DAT_COMP_LEVEL_BENCHMARK1;
    verbosityCfg.isEnabled        = 1;

    if (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
    {
        printf("Error: Unable to modify common components verbosity for %s, errCode = %d\n", verbosityCfg.traceObjectName, errCode);
    }

    printf("Debug: Test modifying common component trace level is done. \n");

    /* Modify class verbosity level */
    verbosityCfg.verbosityOpt     = Dat_Verbosity_Class;
    verbosityCfg.verbosityLevel   = DAT_CLASS_LEVEL_BIOSLOAD | DAT_CLASS_LEVEL_BIOSMAIN | DAT_CLASS_LEVEL_BIOSTASK |
	                            DAT_CLASS_LEVEL_USERDEF0 | DAT_CLASS_LEVEL_USERDEF2;
    verbosityCfg.isEnabled        = 1;

    if (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
    {
        printf("Error: Unable to modify class verbosity for %s, errCode = %d\n", verbosityCfg.traceObjectName, errCode);
    }

    printf("Debug: Test modifying class trace level is done. \n");


    if (Dat_getNumComponents(traceObjectHandle, "L2-TRACE-OBJECT", &traceObjectCfg.numTraceComponents, &errCode) < 0)
    {
        printf("Error: Unable to get number of trace components for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
    }
    else
        printf("Debug: test verbosity modification for %d components\n", traceObjectCfg.numTraceComponents);

    /* Modify component non-focus level */
    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        verbosityCfg.verbosityOpt     = Dat_Verbosity_Component;
        verbosityCfg.verbosityLevel   = DAT_COMP_LEVEL_PE0 | DAT_COMP_LEVEL_PE1 | DAT_COMP_LEVEL_PE2;
        verbosityCfg.isEnabled        = 1;
        verbosityCfg.isComponentFocused = 0;
        verbosityCfg.traceComponentId = index;

        while (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
        {
            if(errCode == DAT_ENORESOURCE)
            {
                 /* Retry after 10ms */
                 usleep(10000);
            }
            else
                printf("Error: Unable to modify component level verbosity for %s, componentId=%d errCode = %d\n",
                    verbosityCfg.traceObjectName, index, errCode);

        }
    }

    /* Modify component focus level */
    for (index=0; index < traceObjectCfg.numTraceComponents; index++)
    {
        verbosityCfg.verbosityOpt     = Dat_Verbosity_Component;
        verbosityCfg.verbosityLevel   = DAT_COMP_LEVEL_USERDEF0 | DAT_COMP_LEVEL_USERDEF1 | DAT_COMP_LEVEL_USERDEF2 | DAT_COMP_LEVEL_USERDEF3;
        verbosityCfg.isEnabled        = 1;
        verbosityCfg.isComponentFocused = 1;
        verbosityCfg.traceComponentId = index;

        while (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
        {
            if(errCode == DAT_ENORESOURCE)
            {
                 /* Retry after 10ms */
                 usleep(10000);
            }
            else
            {
                printf("Error: Unable to modify component level verbosity for %s, componentId=%d errCode = %d\n",
                           verbosityCfg.traceObjectName, index, errCode);
            }
        }
    }

    /* Show the verbosity levels after modificaion */
    Test_showTraceVerbosity(traceObjectHandle, verbosityCfg.traceObjectName);

    printf("Debug: Test modifying component trace level is done. \n");

    /* Test get trace object name list */
    numTraceObject = (uint32_t)Dat_getTraceObjectNames(datClientHandle, 16 * DAT_MAX_CHAR, &traceObjectNameArray[0][0], &errCode);

    if ((int32_t)numTraceObject < 0)
    {
        printf("Error: getting trace object name list returns error:%x\n", errCode);
    }
    else
    {

        /* Print out the trace object list */
        for (index=0; index < numTraceObject; index++)
        {
            printf("Index:%d\t name:%s\n", index, traceObjectNameArray[index]);
        }
    }

    return traceObjectHandle;
}


/**
 *  @b Description
 *  @n
 *      This fucntion will Test verbosity by get/modify verbosity levels.
 *
 *  @param[in]  traceObjectHandle
 *      Application Identifier
 *  @param[in]  traceObjectName
 *      Name of the traceObject
 *  @retval
 *      Not Applicable.
 */
static void Test_traceVerbosity
(
    Dat_TraceObjHandle traceObjectHandle,
    char*              traceObjectName
)
{
    int32_t                  errCode;
    Dat_VerbosityCfg         verbosityCfg;
    uint32_t                 numComponent;
    uint32_t                 testSelection;
    uint32_t                 verbosityTestComplete = 0;
    uint32_t                 loop=1;

    if(traceObjectHandle == NULL)
    {
        printf("Invalid trace object handle, Test failed.\n");
        return ;
    }

    while(!verbosityTestComplete)
    {
        /* Print the verbosity test menu */
        printf("**********************************************************\n");
        printf("DAT Verbosity CLI Menu:\n");
        printf("**********************************************************\n");
        printf("1. Show current Verbosity setting\n");
        printf("2. Disable all\n");
        printf("3. Modify class verbosity level\n");
        printf("4. Modify common component verbosity level\n");
        printf("5. Modify component verbosity level\n");
        printf("6. End test\n");
        printf("Enter the selection:\n ");

        /* Get the test selection */
        scanf("%d", &testSelection);

        /* Handle different test cases */
        switch(testSelection)
        {
            case 1:
                Test_showTraceVerbosity(traceObjectHandle,traceObjectName);
                break;

            case 2:

                 /* Get the current class level setting */
                 memset((void *)&verbosityCfg, 0, sizeof(Dat_VerbosityCfg));
                 strncpy(verbosityCfg.traceObjectName, traceObjectName, DAT_MAX_CHAR);

                 /* Modify class verbosity level */
                 verbosityCfg.verbosityOpt     = Dat_Verbosity_Class;
                 verbosityCfg.verbosityLevel   = DAT_CLASS_LEVEL_DISABLEALL;
                 verbosityCfg.isEnabled        = 1;

                 /* Modify Class level verbosity */
                 if (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
                 {
                     printf("Error: Unable to modify class verbosity for %s, errCode = %d\n", verbosityCfg.traceObjectName, errCode);
                 }
                 else
                     Test_showTraceVerbosity(traceObjectHandle, verbosityCfg.traceObjectName);

                 break;

           case 3:

                 /* Prepare for the verbosity setting */
                 memset((void *)&verbosityCfg, 0, sizeof(Dat_VerbosityCfg));
                 strncpy(verbosityCfg.traceObjectName, traceObjectName, DAT_MAX_CHAR);

                 /* Get settings from user input */
                 verbosityCfg.verbosityOpt     = Dat_Verbosity_Class;
                 printf("Enter class verbosity level mask:\n");
                 scanf("%x", (int32_t *)&verbosityCfg.verbosityLevel);

                 printf("Enter enable(1) or disable (0):\n");
                 scanf("%d", (int32_t *)&verbosityCfg.isEnabled);
                 if ( verbosityCfg.isEnabled != 0 && verbosityCfg.isEnabled != 1)
                 {
                     printf("Invalid verbosity setting\n");
                     break;
                 }

                 /* Get the current class level setting */
                 if (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
                 {
                     printf("Error: Unable to modify class verbosity for %s, errCode = %d\n", verbosityCfg.traceObjectName, errCode);
                 }
                 else
                     Test_showTraceVerbosity(traceObjectHandle, verbosityCfg.traceObjectName);

                 break;

           case 4:

                 /* Get the current class level setting */
                 memset((void *)&verbosityCfg, 0, sizeof(Dat_VerbosityCfg));
                 strncpy(verbosityCfg.traceObjectName, traceObjectName, DAT_MAX_CHAR);

                 /* Modify class verbosity level */
                 verbosityCfg.verbosityOpt     = Dat_Verbosity_CommonComponent;
                 printf("Enter common component verbosity level mask:\n");
                 scanf("%x", &verbosityCfg.verbosityLevel);

                 printf("Enter enable(1) or disable (0):\n");
                 scanf("%d", (int32_t *)&verbosityCfg.isEnabled);
                 if ( verbosityCfg.isEnabled != 0 && verbosityCfg.isEnabled != 1)
                 {
                     printf("Invalid verbosity setting\n");
                     break;
                 }

                 if (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
                 {
                     printf("Error: Unable to modify class verbosity for %s, errCode = %d\n", verbosityCfg.traceObjectName, errCode);
                 }
                 else
                     Test_showTraceVerbosity(traceObjectHandle, verbosityCfg.traceObjectName);

                 break;

             case 5:

                 /* Get the current class level setting */
                 memset((void *)&verbosityCfg, 0, sizeof(Dat_VerbosityCfg));
                 strncpy(verbosityCfg.traceObjectName, traceObjectName, DAT_MAX_CHAR);

                 /* Modify class verbosity level */
                 verbosityCfg.verbosityOpt     = Dat_Verbosity_Component;
                 printf("Enter component verbosity level mask:\n");
                 scanf("%x", &verbosityCfg.verbosityLevel);

                 /* Get user input: isEnabled */
                 printf("Enter enable(1) or disable (0):\n");
                 scanf("%d", (int32_t *)&verbosityCfg.isEnabled);
                 if ( verbosityCfg.isEnabled != 0 && verbosityCfg.isEnabled != 1)
                 {
                     printf("Invalid verbosity enable/disable setting\n");
                     break;
                 }

                 /* Get user input: iscomponentfoucused  */
                 printf("Is component verbosity focused:\n");
                 scanf("%d", (int32_t *)&verbosityCfg.isComponentFocused);

                 if ( verbosityCfg.isComponentFocused != 0 && verbosityCfg.isComponentFocused != 1)
                 {
                     printf("Invalid verbosity fouse setting\n");
                     break;
                 }

                 /* Get user input: component id */
                 printf("Enter component id:\n");
                 scanf("%d", &verbosityCfg.traceComponentId);

                 /* Get test loop count */
                 printf("Enter loop count:(1 for non-stress testing)\n");
                 scanf("%d", &loop);

                 if (Dat_getNumComponents(traceObjectHandle, traceObjectName, &numComponent, &errCode) < 0)
                 {
                     printf("Error: Unable to get number of trace components for %s, errCode = %d\n", "L2-TRACE-OBJECT", errCode);
                 }
                 if (verbosityCfg.traceComponentId > numComponent)
                 {
                     printf("Invalid componnet id\n");
                     break;
                 }

                 while(loop--)
                 {

                     while (Dat_modifyVerbosity(traceObjectHandle, &verbosityCfg, &errCode) < 0)
                     {
                         if(errCode == DAT_ENORESOURCE)
                         {
                             /* Retry after 10ms */
                             usleep(10000);
                         }
                         else
                         {
                             printf("Error: Unable to modify component level verbosity for %s componentId=%d errCode = %d\n",
                                     verbosityCfg.traceObjectName, verbosityCfg.traceComponentId, errCode);
                         }
                     }
                     if (loop%1024 == 0)
                         Test_showTraceVerbosity(traceObjectHandle, verbosityCfg.traceObjectName);
                 }
                 break;

            case 6:
                 verbosityTestComplete = 1;
                 printf("Verbosity test is completed\n");
                 break;

            default:
                 break;

         }
    }
}


/**
 *  @b Description
 *  @n
 *      ARM logging thread
 *
 *  @retval
 *      Not Applicable.
 */
static void* Dat_armLoggingforMemLoggingTask(void* arg)
{
    uint32_t index;
    /* Generate logging event through loggerStreamer2 interface every 1ms */
    while(!testComplete)
    {
        for(index=0; index<50; index++)
            Log_iwriteUC3(logger0, UIAEvt_detailWithStr, 0x55, (IArg)"WARNING: ARM producer sample log message %x",(UArg)0x1000);
        usleep(1000);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      ARM logging thread
 *
 *  @retval
 *      Not Applicable.
 */
static void* Dat_armLoggingTask(void* arg)
{
    printf("Arm logging Task started\n");
    /* Generate logging event through loggerStreamer2 interface every 1ms */
    while(!testComplete)
    {

        if(Dat_filter(ptrTraceObjBody, TRACE_COMPONENT_L2_RLC_UL, DAT_COMP_LEVEL_WARNING, TRACE_FOCUSED))
            Log_iwriteUC3(logger0, UIAEvt_detailWithStr, 0x55, (IArg)"WARNING: ARM producer sample log message %x",(UArg)0x1000);
        usleep(1000);
    }

    printf("ARM logging task exit\n");
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      ARM General Purpose logging thread
 *
 *  @param[in]  arg
 *      Handle to the general purpose producer
 *
 *  @retval
 *      Not Applicable.
 */
static void* Dat_armGPLoggingTask(void* arg)
{
    uint32_t         idx;
    uint8_t*         buffer;
    Dat_ProdHandle   prodHandle;
    int32_t          errCode;

    printf("ARM General Purpose producer logging Task started for %p\n", arg);

    /* Get the producer handle */
    prodHandle = (Dat_ProdHandle)arg;

    /* Get general purpose buffer */
    buffer = Dat_getProducerBuffer(prodHandle, "Test-GP-ARM", &errCode);
    if(buffer == NULL)
    {
        printf ("Error: Unable to get the first general purpose producer(%p) buffer [Error code %d]\n",
                        prodHandle, errCode);
        while(1);
    }

    /* Fill data pattern in the buffer */
    for(idx=0;idx <1000 ;idx++)
        buffer[idx] = idx + 1 ;

    /* Perform buffer exchange every 1ms */
    while(!testComplete)
    {
        /* Wait for 1ms */
        usleep(1000);

        /* Exchange the PM buffers. */
        buffer = Dat_bufferExchange (datClientHandle, prodHandle, buffer);
        /* Fill data pattern in the buffer */
        for(idx=0;idx <1000 ;idx++)
            buffer[idx] = idx + 1 ;

    }

    printf("ARM General Purpose producer logging task exit\n");
    return NULL;
}


/**
 *  @b Description
 *  @n
 *      Entry point to the test task which executes the unit tests.
 *
 *  @retval
 *      Not Applicable.
 */
void SetupTask(void* arg0)
{
    int32_t                 errCode = 0;
    Dat_ConsHandle          consumerHandle;
    Dat_ConsumerCfg         consumerCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockAddr          sockAddress;
    Dat_ConsumerStatus      consumerStatus;
    Dat_TraceObjHandle      traceObjectHandle;
    Dat_ProdHandle          prodHandle;
    Dat_ProdHandle          prodGPHandle;
    pthread_t               consumerThread;
    pthread_t               memLoggingThread[5];
    uint32_t                testSelection;
    uint32_t                index;

    Resmgr_ResourceCfg      memLoggingResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        0,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                      Linking RAM,                              Num,     Size */
            { "MemLoggingARM",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,   512,     128},
        }
    };

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (handleSysCfg, &memLoggingResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: Memory Logging resource configuration failed [Error code %d]\n", errCode);
        return ;
    }

    printf ("Debug: Waiting for the Ingress Fast Path to be created\n");
    while (1)
    {
        /* Get the ingress fast path handle */
        fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
        if (fpIngressHandle != NULL)
            break;
        usleep(100);
    }

    /* Get the egress fast path handle */
    printf ("Debug: Waiting for the Egress Fast Path to be created\n");
    while (1)
    {
        fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
        if (fpEgressHandle != NULL)
            break;
        usleep(100);
    }

    /* Create a socket */
    datSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (datSockHandle == NULL)
    {
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
	return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 10000;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;

    /* Bind the socket. */
    if (Netfp_bind (datSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    printf ("Debug: DAT consumer socket has been bound successfully\n");

    /*********************************************************************************************
     **********************   Create ARM producer & consumer *************************************
     ********************************************************************************************/
    /* Create debug socket */
    datProducerSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (datProducerSockHandle == NULL)
    {
        printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 12350;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;

    /* Bind the socket. */
    if (Netfp_bind (datProducerSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    printf ("Debug: DAT ARM producer debug socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 51235;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(datProducerSockHandle, &sockAddress, &errCode) < 0)
    {
        printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    printf ("Debug: DAT ARM producer debug socket has been connected successfully\n");

    /* Print the test menu */
    printf("**********************************************************\n");
    printf("DAT Producer and Consumer test:\n");
    printf("**********************************************************\n");
    printf("1. Verbosity test \n");
    printf("2. ARM Producer test \n");
    printf("3. Consumer test(UDP port 10000) \n");
    printf("4. PM consumer\n");
    printf("5. Memory logging test for DSP producers (no consumer)\n");
    printf("6. Memory logging for ARM producer (no consumer)\n");
    printf("7. Exit\n");
    printf("Enter the selection:\n ");

    /* Get the test selection */
    scanf("%d", &testSelection);

    printf("Test selection:%d\n", testSelection);

    /* Handle different test cases */
    switch(testSelection)
    {
        case 1:
            /* *************************************************************
             * *********** Trace Verbosity Setup & Test  *******************
             ***************************************************************/
            /* Test Verbosity functions */
            traceObjectHandle = Test_setupTraceVerbosity();

            /* Perform verbosity test */
            Test_traceVerbosity(traceObjectHandle, "L2-TRACE-OBJECT");
            break;

        case 2:
            /* Create ARM producer */
            {
                Dat_ProducerCfg             producerCfg;
                pthread_t                   armLoggingThread;
                uint16_t                    crcApp16;
                char appName[] =            "test_dat_k2h";

                /* Initialize and create the DAT producer */
                memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

                /* Generate crc16 for loggerStreamer */
                crcApp16 = LoggerStreamer2_generateCRC((unsigned char const *)appName, (int)strlen(appName));

                /* Populate the producer configuration. */
                strcpy(producerCfg.name, "Test-UIA-ARM");
                producerCfg.heapHandle           = datProducerHeap;
                producerCfg.debugSocketHandle    = datProducerSockHandle;
                producerCfg.loggerStreamerHandle = NULL;
                producerCfg.bufferSize           = 1408;
                producerCfg.crcApp16             = crcApp16;
                producerCfg.producerType         = DAT_PRODUCER_UIA;
                producerCfg.isMainLogger         = 1;

                /* Create the producer */
                prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &logger0, &errCode);
                if (prodHandle == NULL || logger0 == NULL)
                {
                    printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: Producer '%s' Handle %p logger: %p [Streaming to System Analyzer] created successfully\n",
                                   producerCfg.name, prodHandle, logger0);

                /* Test Verbosity functions */
                traceObjectHandle = Test_setupTraceVerbosity();

                /* Start thread to generate logging traffic */
                errCode = pthread_create(&armLoggingThread, NULL, Dat_armLoggingTask, NULL);
                if(errCode < 0)
                {
                     printf ("Error: Unable to create arm logging thread %d\n", errCode);
                     return ;
                }
            }

            /* Create ARM GP producer */
            {
                Dat_ProducerCfg             producerCfg;
                pthread_t                   armGPLoggingThread;
                uint16_t                    crcApp16;
                char appName[] =            "test_dat_k2h";

                /* Initialize and create the DAT producer */
                memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

                /* Generate crc16 for loggerStreamer */
                crcApp16 = LoggerStreamer2_generateCRC((unsigned char const *)appName, (int)strlen(appName));

                /* Populate the producer configuration. */
                strcpy(producerCfg.name, "Test-GP-ARM");
                producerCfg.heapHandle           = datGPProducerHeap;
                producerCfg.debugSocketHandle    = datProducerSockHandle;
                producerCfg.loggerStreamerHandle = NULL;
                producerCfg.bufferSize           = 1000;
                producerCfg.crcApp16             = crcApp16;
                producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
                producerCfg.isMainLogger         = 0;

                /* Create the producer */
                prodGPHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
                if (prodGPHandle == NULL)
                {
                    printf ("Error: Unable to create the General purpose producer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: GP Producer '%s' Handle %p  [Streaming to System Analyzer] created successfully\n",
                                   producerCfg.name, prodGPHandle);

                /* Start thread to generate logging traffic */
                errCode = pthread_create(&armGPLoggingThread, NULL, Dat_armGPLoggingTask, prodGPHandle);
                if(errCode < 0)
                {
                     printf ("Error: Unable to create arm GP logging thread %d\n", errCode);
                     return ;
                }

                /* Test get GP data offset on ARM GP producer */
                /* Populate the connect information. */
                sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
                sockAddress.sin_port                    = 40000;
                sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

                /* Connect the socket */
                if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
                {
                    printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
                    return;
                }
                printf ("Debug: DAT consumer socket has been connected successfully\n");

                /* Initialize the consumer configuration */
                memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

                /* Populate the consumer configuration. */
                strcpy(consumerCfg.producerName, "Test-GP-ARM");
                consumerCfg.heapHandle          = datConsumerHeap;

                /* Create the consumer */
                consumerHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
                if (consumerHandle == NULL)
                {
                    printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: Consumer Handle %p [Streaming Data to Port %d] created successfully\n",
                        consumerHandle, sockAddress.sin_port);

                /* Consumer has not been connected; get the consumer status to ensure this is the case */
                if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                {
                    printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                    return;
                }
                if (consumerStatus != Dat_ConsumerStatus_DISCONNECTED)
                {
                    printf ("Error: Incorrect consumer status %d has been reported \n", consumerStatus);
                    return;
                }

                /* Try and connect the consumer to the producer  */
                while (1)
                {
                    /* Connect the consumer & producer */
                    if (Dat_connectConsumer (consumerHandle, &errCode) == 0)
                    {
                        printf ("Debug: Producer & consumer successfully connected\n");
                        break;
                    }

                    /* Error: Connect between the consumer & producer failed. Use the error code to determine
                     * the reason for failure. */
                    if (errCode == DAT_ENOTREADY)
                    {
                        /* Producer was not operational. */
                        usleep(10);
                        continue;
                    }
                    printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
                    return;
                }
                /* Consumer & producer have been connected; ensure that the consumer is connected before we proceed */
                while (1)
                {
                    if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                    {
                        printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                        return;
                    }
                    if (consumerStatus == Dat_ConsumerStatus_CONNECTED)
                        break;
                    usleep(10);
                }

                /* Get GP producer data buffer offset */
                {
                    uint32_t     dataOffset = 0;

                    dataOffset = Dat_getGPProducerDataOffset(consumerHandle, &errCode);
                    if(dataOffset == 0)
                    {
                        printf("Getting ARM GP  producer data offset failed with error [%x]\n", errCode);
                    }
                    else
                        printf("ARM GP producer data offset = %d\n", dataOffset);
                }
            }

            break;

        case 5:
            /* Setup memory logging for core0 UIA producer */
            errCode = pthread_create (&memLoggingThread[0], NULL, Test_memLoggingThread, "MemlogCore0_UIAProducer");
            if (errCode < 0)
            {
                printf ("Error: Memory logging thread for Core0_UIAProducer failed to start error code %d \n", errCode);
                return;
            }

            /* Setup memory logging for core1 UIA producer */
            errCode = pthread_create (&memLoggingThread[1], NULL, Test_memLoggingThread, "MemlogCore1_UIAProducer");
            if (errCode < 0)
            {
                printf ("Error: Memory logging for Core1_UIAProducer thread failed to start error code %d \n", errCode);
                return;
            }

            /* Setup memory logging for core0 UIA producer */
            errCode = pthread_create (&memLoggingThread[2], NULL, Test_memLoggingThread, "MemlogCore0_GPProducer");
            if (errCode < 0)
            {
                printf ("Error: Memory logging for Core0_GPProducer thread failed to start error code %d \n", errCode);
                return;
            }

            /* Setup memory logging for core1 UIA producer */
            errCode = pthread_create (&memLoggingThread[3], NULL, Test_memLoggingThread, "MemlogCore1_GPProducer");
            if (errCode < 0)
            {
                printf ("Error: Memory logging for Core1_GPProducer thread failed to start error code %d \n", errCode);
                return;
            }
            break;

        case 3:
            {
                /* Populate the connect information. */
                sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
                sockAddress.sin_port                    = 10000;
                sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

                /* Connect the socket */
                if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
                {
                    printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
                    return;
                }
                printf ("Debug: DAT consumer socket has been connected successfully\n");

                /* Initialize the consumer configuration */
                memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

                /* Populate the consumer configuration. */
                strcpy(consumerCfg.producerName, gProducerName);
                consumerCfg.heapHandle          = datConsumerHeap;

                /* Create the consumer */
                consumerHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
                if (consumerHandle == NULL)
                {
                    printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: Consumer Handle %p [Streaming Data to Port %d] created successfully\n",
                        consumerHandle, sockAddress.sin_port);

                /* Consumer has not been connected; get the consumer status to ensure this is the case */
                if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                {
                    printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                    return;
                }
                if (consumerStatus != Dat_ConsumerStatus_DISCONNECTED)
                {
                    printf ("Error: Incorrect consumer status %d has been reported \n", consumerStatus);
                    return;
                }

                /* Try and connect the consumer to the producer  */
                while (1)
                {
                    /* Connect the consumer & producer */
                    if (Dat_connectConsumer (consumerHandle, &errCode) == 0)
                    {
                        printf ("Debug: Producer & consumer successfully connected\n");
                        break;
                    }

                    /* Error: Connect between the consumer & producer failed. Use the error code to determine
                     * the reason for failure. */
                    if (errCode == DAT_ENOTREADY)
                    {
                        /* Producer was not operational. */
                        usleep(10);
                        continue;
                    }
                    printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
                    return;
                }
                /* Consumer & producer have been connected; ensure that the consumer is connected before we proceed */
                while (1)
                {
                    if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                    {
                        printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                        return;
                    }
                    if (consumerStatus == Dat_ConsumerStatus_CONNECTED)
                        break;
                    usleep(10);
                }

                /* Create a thread to handle consumer */
                errCode = pthread_create (&consumerThread, NULL, Test_consumerTask, consumerHandle);
                if (errCode < 0)
                {
                    printf ("Error: PM consumer Thread failed to start error code %d \n", errCode);
                    return;
                }
            }
            break;

        case 4:
            {

                uint32_t        dataOffset;
                /************************************************************
                 *   Create consumer for PM Producer
                 ************************************************************/
                /* Initialize the consumer configuration */
                memset ((void*)&consumerCfg, 0, sizeof(Dat_ConsumerCfg));

                /* Populate the consumer configuration. */
                strcpy(consumerCfg.producerName, "PM_Producer");
                consumerCfg.heapHandle          = datPMConsumerHeap;

                /* Create the consumer */
                consumerHandle = Dat_createConsumer (datClientHandle, &consumerCfg, &errCode);
                if (consumerHandle == NULL)
                {
                    printf ("Error: Unable to create the consumer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: Consumer Handle %p for PM produceris created successfully\n",
                        consumerHandle);

                /* Consumer has not been connected; get the consumer status to ensure this is the case */
                if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                {
                    printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                    return;
                }
                if (consumerStatus != Dat_ConsumerStatus_DISCONNECTED)
                {
                    printf ("Error: Incorrect consumer status %d has been reported \n", consumerStatus);
                    return;
                }

                /* Try and connect the consumer to the producer  */
                while (1)
                {
                    /* Connect the consumer & producer */
                    if (Dat_connectConsumer (consumerHandle, &errCode) == 0)
                    {
                        printf ("Debug: PM Producer & consumer successfully connected\n");
                        break;
                    }

                    /* Error: Connect between the consumer & producer failed. Use the error code to determine
                     * the reason for failure. */
                    if (errCode == DAT_ENOTREADY)
                    {
                        /* Producer was not operational. */
                        usleep(1000);
                        continue;
                    }
                    printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
                    return;
                }

                /* Consumer & producer have been connected; ensure that the consumer is connected before we proceed */
                while (1)
                {
                    if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
                    {
                        printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                        return;
                    }
                    if (consumerStatus == Dat_ConsumerStatus_CONNECTED)
                        break;
                    usleep(10);
                }
                printf ("Debug: ARM consumer for PM producer is now operational and is waiting for logs\n");

                dataOffset = Dat_getGPProducerDataOffset(consumerHandle, &errCode);
                if(dataOffset == 0)
                {
                    printf("Getting PM producer data offset failed with error [%x]\n", errCode);
                }
                else
                    printf("PM producer data offset = %d\n", dataOffset);

                /* Create a thread to handle PM consumer */
                errCode = pthread_create (&consumerThread, NULL, Test_PMConsumerTask, consumerHandle);
                if (errCode < 0)
                {
                    printf ("Error: PM consumer Thread failed to start error code %d \n", errCode);
                    return;
                }

                printf("Debug: PM consumer thread is created!\n");
            }

            break;

        case 6:
            /* Create ARM producer */
            {
                Dat_ProducerCfg             producerCfg;
                pthread_t                   armLoggingThread;
                uint16_t                    crcApp16;
                char                        appName[] = "test_dat_k2h";
                uint32_t                    memLoggingMemSize=0x100000;
                Memlog_ChannelCfg           memlogChanConfig;
                Memlog_ChHandle             memlogChanHandle;

                /*******************************************************************************
                 * TEST: Creating MEMLOG channel for ARM UIAProducer
                 *******************************************************************************/

                /* Initialize and create MEMLOG channel */
                memset((void *)&memlogChanConfig, 0, sizeof(Memlog_ChannelCfg) );

                /* Create memlog channel */
                strncpy ( &memlogChanConfig.name[0], "MemlogArm_UIAProducer", MEMLOG_MAX_CHAR - 1);
                memlogChanConfig.memlogInstHandle = memlogInstHandle;
                memlogChanConfig.memRegion        = memLoggingResourceCfg.memRegionResponse[0].memRegionHandle;
                memlogChanConfig.memBaseAddr      = (uint32_t)hplib_vmMemAlloc(memLoggingMemSize, 0, 0);
                memlogChanConfig.memSize          = memLoggingMemSize;
                memlogChanConfig.bufferSize       = 1408;
                memlogChanConfig.numPktDescs      = Qmss_getQueueEntryCount(Pktlib_getInternalHeapQueue(datProducerHeap));

                if ((memlogChanHandle = Memlog_create(&memlogChanConfig, &errCode)) == NULL)
                {
                    System_printf ("Error: MEMLOG create channel Failed [Error Code %d]\n", errCode);
                    return;
                }
                System_printf ("Debug: MEMLOG channel(%p) has been created successfully\n", memlogChanHandle );

                /* Push the channel name from the DSP to the ARM realm. */
                if (Name_push (nameClientHandle, "MemlogArm_UIAProducer", Name_ResourceBucket_INTERNAL_SYSLIB,
                               Name_ResourceOperationType_CREATE, &errCode) < 0)
                {
                    printf ("Error: Channel name '%s' PUSH to ARM realm failed [Error code %d] \n", "MemlogCore0_UIAProducer", errCode);
                    return ;
                }

                /* Initialize and create the DAT producer */
                memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

                /* Generate crc16 for loggerStreamer */
                crcApp16 = LoggerStreamer2_generateCRC((unsigned char const *)appName, (int)strlen(appName));

                /* Populate the producer configuration. */
                strcpy(producerCfg.name, "Test-UIA-ARM");
                producerCfg.heapHandle           = datProducerHeap;
                producerCfg.debugSocketHandle    = datProducerSockHandle;
                producerCfg.loggerStreamerHandle = NULL;
                producerCfg.bufferSize           = 1408;
                producerCfg.crcApp16             = crcApp16;
                producerCfg.producerType         = DAT_PRODUCER_UIA;
                producerCfg.isMainLogger         = 1;
                producerCfg.memlogChanHandle     = memlogChanHandle;

                printf("Debug: Memory logging configuration:\n");
                printf("Debug: Logging memory base: %x\n", memlogChanConfig.memBaseAddr);
                printf("Debug: Logging memory size: %x\n", memlogChanConfig.memSize);

                /* Create the producer */
                prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &logger0, &errCode);
                if (prodHandle == NULL || logger0 == NULL)
                {
                    printf ("Error: Unable to create the producer [Error code %d]\n", errCode);
                    return;
                }
                printf ("Debug: Producer '%s' Handle %p logger: %p [Streaming to System Analyzer] created successfully\n",
                         producerCfg.name, prodHandle, logger0);

                /* Start thread to generate logging traffic */
                errCode = pthread_create(&armLoggingThread, NULL, Dat_armLoggingforMemLoggingTask, NULL);
                if(errCode < 0)
                {
                     printf ("Error: Unable to create arm logging thread %d\n", errCode);
                     return ;
                }
            }

            /* Setup memory logging for ARM  producer */
            errCode = pthread_create (&memLoggingThread[4], NULL, Test_memLoggingThread, "MemlogArm_UIAProducer");
            if (errCode < 0)
            {
                printf ("Error: Memory logging thread for Test-UIA-ARM failed to start error code %d \n", errCode);
                return;
            }

            break;
        case 7:
        default:
            testComplete = 1;
            break;
    }

    /* Test wait loop until test is completed */
    while(testComplete == 0)
    {
        usleep(1000);
    }

    /* ======================================================
     *                Test cleanup
     * ======================================================*/

    /* Remove trace object */
    if(traceObjectHandle)
    {
        if (Test_deleteTraceObjects(traceObjectHandle) < 0)
        {
            printf ("Error: Removing Trace object failed\n");
            return;
        }
        printf("Debug: Trace object (%s) is deleted on client %p. \n", "L2-TRACE-OBJECT", datClientHandle);
    }

    if (consumerHandle)
    {
        /* Disconnect the consumer */
        if (Dat_disconnectConsumer (consumerHandle, &errCode) < 0)
        {
            printf ("Error: Disconnecting consumer failed [Error code %d]\n", errCode);
            return;
        }

        /* Ensure that the consumer has been disconnected before we delete it. */
        while (1)
        {
            if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
            {
                printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
                return;
            }
            if (consumerStatus == Dat_ConsumerStatus_DISCONNECTED)
                break;
        }

        /* Consumer has been disconnected and it can now be deleted. */
        if (Dat_deleteConsumer (consumerHandle, &errCode) < 0)
        {
            printf ("Error: Deleting consumer failed [Error code %d]\n", errCode);
            return;
        }
        printf("Debug: Consumer  is deleted on client %p. \n",  datClientHandle);
    }

    /* Delete ARM producer */
    if (prodHandle)
    {
        if (Dat_deleteProducer (prodHandle, &errCode) < 0 )
        {
            printf ("Error: Deleting producer failed [Error code %d]\n", errCode);
            return;
        }
        printf ("Debug: Deleting ARM producer succeeded\n");
    }

    if (prodGPHandle)
    {
        if (Dat_deleteProducer (prodGPHandle, &errCode) < 0 )
        {
            printf ("Error: Deleting General purpose producer failed [Error code %d]\n", errCode);
            return;
        }
        printf ("Debug: Deleting ARM Gerneral purpose producer succeeded\n");
    }

    /* Test is terminated, wait for all test threads are terminated */
    for(index=0; index < 5; index++)
    {
        if(memLoggingThread[index])
            pthread_join (memLoggingThread[index], NULL);
    }

    /* Close the socket handle used by the consumer */
    if (Netfp_closeSocket (datSockHandle, &errCode) < 0)
    {
        printf ("Error: Closing socket failed [Error code %d]\n", errCode);
        return;
    }

    printf("Test thread exits!\n");
    return;
}

