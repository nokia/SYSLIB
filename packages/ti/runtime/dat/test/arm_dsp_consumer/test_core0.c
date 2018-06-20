/**
 *   @file  test_core0.c
 *
 *   @brief
 *      Debug & Trace Unit Test Core0 Code.
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
#include <ti/sysbios/knl/Semaphore.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/name/name_db.h>

/* Logger streamer Include Files */
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

/* Test trace log Include Files */
#include "../trace_L2.h"
#include "../trace_log.h"
#include "../benchmark.h"
#include "../netCfg.h"


/**********************************************************************
 ************************** Unit Test Externs *************************
 **********************************************************************/

/* Global SYSLIB Module Instance Handles: */
extern Resmgr_ResourceCfg   appResourceConfig;
extern Pktlib_InstHandle    appPktlibInstanceHandle;
extern Msgcom_InstHandle    appMsgcomInstanceHandle;
extern Pktlib_HeapHandle    datUIAProducerHeap;
extern Pktlib_HeapHandle    datProducerHeap1;
extern Pktlib_HeapHandle    datProducerHeap2;
extern Netfp_ClientHandle   netfpClientHandle;
extern Name_ClientHandle    nameClientHandle;
extern Name_DBHandle        globalNameDatabaseHandle;

/* Heaps */
extern Pktlib_HeapHandle    datClientHeap;
extern Pktlib_HeapHandle    datConsumerHeap;

/* Cache Invalidate API: */
extern void appInvalidateBuffer(void* ptr, uint32_t size);

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
static const char*      gProducerName = "Test-UIA";

/* Statistics: Counter which keeps track of the number of messages received on the
 * consumer */
uint32_t                consumerMessage = 0;

/* Global varible to indicate test completion */
uint32_t gTestComplete = 0;

/* Buffer used to store NETFP configuration file */
char                    netfpConfigBuffer[2500];

/* NETFP Configuration after parsing the DAT file. */
Test_NetfpConfigInfo    netfpConfig;

/* Display the DAT Server */
uint32_t                gDebugDatServer = 0;

/* DAT Client Handle: */
Dat_ClientHandle        datClientHandle;

/* DAT Consumer Socket Handle */
Netfp_SockHandle        datSockHandle;

/* DAT general purpose producer debug socket handle */
Netfp_SockHandle        debugGPSockHandle;

/* DAT Server Name: */
const char*             gDatServerName   = "DatServer_LTE9A";

/* DAT General Purpose producer handles */
#define	                MAX_GP_PRODUCER    2
typedef struct GenPurposeProducer
{
    char            name[DAT_MAX_CHAR];
    uint32_t        bufSize;
    uint8_t*        buffer;
    Dat_ProdHandle  prodHandle;
    void*           logger;
}GenPurposeProducer;

GenPurposeProducer GPProducer[MAX_GP_PRODUCER];

/* Trace object body handle */
Dat_TraceObjectBody*    ptrTraceObjBody = NULL;

/* DAT Consumer */
Dat_ConsHandle      datConsumerHandle;

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
 *      This task to consumer buffers for all created consumers.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_consumerTask(UArg arg0, UArg arg1)
{
    Ti_Pkt*     ptrMessage;
    uint8_t*    ptrBuffer;
    uint32_t    bufferSize;
    int32_t     errCode;

    /* This task is created after consumer is created. */
    while(datConsumerHandle)
    {
        /* Poll all the messages for the consumer (non-pacing consumer) */
        do{
            ptrMessage = Dat_processConsumer(datConsumerHandle);
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

        /* Poll consumer every 2ms */
        Task_sleep(2);
    }
    return;
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

        if (Dat_filter (ptrTraceObjBody, TRACE_COMPONENT_L2_RLC1, DAT_COMP_LEVEL_PE0, TRACE_NON_FOCUSED))
        {
            /* Exchange the PM buffers. */
            GPProducer[0].buffer = Dat_bufferExchange (datClientHandle, GPProducer[0].prodHandle, GPProducer[0].buffer);
            GPProducer[1].buffer = Dat_bufferExchange (datClientHandle, GPProducer[1].prodHandle, GPProducer[1].buffer);

            /* Re-fill data for producer 2, since the buffer is cleared during buffer exchange */
            for(idx=0;idx < GPProducer[1].bufSize;idx++)
                GPProducer[1].buffer[idx] = idx + 1;
        }
    }

}
/**
 *  @b Description
 *  @n
 *      The function is used to parse the NETFP configuration file which
 *      is indicated by the file pointer.
 *
 *  @param[in]  configBuffer
 *      Data Buffer which has the NETFP configuration to be parsed
 *  @param[in]  configBufferSize
 *      Size of NETFP configuration file.
 *  @param[out] ptrNetfpConfig
 *      NETFP Configuration populated by this API.
 *
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Test_parseConfigFile(char* configBuffer, int32_t configBufferSize, Test_NetfpConfigInfo* ptrNetfpConfig)
{
    char    buffer[256];
    char*   tokenName, *ptrConfigBuffer, *tmp;
    char*   tokenValue;
    int32_t index, size = 0, totalSize = 0;
    const char*   delimitters = " =,;\n\r";
    const char*   newline = "\n";

    ptrConfigBuffer = configBuffer;
    tmp = ptrConfigBuffer;

    /* Parse the file contents till complete. */
    while (totalSize < configBufferSize)
    {
        /* Read one line at a time */
        tmp = strpbrk  (tmp, newline);

        if (tmp == NULL)
            return -1;

        /* Skip the newline character */
        tmp = tmp + 1;

        /* Get the size of the line read */
        size = (tmp - ptrConfigBuffer);

        totalSize += size;

        memcpy (buffer, ptrConfigBuffer, size);

        /* Store the last processed line */
        ptrConfigBuffer = tmp;

        /* Skip the comments and empty lines */
        if (buffer[0] == '#' || buffer[0] == '\n' || buffer[0] == '\r')
            continue;

        /* Get the Token Name */
        tokenName = strtok(buffer, delimitters);

        /* Get the Token Value */
        tokenValue = strtok(NULL, delimitters);

        /* Validate the token name with what is expected. */
        if (strcmp (tokenName, "ENODEB_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                ptrNetfpConfig->eNodeBMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "ENODEB_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                ptrNetfpConfig->eNodeBIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS0") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[0]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS1") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[1]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS2") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[2]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "ENODEB_IP6_ADDRESS3") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->eNodeBIPAddress6[3]) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "SGW0_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                ptrNetfpConfig->secGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "SGW0_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->secGwIPAddress6) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "SGW0_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                ptrNetfpConfig->secGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN_IP_ADDRESS") == 0)
        {
            /* Cycle through and get the actual IP address */
            for (index = 0; index < 4; index++)
            {
                ptrNetfpConfig->pdnGwIPAddress[index] = atoi (strtok (tokenValue, ".;"));
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "PDN_MAC_ADDRESS") == 0)
        {
            /* Cycle through and get the actual MAC address */
            for (index = 0; index < 6; index++)
            {
                char*   p1;
                char*   p2;

                /* Get the initial token value. */
                p1 = strtok (tokenValue, ".;");

                /* MAC Address can be HEX. So set the errno to be 0 and then
                 * call the conversion API. */
                ptrNetfpConfig->pdnGwMACAddress[index] = (uint8_t)strtol(p1, &p2, 16);
                if ((*p2 != 0) || (p2 == p1))
                    return -1;

                /* For subsequent iterations we wish to continue parsing from where we left off. */
                tokenValue = NULL;
            }
        }
        else if (strcmp (tokenName, "RNC_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGwIPAddress6) < 0)
                return -1;
        }
        else if (strcmp (tokenName, "PDN_IP6_ADDRESS") == 0)
        {
            if (Netfp_convertStrToIP6 (tokenValue, &ptrNetfpConfig->pdnGwIPAddress6) < 0)
                return -1;
        }
        else
        {
            /* Invalid Token Name. */
            //return -1;
        }
    }

    /* File has been successfully parsed. */
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
 *      The function is used to setup the trace verbosity.
 *
 *  @retval
 *      Handle to the trace object
 */
Dat_TraceObjHandle Test_setupTraceVerbosity(void)
{
    int32_t                  errCode;
    uint32_t                 index;
    Dat_TraceObjectCfg       traceObjectCfg;
    Dat_TraceComponentCfg    componentList[TRACE_NUM_COMPONENT_L2];
    uint32_t                 defaultNonFocusedMask = 0x3ff;
    uint32_t                 defaultFocusedMask    = 0x3ff;
    Dat_TraceObjHandle       traceObjectHandle = NULL;

    /**************************************************************************
     * Set up the default focused and non-focused verbosity levels for each
     * component. Here, we are assigning all components the same default
     * verbosity level.
     *************************************************************************/
    for(index = 0; index < TRACE_NUM_COMPONENT_L2; index++)
    {
        strcpy (componentList[index].componentName, l2ComponentList[index]);
        componentList[index].nonFocusedLevel  = defaultNonFocusedMask;
        componentList[index].focusedLevel     = defaultFocusedMask;
    }

    /**************************************************************************
     * Set up the L2 trace object properties including the default verbosity
     * levels for the class and components. Exceptions, errors and warnings
     * are enabled for the entire class, and hence, all the components.
     *************************************************************************/
    memset (&traceObjectCfg, 0, sizeof (Dat_TraceObjectCfg));
    traceObjectCfg.numTraceComponents = TRACE_NUM_COMPONENT_L2;
    traceObjectCfg.classLevel         = 0; /* SYS/BIOS and user-defined logging is disabled by default. */
    traceObjectCfg.commonCompLevel    = DAT_COMP_LEVEL_EXCEPTION | DAT_COMP_LEVEL_ERROR | DAT_COMP_LEVEL_WARNING | DAT_COMP_LEVEL_PE0;
    traceObjectCfg.ptrComponentArray  = componentList;
    strcpy (traceObjectCfg.traceObjectName, "L2-TRACE-OBJECT");

    /* Create the L2 global trace object. */
    if (Dat_createTraceObject (datClientHandle, &traceObjectCfg, &errCode) < 0)
    {
        System_printf ("Error: Creating global trace object for L2 failed. Error code %d\n", errCode);
        return NULL;
    }

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
            return NULL;
        }

        /* Created the local instance of the L2 trace object successfully. */
        System_printf ("Debug: Local instance of L2 trace object created.\n");
        break;
    }

    //Test_showTraceVerbosity(traceObjectHandle, "L2-TRACE-OBJECT");

    return traceObjectHandle;
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
        System_printf("Error: Removing local trace object(%p) failed with errCode = %d\n", traceObjectHandle, errCode);
    }
    System_printf("Debug: Local trace object (%p) is deleted. \n", traceObjectHandle);

    /* Remove global trace object */
    if (Dat_deleteTraceObject(datClientHandle, "L2-TRACE-OBJECT", &errCode) < 0 )
    {
        System_printf("Error: Removing global trace object(%s) failed with errCode = %d\n", "L2-TRACE-OBJECT", errCode);
    }

    System_printf("Debug: Global trace object (%s) is deleted. \n", "L2-TRACE-OBJECT");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Unit Test Task which generates logs and benchmarks the logging APIs.
 *
 *  @retval
 *      Not Applicable.
 */
static void Test_UIAloggingTask (UArg arg0, UArg arg1)
{
    uint32_t index;
    uint32_t filter;

    /* Declare the variables used for benchmarking. */
    BENCHMARK_INIT(dat_logwrite2);
    BENCHMARK_INIT(dat_logwrite3);
    BENCHMARK_INIT(dat_logwrite4);
    BENCHMARK_INIT(dat_filter);
    BENCHMARK_INIT(dat_logwrite2_noFilter);

    while(!gTestComplete)
    {
        Task_sleep(1);
        /**********************************************************************
         ******************* Benchmarking Dat_filter **************************
         **********************************************************************/

        BENCHMARK_START(dat_filter);
        filter = Dat_filter (ptrTraceObjBody, TRACE_COMPONENT_L2_RLC_UL, DAT_COMP_LEVEL_WARNING, TRACE_FOCUSED);
        BENCHMARK_END(dat_filter);

        /* Send the time taken for Trace_log2 for count analysis. */
        Trace_log4 (TRACE_COMPONENT_L2_DAT, DAT_COMP_LEVEL_USERDEF0, TRACE_NON_FOCUSED, UIAEvt_intWithKey,
               (IArg)BENCHMARK_VALUE(dat_filter), (IArg)0, (IArg)0, (IArg)"Dat_filter benchmark ");
        BENCHMARK_UPDATE(dat_filter);

        /**********************************************************************
         ******************* Benchmarking Trace_log2 **************************
         **********************************************************************/
        BENCHMARK_START(dat_logwrite2_noFilter);
        if (filter) Log_iwriteUC2 (logger0, UIAEvt_warningWithStr,(IArg)TRACE_DEFAULT_EVENT_CODE, (IArg)"L2: Overflow. Restarting next TTI.");
        BENCHMARK_END(dat_logwrite2_noFilter);

        /* Send the time taken for Trace_log2 for count analysis. */
        Trace_log4 (TRACE_COMPONENT_L2_DAT, DAT_COMP_LEVEL_USERDEF0, TRACE_NON_FOCUSED, UIAEvt_intWithKey,
               (IArg)BENCHMARK_VALUE(dat_logwrite2_noFilter), (IArg)0, (IArg)0, (IArg)"Log_iwriteUC2 benchmark NOT incl. filtering");
        BENCHMARK_UPDATE(dat_logwrite2_noFilter);

        /**********************************************************************
         ******************* Benchmarking Trace_log2 **************************
         **********************************************************************/
        BENCHMARK_START(dat_logwrite2);
        Trace_log2 (TRACE_COMPONENT_L2_RLC_UL, DAT_COMP_LEVEL_WARNING, TRACE_FOCUSED, UIAEvt_warningWithStr,
                (IArg)TRACE_DEFAULT_EVENT_CODE, (IArg)"L2: Overflow. Restarting next TTI..");
        BENCHMARK_END(dat_logwrite2);

         /* Send the time taken for Trace_log2 for count analysis. */
         Trace_log4 (TRACE_COMPONENT_L2_DAT, DAT_COMP_LEVEL_USERDEF0, TRACE_NON_FOCUSED, UIAEvt_intWithKey,
               (IArg)BENCHMARK_VALUE(dat_logwrite2), (IArg)0, (IArg)0, (IArg)"Log_iwriteUC2 benchmark incl. filtering");
         BENCHMARK_UPDATE(dat_logwrite2);


        /**********************************************************************
         ******************* Benchmarking Trace_log3 **************************
         **********************************************************************/
        BENCHMARK_START(dat_logwrite3);
        Trace_log3 (TRACE_COMPONENT_L2_RLC_DL, DAT_COMP_LEVEL_PE0, TRACE_FOCUSED, UIAEvt_detailWithStr,
                    (IArg)TRACE_DEFAULT_EVENT_CODE, (IArg)"L2_RLC_DL(): symbolNum = %d", (IArg)index);
        BENCHMARK_END(dat_logwrite3);
        BENCHMARK_UPDATE(dat_logwrite3);

         /* Send the time taken for Trace_log2 for count analysis. */
         Trace_log4 (TRACE_COMPONENT_L2_DAT, DAT_COMP_LEVEL_USERDEF0, TRACE_NON_FOCUSED, UIAEvt_intWithKey,
               (IArg)BENCHMARK_VALUE(dat_logwrite3), (IArg)0, (IArg)0, (IArg)"Log_iwriteUC3 benchmark incl. filtering");

        /**********************************************************************
         ******************* Benchmarking Trace_log4 **************************
         **********************************************************************/
        BENCHMARK_START(dat_logwrite4);
        Trace_log4 (TRACE_COMPONENT_L2_MAC_UL, DAT_COMP_LEVEL_PE0, TRACE_FOCUSED, UIAEvt_detailWithStr,
                    (IArg)TRACE_DEFAULT_EVENT_CODE, (IArg)"L2_MAC_UL(): symbolNum = %d, pilotSymbolNum = %d",
                    (IArg)index, (IArg)index * 2);
        BENCHMARK_END(dat_logwrite4);

        /* Send the time taken for Trace_log2 for count analysis. */
        Trace_log4 (TRACE_COMPONENT_L2_DAT, DAT_COMP_LEVEL_USERDEF0, TRACE_NON_FOCUSED, UIAEvt_intWithKey,
               (IArg)BENCHMARK_VALUE(dat_logwrite4), (IArg)0, (IArg)0, (IArg)"Log_iwriteUC4 benchmark incl. filtering");
        BENCHMARK_UPDATE(dat_logwrite4);

    }
    BENCHMARK_DISPLAY(dat_logwrite2);
    BENCHMARK_DISPLAY(dat_logwrite3);
    BENCHMARK_DISPLAY(dat_logwrite4);

}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the networking environment.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Test_netEnvironmentSetup(void)
{
    int32_t                 size;
    FILE*                   configFileHandle;
    int32_t                 errCode;
    Netfp_IfHandle          ifHandle;
    Netfp_InterfaceCfg      ifConfig;
    Netfp_InboundFPCfg      inboundFPCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPCfg     outboundFPCfg;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Test_NetfpConfigInfo*   ptrNetfpConfig;
    int32_t                 i;

    /* Open the NETFP configuration file for the tests */
    configFileHandle = fopen ("c:\\netfp.dat", "r");
    if (configFileHandle == NULL)
    {
        System_printf ("Error: Unable to open NETFP configuration file (c:\\netfp.dat)\n");
        return -1;
    }

    /* Initialize the size. */
    size = 0;

    /* Read data from the file and place it into the buffer */
    while (!feof(configFileHandle))
    {
        netfpConfigBuffer[size++] = fgetc(configFileHandle);
        if (size == sizeof (netfpConfigBuffer))
        {
            System_printf ("Error: Configuration file is too big\n");
            return -1;
        }
    }

    /* Remove the extra byte which was incremented */
    size = size - 1;

    /* Close the file. */
    fclose (configFileHandle);

    /* Parse the NETFP Configuration. */
    if (Test_parseConfigFile(netfpConfigBuffer, size, &netfpConfig) < 0)
    {
        /* Error: NETFP configuration file parsing failed. */
        System_printf ("Error: Unable to parse the NETFP configuration file.\n");
        return -1;
    }

    /* Create the interface: The interface is common to IPv4 or IPv6; so we create this
     * upfront. Initialize the configuration */
    memset ((void *)&ifConfig, 0, sizeof(Netfp_InterfaceCfg));

    /* Populate the configuration block. */
    ifConfig.type                          = Netfp_InterfaceType_ETH;
    ifConfig.mtu                           = 1500;
    ifConfig.ipAddress.ver                 = Netfp_IPVersion_IPV4;
    ifConfig.ipAddress.addr.ipv4.u.a8[0]   = netfpConfig.eNodeBIPAddress[0];
    ifConfig.ipAddress.addr.ipv4.u.a8[1]   = netfpConfig.eNodeBIPAddress[1];
    ifConfig.ipAddress.addr.ipv4.u.a8[2]   = netfpConfig.eNodeBIPAddress[2];
    ifConfig.ipAddress.addr.ipv4.u.a8[3]   = netfpConfig.eNodeBIPAddress[3];
    ifConfig.subnetMask.ver                = Netfp_IPVersion_IPV4;
    ifConfig.subnetMask.addr.ipv4.u.a8[0]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[1]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[2]  = 0xFF;
    ifConfig.subnetMask.addr.ipv4.u.a8[3]  = 0x0;
    strcpy (ifConfig.name, "eth0.100");

    /* Populate a MAC Address. */
    ifConfig.macAddress[0] = netfpConfig.eNodeBMACAddress[0];
    ifConfig.macAddress[1] = netfpConfig.eNodeBMACAddress[1];
    ifConfig.macAddress[2] = netfpConfig.eNodeBMACAddress[2];
    ifConfig.macAddress[3] = netfpConfig.eNodeBMACAddress[3];
    ifConfig.macAddress[4] = netfpConfig.eNodeBMACAddress[4];
    ifConfig.macAddress[5] = netfpConfig.eNodeBMACAddress[5];

    /* Create the interface */
    ifHandle = Netfp_createInterface (netfpClientHandle, &ifConfig, &errCode);
    if (ifHandle == NULL)
    {
        System_printf ("Error: Unable to create the interface [Error Code %d]\n", errCode);
        return -1;
    }

    /* Get the pointer to the NETFP Configuration. */
    ptrNetfpConfig = &netfpConfig;

    /* Display the IPv4 Setup Configuration: */
    System_printf ("-------------------------------------------------------------\n");
    System_printf ("Default IPv4 Test Environment Setup:\n");
    System_printf ("NETFP Client Handle     : %p\n", netfpClientHandle);
    System_printf ("My IP Address           : %d.%d.%d.%d\n",
                    ptrNetfpConfig->eNodeBIPAddress[0], ptrNetfpConfig->eNodeBIPAddress[1],
                    ptrNetfpConfig->eNodeBIPAddress[2], ptrNetfpConfig->eNodeBIPAddress[3]);
    System_printf ("Remote IP Address       : %d.%d.%d.%d via MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                    ptrNetfpConfig->pdnGwIPAddress[0], ptrNetfpConfig->pdnGwIPAddress[1],
                    ptrNetfpConfig->pdnGwIPAddress[2], ptrNetfpConfig->pdnGwIPAddress[3],
                    ptrNetfpConfig->pdnGwMACAddress[0], ptrNetfpConfig->pdnGwMACAddress[1],
                    ptrNetfpConfig->pdnGwMACAddress[2], ptrNetfpConfig->pdnGwMACAddress[3],
                    ptrNetfpConfig->pdnGwMACAddress[4], ptrNetfpConfig->pdnGwMACAddress[5]);
    System_printf ("-------------------------------------------------------------\n");

    /************************************************************************************
     * Ingress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    inboundFPCfg.spId                       = NETFP_INVALID_SPID;
    inboundFPCfg.spidMode                   = Netfp_SPIDMode_INVALID;
    inboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
    inboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];
    strcpy (inboundFPCfg.name, "Ingress-IPv4-FastPath");

    /* Add the Ingress Fast Path. */
    fpIngressHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to create Ingress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Ingress IPv4 Fast Path Handle 0x%p\n", fpIngressHandle);

    /************************************************************************************
     * Egress IPv4 Fast Path:
     ************************************************************************************/

    /* Initialize the fast path configuration. */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    outboundFPCfg.spId                       = NETFP_INVALID_SPID;
    outboundFPCfg.dstIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->pdnGwIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->pdnGwIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->pdnGwIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->pdnGwIPAddress[3];
    outboundFPCfg.srcIP.ver                  = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0]    = ptrNetfpConfig->eNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1]    = ptrNetfpConfig->eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2]    = ptrNetfpConfig->eNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3]    = ptrNetfpConfig->eNodeBIPAddress[3];

    /* Setup QCI-DSCP mapping for the egress fastpath */
    for (i = 0; i < 64; i ++)
        outboundFPCfg.dscpMapping[i] = i;
    strcpy (outboundFPCfg.name, "Egress-IPv4-FastPath");

    /* Setup the manual route: */
    outboundFPCfg.ifHandle             = ifHandle;
    outboundFPCfg.nextHopMACAddress[0] = ptrNetfpConfig->pdnGwMACAddress[0];
    outboundFPCfg.nextHopMACAddress[1] = ptrNetfpConfig->pdnGwMACAddress[1];
    outboundFPCfg.nextHopMACAddress[2] = ptrNetfpConfig->pdnGwMACAddress[2];
    outboundFPCfg.nextHopMACAddress[3] = ptrNetfpConfig->pdnGwMACAddress[3];
    outboundFPCfg.nextHopMACAddress[4] = ptrNetfpConfig->pdnGwMACAddress[4];
    outboundFPCfg.nextHopMACAddress[5] = ptrNetfpConfig->pdnGwMACAddress[5];

    /* Add the Egress Fast Path. */
    fpEgressHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to create Egress fast path Error %d\n", errCode);
        return -1;
    }
    System_printf ("Debug: Egress IPv4 Fast Path Handle 0x%p\n", fpEgressHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_core0Task(UArg arg0, UArg arg1)
{
    Dat_ConsHandle          consumerHandle;
    Dat_ConsumerCfg         consumerCfg;
    Netfp_InboundFPHandle   fpIngressHandle;
    Netfp_OutboundFPHandle  fpEgressHandle;
    Netfp_SockAddr          sockAddress;
    int32_t                 errCode;
    Dat_ClientCfg           datClientCfg;
    Task_Params             taskParams;
    Task_Handle             datTaskHandle;
    Name_ResourceCfg        namedResourceCfg;
    Dat_ConsumerStatus      consumerStatus;
    int32_t                 clientStatus;
    Dat_ProducerCfg         producerCfg;
    Dat_ProdHandle	        prodHandle;
    Dat_TraceObjHandle      traceObjectHandle = NULL;
    Dat_clientRuntimeCfg    datClientRTCfg;

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
    strcpy (datClientCfg.clientName, "Dat_Client_LTE9A_L2DP");
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
    datClientCfg.id                = DNUM;
    datClientCfg.realm             = Dat_ExecutionRealm_DSP;
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
    //datClientCfg.logSync.syncPeriod = 50;
    //datClientCfg.logSync.syncLogger = logger0;

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

    /* Initialize the structure */
    memset (&datClientRTCfg, 0, sizeof(datClientRTCfg));

    /* Setting up the logSync */
    datClientRTCfg.cfgType = DAT_CONF_LOGSYNC;
    datClientRTCfg.logSync.syncLogger = logger0;
    datClientRTCfg.logSync.syncPeriod = 50;

    if (Dat_configureClient(datClientHandle, &datClientRTCfg, &errCode) < 0)
    {
        System_printf ("Error: DAT Client configuration failed [Error code %d]\n", errCode);
        return;
    }

    /* Launch the DAT Client Execution Task:
     * - This is setup to be a lower priority task which executes in the background when there is no
     *   other task to execute */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 1;
    datTaskHandle = Task_create(Test_datClient, &taskParams, NULL);

    /* Setup the NETFP environment. */
    if (Test_netEnvironmentSetup() < 0)
    {
        System_printf ("Error: Unable to setup the NETFP environment\n");
        return;
    }

    /* Get the ingress fast path handle */
    fpIngressHandle = Netfp_findInboundFastPath (netfpClientHandle, "Ingress-IPv4-FastPath", &errCode);
    if (fpIngressHandle == NULL)
    {
        System_printf ("Error: Unable to find the ingress fast path [Error code %d]\n", errCode);
        return;
    }

    /* Get the egress fast path handle */
    fpEgressHandle = Netfp_findOutboundFastPath (netfpClientHandle, "Egress-IPv4-FastPath", &errCode);
    if (fpEgressHandle == NULL)
    {
        System_printf ("Error: Unable to find the egress fast path [Error code %d]\n", errCode);
        return;
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
    sockAddress.sin_port                = 2000;
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
    sockAddress.sin_port                    = 2000;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(datSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT consumer socket has been connected successfully\n");

    /***********************************************************************************
    * Socket which streams the debug info for general purpose producer
    ***********************************************************************************/

    /* Create a socket */
    debugGPSockHandle = Netfp_socket (netfpClientHandle, Netfp_SockFamily_AF_INET, &errCode);
    if (debugGPSockHandle == NULL)
    {
        System_printf ("Error: NETFP Socket Creation Failed [Error Code %d]\n", errCode);
        return;
    }

    /* Populate the binding information */
    memset ((void*)&sockAddress, 0, sizeof(Netfp_SockAddr));
    sockAddress.sin_family              = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                = 4000;
    sockAddress.op.bind.inboundFPHandle = fpIngressHandle;
    sockAddress.op.bind.flowId          = 0xFFFFFFFF;
    sockAddress.op.bind.queueHandle     = NULL;

    /* Bind the socket. */
    if (Netfp_bind (debugGPSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Bind Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT debug socket has been bound successfully\n");

    /* Populate the connect information. */
    sockAddress.sin_family                  = Netfp_SockFamily_AF_INET;
    sockAddress.sin_port                    = 1235;
    sockAddress.op.connect.outboundFPHandle = fpEgressHandle;

    /* Connect the socket */
    if (Netfp_connect(debugGPSockHandle, &sockAddress, &errCode) < 0)
    {
        System_printf ("Error: NETFP Connect Failed [Error Code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: DAT debug socket has been connected successfully\n");


    /*******************************************************************************
    * TEST: Setting & Querying Verbosity levels
    *******************************************************************************/
    traceObjectHandle = Test_setupTraceVerbosity();
    if( traceObjectHandle == NULL)
    {
        System_printf ("Error: DAT Trace verbosity setting failed. \n");
        return;
    }
    ptrTraceObjBody = Dat_getTraceObjectBody (traceObjectHandle);
    /*******************************************************************************
     * TEST: Creating UIA producer
     *******************************************************************************/
    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Test_Core0_UIAProducer");
    producerCfg.producerType 		 = DAT_PRODUCER_UIA;
    producerCfg.heapHandle           = datUIAProducerHeap;
    producerCfg.debugSocketHandle    = debugGPSockHandle;
    producerCfg.loggerStreamerHandle = logger0;

    /* Create the producer */
    prodHandle = Dat_createProducer (datClientHandle, &producerCfg, NULL, &errCode);
    if (prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the UIA producer [Error code %d]\n", errCode);
        return;
    }

    System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                    producerCfg.name, prodHandle);

    /*******************************************************************************
     * TEST: Creating general purpose producer
     *******************************************************************************/

    /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Test_GPProducer");
    producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
    producerCfg.bufferSize	         = 921;
    producerCfg.clearBuffer          = 0;
    producerCfg.heapHandle           = datProducerHeap1;
    producerCfg.debugSocketHandle    = debugGPSockHandle;
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


    /* Crate a logger streamer instance for the procuder */

        /* Initialize and create the DAT producer */
    memset ((void *)&producerCfg, 0, sizeof(Dat_ProducerCfg));

    /* Populate the producer configuration. */
    strcpy(producerCfg.name, "Test_GPProducer2");
    producerCfg.producerType         = DAT_PRODUCER_GENERAL_PURPOSE;
    producerCfg.bufferSize           = 1200;
    producerCfg.clearBuffer          = 1;
    producerCfg.heapHandle           = datProducerHeap2;
    producerCfg.debugSocketHandle    = debugGPSockHandle;
    producerCfg.loggerStreamerHandle = NULL;

    /* Create the producer */
    GPProducer[1].prodHandle = Dat_createProducer (datClientHandle, &producerCfg, &GPProducer[1].logger, &errCode);
    if (GPProducer[1].prodHandle == NULL)
    {
        System_printf ("Error: Unable to create the general purpose producer [Error code %d]\n", errCode);
        return;
    }
    GPProducer[1].bufSize = producerCfg.bufferSize;
    strcpy(GPProducer[1].name, producerCfg.name);

    System_printf ("Debug: Producer '%s' Handle %x [Streaming to System Analyzer] created successfully\n",
                    producerCfg.name, GPProducer[1].prodHandle);

    System_printf ("Debug: logger streamer instance is :%p\n", producerCfg.loggerStreamerHandle);

    /* Create a task to generate General Purpose producer logs */
    {
        Task_Params taskParams;

        Task_Params_init(&taskParams);
        taskParams.stackSize = 8*1024;
        taskParams.priority  = 2;
        Task_create(Test_GPProducerLoggingTask, &taskParams, NULL);
    }

    /*******************************************************************************
     * TEST: Connecting consumer
     *******************************************************************************/

    /* Initialize the consumer configuration */
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

    /* Consumer has not been connected; get the consumer status to ensure this is the case */
    if (Dat_getConsumerStatus (consumerHandle, &consumerStatus, &errCode) < 0)
    {
        System_printf ("Error: Unable to get the consumer status [Error code %d]\n", errCode);
        return;
    }
    if (consumerStatus != Dat_ConsumerStatus_DISCONNECTED)
    {
        System_printf ("Error: Incorrect consumer status %d has been reported \n", consumerStatus);
        return;
    }

    /* Try and connect the consumer to the producer  */
    while (1)
    {
        /* Connect the consumer & producer */
        if (Dat_connectConsumer (consumerHandle, &errCode) == 0)
        {
            System_printf ("Debug: Producer & consumer successfully connected\n");
            break;
        }

        /* Error: Connect between the consumer & producer failed. Use the error code to determine
         * the reason for failure. */
        if (errCode == DAT_ENOTREADY)
        {
            /* Producer was not operational. */
            Task_sleep(10);
            continue;
        }
        System_printf ("FATAL Error: DAT connect consumer failed [Error code %d]\n", errCode);
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
        Task_sleep(10);
    }

    /* Save consumer handle to be used in process Consumer */
    datConsumerHandle =  consumerHandle;

    /* Create consumer task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 1;
    Task_create(Test_consumerTask, &taskParams, NULL);

    System_printf ("Debug: DSP %d consumer is now operational and is waiting for logs\n", DNUM);

    /* Create logging Test Task: */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 4;
    Task_create(Test_UIAloggingTask, &taskParams, NULL);

    /* Loop around waiting for the test completion status */
    while (1)
    {
        if (Name_findResource (globalNameDatabaseHandle, Name_ResourceBucket_USER_DEF1,
                               "DAT_UNIT_TEST_STATUS", &namedResourceCfg, &errCode) == 0)
            break;
        Task_sleep(10);
    }

    /* Mark Test is completed, so that logs are not generated anymore. */
    gTestComplete = 1;
    System_printf ("Debug: DAT Log generation complete on Core %d\n", DNUM);

    if (Test_deleteTraceObjects(traceObjectHandle) < 0)
    {
        System_printf ("Error: Removing Trace object failed\n");
        return;
    }

    /* Disconnect the consumer */
    if (Dat_disconnectConsumer (consumerHandle, &errCode) < 0)
    {
        System_printf ("Error: Disconnecting consumer failed [Error code %d]\n", errCode);
        return;
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

    /* Delete the general purpose producer: Producers can only be deleted once all the consumers have been
     * disconnected and deleted. */
    while (1)
    {
        if (Dat_deleteProducer (GPProducer[1].prodHandle, &errCode) == 0)
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

