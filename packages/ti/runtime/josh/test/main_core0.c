/**
 *   @file  main_core0.c
 *
 *   @brief
 *      Unit Test for the JOSH which executes on the Core0.
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
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_cpIntcAux.h>

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

#ifndef DEVICE_K2
#include <ti/runtime/netfp/netfp.h>
#endif

/**********************************************************************
 ************************* Unit Test Definitions **********************
 **********************************************************************/

/* MAX size of the JOSH Message exchanged between the DSP Cores i.e. the
 * processing nodes in the Test case. */
#define TEST_MAX_JOSH_MSG_SIZE      1024

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Global JOSH Node Handle. */
Josh_NodeHandle         gJoshNodeHandle;

/* Global JOSH Heap used for sending JOSH Packets between the processing
 * nodes. */
Pktlib_HeapHandle       joshHeap;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global system named resource instance handle. */
Name_DBHandle           globalNameDatabaseHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Application requested resources */
Resmgr_ResourceCfg    appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
	2,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
    1,    /* Number of QPEND Queues requested                                 */
	/* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                     Linking RAM,                            Num,   Size */
		{ "Core0-DDR3",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,   256,   128},
		{ "Core0-Local", Resmgr_MemRegionType_LOCAL, Resmgr_MemRegionLinkingRAM_DONT_CARE,   64,    128},
    }
};

/**********************************************************************
 *********************** Unit Test Extern Definitions *****************
 **********************************************************************/

/* Application Cache Management API */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* OSAL callout */
extern uint32_t l2_global_address (uint32_t addr);

/*****************************************************************************
 * OSAL Callout Functions:
 *****************************************************************************/

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

/* MSGCOM */
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

/* JOSH */
extern void* Josh_osalMalloc(uint32_t, uint32_t);
extern void  Josh_osalFree (void* , uint32_t );
extern void* Josh_osalEnterSingleCoreCS(void);
extern void  Josh_osalExitSingleCoreCS(void* );
extern void* Josh_osalCreateSem(void);
extern void  Josh_osalDeleteSem(void* );
extern void  Josh_osalPendSem(void* );
extern void  Josh_osalPostSem(void* );

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
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Code.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t system_init (void)
{
    int32_t                     errCode;
    Resmgr_SystemCfg            sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;

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
    sysConfig.dspSystemCfg.sharedMemAddress 	= DDR3_SYSLIB_RESMGR_RSVD;
    sysConfig.dspSystemCfg.sizeSharedMemory 	= DDR3_SYSLIB_RESMGR_RSVD_LEN;

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

    /* Create the global database */
    globalNameDatabaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (globalNameDatabaseHandle == NULL)
	{
	    System_printf ("Error: Name database creation failed [Error code %d]\n", errCode);
	    return -1;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

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

    /* System has been initialized successfully */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Dummy STUB Job1 Function for the MASTER. The actual JOB will reside on the
 *      SLAVE
 */
uint32_t job1 (void)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Dummy STUB Square Number Function for the MASTER. The actual JOB will reside on the
 *      SLAVE
 */
uint32_t squareNumber (uint32_t seed)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Dummy STUB SUM Function for the MASTER. The actual JOB will reside on the
 *      SLAVE
 */
static uint32_t sum (uint32_t number1, uint32_t number2, uint32_t number3, uint32_t number4)
{
    uint32_t sum;

    /* Sum of all the numbers but first convert the numbers to host format */
    sum = josh_toNativeU32(number1) + josh_toNativeU32(number2) + josh_toNativeU32(number3) + josh_toNativeU32(number4);
    return sum;
}


/**
 *  @b Description
 *  @n
 *      Dummy STUB sumArray Function for the MASTER. The actual JOB
 *      will reside on the SLAVE
 */
uint32_t sumArray (uint32_t* array, uint32_t numElements)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Dummy STUB arrayMultiply Function for the MASTER. The actual JOB
 *      will reside on the SLAVE
 */
uint32_t arrayMultiply (uint32_t* inArray, int32_t numElements, uint32_t* outArray)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Dummy STUB stringPlay Function for the MASTER. The actual JOB
 *      will reside on the SLAVE
 */
uint32_t stringPlay (uint8_t* str1)
{
    return 0;
}

#ifndef DEVICE_K2

/**
 *  @b Description
 *  @n
 *      Dummy STUB jobStructProcessing Function for the MASTER. The actual JOB
 *      will reside on the SLAVE
 */
uint32_t jobStructProcessing(Netfp_FastPathCfg* ptrFPConfig)
{
    return 0;
}
#endif

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to allocate a message which is to be sent across
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be allocated
 *  @param[out]  msgBuffer
 *      Opaque handle to the message to be sent out
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Test_joshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Ti_Pkt*     ptrPkt;
    uint8_t*    ptrDataBuffer;
    uint32_t    dataBufferLen;

    /* Allocate a packet from the Data Rx Heap */
    ptrPkt = Pktlib_allocPacket(appPktlibInstanceHandle, joshHeap, size);
    if (ptrPkt == NULL)
        return NULL;

    /* Get the data buffer. */
    Pktlib_getDataBuffer(ptrPkt, &ptrDataBuffer, &dataBufferLen);

    /* Populate the memory handle with the pointer to the PKTLIB Packet. */
    *msgBuffer = (MsgCom_Buffer*)ptrPkt;

    /* Return the data buffer. */
    return ptrDataBuffer;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to free a previously allocated message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be freed up
 *  @param[in]  msgBuffer
 *      Opaque handle to the message to be cleaned up
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static void Test_joshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Ti_Pkt*     ptrPkt;

    /* Get the packet buffer. */
    ptrPkt = (Ti_Pkt*)msgBuffer;
    if (ptrPkt == NULL)
    {
        System_printf ("Error: JOSH Free Packet got a NULL message to cleanup\n");
        return;
    }

    /* Cleanup the packet. */
    Pktlib_freePacket(appPktlibInstanceHandle, ptrPkt);
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to receive a message.
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   readerChannel
 *      Opaque reader channel on which the message is to be received
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has been received
 *  @param[in]  ptrDataBuffer
 *      Data buffer which points to the received message
 *
 *  @retval
 *      Not applicable
 */
static void Test_joshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    uint32_t    dataBufferLen;

    /* Read the message from the specified reader channel. */
    Msgcom_getMessage((MsgCom_ChHandle)readerChannel, (MsgCom_Buffer**)msgBuffer);
    if (*msgBuffer == NULL)
    {
        /* No message was available. */
        *ptrDataBuffer = NULL;
        return;
    }

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)(*msgBuffer), ptrDataBuffer, &dataBufferLen);

    /* Invalidate the data buffer. */
    appInvalidateBuffer (*ptrDataBuffer, dataBufferLen);
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to send a message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   writerChannel
 *      Opaque writer channel on which the message is to be transmitted
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has to be sent
 *
 *  @retval
 *      Not applicable
 */
static void Test_joshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    uint32_t    dataBufferLen;
    uint8_t*    ptrDataBuffer;

    /* Get the packet and extract the data buffer from the packet */
    Pktlib_getDataBuffer((Ti_Pkt*)msgBuffer, &ptrDataBuffer, &dataBufferLen);

    /* Writeback the data buffer. */
    appWritebackBuffer (ptrDataBuffer, dataBufferLen);

    /* Send the message */
    Msgcom_putMessage((MsgCom_ChHandle)writerChannel, (MsgCom_Buffer*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      JOSH Receive Task
 *
 *  @retval
 *      Not Applicable.
 */
static void JoshReceiveTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    /* Since the receive channel associated with the JOSH Node is a Non-blocking receive
     * channel we process all the received packets */
    while (1)
    {
        /* Try and process a JOSH message: */
        if (Josh_receive(gJoshNodeHandle, &errCode) < 0)
        {
            /* Error: Unable to handle it. Have we processed all the messages */
            if (errCode == JOSH_ENOMSG)
            {
                Task_sleep(2);
                continue;
            }

            /* This is a FATAL Error. */
            System_printf ("Error: JOSH Receive Failed with Error %d\n", errCode);
            return;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to test asynchronous job execution
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -  <0
 */
static int32_t Test_asyncJobs(void)
{
    Josh_ArgHandle          argHandle;
    Josh_JobHandle          jobHandle;
    Josh_Argument           args[JOSH_MAX_ARGS];
    int32_t                 errCode;
    uint32_t*               inputArray;
    uint32_t*               outputArray;
    int32_t                 retVal;
    uint32_t                expectedResult[16];
    int32_t                 jobId[16];
    uint32_t                index;
    uint32_t*               number;

    /* Initialize the expected result and job identifiers */
    memset ((void *)&expectedResult, 0xFF, sizeof(expectedResult));
    memset ((void *)&jobId, 0xFF, sizeof(jobId));

    /* ------------------------------------------------------------------------------
     * Job with two arguments passed by reference. One has actual data and the results
     * are populated in the second argument and passed back.
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)arrayMultiply);

    /* Cycle through and submit the various asynchronous jobs */
    for (index = 0; index < 16; index++)
    {
        /* Initialize the arguments. */
        memset ((void *)&args, 0, sizeof(args));

        /* Populate the arguments.
         *  - For the arrray increment JOB the first argument is an array of uint32_t
         *    In the example here we are setting this array to have 2 elements
         *  - The second argument is the number of elements in the input array.
         *  - The third argument is an array where the result is stored which
         *    also has the same size. */
        args[0].type   = Josh_ArgumentType_PASS_BY_REF;
        args[0].length = sizeof(uint32_t) * 2;
        args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
        args[1].length = 4;
        args[2].type   = Josh_ArgumentType_PASS_BY_REF;
		args[2].length = sizeof(uint32_t) * 2;

		/* Add the arguments. */
		argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
		if (argHandle == NULL)
		{
			System_printf ("Error: Unable to add arguments for job %d\n", index);
			return -1;
		}

		/* Get the pointer to the input, output arrays & number of elements in the array. */
		inputArray  = (uint32_t*)args[0].argBuffer;
		number      = (uint32_t*)args[1].argBuffer;
		outputArray = (uint32_t*)args[2].argBuffer;

		/* Populate the input array. */
		inputArray[0] = josh_toRemoteU32(10);
		inputArray[1] = josh_toRemoteU32(20);

		/* There are 2 elements in the array. */
		*number = josh_toRemoteU32(2);

		/* Reset the output array. */
		outputArray[0] = 0;
		outputArray[1] = 0;

		/* Submit the JOB to JOSH for execution. */
		jobId[index] = Josh_submitAsyncJob(jobHandle, argHandle, &errCode);
		if (jobId[index] < 0)
		{
			System_printf ("Error: JOSH Job submit async job %d failed [Error code %d]\n", index, errCode);
			return -1;
		}
    }
    System_printf ("Debug: %d asynchronous jobs have been submitted\n", index);

    /* Relinquish time and allow the JOSH task to execute. */
    Task_sleep(10);
    System_printf ("Debug: Checking if %d asynchronous jobs have been executed\n", index);

    /* Cycle through and ensure that if the jobs have completed execution: */
    while (1)
    {
        /* Cycle through all the jobs and check if they have been completed successfully. */
        for (index = 0; index < 16; index++)
        {
            /* Have we already processed the job? */
            if (expectedResult[index] != 0x0)
            {
                /* NO. Did the asynchronous job complete its execution? */
                retVal = Josh_isJobCompleted (gJoshNodeHandle, jobId[index], &expectedResult[index], &errCode);
                if (retVal < 0)
                {
                    System_printf ("Error: JOB Id %d job completion check failed [Error code %d]\n", errCode);
                    return -1;
                }
            }
        }

        /* Have all the jobs been serviced? This can be done by checking the expected results. If
         * the expected results are non-zero then job has not been processed. If the expected result
         * is 0 then the job is done. */
        for (index = 0; index < 16; index++)
        {
            if (expectedResult[index] != 0x0)
                break;
        }

        /* If we have processed all the jobs; we can get out of the loop. */
        if (index == 16)
            break;
    }
    System_printf ("Debug: %d asynchronous jobs have been processed\n", index);

    /* Cycle through and get the results of all the jobs */
    for (index = 0; index < 16; index++)
    {
        /* Get the result arguments. */
        if (Josh_getArguments (gJoshNodeHandle, jobId[index], &args[0]) < 0)
        {
            System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
            return -1;
        }

        /* Initialize the output parameters from the received arguments. */
        inputArray  = (uint32_t*)args[0].argBuffer;
        number      = (uint32_t*)args[1].argBuffer;
        outputArray = (uint32_t*)args[2].argBuffer;

        /* Verify each output parameter */
        if (outputArray[0] != (inputArray[0] * 2))
        {
            System_printf ("Error: Job %d result verification for 0 failed [Expected %d Got %d]\n",
                            jobId[index], (inputArray[0] * 2), outputArray[0]);
            return -1;
        }

        /* Verify each output parameter */
        if (outputArray[1] != (inputArray[1] * 2))
        {
            System_printf ("Error: Job %d result verification for 1 failed [Expected %d Got %d]\n",
                            jobId[index], (inputArray[1] * 2), outputArray[1]);
            return -1;
        }

        /* Free the JOB Instance. */
        if (Josh_freeJobInstance(gJoshNodeHandle, jobId[index]) < 0)
        {
            System_printf ("Error: Freeing JOB Instance failed\n");
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      JOSH Task
 *
 *  @retval
 *      Not Applicable.
 */
void JoshTask(UArg arg0, UArg arg1)
{
    Josh_ArgHandle          argHandle;
    Josh_JobHandle          jobHandle;
    Josh_Argument           args[JOSH_MAX_ARGS];
    int32_t                 errCode;
    uint32_t                result;
    uint32_t*               inputArray;
    uint32_t*               outputArray;
    uint32_t*               number;
    uint32_t                squareInputNumber;
    uint32_t                expectedResult;
    int32_t                 jobId;
    uint32_t                inputNumber[4];
    Pktlib_HeapStats        startHeapStats;
    Pktlib_HeapStats        endHeapStats;

    /* Get the heap statistics at the start of the test */
    Pktlib_getHeapStats (joshHeap, &startHeapStats);

    /* Register the jobs in the MASTER: These are registered with the DUMMY Stubs. */
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)job1);
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)squareNumber);
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)sum);
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)sumArray);
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)arrayMultiply);
    Josh_registerJob(gJoshNodeHandle, (Josh_JobProtype)stringPlay);

    /* Setup the error code. */
    errCode = -1;

    /* Sanity Check: */
    if (Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)job1) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)squareNumber) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)sum) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)sumArray) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)arrayMultiply) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else
        errCode = 0;

    /* Did we get an error? */
    if (errCode == -1)
        return;

    /* No error proceed with testing. */
    System_printf ("Debug: JOSH Job Registration & Lookup was successful.\n");

    /* Test the asynhronous jobs. */
    if (Test_asyncJobs () < 0)
    {
        System_printf ("Error: Asynchronous Job execution failed\n");
        return;
    }

    /*************************************************************************************
     ************************************* JOB TESTING ***********************************
     *************************************************************************************/

    /* ------------------------------------------------------------------------------
     * Test1: Job with no arguments. The goal behind the test is to ensure that the
     * function gets invoked on the SLAVE and the return value is correctly received
     * on the MASTER
     * ------------------------------------------------------------------------------ */
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)job1);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* No arguments associated with the JOB. */
    args[0].type = Josh_ArgumentType_INVALID;
    args[1].type = Josh_ArgumentType_INVALID;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments for Test1\n");
        return;
    }

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Validate the return value */
    if (result != 10)
    {
        System_printf ("Error: JOSH Job result does NOT match (Expected 10 got %d)\n", result);
        return;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test1 Passed (Result %d).\n", result);

    /* ------------------------------------------------------------------------------
     * Test2: Job with a single arguments passed by value. The goal behind the test
     * is to ensure that the function gets invoked with the correct argument on
     * the SLAVE and the return value is correctly received on the MASTER
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)squareNumber);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* Populate the arguments. */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = 4;
    args[1].type   = Josh_ArgumentType_INVALID;
    args[1].length = 0;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments\n");
        return;
    }

    /* Argument being passed to the JOB2 has a value of 5 */
    squareInputNumber = 5;
    number = (uint32_t*)args[0].argBuffer;
    *number = josh_toRemoteU32(squareInputNumber);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (gJoshNodeHandle, jobId, &args[0]) < 0)
    {
        System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
        return;
    }

    /* Validate the return value */
    if (result != (squareInputNumber * squareInputNumber))
    {
        System_printf ("Error: JOSH Job result does NOT match (Expected %d got %d)\n",
                       (squareInputNumber * squareInputNumber), result);
        return;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test2 Passed (Result %d).\n", result);

    /* ------------------------------------------------------------------------------
     * Test3: Job with 4 arguments passed by value. The goal behind the test
     * is to ensure that the function gets invoked with the correct arguments on
     * the SLAVE and the return value is correctly received on the MASTER
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)sum);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* Populate the arguments. */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = 4;
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = 4;
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = 4;
    args[3].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[3].length = 4;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments\n");
        return;
    }

    /* Populate the input numbers. */
    inputNumber[0] = 11;
    inputNumber[1] = 12;
    inputNumber[2] = 13;
    inputNumber[3] = 14;

    /* Compute the expected result. */
    expectedResult = inputNumber[0] + inputNumber[1] + inputNumber[2] + inputNumber[3];

    /* Populate the arguments. */
    number = (uint32_t*)args[0].argBuffer;
    *number = josh_toRemoteU32(inputNumber[0]);
    number = (uint32_t*)args[1].argBuffer;
    *number = josh_toRemoteU32(inputNumber[1]);
    number = (uint32_t*)args[2].argBuffer;
    *number = josh_toRemoteU32(inputNumber[2]);
    number = (uint32_t*)args[3].argBuffer;
    *number = josh_toRemoteU32(inputNumber[3]);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (gJoshNodeHandle, jobId, &args[0]) < 0)
    {
        System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
        return;
    }

    /* Validate the return value */
    if (result != expectedResult)
    {
        System_printf ("Error: JOSH Job result does NOT match (Expected %d got %d)\n",
                       expectedResult, result);
        return;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test3 Passed (Result %d).\n", result);

    /* ------------------------------------------------------------------------------
     * Test4: Job with a single argument passed by reference and one by value.
     * The goal behind the test is to ensure that the function gets invoked with
     * the correct pass by reference argument on the SLAVE and the return value
     * is correctly received on the MASTER
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)sumArray);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* Populate the arguments.
     *  - For the arrray increment JOB the first argument is an array of uint32_t
     *    In the example here we are setting this array to have 4 elements
     *  - The second argument is the number of elements in the array */
    args[0].type   = Josh_ArgumentType_PASS_BY_REF;
    args[0].length = sizeof(uint32_t) * 4;
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = 4;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments\n");
        return;
    }

    /* Setup the input array and populate it  */
    inputArray = (uint32_t*)args[0].argBuffer;
    inputArray[0] = josh_toRemoteU32(10);
    inputArray[1] = josh_toRemoteU32(20);
    inputArray[2] = josh_toRemoteU32(30);
    inputArray[3] = josh_toRemoteU32(40);

    /* Argument being passed is the number of elements in the array. */
    number = (uint32_t*)args[1].argBuffer;
    *number = josh_toRemoteU32(4);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (gJoshNodeHandle, jobId, &args[0]) < 0)
    {
        System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
        return;
    }

    /* Calculate the expected result. We need to reset the Input Array back to Host order for this. */
    expectedResult = (josh_toNativeU32(inputArray[0]) + josh_toNativeU32(inputArray[1]) +
                      josh_toNativeU32(inputArray[2]) + josh_toNativeU32(inputArray[3]));

    /* Validate the return value */
    if (result != expectedResult)
    {
        System_printf ("Error: JOSH Job result does NOT match (Expected %d got %d)\n", expectedResult, result);
        return;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test4 Passed (Result %d).\n", result);

    /* ------------------------------------------------------------------------------
     * Test5: Job with two arguments passed by reference. One has actual data and the
     * results are populated in the second argument and passed back.
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)arrayMultiply);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* Populate the arguments.
     *  - For the arrray increment JOB the first argument is an array of uint32_t
     *    In the example here we are setting this array to have 2 elements
     *  - The second argument is the number of elements in the input array.
     *  - The third argument is an array where the result is stored which
     *    also has the same size. */
    args[0].type   = Josh_ArgumentType_PASS_BY_REF;
    args[0].length = sizeof(uint32_t) * 2;
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = 4;
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(uint32_t) * 2;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments\n");
        return;
    }

    /* Get the pointer to the input, output arrays & number of elements in the array. */
    inputArray  = (uint32_t*)args[0].argBuffer;
    number      = (uint32_t*)args[1].argBuffer;
    outputArray = (uint32_t*)args[2].argBuffer;

    /* Populate the input array. */
    inputArray[0] = josh_toRemoteU32(10);
    inputArray[1] = josh_toRemoteU32(20);

    /* There are 2 elements in the array. */
    *number = josh_toRemoteU32(2);

    /* Reset the output array. */
    outputArray[0] = 0;
    outputArray[1] = 0;

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Validate the return value */
    if (result != 0)
    {
        System_printf ("Error: JOSH Job result does NOT match (%d)\n", result);
        return;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (gJoshNodeHandle, jobId, &args[0]) < 0)
    {
        System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
        return;
    }

    /* Initialize the output parameters from the received arguments. */
    outputArray = (uint32_t*)args[2].argBuffer;

    /* Validate if the calculation was correct or not? */
    for (result = 0; result < 2; result++)
    {
        /* Calculate the expected result. We need to convert it back to host order though. */
        expectedResult = josh_toNativeU32(inputArray[result]) * 2;

        /* Match it but first convert the output to host order. */
        if (josh_toRemoteU32(outputArray[result]) != expectedResult)
        {
            /* No Match. */
            System_printf ("Error: Result is not correct Expected %d Got %d\n",
                            expectedResult, josh_toRemoteU32(outputArray[result]));
            return;
        }
        System_printf ("Debug: Output[%d]=%d Input[%d]=%d\n", result, josh_toRemoteU32(outputArray[result]),
                        result, inputArray[result]);
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test5 Passed\n");

    /* ------------------------------------------------------------------------------
     * Test6: Job with single arguments passed by reference. The data is manipulated
     * and passed back.
     * ------------------------------------------------------------------------------*/
    jobHandle = Josh_findJobByAddress(gJoshNodeHandle, (Josh_JobProtype)stringPlay);

    /* Initialize the arguments. */
    memset ((void *)&args, 0, sizeof(args));

    /* Populate the arguments.
     *  - Argument 1 is the data string passed to the function. */
    args[0].type   = Josh_ArgumentType_PASS_BY_REF;
    args[0].length = strlen ("Hello from Master") + 1;

    /* Add the arguments. */
    argHandle = Josh_addArguments (gJoshNodeHandle, &args[0]);
    if (argHandle == NULL)
    {
        System_printf ("Error: Unable to add arguments\n");
        return;
    }

    /* Populate Argument1 */
    strcpy((char*)args[0].argBuffer, "Hello from Master");

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, &errCode);
    if (jobId < 0)
    {
        System_printf ("Error: JOSH Job submit job failed with error code %d\n", errCode);
        return;
    }

    /* Get the result arguments. */
    if (Josh_getArguments (gJoshNodeHandle, jobId, &args[0]) < 0)
    {
        System_printf ("Error: JOSH get arguments failed with error code %d\n", errCode);
        return;
    }

    /* Validate the return value */
    if (result != 100)
    {
        System_printf ("Error: JOSH Job result does NOT match (%d)\n", result);
        return;
    }
    System_printf ("Debug: Return from Slave core is '%s'\n", (char*)args[0].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(gJoshNodeHandle, jobId) < 0)
    {
        System_printf ("Error: Freeing JOB Instance failed\n");
        return;
    }
    System_printf ("Debug: Test6 Passed.\n");

    /* Get the heap statistics at the end of the test */
    Pktlib_getHeapStats (joshHeap, &endHeapStats);

    /* Validation: Ensure that there are no memory leaks in the JOSH framework */
    if ((startHeapStats.numFreeDataPackets != endHeapStats.numFreeDataPackets)      ||
        (startHeapStats.numZeroBufferPackets != endHeapStats.numZeroBufferPackets)  ||
        (startHeapStats.numPacketsinGarbage != endHeapStats.numPacketsinGarbage))
    {
        System_printf ("Error: JOSH Framework tests failed because of memory leaks\n");
        return;
    }

    /* Control comes here implies that all the tests passed. */
    System_printf ("Debug: All Josh Tests Passed\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_sysInitTask(UArg arg0, UArg arg1)
{
    Task_Params             taskParams;
    Resmgr_ResourceCfg*     ptrCfg;
    Josh_NodeCfg            nodeCfg;
    MsgCom_ChHandle         joshWriterChannel;
    MsgCom_ChHandle         joshReaderChannel;
    Msgcom_ChannelCfg       chConfig;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 errCode;

    /* Initialize the CPPI and QMSS Modules. */
	if (system_init() < 0)
	    return;

    /* Get the resource manager configuration for the specified DSP Core. */
    ptrCfg = &appResourceConfig;

    /* Create the JOSH Heap which is used to manage the packets between the processing
     * entitities. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy(heapCfg.name, "JOSH-Heap");
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.memRegion                       = ptrCfg->memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = TEST_MAX_JOSH_MSG_SIZE;
    heapCfg.numPkts                         = 40;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create a JOSH Shared heap */
    joshHeap = Pktlib_createHeap(&heapCfg, &errCode);
    if (joshHeap == NULL)
    {
        System_printf ("Error: Unable to JOSH Heap error code %d\n", errCode);
        return;
    }

    /* Wait for the writer channel i.e. Core1 reader channel to get created */
    while (1)
    {
        joshWriterChannel = Msgcom_find (appMsgcomInstanceHandle, "Core1-Reader-Channel", &errCode);
        if (joshWriterChannel != NULL)
            break;
    }

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                            = NULL;
    chConfig.msgcomInstHandle                                       = appMsgcomInstanceHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the reader channel. */
    joshReaderChannel = Msgcom_create ("Core0-Reader-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (joshReaderChannel == NULL)
    {
        System_printf ("Error: Unable to create the Core0-Reader-Channel (Error Code %d)\n", errCode);
        return;
    }

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId = 0xDEAD;
    nodeCfg.transportCfg.readerChannel = joshReaderChannel;
    nodeCfg.transportCfg.writerChannel = joshWriterChannel;

    /* Populate the transport API */
    nodeCfg.transportCfg.transport.alloc = Test_joshAlloc;
    nodeCfg.transportCfg.transport.free  = Test_joshFree;
    nodeCfg.transportCfg.transport.get   = Test_joshGet;
    nodeCfg.transportCfg.transport.put   = Test_joshPut;

    /* Populate the OSAL table */
    nodeCfg.malloc      = Josh_osalMalloc;
    nodeCfg.free        = Josh_osalFree;
    nodeCfg.enterCS     = Josh_osalEnterSingleCoreCS;
    nodeCfg.exitCS      = Josh_osalExitSingleCoreCS;
    nodeCfg.createSem   = Josh_osalCreateSem;
    nodeCfg.deleteSem   = Josh_osalDeleteSem;
    nodeCfg.postSem     = Josh_osalPostSem;
    nodeCfg.pendSem     = Josh_osalPendSem;

    /* Initialize the JOSH. */
    gJoshNodeHandle = Josh_initNode (&nodeCfg, &errCode);
    if (gJoshNodeHandle == NULL)
    {
        /* Error: JOSH Initialization failed. */
        System_printf ("Error: Unable to initialize JOSH Error Code %d\n", errCode);
        return;
    }
    System_printf ("Debug: JOSH Node was initialized successfully\n");

    /* Initialize the Task Parameters for the test task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority  = 2;
    Task_create(JoshTask, &taskParams, NULL);

    /* Initialize the Task Parameters for the JOSH receive task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    taskParams.priority  = 5;
    Task_create(JoshReceiveTask, &taskParams, NULL);
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

    System_printf ("***********************************\n");
    System_printf ("********** JOSH Unit Test *********\n");
    System_printf ("***********************************\n");

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

