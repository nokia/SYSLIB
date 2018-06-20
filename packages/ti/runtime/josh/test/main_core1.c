/**
 *   @file  main.c
 *
 *   @brief
 *      Unit Test for the JOSH which executes on the Core1.
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
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK Include Files */
#include <ti/platform/platform.h>
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
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/netfp/netfp.h>

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Shared Heaps: */
Pktlib_HeapHandle       joshHeap;

/* Global System configuration handle */
Resmgr_SysCfgHandle     handleSysCfg;

/* Global name database handle */
Name_DBHandle           globalNameDatabaseHandle;

/* MSGCOM Instance handle. */
Msgcom_InstHandle       appMsgcomInstanceHandle;

/* PKTLIB Instance handle. */
Pktlib_InstHandle       appPktlibInstanceHandle;

/* Application requested resources */
Resmgr_ResourceCfg    appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
	1,    /* Number of Accumulator Channels requested                         */
	2,    /* Number of Hardware Semaphores requested                          */
    4,    /* Number of QPEND Queues requested                                 */
	/* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                     Linking RAM,                            Num,   Size */
		{ "Core1-DDR3", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,    128,    64}
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
 *      Job1 Code
 *
 *  @retval
 *      Always returns 10
 */
static uint32_t job1 (void)
{
    return 10;
}

/**
 *  @b Description
 *  @n
 *      Square Number
 *
 *  @param[in]  number
 *      Input Value
 *
 *  @retval
 *      Square of the number.
 */
static uint32_t squareNumber (uint32_t number)
{
    number = josh_toNativeU32(number);
    return number*number;
}

/**
 *  @b Description
 *  @n
 *      Sum of all numbers
 *
 *  @param[in]  number1
 *      Input Value
 *  @param[in]  number2
 *      Input Value
 *  @param[in]  number3
 *      Input Value
 *  @param[in]  number4
 *      Input Value
 *
 *  @retval
 *      Sum of all the numbers.
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
 *      Sum all the elements in the array.
 *
 *  @param[in]  array
 *      The starting address of the array
 *  @param[in]  numElements
 *      The number of elements in the array
 *
 *  @retval
 *      Returns sum of all the elements.
 */
static uint32_t sumArray (uint32_t* array, uint32_t numElements)
{
    uint32_t index = 0;
    uint32_t sum   = 0;

    for (index = 0; index < josh_toNativeU32(numElements); index++)
        sum = sum + josh_toNativeU32(array[index]);

    return sum;
}

/**
 *  @b Description
 *  @n
 *      Multiple each element of an array by 2 and store in another array.
 *
 *  @param[in]  inArray
 *      The starting address of the array
 *  @param[in]  number of elements
 *      The number of elements in the array
 *  @param[out] outArray
 *      Output array populated with the results.
 *
 *  @retval
 *      Returns number of elements
 */
static uint32_t arrayMultiply (uint32_t* inArray, int32_t numElements, uint32_t* outArray)
{
    uint32_t index = 0;

    for (index = 0; index < josh_toNativeU32(numElements); index++)
        outArray[index] = josh_toRemoteU32(josh_toNativeU32(inArray[index]) * 2);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Displays the string which was passed and then modifies it.
 *
 *  @param[in]  str1
 *      String received by the MASTER core.
 *
 *  @retval
 *      Always returns 100
 */
uint32_t stringPlay (uint8_t* str1)
{
    System_printf ("Argument passed to the JOB is '%s'\n", str1);

    /* Modify the argument. */
    strcpy ((char *)str1, "Hello from Slave");
    return 100;
}

/**
 *  @b Description
 *  @n
 *      Job which process the Fast Path structure
 *
 *  @param[in]  ptrFPConfig
 *      Pointer to the fast path configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   0xFFFFFFFF
 *
 */
uint32_t jobStructProcessing(Netfp_InboundFPCfg* ptrFPConfig)
{
    /* Validate the received configuration. */
    if (ptrFPConfig->srcIP.ver != josh_toNativeU32(Netfp_IPVersion_IPV4))
        return (uint32_t)(-1);

    if (ptrFPConfig->dstIP.ver != josh_toNativeU32(Netfp_IPVersion_IPV4))
        return (uint32_t)(-1);

    if ( (ptrFPConfig->srcIP.addr.ipv4.u.a8[0] != 0xc0) ||
         (ptrFPConfig->srcIP.addr.ipv4.u.a8[1] != 0xa8) ||
         (ptrFPConfig->srcIP.addr.ipv4.u.a8[2] != 0x01) ||
         (ptrFPConfig->srcIP.addr.ipv4.u.a8[3] != 0x01) )
        return (uint32_t)(-1);

    if ( (ptrFPConfig->dstIP.addr.ipv4.u.a8[0] != 0xc0) ||
         (ptrFPConfig->dstIP.addr.ipv4.u.a8[1] != 0xa8) ||
         (ptrFPConfig->dstIP.addr.ipv4.u.a8[2] != 0x01) ||
         (ptrFPConfig->dstIP.addr.ipv4.u.a8[3] != 0x02) )
        return (uint32_t)(-1);

    return 0;
}

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
 *      JOSH Task
 *
 *  @retval
 *      Not Applicable.
 */
void JoshTask(UArg arg0, UArg arg1)
{
    Josh_NodeHandle         nodeHandle;
    Josh_NodeCfg            nodeCfg;
    int32_t                 errCode;
    MsgCom_ChHandle         joshWriterChannel;
    MsgCom_ChHandle         joshReaderChannel;
    Msgcom_ChannelCfg       chConfig;

    /* Populate the channel configuration. */
    chConfig.mode                     = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.msgcomInstHandle         = appMsgcomInstanceHandle;
    chConfig.appCallBack              = NULL;
    chConfig.u.queueCfg.interruptMode = Msgcom_QueueInterruptMode_NO_INTERRUPT;

    /* Create the reader channel. */
    joshReaderChannel = Msgcom_create ("Core1-Reader-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (joshReaderChannel == NULL)
    {
        System_printf ("Error: Unable to create the Core0-Reader-Channel (Error Code %d)\n", errCode);
        return;
    }

    /* Wait for the writer channel i.e. Core1 reader channel to get created */
    while (1)
    {
        joshWriterChannel = Msgcom_find (appMsgcomInstanceHandle, "Core0-Reader-Channel", &errCode);
        if (joshWriterChannel != NULL)
            break;
    }

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration. */
    nodeCfg.nodeId        = 0xBEEF;
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
    nodeHandle = Josh_initNode (&nodeCfg, &errCode);
    if (nodeHandle == NULL)
    {
        /* Error: JOSH Initialization failed. */
        System_printf ("Error: Unable to initialize JOSH Error Code %d\n", errCode);
        return;
    }
    System_printf ("Debug: JOSH Node was initialized successfully\n");

    /* Register the jobs in the SLAVE */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)job1);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)squareNumber);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)sum);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)sumArray);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)arrayMultiply);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)stringPlay);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)jobStructProcessing);

    /* Setup the error code. */
    errCode = -1;

    /* Sanity Check: */
    if (Josh_findJobByAddress(nodeHandle, (Josh_JobProtype)job1) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(nodeHandle, (Josh_JobProtype)squareNumber) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(nodeHandle, (Josh_JobProtype)sum) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(nodeHandle, (Josh_JobProtype)sumArray) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else if (Josh_findJobByAddress(nodeHandle, (Josh_JobProtype)arrayMultiply) == NULL)
        System_printf ("Error: JOSH Find Job by address failed\n");
    else
        errCode = 0;

    /* Did we get an error? */
    if (errCode == -1)
        return;

    /* No error proceed with testing. */
    System_printf ("Debug: JOSH Job Registration & Lookup was successful.\n");

    /* Since this is a slave loop around waiting for JOSH requests to arrive */
    while (1)
    {
        /* Try and process a JOSH message: */
        if (Josh_receive(nodeHandle, &errCode) < 0)
        {
            /* Error: Unable to handle it. Check the error code? */
            if (errCode == JOSH_ENOMSG)
            {
                /* No MSG was received; we will retry after some time. */
                Task_sleep(2);
                continue;
            }
            System_printf ("Error: JOSH Receive Failed with Error %d\n", errCode);
            return;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task
 *
 *  @retval
 *      Not Applicable.
 */
void Test_sysInitTask(UArg arg0, UArg arg1)
{
    Task_Params                 taskParams;
    int32_t                     errCode;
    Resmgr_SystemCfg		    sysConfig;
    Name_DatabaseCfg            databaseCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    Pktlib_InstCfg              pktlibInstCfg;

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    sysConfig.coreId                            = DNUM;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L1");
    strcpy (sysConfig.rmServer, "Rm_Server");
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
	    return;
    }
    System_printf ("Debug: SYSRM initialized successfully [Handle %p]\n", handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (handleSysCfg, &appResourceConfig, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return;
    }

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration: */
    databaseCfg.instanceId                        = 1;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strcpy (databaseCfg.owner, "LTE9A_L2");
    databaseCfg.dspCfg.baseNamedResourceAddress   = LTE1_DDR3_NAMED_RESOURCE;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = LTE1_DDR3_NAMED_RESOURCE_LEN;
    databaseCfg.dspCfg.initNamedResourceDatabase  = 0;
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
	    return;
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
        return;
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
        return;
    }

    /* Find all the heaps. */
    while (1)
    {
        joshHeap = Pktlib_findHeapByName(appPktlibInstanceHandle, "JOSH-Heap", &errCode);
        if (joshHeap != NULL)
            break;
    }

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority = 2;
    Task_create(JoshTask, &taskParams, NULL);
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
    Task_Params	taskParams;

    /* Initialize TSCL register */
    TSCL = 0;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the heap in shared memory. Using IPC module to do that */
    Ipc_start();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority = 2;
    Task_create(Test_sysInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

