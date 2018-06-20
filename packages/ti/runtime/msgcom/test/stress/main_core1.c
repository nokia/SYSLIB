/**
 *   @file  main_core1.c
 *
 *   @brief
 *      Unit Test Code for the message communicator module which executes
 *      on Core1
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

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* PDK & CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* SYSLIB Include Files */
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>

/**********************************************************************
 ************************* Unit Test Definitions **********************
 **********************************************************************/

/* Maximum number of messages sent and received by the test */
#define TEST_MAX_MSG_COUNT          10

/**********************************************************************
 ************************ Unit Test Global Variables ******************
 **********************************************************************/

/* Global Name Proxy */
const char* gNameProxyName  = "NameServer_LTE9A";

/* Global SYSLIB Handle(s): */
Name_ClientHandle       nameClientHandle;
Resmgr_SysCfgHandle     handleSysCfg;
Name_DBHandle           globalNameDatabaseHandle;
Msgcom_InstHandle       appMsgcomInstanceHandle;
Pktlib_InstHandle       appPktlibInstanceHandle;
Pktlib_HeapHandle       myPktLibHeapHandle;
Pktlib_HeapHandle       mySharedHeapHandle;

/* Global Variable: Test which we need to execute */
uint32_t                testSelection;

/* Application requested resources */
Resmgr_ResourceCfg    appResourceConfig =
{
    0,    /* Number of CPINTC Output  requested                               */
	0,    /* Number of Accumulator Channels requested                         */
	0,    /* Number of Hardware Semaphores requested                          */
    1,    /* Number of QPEND Queues requested                                 */
	/* Requested Memory Region Configuration. */
	{
        /* Name,           Type,                    Linking RAM,                            Num,    Size */
		{ "Core1-Local", Resmgr_MemRegionType_LOCAL, Resmgr_MemRegionLinkingRAM_DONT_CARE,  64,     128},
        { "Core1-DDR3",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  256,    128},
    }
};

/**********************************************************************
 *********************** Unit Test Extern Definitions *****************
 **********************************************************************/

/* Test tasks: */
extern Void stressWriterTask(UArg arg0, UArg arg1);

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

/* Name Proxy/Client: */
extern void* Name_osalMalloc (uint32_t, uint32_t);
extern void  Name_osalFree (void* , uint32_t);
extern void  Name_osalBeginMemAccess (void*, uint32_t);
extern void  Name_osalEndMemAccess (void*, uint32_t);
extern void* Name_osalEnterCS (void);
extern void  Name_osalExitCS (void*);
extern void* Name_osalCreateSem (void);
extern void  Name_osalDeleteSem(void*);
extern void  Name_osalPendSem(void*);
extern void  Name_osalPostSem(void*);

/**********************************************************************
 ************************** Unit Test Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Name Client Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void NameClientExecutionTask(UArg arg0, UArg arg1)
{
    int32_t                     errCode;
    Pktlib_HeapHandle           nameClientProxyHeapHandle;
    Name_ClientCfg              clientCfg;

    /***************************************************************************
     * Name Proxy Client Heap: The heap is shared between the NAME proxy and the
     * local name clients. This is created by the core executing the NAME proxy
     * and is found on the client cores.
     ***************************************************************************/
    while (1)
    {
        nameClientProxyHeapHandle = Pktlib_findHeapByName (appPktlibInstanceHandle,
                                                           "NameProxyClientHeap", &errCode);
        if (nameClientProxyHeapHandle != NULL)
            break;

        /* Relinquish time slice */
        Task_sleep(1);
    }

    /* Initialize the name client configuration block. */
    memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

    /* Populate the client configuration: */
    strcpy (clientCfg.clientName, "NameClient_LTE9A_L2SCHED");
    strcpy (clientCfg.proxyName, gNameProxyName);
    clientCfg.databaseHandle            = globalNameDatabaseHandle;
    clientCfg.realm                     = Name_ExecutionRealm_DSP;
    clientCfg.u.dspCfg.pktlibInstHandle = appPktlibInstanceHandle;
    clientCfg.u.dspCfg.msgcomInstHandle = appMsgcomInstanceHandle;
    clientCfg.u.dspCfg.clientHeapHandle = nameClientProxyHeapHandle;
    clientCfg.malloc                    = Name_osalMalloc;
    clientCfg.free                      = Name_osalFree;
    clientCfg.beginMemAccess            = Name_osalBeginMemAccess;
    clientCfg.endMemAccess              = Name_osalEndMemAccess;
    clientCfg.enterCS                   = Name_osalEnterCS;
    clientCfg.exitCS                    = Name_osalExitCS;
    clientCfg.createSem                 = Name_osalCreateSem;
    clientCfg.deleteSem                 = Name_osalDeleteSem;
    clientCfg.postSem                   = Name_osalPostSem;
    clientCfg.pendSem                   = Name_osalPendSem;

    /* Name client are only created if the proxy is operational. */
    while (1)
    {
        /* Initialize the name client */
        nameClientHandle = Name_initClient (&clientCfg, &errCode);
        if (nameClientHandle != NULL)
            break;

        /* Error: Unable to create the client. */
        if (errCode != NAME_ENOTREADY)
        {
            System_printf ("Error: Unable to create the name client [Error code %d]\n", errCode);
            return;
        }
        /* Server was not operational. Wait for some time and try again. */
        Task_sleep(1);
        break;
    }
    System_printf ("Debug: Name Client %p has been created successfully\n", nameClientHandle);

    /* Execute the name client */
    while (1)
    {
        Name_executeClient (nameClientHandle);
        Task_sleep(1);
    }
}

/**
 *  @b Description
 *  @n
 *      Msgcom test task
 *
 *  @retval
 *      Not Applicable.
 */
Void Test_MsgcomTask(UArg arg0, UArg arg1)
{
    Task_Params                 taskParams;
    int32_t                     errCode;
    Name_ResourceCfg            namedResCfg;
    Resmgr_SystemCfg            sysConfig;
    Pktlib_InstCfg              pktlibInstCfg;
    Msgcom_InstCfg              msgcomInstCfg;
    Name_DatabaseCfg            databaseCfg;

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
	    return;
    }
    System_printf ("Debug: Name database created successfully [Handle %p]\n", globalNameDatabaseHandle);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

    /* Populate the configuration.*/
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    strcpy (sysConfig.rmClient, "Rm_LTE9A_L1");
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
    sysConfig.dspSystemCfg.armCoreId            = SYSLIB_ARM_CORE_ID;
    sysConfig.dspSystemCfg.sourceId             = 17;
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

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId      = 1;
    pktlibInstCfg.databaseHandle  = globalNameDatabaseHandle;
    pktlibInstCfg.sysCfgHandle    = handleSysCfg;
    pktlibInstCfg.malloc          = Pktlib_osalMalloc;
    pktlibInstCfg.free            = Pktlib_osalFree;
    pktlibInstCfg.beginMemAccess  = Pktlib_osalBeginMemAccess;
    pktlibInstCfg.endMemAccess    = Pktlib_osalEndMemAccess;
    pktlibInstCfg.beginPktAccess  = Pktlib_osalBeginPktAccess;
    pktlibInstCfg.endPktAccess    = Pktlib_osalEndPktAccess;
    pktlibInstCfg.enterCS         = Pktlib_osalEnterCS;
    pktlibInstCfg.exitCS          = Pktlib_osalExitCS;

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

    /* Loop around till the test selection has been entered on the core0. */
    while (1)
    {
        if (Name_findResource(globalNameDatabaseHandle,Name_ResourceBucket_USER_DEF1,
                              "TestSelection", &namedResCfg, &errCode) < 0)
        {
            if (errCode == NAME_ENOTFOUND)
                continue;

            /* FATAL Error: Find API returned an error. */
            System_printf ("Error: Find Named resource failed with error code %d\n", errCode);
            return;
        }
        /* Get the test selection. */
        testSelection = namedResCfg.handle1;
        break;
    }

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    taskParams.priority  = 5;
    Task_create(NameClientExecutionTask, &taskParams, NULL);

    /* Initialize the task parameters for the test task. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.priority  = 6;
    Task_create(stressWriterTask, &taskParams, NULL);
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
    Task_create(Test_MsgcomTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

