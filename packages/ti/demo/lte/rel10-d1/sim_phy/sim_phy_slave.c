/**
 *   @file  sim_phy_slave.c
 *
 *   @brief
 *      Simulated Slave PHY initialization code
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
#include <ti/runtime/root/root.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/memlog/memlog.h>
#include <ti/runtime/domain/domain.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>

/* Application Include Files */
#include "sim_phy.h"

/**********************************************************************
 ************************* Global Declarations ************************
 **********************************************************************/

/* Simulated PHY Domain MCB */
AppSimPHYDomainMCB    myAppDomain;

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* ROOT Slave Handle */
extern Root_SlaveHandle      rootSlaveHandle;

/* Logging: */
extern int32_t Log_initLogging (AppSimPHYDomainMCB* ptrSimPhyDomain, char* producerName, Resmgr_ResourceCfg* ptrResourceCfg);
extern int32_t Log_deinitLogging (AppSimPHYDomainMCB* ptrSimPhyDomain);

/*****************************************************************************
 * OSAL Callout Functions:
 *****************************************************************************/

/* DOMAIN: */
extern void* Domain_osalMalloc(uint32_t size);
extern void  Domain_osalFree(void* ptr, uint32_t size);
extern void* Domain_osalDataBufferMalloc (Domain_MallocMode mode, uint32_t size, uint32_t align);
extern void  Domain_osalDataBufferFree (Domain_MallocMode mode, void* ptr, uint32_t size);
extern void  Domain_osalLog(Domain_LogLevel level, char* fmt, va_list arg);
extern void* Domain_osalTaskCreate(Domain_SyslibServiceTask, Domain_TaskContext, uint32_t arg);
extern void  Domain_osalTaskDelete(void* taskHandle);
extern void  Domain_osalTaskRelinquish(uint32_t time);

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

/* NETFP: */
extern void* Netfp_osalMalloc (uint32_t , uint32_t );
extern void  Netfp_osalFree (void* , uint32_t );
extern void* Netfp_osalEnterSingleCoreCriticalSection (void);
extern void  Netfp_osalExitSingleCoreCriticalSection (void* csHandle);
extern void  Netfp_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Netfp_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Netfp_osalCreateSem(void);
extern void  Netfp_osalDeleteSem(void*);
extern void  Netfp_osalPostSem(void*);
extern void  Netfp_osalPendSem(void*);

/* DAT: */
extern void* Dat_osalMalloc (uint32_t , uint32_t );
extern void  Dat_osalFree (void* , uint32_t );
extern void* Dat_osalMallocLocal(uint32_t numBytes, uint32_t alignment);
extern void  Dat_osalFreeLocal(void* ptr, uint32_t numBytes);
extern void* Dat_osalEnterCS (void);
extern void  Dat_osalExitCS (void* csHandle);
extern void  Dat_osalBeginMemoryAccess (void* ptr, uint32_t size);
extern void  Dat_osalEndMemoryAccess (void* ptr, uint32_t size);
extern void* Dat_osalCreateSem(void);
extern void  Dat_osalDeleteSem(void*);
extern void  Dat_osalPostSem(void*);
extern void  Dat_osalPendSem(void*);

/* MEMLOG: */
extern void* Memlog_osalMalloc (Memlog_MemAllocMode , uint32_t );
extern void  Memlog_osalFree (Memlog_MemAllocMode, void*, uint32_t );
extern void* Memlog_osalEnterCS (void);
extern void  Memlog_osalExitCS (void* csHandle);

/**********************************************************************
 *********************** Application Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      L2 Initialization Task which is spawned once the root master
 *      has initiated the application initialization.
 *
 *  @param[in]  arg0
 *      Argument 0
 *  @param[in]  arg1
 *      Argument 1
 *
 *  @retval
 *      Not Applicable.
 */
static void SimPhy_initTask (UArg arg0, UArg arg1)
{
    AppSimPHYDomainMCB*         ptrSimPhyDomain;
    Domain_SyslibCfg            syslibDomainCfg;
    int32_t                     errCode;
    Resmgr_ResourceCfg          domainResourceCfg =
    {
    	0,    /* Number of CPINTC Output  requested                               */
	    0,    /* Number of Accumulator Channels requested                         */
    	0,    /* Number of Hardware Semaphores requested                          */
	    1,    /* Number of QPEND Queues requested                                 */
    	/* Requested Memory Region Configuration. */
	    {
    		/* Name,           Type,                       Linking RAM,                           Num,      Size */
	    	{ "Domain-Sylib",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  1024,      128},
	    }
    };
    Resmgr_ResourceCfg      phySlaveResourceCfg =
    {
    	0,    /* Number of CPINTC Output  requested                               */
	    0,    /* Number of Accumulator Channels requested                         */
    	0,    /* Number of Hardware Semaphores requested                          */
	    0,    /* Number of QPEND Queues requested                                 */
    	/* Requested Memory Region Configuration. */
	    {
    		/* Name,                Type,                       Linking RAM,                           Num,     Size */
            { "Slave-MSMC-Region",  Resmgr_MemRegionType_MSMC,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  64,      128},
	    	{ "Slave-DDR3-Region",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  1024,    128},
	    }
    };

    /* Get the application domain. */
    ptrSimPhyDomain = (AppSimPHYDomainMCB*)arg0;

    /* Initialize the SYSLIB domain configuration */
    memset ((void *)&syslibDomainCfg, 0, sizeof(Domain_SyslibCfg));

    /* Populate the SYSLIB Domain configuration: */
    memcpy ((void *)&syslibDomainCfg.rootSyslibCfg, (void *)&ptrSimPhyDomain->rootSyslibCfg, sizeof(Root_SyslibConfig));
    memcpy ((void *)&syslibDomainCfg.domainResourceCfg, (void *)&domainResourceCfg, sizeof(Resmgr_ResourceCfg));

    /* Populate the DOMAIN OSAL function table */
    syslibDomainCfg.domainOsalFxnTable.malloc               = Domain_osalMalloc;
    syslibDomainCfg.domainOsalFxnTable.free                 = Domain_osalFree;
    syslibDomainCfg.domainOsalFxnTable.dataMalloc           = Domain_osalDataBufferMalloc;
    syslibDomainCfg.domainOsalFxnTable.dataFree             = Domain_osalDataBufferFree;
    syslibDomainCfg.domainOsalFxnTable.log                  = Domain_osalLog;
    syslibDomainCfg.domainOsalFxnTable.taskCreate           = Domain_osalTaskCreate;
    syslibDomainCfg.domainOsalFxnTable.taskDelete           = Domain_osalTaskDelete;
    syslibDomainCfg.domainOsalFxnTable.taskRelinquish       = Domain_osalTaskRelinquish;

    /* Populate the DOMAIN Resource OSAL function table */
    syslibDomainCfg.resmgrOsalFxnTable.malloc               = Resmgr_osalMalloc;
    syslibDomainCfg.resmgrOsalFxnTable.free                 = Resmgr_osalFree;
    syslibDomainCfg.resmgrOsalFxnTable.mallocMemoryRegion   = Resmgr_osalMallocMemoryRegion;
    syslibDomainCfg.resmgrOsalFxnTable.freeMemoryRegion     = Resmgr_osalFreeMemoryRegion;
    syslibDomainCfg.resmgrOsalFxnTable.createSem            = Resmgr_osalCreateSem;
    syslibDomainCfg.resmgrOsalFxnTable.deleteSem            = Resmgr_osalDeleteSem;
    syslibDomainCfg.resmgrOsalFxnTable.postSem              = Resmgr_osalPostSem;
    syslibDomainCfg.resmgrOsalFxnTable.pendSem              = Resmgr_osalPendSem;
    syslibDomainCfg.resmgrOsalFxnTable.beginMemAccess       = Resmgr_osalBeginMemAccess;
    syslibDomainCfg.resmgrOsalFxnTable.endMemAccess         = Resmgr_osalEndMemAccess;

    /* Populate the DOMAIN Name Database OSAL function table */
    syslibDomainCfg.nameDBOsalFxnTable.malloc             	= Name_osalDBMalloc;
    syslibDomainCfg.nameDBOsalFxnTable.free               	= Name_osalDBFree;
    syslibDomainCfg.nameDBOsalFxnTable.enterCS            	= Name_osalEnterMultipleCoreCS;
    syslibDomainCfg.nameDBOsalFxnTable.exitCS             	= Name_osalExitMultipleCoreCS;
    syslibDomainCfg.nameDBOsalFxnTable.beginMemAccess     	= Name_osalBeginMemAccess;
    syslibDomainCfg.nameDBOsalFxnTable.endMemAccess       	= Name_osalEndMemAccess;

    /* Populate the DOMAIN AGENT Client OSAL Function table */
    syslibDomainCfg.nameOsalFxnTable.malloc                 = Name_osalMalloc;
    syslibDomainCfg.nameOsalFxnTable.free                   = Name_osalFree;
    syslibDomainCfg.nameOsalFxnTable.beginMemAccess         = Name_osalBeginMemAccess;
    syslibDomainCfg.nameOsalFxnTable.endMemAccess           = Name_osalEndMemAccess;
    syslibDomainCfg.nameOsalFxnTable.enterCS                = Name_osalEnterCS;
    syslibDomainCfg.nameOsalFxnTable.exitCS                 = Name_osalExitCS;
    syslibDomainCfg.nameOsalFxnTable.createSem              = Name_osalCreateSem;
    syslibDomainCfg.nameOsalFxnTable.deleteSem              = Name_osalDeleteSem;
    syslibDomainCfg.nameOsalFxnTable.postSem                = Name_osalPostSem;
    syslibDomainCfg.nameOsalFxnTable.pendSem                = Name_osalPendSem;

    /* Populate the DOMAIN PKTLIB OSAL Function table */
    syslibDomainCfg.pktlibOsalFxnTable.malloc               = Pktlib_osalMalloc;
    syslibDomainCfg.pktlibOsalFxnTable.free                 = Pktlib_osalFree;
    syslibDomainCfg.pktlibOsalFxnTable.beginMemAccess       = Pktlib_osalBeginMemAccess;
    syslibDomainCfg.pktlibOsalFxnTable.endMemAccess         = Pktlib_osalEndMemAccess;
    syslibDomainCfg.pktlibOsalFxnTable.beginPktAccess       = Pktlib_osalBeginPktAccess;
    syslibDomainCfg.pktlibOsalFxnTable.endPktAccess         = Pktlib_osalEndPktAccess;
    syslibDomainCfg.pktlibOsalFxnTable.enterCS              = Pktlib_osalEnterCS;
    syslibDomainCfg.pktlibOsalFxnTable.exitCS               = Pktlib_osalExitCS;

    /* Populate the DOMAIN MSGCOM OSAL Function table */
    syslibDomainCfg.msgcomOsalFxnTable.malloc               = Msgcom_osalMalloc;
    syslibDomainCfg.msgcomOsalFxnTable.free                 = Msgcom_osalFree;
    syslibDomainCfg.msgcomOsalFxnTable.registerIsr          = Msgcom_osalRegisterIsr;
    syslibDomainCfg.msgcomOsalFxnTable.deregisterIsr        = Msgcom_osalDeregisterIsr;
    syslibDomainCfg.msgcomOsalFxnTable.disableSysInt        = Msgcom_osalDisableSysInt;
    syslibDomainCfg.msgcomOsalFxnTable.enableSysInt         = Msgcom_osalEnableSysInt;
    syslibDomainCfg.msgcomOsalFxnTable.enterCS              = Msgcom_osalEnterSingleCoreCS;
    syslibDomainCfg.msgcomOsalFxnTable.exitCS               = Msgcom_osalExitSingleCoreCS;
    syslibDomainCfg.msgcomOsalFxnTable.createSem            = Msgcom_osalCreateSem;
    syslibDomainCfg.msgcomOsalFxnTable.deleteSem            = Msgcom_osalDeleteSem;
    syslibDomainCfg.msgcomOsalFxnTable.postSem              = Msgcom_osalPostSem;
    syslibDomainCfg.msgcomOsalFxnTable.pendSem              = Msgcom_osalPendSem;

    /* Populate the DOMAIN NETFP OSAL Function table */
    syslibDomainCfg.netfpOsalFxnTable.malloc                = Netfp_osalMalloc;
    syslibDomainCfg.netfpOsalFxnTable.free                  = Netfp_osalFree;
    syslibDomainCfg.netfpOsalFxnTable.beginMemAccess        = Netfp_osalBeginMemoryAccess;
    syslibDomainCfg.netfpOsalFxnTable.endMemAccess          = Netfp_osalEndMemoryAccess;
    syslibDomainCfg.netfpOsalFxnTable.enterCS               = Netfp_osalEnterSingleCoreCriticalSection;
    syslibDomainCfg.netfpOsalFxnTable.exitCS                = Netfp_osalExitSingleCoreCriticalSection;
    syslibDomainCfg.netfpOsalFxnTable.createSem             = Netfp_osalCreateSem;
    syslibDomainCfg.netfpOsalFxnTable.deleteSem             = Netfp_osalDeleteSem;
    syslibDomainCfg.netfpOsalFxnTable.postSem               = Netfp_osalPostSem;
    syslibDomainCfg.netfpOsalFxnTable.pendSem               = Netfp_osalPendSem;

    /* Populate the DOMAIN DAT OSAL Function table */
    syslibDomainCfg.datOsalFxnTable.malloc                 = Dat_osalMalloc;
    syslibDomainCfg.datOsalFxnTable.free                   = Dat_osalFree;
    syslibDomainCfg.datOsalFxnTable.mallocLocal            = Dat_osalMallocLocal;
    syslibDomainCfg.datOsalFxnTable.freeLocal              = Dat_osalFreeLocal;
    syslibDomainCfg.datOsalFxnTable.beginMemAccess         = Dat_osalBeginMemoryAccess;
    syslibDomainCfg.datOsalFxnTable.endMemAccess           = Dat_osalEndMemoryAccess;
    syslibDomainCfg.datOsalFxnTable.enterCS                = Dat_osalEnterCS;
    syslibDomainCfg.datOsalFxnTable.exitCS                 = Dat_osalExitCS;
    syslibDomainCfg.datOsalFxnTable.createSem              = Dat_osalCreateSem;
    syslibDomainCfg.datOsalFxnTable.deleteSem              = Dat_osalDeleteSem;
    syslibDomainCfg.datOsalFxnTable.postSem                = Dat_osalPostSem;
    syslibDomainCfg.datOsalFxnTable.pendSem                = Dat_osalPendSem;

    /* Populate the DOMAIN MEMLOG OSAL Function table */
    syslibDomainCfg.memlogOsalFxnTable.malloc              = Memlog_osalMalloc;
    syslibDomainCfg.memlogOsalFxnTable.free                = Memlog_osalFree;
    syslibDomainCfg.memlogOsalFxnTable.enterCS             = Memlog_osalEnterCS;
    syslibDomainCfg.memlogOsalFxnTable.exitCS              = Memlog_osalExitCS;

    /* Initialize the Domain */
    ptrSimPhyDomain->syslibHandle = Domain_initSyslibServices (ptrSimPhyDomain->appId, &syslibDomainCfg, &errCode);
    if (ptrSimPhyDomain->syslibHandle == NULL)
    {
        System_printf ("FATAL Error: Domain initialization failed [Error code %d]\n", errCode);
        return;
    }

    /* Request the resource manager for the resources requested by the application */
    if (Resmgr_processConfig (Domain_getSysCfgHandle(ptrSimPhyDomain->syslibHandle), &phySlaveResourceCfg, &errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", errCode);
	    return;
    }

    /*******************************************************************************
     * Application Developers: Please add any *important* application initialization
     * if any which need to be done here. We dont want to announce that the domain
     * is up till all the services are ready.
     *
     * For example:
     * - Initialize the DAT Logging Infrastructure
     *******************************************************************************/
    if (ptrSimPhyDomain->rootSyslibCfg.datClientConfig.instantiateDatClient == 1)
    {
        if (Log_initLogging (ptrSimPhyDomain, "UIA_CONTRACT_PHY_SLAVE", &phySlaveResourceCfg) < 0)
            return;
    }

    /*******************************************************************************
     * Application Developers: Announce to the master that the domain is operational
     *******************************************************************************/
    if (Root_appUp (rootSlaveHandle, ptrSimPhyDomain->appId, (void*)ptrSimPhyDomain, &errCode) < 0)
    {
        System_printf ("Error: Root application UP failed [Error code %d]\n", errCode);
        return;
    }

    /*******************************************************************************
     * Application Developers: Do your work here.
     *******************************************************************************/
    while (1)
        Task_sleep(1000);
}

/**
 *  @b Description
 *  @n
 *      Application initialization entry point which is invoked on each
 *      root slave when an application domain is being created by the
 *      root master.
 *
 *      NOTE: This is called in the context of the root thread. Please be
 *      careful and place only basic initialization code here. We do not
 *      want the root to crash.
 *
 *  @param[in]  appId
 *      Application Identifier
 *  @param[in]  ptrDomainCfg
 *      Pointer to the application domain configuration passed by the root
 *      master
 *  @param[in]  ptrRootSyslibCfg
 *      Pointer to the ROOT SYSLIB configuration.
 *
 *  @retval
 *      Not Applicable.
 */
void appInit (uint32_t appId, void* ptrDomainCfg, Root_SyslibConfig* ptrRootSyslibCfg)
{
    Task_Params             taskParams;
    AppSimPHYDomainMCB*     ptrSimPhyDomain;
    uint32_t*               appConfig;

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the application domain pointer */
    ptrSimPhyDomain = &myAppDomain;

    /* Get the application configuration: */
    appConfig = (uint32_t*)ptrDomainCfg;

    /* Initialize the allocated memory block. */
    memset ((void *)ptrSimPhyDomain, 0, sizeof(AppSimPHYDomainMCB));

    /* Populate the simulated PHY domain. */
    ptrSimPhyDomain->appId                 = appId;
    ptrSimPhyDomain->phyId                 = (*appConfig & 0x1) ? 'A' : 'B';
    ptrSimPhyDomain->localHeapHandle       = NULL;
    ptrSimPhyDomain->msmcSharedHeapHandle  = (xdc_runtime_IHeap_Handle)SharedRegion_getHeap(0);
    ptrSimPhyDomain->ddr3SharedHeapHandle  = (xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1);
    ptrSimPhyDomain->ddr3PrivateHeapHandle = (xdc_runtime_IHeap_Handle)ddr3PrivateHeap;

    /* Copy the SYSLIB configuration: */
    memcpy ((void *)&ptrSimPhyDomain->rootSyslibCfg, (void*)ptrRootSyslibCfg, sizeof(Root_SyslibConfig));

    /* Debug Message: */
    System_printf ("Debug: Application Initialization %x on Core %d PHY %c %d starting\n", appId, DNUM, ptrSimPhyDomain->phyId, *appConfig);

    /* Launch the PHY initialization task. The domain startup sequence has been initialized
     * and the initialization task has been spawned with the necessary configuration information.
     * The appInit is called in the context of the root and we wish to keep the code executing in
     * this context to be as minimal as possible. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority  = 4;
    taskParams.arg0      = (UArg)ptrSimPhyDomain;
    ptrSimPhyDomain->simPhyInitTaskHandle = Task_create((ti_sysbios_knl_Task_FuncPtr)SimPhy_initTask, &taskParams, NULL);
    return;
}

/**
 *  @b Description
 *  @n
 *      L2 Initialization Task which is spawned once the root master
 *      has initiated the application initialization.
 *
 *  @param[in]  arg0
 *      Argument 0
 *  @param[in]  arg1
 *      Argument 1
 *
 *  @retval
 *      Not Applicable.
 */
static void SimPhy_deinitTask (UArg arg0, UArg arg1)
{
    AppSimPHYDomainMCB*     ptrSimPhyDomain;
    int32_t                 errCode;

    /* Get the application domain. */
    ptrSimPhyDomain = (AppSimPHYDomainMCB*)arg0;

    /*******************************************************************************
     * Application Developers: Please add application specific cleanup here.
     * Example: Shutdown the application tasks
     *******************************************************************************/
    if (ptrSimPhyDomain->simPhyInitTaskHandle != NULL)
        Task_delete(&ptrSimPhyDomain->simPhyInitTaskHandle);

    /* Deinitialize and shutdown the logging module */
    Log_deinitLogging(ptrSimPhyDomain);

    /* Shutdown the domain */
    if (Domain_deinitSyslibServices (ptrSimPhyDomain->syslibHandle, &errCode) < 0)
    {
        System_printf ("FATAL Error: Shutting down the application domain failed [Error code %d]\n", errCode);
        return;
    }

    /* Inform the root master that the domain has been completely deinitialized. */
    if (Root_appDeinitialized (rootSlaveHandle, ptrSimPhyDomain->appId, &errCode) < 0)
        System_printf ("Error: Root deinitialization failed [Error code %d]\n", errCode);

    return;
}

/**
 *  @b Description
 *  @n
 *      Application deinitialization entry point which is invoked on each root slave
 *      when an application domain is destroyed.
 *
 *  @param[in]  appDomainHandle
 *      Application Domain Handle
 *
 *      NOTE: This is called in the context of the root thread. Please be
 *      careful and place only basic deinitialization code here. We do not
 *      want the root to crash.
 *
 *  @retval
 *     Not application
 */
void appDeinit (void* appDomainHandle)
{
    AppSimPHYDomainMCB*     ptrSimPhyDomain;
    Task_Params             taskParams;

    /* Initialize the application domain pointer */
    ptrSimPhyDomain = (AppSimPHYDomainMCB*)appDomainHandle;

    /* Debug Message: */
    System_printf ("Debug: Application Deinitialization %x on Core %d starting\n", ptrSimPhyDomain, DNUM);

    /* Launch the L2 deinitialization task. The domain cleanup sequence has been initialized and
     * the deinitialization task has been spawned. This function is called in the context of the root
     * task and we wish to preserve its integrity. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority  = 4;
    taskParams.arg0      = (UArg)ptrSimPhyDomain;
    Task_create((ti_sysbios_knl_Task_FuncPtr)SimPhy_deinitTask, &taskParams, NULL);
}

