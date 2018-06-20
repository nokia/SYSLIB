/**
 *   @file  l2.c
 *
 *   @brief
 *      Sample L2 LTE stack application executing on the ARM
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>

/* Device specific dependencies: */
#ifdef DEVICE_K2H
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K)
#include <ti/drv/qmss/device/k2k/src/qmss_device.c>
#include <ti/drv/cppi/device/k2k/src/cppi_device.c>
#elif defined (DEVICE_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#endif

/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/domain/domain.h>
#include <ti/runtime/dat/dat.h>

/* Application Include Files */
#include "l2_lte.h"

/*********************************************************************
 * OSAL Callout Functions:
 *********************************************************************/

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
extern void  Resmgr_osalBeginMemAccess (void*, uint32_t);
extern void  Resmgr_osalEndMemAccess (void*, uint32_t);
extern void* Resmgr_osalCreateSem (void);
extern void  Resmgr_osalDeleteSem (void*);
extern void  Resmgr_osalPostSem (void*);
extern void  Resmgr_osalPendSem (void*);

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
extern void* Pktlib_osalPhyToVirt(void* ptrPhysicalAddress);

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
 ************************* Global Declarations ************************
 **********************************************************************/

/* LTE Stack Domain MCB */
AppLTEStackDomainMCB    myAppDomain;

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* ROOT Slave Handle */
extern Root_SlaveHandle      rootSlaveHandle;

/* FAPI component: */
extern int32_t Fapi_setupIPv4Env (AppLTEStackDomainMCB* ptrLTEStackDomain);
extern void Fapi_deInitIPv4Env (AppLTEStackDomainMCB* ptrLTEStackDomain);
extern void* Fapi_initComponent (AppLTEStackDomainMCB* ptrLTEStackDomain, char phyId, Resmgr_ResourceCfg* ptrL2ResourceCfg, int32_t* errCode);
extern void Fapi_deinitComponent (void* fapiHandle);
extern void Fapi_initPhy (void* fapiHandle);

/* Logging component: */
extern int32_t Log_initLogging (AppLTEStackDomainMCB* ptrLTEStackDomain, Resmgr_ResourceCfg* ptrL2ResourceCfg);
extern int32_t Log_deinitLogging (AppLTEStackDomainMCB* ptrLTEStackDomain);

/* L2 UserPlane APIs */
extern void L2_UserPlane_DeInitIPv4Env (AppLTEStackDomainMCB* ptrLTEStackDomain);
extern int32_t L2_UserPlane_SetupIPv4Env (AppLTEStackDomainMCB* ptrLTEStackDomain, Resmgr_ResourceCfg* ptrL2ResourceCfg);

/**********************************************************************
 ************************** L2 Functions ******************************
 **********************************************************************/

uint32_t* ptrMemMonitor = NULL;
void debugMemMonitor (uint32_t id)
{
    if (ptrMemMonitor != NULL)
    {
        printf ("%p[%d]: 0x%08x 0x%08x 0x%08x 0x%08x\n", ptrMemMonitor,   id, *ptrMemMonitor,     *(ptrMemMonitor+1), *(ptrMemMonitor+2), *(ptrMemMonitor+3));
        printf ("%p[%d]: 0x%08x 0x%08x 0x%08x 0x%08x\n", ptrMemMonitor+4, id, *(ptrMemMonitor+4), *(ptrMemMonitor+5), *(ptrMemMonitor+6), *(ptrMemMonitor+7));
        printf ("%p[%d]: 0x%08x 0x%08x 0x%08x 0x%08x\n", ptrMemMonitor+8, id, *(ptrMemMonitor+8), *(ptrMemMonitor+9), *(ptrMemMonitor+10), *(ptrMemMonitor+11));
    }
}

/**
 *  @b Description
 *  @n
 *      This is the UINTC thread
 *
 *  @param[in]  arg
 *      Argument passed to the thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* UintcThread(void *arg)
{
    int32_t                 errCode;
    struct sched_param      param;
    UintcHandle*            ptrUintcHandle;
    int32_t                 retVal;

    /* Get the interrupt handle. */
    ptrUintcHandle = (UintcHandle*)arg;

    /* Set the configured policy and priority */
    param.sched_priority = 10;
    errCode = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (errCode != 0)
    {
        printf ("Error: Unable to set the UINTC thread priority & policy [Error Code %d]\n", errCode);
        return NULL;
    }

    /* Debug Message: */
    printf("Debug: UINTC execution thread started\n");

    /* Loop around: The UINTC thread is waiting for an interrupt to arrive on events which have been registered
     * with the UINTC instance. */
    while (1)
    {
        /* Dispatch received events to the appropriate handler. */
        retVal = Uintc_select (ptrUintcHandle, NULL, &errCode);
        if (retVal < 0)
        {
            /* Error: UINTC select failed. Has the UINTC module been deinitialized? */
            if (errCode == UINTC_EDEINIT)
                break;

            /* Report the UINTC Module error: */
            printf ("Error: UINTC select failed [Error code %d]\n", errCode);
            return NULL;
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Root Task which executes and is listening for commands from the root
 *      master
 *
 *  @retval
 *      Not Applicable.
 */
static void* L2_initTask(void *arg)
{
    pthread_t               uintcThread;
    uint32_t*               memory;
    AppLTEStackDomainMCB*   ptrLTEStackDomain;
    Domain_SyslibCfg        syslibDomainCfg;
    UintcConfig             uintcConfig;
    int32_t                 errCode;
    Resmgr_ResourceCfg      domainResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        0,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        1,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                       Linking RAM,                           Num,      Size */
        { "Domain-Sylib",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  2048,     128},
        }
    };
    Resmgr_ResourceCfg      l2ResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                                         */
        1,    /* Number of Accumulator Channels requested FP) ]                             */
        0,    /* Number of Hardware Semaphores requested                                    */
        6,    /* Number of QPEND Queues requested [2 (L1A) + 2 (L1B) ]                      */
        /* Requested Memory Region Configuration.                                           */
        {
            /* Name,           Type,                       Linking RAM,                           Num,      Size */
            { "L2-MemRegion",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  4096,     128},
            { "FAPI-Memlog",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  4096,     128},
        }
    };

    /* Get the pointer to the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg;

    /* Initialize the UINTC module: This will allow the L2 application to manage interrupts  */
    sprintf(uintcConfig.name, "%s_FAPI", syslibDomainCfg.rootSyslibCfg.name);
    uintcConfig.mode                  = Uintc_Mode_APPLICATION_MANAGED;
    ptrLTEStackDomain->fapiUintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (ptrLTEStackDomain->fapiUintcHandle == NULL)
    {
        printf ("Error: Unable to open the UINTC module for FAPI channels[Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: UINTC module for FAPI channels has been opened successfully.\n");

    /* Initialize the UINTC module: This will allow the L2 application to manage interrupts  */
    strncpy(uintcConfig.name, syslibDomainCfg.rootSyslibCfg.name, UINTC_MAX_CHAR);
    uintcConfig.mode                  = Uintc_Mode_UINTC_MANAGED;
    ptrLTEStackDomain->appUintcHandle = Uintc_init (&uintcConfig, &errCode);
    if (ptrLTEStackDomain->appUintcHandle == NULL)
    {
        printf ("Error: Unable to open the UINTC module [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: UINTC module has been opened successfully.\n");

    /* Create the UINTC thread: This thread will handle all the interrupts in this thread context and will
     * then dispatch it appropriately to the all the users of the interrupts. */
    errCode = pthread_create (&uintcThread, NULL, UintcThread, ptrLTEStackDomain->appUintcHandle);
    if (errCode < 0)
    {
        printf ("Error: UINTC thread create failed error code %d\n", errCode);
        return NULL;
    }

    /* Populate the SYSLIB Domain configuration: */
    memcpy ((void *)&syslibDomainCfg.rootSyslibCfg, (void *)&ptrLTEStackDomain->rootSyslibCfg, sizeof(Root_SyslibConfig));
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

    /* Populate the DOMAIN Name Database OSAL function table: This is only needed on the DSP */
    syslibDomainCfg.nameDBOsalFxnTable.malloc             	= NULL;
    syslibDomainCfg.nameDBOsalFxnTable.free               	= NULL;
    syslibDomainCfg.nameDBOsalFxnTable.enterCS            	= NULL;
    syslibDomainCfg.nameDBOsalFxnTable.exitCS             	= NULL;
    syslibDomainCfg.nameDBOsalFxnTable.beginMemAccess     	= NULL;
    syslibDomainCfg.nameDBOsalFxnTable.endMemAccess       	= NULL;

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
    syslibDomainCfg.pktlibOsalFxnTable.phyToVirt            = Pktlib_osalPhyToVirt;

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
    ptrLTEStackDomain->syslibHandle = Domain_initSyslibServices (ptrLTEStackDomain->appId, &syslibDomainCfg, &errCode);
    if (ptrLTEStackDomain->syslibHandle == NULL)
    {
        printf ("FATAL Error: Domain initialization failed [Error code %d]\n", errCode);
        return NULL;
    }

    /* Process the Resource Manager configuration for L2 */
    if (Resmgr_processConfig (Domain_getSysCfgHandle (ptrLTEStackDomain->syslibHandle), &l2ResourceCfg, &errCode) < 0)
    {
        printf ("FATAL Error: Resource configuration resource allocation failure [Error code %d]\n", errCode);
        return NULL;
    }

    /* Setup IPv4 environment for L2 UserPlane Data Path */
    if (L2_UserPlane_SetupIPv4Env (ptrLTEStackDomain, &l2ResourceCfg) < 0)
    {
        printf ("Error: Failed setting up IPv4 environment for CP/UP for appId: %d\n",
                        ptrLTEStackDomain->appId);
        return NULL;
    }
    System_printf ("Debug: L2 IPv4 User Plane setup done.\n");

    {
        Ti_Pkt*                 ptrPkt;
        Pktlib_HeapHandle       heapHandle;
        char                    heapName[PKTLIB_MAX_CHAR];

        /* Construct the heap name */
        sprintf (heapName, "NETFP_ClientHeaderHeap_%x", ptrLTEStackDomain->appId);
        heapHandle = Pktlib_findHeapByName (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle), heapName, &errCode);
        if (heapHandle == NULL)
        {
            System_printf ("Error: Unable to find the NETFP client heap\n");
            return NULL;
        }

        /* Allocate a packet from the heap which is getting corrupted */
        ptrPkt = Pktlib_allocPacket (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle), heapHandle, 64);
        memory = (uint32_t*)ptrPkt;
        Pktlib_freePacket(Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle), ptrPkt);
    }

    /* Display the memory: */
    ptrMemMonitor = memory;
    debugMemMonitor (1);

    /* Setup the Fast Paths required for FAPI instances to be operational */
    if (Fapi_setupIPv4Env (ptrLTEStackDomain) < 0)
    {
        printf ("Error: FAPI IPv4 environment setup failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: FAPI IPv4 FastPaths initialized successfully\n");

    /* Setup the Fast Paths required for FAPI instances to be operational */
    if (Fapi_setupIPv6Env (ptrLTEStackDomain) < 0)
    {
        printf ("Error: FAPI IPv6 environment setup failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: FAPI IPv6 FastPaths initialized successfully\n");

    /*******************************************************************************
     * Initialize the FAPI L1A component
     *******************************************************************************/
    ptrLTEStackDomain->fapiL1AHandle = Fapi_initComponent(ptrLTEStackDomain, 'A', &l2ResourceCfg, &errCode);
    if (ptrLTEStackDomain->fapiL1AHandle == NULL)
    {
        printf ("Error: FAPI Component initialization failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: FAPI Component for L1A initialized successfully\n");

    /*******************************************************************************
     * Initialize the FAPI L1B component
     *******************************************************************************/
    ptrLTEStackDomain->fapiL1BHandle = Fapi_initComponent(ptrLTEStackDomain, 'B', &l2ResourceCfg, &errCode);
    if (ptrLTEStackDomain->fapiL1BHandle == NULL)
    {
        printf ("Error: FAPI Component initialization failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: FAPI Component for L1B initialized successfully\n");

    /* Initialize the Logging infrastructure: This is done only if the DAT clients are instantiated for the domain */
    if (ptrLTEStackDomain->rootSyslibCfg.datClientConfig.instantiateDatClient == 1)
    {
        if (Log_initLogging(ptrLTEStackDomain, &l2ResourceCfg) < 0)
        {
            System_printf ("Error: L2 logging setup failed\n");
            return NULL;
        }
    }

    /*******************************************************************************
     * Application Developers: Please add any *important* application initialization
     * if any which need to be done here. We dont want to announce that the domain
     * is up till all the basic services are ready.
     *******************************************************************************/


    /*******************************************************************************
     * Application Developers: Announce to the master that the domain is operational
     *******************************************************************************/
    if (Root_appUp (rootSlaveHandle, ptrLTEStackDomain->appId, (void*)ptrLTEStackDomain, &errCode) < 0)
    {
        printf ("Error: Root application UP failed [Error code %d]\n", errCode);
        return NULL;
    }
    printf ("Debug: Application is operational\n");

    /*******************************************************************************
     * Application Developers: Do your work here.
     * - This is where we can start initializing and starting the application components
     *   As an example: The L2 FAPI component is started here
     *******************************************************************************/
    Fapi_initPhy(ptrLTEStackDomain->fapiL1AHandle);
    Fapi_initPhy(ptrLTEStackDomain->fapiL1BHandle);

    while (1)
        sleep(1);
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
 *      Pointer to the SYSLIB configuration passed by the root master
 *
 *  @retval
 *      Not Applicable.
 */
void appInit (uint32_t appId, void* ptrDomainCfg, Root_SyslibConfig* ptrRootSyslibCfg)
{
    AppLTEStackDomainMCB*   ptrLTEStackDomain;
    int32_t                 errCode;

    /* Debug Message: */
    printf ("Debug: Application Initialization %x on ARM process %d starting\n", appId, getpid());

    /* Initialize the application domain pointer */
    ptrLTEStackDomain = &myAppDomain;

    /* Initialize the allocated memory block. */
    memset ((void *)ptrLTEStackDomain, 0, sizeof(AppLTEStackDomainMCB));

    /* Populate the LTE STACK Domain */
    ptrLTEStackDomain->appId = appId;

    /* Copy the SYSLIB configuration: */
    memcpy ((void *)&ptrLTEStackDomain->rootSyslibCfg, (void*)ptrRootSyslibCfg, sizeof(Root_SyslibConfig));

    /* Launch the initialization thread */
    errCode = pthread_create (&ptrLTEStackDomain->initThread, NULL, L2_initTask, ptrLTEStackDomain);
    if (errCode < 0)
    {
        printf ("Error: Unable to create the root slave thread [Error code %d]\n", errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      L2 Deinitialization thread responsible for shutting down the application
 *
 *  @retval
 *      Not Applicable.
 */
static void* L2_deinitThread(void *arg)
{
    AppLTEStackDomainMCB*       ptrLTEStackDomain;
    int32_t                     errCode;

    /* Get the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg;

    /*******************************************************************************
     * Application Developers: Please add application specific cleanup here.
     *
     * Example:
     *  Shutdown and deinitialize the L2 UserPlane
     *  Shutdown and deinitialize the FAPI interface
     *  Shutdown all the application created tasks
     *******************************************************************************/
    L2_UserPlane_DeInitIPv4Env (ptrLTEStackDomain);
    Fapi_deInitIPv4Env (ptrLTEStackDomain);
    Fapi_deInitIPv6Env (ptrLTEStackDomain);
    if (ptrLTEStackDomain->fapiL1AHandle != NULL)
        Fapi_deinitComponent (ptrLTEStackDomain->fapiL1AHandle);
    if (ptrLTEStackDomain->fapiL1BHandle != NULL)
        Fapi_deinitComponent (ptrLTEStackDomain->fapiL1BHandle);

    /* Shutdown the logging infrastructure. */
    if (ptrLTEStackDomain->rootSyslibCfg.datClientConfig.instantiateDatClient == 1)
        Log_deinitLogging (ptrLTEStackDomain);

    /* Cancel the application threads. */
    pthread_cancel (ptrLTEStackDomain->initThread);

    /* Cancel the consumer threads. */
    pthread_cancel (ptrLTEStackDomain->consumerThread);

    /* Shutdown the domain */
    if (Domain_deinitSyslibServices (ptrLTEStackDomain->syslibHandle, &errCode) < 0)
    {
        printf ("FATAL Error: Shutting down the application domain failed [Error code %d]\n", errCode);
        return NULL;
    }

    /* Shutdown the UINTC module. */
    if (Uintc_deinit (ptrLTEStackDomain->appUintcHandle, &errCode) < 0)
        printf ("Error: UINTC deinitialization failed [Error code %d]\n", errCode);

    /* Shutdown the UINTC module. */
    if (Uintc_deinit (ptrLTEStackDomain->fapiUintcHandle, &errCode) < 0)
        printf ("Error: UINTC deinitialization failed  for FAPI uintc channels[Error code %d]\n", errCode);

    /* Inform the root master that the domain has been completely deinitialized. */
    if (Root_appDeinitialized (rootSlaveHandle, ptrLTEStackDomain->appId, &errCode) < 0)
        printf ("Error: Root deinitialization failed [Error code %d]\n", errCode);

    return NULL;
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
    pthread_t   l2DeinitTask;
    int32_t     errCode;

    /* Create the L2 Deinitalization thread: This thread will shutdown the L2 thread. */
    errCode = pthread_create (&l2DeinitTask, NULL, L2_deinitThread, appDomainHandle);
    if (errCode < 0)
        printf ("Error: UINTC thread create failed error code %d\n", errCode);
}

