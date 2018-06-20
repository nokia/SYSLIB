/**
 *   @file  l2_dp.c
 *
 *   @brief
 *      Sample Layer2 LTE Data plane stack code.
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
#include <ti/sysbios/knl/Clock.h>

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
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include "l2_lte.h"

/**********************************************************************
 ************************* Global Declarations ************************
 **********************************************************************/

/* LTE Stack Domain MCB */
AppLTEStackDomainMCB                            myAppDomain;

/* eNodeB IPv4 Address. Used for GTPU Fast path creation */
uint8_t eNodeBIPAddress[4] = {192, 168, 1, 2};

/* PDN Gw IPv4 Address. Used for GTPU Fast path creation */
uint8_t pdnGwIPAddress[4] = {192, 168, 1, 1};

/* Logger IPv4 Address. Used for FAPI Fast path creation */
uint8_t loggerIPAddress[4] = {192, 168, 1, 1};

/* Reassembly timeout in milliseconds */
#define REASSEMBLY_TIMER_PERIOD_MSEC            500

/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/

/* ROOT Slave Handle */
extern Root_SlaveHandle      rootSlaveHandle;

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

/**********************************************************************
 *********************** Application Functions ************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory which is to be allocated.
 *  @param[in]  arg
 *      FAPI Module specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* L2_DDR3MemoryMalloc(uint32_t size, uint32_t arg)
{
    Error_Block	errorBlock;

    /* Allocate memory from the shared memory heap. */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)arg, size, 0, &errorBlock);
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
 *      FAPI Module specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void L2_DDR3MemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    /* Cleanup the memory block. */
    Memory_free ((xdc_runtime_IHeap_Handle)arg, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      Msgcom buffer cleanup handler.
 *
 *  @param[in]  pktlibInstHandle
 *      Pktlib instance handle
 *  @param[in]  chHandle
 *      Msgcom channel handle
 *  @param[in]  msgBuffer
 *      Descriptor to be freed up
 *
 *  @retval
 *      Not Applicable.
 */
static void L2_MsgcomFree(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    /* Free the packet */
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

/**
 *  @b Description
 *  @n
 *      Processes the GTPU packets received posted on GTPU channel.
 *      Loops back the packets to the sender.
 *
 *  @param[in]  ptrLTEStackDomain
 *      Domain Info Handle
 *
 *  @retval
 *      None
 */
static void L2_ProcessGTPUPacket (AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    Ti_Pkt*     ptrRxPkt;
	uint8_t*    ptrRxDataBuffer;
    uint32_t    rxDataBufferLen;
    int32_t     retVal, errCode;

    while (1)
    {
        /* Block on the GTP-U Rx channel waiting for a packet */
        Msgcom_getMessage (ptrLTEStackDomain->gtpuChannelHandle, (MsgCom_Buffer**)&ptrRxPkt);

        /* Received a packet. Process it. */
        if (ptrRxPkt != NULL)
        {
            /* Increment the GTPU Rx counter */
            ptrLTEStackDomain->gtpuRxPktCtr ++;

            /* Get the GTPU payload from the packet received */
            Pktlib_getDataBuffer (ptrRxPkt, &ptrRxDataBuffer, &rxDataBufferLen);

            /* Echo back the packet to sender */
            retVal = Netfp_send (ptrLTEStackDomain->l2DRBChannel, ptrRxPkt, 0x0, &errCode);
            if (retVal < 0)
            {
                /* Error looping back Rx packet. Increment error stats */
                System_printf ("Error: Failed sending GTPU packet of size %d bytes [Error code %d]\n",
                                rxDataBufferLen, errCode);
                ptrLTEStackDomain->gtpuTxErrorPktCtr ++;
                return;
            }

            /* Successfully echoed back Rx packet. Increment success stats */
            ptrLTEStackDomain->gtpuTxPktCtr ++;
        }
        else
        {
            /* Done handling all Rx packets */
            break;
        }
    }

    /* Done processing the received packet */
    return;
}

/**
 *  @b Description
 *  @n
 *      Callback function registered with Msgcom. Called when a
 *      GTPU packet is posted by NetCP to the GTPU msgcom channel.
 *
 *  @param[in]  gtpuChHandle
 *      GTPU channel handle
 *
 *  @param[in]  ptrLTEStackDomain
 *      Domain Info Handle
 *
 *  @retval
 *      None
 */
static void L2_UserPlaneRxCallback (MsgCom_ChHandle gtpuChHandle, AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    /* Post an GTPU Rx packet event */
    Event_post (ptrLTEStackDomain->netfpEventHnd, Event_Id_00);
}

/**
 *  @b Description
 *  @n
 *      Callback function invoked whenever an inner IP/outer IP
 *      fragment is received by NetCP. Triggers NetFP S/w assisted
 *      reassembly.
 *
 *  @param[in]  chHandle
 *      Inner/Outer fragment channel handle
 *
 *  @param[in]  ptrLTEStackDomain
 *      Domain Info Handle
 *
 *  @retval
 *      None
 */
static void L2_FragRxCallback (MsgCom_ChHandle chHandle, AppLTEStackDomainMCB* ptrLTEStackDomain)
{
    /* Post an IP Fragment Rx event */
    Event_post (ptrLTEStackDomain->netfpEventHnd, Event_Id_01);
}

/**
 *  @b Description
 *  @n
 *      Timer based function invoked whenever the reassembly
 *      timeout is met. Posts a reassembly timeout event.
 *
 *  @retval
 *      None
 */
static void L2_ReassTimeoutHandler (void)
{
    /* Post an Reassembly timeout event */
    Event_post (myAppDomain.netfpEventHnd, Event_Id_02);
}

/**
 *  @b Description
 *  @n
 *      L2 UserPlane Management Task. Handles the following:
 *          - GTPU Rx (Event Id 0)
 *          - Fragment Rx (Event Id 1)
 *          - Reassembly Timeout (Event Id 2)
 *
 *  @retval
 *      Not Applicable.
 */
static void L2_UserPlaneMgmtTask (UArg arg0, UArg arg1)
{
    uint32_t                    events;
    int32_t                     retVal, errCode;
    AppLTEStackDomainMCB*       ptrLTEStackDomain;

    /* Get the domain info handle */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg0;

    while (1)
    {
        /* Wait for ANY of the events to be posted */
        events = Event_pend(ptrLTEStackDomain->netfpEventHnd,
                            Event_Id_NONE,
                            Event_Id_00 + Event_Id_01 + Event_Id_02,
                            BIOS_WAIT_FOREVER);

        /* GTPU packet has been received. Process it */
        if (events & Event_Id_00)
            L2_ProcessGTPUPacket (ptrLTEStackDomain);

        /* Fragmented packet has been received; execute the reassembly operation. */
        if (events & Event_Id_01)
        {
            /* Process all fragments received */
            retVal = Netfp_reassembly (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                       &errCode);

            /* NETFP Client: Reassembly Processing failed.  */
            if (retVal < 0)
            {
                System_printf ("Error: Netfp Reassembly returned error [Error code %d]\n", errCode);
            }
        }

        /* Handle the reassembly timeout? */
        if (events & Event_Id_02)
        {
            /* Reassembly Timer: Checks for any stale fragments and clean up */
            if (Netfp_reassemblyTimerTick (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                           REASSEMBLY_TIMER_PERIOD_MSEC,
                                           &errCode) < 0)
            {
                /* NETFP Client: Reassembly Timer Processing failed.  */
                System_printf ("Error: NETFP Reassembly Timer Tick failed [Error code %d]\n", errCode);
            }
        }

#if 0
        /* Process the IPSec lifetime timer expiry */
        if (events & Event_Id_03)
            Test_reportExpiredSA();
#endif
    }
}


/**
 *  @b Description
 *  @n
 *      Initializes NetFP reassembly module.
 *
 *  @param[in]  ptrLTEStackDomain
 *      LTE stack domain configuration.
 *  @param[in]  ptrL2ResourceCfg
 *      L2 Resource configuration
 *
 *  @retval
 *      <0      -   Error initializing NetFP Reassembly
 *      0       -   Succesfully setup NetFP for S/w assisted
 *                  reassembly.
 */
static int32_t L2_InitReassembly
(
    AppLTEStackDomainMCB*           ptrLTEStackDomain,
    Resmgr_ResourceCfg*             ptrL2ResourceCfg
)
{
    int32_t                         errCode;
    Netfp_ReassemblyConfig          reassemblyConfig;
    Netfp_DefaultReassemblyMgmtCfg  defaultReassemblyCfg;
    Netfp_ClientHandle              netfpClientHandle;
    Msgcom_ChannelCfg               chConfig;
    Clock_Params                    clockParams;
    Clock_Handle                    reassemblyTimer;

    /* NETFP Client need to be instantiated to use the reassembly service. */
    netfpClientHandle = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (netfpClientHandle == NULL)
    {
        System_printf ("Error: NetFP client has not been instantiated for 0x%x\n", ptrLTEStackDomain->appId);
        return -1;
    }

    /* Initialize the channel configuration. */
	memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                        = (MsgCom_AppCallBack)L2_FragRxCallback;
    chConfig.arg                                                = (uint32_t)ptrLTEStackDomain;
    chConfig.msgcomInstHandle                                   = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);
    chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = ptrL2ResourceCfg->accChannelResponse[1].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = ptrL2ResourceCfg->accChannelResponse[1].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = ptrL2ResourceCfg->accChannelResponse[1].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = ptrL2ResourceCfg->accChannelResponse[1].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 1;

    /* Outer-IP reassembly channel: */
    ptrLTEStackDomain->outerIPChannelHandle = Msgcom_create ("OuterIP-Reassembly-Channel",
                                                             Msgcom_ChannelType_QUEUE,
                                                             &chConfig,
                                                             &errCode);
	if ( ptrLTEStackDomain->outerIPChannelHandle == NULL)
	{
        System_printf ("Error: Unable to open the Msgcom channel for Outer IP fragments [Error code %d]\n", errCode);
        return -1;
    }

	/* Initialize the channel configuration. */
	memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                        = (MsgCom_AppCallBack)L2_FragRxCallback;
    chConfig.arg                                                = (uint32_t)ptrLTEStackDomain;
    chConfig.msgcomInstHandle                                   = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);
    chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = ptrL2ResourceCfg->accChannelResponse[2].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = ptrL2ResourceCfg->accChannelResponse[2].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = ptrL2ResourceCfg->accChannelResponse[2].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = ptrL2ResourceCfg->accChannelResponse[2].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 1;

	/* Inner IP reassembly channel: */
    ptrLTEStackDomain->innerIPChannelHandle = Msgcom_create ("InnerIP-Reassembly-Channel",
                                                            Msgcom_ChannelType_QUEUE,
                                                            &chConfig,
                                                            &errCode);
	if (ptrLTEStackDomain->innerIPChannelHandle == NULL)
	{
        System_printf ("Error: Unable to open the Msgcom channel for Inner IP fragments [Error code %d]\n", errCode);
        return -1;
    }

	/* Initialize the channel configuration. */
	memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration: */
	chConfig.mode                           = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.msgcomInstHandle               = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);
	chConfig.appCallBack                    = NULL;
	chConfig.u.queueCfg.interruptMode       = Msgcom_QueueInterruptMode_NO_INTERRUPT;

	/* Large Packet Channel: This is the channel where the packets > NETFP_PASS_MAX_BUFFER_SIZE will be received
	 * since the NETCP cannot handle these packets. */
    ptrLTEStackDomain->largeChannelHandle = Msgcom_create ("Large-Channel", Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
	if (ptrLTEStackDomain->largeChannelHandle == NULL)
	{
        System_printf ("Error: Unable to open the Msgcom channel for Large packets [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the reassembly configuration */
	memset ((void*)&reassemblyConfig, 0, sizeof(Netfp_ReassemblyConfig));

	/* Initialize the default reassembly configuration. */
	memset ((void*)&defaultReassemblyCfg, 0, sizeof(Netfp_DefaultReassemblyMgmtCfg));

    /* Populate the default reassembly configuration: */
	defaultReassemblyCfg.bufferThresholdUpper = 20000;
    defaultReassemblyCfg.bufferThresholdLower = 18000;

	/* Populate the reassembly configuration:
     *  - Memory region 1 allocated for the reassembly operations.
	 *  - The reassembly inner, outer and large channels should be *WRITER* channels which
	 *    are always found. */
    reassemblyConfig.reassemblyMemRegion    = ptrL2ResourceCfg->memRegionResponse[0].memRegionHandle;
    reassemblyConfig.numFragmentedPkt       = 64;
	reassemblyConfig.numReassemblyContexts  = 128;
	reassemblyConfig.reassemblyTimeout      = 30;               /* Unit - Seconds */
	reassemblyConfig.outerIpReassemChannel  = ptrLTEStackDomain->outerIPChannelHandle;
	reassemblyConfig.innerIpReassemChannel  = ptrLTEStackDomain->innerIPChannelHandle;
    reassemblyConfig.largePacketChannel     = ptrLTEStackDomain->largeChannelHandle;
    reassemblyConfig.reassemblyHeapArg      = (uint32_t)ptrLTEStackDomain->ddr3SharedHeapHandle;
    reassemblyConfig.reassemblyHeapAlloc    = L2_DDR3MemoryMalloc;
    reassemblyConfig.reassemblyHeapFree     = L2_DDR3MemoryFree;
    reassemblyConfig.reassemblyMgmt         = NULL;
    reassemblyConfig.ptrReassemblyMgmtCfg   = &defaultReassemblyCfg;
    reassemblyConfig.sizeReassemblyCfg      = sizeof(defaultReassemblyCfg);

    /* Register the client to handle the reassembly for all fragmented packets */
	if (Netfp_registerReassemblyService (netfpClientHandle, &reassemblyConfig, &errCode) < 0)
	{
	    /* Error: Failed to initialize the NETFP reassembly */
        System_printf ("Error: NETFP Reassembly initialization failed\n");
        return -1;
    }

    /* Initialize the Clock Parameters */
    Clock_Params_init(&clockParams);
    clockParams.period      = REASSEMBLY_TIMER_PERIOD_MSEC;     /* Unit - milliseconds */
    clockParams.startFlag   = TRUE;

    /* Instantiate a clock to check for reassembly timeouts */
    reassemblyTimer = Clock_create ((Clock_FuncPtr)L2_ReassTimeoutHandler,
                                    REASSEMBLY_TIMER_PERIOD_MSEC,
                                    &clockParams,
                                    NULL);
    if (reassemblyTimer == NULL)
    {
        System_printf ("Error: Reassembly Timer creation failed.\n");
        return -1;
    }
    System_printf ("Debug: NetFP Reassembly registration succesful.\n");

    /* Done setting up reassembly operations */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to synchronize with the RAT database until the specific
 *      policy name has been added.
 *
 *  @param[in]  nameClientHandle
 *      Name Client Handle which is used to access the remote RAT database
 *  @param[in]  nrInstanceId
 *      Named resource identifier for the RAT database
 *  @param[in]  policyName
 *      Name of the policy
 *  @param[out] errCode
 *      Error code populated on error.
 *
 *  @retval
 *      Error   -   <0
 *  @retval
 *      Success -   Policy identifier
 */
static int32_t L2_SyncPolicyId
(
    Name_ClientHandle nameClientHandle,
    uint32_t          nrInstanceId,
    char*             policyName,
    int32_t*          errCode
)
{
    Name_ResourceCfg    nrConfig;

    /* Find the policy in the remote database. */
    while (1)
    {
        if (Name_get (nameClientHandle, nrInstanceId, Name_ResourceBucket_USER_DEF1,
                      policyName, &nrConfig, errCode) == 0)
        {
            /* Policy has been created in the remote database. */
            break;
        }

        /* Error finding the policy. Check the error */
        if (*errCode != NAME_ENOTFOUND)
        {
            System_printf ("Error: Synchronizing Policy '%s' failed [Error code %d]\n", policyName, *errCode);
            return -1;
        }

        /* Policy not found. Wait for sometime and retry */
        Task_sleep (1);
    }
    return (uint32_t)nrConfig.handle1;
}

/**
 *  @b Description
 *  @n
 *      Sets up SYSLIB environment for L2 stack's IPv4 user plane.
 *
 *  @param[in]  ptrLTEStackDomain
 *      LTE stack domain configuration.
 *  @param[in]  l2Id
 *      L2 Identifier
 *  @param[in]  ingressSPID
 *      Ingress SP identifier
 *  @param[in]  egressSPID
 *      Egress SP identifier
 *
 *  @retval
 *      Error       -   <0
 *  @retval
 *      Success     -   0
 */
static int32_t L2_UserPlane_SetupIPv4Env
(
    AppLTEStackDomainMCB*   ptrLTEStackDomain,
    char                    l2Id,
    uint32_t                ingressSPID,
    uint32_t                egressSPID
)
{
    Netfp_ClientHandle          netfpClientHandle;
    Name_ClientHandle           nameClientHandle;
    int32_t                     errCode, i;
    Netfp_InboundFPCfg          inboundFPCfg;
    Netfp_OutboundFPCfg         outboundFPCfg;
    Pktlib_HeapCfg              heapConfig;
    Netfp_FlowCfg               flowConfig;
    Msgcom_ChannelCfg           chConfig;
    Netfp_UserCfg               userConfig;
    uint8_t                     encryptionKey[16];
    uint8_t                     integrityKey[16];
    uint32_t                    index;
    Netfp_LTEChannelBindCfg     bindConfig;
    Netfp_LTEChannelConnectCfg  connectConfig;
    uint32_t                    ueId = 0x12 + DNUM;
    uint32_t                    rbId = 0x03;
    uint32_t                    gtpuId = 0xdead12 + DNUM;
    char                        channelName[MSG_COM_MAX_CHANNEL_LEN];
    Error_Block                 eb;
    Task_Params                 taskParams;
    Resmgr_ResourceCfg          l2ResourceCfg =
    {
        0,    /* Number of CPINTC Output  requested                               */
        3,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        0,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
            /* Name,           Type,                       Linking RAM,                           Num,      Size */
            { "Domain-L2-UserPlane",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  1024,     128},
        }
    };

    /* Get various SYSLIB handles required for UserPlane operation */
    netfpClientHandle = Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (netfpClientHandle == NULL)
    {
        System_printf ("Error: NETFP client has not been instantiated; cannot use the data plane services\n");
        return -1;
    }
    nameClientHandle = Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle);
    if (nameClientHandle == NULL)
    {
        System_printf ("Error: Name client handle services are required for the data plane services\n");
        return -1;
    }

    /* Process the Resource Manager configuration for L2 */
    if (Resmgr_processConfig (Domain_getSysCfgHandle (ptrLTEStackDomain->syslibHandle), &l2ResourceCfg, &errCode) < 0)
    {
        System_printf ("Error: ResMgr resource allocation failure [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the inbound fast path configuration for the UP.  */
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));
    snprintf (inboundFPCfg.name, NETFP_MAX_CHAR, "Ingress_FastPath_IPv4_UP_%c", l2Id);

    /* Initialize spidMode */
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else if (ingressSPID == 0)
        inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    inboundFPCfg.spId                    = ingressSPID;
    inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];
    inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = pdnGwIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = pdnGwIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = pdnGwIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = pdnGwIPAddress[3];

    /* Create the Inbound Fast Path: */
    ptrLTEStackDomain->gtpuIngressFPHandle = Netfp_createInboundFastPath (netfpClientHandle, &inboundFPCfg, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFPHandle == NULL)
    {
        /* Error creating UP Ingress Fastpath */
        System_printf ("Error: Failed to create UserPlane IPv4 Ingress Fastpath [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the egress IPv4 fast path for UP */
    memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));
    snprintf (outboundFPCfg.name, NETFP_MAX_CHAR, "Egress_FastPath_IPv4_UP_%c", l2Id);
    outboundFPCfg.spId                    = egressSPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = pdnGwIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = pdnGwIPAddress[1];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = pdnGwIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = pdnGwIPAddress[3];
    outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];

    /* Setup QCI-DSCP mapping for egress fast path */
    for (i = 0; i < 64; i ++)
	    outboundFPCfg.dscpMapping[i] = i;

    /* Setup the egress fast path: */
    ptrLTEStackDomain->gtpuEgressFPHandle = Netfp_createOutboundFastPath (netfpClientHandle, &outboundFPCfg, &errCode);
    if (ptrLTEStackDomain->gtpuEgressFPHandle == NULL)
    {
        /* Error creating UP Egress Fastpath */
        System_printf ("Error: Failed to create UserPlane IPv4 Egress Fastpath [Error code %d]\n", errCode);
        return -1;
    }

    /* Loop around till the fast path is active: */
    while (1)
    {
        int32_t status;
        if (Netfp_isOutboundFastPathActive (netfpClientHandle, ptrLTEStackDomain->gtpuEgressFPHandle, &status, &errCode) < 0)
        {
            /* Error creating UP Egress Fastpath */
            System_printf ("Error: Getting outbound fast path status failed [Error code %d]\n", errCode);
            return -1;
        }
        if(status == 1)
            break;
        Task_sleep(1);
    }

    /* Create a PKTLIB Heap for User plane Rx data path */
    memset ((void *)&heapConfig, 0, sizeof(Pktlib_HeapCfg));
    snprintf (heapConfig.name, PKTLIB_MAX_CHAR, "L2_UP_Rx_Heap_%c", l2Id);
    heapConfig.pktlibInstHandle                = Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle);
    heapConfig.memRegion                       = l2ResourceCfg.memRegionResponse[0].memRegionHandle;
    heapConfig.sharedHeap                      = 0;
    heapConfig.useStarvationQueue              = 0;
    heapConfig.dataBufferSize                  = 10*1024;
    heapConfig.numPkts                         = 64;
    heapConfig.numZeroBufferPackets            = 0;
    heapConfig.dataBufferPktThreshold          = 0;
    heapConfig.zeroBufferPktThreshold          = 0;
    heapConfig.arg                             = (uint32_t)ptrLTEStackDomain->ddr3SharedHeapHandle;
    heapConfig.heapInterfaceTable.dataMalloc   = L2_DDR3MemoryMalloc;
    heapConfig.heapInterfaceTable.dataFree     = L2_DDR3MemoryFree;
    ptrLTEStackDomain->gtpuRxHeap = Pktlib_createHeap(&heapConfig, &errCode);
    if (ptrLTEStackDomain->gtpuRxHeap == NULL)
    {
        System_printf ("Error: Unable to create L2 UserPlane Rx heap [Error code %d]\n", errCode);
        return -1;
    }

    /* Setup an Ingress flow to receive UP packets from NetCP  */
    memset((void *)&flowConfig, 0, sizeof(Netfp_FlowCfg));
    flowConfig.numHeaps         =   1;
    flowConfig.heapHandle[0]    =   ptrLTEStackDomain->gtpuRxHeap;
    flowConfig.sopOffset        =   0;
    snprintf (flowConfig.name, NETFP_MAX_CHAR, "L2_UP_Flow_%c", l2Id);
    ptrLTEStackDomain->gtpuIngressFlowId = Netfp_createFlow (netfpClientHandle, &flowConfig, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFlowId < 0)
    {
        System_printf ("Error: Failed to create Ingress Flow for L2 UserPlane [Error code %d]\n", errCode);
        return -1;
    }

    /* Setup a Msgcom channel to receive UP Ingress (Uplink) packets from NetCP */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "L2_UP_IPv4_UL_Channel_%c", l2Id);
    chConfig.mode                                               = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.appCallBack                                        = (MsgCom_AppCallBack)L2_UserPlaneRxCallback;
    chConfig.arg                                                = (uint32_t)ptrLTEStackDomain;
    chConfig.msgcomInstHandle                                   = Domain_getMsgcomInstanceHandle(ptrLTEStackDomain->syslibHandle);
    chConfig.u.queueCfg.interruptMode                           = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type              = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel        = l2ResourceCfg.accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue          = l2ResourceCfg.accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId            = l2ResourceCfg.accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId       = l2ResourceCfg.accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries    = 50;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount  = 1; /* Unit - 25us */
    ptrLTEStackDomain->gtpuChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE, &chConfig, &errCode);
    if (ptrLTEStackDomain->gtpuChannelHandle == NULL)
    {
        System_printf ("Error: Failed to create Msgcom channel for receiving GTP-U packets [Error code %d]\n",
                        errCode);
        return -1;
    }

    /* Populate the user security configuration. */
    memset ((void *)&userConfig, 0, sizeof(Netfp_UserCfg));
    for (index = 0; index < 16; index++)
    {
        encryptionKey[index] = (index & 0xFF);
        integrityKey[index]  = (index & 0xFF);
    }
    userConfig.authMode         = Netfp_3gppAuthMode_EIA2;
    userConfig.srbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userConfig.drbCipherMode    = Netfp_3gppCipherMode_EEA2;
    userConfig.ueId             = ueId;
    userConfig.srbFlowId        = ptrLTEStackDomain->gtpuIngressFlowId;
    userConfig.initialCountC    = 0;
    userConfig.chSrb1Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb1Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb2Enc        = 0;    /* Set to NULL only because we are testing DRB's here */
    userConfig.chSrb2Dec        = 0;    /* Set to NULL only because we are testing DRB's here */
    memcpy ((void *)&userConfig.hKeyRrcInt[0],(void *)integrityKey,  sizeof(userConfig.hKeyRrcInt));
    memcpy ((void *)&userConfig.hKeyRrcEnc[0],(void *)encryptionKey, sizeof(userConfig.hKeyRrcEnc));
    memcpy ((void *)&userConfig.hKeyUpEnc[0], (void *)encryptionKey, sizeof(userConfig.hKeyUpEnc));

    /* Create the user */
    ptrLTEStackDomain->ueHandle = Netfp_createUser (netfpClientHandle, &userConfig, &errCode);
    if (ptrLTEStackDomain->ueHandle == NULL)
    {
        System_printf ("Error: User creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the channel bind configuration */
    memset ((void *)&bindConfig, 0, sizeof (Netfp_LTEChannelBindCfg));
    bindConfig.flowId          = ptrLTEStackDomain->gtpuIngressFlowId;
    bindConfig.notifyFunction  = NULL;
    bindConfig.chDrbRohc       = Msgcom_getInternalMsgQueueInfo (ptrLTEStackDomain->gtpuChannelHandle);
    bindConfig.fpHandle        = ptrLTEStackDomain->gtpuIngressFPHandle;
    bindConfig.sin_gtpuId      = gtpuId;
    bindConfig.countC          = 0;
    bindConfig.enableFastPath  = 0;
    bindConfig.chDrbEnc        = NULL;

    /* Populate the channel connect configuration */
    memset ((void *)&connectConfig, 0, sizeof (Netfp_LTEChannelConnectCfg));
    connectConfig.fpHandle       = ptrLTEStackDomain->gtpuEgressFPHandle;
    connectConfig.sin_gtpuId     = gtpuId;
    connectConfig.qci            = 3;
    connectConfig.dscp           = 0x22;
    connectConfig.flowId         = ptrLTEStackDomain->gtpuIngressFlowId;
    connectConfig.chDrbDec       = NULL;

    /* Create the DRB LTE channel */
    ptrLTEStackDomain->l2DRBChannel = Netfp_createLTEChannel (ptrLTEStackDomain->ueHandle,
                                                              rbId,
                                                              Netfp_SockFamily_AF_INET,
                                                              &bindConfig,
                                                              &connectConfig,
                                                              &errCode);
    if (ptrLTEStackDomain->l2DRBChannel == NULL)
    {
        System_printf ("Error: Failed to create the LTE channel for DRB: %d [Error code %d]\n",
                        rbId, errCode);
        return -1;
    }
    System_printf ("Debug: Succesfully created GTP-U channel with GTPU-Id: 0x%x\n", gtpuId);

    /* Create an Event object. */
    Error_init(&eb);
    ptrLTEStackDomain->netfpEventHnd = Event_create(NULL, &eb);
    if (ptrLTEStackDomain->netfpEventHnd == NULL)
    {
        System_printf ("Error: Netfp Event creation failed.\n");
        return -1;
    }

    /* Create the UserPlane Management Task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 32*1024;
    taskParams.priority  = 6;
    taskParams.arg0      = (UArg)ptrLTEStackDomain;
    ptrLTEStackDomain->l2UserPlaneTask = Task_create(L2_UserPlaneMgmtTask, &taskParams, NULL);

    /* Register for NetFP Reassembly services on this core. */
    if (L2_InitReassembly (ptrLTEStackDomain, &l2ResourceCfg) < 0)
    {
        System_printf ("Error: Failed registering for NetFP Reassembly services\n");
        return -1;
    }

    /* Done setting up the L2 Datapath  */
    return 0;
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
static void L2_initTask (UArg arg0, UArg arg1)
{
    AppLTEStackDomainMCB*       ptrLTEStackDomain;
    Domain_SyslibCfg            syslibDomainCfg;
    Netfp_InboundFPCfg          inboundFPCfg;
    Netfp_OutboundFPCfg         outboundFPCfg;
    char                        l2Id;
    uint32_t                    ingressSPID;
    uint32_t                    egressSPID;
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
	    	{ "Domain-Sylib",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  2048,     128},
	    }
    };

    /* Get the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg0;

    /* Initialize the SYSLIB domain configuration */
    memset ((void *)&syslibDomainCfg, 0, sizeof(Domain_SyslibCfg));

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

    /* Initialize the Domain */
    ptrLTEStackDomain->syslibHandle = Domain_initSyslibServices (ptrLTEStackDomain->appId, &syslibDomainCfg, &errCode);
    if (ptrLTEStackDomain->syslibHandle == NULL)
    {
        System_printf ("FATAL Error: Domain initialization failed [Error code %d]\n", errCode);
        return;
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
        System_printf ("Error: Root application UP failed [Error code %d]\n", errCode);
        return;
    }

    /*******************************************************************************
     * Application Developers: Do your work here.
     *******************************************************************************/

    /* Determine the L2 instance identifier. We use the DSP core number to determine this */
    l2Id = (DNUM == 0) ? 'A' : 'B';

    /* Find the ingress policy to use for the debug fast paths */
    ingressSPID = L2_SyncPolicyId (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                   ptrLTEStackDomain->rootSyslibCfg.nrInstanceId, "Ingress_SPID_IPv4_UP", &errCode);
    if ((int32_t)ingressSPID < 0)
        return;

    /* Find the egress policy to use for the debug fast paths */
    egressSPID = L2_SyncPolicyId (Domain_getNameClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                  ptrLTEStackDomain->rootSyslibCfg.nrInstanceId, "Egress_SPID_IPv4_UP", &errCode);
    if ((int32_t)egressSPID < 0)
        return;

    /******************************************************************************
     * Setup the debug fast paths which are used for all debug streaming which
     * includes FAPI tracing and DAT UIA log streaming.
     ******************************************************************************/
    memset ((void *)&inboundFPCfg, 0, sizeof (Netfp_InboundFPCfg));

    /* Populate the Fast Path configuration. */
    snprintf (inboundFPCfg.name, NETFP_MAX_CHAR, "Debug-Tracing-Ingress_%c", l2Id);

    /* Initialize spidMode */
    if(ingressSPID == NETFP_INVALID_SPID)
        inboundFPCfg.spidMode = Netfp_SPIDMode_INVALID;
    else if (ingressSPID == 0)
        inboundFPCfg.spidMode = Netfp_SPIDMode_ANY_SECURE;
    else
        inboundFPCfg.spidMode = Netfp_SPIDMode_SPECIFIC;

    inboundFPCfg.spId			 = ingressSPID;
    inboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.srcIP.addr.ipv4.u.a8[0] = loggerIPAddress[0];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[1] = loggerIPAddress[1];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[2] = loggerIPAddress[2];
    inboundFPCfg.srcIP.addr.ipv4.u.a8[3] = loggerIPAddress[3];
    inboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
    inboundFPCfg.dstIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
    inboundFPCfg.dstIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];

    /* Debug Tracing ingress Fast Path: */
    ptrLTEStackDomain->debugTracingIngressFastPath = Netfp_createInboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                                  &inboundFPCfg, &errCode);
    if (ptrLTEStackDomain->debugTracingIngressFastPath == NULL)
    {
        System_printf ("Error: Unable to create Debug Tracing Ingress fast path [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Debug Tracing Ingress Fast Path Handle 0x%p\n", ptrLTEStackDomain->debugTracingIngressFastPath);

    /* Initialize the fast path configuration. */
	memset ((void *)&outboundFPCfg, 0, sizeof (Netfp_OutboundFPCfg));

    /* Populate the Fast Path configuration. */
    snprintf (outboundFPCfg.name, NETFP_MAX_CHAR, "Debug-Tracing-Egress_%c", l2Id);
    outboundFPCfg.spId					  = egressSPID;
    outboundFPCfg.dstIP.ver               = Netfp_IPVersion_IPV4;
	outboundFPCfg.srcIP.ver               = Netfp_IPVersion_IPV4;
    outboundFPCfg.srcIP.addr.ipv4.u.a8[0] = eNodeBIPAddress[0];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[1] = eNodeBIPAddress[1];
    outboundFPCfg.srcIP.addr.ipv4.u.a8[2] = eNodeBIPAddress[2];
	outboundFPCfg.srcIP.addr.ipv4.u.a8[3] = eNodeBIPAddress[3];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[0] = loggerIPAddress[0];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[1] = loggerIPAddress[1];
	outboundFPCfg.dstIP.addr.ipv4.u.a8[2] = loggerIPAddress[2];
    outboundFPCfg.dstIP.addr.ipv4.u.a8[3] = loggerIPAddress[3];

    /* Debug Tracing egress Fast Path: */
    ptrLTEStackDomain->debugTracingEgressFastPath = Netfp_createOutboundFastPath (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                                                                  &outboundFPCfg, &errCode);
    if (ptrLTEStackDomain->debugTracingEgressFastPath == NULL)
    {
        System_printf ("Error: Unable to create FAPI Tracing Egress fast path [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Debug Tracing Egress Fast Path Handle 0x%p\n", ptrLTEStackDomain->debugTracingEgressFastPath);

    /* Loop around till the outbound fast path is active: */
    while (1)
    {
        int32_t status;

        if (Netfp_isOutboundFastPathActive (Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                            ptrLTEStackDomain->debugTracingEgressFastPath, &status, &errCode) < 0)
        {
            System_printf ("Error: Getting the outbound status failed [Error code %d]\n", errCode);
            return;
        }
        if(status == 1)
            break;
        Task_sleep(1);
    }

    /* Setup the user plane IPv4 environment. */
    if (L2_UserPlane_SetupIPv4Env (ptrLTEStackDomain, l2Id, ingressSPID, egressSPID) < 0)
    {
        System_printf ("Error: Failed setting up IPv4 environment for CP/UP for appId: %d\n", ptrLTEStackDomain->appId);
        return;
    }
    System_printf ("Debug: L2 IPv4 User Plane setup done. \n");
    return;
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
    Task_Params                 taskParams;
    AppLTEStackDomainMCB*       ptrLTEStackDomain;

    /* Debug Message: */
    System_printf ("Debug: Application Initialization %x on Core %d starting\n", appId, DNUM);

    /* Enable the caches. */
	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

    /* Initialize the application domain pointer */
    ptrLTEStackDomain = &myAppDomain;

    /* Initialize the allocated memory block. */
    memset ((void *)ptrLTEStackDomain, 0, sizeof(AppLTEStackDomainMCB));

    /* Populate the LTE STACK Domain */
    ptrLTEStackDomain->appId                 = appId;
    ptrLTEStackDomain->localHeapHandle       = NULL;
    ptrLTEStackDomain->msmcSharedHeapHandle  = (xdc_runtime_IHeap_Handle)SharedRegion_getHeap(0);
    ptrLTEStackDomain->ddr3SharedHeapHandle  = (xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1);
    ptrLTEStackDomain->ddr3PrivateHeapHandle = (xdc_runtime_IHeap_Handle)ddr3PrivateHeap;

    /* Copy the SYSLIB configuration: */
    memcpy ((void *)&ptrLTEStackDomain->rootSyslibCfg, (void*)ptrRootSyslibCfg, sizeof(Root_SyslibConfig));

    /* Launch the L2 initialization task. The domain startup sequence has been initialized
     * and the initialization task has been spawned with the necessary configuration information.
     * The appInit is called in the context of the root and we wish to keep the code executing in
     * this context to be as minimal as possible. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 64*1024;
    taskParams.priority  = 4;
    taskParams.arg0      = (UArg)ptrLTEStackDomain;
    ptrLTEStackDomain->l2InitTaskHandle = Task_create((ti_sysbios_knl_Task_FuncPtr)L2_initTask, &taskParams, NULL);
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
static void L2_deinitTask (UArg arg0, UArg arg1)
{
    AppLTEStackDomainMCB*       ptrLTEStackDomain;
    int32_t                     errCode;

    /* Get the application domain. */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)arg0;

    /*******************************************************************************
     * Application Developers: Please add application specific cleanup here.
     * - Example: Shutdown all the application owned tasks
     *******************************************************************************/

    /* Dump the GTPU stats before we clean up */
    System_printf ("************** GTP-U Statistics **************\n");
    System_printf ("No. GTPU IPv4 Packets Received:         %d\n", ptrLTEStackDomain->gtpuRxPktCtr);
    System_printf ("No. GTPU IPv4 Packets Sent:             %d\n", ptrLTEStackDomain->gtpuTxPktCtr);
    System_printf ("No. GTPU IPv4 Send Errors:              %d\n", ptrLTEStackDomain->gtpuTxErrorPktCtr);

    /* Clean up all UserPlane related SYSLIB handles */
    if (ptrLTEStackDomain->netfpEventHnd != NULL)
        Event_delete(&ptrLTEStackDomain->netfpEventHnd);
    if (ptrLTEStackDomain->l2DRBChannel)
        Netfp_deleteLTEChannel (ptrLTEStackDomain->l2DRBChannel, &errCode);
    if (ptrLTEStackDomain->ueHandle)
        Netfp_deleteUser (ptrLTEStackDomain->ueHandle, &errCode);
    if (ptrLTEStackDomain->gtpuChannelHandle)
        Msgcom_delete (ptrLTEStackDomain->gtpuChannelHandle, &L2_MsgcomFree);
    if (ptrLTEStackDomain->gtpuEgressFPHandle)
        Netfp_deleteOutboundFastPath(Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                     ptrLTEStackDomain->gtpuEgressFPHandle, &errCode);
    if (ptrLTEStackDomain->gtpuIngressFPHandle)
        Netfp_deleteInboundFastPath(Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                    ptrLTEStackDomain->gtpuIngressFPHandle, &errCode);
    if (ptrLTEStackDomain->debugTracingIngressFastPath)
        Netfp_deleteInboundFastPath(Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                    ptrLTEStackDomain->debugTracingIngressFastPath, &errCode);
    if (ptrLTEStackDomain->debugTracingEgressFastPath)
        Netfp_deleteOutboundFastPath(Domain_getNetfpClientInstanceHandle(ptrLTEStackDomain->syslibHandle),
                                     ptrLTEStackDomain->debugTracingEgressFastPath, &errCode);
    if (ptrLTEStackDomain->outerIPChannelHandle)
        Msgcom_delete (ptrLTEStackDomain->outerIPChannelHandle, &L2_MsgcomFree);
    if (ptrLTEStackDomain->innerIPChannelHandle)
        Msgcom_delete (ptrLTEStackDomain->innerIPChannelHandle, &L2_MsgcomFree);
    if (ptrLTEStackDomain->largeChannelHandle)
        Msgcom_delete (ptrLTEStackDomain->largeChannelHandle, &L2_MsgcomFree);
    if (ptrLTEStackDomain->gtpuRxHeap)
        Pktlib_deleteHeap (Domain_getPktlibInstanceHandle(ptrLTEStackDomain->syslibHandle),
                           ptrLTEStackDomain->gtpuRxHeap, &errCode);
#if 0
    if (ptrLTEStackDomain->l2UserPlaneTask != NULL)
        Task_delete(&ptrLTEStackDomain->l2UserPlaneTask);
    if (ptrLTEStackDomain->l2InitTaskHandle != NULL)
        Task_delete(&ptrLTEStackDomain->l2InitTaskHandle);
#endif

    /* Shutdown the domain */
    if (Domain_deinitSyslibServices (ptrLTEStackDomain->syslibHandle, &errCode) < 0)
    {
        System_printf ("FATAL Error: Shutting down the application domain failed [Error code %d]\n", errCode);
        return;
    }

    /* Inform the root master that the domain has been completely deinitialized. */
    if (Root_appDeinitialized (rootSlaveHandle, ptrLTEStackDomain->appId, &errCode) < 0)
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
    AppLTEStackDomainMCB*       ptrLTEStackDomain;
    Task_Params                 taskParams;

    /* Initialize the application domain pointer */
    ptrLTEStackDomain = (AppLTEStackDomainMCB*)appDomainHandle;

    /* Debug Message: */
    System_printf ("Debug: Application Deinitialization %x on Core %d starting\n", ptrLTEStackDomain, DNUM);

    /* Launch the L2 deinitialization task. The domain cleanup sequence has been initialized and
     * the deinitialization task has been spawned. This function is called in the context of the root
     * task and we wish to preserve its integrity. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 8*1024;
    taskParams.priority  = 4;
    taskParams.arg0      = (UArg)ptrLTEStackDomain;
    Task_create((ti_sysbios_knl_Task_FuncPtr)L2_deinitTask, &taskParams, NULL);
}
