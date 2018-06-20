/*
 *   @file  fapi.c
 *
 *   @brief
 *      FAPI Template Implementation. This file implements the basic
 *      functionality which showcases the communication of FAPI messages
 *      between the PHY and L2.
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
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* IPC Include Files for Shared Memory Allocation. */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* Logger streamer Include Files */
#include <ti/uia/events/UIAEvt.h>
#include <ti/uia/runtime/LogUC.h>
#include <ti/uia/runtime/LogSnapshot.h>
#include <ti/uia/sysbios/LoggerStreamer2.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/platform/platform.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/domain/domain.h>

/* FAPI Interface */
#include <ti/demo/lte/common/ti_fapi.h>

/* Domain Include Files */
#include "sim_phy.h"

/**********************************************************************
 ************************** Local Structures **************************
 **********************************************************************/

/**
 * @brief
 *  FAPI States on the eNodeB
 *
 * @details
 *  The enumerating which describes the FAPI states
 */
typedef enum Fapi_State
{
    /**
     * @brief   Initial state of the FAPI
     */
    Fapi_State_INITIAL                   =   0x0,

    /**
     * @brief   State after PARAMETERS have been exchanged
     */
    Fapi_State_PARAM_EXCHANGED          =   0x1,

    /**
     * @brief   State after CELL Configuration has been done.
     */
    Fapi_State_CELL_CONFIG_RESPONSE      =   0x2,

    /**
     * @brief   State after which the PHY has been started.
     */
    Fapi_State_PHY_STARTED               =   0x3
}Fapi_State;

/**
 * @brief
 *  Master Control Block for the FAPI
 *
 * @details
 *  The structure holds all the information which is required for the FAPI
 *  interface to function.
 */
typedef struct Fapi_MCB
{
    /**
     * @brief   eNodeB FAPI State
     */
    Fapi_State            state;

    /**
     * @brief   Subframe numbers
     */
    uint16_t              sf;

    /**
     * @brief   Subframe numbers
     */
    uint16_t              sfn;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle       fapiL1L2HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to receive messages
     * from the L1
     */
    MsgCom_ChHandle       fapiL1L2LowPriorityChannel;

    /**
     * @brief   FAPI High Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle       fapiL2L1HighPriorityChannel;

    /**
     * @brief   FAPI Low Priority MSGCOM channel used to send messages
     * to the L1
     */
    MsgCom_ChHandle       fapiL2L1LowPriorityChannel;

    /**
     * @brief   FAPI Heap UL Shared Channel to be used.
     */
    Pktlib_HeapHandle     fapiUlSchDataHeapHandle;

    /**
     * @brief   FAPI MSMC Heap
     */
    Pktlib_HeapHandle     fapiMSMCHeapHandle;

    /**
     * @brief   FAPI DDR3 Heap
     */
    Pktlib_HeapHandle     fapiDDR3HeapHandle;

    /**
     * @brief   FAPI Event Object
     */
    Event_Handle          fapiEventHandle;

    /**
     * @brief   Simulated PHY Clock handle
     */
    Clock_Handle          clockHandle;

    /**
     * @brief   FAPI component task handle
     */
    Task_Handle           taskHandle;

    /**
     * @brief   PKTLIB Instance Handle associated with the domain
     */
    Pktlib_InstHandle     pktlibInstHandle;

    /**
     * @brief   MSGCOM Instance Handle associated with the domain
     */
    Msgcom_InstHandle     msgcomInstHandle;

    /**
     * @brief   Pointer to the application domain
     */
    AppSimPHYDomainMCB*   ptrSimPhyDomainMCB;
}Fapi_MCB;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Constant Param Response */
const uint8_t Fapi_paramResponse[] =
{
    0x00,0x00,0x00,0x00,0x07,0x00,0x3c,0x04,0x00,0x00,0x28,0x04,0x2c,0x00,0x29,0x04,
    0x2c,0x00,0x2a,0x04,0x07,0x00,0x2b,0x04,0x03,0x00,0x2c,0x04,0x02,0x00,0x63,0x04,
    0x03,0x01
};

/* Constant cell config response (This is just a template) */
const uint8_t Fapi_cellConfigResponse[] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/* Constant UL Config Request (This is just a template) */
const uint8_t Fapi_ulConfigRequest[] =
{
    0x33,0x25,0x00,0x00,0x00,0x00,0x00,0x00
};

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

/* Cache operations. */
extern void appInvalidateBuffer(void* ptr, uint32_t size);
extern void appWritebackBuffer(void* ptr, uint32_t size);

/**********************************************************************
 ************************** FAPI Functions ****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to cleanup the MSGCOM data buffers if there are
 *      any pending packets while deleting the channel.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB instance handle
 *  @param[in]  chHandle
 *      MSGCOM Channel Handle which is being deleted.
 *  @param[in]  msgBuffer
 *      MSGCOM Buffer to be deleted.
 *
 *  @retval
 *      Not Applicable.
 */
static void myFreeMsgBuffer(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer)
{
    System_printf ("Debug: Channel Handle 0x%p is being deleted MSGCOM Buffer is 0x%p\n", chHandle, msgBuffer);
    Pktlib_freePacket(pktlibInstHandle, (Ti_Pkt*)msgBuffer);
}

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
static uint8_t* Fapi_MSMCMemoryMalloc(uint32_t size, uint32_t arg)
{
    Error_Block	errorBlock;

    /* Allocate memory from the shared memory heap. */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)arg, size, 128, &errorBlock);
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
static void Fapi_MSMCMemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    /* Cleanup the memory block. */
    Memory_free ((xdc_runtime_IHeap_Handle)arg, ptr, size);
}

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
static uint8_t* Fapi_DDR3MemoryMalloc(uint32_t size, uint32_t arg)
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
static void Fapi_DDR3MemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    /* Cleanup the memory block. */
    Memory_free ((xdc_runtime_IHeap_Handle)arg, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      FAPI Message callback function which is invoked whenever a message
 *      is received on the High Priority FAPI channel.
 *
 *  @param[in]  chHandle
 *      FAPI MSGCOM Channel on which the message is received
 *  @param[in]  arg
 *      Optional application defined argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_highPriorityMsgCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post((Event_Handle)arg, Event_Id_00);
    return;
}

/**
 *  @b Description
 *  @n
 *      FAPI Message callback function which is invoked whenever a message
 *      is received on the Low Priority FAPI channel.
 *
 *  @param[in]  chHandle
 *      FAPI MSGCOM Channel on which the message is received
 *  @param[in]  arg
 *      Optional application defined argument
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_lowPriorityMsgCallback(MsgCom_ChHandle chHandle, uint32_t arg)
{
    Event_post((Event_Handle)arg, Event_Id_01);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a message of the specific
 *      message type.
 *
 *  @param[in]  ptrFAPI
 *      Pointer to the FAPI block
 *  @param[in]  msgType
 *      Message type
 *
 *  @retval
 *      Success -   Pointer to the packet
 *  @retval
 *      Error   -   NULL
 */
static int32_t Fapi_sendMessage(Fapi_MCB* ptrFAPI, uint8_t msgType)
{
    Ti_Pkt*             ptrFAPIPkt;
    TiFapi_msg_s*       ptrFAPIMessage;
    uint32_t            messageLength;
    MsgCom_ChHandle     fapiChannel;

    /* Populate the message on the basis of the message type */
    if (msgType == TI_FAPI_PARAM_RESPONSE)
    {
        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle,
                                        sizeof(TiFapi_msg_s) + sizeof(Fapi_paramResponse));
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 1;
        ptrFAPIMessage->msgLen            = 36;

        /* Copy the hard-coded cell configuration request */
        memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_paramResponse[0] , sizeof(Fapi_paramResponse));

        /* Debug Message: */
        Log_write3(UIAEvt_detailWithStr, 0x10, (IArg)"Sending Parameter Response Message", 0);

        /* Send the message on the low priority channel */
        fapiChannel = ptrFAPI->fapiL1L2LowPriorityChannel;
    }
    else if (msgType == TI_FAPI_CELL_CONFIG_RESPONSE)
    {
        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle,
                                        sizeof(TiFapi_msg_s) + 6);
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = 12;

        /* Copy the hard-coded cell configuration request */
        memcpy ((void *)ptrFAPIMessage->msgBody, (void *)&Fapi_cellConfigResponse[0] , sizeof(Fapi_cellConfigResponse));

        /* Debug Message: */
        Log_write3(UIAEvt_detailWithStr, 0x10, (IArg)"Sending cell configuration response", 0);

        /* Send the message on the low priority channel */
        fapiChannel = ptrFAPI->fapiL1L2LowPriorityChannel;
    }
    else if (msgType == TI_FAPI_UL_SUBFRAME_INDICATION)
    {
        TiFapi_sfInd_s*     ptrSubframeIndication;

        /* Allocate memory from the FAPI DDR3 Heap. */
        ptrFAPIPkt = Pktlib_allocPacket(ptrFAPI->pktlibInstHandle,
                                        ptrFAPI->fapiDDR3HeapHandle,
                                        sizeof(TiFapi_msg_s) + 6);
        if (ptrFAPIPkt == NULL)
        {
            System_printf ("Error: Unable to allocate the FAPI Message\n");
            return -1;
        }

        /* Get the FAPI message */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &messageLength);

        /* Populate the FAPI message */
        ptrFAPIMessage->msgId             = msgType;
        ptrFAPIMessage->lenVendorSpecific = 0;
        ptrFAPIMessage->msgLen            = 6;

        /* Get the subframe indication. */
        ptrSubframeIndication = (TiFapi_sfInd_s*)&ptrFAPIMessage->msgBody[0];

        /* Initialize the subframe indication. */
        ptrSubframeIndication->sfnSf = ((ptrFAPI->sfn & 0xFFF) << 4) | (ptrFAPI->sf & 0xF);

        /* Debug Message: */
        Log_write4(UIAEvt_detailWithStr, 0x10, (IArg)"Sending subframe indication %d:%d", ptrFAPI->sf, ptrFAPI->sfn);

        /* Increment the subframe */
        ptrFAPI->sf = ptrFAPI->sf + 1;
        if (ptrFAPI->sf == 10)
        {
            ptrFAPI->sf = 0;
            ptrFAPI->sfn++;
        }

        /* Send the message on the high priority channel */
        fapiChannel = ptrFAPI->fapiL1L2HighPriorityChannel;
    }
    else
    {
        /* Currently no other message is supported. */
        return -1;
    }

    /* Writeback the data buffer */
    appWritebackBuffer(ptrFAPIMessage, messageLength);

    /* Send the message out. */
    Msgcom_putMessage(fapiChannel, (MsgCom_Buffer*)ptrFAPIPkt);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Software clock timer expiry routine.
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_subFrameTimerExpiry(UArg arg)
{
    Fapi_MCB* ptrFAPI;

    /* Get the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)arg;

    /* Timer has expired. Send the subframe indication */
    Fapi_sendMessage(ptrFAPI, TI_FAPI_UL_SUBFRAME_INDICATION);
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the simulated PHY
 *
 *  @param[in]  ptrFAPI
 *      Pointer to the FAPI MCB
 *
 *  @retval
 *      Not Applicable.
 */
static void Fapi_phyStart (Fapi_MCB* ptrFAPI)
{
    Clock_Params    clockCfg;

    /* Initialize the parametsr */
    Clock_Params_init (&clockCfg);

    /* Setup the clock configuration */
    clockCfg.period    = 1000;
    clockCfg.startFlag = 1;
    clockCfg.arg       = (UArg)ptrFAPI;

    /* Start the clock timer. */
    ptrFAPI->clockHandle = Clock_create(Fapi_subFrameTimerExpiry, 1000, &clockCfg, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used to process all the FAPI messages received on
 *      the specific FAPI channel.
 *
 *  @param[in]  ptrFAPI
 *      Pointer to the FAPI MCB
 *  @param[in]  fapiChannelHandle
 *      FAPI MSGCOM Channel on which the messages are received
 *
 *  @retval
 *      Not Applicable
 */
static void Fapi_processMessage(Fapi_MCB* ptrFAPI, MsgCom_ChHandle fapiChannelHandle)
{
    Ti_Pkt*             ptrFAPIPkt;
    TiFapi_msg_s*       ptrFAPIMessage;
    uint32_t            fapiMessageLen;

    /* Cycle through and process all the received FAPI messages */
    while (1)
    {
        /* Process all packets on the handover channel. */
        Msgcom_getMessage (fapiChannelHandle, (MsgCom_Buffer**)&ptrFAPIPkt);

        /* Did we process all the messages */
        if (ptrFAPIPkt == NULL)
            break;

        /* Get the actual FAPI Message. */
        Pktlib_getDataBuffer(ptrFAPIPkt, (uint8_t**)&ptrFAPIMessage, &fapiMessageLen);

        /* The actual length of the FAPI message is stored in the packet length */
        fapiMessageLen = Pktlib_getPacketLen(ptrFAPIPkt);

        /* Invalidate the FAPI Message */
        appInvalidateBuffer(ptrFAPIMessage, fapiMessageLen);

        /* Populate the message on the basis of the message type */
        if (ptrFAPIMessage->msgId == TI_FAPI_PARAM_REQUEST)
        {
            /* Send the PARAM response message. */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_PARAM_RESPONSE);
        }
        else if (ptrFAPIMessage->msgId == TI_FAPI_CELL_CONFIG_REQUEST)
        {
            /* Send the Cell Configuation response */
            Fapi_sendMessage(ptrFAPI, TI_FAPI_CELL_CONFIG_RESPONSE);
        }
        else if (ptrFAPIMessage->msgId == TI_FAPI_START_REQUEST)
        {
            /* Simulated PHY has been started & initialized. */
            System_printf ("Debug: Simulated PHY is operational. This should be the LAST print!\n");

            /* Start the PHY. */
            Fapi_phyStart(ptrFAPI);
        }

        /* Cleanup the FAPI Packet */
        Pktlib_freePacket(ptrFAPI->pktlibInstHandle, ptrFAPIPkt);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the FAPI Receive Task
 *
 *  @retval
 *      Not Applicable
 */
static void Fapi_receiveTask(UArg arg0, UArg arg1)
{
    uint32_t    events;
    Fapi_MCB*   ptrFAPI;

    /* Get the pointer to the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)arg0;

    /* Loop around forever. */
    while (1)
    {
        /* Wait for an event to occur. */
        events = Event_pend(ptrFAPI->fapiEventHandle, Event_Id_NONE,
                            Event_Id_00 + Event_Id_01, BIOS_WAIT_FOREVER);

        /* Did we get a message on the HIGH Priority Channel? If so process all
         * the FAPI messages */
        if (events & Event_Id_00)
            Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL2L1HighPriorityChannel);

        /* Did we get a message on the LOW Priority Channel? If so process all
         * the FAPI messages */
        if (events & Event_Id_01)
            Fapi_processMessage(ptrFAPI, ptrFAPI->fapiL2L1LowPriorityChannel);
    }
}

/**
 *  @b Description
 *  @n
 *      The function initializes the FAPI component on the simulated PHY
 *
 *  @param[in]  ptrSimPhyMCB
 *      Pointer to the Simulated PHY MCB
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     -   Handle to the FAPI component
 *  @retval
 *      Error       -   NULL
 */
void* Fapi_initComponent (AppSimPHYDomainMCB* ptrSimPhyMCB, int32_t* errCode)
{
    Task_Params           taskParams;
    Msgcom_ChannelCfg     chConfig;
    Pktlib_HeapCfg        heapCfg;
    Fapi_MCB*             ptrFAPI;
    Resmgr_ResourceCfg    fapiResourceConfig =
    {
        0,    /* Number of CPINTC Output  requested                               */
        1,    /* Number of Accumulator Channels requested                         */
        0,    /* Number of Hardware Semaphores requested                          */
        1,    /* Number of QPEND Queues requested                                 */
        /* Requested Memory Region Configuration. */
        {
		    /* Name,           Type,                     Linking RAM,                          Num,    Size */
        	{ "FAPI-DDR3",  Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  128,     128},
            { "FAPI-MSMC",  Resmgr_MemRegionType_MSMC,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  128,     128},
        }
    };

    /* Allocate memory for the FAPI object. */
    ptrFAPI = Memory_alloc ((xdc_runtime_IHeap_Handle)ptrSimPhyMCB->ddr3PrivateHeapHandle, sizeof(Fapi_MCB), 0, NULL);
    if (ptrFAPI == NULL)
	{
		System_printf ("Error: Unable to allocate memory for the FAPI object\n", DNUM);
		return NULL;
	}

    /* Initialize the FAPI Master Control block */
    memset((void *)ptrFAPI, 0, sizeof(Fapi_MCB));

    /* Keep a copy to the application domain configuration. */
    ptrFAPI->ptrSimPhyDomainMCB = ptrSimPhyMCB;
    ptrFAPI->pktlibInstHandle   = Domain_getPktlibInstanceHandle(ptrSimPhyMCB->syslibHandle);
    ptrFAPI->msgcomInstHandle   = Domain_getMsgcomInstanceHandle(ptrSimPhyMCB->syslibHandle);

    /* Request the resource manager for the resources requested by the FAPI component. */
    if (Resmgr_processConfig (Domain_getSysCfgHandle(ptrSimPhyMCB->syslibHandle), &fapiResourceConfig, errCode) < 0)
	{
	    System_printf ("Error: SYSRM configuration failed with error code %d\n", *errCode);
	    return NULL;
    }

    /* Create the FAPI Event object */
    ptrFAPI->fapiEventHandle = Event_create(NULL, NULL);
    if (ptrFAPI->fapiEventHandle == NULL)
    {
        System_printf ("Error: FAPI Event Object creation failed\n");
        return NULL;
    }

    /*************************************************************************
     * FAPI High Priority Channel
     *   This is the HIGH Priority Channel which is created by the L1 (Reader)
     *   and is used to receive messages from the L2(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Initialize the channel configuration */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                    = (uint32_t)ptrFAPI->fapiEventHandle;
    chConfig.appCallBack                                            = Fapi_highPriorityMsgCallback;
    chConfig.msgcomInstHandle                                       = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.queuePendQueue  = fapiResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.cpIntcId        = fapiResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.systemInterrupt = fapiResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueCfg.queueIntrUnion.queuePendCfg.hostInterrupt   = fapiResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the FAPI Channel. */
	ptrFAPI->fapiL2L1HighPriorityChannel = Msgcom_create (TI_FAPI_L2L1_HP_CHANNEL_NAME,
                                                          Msgcom_ChannelType_QUEUE, &chConfig, errCode);
    if (ptrFAPI->fapiL2L1HighPriorityChannel == 0)
    {
        System_printf ("Error: Unable to create the High Priority FAPI channel Error : %d\n", *errCode);
        return NULL;
    }

    /*************************************************************************
     * FAPI Low Priority Channel
     *   This is the Low Priority Channel which is created by the L1 (Reader)
     *   and is used to receive messages from the L2(Writer)
     *   It is *recommended* that applications verify the channel
     *   configuration and modify it while designing the FAPI Interface.
     *************************************************************************/

    /* Initialize the channel configuration */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                   = Msgcom_ChannelMode_NON_BLOCKING;
    chConfig.arg                                                    = (uint32_t)ptrFAPI->fapiEventHandle;
    chConfig.appCallBack                                            = Fapi_lowPriorityMsgCallback;
    chConfig.msgcomInstHandle                                       = ptrFAPI->msgcomInstHandle;
    chConfig.u.queueCfg.interruptMode                               = Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.type                  = Msgcom_AccumulatedChannelType_HIGH;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accChannel            = fapiResourceConfig.accChannelResponse[0].accChannel;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.accQueue              = fapiResourceConfig.accChannelResponse[0].queue;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.pdspId                = fapiResourceConfig.accChannelResponse[0].pdspId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.interruptId           = fapiResourceConfig.accChannelResponse[0].eventId;
    chConfig.u.queueCfg.queueIntrUnion.accCfg.maxPageEntries        = 5;
	chConfig.u.queueCfg.queueIntrUnion.accCfg.pacingTimerCount      = 0;

    /* Create the FAPI Channel. */
	ptrFAPI->fapiL2L1LowPriorityChannel = Msgcom_create (TI_FAPI_L2L1_LP_CHANNEL_NAME,
                                                         Msgcom_ChannelType_QUEUE, &chConfig, errCode);
    if (ptrFAPI->fapiL2L1LowPriorityChannel == 0)
    {
        System_printf ("Error: Unable to create the Low Priority FAPI channel Error : %d\n", *errCode);
        return NULL;
    }

    /*************************************************************************
     * PKTLIB FAPI MSMC Heap:
     *************************************************************************/

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy (heapCfg.name, TI_FAPI_MSG_HEAP_MSMC_NAME);
    heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
    heapCfg.memRegion                       = fapiResourceConfig.memRegionResponse[1].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
    heapCfg.numPkts                         = 4;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(0);
    heapCfg.heapInterfaceTable.dataMalloc   = Fapi_MSMCMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Fapi_MSMCMemoryFree;

    /* Create the FAPI Heap. */
    ptrFAPI->fapiMSMCHeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrFAPI->fapiMSMCHeapHandle == NULL)
    {
	    System_printf ("Error: Unable to create FAPI MSMC heap (Error code %d)\n", errCode);
        return NULL;
    }

    /*************************************************************************
     * PKTLIB FAPI DDR3 Heap:
     *************************************************************************/

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    strcpy (heapCfg.name, TI_FAPI_MSG_HEAP_DDR_NAME);
    heapCfg.pktlibInstHandle                = ptrFAPI->pktlibInstHandle;
    heapCfg.memRegion                       = fapiResourceConfig.memRegionResponse[0].memRegionHandle;
    heapCfg.sharedHeap                      = 1;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = sizeof(TiFapi_nonTlvMsg_s);
    heapCfg.numPkts                         = 64;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.arg                             = (uint32_t)SharedRegion_getHeap(1);
    heapCfg.heapInterfaceTable.dataMalloc   = Fapi_DDR3MemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = Fapi_DDR3MemoryFree;

    /* Create the FAPI DDR3 Heap. */
    ptrFAPI->fapiDDR3HeapHandle = Pktlib_createHeap(&heapCfg, errCode);
    if (ptrFAPI->fapiDDR3HeapHandle == NULL)
    {
	    System_printf ("Error: Unable to create FAPI DDR3 heap (Error code %d)\n", errCode);
        return NULL;
    }

    /*************************************************************************
     * PKTLIB FAPI Heap:
     *  - Used for the UL-SCH and this is created by the L2 stack and is found
     *    on the L1 PHY
     *************************************************************************/
    while (1)
    {
        ptrFAPI->fapiUlSchDataHeapHandle = Pktlib_findHeapByName (ptrFAPI->pktlibInstHandle,
                                                                  TI_FAPI_ULSCH_DATA_HEAP_NAME, errCode);
        if (ptrFAPI->fapiUlSchDataHeapHandle != NULL)
            break;
    }

    /* Find the FAPI High Priority MSGCOM Channels:
     *  - These channels are created by the L1 writer to send messages to the L2 reader.
     *  - This is also a SYNC point since we cannot proceed till the L2 stack has created the
     *    channel. */
    while (1)
    {
        ptrFAPI->fapiL1L2HighPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                            TI_FAPI_L1L2_HP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL1L2HighPriorityChannel != NULL)
            break;
    }

    /* Find the FAPI Low Priority MSGCOM Channels:
     *  - These channels are created by the L1 writer to send messages to the L2 reader.
     *  - This is also a SYNC point since we cannot proceed till the L2 stack has created the
     *    channel. */
    while (1)
    {
        ptrFAPI->fapiL1L2LowPriorityChannel = Msgcom_find (ptrFAPI->msgcomInstHandle,
                                                           TI_FAPI_L1L2_LP_CHANNEL_NAME, errCode);
        if (ptrFAPI->fapiL1L2LowPriorityChannel != NULL)
            break;
    }

    /* Debug Message: */
    System_printf ("-----------------------------------------------------------------\n");
    System_printf ("FAPI Informational Block: \n");
    System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                    TI_FAPI_L1L2_HP_CHANNEL_NAME, ptrFAPI->fapiL1L2HighPriorityChannel);
    System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                    TI_FAPI_L1L2_LP_CHANNEL_NAME, ptrFAPI->fapiL1L2LowPriorityChannel);
    System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                    TI_FAPI_L2L1_HP_CHANNEL_NAME, ptrFAPI->fapiL2L1HighPriorityChannel);
    System_printf ("Debug: FAPI Channel %s -> Channel Handle 0x%p\n",
                    TI_FAPI_L2L1_LP_CHANNEL_NAME, ptrFAPI->fapiL2L1LowPriorityChannel);
    System_printf ("Debug: FAPI UL Sch Heap %s -> Heap Handle 0x%p\n",
                    TI_FAPI_ULSCH_DATA_HEAP_NAME, ptrFAPI->fapiUlSchDataHeapHandle);
    System_printf ("Debug: FAPI MSMC Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                    TI_FAPI_MSG_HEAP_MSMC_NAME, ptrFAPI->fapiMSMCHeapHandle,
                    Pktlib_getInternalHeapQueue(ptrFAPI->fapiMSMCHeapHandle));
    System_printf ("Debug: FAPI DDR3 Heap %s -> Heap Handle 0x%p (Free Queue 0x%x)\n",
                    TI_FAPI_MSG_HEAP_DDR_NAME, ptrFAPI->fapiDDR3HeapHandle,
                    Pktlib_getInternalHeapQueue(ptrFAPI->fapiDDR3HeapHandle));
    System_printf ("-----------------------------------------------------------------\n");

    /* Spawn the FAPI receive task */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    taskParams.arg0      = (UArg)ptrFAPI;
    taskParams.priority  = 3;
    ptrFAPI->taskHandle  = Task_create(Fapi_receiveTask,  &taskParams, NULL);

    /* Return the handle to the FAPI Object */
    return (void*)ptrFAPI;
}

/**
 *  @b Description
 *  @n
 *      The function deinitializes the FAPI component on the simulated PHY
 *
 *  @param[in]  fapiHandle
 *      FAPI component handle
 *
 *  @retval
 *      Not applicable
 */
void Fapi_deinitComponent (void* fapiHandle)
{
    Fapi_MCB*   ptrFAPI;
    int32_t     errCode;

    /* Get the FAPI MCB */
    ptrFAPI = (Fapi_MCB*)fapiHandle;
    if (ptrFAPI == NULL)
        return;

    /* Shutdown the Simulated PHY Clock */
    if (ptrFAPI->clockHandle != NULL)
        Clock_delete(&ptrFAPI->clockHandle);

    /* Shutdown the FAPI Task */
    if (ptrFAPI->taskHandle != NULL)
        Task_delete(&ptrFAPI->taskHandle);

    /* Shutdown the Event block */
    if (ptrFAPI->fapiEventHandle != NULL)
        Event_delete(&ptrFAPI->fapiEventHandle);

    /* Shutdown the writer channels:
     *  - High & Low Priority writer channels which sends messages to the L2  */
    if (ptrFAPI->fapiL1L2HighPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL1L2HighPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: Unable to delete the high priority writer channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
            return;
        }
    }
    if (ptrFAPI->fapiL1L2LowPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL1L2LowPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf  ("Error: Unable to delete the low priority writer channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
            return;
        }
    }

    /* Shutdown the reader channels:
     *  - High & Low Priority Reader channel which receives messages from L2 */
    if (ptrFAPI->fapiL2L1HighPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1HighPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf ("Error: Unable to delete the high priority reader channel '%p'\n", ptrFAPI->fapiL1L2HighPriorityChannel);
            return;
        }
    }
    if (ptrFAPI->fapiL2L1LowPriorityChannel != NULL)
    {
        if (Msgcom_delete (ptrFAPI->fapiL2L1LowPriorityChannel, myFreeMsgBuffer) < 0)
        {
            System_printf  ("Error: Unable to delete the low priority reader channel '%p'\n", ptrFAPI->fapiL1L2LowPriorityChannel);
            return;
        }
    }

    /* Shutdown the MSMC and DDR3 Heaps */
    if (ptrFAPI->fapiDDR3HeapHandle != NULL)
    {
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle,
                               ptrFAPI->fapiDDR3HeapHandle, &errCode) < 0)
        {
            System_printf  ("Error: Unable to delete the FAPI DDR3 Heap [Error code %d]\n", errCode);
            return;
        }
    }
    if (ptrFAPI->fapiMSMCHeapHandle != NULL)
    {
        if (Pktlib_deleteHeap (ptrFAPI->pktlibInstHandle,
                               ptrFAPI->fapiMSMCHeapHandle, &errCode) < 0)
        {
            System_printf  ("Error: Unable to delete the FAPI MSMC Heap [Error code %d]\n", errCode);
            return;
        }
    }

    /* Cleanup memory allocated to the FAPI MCB */
    Memory_free ((xdc_runtime_IHeap_Handle)ptrFAPI->ptrSimPhyDomainMCB->ddr3PrivateHeapHandle, ptrFAPI, sizeof(Fapi_MCB));
    return;
}

