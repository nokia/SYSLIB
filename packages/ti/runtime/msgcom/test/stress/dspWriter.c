/**
 *   @file  dspWriter.c
 *
 *   @brief
 *      DSP Writer Unit Test Code.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

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

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_tmrAux.h>

/* SYSLIB Include Files. */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/name/name_proxyClient.h>

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Global SYSLIB Handle(s): */
extern Pktlib_HeapHandle        myPktLibHeapHandle;
extern Pktlib_HeapHandle        mySharedHeapHandle;
extern Name_DBHandle            globalNameDatabaseHandle;
extern Pktlib_InstHandle        appPktlibInstanceHandle;
extern Msgcom_InstHandle        appMsgcomInstanceHandle;
extern Name_ClientHandle        nameClientHandle;

/* Global Variable which tracks which test needs to be executed */
extern uint32_t  testSelection;

/* Application Cache Management API */
extern void appWritebackBuffer(void* ptr, uint32_t size);

/* Global application requested resource configuration */
extern Resmgr_ResourceCfg      appResourceConfig;

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* The maximum size of the data messages allocated in the packet heap */
#define TEST_MAX_DATA_SIZE          512

/**********************************************************************
 *************************** Writer Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory to be allocated
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

    /* Allocate a buffer from the default HeapMemMp */
    return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(0), size, 128, &errorBlock);
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
 *      This is the test task for executing the DSP Writer and ARM reader
 *
 *  @retval
 *      Not Applicable.
 */
Void stressWriterTask (UArg arg0, UArg arg1)
{
    MsgCom_ChHandle     armChannelHandle;
    MsgCom_ChHandle     dspChannelHandle;
    Ti_Pkt*             ptrMessage;
    uint32_t*           ptrDataBuffer;
    uint32_t            dataLen;
    int32_t             messageCounter = 0;
    int32_t             errCode;
    Msgcom_ChannelCfg   chConfig;
    Pktlib_HeapCfg      heapCfg;
    CSL_Status          status;
    CSL_TmrObj          tmrObj;
    uint32_t            countLo;
    uint32_t            countHi;
    CSL_TmrHandle       timer6Handle;
    UInt                key;
    char                channelName[MSG_COM_MAX_CHANNEL_LEN];

#if (defined(DEVICE_K2H) || defined (DEVICE_K2K))
    /* Open the Timer6 Handle from the CSL: */
    timer6Handle = CSL_tmrOpen (&tmrObj, CSL_TIMER_6, NULL, &status);
    if (timer6Handle == NULL)
    {
	    System_printf ("Error: Unable to open the CSL Timer6 [Error code %d]\n", status);
		return;
    }
#elif defined (DEVICE_K2L)
    /* Open the Timer8 Handle from the CSL: */
    timer6Handle = CSL_tmrOpen (&tmrObj, CSL_TIMER_8, NULL, &status);
    if (timer6Handle == NULL)
    {
	    System_printf ("Error: Unable to open the CSL Timer6 [Error code %d]\n", status);
		return;
    }
#else
#error "Unsupported Device"
#endif

    /* Launch Timer6 */
    timer6Handle->regs->TGCR = 0x0; 			// Stop
    timer6Handle->regs->CNTLO = 0x00000000; 	// Clear the counts
    timer6Handle->regs->CNTHI = 0x00000000;
    timer6Handle->regs->PRDLO = 0xFFFFFFFF;		// Set period
    timer6Handle->regs->PRDHI = 0xFFFFFFFF;
    timer6Handle->regs->TCR = 0x00800080;		// Set Mode
    timer6Handle->regs->TGCR = 0x3; 			// Start

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

    /* Populate the heap configuration */
    sprintf(heapCfg.name, "Test_L2Heap_Core%d", DNUM);
    heapCfg.memRegion                       = (Qmss_MemRegion)appResourceConfig.memRegionResponse[0].memRegionHandle;
    heapCfg.pktlibInstHandle                = appPktlibInstanceHandle;
    heapCfg.sharedHeap                      = 0;
    heapCfg.useStarvationQueue              = 0;
    heapCfg.dataBufferSize                  = TEST_MAX_DATA_SIZE;
    heapCfg.numPkts                         = 2;
    heapCfg.numZeroBufferPackets            = 0;
    heapCfg.dataBufferPktThreshold          = 0;
    heapCfg.zeroBufferPktThreshold          = 0;
    heapCfg.heapInterfaceTable.dataMalloc   = mySharedMemoryMalloc;
    heapCfg.heapInterfaceTable.dataFree     = mySharedMemoryFree;

    /* Create the LOCAL Packet Library Heap. */
    myPktLibHeapHandle = Pktlib_createHeap(&heapCfg, &errCode);
    if (myPktLibHeapHandle == NULL)
    {
	    System_printf ("Error: Unable to create packet library heap error code %d\n", errCode);
		return;
    }
    System_printf ("Debug: Packet Library Heap [%s] 0x%x (Internal Queue: 0x%x) has been created\n",
                   heapCfg.name, myPktLibHeapHandle, Pktlib_getInternalHeapQueue(myPktLibHeapHandle));

    /* Create the channel name: */
    snprintf (channelName, MSG_COM_MAX_CHANNEL_LEN, "Stress-Test:DSP Reader%d", DNUM);

    /* Initialize the channel configuration. */
    memset ((void *)&chConfig, 0, sizeof(Msgcom_ChannelCfg));

    /* Populate the channel configuration. */
    chConfig.mode                                                           = Msgcom_ChannelMode_BLOCKING;
    chConfig.appCallBack                                                    = NULL;
    chConfig.msgcomInstHandle                                               = appMsgcomInstanceHandle;
    chConfig.u.queueDMACfg.interruptMode                                    = Msgcom_QueueInterruptMode_DIRECT_INTERRUPT;
    chConfig.u.queueDMACfg.rxFreeQueueNum                                   = (Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(myPktLibHeapHandle)));
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.queuePendQueue    = appResourceConfig.qPendResponse[0].queue;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.cpIntcId          = appResourceConfig.qPendResponse[0].cpIntcId;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.systemInterrupt   = appResourceConfig.qPendResponse[0].systemInterrupt;
    chConfig.u.queueDMACfg.queueDMAIntrUnion.queuePendCfg.hostInterrupt     = appResourceConfig.qPendResponse[0].hostInterrupt;

    /* Create the DSP MSGCOM channel. */
    dspChannelHandle = Msgcom_create (channelName, Msgcom_ChannelType_QUEUE_DMA, &chConfig, &errCode);
    if (dspChannelHandle == 0)
    {
        System_printf ("Error: Unable to open the DSP channel [Error: %d]\n", errCode);
        return;
    }

    /* Push the channel name from the ARM to the DSP realm. */
    if (Name_push (nameClientHandle, channelName, Name_ResourceBucket_INTERNAL_SYSLIB,
                   Name_ResourceOperationType_CREATE, &errCode) < 0)
    {
        System_printf ("Error: Channel name PUSH to ARM realm failed [Error code %d] \n", errCode);
        return;
    }
    System_printf ("Debug: Channel Name '%s' pushed successfully to the ARM realm\n", channelName);

    /* Wait for the message communicator channel to be created. */
    while (1)
    {
        /* Check if the communicator channel has been created or not? */
    	if (DNUM == 0)
    		armChannelHandle = Msgcom_find(appMsgcomInstanceHandle, "Stress-Test:ARM Reader0", &errCode);
    	else
    		armChannelHandle = Msgcom_find(appMsgcomInstanceHandle, "Stress-Test:ARM Reader1", &errCode);

        /* Did we get the ARM Channel? */
        if (armChannelHandle != NULL)
            break;
        Task_sleep (2);
    }

    /* Is this a single core test: */
    if ((testSelection == 1) || (testSelection == 2))
    {
        /* YES. This is a single core test. Skip DSP Core1 */
        if (DNUM == 1)
        {
            System_printf ("Debug: Skipping the test on DSP Core %d\n", DNUM);
            return;
        }
    }

    /* Get the Timer count values */
    CSL_tmrGetTimLoCount (timer6Handle, &countLo);
    CSL_tmrGetTimHiCount (timer6Handle, &countHi);

    /* Debug Message: Start the test test after a delay */
    Task_sleep(1000);
    System_printf ("Debug: Starting the stress test [%x:%x]...\n", countHi, countLo);

    while (1)
    {
        /* Allocate a message from the Local core specific Heap. */
        ptrMessage = Pktlib_allocPacket (appPktlibInstanceHandle, myPktLibHeapHandle, 16);
        if (ptrMessage == NULL)
            continue;

        /* Get the data buffer. */
        Pktlib_getDataBuffer (ptrMessage, (uint8_t**)&ptrDataBuffer, &dataLen);

        /* Set the data length */
        dataLen = 12;

    	/* Disable Interrupts */
	    key = Hwi_disable();

        /* Get the Timer count values */
        CSL_tmrGetTimLoCount (timer6Handle, &countLo);
        CSL_tmrGetTimHiCount (timer6Handle, &countHi);

        /* Populate the data buffer
         *  4 Bytes -> DSP Core Number
         *  4 Bytes -> CountLo
         *  4 Bytes -> CountHi */
        *ptrDataBuffer       = DNUM;
        *(ptrDataBuffer + 1) = countLo;
        *(ptrDataBuffer + 2) = countHi;

        /* Set the data buffer length and populate the payload. */
        Pktlib_setDataBufferLen (ptrMessage, dataLen);

        /* Set the packet length of the message */
        Pktlib_setPacketLen (ptrMessage, dataLen);

        /* Writeback the data buffer */
        appWritebackBuffer (ptrDataBuffer, dataLen);

        /* Send the message. */
        if (Msgcom_putMessage(armChannelHandle, (MsgCom_Buffer*)ptrMessage) < 0)
        {
            System_printf ("Error: Unable to send the message\n");
        	/* Reenable Interrupts. */
	        Hwi_restore(key);
            break;
        }

    	/* Reenable Interrupts. */
	    Hwi_restore(key);

        /* Do we need to wait for a response or not? */
        if ((testSelection == 1) || (testSelection == 3))
        {
            /* YES. We need to wait for a response back from the ARM application */
            Msgcom_getMessage (dspChannelHandle, (MsgCom_Buffer**)&ptrMessage);

            /* Cleanup the memory */
            Pktlib_freePacket(appPktlibInstanceHandle, (Ti_Pkt*)ptrMessage);
        }

        /* Increment the message counter */
        messageCounter++;

        /* Display the number of messages received. */
        if ((messageCounter%100000) == 0)
            System_printf ("Debug: %d messages transmitted.\n", messageCounter);
    }
    System_printf ("Debug: DSP Writer has sent all the packets\n");

    /* Delete the msgcom channel */
    if (Msgcom_delete (armChannelHandle, myFreeMsgBuffer) < 0)
    {
        System_printf ("Error: Unable to delete the writer channel\n");
        return;
    }
    return;
}

