/**
 *   @file  dat_log.c
 *
 *   @brief
 *      The file implements the DAT interface with the Logger Stream2
 *      module
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
/* SYSLIB Include Files */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/dat/include/dat_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf

/* HPLib Include Files */
#include <sys/time.h>
#include <ti/runtime/hplib/hplib_util.h>
#include <ti/runtime/hplib/hplib_sync.h>
#else
#include <xdc/runtime/System.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>

/* UIA Include Files */
#include <ti/uia/sysbios/LoggerStreamer2.h>
#endif

#include <ti/uia/runtime/UIAPacket.h>
#include <ti/uia/runtime/EventHdr.h>
#include <ti/uia/events/UIASnapshot.h>


/**********************************************************************
 ************************** DAT Log Functions *************************
 **********************************************************************/

#ifndef __ARMv7

/* UIA Include Files */
#include <ti/uia/sysbios/LoggerStreamer2.h>

/**
 *  @b Description
 *  @n
 *      This function initializes the general-purpose producer buffers with
 *      UIA packet and event headers.
 *
 *  @param[in]  handle
 *      Handle to the Logger Streamer
 *  @param[in]  buffer
 *      Pointer to the buffer to be initialzed
 *  @param[in]  src
 *      Source id used by loggerStreamer to identify the source of the buffer
 *  @param[in]  instance
 *      Not Used
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_initBufferUIA (void* handle, void* buffer, uint16_t src, uint16_t instance)
{
    LoggerStreamer2_initBuffer (handle, buffer, src);
}
/**
 *  @b Description
 *  @n
 *      The function is registered with the DAT producer to initialize
 *      and prime the buffers for initial use.
 *
 *  @param[in]  handle
 *      Handle to the Logger Streamer
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_primeProducerUIA (void* handle)
{
    Dat_Producer*   ptrProducer;
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);
    if(ptrProducer == NULL)
    {
        System_printf("Error: Dat_primeProducerUIA ptrProducer is NULL\n");
        return -1;
    }
    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle,
                                 (uint32_t)LoggerStreamer2_getBufSize((LoggerStreamer2_Handle)handle));
    if (ptrPkt == NULL)
        return -1;

    /* Get the data buffer associated with the */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Remember the packet in production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Prime the Logger streamer */
    LoggerStreamer2_prime ((LoggerStreamer2_Handle)handle, ptrDataBuffer);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the logger streamer object to
 *      be invoked when the logger buffer is full and needs to be
 *      exchanged with a newer buffer.
 *
 *  @param[in]  handle
 *      Logger Streamer object Handle.
 *  @param[out] fillBuffer
 *      Pointer to the buffer which is to be exchanged.
 *
 *  \ingroup DAT_FUNCTION
 *
 *  @retval
 *      Pointer to the allocated buffer
 */
Ptr Dat_exchangeFunction(LoggerStreamer2_Handle handle, uint8_t* fillBuffer)
{
    Dat_Producer*   ptrProducer;
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint32_t        queueDepth;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);

    if(ptrProducer == NULL )
        return fillBuffer;

    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Increment the stats for the number of buffer being exchanged. */
    ptrProducer->stats.bufferExchange++;

    /* Keep track of the maximum queue depth of the pending queue. */
    queueDepth = Qmss_getQueueEntryCount(ptrProducer->pendingQueueHandle);
    if (queueDepth > ptrProducer->stats.maxQueueDepth)
        ptrProducer->stats.maxQueueDepth = queueDepth;

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle,
                                 (uint32_t)LoggerStreamer2_getBufSize((LoggerStreamer2_Handle)handle));
    if (ptrPkt == NULL)
    {
        /* Error: There are no more packets available to keep up with the producer */
        ptrProducer->stats.bufferOverrun++;
        return fillBuffer;
    }

    /* Push the previous production packet to the pending queue. */
    Qmss_queuePushDesc (ptrProducer->pendingQueueHandle, (void*)ptrProducer->ptrProductionPkt);

    /* Get the data buffer associated with the */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Remember the new packet which has been given for production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Return the new data buffer. */
    return (void*)ptrDataBuffer;
}

/**
 *  @b Description
 *  @n
 *      This function initializes the general-purpose producer buffers with
 *      UIA packet and event headers.
 *
 *  @param[in]  handle
 *      Handle to the Logger Streamer
 *  @param[in]  buffer
 *      Pointer to the buffer to be initialzed
 *  @param[in]  src
 *      Source id used by loggerStreamer to identify the source of the buffer
 *  @param[in]  instance
 *      Not Used

 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_initBufferGP (void* handle, void* buffer, uint16_t src, uint16_t instance)
{
    uint32_t        bufferSize;
    uint32_t        dataSize;
    uint32_t*       ptrEventHeader;
    uint32_t*       writePtr;
    uint8_t*        ptrDataBuffer;
    uint32_t        eventSize;
    uint32_t        numPaddingBytes;
    Dat_Producer*   ptrProducer;

    /* Initialize the UIA packet header. */
    LoggerStreamer2_initBuffer((LoggerStreamer2_Object*)handle, (Ptr)buffer, src);

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);
    if(ptrProducer == NULL)
        System_printf("Error: Dat_primeProducerUIA Producer handle is NULL\n");

    /* Get the buffer size :
       Note: Buffer size should be set when logger stream instance is created.
             Buffer size should include UIA/EVENT header size.
     */
    bufferSize = (uint32_t)LoggerStreamer2_getBufSize((LoggerStreamer2_Handle)handle);
    eventSize        = bufferSize - DAT_UIA_PKT_HEADER_SIZE;
    dataSize         = (eventSize - DAT_UIA_EVT_HEADER_SIZE) & 0xfffffffc;
    numPaddingBytes  = (eventSize - DAT_UIA_EVT_HEADER_SIZE) & 0x3;

    /* Set the event length in the packet. */
    UIAPacket_setEventLength((UIAPacket_Hdr*)buffer, bufferSize);

    /* Initialize the UIA event header. */
    /* Module's ID (Reference: Text.xs) */
    xdc_runtime_Types_ModuleId mid = Module__MID;

    /* Initialize the event header, including timestamps */
    ptrEventHeader = (uint32_t*)((uint32_t)buffer + DAT_UIA_PKT_HEADER_SIZE);

    /* Save the start of the data buffer. */
    ptrDataBuffer = (uint8_t*)ptrEventHeader + DAT_UIA_EVT_HEADER_SIZE;

    /* Fill in the event header fields.
       HdrType_EventWithSnapshotIdAndTimestamp:
        word0: EventHdr
        word1: Timestamp lower 32 bits
        word2: Timestamp upper 32 bits
        word3: event Id (top 16 bits) & module Id (bottom 16 bits)
        word4: filename pointer
        word5: linenum
        word6: snapshotId
        word7: address where the data was located
        word8: total length of data (top 16-bits)
               length for this record (bottom 16 bits)
        word9: format pointer
        data:  the rest of the record contains the data
    */
    writePtr = ptrEventHeader;
    *(writePtr++) = ti_uia_runtime_EventHdr_genEventHdrWord1(eventSize, 0,
                    ti_uia_runtime_EventHdr_HdrType_EventWithSnapshotIdAndTimestamp);
    *(writePtr++) = TSCL;
    *(writePtr++) = TSCH;
    *(writePtr++) = ((ti_uia_events_UIASnapshot_memoryRange) & 0xffff0000) | mid;
    *(writePtr++) = (IArg)__FILE__;
    *(writePtr++) = (IArg)__LINE__;
    *(writePtr++) = 0;
    *(writePtr++) = (Bits32)ptrDataBuffer;
    *(writePtr++) = (((dataSize & 0xFFFF)<<16) | (dataSize & 0xFFFF));

    /* The format string used by System Analyzer to identify what the snapshot
     * data represents. */
    *(writePtr++) = (IArg)ptrProducer->cfg.name;

    /* Increment the write pointer to end of the data section. */
    writePtr = (uint32_t*)(ptrDataBuffer + dataSize);

    /* Add an invalid UIA header for the rest of the data in the buffer. */
    if(numPaddingBytes)
        UIAPacket_setInvalidHdr (writePtr, numPaddingBytes);

    /* Clear the buffer (skip the UIA headers) if configured. */
    if (ptrProducer->cfg.clearBuffer)
        memset ((void*)ptrDataBuffer, 0, dataSize);
}
/**
 *  @b Description
 *  @n
 *      The function is registered with the DAT producer to initialize
 *      and prime the buffers for initial use.
 *      For General purpose producer, this function is called from
 *      Application to get the first buffer.
 *
 *  @param[in]  handle
 *      Handle to the Logger Streamer
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_primeProducerGP (void* handle)
{

    /* Nothing is needed for General purpose Producer */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function closes the active buffer and returns a new buffer.
 *      If the consumer is slow, the same buffer will be returned again.
 *      This is applicable only for general-purpose producers.
 *
 *  @param[in]  handle
 *      Logger Streamer handle.
 *  @param[out] fillBuffer
 *      Active buffer that has to be closed
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to new buffer
 *  @retval
 *      Error   -   NULL. This occurs if an invalid producer handle
 *                        or NULL buffer is passed.
 */
Ptr Dat_bufferExchangeGP(LoggerStreamer2_Handle handle, uint8_t* fillBuffer)
{
    Dat_Producer*   ptrProducer;
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint32_t        queueDepth;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;

    uint32_t        sequenceCount;
    UIAPacket_Hdr*  ptrUIAPktHeader;
    uint32_t*       ptrUIAEvtHeader;
    uint32_t        bufferSize;

    /* Sanity Check: Ensure that the parameters are valid. */
    if ((handle == NULL) || (fillBuffer == NULL))
        return NULL;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext(handle);

    if(ptrProducer == NULL )
        return (Ptr)fillBuffer;

    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Increment the stats for the number of buffer being exchanged. */
    ptrProducer->stats.bufferExchange++;

    /* Get the buffer size */
    bufferSize = (uint32_t)LoggerStreamer2_getBufSize((LoggerStreamer2_Handle)handle);

    /* Adding UIA and Event Header at the begging of the buffer */
    ptrUIAPktHeader = (UIAPacket_Hdr*)((uint32_t)fillBuffer - DAT_UIA_HEADER_OFFSET);
    ptrUIAEvtHeader = (uint32_t*)((uint32_t)ptrUIAPktHeader + DAT_UIA_PKT_HEADER_SIZE);

    /* Update sequence number in UIA packet and event header. */
    sequenceCount = ptrProducer->sequenceCount;
    UIAPacket_setSequenceCount (ptrUIAPktHeader, sequenceCount);
    *(ptrUIAEvtHeader++) = EventHdr_setSeqCount (*ptrUIAEvtHeader, sequenceCount);

    /* Update timestamp in event header. */
    *(ptrUIAEvtHeader++) = TSCL;
    *(ptrUIAEvtHeader++) = TSCH;

    /* Keep track of the maximum queue depth of the pending queue. */
    queueDepth = Qmss_getQueueEntryCount(ptrProducer->pendingQueueHandle);
    if (queueDepth > ptrProducer->stats.maxQueueDepth)
        ptrProducer->stats.maxQueueDepth = queueDepth;

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle, bufferSize);
    if (ptrPkt == NULL)
    {
        /* Error: There are no more packets available to keep up with the producer */
        ptrProducer->stats.bufferOverrun++;
        return fillBuffer;
    }

    /* Push the previous production packet to the pending queue. */
    Qmss_queuePushDesc (ptrProducer->pendingQueueHandle, (void*)ptrProducer->ptrProductionPkt);

    /* Get the data buffer associated with the new packet */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Increment the sequence number */
    ptrProducer->sequenceCount++;

    /* Clear the buffer (skip the UIA headers) if configured. */
    if (ptrProducer->cfg.clearBuffer)
        memset ((void*)(ptrDataBuffer + DAT_UIA_HEADER_OFFSET), 0,
                (bufferSize - DAT_UIA_HEADER_OFFSET));

    /* Remember the new packet which has been given for production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Return the new data buffer. */
    return (void*)(ptrDataBuffer + DAT_UIA_HEADER_OFFSET);

}

/* Logger Function table for UIA type of Producer */
Dat_LoggerFuncTbl Dat_LoggerFuncUIA =
{
    Dat_initBufferUIA,
    Dat_primeProducerUIA,
    Dat_exchangeFunction
};

/* Logger Function table for General Purpose type of Producer */
Dat_LoggerFuncTbl Dat_LoggerFuncGP =
{
    Dat_initBufferGP,
    Dat_primeProducerGP,
    Dat_bufferExchangeGP
};

#else
/* A global Lock used to protect logger buffers , it is used in CUIA Osal functions */
extern hplib_spinLock_T     loggerLock;

/**
 *  @b Description
 *  @n
 *      This function initializes the UIA producer buffers with
 *      UIA packet and event headers.
 *
 *  @param[in]  handle
 *      Pointer to the procuder configuration
 *  @param[in]  buffer
 *      Pointer to the buffer to be initialzed
 *  @param[in]  src
 *      Source id used by loggerStreamer to identify the source of the buffer
 *  @param[in]  instance
 *      Logger Instance Id, it is created dynamically on ARM by DAT module
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_initBufferUIA (void* handle, void* buffer, uint16_t src, uint16_t instance)
{
    LoggerStreamer2_initBuffer(buffer, 32, instance,  src);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the DAT producer to initialize
 *      and prime the buffers for initial use.
 *
 *  @param[in]  handle
 *      Handle to the producer
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dat_primeProducerUIA (void* handle)
{

    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is registered with the logger streamer object to
 *      be invoked when the logger buffer is full and needs to be
 *      exchanged with a newer buffer.
 *
 *  @param[in]  handle
 *      Logger Streamer object
 *  @param[out] fillBuffer
 *      Pointer to the buffer which is to be exchanged
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the allocated buffer
 */
Ptr Dat_exchangeFunction(LoggerStreamer2_Handle handle, uint8_t* fillBuffer)
{
    Dat_Producer*   ptrProducer;
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint32_t        queueDepth;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);

    if(ptrProducer == NULL )
        return fillBuffer;

    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Increment the stats for the number of buffer being exchanged. */
    ptrProducer->stats.bufferExchange++;

    /* Keep track of the maximum queue depth of the pending queue. */
    queueDepth = Qmss_getQueueEntryCount(ptrProducer->pendingQueueHandle);
    if (queueDepth > ptrProducer->stats.maxQueueDepth)
        ptrProducer->stats.maxQueueDepth = queueDepth;

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle,
                                 ptrProducer->cfg.bufferSize);
    if (ptrPkt == NULL)
    {
        /* Error: There are no more packets available to keep up with the producer */
        ptrProducer->stats.bufferOverrun++;
        return fillBuffer;
    }

    /* Push the previous production packet to the pending queue. */
    Qmss_queuePushDesc (ptrProducer->pendingQueueHandle, (void*)ptrProducer->ptrProductionPkt);

    /* Get the data buffer associated with the */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Remember the new packet which has been given for production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Return the new data buffer. */
    return (void*)ptrDataBuffer;
}


/**********************************************************************
 ********************* Functions for CUIA LoggerStreamer2 *************
 **********************************************************************/
/**
 *  @b Description
 *  @n
 *      Function to prevent the current thread from being preempted, used by LoggerStreammer2.
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Current preemption enable state when the function was entered.
 */
UInt disablePreemption(void)
{
    /* Aquire spin Lock */
    hplib_mSpinLockLock (&loggerLock);

    return ((UInt)&loggerLock);
}

/**
 *  @b Description
 *  @n
 *      Function to restore the preempton enable state.
 *
 *  @param[in]  key
 *      Preemption enable state key value.
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void restorePreemption(UInt key)
{
    /* Release spin Lock */
    hplib_mSpinLockUnlock (&loggerLock);
}

/**
 *  @b Description
 *  @n
 *      The function is called from loggerStreamer2 interface when the logger
 *      buffer is full and needs to be exchanged with a newer buffer.
 *
 *  @param[in]  obj
 *      Logger Streamer object
 *  @param[out] full
 *      Pointer to the buffer which is to be exchanged
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the allocated buffer
 */
Ptr bufferExchange(Ptr obj, Char *full)
{
   return ( (Ptr)Dat_exchangeFunction ((void *)obj, (void *)full));
}

/**
 *  @b Description
 *  @n
 *      This funtion will returne 64bit tiemstamp to be used by LoggerStreamer2.
 *  The timestamp is returned in 2 32bits value format. In order to be correlated to the
 *  events loggered from DSP cores, same timestamp source should be used as DSP.
 *
 *  @param[in]  lsw
 *       Pointer to store lower 32bit of timestamp
 *  @param[in]  msw
 *       Pointer to store higher 32bit of timestamp
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void getTimestamp(UInt32* lsw, UInt32* msw)
{
    unsigned long long timeStamp;

    timeStamp = hplib_mUtilGetTimestamp();

    *lsw = timeStamp & 0xFFFFFFFF;
    *msw = (timeStamp >> 32 ) & 0xFFFFFFFF ;
}


/**
 *  @b Description
 *  @n
 *      This funtion will return 64bit CPU frequency value to be used by LoggerStreamer2.
 *  The 32 LSBs of the timestamp frequency are to be written into the address pointed to by lsw.
 *  The 32 MSBs of the timestamp frequency are to be written into the address pointed to by msw.
 *
 *  @param[in]  lsw
 *       Pointer to store lower 32bit of timestamp frequency
 *  @param[in]  msw
 *       Pointer to store higher 32bit of timestamp frequency
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void getTimestampFreq(Uint32* lsw, Uint32* msw)
{
    /* HPlib uses ARM clock as time stemap source.
       Here the rate is ARM clcok rate = 125000000
    */
    *lsw = (uint32_t)0x7735940;
    *msw = 0;
}

/**
 *  @b Description
 *  @n
 *      This funtion will return 64bit CPU timestamp to be used by LoggerStreamer2 LogSync.
 *  The 32 LSBs of the timestamp are to be written into the address pointed to by lsw.
 *  The 32 MSBs of the timestamp are to be written into the address pointed to by msw.
 *
 *  @param[in]  lsw
 *       Pointer to store lower 32bit of timestamp
 *  @param[in]  msw
 *       Pointer to store higher 32bit of timestamp
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void getGlobalTimestamp(LoggerStreamer2_Handle handle, Uint32* lsw, Uint32* msw)
{
    Dat_Producer*       ptrProducer;
    Dat_ClientMCB*      ptrDatClient;

    *lsw = 0;
    *msw = 0;

    if(handle == NULL)
        return;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);
    if (ptrProducer != NULL )
    {
       ptrDatClient = (Dat_ClientMCB *)ptrProducer->clientHandle;

       /* Use EMU counter as the Global timestamp */
       *lsw = Dat_uio_read32(ptrDatClient->ptrUioMapInfo, 0);
       *msw = Dat_uio_read32(ptrDatClient->ptrUioMapInfo, 4);
    }
}

/**
 *  @b Description
 *  @n
 *      This funtion will return 64bit CPU timestamp Frequency to be used by LoggerStreamer2 LogSync.
 *  The 32 LSBs of the timestamp frequency are to be written into the address pointed to by lsw.
 *  The 32 MSBs of the timestamp frequency are to be written into the address pointed to by msw.
 *
 *  @param[in]  lsw
 *       Pointer to store lower 32bit of timestamp frequency
 *  @param[in]  msw
 *       Pointer to store higher 32bit of timestamp frequency
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void getGlobalTimestampFreq(LoggerStreamer2_Handle handle, Uint32* lsw, Uint32* msw)
{
    Dat_Producer*       ptrProducer;
    Dat_ClientMCB*      ptrDatClient;

    *lsw = 0;
    *msw = 0;

    if(handle == NULL)
        return;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);
    if (ptrProducer != NULL )
    {
       ptrDatClient = (Dat_ClientMCB *)ptrProducer->clientHandle;

       /* Use EMU counter as the Global timestamp */
       *lsw = ptrDatClient->cfg.globalTsFreq.lo;
       *msw = ptrDatClient->cfg.globalTsFreq.hi;
    }
}

/**
 *  @b Description
 *  @n
 *      LoggerStreamer Helper function
 *  This funtion allocates memory to be used in loggerStreamer
 *
 *  @param[in]  context
 *       DAT Producer handle passed to loggerStreamer2
 *  @param[in]  num_bytes
 *       Number of bytes need to be allocated
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
xdc_Ptr cuia_alloc (xdc_Ptr context, UInt32 num_bytes)
{
    xdc_Ptr                 result = NULL;
    Dat_Producer*           ptrDatProducer;
    Dat_ClientMCB*          ptrDatClient;

    ptrDatProducer = (Dat_Producer*)context;
    ptrDatClient   = (Dat_ClientMCB*)ptrDatProducer->clientHandle;

    result = ptrDatClient->cfg.malloc((size_t)num_bytes, 4);

    return (result);
}

/**
 *  @b Description
 *  @n
 *      LoggerStreamer Helper function
 *  This fucntion frees memory allocated to loggerStreamer
 *
 *  @param[in]  context
 *       DAT Producer handle passed to loggerStreamer2
 *  @param[in]  dataPtr
 *       Pointer to the memory to be freed.
 *  @param[in]  num_bytes
 *       Number of bytes need to be freed.
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
Void cuia_free (xdc_Ptr context, xdc_Ptr dataPtr, UInt32 num_bytes)
{
    Dat_Producer*           ptrDatProducer;
    Dat_ClientMCB*          ptrDatClient;

    ptrDatProducer = (Dat_Producer*)context;
    ptrDatClient   = (Dat_ClientMCB*)ptrDatProducer->clientHandle;

    ptrDatClient->cfg.free(dataPtr, num_bytes);
}

/**
 *  @b Description
 *  @n
 *      This function initializes the general-purpose producer buffers with
 *      UIA packet and event headers.
 *
 *  @param[in]  handle
 *      Pointer to the procuder configuration
 *  @param[in]  buffer
 *      Pointer to the buffer to be initialzed
 *  @param[in]  src
 *      Source id used by loggerStreamer to identify the source of the buffer
 *  @param[in]  instance
 *      Logger Instance Id, it is created dynamically on ARM by DAT module
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_initBufferGP (void* handle, void* buffer, uint16_t src, uint16_t instance)
{
    uint32_t        bufferSize;
    uint32_t        dataSize;
    uint32_t*       ptrEventHeader;
    uint32_t*       writePtr;
    uint8_t*        ptrDataBuffer;
    uint32_t        eventSize;
    uint32_t        numPaddingBytes;
    Dat_ProducerCfg*  ptrProducerCfg;

    /* Initialize the UIA packet header. */
    LoggerStreamer2_initBuffer(buffer, 32, instance, src);

    /* Get the producer configuration. */
    ptrProducerCfg = (Dat_ProducerCfg*)handle;

    /* Get the buffer size :
       Note: Buffer size should be set when logger stream instance is created.
             Buffer size should include UIA/EVENT header size.
     */
    bufferSize       = (uint32_t)ptrProducerCfg->bufferSize;
    eventSize        = bufferSize - DAT_UIA_PKT_HEADER_SIZE;
    dataSize         = (eventSize - DAT_UIA_EVT_HEADER_SIZE) & 0xfffffffc;
    numPaddingBytes  = (eventSize - DAT_UIA_EVT_HEADER_SIZE) & 0x3;

    /* Set the event length in the packet. */
    UIAPacket_setEventLength((UIAPacket_Hdr*)buffer, bufferSize);

    /* Initialize the UIA event header. */
    /* Module's ID (Reference: Text.xs) */
    xdc_runtime_Types_ModuleId mid = Module__MID;

    /* Initialize the event header, including timestamps */
    ptrEventHeader = (uint32_t*)((uint32_t)buffer + DAT_UIA_PKT_HEADER_SIZE);

    /* Save the start of the data buffer. */
    ptrDataBuffer = (uint8_t*)ptrEventHeader + DAT_UIA_EVT_HEADER_SIZE;

    /* Fill in the event header fields.
       HdrType_EventWithSnapshotIdAndTimestamp:
        word0: EventHdr
        word1: Timestamp lower 32 bits
        word2: Timestamp upper 32 bits
        word3: event Id (top 16 bits) & module Id (bottom 16 bits)
        word4: filename pointer
        word5: linenum
        word6: snapshotId
        word7: address where the data was located
        word8: total length of data (top 16-bits)
               length for this record (bottom 16 bits)
        word9: format pointer
        data:  the rest of the record contains the data
    */
    writePtr = ptrEventHeader;
    *(writePtr++) = ti_uia_runtime_EventHdr_genEventHdrWord1(eventSize, 0,
                    ti_uia_runtime_EventHdr_HdrType_EventWithSnapshotIdAndTimestamp);

    /* Prepare for the timestamp */
    *(writePtr++) = 0;
    *(writePtr++) = 0;
    *(writePtr++) = ((ti_uia_events_UIASnapshot_memoryRange) & 0xffff0000) | mid;
    *(writePtr++) = (IArg)__FILE__;
    *(writePtr++) = (IArg)__LINE__;
    *(writePtr++) = 0;
    *(writePtr++) = (Bits32)ptrDataBuffer;
    *(writePtr++) = (((dataSize & 0xFFFF)<<16) | (dataSize & 0xFFFF));

    /* The format string used by System Analyzer to identify what the snapshot
     * data represents. */
    *(writePtr++) = (IArg)ptrProducerCfg->name;

    /* Increment the write pointer to end of the data section. */
    writePtr = (uint32_t*)(ptrDataBuffer + dataSize);

    /* Add an invalid UIA header for the rest of the data in the buffer. */
    if(numPaddingBytes)
        UIAPacket_setInvalidHdr (writePtr, numPaddingBytes);

    /* Clear the buffer (skip the UIA headers) if configured. */
    if (ptrProducerCfg->clearBuffer)
        memset ((void*)ptrDataBuffer, 0, dataSize);
}

/**
 *  @b Description
 *  @n
 *      The function closes the active buffer and returns a new buffer.
 *      If the consumer is slow, the same buffer will be returned again.
 *      This is applicable only for general-purpose producers.
 *
 *  @param[in]  handle
 *      Logger Streamer handle.
 *  @param[out] fillBuffer
 *      Active buffer that has to be closed
 *
 *  \ingroup DAT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to new buffer
 *  @retval
 *      Error   -   NULL. This occurs if an invalid producer handle
 *                        or NULL buffer is passed.
 */
Ptr Dat_bufferExchangeGP(LoggerStreamer2_Handle handle, uint8_t* fillBuffer)
{
    Dat_Producer*   ptrProducer;
    Dat_ClientMCB*  ptrDatClient;
    Ti_Pkt*         ptrPkt;
    uint32_t        queueDepth;
    uint8_t*        ptrDataBuffer;
    uint32_t        dataBufferSize;
    uint32_t        sequenceCount;
    UIAPacket_Hdr*  ptrUIAPktHeader;
    uint32_t*       ptrUIAEvtHeader;
    uint32_t        bufferSize;
    uint32_t        temp;
    uint32_t        tsHi=0;
    uint32_t        tsLo=0;

    /* Sanity Check: Ensure that the parameters are valid. */
    if ((handle == NULL) || (fillBuffer == NULL))
        return NULL;

    /* Get the producer handle. */
    ptrProducer = (Dat_Producer*)LoggerStreamer2_getContext((LoggerStreamer2_Handle)handle);

    if(ptrProducer == NULL )
        return fillBuffer;

    /* Get the DAT Client handle. */
    ptrDatClient = (Dat_ClientMCB*)ptrProducer->clientHandle;

    /* Increment the stats for the number of buffer being exchanged. */
    ptrProducer->stats.bufferExchange++;

    /* Get the buffer size */
    bufferSize = (uint32_t)ptrProducer->cfg.bufferSize;

    /* Adding UIA and Event Header at the begging of the buffer */
    ptrUIAPktHeader = (UIAPacket_Hdr*)((uint32_t)fillBuffer - DAT_UIA_HEADER_OFFSET);
    ptrUIAEvtHeader = (uint32_t*)((uint32_t)ptrUIAPktHeader + DAT_UIA_PKT_HEADER_SIZE);

    /* Update sequence number in UIA packet and event header. */
    sequenceCount = ptrProducer->sequenceCount;
    UIAPacket_setSequenceCount (ptrUIAPktHeader, sequenceCount);
    temp = EventHdr_setSeqCount (*ptrUIAEvtHeader, sequenceCount);
    *(ptrUIAEvtHeader++) = temp;

    /* Add Time stamp */
    getTimestamp((UInt32 *)&tsLo, (UInt32 *)&tsHi);
    *(ptrUIAEvtHeader++) = tsLo;
    *(ptrUIAEvtHeader++) = tsHi;

    /* Keep track of the maximum queue depth of the pending queue. */
    queueDepth = Qmss_getQueueEntryCount(ptrProducer->pendingQueueHandle);
    if (queueDepth > ptrProducer->stats.maxQueueDepth)
        ptrProducer->stats.maxQueueDepth = queueDepth;

    /* Allocate a packet from the producer heap */
    ptrPkt = Pktlib_allocPacket (ptrDatClient->cfg.pktlibInstHandle, ptrProducer->cfg.heapHandle, bufferSize);
    if (ptrPkt == NULL)
    {
        /* Error: There are no more packets available to keep up with the producer */
        ptrProducer->stats.bufferOverrun++;
        return fillBuffer;
    }

    /* Push the previous production packet to the pending queue. */
    Qmss_queuePushDesc (ptrProducer->pendingQueueHandle, (void*)ptrProducer->ptrProductionPkt);

    /* Get the data buffer associated with the new packet */
    Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &dataBufferSize);

    /* Increment the sequence number */
    ptrProducer->sequenceCount++;

    /* Clear the buffer (skip the UIA headers) if configured. */
    if (ptrProducer->cfg.clearBuffer)
        memset ((void*)(ptrDataBuffer + DAT_UIA_HEADER_OFFSET), 0,
                (bufferSize - DAT_UIA_HEADER_OFFSET));

    /* Remember the new packet which has been given for production. */
    ptrProducer->ptrProductionPkt = ptrPkt;

    /* Return the new data buffer. */
    return (void*)(ptrDataBuffer + DAT_UIA_HEADER_OFFSET);

}

/* Logger Function table for UIA type of Producer */
Dat_LoggerFuncTbl Dat_LoggerFuncUIA =
{
    Dat_initBufferUIA,
    NULL,
    NULL
};
/* Logger Function table for General Purpose type of Producer */
Dat_LoggerFuncTbl Dat_LoggerFuncGP =
{
    Dat_initBufferGP,
    NULL,
    Dat_bufferExchangeGP
};

#endif /* __ARMv7 */

