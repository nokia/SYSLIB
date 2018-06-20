/**
 *   @file  core1_osal.c
 *
 *   @brief
 *      Core1 OSAL Implementation
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012, Texas Instruments, Inc.
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
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>
#include <ti/sysbios/hal/Timer.h>

/* CSL Include Files */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/csl_chipAux.h>

/* IPC includes */
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>

/* PDK & CSL Include Files */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/csl_ipc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/rm/rm.h>

/* SA LLD include */
#include <ti/drv/sa/sa_osal.h>

/* SYSLIB Include Files.  */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

/**********************************************************************
 ************************** Local Definitions *************************
 **********************************************************************/

#define CPPI_HW_SEM             1
#define QMSS_HW_SEM             2
#define RM_HW_SEM               3
#define NAMED_RES_HW_SEM        4

/* Define this option during DEBUG and it can be turned off during final release
 * as it can help debug cache misaligned buffers */
#define CACHE_ALIGNMENT_CHECK

/**********************************************************************
 *************************** OSAL Functions ***************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which convers a global address to a local address
 *
 *  @param[in]  globalAddress
 *      Global Address to be converted
 *
 *  @retval
 *      Local address
 */
static uint32_t global_l2_address(uint32_t globalAddress)
{
    return (globalAddress & 0x00ffffff);
}

/**
 *  @b Description
 *  @n
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
uint32_t l2_global_address (uint32_t addr)
{
	uint32_t corenum;

	/* Get the core number. */
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

	/* Compute the global address. */
    if ((addr >= 0x800000) && (addr < 0x900000))
	    return (addr + (0x10000000 + (corenum*0x1000000)));
    else
        return addr;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *  @param[in]  checkAlign
 *       Flags that enables checks for address cache line alignment
 *
 *  @retval
 *      Not Applicable
 */
static void Osal_beginMemAccess (void *ptr, uint32_t size, uint32_t checkAlign)
{
	UInt  key;

#ifdef CACHE_ALIGNMENT_CHECK
    /* Validate the address only if required to do so. */
    if (checkAlign == 1)
    {
		/* Check if DDR3 address is 128 byte aligned */
		if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x80) != 0))
			asm (" swbp 0");

		/* Check if MSMC address is 64 byte aligned */
		if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0))
			asm (" swbp 0");
	}
#endif /* CACHE_ALIGNMENT_CHECK */

	/* Disable Interrupts */
	key = Hwi_disable();

	/*  Cleanup the prefetch buffer also. */
	CSL_XMC_invalidatePrefetchBuffer();

	/* Invalidate the cache. */
    CACHE_invL1d  (ptr, size, CACHE_FENCE_WAIT);

	asm	(" nop	4");
	asm	(" nop	4");
	asm	(" nop	4");
	asm	(" nop	4");

	/* Reenable Interrupts. */
	Hwi_restore(key);
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *  @param[in]  checkAlign
 *       Flags that enables checks for address cache line alignment
 *
 *  @retval
 *      Not Applicable
 */
static void Osal_endMemAccess (void *ptr, uint32_t size, uint32_t checkAlign)
{
	UInt  key;

#ifdef CACHE_ALIGNMENT_CHECK
    /* Validate the address only if required to do so. */
    if (checkAlign == 1)
    {
		/* Check if DDR3 address is 128 byte aligned */
		if ((((int)ptr & 0x80000000) == 0x80000000) && (((int)ptr % 0x80) != 0))
			asm (" swbp 0");

		/* Check if MSMC address is 64 byte aligned */
		if ((((int)ptr & 0x0C000000) == 0x0C000000) && (((int)ptr % 0x40) != 0))
			asm (" swbp 0");
	}
#endif /* CACHE_ALIGNMENT_CHECK */

	/* Disable Interrupts */
	key = Hwi_disable();

	/* Writeback the contents of the cache. */
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);

	asm	 (" nop	4");
	asm	 (" nop	4");
	asm	 (" nop	4");
	asm	 (" nop	4");

	/* Reenable Interrupts. */
	Hwi_restore(key);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the application to invalidate the buffer
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void appInvalidateBuffer(void* ptr, uint32_t size)
{
	Osal_beginMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the application to writeback the buffer
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void appWritebackBuffer(void* ptr, uint32_t size)
{
	Osal_endMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a memory block of the specified size.
 *      from shared memory.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Allocated block address
 */
Ptr Osal_cppiMalloc (UInt32 num_bytes)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, num_bytes, CACHE_L2_LINESIZE, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a memory block of the specified size allocated
 *      using Osal_cppiMalloc() API.
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up.
 *  @param[in]  size
 *      Size of the memory block to be cleaned up.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_cppiFree (Ptr ptr, UInt32 size)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter a critical section.
 *      Function protects against
 *
 *      access from multiple cores
 *      and
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_cppiCsEnter (Void)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (CPPI_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_cppiCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_cppiCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (CPPI_HW_SEM);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
{
	Osal_beginMemAccess(ptr, size, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
{
	Osal_endMemAccess(ptr, size, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter a critical section.
 *      Function protects against
 *
 *      access from multiple cores
 *      and
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_qmssCsEnter (Void)
{
    /* Get the hardware semaphore */
    while ((CSL_semAcquireDirect (QMSS_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_qmssCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_qmssCsExit (Ptr CsHandle)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (QMSS_HW_SEM);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter a critical section.
 *      Function protects against
 *      access from multiple threads on single core
 *
 *  @retval
 *      Handle used to lock critical section
 */
Ptr Osal_qmssMtCsEnter (Void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_qmssCsEnter() API.
 *
 *  @param[in]  CsHandle
 *      Handle for unlocking critical section.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_qmssMtCsExit (Ptr CsHandle)
{
    /* Release Semaphore using handle */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. If the memory block is cached then this
 *      indicates that the application would need to ensure that the
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
{
	Osal_beginMemAccess(ptr, size, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that the block of memory has
 *      finished being accessed. If the memory block is cached then the
 *      application would need to ensure that the contents of the cache
 *      are updated immediately to the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
{
	Osal_endMemAccess(ptr, size, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is used to traverse a CPPI descriptor and convert all
 *      address references from virtual to physical.
 *
 *  @param[in]  QID
 *      16-bit queue ID which can be used to quickly access translation
 *      table which could be different per queue.
 *  @param[in]  descAddr
 *      Descriptor address
 *
 *  @retval
 *      Virtual/logical address
 */
void* Osal_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr)
{
    /* On the DSP there is no conversion required between the PHYSICAL and VIRTUAL address */
    return descAddr;
}

/**
 *  @b Description
 *  @n
 *      The function is used to traverse a CPPI descriptor and convert all
 *      address references from physical to virtual.
 *
 *  @param[in]  QID
 *      16-bit queue ID which can be used to quickly access translation
 *      table which could be different per queue.
 *  @param[in]  descAddr
 *      Descriptor address
 *
 *  @retval
 *      Virtual/logical address
 */
void* Osal_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr)
{
    /* On the DSP there is no conversion required between the PHYSICAL and VIRTUAL address */
    return descAddr;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the programming of the accumulator
 *      by entering the critical section.
 *
 *  @retval
 *      Handle to the critical section.
 */
void* Osal_qmssAccCsEnter (void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the programming of the accumulator
 *      and by exiting the critical section.
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_qmssAccCsExit (void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the PKTLIB instance to allocate
 *      memory for the PKTLIB module.
 *
 *  @param[in]  mode
 *      PKTLIB memory allocation mode
 *  @param[in]  numBytes
 *      Size of the buffer to be allocated
 *
 *  @retval
 *      Allocated memory
 */
void* Pktlib_osalMalloc(Pktlib_MallocMode mode, uint32_t numBytes)
{
    if (mode == Pktlib_MallocMode_LOCAL)
        return Memory_alloc((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, 0, NULL);
    else
        return Memory_alloc(SharedRegion_getHeap(1), numBytes, 0, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the PKTLIB instance to free
 *      memory for the PKTLIB module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the buffer to be cleaned up
 *  @param[in]  numBytes
 *      Size of the buffer to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_osalFree(Pktlib_MallocMode mode, void* ptr, uint32_t numBytes)
{
    if (mode == Pktlib_MallocMode_LOCAL)
        Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
    else
        Memory_free (SharedRegion_getHeap(1), ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the PKTLIB instance to invalidate the
 *      memory from the cache
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  numBytes
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_osalBeginMemAccess(void* ptr, uint32_t numBytes)
{
    Osal_beginMemAccess(ptr, numBytes, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the PKTLIB instance to writeback
 *      memory from the cache
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  numBytes
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_osalEndMemAccess(void* ptr, uint32_t numBytes)
{
    Osal_endMemAccess(ptr, numBytes, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the PKTLIB to invalidate the packet contents
 *
 *  @param[in]  heapHandle
 *      Heap Handle to which the packet belongs.
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  size
 *      Size of the packet
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_osalBeginPktAccess(Pktlib_HeapHandle heapHandle, Ti_Pkt* ptrPkt, uint32_t size)
{
    Osal_beginMemAccess(ptrPkt, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the PKTLIB to writeback the packet contents
 *
 *  @param[in]  heapHandle
 *      Heap Handle to which the packet belongs.
 *  @param[in]  ptrPkt
 *      Pointer to the packet
 *  @param[in]  size
 *      Size of the packet
 *
 *  @retval
 *      Not applicable
 */
void Pktlib_osalEndPktAccess(Pktlib_HeapHandle heapHandle, void* ptrPkt, uint32_t size)
{
    Osal_endMemAccess(ptrPkt, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the PKTLIB module to enter a critical section
 *
 *  @param[in]  heapHandle
 *      Heap Handle
 *
 *  @retval
 *      Critical Section Handle
 */
void* Pktlib_osalEnterCS(Pktlib_HeapHandle heapHandle)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the PKTLIB module to exit a critical section
 *
 *  @param[in]  heapHandle
 *      Heap Handle
 *  @param[in]  csHandle
 *      Critical Section Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Pktlib_osalExitCS(Pktlib_HeapHandle heapHandle, void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to allocate memory for its
 *     internal data structures.
 *
 *  @param[in]  mode
 *      Memory mode
 *  @param[in]  numBytes
 *      Number of bytes to be allocated
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
void* Msgcom_osalMalloc(Msgcom_MemAllocMode mode, uint32_t numBytes)
{
    void* ptr;

	if (mode == Msgcom_MemAllocMode_CACHE_COHERENT)
        ptr = (void *)l2_global_address ((uint32_t)Memory_alloc (NULL, numBytes, 128, NULL));
    else
        ptr = (void *)((uint32_t)Memory_alloc((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, 128, NULL));

    return ptr;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to free memory which had been
 *     previously allocated
 *
 *  @param[in]  mode
 *      Memory mode
 *  @param[in]  ptr
 *      Pointer to the memory to be freed
 *  @param[in]  numBytes
 *      Number of bytes to be freed
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalFree(Msgcom_MemAllocMode mode, void* ptr, uint32_t numBytes)
{
	if (mode == Msgcom_MemAllocMode_CACHE_COHERENT)
    {
        /* The address passed here is a global address and this needs to be converted
         * back to a local address before it can get placed back into the heap. */
        Memory_free (NULL, (void *)global_l2_address((uint32_t)ptr), numBytes);
    }
    else
    {
        /* Cleanup the memory as is. */
        Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is invoked by the MSGCOM module on channel creation to allow the
 *     operating system to register the MSGCOM ISR for the specific queue.
 *
 *  @param[in]  channelName
 *      Name of the MSGCOM channel for which the interrupt is plugged.
 *  @param[in]  queueInfo
 *      Queue Information for which the interrupt is being registered
 *  @param[in]  isr
 *      MSGCOM provided ISR function which is to be plugged into the operating system. Please
 *      ensure that the "chHandle" specified above is registered as an argument.
 *  @param[in]  chHandle
 *      MSGCOM channel handle. This should be registered as an argument while registering the "isr"
 *  @param[in] ptrInterruptInfo
 *      Pointer to the interrupt information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Msgcom_osalRegisterIsr
(
    const char*         channelName,
    Qmss_Queue          queueInfo,
    MsgCom_Isr          isr,
    MsgCom_ChHandle     chHandle,
    MsgCom_Interrupt*   ptrInterruptInfo
)
{
    int32_t eventId;

    /* DSP Realm OSAL functions need to handle both the System Interrupts and Accumulated
     * interrupts since these are routed differently */
    if (ptrInterruptInfo->sysInterrupt == -1)
    {
        /* Accumulated Interrupt: Route the interrupt via the DSP INTC Event combiner module. */
        EventCombiner_dispatchPlug (ptrInterruptInfo->hostInterrupt, (EventCombiner_FuncPtr)isr, (UArg)chHandle, TRUE);
        EventCombiner_enableEvent(ptrInterruptInfo->hostInterrupt);

        /* Debug Message: */
        System_printf ("Debug: [Enabling] Queue Manager %d Accumulated Queue %d -> Event Id %d\n",
                        queueInfo.qMgr, queueInfo.qNum, ptrInterruptInfo->hostInterrupt);
    }
    else
    {
        /* Queue Pend System Interrupt: Map the System Interrupt to the specified ISR */
        CpIntc_dispatchPlug(ptrInterruptInfo->sysInterrupt, (CpIntc_FuncPtr)isr, (UArg)chHandle, TRUE);

        /* We map system interrupt to Host Interrupt X; which has already been mapped by the resource manager. */
        CpIntc_mapSysIntToHostInt(ptrInterruptInfo->cpIntcId, ptrInterruptInfo->sysInterrupt, ptrInterruptInfo->hostInterrupt);

        /* Enable the Host Interrupt. */
	    CpIntc_enableHostInt(ptrInterruptInfo->cpIntcId, ptrInterruptInfo->hostInterrupt);

        /* Get the event id associated with the host interrupt. */
        eventId = CpIntc_getEventId(ptrInterruptInfo->hostInterrupt);

        /* Plug the CPINTC Dispatcher. */
        EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, ptrInterruptInfo->hostInterrupt, TRUE);

        /* Debug Message: */
        System_printf ("Debug: [Enabling] Queue Manager %d Queue Number %d System Interrupt %d -> HostInterrupt %d -> EventId %d \n",
                       queueInfo.qMgr, queueInfo.qNum, ptrInterruptInfo->sysInterrupt, ptrInterruptInfo->hostInterrupt, eventId);
    }
    /* Mapping was successfully completed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *     The function is invoked by the MSGCOM module on channel deletion to allow the
 *     operating system to deregister the MSGCOM ISR for the specific queue.
 *
 *  @param[in]  channelName
 *      Name of the MSGCOM channel for which the interrupt is being deregistered.
 *  @param[in]  queueInfo
 *      Queue Information for which the interrupt is being deregistered
 *  @param[in] ptrInterruptInfo
 *      Pointer to the interrupt information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Msgcom_osalDeregisterIsr
(
    const char*         channelName,
    Qmss_Queue          queueInfo,
    MsgCom_Interrupt*   ptrInterruptInfo
)
{
    int32_t eventId;

    /* DSP Realm OSAL functions need to handle both the System Interrupts and Accumulated
     * interrupts since these are routed differently */
    if (ptrInterruptInfo->sysInterrupt == -1)
    {
        /* Accumulated Interrupt: Channel was hooked up directly to the DSP Interrupt controller so simply
         * disable the event. */
        EventCombiner_disableEvent(ptrInterruptInfo->hostInterrupt);

        /* Debug Message: */
        System_printf ("Debug: [Disabling] Queue Manager %d Accumulated Queue %d -> Event Id %d\n",
                        queueInfo.qMgr, queueInfo.qNum, ptrInterruptInfo->hostInterrupt);
    }
    else
    {
        /* Queue Pend System Interrupt: Clear the system interrupt. */
        CpIntc_clearSysInt(ptrInterruptInfo->cpIntcId, ptrInterruptInfo->sysInterrupt);

        /* Disable the system interrupt. */
        CpIntc_disableSysInt(ptrInterruptInfo->cpIntcId, ptrInterruptInfo->hostInterrupt);

        /* Disable the host interrupt. */
        CpIntc_disableHostInt (ptrInterruptInfo->cpIntcId, ptrInterruptInfo->hostInterrupt);

        /* Get the event id associated with the host interrupt. */
        eventId = CpIntc_getEventId(ptrInterruptInfo->hostInterrupt);

        /* Disable the event. */
        EventCombiner_disableEvent(eventId);

        /* Debug Message: */
        System_printf ("Debug: [Disabling] Queue Manager %d Queue Number %d System Interrupt %d -> HostInterrupt %d -> EventId %d \n",
                       queueInfo.qMgr, queueInfo.qNum, ptrInterruptInfo->sysInterrupt, ptrInterruptInfo->hostInterrupt, eventId);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *     The function is invoked by the MSGCOM module to disable the system interrupt
 *
 *  @param[in] cpIntcId
 *      CPINTC Identifier on which the system interrupt is mapped.
 *  @param[in] sysIntr
 *      System Interrupt associated with the queue
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalDisableSysInt(int32_t cpIntcId, int32_t sysIntr)
{
    CpIntc_disableSysInt(cpIntcId, sysIntr);
}

/**
 *  @b Description
 *  @n
 *     The function is invoked by the MSGCOM module to enable the system interrupt
 *
 *  @param[in] cpIntcId
 *      CPINTC Identifier on which the system interrupt is mapped.
 *  @param[in] sysIntr
 *      System Interrupt associated with the queue
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalEnableSysInt(int32_t cpIntcId, int32_t sysIntr)
{
    CpIntc_enableSysInt(cpIntcId, sysIntr);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
void* Msgcom_osalEnterSingleCoreCS(void)
{
    return (void *)Hwi_disable();
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque handle to the criticial section object
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalExitSingleCoreCS(void* csHandle)
{
    UInt  key = (UInt)csHandle;
    Hwi_restore(key);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to create a semaphore
 *     for the MSGCOM blocking channel.
 *
 *  @retval
 *      Handle to the semaphore object.
 */
void* Msgcom_osalCreateSem(void)
{
    return (Void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to delete the semaphore
 *     which was opened by the blocking MSGCOM channel.
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore object
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalDeleteSem(void* semHandle)
{
    Semaphore_delete(semHandle);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to pend on the semaphore to
 *     implement the blocking channel functionality.
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore object
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalPendSem(void* semHandle)
{
    Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to post the semaphore since
 *     data has been received on the blocking channel and the callee needs to
 *     be woken up now.
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore object
 *
 *  @retval
 *      Not applicable
 */
void Msgcom_osalPostSem(void* semHandle)
{
    Semaphore_post(semHandle);
}

/**********************************************************************
 *********************** PASS OSAL Functions **************************
 **********************************************************************/

void Osal_paBeginMemAccess (Ptr addr, UInt32 size)
{
    /* There is no need to perform cache operations on the PA; since the PA
     * is accessed only by the NETFP Server which executes on a single core. */
    return;
}

void Osal_paEndMemAccess (Ptr addr, UInt32 size)
{
    /* There is no need to perform cache operations on the PA; since the PA
     * is accessed only by the NETFP Server which executes on a single core. */
    return;
}

void Osal_paMtCsEnter(void* key)
{
    /* There is no need to perform multicore or multithread protection since all
     * the PA calls are serially executed by the NETFP Server. */
    return;
}

void Osal_paMtCsExit(void* key)
{
    /* There is no need to perform multicore or multithread protection since all
     * the PA calls are serially executed by the NETFP Server. */
    return;
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory required by the resource manager module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  size
 *      Number of bytes of memory to be allocated
 *
 *  @retval
 *      Pointer to the allocated memory
 */
void* Resmgr_osalMalloc(Resmgr_MallocMode mode, uint32_t size)
{
    /* Use the shared memory pool for global memory allocations; else use the local
     * core specific default heap */
    if (mode == Resmgr_MallocMode_GLOBAL)
        return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), size, 0, NULL);
    else
        return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, size, 0, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to free memory which was allocated by the resource manager module.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the allocated memory
 *  @param[in]  size
 *      Number of bytes of memory which were allocated
 *
 *  @retval
 *      Not applicable.
 */
void Resmgr_osalFree(Resmgr_MallocMode mode, void* ptr, uint32_t size)
{
    if (mode == Resmgr_MallocMode_GLOBAL)
        Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, size);
    else
        Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to allocate memory for the memory region.
 *
 *  @param[in]  name
 *      Name of the memory region being inserted
 *  @param[in]  memRegType
 *      Memory region type being specified.
 *  @param[in]  numBytes
 *      Number of bytes of memory being allocated
 *
 *  @retval
 *      Pointer to the allocated memory region
 */
uint8_t* Resmgr_osalMallocMemoryRegion
(
    char*                   name,
    Resmgr_MemRegionType    memRegType,
    uint32_t                numBytes
)
{
    /* Check if we are allocating for MSMC */
    if (memRegType == Resmgr_MemRegionType_MSMC)
        return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(0), numBytes, 16, NULL);

    /* Check if we are allocating for DDR3 */
    if (memRegType == Resmgr_MemRegionType_DDR3)
        return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), numBytes, 16, NULL);

    /* Check if we are allocating for LOCAL */
    if (memRegType == Resmgr_MemRegionType_LOCAL)
        return (uint8_t*)l2_global_address((uint32_t)Memory_alloc (NULL, numBytes, 16, NULL));

    /* Control should NOT come here. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the resource manager to free memory
 *      which had been allocated for the memory region request.
 *
 *  @param[in]  name
 *      Name of the memory region for which memory is being cleaned up
 *  @param[in]  memRegType
 *      Memory Region Type
 *  @param[in]  ptr
 *      Pointer to the memory being cleaned up
 *  @param[in]  numBytes
 *      Number of bytes of memory which is to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalFreeMemoryRegion
(
    char*                   name,
    Resmgr_MemRegionType    memRegType,
    uint8_t*                ptr,
    uint32_t                numBytes
)
{
    /* Check if we are cleaning for MSMC */
    if (memRegType == Resmgr_MemRegionType_MSMC)
        Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(0), ptr, numBytes);

    /* Check if we are cleaning for DDR3 */
    if (memRegType == Resmgr_MemRegionType_DDR3)
        Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, numBytes);

    /* Check if we are cleaning for LOCAL */
    if (memRegType == Resmgr_MemRegionType_LOCAL)
        Memory_free (NULL, (void*)global_l2_address((uint32_t)ptr), numBytes);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore. The semaphore is used by the underlying
 *      RM layer to ensure protection against multiple threads.
 *
 *  @retval
 *      Opaque semaphore handle
 */
void* Resmgr_osalCreateSem (void)
{
    return (void*)Semaphore_create(1, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalPendSem (void* semHandle)
{
    Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalPostSem (void* semHandle)
{
    Semaphore_post(semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete the semaphore which had been created to allow the underlying
 *      RM layer protection against multiple threads.
 *
 *  @param[in]  semHandle
 *      Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalDeleteSem (void* semHandle)
{
    Semaphore_delete(semHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the resource manager to indicate that
 *      memory is about to be accessed and so the contents of the cache
 *      should be invalidated.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be accessed
 *  @param[in]  size
 *      Size of memory to be accessed
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalBeginMemAccess (void* ptr, uint32_t size)
{
    Osal_beginMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the resource manager to indicate that
 *      memory has been accessed and so the contents of the cache
 *      should be written back.
 *
 *  @param[in]  ptr
 *      Pointer to the memory
 *  @param[in]  size
 *      Size of memory
 *
 *  @retval
 *      Not applicable
 */
void Resmgr_osalEndMemAccess (void* ptr, uint32_t size)
{
    Osal_endMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the NETFP module to allocate a block of memory
 *      for its internal use.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Allocated block address
 */
void* Netfp_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the NETFP module to free a block of memory
 *
 *  @param[in]  ptr
 *      Pointer to the allocated memory block.
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Not Applicable.
 */
void  Netfp_osalFree(void* ptr, uint32_t numBytes)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
void* Netfp_osalEnterSingleCoreCriticalSection (void)
{
    return (void*)Hwi_disable();
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
void Netfp_osalExitSingleCoreCriticalSection (void* csHandle)
{
    Hwi_restore((UInt)csHandle);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
void Netfp_osalBeginMemoryAccess (void* ptr, uint32_t size)
{
    Osal_beginMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Netfp_osalEndMemoryAccess (void* ptr, uint32_t size)
{
    Osal_endMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a NETFP job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
void* Netfp_osalCreateSem (void)
{
    return (void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the NETFP job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_osalDeleteSem(void* semHandle)
{
    Semaphore_delete((Semaphore_Handle *)&semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the NETFP job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_osalPendSem(void* semHandle)
{
    Semaphore_pend (semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
void Netfp_osalPostSem(void* semHandle)
{
    Semaphore_post (semHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter a critical section.
 *      Function protects against
 *      access from multiple threads on single core
 *
 *  @param[in]  key
 *      Key used to lock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_saCsEnter (uint32_t *key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_salldCsEnter() API.
 *
 *  @param[in]  key
 *      Key used to unlock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_saCsExit (uint32_t key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter a critical section.
 *      Function protects against
 *
 *      access from multiple threads on single core
 *      and
 *      access from multiple cores
 *
 *  @param[in]  key
 *      Key used to lock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_saMtCsEnter (uint32_t *key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit a critical section
 *      protected using Osal_salldCsEnter() API.
 *
 *  @param[in]  key
 *      Key used to unlock the critical section.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_saMtCsExit (uint32_t key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the SA to invalidate the cache.
 *
 *  @param[in]  addr
 *      Pointer to the address to be invalidated
 *  @param[in]  size
 *      Size of the buffer
 *
 *  @retval
 *      None
 */
void Osal_saBeginMemAccess (Ptr addr, UInt32 size)
{
	Osal_beginMemAccess(addr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the SA to writeback the cache.
 *
 *  @param[in]  addr
 *      Pointer to the address to be written back
 *  @param[in]  size
 *      Size of the buffer
 *
 *  @retval
 *      None
 */
void Osal_saEndMemAccess (Ptr addr, UInt32 size)
{
	Osal_endMemAccess(addr, size, 1);
}

/**
 * @brief   The macro is used by the SA LLD to indicate that the security
 * context byuffer is about to be accessed. If the security context buffer
 * is cacheable then this indicates that the application would need to ensure
 * that the cache is updated with the data from the actual memory since the
 * security context will be updated by SASS Cache engine.
 * If the security context buffers are non-cacheable then these macros can
 * be defined to be NOP.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_saBeginScAccess (void* addr, uint32_t sizeWords)
    @endverbatim
 *
 *  <b> Parameters </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 */

Void Osal_saBeginScAccess (void* addr, UInt32 size)
{
    Osal_beginMemAccess(addr, size, 1);
}

/**
 * @brief   The macro is used by the SA LLD to indicate that the security
 * context buffer has finished being updated. If the memory block is cacheable
 * then the application would need to ensure that the contents of the cache are
 * updated immediately to the actual memory.
 * If the security context buffers are non-cacheable then these macros can
 * be defined to be NOP.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
        void Osal_saEndScAccess (void* addr, uint32_t sizeWords)
    @endverbatim
 *
 *  <b> Parameters </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 */

Void Osal_saEndScAccess   (void* addr, UInt32 size)
{
	Osal_endMemAccess(addr, size, 0);
}

/**
 *  @b Description
 *  @n
 *      The function is the SA LLD OSAL Logging API which logs
 *      the messages on the console.
 *
 *  @param[in]  fmt
 *      Formatted String.
 *
 *  @retval
 *      Not Applicable
 */
Void Osal_saLog ( String fmt, ... )
{

}

/**
 * @brief   The macro is used by the SA LLD to get the physical address
 * of the security context buffer or its system instance.
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      void* Osal_saGetSCPhyAddr(void* vaddr)
 *   @endverbatim
 *
 *  <b> Return Value Physical address of the Security Context buffer or system instance </b>
 *  <b> Parameters </b>
 *  @n  Virtual Address of the Security Context buffer.
 */

void* Osal_saGetSCPhyAddr(void* vaddr)
{
    return vaddr;
}

/**
 * @brief   The macro is used by the SA LLD to get the processor ID(zero based)
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      uint32_t Osal_saGetProcId ()
 *   @endverbatim
 *
 *  <b> Return Value Processor ID zero based </b>
 *  @n  Not applicable.
 */
uint16_t Osal_saGetProcId (void )
{
    return (uint16_t)CSL_chipReadReg(CSL_CHIP_DNUM);
}

/**
 * @brief   The macro is used by the SA LLD to the Endian mode of the system (SoC).
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
 *      int Osal_saGetSysEndianMode(void)
 *   @endverbatim
 *
 *  <b> Return Value Endian mode of the system (SoC) </b>
 *  <b> Parameters </b>
 *  @n  Endian mode of the system (SoC).
 */
int   Osal_saGetSysEndianMode(void)
{
#if defined( _BIG_ENDIAN )
    return((int)sa_SYS_ENDIAN_MODE_BIG);
#else
    return((int)sa_SYS_ENDIAN_MODE_LITTLE);
#endif
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to allocate memory.
 *
 *  @param[in]  numBytes
 *      Number of bytes of data which are to be allocated
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
void* Osal_rmMalloc (uint32_t numBytes)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, 0, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to cleanup memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up.
 *  @param[in]  numBytes
 *      Number of bytes of data which are to be cleaned up
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmFree (void* ptr, uint32_t numBytes)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to acquire a multicore/multithread
 *      safe lock.
 *
 *  @retval
 *      Not Applicable
 */
void *Osal_rmCsEnter(void)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (RM_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to release a multicore/multithread
 *      safe lock acquired earlier.
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmCsExit(void *CsHandle)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (RM_HW_SEM);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to invalidate the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      None
 */
void Osal_rmBeginMemAccess(void *ptr, uint32_t size)
{
	Osal_beginMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the RM to writeback the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void Osal_rmEndMemAccess(void *ptr, uint32_t size)
{
	Osal_endMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a task blocking object
 *      capable of blocking the task.
 *
 *  @retval
 *      Opaque handle 
 */
void *Osal_rmTaskBlockCreate(void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the RM from multiple thread
 *      access.
 *
 *  @param[in]  handle
 *      Opaque handle
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskBlock(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the RM from multiple thread
 *      access.
 *
 *  @param[in]  handle
 *      Opaque handle
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskUnblock(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a task blocking object
 *
 *  @param[in]  handle
 *      Opaque handle
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmTaskBlockDelete(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to log messages from the RM
 *
 *  @param[in]  fmt
 *      Format string with variable arguments
 *
 *  @retval
 *      Not Applicable
 */
void Osal_rmLog( char *fmt, ... )
{
    VaList ap;

    va_start(ap, fmt);
    System_vprintf(fmt, ap);
    va_end(ap);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Agent module to allocate memory.
 *      - In the DSP realm the memory allocated should be accessible across
 *        all the DSP cores.
 *
 *  @param[in]  size
 *      Number of bytes to allocate
 *  @param[in]  alignment
 *      Alignment requirements
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
void* Agent_osalMalloc(uint32_t size, uint32_t alignment)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, size, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Agent module to free memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes to freeup
 *
 *  @retval
 *      None
 */
void Agent_osalFree(void* ptr, uint32_t size)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Agent to invalidate the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void Agent_osalBeginMemAccess(void* ptr, uint32_t size)
{
	Osal_beginMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Agent to writeback the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      None
 */
void Agent_osalEndMemAccess(void* ptr, uint32_t size)
{
	Osal_endMemAccess(ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the Agent against concurrent
 *      access via multiple threads on the same core.
 *
 *  @retval
 *      Opaque handle.
 */
void* Agent_osalEnterSingleCoreCS(void)
{
    return (void*)Hwi_disable();
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to release the critical section against
 *      concurrent access from multiple threads
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Opaque handle.
 */
void Agent_osalExitSingleCoreCS(void* semHandle)
{
    Hwi_restore ((UInt)semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a JOB.
 *      If the jobs are executing in SYNC mode then once a JOB is submitted
 *      the callee will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
void* Agent_osalCreateSem(void)
{
    return (void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *      Each JOB is associated with a unique semaphore.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not applicable
 */
void Agent_osalDeleteSem(void* semHandle)
{
    /* Delete the semaphore. */
    Semaphore_delete ((Semaphore_Handle*)&semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the agent job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Agent_osalPendSem (void* semHandle)
{
    Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
void Agent_osalPostSem (void* semHandle)
{
    Semaphore_post(semHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the DAT module to allocate a block of memory
 *      for its internal use.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Allocated block address
 */
void* Dat_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the DAT module to free a block of memory
 *
 *  @param[in]  ptr
 *      Pointer to the allocated memory block.
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Not Applicable.
 */
void  Dat_osalFree(void* ptr, uint32_t numBytes)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the DAT module to allocate a block of local memory
 *      for its internal use.
 *
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Allocated block address
 */
void* Dat_osalMallocLocal(uint32_t numBytes, uint32_t alignment)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)L2SramPrivateHeap, numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the DAT module to free a block of local memory
 *
 *  @param[in]  ptr
 *      Pointer to the allocated memory block.
 *  @param[in]  num_bytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Not Applicable.
 */
void  Dat_osalFreeLocal(void* ptr, uint32_t numBytes)
{
    Memory_free ((xdc_runtime_IHeap_Handle)L2SramPrivateHeap, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *     DAT Critical Sections should NOT be implemented with a
 *     HWI Disable/Enable because the critical section is held for the
 *     entire duration while processing consumers/producers in the
 *     background thread.
 *
 *  @retval
 *      Opaque handle to the critical section object
 */
void* Dat_osalEnterSingleCoreCriticalSection (void)
{
    static Semaphore_Handle datSemHandle = NULL;

    /* Do we need to create the semaphore?
     * - Ensure that the semaphore is initially created with a count of 1
     *   since the semaphore is initially available. */
    if (datSemHandle == NULL)
        datSemHandle = Semaphore_create(1, NULL, NULL);

    /* Pend on the semaphore */
    Semaphore_pend (datSemHandle, BIOS_WAIT_FOREVER);
    return datSemHandle;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Dat_osalExitSingleCoreCriticalSection (void* csHandle)
{
    Semaphore_post ((Semaphore_Handle)csHandle);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
void Dat_osalBeginMemoryAccess (void* ptr, uint32_t size)
{
    Osal_beginMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the DAT module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Dat_osalEndMemoryAccess (void* ptr, uint32_t size)
{
    Osal_endMemAccess (ptr, size, 1);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a DAT job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
void* Dat_osalCreateSem (void)
{
    return (void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the DAT job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_osalDeleteSem(void* semHandle)
{
    Semaphore_delete((Semaphore_Handle *)&semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the DAT job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Dat_osalPendSem(void* semHandle)
{
    Semaphore_pend (semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
void Dat_osalPostSem(void* semHandle)
{
    Semaphore_post (semHandle);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Name database module to allocate a
 *      block of memory for its internal use.
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Allocated block address
 */
void* Name_osalDBMalloc(Name_MallocMode mode, uint32_t numBytes, uint32_t alignment)
{
    if (mode == Name_MallocMode_LOCAL)
        return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, alignment, NULL);
    else
        return Memory_alloc ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Name database module to free a block of memory
 *
 *  @param[in]  mode
 *      Memory allocation mode
 *  @param[in]  ptr
 *      Pointer to the allocated memory block.
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Not Applicable.
 */
void  Name_osalDBFree(Name_MallocMode mode,void* ptr, uint32_t numBytes)
{
    if (mode == Name_MallocMode_LOCAL)
        Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
    else
        Memory_free ((xdc_runtime_IHeap_Handle)SharedRegion_getHeap(1), ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the Name module to invalidate the
 *      memory from the cache
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  numBytes
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Name_osalBeginMemAccess(void* ptr, uint32_t numBytes)
{
    Osal_beginMemAccess(ptr, numBytes, 1);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the Name module to writeback
 *      memory from the cache
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  numBytes
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
void Name_osalEndMemAccess(void* ptr, uint32_t numBytes)
{
    Osal_endMemAccess(ptr, numBytes, 1);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to protect the named resource database against concurrent
 *      access. The named resource database is shared across multiple cores and so the
 *      implementation should use a well known hardware semaphore which should be the same
 *      across all the cores.
 *
 *  @retval
 *      Opaque critical section handle
 */
void* Name_osalEnterMultipleCoreCS(void)
{
    /* Get the hardware semaphore for protection against multiple core access */
    while ((CSL_semAcquireDirect (NAMED_RES_HW_SEM)) == 0);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to release the named resource database against concurrent
 *      access. The named resource database is shared across multiple cores and so the
 *      implementation should use a well known hardware semaphore which should be the same
 *      across all the cores.
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Name_osalExitMultipleCoreCS(void* csHandle)
{
    /* Release the hardware semaphore */
    CSL_semReleaseSemaphore (NAMED_RES_HW_SEM);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Name proxy/client module to allocate a
 *      block of memory for its internal use.
 *
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *  @param[in]  alignment
 *      Alignment requirements.
 *
 *  @retval
 *      Allocated block address
 */
void* Name_osalMalloc (uint32_t numBytes, uint32_t alignment)
{
    return Memory_alloc ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, alignment, NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used by the Name proxy/client module to free a block of memory
 *
 *  @param[in]  ptr
 *      Pointer to the allocated memory block.
 *  @param[in]  numBytes
 *      Number of bytes to be allocated.
 *
 *  @retval
 *      Not Applicable.
 */
void  Name_osalFree (void* ptr, uint32_t numBytes)
{
    Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
}

/**
 *  @b Description
 *  @n
 *      The function is registered with the Name module to enter a critical
 *      section
 *
 *  @retval
 *      Opaque critical section handle
 */
void* Name_osalEnterCS (void)
{
   return (void*)Hwi_disable();
}

/**
 *  @b Description
 *  @n
 *     The function is used by the Name module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Name_osalExitCS (void* csHandle)
{
    Hwi_restore((UInt)csHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to create a semaphore associated with a NAME job.
 *      If the jobs are executing in SYNC mode then once a job is submitted the callee
 *      will block on this semaphore until the result is received.
 *
 *  @retval
 *      Opaque Semaphore handle
 */
void* Name_osalCreateSem (void)
{
    return (void*)Semaphore_create(0, NULL, NULL);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the NAME job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Name_osalDeleteSem(void* semHandle)
{
    Semaphore_delete((Semaphore_Handle *)&semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the NAME job
 *      This is done once a sync job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
void Name_osalPendSem(void* semHandle)
{
    Semaphore_pend (semHandle, BIOS_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      once the result packet associated with the SYNC job has been received.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable
 */
void Name_osalPostSem(void* semHandle)
{
    Semaphore_post (semHandle);
}


/**
 *  @b Description
 *  @n
 *     The function is used by the MEMLOG module to allocate memory for its
 *     internal data structures.
 *
 *  @param[in]  mode
 *      Memory mode
 *  @param[in]  numBytes
 *      Number of bytes to be allocated
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
void* Memlog_osalMalloc(Msgcom_MemAllocMode mode, uint32_t numBytes)
{
    void* ptr;

    if (mode == Msgcom_MemAllocMode_CACHE_COHERENT)
        ptr = (void *)l2_global_address ((uint32_t)Memory_alloc (NULL, numBytes, 128, NULL));
    else
        ptr = (void *)((uint32_t)Memory_alloc((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, numBytes, 128, NULL));

    return ptr;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MEMLOG module to free memory which had been
 *     previously allocated
 *
 *  @param[in]  mode
 *      Memory mode
 *  @param[in]  ptr
 *      Pointer to the memory to be freed
 *  @param[in]  numBytes
 *      Number of bytes to be freed
 *
 *  @retval
 *      Not applicable
 */
void Memlog_osalFree(Msgcom_MemAllocMode mode, void* ptr, uint32_t numBytes)
{
    if (mode == Msgcom_MemAllocMode_CACHE_COHERENT)
    {
        /* The address passed here is a global address and this needs to be converted
         * back to a local address before it can get placed back into the heap. */
        Memory_free (NULL, (void *)global_l2_address((uint32_t)ptr), numBytes);
    }
    else
    {
        /* Cleanup the memory as is. */
        Memory_free ((xdc_runtime_IHeap_Handle)ddr3PrivateHeap, ptr, numBytes);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MEMLOG module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
void* Memlog_osalEnterSingleCoreCS(void)
{
    return (void *)Hwi_disable();
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MEMLOG module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @param[in]  csHandle
 *      Opaque handle to the criticial section object
 *
 *  @retval
 *      Not applicable
 */
void Memlog_osalExitSingleCoreCS(void* csHandle)
{
    UInt  key = (UInt)csHandle;
    Hwi_restore(key);
}
