/**
 *   @file  core0_osal.c
 *
 *   @brief   
 *      Core0 OSAL Implementation
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2013-2014, Texas Instruments, Inc.
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
#include <ti/csl/csl_device_interrupt.h>
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
 ************************* Extern Definitions *************************
 **********************************************************************/

extern Pktlib_HeapHandle    netfpDataRxHeap;
extern Event_Handle         netfpEventHnd;

/* System Configuration Handle. */
extern Resmgr_SysCfgHandle  handleSysCfg;

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
    CACHE_invL2  (ptr, size, CACHE_FENCE_WAIT);

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
    CACHE_wbL2 (ptr, size, CACHE_FENCE_WAIT);

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
 *      The function is used by the Name module to allocate a block of memory
 *      for its internal use.
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
 *      The function is used by the Name module to free a block of memory
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
void Osal_rmBeginMemAccess(void* ptr, uint32_t size)
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


