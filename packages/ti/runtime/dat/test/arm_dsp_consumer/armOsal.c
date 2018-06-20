/**
 *   @file  armOsal.c
 *
 *   @brief
 *      OSAL Implementation
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include <malloc.h>
#include <semaphore.h>

/* MCSDK Include files. */
#include <ti/drv/sa/sa_osal.h>
#include <ti/runtime/hplib/hplib.h>

/* SYSLIB Include files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/memlog/memlog.h>

/**********************************************************************
 ************************** Extern Declarations ***********************
 **********************************************************************/

/* Extern Declarations: */
extern UintcHandle             uintcHandle;

/**********************************************************************
 *************************** OSAL Functions ***************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to protect the QMSS data structures from
 *      concurrent access from multiple processes. This function is called
 *      from the QMSS LLD when the critical section is entered
 *
 *  @retval
 *      Always returns NULL
 */
void* Osal_qmssCsEnter(void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the QMSS data structures from
 *      concurrent access from multiple processes. This function is called
 *      from the QMSS LLD when the critical section is exited
 *
 *  @retval
 *      Always returns NULL
 */
void Osal_qmssCsExit (void* key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the QMSS data structures from
 *      concurrent access from multiple threads. This function is called
 *      from the QMSS LLD when the critical section is entered
 *
 *  @retval
 *      Always returns NULL
 */
void* Osal_qmssMtCsEnter()
{
    /* QMSS resources are accessed from the context of a single thread so this is
     * a dummy function. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the QMSS data structures from
 *      concurrent access from multiple threads. This function is called
 *      from the QMSS LLD when the critical section is exited
 *
 *  @param[in]  key
 *      Opaque critical section object
 *
 *  @retval
 *      Not applicable
 */
void Osal_qmssMtCsExit(void *key)
{
    /* QMSS resources are accessed from the context of a single thread so this is
     * a dummy function. */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to convert the virtual address to a physical
 *      address
 *
 *  @param[in]  ptr
 *      Virtual address
 *
 *  @retval
 *      Physical address
 */
void* Osal_qmssVirtToPhy (void *ptr)
{
    return hplib_mVMVirtToPhy(ptr);
}

/**
 *  @b Description
 *  @n
 *      The function is used to convert the physical address to a virtual
 *      address
 *
 *  @param[in]  ptr
 *      Physical address
 *
 *  @retval
 *      Virtual address
 */
void* Osal_qmssPhyToVirt (void *ptr)
{
    return hplib_mVMPhyToVirt(ptr);
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
 *      Not applicable
 */
void* Osal_qmssConvertDescVirtToPhy(uint32_t QID, void *descAddr)
{
    return hplib_mVMConvertDescVirtToPhy(descAddr);
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
 *      Not applicable
 */
void* Osal_qmssConvertDescPhyToVirt(uint32_t QID, void *descAddr)
{
    return hplib_mVMConvertDescPhyToVirt(descAddr);
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
 *      The function is the cache invalidation function which is used to
 *      indicate that the QMSS memory is about to be accessed.
 *
 *  @param[in]  blockPtr
 *      Memory address to be accessed
 *  @param[in]  size
 *      Size of memory to be accessed
 *
 *  @retval
 *      Not applicable
 */
void Osal_qmssBeginMemAccess (void *blockPtr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the cache writeback function which is used to
 *      indicate that the QMSS memory access is complete
 *
 *  @param[in]  blockPtr
 *      Memory address accessed
 *  @param[in]  size
 *      Size of memory accessed
 *
 *  @retval
 *      Not applicable
 */
void  Osal_qmssEndMemAccess (void *blockPtr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the CPPI data structures from
 *      concurrent access. This is called on critical section entry
 *
 *  @param[out]  key
 *      Opaque critical section handle.
 *
 *  @retval
 *      Not applicable
 */
void Osal_cppiCsEnter (uint32_t* key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to protect the CPPI data structures from
 *      concurrent access. This is called on critical section exit
 *
 *  @param[in]  key
 *      Opaque critical section handle.
 *
 *  @retval
 *      Not applicable
 */
void Osal_cppiCsExit (uint32_t key)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the cache invalidation function which is used to
 *      indicate that the CPPI memory is about to be accessed.
 *
 *  @param[in]  blockPtr
 *      Memory address to be accessed
 *  @param[in]  size
 *      Size of memory to be accessed
 *
 *  @retval
 *      Not applicable
 */
void Osal_cppiBeginMemAccess (void *blockPtr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the cache writeback function which is used to
 *      indicate that the CPPI memory access is complete
 *
 *  @param[in]  blockPtr
 *      Memory address accessed
 *  @param[in]  size
 *      Size of memory accessed
 *
 *  @retval
 *      Not applicable
 */
void Osal_cppiEndMemAccess (void* blockPtr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a memory block of the specified size.
 *
 *  @param[in]  numBytes
 *      Number of bytes of memory to be allocated.
 *
 *  @retval
 *      Success 	- Pointer to the allocated memory block.
 *  @retval
 *      Error		- NULL
 */
Ptr Osal_cppiMalloc (uint32_t num_bytes)
{
    Ptr ret;

    num_bytes += (127);
    ret = malloc (num_bytes);
    return ret;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free a memory block of the specified size.
 *
 *  @param[in]  ptr
 *      Pointer to the memory block to be cleaned up
 *  @param[in]  numBytes
 *      Number of bytes of memory to be cleaned up.
 *
 *  @retval
 *      Success 	- Pointer to the allocated memory block.
 *  @retval
 *      Error		- NULL
 */
void Osal_cppiFree (Ptr ptr, uint32_t size)
{
    free(ptr);
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a memory block of the specified size.
 *
 *  @param[in]  numBytes
 *      Number of bytes of memory to be allocated.
 *
 *  @retval
 *      Success 	- Pointer to the allocated memory block.
 *  @retval
 *      Error		- NULL
 */
void *Osal_rmMalloc (uint32_t numBytes)
{
	return malloc(numBytes);
}



/**
 *  @b Description
 *  @n
 *      The function is used to free a memory block of the specified size.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up.
 *  @param[in]  numBytes
 *      Number of bytes of memory to be cleaned up.
 *
 *  @retval
 *      Not applicable.
 */
void Osal_rmFree (void* ptr, uint32_t numBytes)
{
	free(ptr);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enter the critical section
 *
 *  @retval
 *      Opaque critical section handle.
 */
void *Osal_rmCsEnter(void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to exit the critical section
 *
 *  @param[in]  csHandle
 *      Critical section handle.
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmCsExit(void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory is
 *      about to be accessed. On Linux there is no need to implement
 *      the cache operations.
 *
 *  @param[in]  ptr
 *      Pointer to the memory about to be accessed.
 *  @param[in]  size
 *      Size of the memory
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmBeginMemAccess(void *ptr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to indicate that a block of memory has
 *      been accessed. On Linux there is no need to implement
 *      the cache operations.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to which access is complete.
 *  @param[in]  size
 *      Size of the memory
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmEndMemAccess(void *ptr, uint32_t size)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a task blocking object capable of
 *      blocking the task a RM instance is running within
 *
 *  @retval
 *      Opaque critical section handle.
 */
void* Osal_rmTaskBlockCreate(void)
{
    return(NULL);
}

/**
 *  @b Description
 *  @n
 *      The function is used to block a task whose context a RM instance
 *      is running within.
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmTaskBlock(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to unblock a task whose context a RM instance is
 *      running within.
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmTaskUnblock(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a task blocking object
 *      provided to a RM instance
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmTaskBlockDelete(void *handle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to print a string to the console
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmLog (char *fmt, ... )
{
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
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
    return malloc (numBytes);
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
    free(ptr);
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
    /* Cache operations are not required on ARM */
    return;
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
    /* Cache operations are not required on ARM */
    return;
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
    /* Cache operations are not required on ARM */
    return;
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
    /* Cache operations are not required on ARM */
    return;
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
 *      The function is used by the packet library to convert the physical address
 *      to a virtual address
 *
 *  @param[in]  ptrPhysicalAddress
 *      Physical address
 *
 *  @retval
 *      Virtual address
 */
void* Pktlib_osalPhyToVirt(void* ptrPhysicalAddress)
{
    return(hplib_mVMPhyToVirt(ptrPhysicalAddress));
}

/**
 *  @b Description
 *  @n
 *      The function is called by hplib_utilSetupThread. This is a DUMMY implementation
 *
 *  @param[in]  core_id
 *      Core identifier
 *
 *  @retval
 *      Not Applicable
 */
void Osal_nwalSetProcId (uint16_t core_id)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      HPLIB Spinlock interface type
 *
 *  @param[in]  if_type
 *      Spinlock type
 *
 *  @retval
 *      Not applicable
 */
void Osal_setHplibSpinLockIfType(hplib_spinLock_Type if_type)
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
    /* Ignore the mode parameter: On ARM Global allocations are not required. */
    return malloc(size);
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
    free (ptr);
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
    void* ptr;

    /* Check if we are allocating for MSMC or LOCAL? */
    if ((memRegType == Resmgr_MemRegionType_MSMC) || (memRegType == Resmgr_MemRegionType_LOCAL))
    {
        printf ("Error: MSMC and LOCAL memory regions are NOT supported on the ARM clients\n");
        return NULL;
    }

    /* Allocate memory for the memory region. */
    ptr = hplib_vmMemAlloc (numBytes, 128, 0);
    if (ptr == NULL)
    {
        printf ("Error: Unable to allocate memory for the memory region: %s\n", name);
        return NULL;
    }

    /* Debug Message: */
    printf ("Debug: Memory Region [%s] Physical %p Virtual %p\n", name, hplib_mVMVirtToPhy(ptr), ptr);
    return ptr;
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
    /* Debug Message: */
    printf ("Debug: Cleaning Memory Region [%s] Physical %p Virtual %p Size:%d\n", name, hplib_mVMVirtToPhy(ptr), ptr, numBytes);
    hplib_vmMemFree (ptr, numBytes, 0);

    return;
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
    pthread_mutex_t*    ptrMutexThread;
    int32_t             errCode;

    /* Allocate memory for the mutex */
    ptrMutexThread = (pthread_mutex_t*)malloc (sizeof(pthread_mutex_t));
    if (ptrMutexThread == NULL)
    {
        printf ("Error: Out of memory error while allocating the pthread mutex\n");
        return NULL;
    }

    /* Initialize the mutex */
    errCode = pthread_mutex_init (ptrMutexThread, NULL);
    if (errCode != 0)
    {
        printf ("Error: Mutex initialization failed [Error code %d]\n", errCode);
        return NULL;
    }
    return (void *)ptrMutexThread;
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
    pthread_mutex_lock((pthread_mutex_t *)semHandle);
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
    pthread_mutex_unlock((pthread_mutex_t *)semHandle);
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
    pthread_mutex_t*    ptrMutexThread;
    int32_t             errCode;

    /* Get the mutex handle */
    ptrMutexThread = (pthread_mutex_t*)semHandle;
    errCode = pthread_mutex_destroy (ptrMutexThread);
    if (errCode != 0)
        printf ("Error: Destroying the mutex failed [Error code %d]\n", errCode);

    /* Cleanup the memory */
    free (ptrMutexThread);
    return;
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
    /* Cache operations: This is not needed on ARM */
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
    /* Cache operations: This is not needed on ARM */
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
	if (mode == Msgcom_MemAllocMode_CACHE_COHERENT)
    {
        printf ("Error: Cache coherent memory allocation is NOT currently supported\n");
        return NULL;
    }
    /* Return memory from the default heap. */
    return malloc (numBytes);
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
        printf ("Error: Cache coherent memory free is NOT supported\n");
        return;
    }
    free (ptr);
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
    int32_t errCode;

    /* In the ARM realm system interrupt is never used. */
    if (ptrInterruptInfo->sysInterrupt != -1)
    {
        printf ("Error: Channel Name '%s' invalid configuration [System Interrupt %d].\n", channelName, ptrInterruptInfo->sysInterrupt);
        return -1;
    }

    /* Debug Message: */
    printf ("Debug: Enabling GIC interrupt %d for channel '%s'\n", ptrInterruptInfo->hostInterrupt, channelName);

    /* Use the UINTC module to register the ISR */
    if (Uintc_registerIsr(uintcHandle, ptrInterruptInfo->hostInterrupt, (UintcIsrHandler)isr, chHandle, &errCode) < 0)
    {
        printf ("Error: UINTC Interrupt registeration for %d failed [Error code %d]\n", ptrInterruptInfo->hostInterrupt, errCode);
        return -1;
    }
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
    int32_t errCode;

    /* In the ARM realm system interrupt is never used. */
    if (ptrInterruptInfo->sysInterrupt != -1)
    {
        printf ("Error: Channel Name '%s' invalid configuration [System Interrupt %d].\n", channelName, ptrInterruptInfo->sysInterrupt);
        return -1;
    }

    /* Debug Message: */
    printf ("Debug: Disabling GIC interrupt %d for channel '%s'\n", ptrInterruptInfo->hostInterrupt, channelName);

    /* Use the UINTC module to deregister the ISR */
    if (Uintc_deregisterIsr(uintcHandle, ptrInterruptInfo->hostInterrupt, &errCode) < 0)
    {
        printf ("Error: UINTC Interrupt deregisteration for %d failed [Error code %d]\n", ptrInterruptInfo->hostInterrupt, errCode);
        return -1;
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
    int32_t errCode;

    /* Use the UINTC module to disable the event */
    if (Uintc_disableEvent(uintcHandle, sysIntr, &errCode) < 0)
        printf ("Error: Disabling the UINTC event failed [Error code %d]\n", errCode);
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
    int32_t errCode;

    /* Use the UINTC module to enable the event */
    if (Uintc_enableEvent(uintcHandle, sysIntr, &errCode) < 0)
        printf ("Error: Enabling the UINTC event failed [Error code %d]\n", errCode);
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
    return NULL;
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
    return;
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
    sem_t*  ptrSemaphore;

    /* Allocate memory for the semaphore */
    ptrSemaphore = malloc (sizeof(sem_t));
    if (ptrSemaphore == NULL)
    {
        printf ("Error: Unable to allocate memory for the MSGCOM semaphore\n");
        return NULL;
    }

    /* Initialize the semaphore: This is not shared and initially the semaphore is marked
     * as NOT available. */
    if (sem_init (ptrSemaphore, 0, 0) < 0)
    {
        printf ("Error: MSGCOM semaphore initialization failed [Error %s]\n", strerror(errno));
        return NULL;
    }

    /* Return the semaphore handle. */
    return (void*)ptrSemaphore;
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
    sem_close ((sem_t*)semHandle);
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
    sem_wait ((sem_t*)semHandle);
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
    sem_post ((sem_t*)semHandle);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to allocate memory.
 *
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *  @param[in]  alignment
 *      Memory alignment requirments
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
void* Netfp_osalMalloc (uint32_t size, uint32_t alignment)
{
    return memalign (alignment, size);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the NETFP module to cleanup memory.
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  size
 *      Number of bytes which need to be allocated
 *
 *  @retval
 *      Not applicable.
 */
void Netfp_osalFree (void* ptr, uint32_t size)
{
    free(ptr);
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
    static sem_t*   ptrNetfpSem = NULL;

    /* We initialize and create the NETFP Semaphore the first time the
     * function is called. Subsequently we simply use the created semaphores. */
    if (ptrNetfpSem == NULL)
    {
        /* Allocate memory for a semaphore. */
        ptrNetfpSem = malloc (sizeof(sem_t));
        if (ptrNetfpSem == NULL)
        {
            printf ("Error: NETFP Semaphore memory allocation failed\n");
            return NULL;
        }

        /* Initialize the semaphore. */
        if (sem_init (ptrNetfpSem, 0, 1) < 0)
        {
            free (ptrNetfpSem);
            ptrNetfpSem = NULL;
            return NULL;
        }
    }

    /* Wait on the semaphore. */
    sem_wait (ptrNetfpSem);
    return (void *)ptrNetfpSem;
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
    /* Get the NETFP semaphore */
    sem_t* ptrNetfpSem = (sem_t*)csHandle;

    /* Post the semaphore to indicate that the critical section is done. */
    sem_post (ptrNetfpSem);
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
    /* There is no need to perform cache operations on ARM */
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
    /* There is no need to perform cache operations on ARM */
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
    int32_t     retVal;
    sem_t*      ptrSemaphore;

    /* Allocate memory for a semaphore. */
    ptrSemaphore = malloc (sizeof(sem_t));
    if (ptrSemaphore == NULL)
    {
        printf ("Error: Out of memory while allocating agent semaphore\n");
        return NULL;
    }

    /* Create a semaphore. */
    retVal = sem_init (ptrSemaphore, 0, 0);
    if (retVal < 0)
    {
        free (ptrSemaphore);
        return NULL;
    }
    return (void*)ptrSemaphore;
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
    sem_close ((sem_t*)semHandle);
    free(semHandle);
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
    sem_wait ((sem_t*)semHandle);
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
    sem_post ((sem_t*)semHandle);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the PA LLD to indicate that the memory
 *     access is about to start
 *
 *  @param[in]  addr
 *      Address to be accessed
 *  @param[in]  size
 *      Size of memory to be accessed
 *
 *  @retval
 *      Not applicable
 */
void Osal_paBeginMemAccess (Ptr addr, uint32_t size)
{
    /* There is no need to perform cache operations on ARM */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the PA LLD to indicate that the memory
 *     access is complete
 *
 *  @param[in]  addr
 *      Address accessed
 *  @param[in]  size
 *      Size of memory accessed
 *
 *  @retval
 *      Not applicable
 */
void Osal_paEndMemAccess (Ptr addr, uint32_t size)
{
    /* There is no need to perform cache operations on ARM */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the PA LLD to indicate that the critical
 *     section access is starting
 *
 *  @param[out]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_paMtCsEnter (uint32_t *key)
{
    /* NETFP Server: All the PA resources are accessed from a single thread */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the PA LLD to indicate that the critical
 *     section access is ending
 *
 *  @param[in]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_paMtCsExit (uint32_t key)
{
    /* NETFP Server: All the PA resources are accessed from a single thread */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to get the process identifier
 *
 *  @retval
 *      Always returns 0
 */
uint16_t Osal_saGetProcId (void)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to get the physical address.
 *
 *  @param[in]  vaddr
 *      Virtual address
 *
 *  @retval
 *      Physical address
 */
void* Osal_saGetSCPhyAddr(void* vaddr)
{
    if(vaddr == NULL)
        return NULL;
    return (void *)(memPoolAddr[0].memStartPhy + ((uint8_t*)vaddr - memPoolAddr[0].memStart));
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to invalidate the security context
 *
 *  @param[in]  addr
 *      Security context address
 *  @param[in]  size
 *      Size of the security context
 *
 *  @retval
 *      Not applicable
 */
void Osal_saBeginScAccess (void* addr, uint32_t size)
{
    hplib_cacheInv(addr,size);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to writeback the security context
 *
 *  @param[in]  addr
 *      Security context address
 *  @param[in]  size
 *      Size of the security context
 *
 *  @retval
 *      Not applicable
 */
void Osal_saEndScAccess   (void* addr, uint32_t size)
{
    hplib_cacheWb(addr,size);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to provide a critical
 *     section to protect its internal data structures.
 *
 *  @param[out]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_saCsEnter (uint32_t* key)
{
    /* The NETFP server accesses all the SA LLD resources from a single thread. Hence there
     * is no need for a critical section. */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to provide a critical
 *     section to protect its internal data structures.
 *
 *  @param[in]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_saCsExit (uint32_t key)
{
    /* The NETFP server accesses all the SA LLD resources from a single thread. Hence there
     * is no need for a critical section. */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to provide a critical
 *     section to protect its internal data structures.
 *
 *  @param[in]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_saMtCsEnter (uint32_t *key)
{
    /* The NETFP server accesses all the SA LLD resources from a single thread. Hence there
     * is no need for a critical section. */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to provide a critical
 *     section to protect its internal data structures.
 *
 *  @param[in]  key
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_saMtCsExit (uint32_t key)
{
    /* The NETFP server accesses all the SA LLD resources from a single thread. Hence there
     * is no need for a critical section. */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to invalidate the cache contents
 *
 *  @param[in]  blockPtr
 *      Address of the memory
 *  @param[in]  size
 *      Size of the memory
 *
 *  @retval
 *      Not applicable
 */
void Osal_saBeginMemAccess (void *blockPtr, uint32_t size)
{
    /* There is no need to perform cache operations on ARM */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to writeback the cache contents
 *
 *  @param[in]  blockPtr
 *      Address of the memory
 *  @param[in]  size
 *      Size of the memory
 *
 *  @retval
 *      Not applicable
 */
void Osal_saEndMemAccess (void *blockPtr, uint32_t size)
{
    /* There is no need to perform cache operations on ARM */
    return;
}

/**
 *  @b Description
 *  @n
 *     The function is used by the SA LLD to get the endianess
 *
 *  @retval
 *      Endianess
 */
int   Osal_saGetSysEndianMode(void)
{
#if defined( _BIG_ENDIAN)
    return((int)sa_SYS_ENDIAN_MODE_BIG);
#else
    return((int)sa_SYS_ENDIAN_MODE_LITTLE);
#endif
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
    return memalign (alignment, numBytes);
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
    free(ptr);
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
    static sem_t*  datSemaphore = NULL;

    /* Do we need to initialize the semaphore? */
    if (datSemaphore == NULL)
    {
        datSemaphore = malloc (sizeof(sem_t));
        if (datSemaphore == NULL)
        {
            printf ("Error: Unable to allocate memory for the MSGCOM semaphore\n");
            return NULL;
        }

        /* Initialize the semaphore: This is not shared and initially the semaphore is
         * marked as available. */
        if (sem_init (datSemaphore, 0, 1) < 0)
            return NULL;
    }

    /* Wait for the semaphore to be available */
    sem_wait (datSemaphore);

    /* Return the semaphore handle. */
    return (void*)datSemaphore;
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
    /* Release the critical section */
    sem_post ((sem_t*)csHandle);
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
    /* No need for cache operations on ARM */
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
    /* No need for cache operations on ARM */
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
    sem_t*  ptrSemaphore;

    /* Allocate memory for the semaphore */
    ptrSemaphore = malloc (sizeof(sem_t));
    if (ptrSemaphore == NULL)
    {
        printf ("Error: Unable to allocate memory for the MSGCOM semaphore\n");
        return NULL;
    }

    /* Initialize the semaphore: This is not shared and initially the semaphore is marked
     * as NOT available. */
    if (sem_init (ptrSemaphore, 0, 0) < 0)
    {
        printf ("Error: MSGCOM semaphore initialization failed [Error %s]\n", strerror(errno));
        return NULL;
    }

    /* Return the semaphore handle. */
    return (void*)ptrSemaphore;
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
    sem_close ((sem_t*)semHandle);
    free(semHandle);
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
    sem_wait ((sem_t*)semHandle);
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
    sem_post ((sem_t*)semHandle);
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
void* Name_osalMalloc(uint32_t size, uint32_t alignment)
{
    return memalign (alignment, size);
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
void Name_osalFree(void* ptr, uint32_t size)
{
    free (ptr);
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
void Name_osalBeginMemAccess(void* ptr, uint32_t size)
{
	/* There is no need to perform cache operations on ARM */
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
void Name_osalEndMemAccess(void* ptr, uint32_t size)
{
	/* There is no need to perform cache operations on ARM */
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
void* Name_osalEnterCS(void)
{
    /* No protection required as the DAT server executes in a single thread */
    return NULL;
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
void Name_osalExitCS(void* semHandle)
{
    /* No protection required as the DAT server executes in a single thread */
    return;
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
void* Name_osalCreateSem(void)
{
    int32_t     retVal;
    sem_t*      ptrSemaphore;

    /* Allocate memory for a semaphore. */
    ptrSemaphore = malloc (sizeof(sem_t));
    if (ptrSemaphore == NULL)
    {
        printf ("Error: Out of memory while allocating agent semaphore\n");
        return NULL;
    }

    /* Create a semaphore. */
    retVal = sem_init (ptrSemaphore, 0, 0);
    if (retVal < 0)
    {
        free (ptrSemaphore);
        return NULL;
    }
    return (void*)ptrSemaphore;
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
void Name_osalDeleteSem(void* semHandle)
{
    sem_close ((sem_t*)semHandle);
    free (semHandle);
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
void Name_osalPendSem (void* semHandle)
{
    /* Pend on the semaphore. */
    sem_wait (semHandle);
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
void Name_osalPostSem (void* semHandle)
{
    /* Post the semaphore. */
    sem_post (semHandle);
    return;
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
void* Memlog_osalMalloc(Memlog_MemAllocMode mode, uint32_t numBytes)
{
    if (mode == Memlog_MemAllocMode_CACHE_COHERENT)
    {
        printf ("Error: Cache coherent memory allocation is NOT currently supported\n");
        return NULL;
    }
    /* Return memory from the default heap. */
    return malloc (numBytes);
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
void Memlog_osalFree(Memlog_MemAllocMode mode, void* ptr, uint32_t numBytes)
{
    if (mode == Memlog_MemAllocMode_CACHE_COHERENT)
    {
        printf ("Error: Cache coherent memory allocation is NOT currently supported\n");
        return ;
    }

    free(ptr);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the MEMLOG module to protect its internal
 *     resources from concurrent access within a single core.K
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
void* Memlog_osalEnterCS(void)
{
    return NULL;
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
void Memlog_osalExitCS(void* csHandle)
{
    return;
}

