/**
 *   @file  osal.c
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
#include <ti/runtime/hplib/hplib.h>
#include <ti/drv/sa/sa_osal.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/josh/josh.h>
#include <ti/apps/name_proxy/include/name_proxy.h>

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
 *      The function is registered with the PKTLIB instance to invalidate
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
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: MSMC and LOCAL memory regions are NOT supported on the ARM clients\n");
        return NULL;
    }

    /* Allocate memory for the memory region. */
    ptr = hplib_vmMemAlloc (numBytes, 128, 0);
    if (ptr == NULL)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Unable to allocate memory for the memory region: %s\n", name);
        return NULL;
    }

    /* Debug Message: */
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Memory Region [%s] Physical %p Virtual %p\n", name, hplib_mVMVirtToPhy(ptr), ptr);
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
    NameProxy_log(NameProxy_LogLevel_DEBUG, "Debug: Cleaning Memory Region [%s] Physical %p Virtual %p\n", name, hplib_mVMVirtToPhy(ptr), ptr);

    /* Cleanup the memory region: */
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
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Out of memory error while allocating the RESMGR mutex\n");
        return NULL;
    }

    /* Initialize the mutex */
    errCode = pthread_mutex_init (ptrMutexThread, NULL);
    if (errCode != 0)
    {
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: RESMGR Mutex initialization failed [Error code %d]\n", errCode);
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
        NameProxy_log(NameProxy_LogLevel_ERROR, "Error: Destroying the RESMGR mutex failed [Error code %d]\n", errCode);

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
void* Name_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return malloc(numBytes);
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
    free (ptr);
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
    /* No need for critical sections; since the name proxy executes in a single thread */
    return NULL;
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
    /* No need for critical sections; since the name proxy executes in a single thread */
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
    /* No need for cache operations on ARM */
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
    /* No need for cache operations on ARM */
}
