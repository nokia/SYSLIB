/**
 *   @file  osal.c
 *
 *   @brief
 *      This is the OS abstraction layer used by the Resource Manager in Linux.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2013, Texas Instruments, Inc.
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
 *
 *  \par
*/

/* Standard Includes */
#include <ddal/ddal_common.h>
#include <ddal/ddal_cma.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>

/* MCSDK Include files */
#include <ti/runtime/hplib/hplib.h>
#include <ti/runtime/hplib/hplib_vm.h>

/* SYSLIB include files */
#include <ti/apps/resmgr_server/include/resmgr_server.h>
#include <ti/runtime/pktlib/pktlib.h>

/**********************************************************************
 *************************** Extern Functions *************************
 **********************************************************************/

/* Logging functions: */
extern void ResmgrServer_dumpMsg(char* fmt, va_list arg);

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
    void *physical = NULL;
    ddal_cma_virt_to_phy(ptr, &physical);

    return physical;
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
    void *virt = NULL;
    ddal_cma_phy_to_virt(ptr, &virt);

    return virt;
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
    (void)QID; // suppress -Wunused-parameter GCC warning
    return Pktlib_mVMConvertDescVirtToPhy(descAddr);
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
    (void)QID; // suppress -Wunused-parameter GCC warning
    return Pktlib_mVMConvertDescPhyToVirt(descAddr);
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
    ResmgrServer_dumpMsg(fmt, ap);
    va_end(ap);
}


/**
 *  @b Description
 *  @n
 *      The function is to protect the RM resources from multiple threads.
 *
 *  @param[in]  mtSemObj
 *      Multi-threaded critical section object handle.
 *
 *  @retval
 *      Handle used to lock the multi-threaded critical section
 */
void* Osal_rmMtCsEnter (void *mtSemObj)
{
    /* In the SYSRM server; there is only 1 single thread which is operating and so there is
     * no need for any protection here. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is to protect the RM resources from multiple threads.
 *
 *  @param[in]  mtSemObj
 *      Multi-threaded critical section object handle.
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmMtCsExit (void *mtSemObj, void *CsHandle)
{
    /* In the SYSRM server; there is only 1 single thread which is operating and so there is
     * no need for any protection here. */
    return;
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
 *      HPLIB Critical section enter function
 *
 *  @retval
 *      Opaque critical section handle
 */
void* Osal_hplibCsEnter (void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      HPLIB Critical section enter function
 *
 *  @param[in]  core_id
 *      Opaque critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_hplibCsExit (void *CsHandle)
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

