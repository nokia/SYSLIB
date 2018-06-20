/**
 *   @file  msgcom_osal.h
 *
 *   @brief   
 *      This is the sample OS Adaptation layer which is used by the message 
 *      communicator module. The OSAL layer can be ported in either of the following 
 *      manners to a native OS:
 *
 *      <b> Approach 1: </b>
 *      @n  Use Prebuilt Libraries
 *           - Ensure that the provide an implementation of all 
 *             Osal_XXX API for their native OS.
 *           - Link the prebuilt libraries with their application.
 *           - Refer to the "example" directory for an example of this
 *       @n <b> Pros: </b>
 *           - Customers can reuse prebuilt TI provided libraries
 *       @n <b> Cons: </b>
 *           - Level of indirection in the API to get to the actual OS call
 *              
 *      <b> Approach 2: </b>
 *      @n  Rebuilt Library 
 *           - Create a copy of this file and modify it to directly 
 *             inline the native OS calls
 *           - Rebuild the Message communicator; ensure that the Include 
 *             path points to the directory where the copy of this file 
 *             has been provided.
 *           - Please refer to the "test" directory for an example of this 
 *       @n <b> Pros: </b>
 *           - Optimizations can be done to remove the level of indirection
 *       @n <b> Cons: </b>
 *           - Message communicator Libraries need to be rebuilt by the customer.
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
 *
 */
#ifndef __MSGCOM_OSAL_H__
#define __MSGCOM_OSAL_H__

/* MsgCom Core Library include */
#include <ti/runtime/msgcom/msgcom.h>

/** @addtogroup MSGCOM_OSAL_API
 @{ */

extern void  Osal_msgComBeginMemAccess(void* ptr, uint32_t size);
extern void  Osal_msgComEndMemAccess(void* ptr, uint32_t size);
extern void* Osal_msgComEnterSingleCoreCS(void);
extern void  Osal_msgComExitSingleCoreCS(void* csHandle);
extern void* Osal_msgComEnterMultipleCoreCS(void);
extern void  Osal_msgComExitMultipleCoreCS(void* csHandle);
extern void* Osal_msgComCreateSem(void);
extern void  Osal_msgComPendSem(void* semHandle);
extern void  Osal_msgComPostSem(void* semHandle);
extern void  Osal_msgComDeleteSem(void* semHandle);
extern void* Osal_msgComMallocGlobal(uint32_t numBytes);
extern void  Osal_msgComFreeGlobal(void* ptr, uint32_t numBytes);
extern int32_t Osal_msgComRegisterIsr(const char*    channelName,
                                      int32_t        qMgr,
                                      int32_t        qNum,
                                      void (*isr)(MsgCom_ChHandle msgChHandle),
                                      MsgCom_ChHandle arg,
                                      int32_t*        cpIntcId,
                                      int32_t*        sysIntr);
extern void Osal_msgComDisableSysInt(int32_t cpIntcId, int32_t sysIntr);
extern void Osal_msgComEnableSysInt(int32_t cpIntcId, int32_t sysIntr);

/**
 * @brief   The macro is used by the Message communicator to indicate that 
 * a shared resource (across single core) access is starting. This is used 
 * on the READER API to protect 'internal' shared resource access between the 
 * 'receive handler' and the 'get' API. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void*  Osal_msgComEnterSingleCoreCS(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not Applicable.
 *
 *  <b> Return Value </b>
 *  @n  Handle to a single-core critical section object.
 */
#define Msgcom_osalEnterSingleCoreCS      Osal_msgComEnterSingleCoreCS

/**
 * @brief   The macro is used by the Message communicator to 
 * indicate that a shared resource (across single core) access 
 * is ending. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComExitSingleCoreCS(void* csHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not Applicable.
 *
 *  <b> Return Value </b>
 *  @n  Handle to a single-core critical section object.
 */
#define Msgcom_osalExitSingleCoreCS     Osal_msgComExitSingleCoreCS

/**
 * @brief   The macro is used by the Message communicator to 
 * indicate that a shared resource (across multiple cores) access is 
 * starting
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void*  Osal_msgComEnterMultipleCoreCS(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not Applicable.
 *
 *  <b> Return Value </b>
 *  @n  Handle to a multi-core critical section object.
 */
#define Msgcom_osalEnterMultiCoreCS      Osal_msgComEnterMultipleCoreCS

/**
 * @brief   The macro is used by the Message communicator to 
 * indicate that a shared resource (across multiple cores) access 
 * is ending.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComExitMultipleCoreCS(void* csHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Handle to a multi-core critical section object.
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalExitMultiCoreCS      Osal_msgComExitMultipleCoreCS

/**
 * @brief   The macro is used by the Message communicator to indicate 
 * that the shared memory is about to be accessed. If the memory block 
 * is cached then the implementation should invalidate the cache contents.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComBeginMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory
 *  @n  size - Size of the memory being accessed.
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Msgcom_osalBeginMemAccess      Osal_msgComBeginMemAccess

/**
 * @brief   The macro is used by the Message communicator to indicate 
 * that the shared memory access is complete. If the memory block is 
 * cached then the implementation should writeback the cache contents
 * to ensure that the cache and the memory are in sync with each other.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComEndMemAccess(void* ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  ptr  - Pointer to the memory 
 *  @n  size - Size of the memory
 *
 *  <b> Return Value </b>
 *  @n  None
 */
#define Msgcom_osalEndMemAccess       Osal_msgComEndMemAccess

/**
 * @brief   The macro is used by the Message communicator to create 
 * a semaphore for each BLOCKING channel.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void*  Osal_msgComCreateSem(void)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Not Applicable.
 *
 *  <b> Return Value </b>
 *  @n  Handle to the semaphore.
 */
#define Msgcom_osalCreateSem            Osal_msgComCreateSem

/**
 * @brief   The macro is used by the Message communicator to pend
 * on the previously created semaphore if there was no message available
 * on a BLOCKING channel.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComPendSem(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalPendSem            Osal_msgComPendSem

/**
 * @brief   The macro is used by the Message communicator to post 
 * a semaphore. This is called internally by the module when a message
 * is received on a BLOCKING channel.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComPostSem(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalPostSem            Osal_msgComPostSem

/**
 * @brief   The macro is used by the Message communicator to delete 
 * a semaphore. This is called internally by the module when a BLOCKING
 * channel is being deleted.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void  Osal_msgComDeleteSem(void* semHandle)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Semaphore Handle
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalDeleteSem            Osal_msgComDeleteSem

/**
 * @brief   The macro is used by the Message communicator to allocate
 * memory. Depending on the mode parameter, memory is allocated for MsgCom internal runtime data structures 
 * or 
 * for the accumulator list configuration. Accumulator list configuration address must always 
 * return a global address, aligned on the 16 byte boundary and coming from LL2 
 * and NOT from shared memory (MSMC/DDR3)
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void* Osal_msgComMalloc(Msgcom_MemAllocMode mode, uint32_t numBytes)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Mode specfying the type of memory to allocate
 *  @n GLOBAL - Memory for accumulator list configuration. Must return global address
 *  @n INTERNAL - MsgCom internal runtime memory
 *  @n  Number of bytes to allocate
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalMalloc               Osal_msgComMalloc

/**
 * @brief   The macro is used by the Message communicator to free
 * memory which was allocated. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_msgComFree(Msgcom_MemAllocMode mode, void* ptr, uint32_t numBytes)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Mode specfying the type of memory to free
 *  @n  Memory address to be freed
 *  @n  Number of bytes which were allocated
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable.
 */
#define Msgcom_osalFree                 Osal_msgComFree


/**
 * @brief   The macro is used by the Message communicator to register
 * the specified ISR for the queue. 
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       int32_t  Osal_msgComRegisterIsr(const int8_t*  channelName,
                                       int32_t        queueMgr,
                                       int32_t        queueNum,
                                       void (*isr)(MsgCom_ChHandle msgChHandle),
                                       MsgCom_ChHandle arg,
                                       int32_t*        cpIntcId,
                                       int32_t*        sysIntr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  channelName - Name of the channel for which the ISR is being registered
 *  @n  queueMgr    - Queue Manager to which the queue belongs
 *  @n  queueNum    - Queue Number 
 *  @n  isr         - Function pointer to the exported API which should be registered.
 *  @n  arg         - Argument to be passed to the ISR.
 *  @n  cpIntcId    - CPINTC Identifier on which the system interrupt is mapped.
 *  @n  sysIntr     - System Interrupt populated by the API. Applicable only for 
 *                    interrupts hooked via the CPINTC module.
 *
 *  <b> Return Value </b>
 *  @n  Success - ISR was mapped successfully
 *  @n  Error   - ISR could not be mapped.
 */
#define Msgcom_osalRegisterIsr            Osal_msgComRegisterIsr

/**
 * @brief   The macro is used by the Message communicator to disable
 * system interrupts. System Interrupts are hooked via the CPINTC module and need
 * to be handled differently.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_msgComDisableSysInt(int32_t cpIntcId, int32_t sysIntr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  cpIntcId    - CPINTC Module identifier to which the System Interrupt is mapped to. 
 *  @n  sysIntr     - System Interrupt to be disabled. 
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Msgcom_osalDisableSysInt            Osal_msgComDisableSysInt

/**
 * @brief   The macro is used by the Message communicator to enable
 * system interrupts. System Interrupts are hooked via the CPINTC module and need
 * to be handled differently.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_msgComEnableSysInt(int32_t cpIntcId, int32_t sysIntr)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  cpIntcId    - CPINTC Module identifier to which the System Interrupt is mapped to. 
 *  @n  sysIntr     - System Interrupt to be enabled. 
 *
 *  <b> Return Value </b>
 *  @n  Not Applicable
 */
#define Msgcom_osalEnableSysInt            Osal_msgComEnableSysInt


/**
@}
*/

#endif /* __MSGCOM_OSAL_H__ */

