/**
 *   @file  msgcom.h
 *
 *   @brief
 *      Header file for the Message Communicator. The file exposes the
 *      data structures and exported API which are available for use
 *      by applications.
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

/** @defgroup MSG_COM_API   Message Communicator
 */
#ifndef __MSG_COM_H__
#define __MSG_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**
@defgroup MSG_COM_SYMBOL                   MSGCOM Defined Symbols
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_FUNCTION                 MSGCOM Exported Functions
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_ENUM                     MSGCOM Exported Enumerations
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_DATA_STRUCTURE           MSGCOM Exported Data Structures
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_INTERNAL_SYMBOL          MSGCOM Internal Symbols
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_INTERNAL_ENUM            MSGCOM Internal Enumerations
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_INTERNAL_FUNCTION        MSGCOM Internal Functions
@ingroup MSG_COM_API
*/
/**
@defgroup MSG_COM_INTERNAL_DATA_STRUCTURE  MSGCOM Internal Data Structures
@ingroup MSG_COM_API
*/
/**
@defgroup MSGCOM_OSAL_API                  MSGCOM OSAL API
@ingroup MSG_COM_API
*/
/**
@defgroup MSGCOM_DEV_API                   MSGCOM Device API
@ingroup MSG_COM_API
*/
/**
@defgroup MSGCOM_ERROR_CODE                MSGCOM Error Codes
@ingroup MSG_COM_API
*/

/** @addtogroup MSG_COM_SYMBOL
 @{ */

/**
 * @brief   Message Communicator Version. Versions numbers are encoded in the following
 * format:
 *  0xAABBCCDD -> Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 */
#define MSG_COM_VERSION_ID                         (0x03000000)

/**
 * @brief   Maximum length of the channel name
 */
#define MSG_COM_MAX_CHANNEL_LEN                     32

/**
 * @brief   Maximum number of virtual channels which can be supported
 */
#define MSG_COM_MAX_VIRTUAL_CHANNEL                 32

/**
@}
*/

/** @addtogroup MSGCOM_ERROR_CODE
 *
 * @brief
 *  Base error code for the MSGCOM module is defined in the
 *  \include ti/runtime/common/syslib.h
 *
 @{ */

/** @brief MsgCom invalid parameter */
#define MSGCOM_INVALID_PARAM                        SYSLIB_ERRNO_MSGCOM_BASE-1
/** @brief MsgCom Max supported channels configured */
#define MSGCOM_MAX_CHANNELS_CONFIGURED              SYSLIB_ERRNO_MSGCOM_BASE-2
/** @brief MsgCom virtual channel not supported */
#define MSGCOM_VIRTUAL_CHANNEL_NOT_SUPPORTED        SYSLIB_ERRNO_MSGCOM_BASE-3
/** @brief MsgCom Low priority interrupt not supported */
#define MSGCOM_LOW_PRIO_INTERRUPT_NOT_SUPPORTED     SYSLIB_ERRNO_MSGCOM_BASE-4
/** @brief MsgCom failed to open queue */
#define MSGCOM_QUEUE_OPEN_FAILED                    SYSLIB_ERRNO_MSGCOM_BASE-5
/** @brief MsgCom failed to open channel */
#define MSGCOM_CHANNEL_CONFIG_FAILED                SYSLIB_ERRNO_MSGCOM_BASE-6
/** @brief MsgCom failed to configure flow */
#define MSGCOM_FLOW_CONFIG_FAILED                   SYSLIB_ERRNO_MSGCOM_BASE-7
/** @brief MsgCom failed to register ISR */
#define MSGCOM_REGISTER_ISR_FAILED                  SYSLIB_ERRNO_MSGCOM_BASE-8
/** @brief MsgCom failed to configure accumulator */
#define MSGCOM_ACCUMULATOR_CONFIG_FAILED            SYSLIB_ERRNO_MSGCOM_BASE-9
/** @brief MsgCom failed to configure socket */
#define MSGCOM_SOCKET_CONFIG_FAILED                 SYSLIB_ERRNO_MSGCOM_BASE-10
/** @brief MsgCom duplicate channel */
#define MSGCOM_CHANNEL_ALREADY_EXISTS               SYSLIB_ERRNO_MSGCOM_BASE-11
/** @brief MsgCom memory allocation failed */
#define MSGCOM_MALLOC_FAILED                        SYSLIB_ERRNO_MSGCOM_BASE-12
/** @brief Named resource allocation failed */
#define MSGCOM_NAMEDRES_CONFIG_FAILED               SYSLIB_ERRNO_MSGCOM_BASE-13
/** @brief MsgCom failed to deregister ISR */
#define MSGCOM_DEREGISTER_ISR_FAILED                SYSLIB_ERRNO_MSGCOM_BASE-14
/** @brief MsgCom delete failed since channel is in use */
#define MSGCOM_CHANNEL_IN_USE                       SYSLIB_ERRNO_MSGCOM_BASE-15

/**
@}
*/
//fzm
typedef enum Queue_BarrierType
{
    DDR = 0,
    MSMC = 1,
    OTHER = 2
} Queue_BarrierType;

/** @addtogroup MSG_COM_ENUM
 @{ */

/**
 * @brief
 *  Message Mode
 *
 * @details
 *  When a message is being retrieved it is possible to specify the specific
 *  message retreival mode for each GET operation.
 */
typedef enum Msgcom_MessageMode
{
    /**
     * @brief   If there is no message available the get API will block waiting
     * for the message to arrive.
     */
    Msgcom_MessageMode_BLOCKING          = 0x1,

    /**
     * @brief   If there is no message available the get API will return immediately
     * indicating there was no message.
     */
    Msgcom_MessageMode_NON_BLOCKING      = 0x2
}Msgcom_MessageMode;

/**
 * @brief
 *  Memory allocation mode
 *
 * @details
 *  The MSGCOM module allocates memory for multiple purposes. There are certain considerations
 *  which need to be accounted for while allocating memory.
 */
typedef enum Msgcom_MemAllocMode
{
    /**
     * @brief  This mode is used when the requesting memory for the accumulator list.
     * This memory needs to be cache coherent as this is allocated and used to program the
     * accumulator. The MSGCOM module does NOT perform any cache operations on this list
     * for performance.
     *
     * In the DSP realm:
     * - Allocate memory from the local L2 memory and return the global address
     * In the ARM realm:
     * - Allocate memory from the HPLIB memory pools.
     */
    Msgcom_MemAllocMode_CACHE_COHERENT  = 0x1,

    /**
     * @brief   This mode is used to request memory for internal runtime data structures.
     */
    Msgcom_MemAllocMode_INTERNAL        = 0x2
}Msgcom_MemAllocMode;

/**
 * @brief
 *  Channel Mode
 *
 * @details
 *  MSGCOM channels can be created in either of the following modes.
 */
typedef enum Msgcom_ChannelMode
{
    /**
     * @brief   Channel get is blocked if there is no message available.
     */
    Msgcom_ChannelMode_BLOCKING         = 0x1,

    /**
     * @brief   Channel get will return with 0 indicating that there is no
     * message available.
     */
    Msgcom_ChannelMode_NON_BLOCKING     = 0x2
}Msgcom_ChannelMode;

/**
 * @brief
 *  Enumeration Type which describes the different types of channels.
 *
 * @details
 *  The message communicator supports the following channels. Applications
 *  can create channels of the specific type with the associated properties.
 */
typedef enum Msgcom_ChannelType
{
    /**
     * @brief   Queue based channels are used to send and receive messages by
     * simply sharing the descriptor address betweent the writer and reader.
     * Applications need to ensure that the descriptor address is visible between
     * the writer & reader.
     */
    Msgcom_ChannelType_QUEUE        = 0x0,

    /**
     * @brief   Queue-Data channels are used to send & receive messages by using
     * DMA to transfer data from the writer to the reader.
     */
    Msgcom_ChannelType_QUEUE_DMA    = 0x1,

    /**
     * @brief   Virtual Channels are created on top of PHYSICAL channels. PHYSICAL
     * channels are limited because of hardware constraints. However multiple
     * virtual channels can be created on top of a physical channel to help address
     * this issue. Virtual channels inherit the same properties and configuration
     * as their physical channels.
     */
    Msgcom_ChannelType_VIRTUAL      = 0x2
}Msgcom_ChannelType;

/**
 * @brief
 *  Enumeration Type which describes the different types of accumulated channels.
 *
 * @details
 *  Accumulated channels can be of either type. The configuration can be used by
 *  applications to decide the type of accumulator channel it wishes to open.
 */
typedef enum Msgcom_AccumulatedChannelType
{
    /**
     * @brief   LOW accumulated channels allow multiple low priority queues to be
     * monitored by a single accumulator channel.
     */
    Msgcom_AccumulatedChannelType_LOW   =  1,

    /**
     * @brief   HIGH accumulated channels allow a single high priority queues to be
     * monitored by a single accumulator channel.
     */
    Msgcom_AccumulatedChannelType_HIGH  =  2
}Msgcom_AccumulatedChannelType;

/**
 * @brief
 *  Enumeration Type which describes the different types of channels.
 *
 * @details
 *  The message communicator supports the following channels. Applications
 *  can create channels of the specific type with the associated properties.
 */
typedef enum Msgcom_QueueInterruptMode
{
    /**
     * @brief   No Interrupt support. The channel needs to be polled to check
     * if there are messages available or not?
     */
    Msgcom_QueueInterruptMode_NO_INTERRUPT          = 0x1,

    /**
     * @brief   Interrupt support. The channel is mapped to a queue which
     * directly generates interrupts.
     */
    Msgcom_QueueInterruptMode_DIRECT_INTERRUPT      = 0x2,

    /**
     * @brief   Interrupt support through the accumulator.
     */
    Msgcom_QueueInterruptMode_ACCUMULATED_INTERRUPT = 0x3
}Msgcom_QueueInterruptMode;

/**
@}
*/

/** @addtogroup MSG_COM_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  Opaque MSGCOM Channel Handle.
 */
typedef void*   MsgCom_ChHandle;

/**
 * @brief
 *  Opaque MSGCOM Data Buffer.
 */
typedef void*   MsgCom_Buffer;

/**
 * @brief
 *  Opaque MSGCOM Instance handle
 */
typedef void*   Msgcom_InstHandle;

/**
 * @brief
 *  Application registered call back function which is invoked by the MSGCOM module
 *  to clean up any pending packets in the MSGCOM channel when the channel is being
 *  deleted.
 *
 *  @param[in]  pktlibInstHandle
 *      PKTLIB Instance handle registered with the channel
 *  @param[in]  chHandle
 *      Channel handle being deleted
 *  @param[in]  msgBuffer
 *      Pointer to the MSGCOM buffer
 *
 *  @retval
 *      Not applicable
 */
typedef void (*Msgcom_freePkt)(Pktlib_InstHandle pktlibInstHandle, MsgCom_ChHandle chHandle, MsgCom_Buffer* msgBuffer);

/**
 * @brief
 *  ISR function which is registered with the interrupt services to be invoked when
 *  the channel receives a packet.
 *
 *  @param[in]  chHandle
 *      Channel handle
 *
 *  @retval
 *      Not applicable
 */
typedef void (*MsgCom_Isr)(MsgCom_ChHandle chHandle);

/**
 * @brief
 *  Application registered call back function which is invoked by the MSGCOM module
 *  to indicate that data has been received on the channel.
 *
 *  @param[in]  chHandle
 *      Channel handle on which data is received
 *  @param[in]  arg
 *      Optional application defined argument
 *
 *  @retval
 *      Not applicable
 */
typedef void (*MsgCom_AppCallBack)(MsgCom_ChHandle chHandle, uint32_t arg);

/**
 * @brief
 *  MSGCOM Interrupt
 *
 * @details
 *  The structure is used to specify the interrupt characteristics and is passed to
 *  the application to allow the configuration of interrupts.
 */
typedef struct MsgCom_Interrupt
{
    /**
     * @brief   System Interrupt
     *
     * This field is not used in the ARM realm since the ARM interrupts are routed
     * via the GIC.
     * This field is only applicable in the DSP realm for Direct Interrupts only
     * since these interrupt lines are routed to the DSP INTC via the CPINTC module
     * Set to -1 indicates that the system interrupt field should be ignored
     */
    int32_t         sysInterrupt;

    /**
     * @brief   CPINTC identifier
     *
     * The CPINTC identifier is associated with the system interrupt field above
     * and is valid only if system interrupt is a valid value. There are multiple
     * CPINTC blocks in the system and this indicates the CPINTC identifier through
     * which the direct interrupt system interrupt is being routed. Valid only in
     * the DSP realm.
     */
    int32_t         cpIntcId;

    /**
     * @brief   Host Interrupt
     *
     * In the DSP realm
     *  - If the system interrupt is valid [Direct interrupts] this is the host interrupt
     *    to which the system interrupt has been mapped to
     *  - If the system interrupt is invalid [Accumulated interrupts] this is the DSP INTC
     *    event identifier
     *
     * In the ARM realm
     *  - This is the GIC interrupt identifier.
     */
    int32_t         hostInterrupt;
}MsgCom_Interrupt;

/**
@}
*/

/** @addtogroup MSGCOM_OSAL_API
 @{ */

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
typedef void* (*Osal_MsgcomMalloc)(Msgcom_MemAllocMode mode, uint32_t numBytes);

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
typedef void (*Osal_MsgcomFree)(Msgcom_MemAllocMode mode, void* ptr, uint32_t numBytes);

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
 *      Pointer to the MSGCOM interrupt information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
typedef int32_t (*Osal_MsgcomRegisterIsr)
(
    const char*         channelName,
    Qmss_Queue          queueInfo,
    MsgCom_Isr          isr,
    MsgCom_ChHandle     chHandle,
    MsgCom_Interrupt*   ptrInterruptInfo
);

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
 *      Pointer to the MSGCOM interrupt information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
typedef int32_t (*Osal_MsgcomDeregisterIsr)
(
    const char*         channelName,
    Qmss_Queue          queueInfo,
    MsgCom_Interrupt*   ptrInterruptInfo
);

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
typedef void (*Osal_MsgcomDisableSysInt)(int32_t cpIntcId, int32_t sysIntr);

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
typedef void (*Osal_MsgcomEnableSysInt)(int32_t cpIntcId, int32_t sysIntr);

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to protect its internal
 *     resources from concurrent access within a single core.
 *
 *  @retval
 *      Opaque handle to the criticial section object
 */
typedef void* (*Osal_MsgcomEnterSingleCoreCS)(void);

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
typedef void (*Osal_MsgcomExitSingleCoreCS)(void* csHandle);

/**
 *  @b Description
 *  @n
 *     The function is used by the MSGCOM module to create a semaphore
 *     for the MSGCOM blocking channel.
 *
 *  @retval
 *      Handle to the semaphore object.
 */
typedef void* (*Osal_MsgcomCreateSem)(void);

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
typedef void (*Osal_MsgcomPendSem)(void* semHandle);

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
typedef void (*Osal_MsgcomPostSem)(void* semHandle);

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
typedef void (*Osal_MsgcomDeleteSem)(void* semHandle);

/**
@}
*/

/** @addtogroup MSG_COM_DATA_STRUCTURE
 @{ */

/**
 * @brief
 *  MSGCOM Accumulator Interrupt configuration
 *
 * @details
 *  The structure is used to specify accumulator interrupt specific
 *  configuration.
 */
typedef struct Msgcom_AccumulatorInterruptCfg
{
    /**
     * @brief   Accumulator channel type
     */
    Msgcom_AccumulatedChannelType       type;

    /**
     * @brief   Accumulator channel to be used
     */
    uint8_t                             accChannel;

    /**
     * @brief   Accumulator queue associated with the accumulator channel.
     */
    Qmss_Queue                          accQueue;

    /**
     * @brief   QMSS PDSP identifier associated with the accumulator channel.
     */
    Qmss_PdspId                         pdspId;

    /**
     * @brief   Interrupt identifier to which the accumulator channel is mapped to.
     */
    int32_t                             interruptId;

    /**
     * @brief   Maximum number of paging entries which can be used to store the received messages.
     */
    uint32_t                            maxPageEntries;

    /**
     * @brief   Timer pacing after which an interrupt is generated if there is a message
     * available on the channel.
     */
    uint32_t                            pacingTimerCount;
}Msgcom_AccumulatorInterruptCfg;

/**
 * @brief
 *  MSGCOM Direct Interrupt configuration
 *
 * @details
 *  The structure is used to specify direct interrupt specific configuration.
 */
typedef struct Msgcom_DirectInterruptCfg
{
    /**
     * @brief   Queue pend queue which is being used
     */
    Qmss_Queue              queuePendQueue;

    /**
     * @brief   Interrupt identifier to which the direct interrupt queue is mapped to.
     * In the DSP realm; direct interrupt queues are mapped via the CPINTC block. This
     * is the CPINTC identifier block to be used. This configuration is *NOT* used in the
     * ARM realm since the interrupts are mapped via the GIC.
     */
    int32_t                 cpIntcId;

    /**
     * @brief   This is the system interrupt to which the queue pend is mapped to.
     * This field is *only* application from the DSP realm and is NOT used in the
     * ARM realm.
     */
    int32_t                 systemInterrupt;

    /**
     * @brief   Host Interrupt to which the direct interrupt queues are mapped to.
     * - DSP the queue pend queues are wired via the CPINTC block and so this is
     *   the host interrupt to which the system interrupt will be routed to.
     * - ARM the queue pend queues are wired via the GIC block and so this is the
     *   GIC event identifier.
     */
    uint32_t                hostInterrupt;
}Msgcom_DirectInterruptCfg;

/**
 * @brief
 *  MSGCOM Queue Channel Information.
 *
 * @details
 *  The structure describes the Queue channel information which is required to be
 *  specified while creating a QUEUE channel. Queue channels are used to transfer
 *  data by simply sharing the descriptor information; there is NO DMA of data
 *  between the reader & writer and this warrants that the memory contents be visible
 *  between the two entities
 */
typedef struct Msgcom_QueueChannelCfg
{
    /**
     * @brief   Specifies the interrupt mode for the channel.
     */
    Msgcom_QueueInterruptMode               interruptMode;

    /**
     * @brief   Interrupt specific configuration
     */
    union  queueIntrUnion
    {
        /**
         * @brief   Accumulated interrupt specific configuration
         */
        Msgcom_AccumulatorInterruptCfg      accCfg;

        /**
         * @brief   Direct Interrupt specific configuration
         */
        Msgcom_DirectInterruptCfg           queuePendCfg;
    }queueIntrUnion;
}Msgcom_QueueChannelCfg;

/**
 * @brief
 *  Message Communicator Queue DMA Channel Information.
 *
 * @details
 *  The structure describes the Queue DMA channel information which is required to be
 *  specified while creating a QUEUE-DMA channel. These channels are used to transfer
 *  data by ensuring that there is a DMA transfer of data between the reader and
 *  writer.
 */
typedef struct Msgcom_QueueDMAChannelCfg
{
    /**
     * @brief   Specifies the interrupt mode for the channel.
     */
    Msgcom_QueueInterruptMode           interruptMode;

    /**
     * @brief   Receive Free Queue which has the descriptors and buffers linked together.
     * This queue handle is programmed into the Receive Flow
     */
    int32_t                             rxFreeQueueNum;

    /**
     * @brief   Interrupt specific configuration
     */
    union queueDMAIntrUnion
    {
        /**
         * @brief   Accumulated interrupt specific configuration
         */
        Msgcom_AccumulatorInterruptCfg      accCfg;

        /**
         * @brief   Direct Interrupt specific configuration
         */
        Msgcom_DirectInterruptCfg           queuePendCfg;
    }queueDMAIntrUnion;
}Msgcom_QueueDMAChannelCfg;

/**
 * @brief
 *  Message Communicator Virtual Channel Configuration.
 *
 * @details
 *  Virtual channels are configured over a PHYSICAL channel. These channels inherit the
 *  same properties & hardware configuration as the physical channels. However the virtual
 *  channels require some additional configuration which is described in this data structure.
 */
typedef struct Msgcom_VirtualChannelCfg
{
    /**
     * @brief  This is the Physical channel over which the virtual channel will reside
     */
    MsgCom_ChHandle                     phyChannel;
}Msgcom_VirtualChannelCfg;

/**
 * @brief
 *  Message Communicator Channel Configuration.
 *
 * @details
 *  This structure carries specific configuration information specific to a channel
 *  type.
 */
typedef struct Msgcom_ChannelCfg
{
    /**
     * @brief   This is the mode in which the channel is operating.
     */
    Msgcom_ChannelMode                  mode;

    /**
     * @brief   MSGCOM Instance handle associated with the channel. Channels exist
     * only in the context of the MSGCOM instance.
     */
    Msgcom_InstHandle                   msgcomInstHandle;

    /**
     * @brief   Application reader specified callback function which is invoked
     * by the message communicator module once data has been received on the
     * specified channel.
     */
    MsgCom_AppCallBack                  appCallBack;

    /**
     * @brief   Optional application defined argument which is passed to the application
     * during the application callback.
     */
    uint32_t                            arg;

    Queue_BarrierType                  queueBarrierType; //fzm

    /**
     * @brief   Channel configuration is dependent on the type of channel.
     */
    union                               u
    {
        /**
         * @brief   This is the Queue Based channel configuration information.
         */
        Msgcom_QueueChannelCfg          queueCfg;

        /**
         * @brief   This is the Queue-DMA Based channel configuration information.
         */
        Msgcom_QueueDMAChannelCfg       queueDMACfg;

        /**
         * @brief   This is the Virtual Channel Configuration.
         */
        Msgcom_VirtualChannelCfg        virtualChannelCfg;
    }u;
}Msgcom_ChannelCfg;

/**
 * @brief
 *  MSGCOM instance configuration
 *
 * @details
 *  Each processing entity (DSP core or ARM process) needs to create a MSGCOM
 *  instance. MSGCOM channels are created and exist only within each instance.
 */
typedef struct Msgcom_InstCfg
{
    /**
     * @brief   Database handle associated with the MSGCOM instance. Information
     * for the MSGCOM channels created & found using this instance will refer only
     * to this specific database
     */
    Name_DBHandle                   databaseHandle;

    /**
     * @brief   This is the handle to the system configuration.
     */
    Resmgr_SysCfgHandle             sysCfgHandle;

    /**
     * @brief   This is the handle to the PKTLIB instance. Packets which are
     * received and sent via the MSGCOM channels belong to this PKTLIB instance
     */
    Pktlib_InstHandle               pktlibInstHandle;

    /**
     * @brief   OSAL Malloc
     */
    Osal_MsgcomMalloc               malloc;

    /**
     * @brief   OSAL Free
     */
    Osal_MsgcomFree                 free;

    /**
     * @brief   OSAL Register ISR
     */
    Osal_MsgcomRegisterIsr          registerIsr;

    /**
     * @brief   OSAL Deregister ISR
     */
    Osal_MsgcomDeregisterIsr        deregisterIsr;

    /**
     * @brief   OSAL Disable System Interrupt
     */
    Osal_MsgcomDisableSysInt        disableSysInt;

    /**
     * @brief   OSAL Enable System Interrupt
     */
    Osal_MsgcomEnableSysInt         enableSysInt;

    /**
     * @brief   OSAL Enter Critical Section
     */
    Osal_MsgcomEnterSingleCoreCS    enterCS;

    /**
     * @brief   OSAL Exit Critical Section
     */
    Osal_MsgcomExitSingleCoreCS     exitCS;

    /**
     * @brief   OSAL Semaphore creation
     */
    Osal_MsgcomCreateSem            createSem;

    /**
     * @brief   OSAL Semaphore deletion
     */
    Osal_MsgcomDeleteSem            deleteSem;

    /**
     * @brief   OSAL Post Semaphore
     */
    Osal_MsgcomPostSem              postSem;

    /**
     * @brief   OSAL Pend Semaphore
     */
    Osal_MsgcomPendSem              pendSem;
}Msgcom_InstCfg;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern Msgcom_InstHandle Msgcom_createInstance(Msgcom_InstCfg* ptrInstCfg, int32_t* errCode);
extern int32_t Msgcom_deleteInstance(Msgcom_InstHandle instHandle, int32_t* errCode);
extern MsgCom_ChHandle Msgcom_create(const char* name, Msgcom_ChannelType channelType, Msgcom_ChannelCfg* ptrChCfg, int32_t* errCode);
extern MsgCom_ChHandle Msgcom_find (Msgcom_InstHandle msgcomInstHandle, const char* channelName, int32_t* errCode);
extern int32_t Msgcom_delete(MsgCom_ChHandle msgChHandle, Msgcom_freePkt freePkt);
extern int32_t Msgcom_putMessage(MsgCom_ChHandle msgChHandle, MsgCom_Buffer* msgBuffer);
extern int32_t Msgcom_getMessage(MsgCom_ChHandle msgChHandle, MsgCom_Buffer** msgBuffer);
extern void    Msgcom_channelRxHandler(MsgCom_ChHandle msgChHandle);
extern int32_t Msgcom_getInternalMsgQueueInfo(MsgCom_ChHandle msgChHandle);
extern int32_t Msgcom_registerCallBack (MsgCom_ChHandle msgChHandle, MsgCom_AppCallBack appCallback, uint32_t arg);

#ifdef __cplusplus
}
#endif

#endif /* __MSG_COM_H__ */

