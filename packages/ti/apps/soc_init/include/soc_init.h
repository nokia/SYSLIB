/**
 *   @file  soc_init.h
 *
 *   @brief
 *      SYSLIB SOC Initialization Header file
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2015 Texas Instruments, Inc.
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
#ifndef __SOC_INIT_H__
#define __SOC_INIT_H__

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
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <signal.h>
#include <getopt.h>

/* MCSDK Include files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>
#include <ti/csl/csl_psc.h>

/* SYSLIB Include Files */
#include <ti/apps/soc_init/include/listlib.h>
#include <ti/runtime/resmgr/resmgr.h>

/* BCP include files */
#include <ti/drv/bcp/bcp_lld.h>


/**
@defgroup SOC_INIT_SYMBOL  SYSLIB SOC Init Defined Symbols
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_FUNCTION  SYSLIB SOC Init  Exported Functions
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_DATA_STRUCTURE  SYSLIB SOC Init  Data Structures
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_ENUM  SYSLIB SOC Init  Enumerations
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_INTERNAL_ENUM  SYSLIB SOC Init Internal Enumerations
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_INTERNAL_DATA_STRUCTURE  SYSLIB SOC Init Internal Data Structure
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_INTERNAL_FUNCTION  SYSLIB SOC Init Internal Functions
@ingroup SOC_INIT_LIB_API
*/
/**
@defgroup SOC_INIT_ERROR_CODE  SYSLIB SOC Init Error code
@ingroup SOC_INIT_LIB_API
*/

/**
@addtogroup SOC_INIT_SYMBOL
@{
*/

/**
 * @brief   Maximum number of characters
 */
#define SOCINIT_MAX_CHAR            32

/**
@}
*/

/**
@addtogroup SOC_INIT_INTERNAL_ENUM
@{
*/

/**
 * @brief
 *  Enumeration for logging levels
 *
 * @details
 *  Log messages are logged at the following levels.
 */
typedef enum SOCInit_LogLevel
{
    /**
     * @brief  Debug messages
     */
    SOCInit_LogLevel_DEBUG = 0x1,

    /**
     * @brief  Information messages
     */
    SOCInit_LogLevel_INFO = 0x2,

    /**
     * @brief  Error messages
     */
    SOCInit_LogLevel_ERROR = 0x3
}SOCInit_LogLevel;

/**
@}
*/

/**
@addtogroup SOC_INIT_INTERNAL_DATA_STRUCTURE
@{
*/

/**
 * @brief
 *  SOC Init DMA Block
 *
 * @details
 *  The SOC Init Master control block which keeps track of all the
 *  persistent information used by the SYSLIB SOC Initialization
 *  application
 */
typedef struct SOCInit_DMABlock
{
    /**
     * @brief  Links to other DMA blocks
     */
    SOCInit_ListNode        links;

    /**
     * @brief  Name of the DMA Block.
     */
    char                    name[SOCINIT_MAX_CHAR];

    /**
     * @brief  CPDMA Identifier: The DMA identifier should match the CPPI identifier
     */
    uint32_t                cpdmaId;

    /**
     * @brief  Power Up: Status flag which indicates if the CPDMA should be powered up.
     */
    uint32_t                powerUp;

    /**
     * @brief  CPDMA FIFO Write depth:
     */
    uint32_t                writeFifoDepth;

    /**
     * @brief  CPDMA Timeout count:
     */
    uint32_t                timeoutCount;

    /**
     * @brief  Priority of all Receive transactions of this CPDMA
     */
    uint32_t                rxPriority;

    /**
     * @brief  Priority of all Transmit transactions of this CPDMA
     */
    uint32_t                txPriority;
}SOCInit_DMABlock;

/**
 * @brief
 *  SOC Init Peripheral Block
 *
 * @details
 *  The structure describes a SOC Peripheral.
 */
typedef struct SOCInit_PeripheralBlock
{
    /**
     * @brief  Links to other peripheral blocks
     */
    SOCInit_ListNode        links;

    /**
     * @brief  Name of the peripheral Block.
     */
    char                    name[SOCINIT_MAX_CHAR];

    /**
     * @brief  Peripheral instance number
     */
    uint32_t                instanceId;

    /**
     * @brief  Power Up: Status flag which indicates if the peripheral should be powered up.
     */
    uint32_t                powerUp;
}SOCInit_PeripheralBlock;

/**
 * @brief
 *  SOC Init BCP Block
 *
 * @details
 *  The structure describes the BCP Block
 */
typedef struct SOCInit_BCPBlock
{
    /**
     * @brief  Links to other BCP blocks
     */
    SOCInit_ListNode        links;

    /**
     * @brief  peripheral instance number
     */
    uint32_t                instanceId;

    /**
     * @brief  Queues to Port Mappings
     */
    uint8_t                 Q_to_PPB_map [BCP_MAX_NUM_TXQUEUES];

    /**
     * @brief  Queues to Port Mappings
     */
    uint8_t                 Qpri [BCP_MAX_NUM_TXQUEUES];
}SOCInit_BCPBlock;

/**
 * @brief
 *  SOC Init DDR Block
 *
 * @details
 *  The structure describes the DDR3 Block
 */
typedef struct SOCInit_DDRBlock
{
    /**
     * @brief  Links to other DDR3 blocks
     */
    SOCInit_ListNode        links;

    /**
     * @brief  peripheral instance number
     */
    uint32_t                instanceId;

    /**
     * @brief  VBUSM_CONFIG register value
     */
    uint32_t                 vbusm_config;

    /**
     * @brief  MSTID_COS_1_MAP register value
     */
    uint32_t                 mstid_cos_1_map;

    /**
     * @brief  MSTID_COS_2_MAP register value
     */
    uint32_t                 mstid_cos_2_map;
}SOCInit_DDRBlock;

/**
 * @brief
 *  SOC Init MCB
 *
 * @details
 *  The SOC Init Master control block which keeps track of all the
 *  persistent information used by the SYSLIB SOC Initialization
 *  application
 */
typedef struct SOCInit_MCB
{
    /**
     * @brief   Name of the configuration file
     */
    char                        cfgFile[PATH_MAX];

    /**
     * @brief   Log Level for the application
     */
    SOCInit_LogLevel            logLevel;

    /**
     * @brief Global System configuration handle
     */
    Resmgr_SysCfgHandle         handleSysCfg;

    /**
     * @brief Pointer to the PSC Register space for powering up the DMA blocks.
     */
    CSL_PscRegs*                pscReg;

    /**
     * @brief Pointer to the BCP CFG Register space
     */
    uint32_t                    BCPCfgRegs;

    /**
     * @brief RM Client Name
     */
    char                        rmClientName[128];

    /**
     * @brief   Pointer to the CPDMA Block list
     */
    SOCInit_DMABlock*           ptrDMABlockList;

    /**
     * @brief   Pointer to the peripheral Block list
     */
    SOCInit_PeripheralBlock*    ptrPeripheralBlockList;

    /**
     * @brief   Pointer to the BCP Block list
     */
    SOCInit_BCPBlock*           ptrBCPBlockList;

    /**
     * @brief   Pointer to the DDR Block list
     */
    SOCInit_DDRBlock*           ptrDDRBlockList;
}SOCInit_MCB;

/**
 *  @b Description
 *  @n
 *      This is the power up function which allows the CPDMA block to be powered up
 *      CPDMA blocks need to be powered up for them to be configured.
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]  ptrSOCInitDMABlock
 *      Pointer to the DMA Block
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
typedef int32_t (*SOCInit_powerUpFxn)(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock);

/**
@}
*/

/**
 *  @b Description
 *  @n
 *      This function enables the specified power domain.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  pwrDmnNum
 *      Power domain number
 *
 *  @retval
 *      Not applicable
 */
static inline void CSL_PSC_enablePowerDomain (CSL_PscRegs* ptrPSCRegs, uint32_t pwrDmnNum)
{
    CSL_FINST (ptrPSCRegs->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, ON);
}

/**
 *  @b Description
 *  @n
 *      This function sets up the "Next" state the module should be transitioned for
 *      a given module.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  moduleNum
 *      Module number
 *  @param[in]  state
 *      Next state to which the module must be transitioned.
 *
 *  @retval
 *      Not applicable
 */
static inline void CSL_PSC_setModuleNextState (CSL_PscRegs* ptrPSCRegs, uint32_t moduleNum, CSL_PSC_MODSTATE state)
{
    CSL_FINS (ptrPSCRegs->MDCTL[moduleNum], PSC_MDCTL_NEXT, state);
}

/**
 *  @b Description
 *  @n
 *      This function sets the 'GO' bit in the PSC Command register.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  pwrDmnNum
 *      Power domain number
 *
 *  @retval
 *      Not applicable
 */
static inline void CSL_PSC_startStateTransition (CSL_PscRegs* ptrPSCRegs, uint32_t pwrDmnNum)
{
    ptrPSCRegs->PTCMD =   (1 << pwrDmnNum);
}

/**
 *  @b Description
 *  @n
 *      This function gets the transition status of the power domain.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  pwrDmnNum
 *      Power domain number for which the state must be retrieved
 *
 *  @retval
 *      Power domain transition status value
 */
static inline uint32_t CSL_PSC_isStateTransitionDone (CSL_PscRegs* ptrPSCRegs, uint32_t pwrDmnNum)
{
    uint32_t  pdTransStatus;

    pdTransStatus = CSL_FEXTR (ptrPSCRegs->PTSTAT, pwrDmnNum, pwrDmnNum);
    if (pdTransStatus)
    {
        /* Power domain transition is in progress. Return 0 to indicate not yet done. */
        return 0;
    }
    else
    {
        /* Power domain transition is done. */
        return 1;
    }
}

/**
 *  @b Description
 *  @n
 *      This function returns the current state of a given power domain.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  pwrDmnNum
 *      Power domain number for which the state must be retrieved
 *
 *  @retval
 *      Power Domain status
 */
static inline CSL_PSC_PDSTATE CSL_PSC_getPowerDomainState (CSL_PscRegs* ptrPSCRegs, uint32_t pwrDmnNum)
{
    return (CSL_PSC_PDSTATE) CSL_FEXT(ptrPSCRegs->PDSTAT[pwrDmnNum], PSC_PDSTAT_STATE);
}

/**
 *  @b Description
 *  @n
 *      This function returns the current state of a given module.
 *
 *  @param[in]  ptrPSCRegs
 *      Pointer to the PSC registers
 *  @param[in]  moduleNum
 *      Module number
 *
 *  @retval
 *      Module state
 */
static inline CSL_PSC_MODSTATE CSL_PSC_getModuleState (CSL_PscRegs* ptrPSCRegs, uint32_t moduleNum)
{
    return (CSL_PSC_MODSTATE) CSL_FEXT(ptrPSCRegs->MDSTAT[moduleNum], PSC_MDSTAT_STATE);
}

/***********************************************************************************************
 ********************************** EXTERN Declarations ****************************************
 ***********************************************************************************************/

/* Device Exported API: */
extern int32_t SOCInit_validateDMABlocks(SOCInit_MCB* ptrSOCInitMCB);
extern int32_t SOCInit_powerPeripheral(SOCInit_MCB* ptrSOCInitMCB, SOCInit_PeripheralBlock* ptrSOCInitPeripheralBlock);
extern int32_t SOCInit_initBCPDevice(SOCInit_MCB* ptrSOCInitMCB, SOCInit_BCPBlock* ptrSOCInitBCPBlock);
extern int32_t SOCInit_configureDDR(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DDRBlock* ptrSOCInitDDRBlock);

/* Logging Function: */
extern void SOCInit_log (SOCInit_LogLevel level, char* fmt, ...);

#endif /* __SOC_INIT_H__ */

