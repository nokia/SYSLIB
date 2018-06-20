/**
 *   @file  k2k.c
 *
 *   @brief
 *      Device specific K2K file for the SOC Initialization.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <signal.h>
#include <getopt.h>
#include <getopt.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/runtime/hplib/hplib.h>
#include <ti/csl/csl_psc.h>

/* SYSLIB Include Files */
#include <ti/apps/soc_init/include/soc_init.h>
#include <ti/runtime/resmgr/resmgr.h>

/* BCP files */
#include <ti/drv/bcp/bcp.h>

/* EMIF include files */
#include <ti/csl/cslr_emif4f.h>

/**********************************************************************
 ************************** Extern Variables **************************
 **********************************************************************/

/* Global MCB which keeps track of the SOC Initialization information: */
extern SOCInit_MCB      gSocInitMCB;

/**********************************************************************
 ************************** Static Functions **************************
 **********************************************************************/

/* Static power up functions: */
static int32_t SOCInit_powerBCP(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock);
static int32_t SOCInit_powerFFTC(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock);
static int32_t SOCInit_powerAIF(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock);

/* Device specific power up table: This maps the CPDMA id to the power-up function */
SOCInit_powerUpFxn  gDevicePowerUpTable[] =
{
    NULL,                   /* SRIO     */
    SOCInit_powerAIF,       /* AIF      */
    SOCInit_powerFFTC,      /* FFTC-A   */
    SOCInit_powerFFTC,      /* FFTC-B   */
    SOCInit_powerFFTC,      /* FFTC-C   */
    SOCInit_powerFFTC,      /* FFTC-D   */
    SOCInit_powerFFTC,      /* FFTC-E   */
    SOCInit_powerFFTC,      /* FFTC-F   */
    NULL,                   /* NETCP    */
    NULL,                   /* QMSS     */
    NULL,                   /* QMSS-2   */
    SOCInit_powerBCP,       /* BCP      */
};

/* On K2K there are 4 ARM cores. */
const int32_t   gNumARMCores    = 4;

/**********************************************************************
 ************************ K2H Functions *******************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to validate the DMA blocks.
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t SOCInit_validateDMABlocks(SOCInit_MCB* ptrSOCInitMCB)
{
    SOCInit_DMABlock*   ptrSOCInitDMABlock;

    /* Cycle through all the DMA Blocks: */
    ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetHead ((SOCInit_ListNode**)&gSocInitMCB.ptrDMABlockList);
    while (ptrSOCInitDMABlock != NULL)
    {
        /* Ensure that the CPDMA Identifiers are valid and correct. These are the only one supported for the device */
        switch (ptrSOCInitDMABlock->cpdmaId)
        {
            case Cppi_CpDma_SRIO_CPDMA:
                break;
            case Cppi_CpDma_AIF_CPDMA:
                break;
            case Cppi_CpDma_FFTC_A_CPDMA:
                break;
            case Cppi_CpDma_FFTC_B_CPDMA:
                break;
            case Cppi_CpDma_FFTC_C_CPDMA:
                break;
            case Cppi_CpDma_FFTC_D_CPDMA:
                break;
            case Cppi_CpDma_FFTC_E_CPDMA:
                break;
            case Cppi_CpDma_FFTC_F_CPDMA:
                break;
            case Cppi_CpDma_PASS_CPDMA:
            {
                /* PASS CPDMA has already been configured by the Linux Kernel: */
                if (ptrSOCInitDMABlock->powerUp == 1)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: PASS DMA is already powered up by the Linux kernel. Please fix conf file\n");
                    return -1;
                }
                break;
            }
            case Cppi_CpDma_QMSS_CPDMA:
            {
                /* QMSS CPDMA has already been configured by the Linux Kernel: */
                if (ptrSOCInitDMABlock->powerUp == 1)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: QMSS DMA is already powered up by the Linux kernel. Please fix conf file\n");
                    return -1;
                }
                break;
            }
            case Cppi_CpDma_QMSS_QM2_CPDMA:
            {
                /* QMSS2 CPDMA has already been configured by the Linux Kernel: */
                if (ptrSOCInitDMABlock->powerUp == 1)
                {
                    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: QMSS2 DMA is already powered up by the Linux kernel. Please fix conf file\n");
                    return -1;
                }
                break;
            }
            case Cppi_CpDma_BCP_CPDMA:
                break;
            default:
            {
                SOCInit_log (SOCInit_LogLevel_ERROR, "Error: DMA '%s' is not supported. Please fix conf file\n", ptrSOCInitDMABlock->name);
                return -1;
            }
        }

        /* Get the next DMA block */
        ptrSOCInitDMABlock = (SOCInit_DMABlock*)SOCInit_listGetNext((SOCInit_ListNode*)ptrSOCInitDMABlock);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the BCP.
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
static int32_t SOCInit_powerBCP(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock)
{
    /* Set BCP Power domain to ON */
    CSL_PSC_enablePowerDomain (ptrSOCInitMCB->pscReg, CSL_PSC_PD_BCP);

    /* Enable the clocks too for BCP */
    CSL_PSC_setModuleNextState (ptrSOCInitMCB->pscReg, CSL_PSC_LPSC_BCP, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (ptrSOCInitMCB->pscReg, CSL_PSC_PD_BCP);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (ptrSOCInitMCB->pscReg, CSL_PSC_PD_BCP));

    /* Return BCP PSC status */
    if ((CSL_PSC_getPowerDomainState(ptrSOCInitMCB->pscReg, CSL_PSC_PD_BCP) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (ptrSOCInitMCB->pscReg, CSL_PSC_LPSC_BCP) == PSC_MODSTATE_ENABLE))
    {
        /* BCP ON. Ready for use */
        return 0;
    }
    else
    {
        /* BCP Power on failed. Return error */
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the AIF.
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
static int32_t SOCInit_powerAIF(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock)
{
    /* Set  Power domain to ON */
    CSL_PSC_enablePowerDomain (ptrSOCInitMCB->pscReg, CSL_PSC_PD_AIF);

    /* Enable the clocks too  */
    CSL_PSC_setModuleNextState (ptrSOCInitMCB->pscReg, CSL_PSC_LPSC_AIF, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (ptrSOCInitMCB->pscReg, CSL_PSC_PD_AIF);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (ptrSOCInitMCB->pscReg, CSL_PSC_PD_AIF));

    /* Return AIF PSC status */
    if ((CSL_PSC_getPowerDomainState(ptrSOCInitMCB->pscReg, CSL_PSC_PD_AIF) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (ptrSOCInitMCB->pscReg, CSL_PSC_LPSC_AIF) == PSC_MODSTATE_ENABLE))
    {
        return 0;
    }
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the FFTC.
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
static int32_t SOCInit_powerFFTC(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DMABlock* ptrSOCInitDMABlock)
{
    uint32_t    powerDomain;
    uint32_t    moduleNum;

    /* Determine the power domain: */
    if ((ptrSOCInitDMABlock->cpdmaId == Cppi_CpDma_FFTC_A_CPDMA) || (ptrSOCInitDMABlock->cpdmaId == Cppi_CpDma_FFTC_B_CPDMA))
    {
        powerDomain = CSL_PSC_PD_FFTC_01;
    }
    else
    {
        powerDomain = CSL_PSC_PD_FFTC_2345;
    }

    /* Determine the module number: */
    switch (ptrSOCInitDMABlock->cpdmaId)
    {
        case Cppi_CpDma_FFTC_A_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_0;
            break;
        }
        case Cppi_CpDma_FFTC_B_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_1;
            break;
        }
        case Cppi_CpDma_FFTC_C_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_2;
            break;
        }
        case Cppi_CpDma_FFTC_D_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_3;
            break;
        }
        case Cppi_CpDma_FFTC_E_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_4;
            break;
        }
        case Cppi_CpDma_FFTC_F_CPDMA:
        {
            moduleNum = CSL_PSC_LPSC_FFTC_5;
            break;
        }
        default:
        {
            /* Error: Invalid CPDMA identifier */
            return -1;
        }
    }

    /* Set FFTC Power domain to ON for all FFTC devices */
    CSL_PSC_enablePowerDomain (ptrSOCInitMCB->pscReg, powerDomain);

    /* Enable the clocks too for FFTC */
    CSL_PSC_setModuleNextState (ptrSOCInitMCB->pscReg, moduleNum, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (ptrSOCInitMCB->pscReg, powerDomain);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (ptrSOCInitMCB->pscReg, powerDomain));

    /* Return FFTC PSC status */
    if ((CSL_PSC_getPowerDomainState(ptrSOCInitMCB->pscReg, powerDomain) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (ptrSOCInitMCB->pscReg, moduleNum) == PSC_MODSTATE_ENABLE))
    {
        /* FFTC ON. Ready for use */
        return 0;
    }
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the TCP3D
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]   instanceId
 *      TCP3D Instance identifier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SOCInit_powerTCP3d(SOCInit_MCB* ptrSOCInitMCB, int32_t instanceId)
{
    uint32_t tcp3dPowerReg;
    uint32_t tcp3dPowerRegL;

    /* TCP3D power domain is turned OFF by default.
     * It needs to be turned on before doing any TCP3D device register access.
     */
    switch (instanceId)
    {
        case 0:
        {
            tcp3dPowerReg = CSL_PSC_PD_TCP3D_01;
            tcp3dPowerRegL = CSL_PSC_LPSC_TCP3D_0;
            break;
        }
        case 1:
        {
            tcp3dPowerReg = CSL_PSC_PD_TCP3D_01;
            tcp3dPowerRegL = CSL_PSC_LPSC_TCP3D_1;
            break;
        }
        default:
        {
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: TCP3d '%d' is not supported. Please fix conf file\n", instanceId);
            return -1;
        }
    }

    /* Set TCP3D Power domain to ON */
    CSL_PSC_enablePowerDomain (ptrSOCInitMCB->pscReg, tcp3dPowerReg);

    /* Enable the clocks too for TCP3D */
    CSL_PSC_setModuleNextState (ptrSOCInitMCB->pscReg, tcp3dPowerRegL, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (ptrSOCInitMCB->pscReg, tcp3dPowerReg);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (ptrSOCInitMCB->pscReg, tcp3dPowerReg));

    /* Return TCP3D PSC status */
    if (!((CSL_PSC_getPowerDomainState(ptrSOCInitMCB->pscReg, tcp3dPowerReg) == PSC_PDSTATE_ON) &&
          (CSL_PSC_getModuleState (ptrSOCInitMCB->pscReg, tcp3dPowerRegL) == PSC_MODSTATE_ENABLE)))
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the VCP
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]   instanceId
 *      VCP Instance identifier
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SOCInit_powerVCP(SOCInit_MCB* ptrSOCInitMCB, int32_t instanceId)
{
    uint32_t vcpPowerReg;
    uint32_t vcpPowerRegL;

    /* VCP power domain is turned OFF by default.
     * It needs to be turned on before doing any VCP device register access.
     */
    switch (instanceId)
    {
        case 0:
        {
            vcpPowerReg = CSL_PSC_PD_VCP_0123;
            vcpPowerRegL = CSL_PSC_LPSC_VCP_0;
            break;
        }
        case 1:
        {
            vcpPowerReg = CSL_PSC_PD_VCP_0123;
            vcpPowerRegL = CSL_PSC_LPSC_VCP_1;
            break;
        }
        case 2:
        {
            vcpPowerReg = CSL_PSC_PD_VCP_0123;
            vcpPowerRegL = CSL_PSC_LPSC_VCP_2;
            break;
        }
        case 3:
        {
            vcpPowerReg = CSL_PSC_PD_VCP_0123;
            vcpPowerRegL = CSL_PSC_LPSC_VCP_3;
            break;
        }
        default:
        {
            SOCInit_log (SOCInit_LogLevel_ERROR, "Error: VCP '%d' is not supported. Please fix conf file\n", instanceId);
            return -1;
        }
    }

    /* Set VCP  Power domain to ON */
    CSL_PSC_enablePowerDomain (ptrSOCInitMCB->pscReg, vcpPowerReg);

    /* Enable the clocks too for VCP */
    CSL_PSC_setModuleNextState (ptrSOCInitMCB->pscReg, vcpPowerRegL, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (ptrSOCInitMCB->pscReg, vcpPowerReg);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (ptrSOCInitMCB->pscReg, vcpPowerReg));

    /* Return VCP PSC status */
    if (!((CSL_PSC_getPowerDomainState(ptrSOCInitMCB->pscReg, vcpPowerReg) == PSC_PDSTATE_ON) &&
          (CSL_PSC_getModuleState (ptrSOCInitMCB->pscReg, vcpPowerRegL) == PSC_MODSTATE_ENABLE)))
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to power up the peripheral
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]   ptrSOCInitPeripheralBlock
 *      Pointer to the SOC Peripheral block to be powered up
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t SOCInit_powerPeripheral(SOCInit_MCB* ptrSOCInitMCB, SOCInit_PeripheralBlock* ptrSOCInitPeripheralBlock)
{
    if (strcmp(ptrSOCInitPeripheralBlock->name,"tcp3d")==0)
    {
        return SOCInit_powerTCP3d(ptrSOCInitMCB, ptrSOCInitPeripheralBlock->instanceId);
    }
    if (strcmp(ptrSOCInitPeripheralBlock->name,"vcp")==0)
    {
        return SOCInit_powerVCP(ptrSOCInitMCB, ptrSOCInitPeripheralBlock->instanceId);
    }
    SOCInit_log (SOCInit_LogLevel_ERROR, "Error: Peripheral '%s' is not supported. Please fix conf file\n", ptrSOCInitPeripheralBlock->name);
    return -1;
}


/**
 *  @b Description
 *  @n
 *      The function is used to populate the BCP LLD Object
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]  instNum
 *      BCP Instance number
 *  @param[out] pBcpLldObj
 *      Pointer to the BCP LLD Object
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t SOCInit_bcpFillLLDObj
(
    SOCInit_MCB*                ptrSOCInitMCB,
    uint8_t                     instNum,
    Bcp_LldObj*                 pBcpLldObj
)
{
    int32_t                     i;

    /* Validate the BCP LLD Object Handle passed */
    if (pBcpLldObj == NULL)
        return -1;

    pBcpLldObj->instNum = instNum;

    /* Setup the base addresses for CFG, Interrupt and Data logger registers for all BCP submodules. */
    for (i = 0; i < BCP_NUM_SUBMODULES; i ++)
    {
        switch (i)
        {
            case Bcp_ModuleId_TM:
            {
                /* PPB0..PPB3 are all part of TM and hence share the same set of MMRs */
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0000);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0080);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x00F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x2000);

                break;
            }
            case Bcp_ModuleId_INT:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0500);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0580);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x05F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x4000);

                break;
            }
            case Bcp_ModuleId_RM:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0600);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0680);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x06F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x5000);

                break;
            }
            case Bcp_ModuleId_ENC:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0700);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0780);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x07F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x6000);

                break;
            }
            case Bcp_ModuleId_MOD:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0800);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0880);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x08F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x7000);

                break;
            }
            case Bcp_ModuleId_CRC:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0D00);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0D80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0DF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0xC000);

                break;
            }
            case Bcp_ModuleId_SSL:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0A00);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0A80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0AF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x9000);

                break;
            }
            case Bcp_ModuleId_RD:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0B00);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0B80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0BF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0xA000);

                break;
            }
            case Bcp_ModuleId_COR:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0C00);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0C80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0CF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0xB000);

                break;
            }
            case Bcp_ModuleId_DNT:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0900);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0980);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x09F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x8000);

                break;
            }
            case Bcp_ModuleId_DIO:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (ptrSOCInitMCB->BCPCfgRegs + 0x0400);
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x0480);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (ptrSOCInitMCB->BCPCfgRegs + 0x04F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (ptrSOCInitMCB->BCPCfgRegs + 0x3000);

                break;
            }
            default:
            {
                break;
            }
        }
    }

    /* BCP LLD Open successful. Return success. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the BCP device
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]  ptrSOCInitBCPBlock
 *      Pointer to the BCP block to be initialized
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t SOCInit_initBCPDevice(SOCInit_MCB* ptrSOCInitMCB, SOCInit_BCPBlock* ptrSOCInitBCPBlock)
{
    Bcp_LldObj                 bcpLldObj;
    Bcp_LldObj*                pBcpLldObj = &bcpLldObj;

    /* create a LLD obj here so that we can reuse the BCP drivers function directly */
    if (SOCInit_bcpFillLLDObj(ptrSOCInitMCB,ptrSOCInitBCPBlock->instanceId,pBcpLldObj)<0)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Bcp_fillLLDObj failed\n");
        return -1;
    }

    /* Setup TM MMR */
    Bcp_setCdmaHpSrcId (pBcpLldObj, 0xbc);
    Bcp_enableTxCdmaHpReadArbPrio (pBcpLldObj);

    Bcp_setTxQfifoReadDestSelReg (pBcpLldObj, ptrSOCInitBCPBlock->Q_to_PPB_map, ptrSOCInitBCPBlock->Qpri);

    /* Initialize the Encoder engine block */
    Bcp_setEncPolyCoef1Reg (pBcpLldObj, 0x4D9413);      //WCDMA code rate 1/2
    Bcp_setEncPolyCoef2Reg (pBcpLldObj, 0xD5AA5F);      //WCDMA code rate 1/3
    Bcp_setEncPolyCoef3Reg (pBcpLldObj, 0xDDC3C0);      //LTE code rate 1/3

    Bcp_setEncScrInit0Reg (pBcpLldObj, 0x0);            //scrambler initialization
    Bcp_setEncScrPoly0Reg (pBcpLldObj, 0x0);            //scrambler polynomial
    Bcp_setEncCrc24Init0Reg (pBcpLldObj, 0x0);          //crc24 initialization
    Bcp_setEncCrc24Poly0Reg (pBcpLldObj, 0x800063);     //crc24 polynomial

    Bcp_setEncScrInit1Reg (pBcpLldObj, 0x6E2A);         //scrambler initialization
    Bcp_setEncScrPoly1Reg (pBcpLldObj, 0x0006);         //scrambler polynomial
    Bcp_setEncCrc16Init1Reg (pBcpLldObj, 0x0);          //crc16 initialization
    Bcp_setEncCrc16Poly1Reg (pBcpLldObj, 0x1021);       //crc16 polynomial

    Bcp_setEncScrInit2Reg (pBcpLldObj, 0x6E2A);         //scrambler initialization
    Bcp_setEncScrPoly2Reg (pBcpLldObj, 0x0006);         //scrambler polynomial

    /* Initialize the Correlator MMR */
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 0, 0xFFFFF);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 1, 0x5A933);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 2, 0x10E5A);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 3, 0x6339C);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 4, 0x7C3E0);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 5, 0xFFC00);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 6, 0xD8E64);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 7, 0x4F5B0);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 8, 0x218EC);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 9, 0x1B746);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 10, 0xFFFF);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 11, 0x33FFF);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 12, 0x3FFFC);
    Bcp_setCorControlReg (pBcpLldObj, 0x0, 0x1, 0x2, 0x3);

    {
        //TCP3d overflow protection for K2 devices
        CSL_Bcp_rdRegs* pRdRegs    =   (CSL_Bcp_rdRegs *) (pBcpLldObj->modCfgRegs[Bcp_ModuleId_RD]);
        CSL_FINS (pRdRegs->LTE_SATVAL, BCP_RD_LTE_SATVAL_LLRSAT_0, 0x1F);
        CSL_FINS (pRdRegs->LTE_SATVAL, BCP_RD_LTE_SATVAL_LLRSAT_1, 0x1F);
        CSL_FINS (pRdRegs->LTE_SATVAL, BCP_RD_LTE_SATVAL_BETASAT_0, 0x1F);
        CSL_FINS (pRdRegs->LTE_SATVAL, BCP_RD_LTE_SATVAL_BETASAT_1, 0x1F);


        CSL_FINS (pRdRegs->XCDMA_HSPA_SATVAL, BCP_RD_XCDMA_HSPA_SATVAL_LLRSAT_0, 0x1F);
        CSL_FINS (pRdRegs->XCDMA_HSPA_SATVAL, BCP_RD_XCDMA_HSPA_SATVAL_LLRSAT_1, 0x1F);
        CSL_FINS (pRdRegs->XCDMA_HSPA_SATVAL, BCP_RD_XCDMA_HSPA_SATVAL_BETASAT_0, 0x1F);
        CSL_FINS (pRdRegs->XCDMA_HSPA_SATVAL, BCP_RD_XCDMA_HSPA_SATVAL_BETASAT_1, 0x1F);

        CSL_FINS (pRdRegs->XCDMA_R99_SATVAL, BCP_RD_XCDMA_R99_SATVAL_LLRSAT_0, 0x1F);
        CSL_FINS (pRdRegs->XCDMA_R99_SATVAL, BCP_RD_XCDMA_R99_SATVAL_LLRSAT_1, 0x1F);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the DDR3
 *
 *  @param[in]  ptrSOCInitMCB
 *      Pointer to the application MCB
 *  @param[in]  ptrSOCInitDDRBlock
 *      Pointer to the DDR3 configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t SOCInit_configureDDR(SOCInit_MCB* ptrSOCInitMCB, SOCInit_DDRBlock* ptrSOCInitDDRBlock)
{
    uint32_t            emifDDRBaseAddr;
    CSL_Emif4fvRegs*    emifDDRCfgRegs;

    if (ptrSOCInitDDRBlock->instanceId > 1)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Invalid instance id (%d) for this device\n",ptrSOCInitDDRBlock->instanceId);
        return -1;
    }
    emifDDRBaseAddr = ptrSOCInitDDRBlock->instanceId?CSL_DDR3_1_SLV_CFG_REGS:CSL_DDR3_0_SLV_CFG_REGS;
    emifDDRCfgRegs = (CSL_Emif4fvRegs*)hplib_VM_MemMap ((void*)emifDDRBaseAddr, 0x1000);
    if (emifDDRCfgRegs == NULL)
    {
        SOCInit_log(SOCInit_LogLevel_ERROR, "Error: Memory mapping EMIF DDR register space failed\n");
        return -1;
    }
    SOCInit_log(SOCInit_LogLevel_DEBUG, "Debug: EMIF DDR reg mapping successful\n");

    /* Configure the DDR3 */
    emifDDRCfgRegs->VBUSM_CONFIG    = ptrSOCInitDDRBlock->vbusm_config;
    emifDDRCfgRegs->MSTID_COS_1_MAP = ptrSOCInitDDRBlock->mstid_cos_1_map;
    emifDDRCfgRegs->MSTID_COS_2_MAP = ptrSOCInitDDRBlock->mstid_cos_2_map;
    return 0;
}

