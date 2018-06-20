/**
 *   @file  resmgr.c
 *
 *   @brief
 *      The file implements the resource management library
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
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* PDK Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

#ifdef __ARMv7
#include <sys/mman.h>
#include <semaphore.h>
#include <fcntl.h>
#include <limits.h>
#include <ti/runtime/hplib/hplib.h>
#endif

/* SYSLIB include files. */
#include <ti/runtime/resmgr/resmgr.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************ Internal Definitions ************************
 **********************************************************************/

/**
 * @brief   Maximum number of hardware semaphores supported on the device
 */
#define RESMGR_MAX_HW_SEM                       32

/**
 * @brief   Maximum number of accumulator channels supported on the device
 */
#define RESMGR_MAX_ACCUMULATOR_CHANNEL          64

/**
 * @brief   Maximum number of host interrupts supported on the device
 */
#define RESMGR_MAX_HOST_INTERRUPT               1024

/**
 * @brief   Maximum number of host interrupts supported on the device
 */
#define RESMGR_MAX_QUEUE_PEND                   64

/**********************************************************************
 ************************ Internal Structures *************************
 **********************************************************************/

/**
 * @brief
 *  Memory Region Block
 *
 * @details
 *  Memory regions allocated are tracked in this structure. This structure is then used
 *  to cleanup the allocated memory regions on the deinitialization of the RESMGR API.
 */
typedef struct Resmgr_MemoryRegionBlock
{
    /**
     * @brief  This is the base address of the memory allocated to the memory region.
     * Value of NULL implies that the memory region block is UNUSED.
     */
    void*                   memoryRegionBaseAddress;

    /**
     * @brief  This is the size of the memory region.
     */
    uint32_t                sizeMemoryRegion;

    /**
     * @brief   Memory Region configuration:
     */
    Resmgr_MemRegionCfg     cfg;
}Resmgr_MemoryRegionBlock;

/**
 * @brief
 *  Internal Semaphore structure
 *
 * @details
 *  The structure is a wrapper semaphore internal structure which encapsulates
 *  the OS semaphore and allows this to be plugged in with the RESMGR OSAL
 *  functions.
 */
typedef struct Resmgr_Sem
{
    /**
     * @brief  Pointer to the resource manager MCB
     */
    struct Resmgr_MCB*      ptrResmgrMCB;

    /**
     * @brief  Opaque handle to the operating system semaphore.
     */
    void*                   rmSemaphore;
}Resmgr_Sem;

/**
 * @brief
 *  Resource Manager MCB
 *
 * @details
 *  Resource manager master control block.
 */
typedef struct Resmgr_MCB
{
    /**
     * @brief  System configuration.
     */
    Resmgr_SystemCfg            cfg;

    /**
     * @brief  RM client handle
     */
    Rm_Handle                   rmClientHandle;

    /**
     * @brief  RM Client Service handle
     */
    Rm_ServiceHandle*           rmClientServiceHandle;

    /**
     * @brief  RM Client Transport handle
     */
    void*                       rmClientTransportHandle;

    /**
     * @brief  RM Semaphore used for multiple thread protection
     */
    Resmgr_Sem                  semaphore;

#ifdef __ARMv7
    /**
     * @brief  HPLIB Virtual address information block.
     */
    hplib_virtualAddrInfo_T     hplibVirtualAddressInfo;
#endif

    /**
     * @brief  Allocated memory region
     */
    Resmgr_MemoryRegionBlock    allocatedMemoryRegion[QMSS_MAX_MEM_REGIONS];

    /**
     * @brief  Hardware semaphores which are allocated by the specific module
     */
    uint8_t                     allocatedHwSemapore[RESMGR_MAX_HW_SEM];

    /**
     * @brief  Accumulated channels which are allocated by the specific module
     */
    uint8_t                     allocatedAccumulatedChannel[RESMGR_MAX_ACCUMULATOR_CHANNEL];

    /**
     * @brief  Host interrupts which are allocated by the specific module. These are
     * valid only in the DSP realm.
     */
    uint8_t                     allocatedHostInterrupt[RESMGR_MAX_HOST_INTERRUPT];

    /**
     * @brief  Allocated Queue Pend queues.
     */
    uint32_t                    allocatedQueuePendQueue[RESMGR_MAX_QUEUE_PEND];
}Resmgr_MCB;

/**********************************************************************
 ************************ Extern Functions ****************************
 **********************************************************************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* Client Transport: */
extern Rm_TransportHandle Resmgr_setupClientTransport(Resmgr_SystemCfg* ptrSystemCfg,Rm_Handle rmClientHandle, int32_t* errCode);
extern int32_t Resmgr_deleteClientTransport (void* clientTransportHandle, int32_t* errCode);

/* Device specific exported API: */
extern int32_t Resmgr_mapQueuePendToInterrupt(int32_t cpIntcId, int32_t qMgr, int32_t qNum);
extern int32_t Resmgr_getAccumulatedChannelMapping(uint8_t dtsAccChannel, uint8_t dspCoreId, uint8_t* accChannel,
                                                   Qmss_Queue* queueInfo, uint32_t* eventId, Qmss_PdspId* qmssPdspId);

/**********************************************************************
 ******************** Resource Management Functions *******************
 **********************************************************************/

#ifdef __ARMv7

/* Global Variable which points to shared HPLIB Semaphore which is placed in shared
 * memory and is used to protect the HPLIB Memory allocation across multiple processes
 * The HPLIB OSAL Functions API do not allow passing an argument to the CS Enter so
 * we need to define a global variable to be able to solve the issue. */
static sem_t*   gPtrHPLIBSemaphore;

/**
 *  @b Description
 *  @n
 *      The function is used to create an HPLIB Critical section. This can be used
 *      to ensure that the HPLIB data structure are protected against concurrent
 *      access across multiple threads.
 *
 *  @retval
 *      Success - Opaque critical section handle
 *  @retval
 *      Error   - NULL
 */
static void* Osal_hplibCSCreate (void)
{
    char        hplibFile[PATH_MAX];
    int32_t     hplibFileDescriptor;
    int32_t     initMemoryFlag;
    uint32_t*   addr;

    /* HPLIB Shared Semaphore name: */
    snprintf (hplibFile, PATH_MAX, "/hplib-sem");

    /* Open the HPLIB Shared memory */
    hplibFileDescriptor = shm_open (hplibFile, O_RDWR, 0777);
    if (hplibFileDescriptor == -1)
    {
        /* Error while opening file; maybe it does not exist so lets
         * try creating one and also set the flag to initialize this. */
        initMemoryFlag = 1;

        /* Create the HPLIB shared memory section: */
        hplibFileDescriptor = shm_open (hplibFile, O_RDWR | O_CREAT, 0777);
        if (hplibFileDescriptor == -1)
        {
            System_printf ("Error: Unable to create the HPLIB Shared Memory section [%s]\n", strerror(errno)); //fzm
            return NULL;
        }
    }
    else
    {
        /* File already opened and created. So we dont need to perform any initializations. */
        initMemoryFlag = 0;
    }

    /* Set the memory object size: We are only sharing a semaphore here */
    if (ftruncate(hplibFileDescriptor, sizeof(sem_t)) == -1)
    {
        System_printf ("Error: ftruncate failed [%s]\n", strerror(errno)); //fzm
        return NULL;
    }

    /* Map the memory object */
    addr = mmap(0, sizeof(sem_t), PROT_READ | PROT_WRITE, MAP_SHARED, hplibFileDescriptor, 0);
    if (addr == MAP_FAILED)
    {
        System_printf ("Error: mmap failed [%s]\n", strerror(errno)); //fzm
        return NULL;
    }

    /* Remember the HPLIB Semaphore: */
    gPtrHPLIBSemaphore = (sem_t*)addr;

    /* Do we need to initialize the semaphore? */
    if (initMemoryFlag == 1)
    {
        /* YES: Initialize the semaphore: */
        if (sem_init(gPtrHPLIBSemaphore, 1, 1) < 0)
        {
            System_printf ("Error: HPLIB Semaphore Initialization Failed [%s]\n", strerror(errno)); //fzm
            return NULL;
        }
        System_printf ("Debug: HPLIB initialized memory semaphore %p\n", gPtrHPLIBSemaphore);
    }
    else
    {
        /* NO: Semaphore has already been initialized */
        System_printf ("Debug: HPLIB shared memory semaphore %p\n", gPtrHPLIBSemaphore);
    }
    return (void*)addr;
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
    sem_wait (gPtrHPLIBSemaphore);
    return (void*)gPtrHPLIBSemaphore;
}

/**
 *  @b Description
 *  @n
 *      HPLIB Critical section enter function
 *
 *  @param[in]  ptrCSHandle
 *      Pointer to the critical section handle
 *
 *  @retval
 *      Not applicable
 */
void Osal_hplibCsExit (void* ptrCSHandle)
{
    sem_post (gPtrHPLIBSemaphore);
}

/**
 *  @b Description
 *  @n
 *      The function is used to convert the physical address to a virtual address.
 *      This is used only in the ARM context.
 *
 *  @param[in]  virtBaseAddr
 *      Virtual base address mapped using mmap
 *  @param[in]  phyBaseAddr
 *      Physical base address
 *  @param[in]  phyOffset
 *      Physical address offset
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Virtual address.
 */
static inline void* Resmgr_phyToVirt(void* virtBaseAddr, uint32_t  phyBaseAddr, uint32_t  phyOffset)
{
    return((void *)((uint8_t *)virtBaseAddr + (phyOffset - phyBaseAddr)));
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the virtual address associated with the PASS subsystem.
 *
 *  @param[in]  hndSysConfig
 *      Handle to the system configuration.
 *
 *  \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Virtual address.
 */
uint32_t Resmgr_getPASSVirtualAddress(Resmgr_SysCfgHandle hndSysConfig)
{
    Resmgr_MCB* ptrResmgrMCB;

    /* Get the handle to the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)hndSysConfig;
    if (ptrResmgrMCB == NULL)
        return 0;

    /* Get the PASS virtual address.*/
    return (uint32_t)ptrResmgrMCB->hplibVirtualAddressInfo.passCfgVaddr;
}

/**
 *  @b Description
 *  @n
 *      Utility function which is used to map a physical address to a virtual
 *      address.
 *
 *  @param[in]  phyAddress
 *      Physical address
 *  @param[in]  size
 *      Size
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - Virtual address
 *  @retval
 *      Error     - 0
 */
static uint32_t Resmgr_map (uint32_t* phyAddress, int32_t size)
{
    long        retval;
    uint32_t    pageSize;
    uint32_t    virtualAddress;

    /* Get the page size: */
    retval = sysconf(_SC_PAGE_SIZE);
    if (retval == -1)
    {
        System_printf ("Error: Failed to get page size err=%s\n", strerror(errno));
        return 0;
    }
    pageSize = (uint32_t)retval;

    /* Ensure that the size if page size aligned */
    if ((size % pageSize) != 0)
        size = ((size/pageSize) + 1) * pageSize;

    /* Ensure that the address is aligned on the page size boundary: If not align it */
    if ((*phyAddress % pageSize) != 0)
    {
        /* Not aligned on the page boundary. Move back the physical address and increase the size by the additional page size */
        *phyAddress = ((*phyAddress / pageSize) ) * pageSize;
        size = size + pageSize;
    }

    /* Map the CPPI Global configuration physical address: */
    virtualAddress = (uint32_t)hplib_VM_MemMap ((void*)(*phyAddress), size);
    return virtualAddress;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to get the RM service handle associated
 *      with the SYSLIB Resource manager instance handle.
 *
 *  @param[in]  hndSysConfig
 *      Handle to the system configuration.
 *
 *  \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Associated RM service handle.
 */
Rm_ServiceHandle* Resmgr_getRMServiceHandle(Resmgr_SysCfgHandle hndSysConfig)
{
    Resmgr_MCB* ptrResmgrMCB;

    /* Get the handle to the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)hndSysConfig;
    if (ptrResmgrMCB == NULL)
        return 0;

    /* Return the associated client service handle. */
    return ptrResmgrMCB->rmClientServiceHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the CPPI and QMSS modules.
 *
 *  @param[in]  ptrResmgrMCB
 *      Pointer to the resource manager MCB
 *  @param[in]  totalDescriptors
 *      Total number of descriptors
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_initSystem(Resmgr_MCB* ptrResmgrMCB, uint32_t totalDescriptors)
{
    Qmss_InitCfg                qmssInitConfig;
    Qmss_Result                 result;
    Qmss_GlobalConfigParams     virtualQmssGblCfgParams;
    Cppi_GlobalConfigParams     virtualCppiGblCfgParams;

    /* Initialize the QMSS Configuration block. */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Populate the QMSS Configuration block:
     * This is not really required because in Hawking the QMSS linking RAM
     * has been configured by ARM. The configuration here is populated because
     * we need to initialize the QMSS driver and its globals variables.
     * NOTE: The external linking RAM address here is bogus and will not be used
     * This is just added here to ensure that the QMSS initializations go through */
    qmssInitConfig.linkingRAM0Base  = 0;
    qmssInitConfig.linkingRAM0Size  = 0;
    qmssInitConfig.linkingRAM1Base  = 0xA0000000;
    qmssInitConfig.maxDescNum       = totalDescriptors;
    qmssInitConfig.qmssHwStatus     = QMSS_HW_INIT_COMPLETE;

#ifdef __ARMv7
    {
        uint32_t    count;

        /* In the ARM; we need to convert the physical address to a virtual address */
        /* Convert the QMSS Global CFG paramters from the physical address to the global address. */
        virtualQmssGblCfgParams = qmssGblCfgParams;
        for(count=0;count < qmssGblCfgParams.maxQueMgrGroups;count++)
        {
            virtualQmssGblCfgParams.groupRegs[count].qmConfigReg =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmConfigReg);
            virtualQmssGblCfgParams.groupRegs[count].qmDescReg =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmDescReg);
            virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtReg =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtReg);
            virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtProxyReg =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtProxyReg);
            virtualQmssGblCfgParams.groupRegs[count].qmQueStatReg =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmQueStatReg);
            virtualQmssGblCfgParams.groupRegs[count].qmStatusRAM =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmStatusRAM);
            virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtDataReg = Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssDataVaddr,
                                CSL_QMSS_DATA_BASE, (uint32_t)virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtDataReg);
            virtualQmssGblCfgParams.groupRegs[count].qmQueMgmtProxyDataReg = NULL; /* not supported on k2 hardware, and not used by lld */
        }

        for(count=0;count < QMSS_MAX_INTD;count++)
        {
            virtualQmssGblCfgParams.regs.qmQueIntdReg[count] = Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr,
                                        CSL_QMSS_CFG_BASE, (uint32_t)virtualQmssGblCfgParams.regs.qmQueIntdReg[count]);
        }

        for(count=0;count < QMSS_MAX_PDSP;count++)
        {
            virtualQmssGblCfgParams.regs.qmPdspCmdReg[count] =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.regs.qmPdspCmdReg[count]);
            virtualQmssGblCfgParams.regs.qmPdspCtrlReg[count] =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.regs.qmPdspCtrlReg[count]);
            virtualQmssGblCfgParams.regs.qmPdspIRamReg[count] =
                Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                                (uint32_t)virtualQmssGblCfgParams.regs.qmPdspIRamReg[count]);
        }

        virtualQmssGblCfgParams.regs.qmLinkingRAMReg =
            Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                            (uint32_t)virtualQmssGblCfgParams.regs.qmLinkingRAMReg);
        virtualQmssGblCfgParams.regs.qmBaseAddr =
            Resmgr_phyToVirt(ptrResmgrMCB->hplibVirtualAddressInfo.qmssCfgVaddr, CSL_QMSS_CFG_BASE,
                            (uint32_t)virtualQmssGblCfgParams.regs.qmBaseAddr);
    }
#else
    /* In the DSP; the virtual configuration is the same as the actual configuration;
     * so we simply copy the physical configuration. */
    memcpy ((void *)&virtualQmssGblCfgParams, (void *)&qmssGblCfgParams, sizeof(Qmss_GlobalConfigParams));
#endif
    /* We are not downloading the QMSS Firmware here. This needs to be handled properly here.
     * PDSP7 Firmware has already been downloaded by the kernel. PDSP3 firmware needs to be downloaded
     * now. But we dont want every application which invokes the API to download the firmware. This
     * will kill the PDSP3 if it is being used already. Besides the Qmss_init is not handling the
     * downloads and status reporting correctly. PDSP3 is being downloaded by the SOC Init application. */
    virtualQmssGblCfgParams.maxPDSP = 0;

    /* Get the RM Handle and pass it. */
    virtualQmssGblCfgParams.qmRmServiceHandle = ptrResmgrMCB->rmClientServiceHandle;

    /* Initialize Queue Manager Sub System: The QMSS hardware & linking RAM has already been setup
     * by the ARM. We simply use this API to initialize the QMSS driver internal data structures. */
    result = Qmss_init (&qmssInitConfig, &virtualQmssGblCfgParams);
    if (result != QMSS_SOK)
        return -1;

#ifdef __ARMv7
    {
        int32_t         index;
        uint32_t        phyAddress;
        void*           ptrVirtualAddress;
        uint32_t        size;

        /* In the ARM we need to convert the CPPI Global CFG paramters from the physical address to virtual address */
        virtualCppiGblCfgParams = cppiGblCfgParams;

        /* Cycle through all the CPDMA Blocks which have been registered in the System */
        for (index = 0; index < CPPI_MAX_CPDMA; index++)
        {
            /* Does the CPDMA specify a global configuration space? */
            if (virtualCppiGblCfgParams.cpDmaCfgs[index].gblCfgRegs == NULL)
                continue;

            /* Determine the size of the memory which needs to be mapped: */
            size = sizeof(CSL_Cppidma_global_configRegs)     + sizeof(CSL_Cppidma_tx_channel_configRegs)   +
                   sizeof(CSL_Cppidma_rx_channel_configRegs) + sizeof(CSL_Cppidma_tx_scheduler_configRegs) +
                   sizeof(CSL_Cppidma_rx_flow_configRegs);

            /* Map the physical address: */
            phyAddress = (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].gblCfgRegs;
            ptrVirtualAddress = (void*)Resmgr_map (&phyAddress, size);
            if (ptrVirtualAddress == NULL)
            {
                System_printf("Error: Unable to map the CPPI address space for DMA '%d' [Address: %p]\n", index, (void*)phyAddress);
                return -1;
            }

            /* Get the virtual address: */
            virtualCppiGblCfgParams.cpDmaCfgs[index].gblCfgRegs = Resmgr_phyToVirt (ptrVirtualAddress, phyAddress,
                                                                                    (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].gblCfgRegs);
            virtualCppiGblCfgParams.cpDmaCfgs[index].txChRegs   = Resmgr_phyToVirt (ptrVirtualAddress, phyAddress,
                                                                                    (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].txChRegs);
            virtualCppiGblCfgParams.cpDmaCfgs[index].rxChRegs   = Resmgr_phyToVirt (ptrVirtualAddress, phyAddress,
                                                                                    (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].rxChRegs);
            virtualCppiGblCfgParams.cpDmaCfgs[index].txSchedRegs= Resmgr_phyToVirt (ptrVirtualAddress, phyAddress,
                                                                                    (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].txSchedRegs);
            virtualCppiGblCfgParams.cpDmaCfgs[index].rxFlowRegs = Resmgr_phyToVirt (ptrVirtualAddress, phyAddress,
                                                                                    (uint32_t)virtualCppiGblCfgParams.cpDmaCfgs[index].rxFlowRegs);
        }
    }
#else
    /* In the DSP; the virtual configuration is the same as the actual configuration;
     * so we simply copy the physical configuration. */
    memcpy ((void *)&virtualCppiGblCfgParams, (void *)&cppiGblCfgParams, sizeof(Cppi_GlobalConfigParams));
#endif

    /* Initialize CPPI CPDMA */
    if (Cppi_init (&virtualCppiGblCfgParams) != CPPI_SOK)
        return -1;

    /* CPPI and QMSS has been initialized and is operational. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal function which allocates the hardware semaphores & populates the
 *      allocated resources in the configuration block.
 *
 *  @param[in]  ptrResmgrMCB
 *      Pointer to the resource manager MCB
 *  @param[in]  ptrResCfg
 *      Pointer to the requested resources
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_allocateHwSemaphores
(
    Resmgr_MCB*         ptrResmgrMCB,
    Resmgr_ResourceCfg* ptrResCfg,
    int32_t*            errCode
)
{
    int32_t             hwSemIndex;
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;

    /* Cycle through and satisfy all the requested hardware semaphores. */
    for (hwSemIndex = 0; hwSemIndex < ptrResCfg->numHwSemaphores; hwSemIndex++)
    {
        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = "hw-semaphores";
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Debug Message: */
            System_printf ("Debug: Hardware semaphore %d has been allocated\n", rmServiceResp.resourceBase);

            /* Populate the allocated hardware semaphore in the resource response section. */
            ptrResCfg->hwSemResponse[hwSemIndex] = rmServiceResp.resourceBase;

            /* Sanity Check: This should never happen but we want to ensure that the limits are
             * not exceeded*/
            if (rmServiceResp.resourceBase > RESMGR_MAX_HW_SEM)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Track the allocated hardware semaphore in the SYSRM module also. */
            ptrResmgrMCB->allocatedHwSemapore[rmServiceResp.resourceBase] = 1;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Hardware semaphore response service state %d\n", rmServiceResp.serviceState);
            *errCode = RESMGR_EHW_SEM_LIMIT;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal function which allocates the High Priority Accumulator channels
 *      & populates the allocated resources in the configuration block.
 *
 *  @param[in]  ptrResmgrMCB
 *      Pointer to the resource manager MCB
 *  @param[in]  ptrResCfg
 *      Pointer to the requested resources
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_allocateHiPrioAccumChannels
(
    Resmgr_MCB*         ptrResmgrMCB,
    Resmgr_ResourceCfg* ptrResCfg,
    int32_t*            errCode
)
{
#ifdef __ARMv7
    int32_t             accChanIndex;
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    char                resourceName[RM_NAME_MAX_CHARS];

    /* Cycle through all and allocate accumulator channels for the specific configuration. */
    for (accChanIndex = 0; accChanIndex < ptrResCfg->numAccumalatorChannels; accChanIndex++)
    {
        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Construct the resource name. */
        strncpy(resourceName, "hi_accumulator_channel_arm", RM_NAME_MAX_CHARS);

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Get the accumulated channel mapping and use it to populate the resource response section. */
            if (Resmgr_getAccumulatedChannelMapping(rmServiceResp.resourceBase, ptrResmgrMCB->cfg.coreId,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].accChannel,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].queue,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].eventId,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].pdspId) < 0)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Debug Message: */
            System_printf ("Debug: Accumulator Channel [%s] %d Core %d Queue %d:%d EventId:%d PDSPID:%d\n",
                            resourceName, ptrResCfg->accChannelResponse[accChanIndex].accChannel, ptrResmgrMCB->cfg.coreId,
                            ptrResCfg->accChannelResponse[accChanIndex].queue.qMgr, ptrResCfg->accChannelResponse[accChanIndex].queue.qNum,
                            ptrResCfg->accChannelResponse[accChanIndex].eventId, ptrResCfg->accChannelResponse[accChanIndex].pdspId);

            /* Sanity Check: This should never happen but we want to ensure that the limits are
             * not exceeded */
            if (rmServiceResp.resourceBase > RESMGR_MAX_ACCUMULATOR_CHANNEL)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Track the allocated accumulated channel in the SYSRM module also. */
            ptrResmgrMCB->allocatedAccumulatedChannel[rmServiceResp.resourceBase] = 1;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Accumulated channel state %d\n", rmServiceResp.serviceState);
            *errCode = RESMGR_EACC_CHANNEL_LIMIT;
            return -1;
        }
    }

    /* Accumulator channels have been successfully allocated. */
    return 0;
#else
    int32_t             accChanIndex;
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    char                resourceName[RM_NAME_MAX_CHARS];

    /* Cycle through all and allocate accumulator channels for the specific configuration. */
    for (accChanIndex = 0; accChanIndex < ptrResCfg->numAccumalatorChannels; accChanIndex++)
    {
        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Construct the resource name. */
        snprintf (resourceName, RM_NAME_MAX_CHARS, "hi_accumulator_channel_%d", ptrResmgrMCB->cfg.coreId);

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Get the accumulated channel mapping and use it to populate the resource response section. */
            if (Resmgr_getAccumulatedChannelMapping(rmServiceResp.resourceBase, ptrResmgrMCB->cfg.coreId,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].accChannel,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].queue,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].eventId,
                                                    &ptrResCfg->accChannelResponse[accChanIndex].pdspId) < 0)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Debug Message: */
            System_printf ("Debug: Accumulator Channel [%s] %d Core %d Queue %d:%d EventId:%d PDSPID:%d\n",
                           resourceName, ptrResCfg->accChannelResponse[accChanIndex].accChannel, ptrResmgrMCB->cfg.coreId,
                           ptrResCfg->accChannelResponse[accChanIndex].queue.qMgr, ptrResCfg->accChannelResponse[accChanIndex].queue.qNum,
                           ptrResCfg->accChannelResponse[accChanIndex].eventId, ptrResCfg->accChannelResponse[accChanIndex].pdspId);

            /* Sanity Check: This should never happen but we want to ensure that the limits are
             * not exceeded */
            if (rmServiceResp.resourceBase > RESMGR_MAX_ACCUMULATOR_CHANNEL)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Track the allocated accumulated channel in the SYSRM module also. */
            ptrResmgrMCB->allocatedAccumulatedChannel[rmServiceResp.resourceBase] = 1;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Accumulated channel state %d\n", rmServiceResp.serviceState);
            *errCode = RESMGR_EACC_CHANNEL_LIMIT;
            return -1;
        }
    }

    /* Accumulator channels have been successfully allocated. */
    return 0;
#endif
}

/**
 *  @b Description
 *  @n
 *      Internal function which allocates the host interrupts. On the DSP the system
 *      interrupts are mapped to host interrupts which are then wired to the DSP INTC
 *      event identifiers. This is not the case on the ARM where the interrupts are
 *      wired via the GIC controller.
 *
 *  @param[in]  ptrResmgrMCB
 *      Pointer to the resource manager MCB
 *  @param[in]  ptrResCfg
 *      Pointer to the requested resources
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_allocateCPINTCInterrupts
(
    Resmgr_MCB*         ptrResmgrMCB,
    Resmgr_ResourceCfg* ptrResCfg,
    int32_t*            errCode
)
{
    int32_t                     cpIntcIndex;
    int32_t                     index;
    Qmss_QueueHnd               queueHandle;
    Resmgr_QpendQueueResponse*  ptrQueuePendResponse;
    Rm_ServiceReqInfo           rmServiceReq;
    Rm_ServiceRespInfo          rmServiceResp;
    const char*                 qpendResourceName;
    char                        resourceName[RM_NAME_MAX_CHARS];

#ifdef __ARMv7
    /* ARM: There is no support for CPINTC interrupts. This is an invalid configuration since on
     * ARM the interrupts are via the GIC controller. */
    if (ptrResCfg->numCPINTCInterrupts > 0)
    {
        System_printf ("Error: On ARM interrupts are wired via the GIC. Incorrect configuration\n");
        *errCode = RESMGR_EINVAL;
        return -1;
    }
    /* Initialize the QPEND resource name correctly for the ARM domain */
    qpendResourceName = "direct_interrupt_arm";
#else
    /* Initialize the QPEND resource name correctly for the DSP domain */
    qpendResourceName = "direct_interrupt_dsp";
#endif

    /* Cycle through all and allocate all the CPINTC host interrupts which had been requested  */
    for (cpIntcIndex = 0; cpIntcIndex < ptrResCfg->numCPINTCInterrupts; cpIntcIndex++)
    {
        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Construct the resource name. */
        snprintf (resourceName, RM_NAME_MAX_CHARS, "cic_output_%d", ptrResmgrMCB->cfg.coreId);

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Populate the allocated host interrupt in the resource response section. */
            ptrResCfg->cpIntcHostIntrResponse[cpIntcIndex] = rmServiceResp.resourceBase;

            /* Debug Message: */
            System_printf ("Debug: Allocated Host interrupt %d\n", ptrResCfg->cpIntcHostIntrResponse[cpIntcIndex]);

            /* Sanity Check: This should never happen but we want to ensure that the limits are
             * not exceeded */
            if (rmServiceResp.resourceBase > RESMGR_MAX_HOST_INTERRUPT)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Track the allocated accumulated host interrupt in the SYSRM module also. */
            ptrResmgrMCB->allocatedHostInterrupt[rmServiceResp.resourceBase] = 1;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Host Interrupt state %d\n", rmServiceResp.serviceState);
            return -1;
        }
    }

    /* Cycle through all and allocate all the queue pends which had been requested  */
    for (cpIntcIndex = 0; cpIntcIndex < ptrResCfg->numQpendQueues; cpIntcIndex++)
    {
        /******************************************************************************
         * STEP1: Request the RM Server for a direct interrupt queue.
         ******************************************************************************/

        /* Get the pointer to the queue pend response which is to be populated. */
        ptrQueuePendResponse = &ptrResCfg->qPendResponse[cpIntcIndex];

#ifdef __ARMv7
        /* On ARM there queue pend interrupts are NOT routed via CPINTC. */
        ptrQueuePendResponse->cpIntcId = -1;
#else
        /* The specification states that:-
         *  - All interrupts for COREPac[0-3] are routed via CIC0
         *  - All interrupts for COREPac[4-7] are routed via CIC1 */
        if (ptrResmgrMCB->cfg.coreId < 4)
            ptrQueuePendResponse->cpIntcId = 0;
        else
            ptrQueuePendResponse->cpIntcId = 1;
#endif

        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = qpendResourceName;
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* The resource manager returns a QID; which we convert to a queue handle */
            queueHandle = Qmss_getHandleFromQID(rmServiceResp.resourceBase);

            /* Populate the allocated host interrupt in the resource response section. */
            ptrQueuePendResponse->queue     = Qmss_getQueueNumber (queueHandle);

            /* Use the mapping function to get the corresponsing system interrupt */
            ptrQueuePendResponse->systemInterrupt = Resmgr_mapQueuePendToInterrupt (ptrQueuePendResponse->cpIntcId,
                                                                                    ptrQueuePendResponse->queue.qMgr,
                                                                                    ptrQueuePendResponse->queue.qNum);

            /* Debug Message: */
            System_printf ("Debug: Allocated CPINTC Id %d Direct Interrupt Queue %d System Interrupt %d\n",
                            ptrQueuePendResponse->cpIntcId, queueHandle, ptrQueuePendResponse->systemInterrupt);

            /* Track the allocated direct interrupt queue in the SYSRM module also. */
            for (index = 0; index < RESMGR_MAX_QUEUE_PEND; index++)
            {
                if (ptrResmgrMCB->allocatedQueuePendQueue[index] == 0)
                {
                    ptrResmgrMCB->allocatedQueuePendQueue[index] = queueHandle;
                    break;
                }
            }

            /* Sanity Check: Ensure that the internal RM tracking was successful */
            if (index == RESMGR_MAX_QUEUE_PEND)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Host Interrupt state %d\n", rmServiceResp.serviceState);
            return -1;
        }

        /******************************************************************************
         * STEP2: Request the RM Server for a host interrupt mapping which can be used
         * to map the queue pend system interrupt to a host interrupt.
         ******************************************************************************/
#ifdef __ARMv7
        /* In the case of ARM; there is no need to perform another mapping since the
         * queue pend interrupt lines are wired via the GIC. So here we simply set the
         * event identifier to be the same as the system interrupt. */
        ptrQueuePendResponse->hostInterrupt = ptrQueuePendResponse->systemInterrupt;

        /* In ARM; the CPINTC identifier and system interrupt fields are NOT used. */
        ptrQueuePendResponse->cpIntcId          = -1;
        ptrQueuePendResponse->systemInterrupt   = -1;
#else
        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Construct the resource name. */
        snprintf (resourceName, RM_NAME_MAX_CHARS, "cic_output_%d", ptrResmgrMCB->cfg.coreId);

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Populate the allocated host interrupt in the resource response section. */
            ptrQueuePendResponse->hostInterrupt   = rmServiceResp.resourceBase;

            /* Debug Message: */
            System_printf ("Debug: Allocated Host interrupt %d CIC %d\n", ptrQueuePendResponse->hostInterrupt, ptrQueuePendResponse->cpIntcId);

            /* Sanity Check: This should never happen but we want to ensure that the limits are
             * not exceeded */
            if (rmServiceResp.resourceBase > RESMGR_MAX_HOST_INTERRUPT)
            {
                *errCode = RESMGR_EINTERNAL;
                return -1;
            }

            /* Track the allocated accumulated host interrupt in the SYSRM module also. */
            ptrResmgrMCB->allocatedHostInterrupt[rmServiceResp.resourceBase] = 1;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Host Interrupt state %d\n", rmServiceResp.serviceState);
            return -1;
        }
#endif
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate and insert the requested memory regions.
 *
 *  @param[in]  ptrResmgrMCB
 *      Pointer to the resource manager block
 *  @param[in]  ptrResCfg
 *      Pointer to the requested resource configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
static int32_t Resmgr_allocMemoryRegions
(
    Resmgr_MCB*         ptrResmgrMCB,
    Resmgr_ResourceCfg* ptrResCfg,
    int32_t*            errCode
)
{
    int32_t                     result;
    Qmss_MemRegInfo             memRegInfo;
    int32_t                     index;
    Resmgr_MemRegionCfg*        ptrMemoryRegionRequest;

    /* Cycle through all the memory region resource requests. */
    for (index = 0; index < QMSS_MAX_MEM_REGIONS; index++)
    {
        /* Get the memory region request */
        ptrMemoryRegionRequest = &ptrResCfg->memRegionCfg[index];

        /* Is there a valid request? */
        if (ptrMemoryRegionRequest->type == 0)
            break;

        /* Initialize the memory region configuration. */
        memset ((void *)&memRegInfo, 0, sizeof(Qmss_MemRegInfo));

        /* Populate the memory region configuration:
         *  - We allow the QMSS & RM to determine which memory region is to be used. */
        memRegInfo.descSize         = ptrMemoryRegionRequest->sizeDesc;
        memRegInfo.descNum          = ptrMemoryRegionRequest->numDesc;
        memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
        memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;

        /* Setup the start index depending upon the type of memory region requested: */
        switch (ptrMemoryRegionRequest->linkingRAM)
        {
            case Resmgr_MemRegionLinkingRAM_INTERNAL:
            {
                /* Internal memory region requested: */
                memRegInfo.startIndex = QMSS_START_INDEX_INTERNAL;
                break;
            }
            case Resmgr_MemRegionLinkingRAM_DONT_CARE:
            {
                /* Memory region requested is a dont care: Since internal memory regions are a very precious
                 * resource and we dont want to take it for somebody who does not care. Move this into the
                 * external linking RAM. */
                memRegInfo.startIndex = QMSS_START_INDEX_EXTERNAL;
                break;
            }
        }

        /* Allocate memory for the memory region: */
        memRegInfo.descBase = (uint32_t*)ptrResmgrMCB->cfg.mallocMemoryRegion(&ptrMemoryRegionRequest->name[0],
                                                                              ptrMemoryRegionRequest->type,
                                                                              memRegInfo.descSize*memRegInfo.descNum);
        if (memRegInfo.descBase == NULL)
        {
            *errCode = RESMGR_ENOMEM;
            return -1;
        }

        /* Initialize the allocated memory region: */
        memset((void *)memRegInfo.descBase, 0, memRegInfo.descSize*memRegInfo.descNum);

        /* Writeback the cache contents. */
        ptrResmgrMCB->cfg.endMemAccess(memRegInfo.descBase, memRegInfo.descSize*memRegInfo.descNum);

        /* Initialize and insert the memory region. */
        result = Qmss_insertMemoryRegion (&memRegInfo);
        if (result < QMSS_SOK)
        {
            /* QMSS insertion failed; propogate the error code. */
            *errCode = result;
            System_printf ("Error: Inserting memory region (%s) failed: %d\n", ptrMemoryRegionRequest->name, result);
            return -1;
        }

        /* Debug Message: */
        System_printf ("Debug: Inserting Memory Region (%s) @ Index %d Number of Descriptors %d\n",
                        ptrMemoryRegionRequest->name, result, memRegInfo.descNum);

        /* Populate the memory region response section:
         * - TODO We need to get the linking RAM where the memory region has been placed from the QMSS */
        ptrResCfg->memRegionResponse[index].linkingRAM      = Resmgr_MemRegionLinkingRAM_INTERNAL;
        ptrResCfg->memRegionResponse[index].memRegionHandle = (Qmss_MemRegion)result;
        memcpy ((void *)&ptrResCfg->memRegionResponse[index].memRegionCfg, (void *)ptrMemoryRegionRequest, sizeof(Resmgr_MemRegionCfg));

        /* Sanity Check: This should never happen but we want to ensure that the limits are not exceeded */
        if (result >= QMSS_MAX_MEM_REGIONS)
        {
            *errCode = RESMGR_EINTERNAL;
            return -1;
        }

        /* Keep track of the allocated memory regions. */
        ptrResmgrMCB->allocatedMemoryRegion[result].memoryRegionBaseAddress = (void*)memRegInfo.descBase;
        ptrResmgrMCB->allocatedMemoryRegion[result].sizeMemoryRegion        = memRegInfo.descSize*memRegInfo.descNum;
        memcpy ((void *)&ptrResmgrMCB->allocatedMemoryRegion[result].cfg, (void *)ptrMemoryRegionRequest, sizeof(Resmgr_MemRegionCfg));
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate "custom" resources from the RM Server. It is
 *      possible to allocate upto 'N' number of resources in a single call. The RMv2
 *      infrastructure supports an allocation of 'N' contiguous resources. The function
 *      will fail if 'N' contiguous resources are not found.
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  customName
 *      Custom resource name
 *  @param[in]  numResource
 *      Number of resources to allocate
 *  @param[out] value
 *      Allocate resource value or the 'base' if N resources were requested
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - Handle to the resource manager module.
 *  @retval
 *      Error     - NULL
 */
int32_t Resmgr_allocCustomResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         customName,
    uint32_t            numResource,
    uint32_t*           value,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (customName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
    rmServiceReq.resourceName             = customName;
    rmServiceReq.resourceBase             = RM_RESOURCE_BASE_UNSPECIFIED;
    rmServiceReq.resourceLength           = numResource;
    rmServiceReq.resourceAlignment        = 0;
    rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        /* Populate the allocated resource name. */
        *value = rmServiceResp.resourceBase;
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}
//fzm-->
int32_t Resmgr_allocSpecificCustomResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         customName,
    uint32_t*           value,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
	Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (customName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
	memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
	memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
	rmServiceReq.type                     = Rm_service_RESOURCE_ALLOCATE_INIT;
	rmServiceReq.resourceName             = customName;
	rmServiceReq.resourceBase             = *value;
	rmServiceReq.resourceLength           = 1;
	rmServiceReq.resourceAlignment        = 0;
	rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
	ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
	if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
	    (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        /* Populate the allocated resource name. */
        *value = rmServiceResp.resourceBase;
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}
//fzm<--



static int32_t Resmgr_nameServiceInternal
(
    Resmgr_SysCfgHandle  sysRMHandle,
    Rm_ServiceType const type,
    char const * const   customName,
    uint32_t* const      value
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (customName == NULL) || (value == NULL)) {
        return RESMGR_EINVAL;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type           = type;
    rmServiceReq.resourceName   = customName;
    rmServiceReq.resourceNsName = customName;
    rmServiceReq.resourceLength = 1;
    rmServiceReq.resourceBase   = *value;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle,
                                                           &rmServiceReq,
                                                           &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        if (type == Rm_service_RESOURCE_GET_BY_NAME) {
            *value = rmServiceResp.resourceBase;
        }
        return 0;
    }

    /* The request was denied. */
    return rmServiceResp.serviceState;
}

int32_t Resmgr_nameServiceSet
(
    Resmgr_SysCfgHandle sysRMHandle,
    char const * const  customName,
    uint32_t const      value,
    int32_t* const      errCode
)
{
    uint32_t tmp = value;
    if (!errCode) return RESMGR_EINVAL;
    *errCode = Resmgr_nameServiceInternal(sysRMHandle, Rm_service_RESOURCE_MAP_TO_NAME, customName, &tmp);
    return *errCode;
}

int32_t Resmgr_nameServiceGet
(
    Resmgr_SysCfgHandle sysRMHandle,
    char const * const  customName,
    uint32_t* const     value,
    int32_t* const      errCode
)
{
    if (!errCode) return RESMGR_EINVAL;
    *errCode = Resmgr_nameServiceInternal(sysRMHandle, Rm_service_RESOURCE_GET_BY_NAME, customName, value);
    return *errCode;
}

int32_t Resmgr_nameServiceDel
(
    Resmgr_SysCfgHandle sysRMHandle,
    char const * const  customName,
    int32_t* const      errCode
)
{
    if (!errCode) return RESMGR_EINVAL;
    *errCode = Resmgr_nameServiceInternal(sysRMHandle, Rm_service_RESOURCE_UNMAP_NAME, customName, NULL);
    return *errCode;
}



/**
 *  @b Description
 *  @n
 *      The function is used to free "custom" resources back to the RM Server. It
 *      is possible to free N resources; in which value the value should be the
 *      base of N contiguous resources to be freed up
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  customName
 *      Custom resource name
 *  @param[in]  numResource
 *      Number of resources to allocate
 *  @param[in] value
 *      Resource value to be cleaned up
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - Handle to the resource manager module.
 *  @retval
 *      Error     - NULL
 */
int32_t Resmgr_freeCustomResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         customName,
    uint32_t            numResource,
    uint32_t            value,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (customName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type                     = Rm_service_RESOURCE_FREE;
    rmServiceReq.resourceName             = customName;
    rmServiceReq.resourceBase             = value;
    rmServiceReq.resourceLength           = numResource;
    rmServiceReq.resourceAlignment        = 0;
    rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to map a resource name to an opaque handle. The mapping is
 *      visible across the entire system
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  resourceName
 *      Resource name
 *  @param[in]  handle
 *      Opaque handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_mapResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         resourceName,
    uint32_t            handle,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (resourceName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type                     = Rm_service_RESOURCE_MAP_TO_NAME;
    rmServiceReq.resourceName             = resourceName;
    rmServiceReq.resourceBase             = handle;
    rmServiceReq.resourceLength           = 1;
    rmServiceReq.resourceAlignment        = 0;
    rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to which the resource name has been mapped
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  resourceName
 *      Resource name
 *  @param[in]  handle
 *      Opaque handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_getMappedResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         resourceName,
    uint32_t*           handle,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (resourceName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type                     = Rm_service_RESOURCE_GET_BY_NAME;
    rmServiceReq.resourceName             = resourceName;
    rmServiceReq.resourceBase             = 0;
    rmServiceReq.resourceLength           = 0;
    rmServiceReq.resourceAlignment        = 0;
    rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        *handle = rmServiceResp.resourceBase;
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to unmap a resource name
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  resourceName
 *      Resource name
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_unmapResource
(
    Resmgr_SysCfgHandle sysRMHandle,
    const char*         resourceName,
    int32_t*            errCode
)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    Resmgr_MCB*         ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (resourceName == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Initialize the RM request/response */
    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    /* Populate the RM request data. */
    rmServiceReq.type                     = Rm_service_RESOURCE_UNMAP_NAME;
    rmServiceReq.resourceName             = resourceName;
    rmServiceReq.resourceBase             = 0;
    rmServiceReq.resourceLength           = 1;
    rmServiceReq.resourceAlignment        = 0;
    rmServiceReq.callback.serviceCallback = NULL;

    /* Send the request to the RM server. */
    ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

    /* Handle the RM response. */
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        return 0;
    }

    /* The request was denied. */
    *errCode = rmServiceResp.serviceState;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the resource manager module.
 *
 *  @param[in]  ptrCfg
 *      Pointer to the resource manager configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - Handle to the resource manager module.
 *  @retval
 *      Error     - NULL
 */
Resmgr_SysCfgHandle Resmgr_init
(
    Resmgr_SystemCfg*   ptrCfg,
    int32_t*            errCode
)
{
    Resmgr_MCB*         ptrResmgrMCB;
    Qmss_StartCfg       qmssCfg;
    Cppi_StartCfg       cppiCfg;
    Rm_InitCfg          rmInitCfg;
    int32_t             result;
    uint32_t            totalDescriptors;

    /* Sanity Check: Validate the arguments. */
    if (ptrCfg == NULL)
    {
        *errCode = RESMGR_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the OSAL callout table has been specified. */
    if ((ptrCfg->malloc             == NULL) || (ptrCfg->free               == NULL) ||
        (ptrCfg->mallocMemoryRegion == NULL) || (ptrCfg->freeMemoryRegion   == NULL) ||
        (ptrCfg->createSem          == NULL) || (ptrCfg->deleteSem          == NULL) ||
        (ptrCfg->postSem            == NULL) || (ptrCfg->pendSem            == NULL) ||
        (ptrCfg->beginMemAccess     == NULL) || (ptrCfg->endMemAccess       == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return NULL;
    }

    /* Allocate memory for the resource manager block. */
    ptrResmgrMCB = (Resmgr_MCB*)ptrCfg->malloc (Resmgr_MallocMode_LOCAL, sizeof(Resmgr_MCB));
    if (ptrResmgrMCB == NULL)
    {
        *errCode = RESMGR_ENOMEM;
        return NULL;
    }

    /* Initialize the memory block */
    memset ((void *)ptrResmgrMCB, 0, sizeof(Resmgr_MCB));

    /* Copy the system configuration block. */
    memcpy ((void *)&ptrResmgrMCB->cfg, (void*)ptrCfg, sizeof(Resmgr_SystemCfg));

    /* Initialize the semaphore */
    ptrResmgrMCB->semaphore.ptrResmgrMCB = ptrResmgrMCB;
    ptrResmgrMCB->semaphore.rmSemaphore  = ptrResmgrMCB->cfg.createSem();

#ifdef __ARMv7
    /* For ARM resource manager clients; we need to initialize and allocate the HPLIB component which allows
     * us to allocate a continuous block of memory so that there can be a translation from virtual to physical
     * address. */
    {
        hplib_memPoolAttr_T     memPoolAttributes;

        /* Initialize the memory pool attributes */
        memset ((void *)&memPoolAttributes, 0, sizeof(hplib_memPoolAttr_T));

        /* Setup a single memory block in DDR3 for all memory allocations. */
        memPoolAttributes.attr       = HPLIB_ATTR_KM_CACHED0;
        memPoolAttributes.phys_addr  = 0;
        memPoolAttributes.size       = 0;

        /* Create the shared memory segment. */
        hplib_shmCreate(HPLIB_SHM_SIZE);

        /* Initialize the virtual memory */
        result = hplib_vmInit(&ptrResmgrMCB->hplibVirtualAddressInfo, 1, &memPoolAttributes);
        if (result != hplib_OK)
        {
            System_printf ("Error: HPLIB VM Initialization failed with %d\n", result);
            return NULL;
        }

        /* Initialize the HPLIB OSAL Critical Section: */
        if (Osal_hplibCSCreate () == NULL)
            return NULL;
    }
#endif

    /* Initialize the configuration block. */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));

    /* Populate the RM Client configuration. */
    rmInitCfg.instName = &ptrResmgrMCB->cfg.rmClient[0];
    rmInitCfg.instType = Rm_instType_CLIENT;
    rmInitCfg.mtSemObj = (uint32_t*)&ptrResmgrMCB->semaphore;

    /* Initialize the RM Client */
    ptrResmgrMCB->rmClientHandle = Rm_init(&rmInitCfg, &result);
    if (result != RM_OK)
    {
        *errCode = result;
        return NULL;
    }

    /* Open the RM client service handle */
    ptrResmgrMCB->rmClientServiceHandle = Rm_serviceOpenHandle(ptrResmgrMCB->rmClientHandle, &result);
    if (result != RM_OK)
    {
        *errCode = result;
        return NULL;
    }

    /* Setup the client transport: */
    ptrResmgrMCB->rmClientTransportHandle = Resmgr_setupClientTransport (&ptrResmgrMCB->cfg,
                                                                         ptrResmgrMCB->rmClientHandle, errCode);
    if (ptrResmgrMCB->rmClientTransportHandle == NULL)
        return NULL;

    /* TODO: Initialize the number of descriptors falsely adveritizing this to be 512K.
     * We need to specify some value here for the Qmss_init to be operate correctly.
     * But at this time there is no information on how many descriptors are currently
     * being configured. QMSS has already been initialized by the Linux kernel so there
     * is no side affect of doing this here. */
    totalDescriptors = 512*1024;

    /* CPPI and QMSS Initializations: */
    if (Resmgr_initSystem(ptrResmgrMCB, totalDescriptors) < 0)
    {
        /* Error: Internal CPPI and QMSS failed to initialize and start up. */
        *errCode = RESMGR_EINTERNAL;
        return NULL;
    }

    /* Initialize the configurations */
    memset ((void *)&qmssCfg, 0, sizeof(Qmss_StartCfg));
    memset ((void *)&cppiCfg, 0, sizeof(Cppi_StartCfg));

    /* Start the QMSS */
    qmssCfg.rmServiceHandle = ptrResmgrMCB->rmClientServiceHandle;
    if (Qmss_startCfg(&qmssCfg) != QMSS_SOK)
        return NULL;

    /* Start the CPPI */
    cppiCfg.rmServiceHandle = ptrResmgrMCB->rmClientServiceHandle;
    Cppi_startCfg(&cppiCfg);

    /* Resource Manager is operational. */
    return (Resmgr_SysCfgHandle)ptrResmgrMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to process the requested resource configuration
 *      requests. Application populate the configuration and pass this to the
 *      function which will parse the request configuration and allocate the
 *      requested resources.
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[in]  ptrResCfg
 *      Pointer to the requested resource configuration which is to be processed
 *  @param[out] errCode
 *      Error code populated by the API on error
 *
 * \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_processConfig
(
    Resmgr_SysCfgHandle sysRMHandle,
    Resmgr_ResourceCfg* ptrResCfg,
    int32_t*            errCode
)
{
    Resmgr_MCB* ptrResmgrMCB;

    /* Validate the arguments: */
    if ((sysRMHandle == NULL) || (ptrResCfg == NULL))
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;

    /* Hardware Semaphore Allocations: */
    if (Resmgr_allocateHwSemaphores(ptrResmgrMCB, ptrResCfg, errCode) < 0)
        return -1;

    /* High Priority Accumulator Channel Allocations: */
    if (Resmgr_allocateHiPrioAccumChannels(ptrResmgrMCB, ptrResCfg, errCode) < 0)
        return -1;

    /* CPINTC Interrupt Allocations: */
    if (Resmgr_allocateCPINTCInterrupts(ptrResmgrMCB, ptrResCfg, errCode) < 0)
        return -1;

    /* Memory Region Allocations: */
    if (Resmgr_allocMemoryRegions(ptrResmgrMCB, ptrResCfg, errCode) < 0)
        return -1;

    /* Resource requests have been processed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clean the resource configuration which had been requested by
 *      the system during system initialization. Each core which has initialized or started
 *      the resource manager should use to clean their resources using this
 *
 *  @param[in]  sysRMHandle
 *      Handle to the SYSLIB resource manager
 *  @param[out] errCode
 *      Error code populated by the API
 *
 *  \ingroup RES_MGR_FUN
 *
 *  @retval
 *      Success   - 0
 *  @retval
 *      Error     - <0
 */
int32_t Resmgr_deinit
(
    Resmgr_SysCfgHandle     sysRMHandle,
    int32_t*                errCode
)
{
    Resmgr_MCB*             ptrResmgrMCB;
    uint32_t                index;
    int32_t                 result;
    Rm_ServiceReqInfo       rmServiceReq;
    Rm_ServiceRespInfo      rmServiceResp;
    char                    resourceName[RM_NAME_MAX_CHARS];
#ifdef __ARMv7
    const char*             qpendResourceName = "direct_interrupt_arm";
#else
    const char*             qpendResourceName = "direct_interrupt_dsp";
#endif

    /* Get the resource manager MCB */
    ptrResmgrMCB = (Resmgr_MCB*)sysRMHandle;
    if (ptrResmgrMCB == NULL)
    {
        *errCode = RESMGR_EINVAL;
        return -1;
    }

    /* Initialize the error code. */
    *errCode = 0;

    /* Shutdown the memory regions. */
    for (index = 0; index < QMSS_MAX_MEM_REGIONS; index++)
    {
        /* Is there a valid memory region? */
        if (ptrResmgrMCB->allocatedMemoryRegion[index].memoryRegionBaseAddress == NULL)
            continue;

        /* Debug Message: */
        System_printf ("Debug: Shutting down memory region %d\n", index);

        /* Remove the allocated memory region */
        result = Qmss_removeMemoryRegion (index, 0);
        if (result != QMSS_SOK)
        {
            /* Error: Removing the memory region failed; remember the error code but continue with the removal of
             * other memory regions. */
            *errCode = result;
            System_printf ("Error: Deleting memory region failed %d\n", *errCode);
            return -1;
        }

        /* Cleanup the memory region: */
        ptrResmgrMCB->cfg.freeMemoryRegion(&ptrResmgrMCB->allocatedMemoryRegion[index].cfg.name[0],
                                           ptrResmgrMCB->allocatedMemoryRegion[index].cfg.type,
                                           ptrResmgrMCB->allocatedMemoryRegion[index].memoryRegionBaseAddress,
                                           ptrResmgrMCB->allocatedMemoryRegion[index].sizeMemoryRegion);

        /* Memory region has been cleaned up successfully. */
        ptrResmgrMCB->allocatedMemoryRegion[index].memoryRegionBaseAddress = NULL;
    }

    /* Shutdown all the allocated hardware semaphores */
    for (index = 0; index < RESMGR_MAX_HW_SEM; index++)
    {
        /* Is there a valid hardware semaphore? */
        if (ptrResmgrMCB->allocatedHwSemapore[index] == 0)
            continue;

        /* Debug Message: */
        System_printf ("Debug: Releasing hardware semaphore %d\n", index);

        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_FREE;
        rmServiceReq.resourceName             = "hw-semaphores";
        rmServiceReq.resourceBase             = index;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Track the allocated hardware semaphore in the SYSRM module also. */
            ptrResmgrMCB->allocatedHwSemapore[rmServiceResp.resourceBase] = 0;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Hardware semaphore response service state %d\n", rmServiceResp.serviceState);
        }
    }

    /* Shutdown all the allocated accumulated channels */
    for (index = 0; index < RESMGR_MAX_ACCUMULATOR_CHANNEL; index++)
    {
        /* Is there a valid accumulated channel? */
        if (ptrResmgrMCB->allocatedAccumulatedChannel[index] == 0)
            continue;

        /* Debug Message: */
        System_printf ("Debug: Releasing accumulated channel %d\n", index);

        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

#ifndef __ARMv7
        /* Construct the resource name*/
        snprintf (resourceName, RM_NAME_MAX_CHARS, "hi_accumulator_channel_%d", ptrResmgrMCB->cfg.coreId);
#else
        /* Construct the resource name. */
        strncpy(resourceName, "hi_accumulator_channel_arm", RM_NAME_MAX_CHARS);
#endif

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_FREE;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = index;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Track the allocated hardware semaphore in the SYSRM module also. */
            ptrResmgrMCB->allocatedHwSemapore[rmServiceResp.resourceBase] = 0;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Accumulated channel response service state %d\n", rmServiceResp.serviceState);
        }
    }

    /* Shutdown all the allocated host interrupts */
    for (index = 0; index < RESMGR_MAX_HOST_INTERRUPT; index++)
    {
        /* Is there a valid host interrupt? */
        if (ptrResmgrMCB->allocatedHostInterrupt[index] == 0)
            continue;

        /* Debug Message: */
        System_printf ("Debug: Releasing host interrupt %d\n", index);

        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Construct the resource name*/
        snprintf (resourceName, RM_NAME_MAX_CHARS, "cic_output_%d", ptrResmgrMCB->cfg.coreId);

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_FREE;
        rmServiceReq.resourceName             = &resourceName[0];
        rmServiceReq.resourceBase             = index;
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Track the allocated hardware semaphore in the SYSRM module also. */
            ptrResmgrMCB->allocatedHwSemapore[rmServiceResp.resourceBase] = 0;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Host interrupt response service state %d\n", rmServiceResp.serviceState);
        }
    }

    /* Shutdown all the allocated queue pend queues also. */
    for (index = 0; index < RESMGR_MAX_QUEUE_PEND; index++)
    {
        /* Is there a valid host interrupt? */
        if (ptrResmgrMCB->allocatedQueuePendQueue[index] == 0)
            continue;

        /* Debug Message: */
        System_printf ("Debug: Releasing queue pend %d\n", ptrResmgrMCB->allocatedQueuePendQueue[index]);

        /* Initialize the RM request/response */
        memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
        memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

        /* Populate the RM request data. */
        rmServiceReq.type                     = Rm_service_RESOURCE_FREE;
        rmServiceReq.resourceName             = qpendResourceName;
        rmServiceReq.resourceBase             = ptrResmgrMCB->allocatedQueuePendQueue[index];
        rmServiceReq.resourceLength           = 1;
        rmServiceReq.resourceAlignment        = 0;
        rmServiceReq.callback.serviceCallback = NULL;

        /* Send the request to the RM server. */
        ptrResmgrMCB->rmClientServiceHandle->Rm_serviceHandler(ptrResmgrMCB->rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);

        /* Handle the RM response. */
        if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
            (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
        {
            /* Cleanup the queue pend in the SYSRM module also. */
            ptrResmgrMCB->allocatedQueuePendQueue[index] = 0;
        }
        else
        {
            /* The request was denied. */
            System_printf ("Error: Direct Interrupt Queue service state %d\n", rmServiceResp.serviceState);
        }
    }

    /* Shutdown the QMSS and CPPI LLD */
    *errCode = Qmss_exit ();
    if ((*errCode != QMSS_SOK) && (*errCode != QMSS_RESOURCE_ALLOCATE_USE_DENIED))
    {
        System_printf ("Error: QMSS exit failed %d\n", *errCode);
        return -1;
    }
    *errCode = Cppi_exit ();
    if (*errCode != CPPI_SOK)
    {
        System_printf ("Error: CPPI exit failed %d\n", *errCode);
        return -1;
    }

    /* Close the RM service */
    *errCode = Rm_serviceCloseHandle (ptrResmgrMCB->rmClientServiceHandle);
    if (*errCode != RM_OK)
    {
        System_printf ("Error: Closing the RM service failed [Error code %d]\n", *errCode);
        return -1;
    }

    /* Shutdown the client transport */
    if (Resmgr_deleteClientTransport(ptrResmgrMCB->rmClientTransportHandle, errCode) < 0)
    {
        System_printf ("Error: Deleting the RM client transport failed [Error code %d]\n", *errCode);
        *errCode = RESMGR_EINTERNAL;
        return -1;
    }

    /* Shutdown the RM client: Ignore any pending services. */
    *errCode = Rm_delete (ptrResmgrMCB->rmClientHandle, 1);
    if (*errCode != RM_OK)
    {
        System_printf ("Error: Deleting the RM client failed [Error code %d]\n", *errCode);
        return -1;
    }

    /* Delete the semaphore */
    ptrResmgrMCB->cfg.deleteSem (ptrResmgrMCB->semaphore.rmSemaphore);

    /* Cleanup the resource manager memory */
    ptrResmgrMCB->cfg.free (Resmgr_MallocMode_LOCAL, ptrResmgrMCB, sizeof(Resmgr_MCB));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the wrapper OSAL API which wraps the multiple thread
 *      protection required by RM through the RESMGR OSAL API
 *
 *  @param[in]  mtSemObj
 *      Multi-threaded critical section object handle.
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Handle used to lock the multi-threaded critical section
 */
void* Osal_rmMtCsEnter (void *mtSemObj)
{
    Resmgr_Sem* ptrResmgrSem;
    Resmgr_MCB* ptrResmgrMCB;

    /* Get the resource manager semaphore wrapper: Validate and ensure that the RM
     * layer is passing back the correct object */
    ptrResmgrSem = (Resmgr_Sem*)mtSemObj;
    if (ptrResmgrSem == NULL)
    {
        System_printf ("Error: Multiple thread semaphore object is NULL in enter\n");
        return NULL;
    }

    /* Sanity Check: Ensure that a valid resource manager MCB has been registered */
    ptrResmgrMCB = ptrResmgrSem->ptrResmgrMCB;
    if (ptrResmgrMCB == NULL)
    {
        System_printf ("Error: Resource Manager MCB is NOT registered in enter\n");
        return NULL;
    }

    /* Pend on the semaphore */
    ptrResmgrMCB->cfg.pendSem (ptrResmgrMCB->semaphore.rmSemaphore);
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is the wrapper OSAL API which wraps the multiple thread
 *      protection required by RM through the RESMGR OSAL API
 *
 *  @param[in]  mtSemObj
 *      Multi-threaded critical section object handle.
 *  @param[in]  CsHandle
 *      Critical section handle
 *
 *  \ingroup RES_MGR_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
void Osal_rmMtCsExit (void *mtSemObj, void *CsHandle)
{
    Resmgr_Sem* ptrResmgrSem;
    Resmgr_MCB* ptrResmgrMCB;

    /* Get the resource manager semaphore wrapper: Validate and ensure that the RM
     * layer is passing back the correct object */
    ptrResmgrSem = (Resmgr_Sem*)mtSemObj;
    if (ptrResmgrSem == NULL)
    {
        System_printf ("Error: Multiple thread semaphore object is NULL in exit\n");
        return;
    }

    /* Sanity Check: Ensure that a valid resource manager MCB has been registered */
    ptrResmgrMCB = ptrResmgrSem->ptrResmgrMCB;
    if (ptrResmgrMCB == NULL)
    {
        System_printf ("Error: Resource Manager MCB is NOT registered in exit\n");
        return;
    }

    /* Post the semaphore */
    ptrResmgrMCB->cfg.postSem (ptrResmgrMCB->semaphore.rmSemaphore);
    return;
}
