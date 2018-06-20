/**
 *   @file  eqos.c
 *
 *   @brief
 *      The file implements the functionality required to configure the
 *      eQoS subsystem.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
//fzm---->
#include <ddal/ddal_common.h>
#include <ddal/ddal_cma.h>
//<----fzm
#include <stdio.h>
#include <malloc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* SYSLIB Include Files */
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_master/include/netfp_master_internal.h>

/**********************************************************************
 ************************** QOS Functions *****************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  size
 *      Size of the memory to be allocated
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* NetfpMaster_eQosMalloc(uint32_t size, uint32_t arg)
{
//fzm---->
    (void)arg;

    uint8_t* ptr = NULL;
    if(ddal_cma_alloc(DDAL_CMA_MEM_TYPE_DDR, size, (void *)&ptr) != DDAL_OK)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to allocate memory");
        return NULL;
    }
    return ptr;
//<----fzm
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap
 *      interface table.
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      Application specific argument
 *
 *  @retval
 *      Not Applicable.
 */
static void NetfpMaster_eQosFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    (void)ptr; (void)size; (void)arg; //fzm
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the L2 Shaper with the configuration
 *      present in the NETFP interface block
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the application MCB
 *  @param[in]  ptrNetfpIfBlock
 *      Pointer to the NETFP Interface block
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t NetfpMaster_setupL2Shaper
(
    NetfpMaster_MCB*        ptrNetfpMasterMCB,
    NetfpMaster_IfBlock*    ptrNetfpIfBlock,
    int32_t*                errCode
)
{
    paEQosModeConfig_t      eQosConfig;
    int32_t                 index;

    /* Initialize the eQoS configuration */
    memset ((void* )&eQosConfig, 0 , sizeof(paEQosModeConfig_t));

    /* Translate into the PA LLD Language. */
    if (ptrNetfpIfBlock->routingMode == NetfpMaster_RoutingMode_DSCP)
    {
        /* DSCP Mode: */
        eQosConfig.ctrlBitMap = 0;
    }
    else
    {
        /* DP-Bit Mode: */
        eQosConfig.ctrlBitMap = eQosConfig.ctrlBitMap | pa_IF_EQoS_ROUTE_DP_BIT_MODE;

        /* Enable the priority override */
        if (ptrNetfpIfBlock->priorityOverride == 1)
            eQosConfig.ctrlBitMap = eQosConfig.ctrlBitMap | pa_IF_EQoS_PRIORITY_OVERRIDE_ENABLE;
    }

    /* Cycle through and configure the VLAN pbit map */
    for (index = 0; index < NETFP_MASTER_MAX_VLAN_ENTRY; index++)
    {
        eQosConfig.pbitMap[index].flowOffset  = ptrNetfpIfBlock->vlanMap[index].flowOffset;
        eQosConfig.pbitMap[index].queueOffset = ptrNetfpIfBlock->vlanMap[index].queueOffset;
    }

    /* Cycle through and configure the DSCP pbit map */
    for (index = 0; index < NETFP_MASTER_MAX_DSCP_ENTRY; index++)
    {
        eQosConfig.dscpMap[index].flowOffset  = ptrNetfpIfBlock->dscpMap[index].flowOffset;
        eQosConfig.dscpMap[index].queueOffset = ptrNetfpIfBlock->dscpMap[index].queueOffset;
    }

    /* Populate the remaining fields. */
    eQosConfig.port          = ptrNetfpIfBlock->switchPort;
    eQosConfig.ingressDefPri = ptrNetfpIfBlock->defaultFwdPriority;
    eQosConfig.vlanId        = 0;
    eQosConfig.flowBase      = ptrNetfpIfBlock->baseFlowId;
    eQosConfig.queueBase     = ptrNetfpIfBlock->eqosQueueHandle[0];

    /* Enable the L2 shaper for each interface */
    if (Netfp_enableL2Shaper(ptrNetfpMasterMCB->netfpServerHandle, &eQosConfig, errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: eQoS initialization for interface %s failed [Error code %d]\n",
                        ptrNetfpIfBlock->name, *errCode);
        return -1;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: eQoS for interface %s [Mode %s] initialized successfully\n",
                    ptrNetfpIfBlock->name, (ptrNetfpIfBlock->routingMode == NetfpMaster_RoutingMode_DSCP) ? "dscp" : "dp-bit");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and configure the eQoS subsystem
 *      using the configuration information provided to the master
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master MCB
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpMaster_initQoS(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    NetfpMaster_IfBlock*    ptrNetfpIfBlock;
    Netfp_FlowCfg           flowCfg;
    Pktlib_HeapCfg          heapCfg;
    int32_t                 index;
    int32_t                 errCode;
#if 0 //fzm
    uint32_t                totalDescriptors = 0;
    Resmgr_ResourceCfg      eQoSMasterResourceConfig =
    {
    	0,    /* Number of CPINTC Output  requested                     */
	    0,    /* Number of Accumulator Channels requested               */
    	0,    /* Number of Hardware Semaphores requested                */
	    0,    /* Number of QPEND Queues requested                       */
        /* Requested Memory Region Configuration.                       */
        {
            /* Name,                    Type,                       Linking RAM,                           Num,        Size */
            { "NETFPMaster_eQoSRegion", Resmgr_MemRegionType_DDR3,  Resmgr_MemRegionLinkingRAM_DONT_CARE,  0,        128},
        }
    };

    /*********************************************************************************
     * Step1: Insert the memory region by determining the total number of descriptors
     * which are required by all the interfaces
     *********************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Cycle through all the flows configurations for the interface and determine the number of descriptors. */
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
            totalDescriptors = totalDescriptors + ptrNetfpIfBlock->flowBlock[index].numDescriptors;

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: eQoS needs %d descriptors for all interfaces\n", totalDescriptors);

    /* Round off the total number of descriptors required by all the interface flows: */
    totalDescriptors = NetfpMaster_roundNumberDescriptors (totalDescriptors);
    if (totalDescriptors == 0)
        return -1;

    /* Populate the resource configuration: */
    eQoSMasterResourceConfig.memRegionCfg[0].numDesc = totalDescriptors;

    /* Process the eQoS configuration: */
    if (Resmgr_processConfig (ptrNetfpMasterMCB->handleSysCfg, &eQoSMasterResourceConfig, &errCode) < 0)
	{
	    NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: SYSRM configuration failed with error code %d\n", errCode);
	    return -1;
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Successfully processed eQoS Resource configuration\n");
#endif //fzm

    /*********************************************************************************
     * Step2: Cycle through all the interfaces and open the requested queues for each
     * interface. Queues allocated should be contiguous.
     *********************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Debug Message: */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Interface %s requested for %d queues\n",
                        ptrNetfpIfBlock->name, ptrNetfpIfBlock->maxQueueOffset + 1);

        /* The L2 shaper queues have already been opened and assigned to the Linux kernel. We are simply passing the queue
         * numbers to the eQOS configuration block. We dont use these queues for any other purpose. Open up the requested
         * queues. We dont use the Qmss_queueOpen API because that will traverse the RM path and we will then need to
         * mark the queues in the RM policy files as shared. */
        for (index = 0; index < (ptrNetfpIfBlock->maxQueueOffset+1); index++)
            ptrNetfpIfBlock->eqosQueueHandle[index] = ptrNetfpIfBlock->baseQueue + index;

        /* Queues have been successfully allocated: Display the allocated eQoS queues. */
        for (index = 0; index < (ptrNetfpIfBlock->maxQueueOffset+1); index++)
        {
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Interface %s Queue allocated %d\n",
                            ptrNetfpIfBlock->name, ptrNetfpIfBlock->eqosQueueHandle[index]);
        }

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Successfully allocated eQoS queues\n");

    /*********************************************************************************
     * Step3: Create PKTLIB Heaps for each interface as requested by the flow
     * configuration block.
     *********************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Cycle through all the interface flows: */
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
        {
            /* Is the slot free? */
            if (ptrNetfpIfBlock->flowBlock[index].numDescriptors == 0)
                continue;

            /* Initialize the heap configuration */
            memset((void *)&heapCfg, 0 , sizeof(Pktlib_HeapCfg));

            /* Populate the heap configuration */
            snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "__eQoS_Heap_%s_%d", ptrNetfpIfBlock->name, index);
            //heapCfg.memRegion                       = eQoSMasterResourceConfig.memRegionResponse[0].memRegionHandle; //fzm
            heapCfg.memRegion                       = ptrNetfpMasterMCB->memoryRegionHandle;
            heapCfg.pktlibInstHandle                = ptrNetfpMasterMCB->pktlibInstanceHandle;
            heapCfg.sharedHeap                      = 0;
            heapCfg.useStarvationQueue              = 1;
            heapCfg.dataBufferSize                  = ETH_MAX_PKT_SIZE;
            heapCfg.numPkts                         = ptrNetfpIfBlock->flowBlock[index].numDescriptors;
            heapCfg.numZeroBufferPackets            = 0;
            heapCfg.dataBufferPktThreshold          = 0;
            heapCfg.zeroBufferPktThreshold          = 0;
            heapCfg.heapInterfaceTable.dataMalloc   = NetfpMaster_eQosMalloc;
            heapCfg.heapInterfaceTable.dataFree     = NetfpMaster_eQosFree;
            heapCfg.linearAlloc = 1; //fzm

            /* Create the interface eQoS heap: */
            ptrNetfpIfBlock->flowBlock[index].heapHandle = Pktlib_createHeap(&heapCfg, &errCode);
            if (ptrNetfpIfBlock->flowBlock[index].heapHandle == NULL)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to create eQoS Heap '%s' [Error code %d]\n",
                                heapCfg.name, errCode);
                return -1;
            }
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: eQoS Heap '%s' created successfully\n", heapCfg.name);
        }

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Successfully created eQoS PKTLIB heaps\n");

    /*********************************************************************************
     * Step4: Create flows for each interface
     *********************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Cycle through all the interface flows: */
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
        {
            /* Is the slot free? */
            if (ptrNetfpIfBlock->flowBlock[index].numDescriptors == 0)
                continue;

            /* Initialize the flow configuration. */
            memset((void *)&flowCfg, 0, sizeof(Netfp_FlowCfg));

            /* Populate the flow configuration */
            flowCfg.numHeaps      = 1;
            flowCfg.heapHandle[0] = ptrNetfpIfBlock->flowBlock[index].heapHandle;
            flowCfg.sopOffset     = 0;
            snprintf (flowCfg.name, NETFP_MAX_CHAR, "__eQoS_Flow_%s_%d", ptrNetfpIfBlock->name, index);

            /* Create the flow */
            ptrNetfpIfBlock->flowBlock[index].flowId = Netfp_createFlow (ptrNetfpMasterMCB->netfpClientHandle, &flowCfg, &errCode);
            if (ptrNetfpIfBlock->flowBlock[index].flowId < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Unable to create eQoS Flow for '%s' Flow configuration %d [Error code %d]\n",
                                ptrNetfpIfBlock->name, index, errCode);
                return -1;
            }

            /* Debug Message: */
            NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: eQoS Flow for '%s' Flow configuration %d = %d\n",
                            ptrNetfpIfBlock->name, index, ptrNetfpIfBlock->flowBlock[index].flowId);
        }

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Successfully created eQoS flows\n");

    /*********************************************************************************
     * Step5: Validate and ensure that the flows are sequential for each interface
     *********************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        int32_t baseFlowId = -1;

        /* Cycle through all the interface flows: */
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
        {
            /* Is the slot free? */
            if (ptrNetfpIfBlock->flowBlock[index].numDescriptors == 0)
                continue;

            /* Is this the first flow for the interface? */
            if (baseFlowId == -1)
            {
                /* YES: Remember it and continue to the next entry. */
                baseFlowId = ptrNetfpIfBlock->flowBlock[index].flowId;
                ptrNetfpIfBlock->baseFlowId = baseFlowId;
            }
            else
            {
                /* Next flow identifier should always be +1 the base identifier */
                if (ptrNetfpIfBlock->flowBlock[index].flowId == (baseFlowId + 1))
                {
                    /* YES. Update the base flow identifier and continue */
                    baseFlowId = ptrNetfpIfBlock->flowBlock[index].flowId;
                }
                else
                {
                    /* NO. Flow identifiers are not contiguous. */
                    NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Flow identifiers are not contigious for %s [%d %d]\n",
                                    ptrNetfpIfBlock->name, ptrNetfpIfBlock->flowBlock[index].flowId, baseFlowId+1);
                }
            }
        }
        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* Debug Message: */
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Successfully validated contiguous eQoS flows\n");

    /************************************************************************************
     * Debug: Display the eQoS configuration as it will be configured in the NETCP
     ************************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Debug: */
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Interface                  : %s\n", ptrNetfpIfBlock->name);
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Flow Base                  : %d\n", ptrNetfpIfBlock->baseFlowId);
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Queue Base                 : %d\n", ptrNetfpIfBlock->eqosQueueHandle[0]);
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Port                       : %d\n", ptrNetfpIfBlock->switchPort);
        NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Ingress Default Priority   : %d\n", ptrNetfpIfBlock->defaultFwdPriority);

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /************************************************************************************
     * Enable Global enhanced QOS across the system.
     ************************************************************************************/
    if (Netfp_initEQOS (ptrNetfpMasterMCB->netfpServerHandle, ptrNetfpMasterMCB->defaultHostPriority, &errCode) < 0)
    {
        NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Global eQOS configuration failed [Error code %d]\n", errCode);
        return -1;
    }
    NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Global eQOS configured successfully\n");

    /************************************************************************************
     * Cycle through each interface and setup the enhanced QoS block for each interface
     ************************************************************************************/
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Setup and enable the L2 shaper for the specified NETFP interface block. */
        if (NetfpMaster_setupL2Shaper (ptrNetfpMasterMCB, ptrNetfpIfBlock, &errCode) < 0)
            return -1;

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the eQoS subsystem
 *
 *  @param[in]  ptrNetfpMasterMCB
 *      Pointer to the NETFP Master MCB
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t NetfpMaster_deInitQoS(NetfpMaster_MCB* ptrNetfpMasterMCB)
{
    NetfpMaster_IfBlock*    ptrNetfpIfBlock;
    int32_t                 index;
    int32_t                 errCode;

    /* Cycle through all the registered interfaces. */
    ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetHead((NetfpMaster_ListNode**)&ptrNetfpMasterMCB->ptrInterfaceList);
    while (ptrNetfpIfBlock != NULL)
    {
        /* Cycle through all the interface flows: */
        for (index = 0; index < NETFP_MASTER_MAX_FLOW; index++)
        {
            /* Is the slot free? */
            if (ptrNetfpIfBlock->flowBlock[index].heapHandle == NULL)
                continue;

            /* Delete the interface eQoS heap: */
            if (Pktlib_deleteHeap (ptrNetfpMasterMCB->pktlibInstanceHandle, ptrNetfpIfBlock->flowBlock[index].heapHandle, &errCode) < 0)
            {
                NetfpMaster_log(NetfpMaster_LogLevel_ERROR, "Error: Pktlib eQoS Heap [%p] for %s failed [Error code %d]\n",
                                ptrNetfpIfBlock->flowBlock[index].heapHandle, ptrNetfpIfBlock->name, errCode);
            }
            else
            {
                NetfpMaster_log(NetfpMaster_LogLevel_DEBUG, "Debug: Pktlib eQoS Heap [%p] for %s deleted\n",
                                ptrNetfpIfBlock->flowBlock[index].heapHandle, ptrNetfpIfBlock->name);
            }
        }

        /* Get the next interface block */
        ptrNetfpIfBlock = (NetfpMaster_IfBlock*)NetfpMaster_listGetNext((NetfpMaster_ListNode*)ptrNetfpIfBlock);
    }

    /* TODO: We need to release the flows and queues also. */
    return 0;
}
