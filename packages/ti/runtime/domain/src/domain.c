/**
 *   @file  domain.c
 *
 *   @brief
 *      Domain SYSLIB management code for DSP & ARM realms.
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
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* MCSDK Include Files */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>

/* DSP specific includes */
#ifndef __ARMv7
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_chip.h>
#else
#define CACHE_L2_LINESIZE           128
#endif

/* SYSLIB Include Files */
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/runtime/memlog/memlog.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/domain/domain.h>
#include <ti/runtime/domain/include/domain_internal.h>

/**********************************************************************
 ************************* Domain Functions ***************************
 **********************************************************************/

#ifndef __ARMv7
/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table for heaps which require data buffers from the shared
 *      memory pool
 *
 *  @param[in]  size
 *      Size of memory block to be allocated
 *  @param[in]  arg
 *      Application specified argument
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Domain_sharedMemoryMalloc(uint32_t size, uint32_t arg)
{
    Domain_Syslib* ptrSyslibDomain;

    /* Get the SYSLIB domain */
    ptrSyslibDomain = (Domain_Syslib*)arg;

    /* Return the allocated data buffer */
    return ptrSyslibDomain->cfg.domainOsalFxnTable.dataMalloc(Domain_MallocMode_GLOBAL, size, CACHE_L2_LINESIZE);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap interface table
 *      to clean memory for data buffers allocated from the shared memory pool
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      Application specified argument
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_sharedMemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    Domain_Syslib* ptrSyslibDomain;

    /* Get the SYSLIB domain */
    ptrSyslibDomain = (Domain_Syslib*)arg;

    /* Cleanup the data buffer memory. */
    ptrSyslibDomain->cfg.domainOsalFxnTable.dataFree(Domain_MallocMode_GLOBAL, ptr, size);
}
#endif /* __ARMv7 */

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Allocator which is instantiated into the heap
 *      interface table for heaps which require data buffers from the private
 *      memory pool
 *
 *  @param[in]  size
 *      Size of memory block to be allocated
 *  @param[in]  arg
 *      Application specified argument
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Success     -   Pointer to the allocated block of memory.
 *  @retval
 *      Error       -   NULL
 */
static uint8_t* Domain_localMemoryMalloc(uint32_t size, uint32_t arg)
{
    Domain_Syslib* ptrSyslibDomain;

    /* Get the SYSLIB domain */
    ptrSyslibDomain = (Domain_Syslib*)arg;

    /* Return the allocated data buffer */
    return ptrSyslibDomain->cfg.domainOsalFxnTable.dataMalloc(Domain_MallocMode_LOCAL, size, CACHE_L2_LINESIZE);
}

/**
 *  @b Description
 *  @n
 *      Heap Data Buffer Cleanup which is instantiated into the heap interface table
 *      to clean memory for data buffers allocated from the private memory pool
 *
 *  @param[in]  ptr
 *      Pointer to the memory which is to be cleaned up.
 *  @param[in]  size
 *      Size of the memory which is to be cleaned up.
 *  @param[in]  arg
 *      Application specified argument
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_localMemoryFree(uint8_t* ptr, uint32_t size, uint32_t arg)
{
    Domain_Syslib* ptrSyslibDomain;

    /* Get the SYSLIB domain */
    ptrSyslibDomain = (Domain_Syslib*)arg;

    /* Cleanup the data buffer memory. */
    ptrSyslibDomain->cfg.domainOsalFxnTable.dataFree(Domain_MallocMode_LOCAL, ptr, size);
}

/**
 *  @b Description
 *  @n
 *      The function is a wrapper function which is used to log all messages generated by
 *      the DOMAIN module.
 *
 *  @param[in]  ptrSyslibDomain
 *      Pointer to the SYSLIB domain
 *  @param[in]  logLevel
 *      Logging Level
 *  @param[in]  fmt
 *      Formatted string
 *  @param[in]  ...
 *      Variable arguments
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_logMessage (Domain_Syslib* ptrSyslibDomain, Domain_LogLevel logLevel, char* fmt, ...)
{
    va_list arg;

    /* Pass control to the application supplied logging function. */
    if (ptrSyslibDomain->cfg.domainOsalFxnTable.log)
    {
        va_start (arg, fmt);
        ptrSyslibDomain->cfg.domainOsalFxnTable.log (logLevel, fmt, arg);
        va_end (arg);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create and initialize the PKTLIB heaps which are
 *      used by the SYSLIB services for the domain.
 *
 *  @param[in]  ptrSyslibDomain
 *      Pointer to the SYSLIB domain configuration
 *  @param[in]  ptrDomainResourceCfg
 *      Pointer to the domain resource configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Domain_createPktlibHeaps
(
    Domain_Syslib*      ptrSyslibDomain,
    Resmgr_ResourceCfg* ptrDomainResourceCfg,
    int32_t*            errCode
)
{
    Pktlib_HeapCfg        heapCfg;

#ifndef __ARMv7
    /* Name Proxy are created only in the context of the DSP realm and so we create the
     * heaps only for this realm. There are 2 heaps which are created in the DSP realm:
     *
     * Peer Heap:
     *  - This is a private heap which is visible only to the DSP core executing the proxy
     *    and is used to allocate messages and then DMA them out to the peer agent server on
     *    the ARM.
     *
     * Proxy-Client Shared Message Heap:
     *  - This is a shared heap which is used to exchange messages between the Name proxy
     *    and name clients (on DSP). This heap is because on the DSP the clients and servers
     *    communicate with each other via MSGCOM queue channels.
     *
     * On ARM Name proxy is present as a standalone application which can be launched. */
    if (ptrSyslibDomain->cfg.rootSyslibCfg.dspSyslibConfig.instantiateNameProxy)
    {
        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration
         *  - The heap is used for the reception and transmission of all messages
         *    exchanged with the peer name proxy server. */
        snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-ProxyHeap", ptrSyslibDomain->cfg.rootSyslibCfg.proxyName);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 1536;
        heapCfg.numPkts                         = 32;
        heapCfg.numZeroBufferPackets            = 0;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_localMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_localMemoryFree;
        ptrSyslibDomain->proxyHeapHandle = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->proxyHeapHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the Name Proxy heap '%s' [Error code %d]\n", heapCfg.name, *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug:Name Proxy heap [%s] created\n", heapCfg.name);

        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the configuration:
         *  - The agent server is responsible for creating the heap which is a shared heap with
         *    all the agent clients. This heap is used by the agent clients to send & receive
         *    messages with the agent server. */
        snprintf(heapCfg.name, PKTLIB_MAX_CHAR, "%s-ProxyClientHeap", ptrSyslibDomain->cfg.rootSyslibCfg.proxyName);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 1;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 1536;
        heapCfg.numPkts                         = 32;
        heapCfg.numZeroBufferPackets            = 0;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_sharedMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_sharedMemoryFree;
        ptrSyslibDomain->sharedProxyClientHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->sharedProxyClientHeap == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the Shared Agent Receive heap [Error code %d]\n", *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug:Name Proxy Client heap [%s] created\n", heapCfg.name);
    }
#endif

    /* NETFP Heaps: Each NETFP Client is responsible for creating its own networking header and fragment heap */
    if (ptrSyslibDomain->cfg.rootSyslibCfg.netfpClientConfig.instantiateNetfpClient)
    {
        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration
         *  - The heap is used to create networking headers which are populated by the NETFP
         *    library for every packet which is to be transmitted. */
        sprintf(heapCfg.name, "NETFP_ClientHeaderHeap_%x", ptrSyslibDomain->domainId);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 256;
        heapCfg.numPkts                         = 64;
        heapCfg.numZeroBufferPackets            = 0;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_localMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_localMemoryFree;
        ptrSyslibDomain->netfpHeaderHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->netfpHeaderHeap == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the networking header heap [Error code %d]\n", *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Header heap [%s] created [Queue %x]\n",
                           heapCfg.name, Pktlib_getInternalHeapQueue(ptrSyslibDomain->netfpHeaderHeap));

        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration
         *  - This heap is used to send fragmented packets provided the fragmentation is done
         *    in NETFP software. */
        sprintf(heapCfg.name, "NETFP_ClientFragmentHeap_%x", ptrSyslibDomain->domainId);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 0;
        heapCfg.numPkts                         = 0;
        heapCfg.numZeroBufferPackets            = 128;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_localMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_localMemoryFree;
        ptrSyslibDomain->netfpFragHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->netfpFragHeap == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the networking fragmentation heap [Error code %d]\n", *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Fragment heap [%s] created\n", heapCfg.name);

        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration
         *  - The heap is used for the reception and transmission of all packets on this core */
        sprintf(heapCfg.name, "NETFP_ClientServerHeap_%x", ptrSyslibDomain->domainId);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 1536;
        heapCfg.numPkts                         = 64;
        heapCfg.numZeroBufferPackets            = 64;
        heapCfg.dataBufferPktThreshold          = 8;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_localMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_localMemoryFree;
        ptrSyslibDomain->netfpClientSrvHeap = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->netfpClientSrvHeap == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the NETFP Client/Server heap [Error code %d]\n", *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Client/Server heap [%s] created\n", heapCfg.name);
    }

    /* DAT Heap: Each DAT Client is responsible for creating its heap which allows it to exchange messages
     * with the DAT server. */
    if (ptrSyslibDomain->cfg.rootSyslibCfg.datClientConfig.instantiateDatClient)
    {
        /* Initialize the heap configuration. */
        memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));

        /* Populate the heap configuration
         *  - This heap is used to exchange messages between the DAT server & client. */
        sprintf(heapCfg.name, "DAT_ClientServerHeap_%x", ptrSyslibDomain->domainId);
        heapCfg.memRegion                       = ptrDomainResourceCfg->memRegionResponse[0].memRegionHandle;
        heapCfg.pktlibInstHandle                = ptrSyslibDomain->appPktlibInstanceHandle;
        heapCfg.sharedHeap                      = 0;
        heapCfg.useStarvationQueue              = 0;
        heapCfg.dataBufferSize                  = 1536;
        heapCfg.numPkts                         = 32;
        heapCfg.numZeroBufferPackets            = 0;
        heapCfg.dataBufferPktThreshold          = 0;
        heapCfg.zeroBufferPktThreshold          = 0;
        heapCfg.arg                             = (uint32_t)ptrSyslibDomain;
        heapCfg.heapInterfaceTable.dataMalloc   = Domain_localMemoryMalloc;
        heapCfg.heapInterfaceTable.dataFree     = Domain_localMemoryFree;
        ptrSyslibDomain->datClientHeap          = Pktlib_createHeap(&heapCfg, errCode);
        if (ptrSyslibDomain->datClientHeap == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the DAT client/server heap [Error code %d]\n", *errCode);
            return -1;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: DAT Client/Server heap [%s] created\n", heapCfg.name);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The domain name execution task which executes the name proxy/clients
 *
 *  @param[in]  arg
 *      Task Argument 0 which is set to the pointer to the SYSLIB Domain
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_nameExecutionTask(uint32_t arg)
{
    int32_t                 errCode;
    Domain_Syslib*          ptrSyslibDomain;

    /* Get the handle to the SYSLIB domain. */
    ptrSyslibDomain = (Domain_Syslib*)arg;
    if (ptrSyslibDomain == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "FATAL Error: Agent services invalid argument; aborting...\n");
        return;
    }

    /* SYSLIB name services have been initialized and will be available */
    ptrSyslibDomain->nameOperationalStatus = Domain_OperationStatus_AVAILABLE;

    /* Execute the name services for the domain unless indicated to stop. */
    while (ptrSyslibDomain->nameOperationalStatus == Domain_OperationStatus_AVAILABLE)
    {
        /* Execute the name proxy if configured */
        if (ptrSyslibDomain->nameProxyHandle != NULL)
        {
            if (Name_executeProxy (ptrSyslibDomain->nameProxyHandle, &errCode) < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "FATAL Error: Name Proxy execution failed [Error code %d]\n", errCode);
                return;
            }
        }

        /* Execute the name client if present */
        if (ptrSyslibDomain->nameClientHandle != NULL)
            Name_executeClient (ptrSyslibDomain->nameClientHandle);

        /* Relinquish time slice allowing other tasks to execute */
        ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
    }

    /***********************************************************************
     * Shutdown the name client services:
     ***********************************************************************/
    if (ptrSyslibDomain->nameClientHandle != NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Shutting down the Name client services...\n");
        while (1)
        {
            /* Delete the name client */
            if (Name_deleteClient (ptrSyslibDomain->nameClientHandle, &errCode) == 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Name client has been shutdown successfully\n");
                ptrSyslibDomain->nameClientHandle = NULL;
                break;
            }

            /* Name client deletion failed. It could be possible that the name services are still
             * being used. Use the error code to determine this */
            if (errCode == NAME_ENOTREADY)
            {
                /* Agent services are being used. Execute the client to get rid of any pending
                 * requests. Once the pending requests are services the client deletion will
                 * automatically succeed */
                Name_executeClient (ptrSyslibDomain->nameClientHandle);
                continue;
            }

            /* FATAL Error: Report and continue with the cleanup. */
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Error: Name client deletion failed [Error code %d]\n", errCode);
            break;
        }
    }

    /***********************************************************************
     * Shutdown the name proxy services:
     ***********************************************************************/
    if (ptrSyslibDomain->nameProxyHandle != NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Shutting down the Name Proxy services...\n");
        while (1)
        {
            /* Delete the name proxy */
            if (Name_deleteProxy (ptrSyslibDomain->nameProxyHandle, &errCode) == 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Name Proxy has been shutdown successfully\n");
                ptrSyslibDomain->nameProxyHandle = NULL;
                break;
            }

            /* Name proxy deletion failed. It could be possible that the name services are still being used
             * Use the error code to determine this */
            if (errCode == NAME_ENOTREADY)
            {
                /* Name services are being used. Execute the proxy to get rid of any pending
                 * requests. Once the pending requests are services the proxy deletion will
                 * automatically succeed */
                if (Name_executeProxy (ptrSyslibDomain->nameProxyHandle, &errCode) < 0)
                {
                    Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                       "FATAL Error: Name proxy execution during shutdown failed [Error code %d]\n", errCode);
                    return;
                }
                continue;
            }

            /* FATAL Error: Report and continue with the cleanup. */
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Error: Name proxy deletion failed [Error code %d]\n", errCode);
            break;
        }
    }

    /* Name services are not available */
    ptrSyslibDomain->nameOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    return;
}

/**
 *  @b Description
 *  @n
 *      NETFP client execution task which executes the NETFP client
 *      services which are being exchanged with the NETFP server. This
 *      task needs to be executed only if the root slave is configured
 *      to execute the NETFP client.
 *
 *  @param[in]  arg
 *      Task Argument 0 which is set to the pointer to the SYSLIB Domain
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_netfpClientExecutionTask(uint32_t arg)
{
    Domain_Syslib*          ptrSyslibDomain;

    /* Get the handle to the SYSLIB domain. */
    ptrSyslibDomain = (Domain_Syslib*)arg;
    if (ptrSyslibDomain == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "FATAL Error: NETFP Client service invalid argument; aborting...\n");
        return;
    }

    /* SYSLIB NETFP services have been initialized but are available */
    ptrSyslibDomain->netfpOperationalStatus = Domain_OperationStatus_AVAILABLE;

    /* Execute the NETFP services for the domain unless indicated to stop. */
    while (1)
        Netfp_executeClient (ptrSyslibDomain->netfpClientHandle);
}

/**
 *  @b Description
 *  @n
 *      DAT client execution Task which executes the DAT client services. It
 *      also provides a context for the DAT background services which handles
 *      all the producers and consumers. This should be the lowest priority
 *      task in the system since this does not relinuqish control.
 *
 *  @param[in]  arg
 *      Task Argument 0 which is set to the pointer to the SYSLIB Domain
 *
 *  \ingroup DOMAIN_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable.
 */
static void Domain_datClientExecutionTask(uint32_t arg)
{
    Domain_Syslib*  ptrSyslibDomain;

    /* Get the handle to the SYSLIB domain. */
    ptrSyslibDomain = (Domain_Syslib*)arg;
    if (ptrSyslibDomain == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "FATAL Error: DAT Client service invalid argument; aborting...\n");
        return;
    }

    /* SYSLIB DAT services have been initialized and are available */
    ptrSyslibDomain->datOperationalStatus = Domain_OperationStatus_AVAILABLE;

    /* Execute the DAT services for the domain unless indicated to stop. */
    while (ptrSyslibDomain->datOperationalStatus == Domain_OperationStatus_AVAILABLE)
    {
        Dat_executeClient (ptrSyslibDomain->datClientHandle);
        Dat_executeBackgroundActions (ptrSyslibDomain->datClientHandle);

        /**********************************************************************************
         * This needs to be analyzed; on DSP/ARM we need to allow other tasks to execute;
         * and we also allow the Idle Task a chance to run instead of taking the entire CPU.
         **********************************************************************************/
        ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
    }

    /* DAT services are not available */
    ptrSyslibDomain->datOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the SYSLIB services for the specific domain
 *
 *  @param[in]  domainHandle
 *      Pointer to the SYSLIB domain handle
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *     Success  -   0
 *  @retval
 *     Error    -   <0
 */
int32_t Domain_deinitSyslibServices (Domain_SyslibHandle domainHandle, int32_t* errCode)
{
    Domain_Syslib*  ptrSyslibDomain;

    /* Get the handle to the SYSLIB domain. */
    ptrSyslibDomain = (Domain_Syslib*)domainHandle;
    if (ptrSyslibDomain == NULL)
    {
        *errCode = DOMAIN_EINVAL;
        return -1;
    }

    /* Deactivate the DAT client services if required to do so */
    if (ptrSyslibDomain->datOperationalStatus == Domain_OperationStatus_AVAILABLE)
    {
        /* Stop the DAT client */
        if (Dat_stopClient (ptrSyslibDomain->datClientHandle, errCode) < 0)
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Error: DAT stop client failed [Error code %d]\n", *errCode);
        else
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Debug: DAT client stopped successfully\n");

        /* Set the flag to deactivate the DAT services. */
        ptrSyslibDomain->datOperationalStatus = Domain_OperationStatus_DEACTIVATE;

        /* SYNC point: Ensure that the DAT services have been shutdown and are not available */
        while (1)
        {
            if (ptrSyslibDomain->datOperationalStatus == Domain_OperationStatus_NOT_AVAILABLE)
                break;

            /* Still not down; relinquish control */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: DAT services have been deactivated\n");

        /* Delete the client. */
        if (Dat_deleteClient (ptrSyslibDomain->datClientHandle, errCode) < 0)
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Error: DAT Client deletion failed [Error code %d]\n", *errCode);
        else
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: DAT Client deleted successfully\n");
    }

    /* Deactivate the NETFP client services if required to do so. */
    if (ptrSyslibDomain->netfpOperationalStatus == Domain_OperationStatus_AVAILABLE)
    {
        /* Set the flag to deactivate the NETFP services. */
        ptrSyslibDomain->netfpOperationalStatus = Domain_OperationStatus_DEACTIVATE;

        /* Stop the NETFP client */
        if (Netfp_stopClient (ptrSyslibDomain->netfpClientHandle, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to stop the NETFP client [Error code %d]\n", errCode);
            return -1;
        }

        /* Shutdown the NETFP Client execution task */
        ptrSyslibDomain->cfg.domainOsalFxnTable.taskDelete (ptrSyslibDomain->netfpClientTaskHandle);

        /***********************************************************************
         * Shutdown the NETFP client services:
         ***********************************************************************/
        if (Netfp_deleteClient (ptrSyslibDomain->netfpClientHandle, errCode) < 0)
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR, "Error: NETFP Client deletion failed [Error code %d]\n", *errCode);
        else
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Client deleted successfully\n");
    }

    /* Deactivate the name client services if required to do so. */
    if (ptrSyslibDomain->nameOperationalStatus == Domain_OperationStatus_AVAILABLE)
    {
        /* Set the flag to deactivate the name services. */
        ptrSyslibDomain->nameOperationalStatus = Domain_OperationStatus_DEACTIVATE;

        /* SYNC point: Ensure that the name services have been shutdown and are not available */
        while (1)
        {
            if (ptrSyslibDomain->nameOperationalStatus == Domain_OperationStatus_NOT_AVAILABLE)
                break;

            /* Still not down; relinquish control */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Name services have been deactivated\n");
    }

    /***********************************************************************************
     * Shutdown the PKTLIB Heaps.
     ***********************************************************************************/
    if (ptrSyslibDomain->proxyHeapHandle != NULL)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle, ptrSyslibDomain->proxyHeapHandle, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Name Proxy Heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: Name Proxy Heap has been deleted successfully\n");
        }
    }

    if (ptrSyslibDomain->datClientHeap != NULL)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle, ptrSyslibDomain->datClientHeap, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: DAT Client/Server Heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: DAT Client/Server Heap has been deleted successfully\n");
        }
    }

#ifndef __ARMv7
    /* DSP Realm: The Name proxy created a shared message heap which was used by all the clients
     * to communicate with the proxy. Once the DSP core executing the proxy is bought down
     * this heap is deleted with it.
     *
     * It is thus recommended that RAT Masters bring down the DSP core executing the name proxy
     * right at the end. */
    if (ptrSyslibDomain->cfg.rootSyslibCfg.dspSyslibConfig.instantiateNameProxy)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle,
                               ptrSyslibDomain->sharedProxyClientHeap, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                              "Error: Name Proxy-Client Heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: Name Proxy-Client Heap has been deleted successfully\n");
        }
    }
#else
    /* ARM Realm: Name proxy and clients communicate using UNIX sockets. In the ARM realm the name proxy is
     * executed as a seperate standalone server so this is NOT required to be done. */
#endif

    if (ptrSyslibDomain->netfpHeaderHeap != NULL)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle, ptrSyslibDomain->netfpHeaderHeap, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: NETFP Header Heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: NETFP Header Heap has been deleted successfully\n");
        }
    }
    if (ptrSyslibDomain->netfpFragHeap != NULL)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle, ptrSyslibDomain->netfpFragHeap, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: NETFP Fragmentation Heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: NETFP Fragmentation Heap has been deleted successfully\n");
        }
    }
    if (ptrSyslibDomain->netfpClientSrvHeap != NULL)
    {
        if (Pktlib_deleteHeap (ptrSyslibDomain->appPktlibInstanceHandle, ptrSyslibDomain->netfpClientSrvHeap, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: NETFP Client server heap deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: Netfp Client server heap has been deleted successfully\n");
        }
    }

    /* Shutdown the MSGCOM instance */
    if (ptrSyslibDomain->appMsgcomInstanceHandle)
    {
        if (Msgcom_deleteInstance (ptrSyslibDomain->appMsgcomInstanceHandle, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: MSGCOM delete instance failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: MSGCOM instance has been deleted successfully\n");
        }
    }

    /* Shutdown the PKTLIB instance */
    if (ptrSyslibDomain->appPktlibInstanceHandle)
    {
        if (Pktlib_deleteInstance (ptrSyslibDomain->appPktlibInstanceHandle, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: PKTLIB delete instance failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: PKTLIB instance has been deleted successfully\n");
        }
    }

    /* Close the resource manager */
    if (Resmgr_deinit (ptrSyslibDomain->handleSysCfg, errCode) < 0)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: Resource Manager clean up failed [Error code %d]\n", *errCode);
    }
    else
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Resource Manager deinitialized successfully\n");
    }

    /* Shutdown the named resource instance */
    if (ptrSyslibDomain->databaseHandle)
    {
        if (Name_deleteDatabase (ptrSyslibDomain->databaseHandle, errCode) < 0)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Name database deletion failed [Error code %d]\n", *errCode);
        }
        else
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                               "Debug: Name database has been deleted successfully\n");
        }
    }

    /* Cleanup memory allocated for the SYSLIB domain instance */
    ptrSyslibDomain->cfg.domainOsalFxnTable.free ((void*)ptrSyslibDomain, sizeof(Domain_Syslib));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function will initialize the configure and initialize the SYSLIB services used by
 *      the specific domain
 *
 *  @param[in]  appId
 *      Unique application identifier to identify the domain
 *  @param[in]  ptrDomainSyslibCfg
 *      Pointer to the Domain SYSLIB configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Success  -   Pointer to the domain handle
 *  @retval
 *      Error    -   NULL
 */
Domain_SyslibHandle Domain_initSyslibServices
(
    uint32_t            appId,
    Domain_SyslibCfg*   ptrDomainSyslibCfg,
    int32_t*            errCode
)
{
    Resmgr_SystemCfg            sysConfig;
    Name_ClientCfg              clientCfg;
    Netfp_ClientConfig          netfpClientConfig;
    Root_SyslibConfig*          ptrRootSyslibCfg;
    int32_t                     clientStatus;
    Msgcom_InstCfg              msgcomInstCfg;
    Pktlib_InstCfg              pktlibInstCfg;
    Name_DatabaseCfg            databaseCfg;
    int32_t                     tmpErrCode;
    Domain_Syslib*              ptrSyslibDomain;
    Dat_ClientCfg               datClientCfg;

    /* Sanity Check: Validate the arguments: */
    if (ptrDomainSyslibCfg == NULL)
    {
        *errCode = DOMAIN_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the DOMAIN OSAL call functions are populated correctly
     * - Logging Function is optional */
    if ((ptrDomainSyslibCfg->domainOsalFxnTable.malloc          == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.free            == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.dataMalloc      == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.dataFree        == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate      == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.taskDelete      == NULL) ||
        (ptrDomainSyslibCfg->domainOsalFxnTable.taskRelinquish  == NULL))
    {
        *errCode = DOMAIN_EINVAL;
        return NULL;
    }

    /* Sanity Check: We need to have 1 memory region valid request to initialize the SYSLIB services. */
    if (ptrDomainSyslibCfg->domainResourceCfg.memRegionCfg[0].numDesc == 0)
    {
        *errCode = DOMAIN_EINVAL;
        return NULL;
    }

    /* Sanity Check: We need to have at least 1 Direct Interrupt queue requested for the NETFP Clients
     * NETFP Clients use Blocking Direct Interrupt MSGCOM channels. */
    if (ptrDomainSyslibCfg->rootSyslibCfg.netfpClientConfig.instantiateNetfpClient == 1)
    {
        if (ptrDomainSyslibCfg->domainResourceCfg.numQpendQueues != 1)
        {
            *errCode = DOMAIN_EINVAL;
            return NULL;
        }
    }

    /* Allocate memory for the SYSLIB Domain. */
    ptrSyslibDomain = (Domain_Syslib*)ptrDomainSyslibCfg->domainOsalFxnTable.malloc(sizeof(Domain_Syslib));
    if (ptrSyslibDomain == NULL)
    {
        *errCode = DOMAIN_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrSyslibDomain, 0 , sizeof(Domain_Syslib));

    /* Populate the SYSLIB Domain */
    ptrSyslibDomain->domainId = appId;
    memcpy ((void *)&ptrSyslibDomain->cfg, (void *)ptrDomainSyslibCfg, sizeof(Domain_SyslibCfg));

    /* Get the SYSLIB configuration */
    ptrRootSyslibCfg = (Root_SyslibConfig*)&ptrSyslibDomain->cfg.rootSyslibCfg;

    /* Initialize the database configuration */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

#ifndef __ARMv7
    /* DSP named resource instance configuration:
     *  - We specify the shared memory address & size which is used to store the database */
    databaseCfg.instanceId                        = ptrRootSyslibCfg->nrInstanceId;
    databaseCfg.realm                             = Name_ExecutionRealm_DSP;
    strncpy (databaseCfg.owner, ptrRootSyslibCfg->name, NAME_MAX_CHAR);
    databaseCfg.dspCfg.baseNamedResourceAddress   = ptrRootSyslibCfg->dspSyslibConfig.namedResourceSharedMemAddress;
    databaseCfg.dspCfg.sizeNamedResourceMemory    = ptrRootSyslibCfg->dspSyslibConfig.sizeNamedResourceSharedMemAddress;
    databaseCfg.dspCfg.initNamedResourceDatabase  = ptrRootSyslibCfg->dspSyslibConfig.initNamedResourceDatabase;
    databaseCfg.dspCfg.malloc                     = ptrDomainSyslibCfg->nameDBOsalFxnTable.malloc;
    databaseCfg.dspCfg.free                       = ptrDomainSyslibCfg->nameDBOsalFxnTable.free;
    databaseCfg.dspCfg.enterCS                    = ptrDomainSyslibCfg->nameDBOsalFxnTable.enterCS;
    databaseCfg.dspCfg.exitCS                     = ptrDomainSyslibCfg->nameDBOsalFxnTable.exitCS;
    databaseCfg.dspCfg.beginMemAccess             = ptrDomainSyslibCfg->nameDBOsalFxnTable.beginMemAccess;
    databaseCfg.dspCfg.endMemAccess               = ptrDomainSyslibCfg->nameDBOsalFxnTable.endMemAccess;
#else
    /* ARM named resource instance configuration: */
    databaseCfg.instanceId                        = ptrRootSyslibCfg->nrInstanceId;
    databaseCfg.realm                             = Name_ExecutionRealm_ARM;
    strncpy (databaseCfg.owner, ptrRootSyslibCfg->name, NAME_MAX_CHAR);
#endif

    /* Initialize the named resource domain. */
    ptrSyslibDomain->databaseHandle = Name_createDatabase (&databaseCfg, errCode);
    if (ptrSyslibDomain->databaseHandle == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: Database failed to initialize [Error code %d]\n", *errCode);

        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
        return NULL;
    }
    Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                       "Debug: Database [0x%p] with Instance Id %d created successfully \n",
                        ptrSyslibDomain->databaseHandle, databaseCfg.instanceId);

    /* Initialize the system configuration. */
    memset ((void *)&sysConfig, 0, sizeof(Resmgr_SystemCfg));

#ifndef __ARMv7
    /* DSP Realm specific system configuration:
     *  - We need to specify the shared memory address and IPC source identifier which is used
     *    to communicate with the RM server executing on the ARM
     *  - The "coreId" is the core number for the DSP. */
    sysConfig.realm                             = Resmgr_ExecutionRealm_DSP;
    sysConfig.coreId                            = ptrRootSyslibCfg->coreId;
    strcpy (sysConfig.rmClient, ptrRootSyslibCfg->rmClient);
    strcpy (sysConfig.rmServer, ptrRootSyslibCfg->rmServer);
    sysConfig.malloc                            = ptrDomainSyslibCfg->resmgrOsalFxnTable.malloc;
    sysConfig.free                              = ptrDomainSyslibCfg->resmgrOsalFxnTable.free;
    sysConfig.mallocMemoryRegion                = ptrDomainSyslibCfg->resmgrOsalFxnTable.mallocMemoryRegion;
    sysConfig.freeMemoryRegion                  = ptrDomainSyslibCfg->resmgrOsalFxnTable.freeMemoryRegion;
    sysConfig.createSem                         = ptrDomainSyslibCfg->resmgrOsalFxnTable.createSem;
    sysConfig.deleteSem                         = ptrDomainSyslibCfg->resmgrOsalFxnTable.deleteSem;
    sysConfig.postSem                           = ptrDomainSyslibCfg->resmgrOsalFxnTable.postSem;
    sysConfig.pendSem                           = ptrDomainSyslibCfg->resmgrOsalFxnTable.pendSem;
    sysConfig.beginMemAccess                    = ptrDomainSyslibCfg->resmgrOsalFxnTable.beginMemAccess;
    sysConfig.endMemAccess                      = ptrDomainSyslibCfg->resmgrOsalFxnTable.endMemAccess;
    sysConfig.dspSystemCfg.armCoreId            = ptrRootSyslibCfg->dspSyslibConfig.armCoreId;
    sysConfig.dspSystemCfg.sourceId             = ptrRootSyslibCfg->dspSyslibConfig.sysRMIPCSourceId;
    sysConfig.dspSystemCfg.sharedMemAddress     = ptrRootSyslibCfg->dspSyslibConfig.sharedMemAddress;
    sysConfig.dspSystemCfg.sizeSharedMemory     = ptrRootSyslibCfg->dspSyslibConfig.sizeSharedMemory;
#else
    /* ARM Realm specific system configuration:
     *  - The "coreId" is the core number for the ARM on which the server is executing.
     *  - The RM clients on ARM communicate with the server using UNIX sockets  */
    sysConfig.realm                             = Resmgr_ExecutionRealm_ARM;
    sysConfig.coreId                            = ptrRootSyslibCfg->coreId;
    strcpy (sysConfig.rmClient, ptrRootSyslibCfg->rmClient);
    strcpy (sysConfig.rmServer, ptrRootSyslibCfg->rmServer);
    sysConfig.malloc                            = ptrDomainSyslibCfg->resmgrOsalFxnTable.malloc;
    sysConfig.free                              = ptrDomainSyslibCfg->resmgrOsalFxnTable.free;
    sysConfig.mallocMemoryRegion                = ptrDomainSyslibCfg->resmgrOsalFxnTable.mallocMemoryRegion;
    sysConfig.freeMemoryRegion                  = ptrDomainSyslibCfg->resmgrOsalFxnTable.freeMemoryRegion;
    sysConfig.createSem                         = ptrDomainSyslibCfg->resmgrOsalFxnTable.createSem;
    sysConfig.deleteSem                         = ptrDomainSyslibCfg->resmgrOsalFxnTable.deleteSem;
    sysConfig.postSem                           = ptrDomainSyslibCfg->resmgrOsalFxnTable.postSem;
    sysConfig.pendSem                           = ptrDomainSyslibCfg->resmgrOsalFxnTable.pendSem;
    sysConfig.beginMemAccess                    = ptrDomainSyslibCfg->resmgrOsalFxnTable.beginMemAccess;
    sysConfig.endMemAccess                      = ptrDomainSyslibCfg->resmgrOsalFxnTable.endMemAccess;
#endif

    /* Initialize the system configuration. */
    ptrSyslibDomain->handleSysCfg = Resmgr_init(&sysConfig, errCode);
    if (ptrSyslibDomain->handleSysCfg == NULL)
	{
	    Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: SYSRM initialization failed with error code %d\n", *errCode);

        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
	    return NULL;
    }
    Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                       "Debug: SYSRM initialized successfully [Handle %p]\n", ptrSyslibDomain->handleSysCfg);

    /* Process the configuration: */
    if (Resmgr_processConfig (ptrSyslibDomain->handleSysCfg, &ptrDomainSyslibCfg->domainResourceCfg, errCode) < 0)
	{
	    Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: SYSRM configuration failed with error code %d\n", *errCode);

        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
	    return NULL;
    }

    /* Initialize the PKTLIB instance configuration. */
    memset ((void *)&pktlibInstCfg, 0, sizeof(Pktlib_InstCfg));

    /* Populate the PKTLIB Instance configuration: */
    pktlibInstCfg.instanceId        = 1;
    pktlibInstCfg.databaseHandle    = ptrSyslibDomain->databaseHandle;
    pktlibInstCfg.sysCfgHandle      = ptrSyslibDomain->handleSysCfg;
    pktlibInstCfg.malloc            = ptrDomainSyslibCfg->pktlibOsalFxnTable.malloc;
    pktlibInstCfg.free              = ptrDomainSyslibCfg->pktlibOsalFxnTable.free;
    pktlibInstCfg.beginMemAccess    = ptrDomainSyslibCfg->pktlibOsalFxnTable.beginMemAccess;
    pktlibInstCfg.endMemAccess      = ptrDomainSyslibCfg->pktlibOsalFxnTable.endMemAccess;
    pktlibInstCfg.beginPktAccess    = ptrDomainSyslibCfg->pktlibOsalFxnTable.beginPktAccess;
    pktlibInstCfg.endPktAccess      = ptrDomainSyslibCfg->pktlibOsalFxnTable.endPktAccess;
    pktlibInstCfg.enterCS           = ptrDomainSyslibCfg->pktlibOsalFxnTable.enterCS;
    pktlibInstCfg.exitCS            = ptrDomainSyslibCfg->pktlibOsalFxnTable.exitCS;
#ifdef __ARMv7
    pktlibInstCfg.phyToVirt         = ptrDomainSyslibCfg->pktlibOsalFxnTable.phyToVirt;
#endif

    /* Create the PKTLIB instance */
    ptrSyslibDomain->appPktlibInstanceHandle = Pktlib_createInstance (&pktlibInstCfg, errCode);
    if (ptrSyslibDomain->appPktlibInstanceHandle == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: Creating the PKTLIB instance failed [Error code %d]\n", *errCode);

        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
        return NULL;
    }

    /* Create the heaps used by the SYSLIB services. */
    if (Domain_createPktlibHeaps(ptrSyslibDomain, &ptrDomainSyslibCfg->domainResourceCfg, errCode) < 0)
    {
        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
        return NULL;
    }

    /* Initialize the MSGCOM instance. */
    memset ((void *)&msgcomInstCfg, 0, sizeof(Msgcom_InstCfg));

    /* Populate the MSGCOM Instance configuration */
    msgcomInstCfg.databaseHandle    = ptrSyslibDomain->databaseHandle;
    msgcomInstCfg.sysCfgHandle      = ptrSyslibDomain->handleSysCfg;
    msgcomInstCfg.pktlibInstHandle  = ptrSyslibDomain->appPktlibInstanceHandle;
    msgcomInstCfg.malloc            = ptrDomainSyslibCfg->msgcomOsalFxnTable.malloc;
    msgcomInstCfg.free              = ptrDomainSyslibCfg->msgcomOsalFxnTable.free;
    msgcomInstCfg.registerIsr       = ptrDomainSyslibCfg->msgcomOsalFxnTable.registerIsr;
    msgcomInstCfg.deregisterIsr     = ptrDomainSyslibCfg->msgcomOsalFxnTable.deregisterIsr;
    msgcomInstCfg.disableSysInt     = ptrDomainSyslibCfg->msgcomOsalFxnTable.disableSysInt;
    msgcomInstCfg.enableSysInt      = ptrDomainSyslibCfg->msgcomOsalFxnTable.enableSysInt;
    msgcomInstCfg.enterCS           = ptrDomainSyslibCfg->msgcomOsalFxnTable.enterCS;
    msgcomInstCfg.exitCS            = ptrDomainSyslibCfg->msgcomOsalFxnTable.exitCS;
    msgcomInstCfg.createSem         = ptrDomainSyslibCfg->msgcomOsalFxnTable.createSem;
    msgcomInstCfg.deleteSem         = ptrDomainSyslibCfg->msgcomOsalFxnTable.deleteSem;
    msgcomInstCfg.postSem           = ptrDomainSyslibCfg->msgcomOsalFxnTable.postSem;
    msgcomInstCfg.pendSem           = ptrDomainSyslibCfg->msgcomOsalFxnTable.pendSem;

    /* Create the MSGCOM instance */
    ptrSyslibDomain->appMsgcomInstanceHandle = Msgcom_createInstance (&msgcomInstCfg, errCode);
    if (ptrSyslibDomain->appMsgcomInstanceHandle == NULL)
    {
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error: MSGCOM Instance creation failed [Error code %d]\n", *errCode);

        /* Cleanup the domain. Use a temporary error code to prevent error code override */
        Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
        return NULL;
    }

#ifndef __ARMv7
    /* DSP Realm: Name Proxy need to be created only on the DSP */
    if (ptrRootSyslibCfg->dspSyslibConfig.instantiateNameProxy)
    {
        Name_ProxyCfg proxyCfg;

        /* Initialize the proxy configuration */
        memset ((void *)&proxyCfg, 0, sizeof(Name_ProxyCfg));

        /* Populate the Agent Server configuration: */
        strcpy(proxyCfg.proxyName, ptrRootSyslibCfg->proxyName);
        proxyCfg.realm            	                = Name_ExecutionRealm_DSP;
        proxyCfg.databaseHandle                     = ptrSyslibDomain->databaseHandle;
        proxyCfg.localFlowId      	                = ptrRootSyslibCfg->dspSyslibConfig.localFlowId;
        proxyCfg.remoteFlowId     	                = ptrRootSyslibCfg->dspSyslibConfig.remoteFlowId;
        proxyCfg.sharedMemoryAddress                = ptrRootSyslibCfg->dspSyslibConfig.nameProxySharedMemAddress;
        proxyCfg.pktlibInstHandle				    = ptrSyslibDomain->appPktlibInstanceHandle;
        proxyCfg.proxyHeapHandle 	                = ptrSyslibDomain->proxyHeapHandle;
        proxyCfg.malloc                             = ptrDomainSyslibCfg->nameOsalFxnTable.malloc;
        proxyCfg.free                               = ptrDomainSyslibCfg->nameOsalFxnTable.free;
        proxyCfg.beginMemAccess                     = ptrDomainSyslibCfg->nameOsalFxnTable.beginMemAccess;
        proxyCfg.endMemAccess                       = ptrDomainSyslibCfg->nameOsalFxnTable.endMemAccess;
        proxyCfg.u.dspCfg.clientProxyHeapHandle     = ptrSyslibDomain->sharedProxyClientHeap;
        proxyCfg.u.dspCfg.msgcomInstHandle          = ptrSyslibDomain->appMsgcomInstanceHandle;

        /* Create the Agent Server: */
        ptrSyslibDomain->nameProxyHandle = Name_initProxy (&proxyCfg, errCode);
        if (ptrSyslibDomain->nameProxyHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: Unable to create the name proxy [Error code %d]\n", *errCode);

            /* Cleanup the domain. Use a temporary error code to prevent error code override */
            Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Name Proxy %p has been created successfully\n", ptrSyslibDomain->nameProxyHandle);

        /* Synchronization: Loop around and ensure that the the name proxy has synchronized with its peer. */
        while (1)
        {
            int32_t     synchStatus;

            /* Get the synchronization status */
            synchStatus = Name_isProxySynched (ptrSyslibDomain->nameProxyHandle, errCode);
            if (synchStatus < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: Name proxy synchronization failed[Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            if (synchStatus == 1)
                break;
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: Name Proxy have been synchronized\n");
    }

    /* Do we need to instantiate the Name client on the DSP? The Name Proxy and clients on the DSP use
     * a shared memory heap for communication. This heap is created by the DSP core executing the name
     * servers and all the name clients synchronize on that specific heap */
    if (ptrRootSyslibCfg->nameClientConfig.instantiateNameClient)
    {
        /* SYNC Point: We need to wait for the name proxy to create the heap which is used
         * to exchange messages between the proxy and client */
        char    heapName[PKTLIB_MAX_CHAR];

        /* Construct the message heap name */
        snprintf(heapName, PKTLIB_MAX_CHAR, "%s-ProxyClientHeap", ptrSyslibDomain->cfg.rootSyslibCfg.proxyName);

        /* Loop around till the heap is created. */
        while (1)
        {
            ptrSyslibDomain->sharedProxyClientHeap = Pktlib_findHeapByName (ptrSyslibDomain->appPktlibInstanceHandle,
                                                                            &heapName[0], errCode);
            if (ptrSyslibDomain->sharedProxyClientHeap != NULL)
                break;
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Initialize the name client configuration block. */
        memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

        /* Populate the agent client configuration. */
        strcpy (clientCfg.proxyName,  ptrRootSyslibCfg->proxyName);
        strcpy (clientCfg.clientName, ptrRootSyslibCfg->name);
        clientCfg.realm                        = Name_ExecutionRealm_DSP;
        clientCfg.databaseHandle               = Name_getDatabaseHandle(ptrRootSyslibCfg->nrInstanceId);
        clientCfg.u.dspCfg.pktlibInstHandle    = ptrSyslibDomain->appPktlibInstanceHandle;
        clientCfg.u.dspCfg.clientHeapHandle    = ptrSyslibDomain->sharedProxyClientHeap;
        clientCfg.u.dspCfg.msgcomInstHandle    = ptrSyslibDomain->appMsgcomInstanceHandle;
        clientCfg.malloc                       = ptrDomainSyslibCfg->nameOsalFxnTable.malloc;
        clientCfg.free                         = ptrDomainSyslibCfg->nameOsalFxnTable.free;
        clientCfg.beginMemAccess               = ptrDomainSyslibCfg->nameOsalFxnTable.beginMemAccess;
        clientCfg.endMemAccess                 = ptrDomainSyslibCfg->nameOsalFxnTable.endMemAccess;
        clientCfg.enterCS                      = ptrDomainSyslibCfg->nameOsalFxnTable.enterCS;
        clientCfg.exitCS                       = ptrDomainSyslibCfg->nameOsalFxnTable.exitCS;
        clientCfg.createSem                    = ptrDomainSyslibCfg->nameOsalFxnTable.createSem;
        clientCfg.deleteSem                    = ptrDomainSyslibCfg->nameOsalFxnTable.deleteSem;
        clientCfg.postSem                      = ptrDomainSyslibCfg->nameOsalFxnTable.postSem;
        clientCfg.pendSem                      = ptrDomainSyslibCfg->nameOsalFxnTable.pendSem;

        /* Name client are only created if the proxy is operational */
        while (1)
        {
            /* Initialize the name client. */
            ptrSyslibDomain->nameClientHandle = Name_initClient (&clientCfg, errCode);
            if (ptrSyslibDomain->nameClientHandle != NULL)
                break;

            /* Error: Unable to create the client; use the error code to determine the reason for the failure. */
            if (*errCode != NAME_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                    "Error: Unable to create the name client [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            /* Server was not operational. Wait for some time and try again. */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Client is now operational. */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Name Client %p has been created successfully\n", ptrSyslibDomain->nameClientHandle);
    }

    /* In the DSP realm: Launch the agent execution task if either of the following conditions are met:
     *  - DSP core is configured to execute the agent server
     *  - DSP core is configured to execute the agent client */
    if ((ptrRootSyslibCfg->nameClientConfig.instantiateNameClient == 1) ||
        (ptrRootSyslibCfg->dspSyslibConfig.instantiateNameProxy   == 1))
    {
        /* Launch the Agent execution task: */
        ptrSyslibDomain->nameProxyClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_nameExecutionTask,
                                                                                                        Domain_TaskContext_NAME,
                                                                                                        (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB agent services are not active */
        ptrSyslibDomain->nameOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#else
    /* ARM Realm: Name Proxy do NOT need to be created since the name proxy executes as a seperate process
     * However name clients need to be instantiated in the context of each domain (process) */
    if (ptrRootSyslibCfg->nameClientConfig.instantiateNameClient)
    {
        /* Initialize the name client configuration block. */
        memset ((void *)&clientCfg, 0, sizeof(Name_ClientCfg));

        /* Populate the name client configuration. */
        strcpy (clientCfg.proxyName,  ptrRootSyslibCfg->proxyName);
        strcpy (clientCfg.clientName, ptrRootSyslibCfg->name);
        clientCfg.realm               = Name_ExecutionRealm_ARM;
        clientCfg.databaseHandle      = Name_getDatabaseHandle(ptrRootSyslibCfg->nrInstanceId);
        clientCfg.malloc              = ptrDomainSyslibCfg->nameOsalFxnTable.malloc;
        clientCfg.free                = ptrDomainSyslibCfg->nameOsalFxnTable.free;
        clientCfg.beginMemAccess      = ptrDomainSyslibCfg->nameOsalFxnTable.beginMemAccess;
        clientCfg.endMemAccess        = ptrDomainSyslibCfg->nameOsalFxnTable.endMemAccess;
        clientCfg.enterCS             = ptrDomainSyslibCfg->nameOsalFxnTable.enterCS;
        clientCfg.exitCS              = ptrDomainSyslibCfg->nameOsalFxnTable.exitCS;
        clientCfg.createSem           = ptrDomainSyslibCfg->nameOsalFxnTable.createSem;
        clientCfg.deleteSem           = ptrDomainSyslibCfg->nameOsalFxnTable.deleteSem;
        clientCfg.postSem             = ptrDomainSyslibCfg->nameOsalFxnTable.postSem;
        clientCfg.pendSem             = ptrDomainSyslibCfg->nameOsalFxnTable.pendSem;

        /* Name client are only created if the proxy is operational */
        while (1)
        {
            /* Initialize the name client. */
            ptrSyslibDomain->nameClientHandle = Name_initClient (&clientCfg, errCode);
            if (ptrSyslibDomain->nameClientHandle != NULL)
                break;

            /* Error: Unable to create the client; use the error code to determine the reason for the failure. */
            if (*errCode != NAME_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                    "Error: Unable to create the name client [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            /* Server was not operational. Wait for some time and try again. */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Client is now operational. */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Name Client %p has been created successfully\n", ptrSyslibDomain->nameClientHandle);

        /* Launch the name client execution task: */
        ptrSyslibDomain->nameProxyClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_nameExecutionTask,
                                                                                                        Domain_TaskContext_NAME,
                                                                                                        (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB agent services are not active */
        ptrSyslibDomain->nameOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#endif

#ifndef __ARMv7
    /* Do we need to instantiate the NETFP client on the DSP? */
    if (ptrRootSyslibCfg->netfpClientConfig.instantiateNetfpClient)
    {
        /* Initialize the NETFP client configuration block. */
        memset ((void *)&netfpClientConfig, 0, sizeof(Netfp_ClientConfig));

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Instantiating NETFP Client Service\n");

        /* SYNC Point: Loop around till the NETFP Server is operational. */
        while (1)
        {
            /* Try to start the server: The server might not be ready */
            ptrSyslibDomain->netfpServerHandle = Netfp_startServer (ptrSyslibDomain->databaseHandle,
                                                                    ptrSyslibDomain->nameClientHandle,
                                                                    ptrRootSyslibCfg->netfpClientConfig.serverName,
                                                                    errCode);
            if (ptrSyslibDomain->netfpServerHandle != NULL)
                break;

            /* Check the error code. */
            if (*errCode != NETFP_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: NETFP Starting server failed [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Server Synchronized\n");

        /* Configure the NETFP client configuration block.  */
        strcpy (netfpClientConfig.serverName, ptrRootSyslibCfg->netfpClientConfig.serverName);
        strcpy (netfpClientConfig.clientName, ptrRootSyslibCfg->netfpClientConfig.clientName);
        netfpClientConfig.nrInstanceId                       = ptrRootSyslibCfg->nrInstanceId;
        netfpClientConfig.pktlibInstHandle                   = ptrSyslibDomain->appPktlibInstanceHandle;
        netfpClientConfig.directInterruptCfg.queuePendQueue  = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].queue;
        netfpClientConfig.directInterruptCfg.cpIntcId        = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].cpIntcId;
        netfpClientConfig.directInterruptCfg.systemInterrupt = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].systemInterrupt;
        netfpClientConfig.directInterruptCfg.hostInterrupt   = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].hostInterrupt;
        netfpClientConfig.clientHeapHandle                   = ptrSyslibDomain->netfpClientSrvHeap;
        netfpClientConfig.msgcomInstHandle                   = ptrSyslibDomain->appMsgcomInstanceHandle;
        netfpClientConfig.serverHandle                       = ptrSyslibDomain->netfpServerHandle;
        netfpClientConfig.nameClientHandle                   = ptrSyslibDomain->nameClientHandle;
        netfpClientConfig.netHeaderHeapHandle                = ptrSyslibDomain->netfpHeaderHeap;
        netfpClientConfig.fragmentHeap				         = ptrSyslibDomain->netfpFragHeap;
        netfpClientConfig.malloc                             = ptrDomainSyslibCfg->netfpOsalFxnTable.malloc;
        netfpClientConfig.free                               = ptrDomainSyslibCfg->netfpOsalFxnTable.free;
        netfpClientConfig.beginMemAccess                     = ptrDomainSyslibCfg->netfpOsalFxnTable.beginMemAccess;
        netfpClientConfig.endMemAccess                       = ptrDomainSyslibCfg->netfpOsalFxnTable.endMemAccess;
        netfpClientConfig.enterCS                            = ptrDomainSyslibCfg->netfpOsalFxnTable.enterCS;
        netfpClientConfig.exitCS                             = ptrDomainSyslibCfg->netfpOsalFxnTable.exitCS;
        netfpClientConfig.createSem                          = ptrDomainSyslibCfg->netfpOsalFxnTable.createSem;
        netfpClientConfig.deleteSem                          = ptrDomainSyslibCfg->netfpOsalFxnTable.deleteSem;
        netfpClientConfig.postSem                            = ptrDomainSyslibCfg->netfpOsalFxnTable.postSem;
        netfpClientConfig.pendSem                            = ptrDomainSyslibCfg->netfpOsalFxnTable.pendSem;
#ifdef __ARMv7
        netfpClientConfig.realm                              = Netfp_ExecutionRealm_ARM;
#else
        netfpClientConfig.realm                              = Netfp_ExecutionRealm_DSP;
#endif
        /* Create the NETFP Client. */
        ptrSyslibDomain->netfpClientHandle = Netfp_initClient (&netfpClientConfig, errCode);
        if (ptrSyslibDomain->netfpClientHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: NETFP Client Initialization failed [Error code %d]\n", *errCode);

            /* Cleanup the domain. Use a temporary error code to prevent error code override */
            Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: NETFP Client [%s] Initialized %p\n",
                           netfpClientConfig.clientName, ptrSyslibDomain->netfpClientHandle);

        /* Loop around and synchronize the client registration with the server. */
        while (1)
        {
            /* Get the client status. */
            clientStatus = Netfp_startClient (ptrSyslibDomain->netfpClientHandle, errCode);
            if (clientStatus < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: NETFP Client registration status failed [Error code %d]\n",
                                   *errCode);
                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }

            /* If the client has been registered; we can proceed */
            if (clientStatus == 1)
                break;

            /* Client has not been registered; wait for some time and try again */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: NETFP Client [%s] registered successfully with the server\n",
                           netfpClientConfig.clientName);

        /* Launch the NETFP client execution task: */
        ptrSyslibDomain->netfpClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_netfpClientExecutionTask,
                                                                                                    Domain_TaskContext_NETFP,
                                                                                                    (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB NETFP services are not available. */
        ptrSyslibDomain->netfpOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#else
    /* Do we need to instantiate the NETFP client on the ARM? */
    if (ptrRootSyslibCfg->netfpClientConfig.instantiateNetfpClient)
    {
        /* Initialize the NETFP client configuration block. */
        memset ((void *)&netfpClientConfig, 0, sizeof(Netfp_ClientConfig));

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: Instantiating NETFP Client Service\n");

        /* SYNC Point: Loop around till the NETFP Server is operational. */
        while (1)
        {
            /* Try to start the server: The server might not be ready; On ARM the NETFP Server is executed in
             * the ARM domain itself. */
            ptrSyslibDomain->netfpServerHandle = Netfp_startServer (ptrSyslibDomain->databaseHandle,
                                                                    NULL,
                                                                    ptrRootSyslibCfg->netfpClientConfig.serverName,
                                                                    errCode);
            if (ptrSyslibDomain->netfpServerHandle != NULL)
                break;

            /* Check the error code. */
            if (*errCode != NETFP_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: NETFP Starting server failed [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: NETFP Server Synchronized\n");

        /* Configure the NETFP client configuration block.
         * - The NETFP server executes in the same realm as the server so we dont need to specify
         *   the NAME Client handle to cross the realm boundary. */
        strcpy (netfpClientConfig.serverName, ptrRootSyslibCfg->netfpClientConfig.serverName);
        strcpy (netfpClientConfig.clientName, ptrRootSyslibCfg->netfpClientConfig.clientName);
        netfpClientConfig.nrInstanceId                       = ptrRootSyslibCfg->nrInstanceId;
        netfpClientConfig.pktlibInstHandle                   = ptrSyslibDomain->appPktlibInstanceHandle;
        netfpClientConfig.directInterruptCfg.queuePendQueue  = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].queue;
        netfpClientConfig.directInterruptCfg.cpIntcId        = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].cpIntcId;
        netfpClientConfig.directInterruptCfg.systemInterrupt = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].systemInterrupt;
        netfpClientConfig.directInterruptCfg.hostInterrupt   = ptrDomainSyslibCfg->domainResourceCfg.qPendResponse[0].hostInterrupt;
        netfpClientConfig.clientHeapHandle                   = ptrSyslibDomain->netfpClientSrvHeap;
        netfpClientConfig.msgcomInstHandle                   = ptrSyslibDomain->appMsgcomInstanceHandle;
        netfpClientConfig.serverHandle                       = ptrSyslibDomain->netfpServerHandle;
        netfpClientConfig.nameClientHandle                   = NULL;
        netfpClientConfig.netHeaderHeapHandle                = ptrSyslibDomain->netfpHeaderHeap;
        netfpClientConfig.fragmentHeap				         = ptrSyslibDomain->netfpFragHeap;
        netfpClientConfig.malloc                             = ptrDomainSyslibCfg->netfpOsalFxnTable.malloc;
        netfpClientConfig.free                               = ptrDomainSyslibCfg->netfpOsalFxnTable.free;
        netfpClientConfig.beginMemAccess                     = ptrDomainSyslibCfg->netfpOsalFxnTable.beginMemAccess;
        netfpClientConfig.endMemAccess                       = ptrDomainSyslibCfg->netfpOsalFxnTable.endMemAccess;
        netfpClientConfig.enterCS                            = ptrDomainSyslibCfg->netfpOsalFxnTable.enterCS;
        netfpClientConfig.exitCS                             = ptrDomainSyslibCfg->netfpOsalFxnTable.exitCS;
        netfpClientConfig.createSem                          = ptrDomainSyslibCfg->netfpOsalFxnTable.createSem;
        netfpClientConfig.deleteSem                          = ptrDomainSyslibCfg->netfpOsalFxnTable.deleteSem;
        netfpClientConfig.postSem                            = ptrDomainSyslibCfg->netfpOsalFxnTable.postSem;
        netfpClientConfig.pendSem                            = ptrDomainSyslibCfg->netfpOsalFxnTable.pendSem;
        netfpClientConfig.realm                              = Netfp_ExecutionRealm_ARM;

        /* Create the NETFP Client. */
        ptrSyslibDomain->netfpClientHandle = Netfp_initClient (&netfpClientConfig, errCode);
        if (ptrSyslibDomain->netfpClientHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: NETFP Client Initialization failed [Error code %d]\n", *errCode);

            /* Cleanup the domain. Use a temporary error code to prevent error code override */
            Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: NETFP Client [%s] Initialized %p\n",
                           netfpClientConfig.clientName, ptrSyslibDomain->netfpClientHandle);

        /* Loop around and synchronize the client registration with the server. */
        while (1)
        {
            /* Get the client status. */
            clientStatus = Netfp_startClient (ptrSyslibDomain->netfpClientHandle, errCode);
            if (clientStatus < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: NETFP Client registration status failed [Error code %d]\n",
                                   *errCode);
                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }

            /* If the client has been registered; we can proceed */
            if (clientStatus == 1)
                break;

            /* Client has not been registered; wait for some time and try again */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: NETFP Client [%s] registered successfully with the server\n",
                           netfpClientConfig.clientName);

        /* Launch the NETFP client execution task: */
        ptrSyslibDomain->netfpClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_netfpClientExecutionTask,
                                                                                                    Domain_TaskContext_NETFP,
                                                                                                    (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB NETFP services are not available. */
        ptrSyslibDomain->netfpOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#endif

#ifndef __ARMv7
    /* Do we need to instantiate the DAT client on the DSP? */
    if (ptrRootSyslibCfg->datClientConfig.instantiateDatClient)
    {
        /* SYNC Point: Ensure that the DAT Server has been started. This is required before
         * we create and register the DAT client. */
        while (1)
        {
            /* Try to start the server: The server might not be ready */
            if (Dat_startServer (ptrSyslibDomain->databaseHandle, ptrSyslibDomain->nameClientHandle,
                                 (char*)ptrRootSyslibCfg->datClientConfig.serverName,
                                 errCode) != NULL)
                break;

            /* Check the error code. */
            if (*errCode != DAT_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: DAT Starting server failed [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: DAT Server Synchronized\n");

        /* Initialize the DAT client configuration */
        memset ((void *)&datClientCfg, 0, sizeof(Dat_ClientCfg));

        /* Populate the DAT client configuration */
        strcpy (datClientCfg.serverName, ptrRootSyslibCfg->datClientConfig.serverName);
        strcpy (datClientCfg.clientName, ptrRootSyslibCfg->datClientConfig.clientName);
        datClientCfg.pktlibInstHandle  = ptrSyslibDomain->appPktlibInstanceHandle;
        datClientCfg.msgcomInstHandle  = ptrSyslibDomain->appMsgcomInstanceHandle;
        datClientCfg.databaseHandle    = ptrSyslibDomain->databaseHandle;
        datClientCfg.clientHeapHandle  = ptrSyslibDomain->datClientHeap;
        datClientCfg.nameClientHandle  = ptrSyslibDomain->nameClientHandle;
        datClientCfg.id                = ptrRootSyslibCfg->datClientConfig.datClientId;
        datClientCfg.malloc            = ptrDomainSyslibCfg->datOsalFxnTable.malloc;
        datClientCfg.free              = ptrDomainSyslibCfg->datOsalFxnTable.free;
        datClientCfg.mallocLocal       = ptrDomainSyslibCfg->datOsalFxnTable.mallocLocal;
        datClientCfg.freeLocal         = ptrDomainSyslibCfg->datOsalFxnTable.freeLocal;
        datClientCfg.beginMemAccess    = ptrDomainSyslibCfg->datOsalFxnTable.beginMemAccess;
        datClientCfg.endMemAccess      = ptrDomainSyslibCfg->datOsalFxnTable.endMemAccess;
        datClientCfg.createSem         = ptrDomainSyslibCfg->datOsalFxnTable.createSem;
        datClientCfg.deleteSem         = ptrDomainSyslibCfg->datOsalFxnTable.deleteSem;
        datClientCfg.postSem           = ptrDomainSyslibCfg->datOsalFxnTable.postSem;
        datClientCfg.pendSem           = ptrDomainSyslibCfg->datOsalFxnTable.pendSem;
        datClientCfg.enterCS           = ptrDomainSyslibCfg->datOsalFxnTable.enterCS;
        datClientCfg.exitCS            = ptrDomainSyslibCfg->datOsalFxnTable.exitCS;
#ifdef __ARMv7
        datClientCfg.realm             = Dat_ExecutionRealm_ARM;
#else
        datClientCfg.realm             = Dat_ExecutionRealm_DSP;
#endif
        /* Initialize the DAT client. */
        ptrSyslibDomain->datClientHandle = Dat_initClient (&datClientCfg, errCode);
        if (ptrSyslibDomain->datClientHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: DAT Client initialization failed [Error code %d]\n",
                               *errCode);

            /* Cleanup the domain. Use a temporary error code to prevent error code override */
            Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: DAT Client [%s] initialized %p\n",
                           ptrRootSyslibCfg->datClientConfig.clientName);

        /* Start the DAT client: DAT clients can only be started after they have been registered
         * by the server */
        while (1)
        {
            /* Start the DAT client */
            clientStatus = Dat_startClient (ptrSyslibDomain->datClientHandle, errCode);
            if (clientStatus < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: DAT Client registration status failed [Error code %d]\n",
                                   *errCode);
                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }

            /* If the client has been registered; we can proceed */
            if (clientStatus == 1)
                break;

            /* Client has not been registered; wait for some time and try again */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: DAT Client [%s] registered successfully with the server\n",
                           datClientCfg.clientName);

        /* Launch the DAT client execution task: */
        ptrSyslibDomain->datClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_datClientExecutionTask,
                                                                                                  Domain_TaskContext_DAT,
                                                                                                  (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB DAT services are not available. */
        ptrSyslibDomain->datOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#else
    /* Do we need to instantiate the DAT client on the ARM? */
    if (ptrRootSyslibCfg->datClientConfig.instantiateDatClient)
    {
        /* SYNC Point: Ensure that the DAT Server has been started. This is required before
         * we create and register the DAT client. */
        while (1)
        {
            /* Try to start the server: The server might not be ready. The DAT Server executes
             * on ARM and so there is no need to have a NAME client to cross the execution realms. */
            if (Dat_startServer (ptrSyslibDomain->databaseHandle, NULL,
                                 (char*)ptrRootSyslibCfg->datClientConfig.serverName,
                                 errCode) != NULL)
                break;

            /* Check the error code. */
            if (*errCode != DAT_ENOTREADY)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: DAT Starting server failed [Error code %d]\n", *errCode);

                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }

        /* Debug Message: */
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: DAT Server Synchronized\n");

        /* Initialize the DAT client configuration */
        memset ((void *)&datClientCfg, 0, sizeof(Dat_ClientCfg));

        /* Populate the DAT client configuration
         * - Since the ARM DAT client and DAT server are on the same realm there is no need to
         *   specify the NAME client handle */
        strcpy (datClientCfg.serverName, ptrRootSyslibCfg->datClientConfig.serverName);
        strcpy (datClientCfg.clientName, ptrRootSyslibCfg->datClientConfig.clientName);
        datClientCfg.pktlibInstHandle  = ptrSyslibDomain->appPktlibInstanceHandle;
        datClientCfg.msgcomInstHandle  = ptrSyslibDomain->appMsgcomInstanceHandle;
        datClientCfg.databaseHandle    = ptrSyslibDomain->databaseHandle;
        datClientCfg.clientHeapHandle  = ptrSyslibDomain->datClientHeap;
        datClientCfg.nameClientHandle  = NULL;
        datClientCfg.id                = ptrRootSyslibCfg->datClientConfig.datClientId;
        datClientCfg.malloc            = ptrDomainSyslibCfg->datOsalFxnTable.malloc;
        datClientCfg.free              = ptrDomainSyslibCfg->datOsalFxnTable.free;
        datClientCfg.mallocLocal       = ptrDomainSyslibCfg->datOsalFxnTable.mallocLocal;
        datClientCfg.freeLocal         = ptrDomainSyslibCfg->datOsalFxnTable.freeLocal;
        datClientCfg.beginMemAccess    = ptrDomainSyslibCfg->datOsalFxnTable.beginMemAccess;
        datClientCfg.endMemAccess      = ptrDomainSyslibCfg->datOsalFxnTable.endMemAccess;
        datClientCfg.createSem         = ptrDomainSyslibCfg->datOsalFxnTable.createSem;
        datClientCfg.deleteSem         = ptrDomainSyslibCfg->datOsalFxnTable.deleteSem;
        datClientCfg.postSem           = ptrDomainSyslibCfg->datOsalFxnTable.postSem;
        datClientCfg.pendSem           = ptrDomainSyslibCfg->datOsalFxnTable.pendSem;
        datClientCfg.enterCS           = ptrDomainSyslibCfg->datOsalFxnTable.enterCS;
        datClientCfg.exitCS            = ptrDomainSyslibCfg->datOsalFxnTable.exitCS;
        datClientCfg.realm             = Dat_ExecutionRealm_ARM;

        /* Initialize the DAT client. */
        ptrSyslibDomain->datClientHandle = Dat_initClient (&datClientCfg, errCode);
        if (ptrSyslibDomain->datClientHandle == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                               "Error: DAT Client initialization failed [Error code %d]\n",
                               *errCode);

            /* Cleanup the domain. Use a temporary error code to prevent error code override */
            Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: DAT Client [%s] initialized %p\n",
                           ptrRootSyslibCfg->datClientConfig.clientName);

        /* Start the DAT client: DAT clients can only be started after they have been registered
         * by the server */
        while (1)
        {
            /* Start the DAT client */
            clientStatus = Dat_startClient (ptrSyslibDomain->datClientHandle, errCode);
            if (clientStatus < 0)
            {
                Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                                   "Error: DAT Client registration status failed [Error code %d]\n",
                                   *errCode);
                /* Cleanup the domain. Use a temporary error code to prevent error code override */
                Domain_deinitSyslibServices(ptrSyslibDomain, &tmpErrCode);
                return NULL;
            }

            /* If the client has been registered; we can proceed */
            if (clientStatus == 1)
                break;

            /* Client has not been registered; wait for some time and try again */
            ptrSyslibDomain->cfg.domainOsalFxnTable.taskRelinquish(1);
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG,
                           "Debug: DAT Client [%s] registered successfully with the server\n",
                           datClientCfg.clientName);

        /* Launch the DAT client execution task: */
        ptrSyslibDomain->datClientTaskHandle = ptrDomainSyslibCfg->domainOsalFxnTable.taskCreate (Domain_datClientExecutionTask,
                                                                                                  Domain_TaskContext_DAT,
                                                                                                  (uint32_t)ptrSyslibDomain);
    }
    else
    {
        /* SYSLIB DAT services are not available. */
        ptrSyslibDomain->datOperationalStatus = Domain_OperationStatus_NOT_AVAILABLE;
    }
#endif

    /* Do we need to instantiate the DAT client on the DSP? */
    if (ptrRootSyslibCfg->memlogConfig.instantiateMemlog)
    {
        Memlog_InstCfg              memlogInstConfig;

        /* Initialize and create the MEMLOG instance */
        memset ((void *)&memlogInstConfig, 0, sizeof(Memlog_InstCfg));

        memlogInstConfig.databaseHandle    = ptrSyslibDomain->databaseHandle;
        memlogInstConfig.pktlibInstHandle  = ptrSyslibDomain->appPktlibInstanceHandle;
        memlogInstConfig.msgcomInstHandle  = ptrSyslibDomain->appMsgcomInstanceHandle;
        memlogInstConfig.malloc            = ptrDomainSyslibCfg->memlogOsalFxnTable.malloc;
        memlogInstConfig.free              = ptrDomainSyslibCfg->memlogOsalFxnTable.free;
        memlogInstConfig.enterCS           = ptrDomainSyslibCfg->memlogOsalFxnTable.enterCS;
        memlogInstConfig.exitCS            = ptrDomainSyslibCfg->memlogOsalFxnTable.exitCS;

#ifdef __ARMv7
        memlogInstConfig.realm             = Memlog_ExecutionRealm_ARM;
#else
        memlogInstConfig.realm             = Memlog_ExecutionRealm_DSP;
#endif

        if ((ptrSyslibDomain->memlogInstHandle =  Memlog_createInstance(&memlogInstConfig, errCode)) == NULL)
        {
            Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_ERROR,
                           "Error:  MEMLOG create instance Failed [Error Code %d]\n", *errCode);
            return NULL;
        }
        Domain_logMessage (ptrSyslibDomain, Domain_LogLevel_DEBUG, "Debug: MEMLOG instance has been created successfully\n");
    }

    return (Domain_SyslibHandle)ptrSyslibDomain;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the resource manager handle from the
 *      domain handle. There is no sanity checking done of the argument.
 *      All domain should have a valid resource manager domain handle
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the resource manager
 */
Resmgr_SysCfgHandle Domain_getSysCfgHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->handleSysCfg;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the named resource handle from the
 *      domain handle. There is no sanity checking done of the argument.
 *      All domain should have a valid named resource instance handle
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the named resource instance associated with the domain
 */
Name_DBHandle Domain_getDatabaseHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->databaseHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the PKTLIB handle from the domain handle.
 *      There is no sanity checking done of the argument. All domain should have
 *      a valid PKTLIB instance handle
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the PKTLIB instance associated with the domain
 */
Pktlib_InstHandle Domain_getPktlibInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->appPktlibInstanceHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the MSGCOM handle from the domain handle.
 *      There is no sanity checking done of the argument. All domain should have
 *      a valid MSGCOM instance handle
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the MSGCOM instance associated with the domain
 */
Msgcom_InstHandle Domain_getMsgcomInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->appMsgcomInstanceHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the AGENT client handle from the domain handle.
 *      There is no sanity checking done of the argument. Agent client handles need
 *      not exist on each domain
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the Name client instance associated with the domain
 */
Name_ClientHandle Domain_getNameClientInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->nameClientHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the NETFP client handle from the domain handle.
 *      There is no sanity checking done of the argument. NETFP client handles need
 *      not exist on each domain
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the NETFP client instance associated with the domain
 */
Netfp_ClientHandle Domain_getNetfpClientInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->netfpClientHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the DAT client handle from the domain handle.
 *      There is no sanity checking done of the argument. DAT client handles need
 *      not exist on each domain
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the DAT client instance associated with the domain
 */
Dat_ClientHandle Domain_getDatClientInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->datClientHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the MEMLOG handle from the domain handle.
 *      There is no sanity checking done of the argument. MEMLOG client handles need
 *      not exist on each domain
 *
 *  @param[in]  domainHandle
 *      Domain handle
 *
 *  \ingroup DOMAIN_FUNCTION
 *
 *  @retval
 *      Handle to the DAT client instance associated with the domain
 */
Memlog_InstHandle Domain_getMemlogInstanceHandle (Domain_SyslibHandle domainHandle)
{
    return ((Domain_Syslib*)domainHandle)->memlogInstHandle;
}

