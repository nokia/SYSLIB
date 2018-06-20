/**
 *   @file  domain.h
 *
 *   @brief
 *      Domain Header Files
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

#ifndef __DOMAIN_INTERNAL_H__
#define __DOMAIN_INTERNAL_H__

#include <ti/runtime/root/root.h>
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/msgcom/msgcom.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>

/**
 * @brief 
 *  Domain Operational status
 *
 * @details
 *  The structure is used to describe the domain operational status for
 *  the SYSLIB services
 */
typedef enum Domain_OperationStatus
{
    /**
     * @brief  Service is initialized and available.
     */
    Domain_OperationStatus_AVAILABLE        = 0x1,

    /**
     * @brief  Service is not available
     */
    Domain_OperationStatus_NOT_AVAILABLE    = 0x2,

    /**
     * @brief  Deactivate an active service
     */
    Domain_OperationStatus_DEACTIVATE       = 0x3
}Domain_OperationStatus;

/**
 * @brief 
 *  Domain SYSLIB MCB
 *
 * @details
 *  The structure is used to describe the domain platform SYSLIB information.
 */
typedef struct Domain_Syslib
{
    /**
     * @brief  Domain identifier
     */
    uint32_t                            domainId;

    /**
     * @brief  SYSLIB configuration  
     */
    Domain_SyslibCfg                    cfg;

    /**
     * @brief  Resource Manager configuration handle. 
     */
    Resmgr_SysCfgHandle                 handleSysCfg;

    /**
     * @brief  Name Database handle
     */
    Name_DBHandle                       databaseHandle;

    /**
     * @brief  MSGCOM Instance handle. 
     */
    Msgcom_InstHandle                   appMsgcomInstanceHandle;

    /**
     * @brief  PKTLIB Instance handle.
     */
    Pktlib_InstHandle		            appPktlibInstanceHandle;

    /**
     * @brief  This is the name proxy handle instantiated for the domain.
     * This is applicable only if the root slave was configured to create
     * a name proxy else this is set to NULL
     */
    Name_ProxyHandle                    nameProxyHandle;

    /**
     * @brief  This is the name client handle instantiated for the domain.
     * This is applicable only if the root slave was configured to create
     * a name client else this is set to NULL
     */
    Name_ClientHandle                   nameClientHandle;

    /**
     * @brief  Domain name services operational status
     */
    volatile Domain_OperationStatus     nameOperationalStatus;

    /**
     * @brief  This is the NETFP client handle instantiated for the domain.
     * This is applicable only if the root slave was configured to create
     * a NETFP client else this is set to NULL
     */
    Netfp_ClientHandle                  netfpClientHandle;

    /**
     * @brief  This is the NETFP server handle instantiated for the domain.
     * This is applicable for all root slaves which use the NETFP client services
     */
    Netfp_ServerHandle                  netfpServerHandle;

    /**
     * @brief  Domain NETFP services operational status
     */
    volatile Domain_OperationStatus     netfpOperationalStatus;

    /**
     * @brief  This is the heap which is used to exchange messages between the 
     * peer name proxies.
     */
    Pktlib_HeapHandle                   proxyHeapHandle;

    /**
     * @brief  This is the shared heap which is shared between the name proxy and clients and 
     * is used to exchange messages between them
     */
    Pktlib_HeapHandle                   sharedProxyClientHeap;

    /**
     * @brief  This is the NETFP header heap which is used to allocate packets 
     * which are populated with the networking headers.
     */
    Pktlib_HeapHandle                   netfpHeaderHeap;

    /**
     * @brief  This is the NETFP fragment heap which are used to allocate
     * packets used for creating IP fragments.
     */
    Pktlib_HeapHandle                   netfpFragHeap;

    /**
     * @brief  This is the NETFP Client heap which is used to communicate with the 
     * NETFP server executing on ARM.
     */
    Pktlib_HeapHandle                   netfpClientSrvHeap;

    /**
     * @brief  This is the DAT client heap which is used to communicate with the DAT
     * server executing on ARM
     */
    Pktlib_HeapHandle                   datClientHeap;

    /**
     * @brief  This is the Name Proxy Client task handle
     */
    void*                               nameProxyClientTaskHandle;

    /**
     * @brief  This is the NETFP Client Task handle
     */
    void*                               netfpClientTaskHandle;

    /**
     * @brief  This is the DAT Client handle instantiated for the domain
     */
    Dat_ClientHandle                    datClientHandle;

    /**
     * @brief  Domain DAT services operational status
     */
    volatile Domain_OperationStatus     datOperationalStatus;

    /**
     * @brief  This is the DAT Client Task handle
     */
    void*                               datClientTaskHandle;

    /**
     * @brief  MEMLOG instance handle
     */
    Memlog_InstHandle       memlogInstHandle;
}Domain_Syslib;

#endif /* __DOMAIN_INTERNAL_H__ */


