/**
 *   @file  master.c
 *
 *   @brief
 *      The master executes on the ARM and is responsible for the configuration
 *      of the slaves and ensuring that the domain is operational.
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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <malloc.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <limits.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

/* MCSDK include files. */
#include <ti/runtime/root/root.h>
#include <ti/csl/cslr_device.h>

/* SYSLIB include files */
#include <ti/runtime/name/name.h>
#include <ti/runtime/root/root.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/dat/dat.h>
#include <ti/apps/netfp_proxy/netfp_proxy_ipc.h>

/* RAT Master file */
#include "master.h"

/**********************************************************************
 *********************** Global Declarations **************************
 **********************************************************************/

/* Root Master Handle: */
Root_MasterHandle   rootMasterHandle;

/* Application Domain Handles for the DSP Core(s) & ARM processes */
void*   appDomainHandle[16];

/**********************************************************************
 ************************ Root Master Functions ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to allocate memory
 *
 *  @param[in]  numBytes
 *      Number of bytes of memory to be allocated
 *  @param[in]  alignment
 *      Alignment requirements
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
static void* Root_osalMalloc(uint32_t numBytes, uint32_t alignment)
{
    return memalign (alignment, numBytes);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to free memory
 *
 *  @param[in]  ptr
 *      Pointer to the memory to be cleaned up
 *  @param[in]  numBytes
 *      Number of bytes of memory to be cleaned up
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalFree(void* ptr, uint32_t numBytes)
{
    free (ptr);
}

/**
 *  @b Description
 *  @n
 *     The function is used by the root module to create a semaphore
 *
 *  @retval
 *      Opaque critical section handle
 */
static void* Root_osalCreateSem(void)
{
    int32_t     retVal;
    sem_t*      ptrSem;

    /* Allocate memory for a semaphore. */
    ptrSem = malloc (sizeof(sem_t));

    /* Create a semaphore. */
    retVal = sem_init (ptrSem, 0, 0);
    if (retVal < 0)
    {
        free (ptrSem);
        return NULL;
    }
    return (void*)ptrSem;
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to pend on the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalPendSem(void* semHandle)
{
    sem_wait (semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to post the semaphore associated with the job
 *      This is done once a SYNC job is submitted.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalPostSem(void* semHandle)
{
    sem_post (semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to delete a semaphore associated with a JOB.
 *
 *  @param[in]  semHandle
 *      Opaque Semaphore Handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalDeleteSem(void* semHandle)
{
    /* Delete the semaphore. */
    sem_close ((sem_t*)semHandle);
    free (semHandle);
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to enter the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @retval
 *      Opaque critical section handle
 */
static void* Root_osalEnterCS(void)
{
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      OSAL API which is used to exit the critical section. This is required to
 *      protect the ROOT services from multiple threads.
 *
 *  @param[in]  csHandle
 *      Opaque critical section handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Root_osalExitCS(void* csHandle)
{
    return;
}

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to invalidate the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be invalidated
 *  @param[in]  size
 *      Size of the buffer to be invalidated
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalBeginMemoryAccess(void* ptr, uint32_t size)
{
    /* No need to perform cache operations on ARM */
}

/**
 *  @b Description
 *  @n
 *     OSAL API is used by the root module to writeback the contents
 *     of the cache.
 *
 *  @param[in]  ptr
 *      Pointer to the buffer to be written back
 *  @param[in]  size
 *      Size of the buffer to be written back
 *
 *  @retval
 *      Not applicable
 */
static void Root_osalEndMemoryAccess(void* ptr, uint32_t size)
{
    /* No need to perform cache operations on ARM */
}

/**
 *  @b Description
 *  @n
 *      Once the application domain has been created on the root slave the application
 *      uses the following function to inform the root master that the domain is active
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appId
 *      Application Identifier
 *  @param[in]  appDomainHnd
 *      Opaque application domain handle passed back to the root master
 *
 *  @retval
 *     Not applicable
 */
void appUp (uint32_t appId, void* appDomainHnd)
{
    switch (appId)
    {
        case LTE_REL9A_DOMAIN_ID:
        {
            appDomainHandle[0] = appDomainHnd;
            LOG_INFO ("Debug: LTE9A Core0 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9A_DOMAIN_ID+1:
        {
            appDomainHandle[1] = appDomainHnd;
            LOG_INFO ("Debug: LTE9A Core1 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9A_DOMAIN_ID+2:
        {
            appDomainHandle[2] = appDomainHnd;
            LOG_INFO ("Debug: LTE9A Core2 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9A_DOMAIN_ID+3:
        {
            appDomainHandle[3] = appDomainHnd;
            LOG_INFO ("Debug: LTE9A Core3 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9B_DOMAIN_ID:
        {
            appDomainHandle[4] = appDomainHnd;
            LOG_INFO ("Debug: LTE9B Core4 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9B_DOMAIN_ID+1:
        {
            appDomainHandle[5] = appDomainHnd;
            LOG_INFO ("Debug: LTE9B Core5 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9B_DOMAIN_ID+2:
        {
            appDomainHandle[6] = appDomainHnd;
            LOG_INFO ("Debug: LTE9B Core6 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL9B_DOMAIN_ID+3:
        {
            appDomainHandle[7] = appDomainHnd;
            LOG_INFO ("Debug: LTE9B Core7 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D1_L2_DOMAIN_ID:
        {
            appDomainHandle[0] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment1 L2 ARM Process is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D1_L1A_DOMAIN_ID:
        {
            appDomainHandle[1] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment1 L1A DSP Core0 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D1_L1A_DOMAIN_ID+1:
        {
            appDomainHandle[2] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment1 L1A DSP Core1 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D1_L1B_DOMAIN_ID:
        {
            appDomainHandle[3] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment1 L1B DSP Core2 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D1_L1B_DOMAIN_ID+1:
        {
            appDomainHandle[4] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment1 L1B DSP Core3 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L2A_DOMAIN_ID:
        {
            appDomainHandle[0] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core0 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L1A_DOMAIN_ID:
        {
            appDomainHandle[1] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L1A DSP Core1 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L1A_DOMAIN_ID+1:
        {
            appDomainHandle[2] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L1A DSP Core2 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L2A_DOMAIN_ID+1:
        {
            appDomainHandle[3] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core3 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L2B_DOMAIN_ID:
        {
            appDomainHandle[4] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core4 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L1B_DOMAIN_ID:
        {
            appDomainHandle[5] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L1B DSP Core5 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L1B_DOMAIN_ID+1:
        {
            appDomainHandle[6] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L1B DSP Core6 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        case LTE_REL10D2_L2B_DOMAIN_ID+1:
        {
            appDomainHandle[7] = appDomainHnd;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core7 is up [Domain Handle %x]\n", (uint32_t)appDomainHnd);
            break;
        }
        default:
        {
            LOG_ERROR ("Error: Invalid domain identifier received %x\n", appId);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is invoked from the application domain on the root slave to the
 *      root master to inform that the domain on the root slave is not operational
 *      any more. The arguments could be used to pass any application specific
 *      information back to the root master.
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appDomainHandle
 *      Opaque application domain handle which has gone down
 *  @param[in]  arg0
 *      Application specific argument0
 *  @param[in]  arg1
 *      Application specific argument1
 *  @param[in]  arg2
 *      Application specific argument2
 *
 *  @retval
 *     Not applicable
 */
void appDown (void* appDomainHandle, uint32_t arg0, uint32_t arg1, uint32_t arg2)
{
    LOG_INFO ("Debug: Application domain %p is down [%x %x %x]\n", appDomainHandle, arg0, arg1, arg2);
    return;
}

/**
 *  @b Description
 *  @n
 *      The root master will notify the slave to deinitialize themselves. Once the slaves
 *      have deinitialized themselves they will announce this back to the master. This
 *      function is invoked on the master on the slave notification.
 *
 *      The function needs to be implemented on the root *master*
 *
 *  @param[in]  appId
 *      Application Identifier
 *
 *  @retval
 *     Not applicable
 */
void appDeinitialized (uint32_t appId)
{
    switch (appId)
    {
        case LTE_REL9A_DOMAIN_ID:
        {
            appDomainHandle[0] = NULL;
            LOG_INFO ("Debug: LTE9A Core0 has been deinitialized\n");
            break;
        }
        case LTE_REL9A_DOMAIN_ID+1:
        {
            appDomainHandle[1] = NULL;
            LOG_INFO ("Debug: LTE9A Core1 has been deinitialized\n");
            break;
        }
        case LTE_REL9A_DOMAIN_ID+2:
        {
            appDomainHandle[2] = NULL;
            LOG_INFO ("Debug: LTE9A Core2 has been deinitialized\n");
            break;
        }
        case LTE_REL9A_DOMAIN_ID+3:
        {
            appDomainHandle[3] = NULL;
            LOG_INFO ("Debug: LTE9A Core3 has been deinitialized\n");
            break;
        }
        case LTE_REL9B_DOMAIN_ID:
        {
            appDomainHandle[4] = NULL;
            LOG_INFO ("Debug: LTE9B Core4 has been deinitialized\n");
            break;
        }
        case LTE_REL9B_DOMAIN_ID+1:
        {
            appDomainHandle[5] = NULL;
            LOG_INFO ("Debug: LTE9B Core5 has been deinitialized\n");
            break;
        }
        case LTE_REL9B_DOMAIN_ID+2:
        {
            appDomainHandle[6] = NULL;
            LOG_INFO ("Debug: LTE9B Core6 has been deinitialized\n");
            break;
        }
        case LTE_REL9B_DOMAIN_ID+3:
        {
            appDomainHandle[7] = NULL;
            LOG_INFO ("Debug: LTE9B Core7 has been deinitialized\n");
            break;
        }
        case LTE_REL10D1_L2_DOMAIN_ID:
        {
            appDomainHandle[0] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment1 L2 ARM Process has been deinitialized\n");
            break;
        }
        case LTE_REL10D1_L1A_DOMAIN_ID:
        {
            appDomainHandle[1] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment1 L1A DSP Core0 has been deinitialized\n");
            break;
        }
        case LTE_REL10D1_L1A_DOMAIN_ID+1:
        {
            appDomainHandle[2] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment1 L1A DSP Core1 has been deinitialized\n");
            break;
        }
        case LTE_REL10D1_L1B_DOMAIN_ID:
        {
            appDomainHandle[3] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment1 L1B DSP Core2 has been deinitialized\n");
            break;
        }
        case LTE_REL10D1_L1B_DOMAIN_ID+1:
        {
            appDomainHandle[4] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment1 L1B DSP Core3 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L2A_DOMAIN_ID:
        {
            appDomainHandle[0] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core0 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L1A_DOMAIN_ID:
        {
            appDomainHandle[1] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L1A DSP Core1 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L1A_DOMAIN_ID+1:
        {
            appDomainHandle[2] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L1A DSP Core2 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L2A_DOMAIN_ID+1:
        {
            appDomainHandle[3] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core3 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L2B_DOMAIN_ID:
        {
            appDomainHandle[4] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core4 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L1B_DOMAIN_ID:
        {
            appDomainHandle[5] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L1B DSP Core5 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L1B_DOMAIN_ID+1:
        {
            appDomainHandle[6] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L1B DSP Core6 has been deinitialized\n");
            break;
        }
        case LTE_REL10D2_L2B_DOMAIN_ID+1:
        {
            appDomainHandle[7] = NULL;
            LOG_INFO ("Debug: LTE10 Deployment2 L2A DSP Core7 has been deinitialized\n");
            break;
        }
        default:
        {
            LOG_ERROR ("Error: Invalid domain identifier received %x\n", appId);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Root Master Execution thread.
 *
 *  @retval
 *      Not Applicable.
 */
static void* RootMasterExecution(void* arg)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the root master. */
        if (Root_executeMaster (rootMasterHandle, &errCode) < 0)
            LOG_ERROR ("Error: Root Master execution failed [Error code %d]\n", errCode);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Sets up a basic NetFP Proxy non-secure
 *      policy Offload request.
 *
 *  @param[in]  xid
 *      Unique transaction id to identify this request
 *      to Proxy.
 *  @param[in]  spId
 *      Policy Id to offload
 *  @param[out]  ptrReqMsg
 *      Formatted NetFP Proxy IPC request
 *
 *  @retval
 *      <0      -   Error building Proxy request
 *      0       -   Succesfully setup the request
 */
static int32_t Rat_BuildNonSecureOffloadRequest
(
    uint32_t            xid,
    uint32_t            spId,
    NetfpProxy_msg*     ptrReqMsg
)
{
    /* Validate inputs */
    if (ptrReqMsg == NULL)
        return -1;

    /* Setup a basic non-secure policy offload request */
    memset ((void *)ptrReqMsg, 0, sizeof (NetfpProxy_msg));
    ptrReqMsg->hdr.msgType          =   NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ;
    ptrReqMsg->hdr.transId          =   xid;
    ptrReqMsg->body.spReq.policyId  =   spId;
    ptrReqMsg->body.spReq.bIsSecure =   0;

    /* Done building request */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Sets up a basic NetFP Proxy secure
 *      policy Offload request.
 *
 *  @param[in]  xid
 *      Unique transaction id to identify this request
 *      to Proxy.
 *  @param[in]  spId
 *      Policy Id to offload
 *  @param[in]  bIsShared
 *      Indicates if the policy is shared between
 *      Linux and NetFP. Set to 0 to indicate that
 *      policy once offloaded will be used only for
 *      traffic termination at NetFP. Set to 1 to
 *      indicate that traffic using this policy can
 *      terminate at either Linux/NetFP.
 *  @param[in]  bFragmentOnOuter
 *      Indicates if the tunnel should be configured
 *      to fragment on outer/inner IP by NetFP. Set
 *      to 1, to enable NetFP to fragment on outer IP.
 *      Set to 0 to enable inner IP fragmentation at NetFP.
 *  @param[out]  ptrReqMsg
 *      Formatted NetFP Proxy IPC request
 *
 *  @retval
 *      <0      -   Error building Proxy request
 *      0       -   Succesfully setup the request
 */
static int32_t Rat_BuildSecureOffloadRequest
(
    uint32_t            xid,
    uint32_t            spId,
    uint32_t            bIsShared,
    uint32_t            bFragmentOnOuter,
    NetfpProxy_msg*     ptrReqMsg
)
{
    /* Validate inputs */
    if (ptrReqMsg == NULL)
        return -1;

    /* Setup a basic secure policy offload request */
    memset ((void *)ptrReqMsg, 0, sizeof (NetfpProxy_msg));
    ptrReqMsg->hdr.msgType                  =   NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ;
    ptrReqMsg->hdr.transId                  =   xid;
    ptrReqMsg->body.spReq.policyId          =   spId;
    ptrReqMsg->body.spReq.bIsSecure         =   1;
    ptrReqMsg->body.spReq.bIsSharedPolicy   =   bIsShared;
    ptrReqMsg->body.spReq.fragLevel         =   NETFP_PROXY_IPSEC_FRAG_INNER_IP;

    /* Enable Outer IP fragmentation on this policy */
    if (bFragmentOnOuter)
        ptrReqMsg->body.spReq.fragLevel     =   NETFP_PROXY_IPSEC_FRAG_OUTER_IP;

    /* Done building request */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Processes the start/stop offload response from
 *      Proxy and takes care of setting up/cleaning up
 *      corresponding NR resources.
 *
 *  @param[in]  nameDBHandle
 *      RAT Cell Named database handle
 *  @param[in]  ptrRespMsg
 *      NetFP Proxy Start/Stop Offload response
 *      received.
 *  @param[in]  policyName
 *      Name to identify this policy with in Name database
 *  @retval
 *      <0      -   Error processing the response
 *      0       -   Succesfully parsed the response
 */
static int32_t Rat_ProcessOffloadResponse
(
    Name_DBHandle       nameDBHandle,
    NetfpProxy_msg*     ptrRespMsg,
    char*               policyName
)
{
    Name_ResourceCfg        namedResourceCfg;
    int32_t                 errCode;

    /* Validate input */
    if (ptrRespMsg == NULL)
        return -1;

    switch (ptrRespMsg->hdr.msgType)
    {
        case    NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP:
        {
            if (ptrRespMsg->body.spResp.retVal ==  NETFP_PROXY_IPC_RETVAL_SUCCESS)
            {
                /* Policy offloaded succesfully. Publish the Policy Id in this domain */
                namedResourceCfg.handle1  = (uint32_t)ptrRespMsg->body.spResp.policyId;
                strcpy (namedResourceCfg.name, policyName);

                /* Create & publish the offloaded policy Id. */
                if (Name_createResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                                        &namedResourceCfg, &errCode) < 0)
                {
                    LOG_ERROR ("Error: Failed creating NR for '%s' [Error code %d]\n", namedResourceCfg.name, errCode);
                    return -1;
                }
                else
                {
                    LOG_DEBUG ("Debug: Start offload of '%s' policy with Id: '%d' succesfully done NameDB: 0x%x\n",
                                policyName, ptrRespMsg->body.spResp.policyId, nameDBHandle);
                }
            }
            else
            {
                /* Policy offload failed. Log the event */
                LOG_ERROR ("Error: Failed start offload of '%s' policy with Id: '%d' [Error code %d]\n",
                            policyName, ptrRespMsg->body.spResp.policyId, ptrRespMsg->body.spResp.netfpErrCode);
                return -1;
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP:
        {
            if (ptrRespMsg->body.spResp.retVal ==  NETFP_PROXY_IPC_RETVAL_SUCCESS)
            {
                /* Remove the corresponding NR for the policy from Name database. */
                if (Name_deleteResource (nameDBHandle, Name_ResourceBucket_USER_DEF1,
                                         policyName,  &errCode) < 0)
                {
                    LOG_ERROR ("Error: Failed deleting NR for '%s' [Error code %d]\n", policyName, errCode);
                    return -1;
                }
                else
                {
                    LOG_DEBUG ("Debug: Stop offload of '%s' policy with Id: '%d' succesfully done\n",
                                policyName, ptrRespMsg->body.spResp.policyId);
                }
            }
            else
            {
                /* Policy offload stop failed. Log the event */
                LOG_ERROR ("Error: Failed stop offload of '%s' policy with Id: '%d' [Error code %d]\n",
                            policyName, ptrRespMsg->body.spResp.policyId, ptrRespMsg->body.spResp.netfpErrCode);
                return -1;
            }
            break;
        }
        default:
        {
            /* Unhandled response */
            return -1;
        }
    }

    /* Return success */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Offloads policies to NetFP server using NetFP Proxy IPC interface.
 *
 *  @param[in]  nameDBHandle
 *      RAT cell name database handle
 *  @param[in]  netfpProxyPid
 *      NetFP Proxy PID to identify the Proxy instance
 *      started by Root master.
 *  @param[in]  reqMsg
 *      Offload request to be sent to Proxy
 *  @param[in]  policyName
 *      Name to identify this policy request with
 *  @retval
 *      <0      -   Policy offload failed.
 *      0       -   Succesfully offloaded policy to NetFP.
 */
static int32_t Rat_OffloadPolicy
(
    Name_DBHandle       nameDBHandle,
    pid_t               netfpProxyPid,
    NetfpProxy_msg*     ptrReqMsg,
    char*               policyName
)
{
    NetfpProxy_msg                  respMsg;
    struct sockaddr_un              proxySunAddr;
    struct sockaddr_un              rootSunAddr;
    int32_t                         rootSockFd;
    char                            sunPath[64];
    struct sockaddr_un              fromAddr;
    uint32_t                        fromAddrLen =  sizeof (struct sockaddr_un);
    int32_t                         retVal;

    /* Open a Unix socket to talk to NetFP Proxy daemon */
    if ((rootSockFd = socket (AF_UNIX, SOCK_DGRAM, 0)) < 0)
    {
        LOG_ERROR ("Error: Failed creating unix socket for Root Master [Error code %d]\n", errno);
        return -1;
    }

    /* Setup a socket name for Root master to receive responses from Proxy */
    memset ((void *)&rootSunAddr, 0, sizeof (struct sockaddr_un));
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_USER_PATH, netfpProxyPid);
    strcpy (rootSunAddr.sun_path, sunPath);

    /* Make sure that this is the only application listening on this socket */
    unlink (rootSunAddr.sun_path);

    /* Bind the socket to Root master unix path name */
    rootSunAddr.sun_family    =   AF_UNIX;
    if (bind (rootSockFd, (const struct sockaddr *)&rootSunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        LOG_ERROR ("Error: Root Master bind to %s failed, [Error code %d]\n",
                   sunPath, errno);
        goto cleanup_and_return_error;
    }

    /* Setup the socket name for Proxy on which its listening for Offload requests */
    memset ((void *)&proxySunAddr, 0, sizeof (struct sockaddr_un));
    proxySunAddr.sun_family =   AF_UNIX;
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH, netfpProxyPid);
    strcpy (proxySunAddr.sun_path, sunPath);

    /* Send the offload request to NetFP Proxy daemon */
    if (sendto (rootSockFd, (void *)ptrReqMsg, sizeof (NetfpProxy_msg), 0,
                (const struct sockaddr *)&proxySunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        LOG_ERROR ("Error: Failed offloading policy %d from Root Master [Error code %d]\n",
                    ptrReqMsg->body.spReq.policyId, errno);
        goto cleanup_and_return_error;
    }
    else
    {
        /* Valid response from Proxy. */
        if (recvfrom (rootSockFd, (void *)&respMsg, sizeof (NetfpProxy_msg), 0,
                      (struct sockaddr *)&fromAddr, &fromAddrLen) > 0)
        {
            /* Process the offload response */
            retVal = Rat_ProcessOffloadResponse (nameDBHandle, &respMsg, policyName);
            if (retVal < 0)
                goto cleanup_and_return_error;
        }
        else
        {
            /* Error reading response from the socket */
            LOG_ERROR ("Error: Error reading response from NetFP Proxy \n");
            goto cleanup_and_return_error;
        }
    }

    /* Succesfully setup policy in NetFP */
    close (rootSockFd);
    return 0;

cleanup_and_return_error:
    close (rootSockFd);
    return -1;
}

/**
 *  @b Description
 *  @n
 *      CLI to initiate policy offloads using NETFP Proxy
 *
 *  @param[in]  nameDBHandle
 *      RAT Cell Database handle
 *  @param[in]  netfpProxyPid
 *      Netfp Proxy process identifier
 *
 *  @retval
 *      Not Applicable.
 */
void Rat_DoPolicyOffloads
(
    Name_DBHandle   nameDBHandle,
    pid_t           netfpProxyPid
)
{
    uint32_t            secureMode, spId, sharedMode, enableOuterIPFrag;
    char                policyName [NAME_MAX_CHAR + 1];
    int32_t             retVal;
    static uint32_t     xid;
    NetfpProxy_msg      reqMsg;

    while (1)
    {
        LOG_DEBUG ("**************************************\n");
        LOG_DEBUG ("1. Offload non-secure policies\n");
        LOG_DEBUG ("2. Offload secure policies\n");
        LOG_DEBUG ("3. Done offloading policies\n");
        LOG_DEBUG ("**************************************\n");

        /* Get the user input. */
        LOG_DEBUG ("Enter the selection:");
        scanf("%d", &secureMode);

        if (secureMode == 1)
        {
            /* Offload a non-secure policy */
            LOG_DEBUG ("Enter Policy Id:\n");
            scanf("%d", &spId);
            LOG_DEBUG ("Enter Name to identify this policy:\n");
            scanf("%s", policyName);

            /* Build an offload request to send to NetFP Proxy */
            retVal  =   Rat_BuildNonSecureOffloadRequest (xid++,
                                                          spId,
                                                          &reqMsg);
            if (retVal < 0)
            {
                LOG_ERROR ("Error: Failed to create a non-secure policy request\n");
                continue;
            }
            Rat_OffloadPolicy (nameDBHandle, netfpProxyPid, &reqMsg, policyName);
        }
        else if (secureMode == 2)
        {
            /* Offload secure policy */
            LOG_DEBUG ("Enter Policy Id:\n");
            scanf("%d", &spId);
            LOG_DEBUG ("Enable shared mode?: 0 - No, 1- Yes\n");
            scanf("%d", &sharedMode);
            /* No support for enabling Outer IP fragmentation in NetFP */
#if 0
            LOG_DEBUG ("Enable Outer IP fragmentation?: 0 - No, 1- Yes\n");
            scanf("%d", &enableOuterIPFrag);
#else
            enableOuterIPFrag = 0;
#endif
            LOG_DEBUG ("Enter Name to identify this policy:\n");
            scanf("%s", policyName);

            /* Build an offload request to send to NetFP Proxy */
            retVal  =   Rat_BuildSecureOffloadRequest (xid++,
                                                       spId,
                                                       sharedMode,
                                                       enableOuterIPFrag,
                                                       &reqMsg);
            if (retVal < 0)
            {
                LOG_ERROR ("Error: Failed to create a non-secure policy request\n");
                continue;
            }
            Rat_OffloadPolicy (nameDBHandle, netfpProxyPid, &reqMsg, policyName);
        }
        else if (secureMode == 3)
        {
            /* Done with offloads for this cell. */
            break;
        }
        else
        {
            /* Invalid selection */
            continue;
        }
    }

    /* Done with policy offloads */
    return;
}

/**
 *  @b Description
 *  @n
 *      Initiates policy offloads using NETFP Proxy by reading
 *      from a policy file
 *
 *  @param[in]  nameDBHandle
 *      RAT Cell Database handle
 *  @param[in]  netfpProxyPid
 *      Netfp Proxy process identifier
 *  @retval
 *      Not Applicable.
 */
void Rat_DoPolicyOffloads_FromFile (Name_DBHandle nameDBHandle, char* cellName, pid_t netfpProxyPid)
{
    int32_t             secureMode, spId, sharedMode, enableOuterIPFrag;
    char                policyName [NAME_MAX_CHAR + 1];
    int32_t             retVal;
    static uint32_t     xid;
    NetfpProxy_msg      reqMsg;
    FILE*               fp;
    char                spConfigFileName [128];
    char*               line = NULL;
    size_t              len;
    ssize_t             readLen;
    char*               delimiter = " ";
    char*               token;

    /* Open and read the Policy config file.
     * The Policy config file lists all the policies to be offloaded
     * to NETFP */
    sprintf (spConfigFileName, "/home/root/netfpPolicy%s.conf", cellName);
    if ((fp = fopen (spConfigFileName, "r")) == NULL)
    {
        LOG_ERROR ("Error: Cant open file %s, error: %d\n", spConfigFileName, errno);
        return;
    }

    /* Read and process the Policy configuration file
     *
     * Format for Non-Secure policies:
     * <is_secure_mode=0> <spId> <policy_name>
     *
     * Format for Secure policies:
     * <is_secure_mode=1> <spId> <is_tunnel_shared> <policy_name>*/
    while ((readLen = getline (&line, &len, fp)) != -1)
    {
        token           =   strtok (line, delimiter);
        secureMode      =   atoi (token);

        if (secureMode == 0)
        {
            token           =   strtok (NULL, delimiter);
            spId            =   atoi (token);
            token           =   strtok (NULL, delimiter);
            strncpy (policyName, token, NAME_MAX_CHAR);

            if ((spId <= 0) || policyName[0] == '\0')
            {
                LOG_ERROR ("Invalid Non-IPSec policy configuration \n");
            }

            /* Build an offload request to send to NetFP Proxy */
            retVal  =   Rat_BuildNonSecureOffloadRequest (xid++,
                                                          spId,
                                                          &reqMsg);
            if (retVal < 0)
            {
                LOG_ERROR ("Error: Failed to create a non-secure policy request\n");
            }
            Rat_OffloadPolicy (nameDBHandle, netfpProxyPid, &reqMsg, policyName);
        }
        else if (secureMode == 1)
        {
            token           =   strtok (NULL, delimiter);
            spId            =   atoi (token);
            token           =   strtok (NULL, delimiter);
            sharedMode      =   atoi (token);
            token           =   strtok (NULL, delimiter);
            strncpy (policyName, token, NAME_MAX_CHAR);
            enableOuterIPFrag=  0;

            if ((spId <= 0) ||
                ((sharedMode != 0) && (sharedMode != 1)) ||
                (policyName[0] == '\0'))
            {
                LOG_ERROR ("Invalid IPSec policy configuration \n");
            }

            /* Build an offload request to send to NetFP Proxy */
            retVal  =   Rat_BuildSecureOffloadRequest (xid++,
                                                       spId,
                                                       sharedMode,
                                                       enableOuterIPFrag,
                                                       &reqMsg);
            if (retVal < 0)
            {
                LOG_ERROR ("Error: Failed to create a non-secure policy request\n");
                continue;
            }
            Rat_OffloadPolicy (nameDBHandle, netfpProxyPid, &reqMsg, policyName);
        }
        else
        {
            LOG_ERROR ("Invalid mode for policy \n");
        }

        free (line);
        line = NULL;
    }

    /* Done with policy offloads */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to spawn the specified SYSLIB server.
 *
 *  @retval
 *      Not Applicable.
 */
pid_t Rat_SpawnSyslibServer(char* syslibServerName, char* argv[])
{
    /* Fork the process */
    pid_t pid = fork();
    if (pid == -1)
    {
        perror("fork");
        return -1;
    }

    /* Is this the child process */
    if (pid == 0)
    {
        /* This is the child process. Execute the SYSLIB server which
         * will replace the current process. */
        execvp(syslibServerName, argv);
        perror(syslibServerName);
        abort();
    }
    /* This is the parent process return the child pid back */
    return pid;
}

/**
 *  @b Description
 *  @n
 *      Thread which executes the CLI based interface
 *
 *  @retval
 *      Not Applicable.
 */
static void* Rat_CLI(void* arg)
{
    int32_t testSelection;

    while (1)
    {
        LOG_DEBUG ("**************************************\n");
        LOG_DEBUG ("Root Master CLI Menu:\n");
        LOG_DEBUG ("**************************************\n");
        LOG_DEBUG ("1. LTE Rel9  Dual Cell\n");
        LOG_DEBUG ("2. LTE Rel10 Deployment-1 Cell\n");
        LOG_DEBUG ("3. LTE Rel10 Deployment-2 Cell\n");
        LOG_DEBUG ("4. Exit\n");
        LOG_DEBUG ("**************************************\n");

        /* Get the user input. */
        LOG_DEBUG ("Enter the selection:");
        scanf("%d", &testSelection);

        /* Process the selection. */
        switch (testSelection)
        {
            case 1:
            {
                Rat_Rel9CLI();
                break;
            }
            case 2:
            {
                Rat_Rel10D1_CLI();
                break;
            }
            case 3:
            {
                Rat_Rel10D2_CLI();
                break;
            }
            case 4:
            {
                /* Shutdown the application. */
                exit(0);
            }
            default:
            {
                LOG_ERROR ("Error: Invalid selection\n");
                continue;
            }
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    pthread_t               rootMasterThread;
    pthread_t               testThread;
    Root_MasterConfig       rootMasterConfig;
    int32_t                 errCode;

    /* Initialize the root master configuration */
    memset ((void *)&rootMasterConfig, 0, sizeof(Root_MasterConfig));

    /* Populate the configuration block:
     * The ROOT shared memory address is reserved and should match the DSP memory map */
    strcpy(rootMasterConfig.rootMasterName, "SYSLIB-TestRootMaster");
    rootMasterConfig.ptrSharedMemory        = (uint8_t*)0xA0000000;
    rootMasterConfig.sizeSharedMemory       = 64*1024;
    rootMasterConfig.bootCfgAddress         = (uint32_t)CSL_BOOT_CFG_REGS;

#if defined (DEVICE_K2H)
    rootMasterConfig.numSlaves              = 9;
    rootMasterConfig.slaveCoreId[0]         = ROOT_SLAVE_COREID(1, 0);    /* DSP Core0      */
    rootMasterConfig.slaveCoreId[1]         = ROOT_SLAVE_COREID(1, 1);    /* DSP Core1      */
    rootMasterConfig.slaveCoreId[2]         = ROOT_SLAVE_COREID(1, 2);    /* DSP Core2      */
    rootMasterConfig.slaveCoreId[3]         = ROOT_SLAVE_COREID(1, 3);    /* DSP Core3      */
    rootMasterConfig.slaveCoreId[4]         = ROOT_SLAVE_COREID(1, 4);    /* DSP Core4      */
    rootMasterConfig.slaveCoreId[5]         = ROOT_SLAVE_COREID(1, 5);    /* DSP Core5      */
    rootMasterConfig.slaveCoreId[6]         = ROOT_SLAVE_COREID(1, 6);    /* DSP Core6      */
    rootMasterConfig.slaveCoreId[7]         = ROOT_SLAVE_COREID(1, 7);    /* DSP Core7      */
    rootMasterConfig.slaveCoreId[8]         = ROOT_SLAVE_COREID(0, 8);    /* ARM Process 1  */
#elif (defined (DEVICE_K2L))
    rootMasterConfig.numSlaves              = 5;
    rootMasterConfig.slaveCoreId[0]         = ROOT_SLAVE_COREID(1, 0);    /* DSP Core0      */
    rootMasterConfig.slaveCoreId[1]         = ROOT_SLAVE_COREID(1, 1);    /* DSP Core1      */
    rootMasterConfig.slaveCoreId[2]         = ROOT_SLAVE_COREID(1, 2);    /* DSP Core2      */
    rootMasterConfig.slaveCoreId[3]         = ROOT_SLAVE_COREID(1, 3);    /* DSP Core3      */
    rootMasterConfig.slaveCoreId[4]         = ROOT_SLAVE_COREID(0, 4);    /* ARM Process 1  */
#else
#error "Error: Unsupported Device"
#endif
    rootMasterConfig.ipcSrcId               = 16;                         /* IPC Src Id: 16 */
    rootMasterConfig.osalFxn.malloc         = Root_osalMalloc;
    rootMasterConfig.osalFxn.free           = Root_osalFree;
    rootMasterConfig.osalFxn.beginMemAccess = Root_osalBeginMemoryAccess;
    rootMasterConfig.osalFxn.endMemAccess   = Root_osalEndMemoryAccess;
    rootMasterConfig.osalFxn.enterCS        = Root_osalEnterCS;
    rootMasterConfig.osalFxn.exitCS         = Root_osalExitCS;
    rootMasterConfig.osalFxn.createSem      = Root_osalCreateSem;
    rootMasterConfig.osalFxn.deleteSem      = Root_osalDeleteSem;
    rootMasterConfig.osalFxn.postSem        = Root_osalPostSem;
    rootMasterConfig.osalFxn.pendSem        = Root_osalPendSem;

    /* Create the root master */
    rootMasterHandle = Root_masterCreate (&rootMasterConfig, &errCode);
    if (rootMasterHandle == NULL)
    {
        LOG_ERROR ("Error: Unable to create the root master [Error code %d]\n", errCode);
        return -1;
    }
    LOG_INFO ("Debug: Root master [%p] has been created successfully\n", rootMasterHandle);

    /* Create the root master execution thread. */
    errCode = pthread_create (&rootMasterThread, NULL, RootMasterExecution, NULL);
    if (errCode < 0)
    {
        LOG_ERROR ("Error: Unable to create the root master thread [Error code %d]\n", errCode);
        return -1;
    }

    /* Create the master interface thread which interfaces with the CLI */
    errCode = pthread_create (&testThread, NULL, Rat_CLI, NULL);
    if (errCode < 0)
    {
        LOG_ERROR ("Error: Unable to create the test thread [Error code %d]\n", errCode);
        return -1;
    }

    /* Blocked till the threads are done. */
    pthread_join (rootMasterThread, NULL);
    pthread_join (testThread, NULL);

    /* Test executed successfully. */
    return 0;
}

