/**
 *   @file  netfp_fastPath.c
 *
 *   @brief
 *      The NETFP fast path module which is responsible for the management
 *      of the INBOUND and OUTBOUND fast paths.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/* NETFP Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>
#include <ti/runtime/netfp/include/netfp_net.h>
#include <ti/runtime/netfp/include/netfp_ipv6.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ****************** NETFP Inbound Fast Path Functions *********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to validate the INBOUND Fast Path handle to verify
 *      if the handle is still valid or if it has been deleted
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFP
 *      Inbound Fast Path which is to be validated
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Valid       -   1
 *  @retval
 *      Invalid     -   0
 */
int32_t Netfp_isValidInboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFastPath*  ptrInboundFP
)
{
    Netfp_InboundFastPath*   ptrInboundFastPath;

    /* Cycle through all the INBOUND Fast Paths */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* Do we have a match? */
        if (ptrInboundFastPath == ptrInboundFP)
            return 1;

        /* Goto the next element in the list. */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }

    /* Control comes here implies that there was no match and the fast path handle is invalid */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function searches the inbound fast path database to determine if there
 *      is another fast path which has been created with the same configuration.
 *      The search here would include zombie parents also because zombies still hold
 *      the references to the LUT handle.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFastPathCfg
 *      Fast path configuration
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to NETFP Fast path
 *  @retval
 *      Error   -   NULL
 */
static Netfp_InboundFastPath* Netfp_findParentInboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFPCfg*     ptrInboundFastPathCfg
)
{
    Netfp_InboundFastPath*   ptrInboundFastPath;

    /* Are we trying to create a fast path in the Multicast shared mode? */
    if (ptrInboundFastPathCfg->fastpathMode == Netfp_FastpathMode_MULTICASTSHARED)
    {
        /* YES: In this mode there IPv4 and IPv6 have a fast path each which are shared. */
        if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV4)
            return ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP;
        if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV6)
            return ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP;
    }

    /* Cycle through all the Inbound Fast Paths: */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* We are only looking for parents here; even if they are zombies. */
        if (ptrInboundFastPath->isParent == 1)
        {
            /* Check if this is what we are looking for?
             * Match spidMode, so that different spidMode will have different Fastpath parent to allow
             * programing LUT entries in LUT1-1 and LUT1-2 */
            if ((Netfp_matchIP(&ptrInboundFastPath->cfg.srcIP, &ptrInboundFastPathCfg->srcIP) == 1) &&
                (Netfp_matchIP(&ptrInboundFastPath->cfg.dstIP, &ptrInboundFastPathCfg->dstIP) == 1) &&
                (ptrInboundFastPathCfg->spidMode == ptrInboundFastPath->cfg.spidMode))
            {
                /* Similar entry found; return the handle to the fast path entry. */
                return ptrInboundFastPath;
            }
        }
        /* Goto the next element in the list. */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function returns the number of children which rely on the parent to have
 *      prgrammed the LUT.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundParentFastPath
 *      Parent Fast Path
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of children
 */
static int32_t Netfp_getNumChildren
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFastPath*  ptrInboundParentFastPath
)
{
    Netfp_InboundFastPath*  ptrInboundFastPath;
    int32_t                 count = 0;

    /* Cycle through the inbound fast path */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* Is the fast path a child? */
        if (ptrInboundFastPath->ptrParentIngressFPInfo == ptrInboundParentFastPath)
            count++;

        /* Goto the next element in the list. */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function executes on the NETFP Server and is used to determine the status
 *      of the fast path.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  inboundFPHandle
 *      Inbound fast handle to be checked for
 *  @param[out] status
 *      Set to 1 indicating that the Fast path is active else 0. Ignore the value on
 *      error
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_isInboundFastPathActive
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFPHandle   inboundFPHandle,
    int32_t*                status,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath* ptrInboundFastPath;

    /* Cycle through all the registered fast paths to verify the sanity of the outbound fast path handle */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* Do we have a match? */
        if (ptrInboundFastPath == (Netfp_InboundFastPath*)inboundFPHandle)
        {
            /* YES: We have a match return the status */
            if (ptrInboundFastPath->status == Netfp_Status_ACTIVE)
                *status = 1;
            else
                *status = 0;
            return 0;
        }

        /* Get the next outbound fast path: */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }

    /* Control comes here we are trying to get the status of a fast path which has not been registered or an invalid
     * handle was passed. */
    *errCode = NETFP_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check the status of the inbound fast paths.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  inboundFPHandle
 *      Inbound fast handle to be checked for
 *  @param[out] status
 *      Set to 1 indicating that the Fast path is active else 0. Ignore the value on
 *      error
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_isInboundFastPathActive
(
    Netfp_ClientHandle      clientHandle,
    Netfp_InboundFPHandle   inboundFPHandle,
    int32_t*                status,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Netfp_ClientMCB*        ptrNetfpClient;
    int32_t                 jobId;
    uint32_t                result;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (inboundFPHandle == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_isInboundFastPathActive);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_isInboundFastPathActive
     * (
     * Netfp_ServerMCB*        ptrNetfpServer,
     * Netfp_InboundFPHandle   inboundFPHandle,
     * int32_t*                status,
     * int32_t*                errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_InboundFPHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(inboundFPHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Extract the output arguments */
    *status  = *((int32_t*)args[2].argBuffer);
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a fast path using the specified name.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  fastPathName
 *      Name of the fast path.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the matching inbound fast path
 *  @retval
 *      Error   -   NULL
 */
static Netfp_InboundFPHandle _Netfp_findInboundFastPath
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         fastPathName
)
{
    Netfp_InboundFastPath*   ptrInboundFastPath;

    /* INBOUND Fast Path: Cycle through all the INBOUND path nodes to determine if there is a match or not? */
    ptrInboundFastPath  = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* Zombie fast paths dont exist to the outside world. */
        if (ptrInboundFastPath->status == Netfp_Status_ACTIVE)
        {
            /* Is this what we are looking for? */
            if (strncmp (fastPathName, ptrInboundFastPath->cfg.name, NETFP_MAX_CHAR) == 0)
                return (Netfp_InboundFPHandle)ptrInboundFastPath;
        }

        /* Goto the next element in the list. */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to find
 *      an INBOUND fast path.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in] name
 *      Name of the fast path
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - Handle to the fast path
 *  @retval
 *      NULL    - Fast Path is not found
 */
Netfp_InboundFPHandle Netfp_findInboundFastPath
(
    Netfp_ClientHandle  clientHandle,
    const char*         name,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (name == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_findInboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_InboundFPHandle _Netfp_findInboundFastPath
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  char*                   fastPathName
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = strlen(name) + 1;

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    strncpy ((char*)args[1].argBuffer, name, strlen(name) + 1); // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_InboundFPHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the fully specified IP LUT configuration.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFastPathCfg
 *      Pointer to the Inbound Fast path configuration
 *  @param[in]  prevLinkHandle
 *      PA Previous Link Handle
 *  @param[out]  ptrIPLutCfg
 *      Pointer to the populated IP LUT configuration
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Netp_populateSpecifiedLUTCfg
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFPCfg*     ptrInboundFastPathCfg,
    paLnkHandle_t           prevLinkHandle,
    Netfp_IPLutCfg*         ptrIPLutCfg
)
{
    /* Initialize the LUT configuration: */
    memset((void *)ptrIPLutCfg, 0, sizeof(Netfp_IPLutCfg));

    /* Are we adding an IPv4 or IPv6 address in the LUT? */
    if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV4)
    {
        /* IPv4: */
        ptrIPLutCfg->ipCfg.dstIP.ver             = Netfp_IPVersion_IPV4;
        ptrIPLutCfg->ipCfg.dstIP.addr.ipv4.u.a32 = ptrInboundFastPathCfg->dstIP.addr.ipv4.u.a32;
        ptrIPLutCfg->ipCfg.srcIP.ver             = Netfp_IPVersion_IPV4;
        ptrIPLutCfg->ipCfg.srcIP.addr.ipv4.u.a32 = ptrInboundFastPathCfg->srcIP.addr.ipv4.u.a32;
    }
    else
    {
        /* IPv6: */
        ptrIPLutCfg->ipCfg.dstIP.ver                = Netfp_IPVersion_IPV6;
        ptrIPLutCfg->ipCfg.dstIP.addr.ipv6.u.a32[0] = ptrInboundFastPathCfg->dstIP.addr.ipv6.u.a32[0];
        ptrIPLutCfg->ipCfg.dstIP.addr.ipv6.u.a32[1] = ptrInboundFastPathCfg->dstIP.addr.ipv6.u.a32[1];
        ptrIPLutCfg->ipCfg.dstIP.addr.ipv6.u.a32[2] = ptrInboundFastPathCfg->dstIP.addr.ipv6.u.a32[2];
        ptrIPLutCfg->ipCfg.dstIP.addr.ipv6.u.a32[3] = ptrInboundFastPathCfg->dstIP.addr.ipv6.u.a32[3];

        ptrIPLutCfg->ipCfg.srcIP.ver                = Netfp_IPVersion_IPV6;
        ptrIPLutCfg->ipCfg.srcIP.addr.ipv6.u.a32[0] = ptrInboundFastPathCfg->srcIP.addr.ipv6.u.a32[0];
        ptrIPLutCfg->ipCfg.srcIP.addr.ipv6.u.a32[1] = ptrInboundFastPathCfg->srcIP.addr.ipv6.u.a32[1];
        ptrIPLutCfg->ipCfg.srcIP.addr.ipv6.u.a32[2] = ptrInboundFastPathCfg->srcIP.addr.ipv6.u.a32[2];
        ptrIPLutCfg->ipCfg.srcIP.addr.ipv6.u.a32[3] = ptrInboundFastPathCfg->srcIP.addr.ipv6.u.a32[3];
    }

    /* Setup the next protocol. */
    ptrIPLutCfg->ipCfg.protocol = IPPROTO_UDP;
    ptrIPLutCfg->ipCfg.spidMode = ptrInboundFastPathCfg->spidMode;

    /* Populate the destination information: This will ensure that the LUT2 will be consulted */
    ptrIPLutCfg->dstInfo.dstType = Netfp_DestType_LINKED_RULE;
    ptrIPLutCfg->dstInfo.flowId  = 0xFFFFFFFF;

    /* Prepare IP Lut configuration */
    ptrIPLutCfg->prevLinkHandle  = prevLinkHandle;
    ptrIPLutCfg->nextLinkHandle  = NULL;
    ptrIPLutCfg->ptrIPSecChannel = NULL;
    ptrIPLutCfg->flags           = NETFP_IPCFG_FLAGS_NONE;
    ptrIPLutCfg->fastpathMode    = ptrInboundFastPathCfg->fastpathMode;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the wildcarded IP LUT configuration.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFastPathCfg
 *      Pointer to the Inbound Fast path configuration
 *  @param[out]  ptrIPLutCfg
 *      Pointer to the populated IP LUT configuration
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Netp_populateUnspecifiedLUTCfg
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFPCfg*     ptrInboundFastPathCfg,
    Netfp_IPLutCfg*         ptrIPLutCfg
)
{
    paLnkHandle_t   prevLinkHandle;

    /* Initialize the LUT configuration: */
    memset((void *)ptrIPLutCfg, 0, sizeof(Netfp_IPLutCfg));

    /* Are we adding an IPv4 or IPv6 address in the LUT? */
    if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV4)
    {
        /* IPv4: The IP address is 0.0.0.0 */
        ptrIPLutCfg->ipCfg.dstIP.ver  = Netfp_IPVersion_IPV4;
        ptrIPLutCfg->ipCfg.srcIP.ver  = Netfp_IPVersion_IPV4;

        /* Set the previous link to the IPv4 Virtual link */
        prevLinkHandle = ptrNetfpServer->multicastInfo.vlinkIPv4Handle;
    }
    else
    {
        /* IPv6: The IP address is :: */
        ptrIPLutCfg->ipCfg.dstIP.ver = Netfp_IPVersion_IPV6;
        ptrIPLutCfg->ipCfg.srcIP.ver = Netfp_IPVersion_IPV6;

        /* Set the previous link to the IPv6 Virtual link */
        prevLinkHandle = ptrNetfpServer->multicastInfo.vlinkIPv6Handle;
    }

    /* Setup the next protocol. */
    ptrIPLutCfg->ipCfg.protocol = IPPROTO_UDP;
    ptrIPLutCfg->ipCfg.spidMode = ptrInboundFastPathCfg->spidMode;

    /* Populate the destination information: This will ensure that the LUT2 will be consulted */
    ptrIPLutCfg->dstInfo.dstType = Netfp_DestType_LINKED_RULE;
    ptrIPLutCfg->dstInfo.flowId  = 0xFFFFFFFF;

    /* Prepare IP Lut configuration */
    ptrIPLutCfg->prevLinkHandle  = prevLinkHandle;
    ptrIPLutCfg->nextLinkHandle  = NULL;
    ptrIPLutCfg->ptrIPSecChannel = NULL;
    ptrIPLutCfg->flags           = NETFP_IPCFG_FLAGS_NONE;
    ptrIPLutCfg->fastpathMode    = ptrInboundFastPathCfg->fastpathMode;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add an endpoint.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFastPathCfg
 *      Pointer to the Inbound Fast path configuration
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to Inbound fast path handle
 *  @retval
 *      Error   -   NULL
 */
static Netfp_InboundFPHandle _Netfp_createInboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFPCfg*     ptrInboundFastPathCfg,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath*  ptrInboundFastPath;
    Netfp_SPInfo*           ptrSPInfo;
    paLnkHandle_t           prevLinkHandle;
    Netfp_InboundFastPath*  ptrParentIngressFP;
    Netfp_IPSecChannel*     ptrIPSecChannel;
    Netfp_IPLutCfg          ipLutCfg;

    /* Sanity Check: Both the source & destination IP address should be of the same IP family. */
    if (unlikely(ptrInboundFastPathCfg->dstIP.ver != ptrInboundFastPathCfg->srcIP.ver))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Fast Path names need to be unique in the system. */
    if (unlikely(_Netfp_findInboundFastPath(ptrNetfpServer, ptrInboundFastPathCfg->name) != NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Special Case: The security policy identifier NETFP_INVALID_SPID is a special identifier which
     * needs to be handled differently */
    if (ptrInboundFastPathCfg->spidMode == Netfp_SPIDMode_SPECIFIC)
    {
        /* Security Policy identifer was specified. Find this in the database */
        ptrSPInfo = (Netfp_SPInfo *)Netfp_findSPById (ptrNetfpServer, ptrInboundFastPathCfg->spId);
        if (ptrSPInfo == NULL)
        {
            /* Security Policy does not exist in the database. This is an invalid configuration. */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }
    else if (ptrInboundFastPathCfg->spidMode == Netfp_SPIDMode_INVALID)
    {
        /* Special case: There is no security policy associated with the fast path. */
        ptrSPInfo = NULL;
    }
    else if (ptrInboundFastPathCfg->spidMode == Netfp_SPIDMode_ANY_SECURE)
    {
        /* Wild Carded SPI: no link to LUT1-1 will be added */
        ptrSPInfo = NULL;
    }
    else
    {
        /* Invalid SPI mode */
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Match the security policy checks if there was a valid security policy. */
    if (ptrSPInfo != NULL)
    {
        /* Security Policy was specified. Is the security policy active? */
        if (ptrSPInfo->status == Netfp_Status_ZOMBIE)
        {
            *errCode = NETFP_ENOTREADY;
            return NULL;
        }

        /* We need to ensure that the fast path IP and the IP specified in the security
         * policy belong to the same family. NOTE: We have already verified that the
         * src IP & dst IP specified in the Fast path & Security Policy configuration
         * belong to the same family. */
        if (ptrSPInfo->spCfg.dstIP.ver != ptrInboundFastPathCfg->dstIP.ver)
        {
            *errCode = NETFP_EINVAL;
            return NULL;
        }

        /* Validate the fast path source & destination IP address against the specified security policy:
         * This will help determine if the fast path creation is allowed by the policy or not. */
        if ((Netfp_policyCheckIP (Netfp_Direction_INBOUND,  ptrSPInfo, ptrInboundFastPathCfg->srcIP) == 0) ||
            (Netfp_policyCheckIP (Netfp_Direction_OUTBOUND, ptrSPInfo, ptrInboundFastPathCfg->dstIP) == 0))
        {
            /* Error: Security policy did not allow the connection. */
            *errCode = NETFP_ENOPERM;
            return NULL;
        }

        /* Is the SP associated with an SA? */
        ptrIPSecChannel = (Netfp_IPSecChannel*)ptrSPInfo->spCfg.saHandle;
        if (ptrIPSecChannel == NULL)
        {
            /* No SA association */
            prevLinkHandle = NULL;
        }
        else
        {
            /* YES. Link the inner IP rule to outer IP rule using the virtual link. */
            prevLinkHandle = ptrIPSecChannel->ptrNetfpPAVlink->vlinkHandle;
        }
    }
    else
    {
        /* No Security Policy was associated; so there is no previous LUT1-1 entry to which we
         * need to link the rule. There is also no valid security association */
        prevLinkHandle  = NULL;
        ptrIPSecChannel = NULL;
    }

    /* Populate the IP LUT Configuration: */
    switch (ptrInboundFastPathCfg->fastpathMode)
    {
        case Netfp_FastpathMode_NORMAL:
        {
            /* Unicast/Multicast Mode: We need to use the fully specified LUT configuration */
            Netp_populateSpecifiedLUTCfg (ptrNetfpServer, ptrInboundFastPathCfg, prevLinkHandle, &ipLutCfg);
            break;
        }
        case Netfp_FastpathMode_INFOONLY:
        {
            /* No LUT entry will be added, no LUT configuration will be needed. */
            memset((void *)&ipLutCfg, 0, sizeof(Netfp_IPLutCfg));
            break;
        }
        case Netfp_FastpathMode_MULTICASTSHARED:
        {
            /* Multicast Shared Mode: In this mode we use the wildcarded IP address linked to the MAC virtual
             * links. */
            Netp_populateUnspecifiedLUTCfg (ptrNetfpServer, ptrInboundFastPathCfg, &ipLutCfg);
            break;
        }
        default:
        {
            /* Error: Bad multicast mode configuration */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }

    /* Allocate memory for the fast path */
    ptrInboundFastPath = ptrNetfpServer->cfg.malloc (sizeof(Netfp_InboundFastPath), 0);
    if (ptrInboundFastPath == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrInboundFastPath, 0, sizeof(Netfp_InboundFastPath));

    /* Populate the fast path information. */
    memcpy ((void *)&ptrInboundFastPath->cfg, (void *)ptrInboundFastPathCfg, sizeof (Netfp_InboundFPCfg));

    if (ptrInboundFastPathCfg->fastpathMode == Netfp_FastpathMode_INFOONLY)
    {
        /* For information only fast path , no LUT entry programming is needed, hence no need to have a parent */
        ptrParentIngressFP                         = NULL;
        ptrInboundFastPath->ptrParentIngressFPInfo = NULL;
        ptrInboundFastPath->ptrFpLutInfo           = NULL;

        /* Setup the parent flag for the INBOUND fast path. */
        ptrInboundFastPath->isParent = 0;
    }
    else
    {
        /* Determine if we need to program the LUT Entry or not. There could be multiple fast paths
         * created using the same configuration. Since the PA does not allow multiple configurations
         * with the same LUT; we catch the condition here. */
        ptrParentIngressFP = Netfp_findParentInboundFastPath(ptrNetfpServer, ptrInboundFastPathCfg);
        if (ptrParentIngressFP == NULL)
        {
            /* Allocating LUT resources before add IP: This is NOT an IPSEC Channel */
            ptrInboundFastPath->ptrFpLutInfo = Netfp_allocIPLutEntry (ptrNetfpServer, &ipLutCfg, 0, errCode);
            if (ptrInboundFastPath->ptrFpLutInfo == NULL)
            {
                /* Error: Cleanup the memory allocated for the fast path. */
                ptrNetfpServer->cfg.free (ptrInboundFastPath, sizeof(Netfp_InboundFastPath));
                return NULL;
            }

            /* Parents dont have a parent. */
            ptrInboundFastPath->ptrParentIngressFPInfo = NULL;

            /* Setup the parent flag for the INBOUND fast path. */
            ptrInboundFastPath->isParent = 1;
        }
        else
        {
            /* Child: Inherit the properties from the parent */
            ptrInboundFastPath->ptrFpLutInfo           = ptrParentIngressFP->ptrFpLutInfo;
            ptrInboundFastPath->ptrParentIngressFPInfo = ptrParentIngressFP;

            /* Setup the parent flag for the INBOUND fast path. */
            ptrInboundFastPath->isParent = 0;
        }
    }

    /* Track the security policy and association: */
    ptrInboundFastPath->ptrSPInfo       = ptrSPInfo;
    ptrInboundFastPath->ptrIPSecChannel = ptrIPSecChannel;

    /* Set the flag to determine if the fast path is secure or not. */
    if (ptrIPSecChannel == NULL)
        ptrInboundFastPath->isSecure = 0;
    else
        ptrInboundFastPath->isSecure = 1;

    /* The FP is active and can be used. */
    ptrInboundFastPath->status    = Netfp_Status_ACTIVE;
    ptrInboundFastPath->direction = Netfp_Direction_INBOUND;

    /* Is the fast path operating in Multicast-Sharing mode? */
    if (ptrInboundFastPathCfg->fastpathMode == Netfp_FastpathMode_MULTICASTSHARED)
    {
        /* YES: Did we create an IPv4 fast path? */
        if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV4)
        {
            /* YES: Remember the Multicast handle if not already known */
            if (ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP == NULL)
                ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP = ptrInboundFastPath;
        }

        /* YES: Did we create an IPv6 fast path? */
        if (ptrInboundFastPathCfg->dstIP.ver == Netfp_IPVersion_IPV6)
        {
            /* YES: Remember the Multicast handle if not already known */
            if (ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP == NULL)
                ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP = ptrInboundFastPath;
        }

        /* Debug Message: */
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: [Creation] Multicast Sharing Mode IPv4:0x%x IPv6:0x%x\n",
                      ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP, ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP);
    }

    /* Record the Fast Path into the server database */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList, (Netfp_ListNode*)ptrInboundFastPath);
    return (Netfp_InboundFPHandle)ptrInboundFastPath;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to create
 *      an inbound fast path. Inbound fast paths create a Layer3 endpoint to
 *      receive packets.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrInboundFastPathCfg
 *      Pointer to the fast path configuration
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_InboundFPHandle Netfp_createInboundFastPath
(
    Netfp_ClientHandle  clientHandle,
    Netfp_InboundFPCfg* ptrInboundFastPathCfg,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_InboundFPCfg*     ptrRemoteInboundFPCfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (ptrInboundFastPathCfg == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Sanity Check: Ensure that the NETFP Client has setup the multicast services before using them. */
    if (unlikely(ptrInboundFastPathCfg->fastpathMode == Netfp_FastpathMode_MULTICASTSHARED))
    {
        /* Inbound Fast Path is trying to Using Multicast Services: Has this been initialized? */
        if (ptrNetfpClient->multicastInfo.isInitialized == 0)
        {
            *errCode = NETFP_ENOTREADY;
            return NULL;
        }
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_createInboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_InboundFPHandle _Netfp_createInboundFastPath
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_InboundFPCfg*     ptrInboundFastPathCfg,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_InboundFPCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the interface configuration. */
    ptrRemoteInboundFPCfg = (Netfp_InboundFPCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteInboundFPCfg, (void *)ptrInboundFastPathCfg, sizeof (Netfp_InboundFPCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_InboundFPHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete an INBOUND fast path
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrInboundFastPath
 *      Pointer to the Inbound fast path which is to be deleted
 *  @param[in]  reason
 *      Original reason because of which the function is invoked
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_deleteInboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_InboundFastPath*  ptrInboundFastPath,
    Netfp_Reason            reason,
    int32_t*                errCode
)
{
    Netfp_InboundFastPath*  ptrParentFastPath;
    uint32_t                numChildren;
    Netfp_EventMetaInfo     eventInfo;

    /* Sanity Check: Is the path valid or not? */
    if (unlikely(Netfp_isValidInboundFastPath(ptrNetfpServer, ptrInboundFastPath) == 0))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* INBOUND Fast Path: These are fast paths which have configured the LUT; but because we
     * support multiple fast paths with the same configuration. There is only 1 parent which
     * actually programs the LUT while the remaining children inherit the LUT entry. Is this
     * a parent which is being deleted? */
    if (ptrInboundFastPath->isParent == 1)
    {
        /* Parent Fast Path is being deleted. The LUT rule in the PA can only be deleted
         * if there are no active children which are also using it. */
        numChildren = Netfp_getNumChildren (ptrNetfpServer, ptrInboundFastPath);
        if (numChildren == 0)
        {
            /* Free LUT resource */
            *errCode = Netfp_freeIPLutEntry (ptrNetfpServer, ptrInboundFastPath->ptrFpLutInfo);
            if (*errCode < 0)
                return -1;
        }
        else
        {
            /* Active children are still using the LUT. The parent fast is now a zombie.
             * The zombie fast path will remain till all the children using this have been
             * deleted. The last child fast path interface will then kill the parent zombie.
             * We are done for now. */
            ptrInboundFastPath->status = Netfp_Status_ZOMBIE;
			*errCode = 0;
            return 0;
        }
    }
    else
    {
        /* Children Fast Path is being deleted. Since children dont program the LUT but
         * simply inherit it; there is no LUT deletion required here. Fall through and
         * continue to delete the fast path. */
    }

    /* There are no more references to the security policy. */
    ptrInboundFastPath->ptrSPInfo = NULL;

    /* Is the fast path being deleted operating in Multicast-Sharing mode? */
    if (ptrInboundFastPath->cfg.fastpathMode == Netfp_FastpathMode_MULTICASTSHARED)
    {
        /* YES: Is this an IPv4 fast path? */
        if (ptrInboundFastPath->cfg.dstIP.ver == Netfp_IPVersion_IPV4)
        {
            /* YES: Is this the wildcarded multicast IPv4 fast path? */
            if (ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP == ptrInboundFastPath)
                ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP = NULL;
        }

        /* YES: Is this an IPv6 fast path? */
        if (ptrInboundFastPath->cfg.dstIP.ver == Netfp_IPVersion_IPV6)
        {
            /* YES: Is this the wildcarded multicast IPv6 fast path? */
            if (ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP == ptrInboundFastPath)
                ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP = NULL;
        }

        /* Debug Message: */
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Debug: [Deletion] Multicast Sharing Mode IPv4:0x%x IPv6:0x%x\n",
                      ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP, ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP);
    }

    /* Remove the fast path from the NETFP Server */
    Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList, (Netfp_ListNode*)ptrInboundFastPath);

    /* Generate the delete fast path event and push it to all the NETFP clients: */
    eventInfo.eventId           = Netfp_EventId_DELETE_FP;
    eventInfo.u.fpMeta.fpHandle = (void*)ptrInboundFastPath;
    eventInfo.u.fpMeta.reason   = reason;
    memset ((void*)&eventInfo.u.fpMeta.l2ConnectInfo, 0, sizeof(Netfp_SockL2ConnectInfo));
    Netfp_generateEvent (ptrNetfpServer, &eventInfo);

    /* Get the parent fast path */
    ptrParentFastPath = ptrInboundFastPath->ptrParentIngressFPInfo;

    /* Cleanup the memory associated with the fast path */
    ptrNetfpServer->cfg.free (ptrInboundFastPath, sizeof(Netfp_InboundFastPath));

    /* Was this a child fast path being deleted? */
    if (ptrParentFastPath != NULL)
    {
        /* YES. Was the parent a zombie? Try to delete the zombie parent fast path */
        if (ptrParentFastPath->status == Netfp_Status_ZOMBIE)
            return _Netfp_deleteInboundFastPath (ptrNetfpServer, ptrParentFastPath, Netfp_Reason_FAST_PATH_DELETE, errCode);
    }
	*errCode = 0;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to delete an inbound
 *      fast path
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  fpHandle
 *      Handle to the fast path to be deleted
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteInboundFastPath
(
    Netfp_ClientHandle      clientHandle,
    Netfp_InboundFPHandle   fpHandle,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (fpHandle == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_deleteInboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_deleteInboundFastPath
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_InboundFPHandle   fpHandle,
     *  Netfp_Reason            reason,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_InboundFPHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_Reason);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the NETFP Fast Path handle.  */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(fpHandle);

    /* Original Event: This is because the NETFP Fast Path event is being deleted  */
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(Netfp_Reason_FAST_PATH_DELETE);

    /* We should initialize reference type parameter */
    *(uint32_t*)args[3].argBuffer = *errCode; //fzm

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill all the inbound fast paths registered with the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success  -   0
 *  @retval
 *      Error    -   <0
 */
int32_t Netfp_killInboundFastPath
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_InboundFastPath*     ptrInboundFastPath;

    /* Close all the inbound fast paths which have been configured in the server */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    while (ptrInboundFastPath != NULL)
    {
        /* Delete the fast path from the server. */
        if (_Netfp_deleteInboundFastPath (ptrNetfpServer, ptrInboundFastPath, Netfp_Reason_FAST_PATH_DELETE, errCode) < 0)
            return -1;

        /* Get the next fast path */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function which returns a string matching the multicast mode
 *
 *  @param[in]  mode
 *      Multicast mode
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Multicast Mode string
 */
static const char* Netfp_getFastpathModeString(Netfp_FastpathMode mode)
{
    switch (mode)
    {
        case Netfp_FastpathMode_NORMAL:
        {
            return "Normal";
        }
        case Netfp_FastpathMode_INFOONLY:
        {
            return "Information Only";
        }
        case Netfp_FastpathMode_MULTICASTSHARED:
        {
            return "Multicast Shared";
        }
        default:
        {
            return "Invalid";
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of the INBOUND
 *      fast paths
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of fast paths which are present
 */
int32_t Netfp_displayInboundFastPath(Netfp_ServerHandle serverHandle)
{
    Netfp_InboundFastPath*  ptrInboundFastPath;
    Netfp_ServerMCB*        ptrNetfpServer;
    int32_t                 errCode;
    int32_t                 count = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the Inbound Fast Path */
    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");

    /* Display the Multicast Information: */
    if (ptrNetfpServer->multicastInfo.isInitialized == 1)
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Multicast services initialized\n");
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Shared Multicast IPv4: 0x%x IPv6: 0x%x\n",
                      ptrNetfpServer->multicastInfo.ptrIPv4MulticastFP, ptrNetfpServer->multicastInfo.ptrIPv6MulticastFP);
    }
    else
    {
        Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Multicast services NOT initialized\n");
    }

    /* Does the server support the configuration of user counters? */
    if (ptrNetfpServer->cfg.enableIPLutEntryCount)
    {
        /* YES: Get the user statistics */
        if (Netfp_getUserStats (ptrNetfpServer, ptrNetfpServer->ptrUserStats, &errCode) < 0)
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Unable to get the user stats [Error code %d]\n", errCode);
    }

    /* Cycle through all the fast paths and display them on the console */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Inbound Fast Path Table\n");
    while (ptrInboundFastPath != NULL)
    {
        /* Display the fast path */
        if (ptrInboundFastPath->cfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] Name: %s Policy: %d Src IP %d.%d.%d.%d -> Dst IP %d.%d.%d.%d\n",
                          ptrInboundFastPath, (ptrInboundFastPath->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE",
                          ptrInboundFastPath->cfg.name, ptrInboundFastPath->cfg.spId,
                          ptrInboundFastPath->cfg.srcIP.addr.ipv4.u.a8[0], ptrInboundFastPath->cfg.srcIP.addr.ipv4.u.a8[1],
                          ptrInboundFastPath->cfg.srcIP.addr.ipv4.u.a8[2], ptrInboundFastPath->cfg.srcIP.addr.ipv4.u.a8[3],
                          ptrInboundFastPath->cfg.dstIP.addr.ipv4.u.a8[0], ptrInboundFastPath->cfg.dstIP.addr.ipv4.u.a8[1],
                          ptrInboundFastPath->cfg.dstIP.addr.ipv4.u.a8[2], ptrInboundFastPath->cfg.dstIP.addr.ipv4.u.a8[3]);
        }
        else
        {
            char    srcIP[128];
            char    dstIP[128];

            /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrInboundFastPath->cfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrInboundFastPath->cfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] Name: %s Policy: %d Src IP %s -> Dst IP %s\n",
                          ptrInboundFastPath, (ptrInboundFastPath->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE",
                          ptrInboundFastPath->cfg.name, ptrInboundFastPath->cfg.spId, srcIP, dstIP);
        }

        /* Display the LUT Instance & Fastpath mode */
        if(ptrInboundFastPath->ptrFpLutInfo)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            LUT: %d(%d) FastpathMode: %s\n",
                          ptrInboundFastPath->ptrFpLutInfo->lutInfo.lut1Inst, ptrInboundFastPath->ptrFpLutInfo->lutInfo.lut1Index,
                          Netfp_getFastpathModeString(ptrInboundFastPath->cfg.fastpathMode));

            /* Does the server support the configuration of user counters? */
            if (ptrNetfpServer->cfg.enableIPLutEntryCount)
            {
                /* YES: Display the user statistics counters matching the user statistics index: */
                Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                              "            UserStatIndex:%d  Packets:%d\n",
                              ptrInboundFastPath->ptrFpLutInfo->matchPktCntIndex,
                              ptrNetfpServer->ptrUserStats->count32[ptrInboundFastPath->ptrFpLutInfo->matchPktCntIndex -
                              ptrNetfpServer->userStatCfg.num64bUserStats]);
            }
        }
        else
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            FastpathMode: %s \n",
                          Netfp_getFastpathModeString(ptrInboundFastPath->cfg.fastpathMode));
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next fast path */
        ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
    }
    return count;
}

/**************************************************************************
 ****************** NETFP Outbound Fast Path Functions ********************
 **************************************************************************/

void Netfp_removeFPFromRecomputationList(Netfp_ServerMCB* ptrNetfpServer, const Netfp_OutboundFastPath *ptrOutboundFastPath)
{
    Netfp_RecomputeRoute* ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList);
    while(ptrRecomputeRoute != NULL)
    {
        if(ptrRecomputeRoute->ptrResolvedId == &ptrOutboundFastPath->requestId)
        {
            Netfp_RecomputeRoute* tmpNode = ptrRecomputeRoute;
            ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetNext ((Netfp_ListNode*)ptrRecomputeRoute);

            Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrRouteRecomputationList, (Netfp_ListNode*)tmpNode);
            ptrNetfpServer->cfg.free(tmpNode, sizeof(Netfp_RecomputeRoute));

            break;
        }
        else
            ptrRecomputeRoute = (Netfp_RecomputeRoute*)Netfp_listGetNext ((Netfp_ListNode*)ptrRecomputeRoute);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the PMTU of the fast path.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOutboundFastPath
 *      Pointer to the outbound fast path for which the MTU is to be setup.
 *  @param[in]  newPMTU
 *      New PMTU to be configured
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_setupFastPathPMTU
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_OutboundFastPath* ptrOutboundFastPath,
    uint32_t                newPMTU
)
{
    Netfp_IPSecChannel*     ptrIPSecChannel;
    uint32_t                minPMTU;
    uint32_t                currentPMTU;

    /* Take a snapshot of the current PMTU */
    currentPMTU = ptrOutboundFastPath->fastPathPMTU;

    /* Is the fast path ACTIVE or ZOMBIE? */
    if (ptrOutboundFastPath->status == Netfp_Status_ZOMBIE)
    {
        /* ZOMBIE: The fast path is not reachable. This could happen in the following cases:-
         *  (a): Non-secure Outbound Fast Path creation where the route resolution is not done
         *  (b): Secure Outbound Fast Path creation where the SA route resolution is not complete
         *  (c): Moving from an ACTIVE to ZOMBIE state
         *
         * In all the cases we reset the fast path MTU back to INVALID. Case (a) and (b) are
         * valid only during the Outbound fast path creation.
         *
         * Case (c) is encountered on a route flush or if the next hop neighbor is no longer
         * reachable. In this case we can no longer use the older MTU because there might be
         * another route which has a different MTU. */
        ptrOutboundFastPath->fastPathPMTU = NETFP_INVALID_MTU;
    }
    else
    {
        /* ACTIVE Fast Path: Is the fast path Secure or Non-Secure? */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /* Non-Secure Fast Path: We need to consider the following
             *  (a) Existing Fast Path PMTU configuration
             *  (b) New PMTU which is to be configured
             * Select the minimum of the above */
            ptrOutboundFastPath->fastPathPMTU = Netfp_min (ptrOutboundFastPath->fastPathPMTU, newPMTU);

            /* Setup the age of the PMTU: */
            if (currentPMTU != ptrOutboundFastPath->fastPathPMTU)
                ptrOutboundFastPath->agePMTU = NETFP_MAX_PMTU_AGE;
        }
        else
        {
            /* Secure Fast Path: We need to consider the following:
             *  (a) Existing Fast Path PMTU configuration
             *  (b) IPSEC PMTU (Since the fast path resides over the IPSEC Channel)
             *  (c) New PMTU which is to be configured
             * Select the minimum. */
            ptrIPSecChannel = (Netfp_IPSecChannel*)ptrOutboundFastPath->ptrSPInfo->spCfg.saHandle;
            minPMTU = Netfp_min (ptrOutboundFastPath->fastPathPMTU, ptrIPSecChannel->ipsecPMTU);
            ptrOutboundFastPath->fastPathPMTU = Netfp_min (minPMTU, newPMTU);

            /* Setup the age of the PMTU: */
            if (ptrOutboundFastPath->fastPathPMTU == ptrIPSecChannel->ipsecPMTU)
            {
                /* Fast Path inherited the PMTU from the IPSEC channel and so it inherits the age also. */
                ptrOutboundFastPath->agePMTU = ptrIPSecChannel->agePMTU;
            }
            else if (currentPMTU != ptrOutboundFastPath->fastPathPMTU)
            {
                /* Fast Path PMTU was modified. So initialize the age to the default. */
                ptrOutboundFastPath->agePMTU = NETFP_MAX_PMTU_AGE;
            }
            else
            {
                /* Fall through: There was no change in the PMTU so continue with the current age. */
            }
        }
    }
    /* Debug Message: */
    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Fast Path [%s] PMTU:%d bytes Age:%d sec\n",
                  ptrOutboundFastPath->cfg.name, ptrOutboundFastPath->fastPathPMTU, ptrOutboundFastPath->agePMTU);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to age the PMTU of the outbound fast paths.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  timeout
 *      PMTU ageing timeout
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_ageFastPathPMTU
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            timeout
)
{
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    Netfp_EventMetaInfo         eventInfo;
    Netfp_IPSecChannel*         ptrIPSecChannel;

    /* Cycle through all the outbound fast paths */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Is this a secure or non-secure fast path? */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /***********************************************************************************
             * Non secure: Did we receive a PMTU update?
             ***********************************************************************************/
             if (ptrOutboundFastPath->fastPathPMTU != NETFP_INVALID_MTU)
             {
                /* YES: Decrement the age of the PMTU */
                ptrOutboundFastPath->agePMTU = ptrOutboundFastPath->agePMTU - timeout;
                if (ptrOutboundFastPath->agePMTU <= 0)
                {
                    /* YES: We did this implies that the fast path is also active. Reset the PMTU */
                    ptrOutboundFastPath->fastPathPMTU = NETFP_INVALID_MTU;
                    ptrOutboundFastPath->agePMTU      = 0;

                    /* Debug Message: */
                    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Ageing Non-secure FP [%s] FP PMTU %d bytes\n",
                                  ptrOutboundFastPath->cfg.name, ptrOutboundFastPath->fastPathPMTU);

                    /* Generate an event and notify all the clients/sockets about the update to the PMTU */
                    eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                    eventInfo.u.fpMeta.reason   = Netfp_Reason_PMTU_CHANGE;
                    eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                    eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                    Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                    Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                }
            }
        }
        else
        {
            /***********************************************************************************
             * Secure: Get the IPSEC channel associated with the fast path.
             ***********************************************************************************/
            ptrIPSecChannel = (Netfp_IPSecChannel*)ptrOutboundFastPath->ptrSPInfo->spCfg.saHandle;

            /* Did we receive a PMTU update on the IPSEC Channel (Outer IP)? */
            if (ptrIPSecChannel->ipsecPMTU != NETFP_INVALID_MTU)
            {
                /* YES: Decrement the age of the IPSEC PMTU */
                ptrIPSecChannel->agePMTU = ptrIPSecChannel->agePMTU - timeout;
                if (ptrIPSecChannel->agePMTU <= 0)
                {
                    /* YES: Was the fast path PMTU inherited from the IPSEC PMTU */
                    if (ptrOutboundFastPath->fastPathPMTU == ptrIPSecChannel->ipsecPMTU)
                    {
                        /* YES: Reset both the fast path PMTU & IPSEC PMTU */
                        ptrIPSecChannel->ipsecPMTU        = NETFP_INVALID_MTU;
                        ptrIPSecChannel->agePMTU          = 0;
                        ptrOutboundFastPath->fastPathPMTU = NETFP_INVALID_MTU;
                        ptrOutboundFastPath->agePMTU      = 0;

                        /* Debug Message: */
                        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Ageing Secure FP [%s] OuterIP IPSEC PMTU: %d FP PMTU %d bytes\n",
                                      ptrOutboundFastPath->cfg.name, ptrIPSecChannel->ipsecPMTU, ptrOutboundFastPath->fastPathPMTU);

                        /* Generate an event and notify all the clients/sockets about the update to the PMTU */
                        eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                        eventInfo.u.fpMeta.reason   = Netfp_Reason_PMTU_CHANGE;
                        eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                        eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                    }
                    else
                    {
                        /* NO: Reset only the IPSEC PMTU */
                        ptrIPSecChannel->ipsecPMTU = NETFP_INVALID_MTU;
                        ptrIPSecChannel->agePMTU   = 0;

                        /* Debug Message: */
                        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Ageing Secure FP [%s] OuterIP IPSEC PMTU: %d FP PMTU %d bytes\n",
                                      ptrOutboundFastPath->cfg.name, ptrIPSecChannel->ipsecPMTU, ptrOutboundFastPath->fastPathPMTU);

                        /********************************************************************************************************
                         * Since the Fast Path PMTU was never dependent on the IPSEC PMTU there is no *update* required here. So
                         * we dont even need to generate an event to the NETFP Clients/Sockets. NOTE: For clarity the Fast Path
                         * PMTU is calculated as follows:-
                         *  outboundFastPathPMTU = Netfp_min (outboundFastPathPMTU, ipsecPMTU);
                         *
                         * Since ipsecPMTU is already set to NETFP_INVALID_MTU. The above evaluates to
                         *  outboundFastPathPMTU = outboundFastPathPMTU;
                         ********************************************************************************************************/
                    }
                }
            }

            /* Did we receive a PMTU update on the Fast Path (Inner IP)? */
            if (ptrOutboundFastPath->fastPathPMTU != NETFP_INVALID_MTU)
            {
                /* YES: Decrement the age of the Fast path PMTU */
                ptrOutboundFastPath->agePMTU = ptrOutboundFastPath->agePMTU - timeout;
                if (ptrOutboundFastPath->agePMTU <= 0)
                {
                    /* YES: Inner PMTU has aged out: Reset the PMTU for the Fast Path (Inner) */
                    ptrOutboundFastPath->fastPathPMTU = NETFP_INVALID_MTU;
                    ptrOutboundFastPath->agePMTU      = 0;

                    /* Recalculate the Inner Fast Path PMTU again: This is the IPSEC (Outer) could *still* have a valid
                     * PMTU which needs to be accounted for. */
                    ptrOutboundFastPath->fastPathPMTU = Netfp_min (ptrOutboundFastPath->fastPathPMTU, ptrIPSecChannel->ipsecPMTU);

                    /* Setup the age of the PMTU: */
                    if (ptrOutboundFastPath->fastPathPMTU == ptrIPSecChannel->ipsecPMTU)
                        ptrOutboundFastPath->agePMTU = ptrIPSecChannel->agePMTU;

                    /* Debug Message: */
                    Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG, "Debug: Ageing Secure FP [%s] InnerIP IPSEC PMTU: %d FP PMTU %d bytes\n",
                                  ptrOutboundFastPath->cfg.name, ptrIPSecChannel->ipsecPMTU, ptrOutboundFastPath->fastPathPMTU);

                    /* Generate an event and notify all the clients/sockets about the update to the PMTU */
                    eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                    eventInfo.u.fpMeta.reason   = Netfp_Reason_PMTU_CHANGE;
                    eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                    eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                    Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                    Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                }
            }
        }
        /* Goto the next element in the list. */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the OUTBOUND Fast Path handle to verify
 *      if the handle is still valid or if it has been deleted
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOutboundFP
 *      Outbound Fast Path which is to be validated
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Valid       -   1
 *  @retval
 *      Invalid     -   0
 */
int32_t Netfp_isValidOutboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_OutboundFastPath* ptrOutboundFP
)
{
    Netfp_OutboundFastPath*  ptrOutboundFastPath;

    /* Cycle through all the OUTBOUND Fast Paths */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Do we have a match? */
        if (ptrOutboundFastPath == ptrOutboundFP)
            return 1;

        /* Goto the next element in the list. */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }

    /* Control comes here implies that there was no match and the fast path handle is invalid */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a fast path using the specified name.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  fastPathName
 *      Name of the fast path.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the matching outbound fast path
 *  @retval
 *      Error   -   NULL
 */
static Netfp_OutboundFPHandle _Netfp_findOutboundFastPath
(
    Netfp_ServerMCB*    ptrNetfpServer,
    const char*         fastPathName
)
{
    Netfp_OutboundFastPath*  ptrOutboundFastPath;

    /* OUTBOUND Fast Path: Cycle through all the OUTBOUND path nodes to determine if there is a match or not? */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Zombie fast paths dont exist to the outside world. */
        if (ptrOutboundFastPath->status == Netfp_Status_ACTIVE)
        {
            /* Is this what we are looking for? */
            if (strncmp (fastPathName, ptrOutboundFastPath->cfg.name, NETFP_MAX_CHAR) == 0)
                return (Netfp_OutboundFPHandle)ptrOutboundFastPath;
        }

        /* Goto the next element in the list. */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to find
 *      an OUTBOUND fast path.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in] name
 *      Name of the fast path
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - Handle to the fast path
 *  @retval
 *      NULL    - Fast Path is not found
 */
Netfp_OutboundFPHandle Netfp_findOutboundFastPath
(
    Netfp_ClientHandle  clientHandle,
    const char*         name,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (name == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_findOutboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_OutboundFPHandle _Netfp_findOutboundFastPath
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  char*                   fastPathName
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = strlen(name) + 1;

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    strncpy ((char*)args[1].argBuffer, name, strlen(name) + 1); // fzm: https://e2eprivate.ti.com/nokia_siemens_networks/k2_-_fsm4_-_fzm_-_lrc_nokia/f/191/t/7108.aspx

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_OutboundFPHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the outbound fast path. Once an outbound fast
 *      path is being deleted we need to inform the NETFP Proxy to stop monitoring the
 *      IP address. The fast path will be deleted on the response back from the NETFP
 *      Proxy. However in the case of manual override fast paths the fast path can be
 *      immediately deleted; since we dont need to inform the NETFP Proxy.
 *
 *      The return values of 0 and 1 are both treated as a success. But the return values
 *      are used internally to parse the lists.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOutboundFastPath
 *      Pointer to the OUTBOUND fast path
 *  @param[in]  reason
 *      Original reason because of which the function is invoked
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      1   - Fast path deletion has been initiated successfully
 *  @retval
 *      0   - Fast path deletion was completed
 *  @retval
 *     -1   - Fast path deletion could NOT be initiated
 */
static int32_t _Netfp_deleteOutboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_OutboundFastPath* ptrOutboundFastPath,
    Netfp_Reason            reason,
    int32_t*                errCode
)
{
    Netfp_ProxyServerInfo*      ptrProxyServerInfo;
    Netfp_ProxyServerOneMsg     proxyServerOneMsg;
    Netfp_EventMetaInfo         eventInfo;

    /* Sanity Check: Is the path valid or not? */
    if (unlikely(Netfp_isValidOutboundFastPath(ptrNetfpServer, ptrOutboundFastPath) == 0))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Was the fast path using the manual route override mode? */
    //fzm
    if (ptrOutboundFastPath->cfg.ifHandle == NULL && ptrOutboundFastPath->isSecure == 0)
    {
        /* Initialize the proxy server information, only the number to not set all array when only one item will be used */
        proxyServerOneMsg.numberOfEntries = 1;

        ptrProxyServerInfo = &proxyServerOneMsg.proxyServerInfo;

        /* Initialize the proxy server information */
        memset ((void *)ptrProxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

        /* Populate the informational block: */
        ptrProxyServerInfo->opType = Netfp_ProxyServerOp_REQUEST;
        ptrProxyServerInfo->u.req.startMonitor = 0;
        memcpy ((void *)&ptrProxyServerInfo->dstIP, (void*)&ptrOutboundFastPath->cfg.dstIP, sizeof(Netfp_IPAddr));
        memcpy ((void *)&ptrProxyServerInfo->srcIP, (void*)&ptrOutboundFastPath->cfg.srcIP, sizeof(Netfp_IPAddr));

        *errCode = Netfp_populateServerToProxyNode(ptrNetfpServer, ptrProxyServerInfo, &ptrOutboundFastPath->requestId);

        if (*errCode < 0)
        {
            Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Netfp_prepareProxyRequest failed 0x%x\n", *errCode);
        }
        else
        {
            /* Send a message to the NETFP Proxy to stop monitoring: */
            *errCode = Netfp_sendProxyRequest (ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Netfp_sendProxyRequest failed %d\n", *errCode);
            }
        }
    }
    else
    {
        /* YES. The NETFP Proxy was not involved so no need to inform the proxy */
        *errCode = 0;
    }

    /* Generate the delete fast path event and push it to all the NETFP clients */
    eventInfo.eventId            = Netfp_EventId_DELETE_FP;
    eventInfo.u.fpMeta.fpHandle  = (void*)ptrOutboundFastPath;
    eventInfo.u.fpMeta.reason    = reason;
    memset ((void*)&eventInfo.u.fpMeta.l2ConnectInfo, 0, sizeof(Netfp_SockL2ConnectInfo));
    Netfp_generateEvent (ptrNetfpServer, &eventInfo);

    /* Irrespective of the error code; the fast path is deleted from the NETFP Server. Worst case: The proxy will
     * be monitoring a fast path which does not exist. Sending a proxy request can only fail because of a JOSH error
     * which should never occur. Log the error message */
    if (*errCode < 0)
        Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Sending Proxy Request on Delete Outbound failed [Error code %d]\n", *errCode);

    /* Cleanup the fast path */
    Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList, (Netfp_ListNode*)ptrOutboundFastPath);
    Netfp_removeFPFromRecomputationList(ptrNetfpServer, ptrOutboundFastPath);
    ptrNetfpServer->cfg.free (ptrOutboundFastPath, sizeof(Netfp_OutboundFastPath));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to delete an outbound fast path
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  fpHandle
 *      Handle to the fast path to be deleted
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteOutboundFastPath
(
    Netfp_ClientHandle      clientHandle,
    Netfp_OutboundFPHandle  fpHandle,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (fpHandle == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_deleteOutboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_deleteOutboundFastPath
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_OutboundFPHandle  fpHandle,
     *  Netfp_Reason            reason,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_OutboundFPHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_EventId);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the NETFP Fast Path handle.  */
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(fpHandle);

    /* Populate the NETFP Event which is causing the deletion: */
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(Netfp_Reason_FAST_PATH_DELETE);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Outbound fast path deletion can return the following values:
     * (a) 0  - Fast Path deletion was completed
     * (b) 1  - Fast Path deletion was initiated successfully.
     * (c) -1 - Fast Path deletion could not be initiated
     *
     * Case (a) and (b) are a success for the callee. The difference between the two cases is for
     * internal NETFP List management.
     * Case (c) however needs to be reported back to the callee with the error code */
    if ((result == 0) || (result == 1))
        return 0;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      This function is used to used to determine if given the new resolved information
 *      the NETFP universe needs to be notified or not. Notifying the clients can be
 *      expensive so we will do this only if required to do so.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOutboundFastPath
 *      Pointer to the outbound fast path for which the route was resolved
 *  @param[in]  bIsSameInterface
 *      Flag which indicates if the interface handle associated with the fast path
 *      has changed or not.
 *  @param[in]  ptrResolvedInterface
 *      Pointer to the resolved interface to be used. NULL indicates that there was no
 *      next hop MAC address
 *  @param[in]  oldNextHopMACAddress
 *      Old Next HOP MAC Address which was being used
 *  @param[in]  resolvedNextHopMACAddress
 *      Pointer to the next hop MAC address to be used.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_notifyOutboundFPChanges
(
    Netfp_ServerMCB*            ptrNetfpServer,
    Netfp_OutboundFastPath*     ptrOutboundFastPath,
    uint8_t                     bIsSameInterface,
    Netfp_Interface*            ptrResolvedInterface,
    uint8_t*                    oldNextHopMACAddress,
    uint8_t*                    resolvedNextHopMACAddress
)
{
    Netfp_Status            currentStatus;
    Netfp_Status            newStatus;
    Netfp_EventMetaInfo     eventInfo;

    /* Get the current status of the outbound fast path: */
    currentStatus = ptrOutboundFastPath->status;

    /* Determine the new status of the outbound fast path */
    if (ptrResolvedInterface != NULL)
        newStatus = Netfp_Status_ACTIVE;
    else
        newStatus = Netfp_Status_ZOMBIE;

    /* Update the fast path to the new status: */
    ptrOutboundFastPath->status = newStatus;

#if 0
    /* Setup the Fast path MTU: We need to perform the fast path MTU computation once there is a status
     * change to the fast path. */
    Netfp_setupFastPathMTU (ptrNetfpServer, ptrOutboundFastPath, NETFP_INVALID_MTU);
#endif

    /* Initialize the event information block: */
    memset ((void *)&eventInfo, 0, sizeof(Netfp_EventMetaInfo));

    /* Has there been a status modification? */
    if ((currentStatus == Netfp_Status_ZOMBIE) && (newStatus == Netfp_Status_ZOMBIE))
    {
        /************************************************************************************
         * Status Quo: Once a zombie always a zombie. There is no status change so there
         * is no need to inform the NETFP universe.
         ***********************************************************************************/
        return;
    }
    else if ((currentStatus == Netfp_Status_ZOMBIE) && (newStatus == Netfp_Status_ACTIVE))
    {
        /************************************************************************************
         * Moving from an ZOMBIE state to ACTIVE: We need to inform the NETFP universe that
         * the outbound fast path is now operational.
         ***********************************************************************************/
        eventInfo.eventId               = Netfp_EventId_UPDATE_FP;
        eventInfo.u.fpMeta.reason       = Netfp_Reason_NEIGH_REACHABLE;
        eventInfo.u.fpMeta.fpHandle     = (void*)ptrOutboundFastPath;
        eventInfo.u.fpMeta.status       = newStatus;
        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);

        /* Generate the event: */
        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
        return;
    }
    else if ((currentStatus == Netfp_Status_ACTIVE) && (newStatus == Netfp_Status_ZOMBIE))
    {
        /************************************************************************************
         * Moving from an ACTIVE state to a ZOMBIE: We need to inform the NETFP universe
         * that the outbound fast path is NOT operational.
         ***********************************************************************************/
        eventInfo.eventId               = Netfp_EventId_UPDATE_FP;
        eventInfo.u.fpMeta.reason       = Netfp_Reason_NEIGH_UNREACHABLE;
        eventInfo.u.fpMeta.fpHandle     = (void*)ptrOutboundFastPath;
        eventInfo.u.fpMeta.status       = newStatus;

        /* Generate the event: */
        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
        return;
    }

    /************************************************************************************
     * We remain in an ACTIVE state. Has there been an interface change? This can happen
     * because of routing table updates
     ***********************************************************************************/
    if (bIsSameInterface == 0)
    {
        /************************************************************************************
         * Interface handle was updated: Generate an event
         ************************************************************************************/
        eventInfo.eventId               = Netfp_EventId_UPDATE_FP;
        eventInfo.u.fpMeta.reason       = Netfp_Reason_NEIGH_REACHABLE;
        eventInfo.u.fpMeta.fpHandle     = (void*)ptrOutboundFastPath;
        eventInfo.u.fpMeta.status       = newStatus;
        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);

        /* Generate the event: */
        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
        return;
    }

    /************************************************************************************
     * Has there been an update to the NEXT Hop MAC address which was being used?
     ***********************************************************************************/
    if ((oldNextHopMACAddress[0] != resolvedNextHopMACAddress[0]) ||
        (oldNextHopMACAddress[1] != resolvedNextHopMACAddress[1]) ||
        (oldNextHopMACAddress[2] != resolvedNextHopMACAddress[2]) ||
        (oldNextHopMACAddress[3] != resolvedNextHopMACAddress[3]) ||
        (oldNextHopMACAddress[4] != resolvedNextHopMACAddress[4]) ||
        (oldNextHopMACAddress[5] != resolvedNextHopMACAddress[5]))
    {
        /************************************************************************************
         * Next HOP MAC address was modified.
         ************************************************************************************/
        eventInfo.eventId               = Netfp_EventId_UPDATE_FP;
        eventInfo.u.fpMeta.reason       = Netfp_Reason_NEIGH_REACHABLE;
        eventInfo.u.fpMeta.fpHandle     = (void*)ptrOutboundFastPath;
        eventInfo.u.fpMeta.status       = newStatus;
        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);

        /* Generate the event: */
        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
        return;
    }

    /* Control comes here implies that the fast path remained in ACTIVE stage. There was no
     * change to the Interface and the Next Hop MAC address which was being used. There is no
     * need to generate an event for this since there has been no configuration change detected */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the L3 outbound fast path which allows
 *      packets to be sent out.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrOutboundFastPathCfg
 *      Pointer to the outbound fast path configuration
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Fast Path Handle
 *  @retval
 *      Error   -   NULL
 */
Netfp_OutboundFPHandle _Netfp_createOutboundFastPath
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_OutboundFPCfg*    ptrOutboundFastPathCfg,
    int32_t*                errCode
)
{
    Netfp_OutboundFastPath* ptrOutboundFastPath;
    Netfp_SPInfo*           ptrSPInfo;
    Netfp_IPSecChannel*     ptrIPSecChannel;
    Netfp_ProxyServerInfo*  ptrProxyServerInfo;
    Netfp_ProxyServerOneMsg proxyServerOneMsg;

    /* Sanity Check: Both the source & destination IP address should be of the same IP family. */
    if (unlikely(ptrOutboundFastPathCfg->dstIP.ver != ptrOutboundFastPathCfg->srcIP.ver))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Fast Path names need to be unique in the system. */
    if (unlikely(_Netfp_findOutboundFastPath (ptrNetfpServer, ptrOutboundFastPathCfg->name) != NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Special Case: The security policy identifier NETFP_INVALID_SPID is a special identifier which
     * needs to be handled differently */
    if (ptrOutboundFastPathCfg->spId != NETFP_INVALID_SPID)
    {
        /* Security Policy identifer was specified. Find this in the database */
        ptrSPInfo = (Netfp_SPInfo *)Netfp_findSPById (ptrNetfpServer, ptrOutboundFastPathCfg->spId);
        if (ptrSPInfo == NULL)
        {
            /* Security Policy does not exist in the database. This is an invalid configuration. */
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }
    else
    {
        /* Special case: There is no security policy associated with the fast path. */
        ptrSPInfo = NULL;
    }

    /* Match the security policy checks if there was a valid security policy. */
    if (ptrSPInfo != NULL)
    {
        /* Security Policy was specified. We need to ensure that the fast path IP
         * and the IP specified in the security policy belong to the same family.
         * NOTE: We have already verified that the src IP & dst IP specified in the
         * Fast path & Security Policy configuration belong to the same family.
         *
         * But before we can do the check we need to get the latest version of the
         * Security Policy by invalidating the cache. */
        if (ptrSPInfo->spCfg.dstIP.ver != ptrOutboundFastPathCfg->dstIP.ver)
        {
            *errCode = NETFP_EINVAL;
            return NULL;
        }

        /* Validate the fast path source & destination IP address against the specified security policy:
         * This will help determine if the fast path creation is allowed by the policy or not. */
        if ((Netfp_policyCheckIP (Netfp_Direction_INBOUND,  ptrSPInfo, ptrOutboundFastPathCfg->srcIP) == 0) ||
            (Netfp_policyCheckIP (Netfp_Direction_OUTBOUND, ptrSPInfo, ptrOutboundFastPathCfg->dstIP) == 0))
        {
            /* Error: Security policy did not allow the connection. */
            *errCode = NETFP_ENOPERM;
            return NULL;
        }

        /* Get the IPSEC channel associated with the security policy. */
        ptrIPSecChannel = (Netfp_IPSecChannel*)ptrSPInfo->spCfg.saHandle;
    }
    else
    {
        /* There is no IPSEC channel associated with the outbound fast path */
        ptrIPSecChannel = NULL;
    }

    /* Allocate memory for the fast path */
    ptrOutboundFastPath = ptrNetfpServer->cfg.malloc (sizeof(Netfp_OutboundFastPath), 0);
    if (ptrOutboundFastPath == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrOutboundFastPath, 0, sizeof(Netfp_OutboundFastPath));

    /* Populate the fast path information. */
    memcpy ((void *)&ptrOutboundFastPath->cfg, (void *)ptrOutboundFastPathCfg, sizeof (Netfp_OutboundFPCfg));

    /* Link the fast path with the security policy */
    ptrOutboundFastPath->ptrSPInfo = ptrSPInfo;

    /* Setup the direction: */
    ptrOutboundFastPath->direction = Netfp_Direction_OUTBOUND;

    /* Set the flag to determine if the fast path is secure or not. */
    if (ptrIPSecChannel == NULL)
        ptrOutboundFastPath->isSecure = 0;
    else
        ptrOutboundFastPath->isSecure = 1;

    /* Initialize the outbound fast path PMTU */
    ptrOutboundFastPath->fastPathPMTU = NETFP_INVALID_MTU;

    /***********************************************************************************************
     * Route Resolution Procedure:
     * (1) Manual override mode:
     *     If configured the application has resolved the route. We blindly use these parameters
     * (2) NETFP Proxy mode:
     *     In this mode we need to handle the following additional conditions:
     *     (a) Non Secure Fast Path: Fast path is responsible for route resolution
     *     (b) Secure Fast Path: Inherit the route resolution from the security association. The fast
     *         path is NOT responsible for the route resolution.
     ***********************************************************************************************/
    if (ptrOutboundFastPathCfg->ifHandle != NULL)
    {
        /* Manual Override Mode: We blindly use the next HOP MAC address. Route has been resolved. */
        ptrOutboundFastPath->status = Netfp_Status_ACTIVE;

        /* Copy over the resolution parameters: */
        ptrOutboundFastPath->ptrNetfpInterface = (Netfp_Interface*)ptrOutboundFastPathCfg->ifHandle;
        memcpy ((void*)&ptrOutboundFastPath->nextHopMACAddress[0], (void*)&ptrOutboundFastPathCfg->nextHopMACAddress[0], 6);
    }
    else
    {
        /* NETFP Proxy Mode: Determine if we need to start the route resolution procedure. */
        if (ptrOutboundFastPath->isSecure == 0)
        {
            /* Case (a) NON Secure connection: Route has NOT been resolved */
            ptrOutboundFastPath->status = Netfp_Status_ZOMBIE;

            /* Initialize the proxy server information, only the number to not set all array when only one item will be used */
            proxyServerOneMsg.numberOfEntries = 1;

            ptrProxyServerInfo = &proxyServerOneMsg.proxyServerInfo;

            /* Initialize the proxy server information */
            memset ((void *)ptrProxyServerInfo, 0, sizeof(Netfp_ProxyServerInfo));

            /* Populate the informational block: */
            ptrProxyServerInfo->opType = Netfp_ProxyServerOp_REQUEST;
            ptrProxyServerInfo->u.req.startMonitor = 1;
            memcpy ((void *)&ptrProxyServerInfo->dstIP, (void*)&ptrOutboundFastPathCfg->dstIP, sizeof(Netfp_IPAddr));
            memcpy ((void *)&ptrProxyServerInfo->srcIP, (void*)&ptrOutboundFastPathCfg->srcIP, sizeof(Netfp_IPAddr));

            *errCode = Netfp_populateServerToProxyNode(ptrNetfpServer, ptrProxyServerInfo, &ptrOutboundFastPath->requestId);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                /* Error: Route resolution failed */
                ptrNetfpServer->cfg.free (ptrOutboundFastPath, sizeof(Netfp_OutboundFastPath));

                return NULL;
            }

            /* Send a message to the NETFP Proxy to start monitoring: */
            *errCode = Netfp_sendProxyRequest (ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

            if (*errCode < 0)
            {
                Netfp_cleanupRequest(ptrNetfpServer, (Netfp_ProxyServerBulkMsg*)&proxyServerOneMsg);

                ptrNetfpServer->cfg.free (ptrOutboundFastPath, sizeof(Netfp_OutboundFastPath));

                Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Netfp_sendProxyRequest failed %d\n", *errCode);

                return NULL;
            }
        }
        else
        {
            /* Case (b) Secure connection: The fast path will inherit the route from the outbound IPSEC channel.
             * Is the Security Policy Active? */
            if (ptrSPInfo->status == Netfp_Status_ACTIVE)
            {
                /* IPSEC channel was active: The route was resolved */
                ptrOutboundFastPath->status = Netfp_Status_ACTIVE;

                /* Inherit the route resolution parameters from the IPSEC channel. */
                ptrOutboundFastPath->ptrNetfpInterface = (Netfp_Interface*)ptrIPSecChannel->ptrNetfpInterface;
                memcpy ((void*)&ptrOutboundFastPath->nextHopMACAddress[0], (void*)&ptrIPSecChannel->nextHopMACAddress[0], 6);
            }
            else
            {
                /* IPSEC channel was NOT active; so the fast path route has also not been resolved. */
                ptrOutboundFastPath->status = Netfp_Status_ZOMBIE;
            }

            /* ESP allocation for IV size + SPI + seq number */
            ptrOutboundFastPath->ptrSPInfo->ipsecHdrLen =
	            Netfp_getEncIvSize(ptrIPSecChannel) + 4 + 4;

            /* ESP trailer allocation w/o padding, +2 bytes for padding size and next header */
            ptrOutboundFastPath->ptrSPInfo->ipsecTrlLen =
                Netfp_getAuthDigestSize(ptrIPSecChannel) + 2;

            if( ptrOutboundFastPath->ptrSPInfo->ipsecHdrLen < 8 ||
                ptrOutboundFastPath->ptrSPInfo->ipsecTrlLen < 2 )
            {
	            System_printf("ERROR, invalid header/trailer size, set to default allocation!\n");
	            ptrOutboundFastPath->ptrSPInfo->ipsecHdrLen = NETFP_SA_PKT_HDR_MARGIN;
                ptrOutboundFastPath->ptrSPInfo->ipsecTrlLen = NETFP_SA_PKT_TAIL_MARGIN;
            }

        }
    }

    /* Record the fast path into the server database */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList, (Netfp_ListNode*)ptrOutboundFastPath);
    return (Netfp_OutboundFPHandle)ptrOutboundFastPath;
}

/**
 *  @b Description
 *  @n
 *      The function which is invoked by the NETFP Client to create an outbound fast path.
 *      Outbound fast paths create a Layer3 endpoint to send packets which implies that
 *      the routing tables need to be consulted in order to determine the next hop mac
 *      address.
 *
 *      Outbound fast paths are not immediately active after creation [Unless a manual
 *      route is inserted] They kick off the route resolution process and become active
 *      once the route resolution process completes successfully.
 *
 *      It is possible to create and connect sockets on an unresolved fast paths but in
 *      such cases the Netfp_send will fail.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrOutboundFastPathCfg
 *      Pointer to the outbound fast path configuration
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 * @sa
 *   Netfp_isOutboundFastPathActive
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
Netfp_OutboundFPHandle Netfp_createOutboundFastPath
(
    Netfp_ClientHandle      clientHandle,
    Netfp_OutboundFPCfg*    ptrOutboundFastPathCfg,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_OutboundFPCfg*    ptrRemoteOutboundFPCfg;
    int32_t                 jobId;
    uint32_t                result;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (ptrOutboundFastPathCfg == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_createOutboundFastPath);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * Netfp_OutboundFPHandle _Netfp_createOutboundFastPath
     * (
     * Netfp_ServerMCB*        ptrNetfpServer,
     * Netfp_OutboundFPCfg*    ptrOutboundFastPathCfg,
     * int32_t*                errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_OutboundFPCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Get the pointer to the interface configuration. */
    ptrRemoteOutboundFPCfg = (Netfp_OutboundFPCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteOutboundFPCfg, (void *)ptrOutboundFastPathCfg, sizeof (Netfp_OutboundFPCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return NULL;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Get the error code.  */
    *errCode = *((int32_t*)args[2].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return NULL;
    }
    return (Netfp_OutboundFPHandle)result;
}

/**
 *  @b Description
 *  @n
 *      The function executes on the NETFP Server and is used to determine the status
 *      of the fast path.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP server
 *  @param[in]  outboundFPHandle
 *      Output fast handle to be checked for
 *  @param[out] status
 *      Set to 1 indicating that the Fast path is active else 0. Ignore the value on
 *      error
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_isOutboundFastPathActive
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_OutboundFPHandle  outboundFPHandle,
    int32_t*                status,
    int32_t*                errCode
)
{
    Netfp_OutboundFastPath* ptrOutboundFastPath;

    /* Cycle through all the registered fast paths to verify the sanity of the outbound fast path handle */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Do we have a match? */
        if (ptrOutboundFastPath == (Netfp_OutboundFastPath*)outboundFPHandle)
        {
            /* YES: We have a match return the status */
            if (ptrOutboundFastPath->status == Netfp_Status_ACTIVE)
                *status = 1;
            else
                *status = 0;
            return 0;
        }

        /* Get the next outbound fast path: */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }

    /* Control comes here we are trying to get the status of a fast path which has not been registered or an invalid
     * handle was passed. */
    *errCode = NETFP_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check the status of the outbound fast paths.
 *      Outbound fast paths can be active or not depending upon the route
 *      resolution process.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  outboundFPHandle
 *      Output fast handle to be checked for
 *  @param[out] status
 *      Set to 1 indicating that the Fast path is active else 0. Ignore the value on
 *      error
 *  @param[out] errCode
 *      Error code populated on error.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_isOutboundFastPathActive
(
    Netfp_ClientHandle      clientHandle,
    Netfp_OutboundFPHandle  outboundFPHandle,
    int32_t*                status,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Netfp_ClientMCB*        ptrNetfpClient;
    int32_t                 jobId;
    uint32_t                result;

    /* Sanity Check: Ensure that the arguments are valid */
    if (unlikely((clientHandle == NULL) || (outboundFPHandle == NULL)))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (unlikely(ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE))
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_isOutboundFastPathActive);
    if (unlikely(jobHandle == NULL))
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     * int32_t _Netfp_isOutboundFastPathActive
     * (
     * Netfp_ServerMCB*        ptrNetfpServer,
     * Netfp_OutboundFPHandle  outboundFPHandle,
     * int32_t*                status,
     * int32_t*                errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Netfp_OutboundFPHandle);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(outboundFPHandle);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNetfpClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Extract the output arguments */
    *status  = *((int32_t*)args[2].argBuffer);
    *errCode = *((int32_t*)args[3].argBuffer);

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNetfpClient->joshHandle, jobId) < 0)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill all the outbounds fast paths registered with the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success  -   0
 *  @retval
 *      Error    -   <0
 */
int32_t Netfp_killOutboundFastPath
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    int32_t                     retVal;

    /* Close all the outbound fast paths which have been configured in the server */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
    while (ptrOutboundFastPath != NULL)
    {
        /* Delete the fast path from the server */
        retVal = _Netfp_deleteOutboundFastPath (ptrNetfpServer, ptrOutboundFastPath, Netfp_Reason_FAST_PATH_DELETE, errCode);

        /* Was the fast path deletion completed already? */
        if (retVal == 0)
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
        else
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a debug function to display the contents of the OUTBOUND
 *      fast paths
 *
 *  @param[in]  serverHandle
 *      Handle to the NETFP Server
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of fast paths which are present
 */
int32_t Netfp_displayOutboundFastPath(Netfp_ServerHandle serverHandle)
{
    Netfp_OutboundFastPath*     ptrOutboundFastPath;
    Netfp_ServerMCB*            ptrNetfpServer;
    int32_t                     count = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the OUTBOUND fast path information */
    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);

//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "Outbound Fast Path Table\n");

    /* Cycle through all the fast paths and display them on the console */
    while (ptrOutboundFastPath != NULL)
    {
        /* Display the fast path */
        if (ptrOutboundFastPath->cfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] Name: %s Policy: %d %03d.%03d.%03d.%03d -> %03d.%03d.%03d.%03d\n",
                          ptrOutboundFastPath, (ptrOutboundFastPath->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE",
                          ptrOutboundFastPath->cfg.name, ptrOutboundFastPath->cfg.spId,
                          ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[0], ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[1],
                          ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[2], ptrOutboundFastPath->cfg.srcIP.addr.ipv4.u.a8[3],
                          ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[0], ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[1],
                          ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[2], ptrOutboundFastPath->cfg.dstIP.addr.ipv4.u.a8[3]);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                          ptrOutboundFastPath->nextHopMACAddress[0], ptrOutboundFastPath->nextHopMACAddress[1],
                          ptrOutboundFastPath->nextHopMACAddress[2], ptrOutboundFastPath->nextHopMACAddress[3],
                          ptrOutboundFastPath->nextHopMACAddress[4], ptrOutboundFastPath->nextHopMACAddress[5]);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            PMTU:%d bytes Age:%d sec\n", ptrOutboundFastPath->fastPathPMTU, ptrOutboundFastPath->agePMTU);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrOutboundFastPath->cfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrOutboundFastPath->cfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] Name: %s Policy: %d %s -> %s\n",
                          ptrOutboundFastPath, (ptrOutboundFastPath->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE",
                          ptrOutboundFastPath->cfg.name, ptrOutboundFastPath->cfg.spId, srcIP, dstIP);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            [0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x]\n",
                          ptrOutboundFastPath->nextHopMACAddress[0], ptrOutboundFastPath->nextHopMACAddress[1],
                          ptrOutboundFastPath->nextHopMACAddress[2], ptrOutboundFastPath->nextHopMACAddress[3],
                          ptrOutboundFastPath->nextHopMACAddress[4], ptrOutboundFastPath->nextHopMACAddress[5]);
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "            PMTU:%d bytes Age:%d sec\n", ptrOutboundFastPath->fastPathPMTU, ptrOutboundFastPath->agePMTU);
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next fast path */
        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
    }
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is the notification function which is invoked by the modules which
 *      can reside below the fast path:
 *          - Security Policy
 *          - Interface
 *
 *      These modules use this interface API to notify the fast path module that an event
 *      has occurred which could have an impact on the operational status of the fast path.
 *      The function analyzes the impact of the event and handles it accordingly.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  reason
 *      Reason which is causing the update to the fast path
 *  @param[in]  spId
 *      [Optional] Security Policy identifier
 *  @param[in]  ptrNetfpInterface
 *      [Optional] Pointer to the NETFP Interface
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_updateFP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Reason        reason,
    uint32_t            spId,
    Netfp_Interface*    ptrNetfpInterface
)
{
    Netfp_InboundFastPath*   ptrInboundFastPath;
    Netfp_OutboundFastPath*  ptrOutboundFastPath;
    Netfp_SPInfo*            ptrSPInfo;
    int32_t                  errCode;
    Netfp_EventMetaInfo      eventInfo;
    int32_t                  retVal;
    Netfp_IPSecChannel*      ptrOutboundIPSecChannel;

    /* Processing is done on the basis of the reason: */
    switch (reason)
    {
        case Netfp_Reason_INTERFACE_DELETE:
        {
            /****************************************************************************************************
             * Interface has been deleted: Cycle through all the outbound fast paths and delete them
             ****************************************************************************************************/
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
            while (ptrOutboundFastPath != NULL)
            {
                /* Does this match the interface? */
                if (ptrOutboundFastPath->ptrNetfpInterface == ptrNetfpInterface)
                {
                    /* YES. The fast path is infected kill it */
                    retVal = _Netfp_deleteOutboundFastPath (ptrNetfpServer, ptrOutboundFastPath, reason, &errCode);

                    /* Was the fast path deletion completed already? */
                    if (retVal == 0)
                        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
                    else
                        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
                else
                {
                    /* NO. Get the next fast path */
                    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
            }
            break;
        }
        case Netfp_Reason_SA_DELETE:
        case Netfp_Reason_SP_DELETE:
        {
            /**********************************************************************************************************
             * Security Association/Security Policy have been deleted: Cycle through and delete all the fast paths
             * which use these blocks.
             **********************************************************************************************************/
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
            while (ptrOutboundFastPath != NULL)
            {
                /* Does this match the infected security policy identifier? */
                if (ptrOutboundFastPath->cfg.spId == spId)
                {
                    /* YES. The fast path is infected kill it */
                    retVal = _Netfp_deleteOutboundFastPath (ptrNetfpServer, ptrOutboundFastPath, reason, &errCode);

                    /* Was the fast path deletion completed already? */
                    if (retVal == 0)
                        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
                    else
                        ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
                else
                {
                    /* NO. Get the next fast path */
                    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
            }

            /* Cycle through all the inbound fast paths and delete them if it matches the security policy identifier */
            ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
            while (ptrInboundFastPath != NULL)
            {
                /* Does this match the infected security policy identifier? */
                if (ptrInboundFastPath->cfg.spId == spId)
                {
                    /* YES. The fast path is infected kill it  */
                    _Netfp_deleteInboundFastPath (ptrNetfpServer, ptrInboundFastPath, reason, &errCode);

                    /* Restart from the head of the list again. */
                    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
                }
                else
                {
                    /* NO. Get the next fast path */
                    ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
                }
            }
            break;
        }
        case Netfp_Reason_SP_ACTIVE:
        case Netfp_Reason_SP_INACTIVE:
        case Netfp_Reason_REKEY_SA:
        {
            /**********************************************************************************************************
             * SA has been stopped or rekeyed or has become active
             **********************************************************************************************************/
            ptrSPInfo = (Netfp_SPInfo *)Netfp_findSPById (ptrNetfpServer, spId);
            if (ptrSPInfo == NULL)
                return;

            /* Cycle through all the outbound fast paths changing their status to that of the security policy */
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
            while (ptrOutboundFastPath != NULL)
            {
                /* Does this match the updated security policy identifier? */
                if (ptrOutboundFastPath->cfg.spId == spId)
                {
                    /* YES: Get the IPSEC channel */
                    ptrOutboundIPSecChannel = (Netfp_IPSecChannel*)ptrSPInfo->spCfg.saHandle;

                    /* Is this a secure policy? */

//fzm-->
                    if (ptrOutboundFastPath->isSecure == 1 && (ptrOutboundIPSecChannel == NULL ||
                        ptrOutboundIPSecChannel->status == Netfp_Status_ZOMBIE ||
                        ptrOutboundIPSecChannel->status == Netfp_Status_STOP))
                        ptrOutboundFastPath->status = Netfp_Status_ZOMBIE;
                    else
                        ptrOutboundFastPath->status = ptrSPInfo->status;

                    if(ptrOutboundIPSecChannel != NULL && ptrOutboundIPSecChannel->status == Netfp_Status_ACTIVE &&
                        memcmp((void *)ptrOutboundFastPath->nextHopMACAddress, (void*)ptrOutboundIPSecChannel->nextHopMACAddress, 6) != 0)
                    {
                        ptrOutboundFastPath->ptrNetfpInterface = ptrOutboundIPSecChannel->ptrNetfpInterface;
                        memcpy((void *)ptrOutboundFastPath->nextHopMACAddress, (void*)ptrOutboundIPSecChannel->nextHopMACAddress, 6);
                    }
                    else if(ptrOutboundIPSecChannel != NULL &&
                            (ptrOutboundIPSecChannel->status == Netfp_Status_ZOMBIE || ptrOutboundIPSecChannel->status == Netfp_Status_STOP))
                    {
                        memset((void *)ptrOutboundFastPath->nextHopMACAddress, 0, 6);
                        ptrOutboundFastPath->ptrNetfpInterface = NULL;
                    }

                    /* Generate an event: */
                    eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                    eventInfo.u.fpMeta.reason   = reason;
                    eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                    eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                    if(ptrOutboundFastPath->isSecure && (NULL != ptrOutboundIPSecChannel))
                    {
                        memcpy ((void *)&eventInfo.u.fpMeta.srcIP, (void*)&ptrOutboundIPSecChannel->saCfg.srcIP, sizeof(Netfp_IPAddr));
                        memcpy ((void *)&eventInfo.u.fpMeta.dstIP, (void*)&ptrOutboundIPSecChannel->saCfg.dstIP, sizeof(Netfp_IPAddr));
                    } else {
                        memcpy ((void *)&eventInfo.u.fpMeta.srcIP, (void*)&ptrOutboundFastPath->cfg.srcIP, sizeof(Netfp_IPAddr));
                        memcpy ((void *)&eventInfo.u.fpMeta.dstIP, (void*)&ptrOutboundFastPath->cfg.dstIP, sizeof(Netfp_IPAddr));
                    }

                    if(ptrOutboundFastPath->status == Netfp_Status_ACTIVE)
                        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);

//fzm<--
                    /* Generate the event: */
                    Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                }

                /* Get the next outbound fast path. */
                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
            }

            /* Cycle through all the inbound fast paths changing their status to that of the security policy */
            ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundFPList);
            while (ptrInboundFastPath != NULL)
            {
                /* Does this match the infected security policy identifier? */
                if (ptrInboundFastPath->cfg.spId == spId)
                {
                    if ( ptrInboundFastPath->status == Netfp_Status_ACTIVE )
                    {
                        /* Fast Path will inherit the status from the security policy. */
                        ptrInboundFastPath->status = ptrSPInfo->status;
                    }

                    /* Generate an event: */
                    eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                    eventInfo.u.fpMeta.reason   = reason;
                    eventInfo.u.fpMeta.fpHandle = (void*)ptrInboundFastPath;
                    eventInfo.u.fpMeta.status   = ptrInboundFastPath->status;
//fzm-->
                    Netfp_IPSecChannel* ptrInboundIPSecChannel = (Netfp_IPSecChannel*)ptrSPInfo->spCfg.saHandle;
                    if(ptrInboundFastPath->isSecure && (NULL != ptrInboundIPSecChannel))
                    {
                        memcpy ((void *)&eventInfo.u.fpMeta.srcIP, (void*)&ptrInboundIPSecChannel->saCfg.srcIP, sizeof(Netfp_IPAddr));
                        memcpy ((void *)&eventInfo.u.fpMeta.dstIP, (void*)&ptrInboundIPSecChannel->saCfg.dstIP, sizeof(Netfp_IPAddr));
                    } else {
                        memcpy ((void *)&eventInfo.u.fpMeta.srcIP, (void*)&ptrInboundFastPath->cfg.srcIP, sizeof(Netfp_IPAddr));
                        memcpy ((void *)&eventInfo.u.fpMeta.dstIP, (void*)&ptrInboundFastPath->cfg.dstIP, sizeof(Netfp_IPAddr));
                    }
//fzm<--
                    /* Generate the event: */
                    Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                }

                /* Get the next fast path */
                ptrInboundFastPath = (Netfp_InboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrInboundFastPath);
            }
            break;
        }
        case Netfp_Reason_PMTU_CHANGE:
        {
            /**********************************************************************************************************
             * SA Path MTU has changed:
             **********************************************************************************************************/
            ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
            while (ptrOutboundFastPath != NULL)
            {
                /* Does this match the updated security policy identifier? */
                if (ptrOutboundFastPath->cfg.spId == spId)
                {
                    uint32_t currentPMTU;

                    /* Take a snapshot of the current fast path PMTU. */
                    currentPMTU = ptrOutboundFastPath->fastPathPMTU;

                    /* Update the Fast Path PMTU: */
                    Netfp_setupFastPathPMTU (ptrNetfpServer, ptrOutboundFastPath, NETFP_INVALID_MTU);

                    /* Do we need to generate an event to the NETFP Client? */
                    if (currentPMTU != ptrOutboundFastPath->fastPathPMTU)
                    {
                        /* YES: The fast path PMTU was changed so inform the NETFP clients about this. */
                        eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                        eventInfo.u.fpMeta.reason   = Netfp_Reason_PMTU_CHANGE;
                        eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                        eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                    }
                }
                /* Get the next outbound fast path */
                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
            }
            break;
        }
        case Netfp_Reason_IF_MTU_CHANGE:
        {
            /**********************************************************************************************************
             * Interface MTU has changed: Fast paths can be notified about this change through the following paths
             *  (a) Non Secure Fast Paths will be notified through the Interface module
             *  (b) Secure Fast Paths will be notified through the SP module
             **********************************************************************************************************/
            if (ptrNetfpInterface != NULL)
            {
                /* Case (a): Cycle through all the outbound fast paths and update the fast path MTU */
                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
                while (ptrOutboundFastPath != NULL)
                {
                    /* Is this a non-secure fast path & does it reside on the affected interface ? */
                    if ((ptrOutboundFastPath->isSecure == 0) && (ptrOutboundFastPath->ptrNetfpInterface == ptrNetfpInterface))
                    {
                        /* Generate an event to notify the NETFP clients that there was an interface MTU change  */
                        eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                        eventInfo.u.fpMeta.reason   = Netfp_Reason_IF_MTU_CHANGE;
                        eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                        eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                    }
                    /* Get the next outbound fast path */
                    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
            }
            else
            {
                /* Case (b): Cycle through all the outbound fast paths matching the security policy */
                ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundFPList);
                while (ptrOutboundFastPath != NULL)
                {
                    /* Does this match the updated security policy identifier? */
                    if (ptrOutboundFastPath->cfg.spId == spId)
                    {
                        /* Generate an event to notify the NETFP clients that there was an interface MTU change  */
                        eventInfo.eventId           = Netfp_EventId_UPDATE_FP;
                        eventInfo.u.fpMeta.reason   = Netfp_Reason_IF_MTU_CHANGE;
                        eventInfo.u.fpMeta.fpHandle = (void*)ptrOutboundFastPath;
                        eventInfo.u.fpMeta.status   = ptrOutboundFastPath->status;
                        Netfp_populateSocketL2ConnectInfo (ptrNetfpServer, ptrOutboundFastPath, &eventInfo.u.fpMeta.l2ConnectInfo);
                        Netfp_generateEvent (ptrNetfpServer, &eventInfo);
                    }
                    /* Get the next outbound fast path */
                    ptrOutboundFastPath = (Netfp_OutboundFastPath*)Netfp_listGetNext ((Netfp_ListNode*)ptrOutboundFastPath);
                }
            }
            break;
        }
        default:
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Reason [%s] is not handled in Netfp_updateFP\n", Netfp_getReasonString(reason));
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the various NETFP services
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle for which the jobs are being registered
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerFastPathServices (Josh_NodeHandle nodeHandle)
{
    /* Inbound Fast Path Services: */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_createInboundFastPath);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_findInboundFastPath);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_isInboundFastPathActive);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_deleteInboundFastPath);

    /* Outbound Fast Path Services: */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_createOutboundFastPath);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_findOutboundFastPath);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_deleteOutboundFastPath);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_isOutboundFastPathActive);
    return 0;
}

