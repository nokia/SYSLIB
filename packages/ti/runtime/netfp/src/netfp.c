/**
 *   @file  netfp.c
 *
 *   @brief
 *      The NETFP core module which implements the basic building blocks
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
 *********************** NETFP Core Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to find a security policy which matches the SPID.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  spId
 *      Security Policy Indentifier
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the matching security policy
 *  @retval
 *      Error   -   NULL
 */
Netfp_SPInfo* Netfp_findSPById
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            spId
)
{
    Netfp_SPInfo*       ptrSPInfo;

    /* Search the inbound list. */
    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
    while (ptrSPInfo != NULL)
    {
        /* Do we have a match? */
        if (ptrSPInfo->spCfg.spId == spId)
            return ptrSPInfo;

        /* Get the next security policy */
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrSPInfo);
    }

    /* Control comes here implies that there was no match in the inbound list. So we now search the outbound
     * list and see if there is anything which matches this identifier */
    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
    while (ptrSPInfo != NULL)
    {
        /* Do we have a match? */
        if (ptrSPInfo->spCfg.spId == spId)
            return ptrSPInfo;

        /* Get the next security policy */
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrSPInfo);
    }

    /* Control comes here implies that there was no match. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the policy check on the specified
 *      source IP address to determine if the address is in the range
 *      of the security policy or not?
 *
 *  @param[in]  direction
 *    - INBOUND implies that the ip address is checked against the src ip
 *      in the security policy
 *    - OUTBOUND implies that the ip address is checked against the dst ip
 *      in the security policy
 *  @param[in]  ptrSP
 *      Pointer to the Security Policy for which the check is to be done.
 *  @param[in]  ip
 *      IP address to be checked
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   1  (Security Policy allows the IP address)
 *  @retval
 *      Error   -   0  (Security Policy does not allow the IP address)
 */
uint8_t Netfp_policyCheckIP
(
    Netfp_Direction     direction,
    Netfp_SPInfo*       ptrSP,
    Netfp_IPAddr        ip
)
{
    Netfp_IPAddr*   ptrSPIPAddress;

    /* Get the security policy address using the direction. */
    if (direction == Netfp_Direction_INBOUND)
        ptrSPIPAddress = &ptrSP->spCfg.srcIP;
    else
        ptrSPIPAddress = &ptrSP->spCfg.dstIP;

    /* We can proceed only if the IP address belongs to the same family. */
    if (ip.ver != ptrSPIPAddress->ver)
        return 0;

    /* Is the matching family IPv4 or IPv6? */
    if (ip.ver == Netfp_IPVersion_IPV4)
    {
        uint32_t mask;

        /* IPv4: Compute the network mask from the security policy. */
        if (direction == Netfp_Direction_INBOUND)
            mask = Netfp_htonl (0xFFFFFFFF << (32 - ptrSP->spCfg.srcIPPrefixLen));
        else
            mask = Netfp_htonl (0xFFFFFFFF << (32 - ptrSP->spCfg.dstIPPrefixLen));

        /* Special case: If the security policy address or the ip address is not
         * specified. TODO: Only the security policy should be checked here.
         * We also check if the source ip is within the network mask range specified in
         * the security policy. */
        if ((ptrSPIPAddress->addr.ipv4.u.a32 == 0))
            return 1;
        else if ((ptrSPIPAddress->addr.ipv4.u.a32 & mask) == (ip.addr.ipv4.u.a32 & mask))
            return 1;
    }
    else
    {
        Netfp_IP6N mask;

        /* Compute the subnet mask. */
        if (direction == Netfp_Direction_INBOUND)
            Netfp_getIPv6SubnetMask(ptrSP->spCfg.srcIPPrefixLen, &mask);
        else
            Netfp_getIPv6SubnetMask(ptrSP->spCfg.dstIPPrefixLen, &mask);

        /* Special case: If the security policy address or the ip address is not
         * specified. TODO: Only the security policy should be checked here.
         * We also check if the source ip is within the network mask range specified in
         * the security policy. */
        if (Netfp_isIPv6Unspecified(&ptrSPIPAddress->addr.ipv6) == 1)
        {
            return 1;
        }
        else
        {
            /* Validate using the subnet mask bits & see if the source ip is allowed or not? */
            if ((ptrSPIPAddress->addr.ipv6.u.a32[0] & mask.u.a32[0]) == (ip.addr.ipv6.u.a32[0] & mask.u.a32[0]) &&
                (ptrSPIPAddress->addr.ipv6.u.a32[1] & mask.u.a32[1]) == (ip.addr.ipv6.u.a32[1] & mask.u.a32[1]) &&
                (ptrSPIPAddress->addr.ipv6.u.a32[2] & mask.u.a32[2]) == (ip.addr.ipv6.u.a32[2] & mask.u.a32[2]) &&
                (ptrSPIPAddress->addr.ipv6.u.a32[3] & mask.u.a32[3]) == (ip.addr.ipv6.u.a32[3] & mask.u.a32[3]))
            {
                return 1;
            }
        }
    }
    /* Not allowed. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to perform the policy check on the specified
 *      UDP port number to determine if the port is allowed or not
 *
 *  @param[in]  direction
 *    - INBOUND implies that the ports are checked against the src ports
 *      in the security policy
 *    - OUTBOUND implies that the ports are checked against the dst ports
 *      in the security policy
 *  @param[in]  ptrSP
 *      Security Policy to be checked against.
 *  @param[in]  port
 *      Port to be checked
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   1 (Policy allows usage)
 *  @retval
 *      Error   -   0 (Policy does not permit usage)
 */
int32_t Netfp_policyCheckPort
(
    Netfp_Direction     direction,
    Netfp_SPInfo*       ptrSP,
    uint16_t            port
)
{
    uint16_t    spPortStart;
    uint16_t    spPortEnd;

    /* Get the port ranges from the security policy. */
    if (direction == Netfp_Direction_INBOUND)
    {
        spPortStart = ptrSP->spCfg.srcPortStart;
        spPortEnd   = ptrSP->spCfg.srcPortEnd;
    }
    else
    {
        spPortStart = ptrSP->spCfg.dstPortStart;
        spPortEnd   = ptrSP->spCfg.dstPortEnd;
    }

    /* TODO: Only the security policy should be checked here and not the port
     * This needs to be reviewed.  */
    if ((spPortStart == 0) || (spPortEnd == 0) || (port == 0))
        return 1;
    else if ((port >= spPortStart) && (port <= spPortEnd))
        return 1;

    /* Policy does not allow usage */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add a security policy.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  ptrSPCfg
 *      Security policy configuration
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
static int32_t _Netfp_addSP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_SPCfg*        ptrSPCfg,
    int32_t*            errCode
)
{
    Netfp_SPInfo*       ptrSPInfo;
    Netfp_IPSecChannel* ptrIPSecChannel;

    /* Sanity Check: Ensure that the security policy identifier is unique in the system */
    if (Netfp_findSPById (ptrNetfpServer, ptrSPCfg->spId) != NULL)
    {
        /* Error: Duplicate security policy identifier detected. */
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Allocate memory for the NETFP Security policy information: */
    ptrSPInfo = ptrNetfpServer->cfg.malloc(sizeof(Netfp_SPInfo), 0);
    if (ptrSPInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrSPInfo, 0, sizeof(Netfp_SPInfo));

    /* Copy over the configuration */
    memcpy ((void *)&ptrSPInfo->spCfg, (void *)ptrSPCfg, sizeof(Netfp_SPCfg));

    /* Get the security association mapped to the security policy */
    ptrIPSecChannel = (Netfp_IPSecChannel*)ptrSPInfo->spCfg.saHandle;

    /* Is this a secure policy? */
    if (ptrIPSecChannel != NULL)
    {
        /* YES: Inherit the status from the security association */
        ptrSPInfo->status = ptrIPSecChannel->status;
    }
    else
    {
        /* NO: Mark the security policy as ACTIVE */
        ptrSPInfo->status = Netfp_Status_ACTIVE;
    }

    /* Register the security policy into the server database */
    if (ptrSPCfg->direction == Netfp_Direction_INBOUND)
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList, (Netfp_ListNode*)ptrSPInfo);
    else
        Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList, (Netfp_ListNode*)ptrSPInfo);

    /* Security policy has been added successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the security policy.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  spId
 *      Security policy identifier
 *  @param[in]  reason
 *      Original reason which caused the deletion of the security policy
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
static int32_t _Netfp_delSP
(
    Netfp_ServerMCB*        ptrNetfpServer,
    uint32_t                spId,
    Netfp_Reason            reason,
    int32_t*                errCode
)
{
    Netfp_SPInfo*       ptrSPInfo;

    /* Sanity Check: Find the security policy informational block */
    ptrSPInfo = Netfp_findSPById (ptrNetfpServer, spId);
    if (ptrSPInfo == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Infect the fast paths since the security policy has been deleted and so any fast path which uses
     * this security policy is also dead and can no longer be used. Pass the original event which has
     * caused all this to happen. */
    Netfp_updateFP (ptrNetfpServer, reason, spId, NULL);

    /* Remove the security policy into the server database */
    if (ptrSPInfo->spCfg.direction == Netfp_Direction_INBOUND)
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList, (Netfp_ListNode*)ptrSPInfo);
    else
        Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList, (Netfp_ListNode*)ptrSPInfo);

    /* Cleanup the security policy */
    ptrNetfpServer->cfg.free (ptrSPInfo, sizeof(Netfp_SPInfo));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The service function which is invoked by the NETFP Client to create
 *      a security policy
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  ptrSPCfg
 *      Pointer to the security policy configuration
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
int32_t Netfp_addSP
(
    Netfp_ClientHandle  clientHandle,
    Netfp_SPCfg*        ptrSPCfg,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_SPCfg*            ptrRemoteSPCfg;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrSPCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_addSP);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  Netfp_FPHandle _Netfp_addSP
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_SPCfg*            ptrSPCfg,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_SPCfg);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)ptrNetfpClient->cfg.serverHandle;

    /* Get the pointer to the interface configuration. */
    ptrRemoteSPCfg = (Netfp_SPCfg*)args[1].argBuffer;
    memcpy ((void*)ptrRemoteSPCfg, (void *)ptrSPCfg, sizeof (Netfp_SPCfg));

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
    *errCode = *((int32_t*)args[2].argBuffer);

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
 *      The function is used to delete the specific security policy identifier
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  spId
 *      Security Policy identifier which is to be deleted
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
int32_t Netfp_delSP
(
    Netfp_ClientHandle  clientHandle,
    uint32_t            spId,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_delSP);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_delSP
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  uint32_t                spId,
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
    args[1].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[2].length = sizeof(Netfp_Reason);

    /*  - Argument 3: */
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
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(spId);
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(Netfp_Reason_SP_DELETE);

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
 *      The function populates the status of the security policy.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  spId
 *      Security Policy identifier
 *  @param[out]  status
 *      Security Policy status populated by the API. 1 implies that the
 *      security policy is active else it is not active
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
int32_t _Netfp_isSPActive
(
    Netfp_ServerMCB*    ptrNetfpServer,
    uint32_t            spId,
    int32_t*            status,
    int32_t*            errCode
)
{
    Netfp_SPInfo*       ptrSPInfo;

    ptrSPInfo = Netfp_findSPById (ptrNetfpServer, spId);
    if (ptrSPInfo == NULL)
    {
        /* Error: No Security policy found */
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Return the appropriate status */
    if (ptrSPInfo->status == Netfp_Status_ACTIVE)
        *status = 1;
    else
        *status = 0;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the status of the security policy.
 *
 *  @param[in]  clientHandle
 *      NETFP Client Handle
 *  @param[in]  spId
 *      Security Policy identifier
 *  @param[out]  status
 *      Security Policy status populated by the API. 1 implies that the
 *      security policy is active else it is not active
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
int32_t Netfp_isSPActive
(
    Netfp_ClientHandle  clientHandle,
    uint32_t            spId,
    int32_t*            status,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;

    /* Sanity Check: Ensure that the arguments are valid */
    if (clientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_isSPActive);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_isSPActive
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  uint32_t                spId,
     *  int32_t*                status,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /*  - Argument 3: */
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
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(spId);

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

    /* Populate the security policy status */
    *status  = *((int32_t*)args[2].argBuffer);

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
 *      The function is used to update a security policy to use the new security association
 *      when a rekey event occurs.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  reason
 *      Reason because of which the SP is being updated
 *  @param[in]  origSAHandle
 *      Original Security association handle.
 *  @param[in]  newSAHandle
 *      New Security association handle.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_updateSP
(
    Netfp_ServerMCB*    ptrNetfpServer,
    Netfp_Reason        reason,
    Netfp_SAHandle      origSAHandle,
    Netfp_SAHandle      newSAHandle
)
{
    Netfp_SPInfo*       ptrSPInfo;
    Netfp_IPSecChannel* ptrNewIPSecChannel;
    Netfp_IPSecChannel* ptrOldIPSecChannel;
    Netfp_Status        currentStatus;
    Netfp_Status        newStatus;
    int32_t             errCode;

    /* Get the new and old IPSEC channels */
    ptrOldIPSecChannel = (Netfp_IPSecChannel*)origSAHandle;
    ptrNewIPSecChannel = (Netfp_IPSecChannel*)newSAHandle;

    /* Processing is done on the basis of the reason because of which the SP is being updated: */
    switch (reason)
    {
        case Netfp_Reason_INTERFACE_DELETE:
        case Netfp_Reason_SA_DELETE:
        {
            /*******************************************************************************************
             * Interface/SA has been deleted: We need to cycle through all the policies residing on top
             * security association and delete them.
             *******************************************************************************************/
            ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
            while (ptrSPInfo != NULL)
            {
                /* Is this a policy which has been affected? */
                if (ptrSPInfo->spCfg.saHandle == origSAHandle)
                {
                    /* SA is being deleted: This causes the security policy to be deleted */
                    _Netfp_delSP (ptrNetfpServer, ptrSPInfo->spCfg.spId, reason, &errCode);

                    /* Restart from the head of the list again. */
                    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
                    continue;
                }
                /* Get the next security policy. */
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrSPInfo);
            }

            /* Cycle through all the OUTBOUND security policies */
            ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
            while (ptrSPInfo != NULL)
            {
                /* Is this a policy which has been affected? */
                if (ptrSPInfo->spCfg.saHandle == origSAHandle)
                {
                    /* SA is being deleted: This causes the security policy to be deleted */
                    _Netfp_delSP (ptrNetfpServer, ptrSPInfo->spCfg.spId, reason, &errCode);

                    /* Restart from the head of the list again. */
                    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
                    continue;
                }
                /* Get the next security policy. */
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrSPInfo);
            }
            break;
        }
        case Netfp_Reason_IF_MTU_CHANGE:
        case Netfp_Reason_PMTU_CHANGE:
        {
            /*******************************************************************************************
             * Interface MTU or PMTU has been changed: Propogate this information to the fast path
             *******************************************************************************************/
            ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
            while (ptrSPInfo != NULL)
            {
                /* Is this a policy which has been affected? */
                if (ptrSPInfo->spCfg.saHandle == origSAHandle)
                {
                    /* YES: We need to update the fast path about this change */
                    Netfp_updateFP (ptrNetfpServer, reason, ptrSPInfo->spCfg.spId, NULL);
                }
                /* Get the next security policy. */
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrSPInfo);
            }
            break;
        }
        case Netfp_Reason_SP_INACTIVE:
        case Netfp_Reason_REKEY_SA:
        case Netfp_Reason_NEIGH_REACHABLE:
        case Netfp_Reason_NEIGH_UNREACHABLE:
        {
            /************************************************************************
             * This handles the following conditions:
             *  a) SA linked to the SP has been stopped [Stop SA]
             *  b) SA linked to the SP has been rekeyed [Rekey SA]
             *  c) SA is no longer reachable [Update Neighbor Status or Proxy Response]
             *  d) SA is now reachable [Update Neighbor Status or Proxy Response]
             ************************************************************************/
            if (ptrOldIPSecChannel->saCfg.direction == Netfp_Direction_INBOUND)
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
            else
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);

            /* Cycle through the security policy database */
            while (ptrSPInfo != NULL)
            {
                /* Did we get a match? */
                if (ptrSPInfo->spCfg.saHandle != origSAHandle)
                {
                    /* NO: Get the next security policy: */
                    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrSPInfo);
                    continue;
                }

                /* Matching security policy found: Get the current status of the security policy */
                currentStatus = ptrSPInfo->status;

                /* Determine the new status of the security policy */
                if (ptrNewIPSecChannel == NULL)
                {
                    /* No new SA Handle; the security policy is now a ZOMBIE */
                    newStatus = Netfp_Status_ZOMBIE;
                }
                else
                {
                    /* Security policy will inherit the status of the new security association */
                    newStatus = ptrNewIPSecChannel->status;
                }

                /* We will update the security policy with the new SA Handle only for the rekeying procedure: */
                if (reason == Netfp_Reason_REKEY_SA)
                    ptrSPInfo->spCfg.saHandle = newSAHandle;

                /* Update the security policy status */
                ptrSPInfo->status = newStatus;

                /******************************************************************************************
                 * Handle the state transitions:
                 *****************************************************************************************/
                //fzm
                if (((currentStatus == Netfp_Status_ZOMBIE) && (newStatus == Netfp_Status_ZOMBIE)) ||
                    ((currentStatus == Netfp_Status_ZOMBIE) && (newStatus == Netfp_Status_STOP)) ||
                    ((currentStatus == Netfp_Status_STOP) && (newStatus == Netfp_Status_ZOMBIE)) ||
                    ((currentStatus == Netfp_Status_STOP) && (newStatus == Netfp_Status_STOP)))
                {
                    /****************************************************************************************
                     * ZOMBIE/STOP -> ZOMBIE/STOP:  Once a zombie always a zombie there is no need to pass this
                     * information to the NETFP universe
                     ****************************************************************************************/
                }
                //fzm
                else if (((currentStatus == Netfp_Status_ZOMBIE)||(currentStatus == Netfp_Status_STOP)) && (newStatus == Netfp_Status_ACTIVE))
                {
                    /****************************************************************************************
                     * ZOMBIE -> ACTIVE: Notify the NETFP Universe
                     ****************************************************************************************/
                    Netfp_updateFP (ptrNetfpServer, Netfp_Reason_SP_ACTIVE, ptrSPInfo->spCfg.spId, NULL);
                }
                //fzm
                else if ((currentStatus == Netfp_Status_ACTIVE) && ((newStatus == Netfp_Status_ZOMBIE) || (newStatus == Netfp_Status_STOP)))
                {
                    /****************************************************************************************
                     * ACTIVE -> ZOMBIE: Notify the NETFP Universe
                     ****************************************************************************************/
                    Netfp_updateFP (ptrNetfpServer, Netfp_Reason_SP_INACTIVE, ptrSPInfo->spCfg.spId, NULL);
                }
                else
                {
                    /****************************************************************************************
                     * ACTIVE -> ACTIVE: We could be moving from one SA to another (after rekey) *OR* the
                     * security association could have moved from one interface to another.
                     ****************************************************************************************/
                    Netfp_updateFP (ptrNetfpServer, Netfp_Reason_SP_ACTIVE, ptrSPInfo->spCfg.spId, NULL);
                }

                /* Get the next security policy: */
                ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrSPInfo);
            }
            break;
        }
        default:
        {
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_ERROR, "Error: Update SP does not handle Reason[%s]\n", Netfp_getReasonString(reason));
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to kill all the security policies registered
 *      with the NETFP Server.
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
int32_t Netfp_killSecurityPolicy
(
    Netfp_ServerMCB*    ptrNetfpServer,
    int32_t*            errCode
)
{
    Netfp_SPInfo*           ptrSPInfo;

    /* Close all the inbound security policies which have been configured in the server */
    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
    while (ptrSPInfo != NULL)
    {
        /* Delete the security policy from the server. */
        if (_Netfp_delSP (ptrNetfpServer, ptrSPInfo->spCfg.spId, Netfp_Reason_SP_DELETE, errCode) < 0)
            return -1;

        /* Get the next security policy */
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
    }

    /* Close all the outbound security policies which have been configured in the server */
    ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
    while (ptrSPInfo != NULL)
    {
        /* Delete the security policy from the server. */
        if (_Netfp_delSP (ptrNetfpServer, ptrSPInfo->spCfg.spId, Netfp_Reason_SP_DELETE, errCode) < 0)
            return -1;

        /* Get the next security policy */
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is a utility function which is used to display a list of all the security policies
 *      applicable in the specific direction.
 *
 *  @param[in]  serverHandle
 *      NETFP Server Handle
 *  @param[in]  direction
 *      Direction for which the security policy are to be displayed
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Number of security policies which have been displayed
 */
int32_t Netfp_displaySP(Netfp_ServerHandle serverHandle, Netfp_Direction direction)
{
    Netfp_SPInfo*       ptrSPInfo;
    Netfp_ServerMCB*    ptrNetfpServer;
    int32_t             count = 0;

    /* Get the server handle. */
    ptrNetfpServer = (Netfp_ServerMCB*)serverHandle;
    if (ptrNetfpServer == NULL)
        return 0;

    /* Get the fast path information */
    if (direction == Netfp_Direction_INBOUND)
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrInboundSPList);
    else
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpServer->ptrOutboundSPList);
//fzm - use Netfp_dumpMsg to capture this with the USR1 signal
    /* Display the banner. */
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO, "**********************************************************\n");
    Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                  "%s Security Policy Table\n", (direction == Netfp_Direction_INBOUND) ? "Inbound" : "Outbound");

    /* Cycle through all the security policies and display them on the console */
    while (ptrSPInfo != NULL)
    {
        /* Display the security policy */
        if (ptrSPInfo->spCfg.srcIP.ver == Netfp_IPVersion_IPV4)
        {
            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] SPID: %d Src IP %d.%d.%d.%d -> Dst IP %d.%d.%d.%d Src Port %d-%d Dst Port %d-%d SA 0x%x\n",
                          ptrSPInfo, (ptrSPInfo->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE", ptrSPInfo->spCfg.spId,
                          ptrSPInfo->spCfg.srcIP.addr.ipv4.u.a8[0], ptrSPInfo->spCfg.srcIP.addr.ipv4.u.a8[1],
                          ptrSPInfo->spCfg.srcIP.addr.ipv4.u.a8[2], ptrSPInfo->spCfg.srcIP.addr.ipv4.u.a8[3],
                          ptrSPInfo->spCfg.dstIP.addr.ipv4.u.a8[0], ptrSPInfo->spCfg.dstIP.addr.ipv4.u.a8[1],
                          ptrSPInfo->spCfg.dstIP.addr.ipv4.u.a8[2], ptrSPInfo->spCfg.dstIP.addr.ipv4.u.a8[3],
                          ptrSPInfo->spCfg.srcPortStart, ptrSPInfo->spCfg.srcPortEnd,
                          ptrSPInfo->spCfg.dstPortStart, ptrSPInfo->spCfg.dstPortEnd,
                          ptrSPInfo->spCfg.saHandle);
        }
        else
        {
            char    srcIP[40];
            char    dstIP[40];

            /* Convert the Src & Dst IP address of the SP into strings using the NETFP IPv6 Utility API */
            Netfp_convertIP6ToStr (ptrSPInfo->spCfg.srcIP.addr.ipv6, &srcIP[0]);
            Netfp_convertIP6ToStr (ptrSPInfo->spCfg.dstIP.addr.ipv6, &dstIP[0]);

            Netfp_dumpMsg (ptrNetfpServer, Netfp_LogLevel_INFO,
                          "0x%x: [%s] SPID: %d Src IP %s/%d -> %s/%d Src Port %d-%d Dst Port %d-%d SA 0x%x\n",
                          ptrSPInfo, (ptrSPInfo->status == Netfp_Status_ACTIVE) ? "ACTIVE" : "ZOMBIE", ptrSPInfo->spCfg.spId,
                          srcIP, ptrSPInfo->spCfg.srcIPPrefixLen,
                          dstIP, ptrSPInfo->spCfg.dstIPPrefixLen,
                          ptrSPInfo->spCfg.srcPortStart, ptrSPInfo->spCfg.srcPortEnd,
                          ptrSPInfo->spCfg.dstPortStart, ptrSPInfo->spCfg.dstPortEnd,
                          ptrSPInfo->spCfg.saHandle);
        }

        /* Increment the counter */
        count = count + 1;

        /* Get the next fast path */
        ptrSPInfo = (Netfp_SPInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrSPInfo);
    }
    return count;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a flow matching the specific name.
 *      Flows are local to a NETFP client however the name space for
 *      flow names has to be unique.
 *
 *  @param[in]  clientHandle
 *      Handle to the NETFP client
 *  @param[in]  flowName
 *      Name of the flow
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - Flow Identifier
 *  @retval
 *      Error   - <0
 */
int32_t Netfp_findFlow
(
    Netfp_ClientHandle  clientHandle,
    const char*         flowName,
    int32_t*            errCode
)
{
    Netfp_FlowInfo*     ptrFlowInfo;
    Netfp_ClientMCB*    ptrNetfpClient;
    void*               context;

    /* Sanity Check: Validate the arguments: */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Single Core Critical Section Enter: */
    context = ptrNetfpClient->cfg.enterCS();

    /* Get the flow information: */
    ptrFlowInfo = (Netfp_FlowInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpClient->ptrFlowList);
    while (ptrFlowInfo != NULL)
    {
        /* Match the name: */
        if (strcmp (ptrFlowInfo->name, flowName) == 0)
            break;

        /* Get the next element in the list. */
        ptrFlowInfo = (Netfp_FlowInfo*)Netfp_listGetNext((Netfp_ListNode*)ptrFlowInfo);
    }

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    /* Did we find a match? */
    if (ptrFlowInfo == NULL)
    {
        /* No matching entry found. */
        *errCode = NETFP_ENOTFOUND;
        return -1;
    }

    /* Return the found flow identifier. */
    return ptrFlowInfo->cppiFlowId;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a named flow. Flows are local to a
 *      specific NETFP client and are not accessible from other clients
 *      or from the server.
 *
 *  @param[in]  clientHandle
 *      Handle of the NETFP client for which the flow is being created
 *  @param[in]  ptrFlowCfg
 *      Pointer to the flow configuration.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - Flow Identifier
 *  @retval
 *      Error   - <0
 */
int32_t Netfp_createFlow
(
    Netfp_ClientHandle  clientHandle,
    Netfp_FlowCfg*      ptrFlowCfg,
    int32_t*            errCode
)
{
    Netfp_FlowInfo*     ptrFlowInfo;
    Netfp_ClientMCB*    ptrNetfpClient;
    Qmss_Queue          queueInfo;
    void*               context;
    uint8_t             isAllocated;

    /* Sanity Check: Make sure we were passed valid parameters */
    if ((ptrFlowCfg == NULL) || (clientHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the arguments. */
    if ((ptrFlowCfg->numHeaps == 0) || (ptrFlowCfg->numHeaps > NETFP_MAX_HEAP_HANDLE))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: Heap Handle 0 should always exist. */
    if (ptrFlowCfg->heapHandle[0] == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Check for a duplicate flow name? Each flow name is to be unique. Duplicate
     * names are not supported */
    if (Netfp_findFlow (ptrNetfpClient, ptrFlowCfg->name, errCode) >= 0)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Allocate memory for the flow */
    ptrFlowInfo = (Netfp_FlowInfo*)ptrNetfpClient->cfg.malloc (sizeof(Netfp_FlowInfo), 0);
    if (ptrFlowInfo == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return -1;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrFlowInfo, 0, sizeof(Netfp_FlowInfo));

    /* Let CPPI pick the next available flow */
    ptrFlowInfo->rxFlowCfg.flowIdNum         = CPPI_PARAM_NOT_SPECIFIED;

    /* The destination of the receive flow is not known we will let the
     * queue information in the routing entry determine the actual queue
     * where the packet is to be pushed. */
    ptrFlowInfo->rxFlowCfg.rx_dest_qmgr      = 0x0;
    ptrFlowInfo->rxFlowCfg.rx_dest_qnum      = 0x0;

    /* Setup the SOP & Descriptor type. */
    ptrFlowInfo->rxFlowCfg.rx_sop_offset     = ptrFlowCfg->sopOffset;
    ptrFlowInfo->rxFlowCfg.rx_desc_type      = Cppi_DescType_HOST;

    /* Enable PS info */
    ptrFlowInfo->rxFlowCfg.rx_ps_location    = Cppi_PSLoc_PS_IN_DESC;
    ptrFlowInfo->rxFlowCfg.rx_psinfo_present = 0x1;

    /* Drop the packet, NO retry on starvation */
    ptrFlowInfo->rxFlowCfg.rx_error_handling = 0x0;

    /* EPIB info present */
    ptrFlowInfo->rxFlowCfg.rx_einfo_present  = 0x1;

    /* Disable tagging */
    ptrFlowInfo->rxFlowCfg.rx_dest_tag_lo_sel= 0x4;
    ptrFlowInfo->rxFlowCfg.rx_dest_tag_hi_sel= 0x0;
    ptrFlowInfo->rxFlowCfg.rx_src_tag_lo_sel = 0x0;
    ptrFlowInfo->rxFlowCfg.rx_src_tag_hi_sel = 0x0;

    /* Check the number of heaps which have been specified? */
    if (ptrFlowCfg->numHeaps == 1)
    {
        /* 1 Heap */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0_en= 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1_en= 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2_en= 0x0;

        /* No threshold sizes need to be configured. */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0   = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1   = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2   = 0x0;

        /* Free Descriptor Queues for Size 1, 2 and 3 do NOT need to be configured. */
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qnum  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qnum  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qnum  = 0x0;
    }
    else if (ptrFlowCfg->numHeaps == 2)
    {
        /* 2 Heaps */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0_en= 0x1;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1_en= 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2_en= 0x0;

        /* Setup the threshold sizes. */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0   = ptrFlowCfg->threshold[1];
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1   = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2   = 0x0;

        /* Free Descriptor Queue for Size 1 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qmgr  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));

        /* Free Descriptor Queue for Size 2 and 3 do NOT need to be configured. */
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qnum  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qnum  = 0x0;
    }
    else if (ptrFlowCfg->numHeaps == 3)
    {
        /* 3 Heaps */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0_en= 0x1;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1_en= 0x1;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2_en= 0x0;

        /* Setup the threshold sizes. */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0   = ptrFlowCfg->threshold[1];
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1   = ptrFlowCfg->threshold[2];
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2   = 0x0;

        /* Free Descriptor Queue for Size 1 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qmgr  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));

        /* Free Descriptor Queue for Size 2 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[2]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qmgr  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[2]));

        /* Free Descriptor Queue for Size 3 do NOT need to be configured. */
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qmgr  = 0x0;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qnum  = 0x0;
    }
    else
    {
        /* 4 Heaps */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0_en= 0x1;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1_en= 0x1;
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2_en= 0x1;

        /* Setup the threshold sizes. */
        ptrFlowInfo->rxFlowCfg.rx_size_thresh0   = ptrFlowCfg->threshold[1];
        ptrFlowInfo->rxFlowCfg.rx_size_thresh1   = ptrFlowCfg->threshold[2];
        ptrFlowInfo->rxFlowCfg.rx_size_thresh2   = ptrFlowCfg->threshold[3];

        /* Free Descriptor Queue for Size 1 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qmgr  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz1_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[1]));

        /* Free Descriptor Queue for Size 2 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[2]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qmgr  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz2_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[2]));

        /* Free Descriptor Queue for Size 3 is configured. */
        queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[3]));
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qnum  = queueInfo.qMgr;
        ptrFlowInfo->rxFlowCfg.rx_fdq0_sz3_qmgr  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[3]));
    }

    /* Heap Handle 0 has always got to be specified and is always written into the
     * Free Descriptor Queue for Size 0. */
    queueInfo = Qmss_getQueueNumber(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[0]));
    ptrFlowInfo->rxFlowCfg.rx_fdq0_sz0_qmgr  = queueInfo.qMgr;
    ptrFlowInfo->rxFlowCfg.rx_fdq0_sz0_qnum  = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[0]));

    /* The Heap Handle 0 is also always used to pick all 'chained' packets. */
    ptrFlowInfo->rxFlowCfg.rx_fdq1_qnum = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[0]));
    ptrFlowInfo->rxFlowCfg.rx_fdq1_qmgr = queueInfo.qMgr;
    ptrFlowInfo->rxFlowCfg.rx_fdq2_qnum = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[0]));
    ptrFlowInfo->rxFlowCfg.rx_fdq2_qmgr = queueInfo.qMgr;
    ptrFlowInfo->rxFlowCfg.rx_fdq3_qnum = Qmss_getQIDFromHandle(Pktlib_getInternalHeapQueue(ptrFlowCfg->heapHandle[0]));
    ptrFlowInfo->rxFlowCfg.rx_fdq3_qmgr = queueInfo.qMgr;

    /* Create the flow */
    ptrFlowInfo->flowHnd = Cppi_configureRxFlow(ptrNetfpClient->passCPDMAHandle, &ptrFlowInfo->rxFlowCfg, &isAllocated);
    if (ptrFlowInfo->flowHnd == NULL)
    {
        /* Error: Unable to open the flow. */
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Copy the flow name: */
    strncpy (ptrFlowInfo->name, ptrFlowCfg->name, NETFP_MAX_CHAR);
    ptrFlowInfo->cppiFlowId = Cppi_getFlowId(ptrFlowInfo->flowHnd);

    /* Single Core Critical Section Enter: */
    context = ptrNetfpClient->cfg.enterCS();

    /* Add the flow to the client list */
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpClient->ptrFlowList, (Netfp_ListNode*)ptrFlowInfo);

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    /* Return the flow identifier. */
    return ptrFlowInfo->cppiFlowId;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a named flow.
 *
 *  @param[in]  clientHandle
 *      Handle of the NETFP client for which the flow is being created
 *  @param[in]  ptrFlowName
 *      Pointer to the flow name to be deleted
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Netfp_deleteFlow
(
    Netfp_ClientHandle  clientHandle,
    const char*         ptrFlowName,
    int32_t*            errCode
)
{
    Netfp_FlowInfo*     ptrFlowInfo;
    void*               context;
    Netfp_ClientMCB*    ptrNetfpClient;

    /* Sanity Check: Validate the arguments */
    if ((clientHandle == NULL) || (ptrFlowName == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the client MCB */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;

    /* Single Core Critical Section Enter: */
    context = ptrNetfpClient->cfg.enterCS();

    /* Search through all the flows: */
    ptrFlowInfo = (Netfp_FlowInfo*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpClient->ptrFlowList);
    while (ptrFlowInfo != NULL)
    {
        /* Do we have a match? */
        if(strncmp (ptrFlowInfo->name, ptrFlowName, NETFP_MAX_CHAR) == 0)
        {
            /* Remove the node from the list */
            Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpClient->ptrFlowList, (Netfp_ListNode*)ptrFlowInfo);

            /* Close the flow */
            Cppi_closeRxFlow (ptrFlowInfo->flowHnd);

            /* Cleanup memory */
            ptrNetfpClient->cfg.free (ptrFlowInfo, sizeof(Netfp_FlowInfo));
            break;
        }

        /* Get the next flow information block. */
        ptrFlowInfo = (Netfp_FlowInfo*)Netfp_listGetNext ((Netfp_ListNode*)ptrFlowInfo);
    }

    /* Single Core Critical Section Exit: */
    ptrNetfpClient->cfg.exitCS(context);

    /* Did we get a match? */
    if (ptrFlowInfo == NULL)
    {
        /* Error: No match found. There was no flow created with the name */
        *errCode = NETFP_EINVAL;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function registers the NETFP Proxy client with the NETFP Server.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[in]  proxyClientName
 *      NETFP Proxy client name
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Netfp_registerProxyService
(
    Netfp_ServerMCB*        ptrNetfpServer,
    const char*             proxyClientName,
    int32_t*                errCode
)
{
    int32_t     clientBlockIndex;

    /* Cycle through all the clients to determine which client is being stopped. */
    for (clientBlockIndex = 0; clientBlockIndex < NETFP_MAX_CLIENTS; clientBlockIndex++)
    {
        /* Is the client active? */
        if (ptrNetfpServer->clientBlock[clientBlockIndex].status != Netfp_ClientStatus_ACTIVE)
            continue;

        /* Active Client: Is this the NETFP proxy client? */
        if (strncmp (ptrNetfpServer->clientBlock[clientBlockIndex].name, proxyClientName, NETFP_MAX_CHAR) == 0)
        {
            /* YES. Remember the NETFP Proxy client. There can only be 1 NETFP Proxy registered with the server */
            if (ptrNetfpServer->proxyClientBlock != NULL)
            {
                *errCode = NETFP_EINUSE;
                return -1;
            }

            /* Register the proxy service. */
            ptrNetfpServer->proxyClientBlock = &ptrNetfpServer->clientBlock[clientBlockIndex];

            /* Debug Message: */
            Netfp_logMsg (ptrNetfpServer, Netfp_LogLevel_DEBUG,
                          "Debug: Registered %s as the NETFP Proxy client\n", proxyClientName);
            return 0;
        }
    }

    /* Control comes here implies that an invalid proxy client name was specified or the PROXY was still not active. */
    *errCode = NETFP_EINVAL;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the NETFP Proxy client with the
 *      NETFP Server. NETFP Proxy is a privilidged client which provides
 *      important services. This API should NOT be called by other NETFP
 *      clients since this will break important internal NETFP features.
 *
 *      There can only be 1 NETFP Proxy associated per server.
 *
 *  @param[in]  proxyNetfpClientHandle
 *      NETFP Proxy client handle
 *  @param[in]  ptrProxyCfg
 *      Pointer to the NETFP Proxy configuration
 *  @param[out] errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_registerProxyService
(
    Netfp_ClientHandle      proxyNetfpClientHandle,
    Netfp_ProxyCfg*         ptrProxyCfg,
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
    if (proxyNetfpClientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: NETFP Proxy needs to be provide support for all the functions. */
    if (ptrProxyCfg->proxyServerInterfaceFunction == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)proxyNetfpClientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Copy over the proxy configuration. */
    memcpy ((void *)&ptrNetfpClient->proxyCfg, (void *)ptrProxyCfg, sizeof(Netfp_ProxyCfg));

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_registerProxyService);
    if (jobHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Netfp_registerProxyService
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  char*                   proxyClientName,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = NETFP_MAX_CHAR;

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(int32_t);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNetfpClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NETFP_EJOSH;
        return -1;
    }

    /* Populate the NETFP Server handle */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

    /* Populate the NETFP Proxy client name */
    strncpy ((char*)&args[1].argBuffer[0], &ptrNetfpClient->cfg.clientName[0], NETFP_MAX_CHAR);

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
    *errCode = *((int32_t*)args[2].argBuffer);

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
int32_t Netfp_registerServices (Josh_NodeHandle nodeHandle)
{
    /* Event Services: */
    if (Netfp_registerEventMgmtServices (nodeHandle) < 0)
        return -1;

    /* Core PA Services: */
    if (Netfp_registerPAServices (nodeHandle) < 0)
        return -1;

    /* Interface Services: */
    if (Netfp_registerInterfaceServices (nodeHandle) < 0)
        return -1;

    /* Fastpath Services: */
    if (Netfp_registerFastPathServices (nodeHandle) < 0)
        return -1;

    /* Socket Services: */
    if (Netfp_registerSocketServices (nodeHandle) < 0)
        return -1;

    /* SA Services: */
    if (Netfp_registerSAServices (nodeHandle) < 0)
        return -1;

    /* IPSEC Services: */
    if (Netfp_registerIPSecServices (nodeHandle) < 0)
        return -1;

    /* Proxy Server Services: */
    if (Netfp_registerProxyServerServices (nodeHandle) < 0)
        return -1;

    /* Multicast Services: */
    if (Netfp_registerMulticastServices (nodeHandle) < 0)
        return -1;

    /* Core Services: */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_addSP);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_delSP);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_isSPActive);
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_registerProxyService);

    /* Client-Server Services: */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_stopClient);

    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_getServerStatus);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility Function to dump memory on the console.
 *
 *  @param[in]  cp
 *      Pointer to the memory to be dumped
 *  @param[in]  length
 *      Length of the memory to be dumped
 *  @param[in]  prefix
 *      Prefix information to be printed.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_xdump(uint8_t*  cp, int  length, const char*  prefix )
{
    int32_t col, count;
    char prntBuf[120];
    char*  pBuf = prntBuf;
    count = 0;
    while(count < length){
        pBuf += sprintf( pBuf, "%s", prefix );
        for(col = 0;count + col < length && col < 16; col++){
            if (col != 0 && (col % 4) == 0)
                pBuf += sprintf( pBuf, " " );
            pBuf += sprintf( pBuf, "%02X ", cp[count + col] );
        }
        while(col++ < 16){      /* pad end of buffer with blanks */
            if ((col % 4) == 0)
                sprintf( pBuf, " " );
            pBuf += sprintf( pBuf, "   " );
        }
        pBuf += sprintf( pBuf, "  " );
        for(col = 0;count + col < length && col < 16; col++){
            if (isprint((int)cp[count + col]))
                pBuf += sprintf( pBuf, "%c", cp[count + col] );
            else
                pBuf += sprintf( pBuf, "." );
                }
        // SPrint(prntBuf);
        System_printf("%s\n", prntBuf);
        count += col;
        pBuf = prntBuf;
    }
}
