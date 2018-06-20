/**
 *   @file  netfp_multicast.c
 *
 *   @brief
 *      The file implements the Multicast functionality in NETFP.
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* MCSDK Include Files. */
#include <ti/csl/csl_cache.h>
#include <ti/drv/cppi/cppi_drv.h>

/* SYSLIB Include Files. */
#include <ti/runtime/pktlib/pktlib.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 **************************** MBMS Functions ******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used by the NETFP client to initialize the multicast
 *      services.
 *
 *  @param[in]  ptrNetfpServer
 *      Pointer to the NETFP Server
 *  @param[out]  ptrMulticastInfo
 *      Pointer to the multicast Information populated by the API
 *  @param[out] errCode
 *      Error code populated by the API
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
static int32_t _Netfp_initMulticastServices
(
    Netfp_ServerMCB*        ptrNetfpServer,
    Netfp_MulticastInfo*    ptrMulticastInfo,
    int32_t*                errCode
)
{
    /* Has the multicast services on the NETFP Server been initialized? */
    if (ptrNetfpServer->multicastInfo.isInitialized == 1)
    {
        /* YES: Simply copy over the informational block and return back to the NETFP Client */
        memcpy ((void *)ptrMulticastInfo, (void *)&ptrNetfpServer->multicastInfo, sizeof(Netfp_MulticastInfo));
        return 0;
    }

    /* Create the MAC Virtual Link for IPv4 Multicast */
    if (Netfp_createMACVirtualLink (ptrNetfpServer, &ptrNetfpServer->multicastInfo.vlinkIPv4Handle,
                                    &ptrNetfpServer->multicastInfo.vlinkIPv4Id, errCode) < 0)
        return -1;

    /* Create the MAC Virtual Link for IPv6 Multicast */
    if (Netfp_createMACVirtualLink (ptrNetfpServer, &ptrNetfpServer->multicastInfo.vlinkIPv6Handle,
                                    &ptrNetfpServer->multicastInfo.vlinkIPv6Id, errCode) < 0)
        return -1;

    /* Multicast services have been initialized successfully on the server: */
    ptrNetfpServer->multicastInfo.isInitialized = 1;

    /* Report back to the NETFP Client */
    memcpy ((void *)ptrMulticastInfo, (void*)&ptrNetfpServer->multicastInfo, sizeof(Netfp_MulticastInfo));

    /* Multicast services initialized successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to initialize the multicast services with the specific NETFP client.
 *
 *  @param[in]  netfpClientHandle
 *      Handle to the NETFP client which is to be executed.
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_initMulticastServices
(
    Netfp_ClientHandle  netfpClientHandle,
    int32_t*            errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_MulticastInfo*    ptrRemoteMulticastInfo;
    int32_t                 jobId;
    uint32_t                result;

    /* Sanity Check: Ensure that the arguments are valid */
    if (netfpClientHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP Client MCB: Services are available only if the client is active. */
    ptrNetfpClient = (Netfp_ClientMCB*)netfpClientHandle;
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return -1;
    }

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_initMulticastServices);
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
     * int32_t _Netfp_initMulticastServices
     * (
     * Netfp_ServerMCB*         ptrNetfpServer,
     * Netfp_MulticastInfo*     ptrMulticastInfo,
     * int32_t*                 errCode
     * )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_MulticastInfo);

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

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(ptrNetfpClient->cfg.serverHandle);

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
    ptrRemoteMulticastInfo = (Netfp_MulticastInfo*)args[1].argBuffer;
    *errCode = *((int32_t*)args[2].argBuffer);

    /* If the operation was successful; copy over the Multicast information into the client */
    if ((int32_t)result == 0)
        memcpy ((void *)&ptrNetfpClient->multicastInfo, (void *)ptrRemoteMulticastInfo, sizeof(Netfp_MulticastInfo));

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
 *      The function is used to register the various MBMS services
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
int32_t Netfp_registerMulticastServices (Josh_NodeHandle nodeHandle)
{
    /* MBMS Services: */
    Josh_registerJob(nodeHandle, (Josh_JobProtype)_Netfp_initMulticastServices);
    return 0;
}

