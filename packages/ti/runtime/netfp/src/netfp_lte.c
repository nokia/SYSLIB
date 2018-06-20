/**
 *   @file  netfp_lte.c
 *
 *   @brief
 *      The file implements the LTE functionality in NETFP.
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
 **************************** LTE Functions *******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to configure the GTPU control message handling.
 *      GTPU control messages are placed into the specific queues as specified
 *      in the control configuration.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  ptrGTPUControlCfg
 *      Pointer to the GTPU control configuration.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_configureGTPUControlMessage
(
    Netfp_ClientHandle          clientHandle,
    Netfp_GTPUControlCfg*       ptrGTPUControlCfg,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Netfp_ClientMCB*        ptrNetfpClient;
    Netfp_GTPUControlCfg*   ptrRemoteGTPUControlCfg;

    /* Initialize the error code */
    *errCode = 0;

    /* Sanity Check: Ensure that the arguments are valid */
    if ((clientHandle == NULL) || (ptrGTPUControlCfg == NULL))
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
    jobHandle = Josh_findJobByAddress(ptrNetfpClient->joshHandle, (Josh_JobProtype)_Netfp_configureGTPUControlMessage);
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
     *  int32_t _Netfp_configureGTPUControlMessage
     *  (
     *  Netfp_ServerMCB*        ptrNetfpServer,
     *  Netfp_GTPUControlCfg*   ptrGTPUControlCfg,
     *  int32_t*                errCode
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(Netfp_ServerHandle);

    /*  - Argument 2:  */
    args[1].type   = Josh_ArgumentType_PASS_BY_REF;
    args[1].length = sizeof(Netfp_GTPUControlCfg);

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

    /* Populate the GTPU control configuration. */
    ptrRemoteGTPUControlCfg = (Netfp_GTPUControlCfg*)args[1].argBuffer;
    memcpy ((void *)ptrRemoteGTPUControlCfg, (void *)ptrGTPUControlCfg, sizeof(Netfp_GTPUControlCfg));

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
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a user and register it with the NETFP
 *      module.
 *
 *  @param[in]  clientHandle
 *      NETFP client handle
 *  @param[in]  ptrUserCfg
 *      Pointer to the user configuration.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the user
 *  @retval
 *      Error   -   NULL
 */
Netfp_UserHandle Netfp_createUser
(
    Netfp_ClientHandle      clientHandle,
    Netfp_UserCfg*          ptrUserCfg,
    int32_t*                errCode
)
{
    Netfp_User*                     ptrNetfpUser;
    Netfp_ClientMCB*                ptrNetfpClient;
    uint32_t                        index;
    void*                           context;
    Netfp_UserSRBInfo               userSrbInfo;

    /* Get the handle to the NETFP client */
    ptrNetfpClient = (Netfp_ClientMCB*)clientHandle;
    if (ptrNetfpClient == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Services are available only if the client is active. */
    if (ptrNetfpClient->status != Netfp_ClientStatus_ACTIVE)
    {
        *errCode = NETFP_ENOTREADY;
        return NULL;
    }

    /* Sanity Check: SNOW3G Ciphering and authentication are supported only if the
     * software SNOW3G modules have been registered with the client. */
    if (ptrUserCfg->authMode == Netfp_3gppAuthMode_EIA1)
    {
        if (ptrNetfpClient->f9 == NULL)
        {
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }
    if (ptrUserCfg->srbCipherMode == Netfp_3gppCipherMode_EEA1)
    {
        if (ptrNetfpClient->f8 == NULL)
        {
            *errCode = NETFP_EINVAL;
            return NULL;
        }
    }

    /* Allocate memory for the user */
    ptrNetfpUser = ptrNetfpClient->cfg.malloc(sizeof(Netfp_User), 0);
    if (ptrNetfpUser == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrNetfpUser, 0, sizeof(Netfp_User));

    /* Populate the NETFP User block:*/
    memcpy ((void *)&ptrNetfpUser->cfg, (void *)ptrUserCfg, sizeof(Netfp_UserCfg));
    ptrNetfpUser->ptrNetfpClient = ptrNetfpClient;

    /* Do we need to create authentication and ciphering channels for AES and NULL. SNOW3G is handled in the software */
    if ((ptrNetfpUser->cfg.authMode != Netfp_3gppAuthMode_EIA1) && (ptrNetfpUser->cfg.srbCipherMode != Netfp_3gppCipherMode_EEA1))
    {
        /* memset the structure passed to Server */
        memset ((void *)&userSrbInfo, 0, sizeof(Netfp_UserSRBInfo));

        /* Intialize the information passed to Server */
        userSrbInfo.SRBAuthInfo[1].destQueueHnd = ptrNetfpClient->F9ToF8QueueHnd[1];
        userSrbInfo.SRBAuthInfo[2].destQueueHnd = ptrNetfpClient->F9ToF8QueueHnd[2];

        userSrbInfo.SRBCipherInfo[1].destQueueHnd = ptrNetfpClient->F8ToF9QueueHnd[1];
        userSrbInfo.SRBCipherInfo[2].destQueueHnd = ptrNetfpClient->F8ToF9QueueHnd[2];

        /* Create the security channels for SRB authentication and ciphering */
        if (Netfp_createSrb (ptrNetfpClient, ptrUserCfg, &userSrbInfo, errCode) < 0)
            return NULL;

        /* Update all book keeping information for SRB AES and NULL authentication channels */
        for (index = 1; index < 3; index++)
        {
            /* Initialize the user which owns the SRB authentication channel */
            ptrNetfpUser->SRBAuthSecurityChannel[index].ptrNetfpUser             = ptrNetfpUser;
            ptrNetfpUser->SRBAuthSecurityChannel[index].ptrNetfpClient           = ptrNetfpClient;
            ptrNetfpUser->SRBAuthSecurityChannel[index].srvSecurityChannelHandle = userSrbInfo.SRBAuthInfo[index].srvSecurityChannelHandle;
            memcpy(&ptrNetfpUser->SRBAuthSecurityChannel[index].swInfo, &userSrbInfo.SRBAuthInfo[index].swInfo, sizeof(Netfp_SecuritySwInfo));

            /* Initialize the user which owns the SRB ciphering channel */
            ptrNetfpUser->SRBCipherSecurityChannel[index].ptrNetfpUser = ptrNetfpUser;
            ptrNetfpUser->SRBCipherSecurityChannel[index].ptrNetfpClient           = ptrNetfpClient;
            ptrNetfpUser->SRBCipherSecurityChannel[index].srvSecurityChannelHandle = userSrbInfo.SRBCipherInfo[index].srvSecurityChannelHandle;
            memcpy(&ptrNetfpUser->SRBCipherSecurityChannel[index].swInfo, &userSrbInfo.SRBCipherInfo[index].swInfo, sizeof(Netfp_SecuritySwInfo));
        }
    }

    /* The user is now active: */
    ptrNetfpUser->status = Netfp_Status_ACTIVE;

    /* Single Core Critical Section: Add the user to the list of users */
    context = ptrNetfpClient->cfg.enterCS();
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpClient->ptrUserList, (Netfp_ListNode*)ptrNetfpUser);
    ptrNetfpClient->cfg.exitCS(context);

    /* User has been created successfully. */
    return (Netfp_UserHandle)ptrNetfpUser;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a created user from the NETFP module
 *
 *  @param[in]  ueHandle
 *      NETFP User handle to be removed
 *  @param[out]  errCode
 *      Error code populated on error
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteUser
(
    Netfp_UserHandle        ueHandle,
    int32_t*                errCode
)
{
    Netfp_User*                     ptrNetfpUser;
    uint32_t                        index;
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    void*                           context;
    Netfp_ClientMCB*                ptrNetfpClient;
    Netfp_UserSRBInfo               userSrbInfo;

    /* Get the NETFP User */
    ptrNetfpUser = (Netfp_User*)ueHandle;
    if (ptrNetfpUser == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* The user is now a zombie: */
    ptrNetfpUser->status = Netfp_Status_ZOMBIE;

    /* Get the NETFP Client associated with the user */
    ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;

    /* Do we need to create authentication and ciphering channels for AES and NULL. SNOW3G is handled in the software */
    if ((ptrNetfpUser->cfg.authMode != Netfp_3gppAuthMode_EIA1) && (ptrNetfpUser->cfg.srbCipherMode != Netfp_3gppCipherMode_EEA1))
    {
        /* memset the structure passed to Server */
        memset ((void *)&userSrbInfo, 0, sizeof(Netfp_UserSRBInfo));

        /* Delete the SRB authentication channels: */
        for (index = 1; index < 3; index++)
        {
            /* Get the handle to the SRB authentication and ciphering client security channel. */
            userSrbInfo.SRBAuthInfo[index].srvSecurityChannelHandle = ptrNetfpUser->SRBAuthSecurityChannel[index].srvSecurityChannelHandle;
            userSrbInfo.SRBCipherInfo[index].srvSecurityChannelHandle = ptrNetfpUser->SRBCipherSecurityChannel[index].srvSecurityChannelHandle;
        }

        /* Delete the SRB security channels */
        if (Netfp_deleteSrb (ptrNetfpClient, &userSrbInfo, errCode) < 0)
        {
            /* Error: SRB channel deletion failed */
            return -1;
        }
    }

    /* Cycle through and delete the DRB ciphering channels: */
    ptr3GPPSecurityChannel = (Netfp_3GPPSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpUser->ptrDRBSecurityChannelList);
    while (ptr3GPPSecurityChannel != NULL)
    {
        /* Valid DRB 3GPP channel exists: Delete the channel before we bring down the user. */
        if (Netfp_deleteLTEChannel (ptr3GPPSecurityChannel->ptrNetfpSocket, errCode) < 0)
            return -1;

        /* Get the next DRB ciphering channel */
        ptr3GPPSecurityChannel = (Netfp_3GPPSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpUser->ptrDRBSecurityChannelList);
    }

    /* Single Core Critical Section: Remove the user from the list */
    context = ptrNetfpClient->cfg.enterCS();
    Netfp_listRemoveNode ((Netfp_ListNode**)&ptrNetfpClient->ptrUserList, (Netfp_ListNode*)ptrNetfpUser);
    ptrNetfpClient->cfg.exitCS(context);

    /* Cleanup the user */
    ptrNetfpClient->cfg.free (ptrNetfpUser, sizeof(Netfp_User));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a 3GPP Security channel which matches the radio bearer ID.
 *
 *  @param[in]  ptrNetfpUser
 *      Pointer to the NETFP User
 *  @param[in]  rbId
 *      Data radio bearer.
 *
 *  \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the matching security channel
 *  @retval
 *      Error   -   No matching entry found
 */
static Netfp_3GPPSecurityChannel* Netfp_findLTEChannelById
(
    Netfp_User* ptrNetfpUser,
    uint8_t     rbId
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    void*                           context;

    /* We can find an LTE channel only if the user is active. */
    if (ptrNetfpUser->status != Netfp_Status_ACTIVE)
        return NULL;

    /* Single Core Critical Section: The DRB channels need to be protected against concurrent access */
    context = ptrNetfpUser->ptrNetfpClient->cfg.enterCS();
    ptr3GPPSecurityChannel = (Netfp_3GPPSecurityChannel*)Netfp_listGetHead ((Netfp_ListNode**)&ptrNetfpUser->ptrDRBSecurityChannelList);
    while (ptr3GPPSecurityChannel != NULL)
    {
        /* Do we have a match? */
        if (ptr3GPPSecurityChannel->rbId == rbId)
            break;

        /* Get the next 3GPP channel */
        ptr3GPPSecurityChannel = (Netfp_3GPPSecurityChannel*)Netfp_listGetNext ((Netfp_ListNode*)ptr3GPPSecurityChannel);
    }
    ptrNetfpUser->ptrNetfpClient->cfg.exitCS(context);
    return ptr3GPPSecurityChannel;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a 3GPP LTE channel for a specific user.
 *      Each DRB is associated with a unique LTE channel.
 *
 *  @param[in]  ueHandle
 *      NETFP user on which the channel is being added
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[in]  family
 *      Socket family associated with the LTE channel
 *  @param[in]  ptrLTEChannelBindCfg
 *      Pointer to the LTE channel binding configuration which describes the
 *      local characteristics of the 3GPP LTE channel which are used to receive
 *      data from the core network
 *  @param[in]  ptrLTEChannelConnectCfg
 *      Pointer to the LTE channel connect configuration which describes the remote
 *      characteristics of the remote peer. This is used to send data to the core
 *      network.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Socket handle associated with the 3GPP channel
 *  @retval
 *      Error   -   NULL
 */
Netfp_SockHandle Netfp_createLTEChannel
(
    Netfp_UserHandle                ueHandle,
    uint8_t                         rbId,
    Netfp_SockFamily                family,
    Netfp_LTEChannelBindCfg*        ptrLTEChannelBindCfg,
    Netfp_LTEChannelConnectCfg*     ptrLTEChannelConnectCfg,
    int32_t*                        errCode
)
{
    Netfp_SecChannelCfg             secChannelCfg;
    Netfp_User*                     ptrNetfpUser;
    Netfp_Socket*                   ptrNetfpSocket;
    Netfp_OptionTLV                 optCfg;
    Netfp_SockAddr                  sockAddr;
    int32_t                         stubErrCode;
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    uint8_t                         isAllocated;
    void*                           context;

    /* Sanity Check: Validate the parameters */
    if (ueHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: This is used add only data radio bearers. SRB are added automatically once a
     * user is created */
    if (rbId < 1 || rbId > NETFP_MAX_DRB)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: LTE channels are bidirectional and both the bind and connect configurations need
     * to be specified. */
    if ((ptrLTEChannelBindCfg == NULL) || (ptrLTEChannelConnectCfg == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* TODO Check if RB is already added? */

    /* Get the NETFP user information */
    ptrNetfpUser = (Netfp_User*)ueHandle;

    /* Allocate memory for the client security channel: */
    ptr3GPPSecurityChannel = ptrNetfpUser->ptrNetfpClient->cfg.malloc (sizeof(Netfp_3GPPSecurityChannel), 0);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptr3GPPSecurityChannel, 0, sizeof(Netfp_3GPPSecurityChannel));

    /* Copy over the configuration: */
    memcpy ((void *)&ptr3GPPSecurityChannel->bindCfg,    (void*)ptrLTEChannelBindCfg,    sizeof(Netfp_LTEChannelBindCfg));
    memcpy ((void *)&ptr3GPPSecurityChannel->connectCfg, (void*)ptrLTEChannelConnectCfg, sizeof(Netfp_LTEChannelConnectCfg));

    /* Remember the user which has created the 3GPP channel */
    ptr3GPPSecurityChannel->ptrNetfpUser   = ptrNetfpUser;
    ptr3GPPSecurityChannel->rbId		   = rbId;
    ptr3GPPSecurityChannel->ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;

    /* Store the current Mark */
    ptr3GPPSecurityChannel->currentMark = 0;

    /* Populate the security channel configuration */
    memset ((void *)&secChannelCfg, 0, sizeof(Netfp_SecChannelCfg));

    /* Populate the security channel configuration for ciphering */
    secChannelCfg.type                       = Netfp_SecurityChannelType_AIR_F8;
    secChannelCfg.u.f8Cfg.ueId               = ptrNetfpUser->cfg.ueId;
    secChannelCfg.u.f8Cfg.qci                = ptrLTEChannelConnectCfg->qci;
    secChannelCfg.u.f8Cfg.isDataRadioBearer  = 1;
    secChannelCfg.u.f8Cfg.cipherMode         = ptrNetfpUser->cfg.drbCipherMode;
    secChannelCfg.u.f8Cfg.encodeFlowId       = ptrLTEChannelBindCfg->flowId;
    secChannelCfg.u.f8Cfg.decodeFlowId       = ptrLTEChannelConnectCfg->flowId;
    secChannelCfg.u.f8Cfg.countC             = ptrLTEChannelBindCfg->countC;
    secChannelCfg.u.f8Cfg.rbId               = rbId;
    secChannelCfg.u.f8Cfg.encodeQueue        = ptrLTEChannelBindCfg->chDrbEnc;
    secChannelCfg.u.f8Cfg.decodeQueue        = ptrLTEChannelConnectCfg->chDrbDec;
    secChannelCfg.u.f8Cfg.currentMark        = ptr3GPPSecurityChannel->currentMark;
    memcpy ((void *)&secChannelCfg.u.f8Cfg.hKeyUpEnc, (void *)ptrNetfpUser->cfg.hKeyUpEnc, 16);

    /* Create the security channel for the radio bearer */
    ptr3GPPSecurityChannel->srvSecurityChannelHandle = Netfp_createSecurityChannel (ptrNetfpUser->ptrNetfpClient, &secChannelCfg,
                                                                                    &ptr3GPPSecurityChannel->swInfo, errCode);
    if (ptr3GPPSecurityChannel->srvSecurityChannelHandle == NULL)
    {
        /* Error: Unable to create the security channel; error code is already populated */
        ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Is there a handover in progress? */
    if (ptrLTEChannelBindCfg->isHOInProgress == 1)
    {
        /* YES: Open an internal handover queue */
        ptr3GPPSecurityChannel->hoDataBufferQueue = Qmss_queueOpenInGroup(0, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                          QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (ptr3GPPSecurityChannel->hoDataBufferQueue < 0)
        {
            /* Error: Unable to open buffering queue. Cleanup after ourselves */
            *errCode = NETFP_EINTERNAL;
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }

    /* Open a queue to buffer packets. This will be used to buffer packets during re-establishment. */
    if (ptrLTEChannelBindCfg->enableFastPath == 1)
    {
        ptr3GPPSecurityChannel->reEstDataBufferQueue = Qmss_queueOpenInGroup(0, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                             QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (ptr3GPPSecurityChannel->reEstDataBufferQueue < 0)
        {
            /* Error: Unable to open buffering queue. Cleanup after ourselves */
            *errCode = NETFP_EINTERNAL;
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }

    /* Create and register the socket which is used to send & receive data from the core network */
    ptrNetfpSocket = Netfp_socket (ptrNetfpUser->ptrNetfpClient, family, errCode);
    if (ptrNetfpSocket == NULL)
    {
        /* Error: Unable to create the socket. */
        ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Convert the LTE binding configuration to the socket address structure.
     * - LTE channels always use GTPU port
     * - The queue handle is configured to be the ROHC queue because in the
     *   case of non fast path this where we expect the packets to land up. */
    sockAddr.sin_family              = family;
    sockAddr.op.bind.flowId          = ptrLTEChannelBindCfg->flowId;
    sockAddr.op.bind.notifyFunction  = ptrLTEChannelBindCfg->notifyFunction;
    sockAddr.op.bind.inboundFPHandle = ptrLTEChannelBindCfg->fpHandle;
    sockAddr.sin_port                = 2152;
    sockAddr.sin_gtpuId              = ptrLTEChannelBindCfg->sin_gtpuId;

    /****************************************************************************************
     * Bind the socket: We need to handle different scenarios here:
     * Case (1): Normal Operation
     *           (a) Fast Path Channel
     *           (b) Slow Path Channel
     * Case (2): Handover Operation
     *           (a) Fast Path Channel
     *           (b) Slow Path Channel
     ****************************************************************************************/
    if ((ptrLTEChannelBindCfg->isHOInProgress == 0) && (ptrLTEChannelBindCfg->enableFastPath == 0))
    {
        /******************************************************************************************
         * Case (1)(a): Slow Path Channel is being created under normal conditions. Program
         * the LUT2 to match the GTPU Identifier and send the matched packets to the ROHC Queue.
         ******************************************************************************************/
        sockAddr.op.bind.queueHandle = ptrLTEChannelBindCfg->chDrbRohc;
        sockAddr.op.bind.appInfo     = ((ptrNetfpUser->cfg.ueId << 16) | (ptrLTEChannelConnectCfg->qci << 8) | (rbId << 2));

        /* Bind the socket: */
        if (Netfp_bind(ptrNetfpSocket, &sockAddr, errCode) < 0)
        {
            /* Error: Unable to bind the socket. Cleanup after ourselves. */
            Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }
    else if ((ptrLTEChannelBindCfg->isHOInProgress == 0) && (ptrLTEChannelBindCfg->enableFastPath == 1))
    {
        /******************************************************************************************
         * Case (1)(b): Fast Path Channel is being created under normal conditions. Program the
         * LUT2 to match the GTPU Identifier and send the matched packets to the SA for ciphering.
         ******************************************************************************************/
        sockAddr.op.bind.queueHandle = ptrNetfpUser->ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX];

        /* Secure Bind: */
        if (Netfp_secureBind (ptrNetfpSocket, &sockAddr, ptr3GPPSecurityChannel->srvSecurityChannelHandle, errCode) < 0)
        {
            /* Error: Unable to secure bind the socket. Cleanup after ourselves. Use the stub error code since
             * we dont want to loose the real reason for the failure. */
            Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }
    else if ((ptrLTEChannelBindCfg->isHOInProgress == 1) && (ptrLTEChannelBindCfg->enableFastPath == 0))
    {
        /******************************************************************************************
         * Case (2)(a): Slow Path Channel is being created while a Handover is in progress. Program
         * the LUT2 to match the GTPU Identifier and send the matched packets to the internal HO
         * queue.
         ******************************************************************************************/
        sockAddr.op.bind.queueHandle = ptr3GPPSecurityChannel->hoDataBufferQueue;
        sockAddr.op.bind.appInfo     = ((ptrNetfpUser->cfg.ueId << 16) | (ptrLTEChannelConnectCfg->qci << 8) | (rbId << 2));

        /* Bind the socket: */
        if (Netfp_bind(ptrNetfpSocket, &sockAddr, errCode) < 0)
        {
            /* Error: Unable to bind the socket. Cleanup after ourselves. */
            Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }

        /* Update the channel status to indicate the channel was added with HandOver data */
        ptr3GPPSecurityChannel->state = Netfp_ChannelState_HO_INITIATED;
    }
    else
    {
        /******************************************************************************************
         * Case (2)(b): Fast Path Channel is being created while the Handover is in progress.
         * Program the LUT2 to match the GTPU Identifiers and mark the packets but dont sent it to
         * the SA
         ******************************************************************************************/
        sockAddr.op.bind.queueHandle = ptr3GPPSecurityChannel->hoDataBufferQueue;

        /* Secure Bind: */
        if (Netfp_secureBind (ptrNetfpSocket, &sockAddr, ptr3GPPSecurityChannel->srvSecurityChannelHandle, errCode) < 0)
        {
            /* Error: Unable to secure bind the socket. Cleanup after ourselves. Use the stub error code since
             * we dont want to loose the real reason for the failure. */
            Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
            ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }

    /* Convert the LTE connect configuration to the socket address structure.
     * - LTE channels always use GTPU port */
    sockAddr.sin_family                  = family;
    sockAddr.sin_port                    = 2152;
    sockAddr.sin_gtpuId                  = ptrLTEChannelConnectCfg->sin_gtpuId;
    sockAddr.op.connect.outboundFPHandle = ptrLTEChannelConnectCfg->fpHandle;

    /* Connect the socket to the remote peer. */
    if (Netfp_connect(ptrNetfpSocket, &sockAddr, errCode) < 0)
    {
        /* Error: Unable to connect the socket. Cleanup after ourselves */
        Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
        ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Setup the socket priority: This is the DSCP to be placed into the IP header of the
     * packet sent via the specified socket. */
    optCfg.type     =   Netfp_Option_PRIORITY;
    optCfg.length   =   sizeof (uint8_t);
    optCfg.value    =   (void *)&ptrLTEChannelConnectCfg->dscp;
    if (Netfp_setSockOpt (ptrNetfpSocket, &optCfg, errCode) < 0)
    {
        /* Error: Unable to set socket options. Cleanup after ourselves */
        Netfp_closeSocket(ptrNetfpSocket, &stubErrCode);
        ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Create the links between the security channels and socket. */
    ptrNetfpSocket->ptr3GPPSecurityChannel = ptr3GPPSecurityChannel;
    ptr3GPPSecurityChannel->ptrNetfpSocket = ptrNetfpSocket;

    /* Single Core Critical Section: Register the DRB channel with the User */
    context = ptrNetfpUser->ptrNetfpClient->cfg.enterCS();
    Netfp_listAdd ((Netfp_ListNode**)&ptrNetfpUser->ptrDRBSecurityChannelList, (Netfp_ListNode*)ptr3GPPSecurityChannel);
    ptrNetfpUser->ptrNetfpClient->cfg.exitCS(context);

    return (Netfp_SockHandle)ptr3GPPSecurityChannel->ptrNetfpSocket;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the LTE channel. The API should be used to delete
 *      LTE DRB channels which had been created.
 *
 *  @param[in]  socketHandle
 *      Socket handle associated with the LTE channel
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_deleteLTEChannel
(
    Netfp_SockHandle  socketHandle,
    int32_t*          errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    Netfp_User*                     ptrNetfpUser;
    Netfp_Socket*                   ptrNetfpSocket;
    Ti_Pkt*                         ptrPkt;
    void*                           context;

    /* Get the socket information */
    ptrNetfpSocket = (Netfp_Socket*)socketHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpSocket == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity Check: The function is applicable only for 3GPP LTE channels */
    if (ptrNetfpSocket->ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP 3GPP security channel information */
    ptr3GPPSecurityChannel = ptrNetfpSocket->ptr3GPPSecurityChannel;

    /* Check if the channel is HandOver is in progress. We don't want to delete channels where HO is in progress.
     * The application is expected to complete the processing of buffered HandOver packets before deleting the channel */
    if (ptr3GPPSecurityChannel->state == Netfp_ChannelState_HO_INPROGRESS)
    {
        *errCode = NETFP_EINUSE;
        return -1;
    }

    /* Get the NETFP 3GPP user information */
    ptrNetfpUser = ptr3GPPSecurityChannel->ptrNetfpUser;

    /* Single Core Critical Section: Remove the channel from the User DRB List */
    context = ptrNetfpUser->ptrNetfpClient->cfg.enterCS();
    Netfp_listRemoveNode((Netfp_ListNode**)&ptrNetfpUser->ptrDRBSecurityChannelList, (Netfp_ListNode*)ptr3GPPSecurityChannel);
    ptrNetfpUser->ptrNetfpClient->cfg.exitCS(context);

    /* Clear up the 3GPP security channel from the socket. We have already addressed the 3GPP security
     * channel so there is no need to keep the association. */
    ptrNetfpSocket->ptr3GPPSecurityChannel = NULL;
    if (Netfp_closeSocket (ptrNetfpSocket, errCode) < 0)
        return -1;

    /* Delete the DRB ciphering security channel */
    if (Netfp_deleteSecurityChannel (ptrNetfpUser->ptrNetfpClient, ptr3GPPSecurityChannel->srvSecurityChannelHandle,
                                     errCode) < 0)
    {
        /* Error: DRB ciphering security channel deletion failed */
        return -1;
    }

    /**************************************************************************************
     * Flush out any packet in the reestablishment queue: The queue is created and used
     * only for fast path channels.
     **************************************************************************************/
    if (ptr3GPPSecurityChannel->bindCfg.enableFastPath == 1)
    {
        while (1)
        {
            /* Pop off and cleanup the packets from the reestablishment queue. */
            ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (ptr3GPPSecurityChannel->reEstDataBufferQueue));
            if (ptrPkt == NULL)
                break;

            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
             * to ensure that the packet contents here are invalidated
            ******************************************************************************/
            Pktlib_getOwnership(ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
            Pktlib_freePacket (ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
        }
        /* Close the reestablishment queues */
        Qmss_queueClose (ptr3GPPSecurityChannel->reEstDataBufferQueue);
    }

    /**************************************************************************************
     * Flush out any packet in the Handover queue: The queue is created and used only if
     * the handover is in progress
     **************************************************************************************/
    if (ptr3GPPSecurityChannel->bindCfg.isHOInProgress == 1)
    {
        while (1)
        {
            /* Pop off and cleanup the packets from the reestablishment queue. */
            ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (ptr3GPPSecurityChannel->hoDataBufferQueue));
            if (ptrPkt == NULL)
                break;

            /******************************************************************************
             * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
             * to ensure that the packet contents here are invalidated
            ******************************************************************************/
            Pktlib_getOwnership(ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
            Pktlib_freePacket (ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
        }
        /* Close the handover queue. */
        Qmss_queueClose (ptr3GPPSecurityChannel->hoDataBufferQueue);
    }

    /* Cleanup the memory associated with the 3GPP security channel. */
    ptrNetfpUser->ptrNetfpClient->cfg.free (ptr3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The API integrity protects and ciphers the control plane PDU in the downlink path.
 *      The API performs F9 and F8 on the PDU as per the channel configuration. The resultant
 *      packet is placed into the encode MSGCOM channel which was registered during user
 *      creation
 *
 *      In case of SNOW3G the packet MUST contain 4 extra bytes to insert the MAC-I value.
 *      In case of AES-CMAC, the flow which is used to receive the packet MUST contain 4 extra
 *      bytes to insert the MAC-I value.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  ptrPkt
 *      Pointer to the packet which should be pointing to the PDCP header.
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  rbId
 *      Radio Bearer Identifier which should be 1 (SRB1) and 2 (SRB2)
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_encodeSrb
(
    Netfp_UserHandle        ueHandle,
    Ti_Pkt*                 ptrPkt,
    uint32_t                countC,
    uint8_t                 rbId
)
{
    Netfp_User*                     ptrNetfpUser;
    Netfp_ClientMCB*                ptrNetfpClient;
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    uint32_t                        psInfo[2];
    uint32_t                        psInfoIndex = 0;
    Netfp_3gppCipherMode            cipherMode;
    Netfp_3gppAuthMode              authMode;
    uint32_t                        payloadLength;
    uint8_t*                        ptrDataBuffer;
    uint32_t                        swInfo;
    uint32_t                        macI;
    uint8_t*                        ptrMacI;
    int32_t                         fresh;
    Qmss_QueueHnd                   F9ToF8QueueHandle;
    Qmss_QueueHnd                   resultQueueHandle;

    /* Validate the arguments: */
    if ((ueHandle == NULL) || (ptrPkt == NULL))
        return NETFP_EINVAL;

    /* Validate the arguments: Only SRB's are handled here. */
    if ((rbId != 1) && (rbId != 2))
        return NETFP_EINVAL;

    /* Get the NETFP user and client */
    ptrNetfpUser   = (Netfp_User*)ueHandle;
    ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;

    /* If the user is NOT active we cannot proceed. */
    if (ptrNetfpUser->status != Netfp_Status_ACTIVE)
        return NETFP_EINVAL;

    /* Get the authentication & ciphering mode */
    authMode   = ptrNetfpUser->cfg.authMode;
    cipherMode = ptrNetfpUser->cfg.srbCipherMode;

    /* Initialize the queues handles on the basis of the bearer identifier. */
    if (rbId == 1)
    {
        resultQueueHandle = ptrNetfpUser->cfg.chSrb1Enc;
        F9ToF8QueueHandle = ptrNetfpClient->F9ToF8QueueHnd[1];
    }
    else
    {
        resultQueueHandle = ptrNetfpUser->cfg.chSrb2Enc;
        F9ToF8QueueHandle = ptrNetfpClient->F9ToF8QueueHnd[2];
    }

    /* Handle the supported cases. */
    if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Authentication : NULL
         * Ciphering      : NULL
         **********************************************************************************/

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to the result queue */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA1) && (cipherMode == Netfp_3gppCipherMode_EEA1))
    {
        /**********************************************************************************
         * Authentication : SNOW3G
         * Ciphering      : SNOW3G
         **********************************************************************************/
        fresh = ((rbId - 1) & 0x1F) << 27;

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

        /* Perform the SNOW3G F9 operation on the packet. */
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 1,
                            ptrDataBuffer, payloadLength * 8, &macI);

    	/* Convert to Network Byte order */
	    macI = Netfp_htonl (macI);

        /* Get the pointer to the MAC-I */
        ptrMacI = (uint8_t*)&macI;

        /* Append MAC-I value to the end of data. We copy the MAC-I byte by byte and dont
         * perform a memory copy because we need to consider the case where the data buffer is
         * misaligned etc. */
        *(uint8_t *)(ptrDataBuffer + payloadLength)     = *ptrMacI;
        *(uint8_t *)(ptrDataBuffer + payloadLength + 1) = *(ptrMacI + 1);
        *(uint8_t *)(ptrDataBuffer + payloadLength + 2) = *(ptrMacI + 2);
        *(uint8_t *)(ptrDataBuffer + payloadLength + 3) = *(ptrMacI + 3);

        /* The new payload length now includes the MAC-I. This is what will be sent to the F8 for
         * encryption. */
        payloadLength = payloadLength + 4;

        /* Set the packet & data buffer length to include MAC-I. */
        Pktlib_setPacketLen(ptrPkt, payloadLength);
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, ptrDataBuffer, payloadLength);

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

        /* Perform the ciphering SNOW3G F8 operation:
         * NOTE: Ciphering is done only on the PDCP payload; so we skip the PDCP header. */
        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 1,
                            ptrDataBuffer + 1, ((payloadLength - 1) * 8));

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA2) && (cipherMode == Netfp_3gppCipherMode_EEA2))
    {
        /**********************************************************************************
         * Authentication : AES
         * Ciphering      : AES
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBAuthSecurityChannel[rbId];

        /* Get the packet length */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for authentication */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);

        /* Loop around; waiting for the authentication operation to complete. */
        while (1)
        {
            /* Is the queue empty? */
            if (Qmss_getQueueEntryCount (F9ToF8QueueHandle) != 0)
                break;
        }

        /* Control comes here implies that we got the packet after authentication. */
        ptrPkt = (Ti_Pkt *)QMSS_DESC_PTR(Qmss_queuePop(F9ToF8QueueHandle));

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
         * to ensure that the packet contents here are invalidated
         ******************************************************************************/
        Pktlib_getOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* We now need to perform the ciphering operation on the packet. */
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBCipherSecurityChannel[rbId];

        /* Get the payload length from the descriptor */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfoIndex = 0;
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for ciphering */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA1) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Authentication : SNOW3G
         * Ciphering      : NULL
         **********************************************************************************/
        fresh = ((rbId - 1) & 0x1F) << 27;

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

        /* Perform the SNOW3G F9 operation on the packet. */
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 1,
                            ptrDataBuffer, payloadLength * 8, &macI);

    	/* Convert to Network Byte order */
	    macI = Netfp_htonl (macI);

        /* Get the pointer to the MAC-I */
        ptrMacI = (uint8_t*)&macI;

        /* Append MAC-I value to the end of data. We copy the MAC-I byte by byte and dont
         * perform a memory copy because we need to consider the case where the data buffer is
         * misaligned etc. */
        *(uint8_t *)(ptrDataBuffer + payloadLength)     = *ptrMacI;
        *(uint8_t *)(ptrDataBuffer + payloadLength + 1) = *(ptrMacI + 1);
        *(uint8_t *)(ptrDataBuffer + payloadLength + 2) = *(ptrMacI + 2);
        *(uint8_t *)(ptrDataBuffer + payloadLength + 3) = *(ptrMacI + 3);

        /* The new payload length now includes the MAC-I. */
        payloadLength = payloadLength + 4;

        /* Set the packet & data buffer length to include MAC-I. */
        Pktlib_setPacketLen(ptrPkt, payloadLength);
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, ptrDataBuffer, payloadLength);

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA2) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Authentication : AES
         * Ciphering      : NULL
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBAuthSecurityChannel[rbId];

        /* Get the packet length */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt,(uint8_t*)&ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for authentication */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA1))
    {
        /**********************************************************************************
         * Authentication : NULL
         * Ciphering      : SNOW3G
         *
         * Perform the SNOW3G F8 operation but skip the PDCP Header
         **********************************************************************************/

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 1,
                            ptrDataBuffer + 1, ((payloadLength - 1) * 8));

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA2))
    {
        /**********************************************************************************
         * Authentication : NULL
         * Ciphering      : AES
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBCipherSecurityChannel[rbId];

        /* Get the payload length from the descriptor */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfoIndex = 0;
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for ciphering */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
        return 0;
    }

    /* This is NOT a supported combination. */
    return NETFP_ENOTIMPL;
}

/**
 *  @b Description
 *  @n
 *      The API ciphers the control PDU in the uplink path. The API performs F8 and F9
 *      on the PDU as per the channel configuration. The resultant packet is placed
 *      into the decode MSGCOM channel which was specified in the User configuration.
 *      The output PDU will still contain the PDCP header and the 4 byte MAC-I.
 *      Applications can verify the status of the decode using the following API
 *      @sa Netfp_getPacketError.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  ptrPkt
 *      Pointer to the packet which should be pointing to the PDCP header.
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  rbId
 *      Radio Bearer Identifier which should be 1 (SRB1) and 2 (SRB2)
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_decodeSrb
(
    Netfp_UserHandle    ueHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            countC,
    uint8_t             rbId
)
{
    Netfp_ClientMCB*                ptrNetfpClient;
    Netfp_User*                     ptrNetfpUser;
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    uint32_t                        psInfo[2];
    uint32_t                        psInfoIndex = 0;
    uint32_t                        payloadLength;
    uint32_t                        swInfo;
    uint8_t*                        ptrDataBuffer;
    uint32_t                        macI;
    uint32_t                        rxMacI;
    int32_t                         fresh;
    Netfp_3gppCipherMode            cipherMode;
    Netfp_3gppAuthMode              authMode;
    Qmss_QueueHnd                   F8ToF9QueueHandle;
    Qmss_QueueHnd                   resultQueueHandle;

    /* Validate the arguments: */
    if ((ueHandle == NULL) || (ptrPkt == NULL))
        return -1;

    /* Validate the arguments: Only SRB's are handled here. */
    if ((rbId != 1) && (rbId != 2))
        return NETFP_EINVAL;

    /* Get the NETFP user and client */
    ptrNetfpUser   = (Netfp_User*)ueHandle;
    ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;

    /* If the user is NOT active we cannot proceed. */
    if (ptrNetfpUser->status != Netfp_Status_ACTIVE)
        return NETFP_EINVAL;

    /* Get the authentication & ciphering mode */
    authMode   = ptrNetfpUser->cfg.authMode;
    cipherMode = ptrNetfpUser->cfg.srbCipherMode;

    /* Initialize the queues handles on the basis of the bearer identifier. */
    if (rbId == 1)
    {
        resultQueueHandle = ptrNetfpUser->cfg.chSrb1Dec;
        F8ToF9QueueHandle = ptrNetfpClient->F8ToF9QueueHnd[1];
    }
    else
    {
        resultQueueHandle = ptrNetfpUser->cfg.chSrb2Dec;
        F8ToF9QueueHandle = ptrNetfpClient->F8ToF9QueueHnd[2];
    }

    /* Handle the supported cases. */
    if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Ciphering      : NULL
         * Authentication : NULL
         **********************************************************************************/

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)&swInfo);
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to the result queue */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA1) && (cipherMode == Netfp_3gppCipherMode_EEA1))
    {
        /**********************************************************************************
         * Ciphering      : SNOW3G
         * Authentication : SNOW3G
         *
         * Skip the PDCP Header and then perform the SNOW3G F8 operation.
         **********************************************************************************/

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

#ifdef TEST_DECRYPT
        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 1,
                            ptrDataBuffer + 1, (payloadLength - 1) * 8);
#else
        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 0,
                            ptrDataBuffer + 1, (payloadLength - 1) * 8);
#endif
        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);

        /* We now need to perform the F9 operation */
        fresh = ((rbId - 1) & 0x1F) << 27;

#ifdef TEST_DECRYPT
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 1,
                            ptrDataBuffer, (payloadLength - 4) * 8, &macI);
#else
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 0,
                            ptrDataBuffer, (payloadLength - 4) * 8, &macI);
#endif
        /* Convert to network order */
        macI = Netfp_htonl (macI);

        /* Verify the MAC-I and set the error flags in the descriptor */
        memcpy ((void *)&rxMacI, (void *)(ptrDataBuffer + (payloadLength - 4)), 4);
        if (rxMacI != macI)
            CSL_FINSR (((Cppi_HostDesc *)ptrPkt)->packetInfo, 23, 20, 5);
        else
            CSL_FINSR (((Cppi_HostDesc *)ptrPkt)->packetInfo, 23, 20, 0);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue. */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA2) && (cipherMode == Netfp_3gppCipherMode_EEA2))
    {
        /**********************************************************************************
         * Ciphering      : AES
         * Authentication : AES
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBCipherSecurityChannel[rbId];

        /* Get the packet length */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&ptr3GPPSecurityChannel->swInfo.txInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for de-ciphering */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);

        /* Loop around till we get the deciphered packet */
        while (1)
        {
            if (Qmss_getQueueEntryCount(F8ToF9QueueHandle) != 0)
                break;
        }

        /* Deciphered packet has been received; get the packet from the queue */
        ptrPkt = (Ti_Pkt *)QMSS_DESC_PTR(Qmss_queuePop(F8ToF9QueueHandle));

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
         * to ensure that the packet contents here are invalidated
         ******************************************************************************/
        Pktlib_getOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* We now need to perform the authentication for the deciphered packet. */
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBAuthSecurityChannel[rbId];

        /* Get the payload length from the descriptor */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfoIndex = 0;
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt,(uint8_t *)&ptr3GPPSecurityChannel->swInfo.txInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for authentication */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], (Cppi_Desc*)ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA1) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Ciphering      : NULL
         * Authentication : SNOW3G
         **********************************************************************************/
        fresh = ((rbId - 1) & 0x1F) << 27;

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

#ifdef TEST_DECRYPT
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 1,
                            ptrDataBuffer, (payloadLength - 4) * 8, &macI);
#else
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, 0,
                            ptrDataBuffer, (payloadLength - 4) * 8, &macI);
#endif
        /* Convert to Network Byte order */
        macI = Netfp_htonl (macI);

        /* Verify the MAC-I and set the error flags in the descriptor */
        memcpy ((void *)&rxMacI, (void *)(ptrDataBuffer + (payloadLength - 4)), 4);
        if (rxMacI != macI)
            CSL_FINSR (((Cppi_HostDesc *)ptrPkt)->packetInfo, 23, 20, 5);
        else
            CSL_FINSR (((Cppi_HostDesc *)ptrPkt)->packetInfo, 23, 20, 0);

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue. */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA2) && (cipherMode == Netfp_3gppCipherMode_EEA0))
    {
        /**********************************************************************************
         * Ciphering      : NULL
         * Authentication : AES
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBAuthSecurityChannel[rbId];

        /* Get the payload length from the descriptor */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfoIndex = 0;
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt,(uint8_t *)&ptr3GPPSecurityChannel->swInfo.txInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA for authentication */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], (Cppi_Desc*)ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA1))
    {
        /**********************************************************************************
         * Ciphering      : SNOW3G
         * Authentication : NULL
         *
         * Skip the PDCP Header and then perform the SNOW3G F8 operation.
         **********************************************************************************/

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

#ifdef TEST_DECRYPT
        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 1,
                            ptrDataBuffer + 1, (payloadLength - 1) * 8);
#else
        ptrNetfpClient->f8 (ptrNetfpUser->cfg.hKeyRrcEnc, countC, rbId-1, 0,
                            ptrDataBuffer + 1, (payloadLength - 1) * 8);
#endif
        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue. */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    else if ((authMode == Netfp_3gppAuthMode_EIA0) && (cipherMode == Netfp_3gppCipherMode_EEA2))
    {
        /**********************************************************************************
         * Ciphering      : AES
         * Authentication : NULL
         **********************************************************************************/
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBCipherSecurityChannel[rbId];

        /* Get the packet length */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)psInfo, psInfoIndex * 4);
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&ptr3GPPSecurityChannel->swInfo.txInfo.swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to SA */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);

        /* Loop around till we get the deciphered packet */
        while (1)
        {
            if (Qmss_getQueueEntryCount(F8ToF9QueueHandle) != 0)
                break;
        }

        /* Deciphered packet has been received; get the packet from the queue */
        ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop (F8ToF9QueueHandle));

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
         * to ensure that the packet contents here are invalidated
         ******************************************************************************/
        Pktlib_getOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Store the packet Id in the descriptor */
        swInfo = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t *)&swInfo);
        Cppi_setTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Send the packet to the result queue. */
        Qmss_queuePushDescSize (resultQueueHandle, ptrPkt, 128);
        return 0;
    }
    /* This is NOT a supported combination. */
    return NETFP_ENOTIMPL;
}

/**
 *  @b Description
 *  @n
 *      The API generates the MAC-I value with given the parameters.
 *
 *      - Snow3G, the packet MUST contain 4 extra bytes to insert the
 *      - AES-CMAC, the flow which is used to receive the packet MUST
 *        contain 4 extra bytes to insert the MAC-I value.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  rbId
 *      Radio Bearer Identifier.
 *  @param[in]  direction
 *      Indicates UL or DL direction
 *  @param[in]  f9QueueHnd
 *      Destination queue to where the packet with the generated MAC-I will be available.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_generateMacI
(
    Netfp_UserHandle    ueHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            countC,
    uint8_t             rbId,
    uint8_t             direction,
    Qmss_QueueHnd       f9QueueHnd
)
{
    Netfp_User*                     ptrNetfpUser;
    Netfp_ClientMCB*                ptrNetfpClient;
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    uint32_t                        payloadLength;
    uint8_t*                        ptrDataBuffer;
    uint32_t                        macI;
    int32_t                         fresh;
    uint32_t                        swInfo0;

    /* Validate the arguments. */
    if (ueHandle == NULL)
        return NETFP_EINVAL;

    /* Get the NETFP user and client */
    ptrNetfpUser   = (Netfp_User*)ueHandle;
    ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;

    /* If the user is NOT active we cannot proceed. */
    if (ptrNetfpUser->status != Netfp_Status_ACTIVE)
        return NETFP_EINVAL;

    /* Perform the F9 operation on the basis of the configuration. */
    if (ptrNetfpUser->cfg.authMode == Netfp_3gppAuthMode_EIA1)
    {
        /* SNOW3G: */
        if ((rbId == 0x0) || (rbId == 0xFF))
            fresh = (rbId & 0x1F) << 27;
        else
            fresh = ((rbId - 1) & 0x1F) << 27;

        /* Get the pointer to the data buffer and the payload length.
         * We don't support chained packets hence read the 1st data buffer length */
        Pktlib_getDataBuffer (ptrPkt, &ptrDataBuffer, &payloadLength);

        /* Perform the F9 operation: */
        ptrNetfpClient->f9 (ptrNetfpUser->cfg.hKeyRrcInt, countC, fresh, direction,
                            ptrDataBuffer, (payloadLength) * 8, &macI);

        /* Convert to Network Byte order */
	    macI = Netfp_htonl (macI);

        /* Append MAC-I value to the end of data */
        memcpy ((void *)(ptrDataBuffer + payloadLength), (void *)&macI, 4);

        /* Set the packet & data buffer length to include MAC-I. */
        Pktlib_setPacketLen(ptrPkt, payloadLength + 4);
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, ptrDataBuffer, payloadLength + 4);

        /* Store the packet Id in the descriptor */
        swInfo0 = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, swInfo0);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to the result queue. */
        Qmss_queuePushDescSize ((Qmss_QueueHnd)f9QueueHnd, ptrPkt, 128);
    }
    else if (ptrNetfpUser->cfg.authMode == Netfp_3gppAuthMode_EIA2)
    {
        Sa_psInfo_t*    ptrPsInfo;
        uint32_t        psInfo[6];
        uint32_t        psInfoIndex = 0;
        uint32_t        iv[4];
        int32_t         ivSize;
        uint32_t        swInfo[3];

        /* AES: */
        ptr3GPPSecurityChannel = &ptrNetfpUser->SRBAuthSecurityChannel[1];

        /* Get the packet length */
        payloadLength = Pktlib_getPacketLen (ptrPkt);

        /* Attach the command in PS data */
        psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
        psInfo[psInfoIndex++] = countC;

        ptrPsInfo = (Sa_psInfo_t *)&psInfo[0];

        iv[0] = countC;
        if ((rbId == 0x0) || (rbId == 0xFF))
            iv[1] = (rbId << 27) | (direction << 26);
        else
            iv[1] = ((rbId - 1) << 27) | (direction << 26);

        iv[2] = iv[3] = 0;

        //AES-CMAC IV size is 64 bit
        ivSize = 8;
        sa_PSINFO_SET_IV (ptrPsInfo, &iv[0], ivSize);

        psInfoIndex = 6;
        Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)psInfo, psInfoIndex * 4);

        swInfo[0] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[0];
        swInfo[1] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[1];
        swInfo[2] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[2];
        sa_SWINFO_UPDATE_DEST_INFO (swInfo, f9QueueHnd, ptrNetfpUser->cfg.srbFlowId);

        Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

        /******************************************************************************
         * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
         * to ensure that the packet contents here are written back.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Bearer Id will be 1 not what is passed as argument.
         * Push the packet to SA for authentication */
        Qmss_queuePushDescSize (ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
    }
    else
    {
        /* NULL: Store the packet Id in the descriptor */
        swInfo0 = ((ptrNetfpUser->cfg.ueId << 16) | (0 << 8) | (rbId << 2));
        Cppi_setSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, swInfo0);

        /******************************************************************************
         * NOTE: Strictly speaking there is no ownership change here from the DSP but
         * the results are going to be posted to MSGCOM channels which does NOT
         * understand the difference.
         ******************************************************************************/
        Pktlib_releaseOwnership(ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

        /* Push the packet to the result queue. */
        Qmss_queuePushDescSize ((Qmss_QueueHnd)f9QueueHnd, ptrPkt, 128);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to encode data for a data radio bearer. This is applicable
 *      for data radio bearers which are created in non-fast path mode. The ciphering
 *      is done using the keys and ciphering mode which was specified when the user
 *      was created
 *
 *  @param[in]  ueHandle
 *      Handle to the user
 *  @param[in]  ptrPkt
 *      Pointer to the packet. Packet descriptor’s data pointer must point to the
 *      first message byte to be ciphered
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  rbId
 *      Data radio bearer.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_encodeDrb
(
    Netfp_UserHandle    ueHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            countC,
    uint8_t             rbId
)
{
    Netfp_User*                 ptrNetfpUser;
    Netfp_3GPPSecurityChannel*  ptr3GPPSecurityChannel;
    uint32_t                    psInfo[2];
    uint32_t                    psInfoIndex = 0;
    uint32_t                    payloadLength;

    /* Get the NETFP user */
    ptrNetfpUser = (Netfp_User *)ueHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpUser == NULL)
        return NETFP_EINVAL;

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
        return NETFP_EINVAL;

    /* Get the packet payload length */
    payloadLength = Pktlib_getPacketLen (ptrPkt);

    /* Attach the command in PS data */
    psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
    psInfo[psInfoIndex++] = countC;

    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)psInfo, psInfoIndex * 4);
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)&ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

    /* Push the packet to SA for ciphering */
    Qmss_queuePushDescSize (ptrNetfpUser->ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to decode data for a data radio bearer. This is called
 *      in the uplink path. Once the packet is decoded it will be placed in the
 *      decoded queue which was specified while creating the LTE channel.
 *
 *  @param[in]  ueHandle
 *      Handle to the user
 *  @param[in]  ptrPkt
 *      Pointer to the packet. Packet descriptor’s data pointer must point to the
 *      first message byte to be deciphered
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  rbId
 *      Data radio bearer.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_decodeDrb
(
    Netfp_UserHandle    ueHandle,
    Ti_Pkt*             ptrPkt,
    uint32_t            countC,
    uint8_t             rbId
)
{
    Netfp_User*                 ptrNetfpUser;
    Netfp_3GPPSecurityChannel*  ptr3GPPSecurityChannel;
    uint32_t                    psInfo[2];
    uint32_t                    psInfoIndex = 0;
    uint32_t                    payloadLength;

    /* Get the NETFP user */
    ptrNetfpUser = (Netfp_User *)ueHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpUser == NULL)
        return NETFP_EINVAL;

    /* Get the security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
        return NETFP_EINVAL;

    /* Get the packet payload length */
    payloadLength = Pktlib_getPacketLen (ptrPkt);

    /* Attach the command in PS data */
    psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
    psInfo[psInfoIndex++] = countC;

    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)psInfo, psInfoIndex * 4);
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt,(uint8_t*)&ptr3GPPSecurityChannel->swInfo.txInfo.swInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

    /* Push the packet to SA for de-ciphering */
    Qmss_queuePushDescSize (ptrNetfpUser->ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to obtain the countC value used during the decode operation.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *  @param[out] countC
 *      countC value read from the packet.
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void Netfp_getCountC (Ti_Pkt* ptrPkt, uint32_t* countC)
{
    Cppi_getTimeStamp (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, countC);
    return;
}

/**
 *  @b Description
 *  @n
 *      The API prepares and sends the packet to SA for ciphering based on the input parameters.
 *
 *  @param[in]  ueHandle
 *      User Handle
 *  @param[in]  opType
 *      Type of operation to be performed (Ciphering/Deciphering)
 *  @param[in]  rbId
 *      Data radio bearer
 *  @param[in]  direction
 *      Indicates UL or DL direction
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *  @param[in]  countC
 *      Value of Count-C
 *  @param[in]  f8QueueHnd
 *      Destination queue to where the packet post ciphering will be available.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0 [See Error code]
 */
int32_t Netfp_cipher
(
    Netfp_UserHandle        ueHandle,
    uint8_t                 rbId,
    Netfp_3gppOperation     opType,
    uint8_t                 direction,
    Ti_Pkt*                 ptrPkt,
    uint32_t                countC,
    Qmss_QueueHnd           f8QueueHnd
)
{
    Netfp_3GPPSecurityChannel*  ptr3GPPSecurityChannel;
    Netfp_User*                 ptrNetfpUser;
    Sa_psInfo_t*                ptrPsInfo;
    uint32_t                    psInfo[6];
    uint32_t                    psInfoIndex = 0;
    uint32_t                    iv[4];
    int32_t                     ivSize;
    uint32_t                    swInfo[3];
    uint32_t                    payloadLength;
    uint8_t                     flowId;

    /* Sanity Check: Validate the arguments. */
    ptrNetfpUser = (Netfp_User*)ueHandle;
    if (ptrNetfpUser == NULL)
        return NETFP_EINVAL;

    /* Get the security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
        return NETFP_EINVAL;

    /* Get the packet payload length */
    payloadLength = Pktlib_getPacketLen (ptrPkt);

    /* Attach the command in PS data */
    psInfo[psInfoIndex++] = PASAHO_SINFO_FORMAT_CMD(0, payloadLength);
    psInfo[psInfoIndex++] = countC;

    ptrPsInfo = (Sa_psInfo_t *)&psInfo[0];

    if (ptrNetfpUser->cfg.drbCipherMode == Netfp_3gppCipherMode_EEA2)
    {
        iv[0] = countC;
        iv[1] = ((rbId - 1) << 27) | (direction << 26);
        iv[2] = iv[3] = 0;
    }
    else if (ptrNetfpUser->cfg.drbCipherMode == Netfp_3gppCipherMode_EEA1)
    {
        iv[0] = iv[2] = ((rbId - 1) << 27) | (direction << 26);
        iv[1] = iv[3] = countC;
    }
    else
    {
        /* No Encryption: What are we trying to cipher? */
        return NETFP_EINVAL;
    }

    ivSize = 16;
    sa_PSINFO_SET_IV (ptrPsInfo, &iv[0], ivSize);

    psInfoIndex = 6;
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t*)psInfo, psInfoIndex * 4);

    /* Populate the software information: */
    if (opType == Netfp_3gppOperation_Cipher)
    {
        /* Ciphering operation: */
        swInfo[0] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[0];
        swInfo[1] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[1];
        swInfo[2] = ptr3GPPSecurityChannel->swInfo.rxInfo.swInfo[2];

        /* For ciphering operation we use the flow identifier from the binding configuration */
        flowId = ptr3GPPSecurityChannel->bindCfg.flowId;
    }
    else
    {
        /* Deciphering operation: */
        swInfo[0] = ptr3GPPSecurityChannel->swInfo.txInfo.swInfo[0];
        swInfo[1] = ptr3GPPSecurityChannel->swInfo.txInfo.swInfo[1];
        swInfo[2] = ptr3GPPSecurityChannel->swInfo.txInfo.swInfo[2];

        /* For deciphering we use the flow identifier from the connect configuration */
        flowId = ptr3GPPSecurityChannel->connectCfg.flowId;
    }
    sa_SWINFO_UPDATE_DEST_INFO (swInfo, f8QueueHnd, flowId);

    /* Set the new software information: */
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt, (uint8_t  *)&swInfo);

    /******************************************************************************
     * NOTE: There is an ownership change here from the DSP to the CPDMA. We need
     * to ensure that the packet contents here are written back.
     ******************************************************************************/
    Pktlib_releaseOwnership(ptrNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);

    /* Push the packet to SA for ciphering */
    Qmss_queuePushDescSize (ptrNetfpUser->ptrNetfpClient->netcpTxQueue[NSS_SA_QUEUE_SASS2_INDEX], ptrPkt, 128);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The API returns the configured SRB ciphering mode for the specific user
 *
 *  @param[in]  ueHandle
 *      Handle to the user
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Configured ciphering mode
 */
Netfp_3gppCipherMode Netfp_getSrbCipherMode(Netfp_UserHandle ueHandle)
{
    return ((Netfp_User*)ueHandle)->cfg.srbCipherMode;
}

/**
 *  @b Description
 *  @n
 *      The API returns the configured SRB authentication mode for the specific user
 *
 *  @param[in]  ueHandle
 *      Handle to the user
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Configured authentication mode
 */
Netfp_3gppAuthMode Netfp_getSrbAuthMode(Netfp_UserHandle ueHandle)
{
    return ((Netfp_User*)ueHandle)->cfg.authMode;
}

/**
 *  @b Description
 *  @n
 *      This function is used to get the user identifier, radio bearer id and QCI
 *      from the packet.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *  @param[out]  ueId
 *      User identifier
 *  @param[out]  qci
 *      Quality of service class identifier
 *  @param[out]  rbId
 *      Radio Bearer Identifier
 *  @param[out]  contextId
 *      Security context Id indicating a change in state from the previous configuration. Its a toggle
 *      bit that indicates a channel re-establishment or handover.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void Netfp_getPacketId
(
    Ti_Pkt*     ptrPkt,
    uint16_t*   ueId,
    uint8_t*    qci,
    uint8_t*    rbId,
    uint8_t*    contextId
)
{
    uint32_t    swInfo0;

    /* Get the software information: */
    swInfo0 = Cppi_getSoftwareInfo0 (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt);

    /* Get the information from the software information. */
    *ueId       = ((swInfo0 & 0xFFFF0000) >> 16);
    *qci        = ((swInfo0 & 0x0000FF00) >> 8);
    *rbId       = ((swInfo0 & 0x000000FC) >> 2);
    *contextId  = (swInfo0 & 0x00000001);
    return;
}

/**
 *  @b Description
 *  @n
 *      This API is used to get the status of the packet once it has passed
 *      through the NETCP security subystem.
 *
 *  @param[in]  ptrPkt
 *      Pointer to the packet.
 *
 * \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      0       -   No error
 *  @retval
 *      1       -   Packet rejected by SA because it is out of replay window range.
 *  @retval
 *      2       -   Packet rejected by SA because it is duplicated.
 *  @retval
 *      3       -   Packet rejected by SA because the key in the security context is out of date.
 *  @retval
 *      4       -   Packet rejected by SA because the MKI in the security context does not match the one in the packet.
 *  @retval
 *      5       -   Packet rejected by SA because authentication verification failed.
 */
uint32_t Netfp_getPacketError(Ti_Pkt* ptrPkt)
{
    return Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)ptrPkt);
}

/**
 *  @b Description
 *  @n
 *      The function is used to suspend the LTE channel during re-establishment. LTE
 *      channels can operate in either of the following modes:-
 *
 *      - Fast Path
 *          The mode ensures that the GTPU packets received by the application are already
 *          air ciphered. Suspending a channel in this mode will ensure that GTPU Packets
 *          are placed into a temporary internal queue and are not ciphered till the new
 *          user and keys are negotiated.
 *
 *      - Slow Path
 *          The mode ensures that the GTPU packets are received by the application and
 *          the application is responsible for performing the air ciphering operation.
 *          Suspending a channel in this mode has no effect and GTPU packets will continue
 *          to be placed in the application specified ROHC queue
 *
 *  @param[in]  ueHandle
 *      NETFP user to which the channel is being suspended belongs.
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[in]  flowId
 *      Flow identifier used to temporarily receive radio bearers data during re-establishment.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  @sa Netfp_getSuspendedPacket
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_suspendLTEChannel
(
    Netfp_UserHandle    ueHandle,
    uint8_t             rbId,
    uint32_t            flowId,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    Netfp_User*                     ptrNetfpUser;
    Netfp_SockAddr                  sockAddr;

    /* Sanity Check: Validate the arguments */
    ptrNetfpUser = (Netfp_User *)ueHandle;
    if (ptrNetfpUser == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Check if fast path is enabled or not. Slow path bearers can return right away. */
    if (ptr3GPPSecurityChannel->bindCfg.enableFastPath == 0)
        return 0;

    /* Check if channel is bound to a socket. */
    if (ptr3GPPSecurityChannel->ptrNetfpSocket == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Populate the socket address: */
    sockAddr.sin_port                = 2152;
    sockAddr.sin_gtpuId              = ptr3GPPSecurityChannel->bindCfg.sin_gtpuId;
    sockAddr.op.bind.flowId          = flowId;
    sockAddr.op.bind.appInfo         = 0;
    sockAddr.op.bind.queueHandle     = ptr3GPPSecurityChannel->reEstDataBufferQueue;
    sockAddr.op.bind.inboundFPHandle = ptr3GPPSecurityChannel->bindCfg.fpHandle;

    /* Suspend the socket */
    if (Netfp_suspendSocket ((Netfp_SockHandle)ptr3GPPSecurityChannel->ptrNetfpSocket, &sockAddr, errCode) < 0)
        return -1;

    /* Update the channel status */
    ptr3GPPSecurityChannel->state = Netfp_ChannelState_SUSPEND;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to reconfigure the LTE channel during re-establishment
 *      The API creates a new LTE channel. It uses existing LTE channel's bind and
 *      connect configuration parameters to configure the channel.
 *
 *  @param[in]  newUeHandle
 *      New NETFP user to which the channel is being moved
 *  @param[in]  oldUeHandle
 *      Old NETFP user from which the existing channel is being moved
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[in]  countC
 *      CountC value to be used with the new channel. This will be zero or the last used CountC value + 1
 *      based on whether the countC has be reset or resumed.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  @verbatim
    Note: All other channel configuration parameters are reused from when the channel was added to the old UE.
    @endverbatim
 *
 *  @sa Netfp_getSuspendedPacket
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   Socket handle associated with the 3GPP channel
 *  @retval
 *      Error   -   NULL
 */
Netfp_SockHandle Netfp_reconfigureLTEChannel
(
    Netfp_UserHandle    newUeHandle,
    Netfp_UserHandle    oldUeHandle,
    uint8_t             rbId,
    uint32_t            countC,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptrOld3GPPSecurityChannel;
    Netfp_3GPPSecurityChannel*      ptrNew3GPPSecurityChannel;
    Netfp_Socket*                   ptrOldNetfpSocket;
    Netfp_Socket*                   ptrNewNetfpSocket;
    Netfp_User*                     ptrOldNetfpUser;
    Netfp_User*                     ptrNewNetfpUser;
    Netfp_SecChannelCfg             secChannelCfg;
    Netfp_OptionTLV                 optCfg;
    Netfp_SockAddr                  sockAddr;
    Netfp_LTEChannelBindCfg*        ptrLTEChannelBindCfg;
    Netfp_LTEChannelConnectCfg*     ptrLTEChannelConnectCfg;
    int32_t                         stubErrCode;
    uint8_t                         isAllocated;
    void*                           context;

    /* Sanity Check: Validate the arguments */
    if ((oldUeHandle == NULL) || (newUeHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Get the old & new NETFP User. */
    ptrOldNetfpUser = (Netfp_User*)oldUeHandle;
    ptrNewNetfpUser = (Netfp_User*)newUeHandle;

    /* Get the old 3GPP security channel. */
    ptrOld3GPPSecurityChannel = Netfp_findLTEChannelById(ptrOldNetfpUser, rbId);
    if (ptrOld3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return NULL;
    }

    /* Sanity Check: Get the old socket information. */
    ptrOldNetfpSocket = ptrOld3GPPSecurityChannel->ptrNetfpSocket;
    if (ptrOldNetfpSocket == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Check if the PA LUT2 entry is valid. */
    if (ptrOldNetfpSocket->bindInfo.lutEntryValid == 0)
    {
        *errCode = NETFP_EINTERNAL;
        return NULL;
    }

    /* Allocate memory for the client security channel: */
    ptrNew3GPPSecurityChannel = ptrNewNetfpUser->ptrNetfpClient->cfg.malloc (sizeof(Netfp_3GPPSecurityChannel), 0);
    if (ptrNew3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrNew3GPPSecurityChannel, 0, sizeof(Netfp_3GPPSecurityChannel));

    /* Get the existing channel's bind & connect configuration */
    ptrLTEChannelBindCfg    = &ptrOld3GPPSecurityChannel->bindCfg;
    ptrLTEChannelConnectCfg = &ptrOld3GPPSecurityChannel->connectCfg;

    /* Copy over the configuration: */
    memcpy ((void *)&ptrNew3GPPSecurityChannel->bindCfg,    (void*)ptrLTEChannelBindCfg,    sizeof(Netfp_LTEChannelBindCfg));
    memcpy ((void *)&ptrNew3GPPSecurityChannel->connectCfg, (void*)ptrLTEChannelConnectCfg, sizeof(Netfp_LTEChannelConnectCfg));

    /* Remember the user which has created the 3GPP channel */
    ptrNew3GPPSecurityChannel->ptrNetfpUser     = ptrNewNetfpUser;
    ptrNew3GPPSecurityChannel->rbId		        = rbId;
    ptrNew3GPPSecurityChannel->ptrNetfpClient   = ptrNewNetfpUser->ptrNetfpClient;

    /* Toggle old channel mark for the re-established channel */
    if (ptrOld3GPPSecurityChannel->currentMark == 1)
        ptrNew3GPPSecurityChannel->currentMark = 0;
    else
        ptrNew3GPPSecurityChannel->currentMark = 1;

    /* Populate the security channel configuration */
    memset ((void *)&secChannelCfg, 0, sizeof(Netfp_SecChannelCfg));

    /* Populate the security channel configuration for ciphering */
    secChannelCfg.type                       = Netfp_SecurityChannelType_AIR_F8;
    secChannelCfg.u.f8Cfg.ueId               = ptrNewNetfpUser->cfg.ueId;
    secChannelCfg.u.f8Cfg.qci                = ptrLTEChannelConnectCfg->qci;
    secChannelCfg.u.f8Cfg.isDataRadioBearer  = 1;
    secChannelCfg.u.f8Cfg.cipherMode         = ptrNewNetfpUser->cfg.drbCipherMode;
    secChannelCfg.u.f8Cfg.encodeFlowId       = ptrLTEChannelBindCfg->flowId;
    secChannelCfg.u.f8Cfg.decodeFlowId       = ptrLTEChannelConnectCfg->flowId;
    secChannelCfg.u.f8Cfg.countC             = countC;
    secChannelCfg.u.f8Cfg.rbId               = rbId;
    secChannelCfg.u.f8Cfg.encodeQueue        = ptrLTEChannelBindCfg->chDrbEnc;
    secChannelCfg.u.f8Cfg.decodeQueue        = ptrLTEChannelConnectCfg->chDrbDec;
    secChannelCfg.u.f8Cfg.currentMark        = ptrNew3GPPSecurityChannel->currentMark;
    memcpy ((void *)&secChannelCfg.u.f8Cfg.hKeyUpEnc, (void *)ptrNewNetfpUser->cfg.hKeyUpEnc, 16);

    /* Create the security channel for the radio bearer */
    ptrNew3GPPSecurityChannel->srvSecurityChannelHandle = Netfp_createSecurityChannel (ptrNewNetfpUser->ptrNetfpClient, &secChannelCfg,
                                                                                       &ptrNew3GPPSecurityChannel->swInfo, errCode);
    if (ptrNew3GPPSecurityChannel->srvSecurityChannelHandle == NULL)
    {
        /* Error: Unable to create the security channel; error code is already populated */
        ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Create and register the socket which is used to send & receive data from the core network */
    ptrNewNetfpSocket = Netfp_socket (ptrNewNetfpUser->ptrNetfpClient, ptrOldNetfpSocket->localSockAddr.sin_family, errCode);
    if (ptrNewNetfpSocket == NULL)
    {
        /* Error: Unable to create the socket. */
        ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Convert the LTE binding configuration to the socket address structure.
     * - LTE channels always use GTPU port
     * - The queue handle is configured to be the ROHC queue because in the
     *   case of non fast path this where we expect the packets to land up.
     * - We don't want to program the LUT2 entry so pass flowId as 0xFFFFFFFF */
    sockAddr.sin_family              = ptrOldNetfpSocket->localSockAddr.sin_family;
    sockAddr.op.bind.flowId          = 0xFFFFFFFF;
    sockAddr.op.bind.appInfo         = 0;
    sockAddr.op.bind.notifyFunction  = ptrLTEChannelBindCfg->notifyFunction;
    sockAddr.op.bind.queueHandle     = ptrLTEChannelBindCfg->chDrbRohc;
    sockAddr.op.bind.inboundFPHandle = ptrLTEChannelBindCfg->fpHandle;
    sockAddr.sin_port                = 2152;
    sockAddr.sin_gtpuId              = ptrLTEChannelBindCfg->sin_gtpuId;

    /* We are not programming LUT2 entry. Only updating the socket bind information. */
    if (Netfp_bind(ptrNewNetfpSocket, &sockAddr, errCode) < 0)
    {
        /* Error: Unable to bind the socket. Cleanup after ourselves. */
        Netfp_closeSocket(ptrNewNetfpSocket, &stubErrCode);
        ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Populate the L4 handle from the old socket information */
    memcpy (ptrNewNetfpSocket->bindInfo.l4Handle, ptrOldNetfpSocket->bindInfo.l4Handle, sizeof (paHandleL4_t));

    /* Set the flag to indicate that the LUT2 handle is valid */
    ptrNewNetfpSocket->bindInfo.lutEntryValid = 1;

    /* Reset the L4 handle in the old socket. It will not be used any more by the old socket.
     * We have stored the L4 handle in the new socket. */
    ptrOldNetfpSocket->bindInfo.lutEntryValid = 0;

    /* Convert the LTE connect configuration to the socket address structure.
     * - LTE channels always use GTPU port */
    sockAddr.sin_family                  = ptrOldNetfpSocket->peerSockAddr.sin_family;
    sockAddr.sin_port                    = 2152;
    sockAddr.sin_gtpuId                  = ptrLTEChannelConnectCfg->sin_gtpuId;
    sockAddr.op.connect.outboundFPHandle = ptrLTEChannelConnectCfg->fpHandle;

    /* Connect the socket to the remote peer. */
    if (Netfp_connect(ptrNewNetfpSocket, &sockAddr, errCode) < 0)
    {
        /* Error: Unable to connect the socket. Cleanup after ourselves */
        Netfp_closeSocket(ptrNewNetfpSocket, &stubErrCode);
        ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Setup the socket priority: This is the DSCP to be placed into the IP header of the
     * packet sent via the specified socket. */
    optCfg.type   = Netfp_Option_PRIORITY;
    optCfg.length = sizeof (uint8_t);
    optCfg.value  = (void *)&ptrLTEChannelConnectCfg->dscp;
    if (Netfp_setSockOpt (ptrNewNetfpSocket, &optCfg, errCode) < 0)
    {
        /* Error: Unable to connect the socket. Cleanup after ourselves */
        Netfp_closeSocket(ptrNewNetfpSocket, &stubErrCode);
        ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
        return NULL;
    }

    /* Open a queue to buffer packets. This will be used to buffer packets during re-establishment. */
    if (ptrLTEChannelBindCfg->enableFastPath == 1)
    {
        ptrNew3GPPSecurityChannel->reEstDataBufferQueue = Qmss_queueOpenInGroup(0, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                                                                QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (ptrNew3GPPSecurityChannel->reEstDataBufferQueue < 0)
        {
            /* Error: Unable to open buffering queue. Cleanup after ourselves */
            *errCode = NETFP_EINTERNAL;
            Netfp_closeSocket(ptrNewNetfpSocket, &stubErrCode);
            ptrNewNetfpUser->ptrNetfpClient->cfg.free (ptrNew3GPPSecurityChannel, sizeof(Netfp_3GPPSecurityChannel));
            return NULL;
        }
    }

    /* Create the links between the security channels and socket. */
    ptrNewNetfpSocket->ptr3GPPSecurityChannel = ptrNew3GPPSecurityChannel;
    ptrNew3GPPSecurityChannel->ptrNetfpSocket = ptrNewNetfpSocket;

    /* Single Core Critical Section: Add the DRB channel */
    context = ptrNewNetfpUser->ptrNetfpClient->cfg.enterCS();
    Netfp_listAdd ((Netfp_ListNode**)&ptrNewNetfpUser->ptrDRBSecurityChannelList, (Netfp_ListNode*)ptrNew3GPPSecurityChannel);
    ptrNewNetfpUser->ptrNetfpClient->cfg.exitCS(context);

    /* Update the channel status */
    ptrNew3GPPSecurityChannel->state = Netfp_ChannelState_RECONFIG;

    /* Return the socket handle */
    return (Netfp_SockHandle)ptrNew3GPPSecurityChannel->ptrNetfpSocket;
}

/**
 *  @b Description
 *  @n
 *      The function is used to resume the LTE channel after reestablishment. LTE
 *      channels can operate in either of the following modes:-
 *
 *      - Fast Path
 *          The mode ensures that the GTPU packets received by the application are
 *          already air ciphered. When the channel had been suspended all the GTPU
 *          packets received were placed in an internal queue. Applications can use
 *          the NETFP API: 'Netfp_getSuspendedPacket' to get the packets from this
 *          internal queue. Once the LTE channel is resumed the GTPU packets are then
 *          sent to the NETCP for Air ciphering and any packets left behind in the
 *          internal queue are dropped and the Extended socket stats track this.
 *
 *      - Slow Path
 *          The mode ensures that the GTPU packets are received by the application and
 *          the application is responsible for performing the air ciphering operation.
 *          Resuming a channel in this mode has no effect and GTPU packets will continue
 *          to be placed in the application specified ROHC queue.
 *
 *  @param[in]  newUeHandle
 *      New NETFP user to which the channel is being moved
 *  @param[in]  oldUeHandle
 *      Old NETFP user from which the existing channel is being moved
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 * @b NOTE: Once the application performs the resume LTE channel any pending packets which
 * were in the internal buffering queue are dropped and cleaned up. The socket extended statistics
 * track any dropped packets
 *
 * @sa Netfp_getSuspendedPacket
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_resumeLTEChannel
(
    Netfp_UserHandle    newUeHandle,
    Netfp_UserHandle    oldUeHandle,
    uint8_t             rbId,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptrOld3GPPSecurityChannel;
    Netfp_3GPPSecurityChannel*      ptrNew3GPPSecurityChannel;
    Netfp_User*                     ptrOldNetfpUser;
    Netfp_User*                     ptrNewNetfpUser;
    Netfp_SockAddr                  sockAddr;
    Ti_Pkt*                         ptrPkt;

    /* Sanity Check: Validate the arguments */
    if ((oldUeHandle == NULL) || (newUeHandle == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the old and new NETFP User. */
    ptrNewNetfpUser = (Netfp_User*)newUeHandle;
    ptrOldNetfpUser = (Netfp_User*)oldUeHandle;

    /* Get the new 3GPP security channel. */
    ptrNew3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNewNetfpUser, rbId);
    if (ptrNew3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Check if fast path is enabled or not. Slow path bearers can return right away. */
    if (ptrNew3GPPSecurityChannel->bindCfg.enableFastPath == 0)
        return 0;

    /* Get the old 3GPP security channel. */
    ptrOld3GPPSecurityChannel = Netfp_findLTEChannelById(ptrOldNetfpUser, rbId);
    if (ptrOld3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Populate the socket address:
     *  - Ensure that the countC is not programmed */
    sockAddr.sin_gtpuId              = ptrNew3GPPSecurityChannel->bindCfg.sin_gtpuId;
    sockAddr.op.bind.inboundFPHandle = ptrNew3GPPSecurityChannel->bindCfg.fpHandle;
    sockAddr.op.bind.flowId          = ptrNew3GPPSecurityChannel->bindCfg.flowId;
    sockAddr.op.bind.queueHandle     = ptrNew3GPPSecurityChannel->reEstDataBufferQueue;
    sockAddr.sin_port                = 0;
    sockAddr.op.bind.appInfo         = 0;

    /* Resume the socket */
    if (Netfp_resumeSecureSocket ((Netfp_SockHandle)ptrNew3GPPSecurityChannel->ptrNetfpSocket,
                                   &sockAddr, ptrNew3GPPSecurityChannel->srvSecurityChannelHandle, errCode) < 0)
    {
        /* Error: Unable to resume the secure socket */
        return -1;
    }

    /* Store the number of dropped packets from the old context so the count is cumulative */
    ptrNew3GPPSecurityChannel->ptrNetfpSocket->extendedStats.reEstDroppedPackets =
            ptrOld3GPPSecurityChannel->ptrNetfpSocket->extendedStats.reEstDroppedPackets;

    /* Check if there are any packets buffered in the re-establishment queue. We will drop these packets */
    while (1)
    {
        ptrPkt = (Ti_Pkt *) QMSS_DESC_PTR (Qmss_queuePop(ptrOld3GPPSecurityChannel->reEstDataBufferQueue));
        if (ptrPkt == NULL)
            break;

        /* Increment the statistics: */
        ptrNew3GPPSecurityChannel->ptrNetfpSocket->extendedStats.reEstDroppedPackets++;

        /******************************************************************************
         * NOTE: There is an ownership change here from the CPDMA to the DSP. We need
         * to ensure that the packet contents here are invalidated
        ******************************************************************************/
        Pktlib_getOwnership(ptrNewNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
        Pktlib_freePacket (ptrNewNetfpUser->ptrNetfpClient->cfg.pktlibInstHandle, ptrPkt);
    }

    /* Update the channel status */
    ptrNew3GPPSecurityChannel->state = Netfp_ChannelState_RESUME;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called to handle all the packets which had been received on the
 *      S1 Link after the LTE channel had been *SUSPENDED*. Applications can use this API
 *      to retrieve the packets and perform the encode DRB operation on these packets.
 *
 *  @param[in]  ueHandle
 *      This is the *old* UE handle which had been suspended
 *  @param[in]  rbId
 *      This is the DRB identifier which is being reestablished
 *  @param[out]  ptrPkt
 *      Pointer to the any GTPU packet received after the LTE Channel was suspended.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @sa Netfp_suspendLTEChannel
 *  @sa Netfp_reconfigureLTEChannel
 *  @sa Netfp_resumeLTEChannel
 *
 *  @b NOTE: Example calling sequence in the application domain where the countC is being tracked
 *           in the application.
 *
 * @verbatim

    // Starting the reestablishment procedure: Suspend the LTE channel
    Netfp_suspendLTEChannel(...)
    ...
    // Get the countC from the LTE channel
    Netfp_getLTEChannelOpt (..., &appCountC, ...)
    ...
    // Reconfigure the new user
    Netfp_createUser(...)

    // Reconfigure the LTE Channel start using the countC from the suspended LTE channel
    Netfp_reconfigureLTEChannel (..., appCountC, )
    ...
    // Loop around and process the suspended packets
    while (...)
    {
        // Get the suspended packet from the OLD USER
        Netfp_getSuspendedPacket (...)
        ...
        // Encode the DRB using the NEW USER
        Netfp_encodeDRB (.., appCountC++, ...)
    }
    ...
    // Resume the LTE Channel
    Netfp_resumeLTEChannel (...)

   @endverbatim
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getSuspendedPacket
(
    Netfp_UserHandle    ueHandle,
    uint8_t             rbId,
    Ti_Pkt**            ptrPkt,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    Netfp_User*                     ptrNetfpUser;

    /* Sanity Check: Validate the arguments */
    if (ueHandle == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the NETFP User. */
    ptrNetfpUser = (Netfp_User*)ueHandle;

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Packets are buffered into the reestablishment queue only for Fast Path channels */
    if (ptr3GPPSecurityChannel->bindCfg.enableFastPath == 0)
    {
        /* Slow Path Channel: The packets are anyway being placed into the ROHC queue */
        *ptrPkt = NULL;
        return 0;
    }

    /* Sanity Check: Ensure that the Fast path DRB has been suspended. */
    if (ptr3GPPSecurityChannel->state != Netfp_ChannelState_SUSPEND)
    {
        /* Error: Invalid usage of the API */
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Pop off the packet from the reestablished queue: */
    *ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptr3GPPSecurityChannel->reEstDataBufferQueue));
    if (*ptrPkt != NULL)
        Pktlib_getOwnership (ptr3GPPSecurityChannel->ptrNetfpClient->cfg.pktlibInstHandle, *ptrPkt);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get Channel information.
 *      Currently only CountC option is supported.
 *
 *  @param[in]  ueHandle
 *      NETFP user to which the channel belongs.
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[out] ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
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
int32_t Netfp_getLTEChannelOpt
(
    Netfp_UserHandle        ueHandle,
    uint8_t                 rbId,
    Netfp_OptionTLV*        ptrOptInfo,
    int32_t*                errCode
)
{
    Netfp_User*                 ptrNetfpUser;
    Netfp_3GPPSecurityChannel*  ptr3GPPSecurityChannel;
    Netfp_ClientMCB*            ptrNetfpClient;

    /* Get the NETFP user */
    ptrNetfpUser = (Netfp_User *)ueHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrNetfpUser == NULL) || (ptrOptInfo == NULL))
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Sanity check: Validate the security channel. */
    if (ptr3GPPSecurityChannel->srvSecurityChannelHandle == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Process the supported options */
    switch (ptrOptInfo->type)
    {
        case Netfp_Option_COUNTC:
        {
            /* Get the NETFP Client MCB: Services are available only if the client is active. */
            ptrNetfpClient = ptrNetfpUser->ptrNetfpClient;
            if (ptrNetfpClient == NULL)
            {
                *errCode = NETFP_EINTERNAL;
                return -1;
            }

            /* Get the CountC value */
            if (Netfp_getSecurityChannelOpt (ptrNetfpClient, ptr3GPPSecurityChannel->srvSecurityChannelHandle, ptrOptInfo, errCode) <0)
                return -1;

            break;
        }
        default:
        {
            /* Option is NOT supported */
            *errCode = NETFP_EINVAL;
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initiate a HandOver for a LTE channel at source eNB. Packets
 *      received during the Source HO procedure will be buffered in a appliction provided
 *      queue using the application provided flow Id. Its the application's responsibilty to
 *      process these packets and send them over to target eNB using GTPU sockets.
 *
 *  @param[in]  ueHandle
 *      NETFP user to which the channel is being handed over belongs.
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[in]  flowId
 *      Flow identifier used to temporarily receive radio bearers data during HO at source eNB.
 *  @param[in]  sourceHOQueue
 *      Data buffer queue used to temporarily receive radio bearers data during HO at source eNB.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_initiateSourceHandOver
(
    Netfp_UserHandle    ueHandle,
    uint8_t             rbId,
    uint32_t            flowId,
    Qmss_QueueHnd       sourceHOQueue,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    Netfp_User*                     ptrNetfpUser;
    Netfp_SockAddr                  sockAddr;

    /* Sanity Check: We should have a valid UE handle */
    ptrNetfpUser = (Netfp_User *)ueHandle;
    if (ptrNetfpUser == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Check if channel is bound to a  socket. */
    if (ptr3GPPSecurityChannel->ptrNetfpSocket == NULL)
    {
        *errCode = NETFP_EINTERNAL;
        return -1;
    }

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Populate the socket address: */
    sockAddr.sin_port                = 2152;
    sockAddr.sin_gtpuId              = ptr3GPPSecurityChannel->bindCfg.sin_gtpuId;
    sockAddr.op.bind.flowId          = flowId;
    sockAddr.op.bind.appInfo         = ((ptrNetfpUser->cfg.ueId << 16) | (ptr3GPPSecurityChannel->connectCfg.qci << 8) | (rbId << 2));
    sockAddr.op.bind.queueHandle     = sourceHOQueue;
    sockAddr.op.bind.inboundFPHandle = ptr3GPPSecurityChannel->bindCfg.fpHandle;

    /* Suspend the socket */
    if (Netfp_suspendSocket ((Netfp_SockHandle)ptr3GPPSecurityChannel->ptrNetfpSocket, &sockAddr, errCode) < 0)
        return -1;

    /* Update the channel status */
    ptr3GPPSecurityChannel->state = Netfp_ChannelState_HO_INITIATED;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initiate the Target HO procedure and it will ensure that the
 *      LTE channel moves from the HO to the Normal processing state. This API is to be invoked
 *      only if the LTE channel was created with the flag 'isHOInProgress' set to 1. All packets
 *      matching the GTPU Identifier are placed into an internal HO queue.
 *
 *      Applications are responsible for processing all packets from the source eNB. Once all
 *      those packets have been processed the application will then need to invoke this API.
 *      The functions starts the HO Procedure but the application will need to invoke the NETFP
 *      API 'Netfp_getTargetHandOverPackets' to handle any packets that arrived during the
 *      procedure. These packets MUST be processed before proceeding with normal operations.
 *      Failure to do so will result in an out of order processing of packets.
 *
 *  @sa Netfp_getTargetHandOverPackets
 *
 *  @param[in]  ueHandle
 *      NETFP user to which the channel is being handed over belongs.
 *  @param[in]  rbId
 *      Data radio bearer identifier
 *  @param[in]  countC
 *      countC to be used.
 *  @param[out]  targetHandOverId
 *      Target Handover Identifier given back to the application.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_completeTargetHandOver
(
    Netfp_UserHandle    ueHandle,
    uint8_t             rbId,
    uint32_t            countC,
    uint32_t*           targetHandOverId,
    int32_t*            errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;
    Netfp_User*                     ptrNetfpUser;
    Netfp_SockAddr                  sockAddr;

    /* Get the NETFP user */
    ptrNetfpUser = (Netfp_User *)ueHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrNetfpUser == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = Netfp_findLTEChannelById(ptrNetfpUser, rbId);
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Update the channel status to HandOver in progress */
    ptr3GPPSecurityChannel->state = Netfp_ChannelState_HO_INPROGRESS;

    /* Populate the target handover identifier: */
    *targetHandOverId = (uint32_t)ptr3GPPSecurityChannel;

    /* Initialize the socket address information */
    memset ((void *)&sockAddr, 0, sizeof(Netfp_SockAddr));

    /* Populate the socket address: */
    sockAddr.sin_port                = 2152;
    sockAddr.sin_gtpuId              = ptr3GPPSecurityChannel->bindCfg.sin_gtpuId;
    sockAddr.op.bind.inboundFPHandle = ptr3GPPSecurityChannel->bindCfg.fpHandle;
    sockAddr.op.bind.flowId          = ptr3GPPSecurityChannel->bindCfg.flowId;
    sockAddr.op.bind.appInfo         = ((ptrNetfpUser->cfg.ueId << 16) | (ptr3GPPSecurityChannel->connectCfg.qci << 8) | (rbId << 2));

    /* Check if fast path is enabled or not */
    if (ptr3GPPSecurityChannel->bindCfg.enableFastPath == 0)
    {
        /* Fast Path is disabled: LUT2 is reprogrammed to move all the packets matching the GTPU Identifier
         * from the HO Queue to the ROHC Queue. Applications need to get all the HO Queue and process them
         * manually. */
        sockAddr.op.bind.queueHandle = ptr3GPPSecurityChannel->bindCfg.chDrbRohc;
        return Netfp_resumeSocket ((Netfp_SockHandle)ptr3GPPSecurityChannel->ptrNetfpSocket, &sockAddr, errCode);
    }

    /* Fast Path is enabled: We need to ensure that packets from the HO Queue are first diverted
     * before we reconfigure the LUT2 to pass the packets to the NETCP Air Ciphering i.e. we resume
     * the Secure socket. */
    sockAddr.op.bind.queueHandle = ptr3GPPSecurityChannel->hoDataBufferQueue;
    sockAddr.op.bind.appInfo     = countC;      /* New countC */
    sockAddr.sin_port            = 1;           /* Program the countC */
    return Netfp_resumeSecureSocket ((Netfp_SockHandle)ptr3GPPSecurityChannel->ptrNetfpSocket,
                                     &sockAddr, ptr3GPPSecurityChannel->srvSecurityChannelHandle, errCode);
}

/**
 *  @b Description
 *  @n
 *      On target eNB, the function is used to determine if the target HandOver is complete.
 *      Application MUST call this function after calling Netfp_completeTargetHandOver() API
 *      to determine if HandOver is complete.
 *
 *  @param[in]  targetHandOverId
 *      Identifier given back to the application from the Netfp_completeTargetHandOver() API.
 *  @param[out]  ptrPkt
 *      Pointer to the packet received before Netfp_completeTargetHandOver() API was called.
 *      NULL Indicates no more packets are buffered. Application must process these buffered
 *      packets before proceeding with normal operations. This is done to maintain the
 *      ordering of countC used to cipher the packets.
 *  @param[out]  errCode
 *      Error Code populated only on error else ignore
 *
 *  \ingroup NETFP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Netfp_getTargetHandOverPackets
(
    uint32_t    targetHandOverId,
    Ti_Pkt**    ptrPkt,
    int32_t*    errCode
)
{
    Netfp_3GPPSecurityChannel*      ptr3GPPSecurityChannel;

    /* Get the 3GPP security channel. */
    ptr3GPPSecurityChannel = (Netfp_3GPPSecurityChannel*)targetHandOverId;
    if (ptr3GPPSecurityChannel == NULL)
    {
        *errCode = NETFP_EINVAL;
        return -1;
    }

    /* Pop off the packet from the handover queue: */
    *ptrPkt = (Ti_Pkt*)QMSS_DESC_PTR(Qmss_queuePop(ptr3GPPSecurityChannel->hoDataBufferQueue));
    if (*ptrPkt == NULL)
    {
        /* Update the channel status to HandOver is completed */
        ptr3GPPSecurityChannel->state = Netfp_ChannelState_HO_COMPLETE;
    }
    else
    {
        /* Invalidate the descriptor */
        Pktlib_getOwnership (ptr3GPPSecurityChannel->ptrNetfpClient->cfg.pktlibInstHandle, *ptrPkt);
    }
    return 0;
}

