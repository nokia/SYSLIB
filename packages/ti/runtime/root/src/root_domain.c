/**
 *   @file  root_domain.c
 *
 *   @brief
 *      The file implements the root domain functionality which is exposed
 *      to the root master and slaves.
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
#include <string.h>

/* ROOT Include Files. */
#include <ti/runtime/root/root.h>
#include <ti/runtime/root/include/root_internal.h>

/* Internal Debug Flag: This is used to display debug information on packet exchange
 * between the Master & Slave. */
#undef ROOT_DEBUG

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 ************************ Local Definitions *******************************
 **************************************************************************/

/**
 * @brief
 *  Status flag which indicates that the domain is active
 */
#define ROOT_DOMAIN_ACTIVE              0x1

/**
 * @brief
 *  Status flag which indicates that the buffer has been allocated but has yet
 *  not been submitted to its peer domain for processing.
 */
#define ROOT_JOSH_BUFFER_ALLOCATED      0x2

/**
 * @brief
 *  Status flag which indicates that the domain has submitted a job to its
 *  peer domain for execution
 */
#define ROOT_JOSH_REQUEST_PENDING       0x4

/**
 * @brief
 *  Status flag which indicates that the domain has received a job result back
 *  after the peer domain for execution
 */
#define ROOT_JOSH_RESPONSE_PENDING      0x8

/**************************************************************************
 *********************** Root Domain Functions ****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the
 *      JOSH framework to allocate a message which is to be sent across
 *      between the root domains.
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be allocated
 *  @param[out]  msgBuffer
 *      Opaque handle to the message to be sent out
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static uint8_t* Root_joshAlloc(uint32_t nodeId, int32_t size, void** msgBuffer)
{
    Root_MasterSlaveMCB*    ptrMasterSlaveMCB;
    Root_JoshBuffer*        ptrJoshBuffer;

    /* Get the pointer to the root master slave MCB */
    ptrMasterSlaveMCB = (Root_MasterSlaveMCB*)nodeId;

    /* Use the JOSH buffer depending upon the direction: */
    if (ptrMasterSlaveMCB->module == Root_Module_SLAVE)
    {
        /* Slave is allocating and the JOSH request will now head towards the master */
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->slave_master;
    }
    else
    {
        /* Master is allocating and the JOSH request will now head towards the slave */
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->master_slave;
    }

    /* Invalidate the contents of the JOSH buffer status */
    ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

    /* If the JOSH buffer has been allocated there is no point; skip and proceed to the next one */
    if (ptrJoshBuffer->status[0] & ROOT_JOSH_BUFFER_ALLOCATED)
    {
        /* Error: Out of memory. */
        System_printf ("Error: Out of memory in the root '%s'\n",
                       (ptrMasterSlaveMCB->module == Root_Module_SLAVE) ? "Slave" : "Master");
        *msgBuffer = NULL;
        return NULL;
    }

    /* Free and available JOSH buffer has been found. Validate and ensure that the MAX size
     * is within limits and we dont */
    if (size > ROOT_INTERNAL_SIZE_BUFFER)
    {
        System_printf ("Error: '%s' root buffer request for %d exceeds max size %d\n",
                       (ptrMasterSlaveMCB->module == Root_Module_SLAVE) ? "Slave" : "Master",
                       size, ROOT_INTERNAL_SIZE_BUFFER);
        *msgBuffer = NULL;
        return NULL;
    }

    /* Mark the JOSH buffer as allocated */
    ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] | ROOT_JOSH_BUFFER_ALLOCATED;

    /* Writeback the cache contents. */
    ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

#ifdef ROOT_DEBUG
    System_printf ("Debug: Allocating %d bytes at address %p\n", size, &ptrJoshBuffer->buffer[0]);
#endif

    /* Return the allocated JOSH buffer. */
    *msgBuffer = (void*)ptrJoshBuffer;
    return &ptrJoshBuffer->buffer[0];
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to free a previously allocated message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   size
 *      Size of the message to be freed up
 *  @param[in]  msgBuffer
 *      Opaque handle to the message to be cleaned up
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Allocated message
 *  @retval
 *      Error   -   NULL
 */
static void Root_joshFree(uint32_t nodeId, int32_t size, void* msgBuffer)
{
    Root_MasterSlaveMCB*    ptrMasterSlaveMCB;
    Root_JoshBuffer*        ptrJoshBuffer;

    /* Get the pointer to the root master slave MCB */
    ptrMasterSlaveMCB = (Root_MasterSlaveMCB*)nodeId;

    /* Get the pointer to the JOSH Buffer which is being cleaned up */
    ptrJoshBuffer = (Root_JoshBuffer*)msgBuffer;

#ifdef ROOT_DEBUG
    System_printf ("Debug: Cleaning %d bytes at address %p\n", size, &ptrJoshBuffer->buffer[0]);
#endif

    /* Cleanup the status and mark it as free and available. */
    ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] & ~(ROOT_JOSH_BUFFER_ALLOCATED | ROOT_JOSH_REQUEST_PENDING | ROOT_JOSH_RESPONSE_PENDING);
    ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to receive a message.
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   readerChannel
 *      Opaque reader channel on which the message is to be received
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has been received
 *  @param[in]  ptrDataBuffer
 *      Data buffer which points to the received message
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Root_joshGet(uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    Root_MasterSlaveMCB*    ptrMasterSlaveMCB;
    Root_JoshBuffer*        ptrJoshBuffer;

    /* Get the pointer to the root master slave MCB */
    ptrMasterSlaveMCB = (Root_MasterSlaveMCB*)nodeId;

    /* Use the JOSH buffer depending upon the direction: */
    if (ptrMasterSlaveMCB->module == Root_Module_SLAVE)
    {
        /********************************************************************
         * Slave module: On the slave we can receive the following
         * (a) JOSH Request  from the master in the MASTER->SLAVE direction
         * (b) JOSH Response from the master in the SLAVE->MASTER direction
         *
         * Determine if the slave has received data or not?
         ********************************************************************/
        if (Root_isSlaveBufferReady(ptrMasterSlaveMCB) == 0)
        {
            /* Slave was not ready. */
            *msgBuffer     = NULL;
            *ptrDataBuffer = NULL;
            return;
        }

        /* Control comes here that the slave was ready with data */
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->master_slave;

        /* Invalidate the contents of the JOSH buffer */
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

        /* Is there an active request pending? */
        if (ptrJoshBuffer->status[0] & ROOT_JOSH_REQUEST_PENDING)
        {
            /* YES. Request is pending; reset the flag to indicate that the request has been processed */
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] & ~ROOT_JOSH_REQUEST_PENDING;
            ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

#ifdef ROOT_DEBUG
            System_printf ("Debug: Rxed JOSH Request data %p [Master->Slave]\n", &ptrJoshBuffer->buffer[0]);
#endif

            /* Return the buffers. */
            *msgBuffer     = (void *)ptrJoshBuffer;
            *ptrDataBuffer = &ptrJoshBuffer->buffer[0];
            return;
        }

        /* Checking for (b) above. */
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->slave_master;
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

        /* Is there an active response pending? */
        if (ptrJoshBuffer->status[0] & ROOT_JOSH_RESPONSE_PENDING)
        {
            /* YES. Request is pending; reset the flag to indicate that the request has been processed */
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] & ~ROOT_JOSH_RESPONSE_PENDING;
            ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

#ifdef ROOT_DEBUG
            System_printf ("Debug: Rxed JOSH Response data %p [Slave->Master]\n", &ptrJoshBuffer->buffer[0]);
#endif
            /* Return the buffers. */
            *msgBuffer     = (void *)ptrJoshBuffer;
            *ptrDataBuffer = &ptrJoshBuffer->buffer[0];
            return;
        }
    }
    else
    {
        /********************************************************************
         * Master module: On the master we can receive the following
         * (a) JOSH Request  from the slave in the SLAVE->MASTER direction
         * (b) JOSH Response from the slave in the MASTER->SLAVE direction
         ********************************************************************/
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->slave_master;

        /* Invalidate the contents of the JOSH buffer */
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

        /* Is there an active request pending? */
        if (ptrJoshBuffer->status[0] & ROOT_JOSH_REQUEST_PENDING)
        {
            /* YES. Request is pending; reset the flag to indicate that the request has been processed */
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] & ~ROOT_JOSH_REQUEST_PENDING;
            ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

#ifdef ROOT_DEBUG
            System_printf ("Debug: Rxed JOSH Request data %p [Slave->Master]\n", &ptrJoshBuffer->buffer[0]);
#endif
            /* Return the buffers. */
            *msgBuffer     = (void *)ptrJoshBuffer;
            *ptrDataBuffer = &ptrJoshBuffer->buffer[0];
            return;
        }

        /* Checking for (b) above. */
        ptrJoshBuffer = &ptrMasterSlaveMCB->ptrSharedMem->master_slave;

        /* Invalidate the contents of the JOSH buffer. */
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
        ptrMasterSlaveMCB->osalFxn.beginMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

        /* Is there an active response pending? */
        if (ptrJoshBuffer->status[0] & ROOT_JOSH_RESPONSE_PENDING)
        {
            /* YES. Request is pending; reset the flag to indicate that the request has been processed */
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] & ~ROOT_JOSH_RESPONSE_PENDING;
            ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);

#ifdef ROOT_DEBUG
            System_printf ("Debug: Rxed JOSH Response data %p [Master->Slave]\n", &ptrJoshBuffer->buffer[0]);
#endif
            /* Return the buffers. */
            *msgBuffer     = (void *)ptrJoshBuffer;
            *ptrDataBuffer = &ptrJoshBuffer->buffer[0];
            return;
        }
    }

    /* Control comes here implies that there was no message pending */
    *msgBuffer     = NULL;
    *ptrDataBuffer = NULL;
    return;
}

/**
 *  @b Description
 *  @n
 *      Josh transport call back function which is invoked by by the JOSH framework
 *      to send a message
 *
 *  @param[in]   nodeId
 *      Node identifier passed during JOSH initialization
 *  @param[in]   writerChannel
 *      Opaque writer channel on which the message is to be transmitted
 *  @param[in]  msgBuffer
 *      Opaque handle to the message which has to be sent
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Root_joshPut(uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    Root_MasterSlaveMCB*    ptrMasterSlaveMCB;
    Root_JoshBuffer*        ptrJoshBuffer;
    int32_t                 errCode;

    /* Get the pointer to the JOSH Buffer which is being cleaned up */
    ptrJoshBuffer = (Root_JoshBuffer*)msgBuffer;

    /* Get the pointer to the root master slave MCB */
    ptrMasterSlaveMCB = (Root_MasterSlaveMCB*)nodeId;

    /* Use the JOSH buffer depending upon the direction: */
    if (ptrMasterSlaveMCB->module == Root_Module_SLAVE)
    {
        /****************************************************************
         * Slave module: From the slave we can send the following:
         * - JOSH Request  to the master in the SLAVE->MASTER direction
         * - JOSH Response to the master in the MASTER->SLAVE direction
         ***************************************************************/
        if (ptrJoshBuffer == &ptrMasterSlaveMCB->ptrSharedMem->slave_master)
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] | ROOT_JOSH_REQUEST_PENDING;
        else
            ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] | ROOT_JOSH_RESPONSE_PENDING;

        /* Writeback the cache */
        ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
        ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

        /* Notify the master */
        Root_notifyMaster (ptrMasterSlaveMCB, &errCode);
        return;
    }

    /****************************************************************
     * Master module: From the master we can send the following:
     * - JOSH Request  to the slave in the  MASTER->SLAVE direction
     * - JOSH Response to the master in the SLAVE->MASTER direction
     ***************************************************************/
    if (ptrJoshBuffer == &ptrMasterSlaveMCB->ptrSharedMem->master_slave)
        ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] | ROOT_JOSH_REQUEST_PENDING;
    else
        ptrJoshBuffer->status[0] = ptrJoshBuffer->status[0] | ROOT_JOSH_RESPONSE_PENDING;

    /* Writeback the cache */
    ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->status[0], CACHE_L2_LINESIZE);
    ptrMasterSlaveMCB->osalFxn.endMemAccess (&ptrJoshBuffer->buffer[0], ROOT_INTERNAL_SIZE_BUFFER);

    /* Notify the slave */
    Root_notifySlave (ptrMasterSlaveMCB, &errCode);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the application domain and to pass
 *      the domain configuration.
 *
 *  @param[in]  rootMasterHandle
 *      Pointer to the root master handle
 *  @param[in]  slaveCoreId
 *      Slave core identifier to which the configuration is being passed.
 *  @param[in]  appId
 *      Application allocated identifier used to identify the application being started
 *  @param[in]  ptrDomainCfg
 *      Pointer to the domain configuration
 *  @param[in]  sizeDomainCfg
 *      Size of the domain configuration
 *  @param[in]  ptrSyslibCfg
 *      Pointer to the SYSLIB configuration
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Root_appInit
(
    Root_MasterHandle   rootMasterHandle,
    Root_SlaveCoreId    slaveCoreId,
    uint32_t            appId,
    void*               ptrDomainCfg,
    uint32_t            sizeDomainCfg,
    Root_SyslibConfig*  ptrSyslibCfg,
    int32_t*            errCode
)
{
    Root_MasterMCB*         ptrRootMasterMCB;
    Root_MasterSlaveMCB*    ptrRootMasterSlaveMCB;
    int32_t                 index;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the root master MCB */
    ptrRootMasterMCB = (Root_MasterMCB*)rootMasterHandle;
    if (ptrRootMasterMCB == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Ensure that the core identifier has been registered with the root master */
    for (index = 0; index < ptrRootMasterMCB->cfg.numSlaves; index++)
    {
        if (ptrRootMasterMCB->cfg.slaveCoreId[index] == slaveCoreId)
            break;
    }

    /* Did we get a match? */
    if (index == ptrRootMasterMCB->cfg.numSlaves)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Get the pointer to the slave MCB */
    ptrRootMasterSlaveMCB = &ptrRootMasterMCB->masterSlaveMCB[index];

	/* Get the actual job which we will execute */
	jobHandle = Josh_findJobByAddress(ptrRootMasterSlaveMCB->joshNodeHandle, (Josh_JobProtype)_Root_appInit);
	if (jobHandle == NULL)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Initialize the arguments; to avoid any junk */
	memset ((void *)&args, 0, sizeof(args));

	/***************************************************************************
	 * This is the function which is to be invoked:
	 *
     * static void _Root_appInit
     * (
     * uint32_t            appId,
     * void*               ptrDomainCfg,
     * Root_SyslibConfig*  ptrSyslibCfg
     * )
	 ****************************************************************************/

	/* Populate the arguments. */
	args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[0].length = sizeof(uint32_t);
	args[1].type   = Josh_ArgumentType_PASS_BY_REF;
	args[1].length = sizeDomainCfg;
	args[2].type   = Josh_ArgumentType_PASS_BY_REF;
	args[2].length = sizeof(Root_SyslibConfig);

	/* Add the arguments. */
	argHandle = Josh_addArguments (ptrRootMasterSlaveMCB->joshNodeHandle, &args[0]);
	if (argHandle == NULL)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}

	/* Setup the arguments.  */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(appId);
	memcpy ((void*)args[1].argBuffer, (void *)ptrDomainCfg, sizeDomainCfg);
    memcpy ((void*)args[2].argBuffer, (void *)ptrSyslibCfg, sizeof(Root_SyslibConfig));

	/* Submit the JOB to JOSH for execution. */
	jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
	if (jobId < 0)
		return -1;

	/* Get the result arguments. */
	if (Josh_getArguments (ptrRootMasterSlaveMCB->joshNodeHandle, jobId, &args[0]) < 0)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Free the JOB Instance. */
	if (Josh_freeJobInstance(ptrRootMasterSlaveMCB->joshNodeHandle, jobId) < 0)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the root slave to inform the root master that the
 *      application identified using the application identifier is operational
 *
 *  @param[in]  rootSlaveHandle
 *      Pointer to the root slave handle which is initiating the request
 *  @param[in]  appId
 *      Application identifier
 *  @param[in]  appDomainHandle
 *      Handle to the application domain on the slave core which is operational
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Root_appUp
(
    Root_SlaveHandle    rootSlaveHandle,
    uint32_t            appId,
    void*               appDomainHandle,
    int32_t*            errCode
)
{
    Root_MCB*               ptrRoot;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the Root MCB */
    ptrRoot = (Root_MCB*)rootSlaveHandle;
    if (ptrRoot == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

	/* Get the actual job which we will execute */
	jobHandle = Josh_findJobByAddress(ptrRoot->masterSlaveMCB.joshNodeHandle, (Josh_JobProtype)_Root_appUp);
	if (jobHandle == NULL)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Initialize the arguments; to avoid any junk */
	memset ((void *)&args, 0, sizeof(args));

	/***************************************************************************
	 * This is the function which is to be invoked:
	 *
     * static void _Root_appUp
     * (
     * uint32_t    appId,
     * void*       appDomainHandle
     * )
	 ****************************************************************************/

	/* Populate the arguments. */
	args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[0].length = sizeof(appId);
	args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[1].length = sizeof(appDomainHandle);

	/* Add the arguments. */
	argHandle = Josh_addArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, &args[0]);
	if (argHandle == NULL)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}

	/* Pass the application domain handle. */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(appId);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(appDomainHandle);

	/* Submit the JOB to JOSH for execution. */
	jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
	if (jobId < 0)
		return -1;

	/* Get the result arguments. */
	if (Josh_getArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, jobId, &args[0]) < 0)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Free the JOB Instance. */
	if (Josh_freeJobInstance(ptrRoot->masterSlaveMCB.joshNodeHandle, jobId) < 0)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the root slave to inform the root master that the
 *      application is *down*. Applications can pass application specific arguments
 *      to the root master
 *
 *  @param[in]  rootSlaveHandle
 *      Pointer to the root slave handle which is initiating the request
 *  @param[in]  appDomainHandle
 *      Application domain handle
 *  @param[in]  arg0
 *      Application specific argument0
 *  @param[in]  arg1
 *      Application specific argument1
 *  @param[in]  arg2
 *      Application specific argument2
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Root_appDown
(
    Root_SlaveHandle    rootSlaveHandle,
    void*               appDomainHandle,
    uint32_t            arg0,
    uint32_t            arg1,
    uint32_t            arg2,
    int32_t*            errCode
)
{
    Root_MCB*               ptrRoot;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the Root MCB */
    ptrRoot = (Root_MCB*)rootSlaveHandle;
    if (ptrRoot == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

	/* Get the actual job which we will execute */
	jobHandle = Josh_findJobByAddress(ptrRoot->masterSlaveMCB.joshNodeHandle, (Josh_JobProtype)_Root_appDown);
	if (jobHandle == NULL)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Initialize the arguments; to avoid any junk */
	memset ((void *)&args, 0, sizeof(args));

	/***************************************************************************
	 * This is the function which is to be invoked:
	 *
     * static void _Root_appDown
     * (
     * void*       appDomainHandle,
     * uint32_t    arg0,
     * uint32_t    arg1,
     * uint32_t    arg2
     * )
	 ****************************************************************************/

	/* Populate the arguments. */
	args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[0].length = sizeof(appDomainHandle);
	args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[1].length = sizeof(arg0);
	args[2].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[2].length = sizeof(arg1);
	args[3].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[3].length = sizeof(arg2);

	/* Add the arguments. */
	argHandle = Josh_addArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, &args[0]);
	if (argHandle == NULL)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}

	/* Pass the application domain handle. */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(appDomainHandle);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(arg0);
    *(uint32_t*)args[2].argBuffer = (uint32_t)josh_toRemoteU32(arg1);
    *(uint32_t*)args[3].argBuffer = (uint32_t)josh_toRemoteU32(arg2);

	/* Submit the JOB to JOSH for execution. */
	jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
	if (jobId < 0)
		return -1;

	/* Get the result arguments. */
	if (Josh_getArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, jobId, &args[0]) < 0)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Free the JOB Instance. */
	if (Josh_freeJobInstance(ptrRoot->masterSlaveMCB.joshNodeHandle, jobId) < 0)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the application domain
 *
 *  @param[in]  rootMasterHandle
 *      Pointer to the root master handle
 *  @param[in]  slaveCoreId
 *      Slave core identifier to which the configuration is being passed.
 *  @param[in]  appDomainHandle
 *      Handle to the application domain on the slave core which is being deinitialized
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Root_appDeinit
(
    Root_MasterHandle   rootMasterHandle,
    Root_SlaveCoreId    slaveCoreId,
    void*               appDomainHandle,
    int32_t*            errCode
)
{
    Root_MasterMCB*         ptrRootMasterMCB;
    Root_MasterSlaveMCB*    ptrRootMasterSlaveMCB;
    int32_t                 index;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the root master MCB */
    ptrRootMasterMCB = (Root_MasterMCB*)rootMasterHandle;
    if (ptrRootMasterMCB == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Ensure that the core identifier has been registered with the root master */
    for (index = 0; index < ptrRootMasterMCB->cfg.numSlaves; index++)
    {
        if (ptrRootMasterMCB->cfg.slaveCoreId[index] == slaveCoreId)
            break;
    }

    /* Did we get a match? */
    if (index == ptrRootMasterMCB->cfg.numSlaves)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

    /* Get the pointer to the slave MCB */
    ptrRootMasterSlaveMCB = &ptrRootMasterMCB->masterSlaveMCB[index];

	/* Get the actual job which we will execute */
	jobHandle = Josh_findJobByAddress(ptrRootMasterSlaveMCB->joshNodeHandle, (Josh_JobProtype)_Root_appDeinit);
	if (jobHandle == NULL)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Initialize the arguments; to avoid any junk */
	memset ((void *)&args, 0, sizeof(args));

	/***************************************************************************
	 * This is the function which is to be invoked:
	 *
	 *  int32_t _Root_appDeinit (void* appDomainHandle)
	 *
	 ****************************************************************************/

	/* Populate the arguments. */
	args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[0].length = sizeof(appDomainHandle);

	/* Add the arguments. */
	argHandle = Josh_addArguments (ptrRootMasterSlaveMCB->joshNodeHandle, &args[0]);
	if (argHandle == NULL)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}

	/* Pass the application domain handle. */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(appDomainHandle);

	/* Submit the JOB to JOSH for execution. */
	jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
	if (jobId < 0)
		return -1;

	/* Get the result arguments. */
	if (Josh_getArguments (ptrRootMasterSlaveMCB->joshNodeHandle, jobId, &args[0]) < 0)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Free the JOB Instance. */
	if (Josh_freeJobInstance(ptrRootMasterSlaveMCB->joshNodeHandle, jobId) < 0)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used by the root slave to inform the root master that the
 *      application has deinitialized itself
 *
 *  @param[in]  rootSlaveHandle
 *      Pointer to the root slave handle
 *  @param[in]  appId
 *      Application identifier
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_FUNCTIONS
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Root_appDeinitialized
(
    Root_SlaveHandle    rootSlaveHandle,
    uint32_t            appId,
    int32_t*            errCode
)
{
    Root_MCB*               ptrRoot;
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;

    /* Get the Root MCB */
    ptrRoot = (Root_MCB*)rootSlaveHandle;
    if (ptrRoot == NULL)
    {
        *errCode = ROOT_EINVAL;
        return -1;
    }

	/* Get the actual job which we will execute */
	jobHandle = Josh_findJobByAddress(ptrRoot->masterSlaveMCB.joshNodeHandle, (Josh_JobProtype)_Root_appDeinitialized);
	if (jobHandle == NULL)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Initialize the arguments; to avoid any junk */
	memset ((void *)&args, 0, sizeof(args));

	/***************************************************************************
	 * This is the function which is to be invoked:
	 *
     * void _Root_appDeinitialized
     * (
     * uint32_t    appId,
     * )
	 ****************************************************************************/

	/* Populate the arguments. */
	args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
	args[0].length = sizeof(uint32_t);

	/* Add the arguments. */
	argHandle = Josh_addArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, &args[0]);
	if (argHandle == NULL)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}

	/* Pass the application domain handle. */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(appId);

	/* Submit the JOB to JOSH for execution. */
	jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
	if (jobId < 0)
		return -1;

	/* Get the result arguments. */
	if (Josh_getArguments (ptrRoot->masterSlaveMCB.joshNodeHandle, jobId, &args[0]) < 0)
	{
		*errCode = ROOT_EINTERNAL;
		return -1;
	}

	/* Free the JOB Instance. */
	if (Josh_freeJobInstance(ptrRoot->masterSlaveMCB.joshNodeHandle, jobId) < 0)
	{
		*errCode = ROOT_EJOSH;
		return -1;
	}
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the domain services
 *
 *  @param[in] ptrMasterSlaveMCB
 *      Root Slave Master control block on which the domain services are available
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup ROOT_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Josh node handle
 *  @retval
 *      Error   -   NULL
 */
Josh_NodeHandle Root_initializeDomainServices
(
    Root_MasterSlaveMCB* ptrMasterSlaveMCB,
    int32_t*             errCode
)
{
    Josh_NodeCfg        nodeCfg;
    Josh_NodeHandle     joshNodeHandle;

    /* Initialize the node information. */
    memset ((void*)&nodeCfg, 0, sizeof(Josh_NodeCfg));

    /* Populate the node configuration: The reader & writer channels are not used
     * since ROOT is using shared memory transport for its operation. */
    nodeCfg.nodeId                     = (uint32_t)ptrMasterSlaveMCB;
    nodeCfg.transportCfg.readerChannel = (void*)ptrMasterSlaveMCB;
    nodeCfg.transportCfg.writerChannel = (void*)ptrMasterSlaveMCB;

    /* Populate the transport interface */
    nodeCfg.transportCfg.transport.alloc = Root_joshAlloc;
    nodeCfg.transportCfg.transport.free  = Root_joshFree;
    nodeCfg.transportCfg.transport.get   = Root_joshGet;
    nodeCfg.transportCfg.transport.put   = Root_joshPut;

    /* Populate the OSAL table */
    nodeCfg.malloc      = ptrMasterSlaveMCB->osalFxn.malloc;
    nodeCfg.free        = ptrMasterSlaveMCB->osalFxn.free;
    nodeCfg.enterCS     = ptrMasterSlaveMCB->osalFxn.enterCS;
    nodeCfg.exitCS      = ptrMasterSlaveMCB->osalFxn.exitCS;
    nodeCfg.createSem   = ptrMasterSlaveMCB->osalFxn.createSem;
    nodeCfg.deleteSem   = ptrMasterSlaveMCB->osalFxn.deleteSem;
    nodeCfg.postSem     = ptrMasterSlaveMCB->osalFxn.postSem;
    nodeCfg.pendSem     = ptrMasterSlaveMCB->osalFxn.pendSem;

    /* Initialize the JOSH. */
    joshNodeHandle = Josh_initNode (&nodeCfg, errCode);
    if (joshNodeHandle == NULL)
    {
        /* Error: JOSH Initialization failed. */
        *errCode = ROOT_EINTERNAL;
        return NULL;
    }

    /* Register the domain services */
    Josh_registerJob(joshNodeHandle, (Josh_JobProtype)_Root_appInit);
    Josh_registerJob(joshNodeHandle, (Josh_JobProtype)_Root_appDeinit);
    Josh_registerJob(joshNodeHandle, (Josh_JobProtype)_Root_appUp);
    Josh_registerJob(joshNodeHandle, (Josh_JobProtype)_Root_appDown);
    Josh_registerJob(joshNodeHandle, (Josh_JobProtype)_Root_appDeinitialized);

    /* The slave/master is operational. */
    if (ptrMasterSlaveMCB->module == Root_Module_SLAVE)
        ptrMasterSlaveMCB->ptrSharedMem->slaveRootStatus[0]  = ROOT_DOMAIN_ACTIVE;
    else
        ptrMasterSlaveMCB->ptrSharedMem->masterRootStatus[0] = ROOT_DOMAIN_ACTIVE;

    /* Writeback the cache contents. */
    ptrMasterSlaveMCB->osalFxn.endMemAccess(&ptrMasterSlaveMCB->ptrSharedMem->masterRootStatus[0], CACHE_L2_LINESIZE);
    return joshNodeHandle;
}

