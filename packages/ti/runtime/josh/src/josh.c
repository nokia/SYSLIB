/**
 *   @file  josh.c
 *
 *   @brief
 *      The file implements the JOSH (Job Scheduler) which allows
 *      communication between multiple processing entities.
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

/**********************************************************************
 *************************** Include Files ****************************
 **********************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

/* MCSDK Include Files */
#include <ti/csl/csl_chip.h>

/* SYSLIB Include Files. */
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/josh/include/josh_internal.h>
#include <ti/runtime/josh/include/listlib.h>
#include <ti/runtime/msgcom/msgcom.h>

/* For Debugging only. */
#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************ Local Definitions ***************************
 **********************************************************************/

/**
 * @brief
 *  Maximum number of Jobs which can concurrently exist in the System.
 *  There may be only 1 job as long as there is only one thread which
 *  sends josh requests.
 */
#define JOSH_MAX_JOB_INSTANCE           1 //fzm

/**
 * @brief
 *  Status Flag which indicates that the specific JOB instance is mapped to
 *  a synchronous job which implies that the callee will be blocked till the
 *  result packet is received back.
 */
#define JOSH_FLAG_SYNC                  0x1

/**
 * @brief
 *  Status Flag which indicates that the specific JOB instance is mapped to
 *  an asynchronous job which implies that the callee will not be blocked
 *  till the result packet is received back. The application will need to check
 *  by itself if the job has been completed or not
 */
#define JOSH_FLAG_ASYNC                 0x2

/**
 * @brief
 *  Status Flag which indicates that the specific JOB instance has been submitted
 *  and is awaiting its results.
 */
#define JOSH_FLAG_SUBMIT                0x4

/**
 * @brief
 *  Status Flag which indicates that the specific JOB instance has received its
 *  result available.
 */
#define JOSH_FLAG_RESULT                0x8

/**
 * @brief
 * JOSH Debug flag
 */
#undef JOSH_DEBUG

/**********************************************************************
 ************************* Local Structures ***************************
 **********************************************************************/

/**
 * @brief
 *  Job Description
 *
 * @details
 *  The Job is a fundamental unit which explains a finite amount of work
 *  which needs to be completed and which is submitted to the JOSH framework
 *  for scheduling.
 */
typedef struct Josh_JobDesc
{
    /**
     * @brief   Links to other jobs.
     */
    Josh_ListNode    links;

    /**
     * @brief   Unique Job Signature.
     */
    uint32_t         jobSignature;

    /**
     * @brief   Job Function to be executed
     */
    Josh_JobProtype  jobFn;
}Josh_JobDesc;

/**
 * @brief
 *  Job Instance
 *
 * @details
 *  Jobs instances are created when a processing node element submits a
 *  job to execute.
 */
typedef struct Josh_JobInstance
{
    /**
     * @brief   Links to job instances.
     */
    Josh_ListNode               links;

    /**
     * @brief   Each Job Instance is associated with a unique identifer.
     */
    uint16_t                    id;

    /**
     * @brief   Status flag which indicates the state of the JOB
     */
    uint32_t                    status;

    /**
     * @brief   Processing Node Information associated with the Job.
     */
    struct Josh_NodeMCB*        ptrNodeMCB;

    /**
     * @brief   Size of the JOB Packet.
     */
    uint32_t                    jobPacketSize;

    /**
     * @brief  Memory handle which was allocated by the transport engine
     */
    MsgCom_Buffer*              msgBuffer;

    /**
     * @brief  Pointer to the start of the memory for the Josh Packet which stores
     * the arguments.
     */
    Josh_Packet*                ptrJoshPacket;

    /**
     * @brief   Semaphore handle. This is used only for synchronous jobs and the JOSH
     * framework will block on this handle till the job is complete.
     */
    void*                       semHandle;

    /**
     * @brief   Result of the JOB execution.
     */
    uint32_t                    jobResult;

    /**
     * @brief   Result of the JOSH Framework.
     */
    int32_t                     joshResult;
}Josh_JobInstance;

/**
 * @brief
 *  The structure is the Master Control block structure of the JOSH Module
 *
 * @details
 *  The Programmable Job Scheduler framework master control block which is
 *  encapsulates all the information required for the JOSH framework to
 *  work.
 */
typedef struct Josh_NodeMCB
{
    /**
     * @brief   Job List which has all the registered jobs in the system
     * associated with the specific node.
     */
    Josh_JobDesc*                   registeredJobs;

    /**
     * @brief   Job Signature Counter which is used to generated unique
     * signatures for each registered job.
     */
    uint32_t                        jobSignature;

    /**
     * @brief   List of free job instances which exist for the specific node.
     */
    Josh_JobInstance*               freeJobInstanceList;

    /**
     * @brief  Processing Node configuration information.
     */
    Josh_NodeCfg                    nodeCfg;

    /**
     * @brief   Mapper which maps Job Identifier to Job Instances.
     */
    Josh_JobInstance*               jobIdToInstanceMapper[JOSH_MAX_JOB_INSTANCE];
}Josh_NodeMCB;

/**********************************************************************
 *************************** JOSH Functions ***************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to get the transaction identifier associated
 *      with the JOSH packet.
 *
 *  @param[in]  ptrPacket
 *      JOSH packet which is being exchanged between the JOSH nodes
 *
 * \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Transaction Identifier
 */
uint32_t Josh_getTransactionId (uint8_t* ptrPacket)
{
    Josh_Packet*    ptrJoshPacket;

    /* Get the JOSH packet. */
    ptrJoshPacket = (Josh_Packet*)ptrPacket;

    /* Get the transaction identifier and convert it to native format. */
    return josh_toNativeU32(ptrJoshPacket->transactionId);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the node identifier associated
 *      with the JOSH packet.
 *
 *  @param[in]  ptrPacket
 *      JOSH packet which is being exchanged between the JOSH nodes
 *
 * \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Node Identifier
 */
uint32_t Josh_getNodeId (uint8_t* ptrPacket)
{
    Josh_Packet*    ptrJoshPacket;

    /* Get the JOSH packet. */
    ptrJoshPacket = (Josh_Packet*)ptrPacket;

    /* Get the transaction identifier and convert it to native format. */
    return josh_toNativeU32(ptrJoshPacket->nodeId);
}

uint32_t Josh_getJobInstanceId (Josh_ArgHandle argHandle)
{
    Josh_JobInstance* ptrJobInstance = (Josh_JobInstance *)argHandle;
    return ptrJobInstance->id;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the packet type associated
 *      with the JOSH packet.
 *
 *  @param[in]  ptrPacket
 *      JOSH packet which is being exchanged between the JOSH nodes
 *
 * \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Packet Type
 */
uint32_t Josh_getPacketType (uint8_t* ptrPacket)
{
    Josh_Packet*    ptrJoshPacket;

    /* Get the JOSH packet. */
    ptrJoshPacket = (Josh_Packet*)ptrPacket;

    /* Get the transaction identifier and convert it to native format. */
    return josh_toNativeU32(ptrJoshPacket->packetType);
}

/**
 *  @b Description
 *  @n
 *      The function is an internal API which gets an element from the
 *      Job Instance Free List.
 *
 *  @param[in]  ptrNodeMCB
 *      Processing Node information for which a job is being allocated
 *
 * \ingroup JOSH_INTERNAL_FUN
 *
 *  @retval
 *      Pointer to a job instance
 */
static inline Josh_JobInstance* Josh_allocJobInstance(Josh_NodeMCB* ptrNodeMCB)
{
    Josh_JobInstance*   ptrJobInstance;
    void*               semHandle;

    /* Critical Section Start: */
    semHandle = ptrNodeMCB->nodeCfg.enterCS();

    /* Get an element from the free Job Instance List */
    ptrJobInstance = (Josh_JobInstance*)Josh_listRemove((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList);

    /* Critical Section End: */
    ptrNodeMCB->nodeCfg.exitCS(semHandle);

    /* Ensure that the status flags are reset. */
    if(ptrJobInstance)
        ptrJobInstance->status = 0;

    /* Return the Job Instance */
    return ptrJobInstance;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free the  Job Instance back into the free list
 *      after the work has been done.
 *
 *  @param[in]  nodeHandle
 *      Master Node Handle for which the arguments are being prepared.
 *  @param[in]  jobId
 *      Job Identifier which is be marked as completed
 *
 * \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success     -  0
 *  @retval
 *      Error       - <0
 */
int32_t Josh_freeJobInstance(Josh_NodeHandle nodeHandle, int32_t jobId)
{
    void*               semHandle;
    Josh_JobInstance*   ptrJobInstance;
    Josh_NodeMCB*       ptrNodeMCB;

    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;

    if (jobId >= JOSH_MAX_JOB_INSTANCE)
    {
        System_printf("Error: JobId out of the scope: %d (%s:%d)", jobId, __FILE__, __LINE__); //fzm
        /* Error: Fatal error; there is no pending job matching the job identifier. */
        return JOSH_EINTERNAL;
    }

    /* Get the Job instance */
    ptrJobInstance = ptrNodeMCB->jobIdToInstanceMapper[jobId];
    if (ptrJobInstance == NULL)
    {
        System_printf("Error: there is no pending job matching the job identifier: %d (%s:%d)", jobId, __FILE__, __LINE__); //fzm
        /* Error: Fatal error; there is no pending job matching the job identifier. */
        return JOSH_EINTERNAL;
    }

    /* Custom Transport: Free up the packet. */
    ptrNodeMCB->nodeCfg.transportCfg.transport.free
            (ptrNodeMCB->nodeCfg.nodeId, ptrJobInstance->jobPacketSize, (void*)ptrJobInstance->msgBuffer);

    /* Ensure that the status flags are reset. */
    ptrJobInstance->status = 0;

    /* Critical Section Start: */
    semHandle = ptrNodeMCB->nodeCfg.enterCS();

    /* Place the Job Instance back into the free list */
    Josh_listAdd((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList, (Josh_ListNode*)ptrJobInstance);

    /* Critical Section End: */
    ptrNodeMCB->nodeCfg.exitCS(semHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to search all the registered jobs in the framework
 *      for a job matching the signature specified.
 *
 *  @param[in]  ptrNodeMCB
 *      Processing Node information which is being searched for.
 *  @param[in]  jobSignature
 *      Job Signature to be looked
 *
 * \ingroup JOSH_INTERNAL_FUN
 *
 *  @retval
 *      NULL     -  No Job matching the name exists in the framework
 *  @retval
 *      Non NULL -  Handle of the matching job.
 */
static Josh_JobHandle Josh_findJobBySignature(Josh_NodeMCB* ptrNodeMCB, uint32_t jobSignature)
{
    Josh_JobDesc*   ptrJobDesc;
    void*           semHandle;

    /* Critical Section Start: */
    semHandle = ptrNodeMCB->nodeCfg.enterCS();

    /* Cycle through all the jobs in the Job List. */
    ptrJobDesc = (Josh_JobDesc *)Josh_listGetHead((Josh_ListNode**)&ptrNodeMCB->registeredJobs);
    while (ptrJobDesc != NULL)
    {
        /* Check if we found a match or not? */
        if (ptrJobDesc->jobSignature == jobSignature)
            break;

        /* No match; goto the next entry. */
        ptrJobDesc = (Josh_JobDesc *)Josh_listGetNext((Josh_ListNode*)ptrJobDesc);
    }

    /* Critical Section End: */
    ptrNodeMCB->nodeCfg.exitCS(semHandle);
    return ptrJobDesc;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get all the arguments from a processed job.
 *
 *  @param[in]  nodeHandle
 *      Master Node Handle for which the arguments are being prepared.
 *  @param[in]  jobId
 *      Job Identifier which is be marked as completed
 *  @param[in]  args
 *      Argument Properties being prepared for a JOB to send.
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Josh_getArguments (Josh_NodeHandle nodeHandle, int32_t jobId, Josh_Argument* args)
{
    Josh_JobInstance*       ptrJobInstance;
    Josh_NodeMCB*           ptrNodeMCB;
    uint8_t*                ptrArgumentData;
    uint32_t                argIndex;

    /* Sanity check. Ensure arguments are valid. */
    if (args == NULL || nodeHandle == NULL)
    {
        System_printf("Error: Sanity check failed, args:%p nodeHandle:%p (%s:%d)", args, nodeHandle, __FILE__, __LINE__); //fzm
        return JOSH_EINVAL;
    }

    /* Get the processing node information */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;

    /* Get the Job instance */
    ptrJobInstance = ptrNodeMCB->jobIdToInstanceMapper[jobId];
    if (ptrJobInstance == NULL)
    {
        System_printf("Error: there is no pending job matching the job identifier [%d] (%s:%d)", jobId, __FILE__, __LINE__); //fzm

        /* Error: Fatal error; there is no pending job matching the job identifier. */
        return JOSH_EINTERNAL;
    }

    if (ptrJobInstance->ptrJoshPacket == NULL)
    {
        System_printf("Error: couldn't get buffer for jobId: %d (%s:%d)", jobId, __FILE__, __LINE__); //fzm

        return JOSH_EINTERNAL;
    }

    /* Set up the argument data pointer. */
    ptrArgumentData = (uint8_t*)ptrJobInstance->ptrJoshPacket + sizeof (Josh_Packet);

    /* Populate the arguments in the arg buffer. */
    for (argIndex = 0; argIndex < josh_toNativeU32(ptrJobInstance->ptrJoshPacket->numArgs); argIndex++)
    {
        /* Populate the argument data buffer also. */
        args[argIndex].argBuffer = ptrArgumentData + josh_toNativeU32(ptrJobInstance->ptrJoshPacket->argOffsets[argIndex]);
        args[argIndex].length = josh_toNativeU32 (ptrJobInstance->ptrJoshPacket->argSizes[argIndex]);
        args[argIndex].type = (Josh_ArgumentType)josh_toNativeU32 (ptrJobInstance->ptrJoshPacket->argFlags[argIndex]);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add all the arguments and prepare them for a job.
 *
 *  @param[in]  nodeHandle
 *      Master Node Handle for which the arguments are being prepared.
 *  @param[in]  args
 *      Argument Properties being prepared for a JOB to send.
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - Handle to the argument memory block.
 *  @retval
 *      Error   - NULL
 */
Josh_ArgHandle Josh_addArguments (Josh_NodeHandle nodeHandle, Josh_Argument* args)
{
    Josh_JobInstance*       ptrJobInstance;
    Josh_NodeMCB*           ptrNodeMCB;
    uint32_t                argIndex;
    uint32_t                argLen;
    uint32_t                size = 0;
    uint32_t                argOffset = 0;
    uint8_t*                ptrArgumentData;

    /* Sanity Check: Validate the arguments. */
    if ((nodeHandle == NULL) || (args == NULL))
    {
        System_printf("Error: Sanity check failed, args:%p nodeHandle:%p (%s:%d)", args, nodeHandle, __FILE__, __LINE__); //fzm
        return NULL;
    }

    /* Get the processing node information */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;

    /* Allocate a job instance */
    ptrJobInstance = Josh_allocJobInstance(ptrNodeMCB);
    if (ptrJobInstance == NULL)
    {
        System_printf("Couldn't allocate JoshJobInstance (%s:%d)", __FILE__, __LINE__); //fzm
        return NULL;
    }

    /* Determine total size of all the arguments. */
    for (argIndex = 0; argIndex < JOSH_MAX_ARGS; argIndex++)
    {
        /* Get the argument length */
        argLen = args[argIndex].length;

        /* Ensure that the argument length are 4 byte aligned. If not make it. */
        if ((argLen % 4) != 0)
            args[argIndex].length = ((argLen/4) + 1) * 4;

        /* Account for the total argument length. */
        size = size + args[argIndex].length;
    }

    /* Allocate memory for the JOSH Packet. */
    size = size + sizeof(Josh_Packet);

    /* Allocate the packet. */
    ptrJobInstance->ptrJoshPacket = (Josh_Packet*)ptrNodeMCB->nodeCfg.transportCfg.transport.alloc
                                                        (ptrNodeMCB->nodeCfg.nodeId, size, (void**)&ptrJobInstance->msgBuffer);

    /* Ensure that we received memory for the JOSH packet. */
    if (ptrJobInstance->ptrJoshPacket == NULL)
    {
        System_printf("Couldn't get descriptor buffer of size %u (%s:%d)", size, __FILE__, __LINE__); //fzm
        return NULL;
    }

    /* Remember the size of the JOB packet which has been allocated. */
    ptrJobInstance->jobPacketSize = size;

    /* Initialize the number of arguments in the packet. */
    ptrJobInstance->ptrJoshPacket->numArgs = 0;

    /* Get the pointer to the actual argument data which starts after the JOSH Header packet. */
    ptrArgumentData = (uint8_t*)ptrJobInstance->ptrJoshPacket + sizeof(Josh_Packet);

    /* Populate the arguments in the JOSH Packet. */
    for (argIndex = 0; argIndex < JOSH_MAX_ARGS; argIndex++)
    {
        /* Is this a valid argument? */
        if (args[argIndex].type == Josh_ArgumentType_INVALID)
            break;

        /* Populate the argument data buffer also. */
        args[argIndex].argBuffer = ptrArgumentData + argOffset;

        /* All PASS BY VALUE arguments have a fixed length of 4 bytes */
        if (args[argIndex].type == Josh_ArgumentType_PASS_BY_VALUE)
            args[argIndex].length = 4;

        /* Increment the number of arguments. */
        ptrJobInstance->ptrJoshPacket->numArgs++;

        /* Setup the arguments in the JOSH Packet. */
        ptrJobInstance->ptrJoshPacket->argFlags[argIndex]   = josh_toRemoteU32(args[argIndex].type);
        ptrJobInstance->ptrJoshPacket->argSizes[argIndex]   = josh_toRemoteU32(args[argIndex].length);
        ptrJobInstance->ptrJoshPacket->argOffsets[argIndex] = josh_toRemoteU32(argOffset);

        /* The next argument will be offset by the length of the previous argument. */
        argOffset = argOffset + args[argIndex].length;
    }

    /* Ensure that we convert the number of arguments to the proper endianess format. */
    ptrJobInstance->ptrJoshPacket->numArgs = josh_toRemoteU32(ptrJobInstance->ptrJoshPacket->numArgs);

    /* Return the argument handle. */
    return (Josh_ArgHandle)ptrJobInstance;
}

/**
 *  @b Description
 *  @n
 *      The function is called to handle the SUBMIT packet. It will parse the
 *      arguments in the received packet and will invoke the appropriate JOB
 *      function.
 *
 *  @param[in]  ptrNodeMCB
 *      Pointer to the processing node which received the JOSH message.
 *  @param[in]  ptrMessage
 *      Pointer to the message
 *  @param[in]  ptrJoshPacket
 *      Pointer to the JOSH packet which was received.
 *  @param[out] errCode
 *      Error code which is populated on error and is 0 on success (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_INTERNAL_FUN
 *
 *  @retval
 *      Always returns 0.
 */
static int32_t Josh_processSubmitRequest
(
    Josh_NodeMCB*   ptrNodeMCB,
    MsgCom_Buffer*  ptrMessage,
    Josh_Packet*    ptrJoshPacket,
    int32_t*        errCode
)
{
    uint8_t*        ptrDataBuffer;
    Josh_JobDesc*   ptrJobDesc;
    uint32_t        argIndex;
    uint32_t        argFlags;
    uint32_t        numArguments;
    uint32_t        totalPacketSize;
    uint32_t        actualArgs[JOSH_MAX_ARGS];

    /* Get the total number of arguments */
    numArguments = josh_toNativeU32(ptrJoshPacket->numArgs);

    /* Account for the JOSH Packet Size. */
	totalPacketSize = sizeof(Josh_Packet);

    /* Find the Job description using the signature. */
	ptrJobDesc = (Josh_JobDesc *)Josh_findJobBySignature(ptrNodeMCB, josh_toNativeU32(ptrJoshPacket->jobSignature));
	if (ptrJobDesc == NULL)
	{
        System_printf("Error: Could NOT find specified JOB by signature (%s:%d)", __FILE__, __LINE__); //fzm

		/* Error: We could NOT find the specified JOB so set the JOB result appropriately. */
		ptrJoshPacket->jobResult  = 0;
		ptrJoshPacket->joshResult = josh_toRemoteU32((uint32_t)JOSH_ENOJOB);
		ptrJoshPacket->packetType = josh_toRemoteU32(JOSH_PACKET_TYPE_RESULT);
	}
	else
	{
		/* Get the pointer to the argument data */
		ptrDataBuffer = (uint8_t*)ptrJoshPacket + sizeof(Josh_Packet);

		/* Populate the arguments in the JOSH Packet. */
		for (argIndex = 0; argIndex < numArguments; argIndex++)
		{
			/* Get the argument flags */
			argFlags = josh_toNativeU32(ptrJoshPacket->argFlags[argIndex]);

			/* Determine the type of argument. */
			if (argFlags == Josh_ArgumentType_PASS_BY_VALUE)
				actualArgs[argIndex] = *((uint32_t*)ptrDataBuffer);
			else
				actualArgs[argIndex] = (uint32_t)ptrDataBuffer;

			/* Increment the argument data buffer to account for the length. */
			ptrDataBuffer = ptrDataBuffer + josh_toNativeU32(ptrJoshPacket->argSizes[argIndex]);

			/* In the total packet size; account for all the argument sizes also. */
			totalPacketSize = totalPacketSize + josh_toNativeU32(ptrJoshPacket->argSizes[argIndex]);
		}

		/* Debug Message: */
#ifdef JOSH_DEBUG
		System_printf ("***********************************************\n");
		System_printf ("Debug: Job Request Packet\n");
        System_printf ("Debug: Node Id          : 0x%x\n", josh_toNativeU32(ptrJoshPacket->nodeId));
        System_printf ("Debug: Transaction Id   : 0x%x\n", josh_toNativeU32(ptrJoshPacket->transactionId));
		System_printf ("Debug: Job Id           : %d\n", josh_toNativeU32(ptrJoshPacket->jobId));
		System_printf ("Debug: Job Signature    : %d\n", josh_toNativeU32(ptrJoshPacket->jobSignature));
		System_printf ("Debug: Num Arguments    : %d\n", josh_toNativeU32(ptrJoshPacket->numArgs));
		System_printf ("----------------------------\n");
        for (argIndex = 0; argIndex < numArguments; argIndex++)
        {
    		System_printf ("Debug: Arg[%d] Flags     : %x\n", argIndex, josh_toNativeU32(ptrJoshPacket->argFlags[argIndex]));
	    	System_printf ("Debug: Arg[%d] Size      : %d\n", argIndex, josh_toNativeU32(ptrJoshPacket->argSizes[argIndex]));
		    System_printf ("Debug: Arg[%d] Offset    : %d\n", argIndex, josh_toNativeU32(ptrJoshPacket->argOffsets[argIndex]));
    		System_printf ("Debug: Arg[%d]           : %x\n", argIndex, actualArgs[argIndex]);
	    	System_printf ("----------------------------\n");
        }
		System_printf ("***********************************************\n");
#endif

		/* We have all the information now to invoke the JOB. */
		ptrJoshPacket->jobResult = ptrJobDesc->jobFn ((void *)actualArgs[0], (void *)actualArgs[1],
                                                      (void *)actualArgs[2], (void *)actualArgs[3]);

		/* Prepare to send back the RESULT packet */
		ptrJoshPacket->jobResult  = josh_toRemoteU32(ptrJoshPacket->jobResult);
		ptrJoshPacket->joshResult = 0;
		ptrJoshPacket->packetType = josh_toRemoteU32(JOSH_PACKET_TYPE_RESULT);
	}

    /* the other side doesn't wait for the response nor it needs the job result,
     * so we can exit early without sending response */
    if((josh_toNativeU32(ptrJoshPacket->joshJobType) & JOSH_FLAG_SYNC) == 0)
    {
        *errCode = 0;
        return 0;
    }

    /* Send out the packet through the transport
       The message is critical, the other side will block if it's not sent
       Need to keep trying till success*/
    /* Send the packet out. */
    int putStatus = ptrNodeMCB->nodeCfg.transportCfg.transport.put
        (ptrNodeMCB->nodeCfg.nodeId, ptrNodeMCB->nodeCfg.transportCfg.writerChannel, (void*)ptrMessage);

    if (putStatus < 0)
    {
        if (putStatus == -1)
            *errCode = JOSH_EINTERNAL;
        else if (putStatus == -2)
            *errCode = JOSH_EBUSY;

        return -1;
    }

    /* There was no error */
    *errCode = 0;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called to handle the RESULT packet. It will parse the
 *      arguments in the received packet and will invoke the appropriate JOB
 *      function.
 *
 *  @param[in]  ptrNodeMCB
 *      Pointer to the processing node which received the JOSH message.
 *  @param[in]  ptrMessage
 *      Pointer to the message
 *  @param[in]  ptrJoshPacket
 *      Pointer to the JOSH packet which was received.
 *  @param[out] errCode
 *      Error code for the JOSH framework (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_INTERNAL_FUN
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int32_t Josh_processResultRequest
(
    Josh_NodeMCB*   ptrNodeMCB,
    MsgCom_Buffer*  ptrMessage,
    Josh_Packet*    ptrJoshPacket,
    int32_t*        errCode
)
{
    uint32_t            jobIdentifier;
    Josh_JobInstance*   ptrJobInstance;

    /* Get the Job Identifier. */
    jobIdentifier = josh_toNativeU32(ptrJoshPacket->jobId);

    /* Get the Job instance */
    ptrJobInstance = ptrNodeMCB->jobIdToInstanceMapper[jobIdentifier];
    if (ptrJobInstance == NULL)
    {
        System_printf("Error: there is no pending job matching the job identifier:%d (%s:%d)", jobIdentifier, __FILE__, __LINE__); //fzm

        /* Error: Fatal error; there is no pending job matching the job identifier. */
        *errCode = JOSH_EINTERNAL;
        return -1;
    }

    /* Get the JOSH result. */
    ptrJobInstance->joshResult = josh_toNativeU32(ptrJoshPacket->joshResult);

    /* Get the result of the JOB execution. */
    ptrJobInstance->jobResult = josh_toNativeU32(ptrJoshPacket->jobResult);

    /* The job has been completed and the results are available. */
    ptrJobInstance->status = ptrJobInstance->status | JOSH_FLAG_RESULT;

    /* Remember the result packet. */
    memcpy(ptrJobInstance->ptrJoshPacket, ptrJoshPacket, ptrJobInstance->jobPacketSize);

    /* Post the semaphore indicating that the JOB has completed. This needs to be done
     * only for synchronous jobs (async josh jobs don't generate response) */
    ptrNodeMCB->nodeCfg.postSem(ptrJobInstance->semHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is an exported API which is required to be invoked
 *      on the SLAVE processing node only and will check & process any
 *      JOSH submitted requests. This API can be plugged into a Task or
 *      can be called periodically but failure to call this API will
 *      imply that JOSH requests will not get executed and the master
 *      will get blocked.
 *
 *      The function reads from the NODE mapped reader channels and if
 *      there is no message it will return -1 but the error code is
 *      set to JOSH_ENOMSG.
 *
 *  @param[in]  nodeHandle
 *      Slave Node Handle.
 *  @param[out] errCode
 *      Error code which is populated on error and is 0 on success.
 *      (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Josh_receive(Josh_NodeHandle nodeHandle, int32_t* errCode)
{
    Josh_NodeMCB*   ptrNodeMCB;
    void*           ptrMessage;
    Josh_Packet*    ptrJoshPacket;
    uint32_t        packetType;

    /* Get the processing node information. */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;
    if (ptrNodeMCB == NULL)
    {
        System_printf("Error: Sanity check failed, ptrNodeMCB:%p (%s:%d)", ptrNodeMCB, __FILE__, __LINE__); //fzm

        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Receive the packet */
    ptrNodeMCB->nodeCfg.transportCfg.transport.get (ptrNodeMCB->nodeCfg.nodeId,
                        ptrNodeMCB->nodeCfg.transportCfg.readerChannel,
                        (void**)&ptrMessage, (uint8_t**)&ptrJoshPacket);

    if (ptrMessage == NULL)
    {
        *errCode = JOSH_ENOMSG;
        return -1;
    }

    /* Message was received successfully: Get the packet type. */
    packetType = josh_toNativeU32(ptrJoshPacket->packetType);
    if (packetType == JOSH_PACKET_TYPE_SUBMIT)
    {
        /* Process the SUBMIT request. */
        return Josh_processSubmitRequest (ptrNodeMCB, ptrMessage, ptrJoshPacket, errCode);
    }
    else
    {
        /* Process the Result Packet */
        return Josh_processResultRequest(ptrNodeMCB, ptrMessage, ptrJoshPacket, errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to submit a job. The function will then wait for
 *      the results to come back for the JOB.
 *
 *  @param[in]  jobHandle
 *      Handle of the job which is to be submitted.
 *  @param[in]  argHandle
 *      Argument memory handle for the job.
 *  @param[out]  jobResult
 *      Result of the JOB execution
 *  @param[out]  joshErrorCode
 *      Result reported by the JOSH framework (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - Job Identifer
 *  @retval
 *      Error   - <0
 */
int32_t Josh_submitJob
(
    Josh_JobHandle      jobHandle,
    Josh_ArgHandle      argHandle,
    uint32_t*           jobResult,
    int32_t*            joshErrorCode
)
{
    Josh_JobInstance*    ptrJobInstance;
    Josh_JobDesc*        ptrJobDesc;
    Josh_NodeMCB*        ptrNodeMCB;

    /* Sanity Checks: Make sure that a valid job handle and semaphore handle was passed. */
    if ((jobHandle == NULL) || (argHandle == NULL))
    {
        System_printf("Error: Sanity check failed, jobHandle:%p argHandle:%p (%s:%d)", jobHandle, argHandle, __FILE__, __LINE__); //fzm

        *joshErrorCode = JOSH_EINVAL;
        return -1;
    }

    /* Get the job descriptor. */
    ptrJobDesc = (Josh_JobDesc *)jobHandle;

    /* Get the job instance */
    ptrJobInstance = (Josh_JobInstance *)argHandle;

    /* Get the processing node instance: If none is available this is a FATAL internal error. */
    ptrNodeMCB = ptrJobInstance->ptrNodeMCB;
    if (ptrNodeMCB == NULL)
    {
        System_printf("Error: No processing node information is available (%s:%d)", __FILE__, __LINE__); //fzm

        /* Error: No processing node information is available. */
        *joshErrorCode = JOSH_EINTERNAL;
        return -1;
    }

    /* Set the flag in the job instance appropriately. */
    ptrJobInstance->status = ptrJobInstance->status | JOSH_FLAG_SYNC | JOSH_FLAG_SUBMIT;

    /* Populate the remaining fields in the JOSH Packet. */
    ptrJobInstance->ptrJoshPacket->joshJobType   = josh_toRemoteU32(JOSH_FLAG_SYNC);
    ptrJobInstance->ptrJoshPacket->packetType    = josh_toRemoteU32(JOSH_PACKET_TYPE_SUBMIT);
    ptrJobInstance->ptrJoshPacket->jobSignature  = josh_toRemoteU32(ptrJobDesc->jobSignature);
    ptrJobInstance->ptrJoshPacket->jobId         = josh_toRemoteU32(ptrJobInstance->id);
#ifdef __ARMv7
    ptrJobInstance->ptrJoshPacket->transactionId = josh_toRemoteU32((uint32_t)random());
#else
    ptrJobInstance->ptrJoshPacket->transactionId = josh_toRemoteU32(TSCL);
#endif
    ptrJobInstance->ptrJoshPacket->nodeId        = josh_toRemoteU32(ptrNodeMCB->nodeCfg.nodeId);

    /* Send the packet out. */
    int putStatus = ptrNodeMCB->nodeCfg.transportCfg.transport.put
                    (ptrNodeMCB->nodeCfg.nodeId, ptrNodeMCB->nodeCfg.transportCfg.writerChannel, (void*)ptrJobInstance->msgBuffer);

    if (putStatus < 0)
    {
        if (putStatus == -1)
            *joshErrorCode = JOSH_EINTERNAL;
        else if (putStatus == -2)
            *joshErrorCode = JOSH_EBUSY;

        return -1;
    }

    /* We now wait for the JOB result to get back to us. */
    ptrNodeMCB->nodeCfg.pendSem(ptrJobInstance->semHandle);

    /* Get the JOSH result. */
    *joshErrorCode = josh_toNativeU32(ptrJobInstance->joshResult);

    /* Get the result of the JOB execution. */
    *jobResult = josh_toNativeU32(ptrJobInstance->jobResult);
    return ptrJobInstance->id;
}

/**
 *  @b Description
 *  @n
 *      The function is used to submit a job. The will will not block till the
 *      job results are received.
 *
 *  @param[in]  jobHandle
 *      Handle of the job which is to be submitted.
 *  @param[in]  argHandle
 *      Argument memory handle for the job.
 *  @param[out]  errCode
 *      Result reported by the JOSH framework (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - Job Identifer
 *  @retval
 *      Error   - <0
 */
int32_t Josh_submitAsyncJob
(
    Josh_JobHandle      jobHandle,
    Josh_ArgHandle      argHandle,
    int32_t*            errCode
)
{
    Josh_JobInstance*    ptrJobInstance;
    Josh_JobDesc*        ptrJobDesc;
    Josh_NodeMCB*        ptrNodeMCB;

    /* Sanity Checks: Make sure that a valid job handle and semaphore handle was passed. */
    if ((jobHandle == NULL) || (argHandle == NULL))
    {
        System_printf("Error: Sanity check failed, jobHandle:%p argHandle:%p (%s:%d)", jobHandle, argHandle, __FILE__, __LINE__); //fzm

        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Get the job descriptor. */
    ptrJobDesc = (Josh_JobDesc *)jobHandle;

    /* Get the job instance */
    ptrJobInstance = (Josh_JobInstance *)argHandle;

    /* Get the processing node instance: If none is available this is a FATAL internal error. */
    ptrNodeMCB = ptrJobInstance->ptrNodeMCB;
    if (ptrNodeMCB == NULL)
    {
        System_printf("Error: No processing node information is available (%s:%d)", __FILE__, __LINE__); //fzm

        /* Error: No processing node information is available. */
        *errCode = JOSH_EINTERNAL;
        return -1;
    }

    /* Set the flag in the job instance appropriately. */
    ptrJobInstance->status = ptrJobInstance->status | JOSH_FLAG_ASYNC | JOSH_FLAG_SUBMIT;

    /* Populate the remaining fields in the JOSH Packet. */
    ptrJobInstance->ptrJoshPacket->joshJobType   = 0;
    ptrJobInstance->ptrJoshPacket->packetType    = josh_toRemoteU32(JOSH_PACKET_TYPE_SUBMIT);
    ptrJobInstance->ptrJoshPacket->jobSignature  = josh_toRemoteU32(ptrJobDesc->jobSignature);
    ptrJobInstance->ptrJoshPacket->jobId         = josh_toRemoteU32(ptrJobInstance->id);
#ifdef __ARMv7
    ptrJobInstance->ptrJoshPacket->transactionId = josh_toRemoteU32((uint32_t)random());
#else
    ptrJobInstance->ptrJoshPacket->transactionId = josh_toRemoteU32(TSCL);
#endif
    ptrJobInstance->ptrJoshPacket->nodeId        = josh_toRemoteU32(ptrNodeMCB->nodeCfg.nodeId);

    /* Send the packet out. */
    int putStatus = ptrNodeMCB->nodeCfg.transportCfg.transport.put
                    (ptrNodeMCB->nodeCfg.nodeId, ptrNodeMCB->nodeCfg.transportCfg.writerChannel,(void*)ptrJobInstance->msgBuffer);

    if (putStatus < 0)
    {
        if (putStatus == -1)
            *errCode = JOSH_EINTERNAL;
        else if (putStatus == -2)
            *errCode = JOSH_EBUSY;

        return -1;
    }

    /* Return the job identifier. */
    return ptrJobInstance->id;
}

/**
 *  @b Description
 *  @n
 *      The function is used to check the status of an asynchronous job.
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle on which the JOB was submitted
 *  @param[in]  jobId
 *      Job Identifier which was returned when the JOB was submitted
 *  @param[out]  jobResult
 *      Result of the job. This is valid only if the job was completed.
 *  @param[out]  errCode
 *      Result reported by the JOSH framework (@ref JOSH_ERROR_CODE)
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      1 - Job was completed
 *  @retval
 *      0 - Job was still not completed
 *  @retval
 *      Error - <0
 */
int32_t Josh_isJobCompleted
(
    Josh_NodeHandle nodeHandle,
    int32_t         jobId,
    uint32_t*       jobResult,
    int32_t*        errCode
)
{
    Josh_NodeMCB*       ptrNodeMCB;
    Josh_JobInstance*   ptrJobInstance;

    /* Sanity Check: Validate the arguments. */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;
    if (ptrNodeMCB == NULL)
    {
        System_printf("Error: Sanity check failed, ptrNodeMCB:%p (%s:%d)", ptrNodeMCB, __FILE__, __LINE__); //fzm

        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Get the Job instance from the job identifer. */
    ptrJobInstance = ptrNodeMCB->jobIdToInstanceMapper[jobId];
    if (ptrJobInstance == NULL)
    {
        System_printf("Error: there is no pending job matching the job identifier: %d (%s:%d)", jobId, __FILE__, __LINE__); //fzm

        /* Error: Fatal error; there is no pending job matching the job identifier. */
        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Ensure that the JOB has been submitted. */
    if ((ptrJobInstance->status & JOSH_FLAG_SUBMIT) == 0)
    {
        System_printf("Error: JOB has not been submitted (id:%d) status:0x%X (%s:%d)", jobId, ptrJobInstance->status, __FILE__, __LINE__); //fzm

        /* Error: Invalid job. The */
        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* The API is allowed only for ASYNC jobs; we are not supporting other modes */
    if ((ptrJobInstance->status & JOSH_FLAG_ASYNC) == 0)
    {
        System_printf("Error: checking completeness of non-async job (id:%d) status:0x%X (%s:%d)", jobId, ptrJobInstance->status, __FILE__, __LINE__);

        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Has the result been received? */
    if (ptrJobInstance->status & JOSH_FLAG_RESULT)
    {
        /* Job has been completed. Get the result of the job also. */
        *jobResult = ptrJobInstance->jobResult;
        return 1;
    }

    /* Results still not received. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to search all the registered jobs in the framework
 *      for a job matching the address specified.
 *
 *  @param[in]  nodeHandle
 *      Processing node handle for which the job is to be found.
 *  @param[in]  jobFn
 *      Address of the job which is trying to be found.
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      NULL     -  No Job matching the name exists in the framework
 *  @retval
 *      Non NULL -  Handle of the matching job.
 */
Josh_JobHandle Josh_findJobByAddress(Josh_NodeHandle nodeHandle, Josh_JobProtype jobFn)
{
    Josh_JobDesc*   ptrJobDesc;
    Josh_NodeMCB*   ptrNodeMCB;

    /* Sanity Check: Validate the arguments. */
    if ((jobFn == NULL) || (nodeHandle == NULL))
    {
        System_printf("Error: Sanity check failed, jobFn:%p nodeHandle:%p (%s:%d)", jobFn, nodeHandle, __FILE__, __LINE__); //fzm
        return NULL;
    }

    /* Get the processing node. */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;

    /* Cycle through all the jobs in the Job List. */
    ptrJobDesc = (Josh_JobDesc *)Josh_listGetHead((Josh_ListNode**)&ptrNodeMCB->registeredJobs);
    while (ptrJobDesc != NULL)
    {
        /* Check if we found a match or not? */
        if (ptrJobDesc->jobFn == jobFn)
            return (Josh_JobHandle)ptrJobDesc;

        /* No match; goto the next entry. */
        ptrJobDesc = (Josh_JobDesc *)Josh_listGetNext((Josh_ListNode*)ptrJobDesc);
    }

    /* Control comes here that there was no job found. */
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register a job with the JOSH framework
 *      Jobs can be invoked only if they have been initially registered
 *      with the framework.
 *
 *  @param[in]  nodeHandle
 *      Processing node handle for which the job is to be registered.
 *  @param[in]  jobFn
 *      Address of the job which is to be registered.
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - Valid Job Handle
 *  @retval
 *      Error   - NULL
 */
Josh_JobHandle Josh_registerJob(Josh_NodeHandle nodeHandle, Josh_JobProtype jobFn)
{
    Josh_JobDesc*   ptrJobDesc;
    Josh_NodeMCB*   ptrNodeMCB;

    /* Sanity Check: Validate the arguments. */
    if ((jobFn == NULL) || (nodeHandle == NULL))
        return NULL;

    /* Get the processing node. */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;

    /* Sanity Check: Job functions need to be unique we dont want duplicate
     * jobs to exist in the system. */
    if (Josh_findJobByAddress (nodeHandle, jobFn) != NULL)
        return NULL;

    /* Allocate memory for the job. */
    ptrJobDesc = (Josh_JobDesc *)ptrNodeMCB->nodeCfg.malloc(sizeof(Josh_JobDesc), 0);
    if (ptrJobDesc == NULL)
        return NULL;

    /* Initialize the allocated block of memory. */
    memset ((void*)ptrJobDesc, 0, sizeof(Josh_JobDesc));

    /* Populate the structure. */
    ptrJobDesc->jobFn        = jobFn;
    ptrJobDesc->jobSignature = ptrNodeMCB->jobSignature++;

    /* Add the job to the global list of all registered jobs in the core. */
    Josh_listAdd ((Josh_ListNode**)&ptrNodeMCB->registeredJobs, (Josh_ListNode*)ptrJobDesc);

    /* Return the created job handle. */
    return (Josh_JobHandle)ptrJobDesc;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize JOSH for the specific processing
 *      node.
 *
 *  @param[in]  ptrNodeCfg
 *      Processing Node configuration.
 *  @param[out] errCode
 *      Error code which is populated on error and is 0 on success.
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - Returns the handle to the registered processing node.
 *  @retval
 *      Error   - NULL
 */
Josh_NodeHandle Josh_initNode
(
    Josh_NodeCfg*   ptrNodeCfg,
    int32_t*        errCode
)
{
    uint16_t            index;
    Josh_JobInstance*   ptrJobInstance;
    Josh_NodeMCB*       ptrNodeMCB;

    /* Basic Validation: Validate the arguments. */
    if (ptrNodeCfg == NULL)
    {
        /* Error: Invalid argument */
        *errCode = JOSH_EINVAL;
        return NULL;
    }

    /* Basic Validation: Ensure that the reader & writer channels have been specified */
    if ((ptrNodeCfg->transportCfg.readerChannel == NULL) || (ptrNodeCfg->transportCfg.writerChannel == NULL))
    {
        /* Error: Invalid Argument */
        *errCode = JOSH_EINVAL;
        return NULL;
    }

    /* Ensure that the Transport Interface has been specified. */
    if ((ptrNodeCfg->transportCfg.transport.alloc == NULL) ||
        (ptrNodeCfg->transportCfg.transport.free  == NULL) ||
        (ptrNodeCfg->transportCfg.transport.get   == NULL) ||
        (ptrNodeCfg->transportCfg.transport.put   == NULL))
    {
        /* Error: Invalid Argument */
        *errCode = JOSH_EINVAL;
        return NULL;
    }

    /* Ensure that the OSAL Interface has been specified. */
    if ((ptrNodeCfg->malloc  == NULL)   || (ptrNodeCfg->free == NULL)       ||
        (ptrNodeCfg->enterCS == NULL)   || (ptrNodeCfg->exitCS == NULL)     ||
        (ptrNodeCfg->createSem == NULL) || (ptrNodeCfg->deleteSem == NULL)  ||
        (ptrNodeCfg->postSem == NULL)   || (ptrNodeCfg->pendSem == NULL))
    {
        *errCode = JOSH_EINVAL;
        return NULL;
    }

    /* Allocate memory for the JOSH MCB  */
    ptrNodeMCB = ptrNodeCfg->malloc(sizeof(Josh_NodeMCB), 0);
    if (ptrNodeMCB == NULL)
    {
        /* Error: Out of memory */
        *errCode = JOSH_ENOMEM;
        return NULL;
    }

    /* Initialize the Master Control Block. */
    memset ((void *)ptrNodeMCB, 0, sizeof(Josh_NodeMCB));

    /* Copy the processing node information */
    memcpy((void *)&ptrNodeMCB->nodeCfg, (void*)ptrNodeCfg, sizeof(Josh_NodeCfg));

    /* Populate and initialize the Free Job Instance List */
    for (index = 0; index < JOSH_MAX_JOB_INSTANCE; index++)
    {
        /* Allocate memory for the Job Instance */
        ptrJobInstance = (Josh_JobInstance *)ptrNodeMCB->nodeCfg.malloc(sizeof(Josh_JobInstance), 0);
        if (ptrJobInstance == NULL)
        {
            /* Error: Out of memory */
            *errCode = JOSH_ENOMEM;
            return NULL;
        }

        /* Initialize the allocated block of memory. */
        memset ((void*)ptrJobInstance, 0, sizeof(Josh_JobInstance));

        /* Populate the fields. */
        ptrJobInstance->id         = index;
        ptrJobInstance->ptrNodeMCB = ptrNodeMCB;

        /* Create a semaphore for the SYNC Jobs. */
        ptrJobInstance->semHandle  = ptrNodeMCB->nodeCfg.createSem();
        if (ptrJobInstance->semHandle == NULL)
        {
            /* Error:  No Semaphore was passed to the JOSH framework. */
            *errCode = JOSH_ENOSEM;
            return NULL;
        }

        /* Populate the mapper also. */
        ptrNodeMCB->jobIdToInstanceMapper[ptrJobInstance->id] = ptrJobInstance;

        /* Add this job instance to the free list. */
        Josh_listAdd ((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList, (Josh_ListNode*)ptrJobInstance);
    }

    /* Job Scheduler framework is operational. */
    *errCode = 0;
    return (Josh_NodeHandle)ptrNodeMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the JOSH node handle and shutdown
 *      the JOSH services.
 *
 *  @param[in]  nodeHandle
 *      JOSH node handle to be closed.
 *  @param[out] errCode
 *      Error code which is populated on error
 *
 *  \ingroup JOSH_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Josh_deinitNode
(
    Josh_NodeHandle nodeHandle,
    int32_t*        errCode
)
{
    Josh_JobInstance*   ptrJobInstance;
    Josh_NodeMCB*       ptrNodeMCB;
    Josh_JobDesc*       ptrJobDesc;
    uint32_t            pendingJobCounter;

    /* Get the JOSH node MCB */
    ptrNodeMCB = (Josh_NodeMCB*)nodeHandle;
    if (ptrNodeMCB == NULL)
    {
        *errCode = JOSH_EINVAL;
        return -1;
    }

    /* Initialize the pending job counter */
    pendingJobCounter = 0;

    /* Cleanup all the JOB instances. */
    ptrJobInstance = (Josh_JobInstance*)Josh_listGetHead((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList);
    while (ptrJobInstance != NULL)
    {
        /* Is the job pending? */
        if (ptrJobInstance->status != 0)
        {
            /* YES. This job cannot be killed right now. Increment the counter and goto the next one */
            pendingJobCounter++;

            /* Get the next job */
            ptrJobInstance = (Josh_JobInstance*)Josh_listGetNext((Josh_ListNode*)ptrJobInstance);
        }
        else
        {
            /* NO. Delete the semaphore associated with the job */
            ptrNodeMCB->nodeCfg.deleteSem(ptrJobInstance->semHandle);

            /* Remove the JOB Instance from the list. */
            Josh_listRemoveNode ((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList, (Josh_ListNode*)ptrJobInstance);

            /* Cleanup the memory associated with the job. */
            ptrNodeMCB->nodeCfg.free(ptrJobInstance, sizeof(Josh_JobInstance));

            /* Get the next element in the head of the list. */
            ptrJobInstance = (Josh_JobInstance*)Josh_listGetHead((Josh_ListNode**)&ptrNodeMCB->freeJobInstanceList);
        }
    }

    /* Are there any pending job? If so then the JOSH cannot be completed killed. */
    if (pendingJobCounter != 0)
    {
        *errCode = JOSH_EBUSY;
        return -1;
    }

    /* No more pending jobs. Clean up all the registered jobs */
    ptrJobDesc = (Josh_JobDesc*)Josh_listRemove ((Josh_ListNode**)&ptrNodeMCB->registeredJobs);
    while (ptrJobDesc != NULL)
    {
        /* Cleanup the memory associated with the job. */
        ptrNodeMCB->nodeCfg.free(ptrJobDesc, sizeof(Josh_JobDesc));

        /* Get the next element from the job list. */
        ptrJobDesc = (Josh_JobDesc*)Josh_listRemove ((Josh_ListNode**)&ptrNodeMCB->registeredJobs);
    }

    /* Cleanup the MCB */
    ptrNodeMCB->nodeCfg.free (ptrNodeMCB, sizeof(Josh_NodeMCB));
    return 0;
}
