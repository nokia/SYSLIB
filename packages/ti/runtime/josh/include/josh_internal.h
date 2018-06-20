/**
 *   @file  josh_internal.h
 *
 *   @brief
 *      Internal Header file used within the Josh Module.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
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
 *
 */
#ifndef __JOSH_INTERNAL_H__
#define __JOSH_INTERNAL_H__

#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup JOSH_DATASTRUCT
 @{ */

/**
 * @brief
 *  The structure describes a Job packet
 *
 * @details
 *  The JOSH framework encapsulates the job information in the following
 *  packet format and passes it to the transport engine. The packet format
 *  here describes the job to be executed, arguments etc. Responses for the
 *  job execution are also passed in the same format back.
 */
typedef struct Josh_Packet
{
    /**
     * @brief   Packet Type
     */
    uint32_t            packetType;

    /**
     * @brief   Transaction Identifier
     */
    uint32_t            transactionId;

    /**
     * @brief   Node Identifier:
     */
    uint32_t            nodeId;

    /**
     * @brief   The Job Identifier which is being processed.
     */
    uint32_t            jobId;

    /**
     * @brief   The Job Signature to be processed.
     */
    uint32_t            jobSignature;

    /**
     * @brief   Number of arguments in the packet.
     */
    uint32_t            numArgs;

    /**
     * @brief   Argument Flags which describe the arguments are specified
     * here.
     */
    uint32_t            argFlags[JOSH_MAX_ARGS];

    /**
     * @brief   Size of the arguments in the packet if the argument data
     * is being passed by reference.
     */
    uint32_t            argSizes[JOSH_MAX_ARGS];

    /**
     * @brief   Offset into the packet where the actual argument is stored.
     */
    uint32_t            argOffsets[JOSH_MAX_ARGS];

    /**
     * @brief   JOSH Result.
     */
    uint32_t            joshResult;

    /**
     * @brief   Result of the Job
     */
    uint32_t            jobResult;

    /**
     * @brief   Type of Job (sync/async)
     */
    uint32_t            joshJobType;
}Josh_Packet;

/**
@}
*/


#ifdef __cplusplus
}
#endif

#endif /* __JOSH_INTERNAL_H__ */


