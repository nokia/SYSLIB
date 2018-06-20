/**
 *   @file  name_client.c
 *
 *   @brief
 *      The file implements the name client module. Name clients provide
 *      the necessary name services to the applications and are responsible
 *      for interfacing with the name proxy
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* MCSDK Include files */
#include <ti/csl/csl_cache.h>

/* SYSLIB Include files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/josh/josh.h>
#include <ti/runtime/name/name_proxyClient.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**************************************************************************
 **************************** Client Functions ****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the registered job which creates the resource on the
 *      remote database
 *
 *  @param[in]  databaseInstanceId
 *      Database Instance identifier on which the operation is to be performed
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  ptrOwnerName
 *      Name of the owner on whose behalf the name resource is being created
 *  @param[out] ptrResourceCfg
 *      Resource configuration to be added to the database
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Name_createRemoteName
(
    uint32_t                databaseInstanceId,
    Name_ResourceBucket     bucket,
    const char*             ptrOwnerName,
    Name_ResourceCfg*       ptrResourceCfg
)
{
    Name_DBHandle       databaseHandle;
    int32_t             errCode;

    /* Get the database handle from the instance identifier */
    databaseHandle = Name_getDatabaseHandle (databaseInstanceId);
    if (databaseHandle == NULL)
        return NAME_EINVAL;

    /* Create the resource with the owner in the local database */
    if (Name_createResourceOwner (databaseHandle, bucket, ptrResourceCfg, ptrOwnerName, &errCode) < 0)
        return errCode;

    /* Database was created with the new resource. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered job which deletes the resource on the
 *      remote database
 *
 *  @param[in]  databaseInstanceId
 *      Database Instance identifier on which the operation is to be performed
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  ptrName
 *      Name of the resource to be deleted
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Name_deleteRemoteName
(
    uint32_t                databaseInstanceId,
    Name_ResourceBucket     bucket,
    const char*             ptrName
)
{
    Name_DBHandle       databaseHandle;
    int32_t             errCode;

    /* Get the database handle from the instance identifier */
    databaseHandle = Name_getDatabaseHandle (databaseInstanceId);
    if (databaseHandle == NULL)
        return NAME_EINVAL;

    /* Delete the name in the local database */
    if (Name_deleteResource (databaseHandle, bucket, ptrName, &errCode) < 0)
        return errCode;

    /* Resource was deleted successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered job which modifies the resource on the
 *      remote database
 *
 *  @param[in]  databaseInstanceId
 *      Database Instance identifier on which the operation is to be performed
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[out] ptrResourceCfg
 *      Resource configuration to be modified in the database
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Name_modifyRemoteName
(
    uint32_t                databaseInstanceId,
    Name_ResourceBucket     bucket,
    Name_ResourceCfg*       ptrResourceCfg
)
{
    Name_DBHandle       databaseHandle;
    int32_t             errCode;

    /* Get the database handle from the instance identifier */
    databaseHandle = Name_getDatabaseHandle (databaseInstanceId);
    if (databaseHandle == NULL)
        return NAME_EINVAL;

    /* Create the resource with the owner in the local database */
    if (Name_modifyResource (databaseHandle, bucket, ptrResourceCfg, &errCode) < 0)
        return errCode;

    /* Database was modified successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the name by searching for it on the
 *      remote database
 *
 *  @param[in]  remoteDatabaseId
 *      Remote Database Identifier
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  ptrName
 *      Name of the resource to be searched
 *  @param[out] ptrResourceCfg
 *      Resource configuration populated
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t _Name_getRemoteName
(
    uint32_t                remoteDatabaseId,
    Name_ResourceBucket     bucket,
    const char*             ptrName,
    Name_ResourceCfg*       ptrResourceCfg
)
{
    Name_DBHandle       databaseHandle;
    int32_t             errCode;

    /* Get the database handle from the instance identifier */
    databaseHandle = Name_getDatabaseHandle (remoteDatabaseId);
    if (databaseHandle == NULL)
        return NAME_EINVAL;

    /* Find the resource in the local remote database. */
    if (Name_findResource (databaseHandle, bucket, ptrName, ptrResourceCfg, &errCode) < 0)
        return errCode;

    /* Database was modified successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create the name on the remote database
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  name
 *      Name of the resource that is to be added
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Name_createRemoteName
(
    Name_ClientMCB*         ptrNameClient,
    Name_ResourceBucket     bucket,
    const char*             name,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Name_ResourceCfg        nameResourceCfg;
    char                    ownerName[NAME_MAX_CHAR];
    char*                   ptrRemoteOwnerName;
    Name_ResourceCfg*       ptrRemoteResourceCfg;
    int32_t                 databaseInstanceId;

    /* Use the local database to get the name resource configuration from the local database */
    if (Name_findResourceOwner (ptrNameClient->cfg.databaseHandle, bucket, name,
                                &nameResourceCfg, &ownerName[0], errCode) < 0)
    {
        /* Error: Resource does NOT exist in the local database; so it cannot be created on the
         * remote database. */
        return -1;
    }

    /* Get the database instance identifier */
    databaseInstanceId = Name_getDatabaseInstanceId(ptrNameClient->cfg.databaseHandle, errCode);
    if (databaseInstanceId < 0)
        return -1;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNameClient->joshHandle, (Josh_JobProtype)_Name_createRemoteName);
    if (jobHandle == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Name_createRemoteName
     *  (
     *  uint32_t                databaseInstanceId,
     *  Name_ResourceBucket     bucket,
     *  char*                   ptrOwnerName,
     *  Name_ResourceCfg*       ptrResourceCfg,
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Name_ResourceBucket);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = NAME_MAX_CHAR;

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(Name_ResourceCfg);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNameClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(databaseInstanceId);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(bucket);

    /* Get the pointer to the owner name */
    ptrRemoteOwnerName = (char*)args[2].argBuffer;
    strncpy (ptrRemoteOwnerName, ownerName, NAME_MAX_CHAR);

    /* Get the pointer to the reource configuration. */
    ptrRemoteResourceCfg = (Name_ResourceCfg*)args[3].argBuffer;
    memcpy ((void*)ptrRemoteResourceCfg, (void *)&nameResourceCfg, sizeof (Name_ResourceCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNameClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = (int32_t)result;

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNameClient->joshHandle, jobId) < 0)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the name on the remote database
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  name
 *      Name of the resource to be deleted
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Name_deleteRemoteName
(
    Name_ClientMCB*         ptrNameClient,
    Name_ResourceBucket     bucket,
    const char*             name,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    char*                   ptrRemoteName;
    int32_t                 databaseInstanceId;

    /* Get the database instance identifier */
    databaseInstanceId = Name_getDatabaseInstanceId(ptrNameClient->cfg.databaseHandle, errCode);
    if (databaseInstanceId < 0)
        return -1;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNameClient->joshHandle, (Josh_JobProtype)_Name_deleteRemoteName);
    if (jobHandle == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Name_deleteRemoteName
     *  (
     *  uint32_t                databaseInstanceId,
     *  Name_ResourceBucket     bucket,
     *  char*                   ptrName
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Name_ResourceBucket);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = NAME_MAX_CHAR;

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNameClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(databaseInstanceId);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(bucket);

    /* Get the pointer to the name to be deleted */
    ptrRemoteName = (char*)args[2].argBuffer;
    strncpy (ptrRemoteName, name, NAME_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNameClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = (int32_t)result;

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNameClient->joshHandle, jobId) < 0)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to modify the name on the remote database
 *
 *  @param[in]  ptrNameClient
 *      Pointer to the name client
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  name
 *      Name of the resource that has to be pushed
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Name_modifyRemoteName
(
    Name_ClientMCB*         ptrNameClient,
    Name_ResourceBucket     bucket,
    const char*             name,
    int32_t*                errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    Name_ResourceCfg        nameResourceCfg;
    Name_ResourceCfg*       ptrRemoteResourceCfg;
    int32_t                 databaseInstanceId;

    /* Use the local database to get the name resource configuration from the local database */
    if (Name_findResource (ptrNameClient->cfg.databaseHandle, bucket, name,
                           &nameResourceCfg, errCode) < 0)
    {
        /* Error: Resource does NOT exist in the local database; so it cannot be modified on the
         * remote database. */
        return -1;
    }

    /* Get the database instance identifier */
    databaseInstanceId = Name_getDatabaseInstanceId(ptrNameClient->cfg.databaseHandle, errCode);
    if (databaseInstanceId < 0)
        return -1;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNameClient->joshHandle, (Josh_JobProtype)_Name_modifyRemoteName);
    if (jobHandle == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Name_modifyRemoteName
     *  (
     *  uint32_t                databaseInstanceId,
     *  Name_ResourceBucket     bucket,
     *  Name_ResourceCfg*       ptrResourceCfg
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Name_ResourceBucket);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = sizeof(Name_ResourceCfg);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNameClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(databaseInstanceId);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(bucket);

    /* Get the pointer to the reource configuration. */
    ptrRemoteResourceCfg = (Name_ResourceCfg*)args[2].argBuffer;
    memcpy ((void*)ptrRemoteResourceCfg, (void *)&nameResourceCfg, sizeof (Name_ResourceCfg));

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNameClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = (int32_t)result;

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNameClient->joshHandle, jobId) < 0)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create, modify and delete the named resources
 *      between the DSP and ARM realms.
 *
 *  @param[in]  clientHandle
 *      Name client handle
 *  @param[in]  name
 *      Name of the resource that has to be pushed
 *  @param[in]  bucket
 *      Bucket on which the operation is to be done.
 *  @param[in]  operation
 *      Type of operation which is to be performed on the remote name database
 *  @param[out] errCode
 *      Error Code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_push
(
    Name_ClientHandle           clientHandle,
    const char*                 name,
    Name_ResourceBucket         bucket,
    Name_ResourceOperationType  operation,
    int32_t*                    errCode
)
{
    Name_ClientMCB* ptrNameClient;
    int32_t         status;

    /* Sanity Check: Validate the arguments. */
    if ((clientHandle == NULL) || (name == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Get the name client */
    ptrNameClient = (Name_ClientMCB*)clientHandle;

    /* Perform the operation: */
    switch (operation)
    {
        case Name_ResourceOperationType_CREATE:
        {
            status = Name_createRemoteName (ptrNameClient, bucket, name, errCode);
            break;
        }
        case Name_ResourceOperationType_DELETE:
        {
            status = Name_deleteRemoteName (ptrNameClient, bucket, name, errCode);
            break;
        }
        case Name_ResourceOperationType_MODIFY:
        {
            status = Name_modifyRemoteName (ptrNameClient, bucket, name, errCode);
            break;
        }
        default:
        {
            *errCode = NAME_EINVAL;
            return -1;
        }
    }
    return status;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get a resource from the remote database. This is
 *      an expensive operation since it crosses the realm to perform a search in
 *      the remote database. If resources are to be shared explicitly applications
 *      should use the "Name_push" API to do so.
 *
 *  @param[in]  clientHandle
 *      Name client handle
 *  @param[in]   remoteDatabaseId
 *      Instance identifier for the remote database
 *  @param[in]   bucket
 *      Bucket to be searched for the name
 *  @param[in]   ptrName
 *      Pointer to the name to be searched
 *  @param[out]   ptrResourceCfg
 *      Pointer to the name resource configuration populated if the name is found
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Name_get
(
    Name_ClientHandle           clientHandle,
    uint32_t                    remoteDatabaseId,
    Name_ResourceBucket         bucket,
    const char*                 ptrName,
    Name_ResourceCfg*           ptrResourceCfg,
    int32_t*                    errCode
)
{
    Josh_Argument           args[JOSH_MAX_ARGS];
    Josh_JobHandle          jobHandle;
    Josh_ArgHandle          argHandle;
    uint32_t                result;
    int32_t                 jobId;
    char*                   ptrRemoteName;
    Name_ResourceCfg*       ptrRemoteResourceCfg;
    Name_ClientMCB*         ptrNameClient;

    /* Sanity Check: Validate the arguments. */
    if ((clientHandle == NULL) || (ptrName == NULL) || (ptrResourceCfg == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Get the pointer to the name client block. */
    ptrNameClient = (Name_ClientMCB*)clientHandle;

    /* Get the actual job which we will execute */
    jobHandle = Josh_findJobByAddress(ptrNameClient->joshHandle, (Josh_JobProtype)_Name_getRemoteName);
    if (jobHandle == NULL)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Initialize the arguments; to avoid any junk */
    memset ((void *)&args, 0, sizeof(args));

    /***************************************************************************
     * This is the function which is to be invoked:
     *
     *  int32_t _Name_getRemoteName
     *  (
     *  uint32_t                remoteDatabaseId,
     *  Name_ResourceBucket     bucket,
     *  char*                   ptrName,
     *  Name_ResourceCfg*       ptrResourceCfg
     *  )
     ****************************************************************************/

    /* Populate the arguments.
     * - Argument 1: */
    args[0].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[0].length = sizeof(uint32_t);

    /*  - Argument 2: */
    args[1].type   = Josh_ArgumentType_PASS_BY_VALUE;
    args[1].length = sizeof(Name_ResourceBucket);

    /*  - Argument 3: */
    args[2].type   = Josh_ArgumentType_PASS_BY_REF;
    args[2].length = NAME_MAX_CHAR;

    /*  - Argument 4: */
    args[3].type   = Josh_ArgumentType_PASS_BY_REF;
    args[3].length = sizeof(Name_ResourceCfg);

    /* Add the arguments. */
    argHandle = Josh_addArguments (ptrNameClient->joshHandle, &args[0]);
    if (argHandle == NULL)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }

    /* Populate the arguments */
    *(uint32_t*)args[0].argBuffer = (uint32_t)josh_toRemoteU32(remoteDatabaseId);
    *(uint32_t*)args[1].argBuffer = (uint32_t)josh_toRemoteU32(bucket);

    /* Get the pointer to the owner name */
    ptrRemoteName = (char*)args[2].argBuffer;
    strncpy (ptrRemoteName, ptrName, NAME_MAX_CHAR);

    /* Submit the JOB to JOSH for execution. */
    jobId = Josh_submitJob(jobHandle, argHandle, &result, errCode);
    if (jobId < 0)
        return -1;

    /* Get the result arguments. */
    if (Josh_getArguments (ptrNameClient->joshHandle, jobId, &args[0]) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }

    /* Copy the error code value. */
    *errCode = (int32_t)result;

    /* Get the pointer to the reource configuration and copy the populated the resource
     * configuration also. */
    ptrRemoteResourceCfg = (Name_ResourceCfg*)args[3].argBuffer;
    memcpy ((void *)ptrResourceCfg, (void*)ptrRemoteResourceCfg, sizeof (Name_ResourceCfg));

    /* Free the JOB Instance. */
    if (Josh_freeJobInstance(ptrNameClient->joshHandle, jobId) < 0)
    {
        *errCode = NAME_EJOSH;
        return -1;
    }
    return (int32_t)result;
}

/**
 *  @b Description
 *  @n
 *      This function is used to register the exported client services with
 *      the JOSH node
 *
 *  @param[in]  joshHandle
 *      JOSH Node handle
 *
 *  \ingroup NAME_INTERNAL_FUN
 *
 *  @retval
 *      Not applicable
 */
static void Name_registerClientJobs (Josh_NodeHandle joshHandle)
{
    Josh_registerJob(joshHandle, (Josh_JobProtype)_Name_createRemoteName);
    Josh_registerJob(joshHandle, (Josh_JobProtype)_Name_deleteRemoteName);
    Josh_registerJob(joshHandle, (Josh_JobProtype)_Name_modifyRemoteName);
    Josh_registerJob(joshHandle, (Josh_JobProtype)_Name_getRemoteName);
    return;
}

/**
 *  @b Description
 *  @n
 *      This function is used to create a name client and connect this with a
 *      Name Proxy. Clients can only be created if the agent server has been
 *      created and synchronized with the peer server.
 *
 *  @param[in]  ptrNameClientCfg
 *      Pointer to the name client configuration
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   Name Client Handle
 *  @retval
 *      Error   -   NULL
 */
Name_ClientHandle Name_initClient
(
    Name_ClientCfg* ptrNameClientCfg,
    int32_t*        errCode
)
{
    Name_ResourceCfg        namedResourceCfg;
    Name_ClientMCB*         ptrNameClient;

    /* Sanity Check: Validate the arguments */
    if (ptrNameClientCfg == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the OSAL call functions have been populated correctly. */
    if ((ptrNameClientCfg->malloc         == NULL) || (ptrNameClientCfg->free         == NULL)    ||
        (ptrNameClientCfg->beginMemAccess == NULL) || (ptrNameClientCfg->endMemAccess == NULL)    ||
        (ptrNameClientCfg->enterCS        == NULL) || (ptrNameClientCfg->exitCS       == NULL)    ||
        (ptrNameClientCfg->createSem      == NULL) || (ptrNameClientCfg->deleteSem    == NULL)    ||
        (ptrNameClientCfg->postSem        == NULL) || (ptrNameClientCfg->pendSem      == NULL))
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that a valid database handle is being used. */
    if (ptrNameClientCfg->databaseHandle == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Check if the server has been created or not?  */
    if (Name_findResource(ptrNameClientCfg->databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB,
                          &ptrNameClientCfg->proxyName[0], &namedResourceCfg, errCode) < 0)
    {
        /* Error: Resource was NOT found or there was an internal error. */
        if (*errCode != NAME_ENOTFOUND)
        {
            /* Internal Error: Named resource module has returned an error. We return the error code
             * of the resource manager back to the application. */
            return NULL;
        }

        /* Resource does not exist. */
        *errCode = NAME_ENOTREADY;
        return NULL;
    }

    /* Proxy has been created but check if it is operational. Do not allow creation of the name clients
     * if this is not the case because the name clients will use the services and these will get dropped
     * since the proxies are not synchronized. */
    if (namedResourceCfg.handle3 == 0)
    {
        *errCode = NAME_ENOTREADY;
        return NULL;
    }

    /* Allocate memory for the client MCB */
    ptrNameClient = (Name_ClientMCB*)ptrNameClientCfg->malloc(sizeof(Name_ClientMCB), 0);
    if (ptrNameClient == NULL)
    {
        *errCode = NAME_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated block of memory */
    memset ((void *)ptrNameClient, 0, sizeof(Name_ClientMCB));

    /* Populate the name client configuration: */
    memcpy ((void *)&ptrNameClient->cfg, (void *)ptrNameClientCfg, sizeof(Name_ClientCfg));

    /* Initialize the realm client specific transports */
    if (Name_createClientTransport (ptrNameClient, errCode) < 0)
    {
        /* Clean up the memory: */
        ptrNameClientCfg->free (ptrNameClient, sizeof(Name_ClientMCB));
        return NULL;
    }

    /* Register the client services: */
    Name_registerClientJobs (ptrNameClient->joshHandle);
    return (Name_ClientHandle)ptrNameClient;
}

/**
 *  @b Description
 *  @n
 *      This function is used to delete a name client and deregister this with
 *      the name proxy. The function can only be invoked under the following
 *      conditions:
 *          - Application should kill client execution
 *          - Application should ensure that there the client services are no
 *            longer being used.
 *
 *  @param[in]  clientHandle
 *      Handle to the name client to be deleted
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteClient (Name_ClientHandle clientHandle, int32_t* errCode)
{
    Name_ClientMCB*         ptrNameClient;

    /* Get the pointer to the name client. */
    ptrNameClient = (Name_ClientMCB*)clientHandle;
    if (ptrNameClient == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Shutdown the realm client specific transports */
    if (Name_deleteClientTransport (ptrNameClient, errCode) < 0)
        return -1;

    /* Cleanup the memory */
    ptrNameClient->cfg.free (ptrNameClient, sizeof(Name_ClientMCB));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is used to execute the name client. The function needs to
 *      be called periodically if the application is using the name client
 *      services to get/push names between realms.
 *
 *  @param[in]  clientHandle
 *      Handle to the name client
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void Name_executeClient (Name_ClientHandle clientHandle)
{
    Name_ClientMCB* ptrNameClient;
    int32_t         errCode;

    /* Get the pointer to the name client */
    ptrNameClient = (Name_ClientMCB*)clientHandle;
    if (ptrNameClient == NULL)
        return;

    /* Receive and process any JOSH messages: */
    Josh_receive(ptrNameClient->joshHandle, &errCode);
    return;
}

