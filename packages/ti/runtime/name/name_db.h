/**
 *   @file  name_db.h
 *
 *   @brief
 *      Header file for the Name library which exposes the database functionality
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
 *  \par
 */

#ifndef __NAME_DB_H__
#define __NAME_DB_H__

/* SYSLIB Include Files. */
#include <ti/runtime/common/syslib.h>
#include <ti/runtime/name/name.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NAME_SYMBOL
 @{ */

/**
 * @brief   Maximum number of databases which can exist in the system
 */
#define NAME_MAX_DATABASE_INSTANCE       64

/**
@}
*/

/** @addtogroup NAME_NAMEASTRUCT
 @{ */

/**
 * @brief   Handle to the name database
 */
typedef void*   Name_DBHandle;

/**
 * @brief
 *  Name resource buckets
 *
 * @details
 *  The enumeration describes the buckets to which a name can be added.
 *  Each database maintains multiple buckets which can be used to store
 *  information.
 */
typedef enum Name_ResourceBucket
{
    /**
     * @brief   Internal SYSLIB Usage: This bucket is used for storing
     * names created by the SYSLIB modules. Applications are advised not
     * to use this bucket for storing their own information.
     */
    Name_ResourceBucket_INTERNAL_SYSLIB    = 0x0,

    /**
     * @brief   User Defined Type1: These are application defined buckets
     * which can be used for storing application defined names
     */
    Name_ResourceBucket_USER_DEF1          = 0x1,

    /**
     * @brief   User Defined Type2: These are application defined buckets
     * which can be used for storing application defined names
     */
    Name_ResourceBucket_USER_DEF2          = 0x2,

    /**
     * @brief   User Defined Type3: These are application defined buckets
     * which can be used for storing application defined names
     */
    Name_ResourceBucket_USER_DEF3          = 0x3,

    /**
     * @brief   User Defined Type4: These are application defined buckets
     * which can be used for storing application defined names
     */
    Name_ResourceBucket_USER_DEF4          = 0x4,

    /**
     * @brief   MAX Named Resource:
     */
    Name_ResourceBucket_MAX_ALLOWED        = 0x5
}Name_ResourceBucket;

/**
 * @brief
 *  Name resource operation types
 *
 * @details
 *  The enumeration describes the operation which is to be performed on the
 *  remote database
 */
typedef enum Name_ResourceOperationType
{
    /**
     * @brief   Create & replicate the named resource from the local database
     * to the remote database
     */
    Name_ResourceOperationType_CREATE   = 0x1,

    /**
     * @brief   Delete the named resource in the remote database
     */
    Name_ResourceOperationType_DELETE   = 0x2,

    /**
     * @brief   Modify the named resource in the remote database using the values from
     * the local database
     */
    Name_ResourceOperationType_MODIFY   = 0x3
}Name_ResourceOperationType;

/**
 * @brief
 *  Named Resource configuration
 *
 * @details
 *  The structure specifies the configuration which maps a name to
 *  additional meta-information which is then registered in the name
 *  server. Name sharing allows this meta-information to be exchanged
 *  across the system.
 */
typedef struct Name_ResourceCfg
{
    /**
     * @brief  Name of the resource
     */
    char                        name[NAME_MAX_CHAR + 1];

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle1;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle2;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle3;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle4;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle5;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle6;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                    handle7;

    /**
     * @brief  Opaque resource handle(s) associated with the resource.
     */
    uint32_t                   handle8;
}Name_ResourceCfg;

/**
 * @brief
 *  DSP Named Resource Instance configuration
 *
 * @details
 *  The structure describes the information which is required to instantiate
 *  the named resource instance on a specific DSP core.
 */
typedef struct Name_DSPDatabaseCfg
{
    /**
     * @brief  This is the base address which is used to store the named resource
     * instance database information. This database is then visible to all the
     * processing entities. It is recommended that applications define and reserve
     * a memory section in the platform memory map for this purpose.
     */
    uint32_t                        baseNamedResourceAddress;

    /**
     * @brief  This is the size of the shared memory block which is allocated by
     * the application. This is validated during initialization to ensure that the
     * shared memory size is sufficient and is not overwritten.
     */
    uint32_t                        sizeNamedResourceMemory;

    /**
     * @brief  Flag which indicates if the named resource subsystem needs to be
     * initialized or not. If there are multiple DSP cores sharing a common named
     * resource instance it is advised that only of these cores resets the memory
     * to ensure that there is no garbage.
     */
    uint32_t                        initNamedResourceDatabase;

    /**
     * @brief  OSAL API to allocate memory
     */
    Osal_NameDBMalloc               malloc;

    /**
     * @brief  OSAL API to clean memory
     */
    Osal_NameDBFree                 free;

    /**
     * @brief  OSAL API to enter the critical section
     */
    Osal_NameEnterMultipleCoreCS    enterCS;

    /**
     * @brief  OSAL API to exit the critical section
     */
    Osal_NameExitMultipleCoreCS     exitCS;

    /**
     * @brief  OSAL API to invalidate the cache
     */
    Osal_NameBeginMemoryAccess      beginMemAccess;

    /**
     * @brief  OSAL API to writeback the cache
     */
    Osal_NameEndMemoryAccess        endMemAccess;
}Name_DSPDatabaseCfg;

/**
 * @brief
 *  Name Database configuration
 *
 * @details
 *  The structure describes the configuration for the database.
 */
typedef struct Name_DatabaseCfg
{
    /**
     * @brief  Database identifier: Each unique database in the system should have a
     * unique signature.
     */
    uint32_t                instanceId;

    /**
     * @brief  Realm in which the database is executing
     */
    Name_ExecutionRealm     realm;

    /**
     * @brief  Database owner: All resources created by this instance will be tagged
     * with the name of the owner.
     */
    char                    owner[NAME_MAX_CHAR + 1];

    /**
     * @brief  DSP realm named resource instance configuration.
     */
    Name_DSPDatabaseCfg     dspCfg;
}Name_DatabaseCfg;

/*****************************************************************************
 ***************************** Exported API **********************************
 *****************************************************************************/

/* Name Database Services: */
extern Name_DBHandle Name_createDatabase (Name_DatabaseCfg* ptrDatabaseCfg, int32_t* errCode);
extern int32_t Name_deleteDatabase (Name_DBHandle databaseHandle, int32_t* errCode);
extern Name_DBHandle Name_getDatabaseHandle(int32_t databaseId);
extern int32_t Name_getDatabaseInstanceId(Name_DBHandle databaseHandle, int32_t* errCode);
extern int32_t Name_purgeDatabase (Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                   const char* ptrOwnerName, int32_t* errCode);
extern int32_t Name_createResource(Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                   Name_ResourceCfg* ptrNameResourceCfg, int32_t* errCode);
extern int32_t Name_findResource(Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                 const char* ptrName, Name_ResourceCfg* ptrNamedResourceCfg, int32_t* errCode);
extern int32_t Name_modifyResource(Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                 Name_ResourceCfg* ptrNameResourceCfg, int32_t* errCode);
extern int32_t Name_deleteResource(Name_DBHandle databaseHandle, Name_ResourceBucket bucket,
                                 const char* ptrName, int32_t* errCode);
extern int32_t Name_dumpDatabase (Name_DBHandle databaseHandle, Name_ResourceBucket bucket, int32_t* errCode);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* __NAME_DB_H__ */

