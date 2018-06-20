/**
 *   @file  name_dspDatabase.c
 *
 *   @brief
 *      The file implements the name database module. The name database
 *      allows applications the ability to store the information into
 *      the central database.
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

/* MCSDK Include files. */
#include <ti/csl/csl_cacheAux.h>

/* SYSLIB Include files */
#include <ti/runtime/resmgr/resmgr.h>
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/include/name_internal.h>

/* For Debugging only. */
#ifdef __ARMv7
#define System_printf   printf
#else
#include <xdc/runtime/System.h>
#endif

/**********************************************************************
 ************************ Internal Declarations ***********************
 **********************************************************************/

/**
 * @brief
 *  Linked List.
 *
 * @details
 * The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 */
typedef struct Name_DBListNode
{
    /**
     * @brief  Pointer to the next element in the list.
     */
	struct Name_Resource*	p_next;

    /**
     * @brief  Pointer to the previous element in the list.
     */
    struct Name_Resource*   p_prev;
}Name_DBListNode;

/**
 * @brief
 *  Name Resource
 *
 * @details
 *  The structure describes the name resource entry which maps the name
 *  with additional meta-information.
 */
typedef struct Name_Resource
{
    /**
     * @brief  Links to other named resources.
     */
    Name_DBListNode         links;

    /**
     * @brief  Named Resource Configuration.
     */
    Name_ResourceCfg        cfg;

    /**
     * @brief  Named Resource Configuration.
     */
    char                    owner[NAME_MAX_CHAR + 1];

    /**
     * @brief  Padding for cache alignment
     */
    uint8_t                 padding[19];
}Name_Resource;

/**
 * @brief
 *  Named Resource Type Database
 *
 * @details
 *  Each named resource type database is a hash bucket linked list implementation
 *  of named resource entries.
 */
typedef struct Name_BucketDatabase
{
    /**
     * @brief  Hash Bucket which stores the named resources.
     */
    Name_Resource*      hash[NAME_DATABASE_HASH_BUCKET];
}Name_BucketDatabase;

/**
 * @brief
 *  Named Resource Database
 *
 * @details
 *  This structure describes the database.
 */
typedef struct Name_Database
{
    /**
     * @brief  Each database is made up of buckets
     */
    Name_BucketDatabase     bucket[Name_ResourceBucket_MAX_ALLOWED];
}Name_Database;

/**
 * @brief
 *  Database Instance
 *
 * @details
 *  This structure describes the instance of the database
 */
typedef struct Name_DatabaseInstance
{
    /**
     * @brief  Database configuration
     */
    Name_DatabaseCfg        cfg;

    /**
     * @brief  Pointer to the database
     */
    Name_Database*          ptrDatabase;
}Name_DatabaseInstance;

/**********************************************************************
 ************************ Global Declarations *************************
 **********************************************************************/

/**
 * @brief   List of all the registered database instances.
 */
Name_DatabaseInstance*  gDatabaseInstance[NAME_MAX_DATABASE_INSTANCE] = { 0 };

/**********************************************************************
 ************************* Database Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is called to add a node to the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  ptr_list
 *      This is the list to which the node is to be added.
 *  @param[in]  ptr_node
 *      This is the node which is to be added.
 *
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Not Applicable
 */
static void Name_listDBAdd
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource**         ptr_list,
    Name_Resource*          ptr_node
)
{
	Name_Resource*	ptr_head;

	/* Check if the list is empty ? */
	if (*ptr_list == NULL)
	{
		/* YES the list is empty. Initialize the links */
		ptr_node->links.p_next = NULL;
        ptr_node->links.p_prev = NULL;

		/* Initialize the LIST */
		*ptr_list = ptr_node;

        /* CACHE Writeback: Writeback the new head. */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_node, sizeof(Name_Resource));
	}
    else
    {
    	/* No the list was NOT empty. Add the node to the beginning of list.
         * Get the current head of the list. */
    	ptr_head = *ptr_list;

    	/* Initialize the new head of the list. */
	    ptr_node->links.p_next = ptr_head;
        ptr_node->links.p_prev = NULL;

        /* Update the old head to point to the new head */
        ptr_head->links.p_prev = ptr_node;

        /* Update the pointer to the head of the list. */
    	*ptr_list = ptr_node;

        /* CACHE Writeback: Writeback the old and new head. */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_head, sizeof(Name_Resource));
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_node, sizeof(Name_Resource));
    }
	return;
}

/**
 *  @b Description
 *  @n
 *      The function is called to remove the head node from the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  ptr_list
 *      This is the pointer to the list from where nodes will be removed.
 *
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Pointer to the head of the list.
 */
static Name_Resource* Name_listDBRemove
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource**         ptr_list
)
{
	Name_Resource*	ptr_head;
	Name_Resource*	ptr_node;

	/* Check if the list is empty ? */
	if (*ptr_list == NULL)
		return NULL;

	/* Get the head of the list. */
	ptr_node = *ptr_list;

	/* Move the head to the next element in the list. */
	ptr_head = ptr_node->links.p_next;
	*ptr_list = ptr_head;

    /* Did we remove the last element?*/
    if (ptr_head != NULL)
    {
        /* No; in that case update the pointers for the new head. */
        ptr_head->links.p_prev = NULL;

        /* CACHE Writeback: Writeback the new head */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_head, sizeof(Name_Resource));
    }

	/* Kill the links before returning the OLD head. */
	ptr_node->links.p_next = NULL;
    ptr_node->links.p_prev = NULL;

    /* CACHE Writeback: Writeback the removed node */
    ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_node, sizeof(Name_Resource));

    /* Return the head. */
	return ptr_node;
}

/**
 *  @b Description
 *  @n
 *      The function is called to remove the specified node from the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  ptr_list
 *      This is the pointer to the list from where node will be removed.
 *  @param[in]  ptr_remove
 *      This is the node which is to be removed.
 *
 *  \ingroup RES_MGR_DSP_INTERNAL_FUN
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
static int32_t Name_listDBRemoveNode
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource**         ptr_list,
    Name_Resource*          ptr_remove
)
{
	Name_Resource*	ptr_next;
	Name_Resource*	ptr_prev;

    /* Are there any nodes in the list? */
    if (*ptr_list == NULL)
		return -1;

    /* Are we removing the head? */
    if (ptr_remove == *ptr_list)
    {
        /* Use the other API to acheive the needful. */
        Name_listDBRemove(ptrDatabaseInstance, ptr_list);
        return 0;
    }

    /* OK; we are trying to remove a non head element; so lets get the
     * previous and next pointer of the elements that needs to be removed. */
    ptr_prev = ptr_remove->links.p_prev;
    ptr_next = ptr_remove->links.p_next;

    /* Invalidate the cache for the previous & next nodes also */
    if (ptr_prev != NULL)
        ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptr_prev, sizeof(Name_Resource));
    if (ptr_next != NULL)
        ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptr_next, sizeof(Name_Resource));

    /* Kill the Links for element that is being removed. */
    ptr_remove->links.p_prev = NULL;
    ptr_remove->links.p_next = NULL;

    /* Are we removing the last element */
    if (ptr_next == NULL)
    {
        /* The last element points to nothing. */
        ptr_prev->links.p_next = NULL;

        /* CACHE Writeback: Writeback the contents of the removed node and its
         * previous node only since the NEXT node is NULL. */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_remove, sizeof(Name_Resource));
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptr_prev,   sizeof(Name_Resource));
        return 0;
    }

    /* We are trying to remove an element in the middle of the list. */
	ptr_prev->links.p_next = ptr_next;
    ptr_next->links.p_prev = ptr_prev;

    /* CACHE Writeback: Writeback the contents of the removed node and its
     * previous & next nodes also. */
    ptrDatabaseInstance->cfg.dspCfg.endMemAccess(ptr_remove, sizeof(Name_Resource));
    ptrDatabaseInstance->cfg.dspCfg.endMemAccess(ptr_prev,   sizeof(Name_Resource));
    ptrDatabaseInstance->cfg.dspCfg.endMemAccess(ptr_next,   sizeof(Name_Resource));

	/* Successful. */
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal function which checks if the name exists in the specified
 *      hash bucket. The function does NOT hold any locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  ptrHashBucket
 *      Pointer to the hash bucket where the search is to be done.
 *  @param[in]  name
 *      Name of the resource which is to be found
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the named resource entry which is found.
 *  @retval
 *      Error   -   NULL
 */
static inline Name_Resource* Name_find
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource*          ptrHashBucket,
    const char*             name
)
{
    /* Search the hash bucket */
    while (ptrHashBucket != NULL)
    {
        /* Invalidate the cache & read the contents from the actual memory. */
        ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrHashBucket, sizeof(Name_Resource));

        /* Compare the name. */
        if (strncmp(name, ptrHashBucket->cfg.name, NAME_MAX_CHAR) == 0)
            break;

        /* No match. Get the next entry in the list */
        ptrHashBucket = ptrHashBucket->links.p_next;
    }
    /* Return the entry. */
    return ptrHashBucket;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to add the name configuration in
 *      the database. The function performs the necessary cache operations
 *      and holds the appropriate locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket on which the operation is to be performed
 *  @param[in]  ptrNameResource
 *      Pointer to the name resource to be added
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static inline int32_t Name_databaseAdd
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket,
    Name_Resource*          ptrNameResource
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)&ptrNameResource->cfg.name[0], strlen(ptrNameResource->cfg.name), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Check for duplicate names in the has bucket? */
    if (Name_find(ptrDatabaseInstance, ptrBucketDatabase->hash[hashCode], ptrNameResource->cfg.name) == NULL)
    {
        /* Add the entry to the hash bucket in the database. */
        Name_listDBAdd (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrNameResource);

        /* Writeback the named resource type database completely; since the hash buckets might have changed */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptrBucketDatabase, sizeof(Name_BucketDatabase));
    }
    else
    {
        /* Error: Duplicate name detected. */
        success = -1;
    }

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return success;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to find the name configuration in
 *      the database. The function performs the necessary cache operations
 *      and holds the appropriate locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket on which the operation is to be performed
 *  @param[in]  ptrName
 *      Name of the resource which is to be found
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the named resource entry which is found.
 *  @retval
 *      Error   -   NULL
 */
static inline Name_Resource* Name_databaseFind
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket,
    const char*             ptrName
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    Name_Resource*            ptrNameResource;
    uint32_t                  hashCode;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)ptrName, strlen(ptrName), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Find the name in the corresponding hash bucket. */
    ptrNameResource = Name_find(ptrDatabaseInstance, ptrBucketDatabase->hash[hashCode], ptrName);

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return ptrNameResource;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to modify the name configuration in the database
 *      The function performs the necessary cache operations and holds the
 *      appropriate locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket on which the operation is to be performed
 *  @param[in]  ptrResourceCfg
 *      Pointer to the name resource configuration
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t Name_databaseModify
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket,
    Name_ResourceCfg*       ptrResourceCfg
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)&ptrResourceCfg->name[0], strlen(ptrResourceCfg->name), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Find the matching resource entry. */
    ptrResource = Name_find(ptrDatabaseInstance, ptrBucketDatabase->hash[hashCode], ptrResourceCfg->name);
    if (ptrResource != NULL)
    {
        /* Match found: Overwrite the configuration */
        memcpy ((void *)&ptrResource->cfg, (void*)ptrResourceCfg, sizeof(Name_ResourceCfg));

        /* Writeback the resource; since its configuration has been modified. */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptrResource, sizeof(Name_Resource));
    }
    else
    {
        /* Error: Name was not found. */
        success = -1;
    }

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return success;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to delete the name from the database
 *      The function performs the necessary cache operations and holds the
 *      appropriate locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket from which the name is to be removed
 *  @param[in]  ptrName
 *      Name of the resource which is to be found
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t Name_databaseDelete
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket,
    const char*             ptrName
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)ptrName, strlen(ptrName), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Find the matching resource entry. */
    ptrResource = Name_find(ptrDatabaseInstance, ptrBucketDatabase->hash[hashCode], ptrName);
    if (ptrResource != NULL)
    {
        /* Match found: Remove from the hash list */
        Name_listDBRemoveNode (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrResource);

        /* Writeback the bucket database since the hash buckets have been modified. */
        ptrDatabaseInstance->cfg.dspCfg.endMemAccess (ptrBucketDatabase, sizeof(Name_BucketDatabase));
    }
    else
    {
        /* Error: Name was not found. */
        success = -1;
    }

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);

    /* Cleanup the memory */
    if (success == 0)
        ptrDatabaseInstance->cfg.dspCfg.free (Name_MallocMode_GLOBAL, ptrResource, sizeof(Name_Resource));

    return success;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to purge all the entries in the database
 *      The function performs the necessary cache operations and holds the
 *      appropriate locks
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket on which the operation is to be performed
 *  @param[in]  ptrOwnerName
 *      Pointer to the owner name
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t Name_databasePurge
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket,
    const char*             ptrOwnerName
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Cycle through all the hash in the bucket */
    for (hashCode = 0; hashCode < NAME_DATABASE_HASH_BUCKET; hashCode++)
    {
        /* Get the resource from the hash bucket. */
        ptrResource = ptrBucketDatabase->hash[hashCode];
        while (ptrResource != NULL)
        {
            /* Do we need to purge the entry? */
            if ((ptrOwnerName != NULL) && (strcmp (ptrOwnerName, ptrResource->owner) != 0))
            {
                /* NO. Owner name was specified and it does not match the resource owner */
                ptrResource = ptrResource->links.p_next;
                continue;
            }

            /* YES. Either the name was not specified or it was a match; remove the node from the
             * hash list. */
            Name_listDBRemoveNode (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrResource);

            /* Cleanup the memory for the resource */
            ptrDatabaseInstance->cfg.dspCfg.free (Name_MallocMode_GLOBAL, ptrResource, sizeof(Name_Resource));

            /* Restart from the head again. */
            ptrResource = ptrBucketDatabase->hash[hashCode];
        }
    }

    /* Writeback the cache contents of the entire bucket database */
    ptrDatabaseInstance->cfg.dspCfg.endMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Internal function which is used to display the contents of a specific
 *      bucket.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *  @param[in]  bucket
 *      Bucket on which the operation is to be performed
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   Number of entries
 */
static inline uint32_t Name_databaseDisplay
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_ResourceBucket     bucket
)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    void*                     csHandle;
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    uint32_t                  numEntries = 0;

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    System_printf ("-----------------------------------------\n");
    System_printf ("Database Handle %p Bucket %d\n", ptrDatabaseInstance, bucket);

    /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
     * copy of the database from the memory.  */
    ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

    /* Cycle through all the hash in the bucket */
    for (hashCode = 0; hashCode < NAME_DATABASE_HASH_BUCKET; hashCode++)
    {
        /* Get the resource from the hash bucket. */
        ptrResource = ptrBucketDatabase->hash[hashCode];
        while (ptrResource != NULL)
        {
            /* Increment the number of entries in the table. */
            numEntries = numEntries + 1;

            /* Display the Named resource configuration. */
            System_printf ("NR%d: Name: '%s' Owner: '%s'\n", numEntries, ptrResource->cfg.name, ptrResource->owner);
            System_printf ("      Handle1: 0x%08x Handle2: 0x%08x Handle3: 0x%08x Handle4: 0x%08x \n",
                    ptrResource->cfg.handle1, ptrResource->cfg.handle2, ptrResource->cfg.handle3, ptrResource->cfg.handle4);
            System_printf ("      Handle5: 0x%08x Handle6: 0x%08x Handle7: 0x%08x Handle8: 0x%08x \n",
                    ptrResource->cfg.handle5, ptrResource->cfg.handle6, ptrResource->cfg.handle7, ptrResource->cfg.handle8);

            /* Get the next entry. */
            ptrResource = ptrResource->links.p_next;
        }
    }

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return numEntries;
}

#if 0
/**
 *  @b Description
 *  @n
 *      Internal function which is used to get the number of entries in the database.
 *      The function performs the necessary cache operations and holds the appropriate locks.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success -   Number of entries in the database
 */
static inline uint32_t Name_databaseNumEntries(Name_DatabaseInstance* ptrDatabaseInstance)
{
    Name_BucketDatabase*      ptrBucketDatabase;
    Name_ResourceBucket       bucket;
    void*                     csHandle;
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    uint32_t                  numEntries = 0;

    /* Critical Section Enter: This is a multicore critical section */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();

    /* Cycle through all the buckets. */
    for (bucket = Name_ResourceBucket_INTERNAL_SYSLIB; bucket < Name_ResourceBucket_MAX_ALLOWED; bucket++)
    {
        /* Get the pointer to the bucket database */
        ptrBucketDatabase = &ptrDatabaseInstance->ptrDatabase->bucket[bucket];

        /* Invalidate the cache contents of the entire bucket database: This will ensure we get a fresh
         * copy of the database from the memory.  */
        ptrDatabaseInstance->cfg.dspCfg.beginMemAccess(ptrBucketDatabase, sizeof(Name_BucketDatabase));

        /* Cycle through all the hash in the bucket */
        for (hashCode = 0; hashCode < NAME_DATABASE_HASH_BUCKET; hashCode++)
        {
            /* Get the resource from the hash bucket. */
            ptrResource = ptrBucketDatabase->hash[hashCode];
            while (ptrResource != NULL)
            {
                /* There is a valid entry in the hash bucket; increment the statistics. */
                numEntries++;

                /* Get the next entry in the list */
                ptrResource = ptrResource->links.p_next;
            }
        }
    }

    /* Critical Section End */
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);
    return numEntries;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to create a resource. Resources are created
 *      using the client name.
 *
 *  @param[in]   databaseHandle
 *      Name Database Handle
 *  @param[in]   bucket
 *      Bucket to which the resource is added
 *  @param[in]   ptrResourceCfg
 *      Pointer to the name resource configuration
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
int32_t Name_createResource
(
    Name_DBHandle           databaseHandle,
    Name_ResourceBucket     bucket,
    Name_ResourceCfg*       ptrResourceCfg,
    int32_t*                errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;
    Name_Resource*          ptrNameResource;

    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrResourceCfg == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Get the database handle. */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;

    /* Allocate memory for the named resource block. */
    ptrNameResource = (Name_Resource*)ptrDatabaseInstance->cfg.dspCfg.malloc(Name_MallocMode_GLOBAL, sizeof(Name_Resource), 0);
    if (ptrNameResource == NULL)
    {
        *errCode = NAME_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrNameResource, 0, sizeof(Name_Resource));

    /* Populate the named resource configuration. */
    memcpy ((void *)&ptrNameResource->cfg, (void *)ptrResourceCfg, sizeof(Name_ResourceCfg));

    /* Tag the resource name: */
    strncpy (ptrNameResource->owner, ptrDatabaseInstance->cfg.owner, NAME_MAX_CHAR);

    /* Add the Named resource to the TYPE database. */
    if (Name_databaseAdd (ptrDatabaseInstance, bucket, ptrNameResource) < 0)
    {
        /* Error: Duplicate name detected. Named resources should be unique for the bucket */
        *errCode = NAME_EDUP;

        /* Cleanup the allocated memory block also. */
        ptrDatabaseInstance->cfg.dspCfg.free (Name_MallocMode_GLOBAL, ptrNameResource, sizeof(Name_Resource));
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a resource with an owner name. This is an internal
 *      function and is not exposed to the applications.
 *
 *  @param[in]   databaseHandle
 *      Name Database Handle
 *  @param[in]   bucket
 *      Bucket to which the resource is added
 *  @param[in]   ptrResourceCfg
 *      Pointer to the name resource configuration
 *  @param[in]   ptrOwnerName
 *      Pointer to the owner name
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Name_createResourceOwner
(
    Name_DBHandle           databaseHandle,
    Name_ResourceBucket     bucket,
    Name_ResourceCfg*       ptrResourceCfg,
    const char*             ptrOwnerName,
    int32_t*                errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;
    Name_Resource*          ptrNameResource;

    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrResourceCfg == NULL) || (ptrOwnerName == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Get the database handle. */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;

    /* Allocate memory for the named resource block. */
    ptrNameResource = (Name_Resource*)ptrDatabaseInstance->cfg.dspCfg.malloc(Name_MallocMode_GLOBAL, sizeof(Name_Resource), 0);
    if (ptrNameResource == NULL)
    {
        *errCode = NAME_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrNameResource, 0, sizeof(Name_Resource));

    /* Populate the named resource configuration. */
    memcpy ((void *)&ptrNameResource->cfg, (void *)ptrResourceCfg, sizeof(Name_ResourceCfg));

    /* Tag the resource name: Using the specified owner name */
    strncpy (ptrNameResource->owner, ptrOwnerName, NAME_MAX_CHAR);

    /* Add the Named resource to the TYPE database. */
    if (Name_databaseAdd (ptrDatabaseInstance, bucket, ptrNameResource) < 0)
    {
        /* Error: Duplicate name detected. Named resources should be unique for the bucket */
        *errCode = NAME_EDUP;

        /* Cleanup the allocated memory block also. */
        ptrDatabaseInstance->cfg.dspCfg.free (Name_MallocMode_GLOBAL, ptrNameResource, sizeof(Name_Resource));
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a resource
 *
 *  @param[in]   databaseHandle
 *      Database handle
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
int32_t Name_findResource
(
    Name_DBHandle               databaseHandle,
    Name_ResourceBucket         bucket,
    const char*                 ptrName,
    Name_ResourceCfg*           ptrResourceCfg,
    int32_t*                    errCode
)
{
    Name_Resource*  ptrNameResource;

    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrResourceCfg == NULL) || (ptrName == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Find the entry in the database. */
    ptrNameResource = Name_databaseFind ((Name_DatabaseInstance*)databaseHandle, bucket, ptrName);
    if (ptrNameResource != NULL)
    {
        /* Entry found: Copy over the configuration */
        memcpy ((void *)ptrResourceCfg, (void *)&ptrNameResource->cfg, sizeof(Name_ResourceCfg));
        return 0;
    }
    *errCode = NAME_ENOTFOUND;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      This is an internal function which is used to find a resource and it also
 *      returns the owner of the resource.
 *
 *  @param[in]   databaseHandle
 *      Database handle
 *  @param[in]   bucket
 *      Bucket to be searched for the name
 *  @param[in]   ptrName
 *      Pointer to the name to be searched
 *  @param[out]   ptrResourceCfg
 *      Pointer to the name resource configuration populated if the name is found
 *  @param[out]   ptrOwnerName
 *      Pointer to the owner name of the resource
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_INTERNAL_DSP_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t Name_findResourceOwner
(
    Name_DBHandle               databaseHandle,
    Name_ResourceBucket         bucket,
    const char*                 ptrName,
    Name_ResourceCfg*           ptrResourceCfg,
    const char*                 ptrOwnerName,
    int32_t*                    errCode
)
{
    Name_Resource*  ptrNameResource;

    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrResourceCfg == NULL) || (ptrName == NULL) || (ptrOwnerName == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Find the entry in the database. */
    ptrNameResource = Name_databaseFind ((Name_DatabaseInstance*)databaseHandle, bucket, ptrName);
    if (ptrNameResource != NULL)
    {
        /* Entry found: Copy over the configuration */
        memcpy ((void *)ptrResourceCfg, (void *)&ptrNameResource->cfg, sizeof(Name_ResourceCfg));

        /* Copy over the owner information. */
        strcpy (ptrOwnerName, ptrNameResource->owner);
        return 0;
    }
    *errCode = NAME_ENOTFOUND;
    return -1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to modify a resource.
 *
 *  @param[in]   databaseHandle
 *      Database Handle
 *  @param[in]   bucket
 *      Bucket in which the resource is to be modified
 *  @param[in]   ptrResourceCfg
 *      Pointer to the name resource configuration
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
int32_t Name_modifyResource
(
    Name_DBHandle           databaseHandle,
    Name_ResourceBucket     bucket,
    Name_ResourceCfg*       ptrResourceCfg,
    int32_t*                errCode
)
{
    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrResourceCfg == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Modify the entry in the database. */
    if (Name_databaseModify ((Name_DatabaseInstance*)databaseHandle, bucket, ptrResourceCfg) < 0)
    {
        *errCode = NAME_ENOTFOUND;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete a resource
 *
 *  @param[in]   databaseHandle
 *      Database Handle
 *  @param[in]   bucket
 *      Bucket in which the resource is to be deleted
 *  @param[in]   ptrName
 *      Name to be deleted
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
int32_t Name_deleteResource
(
    Name_DBHandle           databaseHandle,
    Name_ResourceBucket     bucket,
    const char*             ptrName,
    int32_t*                errCode
)
{
    /* Sanity Check: Validate the arguments */
    if ((databaseHandle == NULL) || (ptrName == NULL))
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the type is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Delete the entry in the database. */
    if (Name_databaseDelete ((Name_DatabaseInstance*)databaseHandle, bucket, ptrName) < 0)
    {
        *errCode = NAME_ENOTFOUND;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to remove entries from a database.
 *      Purging can be done as follows:-
 *      - Remove all entries from a specific bucket type.
 *      - Remove all entries from a specific bucket type belonging
 *        to a specific owner.
 *
 *  @param[in]   databaseHandle
 *      Handle to the database
 *  @param[in]   bucket
 *      Bucket from which the resource are to be purged
 *  @param[in]   ptrOwnerName
 *      Pointer to the owner name; NULL implies remove entries for all owners
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_purgeDatabase
(
    Name_DBHandle       databaseHandle,
    Name_ResourceBucket bucket,
    const char*         ptrOwnerName,
    int32_t*            errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;

    /* Get the pointer to the database instance */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the bucket is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Purge all the entries in the database. */
    if (Name_databasePurge ((Name_DatabaseInstance*)databaseHandle, bucket, ptrOwnerName) < 0)
    {
        *errCode = NAME_EINTERNAL;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the database handle given an identifier.
 *
 *  @param[in]   databaseId
 *      Database Identifier
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the database
 *  @retval
 *      Error   -   NULL
 */
Name_DBHandle Name_getDatabaseHandle(int32_t databaseId)
{
    int32_t index;

    /* Cycle through and find a matching entry. */
    for (index = 0; index < NAME_MAX_DATABASE_INSTANCE; index++)
    {
        /* Is this a free slot? */
        if (gDatabaseInstance[index] != NULL)
        {
            /* YES. Register the instance. */
            if (gDatabaseInstance[index]->cfg.instanceId == databaseId)
                return gDatabaseInstance[index];
        }
    }
    return NULL;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the database instance identifier
 *      from the database handle
 *
 *  @param[in]   databaseHandle
 *      Database Handle
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Handle to the database identifier
 */
int32_t Name_getDatabaseInstanceId(Name_DBHandle databaseHandle, int32_t* errCode)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;

    /* Sanity Check: Ensure that a valid database handle is passed */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Return the database instance identifier. */
    return ptrDatabaseInstance->cfg.instanceId;
}

/**
 *  @b Description
 *  @n
 *      The function is used to dump the contents of the database
 *      on the console. This is an expensive function since the critical
 *      section locks are held until the display is complete.
 *
 *  @param[in]   databaseHandle
 *      Handle to the database
 *  @param[in]   bucket
 *      Bucket to be displayed
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   Number of entries in the database
 *  @retval
 *      Error   -   <0
 */
int32_t Name_dumpDatabase
(
    Name_DBHandle       databaseHandle,
    Name_ResourceBucket bucket,
    int32_t*            errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;

    /* Get the pointer to the database instance */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that the bucket is VALID. */
    if (bucket >= Name_ResourceBucket_MAX_ALLOWED)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Display the entries in the database */
    return Name_databaseDisplay (ptrDatabaseInstance, bucket);
}

/**
 *  @b Description
 *  @n
 *      The function is used to create a database. Database can span
 *      multiple cores/process in the same realm. To cross execution realms
 *      (ARM and DSP) the database requires the use of Name clients & proxy
 *
 *  @param[in]   ptrDatabaseCfg
 *      Pointer to the database configuration
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
Name_DBHandle Name_createDatabase
(
    Name_DatabaseCfg*   ptrDatabaseCfg,
    int32_t*            errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;
    void*                   csHandle;
    int32_t                 index;

    /* Sanity Check: Validate the arguments */
    if (ptrDatabaseCfg == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: This is the DSP realm. */
    if (ptrDatabaseCfg->realm != Name_ExecutionRealm_DSP)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Validate the OSAL callout table */
    if ((ptrDatabaseCfg->dspCfg.malloc         == NULL)  || (ptrDatabaseCfg->dspCfg.free         == NULL) ||
        (ptrDatabaseCfg->dspCfg.enterCS        == NULL)  || (ptrDatabaseCfg->dspCfg.exitCS       == NULL) ||
        (ptrDatabaseCfg->dspCfg.beginMemAccess == NULL)  || (ptrDatabaseCfg->dspCfg.endMemAccess == NULL))
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure that a valid owner name is passed */
    if (ptrDatabaseCfg->owner[0] == 0x0)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: Ensure cache alignment requirements are met. */
    if ((sizeof(Name_Resource) % CACHE_L2_LINESIZE) != 0)
    {
        *errCode = NAME_EINTERNAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the memory block configured is aligned on the cache boundary */
    if ((ptrDatabaseCfg->dspCfg.baseNamedResourceAddress % CACHE_L2_LINESIZE) != 0)
    {
        *errCode = NAME_EINTERNAL;
        return NULL;
    }

    /* Sanity Check: Ensure the shared memory block allocated for the database is sufficient. */
    if (ptrDatabaseCfg->dspCfg.sizeNamedResourceMemory <= sizeof(Name_DatabaseInstance))
    {
        *errCode = NAME_EINTERNAL;
        return NULL;
    }

    /* Sanity Check: Ensure that the identifier is unique */
    if (Name_getDatabaseHandle(ptrDatabaseCfg->instanceId) != NULL)
    {
        *errCode = NAME_EDUP;
        return NULL;
    }

    /* Allocate memory for the name server */
    ptrDatabaseInstance = (Name_DatabaseInstance*)ptrDatabaseCfg->dspCfg.malloc (Name_MallocMode_LOCAL, sizeof(Name_DatabaseInstance), 0);
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_ENOMEM;
        return NULL;
    }

    /* Initialize the allocated memory block. */
    memset ((void *)ptrDatabaseInstance, 0, sizeof(Name_DatabaseInstance));

    /* Critical Section Enter: We need to register the named resource instance */
    csHandle = ptrDatabaseCfg->dspCfg.enterCS();

    /* Cycle through and find a free entry to register the named resource instance */
    for (index = 0; index < NAME_MAX_DATABASE_INSTANCE; index++)
    {
        /* Is this a free slot? */
        if (gDatabaseInstance[index] == NULL)
        {
            /* YES. Register the instance. */
            gDatabaseInstance[index] = ptrDatabaseInstance;
            break;
        }
    }

    /* Critical Section Exit: */
    ptrDatabaseCfg->dspCfg.exitCS(csHandle);

    /* Was the registeration complete? */
    if (index == NAME_MAX_DATABASE_INSTANCE)
    {
        /* NO space left to register the instance. */
        *errCode = NAME_ENOSPACE;

        /* Cleanup the allocated memory. */
        ptrDatabaseCfg->dspCfg.free (Name_MallocMode_LOCAL, ptrDatabaseInstance, sizeof(Name_DatabaseInstance));
        return NULL;
    }

    /* Populate the database instance */
    memcpy ((void *)&ptrDatabaseInstance->cfg, (void *)ptrDatabaseCfg, sizeof(Name_DatabaseCfg));
    ptrDatabaseInstance->ptrDatabase = (Name_Database*)ptrDatabaseCfg->dspCfg.baseNamedResourceAddress;

    /* Do we need to initialize the database memory? */
    if (ptrDatabaseCfg->dspCfg.initNamedResourceDatabase == 1)
        memset ((void *)ptrDatabaseInstance->ptrDatabase, 0, ptrDatabaseCfg->dspCfg.sizeNamedResourceMemory);

    /* Database instance has been successfully created */
    return (Name_DBHandle)ptrDatabaseInstance;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the database. Deleting a database without
 *      purging all the resources will lead to a memory leak. Please ensure that
 *      entries are purged before deleting the database handle
 *
 *  @param[in]   databaseHandle
 *      Handle to the database
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Name_deleteDatabase
(
    Name_DBHandle       databaseHandle,
    int32_t*            errCode
)
{
    Name_DatabaseInstance*  ptrDatabaseInstance;
    void*                   csHandle;
    uint32_t                index;

    /* Get the pointer to the database instance */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Critical Section Enter: We need to deregister the named resource instance */
    csHandle = ptrDatabaseInstance->cfg.dspCfg.enterCS();
    for (index = 0; index < NAME_MAX_DATABASE_INSTANCE; index++)
    {
        /* Is this the slot? */
        if (gDatabaseInstance[index] == ptrDatabaseInstance)
        {
            /* YES. Deregister the instance. */
            gDatabaseInstance[index] = NULL;
            break;
        }
    }
    ptrDatabaseInstance->cfg.dspCfg.exitCS(csHandle);

    /* Cleanup the memory for the database instance */
    ptrDatabaseInstance->cfg.dspCfg.free (Name_MallocMode_LOCAL, ptrDatabaseInstance, sizeof(Name_DatabaseInstance));
    return 0;
}

