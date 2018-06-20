/**
 *   @file  name_armDatabase.c
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>

/* MCSDK Include files. */
#include <ti/csl/csl_cache.h>

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
 *********************** Internal Definitions *************************
 **********************************************************************/

/**
 * @brief   Identifier which indicates that this is the head of the list.
 */
#define NAME_HEAD_ID                       -2

/**
 * @brief   Identifier which indicates that it is a NULL link
 */
#define NAME_NULL_ID                       -1

/**
 * @brief   Maximum number of named resource entries which can exist.
 */
#define NAME_MAX_ENTRIES                    1024

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
     * @brief  Index to the next element in the list.
     */
	int32_t     p_next;

    /**
     * @brief  Index to the previous element in the list.
     */
    int32_t     p_prev;
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
    Name_DBListNode     links;

    /**
     * @brief  Identifier used to identify the entry.
     */
    int32_t             id;

    /**
     * @brief  Named Resource Configuration.
     */
    Name_ResourceCfg    cfg;

    /**
     * @brief  Named Resource Configuration.
     */
    char                owner[NAME_MAX_CHAR + 1];
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
    Name_DBListNode       hash[NAME_DATABASE_HASH_BUCKET];
}Name_BucketDatabase;

/**
 * @brief
 *  Shared Instance Information
 *
 * @details
 *  This structure describes the instance of the database which is shared
 *  by all the other process.
 */
typedef struct Name_SharedInstanceInfo
{
    /**
     * @brief  Each database is made up of buckets
     */
    Name_BucketDatabase     bucket[Name_ResourceBucket_MAX_ALLOWED];

    /**
     * @brief  Mutex which protects the database.
     */
    sem_t                   semaphore;

    /**
     * @brief  Database for all the named resources which can exist on Linux
     */
    Name_Resource           namedResourceEntries[NAME_MAX_ENTRIES];

    /**
     * @brief  Free list which has all the available named resources which
     * are available to be used.
     */
    Name_DBListNode         nameFreeList;
}Name_SharedInstanceInfo;

/**
 * @brief
 *  Database Instance
 *
 * @details
 *  This structure describes the instance of the database specific to a process.
 */
typedef struct Name_DatabaseInstance
{
    /**
     * @brief  Database configuration: This is specific to the process
     */
    Name_DatabaseCfg            cfg;

    /**
     * @brief  Pointer to the shared instance information accessible across all the processes.
     */
    Name_SharedInstanceInfo*    ptrSharedInstanceInfo;
}Name_DatabaseInstance;

/**********************************************************************
 ************************ Global Declarations *************************
 **********************************************************************/

/**
 * @brief   List of all the registered database instances. This is maintained
 * on a per process basis
 */
Name_DatabaseInstance*  gDatabaseInstance[NAME_MAX_DATABASE_INSTANCE] = { 0 };

/**********************************************************************
 ************************* Database Functions *************************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to acquire a pthread mutex lock to protect
 *      the named Resource shared resources against concurrent
 *      access from multiple tasks.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Name_osalEnterCS(Name_DatabaseInstance* ptrDatabaseInstance)
{
    sem_wait (&ptrDatabaseInstance->ptrSharedInstanceInfo->semaphore);
}

/**
 *  @b Description
 *  @n
 *      The function is used to release a mutex lock since the
 *      shared resource access is complete.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the database instance
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void Name_osalExitCS(Name_DatabaseInstance* ptrDatabaseInstance)
{
    sem_post (&ptrDatabaseInstance->ptrSharedInstanceInfo->semaphore);
}

/**
 *  @b Description
 *  @n
 *      The function gets the named resource entry by the index.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *  @param[in]  index
 *      Index entry
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static Name_Resource* Name_getEntryByIndex
(
    Name_DatabaseInstance* ptrDatabaseInstance,
    int32_t                index
)
{
    Name_Resource*      ptrResource;

    /* Validate: Make sure the index is within range. */
    if ((index < 0) || (index >= NAME_MAX_ENTRIES))
        return NULL;

    /* Get the named resource entry */
    ptrResource = &ptrDatabaseInstance->ptrSharedInstanceInfo->namedResourceEntries[index];

    /* Sanity Check: Make sure WYSIWYG */
    if (ptrResource->id != index)
    {
        printf ("Error: Index is %d but the Named Resource Entry %p has %d\n",
                 index, ptrResource, ptrResource->id);
        return NULL;
    }

    /* Return the named resource entry block. */
    return &ptrDatabaseInstance->ptrSharedInstanceInfo->namedResourceEntries[index];
}

/**
 *  @b Description
 *  @n
 *      The function is called to initialize the list.
 *
 *  @param[in]  ptr_list
 *      List to be initialized.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Name_initDBListNode (Name_DBListNode* ptr_list)
{
    /* Ensure that the next & previous pointers are set correctly.
     * All links in the list are killed. */
    ptr_list->p_next = NAME_NULL_ID;
    ptr_list->p_prev = NAME_NULL_ID;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function checks if the specified list is empty or not?
 *
 *  @param[in]  ptr_list
 *      List to be checked.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      0   -   List is not empty
 *  @retval
 *      1   -   List is empty
 */
static int32_t Name_isDBListEmpty (Name_DBListNode* ptr_list)
{
    if ((ptr_list->p_next == NAME_NULL_ID) && (ptr_list->p_prev == NAME_NULL_ID))
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called to get the head of the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *  @param[in]  ptr_list
 *      List from where the head element is required.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Head of the list (NULL if empty)
 */
static Name_Resource* Name_listDBGetHead
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_DBListNode*        ptr_list
)
{
    return Name_getEntryByIndex(ptrDatabaseInstance, ptr_list->p_next);
}

/**
 *  @b Description
 *  @n
 *      The function is called to get the next element of the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *  @param[in]  ptr_node
 *      Node from where the next element is needed.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Next Element of the list
 */
static Name_Resource* Name_listDBGetNext
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource*          ptr_node
)
{
    return Name_getEntryByIndex(ptrDatabaseInstance, ptr_node->links.p_next);
}

/**
 *  @b Description
 *  @n
 *      The function is called to add a node to the list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *  @param[in]  ptr_list
 *      This is the list to which the node is to be added.
 *  @param[in]  ptr_node
 *      This is the node which is to be added.
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Name_listDBAdd
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_DBListNode*        ptr_list,
    Name_Resource*          ptr_node
)
{
    Name_Resource*      ptr_head;

    /* Check if the list is empty? */
    if (Name_isDBListEmpty(ptr_list) == 1)
    {
        /* YES. List is empty. Initialize the list. */
        ptr_list->p_next = ptr_node->id;
        ptr_list->p_prev = NAME_NULL_ID;

        /* Initialize the node (0 indicates that the node points to the head) */
        ptr_node->links.p_next = NAME_NULL_ID;
        ptr_node->links.p_prev = NAME_HEAD_ID;
    }
    else
    {
    	/* No the list was NOT empty. Add the node to the beginning of list.
         * Get the current head of the list. */
        ptr_head = Name_getEntryByIndex(ptrDatabaseInstance, ptr_list->p_next);
        if (ptr_head == NULL)
        {
            printf ("Error: In listAdd head identifier is invalid (%d)\n", ptr_list->p_next);
            return;
        }

        /* Initialize the new head of the list */
        ptr_node->links.p_next = ptr_head->id;
        ptr_node->links.p_prev = NAME_HEAD_ID;

        /* Update the old head to point to the new head */
        ptr_head->links.p_prev = ptr_node->id;

        /* The list head now points to the new node. */
        ptr_list->p_next = ptr_node->id;
        ptr_list->p_prev = NAME_NULL_ID;
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
    Name_DBListNode*          ptr_list
)
{
	Name_Resource*	ptr_head;
	Name_Resource*	ptr_node;

    /* Check if the list is empty? */
    if (Name_isDBListEmpty(ptr_list) == 1)
		return NULL;

    /* Get the current head of the list. */
    ptr_head = Name_getEntryByIndex(ptrDatabaseInstance, ptr_list->p_next);
    if (ptr_head == NULL)
    {
        printf ("Error: In listRemove head identifier is invalid (%d)\n", ptr_list->p_next);
        return NULL;
    }

    /* Is this the last element in the list? */
    if (ptr_head->links.p_next == NAME_NULL_ID)
    {
        /* YES. Initialize the list to be empty. */
        Name_initDBListNode(ptr_list);
    }
    else
    {
        /* No. Get the pointer to the new head of the list */
        ptr_node = Name_getEntryByIndex(ptrDatabaseInstance, ptr_head->links.p_next);
        if (ptr_node == NULL)
        {
            printf ("Error: In listRemove node identifier is invalid (%d)\n", ptr_head->links.p_next);
            return NULL;
        }

        /* Setup the new head in the list appropriately. */
        ptr_list->p_next = ptr_node->id;
        ptr_list->p_prev = NAME_NULL_ID;

        /* The new head now points back to the head of the list */
        ptr_node->links.p_prev = NAME_HEAD_ID;
    }

    /* Kill the links in the current entry being removed. */
    Name_initDBListNode(&ptr_head->links);
    return ptr_head;
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
    Name_DBListNode*        ptr_list,
    Name_Resource*          ptr_remove
)
{
	Name_Resource*	ptr_next;
	Name_Resource*	ptr_prev;

    /* Check if the list is empty? */
    if (Name_isDBListEmpty(ptr_list) == 1)
		return -1;

    /* Are we removing the head? */
    if (ptr_list->p_next == ptr_remove->id)
    {
        /* Use the other API to acheive the needful. */
        Name_listDBRemove(ptrDatabaseInstance, ptr_list);
        return 0;
    }

    /* OK; we are trying to remove a non head element; so lets get the
     * previous and next pointer of the elements that needs to be removed. */
    ptr_prev = Name_getEntryByIndex(ptrDatabaseInstance, ptr_remove->links.p_prev);
    ptr_next = Name_getEntryByIndex(ptrDatabaseInstance, ptr_remove->links.p_next);

    /* The Previous pointer should always be valid; since we have already verified that
     * this is not the head of the list; so there should always be a valid previous link */
    if (ptr_prev == NULL)
    {
        printf ("Error: In listRemoveNode previous id is invalid (%d)\n", ptr_remove->links.p_prev);
        return -1;
    }

    /* Kill the Links for element that is being removed. */
    Name_initDBListNode(&ptr_remove->links);

    /* Are we removing the last element */
    if (ptr_next == NULL)
    {
        /* Yes. Kill the next link to nothing. */
        ptr_prev->links.p_next = NAME_NULL_ID;
        return 0;
    }

    /* We are trying to remove an element in the middle of the list. */
    ptr_prev->links.p_next = ptr_next->id;
    ptr_next->links.p_prev = ptr_prev->id;
	return 0;
}

/**
 *  @b Description
 *  @n
 *      The function allocates a named resource entry block from the free list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Success -   Pointer to the allocated memory
 *  @retval
 *      Error   -   NULL
 */
static Name_Resource* Name_osalAllocResource(Name_DatabaseInstance* ptrDatabaseInstance)
{
    Name_Resource*   ptrResource;

    /* Critical Section Enter: */
    Name_osalEnterCS(ptrDatabaseInstance);

    /* Remove the named resource entry from the free list */
    ptrResource = (Name_Resource*)Name_listDBRemove(ptrDatabaseInstance, &ptrDatabaseInstance->ptrSharedInstanceInfo->nameFreeList);

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);

    /* Named Resource Allocated block. */
    return ptrResource;
}

/**
 *  @b Description
 *  @n
 *      The function allocates a named resource entry block from the free list.
 *
 *  @param[in]  ptrDatabaseInstance
 *      Pointer to the named resource instance
 *  @param[in]  ptrResource
 *      Pointer to the resource being freed up
 *
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
 *
 *  @retval
 *      Not Applicable
 */
static void Name_osalFreeResource
(
    Name_DatabaseInstance*  ptrDatabaseInstance,
    Name_Resource*          ptrResource
)
{
    /* Place this entry back into the free list. */
    Name_listDBAdd (ptrDatabaseInstance, &ptrDatabaseInstance->ptrSharedInstanceInfo->nameFreeList, ptrResource);
    return;
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
    Name_DBListNode*        ptrHashBucket,
    const char*             name
)
{
    Name_Resource*  ptrResource;

    /* Get the head element of the Hash Bucket. */
    ptrResource = Name_listDBGetHead(ptrDatabaseInstance, ptrHashBucket);

    /* Search the hash bucket */
    while (ptrResource != NULL)
    {
        /* Compare the name. */
        if (strncmp(name, ptrResource->cfg.name, NAME_MAX_CHAR) == 0)
            break;

        /* No match. Get the next entry in the list */
        ptrResource = Name_listDBGetNext(ptrDatabaseInstance, ptrResource);
    }
    /* Return the entry. */
    return ptrResource;
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
    Name_Resource*            ptrNameResource;
    uint32_t                  hashCode;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)ptrName, strlen(ptrName), 0);

    /* Critical Section Enter: */
    Name_osalEnterCS (ptrDatabaseInstance);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Find the name in the corresponding hash bucket. */
    ptrNameResource = Name_find(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrName);

    /* Critical Section End */
    Name_osalExitCS (ptrDatabaseInstance);
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
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)&ptrResourceCfg->name[0], strlen(ptrResourceCfg->name), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Critical Section Enter: */
    Name_osalEnterCS (ptrDatabaseInstance);

    /* Find the matching resource entry. */
    ptrResource = Name_find(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrResourceCfg->name);
    if (ptrResource != NULL)
    {
        /* Match found: Overwrite the configuration */
        memcpy ((void *)&ptrResource->cfg, (void*)ptrResourceCfg, sizeof(Name_ResourceCfg));
    }
    else
    {
        /* Error: Name was not found. */
        success = -1;
    }

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);
    return success;
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
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)&ptrNameResource->cfg.name[0], strlen(ptrNameResource->cfg.name), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Critical Section Enter: */
    Name_osalEnterCS (ptrDatabaseInstance);

    /* Check for duplicate names in the has bucket? */
    if (Name_find(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrNameResource->cfg.name) == NULL)
    {
        /* Add the entry to the hash bucket in the database. */
        Name_listDBAdd (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrNameResource);
    }
    else
    {
        /* Error: Duplicate name detected. */
        success = -1;
    }

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);
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
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    int32_t                   success = 0;

    /* Compute the hash code using the name.*/
    hashCode = hash((uint8_t*)ptrName, strlen(ptrName), 0);

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Critical Section Enter: */
    Name_osalEnterCS (ptrDatabaseInstance);

    /* Find the matching resource entry. */
    ptrResource = Name_find(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrName);
    if (ptrResource != NULL)
    {
        /* Match found: Remove from the hash list */
        Name_listDBRemoveNode (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrResource);

        /* Cleanup the memory block. */
        Name_osalFreeResource (ptrDatabaseInstance, ptrResource);
    }
    else
    {
        /* Error: Name was not found. */
        success = -1;
    }

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);
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
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Critical Section Enter: */
    Name_osalEnterCS(ptrDatabaseInstance);

    /* Cycle through all the hash in the bucket */
    for (hashCode = 0; hashCode < NAME_DATABASE_HASH_BUCKET; hashCode++)
    {
        /* Get the resource from the hash bucket. */
        ptrResource = Name_listDBGetHead(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode]);

        /* Cycle through all the elements in the hash bucket. */
        while (ptrResource != NULL)
        {
            /* Do we need to purge the entry? */
            if ((ptrOwnerName != NULL) && (strcmp (ptrOwnerName, ptrResource->owner) != 0))
            {
                /* NO. Owner name was specified and it does not match the resource owner */
                ptrResource = Name_listDBGetNext(ptrDatabaseInstance, ptrResource);
                continue;
            }

            /* YES. Either the name was not specified or it was a match; remove the node from the
             * hash list. */
            Name_listDBRemoveNode (ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode], ptrResource);

            /* Cleanup the memory for the resource */
            Name_osalFreeResource (ptrDatabaseInstance, ptrResource);

            /* Restart from the head again. */
            ptrResource = Name_listDBGetHead(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode]);
        }
    }

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);
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
    Name_Resource*            ptrResource;
    uint32_t                  hashCode;
    uint32_t                  numEntries = 0;

    /* Get the pointer to the bucket database */
    ptrBucketDatabase = &ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket];

    /* Critical Section Enter: */
    Name_osalEnterCS(ptrDatabaseInstance);

    /* Cycle through all the entries in the hash bucket. */
    for (hashCode = 0; hashCode < NAME_DATABASE_HASH_BUCKET; hashCode++)
    {
        /* Get the resource from the hash bucket. */
        ptrResource = Name_listDBGetHead(ptrDatabaseInstance, &ptrBucketDatabase->hash[hashCode]);

        /* Cycle through all the elements in the hash bucket. */
        while (ptrResource != NULL)
        {
            /* Increment the number of entries in the table. */
            numEntries = numEntries + 1;

            /* Display the Named resource configuration. */
            printf ("NR%d: Name: '%s' Owner: '%s'\n", numEntries, ptrResource->cfg.name, ptrResource->owner);
            printf ("      Handle1: 0x%08x Handle2: 0x%08x Handle3: 0x%08x Handle4: 0x%08x \n",
                    ptrResource->cfg.handle1, ptrResource->cfg.handle2, ptrResource->cfg.handle3, ptrResource->cfg.handle4);
            printf ("      Handle5: 0x%08x Handle6: 0x%08x Handle7: 0x%08x Handle8: 0x%08x \n",
                    ptrResource->cfg.handle5, ptrResource->cfg.handle6, ptrResource->cfg.handle7, ptrResource->cfg.handle8);

            /* Get the next entry. */
            ptrResource = Name_listDBGetNext(ptrDatabaseInstance, ptrResource);
        }
    }

    /* Critical Section End */
    Name_osalExitCS(ptrDatabaseInstance);

    /* Return the number of detected entries. */
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
        for (hashCode = 0; hashCode < NAME_DATABASE_HAS_BUCKET; hashCode++)
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
    ptrNameResource = (Name_Resource*)Name_osalAllocResource(ptrDatabaseInstance);
    if (ptrNameResource == NULL)
    {
        /* Error: Out of memory error. */
        *errCode = NAME_ENOMEM;
        return -1;
    }

    /* Populate the named resource configuration. */
    memcpy ((void *)&ptrNameResource->cfg, (void *)ptrResourceCfg, sizeof(Name_ResourceCfg));

    /* Tag the resource name: */
    strncpy (ptrNameResource->owner, ptrDatabaseInstance->cfg.owner, NAME_MAX_CHAR);

    /* Add the Named resource to the TYPE database. */
    if (Name_databaseAdd (ptrDatabaseInstance, bucket, ptrNameResource) < 0)
    {
        /* Error: Duplicate name detected. Named resources should be unique for the bucket */
        *errCode = NAME_EDUP;

        /* Cleanup the memory block. */
        Name_osalFreeResource (ptrDatabaseInstance, ptrNameResource);
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
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
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

    /* Get the database handle. */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;

    /* Allocate memory for the named resource block. */
    ptrNameResource = (Name_Resource*)Name_osalAllocResource(ptrDatabaseInstance);
    if (ptrNameResource == NULL)
    {
        /* Error: Out of memory error. */
        *errCode = RESMGR_ENOMEM;
        return -1;
    }

    /* Populate the named resource configuration. */
    memcpy ((void *)&ptrNameResource->cfg, (void *)ptrResourceCfg, sizeof(Name_ResourceCfg));

    /* Tag the resource name: Use the specified owner name. */
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
 *      Database Handle
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
 *  \ingroup NAME_INTERNAL_ARM_FUNCTION
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
    char*                       ptrOwnerName,
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
 *      Handle to the database
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
 *
 *  \ingroup NAME_FUNCTION
 *
 *  @retval
 *      Success -   0
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
    int32_t                 namedResourceFd;
    int32_t                 initMemoryFlag;
    uint32_t*               addr;
    uint32_t                index;
    char                    namedResourceFile[PATH_MAX];
    Name_Resource*          ptrResource;
    Name_ResourceBucket     bucket;

    /* Sanity Check: Validate the arguments */
    if (ptrDatabaseCfg == NULL)
    {
        *errCode = NAME_EINVAL;
        return NULL;
    }

    /* Sanity Check: This is the ARM realm. */
    if (ptrDatabaseCfg->realm != Name_ExecutionRealm_ARM)
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

    /* Sanity Check: Ensure that the identifier is unique */
    if (Name_getDatabaseHandle(ptrDatabaseCfg->instanceId) != NULL)
    {
        *errCode = NAME_EDUP;
        return NULL;
    }

    /* Create the named resource file name */
    snprintf (namedResourceFile, PATH_MAX, "/syslib_NR-%d", ptrDatabaseCfg->instanceId);

    /* Open the named resource file. */
    namedResourceFd = shm_open (namedResourceFile, O_RDWR, 0777);
    if (namedResourceFd == -1)
    {
        /* Error while opening file; maybe it does not exist so lets
         * try creating one and also set the flag to initialize this. */
        initMemoryFlag = 1;

        /* Create the named resource shared memory section. */
        namedResourceFd = shm_open (namedResourceFile, O_RDWR | O_CREAT, 0777);
        if (namedResourceFd == -1)
        {
            perror ("Error: Unable to create the named resource file; aborting\n");
            *errCode = errno;
            return NULL;
        }
    }
    else
    {
        /* File already opened and created. So we dont need to perform any initializations. */
        initMemoryFlag = 0;
    }

    /* Set the memory object's size */
    if (ftruncate(namedResourceFd, sizeof(Name_SharedInstanceInfo)) == -1)
    {
        perror("Error: ftruncate failed");
        *errCode = errno;
        return NULL;
    }

    /* Map the memory object */
    addr = mmap(0, sizeof(Name_SharedInstanceInfo), PROT_READ | PROT_WRITE, MAP_SHARED, namedResourceFd, 0);
    if (addr == MAP_FAILED)
    {
        perror("Error: mmap failed");
        *errCode = errno;
        return NULL;
    }

    /* Allocate memory for the database instance which is specific to each process. */
    ptrDatabaseInstance = (Name_DatabaseInstance*)malloc (sizeof(Name_DatabaseInstance));
    if (ptrDatabaseInstance == NULL)
    {
        printf ("Error: Out of memory\n");
        return NULL;
    }

    /* Initialize the allocated memory block.  */
    memset ((void *)ptrDatabaseInstance, 0, sizeof(Name_DatabaseInstance));

    /* Populate the database instance  */
    memcpy ((void* )&ptrDatabaseInstance->cfg, (void *)ptrDatabaseCfg, sizeof(Name_DatabaseCfg));
    ptrDatabaseInstance->ptrSharedInstanceInfo = (Name_SharedInstanceInfo*)addr;

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

    /* Was the registeration complete? */
    if (index == NAME_MAX_DATABASE_INSTANCE)
    {
        /* NO space left to register the instance. */
        *errCode = NAME_ENOSPACE;
        return NULL;
    }

    /* Debug Message: */
    printf ("Debug: Named resource instance [%d] mapped to Database %p shared address %p [Init %d] \n",
             ptrDatabaseCfg->instanceId, ptrDatabaseInstance, ptrDatabaseInstance->ptrSharedInstanceInfo, initMemoryFlag);

    /* Initialize the memory block if required to do so */
    if (initMemoryFlag == 1)
    {
        /* Initialize the shared memory contents. */
        memset ((void *)ptrDatabaseInstance->ptrSharedInstanceInfo, 0, sizeof(Name_SharedInstanceInfo));

        /* Initialize all the lists in the Named Resource Database. */
        Name_initDBListNode(&ptrDatabaseInstance->ptrSharedInstanceInfo->nameFreeList);

        /* Initialize all the databases. */
        for (bucket = Name_ResourceBucket_INTERNAL_SYSLIB; bucket < Name_ResourceBucket_MAX_ALLOWED; bucket++)
        {
            /* For each type initialize the list of all the hash buckets. */
            for (index = 0; index < NAME_DATABASE_HASH_BUCKET; index++)
                Name_initDBListNode(&ptrDatabaseInstance->ptrSharedInstanceInfo->bucket[bucket].hash[index]);
        }

        /* Chop up the named resource entries and add them to the free list. */
        for (index = 0; index < NAME_MAX_ENTRIES; index++)
        {
            /* Get the named resource entry block. */
            ptrResource = &ptrDatabaseInstance->ptrSharedInstanceInfo->namedResourceEntries[index];

            /* Initialize the named resource block too. */
            memset ((void *)ptrResource, 0, sizeof(Name_Resource));

            /* Setup the identifier */
            ptrResource->id = index;
            Name_initDBListNode(&ptrResource->links);

            /* Add this to the free list. */
            Name_listDBAdd (ptrDatabaseInstance, &ptrDatabaseInstance->ptrSharedInstanceInfo->nameFreeList, ptrResource);
        }

        /* Initialize the Named Resource Semaphore. */
        if (sem_init(&ptrDatabaseInstance->ptrSharedInstanceInfo->semaphore, 1, 1) < 0)
        {
            perror("Error: Named Resource Semaphore Initialization Failed\n");
            *errCode = errno;
            return NULL;
        }
    }
    /* Named resource instance handle created successfully. */
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
    uint32_t                index;

    /* Get the pointer to the database instance */
    ptrDatabaseInstance = (Name_DatabaseInstance*)databaseHandle;
    if (ptrDatabaseInstance == NULL)
    {
        *errCode = NAME_EINVAL;
        return -1;
    }

    /* Critical Section Enter: We need to deregister the named resource instance */
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

    /* Shutdown the semaphore */
    sem_destroy (&ptrDatabaseInstance->ptrSharedInstanceInfo->semaphore);
    return 0;
}

