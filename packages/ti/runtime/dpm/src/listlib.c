/** 
 *   @file  listlib.c
 *
 *   @brief
 *      Implementation of a circular doubly linked list.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012, Texas Instruments, Inc.
 *  \par
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#include <stdint.h>
#include <stdio.h>

#include <ti/runtime/dpm/include/listlib.h>

/**************************************************************************
 ***************************** ListLib Functions **************************
 **************************************************************************/

/** 
 *  @b Description
 *  @n  
 *      The function is called to initialize the list.
 *
 *  @param[in]  ptrList
 *      List to be initialized
 *
 *  @retval
 *      Not Applicable
 */
void Dpm_listInit (Dpm_ListObj* ptrList)
{
    /* Initialize the fields in the list object. */
    ptrList->count      = 0;
    ptrList->links.next = &ptrList->links;
    ptrList->links.prev = &ptrList->links;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to check if the list is empty or not
 *
 *  @param[in]  ptrList
 *      List to be checked.
 *
 *  @retval
 *      1	-	List is empty
 *  @retval
 *      0	-	List
 */
int32_t Dpm_isListEmpty (Dpm_ListObj* ptrList)
{
    if (ptrList->count == 0)
        return 1;
    return 0;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to enqueue an element to the end of the list.
 *
 *  @param[in]  ptrList
 *      List from where the element is to be added
 *  @param[in]  ptrListNode
 *      Element to be added to the end of the list
 *
 *  @retval
 *      Not Applicable.
 */
void Dpm_listEnqueue (Dpm_ListObj* ptrList, Dpm_ListNode* ptrListNode)
{
    Dpm_ListNode*   ptrPrev;
    
    /* Store the original previous pointer. */
    ptrPrev = ptrList->links.prev;

    /* The new element is added to the end of the list; so the next points
     * back to the head of the list. The prev points back to the older previous */
    ptrListNode->next = &ptrList->links;
    ptrListNode->prev = ptrPrev;

    /* The previous element of the head of the list points to the new element */
    ptrList->links.prev = ptrListNode;

    /* The original previous also now points to the new element. */
    ptrPrev->next       = ptrListNode;

    /* Increment the counter */
    ptrList->count++;
    return;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to dequeue an element from the head of the list.
 *
 *  @param[in]  ptrList
 *      List from where the element is to be removed
 *
 *  @retval
 *      Head of the list (NULL if the list was empty)
 */
Dpm_ListNode* Dpm_listDequeue (Dpm_ListObj* ptrList)
{
    Dpm_ListNode*   ptrListNode;

    /* Is the list empty? */
    if (Dpm_isListEmpty(ptrList))
        return NULL;

    /* List was not empty. Get the first element in the list. */
    ptrListNode = ptrList->links.next;

    ptrList->links.next     = ptrListNode->next;
    ptrListNode->next->prev = &ptrList->links;

    /* Kill the links in the node being removed */
    ptrListNode->next = NULL;
    ptrListNode->prev = NULL;

    /* Decrement the counter */
    ptrList->count--;

    /* Return the element. */
    return ptrListNode;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to get the head of the list but it is not removed
 *
 *  @param[in]  ptrList
 *      List for which the head is to be retreived
 *
 *  @retval
 *      Head of the list (NULL if the list was empty)
 */
Dpm_ListNode* Dpm_listGetHead (Dpm_ListObj* ptrList)
{
    /* Is the list empty? */
    if (Dpm_isListEmpty(ptrList))
        return NULL;

    /* List was not empty. Get the first element in the list. */
    return ptrList->links.next;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to get the next element in the list.
 *
 *  @param[in]  ptrList
 *      List for which the elements are required.
 *  @param[in]  ptrElement
 *      The element after which the next element is required
 *
 *  @retval
 *      Next element in the list (or NULL if the ptrElement was the last)
 */
Dpm_ListNode* Dpm_listGetNext (Dpm_ListObj* ptrList, Dpm_ListNode* ptrElement)
{
    if (ptrElement->next == &ptrList->links)
        return NULL;
    return ptrElement->next;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to get the number of elements in the list.
 *
 *  @param[in]  ptrList
 *      List for which the number of elements are required.
 *
 *  @retval
 *      Counter
 */
uint32_t Dpm_listGetCount (Dpm_ListObj* ptrList)
{
    return ptrList->count;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to remove the specific element from 
 *      the list.
 *
 *  @param[in]  ptrList
 *      List from which the element is to be removed.
 *  @param[in]  ptrElement
 *      The element to be removed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t Dpm_listRemoveElement (Dpm_ListObj* ptrList, Dpm_ListNode* ptrElement)
{
    Dpm_ListNode*   ptrListNodeNext;
    Dpm_ListNode*   ptrListNodePrev;

    /* Is the list empty? */
    if (Dpm_isListEmpty(ptrList))
        return -1;

    /* Get the next & previous element */
    ptrListNodeNext = ptrElement->next;
    ptrListNodePrev = ptrElement->prev;

    /* Link the 2 elements together */
    ptrListNodeNext->prev = ptrListNodePrev;
    ptrListNodePrev->next = ptrListNodeNext;

    /* Kill the links in the removed node. */
    ptrElement->next = NULL;
    ptrElement->prev = NULL;

    /* Decrement the counter */
    ptrList->count--;

    /* Element has been removed. */
    return 0;    
}

