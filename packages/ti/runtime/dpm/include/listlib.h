/**
 *   @file  listlib.h
 *
 *   @brief   
 *      Header file for a circular double linked list 
 *      implementation.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
 *  \par
 */
#ifndef __DPM_LIST_LIB_H__
#define __DPM_LIST_LIB_H__

/**
 * @brief 
 *  Dpm ListNode
 *
 * @details
 *  The structure describes a list node which has links to the previous
 *  and next element in the list.
 */
typedef struct Dpm_ListNode
{
	struct Dpm_ListNode*	prev;
	struct Dpm_ListNode*	next;
}Dpm_ListNode;

/**
 * @brief 
 *  DPM List Object
 *
 * @details
 *  The structure describes the list object 
 */
typedef struct Dpm_ListObj
{
    uint32_t        count;
    Dpm_ListNode    links;
}Dpm_ListObj;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void Dpm_listInit (Dpm_ListObj* ptrList);
extern int32_t Dpm_isListEmpty (Dpm_ListObj* ptrList);
extern void Dpm_listEnqueue (Dpm_ListObj* ptrList, Dpm_ListNode* ptrListNode);
extern Dpm_ListNode* Dpm_listDequeue (Dpm_ListObj* ptrList);
extern int32_t Dpm_isListEmpty (Dpm_ListObj* ptrList);
extern Dpm_ListNode* Dpm_listGetHead (Dpm_ListObj* ptrList);
extern Dpm_ListNode* Dpm_listGetNext (Dpm_ListObj* ptrList, Dpm_ListNode* ptrElement);
extern uint32_t Dpm_listGetCount (Dpm_ListObj* ptrList);
extern int32_t Dpm_listRemoveElement (Dpm_ListObj* ptrList, Dpm_ListNode* ptrElement);

#endif /* __DPM_LIST_LIB_H__ */

