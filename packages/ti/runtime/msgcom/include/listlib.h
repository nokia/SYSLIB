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
#ifndef __MSGCOM_LIST_LIB_H__
#define __MSGCOM_LIST_LIB_H__

/** @addtogroup MSG_COM_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief 
 *  MSGCOM ListNode
 *
 * @details
 *  The structure describes a list node which has links to the previous
 *  and next element in the list.
 */
typedef struct Msgcom_ListNode
{
	struct Msgcom_ListNode*	prev;
	struct Msgcom_ListNode*	next;
}Msgcom_ListNode;

/**
 * @brief 
 *  MSGCOM List Object
 *
 * @details
 *  The structure describes the list object 
 */
typedef struct Msgcom_ListObj
{
    uint32_t           count;
    Msgcom_ListNode    links;
}Msgcom_ListObj;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void Msgcom_listInit (Msgcom_ListObj* ptrList);
extern int32_t Msgcom_isListEmpty (Msgcom_ListObj* ptrList);
extern void Msgcom_listEnqueue (Msgcom_ListObj* ptrList, Msgcom_ListNode* ptrListNode);
extern Msgcom_ListNode* Msgcom_listDequeue (Msgcom_ListObj* ptrList);
extern int32_t Msgcom_isListEmpty (Msgcom_ListObj* ptrList);
extern Msgcom_ListNode* Msgcom_listGetHead (Msgcom_ListObj* ptrList);
extern Msgcom_ListNode* Msgcom_listGetNext (Msgcom_ListObj* ptrList, Msgcom_ListNode* ptrElement);
extern uint32_t Msgcom_listGetCount (Msgcom_ListObj* ptrList);
extern int32_t Msgcom_listRemoveElement (Msgcom_ListObj* ptrList, Msgcom_ListNode* ptrElement);

#endif /* __MSGCOM_LIST_LIB_H__ */

