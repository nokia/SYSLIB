/**
 *   @file  listlib.h
 *
 *   @brief   
 *      Header file for the linked list library
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
 *  \par
 */
#ifndef __JOSH_LIST_LIB_H__
#define __JOSH_LIST_LIB_H__

/**************************************************************************
 * STRUCTURE -  Josh_ListNode
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the 
 *	previous and next element in the list.
 **************************************************************************/
typedef struct Josh_ListNode
{
	void*	p_next;		/* Pointer to the next element in the list. */	
    void*   p_prev;     /* Pointer to the prev element in the list. */
}Josh_ListNode;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void Josh_listAdd (Josh_ListNode **ptr_list, Josh_ListNode *ptr_node);
extern Josh_ListNode* Josh_listRemove (Josh_ListNode **ptr_list);
extern Josh_ListNode* Josh_listGetHead (Josh_ListNode** ptr_list);
extern Josh_ListNode* Josh_listGetNext (Josh_ListNode* ptr_list);
extern int Josh_listRemoveNode (Josh_ListNode** ptr_list, Josh_ListNode* ptr_remove);
extern void Josh_listCat (Josh_ListNode **ptr_dst, Josh_ListNode **ptr_src);

#endif /* __JOSH_LIST_LIB_H__ */




