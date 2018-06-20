/**
 *   @file  listlib.h
 *
 *   @brief
 *      Header file for the linked list library
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

#ifndef __NETFP_LIST_LIB_H__
#define __NETFP_LIST_LIB_H__

/**************************************************************************
 * STRUCTURE -  Netfp_ListNode
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 **************************************************************************/
typedef struct Netfp_ListNode
{
	void*	p_next;		/* Pointer to the next element in the list. */
    void*   p_prev;     /* Pointer to the prev element in the list. */
}Netfp_ListNode;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void Netfp_listAdd (Netfp_ListNode **ptr_list, Netfp_ListNode *ptr_node);
extern void Netfp_listAddTail (Netfp_ListNode **ptr_list, Netfp_ListNode *ptr_node);
extern Netfp_ListNode* Netfp_listRemove (Netfp_ListNode **ptr_list);
extern Netfp_ListNode* Netfp_listGetHead (Netfp_ListNode** ptr_list);
extern Netfp_ListNode* Netfp_listGetNext (Netfp_ListNode* ptr_list);
extern Netfp_ListNode* Netfp_listGetPrev (Netfp_ListNode* ptr_list);
extern int Netfp_listRemoveNode (Netfp_ListNode** ptr_list, Netfp_ListNode* ptr_remove);
extern void Netfp_listCat (Netfp_ListNode **ptr_dst, Netfp_ListNode **ptr_src);

#endif /* __NETFP_LIST_LIB_H__ */




