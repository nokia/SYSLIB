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

#ifndef __MEMLOG_LIST_LIB_H__
#define __MEMLOG_LIST_LIB_H__

/**************************************************************************
 * STRUCTURE -  Memlog_ListNode
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 **************************************************************************/
typedef struct Memlog_ListNode
{
    void*   p_next;	/* Pointer to the next element in the list. */
    void*   p_prev;     /* Pointer to the prev element in the list. */
}Memlog_ListNode;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void Memlog_listAdd (Memlog_ListNode **ptr_list, Memlog_ListNode *ptr_node);
extern Memlog_ListNode* Memlog_listRemove (Memlog_ListNode **ptr_list);
extern Memlog_ListNode* Memlog_listGetHead (Memlog_ListNode** ptr_list);
extern Memlog_ListNode* Memlog_listGetNext (Memlog_ListNode* ptr_list);
extern Memlog_ListNode* Memlog_listGetPrev (Memlog_ListNode* ptr_list);
extern int Memlog_listRemoveNode (Memlog_ListNode** ptr_list, Memlog_ListNode* ptr_remove);
extern void Memlog_listCat (Memlog_ListNode **ptr_dst, Memlog_ListNode **ptr_src);
extern void Memlog_listAddTail (Memlog_ListNode** ptr_list, Memlog_ListNode* ptr_node);

#endif /* __MEMLOG_LIST_LIB_H__ */

