/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/
#ifndef __LIST_LIB_H__
#define __LIST_LIB_H__

/**************************************************************************
 * STRUCTURE -  List_Node
 **************************************************************************
 *  The structure defines a LIST NODE structure that contains links to the
 *  previous and next element in the list.
 **************************************************************************/
typedef struct List_Node
{
    void*   p_next;     /* Pointer to the next element in the list. */
    void*   p_prev;     /* Pointer to the prev element in the list. */
}List_Node;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void List_addNode (List_Node **ptr_list, List_Node *ptr_node);
extern void List_addNodeTail (List_Node **ptr_list, List_Node *ptr_node);
extern List_Node* List_removeHead (List_Node **ptr_list);
extern List_Node* List_getHead (List_Node** ptr_list);
extern List_Node* List_getNext (List_Node* ptr_list);
extern int List_removeNode (List_Node** ptr_list, List_Node* ptr_remove);
extern void List_catNode (List_Node **ptr_dst, List_Node **ptr_src);

#endif /* __LIST_LIB_H__ */




