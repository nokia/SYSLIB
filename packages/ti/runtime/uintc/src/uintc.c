/**
 *   @file  uintc.c
 *
 *   @brief
 *      The file implements the user space interrupt management library
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>

/* SYSLIB include files. */
#include <ti/runtime/uintc/uintc.h>
#include <ti/runtime/uintc/include/listlib.h>

/**************************************************************************
 ********************* UINTC Internal Definitions *************************
 **************************************************************************/

#ifdef __ARMv7
#include <ti/apps/netfp_config/include/NetFP_System_printf.h> //fzm
#else
#include <xdc/runtime/System.h>
#endif

/**
 * @brief
 *  This is the MAX number of ARM events which can be handled by the UINTC
 *  module. This definition is derived to account for the following:
 *      - ARM GIC Events
 *          This is the maximum number of GIC events which are platform
 *          specific
 *      - FD_SETSIZE
 *          Since the UIO uses file descriptors this is the MAX number of
 *          file descriptors which are supported. This is an OS limit.
 */
#define UINTC_MAX_ARM_EVENT                         1024

/**************************************************************************
 ********************* UINTC Internal Data Structures *********************
 **************************************************************************/

/**
 * @brief
 *  UINTC Event Block
 *
 * @details
 *  Each system event which is registered in the UINTC module is associated
 *  with this informational block.
 */
typedef struct Uintc_EventBlock
{
    /**
     * @brief  Links to the other event blocks.
     */
    Uintc_ListNode      links;

    /**
     * @brief  Event identifier
     */
    uint32_t            eventId;

    /**
     * @brief  Application registered ISR handler.
     */
    UintcIsrHandler     isrHandler;

    /**
     * @brief  File descriptor associated with the event
     */
    int32_t             fd;

    /**
     * @brief  Application supplied argument.
     */
    void*               argument;

    /**
     * @brief  Counter which tracks the number of interrupts reported by the UIO driver.
     */
    uint32_t            numInterrupts;
}Uintc_EventBlock;

// fzm ->
typedef struct Uintc_AdditionalEventBlock
{
    Uintc_ListNode links;
    int32_t        fd;
} Uintc_AdditionalEventBlock;
// <- fzm

/**
 * @brief
 *  UINTC Message Id Node
 *
 * @details
 *  The structure tracks messages nodes which have been registered through custom message identifiers
 */
typedef struct Uintc_MessageNode
{
    /**
     * @brief  Links to the other event blocks.
     */
    Uintc_ListNode      links;

    /**
     * @brief  Message identifier
     */
    uint32_t            messageId;

    /**
     * @brief  Application registered message callback handler.
     */
    UintcIsrHandler     isrHandler;

    /**
     * @brief  Application supplied argument.
     */
    void*               argument;
}Uintc_MessageNode;

/**
 * @brief
 *  UINTC Module master control block
 *
 * @details
 *  The UINTC module configuration is used to specify the properties
 */
typedef struct Uintc_MCB
{
    /**
     * @brief  Configuration block.
     */
    UintcConfig             cfg;

    /**
     * @brief  UNIX socket name used for the management channel.
     */
    char                    uintcSocketName[NAME_MAX];

    /**
     * @brief  File descriptor set which comprises all the registered event blocks
     */
    fd_set                  fdSet;

    /**
     * @brief  Semaphore which is used for event database protection.
     */
    pthread_mutex_t          mutex;

    /**
     * @brief  Maximum file descriptor in the file descriptor set.
     */
    int32_t                 maxFdSet;

    /**
     * @brief  Management socket: This is used to handle dynamic changes to the
     * file descriptor set.
     */
    int32_t                 mgmtSocket;

    /**
     * @brief  Hash bucket which maps event identifier to event block.
     */
    Uintc_EventBlock*       mapEventToEventBlock[UINTC_MAX_ARM_EVENT];

    /**
     * @brief  Registered event database
     */
    Uintc_EventBlock*       ptrEventDatabase;

    /**
     * @brief  Registered message node database
     */
    Uintc_MessageNode*      ptrMessageNodeDatabase;

    /**
     * @brief  Additional fd's uintc_select should wake on (fzm)
     */
    Uintc_AdditionalEventBlock* ptrAdditionalEventDatabase;
}Uintc_MCB;

/**************************************************************************
 ************************** UINTC Functions *******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the registered function which handles the shutdown
 *      of the UINTC module.
 *
 *  @param[in]  arg
 *      Registered argmument
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Uintc_deadHandler (void* arg)
{
    /* Get the UINTC MCB */
    Uintc_MCB* ptrUintcMCB = (Uintc_MCB*)arg;

    /* UINTC module has been shutdown: Complete the rest of the UINTC cleanup here
     * Kill the management socket */
    unlink (ptrUintcMCB->uintcSocketName);
    close (ptrUintcMCB->mgmtSocket);

    /* Release the mutex and close it  */
    pthread_mutex_unlock (&ptrUintcMCB->mutex);
    pthread_mutex_destroy(&ptrUintcMCB->mutex);

    /* Cleanup memory for the UINTC module */
    free (ptrUintcMCB);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered function which handles the updation
 *      of the event list in the UINTC module.
 *
 *  @param[in]  arg
 *      Registered argmument
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Uintc_updateHandler (void* arg)
{
    (void)arg;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function constructs the file descriptor set from a list of all
 *      the registered events in the UINTC module.
 *
 *  @param[in]  ptrUintcMCB
 *      Pointer to the UINTC MCB
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Uintc_constructFDSet (Uintc_MCB* ptrUintcMCB)
{
    Uintc_EventBlock*   ptrEventBlock;

    /* Clear the file descriptor set */
    FD_ZERO(&ptrUintcMCB->fdSet);

    /* Always set the management socket */
    FD_SET(ptrUintcMCB->mgmtSocket, &ptrUintcMCB->fdSet);

    /* Set the maximum file descriptor */
    ptrUintcMCB->maxFdSet = ptrUintcMCB->mgmtSocket;

    /* Acquire the mutex: */
    pthread_mutex_lock (&ptrUintcMCB->mutex);

    /* Get the head of the event block. */
    ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase);
    while (ptrEventBlock != NULL)
    {
        /* Set the event block file descriptor in the set. */
        FD_SET(ptrEventBlock->fd, &ptrUintcMCB->fdSet);

        /* Keep track of the maximum file descriptor in the set. */
        if (ptrEventBlock->fd > ptrUintcMCB->maxFdSet)
            ptrUintcMCB->maxFdSet = ptrEventBlock->fd;

        /* Get the next event block. */
        ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetNext((Uintc_ListNode*)ptrEventBlock);
    }

    // fzm ->
    Uintc_AdditionalEventBlock* ptrAddEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrAdditionalEventDatabase);
    while (ptrAddEventBlock != NULL)
    {
        FD_SET(ptrAddEventBlock->fd, &ptrUintcMCB->fdSet);

        /* Keep track of the maximum file descriptor in the set. */
        if (ptrAddEventBlock->fd > ptrUintcMCB->maxFdSet)
            ptrUintcMCB->maxFdSet = ptrAddEventBlock->fd;

        ptrAddEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetNext((Uintc_ListNode*)ptrAddEventBlock);
    }

    pthread_mutex_unlock (&ptrUintcMCB->mutex);
    // <- fzm

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the UIO name which is present in the
 *      directory name.
 *
 *  @param[in]  entry
 *      Directory entry which corresponds to the UIO class name
 *  @param[in]  uioNameLen
 *      Maximum length of the UIO name
 *  @param[out] uioName
 *      UIO name associated with the UIO class name
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Uintc_getUioName
(
    struct dirent*  entry,
    int32_t         uioNameLen,
    char*           uioName
)
{
    char        filename[PATH_MAX];
    FILE*       fp;

    /* UIO names are present in the UIO class directory under a file called "name" */
    snprintf(filename, PATH_MAX, "/sys/class/uio/%s/name", entry->d_name);

    /* Open the file in read mode. */
	fp = fopen(filename, "r");
	if (fp == NULL)
        return -1;

    /* Read the contents of the file.  */
	fgets(uioName, uioNameLen, fp);

    /* UIO name has been read successfully. */
    fclose(fp);
	return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the UIO class name given the UIO name.
 *      UIO Names are defined as "event<X>" where X is the system event
 *      identifer. UIO class names are defined as "uio<Y>".
 *
 *      UIO class names is a subdirectory present in the /sys/class/uio. Each
 *      subdirectory has a "name" folder which contains the UIO name
 *
 *  @param[in]  uioName
 *      UIO name
 *  @param[out] uioClassName
 *      UIO class name
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      No match found  -   0
 *  @retval
 *      Match found     -   1
 *  @retval
 *      Error           -   <0
 */
static int32_t Uintc_getClassName
(
    char*   uioName,
    char*   uioClassName
)
{
	struct dirent*  entry;
	char            name[PATH_MAX];
	DIR*            dir;

    /* Open the directory where all the UIO class names should be located */
	dir = opendir("/sys/class/uio");
	if (!dir)
        return -1;

    /* Loop around and process all the entries in the directory. */
	while (1)
    {
        /* Read the entry in the directory */
        entry = readdir(dir);
        if (entry == NULL)
            break;

        /* All UIO class names have the "uio" prefix. If not skip the entry and continue */
		if (strstr (entry->d_name, "uio") == NULL)
			continue;

        /* Get the UIO name associated with the directory entry. */
        if (Uintc_getUioName (entry, PATH_MAX, &name[0]) < 0)
        {
            /* FATAL Error: This implies that the UIO name retreival failed. */
            closedir(dir);
            return -1;
        }

        /* Do we have a match? */
        if (strncmp (uioName, name, strlen (uioName)) == 0)
        {
            /* YES. Match found. Copy the UIO class name. We are done. */
            strcpy (uioClassName, entry->d_name);
            break;
        }
	}

    /* Close the directory. */
    closedir(dir);

    /* Did we get a match? */
    if (entry == NULL)
        return 0;
    return 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send a specific message to the UINTC module.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC module
 *  @param[in]  messageId
 *      Message Identifier to be sent
 *  @param[out] errCode
 *      Error code populated on an error
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_sendMessageId
(
    UintcHandle     uintcHandle,
    uint32_t        messageId,
    int32_t*        errCode
)
{
    struct sockaddr_un  socketAddress;
    Uintc_MCB*          ptrUintcMCB;

    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Initialize the socket information. */
    memset(&socketAddress, 0, sizeof(struct sockaddr_un));

    /* Map the client identifer to the unix socket address */
    socketAddress.sun_family = AF_UNIX;
    snprintf (socketAddress.sun_path, sizeof(socketAddress.sun_path), "%s/UINTC-%s", Syslib_getRunTimeDirectory(), ptrUintcMCB->cfg.name);

    /* Send the packet over the management socket. */
    if (sendto((int32_t)ptrUintcMCB->mgmtSocket, &messageId, sizeof(messageId), 0,
               (struct sockaddr *)&socketAddress, sizeof(socketAddress)) < 0)
    {
        *errCode = errno;
        System_printf("Error: Unable to send back the management update [%s]\n", strerror(errno));
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send updates to the event blocks via the
 *      management channel.
 *
 *  @param[in]  ptrUintcMCB
 *      Pointer to the UINTC MCB
 *  @param[out] messageId
 *      Pointer to the message id
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void Uintc_rcvMessageId (Uintc_MCB* ptrUintcMCB, uint32_t* messageId)
{
    /* Initialize the received management buffer. */
    *messageId = 0;

    /* Read the management update */
    if (recv((int32_t)ptrUintcMCB->mgmtSocket, messageId, sizeof(uint32_t), 0) < 0)
    {
        /* Error: Receive failed */
        System_printf("Error: Unable to receive message from the UINTC management channel [%s]\n", strerror(errno));
        return;
    }

    /* The management channel messages are just dummy messages which simply allow the
     * UINTC thread "select" file descriptor list to get updated dynamically. */
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to find a message identifier mapping in the UINTC module
 *
 *  @param[in]  ptrUintcMCB
 *      Pointer to the UINTC MCB
 *  @param[in] messageId
 *      Message identifer for which the mapping is being looked at
 *
 *  \ingroup UINTC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Pointer to the message node
 */
static Uintc_MessageNode* Uintc_findMessageIdMapping
(
    Uintc_MCB*          ptrUintcMCB,
    uint32_t            messageId
)
{
    Uintc_MessageNode*  ptrMessageNode;

    /* Cycle through all the event blocks and dispatch the application interrupt handler */
    ptrMessageNode = (Uintc_MessageNode*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrMessageNodeDatabase);
    while (ptrMessageNode != NULL)
    {
        /* Do we have a match? */
        if (ptrMessageNode->messageId == messageId)
            break;

        /* Get the next element: */
        ptrMessageNode = (Uintc_MessageNode*)Uintc_listGetNext ((Uintc_ListNode*)ptrMessageNode);
    }
    return ptrMessageNode;
}

/**
 *  @b Description
 *  @n
 *      The function allows an application to register a call back function which is
 *      invoked when a specific 'message id' is passed to the UINTC module.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  messageId
 *      Message identifier to be registered
 *  @param[in]  isrHandler
 *      ISR Handler which is registered with the UINTC module. This is invoked when
 *      the specific message is sent to the UINTC module.
 *  @param[in]  arg
 *      Argument to be passed to the ISR handler
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  @sa Uintc_sendMessageId
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_registerMessageId
(
    UintcHandle         uintcHandle,
    uint32_t            messageId,
    UintcIsrHandler     isrHandler,
    void*               arg,
    int32_t*            errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_MessageNode*  ptrMessageNode;

    /* Get the UINTC MCB Block. */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: This function is applicable only for UINTC managed mode. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_APPLICATION_MANAGED)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Ensure that a valid ISR Handler has been registered */
    if (isrHandler == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Duplicate mapping check */
    pthread_mutex_lock (&ptrUintcMCB->mutex);
    ptrMessageNode = Uintc_findMessageIdMapping (ptrUintcMCB, messageId);
    pthread_mutex_unlock (&ptrUintcMCB->mutex);
    if (ptrMessageNode != NULL)
    {
        *errCode = UINTC_EDUP;
        return -1;
    }

    /* Allocate memory for the message node: */
    ptrMessageNode = (Uintc_MessageNode*)malloc (sizeof(Uintc_MessageNode));
    if (ptrMessageNode == NULL)
    {
        *errCode = UINTC_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrMessageNode, 0, sizeof(Uintc_MessageNode));

    /* Populate the message node: */
    ptrMessageNode->messageId  = messageId;
    ptrMessageNode->isrHandler = isrHandler;
    ptrMessageNode->argument   = arg;

    /* Acquire the mutex: */
    pthread_mutex_lock (&ptrUintcMCB->mutex);

    /* Add the message node to the list: */
    Uintc_listAdd ((Uintc_ListNode**)&ptrUintcMCB->ptrMessageNodeDatabase, (Uintc_ListNode*)ptrMessageNode);

    /* Release the mutex: */
    pthread_mutex_unlock (&ptrUintcMCB->mutex);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function allows an application to deregister a call back function which had
 *      been registered with the specific message identifier.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  messageId
 *      Message identifier to be deregistered
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_deregisterMessageId
(
    UintcHandle         uintcHandle,
    uint32_t            messageId,
    int32_t*            errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_MessageNode*  ptrMessageNode;

    /* Get the UINTC MCB Block. */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: This function is applicable only for UINTC managed mode. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_APPLICATION_MANAGED)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Duplicate mapping check */
    pthread_mutex_lock (&ptrUintcMCB->mutex);
    ptrMessageNode = Uintc_findMessageIdMapping (ptrUintcMCB, messageId);
//fzm    pthread_mutex_unlock (&ptrUintcMCB->mutex);
    if (ptrMessageNode == NULL)
    {
        pthread_mutex_unlock (&ptrUintcMCB->mutex); //fzm
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Acquire the mutex: */
//fzm    pthread_mutex_lock (&ptrUintcMCB->mutex);

    /* Remove the message node from the list: */
    Uintc_listRemoveNode ((Uintc_ListNode**)&ptrUintcMCB->ptrMessageNodeDatabase, (Uintc_ListNode*)ptrMessageNode);

    /* Release the mutex: */
    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    /* Clenaup the memory */
    free (ptrMessageNode);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function blocks and waits for an interrupt or specific message id to to arrive
 *      Once this is detected the function will dispatch the registered handler.
 *
 *      The function is application only if the UINTC instance is configured
 *      to operate in the UINTC managed mode. For application managed modes
 *      applications will wait for the interrupts and perform the select in the
 *      application context.
 *
 *  @sa Uintc_registerIsr
 *  @sa Uintc_registerMessageId
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  ptrTimeout
 *      Optional argument for the timeout after which the select call is unblocked.
 *      This can be set to NULL if timeouts are not required
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0  [Timeout has been detected]
 *  @retval
 *      Success -   >0 [Number of active UINTC descriptors handled]
 *  @retval
 *      Error   -   <0 [errCode is set to UINTC_EDEINIT if the module has been deinitialized]
 */
int32_t Uintc_select
(
    UintcHandle     uintcHandle,
    struct timeval* ptrTimeout,
    int32_t*        errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_EventBlock*   ptrEventBlock;
    uint32_t            messageId;
    int32_t             numFd;
    Uintc_MessageNode*  ptrMessageNode;

    /* Get the UINTC MCB */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* This function is applicable only for UINTC managed mode. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_APPLICATION_MANAGED)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Loop around and process all the UINTC managed descriptors. */
    while (1)
    {
        /* Construct the new file descriptor set. */
        Uintc_constructFDSet(ptrUintcMCB);

        /* Wait for the interrupts to get fired. */
        numFd = select(ptrUintcMCB->maxFdSet + 1, &ptrUintcMCB->fdSet, NULL, NULL, ptrTimeout);
    	if (numFd == -1)
        {
            /* Error: Select failed. Determine the error on why this happened? */
            if (errno == EBADF)
            {
                /* Bad file descriptor actually implies that one of the descriptors
                 * in the file set has been closed or is no longer valid. This can happen
                 * because once an ISR is deregistered; the file descriptor is closed in
                 * that context. Ignore this error, reconstruct the file descriptor list
                 * and try again too see if there are other events. */
                continue;
            }

            /* All other errors are FATAL. */
            System_printf ("Error: UINTC select failed [%s]\n", strerror(errno));
        	*errCode = errno;
            return -1;
        }

        /* Is there a timeout? */
        if (numFd == 0)
            return 0;

        /* Do we have a management update? */
        if (FD_ISSET(ptrUintcMCB->mgmtSocket, &ptrUintcMCB->fdSet))
        {
            /* Management update detected. This causes the select to unblock. */
            Uintc_rcvMessageId (ptrUintcMCB, &messageId);

            /* Acquire the mutex: */
            pthread_mutex_lock (&ptrUintcMCB->mutex);

            /* Find the registered message mapping: */
            ptrMessageNode = Uintc_findMessageIdMapping (ptrUintcMCB, messageId);
            if (ptrMessageNode == NULL)
            {
                /* Warning: Received a message identifier which was not registered. Dropping the message */
                pthread_mutex_unlock (&ptrUintcMCB->mutex);
                continue;
            }

            /* Invoke the registered callback function: */
            ptrMessageNode->isrHandler (ptrMessageNode->argument);

            /* Special Case Handling: Is the UINTC module dying? If so kill the UINTC thread */
            if (messageId == 0xdead)
            {
                /* UINTC Module has been deinitialized. The UINTC memory cleanup has already been done
                 * in the UINTC dead message handler. */
                *errCode = UINTC_EDEINIT;
                return -1;
            }

            /* Release the mutex: */
            pthread_mutex_unlock (&ptrUintcMCB->mutex);
            continue;
        }

        /* Acquire the mutex: */
        pthread_mutex_lock (&ptrUintcMCB->mutex);

        /* Cycle through all the event blocks and dispatch the application interrupt handler */
        ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase);
        while (ptrEventBlock != NULL)
        {
            /* Is the event pending? Dispatch the application registered ISR handler. */
            if (FD_ISSET(ptrEventBlock->fd, &ptrUintcMCB->fdSet))
            {
                /* YES. Read from the device: This is required to clean up the pending events
                 * from the UIO driver. This is required else the file descriptor will always
                 * be ready with data. */
                if (read(ptrEventBlock->fd, &ptrEventBlock->numInterrupts, sizeof(uint32_t)) != sizeof(uint32_t))
                {
                    /* FATAL Error: UIO read should never fail. */
                    System_printf("Internal Error: UINTC UIO read failed [%s]\n", strerror(errno));
                    *errCode = UINTC_EINTERR;
                    pthread_mutex_unlock (&ptrUintcMCB->mutex);
                    return -1;
                }

                /* Pass control to the application registered ISR handler. */
                ptrEventBlock->isrHandler (ptrEventBlock->argument);
            }

            /* Get the next event block. */
            ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetNext((Uintc_ListNode*)ptrEventBlock);
        }

        /* Release the mutex: */
        pthread_mutex_unlock (&ptrUintcMCB->mutex);
        return numFd;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to register an interrupt handler for a specific system event
 *      with the UINTC module. The functions returns the mapped UIO file descriptor on success
 *      which can be used to wait for event. Waiting for events can be done by the application
 *      or by the UINTC module itself.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  sysEvent
 *      ARM event identifier
 *  @param[in]  isrHandler
 *      ISR Handler which is registered with the UINTC module.
 *  @param[in]  arg
 *      Argument to be passed to the ISR handler
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 * @ sa Uintc_select
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   File descriptor mapped to the specific ISR
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_registerIsr
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    UintcIsrHandler     isrHandler,
    void*               arg,
    int32_t*            errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_EventBlock*   ptrEventBlock;
    int32_t             retVal;
    char                uioName[NAME_MAX];
    char                uioClassName[NAME_MAX];
    char                uioDeviceName[PATH_MAX];

    /* Get the UINTC MCB Block. */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the range. */
    if (sysEvent > UINTC_MAX_ARM_EVENT)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Is there an event already registered? */
    if (ptrUintcMCB->mapEventToEventBlock[sysEvent] != NULL)
    {
        /* Error: Duplicate mapping. */
        *errCode = UINTC_EDUP;
        return -1;
    }

    /* Allocate memory for the UINTC Event Block. */
    ptrEventBlock = malloc (sizeof(Uintc_EventBlock));
    if (ptrEventBlock == NULL)
    {
        *errCode = UINTC_ENOMEM;
        return -1;
    }

    /* Initialize the allocated memory block. */
    memset ((void*)ptrEventBlock, 0, sizeof(Uintc_EventBlock));

    /* Populate the event block. */
    ptrEventBlock->isrHandler   = isrHandler;
    ptrEventBlock->eventId      = sysEvent;
    ptrEventBlock->argument     = arg;

    /* The DTS file has a convention which is used here to derive the UIO name from the
     * event identifier */
    snprintf (uioName, NAME_MAX, "uintc%d", sysEvent);

    /* Get the UIO class name */
    retVal = Uintc_getClassName (uioName, &uioClassName[0]);
    if (retVal < 0)
    {
        /* FATAL Error: This implies that the UIO mapping has been modified. */
        *errCode = UINTC_EINTERR;
        free (ptrEventBlock);
        return -1;
    }
    if (retVal == 0)
    {
        /* Error: The system event identifier has not been mapped by UIO. There seems to be an
         * issue with the DTS configuration. */
        *errCode = UINTC_EDTSCFG;
        free (ptrEventBlock);
        return -1;
    }

    /* UIO class name has been successfully retreived. Use this to get the UIO device name */
    snprintf(uioDeviceName, PATH_MAX, "/dev/%s", uioClassName);

    /* Open the UIO device. This is used for getting the interrupts. */
    ptrEventBlock->fd = open (uioDeviceName, (O_RDWR | O_SYNC));
	if (ptrEventBlock->fd < 0)
    {
        /* FATAL Error: This implies that the UIO device mapping has been modified */
		*errCode = UINTC_EINTERR;
        free (ptrEventBlock);
		return -1;
	}

    /* Acquire the mutex: */
    pthread_mutex_lock (&ptrUintcMCB->mutex);

    /* Register the mapping between the event identifier and event block */
    ptrUintcMCB->mapEventToEventBlock[sysEvent] = ptrEventBlock;

    /* Add the event block to the registered database */
    Uintc_listAdd ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase, (Uintc_ListNode*)ptrEventBlock);

    /* Release the mutex: */
    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    /* Send the management update to the UINTC thread so that it can update its file descriptor list.
     * This is done only for the UINTC managed modes. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_UINTC_MANAGED)
        Uintc_sendMessageId(ptrUintcMCB, 0xbeef, errCode);

    /* Enable the event */
    if (Uintc_enableEvent (ptrUintcMCB, sysEvent, errCode) < 0)
        return -1;

    /* Return the mapped file descriptor back to the application. */
    return ptrEventBlock->fd;
}

// fzm ->
/**
 *  @b Description
 *  @n
 *      The function is used to register an additional file descriptor that uintc_select
 *      should wake on.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  fd
 *      File descriptor
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 * @ sa Uintc_select
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   File descriptor
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_registerAdditionalFd
(
    UintcHandle         uintcHandle,
    int32_t             fd,
    int32_t*            errCode
)
{
    Uintc_MCB* ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    int entryFound = 0;

    pthread_mutex_lock (&ptrUintcMCB->mutex);

    Uintc_AdditionalEventBlock* ptrEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrAdditionalEventDatabase);
    while (ptrEventBlock != NULL)
    {
        if(ptrEventBlock->fd == fd)
        {
            entryFound = 1;
            break;
        }

        ptrEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetNext((Uintc_ListNode*)ptrEventBlock);
    }

    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    if(entryFound)
    {
        *errCode = UINTC_EDUP;
        return -1;
    }

    ptrEventBlock = malloc (sizeof(Uintc_AdditionalEventBlock));
    if (ptrEventBlock == NULL)
    {
        *errCode = UINTC_ENOMEM;
        return -1;
    }

    memset (ptrEventBlock, 0, sizeof(*ptrEventBlock));

    ptrEventBlock->fd = fd;

    pthread_mutex_lock (&ptrUintcMCB->mutex);
    Uintc_listAdd ((Uintc_ListNode**)&ptrUintcMCB->ptrAdditionalEventDatabase, (Uintc_ListNode*)ptrEventBlock);
    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    return ptrEventBlock->fd;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deregister an additional file descriptor that uintc_select
 *      wakes up on.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  fd
 *      File descriptor
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_deregisterAdditionalFd
(
    UintcHandle         uintcHandle,
    int32_t             fd,
    int32_t*            errCode
)
{
    Uintc_MCB* ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    int entryFound = 0;

    pthread_mutex_lock (&ptrUintcMCB->mutex);

    Uintc_AdditionalEventBlock* ptrEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrAdditionalEventDatabase);
    while (ptrEventBlock != NULL)
    {
        if(ptrEventBlock->fd == fd)
        {
            entryFound = 1;
            break;
        }

        ptrEventBlock = (Uintc_AdditionalEventBlock*)Uintc_listGetNext((Uintc_ListNode*)ptrEventBlock);
    }

    if(entryFound)
        Uintc_listRemoveNode ((Uintc_ListNode**)&ptrUintcMCB->ptrAdditionalEventDatabase, (Uintc_ListNode*)ptrEventBlock);

    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    if(entryFound)
    {
        free (ptrEventBlock);
        return 0;
    }

    *errCode = UINTC_EINVAL;
    return -1;
}

// <- fzm

/**
 *  @b Description
 *  @n
 *      The function is used to deregister the ISR. This will close the UIO device
 *      and disable the event in the ARM interrupt controller.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  sysEvent
 *      ARM event identifier
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_deregisterIsr
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_EventBlock*   ptrEventBlock;

    /* Get the UINTC MCB Block. */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the range. */
    if (sysEvent > UINTC_MAX_ARM_EVENT)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Get the UINTC Event block. */
    ptrEventBlock = ptrUintcMCB->mapEventToEventBlock[sysEvent];
    if (ptrEventBlock == NULL)
    {
        /* Error: Event is not registered. */
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Disable the system event before we remove it. */
    if (Uintc_disableEvent (ptrUintcMCB, sysEvent, errCode) < 0)
        return -1;

    /* Close the file descriptor. */
    close (ptrEventBlock->fd);

    /* Acquire the mutex: */
    pthread_mutex_lock (&ptrUintcMCB->mutex);

    /* Remove the event block from the list of registered events */
    Uintc_listRemoveNode ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase, (Uintc_ListNode*)ptrEventBlock);

    /* Unmap from the hash bucket also */
    ptrUintcMCB->mapEventToEventBlock[sysEvent] = NULL;

    /* Release the mutex: */
    pthread_mutex_unlock (&ptrUintcMCB->mutex);

    /* Cleanup memory */
    free (ptrEventBlock);

    /* Send the management update to the UINTC thread so that it can update its file descriptor list.
     * This is done only for the UINTC managed modes. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_UINTC_MANAGED)
        Uintc_sendMessageId(ptrUintcMCB, 0xbeef, errCode);

    /* Disable the event */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the specific system event. Applications
 *      should install an interrupt handler before invoking the API. Failure
 *      to do so could cause interrupts to be missed.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  sysEvent
 *      ARM event identifier
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_enableEvent
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
)
{
    Uintc_EventBlock*   ptrEventBlock;
    Uintc_MCB*          ptrUintcMCB;
    uint32_t            eventStatus;

    /* Get the UINTC MCB */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the arguments. */
    if (sysEvent > UINTC_MAX_ARM_EVENT)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Get the event block using the hash bucket */
    ptrEventBlock = ptrUintcMCB->mapEventToEventBlock[sysEvent];
    if (ptrEventBlock == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Enable the interrupt */
    eventStatus = 1;
    if (write(ptrEventBlock->fd, &eventStatus, sizeof(uint32_t)) != sizeof(uint32_t))
    {
        *errCode = UINTC_EINTERR;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to disable the specific system event.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  sysEvent
 *      ARM event identifier
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_disableEvent
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    int32_t*            errCode
)
{
    Uintc_EventBlock*   ptrEventBlock;
    Uintc_MCB*          ptrUintcMCB;
    uint32_t            eventStatus;

    /* Get the UINTC MCB */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the arguments. */
    if (sysEvent > UINTC_MAX_ARM_EVENT)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Get the event block using the hash bucket */
    ptrEventBlock = ptrUintcMCB->mapEventToEventBlock[sysEvent];
    if (ptrEventBlock == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Disable the interrupt */
    eventStatus = 0;
    if (write(ptrEventBlock->fd, &eventStatus, sizeof(uint32_t)) != sizeof(uint32_t))
    {
        *errCode = UINTC_EINTERR;
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the statistics associated with the system event.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[in]  sysEvent
 *      ARM event identifier
 *  @param[out]  numInterrupts
 *      Number of detected interrupts
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_getStats
(
    UintcHandle         uintcHandle,
    uint32_t            sysEvent,
    uint32_t*           numInterrupts,
    int32_t*            errCode
)
{
    Uintc_EventBlock*   ptrEventBlock;
    Uintc_MCB*          ptrUintcMCB;

    /* Get the UINTC MCB */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Sanity Check: Validate the arguments. */
    if (sysEvent > UINTC_MAX_ARM_EVENT)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Get the event block using the hash bucket */
    ptrEventBlock = ptrUintcMCB->mapEventToEventBlock[sysEvent];
    if (ptrEventBlock == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Populate the number of interrupts. */
    *numInterrupts = ptrEventBlock->numInterrupts;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function implements the UINTC module initialization.
 *
 *  @param[in]  ptrUintcConfig
 *      Pointer to the UINTC configuration
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the UINTC
 *  @retval
 *      Error   -   NULL
 */
UintcHandle Uintc_init
(
    UintcConfig*    ptrUintcConfig,
    int32_t*        errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    struct sockaddr_un 	socketAddress;

    /* Allocate memory for the UINTC MCB */
    ptrUintcMCB = malloc (sizeof(Uintc_MCB));
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return NULL;
    }

    /* Initialize the allocated memory block */
    memset ((void *)ptrUintcMCB, 0, sizeof(Uintc_MCB));

    /* Copy over the configuration: */
    memcpy ((void *)&ptrUintcMCB->cfg, (void*)ptrUintcConfig, sizeof(UintcConfig));

    /* Initializations are done for the application managed mode. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_APPLICATION_MANAGED)
        return (UintcHandle)ptrUintcMCB;

    /* UINTC managed mode: Create the management socket. */
    ptrUintcMCB->mgmtSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (ptrUintcMCB->mgmtSocket < 0)
    {
        /* Error: Control Socket creation failed. */
        *errCode = UINTC_EINTERR;
        free (ptrUintcMCB);
        return NULL;
	}

    /* Initialize the socket address */
    memset(&socketAddress, 0, sizeof(struct sockaddr_un));

    /* Map the client identifer to a unix socket address */
    socketAddress.sun_family = AF_UNIX;
    snprintf (socketAddress.sun_path, sizeof(socketAddress.sun_path), "%s/UINTC-%s", Syslib_getRunTimeDirectory(), ptrUintcMCB->cfg.name);

    /* Keep a copy of the socket name. */
    strcpy (ptrUintcMCB->uintcSocketName, socketAddress.sun_path);

    /* Unlink any previous client instances: */
    unlink (socketAddress.sun_path);

	/* Bind the client socket. */
    if (bind(ptrUintcMCB->mgmtSocket, (struct sockaddr *)&socketAddress, SUN_LEN(&socketAddress)) < 0)
    {
		/* Error: Unable to bind the socket. */
        *errCode = UINTC_EINTERR;
        return NULL;
	}

    /* Initialize the mutex */
    if (pthread_mutex_init(&ptrUintcMCB->mutex, NULL) != 0)
    {
        *errCode = UINTC_EINTERR;
        return NULL;
    }

    /* Register the special message identifier: This handles the case when the UINTC module is going down */
    if (Uintc_registerMessageId(ptrUintcMCB, 0xdead, Uintc_deadHandler, (void*)ptrUintcMCB, errCode) < 0)
        return NULL;

    /* Register the special message identifier: This handles the case when the UINTC Event list has been updated */
    if (Uintc_registerMessageId(ptrUintcMCB, 0xbeef, Uintc_updateHandler, (void*)ptrUintcMCB, errCode) < 0)
        return NULL;

    /* Successfully created the UINTC module. */
    return (UintcHandle)ptrUintcMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to shutdown the UINTC module. This will disable and
 *      all events which had been registered with the UINTC module.
 *
 *  @param[in]  uintcHandle
 *      Handle to the UINTC
 *  @param[out] errCode
 *      Error Code populated by the API
 *
 *  \ingroup UINTC_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Uintc_deinit
(
    UintcHandle         uintcHandle,
    int32_t*            errCode
)
{
    Uintc_MCB*          ptrUintcMCB;
    Uintc_EventBlock*   ptrEventBlock;

    /* Get the UINTC module */
    ptrUintcMCB = (Uintc_MCB*)uintcHandle;
    if (ptrUintcMCB == NULL)
    {
        *errCode = UINTC_EINVAL;
        return -1;
    }

    /* Deregister all the event blocks which are still active in the event database */
    ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase);
    while (ptrEventBlock != NULL)
    {
        /* Deregister each ISR which is present */
        if (Uintc_deregisterIsr (ptrUintcMCB, ptrEventBlock->eventId, errCode) < 0)
           return -1;

        /* Get the head of the event block */
        ptrEventBlock = (Uintc_EventBlock*)Uintc_listGetHead ((Uintc_ListNode**)&ptrUintcMCB->ptrEventDatabase);
    }

    /* Send the management updates only for UINTC managed mode. */
    if (ptrUintcMCB->cfg.mode == Uintc_Mode_APPLICATION_MANAGED)
    {
        /* For application managed modes: Cleanup memory for the UINTC module */
        free (ptrUintcMCB);
    }
    else
    {
        /* Send an update to the management socket indicating that the UINTC module is going down
         * This will unblock the uintc_select where the remaining cleanup is done. */
        Uintc_sendMessageId(ptrUintcMCB, 0xdead, errCode);
    }
    return 0;
}

