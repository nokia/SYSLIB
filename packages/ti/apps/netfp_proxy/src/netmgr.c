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

/* Standard library include files */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>

/* NetFP Proxy includes */
#include <ti/apps/netfp_proxy/include/netfp_proxy_pvt.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

#undef  NETMGR_DEBUG
//fzm
static int32_t arping (struct nl_addr* ip_addr, const char* if_name);
int (*netmgr_nl_update_neigh)(const char *if_name, struct sockaddr *dst_addr, int family) = NULL;
extern int32_t NetfpProxy_isProxyOverloaded (void);

/**
 * Watch type
 *
 * Defines if network events are being filtered based on
 * Interface name or IP address.
 */
typedef enum watch_type_t
{
    /** Unspecified watch type */
    NETMGR_WATCH_TYPE_INVALID       =   0,
    /** Watch based on Interface name */
    NETMGR_WATCH_TYPE_INTF_BASED    =   1,
    /** Watch based on IP address */
    NETMGR_WATCH_TYPE_ADDR_BASED    =   2
} watch_type_t;

/**
 * Subscriber info
 *
 * Holds the subscriber's callback function and
 * custom application data (cookie).
 */
typedef struct subsc_info_t
{
    /** Link to other subscriber info objects */
    List_Node               list_n;

    /** Subscriber's event notifier callback function */
    netmgr_cb_fxn_t         cb_fxn;

    /** Data to be passed back to callback function */
    void*                   cb_arg;

    /** Watch entry this subscriber is listening on (parent) */
    void*                   subsc_of;
} subsc_info_t;

/**
 * Network event watch info. Holds all info regarding the
 * network event watch point.
 *
 * Holds the active subscriber list, count registered for any
 * LINK, ADDR, NEIGH, ROUTE event. Each watch info is distinguished
 * either based on the interface it is listening on or the IP
 * address its listening on for any network updates based on the
 * watch type.
 */
typedef struct watch_info_t
{
    /** Link to other watch info objects */
    List_Node               list_n;

    /** Watch type */
    watch_type_t            type;

    /** Watch properties */
    union {
        /** Interface name */
        char                if_name[IFNAMSIZ];

        /** Destination IP address for this neighbor entry */
        struct nl_addr*     ip_addr;
    } u;

    /** Number of subscribers registered for updates on this watch point */
    uint32_t                subsc_ctr;

    /** Subscriber list for this watch point */
    subsc_info_t*           subsc_list;
} watch_info_t;

/**
 * Network Manager Master Control Block
 *
 * Holds the configuration and state information for
 * network manager.
 */
typedef struct netmgr_mcb_t
{
    /** Type of cache this MCB belongs to */
    NetMgr_CacheType        cacheType;

    /** Cache manager instance for NETLINK_ROUTE family */
    struct nl_cache_mngr*   net_cache_mngr;

    /** Cache instances for various NETLINK_ROUTE family sub-modules */
    struct nl_cache*        net_cache;

    /** Address cache. Valid only when the address cache is loaded */
    struct nl_cache*        addr_cache;

    /** Netlink socket to issue GET/SET requests from kernel */
    struct nl_sock*         nl_sock;

    /** Routing rule cache instance */
    struct nl_cache*        rule_cache;

    /** Subscriber info for link cache */
    watch_info_t*           link_watch_list;

    /** Subscriber info for address cache */
    watch_info_t*           addr_watch_list;

    /** Subscriber info for neighbor cache */
    watch_info_t*           neigh_watch_list;

    /** socket used to read interface properties */
    int32_t                 if_sock;
} netmgr_mcb_t;

/* Internal APIs */
static void netmgr_receive_link_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data);
static void netmgr_receive_addr_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data);
static void netmgr_receive_neigh_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data);
static void netmgr_receive_route_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data);

/**
 * Formats LIBNL log messages and redirects them to Proxy
 * for output
 *
 * @arg dump_params LIBNL dump configuration
 * @arg buf         String to be output
 *
 * @return  None
 */
static void netmgr_log (struct nl_dump_params* dump_params, char* buf)
{
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "%s", buf);
    return;
}

/**
 * Given a watch list, watch point type and value, this API
 * checks if there is any existing watch point matching them.
 *
 * @arg watch_list  Watch list to search in
 * @arg type        Watch type (IP address based search/interface name
 *                  based search)
 * @arg watch_val   IP address/interface name to search for
 *
 * This API searches for an entry matching an IP address/interface name
 * specified in the LINK/ADDR/NEIGH/ROUTE watch list specified.
 *
 * On success, returns the watch point info handle otherwise returns
 * NULL.
 *
 * @return  >0 on success,
 *          NULL on error
 */
static watch_info_t* netmgr_find_watch_entry
(
    netmgr_mcb_t*   ptrNetMgrMCB,
    watch_info_t**  watch_list,
    watch_type_t    type,
    void*           watch_val
)
{
    watch_info_t*   w_entry = NULL;

    /* Search through the watch list to see if we have a watch point already
     * for the watch value specified */
    for (w_entry = (watch_info_t*)List_getHead ((List_Node**)watch_list);
         w_entry != NULL;
         w_entry = (watch_info_t*)List_getNext ((List_Node*)w_entry))
    {
        if (type == NETMGR_WATCH_TYPE_INTF_BASED)
        {
            /* Interface name based search */
            if (watch_val == NULL && w_entry->u.if_name[0] == '\0')
            {
                /* No interface name specified. Found an entry that captures
                 * events on all interfaces (no filtering). */
                break;
            }
            //fzm
            else if (watch_val != NULL && !strncmp (w_entry->u.if_name, (char *)watch_val, sizeof(w_entry->u.if_name)))
            {
                /* Interface name specified. Found matching entry for the name given */
                break;
            }
        }
        else if (type == NETMGR_WATCH_TYPE_ADDR_BASED)
        {
            /* IP address based search */
            if ((watch_val == NULL) &&
                (w_entry->u.ip_addr == NULL || nl_addr_iszero ((struct nl_addr*)w_entry->u.ip_addr)))
            {
                /* No IP address specified. Found an entry that captures
                 * events on all addresses (no filtering). */
                break;
            }
            else if (watch_val != NULL && !nl_addr_cmp (w_entry->u.ip_addr, (struct nl_addr*)watch_val))
            {
                /* IP address specified. Found matching entry for the address given */
                break;
            }
        }
        else
        {
            /* Invalid watch type. Return error. */
            w_entry = NULL;
            break;
        }
    }

    /* Return the corresponding watch entry found */
    return w_entry;
}

/**
 * Function to add a watch point to a watch list.
 *
 * @arg watch_list  Global watch list to which this watch point must be
 *                  added to.
 * @arg type        Watch point type (Interface based/IP address based)
 * @arg watch_val   Interface name/IP address based on which the watch (filtering)
 *                  must be done.
 *
 * Called by register APIs to add a new LINK/ADDR/NEIGH/ROUTE watch point
 * based on an interface name/IP address.
 *
 * On success, returns a watch point info handle otherwise returns NULL.
 *
 * @return  >0 on success,
 *          NULL on error
 */
static watch_info_t* netmgr_add_watch_entry
(
    netmgr_mcb_t*   ptrNetMgrMCB,
    watch_info_t**  watch_list,
    watch_type_t    type,
    void*           watch_val
)
{
    watch_info_t*   w_entry;

    /* Validate watch type */
    if (type != NETMGR_WATCH_TYPE_INTF_BASED && type != NETMGR_WATCH_TYPE_ADDR_BASED)
        return NULL;

    /* Allocate memory to hold the network watch point info */
    if ((w_entry = (watch_info_t *)malloc (sizeof (watch_info_t))) == NULL)
    {
        /* OOM. Return error */
        return NULL;
    }

    /* Setup the watch info as per the info passed */
    memset ((void *)w_entry, 0, sizeof (watch_info_t));
    w_entry->type   =   type;
    if (watch_val != NULL)
    {
        /* A valid filter specified */
        if (type == NETMGR_WATCH_TYPE_INTF_BASED)
        {
            strncpy (w_entry->u.if_name, (char *)watch_val, sizeof(w_entry->u.if_name)-1);
        }
        else
        {
            w_entry->u.ip_addr = nl_addr_clone ((struct nl_addr *) watch_val);
            if (w_entry->u.ip_addr == NULL)
            {
                /* Failed to setup watch point for the IP address specified.
                 * Return error. */
                free (w_entry);
                return NULL;
            }
        }
    }

    /* Append this to the existing Watch list */
    List_addNode ((List_Node**)watch_list, (List_Node*)w_entry);

    /* Return the newly created watch point entry */
    return w_entry;
}

/**
 * Function to remove a watch point from Network manager's watch list.
 *
 * @arg watch_list  Watch list from which the watch point must be removed
 * @arg w_entry     Entry to be removed
 *
 * Called from unregister to clean up any watch points on which no
 * active subscribers present.
 */
static void netmgr_remove_watch_entry
(
    netmgr_mcb_t*   ptrNetMgrMCB,
    watch_info_t**  watch_list,
    watch_info_t*   w_entry
)
{
    /* Check if any cloned addresses must be freed */
    if (w_entry->type == NETMGR_WATCH_TYPE_ADDR_BASED)
    {
        if (w_entry->u.ip_addr != NULL)
        {
            nl_addr_put ((struct nl_addr *)w_entry->u.ip_addr);
        }
    }
    /* Remove this entry from the watch list */
    List_removeNode ((List_Node**)watch_list, (List_Node*)w_entry);

    /* Free this entry */
    free (w_entry);

    return;
}

/**
 * Adds a subscriber to a watch point's subscriber list.
 *
 * @arg w_entry     Watch point to subscribe to
 * @arg subsc_info  Subscriber info pointer
 *
 * Called from register APIs to add a subscriber to a watch
 * point's subscriber list.
 */
static void netmgr_add_watch_subsc
(
    netmgr_mcb_t*   ptrNetMgrMCB,
    watch_info_t*   w_entry,
    subsc_info_t*   subsc_info
)
{
    /* Add the subscriber to the watch list and increment the
     * number of subscribers listening on this watch list for updates */
    List_addNode ((List_Node**)&w_entry->subsc_list, (List_Node*)subsc_info);
    w_entry->subsc_ctr++;

    /* Setup the parent info (watch entry pointer) for the subscriber */
    subsc_info->subsc_of = w_entry;

    return;
}

/**
 * Removes a subscriber from a watch point's subscriber list.
 *
 * @arg subsc_info  Subscriber info pointer
 *
 * Called from unregister APIs to remove a subscriber from a watch
 * entry's subscriber list. This API removes the subscriber from
 * the watch point's subscriber list and frees its memory.
 */
static void netmgr_remove_watch_subsc
(
    netmgr_mcb_t*   ptrNetMgrMCB,
    subsc_info_t*   subsc_info
)
{
    watch_info_t*       w_entry = subsc_info->subsc_of;

    /* Remove the subscriber from subscriber list listening on
     * this watch point for updates */
    List_removeNode ((List_Node**)&w_entry->subsc_list, (List_Node*)subsc_info);
    w_entry->subsc_ctr--;

    /* Free the subscriber info memory */
    free (subsc_info);
    return;
}

/**
 * Returns the number of susbscribers on any given watch point.
 *
 * @arg w_entry     Watch entry for which the count must be retrieved
 *
 * @return  >=0 number of subscribers
 */
static int32_t netmgr_get_watch_subsc_count (watch_info_t* w_entry)
{
    return w_entry->subsc_ctr;
}

/**
 * Network manager interface's internal Link event notification handler.
 *
 * @arg cache       Link cache instance
 * @arg obj         Link update received
 * @arg action      Event reported
 * @arg data        Data to the call back function. Not used.
 *
 * Receives all Link notification events from the kernel. Parses all
 * the events, filters the events based on interface received for and
 * calls the necessary callback functions to notify them.
 *
 * @return None
 */
static void netmgr_receive_link_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data)
{
    struct rtnl_link*   nl_link = (struct rtnl_link *)obj;
    watch_info_t*       watch_info;
    subsc_info_t*       subsc_info;
    subsc_info_t*       next_subsc;
    netmgr_mcb_t*       ptrNetMgrMCB = (netmgr_mcb_t*)data;

#ifdef NETMGR_DEBUG
    if (1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: NetMgr Received link update ... action: %s\n",NetfpProxy_action2Str(action));

        static struct nl_dump_params    dump_params;
        dump_params.dp_type = NL_DUMP_LINE;
        dump_params.dp_cb   = netmgr_log;

        nl_object_dump(obj, &dump_params);
    }
#endif

    /* Is it an event that we handle? */
    if ((action != NL_ACT_NEW) && (action != NL_ACT_DEL) && (action != NL_ACT_CHANGE))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Ignoring event: %s\n", NetfpProxy_action2Str(action));
        return;
    }

#if 0
    /* Step 1:  Broadcast this event to callback functions that registered for
     *          all link updates.  */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->link_watch_list,
                                               NETMGR_WATCH_TYPE_INTF_BASED,
                                               NULL)) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_LINK_EVT,
                                 nl_link,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }
#endif

    /* Filter and broadcast this event to callback functions that registered for
     *          just this link's updates. */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->link_watch_list,
                                               NETMGR_WATCH_TYPE_INTF_BASED,
                                               (void*)rtnl_link_get_name (nl_link))) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_LINK_EVT,
                                 nl_link,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }

    return;
}

/**
 * API to retrieve link information based on interface name.
 *
 * @arg if_name     Interface name
 * @arg nl_link     Link info structure
 *
 * Gets the latest link info from the kernel and passes back
 * relevant information back to the caller in link info
 * data structure.
 *
 * @return  NETFP_PROXY_RETVAL_E_INVALID_PARAMS if bad input passed
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          0 on success.
 */
int32_t netmgr_link_get_by_name
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    const char*             if_name,
    struct rtnl_link**      nl_link
)
{
    netmgr_mcb_t*   ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return -1;

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (if_name == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if ((*nl_link = rtnl_link_get_by_name (ptrNetMgrMCB->net_cache,
                                           if_name)) == NULL)
        return NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Return success. */
    return 0;
}

/**
 * Network manager interface's internal Address event notification handler.
 *
 * @arg cache       Address cache instance
 * @arg obj         Address update received
 * @arg action      Event reported
 * @arg data        Data to the call back function. Not used.
 *
 * Receives all Address notification events from the kernel. Parses all
 * the events, filters the events based on IP address received for and
 * calls the necessary callback functions to notify them.
 *
 * @return None
 */
static void netmgr_receive_addr_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data)
{
    struct rtnl_addr*   rtnl_addr = (struct rtnl_addr *)obj;
    char                if_name [IFNAMSIZ];
    watch_info_t*       watch_info;
    subsc_info_t*       subsc_info;
    subsc_info_t*       next_subsc;
    netmgr_mcb_t*       ptrNetMgrMCB = (netmgr_mcb_t*)data;

#ifdef NETMGR_DEBUG
    if (1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: NetMgr Received address update ...action: %s\n", NetfpProxy_action2Str(action));

        static struct nl_dump_params    dump_params;
        dump_params.dp_type = NL_DUMP_LINE;
        dump_params.dp_cb   = netmgr_log;

        nl_object_dump(obj, &dump_params);
    }
#endif

    /* Is it an event that we handle? */
    if ((action != NL_ACT_NEW) && (action != NL_ACT_DEL) && (action != NL_ACT_CHANGE))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Ignoring event: %s\n", NetfpProxy_action2Str(action));
        return;
    }
#if 0
    /* Step 1:  Broadcast this event to callback functions that registered for
     *          all address updates.  */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->addr_watch_list,
                                               NETMGR_WATCH_TYPE_INTF_BASED,
                                               NULL)) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_ADDR_EVT,
                                 rtnl_addr,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }
#endif
    /* Get the interface name corresponding to the address received */
    rtnl_link_i2name (ptrNetMgrMCB->net_cache,
                      rtnl_addr_get_ifindex (rtnl_addr),
                      if_name,
                      IFNAMSIZ);

    /* Filter and broadcast this event to callback functions that registered for
     *          just this interface's address updates. */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->addr_watch_list,
                                               NETMGR_WATCH_TYPE_INTF_BASED,
                                               (void *)if_name)) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_ADDR_EVT,
                                 rtnl_addr,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }

    return;
}

/**
 * API to retrieve IP address information based on interface name.
 *
 * @arg if_name             Interface name
 * @arg ipaddr_list         List of all the IPv4/v6 addresses on
 *                          this interface
 * @arg num_addrs           Number of IP addresses populated and
 *                          returned by this function
 *
 * Gets the list of all the IPv4/v6 addresses setup on the interface
 * specified. Returns it as an array of IP address info data
 * structures to the caller. If if_name set to NULL, this API
 * returns all the IP addresses configured on all the interfaces.
 *
 * @return  NETFP_PROXY_RETVAL_E_INVALID_PARAMS if bad input passed
 *          NETFP_PROXY_RETVAL_E_NO_MEM if memory allocation fails for the IP address
 *          list
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          0 on success.
 */
int32_t netmgr_addr_get
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    const char*             if_name,
    uint32_t**              ipaddr_list,
    uint32_t*               num_addrs
)
{
    struct rtnl_addr*   rtnl_addr;
    struct rtnl_link*   link;
    uint32_t            *ip_addr_list;
    uint32_t            num = 0, i;
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (num_addrs == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if ((ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK) || (ptrNetMgrMCB->addr_cache == NULL))
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Calculate number of IP addresses that are relevant for the application.
     * Accordingly allocate memory for IP address info list. */
    if (if_name == NULL)
    {
        /* Return all IP addresses. Do not filter based on interface. */
        num = nl_cache_nitems (ptrNetMgrMCB->addr_cache);
    }
    else
    {
        /* Find all IP addresses configured on the interface passed. */
        rtnl_addr = (struct rtnl_addr*)nl_cache_get_first (ptrNetMgrMCB->addr_cache);
        while (rtnl_addr != NULL)
        {
            link = rtnl_addr_get_link (rtnl_addr);
            //fzm - Prevent false positives based on partial name matching
            if (!strncmp (rtnl_link_get_name (link), if_name, NETFP_MAX_CHAR))
            {
                /* matching entry found. increment counter */
                num ++;
            }
            rtnl_addr_set_link (rtnl_addr, NULL);

            rtnl_addr = (struct rtnl_addr*)nl_cache_get_next ((struct nl_object*)rtnl_addr);
        }
    }

    /* No address configured on this interface */
    if (num == 0) {
        *num_addrs      =   num;
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }

    /* Allocate memory for the IP address info list */
    if ((ip_addr_list = (uint32_t *)malloc (sizeof (uint32_t) * num)) == NULL)
        return NETFP_PROXY_RETVAL_E_NO_MEM;

    rtnl_addr           =   (struct rtnl_addr *)nl_cache_get_first (ptrNetMgrMCB->addr_cache);
    i = 0;
    while (rtnl_addr != NULL)
    {
        if (if_name == NULL)
        {
            ip_addr_list[i] =   (uint32_t)nl_object_clone ((struct nl_object *)rtnl_addr);
            i ++;
        }
        else
        {
            link = rtnl_addr_get_link (rtnl_addr);
            //fzm - Prevent false positives based on partial name matching
            if (!strncmp (rtnl_link_get_name (link), if_name, NETFP_MAX_CHAR))
            {
                ip_addr_list[i] =   (uint32_t)nl_object_clone ((struct nl_object *)rtnl_addr);
                i++;
            }
            rtnl_addr_set_link (rtnl_addr, NULL);
        }

        rtnl_addr = (struct rtnl_addr*)nl_cache_get_next ((struct nl_object*)rtnl_addr);
    }

    /* Pass back the application the results */
    *num_addrs      =   num;
    *ipaddr_list    =   ip_addr_list;

    /* Return success. */
    return 0;
}

/**
 * Network manager interface's internal Neighbor Table (ARP) update event
 * notification handler.
 *
 * @arg cache       Neighbor cache instance
 * @arg obj         Neighbor update received
 * @arg action      Event reported
 * @arg data        Data to the call back function. Not used.
 *
 * Receives all Neighbor notification events from the kernel. Parses all
 * the events, filters the events based on IP address received for and
 * calls the necessary callback functions to notify them.
 *
 * @return None
 */
static void netmgr_receive_neigh_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data)
{
    struct rtnl_neigh*      nl_neigh = (struct rtnl_neigh *)obj;
    watch_info_t*           watch_info;
    subsc_info_t*           subsc_info;
    subsc_info_t*           next_subsc;
    netmgr_mcb_t*           ptrNetMgrMCB = (netmgr_mcb_t*)data;

#ifdef NETMGR_DEBUG
    if (1)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: NetMgr Received neighbor update ...action: %s\n", NetfpProxy_action2Str(action));

        static struct nl_dump_params    dump_params;
        dump_params.dp_type = NL_DUMP_LINE;
        dump_params.dp_cb   = netmgr_log;

        nl_object_dump(obj, &dump_params);
    }
#endif

    /* Is it an event that we handle? */
    if ((action != NL_ACT_NEW) && (action != NL_ACT_DEL) && (action != NL_ACT_CHANGE))
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Ignoring event: %s\n", NetfpProxy_action2Str(action));
        return;
    }
#if 0
    /* Step 1:  Broadcast this event to callback functions that registered for
     *          all neighbor updates.  */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->neigh_watch_list,
                                               NETMGR_WATCH_TYPE_ADDR_BASED,
                                               NULL)) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_NEIGH_EVT,
                                 nl_neigh,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }
#endif
    /* Filter and broadcast this event to callback functions that registered for
     *          just this link's updates. */
    if ((watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->neigh_watch_list,
                                               NETMGR_WATCH_TYPE_ADDR_BASED,
                                               (void *)rtnl_neigh_get_dst (nl_neigh))) != NULL)
    {
        /* Broadcast the event to all subscribers on this watch list */
        subsc_info = (subsc_info_t*)List_getHead ((List_Node**)&watch_info->subsc_list);
        while (subsc_info != NULL)
        {
            next_subsc = (subsc_info_t*)List_getNext ((List_Node*)subsc_info);
            subsc_info->cb_fxn  (action,
                                 NETMGR_NEIGH_EVT,
                                 nl_neigh,
                                 subsc_info->cb_arg);
            subsc_info = next_subsc;
        }
    }

    return;
}

/**
 * API that checks if a given Neighbor cache entry is valid for
 * application to use.
 *
 * @arg rtnl_neigh          Neighbor cache entry that needs to be checked
 *
 * API that checks if the given Neighbor cache entry is in a valid
 * state that the application can use. The only supported valid
 * states are: NUD_REACHABLE, NUD_PROBE, NUD_STALE, NUD_DELAY, NUD_PERMANENT.
 * A neighbor cache entry that is in any of the above mentioned
 * states is considered valid, since it will have a MAC address
 * associated with it that the application can use to reach it.
 * Static ARP entries are NOT supported at the moment.
 *
 * @return  1 if the entry is valid and is ok to use.
 *          0 if this entry is not in valid state
 */
static int32_t netmgr_neigh_is_valid (struct rtnl_neigh* rtnl_neigh)
{
    /* Must have a valid MAC address and be in one of the valid states */
    if (rtnl_neigh_get_lladdr(rtnl_neigh) &&
        ((rtnl_neigh_get_state (rtnl_neigh) == NETMGR_NUD_RESET) ||
        (rtnl_neigh_get_state (rtnl_neigh) & NETMGR_NUD_VALID)))
        return 1;
    else
        return 0;
}

static struct nl_cache* netmgr_poll_for_neigh
(
    netmgr_mcb_t* ptrNetMgrMCB,
    struct rtnl_neigh* neigh_filter,
    struct nl_addr* ip_addr
)
{
    do
    {
        struct nl_cache* neigh_cache_filtered = NULL;
        if ((neigh_cache_filtered = nl_cache_subset (ptrNetMgrMCB->net_cache,
                                                     (struct nl_object *)neigh_filter)) != NULL)
        {
            if (nl_cache_nitems (neigh_cache_filtered) == 0)
            {
                nl_cache_free(neigh_cache_filtered);
                break;
            }

            struct rtnl_neigh *rtnl_neigh = (struct rtnl_neigh*)nl_cache_get_first(neigh_cache_filtered);
            while (rtnl_neigh != NULL)
            {
                if (netmgr_neigh_is_valid (rtnl_neigh) == 1)
                    return neigh_cache_filtered;

                rtnl_neigh = (struct rtnl_neigh*)nl_cache_get_next((struct nl_object*)rtnl_neigh);
            }

            nl_cache_free(neigh_cache_filtered);
        }
    } while(0);

    char ipBuff[128];
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: MAC for %s not present on neighbor list", nl_addr2str (ip_addr, ipBuff, sizeof(ipBuff)));

    return NULL;
}

/**
 * API to retrieve Neighbor information based on IP address
 * provided.
 *
 * @arg ip_addr             IPv4/v6 address for which ARP entry
 *                          needs to be found.
 * @arg if_name             Interface name to use to resolve ARP
 * @arg neigh_subsc_list    List of all the neighbor entries
 * @arg num_neighs          Number of neighbor entries being returned
 *                          by this function.
 *
 * Gets the neighbor entry for the destination IP address, link name
 * provided to this API. If an IP address is specified, num_neighs
 * always will be 1 and the neigh_subsc_list will have corresponding
 * one neighbor entry information. However, if no IP address, interface
 * name is specified, then all neighbor entries are returned by this API.
 *
 * @return  NETFP_PROXY_RETVAL_E_INVALID_PARAMS if bad input passed
 *          NETFP_PROXY_RETVAL_E_NO_MEM if memory allocation fails for the neighbor
 *          list
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          0 on success.
 */
int32_t netmgr_neigh_get
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    struct nl_addr*         ip_addr,
    const char*             if_name,
    uint32_t**              neigh_subsc_list,
    uint32_t*               num_neighs
)
{
    struct rtnl_neigh   *neigh_filter = NULL, *rtnl_neigh;
    struct nl_cache     *neigh_cache_filtered = NULL;
    uint32_t            *neigh_list;
    uint32_t            num = 0, i = 0;
    struct nl_addr      *addr_filter = NULL;
    char                buf1[128], buf2[128], buf3[128];
    int32_t             error_no;
    netmgr_mcb_t*       ptrNetMgrMCB;
    struct ifreq        ifReq;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (num_neighs == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_NEIGH)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* If a valid IP address specified, an interface MUST be specified too */
    if (ip_addr != NULL && if_name == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Calculate number of neighbor entries that are relevant for the application.
     * Accordingly allocate memory for neighbor info list and populate it. */
    if (ip_addr)
    {
        /* Create a filter to search through neighbor cache */
        if ((neigh_filter = rtnl_neigh_alloc ()) == NULL)
            return NETFP_PROXY_RETVAL_E_OP_FAILED;

        if ((addr_filter = nl_addr_clone (ip_addr)) == NULL)
        {
            rtnl_neigh_put (neigh_filter);
            return NETFP_PROXY_RETVAL_E_OP_FAILED;
        }
        rtnl_neigh_set_family (neigh_filter, nl_addr_get_family (addr_filter));
        rtnl_neigh_set_dst (neigh_filter, addr_filter);

        /* Get the interface index from the interface name. */
        memset (&ifReq, 0, sizeof (struct ifreq));
        strncpy (ifReq.ifr_name, if_name, NETFP_MAX_CHAR);
        if (ioctl(ptrNetMgrMCB->if_sock, SIOCGIFINDEX, &ifReq) == -1)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Cannot get interface index for interface %s\n", if_name);
            error_no    = NETFP_PROXY_RETVAL_E_OP_FAILED;
            goto cleanup_and_return;
        }

        rtnl_neigh_set_ifindex (neigh_filter, ifReq.ifr_ifindex);

        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Resolving %s MAC using interface %s\n",
                           nl_addr2str (ip_addr, buf1, sizeof (buf1)), if_name);
        // fzm->
        /* Do an ARPing to resolve the IP address using link specified. If no entry found even
         * after that. We give up and return error saying no match found. */
        if (arping (addr_filter, if_name) < 0)
        {
            /* clean up and return error */
            error_no    =   NETFP_PROXY_RETVAL_E_OP_FAILED;
            goto cleanup_and_return;
        }

        if ((neigh_cache_filtered = netmgr_poll_for_neigh(ptrNetMgrMCB, neigh_filter, ip_addr)) == NULL)
        {
            error_no = NETFP_PROXY_RETVAL_E_OP_FAILED;
            goto cleanup_and_return;
        }

        /* We are here indicates that filtered neighbor
             * cache has valid data. Indicates a match. */
        num =   nl_cache_nitems (neigh_cache_filtered);

        /* Allocate memory for the neighbor info list */
        if ((neigh_list = (uint32_t *)malloc (sizeof (uint32_t) * num)) == NULL)
        {
            /* clean up and return error */
            error_no    =   NETFP_PROXY_RETVAL_E_NO_MEM;
            goto cleanup_and_return;
        }

        rtnl_neigh      =   (struct rtnl_neigh*)nl_cache_get_first (neigh_cache_filtered);
        i               =   0;
        while (rtnl_neigh != NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: IP: %s MAC: %s NUD State: %s\n",
                               nl_addr2str (rtnl_neigh_get_dst (rtnl_neigh), buf1, sizeof (buf1)),
                               nl_addr2str (rtnl_neigh_get_lladdr (rtnl_neigh), buf2, sizeof (buf2)),
                               rtnl_neigh_state2str (rtnl_neigh_get_state (rtnl_neigh), buf3, sizeof (buf3)));

            /* Use only ARP cache entries that are in valid state. */
            if (netmgr_neigh_is_valid (rtnl_neigh) == 1)
            {
                neigh_list[i]       =   (uint32_t)nl_object_clone ((struct nl_object *)rtnl_neigh);
                i ++;

                /* If we are loading an entry in STALE state, we need to reset the cache
                     * state so that, the event change in kernel is propagated upto the
                     * application again. */
                if (rtnl_neigh_get_state(rtnl_neigh) == NUD_STALE)
                    rtnl_neigh_unset_state (rtnl_neigh, NUD_STALE);
            }
            rtnl_neigh          =   (struct rtnl_neigh*)nl_cache_get_next ((struct nl_object*)rtnl_neigh);
        }

        /* No valid neighbor cache entry found for the IP address specified.
             * Clean up and return error. */
        if (i == 0)
        {
            free (neigh_list);
            error_no    =   NETFP_PROXY_RETVAL_E_OP_FAILED;
            goto cleanup_and_return;
        }
    }
    else
    {
        /* Return all neighbor entries. Do not filter based on destination IP address. */
        num = nl_cache_nitems (ptrNetMgrMCB->net_cache);

        /* Allocate memory for the neighbor info list */
        if ((num == 0) || (neigh_list = (uint32_t *)malloc (sizeof (uint32_t) * num)) == NULL)
            return NETFP_PROXY_RETVAL_E_NO_MEM;

        /* No destination IP address provided. Copy all neighbor entries to
         * neighbor list. */
        rtnl_neigh  =   (struct rtnl_neigh*)nl_cache_get_first (ptrNetMgrMCB->net_cache);
        i           =   0;
        while (rtnl_neigh != NULL)
        {
            neigh_list[i]       =   (uint32_t)nl_object_clone ((struct nl_object *)rtnl_neigh);
            rtnl_neigh          =   (struct rtnl_neigh*)nl_cache_get_next ((struct nl_object*)rtnl_neigh);
            i ++;
        }
    }

    /* Success case. Pass back the application the results */
    *num_neighs         =   i;
    *neigh_subsc_list   =   neigh_list;
    error_no            =   NETFP_PROXY_RETVAL_SUCCESS;

cleanup_and_return:
    if (addr_filter)
        nl_addr_put (addr_filter);
    if (neigh_filter)
        rtnl_neigh_put (neigh_filter);
    if (neigh_cache_filtered)
        nl_cache_free (neigh_cache_filtered);

    /* Return. */
    return error_no;
}

//fzm-->
/**
 * Network manager interface's internal ARP_Ping function.
 *
 * @arg ip_addr             IP address for which ARP needs to be resolved
 * @arg if_name             Interface to use to resolve the ARP
  *
 * Initiates a v4 ARP Ping request/v6 Neighbor discovery request
 * to resolve a given IPv4/IPv6 address to its MAC.
 *
 * @return  NETFP_PROXY_RETVAL_E_OP_FAILED if operation fails
 *          NETFP_PROXY_RETVAL_E_INVALID_PARAMS if bad input passed
 *          0 on success.
 */
static int32_t arping (struct nl_addr* ip_addr, const char* if_name)
{
    if(netmgr_nl_update_neigh != NULL)
    {
        if (nl_addr_get_family (ip_addr) != AF_INET
            && nl_addr_get_family (ip_addr) != AF_INET6)
            return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

        char ipAddrStr[INET6_ADDRSTRLEN];
        nl_addr2str (ip_addr, ipAddrStr, INET6_ADDRSTRLEN);
        union Address
        {
            struct sockaddr_in ipv4;
            struct sockaddr_in6 ipv6;
        } addr;
        socklen_t addrLen = sizeof(addr);
        nl_addr_fill_sockaddr(ip_addr, (struct sockaddr *)&addr, &addrLen);

        netmgr_nl_update_neigh(if_name,
                              (struct sockaddr *)&addr,
                               nl_addr_get_family(ip_addr));

        return NETFP_PROXY_RETVAL_SUCCESS;
    }

    return NETFP_PROXY_RETVAL_E_OP_FAILED;
}
//<--fzm

/**
 * Network manager interface's internal Route event notification handler.
 *
 * @arg cache       Route cache instance
 * @arg obj         Route update received
 * @arg action      Event reported
 * @arg data        Data to the call back function. Not used.
 *
 * Receives all Route notification events from the kernel. Parses all
 * the events, filters the events based on IP address received for and
 * calls the necessary callback functions to notify them.
 *
 * @return None
 */
static void netmgr_receive_route_updates (struct nl_cache* cache, struct nl_object* obj, int action, void* data)
{
    return;
}


/**
 * API to check if the table specified is based off a source address
 * rule
 *
 * @arg table_no        Table number that needs to be checked
 * @arg src_addr        Source address from the rule if the route
 *                      is a source based route
 *
 * Checks if a table is based off a source address rule. If so,
 * returns the source address from the rule
 *
 * @return  1 if the table is derived from source based rule.
 *          0 if the table is not source based.
 */
static int32_t netmgr_table_is_srcbased
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    uint32_t                table_no,
    struct nl_addr**        src_addr
)
{
    struct rtnl_rule        *nl_rule;
    netmgr_mcb_t*           ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Valid memory must be specified to return the address rule */
    if (src_addr == NULL)
        return NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Is the route from one of the Standard Kernel routing tables? */
    if (table_no >= RT_TABLE_COMPAT && table_no <= RT_TABLE_LOCAL)
    {
        /* Yes, the route uses one of the following standard kernel routing rule/tables:
         * RT_TABLE_COMPAT (252),
         * RT_TABLE_DEFAULT (253),
         * RT_TABLE_MAIN (254),
         * RT_TABLE_LOCAL (255),
         * */
        return 0;
    }
    else
    {
        /* No, the route uses a user-defined rule/table. Check the rules to see if
         * this user-defined table is used for source IP based routing. */

        /* Load the first rule from the cache */
        nl_rule      =   (struct rtnl_rule*)nl_cache_get_first (ptrNetMgrMCB->rule_cache);

        /* Iterate through the rule cache to check for any rule that
         * matches the table number specified */
        while (nl_rule != NULL)
        {
            /* Log the rule info */
            //netmgr_log_rule (nl_rule);

            /* Is this the rule that uses the table under question? */
            if (rtnl_rule_get_table (nl_rule) == table_no)
            {
                /* Matching rule found. Is this rule source IP address based? */
                if ((rtnl_rule_get_src (nl_rule) == NULL) ||
                    (nl_addr_get_len (rtnl_rule_get_src (nl_rule)) == 0))
                {
                    /* Not a source IP address based rule */
                    return 0;
                }
                else
                {
                    /* Source IP address based rule. Return src address info */
                    *src_addr = nl_addr_clone (rtnl_rule_get_src (nl_rule));
                    return 1;
                }
            }

            /* Continue traversing the list */
            nl_rule =   (struct rtnl_rule*)nl_cache_get_next ((struct nl_object*)nl_rule);
        }
    }

    /* Error case. No rule found using the table in route.
     * Not enough info to decide whether the route is src based or not. */
    return NETFP_PROXY_RETVAL_E_OP_FAILED;
}

/**
 * API to retrieve list of matching rules/tables for a given
 * destination address family and source address.
 *
 * @arg family              Destination address family.
 * @arg src_addr            Source IPv4/v6 address to use for lookup.
 * @arg table_list          List of all the matching routing tables
 * @arg num_tables          Number of routing tables returned
 *
 * Gets all matching routing tables to use for route lookup based
 * on the family and source address specified.
 *
 * @return  NETFP_PROXY_RETVAL_E_NO_MEM if memory allocation fails for list
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          >0 on success.
 */
static int32_t netmgr_route_get_tables
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    int32_t                 family,
    struct nl_addr*         src_addr,
    uint32_t**              table_list,
    uint32_t*               num_tables
)
{
    struct rtnl_rule        *nl_rule;
    uint32_t                *match_table_list = NULL, num, i = 0;
    netmgr_mcb_t*           ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;

    /* Dump the rule cache contents */
    if (0)
    {
        static struct       nl_dump_params    dump_params;
        dump_params.dp_type = NL_DUMP_LINE;
        dump_params.dp_cb   = netmgr_log;

        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Rule cache dump.. \n");
        nl_cache_dump (ptrNetMgrMCB->rule_cache, &dump_params);
    }

    /* Get total number of rules */
    num =   nl_cache_nitems (ptrNetMgrMCB->rule_cache);

    /* Allocate memory to save all the rule/table entries */
    if ((num == 0) || (match_table_list = (uint32_t *)malloc (sizeof (uint32_t) * num)) == NULL)
    {
        return NETFP_PROXY_RETVAL_E_NO_MEM;
    }

    /* Initialize the number of table matches. */
    i = 0;

    /* Load the first rule from the cache */
    nl_rule      =   (struct rtnl_rule*)nl_cache_get_first (ptrNetMgrMCB->rule_cache);

    /* Iterate through the rule cache to check for any rule that
     * matches the source IP/family specified */
    while (nl_rule != NULL)
    {
#if 0
        /* Log the rule info */
        netmgr_log_rule (nl_rule);
#endif

        if (rtnl_rule_get_family (nl_rule) != family)
        {
            /* Not a match for the destination family specified */
            nl_rule =   (struct rtnl_rule*)nl_cache_get_next ((struct nl_object*)nl_rule);
            continue;
        }

        if ((rtnl_rule_get_src (nl_rule) == NULL) ||
            (nl_addr_get_len (rtnl_rule_get_src (nl_rule)) == 0))
        {
            /* Source == "all". Save this */
        }
        else if (src_addr && nl_addr_cmp_prefix (rtnl_rule_get_src (nl_rule), src_addr) == 0)
        {
            /* Match found for source specified. Save this */
        }
        else
        {
            /* Not a match for the source IP address specified. Continue searching */
            nl_rule =   (struct rtnl_rule*)nl_cache_get_next ((struct nl_object*)nl_rule);
            continue;
        }

        /* Save the table number and increment match counter */
        match_table_list[i]     =   rtnl_rule_get_table (nl_rule);
        i++;

        /* Continue traversing the list */
        nl_rule =   (struct rtnl_rule*)nl_cache_get_next ((struct nl_object*)nl_rule);
    }

    /* Pass back the results */
    *num_tables =   i;
    *table_list =   match_table_list;

    /* Return the number of tables saved */
    return i;
}
/*
 * API to log contents of a route cache entry.
 *
 * @arg route               Route to be logged.
 *
 * Logs the contents of a given routing entry.
 *
 * @return  void.
 */
static void netmgr_log_route (struct rtnl_route* route)
{
    char                    buf[128];

    //fzm change debug to verbose
    NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: dst: %s/%d/%d ",
                       nl_addr2str (rtnl_route_get_dst (route), buf, sizeof(buf)),
                       nl_addr_get_len (rtnl_route_get_dst (route)),
                       nl_addr_get_prefixlen (rtnl_route_get_dst (route)));
    if (rtnl_route_nexthop_n (route, 0) && rtnl_route_nh_get_gateway(rtnl_route_nexthop_n (route, 0)))
        NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "nh: %s/%d/%d/%d ",
                           nl_addr2str (rtnl_route_nh_get_gateway(rtnl_route_nexthop_n (route, 0)), buf, sizeof(buf)),
                           nl_addr_get_len (rtnl_route_nh_get_gateway(rtnl_route_nexthop_n (route, 0))),
                           nl_addr_get_prefixlen (rtnl_route_nh_get_gateway(rtnl_route_nexthop_n (route, 0))),
                           rtnl_route_nh_get_ifindex (rtnl_route_nexthop_n (route, 0)));
    else
        NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "nh: NULL ");
    NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "table: %s\n", rtnl_route_table2str (rtnl_route_get_table (route), buf, sizeof(buf)));

    return;
}

/**
 * API to retrieve the entire routing table
 *
 * @arg table_no            Routing table to retrieve.
 * @arg route_watch_list    List of all the route entries
 * @arg num_routes          Number of routes being returned by this
 *                          function
 *
 * Gets routing table dump and returns it to the caller.
 *
 * @return  NETFP_PROXY_RETVAL_E_NO_MEM if memory allocation fails for the route
 *          list
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          0 on success.
 */
static int32_t netmgr_get_full_routing_table
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    int32_t                 table_no,
    uint32_t**              route_watch_list,
    uint32_t*               num_routes
)
{
    struct rtnl_route       *nl_route_filter = NULL, *rtnl_route;
    uint32_t                *route_list = NULL;
    uint32_t                num = 0, i;
    struct nl_cache*        nl_cache_filtered;
    struct nl_addr*         srcaddr_rule = NULL;
    netmgr_mcb_t*           ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;

    /* Filter out local routes. Use routes from main routing table only */
    if ((nl_route_filter = rtnl_route_alloc ()) == NULL)
        return NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Only get the Gateway/Direct routes. Skip rest */
    rtnl_route_set_type (nl_route_filter, RTN_UNICAST);

    /* Filter based on routing table number if specified */
    if (table_no != RT_TABLE_UNSPEC && table_no != RT_TABLE_MAX)
    {
        rtnl_route_set_table (nl_route_filter, table_no);
    }

    if ((nl_cache_filtered = nl_cache_subset (ptrNetMgrMCB->net_cache,
                                              (struct nl_object *)nl_route_filter)) == NULL)
    {
        rtnl_route_put (nl_route_filter);
        return NETFP_PROXY_RETVAL_E_OP_FAILED;
    }

    /* Get total number of filtered routes */
    num =   nl_cache_nitems (nl_cache_filtered);

    /* Allocate memory to save all the route entries */
    if ((num == 0) || (route_list = (uint32_t *)malloc (sizeof (uint32_t) * num)) == NULL)
    {
        rtnl_route_put (nl_route_filter);
        nl_cache_free (nl_cache_filtered);
        return NETFP_PROXY_RETVAL_E_NO_MEM;
    }

    /* Traverse route list and save info for the application */
    rtnl_route      =   (struct rtnl_route*)nl_cache_get_first (nl_cache_filtered);
    i               =   0;
    while (rtnl_route != NULL)
    {
        /* If no source based rule/table was specified for filtering, skip routes from
         * source based tables for lookups */
        if ((table_no == RT_TABLE_UNSPEC) &&
            (netmgr_table_is_srcbased (netMgrHandle, rtnl_route_get_table (rtnl_route), &srcaddr_rule) == 1))
        {
            if (srcaddr_rule)
                nl_addr_put (srcaddr_rule);
            rtnl_route          =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
            continue;
        }

        /* Log the route info */
        netmgr_log_route (rtnl_route);

        /* Save the route info */
        route_list[i]   =   (uint32_t)nl_object_clone ((struct nl_object *)rtnl_route);

        /* Continue traversing the list */
        rtnl_route      =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
        i ++;
    }
    /* Save the number of routes being returned */
    num = i;

    /* Pass back the application the results */
    *num_routes         =   num;
    *route_watch_list   =   route_list;

    /* Free the cache */
    rtnl_route_put (nl_route_filter);
    nl_cache_free (nl_cache_filtered);

    /* Got entire routing table */
    return 0;
}


/**
 * API to retrieve Route information based on IP address, inerface and
 * routing table provided.
 *
 * @arg ip_addr             Destination IPv4/v6 address for which
 *                          route must be found.
 * @arg table_no            Routing table to use for lookup.
 * @arg route_watch_list    List of all the matching route entries
 * @arg num_neighs          Number of routes being returned by this
 *                          function
 *
 * Gets all matching routes for the destination IP address,
 * interface combination specified from a given routing table. The caller
 * can specify any combination of the ip_addr, table_no parameters,
 * i.e., specify either one of them, or all three of them or set them
 * all to NULL and the API will accordingly filter the routing
 * table entries.
 * If table_no is set to:
 * RT_TABLE_MAX     - Routes from all routing tables are returned
 * RT_TABLE_UNSPEC  - Routes from only non-source based routing tables are returned
 * valid table number-Routes from only the table number specified are returned
 *
 * @return  NETFP_PROXY_RETVAL_E_INVALID_PARAMS if bad input passed
 *          NETFP_PROXY_RETVAL_E_NO_MEM if memory allocation fails for the route
 *          list
 *          NETFP_PROXY_RETVAL_E_OP_FAILED if get operation from kernel fails
 *          0 on success.
 */
static int32_t netmgr_route_get
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    struct nl_addr*         ip_addr,
    int32_t                 table_no,
    uint32_t**              route_watch_list,
    uint32_t*               num_routes
)
{
    struct rtnl_route       *nl_route_filter = NULL, *rtnl_route, *route_match = NULL;
    uint32_t                *route_list = NULL;
    uint32_t                num = 0;
    struct nl_cache         *nl_cache_filtered = NULL;
    int32_t                 error_no;
    struct nl_addr*         srcaddr_rule = NULL;
    netmgr_mcb_t*           ptrNetMgrMCB;
    uint32_t                metric, metric_low;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;

    if (num_routes == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* If no filters specified return the entire cache */
    if (!ip_addr)
    {
        return netmgr_get_full_routing_table (netMgrHandle, table_no, route_watch_list, num_routes);
    }

    /* Filter routes to match the inputs specified */
    if ((nl_route_filter = rtnl_route_alloc ()) == NULL)
        return NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Only get the Gateway/Direct routes. Skip rest */
    rtnl_route_set_type (nl_route_filter, RTN_UNICAST);

    /* Step 1: Get all the routes based on the
     * destination IP address family and interface in routing table specified. */
    if (ip_addr)
    {
        rtnl_route_set_family (nl_route_filter, nl_addr_get_family (ip_addr));
    }

    if (table_no != RT_TABLE_UNSPEC && table_no != RT_TABLE_MAX)
    {
        /* If a valid table number is specified, use it for filtering */
        rtnl_route_set_table (nl_route_filter, table_no);
    }

    /* Step 2: Filter further based on the destination IP address specified. */
    if ((nl_cache_filtered = nl_cache_subset (ptrNetMgrMCB->net_cache, (struct nl_object *)nl_route_filter)))
    {
        if (0)
        {
            static struct       nl_dump_params    dump_params;
            dump_params.dp_type = NL_DUMP_LINE;
            dump_params.dp_cb   = netmgr_log;

            nl_cache_dump (nl_cache_filtered, &dump_params);
        }

        if (ip_addr)
        {
            /* Find the first matching route, that will be the best match since
             * thats how they are ordered in the kernel */
            unsigned int foundRoutePrefixLen = 0;
            rtnl_route      =   (struct rtnl_route*)nl_cache_get_first (nl_cache_filtered);
            while (rtnl_route != NULL)
            {
                /* Log the route info */
                netmgr_log_route (rtnl_route);

                /* Save default routes. Will be used if no better match is found */
                if (rtnl_route_get_dst (rtnl_route) == NULL || nl_addr_get_len(rtnl_route_get_dst (rtnl_route)) == 0)
                {
                    /* Found a default route in the table. Save it. We will use this if we cant
                     * find a better match */
                    if (route_match == NULL)
                    {
                        route_match         =   rtnl_route;
                        metric_low          =   rtnl_route_get_priority (rtnl_route);
                        NetfpProxy_logMsg (NETFP_PROXY_LOG_VRB, "VRB: netmgr_find_route() 1st match metric %d\n", metric_low); //fzm
                    }
                    else
                    {
                        /* Check if this default route is better than the one saved AND saved route is a default route */
                        metric = rtnl_route_get_priority (rtnl_route);
                        if ((metric < metric_low) &&
                            (rtnl_route_get_dst (route_match) == NULL || nl_addr_get_len(rtnl_route_get_dst (route_match)) == 0))
                        {
                            route_match         =  rtnl_route;
                            metric_low          =  metric;
                            NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: netmgr_find_route() low metric %u\n", metric_low);
                        }
                    }

                    /* Get the next route entry */
                    rtnl_route          =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
                    continue;
                }

                /* If no source based rule/table was specified for filtering, skip routes from
                 * source based tables for lookups */
                if ((table_no == RT_TABLE_UNSPEC) &&
                    (netmgr_table_is_srcbased (netMgrHandle, rtnl_route_get_table (rtnl_route), &srcaddr_rule) == 1))
                {
                    if (srcaddr_rule)
                        nl_addr_put (srcaddr_rule);
                    rtnl_route          =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
                    continue;
                }

                struct nl_addr* route_dst = rtnl_route_get_dst(rtnl_route);
                /* Check if the route is applicable for the IP address we are looking up. Since  */
                if (nl_addr_cmp_prefix (route_dst, ip_addr) == 0)
                {
                    metric = rtnl_route_get_priority (rtnl_route);
                    if((foundRoutePrefixLen < nl_addr_get_prefixlen(route_dst)) ||
                       (foundRoutePrefixLen == nl_addr_get_prefixlen(route_dst) && metric < metric_low))
                    {
                        /* Found a match. Save this route for later and continue, maybe there's a better one. */
                        route_match         = rtnl_route;
                        metric_low          = metric;
                        foundRoutePrefixLen = nl_addr_get_prefixlen(route_dst);
                        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: netmgr_find_route() prefix_len: %u metric: %u\n", foundRoutePrefixLen, metric_low);

                        rtnl_route          =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
                        continue;
                    }
                }

                /* No match, continue searching */
                rtnl_route          =   (struct rtnl_route*)nl_cache_get_next ((struct nl_object*)rtnl_route);
            }

            if (route_match == NULL)
            {
                /* No matching direct/default route found. Return error. */
                error_no    =   NETFP_PROXY_RETVAL_E_OP_FAILED;
                goto cleanup_and_return;
            }
            else
            {
                /* Found a match. Allocate memory to save the route */
                if ((route_list = (uint32_t *)malloc (sizeof (uint32_t) * 1)) == NULL)
                {
                    error_no    =   NETFP_PROXY_RETVAL_E_NO_MEM;
                    goto cleanup_and_return;
                }

                /* Found match, copy over route information to user */
                num                 =   1;
                *route_list         =   (uint32_t)nl_object_clone ((struct nl_object *)route_match);
            }
        }
    }
    else
    {
        /* No routing entries found in the filtered cache. */
        error_no    =   NETFP_PROXY_RETVAL_E_OP_FAILED;
        goto cleanup_and_return;
    }

    /* Pass back the application the results */
    *num_routes         =   num;
    *route_watch_list   =   route_list;
    error_no            =   NETFP_PROXY_RETVAL_SUCCESS;

cleanup_and_return:
    if (nl_route_filter)
        rtnl_route_put (nl_route_filter);
    if (nl_cache_filtered)
        nl_cache_free (nl_cache_filtered);

    return error_no;
}

/**
 * API to find the route
 *
 * @arg dest_addr           Destination IPv4/v6 address for which
 *                          next hop info must be found.
 * @arg src_addr            Source IPv4/IPv6 address to filter the route on.
 * @arg nh_addr             Next Hop IP address found
 * @arg if_name             Interface on which the route is found.
 * @arg table_no            Routing table to which this route belongs.
 *
 * Given a destination IPv4/v6 address, this API retrieves the Next
 * hop IP address, MAC address and interface to use to reach it.
 *
 * @return  NETFP_PROXY_RETVAL_E_OP_FAILED if no route found
 *          0 on success.
 */
int32_t netmgr_find_route
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    struct nl_addr*         dest_addr,
    struct nl_addr*         src_addr,
    struct nl_addr**        nh_addr,
    char*                   if_name,
    int32_t*                table_no
)
{
    struct nl_addr          *nl_gwaddr = NULL, *nl_nhaddr;
    struct rtnl_route*      nl_route;
    struct rtnl_nexthop*    nl_nh;
    uint32_t                *routeList, *tableList;
    uint32_t                numEntries = 0, i, numTables, tableNo;
    int32_t                 if_index, error_no = NETFP_PROXY_RETVAL_E_OP_FAILED;
    netmgr_mcb_t*           ptrNetMgrMCB;
    struct ifreq            ifReq;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_ROUTE)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Get a list of routing rules/tables matching the destination address family and
     * source IP address specified. */
    if (netmgr_route_get_tables (netMgrHandle, nl_addr_get_family (dest_addr), src_addr, &tableList, &numTables) <= 0)
        return NETFP_PROXY_RETVAL_E_OP_FAILED;

    /* Check each of the routing tables for a directed/default route match */
    for (tableNo = 0; tableNo < numTables; tableNo++)
    {
        /* Get the gateway corresponding to the destination IP address from the table specified */
        if (netmgr_route_get (netMgrHandle, dest_addr, tableList [tableNo], &routeList, &numEntries) < 0)
        {
            /* No direct/default route in the table specified. Continue searching
             * next table in the list. */
            continue;
        }
        else
        {
            /* Found route. Go ahead and process it */
            nl_route    =   (struct rtnl_route *)routeList[0];
            nl_nh       =   rtnl_route_nexthop_n (nl_route, 0);
            if (nl_nh)
            {
                nl_gwaddr   =   rtnl_route_nh_get_gateway (nl_nh);
                if_index    =   rtnl_route_nh_get_ifindex (nl_nh);

                memset (&ifReq, 0, sizeof (struct ifreq));
                ifReq.ifr_ifindex = if_index;
                if (ioctl(ptrNetMgrMCB->if_sock, SIOCGIFNAME, &ifReq) == -1)
                {
                    NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Cannot get interface name for interface index %d\n", if_index);
                    error_no =  NETFP_PROXY_RETVAL_E_OP_FAILED;
                    goto cleanup_and_return;
                }
                NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Interface index %d Interface Name %s\n", if_index, ifReq.ifr_name);

                strncpy (if_name, ifReq.ifr_name, NETFP_MAX_CHAR);
            }
            else
            {
                nl_gwaddr   =   NULL;

                /* Cleanup and return error */
                error_no    =   NETFP_PROXY_RETVAL_E_OP_FAILED;
                goto cleanup_and_return;
            }

            /* If no gateway present, indicates that the destination is
             * on same subnet and is directly reachable. In this case,
             * use destination as next hop, otherwise gateway is next hop */
            if (nl_gwaddr == NULL || nl_addr_iszero (nl_gwaddr))
            {
                // No gateway. Use destination IP as next hop
                nl_nhaddr   =   nl_addr_clone (dest_addr);
            }
            else
            {
                // Use gateway IP as next hop
                nl_nhaddr   =   nl_addr_clone (nl_gwaddr);
            }

            /* Update application with next hop info found. Return success. */
            *nh_addr    =   nl_nhaddr;
            *table_no   =   rtnl_route_get_table (nl_route);
            error_no    =   NETFP_PROXY_RETVAL_SUCCESS;

            goto cleanup_and_return;
        }
    }

    if(tableNo == numTables)
    {
        error_no = NETFP_PROXY_RETVAL_E_OP_FAILED;
        return error_no;
    }

cleanup_and_return:
    for (i = 0; i < numEntries; i++)
        rtnl_route_put ((struct rtnl_route *)routeList[i]);
    if(routeList)
        free(routeList);
    free(tableList);

    /* Done processing. Return. */
    return error_no;
}

/**
 * Function to refill the route cache, rule cache and routing tables.
 *
 * @arg subsc_hdl   NetMgr Subscriber handle returned by the register
 *                  function.
 *
 * This API is called to refill the caches when the application issues a FLUSH command after
 * changing the routes or rules.
 *
 * @return 0 on success,
 *         < 0 on error if invalid input passed or cache cannot be reloaded.
 */
int32_t netmgr_route_cache_flush
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    int32_t*                errCode
)
{
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
    {
        *errCode = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return -1;
    }

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_ROUTE)
    {
        *errCode = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return -1;
    }

    /* Reload the rule cache from the kernel */
    int res = nl_cache_refill (ptrNetMgrMCB->nl_sock, ptrNetMgrMCB->rule_cache);
    if ( res < 0)
    {
        *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
        return -1;
    }

    /* Reload the route cache from the kernel */
    res = nl_cache_refill (ptrNetMgrMCB->nl_sock, ptrNetMgrMCB->net_cache);
    if (res < 0)
    {
        *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
        return -1;
    }

    /* Refresh the list of all available table names at the moment */
    rtnl_route_read_table_names (NETMGR_ROUTE_TABLE_PATH);

    return 0;
}

/**
 * Function to register for Link related event notifications.
 *
 * @arg if_name     Name of the interface for which link updates
 *                  are to be provided.
 * @arg cb_fxn      User callback function to be invoked on
 *                  an event update.
 * @arg cb_arg      Argument to be passed to the user callback function.
 * @arg error       Error code returned.
 *
 * Must be called by the application to register for any link related
 * event notifications from the kernel. If user specifies a valid interface
 * name in the if_name parameter, only updates relevant to that interface
 * are passed back to user application. If no interface name is specified,
 * then all link updates from kernel are passed back to the callback function
 * provided here.
 *
 * On success, error is set to 0 and a valid subscriber handle is returned;
 * Otherwise error is set to appropriate error code and NULL is returned.
 *
 * @return  >0 on success,
 *          NULL on error
 */
netmgr_subsc_hdl netmgr_register_link_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    const char*             if_name,
    netmgr_cb_fxn_t         cb_fxn,
    void*                   cb_arg,
    int32_t*                error
)
{
    subsc_info_t*       link_subsc;
    watch_info_t*       link_watch_info;
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    if (cb_fxn == NULL || error == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Allocate memory for and setup the Link subscriber info */
    if ((link_subsc = malloc (sizeof (subsc_info_t))) == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }
    memset ((void *)link_subsc, 0, sizeof (*link_subsc));
    link_subsc->cb_fxn =   cb_fxn;
    link_subsc->cb_arg =   cb_arg;

    /* Check if there is a Link watch list for the interface specified. If
     * not create one. */
    if ((link_watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->link_watch_list,
                                                    NETMGR_WATCH_TYPE_INTF_BASED,
                                                    (void *)if_name)) == NULL)
    {
        /* No existing watch list for interface specified. create one */
        if ((link_watch_info = netmgr_add_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->link_watch_list,
                                                       NETMGR_WATCH_TYPE_INTF_BASED, (void *)if_name)) == NULL)
        {
            free (link_subsc);
            *error = NETFP_PROXY_RETVAL_E_NO_MEM;
            return NULL;
        }
    }

    /* Add this subscriber to the Link watch list for the interface specified */
    netmgr_add_watch_subsc (ptrNetMgrMCB, link_watch_info, link_subsc);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Added Link watch point for %s\n", if_name);

    /* Success */
    *error = 0;
    return link_subsc;
}

/**
 * Function to unregister from Link related event notifications.
 *
 * @arg subsc_hdl   NetMgr Subscriber handle returned by the register
 *                  function.
 *
 * Must be called by the application to unregister from any Link related
 * event notifications from the kernel. The subscriber handle passed to this API
 * must be the handle obtained during event registration.
 *
 * @return 0 on success,
 *         NETFP_PROXY_RETVAL_E_INVALID_PARAMS if invalid input passed.
 */
int32_t netmgr_unregister_link_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    netmgr_subsc_hdl        subsc_hdl
)
{
    watch_info_t*       link_watch_info;
    subsc_info_t*       link_subsc;
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (subsc_hdl == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    link_subsc      =   (subsc_info_t *)subsc_hdl;
    link_watch_info =   (watch_info_t *)link_subsc->subsc_of;

    /* Remove this subscriber from the link watch list */
    netmgr_remove_watch_subsc (ptrNetMgrMCB, link_subsc);

    /* If there are no more subscribers listening on Link updates
     * for this interface that subscriber is using, remove the link watch list
     * for the interface too. */
    if (netmgr_get_watch_subsc_count (link_watch_info) == 0)
    {
        /* No more subscribers. Clean up this Link watch point */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Removing Link watch point for %s\n", link_watch_info->u.if_name);
        netmgr_remove_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->link_watch_list, link_watch_info);
    }

    /* Success */
    return 0;
}

/**
 * Function to register for Address related event notifications.
 *
 * @arg if_name     Name of the interface for which address updates
 *                  are to be provided.
 * @arg cb_fxn      User callback function to be invoked on
 *                  an event update.
 * @arg cb_arg      Argument to be passed to the user callback function.
 * @arg error       Error code returned.
 *
 * Must be called by the application to register for any Address related
 * event notifications from the kernel. If user specifies a valid interface
 * name in the if_name parameter, only updates relevant to that interface
 * are passed back to user application. If no interface name is specified,
 * then all address updates from kernel are passed back to the callback function
 * provided here.
 *
 * On success, error is set to 0 and a valid subscriber handle is returned;
 * Otherwise error is set to appropriate error code and NULL is returned.
 *
 * @return  >0 on success,
 *          NULL on error
 */
netmgr_subsc_hdl netmgr_register_addr_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    const char*             if_name,
    netmgr_cb_fxn_t         cb_fxn,
    void*                   cb_arg,
    int32_t*                error
)
{
    subsc_info_t*       addr_subsc;
    watch_info_t*       addr_watch_info;
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    if (cb_fxn == NULL || error == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Check if Net Manager instance is permitted to call this API. */
    if ((ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK) || (ptrNetMgrMCB->addr_cache == NULL))
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Allocate memory for and setup the address subscriber info */
    if ((addr_subsc = malloc (sizeof (subsc_info_t))) == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }
    memset ((void *)addr_subsc, 0, sizeof (*addr_subsc));
    addr_subsc->cb_fxn =   cb_fxn;
    addr_subsc->cb_arg =   cb_arg;

    /* Check if there is a address watch list for the interface specified. If
     * not create one. */
    if ((addr_watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->addr_watch_list,
                                                    NETMGR_WATCH_TYPE_INTF_BASED, (void *)if_name)) == NULL)
    {
        /* No existing watch list for interface specified. create one */
        if ((addr_watch_info = netmgr_add_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->addr_watch_list,
                                                       NETMGR_WATCH_TYPE_INTF_BASED, (void *)if_name)) == NULL)
        {
            free (addr_subsc);
            *error = NETFP_PROXY_RETVAL_E_NO_MEM;
            return NULL;
        }
    }

    /* Add this subscriber to the address watch list for the interface specified */
    netmgr_add_watch_subsc (ptrNetMgrMCB, addr_watch_info, addr_subsc);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Added Address watch point for %s\n", if_name);

    /* Success */
    *error = 0;
    return addr_subsc;
}

/**
 * Function to unregister from Address related event notifications.
 *
 * @arg subsc_hdl   NetMgr Subscriber handle returned by the register
 *                  function.
 *
 * Must be called by the application to unregister from any address related
 * event notifications from the kernel. The subscriber handle passed to this API
 * must be the handle obtained during event registration.
 *
 * @return 0 on success,
 *         NETFP_PROXY_RETVAL_E_INVALID_PARAMS if invalid input passed.
 */
int32_t netmgr_unregister_addr_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    netmgr_subsc_hdl        subsc_hdl
)
{
    watch_info_t*       addr_watch_info;
    subsc_info_t*       addr_subsc;
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (subsc_hdl == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if ((ptrNetMgrMCB->cacheType != NetMgr_CacheType_LINK) || (ptrNetMgrMCB->addr_cache == NULL))
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    addr_subsc      =   (subsc_info_t *)subsc_hdl;
    addr_watch_info =   (watch_info_t *)addr_subsc->subsc_of;

    /* Remove this subscriber from the address watch list */
    netmgr_remove_watch_subsc (ptrNetMgrMCB, addr_subsc);

    /* If there are no more subscribers listening on address updates
     * for this interface that subscriber is using, remove the address watch list
     * for the interface too. */
    if (netmgr_get_watch_subsc_count (addr_watch_info) == 0)
    {
        /* No more subscribers. Clean up this address watch */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Removing Address watch point for %s\n", addr_watch_info->u.if_name);
        netmgr_remove_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->addr_watch_list, addr_watch_info);
    }

    /* Success */
    return 0;
}

/**
 * Function to register for Neighbor related event notifications.
 *
 * @arg ip_addr     Destination IP(v4/v6) Address for which
 *                  neighbor notification events are to be sent.
 * @arg cb_fxn      User callback function to be invoked on
 *                  an event update.
 * @arg cb_arg      Argument to be passed to the user callback function.
 * @arg error       Error code returned.
 *
 * Must be called by the application to register for any neighbor (ARP) related
 * event notifications from the kernel. If user specifies a valid IP address
 * in the ip_addr parameter, only updates relevant to that destination IP address
 * are passed back to user application. If no IP address is specified,
 * then all neighbor updates from kernel are passed back to the callback function
 * provided here.
 *
 * On success, error is set to 0 and a valid subscriber handle is returned;
 * Otherwise error is set to appropriate error code and NULL is returned.
 *
 * @return  >0 on success,
 *          NULL on error
 */
netmgr_subsc_hdl netmgr_register_neigh_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    struct nl_addr*         ip_addr,
    netmgr_cb_fxn_t         cb_fxn,
    void*                   cb_arg,
    int32_t*                error
)
{
    subsc_info_t*       neigh_subsc;
    watch_info_t*       neigh_watch_info;
    char                tmpBuf[128];
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    if (cb_fxn == NULL || error == NULL)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_NEIGH)
    {
        *error = NETFP_PROXY_RETVAL_E_INVALID_PARAMS;
        return NULL;
    }

    /* Allocate memory for and setup the neighbor subscriber info */
    if ((neigh_subsc = malloc (sizeof (subsc_info_t))) == NULL)
    {
        *error  =   NETFP_PROXY_RETVAL_E_NO_MEM;
        return NULL;
    }
    memset ((void *)neigh_subsc, 0, sizeof (*neigh_subsc));
    neigh_subsc->cb_fxn =   cb_fxn;
    neigh_subsc->cb_arg =   cb_arg;

    /* Check if there is a neighbor watch list for the IP address specified. If
     * not create one. */
    if ((neigh_watch_info = netmgr_find_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->neigh_watch_list,
                                                    NETMGR_WATCH_TYPE_ADDR_BASED, (void *)ip_addr)) == NULL)
    {
        /* No existing watch list for IP address specified. Create one */
        if ((neigh_watch_info = netmgr_add_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->neigh_watch_list,
                                                        NETMGR_WATCH_TYPE_ADDR_BASED, (void *)ip_addr)) == NULL)
        {
            free (neigh_subsc);
            *error  =   NETFP_PROXY_RETVAL_E_NO_MEM;
            return NULL;
        }
    }

    /* Add this subscriber to the neighbor watch list for the address specified */
    netmgr_add_watch_subsc (ptrNetMgrMCB, neigh_watch_info, neigh_subsc);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Added Neighbor watch point for %s\n",
                       nl_addr2str (ip_addr, tmpBuf, sizeof (tmpBuf)));

    /* Success */
    *error = 0;
    return neigh_subsc;
}

/**
 * Function to unregister from Neighbor related event notifications.
 *
 * @arg subsc_hdl   NetMgr Subscriber handle returned by the register
 *                  function.
 *
 * Must be called by the application to unregister from any neighbor related
 * event notifications from the kernel. The subscriber handle passed to this API
 * must be the handle obtained during event registration.
 *
 * @return 0 on success,
 *         NETFP_PROXY_RETVAL_E_INVALID_PARAMS if invalid input passed.
 */
int32_t netmgr_unregister_neigh_updates
(
    NetfpProxy_NetMgrHandle netMgrHandle,
    netmgr_subsc_hdl        subsc_hdl
)
{
    watch_info_t*       neigh_watch_info;
    subsc_info_t*       neigh_subsc;
    char                tmpBuf[128];
    netmgr_mcb_t*       ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    if (subsc_hdl == NULL)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    /* Check if Net Manager instance is permitted to call this API. */
    if (ptrNetMgrMCB->cacheType != NetMgr_CacheType_NEIGH)
        return NETFP_PROXY_RETVAL_E_INVALID_PARAMS;

    neigh_subsc     = (subsc_info_t *)subsc_hdl;
    neigh_watch_info= (watch_info_t *)neigh_subsc->subsc_of;

    /* Remove this subscriber from the neighbor watch list */
    netmgr_remove_watch_subsc (ptrNetMgrMCB, neigh_subsc);

    /* If there are no more subscribers listening on neighbor updates
     * for this IP address that subscriber is using, remove the route watch list
     * for the address too. */
    if (netmgr_get_watch_subsc_count (neigh_watch_info) == 0)
    {
        /* No more subscribers. Clean up this route watch */
        NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "DEBUG: Removing Neighbor watch point for %s\n",
                            nl_addr2str (neigh_watch_info->u.ip_addr, tmpBuf, sizeof (tmpBuf)));
        netmgr_remove_watch_entry (ptrNetMgrMCB, &ptrNetMgrMCB->neigh_watch_list, neigh_watch_info);
    }

    /* Success */
    return 0;
}

/**
 * Network Manager's event poll function. Must be called from an application
 * periodically to enable the interface to check for any event notifications
 * from the networking subsystem in the kernel.
 *
 * @arg timeout_val     Maximum time in milliseconds to wait for any event updates.
 *
 * Blocks for the timeout specified and checks for any event notifications.
 * If any events received, appropriate callback functions are called to
 * notify the application of the event.
 *
 * @return None
 */
void netmgr_poll_updates (NetfpProxy_NetMgrHandle netMgrHandle, uint32_t timeout_val)
{
    int32_t         num_events;
    netmgr_mcb_t*   ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return;

    /* Wait for any event notifications from the cache manager for the timeout
     * value (in milliseconds) provided. */
    /* TODO: Should this be mutex protected also? Hard to test. */
    if ((num_events = nl_cache_mngr_poll (ptrNetMgrMCB->net_cache_mngr, timeout_val)) < 0)
    {
        /* Timeout reached. No updates to report */
    }
    else
    {
        /* Timeout reached. Found event notifications. */
    }
    return;
}

/**
 * Get the socket associated with the NETMGR
 *
 * @return Socket file descriptor
 */
int32_t netmgr_getSocket (NetfpProxy_NetMgrHandle netMgrHandle)
{
    netmgr_mcb_t*   ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return -1;

    return nl_cache_mngr_get_fd(ptrNetMgrMCB->net_cache_mngr);
}

int32_t netmgr_process_message (NetfpProxy_NetMgrHandle netMgrHandle)
{
    netmgr_mcb_t*   ptrNetMgrMCB;
    int32_t res = 0;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return res;

    if(pthread_mutex_trylock(&libnlGuard) == 0)
    {
        res = nl_cache_mngr_data_ready(ptrNetMgrMCB->net_cache_mngr);
        pthread_mutex_unlock(&libnlGuard);
    }

    return res;
}

/**
 * Initializes the Network Manager interface.
 *
 * @return NETFP_PROXY_RETVAL_E_OP_FAILED if initialized failed or 0 on success
 */
NetfpProxy_NetMgrHandle netmgr_init (NetMgr_CacheType cacheType, int32_t* errCode)
{
    int32_t         error;
    netmgr_mcb_t*   ptrNetMgrMCB;

    /* Allocate memory for the NETMGR MCB */
    ptrNetMgrMCB = (netmgr_mcb_t*)malloc (sizeof(netmgr_mcb_t));
    if (ptrNetMgrMCB == NULL)
        return NULL;

    /* Initialize the allocated memory */
    memset ((void *)ptrNetMgrMCB, 0, sizeof (netmgr_mcb_t));

    /* Store cache type in the Net Manager instance */
    ptrNetMgrMCB->cacheType = cacheType;

    /* Setup a Cache manager for NETLINK_ROUTE.
     * Open a NETLINK_ROUTE socket automatically, allocate a cache and keep it updated automatically */
    if ((error = nl_cache_mngr_alloc (NULL, NETLINK_ROUTE, NL_AUTO_PROVIDE, &ptrNetMgrMCB->net_cache_mngr)) < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error allocating NETLINK_ROUTE Cache manager \n");
        *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
        return NULL;
    }

    /* Add caches for the net components we want to monitor */
    if (cacheType == NetMgr_CacheType_NEIGH)
    {
        /* Set up the neighbor cache */
        if ((error = nl_cache_mngr_add (ptrNetMgrMCB->net_cache_mngr, "route/neigh",
                &netmgr_receive_neigh_updates, (void*)ptrNetMgrMCB, &ptrNetMgrMCB->net_cache)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error setting up neigh cache, error: %d \n", error);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* Open a Netlink Route socket to use for Cache resync operations */
        if ((ptrNetMgrMCB->nl_sock = nl_socket_alloc ()) == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error opening Netlink socket for Network Manager \n");
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        if ((error = nl_connect (ptrNetMgrMCB->nl_sock, NETLINK_ROUTE)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error connecting to NETLINK_ROUTE, error: %d \n", error);
            nl_socket_free (ptrNetMgrMCB->nl_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* Open a socket to read the properties of the interface used to reach the neighbor. */
        ptrNetMgrMCB->if_sock = socket(PF_INET, SOCK_DGRAM, 0);
        if (ptrNetMgrMCB->if_sock < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Cannot opening socket for cache type %d\n", cacheType);
            nl_socket_free (ptrNetMgrMCB->nl_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }
    }

    if (cacheType == NetMgr_CacheType_ROUTE)
    {
        /* Set up the neighbor cache */
        if ((error = nl_cache_mngr_add (ptrNetMgrMCB->net_cache_mngr, "route/route",
                &netmgr_receive_route_updates, (void*)ptrNetMgrMCB, &ptrNetMgrMCB->net_cache)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error setting up route cache, error: %d \n", error);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* Open a socket to read the properties of the interface used for routing. */
        ptrNetMgrMCB->if_sock = socket(PF_INET, SOCK_DGRAM, 0);
        if (ptrNetMgrMCB->if_sock < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Cannot opening socket for cache type %d\n", cacheType);
            nl_socket_free (ptrNetMgrMCB->nl_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* Open a Netlink Route socket to use for Cache resync operations */
        if ((ptrNetMgrMCB->nl_sock = nl_socket_alloc ()) == NULL)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error opening Netlink socket for Network Manager \n");
            close (ptrNetMgrMCB->if_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        if ((error = nl_connect (ptrNetMgrMCB->nl_sock, NETLINK_ROUTE)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error connecting to NETLINK_ROUTE, error: %d \n", error);
            nl_socket_free (ptrNetMgrMCB->nl_sock);
            close (ptrNetMgrMCB->if_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* Allocate and load the Routing rules cache for Route caches. */
        if ((error = rtnl_rule_alloc_cache (ptrNetMgrMCB->nl_sock, AF_UNSPEC, &ptrNetMgrMCB->rule_cache)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error allocating/loading rule cache, error: %d\n", error);
            nl_socket_free (ptrNetMgrMCB->nl_sock);
            close (ptrNetMgrMCB->if_sock);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }
        nl_cache_mngt_provide(ptrNetMgrMCB->rule_cache);

        /* Load the list of all available table names at the moment */
        rtnl_route_read_table_names (NETMGR_ROUTE_TABLE_PATH);
    }

    if (cacheType == NetMgr_CacheType_LINK)
    {
        /* Set up the neighbor cache */
        if ((error = nl_cache_mngr_add (ptrNetMgrMCB->net_cache_mngr, "route/link",
                &netmgr_receive_link_updates, (void*)ptrNetMgrMCB, &ptrNetMgrMCB->net_cache)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error setting up link cache, error: %d \n", error);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }

        /* For interface module, we also need to monitor the address cache.
         * Set up the address cache */
        if ((error = nl_cache_mngr_add (ptrNetMgrMCB->net_cache_mngr, "route/addr",
                &netmgr_receive_addr_updates, (void*)ptrNetMgrMCB, &ptrNetMgrMCB->addr_cache)) < 0)
        {
            NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "ERROR: Error setting up addr cache, error: %d \n", error);
            nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);
            free (ptrNetMgrMCB);
            *errCode = NETFP_PROXY_RETVAL_E_OP_FAILED;
            return NULL;
        }
    }

    /* Init done. Return success */
    return (NetfpProxy_NetMgrHandle)ptrNetMgrMCB;
}

/**
 * De-initializes the Network Manager interface state machine.
 */
int32_t netmgr_deinit (NetfpProxy_NetMgrHandle netMgrHandle)
{
    netmgr_mcb_t*   ptrNetMgrMCB;

    /* Sanity Check: Validate the arguments */
    ptrNetMgrMCB = (netmgr_mcb_t*)netMgrHandle;
    if (ptrNetMgrMCB == NULL)
        return -1;

    /* Close and free the resync netlink socket */
    nl_close (ptrNetMgrMCB->nl_sock);
    nl_socket_free (ptrNetMgrMCB->nl_sock);

    /* Free the rule cache */
    if (ptrNetMgrMCB->rule_cache)
        nl_cache_free (ptrNetMgrMCB->rule_cache);

    /* Close the interface socket */
    close (ptrNetMgrMCB->if_sock);

    /* Free the cache manager instance and unregister from all network events */
    nl_cache_mngr_free (ptrNetMgrMCB->net_cache_mngr);

    /* Free the MCB */
    free (ptrNetMgrMCB);

    /* Deinit done. Return success */
    return 0;
}
