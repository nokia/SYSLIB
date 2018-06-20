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
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <ctype.h>
#include <arpa/inet.h>

/* Command shell application local includes */
#include "cmd_shell_loc.h"
#include "listlib.h"

/* For debugging */
#undef  DEBUG
#undef  DEBUG1

#define OPTION_ID_SP_ID             1
#define OPTION_ID_CMD_NAME          2
#define OPTION_ID_GTPU_TEID         3
#define OPTION_ID_NUM_TEIDS         4
#define OPTION_ID_SHARED_SA         5
#define OPTION_ID_NONSECURE_SP      6
#define OPTION_ID_IF_NAME           7
#define OPTION_ID_MAC_ADDR          8
#define OPTION_ID_OUTER_FRAG        9
#define OPTION_ID_SP_NAME           10
#define OPTION_ID_INTERFACE_NAME    11
#define OPTION_ID_QOS_ENABLE        12
#define OPTION_ID_FLOW_ID           13
#define OPTION_ID_SRC_IP            14
#define OPTION_ID_DST_IP            15
#define OPTION_ID_PMTU              16
#define OPTION_ID_MAX               17

enum cmd_id {
    CMD_ID_OFFLOAD_SP           = 1,
    CMD_ID_HELP,
    CMD_ID_STOP_OFFLOAD_SP,
    CMD_ID_START_CASCADE,
    CMD_ID_STOP_CASCADE,
    CMD_ID_IP_ROUTE_FLUSH,
    CMD_ID_ADD_INTERFACE,
    CMD_ID_DEL_INTERFACE,
    CMD_ID_CONFIGURE_L3_QOS,
    CMD_ID_FLUSH_VLAN_PRIORITY,
    CMD_ID_SETUP_PMTU,
    CMD_ID_EXIT
};

/* Maximum strlen() of string variables */
#define MAX_STR_VAR_SIZE        32

/* Maximum number of integer array elements */
#define MAX_UINTARR_VAR_SIZE    16

/* Maximum number of commands */
#define MAX_CMDS                (CMD_ID_EXIT + 1)

typedef union {
    unsigned long           intval;
    char                    strval[MAX_STR_VAR_SIZE];
    uint8_t                 intarrval [MAX_UINTARR_VAR_SIZE];
    Netfp_IPAddr            ipAddress;
} opt_val_gen_t;

typedef struct {
    opt_val_gen_t   value;
    uint8_t         is_valid;
} opt_attr_gen_t;

static opt_attr_gen_t opt_input_gen[OPTION_ID_MAX];

/* Module context */
static cmd_shell_ctx_t shell_ctx;

static int parse_intval(char* int_str, opt_attr_gen_t *opt_attr) {
    int i;
    for (i=0; int_str[i]!= '\0'; i++) int_str[i] = (char) tolower(int_str[i]);
    if (strncmp("0x", int_str, 2)) {
        opt_attr->value.intval = strtoul(int_str, (char **)NULL, 10);
    } else {
        opt_attr->value.intval = strtoul(int_str, (char **)NULL, 16);
    }
    opt_attr->is_valid = 1;
    return 0;
}

static int parse_ipaddress(const char* int_str, opt_attr_gen_t *opt_attr)
{
    struct sockaddr_in  ip4_address;
    struct in6_addr     ip6_address;

    if (inet_pton(AF_INET, int_str, &ip4_address.sin_addr) == 1)
    {
        opt_attr->value.ipAddress.ver = Netfp_IPVersion_IPV4;
        memcpy ((void*)&opt_attr->value.ipAddress.addr.ipv4.u.a8[0], (void *)&ip4_address.sin_addr.s_addr, 4);
        opt_attr->is_valid = 1;
        return 0;
    }
    if (inet_pton(AF_INET6, int_str, &ip6_address) == 1)
    {
        opt_attr->value.ipAddress.ver = Netfp_IPVersion_IPV6;
        memcpy ((void*)&opt_attr->value.ipAddress.addr.ipv6.u.a8[0], (void *)&ip6_address.s6_addr[0], 16);
        opt_attr->is_valid = 1;
        return 0;
    }
    return -1;
}

static int parse_strval(const char* str, opt_attr_gen_t *opt_attr) {

    strncpy(opt_attr->value.strval, str, MAX_STR_VAR_SIZE-1);
    opt_attr->is_valid = 1;
    return 0;
}

static int parse_intarrval(char* int_str, opt_attr_gen_t *opt_attr) {
    int     i = 0;
    char*   tok1;
    char*   tok2;
    const char*   delimiter = ".:";

    /* Get each of the tokens from the string */
    while ((tok1 = strtok (int_str, delimiter)) != NULL)
    {
        /* Get the integer equivalent of the string */
        opt_attr->value.intarrval[i] = (uint8_t)strtol (tok1, &tok2, 16);

        /* Continue parsing rest of the string */
        int_str = NULL;

        i ++;
    }
    opt_attr->is_valid = 1;

    return 0;
}



static char short_opts[200];

#define CMD_NAME_OFFLOAD_SP   "offload_sp"
#define CMD_DESC_OFFLOAD_SP   "Offload an IPSec Security Policy to Fast Path"
#define CMD_MIN_ARGS_OFFLOAD_SP    3
#define CMD_SHORT_OPTS_OFFLOAD_SP sprintf(short_opts, "%d:%d%d:%d:%d:%d:%d:",\
                           OPTION_ID_SP_ID, OPTION_ID_GTPU_TEID, OPTION_ID_NUM_TEIDS,\
                           OPTION_ID_SHARED_SA, OPTION_ID_NONSECURE_SP,  \
                           OPTION_ID_OUTER_FRAG, OPTION_ID_SP_NAME);

static struct option offload_sp_options[] =
{
    {"sp_id",  required_argument, 0, OPTION_ID_SP_ID},
    {"gtpu_teid", required_argument, 0, OPTION_ID_GTPU_TEID},
    {"num_teids", required_argument, 0, OPTION_ID_NUM_TEIDS},
    {"shared", no_argument, 0, OPTION_ID_SHARED_SA},
    {"non_secure", no_argument, 0, OPTION_ID_NONSECURE_SP},
    {"enable_outerip_frag", no_argument, 0, OPTION_ID_OUTER_FRAG},
    {"sp_name", required_argument, 0, OPTION_ID_SP_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_EXIT   "exit"
#define CMD_DESC_EXIT   "Exit the netcpcfg command shell"
#define CMD_MIN_ARGS_EXIT    1
#define CMD_SHORT_OPTS_EXIT sprintf(short_opts, "");
static struct option exit_options[] =
{
    {0, 0, 0, 0}
};

#define CMD_NAME_HELP       "help"
#define CMD_DESC_HELP       "Help on available commands"
#define CMD_MIN_ARGS_HELP   1
#define CMD_SHORT_OPTS_HELP sprintf(short_opts, "%d:", OPTION_ID_CMD_NAME);
static struct option help_options[] =
{
    {"cmd",  required_argument, 0, OPTION_ID_CMD_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_STOP_OFFLOAD_SP        "stop_offload_sp"
#define CMD_DESC_STOP_OFFLOAD_SP        "Stop Offload of an IPSec Security Policy to Fast Path"
#define CMD_MIN_ARGS_STOP_OFFLOAD_SP    3
#define CMD_SHORT_OPTS_STOP_OFFLOAD_SP  sprintf(short_opts, "%d:%d:",OPTION_ID_SP_ID, OPTION_ID_SP_NAME);

static struct option stop_offload_sp_options[] =
{
    {"sp_id",  required_argument, 0, OPTION_ID_SP_ID},
    {"sp_name", required_argument, 0, OPTION_ID_SP_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_IP_ROUTE_FLUSH          "ip_route_flush"
#define CMD_DESC_IP_ROUTE_FLUSH          "Notify IP Route table has been flushed to Fast Path"
#define CMD_MIN_ARGS_IP_ROUTE_FLUSH      0

#define CMD_NAME_ADD_INTERFACE           "add_interface"
#define CMD_DESC_ADD_INTERFACE           "Add an interface"
#define CMD_MIN_ARGS_ADD_INTERFACE       1
#define CMD_SHORT_OPTS_ADD_INTERFACE     sprintf(short_opts, "%d",OPTION_ID_INTERFACE_NAME);

static struct option add_interface_options[] =
{
    {"interface_name",  required_argument, 0, OPTION_ID_INTERFACE_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_DEL_INTERFACE           "del_interface"
#define CMD_DESC_DEL_INTERFACE           "Delete an interface"
#define CMD_MIN_ARGS_DEL_INTERFACE       1
#define CMD_SHORT_OPTS_DEL_INTERFACE     sprintf(short_opts, "%d",OPTION_ID_INTERFACE_NAME);

static struct option del_interface_options[] =
{
    {"interface_name",  required_argument, 0, OPTION_ID_INTERFACE_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_CONFIGURE_L3_QOS       "configure_l3_qos"
#define CMD_DESC_CONFIGURE_L3_QOS       "Configure interface L3 QoS parameters"
#define CMD_MIN_ARGS_CONFIGURE_L3_QOS   2
#define CMD_SHORT_OPTS_CONFIGURE_L3_QOS sprintf(short_opts, "%d:%d:%d",OPTION_ID_INTERFACE_NAME,\
                                            OPTION_ID_QOS_ENABLE, OPTION_ID_FLOW_ID);

static struct option configure_l3_qos_options[] =
{
    {"interface_name",  required_argument, 0, OPTION_ID_INTERFACE_NAME},
    {"qos_enable", required_argument, 0, OPTION_ID_QOS_ENABLE},
    {"flow_id", required_argument, 0, OPTION_ID_FLOW_ID},
    {0, 0, 0, 0}
};

#define CMD_NAME_FLUSH_VLAN_PRIORITY        "flush_vlan_priority"
#define CMD_DESC_FLUSH_VLAN_PRIORITY        "Flush VLAN Egress Priority Mapping"
#define CMD_MIN_ARGS_FLUSH_VLAN_PRIORITY    1
#define CMD_SHORT_OPTS_FLUSH_VLAN_PRIORITY  sprintf(short_opts, "%d", OPTION_ID_INTERFACE_NAME);

static struct option flush_vlan_priority_options[] =
{
    {"interface_name",  required_argument, 0, OPTION_ID_INTERFACE_NAME},
    {0, 0, 0, 0}
};

#define CMD_NAME_SETUP_PMTU        "setup_pmtu"
#define CMD_DESC_SETUP_PMTU        "Setup PMTU associated for all packets from SRC IP -> DST IP"
#define CMD_MIN_ARGS_SETUP_PMTU    2
#define CMD_SHORT_OPTS_SETUP_PMTU  sprintf(short_opts, "%d:%d:%d", OPTION_ID_SRC_IP, OPTION_ID_DST_IP, OPTION_ID_PMTU);

static struct option setup_mtu_options[] =
{
    {"srcip",  required_argument, 0, OPTION_ID_SRC_IP},
    {"dstip",  required_argument, 0, OPTION_ID_DST_IP},
    {"pmtu",    required_argument, 0, OPTION_ID_PMTU},
    {0, 0, 0, 0}
};


struct cmd_tbl_s {
    const char            *cmd_name;
    const struct option   *opt_tbl;
    const char            *desc;
};

const struct cmd_tbl_s cmd_table[MAX_CMDS] =
{
    {CMD_NAME_OFFLOAD_SP, offload_sp_options, CMD_DESC_OFFLOAD_SP},
    {CMD_NAME_EXIT, exit_options, CMD_DESC_EXIT},
    {CMD_NAME_HELP, help_options, CMD_DESC_HELP},
    {CMD_NAME_STOP_OFFLOAD_SP, stop_offload_sp_options, CMD_DESC_STOP_OFFLOAD_SP},
    {CMD_NAME_IP_ROUTE_FLUSH, NULL, CMD_DESC_IP_ROUTE_FLUSH},
    {CMD_NAME_ADD_INTERFACE, add_interface_options, CMD_DESC_ADD_INTERFACE},
    {CMD_NAME_DEL_INTERFACE, del_interface_options, CMD_DESC_DEL_INTERFACE},
    {CMD_NAME_CONFIGURE_L3_QOS, configure_l3_qos_options, CMD_DESC_CONFIGURE_L3_QOS},
    {CMD_NAME_FLUSH_VLAN_PRIORITY, flush_vlan_priority_options, CMD_DESC_FLUSH_VLAN_PRIORITY},
    {CMD_NAME_SETUP_PMTU, setup_mtu_options, CMD_DESC_SETUP_PMTU},
    {NULL, NULL, NULL}
};

/* IPC socket extern definitions */
extern struct sockaddr_un          proxySunAddr;
extern int32_t                     cmdAppIpcSockFd;

static void print_help (char *cmd_name)
{
    int i,j;

    if (!cmd_name) {
        printf ("For help on command options type \"%s --cmd <cmd name>\"\n",
                CMD_NAME_HELP);
        printf ("Available commands:\n");

        for (i=0; ((i < MAX_CMDS) && cmd_table[i].cmd_name); i++) {
            printf ("    %s\t%s\n", cmd_table[i].cmd_name, cmd_table[i].desc);
        }
        return;
    }

    for (i=0; ((i < MAX_CMDS) && (cmd_table[i].cmd_name)); i++) {
        const struct option *opt = cmd_table[i].opt_tbl;
        if (strcmp(cmd_name, cmd_table[i].cmd_name)) {
            continue;
        }

        printf ("options for \"%s\"\n", cmd_name);
        for (j=0; opt->name; j++, opt++) {
            printf ("    --%s\n", opt->name);
        }
        return;
    }

    printf ("No help available for %s\n", cmd_name);
    return;
}

static int setargs (char *args, char **argv)
{
    int count = 0;

    while (isspace(*args)) ++args;
    while (*args) {
        if (argv) argv[count] = args;
        while (*args && !isspace(*args)) ++args;
        if (argv && *args) *args++ = '\0';
        while (isspace(*args)) ++args;
        count++;
    }
    return count;
}

static char **parsedargs(char *args, int *argc)
{
    char **argv = NULL;
    int    argn = 0;
    char *parsed_args = NULL;

    if (args && *args
        && (parsed_args = strdup(args))
        && (argn = setargs(parsed_args,NULL))
        && (argv = malloc((argn+1) * sizeof(char *)))) {
          *argv++ = parsed_args;
          argn = setargs(parsed_args,argv);
    }

    if (parsed_args && !argv) free(parsed_args);

    *argc = argn;
    return argv;
}

static void freeparsedargs(char **argv)
{
    if (argv) {
        free(argv[-1]);
        free(argv-1);
    }
}

typedef struct _sp_info
{
    List_Node   list_n;
    uint32_t    policy_id;
    char        policy_name[32];
} sp_info;

sp_info*    sp_list;
extern int32_t                     nrDBId;

static int32_t save_offloaded_sp (uint32_t policy_id, const char* policy_name)
{
    sp_info*    sp;

    /* Allocate memory for the policy info */
    sp = malloc (sizeof (sp_info));
    if (sp == NULL)
        return -1;

    /* Save the info passed */
    sp->policy_id = policy_id;
    strcpy (sp->policy_name, policy_name);
    List_addNode ((List_Node**)&sp_list, (List_Node*)sp);

    return 0;
}

static int32_t delete_offloaded_sp (uint32_t policy_id)
{
    sp_info*    sp;

    sp = (sp_info*)List_getHead ((List_Node**)&sp_list);
    while (sp != NULL)
    {
        if (sp->policy_id == policy_id)
        {
            List_removeNode ((List_Node**)&sp_list,  (List_Node*)sp);
            free (sp);
            return 0;
        }
        else
        {
            sp = (sp_info*)List_getNext ((List_Node*)sp);
        }
    }

    return -1;
}

static int32_t find_offloaded_sp (uint32_t policy_id, char* policy_name)
{
    sp_info*    sp;

    for (sp = (sp_info*)List_getHead ((List_Node**)&sp_list);
         sp != NULL;
         sp = (sp_info*)List_getNext ((List_Node*)sp))
    {
        if (sp->policy_id == policy_id )
        {
            strcpy (policy_name, sp->policy_name);
            return 0;
        }
    }

    return -1;
}

#define COMMAND_LINE_SIZE 400

void* cmd_shell (void* args)
{
    char *line = NULL;
    size_t len = 0;
    ssize_t read = 0;
    char cmd[32];
    int nargs, c, rargs;
    struct option *long_options;
    char **av;
    enum cmd_id  cmd_id;
    char sp_name[32];

    memset(&shell_ctx, 0, sizeof(shell_ctx));

    while (1) {
        memset(opt_input_gen, 0, sizeof(opt_input_gen));
        cmd_id = 0;

        printf ("\nNETFP-PROXY> ");

        if (line) free(line);
        line = malloc(COMMAND_LINE_SIZE);
        if (line == NULL) {
            printf("\nError allocating memory");
            return NULL;
        }

        read = getline(&line, &len, stdin);
        if (read == -1) {
            printf("\nERROR: reading line");
            continue;
        }
        if (!read) continue;
        memset(cmd, 0, sizeof(cmd));
        sscanf(line, "%s", cmd);

        if ( !strcmp(cmd, "exit"))
            exit(0);

        if (!strcmp(cmd, CMD_NAME_OFFLOAD_SP)) {
            cmd_id = CMD_ID_OFFLOAD_SP;
            long_options = offload_sp_options;
            CMD_SHORT_OPTS_OFFLOAD_SP;
            rargs = CMD_MIN_ARGS_OFFLOAD_SP;
        } else if (!strcmp(cmd, CMD_NAME_HELP)) {
            cmd_id = CMD_ID_HELP;
            long_options = help_options;
            CMD_SHORT_OPTS_HELP;
            rargs = CMD_MIN_ARGS_HELP;
        } else if (!strcmp(cmd, CMD_NAME_STOP_OFFLOAD_SP)) {
            cmd_id = CMD_ID_STOP_OFFLOAD_SP;
            long_options = stop_offload_sp_options;
            CMD_SHORT_OPTS_STOP_OFFLOAD_SP;
            rargs = CMD_MIN_ARGS_STOP_OFFLOAD_SP;
         } else if (!strcmp(cmd, CMD_NAME_IP_ROUTE_FLUSH)) {
            cmd_id = CMD_ID_IP_ROUTE_FLUSH;
            rargs = CMD_MIN_ARGS_IP_ROUTE_FLUSH;
         } else if (!strcmp(cmd, CMD_NAME_ADD_INTERFACE)) {
            cmd_id = CMD_ID_ADD_INTERFACE;
            long_options = add_interface_options;
            CMD_SHORT_OPTS_ADD_INTERFACE;
            rargs = CMD_MIN_ARGS_ADD_INTERFACE;
         } else if (!strcmp(cmd, CMD_NAME_DEL_INTERFACE)) {
            cmd_id = CMD_ID_DEL_INTERFACE;
            long_options = del_interface_options;
            CMD_SHORT_OPTS_DEL_INTERFACE;
            rargs = CMD_MIN_ARGS_DEL_INTERFACE;
         } else if (!strcmp(cmd, CMD_NAME_CONFIGURE_L3_QOS)) {
            cmd_id = CMD_ID_CONFIGURE_L3_QOS;
            long_options = configure_l3_qos_options;
            CMD_SHORT_OPTS_CONFIGURE_L3_QOS;
            rargs = CMD_MIN_ARGS_CONFIGURE_L3_QOS;
         } else if (!strcmp(cmd, CMD_NAME_FLUSH_VLAN_PRIORITY)) {
            cmd_id = CMD_ID_FLUSH_VLAN_PRIORITY;
            long_options = flush_vlan_priority_options;
            CMD_SHORT_OPTS_FLUSH_VLAN_PRIORITY;
            rargs = CMD_MIN_ARGS_FLUSH_VLAN_PRIORITY;
         } else if (!strcmp(cmd, CMD_NAME_SETUP_PMTU)) {
            cmd_id = CMD_ID_SETUP_PMTU;
            long_options = setup_mtu_options;
            CMD_SHORT_OPTS_SETUP_PMTU;
            rargs = CMD_MIN_ARGS_SETUP_PMTU;
         } else {
            if (strlen(cmd))
                printf ("Unknown command: %s\n", cmd);
            continue;
        }

        if ((av = parsedargs(line, &nargs)) == NULL) {
            printf ("error parsing arguments");
            continue;
        }

        if (nargs < rargs) {
            printf ("Insufficient paramaters for command \"%s\"\n", cmd);
            print_help (cmd);
            goto loop_over;
        }

#ifdef DEBUG1
        {
            int i;
            for (i = 0; i < nargs; i++)
            printf("[%s]\n",av[i]);
        }
#endif

        optind = 0;
        while (1) {
            /* getopt_long stores the option index here. */
            int option_index = 0;
            c = getopt_long (nargs, av, short_opts,
                        long_options, &option_index);

            /* Detect the end of the options. */
#ifdef DEBUG1
            printf("c=%d",c);
#endif
            if (c == -1)
                break;

            switch (c)
            {

                case OPTION_ID_SP_ID:
                {
#ifdef DEBUG
                    printf ("option sp_id with value `%s'\n", optarg);
#endif
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for sp_id\n");
                    }
                    break;
                }

                case OPTION_ID_CMD_NAME:
                {
#ifdef DEBUG
                    printf ("option cmd with value `%s'\n", optarg);
#endif
                    if (parse_strval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for cmd\n");
                    }
                    break;
                }

                case OPTION_ID_GTPU_TEID:
                {
#ifdef DEBUG
                    printf ("option gtpu_teid with value `%s'\n", optarg);
#endif
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for gtpu_teid\n");
                    }
                    break;
                }

                case OPTION_ID_NUM_TEIDS:
                {
#ifdef DEBUG
                    printf ("option num_teids with value `%s'\n", optarg);
#endif
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for num_teids\n");
                    }
                    break;
                }

                case OPTION_ID_SHARED_SA:
                {
#ifdef DEBUG
                    printf ("option shared SA enabled\n");
#endif
                    opt_input_gen[c].is_valid = 1;
                    break;
                }
                case OPTION_ID_NONSECURE_SP:
                {
#ifdef DEBUG
                    printf ("option non-secure SP enabled\n");
#endif
                    opt_input_gen[c].is_valid = 1;
                    break;
                }

                case OPTION_ID_MAC_ADDR:
                {
#ifdef DEBUG
                    printf ("option mac_addr with value `%s'\n", optarg);
#endif
                    if (parse_intarrval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for if_name\n");
                    }
                    break;
                }
                case OPTION_ID_OUTER_FRAG:
                {
#ifdef DEBUG
                    printf ("option outer IP fragmentation enabled\n");
#endif
                    opt_input_gen[c].is_valid = 1;
                    break;
                }
                case OPTION_ID_SP_NAME:
                {
#ifdef DEBUG
                    printf ("option sp_name with value `%s'\n", optarg);
#endif
                    if (parse_strval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for sp_name\n");
                    }
                    break;
                }
                case OPTION_ID_INTERFACE_NAME:
                {
#ifdef DEBUG
                    printf ("option interface_name with value `%s'\n", optarg);
#endif
                    if (parse_strval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for interface_name\n");
                    }
                    break;
                }
                case OPTION_ID_QOS_ENABLE:
                {
#ifdef DEBUG
                    printf ("option qos_enable\n");
#endif
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for qos_enable\n");
                    }
                    if (opt_input_gen[c].value.intval == 0)
                    {
                        opt_input_gen[OPTION_ID_FLOW_ID].is_valid = 1;
                    }

                    break;
                }
                case OPTION_ID_FLOW_ID:
                {
#ifdef DEBUG
                    printf ("option flow_id\n");
#endif
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for flow_id\n");
                    }
                    break;
                }
                case OPTION_ID_SRC_IP:
                {
                    if (parse_ipaddress(optarg, &opt_input_gen[c]) < 0) {
                        printf ("Invalid argument for srcip\n");
                    }
                    break;
                }
                case OPTION_ID_DST_IP:
                {
                    if (parse_ipaddress(optarg, &opt_input_gen[c]) < 0) {
                        printf ("Invalid argument for dstip\n");
                    }
                    break;
                }
                case OPTION_ID_PMTU:
                {
                    if (parse_intval(optarg, &opt_input_gen[c])) {
                        printf ("Invalid argument for pmtu\n");
                    }
                    break;
                }
                default:
                {
                    printf ("unknown option c=%d\n", c);
                    break;
                }
            }
        }
#ifdef DEBUG1
    /* Print any remaining command line arguments (not options). */
    if (optind < nargs)
    {
        printf ("non-option elements: ");
        while (optind < nargs)
            printf ("%s ", av[optind++]);
        putchar ('\n');
    }
#endif

    switch (cmd_id) {

        case CMD_ID_HELP:
        {
            if (!opt_input_gen[OPTION_ID_CMD_NAME].is_valid) {
                print_help(NULL);
            } else {
                print_help(opt_input_gen[OPTION_ID_CMD_NAME].value.strval);
            }
            break;
        }

        case CMD_ID_OFFLOAD_SP:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_SP_ID].is_valid) {
                printf ("Mandatory paramater missing: sp_id\n");
                goto loop_over;
            }

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* By default, we assume that the policy is secure, i.e., associated
             * with an SA */
            reqMsg.body.spReq.bIsSecure         =   1;

            /* sp_id */
            reqMsg.body.spReq.policyId =
                    (uint32_t)opt_input_gen[OPTION_ID_SP_ID].value.intval;

            /* dscp TODO */

            /* gtpu_teid */
            if (opt_input_gen[OPTION_ID_GTPU_TEID].is_valid) {
                reqMsg.body.spReq.gtpuIdRange.min =
                    (uint32_t)opt_input_gen[OPTION_ID_GTPU_TEID].value.intval;
            }

            /* num_teids */
            if (opt_input_gen[OPTION_ID_NUM_TEIDS].is_valid) {
                reqMsg.body.spReq.gtpuIdRange.max =   reqMsg.body.spReq.gtpuIdRange.min - 1 +
                    (uint32_t)opt_input_gen[OPTION_ID_NUM_TEIDS].value.intval;
            } else { /* default is 1 TEID */
                reqMsg.body.spReq.gtpuIdRange.max =   reqMsg.body.spReq.gtpuIdRange.min;
            }

            /* shared tunnel */
            if (opt_input_gen[OPTION_ID_SHARED_SA].is_valid) {
                reqMsg.body.spReq.bIsSharedPolicy   =   1;
            }

            /* Is this policy non-secure? */
            if (opt_input_gen[OPTION_ID_NONSECURE_SP].is_valid) {
                reqMsg.body.spReq.bIsSecure         =   0;
            }

#if 0
            /* Is outer IP fragmentation enabled on this tunnel? */
            if (opt_input_gen[OPTION_ID_OUTER_FRAG].is_valid) {
                reqMsg.body.spReq.fragLevel         =   NETFP_PROXY_IPSEC_FRAG_OUTER_IP;
            }
            else {
                reqMsg.body.spReq.fragLevel         =   NETFP_PROXY_IPSEC_FRAG_INNER_IP;
            }
#endif

            /* sp_name */
            if (opt_input_gen[OPTION_ID_SP_NAME].is_valid) {
                strcpy(sp_name, opt_input_gen[OPTION_ID_SP_NAME].value.strval);
            }
            else
            {
                sp_name[0] = '\0';
            }

            /* Send the offload request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_OFFLOAD_SP);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_OFFLOAD_SP,
                       shell_ctx.trans_id);
                /* Is a valid policy name specified for publishing? */
                if (sp_name[0] != '\0')
                {
                    /* Name specified. Save it to the db here so that on
                     * offload response, we can appropriately publish the name too. */
                    save_offloaded_sp (reqMsg.body.spReq.policyId, sp_name);
                }
            }
            break;
        }
        case CMD_ID_STOP_OFFLOAD_SP:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_SP_ID].is_valid) {
                printf ("Mandatory paramater missing: sp_id\n");
                goto loop_over;
            }

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_REQ;
            reqMsg.hdr.transId  =   (uint32_t)opt_input_gen[OPTION_ID_SP_ID].value.intval;

            /* sp_id */
            reqMsg.body.spReq.policyId =
                    (uint32_t)opt_input_gen[OPTION_ID_SP_ID].value.intval;

            /* Send the stop offload request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_STOP_OFFLOAD_SP);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_STOP_OFFLOAD_SP,
                       (uint32_t)opt_input_gen[OPTION_ID_SP_ID].value.intval);
            }
            break;
        }
        case CMD_ID_IP_ROUTE_FLUSH:
        {
            NetfpProxy_msg  reqMsg;

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* Send the flush request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_IP_ROUTE_FLUSH);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_IP_ROUTE_FLUSH,
                       shell_ctx.trans_id);
            }
            break;
        }
        case CMD_ID_ADD_INTERFACE:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                printf ("Mandatory paramater missing: interface_name\n");
                goto loop_over;
            }

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* interface name */
            if (opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                strcpy(reqMsg.body.ifaceAddReq.interfaceName,
                       opt_input_gen[OPTION_ID_INTERFACE_NAME].value.strval);
            }
            /* Send the interface add request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_ADD_INTERFACE);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_ADD_INTERFACE,
                       shell_ctx.trans_id);
            }
            break;
        }
        case CMD_ID_DEL_INTERFACE:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                printf ("Mandatory paramater missing: interface_name\n");
                goto loop_over;
            }
            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* interface name */
            if (opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                strcpy(reqMsg.body.ifaceDelReq.interfaceName,
                       opt_input_gen[OPTION_ID_INTERFACE_NAME].value.strval);
            }
            /* Send the interface add request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_DEL_INTERFACE);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_DEL_INTERFACE,
                       shell_ctx.trans_id);
            }
            break;
        }
        case CMD_ID_CONFIGURE_L3_QOS:
        {
            NetfpProxy_msg  reqMsg;
            uint32_t        index;

            if (!opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                printf ("Mandatory paramater missing: interface_name\n");
                goto loop_over;
            }
            if (!opt_input_gen[OPTION_ID_QOS_ENABLE].is_valid) {
                printf ("Mandatory paramater missing: qos_enable\n");
                goto loop_over;
            }
            if (!opt_input_gen[OPTION_ID_FLOW_ID].is_valid) {
                printf ("Mandatory paramater missing: flow_id\n");
                goto loop_over;
            }

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* interface name */
            if (opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                strcpy(reqMsg.body.configL3QosReq.interfaceName,
                       opt_input_gen[OPTION_ID_INTERFACE_NAME].value.strval);
            }
            /* QoS Enable */
            reqMsg.body.configL3QosReq.l3QoSCfg.isQosEnable =
                    (uint32_t)opt_input_gen[OPTION_ID_QOS_ENABLE].value.intval;

            /* Flow Id */
            reqMsg.body.configL3QosReq.l3QoSCfg.flowId =
                    (uint32_t)opt_input_gen[OPTION_ID_FLOW_ID].value.intval;

            /* By default all DSCP are mapped to the best effort queue i.e. 8072. */
            for (index = 0; index < 64; index++)
                reqMsg.body.configL3QosReq.l3QoSCfg.qid[index] = 8072;

            /***********************************************************************************
             * DSCP2  is mapped to Queue 8079 which is hp-cos7 [See the DTS]
             * DSCP63 is mapped to Queue 8010 which is wrr-cos2 [See the DTS]
             ***********************************************************************************/
            reqMsg.body.configL3QosReq.l3QoSCfg.qid[2]  = 8079;
            reqMsg.body.configL3QosReq.l3QoSCfg.qid[63] = 8074;

            printf("L3 QoS Inner DSCP to Queue Mappings are set to...\n");
            for (index = 0; index < 64; index++)
            {
                printf ("Inner DSCP %d -> Queue %d\n", index, reqMsg.body.configL3QosReq.l3QoSCfg.qid[index]);
            }

            /* Send the interface add request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_CONFIGURE_L3_QOS);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_CONFIGURE_L3_QOS,
                       shell_ctx.trans_id);
            }
            break;
        }
        case CMD_ID_FLUSH_VLAN_PRIORITY:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                printf ("Mandatory paramater missing: interface_name\n");
                goto loop_over;
            }

            memset(&reqMsg, 0, sizeof(reqMsg));

            reqMsg.hdr.msgType  =   NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_REQ;
            reqMsg.hdr.transId  =   ++shell_ctx.trans_id;

            /* interface name */
            if (opt_input_gen[OPTION_ID_INTERFACE_NAME].is_valid) {
                strcpy(reqMsg.body.flushVlanPriorityReq.interfaceName,
                       opt_input_gen[OPTION_ID_INTERFACE_NAME].value.strval);
            }
            /* Send the flush VLAN Egress priority request to NetFP proxy daemon */
            if (sendto ( cmdAppIpcSockFd,
                        (void *)&reqMsg,
                        sizeof (NetfpProxy_msg),
                        0,
                        (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_FLUSH_VLAN_PRIORITY);
            } else {
                printf("%s trans_id: 0x%x\n", CMD_NAME_FLUSH_VLAN_PRIORITY,
                       shell_ctx.trans_id);
            }
            break;
        }
        case CMD_ID_SETUP_PMTU:
        {
            NetfpProxy_msg  reqMsg;

            if (!opt_input_gen[OPTION_ID_SRC_IP].is_valid) {
                printf ("Mandatory paramater missing: srcip\n");
                goto loop_over;
            }
            if (!opt_input_gen[OPTION_ID_DST_IP].is_valid) {
                printf ("Mandatory paramater missing: dstip\n");
                goto loop_over;
            }
            if (!opt_input_gen[OPTION_ID_PMTU].is_valid) {
                printf ("Mandatory paramater missing: pmtu\n");
                goto loop_over;
            }

            /* Initialize the request message: */
            memset(&reqMsg, 0, sizeof(reqMsg));

            /* Populate the message: */
            reqMsg.hdr.msgType  = NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_REQ;
            reqMsg.hdr.transId  = ++shell_ctx.trans_id;
            memcpy ((void *)&reqMsg.body.setupPMTUReq.srcIP, (void *)&opt_input_gen[OPTION_ID_SRC_IP].value.ipAddress, sizeof(Netfp_IPAddr));
            memcpy ((void *)&reqMsg.body.setupPMTUReq.dstIP, (void *)&opt_input_gen[OPTION_ID_DST_IP].value.ipAddress, sizeof(Netfp_IPAddr));
            reqMsg.body.setupPMTUReq.pmtu = opt_input_gen[OPTION_ID_PMTU].value.intval;

            /* Send the flush VLAN Egress priority request to NetFP proxy daemon */
            if (sendto (cmdAppIpcSockFd, (void *)&reqMsg, sizeof (NetfpProxy_msg), 0, (const struct sockaddr *)&proxySunAddr,
                        sizeof (struct sockaddr_un)) < 0)
            {
                printf("%s failed\n", CMD_NAME_SETUP_PMTU);
            }
            else
            {
                printf("%s trans_id: 0x%x\n", CMD_NAME_SETUP_PMTU, shell_ctx.trans_id);
            }
            break;
        }
        default:
        {
            printf ("Unhandled switch case \n");
            break;
        }

    } /* switch cmd_id */

loop_over:
        freeparsedargs(av);
    }
    return (void*)(0);
}

void cmd_shell_offload_sp_rsp (NetfpProxy_msg* rxMsgPtr)
{
    char        sp_name[32];

    switch (rxMsgPtr->hdr.msgType)
    {
        case NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP:
        {
            printf ("\nRecvd START_OFFLOAD_SP response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);
            printf ("  sp_id:\t%d\n", rxMsgPtr->body.spResp.policyId);
            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.spResp.retVal,
                    ((rxMsgPtr->body.spResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.spResp.netfpErrCode)
                printf (" NETFP retval:\t%d\n", rxMsgPtr->body.spResp.netfpErrCode);

            if (rxMsgPtr->body.spResp.netfpSaHandle)
                printf ("  sa_handle:\t0x%x\n", rxMsgPtr->body.spResp.netfpSaHandle);

            if (rxMsgPtr->body.spResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS)
            {
                /* Start offload succeeded. Do we have a policy entry saved for this offload request? */
                if (find_offloaded_sp (rxMsgPtr->body.spResp.policyId, sp_name) < 0)
                {
                    /* No name specified for policy/policy not found. Do nothing */
                }
                else
                {
                    /* Publish this offloaded policy */
                    cmd_shell_publish_sp_rsp (NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP,
                                              rxMsgPtr->body.spResp.policyId,
                                              sp_name);
                }
            }
            else
            {
                /* Start offload failed. Clean up the policy entry if we have any saved */
                delete_offloaded_sp (rxMsgPtr->body.spResp.policyId);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP:
        {
            printf ("\nRecvd STOP_OFFLOAD_SP response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);
            printf ("  sp_id:\t%d\n", rxMsgPtr->hdr.transId);
            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.spResp.retVal,
                    ((rxMsgPtr->body.spResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.spResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS)
            {
                /* Stop offload succeeded. Do we have a policy entry saved for this offload request? */
                if (find_offloaded_sp (rxMsgPtr->hdr.transId, sp_name) < 0)
                {
                    /* No name specified for policy/policy not found. Do nothing */
                }
                else
                {
                    /* Un-publish a previously offloaded policy */
                    cmd_shell_publish_sp_rsp (NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP,
                                             rxMsgPtr->hdr.transId,
                                             sp_name);
                    /* Clean up the policy entry as well */
                    delete_offloaded_sp (rxMsgPtr->hdr.transId);
                }
            }
            else
            {
                /* Start offload failed. Do nothing */
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_IP_ROUTE_FLUSH_RESP:
        {
            printf ("\nRecvd IP_ROUTE_FLUSH response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.ipRouteFlushResp.retVal,
                    ((rxMsgPtr->body.ipRouteFlushResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.ipRouteFlushResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.ipRouteFlushResp.netfpErrCode);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_ADD_INTERFACE_RESP:
        {
            printf ("\nRecvd add_interface response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.ifaceAddResp.retVal,
                    ((rxMsgPtr->body.ifaceAddResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.ifaceAddResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.ifaceAddResp.netfpErrCode);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_DEL_INTERFACE_RESP:
        {
            printf ("\nRecvd del_interface response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.ifaceDelResp.retVal,
                    ((rxMsgPtr->body.ifaceDelResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.ifaceDelResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.ifaceDelResp.netfpErrCode);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_CONFIGURE_L3_QOS_RESP:
        {
            printf ("\nRecvd configure_l3_qos response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.configL3QosResp.retVal,
                    ((rxMsgPtr->body.configL3QosResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.configL3QosResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.configL3QosResp.netfpErrCode);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_FLUSH_VLAN_PRIORITY_RESP:
        {
            printf ("\nRecvd flush_vlan_priority response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.flushVlanPriorityResp.retVal,
                    ((rxMsgPtr->body.flushVlanPriorityResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.flushVlanPriorityResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.flushVlanPriorityResp.netfpErrCode);
            }
            break;
        }
        case NETFP_PROXY_IPC_MSGTYPE_SETUP_PMTU_RESP:
        {
            printf ("\nRecvd setup_pmtu response:\n");
            printf ("  trans_id:\t0x%x\n", rxMsgPtr->hdr.transId);

            printf ("  result:\t(%d) %s\n", rxMsgPtr->body.setupPMTUResp.retVal,
                    ((rxMsgPtr->body.setupPMTUResp.retVal == NETFP_PROXY_IPC_RETVAL_SUCCESS) ? "SUCCESS":"FAIL"));

            if (rxMsgPtr->body.setupPMTUResp.netfpErrCode) {
                printf (" NETFP lib retval:\t%d\n", rxMsgPtr->body.setupPMTUResp.netfpErrCode);
            }
            break;
        }
        default:
        {
            break;
        }
    }

    /* Done processing response */
    return;

}

int32_t cmd_shell_publish_sp_rsp (NetfpProxy_MsgType msgType, NetfpProxy_PolicyId policyId, const char* policyName)
{
    Name_ResourceCfg        namedResourceCfg;
    Name_DBHandle           nameDBHandle;
    int32_t                 errCode;

    /* Get the Name Database Handle */
    if ((nameDBHandle = Name_getDatabaseHandle (nrDBId)) == NULL)
        return -1;

    switch (msgType)
    {
        case    NETFP_PROXY_IPC_MSGTYPE_START_OFFLOAD_SP_RESP:
        {
            /* Policy offloaded succesfully. Publish the Policy Id in this domain */
            namedResourceCfg.handle1  = (uint32_t)policyId;
            strcpy (namedResourceCfg.name, policyName);

            /* Create & publish the offloaded policy Id. */
            if (Name_createResource (nameDBHandle,
                                    Name_ResourceBucket_USER_DEF1,
                                    &namedResourceCfg,
                                    &errCode) < 0)
            {
                printf ("Error: Failed creating NR for '%s' [Error code %d]\n", namedResourceCfg.name, errCode);
                return -1;
            }
            else
            {
                printf ("Start offload of '%s' policy with Id: '%d' succesfully done NameDB: %p\n",
                        policyName, policyId, nameDBHandle);
            }
            break;
        }
        case    NETFP_PROXY_IPC_MSGTYPE_STOP_OFFLOAD_SP_RESP:
        {
            /* Remove the corresponding NR for the policy from Name database. */
            if (Name_deleteResource (nameDBHandle,
                                    Name_ResourceBucket_USER_DEF1,
                                    policyName,
                                    &errCode) < 0)
            {
                printf ("Error: Failed deleting NR for '%s' [Error code %d]\n", policyName, errCode);
                return -1;
            }
            else
            {
                printf ("Stop offload of '%s' policy with Id: '%d' succesfully done\n", policyName, policyId);
            }
        }
        default:
        {
            /* Unhandled response */
            return -1;
        }
    }

    /* Return success */
    return 0;
}
