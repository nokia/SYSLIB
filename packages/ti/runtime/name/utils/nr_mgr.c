/**
 *   @file  nr_mgr.c
 *
 *   @brief   
 *      Named resource manager utility. This is provided as utility which
 *      can help manage named resources on ARM. This is useful during 
 *      debug sessions. This should *NOT* be used as an interface to handle
 *      the management of named resources. Please use the exported C API
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2104 Texas Instruments, Inc.
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
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>

/* SYSLIB Include Files */
#include <ti/runtime/name/name.h>
#include <ti/runtime/name/name_db.h>

/**********************************************************************
 ************************ Local Defintions ****************************
 **********************************************************************/

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/**********************************************************************
 ******************* Named Resource Manager Functions *****************
 **********************************************************************/

static inline void LOG_ERROR (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_RED);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

static inline void LOG_DEBUG (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_RESET);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

static inline void LOG_INFO (char* fmt, ...)
{
    va_list arg;

    va_start (arg, fmt);
    printf ("%s", ANSI_COLOR_GREEN);
    vprintf (fmt, arg);
    printf ("%s", ANSI_COLOR_RESET);
    va_end (arg);
}

/**
 *  @b Description
 *  @n  
 *      The function process the command line arguments passed to the applicaton
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success 	- Named resource instance identifier
 *  @retval
 *      Error		- <0
 */
static int32_t NRMgr_processCmdLineArgs(int32_t argc, char* argv[])
{
    int32_t nrInstanceId = -1;

    while (1)
    {
        int option_index = 0;
        int c;

        static struct option long_options[] = {
            {"instantId",       required_argument, 0,  0 },
            {0,                 0,                 0,  0 }
        };

        c = getopt_long(argc, argv, "i:", long_options, &option_index);
        if (c == -1)
            break;

       switch (c) 
       {
            case 'i':
            {
                /* Named resource instance identifier. */
                nrInstanceId = atoi (optarg);
                break;
            }
            default:
            {
                return -1;
            }
        }
    }
    /* Return the named resource instance identifier. */
    return nrInstanceId;
}

/**
 *  @b Description
 *  @n  
 *      The function gets the user input from the console and returns it
 *      back to the callee 
 *
 *  @retval
 *      Input selection
 */
static uint8_t NRMgr_getUserInput(void)
{
    char    userInput[256];
    uint8_t selection;

    /* Read the user input from the console. */
    fgets(&userInput[0], sizeof(userInput), stdin);

    /* Get the selection from the user input */
    sscanf (userInput, "%c\n", &selection);

    return selection;
}

/**
 *  @b Description
 *  @n  
 *      The function displays the menu for the named resource type and 
 *      returns the selection.
 *
 *  @retval
 *      Named Resource Type selection
 */
static uint8_t NRMgr_menuNamedResourceType(void)
{
    uint8_t selection;

    /* Selection: */
    printf ("**********************************************************\n");
    printf ("Named Resource type selection menu:\n");
    printf ("**********************************************************\n");
    printf ("1. Internal SYSLIB \n");
    printf ("2. User Def 1\n");
    printf ("3. User Def 2\n");
    printf ("4. User Def 3\n");
    printf ("5. User Def 4\n");
    printf ("Please enter the named resource type:");
    selection = NRMgr_getUserInput();

    return selection;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to display the named resources for a specific
 *      type.
 *
 *  @param[in]  databaseHandle
 *      Database Handle
 *
 *  @retval
 *      Not applicable
 */
static void NRMgr_displayNamedResource(Name_DBHandle databaseHandle)
{
    uint8_t selection;
    int32_t errCode;
    int32_t result;

    /* Get the named resource type selection. */
    selection = NRMgr_menuNamedResourceType ();

    /* Process based on the selection menu */
    switch (selection)
    {
        case '1':
        {
            printf ("******************************************************\n");
            printf ("Internal SYSLIB Dump:\n");
            printf ("******************************************************\n");
            result = Name_dumpDatabase (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &errCode);
            break;
        }
        case '2':
        {
            printf ("******************************************************\n");
            printf ("USER DEF1 Dump:\n");
            printf ("******************************************************\n");
            result = Name_dumpDatabase (databaseHandle, Name_ResourceBucket_USER_DEF1, &errCode);
            break;
        }
        case '3':
        {
            printf ("******************************************************\n");
            printf ("USER DEF2 Dump:\n");
            printf ("******************************************************\n");
            result = Name_dumpDatabase (databaseHandle, Name_ResourceBucket_USER_DEF2, &errCode);
            break;
        }
        case '4':
        {
            printf ("******************************************************\n");
            printf ("USER DEF3 Dump:\n");
            printf ("******************************************************\n");
            result = Name_dumpDatabase (databaseHandle, Name_ResourceBucket_USER_DEF3, &errCode);
            break;
        }
        case '5':
        {
            printf ("******************************************************\n");
            printf ("USER DEF4 Dump:\n");
            printf ("******************************************************\n");
            result = Name_dumpDatabase (databaseHandle, Name_ResourceBucket_USER_DEF4, &errCode);
            break;
        }
    }

    if (result < 0)
        LOG_ERROR ("Error: Dump Database failed [Error code %d]\n", errCode);

    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to find the named resource in the specific named resource
 *      database
 *
 *  @param[in]  databaseHandle
 *      Database Handle
 *
 *  @retval
 *      Not applicable
 */
static void NRMgr_findNamedResource(Name_DBHandle databaseHandle)
{
    uint8_t                 selection;
    Name_ResourceCfg        namedResourceCfg;
    int32_t                 errCode = 0;
    int32_t                 result  = 0;
    char                    name[NAME_MAX_CHAR];

    /* Get the named resource type selection. */
    selection = NRMgr_menuNamedResourceType ();

    /* Get the name */
    printf ("Name :");
    fgets (&name[0], sizeof(name), stdin);
    strtok (&name[0], "\n\r"); 

    switch (selection)
    {
        case '1':
        {
            result= Name_findResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &name[0], &namedResourceCfg, &errCode);
            break;
        }
        case '2':
        {
            result= Name_findResource (databaseHandle, Name_ResourceBucket_USER_DEF1, &name[0], &namedResourceCfg, &errCode);
            break;
        }
        case '3':
        {
            result= Name_findResource (databaseHandle, Name_ResourceBucket_USER_DEF2, &name[0], &namedResourceCfg, &errCode);
            break;
        }
        case '4':
        {
            result= Name_findResource (databaseHandle, Name_ResourceBucket_USER_DEF3, &name[0], &namedResourceCfg, &errCode);
            break;
        }
        case '5':
        {
            result= Name_findResource (databaseHandle, Name_ResourceBucket_USER_DEF4, &name[0], &namedResourceCfg, &errCode);
            break;
        }
        default:
        {
            return;
        }
    }

    /* Did the named resource succeed? */
    if (result < 0)
    {
        /* Named Resource Find failed: Use the error code to determine the exact reason for the failure */
        if (errCode == NAME_ENOTFOUND)
        {
            /* Named resource entry does not exist. */
            LOG_ERROR ("Resource does not exist!\n");
        }
        else
        {
            LOG_ERROR ("FATAL Error: Named resource lookup failed [Error code %d]\n", errCode);
        }
    }
    else
    {
        /* Named resource lookup was successful */
        LOG_INFO ("Name      : %s\n",   namedResourceCfg.name);
        LOG_INFO ("Handle1   : 0x%x\n", namedResourceCfg.handle1);
        LOG_INFO ("Handle2   : 0x%x\n", namedResourceCfg.handle2);
        LOG_INFO ("Handle3   : 0x%x\n", namedResourceCfg.handle3);
        LOG_INFO ("Handle4   : 0x%x\n", namedResourceCfg.handle4);
        LOG_INFO ("Handle5   : 0x%x\n", namedResourceCfg.handle5);
        LOG_INFO ("Handle6   : 0x%x\n", namedResourceCfg.handle6);
        LOG_INFO ("Handle7   : 0x%x\n", namedResourceCfg.handle7);
        LOG_INFO ("Handle8   : 0x%x\n", namedResourceCfg.handle8);
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to delete the named resource in the specific named resource
 *      database
 *
 *  @param[in]  databaseHandle
 *      Database Handle
 *
 *  @retval
 *      Not applicable
 */
static void NRMgr_deleteNamedResource(Name_DBHandle databaseHandle)
{
    uint8_t                 selection;
    int32_t                 errCode = 0;
    int32_t                 result  = 0;
    char                    name[NAME_MAX_CHAR];

    /* Get the named resource type selection. */
    selection = NRMgr_menuNamedResourceType ();

    /* Get the name */
    printf ("Name :");
    fgets (&name[0], sizeof(name), stdin);
    strtok (&name[0], "\n\r"); 

    switch (selection)
    {
        case '1':
        {
            result= Name_deleteResource (databaseHandle, Name_ResourceBucket_INTERNAL_SYSLIB, &name[0], &errCode);
            break;
        }
        case '2':
        {
            result= Name_deleteResource (databaseHandle, Name_ResourceBucket_USER_DEF1, &name[0], &errCode);
            break;
        }
        case '3':
        {
            result= Name_deleteResource (databaseHandle, Name_ResourceBucket_USER_DEF2, &name[0], &errCode);
            break;
        }
        case '4':
        {
            result= Name_deleteResource (databaseHandle, Name_ResourceBucket_USER_DEF3, &name[0], &errCode);
            break;
        }
        case '5':
        {
            result= Name_deleteResource (databaseHandle, Name_ResourceBucket_USER_DEF4, &name[0], &errCode);
            break;
        }
        default:
        {
            return;
        }
    }

    /* Did the named resource succeed? */
    if (result < 0)
    {
        /* Named Resource Find failed: Use the error code to determine the exact reason for the failure */
        if (errCode == NAME_ENOTFOUND)
        {
            /* Named resource entry does not exist. */
            LOG_ERROR ("Resource does not exist!\n");
        }
        else
        {
            LOG_ERROR ("FATAL Error: Named resource delete failed [Error code %d]\n", errCode);
        }
    }
    else
    {
        /* Named resource lookup was successful */
        LOG_INFO ("Named resource deleted successfully!\n");
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      Entry point into the named resource manager
 *
 *  @param[in]  argc
 *      Number of arguments.
 *  @param[in]  argv
 *      Command Line arguments.
 *
 *  @retval
 *      Success 	- 0
 *  @retval
 *      Error		- <0
 */
int32_t main (int32_t argc, char* argv[])
{
    Name_DBHandle               databaseHandle;
    Name_DatabaseCfg            databaseCfg;
    int32_t                     nrInstanceId;
    int32_t                     selection;
    int32_t                     errCode;

    /* Sanity Check: Validate the arguments */
    nrInstanceId = NRMgr_processCmdLineArgs (argc, argv);
    if (nrInstanceId < 0)
    {
        printf ("Usage: nr_mgr -i <named resource id>\n");
        exit(-1);
    }

    /* Initialize the named resource instance configuration. */
    memset ((void *)&databaseCfg, 0, sizeof(Name_DatabaseCfg));

    /* Populate the configuration. */
    databaseCfg.instanceId = nrInstanceId;
    databaseCfg.realm      = Name_ExecutionRealm_ARM;
    strcpy (databaseCfg.owner, "NR_MGR");

    /* Initialize the named resource domain instance */
    databaseHandle = Name_createDatabase (&databaseCfg, &errCode);
    if (databaseHandle == NULL)
    {
        LOG_ERROR("Error: Initialization of named resource domain failed [Error code %d]\n", errCode);
        return -1;
    }

    while (1)
    { 
        printf ("**********************************************************\n");
        printf ("Named Resource Manager for ID %d\n", nrInstanceId);
        printf ("**********************************************************\n");
        printf ("1. Display named resource\n");
        printf ("2. Delete named resource\n");
        printf ("3. Find named resource\n");
        printf ("4. Exit\n");
        printf ("Enter selection:");
        selection = NRMgr_getUserInput();

        switch (selection)
        {
            case '1':
            {
                NRMgr_displayNamedResource(databaseHandle);
                break;
            }
            case '2':
            {
                NRMgr_deleteNamedResource(databaseHandle);
                break;
            }
            case '3':
            {
                NRMgr_findNamedResource(databaseHandle);
                break;
            }
            case '4':
            {
                exit (0);
            }
        }
    }
    return 0;
}

