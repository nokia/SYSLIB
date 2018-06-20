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
/* Standard library includes */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <string.h>

/* Command shell local includes */
#include "cmd_shell_loc.h"

/* Global variable definitions */
pthread_t                   cmdShellTaskHandle, ipcTaskHandle;
struct sockaddr_un          cmdAppSunAddr;
struct sockaddr_un          proxySunAddr;
int32_t                     cmdAppIpcSockFd, nrDBId;

/**
 *  @b Description
 *  @n
 *      Applications's IPC message handler.
 *
 *      Polls for responses from Proxy plugin and posts it to
 *      the command shell.
 *
 *  @retval
 *      Not applicable
 */
#define IPC_POLL_INTVL_MSEC 1000
static void* ipc_poll (void* args)
{
    NetfpProxy_msg      rxMsg;
    struct sockaddr_un  fromAddr;
    int32_t             msgLen      =   sizeof (NetfpProxy_msg);
    uint32_t            destAddrLen =   sizeof (struct sockaddr_un);

    /* Keep waiting on messages from the Proxy */
    while (1)
    {
        if (recvfrom (cmdAppIpcSockFd,
                      (void *)&rxMsg,
                      msgLen,
                      0,
                      (struct sockaddr *)&fromAddr,
                      &destAddrLen) > 0)
        {
            /* Got a response from NetFP Proxy. Parse it. */
#if 0
            printf ("Sock: %d Type: %d xid: %d addr: %s\n", cmdAppIpcSockFd,
                    rxMsg.hdr.msgType, rxMsg.hdr.transId, fromAddr.sun_path);
#endif
            cmd_shell_offload_sp_rsp (&rxMsg);
        }
        else
        {
            printf ("Rx error?? \n");
        }
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Entry point for the Command shell application.
 *
 *  @param[in]  argc
 *      Number of arguments passed to the application
 *  @param[in]  argv
 *      Arguments passed to the application.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      ERROR   -   >0
 */
int32_t main (int argc, char **argv)
{
    int32_t             retVal, proxyPid, errCode;
    char                sunPath[64];
    Name_DatabaseCfg    nameDBConfig;
    char                dbOwner [NAME_MAX_CHAR + 1];
    Name_DBHandle       nameDBHandle;

    /* Validate input parameters */
    if (argc < 3)
    {
        printf ("%s <netfpproxy_pid> <named_res_id> \n", argv[0]);
        return -1;
    }

    /* Get the NetFP Proxy PID that this instance of management application will manage */
    proxyPid = atoi (argv[1]);

    /* Get the Name Resource Database Id with which this application needs to communicate */
    nrDBId = atoi (argv[2]);

    /*  Create a Name Database first to share with Root slaves.
     *  This database will be used to propagate the policies
     *  to be used for L2 stack/FAPI operation on Root slaves */
    memset ((void *)&nameDBConfig, 0, sizeof (Name_DatabaseCfg));
    nameDBConfig.instanceId     =   nrDBId;
    nameDBConfig.realm          =   Name_ExecutionRealm_ARM;
    snprintf (dbOwner, NAME_MAX_CHAR, "OAM_LTE%d", nrDBId);
    strncpy (nameDBConfig.owner, dbOwner, strlen (dbOwner));
    if ((nameDBHandle = Name_createDatabase (&nameDBConfig, &errCode)) == NULL)
    {
        printf ("Error: Name database creation failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Open a Unix socket to talk to NetFP Proxy daemon */
    if ((cmdAppIpcSockFd = socket (AF_UNIX, SOCK_DGRAM, 0)) < 0)
    {
        printf ("Socket open failed, error: %d \n", errno);
        return -1;
    }

    /* Setup the socket to listen on messages from NetFP Proxy. */
    memset ((void *)&cmdAppSunAddr, 0, sizeof (struct sockaddr_un));
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_USER_PATH, proxyPid);
    strcpy (cmdAppSunAddr.sun_path, sunPath);

    /* Unlink the socket path to ensure this is the only socket instance
     * running */
    unlink (cmdAppSunAddr.sun_path);

    /* Bind the socket to our unix path name */
    cmdAppSunAddr.sun_family    =   AF_UNIX;
    if (bind (cmdAppIpcSockFd, (const struct sockaddr *)&cmdAppSunAddr, sizeof (struct sockaddr_un)) < 0)
    {
        printf ("Socket bind failed, error: %d \n", errno);
        close (cmdAppIpcSockFd);
        return -1;
    }

    /* Also setup socket address for NetFP Proxy daemon. Will be
     * used to send messages to the daemon */
    memset ((void *)&proxySunAddr, 0, sizeof (struct sockaddr_un));
    proxySunAddr.sun_family =   AF_UNIX;
    snprintf (sunPath, sizeof(sunPath), "%s/%s_%d", Syslib_getRunTimeDirectory(), NETFP_PROXY_IPC_DEFAULT_DAEMON_PATH, proxyPid);
    strcpy (proxySunAddr.sun_path, sunPath);

    /* Create the command line interpreter task */
    if ((retVal = pthread_create (&cmdShellTaskHandle, NULL, cmd_shell, NULL)) < 0)
    {
        printf ("ERROR: Command Shell Thread failed to start, error code %d \n", retVal);
        close (cmdAppIpcSockFd);
        return -1;
    }

    /* Create the CmdShell - NetFP Proxy IPC receive task */
    if ((retVal = pthread_create (&ipcTaskHandle, NULL, ipc_poll, NULL)) < 0)
    {
        printf ("ERROR: Command Shell IPC thread failed to start, error code %d \n", retVal);
        close (cmdAppIpcSockFd);
        return -1;
    }

    /* Wait for the tasks to complete */
    pthread_join (ipcTaskHandle, NULL);
    pthread_join (cmdShellTaskHandle, NULL);

    /* Cleanup and return */
    close (cmdAppIpcSockFd);

    /* exit cleanly */
    return 0;
}

