//***************************************************************************************
//
// Copyright 2015-2016 Nokia, All Rights Reserved
//
//***************************************************************************************

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>

#include <FPDisp/Utils/SharedMemoryLog/SmLog.h>
#include <ti/apps/netfp_config/include/NetFPConfig_SharedMemoryLog.h>
#include <ti/runtime/netfp/netfp.h>
#include <ti/apps/netfp_server/include/netfp_server.h>

#define NETFP_SERVER_LOG_IDENT "netfp_server"

extern NetfpServer_MCB gNetfpServerMCB;

static CSmLog logClient;

int32_t Server_logInit (void)
{
    logClient = SmLog_getClientInstance(SYSLIB_LOG_MEMORY_NAME, NETFP_SERVER);
    assert(logClient != NULL);
    NetfpServer_log(NetfpServer_LogLevel_DEBUG, "Debug: Log client initialized\n");
    return 0;
}

int32_t Server_logDeInit (void)
{
    return 0;
}

static inline LogLvl NetfpServer_getSmLogLvl(NetfpServer_LogLevel logLevel)
{
    switch (logLevel)
    {
    case NetfpServer_LogLevel_VERBOSE:
        return VRB;
    case NetfpServer_LogLevel_DEBUG:
        return DBG;
    case NetfpServer_LogLevel_INFO:
        return INF;
    case NetfpServer_LogLevel_ERROR:
        return ERR;
    default:
        return DBG;
    }
}

static inline NetfpServer_LogLevel NetfpServer_parseNetfpLogLvl(Netfp_LogLevel logLevel)
{
    switch (logLevel)
    {
    case Netfp_LogLevel_VERBOSE:
        return NetfpServer_LogLevel_VERBOSE;
    case Netfp_LogLevel_DEBUG:
        return NetfpServer_LogLevel_DEBUG;
    case Netfp_LogLevel_INFO:
        return NetfpServer_LogLevel_INFO;
    case Netfp_LogLevel_ERROR:
        return NetfpServer_LogLevel_ERROR;
    default:
        return NetfpServer_LogLevel_DEBUG;
    };
}

void NetfpServer_smlogMsg(NetfpServer_LogLevel logLevel, const char* fmt, va_list arg)
{
    /* Log the message as per the configured level. This function may be run directly from netfp library. */
    if (logLevel >= gNetfpServerMCB.logLevel)
    {
        SmLog_vlogf(logClient, NetfpServer_getSmLogLvl(logLevel), NETFP_SERVER_LOG_IDENT, fmt, arg);
    }
}

void Netfp_LoggingFxn(Netfp_LogLevel logLevel, const char* fmt, va_list arg)
{
    NetfpServer_smlogMsg(NetfpServer_parseNetfpLogLvl(logLevel), fmt, arg);
}

static FILE* dumpFile;
static sig_atomic_t dumpingInProgress = 0;

int NetfpServer_dumpInit(void)
{
    if(dumpingInProgress)
    {
        SmLog_logf(logClient, ERR, NETFP_SERVER_LOG_IDENT, "Dumpfile already open");
        return -1;
    }
    dumpingInProgress = 1;

    dumpFile = fopen("/tmp/" NETFP_SERVER_LOG_IDENT ".log", "w");
    if(!dumpFile)
    {
        SmLog_logf(logClient, ERR, NETFP_SERVER_LOG_IDENT, "Opening dumpfile failed with error: %m");
        dumpingInProgress = 0;
        return -1;
    }

    return 0;
}

void NetfpServer_dumpDeInit(void)
{
    if (dumpFile)
        fclose(dumpFile);
    dumpFile = NULL;

    dumpingInProgress = 0;
}

void Netfp_DumpingFxn(Netfp_LogLevel logLevel, const char* fmt, va_list arg)
{
    if (dumpFile)
        vfprintf(dumpFile, fmt, arg);
}

void Osal_syslibLoggingFunction(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);

    if(logClient)
        NetfpServer_smlogMsg(Netfp_LogLevel_INFO, (char*)format, ap);
    else
        vprintf(format, ap);

    va_end(ap);
}
