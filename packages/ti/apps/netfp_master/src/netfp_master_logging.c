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
#include <ti/apps/netfp_master/include/netfp_master_internal.h>

#define NETFP_MASTER_LOG_IDENT "netfp_master"

static CSmLog logClient;

int32_t NetfpMaster_logInit(void)
{
    logClient = SmLog_getClientInstance(SYSLIB_LOG_MEMORY_NAME, NETFP_MASTER);
    if(logClient == NULL)
        return -1;
    NetfpMaster_log(NetfpMaster_LogLevel_INFO, "Info: Log client initialized\n");
    return 0;
}

int32_t NetfpMaster_logDeInit(void)
{
    return 0;
}

static inline LogLvl NetfpMaster_getSmLogLvl(NetfpMaster_LogLevel logLevel)
{
    switch (logLevel)
    {
    case NetfpMaster_LogLevel_VERBOSE:
        return VRB;
    case NetfpMaster_LogLevel_DEBUG:
        return DBG;
    case NetfpMaster_LogLevel_INFO:
        return INF;
    case NetfpMaster_LogLevel_ERROR:
        return ERR;
    default:
        return DBG;
    }
}

void NetfpMaster_smlogMsg(NetfpMaster_LogLevel logLevel, const char* fmt, va_list arg)
{
    SmLog_vlogf(logClient, NetfpMaster_getSmLogLvl(logLevel), NETFP_MASTER_LOG_IDENT, fmt, arg);
}

static FILE* dumpFile;
static sig_atomic_t dumpingInProgress = 0;

int NetfpMaster_dumpInit(void)
{
    if(dumpingInProgress)
    {
        SmLog_logf(logClient, ERR, NETFP_MASTER_LOG_IDENT, "Dumpfile already open");
        return -1;
    }
    dumpingInProgress = 1;

    dumpFile = fopen("/tmp/" NETFP_MASTER_LOG_IDENT ".log", "w");
    if(!dumpFile)
    {
        SmLog_logf(logClient, ERR, NETFP_MASTER_LOG_IDENT, "Opening dumpfile failed with error: %m");
        dumpingInProgress = 0;
        return -1;
    }

    return 0;

}

void NetfpMaster_dumpDeInit(void)
{
    if (dumpFile)
        fclose(dumpFile);
    dumpFile = NULL;

    dumpingInProgress = 0;
}

void NetfpMaster_dumpMsg(Netfp_LogLevel logLevel, const char* fmt, va_list arg)
{
    (void)logLevel;
    if(dumpFile)
        vfprintf(dumpFile, fmt, arg);
}

void Osal_syslibLoggingFunction(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);

    if(logClient)
        NetfpMaster_smlogMsg(NetfpMaster_LogLevel_INFO, (char*)format, ap);
    else
        vprintf(format, ap);

    va_end(ap);
}
