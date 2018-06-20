//***************************************************************************************
//
// Copyright 2015-2016 Nokia, All Rights Reserved
//
//***************************************************************************************

#include <assert.h>
#include <FPDisp/Utils/SharedMemoryLog/SmLog.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <syslog.h>

#include <ti/apps/netfp_config/include/NetFPConfig_SharedMemoryLog.h>
#include <ti/apps/resmgr_server/include/resmgr_server.h>

#define RESMGR_SERVER_LOG_IDENT "resmgr_server"

static CSmLog syslibLogServer;

int32_t ResmgrServer_logInit(void)
{
    /* init syslib Log server as resmgr server is the first syslib run */
    syslibLogServer = SmLog_getServerInstance(SYSLIB_LOG_MEMORY_NAME, SYSLIB_LOG_MAX_SIZE_BYTES, RESMGR);
    assert(syslibLogServer != NULL);
    ResmgrServer_log(ResmgrServer_LogLevel_DEBUG, "Debug: Log server initialized\n");
    return 0;
}

int32_t ResmgrServer_logDeInit(void)
{
    return 0;
}

static inline LogLvl ResmgrServer_getSmLogLvl(ResmgrServer_LogLevel logLevel)
{
    switch (logLevel)
    {
    case ResmgrServer_LogLevel_VERBOSE:
        return VRB;
    case ResmgrServer_LogLevel_DEBUG:
        return DBG;
    case ResmgrServer_LogLevel_INFO:
        return INF;
    case ResmgrServer_LogLevel_ERROR:
        return ERR;
    default:
        return DBG;
    }
}

void ResmgrServer_smlogMsg(ResmgrServer_LogLevel logLevel, const char* fmt, va_list arg)
{
    SmLog_vlogf(syslibLogServer, ResmgrServer_getSmLogLvl(logLevel), RESMGR_SERVER_LOG_IDENT, fmt, arg);
}

static FILE* dumpFile;
static sig_atomic_t dumpingInProgress = 0;

int ResmgrServer_dumpInit(void)
{
    if(dumpingInProgress)
    {
        SmLog_logf(syslibLogServer, ERR, RESMGR_SERVER_LOG_IDENT, "Dumpfile already open");
        return -1;
    }
    dumpingInProgress = 1;

    dumpFile = fopen("/tmp/" RESMGR_SERVER_LOG_IDENT ".log", "w");
    if(!dumpFile)
    {
        SmLog_logf(syslibLogServer, ERR, RESMGR_SERVER_LOG_IDENT, "Opening dumpfile failed with error: %m");
        dumpingInProgress = 0;
        return -1;
    }

    return 0;
}

void ResmgrServer_dumpDeInit(void)
{
    if (dumpFile)
        fclose(dumpFile);
    dumpFile = NULL;

    dumpingInProgress = 0;
}

void ResmgrServer_dumpMsg(const char* fmt, va_list arg)
{
    if (dumpFile)
        vfprintf(dumpFile, fmt, arg);
}

void Osal_syslibLoggingFunction(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    static bool syslogOpened = false;

    if(syslibLogServer)
        ResmgrServer_smlogMsg(ResmgrServer_LogLevel_INFO, format, ap);
    else
    {
        if(!syslogOpened)
        {
            syslogOpened = true;
            openlog(RESMGR_SERVER_LOG_IDENT, LOG_CONS | LOG_PID | LOG_NDELAY, LOG_USER);
        }
        vsyslog(LOG_INFO, format, ap);
    }

    va_end(ap);
}

void ResmgrServer_setLogLevel(ResmgrServer_LogLevel level)
{
    gResmgrServerMCB.logLevel = level;
}

ResmgrServer_LogLevel ResmgrServer_getLogLevel(void)
{
    return gResmgrServerMCB.logLevel;
}
