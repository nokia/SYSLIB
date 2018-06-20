//***************************************************************************************
//
// Copyright 2016 Nokia, All Rights Reserved
//
//***************************************************************************************

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <syslog.h>
#include <signal.h>

#include <FPDisp/Utils/SharedMemoryLog/SmLog.h>
#include <ti/apps/netfp_config/include/NetFPConfig_SharedMemoryLog.h>
#include <ti/apps/netfp_proxy/netfp_proxy.h>

#define NETFP_PROXY_LOG_IDENT "netfp_proxy"

static CSmLog logClient;
static NetfpProxy_LogLevel default_logLevel = NETFP_PROXY_LOG_DEBUG;

int32_t NetfpProxy_logInit(void)
{
    openlog(NETFP_PROXY_LOG_IDENT, LOG_CONS, LOG_USER);
    logClient = SmLog_getClientInstance(SYSLIB_LOG_MEMORY_NAME, NETFP_PROXY);
    assert(logClient != NULL);
    NetfpProxy_logMsg (NETFP_PROXY_LOG_DEBUG, "Debug: Log client initialized\n");
    return 0;
}

int32_t NetfpProxy_logDeInit(void)
{
    closelog();
    return 0;
}

void NetfpProxy_setLogLevel(NetfpProxy_LogLevel level)
{
    default_logLevel = level;
}

NetfpProxy_LogLevel NetfpProxy_getLogLevel(void)
{
    return default_logLevel;
}

static inline LogLvl NetfpProxy_getSmLogLvl(NetfpProxy_LogLevel logLevel)
{
    switch (logLevel)
    {
    case NETFP_PROXY_LOG_VRB:
        return VRB;
    case NETFP_PROXY_LOG_DEBUG:
        return DBG;
    case NETFP_PROXY_LOG_INFO:
        return INF;
    case NETFP_PROXY_LOG_ERROR:
        return ERR;
    default:
        return DBG;
    }
}

static inline int NetfpProxy_getSysLogLvl(NetfpProxy_LogLevel logLevel)
{
    int syslogLevel;
    switch (logLevel)
    {
    case NETFP_PROXY_LOG_DEBUG:
        syslogLevel = LOG_DEBUG;
        break;
    case NETFP_PROXY_LOG_INFO:
        syslogLevel = LOG_INFO;
        break;
    case NETFP_PROXY_LOG_ERROR:
        syslogLevel = LOG_ERR;
        break;
    default:
        syslogLevel = LOG_DEBUG;
        break;
    }
    return syslogLevel;
}

void NetfpProxy_logMsgPlug(NetfpProxy_LogLevel level, const char* fmt, va_list ap)
{
    /* Log the message as per the configured level. */
    if (default_logLevel >= level)
    {
        SmLog_vlogf(logClient, NetfpProxy_getSmLogLvl(level), NETFP_PROXY_LOG_IDENT, fmt, ap);
    }
}

void NetfpProxy_logMsg(NetfpProxy_LogLevel level, const char* fmt, ...)
{
    /* Log the message as per the configured level. */
    if (default_logLevel >= level)
    {
        va_list ap;

        va_start(ap, fmt);
        SmLog_vlogf(logClient, NetfpProxy_getSmLogLvl(level), NETFP_PROXY_LOG_IDENT, fmt, ap);
        va_end(ap);
    }
}

void NetfpProxy_syslogMsg(NetfpProxy_LogLevel level, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsyslog(NetfpProxy_getSysLogLvl(level), fmt, ap);
    va_end(ap);
}

static FILE* dumpFile;
static sig_atomic_t dumpingInProgress = 0;

int NetfpProxy_dumpInit(void)
{
    if(dumpingInProgress)
    {
        SmLog_logf(logClient, ERR, NETFP_PROXY_LOG_IDENT, "Dumpfile already open");
        return -1;
    }
    dumpingInProgress = 1;

    dumpFile = fopen("/tmp/" NETFP_PROXY_LOG_IDENT ".log", "w");
    if(!dumpFile)
    {
        SmLog_logf(logClient, ERR, NETFP_PROXY_LOG_IDENT, "Opening dumpfile failed with error: %m");
        dumpingInProgress = 0;
        return -1;
    }

    return 0;
}

void NetfpProxy_dumpDeInit(void)
{
    if (dumpFile)
        fclose(dumpFile);
    dumpFile = NULL;

    dumpingInProgress = 0;
}

void NetfpProxy_dumpMsg(NetfpProxy_LogLevel level, const char* fmt, ...)
{
    (void)level;

    if (!dumpFile)
        return;

    va_list ap;
    va_start(ap, fmt);
    vfprintf(dumpFile, fmt, ap);
    va_end(ap);
}

void Osal_syslibLoggingFunction(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);

    if(logClient)
        NetfpProxy_logMsgPlug(NETFP_PROXY_LOG_INFO, (char*)format, ap);
    else
        vprintf(format, ap);

    va_end(ap);
}
