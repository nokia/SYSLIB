//*****************************************************************************
//
// Copyright 2016 Nokia, All Rights Reserved
//
//*****************************************************************************

#ifndef NETFP_JOSH_AF_UNIX_FUNCTIONS
#define NETFP_JOSH_AF_UNIX_FUNCTIONS

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <ti/apps/netfp_config/include/NetFP_System_printf.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

#define NETFP_MAX_JOSH_JOB_SIZE 4500

_Static_assert(sizeof(Netfp_ProxyServerBulkMsg) < NETFP_MAX_JOSH_JOB_SIZE, "Too big message for JOSH processing !!!");

typedef struct StaticNetfp_Josh_AF_UNIX_Packet
{
    int msgSize;
    uint8_t msg[NETFP_MAX_JOSH_JOB_SIZE];
} StaticNetfp_Josh_AF_UNIX_Packet;

static StaticNetfp_Josh_AF_UNIX_Packet staticTxBuffer;
static StaticNetfp_Josh_AF_UNIX_Packet staticRxBuffer;

static uint8_t* Netfp_JoshAlloc_af_unix (uint32_t nodeId, int32_t size, void** msgBuffer)
{
    (void)nodeId;

    staticTxBuffer.msgSize = size;
    *msgBuffer = &staticTxBuffer;

    return staticTxBuffer.msg;
}

static void Netfp_JoshFree_af_unix (uint32_t nodeId, int32_t size, void* msgBuffer)
{
    (void)nodeId; (void)size; (void)msgBuffer;
}

static void Netfp_JoshGet_af_unix (uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    (void)readerChannel;

    int sockFd = (int)nodeId;

    int receivedBytes = recv (sockFd, staticRxBuffer.msg, sizeof(staticRxBuffer.msg), 0);
    if (receivedBytes == 0)
    {
        System_printf ("JOSH: Received data zero sized datagram");
        *ptrDataBuffer = NULL;
        *msgBuffer = NULL;
        return;
    }
    else if (receivedBytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            System_printf ("JOSH: Error on recv [%d]: %s", errno, strerror(errno));

        *ptrDataBuffer = NULL;
        *msgBuffer = NULL;
        return;
    }

    *msgBuffer = &staticRxBuffer;
    *ptrDataBuffer = staticRxBuffer.msg;
    staticRxBuffer.msgSize = receivedBytes;

    return;
}

static inline void Netfp_JoshGet_af_unix_client (uint32_t nodeId, void* readerChannel, void** msgBuffer, uint8_t** ptrDataBuffer)
{
    Netfp_ClientMCB* ptrNetfpClient = (Netfp_ClientMCB*)nodeId;
    Netfp_JoshGet_af_unix(ptrNetfpClient->joshConfig.sockFd, readerChannel, msgBuffer, ptrDataBuffer);
}

static int Netfp_JoshPut_af_unix (uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    int sockFd = (int)nodeId;
    struct sockaddr_un dstAddr;
    memset (&dstAddr, 0, sizeof(dstAddr));

    dstAddr.sun_family = AF_UNIX;
    strncpy (dstAddr.sun_path, writerChannel, sizeof(dstAddr.sun_path));

    int sentBytes = 0;
    int retriesLeft = 500; // such definition expects that for busy system the receiver will free resources within 50ms

    StaticNetfp_Josh_AF_UNIX_Packet* ptrPkt = msgBuffer;
    if (ptrPkt->msgSize > NETFP_MAX_JOSH_JOB_SIZE)
    {
        System_printf ("JOSH: Error on sending josh to '%s', message too big %d bytes", dstAddr.sun_path, ptrPkt->msgSize);
        return -1;
    }

    do
    {
        sentBytes = sendto (sockFd, ptrPkt->msg, ptrPkt->msgSize, 0, (const struct sockaddr*)&dstAddr, sizeof(dstAddr));

        if (sentBytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            break;

        if(sentBytes < 0)
        {
            retriesLeft--;
            usleep(100);
        }
    } while (sentBytes < 0 && retriesLeft > 0);

    if(sentBytes <= 0)
    {
        System_printf ("JOSH: Possible critical while sending to '%s' %s", dstAddr.sun_path,  strerror(errno));

        // EAGAIN && EWOULDBLOCK may be treated as special case for error handling
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return -2;
        else
            return -1;
    }

    return 0;
}

static inline int Netfp_JoshPut_af_unix_client (uint32_t nodeId, void* writerChannel, void* msgBuffer)
{
    Netfp_ClientMCB* ptrNetfpClient = (Netfp_ClientMCB*)nodeId;
    return Netfp_JoshPut_af_unix (ptrNetfpClient->joshConfig.sockFd, writerChannel, msgBuffer);
}

#endif //NETFP_JOSH_AF_UNIX_FUNCTIONS
