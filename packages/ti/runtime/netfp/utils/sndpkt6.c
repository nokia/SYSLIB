/*
 *  sndpkt6.c
 *
 * Send packet utility
 *
 * Copyright (C) 2008 Texas Instruments Incorporated - http://www.ti.com/
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


/**************************************************************************
 *************************** Include Files ********************************  
 **************************************************************************/

#ifdef _WIN32
#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <winsock.h>
#else
/* Standard include files */
#include <stdint.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#endif

#ifndef _WIN32
typedef int32_t	SOCKET;
#endif

/**********************************************************************
 ************************* Local Definitions **************************  
 **********************************************************************/

/* Standard GTPU Definitions. */
#define GTPU_PORT       2152
#define GTPHDR_SIZE     8

/**********************************************************************
 ************************* Local Structures ***************************  
 **********************************************************************/

typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned int    uint32_t;
typedef int             int32_t;
typedef short           int16_t;

typedef struct GTPUHDR
{
    uint8_t    Flags;
    uint8_t    MsgType;
    uint16_t   TotalLen;
    uint32_t   TunnelId;
}GTPUHDR;

/**********************************************************************
 ************************* SNDPKT Functions ***************************  
 **********************************************************************/

#ifndef _WIN32

int32_t WSAGetLastError(void)
{
    return errno;
}

#endif

/**
 *  @b Description
 *  @n  
 *      Prints the usage on the console.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static void sndPkt_printUsage(void)
{
    printf("--------------------------------------------------------------------------------\n");
    printf("Version 1.0                                                                     \n");
    printf("Usage: SNDPKT6 <IPv6 addr> <port no> <num packets> <packet size> [0x<gtpuId>]   \n");
    printf(" - If the port no is 2152 then the parameter GTPU id also needs to be specified \n");
    printf(" - The packet size is the actual size of the data which is being sent.          \n");
    printf(" - If packet size and num packets are both set to -1, then packets of varying   \n");
    printf("   sizes are sent in an infinite loop.                                          \n");
    printf("--------------------------------------------------------------------------------\n");
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create a local socket.
 *
 *  @retval
 *      Success     -   Socket Handle
 *  @retval
 *      Error       -   <0
 */
static SOCKET sndPkt_createLocalSocket (void) {
    SOCKET              sLocalSocket;
    struct sockaddr_in6 sin;

	/* Open the socket. */
    sLocalSocket = socket( AF_INET6, SOCK_DGRAM, 0 );
    if (sLocalSocket < 0 )
    {
        printf("failed socket (%d)\n",WSAGetLastError());
        return -1;
    }

	/* Initiliaze the sock address structure, */
	memset ((void *)&sin, 0, sizeof(struct sockaddr_in6));
    sin.sin6_family      = AF_INET6;
    //sin.sin6_port        = htons(0);
    sin.sin6_port        = htons(2048);

	/* Bind the socket */
	if (bind (sLocalSocket, (struct sockaddr *)&sin, sizeof(sin)) < 0)
    {
		printf ("Error: bind failed (%d)\n", WSAGetLastError());
		return -1;
    }

    /* Socket was created successfully. */
    return sLocalSocket;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to create the receive socket.
 *
 *  @param[in]  isGTPUSocket 
 *      Flag which indicates if the packet is a GTPU socket or not
 *
 *  @retval
 *      Success     -   Socket Handle
 *  @retval
 *      Error       -   <0
 */
static SOCKET sndPkt_createRecvSocket (uint8_t isGTPUSocket, uint16_t portNumber) 
{
    SOCKET              sRecvSocket;
    struct sockaddr_in6 sin;

	/* Open the socket. */
    sRecvSocket = socket( AF_INET6, SOCK_DGRAM, 0 );
    if (sRecvSocket < 0 )
    {
        printf("failed socket (%d)\n",WSAGetLastError());
        return -1;
    }

	/* Initiliaze the sock address structure, */
	memset ((void *)&sin, 0, sizeof(struct sockaddr_in6));
    sin.sin6_family      = AF_INET6;

    /* For GTPU Sockets we are waiting for a packet to arrive on 2152 */
    if (isGTPUSocket)
        sin.sin6_port = htons(2152);
    else
        sin.sin6_port = htons(portNumber);

	/* Bind the socket */
	if (bind (sRecvSocket, (struct sockaddr *)&sin, sizeof(sin)) < 0)
    {
		printf ("Error: bind failed (%d)\n", WSAGetLastError());
		return -1;
    }

    /* Debug Message: */
    printf ("Debug: Receive socket has been created on port %d\n", ntohs(sin.sin6_port));

    /* Socket was created successfully. */
    return sRecvSocket;
}

/**
 *  @b Description
 *  @n  
 *      Entry Point for the send packet utility.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int main (int argc, char* argv[])
{
#ifdef _WIN32
    WORD                wVersionRequested;
    WSADATA             wsaData;
#endif
    uint32_t            numPackets;
    int32_t             tmp1,tmp2,tmp3,tmp4;
	int32_t             port_number;
    uint32_t            gtpuId = 0;
    SOCKET              sLocalSocket;
    SOCKET              sRecvSocket;    
    char*               dataBuffer;
    int32_t             index;
    struct sockaddr_in6 to;
    struct sockaddr_in6 from;
    GTPUHDR*            ptrGTPUHeader;
    uint8_t*            ptrDataPayload;
    uint32_t            data_payload_len;
    uint32_t            packetCount = 0;
    int32_t             packetSize;
    int32_t             txPacketSize;
    int32_t             rxPacketSize;
    uint8_t             isVaryingSize = 0;
    uint8_t             isTestDone = 0;
    char				ipv6DstAddress[200];

    /* Sanity Check: Ensure that the command line arguments are correct. */
    if (argc < 5)
    {
        /* Display the usage and exit. We cannot proceed further. */
        sndPkt_printUsage();
        return -1;
    }

	/* Copy the peer destination IPV6 Address */
	{
		int	index = 0;

		while (argv[1][index] != 0)
		{
			ipv6DstAddress[index] = (char)argv[1][index];
			index++;
		}
		ipv6DstAddress[index] = 0;
	}

    /* Get all the required arguments 
     * - Argument 2 is the Port Number. Extract it and validate it. */
    if ((sscanf(argv[2],"%d",&port_number)!=1) || (port_number > 65535))
    {
        printf ("Error: Invalid Port Number specified\n");
        return -1;
    }

    /* Get all the required arguments 
     * - Argument 3 is the number of packets. Extract it */
    if (sscanf(argv[3],"%d",&numPackets) !=1)
    {
        printf ("Error: Invalid number of packets specified.\n");
        return -1;
    }

    /* Get all the required arguments 
     * - Argument 4 is the size of the packet. Extract it */
    if (sscanf(argv[4],"%d",&packetSize) !=1)
    {
        printf ("Error: Invalid packet size specified.\n");
        return -1;
    }

    /* Validate the packet size. We are not sending packets > 64K */
    if (packetSize > 64*1024)
    {
        printf ("Error: Packet size is too big\n");
        return -1;
    }

    /* Is this a GTPU packet we are sending. */
    if (port_number == GTPU_PORT)
    {
        /* Get the optional GTPU id argument. */
        if (argc != 6)
        {
            printf ("Error: GTPU Identifier not specified\n");
            sndPkt_printUsage();
            return -1;
        }

        /* Get all the required arguments 
         * - Argument 5 is the GTPU Identifier. Extract it */
        if (sscanf(argv[5],"0x%x",&gtpuId) !=1)
        {
            printf ("Error: Unable to get the GTPU Identifier\n");
            return -1;
        }
        printf ("Debug: GTPU Identifier is 0x%x\n", gtpuId);
    }

#ifdef _WIN32
    /* Initialize the WINSOCK Library */
    wVersionRequested = MAKEWORD(1, 1);
    if (WSAStartup(wVersionRequested, &wsaData) != 0)
    {
        printf("\r\nUnable to initialize WinSock for host info");
        exit(0);
    }
#endif

    /* Create a local socket. */
    sLocalSocket = sndPkt_createLocalSocket();
    if (sLocalSocket < 0)
        return -1;

    /* Create the receive socket for GTPU or UDP; depending upon the configuration. */
    if (port_number == GTPU_PORT)
        sRecvSocket = sndPkt_createRecvSocket(1, port_number);
    else
        sRecvSocket = sndPkt_createRecvSocket(0, port_number);

    /* Check if we were able to create the socket? */
    if (sRecvSocket < 0)
        return -1;

	/* Populate the destination information to where the packet is to be transmitted. */
	memset ((void *)&to, 0, sizeof(to));
	to.sin6_family      = AF_INET6;    
    to.sin6_port        = htons(port_number);

#ifdef _WIN32
    /* Convert the IPv6 Address. */
	if (inet_pton(AF_INET6, (PCSTR)ipv6DstAddress, &to.sin6_addr) <= 0)
	{
		printf ("Error: inet_pton returned error code %d\n", errno);
        return -1;
	}
#else
    /* Convert the IPv6 Address. */
	if (inet_pton(AF_INET6, (const char*)ipv6DstAddress, &to.sin6_addr) <= 0)
	{
		printf ("Error: inet_pton returned error code %d\n", errno);
        return -1;
	}
#endif

    /* If packet size is -1, then send packets of varying lengths. */
	if (packetSize == -1)
	{
        packetSize     = 1;
        isVaryingSize  = 1;
	}

    /* Send out the packets. */
    while (!isTestDone)
    {
        /* Are we sending GTPU Packets? */
        if (port_number == GTPU_PORT)
        {
            /* GTPU Packet is being sent: The data payload accounts for the GTPU Header also. */
            dataBuffer = (char *)malloc(packetSize + GTPHDR_SIZE);
            if (dataBuffer == NULL)
            {
                printf ("Error: Unable to allocate memory for the data packet\n");
                return -1;
            }

            /* Get the GTPU Header & Data Payload. */
            ptrGTPUHeader = (GTPUHDR*)dataBuffer;
            ptrDataPayload = (uint8_t*)((uint8_t*)ptrGTPUHeader + GTPHDR_SIZE);

            /* Populate the GTPU Header. */
            ptrGTPUHeader->Flags    = 0x30;
            ptrGTPUHeader->MsgType  = 255;
            ptrGTPUHeader->TotalLen = htons(packetSize);
            ptrGTPUHeader->TunnelId = htonl(gtpuId);

            /* Populate the data payload. */
            for (index = 0; index < packetSize; index++)
                *(ptrDataPayload + index) = index;

            /* The data payload length is now incremented to account for the GTPU Header also */
            txPacketSize = packetSize + GTPHDR_SIZE;
        }
        else
        {
            /* UDP Payload: Allocate just the packet size for the data payload. */
            dataBuffer = (char *)malloc(packetSize);
            if (dataBuffer == NULL)
            {
                printf ("Error: Unable to allocate memory for the data packet\n");
                return -1;
            }

            /* Populate the data payload. */
            for (index = 0; index < packetSize; index++)
                *(dataBuffer + index) = index;

            /* Setup the data payload length. */
            txPacketSize = packetSize;
        }

        /* Send out the packet. */
        if(sendto(sLocalSocket, dataBuffer, txPacketSize, 0, (struct sockaddr *)&to, sizeof(to) ) < 0 )
        {
            /* Error: Unable to send out the packet. */
		    printf("send failed (%d)\n",WSAGetLastError());
            return -1;
	    }

        /* Display the appropriate message. */
        if (port_number == GTPU_PORT)
            printf ("Debug: GTPU Packet %d Transmitted Successfully\n", packetCount + 1);
        else
            printf ("Debug: UDP Packet %d (size %d) Transmitted Successfully\n", packetCount + 1, packetSize);

        /* Wait for a response to come back. */
        memset ((void *)&from, 0, sizeof(from));
        data_payload_len = sizeof(from);
        rxPacketSize = recvfrom(sRecvSocket, dataBuffer, txPacketSize, 0, (struct sockaddr *)&from, (int *)&data_payload_len);
        if (rxPacketSize < 0)
        {
		    printf("recv failed (%d)\n",WSAGetLastError());
            return -1;
	    }

        /* TODO: Currently validations are done only for non GTPU packets. */
        if (port_number != GTPU_PORT)
        {
            /* Response packet is validated: */
            if (txPacketSize != rxPacketSize)
            {
                printf ("Error: Sent %d bytes but received only %d bytes\n", txPacketSize, rxPacketSize);
                return -1;
            }

            /* Validate the received packet */
            for (index = 0; index < rxPacketSize; index++)
            {
                if ((uint8_t)(*(dataBuffer + index)) != (uint8_t)index)
                {
                    printf ("Error: Data Validation failed @ index %d Expected 0x%x Got 0x%x\n", 
                            index, (uint8_t)index, (uint8_t)(*(dataBuffer + index)));
                    return -1;
                }
            }
        }

        /* Display the appropriate message. */
        if (port_number == GTPU_PORT)
            printf ("Debug: GTPU Packet %d Received Successfully\n", packetCount + 1);
        else
            printf ("Debug: UDP Packet %d (size %d) Received Successfully\n", packetCount + 1, packetSize);

        /* Clean up the packet. */
        free (dataBuffer);

        /* If varying size flag is set, then increment the packet size. */
        if (isVaryingSize == 1)
		{
            packetSize++;
			if (packetSize > 9000) 
				packetSize = 1;
		}

        /* Increment the packet count. */
        packetCount++;

        /* Check if test is done. */
        if (packetCount == numPackets)
            isTestDone = 1;
    }


#ifdef _WIN32
    /* Close the socket. */
    closesocket(sLocalSocket);
    closesocket(sRecvSocket);

    /* Cleanup the windows socket library */
    WSACleanup();
#else
    /* Close the socket. */
    close(sLocalSocket);
    close(sRecvSocket);
#endif
    return 0;
}

