
/*
 * sndpkt_ho.c
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
#include <netinet/in.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#endif

#ifndef _WIN32
typedef int32_t	SOCKET;
#endif

/**********************************************************************
 ************************* Local Definitions **************************
 **********************************************************************/

/* Standard GTPU Definitions. */
#define GTPU_PORT                   2152
#define GTPHDR_SIZE                 8
#define CONTROL_UDP_PORT_NUMBER     1024

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

int32_t             txPacketSize[1000];
uint32_t            txPacketIndex=0;
uint32_t            rxPacketIndex=0;

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
    printf("-------------------------------------------------------------------------------------------\n");
    printf("Version 1.0                                                                                \n");
    printf("Usage: SNDPKT <x.x.x.x> <port no> <burst size> <packet size> [0x<gtpuId>] [<msgType>]     \n");    
    printf(" - If the port no is 2152 then the parameter GTPU id also needs to be specified            \n");
    printf(" - The packet size is the actual size of the data which is being sent.                     \n");
    printf(" - If packet size and num packets are both set to -1, then packets of varying              \n");
    printf("   sizes are sent in an infinite loop.                                                     \n");
    printf(" - To test GTPU control messages, msgType parameter must be specified                      \n");
    printf("-------------------------------------------------------------------------------------------\n");
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
static SOCKET sndPkt_createLocalSocket (void)
{
    SOCKET              sLocalSocket;
    struct sockaddr_in  sin;

	/* Open the socket. */
    sLocalSocket = socket( AF_INET, SOCK_DGRAM, 0 );
    if (sLocalSocket < 0 )
    {
        printf("failed socket (%d)\n",WSAGetLastError());
        return -1;
    }

	/* Initiliaze the sock address structure, */
	memset ((void *)&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_family      = AF_INET;    
    sin.sin_port        = htons(0);

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
 *  @param[in]  portNumber 
 *      Port number on which the socket is to be bound. Applicable only
 *      for UDP sockets. 
 *
 *  @retval
 *      Success     -   Socket Handle
 *  @retval
 *      Error       -   <0
 */
static SOCKET sndPkt_createRecvSocket (uint8_t isGTPUSocket, uint16_t portNumber)
{
    SOCKET              sRecvSocket;
    struct sockaddr_in  sin;
    int32_t             flags;

	/* Open the socket. */
    sRecvSocket = socket( AF_INET, SOCK_DGRAM, 0 );
    if (sRecvSocket < 0 )
    {
        printf("failed socket (%d)\n",WSAGetLastError());
        return -1;
    }

	/* Initiliaze the sock address structure, */
	memset ((void *)&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_family      = AF_INET;

    /* For GTPU Sockets we are waiting for a packet to arrive on 2152 */
    if (isGTPUSocket)
        sin.sin_port = htons(2152);
    else
        sin.sin_port = htons(portNumber);

	/* Bind the socket */
	if (bind (sRecvSocket, (struct sockaddr *)&sin, sizeof(sin)) < 0)
    {
		printf ("Error: bind failed (%d)\n", WSAGetLastError());
		return -1;
    }

    /* Set socket to non-blocking */
    if ((flags = fcntl(sRecvSocket, F_GETFL, 0)) < 0)
    {
		printf ("Error: fcntl get failed (%d)\n", WSAGetLastError());
		return -1;
    }

    if (fcntl(sRecvSocket, F_SETFL, flags | O_NONBLOCK) < 0)
    {
		printf ("Error: fcntl set failed (%d)\n", WSAGetLastError());
		return -1;
    }
    /* Debug Message: */
    printf ("Debug: Receive socket has been created on port %d\n", ntohs(sin.sin_port));

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
    uint16_t            port_number;
    uint32_t            gtpuId = 0;
    uint8_t             msgType = 255;
    SOCKET              sLocalSocket;
    SOCKET              sRecvSocket;
    SOCKET              sRecvControlSocket;
    char*               dataBuffer;
    char                controlDataBuffer[512];
    char                controlDataSize;
    int32_t             index;
    struct sockaddr_in  to;
    struct sockaddr_in  toControl;
    struct sockaddr_in  from;
    struct sockaddr_in  fromControl;
    GTPUHDR*            ptrGTPUHeader;
    uint8_t*            ptrDataPayload;
    uint32_t            data_payload_len;
    uint32_t            control_data_payload_len;
    uint32_t            packetCount = 0;
    int32_t             packetSize, dataSize;
    int32_t             rxPacketSize;
    uint8_t             isVaryingSize = 0;
    uint8_t             isTestDone = 0;

    /* Sanity Check: Ensure that the command line arguments are correct. */
    if (argc < 5)
    {
        /* Display the usage and exit. We cannot proceed further. */
        sndPkt_printUsage();
        return -1;
    }

    /* Get all the required arguments 
     * - Argument 1 is the IP Address. Extract it and validate it. */
    if (sscanf(argv[1],"%03d.%03d.%03d.%03d",&tmp1,&tmp2,&tmp3,&tmp4) != 4)
    {
        /* Display the usage and exit. We cannot proceed further. */
        printf ("Error: Invalid IP address specified\n");
        return -1;
    }
    if ((tmp1 < 0 || tmp1 > 255 || tmp2 < 0 || tmp2 > 255) ||
        (tmp3 < 0 || tmp3 > 255 || tmp4 < 0 || tmp4 > 255))
    {
        printf ("Error: Invalid IP address specified\n");
        return -1;
    }

    /* Compute the destination IP Address and store it locally. */
    tmp1 |= tmp2 << 8;
    tmp1 |= tmp3 << 16;
    tmp1 |= tmp4 << 24;

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

	if (packetSize == -1)
    {
        printf ("Error: Invalid packet size specified\n");
        return -1;
    }

    /* Validate the packet size. We are not sending packets > 64K */
    if (packetSize > 2000)
    {
        printf ("Error: Packet size is too big\n");
        return -1;
    }

    /* Is this a GTPU packet we are sending. */
    if (port_number == GTPU_PORT)
    {
        /* Get the optional GTPU id argument. */
        if (argc < 6)
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

        if (argc == 7)
        {
            /* Get all the required arguments 
             * - Argument 6 is the message type. Extract it */
            if (sscanf(argv[6],"%d",&msgType) !=1)
            {
	    	    msgType = 255;
            }
            printf ("Debug: Message Type is %d\n", msgType);
        }
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

    /* Create the receive socket for GTPU data
     * Create the receive UDP control socket to communicate with eNB. */
    sRecvSocket = sndPkt_createRecvSocket(1, 2152);

    /* Check if we were able to create the socket? */
    if (sRecvSocket < 0)
        return -1;

    sRecvControlSocket = sndPkt_createRecvSocket(0, CONTROL_UDP_PORT_NUMBER);

    /* Check if we were able to create the socket? */
    if (sRecvControlSocket < 0)
        return -1;

	/* Populate the destination information to where the data packet is to be transmitted. */
	memset ((void *)&to, 0, sizeof(to));
	to.sin_family      = AF_INET;    
    to.sin_addr.s_addr = tmp1;
    to.sin_port        = htons(port_number);

    /* Populate the destination information to where the control packet is to be transmitted. */
	memset ((void *)&toControl, 0, sizeof(to));
	toControl.sin_family      = AF_INET;    
    toControl.sin_addr.s_addr = tmp1;
    toControl.sin_port        = htons(CONTROL_UDP_PORT_NUMBER);

    /* Process the response to come back. */
    memset ((void *)&fromControl, 0, sizeof(fromControl));
    control_data_payload_len = sizeof(fromControl);

    while (1)
    {
    /* If packet size is -1, then send packets of varying lengths. */
	if (packetSize == -1)
	{
        packetSize     = 1;
        isVaryingSize  = 1;
	}

    txPacketIndex = 0;
    rxPacketIndex = 0;
    packetCount = 0;

    /* Send out the packets. */
    while (1)
    {
        dataSize = recvfrom(sRecvControlSocket, controlDataBuffer, 512, 0, (struct sockaddr *)&fromControl, 
                                (int *)&control_data_payload_len);

        if (dataSize > 0)
        {
            gtpuId = 0xdead1000;
            printf ("Debug: Received HO message 1 from eNB. Switching GTPU Id to 0x%x\n", gtpuId);
            memset ((void *)&fromControl, 0, sizeof(fromControl));
            control_data_payload_len = sizeof(fromControl);

            /* Send out the control packet. */
            if(sendto(sLocalSocket, controlDataBuffer, sizeof(controlDataBuffer), 0, (struct sockaddr *)&toControl, sizeof(toControl) ) < 0 )
            {
                /* Error: Unable to send out the packet. */
		        printf("send failed (%d)\n",WSAGetLastError());
                return -1;
	        }
        }

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
            ptrGTPUHeader->MsgType  = msgType;
            ptrGTPUHeader->TotalLen = htons(packetSize);
            ptrGTPUHeader->TunnelId = htonl(gtpuId);

            /* Populate the data payload. */
            for (index = 0; index < packetSize; index++)
                *(ptrDataPayload + index) = txPacketIndex;

            /* The data payload length is now incremented to account for the GTPU Header also */
            txPacketSize[txPacketIndex] = packetSize + GTPHDR_SIZE;
        }

        /* Send out the packet. */
        if(sendto(sLocalSocket, dataBuffer, txPacketSize[txPacketIndex], 0, (struct sockaddr *)&to, sizeof(to) ) < 0 )
        {
            /* Error: Unable to send out the packet. */
		    printf("send failed (%d)\n",WSAGetLastError());
            return -1;
	    }
        txPacketIndex++;

        /* If varying size flag is set, then increment the packet size. */
        if (isVaryingSize == 1)
		{
            packetSize++;
			if (packetSize > 65494) 
				packetSize = 1;
		}

        /* Increment the packet count. */
        packetCount++;

#if 1
        /* Display the appropriate message. */
        if (port_number == GTPU_PORT)
            printf ("Debug: GTPU Packet %d Transmitted Successfully\n", packetCount);
        else
            printf ("Debug: UDP Packet %d (size %d) Transmitted Successfully\n", packetCount, packetSize);
#endif

        /* Check if test is done. */
        if (packetCount == numPackets)
            break;

    }

    while (rxPacketIndex != txPacketIndex)
    {
        dataSize = recvfrom(sRecvControlSocket, controlDataBuffer, 512, 0, (struct sockaddr *)&fromControl, 
                                (int *)&control_data_payload_len);

        if (dataSize > 0)
        {
            gtpuId = 0xdead1000;
            printf ("Debug: Received HO message 2 from eNB. Switching GTPU Id to 0x%x\n", gtpuId);
            memset ((void *)&fromControl, 0, sizeof(fromControl));
            control_data_payload_len = sizeof(fromControl);

            /* Send out the control packet. */
            if(sendto(sLocalSocket, controlDataBuffer, sizeof(controlDataBuffer), 0, (struct sockaddr *)&toControl, sizeof(toControl) ) < 0 )
            {
                /* Error: Unable to send out the packet. */
		        printf("send failed (%d)\n",WSAGetLastError());
                return -1;
	        }
            break;
        }

        /* Process the response to come back. */
        memset ((void *)&from, 0, sizeof(from));
        data_payload_len = sizeof(from);

        /* GTPU Packet is being sent: The data payload accounts for the GTPU Header also. */
        dataBuffer = (char *)malloc(packetSize + GTPHDR_SIZE);
        if (dataBuffer == NULL)
        {
            printf ("Error: Unable to allocate memory for the data packet\n");
            return -1;
        }

        rxPacketSize = recvfrom(sRecvSocket, dataBuffer, txPacketSize[rxPacketIndex], 0, (struct sockaddr *)&from, 
                                (int *)&data_payload_len);
        if (rxPacketSize < 0)
        {
        	free (dataBuffer);
            continue;
	    }

        /* Response packet is validated: */
        if (txPacketSize[rxPacketIndex] != rxPacketSize)
        {
            printf ("Error: Sent %d bytes but received only %d bytes\n", txPacketSize[rxPacketIndex], rxPacketSize);
            return -1;
        }

        /* For GTPU Packets we need to skip the GTPU Header. */ 
        if (port_number == GTPU_PORT)
		{
			dataBuffer   = dataBuffer + GTPHDR_SIZE;
			rxPacketSize = rxPacketSize - GTPHDR_SIZE;
		}

        /* Validate the received packet */
        for (index = 0; index < rxPacketSize; index++)
        {
            if ((uint8_t)(*(dataBuffer + index)) != (uint8_t)rxPacketIndex)
            {
                printf ("Error: Data Validation failed @ index %d Expected 0x%x Got 0x%x\n", 
                        index, (uint8_t)rxPacketIndex, (uint8_t)(*(dataBuffer + index)));
                return -1;
            }
        }
        rxPacketIndex++;
        /* Display the appropriate message & clean up the data buffer */
        if (port_number == GTPU_PORT)
		{
            printf ("Debug: GTPU Packet %d (size %d) Received\n", rxPacketIndex, rxPacketSize);
        	free (dataBuffer - GTPHDR_SIZE);
        }
        else
        {
            printf ("Debug: UDP Packet %d (size %d) Received\n", rxPacketIndex, rxPacketSize);
        	free (dataBuffer);
        }

    }
    }
#ifdef _WIN32
    /* Close the socket. */
    closesocket(sLocalSocket);
    closesocket(sRecvSocket);
    closesocket(sRecvControlSocket);

    /* Cleanup the windows socket library */
    WSACleanup();
#else
    /* Close the socket. */
    close(sLocalSocket);
    close(sRecvSocket);
    close(sRecvControlSocket);
#endif
    return 0;
}

