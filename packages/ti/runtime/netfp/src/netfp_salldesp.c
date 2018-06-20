/**
 *   @file  netfp_salldesp.c
 *
 *   @brief
 *      The file implements the workaround in the SA LLD which allows
 *      the packets to be sent without the SA channel handle.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2013-2014 Texas Instruments, Inc.
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
#include <string.h>

/* MCSDK Include Files. */
#include <ti/csl/csl_cache.h>
#include <ti/drv/cppi/cppi_drv.h>

/* SYSLIB Include Files. */
#include <ti/runtime/netfp/netfp.h>
#include <ti/runtime/netfp/include/netfp_internal.h>

/**************************************************************************
 **************************** Local Definitions ***************************
 **************************************************************************/

/* Legacy typedefs from TI MAS component */
typedef uint16_t    tuint;
typedef int16_t     tint;
typedef uint32_t    tulong;
typedef int32_t     tlong;
typedef uint8_t     tword;

#define TYP_TWORD_SIZE      8
#define TYP_TINT_SIZE       16
#define TYP_TULONG_SIZE     32

#define SALLD_DIV_ROUND_UP(val,rnd)   (((val)+(rnd)-1)/(rnd))
#define SALLD_ROUND_UP(val,rnd)       SALLD_DIV_ROUND_UP(val,rnd)*(rnd)

/******************************************************************************
 * IP Version 4 Protocol Definitions
 ******************************************************************************/
/* IPV4 byte offsets to fields */
#define IPV4_BYTE_OFFSET_VER_HLEN       0
#define IPV4_BYTE_OFFSET_TOS            1
#define IPV4_BYTE_OFFSET_LEN            2
#define IPV4_BYTE_OFFSET_ID             4
#define IPV4_BYTE_OFFSET_FLAGS_FRAGO    6
#define IPV4_BYTE_OFFSET_TTL            8
#define IPV4_BYTE_OFFSET_PROTO          9
#define IPV4_BYTE_OFFSET_HDR_CHKSUM     10
#define IPV4_BYTE_OFFSET_SRC_ADDR       12
#define IPV4_BYTE_OFFSET_DEST_ADDR      16

/* IPV4 definitions */
#define IPV4_VER_MASK                   0xF0
#define IPV4_VER_VALUE                  0x40
#define IPV4_HLEN_MASK                  0x0F
#define IPV4_HLEN_SHIFT                 0
#define IPV4_FRAGO_MASK                 0x1FFF
#define IPV4_FLAGS_MF_MASK              0x2000
#define IPV4_FLAGS_DF_MASK              0x4000
#define IPV4_FLAG_DET_MASK              (IPV4_FLAGS_MF_MASK | \
                                         IPV4_FLAGS_DF_MASK)
#define IPV4_FRAG_DET_VALUE             0x0000
#define IPV4_PROTO_UDP                  0x11
#define IPV4_ADDR_MULTICAST_MASK        0xF0000000ul
#define IPV4_ADDR_MULTICAST_VALUE       0xE0000000ul
#define IPV4_ADDR_BROADCAST_VALUE       0xFFFFFFFFul

#define IPV4_HDR_MIN_SIZE_BYTES         20

#define IP_READ_VER(x)       UTL_GET_BITFIELD(*((x) + IPV4_BYTE_OFFSET_VER_HLEN),4,4)
#define IPV4_READ_IHL(x)     UTL_GET_BITFIELD(*((x) + IPV4_BYTE_OFFSET_VER_HLEN),0,4)

/* IPV4 Option defintions */
#define IPV4_OPTIONS_BYTE_OFFSET_TYPE       0
#define IPV4_OPTIONS_BYTE_OFFSET_LEN        1
#define IPV4_OPTIONS_BYTE_OFFSET_DATA       2

#define IPV4_OPTIONS_TYPE_COPY_MASK         0x80
#define IPV4_OPTIONS_TYPE_COPY_SHIFT           7
#define IPV4_OPTIONS_TYPE_CLASS_MASK        0x60
#define IPV4_OPTIONS_TYPE_CLASS_SHIFT          5
#define IPV4_OPTIONS_TYPE_NUM_MASK          0x1F
#define IPV4_OPTIONS_TYPE_NUM_SHIFT            5

#define IPV4_OPTIONS_GET_TYPE_NUM(x)            ((x) & IPV4_OPTIONS_TYPE_NUM_MASK)
#define IPV4_OPTIONS_MK_TYPE(copy, class, num)  (((copy) << IPV4_OPTIONS_TYPE_COPY_SHIFT)   | \
                                                ((class) << IPV4_OPTIONS_TYPE_CLASS_SHIFT) | \
                                                ((num) & IPV4_OPTIONS_TYPE_NUM_MASK))

#define IPV4_OPTIONS_TYPE_END_OPTION            0
#define IPV4_OPTIONS_TYPE_LOOSE_SRC_ROUTE       0x83
#define IPV4_OPTIONS_TYPE_STRICT_SRC_ROUTE      0x89
#define IPV4_OPTIONS_SRC_ROUTE_BYTE_OFFSET_PTR  2


/******************************************************************************
 * IP Version 6 Protocol Definitions
 ******************************************************************************/
/* IPV6 byte offsets to fields */
#define IPV6_BYTE_OFFSET_VER_TC           0
#define IPV6_BYTE_OFFSET_TC_FLH           1
#define IPV6_BYTE_OFFSET_FL_ML            2
#define IPV6_BYTE_OFFSET_PLEN             4
#define IPV6_BYTE_OFFSET_PROTO            6
#define IPV6_BYTE_OFFSET_HOP_LIMIT        7
#define IPV6_BYTE_OFFSET_SRC_ADDR         8
#define IPV6_BYTE_OFFSET_DEST_ADDR        24

/* Size of IPV6 basic header */
#define IPV6_HDR_SIZE_BYTES               40

/* IPV6 Definitions  */
#define IPV6_VER_MASK                     0xF0
#define IPV6_VER_VALUE                    0x60
#define IPV6_ADDR_SIZE                    16

/* Most significant byte check for IPv6 multicast address */
#define IPV6_ADDR_MULTICAST_MSB_OFFSET    0
#define IPV6_ADDR_MULTICAST_MSB           0xFF

/******************************************************************************
 * IP Common Protocol Definitions
 ******************************************************************************/

/* IPv4/IPV6 type identifiers */
#define IP_TYPE_IPV4        4
#define IP_TYPE_IPV6        6

/* Protocol field values (IPV4) / next header (IPV6) */
#define IP_PROTO_IPV6_HOP_BY_HOP    0   /* IPv6 extension header - hop by hop */
#define IP_PROTO_ICMP               1
#define IP_PROTO_IGMP               2
#define IP_PROTO_GGP                3
#define IP_PROTO_IP_IN_IP           4   /* IP tunneling */
#define IP_PROTO_TCP                6
#define IP_PROTO_EGP                8
#define IP_PROTO_UDP               17
#define IP_PROTO_IPV6_IN_IPV4      41   /* IP tunneling */
#define IP_PROTO_IPV6_ROUTE        43   /* IPv6 extension header - route */
#define IP_PROTO_IPV6_FRAG         44   /* IPv6 extension header - fragmentation */
#define IP_PROTO_GRE               47
#define IP_PROTO_ESP               50   /* Encapsulating security payload */
#define IP_PROTO_AUTH              51   /* Authentication header (ipv4) */
#define IP_PROTO_IPV6_NO_NEXT      59   /* IPv6 extention header - no next header      */
#define IP_PROTO_IPV6_DEST_OPT     60   /* IPv6 extension header - destination options */
#define IP_PROTO_IPCOMP           108   /* IP Compression Protocol */

/* IPv6 extension header offsets */
#define IPV6_OPT_HEADER_OFFSET_PROTO        0
#define IPV6_OPT_HEADER_OFFSET_LEN          1
#define IPV6_OPT_HEADER_LEN_UNIT_IN_BYTES   8

/* IPv6 hop by hop and destination options */
#define IPV6_EXT_HDR_OPT_OFFSET_TYPE        0
#define IPV6_EXT_HDR_OPT_OFFSET_DATA_LEN    1
#define IPV6_EXT_HDR_OPT_OFFSET_DATA        2
#define IPV6_EXT_HDR_OPT_HDR_SIZE           2

#define IPV6_EXT_HDR_OPT_TYPE_PAD0          0
#define IPV6_EXT_HDR_OPT_TYPE_PADN          1
#define IPV6_EXT_HDR_OPT_TYPE_JUMBO         0xc2

#define IPV6_EXT_HDR_OPT_TYPE_CHNAGE_MASK   0x20

#define IPV6_EXT_HDR_OPT_TYPE_IS_MUTABLE(x) ((x) & IPV6_EXT_HDR_OPT_TYPE_CHNAGE_MASK)

/* IPv6 Routing header definitions */
#define IPV6_OPT_ROUTE_HDR_OFFSET_TYPE          2
#define IPV6_OPT_ROUTE_HDR_OFFSET_SEG_LEFT      3
#define IPV6_OPT_ROUTE_HDR_OFFSET_TYPE0_ADDR    8

#define IPV6_OPT_ROUTE_HDR_TYPE_0          0

/* Fixed length IPv6 extension header options */
#define IPV6_OPT_FRAG_EXTENSION_LEN_BYTES  8

/******************************************************************************
 * UDP Protocol Definitions
 ******************************************************************************/
/* UDP byte offsets to fields */
#define UDP_BYTE_OFFSET_SRC_PORT       0
#define UDP_BYTE_OFFSET_DEST_PORT      2
#define UDP_BYTE_OFFSET_LEN            4
#define UDP_BYTE_OFFSET_CKSUM          6

#define UDP_HDR_SIZE_BYTES             8

/******************************************************************************
 * RTP Protocol Definitions
 ******************************************************************************/
/* RTP byte offsets to fields */
#define RTP_BYTE_OFFSET_VPXCC          0    /* V(2)|P(1)|X(1)|CC(4) */
#define RTP_BYTE_OFFSET_M_PT           1    /* M(1)|PT(7) */
#define RTP_BYTE_OFFSET_SEQ_NUM        2
#define RTP_BYTE_OFFSET_TIMESTAMP      4
#define RTP_BYTE_OFFSET_SSRC           8

#define RTP_HDR_BASIC_SIZE_BYTES       12
#define RTP_GET_HDR_SIZE(x)           (RTP_HDR_BASIC_SIZE_BYTES + (((x) & 0xF) << 2))


/******************************************************************************
 * IPSEC Authentication Header Definitions
 ******************************************************************************/
/* IPSEC AH byte offsets to fields */
#define IPSEC_AH_OFFSET_NEXT_HEADER     0
#define IPSEC_AH_OFFSET_PAYLOAD_LEN     1
#define IPSEC_AH_OFFSET_RESERVED1       2
#define IPSEC_AH_OFFSET_SPI             4
#define IPSEC_AH_OFFSET_SEQ_NUM         8
#define IPSEC_AH_OFFSET_AUTH_DATA      12

#define IPSEC_AH_BASIC_SIZE_BYTES      12


/*
 *  The IPSEC Authentication Header in Data Structure Format.
 *
 *  Note: It is defined here for the reference purpose since it works at a Big-Endian processor only.
 *
 */
typedef  struct  SA_IPSEC_AH_tag
{
    uint8_t  next_header;         /*  identifies the next payload after the authentication payload.
                                 *  refer to the IP protocol number defined above */

    uint8_t  auth_data_len;       /* the length of authentication data field in 32-bit words
                                 * 0: null authentication algorithm
                                 * Note: this value is equal to the total AH length minus 2? */

    uint16_t reserved;            /* reserved for the future use */

    uint32_t spi;                 /* Security Parameters Index (SPI). 0: no security associated */

    uint32_t seq_num;             /* Sequence Number */

    uint8_t  auth_data[1];        /* Place holder for the authentication dtata */

} SA_IPSEC_AH_T;

/******************************************************************************
 * IPSEC Encapsulating Security Payload (ESP) Definitions
 ******************************************************************************/
/* IPSEC ESP Header byte offsets to fields */
#define IPSEC_ESP_HDR_OFFSET_SPI         0
#define IPSEC_ESP_HDR_OFFSET_SEQ_NUM     4
#define IPSEC_ESP_HDR_OFFSET_IV          8

#define IPSEC_ESP_HDR_BASIC_SIZE_BYTES   8


/*
 *  The IPSEC ESP Header in Data Structure Format.
 *
 *  Note: It is defined here for the reference purpose since it works at a Big-Endian processor only.
 *
 */
typedef  struct  SA_IPSEC_ESP_HDR_tag
{
    uint32_t spi;                 /* Security Parameters Index (SPI). 0: no security associated */

    uint32_t seq_num;             /* Sequence Number */

    uint8_t  iv_data[1];          /* Place holder for the initialization vector */

} SA_IPSEC_ESP_HDR_T;


/* IPSEC ESP Tail byte offsets to fields */
#define IPSEC_ESP_TAIL_OFFSET_PADDING_LEN       0
#define IPSEC_ESP_TAIL_OFFSET_NEXT_HEADER       1

#define IPSEC_ESP_TAIL_SIZE_BYTES               2

/*
 *  The IPSEC ESP Tail in Data Structure Format.
 *
 *  Note: It is defined here for the reference purpose since it works at a Big-Endian processor only.
 *
 */
typedef  struct  SA_IPSEC_ESP_TAIL_tag
{
    uint8_t  padding_len;         /*  number of bytes that was padded to the payload data */

    uint8_t  next_header;         /*  identifies the next payload after the authentication payload.
                                 *  refer to the IP protocol number defined above */

} SA_IPSEC_ESP_TAIL_T;

#define SALLD_SIZE_OF_TUINT_IN_BYTE ((TYP_TINT_SIZE)/(8))
#define SALLD_SIZE_OF_TUINT_IN_WORD ((TYP_TINT_SIZE)/(TYP_TWORD_SIZE))
#define SALLD_SIZE_OF_WORD_IN_BYTE ((TYP_TWORD_SIZE)/(8))
#define SALLD_BYTE_TO_TUINT(a)  ((a)/SALLD_SIZE_OF_TUINT_IN_BYTE)
#define SALLD_BYTE_TO_WORD(a)  ((a)/SALLD_SIZE_OF_WORD_IN_BYTE)

/**************************************************************************
 **************************** ESP SA LLD Functions ************************
 **************************************************************************/

/* ggdsp abstraction for RTSC misc_utlCopy function */
static inline void misc_utlCopy(uint16_t * a, uint16_t * b, int16_t c) {
    memmove((void *)(b), (void *)(a), (c) * sizeof(uint16_t));
}

/******************************************************************************
 * FUNCTION PURPOSE: Read 8 bit value from 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Returns 8 bit value from 16 bit word.  Assumes nothing.
 *
 * tuint pktRead8bits_m (
 *    tword *base,       - Base of byte array
 *    tuint byteOffset); - Byte offset to read
 *
 *****************************************************************************/
static inline tuint pktRead8bits_m (tword *base, tuint byteOffset)
{
  char *src = (char *)base;
  char wvalue = *(src + byteOffset);
  tuint readByte = (tuint)(wvalue & 0xFF);
  return readByte;
} /* pktRead8bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Write 8 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 8 bit value into 16 bit word; nothing assumed.
 *
 * void pktWrite8bits_m (
 *    tword *base,      - Base of byte array
 *    tuint byteOffset, - byte offset to write
 *    tuint val)        - Byte in low 8 bits of val
 *
 *****************************************************************************/
static inline void pktWrite8bits_m (tword *base, tuint byteOffset, tuint val)
{
  char *wptr = ((char *)base + byteOffset);
  *wptr = (char)(val & 0xFF);
} /* pktWrite8bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Write 16 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 16 bit value into 16 bit word.  No assumptions
 *
 * void pktWrite16bits_m (
 *    tword *base,      - Base of byte array
 *    tuint byteOffset, - byte offset to write; assumed to be even
 *    tuint val)        - 16 bit val
 *
 *****************************************************************************/
static inline void pktWrite16bits_m (tword *base, tuint byteOffset, tuint val)
{
  char *wptr = ((char *)base + byteOffset);

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  wptr[0] = (char)(val>>8);
  wptr[1] = (char)(val & 0xff);

} /* pktWrite16bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Read 16 bit value from 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Returns 16 bit value from 16 bit word.  No assumptions.
 *
 * tuint pktRead16bits_m (
 *    tword *base,       - Base of byte array
 *    tuint byteOffset); - Byte offset to read, assumed to be even
 *
 *****************************************************************************/
static inline tuint pktRead16bits_m (tword *base, tuint byteOffset)
{
  char *wptr = ((char *)base + byteOffset);
  tuint ret;

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  ret = (((tuint)wptr[0]) << 8) | (wptr[1] & 0xFF);

  return ret;
} /* pktRead16bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Write 32 bit value into 16 bit words (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 32 bit value into 16 bit word; No
 *              alignment assumed
 *
 * void pktWrite32bits_m (
 *    tword *base,      - Base of byte array
 *    tuint byteOffset, - byte offset to write; assumed to be even.
 *    tulong val)       - 32 bit val
 *
 *****************************************************************************/
static inline void pktWrite32bits_m (tword *base, tuint byteOffset, tulong val)
{
  /* Shift/mask is endian-portable, but look out for stupid compilers */
  pktWrite16bits_m (base, byteOffset, (tuint)(val>>16));
  pktWrite16bits_m (base, byteOffset+2, (tuint)(val&0xffff));

} /* pktWrite32bits_m */

/******************************************************************************
 * Function:    ones_complement_chksum
 ******************************************************************************
 * Description: Calculate an Internet style one's complement checksum
 *
 ******************************************************************************/
static inline uint16_t salld_ones_complement_chksum ( uint16_t *p_data, uint16_t len )
{
  uint32_t  chksum = 0;

  while (len > 0)
  {
    chksum += (uint32_t)pktRead16bits_m ((tword *)p_data,0);
    p_data++;
    len--;
  }
  chksum = (chksum >> 16) + (chksum & 0xFFFF); /* add in carry   */
  chksum += (chksum >> 16);                    /* maybe one more */
  return (uint16_t)chksum;

} /* end of salld_ones_complement_chksum() */

/**************************************************************************************
 * FUNCTION PURPOSE: Compute and insert the ipv4 checksum
 **************************************************************************************
 * DESCRIPTION: Compute and insert the ipv4 checksum
 **************************************************************************************/
static void salld_set_ipv4_chksum (tword *data)
{
  uint16_t hdr_len; /* Hdr Length in 16-bit word */
  uint16_t ip_hdr_chksum;

  /* calculate IP header length */
  hdr_len =  (pktRead8bits_m(data, IPV4_BYTE_OFFSET_VER_HLEN) & IPV4_HLEN_MASK) << 1;

  pktWrite16bits_m(data, IPV4_BYTE_OFFSET_HDR_CHKSUM, 0);


  /* Length for IP Checksum calculation  should be in terms of 16-bit twords only */
  ip_hdr_chksum = salld_ones_complement_chksum ((uint16_t *)data, hdr_len);

  pktWrite16bits_m(data, IPV4_BYTE_OFFSET_HDR_CHKSUM, ~ip_hdr_chksum);

} /* salld_set_ipv4_chksum */

/**
 *  @b Description
 *  @n
 *      The function is modified from the SA LLD ESP Channel send data
 *      and modified to get the IP SEC configuration parameters instead of
 *      deriving them from the SA channel handle
 *
 *  @param[in]  pConfig
 *      Pointer to the IPSEC configuration
 *  @param[in]  swInfo
 *      Transmit software information which is to be populated in the descriptor.
 *  @param[in]  pktInfo
 *      Packet information which specifies the location of the various headers
 *
 * \ingroup NETFP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int16_t Netfp_salld_esp_send_data
(
    Sa_IpsecConfigParams_t* pConfig,
    Sa_SWInfo_t             swInfo,
    void*                   pktInfo
)
{
	Sa_PktInfo_t* pPktInfo = (Sa_PktInfo_t*)pktInfo;
	Sa_PktDesc_t* pPktDesc = &pPktInfo->pktDesc;
	Sa_ipsecNatTInfo_t* pNatTInfo = &pPktInfo->natTInfo;
	tword *pktIn, *pktSegTail;
	tword *pIP, *pOrig;
	tword *pIPNew, *pNew;
	int16_t espHdrSize, ipVerLen, espMiscSize, udpHdrSize;
	int16_t ipLen, payloadLen, paddingLen, encryptedLen, i, offset, oldPayloadLen;
	/* Segmentation support */
	int16_t ipSegmentIndex, ipSegmentOffset, tailSegmentOffset, payloadOffset, segOffsetTmp, segOffset, segLast;
	uint8_t nextHdr;
	uint16_t f_ipV6;
	uint16_t *segUsedSizes;

	/* Sanity Check */
	if(pPktDesc->nSegments < 1)
	    return (sa_ERR_PARAMS);

	/* Encrypt and Authenticate Packet(s) */
	payloadOffset = pPktDesc->payloadOffset;
	ipSegmentIndex = tailSegmentOffset = segOffsetTmp = i = segOffset = 0;

	segUsedSizes = pPktDesc->segUsedSizes;

	/* Calculate segment containing IP header as well as segment offset to IP Header*/
	segOffsetTmp = segUsedSizes[ipSegmentIndex];
	while(segOffset + segOffsetTmp < payloadOffset)
	{
    	segOffset += segOffsetTmp;
	    tailSegmentOffset += segOffsetTmp;
    	segOffsetTmp = segUsedSizes[++ipSegmentIndex];
	}
	ipSegmentOffset = payloadOffset - segOffset;

	/* Calculate segment containing ESP padding as well as segment offset to ESP padding*/
	for(i = ipSegmentIndex; i < (segLast = pPktDesc->nSegments-1);i++)
	{
	    tailSegmentOffset += segUsedSizes[i];
	}

	pOrig = pIP = (tword *)pPktDesc->segments[ipSegmentIndex];
	pIP += SALLD_BYTE_TO_WORD(ipSegmentOffset);
	pktIn = pIP;

	/*
	* Calculate the size of authentictaion header and retrieve the pointer to the authentication header
	*/
	espHdrSize = IPSEC_ESP_HDR_BASIC_SIZE_BYTES + pConfig->ivSize;
	ipVerLen = pktRead8bits_m(pktIn, IPV4_BYTE_OFFSET_VER_HLEN);

    /* Determine the IP version. */
    f_ipV6 = ((ipVerLen & IPV4_VER_MASK) == IPV6_VER_VALUE);

	if (f_ipV6)
	{
    	/* IPV6 requires that All header to be 8-byte aligned */
	    espHdrSize = SALLD_ROUND_UP(espHdrSize, 8);
    	ipLen = IPV6_HDR_SIZE_BYTES;

    	/* Extract payloadlen from the IP header */
	    payloadLen = pktRead16bits_m(pktIn, IPV6_BYTE_OFFSET_PLEN) + IPV6_HDR_SIZE_BYTES;
	}
	else
	{
    	ipLen = (ipVerLen & IPV4_HLEN_MASK) << 2;
	    /* Extract payloadlen from the IP header */
    	payloadLen = pktRead16bits_m(pktIn, IPV4_BYTE_OFFSET_LEN);
	}

	udpHdrSize = (pPktInfo->validBitMap & sa_PKT_INFO_VALID_IPSEC_NAT_T_INFO)?UDP_HDR_SIZE_BYTES:0;

	pIPNew = pIP - SALLD_BYTE_TO_WORD(espHdrSize + udpHdrSize);
	pNew = pOrig - SALLD_BYTE_TO_WORD(espHdrSize + udpHdrSize);

	misc_utlCopy((uint16_t *)pOrig, (uint16_t *)pNew,  SALLD_BYTE_TO_TUINT(ipLen+ipSegmentOffset));

	/*
	* Calculate padding length
	*/
	encryptedLen = payloadLen - ipLen;

	if (pConfig->encryptionBlockSize == 1)
	{
	    paddingLen = 0;
	}
	else
	{
    	paddingLen = (encryptedLen + IPSEC_ESP_TAIL_SIZE_BYTES) % pConfig->encryptionBlockSize;
	    if (paddingLen)
    	{
	    	paddingLen = pConfig->encryptionBlockSize - paddingLen;
    	}
	}


	espMiscSize = paddingLen + IPSEC_ESP_TAIL_SIZE_BYTES + pConfig->macSize;
	/* Extract the next header and update the IP header for ESP */
	oldPayloadLen = payloadLen;
	payloadLen += espHdrSize + espMiscSize + udpHdrSize;
	if (f_ipV6)
	{
    	nextHdr = pktRead8bits_m(pIPNew, IPV6_BYTE_OFFSET_PROTO);
	    pktWrite8bits_m(pIPNew, IPV6_BYTE_OFFSET_PROTO, udpHdrSize?IP_PROTO_UDP:IP_PROTO_ESP);
    	pktWrite16bits_m(pIPNew, IPV6_BYTE_OFFSET_PLEN, payloadLen - IPV6_HDR_SIZE_BYTES);
	}
	else
	{
    	nextHdr = pktRead8bits_m(pIPNew, IPV4_BYTE_OFFSET_PROTO);
	    pktWrite8bits_m(pIPNew, IPV4_BYTE_OFFSET_PROTO,  udpHdrSize?IP_PROTO_UDP:IP_PROTO_ESP);
    	pktWrite16bits_m(pIPNew, IPV4_BYTE_OFFSET_LEN, payloadLen);

    	/* Reclaculate the IPV4 header checksum */
	    salld_set_ipv4_chksum(pIPNew);
	}

	/* Populate the UDP header */
	pktIn = pIPNew + SALLD_BYTE_TO_WORD(ipLen);
	if (udpHdrSize)
	{
    	pktWrite16bits_m(pktIn, UDP_BYTE_OFFSET_SRC_PORT, pNatTInfo->srcPort);
	    pktWrite16bits_m(pktIn, UDP_BYTE_OFFSET_DEST_PORT, pNatTInfo->dstPort);
    	pktWrite16bits_m(pktIn, UDP_BYTE_OFFSET_LEN, payloadLen - ipLen);
	    pktWrite16bits_m(pktIn, UDP_BYTE_OFFSET_CKSUM, 0);
	}

	/* Populate the ESP header */
	pktIn += SALLD_BYTE_TO_WORD(udpHdrSize);
	pktWrite32bits_m(pktIn, IPSEC_ESP_HDR_OFFSET_SPI, pConfig->spi);

	pktIn += SALLD_BYTE_TO_WORD(IPSEC_ESP_HDR_BASIC_SIZE_BYTES);

	/* Reserve room for IV */
	if (pConfig->ivSize)
	{
    	pktIn += SALLD_BYTE_TO_WORD(pConfig->ivSize);
	}

	/* Assumption here is ESP header will be in same segment as IP header
	* IP Header must be in its own segment
	*/

	pktSegTail = (tword *) pPktDesc->segments[segLast];

	/*
	* set padding values - the first padding byte should be 0x01, the second one should be 0x02 ...
	*/

	offset = oldPayloadLen + payloadOffset - tailSegmentOffset;
	for (i = 1; i <= paddingLen; i++, offset++)
	{
    	pktWrite8bits_m(pktSegTail, offset, i);
	}

	/* Write Padding Len and next payload type */
	pktWrite8bits_m(pktSegTail, offset,  (tword)(paddingLen));
	pktWrite8bits_m(pktSegTail, offset + 1, nextHdr);

	/* Adjust the packet length */
	pPktDesc->payloadLen = payloadLen - ipLen - udpHdrSize;
	pPktDesc->payloadOffset += (ipLen + udpHdrSize);
	pPktDesc->segUsedSizes[ipSegmentIndex] += (espHdrSize + udpHdrSize);
	pPktDesc->segUsedSizes[segLast] += espMiscSize;
	pPktDesc->size += espHdrSize + espMiscSize + udpHdrSize;
	pPktDesc->segments[ipSegmentIndex] = pNew;

	/* Pass the software Info in the packet */
	pPktInfo->validBitMap |= sa_PKT_INFO_VALID_SW_INFO;
	pPktInfo->swInfo = swInfo;

	return(sa_ERR_OK);
}

