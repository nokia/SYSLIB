#include <c6x.h>
#include <stdio.h>

#ifdef ORG_CODE
#include "sch_bytesort.h"
#else
#include <ti/runtime/dpm/include/sch_bytesort.h>
#endif

//#define N_SIZE 1024
//#define K_SIZE 20
//#define K_MARGIN 20
//SORT_KEY_TYPE in_key[(N_SIZE+15)/16*16]={
//	0xFFFFAAAA,8,9,10,4,3,2,1,1000, 200000, 300000,1200000, 1000,1000, 20,30,
//	7,8,9,10,4,3,2,1,1000, 200000, 300000,1200000, 1000,1000, 20,30,};
//SORT_INDEX_TYPE index[N_SIZE];
//SORT_KEY_TYPE key2[N_SIZE];
//uint32_t thrsh;
//uint32_t tmp_key[(N_SIZE+15)/16*16];
//uint32_t out_key[K_SIZE+K_MARGIN];
//uint32_t out_index[256];
//uint32_t Kprime;

//////////////////////////////////////
// Note: N must be a multiple of 16 // 
//////////////////////////////////////


/* Modifications: */
#ifdef ORG_CODE
uint32_t thrsh;
uint32_t tmp_key[512];
uint32_t out_key[512];
uint32_t out_index[512];
uint32_t Kprime;
#else
uint32_t thrsh;
uint32_t tmp_key[2048];
uint32_t out_key[2048];
uint32_t out_index[2048];
uint32_t Kprime;
#endif


// early bailout flag
#define K_MATCH_FLAG 0x100
// one byte search
static inline uint32_t threshold_search(
		uint32_t * restrict intlvArr,
        uint32_t N,
        uint32_t K,
        uint32_t Kmargin)
{
	uint32_t thrsh = 0x80808080;
   	uint32_t accCntA, accCntB;
   	uint32_t thrshAdj = 0x80808080;
   	uint32_t i,j;
    for(j = 0; j < 7; j++)
    { 
    	accCntA = 0;
    	accCntB = 0;
		thrshAdj >>= 1;
	    #pragma MUST_ITERATE(1); 
	    for(i=0; i<N; i+=16)
		{
			uint64_t inA, inB;
			uint32_t cmpOutA, cmpOutB;
			inA = _amem8(&intlvArr[i]);
			inB = _amem8(&intlvArr[i+2]);
			cmpOutA = _dcmpgtu4( _itoll(thrsh,thrsh), inA );
			cmpOutB = _dcmpgtu4( _itoll(thrsh,thrsh), inB );
			accCntA += _bitc4(cmpOutA);
			accCntB += _bitc4(cmpOutB);
		}
		//printf("K:%d A:%d T:%02X\n", K, N -( accCntA + accCntB ), thrsh&0xff);
		if (((N -( accCntA + accCntB )) >= K ) && ((N -( accCntA + accCntB )) <= (K+Kmargin))) return thrsh&0xff;
		if ((N -( accCntA + accCntB )) < K )
		{
			thrsh = _itoll((uint32_t)_loll(thrsh) - thrshAdj,
			               (uint32_t)_loll(thrsh) - thrshAdj );
		}
		else
		{
			thrsh = _itoll((uint32_t)_loll(thrsh) + thrshAdj,
			               (uint32_t)_loll(thrsh) + thrshAdj );
		}
    }
    
    // get count for the last threshold
   	accCntA = 0;
   	accCntB = 0;
    for(i=0; i<N; i+=16)
	{
		uint64_t inA, inB;
		uint32_t cmpOutA, cmpOutB;
		inA = _amem8(&intlvArr[i]);
		inB = _amem8(&intlvArr[i+2]);
		cmpOutA = _dcmpgtu4( _itoll(thrsh,thrsh), inA );
		cmpOutB = _dcmpgtu4( _itoll(thrsh,thrsh), inB );
		accCntA += _bitc4(cmpOutA);
		accCntB += _bitc4(cmpOutB);
	}
	//printf("K:%d A:%d T:%02X\n", K, N -( accCntA + accCntB ), thrsh&0xff);

	// decrease if necessary
	if ((N -( accCntA + accCntB )) < K )
	{
		//if(thrsh&2)	 
		thrsh-=0x01010101;
	}
	
    return thrsh&0xff;
}

// search threshold on high ptr_order bytes and zero lower ptr_order bytes
static inline uint32_t threshold_search_zero_next(
		uint32_t * restrict intlvArr,
        uint32_t N,
        uint32_t K,
        uint32_t Kmargin,
        uint32_t i_start)
{
	uint32_t thrsh = 0x80808080;
   	uint32_t thrshAdj = 0x80808080;
   	uint32_t i, j;
   	uint32_t accCntA, accCntB;
   	
    for(j = 0; j < 7; j++)
    { 
	   	accCntA = 0;
	   	accCntB = 0;
	   	
		thrshAdj >>= 1;
	    #pragma MUST_ITERATE(1); 
	    for(i=i_start; i<N; i+=16)
		{
			uint64_t inA, inB;
			uint32_t cmpOutA, cmpOutB;
			inA = _amem8(&intlvArr[i]);
			inB = _amem8(&intlvArr[i+2]);
			cmpOutA = _dcmpgtu4( _itoll(thrsh,thrsh), inA );
			cmpOutB = _dcmpgtu4( _itoll(thrsh,thrsh), inB );
			accCntA += _bitc4(cmpOutA);
			accCntB += _bitc4(cmpOutB);
		}

		//printf("K:%d A:%d T:%02X\n", K, N -( accCntA + accCntB ), thrsh&0xff);
		if (((N -( accCntA + accCntB )) >= K ) && ((N -( accCntA + accCntB )) <= (K+Kmargin))) return ((thrsh & 0xff) | K_MATCH_FLAG);
		if ((N -( accCntA + accCntB )) < K )
		{
			thrsh = _itoll((uint32_t)_loll(thrsh) - thrshAdj,
			               (uint32_t)_loll(thrsh) - thrshAdj );
		}
		else
		{
			thrsh = _itoll((uint32_t)_loll(thrsh) + thrshAdj,
			               (uint32_t)_loll(thrsh) + thrshAdj );
		}
    }
    
    // get counts for last threshold
   	accCntA = 0;
   	accCntB = 0;
    for(i=i_start; i<N; i+=16)
	{
		uint64_t inA, inB;
		uint32_t cmpOutA, cmpOutB;
		inA = _amem8(&intlvArr[i]);
		inB = _amem8(&intlvArr[i+2]);
		cmpOutA = _dcmpgtu4( _itoll(thrsh,thrsh), inA );
		cmpOutB = _dcmpgtu4( _itoll(thrsh,thrsh), inB );
		accCntA += _bitc4(cmpOutA);
		accCntB += _bitc4(cmpOutB);
	}
	//printf("K:%d A:%d T:%02X\n", K, N -( accCntA + accCntB ), thrsh&0xff);

	// decrease if necessary
	if (((N -( accCntA + accCntB )) >= K) && ((N -( accCntA + accCntB )) <= (K+Kmargin))) return ((thrsh & 0xff) | K_MATCH_FLAG);
	if ((N -( accCntA + accCntB )) < K )
	{
	//if((thrsh&2)&&(thrsh!=0))
		 thrsh-=0x01010101;
	}

    #pragma MUST_ITERATE(1); 
    for( i = i_start-4; i < N; i += 16 )
	{
		uint64_t inA, inB;
		uint64_t nextInA, nextInB;
		uint64_t maskA, maskB;
		uint32_t cmpOutA, cmpOutB;
		inA = _amem8(&intlvArr[i+4]);
		inB = _amem8(&intlvArr[i+6]);
		nextInA = _amem8(&intlvArr[i]);
		nextInB = _amem8(&intlvArr[i+2]);
		cmpOutA = ~_dcmpgtu4( _itoll(thrsh,thrsh), inA );
		cmpOutB = ~_dcmpgtu4( _itoll(thrsh,thrsh), inB );
		maskA = _dxpnd4(cmpOutA);
		maskB = _dxpnd4(cmpOutB);
		_amem8(&intlvArr[i]) = _itoll(
									_hill(maskA)&_hill(nextInA), 
									_loll(maskA)&_loll(nextInA));
		_amem8(&intlvArr[i+2]) = _itoll(
									_hill(maskB)&_hill(nextInB), 
									_loll(maskB)&_loll(nextInB));
	}
	return (thrsh&0xFF);
}


uint32_t selectindex_bit32 (int16_t N, int16_t K, SORT_KEY_TYPE * restrict key, SORT_INDEX_TYPE * restrict index)
{
   	uint32_t thrsh, thrsh32;
   	uint32_t i;

    // byte interleaving
    #pragma MUST_ITERATE(1); 
    for(i=0; i<N; i+=16)
	{
		uint64_t inA, inB, inC, inD;
		uint64_t outA, outB, outC, outD;

		inA = _amem8(&key[i]);
		inB = _amem8(&key[i+2]);
		inC = _amem8(&key[i+4]);
		inD = _amem8(&key[i+6]);
		outA = _dpackl4(_dpackl2(inD,inC),_dpackl2(inB,inA));
		outB = _dpackh4(_dpackl2(inD,inC),_dpackl2(inB,inA));
		outC = _dpackl4(_dpackh2(inD,inC),_dpackh2(inB,inA));
		outD = _dpackh4(_dpackh2(inD,inC),_dpackh2(inB,inA));
		
		
		_amem8(&tmp_key[i])   = _itoll(_saddu4(_hill(outA), 0x01010101),_saddu4(_loll(outA), 0x01010101));
		_amem8(&tmp_key[i+4]) = _itoll(_saddu4(_hill(outB), 0x01010101),_saddu4(_loll(outB), 0x01010101));
		_amem8(&tmp_key[i+8]) = _itoll(_saddu4(_hill(outC), 0x01010101),_saddu4(_loll(outC), 0x01010101));
		_amem8(&tmp_key[i+12])= _itoll(_saddu4(_hill(outD), 0x01010101),_saddu4(_loll(outD), 0x01010101));

		inA = _amem8(&key[i+8]);
		inB = _amem8(&key[i+10]);
		inC = _amem8(&key[i+12]);
		inD = _amem8(&key[i+14]);
		outA = _dpackl4(_dpackl2(inD,inC),_dpackl2(inB,inA));
		outB = _dpackh4(_dpackl2(inD,inC),_dpackl2(inB,inA));
		outC = _dpackl4(_dpackh2(inD,inC),_dpackh2(inB,inA));
		outD = _dpackh4(_dpackh2(inD,inC),_dpackh2(inB,inA));
		_amem8(&tmp_key[i+2])  = _itoll(_saddu4(_hill(outA), 0x01010101),_saddu4(_loll(outA), 0x01010101));
		_amem8(&tmp_key[i+6])  = _itoll(_saddu4(_hill(outB), 0x01010101),_saddu4(_loll(outB), 0x01010101));
		_amem8(&tmp_key[i+10]) = _itoll(_saddu4(_hill(outC), 0x01010101),_saddu4(_loll(outC), 0x01010101));
		_amem8(&tmp_key[i+14]) = _itoll(_saddu4(_hill(outD), 0x01010101),_saddu4(_loll(outD), 0x01010101));
	}


	// threshold search
	thrsh = threshold_search_zero_next(tmp_key, N, K, 0, 12);
	//if(thrsh == 0xFF ) { thrsh32 = 0xFF010100; goto selection; };
	if(thrsh & K_MATCH_FLAG) { thrsh32 = thrsh << 24; goto selection; };
	thrsh32 = thrsh;
	//printf("thrsh:%08X\n", thrsh32);
	
	thrsh = threshold_search_zero_next(tmp_key, N, K, 0, 8);
	//if(thrsh == 0xFF ) { thrsh32 = (thrsh32 << 8)|0x00FE0100; goto selection; };
	if(thrsh & K_MATCH_FLAG) { thrsh32 = (thrsh32 << 24) | (thrsh << 24 >> 8); goto selection;};
	thrsh32 = (thrsh32 << 8) | thrsh;
	//printf("thrsh:%08X\n", thrsh32);
	
	thrsh = threshold_search_zero_next(tmp_key, N, K, 0, 4);
	//if(thrsh == 0xFF ) { thrsh32 = (thrsh32 << 8)|0x0000FE00; goto selection; };
	if(thrsh & K_MATCH_FLAG) { thrsh32 = (thrsh32 << 16) | (thrsh << 24 >> 16); goto selection;};
	thrsh32 = (thrsh32 << 8) | thrsh;
	//printf("thrsh:%08X\n", thrsh32);
	
    thrsh32 = (thrsh32 << 8) | threshold_search(tmp_key, N, K, 0);

selection:
	// selection
	//printf("thrsh:%08X\n", thrsh32);
	thrsh32 -= 0x01010101; // set back threshold one unit
	{
		uint32_t *out_ptr1 = out_key;
		uint32_t *out_ptr2 = out_index;		
		uint64_t keyA, keyB;
		uint64_t indexA, indexB;
		Kprime = 0;		
		for(i=0; i<N; i+=4)
		{
			keyA = _amem8(&key[i]);
			indexA = _amem8(&index[i]);			
			keyB = _amem8(&key[i+2]);
			indexB = _amem8(&index[i+2]);			
		
		   if( _loll(keyA) >= thrsh32 )	{ out_ptr1[Kprime]=_loll(keyA); out_ptr2[Kprime++]=_loll(indexA); }
		   if( _hill(keyA) >= thrsh32 )	{ out_ptr1[Kprime]=_hill(keyA); out_ptr2[Kprime++]=_hill(indexA); }
		   if( _loll(keyB) >= thrsh32 )	{ out_ptr1[Kprime]=_loll(keyB); out_ptr2[Kprime++]=_loll(indexB); }
		   if( _hill(keyB) >= thrsh32 )	{ out_ptr1[Kprime]=_hill(keyB); out_ptr2[Kprime++]=_hill(indexB); }
		}
	}
#ifdef ORG_CODE	
    return thrsh32;
#else
    return Kprime;
#endif
}
