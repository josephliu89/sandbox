#ifndef _IQ_PROCESSOR_H_
#define _IQ_PROCESSOR_H_

#include <ap_int.h>
#include <hls_stream.h>
#include <stdio.h>
#include <hls_math.h>


typedef struct {
	
		ap_uint<16> data;
		ap_uint<1> last;

	} axiWord;
	
typedef struct {
		ap_uint<16> data;
		ap_uint<1> last;

	} axiWordL;
	
typedef struct {
		float data;				//32 bit
		ap_uint<1>		last;		//1 bit
		ap_uint<2>		isHeader;	//2 bit
		unsigned int	length;				//32 bit
		int	n;
	} blockData;

typedef hls::stream<blockData> STREAM_BlockData;

typedef hls::stream<axiWord> STREAM_AXI;

void headerStripper (hls::stream<axiWord> &inDataStream, hls::stream<axiWord> &outDataStream, hls::stream<axiWord> &outHeaderStream);


#endif
