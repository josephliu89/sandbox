
#include "IQ_Processor.h"

using namespace hls;

void headerWrapper (stream<axiWord> &inDataStream, stream<axiWord> &inHeaderStream, stream<axiWord> &outDataStream) {
/* This block preforms exclusively to conversion to floating point with no logic or 
additional functions. This allows RTL simulations to function. All downstream blocks 
must be compatible with the 32 bit float on the AXI bus.  */


	#pragma HLS INTERFACE ap_ctrl_none port=return
	#pragma HLS PIPELINE II=1 enable_flush
	#pragma HLS INTERFACE port=inDataStream axis
	#pragma HLS INTERFACE port=inHeaderStream axis
	#pragma HLS INTERFACE port=outDataStream axis

	axiWord currWord;


	static enum dState {D_STREAM_HEADER = 0, D_STREAM } dropState;

	static unsigned int headCount = 0;

	ap_uint<16> length16 = 0;
	unsigned int length = 0;

	switch (dropState) {

		case D_STREAM_HEADER:

			if(!inHeaderStream.empty()){
				
				inHeaderStream.read(currWord);
				
				headCount++;
				
				//check for zero length, and if so, reset our head count 
				if(headCount == 3) {
					length16 = currWord.data;
					length = (unsigned int) length16;
					
					if (length == 0) {
						headCount = 0;
						currWord.last = 1; //this is the second header of a zero length packet so we must assert the t-last signal
					}
				}

				if(headCount >= 3) {
					dropState = D_STREAM;
					headCount = 0;
					currWord.last = 0;
				}

				outDataStream.write(currWord);
			}
			break;

		case D_STREAM:
		
			if (!inDataStream.empty()) {
				inDataStream.read(currWord);
				
				if(currWord.last) {
					dropState = D_STREAM_HEADER;
				}
				
				outDataStream.write(currWord);
			}

			break;
	}
}


