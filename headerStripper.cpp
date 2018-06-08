
#include "IQ_Processor.h"
// I added this to master
// first add
// second add

using namespace hls;

void headerStripper (stream<axiWord> &inDataStream, stream<axiWord> &outDataStream, stream<axiWord> &outHeaderStream) {
	/* This block preforms exclusively to conversion to floating point with no logic or
	additional functions. This allows RTL simulations to function. All downstream blocks
	must be compatible with the 32 bit float on the AXI bus.  */

	//
	#pragma HLS INTERFACE ap_ctrl_none port=return
	#pragma HLS PIPELINE II=1 enable_flush
	#pragma HLS INTERFACE port=inDataStream axis
	#pragma HLS INTERFACE port=outDataStream axis
	#pragma HLS INTERFACE port=outHeaderStream axis

	axiWord inData;

	static enum dState {D_STREAM_HEADER = 0, D_STREAM } dropState;

	static unsigned int headCount = 0;
git

	if (!inDataStream.empty()){

		switch (dropState){
			case D_STREAM_HEADER:
				inDataStream.read(inData);


				headCount++;


				if(inData.last){
					headCount = 0;
				}

				if(headCount >= 3){
					dropState = D_STREAM;
					headCount = 0;
					inData.last = 1; //we need to add this signal as this will be put on its own bus
				}

				outHeaderStream.write(inData);

				break;

			case D_STREAM:
			
				inDataStream.read(inData);

				if(inData.last){
				  dropState = D_STREAM_HEADER;
				}
  				outDataStream.write(inData);

				break;

		}

	}
	
}


