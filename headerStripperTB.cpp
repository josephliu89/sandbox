
#include "IQ_Processor.h"
#include "Test_Bench.h"

using namespace hls;

#define TOP_LEVEL_TEST_DATA_LENGTH 	200 //this is the length of the packet generated for top level block testing, must be in the range of 1-4096
#define TEST_PACKET_LENGTH 			203
#define HEX_LENGTH					0x000000C8 //this is the hex representation of our data length
#define HEADER_1					0x00001111; //this is a random value we ensure is conserved dec:4396
#define HEADER_2					0x00001110; //this is a random value we ensure is conserved dec:4396
#define RAMP_AMP					32760  //this is the magnitude of the ramp function created for top level block testing
#define RAMP_INC					5000	 //this is the incremental step of the ramp function
#define MATLAB_PACKET_LENGTH		551



int main(){
	int i = 0;

	STREAM_AXI AXI_stream_in;
	STREAM_AXI AXI_stream_out_data;
	STREAM_AXI AXI_stream_out_headers;
	STREAM_AXI AXI_stream_out_final;

	axiWord 	inAxiWord;
	axiWord 	outAxiWord;

	ap_uint<16> header1;
	ap_uint<16> header2;
	ap_uint<16> header3;

	header1 = HEADER_1;
	header2 = HEADER_2;
	header3 = HEX_LENGTH;

	printf("Testing header stripping function....\n\n");

	//**********NORMAL PACKET TEST

	printf("Streaming packet\n");

	streamAxiPacket(AXI_stream_in,header1,header2,header3,203);

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerStripper(AXI_stream_in,AXI_stream_out_data,AXI_stream_out_headers);
	}

	printf("Header Stream:\n");

	printStream(AXI_stream_out_headers);

	printf("Data Stream:\n");

	printStream(AXI_stream_out_data);
	
	//*************** 2 NACK TEST

	printf("Streaming nack packet\n");

	streamNack(AXI_stream_in);

	streamNack(AXI_stream_in);

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerStripper(AXI_stream_in,AXI_stream_out_data,AXI_stream_out_headers);
	}

	printf("Header Stream:\n");

	printStream(AXI_stream_out_headers);

	printf("Data Stream:\n");

	printStream(AXI_stream_out_data);

	//Send another packet into the stream

	printf("Streaming packet\n");

	streamAxiPacket(AXI_stream_in,header1,header2,header3,203);

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerStripper(AXI_stream_in,AXI_stream_out_data,AXI_stream_out_headers);
	}

	printf("Header Stream:\n");

	printStream(AXI_stream_out_headers);

	printf("Data Stream:\n");

	printStream(AXI_stream_out_data);


	printf("Testing Header wrapper function...\n\n");


	printf("Streaming packet\n");

	streamAxiPacket(AXI_stream_in,header1,header2,header3,203);

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerStripper(AXI_stream_in,AXI_stream_out_data,AXI_stream_out_headers);
	}

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerWrapper(AXI_stream_out_data,AXI_stream_out_headers,AXI_stream_out_final);
	}

	printf("Final Stream:\n");

	printStream(AXI_stream_out_final);

	printf("Streaming nack packets followed by normal data\n");

	streamNack(AXI_stream_in);

	streamNack(AXI_stream_in);

	streamAxiPacket(AXI_stream_in,header1,header2,header3,203);

	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerStripper(AXI_stream_in,AXI_stream_out_data,AXI_stream_out_headers);
	}
    
	for (i=0;i<TEST_PACKET_LENGTH*2;i++){
	headerWrapper(AXI_stream_out_data,AXI_stream_out_headers,AXI_stream_out_final);
	}

	printf("Final Stream:\n");

	printStream(AXI_stream_out_final);



	return 0;


}

void streamAxiPacket(STREAM_AXI &targetStream, ap_uint<16> header1,ap_uint<16> header2,ap_uint<16> header3, int length ){

	axiWord currWord;
	int i;

	currWord.last = 0;
	currWord.data = header1;
	targetStream.write(currWord);

	currWord.last = 0;
	currWord.data = header2;
	targetStream.write(currWord);

	currWord.last = 0;
	currWord.data = header3;
	targetStream.write(currWord);


	for (i=0; i < length-1 ; i++){
		currWord.data = i;
		currWord.last = 0;
		targetStream.write(currWord);
	}


	currWord.data = 0;
	currWord.last = 1;

	targetStream.write(currWord);


}


void printStream ( STREAM_AXI &targetStream){
	int i = 0;
	ap_uint<16> temp;
	axiWord currWord;


	while(!targetStream.empty()){
		targetStream.read(currWord);
		temp = currWord.data;
		printf("Data at %10i = %10i last=%10i\n",i,temp.to_int(),currWord.last.to_int());
		i++;
	}


}


void streamNack (STREAM_AXI &targetStream){

	axiWord currWord;
	int i;

	ap_uint<16> header1 = 0x01;
	ap_uint<16> header2 = 0x02;
	ap_uint<16> header3 = 0;

	currWord.last = 0;
	currWord.data = header1;
	targetStream.write(currWord);

	currWord.last = 0;
	currWord.data = header2;
	targetStream.write(currWord);

	currWord.last = 1;
	currWord.data = header3;
	targetStream.write(currWord);

}













///
