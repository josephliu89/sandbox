#ifndef _TEST_BENCH_H_
#define _TEST_BENCH_H_



void headerWrapper (STREAM_AXI &inDataStream, STREAM_AXI &inHeaderStream, STREAM_AXI &outDataStream) ;
void streamAxiPacket(STREAM_AXI &targetStream, ap_uint<16> header1,ap_uint<16> header2,ap_uint<16> header3, int length );
void printStream ( STREAM_AXI &targetStream);
void streamNack (STREAM_AXI &targetStream);


#endif
