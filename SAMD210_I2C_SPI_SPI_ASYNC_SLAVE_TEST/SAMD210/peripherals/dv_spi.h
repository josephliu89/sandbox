#ifndef _DV_SPI_INCLUDED
#define _DV_SPI_INCLUDED

#pragma  pack(1)

typedef struct {
    uint8_t header;
    uint8_t devAddr;
    uint8_t regAddr;
    uint8_t data[2];
} commPacket;

void initSpi(void);
void printRxTxCount(void);
bool processData(void);


#endif // _DV_SPI_INCLUDED