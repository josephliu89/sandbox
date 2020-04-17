#ifndef _DV_ADC_INCLUDED
#define _DV_ADC_INCLUDED

typedef enum {
    IR = 0x00,
    ANALOG0 = 0x01, 
    ANALOG1 = 0x06
} channelReading;

void initAdc(void);
bool getConversionStatus(void);
uint16_t getRawAdcVal(void);
void adcSetChannel(channelReading channel);

#endif // _DV_ADC_INCLUDED