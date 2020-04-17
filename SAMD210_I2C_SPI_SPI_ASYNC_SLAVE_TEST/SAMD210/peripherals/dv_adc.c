#include <atmel_start.h>
#include "dv_adc.h"

static bool convComplete = false;
static uint8_t adcRawValue[2] = {0};
static uint16_t rawAdc = 0;

/*!
 *  Returns 12-bit raw ADC value with four most-significant bits as zeros
 */
uint16_t getRawAdcVal(void) {
    return rawAdc;
}

bool getConversionStatus(void) {
    return convComplete;
}

static void convert_cb_ADC_0(const struct adc_async_descriptor *const descr, const uint8_t channel) {

    adc_async_read_channel(&ADC_0, 0, adcRawValue, 2);
    rawAdc = (adcRawValue[1] << 8) | adcRawValue[0];
    printf("ADC Raw: 0x%04X\r\n", rawAdc);
    printf("ADC Converted: %.2f\r\n", (3.3*rawAdc)/4095);    // converted        
    convComplete = true;
    
}

void adcSetChannel(channelReading channel) {
    convComplete = false;
    adc_async_set_inputs(&ADC_0, channel, 0x18, 0);
    adc_async_start_conversion(&ADC_0);    
}
//
void initAdc(void) {
    adc_async_register_callback(&ADC_0, 0, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_0);
    adc_async_enable_channel(&ADC_0, 0);
    
    /* Note: Channel in this case means the number of ADC modules, not the number of ADC channels, that is why everything is
    referenced to "Channel" 0 and still works */
}