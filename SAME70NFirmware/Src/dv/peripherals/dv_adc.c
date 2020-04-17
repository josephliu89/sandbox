/*
 * dv_adc.c
 *
 * Created: 11/13/2019 4:29:50 PM
 *  Author: liu
 */ 

 #include "asf.h"
 #include "dv_adc.h"

/* ADC specific initializations */
struct afec_config afec_cfg;                        // struct for configuring ADC
struct afec_ch_config afec_ch_cfg;                  // struct for configuring ADC channel
static volatile uint32_t ul_adc_val;				// holds ADC value read from ADC Channel 1 / PA21
static bool is_conversion_done = false;

static void afec_end_conversion(void) {
    ul_adc_val = afec_channel_get_value(AFEC0, AFEC_CHANNEL_1);
    is_conversion_done = true;
}

 /**
 * \brief Configure and initialize ADC interface
 */
 void init_adc(void)
 {
 	/* ADC setup */
    afec_enable(AFEC0);                             // enable ADC
    afec_get_config_defaults(&afec_cfg);            // read ADC default values
    afec_init(AFEC0, &afec_cfg);                    // initialize the ADC with default values
    afec_set_trigger(AFEC0, AFEC_TRIG_SW);          // set ADC to trigger with software for polling
    afec_ch_get_config_defaults(&afec_ch_cfg);
    afec_ch_cfg.gain = AFEC_GAINVALUE_0;
    afec_ch_set_config(AFEC0, AFEC_CHANNEL_1, &afec_ch_cfg);
    afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_1, 0x200);
    afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_1, afec_end_conversion, 1);
    afec_channel_enable(AFEC0, AFEC_CHANNEL_1);     // enable ADC channel 1 (AD1, pin PA21)
	afec_start_software_conversion(AFEC0);			// start the ADC conversion via SW triggers

	DV_info("ADC interface initialized successfully!");
 }

 /**
 * \brief Print back value read from ADC pin (PA17)
 */
 void print_adc_value(void)
 {
    if (is_conversion_done)	{
        printf("Raw ADC value: 0x%04lX\t\tInput Voltage: %.2fV\r\n", ul_adc_val, 3.3*(ul_adc_val/4096.0));
        is_conversion_done = false;
    }
 }

 /**
 * \brief Return average value from ADC pin (PA21)
 */
 float get_adc_avg(void) {
	uint8_t num_conversions = 10;
    uint32_t adc_sum = 0;
    uint32_t adc_avg = 0;

	for (uint8_t i = 0; i < num_conversions; i++) {
		if (is_conversion_done)	{
            adc_sum += ul_adc_val;
            is_conversion_done = false;
		}
	}

	/* Average out samples */
	adc_avg = adc_sum / num_conversions;

	//printf("Average Voltage: %.2fV\r\n", 3.3*(adc_val/4096.0));

	return 3.3*(adc_avg/4096.0);
 }