/*
 * dv_tc.c
 *
 * Created: 11/13/2019 4:29:50 PM
 *  Author: liu
 */ 

 #include "asf.h"
 #include "dv_tc.h"

/**
 * \brief Initialize 1mSec timer 4 interrupt
 */
void init_ps_sync(void)
{
     uint32_t ra, rc;
     
     // Configure the PMC to enable the TC module.
     sysclk_enable_peripheral_clock(ID_TC0);

     // Init TC to waveform mode.
     tc_init(TC0, TC_CHANNEL,
             TC_CMR_TCCLKS_TIMER_CLOCK2 // Waveform Clock Selection, sysclk/8, 150MHz/8
             | TC_CMR_WAVE       // Waveform mode is enabled
             | TC_CMR_ACPA_SET   // RA Compare Effect: set
             | TC_CMR_ACPC_CLEAR // RC Compare Effect: clear
             | TC_CMR_CPCTRG     // UP mode with automatic trigger on RC Compare
     );

     // Configure waveform frequency and duty cycle.
     rc = (sysclk_get_peripheral_bus_hz(TC0) / TC_WAVEFORM_DIVISOR / TC_WAVEFORM_FREQUENCY);
     tc_write_rc(TC0, TC_CHANNEL, rc);
     ra = (100 - TC_WAVEFORM_FREQUENCY_DUTY_CYCLE) * rc / 100;
     tc_write_ra(TC0, TC_CHANNEL, ra);
}

/**
 * \brief Enable TC TC_CHANNEL_WAVEFORM
 */
void start_ps_sync(void)
{
	tc_start(TC0, TC_CHANNEL);
	//DV_info("Start waveform: Frequency = %d Hz,Duty Cycle = %2d%%\n\r", TC_WAVEFORM_FREQUENCY, TC_WAVEFORM_FREQUENCY_DUTY_CYCLE);
}

/**
 * \brief Disable TC TC_CHANNEL_WAVEFORM
 */
void stop_ps_sync(void)
{
	tc_stop(TC0, TC_CHANNEL);
}
