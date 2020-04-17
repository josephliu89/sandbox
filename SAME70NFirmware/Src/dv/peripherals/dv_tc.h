/*
 * dv_tc.h
 *
 * Created: 11/13/2019 4:30:40 PM
 *  Author: liu
 */ 

#ifndef DV_TC_H_
#define DV_TC_H_

/* LT Power supply synchronization timer definition */
#define TC_WAVEFORM_DIVISOR                 (8U)
#define TC_WAVEFORM_FREQUENCY               (1300000U)
#define TC_WAVEFORM_FREQUENCY_DUTY_CYCLE    (50U)
#define TC_CHANNEL                          0

//void initTimer1ms(void);
void init_ps_sync(void);
void start_ps_sync(void);
void stop_ps_sync(void);

#endif /* DV_TC_H_ */