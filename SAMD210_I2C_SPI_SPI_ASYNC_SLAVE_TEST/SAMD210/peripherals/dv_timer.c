#include <atmel_start.h>
#include "dv_timer.h"

static void TIMER_0_task1_cb(const struct timer_task *const timer_task) 
{
    gpio_toggle_pin_level(GLED_B);    
}    

static struct timer_task TIMER_0_task1;

/*!
 * /brief   Initialize 1 second timer
 */
void initTimer(void) {
    TIMER_0_task1.interval = 1000;
    TIMER_0_task1.cb       = TIMER_0_task1_cb;
    TIMER_0_task1.mode     = TIMER_TASK_REPEAT;

    timer_add_task(&TIMER_0, &TIMER_0_task1);
    timer_start(&TIMER_0);
}