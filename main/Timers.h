#ifndef TIMERS_H
#define TIMERS_H

#include "esp_timer.h"

// Forward declarations and extern declarations
extern esp_timer_handle_t data_timer_handle;
extern esp_timer_handle_t alert_timer_handle;
extern bool data_timer_running;
extern bool alert_timer_running;

void init_data_timer(void);
void init_alert_timer(void);

#endif // TIMERS_H
