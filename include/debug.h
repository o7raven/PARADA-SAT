#ifndef __DEBUG_H
#define __DEBUG_H

#include "pico/stdlib.h"
#include "pico/time.h"
#include "config.h"

int64_t led_off_callback(alarm_id_t id, void *user_data);
void debugLED(uint8_t code);

#endif

