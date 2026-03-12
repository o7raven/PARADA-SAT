#include "debug.h"

int64_t led_off_callback(alarm_id_t id, void *user_data) {
    uint gpio = (uint)user_data;
    gpio_put(gpio, 0);
    return 0;
}

void debugLED(uint8_t code){
    if(code != sLED && code != wLED && code != eLED){
        return;
    }
    gpio_put(code, 1);

    add_alarm_in_ms(200, led_off_callback, (void*)(uintptr_t)code, false);
}
