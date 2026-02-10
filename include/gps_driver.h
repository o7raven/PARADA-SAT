#ifndef _GPS_Driver
#define _GPS_Driver

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "config.h"

void gps_initialize();
bool gps_read_line(char *buf, size_t max_len);

#endif
