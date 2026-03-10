#ifndef _GPS_Driver
#define _GPS_Driver

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "config.h"

typedef struct{
    int32_t lat; // lat * 1e7
    int32_t lon; // lon * 1e7
} gps_data_t;

void gps_initialize();
static bool gps_read_line(char *buf, size_t max_len);
static int32_t parse_coord(const char *s, char dir);
static int parse_int(const char *s, int len);
static char* next_field(char *s);
bool gps_update(gps_data_t *gps);

#endif
