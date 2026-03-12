#ifndef _GPS_Driver
#define _GPS_Driver

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    int32_t lat;
    int32_t lon;
    bool fix;
} gps_data_t;

void gps_initialize(void);
void gps_update(gps_data_t *out);

#endif
