#include "gps_driver.h"
#include "stdlib.h"


void gps_initialize(){
    uart_init(GPS_UART, GPS_BAUD);

    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(GPS_RX_PIN);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_baudrate(GPS_UART, GPS_BAUD);
}
static bool gps_read_line(char *buf, size_t max_len) {
    static size_t i = 0;

    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);

        if (c == '\r') continue;

        if (c == '\n') {
            buf[i] = '\0';
            i = 0;
            return true;
        }

        if (i < max_len - 1) {
            buf[i++] = c;
        } else {
            i = 0;
        }
    }

    return false;
}

static char* next_field(char *s)
{
    while (*s && *s != ',') s++;

    if (*s == ',') return s + 1;

    return s;
}

static int parse_int(const char *s, int len)
{
    int v = 0;

    for(int i=0;i<len;i++)
        v = v*10 + (s[i]-'0');

    return v;
}
static int32_t parse_coord(const char *s, char dir)
{
    int deg_len = (s[4] == '.') ? 2 : 3;

    int deg = parse_int(s, deg_len);

    float minutes = atof(s + deg_len);

    float coord = deg + minutes / 60.0f;

    int32_t val = coord * 10000000;

    if (dir == 'S' || dir == 'W')
        val = -val;

    return val;
}

bool gps_update(gps_data_t *gps)
{
    static char line[128];

    if(!gps_read_line(line,sizeof(line))){
        return false;
    }

    if(line[0] != '$'){
        return false;
    }

    if(line[3] != 'G'){
        return false;
    }

    char *time = next_field(line);
    char *lat = next_field(time);
    char *lat_dir = next_field(lat);
    char *lon = next_field(lat_dir);
    char *lon_dir = next_field(lon);    
    if(!lat || !lat_dir || !lon || !lon_dir){
        return false;
    }
    gps->lat = parse_coord(lat, lat_dir[0]);
    gps->lon = parse_coord(lon, lon_dir[0]);

    return true;
}
