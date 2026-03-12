#include "gps_driver.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>

#define GPS_UART uart1
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUD 9600

static char sentence[128];
static uint8_t sentence_index = 0;

static gps_data_t current_data;


static int32_t convert_degmin_to_fixed(const char *coord)
{
    int32_t deg = 0;
    int32_t minutes = 0;
    int32_t frac = 0;
    int frac_digits = 0;

    int i = 0;

    /* parse integer part */
    while(coord[i] && coord[i] != '.')
    {
        deg = deg * 10 + (coord[i] - '0');
        i++;
    }

    if(coord[i] == '.') i++;

    /* parse fractional minutes */
    while(coord[i] && frac_digits < 6)
    {
        frac = frac * 10 + (coord[i] - '0');
        frac_digits++;
        i++;
    }

    int32_t deg_part = deg / 100;
    minutes = deg % 100;

    /* convert minutes.fraction to scaled degrees */

    int32_t scale = 10000000;

    int32_t min_scaled = minutes * scale;

    int32_t frac_scaled = frac * scale;
    for(int j=0;j<frac_digits;j++)
        frac_scaled /= 10;

    int32_t total_minutes_scaled = min_scaled + frac_scaled;

    int32_t decimal = total_minutes_scaled / 60;

    return deg_part * scale + decimal;
}

static char* next_field(char *p)
{
    while(*p && *p != ',') p++;
    if(*p == ',') return p+1;
    return p;
}

static void parse_rmc(char *s)
{
    char *p = s;

    p = next_field(p); // skip sentence
    p = next_field(p); // time

    /* fix status */
    if(*p != 'A')
    {
        current_data.fix = false;
        return;
    }

    current_data.fix = true;

    p = next_field(p); // skip A

    /* latitude */
    int32_t lat = convert_degmin_to_fixed(p);

    p = next_field(p);

    if(*p == 'S')
        lat = -lat;

    p = next_field(p);

    /* longitude */
    int32_t lon = convert_degmin_to_fixed(p);

    p = next_field(p);

    if(*p == 'W')
        lon = -lon;

    current_data.lat = lat;
    current_data.lon = lon;
}
static void parse_sentence()
{
    if(sentence[3]=='R' && sentence[4]=='M' && sentence[5]=='C')
    {
        parse_rmc(sentence);
    }
}
void gps_initialize()
{
    uart_init(GPS_UART, GPS_BAUD);

    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(GPS_RX_PIN);

    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_baudrate(GPS_UART, GPS_BAUD);

    sentence_index = 0;

    current_data.lat = 0;
    current_data.lon = 0;
    current_data.fix = false;
}

void gps_update(gps_data_t *out)
{
    while(uart_is_readable(GPS_UART))
    {
        char c = uart_getc(GPS_UART);

        if(c == '\n')
        {
            sentence[sentence_index] = 0;

            parse_sentence();

            sentence_index = 0;
        }
        else
        {
            if(sentence_index < sizeof(sentence)-1)
                sentence[sentence_index++] = c;
        }
    }

    *out = current_data;
}
