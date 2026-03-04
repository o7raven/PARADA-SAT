#include "gps_driver.h"

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
bool gps_read_line(char *buf, size_t max_len) {
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
