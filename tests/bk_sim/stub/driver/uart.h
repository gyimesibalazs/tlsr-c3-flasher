// The part of the ESP-IDF UART driver bk72xx.cpp uses; bk_sim.cpp implements it on a simulated wire.
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int uart_port_t;
typedef int esp_err_t;
#define ESP_OK 0
#define UART_NUM_1 1
#define UART_PIN_NO_CHANGE (-1)
enum { UART_DATA_8_BITS = 3, UART_PARITY_DISABLE = 0, UART_STOP_BITS_1 = 1, UART_HW_FLOWCTRL_DISABLE = 0,
       UART_SCLK_DEFAULT = 0 };
typedef struct {
  int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk;
} uart_config_t;
esp_err_t uart_driver_install(uart_port_t port, int rx_buf, int tx_buf, int queue_len, void *queue, int flags);
esp_err_t uart_driver_delete(uart_port_t port);
bool uart_is_driver_installed(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *cfg);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud);
esp_err_t uart_set_rx_timeout(uart_port_t port, uint8_t symbols);
esp_err_t uart_set_rx_full_threshold(uart_port_t port, int threshold);
int uart_write_bytes(uart_port_t port, const void *src, size_t n);
esp_err_t uart_wait_tx_done(uart_port_t port, uint32_t ticks);
int uart_read_bytes(uart_port_t port, void *buf, uint32_t n, uint32_t ticks);
esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t *n);
esp_err_t uart_flush_input(uart_port_t port);
