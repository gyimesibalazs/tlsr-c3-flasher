#include "tuart.h"

#include <Arduino.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "config.h"
#include "uartlog.h"

static const uart_port_t TUART = UART_NUM_1;
static bool on = false;
static uint32_t cur_baud = 0;

bool tuart_begin(uint32_t baud, int rx_buf, int tx_buf, const char *session) {
  tuart_end();
  ulog_start("%s, %u baud", session, (unsigned)baud);
  uart_config_t cfg = {};
  cfg.baud_rate = (int)baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  pinMode(PIN_SWS_TX, INPUT);                  // the resistor side of the SWS wire stays quiet
  if (uart_driver_install(TUART, rx_buf, tx_buf, 0, NULL, 0) != ESP_OK) {
    ulog_event("UART driver install failed");
    return false;
  }
  uart_param_config(TUART, &cfg);
  uart_set_pin(TUART, PIN_BRIDGE_TX, PIN_SWS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_rx_timeout(TUART, 2);               // hand received bytes over after 2 idle character times
  uart_set_rx_full_threshold(TUART, 64);       // more time for the interrupt at 921600 (a byte every 11 µs)
  gpio_pullup_en((gpio_num_t)PIN_SWS_RX);
  uart_flush_input(TUART);
  on = true;
  cur_baud = baud;
  return true;
}

void tuart_end() {
  if (!on) return;
  on = false;
  ulog_event("end");
  ulog_flush();
  if (uart_is_driver_installed(TUART)) uart_driver_delete(TUART);
  pinMode(PIN_BRIDGE_TX, INPUT);
}

bool tuart_on() { return on; }
uint32_t tuart_baud() { return cur_baud; }

void tuart_write(const uint8_t *d, int n) {
  if (n <= 0) return;
  ulog_tx(d, n);
  uart_write_bytes(TUART, (const char *)d, n);
}

bool tuart_wait_tx(uint32_t ms) { return uart_wait_tx_done(TUART, pdMS_TO_TICKS(ms)) == ESP_OK; }

size_t tuart_available() {
  size_t n = 0;
  uart_get_buffered_data_len(TUART, &n);
  return n;
}

int tuart_read(uint8_t *d, int max, uint32_t timeout_ms) {
  if (max <= 0) return 0;
  int got = uart_read_bytes(TUART, d, (uint32_t)max, pdMS_TO_TICKS(timeout_ms));
  if (got <= 0) return 0;
  ulog_rx(d, got);
  return got;
}

void tuart_flush_input() {
  uint8_t b[128];
  size_t n = tuart_available();
  bool any = n > 0;
  if (any) ulog_event("dropped as stale:");
  while (n > 0) {
    int got = tuart_read(b, n < sizeof(b) ? (int)n : (int)sizeof(b), 0);
    if (got <= 0) break;
    n -= got;
  }
  uart_flush_input(TUART);
}

void tuart_set_baud(uint32_t baud) {
  ulog_event("baud %u", (unsigned)baud);
  uart_set_baudrate(TUART, baud);
  cur_baud = baud;
}

void tuart_release_tx() { pinMode(PIN_BRIDGE_TX, INPUT); }

void tuart_attach_tx() { uart_set_pin(TUART, PIN_BRIDGE_TX, PIN_SWS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); }

void tuart_event(const char *fmt, ...) {
  char text[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  ulog_event("%s", text);
}
