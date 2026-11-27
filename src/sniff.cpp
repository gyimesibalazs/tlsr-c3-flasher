#include "sniff.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "config.h"
#include "uartlog.h"

// Line A on UART1 (as tuart.cpp, which is off meanwhile), line B on UART0: the C3's console is its USB port for
// Arduino's Serial; ESP-IDF's own console (boot messages, error logs) is UART0 TX, which is on no pin while this
// runs (the UART driver is installed with RX only, TX left unrouted). Nothing else installs a UART0 driver.
static const uart_port_t UART_A = UART_NUM_1, UART_B = UART_NUM_0;
static const int RX_BUF = 4096;           // per line: a log block being compressed holds the loop up for ~20 ms
static const int EVENT_QUEUE = 16;        // the driver's events: overflow, framing errors

static bool on = false, live = false;
static uint32_t baud_a = 0, baud_b = 0;
static QueueHandle_t q_a = NULL, q_b = NULL;
static SniffRing ring;

struct LineErrors {
  uint32_t overflow = 0, errors = 0, shown_overflow = 0, shown_errors = 0;
};
static LineErrors err_a, err_b;
static uint32_t err_shown_ms = 0;

// Time resolution: a record's time is when the bytes were taken from the driver. The driver hands bytes over when
// its FIFO holds `threshold` of them or the line has been idle for two character times; the loop takes them within
// about a millisecond. At up to 57600 baud every byte is handed over at once.
static int rx_threshold(uint32_t baud) { return baud <= 57600 ? 1 : baud <= 230400 ? 8 : 32; }

static void line_end(uart_port_t port, QueueHandle_t *q) {
  if (uart_is_driver_installed(port)) {
    if (port == UART_NUM_0) uart_set_baudrate(port, 115200);   // ESP-IDF's console speed, as it was
    uart_driver_delete(port);
  }
  *q = NULL;
}

static bool line_begin(uart_port_t port, uint32_t baud, int pin, QueueHandle_t *q) {
  line_end(port, q);
  uart_config_t cfg = {};
  cfg.baud_rate = (int)baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  pinMode(pin, INPUT_PULLUP);                // an unconnected line reads idle, not noise
  if (uart_driver_install(port, RX_BUF, 0, EVENT_QUEUE, q, 0) != ESP_OK) return false;
  uart_param_config(port, &cfg);
  uart_set_pin(port, UART_PIN_NO_CHANGE, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_rx_timeout(port, 2);
  uart_set_rx_full_threshold(port, rx_threshold(baud));
  gpio_pullup_en((gpio_num_t)pin);
  uart_flush_input(port);
  return true;
}

bool sniff_begin(uint32_t a, uint32_t b, uint8_t *live_mem, uint32_t live_size) {
  sniff_end(NULL);
  pinMode(PIN_SWS_TX, INPUT);                // the resistor side of the SWS wire stays quiet
  if (!line_begin(UART_A, a, PIN_SWS_RX, &q_a) || !line_begin(UART_B, b, PIN_SNIFF_B, &q_b)) {
    line_end(UART_A, &q_a);
    line_end(UART_B, &q_b);
    return false;
  }
  on = true;
  baud_a = a;
  baud_b = b;
  err_a = LineErrors();
  err_b = LineErrors();
  err_shown_ms = millis();
  live = live_mem != NULL;
  if (live) ring.init(live_mem, live_size);
  char session[40];
  snprintf(session, sizeof(session), "sniff, A %u, B %u", (unsigned)a, (unsigned)b);
  ulog_start("%s", session);                 // after the drivers: the store is sized to what they left
  if (live) ring.event(micros(), session);
  return true;
}

void sniff_end(const char *why) {
  if (!on) return;
  sniff_capture();                           // what is still in the drivers
  if (why) sniff_event("%s", why);
  sniff_event("end");
  on = false;
  ulog_flush();
  line_end(UART_A, &q_a);
  line_end(UART_B, &q_b);
  pinMode(PIN_SWS_RX, INPUT);
  pinMode(PIN_SNIFF_B, INPUT);
}

bool sniff_on() { return on; }
uint32_t sniff_baud_a() { return baud_a; }
uint32_t sniff_baud_b() { return baud_b; }

void sniff_live_off() {
  live = false;
  ring.clear();
}
bool sniff_live_on() { return live; }
SniffRing &sniff_ring() { return ring; }

void sniff_event(const char *fmt, ...) {
  char text[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  ulog_event("%s", text);
  if (live) ring.event(micros(), text);
}

static bool take(uart_port_t port, int tag) {
  static uint8_t buf[512];
  size_t n = 0;
  uart_get_buffered_data_len(port, &n);
  bool any = false;
  while (n > 0) {
    int got = uart_read_bytes(port, buf, n < sizeof(buf) ? n : sizeof(buf), 0);
    if (got <= 0) break;
    any = true;
    ulog_data(tag, buf, got);
    if (live) ring.push(tag, micros(), buf, got);
    n -= got;
  }
  return any;
}

static void driver_events(QueueHandle_t q, LineErrors &e) {
  uart_event_t ev;
  while (q && xQueueReceive(q, &ev, 0) == pdTRUE) {
    if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) e.overflow++;
    else if (ev.type == UART_FRAME_ERR || ev.type == UART_PARITY_ERR || ev.type == UART_BREAK) e.errors++;
  }
}

static void show_errors(LineErrors &e, char name) {
  if (e.overflow != e.shown_overflow)
    sniff_event("line %c: receive buffer overflow, bytes lost (%u times)", name,
                (unsigned)(e.overflow - e.shown_overflow));
  if (e.errors != e.shown_errors)
    sniff_event("line %c: %u framing errors or breaks (wrong speed, or the line held low)", name,
                (unsigned)(e.errors - e.shown_errors));
  e.shown_overflow = e.overflow;
  e.shown_errors = e.errors;
}

bool sniff_capture() {
  if (!on) return false;
  // A first: when both have bytes, the order between them within this pass is not known anyway
  bool any = take(UART_A, SNIFF_TAG_A);
  any |= take(UART_B, SNIFF_TAG_B);
  driver_events(q_a, err_a);
  driver_events(q_b, err_b);
  if (millis() - err_shown_ms >= 1000) {     // at most one report a second per line and kind
    err_shown_ms = millis();
    show_errors(err_a, 'A');
    show_errors(err_b, 'B');
  }
  return any;
}

void sniff_boot_get(uint32_t *a, uint32_t *b) {
  *a = *b = 0;
  Preferences p;
  if (!p.begin("sniff", true)) return;       // nothing stored yet
  *a = p.getUInt("a", 0);
  *b = p.getUInt("b", 0);
  p.end();
}

void sniff_boot_set(uint32_t a, uint32_t b) {
  Preferences p;
  if (!p.begin("sniff", false)) return;
  p.putUInt("a", a);
  p.putUInt("b", b);
  p.end();
}
