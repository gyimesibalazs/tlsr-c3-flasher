// WCH single-wire debug interface (SDI, the "SWIO" pin of the CH32V003), line and DMI layer.
//
// From WCH's "QingKeV2 Microprocessor Debug Manual" (chapter 2) and WCH's own one-wire host on a CH32F103
// (openwch/ch32v003, CH32V003_1Line_Base_on_CH32F103):
//   - The line idles high. Every bit starts with a falling edge; a 1 is a short low (T..4T), a 0 a long low
//     (6T..64T), each followed by a high of T..16T. A high of 18T or more ends a packet (stop). T is the
//     period of the target's debug clock (8 MHz after reset, the HSI divided by 3), "normal mode 2x".
//   - A packet: start bit 1, 7-bit register address MSB first, 1 = host writes / 0 = host reads, 32 data
//     bits MSB first (an even parity bit may follow; WCH's host sends none, nor does this one).
//   - Reading: for each of the 32 bits the host sends a short low pulse; the target keeps the line low for
//     a 0 and lets it go for a 1. The target only answers after the "output enable" bit (OUTEN, bit 10) has
//     been set in the shadow configuration register 0x7E and then in the configuration register 0x7D, both
//     written with the key 0x5AA5 in the upper half.
//   - A low of 256T or longer resets the debug interface.
//
// Here the RMT peripheral does the timing, as in the Telink SWS mode: GPIO 4 drives the line through the 1 k
// resistor, so the high level is a strong pull-up the target can override, and GPIO 5 measures every low
// pulse on the line, ours and the target's. A read bit is 0 when the low pulse is longer than the threshold.
// Each packet is one RMT transaction of 41 symbols with the stop folded into the last high time; it fits
// into one channel memory block, so no refill interrupt can stretch a bit in the middle of a packet. Writes
// are queued and return at once; a read waits for the queue to empty, arms the receiver and sends.
//
// Plain bit-banging with interrupts off would also meet the timing (the windows are wide), but it blocks the
// CPU for every packet and its timing depends on the cache; the RMT gives fixed 12.5 ns steps and measures
// the answer instead of sampling it at one point.

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "config.h"
#include "wch.h"

static const uint8_t DM_DMCONTROL = 0x10, DM_DMSTATUS = 0x11, DM_CPBR = 0x7C, DM_CFGR = 0x7D, DM_SHDWCFGR = 0x7E;
static const uint32_t CFG_OUTPUT_ON = 0x5AA50400;   // key 0x5AA5 + OUTEN

static WchTiming tim = {WCH_ONE_LOW_NS, WCH_ZERO_LOW_NS, WCH_HIGH_NS, WCH_READ_SLOT_NS, WCH_READ_THRESHOLD_NS,
                        WCH_STOP_NS};
static char detail[112];

// ---------------------------------------------------------------- encoding (pure)

static uint32_t ticks(uint32_t ns) {
  uint32_t t = (ns * 2 + 12) / 25;           // 80 MHz: 12.5 ns per tick
  return t < 1 ? 1 : (t > 32767 ? 32767 : t);
}

static rmt_symbol_word_t pulse(uint32_t low_ns, uint32_t high_ns) {
  rmt_symbol_word_t s;
  s.val = 0;
  s.level0 = 0;
  s.duration0 = ticks(low_ns);
  s.level1 = 1;
  s.duration1 = ticks(high_ns);
  return s;
}

static int put_bits(rmt_symbol_word_t *o, uint32_t v, int nbits, const WchTiming &t) {
  for (int i = nbits - 1; i >= 0; i--) o[nbits - 1 - i] = pulse((v >> i) & 1 ? t.one_low : t.zero_low, t.high);
  return nbits;
}

static int put_header(uint8_t addr, bool write, const WchTiming &t, rmt_symbol_word_t *out) {
  int k = 0;
  out[k++] = pulse(t.one_low, t.high);                    // start bit
  k += put_bits(out + k, addr & 0x7f, 7, t);
  out[k++] = pulse(write ? t.one_low : t.zero_low, t.high);
  return k;
}

int wch_encode_write(uint8_t addr, uint32_t value, const WchTiming &t, rmt_symbol_word_t *out) {
  int k = put_header(addr, true, t, out);
  k += put_bits(out + k, value, 32, t);
  out[k - 1].duration1 = ticks(t.stop);
  return k;
}

int wch_encode_read(uint8_t addr, const WchTiming &t, rmt_symbol_word_t *out) {
  int k = put_header(addr, false, t, out);
  for (int i = 0; i < 32; i++) out[k++] = pulse(t.one_low, t.read_slot - t.one_low);
  out[k - 1].duration1 = ticks(t.stop);
  return k;
}

// A short high between our pulse and the target's answer (the target starts driving a little after our
// edge) would split one bit into two pulses: such highs (< 150 ns) are merged first.
int wch_decode_read(const rmt_symbol_word_t *in, int n, const WchTiming &t, uint32_t *value) {
  static rmt_symbol_word_t m[80];
  const uint32_t gap = ticks(150);
  int k = 0;
  for (int i = 0; i < n && k < 80; i++) {
    if (in[i].level0 != 0) {
      snprintf(detail, sizeof(detail), "pulse %d starts high", i);
      return WCH_ERR_FRAME;
    }
    if (k > 0 && m[k - 1].duration1 < gap && m[k - 1].duration1 != 0) {
      uint32_t d = m[k - 1].duration0 + m[k - 1].duration1 + in[i].duration0;
      m[k - 1].duration0 = d > 32767 ? 32767 : d;
      m[k - 1].duration1 = in[i].duration1;
    } else {
      m[k++] = in[i];
    }
  }
  if (k != 41) {
    snprintf(detail, sizeof(detail), "%d pulses instead of 41 (first low %u, last low %u ticks)", k,
             k ? (unsigned)m[0].duration0 : 0u, k ? (unsigned)m[k - 1].duration0 : 0u);
    return WCH_ERR_FRAME;
  }
  // our own header must come back as we sent it (each low near a 1 or a 0): otherwise something else
  // drives the line
  const uint32_t t1 = ticks(t.one_low), t0 = ticks(t.zero_low);
  for (int i = 0; i < 9; i++) {
    uint32_t d = m[i].duration0;
    bool near1 = d + 8 >= t1 * 3 / 5 && d <= t1 * 7 / 5 + 8;
    bool near0 = d + 8 >= t0 * 3 / 5 && d <= t0 * 7 / 5 + 8;
    if (!near1 && !near0) {
      snprintf(detail, sizeof(detail), "header pulse %d is %u ticks, sent %u or %u", i, (unsigned)d, (unsigned)t1,
               (unsigned)t0);
      return WCH_ERR_FRAME;
    }
  }
  const uint32_t thr = ticks(t.read_threshold);
  uint32_t v = 0;
  for (int i = 9; i < 41; i++) v = (v << 1) | (m[i].duration0 > thr ? 0u : 1u);
  *value = v;
  return WCH_OK;
}

// ---------------------------------------------------------------- RMT driver

static rmt_channel_handle_t tx_ch = NULL, rx_ch = NULL;
static rmt_encoder_handle_t enc = NULL;
static QueueHandle_t rxq = NULL;
static const int RING = 8;                   // packets queued at once; each needs its own buffer until sent
static rmt_symbol_word_t ring[RING][44];
static volatile uint32_t tx_done = 0;
static uint32_t tx_issued = 0;
static rmt_symbol_word_t rx_buf[96];

static bool IRAM_ATTR on_tx_done(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *) {
  tx_done = tx_done + 1;
  return false;
}

static bool IRAM_ATTR on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *ed, void *ctx) {
  BaseType_t woken = pdFALSE;
  size_t n = ed->num_symbols;
  xQueueSendFromISR((QueueHandle_t)ctx, &n, &woken);
  return woken == pdTRUE;
}

bool wch_active() { return tx_ch != NULL; }

void wch_end() {
  bool was_on = tx_ch || rx_ch;
  if (tx_ch) {
    rmt_tx_wait_all_done(tx_ch, 100);
    rmt_disable(tx_ch);
    rmt_del_channel(tx_ch);
    tx_ch = NULL;
  }
  if (rx_ch) {
    rmt_disable(rx_ch);
    rmt_del_channel(rx_ch);
    rx_ch = NULL;
  }
  if (enc) {
    rmt_del_encoder(enc);
    enc = NULL;
  }
  if (was_on) {                              // the SWS wire is quiet until the next mode takes it
    pinMode(PIN_SWS_TX, INPUT);
    pinMode(PIN_SWS_RX, INPUT);
  }
}

static int tx_packet(const rmt_symbol_word_t *s, int n, bool end_high) {
  uint32_t t0 = millis();
  while (tx_issued - tx_done >= (uint32_t)RING) {
    if (millis() - t0 > 200) return WCH_ERR_DRIVER;
  }
  rmt_symbol_word_t *slot = ring[tx_issued % RING];
  memcpy(slot, s, n * sizeof(rmt_symbol_word_t));
  rmt_transmit_config_t c = {};
  c.flags.eot_level = end_high ? 1 : 0;
  if (rmt_transmit(tx_ch, enc, slot, n * sizeof(rmt_symbol_word_t), &c) != ESP_OK) return WCH_ERR_DRIVER;
  tx_issued++;
  return WCH_OK;
}

void wch_line_release() {
  if (!tx_ch) return;
  rmt_symbol_word_t s;
  s.val = 0;
  s.level0 = 1;
  s.duration0 = ticks(tim.stop);
  s.level1 = 1;
  s.duration1 = ticks(tim.stop);
  tx_packet(&s, 1, true);
}

void wch_line_hold_low() {
  if (!tx_ch) return;
  rmt_symbol_word_t s;
  s.val = 0;
  s.level0 = 0;
  s.duration0 = ticks(10000);
  s.level1 = 0;
  s.duration1 = ticks(10000);
  tx_packet(&s, 1, false);
  rmt_tx_wait_all_done(tx_ch, 100);
}

void wch_line_reset() {
  if (!tx_ch) return;
  rmt_symbol_word_t s = pulse(WCH_LINE_RESET_US * 1000, tim.stop);   // a symbol holds up to 409 us
  tx_packet(&s, 1, true);
  rmt_tx_wait_all_done(tx_ch, 100);
}

bool wch_begin() {
  wch_end();
  if (!rxq) rxq = xQueueCreate(2, sizeof(size_t));
  rmt_tx_channel_config_t tc = {};
  tc.gpio_num = (gpio_num_t)PIN_SWS_TX;
  tc.clk_src = RMT_CLK_SRC_DEFAULT;
  tc.resolution_hz = 80000000;
  tc.mem_block_symbols = 48;                 // a packet (41 symbols + end marker) fits in one block
  tc.trans_queue_depth = RING;
  rmt_rx_channel_config_t rc = {};
  rc.gpio_num = (gpio_num_t)PIN_SWS_RX;
  rc.clk_src = RMT_CLK_SRC_DEFAULT;
  rc.resolution_hz = 80000000;
  rc.mem_block_symbols = 48;
  rmt_copy_encoder_config_t ec = {};
  rmt_tx_event_callbacks_t tcb = {};
  tcb.on_trans_done = on_tx_done;
  rmt_rx_event_callbacks_t rcb = {};
  rcb.on_recv_done = on_rx_done;
  if (rmt_new_tx_channel(&tc, &tx_ch) != ESP_OK) {
    tx_ch = NULL;
    return false;
  }
  if (rmt_new_rx_channel(&rc, &rx_ch) != ESP_OK || rmt_new_copy_encoder(&ec, &enc) != ESP_OK ||
      rmt_tx_register_event_callbacks(tx_ch, &tcb, NULL) != ESP_OK ||
      rmt_rx_register_event_callbacks(rx_ch, &rcb, rxq) != ESP_OK || rmt_enable(tx_ch) != ESP_OK ||
      rmt_enable(rx_ch) != ESP_OK) {
    wch_end();
    return false;
  }
  tx_done = 0;
  tx_issued = 0;
  gpio_pullup_en((gpio_num_t)PIN_SWS_RX);
  wch_line_release();                        // the output idles low until the first transmission
  rmt_tx_wait_all_done(tx_ch, 100);
  return true;
}

void wch_set_timing(const WchTiming &t) { tim = t; }
WchTiming wch_timing() { return tim; }
const char *wch_last_detail() { return detail; }

int wch_dmi_write(uint8_t addr, uint32_t value) {
  if (!tx_ch) return WCH_ERR_NOTACTIVE;
  rmt_symbol_word_t s[44];
  int n = wch_encode_write(addr, value, tim, s);
  return tx_packet(s, n, true);
}

int wch_dmi_read(uint8_t addr, uint32_t *value) {
  if (!tx_ch) return WCH_ERR_NOTACTIVE;
  rmt_symbol_word_t s[44];
  int n = wch_encode_read(addr, tim, s);
  if (rmt_tx_wait_all_done(tx_ch, 200) != ESP_OK) return WCH_ERR_DRIVER;
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 100;              // glitch filter
  uint32_t longest = max(max(tim.read_slot, tim.zero_low + tim.high), (uint32_t)1500);
  rc.signal_range_max_ns = min(longest * 2, tim.stop - 500);   // the stop ends the reception
  xQueueReset(rxq);
  if (rmt_receive(rx_ch, rx_buf, sizeof(rx_buf), &rc) != ESP_OK) return WCH_ERR_DRIVER;
  int r = tx_packet(s, n, true);
  if (r != WCH_OK) return r;
  size_t got = 0;
  if (xQueueReceive(rxq, &got, pdMS_TO_TICKS(10)) != pdTRUE) {
    // no edge at all: the reception never ends. Cancel it and look at the level the line rests at.
    rmt_disable(rx_ch);
    rmt_enable(rx_ch);
    bool high = gpio_get_level((gpio_num_t)PIN_SWS_RX) != 0;
    snprintf(detail, sizeof(detail), "no pulse seen, the line rests %s", high ? "high" : "low");
    return high ? WCH_ERR_NOPULSE : WCH_ERR_STUCKLOW;
  }
  detail[0] = 0;
  return wch_decode_read(rx_buf, (int)got, tim, value);
}

// ---------------------------------------------------------------- attach

int wch_attach(void (*power)(bool on), uint32_t off_ms, uint32_t window_ms, WchAttach *a) {
  memset(a, 0, sizeof(*a));
  if (!tx_ch) return WCH_ERR_NOTACTIVE;
  a->powered = power != NULL && off_ms > 0;
  if (a->powered) {
    wch_line_hold_low();                     // an unpowered chip must not be fed through its SWIO pin
    power(false);
    delay(off_ms);
  } else {
    wch_line_reset();
  }
  uint32_t t0 = micros();
  if (a->powered) {
    power(true);
    t0 = micros();
    wch_line_release();
  }
  uint32_t v = 0;
  for (;;) {
    a->tries++;
    // The halt request is a plain write that needs no answer, so it is sent often: about every 0.2 ms. The
    // first write after the debug module comes up only sets dmactive (the debug spec ignores the other bits
    // while dmactive is 0), hence always two in a row. The output enable is needed for the read only.
    wch_dmi_write(DM_DMCONTROL, 0x80000001);
    wch_dmi_write(DM_DMCONTROL, 0x80000001);
    wch_dmi_write(DM_SHDWCFGR, CFG_OUTPUT_ON);
    wch_dmi_write(DM_CFGR, CFG_OUTPUT_ON);
    wch_dmi_write(DM_DMCONTROL, 0x80000001);
    wch_dmi_write(DM_DMCONTROL, 0x80000001);
    int r = wch_dmi_read(DM_DMSTATUS, &v);
    if (r == WCH_OK && v != 0xffffffff && (v & 0x8f) == 0x82) {   // authenticated, debug spec 0.13
      a->first_us = micros() - t0;
      break;
    }
    if (r == WCH_ERR_NOPULSE) a->n_nopulse++;
    else if (r == WCH_ERR_STUCKLOW) a->n_stucklow++;
    else if (r == WCH_ERR_FRAME) a->n_frame++;
    else if (r == WCH_OK && v == 0xffffffff) a->n_ones++;
    else a->n_other++;
    if (micros() - t0 > window_ms * 1000) return WCH_ERR_NOANSWER;
  }
  for (int i = 0; i < 50 && (v & 0x300) != 0x300; i++) {
    wch_dmi_write(DM_DMCONTROL, 0x80000001);
    if (wch_dmi_read(DM_DMSTATUS, &v) != WCH_OK) v = 0;
  }
  a->dmstatus = v;
  a->halted = (v & 0x300) == 0x300;
  wch_dmi_write(DM_DMCONTROL, 0x00000001);   // clear the halt request, as WCH's own host does
  if (wch_dmi_read(DM_CPBR, &a->cpbr) != WCH_OK) a->cpbr = 0;
  return a->halted ? WCH_OK : WCH_ERR_NOTHALTED;
}

const char *wch_attach_reason(const WchAttach &a) {
  static char out[512];
  uint32_t most = max(max(a.n_nopulse, a.n_stucklow), max(max(a.n_frame, a.n_ones), a.n_other));
  const char *why;
  if (most == 0) why = "no try was made";
  else if (most == a.n_nopulse)
    why = a.powered ? "SWIO never went low: the target drives the pin high (on the SOP8 pin 8 is also UART TX "
                      "and GPIO) and was not caught in time after power-on, or the GPIO 4 - 1 k - GPIO 5 wire is open"
                    : "SWIO never went low: the target drives the pin high (on the SOP8 pin 8 is also UART TX and "
                      "GPIO), or the GPIO 4 - 1 k - GPIO 5 wire is open. With external power there is no catch at "
                      "power-up: power the target from the power pins";
  else if (most == a.n_stucklow) why = "SWIO stays low: the target has no power (VDD, GND) or drives the pin low";
  else if (most == a.n_ones)
    why = "the line works but nothing answers (all ones): wrong pin, no power, debug output not enabled, or not a "
          "WCH RISC-V chip";
  else if (most == a.n_frame) why = "the answers are garbled: line timing (WT), a long or loaded wire";
  else why = "the answers do not look like a debug module status";
  snprintf(out, sizeof(out), "%s (%u tries: %u no pulse, %u stuck low, %u garbled, %u all ones, %u other; %s)", why,
           (unsigned)a.tries, (unsigned)a.n_nopulse, (unsigned)a.n_stucklow, (unsigned)a.n_frame, (unsigned)a.n_ones,
           (unsigned)a.n_other, detail[0] ? detail : "-");
  return out;
}

const char *wch_err_name(int r) {
  switch (r) {
    case WCH_OK: return "ok";
    case WCH_ERR_DRIVER: return "RMT driver error";
    case WCH_ERR_NOPULSE: return "no pulse on SWIO (pin driven high by the target, or wire open)";
    case WCH_ERR_STUCKLOW: return "SWIO stuck low (no target power?)";
    case WCH_ERR_FRAME: return "garbled answer";
    case WCH_ERR_NOANSWER: return "no answer";
    case WCH_ERR_BUSY: return "abstract command busy";
    case WCH_ERR_CMD: return "abstract command failed";
    case WCH_ERR_NOTHALTED: return "core not halted";
    case WCH_ERR_TIMEOUT: return "flash busy timeout";
    case WCH_ERR_WRPRT: return "flash write protection error";
    case WCH_ERR_LOCKED: return "flash controller did not unlock";
    case WCH_ERR_ADDRESS: return "address or length not allowed";
    case WCH_ERR_VERIFY: return "read back differs";
    case WCH_ERR_NOTACTIVE: return "not in WCH mode";
  }
  return "error";
}
