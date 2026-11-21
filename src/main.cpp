// ESP32-C3 as a Telink TLSR825x SWire flasher.
//
// Two interchangeable ways of driving the SWS line, on the same wiring (TX -> 1k -> SWS, RX -> SWS):
//   UART mode  a port of pvvx's TLSR825xComFlasher.py (https://github.com/pvvx/TlsrComSwireWriter, public
//              domain): one UART byte per SWire bit, the chip is slowed down to baud / 10.
//   RMT mode   the RMT peripheral generates and measures the pulses (12.5 ns steps). Reads are decoded from
//              the measured low / high ratio, so they do not depend on the chip's speed.
//
// SWire, from Telink's description (pvvx/TLSRPGM/TelinkSWire): a bit is 5 units, 0 = 1 low + 4 high,
// 1 = 4 low + 1 high. A byte is a command bit, 8 data bits MSB first and one unit of low. A message is
// START (0x5a, command bit set), address, RW_ID (bit 7 = read), data, END (0xff, command bit set). To read,
// the master sends one unit of low for every byte; once the line has been high for longer than that, the
// slave sends 8 bits and one unit of low at its own speed (system clock / 5 / reg 0xb2).
//
// Host protocol: one text command per line, the answer is one line starting with "OK" or "ERR".
// Numbers are decimal or 0x-hex, data is hex without separators. See tlsr_c3.py.

#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "config.h"
#include "swd.h"
#include "swdflash.h"
#include "spiflash.h"

#define FW_NAME "tlsr-c3-flasher"
#define FW_VERSION "0.18"

enum SwsMode { MODE_UART = 0, MODE_RMT = 1 };

static const uint32_t FLASH_SECTOR = 4096;
static const uint32_t FLASH_PAGE = 256;
static const int MAX_READ = 1024;       // bytes per FR command
static const int PAGE_PROGRAM_MIN_MS = 2;   // wait after a page program even if the flash reports ready
static const int MAX_MSG = 5 + FLASH_PAGE + 8;   // bytes of one SWire write message without END

static SwsMode mode = MODE_UART;
static int read_burst = 64;             // bytes requested in one go (1 = one byte at a time)

static uint8_t data_buf[MAX_READ];
static char line[1400];

// ================================================================ UART mode

static const uart_port_t SWS_UART = UART_NUM_1;
static const int SWS_RX_BUF = 8192;
static const int READ_IDLE_BITS = 100;  // gap after each read start pulse; the answer takes 86 bit times

static uint32_t sws_baud = SWS_BAUD_DEFAULT;
static uint8_t txbuf[(MAX_MSG + 1) * 10 + 16];
static uint8_t rxbuf[sizeof(txbuf)];

static char read_dbg[96];                  // what the last failed read saw, for the error message

static void uart_teardown() {
  if (uart_is_driver_installed(SWS_UART)) uart_driver_delete(SWS_UART);
}

static bool uart_setup() {
  uart_config_t cfg = {};
  cfg.baud_rate = (int)sws_baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  uart_teardown();
  if (uart_driver_install(SWS_UART, SWS_RX_BUF, 0, 0, NULL, 0) != ESP_OK) return false;
  uart_param_config(SWS_UART, &cfg);
  uart_set_pin(SWS_UART, PIN_SWS_TX, PIN_SWS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_rx_timeout(SWS_UART, 2);          // hand received bytes over after 2 idle character times
  uart_set_rx_full_threshold(SWS_UART, 64);
  uart_set_tx_idle_num(SWS_UART, 0);
  gpio_pullup_en((gpio_num_t)PIN_SWS_RX);    // idle level of the line is high
  return true;
}

static void u_flush_in() { uart_flush_input(SWS_UART); }

static void u_tx(const uint8_t *buf, int len) {
  uart_write_bytes(SWS_UART, (const char *)buf, len);
  uart_wait_tx_done(SWS_UART, pdMS_TO_TICKS(500));
}

// send and read back the echo (RX hears TX); false = the echo is incomplete (wiring problem)
static bool u_txrx(const uint8_t *buf, int len) {
  u_tx(buf, len);
  int got = 0;
  while (got < len) {
    int n = uart_read_bytes(SWS_UART, rxbuf, min(len - got, (int)sizeof(rxbuf)), pdMS_TO_TICKS(30));
    if (n <= 0) break;
    got += n;
  }
  return got == len;
}

// One UART byte per SWire bit: 0x80 = low for 8 of 10 bit times = '1', 0xfe = low for 2 of 10 = '0'.
static int u_encode(const uint8_t *blk, int n, uint8_t *out) {
  int k = 0;
  for (int i = 0; i < n; i++) {
    out[k++] = (i == 0) ? 0x80 : 0xfe;       // command bit: set on the first byte only
    for (uint8_t m = 0x80; m; m >>= 1) out[k++] = (blk[i] & m) ? 0x80 : 0xfe;
    out[k++] = 0xfe;                         // one unit of low closes the byte
  }
  return k;
}

// The master's start pulse and the slave's first bit land in the same UART character (the slave starts
// 4 bit times after our edge), hence the later sampling point for the first bit.
static int u_decode(const uint8_t *b) {
  if ((b[8] & 0xfe) != 0xfe) return -1;
  int d = 0;
  uint8_t mask = 0x20;
  for (int i = 0; i < 8; i++) {
    d <<= 1;
    if ((b[i] & mask) == 0) d |= 1;
    mask = 0x10;
  }
  return d;
}

static int u_enc_end(uint8_t *out) {
  const uint8_t e = 0xff;
  return u_encode(&e, 1, out);
}

static bool u_stop() {
  int k = u_enc_end(txbuf);
  return u_txrx(txbuf, k);
}

static bool u_write(const uint8_t *msg, int n, bool echo) {
  int k = u_encode(msg, n, txbuf);
  k += u_enc_end(txbuf + k);
  if (!echo) { u_tx(txbuf, k); return true; }
  return u_txrx(txbuf, k);
}

static void u_note_fail(int got, int want) {
  int k = snprintf(read_dbg, sizeof(read_dbg), "read failed got=%d want=%d raw=", got, want);
  for (int i = 0; i < got && i < 27 && k < (int)sizeof(read_dbg) - 3; i++)
    k += snprintf(read_dbg + k, sizeof(read_dbg) - k, "%02x", rxbuf[i]);
}

static bool u_read(const uint8_t *head, uint8_t *out, int n) {
  delay(1);
  u_flush_in();
  int k = u_encode(head, 5, txbuf);
  u_txrx(txbuf, k);
  bool ok = true;
  int done = 0;
  while (done < n && ok) {
    int cnt = min(n - done, read_burst);
    // One start pulse per byte, the answer (86 bit times) fills the gap before the next one. The gap is
    // timed here: the UART's own idle setting only acts when its FIFO runs empty, not between characters.
    const uint8_t start = 0xfe;
    const uint32_t gap_us = (uint32_t)(READ_IDLE_BITS + 10) * 1000000UL / sws_baud + 5;
    for (int i = 0; i < cnt; i++) {
      uart_write_bytes(SWS_UART, (const char *)&start, 1);
      if (cnt > 1) delayMicroseconds(gap_us);
    }
    int want = cnt * 9, got = 0;
    while (got < want) {
      int r = uart_read_bytes(SWS_UART, rxbuf + got, want - got, pdMS_TO_TICKS(40));
      if (r <= 0) break;
      got += r;
    }
    if (got != want) { u_note_fail(got, want); ok = false; break; }
    for (int i = 0; i < cnt; i++) {
      int v = u_decode(rxbuf + i * 9);
      if (v < 0) { u_note_fail(got, want); ok = false; break; }
      out[done++] = (uint8_t)v;
    }
  }
  delay(1);
  u_flush_in();
  u_stop();
  return ok;
}

// ================================================================ RMT mode

static const uint32_t RMT_HZ = 80000000;     // 12.5 ns per tick
static const uint32_t RMT_PROBE_RATE = 200000;
static const int RMT_GAP_UNITS = 50;         // high time after a read start pulse; the answer takes ~43 units
static const int RMT_IDLE_UNITS = 20;        // line idle for this long ends a reception

static rmt_channel_handle_t r_tx = NULL, r_rx = NULL;
static rmt_encoder_handle_t r_enc = NULL;
static QueueHandle_t r_rxq = NULL;
static int r_tx_mem = 96, r_rx_mem = 96;     // symbols the channels can hold
static uint32_t sws_rate = SWS_RATE_DEFAULT; // requested bit rate
static uint32_t r_unit = RMT_HZ / (5 * RMT_PROBE_RATE);   // ticks per unit on our side
static uint32_t r_bit_ticks = 0;             // slave bit period measured during the last read
static rmt_symbol_word_t r_sym[(MAX_MSG + 1) * 10 + 16];
static const int RMT_MAX_BURST = 256;        // bytes per read burst
static rmt_symbol_word_t r_rxsym[RMT_MAX_BURST * 10 + 16];

static uint32_t rate_to_unit(uint32_t rate) { return max((uint32_t)4, RMT_HZ / (5 * rate)); }

static bool IRAM_ATTR r_rx_done(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t *ed, void *ctx) {
  BaseType_t woken = pdFALSE;
  size_t n = ed->num_symbols;
  xQueueSendFromISR((QueueHandle_t)ctx, &n, &woken);
  return woken == pdTRUE;
}

static void rmt_teardown() {
  if (r_tx) { rmt_disable(r_tx); rmt_del_channel(r_tx); r_tx = NULL; }
  if (r_rx) { rmt_disable(r_rx); rmt_del_channel(r_rx); r_rx = NULL; }
  if (r_enc) { rmt_del_encoder(r_enc); r_enc = NULL; }
}

static bool r_send(const rmt_symbol_word_t *s, int n);

static bool rmt_setup() {
  rmt_teardown();
  if (!r_rxq) r_rxq = xQueueCreate(2, sizeof(size_t));

  rmt_tx_channel_config_t tc = {};
  tc.gpio_num = (gpio_num_t)PIN_SWS_TX;
  tc.clk_src = RMT_CLK_SRC_DEFAULT;
  tc.resolution_hz = RMT_HZ;
  tc.trans_queue_depth = 32;
  r_tx_mem = 96;
  tc.mem_block_symbols = r_tx_mem;
  if (rmt_new_tx_channel(&tc, &r_tx) != ESP_OK) {
    r_tx_mem = 48;
    tc.mem_block_symbols = r_tx_mem;
    if (rmt_new_tx_channel(&tc, &r_tx) != ESP_OK) { r_tx = NULL; return false; }
  }

  rmt_rx_channel_config_t rc = {};
  rc.gpio_num = (gpio_num_t)PIN_SWS_RX;
  rc.clk_src = RMT_CLK_SRC_DEFAULT;
  rc.resolution_hz = RMT_HZ;
  r_rx_mem = 96;
  rc.mem_block_symbols = r_rx_mem;
  if (rmt_new_rx_channel(&rc, &r_rx) != ESP_OK) {
    r_rx_mem = 48;
    rc.mem_block_symbols = r_rx_mem;
    if (rmt_new_rx_channel(&rc, &r_rx) != ESP_OK) { r_rx = NULL; rmt_teardown(); return false; }
  }

  rmt_copy_encoder_config_t ec = {};
  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = r_rx_done;
  if (rmt_new_copy_encoder(&ec, &r_enc) != ESP_OK ||
      rmt_rx_register_event_callbacks(r_rx, &cbs, r_rxq) != ESP_OK ||
      rmt_enable(r_tx) != ESP_OK || rmt_enable(r_rx) != ESP_OK) {
    rmt_teardown();
    return false;
  }
  gpio_pullup_en((gpio_num_t)PIN_SWS_RX);

  // the output idles low until the first transmission: send a "high" symbol to park the line high
  rmt_symbol_word_t park = {};
  park.level0 = 1; park.duration0 = 40; park.level1 = 1; park.duration1 = 40;
  r_send(&park, 1);
  r_unit = rate_to_unit(RMT_PROBE_RATE);
  delay(2);
  return true;
}

// Queue the symbols in pieces that fit the channel memory (cut on SWire byte boundaries), so that no
// refill interrupt is needed: a late interrupt can then only lengthen the pause between two bytes.
static bool r_send(const rmt_symbol_word_t *s, int n) {
  rmt_transmit_config_t c = {};
  c.flags.eot_level = 1;
  int chunk = ((r_tx_mem - 6) / 10) * 10;
  for (int i = 0; i < n; i += chunk) {
    int cnt = min(chunk, n - i);
    if (rmt_transmit(r_tx, r_enc, s + i, cnt * sizeof(rmt_symbol_word_t), &c) != ESP_OK) return false;
  }
  return rmt_tx_wait_all_done(r_tx, 2000) == ESP_OK;
}

static rmt_symbol_word_t r_bit(bool one) {
  rmt_symbol_word_t s = {};
  s.level0 = 0; s.duration0 = one ? 4 * r_unit : r_unit;
  s.level1 = 1; s.duration1 = one ? r_unit : 4 * r_unit;
  return s;
}

static int r_encode(const uint8_t *blk, int n, rmt_symbol_word_t *out) {
  int k = 0;
  for (int i = 0; i < n; i++) {
    out[k++] = r_bit(i == 0);                // command bit: set on the first byte only
    for (uint8_t m = 0x80; m; m >>= 1) out[k++] = r_bit(blk[i] & m);
    out[k++] = r_bit(false);                 // one unit of low closes the byte
  }
  return k;
}

static int r_enc_end(rmt_symbol_word_t *out) {
  const uint8_t e = 0xff;
  return r_encode(&e, 1, out);
}

static bool r_stop() {
  int k = r_enc_end(r_sym);
  return r_send(r_sym, k);
}

static bool r_write(const uint8_t *msg, int n) {
  int k = r_encode(msg, n, r_sym);
  k += r_enc_end(r_sym + k);
  return r_send(r_sym, k);
}

static void r_rx_config(rmt_receive_config_t *rc) {
  uint32_t unit_ns = r_unit * 1000 / (RMT_HZ / 1000000);
  rc->signal_range_min_ns = min((uint32_t)60, unit_ns / 3);      // glitch filter
  rc->signal_range_max_ns = RMT_IDLE_UNITS * unit_ns;
}

// Each requested byte shows up as 10 low pulses: our start pulse, 8 data bits, the closing unit.
// A data bit is 1 when its low part is longer than its high part.
// A burst may be longer than the channel memory: the driver copies the memory out in halves while the
// reception runs (ping-pong). If an interrupt comes too late, symbols are lost and the count is wrong,
// so the burst fails instead of returning shifted data.
static bool r_read(const uint8_t *head, uint8_t *out, int n) {
  int k = r_encode(head, 5, r_sym);
  if (!r_send(r_sym, k)) return false;
  rmt_receive_config_t rc = {};
  r_rx_config(&rc);
  bool ok = true;
  int done = 0;
  uint32_t period_sum = 0, period_cnt = 0;
  while (done < n && ok) {
    int cnt = min(n - done, min(read_burst, RMT_MAX_BURST));
    xQueueReset(r_rxq);
    if (rmt_receive(r_rx, r_rxsym, sizeof(r_rxsym), &rc) != ESP_OK) { ok = false; break; }
    for (int i = 0; i < cnt; i++) {
      r_sym[i].level0 = 0; r_sym[i].duration0 = r_unit;
      r_sym[i].level1 = 1; r_sym[i].duration1 = RMT_GAP_UNITS * r_unit;
    }
    size_t got = 0;
    if (!r_send(r_sym, cnt) || xQueueReceive(r_rxq, &got, pdMS_TO_TICKS(200)) != pdTRUE ||
        (int)got != cnt * 10) {
      snprintf(read_dbg, sizeof(read_dbg), "read failed: %d symbols instead of %d (burst %d)", (int)got,
               cnt * 10, cnt);
      ok = false;
      break;
    }
    for (int j = 0; j < cnt && ok; j++) {
      const rmt_symbol_word_t *s = &r_rxsym[j * 10];
      int v = 0;
      for (int b = 1; b <= 8; b++) {
        if (s[b].level0 != 0) { ok = false; break; }
        v = (v << 1) | (s[b].duration0 > s[b].duration1 ? 1 : 0);
        period_sum += s[b].duration0 + s[b].duration1;
        period_cnt++;
      }
      out[done++] = (uint8_t)v;
    }
  }
  if (period_cnt) r_bit_ticks = period_sum / period_cnt;
  r_stop();
  return ok;
}

// ================================================================ mode-independent SWire access

static bool swd_on = false;
static void bridge_end();

static bool set_mode(SwsMode m) {
  bridge_end();
  spif_end();
  if (swd_on) {
    swd_end();
    swd_on = false;
  }
  uart_teardown();
  rmt_teardown();
  mode = m;
  return m == MODE_UART ? uart_setup() : rmt_setup();
}

static int build_msg(uint32_t addr, uint8_t rw, const uint8_t *data, int n, uint8_t *msg) {
  msg[0] = 0x5a; msg[1] = addr >> 16; msg[2] = addr >> 8; msg[3] = addr; msg[4] = rw;
  if (n) memcpy(msg + 5, data, n);
  return 5 + n;
}

static bool sws_write_ex(uint32_t addr, const uint8_t *data, int n, bool echo) {
  static uint8_t msg[MAX_MSG];
  if (n > MAX_MSG - 5) return false;
  int len = build_msg(addr, 0x00, data, n, msg);
  return mode == MODE_UART ? u_write(msg, len, echo) : r_write(msg, len);
}

static bool sws_write(uint32_t addr, const uint8_t *data, int n) { return sws_write_ex(addr, data, n, true); }
static bool sws_write1(uint32_t addr, uint8_t v) { return sws_write(addr, &v, 1); }

// read n bytes starting at addr (in fifo mode: n times the same register)
static bool sws_read(uint32_t addr, uint8_t *out, int n) {
  uint8_t head[5];
  build_msg(addr, 0x80, NULL, 0, head);
  return mode == MODE_UART ? u_read(head, out, n) : r_read(head, out, n);
}

static bool sws_stop() { return mode == MODE_UART ? u_stop() : r_stop(); }
static void sws_flush_in() { if (mode == MODE_UART) u_flush_in(); }

// ---------------------------------------------------------------- chip control

static const uint8_t pwr_pins[] = PWR_PINS;
static uint32_t pwr_mask = 0;

// all power pins change with one register write: joined pins must never drive against each other.
// They are inputs until the target is powered the first time (an unused pin touching something must not
// short it), and again after PWR z.
static void target_power(bool on) {
  REG_WRITE(on ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, pwr_mask);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, pwr_mask);
}

static void target_power_release() { REG_WRITE(GPIO_ENABLE_W1TC_REG, pwr_mask); }

// reset wire and power pins together: a target wired to either of them restarts
static void target_reset_pulse() {
  digitalWrite(PIN_RST, LOW);
  target_power(false);                       // also discharges the target's supply
  delay(PWR_OFF_MS);
  target_power(true);
  digitalWrite(PIN_RST, HIGH);               // open drain: released
}

// reset the chip and stall its CPU before the firmware can switch SWS off or go to sleep
static void activate(uint32_t tact_ms) {
  const uint8_t soft_reset = 0x20, stall = 0x05;
  if (mode == MODE_RMT) r_unit = rate_to_unit(RMT_PROBE_RATE);
  target_reset_pulse();
  sws_write_ex(0x006f, &soft_reset, 1, false);
  uint32_t t0 = millis();
  while (millis() - t0 < tact_ms) {
    for (int i = 0; i < 5; i++) sws_write_ex(0x0602, &stall, 1, false);
    sws_flush_in();
  }
  delay(10);
  sws_flush_in();
  sws_stop();
  sws_write1(0x0602, stall);
  delay(10);
  sws_flush_in();
}

// UART mode: find the SWire divider (reg 0xb2) at which the chip answers at our UART speed; 0 = not found
static int match_first = 0, match_last = 0;   // working divider window of the last speed match

static bool uart_try_div(int div) {
  uint8_t blk[9], cmp[10], head[5];
  build_msg(0x00b2, 0x80, NULL, 0, head);
  sws_write1(0x00b2, (uint8_t)div);
  int k = u_encode(head, 5, txbuf);
  u_txrx(txbuf, k);
  const uint8_t start = 0xfe;
  uart_write_bytes(SWS_UART, (const char *)&start, 1);
  int got = uart_read_bytes(SWS_UART, blk, 9, pdMS_TO_TICKS(30));
  u_stop();
  bool ok = false;
  if (got == 9 && blk[8] == 0xfe) {
    const uint8_t d = (uint8_t)div;
    u_encode(&d, 1, cmp);
    ok = (blk[0] & 0xc0) == 0xc0 && blk[1] == cmp[2] && blk[2] == cmp[3] && blk[4] == cmp[5] &&
         blk[6] == cmp[7] && blk[7] == cmp[8];
  }
  if (!ok) u_flush_in();
  return ok;
}

// Several neighbouring dividers answer correctly; the first one found is at the edge of that window and
// reads fail now and then. Scan the whole window and use its middle.
static int uart_match_speed() {
  int div = (int)lround(16000000.0 * 2 / sws_baud);
  int div_max = min((int)lround(48000000.0 * 2 / sws_baud), 0x7f);
  int first = 0, last = 0;
  for (; div <= div_max; div++) {
    if (uart_try_div(div)) {
      if (!first) first = div;
      last = div;
    } else if (first) {
      break;
    }
  }
  if (!first) {
    sws_write1(0x00b2, 0x05);                // back to the chip's default
    return 0;
  }
  int mid = (first + last + 1) / 2;
  sws_write1(0x00b2, (uint8_t)mid);
  match_first = first;
  match_last = last;
  return mid;
}

// RMT mode: read the current divider at a slow, safe speed, measure the chip's bit period, work out its
// clock, then set the divider closest to the requested bit rate and use the same unit ourselves.
static int rmt_match_speed(double *clk_hz) {
  int saved_burst = read_burst;
  read_burst = 1;
  r_unit = rate_to_unit(RMT_PROBE_RATE);
  uint8_t cur = 0, back = 0;
  int div = 0;
  r_bit_ticks = 0;
  if (sws_read(0x00b2, &cur, 1) && cur != 0 && r_bit_ticks != 0) {
    double fclk = 5.0 * cur * RMT_HZ / r_bit_ticks;
    div = (int)lround(fclk / (5.0 * sws_rate));
    div = max(2, min(0x7f, div));
    sws_write1(0x00b2, (uint8_t)div);
    r_unit = max((uint32_t)4, (uint32_t)lround((double)RMT_HZ * div / fclk));
    if (sws_read(0x00b2, &back, 1) && back == div) {
      *clk_hz = fclk;
    } else {
      r_unit = rate_to_unit(RMT_PROBE_RATE);
      sws_write1(0x00b2, cur);
      div = 0;
    }
  }
  read_burst = saved_burst;
  return div;
}

// ---------------------------------------------------------------- flash through the SPI registers

// Write a pattern into the chip's RAM (the CPU is stalled) and read it back, one byte at a time and in a
// burst. Catches a speed at which reads are consistently wrong: two identical wrong reads look fine.
static char check_dbg[96];

static bool sws_readback_check() {
  static const uint8_t head[8] = {0x00, 0xff, 0xa5, 0x5a, 0x80, 0x7f, 0x01, 0xfe};
  static uint8_t pat[256], back[256];
  const uint32_t addr = 0x840000;
  const int len = sizeof(pat);
  for (int i = 0; i < len; i++) pat[i] = i < 8 ? head[i] : (uint8_t)(i * 37 + 11);
  int saved = read_burst;
  bool ok = true;
  for (int i = 0; i < len && ok; i += 64) ok = sws_write(addr + i, pat + i, 64);
  for (int pass = 0; pass < 2 && ok; pass++) {
    int cnt = pass ? len : 8;                // slow single bytes first, then at the burst in use
    read_burst = pass ? saved : 1;
    memset(back, 0, sizeof(back));
    ok = sws_read(addr, back, cnt) && !memcmp(back, pat, cnt);
  }
  read_burst = saved;
  if (!ok) {
    int k = snprintf(check_dbg, sizeof(check_dbg), "read-back check failed at this speed / burst; read ");
    for (int i = 0; i < 8; i++) k += snprintf(check_dbg + k, sizeof(check_dbg) - k, "%02x", back[i]);
  }
  return ok;
}

static void spi_cs(bool high) { sws_write1(0x0d, high ? 0x01 : 0x00); }
static void fifo_mode(bool on) { sws_write1(0x00b3, on ? 0x80 : 0x00); }

// several bytes into the SPI data register without address increment
static void spi_fifo_write(const uint8_t *data, int n) {
  fifo_mode(true);
  sws_write(0x0c, data, n);
  fifo_mode(false);
}

// Status register. As in the chip's own flash code (mspi_read), a dummy byte has to be written after the
// command to clock the answer in; pvvx's SWire script reads without it.
static bool flash_status(uint8_t *st) {
  spi_cs(false);
  sws_write1(0x0c, 0x05);
  sws_write1(0x0c, 0x00);
  bool ok = sws_read(0x0c, st, 1);
  spi_cs(true);
  return ok;
}

static bool flash_ready(int tries, uint8_t *status) {
  uint8_t st = 0xff;
  for (int i = 0; i < tries; i++) {
    if (!flash_status(&st)) return false;
    if ((st & 0x01) == 0) { if (status) *status = st; return true; }
    delay(2);
  }
  if (status) *status = st;
  return false;
}

static void flash_cmd(uint8_t cmd) {
  spi_cs(false);
  sws_write1(0x0c, cmd);
  spi_cs(true);
}

static void flash_write_enable() { flash_cmd(0x06); }

// Proof that the status register is really read: the write-enable latch (bit 1) must follow the
// write enable / write disable commands. Without it "flash ready" could be a constant.
static bool flash_status_works() {
  uint8_t on = 0, off = 0xff;
  flash_cmd(0x06);
  if (!flash_status(&on)) return false;
  flash_cmd(0x04);
  if (!flash_status(&off)) return false;
  return (on & 0x03) == 0x02 && (off & 0x03) == 0x00;
}

// clear the block protection bits; 0 = ok, 1 = status cannot be read reliably, 2 = still protected
static int flash_unlock(uint8_t *status) {
  *status = 0xff;
  if (!flash_status_works()) return 1;
  flash_write_enable();
  spi_cs(false);
  sws_write1(0x0c, 0x01);                    // write status
  sws_write1(0x0c, 0x00);
  spi_cs(true);
  if (!flash_ready(100, status)) return 1;
  return *status == 0 ? 0 : 2;
}

// read after an SPI command: the trailing 0 clocks the first byte in, then auto-read mode
static bool spi_cmd_read(const uint8_t *cmd, int cmd_len, uint8_t *out, int n) {
  uint8_t c[8];
  memcpy(c, cmd, cmd_len);
  c[cmd_len] = 0;
  fifo_mode(true);
  spi_cs(false);
  sws_write(0x0c, c, cmd_len + 1);
  sws_write1(0x0d, 0x0a);                    // auto read mode, cs low
  bool ok = sws_read(0x0c, out, n);
  spi_cs(true);
  fifo_mode(false);
  return ok;
}

static uint64_t stat_read_us = 0, stat_reply_us = 0;   // time spent on the wire / sending answers
static uint32_t stat_read_bytes = 0;

static bool flash_read(uint32_t addr, uint8_t *out, int n) {
  uint32_t t0 = micros();
  stat_read_bytes += n;
  while (n > 0) {
    int cnt = min(n, (int)FLASH_PAGE);
    const uint8_t cmd[4] = {0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    if (!spi_cmd_read(cmd, 4, out, cnt)) return false;
    addr += cnt; out += cnt; n -= cnt;
  }
  stat_read_us += micros() - t0;
  return true;
}

static bool flash_jedec(uint8_t *id) {
  const uint8_t cmd[1] = {0x9f};
  return spi_cmd_read(cmd, 1, id, 3);
}

static bool flash_erase_sector(uint32_t addr) {
  addr &= ~(FLASH_SECTOR - 1);
  flash_write_enable();
  spi_cs(false);
  sws_write1(0x0c, 0x20);
  sws_write1(0x0c, (uint8_t)(addr >> 16));
  sws_write1(0x0c, (uint8_t)(addr >> 8));
  sws_write1(0x0c, (uint8_t)addr);
  spi_cs(true);
  delay(80);
  return flash_ready(200, NULL);
}

static bool flash_erase_all() {
  flash_write_enable();
  spi_cs(false);
  const uint8_t cmd = 0x60;
  spi_fifo_write(&cmd, 1);
  spi_cs(true);
  return flash_ready(15000, NULL);
}

// one page program: the data must not cross a 256-byte page boundary
static bool flash_write_page(uint32_t addr, const uint8_t *data, int n) {
  static uint8_t c[4 + FLASH_PAGE];
  if (n < 1 || (addr & (FLASH_PAGE - 1)) + n > FLASH_PAGE) return false;
  c[0] = 0x02; c[1] = addr >> 16; c[2] = addr >> 8; c[3] = addr;
  memcpy(c + 4, data, n);
  flash_write_enable();
  spi_cs(false);
  spi_fifo_write(c, 4 + n);
  spi_cs(true);
  delay(PAGE_PROGRAM_MIN_MS);
  return flash_ready(200, NULL);
}

// ---------------------------------------------------------------- host protocol

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c |= 0x20;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int parse_hex(const char *s, uint8_t *out, int max) {
  int n = 0;
  while (s[0] && s[1]) {
    int a = hexval(s[0]), b = hexval(s[1]);
    if (a < 0 || b < 0 || n >= max) return -1;
    out[n++] = (uint8_t)(a << 4 | b);
    s += 2;
  }
  return s[0] ? -1 : n;
}

static void reply_hex(const uint8_t *d, int n) {
  static const char hx[] = "0123456789abcdef";
  static char out[2 * MAX_READ + 8];
  int k = 0;
  out[k++] = 'O'; out[k++] = 'K'; out[k++] = ' ';
  for (int i = 0; i < n; i++) { out[k++] = hx[d[i] >> 4]; out[k++] = hx[d[i] & 15]; }
  out[k++] = '\n';
  Serial.write((const uint8_t *)out, k);
}

// The link to the computer is the slow part of a read (about 45 kbytes/s on the C3's USB port), so the
// data goes as base64 instead of hex, and a block that is all 0xff (erased flash) as a short marker.
static void reply_b64(const uint8_t *d, int n) {
  static const char tb[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  static char out[(MAX_READ + 2) / 3 * 4 + 8];
  int k = 0;
  out[k++] = 'O'; out[k++] = 'K'; out[k++] = ' ';
  bool blank = true;
  for (int i = 0; i < n && blank; i++) blank = d[i] == 0xff;
  if (blank) {
    out[k++] = '*';
  } else {
    for (int i = 0; i < n; i += 3) {
      uint32_t v = (uint32_t)d[i] << 16;
      if (i + 1 < n) v |= (uint32_t)d[i + 1] << 8;
      if (i + 2 < n) v |= d[i + 2];
      out[k++] = tb[(v >> 18) & 63];
      out[k++] = tb[(v >> 12) & 63];
      out[k++] = i + 1 < n ? tb[(v >> 6) & 63] : '=';
      out[k++] = i + 2 < n ? tb[v & 63] : '=';
    }
  }
  out[k++] = '\n';
  uint32_t t0 = micros();
  Serial.write((const uint8_t *)out, k);
  Serial.flush();
  stat_reply_us += micros() - t0;
}

// CRC-32 as in zlib; start with 0
static uint32_t crc32_update(uint32_t crc, const uint8_t *d, int n) {
  crc = ~crc;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

static void ok() { Serial.print("OK\n"); }
static void err(const char *why) { Serial.print("ERR "); Serial.print(why); Serial.print("\n"); }

// Line test without a target: send an END byte and look at what comes back on the RX pin.
static void selftest() {
  if (mode == MODE_UART) {
    u_flush_in();
    if (!u_stop()) return err("no echo: check TX - resistor - RX");
    Serial.printf("OK uart echo at %u baud\n", (unsigned)sws_baud);
    return;
  }
  rmt_receive_config_t rc = {};
  r_rx_config(&rc);
  xQueueReset(r_rxq);
  if (rmt_receive(r_rx, r_rxsym, sizeof(r_rxsym), &rc) != ESP_OK) return err("rmt receive");
  size_t got = 0;
  int k = r_enc_end(r_sym);
  if (!r_send(r_sym, k) || xQueueReceive(r_rxq, &got, pdMS_TO_TICKS(60)) != pdTRUE)
    return err("no pulses seen: check TX - resistor - RX");
  if ((int)got != k) { Serial.printf("ERR saw %d pulses, sent %d\n", (int)got, k); return; }
  for (int i = 0; i < k; i++) {
    uint32_t want = r_sym[i].duration0, have = r_rxsym[i].duration0;
    if (have * 4 < want * 3 || have * 4 > want * 5 + 8) {
      Serial.printf("ERR pulse %d is %u ticks, sent %u\n", i, (unsigned)have, (unsigned)want);
      return;
    }
  }
  Serial.printf("OK rmt loopback, unit %u ticks (%.0f ns), long pulse measured %u ticks\n",
                (unsigned)r_unit, r_unit * 12.5, (unsigned)r_rxsym[0].duration0);
}

// ================================================================ UART bridge
// The computer talks straight to the target's UART (bootloaders of BL702, Beken, ESP chips; serial consoles).
// RX is the SWS wire, TX the PIN_BRIDGE_TX wire. Leave it by sending ESCAPE after a pause of a second.

static const uart_port_t BRIDGE_UART = UART_NUM_1;
static const char ESCAPE[] = "~~~X";
static bool bridge_on = false;
static uint32_t bridge_last_ms = 0;
static int esc_pos = -1;
static SwsMode mode_before_bridge = MODE_RMT;

static void bridge_end() {
  if (!bridge_on) return;
  bridge_on = false;
  if (uart_is_driver_installed(BRIDGE_UART)) uart_driver_delete(BRIDGE_UART);
  pinMode(PIN_BRIDGE_TX, INPUT);
  pinMode(PIN_BOOT, INPUT);
}

// boot: 1 = BOOT pin high, 0 = low, -1 = left open; the target is restarted with that level
static bool bridge_begin(uint32_t baud, int boot) {
#if ARDUINO_USB_CDC_ON_BOOT
  if (swd_on) {
    swd_end();
    swd_on = false;
  }
  spif_end();
  uart_teardown();
  rmt_teardown();
  pinMode(PIN_SWS_TX, INPUT);                // the resistor side of the SWS wire stays quiet
  if (boot >= 0) {
    pinMode(PIN_BOOT, OUTPUT);
    digitalWrite(PIN_BOOT, boot ? HIGH : LOW);
  } else {
    pinMode(PIN_BOOT, INPUT);
  }
  uart_config_t cfg = {};
  cfg.baud_rate = (int)baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(BRIDGE_UART, 4096, 4096, 0, NULL, 0) != ESP_OK) return false;
  uart_param_config(BRIDGE_UART, &cfg);
  uart_set_pin(BRIDGE_UART, PIN_BRIDGE_TX, PIN_SWS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  gpio_pullup_en((gpio_num_t)PIN_SWS_RX);
  mode_before_bridge = mode;
  bridge_on = true;
  bridge_last_ms = millis();
  esc_pos = -1;
  target_reset_pulse();                      // the target starts with the BOOT level applied
  uart_flush_input(BRIDGE_UART);
  return true;
#else
  (void)baud;
  (void)boot;
  return false;
#endif
}

static void bridge_pump() {
  static uint8_t buf[512];
  int n = Serial.available();
  if (n > 0) {
    n = Serial.readBytes(buf, min(n, (int)sizeof(buf)));
    uint32_t now = millis();
    for (int i = 0; i < n; i++) {
      if (esc_pos < 0 && i == 0 && now - bridge_last_ms >= 1000) esc_pos = 0;
      if (esc_pos >= 0) esc_pos = buf[i] == (uint8_t)ESCAPE[esc_pos] ? esc_pos + 1 : -1;
      if (esc_pos == (int)sizeof(ESCAPE) - 1) {
        bridge_end();
        set_mode(mode_before_bridge);
        Serial.print("OK bridge closed\n");
        return;
      }
    }
    bridge_last_ms = now;
    uart_write_bytes(BRIDGE_UART, buf, n);
  }
  size_t avail = 0;
  uart_get_buffered_data_len(BRIDGE_UART, &avail);
  if (avail) {
    int got = uart_read_bytes(BRIDGE_UART, buf, min(avail, sizeof(buf)), 0);
    if (got > 0) Serial.write(buf, got);
  }
}

// ================================================================ binary streams (XR / XW)
// The text commands answer one block per request; these move a whole range in one command, in binary
// frames of up to 4 k with a CRC-32 each, so the USB link is never idle waiting for the next request.
//   XR <src> <addr> <len>: "OK <len>\n", then frames 'D' <n:u16> <data> <crc32>, an error is
//                          'E' <n:u16> <text>, the end 'Z'.
//   XW <dst> <addr> <len> <unit> <skip>: "OK\n", then the host sends frames <n:u16> <data> <crc32>; every frame
//                          is answered with 'K' after it is written, or 'E' <n:u16> <text>. The host sends
//                          ahead: the next frames wait in the receive buffer while one is programmed.
// src / dst: F = Telink flash over SWS, S = SWD flash (write) / memory (read), M = SWD memory (write),
// P = SPI flash. unit: the write is split at these boundaries (flash page); skip 1: parts that are all 0xff
// are not written (erased flash).

static const int XBLK = 4096;
static uint8_t xbuf[XBLK + 8];

static bool x_send(const uint8_t *d, int n) {
  uint32_t t0 = millis();
  while (n > 0) {
    int w = Serial.write(d, n);
    d += w;
    n -= w;
    if (w > 0) t0 = millis();
    else if (millis() - t0 > 3000) return false;   // the host stopped reading
  }
  return true;
}

static void x_frame(char type, const uint8_t *d, int n) {
  uint8_t h[3] = {(uint8_t)type, (uint8_t)n, (uint8_t)(n >> 8)};
  x_send(h, 3);
  x_send(d, n);
  if (type == 'D') {
    uint32_t crc = crc32_update(0, d, n);
    x_send((const uint8_t *)&crc, 4);
  }
}

static void x_error(const char *why) { x_frame('E', (const uint8_t *)why, strlen(why)); }

// after a failed XW the frames the host sent ahead must not reach the command parser: drop everything
// until the line has been quiet for 300 ms, then report
static void x_write_error(const char *why) {
  uint32_t last = millis();
  while (millis() - last < 300) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      last = millis();
    }
    delay(1);
  }
  x_error(why);
}

static const char *x_read(char src, uint32_t addr, uint8_t *buf, int n) {
  switch (src) {
    case 'F':
      read_dbg[0] = 0;
      return flash_read(addr, buf, n) ? NULL : (read_dbg[0] ? read_dbg : "read failed");
    case 'S': {
      int r = swd_mem_read(0, addr, buf, n);
      return r == SWD_OK ? NULL : swdf_err_name(r);
    }
    case 'P':
      spif_read(addr, buf, n);
      return NULL;
  }
  return "source";
}

static const char *x_write_part(char dst, uint32_t addr, const uint8_t *d, int n) {
  switch (dst) {
    case 'F':
      return flash_write_page(addr, d, n) ? NULL : "write failed";
    case 'P': {
      int r = spif_program(addr, d, n);
      return r == SPIF_OK ? NULL : r == SPIF_ERR_WEL ? "write enable refused (WP pin or status protection)"
                                                     : "timeout";
    }
    case 'S': {
      int r = swdf_write(addr, d, n);
      return r == SWD_OK ? NULL : swdf_err_name(r);
    }
    case 'M': {
      int r = swd_mem_write(0, addr, d, n);
      return r == SWD_OK ? NULL : swdf_err_name(r);
    }
  }
  return "destination";
}

static const char *x_mode_check(char c) {
  if (c == 'F') return swd_on || spif_active() ? "not in SWire mode" : NULL;
  if (c == 'S' || c == 'M') return swd_on ? NULL : "not in SWD mode";
  if (c == 'P') return spif_active() ? NULL : "not in SPI mode";
  return "source / destination: F, S, M or P";
}

static void x_read_cmd(char src, uint32_t addr, uint32_t len) {
  const char *why = x_mode_check(src);
  if (why) return err(why);
  if (src == 'S' && ((addr | len) & 3)) return err("SWD: address and length in whole words");
  Serial.printf("OK %u\n", (unsigned)len);
  while (len > 0) {
    int n = (int)min((uint32_t)XBLK, len);
    why = x_read(src, addr, xbuf, n);
    if (why) return x_error(why);
    x_frame('D', xbuf, n);
    addr += n;
    len -= n;
  }
  x_frame('Z', NULL, 0);
}

static void x_write_cmd(char dst, uint32_t addr, uint32_t len, uint32_t unit, bool skip) {
  const char *why = x_mode_check(dst);
  if (why) return err(why);
  if (dst == 'F' || dst == 'P') unit = 256;
  if (unit < 4 || unit > XBLK || (unit & (unit - 1))) return err("unit: power of two, 4..4096");
  if ((dst == 'S' || dst == 'M') && ((addr | len) & 3)) return err("SWD: address and length in whole words");
  ok();
  while (len > 0) {
    uint8_t h[2];
    if (Serial.readBytes(h, 2) != 2) return x_write_error("frame header timeout");
    int n = h[0] | (h[1] << 8);
    if (n < 1 || n > XBLK || (uint32_t)n > len) return x_write_error("frame length");
    if (Serial.readBytes(xbuf, n + 4) != (size_t)(n + 4)) return x_write_error("frame data timeout");
    uint32_t crc;
    memcpy(&crc, xbuf + n, 4);
    if (crc != crc32_update(0, xbuf, n)) return x_write_error("frame CRC");
    for (int pos = 0; pos < n;) {
      uint32_t a = addr + pos;
      int k = (int)min((uint32_t)(n - pos), unit - a % unit);
      bool blank = skip;
      for (int i = 0; i < k && blank; i++) blank = xbuf[pos + i] == 0xff;
      if (!blank) {
        why = x_write_part(dst, a, xbuf + pos, k);
        if (why) return x_write_error(why);
      }
      pos += k;
    }
    addr += n;
    len -= n;
    uint8_t k = 'K';
    x_send(&k, 1);
  }
}

// ================================================================ SWD commands

static bool swd_check(int r) {
  if (r == SWD_OK) return true;
  Serial.printf("ERR swd %s\n", swdf_err_name(r));
  return false;
}

static void handle(char *ln) {
  char *argv[9] = {NULL};
  int argc = 0;
  for (char *tok = strtok(ln, " \t\r"); tok && argc < 9; tok = strtok(NULL, " \t\r")) argv[argc++] = tok;
  if (!argc) return;
  const char *cmd = argv[0];
  uint32_t a1 = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
  uint32_t a2 = argc > 2 ? strtoul(argv[2], NULL, 0) : 0;

  if (!strcmp(cmd, "V")) {
    Serial.printf("OK %s %s mode=%s baud=%u rate=%u burst=%d tx=%d rx=%d rst=%d pwr=%d+%d more\n", FW_NAME, FW_VERSION,
                  mode == MODE_UART ? "uart" : "rmt", (unsigned)sws_baud, (unsigned)sws_rate, read_burst,
                  PIN_SWS_TX, PIN_SWS_RX, PIN_RST, pwr_pins[0], (int)sizeof(pwr_pins) - 1);
  } else if (!strcmp(cmd, "MODE")) {         // MODE uart | rmt
    if (argc < 2) return err("uart or rmt");
    SwsMode m;
    if (!strcmp(argv[1], "uart")) m = MODE_UART;
    else if (!strcmp(argv[1], "rmt")) m = MODE_RMT;
    else return err("uart or rmt");
    if (!set_mode(m)) return err("driver setup failed");
    ok();
  } else if (!strcmp(cmd, "BAUD")) {         // BAUD <uart baud>, used in uart mode
    if (a1 < 340000 || a1 > 5000000) return err("baud out of range (340000..5000000)");
    sws_baud = a1;
    if (mode == MODE_UART && !uart_setup()) return err("driver setup failed");
    ok();
  } else if (!strcmp(cmd, "RATE")) {         // RATE <swire bit/s>, used in rmt mode; applied by A
    if (a1 < 30000 || a1 > 2000000) return err("rate out of range (30000..2000000)");
    sws_rate = a1;
    ok();
  } else if (!strcmp(cmd, "BURST")) {        // BURST <1..256>
    if (a1 < 1 || a1 > 256) return err("burst out of range");
    read_burst = (int)a1;
    ok();
  } else if (!strcmp(cmd, "SELFTEST")) {
    selftest();
  } else if (!strcmp(cmd, "PWR")) {          // PWR 1 | 0 | z: power pins high, low, or inputs (not driven)
    if (argc > 1 && argv[1][0] == 'z') target_power_release();
    else target_power(a1 != 0);
    ok();
  } else if (!strcmp(cmd, "RST")) {
    target_reset_pulse();
    ok();
  } else if (!strcmp(cmd, "A")) {            // A <activation ms>: reset, stall the CPU, set the speed
    if (argc > 1 && a1 > 0) activate(a1);
    const char *hint = "no answer on SWS (wiring, power, or the chip sleeps: use a longer activation)";
    if (mode == MODE_UART) {
      int div = uart_match_speed();
      if (!div) return err(hint);
      if (!sws_readback_check()) return err(check_dbg);
      Serial.printf("OK mode=uart div=%d (works %d..%d) clk_mhz=%.1f bitrate=%u\n", div, match_first, match_last,
                    sws_baud * div / 2000000.0, (unsigned)(sws_baud / 10));
    } else {
      double clk = 0;
      int div = rmt_match_speed(&clk);
      if (!div) return err(hint);
      if (!sws_readback_check()) return err(check_dbg);
      Serial.printf("OK mode=rmt div=%d clk_mhz=%.1f bitrate=%u\n", div, clk / 1e6,
                    (unsigned)(RMT_HZ / (5 * r_unit)));
    }
  } else if (!strcmp(cmd, "G")) {            // G <addr> <len>: read registers / RAM
    if (a2 < 1 || a2 > MAX_READ) return err("len");
    read_dbg[0] = 0;
    if (!sws_read(a1, data_buf, (int)a2)) return err(read_dbg[0] ? read_dbg : "read failed");
    reply_hex(data_buf, (int)a2);
  } else if (!strcmp(cmd, "S")) {            // S <addr> <hex>: write registers / RAM
    int n = argc > 2 ? parse_hex(argv[2], data_buf, FLASH_PAGE) : -1;
    if (n < 1) return err("data");
    if (!sws_write(a1, data_buf, n)) return err("no echo");
    ok();
  } else if (!strcmp(cmd, "FJ")) {
    uint8_t id[3];
    if (!flash_jedec(id)) return err("read failed");
    reply_hex(id, 3);
  } else if (!strcmp(cmd, "FR")) {           // FR <addr> <len>
    if (a2 < 1 || a2 > MAX_READ) return err("len");
    read_dbg[0] = 0;
    if (!flash_read(a1, data_buf, (int)a2)) return err(read_dbg[0] ? read_dbg : "read failed");
    reply_hex(data_buf, (int)a2);
  } else if (!strcmp(cmd, "FB")) {           // FB <addr> <len>: as FR, answer in base64, "*" = all 0xff
    if (a2 < 1 || a2 > MAX_READ) return err("len");
    read_dbg[0] = 0;
    if (!flash_read(a1, data_buf, (int)a2)) return err(read_dbg[0] ? read_dbg : "read failed");
    reply_b64(data_buf, (int)a2);
  } else if (!strcmp(cmd, "FC")) {           // FC <addr> <len>: CRC-32 of a flash range, for compare / verify
    if (a2 < 1 || a2 > 65536) return err("len");
    uint32_t crc = 0;
    read_dbg[0] = 0;
    for (uint32_t done = 0; done < a2;) {
      int cnt = (int)min((uint32_t)MAX_READ, a2 - done);
      if (!flash_read(a1 + done, data_buf, cnt)) return err(read_dbg[0] ? read_dbg : "read failed");
      crc = crc32_update(crc, data_buf, cnt);
      done += cnt;
    }
    Serial.printf("OK %08x\n", (unsigned)crc);
  } else if (!strcmp(cmd, "STAT")) {         // timing since the last STAT, then reset
    Serial.printf("OK read_ms=%u read_bytes=%u reply_ms=%u\n", (unsigned)(stat_read_us / 1000),
                  (unsigned)stat_read_bytes, (unsigned)(stat_reply_us / 1000));
    stat_read_us = stat_reply_us = 0;
    stat_read_bytes = 0;
  } else if (!strcmp(cmd, "FS")) {           // flash status register
    uint8_t st;
    if (!flash_status(&st)) return err("read failed");
    reply_hex(&st, 1);
  } else if (!strcmp(cmd, "FU")) {           // check the status read, clear the protection bits
    uint8_t st;
    int r = flash_unlock(&st);
    if (r == 1) return err("flash status cannot be read reliably, refusing to erase or write");
    if (r == 2) { Serial.printf("ERR flash still protected, status=%02x\n", st); return; }
    ok();
  } else if (!strcmp(cmd, "FE")) {           // FE <addr>: erase the 4 k sector
    if (!flash_erase_sector(a1)) return err("flash not ready");
    ok();
  } else if (!strcmp(cmd, "FA")) {
    if (!flash_erase_all()) return err("flash not ready");
    ok();
  } else if (!strcmp(cmd, "FW")) {           // FW <addr> <hex>: program up to one page
    int n = argc > 2 ? parse_hex(argv[2], data_buf, FLASH_PAGE) : -1;
    if (n < 1) return err("data");
    if (!flash_write_page(a1, data_buf, n)) return err("write failed");
    ok();
  } else if (!strcmp(cmd, "XR")) {           // XR <src> <addr> <len>: binary stream read (see above)
    if (argc < 4) return err("src addr len");
    x_read_cmd(argv[1][0], strtoul(argv[2], NULL, 0), strtoul(argv[3], NULL, 0));
  } else if (!strcmp(cmd, "XW")) {           // XW <dst> <addr> <len> <unit> <skip>: binary stream write
    if (argc < 6) return err("dst addr len unit skip");
    x_write_cmd(argv[1][0], strtoul(argv[2], NULL, 0), strtoul(argv[3], NULL, 0), strtoul(argv[4], NULL, 0),
                atoi(argv[5]) != 0);
  } else if (!strcmp(cmd, "SPI")) {          // SPI [kHz]: the wires to an SPI NOR flash (no power switched)
    uint32_t khz = argc > 1 ? a1 : SPI_KHZ_DEFAULT;
    if (khz < 100 || khz > 20000) return err("100..20000 kHz");
    if (!spif_active()) {
      bridge_end();
      if (swd_on) {
        swd_end();
        swd_on = false;
      }
      uart_teardown();
      rmt_teardown();
    }
    if (!spif_begin(khz)) return err("SPI needs a c3-usb board (GPIO 21 is the host link here)");
    Serial.printf("OK %u\n", (unsigned)spif_khz());
  } else if (cmd[0] == 'P' && cmd[1] && !cmd[2] && strchr("TJABCEXWS", cmd[1]) && !spif_active()) {
    err("not in SPI mode (send SPI first)");
  } else if (!strcmp(cmd, "PT")) {           // PT <hex out> [n in]: one raw transaction, OK <hex in>
    int n = parse_hex(argv[1] ? argv[1] : "", data_buf, 300);
    int nin = argc > 2 ? (int)a2 : 0;
    if (n < 1 || nin < 0 || nin > 256) return err("hex out (1..300 bytes), n in 0..256");
    static uint8_t in[256];
    spif_xfer(data_buf, n, in, nin);
    if (!nin) return ok();
    reply_hex(in, nin);
  } else if (!strcmp(cmd, "PJ")) {           // PJ: wake from deep power-down, JEDEC id (3 bytes hex)
    uint8_t c = 0xAB, id[3];
    spif_xfer(&c, 1, NULL, 0);
    delay(1);
    c = 0x9F;
    spif_xfer(&c, 1, id, 3);
    reply_hex(id, 3);
  } else if (!strcmp(cmd, "PA")) {           // PA 3|4: address width
    if (a1 != 3 && a1 != 4) return err("3 or 4");
    spif_addr4(a1 == 4);
    ok();
  } else if (!strcmp(cmd, "PB")) {           // PB <addr> <len>: read, base64 (* = all 0xff)
    if (a2 < 1 || a2 > MAX_READ) return err("len");
    spif_read(a1, data_buf, a2);
    reply_b64(data_buf, a2);
  } else if (!strcmp(cmd, "PC")) {           // PC <addr> <len>: CRC-32 of the range (len up to 16 Mbytes)
    if (a2 < 1 || a2 > 0x1000000) return err("len 1..16 M");
    uint32_t crc = 0;
    for (uint32_t done = 0; done < a2;) {
      int cnt = (int)min((uint32_t)MAX_READ, a2 - done);
      spif_read(a1 + done, data_buf, cnt);
      crc = crc32_update(crc, data_buf, cnt);
      done += cnt;
    }
    Serial.printf("OK %08x\n", (unsigned)crc);
  } else if (!strcmp(cmd, "PE")) {           // PE <addr> [opcode]: erase 0x20 = 4 k (default), 0x52 32 k, 0xD8 64 k
    uint8_t op = argc > 2 ? (uint8_t)a2 : 0x20;
    if (op != 0x20 && op != 0x52 && op != 0xD8) return err("opcode 0x20, 0x52 or 0xD8");
    int r = spif_erase(a1, op, op == 0x20 ? 1000 : 4000);
    if (r) return err(r == SPIF_ERR_WEL ? "write enable refused (WP pin or status protection)" : "timeout");
    ok();
  } else if (!strcmp(cmd, "PX")) {           // PX: chip erase (up to 400 s)
    int r = spif_erase_chip(400000);
    if (r) return err(r == SPIF_ERR_WEL ? "write enable refused (WP pin or status protection)" : "timeout");
    ok();
  } else if (!strcmp(cmd, "PW")) {           // PW <addr> <hex>: program, within one 256-byte page
    int n = argc > 2 ? parse_hex(argv[2], data_buf, 256) : -1;
    if (n < 1 || (a1 & 0xff) + n > 256) return err("data: 1..256 bytes inside one page");
    int r = spif_program(a1, data_buf, n);
    if (r) return err(r == SPIF_ERR_WEL ? "write enable refused (WP pin or status protection)" : "timeout");
    ok();
  } else if (!strcmp(cmd, "PS")) {           // PS: status register 1
    Serial.printf("OK %02x\n", spif_status());
  } else if (!strcmp(cmd, "SWD")) {          // SWD: switch the pins to SWD, target power on
    bridge_end();
    spif_end();
    uart_teardown();
    rmt_teardown();
    swd_begin();
    swd_on = true;
    target_power(true);
    digitalWrite(PIN_RST, HIGH);
    ok();
  } else if (!strcmp(cmd, "SWCLK")) {        // SWCLK <kHz>
    if (a1 < 10 || a1 > 4000) return err("10..4000 kHz");
    swd_set_khz(a1);
    Serial.printf("OK %u\n", (unsigned)swd_khz());
  } else if (!swd_on && ((cmd[0] == 'S' && cmd[1] && strchr("CDAMFHR", cmd[1]) && strcmp(cmd, "SELFTEST") &&
                           strcmp(cmd, "STAT") && strcmp(cmd, "SRST")) || !strcmp(cmd, "TC"))) {
    err("not in SWD mode (send SWD first)");
  } else if (!strcmp(cmd, "SC")) {           // SC [reset | md <targetsel>]: connect, IDCODE, debug power-up
    if (argc > 1 && !strcmp(argv[1], "reset")) digitalWrite(PIN_RST, LOW);
    uint32_t id = 0;
    int r = argc > 2 && !strcmp(argv[1], "md") ? swd_connect_multidrop(strtoul(argv[2], NULL, 0), &id)
                                                : swd_connect(&id);
    if (r == SWD_OK) r = swd_power_up();
    if (!swd_check(r)) return;
    Serial.printf("OK %08x\n", (unsigned)id);
  } else if (!strcmp(cmd, "TC")) {           // TC: cJTAG (OScan1) through a TI ICEPick, then JTAG-DP power-up
    uint32_t icepick = 0, dap = 0;
    int r = ti_cjtag_connect(&icepick, &dap);
    if (r == SWD_OK) r = swd_power_up();
    if (r != SWD_OK) {
      Serial.printf("ERR cjtag %s (ICEPick id %08x)\n", swd_ack_name(r), (unsigned)icepick);
      return;
    }
    Serial.printf("OK %08x %08x\n", (unsigned)icepick, (unsigned)dap);
  } else if (!strcmp(cmd, "SRST")) {         // SRST 0|1: release or hold the reset wire
    digitalWrite(PIN_RST, a1 ? HIGH : LOW);
    ok();
  } else if (!strcmp(cmd, "SDR")) {          // SDR <addr>: DP register read
    uint32_t v = 0;
    if (!swd_check(swd_dp_read(a1, &v))) return;
    Serial.printf("OK %08x\n", (unsigned)v);
  } else if (!strcmp(cmd, "SDW")) {          // SDW <addr> <value>
    if (!swd_check(swd_dp_write(a1, a2))) return;
    ok();
  } else if (!strcmp(cmd, "SAR")) {          // SAR <ap> <addr>: AP register read
    uint32_t v = 0;
    if (!swd_check(swd_ap_read(a1, a2, &v))) return;
    Serial.printf("OK %08x\n", (unsigned)v);
  } else if (!strcmp(cmd, "SAW")) {          // SAW <ap> <addr> <value>
    uint32_t v = argc > 3 ? strtoul(argv[3], NULL, 0) : 0;
    if (!swd_check(swd_ap_write(a1, a2, v))) return;
    ok();
  } else if (!strcmp(cmd, "SM")) {           // SM <ap> <addr> <len>: memory read, base64 like FB
    uint32_t len = argc > 3 ? strtoul(argv[3], NULL, 0) : 0;
    if ((a2 | len) & 3 || len < 4 || len > MAX_READ) return err("addr and len: multiples of 4, len <= 1024");
    if (!swd_check(swd_mem_read(a1, a2, data_buf, len))) return;
    reply_b64(data_buf, len);
  } else if (!strcmp(cmd, "SMW")) {          // SMW <ap> <addr> <hex>: memory write, whole words
    int n = argc > 3 ? parse_hex(argv[3], data_buf, MAX_READ) : -1;
    if (n < 4 || (n & 3) || (a2 & 3)) return err("data: whole words at a word address");
    if (!swd_check(swd_mem_write(a1, a2, data_buf, n))) return;
    ok();
  } else if (!strcmp(cmd, "SFAM")) {         // SFAM <family> [p1] [p2] (see swdflash.h)
    static const struct { const char *name; SwdFlashFamily fam; } fams[] = {
        {"efr32s1", SWDF_EFR32S1}, {"efr32s2", SWDF_EFR32S2}, {"nrf52", SWDF_NRF52},   {"nrf53", SWDF_NRF53},
        {"stm32f1", SWDF_STM32F1}, {"stm32f4", SWDF_STM32F4}, {"stm32v2", SWDF_STM32V2}, {"stm32u5", SWDF_STM32U5},
        {"stm32l0", SWDF_STM32L0}, {"samd21", SWDF_SAMD21},   {"samd51", SWDF_SAMD51}};
    if (argc < 2) return err("family");
    bool found = false;
    for (auto &f : fams) {
      if (!strcmp(argv[1], f.name)) {
        swdf_set_family(f.fam, a2, argc > 3 ? strtoul(argv[3], NULL, 0) : 0);
        found = true;
      }
    }
    if (!found) return err("unknown family");
    ok();
  } else if (!strcmp(cmd, "SHALT")) {        // SHALT [reset]: stop the core
    if (!swd_check(swdf_halt(argc > 1 && !strcmp(argv[1], "reset")))) return;
    ok();
  } else if (!strcmp(cmd, "SCALL")) {        // SCALL <pc> <sp> <lr> [r0..r3]: run a function, OK <r0>
    if (argc < 4) return err("pc sp lr [args]");
    uint32_t args[4] = {0, 0, 0, 0};
    int nargs = 0;
    for (int i = 4; i < argc && nargs < 4; i++) args[nargs++] = strtoul(argv[i], NULL, 0);
    uint32_t r0 = 0;
    if (!swd_check(swdf_call(a1, a2, strtoul(argv[3], NULL, 0), args, nargs, 20000, &r0))) return;
    Serial.printf("OK %08x\n", (unsigned)r0);
  } else if (!strcmp(cmd, "SRUN")) {         // SRUN: debug off, reset, run
    if (!swd_check(swdf_run())) return;
    ok();
  } else if (!strcmp(cmd, "SFE")) {          // SFE <addr>: erase the flash page at addr
    if (!swd_check(swdf_erase_page(a1))) return;
    ok();
  } else if (!strcmp(cmd, "SFX")) {          // SFX: mass erase (EFR32 Series 1, nRF52)
    if (!swd_check(swdf_mass_erase())) return;
    ok();
  } else if (!strcmp(cmd, "SFW")) {          // SFW <addr> <hex>: program whole words within one page
    int n = argc > 2 ? parse_hex(argv[2], data_buf, MAX_READ) : -1;
    if (n < 4 || (n & 3) || (a1 & 3)) return err("data: whole words at a word address");
    if (!swd_check(swdf_write(a1, data_buf, n))) return;
    ok();
  } else if (!strcmp(cmd, "SFC")) {          // SFC <addr> <len>: CRC-32 of target memory (verify without upload)
    if ((a1 | a2) & 3 || a2 < 4 || a2 > 65536) return err("addr and len: multiples of 4, len <= 65536");
    uint32_t crc = 0;
    for (uint32_t done = 0; done < a2;) {
      int cnt = (int)min((uint32_t)MAX_READ, a2 - done);
      if (!swd_check(swd_mem_read(0, a1 + done, data_buf, cnt))) return;
      crc = crc32_update(crc, data_buf, cnt);
      done += cnt;
    }
    Serial.printf("OK %08x\n", (unsigned)crc);
  } else if (!strcmp(cmd, "BRIDGE")) {       // BRIDGE <baud> [hi|lo|z]: transparent UART bridge
    int boot = -1;
    if (argc > 2) boot = !strcmp(argv[2], "hi") ? 1 : !strcmp(argv[2], "lo") ? 0 : -1;
    if (a1 < 1200 || a1 > 3000000) return err("baud 1200..3000000");
    Serial.printf("OK bridge %u baud, boot %s; leave with a 1 s pause and %s\n", (unsigned)a1,
                  boot < 0 ? "open" : boot ? "high" : "low", ESCAPE);
    Serial.flush();
    if (!bridge_begin(a1, boot)) return err("bridge needs a c3-usb board");
  } else if (!strcmp(cmd, "RUN")) {          // restart the target's CPU
    sws_write1(0x006f, 0x22);
    ok();
  } else {
    err("unknown command");
  }
}

void setup() {
  // a page write is one line of about 530 characters; the default 256-byte receive buffer drops part of it
  // the receive buffer holds the frames of XW the host sends ahead, the transmit buffer those of XR
  Serial.setRxBufferSize(16384);
  Serial.setTxBufferSize(16384);
  Serial.begin(HOST_BAUD);
  gpio_set_level((gpio_num_t)PIN_RST, 1);    // preset the level, so enabling the output does not pulse reset
  pinMode(PIN_RST, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(PIN_RST, HIGH);
  for (uint8_t pin : pwr_pins) pwr_mask |= 1u << pin;
  for (uint8_t pin : pwr_pins) {
    pinMode(pin, INPUT);                     // GPIO function, not driven until the target is powered
    gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  }
  REG_WRITE(GPIO_OUT_W1TC_REG, pwr_mask);    // the level they will have when they become outputs
  pinMode(PIN_SWCLK, INPUT);
  pinMode(PIN_BRIDGE_TX, INPUT);
  set_mode(SWS_MODE_DEFAULT ? MODE_RMT : MODE_UART);
#if AUTO_ACTIVATE_MS > 0
  activate(AUTO_ACTIVATE_MS);                // leaves the target's CPU stopped, if it was caught
#endif
}

void loop() {
  static int len = 0;
  if (bridge_on) {
    bridge_pump();
    return;
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line[len] = 0;
      handle(line);
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  delay(1);
}
