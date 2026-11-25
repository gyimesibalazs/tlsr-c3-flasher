// Beken BK72xx download protocol: the real src/bk72xx.cpp against a simulated UART wire and target.
//
//   g++ -std=c++17 -Wall -Wextra -I tests/bk_sim/stub -I src -o /tmp/bk_sim tests/bk_sim/bk_sim.cpp src/bk72xx.cpp
//   /tmp/bk_sim              (its own checks)
//   /tmp/bk_sim --serve      (text protocol on stdin / stdout, used by test_bk_host.py)
//
// The wire: every byte takes 10 bit times at the sender's speed and arrives at its end; a byte read at another
// speed than it was sent arrives garbled or not at all. The target: after power-on its UART is ready after
// `ready` µs and a link check must be complete within `window` µs, otherwise its application starts and prints a
// log line now and then (and ignores the port). In download mode it parses the frames strictly (a malformed frame
// or an unknown command is a violation) and answers as the protocol describes: the BK7231N BootROM ("rom", CRC end
// inclusive, registers, flash id, status registers) or the BK7231T bootloader ("bl": CRC end exclusive, no
// registers / id / status, reads and writes below 0x11000 refused unless given at the address plus the flash size,
// flash write protected after a CRC until the next link check). The flash has status register protection: while
// any BP / CMP bit is set, erase and write are silently ignored, as a real flash does. Above `maxbaud` the target's
// UART does not work; `noise` corrupts one of that many bytes it sends above 115200.

#include <Arduino.h>
#include "driver/uart.h"
#include "config.h"
#include "bk72xx.h"
#include <map>
#include <string>
#include <vector>

// ================================================================ time and the host side of the wire

static double now = 0;                          // µs
uint32_t millis() { return (uint32_t)(now / 1000); }
uint32_t micros() { return (uint32_t)now; }
void delay(uint32_t ms) { now += ms * 1000.0; }

static bool tx_attached = true, installed = false;
static int floating_writes = 0;                 // bytes the firmware sent while its TX pin was an input
void pinMode(int pin, int mode) {
  if (pin == PIN_BRIDGE_TX && mode == INPUT) tx_attached = false;
}

static uint32_t host_baud = 115200;
static double host_tx_free = 0;
static std::vector<std::pair<double, uint32_t>> host_bauds;   // speed changes of the flasher's UART
static std::vector<uint8_t> host_rx;            // the driver's receive buffer

static uint32_t host_baud_at(double t) {
  uint32_t b = host_bauds.empty() ? 115200 : host_bauds[0].second;
  for (auto &c : host_bauds)
    if (c.first <= t) b = c.second;
  return b;
}

// ================================================================ the target

struct Out {
  double t;
  uint8_t b;
  uint32_t baud;
};

enum State { OFF, BOOTWIN, DOWNLOAD, APP };

struct Chip {
  // configuration (SIM new)
  bool present = true, rom = true, reset_wired = true;
  double ready_us = 2000, window_us = 30000;
  uint32_t max_baud = 3000000, noise = 0, size = 0x200000, chip_id = 0x7231C;
  uint8_t id[3] = {0xEB, 0x60, 0x15};
  int sr_bytes = 2, mid_len = 6;
  uint8_t sr1 = 0, sr2 = 0;
  // state
  State state = APP;
  double on_at = 0, busy = 0, tx_free = 0, switch_at = 0;
  uint32_t baud = 115200, next_baud = 0;
  bool switch_pending = false, crc_lock = false;
  std::vector<uint8_t> flash, in;
  std::vector<Out> out;
  uint32_t noise_ctr = 0;
  // statistics
  int violations = 0, link_checks = 0, erases4 = 0, erases64 = 0, writes = 0, reads = 0, crcs = 0, sr_writes = 0;
  int ignored_writes = 0;
  std::string last_violation;
};

static Chip chip;

static void violation(const char *what) {
  chip.violations++;
  chip.last_violation = what;
}

static uint32_t chip_baud_at(double t) {
  if (chip.switch_pending && t >= chip.switch_at) {
    chip.baud = chip.next_baud;
    chip.switch_pending = false;
  }
  return chip.baud;
}

static void put_out(double t, uint8_t b, uint32_t baud) {
  Out o = {t, b, baud};
  auto it = chip.out.end();
  while (it != chip.out.begin() && (it - 1)->t > t) --it;
  chip.out.insert(it, o);
}

static void emit(const std::vector<uint8_t> &bytes, double t) {
  for (uint8_t b : bytes) {
    double start = std::max(t, chip.tx_free);
    uint32_t baud = chip_baud_at(start);
    chip.tx_free = start + 10e6 / baud;
    if (baud > 115200 && chip.noise && ++chip.noise_ctr % chip.noise == 0) b ^= 0x10;
    if (baud > chip.max_baud) b = (uint8_t)(b * 7 + 3);   // its UART does not work at this speed
    put_out(chip.tx_free, b, baud);
  }
}

static void le32(std::vector<uint8_t> &v, uint32_t x) {
  for (int i = 0; i < 4; i++) v.push_back((uint8_t)(x >> (8 * i)));
}
static uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static void emit_short(uint8_t code, const std::vector<uint8_t> &p, double t) {
  std::vector<uint8_t> f = {0x04, 0x0E, (uint8_t)(4 + p.size()), 0x01, 0xE0, 0xFC, code};
  f.insert(f.end(), p.begin(), p.end());
  emit(f, t);
}

static void emit_long(uint8_t code, const std::vector<uint8_t> &p, double t, int declared = -1) {
  uint32_t len = declared >= 0 ? (uint32_t)declared : (uint32_t)(1 + p.size());
  std::vector<uint8_t> f = {0x04, 0x0E, 0xFF, 0x01, 0xE0, 0xFC, 0xF4, (uint8_t)len, (uint8_t)(len >> 8), code};
  f.insert(f.end(), p.begin(), p.end());
  emit(f, t);
}

static void enter_app(double t) {
  if (!chip.present) return;
  chip.state = APP;
  chip.in.clear();
  chip.switch_pending = false;
  chip.baud = 115200;
  // the application's log on TX1, at 115200
  static const char *line = "\r\n[I] BK7231N app 1.2.3 start\r\n";
  for (int i = 0; i < 40; i++) {
    std::vector<uint8_t> v(line, line + strlen(line));
    emit(v, t + 5000 + i * 250000.0);
  }
}

static void power_off() {
  chip.state = OFF;
  chip.in.clear();
  double t = now;
  chip.out.erase(std::remove_if(chip.out.begin(), chip.out.end(), [t](const Out &o) { return o.t > t; }),
                 chip.out.end());
  chip.tx_free = now;
}

static void power_on() {
  if (!chip.present) return;
  chip.state = BOOTWIN;
  chip.on_at = now;
  chip.baud = 115200;
  chip.switch_pending = false;
  chip.crc_lock = false;
  chip.busy = now;
}

// the flasher's reset (target_reset_pulse): RST low and the power pins off for 300 ms, then on
static void sim_reset() {
  if (chip.reset_wired) power_off();
  delay(300);
  if (chip.reset_wired) power_on();
}

static bool is_protected() {   // BP0..BP4 and CMP; on Macronix BP0..BP3 (its bit 6 is the quad enable)
  uint16_t sr = chip.sr1 | chip.sr2 << 8;
  uint16_t mask = chip.id[0] == 0xC2 ? 0x3C : chip.sr_bytes == 2 ? 0x407C : 0x7C;
  return chip.rom && (sr & mask);
}

static uint8_t *at(uint32_t a) { return &chip.flash[a % chip.size]; }

static uint32_t crc_raw(uint32_t a, uint32_t n) {   // CRC-32 from 0xFFFFFFFF, not inverted at the end
  uint32_t c = 0xFFFFFFFF;
  for (uint32_t i = 0; i < n; i++) {
    c ^= *at(a + i);
    for (int b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return c;
}

// the BK7231T bootloader refuses its own area (reachable at the address plus the flash size)
static bool bl_refused(uint32_t a) { return !chip.rom && a < BK_BOOT_END; }

static void handle_short(uint8_t code, const std::vector<uint8_t> &p, double t) {
  double tp = std::max(t, chip.busy);
  if (code == 0x00) {                           // link check
    if (!p.empty()) violation("link check with payload");
    if (chip.state == BOOTWIN) chip.state = DOWNLOAD;
    chip.link_checks++;
    chip.crc_lock = false;
    emit_short(0x01, {0x00}, tp + 50);
    return;
  }
  if (chip.state != DOWNLOAD) return;
  if (code == 0x03) {                           // read register
    if (!chip.rom) return;
    if (p.size() != 4) return violation("read register length");
    uint32_t a = rd32(p.data());
    std::vector<uint8_t> r;
    le32(r, a);
    le32(r, a == 0x800000 ? chip.chip_id : 0);
    emit_short(0x03, r, tp + 50);
  } else if (code == 0x0F) {                    // set baud rate
    if (p.size() != 5) return violation("set baud length");
    chip.next_baud = rd32(p.data());
    chip.switch_at = tp + p[4] * 1000.0;
    chip.switch_pending = true;
    emit_short(0x0F, p, chip.switch_at + 10);
  } else if (code == 0x10) {                    // CRC
    if (p.size() != 8) return violation("CRC length");
    uint32_t s = rd32(p.data()), e = rd32(p.data() + 4);
    if (e < s) return violation("CRC end before start");
    uint32_t n = chip.rom ? e - s + 1 : e - s;
    chip.busy = tp + n * 2.5;
    chip.crcs++;
    if (!chip.rom) chip.crc_lock = true;
    std::vector<uint8_t> r;
    le32(r, crc_raw(s, n));
    emit_short(0x10, r, chip.busy);
  } else if (code == 0x0E) {                    // reboot
    if (p.size() != 1 || p[0] != 0xA5) return violation("reboot payload");
    enter_app(tp + 1000);
  } else {
    violation("unknown short command");
  }
}

static void handle_long(uint8_t code, const std::vector<uint8_t> &p, double t) {
  if (chip.state != DOWNLOAD) return;
  double tp = std::max(t, chip.busy);
  if (code == 0x09) {                           // read 4 k
    if (p.size() != 4) return violation("read 4k length");
    uint32_t a = rd32(p.data());
    std::vector<uint8_t> r = {(uint8_t)(bl_refused(a) ? 6 : 0)};
    le32(r, a);
    for (uint32_t i = 0; i < BK_SECTOR; i++) r.push_back(bl_refused(a) ? 0 : *at(a + i));
    chip.reads++;
    emit_long(0x09, r, tp + 1000);
  } else if (code == 0x07) {                    // write 4 k
    if (p.size() != 4 + BK_SECTOR) return violation("write 4k length");
    uint32_t a = rd32(p.data());
    uint8_t st = a % BK_SECTOR ? 1 : bl_refused(a) ? 6 : 0;
    if (!st && (is_protected() || chip.crc_lock)) {
      chip.ignored_writes++;
    } else if (!st) {
      for (uint32_t i = 0; i < BK_SECTOR; i++) *at(a + i) &= p[4 + i];
      chip.writes++;
    }
    chip.busy = tp + 12000;
    std::vector<uint8_t> r = {st};
    le32(r, a);
    emit_long(0x07, r, chip.busy);
  } else if (code == 0x0F) {                    // erase
    if (p.size() != 5 || (p[0] != 0x20 && p[0] != 0xD8)) return violation("erase command");
    uint32_t a = rd32(p.data() + 1), unit = p[0] == 0x20 ? BK_SECTOR : BK_BLOCK;
    uint8_t st = a % unit ? 1 : bl_refused(a) ? 6 : 0;
    if (!st && (is_protected() || chip.crc_lock)) {
      chip.ignored_writes++;
    } else if (!st) {
      for (uint32_t i = 0; i < unit; i++) *at(a + i) = 0xFF;
      (unit == BK_SECTOR ? chip.erases4 : chip.erases64)++;
    }
    chip.busy = tp + (unit == BK_SECTOR ? 45000 : 350000);
    std::vector<uint8_t> r = {st, p[0]};
    le32(r, a);
    emit_long(0x0F, r, chip.busy);
  } else if (code == 0x0C) {                    // read flash status register
    if (!chip.rom) return;
    if (p.size() != 1) return violation("status read length");
    uint8_t v = p[0] == 0x05 ? chip.sr1 : p[0] == 0x35 ? (chip.sr_bytes == 2 ? chip.sr2 : 0) : 0xFF;
    emit_long(0x0C, {0, p[0], v}, tp + 100);
  } else if (code == 0x0D) {                    // write flash status register
    if (!chip.rom) return;
    if ((p.size() != 2 && p.size() != 3) || p[0] != 0x01) return violation("status write");
    chip.sr1 = p[1];
    if (p.size() == 3) chip.sr2 = p[2];
    chip.sr_writes++;
    std::vector<uint8_t> r = {0};
    r.insert(r.end(), p.begin(), p.end());
    emit_long(0x0D, r, tp + 15000);
  } else if (code == 0x0E) {                    // flash id
    if (!chip.rom) return;
    if (p.size() != 4 || rd32(p.data()) != 0x9F) return violation("flash id command");
    emit_long(0x0E, {0, 0x00, chip.id[0], chip.id[1], chip.id[2]}, tp + 100, chip.mid_len);
  } else {
    violation("unknown long command");
  }
}

static void parse(double t) {
  std::vector<uint8_t> &in = chip.in;
  for (;;) {
    if (in.empty()) return;
    if (in[0] != 0x01) {
      in.erase(in.begin());
      continue;
    }
    if (in.size() < 3) return;
    if (in[1] != 0xE0 || in[2] != 0xFC) {
      in.erase(in.begin());
      continue;
    }
    if (in.size() < 4) return;
    if (in[3] == 0xFF) {
      if (in.size() < 7) return;
      if (in[4] != 0xF4) {
        if (chip.state == DOWNLOAD) violation("long frame without F4");
        in.erase(in.begin());
        continue;
      }
      uint32_t len = in[5] | in[6] << 8;
      if (len < 1 || len > 1 + 4 + BK_SECTOR) {
        if (chip.state == DOWNLOAD) violation("long frame length");
        in.erase(in.begin());
        continue;
      }
      if (in.size() < 7 + len) return;
      uint8_t code = in[7];
      std::vector<uint8_t> p(in.begin() + 8, in.begin() + 7 + len);
      in.erase(in.begin(), in.begin() + 7 + len);
      handle_long(code, p, t);
    } else {
      uint32_t n = in[3];
      if (n < 1) {
        in.erase(in.begin());
        continue;
      }
      if (in.size() < 4 + n) return;
      uint8_t code = in[4];
      std::vector<uint8_t> p(in.begin() + 5, in.begin() + 4 + n);
      in.erase(in.begin(), in.begin() + 4 + n);
      handle_short(code, p, t);
    }
  }
}

static void chip_rx(uint8_t b, double t, uint32_t baud) {
  if (!chip.present || chip.state == OFF) return;
  if (chip.state == BOOTWIN && t > chip.on_at + chip.window_us) enter_app(chip.on_at + chip.window_us);
  if (chip.state == APP) return;               // the application ignores the port
  if (t < chip.on_at + chip.ready_us) return;  // its UART is not running yet
  uint32_t cb = chip_baud_at(t);
  if (cb != baud || cb > chip.max_baud) {
    if (b & 1) return;
    b ^= 0x55;
  }
  chip.in.push_back(b);
  parse(t);
}

// ================================================================ the UART driver, on the simulated wire

static void deliver() {
  size_t k = 0;
  while (k < chip.out.size() && chip.out[k].t <= now) {
    const Out &o = chip.out[k++];
    if (!installed) continue;
    if (host_baud_at(o.t) == o.baud) host_rx.push_back(o.b);
    else if (!(o.b & 1)) host_rx.push_back(o.b ^ 0x55);
  }
  chip.out.erase(chip.out.begin(), chip.out.begin() + k);
  if (chip.state == BOOTWIN && now > chip.on_at + chip.window_us) enter_app(chip.on_at + chip.window_us);
}

esp_err_t uart_driver_install(uart_port_t, int, int, int, void *, int) {
  installed = true;
  host_rx.clear();
  return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t) {
  installed = false;
  return ESP_OK;
}
bool uart_is_driver_installed(uart_port_t) { return installed; }
esp_err_t uart_param_config(uart_port_t, const uart_config_t *cfg) {
  host_baud = cfg->baud_rate;
  host_bauds.push_back({now, host_baud});
  return ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t, int tx, int, int, int) {
  if (tx == PIN_BRIDGE_TX) tx_attached = true;
  return ESP_OK;
}
esp_err_t uart_set_baudrate(uart_port_t, uint32_t baud) {
  host_baud = baud;
  host_bauds.push_back({now, baud});
  return ESP_OK;
}
esp_err_t uart_set_rx_timeout(uart_port_t, uint8_t) { return ESP_OK; }
esp_err_t uart_set_rx_full_threshold(uart_port_t, int) { return ESP_OK; }

int uart_write_bytes(uart_port_t, const void *src, size_t n) {
  const uint8_t *d = (const uint8_t *)src;
  double bt = 10e6 / host_baud;
  for (size_t i = 0; i < n; i++) {
    double start = std::max(now, host_tx_free);
    host_tx_free = start + bt;
    if (!tx_attached) floating_writes++;
    else chip_rx(d[i], host_tx_free, host_baud);
  }
  now = std::max(now, host_tx_free - 128 * bt);   // returns once the rest fits into the 128-byte FIFO
  return (int)n;
}

esp_err_t uart_wait_tx_done(uart_port_t, uint32_t ticks) {
  now = std::max(now, std::min(host_tx_free, now + ticks * 1000.0));
  return ESP_OK;
}

esp_err_t uart_get_buffered_data_len(uart_port_t, size_t *n) {
  deliver();
  *n = host_rx.size();
  return ESP_OK;
}

esp_err_t uart_flush_input(uart_port_t) {
  deliver();
  host_rx.clear();
  return ESP_OK;
}

int uart_read_bytes(uart_port_t, void *buf, uint32_t n, uint32_t ticks) {
  double deadline = now + ticks * 1000.0;
  for (;;) {
    deliver();
    if (host_rx.size() >= n || now >= deadline) break;
    double next = chip.out.empty() ? deadline : std::min(deadline, chip.out[0].t);
    if (chip.state == BOOTWIN) next = std::min(next, chip.on_at + chip.window_us + 1);
    now = std::max(now, next);
  }
  uint32_t k = std::min((uint32_t)host_rx.size(), n);
  memcpy(buf, host_rx.data(), k);
  host_rx.erase(host_rx.begin(), host_rx.begin() + k);
  return (int)k;
}

// ================================================================ a new target

static uint32_t rng = 1;
static uint8_t rnd() {
  rng = rng * 1103515245u + 12345u;
  return (uint8_t)(rng >> 16);
}

static void new_chip(const Chip &proto) {
  chip = proto;
  chip.flash.assign(chip.size, 0xFF);
  for (uint32_t a = 0; a < 0x100000 && a < chip.size; a++) chip.flash[a] = rnd();   // bootloader and app
  uint32_t cal = chip.rom ? 0x1D0000 : 0x1E0000;  // RF calibration with the MAC address (TLV)
  if (cal + BK_SECTOR <= chip.size) {
    static const uint8_t tag[8] = {0x02, 0x11, 0x11, 0x11, 0x06, 0x00, 0x00, 0x00};
    static const uint8_t mac[6] = {0xC8, 0x47, 0x8C, 0x12, 0x34, 0x56};
    memcpy(&chip.flash[cal + 0x1C], tag, 8);
    memcpy(&chip.flash[cal + 0x24], mac, 6);
  }
  chip.state = APP;
  chip.out.clear();
  chip.tx_free = now;
  if (chip.present && chip.reset_wired == false) enter_app(now);
}

// ================================================================ checks

static int failures = 0;
#define CHECK(cond, ...)                         \
  do {                                           \
    if (!(cond)) {                               \
      failures++;                                \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                       \
      printf("\n");                              \
    }                                            \
  } while (0)

static uint32_t zcrc(const uint8_t *d, uint32_t n) {   // zlib CRC-32
  uint32_t c = 0xFFFFFFFF;
  for (uint32_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return ~c;
}

static void fresh(Chip p = Chip()) {
  bk_end();
  new_chip(p);
  CHECK(bk_begin(), "begin");
}

static int link(uint32_t baud, BkLink *l, uint32_t ms = 10000) {
  int r = bk_link(ms, baud, l);
  if (r) printf("  link: %s\n", bk_err_text(r));
  return r;
}

static void test_link() {
  BkLink l;
  fresh();
  CHECK(link(115200, &l) == BK_OK && l.rom && l.baud == 115200 && !l.fallback, "link at 115200");
  CHECK(l.resets == 1 && l.answer_ms < 20, "caught at the first reset: %u resets, %u ms", (unsigned)l.resets,
        (unsigned)l.answer_ms);
  CHECK(chip.violations == 0, "violations: %s", chip.last_violation.c_str());
  CHECK(floating_writes == 0, "bytes sent while TX was floating: %d", floating_writes);

  fresh();
  delay(100);                                   // its application has been running and printing for a while:
                                                // the reset comes first, then the catch
  CHECK(link(921600, &l) == BK_OK && l.baud == 921600 && !l.fallback, "link at 921600");
  CHECK(chip.baud == 921600 && host_baud == 921600, "both at 921600: %u %u", (unsigned)chip.baud, (unsigned)host_baud);

  // a target whose UART does not work above 460800: back to 115200, still linked and usable
  Chip p;
  p.max_baud = 460800;
  fresh(p);
  CHECK(link(921600, &l) == BK_OK && l.fallback && l.baud == 115200, "fallback: %d %u", l.fallback, (unsigned)l.baud);
  uint8_t d[64];
  CHECK(bk_read(0x11000, d, 64) == BK_OK && !memcmp(d, &chip.flash[0x11000], 64), "read after the fallback");
  CHECK(link(460800, &l) == BK_OK && !l.fallback && l.baud == 460800, "460800 works on it");

  // a short window right after power-on: link checks back to back still land in it
  p = Chip();
  p.ready_us = 3000;
  p.window_us = 4500;
  fresh(p);
  CHECK(link(115200, &l) == BK_OK, "short window");

  // the flash id answer with the short length field of some BootROM revisions
  p = Chip();
  p.mid_len = 5;
  fresh(p);
  CHECK(link(921600, &l) == BK_OK, "link");
  char a0[] = "BKI";
  char *argv[] = {a0};
  static std::string answer;
  bk_command(1, argv, [](const char *s) { answer = s; });
  CHECK(answer == "OK 7231c eb6015", "BKI with the short length field: %s", answer.c_str());

  // failures
  p = Chip();
  p.present = false;
  fresh(p);
  CHECK(link(921600, &l, 3000) == BK_ERR_TIMEOUT && strstr(bk_detail(), "nothing at all came back"), "%s", bk_detail());
  p = Chip();
  p.reset_wired = false;
  fresh(p);
  CHECK(link(921600, &l, 3000) == BK_ERR_TIMEOUT && strstr(bk_detail(), "its own firmware runs") &&
            strstr(bk_detail(), "BK7231N app"),
        "%s", bk_detail());
  CHECK(l.resets >= 2, "resets while trying: %u", (unsigned)l.resets);
}

static void test_flash() {
  BkLink l;
  Chip p;
  p.noise = 30000;                              // one corrupted byte in 30000 above 115200
  fresh(p);
  CHECK(link(921600, &l) == BK_OK, "link");
  static uint8_t buf[0x12000];
  CHECK(bk_read(0x1234, buf, 0x11000) == BK_OK && !memcmp(buf, &chip.flash[0x1234], 0x11000),
        "read with noise: every block checked by CRC and read again");
  uint32_t c = 0;
  CHECK(bk_crc(0x100, 0x3000, &c) == BK_OK && c == zcrc(&chip.flash[0x100], 0x3000), "CRC");

  // erase and write: 64 k block and 4 k sectors, then CRC
  chip.noise = 0;
  CHECK(bk_erase(0x20000, true) == BK_OK && chip.erases64 == 1, "64 k erase");
  CHECK(bk_erase(0x31000, false) == BK_OK && chip.erases4 == 1, "4 k erase");
  bool blank = true;
  for (uint32_t i = 0; i < BK_BLOCK; i++) blank = blank && chip.flash[0x20000 + i] == 0xFF;
  CHECK(blank, "64 k block erased");
  CHECK(chip.flash[0x30FFF] != 0xFF || chip.flash[0x30FFE] != 0xFF, "the sector before the 4 k erase stays");
  for (uint32_t i = 0; i < BK_SECTOR; i++) buf[i] = (uint8_t)(i * 13 + 7);
  CHECK(bk_write4k(0x31000, buf) == BK_OK && !memcmp(&chip.flash[0x31000], buf, BK_SECTOR), "write 4 k");
  CHECK(bk_crc(0x31000, BK_SECTOR, &c) == BK_OK && c == zcrc(buf, BK_SECTOR), "CRC of the written sector");
  CHECK(bk_write4k(0x31800, buf) == BK_ERR_ALIGN && bk_erase(0x21000, true) == BK_ERR_ALIGN, "alignment");

  // the bootloader guard, also through the wrap-around
  CHECK(bk_erase(0x10000, false) == BK_ERR_GUARD && bk_write4k(0x0, buf) == BK_ERR_GUARD, "guard");
  CHECK(bk_erase(0x200000, false) == BK_ERR_GUARD, "guard at the flash size (wraps to 0)");
  CHECK(chip.erases4 == 1 && chip.writes == 1, "nothing reached the target");

  // flash protection: ignored silently by the flash, seen in the CRC
  chip.sr1 = 0x1C;
  CHECK(bk_erase(0x32000, false) == BK_OK && chip.ignored_writes == 1, "protected erase ignored");
  CHECK(chip.violations == 0, "violations: %s", chip.last_violation.c_str());

  // reboot: the application starts; the next link needs the catch again
  char a0[] = "BKX";
  char *argv[] = {a0};
  bk_command(1, argv, [](const char *) {});
  CHECK(chip.state == APP && !bk_linked(), "reboot");
  CHECK(link(921600, &l) == BK_OK && l.resets == 1, "caught again");
}

static void test_bootloader_protocol() {
  BkLink l;
  Chip p;
  p.rom = false;
  p.chip_id = 0;
  fresh(p);
  CHECK(link(921600, &l) == BK_OK && !l.rom && l.baud == 921600, "BK7231T bootloader protocol: %d", l.rom);
  static uint8_t buf[BK_SECTOR];
  uint32_t c = 0;
  CHECK(bk_crc(0x11000, 0x1000, &c) == BK_OK && c == zcrc(&chip.flash[0x11000], 0x1000), "CRC, end exclusive");
  CHECK(bk_read(0x0, buf, 16) == BK_ERR_CRC, "its own area is refused");
  CHECK(bk_read(0x200000, buf, BK_SECTOR) == BK_OK && !memcmp(buf, &chip.flash[0], BK_SECTOR),
        "but readable at the address plus the flash size");
  // after a CRC the flash is write protected until a link check: the firmware sends one
  for (uint32_t i = 0; i < BK_SECTOR; i++) buf[i] = (uint8_t)i;
  CHECK(bk_erase(0x40000, false) == BK_OK && bk_write4k(0x40000, buf) == BK_OK, "erase, write");
  CHECK(chip.ignored_writes == 0 && !memcmp(&chip.flash[0x40000], buf, BK_SECTOR), "not locked by the CRC");
  char a0[] = "BKI";
  char *argv[] = {a0};
  static std::string answer;
  bk_command(1, argv, [](const char *s) { answer = s; });
  CHECK(answer == "OK 0 000000", "BKI without registers: %s", answer.c_str());
  CHECK(chip.violations == 0, "violations: %s", chip.last_violation.c_str());
}

// ================================================================ serve: the text protocol for the Python tests

static void out_line(const char *s) {
  fputs(s, stdout);
  fputc('\n', stdout);
  fflush(stdout);
}

static void serve() {
  static char ln[20000];
  new_chip(Chip());
  while (fgets(ln, sizeof(ln), stdin)) {
    char *argv[12] = {NULL};
    int argc = 0;
    for (char *tok = strtok(ln, " \t\r\n"); tok && argc < 12; tok = strtok(NULL, " \t\r\n")) argv[argc++] = tok;
    if (!argc) continue;
    if (!strcmp(argv[0], "SIM")) {
      // SIM new [rom=0] [present=0] [reset=0] [window=us] [ready=us] [maxbaud=n] [noise=n] [id=hex6]
      //         [sr1=hex] [sr2=hex] [srbytes=1|2] [midlen=5|6] [size=n]
      // SIM peek <addr> <len>, SIM poke <addr> <hex>, SIM fill <addr> <len> <byte>, SIM stat
      static std::string o;
      if (argc > 1 && !strcmp(argv[1], "new")) {
        Chip p;
        for (int i = 2; i < argc; i++) {
          char *eq = strchr(argv[i], '=');
          if (!eq) continue;
          *eq = 0;
          const char *k = argv[i], *v = eq + 1;
          uint32_t n = strtoul(v, NULL, 0);
          if (!strcmp(k, "rom")) p.rom = n != 0, p.chip_id = n ? 0x7231C : 0;
          if (!strcmp(k, "present")) p.present = n != 0;
          if (!strcmp(k, "reset")) p.reset_wired = n != 0;
          if (!strcmp(k, "window")) p.window_us = n;
          if (!strcmp(k, "ready")) p.ready_us = n;
          if (!strcmp(k, "maxbaud")) p.max_baud = n;
          if (!strcmp(k, "noise")) p.noise = n;
          if (!strcmp(k, "size")) p.size = n;
          if (!strcmp(k, "srbytes")) p.sr_bytes = (int)n;
          if (!strcmp(k, "midlen")) p.mid_len = (int)n;
          if (!strcmp(k, "sr1")) p.sr1 = (uint8_t)strtoul(v, NULL, 16);
          if (!strcmp(k, "sr2")) p.sr2 = (uint8_t)strtoul(v, NULL, 16);
          if (!strcmp(k, "id")) {
            uint32_t x = strtoul(v, NULL, 16);
            p.id[0] = x >> 16;
            p.id[1] = x >> 8;
            p.id[2] = x;
          }
        }
        new_chip(p);
        out_line("OK");
      } else if (argc > 3 && !strcmp(argv[1], "peek")) {
        uint32_t a = strtoul(argv[2], NULL, 0), n = strtoul(argv[3], NULL, 0);
        o = "OK ";
        char h[3];
        for (uint32_t i = 0; i < n; i++) {
          snprintf(h, sizeof(h), "%02x", *at(a + i));
          o += h;
        }
        out_line(o.c_str());
      } else if (argc > 3 && !strcmp(argv[1], "poke")) {
        uint32_t a = strtoul(argv[2], NULL, 0);
        for (size_t i = 0; argv[3][2 * i] && argv[3][2 * i + 1]; i++) {
          char h[3] = {argv[3][2 * i], argv[3][2 * i + 1], 0};
          *at(a + (uint32_t)i) = (uint8_t)strtoul(h, NULL, 16);
        }
        out_line("OK");
      } else if (argc > 4 && !strcmp(argv[1], "fill")) {
        uint32_t a = strtoul(argv[2], NULL, 0), n = strtoul(argv[3], NULL, 0);
        uint8_t v = (uint8_t)strtoul(argv[4], NULL, 0);
        for (uint32_t i = 0; i < n; i++) *at(a + i) = v;
        out_line("OK");
      } else if (argc > 1 && !strcmp(argv[1], "stat")) {
        static const char *names[] = {"off", "window", "download", "app"};
        char t[400];
        snprintf(t, sizeof(t),
                 "OK violations=%d state=%s baud=%u sr=%02x%02x erase4=%d erase64=%d writes=%d reads=%d crcs=%d "
                 "srwrites=%d ignored=%d floating=%d time_ms=%u last=%s",
                 chip.violations, names[chip.state], (unsigned)chip.baud, chip.sr2, chip.sr1, chip.erases4,
                 chip.erases64, chip.writes, chip.reads, chip.crcs, chip.sr_writes, chip.ignored_writes,
                 floating_writes, millis(), chip.last_violation.empty() ? "-" : chip.last_violation.c_str());
        out_line(t);
      } else {
        out_line("ERR SIM new|peek|poke|fill|stat");
      }
      continue;
    }
    if (!strcmp(argv[0], "V")) {
      out_line("OK tlsr-c3-flasher 0.24 simulated BK72xx");
      continue;
    }
    if (!strcmp(argv[0], "PWR")) {
      out_line("OK");
      continue;
    }
    if (!bk_command(argc, argv, out_line)) out_line("ERR unknown command");
  }
}

int main(int argc, char **argv) {
  bk_set_reset(sim_reset);
  if (argc > 1 && !strcmp(argv[1], "--serve")) {
    serve();
    return 0;
  }
  test_link();
  test_flash();
  test_bootloader_protocol();
  printf(failures ? "%d FAILED\n" : "all BK72xx simulation checks passed\n", failures);
  return failures ? 1 : 0;
}
