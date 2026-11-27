// LN882H RAM loader of tlsr-c3-flasher ("C3LN"): reads, erases and writes the chip's SPI flash for tlsr_c3.py.
//
// The chip's ROM loads it over YMODEM to 0x20000000 (`download [rambin] [0x20000000] [size]`) and starts it. It talks
// on UART0 (A2 TX, A3 RX) at 115200, polled, with interrupts off. It is reached through the flasher's UART bridge in
// its "ln" mode (BRIDGE 115200 lo ln), which passes every byte both ways and switches its own speed at the end of a
// host line "baudrate <digits>" (src/lnscan.h).
//
// Start (ln882h.c hal_start). It reads what the ROM left (clocks, UART0, pins; the info answer reports it), says
//     \r\nC3LN 2 rom clock\r\n
// on UART0 as the ROM left it (for the log only: garbled when the ROM's divider does not fit the clock it hands over
// with), then sets the clocks itself as Lightning's loader does (PLL, core 160 MHz, APB0 40 MHz) and UART0 from its
// reset state at 115200 with the divider computed from that clock. Then a banner and the number of each init stage as
// the stage begins, each drained before the stage runs (loader.h: 1 clocks and UART, 2 cache off, 3 QSPI set-up,
// 4 JEDEC id, 5 status / unique id / MAC), and "ready" or the error:
//     \r\nC3LN 2 init 1 2 3 4 5 ready\r\n          C3LN 2 init 1 2 3 4 error 13\r\n
// so the UART log (tlsr_c3.py log) shows how far a start got even when it hangs. After an init error the loader
// still answers version, info and reboot; flash commands answer with the init error.
// Until it has received a valid line (version, baudrate) or frame it repeats every 200 ms
//     \r\nC3LN 2 wait rx <bytes received> err <receive errors> ready\r\n     (or "error <code> at <stage>")
// so the host can tell a loader that runs but does not hear it (rx 0: nothing arrives; errors: a speed mismatch) from
// one that stopped, and finds its speed by listening when the clock is not what it should be.
//
// Two kinds of input:
// - Text lines, only between frames: printable characters ended by CR or LF.
//     version            -> "C3LN <protocol> ready\r\n", or "C3LN <protocol> error <code> at <stage>\r\n"
//     baudrate <digits>  -> no answer; the loader waits until its last byte has left, then switches at the line's
//                           end, as the bridge does (so nothing is ever sent at the old speed after the line)
//   Anything else is ignored, so the ROM's and the vendor loader's line commands do no harm.
// - Binary frames, both directions:   C0  escaped(body)  C1
//     body = type u8, seq u8, status u8, length u16, data[length], CRC-32 u32 (zlib's, of everything before it)
//   integers little-endian. Inside the frame the bytes C0, C1, DB, 0A (LF) and 0D (CR) are sent as DB, b ^ 0x20.
//   So a frame never holds CR or LF: the bridge cannot find a "baudrate" line in one (its line scanner is spoiled by
//   the frame's first byte and stays so up to the next CR / LF, which only a real text line brings), and the host's
//   data needs no check for one. A raw CR / LF inside a frame ends the frame as broken.
//   A request has status 0 and seq chosen by the host; the answer has type | 0x80, the same seq, and a status
//   (loader.h). An error answer's data is a 32-bit detail (an address or a status register value).
//
// Resynchronisation. C0 always starts a new frame and drops a partial one, C1 ends one, so the next frame after any
// damage is read again. A frame whose CRC, escape or length is wrong is answered with type FF and the status
// (E_CRC / E_FRAME), so the host can send it again at once instead of waiting. A request that changes the flash
// (erase, write, status write) and comes again with the same seq and CRC (the host repeating it because the answer
// was lost) right after it is not done again: its stored answer is sent.
//
// Commands (data of the request -> data of the answer):
//   01 info                                      -> 160 bytes, see cmd_info
//   02 read    addr u32, len u32 (1..4096)       -> the data (the frame's CRC-32 checks it)
//   03 crc     addr u32, len u32                 -> zlib CRC-32 u32 of the range
//   04 erase   addr u32, len u32 (4 k multiples) -> nothing; 64 k blocks where a whole aligned one is in the range,
//                                                   4 k sectors elsewhere; each one is read back blank or E_ERASE
//   05 write   addr u32, data (1..4096 bytes)    -> nothing; page programs, no erase: E_NOT_ERASED (nothing
//                                                   written) if a 0 bit would have to become 1; all-FF pages are
//                                                   skipped; read back after it or E_VERIFY
//   06 status                                    -> status registers 05, 35, 15 (3 bytes)
//   07 srwrite n u8 (1 or 2), n bytes            -> 06 then 01 with the n bytes; the 3 status registers after it
//   08 reboot                                    -> nothing; the chip restarts after the answer
// Nothing erases or writes anything it was not asked to. The flash size comes from the JEDEC id's capacity byte;
// a range beyond it is refused (E_RANGE).
#include "loader.h"
#include <string.h>
#include "flash.h"
#include "hal.h"

#define STR_(x) #x
#define STR(x) STR_(x)
#define LINE_MAX 40
#define INFO_SIZE 160
#define WAIT_PERIOD_DIV 5               // the wait line: every core_hz / 5 cycles, 200 ms

static struct {
  // input
  int in_frame, esc, bad_frame;
  uint32_t n;
  uint8_t body[LN_MAX_BODY];
  char line[LINE_MAX];
  int line_n, bad_line;
  // start
  uint8_t stage, init_error;
  uint32_t init_detail, id, size, cpu_hz, baud;
  uint8_t sr[3], uid[16], mac[6];
  struct ln_regs regs;
  struct ln_clock clk;
  // until the host is heard: the wait line
  int linked;
  uint32_t rx_bytes, last_cycles, elapsed;
  // the last request that changed the flash, for repeats
  int last_valid;
  uint8_t last_type, last_seq, last_status;
  uint32_t last_crc, last_detail;
  // output
  uint8_t out[256];
  uint32_t out_n;
  uint8_t buf[LN_MAX_DATA];
} st;

static uint32_t crc_table[256];

static void crc_init(void) {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    crc_table[i] = c;
  }
}

// zlib's crc32(): start with 0, feed the pieces in order
static uint32_t crc32(uint32_t crc, const uint8_t *p, uint32_t n) {
  crc = ~crc;
  while (n--) crc = crc_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

static uint32_t get32(const uint8_t *p) {
  return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// ---------------------------------------------------------------- output
static void out_flush(void) {
  if (st.out_n) hal_uart_write(st.out, st.out_n);
  st.out_n = 0;
}

static void out_raw(uint8_t c) {
  if (st.out_n == sizeof(st.out)) out_flush();
  st.out[st.out_n++] = c;
}

static int special(uint8_t c) {
  return c == LN_FR_START || c == LN_FR_END || c == LN_FR_ESC || c == '\n' || c == '\r';
}

static void out_esc(const uint8_t *p, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    if (special(p[i])) {
      out_raw(LN_FR_ESC);
      out_raw(p[i] ^ 0x20);
    } else {
      out_raw(p[i]);
    }
  }
}

static void put_str(const char *s) {
  while (*s) out_raw((uint8_t)*s++);
}

static void put_dec(uint32_t v) {
  char b[11];
  int i = 10;
  b[i] = 0;
  do {
    b[--i] = (char)('0' + v % 10);
    v /= 10;
  } while (v);
  put_str(b + i);
}

static void answer(uint8_t type, uint8_t seq, uint8_t status, const uint8_t *data, uint32_t n) {
  uint8_t head[LN_HEAD] = {type, seq, status, (uint8_t)n, (uint8_t)(n >> 8)};
  uint8_t crc[4];
  put32(crc, crc32(crc32(0, head, LN_HEAD), data, n));
  out_raw(LN_FR_START);
  out_esc(head, LN_HEAD);
  out_esc(data, n);
  out_esc(crc, 4);
  out_raw(LN_FR_END);
  out_flush();
}

static void answer_error(uint8_t type, uint8_t seq, uint8_t status, uint32_t detail) {
  uint8_t d[4];
  put32(d, detail);
  answer(type, seq, status, d, 4);
}

// ---------------------------------------------------------------- commands
static int in_flash(uint32_t addr, uint32_t len) {
  return addr <= st.size && len <= st.size - addr;
}

// protocol, init stage, init error, 1 = core from the PLL; init detail, JEDEC id, size; status registers 05 35 15;
// unique id; MAC; core clock measured over the banner, baud, receive errors, data limit, core and APB0 clock set,
// bytes received; then struct ln_regs (what the ROM left, all ones = not read)
static void cmd_info(uint8_t seq) {
  uint8_t d[INFO_SIZE];
  memset(d, 0, sizeof(d));
  d[0] = LN_PROTO;
  d[1] = st.stage;
  d[2] = st.init_error;
  d[3] = (uint8_t)st.clk.pll;
  put32(d + 4, st.init_detail);
  put32(d + 8, st.id);
  put32(d + 12, st.size);
  memcpy(d + 16, st.sr, 3);
  memcpy(d + 20, st.uid, 16);
  memcpy(d + 36, st.mac, 6);
  put32(d + 44, st.cpu_hz);
  put32(d + 48, st.baud);
  put32(d + 52, hal_uart_errors());
  put32(d + 56, LN_MAX_DATA);
  put32(d + 60, st.clk.core_hz);
  put32(d + 64, st.clk.apb_hz);
  put32(d + 68, st.rx_bytes);
  const uint32_t *r = &st.regs.uart_brr;  // struct ln_regs: LN_NREGS words in a row
  for (int i = 0; i < LN_NREGS; i++) put32(d + 72 + 4 * i, r[i]);
  answer(LN_C_INFO | 0x80, seq, LN_OK, d, sizeof(d));
}

// the address of the first byte in [addr, addr + len) that is not FF, or 0xFFFFFFFF; *e: the read's error
static uint32_t first_not_blank(uint32_t addr, uint32_t len, int *e) {
  for (uint32_t pos = 0; pos < len; pos += LN_MAX_DATA) {
    uint32_t n = len - pos < LN_MAX_DATA ? len - pos : LN_MAX_DATA;
    *e = fl_read(addr + pos, st.buf, n);
    if (*e) return addr + pos;
    for (uint32_t i = 0; i < n; i++)
      if (st.buf[i] != 0xFF) return addr + pos + i;
  }
  return 0xFFFFFFFFu;
}

static int do_erase(uint32_t addr, uint32_t len, uint32_t *detail) {
  if (len == 0 || addr % FL_SECTOR || len % FL_SECTOR) return LN_E_ALIGN;
  if (!in_flash(addr, len)) return LN_E_RANGE;
  while (len) {
    int block = addr % FL_BLOCK == 0 && len >= FL_BLOCK;
    uint32_t n = block ? FL_BLOCK : FL_SECTOR;
    int e = fl_erase(addr, block);
    if (e) {
      *detail = e == LN_E_QSPI ? addr : fl_detail;
      return e;
    }
    uint32_t bad = first_not_blank(addr, n, &e);
    if (e || bad != 0xFFFFFFFFu) {
      *detail = bad;
      return e ? e : LN_E_ERASE;
    }
    addr += n;
    len -= n;
  }
  return LN_OK;
}

static int do_write(uint32_t addr, const uint8_t *data, uint32_t len, uint32_t *detail) {
  if (len == 0 || len > LN_MAX_DATA) return LN_E_ARGS;
  if (!in_flash(addr, len)) return LN_E_RANGE;
  *detail = addr;
  int e = fl_read(addr, st.buf, len);
  if (e) return e;
  for (uint32_t i = 0; i < len; i++) {
    if ((st.buf[i] & data[i]) != data[i]) {
      *detail = addr + i;
      return LN_E_NOT_ERASED;
    }
  }
  uint32_t pos = 0;
  while (pos < len) {
    uint32_t a = addr + pos;
    uint32_t n = FL_PAGE - a % FL_PAGE;
    if (n > len - pos) n = len - pos;
    uint32_t k = 0;
    while (k < n && data[pos + k] == 0xFF) k++;
    if (k < n) {                        // an all-FF piece changes nothing: not sent
      e = fl_program(a, data + pos, n);
      if (e) {
        *detail = e == LN_E_QSPI ? a : fl_detail;
        return e;
      }
    }
    pos += n;
  }
  e = fl_read(addr, st.buf, len);
  if (e) return e;
  for (uint32_t i = 0; i < len; i++) {
    if (st.buf[i] != data[i]) {
      *detail = addr + i;
      return LN_E_VERIFY;
    }
  }
  return LN_OK;
}

// one request; the answer is the data (out, n) or, with an error, the 32-bit detail
static void execute(uint8_t type, uint8_t seq, const uint8_t *d, uint32_t len, uint32_t crc) {
  uint8_t sr[3], r[4];
  const uint8_t *out = 0;
  uint32_t n = 0, detail = len;
  int e = LN_OK;
  int changes = type == LN_C_ERASE || type == LN_C_WRITE || type == LN_C_SRWRITE;
  int repeat = changes && st.last_valid && type == st.last_type && seq == st.last_seq && crc == st.last_crc;
  if (type == LN_C_INFO) {
    if (!len) {
      cmd_info(seq);
      return;
    }
    e = LN_E_ARGS;
  } else if (type == LN_C_REBOOT) {
    answer(type | 0x80, seq, LN_OK, 0, 0);
    hal_uart_drain();
    hal_reboot();
    return;
  } else if (type < LN_C_READ || type > LN_C_SRWRITE) {
    e = LN_E_CMD;
    detail = type;
  } else if (st.stage != LN_S_READY) {
    e = st.init_error;
    detail = st.init_detail;
  } else if (repeat) {                  // the host did not get the answer: the stored one, without doing it again
    e = st.last_status;
    detail = st.last_detail;
    if (!e && type == LN_C_SRWRITE) {
      e = fl_status(sr);
      out = sr;
      n = 3;
    }
  } else if (type == LN_C_STATUS) {
    if (len) {
      e = LN_E_ARGS;
    } else {
      e = fl_status(sr);
      out = sr;
      n = 3;
    }
  } else if (type == LN_C_SRWRITE) {
    if (len < 2 || (d[0] != 1 && d[0] != 2) || len != 1u + d[0]) {
      e = LN_E_ARGS;
    } else {
      e = fl_write_sr(d + 1, d[0]);
      detail = fl_detail;
      if (!e) e = fl_status(sr);
      if (!e) memcpy(st.sr, sr, 3);
      out = sr;
      n = 3;
    }
  } else if (type == LN_C_WRITE) {
    if (len < 5) {
      e = LN_E_ARGS;
    } else {
      e = do_write(get32(d), d + 4, len - 4, &detail);
    }
  } else if (len != 8) {                // read, crc, erase: address, length
    e = LN_E_ARGS;
  } else {
    uint32_t addr = get32(d), size = get32(d + 4);
    detail = addr;
    if (type == LN_C_ERASE) {
      e = do_erase(addr, size, &detail);
    } else if (type == LN_C_READ && (size == 0 || size > LN_MAX_DATA)) {
      e = LN_E_ARGS;
      detail = size;
    } else if (!in_flash(addr, size)) {
      e = LN_E_RANGE;
    } else if (type == LN_C_READ) {
      e = fl_read(addr, st.buf, size);
      out = st.buf;
      n = size;
    } else {
      uint32_t c = 0;
      for (uint32_t pos = 0; pos < size && !e; pos += LN_MAX_DATA) {
        uint32_t k = size - pos < LN_MAX_DATA ? size - pos : LN_MAX_DATA;
        e = fl_read(addr + pos, st.buf, k);
        c = crc32(c, st.buf, k);
      }
      put32(r, c);
      out = r;
      n = 4;
    }
  }
  if (!changes) {
    st.last_valid = 0;                  // the host has moved on: a later request with this seq is a new one
  } else if (!repeat) {
    st.last_valid = 1;
    st.last_type = type;
    st.last_seq = seq;
    st.last_crc = crc;
    st.last_status = (uint8_t)e;
    st.last_detail = detail;
  }
  if (e) {
    answer_error(type | 0x80, seq, (uint8_t)e, detail);
  } else {
    answer(type | 0x80, seq, LN_OK, out, n);
  }
}

static void frame_end(void) {
  uint8_t seq = st.n > 1 ? st.body[1] : 0;
  if (st.bad_frame || st.n < LN_HEAD + 4) {
    answer_error(LN_T_ERROR, seq, LN_E_FRAME, st.n);
    return;
  }
  uint32_t crc = get32(st.body + st.n - 4);
  if (crc32(0, st.body, st.n - 4) != crc) {
    answer_error(LN_T_ERROR, seq, LN_E_CRC, crc);
    return;
  }
  uint32_t len = st.body[3] | (uint32_t)st.body[4] << 8;
  if (LN_HEAD + len + 4 != st.n) {
    answer_error(LN_T_ERROR, seq, LN_E_FRAME, st.n);
    return;
  }
  st.linked = 1;
  execute(st.body[0], seq, st.body + LN_HEAD, len, crc);
}

// ---------------------------------------------------------------- text lines
static void put_result(void) {
  if (st.stage == LN_S_READY) {
    put_str(" ready\r\n");
  } else {
    put_str(" error ");
    put_dec(st.init_error);
    put_str(" at ");
    put_dec(st.stage);
    put_str("\r\n");
  }
  out_flush();
}

static void version_line(void) {
  put_str("C3LN " STR(LN_PROTO));
  put_result();
}

// until the host is heard: alive, and what has come in so far
static void wait_line(void) {
  put_str("\r\nC3LN " STR(LN_PROTO) " wait rx ");
  put_dec(st.rx_bytes);
  put_str(" err ");
  put_dec(hal_uart_errors());
  put_result();
}

static void line_end(void) {
  static const char key[] = "baudrate ";
  const int k = sizeof(key) - 1;
  st.line[st.line_n] = 0;
  if (st.bad_line || st.line_n == 0) return;
  if (!strcmp(st.line, "version")) {
    st.linked = 1;
    version_line();
    return;
  }
  if (st.line_n > k && !memcmp(st.line, key, k)) {
    uint32_t baud = 0;
    for (int i = k; i < st.line_n; i++) {
      if (st.line[i] < '0' || st.line[i] > '9' || baud > 100000000u) return;
      baud = baud * 10 + (uint32_t)(st.line[i] - '0');
    }
    if (baud < 1200 || baud > 5000000) return;
    st.linked = 1;
    hal_uart_drain();
    if (hal_uart_set_baud(baud) == 0) st.baud = baud;
  }
}

static void feed(uint8_t c) {
  if (c == LN_FR_START) {
    st.in_frame = 1;
    st.n = 0;
    st.esc = st.bad_frame = 0;
    return;
  }
  if (st.in_frame) {
    if (c == LN_FR_END) {
      st.in_frame = 0;
      st.line_n = st.bad_line = 0;
      frame_end();
      return;
    }
    if (c == '\r' || c == '\n') {       // never inside a frame: the end of it was lost; this is a text line's end
      st.in_frame = 0;
      st.line_n = st.bad_line = 0;
      answer_error(LN_T_ERROR, st.n > 1 ? st.body[1] : 0, LN_E_FRAME, st.n);
      return;
    }
    if (st.esc) {
      st.esc = 0;
      c ^= 0x20;
      if (!special(c)) st.bad_frame = 1;
    } else if (c == LN_FR_ESC) {
      st.esc = 1;
      return;
    }
    if (st.n < sizeof(st.body)) {
      st.body[st.n++] = c;
    } else {
      st.bad_frame = 1;
    }
    return;
  }
  if (c == '\r' || c == '\n') {
    line_end();
    st.line_n = st.bad_line = 0;
  } else if (c >= 0x20 && c <= 0x7E && st.line_n < LINE_MAX - 1) {
    st.line[st.line_n++] = (char)c;
  } else {
    st.bad_line = 1;
  }
}

void loader_poll(void) {
  uint8_t b[64];
  uint32_t n;
  while ((n = hal_uart_read(b, sizeof(b))) > 0) {
    st.rx_bytes += n;
    for (uint32_t i = 0; i < n; i++) feed(b[i]);
  }
  if (!st.linked) {
    uint32_t now = hal_cycles();
    st.elapsed += (now - st.last_cycles) & 0xFFFFFFu;   // polled far more often than the counter wraps (100 ms)
    st.last_cycles = now;
    if (st.elapsed >= st.clk.core_hz / WAIT_PERIOD_DIV) {
      st.elapsed = 0;
      wait_line();
    }
  }
}

// ---------------------------------------------------------------- start
static int init_failed(int e, uint32_t detail) {
  st.init_error = (uint8_t)e;
  st.init_detail = detail;
  put_str(" error ");
  put_dec((uint32_t)e);
  put_str("\r\n");
  out_flush();
  return e;
}

// the stage's number, sent out completely before the stage runs
static void stage(uint8_t s) {
  st.stage = s;
  out_raw(' ');
  put_dec(s);
  out_flush();
  hal_uart_drain();
}

void loader_init(void) {
  memset(&st, 0, sizeof(st));
  crc_init();
  st.stage = LN_S_UART;
  st.baud = 115200;
  hal_start(&st.regs, &st.clk, "\r\nC3LN " STR(LN_PROTO) " rom clock\r\n");
  static const char banner[] = "\r\nC3LN " STR(LN_PROTO) " init 1";
  uint32_t t0 = hal_cycles();
  put_str(banner);
  out_flush();
  hal_uart_drain();
  // the banner's bits took this many core cycles: the clock, to a few percent (a check of the clock against UART0)
  st.cpu_hz = ((hal_cycles() - t0) & 0xFFFFFFu) / (10 * (sizeof(banner) - 1)) * st.baud;
  st.last_cycles = hal_cycles();

  stage(LN_S_CACHE);
  hal_flash_cache_off(&st.regs);
  stage(LN_S_QSPI);
  hal_flash_take(&st.regs);
  int e = fl_wake();
  if (e) {
    init_failed(e, 0);
    return;
  }
  stage(LN_S_ID);
  e = fl_id(&st.id);
  if (e) {
    init_failed(e, 0);
    return;
  }
  uint32_t cap = st.id & 0xFF;
  if (st.id == 0 || st.id == 0xFFFFFF || cap < 0x10 || cap > 0x18) {
    init_failed(LN_E_NOFLASH, st.id);
    return;
  }
  st.size = 1u << cap;
  stage(LN_S_DETAILS);
  e = fl_status(st.sr);
  if (!e) e = fl_uid(st.uid);
  if (!e) e = fl_otp_mac(st.mac);
  if (e) {
    init_failed(e, 0);
    return;
  }
  st.stage = LN_S_READY;
  put_str(" ready\r\n");
  out_flush();
}
