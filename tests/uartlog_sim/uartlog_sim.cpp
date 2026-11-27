// Runs src/uartlog.cpp natively on a made-up UART session and writes what the flasher would export (XR L) and the
// records that went in, for tests/test_uartlog.py:
//   uartlog_sim <scenario> <export.bin> <records.txt>
// scenarios: small, long (the whole store), long-wifi (a heap that leaves a store of 26 k after the reserve, as with
// Wi-Fi on: all ring), long-nomem (less than the smallest store: no log)
// records.txt: one line per record, "<t_us> <tag> <hex payload>" (events as hex too).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "../../src/uartlog.h"

static uint32_t clock_us = 0xFFFF0000u;         // starts near the 32-bit wrap: times must unwrap
uint32_t ulog_test_now_us() { return clock_us; }
static uint32_t heap = 1u << 30;
uint32_t ulog_test_heap_largest() { return heap; }

static FILE *rec;
static uint32_t rng = 12345;
static uint32_t rnd() { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static void put(int tag, const uint8_t *d, int n) {
  ulog_data(tag, d, n);
  fprintf(rec, "%u %d ", clock_us, tag);
  for (int i = 0; i < n; i++) fprintf(rec, "%02x", d[i]);
  fprintf(rec, "\n");
}
static void ev(const char *t) { put(ULOG_EVENT, (const uint8_t *)t, strlen(t)); }

int main(int argc, char **argv) {
  if (argc < 4) return 2;
  std::string sc = argv[1];
  rec = fopen(argv[3], "w");
  if (sc == "long-wifi") heap = ULOG_RESERVE_WIFI + 26 * 1024 + 100;
  if (sc == "long-nomem") heap = ULOG_RESERVE_WIFI + 20 * 1024;
  if (sc != "small" && sc != "long") ulog_set_reserve(ULOG_RESERVE_WIFI);
  ulog_clear();
  ev("Beken download, 115200 baud");
  static const uint8_t lc[5] = {0x01, 0xE0, 0xFC, 0x01, 0x00};
  int checks = sc == "small" ? 30 : 3000;
  for (int i = 0; i < checks; i++) {             // the catch: link checks back to back, a reset now and then
    if (i % 1000 == 0) ev("target reset (catch)");
    put(ULOG_TX, lc, 5);
    clock_us += 430;
  }
  static const uint8_t ans[8] = {0x04, 0x0E, 0x05, 0x01, 0xE0, 0xFC, 0x01, 0x00};
  put(ULOG_RX, ans, 8);
  ev("baud 921600");
  int blocks = sc == "small" ? 3 : 512;          // 512 x 4 k = 2 MB of flash read
  std::vector<uint8_t> b(4096 + 15);
  for (int k = 0; k < blocks; k++) {
    uint8_t cmd[12] = {0x01, 0xE0, 0xFC, 0xFF, 0xF4, 5, 0, 0x09};
    memcpy(cmd + 8, &k, 4);
    put(ULOG_TX, cmd, 12);
    clock_us += 100;
    int n = 4096 + 15;
    for (int i = 0; i < n; i++) b[i] = (k % 3 == 0) ? 0xFF : (k % 3 == 1 ? (uint8_t)rnd() : (uint8_t)(i * 7));
    for (int off = 0; off < n;) {                // arrives in pieces of up to 120 bytes
      int c = 1 + rnd() % 120;
      if (c > n - off) c = n - off;
      put(ULOG_RX, b.data() + off, c);
      off += c;
      clock_us += c * 11;
    }
  }
  ev("end");
  ulog_flush();
  FILE *o = fopen(argv[2], "wb");
  uint32_t len = ulog_export_len();
  std::vector<uint8_t> x(len);
  ulog_export(0, x.data(), (int)len);
  fwrite(x.data(), 1, len, o);
  fclose(o);
  fclose(rec);
  UlogStats st = ulog_stats();
  printf("blocks=%u stored=%u raw=%u dropped=%u tx=%u rx=%u cap=%u\n", st.blocks, st.stored, st.raw, st.dropped, st.tx,
         st.rx, st.capacity);
  return 0;
}
