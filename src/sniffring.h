// Live output of the two-line UART logger (sniff.h): a ring of frames waiting for the host.
//
// Frame: tag:u8 (1 = line B, 2 = line A, 3 = event text) t_us:u32 n:u16 (both little endian) and n bytes of data.
// t_us is the flasher's microsecond clock (it wraps after 71 minutes) when the bytes were taken from the UART driver.
// A frame goes into the ring whole or not at all: when it does not fit (the host takes data slower than the lines
// bring it), its data bytes are counted as dropped, and as soon as there is room again an event frame
// "live output: N bytes dropped" goes in before the next one. Only the live output loses bytes; the UART log gets
// everything. The capture never waits for the host: the ring is drained by whoever has room (main.cpp).
// Pure logic without Arduino, built natively by tests/sniff_sim.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const int SNIFF_FRAME_HDR = 7;
static const int SNIFF_MAX_DATA = 1024;      // longer pieces are split into several frames

enum { SNIFF_TAG_B = 1, SNIFF_TAG_A = 2, SNIFF_TAG_EVENT = 3 };

struct SniffRing {
  uint8_t *buf = nullptr;
  uint32_t cap = 0, start = 0, len = 0;
  uint32_t dropped = 0;                       // data bytes dropped since the last report
  uint32_t dropped_total = 0;
  uint32_t last_t = 0;                        // the time of the last frame offered (for the report's own frame)

  void init(uint8_t *mem, uint32_t size) {
    buf = mem;
    cap = size;
    clear();
    dropped_total = 0;
  }
  void clear() {
    start = len = 0;
    dropped = 0;
  }
  uint32_t free_space() const { return cap - len; }

  // one piece of data (or event text): split into frames of at most SNIFF_MAX_DATA bytes; while a report of dropped
  // bytes is waiting for room, what comes after is dropped too, so that the report stands where the gap is
  void push(int tag, uint32_t t_us, const uint8_t *d, int n) {
    last_t = t_us;
    do {
      const int k = n < SNIFF_MAX_DATA ? n : SNIFF_MAX_DATA;
      report();                               // a gap is reported before anything that came after it
      if (dropped || !put_frame(tag, t_us, d, k)) {
        dropped += k;
        dropped_total += k;
      }
      d += k;
      n -= k;
    } while (n > 0);
  }

  void event(uint32_t t_us, const char *text) { push(SNIFF_TAG_EVENT, t_us, (const uint8_t *)text, strlen(text)); }

  // a pending report of dropped bytes goes in when there is room (also called after the host took data)
  void report() {
    if (!dropped) return;
    char text[48];
    const int n = snprintf(text, sizeof(text), "live output: %u bytes dropped", (unsigned)dropped);
    if (put_frame(SNIFF_TAG_EVENT, last_t, (const uint8_t *)text, n)) dropped = 0;
  }

  // the bytes that can be sent now in one piece (the ring may hold more after them)
  int peek(const uint8_t **p) const {
    *p = buf + start;
    const uint32_t first = cap - start;
    return (int)(len < first ? len : first);
  }

  void consume(int n) {
    if (n <= 0) return;
    if ((uint32_t)n > len) n = (int)len;
    start = (start + n) % cap;
    len -= n;
    if (!len) start = 0;
  }

 private:
  void put(const uint8_t *d, int n) {
    uint32_t at = (start + len) % cap;
    for (int i = 0; i < n; i++) {           // frames are small: byte by byte keeps the wrap simple
      buf[at] = d[i];
      at = at + 1 == cap ? 0 : at + 1;
    }
    len += n;
  }

  bool put_frame(int tag, uint32_t t_us, const uint8_t *d, int n) {
    if (!buf || (uint32_t)(SNIFF_FRAME_HDR + n) > free_space()) return false;
    const uint8_t h[SNIFF_FRAME_HDR] = {(uint8_t)tag,        (uint8_t)t_us,       (uint8_t)(t_us >> 8),
                                        (uint8_t)(t_us >> 16), (uint8_t)(t_us >> 24), (uint8_t)n,
                                        (uint8_t)(n >> 8)};
    put(h, SNIFF_FRAME_HDR);
    put(d, n);
    return true;
  }
};
