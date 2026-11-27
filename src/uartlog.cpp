#include "uartlog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#if __has_include(<Arduino.h>)              // the firmware, and the simulations with their stub Arduino.h
#include <Arduino.h>
static uint32_t now_us() { return micros(); }
#if __has_include(<esp_heap_caps.h>)
#include <esp_heap_caps.h>
static uint32_t heap_largest() { return heap_caps_get_largest_free_block(MALLOC_CAP_8BIT); }
#else
static uint32_t heap_largest() { return 1u << 30; }
#endif
#else
uint32_t ulog_test_heap_largest();             // the native test sets the heap it pretends to have
static uint32_t heap_largest() { return ulog_test_heap_largest(); }
uint32_t ulog_test_now_us();
static uint32_t now_us() { return ulog_test_now_us(); }
#endif

// ---------------------------------------------------------------- LZ4 block format

static uint32_t rd32(const uint8_t *p) {
  uint32_t v;
  memcpy(&v, p, 4);
  return v;
}

static int put_len(uint8_t *dst, int op, int l) {   // the length bytes after a token nibble of 15
  while (l >= 255) {
    dst[op++] = 255;
    l -= 255;
  }
  dst[op++] = (uint8_t)l;
  return op;
}

int ulog_lz4(const uint8_t *src, int n, uint8_t *dst, int cap) {
  static const int MINMATCH = 4, LASTLITERALS = 5, MFLIMIT = 12, HASH_BITS = 12;
  static uint16_t htab[1 << HASH_BITS];              // position + 1 in the block (blocks are < 64 k)
  memset(htab, 0, sizeof(htab));
  int ip = 0, anchor = 0, op = 0;
  if (n > MFLIMIT) {
    const int limit = n - MFLIMIT;                   // a match starts before this, ends LASTLITERALS before the end
    while (ip < limit) {
      uint32_t seq = rd32(src + ip);
      uint32_t h = (seq * 2654435761u) >> (32 - HASH_BITS);
      int ref = (int)htab[h] - 1;
      htab[h] = (uint16_t)(ip + 1);
      if (ref < 0 || rd32(src + ref) != seq) {
        ip++;
        continue;
      }
      int mlen = MINMATCH;
      int maxm = n - LASTLITERALS - ip;
      while (mlen < maxm && src[ref + mlen] == src[ip + mlen]) mlen++;
      if (ip + 1 < limit) {                          // lazy: a longer match one byte on wins
        uint32_t s2 = rd32(src + ip + 1);
        uint32_t h2 = (s2 * 2654435761u) >> (32 - HASH_BITS);
        int r2 = (int)htab[h2] - 1;
        if (r2 >= 0 && rd32(src + r2) == s2) {
          int m2 = MINMATCH;
          const int max2 = n - LASTLITERALS - ip - 1;
          while (m2 < max2 && src[r2 + m2] == src[ip + 1 + m2]) m2++;
          if (m2 > mlen + 1) {
            htab[h2] = (uint16_t)(ip + 2);
            ip++;
            ref = r2;
            mlen = m2;
            maxm = max2;
          }
        }
      }
      const int lit = ip - anchor;
      if (op + 1 + lit / 255 + 1 + lit + 2 + mlen / 255 + 1 > cap) return -1;
      const int tok = op++;
      if (lit >= 15) op = put_len(dst, op, lit - 15);
      memcpy(dst + op, src + anchor, lit);
      op += lit;
      const int off = ip - ref;
      dst[op++] = (uint8_t)off;
      dst[op++] = (uint8_t)(off >> 8);
      const int m = mlen - MINMATCH;
      if (m >= 15) op = put_len(dst, op, m - 15);
      dst[tok] = (uint8_t)((lit >= 15 ? 15 : lit) << 4 | (m >= 15 ? 15 : m));
      ip += mlen;
      anchor = ip;
    }
  }
  const int lit = n - anchor;
  if (op + 1 + lit / 255 + 1 + lit > cap) return -1;
  const int tok = op++;
  if (lit >= 15) op = put_len(dst, op, lit - 15);
  memcpy(dst + op, src + anchor, lit);
  op += lit;
  dst[tok] = (uint8_t)((lit >= 15 ? 15 : lit) << 4);
  return op;
}

// ---------------------------------------------------------------- the log

static uint8_t payload[ULOG_RAW_BLOCK];           // the bytes of the records of the block being filled
static int pay_len = 0;
static uint8_t recs[ULOG_REC_BLOCK];              // their headers
static int rec_bytes = 0;
static uint32_t block_t0 = 0, last_t = 0;
static int rec_len_at = -1;                       // the last record: where its length is in recs, -1 none
static int rec_tag = 0, rec_len = 0;
static uint32_t rec_end_t = 0;
static uint8_t *store = NULL;                   // head and ring, from the heap
static uint8_t *head = NULL, *ring = NULL;
static uint32_t head_size = 0, ring_size = 0;
static uint32_t reserve = ULOG_RESERVE;
static int head_used = 0;
static bool head_full = false;
static uint32_t ring_start = 0, ring_len = 0;
static uint8_t packed[ULOG_HDR + ULOG_REC_BLOCK + ULOG_RAW_BLOCK];
static uint32_t seq = 0;
static UlogStats st;

void ulog_set_reserve(uint32_t bytes) { reserve = bytes; }

void ulog_release() {
  free(store);
  store = head = ring = NULL;
  head_size = ring_size = 0;
  ulog_clear();
}

// the smallest store: the ring must hold a block of the worst case (then the store is all ring)
static const uint32_t ULOG_MIN_STORE = (ULOG_WORST_BLOCK + 1023) & ~1023u;

static void storage_get() {
  if (store) return;
  uint32_t size = ULOG_HEAD_SIZE + ULOG_RING_SIZE;
  uint32_t largest = heap_largest();
  uint32_t can = largest > reserve ? largest - reserve : 0;
  if (size > can) size = can & ~1023u;
  if (size < ULOG_MIN_STORE) return;           // no log this time; the blocks are counted as dropped
  store = (uint8_t *)malloc(size);
  if (!store) return;
  head_size = size * 3 / 14;
  if (size - head_size < ULOG_WORST_BLOCK) head_size = size - ULOG_WORST_BLOCK;
  ring_size = size - head_size;
  head = store;
  ring = store + head_size;
}

void ulog_clear() {
  pay_len = rec_bytes = 0;
  rec_len_at = -1;
  head_used = 0;
  head_full = false;
  ring_start = ring_len = 0;
  seq = 0;
  memset(&st, 0, sizeof(st));
}

static uint32_t ring_get16(uint32_t at) {
  return ring[at % ring_size] | (uint32_t)ring[(at + 1) % ring_size] << 8;
}

static void put16(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

// one part of a block, compressed when that makes it smaller; returns its stored size, sets *raw_flag
static int pack(const uint8_t *src, int n, uint8_t *dst, int cap, bool *raw_flag) {
  int c = n ? ulog_lz4(src, n, dst, cap) : 0;
  *raw_flag = c < 0 || c >= n;
  if (*raw_flag) {
    memcpy(dst, src, n);
    c = n;
  }
  return c;
}

void ulog_flush() {
  if (!rec_bytes) return;
  bool recs_raw, pay_raw;
  const int rc = pack(recs, rec_bytes, packed + ULOG_HDR, ULOG_REC_BLOCK, &recs_raw);
  const int pc = pack(payload, pay_len, packed + ULOG_HDR + rc, ULOG_RAW_BLOCK, &pay_raw);
  uint8_t *h = packed;
  h[0] = 'U';
  h[1] = 'L';
  h[2] = (pay_raw ? 1 : 0) | (recs_raw ? 2 : 0);
  h[3] = 2;
  put16(h + 4, pay_len);
  put16(h + 6, pc);
  memcpy(h + 8, &seq, 4);
  memcpy(h + 12, &block_t0, 4);
  put16(h + 16, rec_bytes);
  put16(h + 18, rc);
  const int size = ULOG_HDR + rc + pc;
  seq++;
  st.blocks++;
  st.raw += pay_len + rec_bytes;
  pay_len = rec_bytes = 0;
  rec_len_at = -1;
  storage_get();                                  // normally taken at ulog_start already
  if (!store) {
    st.dropped++;
    return;
  }
  if (!head_full && head_used + size <= (int)head_size) {
    memcpy(head + head_used, packed, size);
    head_used += size;
    return;
  }
  head_full = true;                               // from here on the ring: the first part of the session stays
  while (ring_size - ring_len < (uint32_t)size) {
    const uint32_t old = ULOG_HDR + ring_get16(ring_start + 6) + ring_get16(ring_start + 18);
    ring_start = (ring_start + old) % ring_size;
    ring_len -= old;
    st.dropped++;
  }
  uint32_t at = (ring_start + ring_len) % ring_size;
  const int first = size < (int)(ring_size - at) ? size : (int)(ring_size - at);
  memcpy(ring + at, packed, first);
  memcpy(ring, packed + first, size - first);
  ring_len += size;
}

static int put_varint(uint8_t *p, uint32_t v) {
  int n = 0;
  while (v >= 0x80) {
    p[n++] = (uint8_t)(v | 0x80);
    v >>= 7;
  }
  p[n++] = (uint8_t)v;
  return n;
}

static void put_len2(int at, int len) {          // a varint of always 2 bytes: it can grow in place
  recs[at] = (uint8_t)(0x80 | (len & 0x7F));
  recs[at + 1] = (uint8_t)(len >> 7);
}

void ulog_data(int tag, const uint8_t *d, int n) {
  if (n <= 0 && tag != ULOG_EVENT) return;
  if (tag == ULOG_TX) st.tx += n;
  if (tag == ULOG_RX) st.rx += n;
  bool first = true;                              // an event is a record even when its text is empty
  while (n > 0 || first) {
    first = false;
    const uint32_t t = now_us();
    if (!(rec_len_at >= 0 && tag == rec_tag && tag != ULOG_EVENT && t - rec_end_t < ULOG_MERGE_US &&
          pay_len < ULOG_RAW_BLOCK)) {
      // a new record (more of the same direction right after goes onto the last one)
      if (pay_len >= ULOG_RAW_BLOCK || ULOG_REC_BLOCK - rec_bytes < 1 + 5 + 2) ulog_flush();
      if (!rec_bytes) block_t0 = last_t = t;
      recs[rec_bytes] = (uint8_t)tag;
      rec_len_at = rec_bytes + 1 + put_varint(recs + rec_bytes + 1, t - last_t);
      rec_bytes = rec_len_at + 2;
      rec_tag = tag;
      rec_len = 0;
      last_t = t;
    }
    int chunk = ULOG_RAW_BLOCK - pay_len;
    if (chunk > n) chunk = n;
    memcpy(payload + pay_len, d, chunk);
    pay_len += chunk;
    rec_len += chunk;
    put_len2(rec_len_at, rec_len);
    rec_end_t = t;
    d += chunk;
    n -= chunk;
  }
}

static void vevent(const char *fmt, va_list ap) {
  char text[160];
  int n = vsnprintf(text, sizeof(text), fmt, ap);
  if (n < 0) return;
  if (n >= (int)sizeof(text)) n = sizeof(text) - 1;
  ulog_data(ULOG_EVENT, (const uint8_t *)text, n);
}

void ulog_event(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vevent(fmt, ap);
  va_end(ap);
}

void ulog_start(const char *fmt, ...) {
  ulog_release();                                 // the old log goes anyway; the store is sized to the heap of now
  storage_get();
  va_list ap;
  va_start(ap, fmt);
  vevent(fmt, ap);
  va_end(ap);
}

uint32_t ulog_export_len() { return head_used + ring_len; }

void ulog_export(uint32_t off, uint8_t *dst, int n) {
  for (int i = 0; i < n; i++, off++) {
    if (off < (uint32_t)head_used) dst[i] = head[off];
    else if (off - head_used < ring_len) dst[i] = ring[(ring_start + off - head_used) % ring_size];
    else dst[i] = 0xFF;
  }
}

UlogStats ulog_stats() {
  UlogStats s = st;
  s.stored = ulog_export_len();
  s.capacity = head_size + ring_size;
  return s;
}
