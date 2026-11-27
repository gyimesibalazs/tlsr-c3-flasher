// UART log: what went over the target's UART in the last UART session (bridge, Beken), kept compressed in RAM.
//
// A block holds up to ULOG_RAW_BLOCK bytes of data with the headers of its records kept apart (ULOG_REC_BLOCK): the
// data stays contiguous, so a console's text compresses as text, and the similar headers compress among
// themselves. A full block is compressed (LZ4 block format, the two parts separately) and stored. The first
// part of the store (3/14 of it) holds the first blocks of a session for good (the start: catch, link, speed change),
// the rest is a ring that drops its oldest blocks (the end of the session). The ring always holds a block of the
// worst case (ULOG_WORST_BLOCK): a small store (Wi-Fi on) has a smaller first part, or none, and is at least that. The store is taken from the heap when a
// session starts, up to ULOG_HEAD_SIZE + ULOG_RING_SIZE and as much as the heap gives after keeping a reserve (more
// with Wi-Fi on); ulog_release gives it back (before Wi-Fi starts). Each session takes it anew, so its size follows
// the heap of the moment.
// The host reads the stored blocks with XR L (ulog_export) after LOG (ulog_flush) and decodes them.
//
// Block: header of 20 bytes, then the record headers, then the data (each LZ4, or raw when that is not smaller):
//   'U' 'L' flags(bit 0: data raw, bit 1: headers raw) version(2) data_len:u16 data_stored:u16 seq:u32 t0_us:u32
//   headers_len:u16 headers_stored:u16
// Record header: tag(1 = host -> target, 2 = target -> host, 3 = event text) dt_us:varint len:varint(always 2
//   bytes); its bytes are the next len bytes of the data. dt_us is the time since the record before in the block
//   (the first: since t0_us). Data in the same direction within ULOG_MERGE_US of the record's last piece is added
//   to that record (a console's text arrives in pieces of a few bytes): the time is that of its first byte.
#pragma once
#include <stddef.h>
#include <stdint.h>

static const int ULOG_RAW_BLOCK = 16384;
// 112 k at most: with the block buffers the log takes about 150 k of the C3's RAM; the heap must keep room for the
// UART driver (12 k in the Beken mode) and Wi-Fi's buffers. LOG reports the store's size and the free heap.
static const int ULOG_HEAD_SIZE = 24 * 1024;
static const int ULOG_RING_SIZE = 88 * 1024;
static const int ULOG_REC_BLOCK = 4096;
// heap the store leaves free in its block: normally, and while Wi-Fi runs (its buffers and lwIP's come from the heap).
// With Wi-Fi up its static buffers and tasks are already taken when a session sizes its store; what it still takes
// later is short-lived: frames in flight (one TCP client with a window and a send buffer of about 5.7 k each, Wi-Fi
// buffers of 1.6 k) and bursts of received frames. 40 k of the largest block, plus every smaller free block, leaves
// room for 25 Wi-Fi buffers at once. LOG reports the lowest free heap seen (heap_min) to check it.
static const uint32_t ULOG_RESERVE = 24 * 1024;
static const uint32_t ULOG_RESERVE_WIFI = 40 * 1024;
static const int ULOG_HDR = 20;
static const uint32_t ULOG_WORST_BLOCK = ULOG_HDR + ULOG_REC_BLOCK + ULOG_RAW_BLOCK;   // stored raw
static const uint32_t ULOG_MERGE_US = 2000;

enum { ULOG_TX = 1, ULOG_RX = 2, ULOG_EVENT = 3 };

struct UlogStats {
  uint32_t blocks, stored, raw, dropped, tx, rx, capacity;
};

void ulog_start(const char *fmt, ...);          // a new session: the old log is cleared, the first event
void ulog_data(int tag, const uint8_t *d, int n);
inline void ulog_tx(const uint8_t *d, int n) { ulog_data(ULOG_TX, d, n); }
inline void ulog_rx(const uint8_t *d, int n) { ulog_data(ULOG_RX, d, n); }
void ulog_event(const char *fmt, ...);
void ulog_flush();                              // compress and store the block being filled
void ulog_clear();
uint32_t ulog_export_len();
void ulog_export(uint32_t off, uint8_t *dst, int n);   // the stored blocks, oldest first
UlogStats ulog_stats();
void ulog_release();                            // gives the store back to the heap (the log is lost)
void ulog_set_reserve(uint32_t bytes);          // heap left free when the store is taken

// LZ4 block compression; -1 when it does not fit in cap
int ulog_lz4(const uint8_t *src, int n, uint8_t *dst, int cap);
