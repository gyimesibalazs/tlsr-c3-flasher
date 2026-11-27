// Speed following of the UART bridge for loaders that take a text line to change their speed and switch at the end
// of that line ('\r' or '\n') without an answer at the old speed:
//   "ln":   the LN882H's RAM loader (ramcode), "baudrate <digits>"
//   "ucfg": the Realtek AmebaZ2 ROM's download console, "ucfg <digits> <flash settings>"
// LineBaudScan reads the host's bytes for that line.
//
// The host also sends binary XMODEM / YMODEM packets through the bridge (loaders, flash images). A line counts only
// when it starts right after a CR or LF and holds printable characters only; any other byte (a packet header, binary
// data) spoils it up to the next CR or LF. So a switch inside a transfer needs the bytes
// [CR|LF] key digits [CR|LF] inside one packet; the host refuses to send data that holds them.
#pragma once
#include <stdint.h>
#include <string.h>

struct LineBaudScan {
  const char *key = "baudrate ";
  bool rest_ok = false;        // more after the digits, separated by a space ("ucfg 1500000 0 1")
  char line[40];
  int pos = 0;
  bool bad = false;            // the current line cannot be the speed command any more

  LineBaudScan() {}
  LineBaudScan(const char *k, bool rest) : key(k), rest_ok(rest) {}

  // feeds one byte from the host; returns the speed when b ends the speed command line, else 0
  uint32_t feed(uint8_t b) {
    if (b == '\r' || b == '\n') {
      uint32_t baud = 0;
      const int k = (int)strlen(key);
      if (!bad && pos > k && !memcmp(line, key, k)) {
        int i = k;
        for (; i < pos && line[i] >= '0' && line[i] <= '9'; i++) {
          if (baud > 100000000u) break;
          baud = baud * 10 + (line[i] - '0');
        }
        if (i == k || (i < pos && !(rest_ok && line[i] == ' ' && baud <= 100000000u))) baud = 0;
      }
      pos = 0;
      bad = false;
      return baud >= 1200 && baud <= 5000000 ? baud : 0;
    }
    if (bad) return 0;
    if (b < 0x20 || b > 0x7e || pos >= (int)sizeof(line)) {
      bad = true;
      return 0;
    }
    line[pos++] = (char)b;
    return 0;
  }
};
