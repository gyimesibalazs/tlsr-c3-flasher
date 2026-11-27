// Runs src/sniffring.h (the live output of the two-line UART logger) natively for tests/test_sniff.py:
//   sniff_sim <ring bytes> <host bytes per step, 0 = all> <pieces.txt> <stream.bin>
// pieces.txt: one piece per line, "<t_us> <tag> <hex>", as the capture loop would push them (tag 3: event text).
// After each piece the "host" takes up to the given number of bytes, the way main.cpp drains the ring (its two parts,
// partial frames allowed); at the end it takes everything. stream.bin: the bytes the host received. Prints
// "dropped=<bytes of data dropped in all>".
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "../../src/sniffring.h"

static SniffRing ring;
static FILE *out;

static void drain(long budget) {               // budget < 0: everything
  for (int part = 0; part < 2; part++) {
    const uint8_t *p;
    int k = ring.peek(&p);
    if (!k) break;
    if (budget >= 0 && k > budget) k = (int)budget;
    if (!k) break;
    fwrite(p, 1, k, out);
    ring.consume(k);
    if (budget >= 0) budget -= k;
  }
  ring.report();
}

int main(int argc, char **argv) {
  if (argc < 5) return 2;
  std::vector<uint8_t> mem(atoi(argv[1]));
  const long per_step = atol(argv[2]);
  ring.init(mem.data(), mem.size());
  FILE *in = fopen(argv[3], "r");
  out = fopen(argv[4], "wb");
  if (!in || !out) return 2;
  char line[20000];
  while (fgets(line, sizeof(line), in)) {
    unsigned t;
    int tag;
    char hex[sizeof(line)] = "";
    if (sscanf(line, "%u %d %19999s", &t, &tag, hex) < 2) continue;
    std::vector<uint8_t> d(strlen(hex) / 2);
    for (size_t i = 0; i < d.size(); i++) sscanf(hex + 2 * i, "%2hhx", &d[i]);
    ring.push(tag, t, d.data(), (int)d.size());
    drain(per_step ? per_step : -1);
  }
  drain(-1);                                   // the report goes in once there is room, then out
  drain(-1);
  fclose(out);
  printf("dropped=%u left=%u\n", (unsigned)ring.dropped_total, (unsigned)ring.len);
  return 0;
}
