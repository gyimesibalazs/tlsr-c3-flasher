#include "spiflash.h"

#include <Arduino.h>
#include <SPI.h>

#include "config.h"

static SPIClass spi(FSPI);
static bool on = false;
static bool addr4 = false;
static uint32_t hz = 4000000;

bool spif_active() { return on; }
uint32_t spif_khz() { return hz / 1000; }
void spif_addr4(bool v) { addr4 = v; }
bool spif_addr4_on() { return addr4; }

bool spif_begin(uint32_t khz) {
#if ARDUINO_USB_CDC_ON_BOOT
  hz = khz * 1000;
  if (on) return true;
  pinMode(PIN_SWS_RX, INPUT);                // the direct side of the SWS wire only listens
  pinMode(PIN_SPI_CS, OUTPUT);
  digitalWrite(PIN_SPI_CS, HIGH);
  spi.begin(PIN_SPI_CLK, PIN_SPI_DO, PIN_SPI_DI, -1);
  on = true;
  addr4 = false;
  return true;
#else
  (void)khz;
  return false;                              // GPIO 21 is the link to the computer on these boards
#endif
}

void spif_end() {
  if (!on) return;
  spi.end();
  on = false;
  pinMode(PIN_SPI_CLK, INPUT);
  pinMode(PIN_SPI_DI, INPUT);
  pinMode(PIN_SPI_DO, INPUT);
  // the CS wire is the reset wire: back to open drain, released
  gpio_set_level((gpio_num_t)PIN_SPI_CS, 1);
  pinMode(PIN_SPI_CS, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(PIN_SPI_CS, HIGH);
}

void spif_xfer(const uint8_t *out, int nout, uint8_t *in, int nin) {
  spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SPI_CS, LOW);
  if (nout > 0) spi.writeBytes(out, nout);
  if (nin > 0) {
    memset(in, 0xff, nin);
    spi.transferBytes(in, in, nin);
  }
  digitalWrite(PIN_SPI_CS, HIGH);
  spi.endTransaction();
}

static int put_addr(uint8_t *b, uint8_t op3, uint8_t op4, uint32_t addr) {
  int i = 0;
  b[i++] = addr4 ? op4 : op3;
  if (addr4) b[i++] = addr >> 24;
  b[i++] = addr >> 16;
  b[i++] = addr >> 8;
  b[i++] = addr;
  return i;
}

bool spif_read(uint32_t addr, uint8_t *out, int n) {
  uint8_t cmd[5];
  int len = put_addr(cmd, 0x03, 0x13, addr);
  spif_xfer(cmd, len, out, n);
  return true;
}

uint8_t spif_status() {
  uint8_t c = 0x05, s = 0;
  spif_xfer(&c, 1, &s, 1);
  return s;
}

static int wait_ready(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (spif_status() & 1) {
    if (millis() - t0 > timeout_ms) return SPIF_ERR_TIMEOUT;
    delay(timeout_ms > 100 ? 2 : 0);
  }
  return SPIF_OK;
}

static int write_enable() {
  uint8_t c = 0x06;
  spif_xfer(&c, 1, NULL, 0);
  return (spif_status() & 2) ? SPIF_OK : SPIF_ERR_WEL;
}

int spif_erase(uint32_t addr, uint8_t op, uint32_t timeout_ms) {
  int r = write_enable();
  if (r) return r;
  uint8_t cmd[5];
  uint8_t op4 = op == 0x20 ? 0x21 : op == 0x52 ? 0x5C : 0xDC;
  int len = put_addr(cmd, op, op4, addr);
  spif_xfer(cmd, len, NULL, 0);
  return wait_ready(timeout_ms);
}

int spif_erase_chip(uint32_t timeout_ms) {
  int r = write_enable();
  if (r) return r;
  uint8_t c = 0xC7;
  spif_xfer(&c, 1, NULL, 0);
  return wait_ready(timeout_ms);
}

int spif_program(uint32_t addr, const uint8_t *data, int n) {
  int r = write_enable();
  if (r) return r;
  static uint8_t buf[5 + 256];
  int len = put_addr(buf, 0x02, 0x12, addr);
  memcpy(buf + len, data, n);
  spif_xfer(buf, len + n, NULL, 0);
  return wait_ready(50);
}
