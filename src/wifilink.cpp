#include "wifilink.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <errno.h>
#include <lwip/sockets.h>
#include "uartlog.h"

// ---------------------------------------------------------------- settings (NVS)

enum { CFG_OFF = 0, CFG_AP = 1, CFG_STA = 2 };

struct WifiCfg {
  int mode = CFG_OFF;
  String ssid, pass, key, name = "c3-flasher";
};
static WifiCfg cfg;

static void cfg_load() {
  Preferences p;
  if (!p.begin("wifi", true)) return;        // nothing stored yet
  cfg.mode = p.getInt("mode", CFG_OFF);
  cfg.ssid = p.getString("ssid", "");
  cfg.pass = p.getString("pass", "");
  cfg.key = p.getString("key", "");
  cfg.name = p.getString("name", "c3-flasher");
  p.end();
}

static void cfg_save() {
  Preferences p;
  if (!p.begin("wifi", false)) return;
  p.putInt("mode", cfg.mode);
  p.putString("ssid", cfg.ssid);
  p.putString("pass", cfg.pass);
  p.putString("key", cfg.key);
  p.putString("name", cfg.name);
  p.end();
}

// ---------------------------------------------------------------- the client as a Stream

static const uint32_t WRITE_STALL_MS = 8000;   // a client that takes nothing for this long is dropped
static const uint32_t READ_TIMEOUT_MS = 3000;  // readBytes: Wi-Fi can pause for a second (retransmissions)

static bool would_block() { return errno == EAGAIN || errno == EWOULDBLOCK; }

// The admitted client. The socket is driven directly (non-blocking calls with time limits of our own): the
// NetworkClient's write waits up to ten seconds per call, and its connected() does not notice a closed connection.
class TcpHost : public Stream {
 public:
  NetworkClient conn;          // owns the socket
  int fd = -1;
  bool by_grant = false;       // admitted without the key, as the bridge owner's address
  uint8_t in[512];
  int in_pos = 0, in_n = 0;
  uint8_t out[1436];
  int out_n = 0;

  TcpHost() { setTimeout(READ_TIMEOUT_MS); }

  bool open() const { return fd >= 0; }

  void close() {
    conn.stop();
    fd = -1;
    in_pos = in_n = out_n = 0;
    by_grant = false;
  }

  // takes over a checked connection; bytes already read from it (the probe) are read first
  void take(NetworkClient &c, const uint8_t *first, int n) {
    close();
    conn = c;
    fd = conn.fd();
    memcpy(in, first, n);
    in_n = n;
    // a client that vanished without closing (a laptop switched off) is noticed within about half a minute
    int on = 1, idle = 15, intvl = 5, cnt = 3;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
  }

  // reads what the socket has; notices the end of the connection
  void fill() {
    if (fd < 0) return;
    if (in_pos == in_n) in_pos = in_n = 0;
    if (in_n == (int)sizeof(in)) return;
    int r = recv(fd, in + in_n, sizeof(in) - in_n, MSG_DONTWAIT);
    if (r > 0) in_n += r;
    else if (r == 0 || !would_block()) close_keeping_input();
  }

  // the peer closed: what it sent before is still read
  void close_keeping_input() {
    int keep_pos = in_pos, keep_n = in_n;
    close();
    in_pos = keep_pos;
    in_n = keep_n;
  }

  bool send_all(const uint8_t *d, int n) {
    uint32_t last = millis();
    while (n > 0 && fd >= 0) {
      int r = send(fd, d, n, MSG_DONTWAIT);
      if (r > 0) {
        d += r;
        n -= r;
        last = millis();
      } else if (r < 0 && would_block() && millis() - last < WRITE_STALL_MS) {
        delay(1);
      } else {
        close();                       // failed, or the client stopped reading
      }
    }
    return n == 0;
  }

  void send_out() {
    if (out_n && fd >= 0) send_all(out, out_n);
    out_n = 0;
  }

  int send_now(const uint8_t *d, int n) {
    if (fd < 0) return -1;
    if (out_n) {
      int r = send(fd, out, out_n, MSG_DONTWAIT);
      if (r > 0) {
        memmove(out, out + r, out_n - r);
        out_n -= r;
      } else if (r < 0 && !would_block()) {
        close();
        return -1;
      }
      if (out_n) return 0;
    }
    int r = send(fd, d, n, MSG_DONTWAIT);
    if (r >= 0) return r;
    if (would_block()) return 0;
    close();
    return -1;
  }

  int available() override {
    send_out();                        // whoever looks for input waits for the host: what it should see goes now
    fill();
    return in_n - in_pos;
  }
  int read() override {
    if (in_pos == in_n) fill();
    return in_pos < in_n ? in[in_pos++] : -1;
  }
  int peek() override {
    if (in_pos == in_n) fill();
    return in_pos < in_n ? in[in_pos] : -1;
  }
  size_t readBytes(char *buf, size_t len) override {
    send_out();
    size_t got = 0;
    uint32_t last = millis();
    while (got < len) {
      if (in_pos == in_n) fill();
      if (in_pos < in_n) {
        int k = min((int)(len - got), in_n - in_pos);
        memcpy(buf + got, in + in_pos, k);
        in_pos += k;
        got += k;
        last = millis();
      } else if (fd < 0 || millis() - last >= getTimeout()) {
        break;
      } else {
        delay(1);
      }
    }
    return got;
  }
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t *d, size_t n) override {
    if (fd < 0 || n == 0) return 0;
    if (out_n + n > sizeof(out)) {
      if (n >= sizeof(out)) {          // a stream frame: fill a segment with the start, then the rest as it is
        size_t k = sizeof(out) - out_n;
        memcpy(out + out_n, d, k);
        out_n += k;
        send_out();
        return fd >= 0 && send_all(d + k, n - k) ? n : 0;
      }
      send_out();
      if (fd < 0) return 0;
    }
    memcpy(out + out_n, d, n);
    out_n += n;
    if (d[n - 1] == '\n') send_out();  // a whole answer line
    return fd >= 0 ? n : 0;
  }
  int availableForWrite() override { return fd >= 0 ? (int)(sizeof(out) - out_n) : 0; }
  void flush() override { send_out(); }
  using Print::write;
};

// ---------------------------------------------------------------- server, admission

static NetworkServer server(WIFI_PORT);
static TcpHost tcp;
static uint32_t client_seq = 0;
static bool running = false;

// a connection that has not been admitted yet: its first bytes, one at a time up to the end of the first line
static NetworkClient pend;
static uint32_t pend_ms = 0;                // since when, 0: none
static uint8_t probe[128];
static int probe_n = 0;

static bool grant_on = false;               // the bridge belongs to a TCP client at grant_ip
static IPAddress grant_ip;
// no key needed (or the bridge owner's address): bytes that are not "AUTH " admit at once, silence after PROBE_MS.
// The host sends its AUTH line right after connecting; a second leaves room for a lost first segment, so that a late
// AUTH line is not taken for data (and passed to a bridged target).
static const uint32_t PROBE_MS = 1000;
static const uint32_t AUTH_MS = 5000;       // a key is needed: the AUTH line must be complete by then

static void pend_drop(const char *why) {
  if (why) pend.write((const uint8_t *)why, strlen(why));
  pend.stop();
  pend_ms = 0;
  probe_n = 0;
}

static void admit(bool by_grant, bool give_back) {
  tcp.take(pend, probe, give_back ? probe_n : 0);   // the client before is dropped here
  tcp.by_grant = by_grant;
  client_seq++;
  pend = NetworkClient();
  pend_ms = 0;
  probe_n = 0;
}

static bool key_matches(const uint8_t *k, int n) {
  if (n != (int)cfg.key.length()) return false;
  uint8_t diff = 0;
  for (int i = 0; i < n; i++) diff |= k[i] ^ (uint8_t)cfg.key[i];
  return diff == 0;
}

static void probe_step() {
  const int fd = pend.fd();
  while (probe_n < (int)sizeof(probe) && (probe_n == 0 || probe[probe_n - 1] != '\n')) {
    int r = recv(fd, probe + probe_n, 1, MSG_DONTWAIT);
    if (r == 1) {
      probe_n++;
      continue;
    }
    if (r == 0 || !would_block()) return pend_drop(NULL);   // gone before it was admitted
    break;
  }
  const uint32_t age = millis() - pend_ms;
  const bool granted = grant_on && pend.remoteIP() == grant_ip;
  const bool need_key = !cfg.key.isEmpty() && !granted;
  static const char PREFIX[] = "AUTH ";
  const bool auth_line = !memcmp(probe, PREFIX, min(probe_n, 5));   // so far; true while nothing came
  if (!auth_line) {
    if (need_key) return pend_drop("ERR auth: send AUTH <key> first\n");
    return admit(granted && !cfg.key.isEmpty(), true);   // a command or a tool's bytes: passed on as they came
  }
  if (probe_n == 0) {
    if (!need_key && age >= PROBE_MS) admit(granted && !cfg.key.isEmpty(), false);
    else if (age >= AUTH_MS) pend_drop("ERR auth: send AUTH <key> first\n");
    return;
  }
  if (probe[probe_n - 1] == '\n') {
    int n = probe_n - 1;
    while (n > 5 && (probe[n - 1] == '\r' || probe[n - 1] == ' ')) n--;
    if (n < 5) return pend_drop("ERR auth: send AUTH <key> first\n");
    // no key set: any key is taken (the host sends its key whether one is set or not)
    if (!cfg.key.isEmpty() && !key_matches(probe + 5, n - 5)) {
      memset(probe, 0, sizeof(probe));
      return pend_drop("ERR auth\n");
    }
    memset(probe, 0, sizeof(probe));
    admit(false, false);
    tcp.print("OK auth\n");
    return;
  }
  if (probe_n >= (int)sizeof(probe) || age >= AUTH_MS) {
    memset(probe, 0, sizeof(probe));
    pend_drop("ERR auth: line too long or too slow\n");
  }
}

static void wifi_stop() {
  tcp.close();
  if (pend_ms) pend_drop(NULL);
  if (!running) return;
  server.end();
  MDNS.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  running = false;
  ulog_set_reserve(ULOG_RESERVE);
}

static bool wifi_start() {
  wifi_stop();
  if (cfg.mode == CFG_OFF || cfg.ssid.isEmpty()) return false;
  // Wi-Fi's buffers come from the heap while it runs: the UART log's store goes back now and the next session takes
  // a smaller one, leaving ULOG_RESERVE_WIFI of the largest free block
  ulog_release();
  ulog_set_reserve(ULOG_RESERVE_WIFI);
  WiFi.setHostname(cfg.name.c_str());
  if (cfg.mode == CFG_AP) {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(cfg.ssid.c_str(), cfg.pass.c_str())) return false;
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                    // modem sleep delays every answer by up to a beacon interval
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfg.ssid.c_str(), cfg.pass.isEmpty() ? NULL : cfg.pass.c_str());
  }
  server.begin();
  server.setNoDelay(true);                   // answers are gathered into lines (TcpHost); no Nagle delay on top
  if (MDNS.begin(cfg.name.c_str())) MDNS.addService("c3flasher", "tcp", WIFI_PORT);
  running = true;
  return true;
}

void wifi_setup() {
  cfg_load();
  if (cfg.mode != CFG_OFF) wifi_start();
}

void wifi_poll() {
  static uint32_t last_ms = 0;
  if (!running || millis() == last_ms) return;   // once a millisecond is enough (the bridge loop spins)
  last_ms = millis();
  NetworkClient fresh = server.accept();
  if (fresh) {
    if (pend_ms) pend_drop(NULL);            // only the newest waits for admission
    pend = fresh;
    pend.setNoDelay(true);
    pend_ms = millis() | 1;
    probe_n = 0;
  }
  if (pend_ms) probe_step();
  if (tcp.open() && tcp.in_pos == tcp.in_n) tcp.fill();   // notices a client that has gone
}

Stream *wifi_client() { return tcp.open() || tcp.in_pos < tcp.in_n ? &tcp : NULL; }

uint32_t wifi_client_seq() { return client_seq; }

bool wifi_alive(Stream *s) { return s == &tcp && tcp.open(); }

int wifi_send_now(const uint8_t *d, int n) { return tcp.send_now(d, n); }

void wifi_bridge_owner(bool on) {
  if (on) {
    grant_on = tcp.open();
    if (grant_on) grant_ip = tcp.conn.remoteIP();
    return;
  }
  grant_on = false;
  if (tcp.by_grant && !cfg.key.isEmpty()) {
    tcp.flush();
    tcp.close();                             // came in without the key for the bridge only
  }
}

// ---------------------------------------------------------------- the WIFI command

static bool from_hex(const char *h, String &out) {
  out = "";
  if (!strcmp(h, "-")) return true;          // empty
  size_t n = strlen(h);
  if (n % 2 || n > 128) return false;
  for (size_t i = 0; i < n; i += 2) {
    char b[3] = {h[i], h[i + 1], 0};
    char *end;
    long v = strtol(b, &end, 16);
    if (*end || v == 0) return false;
    out += (char)v;
  }
  return true;
}

static void wipe(String &s) {
  for (size_t i = 0; i < s.length(); i++) s.setCharAt(i, 0);
  s = "";
}

static void status(Print &out) {
  out.printf("OK wifi mode=%s ssid=", cfg.mode == CFG_AP ? "ap" : cfg.mode == CFG_STA ? "sta" : "off");
  if (cfg.ssid.isEmpty()) out.print("-");
  for (size_t i = 0; i < cfg.ssid.length(); i++) out.printf("%02x", (uint8_t)cfg.ssid[i]);
  out.printf(" pass=%s key=%s name=%s port=%u", cfg.pass.isEmpty() ? "none" : "set",
             cfg.key.isEmpty() ? "none" : "set", cfg.name.c_str(), (unsigned)WIFI_PORT);
  if (running && cfg.mode == CFG_AP) {
    out.printf(" ip=%s stations=%d", WiFi.softAPIP().toString().c_str(), (int)WiFi.softAPgetStationNum());
  } else if (running) {
    if (WiFi.status() == WL_CONNECTED)
      out.printf(" ip=%s rssi=%d", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    else
      out.print(" ip=none state=connecting");
  }
  out.printf(" client=%s heap=%u largest=%u\n", tcp.open() ? tcp.conn.remoteIP().toString().c_str() : "none",
             (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

void wifi_command(int argc, char **argv, Print &out) {
  if (argc < 2) return status(out);
  String a, b;
  if (!strcmp(argv[1], "OFF")) {
    cfg.mode = CFG_OFF;
    cfg_save();
    out.print("OK wifi off\n");
    out.flush();
    delay(50);
    wifi_stop();
  } else if ((!strcmp(argv[1], "AP") || !strcmp(argv[1], "STA")) && argc > 3) {
    const bool ap = !strcmp(argv[1], "AP");
    if (!from_hex(argv[2], a) || a.isEmpty() || a.length() > 32)
      return (void)out.print("ERR ssid: 1..32 bytes in hex\n");
    if (!from_hex(argv[3], b)) return (void)out.print("ERR password: in hex, or - for none\n");
    if ((ap || !b.isEmpty()) && (b.length() < 8 || b.length() > 63)) {
      wipe(b);
      return (void)out.print(ap ? "ERR password: an access point needs 8..63 characters\n"
                                : "ERR password: 8..63 characters, or - for none\n");
    }
    cfg.mode = ap ? CFG_AP : CFG_STA;
    cfg.ssid = a;
    cfg.pass = b;
    wipe(b);
    cfg_save();
    out.printf("OK wifi %s, starting; a TCP client is dropped\n", ap ? "access point" : "station");
    out.flush();
    delay(50);
    wifi_start();
  } else if (!strcmp(argv[1], "KEY") && argc > 2) {
    bool good = from_hex(argv[2], a);
    for (size_t i = 0; i < a.length() && good; i++) good = a[i] > ' ' && a[i] < 0x7f;
    if (!good || a.length() > 64) {
      wipe(a);
      return (void)out.print("ERR key: 1..64 printable characters without spaces, in hex, or - for none\n");
    }
    cfg.key = a;
    out.printf("OK wifi key %s\n", a.isEmpty() ? "removed" : "set");
    wipe(a);
    cfg_save();
  } else if (!strcmp(argv[1], "NAME") && argc > 2) {
    String n = argv[2];
    bool good = n.length() >= 1 && n.length() <= 31;
    for (size_t i = 0; i < n.length() && good; i++) good = isalnum((unsigned char)n[i]) || n[i] == '-';
    if (!good) return (void)out.print("ERR name: 1..31 letters, digits, -\n");
    cfg.name = n;
    cfg_save();
    out.print("OK wifi name set; used from the next start of Wi-Fi\n");
  } else {
    out.print("ERR WIFI [OFF | AP <ssid hex> <password hex> | STA <ssid hex> <password hex | -> | KEY <hex | -> | "
              "NAME <name>]\n");
  }
}
