"""Wi-Fi link (firmware 0.29), host side: tlsr_c3.py over TCP against a local server thread that models the
flasher's command side (V, AUTH with a key, WIFI, one XR stream, BRIDGE with the escape and the owner-address rule).
The firmware's Wi-Fi part itself (src/wifilink.cpp) runs only on the C3 and is not simulated here."""
import io, os, socket, subprocess, sys, threading, time, types, zlib, contextlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import tlsr_c3 as T

PASSWORD = 'Sw0rdf1sh-pass'
KEY = 'k3y-0f-the-day'
os.environ.pop('C3_FLASHER_KEY', None)
os.environ.pop('C3_FLASHER_WIFI_PASSWORD', None)


class FakeFlasherServer:
    """One client at a time, like the firmware: the first line is looked at for AUTH, then command lines (or, while
    the bridge is on, raw bytes echoed back as a target would, until "~~~X" after a pause of a second)."""
    def __init__(self, key=''):
        self.key = key
        self.lsock = socket.socket()
        self.lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.lsock.bind(('127.0.0.1', 0))
        self.lsock.listen(4)
        self.port = self.lsock.getsockname()[1]
        self.url = 'socket://127.0.0.1:%d' % self.port
        self.lines = []          # every command line taken (WIFI lines too: the test looks at their encoding)
        self.auth = []           # per connection: 'key', 'any key' (none set), 'refused', 'none', 'owner'
        self.bridge = False
        self.owner_ip = None
        self.bridged_bytes = b''
        self.drop_stream_after = None
        self.mem = bytes(range(256)) * 512
        self.wifi = {'mode': 'off', 'ssid': '', 'pass': 'none'}
        self.stop = False
        threading.Thread(target=self.serve, daemon=True).start()

    def serve(self):
        while not self.stop:
            try:
                c, addr = self.lsock.accept()
            except OSError:
                return
            try:
                self.client(c, addr[0])
            except (OSError, ConnectionError):
                pass
            finally:
                c.close()

    def first_line(self, c, ip):
        """The admission step: returns the bytes to pass on, or None when the connection was refused."""
        c.settimeout(0.5)
        data = b''
        try:
            while not data.startswith(b'AUTH ') or b'\n' not in data:
                chunk = c.recv(1)
                if not chunk:
                    return None
                data += chunk
                if not b'AUTH '.startswith(data[:5]):
                    break
        except socket.timeout:
            pass
        owner = self.bridge and ip == self.owner_ip
        if data.startswith(b'AUTH ') and data.endswith(b'\n'):
            if self.key and data[5:].strip().decode() != self.key:
                self.auth.append('refused')
                c.sendall(b'ERR auth\n')
                return None
            self.auth.append('key' if self.key else 'any key')
            c.sendall(b'OK auth\n')
            return b''
        if self.key and not owner:
            self.auth.append('refused')
            c.sendall(b'ERR auth: send AUTH <key> first\n')
            return None
        self.auth.append('owner' if owner else 'none')
        return data

    def client(self, c, ip):
        pending = self.first_line(c, ip)
        if pending is None:
            return
        c.settimeout(0.05)
        buf, last = pending, time.time()
        while True:
            if self.bridge:
                if buf:
                    if time.time() - last >= 1.0 and buf.startswith(b'~~~X'):
                        self.bridge = False
                        c.sendall(b'OK bridge closed\n')
                        buf = buf[4:]
                        continue
                    self.bridged_bytes += buf
                    c.sendall(buf)                        # the target echoes
                    buf, last = b'', time.time()
            else:
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    if not self.command(c, line.decode().strip()):
                        return
                    if self.bridge:
                        last = time.time()
                        break
            try:
                chunk = c.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return
            buf += chunk

    def command(self, c, line):
        """False: the connection ends (as the firmware drops it when Wi-Fi restarts)."""
        if not line:
            return True
        self.lines.append(line)
        a = line.split()
        if a[0] == 'V':
            c.sendall(b'OK tlsr-c3-flasher 0.29 mode=rmt baud=921600 rate=1000000 burst=64 tx=4 rx=5 rst=6 '
                      b'pwr=7+2 more\n')
        elif a[0] == 'WIFI' and len(a) == 1:
            w = self.wifi
            ssid = w['ssid'].encode().hex() or '-'
            c.sendall(('OK wifi mode=%s ssid=%s pass=%s key=%s name=c3-flasher port=7171%s client=%s heap=91234 '
                       'largest=61440\n' % (w['mode'], ssid, w['pass'], 'set' if self.key else 'none',
                                            ' ip=192.168.4.1 stations=1' if w['mode'] == 'ap' else
                                            ' ip=10.0.0.7 rssi=-51' if w['mode'] == 'sta' else '',
                                            '127.0.0.1')).encode())
        elif a[0] == 'WIFI' and a[1] in ('AP', 'STA'):
            pw = '' if a[3] == '-' else bytes.fromhex(a[3]).decode()
            self.wifi = {'mode': a[1].lower(), 'ssid': bytes.fromhex(a[2]).decode(), 'pass': 'set' if pw else 'none'}
            self.wifi_password = pw
            c.sendall(b'OK wifi %s, starting; a TCP client is dropped\n' %
                      (b'access point' if a[1] == 'AP' else b'station'))
            return False
        elif a[0] == 'WIFI' and a[1] == 'KEY':
            self.key = '' if a[2] == '-' else bytes.fromhex(a[2]).decode()
            c.sendall(b'OK wifi key %s\n' % (b'set' if self.key else b'removed'))
        elif a[0] == 'WIFI' and a[1] == 'OFF':
            self.wifi['mode'] = 'off'
            c.sendall(b'OK wifi off\n')
        elif a[0] == 'XR':
            addr, n = int(a[2]), int(a[3])
            c.sendall(b'OK %d\n' % n)
            sent = 0
            while n:
                if self.drop_stream_after is not None and sent >= self.drop_stream_after:
                    return False                          # the connection breaks in the middle of the stream
                k = min(4096, n)
                d = self.mem[addr:addr + k]
                c.sendall(b'D' + k.to_bytes(2, 'little') + d + zlib.crc32(d).to_bytes(4, 'little'))
                addr, n, sent = addr + k, n - k, sent + k
            c.sendall(b'Z\x00\x00')
        elif a[0] == 'BRIDGE':
            self.bridge = True
            self.owner_ip = c.getpeername()[0]
            c.sendall(b'OK bridge %s baud, boot low; leave with a 1 s pause and ~~~X\n' % a[1].encode())
        else:
            c.sendall(b'OK\n')
        return True


def expect_error(fn, *words):
    try:
        fn()
    except (T.FlasherError, T.serial.SerialException) as e:
        for w in words:
            assert w in str(e), (w, str(e))
        return str(e)
    raise AssertionError('no error')


def run_cli(*argv, env=None):
    e = dict(os.environ)
    e.pop('C3_FLASHER_KEY', None)
    e.pop('C3_FLASHER_WIFI_PASSWORD', None)
    e.update(env or {})
    p = subprocess.run([sys.executable, os.path.join(HERE, '..', 'tlsr_c3.py')] + list(argv), env=e,
                       stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
    return p.returncode, p.stdout + p.stderr


def no_secret(text, *secrets):
    for s in secrets:
        assert s not in text and s.encode().hex() not in text, 'a secret is in the output: %r' % text


# --host: name and port
assert T.tcp_url('c3-flasher.local') == 'socket://c3-flasher.local:7171'
assert T.tcp_url('192.168.4.1:8000') == 'socket://192.168.4.1:8000'

# connect without a key: no AUTH is sent, V answers
srv = FakeFlasherServer()
f = T.Flasher(srv.url)
assert f.version.startswith('tlsr-c3-flasher 0.29'), f.version
assert srv.auth == ['none'] and srv.lines == ['V'], (srv.auth, srv.lines)
f.ser.close()

# a key given while the flasher has none: taken out at the start (firmware: any key is taken), never a command
f = T.Flasher(srv.url, key=KEY)
assert srv.auth[-1] == 'any key' and all(KEY not in l for l in srv.lines), srv.lines
f.ser.close()

# with a key: the right one, a wrong one, none
srv = FakeFlasherServer(key=KEY)
f = T.Flasher(srv.url, key=KEY)
assert srv.auth == ['key'] and f.cmd('V').startswith('tlsr-c3-flasher')
f.ser.close()
msg = expect_error(lambda: T.Flasher(srv.url, key='wrong-key'), 'did not take the key')
no_secret(msg, 'wrong-key', KEY)
msg = expect_error(lambda: T.Flasher(srv.url), 'wants its key')
assert srv.auth[-2:] == ['refused', 'refused'], srv.auth
expect_error(lambda: T.Flasher('socket://127.0.0.1:1'), 'cannot reach')

# the CLI: wifi status, then ap / join with spaces in the SSID; the password never in the output, sent in hex
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, 'wifi', env={'C3_FLASHER_KEY': KEY})
assert code == 0 and 'Wi-Fi:   off' in out and 'key set' in out, out
no_secret(out, KEY)
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, '--key', KEY, 'wifi', 'ap', 'My Flasher AP',
                    env={'C3_FLASHER_WIFI_PASSWORD': PASSWORD})
assert code == 0 and 'this connection ends' in out, out
no_secret(out, PASSWORD, KEY)
assert srv.lines[-1] == 'WIFI AP %s %s' % ('My Flasher AP'.encode().hex(), PASSWORD.encode().hex()), srv.lines[-1]
assert srv.wifi == {'mode': 'ap', 'ssid': 'My Flasher AP', 'pass': 'set'} and srv.wifi_password == PASSWORD
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, 'wifi', 'join', 'Home Net 5G',
                    env={'C3_FLASHER_WIFI_PASSWORD': PASSWORD, 'C3_FLASHER_KEY': KEY})
assert code == 0 and srv.lines[-1] == 'WIFI STA %s %s' % ('Home Net 5G'.encode().hex(), PASSWORD.encode().hex())
no_secret(out, PASSWORD, KEY)
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, 'wifi', env={'C3_FLASHER_KEY': KEY})
assert code == 0 and 'station "Home Net 5G", IP 10.0.0.7' in out and 'password set' in out, out
no_secret(out, PASSWORD, KEY)
# an open network: '-' for the password; a short one is refused before anything is sent
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, 'wifi', 'join', 'Cafe',
                    env={'C3_FLASHER_WIFI_PASSWORD': '', 'C3_FLASHER_KEY': KEY})
assert code == 0 and srv.lines[-1] == 'WIFI STA %s -' % b'Cafe'.hex(), (code, out, srv.lines[-1])
n = len(srv.lines)
code, out = run_cli('--host', '127.0.0.1:%d' % srv.port, 'wifi', 'ap', 'X',
                    env={'C3_FLASHER_WIFI_PASSWORD': 'short', 'C3_FLASHER_KEY': KEY})
assert code == 1 and '8..63' in out and len(srv.lines) == n + 1 and srv.lines[-1] == 'V', (out, srv.lines[n:])
no_secret(out, 'short')

# wifi key, typed (getpass, asked twice): sent in hex, never printed; the flasher wants it from then on
srv = FakeFlasherServer()
f = T.Flasher(srv.url)
typed = iter([KEY, KEY])
T.getpass.getpass = lambda prompt='': next(typed)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    T.wifi_main(f, types.SimpleNamespace(port=srv.url, action='key', value=None))
assert srv.lines[-1] == 'WIFI KEY %s' % KEY.encode().hex() and srv.key == KEY, srv.lines
no_secret(buf.getvalue(), KEY)
typed = iter(['one', 'two'])
expect_error(lambda: T.wifi_main(f, types.SimpleNamespace(port=srv.url, action='key', value=None)), 'differ')
f.ser.close()
expect_error(lambda: T.Flasher(srv.url), 'wants its key')

# a stream read over TCP, several frames; nothing left over for the next command
srv = FakeFlasherServer(key=KEY)
f = T.Flasher(srv.url, key=KEY)
assert f.stream_read('F', 0x100, 70000, progress=False) == srv.mem[0x100:0x100 + 70000]
assert f.rxbuf == b'' and f.cmd('V').startswith('tlsr-c3-flasher')
# the connection breaks in the middle of a stream: an error, not a hang
srv.drop_stream_after = 8192
t = time.time()
expect_error(lambda: f.stream_read('F', 0, 65536, progress=False), 'closed')
assert time.time() - t < 5
f.ser.close()

# bridge-exit over TCP: the key is checked, the target's bytes that waited are dropped, the escape after a pause
srv = FakeFlasherServer(key=KEY)
srv.bridge, srv.owner_ip = True, '10.9.9.9'                   # opened from another computer: the key is needed
assert T.bridge_exit(srv.url, KEY) == 'OK bridge closed' and not srv.bridge and srv.auth == ['key']
assert b'AUTH' not in srv.bridged_bytes and KEY.encode() not in srv.bridged_bytes
expect_error(lambda: T.bridge_exit(srv.url), 'closed')      # no key, not the owner: refused

# --esp over TCP: BRIDGE from the host, esptool gets socket://NAME:PORT and is let in without the key (the owner's
# address), its bytes reach the "target"; then bridge-exit and RST, both with the key
log = []


def fake_esptool_main(argv):
    log.append(('esptool',) + tuple(argv))
    port = argv[argv.index('--port') + 1]
    assert port == srv.url, port
    with T.TcpPort(port, timeout=2) as s:                      # no AUTH: esptool cannot send one
        s.write(b'\xc0sync\xc0')
        assert s.read(6) == b'\xc0sync\xc0'


srv = FakeFlasherServer(key=KEY)
sys.modules['esptool'] = types.SimpleNamespace(__version__='5.1.0', main=fake_esptool_main)
a = types.SimpleNamespace(op='info', port=srv.url, key=KEY, esp_baud=0, esp_chip='', esp_stay=False, address=0,
                          size=0, filename='', twice=False, no_verify=False, no_erase=False)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    T.esp_main(T.Flasher(srv.url, key=KEY), a)
assert [x for x in log if x[0] == 'esptool'][0][1:5] == ('--port', srv.url, '--baud', '921600'), log
assert srv.bridged_bytes == b'\xc0sync\xc0' and not srv.bridge, srv.bridged_bytes
assert srv.auth == ['key', 'owner', 'key', 'key'], srv.auth     # host, esptool, bridge-exit, RST
assert srv.lines == ['V', 'BRIDGE 115200 lo esp', 'V', 'RST'], srv.lines
no_secret(buf.getvalue(), KEY)

print('wifi host: all checks passed')
