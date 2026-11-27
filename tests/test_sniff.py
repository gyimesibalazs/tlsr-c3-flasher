"""Two-line UART logger (firmware 0.30). src/sniffring.h (the live output's ring and frames) built natively
(tests/sniff_sim) and fed with made-up captures, its output parsed by tlsr_c3.py; the Tuya MCU protocol decoder; the
host's live output (sniff) against a fake flasher over a pseudo-terminal (as the USB port) and over TCP; bridge-exit
in the middle of the live frames; log --tuya. The firmware's capture itself (two UART drivers, src/sniff.cpp) runs
only on the C3. Needs g++."""
import os, random, re, select, socket, subprocess, sys, tempfile, threading, time, tty
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import tlsr_c3 as T

d = tempfile.mkdtemp()
exe = os.path.join(d, 'sniff_sim')
subprocess.check_call(['g++', '-std=c++17', '-Wall', '-Wextra', '-O2', '-o', exe,
                       os.path.join(HERE, 'sniff_sim', 'sniff_sim.cpp')])
rng = random.Random(7)


def ring_sim(pieces, ring_size, per_step):
    """pieces [(t_us, tag, bytes)] through the firmware's ring with a host taking per_step bytes after each piece."""
    src, stream = os.path.join(d, 'pieces.txt'), os.path.join(d, 'stream.bin')
    with open(src, 'w') as fh:
        for t, tag, data in pieces:
            fh.write('%d %d %s\n' % (t, tag, data.hex()))
    out = subprocess.check_output([exe, str(ring_size), str(per_step), src, stream]).decode()
    st = dict(kv.split('=') for kv in out.split())
    assert st['left'] == '0', st
    return open(stream, 'rb').read(), int(st['dropped'])


def parse(stream):
    """The stream fed to SniffParser in random pieces, as it comes over USB or TCP."""
    p, items, i = T.SniffParser(), [], 0
    while i < len(stream):
        k = rng.randint(1, 700)
        items += p.feed(stream[i:i + k])
        i += k
    assert not p.buf
    return items


def check_stream(pieces, items, dropped):
    """Every frame is a piece (split at 1024 bytes) in order; what is missing between two frames was reported by a
    "live output: N bytes dropped" event before the frame after it, with N right."""
    expected = [(t, tag, data[k:k + 1024]) for t, tag, data in pieces for k in range(0, max(len(data), 1), 1024)]
    i = reports = said = total = 0
    for kind, tag, t, data in items:
        assert kind == 'frame'
        m = re.fullmatch(rb'live output: (\d+) bytes dropped', data) if tag == 3 else None
        if m:
            said, reports = said + int(m.group(1)), reports + 1
            continue
        gap = 0
        while expected[i] != (t, tag, data):
            gap += len(expected[i][2])
            i += 1
        assert gap == said, 'dropped %d bytes before this frame, reported %d' % (gap, said)
        total, said, i = total + gap, 0, i + 1
    gap = sum(len(e[2]) for e in expected[i:])
    assert gap == said and total + gap == dropped, (gap, said, total, dropped)
    return reports


def tuya(cmd, data=b'', ver=3):
    f = b'\x55\xaa' + bytes([ver, cmd]) + len(data).to_bytes(2, 'big') + data
    return f + bytes([sum(f) & 0xFF])


def dp(dpid, typ, v):
    return bytes([dpid, typ]) + len(v).to_bytes(2, 'big') + v


# ---------------------------------------------------------------- the ring (native)
pieces, t = [], 0x7FFFF000                       # the clock wraps on the way
for k in range(600):
    t = (t + rng.randint(50, 3000)) & 0xFFFFFFFF
    if k % 50 == 0:
        pieces.append((t, 3, b'line A: 1 framing errors or breaks (wrong speed, or the line held low)'))
    else:
        n = 1500 if k % 97 == 0 else rng.randint(1, 400)
        pieces.append((t, 1 + k % 2, bytes(rng.getrandbits(8) for _ in range(n))))
# a host that keeps up: nothing dropped, every piece comes back as frames (long ones split)
stream, dropped = ring_sim(pieces, 10864, 0)
items = parse(stream)
assert dropped == 0 and check_stream(pieces, items, dropped) == 0
assert sum(len(x[3]) for x in items if x[1] != 3) == sum(len(p[2]) for p in pieces if p[1] != 3)
# a slow host (150 bytes per piece, frames cut anywhere): drops, each gap reported where it is
stream, dropped = ring_sim(pieces, 4096, 150)
items = parse(stream)
reports = check_stream(pieces, items, dropped)
assert dropped > 10000 and reports > 5, (dropped, reports)
slow_dropped, slow_reports = dropped, reports
# a host that takes a byte now and then: a small ring stays full, most is dropped, every gap still reported
stream, dropped = ring_sim(pieces[:100], 2048, 1)
assert check_stream(pieces[:100], parse(stream), dropped) >= 1 and dropped > sum(len(p[2]) for p in pieces[:100]) // 2
# the time of a frame: the 32-bit clock unwrapped
clock = T.UsClock()
assert [clock(x) for x in (0xFFFFFF00, 0x00000100, 0x00000200)] == [0, 0x200, 0x300]

# ---------------------------------------------------------------- the Tuya decoder
hb = tuya(0x00, b'\x01', ver=0)
rep = tuya(0x07, dp(1, 0, b'\x01\x02') + dp(2, 1, b'\x01') + dp(3, 2, (250).to_bytes(4, 'big')) +
           dp(4, 2, (-5 & 0xFFFFFFFF).to_bytes(4, 'big')) + dp(5, 3, b'hello') + dp(6, 4, b'\x02') +
           dp(7, 5, b'\x00\x05'))
dec = T.TuyaDecoder()
assert dec.feed(hb[:3]) == [] and dec.feed(hb[3:5]) == []      # split across records
assert dec.feed(hb[5:]) == ['tuya v0 heartbeat (0x00), data 01']
out = T.TuyaDecoder().feed(rep)
assert out == ["tuya v3 DP report (0x07): dp 1 raw len 2 = 01 02; dp 2 bool len 1 = true; dp 3 value len 4 = 250; "
               "dp 4 value len 4 = 4294967291 (-5 signed); dp 5 string len 5 = 'hello'; dp 6 enum len 1 = 2; "
               "dp 7 bitmap len 2 = 0x0005"], out
out = T.TuyaDecoder().feed(hb + tuya(0x06, dp(1, 1, b'\x00')))  # two frames in one record
assert out == ['tuya v0 heartbeat (0x00), data 01', 'tuya v3 DP command (0x06): dp 1 bool len 1 = false'], out
bad = bytearray(tuya(0x08))
bad[-1] ^= 0xFF
out = T.TuyaDecoder().feed(bytes(bad) + hb)                     # bad checksum, the next frame still found
assert out[0].startswith('tuya: bad checksum') and out[-1] == 'tuya v0 heartbeat (0x00), data 01', out
out = T.TuyaDecoder().feed(hb + b'\x00\x13noise' + hb)          # garbage between frames
assert out == ['tuya v0 heartbeat (0x00), data 01', 'tuya: 7 bytes outside frames: 00 13 6e 6f 69 73 65',
               'tuya v0 heartbeat (0x00), data 01'], out
dec = T.TuyaDecoder()
assert dec.feed(b'\x42\x55') == ['tuya: 1 bytes outside frames: 42'] and dec.feed(hb[1:])[-1].endswith('data 01')
out = T.TuyaDecoder().feed(b'\x55\xaa\x03\x07\xff\xff' + hb)     # a length no frame has: resync
assert 'not a frame' in out[0] and out[-1].endswith('data 01'), out
out = T.TuyaDecoder().feed(tuya(0x01, b'{"p":"abc","v":"1.0.0","m":0}') + tuya(0x1C, bytes([1, 24, 3, 9, 13, 5, 0, 6])) +
                           tuya(0x2B) + tuya(0x07, b'\x01'))
assert out == ['tuya v3 product info (0x01): \'{"p":"abc","v":"1.0.0","m":0}\'',
               'tuya v3 local time (0x1c): 2024-03-09 13:05:00, weekday 6', 'tuya v3 command 0x2b',
               'tuya v3 DP report (0x07), data 01'], out


# ---------------------------------------------------------------- live output against a fake flasher
class FakeSniffFlasher:
    """The flasher's command side for the logger: V, SNIFF (then the given live stream in random pieces until the
    escape after a pause of a second, then "OK sniff closed"), SNIFFBOOT. Over a pseudo-terminal (as the USB port)
    or TCP (one connection after another)."""
    def __init__(self, stream, tcp=False, live=False):
        self.stream, self.live = stream, live
        self.lines, self.pauses = [], []
        self.boot = '0 0'
        if tcp:
            self.lsock = socket.socket()
            self.lsock.bind(('127.0.0.1', 0))
            self.lsock.listen(2)
            self.port = '127.0.0.1:%d' % self.lsock.getsockname()[1]
            threading.Thread(target=self.serve_tcp, daemon=True).start()
        else:
            self.fd, slave = os.openpty()
            tty.setraw(self.fd)
            self.port = os.ttyname(slave)
            self.slave = slave
            threading.Thread(target=self.serve, args=(self.pty_recv, lambda b: os.write(self.fd, b)),
                             daemon=True).start()

    def pty_recv(self, wait):
        if not select.select([self.fd], [], [], wait)[0]:
            return b''
        try:
            return os.read(self.fd, 4096)
        except OSError:
            return b''

    def serve_tcp(self):
        while True:
            c, _ = self.lsock.accept()

            def recv(wait, c=c):
                if not select.select([c], [], [], wait)[0]:
                    return b''
                return c.recv(4096) or None
            try:
                self.serve(recv, c.sendall)
            except OSError:
                pass
            c.close()

    def serve(self, recv, send):
        buf, last = b'', time.time()
        pending = self.stream if self.live else b''
        while True:
            if self.live and pending:
                k = rng.randint(1, 300)
                send(pending[:k])
                pending = pending[k:]
            data = recv(0.005)
            if data is None:
                return
            if not data:
                continue
            now = time.time()
            buf += data
            if self.live:
                if buf.endswith(T.SNIFF_ESCAPE):
                    self.pauses.append(now - last)
                    send(pending + b'OK sniff closed\n')
                    self.live, pending, buf = False, b'', b''
                last = now
                continue
            last = now
            while b'\n' in buf:
                line, _, buf = buf.partition(b'\n')
                line = line.decode().strip()
                if not line:
                    continue
                self.lines.append(line)
                a = line.split()
                if a[0] == 'V':
                    send(b'OK tlsr-c3-flasher 0.30 mode=rmt baud=921600 rate=1000000 burst=64 tx=4 rx=5 rst=6 '
                         b'pwr=7+3 more\n')
                elif a[0] == 'SNIFF':
                    send(b'OK sniff A %s B %s; frames follow, leave with a 1 s pause and ~~~X\n' % (
                        a[1].encode(), a[2].encode()))
                    self.live, pending = True, self.stream
                elif a[0] == 'SNIFFBOOT':
                    self.boot = '0 0' if a[1] == '0' else '%s %s' % (a[1], a[2])
                    send(b'OK sniffboot off\n' if a[1] == '0' else b'OK sniffboot A %s B %s\n' % (
                        a[1].encode(), a[2].encode()))
                else:
                    send(b'ERR unknown command\n')


def run_cli(*argv):
    e = dict(os.environ)
    e.pop('C3_FLASHER_KEY', None)
    p = subprocess.run([sys.executable, os.path.join(HERE, '..', 'tlsr_c3.py')] + list(argv), env=e,
                       stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
    return p.returncode, p.stdout + p.stderr


# a Tuya exchange: the module asks on line B, the MCU answers on line A; frames cut across records
frames_b = [tuya(0x00), tuya(0x08), tuya(0x06, dp(1, 1, b'\x01'))]
frames_a = [tuya(0x00, b'\x01'), tuya(0x07, dp(1, 1, b'\x01') + dp(2, 2, (250).to_bytes(4, 'big'))),
            tuya(0x07, dp(1, 1, b'\x01'))]
live_pieces, t = [(1000, 3, b'sniff, A 9600, B 9600')], 1000
for fb, fa in zip(frames_b, frames_a):
    for tag, fr in ((1, fb), (2, fa)):
        cut = rng.randint(1, len(fr) - 1)
        for part in (fr[:cut], fr[cut:]):
            t += rng.randint(500, 5000)
            live_pieces.append((t, tag, part))
live_stream, dropped = ring_sim(live_pieces, 10864, 0)
assert dropped == 0
data_a = sum(len(p[2]) for p in live_pieces if p[1] == 2)
data_b = sum(len(p[2]) for p in live_pieces if p[1] == 1)

# USB (a pseudo-terminal): SNIFF, the frames shown as the log shows records, decoded, then the escape after a pause
fake = FakeSniffFlasher(live_stream)
f = T.Flasher(fake.port)
shown = []
view = T.SniffView(tuya=True, say=shown.append)
assert T.sniff_live(f, 9600, 9600, seconds=1.5, view=view) == 'OK sniff closed'
assert fake.lines == ['V', 'SNIFF 9600 9600'] and fake.pauses and fake.pauses[0] >= 1.0, (fake.lines, fake.pauses)
text = '\n'.join(shown)
assert '# sniff, A 9600, B 9600' in text and re.search(r'\d\.\d{6} B 55 aa', text), text
assert 'tuya v3 DP report (0x07): dp 1 bool len 1 = true; dp 2 value len 4 = 250' in text, text
assert 'tuya v3 DP query (0x08)' in text and 'outside frames' not in text, text
assert view.bytes == {1: data_b, 2: data_a} and view.dropped == 0
assert f.cmd('V').startswith('tlsr-c3-flasher 0.30')          # the flasher answers commands again
f.ser.close()

# over TCP through the command line, with a host that was too slow (drops reported), --out written
slow_pieces = live_pieces + [(p[0] + 100000, p[1], p[2]) for p in live_pieces[1:]]
slow_stream, dropped = ring_sim(slow_pieces, 48, 4)
assert dropped > 0
fake = FakeSniffFlasher(slow_stream, tcp=True)
out_file = os.path.join(d, 'sniff.txt')
code, out = run_cli('--host', fake.port, 'sniff', '9600', '--baud-b', '115200', '--tuya', '--seconds', '1.2',
                    '--out', out_file)
assert code == 0 and fake.lines == ['V', 'SNIFF 9600 115200'] and fake.pauses[0] >= 1.0, (out, fake.lines)
assert 'Listening: sniff A 9600 B 115200' in out, out
assert sum(int(n) for n in re.findall(r'live output: (\d+) bytes dropped', out)) == dropped, out
assert '%d bytes dropped from the live output' % dropped in out, out
saved = open(out_file).read()
assert saved and saved in out.replace('\r', ''), saved

# capture at power-up: set and off (USB)
fake = FakeSniffFlasher(b'')
code, out = run_cli('-p', fake.port, 'sniff', '9600', '--baud-b', '115200', '--at-boot')
assert code == 0 and fake.lines[-1] == 'SNIFFBOOT 9600 115200' and 'A 9600 B 115200' in out, out
assert 'lost when the flasher loses power' in out
code, out = run_cli('-p', fake.port, 'sniff', '--no-at-boot')
assert code == 0 and fake.lines[-1] == 'SNIFFBOOT 0' and fake.boot == '0 0' and 'off' in out, out
code, out = run_cli('-p', fake.port, 'sniff')
assert code == 1 and 'speed of line A' in out, out

# bridge-exit while the logger sends frames: the closing line is found behind them (USB and TCP)
fake = FakeSniffFlasher(live_stream * 20, live=True)
assert T.bridge_exit(fake.port) == 'OK sniff closed' and not fake.live
fake = FakeSniffFlasher(live_stream * 20, tcp=True, live=True)
assert T.bridge_exit('socket://' + fake.port) == 'OK sniff closed' and not fake.live

# log --tuya: a sniff session in the log reads A / B, each line decoded on its own
entries = [(0, 3, b'sniff, A 9600, B 9600')] + [(p[0], p[1], p[2]) for p in live_pieces[1:]] + [(99999, 3, b'end')]
text = T.ulog_text(entries, T.TuyaLines())
assert re.search(r'^ +\d+\.\d{6} A 55 aa', text, re.M) and ' > ' not in text, text
assert text.count('tuya v3 DP report (0x07)') == 2 and text.count('tuya v3 heartbeat') == 2, text
assert 'outside frames' not in text
assert ' A ' in T.ulog_text(entries) and 'tuya' not in T.ulog_text(entries)
print('sniff: all checks passed (slow host: %d bytes dropped from the live output, %d reports)' % (slow_dropped,
                                                                                               slow_reports))
