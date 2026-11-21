import sys, os, zlib, random
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tlsr_c3 as T

class FakeC3:
    """Byte-level model of the firmware's XR / XW handling."""
    def __init__(self, mem, fail_at=None):
        self.mem, self.fail_at = mem, fail_at
        self.inb, self.out = bytearray(), bytearray()
        self.state = None; self.max_ahead = 0; self.writes = []; self.dropped = 0
    in_waiting = property(lambda self: len(self.out) or (1 if self.state == 'drain' else 0))
    def read(self, n):
        if self.state == 'drain' and not self.out:      # the host waits: the line is quiet, report
            self.state = None; self.out += b'E' + (6).to_bytes(2, 'little') + b'failed'
        d = bytes(self.out[:n]); del self.out[:n]; return d
    def write(self, b):
        if self.state == 'drain': self.dropped += len(b); return
        self.inb += b; self.pump()
    def frame(self, t, d=b''):
        self.out += t + len(d).to_bytes(2, 'little') + d + (zlib.crc32(d).to_bytes(4, 'little') if t == b'D' else b'')
    def pump(self):
        while True:
            if self.state is None:
                if b'\n' not in self.inb: return
                line, _, rest = bytes(self.inb).partition(b'\n'); self.inb = bytearray(rest)
                a = line.decode().split()
                if a[0] == 'XR':
                    addr, n = int(a[2]), int(a[3]); self.out += b'OK %d\n' % n
                    while n:
                        k = min(4096, n); self.frame(b'D', bytes(self.mem[addr:addr + k])); addr += k; n -= k
                    self.out += b'Z\x00\x00'                       # as x_frame sends it: type, length
                elif a[0] == 'XW':
                    self.state = [int(a[2]), int(a[3]), int(a[4]), a[5] == '1', 0]; self.out += b'OK\n'
                else:
                    self.out += b'OK\n'
            else:
                addr, left, unit, skip, nframe = self.state
                self.max_ahead = max(self.max_ahead, len(self.inb))
                if len(self.inb) < 2: return
                n = self.inb[0] | self.inb[1] << 8
                if len(self.inb) < n + 6: return
                data = bytes(self.inb[2:2 + n]); crc = int.from_bytes(self.inb[2 + n:6 + n], 'little'); del self.inb[:6 + n]
                assert crc == zlib.crc32(data)
                if nframe == self.fail_at:
                    self.dropped += len(self.inb); self.inb.clear(); self.state = 'drain'; return
                for p in range(0, n):
                    pass
                pos = 0
                while pos < n:
                    a = addr + pos; k = min(n - pos, unit - a % unit)
                    assert k > 0 and (a % unit) + k <= unit
                    part = data[pos:pos + k]
                    if not (skip and part.count(0xff) == k):
                        self.mem[a:a + k] = part; self.writes.append((a, k))
                    pos += k
                self.state = [addr + n, left - n, unit, skip, nframe + 1]
                self.out += b'K'
                if left - n == 0: self.state = None

def flasher(c3):
    f = object.__new__(T.Flasher); f.rxbuf = b''; f.ser = c3; return f

mem = bytearray(os.urandom(1 << 20)); c3 = FakeC3(mem); f = flasher(c3)
assert f.stream_read('P', 0x1234, 50000, progress=False) == bytes(mem[0x1234:0x1234 + 50000])
data = os.urandom(30000) + b'\xff' * 9000 + os.urandom(777)
mem2 = bytearray(b'\xee' * (1 << 20)); c3 = FakeC3(mem2); f = flasher(c3)
f.stream_write('S', 0x10f00, data, unit=512, skip=True, progress=False)
ff = [i for i in range(len(data)) if mem2[0x10f00 + i] != data[i]]
# skipped blank 512-unit parts keep the old 0xee bytes; everything else must match
assert all(data[i] == 0xff for i in ff), 'non-blank mismatch'
assert c3.max_ahead <= 3 * (4096 + 6), c3.max_ahead
print('frames ahead max bytes:', c3.max_ahead, ' writes:', len(c3.writes))
c3 = FakeC3(bytearray(1 << 20), fail_at=2); f = flasher(c3)
try: f.stream_write('P', 0, os.urandom(40000), progress=False); print('FAIL no error')
except T.FlasherError as e: print('error reported:', e, '| dropped bytes sent ahead:', c3.dropped)
assert c3.state is None and not c3.inb
# nothing may be left over after a read stream: the next command must get its own answer
f = flasher(FakeC3(bytearray(8192))); f.stream_read('P', 0, 8192, progress=False)
assert f.rxbuf == b'' and f.ser.out == b'', (f.rxbuf, f.ser.out)
assert f.cmd('V') == ''
print('STREAM OK')
