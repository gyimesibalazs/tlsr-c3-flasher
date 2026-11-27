"""LN882H mode (--ln): tlsr_c3.py against a model of the flasher's bridge ("BRIDGE 115200 lo ln", the speed following
of src/lnscan.h, the escape) and of the chip: the ROM download mode (version, download [rambin] over YMODEM) and the
RAM loader as its source in the OpenLN882H SDK behaves (echo, version, flash_info, flash_uid, flash_read with a CRC-16,
ferase, ferase_all, startaddr, upgrade over YMODEM erasing the range first, baudrate). Bytes sent while the bridge and
the chip run at different speeds arrive garbled, so a switch at the wrong moment fails the test.
Also builds src/lnscan.h natively and checks it on its own."""
import os, re, subprocess, sys, tempfile, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tlsr_c3 as T

T.BRIDGE_PAUSE = 0
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SOH, EOT, ACK, NAK, CAN = 0x01, 0x04, 0x06, 0x15, 0x18


from lnscan_model import LnScan


class Sim:
    def __init__(self, size=0x200000, boot_wired=True, fail_above=0, bad_crc_once=False, ramcode=b''):
        self.flash = bytearray(os.urandom(size))
        self.size = size
        self.boot_wired, self.fail_above, self.bad_crc_once = boot_wired, fail_above, bad_crc_once
        self.ramcode_expected = ramcode
        self.out = bytearray()
        self.mode = 'cmd'                  # the flasher: cmd | bridge
        self.cmdline = b''
        self.target = 'off'                # rom | ramcode | app | off
        self.bridge_baud = self.target_baud = 115200
        self.violations = []
        self.log = []
        self.ym = None
        self.startaddr = 0
        self.line = b''

    # ---- the port
    def write(self, data):
        data = bytes(data)
        if self.mode == 'bridge':
            if data == b'~~~X':
                self.mode = 'cmd'
                self.log.append('bridge closed')
                self.out += b'OK bridge closed\n'
                return len(data)
            self.bridge_in(data)
            return len(data)
        for b in data:
            if b == 10:
                self.flasher_cmd(self.cmdline.decode().strip())
                self.cmdline = b''
            else:
                self.cmdline += bytes([b])
        return len(data)

    def read(self, n=1):
        d = bytes(self.out[:n])
        del self.out[:n]
        return d

    @property
    def in_waiting(self):
        return len(self.out)

    def reset_input_buffer(self):
        self.out.clear()

    # ---- the flasher's command mode
    def flasher_cmd(self, line):
        if not line:
            return
        self.log.append(line)
        if line == 'V':
            self.out += b'OK tlsr-c3 0.26 mode=rmt\n'
        elif line.startswith('BRIDGE'):
            a = line.split()
            self.bridge_baud = int(a[1])
            self.follow = len(a) > 3 and a[3] == 'ln'
            self.scan = LnScan()
            self.mode = 'bridge'
            self.out += b'OK bridge %s baud, boot low\n' % a[1].encode()
            self.power_on(boot_low=len(a) > 2 and a[2] == 'lo')
        elif line == 'RST':
            self.power_on(boot_low=False)
            self.out += b'OK\n'
        else:
            self.out += b'ERR unknown command\n'

    def power_on(self, boot_low):
        self.target = 'rom' if boot_low and self.boot_wired else 'app'
        self.target_baud = 115200
        self.line = b''
        self.ym = None
        if self.target == 'app':
            self.to_host(b'\r\nOpenLN882H booting...\r\n')

    # ---- the wire
    def garble(self, b):
        ok = self.bridge_baud == self.target_baud and not (self.fail_above and self.target_baud > self.fail_above)
        return b if ok else 0xFF

    def bridge_in(self, data):
        for b in data:
            baud = self.scan.feed(b) if self.follow else 0
            self.target_in(self.garble(b))
            if baud:
                self.bridge_baud = baud

    def to_host(self, data):
        if self.mode == 'bridge':                    # out of the bridge nobody passes the chip's UART on
            self.out += bytes(self.garble(b) for b in data)

    # ---- the chip
    def target_in(self, b):
        if self.target == 'app' or self.target == 'off':
            return
        if self.ym is not None:
            self.ym_in(b)
            return
        if self.target == 'ramcode' and 0x20 <= b <= 0x7e:
            self.to_host(bytes([b]))                 # the loader's echo
        if b in (13, 10):
            line, self.line = self.line.decode(errors='replace').strip(), b''
            if line:
                self.execute(line)
        elif 0x20 <= b <= 0x7e:
            self.line += bytes([b])
        elif b != 0xFF:
            pass

    def execute(self, line):
        self.log.append('%s: %s' % (self.target, line))
        a = line.split()
        if self.target == 'rom':
            if line == 'version':
                self.to_host(b'Mar 14 2021/00:23:32\r\n')
            elif a[0] == 'download' and a[1] == '[rambin]':
                size = int(a[3].strip('[]'))
                self.ym_start('ram', 0, size)
            return
        if line == 'version':
            self.to_host(b'\r\nRAMCODE\r\n')
        elif line == 'flash_info':
            self.to_host(b'\r\nid:0xEB60%02X,flash size:%dM Byte\r\n' % (self.size.bit_length() - 1,
                                                                        2 if self.size == 0x200000 else 1))
        elif line == 'flash_uid':
            self.to_host(b'\r\nflash uid:0x' + b'A5' * 16 + b'\r\n')
        elif line == 'get_mac_in_flash_otp':
            self.to_host(b'\r\nmac:c0:f5:a1:02:03:04\r\npppp\r\n')
        elif a[0] == 'flash_read' and len(a) == 3:
            addr, n = int(a[1], 16), int(a[2], 16)
            data = bytes(self.flash[addr:addr + n])
            crc = T.crc16_xmodem(data)
            if self.bad_crc_once:
                self.bad_crc_once = False
                crc ^= 1
            self.to_host(b'\r\n' + (data + crc.to_bytes(2, 'big')).hex().upper().encode())
        elif a[0] == 'ferase' and len(a) == 3:
            addr, n = int(a[1], 0), int(a[2], 0)
            if n and addr + n < 4 << 20:
                self.erase(addr, n)
                self.to_host(b'\r\npppp\r\n')
            else:
                self.to_host(b'\r\nffff\r\n')
        elif line == 'ferase_all':
            self.flash[:] = b'\xff' * self.size
            self.to_host(b'\r\npppp\r\n')
        elif a[0] == 'startaddr' and len(a) == 2:
            self.startaddr = int(a[1], 16)
            self.to_host(b'\r\npppp\r\n')
        elif line == 'upgrade':
            self.ym_start('flash', self.startaddr, None)
        elif a[0] == 'baudrate' and len(a) == 2:
            self.target_baud = int(a[1])
        else:
            self.violations.append('unknown loader command %r' % line)

    def erase(self, addr, n):
        """bootram_flash_erase: an unaligned start keeps the bytes before it in the first sector."""
        base = addr - addr % 4096
        end = addr + n
        end += -end % 4096
        keep = bytes(self.flash[base:addr])
        self.flash[base:end] = b'\xff' * (end - base)
        self.flash[base:addr] = keep

    # ---- YMODEM receiver (ymodem_recieve_loop)
    def ym_start(self, where, addr, size):
        self.ym = dict(where=where, addr=addr, size=size, buf=b'', count=0, data=bytearray(), eot=0)
        self.to_host(b'C')

    def ym_in(self, b):
        y = self.ym
        y['buf'] += bytes([b])
        buf = y['buf']
        if buf[0] == EOT:
            y['buf'] = b''
            y['eot'] += 1
            self.to_host(bytes([NAK]) if y['eot'] == 1 else bytes([ACK]) + b'C')
            return
        if buf[0] != SOH:
            # the receiver counts it as an error and goes on (more than 20 cancel the transfer): the LF after the
            # command line that started the transfer is one
            y['buf'] = b''
            y['noise'] = y.get('noise', 0) + 1
            if y['noise'] > 2:
                self.violations.append('YMODEM: byte 0x%02x outside a packet' % buf[0])
            return
        if len(buf) < 133:
            return
        y['buf'] = b''
        seq, nseq, data, crc = buf[1], buf[2], buf[3:131], int.from_bytes(buf[131:133], 'big')
        if seq != 255 - nseq or T.crc16_xmodem(data) != crc:
            self.to_host(bytes([NAK]))
            return
        if y['count'] == 0:
            name, rest = data.split(b'\0', 1)
            y['size'] = int(re.match(rb'[0-9]+', rest).group(0))
            if y['where'] == 'flash':
                self.erase(y['addr'], y['size'] + (-y['size'] % 4096))
            y['count'] = 1
            self.to_host(bytes([ACK]) + b'C')
            return
        if y['eot'] >= 2 and seq == 0:
            self.to_host(bytes([ACK]))
            self.ym_done()
            return
        if seq != y['count'] & 0xFF:
            self.to_host(bytes([NAK]))
            return
        y['data'] += data
        y['count'] += 1
        self.to_host(bytes([ACK]))

    def ym_done(self):
        y, self.ym = self.ym, None
        data = bytes(y['data'][:y['size']])
        if y['where'] == 'ram':
            if data != self.ramcode_expected:
                self.violations.append('the loaded RAM code differs')
            self.target = 'ramcode'
            self.target_baud = 115200
        else:
            a = y['addr']
            for i, b in enumerate(data):
                self.flash[a + i] &= b                  # NOR: a write only clears bits
        self.log.append('ymodem %s %d bytes' % (y['where'], len(data)))


def flasher(sim):
    f = object.__new__(T.Flasher)
    f.ser = sim                        # the port: write, read, in_waiting, reset_input_buffer
    f.rxbuf = b''
    f.version = 'tlsr-c3 0.26'
    return f


tmp = tempfile.mkdtemp()
RAMCODE = os.path.join(tmp, 'ramcode.bin')
RAMCODE_DATA = bytes(range(256)) * 147 + b'\x01\x02'      # 37634 bytes, not a multiple of 128
with open(RAMCODE, 'wb') as fh:
    fh.write(RAMCODE_DATA)


def args(op, **kw):
    a = types.SimpleNamespace(op=op, ln_baud=0, ln_ramcode=RAMCODE, ln_seconds=1, run=False, address=0, size=0,
                              filename='', twice=False, no_verify=False, no_erase=False)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def run(sim, a):
    T.ln_main(flasher(sim), a)
    assert not sim.violations, sim.violations
    assert sim.mode == 'cmd', 'the bridge was left open'


# ---- src/lnscan.h natively, against the Python model, on the same streams
cases = [b'baudrate 1000000\r', b'version\r\nbaudrate 460800\n', b'xbaudrate 9600\r', b'baudrate 12x\r',
         b'\x01baudrate 921600\r', b'\rbaudrate 921600\r', b'baudrate 100\r', b'baudrate 99999999999\r',
         b'baudrate  115200\r', b'baudrate 115200', b'BAUDRATE 115200\r', b'\x00\xffbaudrate 2000000\n',
         b'baudrate 3000000\r\nbaudrate 115200\r', b'baudrate 115200 0 1\r']
UCFG = [b'ucfg 1500000 0 1\n', b'ucfg 921600\n', b'ucfg 1500000x 0 1\n', b'ping\nucfg 2000000 4 1\r',
        b'\x02ucfg 1500000 0 1\n', b'ucfg  1500000\n', b'baudrate 115200\n']
cases = [(c, 0) for c in cases] + [(c, 1) for c in UCFG]
src = os.path.join(tmp, 'lnscan_test.cpp')
with open(src, 'w') as fh:
    fh.write('#include "lnscan.h"\n#include <stdio.h>\n#include <string.h>\nint main() {\n'
             '  static const char *cases[] = {%s};\n'
             '  static const int lens[] = {%s};\n'
             '  static const int ucfg[] = {%s};\n'
             '  for (int k = 0; k < %d; k++) { LineBaudScan s = ucfg[k] ? LineBaudScan("ucfg ", true) :'
             ' LineBaudScan(); for (int i = 0; i < lens[k]; i++) {'
             ' unsigned b = s.feed((uint8_t)cases[k][i]); if (b) printf("%%d:%%d:%%u ", k, i, b); } }\n'
             '  printf("\\n"); return 0; }\n' % (
                 ', '.join('"%s"' % ''.join('\\x%02x' % x for x in c) for c, _ in cases),
                 ', '.join(str(len(c)) for c, _ in cases), ', '.join(str(u) for _, u in cases), len(cases)))
exe = os.path.join(tmp, 'lnscan_test')
subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', os.path.join(ROOT, 'src'), '-o', exe, src],
               check=True)
native = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split()
model = []
for k, (c, u) in enumerate(cases):
    s = LnScan(b'ucfg ', True) if u else LnScan()
    for i, b in enumerate(c):
        v = s.feed(b)
        if v:
            model.append('%d:%d:%d' % (k, i, v))
assert native == model, (native, model)
assert model == ['0:16:1000000', '1:24:460800', '5:16:921600', '12:16:3000000', '12:33:115200',
                 '14:16:1500000', '15:11:921600', '17:21:2000000'], model
print('lnscan.h: %d cases, native = model' % len(cases))

# ---- info: ROM, RAM loader over YMODEM, speed raised to 1000000 by the bridge following the line
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('info'))
assert sim.log[0] == 'BRIDGE 115200 lo ln', sim.log
assert 'ymodem ram %d bytes' % len(RAMCODE_DATA) in sim.log
assert sim.bridge_baud == sim.target_baud == 1000000
assert 'ramcode: baudrate 1000000' in sim.log
print('info: ok')

# ---- rf of a range and the whole flash, --twice; a bad CRC once is read again
sim = Sim(ramcode=RAMCODE_DATA, bad_crc_once=True)
out = os.path.join(tmp, 'part.bin')
run(sim, args('rf', address=0x1234, size=0x3456, filename=out, twice=True))
assert open(out, 'rb').read() == bytes(sim.flash[0x1234:0x1234 + 0x3456])
sim = Sim(size=0x100000, ramcode=RAMCODE_DATA)
run(sim, args('rf', address=0, size=0, filename=out))
assert open(out, 'rb').read() == bytes(sim.flash)
print('rf: ok')

# ---- wf: unaligned start and end; what is around the range in its first and last sectors stays
sim = Sim(ramcode=RAMCODE_DATA)
before = bytes(sim.flash)
img = os.urandom(0x2345)
fn = os.path.join(tmp, 'img.bin')
with open(fn, 'wb') as fh:
    fh.write(img)
run(sim, args('wf', address=0x10100, filename=fn))
assert bytes(sim.flash[0x10100:0x10100 + len(img)]) == img
assert bytes(sim.flash[:0x10100]) == before[:0x10100]
assert bytes(sim.flash[0x10100 + len(img):]) == before[0x10100 + len(img):]
print('wf: ok')

# ---- es, ea
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('es', address=0x5010, size=0x10))
assert all(b == 0xff for b in sim.flash[0x5000:0x6000])
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('ea'))
assert all(b == 0xff for b in sim.flash)
print('es, ea: ok')

# ---- the chip's UART fails above 460800: a new session at 460800
sim = Sim(ramcode=RAMCODE_DATA, fail_above=460800)
run(sim, args('info'))
assert sim.log.count('BRIDGE 115200 lo ln') == 2 and sim.target_baud == 460800, sim.log
print('fallback to 460800: ok')

# ---- BOOT not wired: the chip runs its firmware, the ROM never answers; the bridge is left
sim = Sim(ramcode=RAMCODE_DATA, boot_wired=False)
try:
    T.ln_main(flasher(sim), args('info'))
    raise AssertionError('no error')
except T.FlasherError as e:
    assert 'no answer from the ROM' in str(e), e
assert sim.mode == 'cmd' and sim.log.count('BRIDGE 115200 lo ln') == 1
print('no ROM: ok')

# ---- data with a "baudrate N" line is refused before anything is written
sim = Sim(ramcode=RAMCODE_DATA)
with open(fn, 'wb') as fh:
    fh.write(b'\x00' * 100 + b'\nbaudrate 9600\r\n' + b'\x00' * 100)
before = bytes(sim.flash)
try:
    run(sim, args('wf', address=0x20000, filename=fn))
    raise AssertionError('no error')
except T.FlasherError as e:
    assert 'baudrate' in str(e), e
assert bytes(sim.flash) == before and sim.mode == 'cmd'
print('baudrate line in the data refused: ok')

# ---- --run: restart with BOOT released
sim = Sim(ramcode=RAMCODE_DATA)
run(sim, args('info', run=True))
assert sim.log[-1] == 'RST' and sim.target == 'app'
print('--run: ok')
print('all LN882H host tests passed')
