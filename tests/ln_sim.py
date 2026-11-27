"""Model of the LN882H set-up for tlsr_c3.py's --ln mode: the flasher's bridge ("BRIDGE 115200 lo ln", the speed
following of src/lnscan.h, the escape) and the chip: the ROM download mode (version, download [rambin] over YMODEM)
and either RAM loader:
- Lightning's ("vendor"), modelled as its source in the OpenLN882H SDK behaves (echo, version, flash_info, flash_uid,
  flash_read with a CRC-16, ferase, ferase_all, startaddr, upgrade over YMODEM erasing the range first, baudrate);
- ours (loaders/ln882h), the real loader.c and flash.c built natively against tests/ln_loader_sim/fake_hal.c, with
  the SPI NOR flash of tests/ln_nor_model.py behind it. The ROM starts it when the loaded image is the one embedded
  in tlsr_c3.py.
Bytes sent while the bridge and the chip run at different speeds arrive garbled, so a switch at the wrong moment fails
the test. Used by test_ln_host.py and test_ln_loader.py."""
import ctypes, os, re, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import tlsr_c3 as T
from lnscan_model import LnScan
from ln_nor_model import Nor

T.BRIDGE_PAUSE = 0
SOH, EOT, ACK, NAK, CAN = 0x01, 0x04, 0x06, 0x15, 0x18
LOADER_DIR = os.path.join(ROOT, 'loaders', 'ln882h')

_lib = None


def native_lib():
    """loader.c + flash.c + fake_hal.c as a shared library (built once per run)."""
    global _lib
    if _lib is None:
        out = os.path.join(tempfile.mkdtemp(), 'ln_loader_native.so')
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1', '-shared', '-fPIC', '-I', LOADER_DIR,
                        '-o', out, os.path.join(LOADER_DIR, 'loader.c'), os.path.join(LOADER_DIR, 'flash.c'),
                        os.path.join(HERE, 'ln_loader_sim', 'fake_hal.c')], check=True)
        _lib = ctypes.CDLL(out)
    return _lib


READ_FN = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32)
WRITE_FN = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32)
SPI_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8),
                          ctypes.c_uint32)
BAUD_FN = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_uint32)
EVENT_FN = ctypes.CFUNCTYPE(None, ctypes.c_int)


class NativeLoader:
    """Our loader, built natively; its UART is the sim's wire, its flash the NOR model."""

    def __init__(self, sim, nor):
        self.sim, self.nor = sim, nor
        self.lib = native_lib()
        self.rx = bytearray()
        self.events = []
        self.dead = False                  # a loader that never starts (the sim's "silent" case)
        self.deaf = False                  # a loader that runs but receives nothing (the sim's "deaf" case)

        def read(buf, mx):
            n = min(mx, len(self.rx))
            if n:
                ctypes.memmove(buf, bytes(self.rx[:n]), n)
                del self.rx[:n]
            return n

        def write(buf, n):
            self.sim.from_loader(ctypes.string_at(buf, n))

        def spi(tx, ntx, rx, nrx):
            try:
                out = self.nor.transfer(ctypes.string_at(tx, ntx), nrx)
            except Exception as e:           # an exception cannot cross ctypes: record it
                self.sim.violations.append('NOR model: %r' % e)
                return 12
            if nrx:
                ctypes.memmove(rx, out, nrx)
            return 0

        def baud(b):
            self.sim.target_baud = b
            self.sim.log.append('own: baud %d' % b)
            return 0

        def event(what):
            self.events.append(what)
            if what == 9:
                self.sim.reboot()
        # kept as attributes: ctypes needs the objects alive while C holds the pointers
        self.hooks = (READ_FN(read), WRITE_FN(write), SPI_FN(spi), BAUD_FN(baud), EVENT_FN(event))
        self.lib.sim_hooks(*self.hooks)

    def start(self):
        self.rx.clear()
        if not self.dead:
            self.lib.loader_init()

    def feed(self, b):
        if not self.deaf:
            self.rx.append(b)

    def poll(self):
        if not self.dead:
            self.lib.loader_poll()


class Sim:
    def __init__(self, size=0x200000, boot_wired=True, fail_above=0, bad_crc_once=False, ramcode=b'', nor=None,
                 corrupt_in=0, corrupt_out=0, silent=False, deaf=False, own_baud=0):
        self.flash = bytearray(os.urandom(size))
        self.size = size
        self.boot_wired, self.fail_above, self.bad_crc_once = boot_wired, fail_above, bad_crc_once
        self.ramcode_expected = ramcode
        self.out = bytearray()
        self.mode = 'cmd'                  # the flasher: cmd | bridge
        self.cmdline = b''
        self.target = 'off'                # rom | ramcode | own | app | off
        self.bridge_baud = self.target_baud = 115200
        self.violations = []
        self.log = []
        self.ym = None
        self.startaddr = 0
        self.line = b''
        # our loader: corrupt_in / corrupt_out: change one byte inside the n-th frame to it / from it
        self.nor = nor or Nor(size=size, data=self.flash)
        if nor:
            self.flash, self.size = nor.mem, nor.size
        self.native = None
        self.silent, self.deaf = silent, deaf
        self.own_baud = own_baud           # our loader's UART comes up at this speed instead of 115200
        self.corrupt_in, self.corrupt_out = corrupt_in, corrupt_out
        self.frames_in = self.frames_out = 0
        self.in_pos = -1

    # ---- the port
    def write(self, data):
        data = bytes(data)
        if self.mode == 'bridge':
            if data == b'~~~X':
                self.mode = 'cmd'
                self.log.append('bridge closed')
                self.out += b'OK bridge closed\n'
                return len(data)
            m = re.fullmatch(rb'~~~S([0-9]+)\n', data)
            if m:                                   # the speed escape (firmware 0.27): nothing reaches the chip
                self.bridge_baud = int(m.group(1))
                self.log.append('bridge speed %d' % self.bridge_baud)
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
            self.out += b'OK tlsr-c3 0.30 mode=rmt\n'
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

    def reboot(self):
        self.log.append('own: reboot')
        self.target = 'app'
        self.target_baud = 115200

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
        if self.target == 'own':
            self.native.poll()

    def to_host(self, data):
        if self.mode == 'bridge':                    # out of the bridge nobody passes the chip's UART on
            self.out += bytes(self.garble(b) for b in data)

    def from_loader(self, data):
        data = bytearray(data)
        if self.corrupt_out and data and data[0] == T.LN_FR_START:
            self.frames_out += 1
            k = next((i for i in range(6, len(data) - 2) if data[i] not in (0xC0, 0xC1, 0xDB)), None)
            if self.frames_out == self.corrupt_out and k is not None:
                data[k] ^= 0x01
                self.log.append('corrupted an answer')
        self.to_host(bytes(data))

    # ---- the chip
    def target_in(self, b):
        if self.target == 'app' or self.target == 'off':
            return
        if self.target == 'own':
            if self.corrupt_in and b == T.LN_FR_START:
                self.frames_in += 1
                self.in_pos = 0 if self.frames_in == self.corrupt_in else -1
            elif self.in_pos >= 0:
                self.in_pos += 1
                if self.in_pos == 8 and b not in (0xC0, 0xC1, 0xDB):
                    b ^= 0x04
                    self.in_pos = -1
                    self.log.append('corrupted a request')
            self.native.feed(b)
            if b in (13, 10):
                self.native.poll()               # a line is acted on at its end, before the bridge switches
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
        self.log.append('ymodem %s %d bytes' % (y['where'], len(data)))
        if y['where'] == 'ram':
            self.target_baud = 115200
            if data == T.LN_LOADER:
                self.target = 'own'
                if self.native is None:
                    self.native = NativeLoader(self, self.nor)
                self.native.dead, self.native.deaf = self.silent, self.deaf
                # deaf: 20 ms a poll, so that the wait line comes every 10 polls (the host polls it rarely then)
                self.native.lib.sim_cycles_step(3200000 if self.deaf else 208333)
                if self.own_baud:
                    self.target_baud = self.own_baud
                self.native.start()
            else:
                if data != self.ramcode_expected:
                    self.violations.append('the loaded RAM code differs')
                self.target = 'ramcode'
        else:
            a = y['addr']
            for i, b in enumerate(data):
                self.flash[a + i] &= b                  # NOR: a write only clears bits


def flasher(sim):
    f = object.__new__(T.Flasher)
    f.ser = sim                        # the port: write, read, in_waiting, reset_input_buffer
    f.rxbuf = b''
    f.version = 'tlsr-c3 0.30'
    return f


def check_violations(sim):
    v = sim.violations + sim.nor.violations
    assert not v, v
