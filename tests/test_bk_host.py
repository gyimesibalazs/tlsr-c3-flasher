"""Beken BK72xx mode: the host logic of tlsr_c3.py --bk against the simulated target of tests/bk_sim.

The simulator runs the real src/bk72xx.cpp (catch, link, speed change, the download protocol, the BK commands) on a
simulated UART wire against a model of the BK7231N BootROM / BK7231T bootloader and its flash, so these tests go
from the host's commands down to the bytes on the wire. Only the binary streams are replaced by their text
equivalents here (BKR, BKB + BKW; test_streams.py checks the streams). Needs g++.

    python3 tests/test_bk_host.py
"""
import contextlib
import io
import os
import random
import shutil
import subprocess
import sys
import tempfile
import types
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
import tlsr_c3 as T


def build_sim():
    out = os.path.join(tempfile.gettempdir(), 'bk_sim_%d' % os.getpid())
    subprocess.check_call(['g++', '-std=c++17', '-O1', '-I', os.path.join(HERE, 'bk_sim', 'stub'),
                           '-I', os.path.join(ROOT, 'src'), '-o', out, os.path.join(HERE, 'bk_sim', 'bk_sim.cpp'),
                           os.path.join(ROOT, 'src', 'bk72xx.cpp'), os.path.join(ROOT, 'src', 'tuart.cpp'),
                           os.path.join(ROOT, 'src', 'uartlog.cpp')])
    return out


class SimFlasher:
    """The parts of tlsr_c3.Flasher the BK code uses, talking to bk_sim --serve."""
    def __init__(self, exe):
        self.p = subprocess.Popen([exe, '--serve'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.log = []
        self.ser = types.SimpleNamespace(reset_input_buffer=lambda: None)
        self.rxbuf = b''

    def raw(self, text):
        self.p.stdin.write(text + '\n')
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def cmd(self, text, timeout=5):
        self.log.append(text.split()[0])
        answer = self.raw(text)
        if answer == 'OK':
            return ''
        if answer.startswith('OK '):
            return answer[3:]
        raise T.FlasherError('%s -> %s' % (text.split()[0], answer))

    def stream_read(self, src, addr, size, label='Read', progress=True):
        assert src == 'B'
        self.log.append('XR')
        out = b''
        while len(out) < size:
            n = min(512, size - len(out))
            out += bytes.fromhex(self.cmd('BKR %d %d' % (addr + len(out), n)))
        return out

    def stream_write(self, dst, addr, data, unit=256, skip=True, label='Write', progress=True, window=3):
        assert dst == 'B' and unit == 4096 and addr % 4096 == 0 and len(data) % 4096 == 0
        self.log.append('XW')
        for pos in range(0, len(data), 4096):
            sector = data[pos:pos + 4096]
            if skip and sector == b'\xff' * 4096:
                continue
            for k in range(0, 4096, 512):
                self.cmd('BKB %d %s' % (k, sector[k:k + 512].hex()))
            self.cmd('BKW %d' % (addr + pos))

    def stop(self):
        pass

    def peek(self, addr, size):
        return bytes.fromhex(self.raw('SIM peek %d %d' % (addr, size))[3:])

    def stat(self):
        return dict(kv.split('=', 1) for kv in self.raw('SIM stat')[3:].split(' '))

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def args(op, **kw):
    a = dict(op=op, bk_baud=921600, bk_seconds=10, bk_sr=0, force_bootloader=False, run=False, no_verify=False,
             twice=False, address=0, size=0, filename='')
    a.update(kw)
    return types.SimpleNamespace(**a)


def run(f, a):
    """bk_main with its output captured; returns (target, text)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        t = T.bk_ops(f, a)
    return t, buf.getvalue()


def expect_error(fn, *words):
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            fn()
    except T.FlasherError as e:
        text = str(e)
        for w in words:
            assert w in text, 'expected %r in: %s' % (w, text)
        return text
    raise AssertionError('no error')


def clean(f):
    s = f.stat()
    assert s['violations'] == '0' and s['floating'] == '0', s


def uf2(parts, family=0x7B3EF230, ptable=(('app', 0x11000, 0x119000), ('download', 0x12A000, 0xA6000))):
    """A LibreTiny-style UF2 as ltchiptool's uf2tool writes it (OTA format 2): a header block with the global
    tags, then 256-byte data blocks; the first block of every image carries its OTA_PART_INFO tag.
    parts: [(data, relative offset, partition for the device scheme, partition for the flasher scheme)]"""
    def tag(t, v):
        out = bytes([4 + len(v)]) + t.to_bytes(3, 'little') + v
        return out + b'\0' * (-len(out) % 4)

    pt = b''
    for name, off, ln in ptable:
        pt += (0x45503130).to_bytes(4, 'little') + name.encode().ljust(16, b'\0') + b'flash0'.ljust(16, b'\0') + \
            off.to_bytes(4, 'little') + ln.to_bytes(4, 'little') + b'\0' * 4
    gtags = tag(0x9FC7BC, b'2025.11.0') + tag(0x00DE43, b'esphome') + tag(0xCA25C8, b'cb2s') + \
        tag(0x8288ED, pt) + tag(0x6C8492, b'\x02') + tag(0x59563D, b'1.9.1')
    blocks = [(0x8000 | 0x2000 | 1, 0, b'', gtags)]
    for data, offset, device, flasher in parts:
        names = sorted({p for p in (device, flasher) if p})
        idx = [names.index(device) + 1 if device else 0, 0, 0, names.index(flasher) + 1 if flasher else 0, 0, 0]
        info = bytes.fromhex(''.join(str(i) for i in idx)) + b'\0'.join(n.encode() for n in names) + b'\0'
        for i in range(0, len(data), 256):
            tags = tag(0xC0EE0C, info) if i == 0 else b''
            blocks.append((0x2000 | (0x8000 if tags else 0), offset + i, data[i:i + 256], tags))
    out = b''
    for seq, (flags, addr, data, tags) in enumerate(blocks):
        b = (0x0A324655).to_bytes(4, 'little') + (0x9E5D5157).to_bytes(4, 'little')
        for v in (flags, addr, len(data), seq, len(blocks), family):
            b += v.to_bytes(4, 'little')
        b += data + tags + b'\0' * 4
        b += b'\0' * (508 - len(b)) + (0x0AB16F30).to_bytes(4, 'little')
        assert len(b) == 512
        out += b
    return out


def main():
    if not shutil.which('g++'):
        print('g++ not found: skipped')
        return
    exe = build_sim()
    f = SimFlasher(exe)
    tmp = tempfile.mkdtemp()
    fn = os.path.join(tmp, 'img.bin')
    rnd = random.Random(7)
    try:
        # info: BootROM protocol, chip and flash id, protection, MAC from the RF calibration
        f.raw('SIM new sr1=1c sr2=40')
        t, text = run(f, args('info'))
        assert t.rom and t.chip == 'BK7231N' and t.flash_id == 'eb6015' and t.size == 0x200000, vars(t)
        assert t.baud == 921600 and 'status 401c: protected' in text and 'c8:47:8c:12:34:56' in text, text
        print(text.strip().replace('\n', '\n    '))

        # BK7238 (Tuya T1): the same BootROM protocol; the MAC is in the calibration of the T1 layout
        f.raw('SIM new chip=7238')
        t, text = run(f, args('info'))
        assert t.rom and t.chip == 'BK7238' and 'c8:47:8c:12:34:56' in text and '0x1e3000' in text, text
        f.raw('SIM new sr1=1c sr2=40')

        # full backup, twice: every block checked by CRC while reading, then the CRC of the whole range
        out = os.path.join(tmp, 'dump.bin')
        t, text = run(f, args('rf', size=0, filename=out, twice=True))
        assert open(out, 'rb').read() == f.peek(0, 0x200000), 'backup differs'
        assert 'Second pass' in text
        clean(f)

        # write an image that starts and ends inside sectors: the protection is cleared first, the rest of those
        # sectors is kept, and a whole 64 k block inside the range is erased in one go
        before = f.peek(0, 0x200000)
        img = bytes(rnd.randrange(256) for _ in range(0x15123))
        open(fn, 'wb').write(img)
        f.log.clear()
        run(f, args('wf', address=0x1F456, filename=fn))
        after = f.peek(0, 0x200000)
        assert after[0x1F456:0x1F456 + len(img)] == img
        assert after[:0x1F456] == before[:0x1F456] and after[0x1F456 + len(img):] == before[0x1F456 + len(img):]
        s = f.stat()
        assert 'BKSW' in f.log and s['sr'] == '0000' and int(s['erase64']) == 1, (f.log[:20], s)
        # the same image again: nothing to erase or write
        f.log.clear()
        run(f, args('wf', address=0x1F456, filename=fn))
        assert 'BKE' not in f.log and 'XW' not in f.log, f.log
        clean(f)

        # the bootloader guard
        expect_error(lambda: T.bk_ops(f, args('wf', address=0x10000, filename=fn)), 'bootloader', '0x11000')
        expect_error(lambda: T.bk_ops(f, args('es', address=0x0, size=0x1000)), 'bootloader', '--force-bootloader')
        assert f.peek(0, 0x11000) == after[:0x11000]
        # a full image from 0 (OpenBeken _QIO_, or a backup): written from 0x11000 on, the bootloader stays
        full = bytes(rnd.randrange(256) for _ in range(0x30000))
        open(fn, 'wb').write(full)
        t, text = run(f, args('wf', address=0, filename=fn))
        assert 'DIFFERENT' in text and f.peek(0, 0x11000) == after[:0x11000]
        assert f.peek(0x11000, 0x30000 - 0x11000) == full[0x11000:]
        # --force-bootloader writes it (BK7231N only)
        open(fn, 'wb').write(full[:0x2000])
        t, text = run(f, args('wf', address=0x3000, filename=fn, force_bootloader=True))
        assert 'WARNING' in text and f.peek(0x3000, 0x2000) == full[:0x2000]
        run(f, args('es', address=0x3000, size=0x1000, force_bootloader=True))
        assert f.peek(0x3000, 0x1000) == b'\xff' * 0x1000
        clean(f)

        # OTA packages are refused
        open(fn, 'wb').write(b'RBL\0' + bytes(200))
        expect_error(lambda: T.bk_ops(f, args('wf', address=0x11000, filename=fn)), 'OTA package')

        # UF2 of LibreTiny / ESPHome: the flasher parts at their partitions, the OTA part (device only) not
        app = bytes(rnd.randrange(256) for _ in range(0x9100))
        rblh = bytes(rnd.randrange(256) for _ in range(102))
        ota = bytes(rnd.randrange(256) for _ in range(0x800))
        fu = os.path.join(tmp, 'firmware.uf2')
        open(fu, 'wb').write(uf2([(app, 0, None, 'app'), (rblh, 0x110AD2, None, 'app'), (ota, 0, 'download', None)]))
        down = f.peek(0x12A000, 0x800)
        t, text = run(f, args('wf', address=0, filename=fu))
        assert f.peek(0x11000, len(app)) == app and f.peek(0x11000 + 0x110AD2, 102) == rblh, 'UF2 parts'
        assert f.peek(0x12A000, 0x800) == down, 'the OTA part must not be written'
        assert 'esphome 2025.11.0, board cb2s, family BK7231N' in text, text
        open(fu, 'wb').write(uf2([(app, 0, None, 'app')], family=0x675A40B0))
        expect_error(lambda: T.bk_ops(f, args('wf', address=0, filename=fu)), 'for BK7231T', 'BK7231N')
        clean(f)

        # erase everything but the bootloader; --run starts the application
        boot = f.peek(0, 0x11000)
        t, text = run(f, args('ea', run=True))
        assert f.peek(0x11000, 0x200000 - 0x11000) == b'\xff' * (0x200000 - 0x11000) and f.peek(0, 0x11000) == boot
        assert f.stat()['state'] == 'app'
        clean(f)

        # a target whose UART does not work above 460800: 115200, reported
        f.raw('SIM new maxbaud=460800')
        t, text = run(f, args('info'))
        assert t.baud == 115200 and '921600 baud did not work' in text, text
        # noise at 921600: every 4 k block is checked by CRC and read again when it differs
        f.raw('SIM new noise=20000')
        t, text = run(f, args('rf', address=0x11000, size=0x40000, filename=out))
        assert open(out, 'rb').read() == f.peek(0x11000, 0x40000)
        clean(f)

        # flash ids: one that is not in the list, and Macronix (quad enable in status bit 6 must stay)
        f.raw('SIM new id=a14015 sr1=0c')
        open(fn, 'wb').write(bytes(0x1000))
        expect_error(lambda: T.bk_ops(f, args('wf', address=0x20000, filename=fn)), 'not in the list', '--bk-sr')
        run(f, args('wf', address=0x20000, filename=fn, bk_sr=2))
        assert f.peek(0x20000, 0x1000) == bytes(0x1000)
        f.raw('SIM new id=c22315 sr1=4c')
        run(f, args('wf', address=0x20000, filename=fn))
        assert f.stat()['sr'] == '0040' and f.peek(0x20000, 0x1000) == bytes(0x1000), f.stat()
        # the flash id answer with the short length field
        f.raw('SIM new midlen=5')
        t, _ = run(f, args('info'))
        assert t.flash_id == 'eb6015'

        # BK7231T: its bootloader's protocol; its own area read at the address plus the flash size
        f.raw('SIM new rom=0')
        t, text = run(f, args('info'))
        assert not t.rom and t.chip == 'BK7231T' and 'bootloader protocol' in text, text
        t, _ = run(f, args('rf', address=0, size=0x20000, filename=out, twice=True))
        assert open(out, 'rb').read() == f.peek(0, 0x20000)
        img = bytes(rnd.randrange(256) for _ in range(0x2345))
        open(fn, 'wb').write(img)
        run(f, args('wf', address=0x11000, filename=fn))
        assert f.peek(0x11000, len(img)) == img
        open(fn, 'wb').write(full)
        boot = f.peek(0, 0x11000)
        run(f, args('wf', address=0, filename=fn))
        assert f.peek(0, 0x11000) == boot and f.peek(0x11000, len(full) - 0x11000) == full[0x11000:]
        expect_error(lambda: T.bk_ops(f, args('es', address=0, size=0x1000, force_bootloader=True)), 'BK7231T',
                     'bricks')
        clean(f)

        # failed links, with the reason
        f.raw('SIM new present=0')
        text = expect_error(lambda: T.bk_ops(f, args('info', bk_seconds=3)), 'nothing at all came back')
        print('no target ->', text[:110], '...')
        f.raw('SIM new reset=0')
        text = expect_error(lambda: T.bk_ops(f, args('info', bk_seconds=3)), 'its own firmware runs', 'CEN')
        print('no reset wired ->', text[:110], '...')
        f.raw('SIM new')
        run(f, args('info'))                            # and fine again
        clean(f)
    finally:
        f.close()
        shutil.rmtree(tmp, ignore_errors=True)
        os.remove(exe)
    print('BK72xx host tests passed')


if __name__ == '__main__':
    main()
