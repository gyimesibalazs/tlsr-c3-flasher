"""Puya PY32F002B type (MS32C001) over SWD: the host logic of tlsr_c3.py against tests/py32_sim.

The simulator runs the real src/swdflash.cpp against a bus-level model of the chip and answers the firmware's SWD
commands (SC, SAR, SM, SFAM, SHALT, SFE, SFX, SFW, SFC, SRUN) as text; the binary streams are replaced by their
text equivalents (SFW per write unit, SM per kbyte). Checks identification (with the known and an unknown DBGMCU
IDCODE, flash size from the first faulting kbyte), write + verify, page erase, mass erase, and that the option
bytes are never written. Needs g++.

    python3 tests/test_py32_host.py
"""
import base64
import os
import random
import shutil
import subprocess
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, ROOT)
import tlsr_c3 as T


def build_sim():
    out = os.path.join(tempfile.gettempdir(), 'py32_sim_%d' % os.getpid())
    subprocess.check_call(['g++', '-std=c++17', '-O1', '-I', os.path.join(HERE, 'py32_sim', 'stub'),
                           '-I', os.path.join(ROOT, 'src'), '-o', out, os.path.join(HERE, 'py32_sim', 'py32_sim.cpp'),
                           os.path.join(ROOT, 'src', 'swdflash.cpp')])
    return out


class SimFlasher:
    """The parts of tlsr_c3.Flasher the SWD code uses, talking to py32_sim --serve."""
    def __init__(self, exe):
        self.p = subprocess.Popen([exe, '--serve'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.log = []
        self.last_rate = 1e5

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

    def stream_write(self, dst, addr, data, unit=256, skip=True, label='Write', progress=True, window=3):
        assert dst == 'S' and addr % 4 == 0 and len(data) % 4 == 0
        pos = 0
        while pos < len(data):
            n = min(unit - (addr + pos) % unit, len(data) - pos)
            part = data[pos:pos + n]
            if not (skip and part == b'\xff' * n):
                self.cmd('SFW %d %s' % (addr + pos, part.hex()))
            pos += n

    def stream_read(self, src, addr, size, label='Read', progress=True):
        assert src == 'S'
        out = b''
        while len(out) < size:
            n = min(1024, size - len(out))
            out += base64.b64decode(self.cmd('SM 0 %d %d' % (addr + len(out), n)))
        return out

    def stats(self):
        return [int(x) for x in self.cmd('SIM stats').split()]

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def main():
    if not shutil.which('g++'):
        print('g++ not found: skipped')
        return
    f = SimFlasher(build_sim())
    try:
        lines = []
        # identification, with the output captured
        orig_print = print
        T.print = lambda *a, **k: lines.append(' '.join(str(x) for x in a))
        try:
            t = T.swd_info(f)
        finally:
            T.print = orig_print
        text = '\n'.join(lines)
        assert t.family == 'py32' and t.flash == 18 * 1024 and t.page == 128 and t.base == 0x08000000, text
        assert 'PY32F002B' in text and '18 kbytes' in text, text
        print('PASS  info: %s' % next(l for l in lines if l.startswith('Chip:')))
        # an unknown IDCODE (the MS32C001's own) is recognised from the flash controller and the factory bytes
        f.cmd('SIM idcode 0x12345678')
        t2 = T.swd_info(f, quiet=True)
        assert t2.family == 'py32' and 'MS32C001' in t2.name and t2.flash == 18 * 1024, t2.name
        print('PASS  info with an unknown IDCODE: %s' % t2.name)
        # write + verify a firmware image of odd length
        rnd = random.Random(5)
        img = bytes(rnd.randrange(256) for _ in range(6361))
        data = img + b'\xff' * (-len(img) % 4)
        T.swd_write(f, t, 0x08000000, data)
        assert T.swd_verify(f, 0x08000000, data) is None
        assert f.stream_read('S', 0x08000000, len(data)) == data
        print('PASS  wf: %d bytes written and verified by CRC' % len(img))
        # page erase of one page in the middle
        f.cmd('SFAM py32')
        f.cmd('SFE %d' % 0x08000400)
        back = f.stream_read('S', 0x08000380, 0x180)
        assert back[:0x80] == data[0x380:0x400] and back[0x80:0x100] == b'\xff' * 0x80 and back[0x100:] == data[0x480:0x500]
        print('PASS  es: one 128-byte page erased, its neighbours kept')
        # rewrite over a written image (erase first, as wf does)
        img2 = bytes(rnd.randrange(256) for _ in range(1024))
        T.swd_write(f, t, 0x08001000, img2)
        assert T.swd_verify(f, 0x08001000, img2) is None
        print('PASS  wf over old data at 0x08001000')
        # mass erase
        f.cmd('SHALT reset')
        f.cmd('SFX', timeout=60)
        assert f.stream_read('S', 0x08000000, 18 * 1024) == b'\xff' * (18 * 1024)
        print('PASS  ea: main flash erased')
        option_writes, bad_timing, pg_errors, stray, mass, halted = f.stats()
        assert option_writes == 0 and bad_timing == 0 and pg_errors == 0 and stray == 0 and mass == 1, f.stats()
        print('PASS  no option byte writes, flash timing always loaded, page buffer used correctly')
        f.cmd('SRUN')
        print('all passed')
    finally:
        f.close()


if __name__ == '__main__':
    main()
