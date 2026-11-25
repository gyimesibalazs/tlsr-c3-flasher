"""WCH CH32V003 mode: the host logic of tlsr_c3.py against the simulated chip of tests/wch_sim.

The simulator runs the real src/wch.cpp and src/wchflash.cpp (the line layer on stubbed RMT channels, the debug
module and flash code, and the parser of the W commands) against a model of the chip, so these tests go from the
host's commands down to the SWIO waveform. Only the binary streams are replaced by their text equivalents here
(test_streams.py checks those). Needs g++.

    python3 tests/test_wch_host.py
"""
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
    out = os.path.join(tempfile.gettempdir(), 'wch_sim_%d' % os.getpid())
    subprocess.check_call(['g++', '-std=c++17', '-O1', '-I', os.path.join(HERE, 'wch_sim', 'stub'),
                           '-I', os.path.join(ROOT, 'src'), '-o', out, os.path.join(HERE, 'wch_sim', 'wch_sim.cpp'),
                           os.path.join(ROOT, 'src', 'wch.cpp'), os.path.join(ROOT, 'src', 'wchflash.cpp')])
    return out


class SimFlasher:
    """The parts of tlsr_c3.Flasher the WCH code uses, talking to wch_sim --serve."""
    def __init__(self, exe):
        self.p = subprocess.Popen([exe, '--serve'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.version = 'tlsr-c3-flasher 0.22'
        self.log = []
        self.ser = types.SimpleNamespace(reset_input_buffer=lambda: None)
        self.rxbuf = b''
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

    def stream_read(self, src, addr, size, label='Read', progress=True):
        assert src == 'C' and addr % 4 == 0 and size % 4 == 0
        self.log.append('XR')
        return T.wch_mem(self, addr, size)

    def stream_write(self, dst, addr, data, unit=256, skip=True, label='Write', progress=True, window=3):
        assert dst == 'C' and unit == 64 and addr % 4 == 0 and len(data) % 4 == 0
        self.log.append('XW')
        flash = addr < T.WCH_RAM                        # user flash and boot area; RAM is written as memory
        pos = 0
        while pos < len(data):
            n = min(unit - (addr + pos) % unit, len(data) - pos)
            part = data[pos:pos + n]
            if not (skip and part == b'\xff' * n):
                self.cmd(('WFW %d %s' if flash else 'WMW %d %s') % (addr + pos, part.hex()))
            pos += n

    def stop(self):
        pass

    def peek(self, addr, size):
        return bytes.fromhex(self.raw('SIM peek %d %d' % (addr, size))[3:])

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def args(op, **kw):
    a = dict(op=op, wch_off=300, wch_window=100, external_power=False, wch_timing='', save_options='',
             write_options=False, yes_erase_all=False, run=False, no_verify=False, twice=False)
    a.update(kw)
    return types.SimpleNamespace(**a)


def expect_error(fn, *words):
    try:
        fn()
    except T.FlasherError as e:
        text = str(e)
        for w in words:
            assert w in text, 'expected %r in: %s' % (w, text)
        return text
    raise AssertionError('no error')


def main():
    if not shutil.which('g++'):
        print('g++ not found: skipped')
        return
    exe = build_sim()
    f = SimFlasher(exe)
    tmp = tempfile.mkdtemp()
    try:
        # info on a chip whose firmware takes the SWIO pin 1.5 ms after start: caught at power-up
        f.raw('SIM new take=1500')
        t = T.wch_main(f, args('info'))
        assert t.flash == 16384 and not t.rdprt and t.options == T.WCH_FACTORY_OPTIONS

        # full read, twice, against the model
        out = os.path.join(tmp, 'dump.bin')
        T.wch_main(f, args('rf', address=0, size=16384, filename=out, twice=True))
        assert open(out, 'rb').read() == f.peek(T.WCH_FLASH, 16384)

        # write an image that starts and ends inside pages: the rest of those pages is kept
        before = f.peek(T.WCH_FLASH, 16384)
        img = bytes(random.Random(1).randrange(256) for _ in range(3001))
        fn = os.path.join(tmp, 'img.bin')
        open(fn, 'wb').write(img)
        T.wch_main(f, args('wf', address=0x123, filename=fn))
        after = f.peek(T.WCH_FLASH, 16384)
        assert after[0x123:0x123 + 3001] == img
        assert after[:0x123] == before[:0x123] and after[0x123 + 3001:] == before[0x123 + 3001:]
        # the same image again: nothing to erase
        f.log.clear()
        T.wch_main(f, args('wf', address=0x08000123, filename=fn))
        assert 'WFE' not in f.log and 'XW' not in f.log
        assert f.raw('SIM stat').startswith('OK violations=0'), f.raw('SIM stat')

        # erase a range, then everything
        T.wch_main(f, args('es', address=0x100, size=0x80))
        assert f.peek(T.WCH_FLASH + 0x100, 0x80) == b'\xff' * 0x80
        assert f.peek(T.WCH_FLASH + 0xc0, 0x40) == after[0xc0:0x100]           # the neighbours stay
        assert f.peek(T.WCH_FLASH + 0x180, 0x40) == after[0x180:0x1c0]
        T.wch_main(f, args('ea'))
        assert f.peek(T.WCH_FLASH, 16384) == b'\xff' * 16384

        # boot area, and RAM
        boot = bytes(range(256)) * 2
        open(fn, 'wb').write(boot)
        T.wch_main(f, args('wf', address=T.WCH_BOOT + 0x40, filename=fn))
        assert f.peek(T.WCH_BOOT + 0x40, 512) == boot
        T.wch_main(f, args('wf', address=T.WCH_RAM + 0x10, filename=fn))
        assert f.peek(T.WCH_RAM + 0x10, 512) == boot

        # option bytes: only with --write-options and with valid complements
        ob = bytes([0xa5, 0x5a, 0x97, 0x68, 0x42, 0xbd, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00])
        open(fn, 'wb').write(ob)
        expect_error(lambda: T.wch_main(f, args('wf', address=T.WCH_OPT, filename=fn)), '--write-options')
        bad = bytearray(ob)
        bad[5] = 0
        open(fn, 'wb').write(bad)
        expect_error(lambda: T.wch_main(f, args('wf', address=T.WCH_OPT, filename=fn, write_options=True)),
                     'complement')
        open(fn, 'wb').write(ob)
        saved = os.path.join(tmp, 'opt.bin')
        T.wch_main(f, args('wf', address=T.WCH_OPT, filename=fn, write_options=True, save_options=saved))
        assert f.peek(T.WCH_OPT, 16) == ob and open(saved, 'rb').read() == T.WCH_FACTORY_OPTIONS

        # --run: the firmware starts and takes the pin again
        T.wch_main(f, args('info', run=True))
        f.raw('SIM wait 5')
        assert 'running=1' in f.raw('SIM stat') and 'pin_taken=1' in f.raw('SIM stat')
        T.wch_main(f, args('info'))                     # caught again at the next power-up

        # a read protected chip: reading and writing refused, unlock needs the flag and erases everything
        f.raw('SIM new protect take=1500')
        t = T.wch_main(f, args('info'))
        assert t.rdprt
        expect_error(lambda: T.wch_main(f, args('rf', address=0, size=64, filename=out)), 'read protected')
        open(fn, 'wb').write(img)
        expect_error(lambda: T.wch_main(f, args('wf', address=0, filename=fn)), 'read protected', 'unlock')
        expect_error(lambda: T.wch_main(f, args('unlock')), '--yes-erase-all', 'ERASES')
        assert f.peek(T.WCH_FLASH, 16) != b'\xff' * 16
        T.wch_main(f, args('unlock', yes_erase_all=True, save_options=saved))
        assert open(saved, 'rb').read()[0] != 0xa5        # the protected options were saved first
        t = T.wch_main(f, args('info'))
        assert not t.rdprt and f.peek(T.WCH_FLASH, 16384) == b'\xff' * 16384
        assert f.peek(T.WCH_OPT, 16) == T.WCH_FACTORY_OPTIONS
        T.wch_main(f, args('wf', address=0, filename=fn))
        assert f.peek(T.WCH_FLASH, len(img)) == img

        # failed attaches, with the reason
        f.raw('SIM new pins=0 take=1500')
        f.raw('SIM wait 10')
        text = expect_error(lambda: T.wch_main(f, args('info', external_power=True)), 'never went low',
                            'external power')
        print('external power, pin taken ->', text.splitlines()[0][:120], '...')
        f.raw('SIM new pins=0')                         # the same chip, firmware leaves SWIO alone: works
        T.wch_main(f, args('info', external_power=True))
        f.raw('SIM new present=0')
        expect_error(lambda: T.wch_main(f, args('info')), 'nothing answers')
        f.raw('SIM new pins=0 off=1')
        expect_error(lambda: T.wch_main(f, args('info', external_power=True)), 'stays low')
        f.raw('SIM new take=60')
        expect_error(lambda: T.wch_main(f, args('info')), 'not caught in time')

        # timing option
        f.raw('SIM new')
        T.wch_main(f, args('info', wch_timing='300,1100,400,1600,600,5000'))
        assert f.cmd('WT') == '300 1100 400 1600 600 5000'
        expect_error(lambda: f.cmd('WT 300 400 400 1600 600 5000'), 'timing')
        assert f.raw('SIM stat').startswith('OK violations=0'), f.raw('SIM stat')
    finally:
        f.close()
        shutil.rmtree(tmp, ignore_errors=True)
        os.remove(exe)
    print('WCH host tests passed')


if __name__ == '__main__':
    main()
