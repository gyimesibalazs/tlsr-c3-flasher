"""A model of a 25-series SPI NOR flash at the level of CS-low transactions (bytes out, then bytes in), for the LN882H
loader's tests: the native build (tests/ln_sim.py) and the emulated binary (tests/test_ln_emu.py).

Like a real part: the write enable latch (WEL) is needed for program / erase / status write and is cleared by them;
WIP stays set for a few status polls after them and any other command meanwhile is a violation; page program only
clears bits and wraps inside its 256-byte page; block protection (any BP bit, or CMP) makes program and erase do
nothing, silently; a status write with one byte on a two-byte part clears the second byte (as older parts do)."""
import os


class Nor:
    def __init__(self, size=0x200000, jedec=0xEB6015, sr=0x0200, sr_bytes=2, data=None, busy_polls=2):
        self.size = size
        self.mem = data if isinstance(data, bytearray) else bytearray(data if data is not None else os.urandom(size))
        self.jedec = jedec
        self.sr1, self.sr2 = sr & 0xFF, sr >> 8         # 0x0200: QE set, as on modules that run the cache in quad
        self.sr_bytes = sr_bytes
        self.uid = bytes(range(0x10, 0x20))
        self.otp = bytearray(b'\xff' * 0x400)
        self.otp[0x100:0x106] = bytes.fromhex('c0f5a1020304')
        self.wel = False
        self.wip = 0
        self.busy_polls = busy_polls
        self.violations = []
        self.counts = dict(program=0, erase4k=0, erase64k=0, chip=0, srwrite=0, read=0)
        self.log = []

    @property
    def protected(self):
        return bool(self.sr1 & 0x7C) or bool(self.sr2 & 0x40)

    def status1(self):
        return (self.sr1 & 0xFC) | (2 if self.wel else 0) | (1 if self.wip else 0)

    def _busy(self):
        self.wip = self.busy_polls
        self.wel = False

    def transfer(self, tx, nrx):
        tx = bytes(tx)
        op = tx[0]
        self.log.append((op, len(tx), nrx))
        if self.wip and op != 0x05:
            self.violations.append('command %02x while busy' % op)
            return b'\xff' * nrx
        addr = int.from_bytes(tx[1:4], 'big') if len(tx) >= 4 else 0

        def want(ntx):
            if len(tx) != ntx:
                self.violations.append('command %02x with %d bytes out' % (op, len(tx)))
        if op == 0x05:
            want(1)
            v = self.status1()
            if self.wip:
                self.wip -= 1
            return bytes([v]) * nrx
        if op == 0x35:
            want(1)
            return bytes([self.sr2]) * nrx
        if op == 0x15:
            want(1)
            return b'\x00' * nrx
        if op == 0x9F:
            want(1)
            return self.jedec.to_bytes(3, 'big')[:nrx].ljust(nrx, b'\xff')
        if op == 0x4B:
            want(5)
            return self.uid[:nrx].ljust(nrx, b'\xff')
        if op == 0x48:
            want(5)
            return bytes(self.otp[addr % len(self.otp):][:nrx]).ljust(nrx, b'\xff')
        if op == 0xAB:
            return b'\xff' * nrx
        if op == 0x03:
            want(4)
            self.counts['read'] += 1
            a = addr % self.size
            out = bytes(self.mem[a:a + nrx])
            return out + bytes(self.mem[:nrx - len(out)])     # the address wraps at the end of the flash
        if nrx:
            self.violations.append('command %02x with %d bytes in' % (op, nrx))
        if op == 0x06:
            want(1)
            self.wel = True
            return b''
        if op == 0x04:
            want(1)
            self.wel = False
            return b''
        if op in (0x02, 0x20, 0xD8, 0x60, 0xC7, 0x01) and not self.wel:
            return b''                               # ignored without the latch
        if op == 0x02:
            if len(tx) < 5:
                self.violations.append('page program without data')
            if not self.protected:
                base = addr - addr % 256
                for i, b in enumerate(tx[4:]):
                    a = base + (addr + i) % 256       # wraps inside the page
                    self.mem[a % self.size] &= b
            self.counts['program'] += 1
            self._busy()
            return b''
        if op in (0x20, 0xD8):
            want(4)
            n = 0x1000 if op == 0x20 else 0x10000
            if addr % n:
                self.violations.append('erase %02x at unaligned 0x%x (the part would erase the whole unit)' %
                                       (op, addr))
            if not self.protected:
                base = addr - addr % n
                self.mem[base:base + n] = b'\xff' * n
            self.counts['erase4k' if op == 0x20 else 'erase64k'] += 1
            self._busy()
            return b''
        if op in (0x60, 0xC7):
            want(1)
            if not self.protected:
                self.mem[:] = b'\xff' * self.size
            self.counts['chip'] += 1
            self._busy()
            return b''
        if op == 0x01:
            if len(tx) not in (2, 3):
                self.violations.append('status write with %d bytes' % (len(tx) - 1))
            self.sr1 = tx[1] & 0xFC
            if self.sr_bytes == 2:
                self.sr2 = tx[2] if len(tx) > 2 else 0
            self.counts['srwrite'] += 1
            self._busy()
            return b''
        self.violations.append('unknown command %02x' % op)
        return b''
