"""The LN882H loader as built (the image embedded in tlsr_c3.py) run in the unicorn CPU emulator (Cortex-M4), against
register-level models of what it touches: the clock tree (crystal, PLL, AHB and APB0 dividers with their update
bits), UART0 with its clock gate, reset and the pin functions of A2 / A3, the QSPI controller (DesignWare SSI) with the
SPI NOR flash of ln_nor_model.py behind it, the flash cache, the chip reset, SysTick, the NVIC and VTOR. This checks
what the native test cannot: the boot header and vector table, the start-up code, the linker script and the
register-level driver. tlsr_c3.py's own frame code talks to it (LnLink over a port that runs the emulator).

The register models follow the SDK's register descriptions. UART0's bit rate is computed from the modelled APB0 clock
and its divider (BRR = 16 * APB0 / baud at 8x oversampling, 8 * APB0 / baud at 16x, the SDK's formula); a byte sent at
a rate more than 3 % off the host's arrives as 0xFF, a byte received at a wrong rate is lost with a framing error. Bytes
only pass when UART0's clock is on, it is out of reset and enabled, and A2 / A3 are given to it. The ROM's hand-over
state is not known; the main run starts from one where the ROM's UART divider does not fit the clock it hands over
with (what the first real run looked like), others from a ROM that leaves everything fitting and from a PLL that is
off. The QSPI model counts as violations what a DesignWare SSI would not do as intended (configuration written while
enabled, data while disabled, a transfer without the slave selected, a frame size or mode other than 8-bit standard
SPI, receive data not read, an SPI clock above 25 MHz). Timing is not modelled: FIFOs never fill and the flash
answers at once.

Needs the unicorn package (pip install unicorn); skipped with a message without it."""
import os, re, struct, sys, zlib
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tlsr_c3 as T
from ln_nor_model import Nor

try:
    import unicorn as U
    from unicorn import arm_const as A
except ImportError:
    print('LN882H loader emulation: SKIPPED, the unicorn package is not installed (pip install unicorn)')
    sys.exit(0)

ROM_SP, ROM_VTOR = 0x20047F00, 0x00010000
XTAL = 40e6
BRR_115200 = 5555                         # what the SDK's formula gives for 115200 at a 40 MHz APB, 8x oversampling
# hand-over states of the ROM: clock source, PLL control, AHB divider - 1, APB0 divider - 1, UART divider
ROM_STATES = {
    # on the PLL at 160 MHz with APB0 undivided, but the divider of a 40 MHz APB0: the UART runs 4x too fast
    'misfit': dict(clk_sel=1, pll_ctrl=0x8008, awo_div=0, cmp_div=0, brr=BRR_115200),
    # the crystal, APB0 40 MHz, the divider for it: the ROM's UART still fits
    'fits': dict(clk_sel=0, pll_ctrl=0x8008, awo_div=0, cmp_div=0, brr=BRR_115200),
    # the PLL off: the loader has to stay on the crystal
    'pll_off': dict(clk_sel=0, pll_ctrl=0x0008, awo_div=0, cmp_div=0, brr=BRR_115200),
}


class Chip:
    def __init__(self, image, nor, rom='misfit'):
        self.nor = nor
        self.uc = uc = U.Uc(U.UC_ARCH_ARM, U.UC_MODE_THUMB | U.UC_MODE_MCLASS)
        try:
            uc.ctl_set_cpu_model(A.UC_CPU_ARM_CORTEX_M4)
        except (AttributeError, U.UcError):
            pass
        uc.mem_map(0x20000000, 0x60000)                       # SRAM0..SRAM3
        uc.mem_write(0x20000000, image)
        st = ROM_STATES[rom]
        self.rx, self.tx = bytearray(), bytearray()
        self.host_baud = 115200
        self.send_baud = 0                                     # the host sends at another rate than it receives
        self.garbled_out = self.lost_in = 0
        self.violations, self.events = [], []
        # SYSC_CMP: UART0 and GPIOA clocks on (the ROM uses them), QSPI, cache clocks and flash pads off, nothing in
        # reset, A2 / A3 given to UART0
        self.cmp = {0x00: 0, 0x04: st['cmp_div'], 0x10: 0x410, 0x14: 0xFFFFFFFF, 0x18: 0, 0x1C: 0x0C,
                    0x20: 0x03020000}
        self.awo = {0x00: 0, 0x04: st['awo_div'], 0x0C: 0x1F, 0x10: st['clk_sel'], 0x14: 0, 0x70: st['pll_ctrl'],
                    0x74: 0x200, 0x9C: 0}
        self.ahb_div, self.apb_div = st['awo_div'] + 1, st['cmp_div'] + 1     # the dividers in effect
        self.gpioa_ddr = 0
        self.uart = dict(cr1=0x802D, cr2=0, cr3=0, brr=st['brr'], fe=0)
        self.cache = 1                                         # the cache on, as after a boot from flash
        self.q = dict(ctrlr0=0x70000, ctrlr1=0, ssienr=0, ser=0, baudr=2, imr=0x3F, rsd=0)
        self.q_tx, self.q_rx, self.q_done = bytearray(), bytearray(), False
        self.cvr = 0
        self.vtor = ROM_VTOR
        self.rebooted = False
        uc.mmio_map(0x40000000, 0x10000, self.rd_apb, None, self.wr_apb, None)
        uc.mmio_map(0x40100000, 0x1000, self.rd_awo, None, self.wr_awo, None)
        uc.mmio_map(0x40200000, 0x1000, self.rd_qspi, None, self.wr_qspi, None)
        uc.mmio_map(0x20200000, 0x1000, self.rd_cache, None, self.wr_cache, None)
        uc.mmio_map(0xE000E000, 0x1000, self.rd_scs, None, self.wr_scs, None)
        uc.hook_add(U.UC_HOOK_MEM_INVALID, self.bad_access)
        # the ROM's hand-over as far as it is known: its own stack, interrupts on, then a jump to the reset vector
        reset = struct.unpack_from('<I', image, 0x104)[0]
        uc.reg_write(A.UC_ARM_REG_SP, ROM_SP)
        self.pc = reset

    def bad_access(self, uc, access, addr, size, value, _):
        self.violations.append('access to unmapped 0x%08x at pc 0x%08x' % (addr, uc.reg_read(A.UC_ARM_REG_PC)))
        return False

    # ---- the clock tree: core = crystal, or the PLL at crystal * multiplier / 2; AHB = core / div; APB0 = AHB / div
    def core_hz(self):
        if self.awo[0x10] & 1:
            if not self.awo[0x70] & 0x8000:
                self.violations.append('the core switched to the PLL while it is off')
                self.uc.emu_stop()
            return XTAL * (self.awo[0x70] & 0x1F) / 2
        return XTAL

    def ahb_hz(self):
        return self.core_hz() / self.ahb_div

    def apb_hz(self):
        return self.ahb_hz() / self.apb_div

    # ---- UART0 0x40002000: the bit rate from APB0 and the divider; the pins A2 (TX) and A3 (RX)
    def uart_baud(self):
        brr = self.uart['brr'] & 0x7FFFF
        return (16 if self.uart['cr1'] & 0x8000 else 8) * self.apb_hz() / brr if brr else 0

    def uart_fits(self, baud=0):
        return abs(self.uart_baud() / (baud or self.host_baud) - 1) < 0.03

    def uart_live(self, dir_bit, pin, func):
        return (self.cmp[0x10] & 0x400 and self.cmp[0x14] & 0x400 and self.uart['cr1'] & 1 and
                self.uart['cr1'] & dir_bit and self.cmp[0x1C] & (1 << pin) and not self.gpioa_ddr & (1 << pin) and
                (self.cmp[0x20] >> (8 * pin)) & 0x3F == func)

    def rx_ready(self):
        """The next byte from the host as UART0 receives it; a byte at a wrong rate is lost with a framing error."""
        while self.rx:
            if not self.uart_live(4, 3, 3):
                self.rx.pop(0)
                self.lost_in += 1
            elif not self.uart_fits(self.send_baud):
                self.rx.pop(0)
                self.lost_in += 1
                self.uart['fe'] = 1
            else:
                return True
        return False

    def rd_apb(self, uc, off, size, _):
        if 0x2000 <= off < 0x3000:
            if not self.cmp[0x10] & 0x400:
                self.violations.append('UART0 read with its clock off')
            r = off - 0x2000
            if r in (0x00, 0x04, 0x08, 0x0C):
                return self.uart[{0: 'cr1', 4: 'cr2', 8: 'cr3', 12: 'brr'}[r]]
            if r == 0x1C:
                return 0x40 | 0x80 | (0x20 if self.rx_ready() else 0) | (2 if self.uart['fe'] else 0)  # TC TXE RXNE FE
            if r == 0x24:
                return self.rx.pop(0) if self.rx_ready() else 0
            if r == 0x2C:
                return 0                                           # FIFO levels: all sent
            return 0
        if off == 0xC018:
            return self.gpioa_ddr
        return self.cmp.get(off, 0)

    def wr_apb(self, uc, off, size, value, _):
        if 0x2000 <= off < 0x3000:
            if not self.cmp[0x10] & 0x400:
                self.violations.append('UART0 written with its clock off')
            r = off - 0x2000
            if r in (0x00, 0x04, 0x08, 0x0C):
                self.uart[{0: 'cr1', 4: 'cr2', 8: 'cr3', 12: 'brr'}[r]] = value
                if r == 0x0C:
                    self.events.append(('brr', value))
            elif r == 0x20:
                if value & 2:
                    self.uart['fe'] = 0
            elif r == 0x28:
                if not self.uart_live(8, 2, 2):
                    self.violations.append('UART0: a byte sent while it or A2 is not set up')
                elif self.uart_fits():
                    self.tx.append(value & 0xFF)
                else:
                    self.tx.append(0xFF)
                    self.garbled_out += 1
            return
        if off == 0xC018:
            self.gpioa_ddr = value & 0xFFFF
            return
        if off == 0x14 and not value & 0x400:
            self.uart = dict(cr1=0, cr2=0, cr3=0, brr=0, fe=0)       # UART0 held in reset: its reset values
            self.events.append('uart0 reset')
        if off == 0x00:
            if value & 1:
                self.apb_div = (self.cmp[0x04] & 0xF) + 1
            value = 0                                              # the update bits read back as 0
        self.cmp[off] = value

    # ---- SYSC_AWO: the clock source, the PLL, the AHB divider, the chip reset
    def rd_awo(self, uc, off, size, _):
        return self.awo.get(off, 0)

    def wr_awo(self, uc, off, size, value, _):
        if off == 0x0C and not value & 0x10:
            self.rebooted = True
            uc.emu_stop()
            return
        if off == 0x00:
            if value & 1:
                self.ahb_div = (self.awo[0x04] & 0xF) + 1
            value = 0
        self.awo[off] = value
        if off == 0x10:
            self.core_hz()                                         # a switch to a PLL that is off stops the core

    # ---- the flash cache
    def rd_cache(self, uc, off, size, _):
        return self.cache if off == 0 else 0

    def wr_cache(self, uc, off, size, value, _):
        if off == 0:
            self.cache = value & 1

    # ---- SysTick and the SCB
    def rd_scs(self, uc, off, size, _):
        if off == 0x18:
            self.cvr = (self.cvr - 50000) & 0xFFFFFF                # time passes as it is read
            return self.cvr
        if off == 0xD08:
            return self.vtor
        return 0

    def wr_scs(self, uc, off, size, value, _):
        if off == 0x18:
            self.cvr = 0
        elif off == 0xD08:
            self.vtor = value
        elif off == 0xD0C:
            self.violations.append('core reset (AIRCR) instead of the chip reset')
            uc.emu_stop()

    # ---- QSPI: DesignWare SSI
    def q_end(self):
        """SSIENR 1 -> 0: CS goes high, the transfer is over."""
        tmod = (self.q['ctrlr0'] >> 8) & 3
        if tmod == 1 and self.q_tx:
            if self.q['ser'] != 1:
                self.violations.append('QSPI: transmit without the slave selected')
            self.nor.transfer(bytes(self.q_tx), 0)
        elif tmod == 3 and self.q_tx and not self.q_done:
            self.violations.append('QSPI: EEPROM read %02x ended before data was read' % self.q_tx[0])
        if self.q_rx:
            self.violations.append('QSPI: %d received bytes not read' % len(self.q_rx))
        self.q_tx, self.q_rx, self.q_done = bytearray(), bytearray(), False

    def rd_qspi(self, uc, off, size, _):
        if off == 0x28:
            return 0x06 | (0x08 if self.q_rx or self.q_pending() else 0)   # TFE, TFNF, RFNE; never busy
        if off == 0x60:
            if self.q_pending():
                self.q_start_read()
            if not self.q_rx:
                self.violations.append('QSPI: DR read with nothing received')
                return 0
            return self.q_rx.pop(0)
        return {0x00: self.q['ctrlr0'], 0x04: self.q['ctrlr1'], 0x08: self.q['ssienr'], 0x10: self.q['ser'],
                0x14: self.q['baudr'], 0x2C: self.q['imr'], 0xF0: self.q['rsd']}.get(off, 0)

    def q_pending(self):
        return (self.q['ctrlr0'] >> 8) & 3 == 3 and self.q['ssienr'] and self.q_tx and not self.q_done

    def q_start_read(self):
        if self.q['ser'] != 1:
            self.violations.append('QSPI: EEPROM read without the slave selected')
        self.q_done = True
        self.q_rx = bytearray(self.nor.transfer(bytes(self.q_tx), self.q['ctrlr1'] + 1))

    def wr_qspi(self, uc, off, size, value, _):
        names = {0x00: 'ctrlr0', 0x04: 'ctrlr1', 0x14: 'baudr', 0xF0: 'rsd'}
        if off in names:
            if self.q['ssienr']:
                self.violations.append('QSPI: %s written while enabled' % names[off])
            self.q[names[off]] = value
            if off == 0x00 and value & ~0x300 != 0x70000:
                self.violations.append('QSPI: CTRLR0 0x%x is not 8-bit standard SPI mode 0' % value)
            if off == 0x14 and (value < 2 or value & 1 or self.ahb_hz() / value > 25e6):
                self.violations.append('QSPI: clock divider %d at a %.0f MHz AHB' % (value, self.ahb_hz() / 1e6))
        elif off == 0x08:
            if self.q['ssienr'] and not value & 1:
                self.q_end()
            self.q['ssienr'] = value & 1
        elif off == 0x10:
            self.q['ser'] = value
        elif off == 0x2C:
            self.q['imr'] = value
        elif off == 0x60:
            if not self.q['ssienr']:
                self.violations.append('QSPI: DR written while disabled')
            self.q_tx.append(value & 0xFF)
        else:
            self.violations.append('QSPI: write to 0x%x' % off)

    # ---- running
    def run(self, count=200000):
        if self.rebooted:
            return
        try:
            self.uc.emu_start(self.pc | 1, 0xFFFFFFFF, count=count)
        except U.UcError as e:
            self.violations.append('emulation stopped at 0x%08x: %s' % (self.uc.reg_read(A.UC_ARM_REG_PC), e))
            raise AssertionError(self.violations)
        self.pc = self.uc.reg_read(A.UC_ARM_REG_PC)
        m = re.search(rb'C3LN fault[ -~]*', self.tx)
        assert not m, m.group(0)

    def until(self, pattern, budget=200):
        for _ in range(budget):
            m = re.search(pattern, self.tx)
            if m:
                return m
            self.run()
        raise AssertionError('not seen: %r; output %r' % (pattern, bytes(self.tx[-200:])))


class Port:
    """The serial port LnLink reads and writes: writing feeds UART0, reading runs the emulator."""
    def __init__(self, chip):
        self.chip = chip

    def write(self, data):
        self.chip.rx += data
        return len(data)

    @property
    def in_waiting(self):
        return len(self.chip.tx)

    def read(self, n=1):
        for _ in range(20):
            if self.chip.tx or self.chip.rebooted:
                break
            self.chip.run()
        d = bytes(self.chip.tx[:n])
        del self.chip.tx[:n]
        return d


def start(rom):
    """A chip with the given ROM hand-over state, run until the loader's start is complete."""
    nor = Nor(size=0x200000, jedec=0xC86515, sr=0x0200)
    chip = Chip(T.LN_LOADER, nor, rom)
    m = chip.until(rb'\r\nC3LN 2 init 1 2 3 4 5 ready\r\n')
    assert not chip.violations, chip.violations
    return chip, nor, m


# the PLL off: the crystal kept, APB0 undivided (40 MHz), the QSPI clock divided by 4
chip, nor, m = start('pll_off')
assert chip.awo[0x10] & 1 == 0 and chip.core_hz() == 40e6 and chip.apb_hz() == 40e6 and chip.q['baudr'] == 4, (
    chip.awo, chip.q)
print('ROM hand-over with the PLL off: the crystal kept, APB0 40 MHz, UART0 at 115200, QSPI clock / 4: ok')

# a ROM that leaves a UART divider that fits its clock: its hello comes through at 115200
chip, nor, m = start('fits')
assert chip.tx.startswith(b'\r\nC3LN 2 rom clock\r\n\r\nC3LN 2 init 1'), bytes(chip.tx[:60])
print('ROM hand-over that fits: "rom clock" readable before the loader sets its own clocks: ok')

# what the first real run may have met: the ROM's divider does not fit the clock it hands over with
chip, nor, m = start('misfit')
hello = chip.tx[:chip.tx.find(b'\r\nC3LN 2 init')]
assert hello and set(hello) == {0xFF} and chip.garbled_out == len(hello), bytes(hello)
assert chip.awo[0x10] & 1 and chip.awo[0x70] & 0x1F == 8 and chip.ahb_div == 1 and chip.apb_div == 4, chip.awo
assert (chip.core_hz(), chip.apb_hz(), chip.uart['brr']) == (160e6, 40e6, 5556), (chip.core_hz(), chip.apb_hz())
assert (chip.awo[0x14] >> 8) & 3 == 1 and (chip.awo[0x9C] >> 12) & 7 == 7, chip.awo     # band gap trim, PLL LDO
assert 'uart0 reset' in chip.events and chip.uart['cr1'] == 0x800D and chip.uart['cr2'] == chip.uart['cr3'] == 0
sp = chip.uc.reg_read(A.UC_ARM_REG_SP)
assert 0x20000000 < sp <= 0x20020000, hex(sp)
assert chip.vtor == 0x20000100 and chip.cache == 0 and chip.cmp[0x10] & 2 and chip.cmp[0x18] & 0x20, \
    (hex(chip.vtor), chip.cache, chip.cmp)
assert chip.q['baudr'] == 16 and chip.q['rsd'] == 1
print('ROM hand-over that does not fit: "rom clock" garbled (%d bytes), then PLL 160 MHz, APB0 40 MHz, UART0 reset and '
      'set up (BRR 5556, 8x), banner and all init stages readable at 115200; VTOR 0x20000100, own stack (SP 0x%08x), '
      'cache off, QSPI clock / 16' % (len(hello), sp))
del chip.tx[:m.end()]

# nothing sent: the wait line every 200 ms; garbage at a wrong speed counted as receive errors
chip.until(rb'\r\nC3LN 2 wait rx 0 err 0 ready\r\n', budget=400)
del chip.tx[:]
chip.send_baud = 230400                  # the host's bytes come at the wrong rate
chip.rx += b'version\r\n'
chip.until(rb'C3LN 2 wait rx 0 err [1-9][0-9]* ready\r\n', budget=400)
chip.send_baud = 0
print('no valid line yet: the wait line every 200 ms; bytes at a wrong rate show as receive errors: ok')
del chip.tx[:]

link = T.LnLink(Port(chip))
answer = link.link(5.0)
assert answer == 'C3LN 2 ready' and link.own, answer
t = T.LnTarget()
T.ln_own_info(link, t, answer)
i = t.info
assert (t.flash_id, t.size, i['sr'], t.mac) == (0xC86515, 0x200000, b'\x00\x02\x00', 'C0:F5:A1:02:03:04'), i
assert i['uid'] == nor.uid and i['stage'] == 6 and i['error'] == 0 and i['rx_errors'] >= 1
assert (i['pll'], i['core_hz'], i['apb_hz']) == (1, 160000000, 40000000), i
r = i['regs']
assert (r['uart_brr'], r['uart_cr1'], r['vtor'], r['sp'], r['cache_en'], r['clk_gate'], r['io_en']) == (
    BRR_115200, 0x802D, ROM_VTOR, ROM_SP, 1, 0x410, 0), r
assert (r['clk_sel'], r['pll_ctrl'], r['awo_div'], r['cmp_div'], r['func_sel0'], r['func_en']) == (
    1, 0x8008, 0, 0, 0x03020000, 0x0C), r
assert r['ctrl'] == 0, r                                  # PRIMASK 0, CONTROL 0 as the emulator starts
assert 'APB0 160 MHz' in T.ln_rom_state(r) and '460846 baud' in T.ln_rom_state(r), T.ln_rom_state(r)
print('version and info: id %06x, %d kbytes, MAC %s, the clocks set and the ROM\'s registers reported ("%s")' % (
    t.flash_id, t.size >> 10, t.mac, T.ln_rom_state(r)))
for _ in range(5):
    chip.run()
assert b'wait' not in chip.tx, bytes(chip.tx)
print('after "version": no more wait lines: ok')

t.baud = 115200
data = T.lno_read(link, t, 0x1F00, 300, progress=False)
assert data == bytes(nor.mem[0x1F00:0x1F00 + 300])
data = T.lno_read(link, t, 0x30000, 4096, progress=False)
assert data == bytes(nor.mem[0x30000:0x31000])
print('read: 300 bytes and 4 k: ok')

T.lno_erase(link, t, 0x10000, 0x11000, progress=False)
assert nor.mem[0x10000:0x11000] == b'\xff' * 0x1000 and nor.counts['erase4k'] == 1
img = os.urandom(300)
link.call(T.LN_WRITE, struct.pack('<I', 0x10080) + img, timeout=5)
assert bytes(nor.mem[0x10080:0x10080 + 300]) == img and nor.counts['program'] == 2      # across a page boundary
assert T.lno_crc(link, 0x10000, 0x1000) == zlib.crc32(bytes(nor.mem[0x10000:0x11000]))
assert T.lno_crc(link, 0, 0x20000) == zlib.crc32(bytes(nor.mem[:0x20000]))
print('erase, write across a page, CRC (4 k and 128 k): ok')

nor.sr1 = 0x1C
try:
    link.call(T.LN_WRITE, struct.pack('<I', 0x10400) + b'\x12' * 16, timeout=5)
    raise AssertionError('no error')
except T.LnError as e:
    assert e.status == 8 and e.detail == 0x10400, e                  # protected: the read-back differs
back = link.call(T.LN_SRWRITE, b'\x02\x00\x02', timeout=5)
assert back == b'\x00\x02\x00' and nor.sr1 == 0 and nor.counts['srwrite'] == 1, back
print('protected flash: the write is caught by the read-back; status write: ok')

frame = bytearray(T.ln_frame(bytes([T.LN_STATUS, 0x55, 0, 0, 0])))
frame[2] ^= 0x40
link.write(bytes(frame))
body = link._frame(5.0)
assert body[0] == 0xFF and body[2] == 1, body
assert link.call(T.LN_STATUS) == b'\x00\x02\x00'
print('a damaged frame: answered with "CRC wrong", the next one works')

link.write(b'\rbaudrate 1000000\r')
chip.run()
assert chip.events[-1] == ('brr', 640), chip.events                  # 16 * 40 MHz / 1000000
chip.host_baud = 1000000                                             # the bridge follows the line
assert link.link(3.0) == 'C3LN 2 ready'
print('baudrate 1000000: divider 640 from the 40 MHz APB0, answering at the new speed')

link.call(T.LN_REBOOT)
for _ in range(20):
    if chip.rebooted:
        break
    chip.run()
assert chip.rebooted
assert not chip.violations and not nor.violations, (chip.violations, nor.violations)
print('reboot through the chip reset: ok')
print('LN882H loader emulation passed')
