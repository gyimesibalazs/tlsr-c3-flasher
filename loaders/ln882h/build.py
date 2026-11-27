#!/usr/bin/env python3
"""Builds the LN882H RAM loader and embeds it in tlsr_c3.py.

    python3 loaders/ln882h/build.py            # build, fill the boot header, write it into tlsr_c3.py
    python3 loaders/ln882h/build.py --check    # build and compare with the copy in tlsr_c3.py (exit 1 if they differ)
    python3 loaders/ln882h/build.py --out ln882h_loader.bin

The compiler is PlatformIO's Arm toolchain (~/.platformio/packages/toolchain-gccarmnoneeabi), or --cc / $LN882H_CC.
Cortex-M4, Thumb-2, soft-float: the loader uses no floating point, so it does not matter how the ROM leaves the FPU.

The boot header (start.S) is what the ROM reads: load address, length (16 bits), the offset from which the image CRC
is taken, the image CRC-32 (zlib's), the vector table address, a code read protection flag (0), and a CRC-32 of the
header's first 20 bytes. The layout is the one of the SDK's RAM loader; fill_header() does what its build tools do.
"""
import argparse
import base64
import os
import re
import struct
import subprocess
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.normpath(os.path.join(HERE, '..', '..', 'tlsr_c3.py'))
SOURCES = ['start.S', 'ln882h.c', 'loader.c', 'flash.c']
CFLAGS = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-Os', '-std=c11', '-ffreestanding',
          '-fno-tree-loop-distribute-patterns', '-ffunction-sections', '-fdata-sections', '-fno-unwind-tables',
          '-fno-asynchronous-unwind-tables', '-Wall', '-Wextra', '-Werror']
LDFLAGS = ['-nostdlib', '-nostartfiles', '-Wl,--gc-sections', '-Wl,--build-id=none']
LOAD_ADDR = 0x20000000
RAM_END = 0x20020000                       # SRAM0: image, zeroed data and stack must stay in it
HEADER = struct.Struct('<IHHIIII')         # 24 bytes; the header CRC covers the first 20
BEGIN = '# ---- begin LN882H loader (loaders/ln882h; written by loaders/ln882h/build.py, do not edit)'
END = '# ---- end LN882H loader'
# a line the flasher's bridge would take for a speed change (tlsr_c3.py LN_BAUD_LINE)
BAUD_LINE = re.compile(rb'[\r\n]baudrate [0-9]+[\r\n]')


def compiler(cc=''):
    """The C compiler, or None when it is not installed."""
    cc = cc or os.environ.get('LN882H_CC') or os.path.expanduser(
        '~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc')
    return cc if os.path.isfile(cc) and os.access(cc, os.X_OK) else None


def fill_header(image):
    """The image with its boot header completed: length, image CRC from crc_offset on, header CRC."""
    load, _, crc_offset, _, vectors, crp, _ = HEADER.unpack_from(image)
    if load != LOAD_ADDR or len(image) > 0xFFFF or crc_offset < HEADER.size:
        raise ValueError('boot header: load address 0x%x, length %d, crc_offset %d' % (load, len(image), crc_offset))
    head = bytearray(HEADER.pack(load, len(image), crc_offset, zlib.crc32(image[crc_offset:]), vectors, crp, 0))
    head[20:24] = struct.pack('<I', zlib.crc32(head[:20]))
    return bytes(head) + image[HEADER.size:]


def header_fields(image):
    """The header as a dict, with the values fill_header computes next to the stored ones."""
    load, length, crc_offset, crc, vectors, crp, hcrc = HEADER.unpack_from(image)
    return dict(load=load, length=length, crc_offset=crc_offset, crc=crc, vectors=vectors, crp=crp, header_crc=hcrc,
                length_ok=length == len(image), crc_ok=crc == zlib.crc32(image[crc_offset:]),
                header_crc_ok=hcrc == zlib.crc32(image[:20]))


def build(cc, keep=None):
    """Compiles and links in a temporary directory; returns the finished image."""
    tmp = keep or tempfile.mkdtemp(prefix='ln882h_loader_')
    elf = os.path.join(tmp, 'loader.elf')
    cmd = [cc] + CFLAGS + LDFLAGS + ['-T', os.path.join(HERE, 'ln882h.ld'), '-o', elf,
                                     '-Wl,-Map,' + os.path.join(tmp, 'loader.map')]
    cmd += [os.path.join(HERE, s) for s in SOURCES] + ['-lgcc']
    subprocess.run(cmd, check=True)
    tools = os.path.dirname(cc)
    prefix = os.path.basename(cc)[:-3]     # arm-none-eabi-
    raw = os.path.join(tmp, 'loader.bin')
    subprocess.run([os.path.join(tools, prefix + 'objcopy'), '-O', 'binary', elf, raw], check=True)
    syms = subprocess.run([os.path.join(tools, prefix + 'nm'), elf], check=True, capture_output=True, text=True).stdout
    top = int(re.search(r'^([0-9a-f]+) \w __stack_top$', syms, re.M).group(1), 16)
    if top > RAM_END:
        raise ValueError('the loader needs RAM up to 0x%x, beyond SRAM0' % top)
    with open(raw, 'rb') as fh:
        image = fill_header(fh.read())
    if BAUD_LINE.search(image):
        raise ValueError('the image holds a "baudrate N" line, which the bridge would take for a speed change')
    return image, top


def embedded(path=TOOL):
    with open(path) as fh:
        text = fh.read()
    m = re.search(re.escape(BEGIN) + r'\n(.*?)\n' + re.escape(END), text, re.S)
    if not m:
        raise ValueError('%s has no embedded LN882H loader' % path)
    return base64.b64decode(''.join(re.findall(r"'([A-Za-z0-9+/=]*)'", m.group(1))))


def embed(image, path=TOOL):
    with open(path) as fh:
        text = fh.read()
    b64 = base64.b64encode(image).decode()
    lines = ['    %r' % b64[i:i + 108] for i in range(0, len(b64), 108)]
    block = '%s\nLN_LOADER = base64.b64decode(\n%s)\n%s' % (BEGIN, '\n'.join(lines), END)
    new, n = re.subn(re.escape(BEGIN) + r'\n.*?\n' + re.escape(END), lambda _: block, text, flags=re.S)
    if n != 1:
        raise ValueError('%s: the marker lines of the embedded loader were not found' % path)
    with open(path, 'w') as fh:
        fh.write(new)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='compare with the copy in tlsr_c3.py instead of writing it')
    ap.add_argument('--out', default='', help='also write the image to this file')
    ap.add_argument('--cc', default='', help='the arm-none-eabi-gcc to use')
    ap.add_argument('--keep', default='', help='build in this directory and keep the ELF and the map')
    args = ap.parse_args()
    cc = compiler(args.cc)
    if not cc:
        sys.exit('arm-none-eabi-gcc not found (PlatformIO\'s toolchain, --cc or $LN882H_CC)')
    if args.keep:
        os.makedirs(args.keep, exist_ok=True)
    image, top = build(cc, args.keep or None)
    print('LN882H loader: %d bytes, RAM used up to 0x%x' % (len(image), top))
    if args.out:
        with open(args.out, 'wb') as fh:
            fh.write(image)
    if args.check:
        if embedded() != image:
            sys.exit('tlsr_c3.py holds a different build of the loader: run loaders/ln882h/build.py')
        print('tlsr_c3.py holds this build.')
    else:
        embed(image)
        print('Written into %s.' % TOOL)


if __name__ == '__main__':
    main()
