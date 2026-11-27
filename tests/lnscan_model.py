"""src/lnscan.h (LineBaudScan) in Python, for the bridge models of the tests."""
import re


class LnScan:
    """src/lnscan.h (LineBaudScan) in Python (the native check below runs the C++ on the same cases)."""
    def __init__(self, key=b'baudrate ', rest=False):
        self.line, self.bad = b'', False
        self.pattern = re.compile(re.escape(key) + rb'([0-9]{1,9})' + (rb'(?: [ -~]*)?' if rest else b''))

    def feed(self, b):
        if b in (13, 10):
            baud = 0
            m = None if self.bad else self.pattern.fullmatch(self.line)
            if m:
                baud = int(m.group(1))
            self.line, self.bad = b'', False
            return baud if 1200 <= baud <= 5000000 else 0
        if self.bad:
            return 0
        if b < 0x20 or b > 0x7e or len(self.line) >= 40:
            self.bad = True
            return 0
        self.line += bytes([b])
        return 0
