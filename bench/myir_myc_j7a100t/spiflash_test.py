#!/usr/bin/env python3

#
# SPI Flash test for the MYiR MYC-J7A100T LiteX target (--with-spi-flash).
#
# Talks to the BIOS over the serial console and checks that the memory-mapped SPI Flash reads back
# the bitstream that was programmed with --flash:
#   boot   : reboot and parse the BIOS SPI Flash init (quad enable, clk calibration, memspeed).
#   region : find the SPIFLASH region in mem_list.
#   header : the bitstream sync word (0xAA995566) is in the first 256 bytes.
#   crc    : BIOS "crc" over the whole bitstream == zlib.crc32 of the .bin.
#   alias  : the upper 16MB does not mirror the lower 16MB (4-byte addressing works).
#   speed  : mem_speed (read-only) on the Flash.
#
# Usage (close litex_term first):
#   ./spiflash_test.py --port /dev/ttyUSB1 --bin build/myir_myc_j7a100t/gateware/myir_myc_j7a100t.bin
#

import re
import sys
import time
import zlib
import argparse

import serial

PROMPT      = b"litex> "
ANSI_ESCAPE = re.compile(rb"\x1b\[[0-9;?]*[A-Za-z]")

# BIOS console ------------------------------------------------------------------------------------

class BIOS:
    def __init__(self, port, baudrate, log=None):
        self.port = serial.Serial(port, baudrate, timeout=0.1)
        self.log  = log

    def _read_until(self, marker, timeout):
        # The BIOS prompt is colored (ANSI_LITEX "litex" ANSI_RESET "> "): strip escape sequences
        # before looking for it.
        buf      = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            data = self.port.read(4096)
            if data:
                buf += data
                if self.log:
                    self.log.write(data.decode(errors="replace"))
                    self.log.flush()
                text = ANSI_ESCAPE.sub(b"", buf)
                if text.rstrip(b" ").endswith(marker.rstrip(b" ")):
                    return text.decode(errors="replace"), True
        return ANSI_ESCAPE.sub(b"", buf).decode(errors="replace"), False

    def sync(self):
        self.port.reset_input_buffer()
        for _ in range(3):
            self.port.write(b"\n")
            _, ok = self._read_until(PROMPT, 1.0)
            if ok:
                return True
        return False

    def cmd(self, line, timeout=10.0):
        self.port.reset_input_buffer()
        self.port.write(line.encode() + b"\n")
        out, ok = self._read_until(PROMPT, timeout)
        if not ok:
            raise TimeoutError(f"no prompt after '{line}'")
        # Drop the echoed command and the trailing prompt.
        lines = out.replace("\r", "").split("\n")
        if lines and line in lines[0]:
            lines = lines[1:]
        return "\n".join(l for l in lines if not l.startswith("litex>")).strip()

    def reboot(self, timeout=30.0):
        self.port.reset_input_buffer()
        self.port.write(b"reboot\n")
        out, ok = self._read_until(PROMPT, timeout)
        return out.replace("\r", ""), ok

# Helpers -----------------------------------------------------------------------------------------

def parse_mem_read(out):
    data = b""
    for line in out.split("\n"):
        m = re.match(r"\s*0x[0-9a-fA-F]+\s+((?:[0-9a-fA-F]{2}\s+){1,16})", line)
        if m:
            data += bytes(int(b, 16) for b in m.group(1).split())
    return data

class Result:
    def __init__(self):
        self.items = []

    def add(self, name, ok, msg):
        self.items.append((name, ok, msg))
        state = {True: "PASS", False: "FAIL", None: "SKIP"}[ok]
        print(f"[{state}] {name:7s} {msg}")

    @property
    def ok(self):
        return all(ok is not False for _, ok, _ in self.items)

def flash_crc(bios, addr, length):
    m = re.search(r"CRC32:\s*([0-9a-fA-F]{8})", bios.cmd(f"crc 0x{addr:08x} {length}", timeout=30))
    return int(m.group(1), 16) if m else None

def locate_diffs(bios, base, bitstream, block=64*1024, sub=4096, max_dump=4):
    # 64KB blocks, then 4KB sub-blocks, then a byte compare of the first differing sub-blocks.
    bad_blocks = []
    for off in range(0, len(bitstream), block):
        chunk = bitstream[off:off+block]
        if flash_crc(bios, base + off, len(chunk)) != (zlib.crc32(chunk) & 0xffffffff):
            bad_blocks.append(off)
    n_blocks = (len(bitstream) + block - 1)//block
    print(f"          | {len(bad_blocks)}/{n_blocks} blocks of 64KB differ" +
        (": " + " ".join(f"0x{o:06x}" for o in bad_blocks[:16]) + (" ..." if len(bad_blocks) > 16 else "") if bad_blocks else ""))
    if bad_blocks:
        print(f"          | first bad block 0x{bad_blocks[0]:06x}, last 0x{bad_blocks[-1]:06x}")
    dumped = 0
    for boff in bad_blocks:
        for off in range(boff, min(boff + block, len(bitstream)), sub):
            if dumped >= max_dump:
                return
            chunk = bitstream[off:off+sub]
            if flash_crc(bios, base + off, len(chunk)) == (zlib.crc32(chunk) & 0xffffffff):
                continue
            data  = parse_mem_read(bios.cmd(f"mem_read 0x{base + off:08x} {len(chunk)}", timeout=30))
            diffs = [i for i in range(min(len(data), len(chunk))) if data[i] != chunk[i]]
            if not diffs:
                print(f"          | 0x{off:06x}: CRC differs but mem_read matches (unstable reads?)")
            else:
                i = diffs[0]
                ff = all(data[j] == 0xff for j in diffs)
                print(f"          | 0x{off:06x}: {len(diffs)} bytes differ, first at 0x{off+i:06x}"
                      f"{' (flash reads 0xff: not programmed?)' if ff else ''}")
                print(f"          |   flash: {data[i:i+16].hex(' ')}")
                print(f"          |   .bin : {chunk[i:i+16].hex(' ')}")
            dumped += 1

# Test --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MYC-J7A100T SPI Flash test (via BIOS console).")
    parser.add_argument("--port",      default="/dev/ttyUSB1",  help="Serial port of the BIOS console.")
    parser.add_argument("--baudrate",  default=115200, type=int, help="Serial baudrate.")
    parser.add_argument("--bin",       default="build/myir_myc_j7a100t/gateware/myir_myc_j7a100t.bin",
                                                                help="Bitstream programmed in the Flash (.bin).")
    parser.add_argument("--no-reboot", action="store_true",     help="Do not reboot to check the BIOS Flash init.")
    parser.add_argument("--log",       default=None,            help="Write the raw console output to this file.")
    args = parser.parse_args()

    try:
        bitstream = open(args.bin, "rb").read()
    except OSError as e:
        print(f"Cannot read bitstream: {e}")
        sys.exit(1)
    bin_crc = zlib.crc32(bitstream) & 0xffffffff
    print(f"Bitstream: {args.bin}, {len(bitstream)} bytes, CRC32 {bin_crc:08x}")

    log  = open(args.log, "w") if args.log else None
    bios = BIOS(args.port, args.baudrate, log=log)
    res  = Result()

    # Boot.
    if not args.no_reboot:
        if not bios.sync():
            print("No BIOS prompt on the serial port (is litex_term still open?).")
            sys.exit(1)
        out, ok = bios.reboot()
        lines = [l for l in out.split("\n") if ("SPI Flash" in l) or ("Quad" in l) or ("Memspeed" in l)
                 or ("[DIV" in l) or ("CRC32" in l) or ("Unable" in l) or ("failed" in l)]
        for l in lines:
            print(f"          | {l.strip()}")
        if not ok:
            res.add("boot", False, "no prompt after reboot")
        elif "Initializing mx25l25673g" not in out:
            res.add("boot", False, "BIOS did not initialize the SPI Flash (built without --with-spi-flash?)")
        elif ("quad enable failed" in out) or ("quad enable timeout" in out) or ("Unable to read" in out):
            res.add("boot", False, "quad enable failed")
        else:
            m = re.search(r"SPI Flash clk configured to (\d+) MHz", out)
            res.add("boot", True, f"quad enabled, clk {m.group(1) if m else '?'} MHz")
    elif not bios.sync():
        print("No BIOS prompt on the serial port (is litex_term still open?).")
        sys.exit(1)

    # Region.
    out = bios.cmd("mem_list")
    m   = re.search(r"SPIFLASH\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)", out)
    if m:
        base, size = int(m.group(1), 16), int(m.group(3), 16)
    else:
        m = re.search(r"SPIFLASH\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)", out)
        if not m:
            res.add("region", False, "SPIFLASH region not in mem_list")
            print("RESULT: FAIL")
            sys.exit(1)
        base, size = int(m.group(1), 16), int(m.group(2), 16)
    res.add("region", size == 32*1024*1024, f"base 0x{base:08x}, size 0x{size:x}")

    # Header.
    head = parse_mem_read(bios.cmd(f"mem_read 0x{base:08x} 256"))
    sync = head.find(bytes([0xaa, 0x99, 0x55, 0x66]))
    if len(head) != 256:
        res.add("header", False, f"mem_read returned {len(head)} bytes")
    elif head == b"\xff"*256:
        res.add("header", False, "Flash erased at offset 0 (program it with --flash first)")
    else:
        same = head == bitstream[:256]
        res.add("header", (sync >= 0) and same,
            f"sync word at offset {sync}, first 256 bytes {'match' if same else 'DIFFER from'} the .bin")
        if not same:
            print("          | flash:", head[:32].hex(" "))
            print("          | .bin :", bitstream[:32].hex(" "))

    # CRC over the whole bitstream.
    out = bios.cmd(f"crc 0x{base:08x} {len(bitstream)}", timeout=60)
    m   = re.search(r"CRC32:\s*([0-9a-fA-F]{8})", out)
    if not m:
        res.add("crc", False, f"unexpected crc output: {out!r}")
    else:
        flash_crc = int(m.group(1), 16)
        res.add("crc", flash_crc == bin_crc, f"flash {flash_crc:08x}, .bin {bin_crc:08x}")
        if flash_crc != bin_crc:
            # Read stability: the same range twice must give the same CRC.
            m2 = re.search(r"CRC32:\s*([0-9a-fA-F]{8})", bios.cmd(f"crc 0x{base:08x} {len(bitstream)}", timeout=60))
            if m2:
                same = int(m2.group(1), 16) == flash_crc
                res.add("reread", same, f"second CRC {m2.group(1)} ({'stable' if same else 'UNSTABLE reads'})")
            locate_diffs(bios, base, bitstream)

    # Upper 16MB must not alias the lower 16MB (would mean 3-byte addressing).
    half = 16*1024*1024
    lo   = bios.cmd(f"crc 0x{base:08x} 4096")
    hi   = bios.cmd(f"crc 0x{base + half:08x} 4096")
    lo_m = re.search(r"CRC32:\s*([0-9a-fA-F]{8})", lo)
    hi_m = re.search(r"CRC32:\s*([0-9a-fA-F]{8})", hi)
    if not (lo_m and hi_m):
        res.add("alias", False, "unexpected crc output")
    elif bitstream[:4096] == b"\xff"*4096:
        res.add("alias", None, "first 4KB erased, cannot tell")
    else:
        res.add("alias", lo_m.group(1) != hi_m.group(1),
            f"crc @0 {lo_m.group(1)}, crc @16MB {hi_m.group(1)}")

    # Read speed.
    out = bios.cmd(f"mem_speed 0x{base:08x} 0x10000 1", timeout=60)
    m   = re.search(r"Read speed:\s*([0-9.]+\s*\S+)", out)
    res.add("speed", None if not m else True, f"read {m.group(1)}" if m else f"{out!r}")

    print(f"RESULT: {'PASS' if res.ok else 'FAIL'}")
    sys.exit(0 if res.ok else 1)

if __name__ == "__main__":
    main()
