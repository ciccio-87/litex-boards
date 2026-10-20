#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Host side of the MYC-J7A100T HDMI in loopback bench (see hdmi_in_bench.py): brings up the HDMI out
# (SiI9022A, validated sequence), then the ADV7611 step by step over UARTBone + bit-banged I2C, and
# checks what the FPGA receives against what the HDMI out sends (timing, color bars, frame
# stability). Needs an HDMI cable from the HDMI out connector to the HDMI in connector.
#
#   litex_server --uart --uart-port=/dev/ttyACM0
#   ./hdmi_in_bench_test.py                       # full run
#   ./hdmi_in_bench_test.py --stop-after=id       # only ADV7611 reset + probe + ID
#   ./hdmi_in_bench_test.py --skip-out            # HDMI out already configured (FPGA reloaded)
#   ./hdmi_in_bench_test.py --scan                # eye scan: LLC DLL phase x sampling edge
#   ./hdmi_in_bench_test.py --dump                # only dump ADV7611 status registers
#   ./hdmi_in_bench_test.py --skip-out --bits     # per-pin activity across the color bars
#   ./hdmi_in_bench_test.py --set io:0x04=0x62    # override a register after the config step
#   ./hdmi_in_bench_test.py --skip-out --chsel-scan  # try every OP_CH_SEL (IO 0x04[7:5])
#
# Steps: out, reset, probe, id, maps, config, edid, lock, timing, capture, stability.

import sys
import time
import argparse

from litex import RemoteClient

from hdmi_out_bench_test import Log, I2C, SiI9022, decode_3d

STEPS = ["out", "reset", "probe", "id", "maps", "config", "edid", "lock", "timing", "capture", "stability"]

# ADV7611 I2C maps (7-bit addresses) ---------------------------------------------------------------

# The IO map is fixed at 0x4C (0x98 8-bit); the other maps are programmed through IO 0xF4-0xFD
# (8-bit values), same addresses as the vendor example (15_hdmi_in_ddr_hdmi_out).
IO    = 0x4c
MAPS  = [
    # IO reg, 8-bit addr, name
    (0xf4, 0x80, "CEC"),
    (0xf5, 0x7c, "INFOFRAME"),
    (0xf8, 0x4c, "DPLL"),
    (0xf9, 0x64, "KSV"),
    (0xfa, 0x6c, "EDID"),
    (0xfb, 0x68, "HDMI"),
    (0xfd, 0x44, "CP"),
]
KSV  = 0x64 >> 1
EDID = 0x6c >> 1
HDMI = 0x68 >> 1
CP   = 0x44 >> 1

# Vendor example configuration (ADI recommended settings), RGB 24-bit 4:4:4 SDR output.
CONFIG = [
    (IO,   0x01, 0x05, "PRIM_MODE (vendor value)"),
    (IO,   0x00, 0x14, "VID_STD (vendor value)"),
    (IO,   0x02, 0xf7, "auto input color space, RGB out"),
    (IO,   0x03, 0x40, "24-bit SDR 4:4:4 mode 0"),
    (IO,   0x04, 0x62, "OP_CH_SEL=011: platform r/g/b pins = R/G/B (vendor 0x42 swaps R and B), xtal"),
    (IO,   0x05, 0x28, "AV codes off"),
    (IO,   0x06, 0xa7, "invert HS/VS, LLC polarity (bit0)"),
    (IO,   0x0b, 0x44, "power up part"),
    (IO,   0x0c, 0x42, "power up part"),
    (IO,   0x15, 0x80, "disable tristate of pins"),
    (IO,   0x19, 0x8a, "LLC DLL enable, phase 0x0a"),
    (IO,   0x33, 0x40, "LLC DLL mux enable"),
    (IO,   0x14, 0x3f, "drive strength"),
    (CP,   0xba, 0x01, "HDMI free run"),
    (CP,   0x7c, 0x01, ""),
    (KSV,  0x40, 0x81, "disable HDCP 1.1 features"),
    (HDMI, 0x9b, 0x03, "ADI recommended"),
    *[(HDMI, reg, 0x01, "ADI recommended") for reg in range(0xc1, 0xcd)],
    (HDMI, 0x00, 0x00, "HDMI input port A"),
    (HDMI, 0x83, 0xfe, "clock terminator port A"),
    (HDMI, 0x6f, 0x08, "ADI recommended"),
    (HDMI, 0x85, 0x1f, "ADI recommended"),
    (HDMI, 0x87, 0x70, "ADI recommended"),
    (HDMI, 0x8d, 0x04, "LFG"),
    (HDMI, 0x8e, 0x1e, "HFG"),
    (HDMI, 0x1a, 0x8a, "unmute audio"),
    (HDMI, 0x57, 0xda, "ADI recommended"),
    (HDMI, 0x58, 0x01, "ADI recommended"),
    (HDMI, 0x75, 0x10, "DDC drive strength"),
]

# Vendor EDID (1080p60 + 1080i60 DTDs), with the extension count (byte 126) set to 0: the vendor
# block announces an extension that is never written. Checksum recomputed.
EDID_DATA = [
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x52, 0x74, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x03, 0x80, 0x29, 0x1a, 0x78, 0x0a, 0xe5, 0xb5, 0xa3, 0x55, 0x49, 0x99, 0x27,
    0x13, 0x50, 0x54, 0x23, 0x08, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
    0x45, 0x00, 0x80, 0x68, 0x21, 0x00, 0x00, 0x1e, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40,
    0x58, 0x2c, 0x45, 0x00, 0x80, 0x68, 0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x54,
    0x45, 0x53, 0x54, 0x20, 0x54, 0x56, 0x0a, 0x20, 0x0a, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfd,
    0x00, 0x38, 0x3e, 0x1e, 0x44, 0x0f, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00,
]
EDID_DATA[127] = (-sum(EDID_DATA[:127])) & 0xff

# LiteX ColorBarsPattern: 8 bars of hres/8 pixels.
COLOR_BARS = [
    ("white",   0xff, 0xff, 0xff),
    ("yellow",  0xff, 0xff, 0x00),
    ("cyan",    0x00, 0xff, 0xff),
    ("green",   0x00, 0xff, 0x00),
    ("magenta", 0xff, 0x00, 0xff),
    ("red",     0xff, 0x00, 0x00),
    ("blue",    0x00, 0x00, 0xff),
    ("black",   0x00, 0x00, 0x00),
]

CAPTURE_BASE = 0x20000000

# ADV7611 helpers ----------------------------------------------------------------------------------

class ADV7611:
    def __init__(self, i2c, log):
        self.i2c = i2c
        self.log = log

    def wr(self, addr, reg, val, what="", check=False):
        ok = self.i2c.write_reg(addr, reg, val)
        rb = self.i2c.read_reg(addr, reg) if (ok and check) else None
        rbs = f" readback=0x{rb:02x}" if rb is not None else ""
        self.log.write(f"  W [0x{addr:02x}] 0x{reg:02x} = 0x{val:02x} {'ACK' if ok else 'NACK'}{rbs} {what}")
        return ok

    def rd(self, addr, reg, what="", quiet=False):
        v = self.i2c.read_reg(addr, reg)
        if not quiet:
            self.log.write(f"  R [0x{addr:02x}] 0x{reg:02x} -> " + (f"0x{v:02x}" if v is not None else "NACK") + f" {what}")
        return v

    def status(self):
        # HDMI map status (ADV7611 HDMI map): 0x04 [1] TMDS PLL locked, 0x05 [7] HDMI mode,
        # 0x07 [7] vertical filter locked, [5] DE regen filter locked, 0x07-0x08 line width,
        # 0x09-0x0A field 0 height, 0x1E-0x1F total line width, 0x26-0x27 field 0 total height,
        # 0x51-0x52 TMDS frequency. Decoded as best effort: the FPGA measurements are the reference.
        r = {reg: self.rd(HDMI, reg, quiet=True) for reg in (0x04, 0x05, 0x07, 0x08, 0x09, 0x0a, 0x1e, 0x1f, 0x26, 0x27, 0x51, 0x52)}
        if None in r.values():
            self.log.write("  HDMI map status: NACK")
            return None
        st = {
            "tmds_pll_lock" : (r[0x04] >> 1) & 1,
            "hdmi_mode"     : (r[0x05] >> 7) & 1,
            "v_filter_lock" : (r[0x07] >> 7) & 1,
            "de_filter_lock": (r[0x07] >> 5) & 1,
            "line_width"    : ((r[0x07] & 0x1f) << 8) | r[0x08],
            "field_height"  : ((r[0x09] & 0x1f) << 8) | r[0x0a],
            "total_width"   : ((r[0x1e] & 0x3f) << 8) | r[0x1f],
            "total_height"  : (((r[0x26] & 0x3f) << 8) | r[0x27]) // 2,
            "tmds_mhz"      : ((r[0x51] << 1) | (r[0x52] >> 7)) + (r[0x52] & 0x7f) / 128,
        }
        self.log.write("  ADV7611: " + " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in st.items()))
        return st

# FPGA side measurements ---------------------------------------------------------------------------

def read_timing(bus):
    regs = bus.regs
    t = {}
    for name in ["frames", "de_period", "de_high", "hs_period", "hs_high", "vs_period", "vs_high", "lines", "checksum", "checksum_diffs"]:
        # Values cross clock domains through MultiRegs: read twice and keep a stable value.
        r = getattr(regs, f"video_in_{name}")
        a, b = r.read(), r.read()
        t[name] = b if a != b else a
    t["freq"] = regs.video_in_freq_value.read()
    return t

def describe_timing(t):
    def pol(period, high):
        if period == 0:
            return "?"
        return "+" if high < period - high else "-"
    hs_pulse = min(t["hs_high"], t["hs_period"] - t["hs_high"]) if t["hs_period"] else 0
    vs_pulse = min(t["vs_high"], t["vs_period"] - t["vs_high"]) if t["vs_period"] else 0
    h_total  = t["de_period"] or 1
    return (f"LLC={t['freq']/1e6:.3f}MHz h_active={t['de_high']} h_total={t['de_period']} "
            f"hs={hs_pulse}({pol(t['hs_period'], t['hs_high'])}) v_active={t['lines']} "
            f"v_total={t['vs_period']/h_total:.2f} vs={vs_pulse/h_total:.2f}lines({pol(t['vs_period'], t['vs_high'])}) "
            f"frames={t['frames']} fps={t['freq']/t['vs_period'] if t['vs_period'] else 0:.2f} "
            f"checksum=0x{t['checksum']:08x} diffs={t['checksum_diffs']}")

def capture_line(bus, line, timeout=1.0):
    bus.regs.video_in_capture_line.write(line)
    bus.regs.video_in_capture_start.write(1)
    t0 = time.time()
    while not bus.regs.video_in_capture_done.read():
        if time.time() - t0 > timeout:
            return None
        time.sleep(0.01)
    n = bus.regs.video_in_capture_count.read()
    data = []
    for i in range(0, n, 64): # Chunked burst reads over UARTBone.
        data += bus.read(CAPTURE_BASE + 4*i, min(64, n - i))
    return data

def check_bars(data, h_active, tol):
    # Returns (errors, per-bar summary). A pixel is OK if every channel is within tol of the bar color.
    bar_w  = h_active // 8
    errors = 0
    first  = None
    summary = []
    for i, (name, r, g, b) in enumerate(COLOR_BARS):
        bar = data[i*bar_w:(i + 1)*bar_w]
        bad = 0
        for j, px in enumerate(bar):
            pr, pg, pb = (px >> 16) & 0xff, (px >> 8) & 0xff, px & 0xff
            if max(abs(pr - r), abs(pg - g), abs(pb - b)) > tol:
                bad += 1
                if first is None:
                    first = (i*bar_w + j, px, name)
        mid = bar[len(bar)//2] if bar else 0
        summary.append(f"{name}:0x{mid:06x}" + (f"({bad} bad)" if bad else ""))
        errors += bad
    if len(data) != h_active:
        errors += abs(len(data) - h_active)
    return errors, first, " ".join(summary)

# Capture word layout (hdmi_in_bench.py): bits 0-7 = platform "b" pins, 8-15 = "g", 16-23 = "r".
CAPTURE_PINS = (
    [("b", i, p) for i, p in enumerate("F13 F14 F16 E17 C14 C15 E16 D16".split())] +
    [("g", i, p) for i, p in enumerate("E13 E14 D14 D15 C13 B13 A15 A16".split())] +
    [("r", i, p) for i, p in enumerate("A13 A14 B20 A20 A18 A19 D20 C20".split())]
)

def group_report(data, h_active):
    # Compact view: for each 8-pin group (platform b/g/r), which component its pins follow.
    bar_w = h_active // 8
    expected = {
        "R": "".join("1" if r else "0" for _, r, g, b in COLOR_BARS),
        "G": "".join("1" if g else "0" for _, r, g, b in COLOR_BARS),
        "B": "".join("1" if b else "0" for _, r, g, b in COLOR_BARS),
    }
    out = []
    for group, base in (("b", 0), ("g", 8), ("r", 16)):
        names = []
        for bit in range(base, base + 8):
            pattern = ""
            for i in range(8):
                bar  = data[i*bar_w + 4:(i + 1)*bar_w - 4]
                ones = sum((px >> bit) & 1 for px in bar)
                pattern += "1" if ones > 0.9*len(bar) else "0" if ones < 0.1*len(bar) else "~"
            names.append(next((c for c, e in expected.items() if e == pattern), "?"))
        out.append(f"{group}-pins:" + "".join(names))
    return " ".join(out)

def bit_report(data, h_active, log):
    # For each captured bit, its level in each of the 8 bars ('1' / '0' / '~' if it toggles), and
    # which expected color component (R/G/B, any bit since the bars are 0x00/0xff) it follows.
    bar_w = h_active // 8
    expected = {
        "R": "".join("1" if r else "0" for _, r, g, b in COLOR_BARS),
        "G": "".join("1" if g else "0" for _, r, g, b in COLOR_BARS),
        "B": "".join("1" if b else "0" for _, r, g, b in COLOR_BARS),
    }
    log.write("  expected per bar (white..black): R=" + expected["R"] + " G=" + expected["G"] + " B=" + expected["B"])
    for bit, (bus, idx, pin) in enumerate(CAPTURE_PINS):
        pattern = ""
        for i in range(8):
            # Skip a few pixels at each bar edge.
            bar  = data[i*bar_w + 4:(i + 1)*bar_w - 4]
            ones = sum((px >> bit) & 1 for px in bar)
            pattern += "1" if ones > 0.9*len(bar) else "0" if ones < 0.1*len(bar) else "~"
        inverted = pattern.translate(str.maketrans("01", "10"))
        follows  = [c for c, e in expected.items() if pattern == e] + [f"~{c}" for c, e in expected.items() if inverted == e]
        if "~" in pattern:
            what = "TOGGLING (noise / not driven?)"
        elif pattern in ("00000000", "11111111"):
            what = "stuck " + pattern[0]
        else:
            what = "follows " + (", ".join(follows) if follows else "nothing expected")
        log.write(f"  bit {bit:2d} ({bus}{idx} {pin:>3}): {pattern}  {what}")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MYC-J7A100T HDMI in (ADV7611) loopback bring-up.")
    parser.add_argument("--csr-csv",    default="myir_hdmi_in_bench.csv", help="CSR configuration file.")
    parser.add_argument("--port",       default=1234, type=int, help="litex_server port.")
    parser.add_argument("--stop-after", default="stability", choices=STEPS, help="Stop after this step.")
    parser.add_argument("--skip-out",   action="store_true", help="Do not (re)configure the HDMI out.")
    parser.add_argument("--no-reset",   action="store_true", help="Skip the ADV7611 hardware reset pulse.")
    parser.add_argument("--no-edid",    action="store_true", help="Do not program the internal EDID / HPD.")
    parser.add_argument("--edge",       default="rising", choices=["rising", "falling"], help="FPGA sampling edge.")
    parser.add_argument("--llc-phase",  default=None, type=lambda x: int(x, 0), help="ADV7611 LLC DLL phase (0-31, IO 0x19).")
    parser.add_argument("--line",       default=None, type=int, help="Active line to capture (default: middle).")
    parser.add_argument("--tol",        default=8, type=int, help="Per-channel tolerance for the color bar check.")
    parser.add_argument("--stability",  default=10, type=float, help="Seconds of checksum stability check.")
    parser.add_argument("--scan",       action="store_true", help="Eye scan (LLC DLL phase x edge) after config.")
    parser.add_argument("--dump",       action="store_true", help="Only dump ADV7611 status and FPGA measurements.")
    parser.add_argument("--bits",       action="store_true", help="Per-pin activity report of the captured line.")
    parser.add_argument("--chsel-scan", action="store_true", help="Capture with every OP_CH_SEL value (IO 0x04[7:5]).")
    parser.add_argument("--set",        default=[], action="append", metavar="MAP:REG=VAL",
        help="Register override after the config step (maps: io, cp, hdmi, ksv), e.g. io:0x04=0x62. Repeatable.")
    parser.add_argument("--log",        default="hdmi_in_bench.log", help="Log file (appended).")
    args = parser.parse_args()

    log = Log(args.log)
    log.write("args: " + " ".join(sys.argv[1:]))

    bus = RemoteClient(csr_csv=args.csr_csv, port=args.port)
    bus.open()
    c = bus.constants
    pix_clk  = c.hdmi_bench_pix_clk
    h_active = c.hdmi_bench_h_active
    v_active = c.hdmi_bench_v_active
    h_total  = c.hdmi_bench_h_total
    v_total  = c.hdmi_bench_v_total
    log.write(f"bitstream: out {h_active}x{v_active} total={h_total}x{v_total} pix_clk={pix_clk/1e6:.3f}MHz")

    out_i2c = I2C(bus, prefix="hdmi_i2c")
    adv_i2c = I2C(bus, prefix="adv_i2c")
    adv     = ADV7611(adv_i2c, log)

    def step(name):
        log.write(f"--- step: {name}")

    def done(name):
        if name == args.stop_after:
            log.write(f"stopped after '{name}' as requested.")
            bus.close()
            sys.exit(0)

    def fail(msg):
        log.write(f"FAIL: {msg}")
        bus.close()
        sys.exit(1)

    bus.regs.video_in_edge.write(1 if args.edge == "falling" else 0)

    if args.dump:
        adv.status()
        for reg in (0x6a, 0x6f, 0x19, 0x06):
            adv.rd(IO, reg)
        log.write("  FPGA: " + describe_timing(read_timing(bus)))
        bus.close()
        return

    # 1. HDMI out (validated SiI9022A sequence, DVI, see hdmi_out_bench_test.py).
    step("out")
    if args.skip_out:
        log.write("  skipped (--skip-out).")
    else:
        sii = SiI9022(out_i2c, 0x3b, log)
        bus.regs.hdmi_ctrl_rst_n.write(0); time.sleep(0.02)
        bus.regs.hdmi_ctrl_rst_n.write(1); time.sleep(0.05)
        if not sii.wr(0xc7, 0x00, check=False, what="(TPI enable)"):
            fail("SiI9022A: NACK on C7=00.")
        time.sleep(0.01)
        if sii.rd(0x1b, "(device ID, expect 0xb0)") != 0xb0:
            fail("SiI9022A: unexpected device ID.")
        pclk = int(round(pix_clk/10e3))
        seq  = [(0x1e, 0x00), (0xbc, 0x01), (0xbd, 0x82), (0xbe, 0x25), (0x1a, 0x10), (0x09, 0x00), (0x0a, 0x00)]
        seq += [(reg, 0x00) for reg in range(0x0c, 0x1a)]
        seq += list(enumerate([pclk & 0xff, pclk >> 8, 6000 & 0xff, 6000 >> 8,
                               h_total & 0xff, h_total >> 8, v_total & 0xff, v_total >> 8]))
        seq += [(0x1a, 0x00), (0x08, 0x70)]
        for reg, val in seq:
            if not out_i2c.write_reg(0x3b, reg, val):
                fail(f"SiI9022A: NACK writing 0x{reg:02x}.")
        log.write(f"  SiI9022A configured ({len(seq) + 1} writes), TMDS on, DVI.")
    done("out")

    # 2. ADV7611 reset.
    step("reset")
    if args.no_reset:
        log.write("  skipped (--no-reset).")
    else:
        bus.regs.adv_ctrl_rst_n.write(0); time.sleep(0.02)
        bus.regs.adv_ctrl_rst_n.write(1); time.sleep(0.05)
        log.write("  RESET pulsed (20ms low), waited 50ms.")
    log.write(f"  INT1 pin = {bus.regs.adv_ctrl_status.read() & 1}")
    done("reset")

    # 3. Probe.
    step("probe")
    acks = [a for a in range(0x08, 0x78) if adv_i2c.probe(a)]
    log.write("  ACK at: " + (" ".join(f"0x{a:02x}" for a in acks) or "none"))
    if IO not in acks:
        fail(f"no ACK at the IO map (0x{IO:02x}): check RESET (F15), I2C pull-ups (E22/D22).")
    done("probe")

    # 4. ID (IO 0xEA/0xEB, RD_INFO).
    step("id")
    hi = adv.rd(IO, 0xea, "(RD_INFO high)")
    lo = adv.rd(IO, 0xeb, "(RD_INFO low)")
    if hi is None or lo is None:
        fail("NACK reading RD_INFO.")
    log.write(f"  RD_INFO = 0x{hi:02x}{lo:02x} (ADV7611: 0x2051 per ADI)")
    done("id")

    # 5. Maps.
    step("maps")
    for reg, val, name in MAPS:
        adv.wr(IO, reg, val, f"({name} map at 0x{val >> 1:02x})", check=True)
    time.sleep(0.01)
    acks = [a for a in range(0x08, 0x78) if adv_i2c.probe(a)]
    log.write("  ACK at: " + " ".join(f"0x{a:02x}" for a in acks))
    missing = [name for reg, val, name in MAPS if (val >> 1) not in acks and name != "CEC"]
    if missing:
        fail("maps not answering: " + ", ".join(missing))
    done("maps")

    # 6. Config.
    step("config")
    nacks = 0
    for addr, reg, val, what in CONFIG:
        if not adv.wr(addr, reg, val, what):
            nacks += 1
    if args.llc_phase is not None:
        adv.wr(IO, 0x19, 0x80 | (args.llc_phase & 0x1f), "(LLC DLL phase override)", check=True)
    maps = {"io": IO, "cp": CP, "hdmi": HDMI, "ksv": KSV}
    for ov in args.set:
        name, rest = ov.split(":")
        reg, val   = (int(x, 0) for x in rest.split("="))
        if not adv.wr(maps[name.lower()], reg, val, "(--set override)", check=True):
            nacks += 1
    if nacks:
        fail(f"{nacks} NACKs during config.")
    done("config")

    # 7. EDID + HPD (vendor sequence, manual HPA while the EDID is written).
    step("edid")
    if args.no_edid:
        log.write("  skipped (--no-edid).")
    else:
        adv.wr(HDMI, 0x6c, 0xa3, "(manual HPA)")
        adv.wr(IO,   0x20, 0x70, "(HPD low)")
        adv.wr(KSV,  0x74, 0x00, "(internal EDID off)")
        for i, b in enumerate(EDID_DATA):
            if not adv_i2c.write_reg(EDID, i, b):
                fail(f"NACK writing EDID byte {i}.")
        rb = [adv_i2c.read_reg(EDID, i) for i in range(0, 128, 16)]
        log.write("  EDID written (128 bytes), read-back every 16th: " + " ".join(f"{v:02x}" if v is not None else "--" for v in rb))
        adv.wr(KSV,  0x74, 0x01, "(internal EDID on)")
        adv.wr(IO,   0x20, 0xf0, "(HPD high)")
        adv.wr(HDMI, 0x6c, 0xa2, "(auto HPA)")
        time.sleep(0.2)
        log.write("  SiI9022A " + decode_3d(out_i2c.read_reg(0x3b, 0x3d)))
    done("edid")

    # 8. Lock.
    step("lock")
    st = None
    for i in range(20):
        st = adv.status()
        if st and st["tmds_pll_lock"] and st["de_filter_lock"]:
            break
        time.sleep(0.25)
    if not (st and st["tmds_pll_lock"]):
        log.write("  WARNING: TMDS PLL not reported locked (decoding is best effort, continuing).")
    done("lock")

    # 9. Timing (FPGA side).
    step("timing")
    t = read_timing(bus)
    log.write("  FPGA: " + describe_timing(t))
    exp_freq_ok = abs(t["freq"] - pix_clk) < pix_clk*0.002
    checks = [
        ("LLC frequency", exp_freq_ok,                     f"{t['freq']} vs {pix_clk}"),
        ("h_active",      t["de_high"] == h_active,        f"{t['de_high']} vs {h_active}"),
        ("h_total",       t["de_period"] == h_total,       f"{t['de_period']} vs {h_total}"),
        ("v_active",      t["lines"] == v_active,          f"{t['lines']} vs {v_active}"),
        ("v_total",       t["vs_period"] == h_total*v_total, f"{t['vs_period']} vs {h_total*v_total}"),
    ]
    for name, ok, detail in checks:
        log.write(f"  {'OK  ' if ok else 'FAIL'} {name}: {detail}")
    if args.chsel_scan:
        log.write("--- OP_CH_SEL scan (IO 0x04[7:5], low bits kept): component followed by each pin, bit 0..7")
        line = v_active // 2 if args.line is None else args.line
        reg04 = adv.rd(IO, 0x04, "(current)")
        for chsel in range(8):
            adv.i2c.write_reg(IO, 0x04, (chsel << 5) | (reg04 & 0x1f))
            time.sleep(0.2)
            data = capture_line(bus, line)
            log.write(f"  OP_CH_SEL={chsel:03b}: " + (group_report(data, h_active) if data else "capture timeout"))
        adv.wr(IO, 0x04, reg04, "(restored)")
    if args.scan:
        log.write("--- eye scan (LLC DLL phase x sampling edge): bar errors / checksum diffs over 1s")
        line = v_active // 2 if args.line is None else args.line
        for edge in (0, 1):
            bus.regs.video_in_edge.write(edge)
            row = []
            for phase in range(32):
                adv.i2c.write_reg(IO, 0x19, 0x80 | phase)
                time.sleep(0.1)
                d0   = bus.regs.video_in_checksum_diffs.read()
                data = capture_line(bus, line)
                errs = check_bars(data, h_active, args.tol)[0] if data is not None else -1
                time.sleep(1.0)
                diffs = bus.regs.video_in_checksum_diffs.read() - d0
                row.append(f"{phase:2d}:{errs}/{diffs}")
            log.write(f"  edge={'falling' if edge else 'rising'}: " + " ".join(row))
        bus.regs.video_in_edge.write(1 if args.edge == "falling" else 0)
        adv.wr(IO, 0x19, 0x80 | ((args.llc_phase if args.llc_phase is not None else 0x0a) & 0x1f), "(LLC DLL phase restored)")
    if not all(ok for _, ok, _ in checks[:4]):
        fail("timing mismatch (see above).")
    done("timing")

    # 10. Capture one active line and check the color bars.
    step("capture")
    line = v_active // 2 if args.line is None else args.line
    data = capture_line(bus, line)
    if data is None:
        fail("capture timeout (no DE / no frames?).")
    errors, first, summary = check_bars(data, h_active, args.tol)
    log.write(f"  line {line}: {len(data)} pixels, {errors} errors (tol={args.tol})")
    log.write(f"  bars (middle pixel 0xRRGGBB): {summary}")
    if first:
        log.write(f"  first bad pixel: x={first[0]} 0x{first[1]:06x} in {first[2]}")
    if args.bits or errors:
        bit_report(data, h_active, log)
    done("capture")

    # 11. Stability: with a static source every frame must have the same checksum.
    step("stability")
    t0 = read_timing(bus)
    time.sleep(args.stability)
    t1 = read_timing(bus)
    frames = t1["frames"] - t0["frames"]
    diffs  = t1["checksum_diffs"] - t0["checksum_diffs"]
    log.write(f"  {frames} frames in {args.stability:.0f}s, {diffs} with a different checksum, last checksum=0x{t1['checksum']:08x}")
    ok = (errors == 0) and (diffs == 0) and (frames > 0)
    log.write("RESULT: " + ("PASS" if ok else "FAIL") + f" (bar errors={errors}, unstable frames={diffs}/{frames})")
    bus.close()
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
