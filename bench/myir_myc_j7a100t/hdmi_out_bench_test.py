#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Host side of the MYC-J7A100T HDMI out bench (see hdmi_out_bench.py): step-by-step SiI9022A
# bring-up over UARTBone + bit-banged I2C, with ACK checks, read-backs and a log file.
#
#   litex_server --uart --uart-port=/dev/ttyUSB0
#   ./hdmi_out_bench_test.py                    # full bring-up (DVI mode), then monitor 0x3D
#   ./hdmi_out_bench_test.py --stop-after=id    # only reset + probe + TPI + ID
#   ./hdmi_out_bench_test.py --no-termination   # skip the source termination step
#   ./hdmi_out_bench_test.py --edge=falling     # TPI 0x08 sampling edge override
#   ./hdmi_out_bench_test.py --vfreq-hz         # write VFreq as 60 instead of 6000
#   ./hdmi_out_bench_test.py --dump             # only dump registers (no writes)
#
# Steps: reset, probe, tpi, id, power, termination, config, mode, tmds, status.

import sys
import time
import argparse
from datetime import datetime

from litex import RemoteClient

STEPS = ["reset", "probe", "tpi", "id", "power", "termination", "config", "mode", "tmds", "status"]

# Logging ------------------------------------------------------------------------------------------

class Log:
    def __init__(self, filename):
        self.f = open(filename, "a")
        self.write(f"===== {datetime.now().isoformat()} =====")

    def write(self, msg):
        line = f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {msg}"
        print(line, flush=True)
        self.f.write(line + "\n")
        self.f.flush()

# Bit-banged I2C over LiteX I2CMaster CSRs ---------------------------------------------------------

class I2C:
    # hdmi_i2c_w: bit0 scl (1 = released), bit1 oe, bit2 sda (1 = released). hdmi_i2c_r: bit0 sda, bit1 scl.
    def __init__(self, bus, prefix="hdmi_i2c"):
        self.w = getattr(bus.regs, f"{prefix}_w")
        self.r = getattr(bus.regs, f"{prefix}_r")

    def _set(self, scl, sda):
        self.w.write((scl & 1) | (1 << 1) | ((sda & 1) << 2))

    def _sda(self):
        return self.r.read() & 1

    def start(self):
        self._set(1, 1); self._set(1, 0); self._set(0, 0)

    def stop(self):
        self._set(0, 0); self._set(1, 0); self._set(1, 1)

    def write_byte(self, b):
        for i in range(8):
            bit = (b >> (7 - i)) & 1
            self._set(0, bit); self._set(1, bit); self._set(0, bit)
        # ACK: release SDA, clock, sample.
        self._set(0, 1); self._set(1, 1)
        ack = (self._sda() == 0)
        self._set(0, 1)
        return ack

    def read_byte(self, ack):
        v = 0
        self._set(0, 1)
        for i in range(8):
            self._set(1, 1)
            v = (v << 1) | self._sda()
            self._set(0, 1)
        bit = 0 if ack else 1
        self._set(0, bit); self._set(1, bit); self._set(0, bit)
        self._set(0, 1)
        return v

    def probe(self, addr):
        self.start()
        ack = self.write_byte(addr << 1)
        self.stop()
        return ack

    def write_reg(self, addr, reg, val):
        self.start()
        ok = self.write_byte(addr << 1) and self.write_byte(reg) and self.write_byte(val)
        self.stop()
        return ok

    def read_reg(self, addr, reg):
        self.start()
        if not (self.write_byte(addr << 1) and self.write_byte(reg)):
            self.stop()
            return None
        self.start() # Repeated start.
        if not self.write_byte((addr << 1) | 1):
            self.stop()
            return None
        v = self.read_byte(ack=False)
        self.stop()
        return v

# SiI9022A helpers ---------------------------------------------------------------------------------

class SiI9022:
    def __init__(self, i2c, addr, log):
        self.i2c  = i2c
        self.addr = addr
        self.log  = log

    def wr(self, reg, val, check=True, what=""):
        ok = self.i2c.write_reg(self.addr, reg, val)
        rb = self.i2c.read_reg(self.addr, reg) if (ok and check) else None
        status = "ACK" if ok else "NACK"
        rbs = f" readback=0x{rb:02x}" if rb is not None else ""
        self.log.write(f"  W 0x{reg:02x} = 0x{val:02x} {status}{rbs} {what}")
        return ok

    def rd(self, reg, what=""):
        v = self.i2c.read_reg(self.addr, reg)
        self.log.write(f"  R 0x{reg:02x} -> " + (f"0x{v:02x}" if v is not None else "NACK") + f" {what}")
        return v

def decode_3d(v):
    if v is None:
        return "NACK"
    return (f"HPD={(v >> 2) & 1} RxSense={(v >> 3) & 1} "
            f"(events: hotplug={v & 1} rxsense={(v >> 1) & 1}, raw=0x{v:02x})")

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MYC-J7A100T HDMI out (SiI9022A) bring-up.")
    parser.add_argument("--csr-csv",        default="myir_hdmi_out_bench.csv", help="CSR configuration file.")
    parser.add_argument("--port",           default=1234, type=int, help="litex_server port.")
    parser.add_argument("--addr",           default="0x3b", help="SiI9022A 7-bit I2C address (board: 0x3b).")
    parser.add_argument("--stop-after",     default="status", choices=STEPS, help="Stop after this step.")
    parser.add_argument("--no-reset",       action="store_true", help="Skip the hardware reset pulse.")
    parser.add_argument("--no-termination", action="store_true", help="Skip the source termination step.")
    parser.add_argument("--hdmi",           action="store_true", help="HDMI mode (default DVI; no AVI InfoFrame written).")
    parser.add_argument("--edge",           default=None, choices=["rising", "falling"], help="TPI 0x08 sampling edge.")
    parser.add_argument("--vfreq-hz",       action="store_true", help="Write VFreq in Hz (60) instead of 0.01Hz (6000).")
    parser.add_argument("--dump",           action="store_true", help="Only dump TPI registers.")
    parser.add_argument("--monitor",        default=10, type=float, help="Seconds to monitor 0x3D at the end (0 = skip).")
    parser.add_argument("--log",            default="hdmi_out_bench.log", help="Log file (appended).")
    args = parser.parse_args()

    log = Log(args.log)
    log.write("args: " + " ".join(sys.argv[1:]))

    bus = RemoteClient(csr_csv=args.csr_csv, port=args.port)
    bus.open()
    c = bus.constants
    pix_clk    = getattr(c, "hdmi_bench_pix_clk", 74250000)
    h_total    = getattr(c, "hdmi_bench_h_total", 1650)
    v_total    = getattr(c, "hdmi_bench_v_total", 750)
    clk_invert = getattr(c, "hdmi_bench_clk_invert", 1)
    log.write(f"bitstream: pix_clk={pix_clk/1e6:.3f}MHz total={h_total}x{v_total} clk_invert={clk_invert}")

    i2c  = I2C(bus)
    addr = int(args.addr, 0)
    sii  = SiI9022(i2c, addr, log)

    def step(name):
        log.write(f"--- step: {name}")
        return name

    def done(name):
        if name == args.stop_after:
            log.write(f"stopped after '{name}' as requested.")
            bus.close()
            sys.exit(0)

    def fail(msg):
        log.write(f"FAIL: {msg}")
        bus.close()
        sys.exit(1)

    if args.dump:
        for reg in list(range(0x00, 0x0b)) + [0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x3c, 0x3d, 0x60]:
            sii.rd(reg)
        bus.close()
        return

    # 1. Reset.
    step("reset")
    if not args.no_reset:
        bus.regs.hdmi_ctrl_rst_n.write(0); time.sleep(0.02)
        bus.regs.hdmi_ctrl_rst_n.write(1); time.sleep(0.05)
        log.write("  RESETn pulsed (20ms low), waited 50ms.")
    log.write(f"  INT pin = {bus.regs.hdmi_ctrl_status.read() & 1}")
    done("reset")

    # 2. Probe.
    step("probe")
    found = {a: i2c.probe(a) for a in (0x39, 0x3b)}
    for a, ack in found.items():
        log.write(f"  addr 0x{a:02x}: {'ACK' if ack else 'no ACK'}")
    if not found[addr]:
        fail(f"no ACK at 0x{addr:02x}: check RESETn (K18), I2C pull-ups, address strap.")
    done("probe")

    # 3. TPI mode (must be the first write after reset).
    step("tpi")
    if not sii.wr(0xc7, 0x00, check=False, what="(TPI enable)"):
        fail("NACK on C7=00.")
    time.sleep(0.01)
    done("tpi")

    # 4. ID.
    step("id")
    dev_id = sii.rd(0x1b, "(device ID, expect 0xb0)")
    sii.rd(0x1c, "(production revision)")
    sii.rd(0x1d, "(TPI revision)")
    if dev_id != 0xb0:
        fail(f"unexpected device ID {dev_id}: TPI mode not entered or not a SiI9022A.")
    done("id")

    # 5. Power up (D0).
    step("power")
    sii.wr(0x1e, 0x00, what="(power state D0)")
    done("power")

    # 6. Source termination (SiI9022A/9024A): BC=01, BD=82, BE |= 1.
    step("termination")
    if args.no_termination:
        log.write("  skipped (--no-termination).")
    else:
        sii.wr(0xbc, 0x01, check=False, what="(internal page 1)")
        sii.wr(0xbd, 0x82, check=False, what="(indexed reg 0x82)")
        be = sii.rd(0xbe, "(current value)")
        if be is None:
            fail("NACK reading 0xBE.")
        sii.wr(0xbe, be | 0x01, check=False, what="(source termination on)")
        sii.wr(0xbd, 0x82, check=False)
        sii.rd(0xbe, "(read-back, bit0 should be 1)")
    done("termination")

    # 7. Config: TMDS off, DVI/HDMI, RGB in/out, AVI cleared.
    step("config")
    sii.wr(0x1a, 0x11 if args.hdmi else 0x10, what="(TMDS off, " + ("HDMI" if args.hdmi else "DVI") + ")")
    sii.wr(0x09, 0x00, what="(input format: RGB, auto range)")
    sii.wr(0x0a, 0x00, what="(output format: RGB)")
    for reg in range(0x0c, 0x1a):
        sii.wr(reg, 0x00, check=False)
    log.write("  AVI InfoFrame 0x0C-0x19 cleared.")
    done("config")

    # 8. Video mode (committed when 0x07 is written).
    step("mode")
    pclk  = int(round(pix_clk / 10e3))
    vfreq = 60 if args.vfreq_hz else 6000
    vals  = [pclk & 0xff, pclk >> 8, vfreq & 0xff, vfreq >> 8,
             h_total & 0xff, h_total >> 8, v_total & 0xff, v_total >> 8]
    for reg, val in enumerate(vals):
        sii.wr(reg, val, check=False)
    log.write(f"  video mode: pclk={pclk}x10kHz vfreq={vfreq} pixels={h_total} lines={v_total}")
    for reg in range(8):
        sii.rd(reg)
    done("mode")

    # 9. TMDS on, then 0x08 (must follow 0x1A[4] 1->0).
    step("tmds")
    edge = args.edge or ("rising" if clk_invert else "falling")
    reg08 = 0x70 if edge == "rising" else 0x60
    sii.wr(0x1a, 0x01 if args.hdmi else 0x00, what="(TMDS on)")
    sii.wr(0x08, reg08, what=f"(x1, full pixel, {edge} edge, no repetition)")
    sii.rd(0x1e, "(power state)")
    done("tmds")

    # 10. Status.
    step("status")
    t0 = time.time()
    last = None
    while True:
        v = sii.i2c.read_reg(addr, 0x3d)
        if v != last:
            log.write(f"  0x3D: {decode_3d(v)}  INT pin={bus.regs.hdmi_ctrl_status.read() & 1}")
            last = v
        if args.monitor == 0 or (time.time() - t0) > args.monitor:
            break
        time.sleep(0.5)
    log.write("done: check the monitor and note the result (no signal / out of range / image).")
    bus.close()

if __name__ == "__main__":
    main()
