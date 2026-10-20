#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# HDMI in (ADV7611) bring-up bench for the MYiR MYC-J7A100T, in loopback with the HDMI out.
#
# - HDMI out: same validated path as hdmi_out_bench.py (color bars, SiI9022A, exact pixel clock).
#   Connect an HDMI cable from the HDMI out connector to the HDMI in connector.
# - HDMI in: ADV7611 reset (F15) and interrupt (F21) on CSRs, bit-banged I2CMaster on SCL/SDA
#   (E22/D22), LLC pixel clock (C19, N side of an MRCC pair -> CLOCK_DEDICATED_ROUTE FALSE) on a
#   BUFG, 24-bit RGB + DE/HS/VS captured with IDDRs. The sampling edge (LLC rising/falling) is
#   selectable from a CSR; with the ADV7611 LLC DLL phase (IO 0x19) this allows an eye scan.
# - Measurements in the LLC domain: pixel clock frequency, DE/HS/VS period and high time, active
#   lines per frame, frame counter, per-frame pixel checksum and "checksum changed" counter (with a
#   static source every frame must be identical), and capture of one active line in a 2048-pixel
#   buffer readable from the bus.
# - No CPU: everything is driven from the host over UARTBone (see hdmi_in_bench_test.py).
#
# Build/load:
#   ./hdmi_in_bench.py --build --load [--mode=720p|1080p]
#   litex_server --uart --uart-port=/dev/ttyACM0
#   ./hdmi_in_bench_test.py

import argparse

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex_boards.platforms import myir_myc_j7a100t

from litex.build.io import DDRInput

from litex.soc.cores.clock import *
from litex.soc.cores.freqmeter import FreqMeter
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.bitbang import I2CMaster

from hdmi_out_bench import VIDEO_MODES, _CRG, SiI9022PHY, SiI9022Ctrl

# ADV7611 control (reset / interrupt) --------------------------------------------------------------

class ADV7611Ctrl(LiteXModule):
    def __init__(self, pads):
        self._rst_n  = CSRStorage(reset=1, description="ADV7611 RESET (1 = run, 0 = reset).")
        self._status = CSRStatus(fields=[
            CSRField("int", size=1, description="ADV7611 INT1 pin level."),
        ])
        self.comb += pads.rst_n.eq(self._rst_n.storage)
        self.specials += MultiReg(pads.int, self._status.fields.int)

# ADV7611 parallel video input + measurements ------------------------------------------------------

class ADV7611VideoIn(LiteXModule):
    def __init__(self, pads, clock_domain="hdmi_in", capture_depth=2048):
        self._edge = CSRStorage(description="Sampling edge: 0 = LLC rising, 1 = LLC falling (half a cycle earlier).")

        # Measurements (LLC cycles unless stated, latched once per frame on the VS rising edge).
        self._frames         = CSRStatus(32, description="Frame counter (VS rising edges).")
        self._de_period      = CSRStatus(32, description="DE rising edge to rising edge (h_total).")
        self._de_high        = CSRStatus(32, description="DE high time (h_active).")
        self._hs_period      = CSRStatus(32, description="HS period.")
        self._hs_high        = CSRStatus(32, description="HS high time.")
        self._vs_period      = CSRStatus(32, description="VS period (frame length).")
        self._vs_high        = CSRStatus(32, description="VS high time.")
        self._lines          = CSRStatus(32, description="Active (DE) lines per frame.")
        self._checksum       = CSRStatus(32, description="Sum of active pixels (R<<16|G<<8|B) of the last frame.")
        self._checksum_diffs = CSRStatus(32, description="Frames whose checksum differed from the previous one.")

        # Line capture.
        self._capture_line  = CSRStorage(16, description="Active line to capture (0 = first DE line).")
        self._capture_start = CSR()
        self._capture_done  = CSRStatus(description="Capture done (cleared by capture_start).")
        self._capture_count = CSRStatus(16, description="Pixels captured.")

        # # #

        cd = clock_domain

        # Clock: LLC on C19 (N side of the clock-capable pair, no dedicated route to a BUFG).
        self.cd_hdmi_in = ClockDomain(cd)
        self.specials += Instance("BUFG", i_I=pads.clk, o_O=self.cd_hdmi_in.clk)

        # Inputs: IDDRs, both edges captured, one selected.
        edge = Signal()
        self.specials += MultiReg(self._edge.storage, edge, cd)
        pins   = Cat(pads.b, pads.g, pads.r, pads.de, pads.hsync, pads.vsync)
        rising = Signal(len(pins))
        falling = Signal(len(pins))
        self.specials += DDRInput(i=pins, o1=rising, o2=falling, clk=ClockSignal(cd))
        sample = Signal(len(pins))
        sync = getattr(self.sync, cd)
        sync += sample.eq(Mux(edge, falling, rising))

        b     = sample[0:8]
        g     = sample[8:16]
        r     = sample[16:24]
        de    = sample[24]
        hs    = sample[25]
        vs    = sample[26]
        pixel = Cat(b, g, r)

        de_d = Signal()
        hs_d = Signal()
        vs_d = Signal()
        sync += [de_d.eq(de), hs_d.eq(hs), vs_d.eq(vs)]
        de_rise = Signal()
        hs_rise = Signal()
        vs_rise = Signal()
        self.comb += [
            de_rise.eq(de & ~de_d),
            hs_rise.eq(hs & ~hs_d),
            vs_rise.eq(vs & ~vs_d),
        ]

        # Period / high time of a signal, measured between its rising edges.
        def measure(s, s_rise):
            count  = Signal(32)
            high   = Signal(32)
            period = Signal(32)
            high_l = Signal(32)
            getattr(self.sync, cd).__iadd__([
                count.eq(count + 1),
                If(s, high.eq(high + 1)),
                If(s_rise,
                    period.eq(count),
                    high_l.eq(high),
                    count.eq(1),
                    high.eq(1),
                )
            ])
            return period, high_l

        de_period, de_high = measure(de, de_rise)
        hs_period, hs_high = measure(hs, hs_rise)
        vs_period, vs_high = measure(vs, vs_rise)

        # Per-frame counters (latched on VS rising edge).
        frames         = Signal(32)
        lines          = Signal(32)
        lines_l        = Signal(32)
        checksum       = Signal(32)
        checksum_l     = Signal(32)
        checksum_diffs = Signal(32)
        sync += [
            If(de_rise, lines.eq(lines + 1)),
            If(de, checksum.eq(checksum + pixel)),
            If(vs_rise,
                frames.eq(frames + 1),
                lines.eq(0),
                lines_l.eq(lines),
                checksum.eq(0),
                checksum_l.eq(checksum),
                If((frames > 1) & (checksum != checksum_l),
                    checksum_diffs.eq(checksum_diffs + 1)
                )
            )
        ]

        # Line capture.
        mem = Memory(32, capture_depth)
        wr  = mem.get_port(write_capable=True, clock_domain=cd)
        self.specials += wr
        self.sram = wishbone.SRAM(mem, read_only=True)

        start_ps = PulseSynchronizer("sys", cd)
        self.submodules += start_ps
        self.comb += start_ps.i.eq(self._capture_start.re)
        capture_line = Signal(16)
        self.specials += MultiReg(self._capture_line.storage, capture_line, cd)

        armed    = Signal()
        active   = Signal()
        done     = Signal()
        line     = Signal(16) # DE lines since VS (1 = first active line).
        cur_line = Signal(16) # Same, including the current cycle's DE rising edge.
        addr     = Signal(max=capture_depth + 1)
        in_line  = Signal()
        self.comb += [
            cur_line.eq(Mux(vs_rise, 0, Mux(de_rise, line + 1, line))),
            in_line.eq(cur_line == (capture_line + 1)),
        ]
        sync += [
            line.eq(cur_line),
            If(start_ps.o,
                armed.eq(1),
                active.eq(0),
                done.eq(0),
            ).Elif(armed & vs_rise,
                armed.eq(0),
                active.eq(1),
                addr.eq(0),
            ),
            If(active & de & in_line & (addr < capture_depth),
                addr.eq(addr + 1)
            ),
            If(active & de_d & ~de & in_line,
                active.eq(0),
                done.eq(1),
            ),
        ]
        self.comb += [
            wr.adr.eq(addr),
            wr.dat_w.eq(pixel),
            wr.we.eq(active & de & in_line & (addr < capture_depth)),
        ]

        # Status to sys (static between frames; read twice on the host if in doubt).
        for src, dst in [
            (frames,         self._frames.status),
            (de_period,      self._de_period.status),
            (de_high,        self._de_high.status),
            (hs_period,      self._hs_period.status),
            (hs_high,        self._hs_high.status),
            (vs_period,      self._vs_period.status),
            (vs_high,        self._vs_high.status),
            (lines_l,        self._lines.status),
            (checksum_l,     self._checksum.status),
            (checksum_diffs, self._checksum_diffs.status),
            (done,           self._capture_done.status),
            (addr,           self._capture_count.status),
        ]:
            self.specials += MultiReg(src, dst)

# HDMI in bench SoC --------------------------------------------------------------------------------

class HDMIInBenchSoC(SoCMini):
    def __init__(self, sys_clk_freq=100e6, mode="720p"):
        platform = myir_myc_j7a100t.Platform()
        timings  = VIDEO_MODES[mode]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, timings[1]["pix_clk"])

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident = f"LiteX HDMI in (loopback) bench on MYiR MYC-J7A100T ({mode}).")

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # HDMI out (as hdmi_out_bench.py, inverted forwarded clock) --------------------------------
        out_pads = platform.request("hdmi_out")
        self.hdmi_ctrl = SiI9022Ctrl(out_pads)
        self.hdmi_i2c  = I2CMaster(out_pads)
        self.videophy  = SiI9022PHY(out_pads, clock_domain="hdmi", clk_invert=True)
        self.add_video_colorbars(phy=self.videophy, timings=timings, clock_domain="hdmi")

        # HDMI in ----------------------------------------------------------------------------------
        in_pads = platform.request("hdmi_in")
        self.adv_ctrl = ADV7611Ctrl(in_pads)
        self.adv_i2c  = I2CMaster(in_pads)
        self.video_in = ADV7611VideoIn(in_pads, clock_domain="hdmi_in")
        self.bus.add_slave("video_in_capture", self.video_in.sram.bus,
            SoCRegion(origin=0x20000000, size=4*2048, mode="r", cached=False))
        self.video_in_freq = FreqMeter(period=int(sys_clk_freq), clk=ClockSignal("hdmi_in"))

        platform.add_platform_command("set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {clk}_IBUF]", clk=in_pads.clk)
        platform.add_period_constraint(in_pads.clk, 1e9/148.5e6)
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.video_in.cd_hdmi_in.clk)
        platform.add_false_path_constraints(self.crg.cd_hdmi.clk, self.video_in.cd_hdmi_in.clk)

        # Leds: heartbeat on the sys / HDMI in clock.
        sys_counter = Signal(28)
        llc_counter = Signal(28)
        self.sync         += sys_counter.eq(sys_counter + 1)
        self.sync.hdmi_in += llc_counter.eq(llc_counter + 1)
        self.comb += [
            platform.request("user_led", 0).eq(sys_counter[26]),
            platform.request("user_led", 1).eq(llc_counter[26]),
        ]

        # Constants for the host script.
        t = timings[1]
        self.add_constant("HDMI_BENCH_PIX_CLK",    int(t["pix_clk"]))
        self.add_constant("HDMI_BENCH_H_ACTIVE",   t["h_active"])
        self.add_constant("HDMI_BENCH_V_ACTIVE",   t["v_active"])
        self.add_constant("HDMI_BENCH_H_TOTAL",    t["h_active"] + t["h_blanking"])
        self.add_constant("HDMI_BENCH_V_TOTAL",    t["v_active"] + t["v_blanking"])
        self.add_constant("HDMI_BENCH_CLK_INVERT", 1)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="HDMI in (ADV7611) loopback bench on MYiR MYC-J7A100T.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream (to SRAM).")
    parser.add_argument("--sys-clk-freq", default=100e6, type=float, help="System clock frequency.")
    parser.add_argument("--mode",         default="720p", choices=VIDEO_MODES.keys(), help="HDMI out video mode.")
    args = parser.parse_args()

    soc = HDMIInBenchSoC(sys_clk_freq=args.sys_clk_freq, mode=args.mode)
    builder = Builder(soc, output_dir="build/myir_hdmi_in_bench", csr_csv="myir_hdmi_in_bench.csv")
    builder.build(build_name="myir_hdmi_in_bench", run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
