#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# HDMI out (SiI9022A) bring-up bench for the MYiR MYC-J7A100T.
#
# - Color bars at 720p60 (exact 74.25MHz) or 1080p60 (exact 148.5MHz) from a dedicated MMCM.
# - Parallel RGB 24-bit to the SiI9022A, pixel clock forwarded with an ODDR, inverted by default so
#   data is stable at the rising edge the SiI9022A samples on (TPI 0x08[4] = 1).
# - SiI9022A reset (K18) and interrupt (K19) on CSRs, bit-banged I2CMaster on SCL/SDA (L19/L20).
# - No CPU: everything is driven from the host over UARTBone (see hdmi_out_bench_test.py), so each
#   bring-up step is checked (ACK, read-back) and logged.
#
# Build/load:
#   ./hdmi_out_bench.py --build --load [--mode=720p|1080p] [--no-clk-invert]
#   litex_server --uart --uart-port=/dev/ttyUSB0
#   ./hdmi_out_bench_test.py --csr-csv=myir_hdmi_out_bench.csv

import argparse

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex_boards.platforms import myir_myc_j7a100t

from litex.build.io import SDROutput, DDROutput

from litex.soc.cores.clock import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.video import video_data_layout

# Video modes (CEA-861) ----------------------------------------------------------------------------

# Note: LiteX's built-in "1280x720@60Hz" entry has front/back porch swapped (h_sync_offset=220):
# CEA-861 720p60 is front porch 110, sync 40, back porch 220.
VIDEO_MODES = {
    "720p": ("1280x720@60Hz", {
        "pix_clk"       : 74.25e6,
        "h_active"      : 1280,
        "h_blanking"    : 370,
        "h_sync_offset" : 110,
        "h_sync_width"  : 40,
        "v_active"      : 720,
        "v_blanking"    : 30,
        "v_sync_offset" : 5,
        "v_sync_width"  : 5,
    }),
    "1080p": ("1920x1080@60Hz", {
        "pix_clk"       : 148.5e6,
        "h_active"      : 1920,
        "h_blanking"    : 280,
        "h_sync_offset" : 88,
        "h_sync_width"  : 44,
        "v_active"      : 1080,
        "v_blanking"    : 45,
        "v_sync_offset" : 4,
        "v_sync_width"  : 5,
    }),
}

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, pix_clk_freq):
        self.rst     = Signal()
        self.cd_sys  = ClockDomain()
        self.cd_hdmi = ClockDomain()

        # # #

        clk200 = platform.request("clk200")
        rst_n  = platform.request("rst_n")

        # Sys PLL.
        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

        # Pixel clock MMCM (fractional: 200MHz -> exact 74.25/148.5MHz).
        self.video_mmcm = video_mmcm = S7MMCM(speedgrade=-2)
        self.comb += video_mmcm.reset.eq(~rst_n | self.rst)
        video_mmcm.register_clkin(pll.clkin, 200e6)
        video_mmcm.create_clkout(self.cd_hdmi, pix_clk_freq, margin=0)
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_hdmi.clk)

# SiI9022A parallel video PHY ----------------------------------------------------------------------

class SiI9022PHY(LiteXModule):
    def __init__(self, pads, clock_domain="hdmi", clk_invert=True):
        self.sink = sink = stream.Endpoint(video_data_layout)

        # # #

        cd = ClockSignal(clock_domain)
        self.comb += sink.ready.eq(1)

        # Forwarded pixel clock (ODDR). Inverted: rising edge at the middle of the data eye.
        if clk_invert:
            self.specials += DDROutput(i1=0, i2=1, o=pads.clk, clk=cd)
        else:
            self.specials += DDROutput(i1=1, i2=0, o=pads.clk, clk=cd)

        # Controls / data (registered in the IOBs).
        self.specials += SDROutput(i=sink.de,    o=pads.de,    clk=cd)
        self.specials += SDROutput(i=sink.hsync, o=pads.hsync, clk=cd)
        self.specials += SDROutput(i=sink.vsync, o=pads.vsync, clk=cd)
        for i in range(8):
            self.specials += SDROutput(i=sink.r[i] & sink.de, o=pads.r[i], clk=cd)
            self.specials += SDROutput(i=sink.g[i] & sink.de, o=pads.g[i], clk=cd)
            self.specials += SDROutput(i=sink.b[i] & sink.de, o=pads.b[i], clk=cd)

# SiI9022A control (reset / interrupt) -------------------------------------------------------------

class SiI9022Ctrl(LiteXModule):
    def __init__(self, pads):
        self._rst_n  = CSRStorage(reset=1, description="SiI9022A RESETn (1 = run, 0 = reset).")
        self._status = CSRStatus(fields=[
            CSRField("int", size=1, description="SiI9022A INT pin level (active low by default)."),
        ])
        self.comb += pads.rst_n.eq(self._rst_n.storage)
        self.specials += MultiReg(pads.int, self._status.fields.int)

# HDMI out bench SoC -------------------------------------------------------------------------------

class HDMIOutBenchSoC(SoCMini):
    def __init__(self, sys_clk_freq=100e6, mode="720p", clk_invert=True):
        platform = myir_myc_j7a100t.Platform()
        timings  = VIDEO_MODES[mode]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, timings[1]["pix_clk"])

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident = f"LiteX HDMI out bench on MYiR MYC-J7A100T ({mode} / clk_invert={int(clk_invert)}).")

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # HDMI out ---------------------------------------------------------------------------------
        pads = platform.request("hdmi_out")
        self.hdmi_ctrl = SiI9022Ctrl(pads)
        self.hdmi_i2c  = I2CMaster(pads)
        self.videophy  = SiI9022PHY(pads, clock_domain="hdmi", clk_invert=clk_invert)
        self.add_video_colorbars(phy=self.videophy, timings=timings, clock_domain="hdmi")

        # Leds: heartbeat on sys / pixel clock.
        sys_counter = Signal(28)
        pix_counter = Signal(28)
        self.sync      += sys_counter.eq(sys_counter + 1)
        self.sync.hdmi += pix_counter.eq(pix_counter + 1)
        self.comb += [
            platform.request("user_led", 0).eq(sys_counter[26]),
            platform.request("user_led", 1).eq(pix_counter[26]),
        ]

        # Constants for the host script.
        t = timings[1]
        self.add_constant("HDMI_BENCH_PIX_CLK",    int(t["pix_clk"]))
        self.add_constant("HDMI_BENCH_H_TOTAL",    t["h_active"] + t["h_blanking"])
        self.add_constant("HDMI_BENCH_V_TOTAL",    t["v_active"] + t["v_blanking"])
        self.add_constant("HDMI_BENCH_CLK_INVERT", int(clk_invert))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="HDMI out (SiI9022A) bench on MYiR MYC-J7A100T.")
    parser.add_argument("--build",         action="store_true", help="Build bitstream.")
    parser.add_argument("--load",          action="store_true", help="Load bitstream (to SRAM).")
    parser.add_argument("--sys-clk-freq",  default=100e6, type=float, help="System clock frequency.")
    parser.add_argument("--mode",          default="720p", choices=VIDEO_MODES.keys(), help="Video mode.")
    parser.add_argument("--no-clk-invert", action="store_true", help="Forward the pixel clock non-inverted.")
    args = parser.parse_args()

    soc = HDMIOutBenchSoC(
        sys_clk_freq = args.sys_clk_freq,
        mode         = args.mode,
        clk_invert   = not args.no_clk_invert,
    )
    builder = Builder(soc, output_dir="build/myir_hdmi_out_bench", csr_csv="myir_hdmi_out_bench.csv")
    builder.build(build_name="myir_hdmi_out_bench", run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
