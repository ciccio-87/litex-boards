#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# SFP0/SFP1 PRBS BIST for the MYiR MYC-J7A100T.
#
# Both SFP cages are driven by GTPs of Quad 216 (SFP0 -> GTPE2_CHANNEL_X0Y6, SFP1 -> X0Y7) from a
# single QPLL fed by the 125MHz MGTREFCLK1 (F10/E10). Each GTP transmits a PRBS and checks the PRBS
# it receives, so with a DAC cable between SFP0 and SFP1:
#   - serdes1 RX errors -> SFP0 TX -> cable -> SFP1 RX path.
#   - serdes0 RX errors -> SFP1 TX -> cable -> SFP0 RX path.
# Without a cable, near-end PMA loopback (--loopback) tests each GTP alone (no PCB/cage).
#
# Stand-alone result on the LEDs (PRBS7 runs from power-up, no host needed):
#   user_led0 = SFP0 RX, user_led1 = SFP1 RX
#   - on steady : RX ready and no PRBS errors in the last ~250ms -> PASS.
#   - fast blink: RX ready but PRBS errors                       -> FAIL (signal integrity/polarity).
#   - off       : GTP not initialized (QPLL not locked / no refclk / RX init stuck).
#
# Detailed BER/loop test from the host over UARTBone: see sfp_bist_test.py.
#
# Build/load:
#   ./sfp_bist.py --build --load [--linerate=1.25e9] [--loopback]
#   litex_server --uart --uart-port=/dev/ttyUSB0
#   ./sfp_bist_test.py --csr-csv=myir_sfp_bist.csv

import argparse
from types import SimpleNamespace

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex_boards.platforms import myir_myc_j7a100t

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # # #

        clk200 = platform.request("clk200")
        rst_n  = platform.request("rst_n")

        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)

# GTP Quad PLL on GTREFCLK1 ------------------------------------------------------------------------

class GTPQuadPLLRefClk1(GTPQuadPLL):
    """GTPQuadPLL variant fed from GTREFCLK1.

    liteiclink's GTPQuadPLL always uses GTREFCLK0/PLL0REFCLKSEL=0b001, but on this board the 125MHz
    SFP refclk is MGTREFCLK1 of Quad 216 (F10/E10, IBUFDS_GTE2_X0Y3); MGTREFCLK0 is the PCIe refclk.
    We reuse the "shared" path of GTPQuadPLL (config/signals only) and instantiate GTPE2_COMMON here.
    """
    def __init__(self, refclk, refclk_freq, linerate):
        GTPQuadPLL.__init__(self, refclk, refclk_freq, linerate, channel=0, shared=True)
        config = self.config
        self.specials += Instance("GTPE2_COMMON",
            # Common.
            i_GTREFCLK1       = refclk,
            i_BGBYPASSB       = 1,
            i_BGMONITORENB    = 1,
            i_BGPDB           = 1,
            i_BGRCALOVRD      = 0b11111,
            i_RCALENB         = 1,

            # DRP.
            i_DRPADDR         = self.drp.addr,
            i_DRPCLK          = self.drp.clk,
            i_DRPDI           = self.drp.di,
            o_DRPDO           = self.drp.do,
            i_DRPEN           = self.drp.en,
            o_DRPRDY          = self.drp.rdy,
            i_DRPWE           = self.drp.we,

            # PLL0 (from GTREFCLK1).
            p_PLL0_FBDIV      = config["n2"],
            p_PLL0_FBDIV_45   = config["n1"],
            p_PLL0_REFCLK_DIV = config["m"],
            i_PLL0LOCKEN      = 1,
            i_PLL0PD          = 0,
            i_PLL0REFCLKSEL   = 0b010,
            i_PLL0RESET       = self.reset,
            o_PLL0LOCK        = self.lock,
            o_PLL0OUTCLK      = self.clk,
            o_PLL0OUTREFCLK   = self.refclk,

            # PLL1 (not used: power down).
            i_PLL1PD          = 1,
        )

# GTP with PRBS enabled at reset -------------------------------------------------------------------

class BISTGTP(GTP):
    """GTP whose PRBS CSRs reset to a given mode, so the BIST runs from power-up without a host."""
    def __init__(self, *args, prbs_default=0b01, loopback_default=0b000, **kwargs):
        self._prbs_default     = prbs_default
        self._loopback_default = loopback_default
        GTP.__init__(self, *args, **kwargs)

    def add_prbs_control(self, rx_errors_width=32):
        values = [
            ("``0b00``", "PRBS   Disabled."),
            ("``0b01``", "PRBS7  Enabled."),
            ("``0b10``", "PRBS15 Enabled."),
            ("``0b11``", "PRBS31 Enabled."),
        ]
        self._tx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=values, reset=self._prbs_default),
        ])
        self._rx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=values, reset=self._prbs_default),
            CSRField("pause",  size=1, description="Pause RX PRBS."),
        ])
        self._rx_prbs_errors = CSRStatus(rx_errors_width, description="RX PRBS errors (20-bit words with at least one bit error).")
        self.comb += [
            self.tx_prbs_config.eq(self._tx_prbs_config.fields.config),
            self.rx_prbs_config.eq(self._rx_prbs_config.fields.config),
            self.rx_prbs_pause.eq(self._rx_prbs_config.fields.pause),
            self._rx_prbs_errors.status.eq(self.rx_prbs_errors),
        ]

    def add_loopback_control(self):
        self._loopback = CSRStorage(fields=[
            CSRField("config", size=3, reset=self._loopback_default, values=[
                ("``0b000``", "Normal operation."),
                ("``0b001``", "Near-End PCS Loopback."),
                ("``0b010``", "Near-End PMA Loopback."),
                ("``0b100``", "Far-End PMA Loopback."),
                ("``0b110``", "Far-End PCS Loopback."),
            ])
        ])
        self.comb += self.loopback.eq(self._loopback.fields.config)

# Stand-alone link monitor (LED + CSR) -------------------------------------------------------------

class LinkMonitor(LiteXModule):
    """Flags whether a PRBS error occurred in the last window (computed in the RX domain, so only
    single-bit signals cross to sys) and drives a LED: on = pass, fast blink = errors, off = not ready."""
    def __init__(self, serdes, sys_clk_freq, led=None, window=0.25):
        self._status = CSRStatus(fields=[
            CSRField("ready",  size=1, description="TX and RX initialized."),
            CSRField("errors", size=1, description="PRBS errors seen in the last window."),
            CSRField("ok",     size=1, description="Ready, PRBS enabled and no errors in the last window (PASS)."),
        ])

        # # #

        # RX domain: error-in-window flag.
        window_cycles = int(serdes.rx_clk_freq*window)
        timer         = Signal(max=window_cycles + 1)
        errors_last   = Signal(32)
        err_acc       = Signal()
        err_window    = Signal()
        self.sync.rx += [
            errors_last.eq(serdes.rx_prbs.errors),
            If(serdes.rx_prbs.errors != errors_last, err_acc.eq(1)),
            If(timer == 0,
                timer.eq(window_cycles),
                err_window.eq(err_acc | (serdes.rx_prbs.errors != errors_last)),
                err_acc.eq(0),
            ).Else(
                timer.eq(timer - 1),
            )
        ]

        # Sys domain.
        err_window_sys = Signal()
        self.specials += MultiReg(err_window, err_window_sys)
        ready     = Signal()
        prbs_on   = Signal()
        pass_     = Signal()
        self.comb += [
            ready.eq(serdes.tx_ready & serdes.rx_ready),
            prbs_on.eq(serdes.rx_prbs_config != 0),
            pass_.eq(ready & prbs_on & ~err_window_sys),
            self._status.fields.ready.eq(ready),
            self._status.fields.errors.eq(err_window_sys),
            self._status.fields.ok.eq(pass_),
        ]

        if led is not None:
            blink = Signal(32)
            self.sync += blink.eq(blink + 1)
            fast  = blink[log2_int(int(sys_clk_freq), need_pow2=False) - 4] # ~6Hz @ 100MHz.
            self.comb += led.eq(ready & (pass_ | (~prbs_on) | fast))

# SFP BIST SoC -------------------------------------------------------------------------------------

class SFPBISTSoC(SoCMini):
    def __init__(self, sys_clk_freq=100e6, linerate=1.25e9, prbs="prbs7", loopback=False,
        tx_polarity = (0, 0),
        rx_polarity = (0, 0)):
        platform = myir_myc_j7a100t.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteX SFP BIST on MYiR MYC-J7A100T.")

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # GTP RefClk (125MHz, MGTREFCLK1 Quad 216) -------------------------------------------------
        refclk_pads = platform.request("gtp_refclk", 1)
        refclk      = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk,
        )
        platform.add_period_constraint(refclk_pads.p, 1e9/125e6)

        # GTP Quad PLL -----------------------------------------------------------------------------
        self.qpll = qpll = GTPQuadPLLRefClk1(refclk, 125e6, linerate)
        print(qpll)

        # GTPs (SFP0/SFP1) -------------------------------------------------------------------------
        prbs_modes = {"prbs7": 0b01, "prbs15": 0b10, "prbs31": 0b11}
        leds       = [platform.request("user_led", i) for i in range(2)]
        serdes_list = []
        for i in range(2):
            sfp     = platform.request("sfp", i)
            # Platform uses txp/txn/rxp/rxn naming: alias them to the p/n names GTP expects.
            tx_pads = SimpleNamespace(p=sfp.txp, n=sfp.txn)
            rx_pads = SimpleNamespace(p=sfp.rxp, n=sfp.rxn)

            serdes = BISTGTP(qpll, tx_pads, rx_pads, sys_clk_freq,
                qpll_reset       = False, # QPLL is shared: reset driven below from both TX inits.
                data_width       = 20,
                tx_buffer_enable = True,
                rx_buffer_enable = True,
                clock_aligner    = False,
                tx_polarity      = tx_polarity[i],
                rx_polarity      = rx_polarity[i],
                prbs_default     = prbs_modes[prbs],
                loopback_default = 0b010 if loopback else 0b000,
            )
            serdes.add_stream_endpoints()
            serdes.add_controls()
            serdes.add_clock_cycles()
            setattr(self, f"serdes{i}", serdes)
            serdes_list.append(serdes)

            platform.add_period_constraint(serdes.cd_tx.clk, 1e9/serdes.tx_clk_freq)
            platform.add_period_constraint(serdes.cd_rx.clk, 1e9/serdes.rx_clk_freq)
            platform.add_false_path_constraints(self.crg.cd_sys.clk, serdes.cd_tx.clk, serdes.cd_rx.clk)

            # Idle stream (unused: TX data comes from the PRBS generator).
            self.comb += [
                serdes.sink.valid.eq(1),
                serdes.sink.ctrl.eq(0),
                serdes.sink.data.eq(0),
            ]

            # Stand-alone monitor.
            monitor = LinkMonitor(serdes, sys_clk_freq, led=leds[i])
            setattr(self, f"monitor{i}", ClockDomainsRenamer({"rx": f"serdes{i}_rx"})(monitor))

        # Shared QPLL: reset requested by either TX init FSM.
        self.comb += qpll.reset.eq(serdes_list[0].tx_init.pllreset | serdes_list[1].tx_init.pllreset)

        # Note: no LOC constraints needed: GTPE2_CHANNEL (SFP0 -> X0Y6, SFP1 -> X0Y7), GTPE2_COMMON
        # (X0Y1) and IBUFDS_GTE2 (X0Y3) placement follows from the dedicated pins in the platform.

        # Constants for the host script.
        self.add_constant("SFP_BIST_LINERATE", int(linerate))
        self.add_constant("SFP_BIST_DATA_WIDTH", 20)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SFP PRBS BIST on MYiR MYC-J7A100T.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream (to SRAM).")
    parser.add_argument("--sys-clk-freq", default=100e6,  type=float, help="System clock frequency.")
    parser.add_argument("--linerate",     default=1.25e9, type=float,
        help="Linerate from 125MHz refclk: 1.0/1.25/1.5625/1.875/2.0/2.5/3.125/3.75/4.0/5.0/6.25 Gbps.")
    parser.add_argument("--prbs",         default="prbs7", choices=["prbs7", "prbs15", "prbs31"], help="Power-up PRBS mode.")
    parser.add_argument("--loopback",     action="store_true", help="Power-up in Near-End PMA loopback (no cable).")
    parser.add_argument("--tx-polarity",  default="0,0", help="TX polarity swap for SFP0,SFP1 (e.g. 0,1).")
    parser.add_argument("--rx-polarity",  default="0,0", help="RX polarity swap for SFP0,SFP1 (e.g. 1,0).")
    args = parser.parse_args()

    soc = SFPBISTSoC(
        sys_clk_freq = args.sys_clk_freq,
        linerate     = args.linerate,
        prbs         = args.prbs,
        loopback     = args.loopback,
        tx_polarity  = tuple(int(x) for x in args.tx_polarity.split(",")),
        rx_polarity  = tuple(int(x) for x in args.rx_polarity.split(",")),
    )
    builder = Builder(soc, output_dir="build/myir_sfp_bist", csr_csv="myir_sfp_bist.csv")
    builder.build(build_name="myir_sfp_bist", run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
