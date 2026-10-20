#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# SFP 1000BASE-X/SGMII PCS bench for the MYiR MYC-J7A100T.
#
# One A7_1000BASEX PHY per SFP cage, each built with its own PCS mode, sharing the Quad 216 QPLL
# (125MHz MGTREFCLK1). No MAC/CPU: the goal is to see what a module (copper SFP, media converter...)
# does at PCS level, through the PCS status CSRs read over UARTBone:
#   - link_up, is_sgmii, config_reg (link partner configuration word).
# With two cages built in two different modes, one bitstream tests two configurations: move the
# module (and its cable) from one cage to the other.
#
# Copper partner: the PC, or simply the board's own RJ45 port (the RGMII PHYs are released from
# reset here so they autonegotiate on their own).
#
# LEDs: user_led0 = SFP0 link_up, user_led1 = SFP1 link_up.
#
# Build/load:
#   ./sfp_pcs_bench.py --sfp0-mode=sgmii --sfp1-mode=basex --build --load
#   litex_server --uart --uart-port=/dev/ttyUSB0
#   ./sfp_pcs_bench_test.py --csr-csv=myir_sfp_pcs_bench.csv
#
# Modes:
#   auto        : sgmii=None  (1000BASE-X words first, switch to SGMII if partner says so), autoneg.
#   basex       : sgmii=False (1000BASE-X only), autoneg (Clause 37).
#   basex-noan  : sgmii=False, no autoneg (fixed 1000BASE-X, link = valid code-groups).
#   sgmii       : sgmii=True  (SGMII MAC side from the start), autoneg.
#   sgmii-noan  : sgmii=True, no autoneg (fixed 1000Mbps SGMII).

import argparse

from migen import *

from litex.gen import *

from litex_boards.platforms import myir_myc_j7a100t

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy.serial.gtp_7series import QPLL, QPLLSettings
from liteeth.phy.serial.basex.wrappers.a7_gtp import A7_1000BASEX

PCS_MODES = {
    "auto"       : dict(sgmii=None,  with_autoneg=True),
    "basex"      : dict(sgmii=False, with_autoneg=True),
    "basex-noan" : dict(sgmii=False, with_autoneg=False),
    "sgmii"      : dict(sgmii=True,  with_autoneg=True),
    "sgmii-noan" : dict(sgmii=True,  with_autoneg=False),
}

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

# Shared QPLL channel ------------------------------------------------------------------------------

class _SharedQPLLChannel:
    """Per-PHY view of one QPLL channel: clk/refclk/lock are shared, each PHY gets its own reset
    request, ORed into the real QPLL reset (two PMAs driving the same reset signal would conflict)."""
    def __init__(self, channel):
        self.index  = channel.index
        self.clk    = channel.clk
        self.refclk = channel.refclk
        self.lock   = channel.lock
        self.reset  = Signal()

# SFP PCS Bench SoC --------------------------------------------------------------------------------

class SFPPCSBenchSoC(SoCMini):
    def __init__(self, sys_clk_freq=100e6, modes=("sgmii", "basex"), release_rgmii_phys=True):
        platform = myir_myc_j7a100t.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident = f"LiteX SFP PCS bench on MYiR MYC-J7A100T (SFP0 {modes[0]} / SFP1 {modes[1]}).")

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # RGMII PHYs out of reset (so the board RJ45 ports can be used as copper link partner) -----
        if release_rgmii_phys:
            for i in range(2):
                self.comb += platform.request("eth", i).rst_n.eq(1)

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

        # QPLL (PLL0 from GTREFCLK1: 125MHz -> 2.5GHz VCO -> 1.25Gbps) -----------------------------
        self.qpll = qpll = QPLL(
            gtrefclk1     = refclk,
            qpllsettings0 = QPLLSettings(refclksel=0b010, fbdiv=4, fbdiv_45=5, refclk_div=1),
        )
        qpll_channel = qpll.channels[0]

        # PHYs -------------------------------------------------------------------------------------
        leds     = [platform.request("user_led", i) for i in range(2)]
        channels = []
        for i in range(2):
            channel = _SharedQPLLChannel(qpll_channel)
            channels.append(channel)
            pcs_kwargs = dict(PCS_MODES[modes[i]])
            pcs_kwargs.update(with_csr=True, sys_clk_freq=sys_clk_freq)
            phy = A7_1000BASEX(
                qpll_channel = channel,
                data_pads    = platform.request("sfp", i),
                sys_clk_freq = sys_clk_freq,
                pcs_kwargs   = pcs_kwargs,
                # TX clock: the default PLL+BUFH must sit in the GTs' clock-region row, which has
                # a single PLL: only SFP0 can use it, SFP1 goes through a BUFG (no region rule).
                tx_cm_buf_type = "BUFH" if i == 0 else "BUFG",
            )
            # Each PHY defines eth_tx/eth_rx(/_half) clock domains: give them unique names.
            phy = ClockDomainsRenamer({
                "eth_tx"      : f"sfp{i}_tx",
                "eth_rx"      : f"sfp{i}_rx",
                "eth_tx_half" : f"sfp{i}_tx_half",
                "eth_rx_half" : f"sfp{i}_rx_half",
            })(phy)
            setattr(self, f"sfp{i}", phy)

            # No MAC: TX idle, RX discarded.
            self.comb += [
                phy.sink.valid.eq(0),
                phy.source.ready.eq(1),
                leds[i].eq(phy.link_up),
            ]

            # Constrain the GTP output clocks (62.5MHz); Vivado derives the TX/RX PLL outputs.
            platform.add_period_constraint(phy.txoutclk, 1e9/(phy.tx_clk_freq/2))
            platform.add_period_constraint(phy.rxoutclk, 1e9/(phy.rx_clk_freq/2))
            platform.add_false_path_constraints(self.crg.cd_sys.clk, phy.cd_eth_tx.clk, phy.cd_eth_rx.clk)

        # Shared QPLL reset: requested by either TX init FSM.
        self.comb += qpll_channel.reset.eq(channels[0].reset | channels[1].reset)

        # Modes as constants for the host script.
        for i in range(2):
            self.add_constant(f"SFP{i}_PCS_MODE_" + modes[i].upper().replace("-", "_"))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SFP PCS bench on MYiR MYC-J7A100T.")
    parser.add_argument("--build",        action="store_true", help="Build bitstream.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream (to SRAM).")
    parser.add_argument("--sys-clk-freq", default=100e6, type=float, help="System clock frequency.")
    parser.add_argument("--sfp0-mode",    default="sgmii", choices=PCS_MODES.keys(), help="PCS mode of SFP0.")
    parser.add_argument("--sfp1-mode",    default="basex", choices=PCS_MODES.keys(), help="PCS mode of SFP1.")
    parser.add_argument("--no-rgmii-release", action="store_true", help="Leave the RGMII PHYs reset pins undriven.")
    args = parser.parse_args()

    soc = SFPPCSBenchSoC(
        sys_clk_freq       = args.sys_clk_freq,
        modes              = (args.sfp0_mode, args.sfp1_mode),
        release_rgmii_phys = not args.no_rgmii_release,
    )
    builder = Builder(soc, output_dir="build/myir_sfp_pcs_bench", csr_csv="myir_sfp_pcs_bench.csv")
    builder.build(build_name="myir_sfp_pcs_bench", run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
