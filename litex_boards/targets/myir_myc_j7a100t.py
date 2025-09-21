#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Notes: 
# - default clock set to 100MHz, above that timings start to crap out

from migen import *

from litex.gen import *

from litex_boards.platforms import myir_myc_j7a100t

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOIn

# Video
from litex.soc.cores.video import VideoVGAPHY
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.integration.soc import SoCRegion

# DDR 3
from litedram.modules import MT41J128M16
from litedram.phy import s7ddrphy

# ETH
from liteeth.phy.s7rgmii import LiteEthPHYRGMII

# SFP
from liteeth.phy import A7_1000BASEX

# PCIE
from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.phy.usppciephy import USPHBMPCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge
from litepcie.software import generate_litepcie_software

import sys

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_dram=True, with_video_pll=False, with_video_in_pll=False, with_rst=True):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        #self.cd_eth = ClockDomain()
        if with_dram:
            self.cd_sys4x     = ClockDomain()
            self.cd_sys4x_dqs = ClockDomain()
            self.cd_idelay    = ClockDomain()

        # # #

        # Clk/Rst.
        clk200 = platform.request("clk200")

        rst    = ~platform.request("rst_n") if with_rst else 0

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(rst | self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        if with_dram:
            self.dram_pll = dram_pll = S7PLL(speedgrade=-1)
            dram_pll.register_clkin(self.cd_sys.clk, sys_clk_freq)
            dram_pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
            dram_pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
            dram_pll.create_clkout(self.cd_idelay,    200e6)

            # IdelayCtrl.
            self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        # Video PLL
        if with_video_pll:
            self.video_mmcm = video_mmcm = S7MMCM(speedgrade=-1)
            self.comb += video_mmcm.reset.eq(rst | self.rst)
            video_mmcm.register_clkin(self.cd_sys.clk, sys_clk_freq)
            self.cd_vga = ClockDomain()
            video_mmcm.create_clkout(self.cd_vga, 25.175e6)

        # Video IN PLL
        if with_video_in_pll:
            self.mmcm = mmcm = S7MMCM(speedgrade=-1)
            self.comb += self.mmcm.reset.eq(rst | self.rst)
            mmcm.register_clkin(self.cd_sys.clk, sys_clk_freq)
            self.cd_vga_in = ClockDomain()
            mmcm.create_clkout(self.cd_vga_in, 25.175e6)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, toolchain="vivado", sys_clk_freq=100e6,
        with_ethernet   = False,
        with_etherbone  = False,
        eth_ip          = "192.168.1.50",
        remote_ip       = None,
        eth_dynamic_ip  = False,
        # with_spi_flash  = False,
        with_buttons    = False,
        with_led_chaser = True,
        with_video_terminal    = False,
        with_video_framebuffer = False,
        with_video_colorbars = False,
        with_video_in = False,
        with_sfp = False,
        with_pcie = False,
        with_sdram_256 = False,
        **kwargs):
        platform = myir_myc_j7a100t.Platform(toolchain=toolchain)

        # CRG --------------------------------------------------------------------------------------
        with_dram = (kwargs.get("integrated_main_ram_size", 0) == 0)
        with_video_pll = with_video_terminal or with_video_framebuffer or with_video_colorbars
        with_video_in_pll = with_video_in
        self.crg  = _CRG(platform, sys_clk_freq, with_dram, with_video_pll, with_video_in_pll)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on MYiR MYC-J7A100T.", **kwargs)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            from litedram.common import PHYPadsCombiner
            if (with_sdram_256):
                self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram", 0),
                    memtype        = "DDR3",
                    nphases        = 4,
                    sys_clk_freq   = sys_clk_freq)
            else:
                self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram", 1),
                    memtype        = "DDR3",
                    nphases        = 4,
                    sys_clk_freq   = sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41J128M16(sys_clk_freq, "1:4"),
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks", 0),
                pads       = self.platform.request("eth", 0))
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, with_ethmac=with_ethernet)
            elif with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip, local_ip=eth_ip, remote_ip=remote_ip)
        
        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq,
            )

         # Video ------------------------------------------------------------------------------------
         # TODO: **NOT WORKING YET** needs another attempt with the bitbanged i2c init or a better approach
        if with_video_terminal or with_video_framebuffer or with_video_colorbars:
            hdmi_pads = platform.request("hdmi_out")
            self.comb += hdmi_pads.rst_n.eq(1)
            self.videophy = VideoVGAPHY(hdmi_pads, clock_domain="vga")
            self.videoi2c = I2CMaster(hdmi_pads)
            self.videoi2c.add_init(addr=0x72, init=[
                # TODO (look at examples or in the (unofficial) chip docs) (check the address too)
            ])

            if with_video_terminal:
                self.add_video_terminal(phy=self.videophy, timings="640x480@60Hz", clock_domain="vga")
            elif with_video_framebuffer:
                self.add_video_framebuffer(phy=self.videophy, timings="640x480@60Hz", clock_domain="vga")
            elif with_video_colorbars:
                self.add_video_colorbars(phy=self.videophy, timings="640x480@60Hz", clock_domain="vga")

        # # SFP Ethernet -------------------------------------------------------------------------------
        if with_sfp:
            # shamefully copied and adapted from alientek_davincipro and acorn board files (still not working/untestable)
            from liteeth.phy.a7_gtp import QPLLSettings, QPLL

            sfp_pads = self.platform.request("sfp", 0)
            qpll_settings = QPLLSettings(
                    refclksel  = 0b010,
                    fbdiv      = 4,
                    fbdiv_45   = 5,
                    refclk_div = 1)

            refclk125 = self.platform.request("gtp_refclk", 1)
            refclk125_se = Signal()
            self.specials += \
                Instance("IBUFDS_GTE2",
                    i_CEB = 0,
                    i_I   = refclk125.p,
                    i_IB  = refclk125.n,
                    o_O   = refclk125_se,
                    # p_CLKRCV_TRST = 0b1,
                    # p_CLKCM_CFG = 0b1,
                    p_CLKSWING_CFG = 0b11)
            qpll = QPLL(refclk125_se, qpll_settings)

            self.submodules += qpll

            self.sfpethphy = A7_1000BASEX(
                qpll_channel   = qpll.channels[0],
                data_pads      = sfp_pads,
                sys_clk_freq   = sys_clk_freq,
                #rx_cm_buf_type = 'BUFG',
                #tx_cm_buf_type = 'BUFG'
                #rx_polarity  = 0,
                #tx_polarity  = 1,
                )
            # FIXME: necessary cause all the instances end up unconstrained and (luckily) get placed on MGTP_B216_TX_P3, where these pads are.
            #        This is **DANGEROUS** and there must be a better way to do it, as usual do as I say, not as I do
            #platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks UCIO-1]")

            self.add_ethernet(phy=self.sfpethphy, dynamic_ip=eth_dynamic_ip, local_ip=eth_ip, remote_ip=remote_ip)

        # # TODO: find/port module
        # SPI Flash --------------------------------------------------------------------------------
        # needs module added (or a similar one found) to LiteSPI and testing
        # if with_spi_flash:
        #     from litespi.modules import MX25L25645G
        #     from litespi.opcodes import SpiNorFlashOpCodes as Codes
        #     self.add_spi_flash(name='spiflash', mode="4x", module=MX25L25645G(Codes.READ_1_1_4_4B), rate="1:1", with_master=True)


        # FIXME: NEEDS A SECOND CHECK
        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            assert self.csr_data_width == 32
            # pcie_config = {
            #     # "Component_Name"     : "pcie_s7",
            #     "Link_Speed"         : "2.5_GT/s",
            # }
            # PHY
            self.pcie_phy   = S7PCIEPHY(platform, platform.request("pcie_x2"),
                data_width  = 64,
                refclk_freq = 100e6)

            # self.pcie_phy.update_config(pcie_config)

            self.add_pcie(phy=self.pcie_phy, ndmas=1)


        # Buttons ----------------------------------------------------------------------------------
        if with_buttons:
            self.buttons = GPIOIn(
                pads     = platform.request_all("user_btn"),
                with_irq = self.irq.enabled
            )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=myir_myc_j7a100t.Platform, description="LiteX SoC on MYiR MYC-J7A100T.")
    parser.add_target_argument("--flash",          action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--sys-clk-freq",   default=100e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-ethernet",  action="store_true",       help="Enable Ethernet support.")
    parser.add_target_argument("--with-etherbone", action="store_true",       help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",         default="192.168.1.50",    help="Ethernet/Etherbone IP address.")
    parser.add_target_argument("--remote-ip",      default="192.168.1.100",   help="Remote IP address of TFTP server.")
    parser.add_target_argument("--eth-dynamic-ip", action="store_true",       help="Enable dynamic Ethernet IP addresses setting.")
    parser.add_target_argument("--with-sdram-256", action="store_true",       help="Enable a single DDR chip only (256MB)")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",       action="store_true",       help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",           action="store_true",       help="Enable SDCard support.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    viopts.add_argument("--with-video-terminal",    action="store_true",       help="Enable Video Terminal (HDMI).")
    viopts.add_argument("--with-video-framebuffer", action="store_true",       help="Enable Video Framebuffer (HDMI).")
    viopts.add_argument("--with-video-colorbars",   action="store_true",       help="Enable Video Colorbars (HDMI).")
    viinopts = parser.target_group.add_mutually_exclusive_group()
    viinopts.add_argument("--with-video-in",        action="store_true",       help="Enable Video IN (HDMI)")
    # parser.add_target_argument("--with-spi-flash",  action="store_true",       help="Enable SPI Flash (MMAPed).")
    parser.add_argument("--with-sfp",        action="store_true",       help="Enable SFP eth support.")
    parser.add_argument("--with-pcie",       action="store_true",       help="Enable PCIE support.")
    parser.add_target_argument("--driver",       action="store_true",       help="Generate PCIe driver.")
    args = parser.parse_args()

    #assert not (args.with_etherbone and args.eth_dynamic_ip)

    soc = BaseSoC(
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        with_ethernet          = args.with_ethernet,
        with_etherbone         = args.with_etherbone,
        eth_ip                 = args.eth_ip,
        remote_ip              = args.remote_ip,
        eth_dynamic_ip         = args.eth_dynamic_ip,
        # with_spi_flash         = args.with_spi_flash,
        with_video_terminal    = args.with_video_terminal,
        with_video_framebuffer = args.with_video_framebuffer,
        with_video_colorbars   = args.with_video_colorbars,
        with_video_in          = args.with_video_in,
        with_sfp               = args.with_sfp,
        with_pcie              = args.with_pcie,
        with_sdram_256         = args.with_sdram_256,
        **parser.soc_argdict
    )

    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.with_pcie and args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
