#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Host side of the MYC-J7A100T SFP PCS bench (see sfp_pcs_bench.py): prints, for each cage, the PCS
# status (link_up, is_sgmii) and the decoded link partner configuration word, only when it changes
# (plus a heartbeat line every --heartbeat seconds).
#
#   litex_server --uart --uart-port=/dev/ttyUSB0
#   ./sfp_pcs_bench_test.py --csr-csv=myir_sfp_pcs_bench.csv

import time
import argparse
from datetime import datetime

from litex import RemoteClient

SGMII_SPEEDS = {0b00: "10M", 0b01: "100M", 0b10: "1000M", 0b11: "rsvd"}

def decode_config(cfg, is_sgmii):
    if cfg == 0:
        return "no config word received"
    ack = "ACK" if cfg & (1 << 14) else "---"
    if cfg & 1:
        # SGMII (PHY -> MAC): bit15 link, bit12 duplex, bits11:10 speed.
        link   = "link" if cfg & (1 << 15) else "nolink"
        duplex = "FD" if cfg & (1 << 12) else "HD"
        speed  = SGMII_SPEEDS[(cfg >> 10) & 0b11]
        return f"SGMII  {link:6s} {speed:5s} {duplex} {ack}"
    else:
        # 1000BASE-X (Clause 37): bit5 FD, bit6 HD, bits8:7 pause, bits13:12 remote fault, bit15 NP.
        fd    = "FD" if cfg & (1 << 5) else "--"
        hd    = "HD" if cfg & (1 << 6) else "--"
        pause = (cfg >> 7) & 0b11
        rf    = (cfg >> 12) & 0b11
        return f"BASE-X {fd} {hd} pause={pause} rf={rf} {ack}"

def main():
    parser = argparse.ArgumentParser(description="MYC-J7A100T SFP PCS bench (host side).")
    parser.add_argument("--csr-csv",   default="myir_sfp_pcs_bench.csv", help="CSR configuration file.")
    parser.add_argument("--port",      default=1234, type=int,           help="litex_server port.")
    parser.add_argument("--interval",  default=0.1,  type=float,         help="Polling interval (s).")
    parser.add_argument("--heartbeat", default=5.0,  type=float,         help="Print status anyway every N s.")
    parser.add_argument("--reset",     default=None, type=int,           help="Pulse PHY reset of SFP<n> then exit.")
    args = parser.parse_args()

    bus = RemoteClient(csr_csv=args.csr_csv, port=args.port)
    bus.open()

    modes = {}
    for name in bus.constants.d.keys():
        for i in range(2):
            prefix = f"sfp{i}_pcs_mode_"
            if name.startswith(prefix):
                modes[i] = name[len(prefix):]
    print("PCS modes:", ", ".join(f"SFP{i}={modes.get(i, '?')}" for i in range(2)))

    regs = [getattr(bus.regs, f"sfp{i}_pcs_status") for i in range(2)]

    if args.reset is not None:
        rst = getattr(bus.regs, f"sfp{args.reset}_reset")
        rst.write(1); time.sleep(0.01); rst.write(0)
        print(f"SFP{args.reset} PHY reset pulsed.")
        bus.close()
        return

    last      = [None, None]
    changes   = [0, 0]
    last_beat = 0
    try:
        while True:
            now = time.time()
            for i, reg in enumerate(regs):
                v        = reg.read()
                link_up  = v & 1
                is_sgmii = (v >> 1) & 1
                cfg      = (v >> 16) & 0xffff
                state    = (link_up, is_sgmii, cfg)
                if state != last[i]:
                    changes[i] += 1
                if state != last[i] or (now - last_beat) >= args.heartbeat:
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"{ts} SFP{i} [{modes.get(i, '?'):10s}] link_up={link_up} is_sgmii={is_sgmii} "
                          f"config=0x{cfg:04x} ({decode_config(cfg, is_sgmii)}) changes={changes[i]}", flush=True)
                last[i] = state
            if (now - last_beat) >= args.heartbeat:
                last_beat = now
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    bus.close()

if __name__ == "__main__":
    main()
