#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2025 Samuele Baisi <samuele.baisi@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Host side of the MYC-J7A100T SFP BIST (see sfp_bist.py).
#
# Prerequisite (in another terminal):
#   litex_server --uart --uart-port=/dev/ttyUSB0
#
# Examples:
#   ./sfp_bist_test.py                          # DAC SFP0<->SFP1, PRBS7, loop forever (Ctrl-C to stop).
#   ./sfp_bist_test.py --loopback               # Near-End PMA loopback (no cable).
#   ./sfp_bist_test.py --mode=prbs31 --duration=600 --max-ber=1e-12
#   ./sfp_bist_test.py --inject                 # Self-check: verify the checker really detects errors.
#
# Exit code: 0 = PASS, 1 = FAIL (useful in a scripted smoke test).

import sys
import time
import argparse

from litex import RemoteClient

PRBS_MODES = {"disabled": 0b00, "prbs7": 0b01, "prbs15": 0b10, "prbs31": 0b11}
PRBS_PAUSE = 0b100
LOOPBACK   = {"none": 0b000, "near-pcs": 0b001, "near-pma": 0b010, "far-pma": 0b100, "far-pcs": 0b110}

# SerDes -------------------------------------------------------------------------------------------

class SerDes:
    def __init__(self, bus, n):
        self.n    = n
        self.name = f"SFP{n}"
        found = False
        for k, v in bus.regs.d.items():
            if k.startswith(f"serdes{n}_"):
                setattr(self, k[len(f"serdes{n}_"):], v)
                found = True
            if k.startswith(f"monitor{n}_"):
                setattr(self, "monitor_" + k[len(f"monitor{n}_"):], v)
        if not found:
            raise ValueError(f"serdes{n} not present in design")
        self.errors_last = None

    def measure_clocks(self, duration=1.0):
        self.clock_latch.write(1)
        tx0, rx0 = self.clock_tx_cycles.read(), self.clock_rx_cycles.read()
        time.sleep(duration)
        self.clock_latch.write(1)
        tx1, rx1 = self.clock_tx_cycles.read(), self.clock_rx_cycles.read()
        return ((tx1 - tx0) % 2**32)/(duration*1e6), ((rx1 - rx0) % 2**32)/(duration*1e6)

    def configure(self, mode, loopback, tx_polarity=None, rx_polarity=None):
        if tx_polarity is not None:
            self.tx_polarity.write(tx_polarity)
        if rx_polarity is not None:
            self.rx_polarity.write(rx_polarity)
        self.loopback.write(LOOPBACK[loopback])
        self.tx_prbs_config.write(PRBS_MODES[mode])
        self.rx_prbs_config.write(PRBS_MODES["disabled"]) # Clears error counter.
        self.rx_prbs_config.write(PRBS_MODES[mode])
        self.mode = mode

    def restart_rx(self):
        self.rx_enable.write(0)
        time.sleep(0.01)
        self.rx_enable.write(1)

    def ready(self):
        return bool(self.tx_ready.read() & self.rx_ready.read())

    def read_errors_delta(self):
        # Pause the counter so the 32-bit value crosses clock domains consistently.
        self.rx_prbs_config.write(PRBS_PAUSE | PRBS_MODES[self.mode])
        errors = self.rx_prbs_errors.read()
        self.rx_prbs_config.write(PRBS_MODES[self.mode])
        delta = 0 if self.errors_last is None else (errors - self.errors_last) % 2**32
        self.errors_last = errors
        return delta

# Test ---------------------------------------------------------------------------------------------

def wait_ready(serdes_list, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if all(s.ready() for s in serdes_list):
            return True
        time.sleep(0.05)
    return False

def main():
    parser = argparse.ArgumentParser(description="MYC-J7A100T SFP PRBS BIST (host side).")
    parser.add_argument("--csr-csv",   default="myir_sfp_bist.csv", help="CSR configuration file.")
    parser.add_argument("--port",      default=1234, type=int,     help="litex_server port.")
    parser.add_argument("--serdes",    default="0,1",              help="SerDes to test (default: 0,1).")
    parser.add_argument("--mode",      default="prbs7", choices=["prbs7", "prbs15", "prbs31"], help="PRBS mode.")
    parser.add_argument("--loopback",  action="store_true",        help="Near-End PMA loopback (no cable).")
    parser.add_argument("--interval",  default=1.0, type=float,    help="Report interval (s).")
    parser.add_argument("--duration",  default=0,   type=float,    help="Test duration in s (0 = forever).")
    parser.add_argument("--max-ber",   default=0,   type=float,    help="Max BER for PASS (default 0: any error = FAIL).")
    parser.add_argument("--inject",    action="store_true",        help="Self-check: mismatch TX/RX PRBS and expect errors.")
    args = parser.parse_args()

    bus = RemoteClient(csr_csv=args.csr_csv, port=args.port)
    bus.open()
    ident = ""
    try:
        for i in range(256):
            c = bus.read(bus.bases.identifier_mem + 4*i) & 0xff
            if c == 0: break
            ident += chr(c)
    except Exception:
        pass
    print(f"Design: {ident}")

    linerate   = getattr(bus.constants, "sfp_bist_linerate", 1.25e9)
    data_width = getattr(bus.constants, "sfp_bist_data_width", 20)
    print(f"Linerate: {linerate/1e9:.4f} Gbps, PRBS: {args.mode.upper()}, "
          f"{'Near-End PMA loopback' if args.loopback else 'external (DAC SFP0<->SFP1)'}")

    serdes_list = [SerDes(bus, int(n)) for n in args.serdes.split(",")]

    # Clocks.
    expected = linerate/data_width/1e6
    for s in serdes_list:
        tx, rx = s.measure_clocks()
        print(f"{s.name}: TX clk {tx:8.3f} MHz / RX clk {rx:8.3f} MHz (expected {expected:.3f} MHz)")

    # Configure.
    for s in serdes_list:
        s.configure(args.mode, "near-pma" if args.loopback else "none")
    for s in serdes_list:
        s.restart_rx()
    if not wait_ready(serdes_list):
        for s in serdes_list:
            print(f"{s.name}: tx_ready={s.tx_ready.read()} rx_ready={s.rx_ready.read()}")
        print("FAIL: transceivers not ready (QPLL lock / refclk / RX init).")
        bus.close()
        sys.exit(1)

    # Self-check: TX PRBS7 vs RX PRBS31 -> checker must report errors.
    if args.inject:
        for s in serdes_list:
            s.tx_prbs_config.write(PRBS_MODES["prbs7"] if args.mode != "prbs7" else PRBS_MODES["prbs31"])
        time.sleep(0.2)
        for s in serdes_list:
            s.read_errors_delta()
        time.sleep(0.5)
        ok = True
        for s in serdes_list:
            d = s.read_errors_delta()
            print(f"{s.name}: injected-mismatch errors = {d} -> {'OK (checker alive)' if d else 'NOT DETECTED'}")
            ok &= d > 0
            s.tx_prbs_config.write(PRBS_MODES[args.mode])
        bus.close()
        sys.exit(0 if ok else 1)

    # Run.
    time.sleep(0.2) # Let PRBS checkers lock.
    for s in serdes_list:
        s.read_errors_delta()
    totals = {s.name: 0 for s in serdes_list}
    t0     = time.time()
    try:
        while True:
            time.sleep(args.interval)
            elapsed = time.time() - t0
            line = f"[{elapsed:8.1f}s]"
            for s in serdes_list:
                d = s.read_errors_delta()
                totals[s.name] += d
                ready = "RDY" if s.ready() else "---"
                # Errors are counted per 20-bit word: upper bound on BER = words/bits.
                ber = totals[s.name]/(elapsed*linerate)
                line += f"  {s.name} {ready} err {d:8d} tot {totals[s.name]:10d} BER<={ber:.2e}"
            print(line, flush=True)
            if args.duration and elapsed >= args.duration:
                break
    except KeyboardInterrupt:
        print()

    elapsed = time.time() - t0
    bits    = elapsed*linerate
    passed  = True
    print("-"*80)
    for s in serdes_list:
        ber = totals[s.name]/bits
        ok  = (totals[s.name] == 0) if args.max_ber == 0 else (ber <= args.max_ber)
        passed &= ok
        src = s.name if args.loopback else f"SFP{1 - s.n}->{s.name}"
        floor = f"(no errors in {bits:.2e} bits: BER < ~{3/bits:.1e} @95%)" if totals[s.name] == 0 else ""
        print(f"{src:12s}: {'PASS' if ok else 'FAIL'}  errors {totals[s.name]}  BER<={ber:.2e} {floor}")
    print("RESULT:", "PASS" if passed else "FAIL")
    bus.close()
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
