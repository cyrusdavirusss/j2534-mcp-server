#!/usr/bin/env python3
"""High-rate ECM sampler for stall / bad-ground diagnostics.

Samples module voltage (0142), engine speed (010C) and coolant (0105) as fast as the
VIM allows (~3 Hz per PID), timestamps every read, and logs missing responses as empty
fields. A COMMUNICATIONS DROPOUT during cranking = the ECM reset = rock-solid evidence
of an intermittent supply/ground fault.

Usage:
    C:/Python311-32/python.exe stall_sampler.py <seconds> <leadin> [outfile]

Run AFTER the MCP server has closed the device (single-owner on the cable).
"""
import sys, time, csv, os

sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")
from J2534 import (
    pt_open, pt_close, pt_connect, pt_disconnect,
    pt_write_message, pt_read_message,
    pt_start_ecu_filter, clear_message_filters,
    set_j2534_device_to_connect,
    ProtocolId, PassThruMsg,
)

HERE = os.path.dirname(os.path.abspath(__file__))
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
LEADIN = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
OUT = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "stall_log.csv")

PIDS = [("0142", "module_V"), ("010C", "rpm"), ("0105", "coolant_C")]


def send_recv(ch, data_hex, tx_hex="7DF", timeout=120):
    """Send one mode-01 request, return the payload list (e.g. [0x41,0x42,0x36,0x9B]) or None."""
    payload = bytes.fromhex(data_hex)
    msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
    msg.set_identifier_and_data(int(tx_hex, 16), list(payload))
    if pt_write_message(ch, msg, 1, 500) is False:
        return None
    for _ in range(6):
        rx = PassThruMsg(0, 0)
        if pt_read_message(ch, rx, 1, timeout) != 0:
            return None
        if rx.DataSize < 5:
            continue
        data = [rx.Data[i] for i in range(rx.DataSize)]
        p = data[4:]
        if p and (p[0] == 0x41 or p[0] == 0x7F):
            return p
    return None


def decode(pid_hex, p):
    if not p or len(p) < 3 or p[0] != 0x41:
        return None
    if pid_hex == "0142":
        return round(((p[2] << 8) | p[3]) / 1000.0, 3)       # volts
    if pid_hex == "010C":
        return round(((p[2] << 8) | p[3]) / 4.0, 1)           # rpm
    if pid_hex == "0105":
        return p[2] - 40                                      # deg C
    return None


def main():
    set_j2534_device_to_connect(1)  # MVCI Autokent 1.4.3 (index 2 fails on this cable)
    did = pt_open()
    if not did:
        print("FAILED to open device", flush=True)
        sys.exit(1)
    ch = pt_connect(did, ProtocolId.ISO15765, 0, 500000)
    if not ch:
        print("FAILED to connect channel", flush=True)
        pt_close(did)
        sys.exit(1)
    clear_message_filters(ch)
    pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7DF, 0x40)

    print(f"READY - sampling starts in {LEADIN:.0f}s, runs {SECONDS:.0f}s", flush=True)
    time.sleep(LEADIN)

    rows, miss = [], {k: 0 for _, k in PIDS}
    t0 = time.time()
    print("SAMPLING NOW", flush=True)
    while time.time() - t0 < SECONDS:
        t = round(time.time() - t0, 3)
        row = {"t_sec": t}
        for pid, name in PIDS:
            v = decode(pid, send_recv(ch, pid))
            row[name] = "" if v is None else v
            if v is None:
                miss[name] += 1
        rows.append(row)
        if len(rows) % 10 == 0:
            print(f"  t={t:6.1f}s  {rows[-1]}", flush=True)

    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t_sec", "module_V", "rpm", "coolant_C"])
        w.writeheader()
        w.writerows(rows)

    vs = [r["module_V"] for r in rows if r["module_V"] != ""]
    rs = [r["rpm"] for r in rows if r["rpm"] != ""]
    print("\n=== SUMMARY ===", flush=True)
    print(f"samples: {len(rows)}   rows w/ voltage: {len(vs)}", flush=True)
    if vs:
        print(f"module_V  min={min(vs):.2f}  max={max(vs):.2f}  first={vs[0]:.2f}  last={vs[-1]:.2f}", flush=True)
    if rs:
        print(f"rpm       min={min(rs):.0f}  max={max(rs):.0f}  first={rs[0]:.0f}  last={rs[-1]:.0f}", flush=True)
    print(f"NO-RESPONSE counts (dropouts): {miss}", flush=True)
    print(f"CSV: {OUT}", flush=True)

    pt_disconnect(ch)
    pt_close(did)


if __name__ == "__main__":
    main()
