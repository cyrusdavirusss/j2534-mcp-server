#!/usr/bin/env python3
"""Fast APP pedal sweep sampler - reads 0x49 (APP D) and 0x4A (APP E)
continuously and logs raw bytes + ratio to CSV.

Usage: C:/Python311-32/python.exe sweep_sampler.py <seconds>
Run AFTER the MCP server has closed the device.
"""
import sys, time, csv, os

sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")
from J2534 import (
    pt_open, pt_close, pt_connect, pt_disconnect,
    pt_write_message, pt_read_message,
    pt_start_ecu_filter, clear_message_filters, clear_receive_buffer,
    set_j2534_device_to_connect,
    ProtocolId, PassThruMsg,
)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweep_log.csv")
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
LEADIN = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

def send_recv(ch, data_hex, tx_hex="7E0", timeout=300):
    payload = bytes.fromhex(data_hex)
    msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
    msg.set_identifier_and_data(int(tx_hex, 16), list(payload))
    w = pt_write_message(ch, msg, 1, 500)
    if w is False:
        return None
    # drain until we get the actual response (skip echo / flow control frames)
    for _ in range(8):
        rx = PassThruMsg(0, 0)
        r = pt_read_message(ch, rx, 1, timeout)
        if r != 0:
            return None
        if rx.DataSize < 5:
            continue
        data = [rx.Data[i] for i in range(rx.DataSize)]
        # data layout: [00 00 07 E8] header + ISO-TP payload
        payload = data[4:]
        if payload and (payload[0] == 0x41 or payload[0] == 0x7F):
            return payload  # e.g. [0x41, 0x49, XX]
    return None

def main():
    set_j2534_device_to_connect(1)  # MVCI Autokent 1.4.3
    did = pt_open()
    if not did:
        print("FAILED to open device")
        sys.exit(1)

    ch = pt_connect(did, ProtocolId.ISO15765, 0, 500000)
    if not ch:
        print("FAILED to connect channel")
        pt_close(did)
        sys.exit(1)

    clear_message_filters(ch)
    pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40)

    rows = []
    print(f"Get ready... sweep starts in {LEADIN}s", flush=True)
    time.sleep(LEADIN)
    t0 = time.time()
    print(f"Sampling for {SECONDS}s ... SWEEP THE PEDAL SLOWLY NOW", flush=True)
    while time.time() - t0 < SECONDS:
        t = time.time() - t0
        d = send_recv(ch, "0149")
        e = send_recv(ch, "014A")
        dv = d[2] if d and len(d) >= 3 else None
        ev = e[2] if e and len(e) >= 3 else None
        if dv is not None and ev is not None:
            ratio = (ev / dv) if dv else 0
            rows.append((round(t, 3), dv, ev, round(ratio, 3)))

    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "APP_D_raw", "APP_E_raw", "E_D_ratio"])
        w.writerows(rows)

    pt_disconnect(ch)
    pt_close(did)
    print(f"Done. {len(rows)} samples written to {OUT}", flush=True)

if __name__ == "__main__":
    main()
