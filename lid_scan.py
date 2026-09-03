#!/usr/bin/env python3
"""Scan Toyota enhanced mode 0x21 (ReadDataByLocalIdentifier) LIDs over J2534.

Toyota's 1VD-FTV diesel ECU supports enhanced mode 0x21: request = 0x21 + 2-byte
LID (big-endian), response = 0x61 + LID + data bytes. This sweeps a LID range and
logs which LIDs respond and the raw bytes, so we can identify turbo vane position /
actuator parameters empirically.

Usage: C:/Python311-32/python.exe lid_scan.py <start_hex> <end_hex> [timeout_ms]
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

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lid_scan.csv")

def send_recv(ch, payload_bytes, tx_hex="7E0", timeout=50):
    msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
    msg.set_identifier_and_data(int(tx_hex, 16), list(payload_bytes))
    w = pt_write_message(ch, msg, 1, 300)
    if w is False:
        return None
    for _ in range(8):
        rx = PassThruMsg(0, 0)
        r = pt_read_message(ch, rx, 1, timeout)
        if r != 0:
            return None
        if rx.DataSize < 5:
            continue
        data = [rx.Data[i] for i in range(rx.DataSize)]
        payload = data[4:]
        if payload and payload[0] in (0x61, 0x7F):
            return payload
    return None

def main():
    start = int(sys.argv[1], 16)
    end = int(sys.argv[2], 16)
    timeout = int(sys.argv[3]) if len(sys.argv) > 3 else 50

    set_j2534_device_to_connect(1)  # MVCI Autokent 1.4.3
    did = pt_open()
    if not did:
        print("FAILED open"); sys.exit(1)
    ch = pt_connect(did, ProtocolId.ISO15765, 0, 500000)
    if not ch:
        print("FAILED connect"); pt_close(did); sys.exit(1)
    clear_message_filters(ch)
    pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40)

    rows = []
    t0 = time.time()
    print(f"Scanning LID 0x{start:04X}..0x{end:04X} ...", flush=True)
    for lid in range(start, end + 1):
        req = [0x21, (lid >> 8) & 0xFF, lid & 0xFF]
        clear_receive_buffer(ch)
        resp = send_recv(ch, req, timeout=timeout)
        if resp and resp[0] == 0x61:
            # positive response: [0x61, lid_hi, lid_lo, data...]
            data_hex = " ".join(f"{b:02X}" for b in resp[3:])
            rows.append((f"{lid:04X}", data_hex))
            print(f"  LID {lid:04X} -> {data_hex}", flush=True)

    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lid_hex", "data_hex"])
        w.writerows(rows)

    pt_disconnect(ch); pt_close(did)
    print(f"Done in {time.time()-t0:.1f}s: {len(rows)} responding LIDs -> {OUT}", flush=True)

if __name__ == "__main__":
    main()
