#!/usr/bin/env python3
"""Toyota enhanced-diagnostic scanner: build the real ECU map over J2534.

Speaks ISO15765 (11-bit, 500k) directly - TesterPresent + ReportDTCByStatusMask
(0x19 02 FF 00) - to every standard logical address, then to functional 7DF,
reassembling ISO-TP multi-frame answers, so we get the actual ECUs and their codes
without Techstream's GUI.

Usage: C:/Python311-32/python.exe toyota_scan.py [outfile]
"""
import sys, os, time, json

sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")
from J2534 import (
    pt_open, pt_close, pt_connect, pt_disconnect,
    pt_write_message, pt_read_message,
    pt_start_ecu_filter, clear_message_filters,
    set_j2534_device_to_connect,
    ProtocolId, PassThruMsg,
)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "toyota_scan.json")
TX_IDS = [0x7E0, 0x7E1, 0x7E2, 0x7E3, 0x7E4, 0x7E5, 0x7E6, 0x7E7, 0x7DF]
RX_IDS = [0x7E8, 0x7E9, 0x7EA, 0x7EB, 0x7EC, 0x7ED, 0x7EE, 0x7EF]

REQUESTS = [
    ("TesterPresent", "3E00"),
    ("ReadDTC_19_02_FF00", "1902FF00"),
    ("ReadECUIdent_1A87", "1A87"),
]


def payload_of(msg):
    d = [msg.Data[i] for i in range(msg.DataSize)]
    if len(d) < 4:
        return None, None
    can_id = (d[2] << 8) | d[3]           # 11-bit CAN id sits in bytes 2-3 of the CAN header
    return can_id, d[4:]


def read_all(ch, wait_ms=250, max_msgs=60):
    """Drain every available message; return list of (can_id, payload)."""
    out, deadline = [], time.time() + wait_ms / 1000.0
    while time.time() < deadline and len(out) < max_msgs:
        rx = PassThruMsg(0, 0)
        if pt_read_message(ch, rx, 1, 60) != 0:
            continue
        cid, p = payload_of(rx)
        if cid is None:
            continue
        if not p:
            continue
        if p[0] in (0x00, 0x30, 0x32):      # echo / flow-control frames: ignore
            continue
        out.append((cid, p))
    return out


def send(ch, tx_id, data_hex):
    payload = bytes.fromhex(data_hex)
    msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
    msg.set_identifier_and_data(tx_id, list(payload))
    return pt_write_message(ch, msg, 1, 500)


def reassemble(msgs):
    """Group (can_id, payload) into ISO-TP reassembled byte strings per CAN id."""
    frames = {}
    for cid, p in msgs:
        frames.setdefault(cid, []).append(p)
    out = {}
    for cid, pls in frames.items():
        buf = bytearray()
        for p in pls:
            if p[0] & 0xF0 == 0x10 and len(p) >= 2:          # first frame
                total = ((p[0] & 0x0F) << 8) | p[1]
                buf = bytearray(p[2:])
                buf = buf[:total] if len(buf) > total else buf
            elif p[0] & 0xF0 == 0x20:                        # consecutive frame
                buf += p[1:]
            else:                                            # single frame / UDS
                buf += p
        out[cid] = bytes(buf)
    return out


def decode_dtc_bytes(b):
    """Pull 3-byte DTC entries out of a 0x59 response payload."""
    codes = []
    if not b or b[0] != 0x59 or len(b) < 3:
        return codes
    body = b[3:]
    for i in range(0, len(body) - 2, 3):
        hi, mid, lo = body[i], body[i + 1], body[i + 2]
        if hi == 0 and mid == 0 and lo == 0:
            continue
        letter = "PCBU"[(hi >> 6) & 0x03]
        codes.append(f"{letter}{(hi >> 4) & 0x03}{hi & 0x0F}{mid >> 4}{mid & 0x0F}{lo:02X}")
    return codes


def main():
    set_j2534_device_to_connect(1)          # MVCI Autokent 1.4.3
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
    nfilt = 0
    for rx in RX_IDS:                        # pass + flow-control per responder id
        fid = pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, rx, rx - 8, 0x40)
        if fid:
            nfilt += 1
    print(f"filters installed: {nfilt}/8", flush=True)

    report = {"filters": nfilt, "probes": {}, "functional": {}}

    for tx in TX_IDS:
        for label, req in REQUESTS:
            if label == "ReadECUIdent_1A87" and tx != 0x7DF:
                continue                     # ECU-ident only via functional
            send(ch, tx, req)
            msgs = read_all(ch, wait_ms=200)
            if not msgs:
                continue
            for cid, b in reassemble(msgs).items():
                key = f"{label}@tx{tx:03X}"
                entry = report["probes"].setdefault(key, {"responses": {}})
                entry["responses"][f"{cid:03X}"] = b.hex(" ").upper()
                dtcs = decode_dtc_bytes(b)
                if dtcs:
                    entry["dtcs"] = dtcs
            print(f"  tx {tx:03X} {label:24s} -> " +
                  ", ".join(f"{c:03X}" for c in reassemble(msgs)), flush=True)

    send(ch, 0x7DF, "1902FF00")
    msgs = read_all(ch, wait_ms=800, max_msgs=120)
    for cid, b in reassemble(msgs).items():
        report["functional"][f"{cid:03X}"] = b.hex(" ").upper()
        d = decode_dtc_bytes(b)
        if d:
            report["functional"][f"{cid:03X}_dtcs"] = d

    with open(OUT, "w") as f:
        json.dump(report, f, indent=2)

    print("\n=== ECUs that answered (any probe) ===", flush=True)
    cids = set()
    for v in report["probes"].values():
        cids.update(v["responses"].keys())
    cids.update(k for k in report["functional"] if not k.endswith("_dtcs"))
    print("  ", sorted(cids) or "none", flush=True)
    print(f"raw JSON: {OUT}", flush=True)

    pt_disconnect(ch)
    pt_close(did)


if __name__ == "__main__":
    main()
