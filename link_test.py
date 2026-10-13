#!/usr/bin/env python3
"""Minimal J2534 link test: raw writes/reads, one filter at a time.

Usage: C:/Python311-32/python.exe link_test.py
"""
import sys, time

sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")
from J2534 import (
    pt_open, pt_close, pt_connect, pt_disconnect,
    pt_write_message, pt_read_message,
    pt_start_ecu_filter, clear_message_filters,
    set_j2534_device_to_connect,
    ProtocolId, PassThruMsg,
)


def show(rx):
    d = [rx.Data[i] for i in range(rx.DataSize)]
    return f"size={rx.DataSize} rxstatus=0x{rx.RxStatus:04X} data={' '.join(f'{b:02X}' for b in d)}"


def drain(ch, ms=400, tag=""):
    got = 0
    end = time.time() + ms / 1000.0
    while time.time() < end:
        rx = PassThruMsg(0, 0)
        r = pt_read_message(ch, rx, 1, 80)
        if r == 0:
            print(f"    [{tag}] {show(rx)}", flush=True)
            got += 1
    if not got:
        print(f"    [{tag}] (no messages)", flush=True)
    return got


def tx(ch, can_id, data_hex, tag=""):
    m = PassThruMsg(ProtocolId.ISO15765, 0x40)
    m.set_identifier_and_data(can_id, list(bytes.fromhex(data_hex)))
    w = pt_write_message(ch, m, 1, 500)
    print(f"  write tx={can_id:03X} {data_hex:10s} flags=0x40 -> {w}   [{tag}]", flush=True)
    return w


set_j2534_device_to_connect(1)
did = pt_open()
print("open:", did, flush=True)
if not did:
    sys.exit(1)
ch = pt_connect(did, ProtocolId.ISO15765, 0, 500000)
print("connect:", ch, flush=True)

print("\n--- A: single FC filter 7E8/7E0, functional 0100 ---", flush=True)
clear_message_filters(ch)
print("  filter:", pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40), flush=True)
tx(ch, 0x7DF, "0100", "functional")
drain(ch, 800, "A-7DF-0100")

print("\n--- B: same filter, PHYSICAL 7E0 0100 ---", flush=True)
tx(ch, 0x7E0, "0100", "physical")
drain(ch, 800, "B-7E0-0100")

print("\n--- C: physical 7E0 TesterPresent 3E00 ---", flush=True)
tx(ch, 0x7E0, "3E00", "physical")
drain(ch, 600, "C-7E0-3E00")

print("\n--- D: 8 filters installed, functional 0100 ---", flush=True)
clear_message_filters(ch)
for rx in range(0x7E8, 0x7F0):
    fid = pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, rx, rx - 8, 0x40)
    print(f"  filter {rx:03X}: {fid}", flush=True)
tx(ch, 0x7DF, "0100", "functional-8filters")
drain(ch, 800, "D-7DF-0100")

print("\n--- E: pass-all filter (mask 0, pattern 0, fc 7E0), functional 0100 ---", flush=True)
clear_message_filters(ch)
print("  filter:", pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0x00000000, 0x00000000, 0x7E0, 0x40), flush=True)
tx(ch, 0x7DF, "0100", "passall")
drain(ch, 800, "E-passall")

pt_disconnect(ch)
pt_close(did)
print("\ndone", flush=True)
