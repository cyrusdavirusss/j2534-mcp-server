#!/usr/bin/env python3
"""VAG (VW/Audi/Skoda/SEAT) module scan + DTC read over the Mini VCI J2534 cable.

Built for a 2010 Audi (B8 / 8K era: UDS on CAN 500k, plus a few KWP2000-on-CAN
modules behind the CAN gateway).  Speaks ISO15765 directly, no VCDS/ODIS.

Why a sweep instead of a fixed map: on VAG the physical diagnostic CAN IDs are
not a derivable function of the module address.  The engine/trans use the
OBD-mandated 0x7E0/0x7E1 pair, while body/comfort modules use per-platform IDs
(0x7XX request + either +8 or +0x6A for the response).  So we probe candidates
and report what actually answers on THIS car.

The clone VIM tolerates exactly ONE ISO15765 flow-control filter at a time, so
every candidate gets clear-filters -> install filter -> send -> read.

Usage (32-bit python - MVCI32.dll is PE32):
  C:/Python311-32/python.exe vag_scan.py --link-check
  C:/Python311-32/python.exe vag_scan.py --scan [--out vag_scan.json] [--fast]
  C:/Python311-32/python.exe vag_scan.py --read 0x01 22F187
  C:/Python311-32/python.exe vag_scan.py --dtcs            # re-read codes on found modules
  C:/Python311-32/python.exe vag_scan.py --clear 0x01 --yes       # WRITE: clears that ECU
"""
import sys, os, time, json, csv, argparse

sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")
from J2534 import (
    pt_open, pt_close, pt_connect, pt_disconnect,
    pt_write_message, pt_read_message,
    pt_start_ecu_filter, clear_message_filters,
    set_j2534_device_to_connect,
    ProtocolId, PassThruMsg,
)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)   # embedded python does not add the script dir: needed for pid_decode
BAUD = 500000

# (address, name) - VAG "8K" (A4 B8) auto-scan list + the gateway installation
# list names seen on 2010-era 8K/8T cars.  Addresses are the VAG diagnostic
# addresses, not CAN IDs.
VAG_MODULES = [
    (0x01, "Engine"), (0x02, "Auto Trans"), (0x03, "ABS Brakes"),
    (0x04, "Steering Angle"), (0x05, "Acc/Start Auth (KESSY)"),
    (0x06, "Seat Mem. Drvr"), (0x07, "Control Head"), (0x08, "Auto HVAC"),
    (0x09, "Cent. Elect."), (0x0E, "Media Player 1"), (0x0F, "Digital Radio"),
    (0x10, "Park/Steer Assist"), (0x13, "Distance Regulation (ACC)"),
    (0x14, "Suspension Electr."), (0x15, "Airbags"), (0x16, "Steering wheel"),
    (0x17, "Instruments"), (0x18, "Aux. Heater"), (0x19, "CAN Gateway"),
    (0x1B, "Active Steering"), (0x1E, "Media Player 2"), (0x22, "AWD"),
    (0x25, "Immobilizer"), (0x26, "Auto Roof"), (0x2E, "Media Player 3"),
    (0x36, "Seat Mem. Pass"), (0x37, "Navigation"), (0x3C, "Lane Change"),
    (0x3D, "Special Function"), (0x42, "Door Drvr"), (0x44, "Steering Assist"),
    (0x46, "Central Conv."), (0x47, "Sound System"), (0x52, "Door Pass"),
    (0x53, "Parking Brake"), (0x55, "Headlight Range"), (0x56, "Radio"),
    (0x57, "TV Tuner"), (0x5C, "Lane Assist"), (0x5F, "Information Electr."),
    (0x62, "Door Rear Left"), (0x65, "Tire Pressure"), (0x67, "Voice Control"),
    (0x69, "Trailer"), (0x6C, "Back-up Cam"), (0x72, "Door Rear Right"),
    (0x76, "Park Assist 2"), (0x77, "Telephone"),
]

# Identity DIDs (UDS 0x22 <MSB><LSB>)
DIDS = [
    ("VIN_22F190", "22F190"),
    ("PartNo_22F187", "22F187"),
    ("SWver_22F189", "22F189"),
    ("Serial_22F18C", "22F18C"),
    ("HWver_22F191", "22F191"),
]


# ---------------------------------------------------------------- J2534 glue
def payload_of(msg):
    d = [msg.Data[i] for i in range(msg.DataSize)]
    if len(d) < 4:
        return None, None
    return (d[2] << 8) | d[3], d[4:]      # 11-bit id lives in CAN header bytes 2-3


def _try_open(i):
    """Open device index i; return (device_id, rc) with rc from the raw API."""
    import ctypes
    from J2534 import j2534_api, pt_get_last_error
    set_j2534_device_to_connect(i)
    did = ctypes.c_ulong()
    rc = j2534_api.PassThruOpen(ctypes.c_void_p(None), ctypes.byref(did))
    return (did.value if rc == 0 else 0), rc, pt_get_last_error()


ERR8 = ("ERR_DEVICE_NOT_CONNECTED (0x08) - the DLL loaded but sees no vehicle: "
        "cable not seated in the OBD port, ignition off, or no 12V on pin 16")


def open_device(idx=1, tries=(1, 2, 0)):
    """Try the requested index first, then the others.  Print the raw rc so a
    failure at the car is diagnosable on the spot."""
    try:
        from J2534 import J2534RegistryScanner
        sc = J2534RegistryScanner()
        for n, p in zip(sc.get_device_names(), sc.get_device_dll_paths().values()):
            print(f"  registry VIM: {n}  -> {p}", flush=True)
    except Exception as e:
        print(f"  registry scan failed: {e}", flush=True)
    for i in ([idx] + [t for t in tries if t != idx]):
        try:
            did, rc, err = _try_open(i)
        except Exception as e:
            print(f"  index {i}: exception {type(e).__name__}: {e}", flush=True)
            continue
        if did:
            print(f"device opened (j2534 index {i}, device id {did})", flush=True)
            return did
        print(f"  index {i}: PassThruOpen rc={rc} ({ERR8 if rc == 8 else (err or b'').decode(errors='replace').strip()})", flush=True)
    return None


def read_all(ch, wait_ms=250, max_msgs=120):
    out, deadline = [], time.time() + wait_ms / 1000.0
    while time.time() < deadline and len(out) < max_msgs:
        rx = PassThruMsg(0, 0)
        if pt_read_message(ch, rx, 1, 60) != 0:
            continue
        cid, p = payload_of(rx)
        if not p:
            continue
        if p[0] in (0x00, 0x30, 0x32):    # echo / flow-control -> ignore
            continue
        out.append((cid, bytes(p)))
    return out


def send(ch, tx_id, data_hex):
    msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
    msg.set_identifier_and_data(tx_id, list(bytes.fromhex(data_hex)))
    return pt_write_message(ch, msg, 1, 500)


def reassemble(msgs):
    frames = {}
    for cid, p in msgs:
        frames.setdefault(cid, []).append(p)
    out = {}
    for cid, pls in frames.items():
        buf = bytearray()
        for p in pls:
            if p[0] & 0xF0 == 0x10 and len(p) >= 2:
                total = ((p[0] & 0x0F) << 8) | p[1]
                buf = bytearray(p[2:])[:total]
            elif p[0] & 0xF0 == 0x20:
                buf += p[1:]
            else:
                buf += p
        out[cid] = bytes(buf)
    return out


def probe(ch, req_id, rx_id, data_hex, wait_ms=220, label=""):
    """clear filters -> install FC filter for this pair -> send -> read."""
    clear_message_filters(ch)
    fid = pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, rx_id, req_id, 0x40)
    if not fid:
        return None, [], f"filter rejected (rx={rx_id:03X} req={req_id:03X})"
    send(ch, req_id, data_hex)
    msgs = read_all(ch, wait_ms=wait_ms)
    return fid, msgs, ""


# ---------------------------------------------------------------- decoding
def dtc_code(b0, b1, b2):
    """VAG/ISO UDS DTC display: letter + 4 digits, then the failure-type byte (FTE).

    Validated against a VAG UDS log where raw D1 01 00 prints as "U1101 00" - the
    same shape VCDS shows ("U1123 00", "U1101 00").  Note the nibbles MUST be
    formatted hex: 0x0F is the digit "F", not decimal "15".
    """
    letter = "PCBU"[(b0 >> 6) & 0x03]
    d = f"{letter}{(b0 >> 4) & 0x03:X}{b0 & 0x0F:X}{(b1 >> 4) & 0x0F:X}{b1 & 0x0F:X}"
    return f"{d} {b2:02X}"


STATUS_BITS = [
    (0x01, "testFailed"), (0x02, "testFailedThisCycle"), (0x04, "pending"),
    (0x08, "confirmed"), (0x10, "testNotCompletedSinceClear"),
    (0x20, "testFailedSinceClear"), (0x40, "testNotCompletedThisCycle"),
    (0x80, "warningIndicatorRequested"),
]


def status_text(s):
    return ",".join(n for m, n in STATUS_BITS if s & m) or "clear"


def parse_dtcs(blob):
    """0x59 0x02 <avail mask> then 3-byte DTC + 1 status repeating."""
    if len(blob) < 3 or blob[0] != 0x59 or blob[1] != 0x02:
        return []
    body = blob[3:]
    out = []
    for i in range(0, len(body) - 3, 4):
        b0, b1, b2, st = body[i], body[i + 1], body[i + 2], body[i + 3]
        if not (b0 or b1 or b2):
            continue
        out.append({"code": dtc_code(b0, b1, b2), "raw": f"{b0:02X}{b1:02X}{b2:02X}",
                    "status": f"0x{st:02X}", "status_flags": status_text(st)})
    return out


def read_module_dtcs(ch, req, rx, wait=280):
    """Read DTCs with the RIGHT mask, and separate real faults from table entries.

    UDS mask 0x0F returns only DTCs with a failure bit set (testFailed |
    testFailedThisOperationCycle | pending | confirmed) - i.e. real faults.
    Mask 0xFF returns the ECU's entire DTC table, including every monitor that has
    never run, with status 0x40 (testNotCompletedThisOperationCycle) or 0x50
    (that plus testNotCompletedSinceLastClear).  On this car mask 0xFF produced 65
    entries and ZERO faults; reporting those as codes was wrong.
    """
    neg = None
    for mask in ("19020F", "1902FF", "1902FF00"):
        _, msgs, err = probe(ch, req, rx, mask, wait)
        for cid, b in reassemble(msgs).items():
            if b[:1] == b"\x59":
                d = parse_dtcs(b)
                faults = [e for e in d if int(e["status"], 16) & 0x0F]
                return {"mask": mask, "faults": faults, "entries": len(d), "neg": None}
            if b[:1] == b"\x7f":
                neg = b.hex(" ").upper()
    return {"mask": None, "faults": [], "entries": 0, "neg": neg}


def parse_mode03(blob):
    if len(blob) < 3 or blob[0] != 0x43:
        return []
    codes = []
    body = blob[2:]
    for i in range(0, len(body) - 1, 2):
        hi, lo = body[i], body[i + 1]
        if not (hi or lo):
            continue
        codes.append(f"{'PCBU'[(hi >> 6) & 3]}{(hi >> 4) & 3}{hi & 0xF}{lo >> 4}{lo & 0xF}")
    return codes


# ------------------------------------------------- OBD-II (mode 01/02/03/06)
OBD_LIVE_PIDS = [
    ("0101", "monitor status since clear"), ("0103", "fuel system status"),
    ("0104", "calculated engine load %"), ("0105", "coolant temp C"),
    ("0106", "STFT bank1 %"), ("0107", "LTFT bank1 %"),
    ("010B", "intake MAP kPa"), ("010C", "RPM"), ("010E", "timing advance"),
    ("010F", "intake air temp C"), ("0110", "MAF g/s"), ("0111", "throttle %"),
    ("0114", "O2 B1S1 V"), ("0115", "O2 B1S2 V"), ("011C", "OBD standard"),
    ("012E", "evap purge %"), ("013C", "cat temp B1S1 C"), ("013E", "cat temp B1S2 C"),
    ("0142", "module voltage V"), ("0143", "absolute load %"), ("0149", "APP sensor D %"),
    ("0155", "STFT secondary O2 bank1 %"), ("0156", "LTFT secondary O2 bank1 %"),
]

MISFIRE_MIDS = {0xA0: "misfire general", 0xA1: "misfire total",
                0xA2: "misfire cylinder 1", 0xA3: "misfire cylinder 2",
                0xA4: "misfire cylinder 3", 0xA5: "misfire cylinder 4",
                0xA6: "misfire cylinder 5", 0xA7: "misfire cylinder 6"}
MISFIRE_TIDS = {0x0B: "EWMA count, last 10 cycles", 0x0C: "count last+current cycle"}


def parse_mode06(blob):
    """Parse a 0x46 response into one row per MID/TID group.

    Frame shape seen on this car (VAG MED17): 46 <MID> <TID> <7-byte record> [<MID> <TID> <7-byte record> ...]
    with the 7-byte record = UAS | value_hi | value_lo | min_hi | min_lo | max_hi | max_lo.
    Some monitors return a 3-byte value and no limits.  The raw hex is always
    reported too because the UAS scaling byte is partly vendor-defined.
    """
    if not blob or blob[0] != 0x46 or len(blob) < 4:
        return []
    out, i = [], 1
    while i + 1 < len(blob):
        mid, tid = blob[i], blob[i + 1]
        i += 2
        n = min(7, len(blob) - i)
        r = blob[i:i + n]
        i += n
        if n >= 7:
            uas, v, mn, mx = r[0], (r[1] << 8) | r[2], (r[3] << 8) | r[4], (r[5] << 8) | r[6]
        elif n == 3:
            uas, v, mn, mx = None, (r[0] << 16) | (r[1] << 8) | r[2], None, None
        else:
            continue
        out.append({
            "mid": f"0x{mid:02X}", "mid_name": MISFIRE_MIDS.get(mid, ""),
            "tid": f"0x{tid:02X}", "tid_name": MISFIRE_TIDS.get(tid, ""),
            "uas": f"0x{uas:02X}" if uas is not None else "-",
            "value": v, "min": mn, "max": mx,
            "verdict": "-" if mn is None else ("PASS" if mn <= v <= mx else "FAIL/out-of-range"),
        })
    return out


def obd_session(ch, wait=380, live=True, freeze=True, mode06=True):
    """Generic OBD-II session: works on any car, VIN + codes + freeze frame + live + mode 06."""
    rep = {"vin": None, "supported": None, "codes": {}, "freeze": {}, "live": {},
           "mode06": {}, "voltage_v": None}

    def ask(req, ms=wait):
        clear_message_filters(ch)
        pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40)
        send(ch, 0x7DF, req)
        m = read_all(ch, ms)
        return reassemble(m)

    print("=== VIN / basics ===", flush=True)
    for cid, b in ask("0902", 900).items():
        if b[:2] == b"\x49\x02":
            vin = "".join(chr(x) for x in b[3:] if 32 <= x < 127)
            rep["vin"] = vin
            print(f"  VIN: {vin}", flush=True)

    print("\n=== fault codes (03 = confirmed, 07 = pending, 0A = permanent) ===", flush=True)
    for mode, name in (("03", "confirmed"), ("07", "pending"), ("0A", "permanent")):
        for cid, b in ask(mode).items():
            codes = []
            if b[0] == 0x40 | int(mode, 16):
                body = b[2:]
                for i in range(0, len(body) - 1, 2):
                    hi, lo = body[i], body[i + 1]
                    if not (hi or lo):
                        continue
                    codes.append(f"{'PCBU'[(hi >> 6) & 3]}{(hi >> 4) & 3}{hi & 0xF}{lo >> 4}{lo & 0xF}")
            rep["codes"][name] = codes
            print(f"  {name:9s} {cid:03X}: {codes or 'none'}", flush=True)

    if freeze:
        print("\n=== freeze frame (mode 02) ===", flush=True)
        for cid, b in ask("0200").items():
            rep["freeze"]["DTC"] = b.hex(" ").upper()
            print(f"  frame DTC raw: {b.hex(' ').upper()}", flush=True)
        for pid in ("0102", "010C", "0104", "0105", "0106", "0107", "0110", "0111", "0142", "010F", "0114"):
            for cid, b in ask("02" + pid[2:]).items():
                if b[:1] == b"\x42":
                    rep["freeze"][pid] = b.hex(" ").upper()
                    d = decode_obd(pid[2:], b[3:])
                    print(f"  {pid}: {b.hex(' ').upper()}  {d}", flush=True)

    if live:
        print("\n=== live data (engine running or cranking) ===", flush=True)
        names = {r[2:]: n for r, n in OBD_LIVE_PIDS}
        sup = set()
        for page in ("0100", "0120", "0140"):
            for cid, b in ask(page, wait).items():
                if b[:1] == b"\x41" and b[1:2] == bytes([int(page[2:], 16)]):
                    bits, base = b[2:6], int(page[2:], 16) + 1
                    if len(bits) >= 4:
                        sup |= {base + i for i in range(32) if bits[i // 8] & (0x80 >> (i % 8))}
        if sup:
            print(f"  supported PIDs 01-60: {', '.join(f'{p:02X}' for p in sorted(sup))}", flush=True)
        for pid in sorted(sup or {int(r[2:], 16) for r, _ in OBD_LIVE_PIDS}):
            req = f"01{pid:02X}"
            for cid, b in ask(req, wait).items():
                if b[:1] != b"\x41":
                    continue
                d = decode_obd(req[2:], b[2:])
                rep["live"][req] = {"raw": b.hex(" ").upper(), "decoded": d}
                print(f"  {req} {names.get(req[2:], ''):26s} {b.hex(' ').upper():16s} {d}", flush=True)
                break

    if mode06:
        print("\n=== mode 06 on-board monitoring (misfire story) ===", flush=True)
        targets = [("06", "all")] + [(f"06{mid:02X}", MISFIRE_MIDS[mid]) for mid in (0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7)]
        for req, name in targets:
            for cid, b in ask(req, 1400 if req == "06" else 600).items():
                if b[:1] != b"\x46":
                    continue
                rep["mode06"][req] = b.hex(" ").upper()
                rows = parse_mode06(b)
                for r in rows:
                    print(f"  {r['mid']} {r['mid_name']:20s} {r['tid']} {r['tid_name']:28s} "
                          f"value={r['value']} min={r['min']} max={r['max']} {r['verdict']}", flush=True)
                if not rows:
                    print(f"  {req} raw: {b.hex(' ').upper()}", flush=True)
    return rep


def decode_obd(pid, payload):
    """Decode using the MCP server's SAE J1979 table; fall back to raw."""
    try:
        from pid_decode import decode_pid
        res = decode_pid("01", pid, payload.hex(" "))
        if "error" in res:
            return res["error"]
        return "; ".join(
            f"{s.get('name', s.get('id'))}={s.get('value', s.get('error'))}{s.get('unit', '')}"
            for s in res.get("signals", [])) or payload.hex(" ").upper()
    except Exception as e:
        return f"(no decoder: {type(e).__name__}: {e}) {payload.hex(' ').upper()}"


def selftest():
    """Prove the parsers on canned bytes - no car, no cable needed."""
    print("--- mode 06 misfire parse (0xA2 cyl1, TID 0x0B, value 6) ---")
    blob = bytes([0x46, 0xA2, 0x0B, 0x01, 0x00, 0x06, 0x00, 0x00, 0xFF, 0xFF])
    for r in parse_mode06(blob):
        print("   ", r)
    print("--- UDS 19 02 DTC parse (P030100 status 0x08 confirmed) ---")
    for r in parse_dtcs(bytes([0x59, 0x02, 0xFF, 0x03, 0x01, 0x00, 0x08])):
        print("   ", r)
    print("--- mode 03 DTC parse (P0301, P0420) ---")
    b = bytes([0x43, 0x02, 0x03, 0x01, 0x04, 0x20])
    body = b[2:]
    print("   ", [f"{'PCBU'[(body[i] >> 6) & 3]}{(body[i] >> 4) & 3}{body[i] & 0xF}{body[i+1] >> 4}{body[i+1] & 0xF}"
                   for i in range(0, len(body), 2)])
    print("--- PID decode via the MCP J1979 table ---")
    for pid, hx, expect in (("0C", "09 59", "858 rpm-ish"), ("05", "6C", "68 C"),
                            ("10", "00 3C", "MAF"), ("42", "3C", "12 V")):
        print(f"   01{pid} {hx}: {decode_obd(pid, bytes.fromhex(hx))}")


# ---------------------------------------------------------------- modes
def id_sweep(ch, lo=0x708, hi=0x795, wait=200, include_obd=True):
    """Discover which diagnostic CAN ID pairs this gateway actually answers on.

    Response ID = request + 0x6A was proven on this car by the airbag module
    (0x715/0x77F), and the OBD pair 0x7E0..0x7E7 / +8 covers powertrain.  VAG
    request IDs are NOT a function of the module's VAG address, so a sweep beats
    a fixed table.
    """
    found = {}
    targets = [(0x7E0 + n, 0x7E8 + n) for n in range(8)] + [(r, r + 0x6A) for r in range(lo, hi + 1)]
    targets = [(r, x) for r, x in targets if x <= 0x7FF]
    print(f"sweeping {len(targets)} request/response pairs, {wait} ms each...", flush=True)
    for req, rx in targets:
        fid, msgs, err = probe(ch, req, rx, "3E00", wait)
        if not msgs:
            continue
        tp = reassemble(msgs).get(rx, b"").hex(" ").upper()
        entry = {"req": f"{req:03X}", "rx": f"{rx:03X}", "tester_present": tp, "dids": {}}
        for label, did in DIDS:
            _, m2, _ = probe(ch, req, rx, did, wait)
            for cid, b in reassemble(m2).items():
                if b[:1] == b"\x62":
                    entry["dids"][label] = b.hex(" ").upper()
        rd = read_module_dtcs(ch, req, rx, max(wait, 280))
        if rd["mask"]:
            entry["dtc_read"] = rd
        elif rd["neg"]:
            entry["neg"] = rd["neg"]
        found[f"{req:03X}"] = entry
        pn = entry["dids"].get("PartNo_22F187", "")
        txt = "".join(chr(int(x, 16)) for x in pn.split() if 32 <= int(x, 16) < 127) if pn else ""
        codes = ", ".join(f"{e['code']}({e['status_flags']})" for e in entry.get("dtc_read", {}).get("faults", [])) or "-"
        tested = entry.get("dtc_read", {}).get("entries", 0)
        print(f"  {req:03X}/{rx:03X}  {txt:16s} faults={codes}  [{tested} entries in the ECU table]", flush=True)
    return found


def link_check(dev):
    ch = pt_connect(dev, ProtocolId.ISO15765, 0, BAUD)
    print("connect:", ch, flush=True)
    if not ch:
        return
    print("\n[functional 7DF 0100 - generic OBD, works on any live bus]", flush=True)
    clear_message_filters(ch)
    print("  filter:", pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40), flush=True)
    send(ch, 0x7DF, "0100")
    msgs = read_all(ch, 900)
    for cid, p in msgs:
        print(f"  rx {cid:03X}  {p.hex(' ').upper()}", flush=True)
    if not msgs:
        print("  (nothing - expected with no car / ignition off on the OBD port)", flush=True)
    print("\n[VAG 0x719 gateway TesterPresent - car must be awake]", flush=True)
    _, msgs, err = probe(ch, 0x719, 0x721, "3E00", 400)
    print(f"  {'ERR ' + err if err else ('rx ' + ', '.join(f'{c:03X}' for c, _ in msgs) or 'no answer')}", flush=True)
    pt_disconnect(ch)


def scan(ch, fast=False):
    report = {"when": time.strftime("%Y-%m-%d %H:%M:%S"), "bus": "ISO15765 500k",
              "modules": {}, "functional": {}, "notes": []}
    wait = 130 if fast else 220

    print("=== 1. functional probes (0x7DF) ===", flush=True)
    for label, req in [("OBD_0100", "0100"), ("OBD_03", "0300"), ("UDS_1902FF", "1902FF00"),
                       ("OBD_0902_VIN", "0902")]:
        clear_message_filters(ch)
        pt_start_ecu_filter(ch, ProtocolId.ISO15765, 0xFFFFFFFF, 0x7E8, 0x7E0, 0x40)
        send(ch, 0x7DF, req)
        msgs = read_all(ch, 600 if label == "OBD_0902_VIN" else 350)
        got = {}
        for cid, b in reassemble(msgs).items():
            got[f"{cid:03X}"] = b.hex(" ").upper()
        report["functional"][label] = got
        print(f"  {label:14s} -> {sorted(got) or 'no answer'}", flush=True)
        if label == "OBD_03":
            for k, v in got.items():
                c = parse_mode03(bytes.fromhex(v.replace(' ', '')))
                if c:
                    report["functional"]["OBD_03_codes"] = c
                    print(f"      mode03 {k}: {c}", flush=True)

    print("\n=== 2. module sweep ===", flush=True)
    for addr, name in VAG_MODULES:
        cands = [(0x700 + addr, 0x700 + addr + 0x08, "req=700+a rx=+8"),
                 (0x700 + addr, 0x700 + addr + 0x6A, "req=700+a rx=+6A")]
        if addr <= 0x07:                                  # OBD-mandated pair
            cands.insert(0, (0x7E0 + addr - 1, 0x7E8 + addr - 1, "OBD 7E0+n"))
        found = None
        for req, rx, tag in cands:
            if req > 0x7FF or rx > 0x7FF:
                continue
            fid, msgs, err = probe(ch, req, rx, "3E00", wait, name)
            if err:
                report["notes"].append(f"{name}: {err}")
                break
            if msgs:
                found = {"addr": f"{addr:02X}", "name": name, "req": f"{req:03X}",
                         "rx": f"{rx:03X}", "id_convention": tag,
                         "tester_present": msgs[0][1].hex(" ").upper()}
                break
        if not found:
            print(f"  {addr:02X} {name:22s} -", flush=True)
            continue

        print(f"  {addr:02X} {name:22s} + {found['req']}/{found['rx']} ({found['id_convention']})", flush=True)
        found["dids"] = {}
        for label, did in DIDS:
            _, msgs, err = probe(ch, int(found["req"], 16), int(found["rx"], 16), did, wait, label)
            for cid, b in reassemble(msgs).items():
                if b[:1] == b"\x62":
                    found["dids"][label] = b.hex(" ").upper()
                    txt = "".join(chr(x) if 32 <= x < 127 else "." for x in b[3:])
                    print(f"      {label:14s} {b.hex(' ').upper()}  {txt}", flush=True)
        _, msgs, err = probe(ch, int(found["req"], 16), int(found["rx"], 16), "1902FF", wait, "DTCs")
        for cid, b in reassemble(msgs).items():
            d = parse_dtcs(b)
            if d:
                found["dtcs"] = d
                for e in d:
                    print(f"      DTC {e['code']:8s} raw={e['raw']} status={e['status']} ({e['status_flags']})", flush=True)
            elif b[:1] == b"\x7f":
                # VAG wants a 3-byte request; retry the 4-byte form before giving up
                _, msgs2, _ = probe(ch, int(found["req"], 16), int(found["rx"], 16), "1902FF00", wait, "DTCs")
                for cid2, b2 in reassemble(msgs2).items():
                    d2 = parse_dtcs(b2)
                    if d2:
                        found["dtcs"] = d2
                        for e in d2:
                            print(f"      DTC {e['code']:8s} raw={e['raw']} status={e['status']} ({e['status_flags']})", flush=True)
                    elif b2[:1] == b"\x7f":
                        found["dtc_error"] = b2.hex(" ").upper()
                        print(f"      DTC read negative: {b2.hex(' ').upper()}", flush=True)
        report["modules"][f"{addr:02X}"] = found

    print(f"\nmodules answering: {len(report['modules'])}/{len(VAG_MODULES)}", flush=True)
    return report


def read_one(ch, addr, req_hex):
    # 0x01-0x07 are the OBD powertrain pair (0x7E0+n / 0x7E8+n); everything else on
    # this car answers, if at all, at request 0x700+addr with response request+0x6A.
    req = 0x700 + addr
    pairs = [(0x7E0 + addr - 1, 0x7E8 + addr - 1)] if 1 <= addr <= 7 else []
    pairs += [(req, req + 8), (req, req + 0x6A)]
    for rq, rx in pairs:
        _, msgs, err = probe(ch, rq, rx, req_hex, 500)
        if msgs:
            for cid, b in reassemble(msgs).items():
                print(f"  {rq:03X} -> {cid:03X}  {b.hex(' ').upper()}  ({len(b)} bytes)", flush=True)
            return
    print("  no answer", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link-check", action="store_true")
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--obd", action="store_true",
                    help="generic OBD-II engine session: VIN, codes 03/07/0A, freeze frame, live data, mode 06 misfires")
    ap.add_argument("--selftest", action="store_true", help="run the parsers on canned bytes (no car, no cable)")
    ap.add_argument("--mode06", action="store_true", help="only the mode 06 misfire/monitor read (fast)")
    ap.add_argument("--id-sweep", action="store_true",
                    help="sweep diagnostic CAN ID pairs (req 0x7XX -> resp req+0x6A, plus OBD 0x7E0..7E7)")
    ap.add_argument("--dtcs", action="store_true", help="re-read DTCs from a previous scan json")
    ap.add_argument("--read", nargs=2, metavar=("ADDR", "REQ"), help="e.g. --read 0x01 22F187")
    ap.add_argument("--clear", metavar="ADDR", help="UDS 14FFFFFF on that module (needs --yes)")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--device", type=int, default=1, help="J2534 device index (1=Autokent 1.4.3, 2=2.0.1)")
    ap.add_argument("--out", default=os.path.join(HERE, "vag_scan.json"))
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--in", dest="infile", default=os.path.join(HERE, "vag_scan.json"))
    a = ap.parse_args()

    if a.selftest:
        selftest()
        return

    dev = open_device(a.device)
    if not dev:
        print("FAILED to open any J2534 device", flush=True)
        sys.exit(1)

    if a.link_check:
        link_check(dev)
        pt_close(dev)
        return

    ch = pt_connect(dev, ProtocolId.ISO15765, 0, BAUD)
    if not ch:
        print("FAILED to connect ISO15765 channel", flush=True)
        pt_close(dev)
        sys.exit(1)

    try:
        if a.id_sweep:
            rep = {"when": time.strftime("%Y-%m-%d %H:%M:%S"), "found": id_sweep(ch)}
            out = a.out.rsplit(".", 1)[0] + "_ids.json"
            json.dump(rep, open(out, "w"), indent=2)
            print(f"\nmodules answering: {len(rep['found'])}\nJSON: {out}", flush=True)
        elif a.mode06:
            rep = obd_session(ch, live=False, freeze=False)
            out = a.out.rsplit(".", 1)[0] + "_m06.json"
            json.dump(rep, open(out, "w"), indent=2)
            print(f"\nJSON: {out}", flush=True)
        elif a.obd:
            rep = obd_session(ch)
            out = a.out.rsplit(".", 1)[0] + "_obd.json"
            json.dump(rep, open(out, "w"), indent=2)
            print(f"\nJSON: {out}", flush=True)
        elif a.read:
            addr = int(a.read[0], 16)
            read_one(ch, addr, a.read[1])
        elif a.clear:
            if not a.yes:
                print("refusing: add --yes to clear DTCs on the car", flush=True)
            else:
                addr = int(a.clear, 16)
                read_one(ch, addr, "14FFFFFF")
                print("  clear sent (14 FF FF FF)", flush=True)
        elif a.dtcs:
            data = json.load(open(a.infile))
            mods = data.get("modules") or data.get("found") or {}
            for k, m in mods.items():
                req, rx = int(m["req"], 16), int(m["rx"], 16)
                rd = read_module_dtcs(ch, req, rx, 300)
                m["dtc_read"] = rd
                label = f"{m['req']}/{m['rx']}"
                if rd["faults"]:
                    for e in rd["faults"]:
                        print(f"  {label}  {e['code']}  raw={e['raw']} status={e['status']} ({e['status_flags']})", flush=True)
                elif rd["mask"]:
                    print(f"  {label}  no faults  [mask {rd['mask']}, {rd['entries']} table entries, "
                          f"none with a failure bit set]", flush=True)
                else:
                    print(f"  {label}  no answer / negative: {rd['neg']}", flush=True)
            json.dump(data, open(a.infile, "w"), indent=2)
        else:
            rep = scan(ch, a.fast)
            json.dump(rep, open(a.out, "w"), indent=2)
            csvp = a.out.rsplit(".", 1)[0] + ".csv"
            with open(csvp, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["addr", "name", "req", "rx", "dtc", "status_flags", "raw"])
                for k, m in rep["modules"].items():
                    if m.get("dtcs"):
                        for e in m["dtcs"]:
                            w.writerow([m["addr"], m["name"], m["req"], m["rx"], e["code"],
                                        e["status_flags"], e["raw"]])
                    else:
                        w.writerow([m["addr"], m["name"], m["req"], m["rx"], "", "", ""])
            print(f"\nJSON: {a.out}\nCSV : {csvp}", flush=True)
    finally:
        pt_disconnect(ch)
        pt_close(dev)


if __name__ == "__main__":
    main()
