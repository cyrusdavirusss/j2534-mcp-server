#!/usr/bin/env python3
"""Scan dtc_unified.json for missing codes across valid OBD-II ranges."""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
db = json.load(open(os.path.join(HERE, "dtc_unified.json")))

def pcode(n):
    return f"P{n:04X}"

def ccode(n):
    return f"C{n:04X}"

def bcode(n):
    return f"B{n:04X}"

def ucode(n):
    return f"U{n:04X}"

# Standard valid ranges (SAE J2012):
# P0000-P00FF (with P00A0-P00FF mostly turbo/aux), P0100-P04FF, P0500-P06FF,
# P0700-P09FF (transmission), P0A00-P0AFF (hybrid), P0B00-P0CFF, P0D00-P0DFF,
# P1000-P19FF (mfr specific), P2000-P29FF, P2A00-P2FFF, P3000-P33FF.
# We'll scan P0000-P2FFF as the "generic + common mfr" span and report holes.

ranges = [
    (0x0000, 0x00FF, "P00xx (fuel/air metering + turbo/aux)"),
    (0x0100, 0x04FF, "P01xx-P04xx (fuel/air, ignition, emissions)"),
    (0x0500, 0x06FF, "P05xx-P06xx (idle/speed, computer output)"),
    (0x0700, 0x09FF, "P07xx-P09xx (transmission)"),
    (0x0A00, 0x0AFF, "P0Axx (hybrid)"),
    (0x0B00, 0x0CFF, "P0Bxx-P0Cxx (hybrid/EV)"),
    (0x0D00, 0x0DFF, "P0Dxx (hybrid/EV)"),
    (0x1000, 0x19FF, "P10xx-P19xx (manufacturer specific)"),
    (0x2000, 0x29FF, "P20xx-P29xx (fuel/air aux + emissions)"),
    (0x2A00, 0x2FFF, "P2Axx-P2Fxx (hybrid + fuel/air)"),
    (0x3000, 0x33FF, "P30xx-P33xx (mfr specific)"),
]

total_missing = 0
for lo, hi, label in ranges:
    holes = [pcode(n) for n in range(lo, hi+1) if pcode(n) not in db]
    if holes:
        # Collapse into contiguous spans
        spans = []
        start = prev = lo
        for n in range(lo, hi+1):
            code = pcode(n)
            if code in db:
                if start <= prev and start != n:  # only close a span if a hole preceded
                    spans.append((pcode(start), pcode(prev)))
                start = n + 1
            prev = n
        if start <= prev:
            spans.append((pcode(start), pcode(prev)))
        count = len(holes)
        total_missing += count
        print(f"{label}: {count} missing")
        for a, b in spans:
            print(f"    {a} - {b}")
        print()

print(f"TOTAL missing in scanned P-range: {total_missing}")
print(f"DB size: {len(db)}")

# Also count C/B/U
for prefix in ("C", "B", "U"):
    codes = [k for k in db if k.startswith(prefix)]
    print(f"{prefix}-codes in DB: {len(codes)}")
