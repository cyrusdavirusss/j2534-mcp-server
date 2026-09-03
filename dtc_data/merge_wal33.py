#!/usr/bin/env python3
"""Merge Wal33D multi-brand DTC DB into the MCP server's DTC store.

Produces two files in dtc_data/:
  dtc_unified.json        flat {code: description} — the DEFAULT lookup (generic
                          preferred, Toyota-specific where applicable). Kept as the
                          primary so existing lookups stay correct.
  dtc_manufacturers.json  {code: {MANUFACTURER: description}} — full multi-brand map
                          for manufacturer-aware lookup (lookup_dtc manufacturer=...).

Precedence rules:
  - Generic: Wal33D GENERIC (9415) unioned with existing generic.
  - TOYOTA / LEXUS: existing toyota-club data is AUTHORITATIVE (Wal33D has wrong
    P1605 "Knock Control CPU"); only ADD Wal33D Toyota/Lexus codes we don't already
    have a Toyota description for.
  - All other brands: Wal33D as-is.
"""
import json, os, sqlite3

DTC_DIR = os.path.dirname(os.path.abspath(__file__))
UNIFIED = os.path.join(DTC_DIR, "dtc_unified.json")
MFR = os.path.join(DTC_DIR, "dtc_manufacturers.json")
WAL33_SQLITE = r"C:\Users\Cyrus\dtc_research\dtc-database\data\dtc_codes.db"

unified = json.load(open(UNIFIED))
mfr = {}

conn = sqlite3.connect(WAL33_SQLITE)
cur = conn.cursor()
cur.execute("SELECT code, manufacturer, description FROM dtc_definitions WHERE locale='en'")
rows = cur.fetchall()
conn.close()

# Toyota-authoritative set: codes where I already have a Toyota description in unified
# (these came from toyota-club.net engine tables). Treat these as canonical Toyota.
# We can't easily know which unified entries are Toyota, but we DO know the 1VD/1KD/
# 2KD/1GD/2GR-FKS tables were merged with Toyota wording. For safety, we'll only let
# Wal33D ADD Toyota/Lexus codes, never overwrite an existing unified description.

added_generic = 0
added_mfr = 0
toyota_skip = 0

for code, manuf, desc in rows:
    code = code.upper()
    manuf = manuf.upper()

    if manuf == "GENERIC":
        if code not in unified:
            unified[code] = desc
            added_generic += 1
        # generic also goes into mfr map under GENERIC for fallback
        mfr.setdefault(code, {})["GENERIC"] = desc
        continue

    # manufacturer-specific
    mfr.setdefault(code, {})[manuf] = desc

    if manuf in ("TOYOTA", "LEXUS"):
        # Toyota data is thin/wrong in Wal33D; only add to flat map if code missing
        if code not in unified:
            unified[code] = desc
            added_mfr += 1
        else:
            toyota_skip += 1
    else:
        # other brands: don't touch flat unified (avoid wrong-brand pollution);
        # they live only in the manufacturer map.
        added_mfr += 1

json.dump(unified, open(UNIFIED, "w"), indent=2, ensure_ascii=False)
json.dump(mfr, open(MFR, "w"), indent=2, ensure_ascii=False)

print(f"unified codes: {len(unified)} (added {added_generic} generic + {added_mfr} mfr)")
print(f"manufacturer map: {len(mfr)} codes, {sum(len(v) for v in mfr.values())} brand entries")
print(f"toyota/lexus skipped (already have better): {toyota_skip}")

# sanity: verify key codes
for c in ["P00B0", "P004B", "P1605", "P0171", "P2138", "P0A80"]:
    u = unified.get(c, "MISSING")
    m = mfr.get(c, {})
    print(f"\n{c}: unified={u}")
    for b, d in sorted(m.items()):
        print(f"    {b}: {d}")
