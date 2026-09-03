#!/usr/bin/env python3
"""Apply Toyota-authoritative overrides (toyota-club.net) to the merged DTC store.

Fixes the bug where Wal33D's wrong Toyota entries (e.g. P1605 "Knock Control CPU")
leaked into dtc_manufacturers.json's TOYOTA/LEXUS entries.

For every code in toyota_authoritative.json:
  - unified[code] = toyota description (Toyota wording wins)
  - mfr[code]["TOYOTA"] = toyota description
  - mfr[code]["LEXUS"]  = toyota description (Lexus = Toyota's brand)
"""
import json, os

D = os.path.dirname(os.path.abspath(__file__))
unified = json.load(open(os.path.join(D, "dtc_unified.json")))
mfr = json.load(open(os.path.join(D, "dtc_manufacturers.json")))
toyota = json.load(open(os.path.join(D, "toyota_authoritative.json")))

overrode_unified = 0
overrode_toyota = 0
overrode_lexus = 0

for code, desc in toyota.items():
    code = code.upper()
    if unified.get(code) != desc:
        unified[code] = desc
        overrode_unified += 1
    entry = mfr.setdefault(code, {})
    if entry.get("TOYOTA") != desc:
        entry["TOYOTA"] = desc
        overrode_toyota += 1
    if entry.get("LEXUS") != desc:
        entry["LEXUS"] = desc
        overrode_lexus += 1

json.dump(unified, open(os.path.join(D, "dtc_unified.json"), "w"), indent=2, ensure_ascii=False)
json.dump(mfr, open(os.path.join(D, "dtc_manufacturers.json"), "w"), indent=2, ensure_ascii=False)

print(f"unified overrides: {overrode_unified}")
print(f"TOYOTA map overrides: {overrode_toyota}")
print(f"LEXUS map overrides: {overrode_lexus}")

# sanity
for c in ["P1605", "P00B0", "P004B", "P1100", "P1120"]:
    print(f"\n{c}: unified={unified.get(c)}")
    m = mfr.get(c, {})
    print(f"    TOYOTA={m.get('TOYOTA')}")
    print(f"    LEXUS={m.get('LEXUS')}")
