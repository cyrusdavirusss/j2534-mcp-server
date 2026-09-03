#!/usr/bin/env python3
"""Merge Toyota 1VD-FTV DTC table into dtc_unified.json.

Modes:
  --report   (default) list missing codes and description conflicts, no writes
  --apply    actually merge missing codes and prefer Toyota descriptions
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(HERE, "dtc_unified.json")
TOY_PATH = os.path.join(HERE, "1vdftv_toyota.json")

db = json.load(open(DB_PATH))
toy = json.load(open(TOY_PATH))

missing = {}
conflict = []
for code, tdesc in toy.items():
    if code not in db:
        missing[code] = tdesc
    elif db[code] != tdesc:
        conflict.append((code, db[code], tdesc))

print(f"Toyota table size: {len(toy)}")
print(f"DB size before:    {len(db)}")
print(f"MISSING: {len(missing)}")
for c in sorted(missing):
    print(f"  + {c}: {missing[c]}")
print(f"DESCRIPTION CONFLICTS (DB != Toyota): {len(conflict)}")
for c, d, t in conflict:
    print(f"  ~ {c}\n      DB:    {d}\n      Toyota:{t}")

if "--apply" in sys.argv:
    for code, tdesc in missing.items():
        db[code] = tdesc
    for code, d, t in conflict:
        # Prefer Toyota-specific wording, keep the old as-is only if already richer
        db[code] = t
    json.dump(db, open(DB_PATH, "w"), indent=2, ensure_ascii=False)
    print(f"\nAPPLIED. DB size after: {len(db)}")
else:
    print("\n(run with --apply to write)")
