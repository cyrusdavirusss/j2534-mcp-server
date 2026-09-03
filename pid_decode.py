"""SAE J1979 OBD-II PID signal decoder.

Loads obdb/SAEJ1979 signalsets/v3/default.json and turns raw PID data bytes into
named, scaled values (RPM, coolant temp, MAP, fuel trims, O2 voltages, etc.).

Decode rules (matching the JSON 'fmt'):
  - 'bix' = bit index within the data payload (MSB-first bit numbering)
  - 'len' = bit length of the raw value
  - raw -> value:  value = raw * mul / div + add   (mul/div default 1, add defaults 0)
  - if 'map' present, the raw value indexes a label map instead
"""
import json
import os

_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dtc_data", "saej1979.json")
_COMMANDS = []
_MODE_MAP = {}   # (mode, pid) -> command dict

try:
    with open(_JSON_PATH, "r", encoding="utf-8") as f:
        _data = json.load(f)
    _COMMANDS = _data.get("commands", [])
    for cmd in _COMMANDS:
        c = cmd.get("cmd", {})
        for mode, pid in c.items():
            _MODE_MAP[(mode.upper(), pid.upper())] = cmd
except Exception:
    _COMMANDS = []
    _MODE_MAP = {}


def hex_to_bytes(hex_str: str) -> list:
    h = hex_str.strip().replace(" ", "").replace("0x", "")
    if len(h) % 2:
        h = "0" + h
    return [int(h[i:i + 2], 16) for i in range(0, len(h), 2)]


def _extract_bits(data: list, bix: int, length: int) -> int:
    """Extract `length` bits starting at bit index `bix` (MSB-first)."""
    val = 0
    for i in range(length):
        bit_pos = bix + i
        byte_idx = bit_pos // 8
        bit_in_byte = 7 - (bit_pos % 8)  # MSB-first
        if byte_idx < len(data):
            bit = (data[byte_idx] >> bit_in_byte) & 1
            val = (val << 1) | bit
    return val


def _scale(raw: int, fmt: dict):
    mul = fmt.get("mul", 1) or 1
    div = fmt.get("div", 1) or 1
    add = fmt.get("add", 0) or 0
    return raw * mul / div + add


def decode_signal(sig: dict, data: list):
    fmt = sig.get("fmt", {})
    bix = fmt.get("bix", 0)
    length = fmt.get("len", 8)
    raw = _extract_bits(data, bix, length)

    if "map" in fmt:
        label_map = fmt["map"]
        key = str(raw)
        entry = label_map.get(key)
        return {
            "id": sig.get("id"),
            "name": sig.get("name"),
            "raw": raw,
            "value": entry.get("value") if entry else raw,
            "label": entry.get("description") if entry else None,
            "unit": fmt.get("unit"),
        }

    value = _scale(raw, fmt)
    result = {
        "id": sig.get("id"),
        "name": sig.get("name"),
        "raw": raw,
        "value": round(value, 3) if isinstance(value, float) else value,
        "unit": fmt.get("unit"),
    }
    if "min" in fmt:
        result["min"] = fmt.get("min")
    if "max" in fmt:
        result["max"] = fmt.get("max")
    return result


def decode_pid(mode: str, pid: str, data_hex: str) -> dict:
    """Decode a PID response. `data_hex` is the raw payload AFTER the mode+PID bytes."""
    data = hex_to_bytes(data_hex)
    key = (mode.upper(), pid.upper())
    cmd = _MODE_MAP.get(key)
    if cmd is None:
        return {"error": f"no decoder for mode {mode} PID {pid}", "data_bytes": data}

    signals = cmd.get("signals", [])
    decoded = []
    for sig in signals:
        try:
            decoded.append(decode_signal(sig, data))
        except Exception as e:
            decoded.append({"id": sig.get("id"), "error": str(e)})

    return {
        "mode": mode.upper(),
        "pid": pid.upper(),
        "data_hex": data_hex,
        "signals": decoded,
    }


def available_pids() -> list:
    return sorted({pid for (mode, pid) in _MODE_MAP.keys()})


if __name__ == "__main__":
    # Quick self-test: RPM (0C) 0x0959 -> 598.25 rpm, coolant (05) 0x6C -> 68 C
    print(decode_pid("01", "0C", "09 59"))
    print(decode_pid("01", "05", "6C"))
    print(decode_pid("01", "0B", "4F"))
    print(decode_pid("01", "11", "B2"))
    print(decode_pid("01", "01", "81 06 80 80"))
