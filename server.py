"""
J2534 MCP Server - Bridge between Hermes Agent and J2534 PassThru vehicle diagnostics.

Runs under 32-bit Python to load MVCI32.dll. Implements MCP protocol (JSON-RPC 2.0
over stdio) directly to avoid heavy SDK dependencies.

Usage:
    C:/Python311-32/python.exe server.py
"""
import sys
import json
import ctypes
import traceback
import os
from typing import Any, Optional

# Add the J2534 library to path
sys.path.insert(0, r"C:\Users\Cyrus\Python-J2534-Interface")

# Add this server's own directory to path so sibling modules (pid_decode) import
# regardless of the spawn working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load DTC database
DTC_DB = {}
MFR_DB = {}
_dtc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dtc_data", "dtc_unified.json")
_mfr_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dtc_data", "dtc_manufacturers.json")
try:
    with open(_dtc_path, "r", encoding="utf-8") as f:
        DTC_DB = json.load(f)
except Exception as e:
    sys.stderr.write(f"[DTC load failed] {type(e).__name__}: {e}\n")
    sys.stderr.flush()
try:
    with open(_mfr_path, "r", encoding="utf-8") as f:
        MFR_DB = json.load(f)
except Exception as e:
    sys.stderr.write(f"[MFR load failed] {type(e).__name__}: {e}\n")
    sys.stderr.flush()

# Load SAE J1979 PID decoder (optional)
try:
    import pid_decode
    PID_DECODER_AVAILABLE = bool(pid_decode._MODE_MAP)
except Exception:
    pid_decode = None
    PID_DECODER_AVAILABLE = False

from J2534 import (
    J2534Api, j2534_api,
    pt_open, pt_close,
    pt_connect, pt_disconnect,
    pt_read_message, pt_write_message,
    pt_start_message_filter, pt_stop_message_filter,
    pt_start_ecu_filter,
    pt_read_version, pt_get_last_error,
    pt_ioctl, pt_set_config,
    read_battery_volts, read_programming_voltage,
    clear_transmit_buffer, clear_receive_buffer,
    clear_periodic_messages, clear_message_filters,
    get_list_j2534_devices, set_j2534_device_to_connect,
    ProtocolId, BaudRate, TxFlags, FilterType, IoctlId,
    ConnectFlags, ErrorCode,
    PassThruMsgBuilder, PassThruMsg,
    j2534_config,
)

# ============================================================================
# State
# ============================================================================
device_id: Optional[int] = None
channels: dict[int, dict] = {}  # channel_id -> {protocol, baudrate, ...}
filters: dict[int, int] = {}    # filter_id -> channel_id

# ============================================================================
# MCP Tool Definitions
# ============================================================================

TOOLS = [
    {
        "name": "j2534_list_devices",
        "description": "List all J2534 PassThru devices found in the Windows registry.",
        "inputSchema": {"type": "object", "properties": {}, "required": []}
    },
    {
        "name": "j2534_open_device",
        "description": "Open a J2534 device by index (default 0). Must be called before any vehicle communication.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "device_index": {"type": "integer", "description": "Device index from list_devices (default 0)", "default": 0}
            },
            "required": []
        }
    },
    {
        "name": "j2534_close_device",
        "description": "Close the currently open J2534 device. Disconnects all channels first.",
        "inputSchema": {"type": "object", "properties": {}, "required": []}
    },
    {
        "name": "j2534_read_version",
        "description": "Read firmware version, DLL version, and API version from the open device.",
        "inputSchema": {"type": "object", "properties": {}, "required": []}
    },
    {
        "name": "j2534_read_battery_voltage",
        "description": "Read the vehicle battery voltage through the J2534 device (OBD-II pin 16).",
        "inputSchema": {"type": "object", "properties": {}, "required": []}
    },
    {
        "name": "j2534_connect_channel",
        "description": "Open a communication channel with specified protocol and baud rate.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "protocol": {
                    "type": "string",
                    "description": "Protocol: ISO15765, CAN, ISO9141, ISO14230, J1850_VPW, J1850_PWM",
                    "enum": ["ISO15765", "CAN", "ISO9141", "ISO14230", "J1850_VPW", "J1850_PWM"]
                },
                "baudrate": {
                    "type": "integer",
                    "description": "Baud rate (e.g. 500000 for CAN 500K, 250000 for CAN 250K)",
                    "default": 500000
                },
                "flags": {
                    "type": "integer",
                    "description": "Connection flags (0 for default, 0x800 for CAN_29BIT_ID)",
                    "default": 0
                }
            },
            "required": ["protocol"]
        }
    },
    {
        "name": "j2534_disconnect_channel",
        "description": "Disconnect/close a communication channel.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID to disconnect"}
            },
            "required": ["channel_id"]
        }
    },
    {
        "name": "j2534_setup_ecu_filter",
        "description": "Set up an ISO15765 flow control filter for ECU communication. Required before sending/receiving ISO15765 messages. For standard OBD-II: tx_id=0x7E0, rx_id=0x7E8.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"},
                "tx_id": {"type": "string", "description": "Transmit CAN ID as hex (e.g. '7E0')", "default": "7E0"},
                "rx_id": {"type": "string", "description": "Receive CAN ID as hex (e.g. '7E8')", "default": "7E8"},
                "tx_flags": {"type": "integer", "description": "TX flags (0x40 for ISO15765_FRAME_PAD)", "default": 64}
            },
            "required": ["channel_id"]
        }
    },
    {
        "name": "j2534_send_message",
        "description": "Send a raw message on a channel. For ISO15765: provide CAN ID + UDS payload bytes. Example: tx_id='7E0' with data='22F190' sends a UDS ReadByIdentifier for VIN.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"},
                "tx_id": {"type": "string", "description": "CAN ID as hex string (e.g. '7E0')"},
                "data": {"type": "string", "description": "Payload data as hex string (e.g. '22F190' for UDS ReadByIdentifier VIN)"},
                "protocol": {
                    "type": "string",
                    "description": "Protocol for the message",
                    "enum": ["ISO15765", "CAN", "ISO9141", "ISO14230"],
                    "default": "ISO15765"
                },
                "tx_flags": {"type": "integer", "description": "TX flags (default 0x40 = ISO15765_FRAME_PAD)", "default": 64}
            },
            "required": ["channel_id", "tx_id", "data"]
        }
    },
    {
        "name": "j2534_read_messages",
        "description": "Read messages from a channel's receive buffer. Returns all available messages up to max_msgs.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"},
                "max_msgs": {"type": "integer", "description": "Maximum messages to read (default 1)", "default": 1},
                "timeout_ms": {"type": "integer", "description": "Read timeout in milliseconds (default 2000)", "default": 2000}
            },
            "required": ["channel_id"]
        }
    },
    {
        "name": "j2534_send_and_receive",
        "description": "Send a UDS/ISO15765 message and wait for the response. Convenience wrapper that combines send + read. Perfect for diagnostics like reading DTCs, VIN, live data, etc.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"},
                "tx_id": {"type": "string", "description": "Transmit CAN ID as hex (e.g. '7E0')", "default": "7E0"},
                "data": {"type": "string", "description": "UDS payload as hex (e.g. '22F190' for VIN, '19028F' for read all DTCs, '14FFFFFF' to clear DTCs)"},
                "rx_id": {"type": "string", "description": "Expected response CAN ID as hex (e.g. '7E8')", "default": "7E8"},
                "timeout_ms": {"type": "integer", "description": "Response timeout in ms (default 2000)", "default": 2000},
                "tx_flags": {"type": "integer", "description": "TX flags (default 0x40)", "default": 64}
            },
            "required": ["channel_id", "data"]
        }
    },
    {
        "name": "j2534_uds_scan_ecus",
        "description": "Scan for responding ECUs by sending TesterPresent (0x3E00) to a range of standard CAN IDs. Returns which ECUs respond.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID (must be ISO15765)"},
                "start_id": {"type": "string", "description": "Start CAN ID hex (default '700')", "default": "700"},
                "end_id": {"type": "string", "description": "End CAN ID hex (default '7FF')", "default": "7FF"},
                "timeout_ms": {"type": "integer", "description": "Per-ECU timeout in ms (default 100)", "default": 100}
            },
            "required": ["channel_id"]
        }
    },
    {
        "name": "j2534_clear_buffers",
        "description": "Clear transmit and receive buffers for a channel.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"}
            },
            "required": ["channel_id"]
        }
    },
    {
        "name": "j2534_raw_command",
        "description": "Execute a raw J2534 API call. For advanced users. Available functions: PassThruIoctl, PassThruSetProgrammingVoltage, etc.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "function": {
                    "type": "string",
                    "description": "J2534 function name",
                    "enum": ["ioctl_clear_tx_buffer", "ioctl_clear_rx_buffer", "ioctl_clear_periodic_msgs",
                             "ioctl_clear_msg_filters", "read_programming_voltage"]
                },
                "channel_id": {"type": "integer", "description": "Channel or Device ID"},
            },
            "required": ["function"]
        }
    },
    {
        "name": "j2534_read_lid",
        "description": "Read a Toyota enhanced parameter by Local IDentifier (mode 0x21 ReadDataByLocalIdentifier). Low LIDs (0x00-0x4A) mirror standard OBD-II PIDs (0x0B=MAP, 0x0C=RPM, 0x05=coolant, 0x0F=intake air, 0x10=MAF, 0x11=throttle, 0x42=battery, 0x49/0x4A=APP D/E). Higher LIDs are Toyota-specific (0xC1=engine ID string). Pass lid as 1-4 hex digits.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "channel_id": {"type": "integer", "description": "Channel ID"},
                "lid": {"type": "string", "description": "Local Identifier as hex (e.g. '0B', 'C1')"},
                "tx_id": {"type": "string", "description": "Transmit CAN ID as hex (default '7E0')", "default": "7E0"},
                "rx_id": {"type": "string", "description": "Receive CAN ID as hex (default '7E8')", "default": "7E8"},
                "timeout_ms": {"type": "integer", "description": "Response timeout in ms (default 2000)", "default": 2000}
            },
            "required": ["channel_id", "lid"]
        }
    },
    {
        "name": "j2534_lookup_dtc",
        "description": "Look up one or more DTC codes in the built-in database (12,000+ codes, 18,000+ manufacturer-specific entries: P/B/C/U). Pass code like 'P0301' or a list like 'P0301,C1AC2,U0100'. Optional manufacturer (e.g. 'TOYOTA','FORD','GM','VW') returns that brand's specific meaning when available, else falls back to generic.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "codes": {"type": "string", "description": "Comma-separated DTC codes (e.g. 'P0301,P0171,C1AC2')"},
                "manufacturer": {"type": "string", "description": "Optional manufacturer context (e.g. 'TOYOTA', 'LEXUS', 'FORD', 'GM', 'VW', 'BMW'). Returns brand-specific description when available."}
            },
            "required": ["codes"]
        }
    },
    {
        "name": "j2534_decode_pid",
        "description": "Decode raw OBD-II mode 01 PID response bytes into named, scaled values (RPM, coolant temp, MAP, fuel trims, O2 voltage, etc.) using the SAE J1979 signal definitions. Pass the PID and its raw data bytes (the response payload after the 0x41+0xPID header). Example: pid='0C', data='09 59' -> RPM 598.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "mode": {"type": "string", "description": "OBD mode/service (default '01')", "default": "01"},
                "pid": {"type": "string", "description": "PID hex (e.g. '0C' RPM, '05' coolant, '0B' MAP, '11' throttle, '10' MAF, '42' battery voltage)"},
                "data": {"type": "string", "description": "Raw response data bytes as hex, AFTER the mode+PID bytes (e.g. '09 59' for RPM)"}
            },
            "required": ["pid", "data"]
        }
    },
]

# ============================================================================
# Protocol helpers
# ============================================================================

PROTOCOL_MAP = {
    "ISO15765": ProtocolId.ISO15765,
    "CAN": ProtocolId.CAN,
    "ISO9141": ProtocolId.ISO9141,
    "ISO14230": ProtocolId.ISO14230,
    "J1850_VPW": ProtocolId.J1850_VPW,
    "J1850_PWM": ProtocolId.J1850_PWM,
}

def hex_to_bytes(hex_str: str) -> list[int]:
    """Convert hex string like '7E0' or '22F190' to list of ints."""
    hex_str = hex_str.strip().replace(" ", "")
    if len(hex_str) % 2:
        hex_str = "0" + hex_str
    return [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]

def can_id_to_4bytes(hex_str: str) -> list[int]:
    """Convert CAN ID hex to 4-byte big-endian list."""
    val = int(hex_str, 16)
    return [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]

def format_msg(msg) -> dict:
    """Format a PassThruMsg for JSON output."""
    data_bytes = [msg.Data[i] for i in range(msg.DataSize)]
    data_hex = " ".join(f"{b:02X}" for b in data_bytes)

    result = {
        "protocol_id": msg.ProtocolID,
        "rx_status": f"0x{msg.RxStatus:04X}",
        "timestamp_us": msg.Timestamp,
        "data_size": msg.DataSize,
        "data_hex": data_hex,
        "data_bytes": data_bytes,
    }

    # For ISO15765 / CAN, extract the CAN ID from first 4 bytes
    if msg.DataSize >= 4:
        can_id = (data_bytes[0] << 24) | (data_bytes[1] << 16) | (data_bytes[2] << 8) | data_bytes[3]
        result["can_id"] = f"0x{can_id:03X}"
        if msg.DataSize > 4:
            payload = data_bytes[4:]
            result["payload_hex"] = " ".join(f"{b:02X}" for b in payload)

    return result


def decode_dtc_bytes(byte1: int, byte2: int) -> str:
    """Decode 2 DTC bytes into standard DTC code (e.g. P0301)."""
    # First 2 bits = type
    dtc_types = {0: "P", 1: "C", 2: "B", 3: "U"}
    dtc_type = dtc_types.get((byte1 >> 6) & 0x03, "P")
    # Next 2 bits = second character
    second = (byte1 >> 4) & 0x03
    # Next 4 bits = third character
    third = byte1 & 0x0F
    # byte2 = last two hex digits
    code = f"{dtc_type}{second}{third:01X}{byte2:02X}"
    return code.upper()


def lookup_dtc(code: str, manufacturer: Optional[str] = None) -> dict:
    """Look up a DTC code in the database.

    If manufacturer is given and the code has a brand-specific entry, return that.
    Otherwise fall back to the generic unified description.
    """
    code = code.upper()
    desc = DTC_DB.get(code, "Unknown code")

    result = {"code": code, "description": desc}

    # Manufacturer-aware: include brand-specific meaning when available
    if manufacturer:
        mfr = manufacturer.strip().upper()
        brand_entry = MFR_DB.get(code, {})
        if brand_entry.get(mfr):
            result["manufacturer"] = mfr
            result["manufacturer_description"] = brand_entry[mfr]
            result["description"] = brand_entry[mfr]  # brand wins
        elif brand_entry.get("GENERIC"):
            result["manufacturer"] = mfr
            result["manufacturer_description"] = brand_entry["GENERIC"]

    # Always expose all brand meanings if we have them (useful context)
    brand_entry = MFR_DB.get(code, {})
    if brand_entry:
        result["brands"] = {k: v for k, v in sorted(brand_entry.items())}

    return result

# ============================================================================
# Tool Handlers
# ============================================================================

def handle_tool(name: str, args: dict) -> Any:
    global device_id, channels, filters

    if name == "j2534_list_devices":
        devices = get_list_j2534_devices()
        return {"devices": [{"index": i, "name": d[0], "dll_path": d[1]} for i, d in enumerate(devices)],
                "count": len(devices)}

    elif name == "j2534_open_device":
        idx = args.get("device_index", 0)
        set_j2534_device_to_connect(idx)
        result = pt_open()
        if result is False or result is None:
            err = pt_get_last_error()
            return {"error": f"Failed to open device: {err}"}
        device_id = result
        # Read version info
        ver = pt_read_version(device_id)
        return {
            "device_id": device_id,
            "status": "opened",
            "version": ver if ver else "unknown"
        }

    elif name == "j2534_close_device":
        if device_id is None:
            return {"error": "No device is open"}
        # Disconnect all channels first
        for ch_id in list(channels.keys()):
            pt_disconnect(ch_id)
        channels.clear()
        filters.clear()
        result = pt_close(device_id)
        old_id = device_id
        device_id = None
        return {"status": "closed", "device_id": old_id}

    elif name == "j2534_read_version":
        if device_id is None:
            return {"error": "No device is open. Call j2534_open_device first."}
        ver = pt_read_version(device_id)
        return {"version": ver if ver else "unknown", "device_id": device_id}

    elif name == "j2534_read_battery_voltage":
        if device_id is None:
            return {"error": "No device is open. Call j2534_open_device first."}
        voltage = read_battery_volts(device_id)
        if voltage is False or voltage is None:
            return {"error": "Failed to read battery voltage"}
        # read_battery_volts() returns VOLTS (e.g. 12.5), not millivolts
        return {"voltage_v": round(voltage, 2), "voltage_mv": round(voltage * 1000.0)}

    elif name == "j2534_connect_channel":
        if device_id is None:
            return {"error": "No device is open. Call j2534_open_device first."}
        proto_str = args["protocol"]
        proto = PROTOCOL_MAP.get(proto_str)
        if proto is None:
            return {"error": f"Unknown protocol: {proto_str}"}
        baudrate = args.get("baudrate", 500000)
        flags = args.get("flags", 0)
        ch = pt_connect(device_id, proto, flags, baudrate)
        if ch is False or ch is None:
            err = pt_get_last_error()
            return {"error": f"Failed to connect channel: {err}"}
        channels[ch] = {"protocol": proto_str, "baudrate": baudrate, "flags": flags}
        return {"channel_id": ch, "protocol": proto_str, "baudrate": baudrate, "status": "connected"}

    elif name == "j2534_disconnect_channel":
        ch_id = args["channel_id"]
        if ch_id not in channels:
            return {"error": f"Channel {ch_id} not found in active channels"}
        pt_disconnect(ch_id)
        del channels[ch_id]
        return {"channel_id": ch_id, "status": "disconnected"}

    elif name == "j2534_setup_ecu_filter":
        ch_id = args["channel_id"]
        tx_hex = args.get("tx_id", "7E0")
        rx_hex = args.get("rx_id", "7E8")
        tx_flags = args.get("tx_flags", 0x40)

        tx_int = int(tx_hex, 16)
        rx_int = int(rx_hex, 16)

        # pt_start_ecu_filter(channel_id, protocol_id, mask, pattern(rx), flow_control(tx), tx_flags)
        filter_id = pt_start_ecu_filter(
            ch_id,
            ProtocolId.ISO15765,
            mask_identifier=0xFFFFFFFF,
            pattern_identifier=rx_int,
            flow_control_identifier=tx_int,
            transmit_flags=tx_flags
        )
        if filter_id is False or filter_id is None:
            err = pt_get_last_error()
            return {"error": f"Failed to set up filter: {err}"}
        filters[filter_id] = ch_id
        return {"filter_id": filter_id, "tx_id": tx_hex, "rx_id": rx_hex, "status": "filter_set"}

    elif name == "j2534_send_message":
        ch_id = args["channel_id"]
        tx_hex = args.get("tx_id", "7E0")
        data_hex = args["data"]
        proto_str = args.get("protocol", "ISO15765")
        tx_flags = args.get("tx_flags", 0x40)

        proto = PROTOCOL_MAP.get(proto_str, ProtocolId.ISO15765)
        # Build message: CAN ID (4 bytes) + payload
        can_bytes = can_id_to_4bytes(tx_hex)
        payload_bytes = hex_to_bytes(data_hex)
        msg_data = can_bytes + payload_bytes

        msg = PassThruMsg(proto, tx_flags)
        msg.set_identifier_and_data(int(tx_hex, 16), payload_bytes)

        result = pt_write_message(ch_id, msg, 1, 2000)
        if result is False:
            err = pt_get_last_error()
            return {"error": f"Failed to send message: {err}"}
        return {"status": "sent", "tx_id": tx_hex, "data": data_hex}

    elif name == "j2534_read_messages":
        ch_id = args["channel_id"]
        max_msgs = args.get("max_msgs", 1)
        timeout_ms = args.get("timeout_ms", 2000)

        messages = []
        for i in range(max_msgs):
            msg = PassThruMsg(0, 0)
            result = pt_read_message(ch_id, msg, 1, timeout_ms)
            if result is False:
                break
            messages.append(format_msg(msg))
        return {"messages": messages, "count": len(messages)}

    elif name == "j2534_send_and_receive":
        ch_id = args["channel_id"]
        tx_hex = args.get("tx_id", "7E0")
        data_hex = args["data"]
        rx_hex = args.get("rx_id", "7E8")
        timeout_ms = args.get("timeout_ms", 2000)
        tx_flags = args.get("tx_flags", 0x40)

        proto = ProtocolId.ISO15765
        payload_bytes = hex_to_bytes(data_hex)

        # Send
        msg = PassThruMsg(proto, tx_flags)
        msg.set_identifier_and_data(int(tx_hex, 16), payload_bytes)
        result = pt_write_message(ch_id, msg, 1, 2000)
        if result is False:
            err = pt_get_last_error()
            return {"error": f"Failed to send: {err}"}

        write_info = {"write_result": str(result), "write_type": type(result).__name__}

        # Read responses (may get echo + response)
        responses = []
        for _ in range(10):  # read up to 10 messages
            rx_msg = PassThruMsg(0, 0)
            result = pt_read_message(ch_id, rx_msg, 1, timeout_ms)
            # result is an int error code: 0=success, 0x09=ERR_TIMEOUT, 0x10=ERR_BUFFER_EMPTY
            if result != 0:
                if not responses:  # Only report error if we got nothing
                    responses.append({"read_error_code": f"0x{result:02X}", "error_name": ErrorCode(result).name if result in ErrorCode._value2member_map_ else "unknown"})
                break
            # Skip empty messages
            if rx_msg.DataSize == 0:
                continue
            formatted = format_msg(rx_msg)
            # Mark echo messages (TX_MSG_TYPE flag)
            if rx_msg.RxStatus & 0x01:  # TX echo
                formatted["_echo"] = True
            responses.append(formatted)

        return {
            "request": {"tx_id": tx_hex, "data": data_hex},
            "write_info": write_info,
            "responses": responses,
            "count": len(responses)
        }

    elif name == "j2534_read_lid":
        ch_id = args["channel_id"]
        lid_hex = args["lid"].strip().upper()
        lid_val = int(lid_hex, 16)
        tx_hex = args.get("tx_id", "7E0")
        rx_hex = args.get("rx_id", "7E8")
        timeout_ms = args.get("timeout_ms", 2000)

        # Build mode 0x21 request: [0x21, lid_hi, lid_lo]
        payload = [0x21, (lid_val >> 8) & 0xFF, lid_val & 0xFF]
        proto = ProtocolId.ISO15765

        msg = PassThruMsg(proto, 0x40)
        msg.set_identifier_and_data(int(tx_hex, 16), payload)
        result = pt_write_message(ch_id, msg, 1, 2000)
        if result is False:
            err = pt_get_last_error()
            return {"error": f"Failed to send: {err}"}

        # Read response: 0x61 + lid + data
        data_hex = None
        for _ in range(10):
            rx_msg = PassThruMsg(0, 0)
            r = pt_read_message(ch_id, rx_msg, 1, timeout_ms)
            if r != 0:
                break
            if rx_msg.DataSize < 5:
                continue
            d = [rx_msg.Data[i] for i in range(rx_msg.DataSize)]
            payload_resp = d[4:]  # strip CAN ID header
            if payload_resp and payload_resp[0] == 0x61 and len(payload_resp) >= 3:
                data_hex = " ".join(f"{b:02X}" for b in payload_resp[3:])
                break
            if payload_resp and payload_resp[0] == 0x7F:
                return {"lid": lid_hex, "error": f"negative response 0x7F {payload_resp[1]:02X}", "raw": " ".join(f"{b:02X}" for b in payload_resp)}

        if data_hex is None:
            return {"lid": lid_hex, "error": "no valid 0x61 response", "data_hex": None}

        return {"lid": lid_hex, "data_hex": data_hex, "data_bytes": [int(b, 16) for b in data_hex.split()]}

    elif name == "j2534_uds_scan_ecus":
        ch_id = args["channel_id"]
        start = int(args.get("start_id", "700"), 16)
        end = int(args.get("end_id", "7FF"), 16)
        timeout_ms = args.get("timeout_ms", 100)

        # The default 0x700-0x7FF sweep is 256 ECUs x (write+read timeouts),
        # which can approach the 60s MCP call limit and wedge the subprocess.
        # Cap per-ECU timeout and total ECUs so the worst case stays fast.
        timeout_ms = min(timeout_ms, 150)
        if end - start > 0x7F:  # max 128 ECUs per call
            end = start + 0x7F

        found_ecus = []
        for tx_id in range(start, end + 1):
            rx_id = tx_id + 8  # Standard response offset
            if rx_id > 0x7FF:
                continue
            tx_hex = f"{tx_id:03X}"
            rx_hex = f"{rx_id:03X}"

            # Clear filters and set up new one for this ECU
            clear_message_filters(ch_id)
            fid = pt_start_ecu_filter(
                ch_id, ProtocolId.ISO15765,
                mask_identifier=0xFFFFFFFF,
                pattern_identifier=rx_id,
                flow_control_identifier=tx_id,
                transmit_flags=0x40
            )
            if fid is False:
                continue

            # Clear buffers
            clear_receive_buffer(ch_id)

            # Send TesterPresent
            msg = PassThruMsg(ProtocolId.ISO15765, 0x40)
            msg.set_identifier_and_data(tx_id, [0x3E, 0x00])
            pt_write_message(ch_id, msg, 1, timeout_ms)

            # Try to read response
            rx_msg = PassThruMsg(0, 0)
            result = pt_read_message(ch_id, rx_msg, 1, timeout_ms)
            if result is not False and result is not None:
                if not (rx_msg.RxStatus & 0x01):  # Skip echoes
                    found_ecus.append({
                        "tx_id": tx_hex,
                        "rx_id": rx_hex,
                        "response": format_msg(rx_msg)
                    })

        return {"ecus_found": found_ecus, "count": len(found_ecus),
                "scan_range": f"{args.get('start_id', '700')}-{args.get('end_id', '7FF')}"}

    elif name == "j2534_clear_buffers":
        ch_id = args["channel_id"]
        clear_transmit_buffer(ch_id)
        clear_receive_buffer(ch_id)
        return {"channel_id": ch_id, "status": "buffers_cleared"}

    elif name == "j2534_raw_command":
        func = args["function"]
        ch_id = args.get("channel_id")
        if func == "ioctl_clear_tx_buffer":
            clear_transmit_buffer(ch_id)
            return {"status": "tx_buffer_cleared"}
        elif func == "ioctl_clear_rx_buffer":
            clear_receive_buffer(ch_id)
            return {"status": "rx_buffer_cleared"}
        elif func == "ioctl_clear_periodic_msgs":
            clear_periodic_messages(ch_id)
            return {"status": "periodic_msgs_cleared"}
        elif func == "ioctl_clear_msg_filters":
            clear_message_filters(ch_id)
            return {"status": "msg_filters_cleared"}
        elif func == "read_programming_voltage":
            v = read_programming_voltage(device_id)
            return {"programming_voltage_mv": v}
        return {"error": f"Unknown function: {func}"}

    elif name == "j2534_lookup_dtc":
        codes_str = args.get("codes", "")
        manufacturer = args.get("manufacturer")
        codes = [c.strip().upper() for c in codes_str.split(",") if c.strip()]
        results = []
        for code in codes:
            results.append(lookup_dtc(code, manufacturer))
        return {"results": results, "count": len(results),
                "db_size": len(DTC_DB), "manufacturer_entries": len(MFR_DB),
                "manufacturer": manufacturer.upper() if manufacturer else None}

    elif name == "j2534_decode_pid":
        if not PID_DECODER_AVAILABLE:
            return {"error": "SAE J1979 PID decoder data not loaded"}
        mode = args.get("mode", "01")
        pid = args.get("pid", "").upper()
        data_hex = args.get("data", "")
        try:
            return pid_decode.decode_pid(mode, pid, data_hex)
        except Exception as e:
            return {"error": str(e), "traceback": traceback.format_exc()}

    else:
        return {"error": f"Unknown tool: {name}"}


# ============================================================================
# MCP Protocol (JSON-RPC 2.0 over stdio)
# ============================================================================

def send_response(response: dict):
    """Send a JSON-RPC response to stdout."""
    data = json.dumps(response)
    sys.stdout.write(data + "\n")
    sys.stdout.flush()

def handle_request(request: dict) -> dict:
    """Handle a single JSON-RPC request and return the response."""
    req_id = request.get("id")
    method = request.get("method", "")
    params = request.get("params", {})

    try:
        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {
                        "tools": {"listChanged": False}
                    },
                    "serverInfo": {
                        "name": "j2534-vehicle-diagnostics",
                        "version": "1.0.0"
                    }
                }
            }

        elif method == "notifications/initialized":
            # No response needed for notifications
            return None

        elif method == "tools/list":
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "tools": TOOLS
                }
            }

        elif method == "tools/call":
            tool_name = params.get("name", "")
            tool_args = params.get("arguments", {})

            try:
                result = handle_tool(tool_name, tool_args)
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [
                            {
                                "type": "text",
                                "text": json.dumps(result, indent=2)
                            }
                        ]
                    }
                }
            except Exception as e:
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [
                            {
                                "type": "text",
                                "text": json.dumps({"error": str(e), "traceback": traceback.format_exc()})
                            }
                        ],
                        "isError": True
                    }
                }

        elif method == "ping":
            return {"jsonrpc": "2.0", "id": req_id, "result": {}}

        else:
            # Unknown method
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {
                    "code": -32601,
                    "message": f"Method not found: {method}"
                }
            }

    except Exception as e:
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {
                "code": -32603,
                "message": str(e)
            }
        }


def main():
    """Main MCP server loop - reads JSON-RPC from stdin, writes to stdout."""
    # Ensure debug output goes to stderr, not stdout
    j2534_config.debug = False

    # Log startup to stderr
    sys.stderr.write("J2534 MCP Server starting...\n")
    sys.stderr.flush()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            request = json.loads(line)
        except json.JSONDecodeError as e:
            sys.stderr.write(f"JSON parse error: {e}\n")
            sys.stderr.flush()
            continue

        response = handle_request(request)
        if response is not None:
            send_response(response)

    # Cleanup on exit
    if device_id is not None:
        for ch_id in list(channels.keys()):
            try:
                pt_disconnect(ch_id)
            except:
                pass
        try:
            pt_close(device_id)
        except:
            pass

    sys.stderr.write("J2534 MCP Server shutting down.\n")
    sys.stderr.flush()


if __name__ == "__main__":
    main()
