# J2534 MCP Server — Vehicle Diagnostics for Hermes Agent

A Model Context Protocol (MCP) server that bridges an AI agent to J2534 PassThru
vehicle diagnostics. Tested on Toyota/Lexus via a Mini VCI (FTDI FT232R) cable,
but works with any J2534-compatible adapter and any OBD-II / UDS / KWP2000 vehicle.

## What it does

- Raw J2534 device control (open/close, version, battery voltage)
- ISO15765 (CAN) channel connect/disconnect with ECU flow-control filters
- Send/receive UDS + OBD-II messages (read DTCs, VIN, live data, clear codes)
- **Toyota enhanced mode 0x21** (`read_lid`) — ReadDataByLocalIdentifier for
  non-OBD-II parameters (engine ID string, manufacturer PIDs)
- **Manufacturer-aware DTC lookup** — 12,000+ codes across 33 brands
- **SAE J1979 PID decoder** (`decode_pid`) — raw PID bytes → named, scaled values

## Requirements

- Windows (uses the 32-bit `python.exe` to load 32-bit MVCI32.dll)
- A J2534 PassThru DLL (e.g. Autokent MVCI MultiDriver for Toyota Mini VCI)
- [python-j2534](https://github.com/) bindings (see `sys.path` in `server.py`)

## Quick start

```
C:/Python311-32/python.exe server.py
```

The server speaks JSON-RPC 2.0 over stdio (MCP protocol). Configure it as an MCP
server in your agent:

```yaml
mcp_servers:
  j2534:
    command: C:/Python311-32/python.exe
    args: [C:/path/to/j2534-mcp-server/server.py]
```

## Tools

| Tool | Purpose |
|------|---------|
| `j2534_list_devices` | List J2534 devices in registry |
| `j2534_open_device` / `close_device` | Open/close the adapter |
| `j2534_connect_channel` | Open ISO15765/CAN/etc channel |
| `j2534_setup_ecu_filter` | Flow-control filter (tx 7E0 / rx 7E8) |
| `j2534_send_and_receive` | UDS/ISO15765 request + response |
| `j2534_read_lid` | Toyota mode 0x21 Local IDentifier read |
| `j2534_lookup_dtc` | DTC lookup (optional `manufacturer` param) |
| `j2534_decode_pid` | Decode OBD-II PID bytes to values |

## DTC database

`dtc_data/dtc_unified.json` — flat code→description map (12,000+ codes).
`dtc_data/dtc_manufacturers.json` — code→{brand→description} for brand-specific
meanings (Ford, GM, VW, BMW, Toyota, Lexus, Mazda, etc.).

Sources:
- [Wal33D/dtc-database](https://github.com/Wal33D/dtc-database) (33 brands, SQLite)
- [toyota-club.net](https://toyota-club.net) engine DTC tables (Toyota-authoritative)
- [obdb/SAEJ1979](https://github.com/obdb/SAEJ1979) (OBD-II PID signal definitions)

## License

MIT
