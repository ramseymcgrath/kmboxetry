# KMBoxetry – Cynthion USB Tooling

[![Build Status](https://github.com/yourusername/kmboxetry/actions/workflows/get_bitstream.yml/badge.svg)](https://github.com/yourusername/kmboxetry/actions/workflows/get_bitstream.yml)
[![Code Coverage](https://codecov.io/gh/yourusername/kmboxetry/branch/main/graph/badge.svg)](https://codecov.io/gh/yourusername/kmboxetry)

> 🚧 **Current status – passthrough‑only gateware**  
> The FPGA bitstream (`cynthion_passthrough.py`) implements a **USB Full‑Speed transparent passthrough**.  
> Commands sent by the Rust CLI (`packetry_injector`) have **no effect** on USB traffic with this bitstream.  
> Future work will re‑enable HID injection inside the gateware.

KMBoxetry explores USB manipulation on the **[Cynthion FPGA](https://greatscottgadgets.com/cynthion/)** platform. It ships:

- **Amaranth/LUNA gateware** for the FPGA.
- A **Rust CLI** for device discovery and, eventually, HID injection.

---

## Table of Contents

1. [Features](#features)
2. [Requirements](#requirements)
3. [Installation](#installation)
   - [Environment setup](#1-environment-setup)
   - [Build the FPGA gateware](#2-build-the-fpga-gateware)
   - [Flash the FPGA gateware](#3-flash-the-fpga-gateware)
   - [Build the Rust CLI (optional)](#4-build-the-rust-cli-optional)
4. [Usage](#usage)
5. [Architecture](#architecture)
6. [Development](#development)
7. [Troubleshooting](#troubleshooting)
8. [License](#license)
9. [Acknowledgements](#acknowledgements)

---

## Features

- **FPGA passthrough:** USB FS packets flow between Cynthion’s **TARGET (J2)** ↔ **CONTROL (J3)** ports with minimal latency.
- **Rust CLI (`packetry_injector`):**
  - Dual back‑ends – UDP server or serial listener.
  - Automatic Cynthion discovery & listing.
  - Command parser (`buttons,dx,dy`) ready for future injection support.

---

## Requirements

| Category          | Items                                                                                                                                          |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| **Hardware**      | Cynthion FPGA board                                                                                                                             |
| **FPGA toolchain**| [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) (Yosys + nextpnr‑ecp5 + Project Trellis)                                         |
| **Python**        | Python 3 · `amaranth` · `luna` · `pyserial`                                                                                                     |
| **Flashing**      | `dfu-util`                                                                                                                                      |
| **Rust**          | Stable toolchain (`rustup`, `cargo`)                                                                                                            |

---

## Installation

### 1. Environment setup

```bash
# Python dependencies
pip install amaranth amaranth-boards \
           git+https://github.com/greatscottgadgets/luna.git \
           pyserial
```

Ensure `dfu-util` and OSS CAD Suite binaries are on your `$PATH`.

### 2. Build the FPGA gateware

```bash
# From the repository root
python src/backend/cynthion_passthrough.py
```

This produces `build/gateware/top.bit`.

### 3. Flash the FPGA gateware

1. **Enter DFU mode**  
   - Hold **USR** → press **RST** → release **USR**.  
   - The green **STAT** LED turns off.
2. **Flash**
   ```bash
   dfu-util -d 1d50:615b -a 0 -D build/gateware/top.bit
   ```
   > Replace `1d50:615b` if your board uses a different VID:PID.
3. **Reset Cynthion** – press **RST** to load the new bitstream.

### 4. Build the Rust CLI (optional)

```bash
git clone https://github.com/yourusername/kmboxetry.git
cd kmboxetry
cargo build --release
```

The binary is at `target/release/packetry_injector`.

---

## Usage

### Passthrough demo

1. Host PC ↔ **TARGET (J2)**  
2. USB device ↔ **CONTROL (J3)**  

The device on J3 enumerates on the host as if directly attached.

### Rust CLI (experimental)

```
packetry_injector [OPTIONS]
```

| Option | Description                      |
| ------ | -------------------------------- |
| `--udp IP:PORT`      | Run UDP server             |
| `--serial PORT`      | Run serial server          |
| `--baud RATE`        | Serial baud (default 115200) |
| `--speed {low|full|high}` | Intended USB speed (no effect yet) |
| `--list`             | List Cynthion devices & exit |
| `--device-index N`   | Select device index        |
| `--version`          | Show version information   |

> **Heads‑up:** With the passthrough bitstream loaded, CLI commands do **nothing** – they’re here for future integration.

---

## Architecture

```text
          ┌──────────────────────┐        Rust CLI
          │      Host PC         │<────┐  (UDP / Serial)
          └────────┬─────────────┘     │
                   │ USB FS            │
            TARGET │ (ULPI PHY)        │ commands
               J2  ▼                   │
           ┌──────────────┐            │
           │  Cynthion    │────────────┘
           │  (ECP5 FPGA) │
           └──────────────┘
               ▲  CONTROL J3
               │  USB FS
           Target USB device
```

- **Gateware:** `src/backend/cynthion_passthrough.py` (Amaranth + LUNA)
- **CLI:** `src/` Rust crate – discovery, server back‑ends, placeholder injection logic.

---

## Development

```bash
# Build Rust
cargo build

# Unit tests
cargo test

# Build gateware (requires Python env)
python src/backend/cynthion_passthrough.py
```

### Coverage

```bash
cargo install cargo-tarpaulin  # one‑time install
cargo tarpaulin --out Html --output-dir coverage \
                --exclude-files src/usb.rs --packages packetry_injector
open coverage/tarpaulin-report.html
```

---

## Troubleshooting

<details>
<summary>Device on J3 not detected by host</summary>

- Re‑flash the correct bitstream & reset Cynthion.
- Confirm cabling: Host ↔ J2, Device ↔ J3.
- Only Full‑/Low‑Speed devices are supported.
- Check power on J3.
- Verify Cynthion hardware integrity.
</details>

<details>
<summary>CLI can’t find Cynthion</summary>

- In passthrough mode the FPGA does **not** enumerate over USB. Discovery only works in DFU mode or with firmware exposing a VID:PID.
- Add udev rules (see below) and reconnect.
</details>

### udev rules (Linux)

```udev
# /etc/udev/rules.d/50-cynthion.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="615b", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## License

Distributed under the terms of the **MIT License**. See `LICENSE` for full text.

## Acknowledgements

- **Cynthion** by *Great Scott Gadgets*.
- Built with **Amaranth HDL**, **LUNA USB framework**, and a stack of amazing open‑source crates.

