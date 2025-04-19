# KMBoxetry - USB HID Injection for Cynthion FPGA

KMBoxetry is a tool for injecting USB HID (Human Interface Device) commands using Cynthion FPGA hardware. It allows you to send mouse movements and button presses through a Cynthion device, which can be controlled via UDP or serial communication.

## Features

- **Dual Control Modes**: Operate via UDP server or Serial port
- **Multiple USB Speed Support**: Configure for Low, Full, or High speed USB operation
- **Mouse Control**: Send precise mouse movements and button presses
- **Device Discovery**: Automatically detect and list available Cynthion devices
- **Flexible Command Format**: Simple text-based command format (`buttons,dx,dy`)

## Requirements

- Cynthion FPGA device (VID: 0x1d50, PID: 0x615b)
- Rust toolchain (for building from source)
- For FPGA flashing: openFPGALoader

## Installation

### From Source

1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/kmboxetry.git
   cd kmboxetry
   ```

2. Build the project:
   ```sh
   cargo build --release
   ```

3. The binary will be available at `target/release/packetry_injector`

### Flashing the FPGA

To flash the Cynthion FPGA with the required bitstream:

```bash
cd src/backend
./build_and_flash.sh
```

This script will:
1. Check for dependencies
2. Build the Rust project
3. Flash the FPGA bitstream (if available)
4. Test the connection to the Cynthion device

## Usage

### Basic Command Line Options

```sh
packetry_injector [OPTIONS]
```

### Options

- `--udp <IP:PORT>`: Run in UDP server mode, listening on the specified IP and port
- `--serial <PORT>`: Run in Serial server mode, using the specified port
- `--baud <RATE>`: Set baud rate for serial communication (default: 115200)
- `--speed <SPEED>`: USB speed for injection (low, full, high) (default: full)
- `--list`: List available Cynthion devices and serial ports and exit
- `--device-index <INDEX>`: Index of the Cynthion device to use (default: 0)
- `--dependencies`: Print dependency versions (use with --version)
- `--version`: Print version information

### Command Format

Commands are sent as comma-separated values in the format:
```sh
buttons,dx,dy
```

Where:
- `buttons`: Button state bitmask (0x01 = Left, 0x02 = Right, 0x04 = Middle)
- `dx`: Relative X-axis movement (-127 to 127)
- `dy`: Relative Y-axis movement (-127 to 127)

### Examples

#### UDP Mode

Start the server listening on localhost port 9001:
```sh
packetry_injector --udp 127.0.0.1:9001
```

Send commands using netcat:
```sh
echo "1,0,0" | nc -u 127.0.0.1 9001  # Left click
echo "0,10,0" | nc -u 127.0.0.1 9001  # Move right
echo "0,0,-10" | nc -u 127.0.0.1 9001  # Move up
```

#### Serial Mode

Start the server using a serial port:
```sh
packetry_injector --serial /dev/ttyACM0 --baud 115200
```

Send commands through the serial port using any serial terminal application.

#### Listing Devices

List all available Cynthion devices and serial ports:
```sh
packetry_injector --list
```

## Architecture

The project consists of several key components:

1. **Command Line Interface**: Handles user input and configuration
2. **UDP/Serial Servers**: Receive commands from external sources
3. **HID Injector**: Communicates with the Cynthion device to send HID reports
4. **Device Discovery**: Finds and configures available Cynthion devices

## Development

### Project Structure

- `src/main.rs`: Entry point and server implementations
- `src/inject.rs`: Core HID injection functionality
- `src/version.rs`: Version information
- `src/backend/`: FPGA-specific implementations
- `cynthion_injector.rs`: Cynthion device communication
- `build_and_flash.sh`: Script for building and flashing the FPGA

### Building for Development

```sh
cargo build
```

### Running Tests

```sh
cargo test
```

## Troubleshooting

### Device Not Found

- Ensure the Cynthion device is properly connected
- Check that the device has the correct firmware loaded
- Run with `--list` to verify the device is detected

### Injection Not Working

- Verify the USB speed is supported by your device
- Check that the target system recognizes the injected HID device
- Try different USB speeds (low, full, high)

### Permission Issues

On Linux, you may need to add udev rules to access the USB device without root:

```sh
# /etc/udev/rules.d/50-cynthion.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="615b", MODE="0666"
```

After adding the rule, reload udev rules:
```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## License

This project is licensed under the terms found in the LICENSE file.

## Acknowledgments

- This project uses the Cynthion FPGA platform
- Built with Rust and various open-source libraries
