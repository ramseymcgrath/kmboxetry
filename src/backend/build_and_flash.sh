#!/usr/bin/env bash

set -e  # Exit on any error

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration variables
PROJECT_NAME="cynthion-injector"
SOURCE_FILE="cynthion_injector.rs"  # Change this to the name of your source file
FPGA_BITSTREAM_PATH="./fpga/bitstream.bit" # Adjust path to your bitstream
CYNTHION_VID="1d50"
CYNTHION_PID="615b"

# Print banner
echo -e "${BLUE}==========================================${NC}"
echo -e "${BLUE}    Cynthion FPGA Build & Flash Tool     ${NC}"
echo -e "${BLUE}==========================================${NC}"

# Check dependencies
echo -e "\n${YELLOW}Checking dependencies...${NC}"

check_command() {
    if ! command -v $1 &> /dev/null; then
        echo -e "${RED}Error: $1 is required but not installed.${NC}"
        echo "Please install $1 and try again."
        exit 1
    fi
}

check_command cargo
check_command openFPGALoader

# Create Rust project if it doesn't exist
if [ ! -d "$PROJECT_NAME" ]; then
    echo -e "\n${YELLOW}Creating new Rust project...${NC}"
    cargo new --bin "$PROJECT_NAME"
    
    # Add necessary dependencies to Cargo.toml
    cat >> "$PROJECT_NAME/Cargo.toml" << EOF

[dependencies]
anyhow = "1.0"
nusb = "0.1"
bitfield = "0.14"
EOF
fi

# Copy source file to project
echo -e "\n${YELLOW}Setting up source code...${NC}"
if [ -f "$SOURCE_FILE" ]; then
    # Create necessary directory structure
    mkdir -p "$PROJECT_NAME/src/inject"
    
    # Create a simple Speed enum module if not exists
    if [ ! -f "$PROJECT_NAME/src/inject/mod.rs" ]; then
        cat > "$PROJECT_NAME/src/inject/mod.rs" << EOF
//! USB injection module with Speed enum

/// USB speed for device operation
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Speed {
    Low = 0,
    Full = 1,
    High = 2,
    Auto = 3,
}

impl Speed {
    /// Returns a bitmask representing this speed for device capability queries
    pub fn mask(&self) -> u8 {
        1 << (*self as u8)
    }
}
EOF
    fi
    
    # Copy our main file
    cp "$SOURCE_FILE" "$PROJECT_NAME/src/lib.rs"
    
    # Create a simple main.rs to use our library
    cat > "$PROJECT_NAME/src/main.rs" << EOF
//! Simple Cynthion USB injector utility

use anyhow::{Context, Result};
use std::time::Duration;
use std::thread::sleep;

fn main() -> Result<()> {
    println!("Cynthion USB Injector");
    println!("Looking for Cynthion devices...");
    
    // Find all USB devices
    let device_list = nusb::list_devices()?;
    
    // Filter for our VID:PID
    let mut found_devices = Vec::new();
    for device_info in device_list {
        if (device_info.vendor_id(), device_info.product_id()) == $PROJECT_NAME::VID_PID {
            match $PROJECT_NAME::probe(device_info) {
                Ok(device) => {
                    found_devices.push(device);
                },
                Err(e) => {
                    eprintln!("Found Cynthion USB device but couldn't probe it: {}", e);
                }
            }
        }
    }
    
    if found_devices.is_empty() {
        eprintln!("No Cynthion devices found!");
        return Ok(());
    }
    
    println!("Found {} Cynthion device(s)!", found_devices.len());
    
    // Use the first device found
    let device = &found_devices[0];
    println!("Using device: {}", 
             device.metadata().iface_desc.as_ref().unwrap_or(&String::from("Unknown")));
    
    println!("Supported speeds: {:?}", device.supported_speeds());
    
    // Open the device and prepare for injection
    let mut handle = device.open_injector()
        .context("Failed to open device for injection")?;
    
    // Configure for High Speed (or use Auto if available)
    use $PROJECT_NAME::Speed;
    let speeds = device.supported_speeds();
    let injection_speed = if speeds.contains(&Speed::High) {
        Speed::High
    } else {
        Speed::Auto
    };
    
    println!("Preparing injection at speed: {:?}", injection_speed);
    handle.prepare_injection(injection_speed)
        .context("Failed to prepare injection")?;
    
    // Wait a bit to let the device stabilize
    sleep(Duration::from_millis(500));
    
    // Example: Send a simple test packet
    let test_data = [0x00, 0x01, 0x02, 0x03, 0x04]; // Replace with your actual packet format
    println!("Injecting test packet...");
    handle.inject_packet(&test_data)
        .context("Failed to inject test packet")?;
    
    println!("Test complete! Stopping injection...");
    handle.stop_injection()
        .context("Failed to stop injection")?;
    
    Ok(())
}
EOF
else
    echo -e "${RED}Error: Source file '$SOURCE_FILE' not found.${NC}"
    echo "Please make sure your source file exists in the current directory."
    exit 1
fi

# Build the Rust project
echo -e "\n${YELLOW}Building Rust project...${NC}"
cd "$PROJECT_NAME"
cargo build --release

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to build Rust project.${NC}"
    exit 1
fi
echo -e "${GREEN}Rust build successful!${NC}"

# Check if the FPGA bitstream file exists (if needed)
if [ ! -f "$FPGA_BITSTREAM_PATH" ]; then
    echo -e "${YELLOW}Warning: FPGA bitstream not found at $FPGA_BITSTREAM_PATH${NC}"
    echo "If you need to flash the FPGA first, please provide the correct path."
    read -p "Continue without flashing the FPGA? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    # Detect Cynthion device
    echo -e "\n${YELLOW}Looking for Cynthion device...${NC}"
    lsusb_output=$(lsusb | grep -i "$CYNTHION_VID:$CYNTHION_PID")

    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: No Cynthion device found with VID:PID $CYNTHION_VID:$CYNTHION_PID${NC}"
        echo "Make sure your device is connected and in the correct mode."
        exit 1
    fi

    echo -e "${GREEN}Found Cynthion device: ${NC}$lsusb_output"

    # Flash the FPGA bitstream
    echo -e "\n${YELLOW}Flashing FPGA bitstream...${NC}"
    echo "Using bitstream: $FPGA_BITSTREAM_PATH"

    # Use openFPGALoader to flash the device
    openFPGALoader -b cynthion "$FPGA_BITSTREAM_PATH"

    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: Failed to flash FPGA bitstream.${NC}"
        exit 1
    fi

    echo -e "${GREEN}FPGA successfully flashed!${NC}"
    
    # Give the device time to reset after flashing
    echo "Waiting for device to reset..."
    sleep 3
fi

# Run the Rust application to test connection
echo -e "\n${YELLOW}Testing Cynthion injector...${NC}"
cd ..
"$PROJECT_NAME/target/release/$PROJECT_NAME"

echo -e "\n${GREEN}Build and flash process completed!${NC}"
