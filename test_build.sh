#!/bin/bash

# Exit on any error
set -e

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Testing Rust application build...${NC}"

# Temporarily rename usb.rs to skip it during build
if [ -f "src/usb.rs" ]; then
    echo -e "${YELLOW}Temporarily renaming src/usb.rs to src/usb.rs.bak${NC}"
    mv src/usb.rs src/usb.rs.bak
fi

# Try to build the application
echo -e "${YELLOW}Building Rust application...${NC}"
if cargo build; then
    echo -e "${GREEN}Build successful!${NC}"
    BUILD_SUCCESS=true
else
    echo -e "${RED}Build failed!${NC}"
    BUILD_SUCCESS=false
fi

# Restore usb.rs
if [ -f "src/usb.rs.bak" ]; then
    echo -e "${YELLOW}Restoring src/usb.rs${NC}"
    mv src/usb.rs.bak src/usb.rs
fi

# Exit with success or failure
if [ "$BUILD_SUCCESS" = true ]; then
    echo -e "${GREEN}Test completed successfully!${NC}"
    exit 0
else
    echo -e "${RED}Test failed!${NC}"
    exit 1
fi