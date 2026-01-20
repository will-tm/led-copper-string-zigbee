#!/bin/bash
# Build script for LED Copper String Zigbee Controller
# Target: Pro Micro nRF52840 only
#
# Usage:
#   ./build.sh [options]
#
# Options:
#   clean/pristine  - Clean build
#   flash           - Flash after build (J-Link)

set -e

BOARD="promicro_nrf52840/nrf52840"
PRISTINE=""
DO_FLASH=""

# Parse options
for arg in "$@"; do
    case "$arg" in
        pristine|clean)
            PRISTINE="--pristine"
            ;;
        flash)
            DO_FLASH="1"
            ;;
    esac
done

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# === Setup: Python virtual environment ===
if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate venv
source .venv/bin/activate

# === Setup: Install Python dependencies ===
if ! command -v west &> /dev/null; then
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
fi

# === Setup: Initialize west workspace ===
if [ ! -f ".west/config" ]; then
    echo "Initializing west workspace..."
    mkdir -p .west
    cat > .west/config << 'EOF'
[manifest]
path = .
file = west.yml

[zephyr]
base = deps/zephyr
EOF
fi

# === Setup: Download SDK if needed ===
# Check for deps/zephyr/west.yml as indicator of complete SDK
if [ ! -f "deps/zephyr/west.yml" ]; then
    echo "Downloading nRF Connect SDK (this takes a while, ~4GB)..."
    west update
fi

# === Build ===
echo "========================================"
echo "Building LED Copper String Controller"
echo "Board: ${BOARD}"
echo "========================================"

# Set up Nordic toolchain paths (ninja, cmake, dtc, etc.)
NCS_TOOLCHAIN_BASE=$(ls -d /opt/nordic/ncs/toolchains/*/ 2>/dev/null | head -1)
if [ -n "$NCS_TOOLCHAIN_BASE" ]; then
    # Add Cellar binaries to PATH (ninja, cmake, etc.)
    for pkg in ninja cmake dtc gperf; do
        pkg_bin=$(ls -d "${NCS_TOOLCHAIN_BASE}Cellar/${pkg}"/*/bin 2>/dev/null | head -1)
        if [ -n "$pkg_bin" ]; then
            export PATH="$pkg_bin:$PATH"
        fi
    done
fi

# Set Zephyr SDK path if Nordic toolchain is installed
if [ -z "$ZEPHYR_SDK_INSTALL_DIR" ]; then
    NCS_TOOLCHAIN=$(ls -d /opt/nordic/ncs/toolchains/*/opt/zephyr-sdk 2>/dev/null | head -1)
    if [ -n "$NCS_TOOLCHAIN" ]; then
        export ZEPHYR_SDK_INSTALL_DIR="$NCS_TOOLCHAIN"
        echo "Using SDK: $ZEPHYR_SDK_INSTALL_DIR"
    fi
fi

# Set NRF module directory for MCUboot/OTA integration
export ZEPHYR_NRF_MODULE_DIR="${SCRIPT_DIR}/deps/nrf"

# Build the application using sysbuild (SDK v2.8+)
# MCUboot enabled for OTA support
EXTRA_CMAKE_ARGS="-DZEPHYR_NRF_MODULE_DIR=${SCRIPT_DIR}/deps/nrf"
EXTRA_CMAKE_ARGS="${EXTRA_CMAKE_ARGS} -DSB_CONF_FILE=${SCRIPT_DIR}/firmware/sysbuild_mcuboot.conf"
west build -b "${BOARD}" -d build firmware ${PRISTINE} \
    -- ${EXTRA_CMAKE_ARGS}

echo "========================================"
echo "Build complete!"
echo "========================================"

# === Flash ===
if [ -n "$DO_FLASH" ]; then
    echo ""
    echo "Flashing..."
    west flash --build-dir build

    echo "========================================"
    echo "Flash complete!"
    echo "========================================"
else
    echo ""
    echo "Output files in ${SCRIPT_DIR}/build/:"
    echo "  zephyr/zephyr.signed.hex - Flash via J-Link (includes MCUboot)"
    echo "  zephyr/app_update.bin    - OTA update file for Z2M"
    echo ""
    echo "To flash: ./build.sh flash"
    echo "For OTA:  Copy app_update.bin to Z2M's data/otaImages/"
fi
