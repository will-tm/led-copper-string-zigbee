#!/usr/bin/env python3
"""
Update the firmware over USB through MCUboot serial recovery (no J-Link).

MCUboot opens a USB serial port for a few seconds after every reset, and the
application reboots into that window when USB power is plugged in. This script
waits for the port, catches the window with an SMP echo (which keeps MCUboot
in recovery), uploads the signed image to the primary slot, then resets.

Usage:
    tools/usb_update.py [image]     # default: build/firmware/zephyr/zephyr.signed.bin
Then plug the USB cable in (or reset the board).
"""

import functools
import subprocess
import sys
import time
from pathlib import Path

from serial.tools import list_ports

PRODUCT = "LEDCopper MCUboot"
DEFAULT_IMAGE = Path(__file__).resolve().parent.parent / "build/firmware/zephyr/zephyr.signed.bin"
# Must match CONFIG_BOOT_MAX_LINE_INPUT_LEN / CONFIG_BOOT_LINE_BUFS in MCUboot
SMP_FRAMING = ["--line-length", "128", "--line-buffers", "8"]
WAIT_FOR_PORT_S = 120

print = functools.partial(print, flush=True)


def smp(port, *args, timeout=2.0, run_timeout=10.0):
    cmd = ["smpmgr", "--port", port, "--timeout", str(timeout), *SMP_FRAMING, *args]
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=run_timeout)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 1, "", "smpmgr timed out")


def find_port():
    for p in list_ports.comports():
        if p.product and PRODUCT in p.product:
            return p.device
    return None


def main():
    image = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_IMAGE
    if not image.is_file():
        sys.exit(f"Image not found: {image}")

    print(f"Image: {image} ({image.stat().st_size} bytes)")
    print("Plug in USB (or reset the board) now...")

    deadline = time.monotonic() + WAIT_FOR_PORT_S
    port = None
    while time.monotonic() < deadline:
        port = find_port()
        if port:
            # Any SMP command inside the window keeps MCUboot in recovery
            if smp(port, "os", "echo", "hold", timeout=0.5, run_timeout=3).returncode == 0:
                break
            port = None
        time.sleep(0.05)
    else:
        sys.exit("Bootloader not found. Check the USB cable carries data.")

    print(f"Bootloader held in recovery on {port}")

    result = smp(port, "image", "upload", str(image), timeout=10, run_timeout=300)
    if result.returncode != 0:
        sys.exit(f"Upload failed:\n{result.stdout}{result.stderr}\n"
                 "MCUboot stays in recovery; rerun to retry.")
    print("Upload complete")

    smp(port, "os", "reset")
    print("Reset sent; the new firmware is booting")


if __name__ == "__main__":
    main()
