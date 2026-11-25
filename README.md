# LED Copper String Zigbee Controller

Zigbee-controlled LED copper string light, simple variant: one PWM output on P0.10 sets the brightness. No H-bridge, no battery.

## Hardware

**Target:** Pro Micro nRF52840 (SuperMini, nice!nano compatible)

| Function | Pin |
|----------|-----|
| LED PWM (1 kHz, active low) | P0.10 |

P0.10 is an NFC pin; it is configured as GPIO through UICR (set by the board definition).

### Other Connections

| Function | Pin |
|----------|-----|
| Pairing Button | P0.06 (active low, internal pull-up) |
| Status LED | P0.15 (onboard, PWM0 ch1: mirrors the LED output) |

## Building

```bash
./build.sh          # Build
./build.sh flash    # Build and flash via J-Link
./build.sh usb      # Build and flash over USB (no J-Link)
./build.sh clean    # Clean rebuild
```

First build downloads the nRF Connect SDK (~4GB).

## Updating Firmware

**Over USB (no J-Link):**

```bash
./build.sh usb
```

Then reset or power-cycle the board with USB connected. MCUboot shows up as a USB serial port ("LEDCopper MCUboot") for 3 s after every reset, and the script catches that window, uploads `build/firmware/zephyr/zephyr.signed.bin` and resets. MCUboot stays in recovery when no valid image is present.

**Over Zigbee:** copy `build/*.zigbee` to Zigbee2MQTT's OTA images and start the update from Z2M.

## Output Files

| File | Purpose |
|------|---------|
| `build/merged.hex` | Flash via J-Link (MCUboot + app) |
| `build/dfu_multi_image.bin` | OTA update binary |
| `build/*.zigbee` | Zigbee OTA file with headers |

## Zigbee

- **Device Type:** Dimmable Light (0x0101)
- **Clusters:** Basic, Identify, Groups, Scenes, On/Off, Level Control
- **Model:** LEDCopperSimple (OTA image type 0x0142)
- **OTA:** Supported via MCUboot
- **Watchdog:** 60 s; resets the device if the Zigbee thread or workqueue stalls

- **Role:** End Device, receiver always on (not sleepy), so commands apply immediately and group commands are received.

### Pairing

1. Hold button for 3 seconds to leave the network and enter pairing mode
2. Status LED blinks while not joined (stops after 3 minutes; press the button to resume); otherwise it mirrors the LED output
3. Enable pairing in Zigbee2MQTT/coordinator

Short press toggles the light.

## Operation

- **ON:** P0.10 PWM at 1 kHz, duty cycle = brightness. Generated fully in hardware, no CPU wake-ups.
- **OFF:** P0.10 high; PWM peripheral stopped
- **Brightness:** CIE 1931 perceptual correction for smooth dimming
- **Transitions:** On/off fades use the OnOffTransitionTime attribute (default 1 s, 0 = instant); level transitions are smoothed between steps
- **Power-on behavior:** StartUpOnOff / StartUpCurrentLevel are honored and persisted

## License

MIT
