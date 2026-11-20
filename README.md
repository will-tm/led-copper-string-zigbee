# LED Copper String Zigbee Controller

Zigbee-controlled LED copper string light using a TB6612 H-bridge driver. The LED strip has a special design where one polarity lights half the LEDs and reverse polarity lights the other half. Both polarities are driven alternately within every 1 kHz PWM period, so both halves appear lit simultaneously.

## Hardware

**Target:** Pro Micro nRF52840 (SuperMini, nice!nano compatible)

### TB6612 Wiring

| TB6612 Pin | nRF52840 | Function |
|------------|----------|----------|
| PWMA | P0.22 | Enable (HIGH while lit) |
| AIN1 | P0.20 | Polarity A pulse (PWM0 ch0) |
| AIN2 | P0.17 | Polarity B pulse (PWM0 ch1, inverted) |
| STANDBY | P0.24 | Power save (LOW when OFF) |
| VM | 5V | Motor supply |
| VCC | 3.3V | Logic supply |
| GND | GND | Ground |

### Other Connections

| Function | Pin |
|----------|-----|
| Pairing Button | P0.06 (active low, internal pull-up) |
| Status LED | P0.15 (onboard) |

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

Then plug the board's USB cable into the computer. When USB power appears (after having been absent), the app reboots into MCUboot. MCUboot shows up as a USB serial port ("LEDCopper MCUboot") for 3 s after every reset, and the script catches that window, uploads `build/firmware/zephyr/zephyr.signed.bin` and resets. Plugging into a charger reboots the light once (its state is restored). If the app is broken, any reset (or power cycle) opens the same window, and MCUboot stays in recovery when no valid image is present.

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
- **Model:** LEDCopperV1
- **OTA:** Supported via MCUboot
- **Watchdog:** 60 s; resets the device if the Zigbee thread or workqueue stalls

### Battery

- Measured every 10 min with the output blanked for ~2 ms, so LED load does not skew it
- Not measured while USB is present (that would read the charge voltage)
- Below 3.3 V the light is switched off and stays off until the battery reads 3.5 V or USB is connected
- Reported to the coordinator after every (re)join and 30 s after USB is unplugged

- **Role:** Sleepy End Device. Polls its parent every 250 ms while lit and every 3 s while off, so commands to an off light take up to ~3 s. Being sleepy, it does not receive group (multicast) commands.

### Pairing

1. Hold button for 3 seconds to leave the network and enter pairing mode
2. Status LED blinks while not joined (stops after 3 minutes to save battery; press the button to resume)
3. Enable pairing in Zigbee2MQTT/coordinator

Short press toggles the light.

## Operation

- **ON:** STANDBY and PWMA high. Each 1 ms period, AIN1 is high for D at the start and AIN2 high for D at the end (D ≤ 0.5 ms), so the two polarities never overlap. Generated fully in hardware, no CPU wake-ups.
- **OFF:** STANDBY, PWMA, AIN1, AIN2 low; PWM peripheral stopped
- **Brightness:** CIE 1931 perceptual correction for smooth dimming
- **Transitions:** On/off fades use the OnOffTransitionTime attribute (default 1 s, 0 = instant); level transitions are smoothed between steps
- **Power-on behavior:** StartUpOnOff / StartUpCurrentLevel are honored and persisted

## License

MIT
