# LED Copper String Zigbee Controller

Zigbee-controlled LED copper string light using a TB6612 H-bridge driver. The LED strip has a special design where one polarity lights half the LEDs and reverse polarity lights the other half. Rapid polarity alternation makes both halves appear lit simultaneously.

## Hardware

**Target:** Pro Micro nRF52840 (SuperMini, nice!nano compatible)

### TB6612 Wiring

| TB6612 Pin | nRF52840 | Function |
|------------|----------|----------|
| PWMA | P0.22 | Brightness (PWM) |
| AIN1 | P0.20 | Polarity control |
| AIN2 | P0.17 | Polarity control |
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
./build.sh clean    # Clean rebuild
```

First build downloads the nRF Connect SDK (~4GB).

## Output Files

| File | Purpose |
|------|---------|
| `build/firmware/zephyr/zephyr.signed.hex` | Flash via J-Link |
| `build/dfu_multi_image.bin` | OTA update binary |
| `build/*.zigbee` | Zigbee OTA file with headers |

## Zigbee

- **Device Type:** Dimmable Light (0x0101)
- **Clusters:** Basic, Identify, Groups, Scenes, On/Off, Level Control
- **Model:** LEDCopperV1
- **OTA:** Supported via MCUboot

### Pairing

1. Hold button for 3 seconds to reset/enter pairing mode
2. Status LED blinks when not joined
3. Enable pairing in Zigbee2MQTT/coordinator

## Operation

- **ON:** STANDBY high, AIN1/AIN2 alternate at 100Hz, PWM controls brightness
- **OFF:** STANDBY low (power save), PWM off
- **Brightness:** CIE 1931 perceptual correction for smooth dimming
- **Transitions:** Smooth fade between brightness levels

## License

MIT
