# Nano 33 BLE Camera

## Hardware

- Arduino Nano 33 BLE
- OV7670 camera
- Custom PCB

I2C on valesti ühendatud.

## Edge Impulse

### Install

1. [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-installation)
2. [Arduino CLI](https://arduino.github.io/arduino-cli/installation/)

### Firmware

1. Press RESET twice to launch into the bootloader.
2. Flash Edge Impulse firmware
3. Press the RESET button once to launch the new firmware.

![Flash firmware](img/Screenshot%20from%202024-02-11%2011-23-45.png)

### Project

Create Edge Impulse project.

### Connect

Run:

```bash
edge-impulse-daemon
```