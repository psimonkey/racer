# racer

Firmware for onboard circuit in a small toy car.  Uses an ESP32-C3 OLED development board (similar to the ESP32-C3 Supermini), a BMI160 accelerometer & gyroscope and a set of 20 WS2812B addressable LEDs.

## Description

This project drives a 0.42-inch SSD1306 I2C OLED display to display "X", "Y" or "Z" depending on the orientation of the car, and cycles colours on the LED strip.

## Hardware

- MCU: ESP32-C3
- Display: SSD1306 0.42-inch OLED, 72x40 pixels
- Accelerometer and Gyroscope: BMI160 module
- Interface: I2C

### Wiring

- `GPIO5` -> `SDA`
- `GPIO6` -> `SCL`
- `3V3` -> `VCC`
- `GND` -> `GND`

## Build

From the project directory:

```powershell
cargo build
```

## Flash / Run

Use the configured runner in `.cargo/config.toml`:

```powershell
cargo run
```

## Notes

The project uses the local `esp-hal` workspace packages and the `ssd1306` embedded graphics display driver.
