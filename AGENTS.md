# racer

- Target board: ESP32-C3 OLED development board (similar to ESP32-C3 Supermini)
- Target triple: `riscv32imc-unknown-none-elf`
- Display: 0.42-inch SSD1306 OLED module, 72x40 pixels
- I2C wiring for the display and BMI160 module: GPIO5 = SDA, GPIO6 = SCL
- Set of 20 WS2812B addressable LEDs on GPIO7
- Power wiring: `3V3` = VCC, `GND` = GND
- Function: Show "X", "Y", or "Z" on the display, depending on data from the accelerometer.  Also cycle rainbow colours at half brightness on the LED strip.
