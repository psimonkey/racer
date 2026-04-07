# racer

- Target board: ESP32-C3 OLED development board (similar to ESP32-C3 Supermini)
- Target triple: `riscv32imc-unknown-none-elf`
- Display: 0.42-inch SSD1306 OLED module, 72x40 pixels
- I2C wiring for the display and BMI160 module: GPIO5 = SDA, GPIO6 = SCL
- Set of 20 WS2812B addressable LEDs on GPIO7
- Power wiring: `3V3` = VCC, `GND` = GND
- WiFi: Connects to "psimonkey" network with password "ilikemonkeys", DHCP for IP
- Web server: Runs on port 80, displays current LED effect and accelerometer orientation
- Flashing: Use `cargo run --release` (uses probe-rs)  
- Serial monitoring: Run `espflash monitor` in a separate terminal after flashing
- Debug output: Serial console prints debug statements throughout boot and WiFi connection process
- Status: Project successfully builds with --release profile. Device boots, connects to WiFi, and serves web interface.
- Function: Show "X", "Y", or "Z" on the display, depending on data from the accelerometer.  Also cycle rainbow colours at half brightness on the LED strip. Web interface shows status.
