//! Racer: ESP32-C3 OLED demo showing orientation and LEDs.
//!
//! Uses a 0.42-inch SSD1306 I2C OLED display wired to GPIO5 (SDA) and GPIO6 (SCL).
//! A BMI160 accelerometer/gyroscope is on the same I2C bus, GPIO5 (SDA) and GPIO6 (SCL).
//! A string of 20 WS2812B LEDs is driven from GPIO7.
//! Connects to WiFi network "psimonkey" with password "ilikemonkeys" and serves a web page.

#![no_std]
#![no_main]

use core::convert::Infallible;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use esp_hal::delay::Delay;
use esp_hal::gpio::Level;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rmt::{PulseCode, TxChannelConfig, TxChannelCreator};
use smart_leds::RGB8;

use racer::{Accel, Effect, update_leds, NUM_LEDS};

use embassy_net::{
    IpListenEndpoint,
    Runner,
    Stack,
    StackResources,
    tcp::TcpSocket,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    sta::StationConfig,
};
use static_cell::StaticCell;

const DISPLAY_WIDTH: usize = 72;
const DISPLAY_HEIGHT: usize = 40;
const DISPLAY_PAGES: usize = DISPLAY_HEIGHT / 8;
const SSD1306_I2C_ADDRESS: u8 = 0x3C;
const SSD1306_CMD: u8 = 0x00;
const SSD1306_DATA: u8 = 0x40;

const BMI160_I2C_ADDRESS: u8 = 0x68;
const BMI160_CMD: u8 = 0x7E;
const BMI160_ACCEL_DATA: u8 = 0x12;
const BMI160_CMD_ACCEL_NORMAL: u8 = 0x11;
const BMI160_ACC_RANGE: u8 = 0x41;
const BMI160_ACC_RANGE_2G: u8 = 0x03;

const WIFI_NETWORK: &str = "REDACTED";
const WIFI_PASSWORD: &str = "REDACTED";

static mut CURRENT_EFFECT: Effect = Effect::RainbowSections;
static mut CURRENT_ORIENTATION: char = 'X';

struct Bmi160;

impl Bmi160 {
    fn init(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &mut Delay) -> Result<(), ()> {
        i2c.write(BMI160_I2C_ADDRESS, &[BMI160_CMD, BMI160_CMD_ACCEL_NORMAL]).map_err(|_| ())?;
        delay.delay_micros(50);
        i2c.write(BMI160_I2C_ADDRESS, &[BMI160_ACC_RANGE, BMI160_ACC_RANGE_2G]).map_err(|_| ())?;
        delay.delay_micros(50);
        Ok(())
    }

    fn read_accel(i2c: &mut I2c<'_, esp_hal::Blocking>, _delay: &mut Delay) -> Result<Accel, ()> {
        let mut buf = [0u8; 6];
        i2c.write_read(BMI160_I2C_ADDRESS, &[BMI160_ACCEL_DATA], &mut buf).map_err(|_| ())?;
        Ok(Accel {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }
}



fn encode_ws2812(data: &[RGB8], pulses: &mut [PulseCode]) {
    const T0H: u16 = 28;
    const T0L: u16 = 55;
    const T1H: u16 = 56;
    const T1L: u16 = 28;

    fn encode_byte(value: u8, output: &mut [PulseCode], offset: &mut usize) {
        for bit in (0..8).rev() {
            let code = if (value & (1 << bit)) != 0 {
                PulseCode::new(Level::High, T1H, Level::Low, T1L)
            } else {
                PulseCode::new(Level::High, T0H, Level::Low, T0L)
            };
            output[*offset] = code;
            *offset += 1;
        }
    }

    let mut index = 0;

    for led in data.iter() {
        encode_byte(led.g, pulses, &mut index);
        encode_byte(led.r, pulses, &mut index);
        encode_byte(led.b, pulses, &mut index);
    }

    pulses[index] = PulseCode::end_marker();
}

struct DisplayBuffer {
    buffer: [u8; DISPLAY_WIDTH * DISPLAY_PAGES],
}

impl Default for DisplayBuffer {
    fn default() -> Self {
        Self {
            buffer: [0u8; DISPLAY_WIDTH * DISPLAY_PAGES],
        }
    }
}

impl DisplayBuffer {
    fn clear(&mut self) {
        self.buffer.fill(0);
    }

    fn set_pixel(&mut self, x: i32, y: i32, color: BinaryColor) {
        if x < 0 || y < 0 || x >= DISPLAY_WIDTH as i32 || y >= DISPLAY_HEIGHT as i32 {
            return;
        }

        let x = x as usize;
        let y = y as usize;
        let page = y / 8;
        let index = x + page * DISPLAY_WIDTH;
        let mask = 1 << (y % 8);

        if color == BinaryColor::On {
            self.buffer[index] |= mask;
        } else {
            self.buffer[index] &= !mask;
        }
    }
}

impl OriginDimensions for DisplayBuffer {
    fn size(&self) -> Size {
        Size::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32)
    }
}

impl DrawTarget for DisplayBuffer {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            self.set_pixel(point.x, point.y, color);
        }
        Ok(())
    }
}

fn initialize_display(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &mut Delay) {
    let init_commands: &[u8] = &[
        0xAE, 0xD5, 0x80, 0xA8, 0x27, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0xAF,
    ];

    for chunk in init_commands.chunks(16) {
        let mut packet = [0u8; 17];
        packet[0] = SSD1306_CMD;
        packet[1..1 + chunk.len()].copy_from_slice(chunk);
        i2c.write(SSD1306_I2C_ADDRESS, &packet[..1 + chunk.len()]).unwrap();
    }

    delay.delay_micros(100);
}

fn flush_display(i2c: &mut I2c<'_, esp_hal::Blocking>, buffer: &DisplayBuffer) {
    for page in 0..DISPLAY_PAGES {
        let page_commands = [0xB0 | page as u8, 0x00, 0x10];
        let mut cmd_packet = [0u8; 4];
        cmd_packet[0] = SSD1306_CMD;
        cmd_packet[1..].copy_from_slice(&page_commands);
        i2c.write(SSD1306_I2C_ADDRESS, &cmd_packet).unwrap();

        let mut data_packet = [0u8; DISPLAY_WIDTH + 1];
        data_packet[0] = SSD1306_DATA;
        data_packet[1..].copy_from_slice(&buffer.buffer[page * DISPLAY_WIDTH..(page + 1) * DISPLAY_WIDTH]);
        i2c.write(SSD1306_I2C_ADDRESS, &data_packet).unwrap();
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn web_server(stack: Stack<'static>) {
    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 1536];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        println!("Web server: Waiting for connection...");
        let r = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 80,
            })
            .await;
        println!("Web server: Connection attempt result: {:?}", r.is_ok());

        if let Err(e) = r {
            println!("Web server: Connection error: {:?}", e);
            continue;
        }

        println!("Web server: Client connected, processing request...");

        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        loop {
            match socket.read(&mut buffer).await {
                Ok(0) => {
                    println!("read EOF");
                    break;
                }
                Ok(len) => {
                    let to_print =
                        unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                    if to_print.contains("\r\n\r\n") {
                        print!("{}", to_print);
                        println!();
                        break;
                    }

                    pos += len;
                }
                Err(e) => {
                    println!("read error: {:?}", e);
                    break;
                }
            };
        }

        // For now, just send a simple response
        // TODO: Get current effect name and orientation
        let (effect_name, orientation) = unsafe {
            (CURRENT_EFFECT, CURRENT_ORIENTATION)
        };

        // Simple HTML response
        let response = match (effect_name, orientation) {
            (Effect::RainbowSections, 'X') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Rainbow Sections</p><p>Orientation: X</p></body></html>\r\n",
            (Effect::RainbowSections, 'Y') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Rainbow Sections</p><p>Orientation: Y</p></body></html>\r\n",
            (Effect::RainbowSections, 'Z') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Rainbow Sections</p><p>Orientation: Z</p></body></html>\r\n",
            (Effect::WaveChase, 'X') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Wave Chase</p><p>Orientation: X</p></body></html>\r\n",
            (Effect::WaveChase, 'Y') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Wave Chase</p><p>Orientation: Y</p></body></html>\r\n",
            (Effect::WaveChase, 'Z') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Wave Chase</p><p>Orientation: Z</p></body></html>\r\n",
            (Effect::AlternatingGlow, 'X') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Alternating Glow</p><p>Orientation: X</p></body></html>\r\n",
            (Effect::AlternatingGlow, 'Y') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Alternating Glow</p><p>Orientation: Y</p></body></html>\r\n",
            (Effect::AlternatingGlow, 'Z') => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Alternating Glow</p><p>Orientation: Z</p></body></html>\r\n",
            _ => "HTTP/1.0 200 OK\r\n\r\n<html><body><h1>Racer Status</h1><p>Current Effect: Unknown</p><p>Orientation: ?</p></body></html>\r\n",
        };

        let r = socket.write(response.as_bytes()).await;
        if let Err(e) = r {
            println!("write error: {:?}", e);
        }

        let r = socket.flush().await;
        if let Err(e) = r {
            println!("flush error: {:?}", e);
        }
        Timer::after(Duration::from_millis(1000)).await;

        socket.close();
        Timer::after(Duration::from_millis(1000)).await;

        socket.abort();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    println!("Racer ESP32-C3 starting up...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    println!("Initializing ESP-HAL...");
    let peripherals = esp_hal::init(config);
    println!("ESP-HAL initialized successfully");

    println!("Setting up heap allocators...");
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);
    println!("Heap allocators configured");

    println!("Starting RTOS...");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    println!("RTOS started");

    println!("Configuring WiFi for network: {}", WIFI_NETWORK);
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(WIFI_NETWORK)
            .with_password(WIFI_PASSWORD.into()),
    );

    println!("Starting WiFi controller...");
    let (_controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    println!("WiFi controller started successfully");

    let wifi_interface = interfaces.station;

    println!("Setting up network stack with DHCP...");
    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        {
            static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
            STACK_RESOURCES.init(StackResources::<3>::new())
        },
        seed,
    );
    println!("Network stack initialized");

    // Spawn network task
    println!("Spawning network task...");
    spawner.spawn(net_task(runner).unwrap());
    println!("Network task spawned");

    // Wait for DHCP
    println!("Waiting for DHCP configuration...");
    stack.wait_config_up().await;
    let config = stack.config_v4().unwrap();
    println!("DHCP successful! IP address: {}", config.address);

    // Spawn web server task
    println!("Starting web server on port 80...");
    spawner.spawn(web_server(stack).unwrap());
    println!("Web server started");

    // Initialize hardware
    println!("Initializing hardware peripherals...");
    let mut delay = Delay::new();

    println!("Setting up I2C for display and accelerometer...");
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO5)
        .with_scl(peripherals.GPIO6);
    println!("I2C configured");

    println!("Initializing OLED display...");
    initialize_display(&mut i2c, &mut delay);
    let mut display_buffer = DisplayBuffer::default();
    println!("OLED display initialized");

    println!("Initializing BMI160 accelerometer...");
    Bmi160::init(&mut i2c, &mut delay).unwrap();
    println!("BMI160 accelerometer initialized");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    println!("Setting up RMT for LED control...");
    let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80)).unwrap();
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_idle_output(false)
        .with_carrier_modulation(false);

    let mut channel = rmt
        .channel0
        .configure_tx(&tx_config)
        .unwrap()
        .with_pin(peripherals.GPIO7);
    println!("RMT and LED control configured");

    let mut pulses = [PulseCode::default(); 20 * 24 + 1];

    let effects = Effect::all();
    let mut effect_index = 0;
    let mut effect_time = 0usize;
    const EFFECT_DURATION: usize = 250; // 250 * 80ms = 20 seconds
    let mut animations_enabled = false;

    println!("All initialization complete! Starting main loop...");
    println!("Device ready. Point your browser to http://{}:80/", config.address);

    loop {
        // Update OLED display and check accelerometer
        let accel_data = Bmi160::read_accel(&mut i2c, &mut delay).unwrap();
        let orientation = accel_data.dominant_axis();

        // Update shared state
        unsafe {
            CURRENT_ORIENTATION = orientation;
        }

        // Check for positive Z acceleration to enable animations
        if !animations_enabled && accel_data.z > 500 {  // Threshold for positive Z acceleration
            animations_enabled = true;
            effect_time = 0;  // Reset effect timing
            effect_index = 0; // Start with first effect
            println!("Animations enabled! Starting LED effects...");
        }

        let width = DISPLAY_WIDTH as i32;
        let height = DISPLAY_HEIGHT as i32;
        let text_width = 6;
        let text_height = 10;
        let x = (width - text_width) / 2;
        let y = (height - text_height) / 2;

        display_buffer.clear();
        let message = if animations_enabled {
            match orientation {
                'X' => "X*",
                'Y' => "Y*",
                'Z' => "Z*",
                _ => "?*",
            }
        } else {
            match orientation {
                'X' => "X",
                'Y' => "Y",
                'Z' => "Z",
                _ => "?",
            }
        };
        Text::new(message, Point::new(x, y), text_style)
            .draw(&mut display_buffer)
            .unwrap();
        flush_display(&mut i2c, &display_buffer);

        // Handle LED animations
        if animations_enabled {
            let current_effect = effects[effect_index];
            unsafe {
                CURRENT_EFFECT = current_effect;
            }
            let mut colors = [RGB8::new(0, 0, 0); NUM_LEDS];
            update_leds(&mut colors, current_effect, effect_time);

            encode_ws2812(&colors, &mut pulses);
            let transaction = channel.transmit(&pulses).unwrap();
            channel = transaction.wait().unwrap();

            // Cycle through effects
            effect_time = effect_time.wrapping_add(1);
            if effect_time >= EFFECT_DURATION {
                effect_time = 0;
                effect_index = (effect_index + 1) % effects.len();
            }
        } else {
            // LEDs off
            let colors = [RGB8::new(0, 0, 0); NUM_LEDS];
            encode_ws2812(&colors, &mut pulses);
            let transaction = channel.transmit(&pulses).unwrap();
            channel = transaction.wait().unwrap();
        }

        Timer::after(Duration::from_millis(80)).await;
    }
}
