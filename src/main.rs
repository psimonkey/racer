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
use heapless::String;
// use ufmt::uwriteln;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    sta::StationConfig,
};
use static_cell::StaticCell;
use httparse::{Request, EMPTY_HEADER};

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
static mut CURRENT_ACCEL: Accel = Accel { x: 0, y: 0, z: 0 };

// Simple integer to string conversion (handles negative numbers)
fn int_to_str(mut num: i16, buf: &mut String<16>) {
    if num == 0 {
        buf.push('0').unwrap();
        return;
    }
    let negative = num < 0;
    if negative {
        num = -num;
    }
    let mut temp = [0u8; 16];
    let mut i = 0;
    let mut n = num as usize;
    while n > 0 {
        temp[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    if negative {
        buf.push('-').unwrap();
    }
    while i > 0 {
        i -= 1;
        buf.push(temp[i] as char).unwrap();
    }
}

// Simple usize to string conversion (for Content-Length)
fn usize_to_str(mut num: usize, buf: &mut String<16>) {
    if num == 0 {
        buf.push('0').unwrap();
        return;
    }
    let mut temp = [0u8; 16];
    let mut i = 0;
    while num > 0 {
        temp[i] = (num % 10) as u8 + b'0';
        num /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        buf.push(temp[i] as char).unwrap();
    }
}

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
async fn wifi_controller_task(mut controller: esp_radio::wifi::WifiController<'static>) {
    loop {
        println!("WiFi: Attempting to connect...");
        match controller.connect_async().await {
            Ok(info) => {
                println!("WiFi: Connected successfully! Info: {:?}", info);
                
                // Wait for disconnect event
                let disconnect_info = controller.wait_for_disconnect_async().await.ok();
                println!("WiFi: Disconnected: {:?}", disconnect_info);
            }
            Err(e) => {
                println!("WiFi: Failed to connect: {:?}", e);
            }
        }
        
        // Retry connection after 5 seconds
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn web_server(stack: Stack<'static>) {
    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 4096];

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

        // Read the HTTP request
        let mut request_data = heapless::Vec::<u8, 1024>::new();

        // Read until we have a complete HTTP request
        let mut request_complete = false;
        loop {
            let mut temp_buf = [0u8; 256];
            match socket.read(&mut temp_buf).await {
                Ok(0) => {
                    println!("read EOF");
                    break;
                }
                Ok(len) => {
                    if request_data.extend_from_slice(&temp_buf[..len]).is_err() {
                        println!("Request too large");
                        break;
                    }
                    
                    // Check if we have a complete HTTP request (ends with \r\n\r\n)
                    if request_data.len() >= 4 && 
                       request_data[request_data.len()-4..] == [b'\r', b'\n', b'\r', b'\n'] {
                        request_complete = true;
                        break;
                    }
                }
                Err(e) => {
                    println!("read error: {:?}", e);
                    break;
                }
            }
        }

        // Parse the request if we have complete data
        let mut headers = [EMPTY_HEADER; 16];
        let mut request = Request::new(&mut headers);
        let parse_result = if request_complete {
            request.parse(&request_data)
        } else {
            Err(httparse::Error::TooManyHeaders) // Or some other error
        };

        // Process the request
        let response: String<4096> = if request_complete && matches!(parse_result, Ok(httparse::Status::Complete(_))) {
            if let Some(path) = request.path {
                match path {
                    "/data" => {
                        // JSON endpoint for sensor data
                        let (effect_name, orientation, accel) = unsafe {
                            (CURRENT_EFFECT, CURRENT_ORIENTATION, CURRENT_ACCEL)
                        };
                        
                        // Build JSON response with actual data
                        let mut json = String::<256>::new();
                        json.push_str(r#"{"effect":""#).unwrap();
                        json.push_str(effect_name.as_str()).unwrap();
                        json.push_str(r#"","orientation":""#).unwrap();
                        
                        // Convert orientation char to string
                        let mut orientation_str = String::<4>::new();
                        orientation_str.push(orientation).unwrap();
                        json.push_str(orientation_str.as_str()).unwrap();
                        
                        json.push_str(r#"","accel":{"x":"#).unwrap();
                        
                        // Convert numbers to strings (simple implementation)
                        let mut x_str = String::<16>::new();
                        let mut y_str = String::<16>::new();
                        let mut z_str = String::<16>::new();
                        
                        int_to_str(accel.x, &mut x_str);
                        int_to_str(accel.y, &mut y_str);
                        int_to_str(accel.z, &mut z_str);
                        
                        json.push_str(x_str.as_str()).unwrap();
                        json.push_str(r#","y":"#).unwrap();
                        json.push_str(y_str.as_str()).unwrap();
                        json.push_str(r#","z":"#).unwrap();
                        json.push_str(z_str.as_str()).unwrap();
                        json.push_str(r#"}}"#).unwrap();
                        
                        let content_length = json.len();
                        let mut response = String::<4096>::new();
                        response.push_str("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ").unwrap();
                        
                        let mut cl_str = String::<16>::new();
                        usize_to_str(content_length, &mut cl_str);
                        response.push_str(cl_str.as_str()).unwrap();
                        response.push_str("\r\n\r\n").unwrap();
                        response.push_str(json.as_str()).unwrap();
                        response
                    }
                    "/effect" if request.method == Some("POST") => {
                        // Cycle to next effect
                        unsafe {
                            let current_index = Effect::all().iter().position(|&e| e == CURRENT_EFFECT).unwrap_or(0);
                            let next_index = (current_index + 1) % Effect::all().len();
                            CURRENT_EFFECT = Effect::all()[next_index];
                        }
                        
                        let json = r#"{"status":"ok"}"#;
                        let content_length = json.len();
                        let mut response = String::<4096>::new();
                        response.push_str("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ").unwrap();
                        
                        let mut cl_str = String::<16>::new();
                        usize_to_str(content_length, &mut cl_str);
                        response.push_str(cl_str.as_str()).unwrap();
                        response.push_str("\r\n\r\n").unwrap();
                        response.push_str(json).unwrap();
                        response
                    }
                    _ => {
                        // Main HTML page
                        let html = r#"<!DOCTYPE html>
<html>
<head>
    <title>Racer Status</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; }
        .status { margin: 20px 0; padding: 15px; border-radius: 5px; }
        .accel { background: #e8f4fd; border-left: 4px solid #2196F3; }
        .effect { background: #f3e5f5; border-left: 4px solid #9C27B0; }
        .orientation { background: #e8f5e8; border-left: 4px solid #4CAF50; }
        .value { font-size: 24px; font-weight: bold; margin: 10px 0; }
        button { background: #2196F3; color: white; border: none; padding: 15px 30px; font-size: 18px; border-radius: 5px; cursor: pointer; width: 100%; margin: 20px 0; }
        button:hover { background: #1976D2; }
        .loading { color: #666; font-style: italic; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Racer Status</h1>
        
        <div class="status orientation">
            <h3>Orientation</h3>
            <div class="value" id="orientation">--</div>
        </div>
        
        <div class="status accel">
            <h3>Accelerometer</h3>
            <div>X: <span class="value" id="accel-x">--</span></div>
            <div>Y: <span class="value" id="accel-y">--</span></div>
            <div>Z: <span class="value" id="accel-z">--</span></div>
        </div>
        
        <div class="status effect">
            <h3>LED Effect</h3>
            <div class="value" id="effect">--</div>
        </div>
        
        <button onclick="cycleEffect()">Change LED Effect</button>
        
        <div class="loading" id="status">Loading...</div>
    </div>

    <script>
        let lastData = null;
        
        async function fetchData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                lastData = data;
                
                document.getElementById('orientation').textContent = data.orientation;
                document.getElementById('accel-x').textContent = data.accel.x;
                document.getElementById('accel-y').textContent = data.accel.y;
                document.getElementById('accel-z').textContent = data.accel.z;
                document.getElementById('effect').textContent = data.effect;
                document.getElementById('status').textContent = 'Last updated: ' + new Date().toLocaleTimeString();
            } catch (error) {
                document.getElementById('status').textContent = 'Error fetching data: ' + error.message;
            }
        }
        
        async function cycleEffect() {
            try {
                const response = await fetch('/effect', { method: 'POST' });
                const result = await response.json();
                if (result.status === 'ok') {
                    document.getElementById('status').textContent = 'Effect changed!';
                    // Refresh data immediately to show new effect
                    setTimeout(fetchData, 100);
                }
            } catch (error) {
                document.getElementById('status').textContent = 'Error changing effect: ' + error.message;
            }
        }
        
        // Auto-refresh every 500ms
        setInterval(fetchData, 500);
        
        // Initial load
        fetchData();
    </script>
</body>
</html>"#;
                        
                        let content_length = html.len();
                        let mut response = String::<4096>::new();
                        response.push_str("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: ").unwrap();
                        
                        let mut cl_str = String::<16>::new();
                        usize_to_str(content_length, &mut cl_str);
                        response.push_str(cl_str.as_str()).unwrap();
                        response.push_str("\r\n\r\n").unwrap();
                        response.push_str(html).unwrap();
                        response
                    }
                }
            } else {
                // Invalid request - no path
                let mut response = String::<4096>::new();
                response.push_str("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n").unwrap();
                response
            }
        } else {
            // Invalid request - parsing failed
            let mut response = String::<4096>::new();
            response.push_str("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n").unwrap();
            response
        };

        let r = socket.write(response.as_bytes()).await;
        if let Err(e) = r {
            println!("flush error: {:?}", e);
        } else {
            println!("Flush completed successfully");
        }
        
        socket.close();
        Timer::after(Duration::from_millis(100)).await;
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
    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    println!("WiFi controller created successfully");

    let wifi_interface = interfaces.station;
    
    println!("Spawning WiFi controller task...");
    spawner.spawn(wifi_controller_task(controller).unwrap());
    println!("WiFi controller task spawned");

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

    // Wait for DHCP with timeout
    println!("Waiting for DHCP configuration...");
    
    let mut ip_address = None;
    let dhcp_timeout = async {
        let start = embassy_time::Instant::now();
        let mut last_report = embassy_time::Instant::now();
        loop {
            if let Some(config) = stack.config_v4() {
                println!("DHCP successful! IP address: {}", config.address);
                ip_address = Some(config.address);
                break;
            }
            
            // Print status every 5 seconds
            if last_report.elapsed() > Duration::from_secs(5) {
                println!("DHCP waiting... Link up: {}, Elapsed: {}s", 
                    stack.is_link_up(), 
                    start.elapsed().as_secs());
                last_report = embassy_time::Instant::now();
            }
            
            if start.elapsed() > Duration::from_secs(30) {
                println!("DHCP timeout! No IP address received after 30 seconds");
                println!("Final network stack link up: {}", stack.is_link_up());
                println!("Stack resources check: attempting to query stack state");
                break;
            }
            
            Timer::after(Duration::from_millis(500)).await;
        }
    };
    
    dhcp_timeout.await;

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
    if let Some(addr) = ip_address {
        println!("Device ready. Point your browser to http://{}:80/", addr);
    } else {
        println!("Device ready but no IP address configured yet.");
    }

    loop {
        // Update OLED display and check accelerometer
        let accel_data = Bmi160::read_accel(&mut i2c, &mut delay).unwrap();
        let orientation = accel_data.dominant_axis();

        // Update shared state
        unsafe {
            CURRENT_ORIENTATION = orientation;
            CURRENT_ACCEL = accel_data;
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
