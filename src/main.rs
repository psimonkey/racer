//! Racer: ESP32-C3 OLED demo showing orientation and LEDs.
//!
//! Uses a 0.42-inch SSD1306 I2C OLED display wired to GPIO5 (SDA) and GPIO6 (SCL).
//! A BMI160 accelerometer/gyroscope is on the same I2C bus, GPIO5 (SDA) and GPIO6 (SCL).
//! A string of 64 WS2812B LEDs (TOTAL_LEDS) is driven from GPIO7.
//! Connects to WiFi network "psimonkey" with password "ilikemonkeys" and serves a web page.

#![no_std]
#![no_main]

use core::convert::Infallible;
use core::net::Ipv4Addr;
use embedded_graphics::{
    mono_font::{ascii::{FONT_6X10, FONT_10X20}, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use esp_hal::delay::Delay;
use esp_hal::gpio::Level;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rmt::{PulseCode, TxChannelConfig, TxChannelCreator};
use smart_leds::RGB8;

use racer::{Accel, Axis, Effect, update_leds, TOTAL_LEDS};

use embassy_net::{
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    Stack,
    StackResources,
    StaticConfigV4,
    tcp::TcpSocket,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use heapless::String;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethod,
    Config,
    ControllerConfig,
    Interface,
    ap::AccessPointConfig,
};
use static_cell::StaticCell;
use httparse::{Request, EMPTY_HEADER};

const DISPLAY_WIDTH: usize = 72;
const DISPLAY_HEIGHT: usize = 40;
const DISPLAY_PAGES: usize = DISPLAY_HEIGHT / 8;
// The SSD1306 controller has 128 columns; the 72 physical pixels start at column 28.
const DISPLAY_COL_OFFSET: u8 = 28;
const SSD1306_I2C_ADDRESS: u8 = 0x3C;
const SSD1306_CMD: u8 = 0x00;
const SSD1306_DATA: u8 = 0x40;

const BMI160_I2C_ADDRESS: u8 = 0x68;
const BMI160_CMD: u8 = 0x7E;
const BMI160_ACCEL_DATA: u8 = 0x12;
const BMI160_CMD_ACCEL_NORMAL: u8 = 0x11;
const BMI160_ACC_RANGE: u8 = 0x41;
const BMI160_ACC_RANGE_2G: u8 = 0x03;

const AP_SSID: &str = "racer";
const AP_IP: &str = "10.20.90.1";

// BMI160 at ±2g: 16384 LSB = 1g.
// Race detection uses a baseline calibrated when entering RaceReady (car stationary on
// incline), then integrates deviation from that baseline to estimate velocity.
const READY_DELAY_TICKS: u16 = 38;     // 38 × 80ms ≈ 3 s before start detection; baseline averaged throughout
const VELOCITY_NOISE_FLOOR: i32 = 200; // ignore deviations smaller than this (sensor noise)
const RACE_START_VELOCITY: i32 = 4000; // |Σ deviation| to declare race started
const MIN_RACE_TICKS: u16 = 6;         // ~480ms minimum race time before end-stop detection
// During Racing, 3 extra reads are interleaved at 20ms intervals within each 80ms tick,
// giving effective 20ms sampling to catch brief impact spikes.
const IMPACT_DELTA: i32 = 5000;        // |delta LSB| over 20ms indicating end-stop hit
const REBOOT_HOLD_TICKS: u8 = 63;      // 63 × 80ms ≈ 5 s button hold to reboot

#[derive(Clone, Copy, PartialEq, Debug)]
enum RaceMode { Display, RaceReady, Racing, RaceOver }

impl RaceMode {
    fn as_str(self) -> &'static str {
        match self {
            RaceMode::Display => "Display",
            RaceMode::RaceReady => "Race Ready",
            RaceMode::Racing => "Racing",
            RaceMode::RaceOver => "Race Over",
        }
    }
}

fn mode_effect(mode: RaceMode) -> Effect {
    match mode {
        RaceMode::Display   => Effect::WaveChase,
        RaceMode::RaceReady => Effect::PulsingGreen,
        RaceMode::Racing    => Effect::RacingChase,
        RaceMode::RaceOver  => Effect::CheckerPulse,
    }
}

static mut CURRENT_MODE: RaceMode = RaceMode::Display;
static mut CURRENT_ORIENTATION: Axis = Axis::X;
static mut CURRENT_ACCEL: Accel = Accel { x: 0, y: 0, z: 0 };
static mut CURRENT_DISPLAY: [u8; DISPLAY_WIDTH * DISPLAY_PAGES] = [0; DISPLAY_WIDTH * DISPLAY_PAGES];
static mut RACE_START_MS: u64 = 0;
static mut RACE_ELAPSED_MS: u64 = 0;
static mut BEST_RACE_MS: u64 = 0;       // best (shortest) race time seen this session; 0 = none
static mut X_INCLINE_BASELINE: i32 = 0; // X reading while stationary on incline
static mut VELOCITY_EST: i32 = 0;       // Σ(accel_x - baseline), proxy for motion onset
static mut READY_TICKS: u16 = 0;        // ticks spent in RaceReady settling/detecting
static mut RACE_TICKS: u16 = 0;         // ticks elapsed since race start

// SAFETY: Single-core cooperative async. Tasks only switch at .await points, so
// these reads/writes are never interleaved within a single task's non-async section.
fn current_mode() -> RaceMode { unsafe { CURRENT_MODE } }
fn enter_mode(mode: RaceMode) {
    match mode {
        RaceMode::RaceReady => unsafe {
            X_INCLINE_BASELINE = 0;
            VELOCITY_EST = 0;
            READY_TICKS = 0;
        },
        RaceMode::Racing => unsafe {
            RACE_START_MS = embassy_time::Instant::now().as_millis();
            RACE_ELAPSED_MS = 0;
            RACE_TICKS = 0;
        },
        RaceMode::RaceOver => {
            // Snapshot the finish time at the exact moment of detection (not end of tick).
            // Only meaningful when coming from Racing; manual mode switches keep the last value.
            if current_mode() == RaceMode::Racing {
                let elapsed = embassy_time::Instant::now().as_millis() - race_start_ms();
                set_race_elapsed_ms(elapsed);
                unsafe {
                    if elapsed > 0 && (BEST_RACE_MS == 0 || elapsed < BEST_RACE_MS) {
                        BEST_RACE_MS = elapsed;
                    }
                }
            }
        },
        _ => {}
    }
    unsafe { CURRENT_MODE = mode; }
}
fn cycle_mode() {
    let next = match current_mode() {
        RaceMode::Display   => RaceMode::RaceReady,
        RaceMode::RaceReady => RaceMode::Racing,
        RaceMode::Racing    => RaceMode::RaceOver,
        RaceMode::RaceOver  => RaceMode::Display,
    };
    enter_mode(next);
}
fn current_orientation() -> Axis { unsafe { CURRENT_ORIENTATION } }
fn set_orientation(a: Axis) { unsafe { CURRENT_ORIENTATION = a; } }
fn current_accel() -> Accel { unsafe { CURRENT_ACCEL } }
fn set_accel(a: Accel) { unsafe { CURRENT_ACCEL = a; } }
fn current_display() -> [u8; DISPLAY_WIDTH * DISPLAY_PAGES] { unsafe { CURRENT_DISPLAY } }
fn set_display(buf: &[u8; DISPLAY_WIDTH * DISPLAY_PAGES]) { unsafe { CURRENT_DISPLAY = *buf; } }
fn race_elapsed_ms() -> u64 { unsafe { RACE_ELAPSED_MS } }
fn set_race_elapsed_ms(ms: u64) { unsafe { RACE_ELAPSED_MS = ms; } }
fn best_race_ms() -> u64 { unsafe { BEST_RACE_MS } }
fn race_start_ms() -> u64 { unsafe { RACE_START_MS } }
fn x_incline_baseline() -> i32 { unsafe { X_INCLINE_BASELINE } }
fn set_x_incline_baseline(v: i32) { unsafe { X_INCLINE_BASELINE = v; } }
fn velocity_est() -> i32 { unsafe { VELOCITY_EST } }
fn add_velocity(d: i32) { unsafe { VELOCITY_EST = VELOCITY_EST.saturating_add(d); } }
fn ready_ticks() -> u16 { unsafe { READY_TICKS } }
fn inc_ready_ticks() { unsafe { READY_TICKS = READY_TICKS.saturating_add(1); } }
fn race_ticks() -> u16 { unsafe { RACE_TICKS } }
fn inc_race_ticks() { unsafe { RACE_TICKS = RACE_TICKS.saturating_add(1); } }

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

fn format_race_time(ms: u64, buf: &mut String<16>) {
    let secs = ms / 1000;
    let millis = ms % 1000;
    if secs < 10 { buf.push('0').ok(); }
    usize_to_str(secs as usize, buf);
    buf.push('.').ok();
    if millis < 100 { buf.push('0').ok(); }
    if millis < 10 { buf.push('0').ok(); }
    usize_to_str(millis as usize, buf);
}

fn find_header_value<'a>(headers: &'a [httparse::Header<'a>], name: &str) -> Option<&'a str> {
    for header in headers {
        if header.name.eq_ignore_ascii_case(name) {
            if let Ok(value) = core::str::from_utf8(header.value) {
                return Some(value);
            }
        }
    }
    None
}

fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h = [
        0x67452301u32,
        0xEFCDAB89u32,
        0x98BADCFEu32,
        0x10325476u32,
        0xC3D2E1F0u32,
    ];

    let len_bits = (data.len() as u64).wrapping_mul(8);
    let mut chunk = [0u8; 128];
    chunk[..data.len()].copy_from_slice(data);
    chunk[data.len()] = 0x80;

    let total_len = data.len() + 1;
    let pad_len = if total_len % 64 > 56 {
        64 + 56 - (total_len % 64)
    } else {
        56 - (total_len % 64)
    };

    let padded_len = total_len + pad_len + 8;
    chunk[total_len + pad_len..total_len + pad_len + 8]
        .copy_from_slice(&len_bits.to_be_bytes());

    let chunks = padded_len / 64;
    for chunk_index in 0..chunks {
        let start = chunk_index * 64;
        let mut w = [0u32; 80];
        for i in 0..16 {
            let offset = start + i * 4;
            w[i] = u32::from_be_bytes([
                chunk[offset],
                chunk[offset + 1],
                chunk[offset + 2],
                chunk[offset + 3],
            ]);
        }
        for i in 16..80 {
            let value = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
            w[i] = value.rotate_left(1);
        }

        let mut a = h[0];
        let mut b = h[1];
        let mut c = h[2];
        let mut d = h[3];
        let mut e = h[4];

        for i in 0..80 {
            let (f, k) = match i {
                0..=19 => ((b & c) | ((!b) & d), 0x5A827999),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDC),
                _ => (b ^ c ^ d, 0xCA62C1D6),
            };
            let temp = a.rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(w[i]);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temp;
        }

        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
    }

    let mut out = [0u8; 20];
    for (i, word) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    out
}

fn base64_encode(input: &[u8], output: &mut [u8]) -> usize {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out_index = 0;
    let mut i = 0;

    while i + 3 <= input.len() {
        let b0 = input[i];
        let b1 = input[i + 1];
        let b2 = input[i + 2];
        let n = ((b0 as usize) << 16) | ((b1 as usize) << 8) | (b2 as usize);
        output[out_index] = TABLE[(n >> 18) & 0x3F];
        output[out_index + 1] = TABLE[(n >> 12) & 0x3F];
        output[out_index + 2] = TABLE[(n >> 6) & 0x3F];
        output[out_index + 3] = TABLE[n & 0x3F];
        out_index += 4;
        i += 3;
    }

    let rem = input.len() - i;
    if rem == 1 {
        let n = (input[i] as usize) << 16;
        output[out_index] = TABLE[(n >> 18) & 0x3F];
        output[out_index + 1] = TABLE[(n >> 12) & 0x3F];
        output[out_index + 2] = b'=';
        output[out_index + 3] = b'=';
        out_index += 4;
    } else if rem == 2 {
        let n = ((input[i] as usize) << 16) | ((input[i + 1] as usize) << 8);
        output[out_index] = TABLE[(n >> 18) & 0x3F];
        output[out_index + 1] = TABLE[(n >> 12) & 0x3F];
        output[out_index + 2] = TABLE[(n >> 6) & 0x3F];
        output[out_index + 3] = b'=';
        out_index += 4;
    }

    out_index
}

fn websocket_accept_key(key: &str, out: &mut String<32>) {
    const WS_GUID: &[u8] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    let mut buffer = [0u8; 64];
    let key_bytes = key.as_bytes();
    buffer[..key_bytes.len()].copy_from_slice(key_bytes);
    buffer[key_bytes.len()..key_bytes.len() + WS_GUID.len()].copy_from_slice(WS_GUID);
    let hash = sha1(&buffer[..key_bytes.len() + WS_GUID.len()]);
    let mut encoded = [0u8; 32];
    let len = base64_encode(&hash, &mut encoded);
    let accept = core::str::from_utf8(&encoded[..len]).unwrap();
    out.clear();
    out.push_str(accept).unwrap();
}

fn encode_ws_text_frame(payload: &[u8], buf: &mut [u8]) -> usize {
    let mut pos = 0;
    buf[pos] = 0x81;
    pos += 1;

    let len = payload.len();
    if len <= 125 {
        buf[pos] = len as u8;
        pos += 1;
    } else if len <= 65535 {
        buf[pos] = 126;
        pos += 1;
        buf[pos..pos + 2].copy_from_slice(&(len as u16).to_be_bytes());
        pos += 2;
    } else {
        buf[pos] = 127;
        pos += 1;
        buf[pos..pos + 8].copy_from_slice(&(len as u64).to_be_bytes());
        pos += 8;
    }

    buf[pos..pos + len].copy_from_slice(payload);
    pos + len
}

const HTML_PAGE: &str = r#"<!DOCTYPE html>
<html>
<head>
    <title>Racer</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; }
        .status { margin: 20px 0; padding: 15px; border-radius: 5px; }
        .lcd { background: #1a1a2e; border-left: 4px solid #00d4ff; }
        .lcd-header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }
        .lcd-header h3 { color: #ccc; margin: 0; }
        .mode-tag { color: #00d4ff; font-weight: bold; font-size: 18px; }
        .lcd canvas { image-rendering: pixelated; width: 288px; height: 160px; border: 1px solid #333; }
        .chart-box { background: #1a1a2e; border-left: 4px solid #00d4ff; }
        .chart-box h3 { color: #ccc; margin: 0 0 8px 0; }
        .chart-box canvas { display: block; max-width: 100%; }
        button { color: white; border: none; padding: 15px 30px; font-size: 18px; border-radius: 5px; cursor: pointer; width: 100%; margin: 10px 0; }
        .btn-display { background: #2196F3; }
        .btn-display:hover { background: #1976D2; }
        .btn-ready { background: #4CAF50; }
        .btn-ready:hover { background: #388E3C; }
        .btn-racing { background: #FF9800; }
        .btn-racing:hover { background: #F57C00; }
        .btn-over { background: #9C27B0; }
        .btn-over:hover { background: #7B1FA2; }
        .btn-reboot { background: #f44336; }
        .btn-reboot:hover { background: #c62828; }
        .loading { color: #666; font-style: italic; }
        .best-time { color: #00d4ff; text-align: center; font-size: 22px; font-weight: bold; padding: 6px 0 2px; letter-spacing: 1px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Racer</h1>
        <div class="status lcd">
            <div class="lcd-header">
                <h3>LCD Display</h3>
                <span class="mode-tag" id="mode">--</span>
            </div>
            <canvas id="lcd" width="72" height="40"></canvas>
            <div class="best-time" id="best-time">Best: --</div>
        </div>
        <div class="status chart-box">
            <h3>X Acceleration</h3>
            <canvas id="accel-chart" width="520" height="110"></canvas>
        </div>
        <button class="btn-display" onclick="setMode('mode_display')">Display Mode</button>
        <button class="btn-ready" onclick="setMode('mode_ready')">Race Ready</button>
        <button class="btn-racing" onclick="setMode('mode_racing')">Racing</button>
        <button class="btn-over" onclick="setMode('mode_over')">Race Over</button>
        <button class="btn-reboot" onclick="reboot()">Reboot Device</button>
        <div class="loading" id="status">Connecting...</div>
    </div>
    <script>
        const modeEl = document.getElementById('mode');
        const statusEl = document.getElementById('status');
        const bestEl = document.getElementById('best-time');

        function formatTime(ms) {
            var s = Math.floor(ms / 1000);
            var m = ms % 1000;
            return (s < 10 ? '0' : '') + s + '.' +
                   (m < 100 ? '0' : '') + (m < 10 ? '0' : '') + m;
        }

        const accelBuf = [];
        const modeBuf = [];
        const CHART_LEN = 150;
        const chartCanvas = document.getElementById('accel-chart');
        const chartCtx = chartCanvas.getContext('2d');

        function drawChart() {
            const w = chartCanvas.width, h = chartCanvas.height;
            const yMin = -20000, yMax = 20000;
            function toY(v) { return h * (1 - (v - yMin) / (yMax - yMin)); }

            chartCtx.fillStyle = '#0d0d1a';
            chartCtx.fillRect(0, 0, w, h);

            // Shade regions where mode was Racing
            chartCtx.fillStyle = 'rgba(255, 152, 0, 0.15)';
            var inRacing = false, regionStart = 0;
            for (var i = 0; i <= modeBuf.length; i++) {
                var racing = i < modeBuf.length && modeBuf[i] === 'Racing';
                if (racing && !inRacing) {
                    regionStart = (CHART_LEN - modeBuf.length + i) / CHART_LEN * w;
                    inRacing = true;
                } else if (!racing && inRacing) {
                    chartCtx.fillRect(regionStart, 0, (CHART_LEN - modeBuf.length + i) / CHART_LEN * w - regionStart, h);
                    inRacing = false;
                }
            }

            // Grid lines at ±2g, ±1g, 0
            [[16384, '#2a2a3e', '+2g'], [8192, '#1e1e30', '+1g'],
             [0, '#444', ' 0g'], [-8192, '#1e1e30', '-1g'], [-16384, '#2a2a3e', '-2g']
            ].forEach(function(row) {
                var v = row[0], color = row[1], label = row[2];
                chartCtx.strokeStyle = color;
                chartCtx.lineWidth = 1;
                chartCtx.beginPath();
                chartCtx.moveTo(0, toY(v));
                chartCtx.lineTo(w, toY(v));
                chartCtx.stroke();
                chartCtx.fillStyle = '#666';
                chartCtx.font = '10px monospace';
                chartCtx.textAlign = 'right';
                chartCtx.fillText(label, w - 4, toY(v) - 2);
            });

            // Race-start threshold marker
            chartCtx.strokeStyle = '#2a5c2a';
            chartCtx.lineWidth = 1;
            chartCtx.setLineDash([4, 4]);
            chartCtx.beginPath();
            chartCtx.moveTo(0, toY(1500));
            chartCtx.lineTo(w, toY(1500));
            chartCtx.stroke();
            chartCtx.setLineDash([]);
            chartCtx.fillStyle = '#2a8a2a';
            chartCtx.textAlign = 'left';
            chartCtx.fillText('start', 4, toY(1500) - 2);

            if (accelBuf.length < 2) return;
            chartCtx.strokeStyle = '#00d4ff';
            chartCtx.lineWidth = 1.5;
            chartCtx.beginPath();
            for (var i = 0; i < accelBuf.length; i++) {
                var x = (CHART_LEN - accelBuf.length + i) / CHART_LEN * w;
                var y = toY(accelBuf[i]);
                if (i === 0) chartCtx.moveTo(x, y); else chartCtx.lineTo(x, y);
            }
            chartCtx.stroke();
        }

        function updateData(data) {
            if (data.mode !== undefined) modeEl.textContent = data.mode;
            if (data.best && data.best > 0) bestEl.textContent = 'Best: ' + formatTime(data.best);
            accelBuf.push(data.accel.x);
            if (accelBuf.length > CHART_LEN) accelBuf.shift();
            modeBuf.push(data.mode);
            if (modeBuf.length > CHART_LEN) modeBuf.shift();
            drawChart();
            statusEl.textContent = 'Last updated: ' + new Date().toLocaleTimeString();
        }

        const ws = new WebSocket('ws://' + window.location.host + '/ws');
        ws.onopen = () => { statusEl.textContent = 'Connected via WebSocket'; };
        ws.onmessage = (event) => {
            try { const data = JSON.parse(event.data); updateData(data); if (data.display) drawLcd(data.display); }
            catch (e) { statusEl.textContent = 'Invalid WebSocket data'; }
        };
        ws.onclose = () => { statusEl.textContent = 'WebSocket disconnected'; };
        ws.onerror = () => { statusEl.textContent = 'WebSocket error'; };

        const lcdCanvas = document.getElementById('lcd');
        const lcdCtx = lcdCanvas.getContext('2d');
        function drawLcd(hex) {
            const img = lcdCtx.createImageData(72, 40);
            for (let p = 0; p < 5; p++) {
                for (let x = 0; x < 72; x++) {
                    const byte = parseInt(hex.substr((p * 72 + x) * 2, 2), 16);
                    for (let b = 0; b < 8; b++) {
                        const on = (byte >> b) & 1;
                        const i = ((p * 8 + b) * 72 + x) * 4;
                        img.data[i] = on ? 100 : 0; img.data[i+1] = on ? 200 : 0;
                        img.data[i+2] = 255; img.data[i+3] = 255;
                    }
                }
            }
            lcdCtx.putImageData(img, 0, 0);
        }

        function setMode(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                statusEl.textContent = 'Sent: ' + cmd;
            } else { statusEl.textContent = 'WebSocket not connected'; }
        }
        function reboot() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('reboot');
                statusEl.textContent = 'Rebooting...';
            } else { statusEl.textContent = 'WebSocket not connected'; }
        }
    </script>
</body>
</html>"#;

fn build_status_json(mode: RaceMode, orientation: Axis, accel: Accel, elapsed_ms: u64) -> String<1024> {
    let mut json = String::<1024>::new();
    json.push_str(r#"{"mode":""#).unwrap();
    json.push_str(mode.as_str()).unwrap();
    json.push_str(r#"","orientation":""#).unwrap();
    json.push_str(orientation.as_str()).unwrap();
    json.push_str(r#"","accel":{"x":"#).unwrap();
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
    json.push_str(r#"},"display":""#).unwrap();
    let display = current_display();
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in display.iter() {
        json.push(HEX[(byte >> 4) as usize] as char).unwrap();
        json.push(HEX[(byte & 0x0F) as usize] as char).unwrap();
    }
    json.push_str(r#"","elapsed":"#).unwrap();
    let mut elapsed_str = String::<16>::new();
    usize_to_str(elapsed_ms as usize, &mut elapsed_str);
    json.push_str(elapsed_str.as_str()).unwrap();
    json.push_str(r#","best":"#).unwrap();
    let mut best_str = String::<16>::new();
    usize_to_str(best_race_ms() as usize, &mut best_str);
    json.push_str(best_str.as_str()).unwrap();
    json.push('}').unwrap();
    json
}

async fn socket_write_all(socket: &mut TcpSocket<'_>, mut buf: &[u8]) -> Result<(), embassy_net::tcp::Error> {
    while !buf.is_empty() {
        let written = socket.write(buf).await?;
        if written == 0 {
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }
        buf = &buf[written..];
    }
    Ok(())
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

    fn read_accel(i2c: &mut I2c<'_, esp_hal::Blocking>) -> Result<Accel, ()> {
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

// Draw a large "4" filling most of the 72×40 display.
fn draw_number_four(buf: &mut DisplayBuffer) {
    // Right vertical stroke: x=44..49, y=2..37 (6px wide, full height)
    for y in 2i32..38 {
        for x in 44i32..50 {
            buf.set_pixel(x, y, BinaryColor::On);
        }
    }
    // Left vertical stroke: x=22..27, y=2..25 (6px wide, stops at crossbar)
    for y in 2i32..26 {
        for x in 22i32..28 {
            buf.set_pixel(x, y, BinaryColor::On);
        }
    }
    // Horizontal crossbar: x=22..49, y=20..25 (full width, 6px tall)
    for y in 20i32..26 {
        for x in 22i32..50 {
            buf.set_pixel(x, y, BinaryColor::On);
        }
    }
}

// Small plus-sign decorations at the four corners of the display.
fn draw_display_decorations(buf: &mut DisplayBuffer) {
    let corners: [(i32, i32); 4] = [(3, 3), (68, 3), (3, 36), (68, 36)];
    for (cx, cy) in corners {
        buf.set_pixel(cx,     cy,     BinaryColor::On);
        buf.set_pixel(cx - 1, cy,     BinaryColor::On);
        buf.set_pixel(cx + 1, cy,     BinaryColor::On);
        buf.set_pixel(cx,     cy - 1, BinaryColor::On);
        buf.set_pixel(cx,     cy + 1, BinaryColor::On);
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
        let page_commands = [
            0xB0 | page as u8,
            DISPLAY_COL_OFFSET & 0x0F,
            0x10 | (DISPLAY_COL_OFFSET >> 4),
        ];
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
async fn wifi_controller_task(controller: esp_radio::wifi::WifiController<'static>) {
    loop {
        match controller.wait_for_access_point_connected_event_async().await {
            Ok(esp_radio::wifi::AccessPointStationEventInfo::Connected(info)) => {
                println!("AP: station connected: {:?}", info);
            }
            Ok(esp_radio::wifi::AccessPointStationEventInfo::Disconnected(info)) => {
                println!("AP: station disconnected: {:?}", info);
            }
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn dhcp_server_task(stack: Stack<'static>) {
    use core::net::SocketAddrV4;
    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let ip = Ipv4Addr::new(10, 20, 90, 1);
    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound = Udp::new(stack, &buffers);
    let mut bound = unbound
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(ip),
            &ServerOptions::new(ip, Some(&mut gw_buf)),
            &mut bound,
            &mut buf,
        )
        .await
        .inspect_err(|e| println!("DHCP server error: {:?}", e));
        Timer::after(Duration::from_millis(500)).await;
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
            .accept(IpListenEndpoint { addr: None, port: 80 })
            .await;
        println!("Web server: Connection attempt result: {:?}", r.is_ok());

        if let Err(e) = r {
            println!("Web server: Connection error: {:?}", e);
            continue;
        }

        println!("Web server: Client connected, processing request...");

        let mut request_data = heapless::Vec::<u8, 1024>::new();
        let mut request_complete = false;
        loop {
            let mut temp_buf = [0u8; 256];
            match socket.read(&mut temp_buf).await {
                Ok(0) => { println!("read EOF"); break; }
                Ok(len) => {
                    if request_data.extend_from_slice(&temp_buf[..len]).is_err() {
                        println!("Request too large");
                        break;
                    }
                    if request_data.len() >= 4 &&
                       request_data[request_data.len()-4..] == [b'\r', b'\n', b'\r', b'\n'] {
                        request_complete = true;
                        break;
                    }
                }
                Err(e) => { println!("read error: {:?}", e); break; }
            }
        }

        let mut headers = [EMPTY_HEADER; 16];
        let mut request = Request::new(&mut headers);
        let parse_result = if request_complete {
            request.parse(&request_data)
        } else {
            Err(httparse::Error::TooManyHeaders)
        };

        if request_complete && matches!(parse_result, Ok(httparse::Status::Complete(_))) {
            if let Some(path) = request.path {
                let is_websocket = path == "/ws"
                    && request.method == Some("GET")
                    && find_header_value(request.headers, "Upgrade").is_some()
                    && find_header_value(request.headers, "Connection").is_some()
                    && find_header_value(request.headers, "Sec-WebSocket-Key").is_some();

                if is_websocket {
                    let key = find_header_value(request.headers, "Sec-WebSocket-Key").unwrap();
                    let mut accept = String::<32>::new();
                    websocket_accept_key(key, &mut accept);

                    let mut handshake = String::<4096>::new();
                    handshake.push_str("HTTP/1.1 101 Switching Protocols\r\n").unwrap();
                    handshake.push_str("Upgrade: websocket\r\n").unwrap();
                    handshake.push_str("Connection: Upgrade\r\n").unwrap();
                    handshake.push_str("Sec-WebSocket-Accept: ").unwrap();
                    handshake.push_str(accept.as_str()).unwrap();
                    handshake.push_str("\r\n\r\n").unwrap();

                    if let Err(e) = socket_write_all(&mut socket, handshake.as_bytes()).await {
                        println!("WebSocket handshake failed: {:?}", e);
                        socket.close();
                        continue;
                    }

                    println!("WebSocket connected, streaming data...");
                    let mut frame_buffer = [0u8; 1024];
                    let mut rx_buf = [0u8; 64];

                    loop {
                        let timer = Timer::after(Duration::from_millis(500));
                        let read_fut = socket.read(&mut rx_buf);

                        match embassy_futures::select::select(read_fut, timer).await {
                            embassy_futures::select::Either::First(read_result) => {
                                match read_result {
                                    Ok(0) => {
                                        println!("WebSocket client disconnected");
                                        break;
                                    }
                                    Ok(n) if n >= 2 => {
                                        let opcode = rx_buf[0] & 0x0F;
                                        if opcode == 8 {
                                            println!("WebSocket close frame received");
                                            break;
                                        }
                                        if opcode == 1 {
                                            let masked = (rx_buf[1] & 0x80) != 0;
                                            let payload_len = (rx_buf[1] & 0x7F) as usize;
                                            let payload_start = if masked { 6 } else { 2 };
                                            if n >= payload_start + payload_len && payload_len <= 32 {
                                                let mut decoded = [0u8; 32];
                                                if masked && n >= 6 {
                                                    let mask = [rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]];
                                                    for i in 0..payload_len {
                                                        decoded[i] = rx_buf[payload_start + i] ^ mask[i % 4];
                                                    }
                                                } else {
                                                    decoded[..payload_len].copy_from_slice(&rx_buf[payload_start..payload_start + payload_len]);
                                                }
                                                if &decoded[..payload_len] == b"mode_display" {
                                                    enter_mode(RaceMode::Display);
                                                }
                                                if &decoded[..payload_len] == b"mode_ready" {
                                                    enter_mode(RaceMode::RaceReady);
                                                }
                                                if &decoded[..payload_len] == b"mode_racing" {
                                                    enter_mode(RaceMode::Racing);
                                                }
                                                if &decoded[..payload_len] == b"mode_over" {
                                                    enter_mode(RaceMode::RaceOver);
                                                }
                                                if &decoded[..payload_len] == b"reboot" {
                                                    esp_hal::system::software_reset();
                                                }
                                            }
                                        }
                                    }
                                    Ok(_) => {}
                                    Err(e) => {
                                        println!("WebSocket read error: {:?}", e);
                                        break;
                                    }
                                }
                            }
                            embassy_futures::select::Either::Second(_) => {}
                        }

                        let json = build_status_json(current_mode(), current_orientation(), current_accel(), race_elapsed_ms());
                        let frame_len = encode_ws_text_frame(json.as_bytes(), &mut frame_buffer);
                        if let Err(e) = socket_write_all(&mut socket, &frame_buffer[..frame_len]).await {
                            println!("WebSocket stream write failed: {:?}", e);
                            break;
                        }
                    }

                    socket.close();
                    continue;
                }

                let is_api = path == "/data";
                if !is_api {
                    let header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
                    socket_write_all(&mut socket, header).await.ok();
                    socket_write_all(&mut socket, HTML_PAGE.as_bytes()).await.ok();
                    socket.flush().await.ok();
                    socket.close();
                    continue;
                }
            }
        }

        let response: String<4096> = if request_complete && matches!(parse_result, Ok(httparse::Status::Complete(_))) {
            if let Some(path) = request.path {
                match path {
                    "/data" => {
                        let json = build_status_json(current_mode(), current_orientation(), current_accel(), race_elapsed_ms());
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
                    _ => {
                        let mut response = String::<4096>::new();
                        response.push_str("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n").unwrap();
                        response
                    }
                }
            } else {
                let mut response = String::<4096>::new();
                response.push_str("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n").unwrap();
                response
            }
        } else {
            let mut response = String::<4096>::new();
            response.push_str("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n").unwrap();
            response
        };

        if let Err(e) = socket_write_all(&mut socket, response.as_bytes()).await {
            println!("flush error: {:?}", e);
        } else {
            println!("Flush completed successfully");
        }

        socket.flush().await.ok();
        socket.close();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("PANIC: {}", info);
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

    let gw_ip = Ipv4Addr::new(10, 20, 90, 1);
    let ap_config = Config::AccessPoint(
        AccessPointConfig::default()
            .with_ssid(AP_SSID)
            .with_auth_method(AuthenticationMethod::None),
    );

    println!("Starting WiFi AP '{}'...", AP_SSID);
    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();

    let wifi_interface = interfaces.access_point;

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gw_ip, 24),
        gateway: Some(gw_ip),
        dns_servers: Default::default(),
    });

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        {
            static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
            STACK_RESOURCES.init(StackResources::<3>::new())
        },
        seed,
    );

    spawner.spawn(wifi_controller_task(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(dhcp_server_task(stack).unwrap());

    stack.wait_config_up().await;
    println!("AP active at http://{}/", AP_IP);

    println!("Starting web server on port 80...");
    spawner.spawn(web_server(stack).unwrap());
    println!("Web server started");

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
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    let small_style = MonoTextStyleBuilder::new()
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

    let mut pulses = [PulseCode::default(); TOTAL_LEDS * 24 + 1];

    use esp_hal::gpio::{Input, InputConfig, Pull};
    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO10, button_config);
    let mut last_button_state = false;
    let mut button_cooldown: u8 = 0;
    let mut button_hold_ticks: u8 = 0;

    println!("All initialization complete! Starting main loop...");
    println!("Device ready. Point browser to http://{}/", AP_IP);

    let mut prev_x: i16 = 0;

    loop {
        let accel_data = Bmi160::read_accel(&mut i2c).unwrap();
        let axis = accel_data.dominant_axis();
        set_orientation(axis);

        match current_mode() {
            RaceMode::RaceReady => {
                let rt = ready_ticks();
                if rt < READY_DELAY_TICKS {
                    // Refine baseline average throughout the 3-second delay.
                    // X_INCLINE_BASELINE starts at 0 so the first sample gets full weight.
                    let cur = x_incline_baseline();
                    set_x_incline_baseline(
                        (cur * rt as i32 + accel_data.x as i32) / (rt as i32 + 1),
                    );
                    inc_ready_ticks();
                } else {
                    // Integrate deviation from the incline baseline to estimate motion.
                    let dev = accel_data.x as i32 - x_incline_baseline();
                    add_velocity(if dev.abs() > VELOCITY_NOISE_FLOOR { dev } else { 0 });
                    if velocity_est().abs() > RACE_START_VELOCITY {
                        enter_mode(RaceMode::Racing);
                    }
                }
            }
            RaceMode::Racing => {
                inc_race_ticks();
                let elapsed = embassy_time::Instant::now().as_millis() - race_start_ms();
                set_race_elapsed_ms(elapsed);
                // End-stop impact: a large single-tick spike in either direction.
                let delta_x = (accel_data.x as i32) - (prev_x as i32);
                if race_ticks() > MIN_RACE_TICKS && delta_x.abs() > IMPACT_DELTA {
                    enter_mode(RaceMode::RaceOver);
                }
            }
            _ => {}
        }
        prev_x = accel_data.x;
        set_accel(accel_data);

        display_buffer.clear();
        match current_mode() {
            RaceMode::Display => {
                draw_number_four(&mut display_buffer);
                draw_display_decorations(&mut display_buffer);
            }
            RaceMode::RaceReady => {
                let rt = ready_ticks();
                if rt < READY_DELAY_TICKS {
                    // Countdown 3→2→1 across the 3-second baseline delay.
                    let segment = rt * 3 / READY_DELAY_TICKS; // 0, 1, or 2
                    let digit = match segment { 0 => "3", 1 => "2", _ => "1" };
                    // Single digit (10px wide), centred on 72px display at x=31.
                    Text::new(digit, Point::new(31, 26), text_style)
                        .draw(&mut display_buffer)
                        .unwrap();
                } else {
                    // Detection phase: show "READY" (5 chars × 10px = 50px, centred).
                    Text::new("READY", Point::new(11, 26), text_style)
                        .draw(&mut display_buffer)
                        .unwrap();
                }
            }
            RaceMode::Racing => {
                let mut time_str = String::<16>::new();
                format_race_time(race_elapsed_ms(), &mut time_str);
                let x_pos = (72usize.saturating_sub(time_str.len() * 10) / 2) as i32;
                // y=26: glyph top at y=10, vertically centred in 40px display.
                Text::new(time_str.as_str(), Point::new(x_pos, 26), text_style)
                    .draw(&mut display_buffer)
                    .unwrap();
            }
            RaceMode::RaceOver => {
                // Two-line layout: RACE OVER (10px) + 2px gap + time (20px) = 32px total,
                // centred in 40px → top margin 4px.
                // FONT_6X10 baseline ~7px: y = 4+7 = 11. FONT_10X20 baseline ~16px: y = 16+16 = 32.
                Text::new("RACE OVER", Point::new(9, 11), small_style)
                    .draw(&mut display_buffer)
                    .unwrap();
                let mut time_str = String::<16>::new();
                format_race_time(race_elapsed_ms(), &mut time_str);
                let x_pos = (72usize.saturating_sub(time_str.len() * 10) / 2) as i32;
                Text::new(time_str.as_str(), Point::new(x_pos, 32), text_style)
                    .draw(&mut display_buffer)
                    .unwrap();
            }
        }
        flush_display(&mut i2c, &display_buffer);
        set_display(&display_buffer.buffer);

        let tick = (embassy_time::Instant::now().as_millis() as usize) / 80;
        let mut colors = [RGB8::new(0, 0, 0); TOTAL_LEDS];
        let led_effect = if current_mode() == RaceMode::RaceReady && ready_ticks() >= READY_DELAY_TICKS {
            Effect::SolidGreen
        } else {
            mode_effect(current_mode())
        };
        update_leds(&mut colors, led_effect, tick);
        for c in colors.iter_mut() {
            c.r /= 12;
            c.g /= 12;
            c.b /= 12;
        }

        encode_ws2812(&colors, &mut pulses);
        let transaction = channel.transmit(&pulses).unwrap();
        channel = transaction.wait().unwrap();

        let reading = button.is_low();
        if reading {
            button_hold_ticks = button_hold_ticks.saturating_add(1);
            if button_hold_ticks >= REBOOT_HOLD_TICKS {
                println!("Button held 5s: rebooting");
                esp_hal::system::software_reset();
            }
        } else {
            // Fire cycle on release so a long hold never also cycles the mode.
            if last_button_state && button_hold_ticks > 0 && button_cooldown == 0 {
                println!("Button pressed: cycling mode");
                cycle_mode();
                button_cooldown = 5;
            }
            button_hold_ticks = 0;
        }
        last_button_state = reading;
        if button_cooldown > 0 { button_cooldown -= 1; }

        // During Racing, interleave 3 extra accel reads at 20ms intervals so that the
        // effective sampling period is 20ms — necessary to catch brief end-stop impacts.
        if current_mode() == RaceMode::Racing {
            for _ in 0..3 {
                Timer::after(Duration::from_millis(20)).await;
                let s = Bmi160::read_accel(&mut i2c).unwrap();
                let d = (s.x as i32) - (prev_x as i32);
                prev_x = s.x;
                if race_ticks() > MIN_RACE_TICKS && d.abs() > IMPACT_DELTA {
                    enter_mode(RaceMode::RaceOver);
                    break;
                }
            }
            Timer::after(Duration::from_millis(20)).await;
        } else {
            Timer::after(Duration::from_millis(80)).await;
        }
    }
}
