//! Business logic for Racer project.

#![no_std]

use smart_leds::RGB8;

/// Number of LEDs in the strip
pub const NUM_LEDS: usize = 20;

/// Rainbow colors
pub const RAINBOW: [RGB8; 7] = [
    RGB8 { r: 255, g: 0, b: 0 },       // Red
    RGB8 { r: 255, g: 165, b: 0 },     // Orange
    RGB8 { r: 255, g: 255, b: 0 },     // Yellow
    RGB8 { r: 0, g: 255, b: 0 },       // Green
    RGB8 { r: 0, g: 0, b: 255 },       // Blue
    RGB8 { r: 75, g: 0, b: 130 },      // Indigo
    RGB8 { r: 238, g: 130, b: 238 },   // Violet
];

use core::fmt;

#[derive(Clone, Copy)]
pub enum Effect {
    RainbowSections,
    WaveChase,
    AlternatingGlow,
}

impl fmt::Display for Effect {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Effect::RainbowSections => write!(f, "Rainbow Sections"),
            Effect::WaveChase => write!(f, "Wave Chase"),
            Effect::AlternatingGlow => write!(f, "Alternating Glow"),
        }
    }
}

impl Effect {
    pub const fn all() -> [Effect; 3] {
        [Effect::RainbowSections, Effect::WaveChase, Effect::AlternatingGlow]
    }
}

pub fn update_leds(data: &mut [RGB8; NUM_LEDS], effect: Effect, time: usize) {
    for i in 0..NUM_LEDS {
        data[i] = match effect {
            Effect::RainbowSections => {
                // Divide strip into sections, each getting a rainbow color
                let section_size = NUM_LEDS / RAINBOW.len();
                let section = i / section_size;
                let color_idx = (section + time / 10) % RAINBOW.len();
                RAINBOW[color_idx]
            }
            Effect::WaveChase => {
                // Wave effect moving along the strip
                let wave_position = (time / 2 + i) % (RAINBOW.len() * 2);
                let color_idx = wave_position % RAINBOW.len();
                if wave_position < RAINBOW.len() {
                    RAINBOW[color_idx]
                } else {
                    // Fade out on the way back
                    let brightness = 255 - ((wave_position - RAINBOW.len()) * 255 / RAINBOW.len()) as u8;
                    let color = RAINBOW[color_idx];
                    RGB8 {
                        r: (color.r as u16 * brightness as u16 / 255) as u8,
                        g: (color.g as u16 * brightness as u16 / 255) as u8,
                        b: (color.b as u16 * brightness as u16 / 255) as u8,
                    }
                }
            }
            Effect::AlternatingGlow => {
                // Alternating pattern that shifts
                let pattern = (i + time / 5) % 3;
                match pattern {
                    0 => RAINBOW[(time / 8) % RAINBOW.len()],
                    1 => RGB8::default(),
                    _ => {
                        let color = RAINBOW[(time / 8 + 3) % RAINBOW.len()];
                        // Dim the middle LEDs
                        RGB8 {
                            r: color.r / 3,
                            g: color.g / 3,
                            b: color.b / 3,
                        }
                    }
                }
            }
        };
    }
}

/// Convert HSV color to RGB
pub fn hsv_to_rgb(hue: u16, brightness: u8) -> RGB8 {
    let hue = hue % 360;
    let section = hue / 60;
    let fraction = (hue % 60) as u32 * 255 / 60;
    let brightness = brightness as u32;
    let q = brightness - brightness * fraction / 255;
    let t = brightness - brightness * (255 - fraction) / 255;

    let (r, g, b) = match section {
        0 => (brightness, t, 0),
        1 => (q, brightness, 0),
        2 => (0, brightness, t),
        3 => (0, q, brightness),
        4 => (t, 0, brightness),
        _ => (brightness, 0, q),
    };

    RGB8::new(r as u8, g as u8, b as u8)
}

/// Accelerometer data structure
#[derive(Copy, Clone, Debug)]
pub struct Accel {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Accel {
    /// Determine the dominant axis based on absolute values
    pub fn dominant_axis(self) -> char {
        let x = self.x.abs();
        let y = self.y.abs();
        let z = self.z.abs();

        if x >= y && x >= z {
            'X'
        } else if y >= x && y >= z {
            'Y'
        } else {
            'Z'
        }
    }
}