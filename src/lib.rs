//! Business logic for Racer project.

#![no_std]

use smart_leds::RGB8;

/// Number of LEDs in each set
pub const NUM_LEDS: usize = 32;
/// Total number of LEDs across both sets
pub const TOTAL_LEDS: usize = NUM_LEDS * 2;

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

#[derive(Clone, Copy, PartialEq)]
pub enum Effect {
    Off,
    Standby,
    BlueRedChase,
    RainbowTravel,
    RainbowSections,
    WaveChase,
    AlternatingGlow,
    PulsingGreen,
    SolidGreen,
    RacingChase,
    Fireworks,
    CheckerPulse,
}

impl fmt::Display for Effect {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl Effect {
    pub const fn all() -> [Effect; 12] {
        [Effect::Off, Effect::Standby, Effect::BlueRedChase, Effect::RainbowTravel, Effect::RainbowSections, Effect::WaveChase, Effect::AlternatingGlow, Effect::PulsingGreen, Effect::SolidGreen, Effect::RacingChase, Effect::Fireworks, Effect::CheckerPulse]
    }

    pub fn index(self) -> usize {
        Self::all().iter().position(|&e| e == self).unwrap()
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Effect::Off => "Off",
            Effect::Standby => "Standby",
            Effect::BlueRedChase => "Blue Red Chase",
            Effect::RainbowTravel => "Rainbow Travel",
            Effect::RainbowSections => "Rainbow Sections",
            Effect::WaveChase => "Wave Chase",
            Effect::AlternatingGlow => "Alternating Glow",
            Effect::PulsingGreen => "Pulsing Green",
            Effect::SolidGreen => "Solid Green",
            Effect::RacingChase => "Racing Chase",
            Effect::Fireworks => "Fireworks",
            Effect::CheckerPulse => "Checker Pulse",
        }
    }
}

pub fn update_leds(data: &mut [RGB8; TOTAL_LEDS], effect: Effect, time: usize) {
    for i in 0..TOTAL_LEDS {
        let local = i % NUM_LEDS;
        let j = if i >= NUM_LEDS {
            TOTAL_LEDS - 1 - local // Mirror index for second set
        } else {
            local
        };
        data[j] = match effect {
            Effect::Off => RGB8::default(),
            Effect::Standby => match local {
                // Corner markers: LEDs 1,2,31,32 (0-indexed: 0,1,30,31) — amber half-brightness
                0 | 1 => RGB8 { r: 128, g: 80, b: 0 },
                // Centre markers: LEDs 13,14,19,20 (0-indexed: 12,13,18,19) — white half-brightness
                28 | 29 => RGB8 { r: 128, g: 128, b: 128 },
                _ => RGB8::default(),
            },
            Effect::BlueRedChase => {
                // Chasing blue and red LEDs
                let position = (time / 3) % (NUM_LEDS * 2);
                let led_pos = if position < NUM_LEDS {
                    position
                } else {
                    NUM_LEDS * 2 - position - 1
                };
                if local == led_pos {
                    if position < NUM_LEDS {
                        RGB8 { r: 0, g: 0, b: 255 } // Blue
                    } else {
                        RGB8 { r: 255, g: 0, b: 0 } // Red
                    }
                } else {
                    RGB8::default()
                }
            }
            Effect::RainbowTravel => {
                // Rainbow colors traveling along the strip
                let hue = ((local * 360 / NUM_LEDS) + time) % 360;
                hsv_to_rgb(hue as u16, 255)
            }
            Effect::RainbowSections => {
                // Divide strip into sections, each getting a rainbow color
                let section_size = NUM_LEDS / RAINBOW.len();
                let section = local / section_size;
                let color_idx = (section + time / 10) % RAINBOW.len();
                RAINBOW[color_idx]
            }
            Effect::WaveChase => {
                // Wave effect moving along the strip
                let wave_position = (time / 2 + local) % (RAINBOW.len() * 2);
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
                let pattern = (local + time / 5) % 3;
                match pattern {
                    0 => RAINBOW[(time / 8) % RAINBOW.len()],
                    1 => RGB8::default(),
                    _ => {
                        let color = RAINBOW[(time / 8 + 3) % RAINBOW.len()];
                        RGB8 {
                            r: color.r / 3,
                            g: color.g / 3,
                            b: color.b / 3,
                        }
                    }
                }
            }
            Effect::PulsingGreen => {
                // Slow triangle-wave pulse, ~3 second period (38 ticks at 80ms)
                let period = 38usize;
                let phase = time % period;
                let brightness = if phase < period / 2 {
                    (phase * 510 / period) as u8
                } else {
                    ((period - phase) * 510 / period) as u8
                };
                RGB8 { r: 0, g: brightness, b: 0 }
            }
            Effect::SolidGreen => RGB8 { r: 0, g: 255, b: 0 },
            Effect::RacingChase => {
                // Fast rainbow sweep, 15× speed of RainbowTravel
                let hue = ((local * 360 / NUM_LEDS) + time * 90) % 360;
                hsv_to_rgb(hue as u16, 255)
            }
            Effect::Fireworks => {
                // Celebration: full rainbow across all LEDs with random white sparkles
                let base_hue = ((local * 360 / NUM_LEDS) + time * 3) % 360;
                let base_color = hsv_to_rgb(base_hue as u16, 220);
                // Sparkle lasts ~3 ticks (~240ms) for a smooth flash feel
                let sparkle_tick = time / 3;
                let hash = local.wrapping_mul(2654435761usize)
                    ^ sparkle_tick.wrapping_mul(2246822519usize);
                if hash & 0xFF < 40 {
                    RGB8 { r: 255, g: 255, b: 255 }
                } else {
                    base_color
                }
            }
            Effect::CheckerPulse => {
                // Alternating white LEDs pulse in antiphase: even positions fade in while
                // odd positions fade out, then swap — ~1.6 s per full cycle at 80 ms ticks.
                let period = 10usize;
                let phase = time % period;
                let brightness = if phase < period / 2 {
                    (phase * 510 / period) as u8
                } else {
                    ((period - phase) * 510 / period) as u8
                };
                let v = if local % 2 == 0 { brightness } else { 255 - brightness };
                RGB8 { r: v, g: v, b: v }
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

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub fn as_str(self) -> &'static str {
        match self {
            Axis::X => "X",
            Axis::Y => "Y",
            Axis::Z => "Z",
        }
    }
}

impl Accel {
    pub fn dominant_axis(self) -> Axis {
        let x = self.x.abs();
        let y = self.y.abs();
        let z = self.z.abs();

        if x >= y && x >= z {
            Axis::X
        } else if y >= x && y >= z {
            Axis::Y
        } else {
            Axis::Z
        }
    }
}