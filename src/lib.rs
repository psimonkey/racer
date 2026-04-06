//! Business logic for Racer project.

#![no_std]

use smart_leds::RGB8;

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