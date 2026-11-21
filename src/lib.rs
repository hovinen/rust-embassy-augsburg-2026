//! Hardware-independent logic of the firmware.
//!
//! This crate is `#![no_std]` so the firmware can link it, and it is unit
//! tested on the host with `cargo test`.

#![no_std]

// Tests run on the host, where a global allocator exists. The firmware
// build (non-test) never compiles this, so it stays alloc-free.
#[cfg(test)]
extern crate alloc;

#[derive(Debug, Clone, Copy)]
pub struct Reading {
    pub humidity_pp: u8,
    pub humidity_dp: u8,
    pub temperature_c: u8,
    pub temperature_dp: u8,
}

pub enum Dhl11Decoder {
    Collecting { bytes: [u8; 5], bit_index: u8 },
    Failed(DecodeError),
}

impl Dhl11Decoder {
    pub fn new() -> Self {
        Self::Collecting {
            bytes: [0; 5],
            bit_index: 0,
        }
    }

    pub fn push_pulse(&mut self, width_us: u32) {
        match self {
            Self::Collecting { bytes, bit_index } => {
                if *bit_index >= 40 {
                    *self = Self::Failed(DecodeError::TooManySamples(*bit_index));
                    return;
                }
                if !(15..=95).contains(&width_us) {
                    *self = Self::Failed(DecodeError::BitsOutOfTolerance(width_us, *bit_index));
                    return;
                }
                let bit = (width_us > 45) as u8;
                bytes[(*bit_index / 8) as usize] <<= 1;
                bytes[(*bit_index / 8) as usize] |= bit;
                *bit_index += 1;
            }
            Self::Failed(DecodeError::TooManySamples(count)) => {
                *count += 1;
            }
            Self::Failed(_) => {}
        }
    }

    pub fn finish(self) -> Result<Reading, DecodeError> {
        match self {
            Dhl11Decoder::Collecting { bytes, bit_index } => {
                if bit_index < 40 {
                    return Err(DecodeError::NotEnoughSamples(bit_index));
                }
                let checksum =
                    (bytes[0] as u16 + bytes[1] as u16 + bytes[2] as u16 + bytes[3] as u16) & 0xff;
                if checksum == bytes[4] as u16 {
                    Ok(Reading {
                        humidity_pp: bytes[0],
                        humidity_dp: bytes[1],
                        temperature_c: bytes[2],
                        temperature_dp: bytes[3],
                    })
                } else {
                    Err(DecodeError::BadChecksum)
                }
            }
            Dhl11Decoder::Failed(error) => Err(error),
        }
    }
}

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    #[error("Too many samples")]
    TooManySamples(u8),
    #[error("Not enough samples")]
    NotEnoughSamples(u8),
    #[error("Bad checksum")]
    BadChecksum,
    #[error("Bits out of tolerance")]
    BitsOutOfTolerance(u32, u8),
}

#[cfg(test)]
mod tests {
    use super::{DecodeError, Dhl11Decoder, Reading};
    use alloc::vec;
    use alloc::vec::Vec;
    use test_that::prelude::*;

    /// Encodes five bytes as 40 pulse widths: 46 µs for a 1 bit, 45 µs for a 0.
    fn widths_for_bytes(bytes: [u8; 5]) -> Vec<u32> {
        let mut widths = Vec::with_capacity(40);
        for byte in bytes {
            for shift in (0..8).rev() {
                widths.push(if (byte >> shift) & 1 == 1 { 46 } else { 45 });
            }
        }
        widths
    }

    fn decode(widths: &[u32]) -> Result<Reading, DecodeError> {
        let mut decoder = Dhl11Decoder::new();
        for &width in widths {
            decoder.push_pulse(width);
        }
        decoder.finish()
    }

    #[rstest::rstest]
    // All zeros, checksum 0.
    #[case([0, 0, 0, 0, 0], 0, 0, 0, 0)]
    // 45.2 %RH, 23.0 °C: checksum 45 + 2 + 23 + 0 = 70.
    #[case([45, 2, 23, 0, 70], 45, 2, 23, 0)]
    // All 0xFF: checksum (4 * 255) & 0xff = 252.
    #[case([255, 255, 255, 255, 252], 255, 255, 255, 255)]
    fn decodes_valid_frames(
        #[case] bytes: [u8; 5],
        #[case] humidity_pp_expected: u8,
        #[case] humidity_dp_expected: u8,
        #[case] temperature_c_expected: u8,
        #[case] temperature_dp_expected: u8,
    ) -> TestResult<()> {
        verify_that!(
            decode(&widths_for_bytes(bytes)),
            ok(matches_pattern!(Reading {
                humidity_pp: eq(humidity_pp_expected),
                humidity_dp: eq(humidity_dp_expected),
                temperature_c: eq(temperature_c_expected),
                temperature_dp: eq(temperature_dp_expected),
            }))
        )
    }

    #[rstest::rstest]
    #[case(vec![], DecodeError::NotEnoughSamples(0))]
    #[case(vec![45; 39], DecodeError::NotEnoughSamples(39))]
    // 40 valid pulses plus one extra: the 41st is rejected, not buffered.
    #[case(vec![45; 41], DecodeError::TooManySamples(40))]
    #[case(vec![14], DecodeError::BitsOutOfTolerance(14, 0))]
    #[case(vec![96], DecodeError::BitsOutOfTolerance(96, 0))]
    // Out of tolerance mid-frame reports the offending pulse index.
    #[case([vec![45; 7], vec![10], vec![45; 32]].concat(), DecodeError::BitsOutOfTolerance(10, 7))]
    // Wrong checksum byte (should be 1).
    #[case(widths_for_bytes([1, 0, 0, 0, 0]), DecodeError::BadChecksum)]
    fn rejects_invalid_input(#[case] widths: Vec<u32>, #[case] expected: DecodeError) -> TestResult<()> {
        verify_that!(decode(&widths), err(eq(expected)))
    }

    #[rstest::rstest]
    // Bit threshold: 45 µs decodes as 0, 46 µs as 1.
    #[case(45, 0u8)]
    #[case(46, 1u8)]
    // Tolerance range 15..=95 µs is inclusive at both ends.
    #[case(15, 0u8)]
    #[case(95, 1u8)]
    fn decodes_boundary_widths(#[case] width: u32, #[case] expected_bit: u8) -> TestResult<()> {
        let first_byte = expected_bit << 7;
        let mut widths = widths_for_bytes([first_byte, 0, 0, 0, first_byte]);
        widths[0] = width;
        verify_that!(
            decode(&widths),
            ok(matches_pattern!(Reading {
                humidity_pp: eq(first_byte),
                humidity_dp: eq(0),
                temperature_c: eq(0),
                temperature_dp: eq(0),
            }))
        )
    }

    #[test]
    fn failure_is_sticky() -> TestResult<()> {
        let mut decoder = Dhl11Decoder::new();
        decoder.push_pulse(10); // out of tolerance at index 0
        decoder.push_pulse(45);
        decoder.push_pulse(46);
        verify_that!(
            decoder.finish(),
            err(eq(DecodeError::BitsOutOfTolerance(10, 0)))
        )
    }
}
