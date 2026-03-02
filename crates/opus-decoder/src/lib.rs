//! Pure Rust Opus decoder (decoder-first), no FFI.
//!
//! This crate is currently a scaffold; the decoding algorithms are ported in
//! stages with RFC conformance vectors as the primary gate.

#![forbid(unsafe_code)]

mod error;
mod celt;
mod entropy;
mod packet;
pub mod compare;

pub use error::Error;

#[derive(Debug, Clone)]
pub struct Decoder {
    fs_hz: u32,
    channels: u8,
    celt: celt::CeltDecoder,
    last_packet_split_count: usize,
    last_packet_is_transient: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OpusMode {
    SilkOnly,
    Hybrid,
    CeltOnly,
}

impl Decoder {
    pub const MAX_FRAME_SIZE_48K: usize = 5760; // 120 ms @ 48 kHz

    pub fn new(fs_hz: u32, channels: u8) -> Result<Self, Error> {
        if !matches!(fs_hz, 8000 | 12000 | 16000 | 24000 | 48000) {
            return Err(Error::InvalidSampleRate(fs_hz));
        }
        if !matches!(channels, 1 | 2) {
            return Err(Error::InvalidChannels(channels));
        }
        Ok(Self {
            fs_hz,
            channels,
            celt: celt::CeltDecoder::new(fs_hz, channels),
            last_packet_split_count: 0,
            last_packet_is_transient: false,
        })
    }

    pub fn fs_hz(&self) -> u32 {
        self.fs_hz
    }

    pub fn channels(&self) -> u8 {
        self.channels
    }

    pub fn max_frame_size_per_channel(&self) -> usize {
        // Opus internally decodes 48 kHz and can output other Fs by deterministic
        // downsampling/decimation (libopus behavior).
        match self.fs_hz {
            8000 => 960,
            12000 => 1440,
            16000 => 1920,
            24000 => 2880,
            48000 => Self::MAX_FRAME_SIZE_48K,
            _ => 0, // guarded in new()
        }
    }

    pub fn reset(&mut self) {
        // Placeholder. The real implementation will reset SILK/CELT state.
        self.celt.reset();
        self.last_packet_split_count = 0;
        self.last_packet_is_transient = false;
    }

    /// Return range coder final state for conformance comparison.
    pub fn final_range(&self) -> u32 {
        self.celt.final_range()
    }

    /// Return recursive split count from last decoded frame.
    ///
    /// This is a debug metric used for CELT split-path diagnostics.
    pub fn last_split_count(&self) -> usize {
        self.last_packet_split_count
    }

    /// Return transient flag from the last decoded CELT frame.
    ///
    /// This is a debug metric used for synthesis-path diagnostics.
    pub fn last_is_transient(&self) -> bool {
        self.last_packet_is_transient
    }

    /// Return deemphasis memory for channel 0 (debug metric).
    ///
    /// This is used for CELT synthesis diagnostics in mono vectors.
    pub fn deemph_mem(&self) -> f32 {
        self.celt.deemph_mem(0)
    }

    /// Decode one Opus packet to interleaved i16 PCM.
    ///
    /// - `packet=None` triggers PLC (packet loss concealment).
    /// - `out` must be large enough for the maximum frame size (120 ms).
    /// - Returns the number of samples per channel written.
    pub fn decode(&mut self, packet: Option<&[u8]>, out: &mut [i16]) -> Result<usize, Error> {
        let (toc, samples_per_channel_needed) = match packet {
            Some(packet) => {
                let pp = packet::parse_packet(packet)?;
                (Some(pp.toc), pp.samples_per_channel(self.fs_hz))
            }
            None => (None, self.max_frame_size_per_channel()),
        };

        let needed = samples_per_channel_needed * self.channels as usize;
        if out.len() < needed {
            return Err(Error::OutputTooSmall {
                needed,
                got: out.len(),
            });
        }

        let Some(toc) = toc else {
            // PLC not implemented yet.
            return Err(Error::NotImplemented);
        };

        let config = (toc >> 3) & 0x1f;
        let mode = match config {
            0..=11 => OpusMode::SilkOnly,
            12..=15 => OpusMode::Hybrid,
            16..=31 => OpusMode::CeltOnly,
            _ => unreachable!(),
        };
        if mode != OpusMode::CeltOnly {
            return Err(Error::NotImplemented);
        }

        // Parse again to get per-frame slices.
        let pp = packet::parse_packet(packet.unwrap())?;
        self.last_packet_split_count = 0;
        self.last_packet_is_transient = false;

        let mut written_per_channel = 0usize;
        for frame in pp.frames() {
            // CELT frame sizes are specified in 48 kHz samples; this is enough
            // to drive the CELT-side LM selection.
            let frame_samples_48k = pp.samples_per_frame_48k;
            let frame = self
                .celt
                .decode_frame(
                    frame,
                    frame_samples_48k,
                    config,
                    &mut out[written_per_channel * self.channels as usize..],
                )?;
            self.last_packet_split_count += self.celt.last_split_count();
            self.last_packet_is_transient = frame.is_transient;
            written_per_channel += frame.samples_per_channel;
        }

        Ok(written_per_channel)
    }
}
