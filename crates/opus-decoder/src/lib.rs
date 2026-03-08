//! Pure Rust Opus decoder (decoder-first), no FFI.
//!
//! This crate is currently a scaffold; the decoding algorithms are ported in
//! stages with RFC conformance vectors as the primary gate.

#![forbid(unsafe_code)]
#![allow(
    clippy::erasing_op,
    clippy::identity_op,
    clippy::precedence,
    clippy::int_plus_one
)]

macro_rules! debug_trace {
    ($($arg:tt)*) => {{
        if false {
            eprintln!($($arg)*);
        }
    }};
}

mod error;
mod celt;
mod silk;
mod entropy;
mod packet;
pub mod compare;
use core::sync::atomic::{AtomicUsize, Ordering};

pub use error::Error;

static TRACE_DECODE_PACKET_IDX: AtomicUsize = AtomicUsize::new(0);

#[derive(Debug, Clone)]
pub struct Decoder {
    fs_hz: u32,
    channels: u8,
    celt: celt::CeltDecoder,
    silk: silk::SilkDecoder,
    last_packet_split_count: usize,
    last_final_range: u32,
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
            silk: silk::SilkDecoder::new(fs_hz, channels),
            last_packet_split_count: 0,
            last_final_range: 0,
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
        self.silk.reset();
        self.last_packet_split_count = 0;
        self.last_final_range = 0;
    }

    /// Return range coder final state for conformance comparison.
    pub fn final_range(&self) -> u32 {
        self.last_final_range
    }

    /// Return recursive split count from last decoded frame.
    ///
    /// This is a debug metric used for CELT split-path diagnostics.
    pub fn last_split_count(&self) -> usize {
        self.last_packet_split_count
    }

    /// Return deemphasis memory for channel 0 (debug metric).
    ///
    /// This is used for CELT synthesis diagnostics in mono vectors.
    pub fn deemph_mem(&self) -> f32 {
        self.celt.deemph_mem(0)
    }

    /// Return whether the last CELT frame was transient.
    ///
    /// This is a compatibility helper for the local conformance harness.
    pub fn last_is_transient(&self) -> bool {
        false
    }

    /// Decode one Opus packet to interleaved i16 PCM.
    ///
    /// - `packet=None` triggers PLC (packet loss concealment).
    /// - `out` must be large enough for the maximum frame size (120 ms).
    /// - Returns the number of samples per channel written.
    pub fn decode(&mut self, packet: Option<&[u8]>, out: &mut [i16]) -> Result<usize, Error> {
        let packet_idx = TRACE_DECODE_PACKET_IDX.fetch_add(1, Ordering::SeqCst);
        let trace_packet_idx = None;
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
        let silk_trace = std::env::var("OPUS_SILK_TRACE").ok().as_deref() == Some("1");
        let mode = match config {
            0..=11 => OpusMode::SilkOnly,
            12..=15 => OpusMode::Hybrid,
            16..=31 => OpusMode::CeltOnly,
            _ => unreachable!(),
        };
        let mode_name = match mode {
            OpusMode::SilkOnly => "silk",
            OpusMode::Hybrid => "hybrid",
            OpusMode::CeltOnly => "celt",
        };
        // #region agent log
        let log_path = std::env::current_dir()
            .unwrap_or_else(|_| std::path::PathBuf::from("."))
            .join(".cursor")
            .join("debug-bea564.log");
        if let Ok(mut f) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(log_path)
        {
            let ts = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis())
                .unwrap_or(0);
            let line = format!(
                "{{\"sessionId\":\"bea564\",\"runId\":\"run-tv01-tv10-debug1\",\"hypothesisId\":\"H1\",\"location\":\"crates/opus-decoder/src/lib.rs:mode_gate\",\"message\":\"decoder_mode_gate\",\"data\":{{\"packet_idx\":{},\"toc\":{},\"config\":{},\"mode\":\"{}\",\"decoder_channels\":{},\"samples_per_channel_needed\":{}}},\"timestamp\":{}}}\n",
                packet_idx,
                toc,
                config,
                mode_name,
                self.channels,
                samples_per_channel_needed,
                ts
            );
            let _ = std::io::Write::write_all(&mut f, line.as_bytes());
        }
        // #endregion
        if mode == OpusMode::Hybrid {
            // #region agent log
            let log_path = std::env::current_dir()
                .unwrap_or_else(|_| std::path::PathBuf::from("."))
                .join(".cursor")
                .join("debug-bea564.log");
            if let Ok(mut f) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(log_path)
            {
                let ts = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_millis())
                    .unwrap_or(0);
                let line = format!(
                    "{{\"sessionId\":\"bea564\",\"runId\":\"run-tv01-tv10-debug1\",\"hypothesisId\":\"H4\",\"location\":\"crates/opus-decoder/src/lib.rs:mode_gate\",\"message\":\"decoder_mode_not_supported\",\"data\":{{\"packet_idx\":{},\"config\":{},\"mode\":\"{}\"}},\"timestamp\":{}}}\n",
                    packet_idx, config, mode_name, ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
            return Err(Error::NotImplemented);
        }

        // Parse again to get per-frame slices.
        let pp = packet::parse_packet(packet.unwrap())?;
        if silk_trace && packet_idx <= 2 && mode == OpusMode::SilkOnly {
            eprintln!(
                "SILK pkt{} toc={:#04x} config={} code={} nb_frames={} samples_per_frame_48k={} samples_needed={}",
                packet_idx,
                pp.toc,
                config,
                pp.toc & 0x03,
                pp.frame_count,
                pp.samples_per_frame_48k,
                samples_per_channel_needed
            );
        }
        if trace_packet_idx == Some(packet_idx) {
            // #region agent log
            if let Ok(mut f) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open("/Users/tadeusz/Opus/Rasopus/.cursor/debug-4200c5.log")
            {
                let ts = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_millis())
                    .unwrap_or(0);
                let line = format!(
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H1\",\"location\":\"crates/opus-decoder/src/lib.rs:151\",\"message\":\"packet_vs_decoder_channels\",\"data\":{{\"packet_idx\":{},\"packet_channels\":{},\"decoder_channels\":{},\"toc\":{},\"config\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    pp.packet_channels,
                    self.channels,
                    pp.toc,
                    config,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        self.last_packet_split_count = 0;
        self.last_final_range = 0;

        let mut written_per_channel = 0usize;
        for frame in pp.frames() {
            let frame_samples_48k = pp.samples_per_frame_48k;
            let out_frame = &mut out[written_per_channel * self.channels as usize..];
            match mode {
                OpusMode::CeltOnly => {
                    // CELT frame sizes are specified in 48 kHz samples; this is enough
                    // to drive the CELT-side LM selection.
                    let frame = self.celt.decode_frame(
                        frame,
                        frame_samples_48k,
                        config,
                        pp.packet_channels,
                        packet_idx,
                        out_frame,
                    )?;
                    self.last_packet_split_count += self.celt.last_split_count();
                    self.last_final_range = self.celt.final_range();
                    written_per_channel += frame.samples_per_channel;
                }
                OpusMode::SilkOnly => {
                    let frame = self.silk.decode_frame(
                        frame,
                        frame_samples_48k,
                        config,
                        pp.packet_channels,
                        packet_idx,
                        out_frame,
                    )?;
                    self.last_final_range = self.silk.final_range();
                    written_per_channel += frame.samples_per_channel;
                    if silk_trace && packet_idx <= 2 {
                        eprintln!(
                            "SILK pkt{} frame_returned={} written_total={} frame_samples_48k={}",
                            packet_idx,
                            frame.samples_per_channel,
                            written_per_channel,
                            frame_samples_48k
                        );
                    }
                }
                OpusMode::Hybrid => return Err(Error::NotImplemented),
            }
        }

        Ok(written_per_channel)
    }
}
