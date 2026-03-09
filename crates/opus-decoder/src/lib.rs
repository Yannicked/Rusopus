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

mod celt;
pub mod compare;
mod entropy;
mod error;
mod packet;
mod silk;
use core::sync::atomic::{AtomicUsize, Ordering};
use crate::entropy::EcDec;

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
    prev_mode: Option<OpusMode>,
    prev_redundancy: bool,
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
            prev_mode: None,
            prev_redundancy: false,
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
        self.prev_mode = None;
        self.prev_redundancy = false;
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
        if std::env::var("OPUS_DUMP_MODES").is_ok() {
            eprintln!(
                "PKT{} mode={:?} prev_mode={:?}",
                packet_idx, mode, self.prev_mode
            );
        }
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
                packet_idx, toc, config, mode_name, self.channels, samples_per_channel_needed, ts
            );
            let _ = std::io::Write::write_all(&mut f, line.as_bytes());
        }
        // #endregion
        // Parse again to get per-frame slices.
        let pp = packet::parse_packet(packet.unwrap())?;
        if (600..=604).contains(&packet_idx) {
            // #region agent log H57
            eprintln!(
                "DBG H57 rust pkt{} toc={:#04x} packet_channels={} prev_channels={} mode={:?}",
                packet_idx,
                toc,
                pp.packet_channels,
                self.silk.prev_packet_channels(),
                mode
            );
            // #endregion
        }
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
                    packet_idx, pp.packet_channels, self.channels, pp.toc, config, ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        self.last_packet_split_count = 0;
        self.last_final_range = 0;

        let transition = self.prev_mode.is_some()
            && ((mode == OpusMode::CeltOnly
                && self.prev_mode != Some(OpusMode::CeltOnly)
                && !self.prev_redundancy)
                || (mode != OpusMode::CeltOnly && self.prev_mode == Some(OpusMode::CeltOnly)));
        let transition_samples = (self.fs_hz as usize) / 200;
        let transition_overlap = (self.fs_hz as usize) / 400;
        let channels = self.channels as usize;
        let celt_transition = if transition && mode == OpusMode::CeltOnly {
            vec![0i16; transition_samples * channels]
        } else {
            Vec::new()
        };
        let mut apply_celt_transition = transition && mode == OpusMode::CeltOnly;
        let mut reset_silk = transition && self.prev_mode == Some(OpusMode::CeltOnly);
        let mut had_redundancy = false;
        let mut written_per_channel = 0usize;
        for frame in pp.frames() {
            let frame_samples_48k = pp.samples_per_frame_48k;
            let out_frame = &mut out[written_per_channel * self.channels as usize..];
            if (1116..=1120).contains(&packet_idx) {
                // #region agent log H94
                eprintln!(
                    "TV12 pkt{} mode={:?} config={} packet_channels={} prev_mode={:?} prev_redundancy={}",
                    packet_idx,
                    mode,
                    config,
                    pp.packet_channels,
                    self.prev_mode,
                    self.prev_redundancy
                );
                // #endregion
            }
            match mode {
                OpusMode::CeltOnly => {
                    if apply_celt_transition {
                        self.celt.reset();
                    }
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
                    if apply_celt_transition {
                        apply_transition_fade_i16(
                            &celt_transition,
                            &mut out_frame[..frame.samples_per_channel * channels],
                            transition_overlap.min(frame.samples_per_channel / 2),
                            channels,
                            self.celt.window(),
                            self.fs_hz,
                        );
                        apply_celt_transition = false;
                    }
                    self.last_packet_split_count += self.celt.last_split_count();
                    self.last_final_range = self.celt.final_range();
                    written_per_channel += frame.samples_per_channel;
                }
                OpusMode::SilkOnly => {
                    if reset_silk {
                        self.silk.reset();
                        reset_silk = false;
                    }
                    let packet_frame = frame;
                    let silk_frame = self.silk.decode_frame(
                        packet_frame,
                        frame_samples_48k,
                        config,
                        pp.packet_channels,
                        packet_idx,
                        out_frame,
                    )?;
                    if packet_idx == 4 {
                        // #region agent log H47
                        eprintln!(
                            "DBG H47 rust pkt4 top_level_silk frame_samples_48k={} packet_channels={} written={} final_range={} consumed_redundancy={} redundancy_bytes={} celt_to_silk={} crossfade_applied=false",
                            frame_samples_48k,
                            pp.packet_channels,
                            silk_frame.samples_per_channel,
                            self.silk.final_range(),
                            silk_frame.consumed_redundancy,
                            silk_frame.redundancy_bytes,
                            silk_frame.celt_to_silk
                        );
                        // #endregion
                    }
                    if matches!(packet_idx, 99) {
                        eprintln!(
                            "DBG H57 rust pkt{} top_level_silk packet_channels={} written={} final_range={} has_redundancy={} redundancy_bytes={} celt_to_silk={}",
                            packet_idx,
                            pp.packet_channels,
                            silk_frame.samples_per_channel,
                            self.silk.final_range(),
                            silk_frame.consumed_redundancy,
                            silk_frame.redundancy_bytes,
                            silk_frame.celt_to_silk
                        );
                    }
                    let mut redundancy_rng = 0u32;
                    if (1116..=1120).contains(&packet_idx) {
                        // #region agent log H95
                        eprintln!(
                            "TV12 pkt{} silk_frame has_redundancy={} redundancy_bytes={} celt_to_silk={} frame_samples={}",
                            packet_idx,
                            silk_frame.consumed_redundancy,
                            silk_frame.redundancy_bytes,
                            silk_frame.celt_to_silk,
                            silk_frame.samples_per_channel
                        );
                        // #endregion
                    }
                    if packet_idx == 136 {
                        // #region agent log H90
                        eprintln!(
                            "TV12 pkt136 silk_redundancy consumed={} bytes={} celt_to_silk={} samples_per_channel={}",
                            silk_frame.consumed_redundancy,
                            silk_frame.redundancy_bytes,
                            silk_frame.celt_to_silk,
                            silk_frame.samples_per_channel
                        );
                        // #endregion
                    }
                    if silk_frame.consumed_redundancy {
                        let redundancy_data =
                            &packet_frame[packet_frame.len() - silk_frame.redundancy_bytes..];
                        let redundancy_frame_size_48k = 240usize;
                        let redundancy_samples = (self.fs_hz as usize) / 200;
                        let redundancy_end_band = silk_redundancy_end_band(config);
                        let mut redundancy_out =
                            vec![0i16; redundancy_samples * self.channels as usize];
                        self.celt.reset();
                        self.celt.set_start_band(0);
                        self.celt.set_end_band(redundancy_end_band);
                        let redundancy_frame = self.celt.decode_frame(
                            redundancy_data,
                            redundancy_frame_size_48k,
                            config,
                            pp.packet_channels,
                            packet_idx,
                            &mut redundancy_out,
                        )?;
                        self.celt.clear_end_band();
                        if packet_idx == 136 {
                            // #region agent log H93
                            let probe_end = 16.min(redundancy_out.len());
                            let abs_sum: f32 =
                                redundancy_out.iter().map(|x| x.abs() as f32).sum();
                            eprintln!(
                                "TV12 pkt136 redundancy_end_band={} redundancy_out[0..16]={:?}",
                                redundancy_end_band,
                                &redundancy_out[..probe_end]
                            );
                            eprintln!("TV12 pkt136 redundancy_out abs_sum={abs_sum:.2}");
                            // #endregion
                        }
                        redundancy_rng = self.celt.final_range();
                        let xor_range = self.silk.final_range() ^ redundancy_rng;
                        if packet_idx == 1118 {
                            // #region agent log H96
                            eprintln!(
                                "TV12 pkt1118 redundancy_rng={:#010x} xor_range={:#010x} celt_to_silk={}",
                                redundancy_rng,
                                xor_range,
                                silk_frame.celt_to_silk
                            );
                            // #endregion
                        }
                        if !silk_frame.celt_to_silk {
                            let f5 = (self.fs_hz as usize) / 200;
                            let overlap = (self.fs_hz as usize) / 400;
                            let channels = self.channels as usize;
                            let silk_tail_start =
                                (silk_frame.samples_per_channel - overlap) * channels;
                            let silk_tail_end = silk_frame.samples_per_channel * channels;
                            let redundancy_start = overlap * channels;
                            let redundancy_end = redundancy_start + overlap * channels;
                            if packet_idx == 136 {
                                // #region agent log H91
                                let tail_probe_end = (silk_tail_start + 8).min(out_frame.len());
                                let red0_end = 8.min(redundancy_out.len());
                                let red1_end = (redundancy_start + 8).min(redundancy_out.len());
                                eprintln!(
                                    "TV12 pkt136 frame_size={} f5={} f2_5={} channels={}",
                                    silk_frame.samples_per_channel,
                                    f5,
                                    overlap,
                                    self.channels
                                );
                                eprintln!(
                                    "TV12 pkt136 fade_start={} fade_end={} copy_start={}",
                                    (silk_frame.samples_per_channel - f5) * channels,
                                    (silk_frame.samples_per_channel - overlap) * channels,
                                    (silk_frame.samples_per_channel - overlap) * channels
                                );
                                eprintln!(
                                    "TV12 pkt136 pre_fade tail0_8={:?} redund0_8={:?} redund120_8={:?}",
                                    &out_frame[silk_tail_start..tail_probe_end],
                                    &redundancy_out[..red0_end],
                                    &redundancy_out[redundancy_start..red1_end]
                                );
                                // #endregion
                            }
                            let silk_tail = out_frame[silk_tail_start..silk_tail_end].to_vec();
                            smooth_fade_i16(
                                &silk_tail,
                                &redundancy_out[redundancy_start..redundancy_end],
                                &mut out_frame[silk_tail_start..silk_tail_end],
                                overlap,
                                channels,
                                self.celt.window(),
                                self.fs_hz,
                            );
                            if packet_idx == 136 {
                                // #region agent log H92
                                let tail_probe_end = (silk_tail_start + 8).min(out_frame.len());
                                eprintln!(
                                    "TV12 pkt136 post_fade tail0_8={:?}",
                                    &out_frame[silk_tail_start..tail_probe_end]
                                );
                                // #endregion
                            }
                        } else if packet_idx == 1118 {
                            // #region agent log H97
                            eprintln!("TV12 pkt1118 celt_to_silk branch not applied");
                            // #endregion
                        }
                        if packet_idx == 4 {
                            // #region agent log H50
                            let fade_start = ((self.fs_hz as usize) / 400) * self.channels as usize;
                            let fade_end = (fade_start + 8).min(redundancy_out.len());
                            eprintln!(
                                "DBG H50 rust pkt4 redundancy_probe bytes={} decoded={} redundant_rng={:#010x} xor_final_range={:#010x} out0_8={:?} fade_in_8={:?}",
                                silk_frame.redundancy_bytes,
                                redundancy_frame.samples_per_channel,
                                redundancy_rng,
                                xor_range,
                                &redundancy_out[..8.min(redundancy_out.len())],
                                &redundancy_out[fade_start..fade_end]
                            );
                            if !silk_frame.celt_to_silk {
                                let energy = self.celt.prev_energy_head(5);
                                eprintln!(
                                    "R pkt4 post_redund decode_mem_abs={:.4} right={:.4} deemph={:.6} {:.6}",
                                    self.celt.decode_mem_abs_sum(),
                                    self.celt.decode_mem_right_abs_sum(),
                                    self.celt.deemph_mem(0),
                                    self.celt.deemph_mem(1)
                                );
                                eprintln!(
                                    "R pkt4 post_redund energy[0..{}]={:?}",
                                    energy.len(),
                                    energy
                                );
                            }
                            // #endregion
                        }
                    }
                    had_redundancy = silk_frame.consumed_redundancy && !silk_frame.celt_to_silk;
                    self.last_final_range = self.silk.final_range() ^ redundancy_rng;
                    written_per_channel += silk_frame.samples_per_channel;
                    if silk_trace && packet_idx <= 2 {
                        eprintln!(
                            "SILK pkt{} frame_returned={} written_total={} frame_samples_48k={}",
                            packet_idx,
                            silk_frame.samples_per_channel,
                            written_per_channel,
                            frame_samples_48k
                        );
                    }
                }
                OpusMode::Hybrid => {
                    if packet_idx <= 1 {
                        // #region agent log H61
                        eprintln!(
                            "HYBRID pkt{} frame.len()={} config={} channels={}",
                            packet_idx,
                            frame.len(),
                            config,
                            pp.packet_channels
                        );
                        append_hybrid_debug_log(
                            "hybrid-entry",
                            "H61",
                            packet_idx,
                            &format!(
                                "{{\"frame_len\":{},\"config\":{},\"packet_channels\":{},\"tell\":0,\"tell_frac\":0}}",
                                frame.len(),
                                config,
                                pp.packet_channels
                            ),
                        );
                        // #endregion
                    }
                    let mut ec = EcDec::new(frame);
                    let silk_frame = match self.silk.decode_frame_with_ec(
                        frame,
                        &mut ec,
                        frame_samples_48k,
                        config,
                        pp.packet_channels,
                        true,
                        packet_idx,
                        out_frame,
                    ) {
                        Ok(frame) => {
                            if packet_idx <= 1 {
                                // #region agent log H62
                                if packet_idx == 0 {
                                    eprintln!(
                                        "HYB pkt0 post_silk tell={} rng={:#010x} val={:#010x}",
                                        ec.tell(),
                                        ec.rng(),
                                        ec.debug_val()
                                    );
                                }
                                append_hybrid_debug_log(
                                    "hybrid-after-silk",
                                    "H62",
                                    packet_idx,
                                    &format!(
                                        "{{\"samples_per_channel\":{},\"tell\":{},\"tell_frac\":{},\"rng\":{},\"val\":{}}}",
                                        frame.samples_per_channel,
                                        ec.tell(),
                                        ec.tell_frac(),
                                        ec.rng(),
                                        ec.debug_val()
                                    ),
                                );
                                // #endregion
                            }
                            frame
                        }
                        Err(err) => {
                            if packet_idx <= 1 {
                                // #region agent log H62
                                append_hybrid_debug_log(
                                    "hybrid-silk-error",
                                    "H62",
                                    packet_idx,
                                    &format!(
                                        "{{\"error\":\"{}\",\"tell\":{},\"tell_frac\":{}}}",
                                        err,
                                        ec.tell(),
                                        ec.tell_frac()
                                    ),
                                );
                                // #endregion
                            }
                            return Err(err);
                        }
                    };
                    let redundancy = if ec.tell() + 17 + 20 <= (frame.len() as i32) * 8 {
                        ec.dec_bit_logp(12)
                    } else {
                        false
                    };
                    if packet_idx <= 1 {
                        // #region agent log H63
                        if packet_idx == 0 {
                            eprintln!(
                                "HYB pkt0 post_redund tell={} rng={:#010x} val={:#010x}",
                                ec.tell(),
                                ec.rng(),
                                ec.debug_val()
                            );
                        }
                        append_hybrid_debug_log(
                            "hybrid-redundancy-gate",
                            "H63",
                            packet_idx,
                            &format!(
                                "{{\"redundancy\":{},\"tell\":{},\"tell_frac\":{},\"rng\":{},\"val\":{}}}",
                                redundancy,
                                ec.tell(),
                                ec.tell_frac(),
                                ec.rng(),
                                ec.debug_val()
                            ),
                        );
                        // #endregion
                    }
                    if redundancy {
                        let _celt_to_silk = ec.dec_bit_logp(1);
                        let redundancy_bytes = ec.dec_uint(256) as usize + 2;
                        ec.shrink_storage(redundancy_bytes);
                    }
                    self.celt.set_start_band(17);
                    if packet_idx == 0 {
                        eprintln!(
                            "HYB pkt0 celt_start={} celt_end={}",
                            self.celt.debug_start_band(),
                            self.celt.debug_end_band(config)
                        );
                    }
                    let celt_frame = match self.celt.decode_frame_with_ec(
                        frame,
                        &mut ec,
                        frame_samples_48k,
                        config,
                        pp.packet_channels,
                        packet_idx,
                        out_frame,
                        true,
                    ) {
                        Ok(frame) => frame,
                        Err(err) => {
                            if packet_idx <= 1 {
                                // #region agent log H64
                                append_hybrid_debug_log(
                                    "hybrid-celt-error",
                                    "H64",
                                    packet_idx,
                                    &format!(
                                        "{{\"error\":\"{}\",\"tell\":{},\"tell_frac\":{}}}",
                                        err,
                                        ec.tell(),
                                        ec.tell_frac()
                                    ),
                                );
                                // #endregion
                            }
                            self.celt.set_start_band(0);
                            return Err(err);
                        }
                    };
                    self.celt.set_start_band(0);
                    if packet_idx == 0 {
                        eprintln!("HYB pkt0 celt_rng={:#010x}", self.celt.final_range());
                    }
                    debug_assert_eq!(
                        silk_frame.samples_per_channel,
                        celt_frame.samples_per_channel
                    );
                    self.last_packet_split_count += self.celt.last_split_count();
                    self.last_final_range = self.celt.final_range();
                    had_redundancy = redundancy;
                    written_per_channel += silk_frame.samples_per_channel;
                }
            }
        }

        self.prev_mode = Some(mode);
        self.prev_redundancy = had_redundancy;
        Ok(written_per_channel)
    }
}

/// Crossfade SILK PCM with redundant CELT PCM using the CELT overlap window.
///
/// Params: previous `in1`, incoming `in2`, mutable `out`, overlap length,
/// interleaved `channels`, CELT `window`, and output sampling rate `fs_hz`.
/// Returns: nothing; `out` is updated in-place.
fn smooth_fade_i16(
    in1: &[i16],
    in2: &[i16],
    out: &mut [i16],
    overlap: usize,
    channels: usize,
    window: &[f32],
    fs_hz: u32,
) {
    let inc = (48_000 / fs_hz) as usize;
    for c in 0..channels {
        for i in 0..overlap {
            let w = window[i * inc] * window[i * inc];
            let idx = i * channels + c;
            let mixed = w * in2[idx] as f32 + (1.0 - w) * in1[idx] as f32;
            out[idx] = mixed.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        }
    }
}

/// Apply the SILK-to-CELT transition prefix and crossfade.
///
/// Params: previous-mode `transition` PCM, mutable decoded `pcm`, fade length
/// `overlap`, interleaved `channels`, CELT `window`, and output rate `fs_hz`.
/// Returns: nothing; `pcm` is updated in-place.
fn apply_transition_fade_i16(
    transition: &[i16],
    pcm: &mut [i16],
    overlap: usize,
    channels: usize,
    window: &[f32],
    fs_hz: u32,
) {
    if overlap == 0 || channels == 0 {
        return;
    }

    let prefix_len = overlap * channels;
    let copy_len = prefix_len.min(transition.len()).min(pcm.len());
    pcm[..copy_len].copy_from_slice(&transition[..copy_len]);

    let fade_available = (transition.len().saturating_sub(prefix_len))
        .min(pcm.len().saturating_sub(prefix_len))
        / channels;
    if fade_available == 0 {
        return;
    }

    let fade_samples = fade_available.min(overlap);
    let fade_len = fade_samples * channels;
    let fade_start = prefix_len;
    let fade_end = fade_start + fade_len;
    let incoming = pcm[fade_start..fade_end].to_vec();
    smooth_fade_i16(
        &transition[fade_start..fade_end],
        &incoming,
        &mut pcm[fade_start..fade_end],
        fade_samples,
        channels,
        window,
        fs_hz,
    );
}

/// Map SILK packet config to CELT redundancy end band.
///
/// Params: Opus TOC `config`.
/// Returns: exclusive CELT end band matching libopus packet bandwidth.
fn silk_redundancy_end_band(config: u8) -> usize {
    match config {
        0..=3 => 13,
        4..=11 => 17,
        12..=13 => 19,
        14..=15 => 21,
        _ => 21,
    }
}

/// Append one NDJSON debug entry for hybrid decode tracing.
///
/// Params: event `message`, `hypothesis_id`, `packet_idx`, and JSON `data`.
/// Returns: nothing; debug I/O failures are ignored.
fn append_hybrid_debug_log(message: &str, hypothesis_id: &str, packet_idx: usize, data: &str) {
    use std::io::Write;

    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("/Users/tadeusz/Opus/Rasopus/.cursor/debug-9a4e1c.log")
    {
        let ts = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0);
        let line = format!(
            "{{\"sessionId\":\"9a4e1c\",\"runId\":\"hybrid-entry-trace\",\"hypothesisId\":\"{}\",\"location\":\"crates/opus-decoder/src/lib.rs\",\"message\":\"{}\",\"data\":{{\"packet_idx\":{},\"payload\":{}}},\"timestamp\":{}}}\n",
            hypothesis_id, message, packet_idx, data, ts
        );
        let _ = file.write_all(line.as_bytes());
    }
}
