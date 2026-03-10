//! SILK decoder scaffolding (Phase 3a).
//!
//! This module focuses on bitstream parsing and entropy consumption for SILK
//! packet headers (VAD/LBRR/stereo signaling), with zeroed synthesis output.

use crate::Error;
use crate::entropy::EcDec;

mod decode_core;
mod entropy_tables;
mod gain;
mod lbrr;
mod lpc;
mod ltp;
mod nlsf;
mod pitch;
mod resampler;
mod resampler_private;
mod resampler_rom;
mod stereo;
mod tables;

const MAX_INTERNAL_FRAMES: usize = 3;
const MAX_LPC_HALVES: usize = 2;
const MAX_API_FRAME_SAMPLES_48K: usize = 960 * MAX_INTERNAL_FRAMES;

/// Result of decoding one SILK Opus frame.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SilkFrameDecode {
    /// Number of decoded samples per output channel.
    pub samples_per_channel: usize,
    /// Whether a top-level SILK redundancy trailer was consumed.
    pub consumed_redundancy: bool,
    /// Number of trailing redundancy bytes reserved for CELT.
    pub redundancy_bytes: usize,
    /// Transition direction flag from the top-level redundancy trailer.
    pub celt_to_silk: bool,
}

#[derive(Debug, Clone, Copy, Default)]
struct ChannelHeader {
    vad_flags: [bool; MAX_INTERNAL_FRAMES],
    lbrr_flags: [bool; MAX_INTERNAL_FRAMES],
}

#[derive(Debug, Clone, Copy, Default)]
struct ParsedHeader {
    channels: [ChannelHeader; 2],
}

/// Stateful SILK decoder shell used by top-level Opus decoder.
#[derive(Debug, Clone)]
pub(crate) struct SilkDecoder {
    fs_hz: u32,
    channels: u8,
    final_range: u32,
    lbrr_state: [lbrr::ChannelState; 2],
    core_state: [decode_core::SilkChannelState; 2],
    resampler_state: [Option<resampler::SilkResampler>; 2],
    resampler_history: [[i16; 2]; 2],
    stereo_state: stereo::StereoState,
    prev_nlsf_q15: [[i16; tables::MAX_LPC_ORDER]; 2],
    pred_coef_q12: [[[[i16; tables::MAX_LPC_ORDER]; MAX_LPC_HALVES]; MAX_INTERNAL_FRAMES]; 2],
    first_frame_after_reset: [bool; 2],
    prev_decode_only_middle: bool,
    prev_packet_channels: u8,
}

impl SilkDecoder {
    /// Create SILK decoder state.
    ///
    /// Params: `fs_hz` is Opus API output sample rate, `channels` is output channels.
    /// Returns: initialized SILK decoder state with cleared range state.
    pub fn new(fs_hz: u32, channels: u8) -> Self {
        Self {
            fs_hz,
            channels,
            final_range: 0,
            lbrr_state: [lbrr::ChannelState::default(), lbrr::ChannelState::default()],
            core_state: [
                decode_core::SilkChannelState::default(),
                decode_core::SilkChannelState::default(),
            ],
            resampler_state: [None, None],
            resampler_history: [[0; 2]; 2],
            stereo_state: stereo::StereoState::default(),
            prev_nlsf_q15: [[0; tables::MAX_LPC_ORDER]; 2],
            pred_coef_q12: [[[[0; tables::MAX_LPC_ORDER]; MAX_LPC_HALVES]; MAX_INTERNAL_FRAMES]; 2],
            first_frame_after_reset: [true; 2],
            prev_decode_only_middle: false,
            prev_packet_channels: 1,
        }
    }

    /// Reset SILK decoder persistent state.
    ///
    /// Params: none.
    /// Returns: nothing.
    pub fn reset(&mut self) {
        self.final_range = 0;
        self.lbrr_state = [lbrr::ChannelState::default(), lbrr::ChannelState::default()];
        self.core_state = [
            decode_core::SilkChannelState::default(),
            decode_core::SilkChannelState::default(),
        ];
        self.resampler_state = [None, None];
        self.resampler_history = [[0; 2]; 2];
        self.stereo_state = stereo::StereoState::default();
        self.prev_nlsf_q15 = [[0; tables::MAX_LPC_ORDER]; 2];
        self.pred_coef_q12 =
            [[[[0; tables::MAX_LPC_ORDER]; MAX_LPC_HALVES]; MAX_INTERNAL_FRAMES]; 2];
        self.first_frame_after_reset = [true; 2];
        self.prev_decode_only_middle = false;
        self.prev_packet_channels = 1;
    }

    /// Return SILK range coder final state from last decoded frame.
    ///
    /// Params: none.
    /// Returns: `final_range` value for conformance checks.
    pub fn final_range(&self) -> u32 {
        self.final_range
    }

    /// Return coded channel count seen in the previous SILK packet.
    ///
    /// Params: none.
    /// Returns: previous packet coded channel count used for stereo transitions.
    pub fn prev_packet_channels(&self) -> u8 {
        self.prev_packet_channels
    }

    /// Decode one SILK Opus frame payload.
    ///
    /// Params: `frame` payload bytes, `frame_samples_48k` frame duration in 48 kHz samples,
    /// `config` TOC config, `packet_channels` coded channels, `_packet_idx` debug packet index,
    /// and `out` interleaved output slice.
    /// Returns: decoded sample count per output channel at the API output rate.
    pub fn decode_frame(
        &mut self,
        frame: &[u8],
        frame_samples_48k: usize,
        config: u8,
        packet_channels: u8,
        packet_idx: usize,
        out: &mut [i16],
    ) -> Result<SilkFrameDecode, Error> {
        let mut dec = EcDec::new(frame);
        self.decode_frame_with_ec(
            frame,
            &mut dec,
            frame_samples_48k,
            config,
            packet_channels,
            false,
            packet_idx,
            out,
        )
    }

    /// Decode one SILK Opus frame payload using a shared entropy decoder.
    ///
    /// Params: original `frame`, shared mutable `dec`, `frame_samples_48k`, TOC `config`,
    /// coded `packet_channels`, `is_hybrid` mode flag, debug `packet_idx`, and interleaved `out`.
    /// Returns: decoded sample count per output channel at the API output rate.
    pub(crate) fn decode_frame_with_ec(
        &mut self,
        frame: &[u8],
        mut dec: &mut EcDec<'_>,
        frame_samples_48k: usize,
        config: u8,
        packet_channels: u8,
        is_hybrid: bool,
        packet_idx: usize,
        out: &mut [i16],
    ) -> Result<SilkFrameDecode, Error> {
        if !matches!(packet_channels, 1 | 2) {
            return Err(Error::BadPacket);
        }

        let internal_fs_hz = silk_internal_fs_hz(config)?;
        let (internal_frames, nb_subfr, frame_length) =
            internal_frame_shape(frame_samples_48k, internal_fs_hz)?;
        if packet_idx == 0 && (12..=15).contains(&config) {
            // #region agent log H65
            eprintln!(
                "SILK_HYB entry: internal_fs={} nb_subfr={} internal_frames={} frame_length={}",
                internal_fs_hz / 1000,
                nb_subfr,
                internal_frames,
                frame_length
            );
            append_silk_debug_log(
                "silk-hybrid-entry",
                "H65",
                &format!(
                    "{{\"packet_idx\":{},\"config\":{},\"internal_fs_khz\":{},\"nb_subfr\":{},\"internal_frames\":{},\"frame_length\":{},\"tell\":{},\"tell_frac\":{}}}",
                    packet_idx,
                    config,
                    internal_fs_hz / 1000,
                    nb_subfr,
                    internal_frames,
                    frame_length,
                    dec.tell(),
                    dec.tell_frac()
                ),
            );
            // #endregion
        }
        let samples_per_channel = (frame_samples_48k * self.fs_hz as usize) / 48_000;
        let needed = samples_per_channel * self.channels as usize;
        if out.len() < needed {
            return Err(Error::OutputTooSmall {
                needed,
                got: out.len(),
            });
        }

        let mut internal_pcm = [[0i16; MAX_INTERNAL_FRAMES * tables::MAX_FRAME_LENGTH]; 2];
        let mut resampled_pcm = [[0i16; MAX_API_FRAME_SAMPLES_48K]; 2];
        let trace_pkt0 = packet_idx == 0 && silk_trace_enabled();
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "init");
        }
        for ch in 0..packet_channels as usize {
            let prev_fs_khz = self.lbrr_state[ch].fs_khz;
            let new_fs_khz = internal_fs_hz / 1000;
            if prev_fs_khz != 0 && prev_fs_khz != new_fs_khz {
                self.core_state[ch] = decode_core::SilkChannelState::default();
                self.first_frame_after_reset[ch] = true;
                if (1116..=1120).contains(&packet_idx) {
                    // #region agent log H112
                    eprintln!(
                        "TV12 pkt{} silk_fs_reset ch={} prev_fs={} new_fs={}",
                        packet_idx, ch, prev_fs_khz, new_fs_khz
                    );
                    append_silk_debug_log(
                        "tv12-silk-fs-reset",
                        "H112",
                        &format!(
                            "{{\"packet_idx\":{},\"channel\":{},\"prev_fs_khz\":{},\"new_fs_khz\":{}}}",
                            packet_idx, ch, prev_fs_khz, new_fs_khz
                        ),
                    );
                    // #endregion
                }
            }
            lbrr::configure_channel_state(&mut self.lbrr_state[ch], new_fs_khz, nb_subfr);
        }
        if self.fs_hz == 48_000 {
            for ch in 0..packet_channels as usize {
                let needs_reset = self.resampler_state[ch]
                    .as_ref()
                    .is_none_or(|state| !state.matches(internal_fs_hz, self.fs_hz));
                if needs_reset {
                    self.resampler_state[ch] =
                        Some(resampler::SilkResampler::new(internal_fs_hz, self.fs_hz)?);
                    self.resampler_history[ch] = [0; 2];
                }
            }
            if self.channels == 2 && packet_channels == 2 && self.prev_packet_channels == 1 {
                self.stereo_state.pred_prev_q13 = [0; 2];
                self.stereo_state.s_side = [0; 2];
                if let Some(left_resampler) = self.resampler_state[0].clone() {
                    self.resampler_state[1] = Some(left_resampler);
                }
                if matches!(packet_idx, 0 | 4 | 99 | 602) {
                    // #region agent log H39
                    eprintln!(
                        "DBG H39 rust pkt{} mono_to_stereo copied_right_resampler",
                        packet_idx
                    );
                    // #endregion
                }
            }
        }
        let parsed = match parse_header(
            &mut dec,
            packet_channels as usize,
            internal_frames,
            packet_idx,
            trace_pkt0,
        ) {
            Ok(parsed) => parsed,
            Err(err) => {
                if packet_idx == 0 && (12..=15).contains(&config) {
                    // #region agent log H66
                    append_silk_debug_log(
                        "silk-hybrid-parse-header-error",
                        "H66",
                        &format!(
                            "{{\"packet_idx\":{},\"error\":\"{}\",\"tell\":{},\"tell_frac\":{}}}",
                            packet_idx,
                            err,
                            dec.tell(),
                            dec.tell_frac()
                        ),
                    );
                    // #endregion
                }
                return Err(err);
            }
        };
        if packet_idx == 1118 {
            let ch = &self.core_state[0];
            let tail_start = ch.out_buf.len().saturating_sub(8);
            // #region agent log H110
            eprintln!(
                "TV12 pkt1118 silk_entry: out_buf_last8={:?} sLPC_head4={:?} lag_prev={} prev_gain_index={}",
                &ch.out_buf[tail_start..],
                &ch.s_lpc_q14_buf[..4],
                ch.lag_prev,
                ch.last_gain_index
            );
            append_silk_debug_log(
                "tv12-pkt1118-silk-entry",
                "H110",
                &format!(
                    "{{\"packet_idx\":1118,\"out_buf_tail\":{:?},\"s_lpc_head4\":{:?},\"lag_prev\":{},\"prev_gain_index\":{}}}",
                    &ch.out_buf[tail_start..],
                    &ch.s_lpc_q14_buf[..4],
                    ch.lag_prev,
                    ch.last_gain_index
                ),
            );
            // #endregion
        }
        if let Err(err) = consume_lbrr_payload(
            &mut dec,
            &parsed,
            packet_channels as usize,
            internal_frames,
            &mut self.lbrr_state,
            frame_length,
            packet_idx,
            trace_pkt0,
        ) {
            if packet_idx == 0 && (12..=15).contains(&config) {
                // #region agent log H67
                append_silk_debug_log(
                    "silk-hybrid-lbrr-error",
                    "H67",
                    &format!(
                        "{{\"packet_idx\":{},\"error\":\"{}\",\"tell\":{},\"tell_frac\":{}}}",
                        packet_idx,
                        err,
                        dec.tell(),
                        dec.tell_frac()
                    ),
                );
                // #endregion
            }
            return Err(err);
        }
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "after_lbrr_consume");
        }
        if let Err(err) = consume_main_payload(
            &mut dec,
            &parsed,
            packet_channels as usize,
            internal_frames,
            &mut self.lbrr_state,
            frame_length,
            &mut self.prev_nlsf_q15,
            &mut self.pred_coef_q12,
            &mut self.first_frame_after_reset,
            &mut self.core_state,
            &mut internal_pcm,
            &mut self.stereo_state,
            &mut self.prev_decode_only_middle,
            packet_idx,
            trace_pkt0,
        ) {
            if packet_idx == 0 && (12..=15).contains(&config) {
                // #region agent log H68
                append_silk_debug_log(
                    "silk-hybrid-main-payload-error",
                    "H68",
                    &format!(
                        "{{\"packet_idx\":{},\"error\":\"{}\",\"tell\":{},\"tell_frac\":{}}}",
                        packet_idx,
                        err,
                        dec.tell(),
                        dec.tell_frac()
                    ),
                );
                // #endregion
            }
            return Err(err);
        }
        let mut consumed_redundancy = false;
        let mut redundancy_bytes = 0usize;
        let mut celt_to_silk = false;
        if packet_idx == 4 {
            let tell = dec.tell();
            let tell_frac = dec.tell_frac();
            let redundancy_gate = tell + 17 <= (8 * frame.len() as i32);
            let implied_redundancy_bytes = (frame.len() as i32 - ((tell + 7) >> 3)).max(0) as usize;
            // #region agent log H48
            eprintln!(
                "DBG H48 rust pkt4 post_silk_ec tell={} tell_frac={} frame_bytes={} redundancy_gate={} implied_redundancy_bytes={}",
                tell,
                tell_frac,
                frame.len(),
                redundancy_gate,
                implied_redundancy_bytes
            );
            // #endregion
        }
        if !is_hybrid && dec.tell() + 17 <= (8 * frame.len() as i32) {
            celt_to_silk = dec.dec_bit_logp(1);
            redundancy_bytes = frame
                .len()
                .saturating_sub(((dec.tell() + 7).max(0) as usize) >> 3);
            consumed_redundancy = redundancy_bytes > 0;
            if packet_idx == 4 {
                // #region agent log H49
                eprintln!(
                    "DBG H49 rust pkt4 redundancy consumed={} celt_to_silk={} redundancy_bytes={} tell={} tell_frac={}",
                    consumed_redundancy,
                    celt_to_silk,
                    redundancy_bytes,
                    dec.tell(),
                    dec.tell_frac()
                );
                // #endregion
            }
        }
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "after_main_decode");
        }
        if packet_idx == 1117 {
            let ch = &self.core_state[0];
            let tail_start = ch.out_buf.len().saturating_sub(8);
            // #region agent log H111
            eprintln!(
                "TV12 post_pkt1117 out_buf_last8={:?} sLPC_head4={:?} lag_prev={} prev_gain_index={}",
                &ch.out_buf[tail_start..],
                &ch.s_lpc_q14_buf[..4],
                ch.lag_prev,
                ch.last_gain_index
            );
            append_silk_debug_log(
                "tv12-post-pkt1117-silk-state",
                "H111",
                &format!(
                    "{{\"packet_idx\":1117,\"out_buf_tail\":{:?},\"s_lpc_head4\":{:?},\"lag_prev\":{},\"prev_gain_index\":{}}}",
                    &ch.out_buf[tail_start..],
                    &ch.s_lpc_q14_buf[..4],
                    ch.lag_prev,
                    ch.last_gain_index
                ),
            );
            // #endregion
        }
        out[..needed].fill(0);
        let internal_samples_per_channel = internal_frames * frame_length;
        if std::env::var("OPUS_TRACE_INTERNAL_LEN").ok().as_deref() == Some("1") {
            eprintln!(
                "R pkt{} internal_samples={} internal_frames={} frame_length={}",
                packet_idx, internal_samples_per_channel, internal_frames, frame_length
            );
        }
        if std::env::var("OPUS_DUMP_INTERNAL_PCM").ok().as_deref() == Some("1") {
            let dump_path = "/tmp/rust_internal.pcm";
            if packet_idx == 0 {
                let _ = std::fs::remove_file(dump_path);
            }
            if let Ok(mut file) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(dump_path)
            {
                for sample in &internal_pcm[0][..internal_samples_per_channel] {
                    let _ = std::io::Write::write_all(&mut file, &sample.to_le_bytes());
                }
            }
        }
        if self.fs_hz == 48_000 {
            for ch in 0..packet_channels as usize {
                if let Some(resampler) = self.resampler_state[ch].as_mut() {
                    let mut written_total = 0usize;
                    for iframe_idx in 0..internal_frames {
                        let pcm_range = iframe_idx * frame_length..(iframe_idx + 1) * frame_length;
                        let frame_pcm = &internal_pcm[ch][pcm_range];
                        let expected = resampler.output_len(frame_length);
                        if packet_idx == 4 {
                            let (s_iir, s_fir, delay_buf) = resampler.debug_snapshot();
                            // #region agent log H46
                            eprintln!(
                                "DBG H46 rust pkt4 ch{} iframe{} resampler_state s_iir={:?} s_fir={:?} delay={:?}",
                                ch, iframe_idx, s_iir, s_fir, delay_buf
                            );
                            // #endregion
                        }
                        let written = if packet_channels == 2 {
                            resampler.process(
                                frame_pcm,
                                &mut resampled_pcm[ch][written_total..written_total + expected],
                            )
                        } else {
                            let mut resamp_input = vec![0i16; frame_length + 2];
                            resamp_input[..2].copy_from_slice(&self.resampler_history[ch]);
                            resamp_input[2..2 + frame_length].copy_from_slice(frame_pcm);
                            let written = resampler.process(
                                &resamp_input[1..1 + frame_length],
                                &mut resampled_pcm[ch][written_total..written_total + expected],
                            );
                            self.resampler_history[ch][0] = resamp_input[frame_length];
                            self.resampler_history[ch][1] = resamp_input[frame_length + 1];
                            written
                        };
                        debug_assert_eq!(written, expected);
                        written_total += written;
                    }
                    debug_assert_eq!(written_total, samples_per_channel);
                    if matches!(packet_idx, 0 | 4 | 99 | 602) {
                        // #region agent log H35
                        eprintln!(
                            "DBG H35 rust pkt{} ch{} resampled[0..8]={:?}",
                            packet_idx,
                            ch,
                            &resampled_pcm[ch][..8.min(samples_per_channel)]
                        );
                        if packet_idx == 0 && samples_per_channel >= 280 {
                            eprintln!(
                                "DBG H40 rust pkt0 ch{} resampled[268..276]={:?}",
                                ch,
                                &resampled_pcm[ch][268..276]
                            );
                        }
                        if packet_idx == 4 && samples_per_channel >= 860 {
                            eprintln!(
                                "DBG H44 rust pkt4 ch{} resampled[852..860]={:?}",
                                ch,
                                &resampled_pcm[ch][852..860]
                            );
                            eprintln!(
                                "DBG H52 rust pkt4 ch{} resampled_last8={:?}",
                                ch,
                                &resampled_pcm[ch][samples_per_channel - 8..samples_per_channel]
                            );
                        }
                        if matches!(packet_idx, 99) && samples_per_channel >= 8 {
                            eprintln!(
                                "DBG H58 rust pkt{} ch{} resampled_last8={:?}",
                                packet_idx,
                                ch,
                                &resampled_pcm[ch][samples_per_channel - 8..samples_per_channel]
                            );
                        }
                        // #endregion
                    }
                }
            }
        }
        if std::env::var("OPUS_DUMP_RESAMPLED").is_ok() && samples_per_channel > 0 {
            append_pcm_dump(
                "/tmp/rust_tv02_resampled.raw",
                &resampled_pcm[0][..samples_per_channel],
            );
        }
        if self.fs_hz == 48_000 {
            if matches!(packet_idx, 0 | 4 | 602) {
                // #region agent log H36
                eprintln!(
                    "DBG H36 rust pkt{} pre_interleave left[0..8]={:?} right[0..8]={:?} packet_channels={} out_channels={}",
                    packet_idx,
                    &resampled_pcm[0][..8.min(samples_per_channel)],
                    &resampled_pcm[1][..8.min(samples_per_channel)],
                    packet_channels,
                    self.channels
                );
                // #endregion
            }
            for n in 0..samples_per_channel {
                let left = resampled_pcm[0][n];
                match self.channels {
                    1 => out[n] = left,
                    2 => {
                        out[2 * n] = left;
                        out[2 * n + 1] = if packet_channels == 2 {
                            resampled_pcm[1][n]
                        } else {
                            left
                        };
                    }
                    _ => unreachable!(),
                }
            }
            if packet_idx == 0 && self.channels == 2 {
                eprintln!(
                    "DBG H53 rust pkt0 final_output[0..8]={:?} L[0..4]={:?} R[0..4]={:?}",
                    &out[..8.min(samples_per_channel * self.channels as usize)],
                    &resampled_pcm[0][..4.min(samples_per_channel)],
                    &resampled_pcm[1][..4.min(samples_per_channel)]
                );
                if samples_per_channel >= 272 {
                    eprintln!(
                        "DBG H54 rust pkt0 final_output[536..544]={:?} L[268..272]={:?} R[268..272]={:?}",
                        &out[536..544],
                        &resampled_pcm[0][268..272],
                        &resampled_pcm[1][268..272]
                    );
                }
            }
            if packet_idx == 4 && self.channels == 2 {
                eprintln!(
                    "DBG H55 rust pkt4 final_output[0..8]={:?} L[0..4]={:?} R[0..4]={:?}",
                    &out[..8.min(samples_per_channel * self.channels as usize)],
                    &resampled_pcm[0][..4.min(samples_per_channel)],
                    &resampled_pcm[1][..4.min(samples_per_channel)]
                );
            }
        } else {
            for n in 0..internal_samples_per_channel.min(samples_per_channel) {
                let left = internal_pcm[0][n];
                match self.channels {
                    1 => out[n] = left,
                    2 => {
                        out[2 * n] = left;
                        out[2 * n + 1] = if packet_channels == 2 {
                            internal_pcm[1][n]
                        } else {
                            left
                        };
                    }
                    _ => unreachable!(),
                }
            }
        }

        self.prev_packet_channels = packet_channels;
        self.final_range = dec.final_range();
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "after_decode");
        }
        if dec.is_error() {
            return Err(Error::BadPacket);
        }

        if lbrr::any_lbrr(
            &[parsed.channels[0].lbrr_flags, parsed.channels[1].lbrr_flags],
            packet_channels as usize,
        ) {
            debug_trace!("silk: consumed LBRR side payload");
        }

        Ok(SilkFrameDecode {
            samples_per_channel,
            consumed_redundancy,
            redundancy_bytes,
            celt_to_silk,
        })
    }
}

/// Parse SILK packet header signaling used before core frame decode.
///
/// Params: `dec` shared entropy decoder, `packet_channels` coded channels, and
/// `internal_frames` count of 20 ms SILK-internal frames (1/2/3).
/// Returns: parsed VAD/LBRR signaling snapshot for the frame.
fn parse_header(
    dec: &mut EcDec<'_>,
    packet_channels: usize,
    internal_frames: usize,
    packet_idx: usize,
    trace_pkt0: bool,
) -> Result<ParsedHeader, Error> {
    let mut parsed = ParsedHeader::default();

    for ch in 0..packet_channels {
        if packet_idx == 0 {
            // #region agent log H69
            eprintln!(
                "SILK_HDR before_vad ch{} tell={} tell_frac={}",
                ch,
                dec.tell(),
                dec.tell_frac()
            );
            append_silk_debug_log(
                "silk-header-before-vad",
                "H69",
                &format!(
                    "{{\"packet_idx\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"internal_frames\":{}}}",
                    packet_idx,
                    ch,
                    dec.tell(),
                    dec.tell_frac(),
                    internal_frames
                ),
            );
            // #endregion
        }
        for i in 0..internal_frames {
            parsed.channels[ch].vad_flags[i] = dec.dec_bit_logp(1);
        }
        if packet_idx == 0 {
            // #region agent log H70
            eprintln!(
                "SILK_HDR after_vad ch{} tell={} tell_frac={} vad_flags={:?}",
                ch,
                dec.tell(),
                dec.tell_frac(),
                &parsed.channels[ch].vad_flags[..internal_frames]
            );
            append_silk_debug_log(
                "silk-header-after-vad",
                "H70",
                &format!(
                    "{{\"packet_idx\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"vad_flags\":\"{:?}\"}}",
                    packet_idx,
                    ch,
                    dec.tell(),
                    dec.tell_frac(),
                    &parsed.channels[ch].vad_flags[..internal_frames]
                ),
            );
            // #endregion
        }
        if trace_pkt0 {
            trace_checkpoint(dec, packet_idx, &format!("after_vad_flags_ch{}", ch));
        }

        let has_lbrr = dec.dec_bit_logp(1);
        parsed.channels[ch].lbrr_flags = unpack_lbrr_flags(dec, has_lbrr, internal_frames)?;
        if packet_idx == 0 {
            // #region agent log H71
            eprintln!(
                "SILK_HDR after_lbrr_flag ch{} tell={} tell_frac={} has_lbrr={} lbrr_flags={:?}",
                ch,
                dec.tell(),
                dec.tell_frac(),
                has_lbrr,
                &parsed.channels[ch].lbrr_flags[..internal_frames]
            );
            append_silk_debug_log(
                "silk-header-after-lbrr",
                "H71",
                &format!(
                    "{{\"packet_idx\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"has_lbrr\":{},\"lbrr_flags\":\"{:?}\"}}",
                    packet_idx,
                    ch,
                    dec.tell(),
                    dec.tell_frac(),
                    has_lbrr,
                    &parsed.channels[ch].lbrr_flags[..internal_frames]
                ),
            );
            // #endregion
        }
        if trace_pkt0 {
            trace_checkpoint(dec, packet_idx, &format!("after_lbrr_flag_ch{}", ch));
        }
    }

    Ok(parsed)
}

/// Consume LBRR payload side information for all flagged internal frames.
///
/// Params: entropy decoder `dec`, parsed `header`, coded channel count,
/// `internal_frames`, mutable per-channel `states`, and internal `frame_length`.
/// Returns: nothing on success, `BadPacket` on malformed entropy payload.
fn consume_lbrr_payload(
    dec: &mut EcDec<'_>,
    header: &ParsedHeader,
    packet_channels: usize,
    internal_frames: usize,
    states: &mut [lbrr::ChannelState; 2],
    frame_length: usize,
    packet_idx: usize,
    trace_pkt0: bool,
) -> Result<(), Error> {
    for i in 0..internal_frames {
        for (ch, state) in states.iter_mut().enumerate().take(packet_channels) {
            if !header.channels[ch].lbrr_flags[i] {
                continue;
            }
            if packet_channels == 2 && ch == 0 {
                let _ = stereo::decode_stereo_pred(dec);
                if !header.channels[1].lbrr_flags[i] {
                    let _ = stereo::decode_mid_only(dec);
                }
            }
            let cond = if i > 0 && header.channels[ch].lbrr_flags[i - 1] {
                lbrr::CondCoding::Conditionally
            } else {
                lbrr::CondCoding::Independently
            };
            let side =
                lbrr::decode_indices(state, dec, header.channels[ch].vad_flags[i], true, cond)?;
            let _ =
                lbrr::decode_pulses(dec, side.signal_type, side.quant_offset_type, frame_length)?;
            if trace_pkt0 {
                trace_checkpoint(dec, packet_idx, &format!("after_lbrr_i{}_ch{}", i, ch));
            }
        }
    }
    Ok(())
}

/// Consume main SILK frame side information and pulses for all internal frames.
///
/// Params: entropy decoder `dec`, parsed packet `header`, coded channel count,
/// `internal_frames`, mutable per-channel `states`, internal `frame_length`,
/// packet-level `prev_decode_only_middle`, `packet_idx`, and tracing flag.
/// Returns: nothing on success, `BadPacket` on malformed entropy payload.
fn consume_main_payload(
    dec: &mut EcDec<'_>,
    header: &ParsedHeader,
    packet_channels: usize,
    internal_frames: usize,
    states: &mut [lbrr::ChannelState; 2],
    frame_length: usize,
    prev_nlsf_q15: &mut [[i16; tables::MAX_LPC_ORDER]; 2],
    pred_coef_q12: &mut [[[[i16; tables::MAX_LPC_ORDER]; MAX_LPC_HALVES]; MAX_INTERNAL_FRAMES]; 2],
    first_frame_after_reset: &mut [bool; 2],
    core_state: &mut [decode_core::SilkChannelState; 2],
    internal_pcm: &mut [[i16; MAX_INTERNAL_FRAMES * tables::MAX_FRAME_LENGTH]; 2],
    stereo_state: &mut stereo::StereoState,
    prev_decode_only_middle: &mut bool,
    packet_idx: usize,
    trace_pkt0: bool,
) -> Result<(), Error> {
    let mut signal_types = [[-1i32; MAX_INTERNAL_FRAMES]; 2];
    let mut mid_only_flags = [false; MAX_INTERNAL_FRAMES];
    for i in 0..internal_frames {
        let mut decode_only_middle = false;
        let mut pred_q13 = [0i32; 2];
        if packet_channels == 2 {
            pred_q13 = stereo::decode_stereo_pred(dec);
            decode_only_middle = !header.channels[1].vad_flags[i] && stereo::decode_mid_only(dec);
            mid_only_flags[i] = decode_only_middle;
            if matches!(packet_idx, 0 | 4 | 99 | 602) {
                // #region agent log H32
                eprintln!(
                    "DBG H32 rust pkt{} iframe{} stereo pred={:?} vad0={} vad1={} decode_only_middle={} prev_decode_only_middle={}",
                    packet_idx,
                    i,
                    pred_q13,
                    header.channels[0].vad_flags[i],
                    header.channels[1].vad_flags[i],
                    decode_only_middle,
                    *prev_decode_only_middle
                );
                // #endregion
            }
            if trace_pkt0 {
                trace_checkpoint(dec, packet_idx, &format!("after_stereo_i{}", i));
            }
            debug_trace!(
                "silk: stereo signaling i={} pred0={} pred1={} mid_only={}",
                i,
                pred_q13[0],
                pred_q13[1],
                decode_only_middle
            );
        }

        for ch in 0..packet_channels {
            if packet_idx == 0 && i == 0 && ch == 0 {
                // #region agent log H72
                eprintln!(
                    "SILK_MAIN [after_header] tell={} rng={:#010x}",
                    dec.tell(),
                    dec.rng()
                );
                append_silk_debug_log(
                    "silk-main-after-header",
                    "H72",
                    &format!(
                        "{{\"packet_idx\":{},\"iframe\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"rng\":{}}}",
                        packet_idx,
                        i,
                        ch,
                        dec.tell(),
                        dec.tell_frac(),
                        dec.rng()
                    ),
                );
                // #endregion
            }
            if ch == 1 && decode_only_middle {
                if matches!(packet_idx, 0 | 4 | 99 | 602) {
                    // #region agent log H33
                    eprintln!(
                        "DBG H33 rust pkt{} iframe{} skip_ch1_mid_only",
                        packet_idx, i
                    );
                    // #endregion
                }
                continue;
            }
            let cond = if i == 0 {
                if ch > 0 && *prev_decode_only_middle {
                    lbrr::CondCoding::IndependentlyNoLtpScaling
                } else {
                    lbrr::CondCoding::Independently
                }
            } else if ch > 0 && *prev_decode_only_middle {
                lbrr::CondCoding::IndependentlyNoLtpScaling
            } else {
                lbrr::CondCoding::Conditionally
            };
            let side = lbrr::decode_indices(
                &mut states[ch],
                dec,
                header.channels[ch].vad_flags[i],
                false,
                cond,
            )?;
            if packet_idx == 0 && i == 0 && ch == 0 {
                // #region agent log H73
                eprintln!(
                    "SILK_MAIN [after_indices] tell={} rng={:#010x}",
                    dec.tell(),
                    dec.rng()
                );
                append_silk_debug_log(
                    "silk-main-after-indices",
                    "H73",
                    &format!(
                        "{{\"packet_idx\":{},\"iframe\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"rng\":{},\"signal_type\":{},\"quant_offset_type\":{}}}",
                        packet_idx,
                        i,
                        ch,
                        dec.tell(),
                        dec.tell_frac(),
                        dec.rng(),
                        side.signal_type,
                        side.quant_offset_type
                    ),
                );
                // #endregion
            }
            update_lpc_from_nlsf(
                states[ch].fs_khz,
                &side.nlsf,
                &mut prev_nlsf_q15[ch],
                &mut pred_coef_q12[ch][i],
                &mut first_frame_after_reset[ch],
                packet_idx,
                i,
                ch,
            );
            if trace_pkt0 && packet_idx == 0 && i == 0 && ch == 0 {
                trace_lpc(
                    packet_idx,
                    ch,
                    i,
                    "pred0",
                    &pred_coef_q12[ch][i][0],
                    states[ch].fs_khz,
                );
                trace_lpc(
                    packet_idx,
                    ch,
                    i,
                    "pred1",
                    &pred_coef_q12[ch][i][1],
                    states[ch].fs_khz,
                );
            }
            let pulses =
                lbrr::decode_pulses(dec, side.signal_type, side.quant_offset_type, frame_length)?;
            if packet_idx == 0 && i == 0 && ch == 0 {
                // #region agent log H74
                eprintln!(
                    "SILK_MAIN [after_pulses] tell={} rng={:#010x}",
                    dec.tell(),
                    dec.rng()
                );
                append_silk_debug_log(
                    "silk-main-after-pulses",
                    "H74",
                    &format!(
                        "{{\"packet_idx\":{},\"iframe\":{},\"channel\":{},\"tell\":{},\"tell_frac\":{},\"rng\":{}}}",
                        packet_idx,
                        i,
                        ch,
                        dec.tell(),
                        dec.tell_frac(),
                        dec.rng()
                    ),
                );
                // #endregion
            }
            let frame_params = decode_core::SilkFrameParams {
                signal_type: side.signal_type,
                quant_offset_type: side.quant_offset_type,
                gain_indices: side.gain_indices,
                lag_index: side.lag_index,
                contour_index: side.contour_index,
                per_index: side.per_index,
                ltp_indices: side.ltp_indices,
                ltp_scale_index: side.ltp_scale_index,
                seed: side.seed,
                pulses,
                conditional: matches!(cond, lbrr::CondCoding::Conditionally),
                frame_length,
                nb_subfr: states[ch].nb_subfr,
                fs_khz: states[ch].fs_khz,
                lpc_order: tables::nlsf_codebook(states[ch].fs_khz).order,
            };
            signal_types[ch][i] = frame_params.signal_type as i32;
            if packet_idx == 223 && i == 0 && ch == 0 {
                // #region agent log H28
                eprintln!(
                    "DBG H28 rust pkt223 if0 interp_coef_q2={} pred0={:?} pred1={:?}",
                    side.nlsf.interp_coef_q2,
                    &pred_coef_q12[ch][i][0][..frame_params.lpc_order],
                    &pred_coef_q12[ch][i][1][..frame_params.lpc_order]
                );
                // #endregion
            }
            if matches!(packet_idx, 30 | 223) && i == 0 && ch == 0 {
                let state_len = frame_params.lpc_order.min(10);
                let out_tail_start = core_state[ch].out_buf.len().saturating_sub(8);
                // #region agent log H19
                eprintln!(
                    "DBG H19 rust pkt{} if0 entry prev_gain_q16={} lag_prev={} signal_type={} gain_indices={:?} lag_index={} contour_index={} slpc10={:?} out_buf_tail8={:?}",
                    packet_idx,
                    core_state[ch].prev_gain_q16,
                    core_state[ch].lag_prev,
                    frame_params.signal_type,
                    frame_params.gain_indices,
                    frame_params.lag_index,
                    frame_params.contour_index,
                    &core_state[ch].s_lpc_q14_buf[..state_len],
                    &core_state[ch].out_buf[out_tail_start..]
                );
                // #endregion
            }
            let pcm_range = i * frame_length..(i + 1) * frame_length;
            let trace = decode_core::decode_core(
                &frame_params,
                &pred_coef_q12[ch][i],
                &mut internal_pcm[ch][pcm_range],
                &mut core_state[ch],
                packet_idx,
                i,
            );
            if trace_pkt0 && packet_idx == 0 && i == 0 && ch == 0 {
                trace_core(packet_idx, "exc_Q14", &trace.exc_q14_head);
                trace_core(packet_idx, "sLPC_Q14", &trace.s_lpc_q14_head);
            }
            if matches!(packet_idx, 0 | 4 | 99 | 602) {
                let iframe_start = i * frame_length;
                let iframe_end = (iframe_start + 8).min(iframe_start + frame_length);
                // #region agent log H34
                eprintln!(
                    "DBG H34 rust pkt{} ch{} iframe{} internal_pcm[0..8]={:?}",
                    packet_idx,
                    ch,
                    i,
                    &internal_pcm[ch][iframe_start..iframe_end]
                );
                if packet_idx == 4 && frame_length >= 148 {
                    eprintln!(
                        "DBG H45 rust pkt4 ch{} iframe{} internal_pcm[140..148]={:?}",
                        ch,
                        i,
                        &internal_pcm[ch][iframe_start + 140..iframe_start + 148]
                    );
                }
                // #endregion
            }
            if silk_trace_enabled() && ch == 0 {
                let iframe_start = i * frame_length;
                let iframe_end = (iframe_start + 8).min(iframe_start + frame_length);
                if packet_idx == 1 {
                    let iframe_pcm = &internal_pcm[ch][i * frame_length..(i + 1) * frame_length];
                    let max_abs = iframe_pcm.iter().map(|x| x.abs()).max().unwrap_or(0);
                    eprintln!(
                        "R pkt1 iframe{} signal_type={} max_abs_pcm={} pcm[0..8]={:?}",
                        i,
                        frame_params.signal_type,
                        max_abs,
                        &internal_pcm[ch][iframe_start..iframe_end]
                    );
                }
                if matches!(packet_idx, 29 | 30 | 223) && i == 0 {
                    let state_len = frame_params.lpc_order.min(10);
                    // #region agent log H20
                    eprintln!(
                        "DBG H20 rust pkt{} if0 state[0..{}]: {:?}",
                        packet_idx,
                        state_len,
                        &core_state[ch].s_lpc_q14_buf[..state_len]
                    );
                    eprintln!(
                        "DBG H21 rust pkt{} if0 internal_pcm[0..8]: {:?}",
                        packet_idx,
                        &internal_pcm[ch][iframe_start..iframe_end]
                    );
                    // #endregion
                }
            }
            if trace_pkt0 {
                trace_checkpoint(dec, packet_idx, &format!("after_main_i{}_ch{}", i, ch));
            }
        }

        if packet_channels == 2 {
            let pcm_range = i * frame_length..(i + 1) * frame_length;
            let (left_channels, right_channels) = internal_pcm.split_at_mut(1);
            stereo::ms_to_lr(
                stereo_state,
                &mut left_channels[0][pcm_range.clone()],
                &mut right_channels[0][pcm_range.clone()],
                pred_q13,
                states[0].fs_khz,
                packet_idx,
            );
            if matches!(packet_idx, 0 | 4 | 99 | 602) {
                let iframe_start = i * frame_length;
                let iframe_end = (iframe_start + 8).min(iframe_start + frame_length);
                // #region agent log H37
                eprintln!(
                    "DBG H37 rust pkt{} post_ms_to_lr ch0[0..8]={:?} ch1[0..8]={:?}",
                    packet_idx,
                    &internal_pcm[0][iframe_start..iframe_end],
                    &internal_pcm[1][iframe_start..iframe_end]
                );
                if packet_idx == 0 && frame_length >= 52 {
                    eprintln!(
                        "DBG H41 rust pkt0 post_ms_to_lr ch0[44..52]={:?} ch1[44..52]={:?}",
                        &internal_pcm[0][iframe_start + 44..iframe_start + 52],
                        &internal_pcm[1][iframe_start + 44..iframe_start + 52]
                    );
                }
                if packet_idx == 4 && frame_length >= 148 {
                    eprintln!(
                        "DBG H43 rust pkt4 post_ms_to_lr ch0[140..148]={:?} ch1[140..148]={:?}",
                        &internal_pcm[0][iframe_start + 140..iframe_start + 148],
                        &internal_pcm[1][iframe_start + 140..iframe_start + 148]
                    );
                }
                // #endregion
            }
        } else {
            let pcm_range = i * frame_length..(i + 1) * frame_length;
            let frame = &internal_pcm[0][pcm_range.clone()];
            stereo_state
                .s_mid
                .copy_from_slice(&frame[frame_length - 2..frame_length]);
            if packet_idx == 601 {
                // #region agent log H38
                eprintln!(
                    "DBG H38 rust pkt{} iframe{} buffer_smid={:?}",
                    packet_idx, i, stereo_state.s_mid
                );
                // #endregion
            }
        }

        *prev_decode_only_middle = decode_only_middle;
    }
    if matches!(packet_idx, 99) {
        eprintln!(
            "DBG H56 rust pkt{} signal_types_ch0={:?} signal_types_ch1={:?} mid_only={:?}",
            packet_idx,
            &signal_types[0][..internal_frames],
            &signal_types[1][..internal_frames],
            &mid_only_flags[..internal_frames]
        );
    }
    if packet_idx == 1118 {
        let internal_samples_per_channel = internal_frames * frame_length;
        let head_end = 8.min(internal_samples_per_channel);
        let tail_start = internal_samples_per_channel.saturating_sub(8);
        // #region agent log H109
        eprintln!(
            "TV12 pkt1118 internal_pcm[0..8]={:?}",
            &internal_pcm[0][..head_end]
        );
        eprintln!(
            "TV12 pkt1118 internal_pcm_last8={:?}",
            &internal_pcm[0][tail_start..internal_samples_per_channel]
        );
        eprintln!(
            "TV12 pkt1118 internal_fs={} signal_type={} packet_channels={} internal_frames={} frame_length={}",
            states[0].fs_khz,
            signal_types[0][0],
            packet_channels,
            internal_frames,
            frame_length
        );
        append_silk_debug_log(
            "tv12-pkt1118-internal-pcm",
            "H109A",
            &format!(
                "{{\"packet_idx\":1118,\"internal_fs_khz\":{},\"signal_type\":{},\"packet_channels\":{},\"internal_frames\":{},\"frame_length\":{},\"head\":{:?},\"tail\":{:?}}}",
                states[0].fs_khz,
                signal_types[0][0],
                packet_channels,
                internal_frames,
                frame_length,
                &internal_pcm[0][..head_end],
                &internal_pcm[0][tail_start..internal_samples_per_channel]
            ),
        );
        // #endregion
    }
    Ok(())
}

/// Check whether SILK entropy tracing is enabled.
///
/// Params: none.
/// Returns: true when `OPUS_SILK_TRACE=1` is set.
fn silk_trace_enabled() -> bool {
    std::env::var("OPUS_SILK_TRACE").ok().as_deref() == Some("1")
}

/// Append 16-bit PCM samples to a raw little-endian dump file.
///
/// Params: destination file `path` and signed PCM `samples`.
/// Returns: nothing; I/O errors are ignored for debugging dumps.
fn append_pcm_dump(path: &str, samples: &[i16]) {
    use std::io::Write;

    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
    {
        for &sample in samples {
            let _ = file.write_all(&sample.to_le_bytes());
        }
    }
}

/// Append one NDJSON debug line for SILK hybrid tracing.
///
/// Params: event `message`, `hypothesis_id`, and raw JSON `data`.
/// Returns: nothing; debug I/O failures are ignored.
fn append_silk_debug_log(message: &str, hypothesis_id: &str, data: &str) {
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
            "{{\"sessionId\":\"9a4e1c\",\"runId\":\"hybrid-silk-trace\",\"hypothesisId\":\"{}\",\"location\":\"crates/opus-decoder/src/silk/mod.rs\",\"message\":\"{}\",\"data\":{},\"timestamp\":{}}}\n",
            hypothesis_id, message, data, ts
        );
        let _ = file.write_all(line.as_bytes());
    }
}

/// Emit packet-0 SILK entropy checkpoint with tell/tell_frac/rng.
///
/// Params: immutable entropy decoder `dec`, `packet_idx`, and textual `label`.
/// Returns: nothing.
fn trace_checkpoint(dec: &EcDec<'_>, packet_idx: usize, label: &str) {
    eprintln!(
        "SILK pkt{} [{}] tell={} tell_frac={} rng={:#010x}",
        packet_idx,
        label,
        dec.tell(),
        dec.tell_frac(),
        dec.rng()
    );
}

/// Decode, optionally interpolate, and convert one NLSF vector to LPC halves.
///
/// Params: internal `fs_khz`, decoded `nlsf_indices`, mutable `prev_nlsf_q15`,
/// mutable `pred_coef_q12`, and per-channel `first_frame_after_reset`.
/// Returns: nothing; previous NLSF and both LPC halves are updated in place.
fn update_lpc_from_nlsf(
    fs_khz: u32,
    nlsf_indices: &lbrr::DecodedNlsfIndices,
    prev_nlsf_q15: &mut [i16; tables::MAX_LPC_ORDER],
    pred_coef_q12: &mut [[i16; tables::MAX_LPC_ORDER]; MAX_LPC_HALVES],
    first_frame_after_reset: &mut bool,
    packet_idx: usize,
    frame_idx: usize,
    channel_idx: usize,
) {
    let codebook = tables::nlsf_codebook(fs_khz);
    let order = codebook.order;
    let mut curr_nlsf_q15 = [0i16; tables::MAX_LPC_ORDER];
    nlsf::nlsf_decode(
        &mut curr_nlsf_q15[..order],
        &nlsf_indices.values[..order + 1],
        codebook,
    );
    lpc::nlsf2a(
        &mut pred_coef_q12[1][..order],
        &curr_nlsf_q15[..order],
        order,
    );

    let interp_coef_q2 = if *first_frame_after_reset {
        4
    } else {
        nlsf_indices.interp_coef_q2
    };
    if interp_coef_q2 < 4 {
        let mut interp_nlsf_q15 = [0i16; tables::MAX_LPC_ORDER];
        for i in 0..order {
            let delta = curr_nlsf_q15[i] as i32 - prev_nlsf_q15[i] as i32;
            interp_nlsf_q15[i] =
                (prev_nlsf_q15[i] as i32 + ((interp_coef_q2 as i32 * delta) >> 2)) as i16;
        }
        lpc::nlsf2a(
            &mut pred_coef_q12[0][..order],
            &interp_nlsf_q15[..order],
            order,
        );
    } else {
        let pred1 = pred_coef_q12[1];
        pred_coef_q12[0][..order].copy_from_slice(&pred1[..order]);
    }

    if packet_idx == 223 && frame_idx == 0 && channel_idx == 0 {
        // #region agent log H29
        eprintln!(
            "DBG H29 rust pkt223 if0 prev_nlsf={:?} curr_nlsf={:?}",
            &prev_nlsf_q15[..order],
            &curr_nlsf_q15[..order]
        );
        // #endregion
    }
    prev_nlsf_q15[..order].copy_from_slice(&curr_nlsf_q15[..order]);
    *first_frame_after_reset = false;
}

/// Emit packet-0 LPC trace for one channel/frame half.
///
/// Params: `packet_idx`, coded `channel`, internal frame `frame_idx`, `label`,
/// LPC slice `lpc_q12`, and internal `fs_khz`.
/// Returns: nothing.
fn trace_lpc(
    packet_idx: usize,
    channel: usize,
    frame_idx: usize,
    label: &str,
    lpc_q12: &[i16],
    fs_khz: u32,
) {
    let order = tables::nlsf_codebook(fs_khz).order;
    let coeffs = lpc_q12[..order]
        .iter()
        .map(i16::to_string)
        .collect::<Vec<_>>()
        .join(" ");
    eprintln!(
        "SILK pkt{} LPC ch{} i{} {}_q12: {}",
        packet_idx, channel, frame_idx, label, coeffs
    );
}

/// Emit packet-0 SILK core trace for excitation or synthesis state.
///
/// Params: `packet_idx`, textual `label`, and signed sample `values`.
/// Returns: nothing.
fn trace_core(packet_idx: usize, label: &str, values: &[i32; 8]) {
    let rendered = values
        .iter()
        .map(i32::to_string)
        .collect::<Vec<_>>()
        .join(" ");
    eprintln!("SILK pkt{} {}[0..8]: {}", packet_idx, label, rendered);
}

/// Decode and expand LBRR per-internal-frame flags for one coded channel.
///
/// Params: `dec` entropy decoder, `has_lbrr` packet-level LBRR present bit, and
/// `internal_frames` number of SILK-internal frames.
/// Returns: fixed-size LBRR flag array with active entries in `[0..internal_frames)`.
fn unpack_lbrr_flags(
    dec: &mut EcDec<'_>,
    has_lbrr: bool,
    internal_frames: usize,
) -> Result<[bool; MAX_INTERNAL_FRAMES], Error> {
    if !has_lbrr {
        return Ok([false; MAX_INTERNAL_FRAMES]);
    }

    if internal_frames == 1 {
        return Ok([true, false, false]);
    }

    let icdf = tables::lbrr_flags_icdf(internal_frames).ok_or(Error::BadPacket)?;
    let symbol = dec.dec_icdf(icdf, 8) as u32 + 1;
    Ok(unpack_lbrr_symbol(symbol, internal_frames))
}

/// Expand packed LBRR symbol bits to boolean frame flags.
///
/// Params: `symbol` packed bit mask and `internal_frames` active frame count.
/// Returns: fixed-size LBRR flag array with decoded bits in low indices.
fn unpack_lbrr_symbol(symbol: u32, internal_frames: usize) -> [bool; MAX_INTERNAL_FRAMES] {
    let mut out = [false; MAX_INTERNAL_FRAMES];
    for (i, flag) in out.iter_mut().take(internal_frames).enumerate() {
        *flag = ((symbol >> i) & 1) != 0;
    }
    out
}

/// Map Opus TOC config (SILK-only) to SILK internal decode sample rate.
///
/// Params: `config` is TOC config value `(toc >> 3) & 0x1f`.
/// Returns: internal SILK sample rate in Hz (8000/12000/16000).
fn silk_internal_fs_hz(config: u8) -> Result<u32, Error> {
    match config >> 2 {
        0 => Ok(8_000),
        1 => Ok(12_000),
        2 | 3 => Ok(16_000),
        _ => Err(Error::BadPacket),
    }
}

/// Derive SILK internal frame shape from Opus frame duration.
///
/// Params: `frame_samples_48k` Opus frame size at 48 kHz and `internal_fs_hz`.
/// Returns: tuple `(internal_frames, nb_subfr, frame_length_samples)`.
fn internal_frame_shape(
    frame_samples_48k: usize,
    internal_fs_hz: u32,
) -> Result<(usize, usize, usize), Error> {
    let fs_khz = (internal_fs_hz / 1000) as usize;
    match frame_samples_48k {
        480 => Ok((1, 2, 10 * fs_khz)),
        960 => Ok((1, 4, 20 * fs_khz)),
        1_920 => Ok((2, 4, 20 * fs_khz)),
        2_880 => Ok((3, 4, 20 * fs_khz)),
        _ => Err(Error::BadPacket),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Decoder;
    use std::fs;
    use std::io::Read;
    use std::path::{Path, PathBuf};

    const TV02_PKT0_LPC_Q12: [i16; 10] = [5041, -1246, 1017, -614, 431, -560, 131, -357, 284, -46];

    #[test]
    fn maps_silk_internal_sample_rates() {
        assert_eq!(silk_internal_fs_hz(0).unwrap(), 8_000);
        assert_eq!(silk_internal_fs_hz(3).unwrap(), 8_000);
        assert_eq!(silk_internal_fs_hz(4).unwrap(), 12_000);
        assert_eq!(silk_internal_fs_hz(7).unwrap(), 12_000);
        assert_eq!(silk_internal_fs_hz(8).unwrap(), 16_000);
        assert_eq!(silk_internal_fs_hz(11).unwrap(), 16_000);
        assert!(silk_internal_fs_hz(12).is_err());
    }

    #[test]
    fn maps_internal_frame_counts() {
        assert_eq!(internal_frame_shape(480, 8_000).unwrap(), (1, 2, 80));
        assert_eq!(internal_frame_shape(960, 8_000).unwrap(), (1, 4, 160));
        assert_eq!(internal_frame_shape(1_920, 12_000).unwrap(), (2, 4, 240));
        assert_eq!(internal_frame_shape(2_880, 16_000).unwrap(), (3, 4, 320));
        assert!(internal_frame_shape(2_400, 12_000).is_err());
    }

    #[test]
    fn unpacks_lbrr_bitfield() {
        assert_eq!(unpack_lbrr_symbol(0b001, 3), [true, false, false]);
        assert_eq!(unpack_lbrr_symbol(0b011, 3), [true, true, false]);
        assert_eq!(unpack_lbrr_symbol(0b101, 3), [true, false, true]);
    }

    #[test]
    fn phase3a_decode_returns_zeroed_audio() {
        let mut silk = SilkDecoder::new(48_000, 1);
        let mut out = vec![123i16; 960];
        let result = silk.decode_frame(&[0; 8], 960, 0, 1, 0, &mut out).unwrap();
        assert_eq!(result.samples_per_channel, 960);
        assert!(out.iter().all(|&s| s == 0));
    }

    #[test]
    fn testvector02_pkt0_nlsf_to_lpc_matches_cref() {
        let packet = read_first_opus_demo_packet(&vectors_dir().join("testvector02.bit"));
        let mut decoder = Decoder::new(48_000, 2).unwrap();
        let mut out = vec![0i16; decoder.max_frame_size_per_channel() * 2];
        let samples = decoder.decode(Some(&packet), &mut out).unwrap();
        assert_eq!(samples, 2880);
        assert_eq!(decoder.final_range(), 0x5037_3c71);
        assert_eq!(
            &decoder.silk.pred_coef_q12[0][0][0][..10],
            &TV02_PKT0_LPC_Q12
        );
        assert_eq!(
            &decoder.silk.pred_coef_q12[0][0][1][..10],
            &TV02_PKT0_LPC_Q12
        );
    }

    #[test]
    fn testvector02_final_range_matches_reference_for_all_packets() {
        let packets = read_opus_demo_stream(&vectors_dir().join("testvector02.bit"));
        let mut decoder = Decoder::new(48_000, 2).unwrap();
        let mut out = vec![0i16; decoder.max_frame_size_per_channel() * 2];
        for (packet_idx, packet) in packets.iter().enumerate() {
            let _samples = decoder.decode(Some(&packet.payload), &mut out).unwrap();
            assert_eq!(
                decoder.final_range(),
                packet.expected_final_range,
                "packet {packet_idx} final_range"
            );
        }
    }

    /// Resolve the local RFC vector directory used by decoder tests.
    ///
    /// Params: none.
    /// Returns: absolute path to `testdata/opus_testvectors`.
    fn vectors_dir() -> PathBuf {
        if let Ok(path) = std::env::var("OPUS_TESTVECTORS_DIR") {
            return PathBuf::from(path);
        }
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/opus_testvectors")
            .canonicalize()
            .unwrap()
    }

    /// Read the first packet payload from an `opus_demo` `.bit` stream.
    ///
    /// Params: absolute `path` to the vector stream file.
    /// Returns: first packet payload bytes.
    fn read_first_opus_demo_packet(path: &Path) -> Vec<u8> {
        let mut file = fs::File::open(path).unwrap();
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).unwrap();
        let packet_len = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as usize;
        bytes[8..8 + packet_len].to_vec()
    }

    /// Read an `opus_demo` `.bit` stream into packet payloads plus expected ranges.
    ///
    /// Params: absolute `path` to the `.bit` vector file.
    /// Returns: ordered packet list with payloads and reference `final_range`.
    fn read_opus_demo_stream(path: &Path) -> Vec<DemoPacket> {
        let mut file = fs::File::open(path).unwrap();
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).unwrap();
        let mut packets = Vec::new();
        let mut index = 0usize;
        while index < bytes.len() {
            let packet_len = u32::from_be_bytes([
                bytes[index],
                bytes[index + 1],
                bytes[index + 2],
                bytes[index + 3],
            ]) as usize;
            index += 4;
            let expected_final_range = u32::from_be_bytes([
                bytes[index],
                bytes[index + 1],
                bytes[index + 2],
                bytes[index + 3],
            ]);
            index += 4;
            let payload = bytes[index..index + packet_len].to_vec();
            index += packet_len;
            packets.push(DemoPacket {
                payload,
                expected_final_range,
            });
        }
        packets
    }

    /// One `opus_demo` packet plus its reference range coder state.
    #[derive(Debug, Clone)]
    struct DemoPacket {
        /// Encoded Opus payload bytes.
        payload: Vec<u8>,
        /// Reference decoder final range after decoding the packet.
        expected_final_range: u32,
    }
}
