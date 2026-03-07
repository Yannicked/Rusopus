//! SILK decoder scaffolding (Phase 3a).
//!
//! This module focuses on bitstream parsing and entropy consumption for SILK
//! packet headers (VAD/LBRR/stereo signaling), with zeroed synthesis output.

use crate::Error;
use crate::entropy::EcDec;

mod entropy_tables;
mod lbrr;
mod stereo;
mod tables;

const MAX_INTERNAL_FRAMES: usize = 3;

/// Result of decoding one SILK Opus frame.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SilkFrameDecode {
    /// Number of decoded samples per output channel.
    pub samples_per_channel: usize,
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
    prev_decode_only_middle: bool,
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
            prev_decode_only_middle: false,
        }
    }

    /// Reset SILK decoder persistent state.
    ///
    /// Params: none.
    /// Returns: nothing.
    pub fn reset(&mut self) {
        self.final_range = 0;
        self.lbrr_state = [lbrr::ChannelState::default(), lbrr::ChannelState::default()];
        self.prev_decode_only_middle = false;
    }

    /// Return SILK range coder final state from last decoded frame.
    ///
    /// Params: none.
    /// Returns: `final_range` value for conformance checks.
    pub fn final_range(&self) -> u32 {
        self.final_range
    }

    /// Decode one SILK Opus frame payload.
    ///
    /// Params: `frame` payload bytes, `frame_samples_48k` frame duration in 48 kHz samples,
    /// `config` TOC config, `packet_channels` coded channels, `_packet_idx` debug packet index,
    /// and `out` interleaved output slice.
    /// Returns: decoded sample count per output channel (audio is zero-filled in Phase 3a).
    pub fn decode_frame(
        &mut self,
        frame: &[u8],
        frame_samples_48k: usize,
        config: u8,
        packet_channels: u8,
        packet_idx: usize,
        out: &mut [i16],
    ) -> Result<SilkFrameDecode, Error> {
        if !matches!(packet_channels, 1 | 2) {
            return Err(Error::BadPacket);
        }

        let internal_fs_hz = silk_internal_fs_hz(config)?;
        let (internal_frames, nb_subfr, frame_length) =
            internal_frame_shape(frame_samples_48k, internal_fs_hz)?;
        let samples_per_channel = (frame_samples_48k * self.fs_hz as usize) / 48_000;
        let needed = samples_per_channel * self.channels as usize;
        if out.len() < needed {
            return Err(Error::OutputTooSmall {
                needed,
                got: out.len(),
            });
        }

        let mut dec = EcDec::new(frame);
        let trace_pkt0 = packet_idx == 0 && silk_trace_enabled();
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "init");
        }
        for state in self.lbrr_state.iter_mut().take(packet_channels as usize) {
            lbrr::configure_channel_state(state, internal_fs_hz / 1000, nb_subfr);
        }
        let parsed = parse_header(
            &mut dec,
            packet_channels as usize,
            internal_frames,
            packet_idx,
            trace_pkt0,
        )?;
        consume_lbrr_payload(
            &mut dec,
            &parsed,
            packet_channels as usize,
            internal_frames,
            &mut self.lbrr_state,
            frame_length,
            packet_idx,
            trace_pkt0,
        )?;
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "after_lbrr_consume");
        }
        consume_main_payload(
            &mut dec,
            &parsed,
            packet_channels as usize,
            internal_frames,
            &mut self.lbrr_state,
            frame_length,
            &mut self.prev_decode_only_middle,
            packet_idx,
            trace_pkt0,
        )?;
        if trace_pkt0 {
            trace_checkpoint(&dec, packet_idx, "after_main_decode");
        }

        // Phase 3a currently parses signaling only; synthesis is intentionally deferred.
        out[..needed].fill(0);

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
        for i in 0..internal_frames {
            parsed.channels[ch].vad_flags[i] = dec.dec_bit_logp(1);
        }
        if trace_pkt0 {
            trace_checkpoint(dec, packet_idx, &format!("after_vad_flags_ch{}", ch));
        }

        let has_lbrr = dec.dec_bit_logp(1);
        parsed.channels[ch].lbrr_flags = unpack_lbrr_flags(dec, has_lbrr, internal_frames)?;
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
    prev_decode_only_middle: &mut bool,
    packet_idx: usize,
    trace_pkt0: bool,
) -> Result<(), Error> {
    for i in 0..internal_frames {
        let mut decode_only_middle = false;
        if packet_channels == 2 {
            let pred_q13 = stereo::decode_stereo_pred(dec);
            decode_only_middle = !header.channels[1].vad_flags[i] && stereo::decode_mid_only(dec);
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
            if ch == 1 && decode_only_middle {
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
            lbrr::decode_pulses(dec, side.signal_type, side.quant_offset_type, frame_length)?;
            if trace_pkt0 {
                trace_checkpoint(dec, packet_idx, &format!("after_main_i{}_ch{}", i, ch));
            }
        }

        *prev_decode_only_middle = decode_only_middle;
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
        2 => Ok(16_000),
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
}
