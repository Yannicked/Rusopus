//! CELT (Opus CELT-only / hybrid layer) decoder.
//!
//! This module is a decoder-first port of libopus' CELT implementation.

use crate::Error;
use crate::entropy::EcDec;
use core::sync::atomic::{AtomicUsize, Ordering};

mod kiss_fft;
mod laplace;
mod mdct;
mod modes;
mod quant_bands;
mod rate;
mod cwrs;
mod vq;
mod bands;

const OVERLAP_48K_20MS: usize = 120;
const NBANDS_48K_20MS: usize = 21;
const CELT_DEEMPH_COEFF: f32 = 0.85000610;
const BITRES: i32 = 3;
const TRIM_ICDF: [u8; 11] = [126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0];
const SPREAD_ICDF: [u8; 4] = [25, 23, 2, 0];
const TAPSET_ICDF: [u8; 3] = [2, 1, 0];
const TF_SELECT_TABLE: [[i8; 8]; 4] = [
    [0, -1, 0, -1, 0, -1, 0, -1],
    [0, -1, 0, -2, 1, 0, 1, -1],
    [0, -2, 0, -3, 2, 0, 1, -1],
    [0, -2, 0, -3, 3, 0, 1, -1],
];
static TRACE_PACKET_CALL_IDX: AtomicUsize = AtomicUsize::new(0);

/// Result of decoding one CELT frame.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CeltFrameDecode {
    /// Number of decoded samples per channel.
    pub samples_per_channel: usize,
    /// Indicates transient frame path usage in CELT.
    pub is_transient: bool,
}

#[derive(Debug, Clone)]
pub(crate) struct CeltDecoder {
    fs_hz: u32,
    channels: u8,
    mode: &'static modes::CeltMode,
    mdct: mdct::MdctBackward,
    mdct_short: mdct::MdctBackward,
    overlap: Vec<f32>,
    prev_energy: Vec<f32>,
    deemph_mem: Vec<f32>,
    final_range: u32,
    rng_seed: u32,
    last_split_count: usize,
    last_is_transient: bool,
}

impl CeltDecoder {
    /// Create CELT decoder state.
    ///
    /// Params: `fs_hz` is output sample rate and `channels` is output channels.
    /// Returns: initialized CELT decoder state.
    pub fn new(fs_hz: u32, channels: u8) -> Self {
        let mode = modes::mode48000_960_120();
        let ch = channels as usize;
        Self {
            fs_hz,
            channels,
            mode,
            mdct: mdct::MdctBackward::new(1920, mode.overlap),
            mdct_short: mdct::MdctBackward::new(240, mode.overlap),
            overlap: vec![0.0; mode.overlap * ch],
            prev_energy: vec![0.0; NBANDS_48K_20MS * ch],
            deemph_mem: vec![0.0; ch],
            final_range: 0,
            rng_seed: 0,
            last_split_count: 0,
            last_is_transient: false,
        }
    }

    /// Reset decoder state between streams.
    ///
    /// Params: none.
    /// Returns: nothing.
    pub fn reset(&mut self) {
        self.overlap.fill(0.0);
        self.prev_energy.fill(0.0);
        self.deemph_mem.fill(0.0);
        self.final_range = 0;
        self.rng_seed = 0;
        self.last_split_count = 0;
        self.last_is_transient = false;
    }

    /// Return CELT range coder final state.
    ///
    /// Params: none.
    /// Returns: range coder final value for last decoded frame.
    pub fn final_range(&self) -> u32 {
        self.final_range
    }

    /// Return deemphasis memory for the selected channel.
    ///
    /// Params: `ch` channel index.
    /// Returns: deemphasis memory value for that channel.
    pub fn deemph_mem(&self, ch: usize) -> f32 {
        self.deemph_mem[ch]
    }

    /// Return split count captured for last decoded frame.
    ///
    /// Params: none.
    /// Returns: recursive split count.
    pub fn last_split_count(&self) -> usize {
        self.last_split_count
    }

    /// Return transient flag captured for last decoded frame.
    ///
    /// Params: none.
    /// Returns: true when last frame was transient.
    pub fn last_is_transient(&self) -> bool {
        self.last_is_transient
    }

    /// Apply first-order deemphasis to one channel buffer in-place.
    ///
    /// Params: `ch` is channel index, `samples` are per-channel samples.
    /// Returns: nothing.
    fn apply_deemph(&mut self, ch: usize, samples: &mut [f32]) {
        let mut mem = self.deemph_mem[ch];
        for x in samples.iter_mut() {
            let y = *x + CELT_DEEMPH_COEFF * mem;
            mem = y;
            *x = y;
        }
        self.deemph_mem[ch] = mem;
    }

    /// Convert one floating sample to i16 with saturation.
    ///
    /// Params: `x` is floating PCM sample.
    /// Returns: saturated i16 PCM.
    fn float_to_i16(x: f32) -> i16 {
        let v = x.round().clamp(i16::MIN as f32, i16::MAX as f32);
        v as i16
    }

    /// Decode a single CELT frame.
    ///
    /// `frame_size_48k` is samples-per-channel at 48 kHz (120/240/480/960).
    pub fn decode_frame(
        &mut self,
        frame: &[u8],
        frame_size_48k: usize,
        out: &mut [i16],
    ) -> Result<CeltFrameDecode, Error> {
        if self.fs_hz != 48_000 {
            return Err(Error::NotImplemented);
        }
        if self.channels != 1 {
            return Err(Error::NotImplemented);
        }
        if !matches!(frame_size_48k, 120 | 240 | 480 | 960) {
            return Err(Error::NotImplemented);
        }
        let needed = frame_size_48k * self.channels as usize;
        if out.len() < needed {
            return Err(Error::OutputTooSmall {
                needed,
                got: out.len(),
            });
        }

        let packet_idx = TRACE_PACKET_CALL_IDX.fetch_add(1, Ordering::SeqCst);
        let trace_packet_idx = std::env::var("OPUS_TRACE_PACKET_IDX")
            .ok()
            .and_then(|v| v.parse::<usize>().ok());
        let mut ec = EcDec::new(frame);
        let total_bits = (frame.len() * 8) as i32;
        let mut tell = ec.tell();
        let _silence_flag = if tell >= total_bits {
            true
        } else if tell == 1 {
            ec.dec_bit_logp(15)
        } else {
            false
        };

        let mut postfilter_pitch = 0i32;
        if tell + 16 <= total_bits && ec.dec_bit_logp(1) {
            let octave = ec.dec_uint(6) as i32;
            postfilter_pitch = ((16 << octave) + ec.dec_bits((4 + octave) as u32) as i32) - 1;
            let _qg = ec.dec_bits(3);
            if ec.tell() + 2 <= total_bits {
                let _tapset = ec.dec_icdf(&TAPSET_ICDF, 2);
            }
        }
        tell = ec.tell();
        let lm = match frame_size_48k {
            120 => 0,
            240 => 1,
            480 => 2,
            960 => 3,
            _ => unreachable!(),
        };
        let is_transient = if lm > 0 && tell + 3 <= total_bits {
            ec.dec_bit_logp(3)
        } else {
            false
        };
        tell = ec.tell();
        let intra_ener = tell + 3 <= total_bits && ec.dec_bit_logp(3);
        if packet_idx == 35 || packet_idx == 36 {
            eprintln!(
                "pkt{} prev_energy[0..5]: {:.6} {:.6} {:.6} {:.6} {:.6}",
                packet_idx,
                self.prev_energy[0],
                self.prev_energy[1],
                self.prev_energy[2],
                self.prev_energy[3],
                self.prev_energy[4]
            );
        }

        quant_bands::unquant_coarse_energy(
            self.mode,
            0,
            self.mode.nb_ebands,
            &mut self.prev_energy,
            intra_ener,
            &mut ec,
            self.channels as usize,
            lm,
            total_bits,
        );
        let mut tf_res = vec![0i32; self.mode.nb_ebands];
        tf_decode(
            0,
            self.mode.nb_ebands,
            is_transient,
            &mut tf_res,
            lm,
            total_bits,
            &mut ec,
        );
        let spread_decision = if ec.tell() + 4 <= total_bits {
            ec.dec_icdf(&SPREAD_ICDF, 5)
        } else {
            0
        };
        let cap = rate::init_caps(self.mode, lm, self.channels as usize);
        let mut offsets = vec![0i32; self.mode.nb_ebands];
        let mut dynalloc_logp = 6i32;
        let mut total_bits_q = total_bits << BITRES;
        tell = ec.tell_frac() as i32;
        for i in 0..self.mode.nb_ebands {
            let width =
                (self.channels as i32) * (((self.mode.e_bands[i + 1] - self.mode.e_bands[i]) as i32) << lm);
            let quanta = ((width << BITRES).min((6 << BITRES).max(width))).max(0);
            let mut dynalloc_loop_logp = dynalloc_logp;
            let mut boost = 0i32;
            while tell + (dynalloc_loop_logp << BITRES) < total_bits_q && boost < cap[i] {
                let flag = ec.dec_bit_logp(dynalloc_loop_logp as u32);
                tell = ec.tell_frac() as i32;
                if !flag {
                    break;
                }
                boost += quanta;
                total_bits_q -= quanta;
                dynalloc_loop_logp = 1;
            }
            offsets[i] = boost;
            if boost > 0 {
                dynalloc_logp = dynalloc_logp.max(2) - 1;
            }
        }
        let alloc_trim = if tell + (6 << BITRES) <= total_bits_q {
            ec.dec_icdf(&TRIM_ICDF, 7)
        } else {
            5
        };
        let mut bits = ((frame.len() as i32 * 8) << BITRES) - ec.tell_frac() as i32 - 1;
        let anti_collapse_rsv = if is_transient && lm >= 2 && bits >= ((lm as i32 + 2) << BITRES) {
            1 << BITRES
        } else {
            0
        };
        if packet_idx < 5 {
            eprintln!(
                "pkt{} is_transient={} anti_collapse_rsv={}",
                packet_idx, is_transient, anti_collapse_rsv
            );
        }
        bits -= anti_collapse_rsv;
        let alloc = rate::clt_compute_allocation(
            self.mode,
            0,
            self.mode.nb_ebands,
            &offsets,
            &cap,
            alloc_trim,
            bits,
            self.channels as usize,
            lm,
            &mut ec,
        );
        if packet_idx == 36 {
            eprintln!(
                "pkt36: is_transient={} anti_collapse_rsv={} coded_bands={}",
                is_transient, anti_collapse_rsv, alloc.coded_bands
            );
            eprintln!(
                "pkt36: tell_before_bands={} total_bits_q={}",
                ec.tell_frac(),
                (frame.len() as i32 * 8) << 3
            );
        }
        quant_bands::unquant_fine_energy(
            self.mode,
            0,
            self.mode.nb_ebands,
            &mut self.prev_energy,
            &alloc.fine_quant,
            &mut ec,
            self.channels as usize,
        );
        let mut mdct_in = vec![0.0f32; frame_size_48k];
        let _collapse_masks = bands::quant_all_bands_mono(
            self.mode,
            0,
            self.mode.nb_ebands,
            &mut mdct_in,
            &alloc.pulses,
            is_transient,
            spread_decision,
            &tf_res,
            (frame.len() as i32 * 8 << BITRES) - anti_collapse_rsv,
            alloc.balance,
            alloc.coded_bands,
            lm,
            &mut ec,
            &mut self.rng_seed,
        );
        if trace_packet_idx == Some(packet_idx) {
            let m = 1usize << lm;
            for i in 0..self.mode.nb_ebands {
                let start = m * self.mode.e_bands[i] as usize;
                let end = m * self.mode.e_bands[i + 1] as usize;
                let band_sum: f32 = mdct_in[start..end].iter().sum();
                eprintln!("pkt{} band {} norm_sum={:.6}", packet_idx, i, band_sum);
            }
        }
        self.last_split_count = 0;
        self.last_is_transient = is_transient;
        // Anti-collapse bit (consumed from range coder for transient frames)
        if anti_collapse_rsv > 0 {
            let _anti_collapse_on = ec.dec_bits(1);
        }
        quant_bands::unquant_energy_finalise(
            self.mode,
            0,
            self.mode.nb_ebands,
            Some(&mut self.prev_energy),
            &alloc.fine_quant,
            &alloc.fine_priority,
            frame.len() as i32 * 8 - ec.tell(),
            &mut ec,
            self.channels as usize,
        );
        let mut denorm = vec![0.0f32; frame_size_48k];
        bands::denormalise_bands(
            self.mode,
            &mdct_in,
            &mut denorm,
            &self.prev_energy,
            0,
            self.mode.nb_ebands,
            lm,
        );

        let mut base_synth = vec![0.0f32; frame_size_48k];
        if frame_size_48k == 960 {
            if is_transient {
                // Short block path: M=8 short MDCTs of size 120
                let m_blocks = 1usize << lm; // 8 for lm=3
                let short_len = frame_size_48k / m_blocks; // 120
                let mut short_coeffs = vec![0.0f32; short_len];
                let mut short_tmp = vec![0.0f32; short_len + self.mode.overlap]; // 240

                for b in 0..m_blocks {
                    // Deinterleave: pick every M-th coefficient
                    for j in 0..short_len {
                        short_coeffs[j] = denorm[j * m_blocks + b];
                    }
                    // Short MDCT
                    self.mdct_short
                        .backward(&short_coeffs, self.mode.window, &mut short_tmp)
                        .map_err(|_| Error::NotImplemented)?;
                    // Overlap-add with previous block's tail (or frame overlap for b=0)
                    let out_offset = b * short_len;
                    for i in 0..self.mode.overlap {
                        base_synth[out_offset + i] = short_tmp[i] + self.overlap[i];
                    }
                    // If short_len > overlap, copy middle (not the case when short_len==overlap)
                    if short_len > self.mode.overlap {
                        base_synth[out_offset + self.mode.overlap..out_offset + short_len]
                            .copy_from_slice(&short_tmp[self.mode.overlap..short_len]);
                    }
                    // Save tail as overlap for next block/frame
                    self.overlap
                        .copy_from_slice(&short_tmp[short_len..short_len + self.mode.overlap]);
                }
            } else {
                // Long block path
                let mut mdct_tmp = vec![0.0f32; 1920];
                self.mdct
                    .backward(&denorm, self.mode.window, &mut mdct_tmp)
                    .map_err(|_| Error::NotImplemented)?;
                for i in 0..self.mode.overlap {
                    base_synth[i] = mdct_tmp[i] + self.overlap[i];
                }
                base_synth[self.mode.overlap..frame_size_48k]
                    .copy_from_slice(&mdct_tmp[self.mode.overlap..frame_size_48k]);
                self.overlap
                    .copy_from_slice(&mdct_tmp[frame_size_48k..frame_size_48k + self.mode.overlap]);
            }
        }

        for ch in 0..self.channels as usize {
            let mut ch_synth = base_synth.clone();
            self.apply_deemph(ch, &mut ch_synth);
            for i in 0..frame_size_48k {
                out[i * self.channels as usize + ch] = Self::float_to_i16(ch_synth[i]);
            }
        }

        self.final_range = ec.final_range();
        if packet_idx < 5 {
            eprintln!("pkt{} final_range: got={:08x}", packet_idx, self.final_range);
        }
        self.rng_seed = ec.rng();
        let _ = (
            OVERLAP_48K_20MS,
            self.mode.preemph[0],
            self.overlap.len(),
            self.prev_energy.len(),
            alloc.intensity,
            alloc.dual_stereo,
            quant_bands::e_means()[0],
            postfilter_pitch,
            rate::pulses2bits(self.mode, 0, lm, 0),
        );
        Ok(CeltFrameDecode {
            samples_per_channel: frame_size_48k,
            is_transient,
        })
    }
}

/// Decode CELT TF flags for active bands.
///
/// Params: band range, transient flag, mutable tf array, LM, frame bit budget and range decoder.
/// Returns: nothing; `tf_res` updated in-place.
fn tf_decode(
    start: usize,
    end: usize,
    is_transient: bool,
    tf_res: &mut [i32],
    lm: usize,
    total_bits: i32,
    dec: &mut EcDec<'_>,
) {
    let mut budget = total_bits.max(0);
    let mut tell = dec.tell();
    let mut logp = if is_transient { 2 } else { 4 };
    let tf_select_rsv = lm > 0 && tell + logp + 1 <= budget;
    if tf_select_rsv {
        budget = budget.saturating_sub(1);
    }
    let mut tf_changed = 0i32;
    let mut curr = 0i32;
    for i in start..end {
        if tell + logp <= budget {
            curr ^= i32::from(dec.dec_bit_logp(logp as u32));
            tell = dec.tell();
            tf_changed |= curr;
        }
        tf_res[i] = curr;
        logp = if is_transient { 4 } else { 5 };
    }
    let mut tf_select = 0i32;
    let idx0 = 4 * usize::from(is_transient) + tf_changed as usize;
    let idx1 = 4 * usize::from(is_transient) + 2 + tf_changed as usize;
    if tf_select_rsv && TF_SELECT_TABLE[lm][idx0] != TF_SELECT_TABLE[lm][idx1] {
        tf_select = i32::from(dec.dec_bit_logp(1));
    }
    for t in tf_res.iter_mut().take(end).skip(start) {
        let idx = 4 * usize::from(is_transient) + 2 * tf_select as usize + *t as usize;
        *t = TF_SELECT_TABLE[lm][idx] as i32;
    }
}

