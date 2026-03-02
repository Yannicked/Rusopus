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
const LOG_ENERGY_FLOOR_DB: f32 = -28.0;
const BITRES: i32 = 3;
const COMBFILTER_MINPERIOD: usize = 15;
const DECODE_BUFFER_SIZE: usize = 2048;
const COMB_FILTER_GAINS: [[f32; 3]; 3] = [
    [0.306_640_62, 0.217_041_02, 0.129_638_67],
    [0.463_867_2, 0.268_066_4, 0.0],
    [0.799_804_7, 0.100_097_66, 0.0],
];
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
    mdct_480: mdct::MdctBackward,
    mdct_240: mdct::MdctBackward,
    mdct_short: mdct::MdctBackward,
    overlap: Vec<f32>,
    decode_mem: Vec<f32>,
    prev_energy: Vec<f32>,
    old_log_energy: Vec<f32>,
    old_log_energy2: Vec<f32>,
    background_log_energy: Vec<f32>,
    deemph_mem: Vec<f32>,
    postfilter_period: i32,
    postfilter_period_old: i32,
    postfilter_gain: f32,
    postfilter_gain_old: f32,
    postfilter_tapset: usize,
    postfilter_tapset_old: usize,
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
        let output_ch = channels as usize;
        let state_ch = 2usize;
        Self {
            fs_hz,
            channels,
            mode,
            mdct: mdct::MdctBackward::new(1920, mode.overlap),
            mdct_480: mdct::MdctBackward::new(960, mode.overlap),
            mdct_240: mdct::MdctBackward::new(480, mode.overlap),
            mdct_short: mdct::MdctBackward::new(240, mode.overlap),
            overlap: vec![0.0; mode.overlap * output_ch],
            decode_mem: vec![0.0; DECODE_BUFFER_SIZE + mode.overlap],
            prev_energy: vec![0.0; NBANDS_48K_20MS * state_ch],
            old_log_energy: vec![LOG_ENERGY_FLOOR_DB; NBANDS_48K_20MS * state_ch],
            old_log_energy2: vec![LOG_ENERGY_FLOOR_DB; NBANDS_48K_20MS * state_ch],
            background_log_energy: vec![0.0; NBANDS_48K_20MS * state_ch],
            deemph_mem: vec![0.0; state_ch],
            postfilter_period: 0,
            postfilter_period_old: 0,
            postfilter_gain: 0.0,
            postfilter_gain_old: 0.0,
            postfilter_tapset: 0,
            postfilter_tapset_old: 0,
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
        self.decode_mem.fill(0.0);
        self.prev_energy.fill(0.0);
        self.old_log_energy.fill(LOG_ENERGY_FLOOR_DB);
        self.old_log_energy2.fill(LOG_ENERGY_FLOOR_DB);
        self.background_log_energy.fill(0.0);
        self.deemph_mem.fill(0.0);
        self.postfilter_period = 0;
        self.postfilter_period_old = 0;
        self.postfilter_gain = 0.0;
        self.postfilter_gain_old = 0.0;
        self.postfilter_tapset = 0;
        self.postfilter_tapset_old = 0;
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

    /// Apply CELT pitch comb postfilter in-place on decoder memory.
    ///
    /// Params: `buf` is decode memory, `start` is frame start in `buf`, `t0`/`t1`
    /// are old/new periods, `n` is frame segment length, `g0`/`g1` are old/new
    /// gains, `tapset0`/`tapset1` select tap weights, `window` is overlap window,
    /// `overlap` is overlap length.
    /// Returns: nothing.
    fn comb_filter_in_place(
        buf: &mut [f32],
        start: usize,
        t0: usize,
        t1: usize,
        n: usize,
        g0: f32,
        g1: f32,
        tapset0: usize,
        tapset1: usize,
        window: &[f32],
        overlap: usize,
    ) {
        if n == 0 || (g0 == 0.0 && g1 == 0.0) {
            return;
        }
        let period0 = t0.max(COMBFILTER_MINPERIOD);
        let period1 = t1.max(COMBFILTER_MINPERIOD);
        let ts0 = tapset0.min(2);
        let ts1 = tapset1.min(2);
        let g00 = g0 * COMB_FILTER_GAINS[ts0][0];
        let g01 = g0 * COMB_FILTER_GAINS[ts0][1];
        let g02 = g0 * COMB_FILTER_GAINS[ts0][2];
        let g10 = g1 * COMB_FILTER_GAINS[ts1][0];
        let g11 = g1 * COMB_FILTER_GAINS[ts1][1];
        let g12 = g1 * COMB_FILTER_GAINS[ts1][2];
        let mut overlap_len = overlap.min(n);
        if g0 == g1 && period0 == period1 && ts0 == ts1 {
            overlap_len = 0;
        }

        let mut x1 = buf[start - period1 + 1];
        let mut x2 = buf[start - period1];
        let mut x3 = buf[start - period1 - 1];
        let mut x4 = buf[start - period1 - 2];

        for i in 0..overlap_len {
            let x0 = buf[start + i - period1 + 2];
            let f = window[i] * window[i];
            let one_minus_f = 1.0 - f;
            let mut y = buf[start + i];
            y += (one_minus_f * g00) * buf[start + i - period0];
            y += (one_minus_f * g01) * (buf[start + i - period0 + 1] + buf[start + i - period0 - 1]);
            y += (one_minus_f * g02) * (buf[start + i - period0 + 2] + buf[start + i - period0 - 2]);
            y += (f * g10) * x2;
            y += (f * g11) * (x1 + x3);
            y += (f * g12) * (x0 + x4);
            buf[start + i] = y;
            x4 = x3;
            x3 = x2;
            x2 = x1;
            x1 = x0;
        }

        if g1 == 0.0 {
            return;
        }
        for i in overlap_len..n {
            let x0 = buf[start + i - period1 + 2];
            let y = buf[start + i] + g10 * x2 + g11 * (x1 + x3) + g12 * (x0 + x4);
            buf[start + i] = y;
            x4 = x3;
            x3 = x2;
            x2 = x1;
            x1 = x0;
        }
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
        config: u8,
        packet_channels: u8,
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
        let trace_this_packet = trace_packet_idx == Some(packet_idx);
        let coded_channels = packet_channels.clamp(1, 2) as usize;
        if coded_channels == 1 {
            let nb = self.mode.nb_ebands;
            for i in 0..nb {
                self.prev_energy[i] = self.prev_energy[i].max(self.prev_energy[nb + i]);
            }
        }
        let start = 0usize;
        let end = bandwidth_end(config);
        if trace_this_packet {
            eprintln!(
                "R pkt{} config={} end={} coded_channels={} output_channels={}",
                packet_idx, config, end, coded_channels, self.channels
            );
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H4\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:184\",\"message\":\"target_pkt_channels_and_config\",\"data\":{{\"packet_idx\":{},\"config\":{},\"end\":{},\"coded_channels\":{},\"output_channels\":{}}},\"timestamp\":{}}}\n",
                    packet_idx, config, end, coded_channels, self.channels, ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
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
        if trace_this_packet {
            eprintln!("R pkt{} [after_silence_flag] tell={}", packet_idx, ec.tell_frac());
        }

        let mut postfilter_pitch = 0i32;
        let mut postfilter_qg = -1i32;
        let mut postfilter_tapset = 0i32;
        let mut postfilter_gain = 0.0f32;
        if tell + 16 <= total_bits && ec.dec_bit_logp(1) {
            let octave = ec.dec_uint(6) as i32;
            postfilter_pitch = ((16 << octave) + ec.dec_bits((4 + octave) as u32) as i32) - 1;
            postfilter_qg = ec.dec_bits(3) as i32;
            if ec.tell() + 2 <= total_bits {
                postfilter_tapset = ec.dec_icdf(&TAPSET_ICDF, 2) as i32;
            }
            postfilter_gain = 0.09375 * (postfilter_qg + 1) as f32;
        }
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
                "{{\"sessionId\":\"4200c5\",\"runId\":\"run-postfilter-overlap-scan\",\"hypothesisId\":\"H10\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:227\",\"message\":\"postfilter_decoded\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"postfilter_active\":{},\"postfilter_pitch\":{},\"postfilter_qg\":{},\"postfilter_tapset\":{}}},\"timestamp\":{}}}\n",
                packet_idx,
                coded_channels,
                postfilter_pitch > 0,
                postfilter_pitch,
                postfilter_qg,
                postfilter_tapset,
                ts
            );
            let _ = std::io::Write::write_all(&mut f, line.as_bytes());
        }
        // #endregion
        if postfilter_pitch > 0 {
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-postfilter-overlap-scan\",\"hypothesisId\":\"H11\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:249\",\"message\":\"postfilter_active_but_not_applied_in_rust\",\"data\":{{\"packet_idx\":{},\"postfilter_pitch\":{},\"postfilter_qg\":{},\"postfilter_tapset\":{},\"comb_filter_applied\":false}},\"timestamp\":{}}}\n",
                    packet_idx, postfilter_pitch, postfilter_qg, postfilter_tapset, ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        tell = ec.tell();
        if trace_this_packet {
            eprintln!("R pkt{} [after_postfilter] tell={}", packet_idx, ec.tell_frac());
        }
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
        if trace_this_packet {
            eprintln!("R pkt{} [after_transient] tell={}", packet_idx, ec.tell_frac());
        }
        tell = ec.tell();
        let intra_ener = tell + 3 <= total_bits && ec.dec_bit_logp(3);
        if trace_this_packet {
            eprintln!("R pkt{} [after_intra_ener] tell={}", packet_idx, ec.tell_frac());
        }
        if trace_this_packet {
            eprintln!(
                "R pkt{} prev_energy_all: {:?}",
                packet_idx,
                &self.prev_energy[..self.mode.nb_ebands]
            );
            eprintln!(
                "R pkt{} old_log_energy_all: {:?}",
                packet_idx,
                &self.old_log_energy[..self.mode.nb_ebands]
            );
            eprintln!(
                "R pkt{} old_log_energy2_all: {:?}",
                packet_idx,
                &self.old_log_energy2[..self.mode.nb_ebands]
            );
        }

        quant_bands::unquant_coarse_energy(
            self.mode,
            start,
            end,
            &mut self.prev_energy,
            intra_ener,
            &mut ec,
            coded_channels,
            lm,
            total_bits,
        );
        if trace_this_packet {
            eprintln!("R pkt{} [after_coarse_energy] tell={}", packet_idx, ec.tell_frac());
        }
        let mut tf_res = vec![0i32; self.mode.nb_ebands];
        tf_decode(
            start,
            end,
            is_transient,
            &mut tf_res,
            lm,
            total_bits,
            &mut ec,
        );
        if trace_this_packet {
            eprintln!("R pkt{} [after_tf] tell={}", packet_idx, ec.tell_frac());
        }
        let spread_decision = if ec.tell() + 4 <= total_bits {
            ec.dec_icdf(&SPREAD_ICDF, 5)
        } else {
            0
        };
        if trace_this_packet {
            eprintln!("R pkt{} [after_spread] tell={}", packet_idx, ec.tell_frac());
        }
        let cap = rate::init_caps(self.mode, lm, coded_channels);
        let mut offsets = vec![0i32; self.mode.nb_ebands];
        let mut dynalloc_logp = 6i32;
        let mut total_bits_q = total_bits << BITRES;
        tell = ec.tell_frac() as i32;
        if trace_this_packet {
            eprintln!("R pkt{} [after_dynalloc_start] tell={}", packet_idx, ec.tell_frac());
        }
        for i in start..end {
            let width =
                (coded_channels as i32) * (((self.mode.e_bands[i + 1] - self.mode.e_bands[i]) as i32) << lm);
            let quanta = ((width << BITRES).min((6 << BITRES).max(width))).max(0);
            let mut dynalloc_loop_logp = dynalloc_logp;
            let mut boost = 0i32;
            let tell_before_dyn_band = tell;
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
                dynalloc_logp = (dynalloc_logp - 1).max(2);
            }
            if trace_this_packet {
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
                        "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H8\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:318\",\"message\":\"dynalloc_band_state\",\"data\":{{\"packet_idx\":{},\"band\":{},\"width\":{},\"quanta\":{},\"cap\":{},\"boost\":{},\"tell_before\":{},\"tell_after\":{},\"total_bits_q\":{},\"dynalloc_logp\":{}}},\"timestamp\":{}}}\n",
                        packet_idx,
                        i,
                        width,
                        quanta,
                        cap[i],
                        boost,
                        tell_before_dyn_band,
                        tell,
                        total_bits_q,
                        dynalloc_logp,
                        ts
                    );
                    let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                }
                // #endregion
            }
        }
        if trace_this_packet {
            eprintln!("R pkt{} [after_dynalloc_end] tell={}", packet_idx, ec.tell_frac());
        }
        let alloc_trim = if tell + (6 << BITRES) <= total_bits_q {
            ec.dec_icdf(&TRIM_ICDF, 7)
        } else {
            5
        };
        if trace_this_packet {
            eprintln!("R pkt{} [after_trim] tell={}", packet_idx, ec.tell_frac());
        }
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
            start,
            end,
            &offsets,
            &cap,
            alloc_trim,
            bits,
            coded_channels,
            lm,
            &mut ec,
        );
        if trace_this_packet {
            eprintln!("R pkt{} [after_alloc] tell={}", packet_idx, ec.tell_frac());
            for i in start..end {
                eprintln!(
                    "R pkt{} alloc band={} pulse={} fine={} prio={}",
                    packet_idx, i, alloc.pulses[i], alloc.fine_quant[i], alloc.fine_priority[i]
                );
            }
        }
        if trace_this_packet {
            eprintln!(
                "pkt{}: is_transient={} anti_collapse_rsv={} coded_bands={}",
                packet_idx,
                is_transient, anti_collapse_rsv, alloc.coded_bands
            );
            eprintln!(
                "pkt{}: tell_before_bands={} total_bits_q={}",
                packet_idx,
                ec.tell_frac(),
                (frame.len() as i32 * 8) << 3
            );
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H2\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:368\",\"message\":\"tell_before_fine_with_alloc\",\"data\":{{\"packet_idx\":{},\"tell\":{},\"total_bits_q\":{},\"channels\":{},\"intensity\":{},\"dual_stereo\":{},\"coded_bands\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    ec.tell_frac(),
                    (frame.len() as i32 * 8) << 3,
                    coded_channels,
                    alloc.intensity,
                    alloc.dual_stereo,
                    alloc.coded_bands,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        quant_bands::unquant_fine_energy(
            self.mode,
            start,
            end,
            &mut self.prev_energy,
            &alloc.fine_quant,
            &mut ec,
            coded_channels,
        );
        if trace_this_packet {
            eprintln!("R pkt{} [after_fine_energy] tell={}", packet_idx, ec.tell_frac());
            eprintln!("R pkt{} tell_before_bands_actual={}", packet_idx, ec.tell_frac());
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H3\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:383\",\"message\":\"tell_before_bands_actual\",\"data\":{{\"packet_idx\":{},\"tell\":{},\"channels\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    ec.tell_frac(),
                    coded_channels,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        let mut mdct_in = vec![0.0f32; frame_size_48k];
        let tell_before_bands_decode = ec.tell_frac();
        let mut mdct_side = if coded_channels == 2 {
            Some(vec![0.0f32; frame_size_48k])
        } else {
            None
        };
        let band_decode_path = if coded_channels == 2 {
            "quant_all_bands_stereo"
        } else {
            "quant_all_bands_mono"
        };
        if trace_this_packet {
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H5\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:434\",\"message\":\"band_decode_path_selected\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"path\":\"{}\",\"intensity\":{},\"dual_stereo\":{},\"tell_before\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    band_decode_path,
                    alloc.intensity,
                    alloc.dual_stereo,
                    tell_before_bands_decode,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        let _collapse_masks = if let Some(ref mut side) = mdct_side {
            bands::quant_all_bands_stereo(
                self.mode,
                start,
                end,
                &mut mdct_in,
                side,
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
                alloc.dual_stereo,
                alloc.intensity,
            )
        } else {
            bands::quant_all_bands_mono(
                self.mode,
                start,
                end,
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
            )
        };
        if trace_this_packet {
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
                let tell_after = ec.tell_frac();
                let line = format!(
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H6\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:450\",\"message\":\"band_decode_consumed_bits\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"tell_before\":{},\"tell_after\":{},\"consumed\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    tell_before_bands_decode,
                    tell_after,
                    tell_after as i32 - tell_before_bands_decode as i32,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        if trace_packet_idx == Some(packet_idx) {
            let m = 1usize << lm;
            for i in 0..self.mode.nb_ebands {
                let start = m * self.mode.e_bands[i] as usize;
                let end = m * self.mode.e_bands[i + 1] as usize;
                let band_sum: f32 = mdct_in[start..end].iter().sum();
                eprintln!("pkt{} band {} norm_sum={:.6}", packet_idx, i, band_sum);
                if let Some(side) = mdct_side.as_ref() {
                    let side_sum: f32 = side[start..end].iter().sum();
                    eprintln!("pkt{} band {} norm_sum_side={:.6}", packet_idx, i, side_sum);
                }
            }
        }
        if trace_this_packet && coded_channels == 2 {
            let mid_abs = mdct_in.iter().map(|v| v.abs()).sum::<f32>();
            let side_abs = mdct_side
                .as_ref()
                .map(|v| v.iter().map(|x| x.abs()).sum::<f32>())
                .unwrap_or(0.0);
            let output_strategy = if self.channels == 1 {
                "freq_downmix_after_denorm"
            } else {
                "multi_channel_output"
            };
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-stereo-probe\",\"hypothesisId\":\"H9\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:486\",\"message\":\"stereo_vectors_before_denorm\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"mid_abs\":{},\"side_abs\":{},\"output_strategy\":\"{}\"}},\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    mid_abs,
                    side_abs,
                    output_strategy,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        self.last_split_count = 0;
        self.last_is_transient = is_transient;
        if packet_idx == 0 {
            let m = 1usize << lm;
            let mut band_abs = String::new();
            for bi in start..end {
                let bs = m * self.mode.e_bands[bi] as usize;
                let be = m * self.mode.e_bands[bi + 1] as usize;
                let acc: f32 = mdct_in[bs..be].iter().map(|v| v.abs()).sum();
                if !band_abs.is_empty() {
                    band_abs.push(',');
                }
                band_abs.push_str(&format!("{:.6}", acc));
            }
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-norm-band-scan\",\"hypothesisId\":\"H21\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:676\",\"message\":\"norm_band_abs_snapshot\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"start\":{},\"end\":{},\"lm\":{},\"band_abs_csv\":\"{}\"}},\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    start,
                    end,
                    lm,
                    band_abs,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        if (packet_idx == 0 || (2128..=2129).contains(&packet_idx)) && is_transient {
            let m_blocks = 1usize << lm;
            let short_len = frame_size_48k / m_blocks.max(1);
            // #region agent log
            if m_blocks == 8 && short_len > 0 {
                let mut block_abs = [0.0f32; 8];
                for b in 0..8 {
                    let mut acc = 0.0f32;
                    for j in 0..short_len {
                        acc += mdct_in[j * m_blocks + b].abs();
                    }
                    block_abs[b] = acc;
                }
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
                        "{{\"sessionId\":\"4200c5\",\"runId\":\"run-norm-block-scan\",\"hypothesisId\":\"H20\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:684\",\"message\":\"norm_block_abs_snapshot\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"blocks\":{},\"short_len\":{},\"block_abs\":[{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}]}},\"timestamp\":{}}}\n",
                        packet_idx,
                        coded_channels,
                        m_blocks,
                        short_len,
                        block_abs[0],
                        block_abs[1],
                        block_abs[2],
                        block_abs[3],
                        block_abs[4],
                        block_abs[5],
                        block_abs[6],
                        block_abs[7],
                        ts
                    );
                    let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                }
            }
            // #endregion
        }
        // Anti-collapse bit (consumed from range coder for transient frames)
        if anti_collapse_rsv > 0 {
            let _anti_collapse_on = ec.dec_bits(1);
        }
        quant_bands::unquant_energy_finalise(
            self.mode,
            start,
            end,
            Some(&mut self.prev_energy),
            &alloc.fine_quant,
            &alloc.fine_priority,
            frame.len() as i32 * 8 - ec.tell(),
            &mut ec,
            coded_channels,
        );
        let mut denorm = vec![0.0f32; frame_size_48k];
        bands::denormalise_bands(
            self.mode,
            &mdct_in,
            &mut denorm,
            &self.prev_energy[..self.mode.nb_ebands],
            start,
            end,
            lm,
        );
        if coded_channels == 2 && self.channels == 1 {
            if let Some(side) = mdct_side.as_ref() {
                let mut denorm_side = vec![0.0f32; frame_size_48k];
                bands::denormalise_bands(
                    self.mode,
                    side,
                    &mut denorm_side,
                    &self.prev_energy[self.mode.nb_ebands..self.mode.nb_ebands * 2],
                    start,
                    end,
                    lm,
                );
                if packet_idx == 2135 {
                    let sum_l: f32 = denorm.iter().map(|x| x.abs()).sum();
                    let sum_r: f32 = denorm_side.iter().map(|x| x.abs()).sum();
                    eprintln!("R pkt2135 denorm abs_sum: L={:.6} R={:.6}", sum_l, sum_r);
                }
                for i in 0..frame_size_48k {
                    denorm[i] = 0.5 * (denorm[i] + denorm_side[i]);
                }
                if packet_idx == 2135 {
                    let sum_mono: f32 = denorm.iter().map(|x| x.abs()).sum();
                    eprintln!("R pkt2135 mono abs_sum after downmix: {:.6}", sum_mono);
                }
            }
        }
        if packet_idx == 0 && is_transient {
            let m_blocks = 1usize << lm;
            let mut b0_csv = String::new();
            for bi in start..end {
                let bs = m_blocks * self.mode.e_bands[bi] as usize;
                let be = m_blocks * self.mode.e_bands[bi + 1] as usize;
                let mut acc = 0.0f32;
                let mut j = bs;
                while j < be {
                    acc += denorm[j].abs();
                    j += m_blocks;
                }
                if !b0_csv.is_empty() {
                    b0_csv.push(',');
                }
                b0_csv.push_str(&format!("{:.6}", acc));
            }
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-band-block0-scan\",\"hypothesisId\":\"H25\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:734\",\"message\":\"denorm_band_block0_abs\",\"data\":{{\"packet_idx\":{},\"start\":{},\"end\":{},\"lm\":{},\"band_block0_abs_csv\":\"{}\"}},\"timestamp\":{}}}\n",
                    packet_idx,
                    start,
                    end,
                    lm,
                    b0_csv,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }

        if packet_idx == 0 || (2127..=2129).contains(&packet_idx) {
            let ovl_sum: f32 = self.overlap.iter().map(|x| x.abs()).sum();
            let ovl_n = self.overlap.len().min(8);
            eprintln!(
                "R pkt{} overlap_in len={} abs_sum={:.6} first8: {:?}",
                packet_idx,
                self.overlap.len(),
                ovl_sum,
                &self.overlap[..ovl_n]
            );
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-postfilter-overlap-scan\",\"hypothesisId\":\"H12\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:734\",\"message\":\"overlap_in_snapshot\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"overlap_len\":{},\"overlap_abs_sum\":{},\"overlap_first8\":[{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}]}} ,\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    self.overlap.len(),
                    ovl_sum,
                    self.overlap.get(0).copied().unwrap_or(0.0),
                    self.overlap.get(1).copied().unwrap_or(0.0),
                    self.overlap.get(2).copied().unwrap_or(0.0),
                    self.overlap.get(3).copied().unwrap_or(0.0),
                    self.overlap.get(4).copied().unwrap_or(0.0),
                    self.overlap.get(5).copied().unwrap_or(0.0),
                    self.overlap.get(6).copied().unwrap_or(0.0),
                    self.overlap.get(7).copied().unwrap_or(0.0),
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }

        let mut base_synth = vec![0.0f32; frame_size_48k];
        if packet_idx == 0 || (2127..=2129).contains(&packet_idx) {
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
                let denorm_abs: f32 = denorm.iter().map(|x| x.abs()).sum();
                let line = format!(
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-synthesis-path-scan\",\"hypothesisId\":\"H15\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:769\",\"message\":\"synthesis_path_decision\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"is_transient\":{},\"lm\":{},\"frame_size_48k\":{},\"postfilter_active\":{},\"postfilter_pitch\":{},\"denorm_abs_sum\":{}}},\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    is_transient,
                    lm,
                    frame_size_48k,
                    postfilter_pitch > 0,
                    postfilter_pitch,
                    denorm_abs,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }
        if is_transient {
            // Transient CELT frames are composed of 120-sample short blocks.
            let m_blocks = 1usize << lm;
            let short_len = frame_size_48k / m_blocks;
            let mut short_coeffs = vec![0.0f32; short_len];
            let mut synth_tmp = vec![0.0f32; frame_size_48k + self.mode.overlap];
            synth_tmp[..self.mode.overlap].copy_from_slice(&self.overlap);

            for b in 0..m_blocks {
                let out_offset = b * short_len;
                let overlap_in_abs: f32 = synth_tmp[out_offset..out_offset + self.mode.overlap]
                    .iter()
                    .map(|x| x.abs())
                    .sum();
                for j in 0..short_len {
                    short_coeffs[j] = denorm[j * m_blocks + b];
                }
                let short_coeff_abs: f32 = short_coeffs.iter().map(|x| x.abs()).sum();
                let short_coeff_abs_contig: f32 = (0..short_len)
                    .map(|j| denorm[b * short_len + j].abs())
                    .sum();
                self.mdct_short
                    .backward(
                        &short_coeffs,
                        self.mode.window,
                        &mut synth_tmp[out_offset..out_offset + short_len + self.mode.overlap],
                    )
                    .map_err(|_| Error::NotImplemented)?;
                if (packet_idx == 0 || (2127..=2129).contains(&packet_idx))
                    && (packet_idx == 0 || b == 0 || b + 1 == m_blocks)
                {
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
                        let head_abs: f32 = synth_tmp
                            [out_offset..out_offset + self.mode.overlap]
                            .iter()
                            .map(|x| x.abs())
                            .sum();
                        let tail_abs: f32 = synth_tmp
                            [out_offset + short_len..out_offset + short_len + self.mode.overlap]
                            .iter()
                            .map(|x| x.abs())
                            .sum();
                        let line = format!(
                            "{{\"sessionId\":\"4200c5\",\"runId\":\"run-synthesis-path-scan\",\"hypothesisId\":\"H16\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:804\",\"message\":\"short_mdct_block_snapshot\",\"data\":{{\"packet_idx\":{},\"block\":{},\"blocks_total\":{},\"short_len\":{},\"overlap\":{},\"overlap_in_abs_sum\":{},\"short_coeff_abs_sum\":{},\"short_coeff_abs_contig\":{},\"short_tmp_head_abs_sum\":{},\"short_tmp_tail_abs_sum\":{}}},\"timestamp\":{}}}\n",
                            packet_idx,
                            b,
                            m_blocks,
                            short_len,
                            self.mode.overlap,
                            overlap_in_abs,
                            short_coeff_abs,
                            short_coeff_abs_contig,
                            head_abs,
                            tail_abs,
                            ts
                        );
                        let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                    }
                    // #endregion
                }
            }
            base_synth.copy_from_slice(&synth_tmp[..frame_size_48k]);
            self.overlap
                .copy_from_slice(&synth_tmp[frame_size_48k..frame_size_48k + self.mode.overlap]);
        } else {
            // Long block path for all supported frame sizes.
            let mut mdct_tmp = vec![0.0f32; frame_size_48k * 2];
            mdct_tmp[..self.mode.overlap].copy_from_slice(&self.overlap);
            let mdct_impl = match frame_size_48k {
                120 => &self.mdct_short,
                240 => &self.mdct_240,
                480 => &self.mdct_480,
                960 => &self.mdct,
                _ => return Err(Error::NotImplemented),
            };
            mdct_impl
                .backward(&denorm, self.mode.window, &mut mdct_tmp)
                .map_err(|_| Error::NotImplemented)?;
            if packet_idx == 0 || (2127..=2129).contains(&packet_idx) {
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
                    let head_abs: f32 = mdct_tmp[..self.mode.overlap].iter().map(|x| x.abs()).sum();
                    let tail_abs: f32 = mdct_tmp[frame_size_48k..frame_size_48k + self.mode.overlap]
                        .iter()
                        .map(|x| x.abs())
                        .sum();
                    let line = format!(
                        "{{\"sessionId\":\"4200c5\",\"runId\":\"run-synthesis-path-scan\",\"hypothesisId\":\"H17\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:854\",\"message\":\"long_mdct_snapshot\",\"data\":{{\"packet_idx\":{},\"frame_size_48k\":{},\"overlap\":{},\"mdct_head_abs_sum\":{},\"mdct_tail_abs_sum\":{}}},\"timestamp\":{}}}\n",
                        packet_idx,
                        frame_size_48k,
                        self.mode.overlap,
                        head_abs,
                        tail_abs,
                        ts
                    );
                    let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                }
                // #endregion
            }
            base_synth.copy_from_slice(&mdct_tmp[..frame_size_48k]);
            self.overlap
                .copy_from_slice(&mdct_tmp[frame_size_48k..frame_size_48k + self.mode.overlap]);
        }
        if packet_idx == 0 || (2127..=2129).contains(&packet_idx) {
            let ovl_sum: f32 = self.overlap.iter().map(|x| x.abs()).sum();
            let ovl_n = self.overlap.len().min(8);
            eprintln!(
                "R pkt{} overlap_out len={} abs_sum={:.6} first8: {:?}",
                packet_idx,
                self.overlap.len(),
                ovl_sum,
                &self.overlap[..ovl_n]
            );
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-postfilter-overlap-scan\",\"hypothesisId\":\"H13\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:793\",\"message\":\"overlap_out_snapshot\",\"data\":{{\"packet_idx\":{},\"coded_channels\":{},\"overlap_len\":{},\"overlap_abs_sum\":{},\"overlap_first8\":[{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}]}} ,\"timestamp\":{}}}\n",
                    packet_idx,
                    coded_channels,
                    self.overlap.len(),
                    ovl_sum,
                    self.overlap.get(0).copied().unwrap_or(0.0),
                    self.overlap.get(1).copied().unwrap_or(0.0),
                    self.overlap.get(2).copied().unwrap_or(0.0),
                    self.overlap.get(3).copied().unwrap_or(0.0),
                    self.overlap.get(4).copied().unwrap_or(0.0),
                    self.overlap.get(5).copied().unwrap_or(0.0),
                    self.overlap.get(6).copied().unwrap_or(0.0),
                    self.overlap.get(7).copied().unwrap_or(0.0),
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }

        let decode_len = DECODE_BUFFER_SIZE + self.mode.overlap;
        self.decode_mem.copy_within(frame_size_48k..decode_len, 0);
        let out_start = DECODE_BUFFER_SIZE - frame_size_48k;
        self.decode_mem[out_start..out_start + frame_size_48k].copy_from_slice(&base_synth);
        self.decode_mem[out_start + frame_size_48k..out_start + frame_size_48k + self.mode.overlap]
            .copy_from_slice(&self.overlap);

        let postfilter_before_abs: f32 = self.decode_mem[out_start..out_start + frame_size_48k]
            .iter()
            .map(|x| x.abs())
            .sum();
        let pre_s163 = if frame_size_48k > 163 {
            self.decode_mem[out_start + 163]
        } else {
            0.0
        };
        if packet_idx == 0 && frame_size_48k >= self.mode.short_mdct_size {
            let chunk = self.mode.short_mdct_size;
            let chunks = frame_size_48k / chunk;
            if chunks == 8 {
                let mut chunk_abs = [0.0f32; 8];
                for b in 0..8 {
                    let s = b * chunk;
                    let e = s + chunk;
                    chunk_abs[b] = self.decode_mem[out_start + s..out_start + e]
                        .iter()
                        .map(|x| x.abs())
                        .sum();
                }
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
                        "{{\"sessionId\":\"4200c5\",\"runId\":\"run-precomb-chunk-scan\",\"hypothesisId\":\"H23\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:1018\",\"message\":\"precomb_chunk_abs\",\"data\":{{\"packet_idx\":{},\"chunk_size\":{},\"chunk_abs\":[{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}],\"total_abs\":{}}},\"timestamp\":{}}}\n",
                        packet_idx,
                        chunk,
                        chunk_abs[0],
                        chunk_abs[1],
                        chunk_abs[2],
                        chunk_abs[3],
                        chunk_abs[4],
                        chunk_abs[5],
                        chunk_abs[6],
                        chunk_abs[7],
                        postfilter_before_abs,
                        ts
                    );
                    let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                }
                // #endregion
            }
        }
        let pf_period = self.postfilter_period.max(COMBFILTER_MINPERIOD as i32) as usize;
        let pf_period_old = self.postfilter_period_old.max(COMBFILTER_MINPERIOD as i32) as usize;
        let decoded_tapset = postfilter_tapset.clamp(0, 2) as usize;
        let short_n = self.mode.short_mdct_size.min(frame_size_48k);
        Self::comb_filter_in_place(
            &mut self.decode_mem,
            out_start,
            pf_period_old,
            pf_period,
            short_n,
            self.postfilter_gain_old,
            self.postfilter_gain,
            self.postfilter_tapset_old,
            self.postfilter_tapset,
            self.mode.window,
            self.mode.overlap,
        );
        if lm != 0 && frame_size_48k > short_n {
            Self::comb_filter_in_place(
                &mut self.decode_mem,
                out_start + short_n,
                pf_period,
                postfilter_pitch.max(0) as usize,
                frame_size_48k - short_n,
                self.postfilter_gain,
                postfilter_gain,
                self.postfilter_tapset,
                decoded_tapset,
                self.mode.window,
                self.mode.overlap,
            );
        }
        self.postfilter_period_old = self.postfilter_period;
        self.postfilter_gain_old = self.postfilter_gain;
        self.postfilter_tapset_old = self.postfilter_tapset;
        self.postfilter_period = postfilter_pitch;
        self.postfilter_gain = postfilter_gain;
        self.postfilter_tapset = decoded_tapset;
        if lm != 0 {
            self.postfilter_period_old = self.postfilter_period;
            self.postfilter_gain_old = self.postfilter_gain;
            self.postfilter_tapset_old = self.postfilter_tapset;
        }
        if packet_idx <= 2 || (2127..=2129).contains(&packet_idx) {
            let postfilter_after_abs: f32 = self.decode_mem[out_start..out_start + frame_size_48k]
                .iter()
                .map(|x| x.abs())
                .sum();
            let post_s163 = if frame_size_48k > 163 {
                self.decode_mem[out_start + 163]
            } else {
                0.0
            };
            let mut post_win_csv = String::new();
            if frame_size_48k > 170 {
                for i in 160..=170 {
                    if !post_win_csv.is_empty() {
                        post_win_csv.push(';');
                    }
                    post_win_csv.push_str(&format!("{i}:{:.6}", self.decode_mem[out_start + i]));
                }
            }
            let mut post_win2_csv = String::new();
            if frame_size_48k > 220 {
                for i in 200..=220 {
                    if !post_win2_csv.is_empty() {
                        post_win2_csv.push(';');
                    }
                    post_win2_csv.push_str(&format!("{i}:{:.6}", self.decode_mem[out_start + i]));
                }
            }
            let mut post_prefix_csv = String::new();
            if packet_idx == 0 && frame_size_48k > 170 {
                for i in 0..=170 {
                    if !post_prefix_csv.is_empty() {
                        post_prefix_csv.push(';');
                    }
                    post_prefix_csv.push_str(&format!("{i}:{:.6}", self.decode_mem[out_start + i]));
                }
            }
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
                    "{{\"sessionId\":\"4200c5\",\"runId\":\"run-postfilter-apply\",\"hypothesisId\":\"H19\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:980\",\"message\":\"comb_filter_applied\",\"data\":{{\"packet_idx\":{},\"lm\":{},\"postfilter_pitch\":{},\"postfilter_gain\":{},\"postfilter_tapset\":{},\"state_pitch\":{},\"state_gain\":{},\"state_tapset\":{},\"pre_abs_sum\":{},\"post_abs_sum\":{},\"pre_s163\":{},\"post_s163\":{},\"post_win_csv\":\"{}\",\"post_win2_csv\":\"{}\",\"post_prefix_csv\":\"{}\"}},\"timestamp\":{}}}\n",
                    packet_idx,
                    lm,
                    postfilter_pitch,
                    postfilter_gain,
                    decoded_tapset,
                    pf_period,
                    self.postfilter_gain,
                    self.postfilter_tapset,
                    postfilter_before_abs,
                    postfilter_after_abs,
                    pre_s163,
                    post_s163,
                    post_win_csv,
                    post_win2_csv,
                    post_prefix_csv,
                    ts
                );
                let _ = std::io::Write::write_all(&mut f, line.as_bytes());
            }
            // #endregion
        }

        let nb_ebands = self.mode.nb_ebands;
        let state_len = 2 * nb_ebands;
        if coded_channels == 1 {
            let (left, right) = self.prev_energy.split_at_mut(nb_ebands);
            right.copy_from_slice(left);
        }
        if !is_transient {
            self.old_log_energy2[..state_len].copy_from_slice(&self.old_log_energy[..state_len]);
            self.old_log_energy[..state_len].copy_from_slice(&self.prev_energy[..state_len]);
        } else {
            for i in 0..state_len {
                self.old_log_energy[i] = self.old_log_energy[i].min(self.prev_energy[i]);
            }
        }
        let m = 1usize << lm;
        let max_background_increase = (m.min(160) as f32) * 0.001;
        for i in 0..state_len {
            self.background_log_energy[i] =
                (self.background_log_energy[i] + max_background_increase).min(self.prev_energy[i]);
        }
        for c in 0..2usize {
            let base = c * nb_ebands;
            for i in 0..start.min(nb_ebands) {
                self.prev_energy[base + i] = 0.0;
                self.old_log_energy[base + i] = LOG_ENERGY_FLOOR_DB;
                self.old_log_energy2[base + i] = LOG_ENERGY_FLOOR_DB;
            }
            for i in end.min(nb_ebands)..nb_ebands {
                self.prev_energy[base + i] = 0.0;
                self.old_log_energy[base + i] = LOG_ENERGY_FLOOR_DB;
                self.old_log_energy2[base + i] = LOG_ENERGY_FLOOR_DB;
            }
        }
        if trace_this_packet {
            eprintln!(
                "R pkt{} energy_state_post prev={:?} old1={:?} old2={:?}",
                packet_idx,
                &self.prev_energy[..nb_ebands],
                &self.old_log_energy[..nb_ebands],
                &self.old_log_energy2[..nb_ebands]
            );
        }

        for ch in 0..self.channels as usize {
            let mut ch_synth = self.decode_mem[out_start..out_start + frame_size_48k].to_vec();
            self.apply_deemph(ch, &mut ch_synth);
            if packet_idx == 0 && ch == 0 && frame_size_48k > 170 {
                let mut win_csv = String::new();
                for i in 160..=170 {
                    if !win_csv.is_empty() {
                        win_csv.push(';');
                    }
                    let si = Self::float_to_i16(ch_synth[i]);
                    win_csv.push_str(&format!("{i}:{:.6}:{si}", ch_synth[i]));
                }
                let mut win2_csv = String::new();
                if frame_size_48k > 220 {
                    for i in 200..=220 {
                        if !win2_csv.is_empty() {
                            win2_csv.push(';');
                        }
                        let si = Self::float_to_i16(ch_synth[i]);
                        win2_csv.push_str(&format!("{i}:{:.6}:{si}", ch_synth[i]));
                    }
                }
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
                        "{{\"sessionId\":\"4200c5\",\"runId\":\"run-deemph-scan\",\"hypothesisId\":\"H26\",\"location\":\"crates/opus-decoder/src/celt/mod.rs:1110\",\"message\":\"deemph_window_packet0\",\"data\":{{\"packet_idx\":{},\"channel\":{},\"window_csv\":\"{}\",\"window2_csv\":\"{}\"}},\"timestamp\":{}}}\n",
                        packet_idx,
                        ch,
                        win_csv,
                        win2_csv,
                        ts
                    );
                    let _ = std::io::Write::write_all(&mut f, line.as_bytes());
                }
                // #endregion
            }
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
            rate::pulses2bits(self.mode, 0, lm as i32, 0),
        );
        Ok(CeltFrameDecode {
            samples_per_channel: frame_size_48k,
            is_transient,
        })
    }
}

/// Map CELT-only Opus config to active end band.
///
/// Params: TOC config value `(toc >> 3) & 0x1f`.
/// Returns: exclusive end band index.
fn bandwidth_end(config: u8) -> usize {
    match config {
        16..=19 => 13,
        20..=23 => 17,
        24..=27 => 19,
        28..=31 => 21,
        _ => 21,
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

