//! SILK decoder-side resampler for internal-rate PCM to 48 kHz.

use crate::Error;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};

use super::resampler_private;

const RESAMPLER_MAX_BATCH_SIZE_MS: usize = 10;
const SILK_RESAMPLER_MAX_IIR_ORDER: usize = 6;
const SILK_RESAMPLER_MAX_DELAY: usize = 96;
const RESAMPLER_DEBUG_LOG_PATH: &str = "/Users/tadeusz/Opus/Rasopus/.cursor/debug-9a4e1c.log";
const RESAMPLER_DEBUG_SESSION_ID: &str = "9a4e1c";
static RESAMPLER_DEBUG_CALLS: AtomicUsize = AtomicUsize::new(0);

/// Decoder-side SILK resampler state.
#[derive(Debug, Clone)]
pub(super) struct SilkResampler {
    /// Input sampling rate in Hz.
    fs_in_hz: u32,
    /// Output sampling rate in Hz.
    fs_out_hz: u32,
    /// Input rate in kHz.
    fs_in_khz: usize,
    /// One-block input batch size.
    batch_size: usize,
    /// Ratio step used by the FIR interpolator.
    inv_ratio_q16: i32,
    /// Decoder-mode delay compensation in samples.
    input_delay: usize,
    /// IIR/allpass state.
    s_iir: [i32; SILK_RESAMPLER_MAX_IIR_ORDER],
    /// FIR/history state.
    s_fir: [i16; resampler_private::fir_history_len()],
    /// Delay buffer used by the top-level dispatcher.
    delay_buf: [i16; SILK_RESAMPLER_MAX_DELAY],
}

impl SilkResampler {
    /// Create a decoder-mode SILK resampler.
    ///
    /// Params: `fs_in_hz` internal SILK rate and `fs_out_hz` API output rate.
    /// Returns: initialized resampler state for supported decoder paths.
    pub(super) fn new(fs_in_hz: u32, fs_out_hz: u32) -> Result<Self, Error> {
        let input_delay = match (fs_in_hz, fs_out_hz) {
            (8_000, 48_000) => 0,
            (12_000, 48_000) => 4,
            (16_000, 48_000) => 7,
            _ => return Err(Error::NotImplemented),
        };
        let fs_in_khz = (fs_in_hz / 1000) as usize;
        let batch_size = fs_in_khz * RESAMPLER_MAX_BATCH_SIZE_MS;
        let mut inv_ratio_q16 = ((((fs_in_hz as i64) << 15) / fs_out_hz as i64) << 2) as i32;
        while smulww(inv_ratio_q16, fs_out_hz as i32) < ((fs_in_hz as i32) << 1) {
            inv_ratio_q16 += 1;
        }

        Ok(Self {
            fs_in_hz,
            fs_out_hz,
            fs_in_khz,
            batch_size,
            inv_ratio_q16,
            input_delay,
            s_iir: [0; SILK_RESAMPLER_MAX_IIR_ORDER],
            s_fir: [0; resampler_private::fir_history_len()],
            delay_buf: [0; SILK_RESAMPLER_MAX_DELAY],
        })
    }

    /// Check whether the current state matches a rate pair.
    ///
    /// Params: `fs_in_hz` and `fs_out_hz`.
    /// Returns: true when the resampler can be reused as-is.
    pub(super) fn matches(&self, fs_in_hz: u32, fs_out_hz: u32) -> bool {
        self.fs_in_hz == fs_in_hz && self.fs_out_hz == fs_out_hz
    }

    /// Resample internal-rate PCM to API-rate PCM.
    ///
    /// Params: immutable `input` PCM at internal rate and mutable `output` PCM at API rate.
    /// Returns: number of output samples written.
    pub(super) fn process(&mut self, input: &[i16], output: &mut [i16]) -> usize {
        debug_assert!(input.len() >= self.fs_in_khz);

        let n_samples = self.fs_in_khz - self.input_delay;
        self.delay_buf[self.input_delay..self.fs_in_khz].copy_from_slice(&input[..n_samples]);

        let mut written = 0usize;
        let first_block = self.delay_buf[..self.fs_in_khz].to_vec();
        written += self.process_iir_fir(&first_block, &mut output[written..], "first_block");
        let second_len = input.len().saturating_sub(self.fs_in_khz);
        written += self.process_iir_fir(
            &input[n_samples..n_samples + second_len],
            &mut output[written..],
            "second_block",
        );

        if self.input_delay > 0 {
            let start = input.len() - self.input_delay;
            self.delay_buf[..self.input_delay].copy_from_slice(&input[start..]);
        }
        written
    }

    /// Run the IIR+FIR upsampling core on one contiguous input slice.
    ///
    /// Params: immutable `input` block, mutable `output` buffer, and `stage` label.
    /// Returns: number of output samples written.
    fn process_iir_fir(&mut self, input: &[i16], output: &mut [i16], stage: &'static str) -> usize {
        if input.is_empty() {
            return 0;
        }

        let history = resampler_private::fir_history_len();
        let call_idx = resampler_debug_call_index();
        let input_abs_sum = abs_sum_i16(input);
        let s_iir_before = self.s_iir;
        let mut total_written = 0usize;
        let mut up2_abs_sum = 0i64;
        let mut up2_first4 = [0i16; 4];
        let mut captured_up2 = false;
        let mut in_offset = 0usize;

        while in_offset < input.len() {
            let n_samples_in = (input.len() - in_offset).min(self.batch_size);
            let mut buf = vec![0i16; history + (2 * n_samples_in) + history];
            buf[..history].copy_from_slice(&self.s_fir);
            resampler_private::up2_hq(
                &mut self.s_iir,
                &mut buf[history..history + 2 * n_samples_in],
                &input[in_offset..in_offset + n_samples_in],
                n_samples_in,
            );
            let up2_slice = &buf[history..history + 2 * n_samples_in];
            up2_abs_sum += abs_sum_i16(up2_slice);
            if !captured_up2 {
                up2_first4[..up2_slice.len().min(4)].copy_from_slice(&up2_slice[..up2_slice.len().min(4)]);
                captured_up2 = true;
            }

            let max_index_q16 = (n_samples_in as i32) << 17;
            let written = resampler_private::iir_fir_interpol(
                &mut output[total_written..],
                &buf,
                max_index_q16,
                self.inv_ratio_q16,
            );
            total_written += written;

            let src_start = 2 * n_samples_in;
            self.s_fir
                .copy_from_slice(&buf[src_start..src_start + history]);
            in_offset += n_samples_in;
        }

        if let Some(call_idx) = call_idx {
            let out_slice = &output[..total_written];
            // #region agent log H53/H54/H55
            append_resampler_debug_log(
                "resampler.rs:151",
                "resampler per-call internals",
                "resamp-call-trace",
                "H53",
                call_idx,
                stage,
                input.len(),
                total_written,
                input_abs_sum,
                up2_abs_sum,
                abs_sum_i16(out_slice),
                up2_first4,
                s_iir_before,
                self.s_iir,
            );
            // #endregion
        }

        total_written
    }

    /// Return a compact debug snapshot of internal state.
    ///
    /// Params: none.
    /// Returns: leading IIR/FIR/delay state words for runtime comparison.
    pub(super) fn debug_snapshot(&self) -> ([i32; 6], [i16; 8], [i16; 8]) {
        let mut fir = [0i16; 8];
        fir.copy_from_slice(&self.s_fir[..8]);
        let mut delay = [0i16; 8];
        delay.copy_from_slice(&self.delay_buf[..8]);
        (self.s_iir, fir, delay)
    }

    /// Return the number of output samples for a given input length.
    ///
    /// Params: input length in samples.
    /// Returns: exact output sample count for the configured ratio.
    pub(super) fn output_len(&self, input_len: usize) -> usize {
        ((input_len as u64) * self.fs_out_hz as u64 / self.fs_in_hz as u64) as usize
    }
}

/// Approximate signed `((a32 * b32) >> 16)`.
///
/// Params: signed `a32` and `b32`.
/// Returns: signed Q16-scaled product.
fn smulww(a32: i32, b32: i32) -> i32 {
    ((a32 as i64 * b32 as i64) >> 16) as i32
}

/// Return the next traced resampler call index when tracing is enabled.
///
/// Params: none.
/// Returns: `Some(call_idx)` for the first ten traced calls, otherwise `None`.
fn resampler_debug_call_index() -> Option<usize> {
    if std::env::var("OPUS_RESAMP_TRACE").ok().as_deref() != Some("1") {
        return None;
    }
    let call_idx = RESAMPLER_DEBUG_CALLS.fetch_add(1, Ordering::Relaxed);
    (call_idx < 10).then_some(call_idx)
}

/// Sum absolute PCM magnitudes for a slice.
///
/// Params: immutable PCM `samples`.
/// Returns: sum of absolute values as `i64`.
fn abs_sum_i16(samples: &[i16]) -> i64 {
    samples.iter().map(|&sample| i64::from(sample).abs()).sum()
}

/// Append one NDJSON debug record for a traced resampler call.
///
/// Params: source `location`, log `message`, debug `run_id`, `hypothesis_id`, call metadata,
/// summed magnitudes, first up2 samples, and IIR state snapshots.
/// Returns: nothing; best-effort append to the session log file.
fn append_resampler_debug_log(
    location: &str,
    message: &str,
    run_id: &str,
    hypothesis_id: &str,
    call_idx: usize,
    stage: &str,
    input_len: usize,
    output_len: usize,
    input_abs_sum: i64,
    up2_abs_sum: i64,
    output_abs_sum: i64,
    up2_first4: [i16; 4],
    s_iir_before: [i32; 6],
    s_iir_after: [i32; 6],
) {
    let Ok(mut file) = OpenOptions::new()
        .create(true)
        .append(true)
        .open(RESAMPLER_DEBUG_LOG_PATH)
    else {
        return;
    };
    let _ = writeln!(
        file,
        "{{\"sessionId\":\"{}\",\"runId\":\"{}\",\"hypothesisId\":\"{}\",\"location\":\"{}\",\"message\":\"{}\",\"timestamp\":{},\"data\":{{\"callIdx\":{},\"stage\":\"{}\",\"inputLen\":{},\"outputLen\":{},\"inputAbs\":{},\"afterUp2Abs\":{},\"outAbs\":{},\"up2First4\":{:?},\"sIirBefore\":{:?},\"sIirAfter\":{:?}}}}}",
        RESAMPLER_DEBUG_SESSION_ID,
        run_id,
        hypothesis_id,
        location,
        message,
        {
            let now = std::time::SystemTime::now();
            now.duration_since(std::time::UNIX_EPOCH)
                .map(|duration| duration.as_millis())
                .unwrap_or(0)
        },
        call_idx,
        stage,
        input_len,
        output_len,
        input_abs_sum,
        up2_abs_sum,
        output_abs_sum,
        up2_first4,
        s_iir_before,
        s_iir_after,
    );
}
