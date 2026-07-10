#![allow(dead_code)]

//! Minimal CELT FFT primitives for decoder path.
//!
//! This module intentionally implements only the inverse transform behavior
//! needed by the decoder. It follows the libopus scaling convention where the
//! inverse FFT is unscaled and the forward path applies `1/N`.

use core::f32::consts::PI;

/// Complex number used by CELT FFT/MDCT primitives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Complex32 {
    /// Real component.
    pub re: f32,
    /// Imaginary component.
    pub im: f32,
}

impl Complex32 {
    /// Create a complex value from real and imaginary parts.
    ///
    /// Params: `re` is the real component, `im` is the imaginary component.
    /// Returns: a new `Complex32`.
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }
}

/// Inverse-only FFT plan for CELT decoder usage.
#[derive(Debug, Clone)]
pub(crate) struct KissFft {
    nfft: usize,
    inv_twiddles: Vec<Complex32>,
}

impl KissFft {
    /// Build an inverse FFT plan with runtime twiddle generation.
    ///
    /// Params: `nfft` is the FFT size.
    /// Returns: a plan containing precomputed inverse twiddles.
    pub fn new(nfft: usize) -> Self {
        assert!(
            supported_fft_len(nfft),
            "CELT FFT length must factor into radix 2, 3, 4, or 5"
        );
        let mut inv_twiddles = Vec::with_capacity(nfft);
        for k in 0..nfft {
            let phase = 2.0 * PI * (k as f32) / (nfft as f32);
            inv_twiddles.push(Complex32::new(phase.cos(), phase.sin()));
        }
        Self { nfft, inv_twiddles }
    }

    /// Return FFT size configured in this plan.
    ///
    /// Params: none.
    /// Returns: number of points in the transform.
    pub fn len(&self) -> usize {
        self.nfft
    }

    /// Compute unscaled inverse FFT.
    ///
    /// Params: `input` is frequency-domain complex spectrum, `output` is
    /// destination time-domain buffer and must have length `self.len()`.
    /// Returns: `Ok(())` on success, otherwise a static validation error.
    pub fn ifft(&self, input: &[Complex32], output: &mut [Complex32]) -> Result<(), &'static str> {
        if input.len() != self.nfft || output.len() != self.nfft {
            return Err("kiss_fft length mismatch");
        }
        let mut flat_input = vec![0.0f32; 2 * self.nfft];
        let mut flat_output = vec![0.0f32; 2 * self.nfft];
        let mut scratch = vec![0.0f32; 2 * self.nfft];
        for (index, value) in input.iter().enumerate() {
            flat_input[2 * index] = value.re;
            flat_input[2 * index + 1] = value.im;
        }
        self.transform_flat(
            &flat_input,
            &mut flat_output,
            &mut scratch,
            TransformDirection::Inverse,
        )?;
        for (index, value) in output.iter_mut().enumerate() {
            value.re = flat_output[2 * index];
            value.im = flat_output[2 * index + 1];
        }
        Ok(())
    }

    /// Compute unscaled forward FFT.
    ///
    /// Params: `input` is time-domain complex vector, `output` is destination
    /// frequency-domain buffer and must have length `self.len()`.
    /// Returns: `Ok(())` on success, otherwise a static validation error.
    pub fn fft(&self, input: &[Complex32], output: &mut [Complex32]) -> Result<(), &'static str> {
        if input.len() != self.nfft || output.len() != self.nfft {
            return Err("kiss_fft length mismatch");
        }
        let mut flat_input = vec![0.0f32; 2 * self.nfft];
        let mut flat_output = vec![0.0f32; 2 * self.nfft];
        let mut scratch = vec![0.0f32; 2 * self.nfft];
        for (index, value) in input.iter().enumerate() {
            flat_input[2 * index] = value.re;
            flat_input[2 * index + 1] = value.im;
        }
        self.transform_flat(
            &flat_input,
            &mut flat_output,
            &mut scratch,
            TransformDirection::Forward,
        )?;
        for (index, value) in output.iter_mut().enumerate() {
            value.re = flat_output[2 * index];
            value.im = flat_output[2 * index + 1];
        }
        Ok(())
    }

    /// Compute a forward FFT on flat interleaved complex buffers.
    ///
    /// Params: `input`/`output` are `[re0, im0, re1, im1, ...]` and must
    /// contain exactly `2 * self.len()` values.
    /// Returns: `Ok(())` on success, otherwise a static validation error.
    pub(crate) fn fft_flat(&self, input: &[f32], output: &mut [f32]) -> Result<(), &'static str> {
        let mut scratch = vec![0.0f32; 2 * self.nfft];
        self.fft_flat_with_scratch(input, output, &mut scratch)
    }

    /// Compute a forward FFT using a reusable caller-provided scratch buffer.
    pub(crate) fn fft_flat_with_scratch(
        &self,
        input: &[f32],
        output: &mut [f32],
        scratch: &mut [f32],
    ) -> Result<(), &'static str> {
        self.transform_flat(input, output, scratch, TransformDirection::Forward)
    }

    /// Run a mixed-radix transform using one caller-provided scratch buffer.
    fn transform_flat(
        &self,
        input: &[f32],
        output: &mut [f32],
        scratch: &mut [f32],
        direction: TransformDirection,
    ) -> Result<(), &'static str> {
        let required = 2 * self.nfft;
        if input.len() != required || output.len() != required || scratch.len() < required {
            return Err("kiss_fft length mismatch");
        }
        self.transform_recursive(input, 0, 1, self.nfft, output, scratch, direction);
        Ok(())
    }

    /// Recursively split the input into radix-2/3/4/5 sub-transforms.
    #[allow(clippy::too_many_arguments)]
    fn transform_recursive(
        &self,
        input: &[f32],
        input_offset: usize,
        input_stride: usize,
        n: usize,
        output: &mut [f32],
        scratch: &mut [f32],
        direction: TransformDirection,
    ) {
        if n == 1 {
            let source = 2 * input_offset;
            output[0] = input[source];
            output[1] = input[source + 1];
            return;
        }

        let radix = preferred_radix(n);
        let sub_len = n / radix;
        for branch in 0..radix {
            let output_start = 2 * branch * sub_len;
            let output_end = output_start + 2 * sub_len;
            self.transform_recursive(
                input,
                input_offset + branch * input_stride,
                input_stride * radix,
                sub_len,
                &mut output[output_start..output_end],
                &mut scratch[..2 * sub_len],
                direction,
            );
        }

        debug_assert!(radix <= 5, "unsupported FFT radix {radix}");
        let twiddle_step = self.nfft / n;
        for sub_bin in 0..sub_len {
            let mut values = [Complex32::new(0.0, 0.0); 5];
            for branch in 0..radix {
                let source = 2 * (branch * sub_len + sub_bin);
                let value = Complex32::new(output[source], output[source + 1]);
                values[branch] = if branch == 0 || sub_bin == 0 {
                    value
                } else {
                    let twiddle_index = (branch * sub_bin * twiddle_step) % self.nfft;
                    multiply_twiddle(value, self.inv_twiddles[twiddle_index], direction)
                };
            }
            let transformed = butterfly(values, radix, direction);
            for radix_bin in 0..radix {
                let output_bin = sub_bin + sub_len * radix_bin;
                let destination = 2 * output_bin;
                scratch[destination] = transformed[radix_bin].re;
                scratch[destination + 1] = transformed[radix_bin].im;
            }
        }
        output[..2 * n].copy_from_slice(&scratch[..2 * n]);
    }
}

/// Transform phase direction.
#[derive(Debug, Clone, Copy)]
enum TransformDirection {
    Forward,
    Inverse,
}

/// Select radices used by CELT's supported 60/120/240/480-point plans.
fn preferred_radix(n: usize) -> usize {
    if n % 4 == 0 {
        4
    } else if n % 2 == 0 {
        2
    } else if n % 3 == 0 {
        3
    } else if n % 5 == 0 {
        5
    } else {
        n
    }
}

/// Check that the plan can be decomposed entirely into supported radices.
fn supported_fft_len(mut n: usize) -> bool {
    if n == 0 {
        return false;
    }
    for radix in [4usize, 2, 3, 5] {
        while n % radix == 0 {
            n /= radix;
        }
    }
    n == 1
}

/// Multiply by a plan twiddle, conjugating it for the forward transform.
#[inline(always)]
fn multiply_twiddle(
    value: Complex32,
    twiddle: Complex32,
    direction: TransformDirection,
) -> Complex32 {
    let twiddle_im = match direction {
        TransformDirection::Forward => -twiddle.im,
        TransformDirection::Inverse => twiddle.im,
    };
    Complex32::new(
        value.re * twiddle.re - value.im * twiddle_im,
        value.re * twiddle_im + value.im * twiddle.re,
    )
}

/// Apply a small radix-2/3/4/5 DFT butterfly.
#[inline(always)]
fn butterfly(
    values: [Complex32; 5],
    radix: usize,
    direction: TransformDirection,
) -> [Complex32; 5] {
    let mut output = [Complex32::new(0.0, 0.0); 5];
    match radix {
        2 => {
            output[0] = complex_add(values[0], values[1]);
            output[1] = complex_sub(values[0], values[1]);
        }
        3 => {
            const SQRT_3_OVER_2: f32 = 0.866_025_4;
            let sum = complex_add(values[1], values[2]);
            let difference = complex_sub(values[1], values[2]);
            output[0] = complex_add(values[0], sum);
            let base = complex_add(values[0], complex_scale(sum, -0.5));
            let rotation = rotate_i(complex_scale(difference, SQRT_3_OVER_2), direction);
            output[1] = complex_add(base, rotation);
            output[2] = complex_sub(base, rotation);
        }
        4 => {
            let even_sum = complex_add(values[0], values[2]);
            let even_difference = complex_sub(values[0], values[2]);
            let odd_sum = complex_add(values[1], values[3]);
            let odd_difference = complex_sub(values[1], values[3]);
            let rotation = rotate_i(odd_difference, direction);
            output[0] = complex_add(even_sum, odd_sum);
            output[1] = complex_add(even_difference, rotation);
            output[2] = complex_sub(even_sum, odd_sum);
            output[3] = complex_sub(even_difference, rotation);
        }
        5 => {
            const COS_2PI_OVER_5: f32 = 0.309_016_97;
            const COS_4PI_OVER_5: f32 = -0.809_017;
            const SIN_2PI_OVER_5: f32 = 0.951_056_54;
            const SIN_4PI_OVER_5: f32 = 0.587_785_24;

            let pair_14_sum = complex_add(values[1], values[4]);
            let pair_23_sum = complex_add(values[2], values[3]);
            let pair_14_difference = complex_sub(values[1], values[4]);
            let pair_23_difference = complex_sub(values[2], values[3]);
            output[0] = complex_add(values[0], complex_add(pair_14_sum, pair_23_sum));

            let base_14 = complex_add(
                values[0],
                complex_add(
                    complex_scale(pair_14_sum, COS_2PI_OVER_5),
                    complex_scale(pair_23_sum, COS_4PI_OVER_5),
                ),
            );
            let rotation_14 = rotate_i(
                complex_add(
                    complex_scale(pair_14_difference, SIN_2PI_OVER_5),
                    complex_scale(pair_23_difference, SIN_4PI_OVER_5),
                ),
                direction,
            );
            output[1] = complex_add(base_14, rotation_14);
            output[4] = complex_sub(base_14, rotation_14);

            let base_23 = complex_add(
                values[0],
                complex_add(
                    complex_scale(pair_14_sum, COS_4PI_OVER_5),
                    complex_scale(pair_23_sum, COS_2PI_OVER_5),
                ),
            );
            let rotation_23 = rotate_i(
                complex_sub(
                    complex_scale(pair_14_difference, SIN_4PI_OVER_5),
                    complex_scale(pair_23_difference, SIN_2PI_OVER_5),
                ),
                direction,
            );
            output[2] = complex_add(base_23, rotation_23);
            output[3] = complex_sub(base_23, rotation_23);
        }
        _ => unreachable!("unsupported FFT radix {radix}"),
    }
    output
}

#[inline(always)]
fn complex_add(left: Complex32, right: Complex32) -> Complex32 {
    Complex32::new(left.re + right.re, left.im + right.im)
}

#[inline(always)]
fn complex_sub(left: Complex32, right: Complex32) -> Complex32 {
    Complex32::new(left.re - right.re, left.im - right.im)
}

#[inline(always)]
fn complex_scale(value: Complex32, scale: f32) -> Complex32 {
    Complex32::new(value.re * scale, value.im * scale)
}

/// Rotate by -i for forward transforms and +i for inverse transforms.
#[inline(always)]
fn rotate_i(value: Complex32, direction: TransformDirection) -> Complex32 {
    match direction {
        TransformDirection::Forward => Complex32::new(value.im, -value.re),
        TransformDirection::Inverse => Complex32::new(-value.im, value.re),
    }
}

#[cfg(test)]
mod tests {
    use super::{Complex32, KissFft};
    use core::f32::consts::PI;

    /// Compute libopus-style forward DFT with `1/N` scaling for tests.
    ///
    /// Params: `input` is time-domain complex vector.
    /// Returns: scaled frequency-domain vector.
    fn forward_scaled(input: &[Complex32]) -> Vec<Complex32> {
        let n = input.len();
        let mut out = vec![Complex32::new(0.0, 0.0); n];
        for (k, yk) in out.iter_mut().enumerate() {
            let mut acc_re = 0.0f32;
            let mut acc_im = 0.0f32;
            for (n_idx, xn) in input.iter().enumerate() {
                let phase = -2.0 * PI * (k as f32) * (n_idx as f32) / (n as f32);
                let c = phase.cos();
                let s = phase.sin();
                acc_re += xn.re * c - xn.im * s;
                acc_im += xn.re * s + xn.im * c;
            }
            *yk = Complex32::new(acc_re / (n as f32), acc_im / (n as f32));
        }
        out
    }

    /// Compute an accurate unscaled forward DFT for FFT regression tests.
    fn forward_unscaled_f64(input: &[Complex32]) -> Vec<Complex32> {
        let n = input.len();
        let mut out = vec![Complex32::new(0.0, 0.0); n];
        for (frequency, value) in out.iter_mut().enumerate() {
            let mut sum_re = 0.0f64;
            let mut sum_im = 0.0f64;
            for (sample, input_value) in input.iter().enumerate() {
                let phase =
                    -2.0 * core::f64::consts::PI * frequency as f64 * sample as f64 / n as f64;
                let (sin_phase, cos_phase) = phase.sin_cos();
                sum_re += input_value.re as f64 * cos_phase - input_value.im as f64 * sin_phase;
                sum_im += input_value.re as f64 * sin_phase + input_value.im as f64 * cos_phase;
            }
            *value = Complex32::new(sum_re as f32, sum_im as f32);
        }
        out
    }

    #[test]
    fn ifft_roundtrip_matches_input() {
        let n = 60usize;
        let fft = KissFft::new(n);
        let mut input = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f32 / n as f32;
            input.push(Complex32::new(
                (2.0 * PI * 3.0 * t).sin(),
                (2.0 * PI * 5.0 * t).cos(),
            ));
        }
        let freq = forward_scaled(&input);
        let mut recon = vec![Complex32::new(0.0, 0.0); n];
        fft.ifft(&freq, &mut recon).expect("ifft must succeed");
        for (a, b) in input.iter().zip(recon.iter()) {
            assert!(
                (a.re - b.re).abs() < 2e-5,
                "re mismatch: {} vs {}",
                a.re,
                b.re
            );
            assert!(
                (a.im - b.im).abs() < 2e-5,
                "im mismatch: {} vs {}",
                a.im,
                b.im
            );
        }
    }

    #[test]
    fn fft_matches_direct_dft_for_all_celt_plan_sizes() {
        for n in [60usize, 120, 240, 480] {
            let fft = KissFft::new(n);
            let input: Vec<Complex32> = (0..n)
                .map(|index| {
                    let t = index as f32 / n as f32;
                    Complex32::new(
                        (2.0 * PI * 3.0 * t).sin() + 0.25 * (2.0 * PI * 7.0 * t).cos(),
                        0.5 * (2.0 * PI * 5.0 * t).cos(),
                    )
                })
                .collect();
            let expected = forward_unscaled_f64(&input);
            let mut actual = vec![Complex32::new(0.0, 0.0); n];
            fft.fft(&input, &mut actual).expect("fft must succeed");

            for (expected_value, actual_value) in expected.iter().zip(actual.iter()) {
                assert!(
                    (expected_value.re - actual_value.re).abs() < 2e-3,
                    "n={n} re mismatch: {} vs {}",
                    expected_value.re,
                    actual_value.re
                );
                assert!(
                    (expected_value.im - actual_value.im).abs() < 2e-3,
                    "n={n} im mismatch: {} vs {}",
                    expected_value.im,
                    actual_value.im
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "CELT FFT length must factor")]
    fn rejects_zero_length_plan() {
        let _ = KissFft::new(0);
    }

    #[test]
    #[should_panic(expected = "CELT FFT length must factor")]
    fn rejects_unsupported_radix_plan() {
        let _ = KissFft::new(7);
    }
}
