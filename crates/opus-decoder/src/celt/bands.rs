//! CELT band decode (`bands.c`) for mono decode-only vertical slice.

use crate::celt::modes::CeltMode;
use crate::celt::quant_bands;
use crate::celt::rate;
use crate::celt::vq;
use crate::entropy::EcDec;
use core::sync::atomic::{AtomicUsize, Ordering};

const BITRES: i32 = 3;
const QTHETA_OFFSET: i32 = 4;
const SPREAD_AGGRESSIVE: i32 = 3;
const BIT_INTERLEAVE_TABLE: [u32; 16] = [0, 1, 1, 1, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3];
const EXP2_TABLE8: [i32; 8] = [16384, 17866, 19483, 21247, 23170, 25267, 27554, 30048];
static TRACE_BANDS_CALL_IDX: AtomicUsize = AtomicUsize::new(0);

/// CELT linear congruential generator used for folded/noise fill.
///
/// Params: current seed.
/// Returns: next pseudo-random seed.
pub(crate) fn celt_lcg_rand(seed: u32) -> u32 {
    seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223)
}

/// Integer square-root for non-negative 32-bit values.
///
/// Params: input integer value.
/// Returns: floor(sqrt(value)).
fn isqrt32(v: u32) -> u32 {
    if v == 0 {
        return 0;
    }
    let mut x = v;
    let mut y = (x + 1) >> 1;
    while y < x {
        x = y;
        y = (x + v / x) >> 1;
    }
    x
}

/// Integer ilog helper equivalent to CELT `EC_ILOG`.
///
/// Params: positive integer value.
/// Returns: floor(log2(v)) + 1.
fn ec_ilog(v: i32) -> i32 {
    debug_assert!(v > 0);
    32 - (v as u32).leading_zeros() as i32
}

/// Bit-exact fractional multiply equivalent to `FRAC_MUL16`.
///
/// Params: signed 16-bit fixed point multiplicands.
/// Returns: rounded product shifted by 15.
fn frac_mul16(a: i32, b: i32) -> i32 {
    (16384 + ((a as i16 as i32) * (b as i16 as i32))) >> 15
}

/// Bit-exact cosine approximation from libopus `bands.c`.
///
/// Params: angle in Q14 domain [0, 16384].
/// Returns: cosine approximation in Q15-ish integer domain.
fn bitexact_cos(x: i32) -> i32 {
    let tmp = (4096 + x * x) >> 13;
    let x2 = tmp;
    1 + (32767 - x2) + frac_mul16(x2, -7651 + frac_mul16(x2, 8277 + frac_mul16(-626, x2)))
}

/// Bit-exact log2(tan()) approximation for split bit rebalancing.
///
/// Params: side and mid cosine terms.
/// Returns: fixed-point log2(tan()) approximation.
fn bitexact_log2tan(isin: i32, icos: i32) -> i32 {
    let lc = ec_ilog(icos.max(1));
    let ls = ec_ilog(isin.max(1));
    let icos_n = icos << (15 - lc);
    let isin_n = isin << (15 - ls);
    (ls - lc) * (1 << 11)
        + frac_mul16(isin_n, frac_mul16(isin_n, -2597) + 7932)
        - frac_mul16(icos_n, frac_mul16(icos_n, -2597) + 7932)
}

/// Compute theta quantization resolution for recursive split.
///
/// Params: partition size `n`, bit budget `b`, split `offset`, pulse cap and stereo flag.
/// Returns: theta resolution value `qn`.
fn compute_qn(n: usize, b: i32, offset: i32, pulse_cap: i32, stereo: bool) -> i32 {
    let mut n2 = 2 * n as i32 - 1;
    if stereo && n == 2 {
        n2 -= 1;
    }
    let mut qb = (b + n2 * offset) / n2;
    qb = qb.min(b - pulse_cap - (4 << BITRES));
    qb = qb.min(8 << BITRES);
    if qb < (1 << BITRES >> 1) {
        1
    } else {
        let mut qn = EXP2_TABLE8[(qb & 0x7) as usize] >> (14 - (qb >> BITRES));
        qn = ((qn + 1) >> 1) << 1;
        qn.min(256)
    }
}

/// Decode split theta for mono recursive partition.
///
/// Params: mode, band index and split context (`n`, `b`, `blocks`, `fill`).
/// Returns: `(delta, qalloc, itheta, imid, iside)` for recursive split.
#[allow(clippy::too_many_arguments)]
fn decode_theta_mono(
    mode: &CeltMode,
    band: usize,
    dec: &mut EcDec<'_>,
    n: usize,
    b: &mut i32,
    blocks: usize,
    blocks0: usize,
    lm: i32,
    fill: &mut u32,
) -> (i32, i32, i32, i32, i32) {
    let pulse_cap = mode.log_n[band] as i32 + lm * (1 << BITRES);
    let offset = (pulse_cap >> 1) - QTHETA_OFFSET;
    let qn = compute_qn(n, *b, offset, pulse_cap, false);
    let tell_before = dec.tell_frac() as i32;
    let mut itheta = 0i32;
    if qn != 1 {
        if blocks0 > 1 {
            itheta = dec.dec_uint((qn + 1) as u32) as i32;
        } else {
            let ft = ((qn >> 1) + 1) * ((qn >> 1) + 1);
            let fm = dec.decode(ft as u32) as i32;
            let (fl, fs, ith) = if fm < ((qn >> 1) * ((qn >> 1) + 1) >> 1) {
                let ith = ((isqrt32((8 * fm + 1) as u32) as i32) - 1) >> 1;
                let fs = ith + 1;
                let fl = ith * (ith + 1) >> 1;
                (fl, fs, ith)
            } else {
                let ith = (2 * (qn + 1) - (isqrt32((8 * (ft - fm - 1) + 1) as u32) as i32)) >> 1;
                let fs = qn + 1 - ith;
                let fl = ft - ((qn + 1 - ith) * (qn + 2 - ith) >> 1);
                (fl, fs, ith)
            };
            dec.update(fl as u32, (fl + fs) as u32, ft as u32);
            itheta = ith;
        }
        itheta = (itheta * 16384) / qn;
    }
    let qalloc = dec.tell_frac() as i32 - tell_before;
    *b -= qalloc;

    if itheta == 0 {
        *fill &= (1u32 << blocks) - 1;
        return (-16384, qalloc, itheta, 32767, 0);
    }
    if itheta == 16384 {
        *fill &= ((1u32 << blocks) - 1) << blocks;
        return (16384, qalloc, itheta, 0, 32767);
    }
    let imid = bitexact_cos(itheta);
    let iside = bitexact_cos(16384 - itheta);
    let delta = frac_mul16((n as i32 - 1) << 7, bitexact_log2tan(iside, imid));
    (delta, qalloc, itheta, imid, iside)
}

/// Decode one mono partition with recursive split.
///
/// Params: mode, target partition, band index and recursive coding context.
/// Returns: collapse mask for this partition.
#[allow(clippy::too_many_arguments)]
fn quant_partition_mono(
    mode: &CeltMode,
    x: &mut [f32],
    band_idx: usize,
    mut b: i32,
    mut blocks: usize,
    lowband: Option<&[f32]>,
    mut lm: i32,
    gain: f32,
    mut fill: u32,
    spread: i32,
    dec: &mut EcDec<'_>,
    seed: &mut u32,
    remaining_bits: &mut i32,
    trace_this_packet: bool,
    packet_idx: usize,
) -> u32 {
    let n0 = x.len();
    let blocks0 = blocks;
    if n0 == 1 {
        if *remaining_bits >= (1 << BITRES) {
            let sign = dec.dec_bits(1);
            *remaining_bits -= 1 << BITRES;
            x[0] = if sign != 0 { -1.0 } else { 1.0 };
        } else {
            x[0] = 1.0;
        }
        if trace_this_packet && band_idx == 0 {
            // #region agent log
            eprintln!(
                "R pkt{} qpart n1: rem_bits={} tell={} x0={}",
                packet_idx,
                *remaining_bits,
                dec.tell_frac(),
                x[0]
            );
            // #endregion
        }
        return 1;
    }
    if trace_this_packet && band_idx == 12 {
        // #region agent log
        eprintln!(
            "R pkt{} qpart enter: N={} b={} B={} lm={} rem_bits={} tell={}",
            packet_idx,
            n0,
            b,
            blocks,
            lm,
            *remaining_bits,
            dec.tell_frac()
        );
        // #endregion
    }
    let cache_row = ((lm + 1) as usize)
        .saturating_mul(mode.nb_ebands)
        .saturating_add(band_idx);
    let do_split = if lm != -1 && n0 > 2 && cache_row < mode.cache.index.len() {
        let base = mode.cache.index[cache_row];
        if base >= 0 {
            let cache = &mode.cache.bits[base as usize..];
            b > cache[cache[0] as usize] as i32 + 12
        } else {
            false
        }
    } else {
        false
    };
    if do_split {
        let n = n0 >> 1;
        let (x_lo, x_hi) = x.split_at_mut(n);
        lm -= 1;
        if blocks == 1 {
            fill = (fill & 1) | (fill << 1);
        }
        blocks = (blocks + 1) >> 1;
        let (mut delta, qalloc, itheta, imid, iside) = decode_theta_mono(
            mode, band_idx, dec, n, &mut b, blocks, blocks0, lm, &mut fill,
        );
        let mid = (imid as f32) * (1.0 / 32768.0);
        let side = (iside as f32) * (1.0 / 32768.0);
        if blocks0 > 1 && (itheta & 0x3fff) != 0 {
            if itheta > 8192 {
                delta -= delta >> (4 - lm);
            } else {
                delta = 0.min(delta + (((n as i32) << BITRES) >> (5 - lm)));
            }
        }
        let mut mbits = 0.max(b.min((b - delta) / 2));
        let mut sbits = b - mbits;
        *remaining_bits -= qalloc;
        if trace_this_packet && band_idx == 12 {
            // #region agent log
            eprintln!(
                "R pkt{} qpart split: N={} n={} qalloc={} itheta={} delta={} mbits={} sbits={} rem_bits={} tell={}",
                packet_idx,
                n0,
                n,
                qalloc,
                itheta,
                delta,
                mbits,
                sbits,
                *remaining_bits,
                dec.tell_frac()
            );
            // #endregion
        }
        let next_lowband2 = lowband.map(|lb| {
            if lb.len() > n {
                &lb[n..]
            } else {
                &lb[0..0]
            }
        });
        let mut rebalance = *remaining_bits;
        if mbits >= sbits {
            let mut cm = quant_partition_mono(
                mode,
                x_lo,
                band_idx,
                mbits,
                blocks,
                lowband.map(|lb| &lb[..lb.len().min(n)]),
                lm,
                gain * mid,
                fill,
                spread,
                dec,
                seed,
                remaining_bits,
                trace_this_packet,
                packet_idx,
            );
            rebalance = mbits - (rebalance - *remaining_bits);
            if rebalance > (3 << BITRES) && itheta != 0 {
                sbits += rebalance - (3 << BITRES);
            }
            cm |= quant_partition_mono(
                mode,
                x_hi,
                band_idx,
                sbits,
                blocks,
                next_lowband2,
                lm,
                gain * side,
                fill >> blocks,
                spread,
                dec,
                seed,
                remaining_bits,
                trace_this_packet,
                packet_idx,
            ) << (blocks0 >> 1);
            return cm;
        }

        let mut cm = quant_partition_mono(
            mode,
            x_hi,
            band_idx,
            sbits,
            blocks,
            next_lowband2,
            lm,
            gain * side,
            fill >> blocks,
            spread,
            dec,
            seed,
            remaining_bits,
            trace_this_packet,
            packet_idx,
        ) << (blocks0 >> 1);
        rebalance = sbits - (rebalance - *remaining_bits);
        if rebalance > (3 << BITRES) && itheta != 16384 {
            mbits += rebalance - (3 << BITRES);
        }
        cm |= quant_partition_mono(
            mode,
            x_lo,
            band_idx,
            mbits,
            blocks,
            lowband.map(|lb| &lb[..lb.len().min(n)]),
            lm,
            gain * mid,
            fill,
            spread,
            dec,
            seed,
            remaining_bits,
            trace_this_packet,
            packet_idx,
        );
        return cm;
    }

    let mut q = rate::bits2pulses(mode, band_idx, lm, b.max(0));
    let mut curr_bits = rate::pulses2bits(mode, band_idx, lm, q);
    *remaining_bits -= curr_bits;
    while *remaining_bits < 0 && q > 0 {
        *remaining_bits += curr_bits;
        q -= 1;
        curr_bits = rate::pulses2bits(mode, band_idx, lm, q);
        *remaining_bits -= curr_bits;
    }
    if trace_this_packet && band_idx == 12 {
        // #region agent log
        eprintln!(
            "R pkt{} qpart nosplit: N={} b={} q={} curr_bits={} rem_bits={} tell={}",
            packet_idx,
            n0,
            b,
            q,
            curr_bits,
            *remaining_bits,
            dec.tell_frac()
        );
        // #endregion
    }
    if q != 0 {
        let k = rate::get_pulses(q);
        return vq::alg_unquant(x, k, spread, blocks, band_idx, false, dec, gain).collapse_mask as u32;
    }

    let cm_mask = if blocks >= 32 {
        u32::MAX
    } else {
        (1u32 << blocks) - 1
    };
    let fill_masked = fill & cm_mask;
    if fill_masked == 0 {
        x.fill(0.0);
        return 0;
    }
    if let Some(lb) = lowband {
        if lb.is_empty() {
            for v in x.iter_mut() {
                *seed = celt_lcg_rand(*seed);
                *v = ((*seed as i32) >> 20) as f32;
            }
            vq::renormalise_vector(x, gain);
            return cm_mask;
        }
        for (j, v) in x.iter_mut().enumerate() {
            *seed = celt_lcg_rand(*seed);
            let noise = if (*seed & 0x8000) != 0 { 1.0 / 256.0 } else { -1.0 / 256.0 };
            *v = lb[j % lb.len()] + noise;
        }
        vq::renormalise_vector(x, gain);
        return fill_masked;
    }
    for v in x.iter_mut() {
        *seed = celt_lcg_rand(*seed);
        *v = ((*seed as i32) >> 20) as f32;
    }
    vq::renormalise_vector(x, gain);
    cm_mask
}

/// Decode all CELT bands in mono decode-only mode.
///
/// Params: CELT mode, active band range, output normalized spectrum `x`, per-band
/// pulse allocation, transient flag, spread mode, TF flags, total bit budget,
/// running balance, coded band limit, decoder state and random seed.
/// Returns: collapse mask per band.
#[allow(clippy::too_many_arguments)]
pub(crate) fn quant_all_bands_mono(
    mode: &CeltMode,
    start: usize,
    end: usize,
    x: &mut [f32],
    pulses: &[i32],
    short_blocks: bool,
    spread: i32,
    tf_res: &[i32],
    total_bits_q: i32,
    mut balance: i32,
    coded_bands: usize,
    lm: usize,
    dec: &mut EcDec<'_>,
    seed: &mut u32,
) -> Vec<u8> {
    let packet_idx = TRACE_BANDS_CALL_IDX.fetch_add(1, Ordering::SeqCst);
    let trace_target = std::env::var("OPUS_TRACE_PACKET_IDX")
        .ok()
        .and_then(|s| s.parse::<usize>().ok());
    let trace_this_packet = trace_target == Some(packet_idx);
    let m = 1usize << lm;
    let mut masks = vec![0u8; mode.nb_ebands];
    let mut lowband_offset = 0usize;
    let mut update_lowband = true;
    let norm_offset = m * mode.e_bands[start] as usize;
    for i in start..end {
        let tell = dec.tell_frac() as i32;
        if i != start {
            balance -= tell;
        }
        let remaining_bits = total_bits_q - tell - 1;
        let b = if i < coded_bands {
            let curr_balance = balance / ((coded_bands - i).min(3) as i32);
            (pulses[i] + curr_balance).clamp(0, 16_383).min(remaining_bits + 1)
        } else {
            0
        };
        let band_start = m * mode.e_bands[i] as usize;
        let band_end = m * mode.e_bands[i + 1] as usize;
        let n = band_end - band_start;
        if i != start && (update_lowband || lowband_offset == 0) {
            lowband_offset = i;
        }
        let frame_blocks = if short_blocks { m } else { 1 };
        let mut b_blocks = frame_blocks;
        let spread_eff = (spread + tf_res[i]).clamp(0, 3);
        let tf_change = tf_res[i];
        let mut n_b = n / b_blocks.max(1);
        let mut fill = if b_blocks >= 32 {
            u32::MAX
        } else {
            (1u32 << b_blocks) - 1
        };
        let recombine = tf_change.max(0) as usize;
        for _ in 0..recombine {
            let lo = (fill & 0xF) as usize;
            let hi = ((fill >> 4) & 0xF) as usize;
            fill = BIT_INTERLEAVE_TABLE[lo] | (BIT_INTERLEAVE_TABLE[hi] << 2);
        }
        b_blocks >>= recombine;
        n_b <<= recombine;
        let mut tf_for_blocks = tf_change;
        while (n_b & 1) == 0 && tf_for_blocks < 0 {
            if b_blocks < 32 {
                fill |= fill << b_blocks;
            } else {
                fill = u32::MAX;
            }
            b_blocks <<= 1;
            n_b >>= 1;
            tf_for_blocks += 1;
        }
        if trace_this_packet {
            eprintln!(
                "R pkt{} band {} params: b={} N={} B={} fill=0x{:x} lowband_offset={} update_lowband={} spread_eff={} tf_change={}",
                packet_idx, i, b, n, b_blocks, fill, lowband_offset, update_lowband, spread_eff, tf_change
            );
        }
        let mut remaining_band_bits = remaining_bits;
        let lowband_range = if lowband_offset != 0
            && (spread != SPREAD_AGGRESSIVE || frame_blocks > 1 || tf_change < 0)
        {
            let lowband_start = m * mode.e_bands[lowband_offset] as usize;
            let effective_lowband = lowband_start.saturating_sub(norm_offset + n);
            let fold_low = effective_lowband + norm_offset;
            let fold_high = fold_low + n;

            let mut fold_start = lowband_offset;
            while fold_start > 0 && m * mode.e_bands[fold_start - 1] as usize > fold_low {
                fold_start -= 1;
            }

            let mut fold_end = lowband_offset.saturating_sub(1);
            while fold_end + 1 < i && m * (mode.e_bands[fold_end + 1] as usize) < fold_high {
                fold_end += 1;
            }

            if fold_end >= fold_start {
                let fold_count = fold_end - fold_start + 1;
                let fold_pos = (i - 1).saturating_sub(fold_start);
                let src = fold_start + (fold_pos % fold_count);
                let src_start = m * mode.e_bands[src] as usize;
                let src_end = m * mode.e_bands[src + 1] as usize;
                if src_end <= band_start && src_end > src_start {
                    Some((src_start, src_end))
                } else {
                    None
                }
            } else {
                None
            }
        } else {
            None
        };
        let (x_before, x_after) = x.split_at_mut(band_start);
        let lowband = lowband_range.and_then(|(src_start, src_end)| x_before.get(src_start..src_end));
        let tell_before = dec.tell_frac();
        let cm = quant_partition_mono(
            mode,
            &mut x_after[..n],
            i,
            b,
            b_blocks,
            lowband,
            lm as i32,
            1.0,
            fill,
            spread_eff,
            dec,
            seed,
            &mut remaining_band_bits,
            trace_this_packet,
            packet_idx,
        );
        let tell_after = dec.tell_frac();
        if trace_this_packet {
            eprintln!(
                "pkt{} band {} tell: {}->{} ({}bits)",
                packet_idx,
                i,
                tell_before,
                tell_after,
                tell_after as i32 - tell_before as i32
            );
        }
        masks[i] = (cm & 0xFF) as u8;
        balance += pulses[i] + tell;
        update_lowband = b > ((n as i32) << BITRES);
    }
    masks
}

/// De-normalize normalized bands to MDCT bins using decoded energies.
///
/// Params: mode, normalized `x`, output `freq`, decoded `band_loge` and band range.
/// Returns: nothing.
pub(crate) fn denormalise_bands(
    mode: &CeltMode,
    x: &[f32],
    freq: &mut [f32],
    band_loge: &[f32],
    start: usize,
    end: usize,
    lm: usize,
) {
    let m = 1usize << lm;
    for i in start..end {
        let j0 = m * mode.e_bands[i] as usize;
        let j1 = m * mode.e_bands[i + 1] as usize;
        let lg = band_loge[i] + quant_bands::e_means()[i];
        let g = 2.0f32.powf(lg.max(-32.0));
        for j in j0..j1 {
            freq[j] = x[j] * g;
        }
    }
}

