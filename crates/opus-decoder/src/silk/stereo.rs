//! SILK stereo signaling parser helpers for Phase 3a.

use crate::entropy::EcDec;

use super::tables;

const STEREO_QUANT_SUB_STEPS: i32 = 5;
const STEREO_STEP_Q16: i32 = 6_554; // SILK_FIX_CONST(0.5 / 5, 16)

/// Decode SILK stereo predictor signaling (`silk_stereo_decode_pred`).
///
/// Params: `dec` is the shared Opus range decoder over SILK frame payload.
/// Returns: two MS predictor values in Q13.
pub(super) fn decode_stereo_pred(dec: &mut EcDec<'_>) -> [i32; 2] {
    let mut ix = [[0i32; 3]; 2];
    let joint = dec.dec_icdf(&tables::STEREO_PRED_JOINT_ICDF, 8);
    ix[0][2] = joint / 5;
    ix[1][2] = joint - 5 * ix[0][2];

    for side in &mut ix {
        side[0] = dec.dec_icdf(&tables::UNIFORM3_ICDF, 8);
        side[1] = dec.dec_icdf(&tables::UNIFORM5_ICDF, 8);
    }

    let mut pred_q13 = [0i32; 2];
    for (n, side) in ix.iter_mut().enumerate() {
        side[0] += 3 * side[2];
        let low_q13 = tables::STEREO_PRED_QUANT_Q13[side[0] as usize];
        let high_q13 = tables::STEREO_PRED_QUANT_Q13[side[0] as usize + 1];
        let step_q13 = mul_wb(high_q13 - low_q13, STEREO_STEP_Q16);
        pred_q13[n] = low_q13 + step_q13 * (2 * side[1] + 1) / STEREO_QUANT_SUB_STEPS;
    }

    pred_q13[0] -= pred_q13[1];
    pred_q13
}

/// Decode SILK "mid-only coded" stereo flag (`silk_stereo_decode_mid_only`).
///
/// Params: `dec` is the shared Opus range decoder over SILK frame payload.
/// Returns: true when side channel is omitted for this frame.
pub(super) fn decode_mid_only(dec: &mut EcDec<'_>) -> bool {
    dec.dec_icdf(&tables::STEREO_ONLY_CODE_MID_ICDF, 8) != 0
}

/// Multiply helper matching SILK `silk_SMULWB` behavior.
///
/// Params: `a` and `b` are signed fixed-point values.
/// Returns: high-word product `(a*b)>>16`.
fn mul_wb(a: i32, b: i32) -> i32 {
    ((a as i64 * b as i64) >> 16) as i32
}
