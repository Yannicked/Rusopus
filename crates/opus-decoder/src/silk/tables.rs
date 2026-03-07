//! Minimal SILK entropy tables needed for Phase 3a bit parsing.

/// Joint iCDF for stereo predictor coarse index (`silk_stereo_pred_joint_iCDF`).
pub(super) const STEREO_PRED_JOINT_ICDF: [u8; 25] = [
    249, 247, 246, 245, 244, 234, 210, 202, 201, 200, 197, 174, 82, 59, 56, 55, 54, 46, 22, 12, 11,
    10, 9, 7, 0,
];

/// Uniform 3-way iCDF (`silk_uniform3_iCDF`).
pub(super) const UNIFORM3_ICDF: [u8; 3] = [171, 85, 0];

/// Uniform 5-way iCDF (`silk_uniform5_iCDF`).
pub(super) const UNIFORM5_ICDF: [u8; 5] = [205, 154, 102, 51, 0];

/// Mid-only stereo flag iCDF (`silk_stereo_only_code_mid_iCDF`).
pub(super) const STEREO_ONLY_CODE_MID_ICDF: [u8; 2] = [64, 0];

/// Stereo predictor dequantization table (`silk_stereo_pred_quant_Q13`).
pub(super) const STEREO_PRED_QUANT_Q13: [i32; 16] = [
    -13_732, -10_050, -8_266, -7_526, -6_500, -5_000, -2_950, -820, 820, 2_950, 5_000, 6_500,
    7_526, 8_266, 10_050, 13_732,
];

/// LBRR-flag iCDF for 2 internal SILK frames (`silk_LBRR_flags_2_iCDF`).
pub(super) const LBRR_FLAGS_2_ICDF: [u8; 3] = [203, 150, 0];

/// LBRR-flag iCDF for 3 internal SILK frames (`silk_LBRR_flags_3_iCDF`).
pub(super) const LBRR_FLAGS_3_ICDF: [u8; 7] = [215, 195, 166, 125, 110, 82, 0];

/// Return LBRR iCDF table for given internal frame count.
///
/// Params: `internal_frames` is number of 20 ms SILK-internal frames in one Opus frame.
/// Returns: table slice for 2/3-frame packets, or `None` for unsupported counts.
pub(super) fn lbrr_flags_icdf(internal_frames: usize) -> Option<&'static [u8]> {
    match internal_frames {
        2 => Some(&LBRR_FLAGS_2_ICDF),
        3 => Some(&LBRR_FLAGS_3_ICDF),
        _ => None,
    }
}
