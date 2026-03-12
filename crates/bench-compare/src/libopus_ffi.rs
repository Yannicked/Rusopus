//! Safe FFI wrapper around libopus `opus_decode`.

use std::ffi::c_int;

#[repr(C)]
struct OpusDecoder {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn opus_decoder_get_size(channels: c_int) -> c_int;
    fn opus_decoder_init(st: *mut OpusDecoder, fs: i32, channels: c_int) -> c_int;
    fn opus_decode(
        st: *mut OpusDecoder,
        data: *const u8,
        len: i32,
        pcm: *mut i16,
        frame_size: c_int,
        decode_fec: c_int,
    ) -> c_int;
}

/// Owns one libopus decoder state buffer and exposes a safe decode method.
pub struct LibopusDecoder {
    state: Vec<u8>,
    channels: usize,
}

impl LibopusDecoder {
    /// Create a new libopus decoder for the given output configuration.
    ///
    /// Parameters: output `sample_rate` and number of `channels`.
    /// Returns: initialized decoder state ready for `decode()`.
    pub fn new(sample_rate: u32, channels: usize) -> Self {
        let size = unsafe { opus_decoder_get_size(channels as c_int) };
        assert!(size > 0, "opus_decoder_get_size failed: {size}");

        let mut state = vec![0u8; size as usize];
        let ret = unsafe {
            opus_decoder_init(
                state.as_mut_ptr().cast::<OpusDecoder>(),
                sample_rate as i32,
                channels as c_int,
            )
        };
        assert_eq!(ret, 0, "opus_decoder_init failed: {ret}");

        Self { state, channels }
    }

    /// Decode one Opus packet to interleaved i16 PCM using libopus.
    ///
    /// Parameters: raw Opus `packet` bytes and writable interleaved `pcm`.
    /// Returns: samples per channel, or a negative libopus error code.
    pub fn decode(&mut self, packet: &[u8], pcm: &mut [i16]) -> i32 {
        let frame_size = (pcm.len() / self.channels) as c_int;
        let data = if packet.is_empty() {
            std::ptr::null()
        } else {
            packet.as_ptr()
        };

        unsafe {
            opus_decode(
                self.state.as_mut_ptr().cast::<OpusDecoder>(),
                data,
                packet.len() as i32,
                pcm.as_mut_ptr(),
                frame_size,
                0,
            )
        }
    }
}
