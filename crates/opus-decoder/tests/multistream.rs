use opus_decoder::{Decoder, OpusMultistreamDecoder};
use std::fs;
use std::io::{self, Read};
use std::path::{Path, PathBuf};

/// Minimal `opus_demo` packet record used by multistream tests.
#[derive(Debug, Clone)]
struct DemoPacket {
    /// Encoded payload bytes, or `None` for PLC records.
    payload: Option<Vec<u8>>,
}

/// Verify the call-recording use case with two independent mono streams.
///
/// Parameters: none.
/// Returns: nothing; panics on invalid mapping or decode behavior.
#[test]
fn multistream_stereo_call_recording() {
    let packets = read_opus_demo_stream(&vectors_dir().join("testvector07.bit")).unwrap();
    let left = packets[0].payload.as_deref().unwrap();
    let right = packets[1].payload.as_deref().unwrap();
    let packet = build_multistream_packet(&[left, right]);

    let mut decoder = OpusMultistreamDecoder::new(48_000, 2, 2, 0, &[0, 1]).unwrap();
    let mut pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K * 2];
    let samples = decoder.decode(&packet, &mut pcm, false).unwrap();

    assert!(samples > 0);
    assert!(
        pcm[..samples * 2]
            .chunks_exact(2)
            .any(|frame| frame[0] != frame[1])
    );
}

/// Verify that mapping value `255` silences the corresponding output channel.
///
/// Parameters: none.
/// Returns: nothing; panics on invalid output.
#[test]
fn multistream_silence_channel() {
    let packets = read_opus_demo_stream(&vectors_dir().join("testvector07.bit")).unwrap();
    let packet = packets[0].payload.as_deref().unwrap();

    let mut decoder = OpusMultistreamDecoder::new(48_000, 2, 1, 0, &[0, 255]).unwrap();
    let mut pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K * 2];
    let samples = decoder.decode(packet, &mut pcm, false).unwrap();

    assert!(samples > 0);
    assert!(pcm[..samples * 2].chunks_exact(2).all(|frame| frame[1] == 0));
}

/// Reject invalid multistream topology arguments.
///
/// Parameters: none.
/// Returns: nothing; panics on unexpected success.
#[test]
fn multistream_invalid_args() {
    assert!(OpusMultistreamDecoder::new(48_000, 2, 1, 2, &[0, 1]).is_err());
    assert!(OpusMultistreamDecoder::new(48_000, 2, 1, 0, &[0]).is_err());
    assert!(OpusMultistreamDecoder::new(48_000, 2, 1, 0, &[0, 2]).is_err());
}

/// Build a synthetic multistream packet with self-delimited size prefixes.
///
/// Parameters: elementary Opus `sub_packets` in stream order.
/// Returns: concatenated multistream packet bytes.
fn build_multistream_packet(sub_packets: &[&[u8]]) -> Vec<u8> {
    let mut packet = Vec::new();
    for (stream_idx, sub_packet) in sub_packets.iter().enumerate() {
        if stream_idx + 1 != sub_packets.len() {
            packet.extend(encode_self_delimited_size(sub_packet.len()));
        }
        packet.extend_from_slice(sub_packet);
    }
    packet
}

/// Encode one multistream self-delimited sub-packet length.
///
/// Parameters: sub-packet byte `len`.
/// Returns: one- or two-byte self-delimited size field.
fn encode_self_delimited_size(len: usize) -> Vec<u8> {
    if len < 252 {
        vec![len as u8]
    } else {
        let remainder = len - 252;
        vec![(252 + (remainder & 0x03)) as u8, (remainder >> 2) as u8]
    }
}

/// Resolve the local Opus vector directory used by multistream tests.
///
/// Parameters: none.
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

/// Read an `opus_demo` `.bit` stream into packet payloads.
///
/// Parameters: absolute `path` to the `.bit` vector stream.
/// Returns: ordered packet list with payload bytes.
fn read_opus_demo_stream(path: &Path) -> io::Result<Vec<DemoPacket>> {
    let mut file = fs::File::open(path)?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;

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
        index += 4; // expected final_range
        let payload = if packet_len == 0 {
            None
        } else {
            Some(bytes[index..index + packet_len].to_vec())
        };
        index += packet_len;
        packets.push(DemoPacket { payload });
    }

    Ok(packets)
}
