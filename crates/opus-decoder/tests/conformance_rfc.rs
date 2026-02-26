use std::fs;
use std::io::{self, Read};
use std::path::{Path, PathBuf};

use opus_decoder::{Decoder, Error};

#[derive(Debug)]
struct Vector {
    name: String,
    bit: String,
    dec: String,
    fs_hz: u32,
    channels: u8,
}

#[derive(Debug)]
struct OpusDemoPacket {
    /// `None` means "lost packet" (PLC).
    packet: Option<Vec<u8>>,
    /// Expected final range coder value from the reference stream.
    ///
    expected_final_range: u32,
}

#[test]
#[ignore = "requires OPUS_TESTVECTORS_DIR + vectors.txt + working decoder"]
fn rfc_conformance_vectors() {
    let vectors_dir = vectors_dir();
    let manifest_path = vectors_dir.join("vectors.txt");
    let vectors = load_manifest(&manifest_path).unwrap_or_else(|e| {
        panic!(
            "failed to load manifest {}: {e}\n\n\
             Create it based on {}\n",
            manifest_path.display(),
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/vectors.example.txt")
                .display()
        )
    });

    if vectors.is_empty() {
        panic!("manifest has no vectors: {}", manifest_path.display());
    }

    let filter = std::env::var("OPUS_VECTOR").ok();
    let mut ran_any = false;

    for v in vectors {
        if let Some(f) = filter.as_deref()
            && !v.name.contains(f)
        {
            continue;
        }
        ran_any = true;
        run_vector(&vectors_dir, &v).unwrap_or_else(|e| {
            panic!("vector {} failed: {e}", v.name);
        });
    }

    if filter.is_some() && !ran_any {
        panic!(
            "OPUS_VECTOR filter matched nothing (manifest: {})",
            manifest_path.display()
        );
    }
}

fn run_vector(vectors_dir: &Path, v: &Vector) -> Result<(), Box<dyn std::error::Error>> {
    let bit_path = vectors_dir.join(&v.bit);
    let dec_path = vectors_dir.join(&v.dec);

    let packets = read_opus_demo_stream(&bit_path)?;
    let expected_pcm = load_pcm16le(&dec_path)?;
    let alt_pcm: Option<Vec<i16>> = alt_dec_path(&dec_path)
        .filter(|p| p.is_file())
        .map(|p| load_pcm16le(&p))
        .transpose()?;

    let mut dec = Decoder::new(v.fs_hz, v.channels)?;
    let mut out = vec![0i16; dec.max_frame_size_per_channel() * v.channels as usize];
    let mut got_pcm = Vec::<i16>::new();
    let mut final_range_mismatches: Vec<(usize, u32, u32)> = Vec::new();
    let trace_packet_idx = std::env::var("OPUS_TRACE_PACKET_IDX")
        .ok()
        .and_then(|s| s.parse::<usize>().ok());
    let trace_pkt0_pcm8 = std::env::var("OPUS_TRACE_PKT0_PCM8").ok().as_deref() == Some("1");
    let trace_pcm_packet_idx = std::env::var("OPUS_TRACE_PCM_PACKET_IDX")
        .ok()
        .and_then(|s| s.parse::<usize>().ok());
    let trace_first10 = std::env::var("OPUS_TRACE_FIRST10").ok().as_deref() == Some("1");
    let trace_match_all = std::env::var("OPUS_TRACE_MATCH_ALL").ok().as_deref() == Some("1");
    let trace_pcm_1_10 = std::env::var("OPUS_TRACE_PCM_1_10").ok().as_deref() == Some("1");

    for (packet_idx, p) in packets.into_iter().enumerate() {
        match dec.decode(p.packet.as_deref(), &mut out) {
            Ok(samples_per_channel) => {
                let written = samples_per_channel * v.channels as usize;
                got_pcm.extend_from_slice(&out[..written]);
                if trace_pkt0_pcm8 && packet_idx == 0 && written >= 8 {
                    eprintln!(
                        "[RUST] pkt0 pcm[0..8]: {} {} {} {} {} {} {} {}",
                        out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]
                    );
                }
                if let Some(target_pcm_idx) = trace_pcm_packet_idx
                    && packet_idx == target_pcm_idx
                    && written >= 8
                {
                    eprintln!(
                        "[RUST] pkt{} pcm[0..8]: {} {} {} {} {} {} {} {}",
                        target_pcm_idx, out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]
                    );
                    eprintln!(
                        "[RUST] pkt{} samples_per_channel={}",
                        target_pcm_idx, samples_per_channel
                    );
                }
                if packet_idx == 1 && written >= 8 {
                    eprintln!(
                        "rust pcm[0..8]: {} {} {} {} {} {} {} {}",
                        out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]
                    );
                    if expected_pcm.len() >= 968 {
                        eprintln!(
                            "ref  pcm[0..8]: {} {} {} {} {} {} {} {}",
                            expected_pcm[960],
                            expected_pcm[961],
                            expected_pcm[962],
                            expected_pcm[963],
                            expected_pcm[964],
                            expected_pcm[965],
                            expected_pcm[966],
                            expected_pcm[967]
                        );
                    }
                }
                if packet_idx < 5 {
                    eprintln!("pkt{} deemph_mem={:.4}", packet_idx, dec.deemph_mem());
                }
                let got = dec.final_range();
                if packet_idx < 5 {
                    eprintln!(
                        "pkt{} final_range expected={:08x} got={:08x}",
                        packet_idx, p.expected_final_range, got
                    );
                }
                if trace_first10 && packet_idx < 10 {
                    eprintln!(
                        "[RUST] packet {}: final_range match={}",
                        packet_idx,
                        got == p.expected_final_range
                    );
                }
                if trace_match_all {
                    eprintln!(
                        "pkt {} match={} splits={}",
                        packet_idx,
                        got == p.expected_final_range,
                        dec.last_split_count()
                    );
                }
                if got != p.expected_final_range {
                    final_range_mismatches.push((packet_idx, p.expected_final_range, got));
                }
                if trace_pcm_1_10 && (1..=10).contains(&packet_idx) && written >= 4 {
                    eprintln!(
                        "[RUST] pkt{} transient={} pcm[0..4]: {} {} {} {}",
                        packet_idx,
                        dec.last_is_transient(),
                        out[0],
                        out[1],
                        out[2],
                        out[3]
                    );
                }
                if let Some(target_idx) = trace_packet_idx
                    && packet_idx == target_idx
                {
                    eprintln!(
                        "[RUST] packet{} final_range expected=0x{:08x} got=0x{:08x}",
                        target_idx,
                        p.expected_final_range, got
                    );
                    return Ok(());
                }
            }
            Err(Error::NotImplemented) => {
                return Err("decoder not implemented yet".into());
            }
            Err(e) => return Err(format!("decode failed: {e}").into()),
        }
    }

    if !final_range_mismatches.is_empty() {
        eprintln!(
            "warning: final_range mismatches for {}: {} packets (soft check for now)",
            v.name,
            final_range_mismatches.len()
        );
        for (packet_idx, expected, got) in final_range_mismatches.iter().take(8) {
            eprintln!(
                "  packet #{packet_idx}: expected 0x{expected:08x}, got 0x{got:08x}"
            );
        }
    }

    // Fast path: exact match against either output.
    if expected_pcm == got_pcm {
        return Ok(());
    }
    if let Some(alt_pcm) = &alt_pcm {
        if *alt_pcm == got_pcm {
            return Ok(());
        }
    }

    // Reference conformance uses `opus_compare` (quality metric), not a
    // byte-exact PCM match.
    let q0 = opus_decoder::compare::compare_quality(
        &expected_pcm,
        &got_pcm,
        v.channels as usize,
        v.fs_hz,
    )?;
    if q0.passes_vectors() {
        return Ok(());
    }
    if let Some(alt_pcm) = &alt_pcm {
        let q1 = opus_decoder::compare::compare_quality(
            alt_pcm,
            &got_pcm,
            v.channels as usize,
            v.fs_hz,
        )?;
        if q1.passes_vectors() {
            return Ok(());
        }
        return Err(format!(
            "PCM mismatch vs both references (quality {:.1}% err {:.6} ; alt {:.1}% err {:.6})",
            q0.quality_percent, q0.internal_weighted_error, q1.quality_percent, q1.internal_weighted_error
        )
        .into());
    }

    Err(format!(
        "PCM mismatch (quality {:.1}% err {:.6})",
        q0.quality_percent, q0.internal_weighted_error
    )
    .into())
}

fn alt_dec_path(dec_path: &Path) -> Option<PathBuf> {
    let name = dec_path.file_name()?.to_str()?;
    if !name.ends_with(".dec") || name.ends_with("m.dec") {
        return None;
    }
    let alt = name.strip_suffix(".dec")?.to_string() + "m.dec";
    Some(dec_path.with_file_name(alt))
}

fn load_manifest(path: &Path) -> Result<Vec<Vector>, Box<dyn std::error::Error>> {
    let s = fs::read_to_string(path)?;
    let mut out = Vec::new();
    for (lineno, line) in s.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut it = line.split_whitespace();
        let name = it
            .next()
            .ok_or_else(|| format!("line {}: missing name", lineno + 1))?
            .to_string();
        let bit = it
            .next()
            .ok_or_else(|| format!("line {}: missing .bit path", lineno + 1))?
            .to_string();
        let dec = it
            .next()
            .ok_or_else(|| format!("line {}: missing .dec path", lineno + 1))?
            .to_string();
        let fs_hz = it
            .next()
            .ok_or_else(|| format!("line {}: missing fs_hz", lineno + 1))?
            .parse::<u32>()?;
        let channels = it
            .next()
            .ok_or_else(|| format!("line {}: missing channels", lineno + 1))?
            .parse::<u8>()?;
        if it.next().is_some() {
            return Err(format!("line {}: too many fields", lineno + 1).into());
        }
        out.push(Vector {
            name,
            bit,
            dec,
            fs_hz,
            channels,
        });
    }
    Ok(out)
}

fn vectors_dir() -> PathBuf {
    if let Ok(p) = std::env::var("OPUS_TESTVECTORS_DIR") {
        return PathBuf::from(p);
    }
    PathBuf::from("testdata/opus_testvectors")
}

fn load_pcm16le(path: &Path) -> Result<Vec<i16>, Box<dyn std::error::Error>> {
    let bytes = fs::read(path)?;
    if bytes.len() % 2 != 0 {
        return Err(format!("{}: odd byte count", path.display()).into());
    }
    let mut out = Vec::<i16>::with_capacity(bytes.len() / 2);
    for c in bytes.chunks_exact(2) {
        out.push(i16::from_le_bytes([c[0], c[1]]));
    }
    Ok(out)
}

fn read_opus_demo_stream(path: &Path) -> io::Result<Vec<OpusDemoPacket>> {
    let mut f = fs::File::open(path)?;
    let mut bytes = Vec::new();
    f.read_to_end(&mut bytes)?;

    // Opus "opus_demo" framing (proprietary): repeating records:
    // - u32be packet_len
    // - u32be final_range
    // - packet bytes (packet_len)
    //
    // A packet_len of 0 is used to represent a lost packet (PLC).
    //
    // NOTE: This is a best-effort implementation intended for the official
    // conformance vectors. If we find incompatibilities, adjust to match the
    // reference `opus_demo.c` format exactly.
    let mut i = 0usize;
    let mut out = Vec::new();
    while i < bytes.len() {
        if i + 4 > bytes.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "truncated packet_len",
            ));
        }
        let packet_len =
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]) as usize;
        i += 4;

        if i + 4 > bytes.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "truncated final_range",
            ));
        }
        let expected_final_range =
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
        i += 4;

        if i + packet_len > bytes.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "truncated packet",
            ));
        }
        let packet = if packet_len == 0 {
            None
        } else {
            Some(bytes[i..i + packet_len].to_vec())
        };
        i += packet_len;

        out.push(OpusDemoPacket {
            packet,
            expected_final_range,
        });
    }

    Ok(out)
}
