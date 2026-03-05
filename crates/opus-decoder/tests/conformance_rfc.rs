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
    let mut packet_end_samples = Vec::<usize>::new();
    let mut final_range_mismatches: Vec<(usize, u32, u32)> = Vec::new();
    let dump_got_pcm = std::env::var("OPUS_DUMP_GOT_PCM").ok().as_deref() == Some("1");
    let trace_packet_max_delta =
        std::env::var("OPUS_TRACE_PACKET_MAX_DELTA").ok().as_deref() == Some("1");

    for (packet_idx, p) in packets.into_iter().enumerate() {
        match dec.decode(p.packet.as_deref(), &mut out) {
            Ok(samples_per_channel) => {
                let pkt_start = got_pcm.len();
                let written = samples_per_channel * v.channels as usize;
                got_pcm.extend_from_slice(&out[..written]);
                let pkt_end = got_pcm.len();
                packet_end_samples.push(pkt_end);
                let got = dec.final_range();
                if got != p.expected_final_range {
                    final_range_mismatches.push((packet_idx, p.expected_final_range, got));
                }
            }
            Err(Error::NotImplemented) => {
                return Err("decoder not implemented yet".into());
            }
            Err(e) => return Err(format!("decode failed: {e}").into()),
        }
    }

    if !final_range_mismatches.is_empty() {
        for (packet_idx, expected, got) in final_range_mismatches.iter().take(8) {}
    }

    if dump_got_pcm {
        let mut out_bytes = Vec::with_capacity(got_pcm.len() * 2);
        for s in &got_pcm {
            out_bytes.extend_from_slice(&s.to_le_bytes());
        }
        fs::write("/tmp/rust_tv07.pcm", out_bytes)?;
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
    if expected_pcm != got_pcm {
        let min_len = expected_pcm.len().min(got_pcm.len());
        let first_diff = (0..min_len).find(|&i| expected_pcm[i] != got_pcm[i]);
        if let Some(sample_idx) = first_diff {
            let packet_idx = packet_end_samples.partition_point(|&end| end <= sample_idx);
            let mut diff_csv = String::new();
            let mut logged = 0usize;
            for i in sample_idx..min_len {
                if expected_pcm[i] != got_pcm[i] {
                    if !diff_csv.is_empty() {
                        diff_csv.push(';');
                    }
                    let delta = got_pcm[i] as i32 - expected_pcm[i] as i32;
                    diff_csv.push_str(&format!("{i}:{}:{}:{delta}", expected_pcm[i], got_pcm[i]));
                    logged += 1;
                    if logged == 16 {
                        break;
                    }
                }
            }
        }
        let mut top = [(0i32, 0usize); 5];
        for i in 0..min_len {
            let d = (got_pcm[i] as i32 - expected_pcm[i] as i32).abs();
            for slot in 0..5 {
                if d > top[slot].0 {
                    for shift in (slot + 1..5).rev() {
                        top[shift] = top[shift - 1];
                    }
                    top[slot] = (d, i);
                    break;
                }
            }
        }
        let mut top_csv = String::new();
        for (d, i) in top {
            if d == 0 {
                continue;
            }
            if !top_csv.is_empty() {
                top_csv.push(';');
            }
            let packet_idx = packet_end_samples.partition_point(|&end| end <= i);
            top_csv.push_str(&format!(
                "{i}:{packet_idx}:{}:{}:{}",
                expected_pcm[i], got_pcm[i], d
            ));
        }
        if trace_packet_max_delta {
            let mut range_start = 0usize;
            for (packet_idx, &packet_end) in packet_end_samples.iter().enumerate() {
                if range_start >= min_len {
                    break;
                }
                let range_end = packet_end.min(min_len);
                if range_end <= range_start {
                    continue;
                }
                let mut max_delta = 0i32;
                let mut sum_sq = 0.0f64;
                for i in range_start..range_end {
                    let d = expected_pcm[i] as i32 - got_pcm[i] as i32;
                    let ad = d.abs();
                    if ad > max_delta {
                        max_delta = ad;
                    }
                    sum_sq += (d as f64) * (d as f64);
                }
                let rms = (sum_sq / (range_end - range_start) as f64).sqrt();
                if max_delta > 50 {
                    eprintln!(
                        "[RUST] pkt{} max_delta={} rms={:.1}",
                        packet_idx, max_delta, rms
                    );
                }
                range_start = range_end;
            }
        }
    }
    if let Some(alt_pcm) = &alt_pcm {
        if *alt_pcm != got_pcm {
            let min_len = alt_pcm.len().min(got_pcm.len());
            let first_diff = (0..min_len).find(|&i| alt_pcm[i] != got_pcm[i]);
            if let Some(sample_idx) = first_diff {
                let packet_idx = packet_end_samples.partition_point(|&end| end <= sample_idx);
                let mut diff_csv = String::new();
                let mut logged = 0usize;
                for i in sample_idx..min_len {
                    if alt_pcm[i] != got_pcm[i] {
                        if !diff_csv.is_empty() {
                            diff_csv.push(';');
                        }
                        let delta = got_pcm[i] as i32 - alt_pcm[i] as i32;
                        diff_csv.push_str(&format!("{i}:{}:{}:{delta}", alt_pcm[i], got_pcm[i]));
                        logged += 1;
                        if logged == 16 {
                            break;
                        }
                    }
                }
            }
        }
    }
    let q0 = opus_decoder::compare::compare_quality(
        &expected_pcm,
        &got_pcm,
        v.channels as usize,
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
            v.channels as usize,
            v.fs_hz,
        )?;
        if q1.passes_vectors() {
            return Ok(());
        }
        return Err(format!(
            "PCM mismatch vs both references (quality {:.1}% err {:.6} ; alt {:.1}% err {:.6})",
            q0.quality_percent,
            q0.internal_weighted_error,
            q1.quality_percent,
            q1.internal_weighted_error
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
