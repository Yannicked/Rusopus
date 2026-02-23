use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

// Expected file hashes for the pinned official vector set.
//
// NOTE: These are currently the hashes of files extracted from:
//   https://opus-codec.org/docs/opus_testvectors-rfc8251.tar.gz
//
// If upstream changes, `cargo xtask verify-vectors` will fail and we should
// consciously update this list (and ideally also record a pinned archive hash).
const EXPECTED_SHA1: &[(&str, &str)] = &[
    // .bit
    (
        "testvector01.bit",
        "e49b2862ceec7324790ed8019eb9744596d5be01",
    ),
    (
        "testvector02.bit",
        "b809795ae1bcd606049d76de4ad24236257135e0",
    ),
    (
        "testvector03.bit",
        "e0c4ecaeab44d35a2f5b6575cd996848e5ee2acc",
    ),
    (
        "testvector04.bit",
        "a0f870cbe14ebb71fa9066ef3ee96e59c9a75187",
    ),
    (
        "testvector05.bit",
        "9b3d92b48b965dfe9edf7b8a85edd4309f8cf7c8",
    ),
    (
        "testvector06.bit",
        "28e66769ab17e17f72875283c14b19690cbc4e57",
    ),
    (
        "testvector07.bit",
        "bacf467be3215fc7ec288f29e2477de1192947a6",
    ),
    (
        "testvector08.bit",
        "ddbe08b688bbf934071f3893cd0030ce48dba12f",
    ),
    (
        "testvector09.bit",
        "3932d9d61944dab1201645b8eeaad595d5705ecb",
    ),
    (
        "testvector10.bit",
        "521eb2a1e0cc9c31b8b740673307c2d3b10c1900",
    ),
    (
        "testvector11.bit",
        "6bc8f3146fcb96450c901b16c3d464ccdf4d5d96",
    ),
    (
        "testvector12.bit",
        "338c3f1b4b97226bc60bc41038becbc6de06b28f",
    ),
    // .dec (reference output)
    (
        "testvector01.dec",
        "f5ef93884da6a814d311027918e9afc6f2e5c2c8",
    ),
    (
        "testvector02.dec",
        "48ac1ff1995250a756e1e17bd32acefa8cd2b820",
    ),
    (
        "testvector03.dec",
        "d15567e919db2d0e818727092c0af8dd9df23c95",
    ),
    (
        "testvector04.dec",
        "1249dd28f5bd1e39a66fd6d99449dca7a8316342",
    ),
    (
        "testvector05.dec",
        "b85675d81deef84a112c466cdff3b7aaa1d2fc76",
    ),
    (
        "testvector06.dec",
        "55f0b191e90bfa6f98b50d01a64b44255cb4813e",
    ),
    (
        "testvector07.dec",
        "61e8b357ab090b1801eeb578a28a6ae935e25b7b",
    ),
    (
        "testvector08.dec",
        "a58539ee5321453b2ddf4c0f2500e856b3966862",
    ),
    (
        "testvector09.dec",
        "bb96aad2cde188555862b7bbb3af6133851ef8f4",
    ),
    (
        "testvector10.dec",
        "1b6cdf0413ac9965b16184b1bea129b5c0b2a37a",
    ),
    (
        "testvector11.dec",
        "b1fff72b74666e3027801b29dbc48b31f80dee0d",
    ),
    (
        "testvector12.dec",
        "98e09bbafed329e341c3b4052e9c4ba5fc83f9b1",
    ),
    // .dec (alternate output set)
    (
        "testvector01m.dec",
        "1e7d984ea3fbb16ba998aea761f4893fbdb30157",
    ),
    (
        "testvector02m.dec",
        "48ac1ff1995250a756e1e17bd32acefa8cd2b820",
    ),
    (
        "testvector03m.dec",
        "d15567e919db2d0e818727092c0af8dd9df23c95",
    ),
    (
        "testvector04m.dec",
        "1249dd28f5bd1e39a66fd6d99449dca7a8316342",
    ),
    (
        "testvector05m.dec",
        "d70b0bad431e7d463bc3da49bd2d49f1c6d0a530",
    ),
    (
        "testvector06m.dec",
        "6ac1648c3174c95fada565161a6c78bdbe59c77d",
    ),
    (
        "testvector07m.dec",
        "fc5e2f709693738324fb4c8bdc0dad6dda04e713",
    ),
    (
        "testvector08m.dec",
        "aad2ba397bf1b6a18e8e09b50e4b19627d479f00",
    ),
    (
        "testvector09m.dec",
        "6feb7a7b9d7cdc1383baf8d5739e2a514bd0ba08",
    ),
    (
        "testvector10m.dec",
        "1b6cdf0413ac9965b16184b1bea129b5c0b2a37a",
    ),
    (
        "testvector11m.dec",
        "fd3d3a7b0dfbdab98d37ed9aa04b659b9fefbd18",
    ),
    (
        "testvector12m.dec",
        "98e09bbafed329e341c3b4052e9c4ba5fc83f9b1",
    ),
];

fn main() {
    if let Err(e) = real_main() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn real_main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        return Err(usage());
    }

    match args[0].as_str() {
        "fetch-vectors" => {
            let (dest, url, force) = parse_fetch_vectors(&args[1..])?;
            fetch_vectors(&dest, &url, force)
        }
        "verify-vectors" => {
            let dir = parse_verify_vectors(&args[1..])?;
            verify_vectors(&dir)
        }
        "gen-manifest" => {
            let (dir, out, fs_hz, overwrite) = parse_gen_manifest(&args[1..])?;
            gen_manifest(&dir, &out, fs_hz, overwrite)
        }
        _ => Err(usage()),
    }
}

fn usage() -> String {
    [
        "Usage:",
        "  cargo xtask fetch-vectors [--dest <dir>] [--url <tar.gz>] [--force]",
        "  cargo xtask verify-vectors [--dir <dir>]",
        "  cargo xtask gen-manifest [--dir <dir>] [--out <path>] [--fs-hz <hz>] [--overwrite]",
    ]
    .join("\n")
}

fn parse_fetch_vectors(args: &[String]) -> Result<(PathBuf, String, bool), String> {
    let mut dest = PathBuf::from("testdata/opus_testvectors");
    let mut url = "https://opus-codec.org/docs/opus_testvectors-rfc8251.tar.gz".to_string();
    let mut force = false;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--dest" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                dest = PathBuf::from(v);
            }
            "--url" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                url = v.clone();
            }
            "--force" => force = true,
            _ => return Err(usage()),
        }
        i += 1;
    }

    Ok((dest, url, force))
}

fn parse_verify_vectors(args: &[String]) -> Result<PathBuf, String> {
    let mut dir = PathBuf::from("testdata/opus_testvectors");

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--dir" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                dir = PathBuf::from(v);
            }
            _ => return Err(usage()),
        }
        i += 1;
    }
    Ok(dir)
}

fn parse_gen_manifest(args: &[String]) -> Result<(PathBuf, PathBuf, u32, bool), String> {
    let mut dir = PathBuf::from("testdata/opus_testvectors");
    let mut out: Option<PathBuf> = None;
    let mut fs_hz: u32 = 48_000;
    let mut overwrite = false;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--dir" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                dir = PathBuf::from(v);
            }
            "--out" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                out = Some(PathBuf::from(v));
            }
            "--fs-hz" => {
                i += 1;
                let v = args.get(i).ok_or_else(usage)?;
                fs_hz = v.parse::<u32>().map_err(|_| usage())?;
            }
            "--overwrite" => overwrite = true,
            _ => return Err(usage()),
        }
        i += 1;
    }

    let out = out.unwrap_or_else(|| dir.join("vectors.txt"));
    Ok((dir, out, fs_hz, overwrite))
}

fn fetch_vectors(dest: &Path, url: &str, force: bool) -> Result<(), String> {
    if dest.exists() {
        if !force {
            return Err(format!(
                "destination already exists: {} (pass --force to replace)",
                dest.display()
            ));
        }
        fs::remove_dir_all(dest).map_err(|e| format!("remove {}: {e}", dest.display()))?;
    }

    let dest_parent = dest
        .parent()
        .ok_or_else(|| format!("dest has no parent: {}", dest.display()))?;
    fs::create_dir_all(dest_parent)
        .map_err(|e| format!("create {}: {e}", dest_parent.display()))?;

    let tmp = dest_parent.join(format!(".tmp_opus_vectors_{}", std::process::id()));
    if tmp.exists() {
        fs::remove_dir_all(&tmp).map_err(|e| format!("remove {}: {e}", tmp.display()))?;
    }
    fs::create_dir_all(&tmp).map_err(|e| format!("create {}: {e}", tmp.display()))?;

    let archive = tmp.join("opus_testvectors.tar.gz");
    run(
        "curl",
        &["-fL", "-o", archive.to_string_lossy().as_ref(), url],
    )?;

    let extract_root = tmp.join("extract");
    fs::create_dir_all(&extract_root)
        .map_err(|e| format!("create {}: {e}", extract_root.display()))?;
    run(
        "tar",
        &[
            "-xzf",
            archive.to_string_lossy().as_ref(),
            "-C",
            extract_root.to_string_lossy().as_ref(),
        ],
    )?;

    let vectors_dir = find_vectors_dir(&extract_root)?;
    fs::rename(&vectors_dir, dest)
        .map_err(|e| format!("move {} -> {}: {e}", vectors_dir.display(), dest.display()))?;

    verify_vectors(dest)?;
    Ok(())
}

fn gen_manifest(
    dir: &Path,
    out_path: &Path,
    fs_hz_default: u32,
    overwrite: bool,
) -> Result<(), String> {
    if out_path.exists() && !overwrite {
        return Err(format!(
            "manifest already exists: {} (pass --overwrite to replace)",
            out_path.display()
        ));
    }

    let mut bits = Vec::<PathBuf>::new();
    let rd = fs::read_dir(dir).map_err(|e| format!("read_dir {}: {e}", dir.display()))?;
    for ent in rd {
        let ent = ent.map_err(|e| format!("read_dir entry: {e}"))?;
        let p = ent.path();
        if !p.is_file() {
            continue;
        }
        let Some(name) = p.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        if name.starts_with("testvector") && name.ends_with(".bit") {
            bits.push(p);
        }
    }
    bits.sort();

    if bits.is_empty() {
        return Err(format!(
            "no testvector*.bit files found under {}",
            dir.display()
        ));
    }

    if let Some(parent) = out_path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("create {}: {e}", parent.display()))?;
    }

    let mut manifest = String::new();
    manifest.push_str("# Generated by `cargo xtask gen-manifest`\n");
    manifest.push_str("# Format: <name> <bit> <dec> <fs_hz> <channels>\n");

    for bit_path in bits {
        let bit_file = bit_path
            .file_name()
            .and_then(|s| s.to_str())
            .ok_or_else(|| format!("non-utf8 filename: {}", bit_path.display()))?;
        let stem = bit_file
            .strip_suffix(".bit")
            .ok_or_else(|| format!("unexpected .bit filename: {}", bit_file))?;

        let dec_file = format!("{stem}.dec");
        let dec_path = dir.join(&dec_file);
        if !dec_path.is_file() {
            return Err(format!("missing expected file: {}", dec_path.display()));
        }

        let channels = infer_channels_from_bit(&bit_path)?;
        let fs_hz =
            infer_fs_hz_from_lengths(&bit_path, &dec_path, channels).unwrap_or(fs_hz_default);

        manifest.push_str(&format!(
            "{stem} {stem}.bit {dec_file} {fs_hz} {channels}\n"
        ));
    }

    fs::write(out_path, manifest).map_err(|e| format!("write {}: {e}", out_path.display()))?;
    Ok(())
}

fn verify_vectors(dir: &Path) -> Result<(), String> {
    for (rel, expected) in EXPECTED_SHA1 {
        let p = dir.join(rel);
        if !p.is_file() {
            return Err(format!("missing file: {}", p.display()));
        }
        let got = sha1_hex(&p)?;
        if got != *expected {
            return Err(format!(
                "SHA-1 mismatch for {}: expected {}, got {}",
                p.display(),
                expected,
                got
            ));
        }
    }
    Ok(())
}

fn infer_channels_from_bit(bit_path: &Path) -> Result<u8, String> {
    let bytes = fs::read(bit_path).map_err(|e| format!("read {}: {e}", bit_path.display()))?;
    let mut i = 0usize;
    while i < bytes.len() {
        if i + 4 > bytes.len() {
            return Err(format!("{}: truncated packet_len", bit_path.display()));
        }
        let packet_len =
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]) as usize;
        i += 4;

        if i + 4 > bytes.len() {
            return Err(format!("{}: truncated final_range", bit_path.display()));
        }
        i += 4;

        if i + packet_len > bytes.len() {
            return Err(format!("{}: truncated packet", bit_path.display()));
        }
        let packet = &bytes[i..i + packet_len];
        i += packet_len;

        if packet.is_empty() {
            continue; // PLC marker
        }
        let toc = packet[0];
        // RFC 6716: TOC bit 2 (`s`) indicates whether the packet is mono or stereo.
        let stereo = (toc & 0x04) != 0;
        return Ok(if stereo { 2 } else { 1 });
    }
    Err(format!(
        "{}: no non-PLC packets found; cannot infer channels",
        bit_path.display()
    ))
}

fn infer_fs_hz_from_lengths(bit_path: &Path, dec_path: &Path, channels: u8) -> Option<u32> {
    let dec_len = fs::metadata(dec_path).ok()?.len() as usize;
    let denom = 2usize * channels as usize;
    if !dec_len.is_multiple_of(denom) {
        return None;
    }
    let expected_total_samples = dec_len / denom;

    let total_samples_48k = total_samples_48k_from_bit(bit_path).ok()?;

    // Solve: expected_total_samples == total_samples_48k * fs / 48000
    let mut candidates = Vec::new();
    for &fs_hz in &[8000u32, 12000, 16000, 24000, 48000] {
        let num = total_samples_48k.saturating_mul(fs_hz as usize);
        if num % 48000 != 0 {
            continue;
        }
        let out = num / 48000;
        if out == expected_total_samples {
            candidates.push(fs_hz);
        }
    }
    if candidates.len() == 1 {
        Some(candidates[0])
    } else {
        None
    }
}

fn total_samples_48k_from_bit(bit_path: &Path) -> Result<usize, String> {
    let bytes = fs::read(bit_path).map_err(|e| format!("read {}: {e}", bit_path.display()))?;
    let mut i = 0usize;
    let mut total = 0usize;
    let mut last_packet_samples: Option<usize> = None;

    while i < bytes.len() {
        if i + 4 > bytes.len() {
            return Err(format!("{}: truncated packet_len", bit_path.display()));
        }
        let packet_len =
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]) as usize;
        i += 4;

        if i + 4 > bytes.len() {
            return Err(format!("{}: truncated final_range", bit_path.display()));
        }
        i += 4;

        if i + packet_len > bytes.len() {
            return Err(format!("{}: truncated packet", bit_path.display()));
        }
        let packet = &bytes[i..i + packet_len];
        i += packet_len;

        let packet_samples = if packet.is_empty() {
            last_packet_samples.ok_or_else(|| {
                format!(
                    "{}: PLC before first real packet; cannot infer duration",
                    bit_path.display()
                )
            })?
        } else {
            let nb_frames = opus_packet_get_nb_frames(packet)?;
            let spf = opus_packet_get_samples_per_frame_48k(packet[0]);
            let s = nb_frames.saturating_mul(spf);
            last_packet_samples = Some(s);
            s
        };
        total = total.saturating_add(packet_samples);
    }
    Ok(total)
}

fn opus_packet_get_nb_frames(packet: &[u8]) -> Result<usize, String> {
    if packet.is_empty() {
        return Err("empty packet".to_string());
    }
    let code = packet[0] & 0x03;
    match code {
        0 => Ok(1),
        1 | 2 => Ok(2),
        3 => {
            if packet.len() < 2 {
                return Err("count code indicates VBR but packet is too short".to_string());
            }
            let n = (packet[1] & 0x3f) as usize;
            if n == 0 {
                return Err("invalid frame count 0".to_string());
            }
            Ok(n)
        }
        _ => Err("unreachable".to_string()),
    }
}

fn opus_packet_get_samples_per_frame_48k(toc: u8) -> usize {
    // Port of the logic used by libopus' `opus_packet_get_samples_per_frame()`,
    // specialized to Fs=48000.
    //
    // All return values are samples-per-channel.
    if (toc & 0x80) != 0 {
        // CELT-only: 2.5/5/10/20 ms -> 120/240/480/960 @ 48 kHz.
        let audiosize = ((toc & 0x60) >> 5) as usize;
        (48_000usize << audiosize) / 400
    } else if (toc & 0x60) == 0x60 {
        // Hybrid: 10/20 ms -> 480/960 @ 48 kHz.
        if (toc & 0x08) != 0 {
            48_000usize / 50
        } else {
            48_000usize / 100
        }
    } else {
        // SILK-only: 10/20/40/60 ms -> 480/960/1920/2880 @ 48 kHz.
        let audiosize = ((toc & 0x60) >> 5) as usize;
        if audiosize == 3 {
            (48_000usize * 60) / 1000
        } else {
            (48_000usize << audiosize) / 100
        }
    }
}

fn sha1_hex(path: &Path) -> Result<String, String> {
    // Prefer `shasum -a 1`, fall back to `sha1sum`.
    if let Ok(h) = sha1_via_cmd("shasum", &["-a", "1"], path) {
        return Ok(h);
    }
    sha1_via_cmd("sha1sum", &[], path)
}

fn sha1_via_cmd(cmd: &str, args: &[&str], path: &Path) -> Result<String, String> {
    let out = Command::new(cmd)
        .args(args)
        .arg(path)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .map_err(|e| format!("{cmd}: {e}"))?;
    if !out.status.success() {
        return Err(format!("{cmd} exited with {}", out.status));
    }
    let s = String::from_utf8_lossy(&out.stdout);
    let hex = s
        .split_whitespace()
        .next()
        .ok_or_else(|| format!("{cmd}: unexpected output for {}: {s}", path.display()))?;
    Ok(hex.to_lowercase())
}

fn find_vectors_dir(root: &Path) -> Result<PathBuf, String> {
    let target = "testvector01.bit";
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let rd = fs::read_dir(&dir).map_err(|e| format!("read_dir {}: {e}", dir.display()))?;
        for ent in rd {
            let ent = ent.map_err(|e| format!("read_dir entry: {e}"))?;
            let p = ent.path();
            if p.is_dir() {
                stack.push(p);
            } else if p.is_file() && p.file_name().and_then(|s| s.to_str()) == Some(target) {
                return p
                    .parent()
                    .map(|p| p.to_path_buf())
                    .ok_or_else(|| format!("no parent for {}", p.display()));
            }
        }
    }
    Err(format!(
        "could not find {} under {}",
        target,
        root.display()
    ))
}

fn run(cmd: &str, args: &[&str]) -> Result<(), String> {
    let status = Command::new(cmd)
        .args(args)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .map_err(|e| format!("{cmd}: {e}"))?;
    if !status.success() {
        return Err(format!("{cmd} exited with {status}"));
    }
    Ok(())
}
