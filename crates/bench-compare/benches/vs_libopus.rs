use bench_compare::libopus_ffi::LibopusDecoder;
use criterion::{Criterion, Throughput, black_box, criterion_group, criterion_main};
use opus_decoder::OpusDecoder;
use std::fs;

const TESTVECTORS_DIR: &str = "../../testdata/opus_testvectors";

/// Load all non-lost packet payloads from one `opus_demo` `.bit` stream.
///
/// Parameters: canonical `vector_name` like `testvector07`.
/// Returns: raw packet payloads in stream order.
fn load_packets(vector_name: &str) -> Vec<Vec<u8>> {
    let dir = std::env::var("OPUS_TESTVECTORS_DIR")
        .unwrap_or_else(|_| format!("{}/{}", env!("CARGO_MANIFEST_DIR"), TESTVECTORS_DIR));
    let path = format!("{dir}/{vector_name}.bit");
    let data = fs::read(&path).unwrap_or_else(|err| panic!("cannot read {path}: {err}"));

    let mut packets = Vec::new();
    let mut pos = 0usize;
    while pos < data.len() {
        assert!(
            pos + 8 <= data.len(),
            "truncated packet header in benchmark input: {path}"
        );

        let len = u32::from_be_bytes(data[pos..pos + 4].try_into().expect("len bytes")) as usize;
        pos += 4;

        let _final_range = u32::from_be_bytes(data[pos..pos + 4].try_into().expect("range bytes"));
        pos += 4;

        assert!(
            pos + len <= data.len(),
            "truncated packet payload in benchmark input: {path}"
        );
        if len != 0 {
            packets.push(data[pos..pos + len].to_vec());
        }
        pos += len;
    }

    packets
}

/// Compute the maximum interleaved i16 buffer length for one decoder setup.
///
/// Parameters: output `sample_rate` and decoder `channels`.
/// Returns: maximum writable PCM length in interleaved samples.
fn decoder_buffer_len(sample_rate: u32, channels: usize) -> usize {
    let decoder = OpusDecoder::new(sample_rate, channels).expect("failed to create sizing decoder");
    decoder.max_frame_size_per_channel() * channels
}

/// Measure total interleaved samples written by one full decode sweep.
///
/// Parameters: output `sample_rate`, decoder `channels`, and raw `packets`.
/// Returns: total interleaved PCM samples produced across the packet list.
fn decoded_interleaved_samples(sample_rate: u32, channels: usize, packets: &[Vec<u8>]) -> u64 {
    let mut decoder =
        OpusDecoder::new(sample_rate, channels).expect("failed to create throughput decoder");
    let mut pcm = vec![0i16; decoder.max_frame_size_per_channel() * channels];
    let mut total = 0u64;

    for packet in packets {
        let written = decoder
            .decode(packet, &mut pcm, false)
            .expect("throughput sizing decode failed");
        total += (written * channels) as u64;
    }

    total
}

/// Benchmark CELT mono fullband decode throughput at 48 kHz.
///
/// Parameters: Criterion registry `c`.
/// Returns: nothing; registers the CELT mono benchmark group.
fn bench_celt_mono(c: &mut Criterion) {
    let packets = load_packets("testvector07");
    let total_samples = decoded_interleaved_samples(48_000, 1, &packets);
    let pcm_len = decoder_buffer_len(48_000, 1);

    let mut group = c.benchmark_group("celt_mono_48k");
    group.sample_size(20);
    group.throughput(Throughput::Elements(total_samples));

    group.bench_function("rust", |b| {
        b.iter(|| {
            let mut dec = OpusDecoder::new(48_000, 1).expect("failed to create Rust decoder");
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec
                    .decode(pkt, &mut pcm, false)
                    .expect("Rust decode failed");
                black_box(&pcm[..written]);
            }
        })
    });

    group.bench_function("libopus_c", |b| {
        b.iter(|| {
            let mut dec = LibopusDecoder::new(48_000, 1);
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec.decode(pkt, &mut pcm);
                assert!(written >= 0, "libopus decode failed: {written}");
                black_box(&pcm[..written as usize]);
            }
        })
    });

    group.finish();
}

/// Benchmark SILK narrowband decode throughput at 8 kHz.
///
/// Parameters: Criterion registry `c`.
/// Returns: nothing; registers the SILK NB benchmark group.
fn bench_silk_nb(c: &mut Criterion) {
    let packets = load_packets("testvector02");
    let total_samples = decoded_interleaved_samples(8_000, 1, &packets);
    let pcm_len = decoder_buffer_len(8_000, 1);

    let mut group = c.benchmark_group("silk_nb_8k");
    group.sample_size(20);
    group.throughput(Throughput::Elements(total_samples));

    group.bench_function("rust", |b| {
        b.iter(|| {
            let mut dec = OpusDecoder::new(8_000, 1).expect("failed to create Rust decoder");
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec
                    .decode(pkt, &mut pcm, false)
                    .expect("Rust decode failed");
                black_box(&pcm[..written]);
            }
        })
    });

    group.bench_function("libopus_c", |b| {
        b.iter(|| {
            let mut dec = LibopusDecoder::new(8_000, 1);
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec.decode(pkt, &mut pcm);
                assert!(written >= 0, "libopus decode failed: {written}");
                black_box(&pcm[..written as usize]);
            }
        })
    });

    group.finish();
}

/// Benchmark Hybrid fullband stereo decode throughput at 48 kHz.
///
/// Parameters: Criterion registry `c`.
/// Returns: nothing; registers the Hybrid FB benchmark group.
fn bench_hybrid_fb(c: &mut Criterion) {
    let packets = load_packets("testvector06");
    let total_samples = decoded_interleaved_samples(48_000, 2, &packets);
    let pcm_len = decoder_buffer_len(48_000, 2);

    let mut group = c.benchmark_group("hybrid_fb_48k");
    group.sample_size(20);
    group.throughput(Throughput::Elements(total_samples));

    group.bench_function("rust", |b| {
        b.iter(|| {
            let mut dec = OpusDecoder::new(48_000, 2).expect("failed to create Rust decoder");
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec
                    .decode(pkt, &mut pcm, false)
                    .expect("Rust decode failed");
                black_box(&pcm[..written * 2]);
            }
        })
    });

    group.bench_function("libopus_c", |b| {
        b.iter(|| {
            let mut dec = LibopusDecoder::new(48_000, 2);
            let mut pcm = vec![0i16; pcm_len];
            for pkt in &packets {
                let written = dec.decode(pkt, &mut pcm);
                assert!(written >= 0, "libopus decode failed: {written}");
                black_box(&pcm[..written as usize * 2]);
            }
        })
    });

    group.finish();
}

criterion_group!(benches, bench_celt_mono, bench_silk_nb, bench_hybrid_fb);
criterion_main!(benches);
