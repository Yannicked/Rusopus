#[path = "../tests/common.rs"]
mod common;

use common::{build_multistream_packet, load_packets, load_vector_spec};
use criterion::{
    BatchSize, BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main,
};
use opus_decoder::{Decoder, OpusDecoder, OpusMultistreamDecoder};
use std::time::Duration;

/// Benchmark throughput for each major Opus decoding mode.
///
/// Parameters: Criterion benchmark registry `c`.
/// Returns: nothing; registers the mode-throughput benchmark group.
fn bench_per_mode(c: &mut Criterion) {
    let cases = [
        ("celt_mono_48k", "testvector07"),
        ("celt_stereo_48k", "testvector01"),
        ("silk_nb_8k", "testvector02"),
        ("silk_wb_16k", "testvector04"),
        ("hybrid_swb", "testvector05"),
        ("hybrid_fb", "testvector06"),
    ];

    let mut group = c.benchmark_group("per_mode");
    for (bench_name, vector_name) in cases {
        let spec = load_vector_spec(vector_name).expect("failed to load vector spec");
        let packets = load_packets(vector_name).expect("failed to load vector packets");
        let total_samples = compute_total_interleaved_samples(spec.sample_rate, spec.channels, &packets);

        group.throughput(Throughput::Elements(total_samples as u64));
        group.bench_function(bench_name, |b| {
            b.iter(|| {
                black_box(decode_all_i16(spec.sample_rate, spec.channels, &packets));
            })
        });
    }
    group.finish();
}

/// Benchmark CELT mono decode throughput across all supported output rates.
///
/// Parameters: Criterion benchmark registry `c`.
/// Returns: nothing; registers the output-rate benchmark group.
fn bench_output_sample_rates(c: &mut Criterion) {
    let packets = load_packets("testvector07").expect("failed to load testvector07 packets");
    let mut group = c.benchmark_group("output_sample_rates");

    for fs in [8_000u32, 12_000, 16_000, 24_000, 48_000] {
        let total_samples = compute_total_interleaved_samples(fs, 1, &packets);
        group.throughput(Throughput::Elements(total_samples as u64));
        group.bench_with_input(BenchmarkId::new("celt_mono", fs), &fs, |b, &fs| {
            b.iter(|| black_box(decode_all_i16(fs, 1, &packets)));
        });
    }

    group.finish();
}

/// Benchmark normal decode against consecutive PLC bursts.
///
/// Parameters: Criterion benchmark registry `c`.
/// Returns: nothing; registers the PLC-overhead benchmark group.
fn bench_plc_overhead(c: &mut Criterion) {
    let packets = load_packets("testvector07").expect("failed to load testvector07 packets");
    let mut group = c.benchmark_group("plc_overhead");

    let normal_samples = measure_normal_after_warmup(&packets);
    group.throughput(Throughput::Elements(normal_samples as u64));
    group.bench_function("normal_decode", |b| {
        b.iter_batched(
            || prepare_plc_state(&packets),
            |(mut decoder, mut pcm)| {
                black_box(
                    decoder
                        .decode(&packets[10], &mut pcm, false)
                        .expect("normal post-warm decode failed"),
                )
            },
            BatchSize::SmallInput,
        );
    });

    for losses in [1usize, 5, 10] {
        let loss_samples = measure_plc_burst(&packets, losses);
        group.throughput(Throughput::Elements(loss_samples as u64));
        group.bench_function(format!("plc_{losses}_loss"), |b| {
            b.iter_batched(
                || prepare_plc_state(&packets),
                |(mut decoder, mut pcm)| {
                    let mut total_samples = 0usize;
                    for _loss_idx in 0..losses {
                        total_samples += decoder
                            .decode(&[], &mut pcm, false)
                            .expect("PLC decode failed");
                    }
                    black_box(total_samples)
                },
                BatchSize::SmallInput,
            );
        });
    }

    group.finish();
}

/// Benchmark multistream wrapper overhead against two independent decoders.
///
/// Parameters: Criterion benchmark registry `c`.
/// Returns: nothing; registers the multistream benchmark group.
fn bench_multistream_overhead(c: &mut Criterion) {
    let mono_packets = load_packets("testvector07").expect("failed to load testvector07 packets");
    let multistream_packets: Vec<Vec<u8>> = mono_packets
        .iter()
        .map(|packet| build_multistream_packet(&[packet.as_slice(), packet.as_slice()]))
        .collect();

    let total_samples = decode_all_multistream(&multistream_packets);
    let mut group = c.benchmark_group("multistream");

    group.throughput(Throughput::Elements(total_samples as u64));
    group.bench_function("multistream_2ch", |b| {
        b.iter(|| black_box(decode_all_multistream(&multistream_packets)));
    });

    group.throughput(Throughput::Elements(total_samples as u64));
    group.bench_function("two_decoders_2ch", |b| {
        b.iter(|| black_box(decode_all_two_decoders(&mono_packets)));
    });

    group.finish();
}

/// Decode one packet set with a fresh decoder and sum interleaved output samples.
///
/// Parameters: output `sample_rate`, decoder `channels`, and raw `packets`.
/// Returns: total interleaved i16 samples decoded across all packets.
fn decode_all_i16(sample_rate: u32, channels: usize, packets: &[Vec<u8>]) -> usize {
    let mut decoder = OpusDecoder::new(sample_rate, channels).expect("failed to create decoder");
    let mut pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K * channels];
    let mut total_samples = 0usize;

    for packet in packets {
        let written = decoder
            .decode(packet, &mut pcm, false)
            .unwrap_or_else(|err| {
                panic!(
                    "packet decode failed: sample_rate={sample_rate}, channels={channels}, err={err:?}"
                )
            });
        total_samples += written * channels;
    }

    total_samples
}

/// Decode all packets with the multistream wrapper and sum interleaved samples.
///
/// Parameters: synthetic two-stream `packets`.
/// Returns: total interleaved i16 samples decoded across all packets.
fn decode_all_multistream(packets: &[Vec<u8>]) -> usize {
    let mut decoder =
        OpusMultistreamDecoder::new(48_000, 2, 2, 0, &[0, 1]).expect("failed to create decoder");
    let mut pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K * 2];
    let mut total_samples = 0usize;

    for packet in packets {
        let written = decoder
            .decode(packet, &mut pcm, false)
            .expect("multistream decode failed");
        total_samples += written * 2;
    }

    total_samples
}

/// Decode all packets with two mono decoders and manual stereo interleave.
///
/// Parameters: mono `packets` reused for both channels.
/// Returns: total interleaved i16 samples decoded across all packets.
fn decode_all_two_decoders(packets: &[Vec<u8>]) -> usize {
    let mut left = OpusDecoder::new(48_000, 1).expect("failed to create left decoder");
    let mut right = OpusDecoder::new(48_000, 1).expect("failed to create right decoder");
    let mut left_pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K];
    let mut right_pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K];
    let mut stereo_pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K * 2];
    let mut total_samples = 0usize;

    for packet in packets {
        let left_written = left
            .decode(packet, &mut left_pcm, false)
            .expect("left decode failed");
        let right_written = right
            .decode(packet, &mut right_pcm, false)
            .expect("right decode failed");
        assert_eq!(left_written, right_written, "paired mono packets must match");

        for sample_idx in 0..left_written {
            stereo_pcm[2 * sample_idx] = left_pcm[sample_idx];
            stereo_pcm[(2 * sample_idx) + 1] = right_pcm[sample_idx];
        }

        total_samples += left_written * 2;
    }

    total_samples
}

/// Prepare one warmed CELT decoder state for PLC benchmarks.
///
/// Parameters: CELT mono `packets`.
/// Returns: warmed decoder plus reusable PCM buffer.
fn prepare_plc_state(packets: &[Vec<u8>]) -> (OpusDecoder, Vec<i16>) {
    let mut decoder = OpusDecoder::new(48_000, 1).expect("failed to create PLC decoder");
    let mut pcm = vec![0i16; Decoder::MAX_FRAME_SIZE_48K];

    for packet in packets.iter().take(10) {
        decoder
            .decode(packet, &mut pcm, false)
            .expect("warm-up decode failed");
    }

    (decoder, pcm)
}

/// Measure one normal decode immediately after warming the CELT decoder state.
///
/// Parameters: CELT mono `packets`.
/// Returns: interleaved sample count for the measured decode call.
fn measure_normal_after_warmup(packets: &[Vec<u8>]) -> usize {
    let (mut decoder, mut pcm) = prepare_plc_state(packets);
    decoder
        .decode(&packets[10], &mut pcm, false)
        .expect("normal post-warm decode failed")
}

/// Measure a burst of consecutive PLC calls after warming decoder state.
///
/// Parameters: CELT mono `packets` and number of consecutive `losses`.
/// Returns: total interleaved sample count across the PLC burst.
fn measure_plc_burst(packets: &[Vec<u8>], losses: usize) -> usize {
    let (mut decoder, mut pcm) = prepare_plc_state(packets);
    let mut total_samples = 0usize;
    for _loss_idx in 0..losses {
        total_samples += decoder
            .decode(&[], &mut pcm, false)
            .expect("PLC decode failed");
    }

    total_samples
}

/// Precompute total interleaved samples for one benchmark iteration.
///
/// Parameters: output `sample_rate`, decoder `channels`, and raw `packets`.
/// Returns: total interleaved samples written by one full decode sweep.
fn compute_total_interleaved_samples(
    sample_rate: u32,
    channels: usize,
    packets: &[Vec<u8>],
) -> usize {
    decode_all_i16(sample_rate, channels, packets)
}

criterion_group!(
    name = decode_benches;
    config = Criterion::default()
        .warm_up_time(Duration::from_secs(2))
        .measurement_time(Duration::from_secs(10))
        .sample_size(50);
    targets = bench_per_mode, bench_output_sample_rates, bench_plc_overhead, bench_multistream_overhead
);
criterion_main!(decode_benches);
