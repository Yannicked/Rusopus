# Rasopus — Project Status (2026-03-12)

## Achievement: 12/12 RFC8251 Conformance ✅ + Full Feature Complete ✅

All Opus test vectors pass `opus_compare` quality threshold.
All planned phases (1–7 + multistream) implemented and merged.

| Vector | Description | Status |
|--------|-------------|--------|
| tv01 | CELT stereo FB | ✅ PASS |
| tv02 | SILK NB 8kHz mono | ✅ PASS |
| tv03 | SILK MB 12kHz mono | ✅ PASS |
| tv04 | SILK WB 16kHz mono | ✅ PASS |
| tv05 | Hybrid SWB transitions | ✅ PASS |
| tv06 | Hybrid FB transitions | ✅ PASS |
| tv07 | CELT mono FB | ✅ PASS |
| tv08 | SILK stereo + CELT NB stereo | ✅ PASS |
| tv09 | SILK→CELT transitions | ✅ PASS |
| tv10 | CeltOnly + Hybrid mix | ✅ PASS |
| tv11 | CELT stereo FB | ✅ PASS |
| tv12 | SILK bandwidth transitions | ✅ PASS |

## Repository

- **Location**: `/Users/tadeusz/Opus/Rasopus`
- **Workspace**: Cargo workspace, main crate `opus-decoder` in `crates/opus-decoder/`
- **Latest commit**: after Phase 8 benchmark baseline
- **Architecture**: Floating-point decode path (confirmed design decision)
- **Safety**: `#![forbid(unsafe_code)]` throughout correctness phases

## Phase History

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Infrastructure: range coder, packet parser, TOC, test harness, opus_compare | ✅ |
| 2 | CELT: MDCT, band decode, quant_all_bands, stereo, anti-collapse, deemphasis | ✅ |
| 3 | SILK: entropy, NLSF→LPC, excitation, LPC synthesis, IIR_FIR resampler, stereo | ✅ |
| 4 | Hybrid: shared EcDec, start_band=17, redundancy, crossfade, mode transitions | ✅ |
| 5a | CELT PLC: fade-out reusing overlap/decode_mem/deemphasis | ✅ |
| 5b | SILK PLC: `silk_PLC_conceal()` pitch extrapolation, `plc_update()` | ✅ |
| 6 | Multi-stream: `OpusMultistreamDecoder`, channel mapping, call recording | ✅ |
| 7 | Cleanup: removed traces, stabilized public API, clippy clean, doc comments | ✅ |
| **8** | **Benchmark vs libopus** | 🚧 baseline measured |

## Public API

```rust
// Single-stream
OpusDecoder::new(sample_rate: u32, channels: usize) -> Result<Self, OpusError>
OpusDecoder::decode(&mut self, packet: &[u8], pcm: &mut [i16], fec: bool) -> Result<usize, OpusError>
OpusDecoder::decode_float(&mut self, packet: &[u8], pcm: &mut [f32], fec: bool) -> Result<usize, OpusError>
OpusDecoder::reset(&mut self)

// Multi-stream
OpusMultistreamDecoder::new(sample_rate, nb_channels, nb_streams, nb_coupled, mapping)
OpusMultistreamDecoder::decode / decode_float / reset

// Error
OpusError: InvalidPacket | InternalError | BufferTooSmall | InvalidArgument
```

## Test Command

```bash
# Single vector
OPUS_TESTVECTORS_DIR="/Users/tadeusz/Opus/Rasopus/testdata/opus_testvectors" \
OPUS_VECTOR=testvectorXX \
cargo test --release -p opus-decoder rfc_conformance -- --ignored --nocapture

# Full sweep
for v in 01 02 03 04 05 06 07 08 09 10 11 12; do
  echo "=== tv${v} ==="
  OPUS_TESTVECTORS_DIR="/Users/tadeusz/Opus/Rasopus/testdata/opus_testvectors" \
  OPUS_VECTOR=testvector${v} \
  cargo test --release -p opus-decoder rfc_conformance -- --ignored --nocapture 2>&1 | \
  grep -E "PASS|FAIL" | head -1
done

# PLC smoke test
OPUS_TESTVECTORS_DIR="/Users/tadeusz/Opus/Rasopus/testdata/opus_testvectors" \
cargo test -p opus-decoder plc_smoke_test -- --ignored --nocapture

# Multistream
cargo test -p opus-decoder multistream -- --nocapture
```

## Phase 8 Benchmark Baseline (`rasopus_baseline`)

Measured on: Apple M3 Pro (12 CPU cores)  
Rustc: `rustc 1.93.0 (254b59607 2026-01-19)`  
Date: 2026-03-12

| Benchmark | Throughput / Time | Notes |
|-----------|-------------------|-------|
| `celt_mono_48k` | `1.396 Melem/s` | `testvector07`, mono fullband CELT |
| `celt_stereo_48k` | `1.386 Melem/s` | `testvector01`, stereo fullband CELT |
| `silk_nb_8k` | `157.25 Melem/s` | `testvector02`, NB SILK content, `48 kHz` API output |
| `silk_wb_16k` | `92.91 Melem/s` | `testvector04`, WB SILK content, `48 kHz` API output |
| `hybrid_swb` | `1.160 Melem/s` | `testvector05` |
| `hybrid_fb` | `1.162 Melem/s` | `testvector06` |
| `plc_1_loss` | `1298 us/frame` | median burst time `1.298 ms` for one lost frame |
| `plc_5_loss` | `1295 us/frame` | median burst time `6.476 ms / 5` |
| `plc_10_loss` | `1274 us/frame` | median burst time `12.740 ms / 10` |

### CELT Output Rates (`testvector07`)

| Output rate | Throughput | Notes |
|-------------|------------|-------|
| `8000` | `237.36 Kelem/s` | CELT output decimation from internal `48 kHz` |
| `12000` | `354.59 Kelem/s` | CELT output decimation from internal `48 kHz` |
| `16000` | `477.56 Kelem/s` | CELT output decimation from internal `48 kHz` |
| `24000` | `715.48 Kelem/s` | CELT output decimation from internal `48 kHz` |
| `48000` | `1.431 Melem/s` | native CELT output path |

### Multistream Wrapper Overhead

| Benchmark | Throughput | Notes |
|-----------|------------|-------|
| `multistream_2ch` | `1.417 Melem/s` | `OpusMultistreamDecoder` with two mono streams |
| `two_decoders_2ch` | `1.426 Melem/s` | two independent `OpusDecoder` instances |

Real-time factor vs libopus: pending manual C-side measurement (`scripts/bench_libopus.sh` not added yet).

## Known Limitations / TODO

| Item | Priority | Notes |
|------|----------|-------|
| Hybrid CELT highband PLC | Low | Currently SILK-only concealment for Hybrid loss |
| SIMD optimization | Low | Phase 8 scope, requires lifting `forbid(unsafe_code)` locally |
| Ogg/WebM demux | Out of scope | Separate crate |
| Encoder | Out of scope | Decoder-only by design |
| crates.io publish | Pending | After Phase 8 benchmark baseline |

## Key Files

| File | Purpose |
|------|---------|
| `crates/opus-decoder/src/lib.rs` | Public API, mode routing, transitions, redundancy |
| `crates/opus-decoder/src/multistream.rs` | `OpusMultistreamDecoder`, channel mapping |
| `crates/opus-decoder/src/celt/mod.rs` | CELT decoder + PLC |
| `crates/opus-decoder/src/celt/bands.rs` | Band decode: quant_all_bands, denormalise |
| `crates/opus-decoder/src/celt/rate.rs` | Bit allocation |
| `crates/opus-decoder/src/celt/quant_bands.rs` | Energy quantization |
| `crates/opus-decoder/src/silk/mod.rs` | SILK decoder, resampler integration, stereo |
| `crates/opus-decoder/src/silk/decode_core.rs` | SILK LPC synthesis, excitation |
| `crates/opus-decoder/src/silk/plc.rs` | SILK PLC: `plc_update`, `plc_conceal` |
| `crates/opus-decoder/src/silk/resampler.rs` | IIR_FIR resampler 8/12/16kHz → 48kHz |
| `crates/opus-decoder/src/silk/stereo.rs` | SILK MS_to_LR stereo predictor |
| `crates/opus-decoder/src/entropy.rs` | Range coder (EcDec) |
| `crates/opus-decoder/src/compare.rs` | opus_compare port |
| `crates/opus-decoder/tests/conformance_rfc.rs` | RFC8251 conformance harness |
| `crates/opus-decoder/tests/multistream.rs` | Multistream tests |

## Debug Workflow

**Bug mode**: Claude diagnoses → Cursor AI prompt → Tadeusz implements → test → iterate.

**Core technique**: C vs Rust checkpoint comparison:
1. Top-level: mode, config, final_range
2. Post-SILK: ec tell/rng
3. Post-CELT stages: silence → postfilter → transient → coarse → tf → spread → alloc → fine → bands → finalise
4. Per-band: bits, pulses, fine_quant, offsets
5. Synthesis: denorm → IMDCT → overlap → deemphasis → output
6. Resampler: internal_pcm → resampled → interleaved

## Architecture Notes

- **Floating-point path**: All CELT synthesis uses f32. SILK uses i32 fixed-point internally, converts to i16 for resampler, then i16 output.
- **Shared EcDec**: Hybrid mode — SILK reads front, CELT reads back of same range coder.
- **start_band**: 0 for CELT-only, 17 for Hybrid. Must reset to 0 after each Hybrid packet.
- **PLC**: CELT fade-out from overlap buffer; SILK pitch extrapolation from saved LPC/pitch state. Hybrid uses SILK PLC only (no highband CELT PLC).
- **Multistream**: `nb_coupled_streams` stereo decoders + remaining mono decoders; mapping[255] = silence.
