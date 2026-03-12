# Changelog

All notable changes to `opus-decoder` are documented here.
Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)

## [Unreleased]

- bench: added `crates/bench-compare/` crate with FFI wrapper around libopus C
  (`opus_decode`) and criterion benchmarks comparing Rust vs C on CELT/SILK/Hybrid.
  Baseline results (correctness-first, no optimization):
  CELT mono 48k: Rust 803ms vs C 20.3ms (~40×);
  SILK NB 8k: Rust 10.3ms vs C 7.4ms (~1.4×);
  Hybrid FB 48k: Rust 2.16s vs C 35.2ms (~61×).
  SILK nearly matches C; CELT/Hybrid gap expected — libopus uses SIMD and
  hand-optimized float paths not yet present in rasopus. Commit `2475647`.

## [0.1.1] - 2026-03-12

### Fixed
- Corrected repository and homepage URLs in `Cargo.toml` to
  `https://github.com/TadeuszWolfGang/Rusopus`.

## [0.1.0] - 2026-03-12

### Added
- release: published `opus-decoder v0.1.0` to crates.io.
  https://crates.io/crates/opus-decoder
- chore(release): clarified `fec` parameter doc as PLC placeholder for future
  in-band FEC; corrected MSRV in README from 1.70 to 1.85; replaced "clean-room
  port" with accurate "pure Rust port verified against libopus"; consolidated
  dual CHANGELOG into single Keep-a-Changelog format at repo root.
  Commit `440ee50`.
- release: `cargo publish --dry-run` clean — 41 files, 598.4 KiB
  (122.3 KiB compressed). Ready for `cargo publish`.
- Final pre-publish release polish: clarified that `fec` is currently documented as a future in-band FEC placeholder and treated as PLC when `true`, corrected the README MSRV to Rust 1.85, updated the README implementation description, and consolidated the changelog into the repo root.
- Public API cleanup for crates.io release: stabilized `OpusDecoder::{new, decode, decode_float, reset}` and `OpusError`, sealed internal `Decoder`/`Error` internals behind `pub(crate)`, added module-level docs for `lib.rs`, `celt/mod.rs`, `silk/mod.rs`, and `entropy.rs`, and removed remaining `OPUS_TRACE_*` / `eprintln!` instrumentation from `src/`.
- Packet loss concealment support across the public API: `OpusDecoder::{decode, decode_float}` accept empty packets for PLC, decoder state tracks `loss_count` and `last_packet_duration`, CELT PLC reuses prior energy/history with fade-out and noise injection, and an ignored PLC smoke test was added.
- SILK pitch-extrapolation PLC in `silk/plc.rs` with `SilkPlcState`, `plc_update()`, and `plc_conceal()`, saving pitch/LPC/output history after good frames and reusing the existing resampler state on the loss path.
- `OpusMultistreamDecoder` with `new`, `decode`, `decode_float`, and `reset`, including RFC 7845-style channel mapping, self-delimited packet splitting, silence mapping via `255`, and coverage for call-recording, silence-channel, and invalid-argument cases.
- Criterion benchmark coverage for per-mode throughput, CELT output sample rates, PLC overhead, and multistream wrapper overhead, with baseline results recorded in `PROJECT_STATUS.md`.
- Full CELT output-rate decimation for 8/12/16/24 kHz output instead of returning `NotImplemented`, keeping behavior aligned with the libopus decoder output path.
- Hybrid and transition parity fixes needed for full RFC 8251 coverage: corrected mono Hybrid resampler carry, restored Hybrid libopus parity around redundancy/postfilter handling and SWB `effEnd`, and matched CELT redundancy end-band selection to packet bandwidth in SilkOnly transitions.
- Decoder mode-transition state handling for SILK/CELT carry-over, including redundancy tracking and transition fade/reset flow.
- SILK Phase 3c excitation reconstruction and LPC synthesis with libopus-matching subframe gains, pitch/LTP handling, Q14 LPC synthesis, and zero-padding of internal-rate PCM to API frame size.
- SILK Phase 3a entropy-path support for SILK-only packets, including VAD/LBRR parsing, stereo signaling, main/LBRR indices and pulses, correct `final_range` tracking for `testvector02`, and corrected RFC vector manifest channel counts.
- Early CELT conformance and debug groundwork: config-based `OpusMode` routing, exposed `final_range()` plus soft per-packet checks, recursive CELT `quant_partition` split support, and bit-exact MDCT backward-path verification.
- CELT synthesis/band fixes for mono and stereo-coded packets: unified `decode_mem`, libopus-style overlap/history layout, corrected stereo collapse-mask TF mapping and stereo merge semantics, and fixed lowband history propagation for `N=1`.
- CELT packet/band quantization fixes for the initial mono decode milestone: code-1 packet parsing rejects odd payload lengths, TOC bandwidth-dependent band ends are correct, lowband fold wrap-around is fixed, `tf_change` block scaling and `itheta` floor quantization match libopus, and `LM=-1` handling in `bits2pulses` / `pulses2bits` is implemented.
- Project planning/docs baseline: `OPUS_PORT_PLAN.md` was rebuilt to match libopus decoder requirements, including corrected CELT/SILK sample-rate architecture, expanded phased plan details, a decoder state model, RFC 8251 errata notes, and Rust module mapping.
- RFC 8251 conformance milestones: CELT mono `testvector07` reached bit-exact `final_range` parity and `98.1%` `opus_compare` quality, `testvector05` and `testvector06` brought the sweep to `8/12`, and the decoder ultimately reached `12/12` passing RFC 8251 vectors.

[0.1.1]: https://github.com/TadeuszWolfGang/Rusopus/releases/tag/v0.1.1
[0.1.0]: https://github.com/tadeuszwojcik/rasopus/releases/tag/v0.1.0
