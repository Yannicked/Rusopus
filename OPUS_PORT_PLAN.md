# Opus Decoder (Rust, no-FFI) - Port Plan

Goal: build a pure-Rust Opus decoder by porting libopus algorithms (decoder-only, no FFI), with conformance as the primary constraint.

This repo currently targets the "core" primitive:

- Input: a single Opus packet (`&[u8]`) or "packet loss" (`None`)
- Output: decoded PCM (interleaved) at a caller-selected sampling rate (8/12/16/24/48 kHz)

Container demux (Ogg/WebM/Matroska) is intentionally out of scope for this crate/project phase.

## Decisions (confirmed)

- Arithmetic path: floating-point path (libopus float decoder semantics), not fixed-point.
- Scope: decoder-only; encoder is out of scope.
- Conformance: `opus_compare` quality metric plus per-packet `final_range` comparison.
- Sampling rates: support all Opus output `Fs`: 8000 / 12000 / 16000 / 24000 / 48000.
- Stream scope: mono/stereo single-stream decode only for MVP (no multistream).
- Scope boundary: core "packet -> PCM" only; demuxer is a separate project/crate later.
- Safety policy: `#![forbid(unsafe_code)]` through correctness phases; optional SIMD `unsafe` only in Phase 8 with `// SAFETY:` comments and Miri coverage.
- Opus packet channel count (mono/stereo) may vary per packet; decoder output channels (1/2) are independent:
  - if packet is mono and output is stereo -> upmix by duplication,
  - if packet is stereo and output is mono -> downmix.
- Call recording export: recommended industry approach is stereo separation of speakers:
  - Left channel: caller (Stream A, mono Opus)
  - Right channel: callee (Stream B, mono Opus)
  This is implemented by running two independent mono decoders and interleaving their PCM into stereo. It is not "Opus 2-channel" by itself.

## Source of truth and validation

- Spec: RFC 6716 (Opus) and RFC 8251 errata/updates. RFC 8251 corrections (including padding-related behavior) must be reviewed and annotated in tests/comments where relevant.
- Conformance vectors: RFC 6716 vectors + additional vectors (e.g. RFC 8251-related updates / newer official sets).
- Reference implementation: libopus (BSD, royalty-free) used as porting reference and for interpreting corner cases.
  - Vendored snapshot for reference: `third_party/libopus` (pinned commit recorded in `third_party/README.md`).

Hard rule: conformance vectors must pass; mismatches are bugs.

Note: the reference libopus `run_vectors.sh` compares decoded output using the
`opus_compare` quality metric (not a byte-exact PCM compare). For CELT, this is
important because small sample-level differences can occur across builds/platforms
while still being considered acceptable by the reference harness.

## Current status snapshot (2026-02-22)

- `packet.rs`: TOC parse, code 0/1/2/3, VBR/CBR, padding, frame sizes ✅
- `entropy.rs`: full range decoder port (`entdec.c` + `entcode.c`) ✅
- `compare.rs`: `opus_compare.c` port ✅
- `tests/conformance_rfc.rs`: manifest parser + opus_demo reader + quality compare ✅
- `xtask`: `fetch-vectors` (SHA-1 verified), `gen-manifest` (auto channels + `fs_hz`) ✅
- `celt/mod.rs`: scaffold only (`decode_frame()` returns `NotImplemented`) 🚧
- `lib.rs`: `Decoder::new/decode/reset`, CELT-only routing by TOC, `#![forbid(unsafe_code)]` ✅
- `silk/`: no module yet 🚧

## Non-goals (for MVP)

- Encoder.
- Container parsing (Ogg/WebM).
- Multistream (beyond mono/stereo single-stream decode); if needed later, treat as a separate milestone.
- SIMD/perf tuning until correctness is locked.

## Public API sketch (stabilize early)

Minimal, C-like ergonomics to mirror reference semantics:

- `Decoder::new(fs_hz, channels)`
- `decode(packet: Option<&[u8]>, out: &mut [i16]) -> Result<usize, Error>`
  - returns samples-per-channel written
  - `packet=None` triggers PLC (packet loss concealment)
- optional: `decode_float(..., out: &mut [f32])`
- `reset()` and basic `ctl`-like knobs (as needed for conformance)
- `final_range() -> u32`

Output layout: interleaved PCM (like libopus).

## Decoder State Model

State that must persist between `decode()` calls:

- CELT state:
  - overlap buffer (last 120 samples x channels)
  - previous band energies (21 bands)
  - postfilter state (period, gains)
  - previous pitch period
  - deemphasis memory
- SILK state:
  - LPC synthesis memory (16 taps)
  - resampler state
  - PLC/CNG state
  - previous gain
  - previous NLSF
  - previous pitch lag
  - previous subframe gains
- Opus-level state:
  - previous mode (SILK/CELT/Hybrid)
  - previous bandwidth
  - previous frame size
  - total decode count
  - `final_range`

This model is the reference for `reset()` semantics and packet-to-packet continuity.

## Suggested Rust module mapping (reference)

```text
src/
├── lib.rs              # Decoder public API (opus_decoder.c top-level)
├── packet.rs           # TOC parse, frame sizing ✅
├── entropy.rs          # Range decoder (celt/entdec.c, entcode.c) ✅
├── compare.rs          # opus_compare quality metric ✅
├── error.rs            # Error types ✅
├── celt/
│   ├── mod.rs          # celt_decoder.c main decode loop
│   ├── bands.rs        # bands.c (PVQ decode, denormalization)
│   ├── rate.rs         # rate.c (bit allocation)
│   ├── mdct.rs         # mdct.c (IMDCT)
│   ├── kiss_fft.rs     # kiss_fft.c (FFT kernel)
│   ├── quant_bands.rs  # quant_bands.c (energy decode: coarse/fine/fill)
│   ├── laplace.rs      # laplace.c (Laplace-distributed symbols)
│   ├── cwrs.rs         # cwrs.c (PVQ combinatorial indexing)
│   ├── pitch.rs        # pitch.c (postfilter)
│   └── modes.rs        # modes.c, static_modes_float.h (mode tables)
├── silk/
│   ├── mod.rs          # dec_API.c (SILK top-level decode)
│   ├── decode_frame.rs # decode_frame.c
│   ├── nlsf.rs         # NLSF_decode.c, NLSF_VQ.c, NLSF_stabilize.c, NLSF_VQ_weights_laroia.c
│   ├── lpc.rs          # NLSF2A.c, LPC_inv_pred_gain.c, bwexpander.c
│   ├── decode_pulses.rs# decode_pulses.c, shell_coder.c
│   ├── decode_core.rs  # decode_core.c (excitation + LPC synthesis)
│   ├── plc.rs          # PLC.c, CNG.c
│   ├── stereo.rs       # stereo_decode_pred.c, stereo_MS_to_LR.c
│   ├── resampler.rs    # resampler.c + resampler_private_*.c (8 files)
│   └── tables.rs       # tables_*.c (~10 files: gain, NLSF CB, pulses, pitch)
└── resampler.rs        # Output-rate decimation (CELT 48kHz -> target Fs)
```

## Implementation strategy (phased)

### Phase 0 - Repo bootstrap + gating

Deliverables:

- Cargo workspace with a core crate (e.g. `opus-decoder`). ✅
- CI: `cargo fmt`, `cargo clippy`, `cargo test`. ✅
- Conformance harness:
  - add a test runner that reads official vector format(s) ✅
  - decodes packets ✅
  - compares output quality to expected decode (reference metric) ✅
- Test vectors handling:
  - `xtask` downloads pinned artifacts + verifies checksums (SHA-1 today) ✅
  - deterministic manifest generation ✅
- Add per-packet `final_range` comparison from `.bit` metadata to `decoder.final_range()` (currently TODO). 🚧

Definition of done:

- Conformance harness reports both quality and `final_range`.
- Expected implementation gaps are explicit TODOs; harness behavior is trusted.

### Phase 1 - Bitstream/packet plumbing (foundation)

Port/implement:

- Packet parsing:
  - TOC parsing, frame count coding, per-frame sizes, padding rules ✅
  - enforce limits: max packet size, max duration (<= 120 ms), max frames ✅
- Error model: invalid packet -> error; no panics. ✅
- Low-level primitives:
  - saturating arithmetic helpers and constants/tables as needed ✅/🚧
- Entropy / range decoder:
  - CELT range decoder ✅
  - SILK integration over shared range coding semantics 🚧

Definition of done:

- Parser + range decoder(s) pass unit tests and vector smoke tests. ✅
- Fuzz target (smoke) does not panic/hang on random input. ✅

### Phase 2 - CELT decoder (CELT-only)

Port major CELT building blocks:

- Core CELT decode loop and mode data wiring (`celt_decoder.c`, `modes.c`, `static_modes_float.h`).
- Bit allocation (`rate.c`) - per-band bit budget (mandatory).
- Energy decode passes (`quant_bands.c`):
  - coarse energy
  - fine energy
  - fine energy fill
- TF (time-frequency) resolution flags (per-band).
- Spreading parameters and allocation trim decode (bit allocation controls).
- Band decoding / PVQ / normalization (`bands.c`, `cwrs.c`, `laplace.c`).
- Band folding when allocation is insufficient.
- Anti-collapse (`anti_collapse()`).
- Transient path:
  - `isTransient` decode
  - short blocks (M short windows) vs one long block
- IMDCT + overlap-add.
- Stereo coupling (2ch single-stream).
- Post-filter (comb/pitch filter decoded from bitstream).
- Deemphasis (`celt_sig_deemph`) after IMDCT.
- Silence / near-silence frame handling.

Recommended (optional, high-value):

- Differential harness against C libopus in tests (via `cc` in dev-dependencies):
  feed identical input to C and Rust decoders, compare sample-by-sample.
  This catches regressions that `opus_compare` may not expose quickly.

Definition of done:

- CELT-only vectors pass.
- CELT-only vectors also pass per-packet `final_range` comparison.

### Phase 3 - SILK decoder (SILK-only)

#### Phase 3a - SILK frame parsing + stereo unpacking + LBRR parsing

Port/implement:

- SILK frame header parse (VAD, LBRR flags).
- Parse/consume LBRR signaling even when FEC recovery is not used (required to avoid range coder desync).
- Stereo unpack/predictor signaling (`stereo_decode_pred.c`).
- Multi-frame packet handling (up to 3 internal SILK frames for 40/60 ms), including state sharing paths (e.g. NLSF interpolation continuity).

Definition of done:

- No range coder desync due to skipped SILK header/LBRR bits.
- Multi-frame SILK packets parse correctly.

#### Phase 3b - NLSF decode -> LPC (numerically sensitive)

Port/implement:

- NLSF two-stage VQ decode (`NLSF_decode.c`, `NLSF_VQ.c`, `NLSF_VQ_weights_laroia.c`).
- NLSF stabilization (`NLSF_stabilize.c`).
- NLSF interpolation between subframes.
- NLSF -> LPC conversion (`NLSF2A.c`, `table_LSF_cos.c`).
- LPC inverse prediction gain check (`LPC_inv_pred_gain.c`).
- Bandwidth expansion (`bwexpander.c`, `bwexpander_32.c`).

Definition of done:

- Stable LPC generation matching reference behavior across NB/MB/WB vectors.

#### Phase 3c - Pitch/LTP + excitation + synthesis (+ DTX/CNG)

Port/implement:

- Pitch lag decode (`decode_pitch.c` + contour tables).
- LTP (Long-Term Prediction) filter path.
- Shell coder / pulse decode (`shell_coder.c`, `decode_pulses.c`).
- Excitation reconstruction (`decode_core.c`) combining pulses + LTP + gain.
- LPC synthesis filter.
- DTX handling: empty SILK payload periods produce comfort noise (CNG) per state.

Definition of done:

- SILK speech + DTX segments decode without desync and with expected CNG behavior.

#### Phase 3d - SILK resampler (separate deliverable)

Port full polyphase SILK resampler:

- `resampler.c`
- `resampler_down2_3.c`
- `resampler_down2.c`
- `resampler_private_AR2.c`
- `resampler_private_down_FIR.c`
- `resampler_private_IIR_FIR.c`
- `resampler_private_up2_HQ.c`
- `resampler_rom.c`

Purpose:

- Resample SILK internal 8/12/16 kHz output to API-requested output `Fs`.

Definition of done:

- SILK-only decode passes at all output `Fs` via SILK resampler path.

#### Phase 3e - SILK stereo (mid-side -> L/R)

Port/implement:

- `stereo_decode_pred.c`
- `stereo_MS_to_LR.c`

Definition of done:

- Stereo SILK vectors pass with stable image reconstruction.

Phase 3 tables:

- Port SILK `tables_*.c` assets (~10 files: gains, NLSF codebooks NB/MB/WB, pulses/block, pitch contours, LSF cosine, etc.) in the sub-phase that uses each table, not as a disconnected late task.

Phase 3 Definition of done:

- SILK-only vectors pass at all internal rates (NB/MB/WB).
- SILK-only vectors pass per-packet `final_range` comparison.

### Phase 4 - Hybrid glue (SILK+CELT)

Port/implement the hybrid combiner:

- Delay compensation:
  - SILK and CELT have different lookahead; hybrid requires CELT/SILK alignment (CELT side offset ~2.7 ms).
- Band splitting/merging:
  - SILK decodes <= 8 kHz band, CELT decodes > 8 kHz band.
  - Merge requires precise time/band alignment, not naive sample addition.
- Mode transition smoothing:
  - SILK-only <-> Hybrid <-> CELT-only transitions need reference-compatible overlap/crossfade behavior.
- State sync across SILK/CELT during mode and bandwidth transitions.

Definition of done:

- Hybrid vectors pass (including transition-heavy sequences).
- Hybrid vectors pass per-packet `final_range` comparison.

### Phase 5 - Output sampling rates (8/12/16/24/48 kHz)

Implement correct two-path architecture:

- CELT path:
  - decode at 48 kHz
  - deterministic decimation/downsampling to target `Fs`.
- SILK path:
  - decode at internal `fs_kHz` (8/12/16)
  - SILK resampler converts to target `Fs` (polyphase FIR/IIR from Phase 3d).
- Hybrid path:
  - SILK resampled to 48 kHz + CELT at 48 kHz
  - aligned sum in 48 kHz domain
  - final decimation to target `Fs`.

Notes:

- "Internal decode at 48 kHz" is CELT-only. SILK never decodes internally at 48 kHz.
- Keep deterministic behavior and reference-compatible float semantics.

Definition of done:

- Conformance (`opus_compare`) passes at all `Fs` for mono/stereo single-stream decode in all modes.
- `final_range` comparison passes across modes and output rates.

### Phase 6 - PLC/FEC + streaming edge cases

Implement:

- PLC behavior (packet loss input).
- In-band FEC decode where supported/required by vectors.
- LBRR usage for recovery paths (LBRR parsing is already Phase 3a).
- Robust state reset after hard errors.

Definition of done:

- Vectors + targeted tests for loss patterns pass.
- Fuzzing finds no panics/hangs; input size/time limits enforced.

### Phase 7 - Hardening + ergonomics

Add:

- Stricter input limits (packet size, frame duration, nesting).
- Optional "soft failure" modes if needed by product (default remains strict).
- Docs and examples (packet -> PCM).

Definition of done:

- API docs cover reset/state semantics and edge-case behavior.
- Regression suite covers parser limits and transition/reset invariants.

### Phase 8 - Performance (stage 2, after correctness)

Only after conformance is green:

- Remove avoidable allocations, tighten hot loops.
- Optional SIMD (`std::arch`) behind per-arch feature flags.
- Profiling-driven changes only.
- If SIMD requires `unsafe`, keep it scoped, documented with `// SAFETY:`, and covered by tests (including Miri where applicable).

Definition of done:

- Measured speedup on representative workloads with no conformance regression.

## Call recording: stereo export (speaker separation)

Goal: produce a stereo PCM/WAV where each speaker is isolated:

- Left: Stream A (caller), mono decode
- Right: Stream B (callee), mono decode

Recommended approach (export-layer, not in core decoder):

- Maintain `DecoderA` and `DecoderB`, each configured as `channels=1` at the same `Fs`.
- For each time slice:
  - decode packet A -> mono PCM A (or PLC for missing)
  - decode packet B -> mono PCM B (or PLC for missing)
  - interleave to stereo: `L=A[i]`, `R=B[i]`

Alignment requirement:

- Real call recordings often need timestamp-based alignment (RTP timestamps / sample counters).
- Export tooling should accept per-packet timestamps (in samples at chosen `Fs`) and fill gaps with PLC or zeros.
- If one side ends earlier, pad with zeros (or PLC for a limited tail), but keep sample-accurate alignment.

This logic should live in a separate crate or CLI/tooling layer (not in the core decoder crate).

## Risk register (high-level)

- Conformance pitfalls:
  - integer overflow/rounding differences vs C
  - float semantics and denormal behavior differences
  - endian/shift semantics: must be explicit in Rust
  - range coder desync from skipped SILK header bits (especially LBRR parse)
- Complexity hotspots:
  - CELT bit allocation + energy passes + transients + postfilter/deemphasis
  - SILK NLSF -> LPC numerical stability
  - SILK resampler correctness
  - hybrid delay compensation and transitions
- Security:
  - crafted packets causing pathological CPU
  - avoid unbounded loops and allocations; enforce spec limits

## Next step (2026-02-22)

Immediate vertical slice progression:

- Complete Phase 0 `final_range` comparison in `tests/conformance_rfc.rs` (existing TODO).
- Finish Phase 2 CELT essentials needed for first passing CELT-only vector:
  - `rate.c` allocation path
  - `quant_bands.c` coarse/fine/fill passes
  - transient short-block path
  - postfilter + deemphasis
- Start Phase 3a scaffolding (`silk/` module, frame header parse with LBRR consumption, multi-frame packet support) before SILK synthesis internals.
- Keep focused vector runs with `OPUS_VECTOR=...` and add optional C-vs-Rust differential checks for newly completed decode paths.
