# Changelog

## 2026-03-09

- feat(silk): fixed mono Hybrid resampler carry by sourcing the 48 kHz handoff from `stereo_state.s_mid` instead of the stale mono history buffer; `testvector10` now passes (was `err=4.58`).
- milestone(rfc8251): all `12/12` RFC 8251 conformance vectors now pass.
- fix(hybrid): restored Hybrid libopus parity by skipping SILK redundancy parsing in Hybrid, restoring the CELT `start == 0` postfilter guard, and correcting SWB Hybrid CELT end band / `effEnd` handling.
- fix(silk): matched CELT redundancy decode end band to the packet bandwidth in SilkOnly transitions.
- milestone(opus-decoder): `testvector05` and `testvector06` now pass RFC conformance; current sweep status is `8/12` passing vectors.

## 2026-03-08

- feat(opus-decoder): ported decoder mode transition state handling for SILK/CELT carry-over, including redundancy state tracking and transition fade/reset flow.

## 2026-03-05

- feat(silk): Phase 3c excitation reconstruction + LPC synthesis complete; subframe gains, pitch/LTP, and Q14 LPC synthesis now match libopus on `testvector02` pkt0, with internal-rate PCM zero-padded to API frame size.
- feat(silk): completed Phase 3a entropy path for SILK-only packets: VAD/LBRR parsing, stereo signaling, main/LBRR indices+pulses consumption, `final_range` match for all `testvector02` packets, and corrected RFC vector manifest channel counts.
- checkpoint(pre-silk): removed decoder debug instrumentation in `opus-decoder` (`eprintln`, `OPUS_TRACE_*`, `OPUS_DISABLE_*`); kept only `OPUS_TRACE_PACKET_MAX_DELTA` in `conformance_rfc`.
- chore(lint): ran `cargo clippy -p opus-decoder --all-targets`; removed dead transient debug API, removed unused `CeltMode` metadata fields (`fs`, `eff_ebands`, `max_lm`, `nb_short_mdcts`), and removed unused `Complex32` import in MDCT path.

## 2026-03-04

- milestone: CELT mono decode passes RFC 8251 conformance (`testvector07`): `4186/4186` `final_range` bit-exact, `opus_compare` quality `98.1%` (`err=0.004610`), mono and stereo-coded packets decode correctly in mono output mode.
- feat(celt): unified decode_mem buffer, mono synthesis bit-exact. Restructured CELT synthesis to use single contiguous `decode_mem[2048+120]` matching libopus. Overlap, comb filter history, and IMDCT output share one buffer. testvector07 mono section (pkt 0–2127): max_delta=1 (float rounding only). Remaining error is entirely in stereo-coded section (pkt 2128+).
- fix(celt): aligned stereo collapse-mask TF mapping and stereo merge with libopus (`stereo_merge` side scaling semantics). Also fixed stereo N=1 lowband history propagation. `testvector07` vs live C: no `DELTA pkt` entries above threshold in conformance trace.

## 2026-02-26

- Completed mono CELT decode milestone: fixed TOC bandwidth-dependent band end, lowband fold wrap-around, `tf_change` block scaling in `quant_partition`, `itheta` floor quantization, and `LM=-1` handling in `bits2pulses`/`pulses2bits`.
- `testvector07`: `2128/4186` packets match (`100%` of mono-coded packets); first stereo-coded packet is `pkt2128`.

## 2026-02-22

- Rebuilt `OPUS_PORT_PLAN.md` to match libopus decoder requirements: fixed CELT/SILK sample-rate architecture, expanded Phases 2-5 (including SILK 3a-3e), added Decoder State Model, added `final_range` conformance gate, added Rust module mapping, added RFC 8251 errata note, and updated Next step.
