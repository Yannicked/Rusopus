# Changelog

## 2026-03-05

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
