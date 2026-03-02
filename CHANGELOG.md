# Changelog

## 2026-02-26

- Completed mono CELT decode milestone: fixed TOC bandwidth-dependent band end, lowband fold wrap-around, `tf_change` block scaling in `quant_partition`, `itheta` floor quantization, and `LM=-1` handling in `bits2pulses`/`pulses2bits`.
- `testvector07`: `2128/4186` packets match (`100%` of mono-coded packets); first stereo-coded packet is `pkt2128`.

## 2026-02-22

- Rebuilt `OPUS_PORT_PLAN.md` to match libopus decoder requirements: fixed CELT/SILK sample-rate architecture, expanded Phases 2-5 (including SILK 3a-3e), added Decoder State Model, added `final_range` conformance gate, added Rust module mapping, added RFC 8251 errata note, and updated Next step.
