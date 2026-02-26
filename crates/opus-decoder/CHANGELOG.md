# Changelog

## 2026-02-22

- Fixed code-1 packet parsing to reject odd payload lengths and added a unit test.
- Switched TOC routing to config-based `OpusMode` classification (SILK/Hybrid/CELT).
- Exposed decoder `final_range()` and added a soft conformance check against expected per-packet final range.
- Added recursive CELT band split (`quant_partition`) and kept MDCT backward path bit-exact (impulse-verified); quality debugging remains WIP.
