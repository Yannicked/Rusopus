# Test Data

This directory is for downloaded conformance vectors and local manifests.

Downloaded files are `.gitignore`'d. Use:

```bash
cargo xtask fetch-vectors
cargo xtask gen-manifest
```

Then point tests at them:

```bash
OPUS_TESTVECTORS_DIR="$(pwd)/testdata/opus_testvectors" \
  cargo test -p opus-decoder --test conformance_rfc -- --ignored
```

You also need a `vectors.txt` manifest next to the vectors.
Generate it with `cargo xtask gen-manifest` or see `testdata/vectors.example.txt`.
