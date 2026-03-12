/// Emit linker directives for libopus, preferring `pkg-config` and falling
/// back to the local `/tmp/libopus-clean` build.
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    if pkg_config::probe_library("opus").is_ok() {
        return;
    }

    println!("cargo:rustc-link-search=native=/tmp/libopus-clean/.libs");
    println!("cargo:rustc-link-lib=static=opus");
    println!("cargo:rustc-link-lib=dylib=m");
}
