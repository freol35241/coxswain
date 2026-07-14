//! Standard cortex-m-rt build script: exposes memory.x to the linker and
//! passes cortex-m-rt's link.x script directly, rather than through
//! .cargo/config.toml rustflags (the workspace root's config.toml stages
//! `linker = "flip-link"` for the real H7 firmware phase, unavailable in
//! this environment; the host test overrides that target's rustflags to
//! empty when building this crate, and this build script supplies the one
//! link argument this crate actually needs).
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
}
