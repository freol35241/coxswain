#!/usr/bin/env bash
# Provisions what the stock Rust image lacks. The Rust toolchain, components,
# and the thumbv7em target are pinned by rust-toolchain.toml; `rustup show`
# installs them now instead of on the first build. zenohd is pinned here and
# spawned by integration tests (CLAUDE.md: same image as CI).
set -euo pipefail

ZENOH_VERSION=1.9.0

rustup show

if ! command -v unzip >/dev/null; then
    sudo apt-get update && sudo apt-get install -y --no-install-recommends unzip
fi

tmp=$(mktemp -d)
curl -fsSL -o "$tmp/zenoh.zip" \
    "https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip"
unzip -q "$tmp/zenoh.zip" -d "$tmp"
sudo install -m 755 "$tmp/zenohd" /usr/local/bin/zenohd
rm -rf "$tmp"

zenohd --version
