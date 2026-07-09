# Coxswain dev/CI image: one image, both profiles (host + thumbv7em)
FROM mcr.microsoft.com/devcontainers/rust:1-bookworm

# ---- system deps ------------------------------------------------------------
# libudev-dev  : probe-rs
# protobuf-compiler : Keelson envelope codegen
# usbutils/picocom  : probe + serial debugging on the desk
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        pkg-config \
        libudev-dev \
        protobuf-compiler \
        usbutils \
        picocom \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ---- pinned zenoh router for integration tests ------------------------------
# Check https://github.com/eclipse-zenoh/zenoh/releases and bump deliberately;
# keep in lockstep with the zenoh/zenoh-pico versions used on the vessel.
ARG ZENOH_VERSION=1.9.0
RUN curl -fsSL -o /tmp/zenoh.zip \
      "https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip" \
    && unzip -j /tmp/zenoh.zip -d /usr/local/bin zenohd \
    && chmod +x /usr/local/bin/zenohd \
    && rm /tmp/zenoh.zip

# ---- udev rules for debug probes (ST-Link, J-Link, CMSIS-DAP) ---------------
# udev does not run in the container; these matter on the host that creates the
# device nodes. Kept here so a bare-metal user of this image gets them for free.
COPY 69-probe-rs.rules /etc/udev/rules.d/69-probe-rs.rules

# ---- device access for the non-root user ------------------------------------
RUN usermod -aG dialout,plugdev vscode

# ---- build artifact dir -----------------------------------------------------
# Owned by vscode so a freshly created named volume inherits that ownership.
# Pre-existing volumes are repaired in devcontainer.json's postCreateCommand.
RUN mkdir -p /workspaces/target-cache && chown vscode:vscode /workspaces/target-cache

# ---- rust toolchain & embedded tooling --------------------------------------
USER vscode

# Materialize the pinned channel at image build, with its targets and
# components. The file is the single source of truth: `rustup show` reads it and
# installs what it declares. Installing against the image's default toolchain
# instead would leave the pinned channel to be downloaded on first cargo run.
COPY --chown=vscode:vscode rust-toolchain.toml /tmp/toolchain/rust-toolchain.toml
RUN cd /tmp/toolchain && rustup show && rm -rf /tmp/toolchain

# cargo-binstall for fast binary installs. Version-pinned: an unpinned installer
# lets the dev image and CI drift apart, which is the failure D-016 exists to
# prevent.
ARG BINSTALL_VERSION=1.20.1
ARG PROBE_RS_VERSION=0.31.0
ARG FLIP_LINK_VERSION=0.1.12
ARG CARGO_BINUTILS_VERSION=0.4.0
RUN curl -fsSL "https://github.com/cargo-bins/cargo-binstall/releases/download/v${BINSTALL_VERSION}/cargo-binstall-x86_64-unknown-linux-musl.tgz" \
      | tar -xz -C "${CARGO_HOME}/bin" cargo-binstall \
    && cargo binstall -y \
        "probe-rs-tools@${PROBE_RS_VERSION}" \
        "flip-link@${FLIP_LINK_VERSION}" \
        "cargo-binutils@${CARGO_BINUTILS_VERSION}"

ENV CARGO_TARGET_DIR=/workspaces/target-cache

# Install claude code
RUN curl -fsSL https://claude.ai/install.sh | bash
