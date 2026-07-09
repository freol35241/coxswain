# Coxswain dev environment

One container, both profiles: host builds/tests for iteration and replay tuning,
`thumbv7em-none-eabihf` for the H7 conn node. CI runs in the same image
(`devcontainers/ci`), so the no_std gate can't drift from the dev environment.

## Layout

```
.devcontainer/
  Dockerfile            # shared dev/CI image
  devcontainer.json     # cargo caches, extensions, USB passthrough (opt-in)
  69-probe-rs.rules     # debug probe udev rules (baked into image)
.github/workflows/
  ci.yml                # fmt, clippy, host tests, thumbv7em cross-build gate
rust-toolchain.toml     # pinned channel + target + components
```

## Daily commands

```
cargo test --workspace                      # host tests, replay harness
cargo build --target thumbv7em-none-eabihf -p coxswain-contract   # no_std check
cargo embed -p coxswain-conn-h753           # flash + RTT (firmware crate)
zenohd &                                    # local router for integration tests
```

The no_std gate in CI covers contract, estimator, guidance, supervisor, and
manifest. Build whichever of those a change touches before claiming it done.

## WSL2: attaching the debug probe

USB passthrough is **not on by default**: `runArgs` in devcontainer.json is
commented out. Note that `--device=/dev/bus/usb` makes the container refuse to
start when no USB device tree is present, which is the usual state on a desk with
nothing attached. Uncomment it for hardware sessions and rebuild the container.

Then, one-time per machine, on the Windows side (elevated once for `bind`):

```
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

The probe appears under `/dev/bus/usb` in WSL and the device mount picks it up.
Verify inside the container:

```
probe-rs list
```

Re-run `attach` after replugging the probe or restarting WSL. If enumeration is
flaky, temporarily switch the container to `--privileged` for the session.

The udev rules baked into the image do nothing inside the container: udev runs on
the host that creates the device nodes. Access inside the container comes from
the `dialout` and `plugdev` group membership set in the Dockerfile.

Serial devices (desk GNSS, RS-422 adapters) follow the same usbipd route and land
as `/dev/ttyACM*` / `/dev/ttyUSB*`.

## Version pins

Everything the image installs is pinned, so a rebuild months from now produces
the same environment CI ran against.

- Rust channel: `rust-toolchain.toml`, materialized at image build via
  `rustup show`. Bump in its own commit.
- zenohd: `ZENOH_VERSION` arg in the Dockerfile: keep in lockstep with the
  zenoh / zenoh-pico crate versions used on the vessel
- probe tooling: `PROBE_RS_VERSION`, `FLIP_LINK_VERSION`,
  `CARGO_BINUTILS_VERSION`, installed by a pinned `cargo-binstall`
  (`BINSTALL_VERSION`). Rebuild the container to bump.

## Caches

`CARGO_TARGET_DIR` and the cargo registry live in named volumes so cold builds
stay rare. Docker creates named volumes root-owned, so `postCreateCommand`
chowns both to `vscode` before anything builds, then compiles a throwaway crate
to prove the container can actually write there. A `--version` check cannot tell
you that.

If builds fail with `Permission denied` on `/workspaces/target-cache`, the
postCreate step did not run. Rebuild the container, or repair by hand:

```
sudo chown -R vscode:vscode /workspaces/target-cache /usr/local/cargo/registry
```
