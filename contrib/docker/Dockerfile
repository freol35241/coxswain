# Container image for coxswain-hosted. Build from the repository root:
#   docker build -f contrib/docker/Dockerfile -t coxswain-hosted .
#
# Two stages: the toolchain stays in the builder, the runtime image carries
# the binary and glibc only. Both stages are bookworm so the binary's glibc
# matches. Toolchain version must match rust-toolchain.toml.

FROM rust:1.97.0-slim-bookworm AS build
# Empty for the release image. A SIL or bench image passes
# --build-arg CARGO_FEATURES=keelson-bus (D-034); release builds refuse
# manifests that are not self-sufficient, test builds accept them.
ARG CARGO_FEATURES=""
WORKDIR /src
COPY . .
RUN cargo build --release --locked -p coxswain-hosted ${CARGO_FEATURES:+--features "$CARGO_FEATURES"}

FROM debian:bookworm-slim
COPY --from=build /src/target/release/coxswain-hosted /usr/local/bin/coxswain-hosted
# Root is deliberate for now: serial and CAN access in a container is
# granted by --device and host networking, and a non-root user would also
# need the host's dialout gid mapped in. Revisit with a commissioning task.
ENTRYPOINT ["/usr/local/bin/coxswain-hosted"]
