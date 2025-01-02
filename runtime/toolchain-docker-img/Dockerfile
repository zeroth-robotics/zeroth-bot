FROM ubuntu:22.04 AS builder

RUN echo "Acquire::http::Pipeline-Depth 0;" > /etc/apt/apt.conf.d/99custom && \
    echo "Acquire::http::No-Cache true;" >> /etc/apt/apt.conf.d/99custom && \
    echo "Acquire::BrokenProxy    true;" >> /etc/apt/apt.conf.d/99custom

RUN apt-get update && \
    apt-get install -y bc git curl build-essential file bison  \
      build-essential flex libssl-dev unzip wget cpio rsync protobuf-compiler && \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

RUN . "$HOME/.cargo/env" && \
    rustup toolchain install nightly && \
    rustup component add rust-src --toolchain nightly

RUN git clone https://github.com/kinsamanka/milkv-buildroot

RUN make O=$(pwd)/sdk -C milkv-buildroot milkv_duo_toolchain_defconfig && \
    cd sdk && \
    make sdk -j$(nproc)

# Remove unnecessary folders to reduce image size
RUN rm -rf /sdk/build /sdk/images \
    /root/.rustup/toolchains/stable* \
    /root/.rustup/toolchains/nightly-*/share/doc \
    /milkv-buildroot/.git \
    /milkv-buildroot/dl

# Use Ubuntu minimal image for the final stage
FROM ubuntu:22.04

# Install ca-certificates in the final stage as well
RUN apt-get update && apt-get install -y ca-certificates && rm -rf /var/lib/apt/lists/*

COPY --from=builder /sdk /sdk
COPY --from=builder /root/.cargo /root/.cargo
COPY --from=builder /root/.rustup /root/.rustup
COPY --from=builder /milkv-buildroot /milkv-buildroot
COPY --from=builder /bin /bin
COPY --from=builder /usr/bin /usr/bin
COPY --from=builder /lib /lib
COPY --from=builder /usr/lib /usr/lib
COPY --from=builder /etc/alternatives /etc/alternatives

RUN echo 'source ~/.cargo/env' >> ~/.bashrc
