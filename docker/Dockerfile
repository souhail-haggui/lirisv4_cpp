# =========================================================
# Minimal Zephyr build environment
# GNU Arm Embedded Toolchain only - No Zephyr SDK
# =========================================================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# Minimal system dependencies for Zephyr builds
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    ninja-build \
    gperf \
    dfu-util \
    device-tree-compiler \
    wget \
    python3 \
    python3-pip \
    xz-utils \
    file \
    make \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Install GNU Arm Embedded Toolchain
# ---------------------------------------------------------
ENV ARM_GCC_VERSION=12.3.rel1

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_GCC_VERSION}/binrel/arm-gnu-toolchain-${ARM_GCC_VERSION}-x86_64-arm-none-eabi.tar.xz \
    && tar -xf arm-gnu-toolchain-${ARM_GCC_VERSION}-x86_64-arm-none-eabi.tar.xz \
    && mv arm-gnu-toolchain-${ARM_GCC_VERSION}-x86_64-arm-none-eabi /opt/arm-gcc \
    && rm arm-gnu-toolchain-${ARM_GCC_VERSION}-x86_64-arm-none-eabi.tar.xz

ENV PATH="/opt/arm-gcc/bin:${PATH}"

# ---------------------------------------------------------
# Install west and Python requirements
# ---------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir west \
    && pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ---------------------------------------------------------
# Toolchain environment variables
# ---------------------------------------------------------
ENV ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
ENV GNUARMEMB_TOOLCHAIN_PATH=/opt/arm-gcc

# ---------------------------------------------------------
# Git safe directory (needed for mounted volumes)
# ---------------------------------------------------------
RUN git config --global --add safe.directory '*'

# ---------------------------------------------------------
# Create non-root user
# ---------------------------------------------------------
ARG USER_NAME=haggui
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USER_NAME}

USER ${USER_NAME}

# ---------------------------------------------------------
# Default workdir
# ---------------------------------------------------------
WORKDIR /workspace

CMD ["/bin/bash"]
