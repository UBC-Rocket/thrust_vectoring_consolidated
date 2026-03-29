FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    file \
    git \
    libgl1-mesa-dev \
    libqt6opengl6-dev \
    libvulkan-dev \
    libxkbcommon-dev \
    ninja-build \
    patchelf \
    pkg-config \
    jq \
    qt6-base-dev \
    qt6-base-dev-tools \
    qt6-declarative-dev \
    qt6-declarative-dev-tools \
    qt6-quick3d-dev \
    qt6-quick3d-dev-tools \
    libqt6serialport6-dev \
    libqt6shadertools6-dev \
    qt6-tools-dev \
 && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    git clone --depth 1 https://github.com/probonopd/linuxdeployqt.git /tmp/linuxdeployqt; \
    cd /tmp/linuxdeployqt; \
    qmake6 linuxdeployqt.pro; \
    make -j"$(nproc)"; \
    install -m 0755 /tmp/linuxdeployqt/bin/linuxdeployqt /usr/local/bin/linuxdeployqt; \
    rm -rf /tmp/linuxdeployqt

RUN set -eux; \
    curl -fsSL https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage -o /tmp/appimagetool.AppImage; \
    file /tmp/appimagetool.AppImage | grep -q 'ELF 64-bit'; \
    chmod +x /tmp/appimagetool.AppImage; \
    install -m 0755 /tmp/appimagetool.AppImage /usr/local/bin/appimagetool.AppImage; \
    printf '%s\n' '#!/bin/sh' 'exec env ARCH=x86_64 /usr/local/bin/appimagetool.AppImage "$@"' > /usr/local/bin/appimagetool; \
    chmod +x /usr/local/bin/appimagetool; \
    rm -f /tmp/appimagetool.AppImage

WORKDIR /workspace/embedded-software/ground-station

ENTRYPOINT ["/bin/bash", "./distribution/Linux/build-appimage.sh"]
