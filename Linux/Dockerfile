# syntax=docker/dockerfile:1

ARG UBUNTU_VERSION="22.04"

#
# Base Image
#

FROM --platform=$BUILDPLATFORM ubuntu:${UBUNTU_VERSION} AS base
ARG BUILD_REQUIREMENTS="\
    build-essential \
    cmake \
    ninja-build \
    libasound2-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libboost-dev \
    libcurl4-openssl-dev \
    libgtk-3-dev \
    libgif-dev \
    libglu1-mesa-dev \
    libharfbuzz-dev \
    libmpg123-dev \
    libopencv-dev \
    libopus-dev \
    libopusfile-dev \
    libsoundtouch-dev \
    libswresample-dev \
    libtiff-dev \
    libturbojpeg0-dev \
    libvorbis-dev \
    libwebp-dev \
    libxft-dev \
    uuid-dev \
    xorg-dev"

# Install apt packages

RUN --mount=type=cache,id=builder,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,id=builder,target=/var/cache/apt,sharing=locked \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ${BUILD_REQUIREMENTS}

#
# OpenSiv3Dのライブラリをビルド | Build OpenSiv3D Library
#

FROM base AS builder

ARG GCC_OPTIONS="-j4"

# Build OpenSiv3D sources

ARG TARGETARCH

RUN --mount=type=bind,source=/Siv3D,target=/src/Siv3D \
    --mount=type=bind,source=/Linux,target=/src/Linux \
    --mount=type=cache,id=siv3d-build-${TARGETARCH},target=/build,sharing=locked \
    cmake -G Ninja -B /build -DCMAKE_BUILD_TYPE=RelWithDebInfo /src/Linux \
 && cmake --build /build -- ${GCC_OPTIONS}

#
# ライブラリを作成 | Create Library
#
# Usage:
#     cd <path/to/OpenSiv3D>
#     docker build --file=Linux/Dockerfile --output <output_directory> --target lib .
#

# Install built files

FROM builder AS builder-lib

ARG TARGETARCH

RUN --mount=type=bind,source=/Siv3D,target=/src/Siv3D \
    --mount=type=cache,id=siv3d-build-${TARGETARCH},target=/build,sharing=locked \
    mkdir /src/Linux \
 && cmake --install /build --prefix /out

FROM scratch AS lib

COPY --from=builder-lib /out /

#
# debパッケージを作成 | Create DEB Package
#
# Usage:
#     cd <path/to/OpenSiv3D>
#     docker build --file=Linux/Dockerfile --output <output_directory> --target deb .
#

FROM builder AS builder-deb

ARG TARGETARCH

RUN --mount=type=bind,source=/Siv3D,target=/src/Siv3D \
    --mount=type=cache,id=siv3d-build-${TARGETARCH},target=/build,sharing=locked \
    mkdir /src/Linux \
 && mkdir /out \
 && cd /build \
 && cpack -B /out

FROM scratch AS deb

COPY --from=builder-deb /out/*.deb /

#
# OpenSiv3Dインストール済みのUbuntuイメージ | Siv3D-installed Ubuntu Image
#
# Usage:
#     cd <path/to/OpenSiv3D>
#     docker build --file=Linux/Dockerfile . -t opensiv3d
#     docker create opensiv3d
#

FROM base AS devenv

# Install prebuilt library
COPY --from=lib / /usr/local
