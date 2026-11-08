#!/usr/bin/env bash
# Cross-build maburd for Hi3516CV610 with the AIC8800D80 radio. Needs:
#   CV610_TOOLCHAIN_BIN   bin/ of the OpenIPC arm-openipc-linux-musleabi toolchain
#   MABUR_CV610_SDK_INC   an OpenHisilicon checkout (the MPP headers)
#   MABUR_CV610_SDK_LIB   the directory holding the MPP .so files
# docs/hi3516cv610.md has what each one is and how the binary runs.
set -euo pipefail
cd "$(dirname "$0")/.."

: "${CV610_TOOLCHAIN_BIN:?}" "${MABUR_CV610_SDK_INC:?}" "${MABUR_CV610_SDK_LIB:?}"
export CV610_TOOLCHAIN_BIN
export MABUR_CV610_STAGING="$PWD/toolchain/cv610-staging"
LIBUSB_VER=1.0.27
LIBUSB_TAR="$PWD/toolchain/libusb-src/libusb-$LIBUSB_VER.tar.bz2"

if [ ! -e "$MABUR_CV610_STAGING/lib/libusb-1.0.a" ]; then
  [ -f "$LIBUSB_TAR" ] || curl -L --create-dirs -o "$LIBUSB_TAR" \
    "https://github.com/libusb/libusb/releases/download/v$LIBUSB_VER/libusb-$LIBUSB_VER.tar.bz2"
  # A tree of its own: the libusb beside the tarball is configured for the
  # glibc target.
  work="$(mktemp -d)"
  tar xf "$LIBUSB_TAR" -C "$work"
  (cd "$work/libusb-$LIBUSB_VER" &&
    ./configure --host=arm-openipc-linux-musleabi --prefix="$MABUR_CV610_STAGING" \
      --enable-static --disable-shared --disable-udev \
      CC="$CV610_TOOLCHAIN_BIN/arm-openipc-linux-musleabi-gcc" &&
    make -j"$(nproc)" && make install)
  rm -rf "$work"
fi

unset PKG_CONFIG_PATH || true
export PKG_CONFIG_LIBDIR="$MABUR_CV610_STAGING/lib/pkgconfig"
export PKG_CONFIG_SYSROOT_DIR=""
# devourer includes <libusb-1.0/libusb.h>, which needs the parent include dir.
export CPATH="$MABUR_CV610_STAGING/include${CPATH:+:$CPATH}"

cmake -S . -B build-cv610 -DCMAKE_TOOLCHAIN_FILE=cmake/arm-cv610.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF -DMABUR_BUILD_GS=OFF \
  -DMABUR_BUILD_LINKBENCH=OFF \
  -DMABUR_VENC_BACKEND=cv610 \
  -DMABUR_CV610_SDK_INC="$MABUR_CV610_SDK_INC" \
  -DMABUR_CV610_SDK_LIB="$MABUR_CV610_SDK_LIB" \
  -DDEVOURER_AIC8800=ON -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF \
  -DDEVOURER_JAGUAR2_8822B=OFF -DDEVOURER_JAGUAR2_8821C=OFF \
  -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=OFF -DDEVOURER_8733B=OFF \
  -DDEVOURER_KESTREL_8852B=OFF -DDEVOURER_KESTREL_8852C=OFF \
  -DDEVOURER_LOG_MAX_LEVEL=WARN
cmake --build build-cv610 -j"$(nproc)" --target maburd
mkdir -p out/cv610
"$CV610_TOOLCHAIN_BIN/arm-openipc-linux-musleabi-strip" build-cv610/drone/maburd -o out/cv610/maburd
ls -la out/cv610/maburd
