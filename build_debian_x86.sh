#!/usr/bin/env bash
# build_debian_x86.sh
# Author : Gab Young
# Purpose: Install all build dependencies and compile dablast on a fresh
#          Debian x86 system.  Run after a git clone or git pull.
#
# Usage:
#   chmod +x build_debian_x86.sh
#   ./build_debian_x86.sh

set -Eeuo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${REPO_DIR}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"

RTLSDR="${RTLSDR:-ON}"
AIRSPY="${AIRSPY:-OFF}"
SOAPYSDR="${SOAPYSDR:-OFF}"
KISS_FFT="${KISS_FFT:-OFF}"

JOBS="${JOBS:-$(nproc)}"

echo "==> Repo:      ${REPO_DIR}"
echo "==> Build dir: ${BUILD_DIR}"
echo "==> Type:      ${BUILD_TYPE}"
echo "==> Jobs:      ${JOBS}"
echo "==> Options:   RTLSDR=${RTLSDR} AIRSPY=${AIRSPY} SOAPYSDR=${SOAPYSDR} KISS_FFT=${KISS_FFT}"

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    librtlsdr-dev \
    libfftw3-dev \
    libfaad-dev \
    libmpg123-dev

if [[ "${AIRSPY}" == "ON" ]]; then
    sudo apt-get install -y libairspy-dev
fi

if [[ "${SOAPYSDR}" == "ON" ]]; then
    sudo apt-get install -y libsoapysdr-dev
fi

mkdir -p "${BUILD_DIR}"

echo "==> Configuring"
cmake -B "${BUILD_DIR}" -S "${REPO_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DRTLSDR="${RTLSDR}" \
    -DAIRSPY="${AIRSPY}" \
    -DSOAPYSDR="${SOAPYSDR}" \
    -DKISS_FFT="${KISS_FFT}"

echo "==> Building"
cmake --build "${BUILD_DIR}" -j"${JOBS}"

echo "==> Done"
echo "Binary:"
echo "  ${BUILD_DIR}/dablast"
