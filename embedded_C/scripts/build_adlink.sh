#!/usr/bin/env bash
# Native ARM64 build for Ubuntu 20.04 and newer (CMake >= 3.16, C++17).
# Does not install packages, change the OS/compiler, or start the application.
# Traditional CMake commands also work without CMake preset support.
set -eu
source_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${VISTA_BUILD_DIR:-$source_dir/out/build/linux-adlink-ubuntu20-arm64}"
if [ "$(uname -m)" != "aarch64" ]; then
    echo "This script builds natively on ARM64 (aarch64), not on Windows/x86-64." >&2
    exit 1
fi
mkdir -p "$build_dir"
cd "$build_dir"
build_dir="$(pwd)"
cmake -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DVISTA_EXPECTED_ARCHITECTURE=arm64 \
    -DVISTA_ENABLE_REALSENSE=OFF \
    -DVISTA_BUILD_TESTS=ON "$source_dir"
cmake --build . -- -j"${VISTA_BUILD_JOBS:-2}"
ctest --output-on-failure
echo "Executable: $build_dir/vista_edge"
echo "Edit $build_dir/DeviceConfig.txt for the Linux serial port and Grafana host before running."
