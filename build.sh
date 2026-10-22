#!/bin/sh
# Linux build with GCC or Clang (C++20) and CMake 3.20+.
set -eu
cd "$(dirname "$0")"
OUT=${1:-out/ddp-atmos}
BUILD_DIR=${BUILD_DIR:-build-linux}
CXX=${CXX:-c++}
export CXX
JOBS=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --config Release --parallel "$JOBS"
mkdir -p "$(dirname "$OUT")"
cp "$BUILD_DIR/out/ddp-atmos" "$OUT"
echo "Build: $OUT"
