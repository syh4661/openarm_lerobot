#!/bin/bash

set -euo pipefail

SRC_DIR="/home/syhlabtop/src/librealsense"
BUILD_DIR="/home/syhlabtop/src/librealsense/build-py312-rsusb"

mkdir -p "$BUILD_DIR"

cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3.12 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.12 \
  -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so

cmake --build "$BUILD_DIR" --target pyrealsense2 -j4

echo "[INFO] Built cp312 pyrealsense2 in $BUILD_DIR/Release"
