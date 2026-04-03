#!/bin/bash

set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "[ERROR] Please run as root: sudo $0" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt update
apt install -y software-properties-common
add-apt-repository -y ppa:deadsnakes/ppa
apt update
apt install -y \
  python3.12 \
  python3.12-dev \
  python3.12-venv \
  python3-pip \
  ffmpeg

echo "[INFO] Python 3.12 prerequisites installed"
echo "[INFO] Next: rebuild librealsense bindings against python3.12"
