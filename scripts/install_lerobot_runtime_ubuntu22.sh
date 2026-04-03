#!/bin/bash

set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "[ERROR] Please run as root: sudo $0" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt update
apt install -y \
  python3-pip \
  python3.10-venv \
  ffmpeg

echo "[INFO] System prerequisites installed"
echo "[INFO] Next step: run bootstrap_lerobot_runtime.sh as your normal user"
