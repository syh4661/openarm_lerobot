#!/bin/bash

export LD_LIBRARY_PATH="/home/syhlabtop/src/librealsense/build-py312-rsusb/Release:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/home/syhlabtop/src/librealsense/build-py312-rsusb/Release:/home/syhlabtop/workspace/lerobot/src:${PYTHONPATH:-}"
