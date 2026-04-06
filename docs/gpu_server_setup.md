# GPU Server Setup for OpenArm Pi0.5 Policy

## Hardware Requirements
- NVIDIA GPU with ≥8GB VRAM (RTX 3090 / A100 recommended)
- Pi0.5 base model requires ~6GB for inference (bfloat16)
- 16GB+ system RAM recommended

## Software Requirements
- Ubuntu 22.04
- NVIDIA Driver ≥535
- CUDA 12.x
- Python 3.11+
- `uv` package manager

## Setup Steps

### 1. Install NVIDIA Driver and CUDA
```bash
# Check GPU
nvidia-smi

# Install driver (if needed)
sudo apt install -y nvidia-driver-535

# Install CUDA 12
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-12-4
```

### 2. Install Python 3.11 and uv
```bash
sudo apt install -y python3.11 python3.11-venv python3.11-dev
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

### 3. Clone and Setup OpenPI
```bash
git clone https://github.com/AiSaurabhPatil/openpi.git
cd openpi
git submodule update --init --recursive
GIT_LFS_SKIP_SMUDGE=1 uv sync
```

### 4. Download Checkpoint
```bash
# The checkpoint is ~12GB
uv run python -c "
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='AiSaurabhPatil/openarm-pi05-finetuned',
    local_dir='./checkpoints/openarm_pi05',
)
"
```

### 5. Run Policy Server
```bash
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.85
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi05_openarm \
    --policy.dir=./checkpoints/openarm_pi05 \
    --port=8000
```

### 6. Verify Server
```bash
# From robot PC:
python -c "
import websockets, asyncio, json

async def test():
    async with websockets.connect('ws://<GPU_SERVER_IP>:8000') as ws:
        meta = json.loads(await ws.recv())
        print('Connected! Metadata:', meta)

asyncio.run(test())
"
```

## Network Configuration
- Open port 8000 on GPU server firewall
- Ensure robot PC can reach GPU server IP
- For local testing, use `ws://127.0.0.1:8000`

## Troubleshooting
- **OOM Error**: Reduce `XLA_PYTHON_CLIENT_MEM_FRACTION` to 0.7
- **Connection Refused**: Check firewall and server is running
- **Slow Inference**: Verify GPU is being used (`nvidia-smi` should show JAX process)
