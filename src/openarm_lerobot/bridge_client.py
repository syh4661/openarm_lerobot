"""OpenArm bridge client: reads robot state + cameras, queries OpenPI policy server, sends actions back.

Protocol: msgpack over websocket (matches OpenPI's WebsocketPolicyServer).
Uses OpenPI's msgpack_numpy format for numpy array serialization.
"""

import asyncio
import logging
import time
from typing import Any

import numpy as np
import websockets

# Use OpenPI's msgpack_numpy for compatible serialization
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import msgpack_numpy

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class OpenArmBridgeClient:
    """Connects OpenArm runtime to a remote OpenPI policy server."""

    def __init__(
        self,
        server_url: str = "ws://<GPU_SERVER_IP>:8000",
        default_prompt: str = "pick up the object",
    ):
        self.server_url = server_url
        self.default_prompt = default_prompt
        self.ws = None
        self._metadata = None
        self._packer = None

    def build_observation(
        self,
        state: np.ndarray,
        chest_image: np.ndarray,
        left_wrist_image: np.ndarray,
        right_wrist_image: np.ndarray,
        prompt: str | None = None,
    ) -> dict[str, Any]:
        """Build observation dict matching OpenArm Pi0.5 input schema."""
        return {
            "observation/state": state.astype(np.float32),
            "observation/chest_image": np.asarray(chest_image, dtype=np.uint8),
            "observation/left_wrist_image": np.asarray(
                left_wrist_image, dtype=np.uint8
            ),
            "observation/right_wrist_image": np.asarray(
                right_wrist_image, dtype=np.uint8
            ),
            "prompt": prompt or self.default_prompt,
        }

    async def connect(self) -> None:
        logger.info(f"Connecting to policy server at {self.server_url}...")
        self.ws = await websockets.connect(
            self.server_url, compression=None, max_size=None
        )

        # Server sends metadata as msgpack (first frame)
        raw = await self.ws.recv()
        if isinstance(raw, str):
            import json

            self._metadata = json.loads(raw)
        else:
            self._metadata = msgpack_numpy.unpackb(raw, raw=False)

        logger.info(f"Server metadata: {self._metadata}")

        # Initialize msgpack_numpy packer for numpy arrays (matches server)
        self._packer = msgpack_numpy.Packer()

    async def infer(self, observation: dict) -> np.ndarray:
        """Send observation and receive action chunk."""
        packed = self._packer.pack(observation)
        await self.ws.send(packed)

        raw = await self.ws.recv()
        if isinstance(raw, str):
            raise RuntimeError(f"Inference error:\n{raw}")

        response = msgpack_numpy.unpackb(raw, raw=False)
        actions = np.array(response["actions"], dtype=np.float32)
        return actions

    async def close(self) -> None:
        if self.ws:
            await self.ws.close()


async def main():
    """Demo: connect to policy server and run inference on dummy data."""
    client = OpenArmBridgeClient(server_url="ws://10.252.205.103:8000")
    await client.connect()

    obs = client.build_observation(
        state=np.zeros(16, dtype=np.float32),
        chest_image=np.zeros((224, 224, 3), dtype=np.uint8),
        left_wrist_image=np.zeros((224, 224, 3), dtype=np.uint8),
        right_wrist_image=np.zeros((224, 224, 3), dtype=np.uint8),
        prompt="pick up the object",
    )

    logger.info("Sending observation to GPU server...")
    t0 = time.time()
    actions = await client.infer(obs)
    dt = time.time() - t0
    logger.info(f"Inference took {dt * 1000:.0f}ms")
    logger.info(f"Action chunk shape: {actions.shape}")
    logger.info(f"Action range: [{actions.min():.3f}, {actions.max():.3f}]")

    await client.close()


if __name__ == "__main__":
    asyncio.run(main())
