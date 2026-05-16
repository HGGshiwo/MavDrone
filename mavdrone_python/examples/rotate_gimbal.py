from __future__ import annotations

import asyncio
import logging

from mavdrone.drone import Drone
from mavdrone.helpers import log_gimbal_info

logging.basicConfig(level=logging.DEBUG)


async def main() -> None:
    async with Drone() as drone:
        await log_gimbal_info(drone)

        await drone.rotate_gimbal(0, 0.0, -90.0)

        await asyncio.sleep(2.0)

        await drone.rotate_gimbal(0, 0.0, -45.0)


if __name__ == "__main__":
    asyncio.run(main())
