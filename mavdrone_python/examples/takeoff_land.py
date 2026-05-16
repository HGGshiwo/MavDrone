from __future__ import annotations

import asyncio

from mavdrone.drone import Drone


async def main() -> None:
    async with Drone() as drone:
        if input("Arm drone? [y|n]:").lower() != "y":
            return

        await drone.arm()

        if input("Takeoff? [y|n]:").lower() != "y":
            return

        await drone.takeoff()

        if input("Land? [y|n]:").lower() != "y":
            return

        await drone.land()


if __name__ == "__main__":
    asyncio.run(main())
