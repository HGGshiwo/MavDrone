from __future__ import annotations

import asyncio

import numpy as np

from mavdrone.drone import Drone
from mavdrone.mission import create_mission
from mavdrone.utils import translate_latlong


async def main() -> None:
    async with Drone() as drone:
        if input("Start mission? [y|n]:").lower() != "y":
            return

        await drone.arm()
        await drone.takeoff()

        # Fly 10 meter to north
        coordinate = translate_latlong(drone.status.position, north=100.0, east=0.0)
        coordinate = np.hstack((coordinate, 12.0), dtype=np.float64)

        mission = create_mission(
            drone.status.position,
            [coordinate],
            speed=10.0,
            take_photo=False,
            drone_hold_time=0.0,
        )
        drone.set_mission(mission)

        await drone.upload_mission()
        await drone.start_mission()
        await drone.wait_till_mission_finished()

        await asyncio.sleep(10)
        await drone.return_to_launch()


if __name__ == "__main__":
    asyncio.run(main())
