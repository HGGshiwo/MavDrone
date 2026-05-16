# MavDrone for Python.
Python library to connect to a DJI drone using MSDK5 and Mavlink. 

## Installation
Python 3.12 or newer is required. Install the library using:

```commandline
pip install "mavdrone @ git+https://github.com/WUR-ABE/mavdrone.git@main#subdirectory=mavdrone_python"
```

## Usage
The libary expects a running instance of `mavlink-router`. To fly to a specific coordinate:

> **⚠️ Caution:** This will start the drone, takeoff and fly to the specified location (if within the distance limit of the drone). Test your code first using the DJI simulation mode.

```python
import asyncio

import numpy as np

from mavdrone.drone import Drone
from mavdrone.mission import create_mission


async def main() -> None:
    async with Drone() as drone:
        if input("Start mission? [y|n]:").lower() != "y":
            return

        await drone.arm()
        await drone.takeoff()

        # Fly towards GPS coordinate
        coordinate = np.array([[51.9863904, 5.6637806, 12.0]], dtype=np.float64)

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
```

See the [examples](examples) folder for more examples.
