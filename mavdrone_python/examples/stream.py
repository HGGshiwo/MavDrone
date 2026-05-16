from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

import cv2
import numpy as np

from mavdrone.drone import Drone
from mavdrone.enums import MAV_COMPONENT
from mavdrone.exceptions import NoStreamException
from mavdrone.helpers import LiveStreamCamera

if TYPE_CHECKING:
    from numpy.typing import NDArray


async def main() -> None:
    async with Drone() as drone:
        await drone.start_video_stream(MAV_COMPONENT.MAV_COMP_ID_CAMERA.value, 0)

        if drone.status.video_stream_info is None:
            raise NoStreamException("Video stream is not enabled! Cannot show stream in OpenCV.")

        print(f"Start streaming from {drone.status.video_stream_info.settings.uri}")

        camera = LiveStreamCamera(drone.status.video_stream_info.settings.uri)

        cv2.startWindowThread()
        cv2.namedWindow("capture")

        def _show(frame: NDArray[np.uint8]) -> bool:
            cv2.imshow("capture", frame)
            if cv2.waitKey(int((1 / camera.fps) * 1000)) & 0xFF == ord("q"):
                return False
            return True

        camera.add_callback(_show)

        try:
            camera.start()
        except KeyboardInterrupt:
            camera.stop()

        await drone.stop_video_stream(MAV_COMPONENT.MAV_COMP_ID_CAMERA.value, 0)

        cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(main())
