from __future__ import annotations

from logging import getLogger
from queue import Queue
from re import search as regex_search
from threading import Event, Thread
from typing import TYPE_CHECKING

import cv2
import numpy as np

from mavdrone.exceptions import OpenCVNotCompiledWithGstreamerException

if TYPE_CHECKING:
    from typing import Callable

    from numpy.typing import NDArray

    from mavdrone.drone import Drone

log = getLogger(__name__)


class LiveStreamCamera:
    def __init__(self, url: str, buffer_size: int = 1, fps: int = 30) -> None:
        check_opencv_gstreamer_support()
        
        gstreamer_pipeline = (
            f"rtspsrc location={url} latency=50 ! "
            f"rtph264depay ! "
            f"h264parse ! "
            f"avdec_h264 ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink drop=1 max-buffers=1 sync=false"
        )

        self._capture = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

        self._last_frame_queue = Queue(maxsize=buffer_size)

        self._fps = fps

        self._callbacks: list[Callable[[NDArray[np.uint8]], bool]] = []
        self._thread_stop_event = Event()

    @property
    def fps(self) -> float:
        return self._fps

    def start(self) -> None:
        thread = Thread(target=self._update_thread)
        thread.daemon = True
        thread.start()

        _should_stop = False
        while not _should_stop:
            if (frame := self._last_frame_queue.get()) is not None:
                for cb in self._callbacks:
                    _should_stop = not cb(frame)

        self.stop()

    def stop(self) -> None:
        self._thread_stop_event.set()

    def add_callback(self, callback_fn: Callable[[NDArray[np.uint8]], bool]) -> None:
        self._callbacks.append(callback_fn)

    def remove_callbacks(self, callback_fn: Callable[[NDArray[np.uint8]], bool]) -> None:
        self._callbacks.remove(callback_fn)

    def clear_callbacks(self) -> None:
        self._callbacks.clear()

    def _update_thread(self) -> None:
        while not self._thread_stop_event.is_set():
            if self._capture.isOpened():
                ret, frame = self._capture.read()
                if ret:
                    while not self._last_frame_queue.empty():
                        self._last_frame_queue.get_nowait()
                    self._last_frame_queue.put(frame)

        self._capture.release()


async def log_gimbal_info(drone: Drone) -> None:
    _found = False
    async for gimbal_list in drone._system.gimbal.gimbal_list():
        for gimbal in gimbal_list.gimbals:
            log.info(
                f"Found Gimbal: ID={gimbal.gimbal_id}, "
                f"Model={gimbal.model_name}, "
                f"Vendor={gimbal.vendor_name}, "
                f"Device ID={gimbal.gimbal_device_id}"
            )
            _found = True
            break

        if _found:
            break


def check_opencv_gstreamer_support() -> None:
    # Check if OpenCV is compiled with Gstreamer
    if not regex_search(r"GStreamer:\s+YES", cv2.getBuildInformation()):
        raise OpenCVNotCompiledWithGstreamerException("OpenCV is not compiled with Gstreamer. Recompile OpenCV.")
