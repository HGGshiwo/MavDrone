from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Thread
import time
from typing import TYPE_CHECKING, cast

import cv2
import numpy as np
from numpy.typing import NDArray
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time

from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CameraInfo, Image

from mavdrone.helpers import check_opencv_gstreamer_support

from mavdrone_ros.buffer import Buffer

if TYPE_CHECKING:
    from mavdrone_ros.mavdrone_node import MavDroneNode


@dataclass
class Frame:
    stamp: Time
    img: NDArray[np.uint8]


class LivestreamPublisher:
    def __init__(self, node: MavDroneNode, callback_group: CallbackGroup | None = None) -> None:
        self._node = node

        check_opencv_gstreamer_support()

        self._livestream_thread = Thread(target=self._update_livestream, daemon=True)
        self._livestream_thread_stop_event = Event()
        self._frame_buffer: Buffer[Frame] = Buffer()
        self._cv_bridge = CvBridge()
        self._img_msg: Image | None = None

        self._video_crop_params: tuple[int, int, int, int] | None = None

        self._livestream_publisher = self._node.create_publisher(Image, "~/camera_image", 1)
        self._camera_info_publisher = self._node.create_publisher(
            CameraInfo, "~/camera_info", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

        self._image_at_location_publisher = None
        if self._node.parameters.publish_image_at_location:
            self._image_at_location_publisher = self._node.create_publisher(
                Image, "~/camera_image_at_location", QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            )

        self._camera_info_manager = CameraInfoManager(
            self._node,
            cname=self._node.parameters.camera_name,
            url=self._node.parameters.camera_info_file,
            namespace=self._node.get_name(),
        )
        self._camera_info_manager.loadCameraInfo()

        self._node.create_timer(
            self._node.parameters.livestream_update_rate,
            self._publish_livestream,
            callback_group=callback_group,
        )
        self._node.create_timer(
            1.0,
            self._publish_camera_info,
            callback_group=callback_group,
        )

    def start(self) -> None:
        self._livestream_thread.start()

    def stop(self) -> None:
        self._livestream_thread_stop_event.set()

    def publish_latest_image(self) -> None:
        if self._image_at_location_publisher is None:
            return

        if self._img_msg is None:
            self._node.get_logger().warning("There is no image!")
            return

        self._image_at_location_publisher.publish(self._img_msg)

    def _publish_livestream(self) -> None:
        try:
            if frame := self._frame_buffer.get():
                self._img_msg = self._cv_bridge.cv2_to_imgmsg(frame.img, encoding="bgr8")
                self._img_msg.header.stamp = frame.stamp.to_msg()
                self._img_msg.header.frame_id = self._node.parameters.image_frame_id
                self._livestream_publisher.publish(self._img_msg)
        except CvBridgeError as e:
            self._node.get_logger().warning(f"Cannot publish livestream image: {e}")

    def _publish_camera_info(self) -> None:
        self._camera_info_publisher.publish(self._camera_info_manager.getCameraInfo())

    def _update_livestream(self) -> None:
        frame = Frame(self._node.get_clock().now(), np.zeros((0, 0), dtype=np.uint8))

        if self._node.parameters.video_device == "stream":
            while (video_stream_info := self._node.drone.status.video_stream_info) is None:
                pass

            gstreamer_pipeline = (
                f"rtspsrc location={video_stream_info.settings.uri} latency=200 ! "  # higher latency = more buffering = better stability
                f"rtph264depay ! "
                f"h264parse ! "
                f"avdec_h264 ! "
                f"videoconvert ! "
                f"video/x-raw,format=BGR ! "
                f"appsink drop=0 max-buffers=100 sync=true"
            )

            video_capture = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
            video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            self._node.get_logger().info(f"Started livestream from {video_stream_info.settings.uri} in background...")
        elif self._node.parameters.video_device.startswith("/dev"):
            video_capture = cv2.VideoCapture(self._node.parameters.video_device, cv2.CAP_V4L2)
            video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            while not video_capture.isOpened():
                self._node.get_logger().info(
                    f"Waiting for video device {self._node.parameters.video_device} to be available...", throttle_duration_sec=5.0
                )
                time.sleep(1)

            self._node.get_logger().info("Calculating crop parameters...")
            self._video_crop_params = self.calculate_crop_parameters(cast(np.ndarray, video_capture.read()[1]))

            self._node.get_logger().info(f"Started livestream from {self._node.parameters.video_device} in background...")
        else:
            raise NotImplementedError(f"Video device '{self._node.parameters.video_device}' is not implemented!")

        try:
            while not self._livestream_thread_stop_event.is_set():
                if video_capture.isOpened():
                    # t = time.perf_counter()
                    # while time.perf_counter() - t < 0.01:
                    #     video_capture.grab()

                    ret, img = video_capture.read()

                    if ret:
                        frame.img = cast(NDArray[np.uint8], img)

                        if self._video_crop_params is not None:
                            frame.img = frame.img[
                                self._video_crop_params[1] : self._video_crop_params[1] + self._video_crop_params[3],
                                self._video_crop_params[0] : self._video_crop_params[0] + self._video_crop_params[2],
                            ]

                        frame.stamp = self._node.get_clock().now()
                        self._frame_buffer.put(frame)

            self._node.get_logger().info("Releasing video stream...")
            video_capture.release()
        except Exception as e:
            self._node.get_logger().error(f"Cannot read video stream: {e}")
            video_capture.release()

    @staticmethod
    def calculate_crop_parameters(image: NDArray[np.uint8]) -> tuple[int, int, int, int] | None:
        """Screen capture from the drone's remote is not full screen. This function calculates the coordinates of the edges of the video."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            return cast(tuple[int, int, int, int], cv2.boundingRect(largest_contour))
