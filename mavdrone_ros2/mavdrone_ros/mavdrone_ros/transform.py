from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from rclpy.callback_groups import CallbackGroup
from transforms3d.euler import euler2quat

from tf2_ros import TransformBroadcaster

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped

from utm import from_latlon, latitude_to_zone_letter, latlon_to_zone_number

if TYPE_CHECKING:
    from mavdrone_ros.mavdrone_node import MavDroneNode


class TransformPublisher:
    def __init__(self, node: MavDroneNode, callback_group: CallbackGroup | None = None) -> None:
        self._node = node

        self._tf_broadcaster = TransformBroadcaster(self._node)

        self.utm_zone: tuple[str, int] | None = None

        self._base_link_transform = TransformStamped()
        self._base_link_transform.header.frame_id = self._node.parameters.utm_frame_id
        self._base_link_transform.child_frame_id = self._node.parameters.base_frame_id
        self._roll_transform = TransformStamped()
        self._roll_transform.header.frame_id = self._node.parameters.roll_parent_frame_id
        self._roll_transform.child_frame_id = self._node.parameters.roll_child_frame_id
        self._pitch_transform = TransformStamped()
        self._pitch_transform.header.frame_id = self._node.parameters.pitch_parent_frame_id
        self._pitch_transform.child_frame_id = self._node.parameters.pitch_child_frame_id
        self._yaw_transform = TransformStamped()
        self._yaw_transform.header.frame_id = self._node.parameters.yaw_parent_frame_id
        self._yaw_transform.child_frame_id = self._node.parameters.yaw_child_frame_id

        self._node.create_timer(self._node.parameters.transform_update_rate, self._publish_transform, callback_group=callback_group)

    def _publish_transform(self) -> None:
        now = self._node.get_clock().now().to_msg()

        if np.isnan(self._node.drone.status.gimbal_angle).any():
            self._node.get_logger().warning("Cannot update gimbal transform, angles not available yet", throttle_duration_sec=5.0)
        else:
            self._update_and_publish_transform(
                self._roll_transform, now, rpy=(0.0, np.deg2rad(self._node.drone.status.gimbal_angle[0]), 0.0)
            )
            self._update_and_publish_transform(
                self._pitch_transform, now, rpy=(np.deg2rad(self._node.drone.status.gimbal_angle[1]), 0.0, 0.0)
            )
            self._update_and_publish_transform(
                self._yaw_transform, now, rpy=(0.0, 0.0, np.deg2rad(-self._node.drone.status.gimbal_angle[2]))
            )

        if not self._node.drone.status.gps_fixed:
            self._node.get_logger().warn("GPS fix not available, waiting for valid GPS data.", throttle_duration_sec=5.0)
            return

        try:
            # Force UTM zone to be the initial zone
            if self.utm_zone is None:
                zone_letter = latitude_to_zone_letter(self._node.drone.status.position[0])

                if zone_letter is None:
                    self._node.get_logger().error("Failed to determine UTM zone letter from latitude")
                    return

                self.utm_zone = (
                    zone_letter,
                    latlon_to_zone_number(self._node.drone.status.position[0], self._node.drone.status.position[1]),
                )

            easting, northing, *_ = from_latlon(
                self._node.drone.status.position[0],
                self._node.drone.status.position[1],
                force_zone_letter=self.utm_zone[0],
                force_zone_number=self.utm_zone[1],
            )

            self._update_and_publish_transform(
                self._base_link_transform,
                now,
                xyz=(easting, northing, self._node.drone.status.position[2]),
                rpy=(
                    np.deg2rad(self._node.drone.status.attitude[0]),
                    np.deg2rad(self._node.drone.status.attitude[1]),
                    np.deg2rad(-self._node.drone.status.attitude[2]),
                ),
            )

        except Exception as e:
            self._node.get_logger().error(f"Failed to convert GPS coordinates to UTM: {e}", throttle_duration_sec=5.0)
            return

    def _update_and_publish_transform(
        self,
        transform: TransformStamped,
        time_msg: TimeMsg,
        xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        transform.header.stamp = time_msg

        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]

        quaternion = euler2quat(*rpy, axes="sxyz")
        transform.transform.rotation.w = quaternion[0]
        transform.transform.rotation.x = quaternion[1]
        transform.transform.rotation.y = quaternion[2]
        transform.transform.rotation.z = quaternion[3]
        self._tf_broadcaster.sendTransform(transform)
