from __future__ import annotations

from typing import Any, cast

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.wait_for_message import wait_for_message
from transforms3d.euler import quat2euler

from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from mavdrone_msgs.action import FlyToLocation
from sensor_msgs.msg import NavSatFix

from utm import latitude_to_zone_letter, latlon_to_zone_number, to_latlon


class GoalPoseServerNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_server")

        self.declare_parameter("utm_frame_id", "utm")
        self.declare_parameter("fly_to_location_action_name", "/mavdrone/fly_to_location")
        self.declare_parameter("min_altitude", 12.0)

        self.utm_frame_id = self.get_parameter("utm_frame_id").get_parameter_value().string_value
        self.min_altitude = self.get_parameter("min_altitude").get_parameter_value().double_value

        self.fly_to_location_action_client = ActionClient(
            self,
            FlyToLocation,
            self.get_parameter("fly_to_location_action_name").get_parameter_value().string_value,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to goal poses
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_pose_callback, 10)

        self._current_goal_handle = None

        if (utm_zone := self._resolve_utm_zone()) is None:
            self.get_logger().warning("Could not resolve UTM zone, assuming U31")
            utm_zone = ("U", 31)

        self.utm_zone = utm_zone

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info("Received goal pose")

        try:
            transform = self.tf_buffer.lookup_transform(
                self.utm_frame_id,
                msg.header.frame_id,
                Time(),  # Transformation is static, so use latest one
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f"Could not get transform from {msg.header.frame_id} to {self.utm_frame_id}: {e}")
            return

        pose_utm = do_transform_pose(msg.pose, transform)

        utm_x = pose_utm.position.x
        utm_y = pose_utm.position.y
        altitude = max(pose_utm.position.z, self.min_altitude)

        try:
            lat, lon = to_latlon(utm_x, utm_y, zone_letter=self.utm_zone[0], zone_number=self.utm_zone[1])
        except Exception as e:
            self.get_logger().error(f"Failed to convert UTM to lat/lon: {e}")
            return

        _, _, heading = quat2euler(
            [pose_utm.orientation.w, pose_utm.orientation.x, pose_utm.orientation.y, pose_utm.orientation.z], axes="sxyz"
        )
        heading_deg = (-np.rad2deg(heading) + 90) % 360

        self.get_logger().info(f"Goal: lat: {lat:.6f}, lon: {lon:.6f}, alt: {altitude:.2f}, heading: {heading_deg:.2f}deg")

        self._send_fly_to_location_goal(lat, lon, altitude, heading_deg)

    def _send_fly_to_location_goal(self, lat: float, lon: float, alt: float, heading: float) -> None:
        if not self.fly_to_location_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("FlyToLocation action server not available")
            return

        goal_msg = FlyToLocation.Goal()
        goal_msg.latitude = lat
        goal_msg.longitude = lon
        goal_msg.altitude = alt
        goal_msg.heading_deg = heading
        goal_msg.speed = 5.0

        self.get_logger().info("Sending goal to FlyToLocation action server...")
        send_goal_future = self.fly_to_location_action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = cast(ClientGoalHandle | None, future.result())
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        self.get_logger().info("Goal accepted by action server")
        self._current_goal_handle = goal_handle
        result_future: Future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg: ClientGoalHandle) -> None:
        feedback: FlyToLocation.Feedback = feedback_msg.feedback  # type: ignore
        self.get_logger().info(f"Flying towards goal: {feedback.distance_to_goal:.0f}m", throttle_duration_sec=5.0)

    def _result_callback(self, future: Future) -> None:
        result: FlyToLocation.Result = future.result().result  # type: ignore
        if result.success:
            self.get_logger().info("FlyToLocation completed")
        else:
            self.get_logger().error(f"Could not complete FlyToLocation: {result.message}")

    def _resolve_utm_zone(self, timeout: Duration = Duration(seconds=10)) -> tuple[str, int] | None:
        self.get_logger().info("Trying to resolve UTM zone...")

        start_time = self.get_clock().now()

        while self.get_clock().now() - start_time < timeout:
            topics_and_types = self.get_topic_names_and_types()

            for topic, types in topics_and_types:
                if "sensor_msgs/msg/NavSatFix" in types:
                    self.get_logger().info(f"Subscribing to {topic}...", once=True)

                    if (msg := cast(NavSatFix | None, wait_for_message(NavSatFix, self, topic, time_to_wait=2)[1])) is not None:
                        if not np.isnan(msg.latitude) and not np.isnan(msg.longitude):
                            zone_letter = latitude_to_zone_letter(msg.latitude)
                            zone_number = latlon_to_zone_number(msg.latitude, msg.longitude)

                            if zone_letter is None:
                                return None

                            self.get_logger().info(f"Using UTM zone {(zone_letter, zone_number)}...")
                            return (zone_letter, zone_number)

        return None


def main(args: list[Any] | None = None) -> None:
    rclpy.init(args=args)
    node = GoalPoseServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
