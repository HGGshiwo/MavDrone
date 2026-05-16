from __future__ import annotations

from typing import cast

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from mavdrone_msgs.action import FlyToLocation


class FlyToLocationActionClient(Node):
    def __init__(self) -> None:
        super().__init__("fly_to_location_action_client")

        self._action_client = ActionClient(self, FlyToLocation, "mavdrone/fly_to_location")

    def send_goal(self, longitude: float, latitude: float, altitude: float, heading: float, speed: float) -> Future:
        goal_msg = FlyToLocation.Goal()
        goal_msg.longitude = longitude
        goal_msg.latitude = latitude
        goal_msg.altitude = altitude
        goal_msg.heading_deg = heading
        goal_msg.gimbal_pitch = -90
        goal_msg.gimbal_yaw = 0
        goal_msg.speed = speed

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal to ActionServer...")

        goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)
        return goal_future

    def goal_response_callback(self, future: Future) -> None:
        goal_handle = cast(ClientGoalHandle | None, future.result())

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Takeoff rejected by ActionServer")
            return

        self.get_logger().info("FlyToLocation accepted by ActionServer")

        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg: ClientGoalHandle) -> None:
        feedback: FlyToLocation.Feedback = feedback_msg.feedback  # type: ignore
        self.get_logger().info(f"Distance to goal: {feedback.distance_to_goal:.2f}m")

    def get_result_callback(self, future: Future) -> None:
        result: FlyToLocation.Result = future.result().result  # type: ignore

        if result.success:
            self.get_logger().info("FlyToLocation successfull")
        else:
            self.get_logger().error(f"FlyToLocation not successfull: {result.message}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    goal_longitude = 5.660496563677793
    goal_latitude = 51.98925094986974
    goal_altitude = 20.0
    goal_heading_deg = 15.0
    goal_speed = 5.0

    action_client = FlyToLocationActionClient()

    if input("Start executing FlyToLocation? [y|n]: ").lower() != "y":
        exit(1)

    action_client.send_goal(goal_longitude, goal_latitude, goal_altitude, goal_heading_deg, goal_speed)

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
