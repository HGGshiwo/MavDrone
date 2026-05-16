from __future__ import annotations

from typing import cast

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from mavdrone_msgs.action import Land


class LandActionClient(Node):
    def __init__(self) -> None:
        super().__init__("land_action_client")

        self._action_client = ActionClient(self, Land, "mavdrone/land")

    def send_goal(self) -> Future:
        goal_msg = Land.Goal()

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

        self.get_logger().info("Land accepted by ActionServer")

        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg: ClientGoalHandle) -> None:
        feedback: Land.Feedback = feedback_msg.feedback  # type: ignore
        self.get_logger().info(f"Current altitude: {feedback.altitude:.2f}m")

    def get_result_callback(self, future: Future) -> None:
        result: Land.Result = future.result().result  # type: ignore

        if result.success:
            self.get_logger().info("Land successfull")
        else:
            self.get_logger().error(f"Land not successfull: {result.message}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = LandActionClient()

    if input("Start executing Land? [y|n]: ").lower() != "y":
        exit(1)

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
