from __future__ import annotations

import asyncio
from collections.abc import Coroutine
from typing import Any

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from transforms3d.euler import euler2quat, quat2euler

from geometry_msgs.msg import QuaternionStamped
from mavdrone_msgs.action import FlyToLocation, Land, TakeOff
from mavdrone_msgs.srv import Pause, RotateGimbal
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mavdrone.drone import Drone, MavSDKServerConnection

from mavdrone_ros.async_node import AsyncNode
from mavdrone_ros.distances import distance_2d_approx
from mavdrone_ros.livestream import LivestreamPublisher
from mavdrone_ros.log_handler import redirect_logging_to_ros
from mavdrone_ros.mavdrone_ros.mavdrone_parameters import Parameters
from mavdrone_ros.transform import TransformPublisher


class NoLivestreamAvailableException(Exception):
    pass


class MavDroneNode(AsyncNode):
    def __init__(self) -> None:
        super().__init__("mavdrone")

        redirect_logging_to_ros(self)

        self.parameters = Parameters()
        self.parameters.declare_parameters(self)
        self.parameters.get_parameters_from_ros(self)
        self.parameters.register_parameter_updates(self)

        self.drone = Drone(
            address=self.parameters.address,
            timeout=self.parameters.timeout,
            start_attempts=self.parameters.start_attempts,
            upload_attempts=self.parameters.upload_attempts,
            mavsdk_connection=MavSDKServerConnection("localhost", 5001),
        )

        self._exclusive_cb_group = MutuallyExclusiveCallbackGroup()
        self._parallel_cb_group = ReentrantCallbackGroup()

        # Initialize messages
        self._gps_msg = NavSatFix()
        self._gps_msg.header.frame_id = self.parameters.base_frame_id
        self._attitude_msg = QuaternionStamped()
        self._attitude_msg.header.frame_id = self.parameters.base_frame_id

        # Initialize action servers, services and publishers
        self._fly_to_location_action_server = ActionServer(
            self, FlyToLocation, "~/fly_to_location", self._fly_to_location_callback, callback_group=self._exclusive_cb_group
        )
        self._land_action_server = ActionServer(self, Land, "~/land", self._land_callback, callback_group=self._exclusive_cb_group)
        self._takeoff_action_server = ActionServer(
            self, TakeOff, "~/takeoff", self._takeoff_callback, callback_group=self._exclusive_cb_group
        )
        self._rotate_gimbal_service = self.create_service(
            RotateGimbal, "~/rotate_gimbal", self._rotate_gimbal_callback, callback_group=self._exclusive_cb_group
        )
        self._pause_drone_service = self.create_service(Pause, "~/pause", self._pause_callback, callback_group=self._parallel_cb_group)

        if self.parameters.publish_gps:
            self._gps_publisher = self.create_publisher(NavSatFix, "~/gps/fix", 1)
            self.create_timer(
                self.parameters.telemetry_update_rate,
                self._publish_gps,
                callback_group=self._parallel_cb_group,
            )
        if self.parameters.publish_attitude:
            self._attitude_publisher = self.create_publisher(QuaternionStamped, "~/gps/attitude", 1)
            self.create_timer(
                self.parameters.telemetry_update_rate,
                self._publish_attitude,
                callback_group=self._parallel_cb_group,
            )
        if self.parameters.publish_livestream:
            self.livestream_publisher = LivestreamPublisher(self, callback_group=self._parallel_cb_group)  # type: ignore[arg-type]

        if self.parameters.publish_transform:
            self.transform_publisher = TransformPublisher(self, callback_group=self._parallel_cb_group)  # type: ignore[arg-type]

        self.run_async(self.connect_to_drone())

    async def connect_to_drone(self) -> None:
        self.get_logger().info("Start connecting to drone...")

        await self.drone.connect()

        if self.parameters.publish_livestream and self.parameters.video_device == "stream":
            # When no camera connected, the connection will time out
            try:
                await asyncio.wait_for(
                    self.drone.start_video_stream(
                        self.get_parameter("video_component_id").get_parameter_value().integer_value,
                        self.get_parameter("video_stream_id").get_parameter_value().integer_value,
                    ),
                    10.0,
                )

                await asyncio.wait_for(self.drone.wait_for_stream(), timeout=10.0)

                if self.drone.status.video_stream_info is None:
                    raise NoLivestreamAvailableException("Cannot start mission because video stream is not enabled!")

                self.livestream_publisher.start()
            except asyncio.TimeoutError:
                self.get_logger().warning(
                    "Timeout while waiting for drone to start video stream, livestream will not be available. Is the camera connected?"
                )
        elif self.parameters.video_device.startswith("/dev"):
            self.livestream_publisher.start()

        self.get_logger().info("Init drone complete")

    async def arm_and_run(self, fn: Coroutine[Any, Any, None]) -> None:
        if not self.drone.armed:
            await self.drone.arm()

        await fn

    def destroy_node(self) -> None:
        self.livestream_publisher.stop()

        self._fly_to_location_action_server.destroy()
        self._land_action_server.destroy()
        self._takeoff_action_server.destroy()

        future = self.run_async(self.drone.disconnect())
        future.result()  # Wait to finish

        super().destroy_node()

    def _publish_gps(self) -> None:
        self._gps_msg.header.stamp = self.get_clock().now().to_msg()
        self._gps_msg.status.status = NavSatStatus.STATUS_FIX if self.drone.status.gps_fixed else NavSatStatus.STATUS_NO_FIX
        self._gps_msg.latitude = self.drone.status.position[0]
        self._gps_msg.longitude = self.drone.status.position[1]
        self._gps_msg.altitude = self.drone.status.position[2]
        self._gps_publisher.publish(self._gps_msg)

    def _publish_attitude(self) -> None:
        quaternion = euler2quat(
            np.deg2rad(self.drone.status.attitude[0]),
            np.deg2rad(self.drone.status.attitude[1]),
            np.deg2rad(-self.drone.status.attitude[2]),
            axes="sxyz",
        )
        self._attitude_msg.header.stamp = self.get_clock().now().to_msg()
        self._attitude_msg.quaternion.w = quaternion[0]
        self._attitude_msg.quaternion.x = quaternion[1]
        self._attitude_msg.quaternion.y = quaternion[2]
        self._attitude_msg.quaternion.z = quaternion[3]
        self._attitude_publisher.publish(self._attitude_msg)

    def _land_callback(self, goal_handle: ServerGoalHandle) -> Land.Result:
        self.get_logger().info("Land action received, starting landing procedure...")

        feedback = Land.Feedback()
        result = Land.Result()

        future = self.run_async(self.drone.land())

        while not future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                self.get_logger().info("Land canceled!")
                result.success = False
                result.message = "Land canceled!"
                return result

            feedback.altitude = self.drone.status.position[2]
            goal_handle.publish_feedback(feedback)

            self.get_clock().sleep_for(Duration(seconds=1))

        try:
            _ = future.result()
            result.success = True

            # Send final feedback with altitude
            feedback.altitude = self.drone.status.position[2]
            goal_handle.publish_feedback(feedback)

            self.get_logger().warning("Land complete!")

        except Exception as e:
            result.success = False

            self.get_logger().warning(f"Could not land: {e}")

        goal_handle.succeed()
        return result

    def _takeoff_callback(self, goal_handle: ServerGoalHandle) -> TakeOff.Result:
        self.get_logger().info("Takeoff action received, starting takeoff procedure...")

        feedback = TakeOff.Feedback()
        result = TakeOff.Result()

        future = self.run_async(self.arm_and_run(self.drone.takeoff()))

        while not future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                self.get_logger().info("Takeoff canceled!")
                result.success = False
                result.message = "Takeoff canceled!"
                return result

            feedback.altitude = self.drone.status.position[2]
            goal_handle.publish_feedback(feedback)

            self.get_clock().sleep_for(Duration(seconds=1))

        try:
            _ = future.result()

            # Send final feedback with altitude
            feedback.altitude = self.drone.status.position[2]
            goal_handle.publish_feedback(feedback)

            result.success = True

            self.get_logger().warning("Takeoff complete!")

        except Exception as e:
            result.success = False
            result.message = str(e)

            self.get_logger().warning(f"Could not takeoff: {e}")

        goal_handle.succeed()
        return result

    def _fly_to_location_callback(self, goal_handle: ServerGoalHandle) -> FlyToLocation.Result:
        request: FlyToLocation.Goal = goal_handle.request
        feedback = FlyToLocation.Feedback()
        result = FlyToLocation.Result()

        goal = np.array([request.latitude, request.longitude, request.altitude, request.heading_deg], dtype=np.float64)

        future = self.run_async(
            self.arm_and_run(
                self.drone.fly_to_location(
                    goal.reshape(-1, 4).astype(np.float32),
                    speed=request.speed,
                    gimbal_pitch=request.gimbal_pitch,
                    gimbal_yaw=request.gimbal_yaw,
                    drone_hold_time=self.parameters.drone_hold_time,
                    blocking=True,
                )
            )
        )

        while not future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                self.get_logger().info("FlyToLocation canceled!")
                result.success = False
                result.message = "FlyToLocation canceled!"
                return result

            feedback.distance_to_goal = distance_2d_approx(goal, self.drone.status.position)
            goal_handle.publish_feedback(feedback)

            self.get_clock().sleep_for(Duration(seconds=1))

        try:
            _ = future.result()
            result.success = True

            self.get_logger().warning("FlyToLocation complete!")

        except Exception as e:
            result.success = False
            result.message = str(e)

            self.get_logger().warning(f"Could not FlyToLocation: {e}")

        goal_handle.succeed()

        self.livestream_publisher.publish_latest_image()
        return result

    def _rotate_gimbal_callback(self, request: RotateGimbal.Request, response: RotateGimbal.Response) -> RotateGimbal.Response:
        r, p, y = quat2euler((request.orientation.w, request.orientation.x, request.orientation.y, request.orientation.z), axes="sxyz")

        future = self.run_async(self.drone.rotate_gimbal(np.rad2deg(r), np.rad2deg(p), np.rad2deg(y)))

        try:
            future.result()  # Wait for the rotation to complete
            response.success = True

            self.get_logger().info("Gimbal rotated successfully")
        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"Could not rotate gimbal: {e}")

        return response

    def _pause_callback(self, req: Pause.Request, response: Pause.Response) -> Pause.Response:
        future = self.run_async(self.drone._system.mission.is_mission_finished())
        if future.result():
            self.get_logger().info("Cannot pause mission since it is not started!")

            response.success = True
            return response

        if req.pause:
            future = self.run_async(self.drone._system.mission.pause_mission())
        else:
            future = self.run_async(self.drone._system.mission.start_mission())

        future.result()

        response.success = True
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavDroneNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
