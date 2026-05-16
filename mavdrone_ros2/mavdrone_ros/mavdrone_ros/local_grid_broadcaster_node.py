from typing import cast

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import NavSatFix

from utm import from_latlon

from mavdrone_ros.mavdrone_ros.local_grid_parameters import Parameters


class LocalGridBroadcasterNode(Node):
    def __init__(self):
        super().__init__("local_grid_broadcaster")

        self.parameters = Parameters()
        self.parameters.declare_parameters(self)
        self.parameters.get_parameters_from_ros(self)

        self.broadcaster = TransformBroadcaster(self)

        if self.parameters.navsatfix_topic == "":
            self.current_transform = self.default_transform()
        else:
            self.current_transform = self.get_transform_from_topic()

        self.timer = self.create_timer(0.01, self.broadcast_transform)

        self.get_logger().info(f"Broadcasting transform from '{self.parameters.frame_id}' to '{self.parameters.child_frame_id}'")

    def default_transform(self) -> Transform:
        t = Transform()
        t.translation.x = self.parameters.default_transform_x
        t.translation.y = self.parameters.default_transform_y
        t.translation.z = self.parameters.default_transform_z
        t.rotation.x = self.parameters.default_transform_qx
        t.rotation.y = self.parameters.default_transform_qy
        t.rotation.z = self.parameters.default_transform_qz
        t.rotation.w = self.parameters.default_transform_qw
        return t

    def get_transform_from_topic(self, timeout: Duration = Duration(seconds=60.0)) -> Transform:
        start_time = self.get_clock().now()
        topic_name = self.parameters.navsatfix_topic

        while self.get_clock().now() - start_time < timeout:
            self.get_logger().info(f"Trying to resolve default transform from '{topic_name}'...", throttle_duration_sec=5.0)

            if (msg := cast(NavSatFix | None, wait_for_message(NavSatFix, self, topic_name, time_to_wait=2)[1])) is not None:
                if not np.isnan(msg.latitude) and not np.isnan(msg.longitude):
                    eastings, northings, _, _ = from_latlon(msg.latitude, msg.longitude)

                    t = Transform()
                    t.translation.x = eastings
                    t.translation.y = northings
                    t.translation.z = 0
                    t.rotation.x = 0
                    t.rotation.y = 0
                    t.rotation.z = 0
                    t.rotation.w = 1
                    return t

        raise RuntimeError("Could not resolve initial position for local grid!")

    def broadcast_transform(self) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parameters.frame_id
        tf_msg.child_frame_id = self.parameters.child_frame_id
        tf_msg.transform = self.current_transform

        self.broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init()

    try:
        node = LocalGridBroadcasterNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
