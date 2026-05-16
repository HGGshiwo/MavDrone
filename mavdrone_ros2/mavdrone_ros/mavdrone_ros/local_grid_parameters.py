from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node


@dataclass
class Parameters:
    frame_id: str = "world"
    child_frame_id: str = "base_link"
    navsatfix_topic: str = ""
    default_transform_x: float = 0.0
    default_transform_y: float = 0.0
    default_transform_z: float = 0.0
    default_transform_qx: float = 0.0
    default_transform_qy: float = 0.0
    default_transform_qz: float = 0.0
    default_transform_qw: float = 1.0

    def declare_parameters(self, node: Node) -> None:
        node.declare_parameter("frame_id", self.frame_id)
        node.declare_parameter("child_frame_id", self.child_frame_id)
        node.declare_parameter("navsatfix_topic", self.navsatfix_topic)
        node.declare_parameter("default_transform_x", self.default_transform_x)
        node.declare_parameter("default_transform_y", self.default_transform_y)
        node.declare_parameter("default_transform_z", self.default_transform_z)
        node.declare_parameter("default_transform_qx", self.default_transform_qx)
        node.declare_parameter("default_transform_qy", self.default_transform_qy)
        node.declare_parameter("default_transform_qz", self.default_transform_qz)
        node.declare_parameter("default_transform_qw", self.default_transform_qw)

    def get_parameters_from_ros(self, node: Node) -> Parameters:
        self.frame_id = node.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = node.get_parameter("child_frame_id").get_parameter_value().string_value
        self.navsatfix_topic = node.get_parameter("navsatfix_topic").get_parameter_value().string_value
        self.default_transform_x = node.get_parameter("default_transform_x").get_parameter_value().double_value
        self.default_transform_y = node.get_parameter("default_transform_y").get_parameter_value().double_value
        self.default_transform_z = node.get_parameter("default_transform_z").get_parameter_value().double_value
        self.default_transform_qx = node.get_parameter("default_transform_qx").get_parameter_value().double_value
        self.default_transform_qy = node.get_parameter("default_transform_qy").get_parameter_value().double_value
        self.default_transform_qz = node.get_parameter("default_transform_qz").get_parameter_value().double_value
        self.default_transform_qw = node.get_parameter("default_transform_qw").get_parameter_value().double_value

        return self
