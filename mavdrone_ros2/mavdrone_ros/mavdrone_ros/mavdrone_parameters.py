from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEvent, ParameterEventHandler

from mavdrone.enums import MAV_COMPONENT


@dataclass
class Parameters:
    address: str = "udp://:14540"  # Default address for MAVLink connection
    video_device: str = "/dev/video0"  # or 'stream' to use RTSP stream
    timeout: float = 8.0  # Timeout MAVLink connection
    start_attempts: int = 3  # Number of attempts to start the drone
    upload_attempts: int = 3  # Number of attempts to upload missions
    video_component_id: int = MAV_COMPONENT.MAV_COMP_ID_CAMERA.value
    video_stream_id: int = 0
    camera_info_file: str = ""
    camera_name: str = "p1_camera"
    publish_gps: bool = True
    publish_attitude: bool = True
    publish_livestream: bool = True
    publish_transform: bool = True
    publish_image_at_location: bool = True
    telemetry_update_rate: float = 0.01
    livestream_update_rate: float = 0.01
    transform_update_rate: float = 0.01
    drone_hold_time: int = 2  # Time to hold position at each waypoint in seconds
    base_frame_id: str = "base_link"
    utm_frame_id: str = "utm"
    roll_parent_frame_id: str = "roll_base_link"
    roll_child_frame_id: str = "roll_link"
    pitch_parent_frame_id: str = "pitch_base_link"
    pitch_child_frame_id: str = "pitch_link"
    yaw_parent_frame_id: str = "yaw_base_link"
    yaw_child_frame_id: str = "yaw_link"
    image_frame_id: str = "mavdrone/camera_optical_link"

    def declare_parameters(self, node: Node) -> None:
        node.declare_parameter("address", self.address)
        node.declare_parameter("video_device", self.video_device)
        node.declare_parameter("timeout", self.timeout)
        node.declare_parameter("start_attempts", self.start_attempts)
        node.declare_parameter("upload_attempts", self.upload_attempts)
        node.declare_parameter("video_component_id", self.video_component_id)
        node.declare_parameter("video_stream_id", self.video_stream_id)
        node.declare_parameter("camera_info_file", self.camera_info_file)
        node.declare_parameter("camera_name", self.camera_name)
        node.declare_parameter("publish_gps", self.publish_gps)
        node.declare_parameter("publish_attitude", self.publish_attitude)
        node.declare_parameter("publish_livestream", self.publish_livestream)
        node.declare_parameter("publish_transform", self.publish_transform)
        node.declare_parameter("publish_image_at_location", self.publish_image_at_location)
        node.declare_parameter("telemetry_update_rate", self.telemetry_update_rate)
        node.declare_parameter("livestream_update_rate", self.livestream_update_rate)
        node.declare_parameter("transform_update_rate", self.transform_update_rate)
        node.declare_parameter("drone_hold_time", self.drone_hold_time)
        node.declare_parameter("base_frame_id", self.base_frame_id)
        node.declare_parameter("utm_frame_id", self.utm_frame_id)
        node.declare_parameter("roll_parent_frame_id", self.roll_parent_frame_id)
        node.declare_parameter("roll_child_frame_id", self.roll_child_frame_id)
        node.declare_parameter("pitch_parent_frame_id", self.pitch_parent_frame_id)
        node.declare_parameter("pitch_child_frame_id", self.pitch_child_frame_id)
        node.declare_parameter("yaw_parent_frame_id", self.yaw_parent_frame_id)
        node.declare_parameter("yaw_child_frame_id", self.yaw_child_frame_id)
        node.declare_parameter("image_frame_id", self.image_frame_id)

    def get_parameters_from_ros(self, node: Node) -> Parameters:
        self.address = node.get_parameter("address").get_parameter_value().string_value
        self.video_device = node.get_parameter("video_device").get_parameter_value().string_value
        self.timeout = node.get_parameter("timeout").get_parameter_value().double_value
        self.start_attempts = node.get_parameter("start_attempts").get_parameter_value().integer_value
        self.upload_attempts = node.get_parameter("upload_attempts").get_parameter_value().integer_value
        self.video_component_id = node.get_parameter("video_component_id").get_parameter_value().integer_value
        self.video_stream_id = node.get_parameter("video_stream_id").get_parameter_value().integer_value
        self.camera_info_file = node.get_parameter("camera_info_file").get_parameter_value().string_value
        self.camera_name = node.get_parameter("camera_name").get_parameter_value().string_value
        self.publish_gps = node.get_parameter("publish_gps").get_parameter_value().bool_value
        self.publish_attitude = node.get_parameter("publish_attitude").get_parameter_value().bool_value
        self.publish_livestream = node.get_parameter("publish_livestream").get_parameter_value().bool_value
        self.publish_transform = node.get_parameter("publish_transform").get_parameter_value().bool_value
        self.publish_image_at_location = node.get_parameter("publish_image_at_location").get_parameter_value().bool_value
        self.telemetry_update_rate = node.get_parameter("telemetry_update_rate").get_parameter_value().double_value
        self.livestream_update_rate = node.get_parameter("livestream_update_rate").get_parameter_value().double_value
        self.transform_update_rate = node.get_parameter("transform_update_rate").get_parameter_value().double_value
        self.drone_hold_time = node.get_parameter("drone_hold_time").get_parameter_value().integer_value
        self.base_frame_id = node.get_parameter("base_frame_id").get_parameter_value().string_value
        self.utm_frame_id = node.get_parameter("utm_frame_id").get_parameter_value().string_value
        self.roll_parent_frame_id = node.get_parameter("roll_parent_frame_id").get_parameter_value().string_value
        self.roll_child_frame_id = node.get_parameter("roll_child_frame_id").get_parameter_value().string_value
        self.pitch_parent_frame_id = node.get_parameter("pitch_parent_frame_id").get_parameter_value().string_value
        self.pitch_child_frame_id = node.get_parameter("pitch_child_frame_id").get_parameter_value().string_value
        self.yaw_parent_frame_id = node.get_parameter("yaw_parent_frame_id").get_parameter_value().string_value
        self.yaw_child_frame_id = node.get_parameter("yaw_child_frame_id").get_parameter_value().string_value
        self.image_frame_id = node.get_parameter("image_frame_id").get_parameter_value().string_value

        return self

    def register_parameter_updates(self, node: Node) -> None:
        handler = ParameterEventHandler(node)
        handler.add_parameter_event_callback(callback=self.parameter_update)

    def parameter_update(self, event: ParameterEvent) -> None:
        for p in event.changed_parameters:
            setattr(self, p.name, parameter_value_to_python(p.value))
