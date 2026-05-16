from __future__ import annotations

from logging import getLogger
from typing import TYPE_CHECKING

import numpy as np

from mavsdk.mission_raw import MissionItem

from mavdrone.enums import (
    GIMBAL_MANAGER_FLAGS,
    MAV_CMD,
    MAV_FRAME,
    MAV_MISSION_TYPE,
    SET_FOCUS_TYPE,
)
from mavdrone.exceptions import MissionHasTooLowAltitudeException

if TYPE_CHECKING:
    from numpy.typing import NDArray

log = getLogger("mission")


def create_camera_calibration_mission(
    current_position: NDArray[np.float64],
    start_position: NDArray[np.float64],
    gimbal_pitch: float = -90.0,
    gimbal_yaw: float = 0.0,
    drone_yaw: float = float("nan"),
) -> list[MissionItem]:
    mission_items = []

    waypoint = start_position.copy()

    mission_items.append(create_waypoint_item(len(mission_items), current_position, yaw_heading=drone_yaw))

    # Use current height if not defined
    if waypoint.shape[0] == 2:
        waypoint = np.hstack((waypoint, current_position[2]))

    if waypoint[2] < 8.0:
        raise MissionHasTooLowAltitudeException(f"Altitude of mission item is lower than 8.0 meters: {waypoint[2]}")

    mission_items.append(create_waypoint_item(len(mission_items), waypoint, yaw_heading=drone_yaw))

    if not np.isnan(gimbal_pitch) or not np.isnan(gimbal_yaw):
        mission_items.append(create_gimbal_item(len(mission_items), gimbal_pitch, gimbal_yaw, yaw_lock=True))

    mission_items.append(create_delay_item(len(mission_items), 1.0))
    mission_items.append(create_set_focus_type(len(mission_items), SET_FOCUS_TYPE.FOCUS_TYPE_AUTO))
    mission_items.append(create_delay_item(len(mission_items), 1.0))
    mission_items.append(create_set_focus_type(len(mission_items), SET_FOCUS_TYPE.FOCUS_TYPE_METERS))

    if not np.isnan(gimbal_pitch) or not np.isnan(gimbal_yaw):
        mission_items.append(create_gimbal_item(len(mission_items), gimbal_pitch, gimbal_yaw, yaw_lock=True))

    mission_items.append(create_delay_item(len(mission_items), 2.0))

    return mission_items


def create_mission(
    current_position: NDArray[np.float64],
    waypoints: list[NDArray[np.float64]],
    speed: float = 10.0,
    take_photo: bool = False,
    drone_hold_time: float = 0,
    drone_yaw: float = float("nan"),  # Yaw during flight
) -> list[MissionItem]:
    mission_items = []

    if speed != 10:
        mission_items.append(create_change_speed_item(len(mission_items), speed))

    waypoints = list(waypoints)
    for i in range(len(waypoints)):
        # Use current height if not defined
        if waypoints[i].shape[0] == 2:
            waypoints[i] = np.hstack((waypoints[i], current_position[2]))

        # Wait till after the yaw change
        delay_after_yaw_change = None
        _hold_time = drone_hold_time
        if waypoints[i].shape[0] == 4 and drone_hold_time > 0:
            delay_after_yaw_change = drone_hold_time
            _hold_time = 0.0

        # Only add waypoint when it's new, otherwise just add the actions
        if i == 0 or not np.array_equal(waypoints[i][:2], waypoints[i - 1][:2]):
            mission_items.append(create_waypoint_item(len(mission_items), waypoints[i], _hold_time, drone_yaw))

        if waypoints[i].shape[0] == 4:
            mission_items.append(create_rotate_yaw_item(len(mission_items), waypoints[i][3]))

        if delay_after_yaw_change is not None:
            mission_items.append(create_delay_item(len(mission_items), delay_after_yaw_change))

        if take_photo:
            mission_items.append(create_take_photo_item(len(mission_items)))

    return mission_items


def create_take_photo_item(seq: int) -> MissionItem:
    return MissionItem(
        seq,  # SEQ
        MAV_FRAME.MAV_FRAME_MISSION,  # FRAME
        MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE,
        1 if seq == 0 else 0,  # CURRENT
        1,  # AUTOCONTINUE
        0,  # PARAM1
        0,  # INTERVAL
        1,  # TOTAL_IMAGES
        0,  # SEQUENCE
        0,
        0,
        0,
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,  # MISSIONTYPE
    )


def create_delay_item(seq: int, delay: int) -> MissionItem:
    return MissionItem(
        seq,  # SEQ,
        MAV_FRAME.MAV_FRAME_MISSION,
        MAV_CMD.MAV_CMD_NAV_DELAY,
        1 if seq == 0 else 0,  # CURRENT
        1,  # AUTOCONTINUE,
        delay,
        0,
        1,
        0,
        0,
        0,
        0,
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,  # MISSIONTYPE
    )


def create_rotate_yaw_item(seq: int, yaw_angle: int) -> MissionItem:
    return MissionItem(
        seq,  # SEQ,
        MAV_FRAME.MAV_FRAME_MISSION,
        MAV_CMD.MAV_CMD_CONDITION_YAW,
        1 if seq == 0 else 0,  # CURRENT
        1,  # AUTOCONTINUE
        yaw_angle,  # ANGLE
        0,  # ANGLE_SPEED
        -1 if yaw_angle <= 180 else 1,  # DIRECTION
        0,  # RELATIVE
        0,
        0,
        0,
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,  # MISSIONTYPE
    )


def create_gimbal_item(
    seq: int,
    gimbal_pitch: float,
    gimbal_yaw: float,
    pitch_lock: bool = False,
    yaw_lock: bool = False,
) -> MissionItem:
    flags = GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
    flags |= GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_ROLL_LOCK

    if pitch_lock:
        flags |= GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_PITCH_LOCK

    if yaw_lock:
        flags |= GIMBAL_MANAGER_FLAGS.GIMBAL_MANAGER_FLAGS_YAW_LOCK

    return MissionItem(
        seq,  # SEQ
        MAV_FRAME.MAV_FRAME_MISSION,  # FRAME
        MAV_CMD.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        1 if seq == 0 else 0,  # CMD  # CURRENT
        1,  # AUTOCONTINUE
        gimbal_pitch,  # PITCH ANGLE
        gimbal_yaw,  # YAW ANGLE
        1,  # PITCH RATE
        0,  # YAW RATE
        flags,  # GIMBAL FLAGS
        0,  # PARAM 6
        0,  # GIMBAL DEVICE ID
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,  # MISSIONTYPE
    )


def create_waypoint_item(
    seq: int,
    waypoint: NDArray[np.float64],
    hold_time: float = 0,
    yaw_heading: float = float("nan"),
) -> MissionItem:
    if not np.isnan(yaw_heading):
        yaw_heading = (yaw_heading + 180) % 360 - 180  # Normalize heading

    return MissionItem(
        seq,  # SEQ
        MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # FRAME
        MAV_CMD.MAV_CMD_NAV_WAYPOINT,
        1 if seq == 0 else 0,  # CMD  # CURRENT
        1,  # AUTOCONTINUE
        hold_time,  # HOLD_TIME
        0,  # PARAM2
        0,  # PARAM3
        yaw_heading,  # YAW_HEADING
        int(waypoint[0] * 1e7),  # X
        int(waypoint[1] * 1e7),  # Y
        waypoint[2],  # Z
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,  # MISSIONTYPE
    )


def create_change_speed_item(seq: int, speed: float) -> MissionItem:
    return MissionItem(
        seq,  # SEQ
        MAV_FRAME.MAV_FRAME_MISSION,  # FRAME
        MAV_CMD.MAV_CMD_DO_CHANGE_SPEED,
        1 if seq == 0 else 0,  # CMD  # CURRENT
        1,  # AUTOCONTINUE
        1,  # SPEED_TYPE
        speed,  # SPEED
        -1,  # THROTTLE
        0,  # PARAM4
        0,  # X
        0,  # Y
        0,  # Z
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,
    )


def create_set_focus_type(seq: int, focus_type: SET_FOCUS_TYPE) -> MissionItem:
    return MissionItem(
        seq,  # SEQ
        MAV_FRAME.MAV_FRAME_MISSION,  # FRAME
        MAV_CMD.MAV_CMD_SET_CAMERA_FOCUS,
        1 if seq == 0 else 0,  # CMD  # CURRENT
        1,  # AUTOCONTINUE
        focus_type.value,  # FOCUS_TYPE
        0,  # PARAM2
        0,  # PARAM3
        0,  # PARAM4
        0,  # X
        0,  # Y
        0,  # Z
        MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION,
    )
