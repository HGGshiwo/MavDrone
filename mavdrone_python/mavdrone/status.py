from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from mavsdk.camera import VideoStreamInfo
    from mavsdk.mission_raw import MissionItem
    from mavsdk.telemetry import LandedState


@dataclass
class DroneStatus:
    connected: bool = field(default=False)
    gps_fixed: bool = field(default=False)
    armed: bool = field(default=False)

    position: NDArray[np.float64] = field(default_factory=lambda: np.array([np.nan, np.nan, np.nan], dtype=np.float64))
    attitude: NDArray[np.float64] = field(default_factory=lambda: np.array([np.nan, np.nan, np.nan], dtype=np.float64))
    velocity: NDArray[np.float64] = field(default_factory=lambda: np.array([np.nan, np.nan, np.nan], dtype=np.float64))
    gimbal_angle: NDArray[np.float64] = field(default_factory=lambda: np.array([np.nan, np.nan, np.nan], dtype=np.float64))

    land_state: LandedState | None = field(default=None)

    mission_items: list[MissionItem] = field(default_factory=list)
    geofence_items: list[MissionItem] = field(default_factory=list)

    video_stream_info: VideoStreamInfo | None = field(default=None)
