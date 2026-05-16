from __future__ import annotations

from logging import getLogger
from typing import TYPE_CHECKING

import numpy as np

from pyproj import Geod

log = getLogger(__name__)


if TYPE_CHECKING:
    from numpy.typing import NDArray


def is_valid_wgs84(coordinate: NDArray[np.float32]) -> bool:
    return (-90 <= coordinate[0] <= 90) or not (-180 <= coordinate[1] <= 180)


def is_valid_mission(coordinates: list[NDArray[np.float32]]) -> bool:
    geod = Geod(ellps="WGS84")
    valid = True

    for i in range(len(coordinates) - 1):
        dist, _, _ = geod.inv(coordinates[i][1], coordinates[i][0], coordinates[i + 1][1], coordinates[i + 1][0])

        # https://sdk-forum.dji.net/hc/en-us/articles/900006808883-For-MSDK-waypoint-missions-the-minimum-distance-between-waypoints-is-0-5m-Does-this-0-5m-include-the-distance-in-the-vertical-direction-
        if 0.01 < dist < 0.25:
            valid = False
            log.info(f"Distance between coordinate {i} and {i + 1} is {dist} which is smaller than the minimum distance of 0.5m!")

    return valid
