from __future__ import annotations

from re import match
from typing import TYPE_CHECKING

import numpy as np

from pyproj import Transformer
from pyproj.enums import TransformDirection

if TYPE_CHECKING:
    from numpy.typing import NDArray


def translate_latlong(
    coordinate: NDArray[np.float64], north: float = 0.0, east: float = 0.0, utm_zone: str = "epsg:32631"
) -> NDArray[np.float64]:
    transformer = Transformer.from_crs("epsg:4326", utm_zone, always_xy=True)

    utm_x, utm_y = transformer.transform(coordinate[0], coordinate[1])
    utm_x += north
    utm_y += east

    gps = transformer.transform(utm_x, utm_y, direction=TransformDirection.INVERSE)
    return np.array(gps, dtype=np.float64)


def valid_mavlink_url(connection_string: str | None) -> bool:
    # Special case, default connection using mavlink-router
    if connection_string is None:
        return True

    protocol_pattern = r"^(tcp://|udp://|serial://)"
    return bool(match(protocol_pattern, connection_string))
