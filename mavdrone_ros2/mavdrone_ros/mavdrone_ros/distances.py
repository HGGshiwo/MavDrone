from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

EARTH_RADIUS_M = 6371000.0
DEG_TO_RAD = np.pi / 180.0


def meters_per_deg_lat(latitude_deg: float) -> float:
    lat_rad = latitude_deg * DEG_TO_RAD
    return 111132.954 - 559.822 * np.cos(2 * lat_rad) + 1.175 * np.cos(4 * lat_rad)


def meters_per_deg_lon(latitude_deg: float) -> float:
    lat_rad = latitude_deg * DEG_TO_RAD
    return 111412.84 * np.cos(lat_rad) - 93.5 * np.cos(3 * lat_rad) + 0.118 * np.cos(5 * lat_rad)


def _haversine_surface_distance(lat1_rad: float, lon1_rad: float, lat2_rad: float, lon2_rad: float) -> float:
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def distance_3d_approx(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
    lat1, lon1, alt1 = p1
    lat2, lon2, alt2 = p2

    m_per_deg_lat = meters_per_deg_lat(lat1)
    m_per_deg_lon = meters_per_deg_lon(lat1)

    dlat_m = (lat2 - lat1) * m_per_deg_lat
    dlon_m = (lon2 - lon1) * m_per_deg_lon
    dalt = alt2 - alt1

    return np.sqrt(dlat_m**2 + dlon_m**2 + dalt**2)


def distance_2d_approx(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
    lat1, lon1 = p1[:2]
    lat2, lon2 = p2[:2]

    m_per_deg_lat = meters_per_deg_lat(lat1)
    m_per_deg_lon = meters_per_deg_lon(lat1)

    dlat_m = (lat2 - lat1) * m_per_deg_lat
    dlon_m = (lon2 - lon1) * m_per_deg_lon

    return np.sqrt(dlat_m**2 + dlon_m**2)


def distance_3d_exact(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
    lat1_rad, lon1_rad, _ = p1 * DEG_TO_RAD
    lat2_rad, lon2_rad, _ = p2 * DEG_TO_RAD

    surface_dist = _haversine_surface_distance(lat1_rad, lon1_rad, lat2_rad, lon2_rad)
    dalt = p2[2] - p1[2]

    return np.sqrt(surface_dist**2 + dalt**2)


def distance_2d_exact(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
    lat1_rad, lon1_rad = p1[:2] * DEG_TO_RAD
    lat2_rad, lon2_rad = p2[:2] * DEG_TO_RAD

    return _haversine_surface_distance(lat1_rad, lon1_rad, lat2_rad, lon2_rad)


# Example usage
if __name__ == "__main__":
    p1 = np.array([52.2296756, 21.0122287, 100.0])
    p2 = np.array([52.2297756, 21.0123287, 110.0])

    dist_approx = distance_3d_approx(p1, p1)
    dist_exact = distance_3d_exact(p1, p2)

    print(f"Approximate distance: {dist_approx:.3f} m")
    print(f"Exact distance: {dist_exact:.3f} m")
