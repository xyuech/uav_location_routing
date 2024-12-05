'''
Author: Xiyue Chen
Date:   2024-11-19
Usage:  Utility functions used across the whole project.
'''
from math import acos, cos, sin
import numpy as np

from typing import Tuple, Union, List
from input.config import Physics as phys


def calculate_distance_on_earth(
        coord1: Union[Tuple[float, float], List[Tuple[float, float]]],
        coord2: Tuple[float, float]
) -> Union[float, np.ndarray]:
    """
    Calculate great circle distance between points on Earth.
    Args:
        coord2: can only be single point
    """
    # Convert to numpy arrays
    if isinstance(coord1, tuple):
        coord1_arr = np.array([coord1])
    else:
        coord1_arr = np.array(coord1)

    if isinstance(coord2, tuple):
        coord2_arr = np.array([coord2])
    else:
        raise ValueError("Coordinate 2 should only represent 1 location.")

    lat1, long1 = coord1_arr[:, [0, 1]].T
    lat2, long2 = coord2_arr[:, [0, 1]].T

    distances = phys.EARTH_RADIUS * np.arccos(
        np.cos(lat1) * np.cos(lat2) * np.cos(long1 - long2) +
        np.sin(lat1) * np.sin(lat2)
    ).flatten()

    return float(distances[0]) if len(distances) == 1 else distances

def calculate_euclidean_distance(
        coord1: Union[Tuple[float, float], List[Tuple[float, float]]],
        coord2: Tuple[float, float]
) -> Union[float, np.ndarray]:
    """Calculate Euclidean distance between points."""
    if isinstance(coord1, tuple):
        coord1_arr = np.array([coord1])
    else:
        coord1_arr = np.array(coord1)

    if isinstance(coord2, tuple):
        coord2_arr = np.array([coord2])
    else:
        raise ValueError("Coordinate 2 should only represent 1 location.")

    distances = np.sqrt(
        (coord1_arr[:, 0:1] - coord2_arr[:, 0]) ** 2 +
        (coord1_arr[:, 1:2] - coord2_arr[:, 1]) ** 2
    ).flatten()
    # coord1_arr[:, 0:1] maintain shape (n,1);
    # coord1_arr[:, 0] will turn shape into (n,);

    return float(distances[0]) if len(distances) == 1 else distances