'''
Author: Xiyue Chen
Date:   2024-11-19
Usage:  Utility functions used across the whole project.
'''
from typing import Tuple

from math import acos, cos, sin
import numpy as np

from input.config import Physics as phys


def calculate_distance_on_earth(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    lat1, long1 = coord1
    lat2, long2 = coord2
    return phys.EARTH_RADIUS * acos(cos(lat1) * cos(lat2) * cos(long1 - long2) + sin(lat1) * sin(lat2))
def calculate_euclidean_distance(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    return np.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

if __name__ == "__main__":
    dist = calculate_distance_on_earth((1, 1), (1, 1.05))
    print(dist)