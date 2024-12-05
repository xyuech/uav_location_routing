"""
Author: Xiyue Chen
Date:   2024-11-24
Usage:  Calculate parameters prior to modeling.
        Generate dictionary for traveling time between locations
"""
from typing import List, Tuple, Dict
import numpy as np
from dataclasses import dataclass

# import from other codes
from input.config import Environment, UAV
from util.util import calculate_distance_on_earth, calculate_euclidean_distance
from sampling.simulation import DeliveryScenario

@dataclass
class T

def get_travel_time(demand_scenarios: DeliveryScenario,
                    env: Environment,
                    uav: UAV,
                    use_earth_distance:bool = False) -> Dict[Tuple[Tuple[float, float], Tuple[float, float]], float]:
    """
    Calculate distances between all possible locations
    Returns:
        Dict[(location1, location2): distance]
    """
    distances = {}
    calc_func = calculate_distance_on_earth if use_earth_distance else calculate_euclidean_distance

    dist_grid_grid = {}
    dist_grid_wh= {}
    dist_cs_grid = {}
    dist_cs_wh = {}
    dist_cs_cs = {}

    # TODO: change to earth surface distance
    # TODO: calculate travel time
    for n1 in range(len(cs_locations)):
        for n2 in range(n1+1, len(cs_locations)):
            dist_cs_cs[(f"cs_{n1}", f"cs_{n2}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                                cs_locations[n2])
        for i in range(len(grid_locations)):
            dist_cs_grid[(f"cs_{n1}", f"gd_{i}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                                 grid_locations[i])

        for j in range(len(wh_locations)):
            dist_cs_wh[(f"cs_{n1}", f"wh_{j}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                               wh_locations[j])

    for ind1 in range(len(grid_locations)):
        for ind2 in range(ind1+1, len(grid_locations)):
            dist_grid_grid[(f"gd_{ind1}", f"gd_{ind2}")] = calculate_euclidean_distance(grid_locations[ind1],
                                                                                        grid_locations[ind2])

        for j in range(len(wh_locations)):
            dist_grid_wh[(f"gd_{ind1}", f"wh_{j}")] = calculate_euclidean_distance(grid_locations[ind1],
                                                                                   wh_locations[j])
    # merge all distances
    distances = dist_grid_grid | dist_grid_wh | dist_cs_grid | dist_cs_wh | dist_cs_cs
    return distances

@dataclass
class Distances:


