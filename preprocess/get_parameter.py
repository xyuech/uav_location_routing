"""
Author: Xiyue Chen
Date:   2024-11-24
Usage:  Calculate parameters prior to modeling.
        Generate dictionary for traveling time between locations, and energy consumption rate
"""
from typing import List, Tuple, Dict
from dataclasses import dataclass

# import from other codes
from input.config import Environment, UAV
from util.util import calculate_distance_on_earth, calculate_euclidean_distance
from sampling.simulation import DeliveryScenario

@dataclass
class Parameter:
    travel_times: Dict[Tuple[Tuple[float, float], Tuple[float, float]], float]

def get_parameter(ds: DeliveryScenario,
                    env: Environment,
                    uav: UAV,
                    use_earth_distance:bool = True) \
        -> Parameter:
    """
    Calculate distances between all possible locations
    Returns:
        Dict[(location1, location2): distance]
    """
    print("info: get_parameter start.")
    travel_times = {}
    calc_func = calculate_distance_on_earth if use_earth_distance else calculate_euclidean_distance
    all_locations = list(ds.all_locations)
    for loc in all_locations:
        dist_to_loc = calc_func(all_locations, loc)
        travel_times.update(
            [((all_locations[i], loc),
              dist_to_loc[i]/uav.SPEED_HORIZONTAL + 2 * env.HEIGHT / uav.SPEED_VERTICAL)
             for i in range(len(all_locations))]
        )

    return Parameter(travel_times=travel_times)


