"""
Author: Xiyue Chen
Date:   2024-12-05
Usage:  Assignment demand to charging stations, according to closet distances
"""
from typing import List, Tuple, Dict, Set
from dataclasses import dataclass
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt

from sampling.simulation import DeliveryScenario



# import from other codes
from sampling.simulation import DeliveryScenario
from util.util import calculate_distance_on_earth, calculate_euclidean_distance

@dataclass
class DemandAssignmentData:
    """Store the station and demand address mapping"""
    demand_to_station: Dict[Tuple[float, float], Tuple[float, float]]           # Dict[loc, loc]
    station_to_demands: Dict[Tuple[float, float], Set[Tuple[float, float]]]     # Dict[loc, set(loc)]
    vor: Voronoi
def assign_address(
        demand_addresses: List[Tuple[float, float]],
        charging_stations: List[Tuple[float, float]]
) -> DemandAssignmentData:
    """Assign demand address to charging stations (under a single scenario)"""
    print("INFO: assign_address start")
    stations_array = np.array(charging_stations)
    vor = Voronoi(stations_array)

    demand_to_station = {}
    station_to_demands = {station: set() for station in charging_stations}
    for addr in demand_addresses:
        distances = calculate_euclidean_distance(charging_stations, addr)

        closet_idx = np.argmin(distances)
        closet_station = charging_stations[closet_idx]
        demand_to_station[addr] = closet_station
        station_to_demands[closet_station].add(addr)
    return DemandAssignmentData(demand_to_station, station_to_demands, vor)

def plot_assignments(assignment_data: DemandAssignmentData,
                     demand_addresses: List,
                     charging_stations: List):
    plt.figure()
    voronoi_plot_2d(assignment_data.vor, show_vertices=False, line_colors='gray', alpha=0.6)

    stations_array = np.array(charging_stations)
    demands_array = np.array(demand_addresses)

    plt.scatter(stations_array[:, 0], stations_array[:, 1], c="blue", marker="^", label='Charging Stations')
    plt.scatter(demands_array[:, 0], demands_array[:, 1], c="lightgray", marker='.', label="Demand Addresses")

    plt.legend()
    plt.title('Demand Assignment to Charging Stations')
    # plt.axis('equal')
    plt.show()



