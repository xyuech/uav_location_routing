"""
Author: Xiyue Chen
Date:   2024-12-05
Usage:  Generate initial feasible solution for the VNS-MIP algorithm
"""
from typing import Tuple, Dict, List
from dataclasses import dataclass
from math import ceil
import input.config as config
from sampling.simulation import DeliveryScenario
from preprocess.get_parameter import Parameter
from preprocess.assign_demand import DemandAssignmentData, assign_address

@dataclass
class InvestmentVariable:
    working_charging_station: Dict[Tuple[float, float], bool]   # {coordinate: construction decision}
    uav_num_assignment: Dict[Tuple[float, float], int]              # {coordinate: number of UAV assigned}
def sum_trip_time(charging_stations: List[Tuple[float, float]],
                  parameters: Parameter,
                  demands_i: List[Dict],
                  assignment_data_i: DemandAssignmentData):
    """
    Calculate the sum of trip time given demand assignment for all charging stations,
    Trips are triangle
    Arg:
        charging_stations: list contain charing station coordinates
        parameters: Parameter dataclass contain travel times between all possible points
        demands_i: scenario i
        assignment_data_i: assignment data under scenario i
    """
    trip_time = {cs: 0 for cs in charging_stations}
    for d in demands_i:
        d_coord = d['location']
        wh_coord = d['wh_location']
        cs_coord = assignment_data_i.demand_to_station[d_coord]

        trip_time[cs_coord] += (parameters.travel_times[(cs_coord, wh_coord)] +
                                parameters.travel_times[(wh_coord, d_coord)] +
                                parameters.travel_times[(d_coord, cs_coord)])
    return trip_time

def generate_initial_solution_greedy(demand_scenarios: DeliveryScenario,
                                     parameters: Parameter,
                                     uav: config.UAV, mod: config.Model,
                                     ) -> Tuple[DemandAssignmentData, InvestmentVariable]:
    """
    Generate demand assignment across scenarios, initial solution for investment variable
    """
    print("INFO: generate_initial_solution_greedy start")
    # Get charging stations with demand assignment for all scenarios
    charging_stations = demand_scenarios.cs_locations

    working_charging_station = [False for _ in charging_stations]        # initialize flag
    uav_num_assignment = [0 for _ in charging_stations]                  # initialize max count
    demand_to_station_all = {}                                           # initialize empty dictionary
    station_to_demands_all = {cs: set() for cs in charging_stations}
    # 1. iterate all scenarios
    scenario_set = demand_scenarios.clustered_scenarios
    for i in range(len(scenario_set)):
        demands_i = scenario_set[i]
        demand_addresses = [dm["location"] for dm in demands_i]

        # 1.1 get assignment data for scenario i
        assignment_data_i = assign_address(demand_addresses, charging_stations)
        # plot_assignments(assignment_data, demand_addresses, charging_stations)

        # 1.2 update flag for charging station
        working_charging_station = [working_charging_station[cs_ind] or
                                    bool(assignment_data_i.station_to_demands[charging_stations[cs_ind]])
                                    for cs_ind in range(demand_scenarios.cs_num)]
        print(f'DEBUG: scenario {i} update flag of working charging station: {working_charging_station}')

        # 1.3 update number of uav deployed at charging station
        trip_time = sum_trip_time(charging_stations, parameters, demands_i, assignment_data_i)
        for i in range(len(charging_stations)):
            cs = charging_stations[i]
            demand_per_cs_i = assignment_data_i.station_to_demands[cs]
            if demand_per_cs_i:
                k1 = ceil(len(demand_per_cs_i) / (mod.MAX_CHARGING_FREQ - 2))
                k2 = ceil(uav.BATTERY_CAPACITY / (uav.BATTERY_SLOPE * uav.WEIGHT_CAPACITY +
                                             uav.BATTERY_INTERCEPT) * trip_time[cs])
                uav_num_assignment[i] = max(uav_num_assignment[i], k1, k2)
        print(f'DEBUG: scenario {i} update number of uav deployed: {uav_num_assignment}')

        # 1.4 update demand assignment for different demand occured
        demand_to_station_all |= assignment_data_i.demand_to_station
        station_to_demands_all = {cs: station_to_demands_all[cs] | assignment_data_i.station_to_demands[cs] for cs in charging_stations}

    investment_decision = InvestmentVariable(working_charging_station, uav_num_assignment)
    demand_assignment_all = DemandAssignmentData(demand_to_station_all, station_to_demands_all,
                                                 assignment_data_i.vor) # use the Voronoi of last scenario

    return demand_assignment_all, investment_decision

