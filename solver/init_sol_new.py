"""
Author: Xiyue Chen
Date:   2024-12-10
Usage:  Generate initial feasible solution for the VNS-MIP algorithm with CS-UAV-DEMAND assignment mapping
"""
from typing import Tuple, Dict, List
from dataclasses import dataclass
from math import ceil
from collections import defaultdict
import input.config as config
from util.logger import get_logger
from sampling.simulation import DeliveryScenario
from preprocess.get_parameter import Parameter
from preprocess.assign_demand import DemandAssignmentData, assign_address

@dataclass
class InvestmentVariable:
    working_charging_station: List[bool]        # [construction decision for cs_id in range(cs_num)] for all scenarios
    uav_num_lb: List[int]                       # [lower bound of UAV num for cs_id in range(cs_num)] for all scenarios
    cs_uav_deployment: Dict[int, List[str]]           # Mapping: cs_id -> uav_id across scenarios
    uav_demand_assignment: List[Dict[str, List[int]]] # Mapping: uav_id -> demand_id for all scenarios i

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
                                     uav: config.UAV, mod: config.Model) -> InvestmentVariable:
    """
    Generate demand assignment across scenarios by the greedy adaptive search procedure (GRASP), the results are:
    Returns:
        working_charging_station: List[bool] stores binary decision to setup CS
        uav_num_lb: List[int] stores lower bound of UAV num for cs_id in range(cs_num)] for all scenarios
        cs_uav_deployment: Dict[int, str] dictionary for mapping: cs_id -> uav_id across scenarios
        uav_demand_assignment List[Dict[str, int]] dictionary for mapping: uav_id -> demand_id for all scenarios i
    """
    logger = get_logger('init_sol')
    logger.info("Start greedy adaptive search procedure to generate initial solution.\n")
    # Get charging stations with demand assignment for all scenarios
    charging_stations = demand_scenarios.cs_locations

    working_charging_station = [False for _ in charging_stations]       # initialize flag
    uav_num_lb = [0 for _ in charging_stations]                         # initialize lower bound count
    cs_uav_deployment = {cs_id: [] for cs_id in range(demand_scenarios.cs_num)}
                                                                        # initialize empty uav deployment mapping
    uav_demand_assignment_i = defaultdict(list)                         # initialize empty dict for assignment i
    uav_demand_assignment_all = []                                      # initialize empty dict for all assignments

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
        logger.debug(f'Scenario {i} update flag of working charging station: {working_charging_station}')

        # 1.3 update lower bound number of uav deployed at charging station
        trip_time = sum_trip_time(charging_stations, parameters, demands_i, assignment_data_i)
        for n in range(demand_scenarios.cs_num):
            cs = charging_stations[n]
            demand_per_cs_i = assignment_data_i.station_to_demands[cs]
            if demand_per_cs_i:
                k1 = ceil(len(demand_per_cs_i) / (mod.MAX_CHARGING_FREQ - 2))
                k2 = ceil(uav.BATTERY_CAPACITY / (uav.BATTERY_SLOPE * uav.WEIGHT_CAPACITY +
                                             uav.BATTERY_INTERCEPT) * trip_time[cs])
                uav_num_lb[n] = max(uav_num_lb[n], k1, k2)
        logger.debug(f'Scenario {i} update lower bound of UAV num: {uav_num_lb}')

        # 1.5 update the number of UAV deployed to CS
        for n in range(demand_scenarios.cs_num):
            D_n_curr_loc = assignment_data_i.station_to_demands[charging_stations[n]]
            D_n_curr_id = [dm['id'] for dm in demands_i if dm['location'] in D_n_curr_loc]
            K_curr = len(D_n_curr_id)                   # current number of demand, each required one UAV
            K_prev = len(cs_uav_deployment[n])          # existing UAV at CS n
            if K_prev >= K_curr:
                for k in range(K_curr):
                    uav_name = f'uav_{n}_{k}'
                    uav_demand_assignment_i[uav_name].append(D_n_curr_id[k])
            else:
                for k in range(K_prev):
                    uav_name = f'uav_{n}_{k}'
                    uav_demand_assignment_i[uav_name].append(D_n_curr_id[k])

                # add new UAVs to CS n
                for k in range(K_prev, K_curr):
                    uav_name = f'uav_{n}_{k}'
                    uav_demand_assignment_i[uav_name].append(D_n_curr_id[k])
                    cs_uav_deployment[n].append(uav_name)
        logger.debug(f'Scenario {i} update the number of UAV deployment at CS:')
        for n in range(demand_scenarios.cs_num):
            logger.debug(f'\tCS {n}: {cs_uav_deployment[n]}')
            logger.debug(f'\tUAV demand assignment: {[uav_demand_assignment_i[uav_name] for uav_name in cs_uav_deployment[n]]}\n')
        # cs_uav_deployment[n] are updated over scenarios, the final uav num should be the max number of demand_i

        uav_demand_assignment_all.append(uav_demand_assignment_i)
        uav_demand_assignment_i = defaultdict(list)     # initialize the demand assignment under i


    investment_decision = InvestmentVariable(working_charging_station, uav_num_lb, cs_uav_deployment,
                                             uav_demand_assignment_all)

    return investment_decision