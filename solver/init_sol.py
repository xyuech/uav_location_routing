"""
Author: Xiyue Chen
Date:   2024-12-05
Usage:  Generate initial feasible solution for the VNS-MIP algorithm
"""

from preprocess import assign_demand as ad
from sampling.simulation import DeliveryScenario

def generate_initial_solution_greedy(demand_scenarios: DeliveryScenario,
                                     parameters: ):
    ds = demand_scenarios

    # Generate initial solution for charging station
    charging_stations = ds.cs_locations
    working_charging_station = [False for _ in range(ds.cs_num)]
    max_uav_num = [0 for _ in range(ds.cs_num)]

    scenario_set = ds.simulate_scenarios()
    for i in range(len(scenario_set)):
        demands = scenario_set[i]
        demand_addresses = [dm["location"] for dm in demands]

        assignment_data = ad.assign_address(demand_addresses, charging_stations)
        ad.plot_assignments(assignment_data, demand_addresses, charging_stations)
        working_charging_station = [working_charging_station[cs_ind] or
                                    bool(assignment_data.station_to_demands[charging_stations[cs_ind]])
                                    for cs_ind in range(ds.cs_num)]
        print(f'scenario {i}: flag of working charging station: {working_charging_station}')

        for cs_ind in range(ds.cs_num):
            demand_per_cs_scenario = assignment_data.station_to_demands[charging_stations[cs_ind]]
            if demand_per_cs_scenario:
                k = # TODO: calculate upper bound of UAV

    cs_solution = [cs_ind if working_charging_station[cs_ind] else None for cs_ind in range(ds.cs_num)]

