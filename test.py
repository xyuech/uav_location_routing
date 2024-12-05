from util.util import calculate_distance_on_earth, calculate_euclidean_distance
from sampling.simulation import DeliveryScenario
import preprocess.assign_demand as assign_demand
if __name__ == "__main__":
    # Distance test
    # dist = calculate_euclidean_distance([(1, 1), (2,1), (3,1)], (1, 1.05))
    # print(dist.shape)
    # print(dist)

    # Test demand assignment under different scenarios
    ds = DeliveryScenario()
    charging_stations = ds.cs_locations

    # Generate initial solution for charging station
    working_charging_station = [False for _ in range(ds.cs_num)]
    max_uav_num = [0 for _ in range(ds.cs_num)]


    scenario_set = ds.simulate_scenarios()
    for i in range(len(scenario_set)):
        demands = scenario_set[i]
        demand_addresses = [dm["location"] for dm in demands]

        assignment_data = assign_demand.assign_address(demand_addresses, charging_stations)
        assign_demand.plot_assignments(assignment_data, demand_addresses, charging_stations)
        working_charging_station = [working_charging_station[cs_ind] or
                                    bool(assignment_data.station_to_demands[charging_stations[cs_ind]])
                                    for cs_ind in range(ds.cs_num)]
        print(f'scenario {i}: flag of working charging station: {working_charging_station}')

    cs_solution = [cs_ind if working_charging_station[cs_ind] else None for cs_ind in range(ds.cs_num)]
    print(cs_solution)