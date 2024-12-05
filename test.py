from util.util import calculate_distance_on_earth, calculate_euclidean_distance
from sampling.simulation import DeliveryScenario
import preprocess.assign_demand as assign_demand
if __name__ == "__main__":
    # Distance test
    # dist = calculate_euclidean_distance([(1, 1), (2,1), (3,1)], (1, 1.05))
    # print(dist.shape)
    # print(dist)

    # Test demand assignment
    ds = DeliveryScenario()
    demand_addresses = ds.grid_locations
    demand_indices = list(range(ds.grid_num))

    charging_stations = ds.cs_locations
    assignment_data = assign_demand.assign_address(demand_addresses, charging_stations)
    assign_demand.plot_assignments(assignment_data, demand_addresses, charging_stations)
