import input.config as config
from sampling.simulation import DeliveryScenario
from preprocess.assign_demand import assign_address, plot_assignments
import preprocess.get_parameter as get_parameter
import solver.init_sol as init_sol
if __name__ == "__main__":
    # Distance test
    # dist = calculate_euclidean_distance([(1, 1), (2,1), (3,1)], (1, 1.05))
    # print(dist.shape)
    # print(dist)

    # Test demand assignment under different scenarios
    print("Test demand assignment under different scenarios.")
    ds = DeliveryScenario()
    charging_stations = ds.cs_locations

    # Test get_parameter unit
    print("Test get_parameter unit.")
    param = get_parameter.get_parameter(ds=ds, env=config.Environment, uav=config.UAV)

    for loc_pair, dist in param.travel_times.items():
        print(f"travel time between {loc_pair} is {dist:.2f} hour.")

    # Generate intial solution
    print(f'Test for initial solution generation algorithm.')
    uav = config.UAV()
    model = config.Model()      # need dataclass instance
    demand_assignment_all, investment_decision = \
        init_sol.generate_initial_solution_greedy(demand_scenarios=ds, parameters=param,
                                                  uav=uav, mod=model)
    print(demand_assignment_all)
    print(investment_decision)
