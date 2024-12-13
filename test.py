from util.logger import get_logger, get_solver_logger
import input.config as config
from input.read_data import read_locations
from sampling.simulation import DeliveryScenario
import preprocess.get_parameter as get_parameter
# tsp
import solver.init_sol_new as init_sol_new
from solver.tsp_solver import TSPSolver

# vrp
import solver.init_sol as init_sol
from solver.vrp_solver import VRPSolver

if __name__ == "__main__":
    # Distance test
    # dist = calculate_euclidean_distance([(1, 1), (2,1), (3,1)], (1, 1.05))
    # print(dist.shape)
    # print(dist)
    main_logger = get_logger('main')

    # Test demand assignment under different scenarios
    main_logger.info("Test demand assignment under different scenarios.")
    wh_locations = read_locations('wh_locations.csv')
    cs_locations = read_locations('cs_locations.csv')
    ds = DeliveryScenario(cs_locations=cs_locations,wh_locations=wh_locations)
    for i in range(len(ds.clustered_scenarios)):
        ds.visualize_scenarios(ds.clustered_scenarios[i], scenario_id=i+1)
    charging_stations = ds.cs_locations
    uav = config.UAV()
    env = config.Environment()
    # Test get_parameter unit
    main_logger.info("Test get_parameter unit.")
    param = get_parameter.get_parameter(ds=ds, env=env, uav=uav)

    # for loc_pair, dist in param.travel_times.items():
    #     print(f"DEBUG: travel time between {loc_pair} is {dist:.2f} hour.")

    # Generate intial solution with new
    main_logger.info(f'Test for initial solution generation algorithm (new).')
    uav = config.UAV()
    model = config.Model()      # need dataclass instance
    investment_decision_new = init_sol_new.generate_initial_solution_greedy(demand_scenarios=ds, parameters=param,
                                                                        uav=uav, mod=model)

    # demand_assignment_all, investment_decision = \
    #     init_sol.generate_initial_solution_greedy(demand_scenarios=ds, parameters=param, uav=uav, mod=model)
    # # main_logger.info(demand_assignment_all)
    # main_logger.info(f'investment_decision: {investment_decision}')

    # Solve for iteration 0 for TSP subproblems
    main_logger.info(f'Test for subproblem TSP solver.')
    sol_logger = get_solver_logger('tsp_solver', iter_count=0)
    tsp = TSPSolver(config=config,
                    demand_scenarios=ds,
                    investment_variable=investment_decision_new,
                    parameters=param,
                    sol_logger=sol_logger,
                    iter_count=0)
    tsp.solve_model_per_cs_per_sim(scenario_id=0)




    # Solve for VRP subproblems
    # main_logger.info(f'Test for subproblem VRP solver.')
    # sol_logger = get_solver_logger(iter_count=0)
    # vrp = VRPSolver(config=config,
    #                 demand_scenarios=ds,
    #                 demand_assignment_data=demand_assignment_all,
    #                 investment_variable=investment_decision,
    #                 parameters=param,
    #                 sol_logger=sol_logger,
    #                 iter_count=0)
    # vrp.solve_model_per_cs_per_sim(cs_id=0, scenario_id=0)

    # # Solve for 2nd_stage
    # for iter_count in range(1):
    #     # TODO: Variable Neighborhood Search
    #     main_logger.info(f'Iter {iter_count}: Solve second stage problem')
    #     sol_logger = get_solver_logger(iter_count=iter_count)
    #     vrp = VRPSolver(config=config,
    #                     demand_scenarios=ds,
    #                     demand_assignment_data=demand_assignment_all,
    #                     investment_variable=investment_decision,
    #                     parameters=param,
    #                     sol_logger=sol_logger,
    #                     iter_count=iter_count)
    #
    #     for scenario_id in range(1):
    #         main_logger.info(f'Iter {iter_count}: Solve subproblem for scenario {scenario_id}.')
    #         for cs_id, cs_flag in enumerate(investment_decision.working_charging_station):
    #             if cs_flag:
    #                 uav_num = investment_decision.uav_num_assignment[cs_id]
    #                 main_logger.info(f'Iter {iter_count}: Solve subproblem for cs {cs_id} with {uav_num} uavs.')
    #                 vrp.solve_model_per_cs_per_sim(cs_id=cs_id, scenario_id=scenario_id)
    #
    #     # TODO: collect all solutions, calculte objective functions
    #     # TODO: Process control and iterate by VNS


