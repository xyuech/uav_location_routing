from input.config import UAV, Environment
from sampling.simulation import DeliveryScenario
import util.util as ut
import csv
if __name__ == "__main__":
    wh_locations = []
    with open('input/wh_locations.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            wh_locations.append(
                tuple([float(num) for num in row])
            )
        print(wh_locations)

    print(ut.calculate_distance_on_earth([wh_locations[0], wh_locations[1]], wh_locations[2]))

    ds = DeliveryScenario(wh_locations=wh_locations, measure='coordinate')
    single_demand, single_feature = ds.generate_single_scenario()
    print("Single episode simulation results:")
    for k, v in single_feature.items():
        print(f'{k}: {v}')
    ds.visualize_single_scenarios()


    #Generate samples, reduced by k-means
    print("Monte Carlo simulation and scenario clustering results:")
    clustered_scenarios = ds.simulate_scenarios(n_clusters=4, convergence_tol=0.005, max_samples=1000)

    # ds.simulate_scenarios()
    ds.visualize_single_scenarios(clustered_scenarios[0])
    #
