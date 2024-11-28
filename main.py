"""
Author: Xiyue Chen
Date:   2024-11-27
Usage:  Execute the main program of the UAV charging station location selection
"""
from sampling import simulation as sim
from preprocess import get_parameter
def main():
    # Input environment data for grid locations, warehouses, charging station candidates
    # Random generation for testing create data_instance
    ds = sim.DeliveryScenario()


    d_grid_grid, d_grid_wh, d_cs_grid, d_cs_wh, d_cs_cs = get_parameter.get_travel_time(ds.grid_locations,
                                                                                        ds.wh_locations,
                                                                                        ds.cs_locations)
    print(d_grid_grid)
    # Pre-process: calculate distances

    # Initial feasible solution generation

    # Variable neighborhood search
    return

if __name__ == "__main__":
    main()