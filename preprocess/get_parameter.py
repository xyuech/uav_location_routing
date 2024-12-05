"""
Author: Xiyue Chen
Date:   2024-11-24
Usage:  Calculate parameters prior to modeling
"""
from typing import List, Tuple, Dict
import numpy as np
from sklearn.linear_model import LinearRegression

# import from other codes
from input.config import Physics, Drone
from util.util import calculate_distance_on_earth, calculate_euclidean_distance



def get_uav_energy_consumption():
    """Get linear approximation parameters for uav energy consumption per unit time"""
    # Define constants
    g = Physics.GRAVITY  # gravitational acceleration (m/s²)
    rho = Physics.AIR_DENSITY
    A = Drone.ROTOR_AREA  # arbitrary area value
    m0_ls = [5]

    def generate_data(m_values, m0):
        """Generate y values based on the given formula"""
        constant = np.sqrt((g ** 3) / (2 * rho * A))
        y_values = ((m0 + m_values) ** 1.5) * constant
        return y_values

    # Generate data points
    m = np.linspace(0, Drone.CAPACITY, 100)     # generate 100 points from 0 to 5
    y_true = []
    for m0 in m0_ls:                            # m0_ls only considers one initial weight
        y = generate_data(m, m0)
        y_true.append(y)

        # Reshape data for sklearn
        X = m.reshape(-1, 1)
        y = y.reshape(-1, 1)

        # Perform linear regression
        model = LinearRegression()
        model.fit(X, y)

        # Print regression results
        slope = model.coef_[0][0]
        intercept = model.intercept_[0]

    return slope, intercept

def get_travel_time(grid_locations: List[Tuple[float, float]],
                    wh_locations: List[Tuple[float, float]],
                    cs_locations: List[Tuple[float, float]]) -> Tuple[Dict, Dict, Dict, Dict, Dict]:
    """
    Calculate distances between all possible locations
    Returns:
        dist_grid_grid: Dict {(gd_index, gd_index): distance}
        dist_grid_wh: Dict {(wh_index, grid_index): distance}
        dist_cs_wh: Dict {(cs_index, wh_index): distance}
        dist_cs_grid: Dict {(cs_index, grid_index): distance}
        dist_cs_cs: Dict {(cs_index, cs_index): distance}
    """
    dist_grid_grid = {}
    dist_grid_wh= {}
    dist_cs_grid = {}
    dist_cs_wh = {}
    dist_cs_cs = {}

    # TODO: change to earth surface distance
    # TODO: calculate travel time
    for n1 in range(len(cs_locations)):
        for n2 in range(n1+1, len(cs_locations)):
            dist_cs_cs[(f"cs_{n1}", f"cs_{n2}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                                cs_locations[n2])
        for i in range(len(grid_locations)):
            dist_cs_grid[(f"cs_{n1}", f"gd_{i}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                                 grid_locations[i])

        for j in range(len(wh_locations)):
            dist_cs_wh[(f"cs_{n1}", f"wh_{j}")] = calculate_euclidean_distance(cs_locations[n1],
                                                                               wh_locations[j])

    for ind1 in range(len(grid_locations)):
        for ind2 in range(ind1+1, len(grid_locations)):
            dist_grid_grid[(f"gd_{ind1}", f"gd_{ind2}")] = calculate_euclidean_distance(grid_locations[ind1],
                                                                                        grid_locations[ind2])

        for j in range(len(wh_locations)):
            dist_grid_wh[(f"gd_{ind1}", f"wh_{j}")] = calculate_euclidean_distance(grid_locations[ind1],
                                                                                   wh_locations[j])
    # merge all distances
    distances = dist_grid_grid | dist_grid_wh | dist_cs_grid | dist_cs_wh | dist_cs_cs
    return distances




