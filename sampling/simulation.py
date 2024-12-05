'''
Author: Xiyue Chen
Date:   2024-11-19
Usage:  Scenario generation by Monte Carlo simulation.
'''
import math

import pandas as pd
import numpy as np
from numpy import ndarray
from sklearn.cluster import KMeans, kmeans_plusplus
from typing import List, Tuple, Dict, Any
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon

# import from other codes
import os
import sys

# Get absolute path to parent directory
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from input.config import Environment as env
from util import util as ut
# from ..input.config import Environment as env
# from ..util import util as ut


# from input.config import Parameter as param

class DeliveryScenario:
    def __init__(self,
                 grid_range: List[Tuple[float, float]] = env.GRID_SIZE,
                 # coord of [left_down, right_up] points of the grid
                 cell_side=env.CELL_SIDE,
                 height=env.HEIGHT,
                 time_window=env.TIME_WINDOW,
                 time_interval=env.TIME_INTERVAL,
                 response_window: float = env.RESPONSE_WINDOW,
                 load_avg=env.LOAD_AVG,
                 load_std=env.LOAD_STD,
                 demand_rate_range: Tuple[float, float] = env.DEMAND_RATE_RANGE,
                 cs_num=env.CAND_CS_NUM,
                 wh_num=env.WAREHOUSE_NUM,
                 cs_locations=None,
                 wh_locations=None,
                 measure='km',
                 random_seed = env.RANDOM_SEED):
        self.grid_range = grid_range  # TODO: convert coordinates to km and reposition to 0, 0
        self.cell_side = cell_side
        self.grid_num = None
        self.grid_locations = self._generate_hex_grids()
        self.all_locations = set()

        self.time_window = time_window
        self.time_interval = time_interval
        self.period_num = int(time_window[1] / time_interval)
        self.response_window = response_window

        self.load_avg = load_avg
        self.load_std = load_std

        self.demand_rate_range = demand_rate_range

        self.height = height

        self.measure = measure

        if cs_locations is not None:
            self.cs_num = len(cs_locations)
            self.cs_locations = cs_locations
            self.all_locations.update(cs_locations)
        else:
            self.cs_num = cs_num
            self.cs_locations = self._generate_random_locations(cs_num)

        if wh_locations is not None:
            self.wh_num = len(wh_locations)
            self.wh_locations = wh_locations
            self.all_locations.update(wh_locations)
        else:
            self.wh_num = wh_num
            self.wh_locations = self._generate_random_locations(wh_num)

    def _generate_hex_grids(self) -> List[Tuple[float, float]]:
        '''Generate hexagonal cell center in side the grid range.'''
        h = self.cell_side * np.sqrt(3)
        w = self.cell_side * 2
        center = []
        cur_x, cur_y = self.grid_range[0][0] + self.cell_side, self.grid_range[0][1] + self.cell_side * (np.sqrt(3) / 2)
        while cur_x < self.grid_range[1][0]:
            while cur_y < self.grid_range[1][1]:
                center.append((cur_x, cur_y))
                cur_y += h
            cur_x += 1.5 * w
            cur_y = self.grid_range[0][1] + self.cell_side * (np.sqrt(3) / 2)

        cur_x, cur_y = self.grid_range[0][0] + self.cell_side * 2.5, self.grid_range[0][1] + self.cell_side * np.sqrt(3)
        while cur_x < self.grid_range[1][0]:
            while cur_y < self.grid_range[1][1]:
                center.append((cur_x, cur_y))
                cur_y += h
            cur_x += 1.5 * w
            cur_y = self.grid_range[0][1] + self.cell_side * np.sqrt(3)

        self.grid_num = len(center)

        return center

    def _generate_random_locations(self, n: int) -> List[Tuple[float, float]]:
        '''Generate candidate charging station location. Should be '''
        locations = []
        for _ in range(n):
            x = np.random.uniform(self.grid_range[0][0], self.grid_range[1][0])
            y = np.random.uniform(self.grid_range[0][1], self.grid_range[1][1])
            locations.append((x, y))
        self.all_locations.update(locations)
        return locations

    def _calculate_cs_distance(self, demand_location: Tuple[float, float]) -> Tuple[int, float]:
        '''Calculate the demand point distance to candidate cs, return closet cs point'''
        min_dist = ut.calculate_euclidean_distance(*self.grid_range)
        assigned_cs = -1

        for i in range(self.cs_num):
            dist = ut.calculate_euclidean_distance(self.cs_locations[i], demand_location)
            if dist < min_dist:
                min_dist = dist
                assigned_cs = i

        return assigned_cs, min_dist

    def _calculate_scenario_feature(self, demands: List[Dict]) -> list[float | int | ndarray | Any]:
        '''Calculate the feature for the scenario given all demands.'''
        demand_frequency = len(demands)

        times = [d['t_lb'] for d in demands]
        mem = {}
        distances = []
        for d in demands:
            loc = d['location']
            if loc in mem:
                distances.append(mem[loc])
            else:
                _, dist = self._calculate_cs_distance(loc)
                mem[loc] = dist
                distances.append(dist)

        volume_indicator = sum(distances) / (self.grid_num * self.period_num)

        return {
            'volume_indicator': volume_indicator,
            'demand_frequency': demand_frequency,
            'dist_avg': np.mean(distances),
            'dist_std': np.std(distances),
            'time_avg': np.mean(times),
            'time_std': np.std(times)

        }


    def generate_single_scenario(self) -> Tuple[List[Dict], ]:
        '''
        Generate single scenario (omega) with delivery demands (for all grid a and time epoch t)
        Returns:
            demands: List[Dict]
            feature: Dict
        '''
        demands = []
        demand_count = 0

        # for each demand cell
        for grid_id in range(self.grid_num):
            cell_center = self.grid_locations[grid_id]
            # generate demand rate for cell center for all period t
            cell_rate = np.random.uniform(*self.demand_rate_range, self.period_num)

            for t in range(self.period_num):
                # generate demand
                if np.random.uniform() < cell_rate[t]:
                    # # simulate demand occurence time
                    # time = t * self.time_interval + np.random.uniform(60)
                    time = t * self.time_interval  # the demand only occur 60 minutes after last interval
                    mass = np.random.normal(self.load_avg, self.load_std)

                    while mass > 5:
                        mass -= 5
                        demand = {
                            'id': demand_count,
                            'addr_id': grid_id,
                            'location': cell_center,
                            't_lb': time,
                            't_ub': time + self.response_window,
                            'mass': 5
                        }
                        demand_count += 1
                        demands.append(demand)

                    demand = {
                        'id': demand_count,
                        'addr_id': grid_id,
                        'location': cell_center,
                        't_lb': time,
                        't_ub': time + self.response_window,
                        'mass': mass
                    }
                    demand_count += 1
                    demands.append(demand)
                    # print(demand)
        # # Calculate features
        feature = self._calculate_scenario_feature(demands)

        return demands, feature

    def simulate_scenarios(self,
                           n_clusters: int = 4,
                           convergence_tol: float = 0.005,
                           max_samples: int = 1000) -> List[List]:
        """
        Simulate demand samples using Monte Carlo simulation,
        Generate representative scenarios by clustering all samples by k++ and add extreme cases
        Returns:
            scenario_set: List of scenarios which are List[Dict] and each dictionary contains the information for
            one demand (job).
        """
        # Initialize simulation
        volume_indicator_ls = []
        demand_ls = []
        feature_ls = []

        # single_scenario = self.generate_single_scenario()
        # volume_indicator_ls.append(single_scenario['feature']['volume_indicator'])
        #
        # demand_ls.append(single_scenario['demands'])
        # feature_ls.append(list(single_scenario['feature'].values()))

        # Simulation
        coeffcient_covariance = 1

        while coeffcient_covariance > convergence_tol and len(volume_indicator_ls) <= max_samples:
            single_scenario_demand, single_scenario_feature = self.generate_single_scenario()
            volume_indicator_ls.append(single_scenario_feature['volume_indicator'])

            demand_ls.append(single_scenario_demand)
            feature_ls.append(list(single_scenario_feature.values()))
            if len(volume_indicator_ls) < 2:
                continue
            coeffcient_covariance = np.std(volume_indicator_ls) / np.mean(volume_indicator_ls)

        print(f'stop generation at {len(volume_indicator_ls)} samples with cv {coeffcient_covariance}.')
        # Convert features to np.array
        # print(feature_ls)
        feature_array = np.array(feature_ls)


        # K-means: add cluster centroid single scenario to the Representative Scenario Set
        # kmeans_instance = KMeans(clusters=n_clusters, random_state=618)
        # labels = kmeans_instance.fit_predict(X=feature_array)
        centers, indices = kmeans_plusplus(X=feature_array, n_clusters=n_clusters, random_state=618)

        scenario_set = []
        for i in indices:
            scenario_set.append(demand_ls[i])
            self.all_locations.update(
                [d['location'] for d in demand_ls[i]]
            )
            # print(demand_ls[i])

        for j in range(5):
            top_index = feature_ls.index(
                max(feature_ls, key=lambda x: x[j])
            )
            if top_index not in indices:
                scenario_set.append(demand_ls[top_index])
                self.all_locations.update(
                   [d['location'] for d in demand_ls[top_index]]
                )
            # print(feature_ls[top_index])

        return scenario_set



    def visualize_single_scenarios(self, scenario: Dict=None):
        '''
        Visualize single scenario
        param: scenario (optional) represents a single scenario in the representative set
        '''
        plt.figure(figsize=(10, 10))

        # Remove extra space at the axes
        plt.margins(0)

        # Plot grid center and the grid areas
        for center in self.grid_locations:
            hex = RegularPolygon(center,
                                 numVertices=6,
                                 radius=self.cell_side,
                                 orientation=math.pi/6,
                                 facecolor='none',
                                 edgecolor='aquamarine')
            plt.gca().add_patch(hex)

        grid_x, grid_y = zip(*self.grid_locations)
        plt.scatter(grid_x, grid_y, c="lightgray", marker=".")

        # Plot candidate charging station
        cs_x, cs_y = zip(*self.cs_locations)
        plt.scatter(cs_x, cs_y, c="blue", marker="^", label="Candidate charging station")

        # Plot warehouse
        wh_x, wh_y = zip(*self.wh_locations)
        plt.scatter(wh_x, wh_y, c="black", marker="s", label="Warehouses")

        # Plot demand
        if scenario:
            # Get demand locations
            demand_locations = [d['location'] for d in scenario]

            # Count frequency of each location
            from collections import Counter
            location_counts = Counter(map(tuple, demand_locations))

            # Calculate color normalization based on counts
            max_count = max(location_counts.values())
            norm = plt.Normalize(vmin=1, vmax=max_count)

            # Create scalar mappable before plotting hexagons
            sm = plt.cm.ScalarMappable(cmap=plt.cm.Reds, norm=norm)
            sm.set_array([])  # This is the key fix

            # Plot hexagon for each unique demand location
            for location, count in location_counts.items():
                # Create hexagon with shade based on frequency
                hex = RegularPolygon(location,
                                     numVertices=6,
                                     radius=self.cell_side,  # Full grid cell size
                                     orientation=math.pi / 6,
                                     facecolor=plt.cm.Reds(norm(count)),  # Red shade based on frequency
                                     edgecolor='red',
                                     alpha=0.7,
                                     label="Demand" if location == list(location_counts.keys())[0] else "")
                plt.gca().add_patch(hex)

            # Add colorbar with explicit axes specification
            plt.colorbar(sm, ax=plt.gca(), label='Demand Frequency')

            plt.title(f"Delivery Scenario with {len(scenario)} Demand Occurrences")

        else:
            if self.measure == 'km':
                plt.title(f"Grid Plot for {self.cs_num} Candidate Charging Stations and {self.wh_num} Warehouses")

                plt.xlabel("X (km)")
                plt.ylabel("Y (km)")
            else:
                plt.title(f"Map for {self.cs_num} Candidate Charging Stations and {self.wh_num} Warehouses")

                plt.xlabel("Longitude")
                plt.ylabel("Latitude")
        plt.legend()
        plt.grid(visible="True", which="major")
        plt.show()
        return



if __name__ == "__main__":
    ds = DeliveryScenario()
    single_demand, single_feature = ds.generate_single_scenario()
    print("Single episode simulation results:")
    for k, v in single_feature.items():
        print(f'{k}: {v}')
    ds.visualize_single_scenarios()

    print("Monte Carlo simulation and scenario clustering results:")
    # Generate samples, reduced by k-means
    clustered_scenarios = ds.simulate_scenarios(n_clusters=4, convergence_tol=0.005, max_samples=1000)

    # ds.simulate_scenarios()
    ds.visualize_single_scenarios(clustered_scenarios[0])

    # ds = {1:2, 3:4}
    # print(ds.values())
    # print(type(ds.values()))
    # print(list(ds.values()))
    # print(type(list(ds.values())))
    #


