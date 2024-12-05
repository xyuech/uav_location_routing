from dataclasses import dataclass
import numpy as np
from sklearn.linear_model import LinearRegression

@dataclass
class UAV:
    # Type 1: Small drone
    ROTOR_SWEEP: float = 0.8       # seep area per rotor
    ROTOR_NUM: int = 4

    WEIGHT: float = 5
    WEIGHT_CAPACITY: float = 5

    SPEED_VERTICAL = 10          # TODO: verify the speed setting
    SPEED_HORIZONTAL = 20

    # Engergy related
    BATTERY_CAPACITY = 100      # watt

    # Type 2: Large Drone
    # ROTOR_SWEEP = 1.5
    # ROTOR_NUM = 8
    # ROTOR_AREA = ROTOR_NUM * ROTOR_SWEEP
    #
    # WEIGHT = 20
    # CAPACITY = 35
    @property
    def ROTOR_AREA(self):
        return self.ROTOR_NUM * self.ROTOR_SWEEP
    @property
    def BATTERY_APPROX_PARAM(self):
        """Get linear approximation parameters for uav energy consumption per unit time"""

        # Generate data points
        m = np.linspace(0, self.WEIGHT_CAPACITY, 100)  # generate 100 points from 0 to 5
        constant = np.sqrt((Physics.GRAVITY ** 3) / (2 * Physics.AIR_DENSITY * self.ROTOR_AREA))
        y = ((self.WEIGHT + m) ** 1.5) * constant

        X = m.reshape(-1, 1)
        y = y.reshape(-1, 1)

        model = LinearRegression()
        model.fit(X, y)

        # Print regression results
        slope = model.coef_[0][0]
        intercept = model.intercept_[0]
        return slope, intercept

@dataclass
class Physics:
    # Drone energy consumption
    GRAVITY = 9.8
    AIR_DENSITY = 1.293

    # Distance calculation
    EARTH_RADIUS = 6371.9               # km on latitude 32.779167 (Dallas, TX)

class Environment:
    GRID_SIZE = [(-96.95, 32.6), (-96.6, 33)]    # km -> DONE: convert to Dallas lat, long

    CELL_SIDE = 0.015                    # DONE: approximate shift in lat and long by 1.5 km eastward or northward
    HEIGHT = 0.3048                     # km (1000 ft) minimum UAV flight height in Class G (uncontrolled) airspace

    # GRID_SIZE = [(0, 0), (5, 5)]        # km
    # CELL_SIDE = 0.174                   # km (hexagon side length)

    # Time related parameters
    RESPONSE_WINDOW = 15                # minute, time window for delivery
    TIME_WINDOW = (0, 8)                # minute operation time window
    TIME_INTERVAL = 1                   # time interval of demand occurence

    LOAD_AVG = 0.5                      # kg
    LOAD_STD = 0.1                      # kg standard deviation of load weight
    DEMAND_RATE_RANGE = (0, 0.1)      # times/hour demand rate in one cell

    CAND_CS_NUM = 20                    # number of candidate charging station
    WAREHOUSE_NUM = 4                   # number of warehouse

    RANDOM_SEED = 6

class Model:
    MAX_CHARGING_FREQ = 4               # H
    PENALTY_COEF = 1e-6

    OPERATION_UNIT_COST = 10            # $10 dollar

    # Investment cost
    EXPANSION_COST = 1000               # Cost to expand warehouse to warehouse + charging station
    CONSTRUCTION_COST = 2000            # Cost to construct charging station
    UAV_MARGINAL_COST = 300             # Variable cost to have one extra drone

    # Operation cost
    TIME_UNIT_COST = 0.1                # Unit operation cost per unit time



class Algorithm:
    MAX_ITERATION = 300
    MIN_CONVERGE = 10



