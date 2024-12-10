from dataclasses import dataclass, field
import numpy as np
from sklearn.linear_model import LinearRegression

@dataclass
class UAV:
    # Type 1: Small drone
    ROTOR_SWEEP: float = 0.8       # seep area per rotor
    ROTOR_NUM: int = 4
    ROTOR_AREA: float = ROTOR_NUM * ROTOR_SWEEP

    WEIGHT: float = 5
    WEIGHT_CAPACITY: float = 5

    SPEED_VERTICAL = 20             # km/h
    SPEED_HORIZONTAL = 50           # km/h

    # Engergy related
    S_MIN = 0.1                     # min state of charge
    S_MAX = 0.9                     # max state of charge
    BATTERY_INITIAL = 60            # watt
    BATTERY_CAPACITY = 100          # watt
    BATTERY_SLOPE: float = 43.6838          # watt/kg
    BATTERY_INTERCEPT: float = 112.9572     # watt

    # Charging related
    CHARGING_POWER_RATED = 80       # watt/hr (charging efficiency * charging power)

    # Type 2: Large Drone
    # ROTOR_SWEEP = 1.5
    # ROTOR_NUM = 8
    # ROTOR_AREA = ROTOR_NUM * ROTOR_SWEEP
    #
    # WEIGHT = 20
    # CAPACITY = 35

    # # New attributes that will be calculated in __post_init__, i/o property
    # BATTERY_SLOPE: float = field(init=False)
    # BATTERY_INTERCEPT: float = field(init=False)
    # ROTOR_AREA: float = field(init=False)
    #
    # def __post_init__(self):
    #     # Calculate ROTOR_AREA
    #     self.ROTOR_AREA = self.ROTOR_NUM * self.ROTOR_SWEEP
    #
    #     # Generate data points for linear regression
    #     m = np.linspace(0, self.WEIGHT_CAPACITY, 100)
    #     constant = np.sqrt((Physics.GRAVITY ** 3) / (2 * Physics.AIR_DENSITY * self.ROTOR_AREA))
    #     y = ((self.WEIGHT + m) ** 1.5) * constant
    #
    #     X = m.reshape(-1, 1)
    #     y = y.reshape(-1, 1)
    #
    #     # Fit linear regression model
    #     model = LinearRegression()
    #     model.fit(X, y)
    #
    #     # Store results
    #     self.BATTERY_SLOPE = model.coef_[0][0]
    #     self.BATTERY_INTERCEPT = model.intercept_[0]

@dataclass
class Physics:
    # Drone energy consumption
    GRAVITY = 9.8
    AIR_DENSITY = 1.293

    # Distance calculation
    EARTH_RADIUS = 6371.9               # km on latitude 32.779167 (Dallas, TX)

class Environment:
    GRID_SIZE = [(-96.95, 32.6), (-96.6, 33)]    # km -> DONE: convert to Dallas lat, long

    CELL_SIDE = 0.02                    # DONE: approximate shift in lat and long by 2 km eastward or northward
    HEIGHT = 0.3048                     # km (1000 ft) minimum UAV flight height in Class G (uncontrolled) airspace

    # GRID_SIZE = [(0, 0), (5, 5)]        # km
    # CELL_SIDE = 0.174                   # km (hexagon side length)

    # Time related parameters
    RESPONSE_WINDOW = 0.5               # hr, 30 minutes time window for delivery
    TIME_WINDOW = (0, 8)                # hr operation time window
    TIME_INTERVAL = 1                   # time interval of demand occurence
    TIME_DEMAND_HANDLING = 0.01

    LOAD_AVG = 0.5                      # kg
    LOAD_STD = 0.1                      # kg standard deviation of load weight
    DEMAND_RATE_RANGE = (0.1, 0.2)      # times/hour demand rate in one cell

    CAND_CS_NUM = 20                    # number of candidate charging station
    WAREHOUSE_NUM = 4                   # number of warehouse


    RANDOM_SEED = 6

class Model:
    MAX_CHARGING_FREQ = 4               # H
    PENALTY_COEF = 1e-6

    # Investment cost
    EXPANSION_COST = 1000               # Cost to expand warehouse to warehouse + charging station
    CONSTRUCTION_COST = 2000            # Cost to construct charging station
    UAV_MARGINAL_COST = 300             # Variable cost to have one extra drone

    # Operation cost
    TIME_UNIT_COST = 10                 # $10 Unit operation cost per unit time


    BIG_M_TIME = 40                     # 5-times max time
    BIG_M_LOAD = 10                     # 2-times max load
    BIG_M_ENERGY = 200                  # 2-times max watt

class Algorithm:
    MAX_ITERATION = 300
    MIN_CONVERGE = 10

class Path:
    HOME = '/Users/sherrychen/PycharmProject/IOE 618/uav_charging_station'


