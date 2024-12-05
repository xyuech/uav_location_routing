class Drone:
    # Type 1: Small drone
    ROTOR_SWEEP = 0.8       # seep area per rotor
    ROTOR_NUM = 4
    ROTOR_AREA = ROTOR_NUM * ROTOR_SWEEP

    WEIGHT = 5
    CAPACITY = 5

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



class Physics:
    # Drone energy consumption
    GRAVITY = 9.8
    AIR_DENSITY = 1.293

    # Distance calculation
    EARTH_RADIUS = 6371.9  # km on latitude 32.779167 (Dallas, TX)



class Environment:
    GRID_SIZE = [(0, 0), (2, 2)]
    CELL_SIDE = 0.174                   # km (hexagon side length)

    # Time related parameters
    RESPONSE_WINDOW = 15                # minute, time window for delivery
    TIME_WINDOW = (0, 8)              # minute operation time window
    TIME_INTERVAL = 1                  # time interval of demand occurence

    LOAD_AVG = 0.5                      # kg
    LOAD_STD = 0.1                      # kg standard deviation of load weight
    DEMAND_RATE_RANGE = (0, 0.05)      # times/hour demand rate in one cell

    CAND_CS_NUM = 20                    # number of candidate charging station
    WAREHOUSE_NUM = 4                   # number of warehouse

    RANDOM_SEED = 6

class Model:
    MAX_CHARGING_FREQ = 4
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



