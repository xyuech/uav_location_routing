class Drone:
    # Type 1: Small drone
    ROTOR_SWEEP = 0.8       # seep area per rotor
    ROTOR_NUM = 4
    ROTOR_AREA = ROTOR_NUM * ROTOR_SWEEP

    WEIGHT = 5
    CAPACITY = 5

    UNIT_SPPED = 5          # TO DO: verify the speed setting

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
    TIME_WINDOW = (0, 480)              # minute operation time window
    TIME_INTERVAL = 60                  # time interval of demand occurence

    LOAD_AVG = 0.5                      # kg
    LOAD_STD = 0.1                      # kg standard deviation of load weight
    DEMAND_RATE_RANGE = (0.1, 0.3)      # times/hour demand rate in one cell

    CAND_CS_NUM = 20                    # number of candidate charging station
    WAREHOUSE_NUM = 4                   # number of warehouse

    RANDOM_SEED = 6





# class Parameter:



