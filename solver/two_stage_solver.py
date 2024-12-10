"""
Author: Xiyue Chen
Date:   2024-11-27
Usage:  Solve the 2-nd stage routing sub-problem under a realized scenario \omega.
        The investment variable, UAV assignment variable, customer-charging center coverage is pre-determined.
"""
from typing import List, Tuple, Dict, Set, Any
import re
import logging
from datetime import datetime
from gurobipy import GRB
import gurobipy as gp
from util.logger import get_solver_logger
import input.config as config
from sampling.simulation import DeliveryScenario
from preprocess.get_parameter import Parameter
from preprocess.assign_demand import DemandAssignmentData, assign_address
from solver.init_sol import InvestmentVariable

class VRPSolver:
    def __init__(self, config,
                 demand_scenarios: DeliveryScenario,
                 demand_assignment_data: List[DemandAssignmentData],
                 investment_variable: InvestmentVariable,
                 parameters: Parameter,
                 sol_logger: logging,
                 iter_count: int
                 ):
        # Pass problem setup
        self.config = config
        self.H = config.Model.MAX_CHARGING_FREQ
        self.cs_num = demand_scenarios.cs_num
        self.cs_locations = demand_scenarios.cs_locations
        self.wh_num = demand_scenarios.wh_num
        self.wh_locations = demand_scenarios.wh_locations
        self.clustered_scenarios = demand_scenarios.clustered_scenarios

        # Parameters
        self.travel_times: Dict = parameters.travel_times
        self.handling_time = config.Environment.TIME_DEMAND_HANDLING
        self.uav_energy_a = parameters.uav_energy_a
        self.uav_energy_b = parameters.uav_energy_b
        self.uav_energy_init = parameters.uav_energy_init
        self.uav_energy_cap = parameters.uav_energy_cap
        self.uav_weight_0 = parameters.uav_weight_0
        self.uav_weight_cap = parameters.uav_weight_cap
        self.M_time = config.Model.BIG_M_TIME
        self.M_load = config.Model.BIG_M_LOAD
        self.M_energy = config.Model.BIG_M_ENERGY

        # 1-stg decisions
        self.demand_assignment_data_all = demand_assignment_data
        self.investment_variable = investment_variable

        # logging
        self.sol_logger = sol_logger
        if isinstance(self.sol_logger.handlers[0], logging.FileHandler):
            self.log_file_path = self.sol_logger.handlers[0].baseFilename

        # iter_count
        self.iter_count = iter_count

    def _calc_energy_consumption(self, m):
        return self.uav_energy_a * m + self.uav_energy_b

    def solve_model_per_cs_per_sim(self, cs_id, scenario_id) -> Dict:
        """
        Solve the VRP sub-problem for charging station i under scenario j
        Returns:
            Dict of subproblem solution
        """
        self.sol_logger.info(f"Build subproblem for charging station {cs_id} under scenario {scenario_id}.")
        uav_num: int = self.investment_variable.uav_num_assignment[cs_id]
        cs_loc: Tuple[float, float] = self.cs_locations[cs_id]

        station_n_to_demands: Set[Tuple[float, float]] = self\
            .demand_assignment_data_all[scenario_id].station_to_demands[cs_loc]
        demand_to_station_n: Dict[Tuple[float, float], Tuple[float, float]] = self\
            .demand_assignment_data_all[scenario_id].demand_to_station

        # Get targeted demands
        demands_w: List[Dict] = self.clustered_scenarios[scenario_id]
        demands_filtered = [d for d in demands_w if d['location'] in station_n_to_demands]
        demands_filtered_id = [d['id'] for d in demands_filtered]
        demands_filtered_name = [f'd_{d_id}' for d_id in demands_filtered_id]
        demands_filtered_loc = [d['location'] for d in demands_filtered]

        locations_all = [cs_loc] + self.cs_locations * (self.H - 1) + self.wh_locations + demands_filtered_loc
        name_all = [f'cs_{cs_id}_0'] + [f'cs_{c_id}_{h}' for c_id in range(self.cs_num) for h in range(1, self.H)]\
                   + [f'wh_{wh_id}' for wh_id in range(self.wh_num)]\
                   + demands_filtered_name
        # pattern for name recognition
        pattern_wh = f'^wh_\\d+$'
        pattern_cs_0 = f'^cs_{cs_id}_0$'
        pattern_cs_h = [f'^cs_\\d+_{h}' for h in range(self.H)]
        pattern_d = f'^d_\\d+$'

        # Get parameters
        E1, E2, E3, E4, E5, E6, E7, E8 = self._create_edge_sets(cs_id, name_all,
                                                                pattern_wh, pattern_cs_0, pattern_cs_h, pattern_d)

        m = gp.Model(name=f"scenario {scenario_id}-cs {cs_id}")

        # Deployment variables
        z = {}
        for k in range(uav_num):
            z[k] = m.addVar(vtype=GRB.BINARY, name=f'uav_{k}')

        # Deployment constraint
        # 0 uav index dominance constraint
        self.sol_logger.debug("Adding UAV dominance constraints...")
        for k in range(uav_num - 1):  # For each UAV except the last one
            # If UAV k is not deployed (doesn't use any edge from initial CS), then k+1 cannot be deployed
            m.addConstr(z[k] >= z[k+1], name=f"dominance_uav_{k}")
            self.sol_logger.debug(f"Added dominance constraint between UAV {k} and {k + 1}")
        m.update()

        # Route variables
        x = {}                  # key: [k, node1, node2]

        # Create variables for each UAV and valid edge
        for k in range(uav_num):
            # E1: Initial CS to Warehouses
            for name1, name2 in E1:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E2: Warehouses to Intermediate Charging Stations
            for name1, name2 in E2:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E3: Warehouses to Demands
            for name1, name2 in E3:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E4: Between Different Demands
            for name1, name2 in E4:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E5: Demands to Intermediate Charging Stations
            for name1, name2 in E5:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}' )

            # E6: Intermediate Charging Stations to Demands
            for name1, name2 in E6:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E7: Demands to Final Charging Station
            for name1, name2 in E7:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')

            # E8: Initial CS to Final Charging Station
            for name1, name2 in E8:
                x[(k, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_uav_{k}_{name1}-{name2}')


        # Route constraints
        # 1-a: all demands must be visited at least once 
        for did in demands_filtered_id:
            demand_name = f'd_{did}'
            target_arc = set([node_pair for node_pair in (E3 | E4 | E6) if node_pair[1] == demand_name])
            m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in target_arc for k in range(uav_num))
                        >= 1)
            # print(f"DEBUG: route constraint added, with total_constr_num {len(m.getConstrs())}")

        # 1-b: each uav k  can leave starting CS only once if deployed
        for k in range(uav_num):
            m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E1 | E8)) == z[k])

        # 1-c: each uav k must go back to CS if deployed
        for k in range(uav_num):
            m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E7 | E8)) == z[k])

        # 1-d: each uav k could charge for h times
        for k in range(uav_num):
            for h in range(1, self.H):
                m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E2 | E5)
                                        if bool(re.match(pattern_cs_h[h], name2)))
                            <= z[k])
            # index dominance constraint for the number of charging stations
            for h in range(1, self.H-1):
                m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E2 | E5)
                                        if bool(re.match(pattern_cs_h[h], name2)))
                            <= gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E2 | E5)
                                        if bool(re.match(pattern_cs_h[h+1], name2)))
                            )

        # 1-e: each uav k have route flow balance constraint
        for k in range(uav_num):
            for name in name_all:
                if not bool(re.match(pattern_cs_0, name)) and not bool(re.match(pattern_cs_h[self.H-1], name)):

                    target_arc_lhs = [(name1, name2) for (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7)
                                      if name2 == name]
                    target_arc_rhs = [(name1, name2) for (name1, name2) in (E2 | E3 | E4 | E5 | E6 | E7 | E8)
                                      if name1 == name]

                    m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in target_arc_lhs)
                                == gp.quicksum(x[(k, name1, name2)] for (name1, name2) in target_arc_rhs))
                    # print(f"DEBUG: Constraint for route flow balance added.")
        m.update()

        # Time variables
        t_leaving = {}          # time for uav k leaving node (i): Dict{(k, i): float}
        t_charging = {}         # time for uav k being charged at node (i): Dict{(k, i): float}
        t_resting = {}          # time for uav k resting at node (i): Dict{(k, i): float
        t_completion = {}
        for k in range(uav_num):
            for id, loc in enumerate(locations_all):
                name = name_all[id]
                t_leaving[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_leaving_uav_{k}_'+name)
                t_charging[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_charging_uav_{k}_'+name)
                t_resting[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_resting_uav_{k}_'+name)
                t_completion[k] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_completion_uav_{k}')
        t_latest = m.addVar(vtype=GRB.CONTINUOUS, name='t_latest')

        # Time constraints
        for k in range(uav_num):
            for id1, loc1 in enumerate(locations_all):
                name1 = name_all[id1]
                for id2, loc2 in enumerate(locations_all):
                    name2 = name_all[id2]
                    # 2-a relational constraints
                    if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):

                        m.addConstr(t_leaving[(k, name1)] - t_leaving[(k, name2)] + self.travel_times[(loc1, loc2)]
                                    + t_charging[(k, name2)] + t_resting[(k, name2)]
                                    <= self.M_time * (1 - x[(k, name1, name2)])
                                    )

                        m.addConstr(- t_leaving[(k, name1)] + t_leaving[(k, name2)] - self.travel_times[(loc1, loc2)]
                                    - t_charging[(k, name2)] - t_resting[(k, name2)]
                                    <= self.M_time * (1 - x[(k, name1, name2)])
                                    )
                        # print(f"DEBUG: Constraint for time relations added.")

                if loc1 in demands_filtered_loc:
                    d = demands_filtered[demands_filtered_loc.index(loc1)]
                    flag = gp.quicksum(x[(k, name, name1)] for (name, name1) in (E3 | E4 | E6))
                    m.addConstr(t_leaving[(k, name1)] - t_resting[(k, name1)] <= d['t_ub'] * flag)
                    m.addConstr(t_leaving[(k, name1)] - t_resting[(k, name1)] >= d['t_lb'] * flag)
                # 2-c charging time constraints
                if loc1 not in self.cs_locations:

                    m.addConstr(t_charging[(k, name1)] == 0)
                    # print(f"DEBUG: Constraint for charging time added (for {name1}).")

                    # 2-d resting time constraints
                    if loc1 in demands_filtered_loc:

                        m.addConstr(t_resting[(k, name1)] == self.handling_time)
                        # print(f"DEBUG: Constraint for resting time added (for {name1}).")
                    else:

                        m.addConstr(t_resting[(k, name1)] == 0)
                        # print(f"DEBUG: Constraint for resting time added (for {name1}).")

                # 2-a-2 completion time for uav_k constraints
                m.addConstr(t_completion[k] >= t_leaving[(k, name1)])
                # print(f"DEBUG: Constraint for completion time added (for {name1}).")
        # 2-d completion time constraints
        m.addConstrs(t_latest >= t_completion[k] for k in range(uav_num))
        m.update()

        # print(f"DEBUG: Constraint for latest completion time.")

        # Weight variables
        m_leaving = {}          # weight for uav k leaving node (i): Dict{(k, i): float}
        m_changed = {}          # weight changed for uav k leaving node (i): Dict{(k, i): float}
        for k in range(uav_num):
            for name in name_all:
                m_leaving[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=config.UAV.WEIGHT_CAPACITY)
                m_changed[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=config.UAV.WEIGHT_CAPACITY)

        # Weight constraints
        for k in range(uav_num):
        # 3-a relational constraint
            for id1, loc1 in enumerate(locations_all):
                for id2, loc2 in enumerate(locations_all):
                    name1 = name_all[id1]
                    name2 = name_all[id2]
                    if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                        m.addConstr(m_leaving[(k, name1)] - m_leaving[(k, name2)] + m_changed[(k, name2)]
                                    <= (1 - x[(k, name1, name2)]) * self.M_load)
                        m.addConstr(- m_leaving[(k, name1)] + m_leaving[(k, name2)] - m_changed[(k, name2)]
                                    <= (1 - x[(k, name1, name2)]) * self.M_load)
                # 3-b weight change constraint
                if loc1 not in self.cs_locations:
                    # weight change is negative at demand
                    if name1 in demands_filtered_name:
                        m_j = demands_filtered[demands_filtered_name.index(name1)]["mass"]
                        m.addConstr(m_changed[(k, name1)] == - m_j)
                    # weight change is positive at warehouse
                    elif bool(re.match(pattern_wh, name1)):
                        m.addConstr(m_changed[(k, name1)] >= 0)
                else:
                    # weight change is zero at charging stations
                    m.addConstr(m_changed[(k, name1)] == 0)
            # 3-c initial weight constraint
            m.addConstr(m_leaving[(k, f'cs_{cs_id}_0')] == 0)
        m.update()

        # Energy variables
        e_leaving = {}
        e_charged = {}
        e_consumed = {}
        for k in range(uav_num):
            for name in name_all:
                e_leaving[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=config.UAV.S_MIN * config.UAV.BATTERY_CAPACITY,
                                                ub=config.UAV.S_MAX * config.UAV.BATTERY_CAPACITY)
                e_charged[(k, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
                for name2 in name_all:
                    if (name, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                        e_consumed[(k, name, name2)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
        # Energy constraints
        for k in range(uav_num):
        # 4-a energy relational constraint
            for id1, loc1 in enumerate(locations_all):
                for id2, loc2 in enumerate(locations_all):
                    name1 = name_all[id1]
                    name2 = name_all[id2]
                    if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                        m.addConstr(
                            e_leaving[(k, name1)] - e_leaving[(k, name2)] - e_consumed[(k, name1, name2)]
                            + e_charged[(k, name2)] <= (1 - x[(k, name1, name2)]) * self.M_energy
                        )
                        m.addConstr(
                            - e_leaving[(k, name1)] + e_leaving[(k, name2)] + e_consumed[(k, name1, name2)]
                            - e_charged[(k, name2)] <= (1 - x[(k, name1, name2)]) * self.M_energy
                        )

                        # 4-b energy consumption constraint
                        m.addConstr(e_consumed[(k, name1, name2)] == self.travel_times[(loc1, loc2)]
                                    * (config.UAV.BATTERY_SLOPE * m_leaving[(k, name1)] + config.UAV.BATTERY_INTERCEPT)
                                    )
                # 4-c energy charging constraint
                if loc1 in self.cs_locations:
                    m.addConstr(e_charged[(k, name1)] == t_charging[(k, name1)] * config.UAV.CHARGING_POWER_RATED)
                else:
                    m.addConstr(e_charged[(k, name1)] == 0)         # cannot charge else where

            # 4-D initial energy constraint
            m.addConstr(e_leaving[(k, f'cs_{cs_id}_0')] == config.UAV.BATTERY_INITIAL)
        m.update()

        # Validate constraint
        constraint_count = len(m.getConstrs())
        if constraint_count == 0:
            raise ValueError("No constraints were added to the model")
        self.sol_logger.debug(f"Total constraints added: {constraint_count}")

        # Solve the subproblem
        m.setObjective(gp.quicksum(t_completion[k] for k in range(uav_num)) * config.Model.TIME_UNIT_COST
                       + t_latest * config.Model.TIME_UNIT_COST,
                       sense=GRB.MINIMIZE)


        m.setParam(GRB.param.OutputFlag, 1)
        # m.setParam(GRB.param.ImproveStartTime, 600)         # adopt a strategy that gives up on moving the best bound
                                                            # and instead devotes all of its effort towards finding
                                                            # better feasible solutions
        # m.setParam(GRB.param.)
        m.setParam(GRB.param.LogFile, self.log_file_path)
        m.setParam(GRB.param.MIPFocus, 3)                   # Focus on finding objective bound more quickly
        m.optimize()

        if m.status == GRB.OPTIMAL:
            self.sol_logger.info(f"Found optimal solution for CS {cs_id} in scenario {scenario_id}")

            r_vars = [z, x]
            t_vars = [t_leaving, t_resting, t_charging, t_completion, t_latest]
            m_vars = [m_leaving, m_changed]
            e_vars = [e_leaving, e_charged, e_consumed]

            self._get_sol_per_cs_per_sim(cs_id=cs_id, scenario_id=scenario_id,
                                        uav_num=uav_num, routing_variables=r_vars,
                                        time_variables=t_vars, mass_variables=m_vars,
                                        energy_variables=e_vars, filtered_demand=demands_filtered)
            return r_vars, t_vars, m_vars, e_vars
        else:
            logging.error(f"Failed to find optimal solution. Model status: {m.status}")
            return f"Model status: {m.status}"

    def _create_edge_sets(self, cs_id: int, name_all: list,
                          pattern_wh: str, pattern_cs_0: str,
                          pattern_cs_h: list, pattern_d: str) -> tuple:
        """
        Creates edge sets E1-E8 based on mathematical definitions:
        E1: edges from initial CS n to warehouses
        E2: edges from warehouses to charging stations (h=2...H-1)
        E3: edges from warehouses to demands
        E4: edges between different demands
        E5: edges from demands to charging stations (h=2...H-1)
        E6: edges from charging stations to demands (h=2...H-1)
        E7: edges from demands to final charging station H
        E8: edges from initial CS n to final charging station H
        """
        E1, E2, E3, E4, E5, E6, E7, E8 = set(), set(), set(), set(), set(), set(), set(), set()

        for name_1 in name_all:
            for name_2 in name_all:
                if name_1 == name_2:
                    continue

                # E1: Initial CS to Warehouses
                # {(i,j) | i = n, j ∈ W}
                if (re.match(pattern_cs_0, name_1) and
                    re.match(pattern_wh, name_2)):
                    E1.add((name_1, name_2))

                # E2: Warehouses to Intermediate Charging Stations
                # {(i,j) | i ∈ W, j ∈ Ch, h = 2,...,H-1}
                if re.match(pattern_wh, name_1):
                    for h in range(1, self.H-1):  # h starts from 1 to H-2
                        if re.match(pattern_cs_h[h], name_2):
                            E2.add((name_1, name_2))

                # E3: Warehouses to Demands
                # {(i,j) | i ∈ W, j ∈ D}
                if (re.match(pattern_wh, name_1) and
                    re.match(pattern_d, name_2)):
                    E3.add((name_1, name_2))

                # E4: Between Different Demands
                # {(i,j) | i ∈ D, j ∈ D, i ≠ j}
                if (re.match(pattern_d, name_1) and
                    re.match(pattern_d, name_2)):
                    E4.add((name_1, name_2))

                # E5: Demands to Intermediate Charging Stations
                # {(i,j) | i ∈ D, j ∈ Ch, h = 2,...,H-1}
                if re.match(pattern_d, name_1):
                    for h in range(1, self.H-1):
                        if re.match(pattern_cs_h[h], name_2):
                            E5.add((name_1, name_2))

                # E6: Intermediate Charging Stations to Demands
                # {(i,j) | i ∈ Ch, j ∈ D, h = 2,...,H-1}
                for h in range(1, self.H-1):
                    if (re.match(pattern_cs_h[h], name_1) and
                        re.match(pattern_d, name_2)):
                        E6.add((name_1, name_2))

                # E7: Demands to Final Charging Station
                # {(i,j) | i ∈ D, j ∈ CH}
                if (re.match(pattern_d, name_1) and
                    re.match(pattern_cs_h[self.H-1], name_2)):
                    E7.add((name_1, name_2))

                # E8: Initial CS to Final Charging Station
                # {(i,j) | i = n, j ∈ CH}
                if (re.match(pattern_cs_0, name_1) and
                    re.match(pattern_cs_h[self.H-1], name_2)):
                    E8.add((name_1, name_2))

        # Validate edge sets
        edge_counts = {
            'E1': len(E1), 'E2': len(E2), 'E3': len(E3), 'E4': len(E4),
            'E5': len(E5), 'E6': len(E6), 'E7': len(E7), 'E8': len(E8)
        }
        self.sol_logger.info(f"Edge set sizes: {edge_counts}")

        if not any([E1, E2, E3, E4, E5, E6, E7, E8]):
            raise ValueError("No edges were created in any set")

        return E1, E2, E3, E4, E5, E6, E7, E8

    def create_route_variables(self, m: gp.Model, uav_num: int, name_all: list) -> tuple:
        """
        Creates route variables and edge sets for the VRP problem.

        Args:
            m: Gurobi model
            uav_num: Number of UAVs
            name_all: List of all node names

        Returns:
            tuple: (x, E1, E2, E3, E4, E5, E6, E7, E8)
                x: Dictionary of route variables
                E1-E8: Edge sets as defined in the mathematical formulation
        """
        # Create edge sets
        E1, E2, E3, E4, E5, E6, E7, E8 = self.create_edge_sets(
            self.cs_id, name_all, self.pattern_wh, self.pattern_cs_0,
            self.pattern_cs_h, self.pattern_d
        )

        # Initialize route variables dictionary
        x = {}

        # Create variables for each UAV and valid edge
        for k in range(uav_num):
            # E1: Initial CS to Warehouses
            for name1, name2 in E1:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E2: Warehouses to Intermediate Charging Stations
            for name1, name2 in E2:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E3: Warehouses to Demands
            for name1, name2 in E3:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E4: Between Different Demands
            for name1, name2 in E4:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E5: Demands to Intermediate Charging Stations
            for name1, name2 in E5:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E6: Intermediate Charging Stations to Demands
            for name1, name2 in E6:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E7: Demands to Final Charging Station
            for name1, name2 in E7:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

            # E8: Initial CS to Final Charging Station
            for name1, name2 in E8:
                x[(k, name1, name2)] = m.addVar(
                    vtype=GRB.BINARY,
                    name=f'x_uav_{k}_{name1}-{name2}'
                )

        return x, E1, E2, E3, E4, E5, E6, E7, E8


    def _get_sol_per_cs_per_sim(self, cs_id: int, scenario_id: int,
                               uav_num, routing_variables: Tuple[Any],
                               time_variables, mass_variables, energy_variables,
                                filtered_demand: List[Dict]
                               ):
        z, x = routing_variables
        t_leaving, t_resting, t_charging, t_completion, t_latest = time_variables
        m_leaving, m_changed = mass_variables
        e_leaving, e_charged, e_consumed = energy_variables

        filename = config.Path.HOME+ f'/output/two_stage_solution/iter {self.iter_count}_cs {cs_id}_scenario {scenario_id}.txt'
        with open(filename, 'w') as f:
            f.write(f'Charging station {cs_id} is assigned with {uav_num} uavs.\n')
            f.write("=" * 80 + "\n")

            f.write(f"Assigned Demand Table:\n")
            f.write(" Id | Addr Id | Time LB | Time UB | Mass \n")
            for d in filtered_demand:
                f.write(f" {d['id']} |   {d['addr_id']}   |   {d['t_lb']:.2f}   | {d['t_ub']:.2f}   | {d['mass']:.3f} \n")

            f.write("=" * 80 + "\n")
            for k in range(uav_num):
                if z[k].X > 0:
                    f.write(f"UAV {k} is deployed with task completion time {t_completion[k].X: .3f}.\n")
                    f.write("\t"+"-"*76+"\n")
                    related_route = [var for key, var in x.items() if key[0] == k]
                    for r_var in related_route:
                        if r_var.X > 0:
                            f.write(f"\t Route {r_var.varName} is chosen.\n")
                    f.write("\t"+"-"*76+"\n")
                    for key, var in t_leaving.items():
                        if key[0] == k and var.X > 0:
                            f.write(f"\t Time to leave node {key[1]} is {var.X: .3f}.\n")
                    f.write("\t"+"-"*76+"\n")
                    for key, var in t_charging.items():
                        if key[0] == k and var.X > 0:
                            f.write(f"\t Time to charge at node {key[1]} is {var.X: .3f}.\n")
                    f.write("\t" + "-" * 76+"\n")
                    for key, var in m_changed.items():
                        if key[0] == k:
                            if var.X > 0:
                                f.write(f"\t Mass loaded at warehouse {key[1]} is {var.X: .3f}.\n")
                            elif var.X < 0:
                                f.write(f"\t Mass unloaded at demand {key[1]} is {var.X: .3f}.\n")
                    f.write("\t" + "-" * 76+"\n")
                    for key, var in e_charged.items():
                        if key[0] == k and var.X > 0:
                            f.write(f"\t Energy charged at charging station {key[1]} is {var.X: .3f}.\n")
                    f.write("=" * 80 + "\n")

                else:
                    f.write(f"UAV {k} is not deployed.\n")
                    f.write("=" * 80 + "\n")
        return

    def vis_sol_per_cs_per_sim(self):
        return
    def solve_second_stage(self):
        return

    def get_sol_second_stage(self):
        return

