"""
Author: Xiyue Chen
Date:   2024-12-10
Usage:  Solve the 2-nd stage TSP sub-problem under a realized scenario \omega.
        The investment variable, UAV assignment variable, customer-charging center coverage is pre-determined.
"""
from typing import List, Tuple, Dict
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
from solver.init_sol_new import InvestmentVariable

class TSPSolver:
    def __init__(self, config,
                 demand_scenarios: DeliveryScenario,
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
        self.investment_variable = investment_variable
        self.working_station: List[bool] = self.investment_variable.working_charging_station    # binary for setup CS n
        self.uav_num_lb: List[int] = self.investment_variable.uav_num_lb                        # lb for UAV num
        self.cs_uav_deployment: Dict[int, List[str]] = self.investment_variable.cs_uav_deployment
                                                                                    # cs_id -> [uav_name]
        self.uav_demand_assignment: List[Dict[str, List[int]]] = self.investment_variable.uav_demand_assignment
                                                                                    # scenario -> uav_name -> demand

        # pattern for recognition
        self.pattern_wh = f'^wh_\\d+$'
        self.pattern_cs_0 = [f'^cs_{cs_id}_0$' for cs_id in range(self.cs_num)]
        self.pattern_cs_h = [f'^cs_\\d+_{h}' for h in range(self.H)]
        self.pattern_d = f'^d_\\d+$'
        # logging
        self.sol_logger = sol_logger
        # set logging handler to existing sol_logger passed in to the object
        if isinstance(self.sol_logger.handlers[0], logging.FileHandler):
            self.log_file_path = self.sol_logger.handlers[0].baseFilename

        # iter_count
        self.iter_count = iter_count

    def _calc_energy_consumption(self, m):
        return self.uav_energy_a * m + self.uav_energy_b

    def _filter_demand(self, uav_demand, scenario_id) -> Tuple[List, List, List, List, List]:
        """Filter target demand for selected UAV, return lists for demand, demand locations, demand id"""
        demands_w: List[Dict] = self.clustered_scenarios[scenario_id]
        demands_filtered = [d for d in demands_w if d['id'] in uav_demand]
        demands_filtered_id = [d['id'] for d in demands_filtered]           # essentially uav_demand
        demands_filtered_name = [f'd_{d_id}' for d_id in demands_filtered_id]
        demands_filtered_loc = [d['location'] for d in demands_filtered]
        return demands_filtered, demands_filtered_id, demands_filtered_name, demands_filtered_loc

    def _get_tsp_sol_per_cs_per_scenario(self, cs_id: int, scenario_id: int, working_uav_count: int,
                                         x: Dict,
                                         t_leaving: Dict, t_charging: Dict, t_resting: Dict, t_completion: Dict,
                                         m_leaving: Dict, m_changed: Dict,
                                         e_leaving: Dict, e_charged: Dict, e_consumed: Dict,
                                         filtered_demand_cs: Dict[str, List[Dict]]):
        """
        Output the combined TSP result that solve for all working UAV k at CS n under scenario
        Args:
            All input are combined dictionary or list that stores the solution variables.
        """

        filename = config.Path.HOME+ f'/output/tsp_solution/iter {self.iter_count}_scenario {scenario_id}_cs {cs_id}.txt'
        with open(filename, 'w') as f:
            f.write(f'Charging station {cs_id} is assigned with {working_uav_count} uavs.\n')
            f.write("=" * 120 + "\n")

            f.write(f"Assigned Demand Table:\n")
            f.write(" Id | Addr Id | Time LB | Time UB | Mass |  UAV  |\n")
            for uav_name, filtered_demand in filtered_demand_cs.items():
                for d in filtered_demand:
                    f.write(f" {d['id']} |   {d['addr_id']}   |  {d['t_lb']:.2f}  |  {d['t_ub']:.2f}  | {d['mass']:.3f} | {uav_name} |\n")
            f.write("=" * 120 + "\n")

            for uav_name, filtered_demand in filtered_demand_cs.items():
                if len(filtered_demand) > 0:
                    f.write(f"{uav_name} is deployed with task completion time {t_completion[uav_name].X: .3f}.\n")
                    f.write("\t"+"-"*116+"\n")
                    related_route = [var for key, var in x.items() if key[0] == uav_name]
                    for r_var in related_route:
                        if r_var.X > 0:
                            f.write(f"\t Route {r_var.varName} is chosen.\n")
                    f.write("\t"+"-"*116+"\n")
                    for key, var in t_leaving.items():
                        if key[0] == uav_name and var.X > 0:
                            f.write(f"\t Time to leave node {key[1]} is {var.X: .3f}.\n")
                    f.write("\t"+"-"*116+"\n")
                    for key, var in t_charging.items():
                        if key[0] == uav_name and var.X > 0:
                            f.write(f"\t Time to charge at node {key[1]} is {var.X: .3f}.\n")
                    f.write("\t" + "-" * 116+"\n")
                    for key, var in m_changed.items():
                        if key[0] == uav_name:
                            if var.X > 0:
                                f.write(f"\t Mass loaded at warehouse {key[1]} is {var.X: .3f}.\n")
                            elif var.X < 0:
                                f.write(f"\t Mass unloaded at demand {key[1]} is {var.X: .3f}.\n")
                    f.write("\t" + "-" * 116+"\n")
                    for key, var in e_charged.items():
                        if key[0] == uav_name and var.X > 0:
                            f.write(f"\t Energy charged at charging station {key[1]} is {var.X: .3f}.\n")
                    f.write("=" * 120 + "\n")

                else:
                    f.write(f"UAV {uav_name} is not deployed.\n")
                    f.write("=" * 120 + "\n")
        return

    def create_edge_sets(self, cs_id: int, name_all: list, pattern_wh: str, pattern_cs_0: str,
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

    def create_route_variables(self, m: gp.Model, cs_id: int, uav_name: str, name_all: list) -> tuple:
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
        E1, E2, E3, E4, E5, E6, E7, E8 = self.create_edge_sets(cs_id, name_all, self.pattern_wh,
                                                                self.pattern_cs_0[cs_id], self.pattern_cs_h,
                                                                self.pattern_d)

        # Initialize route variables dictionary
        x = {}
        # E1: Initial CS to Warehouses
        for name1, name2 in E1:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E2: Warehouses to Intermediate Charging Stations
        for name1, name2 in E2:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E3: Warehouses to Demands
        for name1, name2 in E3:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E4: Between Different Demands
        for name1, name2 in E4:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E5: Demands to Intermediate Charging Stations
        for name1, name2 in E5:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E6: Intermediate Charging Stations to Demands
        for name1, name2 in E6:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E7: Demands to Final Charging Station
        for name1, name2 in E7:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        # E8: Initial CS to Final Charging Station
        for name1, name2 in E8:
            x[(uav_name, name1, name2)] = m.addVar(vtype=GRB.BINARY, name=f'x_{uav_name}_{name1}-{name2}')

        return x, E1, E2, E3, E4, E5, E6, E7, E8

    def add_route_constraints(self, m: gp.Model, cs_id: int, uav_name: str, name_all: List, locations_all: List,
                             demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                             x, E1, E2, E3, E4, E5, E6, E7, E8):
        # 1-a: all demands must be visited at least once (can be visited again)
        for did in demands_filtered_id:
            demand_name = f'd_{did}'
            target_arc = set([node_pair for node_pair in (E3 | E4 | E6) if node_pair[1] == demand_name])
            m.addConstr(gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in target_arc)
                        >= 1)
            # print(f"DEBUG: route constraint added, with total_constr_num {len(m.getConstrs())}")

        # 1-b: each uav k must leave starting CS only once if deployed
        m.addConstr(gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in (E1 | E8)) == 1)

        # 1-c: each uav k must go back to CS if deployed
        m.addConstr(gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in (E7 | E8)) == 1)

        # 1-d: each uav k could charge for h times
        for h in range(1, self.H):
            m.addConstr(gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in (E2 | E5)
                                    if bool(re.match(self.pattern_cs_h[h], name2_)))
                        <= 1)

        # 1-e: each uav k have route flow balance constraint
        for name in name_all:
            if not bool(re.match(self.pattern_cs_0[cs_id], name)) and not bool(re.match(self.pattern_cs_h[self.H - 1],
                                                                                        name)):
                target_arc_lhs = [(name1_, name2_) for (name1_, name2_) in (E1 | E2 | E3 | E4 | E5 | E6)
                                  if name2_ == name]
                target_arc_rhs = [(name1_, name2_) for (name1_, name2_) in (E2 | E3 | E4 | E5 | E6 | E7)
                                  if name1_ == name]

                m.addConstr(gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in target_arc_lhs)
                            == gp.quicksum(x[(uav_name, name1_, name2_)] for (name1_, name2_) in target_arc_rhs))

    def create_time_variables(self, m: gp.Model, cs_id: int, uav_name: str, name_all: list) -> tuple:
        t_leaving = {}  # time for uav k leaving node (i): Dict{(k, i): float}
        t_charging = {}  # time for uav k being charged at node (i): Dict{(k, i): float}
        t_resting = {}  # time for uav k resting at node (i): Dict{(k, i): float
        t_completion = {}
        for _, name in enumerate(name_all):
            t_leaving[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_leaving_{uav_name}_' + name)
            t_charging[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_charging_{uav_name}_' + name)
            t_resting[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_resting_{uav_name}_' + name)
        t_completion[uav_name] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f't_completion_{uav_name}')

        return t_leaving, t_charging, t_resting, t_completion

    def add_time_constraints(self, m: gp.Model, cs_id: int, uav_name: str, name_all: List, locations_all: List,
                             demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                             x, E1, E2, E3, E4, E5, E6, E7, E8, t_leaving, t_charging, t_resting, t_completion):
        for id1, loc1 in enumerate(locations_all):
            name1 = name_all[id1]
            for id2, loc2 in enumerate(locations_all):
                name2 = name_all[id2]
                # 2-a relational constraints
                if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                    # m.addConstr(t_leaving[(uav_name, name2)] - t_leaving[(uav_name, name1)]
                    #             == self.travel_times[(loc1, loc2)]
                    #             + t_charging[(uav_name, name2)] + t_resting[(uav_name, name2)]
                    #             if bool(x[(uav_name, name1, name2)])
                    #             )
                    # Simpler linearization
                    # m.addConstr(t_leaving[(uav, name2)] >=
                    #             t_leaving[(uav_name, name1)] + self.travel_times[(loc1, loc2)]
                    #             + t_charging[(uav_name, name2)] + t_resting[(uav_name, name2)]
                    #             - self.M_time * (1 - x[(uav_name, name1, name2)]))
                    # Linearization
                    m.addConstr(t_leaving[(uav_name, name1)] - t_leaving[(uav_name, name2)]
                                + self.travel_times[(loc1, loc2)]
                                + t_charging[(uav_name, name2)] + t_resting[(uav_name, name2)]
                                <= self.M_time * (1 - x[(uav_name, name1, name2)])
                                )

                    m.addConstr(- t_leaving[(uav_name, name1)] + t_leaving[(uav_name, name2)]
                                - self.travel_times[(loc1, loc2)]
                                - t_charging[(uav_name, name2)] - t_resting[(uav_name, name2)]
                                <= self.M_time * (1 - x[(uav_name, name1, name2)])
                                )


            # # 2-a-1 restrictive constraint
            # m.addConstr(t_leaving[(uav_name, name1)] <= self.M_time * gp.quicksum(
            #     x[(uav_name, name1, name)] for (name1, name) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8)))


            # 2-a-1: set the time to 0 if node i is note visited
            incoming_edges = [(name, name1_) for (name, name1_) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8) if
                              name1_ == name1]
            outcoming_edges = [(name1_, name) for (name1_, name) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8) if
                               name1_ == name1]
            if incoming_edges or outcoming_edges:
                m.addConstr(t_leaving[(uav_name, name1)] <= self.M_time * (
                        gp.quicksum(x[(uav_name, name, name1_)] for (name, name1_) in incoming_edges)
                        + gp.quicksum(x[(uav_name, name1_, name)] for (name1_, name) in outcoming_edges))
                            )

            # 2-b charging time constraints
            if loc1 not in self.cs_locations:
                # Cannot charge at other node,except charging station
                m.addConstr(t_charging[(uav_name, name1)] == 0)
            else:
                # Cannot charge if charging station is not visited
                # incoming_edges = [(name, name1_) for (name, name1_) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8) if name1_ == name1]
                # outcoming_edges = [(name1_, name) for (name1_, name) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8) if name1_ == name1]
                if incoming_edges or outcoming_edges:
                    m.addConstr(t_charging[(uav_name, name1)] <= self.M_time *
                                (gp.quicksum(x[(uav_name, name, name1_)] for (name, name1_) in incoming_edges)
                                 + gp.quicksum(x[(uav_name, name1_, name)] for (name1_, name) in outcoming_edges))
                                )


            # 2-c resting time constraints
            if loc1 in demands_filtered_loc:
                # resting time for demand = handling time, every demand will be visited, no need to get incoming edges
                m.addConstr(t_resting[(uav_name, name1)] == self.handling_time)
            elif loc1 in self.cs_locations:
                # incoming_edges = [(name, name1_) for (name, name1_) in (E2 | E5 | E7 | E8) if name1_ == name1]
                if incoming_edges:
                    # resting time for cs >= 0 if CS visited
                    m.addConstr(t_resting[(uav_name, name1)] <= self.M_time *
                                gp.quicksum(x[(uav_name, name, name1_)] for (name, name1_) in incoming_edges))
            else:
                # resting time == 0 for WH
                m.addConstr(t_resting[(uav_name, name1)] == 0)

            # 2-d Time window constraints
            if name1 in demands_filtered_name:
                d = demands_filtered[demands_filtered_name.index(name1)]
                flag = gp.quicksum(x[(uav_name, name, name1_)] for (name, name1_) in (E3 | E4 | E6)
                                   if name1_ == name1)
                m.addConstr(t_leaving[(uav_name, name1)] <= d['t_ub'] * flag)
                m.addConstr(t_leaving[(uav_name, name1)] - t_resting[(uav_name, name1)] >= d['t_lb'] * flag)

            # 2-e Start time constraints
            m.addConstr(t_leaving[(uav_name, f'cs_{cs_id}_0')] == 0)
            # 2-f completion time for uav_k constraints
            m.addConstr(t_completion[uav_name] >= t_leaving[(uav_name, name1)])

    def create_mass_variables(self, m: gp.Model, cs_id: int, uav_name: str, name_all: list) -> tuple:
        # Weight variables
        m_leaving = {}  # weight for uav k leaving node (i): Dict{(k, i): float}
        m_changed = {}  # weight changed for uav k leaving node (i): Dict{(k, i): float}

        for name in name_all:
            m_leaving[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=config.UAV.WEIGHT_CAPACITY)
            m_changed[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=config.UAV.WEIGHT_CAPACITY)
        return m_leaving, m_changed

    def add_mass_constraints(self, m: gp.Model, cs_id: int, uav_name: str, name_all: List, locations_all: List,
                             demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                             x, E1, E2, E3, E4, E5, E6, E7, E8, m_leaving, m_changed):
        # 3-a relational constraint
        for id1, loc1 in enumerate(locations_all):
            name1 = name_all[id1]
            for id2, loc2 in enumerate(locations_all):
                name2 = name_all[id2]
                if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                    m.addConstr(m_leaving[(uav_name, name1)] - m_leaving[(uav_name, name2)]
                                + m_changed[(uav_name, name2)]
                                <= (1 - x[(uav_name, name1, name2)]) * self.M_load)
                    m.addConstr(- m_leaving[(uav_name, name1)] + m_leaving[(uav_name, name2)]
                                - m_changed[(uav_name, name2)]
                                <= (1 - x[(uav_name, name1, name2)]) * self.M_load)
            # 3-b weight change constraint
            if loc1 not in self.cs_locations:
                # weight change is negative at demand
                if name1 in demands_filtered_name:
                    m_j = demands_filtered[demands_filtered_name.index(name1)]["mass"]
                    m.addConstr(m_changed[(uav_name, name1)] == - m_j)
                # weight change is positive at warehouse
                elif bool(re.match(self.pattern_wh, name1)):
                    m.addConstr(m_changed[(uav_name, name1)] >= 0)
            else:
                # weight change is zero at charging stations
                m.addConstr(m_changed[(uav_name, name1)] == 0)
        # 3-c initial weight constraint
        m.addConstr(m_leaving[(uav_name, f'cs_{cs_id}_0')] == 0)

    def create_energy_variables(self, m: gp.Model, cs_id: int, uav_name: str, name_all: list,
                                E1, E2, E3, E4, E5, E6, E7, E8) -> tuple:
        # Energy variables
        e_leaving = {}
        e_charged = {}
        e_consumed = {}

        for name in name_all:
            e_leaving[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=config.UAV.S_MIN * config.UAV.BATTERY_CAPACITY,
                                            ub=config.UAV.S_MAX * config.UAV.BATTERY_CAPACITY)
            e_charged[(uav_name, name)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
            for name2 in name_all:
                if (name, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                    e_consumed[(uav_name, name, name2)] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
        return e_leaving, e_charged, e_consumed

    def add_energy_constraints(self, m: gp.Model, cs_id: int, uav_name: str, name_all: List, locations_all: List,
                               demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                               x, E1, E2, E3, E4, E5, E6, E7, E8, e_leaving, e_charged, e_consumed,
                               m_leaving, t_charging):
        # 4-a energy relational constraint
        for id1, loc1 in enumerate(locations_all):
            name1 = name_all[id1]
            for id2, loc2 in enumerate(locations_all):
                name2 = name_all[id2]
                if name1 != name2 and (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7 | E8):
                    m.addConstr(
                        e_leaving[(uav_name, name1)] - e_leaving[(uav_name, name2)]
                        - e_consumed[(uav_name, name1, name2)]
                        + e_charged[(uav_name, name2)] <= (1 - x[(uav_name, name1, name2)]) * self.M_energy
                    )
                    m.addConstr(
                        - e_leaving[(uav_name, name1)] + e_leaving[(uav_name, name2)]
                        + e_consumed[(uav_name, name1, name2)]
                        - e_charged[(uav_name, name2)] <= (1 - x[(uav_name, name1, name2)]) * self.M_energy
                    )

                    # 4-b energy consumption constraint
                    m.addConstr(e_consumed[(uav_name, name1, name2)] == self.travel_times[(loc1, loc2)]
                                * (config.UAV.BATTERY_SLOPE * m_leaving[(uav_name, name1)] + config.UAV.BATTERY_INTERCEPT)
                                )
            # 4-c energy charging constraint
            if loc1 in self.cs_locations:
                # no need to filter incoming edges (already checked in t_charging variables)
                m.addConstr(e_charged[(uav_name, name1)] == t_charging[(uav_name, name1)] * config.UAV.CHARGING_POWER_RATED)
            else:
                m.addConstr(e_charged[(uav_name, name1)] == 0)         # cannot charge else where

        # 4-d initial energy constraint
        m.addConstr(e_leaving[(uav_name, f'cs_{cs_id}_0')] == config.UAV.BATTERY_INITIAL)



    def solve_tsp(self, uav_name: str, cs_id: int, scenario_id: int,
                  demands_filtered, demands_filtered_id,
                  demands_filtered_name, demands_filtered_loc) -> Dict:
        """
        Solve tsp problem for single UAV
        Args:
            uav_name: Name of UAV (name after the initial deloyment by GRASP)
        """
        self.sol_logger.info(f"Build subproblem for uav {uav_name} for demand: {demands_filtered_id}.")

        locations_all = [self.cs_locations[cs_id]] + self.cs_locations * (self.H - 1) + \
                        self.wh_locations + demands_filtered_loc
        name_all = [f'cs_{cs_id}_0'] + [f'cs_{c_id}_{h}' for h in range(1, self.H) for c_id in range(self.cs_num)] \
                   + [f'wh_{wh_id}' for wh_id in range(self.wh_num)] \
                   + demands_filtered_name

        m = gp.Model(name=f"tsp-{uav_name}")
        # Routing variable
        x, E1, E2, E3, E4, E5, E6, E7, E8 = self.create_route_variables(m, cs_id, uav_name, name_all)

        # Routing constraints
        self.add_route_constraints(m, cs_id, uav_name, name_all, locations_all,
                                     demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                                     x, E1, E2, E3, E4, E5, E6, E7, E8)
        m.update()

        # Time variables
        t_leaving, t_charging, t_resting, t_completion = self.create_time_variables(m, cs_id, uav_name, name_all)

        # Time constraints
        self.add_time_constraints(m, cs_id, uav_name, name_all, locations_all,
                                  demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                                  x, E1, E2, E3, E4, E5, E6, E7, E8, t_leaving, t_charging, t_resting, t_completion)
        m.update()

        # Mass variables
        m_leaving, m_changed = self.create_mass_variables(m, cs_id, uav_name, name_all)

        # Mass constraints
        self.add_mass_constraints(m, cs_id, uav_name, name_all, locations_all,
                                  demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                                  x, E1, E2, E3, E4, E5, E6, E7, E8, m_leaving, m_changed)
        m.update()

        # Energy variables
        e_leaving, e_charged, e_consumed = self.create_energy_variables(m, cs_id, uav_name, name_all,
                                                                        E1, E2, E3, E4, E5, E6, E7, E8)

        # Energy constraints
        self.add_energy_constraints(m, cs_id, uav_name, name_all, locations_all,
                                    demands_filtered, demands_filtered_name, demands_filtered_loc, demands_filtered_id,
                                    x, E1, E2, E3, E4, E5, E6, E7, E8, e_leaving, e_charged, e_consumed,
                                    m_leaving, t_charging)
        m.update()

        # Validate constraint
        constraint_count = len(m.getConstrs())
        if constraint_count == 0:
            raise ValueError("No constraints were added to the model")
        self.sol_logger.debug(f"Total constraints added: {constraint_count}")

        # Solve the  subproblem
        # m.setObjective(gp.quicksum(t_completion[k] for k in range(uav_num)) * config.Model.TIME_UNIT_COST
        #                + t_latest * config.Model.TIME_UNIT_COST,
        #                sense=GRB.MINIMIZE)
        m.setObjective(t_completion[uav_name] * config.Model.TIME_UNIT_COST,
                       sense=GRB.MINIMIZE)
        # m.setObjective(gp.quicksum(t_leaving[(uav_name, name)] for name in name_all)
        #                * config.Model.TIME_UNIT_COST * config.Model.LAMBDA,
        #                sense=GRB.MINIMIZE)
        m.setParam(GRB.param.OutputFlag, 1)
        m.setParam(GRB.param.LogFile, self.log_file_path)
        # m.setParam(GRB.param.ImproveStartTime, 50)         # adopt a strategy that gives up on moving the best bound
        # and instead devotes all of its effort towards finding
        # better feasible solutions
        # m.setParam(GRB.param.)
        m.setParam(GRB.param.MIPFocus, 3)  # Focus on finding objective bound more quickly
        m.optimize()

        if m.status == GRB.OPTIMAL:
            self.sol_logger.info(f"Found optimal solution for {uav_name} at CS {cs_id}.")
            # Return both the model and the variables
            return (m, (x, t_leaving, t_charging, t_resting, t_completion, m_leaving, m_changed,
                        e_leaving, e_charged, e_consumed))
        else:
            logging.error(f"Failed to find optimal solution. Model status: {m.status}")
            return f"Model status: {m.status}"


    def solve_model_per_cs_per_sim(self, scenario_id) -> Dict:
        """
        Solve the TSP sub-problem for charging station i under scenario j
        Returns:
            Dict of subproblem solution
        """
        self.sol_logger.info(f"Solve {scenario_id}.")
        for cs_id in range(self.cs_num):
            if self.working_station[cs_id]:
                self.sol_logger.debug(f"\tCS {cs_id}: start TSP solution.")
                # Initialize dictionary to store all variables
                x_cs, t_leaving_cs, t_charging_cs, t_resting_cs, t_completion_cs, m_leaving_cs, m_changed_cs, \
                e_leaving_cs, e_charged_cs, e_consumed_cs = {}, {}, {}, {}, {}, {}, {}, {}, {}, {}
                demands_filtered_cs = {}
                working_uav_count = 0
                models = []                 # store models to keep them active

                # Iterate over all UAV with assigned demand
                for uav_name in self.cs_uav_deployment[cs_id]:
                    uav_demand = self.uav_demand_assignment[scenario_id][uav_name]
                    if len(uav_demand) > 0:
                        # Retrieve demand information
                        demands_filtered, demands_filtered_id, demands_filtered_name, demands_filtered_loc = \
                            self._filter_demand(uav_demand, scenario_id)

                        solution = self.solve_tsp(uav_name, cs_id, scenario_id,
                                                                          demands_filtered, demands_filtered_id,
                                                                          demands_filtered_name, demands_filtered_loc)
                        if isinstance(solution, tuple):
                            model, variables = solution
                            models.append(model)            # Store model to keep it active

                            x, t_leaving, t_charging, t_resting, t_completion, m_leaving, m_changed, \
                            e_leaving, e_charged, e_consumed = variables

                            demands_filtered_cs[uav_name] = demands_filtered
                            x_cs.update(x)
                            t_leaving_cs.update(t_leaving)
                            t_charging_cs.update(t_charging)
                            t_completion_cs.update(t_completion)
                            m_leaving_cs.update(m_leaving)
                            m_changed_cs.update(m_changed)
                            e_leaving_cs.update(e_leaving)
                            e_charged_cs.update(e_charged)
                            e_consumed_cs.update(e_consumed)
                            working_uav_count += 1
                        else:
                            self.sol_logger.error(f"Failed to solve TSP for UAV {uav_name}.")
                    else:
                        demands_filtered_cs[uav_name] = []
                # Only process solutions if we have valid results
                if working_uav_count > 0:
                    self._get_tsp_sol_per_cs_per_scenario(cs_id, scenario_id, working_uav_count,
                                                          x_cs,
                                                          t_leaving_cs, t_charging_cs, t_resting_cs, t_completion_cs,
                                                          m_leaving_cs, m_changed_cs,
                                                          e_leaving_cs, e_charged_cs, e_consumed_cs,
                                                          demands_filtered_cs)
                # Clean up models after writing results
                for model in models:
                    model.dispose()

            else:
                self.sol_logger.debug(f"\tCS {cs_id}: not working")

        # print(f"DEBUG: Constraint for latest completion time.")

    # def vis_sol_per_cs_per_sim(self):
    #     return
    # def solve_second_stage(self):
    #     return
    #
    # def get_sol_second_stage(self):
    #     return