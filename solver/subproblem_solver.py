"""
Author: Xiyue Chen
Date:   2024-11-27
Usage:  Solve the 2-nd stage routing sub-problem under a realized scenario \omega.
        The investment variable, UAV assignment variable, customer-charging center coverage is pre-determined.
"""
from typing import List, Tuple, Dict, Set
import re

from gurobipy import GRB
import gurobipy as gp
import numpy as np
from collections import defaultdict
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
                 parameters: Parameter
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

    def _calc_energy_consumption(self, m):
        return self.uav_energy_a * m + self.uav_energy_b

    def solve_model_per_cs_per_sim(self, cs_id, scenario_id) -> Dict:
        """
        Solve the VRP sub-problem for charging station i under scenario j
        Returns:
            Dict of subproblem solution
        """
        print(f"INFO: Build subproblem for charging station {cs_id} under scenario {scenario_id}.")
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

        t_lb = [d['t_lb'] for d in demands_filtered]
        t_ub = [d['t_ub'] for d in demands_filtered]


        m = gp.Model(name=f"scenario {scenario_id}-cs {cs_id}")

        # Route variables
        x = {}                  # key: [k, node1, node2]

        E1, E2, E3, E4, E5, E6, E7, E8 = set(), set(), set(), set(),set(), set(), set(), set()
                                # set to store the edge index

        for k in range(uav_num):
            for name_1 in name_all:
                for name_2 in name_all:
                    if name_1 != name_2:

                        #E1 E8
                        if bool(re.match(pattern_cs_0, name_1)):
                            if bool(re.match(pattern_wh, name_2)):
                                if k == 0:
                                    E1.add((name_1, name_2))
                                x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                  name=f'x_uav_{k}_{name_1}-{name_2}')

                            elif bool(re.match(pattern_cs_h[self.H-1], name_2)):
                                if k == 0:
                                    E8.add((name_1, name_2))
                                x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                  name=f'x_uav_{k}_{name_1}-{name_2}')

                        # E3 E2
                        if bool(re.match(pattern_wh, name_1)):
                            if bool(re.match(pattern_d, name_2)):
                                if k == 0:
                                    E3.add((name_1, name_2))
                                x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                  name=f'x_uav_{k}_{name_1}-{name_2}')

                            else:
                                for h in range(1, self.H-1):
                                    if bool(re.match(pattern_cs_h[h], name_2)):
                                        if k == 0:
                                            E2.add((name_1, name_2))
                                        x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                          name=f'x_uav_{k}_{name_1}-{name_2}')

                        # E4 E5 E7
                        if bool(re.match(pattern_d, name_1)):
                            if bool(re.match(pattern_d, name_2)):
                                if k == 0:
                                    E4.add((name_1, name_2))
                                x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                  name=f'x_uav_{k}_{name_1}-{name_2}')

                            elif bool(re.match(pattern_cs_h[self.H-1], name_2)):
                                if k == 0:
                                    E7.add((name_1, name_2))
                                x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                  name=f'x_uav_{k}_{name_1}-{name_2}')

                            else:
                                for h in range(1, self.H-1):
                                    if bool(re.match(pattern_cs_h[h], name_2)):
                                        if k == 0:
                                            E5.add((name_1, name_2))
                                        x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                          name=f'x_uav_{k}_{name_1}-{name_2}')


                        for h in range(1, self.H-1):
                            if bool(re.match(pattern_cs_h[h], name_1)):
                                if bool(re.match(pattern_d, name_2)):
                                    if k == 0:
                                        E6.add((name_1, name_2))
                                    x[(k, name_1, name_2)] = m.addVar(vtype=GRB.BINARY,
                                                                      name=f'x_uav_{k}_{name_1}-{name_2}')

        # Route constraints
        # 1-a: all demands must be satisfied
        for did in demands_filtered_id:
            demand_name = f'd_{did}'
            target_arc = set([node_pair for node_pair in (E3 | E4 | E6) if node_pair[1] == demand_name])
            m.addConstr(gp.quicksum(x[(k, name1, name2)]
                                    for (name1, name2) in target_arc
                                    for k in range(uav_num))
                        == 1)
        # 1-b: each uav k  can leave starting CS only once or not
        for k in range(uav_num):
            m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in (E1 | E8)) <= 1)

        # 1-c: each uav k must go back to CS
        for k in range(uav_num):
            m.addConstr(gp.quicksum(x[(k, name1, name2)]
                                    for (name1, name2) in (E7 | E8))
                        <= 1)
        # 1-d: each uav k could charge for h times
        for k in range(uav_num):
            for h in range(1, self.H):
                m.addConstr(gp.quicksum(x[(k, name1, name2)]
                                        for (name1, name2) in E5
                                        if bool(re.match(pattern_cs_h[h], name_2)))
                            <= 1)

        # 1-e: each uav k have flow balance constraint
        for k in range(uav_num):
            for name in name_all:
                if not bool(re.match(pattern_cs_0, name)) and not bool(re.match(pattern_cs_h[self.H-1], name)):

                    target_arc_lhs = [(name1, name2) for (name1, name2) in (E1 | E2 | E3 | E4 | E5 | E6 | E7)
                                      if name2 == name]
                    target_arc_rhs = [(name1, name2) for (name1, name2) in (E2 | E3 | E4 | E5 | E6 | E7 | E8)
                                      if name1 == name]
                    m.addConstr(gp.quicksum(x[(k, name1, name2)] for (name1, name2) in target_arc_lhs)
                                == gp.quicksum(x[(k, name1, name2)] for (name1, name2) in target_arc_rhs))


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
                        m.addConstr(t_leaving[(k, name1)] - t_leaving[(k, name2)] + self.travel_times[(loc1, loc2)] \
                                    + t_charging[(k, name2)] + t_resting[(k, name2)] \
                                    <= self.M_time * (1 - x[(k, name1, name2)])
                                    )
                        m.addConstr(- t_leaving[(k, name1)] + t_leaving[(k, name2)] - self.travel_times[(loc1, loc2)] \
                                    - t_charging[(k, name2)] - t_resting[(k, name2)] \
                                    <= self.M_time * (1 - x[(k, name1, name2)])
                                    )

                # 2-b 2-c charging time constraints
                if loc1 not in self.cs_locations:
                    m.addConstr(t_charging[(k, name1)] == 0)

                    # 2-c resting time constraints
                    if loc1 in demands_filtered_loc:

                        m.addConstr(t_resting[(k, name1)] == self.handling_time)
                    else:
                        m.addConstr(t_resting[(k, name1)] == 0)

                # 2-a-2 completion time for uav_k constraints
                m.addConstr(t_completion[k] >= t_leaving[(k, name1)])
        # 2-d completion time constraints
        m.addConstrs(t_latest >= t_completion[k] for k in range(uav_num))

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
                        m_j = demands_filtered[
                            demands_filtered_name.index(name1)
                        ]["mass"]
                        m.addConstr(m_changed[(k, name1)] == - m_j)
                    # weight change is positive at warehouse
                    elif bool(re.match(pattern_wh, name1)):
                        m.addConstr(m_changed[(k, name1)] >= 0)
                else:
                    # weight change is zero at charging stations
                    m.addConstr(m_changed[(k, name1)] == 0)
        # 3-c initial weight constraint
            m.addConstr(m_leaving[(k, f'cs_{cs_id}_0')] == 0)

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
                        m.addConstr(e_consumed[(k, name1, name2)] == self.travel_times[(loc1, loc2)] \
                                    * (config.UAV.BATTERY_SLOPE * m_leaving[(k, name1)] + config.UAV.BATTERY_INTERCEPT)
                                    )
        # 4-c energy charging constraint
                if loc1 in self.cs_locations:
                    m.addConstr(e_charged[(k, name1)] == t_charging[(k, name1)] * config.UAV.CHARGING_POWER_RATED)

        # 4-D initial energy constraint
            m.addConstr(e_leaving[(k, f'cs_{cs_id}_0')] == config.UAV.BATTERY_INITIAL)

        # Validate constraint
        constraint_count = len(m.getConstrs())
        if constraint_count == 0:
            raise ValueError("No constraints were added to the model")
        print(f"Total constraints added: {constraint_count}")

        # Solve the subproblem
        m.setObjective(gp.quicksum(t_completion[k] for k in range(uav_num)) * config.Model.TIME_UNIT_COST \
                       + t_latest * config.Model.TIME_UNIT_COST,
                       sense=GRB.MINIMIZE)
        m.optimize()

        if m.status == GRB.OPTIMAL:
            print(x)
            return
        else:
            return f"Model status: {m.status}"


    def get_sol_per_cs_per_sim(self):
        return
    def vis_sol_per_cs_per_sim(self):
        return
    def solve_second_stage(self):
        return

    def get_sol_second_stage(self):
        return

