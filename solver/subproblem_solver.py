"""
Author: Xiyue Chen
Date:   2024-11-27
Usage:  Solve the 2-nd stage routing sub-problem under a realized scenario \omega.
        The investment variable, UAV assignment variable, customer-charging center coverage is pre-determined.
"""
from typing import List, Tuple, Dict

from gurobipy import GRB
import gurobipy as gp
import numpy as np
from collections import defaultdict

from ..input.config import Model


def solver_uav_routing_subproblem(cs_id: int, scenario_id: int, demands_for_n: List[Dict],
                                 uav_num: int, cs_num: int, wh_num: int, grid_num: int,

                                  fixed_investments: Dict,
                                  travel_time: Dict
                                  ):
    """
    Solve sub problems of UAV routing (VRP) that split by the charging stations
     have 1 charging station, corresponding k UAVs, corresponding demand points

    Args:
        cs_index: Index of the charging station (n) for this subproblem
        scenario_idx: Index of the representative scenarios (w - omega) for this subproblem
        demands_for_n: The demands for charging station (n) in scenario (w)
            List of Dict {
                'id': demand_count,
                'grid_id': grid_id,
                'location': cell_center,
                't_lb': time,
                't_ub': time + self.response_window,
                'mass': mass
                }


        uav_num: Number of all UAVs (K) - K predetermined by initial solution.
        cs_num
        wh_num
        grid_num

        fixed_investments: Investment variables
            Dict of {
            "y": Binary list to denote constructed charging stations [0, 1, 0 ...],
            "z": Binary Dict denotes the uav deployment to station n {(n, k): 1 | 0}
            }

        travel_time: Dict {(type_ind, type_ind): float}
    Returns:
        sol: contains solution information
            Dict of {
            }

    """
    model = gp.Model(f"Subproblemn_scenario {scenario_id}_cs {cs_id}")

    # Decision Variables
    x = {}                  # binary selection for uav (n, k) to choose edge (i, j):
                            # Dict{(n:int, k:int, i:str, j:str): 0 | 1}

    E1, E3, E4, E7, E8 = [], [], [], [], []
    E2, E5, E6 = defaultdict(list), defaultdict(list), defaultdict(list)
                            # List/Dict to store the edge index
    t_leaving = {}          # time for uav (n, k) leaving node (i): Dict{(n, k, i): float}
    t_charging = {}         # time for uav (n, k) being charged at node (i): Dict{(n, k, i): float}
    t_resting = {}          # time for uav (n, k) resting at node (i): Dict{(n, k, i): float}

    t_latest = {}           # latest time for uav (n, k) to complete all orders

    e_leaving = {}          # energy of uav (n, k) leaving node (i): Dict{(n, k, i): float}
    e_charged = {}          # energy charged for uav (n, k) at CS (i): Dict{(n, k, i): float}
    e_consumed = {}         # energy consumed for uav (n, k) on edge (i, j): Dict{(n, k, i, j): float}

    m_leaving = {}          # weight of uav (n, k) at leaving node (i): Dict{(n, k, i): float}
    m_loaded = {}           # weight loaded for uav (n, k) at WH (i): Dict{(n, k, i): float}
                            # weight unloaded is simply the demand

    uav_deployed = []
    grid_id_unique = []

    for uav_id in range(uav_num):
        if fixed_investments["z"][cs_id, uav_id]:
            uav_deployed.append(uav_id)                 # collect uav deployed to cs_id

    H = Model.MAX_CHARGING_FREQ
    # Indexing for all UAV k
    for uav_id in uav_deployed:

            # Edge variables: E1 E2 E3
            for wh_id in range(wh_num):
                # E1: Starting CS n to WH
                x[cs_id, uav_id, f"cs_{cs_id}_0", f"wh_{wh_id}"] = model.addVar(vtype=GRB.BINARY)
                E1.append((cs_id, uav_id, f"cs_{cs_id}_0", f"wh_{wh_id}"))

                # E2: WH to all CS visits
                for j in range(cs_num):
                    for h in range(1, H-1):         # adjust to 0-index
                        x[cs_id, uav_id, f"wh_{wh_id}", f"cs_{j}_{h}"] = model.addVar(vtype=GRB.BINARY)
                        E2[h].append((cs_id, uav_id, f"wh_{wh_id}", f"cs_{j}_{h}"))

                # E3: WH to all D demand points
                for d in demands_for_n:
                    d_id = d['id']
                    x[cs_id, uav_id, f"wh_{wh_id}", f"d_{d_id}"] = model.addVar(vtype=GRB.BINARY)
                    E3.append((cs_id, uav_id, f"wh_{wh_id}", f"d_{d_id}"))
            # Edge variables: E4 E5 E6 E7
            for d1 in demands_for_n:
                d_id_1 = d1['id']
                # E4: D to D demand points to demand points
                for d2 in demands_for_n:
                    if d2 == d1:
                        continue
                    d_id_2 = d2['id']
                    x[cs_id, uav_id, f"d_{d_id_1}", f"d_{d_id_2}"] = model.addVar(vtype=GRB.BINARY)
                    E4.append((cs_id, uav_id, f"d_{d_id_1}", f"d_{d_id_2}"))


                for j in range(cs_num):
                    for h in range(1, H - 1):  # adjust to 0-index
                        # E5: D to all CS visits
                        x[cs_id, uav_id, f"d_{d_id_1}", f"cs_{j}_{h}"] = model.addVar(vtype=GRB.BINARY)
                        E5[h].append((cs_id, uav_id, f"d_{d_id_1}", f"cs_{j}_{h}"))
                        # E6: All CS visits to D
                        x[cs_id, uav_id, f"cs_{j}_{h}", f"d_{d_id_1}"] = model.addVar(vtype=GRB.BINARY)
                        E6[h].append((cs_id, uav_id, f"cs_{j}_{h}", f"d_{d_id_1}"))
                    # E7: D to final visits CS
                    x[cs_id, uav_id, f"d_{d_id_1}", f"cs_{j}_{H-1}"] = model.addVar(vtype=GRB.BINARY)
                    E7.append((cs_id, uav_id, f"d_{d_id_1}", f"cs_{j}_{H-1}"))
            # Edge variables: E8
            for j in range(cs_num):
                # E8: Starting CS n to final visits CS
                x[cs_id, uav_id, f"cs_{cs_id}_0", f"cs_{j}_{H - 1}"] = model.addVar(vtype=GRB.BINARY)
                E8.append((cs_id, uav_id, f"cs_{cs_id}_0", f"cs_{j}_{H - 1}"))

    # UAV routing constraints
    # 10a each demand are visited at least once

    for d in demands_for_n:
        id = d['id']
        E_filtered = [x for x in E3 | E4 | E6 if x[3] == f"d_{id}"]
        model.addConstr(
            gp.quicksum(x[n_iter, k_iter, i_iter, d_iter] for [n_iter, k_iter, i_iter, d_iter] in x.keys()
                        if (n_iter, k_iter, i_iter, d_iter) in E_filtered)
            >= 1
        )


    for k in uav_deployed:
        # 10b
        E_filtered = [x for x in E1 | E8 if x[1] == k]
        model.addConstr(
            gp.quicksum(x[n_iter, k_iter, i_iter, d_iter] for [n_iter, k_iter, i_iter, d_iter] in x.keys()
                        if (n_iter, k_iter, i_iter, d_iter) in E_filtered)
            == 1
        )

        







    for h in range(H):

        for j in range()

    model.setObjective()

    # UAV routing constraints

    # Time constraint

    #





