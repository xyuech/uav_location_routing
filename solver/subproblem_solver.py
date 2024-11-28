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

from ..util import util as ut


def solver_uav_routing_subproblem(cs_index: int, uav_num: int, scenario_idx: int,
                                  fixed_investments: Dict,

                                  ):
    """
    Solve UAV routing sub problems

    Args:
        cs_index: Index of the charging station (n) for this subproblem
        uav_num: Number of UAV assigned to station n (k) - predetermined.
        scenario_idx: Index of the representative scenarios (o - omega) for this subproblem

        wh_locations: Coordinates of all warehouses in List[(x, y)]
        cs_locations: Coordinates of all charging stations in List[(x, y)]

        scenarios_set: The set of representative scenarios in List[scenario]

        fixed_investments: Investment variables
            Dict of {
            "y": Binary list to denote constructed charging stations,
            "z": Binary rjlkkjjn
            }
    Returns:
        sol: Dict contains solution information
    """
    subproblem = gp.Model(f"Subproblemn_cs {cs_index}_scenario {scenario_idx}")

