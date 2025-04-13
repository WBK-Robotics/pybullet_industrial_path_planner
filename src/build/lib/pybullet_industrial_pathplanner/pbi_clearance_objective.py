import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)
import sys
import copy

class PbiClearanceObjective(ob.StateCostIntegralObjective):
    """
    The cost is computed based on state clearance. Lower cost is preferable.
    States with clearance below the target incur higher cost, while states
    above target incur lower cost until a cap is reached.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:
        """
        The clearance objective is initialized with provided parameters.

        Args:
            si (PbiSpaceInformation): The robot's space information.
            importance (float): Scaling factor in [0, 1] for cost assignment
                when clearance is below target.
            target_clearance (float): Ideal clearance with minimal cost.
            max_clearance (float): Clearance above which cost is capped.
        """
        super(PbiClearanceObjective, self).__init__(si, True)
        self._si = si

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        The cost for a state is computed based on its clearance.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields a higher
            cost.
        """
        if self._si.getStateValidityChecker().clearance(state) is None:
            return ob.Cost(0)
        else:
            return ob.Cost(1 / (
                self._si.getStateValidityChecker().clearance(
                    state) + sys.float_info.min))