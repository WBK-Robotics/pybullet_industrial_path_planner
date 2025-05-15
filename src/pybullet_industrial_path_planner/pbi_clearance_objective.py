from ompl import base as ob
import sys
from pybullet_industrial_path_planner import PbiSpaceInformation


class PbiClearanceObjective(ob.StateCostIntegralObjective):
    """
    Optimization objective based on state clearance.

    States with higher clearance from obstacles are preferred. The cost
    is inversely proportional to the clearance: lower clearance results
    in higher cost, encouraging safer paths.
    Total path cost is evaluated as the integral of state costs along the path.

    Args:
    si (PbiSpaceInformation): Space information that provides
        access to the state validity checker and robot model.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:

        super(PbiClearanceObjective, self).__init__(si, True)
        self._si = si

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        Compute the cost of a state based on its clearance.

        A higher clearance results in a lower cost. The cost is defined
        as the reciprocal of the clearance to penalize states near
        obstacles.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: Inverse clearance as cost; zero if clearance is None.
        """
        if self._si.getStateValidityChecker().clearance(state) is None:
            return ob.Cost(0)
        else:
            return ob.Cost(1 / (
                self._si.getStateValidityChecker().clearance(
                    state) + sys.float_info.min))
