from ompl import base as ob
from pybullet_industrial_path_planner import PbiSpaceInformation


class PbiMultiOptimizationObjective(ob.MultiOptimizationObjective):
    """
    Optimization objective based on a weighted combination of multiple
    robot-specific cost terms.

    Args:
        si (PbiSpaceInformation): Space information including robot model.
        weighted_objective_list (list): List of (objective, weight) pairs,
            where each objective is a class and weight is a float.
    """

    def __init__(self, si: PbiSpaceInformation,
                 weighted_objective_list: list) -> None:

        super().__init__(si)
        self._si = si
        # Each objective is added with its associated weight.
        for objective, weight in weighted_objective_list:
            self.addObjective(objective(self._si), weight)
        self.lock()
