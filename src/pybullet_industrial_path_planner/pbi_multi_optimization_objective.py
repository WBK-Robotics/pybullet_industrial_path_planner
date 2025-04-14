from ompl import base as ob
from pybullet_industrial_path_planner import PbiSpaceInformation


class PbiMultiOptimizationObjective(ob.MultiOptimizationObjective):
    """
    Multiple robot-specific optimization objectives are combined.

    Attributes:
        si (PbiSpaceInformation): The robot's space information.
    """

    def __init__(self, si: PbiSpaceInformation,
                 weighted_objective_list: list) -> None:
        """
        The multi-objective is initialized with weighted objectives.

        Args:
            si (PbiSpaceInformation): The robot's space information.
            weighted_objective_list (list): List of (objective, weight) pairs.
        """
        super().__init__(si)
        self._si = si
        # Each objective is added with its associated weight.
        for objective, weight in weighted_objective_list:
            self.addObjective(objective(self._si), weight)
        self.lock()
