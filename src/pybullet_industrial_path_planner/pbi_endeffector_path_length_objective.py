import numpy as np
from ompl import base as ob
from pybullet_industrial import RobotBase
from pybullet_industrial_path_planner import PbiSpaceInformation


class PbiEndeffectorPathLengthObjective(ob.OptimizationObjective):
    """
    Optimization objective based on the end-effector path length
    in the robots workspace.

    The cost is computed as the spatial and angular distance between
    consecutive end-effector poses, combining translation and rotation.

    Args:
        si (PbiSpaceInformation): Space information including robot model
            and methods for state conversion and setting.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:

        super(PbiEndeffectorPathLengthObjective, self).__init__(si)
        self._si = si
        self._robot: RobotBase = si._robot
        self.setCostToGoHeuristic(ob.CostToGoHeuristic(self.costToGo))

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        Cost for an individual state.

        Always returns zero since only motion cost is considered.

        Args:
            state (ob.State): State to evaluate.

        Returns:
            ob.Cost: Always zero.
        """
        return ob.Cost(0)

    def motionCost(self, s1: ob.State, s2: ob.State) -> ob.Cost:
        """
        Compute the cost between two states based on workspace distance.

        Translation is measured as Euclidean distance, rotation as
        quaternion angular distance. Both components are combined into
        a scalar cost value.

        Args:
            s1 (ob.State): Start state.
            s2 (ob.State): Target state.

        Returns:
            ob.Cost: Combined translational and rotational motion cost.
        """
        self._si.set_state(s1)
        pos1, ori1 = self._robot.get_endeffector_pose()
        self._si.set_state(s2)
        pos2, ori2 = self._robot.get_endeffector_pose()

        # Compute Euclidean distance between translations.
        trans_diff = np.linalg.norm(pos1 - pos2)

        # Compute quaternion distance.
        dot = np.abs(np.dot(ori1, ori2))
        dot = np.clip(dot, -1.0, 1.0)
        rot_diff = 2.0 * np.arccos(dot)

        total_dist = np.sqrt(trans_diff**2 + rot_diff**2)

        return ob.Cost(total_dist)

    def costToGo(self, state: ob.State, goal: ob.Goal) -> ob.Cost:
        """
        Heuristic estimate from a given state to the goal.

        Uses the same metric as motionCost to evaluate distance
        between end-effector poses.

        Args:
            state (ob.State): Current planning state.
            goal (ob.Goal): Goal representation or concrete goal state.

        Returns:
            ob.Cost: Estimated cost-to-go.
        """
        # If the goal has a getState method, extract the state.
        if hasattr(goal, "getState"):
            goal_state = goal.getState()
        else:
            goal_state = goal
        return self.motionCost(state, goal_state)
