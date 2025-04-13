import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)
import sys
import copy

class PbiEndeffectorPathLengthObjective(ob.OptimizationObjective):
    """
    The cost is computed based on the length of the end-effector path.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:
        """
        The end-effector path length objective is initialized.

        Args:
            si (PbiSpaceInformation): The robot's space information.
        """
        super(PbiEndeffectorPathLengthObjective, self).__init__(si)
        self._si = si
        self.setCostToGoHeuristic(ob.CostToGoHeuristic(self.costToGo))

    def stateCost(self, state: ob.State) -> ob.Cost:
        return ob.Cost(0)

    def motionCost(self, s1: ob.State, s2: ob.State) -> ob.Cost:
        """
        The cost for a motion between two states is computed based on
        the SE(3) distance between the end effector poses obtained from
        the states.

        Args:
            s1 (ob.State): Starting state.
            s2 (ob.State): Ending state.

        Returns:
            ob.Cost: The computed cost for the motion.
        """

        self._si.set_state(s1)
        pos1, ori1 = self._si._robot.get_endeffector_pose()
        self._si.set_state(s2)
        pos2, ori2 = self._si._robot.get_endeffector_pose()

        # Compute Euclidean distance between translations.
        trans_diff = np.linalg.norm(pos1 - pos2)

        # Compute rotational difference between quaternions.
        # Quaternions are assumed to be normalized np arrays [x, y, z, w].
        dot = np.abs(np.dot(ori1, ori2))
        dot = np.clip(dot, -1.0, 1.0)
        rot_diff = 2.0 * np.arccos(dot)

        total_dist = np.sqrt(trans_diff**2 + rot_diff**2)
        return ob.Cost(total_dist)

    def costToGo(self, state: ob.State, goal: ob.Goal) -> ob.Cost:
        """
        Heuristic cost from 'state' to the goal based on the SE(3) distance
        between the end-effector pose of the given state and the goal state.
        If the goal is an instance of ob.GoalState, its representative state
        is used.

        Args:
            state (ob.State): The current state.
            goal (ob.Goal): The goal or goal state.

        Returns:
            ob.Cost: The heuristic cost estimate.
        """
        # If the goal has a getState method, extract the state.
        if hasattr(goal, "getState"):
            goal_state = goal.getState()
        else:
            goal_state = goal
        return self.motionCost(state, goal_state)