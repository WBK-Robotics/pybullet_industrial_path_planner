import os
import unittest
import math
import numpy as np
import pybullet as p
import pybullet_data
from ompl import base as ob
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


class TestPbiEndeffectorPathLengthObjective(unittest.TestCase):
    """
    Unit tests for PbiEndeffectorPathLengthObjective using the real
    comau_nj290 robot. This test does not override
    get_endeffector_pose but uses the forward kinematics computed by
    the simulation.
    """
    def setUp(self) -> None:
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        working_dir = os.path.dirname(__file__)
        parent_dir = os.path.dirname(working_dir)
        robot_urdf = os.path.join(
            parent_dir, "examples", "robot_descriptions", "comau_nj290",
            "comau_nj290_robotNC.urdf"
        )
        start_pos = np.array([0, 0, 0])
        start_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = pi.RobotBase(robot_urdf, start_pos, start_ori)
        self.state_space = pbi.PbiStateSpace(self.robot)
        self.si = pbi.PbiSpaceInformation(self.state_space, None)
        self.objective = pbi.PbiEndeffectorPathLengthObjective(self.si)
        # Do not modify self.robot.get_endeffector_pose.
        # The simulation will compute forward kinematics.

    def tearDown(self) -> None:
        p.disconnect()

    def test_motionCost_and_costToGo(self):
        """
        Verify that motionCost computes the SE(3) distance between the
        end-effector poses for two states and that costToGo returns the
        same value.

        The test sets two distinct joint configurations and then uses
        the robot's real forward kinematics to get end-effector poses.
        The expected cost is computed as:

            expected = sqrt(norm(pos1 - pos2)^2 + (2 * acos(dot))^2)

        where 'dot' is the absolute dot product between the two
        quaternions, clamped between -1 and 1.
        """
        # Obtain the state space dimension.
        dim = self.state_space.getDimension()
        # Choose specific joint values within the robot's limits.
        s1_vals = [0.2 * (i + 1) for i in range(dim)]
        s2_vals = [0.3 * (i + 1) for i in range(dim)]
        s1 = self.state_space.list_to_state(s1_vals)
        s2 = self.state_space.list_to_state(s2_vals)
        # Set state s1 and record the corresponding end-effector pose.
        self.si.set_state(s1)
        pos1, ori1 = self.robot.get_endeffector_pose()
        # Set state s2 and record the corresponding end-effector pose.
        self.si.set_state(s2)
        pos2, ori2 = self.robot.get_endeffector_pose()
        # Compute the Euclidean distance between positions.
        trans_diff = np.linalg.norm(np.array(pos1) - np.array(pos2))
        # Compute the rotational difference.
        dot = np.abs(np.dot(ori1, ori2))
        dot = np.clip(dot, -1.0, 1.0)
        rot_diff = 2.0 * math.acos(dot)
        expected_cost = math.sqrt(trans_diff**2 + rot_diff**2)
        # Compute costs from the objective.
        cost_motion = self.objective.motionCost(s1, s2).value()
        cost_to_go = self.objective.costToGo(s1, s2).value()
        # Assert that both computed costs match the expected value.
        self.assertAlmostEqual(cost_motion, expected_cost, places=5)
        self.assertAlmostEqual(cost_to_go, expected_cost, places=5)
        self.assertAlmostEqual(cost_motion, cost_to_go, places=5)


if __name__ == '__main__':
    unittest.main()
