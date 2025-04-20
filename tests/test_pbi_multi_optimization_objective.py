import os
import unittest
import numpy as np
import pybullet as p
import pybullet_data
from ompl import base as ob
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


class DummyObjective1(ob.OptimizationObjective):
    def __init__(self, si):
        super(DummyObjective1, self).__init__(si)

    def stateCost(self, state: ob.State) -> ob.Cost:
        return ob.Cost(1.0)

    def motionCost(self, s1: ob.State, s2: ob.State) -> ob.Cost:
        return ob.Cost(1.0)


class DummyObjective2(ob.OptimizationObjective):
    def __init__(self, si):
        super(DummyObjective2, self).__init__(si)

    def stateCost(self, state: ob.State) -> ob.Cost:
        return ob.Cost(2.0)

    def motionCost(self, s1: ob.State, s2: ob.State) -> ob.Cost:
        return ob.Cost(2.0)


class TestPbiMultiOptimizationObjective(unittest.TestCase):
    """
    Unit tests for PbiMultiOptimizationObjective.
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
        # Weighted objectives: DummyObjective1 with weight 0.5 and
        # DummyObjective2 with weight 2.0.
        weighted_obj_list = [(DummyObjective1, 0.5),
                             (DummyObjective2, 2.0)]
        self.multi_obj = pbi.PbiMultiOptimizationObjective(
            self.si, weighted_obj_list)
        self.state = self.state_space.allocState()
        # For simplicity, set all joint values to 0.
        for i in range(self.state_space.getDimension()):
            self.state[i] = 0.0

    def tearDown(self) -> None:
        p.disconnect()

    def test_multi_objective_motion_cost(self):
        """
        Verify that motionCost returns the weighted sum of the dummy
        objectives. The expected cost is:
        0.5*1.0 + 2.0*2.0 = 4.5.
        """
        cost = self.multi_obj.motionCost(self.state, self.state)
        self.assertAlmostEqual(cost.value(), 4.5, places=5)


if __name__ == '__main__':
    unittest.main()
