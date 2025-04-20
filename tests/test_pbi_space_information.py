import os
import unittest
import numpy as np
import pybullet as p
import pybullet_data
from ompl import base as ob
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


def collision_true():
    return True


def collision_false():
    return False


def constraint_true():
    return True


def constraint_false():
    return False


def clearance_const():
    return 0.5


class TestPbiStateValidityChecker(unittest.TestCase):
    """
    Unit tests for PbiStateValidityChecker using the real robot
    from the comau_nj290 example.
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
        # Create space information without an object mover.
        self.si = pbi.PbiSpaceInformation(self.state_space, None)
        dim = self.state_space.getDimension()
        self.joint_vals = [0.1 * (i + 1) for i in range(dim)]
        self.state = self.state_space.list_to_state(self.joint_vals)

    def tearDown(self) -> None:
        p.disconnect()

    def test_isValid_all_true(self):
        """
        isValid returns True when collision and constraints pass.
        """
        checker = pbi.PbiStateValidityChecker(
            self.si,
            collision_check_function=collision_true,
            constraint_function=constraint_true,
            clearance_function=None
        )
        valid = checker.isValid(self.state)
        self.assertTrue(valid)

    def test_isValid_constraint_fail(self):
        """
        isValid returns False when constraint_function fails.
        """
        checker = pbi.PbiStateValidityChecker(
            self.si,
            collision_check_function=collision_true,
            constraint_function=constraint_false,
            clearance_function=None
        )
        valid = checker.isValid(self.state)
        self.assertFalse(valid)

    def test_isValid_collision_fail(self):
        """
        isValid returns False when collision_check_function fails.
        """
        checker = pbi.PbiStateValidityChecker(
            self.si,
            collision_check_function=collision_false,
            constraint_function=constraint_true,
            clearance_function=None
        )
        valid = checker.isValid(self.state)
        self.assertFalse(valid)

    def test_clearance(self):
        """
        clearance returns the value from clearance_function.
        """
        checker = pbi.PbiStateValidityChecker(
            self.si,
            collision_check_function=collision_true,
            constraint_function=constraint_true,
            clearance_function=clearance_const
        )
        cl = checker.clearance(self.state)
        self.assertEqual(cl, 0.5)


if __name__ == '__main__':
    unittest.main()
