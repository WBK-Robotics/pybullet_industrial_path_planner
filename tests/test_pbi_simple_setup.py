import os
import unittest
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


class TestPbiSimpleSetup(unittest.TestCase):
    """
    Unit tests for PbiSimpleSetup integrated planning setup.
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
        self.colliision_checker = pi.CollisionChecker()
        self.colliision_checker.set_safe_state()
        self.simple_setup = pbi.PbiSimpleSetup(
            self.robot,
            collision_check_function=self.colliision_checker.is_collision_free,
            planner_type=None,
            constraint_function=lambda: True,
            clearance_function=lambda: 1.0,
            objective=None,
            object_mover=None,
            name="DummySetup"
        )
        self.joint_order, _ = self.robot.get_moveable_joints()
        self.start_config = self.robot.get_joint_position()
        self.goal_config = self.robot.get_joint_position()

    def tearDown(self) -> None:
        p.disconnect()

    def test_plan_start_goal(self):
        """
        Verify that plan_start_goal returns a solution.
        The dummy planner returns a path with length 1.0; with an
        interpolation precision of 0.01 the joint path should have
        max(2, int(1.0/0.01)) states.
        """
        solved, joint_path = self.simple_setup.plan_start_goal(
            self.start_config, self.goal_config,
            allowed_time=10, simplify=True
        )
        self.assertTrue(solved)
        self.assertIsNotNone(joint_path)
        num_joints = len(self.joint_order)
        expected_states = 2
        self.assertEqual(joint_path.joint_order, self.joint_order)
        self.assertEqual(joint_path.joint_values.shape,
                         (num_joints, expected_states))


if __name__ == '__main__':
    unittest.main()
