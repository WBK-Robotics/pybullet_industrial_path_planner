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
        self.collision_checker = pi.CollisionChecker()
        self.collision_checker.set_safe_state()

        self.simple_setup = pbi.PbiSimpleSetup(
            robot=self.robot,
            collision_check_function=self.collision_checker.is_collision_free,
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
        Verify that plan_start_goal returns a solution and joint path.
        """
        solved, joint_path = self.simple_setup.plan_start_goal(
            self.start_config, self.goal_config,
            allowed_time=10, simplify=True
        )
        self.assertTrue(solved)
        self.assertIsNotNone(joint_path)
        self.assertEqual(joint_path.joint_order, self.joint_order)
        self.assertEqual(
            joint_path.joint_values.shape[0], len(self.joint_order)
        )

    def test_set_constraint_function(self):
        """
        Verify that the constraint function can be updated.
        """
        def always_false(): return False
        self.simple_setup.set_constraint_function(always_false)
        checker = self.simple_setup._validity_checker
        self.assertFalse(checker._constraint_function())

    def test_set_clearance_function(self):
        """
        Verify that the clearance function can be updated.
        """
        self.simple_setup.set_clearance_function(lambda: 0.42)
        val = self.simple_setup._validity_checker._clearance_function()
        self.assertEqual(val, 0.42)

    def test_set_collision_check_function(self):
        """
        Verify that the collision function can be updated.
        """
        self.simple_setup.set_collision_check_function(lambda: False)
        val = self.simple_setup._validity_checker._collision_check_function()
        self.assertFalse(val)

    def test_interpolation_precision(self):
        """
        Verify that interpolation precision can be set and accessed.
        """
        self.simple_setup.set_interpolation_precision(0.005)
        self.assertEqual(self.simple_setup._interpolation_precision, 0.005)

    def test_set_object_mover(self):
        """
        Ensure object mover can be updated and stored.
        """
        mover = pbi.PbiObjectMover()
        self.simple_setup.set_object_mover(mover)
        self.assertIs(self.simple_setup._si._object_mover, mover)

    def test_set_start_and_goal(self):
        """
        Check that OMPL internal state is updated correctly.
        """
        try:
            self.simple_setup.setStartAndGoalStates(
                self.start_config, self.goal_config)
        except Exception as e:
            self.fail(f"setStartAndGoalStates raised Exception: {e}")


if __name__ == '__main__':
    unittest.main()
