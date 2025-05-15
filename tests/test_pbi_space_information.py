import unittest
import os
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


class DummyObjectMover:
    """
    Dummy object mover to verify pose forwarding from SpaceInformation.
    """
    def __init__(self):
        self.received_position = None
        self.received_orientation = None

    def match_moving_objects(self, position, orientation):
        self.received_position = position
        self.received_orientation = orientation


class TestPbiSpaceInformation(unittest.TestCase):
    """
    Unit tests for the PbiSpaceInformation class.
    """

    def setUp(self):
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        working_dir = os.path.dirname(__file__)
        parent_dir = os.path.dirname(working_dir)
        robot_urdf = os.path.join(
            parent_dir, "examples", "robot_descriptions", "comau_nj290",
            "comau_nj290_robotNC.urdf"
        )
        position = np.array([0, 0, 0])
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = pi.RobotBase(robot_urdf, position, orientation)
        self.state_space = pbi.PbiStateSpace(self.robot)
        self.si = pbi.PbiSpaceInformation(self.state_space)

        # Generate a valid state for testing
        joint_values = [0.1] * self.state_space.getDimension()
        self.state = self.state_space.list_to_state(joint_values)

    def tearDown(self):
        p.disconnect()

    def test_set_state_updates_robot_joints(self):
        """
        The robot joint positions are updated when a state is set.
        """
        self.si.set_state(self.state)
        current_pos = self.robot.get_joint_position()
        expected_pos = dict(zip(self.state_space._joint_order,
                                self.state_space.state_to_list(self.state)))
        for key in expected_pos:
            self.assertAlmostEqual(current_pos[key], expected_pos[key], places=5)

    def test_set_object_mover(self):
        """
        The object mover is correctly set and stored.
        """
        mover = DummyObjectMover()
        self.si.set_object_mover(mover)
        self.assertIs(self.si._object_mover, mover)

    def test_set_state_triggers_object_mover(self):
        """
        The object mover receives the updated robot pose when set_state is called.
        """
        mover = DummyObjectMover()
        self.si.set_object_mover(mover)
        self.si.set_state(self.state)

        self.assertIsNotNone(mover.received_position)
        self.assertIsNotNone(mover.received_orientation)
        # Optional: check that values are numerically valid
        self.assertEqual(len(mover.received_position), 3)
        self.assertEqual(len(mover.received_orientation), 4)


if __name__ == '__main__':
    unittest.main()
