import unittest
import os
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import pybullet_industrial_path_planner as pbi


class DummyPath:
    """
    A minimal dummy OMPL path implementation for testing.
    """
    def __init__(self, states, length_val):
        self._states = states
        self._length = length_val

    def length(self):
        return self._length

    def interpolate(self, num_interp):
        # For testing, simulate interpolation by duplicating the
        # first state.
        self._states = [self._states[0] for _ in range(num_interp)]

    def getStates(self):
        return self._states


class TestPbiStateSpace(unittest.TestCase):
    """
    Unit tests for PbiStateSpace using real RobotBase instances.
    """
    def setUp(self) -> None:
        # Connect to PyBullet in DIRECT mode.
        p.connect(p.DIRECT)
        # Set the additional search path for URDF files.
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Use a valid URDF file from the examples.
        working_dir = os.path.dirname(__file__)
        parent_dir = os.path.dirname(working_dir)
        robot_urdf = os.path.join(
            parent_dir, "examples", "robot_descriptions", "comau_nj290",
            "comau_nj290_robotNC.urdf"
        )
        start_pos = np.array([0, 0, 0])
        start_orient = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = pi.RobotBase(robot_urdf, start_pos, start_orient)
        # Create the state space using the real robot.
        self.state_space = pbi.PbiStateSpace(self.robot)

    def tearDown(self) -> None:
        p.disconnect()

    def test_list_to_state_and_state_to_list(self):
        """
        Verify conversion from a list of joint values to an OMPL state,
        and then back to a list.
        """
        dim = self.state_space.getDimension()
        input_values = [0.1 * (i + 1) for i in range(dim)]
        state = self.state_space.list_to_state(input_values)
        output_values = self.state_space.state_to_list(state)
        self.assertEqual(len(input_values), len(output_values))
        for expected, actual in zip(input_values, output_values):
            self.assertAlmostEqual(expected, actual, places=5)

    def test_dict_to_list(self):
        """
        Verify conversion from a dictionary of joint values to an
        ordered list.
        """
        joint_order, _ = self.robot.get_moveable_joints()
        joint_dict = {joint: idx * 0.1 for idx, joint in
                      enumerate(joint_order)}
        expected = [joint_dict[j] for j in joint_order]
        result = self.state_space.dict_to_list(joint_dict)
        self.assertEqual(expected, result)

    def test_setBounds(self):
        """
        Verify that the state space bounds correspond to the robot's
        joint limits.
        """
        bounds = self.state_space.getBounds()
        lower_limits, upper_limits = self.robot.get_joint_limits()
        joint_order, _ = self.robot.get_moveable_joints()
        for i, joint in enumerate(joint_order):
            self.assertAlmostEqual(bounds.low[i],
                                   lower_limits[joint],
                                   places=5)
            self.assertAlmostEqual(bounds.high[i],
                                   upper_limits[joint],
                                   places=5)

    def test_path_to_joint_path(self):
        """
        Verify that an OMPL path is correctly converted to a JointPath.
        """
        dim = self.state_space.getDimension()
        # Create two OMPL states and assign joint values.
        state1 = self.state_space.allocState()
        state2 = self.state_space.allocState()
        for i in range(dim):
            state1[i] = 0.1 * (i + 1)
            state2[i] = 0.4 * (i + 1)
        # Create a dummy OMPL path with a given length.
        # With length=0.3 and interpolation_precision=0.1,
        # expect max(2, int(0.3/0.1)) = 3 interpolated states.
        dummy_path = DummyPath([state1, state2], length_val=0.3)
        interpolation_precision = 0.1
        joint_path = self.state_space.path_to_joint_path(
            dummy_path, interpolation_precision)
        # Verify that the joint order is preserved.
        joint_order, _ = self.robot.get_moveable_joints()
        self.assertEqual(joint_path.joint_order, joint_order)
        expected_num_states = max(
            2, int(dummy_path.length() / interpolation_precision))
        self.assertEqual(
            joint_path.joint_values.shape,
            (len(joint_order), expected_num_states)
        )


if __name__ == '__main__':
    unittest.main()
