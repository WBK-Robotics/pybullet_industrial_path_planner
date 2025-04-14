import numpy as np
from ompl import base as ob
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)


class PbiObjectMover:
    """
    Objects to be relocated in the PyBullet simulation are managed.
    Each object is specified by its URDF and associated position and
    orientation offsets.
    """

    def __init__(self, urdf: list = None, position_offset: list = None,
                 orientation_offset: list = None):
        """
        The object mover is initialized with optional objects.

        Args:
            urdf (list, optional): List of URDF identifiers.
            position_offset (list, optional): List of 3D position offsets.
            orientation_offset (list, optional): List of quaternion offsets.
        """
        self.moving_objects = []
        if urdf is not None:
            # Each object is created and added with its offsets.
            for urdf_item, pos, ori in zip(urdf, position_offset,
                                           orientation_offset):
                self.add_object(urdf_item, pos, ori)

    def add_object(self, urdf, position_offset=None, orientation_offset=None):
        """
        An object is added by storing its URDF and offsets.

        Args:
            urdf: The URDF identifier for the object.
            position_offset (list or np.array, optional): 3D position offset.
                Defaults to [0, 0, 0].
            orientation_offset (list or np.array, optional): Quaternion offset
                [x, y, z, w]. Defaults to [0, 0, 0, 1].
        """
        if position_offset is None:
            position_offset = np.array([0, 0, 0])
        if orientation_offset is None:
            orientation_offset = np.array([0, 0, 0, 1])
        self.moving_objects.append((urdf, position_offset, orientation_offset))

    def match_moving_objects(self, position, orientation):
        """
        The base of each moving object is aligned with the robot's end
        effector.

        For every object, the new base pose is computed by multiplying the
        robot pose with the object's offset. The object's base is then
        updated in the simulation.

        Args:
            position (list or tuple): Current end effector position.
            orientation (list or tuple): Current end effector orientation.
        """
        for urdf, pos_off, ori_off in self.moving_objects:
            new_pos, new_ori = p.multiplyTransforms(
                position, orientation, pos_off, ori_off)
            p.resetBasePositionAndOrientation(urdf, new_pos, new_ori)


class PbiStateSpace(ob.RealVectorStateSpace):
    """
    An OMPL state space that represents the robot's joint configuration.
    The state space bounds are based on the robot's joint limits.

    Attributes:
        robot (RobotBase): The robot instance.
        joint_order (list): Ordered list of movable joints.
    """

    def __init__(self, robot: RobotBase) -> None:
        """
        The state space is initialized using the robot's joint limits.

        Args:
            robot (RobotBase): The robot instance with joint information.
        """
        self._robot = robot
        self._joint_order: list = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits()
        num_dims: int = len(self._joint_order)
        super().__init__(num_dims)
        self.setBounds(lower_limit, upper_limit)

    def list_to_state(self, joint_values: list) -> ob.State:
        """
        A list of joint values is converted to an OMPL state.

        Args:
            joint_values (list): Ordered list of joint values.

        Returns:
            ob.State: The corresponding OMPL state.
        """
        state = ob.State(self)
        for i, value in enumerate(joint_values):
            state[i] = value
        return state

    def state_to_list(self, state: ob.State) -> list:
        """
        An OMPL state is converted to a list of joint values.

        Args:
            state (ob.State): The OMPL state.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i in range(len(self._joint_order))]

    def dict_to_list(self, joint_dict: dict) -> list:
        """
        A dictionary of joint names and values is converted to an ordered
        list.

        Args:
            joint_dict (dict): Mapping from joint names to values.

        Returns:
            list: Ordered joint values as per self._joint_order.
        """
        return [joint_dict[joint] for joint in self._joint_order]

    def setBounds(self, lower_limit: list, upper_limit: list) -> None:
        """
        The state space bounds are set using the provided joint limits.

        Args:
            lower_limit (list): Lower limits for each joint.
            upper_limit (list): Upper limits for each joint.
        """
        bounds = ob.RealVectorBounds(self.getDimension())
        for i, joint in enumerate(self._joint_order):
            bounds.setLow(i, lower_limit[joint])
            bounds.setHigh(i, upper_limit[joint])
        super().setBounds(bounds)

    def path_to_joint_path(self, path: ob.Path,
                           interpolation_precision: float) -> JointPath:
        """
        Converts an OMPL path to a joint configuration path using a
        specified interpolation precision.

        Args:
            path (ob.Path): OMPL path containing states.
            interpolation_precision (float): Interpolation precision.

        Returns:
            JointPath: The resulting joint configuration path.
        """
        path_length = path.length()
        num_interp = int(path_length / interpolation_precision)
        num_interp = max(num_interp, 2)
        path.interpolate(num_interp)
        states = path.getStates()
        joint_array = np.array([self.state_to_list(st) for st in states])
        return JointPath(joint_array.transpose(),
                         tuple(self._joint_order))
