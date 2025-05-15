import numpy as np
from ompl import base as ob
from pybullet_industrial import (RobotBase, JointPath)


class PbiStateSpace(ob.RealVectorStateSpace):
    """
    Custom real vector state space for robot joint configurations.

    Bounds are defined based on the robot's joint limits. The state space
    supports conversion between OMPL states and joint configurations.

    Attributes:
        robot (RobotBase): The robot instance.
        joint_order (list): Ordered list of movable joint names.
    """

    def __init__(self, robot: RobotBase) -> None:

        self._robot = robot
        self._joint_order: list = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits()
        num_dims: int = len(self._joint_order)
        super().__init__(num_dims)
        self.setBounds(lower_limit, upper_limit)

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
