from ompl import base as ob
import pybullet_industrial_path_planner as pbi


class PbiSpaceInformation(ob.SpaceInformation):
    """
    OMPL SpaceInformation is extended with robot-specific functions.
    It handles state conversion and updates the robot configuration.

    Attributes:
        robot (RobotBase): The robot instance.
        state_space (PbiStateSpace): The custom state space.
        object_mover (PbiObjectMover): Optional mover for updating objects.
    """

    def __init__(self, state_space: pbi.PbiStateSpace,
                 object_mover: pbi.PbiObjectMover) -> None:
        """
        The space information is initialized with the state space and
        object mover.

        Args:
            state_space (PbiStateSpace): The robot's state space.
            object_mover (PbiObjectMover): The object mover instance.
        """
        super().__init__(state_space)
        self._robot = state_space._robot
        self._state_space = state_space
        self._object_mover = object_mover

    def set_state(self, state: ob.State) -> None:
        """
        The robot configuration is updated based on the given state.
        Moving objects are updated if applicable.

        Args:
            state (ob.State): The state to be applied.
        """
        joint_positions = self._state_space.state_to_list(state)
        self._robot.reset_joint_position(
            dict(zip(self._state_space._joint_order, joint_positions)),
            True
        )
        if self._object_mover:
            position, orientation = self._robot.get_endeffector_pose()
            self._object_mover.match_moving_objects(position, orientation)
