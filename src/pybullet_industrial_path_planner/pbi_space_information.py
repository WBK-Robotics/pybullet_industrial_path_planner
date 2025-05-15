from ompl import base as ob
import pybullet_industrial_path_planner as pbi


class PbiSpaceInformation(ob.SpaceInformation):
    """
    Extend OMPL SpaceInformation with robot-specific functionality.

    This includes robot configuration updates and
    interaction with moving simulation objects.

    Attributes:
        robot (RobotBase): The robot instance.
        state_space (PbiStateSpace): Custom state space implementation.
        object_mover (PbiObjectMover): Optional object mover instance.
    """

    def __init__(self, state_space: pbi.PbiStateSpace,
                 object_mover: pbi.PbiObjectMover = None) -> None:

        super().__init__(state_space)
        self._robot = state_space._robot
        self._state_space = state_space
        self._object_mover = object_mover

    def set_object_mover(self, object_mover: pbi.PbiObjectMover) -> None:
        """
        The object mover is set to update moving objects in the simulation.

        Args:
            object_mover (PbiObjectMover): The object mover instance.
        """
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
