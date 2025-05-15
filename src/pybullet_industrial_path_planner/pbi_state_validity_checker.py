from ompl import base as ob
from pybullet_industrial_path_planner import PbiSpaceInformation


class PbiStateValidityChecker(ob.StateValidityChecker):
    """
    A state is validated by updating the robot configuration and performing
    collision and constraint tests.

    Attributes:
        si (PbiSpaceInformation): The robot's space information.
        collision_check_function (callable): Returns True if the state is
            collision free.
        constraint_function (callable): Returns True if constraints are met.
        clearance_function (callable): Returns a clearance value.
    """

    def __init__(self, si: PbiSpaceInformation,
                 collision_check_function,
                 constraint_function=None,
                 clearance_function=None) -> None:

        super().__init__(si)
        self._si = si
        self._collision_check_function = collision_check_function
        self._constraint_function = constraint_function
        self._clearance_function = clearance_function

    def isValid(self, state: ob.State) -> bool:
        """
        State validity is determined by applying clearance, constraints,
        and collision tests.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if the state is valid; False otherwise.
        """
        # Clearance is computed even if not used directly.
        self.clearance(state)
        if self._constraint_function:
            if not self._constraint_function():
                return False
        if not self._collision_check_function():
            return False
        return True

    def clearance(self, state: ob.State):
        """
        Clearance is computed for a state via collision tests.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields
            a higher cost.
        """
        self._si.set_state(state)
        if self._clearance_function is None:
            return None
        else:
            return self._clearance_function()

    def set_collision_check_function(self, collision_check_function):
        """
        Set the function to check for collisions.

        Args:
            collision_check_function (callable): The function to check
                for collisions.
        """
        self._collision_check_function = collision_check_function

    def set_constraint_function(self, constraint_function):
        """
        Set the function to check for constraints.

        Args:
            constraint_function (callable): The function to check
                for constraints.
        """
        self._constraint_function = constraint_function

    def set_clearance_function(self, clearance_function):
        """
        Set the function to compute clearance.

        Args:
            clearance_function (callable): The function to compute
                clearance.
        """
        self._clearance_function = clearance_function
