import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)
import sys
import copy

class PbiValidityChecker(ob.StateValidityChecker):
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
        """
        The validity checker is initialized.

        Args:
            si (PbiSpaceInformation): The space information.
            collision_check_function (callable): Function to perform collision
                checks.
            constraint_function (callable, optional): Function to check
                additional constraints.
            clearance_function (callable, optional): Function returning a
                clearance value.
        """
        super().__init__(si)
        self._si = si
        self.collision_check_function = collision_check_function
        self.constraint_function = constraint_function
        self.clearance_function = clearance_function

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
        if self.constraint_function:
            if not self.constraint_function():
                return False
        if not self.collision_check_function():
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
        if self.clearance_function is None:
            return None
        else:
            return self.clearance_function()
