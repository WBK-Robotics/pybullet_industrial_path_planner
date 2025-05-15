import unittest
import sys
from ompl import base as ob
from ompl import geometric as og
import pybullet_industrial_path_planner as pbi


class DummyValidityChecker(ob.StateValidityChecker):
    """
    Dummy state validity checker for testing clearance-based objectives.

    Always returns a valid state, and provides a fixed clearance value
    to simulate proximity to obstacles.
    """
    def __init__(self, si, clearance_value: float = 1.0):
        super().__init__(si)
        self.clearance_value = clearance_value

    def isValid(self, state: ob.State) -> bool:
        # Always return True for testing purposes
        return True

    def clearance(self, state: ob.State) -> float:
        # Return predefined clearance value for consistent testing
        self.isValid(state)
        return self.clearance_value


class TestPbiClearanceObjective(unittest.TestCase):
    """
    Unit tests for PbiClearanceObjective using a dummy validity checker.

    Tests evaluate cost computation based on the inverse of clearance.
    """

    def setUp(self) -> None:
        """
        Setup test environment with a 1D state space and dummy checker.
        """
        space = ob.RealVectorStateSpace(1)
        simple_setup = og.SimpleSetup(space)
        self.si = simple_setup.getSpaceInformation()
        dummy_checker = DummyValidityChecker(self.si, 1.0)
        simple_setup.setStateValidityChecker(dummy_checker)
        self.state = ob.State(space)
        self.state[0] = 0.0

        # Initialize the clearance-based optimization objective
        self.clearance_obj = pbi.PbiClearanceObjective(self.si)

    def test_clearance_positive(self):
        """
        Test cost computation for a positive clearance value.

        Verifies that the cost equals the reciprocal of the clearance.
        """
        self.si.getStateValidityChecker().clearance_value = 2.5
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0 / 2.5
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_zero(self):
        """
        Test cost computation for zero clearance.

        Verifies that the cost returns a large finite value instead of
        division by zero.
        """
        self.si.getStateValidityChecker().clearance_value = 0
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0 / sys.float_info.min
        self.assertAlmostEqual(cost.value(), expected, places=5)


if __name__ == '__main__':
    unittest.main()
