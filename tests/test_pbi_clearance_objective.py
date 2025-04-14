import unittest
import sys
from ompl import base as ob
from ompl import geometric as og
import pybullet_industrial_path_planner as pbi


class DummyValidityChecker(ob.StateValidityChecker):
    """
    Dummy validity checker returning a preset clearance value.
    """
    def __init__(self, si, clearance_value: float = 1.0):
        super().__init__(si)
        self.clearance_value = clearance_value

    def isValid(self, state: ob.State) -> bool:
        return True

    def clearance(self, state: ob.State) -> float:
        self.isValid(state)
        return self.clearance_value


class TestPbiClearanceObjective(unittest.TestCase):
    """
    Unit tests for the new version of PbiClearanceObjective.
    """
    def setUp(self) -> None:
        space = ob.RealVectorStateSpace(1)
        simple_setup = og.SimpleSetup(space)
        self.si = simple_setup.getSpaceInformation()
        dummy_checker = DummyValidityChecker(self.si, 1.0)
        simple_setup.setStateValidityChecker(dummy_checker)
        self.state = ob.State(space)
        self.state[0] = 0.0

        # Initialize the clearance objective.
        self.clearance_obj = pbi.PbiClearanceObjective(self.si)

    def test_clearance_negative(self):
        """
        Test cost computation for a negative clearance.
        """
        self.si.getStateValidityChecker().clearance_value = -0.1
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0 / (-0.1)
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_positive(self):
        """
        Test cost computation for a clearance above unity.
        """
        self.si.getStateValidityChecker().clearance_value = 2.5
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0 / 2.5
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_zero(self):
        """
        Test cost computation for a clearance value below unity.
        """
        self.si.getStateValidityChecker().clearance_value = 0
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0 / sys.float_info.min
        self.assertAlmostEqual(cost.value(), expected, places=5)


if __name__ == '__main__':
    unittest.main()
