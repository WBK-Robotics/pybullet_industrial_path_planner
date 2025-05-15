from ompl import base as ob
from ompl import geometric as og
from pybullet_industrial import RobotBase
import pybullet_industrial_path_planner as pbi

# Planner constants.
INTERPOLATION_PRECISION = 0.01
VALIDITY_RESOLUTION = 0.005


class PbiSimpleSetup(og.SimpleSetup):
    """
    Simple setup to solve and configure a path planning problem.

    The state space, space information, validity checker, optimization
    objective, and planner are configured for the given robot instance.

    Args:
        robot (RobotBase): The robot instance.
        collision_check_function (callable): Function to perform
                                            collision checks.
        planner_type (callable, optional): Callable returning a
                                            planner instance.
        constraint_function (callable, optional): Additional
                                                constraint checker.
        clearance_function (callable, optional): Function providing
                                                state clearance.
        objective (class, optional): Optimization objective class.
        object_mover (PbiObjectMover, optional): Simulation object manager.
        name (str, optional): Custom setup name.
    """

    def __init__(self, robot: RobotBase,
                 collision_check_function: callable,
                 object_mover: pbi.PbiObjectMover = None,
                 constraint_function: callable = None,
                 clearance_function: callable = None,
                 planner_type: callable = None,
                 objective: callable = None,
                 name: str = None) -> None:
        # Set the state space
        self._robot = robot
        self._state_space = pbi.PbiStateSpace(robot)

        # Set the space information
        super().__init__(pbi.PbiSpaceInformation(self._state_space,
                                                 object_mover))
        self._si = self.getSpaceInformation()

        # Set the state validity checker with the provided collision check
        self._validity_checker = pbi.PbiStateValidityChecker(
            self._si,
            collision_check_function,
            constraint_function=constraint_function,
            clearance_function=clearance_function
        )
        self.setStateValidityChecker(self._validity_checker)

        # Set planner type if provided.
        if planner_type is not None:
            if callable(planner_type):
                self.setPlanner(planner_type)
            else:
                raise TypeError("planner_type must be callable")


        self.setOptimizationObjective(objective)


        # Set interpolation precision and validity resolution.
        self.set_interpolation_precision(INTERPOLATION_PRECISION)
        self._si.setStateValidityCheckingResolution(VALIDITY_RESOLUTION)

        # Assign a custom name or a default name.
        self.name = name if name is not None else "PbiPlannerSimpleSetup"

    def set_collision_check_function(
            self,
            collision_check_function: callable) -> None:
        """
        Sets the collision check function for validity tests.

        Args:
            collision_check_function: Callable that verifies collision-free
                states.
        """
        self._validity_checker.set_collision_check_function(
            collision_check_function
        )

    def set_object_mover(self, object_mover: pbi.PbiObjectMover) -> None:
        """
        Sets the object mover for the space information.

        Args:
            object_mover: PbiObjectMover instance for managing objects.
        """
        self._si.set_object_mover(object_mover)

    def set_constraint_function(self, constraint_function: callable) -> None:
        """
        Updates the validity checker's constraint function.

        Args:
            constraint_function: Callable used to check additional constraints.
        """
        self._validity_checker.set_constraint_function(constraint_function)

    def set_clearance_function(self, clearance_function: callable) -> None:
        """
        Assigns the clearance function for state evaluation.

        Args:
            clearance_function: Callable that returns a clearance value.
        """
        self._validity_checker.set_clearance_function(clearance_function)

    def setPlanner(self, planner_type: callable) -> None:
        """
        The planner is configured.

        Args:
            planner_type: The planner class/type to be instantiated.
        """
        super().setPlanner(planner_type(self._si))

    def setOptimizationObjective(self, objective: callable) -> None:
        """
        The optimization objective for the planner is set.

        Args:
            objective (class or None): The optimization objective class.
        """
        if objective is not None:
            super().setOptimizationObjective(objective(self._si))
        else:
            super().setOptimizationObjective(
                ob.PathLengthOptimizationObjective(self._si))

    def set_interpolation_precision(self, precision: float) -> None:
        """
        The interpolation precision is set for path generation.

        Args:
            precision (float): Distance between interpolated states.
        """
        self._interpolation_precision = precision

    def setStartAndGoalStates(self, start: dict, goal: dict) -> None:
        """
        Defines start and goal states for planning.

        Args:
            start (dict): Starting joint configuration.
            goal (dict): Goal joint configuration.
        """
        # Convert start configuration from dictionary to ordered list.
        start_list = self._state_space.dict_to_list(start)

        # Convert goal configuration from dictionary to ordered list.
        goal_list = self._state_space.dict_to_list(goal)

        # Convert the ordered list to an OMPL state for start.
        start_state = self._state_space.list_to_state(start_list)

        # Convert the ordered list to an OMPL state for goal.
        goal_state = self._state_space.list_to_state(goal_list)

        # Define start and goal states in the parent class.
        super().setStartAndGoalStates(start_state, goal_state)

    def get_path_cost_value(self, path: ob.Path, objective=None):
        """
        The cost of a path is computed using the optimization objective.

        Args:
            ob.Path: The OMPL path to evaluate.

        Returns:
            float: The cost of the path.
        """
        if objective is None:
            objective = self.getOptimizationObjective()
        if objective is not None:
            cost = path.cost(
                objective)
            return cost.value()
        else:
            cost = path.length()
            return cost

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = 5.0,
                        simplify: bool = True) -> tuple:
        """
        Plans a path from start to goal configuration and returns the result
        as a Joint Path instance.

        Args:
            start (dict): Starting joint configuration.
            goal (dict): Goal joint configuration.
            allowed_time (float): Maximum planning time in seconds.
            simplify (float): Factor for solution simplification

        Returns:
            tuple: (solved (bool), JointPath or None) indicating whether
                a valid path was found and the corresponding joint path.
        """
        # Clear previous planning data.
        self.clear()
        joint_path = None
        if self.getPlanner() is not None:
            self.getPlanner().clear()

        # Define start and goal states.
        self.setStartAndGoalStates(start, goal)
        self.setup()

        # Attempt to solve the planning problem.
        solved = self.solve(allowed_time)
        joint_path = None

        # Process solution if a valid path is found.
        if solved:
            path = self.getSolutionPath()
            cost = self.get_path_cost_value(path)
            print("Path cost before simplification: ", cost)
            joint_path = self._state_space.path_to_joint_path(
                path, self._interpolation_precision)

            # Simplify the solution if requested.
            if simplify:
                self.simplifySolution(True)
                simplified_path = self.getSolutionPath()
                new_cost = self.get_path_cost_value(simplified_path)
                print("Path cost after simplification: ", new_cost)

                # Only accept the simplified path if the cost is not higher.
                if new_cost <= cost:
                    joint_path = self._state_space.path_to_joint_path(
                        simplified_path, self._interpolation_precision)
                else:
                    print("Simplified path rejected, as cost increased.")

        return solved, joint_path
