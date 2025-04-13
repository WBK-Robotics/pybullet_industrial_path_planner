import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)
import sys
import copy

# Planner constants.
INTERPOLATION_PRECISION = 0.01
VALIDITY_RESOLUTION = 0.005

class SimpleSetup(og.SimpleSetup):
    """
    All components are integrated to define and solve a robot planning
    problem. The state space, space information, validity checker,
    optimization objective, and planner are configured.
    """

    def __init__(self, robot: RobotBase,
                 collision_check_function: callable,
                 planner_type=None,
                 constraint_function=None,
                 clearance_function=None,
                 objective=None,
                 object_mover=None,
                 name: str = None) -> None:
        """
        Initializes the planning setup with robot-specific configs.

        Args:
            robot (RobotBase): The robot instance.
            collision_check_function (callable): Function to perform
                collision checks.
            planner_type (callable, optional): Callable that instantiates a
                planner given a PbiSpaceInformation instance.
            constraint_function (callable, optional): Function to check extra
                constraints.
            clearance_function (callable, optional): Function that returns a
                clearance value.
            objective (class, optional): Optimization objective class.
            object_mover (PbiObjectMover, optional): Mover for updating
                simulation objects.
            name (str, optional): Custom name for the setup.
        """
        # Store the robot instance.
        self._robot = robot

        # Initialize the state space using the robot's joint limits.
        self._state_space = PbiStateSpace(robot)
        # Create and assign the space information.
        super().__init__(PbiSpaceInformation(self._state_space, object_mover))
        self._si = self.getSpaceInformation()

        # Create and set the validity checker with collision,
        # constraint, and clearance functions.
        self._validity_checker = PbiValidityChecker(
            self._si,
            collision_check_function,
            constraint_function=constraint_function,
            clearance_function=clearance_function
        )
        self.setStateValidityChecker(self._validity_checker)

        # Configure the planner if a planner type is provided.
        if planner_type is not None:
            if callable(planner_type):
                self.setPlanner(planner_type)
            else:
                raise TypeError("planner_type must be callable")

        # Set the optimization objective if provided.
        if objective is not None:
            self.setOptimizationObjective(objective)

        # Set interpolation precision and validity resolution.
        self.set_interpolation_precision(INTERPOLATION_PRECISION)
        self._si.setStateValidityCheckingResolution(VALIDITY_RESOLUTION)

        # Assign a custom name or a default name.
        self.name = name if name is not None else "PbiPlannerSimpleSetup"

        self.clearance_objective = PbiClearanceObjective(self._si)
        self.endeffector_path_length_objective = (
            PbiEndeffectorPathLengthObjective(self._si)
        )
        self.joint_path_length_objective = ob.PathLengthOptimizationObjective(
            self._si
        )

    def set_constraint_function(self, constraint_function: callable) -> None:
        """
        Updates the validity checker's constraint function.

        Args:
            constraint_function: Callable used to check additional constraints.
        """
        self._validity_checker.constraint_function = constraint_function

    def set_clearance_function(self, clearance_function: callable) -> None:
        """
        Assigns the clearance function for state evaluation.

        Args:
            clearance_function: Callable that returns a clearance value.
        """
        self._validity_checker.clearance_function = clearance_function

    def set_collision_check_function(
            self,
            collision_check_function: callable) -> None:
        """
        Sets the collision check function for validity tests.

        Args:
            collision_check_function: Callable that verifies collision-free
                states.
        """
        self._validity_checker.collision_check_function = (
            collision_check_function
        )

    def setOptimizationObjective(self, objective: callable) -> None:
        """
        The optimization objective for the planner is set.

        Args:
            objective (class or None): The optimization objective class.
        """
        if objective is not None:
            super().setOptimizationObjective(objective(self._si))

    def setPlanner(self, planner_type: callable) -> None:
        """
        The planner is configured.

        Args:
            planner_type: The planner class/type to be instantiated.
        """
        super().setPlanner(planner_type(self._si))

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
        else:
            cost = path.length()
        return cost.value()

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = 5.0,
                        simplify: bool = True) -> tuple:
        """
        Plans a path from start to goal configuration and returns the result.

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
            print("Path cost after simplification: ", cost)
            print("Clearance Value:", self.get_path_cost_value(
                path, self.clearance_objective))
            print("Path Length Ojbective Value:",
                  self.get_path_cost_value(
                      path, self.joint_path_length_objective))
            print("Endeffector Path Length Objective Value:",
                  self.get_path_cost_value(
                      path, self.endeffector_path_length_objective))
            joint_path = self._state_space.path_to_joint_path(
                path, self._interpolation_precision)
            # Simplify the solution if requested.
            if simplify:
                self.simplifySolution(True)
                simplified_path = self.getSolutionPath()
                new_cost = self.get_path_cost_value(simplified_path)
                print("Path cost after simplification: ", new_cost)
                print("Clearance Value:", self.get_path_cost_value(
                    simplified_path, self.clearance_objective))
                print("Path Length Ojbective Value:",
                      self.get_path_cost_value(
                          simplified_path, self.joint_path_length_objective))
                print("Endeffector Path Length Objective Value:",
                      self.get_path_cost_value(
                          simplified_path, self.endeffector_path_length_objective))
                # Only accept the simplified path if the cost is not higher.
                if new_cost <= cost:
                    joint_path = self._state_space.path_to_joint_path(
                        simplified_path, self._interpolation_precision)
                else:
                    print("Simplified path rejected, as cost increased.")

        # Return the planning outcome and joint path.
        return solved, joint_path