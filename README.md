# PyBullet Industrial Path Planner

**PyBullet Industrial Path Planner** is a modular Python package for sampling-based path planning with industrial robots in PyBullet environments. It integrates OMPL planners and provides tools to define, execute, and validate motion plans under geometric constraints.

## Features

* **OMPL Integration**: Supports standard OMPL planners such as RRT\*, Informed RRT\*.
* **Custom State Space**: Joint-based space with robot-specific bounds derived from URDF.
* **Collision Checking**: Uses PyBullet's contact engine to detect collisions.
* **Constraint Evaluation**: Optional application-specific constraints.
* **Clearance Computation**: Computes minimum distance to obstacles.
* **Optimization Objectives**: Includes clearance, joint length, and end-effector path length.
* **Multi-Objective Planning**: Weighted combination of objectives.
* **Simple Setup API**: High-level interface to define problems and retrieve paths.
* **Object Support**: Simulate grasped tools or external objects.
* **G-code Generation**: Tools for structured output and code export.

## Installation

```bash
pip install pybullet
# OMPL must be built separately with Python bindings
git clone https://github.com/your-org/pybullet_industrial_path_planner
cd pybullet_industrial_path_planner
# Optional setup script here
```

## Dependencies

* [PyBullet](https://github.com/bulletphysics/bullet3)
* [OMPL](https://ompl.kavrakilab.org/)
* [NumPy](https://numpy.org/)
* [PyBullet Industrial](https://github.com/wbk-path/pybullet_industrial)

## Example Usage

```python
from pybullet_industrial_path_planner import PbiSimpleSetup
from pybullet_industrial import RobotBase, CollisionChecker

# Initialize PyBullet, robot, and checker
robot = RobotBase("robot.urdf", position=[0,0,0])
checker = CollisionChecker()
checker.set_safe_state()

# Create planning setup
setup = PbiSimpleSetup(
    robot,
    collision_check_function=checker.is_collision_free
)

# Define joint configs
goal = robot.get_joint_position()
start = robot.get_joint_position()

# Solve
solved, path = setup.plan_start_goal(start, goal)
if solved:
    path.plot()
```

## Folder Structure

```
pybullet_industrial_path_planner/
├── pbi_state_space.py
├── pbi_space_information.py
├── pbi_validity_checker.py
├── pbi_object_mover.py
├── pbi_simple_setup.py
├── objectives/
│   ├── pbi_clearance_objective.py
│   ├── pbi_endeffector_length_objective.py
│   └── ...
├── tests/
├── examples/
├── docs/
```

## Academic Origin

This package was developed in the context of a Master's thesis at KIT. It is intended for research and prototyping in industrial robotic motion planning.

## License

MIT License

## Contributions

Issues and pull requests are welcome. If you use this package in academic work, please cite or acknowledge the source.
