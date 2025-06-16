# PyBullet Industrial Path Planner

**PyBullet Industrial Path Planner** is a modular Python package for sampling-based path planning with industrial robots in PyBullet environments. It integrates the Open Motion Planning Library and provides tools to define, execute, and validate motion plans under geometric constraints.

## Features

* **OMPL Integration**: Direct access to OMPL through a compatible interface and tailored extensions.

* **Serial-Kinematic Chains**: Builds joint-based configuration spaces with robot-specific bounds derived from URDF models.

* **Object Support**: Supports planning with attached tools and grasped external objects during configuration sampling.

* **Collision Checking**: Enables detailed management of internal, external, and explicitly allowed collision pairs.

* **Task Constraints**: Supports additional application-specific constraints through the state validity module.

* **Clearance Computation**: Calculates minimum distances to obstacles using pairwise collision checks.

* **Optimization Objectives**: Includes objectives for clearance, joint-space distance, and end-effector motion.

* **Multi-Objective Planning**: Supports weighted combinations of multiple optimization criteria.

* **Simple Setup API**: High-level interface for defining motion problems and retrieving feasible solutions.

* **Graphical User Interface**: Provides practical accessibility through visual interaction.

* **G-Code Generation**: Tools for logging or converting solution paths into deployable machine code.

### Example: Comau NJ290-3.0 in a Cell Environment
![Comau Cell Planning Demo](./docs/images/comau_cell.gif)

## Installation

### 1. Install PyBullet Industrial

Ensure that the latest version from the main branch is installed:

```bash
git clone https://github.com/WBK-Robotics/pybullet_industrial.git
cd pybullet_industrial
pip install src/
```

### 2. Install OMPL with Python bindings

To install OMPL with Python bindings, follow the official instructions at:
[https://ompl.kavrakilab.org/python.html](https://ompl.kavrakilab.org/python.html).

For easy setup, we recommend using the prebuilt Python bindings from the [OMPL development build](https://github.com/ompl/ompl/releases/tag/prerelease).

> Note: Prebuilt binaries are currently available for Linux and macOS only.
> Windows users are advised to use WSL (Windows Subsystem for Linux) to install OMPL.

### 3. Install the Path Planner Package

Clone and install the path planner package:

```bash
git clone https://github.com/WBK-Robotics/pybullet_industrial_path_planner.git
cd pybullet_industrial_path_planner
pip install src/
```

## License

MIT License

## Contributions

Issues and pull requests are welcome. If you use this package in academic work, please cite or acknowledge the source.
