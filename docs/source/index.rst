.. pybullet_industrial_path_planner documentation master file, created by
   sphinx-quickstart on Mon Jun 16 10:45:31 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to the PyBullet Industrial Path Planner documentation!
==============================================================

**PyBullet Industrial Path Planner** is a modular Python package for sampling-based path planning with industrial robots in PyBullet environments.
It integrates the Open Motion Planning Library and provides tools to define, execute, and validate motion plans under geometric constraints.

Features
--------

- **OMPL Integration**: Direct access to OMPL through a compatible interface and tailored extensions.
- **Serial-Kinematic Chains**: Builds joint-based configuration spaces with robot-specific bounds derived from URDF models.
- **Object Support**: Supports planning with attached tools and grasped external objects during configuration sampling.
- **Collision Checking**: Enables detailed management of internal, external, and explicitly allowed collision pairs.
- **Task Constraints**: Supports additional application-specific constraints through the state validity module.
- **Clearance Computation**: Calculates minimum distances to obstacles using pairwise collision checks.
- **Optimization Objectives**: Includes objectives for clearance, joint-space distance, and end-effector motion.
- **Multi-Objective Planning**: Supports weighted combinations of multiple optimization criteria.
- **Simple Setup API**: High-level interface for defining motion problems and retrieving feasible solutions.
- **Graphical User Interface**: Provides practical accessibility through visual interaction.
- **G-Code Generation**: Tools for logging or converting solution paths into deployable machine code.

Example: Comau NJ290-3.0 in a Cell Environment
----------------------------------------------

.. image:: images/comau_cell.gif
   :alt: Comau Cell Planning Demo
    :align: center


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   how_to_use
   code_docu

