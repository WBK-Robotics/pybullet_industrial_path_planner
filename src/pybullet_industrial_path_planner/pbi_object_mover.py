import numpy as np
from ompl import base as ob
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)


class PbiObjectMover:
    """
    Objects to be relocated in the PyBullet simulation are managed.
    Each object is specified by its URDF and associated position and
    orientation offsets.
    """

    def __init__(self, urdf: list = None, position_offset: list = None,
                 orientation_offset: list = None):
        """
        The object mover is initialized with optional objects.

        Args:
            urdf (list, optional): List of URDF identifiers.
            position_offset (list, optional): List of 3D position offsets.
            orientation_offset (list, optional): List of quaternion offsets.
        """
        self.moving_objects = []
        if urdf is not None:
            # Each object is created and added with its offsets.
            for urdf_item, pos, ori in zip(urdf, position_offset,
                                           orientation_offset):
                self.add_object(urdf_item, pos, ori)

    def add_object(self, urdf, position_offset=None, orientation_offset=None):
        """
        An object is added by storing its URDF and offsets.

        Args:
            urdf: The URDF identifier for the object.
            position_offset (list or np.array, optional): 3D position offset.
                Defaults to [0, 0, 0].
            orientation_offset (list or np.array, optional): Quaternion offset
                [x, y, z, w]. Defaults to [0, 0, 0, 1].
        """
        if position_offset is None:
            position_offset = np.array([0, 0, 0])
        if orientation_offset is None:
            orientation_offset = np.array([0, 0, 0, 1])
        self.moving_objects.append((urdf, position_offset, orientation_offset))

    def match_moving_objects(self, position, orientation):
        """
        The base of each moving object is aligned with the robot's end
        effector.

        For every object, the new base pose is computed by multiplying the
        robot pose with the object's offset. The object's base is then
        updated in the simulation.

        Args:
            position (list or tuple): Current end effector position.
            orientation (list or tuple): Current end effector orientation.
        """
        for urdf, pos_off, ori_off in self.moving_objects:
            new_pos, new_ori = p.multiplyTransforms(
                position, orientation, pos_off, ori_off)
            p.resetBasePositionAndOrientation(urdf, new_pos, new_ori)