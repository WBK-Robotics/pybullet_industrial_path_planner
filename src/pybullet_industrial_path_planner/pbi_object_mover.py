import numpy as np
from ompl import base as ob
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)


class PbiObjectMover:
    """
    Manage objects to be relocated in the PyBullet simulation.

    Objects are defined by URDF identifiers and pose offsets relative to
    a given input pose. This class enables sampling of robot configurations
    while including coupled tools and grasped objects.

    Args:
        urdf (list, optional): List of URDF identifiers.
        position_offset (list, optional): List of 3D position offsets.
        orientation_offset (list, optional): List of quaternion offsets.
    """

    def __init__(self, urdf: list = None, position_offset: list = None,
                 orientation_offset: list = None):

        self.moving_objects = []
        if urdf is not None:
            for urdf_item, pos, ori in zip(urdf, position_offset,
                                           orientation_offset):
                self.add_object(urdf_item, pos, ori)

    def add_object(self, urdf, position_offset=None, orientation_offset=None):
        """
        Add a movable object with pose offsets relative to the reference.

        Args:
            urdf: URDF identifier of the object.
            position_offset (np.array, optional): 3D position offset.
                Defaults to [0, 0, 0].
            orientation_offset (np.array, optional): Quaternion offset
                [x, y, z, w]. Defaults to [0, 0, 0, 1].
        """
        if position_offset is None:
            position_offset = np.array([0, 0, 0])
        if orientation_offset is None:
            orientation_offset = np.array([0, 0, 0, 1])
        self.moving_objects.append((urdf, position_offset, orientation_offset))

    def match_moving_objects(self, position, orientation):
        """
        Align all stored objects with a given reference pose.

        Applies each object's relative offset to the input pose to compute
        the new base pose and updates the object in the simulation.

        Args:
            position (np.array): Reference position in world coordinates.
            orientation (np.array): Reference orientation (quaternion).
        """
        for urdf, pos_off, ori_off in self.moving_objects:
            new_pos, new_ori = p.multiplyTransforms(
                position, orientation, pos_off, ori_off)
            p.resetBasePositionAndOrientation(urdf, new_pos, new_ori)