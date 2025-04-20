import unittest
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial_path_planner as pbi


class TestPbiObjectMover(unittest.TestCase):
    """
    Unit tests for PbiObjectMover using real collision shapes as URDF
    identifiers.
    """
    def setUp(self) -> None:
        # Connect to PyBullet in DIRECT mode.
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Create a box collision shape.
        self.col_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        # Create two multi-body objects to serve as moving objects.
        self.body1 = p.createMultiBody(
            baseMass=1.0, baseCollisionShapeIndex=self.col_shape,
            basePosition=[0, 0, 0])
        self.body2 = p.createMultiBody(
            baseMass=1.0, baseCollisionShapeIndex=self.col_shape,
            basePosition=[1, 1, 1])
        # Instantiate the object mover.
        self.mover = pbi.PbiObjectMover()

    def tearDown(self) -> None:
        p.disconnect()

    def test_add_object_default_offsets(self):
        """
        Verify that add_object uses default offsets when none are given.
        """
        self.mover.add_object(self.body1)
        self.assertEqual(len(self.mover.moving_objects), 1)
        urdf, pos_off, ori_off = self.mover.moving_objects[0]
        self.assertEqual(urdf, self.body1)
        np.testing.assert_array_equal(pos_off, np.array([0, 0, 0]))
        np.testing.assert_array_equal(ori_off, np.array([0, 0, 0, 1]))

    def test_init_with_objects(self):
        """
        Verify that objects provided during initialization are stored.
        """
        urdfs = [self.body1, self.body2]
        pos_off = [np.array([1, 2, 3]), np.array([4, 5, 6])]
        ori_off = [np.array([0, 0, 0, 1]),
                   np.array([0, 0, 1, 0])]
        mover = pbi.PbiObjectMover(urdfs, pos_off, ori_off)
        self.assertEqual(len(mover.moving_objects), 2)
        # Check first object.
        obj1 = mover.moving_objects[0]
        self.assertEqual(obj1[0], self.body1)
        np.testing.assert_array_equal(obj1[1], np.array([1, 2, 3]))
        np.testing.assert_array_equal(obj1[2], np.array([0, 0, 0, 1]))
        # Check second object.
        obj2 = mover.moving_objects[1]
        self.assertEqual(obj2[0], self.body2)
        np.testing.assert_array_equal(obj2[1], np.array([4, 5, 6]))
        np.testing.assert_array_equal(obj2[2], np.array([0, 0, 1, 0]))

    def test_match_moving_objects(self):
        """
        Verify that match_moving_objects correctly computes new transforms
        and calls resetBasePositionAndOrientation for each object.
        """
        # Define specific offsets.
        pos_off1 = np.array([1, 0, 0])
        ori_off1 = np.array([0, 0, 0, 1])
        pos_off2 = np.array([0, 1, 0])
        ori_off2 = np.array([0, 0, 1, 0])
        self.mover.add_object(self.body1, pos_off1, ori_off1)
        self.mover.add_object(self.body2, pos_off2, ori_off2)
        # Define a test end-effector pose.
        ee_pos = [10, 10, 10]
        ee_ori = [0, 0, 0, 1]
        self.mover.match_moving_objects(ee_pos, ee_ori)
        # Get new poses using real PyBullet functions.
        new_pos1, new_ori1 = p.getBasePositionAndOrientation(self.body1)
        new_pos2, new_ori2 = p.getBasePositionAndOrientation(self.body2)
        # Compute expected transforms.
        exp1 = p.multiplyTransforms(ee_pos, ee_ori,
                                    pos_off1.tolist(),
                                    ori_off1.tolist())
        exp2 = p.multiplyTransforms(ee_pos, ee_ori,
                                    pos_off2.tolist(),
                                    ori_off2.tolist())
        np.testing.assert_array_almost_equal(
            new_pos1, np.array(exp1[0]), decimal=5)
        np.testing.assert_array_almost_equal(
            new_ori1, np.array(exp1[1]), decimal=5)
        np.testing.assert_array_almost_equal(
            new_pos2, np.array(exp2[0]), decimal=5)
        np.testing.assert_array_almost_equal(
            new_ori2, np.array(exp2[1]), decimal=5)


if __name__ == '__main__':
    unittest.main()
