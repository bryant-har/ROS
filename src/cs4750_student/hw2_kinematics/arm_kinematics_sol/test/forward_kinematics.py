#!/usr/bin/env python3
from __future__ import division
import numpy as np
import rosunit
import unittest

from arm_kinematics.fk_broadcaster import Foward_Kinematics_Broadcaster
from tf.transformations import quaternion_from_matrix, translation_from_matrix

class TestForwardKinematics(unittest.TestCase):
    def test_base_2_shoulder(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 2.19
        q_shoulder = -0.96
        q_elbow = 0.86
        test_pos1 = np.array([[1],[1],[1],[1]])
        test_pos2 = np.array([[0.912],[0.6735],[0.2159],[1]])
        base_E_shoulder, _, _, _ = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos1 = np.array(base_E_shoulder * test_pos1)[:3,0]
        computed_pos2 = np.array(base_E_shoulder * test_pos2)[:3,0]
        ground_truth1 = np.array([1.394728,-0.233954, 1.111]).reshape((3,))
        ground_truth2 = np.array([ 1.077771, -0.351788,  0.3269  ]).reshape((3,))
        np.testing.assert_allclose(computed_pos1, ground_truth1, rtol=0, atol=0.01,
        err_msg="test 1a: Wrong position for base_2_shoulder")
        np.testing.assert_allclose(computed_pos2, ground_truth2, rtol=0, atol=0.01,
        err_msg="test 1b: Wrong position for base_2_shoulder")

    def test_shoulder_2_Uarm(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 2.19
        q_shoulder = -0.96
        q_elbow = 0.86
        test_pos1 = np.array([[1],[1],[1],[1]])
        test_pos2 = np.array([[0.912],[0.6735],[0.2159],[1]])
        _, shoulder_E_Uarm, _, _ = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos1 = np.array(shoulder_E_Uarm * test_pos1)[:3,0]
        computed_pos2 = np.array(shoulder_E_Uarm * test_pos2)[:3,0]
        ground_truth1 = np.array([-0.032232,-1, 1.413846]).reshape((3,))
        ground_truth2 = np.array([ 0.143049, -0.2159  ,  1.12467 ]).reshape((3,))
        np.testing.assert_allclose(computed_pos1, ground_truth1, rtol=0, atol=0.01,
        err_msg="test 2a: Wrong position for shoulder_2_Uarm")
        np.testing.assert_allclose(computed_pos2, ground_truth2, rtol=0, atol=0.01,
        err_msg="test 2b: Wrong position for shoulder_2_Uarm")

    def test_Uarm_2_Uforearm(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 2.19
        q_shoulder = -0.96
        q_elbow = 0.86
        test_pos1 = np.array([[1],[1],[1],[1]])
        test_pos2 = np.array([[0.912],[0.6735],[0.2159],[1]])
        _, _, Uarm_E_Uforearm, _ = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos1 = np.array(Uarm_E_Uforearm * test_pos1)[:3,0]
        computed_pos2 = np.array(Uarm_E_Uforearm * test_pos2)[:3,0]
        ground_truth1 = np.array([-1.148614, 0.17322, 1]).reshape((3,))
        ground_truth2 = np.array([-0.837069,  0.304696,  0.2159  ]).reshape((3,))
        np.testing.assert_allclose(computed_pos1, ground_truth1, rtol=0, atol=0.01,
        err_msg="test 3a: Wrong position for shoulder_2_Uarm")
        np.testing.assert_allclose(computed_pos2, ground_truth2, rtol=0, atol=0.01,
        err_msg="test 3b: Wrong position for shoulder_2_Uarm")

    def test_Uforearm_2_EE(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 2.19
        q_shoulder = -0.96
        q_elbow = 0.86
        test_pos1 = np.array([[1],[1],[1],[1]])
        test_pos2 = np.array([[0.912],[0.6735],[0.2159],[1]])
        _, _, _, Uforearm_E_gripper = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos1 = np.array(Uforearm_E_gripper * test_pos1)[:3,0]
        computed_pos2 = np.array(Uforearm_E_gripper * test_pos2)[:3,0]
        ground_truth1 = np.array([ 1.409, -1.   ,  1.]).reshape((3,))
        ground_truth2 = np.array([ 1.321 , -0.2159,  0.6735]).reshape((3,))
        np.testing.assert_allclose(computed_pos1, ground_truth1, rtol=0, atol=0.01,
        err_msg="test 4a: Wrong position for shoulder_2_Uarm")
        np.testing.assert_allclose(computed_pos2, ground_truth2, rtol=0, atol=0.01,
        err_msg="test 4b: Wrong position for shoulder_2_Uarm")


if __name__ == "__main__":
    rosunit.unitrun("arm_kinematics", "test_arm_kinematics", TestForwardKinematics)