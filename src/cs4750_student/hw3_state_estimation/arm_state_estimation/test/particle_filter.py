#!/usr/bin/env python
from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped

from cs4750 import utils
from cs4750.collector import SynchronizedMessageCollector
from arm_particle_filter.particle_filter import ParticleFilter


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        pass

    def test_predict_function(self):
        pf = ParticleFilter(
            np.array([0, 0]),
            np.diag([0, 0]),
            1, 0
        )
        pf.predict(np.array([1, 2]))
        self.assertEqual(pf.particles[0][0], 0.1, "Wrong prediction for x-coordinate.")
        self.assertEqual(pf.particles[0][1], 0.2, "Wrong prediction for y-coordinate.")

    def test_predict_noise_function(self):
        pf = ParticleFilter(
            np.array([0, 0]),
            np.diag([0, 0]),
            1, 1
        )
        pf.predict(np.array([1, 2]))
        try:
            self.assertEqual(pf.particles[0][0], 1.6792128155073915,
                "Wrong prediction for x-coordinate. "
                "Make sure to add noise to the predict method using std deviation std_u.")
            self.assertEqual(pf.particles[0][1], 0.9674347291529088,
                "Wrong prediction for y-coordinate. "
                "Make sure to add noise to the predict method using std deviation std_u.")
        except AssertionError:
            self.assertEqual(pf.particles[0][0], 0.2579212815507392,
                "Wrong prediction for x-coordinate. "
                "Make sure to add noise to the predict method using std deviation std_u.")
            self.assertEqual(pf.particles[0][1], 0.2767434729152909,
                "Wrong prediction for y-coordinate. "
                "Make sure to add noise to the predict method using std deviation std_u.")

    def test_update_function(self):
        pf = ParticleFilter(
            np.array([50, 50]),
            np.diag([100, 100]),
            20, 1
        )
        mean, cov = pf.update(np.array([100, 200]))
        mean, cov = mean.astype(int), cov.astype(int)
        try:
            np.testing.assert_equal(
                mean,
                np.array([[46], [52]]), # 46, 47
                err_msg="Mean after update is incorrect.",
            )
        except AssertionError:
            np.testing.assert_equal(
                mean,
                np.array([[46], [47]]),
                err_msg="Mean after update is incorrect.",
            )

        try:
            np.testing.assert_equal(
                cov,
                np.array([[62, -18], [-18, 125]]),
                err_msg="Cov after update is incorrect.",
            )
        except AssertionError:
            np.testing.assert_equal(
                cov,
                np.array([[62, -19], [-19, 91]]),
                err_msg="Cov after update is incorrect.",
            )

if __name__ == "__main__":
    rospy.init_node("test_particle_filter")
    # The xml report will use the method's name, so we have to manually
    # mangle it with the bag name to ensure all entries appears
    rostest.rosrun(
        "arm_particle_filter", "test_particle_filter", TestParticleFilter
    )
