#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.problems import JointSpace


class TestCustomDistanceRRT(unittest.TestCase):
    def setUp(self):
        self.problem = JointSpace()

    def test_rrt_r2problem(self):

        rrt_planner = search.RRTPlanner(self.problem, map = None, eta=0.5)

        start = np.array([0., 0., 0., 0., 0., 0.])
        goal = np.array([-2.3472, 0.9303, -0.8133, 1.6733, 0.8535, -1.7264])
        
        correct_path = np.array([[ 0.,          0.,          0.,          0.,          0.,          0.        ],
                                [-0.01423024,  0.57786305, -0.83716813,  0.33487886,  0.0339359,  -0.82540513],
                                [-0.36960387,  0.29209684, -1.47917183,  0.36989271,  0.07601973, -0.75442503],
                                [-1.35840193,  0.61119842, -1.14623591,  1.02159635,  0.46475986, -1.24041251],
                                [-2.17866013,  0.11766704, -0.70200987,  1.44337103,  1.0177884,  -1.90267056],
                                [-2.26293007,  0.52398352, -0.75765493,  1.55833551,  0.9356442,  -1.81453528],
                                [-2.30506503,  0.72714176, -0.78547747,  1.61581776,  0.8945721,  -1.77046764],
                                [-2.3472,      0.9303,     -0.8133,      1.6733,      0.8535,     -1.7264    ]])

        path = rrt_planner.Plan(start, goal, epsilon = 0.1)

        self.assertTrue(
            np.allclose(path[0], start),
            msg="Path must begin at start",
        )

        self.assertTrue(
            np.allclose(path[-1], goal),
            msg="Path must end at goal",
        )
        
        self.assertTrue(
            np.allclose(path, correct_path),
            msg="RRT implementation is incorrect",
        )

if __name__ == "__main__":
    np.random.seed(1111)
    rosunit.unitrun("planning", "test_custom_distance_rrt", TestCustomDistanceRRT)