#!/usr/bin/env python

import numpy as np
import rosunit
import unittest

from nav_msgs.msg import MapMetaData

from planning.problems import PlanarProblem, R2Problem
from planning.samplers import LatticeSampler

class TestR2Problem(unittest.TestCase):
    def setUp(self):
        permissible_region = np.ones((10, 10), dtype=bool)
        permissible_region[3:7, 3:7] = 0
        self.problem = R2Problem(permissible_region)
        self.sampler = LatticeSampler(self.problem.extents)

    def test_r2_distance_between_states(self):
        # This sampler creates vertices for x and y = (1.67, 5.0, 8.33).
        # Only the state (5.0, 5.0) should be in collision.

        num_samples = 24
        samples = self.sampler.sample(num_samples)
        valid = self.problem.check_state_validity(samples)

        self.assertEqual(
            valid.sum(),
            20,
            msg="Only four samples should be in collision",
        )
        np.testing.assert_allclose(
            samples[~valid, :],
            np.array([[3.061862, 3.061862],[5.103104, 3.061862],[3.061862, 5.103104],[5.103104, 5.103104]]),
            err_msg="The samples in collision should be (3.061862, 3.061862), (5.103104, 3.061862), (3.061862, 5.103104), and (5.103104, 5.103104)",
        )

        samples = samples[valid, :]
        computed_distances = self.problem.distance_between_states(samples[0:10, :], samples[10:, :])

        actual_distances = np.linalg.norm(samples[0:10, :] - samples[10:, :], axis=1)
        np.testing.assert_allclose(
            computed_distances,
            actual_distances,
            err_msg="The R2 distance between two states should be the Euclidean norm",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_r2_problem", TestR2Problem)
