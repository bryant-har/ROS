#!/usr/bin/env python3

import numpy as np
import rosunit
import time
import unittest

from introduction.listener import norm_python, norm_numpy


def gen_prim_pyth_trips():
    m = np.random.random((10000, 3))
    for i in range(m.shape[0]):
        m[i,2] = np.sum(np.abs(m[i, 0:2]))
    return m


class TestListener(unittest.TestCase):
    def test_norms_equivalent(self):
        data = np.random.random((10000, 19))
        r1 = norm_python(data)
        r2 = norm_numpy(data)
        np.testing.assert_allclose(r1, r2)

    def test_python_norm_is_manhattan(self):
        trips = np.array(list(gen_prim_pyth_trips()))
        np.testing.assert_allclose(norm_python(trips[:, :2]), trips[:, 2])


    def test_numpy_norm_is_manhattan(self):
        trips = np.array(list(gen_prim_pyth_trips()))
        np.testing.assert_allclose(norm_numpy(trips[:, :2]), trips[:, 2])

    def test_numpy_norm_faster(self):
        data = np.random.random((10000, 19))

        t1_begin = time.time()
        _ = norm_python(data)
        t1_end = time.time()

        t2_begin = time.time()
        _ = norm_numpy(data)
        t2_end = time.time()
        self.assertTrue(t1_end - t1_begin > t2_end - t2_begin)


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_listener", TestListener)
