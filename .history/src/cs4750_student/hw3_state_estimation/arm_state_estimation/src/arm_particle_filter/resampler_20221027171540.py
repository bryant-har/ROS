#!/usr/bin/env python
from __future__ import division

from threading import Lock

import numpy as np

# TODO: Add question number for all begin solution


class LowVarianceSampler:
    """Low-variance particle sampler."""

    def __init__(self, particles, weights, state_lock=None):
        """Initialize the particle sampler.

        Args:
            particles: the particles to update
            weights: the weights to update
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
        """
        self.particles = particles
        self.weights = weights
        self.state_lock = state_lock or Lock()
        self.n_particles = particles.shape[0]

        # You may want to cache some intermediate variables here for efficiency
        # BEGIN SOLUTION "QUESTION 1.3"
        # END SOLUTION

    def resample(self):
        """Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        Aim for an efficient O(M) implementation!
        """
        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # BEGIN SOLUTION "QUESTION 1.3"
            M = self.n_particles
            states = np.array([])
            r = np.random.random()/M
            c = self.weights[0]
            i = 0  # i think zero based?
            for m in range(1, M+1):
                u = r + (m-1)/M
                while u > c:
                    i += 1
                    c += self.weights[i]
                states = np.append(states, self.particles[i])
            self.particles = states
            self.weights += 1
        # END SOLUTION
