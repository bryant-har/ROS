#!/usr/bin/env python
from __future__ import division
from cmath import cos
from threading import Lock
import numpy as np
from numpy.core.numeric import roll
import rospy

from std_msgs.msg import Float64

import matplotlib.pyplot as plt


class KinematicCarMotionModel:
    """The kinematic car motion model."""

    def __init__(self, car_length, **kwargs):
        """Initialize the kinematic car motion model.

        Args:
            car_length: the length of the car
            **kwargs (object): any number of optional keyword arguments:
                vel_std (float): std dev of the control velocity noise
                alpha_std (float): std dev of the control alpha noise
                x_std (float): std dev of the x position noise
                y_std (float): std dev of the y position noise
                theta_std (float): std dev of the theta noise
        """

        defaults = {
            "vel_std": 0.1,
            "alpha_std": 0.1,
            "x_std": 0.05,
            "y_std": 0.05,
            "theta_std": 0.1,
        }
        if not set(kwargs).issubset(set(defaults)):
            raise ValueError("Invalid keyword argument provided")
        # These next two lines set the instance attributes from the defaults and
        # kwargs dictionaries. For example, the key "vel_std" becomes the
        # instance attribute self.vel_std.
        self.__dict__.update(defaults)
        self.__dict__.update(kwargs)

        if car_length <= 0.0:
            raise ValueError(
                "The model is only defined for defined for positive, non-zero car lengths"
            )
        self.car_length = car_length

    def compute_changes(self, states, controls, dt, alpha_threshold=1e-2):
        """Integrate the (deterministic) kinematic car model.

        Given vectorized states and controls, compute the changes in state when
        applying the control for duration dt.

        If the absolute value of the applied alpha is below alpha_threshold,
        round down to 0. We assume that the steering angle (and therefore the
        orientation component of state) does not change in this case.

        Args:
            states: np.array of states with shape M x 3
            controls: np.array of controls with shape M x 2
            dt (float): control duration

        Returns:
            M x 3 np.array, where the three columns are dx, dy, dtheta

        """
        # BEGIN "QUESTION 1.2" ALT="return np.zeros_like(states, dtype=float)"
        res = np.zeros(states.shape)
        res2 = np.zeros(states.shape)
        res[:, 0] = controls[:, 0] * dt*np.cos(states[:, 2])
        res[:, 1] = controls[:, 0] * dt*np.sin(states[:, 2])

        res2[:, 2] = (controls[:, 0]/self.car_length) * \
            (dt*np.tan(controls[:, 1]))
        res2[:, 0] = ((self.car_length)/np.tan(controls[:, 1])) * \
            (np.sin(states[:, 2]+res2[:, 2])-np.sin(states[:, 2]))
        res2[:, 1] = (((self.car_length)/np.tan(controls[:, 1]))
                      * (np.cos(states[:, 2])-np.cos(states[:, 2]+res2[:, 2])))

        res[:, 0] = np.where(np.abs(controls[:, 1]) <
                             alpha_threshold, res[:, 0], res2[:, 0])
        res[:, 1] = np.where(np.abs(controls[:, 1]) <
                             alpha_threshold, res[:, 1], res2[:, 1])
        res[:, 2] = np.where(np.abs(controls[:, 1]) <
                             alpha_threshold, res[:, 2], res2[:, 2])
        return res

        # END

    def apply_deterministic_motion_model(self, states, vel, alpha, dt):
        """Propagate states through the determistic kinematic car motion model.

        Given the nominal control (vel, alpha
        ), compute the changes in state 
        and update it to the resulting state.

        NOTE: This function does not have a return value: your implementation
        should modify the states argument in-place with the updated states.

        >>> states = np.ones((3, 2))
        >>> states[2, :] = np.arange(2)  #  modifies the row at index 2
        >>> a = np.array([[1, 2], [3, 4], [5, 6]])
        >>> states[:] = a + a            # modifies states; note the [:]

        Args:
            states: np.array of states with shape M x 3
            vel (float): nominal control velocity
            alpha (float): nominal control steering angle
            dt (float): control duration
        """
        n_particles = states.shape[0]

        # Hint: use same controls for all the particles
        # BEGIN SOLUTION "QUESTION 1.3"

        def reduce(theta):
            theta = theta % (2*np.pi)
            theta = (theta + 2*np.pi) % (2*np.pi)
            theta2 = theta - 2*np.pi

            return np.where(theta <= np.pi, theta, theta2)

        if abs(alpha) < 1e-2:
            states[:, 0] = states[:, 0] + vel*dt*np.cos(states[:, 2])
            states[:, 1] = states[:, 1] + vel*dt*np.sin(states[:, 2])
            states[:, 2] = states[:, 2]
        else:
            states[:, 0] = states[:, 0]+(self.car_length/np.tan(alpha)) * \
                (np.sin(states[:, 2] + (vel/self.car_length)
                 * dt*np.tan(alpha))-np.sin(states[:, 2]))
            states[:, 1] = states[:, 1] + (self.car_length/np.tan(alpha)) * \
                (np.cos(states[:, 2])-np.cos(states[:, 2] +
                 (vel/self.car_length) * dt*np.tan(alpha)))
            states[:, 2] = states[:, 2] + \
                (vel/self.car_length) * dt * np.tan(alpha)
        states[:, 2] = reduce(states[:, 2])
        return
        # END SOLUTION

    def apply_motion_model(self, states, vel, alpha, dt):
        """Propagate states through the noisy kinematic car motion model.

        Given the nominal control (vel, alpha), sample M noisy controls.
        Then, compute the changes in state with the noisy controls.
        Finally, add noise to the resulting states.

        NOTE: This function does not have a return value: your implementation
        should modify the states argument in-place with the updated states.

        >>> states = np.ones((3, 2))
        >>> states[2, :] = np.arange(2)  #  modifies the row at index 2
        >>> a = np.array([[1, 2], [3, 4], [5, 6]])
        >>> states[:] = a + a            # modifies states; note the [:]

        Args:
            states: np.array of states with shape M x 3
            vel (float): nominal control velocity
            alpha (float): nominal control steering angle
            dt (float): control duration
        """
        n_particles = states.shape[0]

        # Hint: you may find the np.random.normal function useful
        # BEGIN SOLUTION "QUESTION 1.4"

        noisycontrol = np.vstack(
            (np.random.normal(vel, self.vel_std, n_particles), np.random.normal(alpha, self.alpha_std, n_particles)))

        changes = self.compute_changes(states, noisycontrol.T, dt)
        noisychanges = changes + np.vstack(
            ((np.random.normal(0, self.x_std, n_particles)), (np.random.normal(
                0, self.y_std, n_particles)), (np.random.normal(0, self.theta_std, n_particles)))
        ).T

        states = np.array([1, 2, 3])
        return

        # END SOLUTION
