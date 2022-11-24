#!/usr/bin/env python
from __future__ import division
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
        changes = np.empty_like(states, dtype=float)
        vel, alpha = controls[:, 0], controls[:, 1]
        theta = states[:, 2]

        changes[:, 2] = (vel / self.car_length) * np.tan(alpha) * dt
        changes[np.abs(alpha) < alpha_threshold, 2] = 0
        dtheta = changes[:, 2]
        changes[:, 0] = vel * np.cos(theta) * dt
        changes[:, 1] = vel * np.sin(theta) * dt

        val_indices = np.abs(alpha) >= alpha_threshold
        val_theta = theta[val_indices]
        val_new_theta = val_theta + dtheta[val_indices]
        share_value = self.car_length / np.tan(alpha[val_indices])
        changes[val_indices, 0] = share_value * (
            np.sin(val_new_theta) - np.sin(val_theta)
        )
        changes[val_indices, 1] = share_value * (
            np.cos(val_theta) - np.cos(val_new_theta)
        )
        return changes
        # END SOLUTION

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
        controls = np.repeat([[vel, alpha]], n_particles, axis=0)
        changes = self.compute_changes(states, controls, dt)

        # Apply the change in-place (numpy += is in-place)
        states += changes

        # First ,normalize to [-pi, pi)
        states[:, 2] = (states[:, 2] + np.pi) % (2 * np.pi) - np.pi
        # Now fix to (-pi, pi]
        states[states[:, 2] == -np.pi, 2] = np.pi
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
        controls = np.random.normal(
            np.array([vel, alpha]),
            scale=[self.vel_std, self.alpha_std],
            size=(n_particles, 2),
        )
        changes = self.compute_changes(states, controls, dt)
        changes = np.random.normal(
            changes, scale=(self.x_std, self.y_std, self.theta_std)
        )

        # Apply the change in-place (numpy += is in-place)
        states += changes

        # First ,normalize to [-pi, pi)
        states[:, 2] = (states[:, 2] + np.pi) % (2 * np.pi) - np.pi
        # Now fix to (-pi, pi]
        states[states[:, 2] == -np.pi, 2] = np.pi
        # END SOLUTION
