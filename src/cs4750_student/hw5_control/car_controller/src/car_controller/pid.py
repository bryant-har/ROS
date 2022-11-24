from __future__ import division
import numpy as np

from car_controller.controller import BaseController
from car_controller.controller import compute_position_in_frame


class PIDController(BaseController):
    def __init__(self, **kwargs):
        self.kp = kwargs.pop("kp")
        self.kd = kwargs.pop("kd")

        # Get the keyword args that we didn't consume with the above initialization
        super(PIDController, self).__init__(**kwargs)

    def reset_params(self, **kwargs):
        with self.state_lock:
            if not set(kwargs).issubset(set(self.__required)):
                raise ValueError(
                    "Invalid or incomplete set of keyword arguments provided", kwargs
                )
            # These next two lines set the instance attributes from the defaults and
            # kwargs dictionaries. For example, the key "kp" becomes the
            # instance attribute self.kp.
            self.__dict__.update(kwargs)

    def get_error(self, pose, reference_xytv):
        """Compute the PD error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: across-track and cross-track error
        """
        return compute_position_in_frame(pose, reference_xytv[:3])

    def get_control(self, pose, reference_xytv, error):
        """Compute the PD control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed [x, y, heading, v]
            error: error vector from get_error [e_at, e_ct]

        Returns:
            control: np.array of velocity and steering angle
                (velocity should be copied from reference velocity)
        """
        # BEGIN SOLUTION "QUESTION 2.1"
        et = pose[2] - reference_xytv[2]
        v = reference_xytv[3]
        deriv = v * np.sin(et)
        u = -(self.kp*et+self.kd * deriv)
        return np.array([v, u])
        # END SOLUTION
