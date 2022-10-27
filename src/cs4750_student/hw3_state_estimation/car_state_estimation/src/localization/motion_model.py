#!/usr/bin/env python
from __future__ import division
from threading import Lock
import numpy as np
import rospy

from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from car_kinematics.kinematic_model import KinematicCarMotionModel


class KinematicCarMotionModelROS:
    """A ROS subscriber that applies the kinematic car motion model.

    This applies the motion model to the particles whenever it receives a
    message from the control topic. Each particle represents a state (pose).

    These implementation details can be safely ignored, although you're welcome
    to continue reading to better understand how the entire state estimation
    pipeline is connected.
    """

    def __init__(self, particles, noise_params=None, state_lock=None, **kwargs):
        """Initialize the kinematic car model ROS subscriber.

        Args:
            particles: the particles to update in-place
            noise_params: a dictionary of parameters for the motion model
            state_lock: guarding access to the particles during update,
                since it is shared with other processes
            **kwargs: must include
                motor_state_topic (str):
                servo_state_topic (str):
                speed_to_erpm_offset (str): Offset conversion param from rpm to speed
                speed_to_erpm_gain (float): Gain conversion param from rpm to speed
                steering_to_servo_offset (float): Offset conversion param from servo position to steering angle
                steering_to_servo_gain (float): Gain conversion param from servo position to steering angle
                car_length (float)
        """
        self.particles = particles
        required_keyword_args = {
            "speed_to_erpm_offset",
            "speed_to_erpm_gain",
            "steering_to_servo_offset",
            "steering_to_servo_gain",
            "car_length",
        }
        if not required_keyword_args.issubset(set(kwargs)):
            raise ValueError("Missing required keyword argument")
        # This sets the instance attributes from the kwargs dictionary.
        self.__dict__.update(kwargs)

        self.state_lock = state_lock or Lock()
        noise_params = {} if noise_params is None else noise_params
        self.motion_model = KinematicCarMotionModel(self.car_length, **noise_params)

        self.last_servo_cmd = None
        self.last_vesc_stamp = None

        self.servo_subscriber = rospy.Subscriber(
            "servo_state", Float64, self.servo_callback, queue_size=1
        )

        self.motion_subscriber = rospy.Subscriber(
            "vesc/sensors/core", VescStateStamped, self.motion_callback, queue_size=1
        )

        self.initialized = False

    def start(self):
        self.initialized = True

    def servo_callback(self, msg):
        """Cache the most recent servo position command.

        This command is used by motion_callback to compute the steering angle.

        Args:
            msg: a std_msgs/Float64 servo message
        """
        self.last_servo_cmd = msg.data

    def motion_callback(self, msg):
        """Apply the motion model to self.particles.

        Convert raw VESC message to vel and delta controls.

        Args:
            msg: a vesc_msgs/VescStateStamped message
        """
        if self.last_servo_cmd is None:
            # We haven't received any servo command, can't apply motion model
            rospy.logwarn_throttle(5, "No servo command received")
            return

        if self.last_vesc_stamp is None:
            rospy.loginfo("Motion information received for the first time")
            self.last_vesc_stamp = msg.header.stamp
            return

        if not self.initialized:
            return
        # Convert raw msgs to controls
        # Note that control = (raw_msg_val - offset_param) / gain_param
        curr_speed = (
            msg.state.speed - self.speed_to_erpm_offset
        ) / self.speed_to_erpm_gain

        curr_steering_angle = (
            self.last_servo_cmd - self.steering_to_servo_offset
        ) / self.steering_to_servo_gain

        dt = (msg.header.stamp - self.last_vesc_stamp).to_sec()

        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # Propagate particles with the motion model
            self.motion_model.apply_motion_model(
                self.particles, curr_speed, curr_steering_angle, dt
            )

        self.last_vesc_stamp = msg.header.stamp
