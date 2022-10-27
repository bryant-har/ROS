#!/usr/bin/python

import random

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

random.seed(42)
np.random.seed(42)


class Env:
    def __init__(self):
        rospy.loginfo("Initialized Env Class")
        self.robot = InterbotixManipulatorXS("wx250", "arm", "gripper")
        self.cylinder_pub = rospy.Publisher('/cylinder_mover', Marker, queue_size=10)
        self.estimated_pose_sub = rospy.Subscriber(
            '/estimated_point', PoseStamped, self.callback)  # check name by rostopic list
        self.prev_pose = np.array([0., 0.])

        # cylinder params to follow
        self.rate = rospy.Rate(10)  # 10hz
        start_angle = 3 * np.pi / 2
        end_angle = np.pi / 2
        theta_start = np.linspace(start_angle, 6.28319, 100)
        theta_end = np.linspace(0, end_angle, 100)
        theta = np.concatenate((theta_start, theta_end))
        radius = 0.5
        self.coord_x = radius * np.cos(theta)
        self.coord_y = radius * np.sin(theta) + 150
        self.idx = 0
        cm = np.array([[500.0, 0.0, 320, 0.0], [
                      0.0, 500, 240, 0.0], [0.0, 0.0, 1.0, 0.0]])
        self.cm_inv = np.linalg.pinv(cm)

    def publish_cylinder(self):
        while not rospy.is_shutdown():
            marker = Marker()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "world"
            marker.header = header
            marker.type = Marker.CYLINDER
            marker.pose = Pose()
            marker.pose.position.x = self.coord_x[self.idx]
            marker.pose.position.y = self.coord_y[self.idx]
            self.idx += 1
            if self.idx == len(self.coord_x):
                rospy.signal_shutdown("Visualization complete!")

            marker.pose.orientation.w = 1.
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 255.0
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            self.cylinder_pub.publish(marker)
            self.rate.sleep()

    def convert_pixel_to_pos(self, pixel_x, pixel_y):
        p = np.array([pixel_x, pixel_y, 1])
        real = np.dot(self.cm_inv, p)
        x = -1.9 * real[0]
        y = 1.9 * (real[1])
        signx = np.sign(x)
        signy = np.sign(y)
        x = min(0.44, np.abs(x))
        y = min(0.44, np.abs(y))
        x = signx * x
        y = signy * y

        return x, y

    def callback(self, data):
        msg_x, msg_y = data.pose.position.x, data.pose.position.y
        x, y = self.convert_pixel_to_pos(msg_x, msg_y)
        self.move_arm(x, y)

    def move_arm(self, x, y):
        current_pos = np.array([x, y])
        if np.linalg.norm(self.prev_pose - current_pos) > 0.15:
            self.prev_pose = current_pos
            self.robot.arm.set_ee_pose_components(
                x=x, y=y, z=0.5, roll=0.0, pitch=0.0)
