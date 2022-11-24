#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import os
import sys
import time
import rospy
import rospkg
import numpy as np

from math import pi
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from planning.roadmap import Roadmap
from moveit_msgs.msg import RobotState
from planning.search import RRTPlanner
from planning.problems import JointSpace
from visualization_msgs.msg import Marker
from arm_final_project.init_env import Env
from planning.samplers import ArmRandomSampler
from geometry_msgs.msg import  Pose, PoseStamped
from arm_controller.pid_controller import PIDController
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK


from planning.search import ASTARPlanner
from planning.problems import JointSpace
from planning.samplers import ArmRandomSampler
from planning.roadmap import Roadmap


class Widowx250ArmPlanning(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(Widowx250ArmPlanning, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('WX250s', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("widowx250_manipulator")
        self.display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.estimated_pose_sub = rospy.Subscriber('/estimated_point', PoseStamped, self.callback) # check name by rostopic list

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.package_location_y = np.zeros(10)
        self.cube_pub = rospy.Publisher('/cube_mover', Marker, queue_size=10)
        self.count = 0
        self.AIRPORT_POS_DICT = {
            'pickup_tray': [0.35, 0.12, 0.18, 0.71, 0, 0, 0.71],
            'dropoff_tray': [0, -0.37, 0.3626, -0.5, 0.5, 0.5, -0.5],
            'pickup_can': [0.564, 0, 0.17, 0.0, 0.0, 0.0, 1.0],
            'dropoff_can': [0.0, 0.42, 0.23, 0, 0, 0.71, 0.71],
            'pickup_wallet': [0.565, 0, 0.19, 0., 0., 0., 1.],
            'dropoff_wallet': [0.0, -0.431, 0.45, 0, 0, -0.695, 0.72]
        }   

        self.tray_pub = rospy.Publisher('/tray_pub', Marker, queue_size=10)
        np.random.seed(1111)

    def publish_tray_marker(self):
        marker = Marker()
        marker.pose = Pose()
        marker.header.frame_id = "wx250s/base_link"
        marker.type = Marker.MESH_RESOURCE   
        marker.pose.position.x = 0.0
        marker.pose.position.y = -0.50
        marker.pose.position.z = 0.36
        marker.scale.x = 0.008
        marker.scale.y = 0.01
        marker.scale.z = 0.005
        marker.color.r = 200
        marker.color.g = 200
        marker.color.b = 200
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.707
        marker.pose.orientation.w = 0.707
        marker.mesh_resource = "package://arm_final_project/scene/tray.dae"
        self.tray_pub.publish(marker)

    def publish_dummy_marker(self):
        marker = Marker()
        marker.header.frame_id = "wx250s/base_link"
        marker.type = Marker.CUBE
        marker.pose = Pose()
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        self.cube_pub.publish(marker)

    def wait_for_state_update(self, shape="box", box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        if shape == "tray": 
            target_name = self.tray_name
        elif shape == "box":
            target_name = self.box_name
        else:
            target_name = self.cylinder_name
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([target_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = target_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_cylinder(self, x, y, z, timeout=4):
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "wx250s/base_link"
        cylinder_pose.pose.orientation.x = 0.0
        cylinder_pose.pose.orientation.y = 0.0
        cylinder_pose.pose.orientation.z = 0.0
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.z = z
        cylinder_pose.pose.position.y = y
        cylinder_pose.pose.position.x = x
        self.cylinder_name = "cylinder"
        self.scene.add_cylinder(self.cylinder_name, cylinder_pose, height=0.035, radius=0.013)
        return self.wait_for_state_update(shape="cylinder", box_is_known=True, timeout=timeout)

    def attach_cylinder(self, shape="cylinder", timeout=4):
        self.scene.attach_box("wx250s/ee_gripper_link", self.cylinder_name, touch_links="wx250s/ee_gripper_link")
        wait = self.wait_for_state_update(shape="cylinder", box_is_attached=True, box_is_known=False, timeout=timeout)
        # self.publish_dummy_marker()
        return wait

    def add_box(self, x, y, z, shape = "box", timeout=4):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "wx250s/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = z
        box_pose.pose.position.y = y
        box_pose.pose.position.x = x
        if shape == "tray":
            self.tray_name = "tray"
            self.scene.add_box(self.tray_name, box_pose, size=(0.10, 0.10, 0.005))
            return self.wait_for_state_update(shape="tray", box_is_known=True, timeout=timeout)
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.03, 0.01, 0.05))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, shape="box", timeout=4):
        if shape == "tray":
            self.scene.attach_box("wx250s/ee_gripper_link", self.tray_name, touch_links="wx250s/ee_gripper_link")
            wait = self.wait_for_state_update(shape="tray", box_is_attached=True, box_is_known=False, timeout=timeout)
            # self.publish_dummy_marker()
            return wait
        self.scene.attach_box("wx250s/ee_gripper_link", self.box_name, touch_links="wx250s/ee_gripper_link")
        wait = self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
        # self.publish_dummy_marker()
        return wait

    def detach_box(self, shape="box", timeout=4):
        if shape == "tray":
            self.scene.remove_attached_object("wx250s/ee_gripper_link", name=self.tray_name)
            return self.wait_for_state_update(shape="tray", box_is_known=True, box_is_attached=False, timeout=timeout)
        self.scene.remove_attached_object("wx250s/ee_gripper_link", name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
    
    def detach_cylinder(self, timeout=4):
        self.scene.remove_attached_object("wx250s/ee_gripper_link", name=self.cylinder_name)
        return self.wait_for_state_update(shape="cylinder", box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, shape="box", timeout=4):
        if shape == "tray":
            self.scene.remove_world_object(self.tray_name)
            return self.wait_for_state_update(shape="tray", box_is_attached=False, box_is_known=False, timeout=timeout)
        self.scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
    
    def remove_cylinder(self, timeout=4):
        self.scene.remove_world_object(self.cylinder_name)
        return self.wait_for_state_update(shape="cylinder", box_is_attached=False, box_is_known=False, timeout=timeout)
    
    def compute_ik(self, js):
        x, y, z, qx, qy, qz, qw = js[0], js[1], js[2], js[3], js[4], js[5], js[6] 
        rs = RobotState()
        rs.joint_state.name = ['waist','shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        ik_srv.wait_for_service()
        gsvr = GetPositionIKRequest()

        gsvr.ik_request.pose_stamped.header.frame_id = "wx250s/base_link"
        gsvr.ik_request.pose_stamped.pose.position.x = x
        gsvr.ik_request.pose_stamped.pose.position.y = y
        gsvr.ik_request.pose_stamped.pose.position.z = z
        gsvr.ik_request.pose_stamped.pose.orientation.x = qx
        gsvr.ik_request.pose_stamped.pose.orientation.y = qy
        gsvr.ik_request.pose_stamped.pose.orientation.z = qz
        gsvr.ik_request.pose_stamped.pose.orientation.w = qw

        gsvr.ik_request.robot_state = rs
        gsvr.ik_request.group_name = "widowx250_manipulator"
        gsvr.ik_request.avoid_collisions = True
        gsvr.ik_request.ik_link_name = 'wx250s/ee_gripper_link'
        gsvr.ik_request.timeout = rospy.Duration(3)
        result = ik_srv.call(gsvr)
        return np.array(result.solution.joint_state.position)

    def convert_pixel_to_pos(self, pixel_x, pixel_y):
        x = (320 - pixel_x) * 0.003589743
        y = (pixel_y - 240) * 0.002910603
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
        self.package_location_y[self.count] = y
        # self.package_location_x[self.count] = x
        self.count += 1
        self.count %= 10
