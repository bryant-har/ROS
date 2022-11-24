#!/usr/bin/python

import time
import rospy
import random
import numpy as np
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

random.seed(42)
np.random.seed(42)

class fake_grasping:
    def __init__(self):
        self.grasp_pub = rospy.Publisher('/cube_mover', Marker, queue_size=10)
        self.rate = rospy.Rate(30) # 30hz

    def publish_marker(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "wx250s/ee_gripper_link"
            marker.pose = Pose()
            marker.pose.position.x = 0.
            marker.pose.position.y = 0.
            marker.pose.position.z = 0.
            marker.pose.orientation.w = 1.
            if rospy.get_param('object') == "tray":
                marker.type = Marker.MESH_RESOURCE   
                marker.pose.position.x = 0.12 
                marker.pose.position.y = -0.01       
                marker.pose.position.z = 0.02
                marker.scale.x = 0.008
                marker.scale.y = 0.01
                marker.scale.z = 0.005
                marker.color.r = 200
                marker.color.g = 200
                marker.color.b = 200
                marker.color.a = 1.0
                marker.pose.orientation.x = -0.7048382;
                marker.pose.orientation.y = -0.0530044;
                marker.pose.orientation.z = 0.0529622;
                marker.pose.orientation.w = 0.7053997;
                marker.mesh_resource = "package://arm_final_project/scene/tray.dae"
                self.grasp_pub.publish(marker)
            elif rospy.get_param('object') == "wallet":
                marker.type = Marker.MESH_RESOURCE
                marker.pose.position.x = 0.03
                marker.pose.position.z = -0.05
                marker.scale.x = 0.005
                marker.scale.y = 0.005
                marker.scale.z = 0.005
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255.0
                marker.color.a = 1.0
                marker.mesh_resource = "package://arm_final_project/scene/wallet.dae"
                self.grasp_pub.publish(marker)
            elif rospy.get_param('object') == "can":
                marker.type = Marker.MESH_RESOURCE
                marker.pose.position.x = 0.01
                marker.pose.position.z = -0.045
                marker.scale.x = 0.005
                marker.scale.y = 0.005
                marker.scale.z = 0.005
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255.0
                marker.color.a = 1.0
                marker.mesh_resource = "package://arm_final_project/scene/can.dae"
                self.grasp_pub.publish(marker)
            else:
                continue
            self.rate.sleep()

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


