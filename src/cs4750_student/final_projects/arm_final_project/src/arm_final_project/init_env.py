#!/usr/bin/python

import time
import rospy
import random
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import Header
from interbotix_xs_modules.arm import InterbotixManipulatorXS

random.seed(42)
np.random.seed(42)

class Env:
    def __init__(self, y, shape="box"):
        self.shape = shape
        self.cube_pub = rospy.Publisher('/cube_mover', Marker, queue_size=10)
        self.prev_pose = np.array([0., 0.])

        # cube params
        self.rate = rospy.Rate(10) # 10hz
        self.coord_y = np.linspace(0.25, y, 100)
        self.idx = 0


    def publish_obj(self):
        time.sleep(1)
        while self.idx < len(self.coord_y):
            marker = Marker()
            marker.header.frame_id = "wx250s/base_link"
            
            marker.pose = Pose()
            marker.pose.position.x = 0.53
            marker.pose.position.y = self.coord_y[self.idx]
            self.idx += 1

            marker.pose.orientation.w = 1.
            if self.shape == "tray":
                marker.type = Marker.MESH_RESOURCE
                marker.pose.position.x = 0.47
                marker.pose.position.z = 0.14
                marker.scale.x = 0.008
                marker.scale.y = 0.01
                marker.scale.z = 0.005
                marker.color.r = 200
                marker.color.g = 200
                marker.color.b = 200
                marker.color.a = 1.0
                marker.mesh_resource = "package://arm_final_project/scene/tray.dae"
            elif self.shape == "wallet":
                marker.type = Marker.MESH_RESOURCE
                marker.pose.position.x = 0.58
                marker.pose.position.z = 0.12
                marker.scale.x = 0.005
                marker.scale.y = 0.005
                marker.scale.z = 0.005
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255.0
                marker.color.a = 1.0
                marker.mesh_resource = "package://arm_final_project/scene/wallet.dae"
            elif self.shape == "can":
                marker.type = Marker.MESH_RESOURCE
                marker.pose.position.x = 0.573
                marker.pose.position.z = 0.14
                marker.scale.x = 0.005
                marker.scale.y = 0.005
                marker.scale.z = 0.005
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255.0
                marker.color.a = 1.0
                marker.mesh_resource = "package://arm_final_project/scene/can.dae"
            else:
                print("unsupported object")
                

                
            self.cube_pub.publish(marker)
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


