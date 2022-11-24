from __future__ import print_function

import errno

import networkx as nx
import numpy as np
import os
import rospkg
import rospy
import tf2_ros
import time

from geometry_msgs.msg import PoseStamped, PoseArray, Point
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Header

from cs4750 import utils
from car_controller.srv import FollowPath
from planning.search import ASTARPlanner
from planning.problems import SE2Problem
from planning.roadmap import Roadmap
from planning.samplers import samplers

from car_planning.planner_ros import PlannerROS
from mushr_sim.srv import CarPose


class PlannerROSWithControl(PlannerROS):
    def __init__(
        self,
        num_vertices,
        connection_radius,
        curvature,
        do_shortcut=True,
        sampler_type="halton",
        tf_prefix="",
        tf_listener=None,
    ):
        self.tf_prefix = tf_prefix

        if tf_listener:
            self.tl = tf_listener
        else:
            self.tl = tf2_ros.TransformListener(tf2_ros.Buffer())

        self.num_vertices = num_vertices
        self.connection_radius = connection_radius
        self.do_shortcut = do_shortcut


        self.permissible_region, self.map_info = utils.get_map("/static_map")
        self.problem = SE2Problem(
            self.permissible_region,
            map_info=self.map_info,
            check_resolution=0.1,
            curvature=curvature,
        )
        self.rm = None

        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.get_goal
        )

        self.nodes_viz = rospy.Publisher(
            "~vertices", PoseArray, queue_size=1, latch=True
        )
        self.edges_viz = rospy.Publisher("~edges", Marker, queue_size=1, latch=True)
        self.path_edges_viz = rospy.Publisher("~path_edges", Marker, queue_size=1, latch=True)
        self._path_pub = rospy.Publisher(
            "~planned_path", PoseArray, queue_size=1, latch=True
        )
        self.car_reposition = rospy.ServiceProxy('/mushr_sim/reposition', CarPose)
        # Load or construct a roadmap
        rospack = rospkg.RosPack()
        saveto = rospack.get_path('car_final_project') + "/config/problem-setup_se2_halton_2000_10.0_1.0.pkl"

        sampler = samplers[sampler_type](self.problem.extents)
        rospy.loginfo("Constructing roadmap...")
        rospy.loginfo(saveto)
        start_stamp = time.time()
        self.rm = Roadmap(
            self.problem,
            sampler,
            num_vertices,
            connection_radius,
            saveto=saveto,
        )
        load_time = time.time() - start_stamp
        rospy.loginfo("Roadmap constructed in {:2.2f}s".format(load_time))
        self.planner = ASTARPlanner(self.rm)
        self.controller = rospy.ServiceProxy(
            "controller/follow_path", FollowPath)
        self.car_status = rospy.Publisher('/car_status', Bool, queue_size=10, latch=True)
        self.car_status.publish(True)

    def get_goal(self, msg):
        """Goal callback function."""
        if self.rm is None:
            return False

        rospy.loginfo("Goal received!")
        self.goal = np.array(utils.pose_to_particle(msg.pose))
        start = self._get_car_pose()

        path_states = self.plan_to_goal(start, self.goal)
        if path_states is None:
            return False
        self.visualize_path_edges(path_states)
        return self.send_path(path_states)

    def send_path(self, path_states):
        """Send a planned path to the controller."""
        """BEGIN SOLUTION. 
        HINT: Send nav_msgs.msg.Path object to controller
        The Path object has two attributes, header and poses.
        Use std_msgs.msg.Path and std.msg.Header to initialize them.
        Use stamp as the current time and frame_id as "map".
        Finally, return a call to self.controller(Path, desired_speed), where desired_speed
        is a float value you choose. Feel free to play around with the value you choose.

        params: 
            path_states: list
        return: 
            function call
        """
        
        """END SOLUTION"""

