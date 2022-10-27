import random
from operator import pos

import cv2
import imutils
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from arm_particle_filter.particle_filter import ParticleFilter

matplotlib.use('Agg')

random.seed(42)
np.random.seed(42)
plt.ylim(top=100)

cm = np.array([[500.0, 0.0, 320, 0.0], [
              0.0, 500, 240, 0.0], [0.0, 0.0, 1.0, 0.0]])
CM_INV = np.linalg.pinv(cm)


def convert_pixel_to_pos(pixel_x, pixel_y):
    p = np.array([pixel_x, pixel_y, 1])
    real = np.dot(CM_INV, p)
    real[0] = 1.95 * real[0]
    real[1] = 1.95 * (real[1])
    real[0] *= -1
    return real[0], real[1]


class CylinderTracker:
    def __init__(self):
        # BEGIN SOLUTION "QUESTION 1.4": Parameters to tune
        ################################################################
        # Number of particles to be initialized for the particle filter
        num_particles = 1000

        # Constant velocity to be supplied to ParticleFilter.predict
        self.constant_vel = np.array([1, 1])

        # Sample motion model noise with this std=std_constant_vel
        std_constant_vel = 1

        # Initial mean and covariance of the sampled particles
        initial_mean = np.array([0, 0]).reshape((-1, 1))
        initial_cov = np.diag([10e7, 10e7])
        ################################################################

        self.pf = ParticleFilter(
            initial_mean,
            initial_cov,
            num_particles,
            std_constant_vel
        )
        self.count = 0

        self.x_prev = int(initial_mean[0])
        self.y_prev = int(initial_mean[1])
        self.cylinder_gt = None
        self.radius = 15

        self.gt_cylinder_pub = rospy.Publisher(
            '/gt_pose', PoseStamped, queue_size=1)
        self.estimated_cylinder_pub = rospy.Publisher(
            '/estimated_point', PoseStamped, queue_size=1)
        self.estimated_3dcylinder_pub = rospy.Publisher(
            '/estimated_cylinder', Marker, queue_size=10)
        self.image_sub = rospy.Subscriber(
            '/camera1/image_raw', Image, self.detect_cb)  # check name by rostopic list
        self.plot = bool(rospy.get_param("~plot", False))
        self.detected = False
        self.running = True
        rospack = rospkg.RosPack()
        self.plot_path = rospack.get_path('arm_particle_filter')

    def pause_pf(self):
        self.running = False
        rospy.loginfo("Pausing the particle filter.")

    def resume_pf(self):
        self.running = True
        rospy.loginfo("Resuming the particle filter.")

    def kill_pf(self):
        rospy.signal_shutdown("Tracking complete!")

    def get_detection(self, image, header):
        """Detect and return cylinder position coordinates.

        Args:
            image: cv2 image frame
            header: ROS header object

        Returns:
            (float, float) comprising of the detected cylinder position
        """

        lower = np.array([100, 50, 20])
        upper = np.array([120, 255, 255])

        thresh = cv2.inRange(image, lower, upper)
        cnts = cv2.findContours(
            thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            # compute the center of the contourestimated_cylinder
            (x_true, y_true), radius = cv2.minEnclosingCircle(c)
            gt_pose = PoseStamped()
            gt_pose.header = header
            gt_pose.pose.position.x, gt_pose.pose.position.y = x_true, y_true
            self.gt_cylinder_pub.publish(gt_pose)
            self.detected = True
            return x_true, y_true, radius
        else:
            return [None]*3

    def add_noise(self, x_true, y_true, radius, current_frame):
        """Add simulated noise to ground truth position for the cylinder.

        Args:
            x_true (int): true x-coordinate of cylinder center
            y_true (int): true y-coordinate of cylinder center
            radius (int) : true radius of cylinder
            current_image: cv2 image frame

        Returns:
            (float, float) comprising of the noisy detected cylinder position
        """
        if np.random.rand() > 0.7:
            x = x_true + int(np.random.normal(0, 50, 1))
            y = y_true + int(np.random.normal(0, 50, 1))
        else:
            x = x_true
            y = y_true
        cv2.circle(current_frame, (int(x), int(y)),
                   int(radius), (255, 0, 0), 2)
        return x, y

    def detect_cb(self, data):
        """Detects, tracks and publishes the estimated position
        a moving cylinder.

        Args:
            data (sensor_msgs.msg.Image): top down RGB image
        """
        self.detected = False
        stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = "world"
        br = CvBridge()
        current_frame = br.imgmsg_to_cv2(data)
        image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        result = current_frame.copy()

        if self.running:
            # Obtain true detected pose of cylinder
            x_true, y_true, radius = self.get_detection(image, header)

            if self.detected:
                cv2.circle(current_frame, (int(x_true), int(y_true)),
                           int(radius), (0, 0, 255), 4)

                # Add simulated noise to detected pose
                x, y = self.add_noise(x_true, y_true, radius, current_frame)
                self.x_prev, self.y_prev = x, y

            if not self.detected:
                x, y = self.x_prev, self.y_prev
            radius = self.radius

            # Predict target pose
            self.pf.predict(self.constant_vel)

            # Update particle and corresponding weights
            mean, cov = self.pf.update(np.array([x, y]))
            self.pf.draw_particles(current_frame)

            # Resample all particles
            self.pf.resampler.resample()

            x_estimated, y_estimated = mean.ravel().astype(int)

            # Draw pf estimated cylinder position on image
            x_draw = x_estimated
            y_draw = y_estimated
            cv2.circle(current_frame, (x_draw, y_draw),
                       radius, (0, 255, 0), 4)

            # Publish estimated cylinder position for the arm to track
            pt = PoseStamped()
            pt.header = header
            pt.pose.position.x, pt.pose.position.y = x_estimated, y_estimated
            self.estimated_cylinder_pub.publish(pt)

            x_estimated, y_estimated = convert_pixel_to_pos(
                x_estimated, y_estimated)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = Marker.CYLINDER
            marker.pose = Pose()
            marker.pose.position.x = x_estimated
            marker.pose.position.y = y_estimated
            marker.pose.orientation.w = 1.
            marker.color.r = 255.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            self.estimated_3dcylinder_pub.publish(marker)

        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
