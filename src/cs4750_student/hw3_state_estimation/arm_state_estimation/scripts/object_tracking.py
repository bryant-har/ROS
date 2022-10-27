import random
from operator import pos

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from arm_particle_filter.tracker import CylinderTracker

matplotlib.use('Agg')

random.seed(42)
np.random.seed(42)
plt.ylim(top=100)

if __name__ == '__main__':
    rospy.init_node('cylinder_tracking', disable_signals=True)
    detector = CylinderTracker()
    rospy.spin()
    cv2.destroyAllWindows()
