#!/usr/bin/env python
from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped

from cs4750 import utils
from cs4750.collector import SynchronizedMessageCollector


class TestParticleFilterServoing(unittest.TestCase):
    def setUp(self):
        pass

    def _test_particle_filter(self):
        reference_pose_topic = "gt_pose"
        plot = bool(rospy.get_param("~plot", False))

        collector = SynchronizedMessageCollector(
            ["/estimated_point", reference_pose_topic],
            [PoseStamped, PoseStamped],
        )
        try:
            rospy.sleep(5)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # We expect time to jump back because the test restarts the bag
            pass
        msgs = collector.start(duration=15)
        self.assertGreaterEqual(
            len(msgs),
            50,
            msg="The test didn't receive enough messages to be able to compare "
            "the particle filter estimate with the ground truth.",
        )
        sentinel = object()

        # Convert synchronized PoseStamped messages to state vectors
        estimates, references = [
            [utils.pose_to_particle(m.pose) for m in ms] for ms in zip(*msgs)
        ]
        estimates = np.array(estimates)
        references = np.array(references)

        position_error = utils.estimation_error_pose_only(
            estimates, references
        )
        # BEGIN SOLUTION NO PROMPT
        rospy.loginfo("Mean position error: {}".format(np.mean(position_error)))

        # END SOLUTION
        plt.xlabel("Time")
        plt.ylabel("Position Error")
        plt.plot(position_error)
        plt.show()
        plt.savefig("./position_error.png")

        pos_threshold = 9
        self.assertLess(
            np.mean(position_error),
            pos_threshold,
            msg="The mean position error should be less than {} pixels".format(
                pos_threshold
            ),
        )


if __name__ == "__main__":
    rospy.init_node("test_particle_filter_tracking")
    # The xml report will use the method's name, so we have to manually
    # mangle it with the bag name to ensure all entries appear
    setattr(
        TestParticleFilterServoing,
        "test_object_tracking_particle_filter",
        lambda self: self._test_particle_filter(),
    )
    rostest.rosrun(
        "arm_particle_filter", "test_object_tracking_particle_filter", TestParticleFilterServoing
    )
