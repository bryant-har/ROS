#!/usr/bin/env python

import rospy
import rostest
import unittest

from std_msgs.msg import Bool


class TestPrimeNumberSmall(unittest.TestCase):
    def test_prime_number_small(self):
        message = rospy.wait_for_message("/introduction/prime_number_output", Bool, timeout=5)
        self.assertEqual(message.data, True)


if __name__ == "__main__":
    rospy.init_node("test_prime_number_small")
    rostest.rosrun("introduction", "test_prime_number_small", TestPrimeNumberSmall)
