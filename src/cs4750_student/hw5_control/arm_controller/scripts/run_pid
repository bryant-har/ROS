from interbotix_xs_modules.arm import InterbotixManipulatorXS
import math
import rospy
import numpy as np
import modern_robotics as mr
from interbotix_xs_sdk.msg import *
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_common_modules import angle_manipulation as ang
from interbotix_xs_modules import mr_descriptions as mrd
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface
from arm_controller.pid_controller import PIDController


def main():
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    pid = PIDController(bot)
    plan = np.array([[ 0. ,0.,0.,0., 0.,0.],
                    [-0.5043, 0.2133, -0.2313, 1.3841, 0.5309, -1.3918],
                    [-0.5136, 0.2568, 0.1185, 2.017, 0.5949, -2.1329],
                    [0.5298, 0.2391, 0.1323, -2.032, 0.5765, 2.1581],
                    [0.5151, 0.1996, -0.2673, -1.2891, 0.5174, 1.3002],
                    [0,0,0,0,0,0]])
    raw_traj = pid.execute(plan)
    bot.arm.core.robot_write_trajectory("group", "arm", "position", raw_traj)

if __name__=='__main__':
    main()