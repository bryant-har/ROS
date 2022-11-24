import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class PIDController():
    def __init__(self, bot):
        self.bot = bot
        self.kp = 3.0
        self.kd = 0.1
        self.pid_clear()
        self.max_speed = np.array([3.14, 3.14, 3.14, 3.14, 3.14, 3.14])
        self.min_speed = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.frequency = 20
        self.arm = None
        self.current_position = self.get_current_joints()

    def pid_clear(self):
        '''
            p_error: error at last iteration
            d_error: change in error, de/dt
        '''
        self.p_error = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.d_error = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def set_refs(self, refs):
        self.refs = refs

    def clamp_speed(self, u):
        abs_vel = np.abs(u)
        neg = abs_vel != u
        abs_vel = np.minimum(abs_vel, self.max_speed)
        return abs_vel - 2 * neg * abs_vel

    def get_current_joints(self):
        if self.bot == None:
            return np.array([0, 0, 0, 0, 0, 0])
        return [self.bot.arm.core.joint_states.position[self.bot.arm.core.js_index_map[name]] for name in self.bot.arm.group_info.joint_names]

    def get_error(self):
        '''
            Return:
                error: a 6D array representing error in each joint.
        '''
        return self.current_position - self.refs

    def get_control(self):
        ''' 
            Input:
                self.kp and self.kd are provided for you.
                Don't forget to update self.d_error and self.p_error.
            Rerturn:
                u: 6D joint velocity
        '''
        # BEGIN SOLUTION Q3 ##
        curr = self.get_error()
        self.d_error = (curr - self.p_error) * self.frequency
        self.p_error = curr
        return -(self.kp * self.p_error + self.kp * self.d_error)
        # END SOLUTION Q3 ##

        # To make sure speed does not exceed joint velocity limit
        u = self.clamp_speed(u)
        return u

    def compute_distance(self, start_config, end_config):
        start_config = np.array(start_config).reshape((1, 6))
        end_config = np.array(end_config).reshape((1, 6))
        return np.linalg.norm(start_config - end_config)

    def interpolate(self, q1, q2, resolution=0.1):
        q1 = q1.reshape((1, 6))
        q2 = q2.reshape((1, 6))
        dist = np.linalg.norm(q1 - q2)
        if dist < resolution:
            return np.vstack((q1, q2))
        else:
            q1_toward_q2 = (q2 - q1)/dist
            steps = np.hstack((np.arange(0, dist, resolution),
                              np.array([dist]))).reshape((-1, 1))
        return q1 + q1_toward_q2 * steps

    def execute(self, plan):
        # r = rospy.Rate(self.frequency)
        traj_dict_list = []
        t = 0
        traj_dict = {}
        traj_dict[t] = self.current_position
        plan = np.vstack((np.array(self.current_position), plan))
        for i in range(plan.shape[0]-1):
            interpolate_line = self.interpolate(plan[i], plan[i + 1])
            for j in range(interpolate_line.shape[0]):
                self.set_refs(interpolate_line[j])
                while self.compute_distance(self.refs, self.current_position) > 0.06:
                    traj_dict = {}
                    vel_cmd = self.get_control().tolist()
                    pos_cmd = np.array(self.current_position) + \
                        np.array(vel_cmd) / self.frequency
                    pos_cmd = pos_cmd.reshape((1, 6))
                    traj_dict[t] = pos_cmd[0].tolist()
                    traj_dict_list.append(traj_dict)
                    self.current_position = pos_cmd[0].tolist()
                    t += 1/self.frequency
                    # r.sleep()

        return traj_dict_list
