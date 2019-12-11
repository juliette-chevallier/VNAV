#!/usr/bin/env python

# Add zero-mean noise to groundtruth husky poses

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import tf

class NoisyPoses:
    def __init__(self):
        self.sub = rospy.Subscriber('/unity_ros/husky/TrueState/pose', PoseStamped, self.corrupt_poses)
        self.br = tf.TransformBroadcaster()
        # errors = np.array([0, .01, .05, .1, .2, .3, .4, .5])
        self.noise_trans = 0
        # errors = np.array([0, .25, .5, .75, 1, 3, 5]) deg
        # errors = np.array([0, 0.00436332, .00872665, 0.01309, 0.0174533, 0.0523599,0.0872665]) rad
        self.noise_rot = 0.0523599

    def corrupt_poses(self, msg):

        state = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

        if self.noise_trans != 0:
            trans_noise = np.random.normal(0, self.noise_trans, 3)
            state = [msg.pose.position.x + trans_noise[0], msg.pose.position.y + trans_noise[1], msg.pose.position.z + trans_noise[2]]
        if self.noise_rot != 0:
            rot_noise = np.random.normal(0, self.noise_rot, 3)
            euler = euler_from_quaternion(quaternion)
            euler_noised = (euler[0] + rot_noise[0], euler[1] + rot_noise[1], euler[2] + rot_noise[2])
            quaternion = quaternion_from_euler(euler_noised[0], euler_noised[1], euler_noised[2])

        self.br.sendTransform(state, quaternion, msg.header.stamp, "/husky/NoisyState", "world")


if __name__ == '__main__':
    rospy.init_node('NoisyPoses')
    sim = NoisyPoses()
    rospy.spin()