#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from anahita_utils import *

pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
odom_msg = Odometry()


def has_reached_with_roll(target_pose, curr_pose, pos_threshold, roll_threshold):
        while (rospy.is_shutdown()):
            return False
        while (calc_dist(target_pose.position, curr_pose.position) > pos_threshold or rospy.is_shutdown()):
            return False
        _curr_roll, _curr_pitch, _curr_yaw = quaternion_to_eulerRPY(curr_pose.orientation)
        _target_roll, _target_pitch, _target_yaw = quaternion_to_eulerRPY(target_pose.orientation)
        while (abs(_curr_roll - _target_roll) > roll_threshold):
            return False
        return True

def barrel_roll_execute():
        rospy.loginfo("Starting to execute barrel_roll")
        initial_pose = Pose()
        fill_pose_msg = (initial_pose, 0, 0, 0, 0, 0, 0, 1)
        pose_cmd_pub.publish(initial_pose)
        while has_reached_with_roll(initial_pose, odom_msg.pose.pose, 1, 5) is False:
                continue
        rospy.loginfo("Reached a stable initial pose")
        target_roll, target_pitch, target_yaw = quaternion_to_eulerRPY(initial_pose.orientation)
        target_pose = Pose()
        target_x = 0
        target_roll_dt = 0
        while target_roll_dt <  360:

            rospy.loginfo("Still trying to roll")

            target_roll += 10
            if(target_roll>180):
                target_roll -= 360
            target_roll_dt += 10
        #     target_x += 1
            rospy.loginfo('The target roll is: ' + str(target_roll))
            target_quaternion = eulerRPY_to_quaternion(TO_DEGREE(target_roll), target_pitch, target_yaw)
            fill_pose_data_xyzq(target_pose, target_x, 0, 0, target_quaternion)
            pose_cmd_pub.publish(target_pose)
            while has_reached_with_roll(target_pose, odom_msg.pose.pose, 1, 2) is False:
                rospy.loginfo("reaching")


def odometry_callback (msg):
    global odom_msg
    odom_msg = msg

if __name__ == '__main__':

    rospy.init_node('barrel_roll')
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    rospy.loginfo('Barrel roll node initialised')
    rospy.sleep(2)
    rospy.loginfo('after the sleep')
    barrel_roll_execute()
