#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from odom_dvl_imu.srv import SetDepthOffset
from odom_dvl_imu.srv import SetWorldXYOffset
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
        rospy.loginfo('The current roll is: ' + str(TO_DEGREE(_curr_roll))  + ", target roll is: " + str(TO_DEGREE(_target_roll)))
        while (abs(TO_DEGREE(_curr_roll) - TO_DEGREE(_target_roll)) > roll_threshold):
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
        target_roll = TO_DEGREE(target_roll) + 30
        target_pitch = TO_DEGREE(target_pitch)
        target_yaw = TO_DEGREE(target_yaw)
        target_x = 0
        target_roll_dt = 0
        while target_roll_dt <  360:
            rospy.loginfo("Still trying to roll")
            target_roll += 15
            if(target_roll>180):
                target_roll -= 360
            target_roll_dt += 15
        #     target_x += 1
            target_quaternion = eulerRPY_to_quaternion(TO_RADIAN(target_roll), TO_RADIAN(target_pitch), TO_RADIAN(target_yaw))
            fill_pose_data_xyzq(target_pose, target_x, 0, 0, target_quaternion)
            pose_cmd_pub.publish(target_pose)
            while has_reached_with_roll(target_pose, odom_msg.pose.pose, 1, 5) is False:
                continue

def odometry_callback (msg):
    global odom_msg
    odom_msg = msg
    #rospy.loginfo(odom_msg)

if __name__ == '__main__':

    rospy.init_node('barrel_roll')
    set_odom_xy = rospy.ServiceProxy('/nav/set_world_x_y_offset', SetWorldXYOffset)
    set_odom_xy_response = set_odom_xy()
    set_odom_depth = rospy.ServiceProxy('/nav/set_depth_offset', SetDepthOffset)
    set_odom_depth_response = set_odom_depth()

    rospy.loginfo("Services completed")

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)

    rospy.loginfo('Barrel roll node initialised')
    rospy.sleep(2)
    rospy.loginfo('after the sleep')
    barrel_roll_execute()
