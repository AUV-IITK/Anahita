#!/usr/bin/env python

import rospy

from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom
from master_layer.srv import CurrentTask
from master_layer.srv import GoToPose
from master_layer.srv import InitCircularTrajectory
from master_layer.srv import Hold
from master_layer.srv import TrajectoryComplete
from master_layer.srv import PoseReach

import time
from std_msgs.msg import Time, String

from anahita_utils import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import tf.transformations as trans

current_p = Point()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose.position
    global status
    status = True    


def fill_pose (p_x, p_y, p_z, q_x, q_y, q_z, q_w):
    
    pose = Pose()
    pose.position.x = p_x
    pose.position.y = p_y
    pose.position.z = p_z
    pose.orientation.x = q_x
    pose.orientation.y = q_y
    pose.orientation.z = q_z
    pose.orientation.w = q_w

    return pose 

def fill_point (x, y, z):

    point = Point()
    point.x = x
    point.y = y
    point.z = z

    return point

if __name__ == '__main__':

    rospy.init_node('gate_qual')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10, latch=True)

    try:
        go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        go_to = rospy.ServiceProxy('anahita/go_to', GoTo)
        current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        init_circular_trajectory = rospy.ServiceProxy('anahita/start_circular_trajectory', InitCircularTrajectory)
        hold_vehicle = rospy.ServiceProxy('anahita/hold_vehicle', Hold)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)

        pose = Pose()
        step_point = Point()

        current_task_resp = current_task(current_task="torpedo")
        change_odom_response = change_odom(odom="vision")

        # 1st torpedo shoot

        pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
        go_to_pose(target_pose=pose)
        rospy.loginfo('Publishing cmd to align')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Aligned center')

        change_odom(odom="stereo_vision")

        pose = fill_pose(0, 0, -1.5, 0, 0, 0, 1)
        go_to_pose(targest_pose=pose)
        rospy.loginfo('cmd to move near the torpedo hole')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Infront of the torpedo hole')

        # fire in the hole

        change_odom(odom="dvl")
        step_point = fill_point(-5, 0, 0)
        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        rospy.loginfo('cmd to go back')

        target_complete(time_out=30)
        rospy.loginfo('bot is back')        

        # 2nd torpedo shoot

        pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
        go_to_pose(target_pose=pose)
        rospy.loginfo('Publishing cmd to align')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Aligned center')

        change_odom(odom="stereo_vision")

        pose = fill_pose(0, 0, -1.5, 0, 0, 0, 1)
        go_to_pose(targest_pose=pose)
        rospy.loginfo('cmd to move near the torpedo hole')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Infront of the torpedo hole')

        # fire in the hole

        change_odom(odom="dvl")
        step_point = fill_point(-5, 0, 0)
        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        rospy.loginfo('cmd to go back')

        target_complete(time_out=30)
        rospy.loginfo('bot is back')        

        # 3rd torpedo shoot

        pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
        go_to_pose(target_pose=pose)
        rospy.loginfo('Publishing cmd to align')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Aligned center')

        change_odom(odom="stereo_vision")

        pose = fill_pose(0, 0, -1.5, 0, 0, 0, 1)
        go_to_pose(targest_pose=pose)
        rospy.loginfo('cmd to move near the torpedo hole')

        pose_reach(time_out=20)
        rospy.sleep(5)
        rospy.loginfo('Infront of the torpedo hole')

        # fire in the hole

        change_odom(odom="dvl")
        step_point = fill_point(-5, 0, 0)
        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        rospy.loginfo('cmd to go back')

        target_complete(time_out=30)
        rospy.loginfo('bot is back')        
