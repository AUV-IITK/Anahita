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

status = False
current_p = Point()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose.position
    global status
    status = True    

def has_reached (pos, threshold):
    
    while (not status or rospy.is_shutdown()):
        continue
    while (calc_dist(pos, current_p) > threshold or rospy.is_shutdown()):
        # print status
        continue

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

        change_odom_response = change_odom(odom="dvl")
        rospy.loginfo('changind the odom source to dvl')
        rospy.sleep(0.1)

        fill_pose(pose, 8)

        step_point.x = 1
        step_point.y = 0
        step_point.z = 0

        go_to_incremental_resp = go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete_resp = trajectory_complete(time_out=60)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
