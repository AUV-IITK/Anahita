#!/usr/bin/env python

import rospy

from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom
from master_layer.srv import CurrentTask
from master_layer.srv import GoToPose
from master_layer.srv import TrajectoryComplete
from master_layer.srv import PoseReach
from master_layer.srv import RequestMarkerAngle

import time
from std_msgs.msg import Time, String

from anahita_utils import *

from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import tf.transformations as trans

current_p = Pose()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose

if __name__=='__main__':
    rospy.init_node ('test_node')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10, latch=True)

    try:
        go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        go_to = rospy.ServiceProxy('anahita/go_to', GoTo)
        current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
        marker_angle = rospy.ServiceProxy('anahita/marker_angle', RequestMarkerAngle)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    change_odom(odom='bottom_vision')
    rospy.sleep(0.1)

    pose = current_p
    pose.position.x = 0
    pose.position.y = 0

    go_to_pose(target_pose=pose)
    rospy.loginfo('cmd to align')
    pose_reach(time_out=20)

    rospy.wait_for_service('anahita/marker_angle')
    marker_angle_resp = marker_angle()
    print('major angle: {}, minor angle: {}'.format(marker_angle_resp.major_angle, marker_angle_resp.minor_angle))

    pose = current_p
    intial_orientation = quaternion_to_eulerRPY(current_p.orientation)
    opp_q = eulerRPY_to_quaternion (intial_orientation[0], intial_orientation[1], intial_orientation[2] + marker_angle_resp.major_angle)
    
    pose.orientation.x = opp_q[0]
    pose.orientation.y = opp_q[1]
    pose.orientation.z = opp_q[2]
    pose.orientation.w = opp_q[3]

    go_to_pose(target_pose=pose)
    rospy.loginfo('cmd to turn')
    pose_reach(time_out=20)