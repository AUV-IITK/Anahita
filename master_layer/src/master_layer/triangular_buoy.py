#!/usr/bin/env python

import rospy
import numpy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float32

from master_layer.srv import GoToIncremental
from master_layer.srv import CurrentTask
from master_layer.srv import ChangeOdom
from master_layer.srv import GoToPose
from master_layer.srv import Hold
from master_layer.srv import TrajectoryComplete
from master_layer.srv import PoseReach
from master_layer.srv import PingerFrontTarget
from master_layer.srv import PingerBottomTarget

from anahita_utils import *

import tf.transformations as trans

current_p = Pose()
depth_p = Point()
pinger_heading = 0
init_pinger = False

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose

if __name__ == '__main__':
    rospy.init_node('pinger_controls')
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    vel_cmd_pub = rospy.Publisher('/anahita/cmd_vel', Twist, queue_size=10, latch=True)
    disable_cmd_pose = rospy.Publisher('/anahita/disable_cmd_pose', Bool, queue_size=10, latch=True)

    try:
        go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        hold_vehicle = rospy.ServiceProxy('anahita/hold_vehicle', Hold)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    current_task(current_task="triangular_buoy")
    rospy.loginfo('vision layer triangular task activated')
    change_odom(odom="stereo_vision")

    # find the buoy
    # locate the buoy
    # approach the buoy
    # stop before a fixed distance
    # align to the center
    # send a request to know the blackout time or speed
    # can't use go to incremental
    # `straight` script and `go_to_pose` have fixed speeds
    # need to give a large value and small time out to `go_to_pose`
    # need to sync the approach with the blackout time

    time = 10.0
    dist = 3.1

    vel = Twist()
    vel.linear.x = dist/time
    print dist/time
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    msg = Bool()
    msg.data = True
    disable_cmd_pose.publish(msg)
    vel_cmd_pub.publish(vel)
    rospy.loginfo('cmd to move forward')
    rospy.sleep(time)
    vel.linear.x = -vel.linear.x
    vel_cmd_pub.publish(vel)
    rospy.loginfo('cmd to move back')
    rospy.sleep(time)
    vel.linear.x = 0
    vel_cmd_pub.publish(vel)
    msg.data = False
    disable_cmd_pose.publish(msg)
