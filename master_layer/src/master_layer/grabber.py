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
from xsens_driver.srv import ResetIMUOrient
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
    rospy.loginfo(str(current_p))

def depth_callback(msg):
    global depth_p
    depth_p = msg

def pinger_callback(msg):
    global pinger_heading
    pinger_heading = msg.data
    global init_pinger
    init_pinger = True

if __name__ == '__main__':
    rospy.init_node('grabber')
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    #straight_pub = rospy.Publisher('/anahita/straight', Float32, queue_size=10, latch=True)

    try:
        go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        hold_vehicle = rospy.ServiceProxy('anahita/hold_vehicle', Hold)
        reset_imu = rospy.ServiceProxy('/nav/set_imu_quat', ResetIMUOrient)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
      
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    current_task(current_task="grabber_test")
    change_odom(odom="vision")
    intial_orientation = quaternion_to_eulerRPY(current_p.orientation)
#    reset_imu_resp = reset_imu()
	
    # 1st torpedo shoot
   # change_torpedo_hole(hole="BOT")
    pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
    go_to_pose(target_pose=pose)
    rospy.loginfo('Publishing cmd to align')
    rospy.sleep(100)

    change_odom(odom="dvl")
#    pose = fill_pose(current_p)
    go_to_pose(target_pose=current_p)
    rospy.loginfo('Publishing cmd to align')
    rospy.sleep(100)
    
    
    
