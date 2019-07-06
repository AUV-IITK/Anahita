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

def depth_callback(msg):
    global depth_p
    depth_p = msg

def pinger_callback(msg):
    global pinger_heading
    pinger_heading = msg.data
    global init_pinger
    init_pinger = True

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
    rospy.init_node('pinger_controls')
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    depth_sub = rospy.Subscriber('/anahita/mean_coordinate', Point, depth_callback)
    pinger_callback = rospy.Subscriber('/anahita/pinger_heading', Float32, pinger_callback)
    straight_pub = rospy.Publisher('/anahita/straight', Float32, queue_size=10, latch=True)

    try:
        go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        hold_vehicle = rospy.ServiceProxy('anahita/hold_vehicle', Hold)
        trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
       # pinger_front = rospy.ServiceProxy('anahita/pinger_front_target', PingerFrontTarget)
      #  pinger_bottom = rospy.ServiceProxy('anahita/pinger_bottom_target', PingerBottomTarget)
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    change_odom(odom="dvl")
    
    #request to hold the vehicle before the pinger is detected

   # while not rospy.is_shutdown():
    #    if init_pinger:
     #       break

    pose = Pose()
    step_point = Point()
    print("Started")

  #  current_task(current_task="pinger")
    
    curr_eu = trans.euler_from_quaternion(current_p.orientation)
    target_eu = curr_eu
    target_eu.z = target_eu.z + pinger_heading

    pose.orientation = trans.quaternion_from_euler(target_eu)
    pose.position = current_p.position

    go_to_pose(target_pose=pose)
    rospy.loginfo("CMD to align to the pinger")

    pose_reach(time_out=20)
    rospy.sleep(2)
    rospy.loginfo('Aligned to the pinger now')

    straight_time = Float32()
    straight_time.data = 15
    straight_pub.publish(straight_time)

    then = rospy.get_time()
    success = False

    while not rospy.is_shutdown():
        now = rospy.get_time()
        if (pinger_front()):
            success = True
            break
        elif (now - then > straight_time.data):
            break
        rospy.sleep(2)

    if (not success):
        rospy.loginfo('Not found target in front camera')

        curr_eu = trans.euler_from_quaternion(current_p.orientation)
        target_eu = curr_eu
        target_eu.z = target_eu.z + pinger_heading

        pose.orientation = trans.quaternion_from_euler(target_eu)
        pose.position = current_p.position

        go_to_pose(target_pose=pose)
        rospy.loginfo("CMD to align to the pinger")

        pose_reach(time_out=20)
        rospy.sleep(2)
        rospy.loginfo('Aligned to the pinger now')

        straight_time.data = 15
        straight_pub.publish(straight_time)

        then = rospy.get_time()
        success = False

        while not rospy.is_shutdown():
            now = rospy.get_time()
            if (pinger_bottom()):
                success = True
                break
            elif (now - then > straight_time.data):
                break
            rospy.sleep(2)

        if (not success):
            rospy.loginfo('Failed to detect the target')
    else:
        change_odom(odom="vision")
        rospy.sleep(0.01)
        pose = current_p
        pose.position.y = 0
        go_to_pose(target_pose=pose)
        rospy.loginfo('aligned to the center of the target')
        pose_reach(time_out=10)
        rospy.loginfo('aligned now')

        straight_time.data = 15
        straight_pub.publish(straight_time)

        then = rospy.get_time()
        success = False

        while not rospy.is_shutdown():
            now = rospy.get_time()
            if (pinger_bottom()):
                success = True
                break
            elif (now - then > straight_time.data):
                break
            rospy.sleep(2)

        if (not success):
            rospy.loginfo('Failed to detect the target')

