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

from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import tf.transformations as trans

current_p = Pose()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose

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
        rospy.sleep(0.1)

        step_point.x = 6
        step_point.y = 0
        step_point.z = 0

        rospy.loginfo('cmd to cross the gate')
        go_to_incremental(step=step_point, max_forward_speed=0.4, interpolator="cubic")
        trajectory_complete(time_out=60)

        rospy.loginfo("past the gate")
        rospy.sleep(5)
        
        current_task_resp = current_task(current_task="marker")
        rospy.sleep(1)
        change_odom_response = change_odom(odom="stereo")
        rospy.sleep(0.1)

        pose = current_p
        pose.position.x = -1.7
        pose.position.y = 0
        pose.position.z = -2.5
        rospy.loginfo('cmd to approach marker')
        
        go_to_pose(target_pose=pose)
        trajectory_complete(time_out=30)
        rospy.loginfo("infront of the marker")
        rospy.sleep(4)

        change_odom_response = change_odom(odom="dvl")
        rospy.sleep(0.1)

        rospy.loginfo("publishing cmd for turning")

        pose = fill_pose(current_p.position.x, current_p.position.y, -2.5, 0, 0, 0.7071068, 0.7071068)
        go_to_pose(target_pose=pose)
        pose_reach(time_out=20)    
        rospy.loginfo("bot turned 90 degrees")

        rospy.loginfo("starting the circle")

        change_odom_response = change_odom(odom="dvl")
        rospy.sleep(2)

        step_point = fill_point(2, 1.5, 0)

        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete(time_out=200)

        rospy.loginfo("reached first quarter of the circle")
        rospy.sleep(1)

        step_point = fill_point(2.5, -1.7, 0)

        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete(time_out=200)

        rospy.loginfo("reached second quarter of the circle")
        rospy.sleep(1)

        step_point = fill_point(-1.5, -1.5, 0)

        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete(time_out=200)

        rospy.loginfo("reached third quarter of the circle")
        rospy.sleep(1)

        step_point = fill_point(-1.5, 1.5, 0)

        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete(time_out=200)

        rospy.loginfo("reached fourth quarter of the circle")
        rospy.sleep(1)

        change_odom_response = change_odom(odom="dvl")
        rospy.loginfo('changing the odom source to vision')
        rospy.sleep(1)

        pose = fill_pose(current_p.position.x, current_p.position.y, -2, opp_q[0], opp_q[1], opp_q[2], opp_q[3])
        go_to_pose(target_pose=pose)
        rospy.loginfo('cmd sent for turning back')

        pose_reach(time_out=20)
        rospy.loginfo('turned back')

        current_task(current_task="gate")
        rospy.loginfo('changing the vision target to gate')
        rospy.sleep(1)
        change_odom(odom="vision")
        rospy.loginfo('changing the odom source to vision')
        rospy.sleep(0.1)

        pose.position.x = current_p.position.x
        pose.position.y = 0
        pose.position.z = 0

        pose.orientation = opp_q
        rospy.loginfo("present coordinate: %f", current_p.x)

        go_to_pose(target_pose=pose)
        rospy.loginfo('cmd to align to the center of the gate')
        pose_reach(time_out=20)
        rospy.loginfo('bot aligned to gate center now')

        change_odom(odom="dvl")
        rospy.loginfo('changind the odom source to dvl')
        rospy.sleep(0.1)

        step_point = fill_point(-4, 0, 0)

        rospy.loginfo('cmd to go near the gate')
        go_to_incremental(step=step_point, max_forward_speed=0.4, interpolator="cubic")
        trajectory_complete(time_out=60)
        rospy.loginfo('infront of the gate')

        change_odom(odom="vision")
        rospy.loginfo('changind the odom source to vision')
        rospy.sleep(0.1)

        pose.position.x = current_p.x
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation = opp_q

        go_to_pose(target_pose=pose)
        rospy.loginfo('cmd to align to the center of the gate')
        pose_reach(time_out=30)
        rospy.loginfo('aligned to the center')

        change_odom(odom="dvl")
        rospy.loginfo('changind the odom source to dvl')
        rospy.sleep(0.1)

        step_point = fill_point(-5, 0, 0)

        go_to_incremental(step=step_point, max_forward_speed=0.2, interpolator="cubic")
        trajectory_complete(time_out=60)
        rospy.loginfo('past the gate')

        rospy

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
