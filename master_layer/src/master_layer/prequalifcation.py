#!/usr/bin/env python

import rospy
from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom
from anahita_utils import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

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
        #print status
        continue

if __name__ == '__main__':

    rospy.init_node('pre_qualification')
    """print 'waiting for waypoint server'
    rospy.wait_for_service('anahita/go_to_incremental')"""

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)

    try:
        """go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        go_to = rospy.ServiceProxy('anahita/go_to', GoTo)"""

        # verify that the object in front is the target (send a request to the object detection node)
        # if not then find it in the horizontal plane (send a request to do the horizontal motion)
        # if found then align to the center of the gate
        
        # change the odom source to the dvl
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
	pose = Pose()        

 	change_odom_response = change_odom(odom="dvl")

        
        fill_pose_data(pose, 0,0,0,0,0,0,1)
        pose_cmd_pub.publish(pose)
        rospy.loginfo('Publishing to the cmd pose')

        point = Point()
        point = pose.position
        has_reached(point, 0.2)

        rospy.loginfo("Aligned to the center of the gate")
        rospy.sleep(4)
	
        change_odom_response = change_odom(odom="dvl")
	

        pose.position.x = 3
        pose.position.y = current_p.y
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        rospy.loginfo('publishing to the cmd pose')

        pose_cmd_pub.publish(pose)

        point = Point()

        point = pose.position
        has_reached(point, 0.2)

        rospy.loginfo("In front of the gate")
        rospy.sleep(3)

        # wait for the vehicle to complete this action

        # then align to the center

        """go_to_resp = go_to(waypoint=center_waypoint, max_forward_speed=0.5, interpolator="cubic")"""
	'''
        change_odom_response = change_odom(odom="vision")

        pose(current_p.x, 0, 0, 0, 0, 0, 0, 1)
        rospy.loginfo('publishing to the cmd pose')       
        pose_cmd_pub.publish(pose)
        point = Point()
        point = pose.position
        has_reached(point, 0.2)
        rospy.loginfo("Aligned to the center of the gate")
        rospy.sleep(3)
	'''
        # then send a goto incremental request

        """go_to_incremental_resp = go_to_incremental(step=step_point, max_forward_speed=0.5, interpolator="cubic")"""

        change_odom_response = change_odom(odom="dvl")
	
        fill_pose_data(pose,current_p.x -3, current_p.y, 0, 0, 0, 0, 1)
        rospy.loginfo('publishing to the cmd pose')
        pose_cmd_pub.publish(pose)

        point = Point()
        point = pose.position
        has_reached(point, 0.2)

        rospy.loginfo("crossed the gate")
        rospy.sleep(3)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
