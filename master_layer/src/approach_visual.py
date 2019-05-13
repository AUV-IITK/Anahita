#!/usr/bin/env python

import rospy
from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

status = False
current_p = Point()

def calc_dist (pose1, pose2):
    x1 = pose1.x
    y1 = pose1.y
    z1 = pose1.z

    x2 = pose2.x
    y2 = pose2.y
    z2 = pose2.z

    dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)
    return dist

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
    rospy.loginfo("Starting the visual-approach task")
    
    rospy.init_node('approach_visual')
    """print 'waiting for waypoint server'
    rospy.wait_for_service('anahita/go_to_incremental')"""

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)

    try:
        rospy.loginfo("Entering the trying statement")
        """go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
        go_to = rospy.ServiceProxy('anahita/go_to', GoTo)"""

        # verify that the object in front is the target (send a request to the object detection node)
        # if not then find it in the horizontal plane (send a request to do the horizontal motion)
        # if found then align to the center of the gate
        
        # change the odom source to the dvl

        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        change_odom_response = change_odom(odom="vision")

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        pose_cmd_pub.publish(pose)
        rospy.loginfo('Publishing to the cmd pose')

        point = Point()

        point = pose.position
        has_reached(point, 3)

        rospy.loginfo("Aligned to the center of the gate")
        rospy.sleep(4)

        """center_waypoint = Waypoint()
        center_waypoint.header.seq = 0
        center_waypoint.header.stamp.nsecs = rospy.get_rostime().to_nsec()
        center_waypoint.header.stamp.secs = rospy.get_rostime().to_sec()
        center_waypoint.header.frame_id = "world"

        center_waypoint.point.x = 0
        center_waypoint.point.y = 0
        center_waypoint.point.z = 0

        center_waypoint.max_forward_speed = 0.5
        center_waypoint.heading_offset = 0
        center_waypoint.radius_of_acceptance = 0.5
        center_waypoint.use_fixed_heading = True

        go_to_resp = go_to(waypoint=center_waypoint, max_forward_speed=0.5, interpolator="cubic")"""

        # wait for the vehicle to complete this action

        # send a goto incremental request
        
        """step_point = Point()
        step_point.x = 5
        step_point.y = 0
        step_point.z = 0

        go_to_incremental_resp = go_to_incremental(step=step_point, max_forward_speed=0.5, interpolator="cubic")"""

        change_odom_response = change_odom(odom="dvl")

        pose.position.x = 4
        pose.position.y = current_p.y
        pose.position.z = -2.5
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
        change_odom_response = change_odom(odom="vision")

        pose.position.x = current_p.x
        pose.position.y = 0
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

        rospy.loginfo("Aligned to the center of the gate")
        rospy.sleep(3)

        # then send a goto incremental request

        """go_to_incremental_resp = go_to_incremental(step=step_point, max_forward_speed=0.5, interpolator="cubic")"""

        change_odom_response = change_odom(odom="dvl")

        pose.position.x = current_p.x + 10
        pose.position.y = current_p.y
        pose.position.z = -2.5
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1


        rospy.loginfo('publishing to the cmd pose')


        pose_cmd_pub.publish(pose)

        point = Point()

        point = pose.position
        has_reached(point, 0.2)

        rospy.loginfo("crossed the gate")
        rospy.sleep(3)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
