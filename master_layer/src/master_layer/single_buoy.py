#!/usr/bin/env python

import rospy
from anahita_utils import *
from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom
from darknet_ros.msg import BoundingBox, BoundingBoxes

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

status = False
_current_pose = Pose()
_target_pose = Pose()
_then = rospy.Time()
_timeout = 120
_step_timeout = 60
_depth_target = 0
num_objects = 0

def has_reached_with_yaw(self, target_pose, curr_pose, pos_threshold, yaw_threshold):
    while (not status or rospy.is_shutdown()):
        return False
    while (calc_dist(target_pose.position, curr_pose.position) > pos_threshold or rospy.is_shutdown()):
        return False
    _curr_roll, _curr_pitch, _curr_yaw = quaternion_to_eulerRPY(curr_pose.orientation)
    _target_roll, _target_pitch, _target_yaw = quaternion_to_eulerRPY(target_pose.orientation)
    while (abs(_curr_yaw - _target_yaw) > yaw_threshold):
        return False
    return True

def rotate_and_look(self):   
        _now = rospy.get_time()
        dt = _now - _then
        _target_pos = _current_pose._pos
        _start_roll, _start_pitch, _start_yaw = quaternion_to_eulerRPY(_current_pose.orientation)
        _target_roll = 0
        _target_pitch = 0
        _target_yaw = TO_DEGREE(_start_yaw)
        rospy.loginfo("The yaw before starting out is: " + str(_start_yaw))
        
        while dt < ._timeout:
            rospy.loginfo("Still trying to search my boi")
            _target_yaw += 30
            if(_target_yaw>180):
                _target_yaw -= 360
            rospy.loginfo('The target yaw is: ' + str(_target_yaw))
            target_quaternion = eulerRPY_to_quaternion(_target_roll, _target_pitch, TO_RADIAN(_target_yaw))    
            fill_pose_data_pq(_target_pose, _target_pos, target_quaternion)
            _pose_cmd_pub.publish(_target_pose)
            rospy.sleep(_step_timeout)

def bounding_boxes_cb(msg):
    bounding_box = msg.bounding_boxes.

def ml_callback(msg):
    rospy.loginfo("The number of objects being detected is: " + msg.data)
    num_objects = msg.data

def odometry_callback(msg):
    global current_pose
    _current_pose = msg.pose.pose
    global status = True

if __name__ == '__main__':

    rospy.init_node('single_buoy')
    rospy.loginfo('Going to slam the torpedo, since I dont know about the buoy')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    sub_object_detector = rospy.Subscriber('/anahita/found_object,')
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
    
    try:
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        set_odom_xy = rospy.ServiceProxy('/nav/set_world_x_y_offset', SetWorldXYOffset)
        set_odom_depth = rospy.ServiceProxy('/nav/set_depth_offset', SetDepthOffset)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    set_odom_xy_response = set_odom_xy()
    set_odom_depth_response = set_odom_depth()
    initial_pose = Pose()
    fill_pose_data(initial_pose, 0, 0 , 0, 0, 0, 0, 1)
    pose_cmd_pub.publish(initial_pose)
    rospy.loginfo('Setting the initial pose')

    rospy.sleep(3)

    while num_objects is 0 and rospy.get_time() - _then < timeout:
        rospy.loginfo('I cannot fucking see anything')
        rotate_and_look()
    
    if rospy.get_time() - _then > timeout:
        rospy.loginfo("Timed out")
        exit()

    change_odom_response = change_odom(odom="vision")
    
    fill_pose_data_xyzq(_target_pose, _depth_target, 0, 0, _current_pose.orientation)       
    pose_cmd_pub.publish(_target_pose)

    while _depth_target > 3:
        rospy.loginfo("Approaching the target")
        continue
    
    rospy.loginfo('')
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

    