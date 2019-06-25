#!/usr/bin/env python

import rospy
from anahita_utils import *
from master_layer.srv import GoToIncremental, GoTo, ChangeOdom
from master_layer.msg import Waypoint
from odom_dvl_imu.srv import SetWorldXYOffset, SetDepthOffset
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Int8
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import time
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

from std_msgs.msg import Time, String

status = False
_current_pose = Pose()
_target_pose = Pose()
_then = 0
_timeout = 120
_step_timeout = 3
_depth_target = 0
num_objects = 0
detected_once = False

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

def has_reached_with_yaw(target_pose, curr_pose, pos_threshold, yaw_threshold):
    while (not status or rospy.is_shutdown()):
        return False
    while (calc_dist(target_pose.position, curr_pose.position) > pos_threshold or rospy.is_shutdown()):
        return False
    _curr_roll, _curr_pitch, _curr_yaw = quaternion_to_eulerRPY(curr_pose.orientation)
    _target_roll, _target_pitch, _target_yaw = quaternion_to_eulerRPY(target_pose.orientation)
    rospy.loginfo('The current yaw is: ' + str(TO_DEGREE(_curr_yaw))  + ", target yaw is: " + str(TO_DEGREE(_target_yaw)))
    while (abs(TO_DEGREE(_curr_yaw) - TO_DEGREE(_target_yaw)) > yaw_threshold):
        return False
    print("Pahunchgye sir"  + str(abs(_curr_yaw - _target_yaw)))
    return True

def rotate_and_look():   
        global _current_pose
        _now = rospy.get_time()
        dt = _now - _then
        _target_pos = _current_pose.position
        _start_roll, _start_pitch, _start_yaw = quaternion_to_eulerRPY(_current_pose.orientation)
        _target_roll = 0
        _target_pitch = 0
        _target_yaw = TO_DEGREE(_start_yaw)
        rospy.loginfo("The yaw before starting out is: " + str(_start_yaw) + " , dt: " + str(dt))
        while dt < _timeout and detected_once is False:
            rospy.loginfo("Still trying to search my boi")
            _target_yaw += 30
            if(_target_yaw>180):
                _target_yaw -= 360
            rospy.loginfo('The target yaw is: ' + str(_target_yaw))
            target_quaternion = eulerRPY_to_quaternion(_target_roll, _target_pitch, TO_RADIAN(_target_yaw))    
            fill_pose_data_pq(_target_pose, _target_pos, target_quaternion)
            pose_cmd_pub.publish(_target_pose)
	    while has_reached_with_yaw(_target_pose, _current_pose, 2, 5) is False and detected_once is False:
		continue

def ml_callback(msg):
    rospy.loginfo("The number of objects being detected is: " + str(msg.data))  
    global num_objects
    num_objects = msg.data
    if num_objects > 0:
	    global detected_once
	    detected_once = True

def odometry_callback(msg):
    global _current_pose
    _current_pose = msg.pose.pose
    global status 
    status = True

if __name__ == '__main__':

    rospy.init_node('single_buoy')
    rospy.loginfo('Going to slam the torpedo, since I dont know about the buoy')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    sub_object_detector = rospy.Subscriber('/anahita/found_object', Int8, ml_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
    _then = rospy.get_time()
    try:
        change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
        set_odom_xy = rospy.ServiceProxy('/nav/set_world_x_y_offset', SetWorldXYOffset)
        set_odom_depth = rospy.ServiceProxy('/nav/set_depth_offset', SetDepthOffset)
        go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
        pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    set_odom_xy_response = set_odom_xy()
    set_odom_depth_response = set_odom_depth()
    initial_pose = Pose()
    fill_pose_data(initial_pose, 0, 0 , 0, 0, 0, 0, 1)
    go_to_pose(target_pose=initial_pose)
    rospy.loginfo('Setting the initial pose')

    rospy.sleep(100)
    # _then = rospy.get_time()
	
    # while num_objects == 0 and detected_once is False:# and rospy.get_time() - _then < _timeout:
    #     rospy.loginfo('I cannot fucking see anything')
    #     rotate_and_look()
    
    # if rospy.get_time() - _then > _timeout:
    #     rospy.loginfo("Timed out")
    #     exit()

    # change_odom_response = change_odom(odom="vision_ml")
    
    # fill_pose_data_xyzq(_target_pose, _depth_target, 0, 0, _current_pose.orientation)
    # pose_cmd_pub.publish(_target_pose)

    # while _depth_target > 3:
    #     rospy.loginfo("Approaching the target")
    #     continue

    # fill_pose_data_xyzq(_target_pose, 3, 0, 0, _current_pose.orientation)       
    # pose_cmd_pub.publish(_target_pose)
    # rospy.loginfo('At a stable location')

    # rospy.sleep(3)

    # # buoy hitting

    # pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('Publishing cmd to align')

    # pose_reach(time_out=20)
    # rospy.sleep(5)
    # rospy.loginfo('Aligned center')

    

    # pose = fill_pose(-2.1, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to move near the torpedo hole')

    # pose_reach(time_out=20)
    # rospy.sleep(1)
    # rospy.loginfo('Infront of the torpedo hole')

    # change_odom(odom="dvl")
    # rospy.sleep(0.1)
    # pose = fill_pose(2.3 + _current_p.x, _current_p.y, _current_p.z, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to hit the buoy')

    # pose_reach(time_out=20)
    # rospy.loginfo('Buoy is hit!')
    # rospy.sleep(1)

    # pose = fill_pose(_current_p.x - 2, _current_p.y, _current_p.z, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to go back')

    # pose_reach(time_out=20)
    # rospy.loginfo('Bot is back')



    
