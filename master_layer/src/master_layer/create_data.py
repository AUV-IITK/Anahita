#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import Imu
import math
from anahita_utils import *

curr_point = Point()
features = Int32MultiArray()
yaw = 0

def callback(msg):
    global features
    features = msg

def depth_cb (msg):
    global curr_point
    curr_point = msg

def get_dist (a, b):
    return math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y))

def make_string (features, dist):
    str_ = ''
    for i in range(len(features.data)):
        str_ += str(features.data[i]) + ' '
    str_ = str_ + str(dist) + '\n'
    return str_

def make_string_ (features, dist, yaw):
    str_ = ''
    for i in range(len(features.data)):
        str_ += str(features.data[i]) + ' '
    str_ = str_ + ' ' + str(yaw) + str(dist) + '\n'
    return str_

def imu_cb (msg):
    global yaw
    orientation = msg.orientation
    yaw = quaternion_to_eulerRPY(orientation)[2]*57.2957795

if __name__=='__main__':
    rospy.init_node ('test_node')

    sub_odometry = rospy.Subscriber('/anahita/features', Int32MultiArray, callback)
    depth_sub = rospy.Subscriber('/anahita/distance', Point, depth_cb)
    imu_sub = rospy.Subscriber('/anahita/imu', Imu, imu_cb)

    marker_center = Point(0, 0, -10)

    f = open('../../data/torpedo_yaw_data.csv', 'w+')

    while not rospy.is_shutdown():
        # depth = get_dist (marker_center, curr_point)
        f.write(make_string(features, yaw))
        f.flush()
        print (make_string(features, yaw))
        rospy.sleep(0.5)
        