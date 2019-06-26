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

def imu_cb (msg):
    global yaw
    orientation = msg.orientation
    yaw = quaternion_to_eulerRPY(orientation)[2]

if __name__=='__main__':
    rospy.init_node ('test_node')

    sub_odometry = rospy.Subscriber('/anahita/features', Int32MultiArray, callback)
    depth_sub = rospy.Subscriber('/anahita/distance', Point, depth_cb)
    imu_sub = rospy.Subscriber('/anahita/imu', Imu, imu_cb)

    gate_center = Point(0.83, 0, -1.2)

    f = open('depth_data4.csv', 'w+')
    g = open('yaw_data.csv', 'w+')

    while not rospy.is_shutdown():
        dist = get_dist (curr_point, gate_center)
        f.write(make_string(features, dist))
        # g.write(make_string(features, yaw))
        f.flush()
        # g.flush()
        # print make_string(features, yaw)
        print make_string(features, dist)
        rospy.sleep(1)
        