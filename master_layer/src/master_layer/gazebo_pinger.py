#!/usr/bin/env python

import rospy
import math
import numpy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose, Point
import tf.transformations as trans
from std_msgs.msg import Float32

curr_pose = Pose()

def odom_callback (msg):
    global curr_pose
    curr_pose = msg.pose.pose

def dist (a, b):
    dist = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)
    return dist


def calculate_heading (pinger_point):
    base = pinger_point.x - curr_pose.position.x
    hypotenuse = dist (pinger_point, curr_pose.position)
    perpendicular = pinger_point.y - curr_pose.position.x

    pinger_heading = (abs(perpendicular)/perpendicular)*math.acos(base/hypotenuse)
    bot_orientation = trans.euler_from_quaternion(curr_pose.orientation)
    bot_heading = bot_orientation.z

    return pinger_heading - bot_heading

if __name__ == '__main__':
    rospy.init_node('acoustic_localisation')
    odom_sub = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odom_callback)
    pinger_heading_pub rospy.Publisher('/anahita/pinger_heading', Float32, queue_size=10, latch=True)

    pinger_point = Point()
    pinger_point.x = 21.096525
    pinger_point.y = 8.886061
    pinger_point.z = -2.450000

    pinger_heading_msg = Float32()

    while not rospy.is_shutdown():
        pinger_heading_msg.data = calculate_heading(pinger_point)
        pinger_heading_pub.publish(pinger_heading_msg)
        rospy.sleep(0.05)
