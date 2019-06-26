#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Time, String
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Pose

import tf.transformations as trans

current_p = Pose()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose

if __name__ == '__main__':

    rospy.init_node('record_path')
    manual_waypoints_list =[]
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    then = rospy.get_time()
    save_timeout = 2
    rospy.loginfo("Started path recorder node")
    try:
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if(now - then > save_timeout):
                manual_waypoints_list.append(current_p)
                rospy.loginfo("Current waypoints list: " + str(manual_waypoints_list))
                then = now
            rospy.sleep(2)
        else:
            rospy.loginfo("Manual waypoints: " + str(manual_waypoints_list))
            rospy.loginfo("Successfully ended recording")

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
