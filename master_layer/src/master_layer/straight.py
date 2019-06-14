#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

odom_msg = Odometry()

pose_cmd_pose = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)

def straight_callback (msg):

    time_duration = msg.data
    then = rospy.get_time()
    now = rospy.get_time()

    while (now - then < time_duration):
        now = rospy.get_time()
        cmd_pose = Pose()
        cmd_pose = odom_msg.pose.pose
        cmd_pose.orientation.x = 0
        cmd_pose.orientation.y = 0
        cmd_pose.orientation.z = 0
        cmd_pose.orientation.w = 1
        cmd_pose.position.z = cmd_pose.position.z
        cmd_pose.position.y = cmd_pose.position.y
        cmd_pose.position.x = cmd_pose.position.x - 2
        pose_cmd_pose.publish(cmd_pose)
	rospy.loginfo("Pose: " + str(cmd_pose))
        rospy.sleep(4)

def odometry_callback (msg):
    global odom_msg
    odom_msg = msg

if __name__ == '__main__':

    rospy.init_node('straight_node')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    straight_sub = rospy.Subscriber('/anahita/straight', Float32, straight_callback)

    rospy.loginfo('Straight node initialised')
    rospy.sleep(2)
    rospy.loginfo('after the sleep')
    rospy.spin()

