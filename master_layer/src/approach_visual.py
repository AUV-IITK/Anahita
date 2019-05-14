#!/usr/bin/env python

import rospy
from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

status = False
current_p = Point()
stop_target = 0

def fill_pose_data(pose_object, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
    pose_object.position.x = pos_x
    pose_object.position.y = pos_y
    pose_object.position.z = pos_z
    pose_object.orientation.x = orient_x
    pose_object.orientation.y = orient_y
    pose_object.orientation.z = orient_z
    pose_object.orientation.w = orient_w
    
def calc_dist (pose1, pose2):
    x1 = pose1.x
    y1 = pose1.y
    z1 = pose1.z

    x2 = pose2.x
    y2 = pose2.y
    z2 = pose2.z

    dist = ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2))^0.5
    rospy.loginfo("The euclidian distance left is still: " + str(dist))
    return dist

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose.position
    global status
    status = True    

def has_reached (pos, threshold):
    
    while (not status or rospy.is_shutdown()):
        return False
    while (calc_dist(pos, current_p) > threshold or rospy.is_shutdown()):
        return False
    return True

def before_target_callback(msg):
    global stop_target
    stop_target = msg.data
    rospy.loginfo("Stop at a distance of: " + str(msg.data) + "from the visual target")


if __name__ == '__main__':
    rospy.loginfo("Starting the visual-approach task")
    
    rospy.init_node('approach_visual')
    
    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
    before_target_sub = rospy.Subscriber('/anahita/before_target', Int32, before_target_callback)

    while True:
        
        try:
            
            change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
            change_odom_response = change_odom(odom="vision")

            target_pose = Pose()
            fill_pose_data(target_pose, stop_target, 0, 0, 0, 0, 0, 1)
            pose_cmd_pub.publish(target_pose)
            rospy.loginfo('Publishing to the cmd pose')

            point = target_pose.position
            if has_reached(point, 3) is False:
                rospy.loginfo("Reached the final location of the vehicle")
                rospy.sleep(4)
                break

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    rospy.loginfo("Exiting the call")
    exit()
